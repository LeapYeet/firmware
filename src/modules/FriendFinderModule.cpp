#include "FriendFinderModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "GPS.h"
#include "Power.h"
#include "TypeConversions.h"
#include "MagnetometerModule.h"
#include "graphics/Screen.h"
#include "graphics/draw/CompassRenderer.h"
#include "graphics/draw/NotificationRenderer.h"
#include "graphics/TimeFormatters.h"
#include "main.h"
#include "input/InputBroker.h"
#include <cmath>

#include <algorithm>
#include <cstring>

// nanopb
#include "pb_encode.h"
#include "pb_decode.h"

#define FRIEND_FINDER_PORTNUM meshtastic_PortNum_FRIEND_FINDER_APP
#define UPDATE_INTERVAL       15    // seconds
#define BACKGROUND_UPDATE_INTERVAL 120   // seconds (2 minutes)
#define HIGH_GPS_INTERVAL     2     // seconds
#define DEFAULT_GPS_INTERVAL  300   // 5 minutes

/* Optional persistence via ESP32 NVS */
#if defined(ARDUINO_ARCH_ESP32)
  #include <Preferences.h>
  static Preferences g_prefs;
  #define FF_HAVE_NVS 1
#else
  #define FF_HAVE_NVS 0
#endif

/* Helper for logs */
static const char* ffTypeName(meshtastic_FriendFinder_RequestType t) {
    switch (t) {
        case meshtastic_FriendFinder_RequestType_NONE:         return "NONE";
        case meshtastic_FriendFinder_RequestType_REQUEST:      return "REQUEST";
        case meshtastic_FriendFinder_RequestType_ACCEPT:       return "ACCEPT";
        case meshtastic_FriendFinder_RequestType_REJECT:       return "REJECT";
        case meshtastic_FriendFinder_RequestType_END_SESSION:  return "END_SESSION";
        default:                                               return "?";
    }
}

static void hexdump(const char *tag, const uint8_t *b, size_t n)
{
    constexpr size_t W = 16;
    char line[3*W + 1];
    for (size_t i = 0; i < n; i += W) {
        size_t m = (n - i > W) ? W : (n - i);
        char *p = line;
        for (size_t j = 0; j < m; ++j) p += snprintf(p, 4, "%02X ", b[i + j]);
        *p = 0;
        LOG_DEBUG("[FriendFinder] %s: %s", tag, line);
    }
}

FriendFinderModule *friendFinderModule = nullptr;
FriendFinderModule *FriendFinderModule::instance = nullptr;

FriendFinderModule::FriendFinderModule()
    : ProtobufModule("friendfinder", FRIEND_FINDER_PORTNUM, meshtastic_FriendFinder_fields),
      OSThread("FriendFinder")
{
    instance = this;
    friendFinderModule = this;

    if (inputBroker) {
        inputObserver.observe(inputBroker);
        LOG_INFO("[FriendFinder] Input observer attached");
    } else {
        LOG_WARN("[FriendFinder] inputBroker is null – no button events");
    }

    loadFriends();
}

void FriendFinderModule::setup() {
    LOG_INFO("[FriendFinder] setup()");
    LOG_INFO("[FriendFinder] build=%s %s", __DATE__, __TIME__);
    LOG_INFO("[FriendFinder] FRIEND_FINDER_PORTNUM=%u", (unsigned)FRIEND_FINDER_PORTNUM);

    // Failsafe: If we boot up and find GPS in high-power mode, assume we crashed and restore it.
    if (config.position.gps_update_interval > 0 && config.position.gps_update_interval <= HIGH_GPS_INTERVAL) {
        LOG_WARN("[FriendFinder] GPS interval is low (%d sec), restoring default. Was device rebooted during a session?", config.position.gps_update_interval);
        config.position.gps_update_interval = DEFAULT_GPS_INTERVAL;
        service->reloadConfig(SEGMENT_CONFIG);
    }
}

/* ---------- Friend store (persist up to MAX_FRIENDS) ---------- */

int FriendFinderModule::getUsedFriendsCount() const {
    int cnt = 0;
    for (const auto &f : friends_) if (f.used) ++cnt;
    return cnt;
}

int FriendFinderModule::getFriendSlotByListIndex(int listIdx) const {
    // Note: listIdx==0 is "Back", so friends start at listIdx==1
    if (listIdx <= 0) return -1;
    int i = 1; 
    for (int slot = 0; slot < MAX_FRIENDS; ++slot) {
        if (friends_[slot].used) {
            if (i == listIdx) return slot;
            ++i;
        }
    }
    return -1; // should not happen if listIdx < count+1
}

void FriendFinderModule::removeFriendAt(int listIdx) {
    int slot = getFriendSlotByListIndex(listIdx);
    if (slot < 0) return;
    friends_[slot].used = false;
    saveFriends();
    LOG_INFO("[FriendFinder] Removed friend at slot %d", slot);
}

void FriendFinderModule::loadFriends() {
    for (auto &f : friends_) {
        f = {}; // Zero-initialize all members
        f.last_data = meshtastic_FriendFinder_init_default;
    }

#if FF_HAVE_NVS
    if (!g_prefs.begin("ffinder", /*ro*/false)) {
        LOG_WARN("[FriendFinder] NVS open failed; friends in RAM only");
        return;
    }
    size_t sz = g_prefs.getBytesLength("friends");
    if (sz == sizeof(friends_)) {
        g_prefs.getBytes("friends", friends_, sizeof(friends_));
        LOG_INFO("[FriendFinder] Loaded %u bytes of friends", (unsigned)sz);
    } else if (sz != 0) {
        LOG_WARN("[FriendFinder] Unexpected friends blob size=%u (expected %u), resetting", (unsigned)sz, sizeof(friends_));
    }
    g_prefs.end();
#endif
}

void FriendFinderModule::saveFriends() {
#if FF_HAVE_NVS
    if (!g_prefs.begin("ffinder", false)) return;
    g_prefs.putBytes("friends", friends_, sizeof(friends_));
    g_prefs.end();
#endif
}

int FriendFinderModule::findFriend(uint32_t node) const {
    for (int i = 0; i < MAX_FRIENDS; ++i)
        if (friends_[i].used && friends_[i].node == node) return i;
    return -1;
}

void FriendFinderModule::upsertFriend(uint32_t node, uint32_t session_id, const uint8_t secret[16]) {
    int idx = findFriend(node);
    if (idx < 0) {
        for (int i = 0; i < MAX_FRIENDS; ++i) { if (!friends_[i].used) { idx = i; break; } }
        if (idx < 0) idx = 0; // overwrite first if full
    }
    friends_[idx].node = node;
    friends_[idx].session_id = session_id;
    if (secret) memcpy(friends_[idx].secret, secret, 16); else memset(friends_[idx].secret, 0, 16);
    friends_[idx].used = true;
    saveFriends();
    LOG_INFO("[FriendFinder] Saved friend 0x%08x at slot %d", (unsigned)node, idx);
}

/* ---------- GPS Mode Control ---------- */
void FriendFinderModule::activateHighGpsMode() {
    if (!isGpsHighPower && config.position.gps_update_interval != HIGH_GPS_INTERVAL) {
        LOG_INFO("[FriendFinder] Activating high-power GPS mode.");
        originalGpsUpdateInterval = config.position.gps_update_interval;
        config.position.gps_update_interval = HIGH_GPS_INTERVAL;
        service->reloadConfig(SEGMENT_CONFIG);
        isGpsHighPower = true;
    }
}

void FriendFinderModule::restoreNormalGpsMode() {
    if (isGpsHighPower) {
        LOG_INFO("[FriendFinder] Restoring normal GPS mode.");
        config.position.gps_update_interval = originalGpsUpdateInterval;
        service->reloadConfig(SEGMENT_CONFIG);
        isGpsHighPower = false;
    }
}

/* ---------- UI entry points ---------- */
void FriendFinderModule::launchMenu()
{
#if HAS_SCREEN
    menuIndex = 0;
    currentState = FriendFinderState::MENU_SELECTION; // lightweight in-frame list (no overlay)
    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, /*focus=*/true);
#endif
}

void FriendFinderModule::beginPairing()
{
    pairingWindowOpen = true;
    pairingWindowExpiresAt = millis() + PAIRING_WINDOW_MS;
    currentState = FriendFinderState::AWAITING_RESPONSE;

#if HAS_SCREEN
    screen->showSimpleBanner("Pairing… Press on BOTH devices", 1500);
#endif

    sendFriendFinderPacket(NODENUM_BROADCAST, meshtastic_FriendFinder_RequestType_REQUEST, 1);
    lastSentPacketTime = millis();
}

void FriendFinderModule::requestMutualTracking(uint32_t nodeNum)
{
    if (nodeNum == 0 || nodeNum == nodeDB->getNodeNum()) return;

    targetNodeNum = nodeNum;
    currentState  = FriendFinderState::AWAITING_RESPONSE;
    pairingWindowOpen = true;
    pairingWindowExpiresAt = millis() + PAIRING_WINDOW_MS;

#if HAS_SCREEN
    screen->showSimpleBanner("Requesting session...", 1500);
#endif
    // Send a directed request to the specific friend
    sendFriendFinderPacket(nodeNum, meshtastic_FriendFinder_RequestType_REQUEST, 0);
    lastSentPacketTime = millis();
}

void FriendFinderModule::startTracking(uint32_t nodeNum)
{
    if (nodeNum == 0 || nodeNum == nodeDB->getNodeNum()) return;

    targetNodeNum = nodeNum;

    const int friend_idx = findFriend(nodeNum);

    // This function is now only for direct, one-way tracking starts (e.g. from an ACCEPT)
    if (friend_idx >= 0) {
        LOG_INFO("[FriendFinder] startTracking(): already friends with 0x%08x -> start immediately", nodeNum);
        currentState = FriendFinderState::TRACKING_TARGET;
        previousDistance = -1.0f; // Reset distance trend

        // Pre-load last known data for a faster UI start
        lastFriendData = friends_[friend_idx].last_data;
        lastFriendPacketTime = friends_[friend_idx].last_heard_time;

        activateHighGpsMode(); // Activate real-time GPS
        lastSentPacketTime = 0; // force quick first beacon
#if HAS_SCREEN
        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
        sendFriendFinderPacket(nodeNum, meshtastic_FriendFinder_RequestType_NONE, 0);
        return;
    }
}

void FriendFinderModule::endSession(bool notifyPeer)
{
    if (notifyPeer && targetNodeNum) {
        sendFriendFinderPacket(targetNodeNum, meshtastic_FriendFinder_RequestType_END_SESSION);
    }
    targetNodeNum = 0;
    pairingWindowOpen = false;
    currentState = FriendFinderState::IDLE;
    previousDistance = -1.0f; // Reset distance trend
    restoreNormalGpsMode(); // Restore original GPS settings
    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
}

/* ---------- Inputs ---------- */
int FriendFinderModule::handleInputEvent(const InputEvent *ev)
{
    if (!ev) return 0;

    // Standard single-button mapping
    const bool isNavDown = ev->inputEvent == INPUT_BROKER_DOWN || ev->inputEvent == INPUT_BROKER_USER_PRESS;
    const bool isSelect  = ev->inputEvent == INPUT_BROKER_SELECT || ev->inputEvent == INPUT_BROKER_ALT_LONG;
    const bool isBack    = ev->inputEvent == INPUT_BROKER_BACK || ev->inputEvent == INPUT_BROKER_CANCEL;
    const bool isNavUp   = ev->inputEvent == INPUT_BROKER_UP;

    /* ---------- Friend Map (Single Button Logic) ---------- */
    if (currentState == FriendFinderState::FRIEND_MAP) {
        if (friendMapMenuVisible) {
            // --- Menu is OPEN ---
            if (isNavDown) { // Short press: navigate menu
                friendMapMenuIndex = (friendMapMenuIndex + 1) % NUM_MAP_MENU;
                screen->forceDisplay();
                return 1;
            }
            if (isSelect) { // Long press: select menu item
                switch(friendMapMenuIndex) {
                    case 0: // Toggle Names
                        friendMapNamesVisible = !friendMapNamesVisible;
                        break;
                    case 1: // Back to Map
                        // No action needed, just close menu
                        break;
                    case 2: // Exit
                        currentState = FriendFinderState::MENU_SELECTION;
                        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
                        break;
                }
                friendMapMenuVisible = false; // Close menu after action
                screen->forceDisplay();
                return 1;
            }
        } else {
            // --- Menu is CLOSED ---
            if (isSelect) { // Long press: open the menu
                friendMapMenuVisible = true;
                friendMapMenuIndex = 0;
                screen->forceDisplay();
                return 1;
            }
        }

        // Double press (back) exits the map screen regardless of menu state
        if (isBack) {
            currentState = FriendFinderState::MENU_SELECTION;
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
            return 1;
        }

        return 0; // Consume all other inputs
    }

    /* ---------- Calibration Submenu ---------- */
    if (currentState == FriendFinderState::CALIBRATION_MENU) {
        if (isNavUp)   { calibrationMenuIndex = (calibrationMenuIndex + NUM_CAL_MENU - 1) % NUM_CAL_MENU; screen->forceDisplay(); return 1; }
        if (isNavDown) { calibrationMenuIndex = (calibrationMenuIndex + 1) % NUM_CAL_MENU; screen->forceDisplay(); return 1; }
        if (isBack) { currentState = FriendFinderState::MENU_SELECTION; screen->forceDisplay(); return 1; }
        if (isSelect) {
            switch (calibrationMenuIndex) {
            case 0: // Back
                currentState = FriendFinderState::MENU_SELECTION;
                screen->forceDisplay();
                break;
            case 1: // Calibrate Compass (figure-8)
                if (magnetometerModule) {
                    magnetometerModule->startFigure8Calibration(15000);
#if HAS_SCREEN
                    screen->showSimpleBanner("Compass Cal: move in a FIGURE-8 for 15s", 1800);
#endif
                    LOG_INFO("[FriendFinder] Requested FIGURE-8 calibration (15s).");
                } else {
#if HAS_SCREEN
                    screen->showSimpleBanner("No magnetometer", 1200);
#endif
                }
                break;
            case 2: // Flat-Spin Cal (table)
                if (magnetometerModule) {
                    magnetometerModule->startFlatSpinCalibration(12000);
#if HAS_SCREEN
                    screen->showSimpleBanner("Spin slowly on table CLOCKWISE FOR 12s", 1600);
#endif
                    LOG_INFO("[FriendFinder] Requested FLAT-SPIN calibration (12s).");
                } else {
#if HAS_SCREEN
                    screen->showSimpleBanner("No magnetometer", 1200);
#endif
                }
                break;
            case 3: // Set North Here (user zero)
                if (magnetometerModule && magnetometerModule->hasHeading()) {
                    magnetometerModule->setNorthHere();
#if HAS_SCREEN
                    screen->showSimpleBanner("North set to current heading", 1200);
#endif
                } else {
#if HAS_SCREEN
                    screen->showSimpleBanner("Heading not ready", 800);
#endif
                }
                break;
            case 4: // Clear North Offset
                if (magnetometerModule) {
                    magnetometerModule->clearNorthOffset();
#if HAS_SCREEN
                    screen->showSimpleBanner("North offset cleared", 1000);
#endif
                }
                break;
            case 5: // Dump Compass Cal to log
                if (magnetometerModule) {
                    magnetometerModule->dumpCalToLog();
#if HAS_SCREEN
                    screen->showSimpleBanner("Cal dumped to log", 1000);
#endif
                }
                break;
            }
            return 1;
        }
        return 0;
    }

    /* ---------- friend-list action menu ("Track / Remove / Back") ---------- */
    if (currentState == FriendFinderState::FRIEND_LIST_ACTION) {
        if (isNavUp)   { overlayIndex = (overlayIndex + NUM_FRIEND_ACTIONS - 1) % NUM_FRIEND_ACTIONS; screen->forceDisplay(); return 1; }
        if (isNavDown) { overlayIndex = (overlayIndex + 1) % NUM_FRIEND_ACTIONS; screen->forceDisplay(); return 1; }
        if (isBack) { currentState = FriendFinderState::FRIEND_LIST; screen->forceDisplay(); return 1; }
        if (isSelect) {
            // friendListIndex is the raw index from the friend list (1=first friend)
            const int slot = getFriendSlotByListIndex(friendListIndex);
            
            switch (overlayIndex) {
            case 0: // Track
                if (slot >= 0) requestMutualTracking(friends_[slot].node);
                break;
            case 1: // Remove
                if (slot >= 0) {
                    removeFriendAt(friendListIndex);
                    int cnt = getUsedFriendsCount();
                    // Go back to friend list, or menu if list is now empty
                    if (cnt == 0) {
                        currentState = FriendFinderState::MENU_SELECTION;
                        screen->showSimpleBanner("No friends saved", 1200);
                        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
                    } else {
                        friendListIndex = std::min(friendListIndex, cnt); // cnt is now count+1 for "Back" item
                        currentState = FriendFinderState::FRIEND_LIST;
                        screen->forceDisplay();
                    }
                }
                break;
            case 2: // Back
                currentState = FriendFinderState::FRIEND_LIST;
                screen->forceDisplay();
                break;
            }
            return 1;
        }
        return 0;
    }

    /* ---------- friend-list Browse ---------- */
    if (currentState == FriendFinderState::FRIEND_LIST) {
        // Total items = number of friends + 1 for "Back"
        int cnt = getUsedFriendsCount() + 1;
        
        if (isNavUp)   { friendListIndex = (friendListIndex + cnt - 1) % cnt; screen->forceDisplay(); return 1; }
        if (isNavDown) { friendListIndex = (friendListIndex + 1) % cnt; screen->forceDisplay(); return 1; }
        if (isBack) { currentState = FriendFinderState::MENU_SELECTION; raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false); return 1; }
        if (isSelect)  {
            if (friendListIndex == 0) { // "Back" selected
                currentState = FriendFinderState::MENU_SELECTION;
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
            } else { // A friend was selected
                overlayIndex = 0; // Default to "Track"
                currentState = FriendFinderState::FRIEND_LIST_ACTION;
                screen->forceDisplay();
            }
            return 1;
        }
        return 0;
    }

    // In-session menu
    if (currentState == FriendFinderState::TRACKING_MENU) {
        if (isNavUp)   { overlayIndex = (overlayIndex + NUM_OVERLAY - 1) % NUM_OVERLAY; screen->forceDisplay(); return 1; }
        if (isNavDown) { overlayIndex = (overlayIndex + 1) % NUM_OVERLAY; screen->forceDisplay(); return 1; }
        if (isBack) {
            currentState = previousState; // Go back to tracking view
            screen->forceDisplay();
            return 1;
        }
        if (isSelect) {
            switch (overlayIndex) {
            case 0: // Stop Tracking
                endSession(/*notifyPeer*/true);
                break;
            case 1: // Back
                currentState = previousState;
                screen->forceDisplay();
                break;
            }
            return 1;
        }
        return 0;
    }

    // In session: SELECT opens menu
    if (currentState == FriendFinderState::TRACKING_TARGET ||
        currentState == FriendFinderState::BEING_TRACKED)
    {
        if (isSelect) {
            previousState = currentState;
            currentState = FriendFinderState::TRACKING_MENU;
            overlayIndex = 0; // Re-use overlayIndex for menu index
            screen->forceDisplay();
            return 1;
        }
        if (isBack) { // Allow ending session with BACK button from main tracking screen
            endSession(/*notifyPeer*/true);
            return 1;
        }
        return 0;
    }

    // In-frame main menu (fast)
    if (currentState == FriendFinderState::MENU_SELECTION) {
        if (isNavUp)   { menuIndex = (menuIndex + NUM_MENU - 1) % NUM_MENU; screen->forceDisplay(); return 1; }
        if (isNavDown) { menuIndex = (menuIndex + 1) % NUM_MENU; screen->forceDisplay(); return 1; }
        if (isBack) {
            currentState = FriendFinderState::IDLE;
            pairingWindowOpen = false;
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
            return 1;
        }
        if (isSelect) {
            switch (menuIndex) {
            case 0: // Back/Exit
                currentState = FriendFinderState::IDLE;
                pairingWindowOpen = false;
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
                return 1;
            case 1: // Start Pairing
                beginPairing();
                return 1;
            case 2: // Track a Friend
#if HAS_SCREEN
                if (getUsedFriendsCount() > 0) {
                    friendListIndex = 0; // Default to "Back"
                    currentState = FriendFinderState::FRIEND_LIST;
                    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
                } else {
                    screen->showSimpleBanner("No friends saved", 1200);
                }
#endif
                return 1;
            case 3: // Friend Map
#if HAS_SCREEN
                currentState = FriendFinderState::FRIEND_MAP;
                // Reset map UI state on entry
                friendMapMenuVisible = false;
                friendMapNamesVisible = true;
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
                return 1;
            case 4: // Compass Calibration
                calibrationMenuIndex = 0; // Default to "Back"
                currentState = FriendFinderState::CALIBRATION_MENU;
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
                return 1;
            }
        }
        return 0;
    }

    return 0;
}

/* ---------- Thread ---------- */
int32_t FriendFinderModule::runOnce()
{
    const uint32_t now = millis();

    // When map is open, force redraws to keep it live
    if (currentState == FriendFinderState::FRIEND_MAP) {
#if HAS_SCREEN
        raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
#endif
    }

    // Send periodic background updates to all friends when idle
    if (currentState == FriendFinderState::IDLE && getUsedFriendsCount() > 0 && gpsStatus->getHasLock()) {
        if (!lastBackgroundUpdateTime || (now - lastBackgroundUpdateTime) > (BACKGROUND_UPDATE_INTERVAL * 1000UL)) {
            LOG_INFO("[FriendFinder] Sending background location updates to %d friends.", getUsedFriendsCount());
            for (int i = 0; i < MAX_FRIENDS; ++i) {
                if (friends_[i].used) {
                    sendFriendFinderPacket(friends_[i].node, meshtastic_FriendFinder_RequestType_NONE);
                }
            }
            lastBackgroundUpdateTime = now;
        }
    }

    if (pairingWindowOpen && (int32_t)(now - pairingWindowExpiresAt) >= 0) {
        pairingWindowOpen = false;
        if (currentState == FriendFinderState::AWAITING_RESPONSE) {
            currentState = FriendFinderState::MENU_SELECTION;
#if HAS_SCREEN
            screen->showSimpleBanner("Request timed out", 1200);
#endif
        }
    }

    // Show small progress to user during calibration (non-blocking)
#if HAS_SCREEN
    if (magnetometerModule) {
        const bool calNow = magnetometerModule->isCalibrating();
        if (calNow && !calWasActive) {
            calWasActive = true;
            screen->showSimpleBanner("Calibrating… move in FIGURE-8", 1200);
        }
        if (!calNow && calWasActive) {
            calWasActive = false;
            screen->showSimpleBanner("Compass calibration done", 1200);
        }

        const bool flatNow = magnetometerModule->isFlatCalibrating();
        if (flatNow && !flatCalWasActive) {
            flatCalWasActive = true;
            screen->showSimpleBanner("Calibrating… spin flat on table", 1200);
        }
        if (!flatNow && flatCalWasActive) {
            flatCalWasActive = false;
            screen->showSimpleBanner("Flat-spin calibration done", 1200);
        }
    }
#endif

    switch (currentState) {
    case FriendFinderState::BEING_TRACKED:
        if ((now - lastSentPacketTime) > UPDATE_INTERVAL * 1000UL && targetNodeNum) {
            sendFriendFinderPacket(targetNodeNum, meshtastic_FriendFinder_RequestType_NONE);
        }
        [[fallthrough]];
    case FriendFinderState::TRACKING_TARGET:
#if HAS_SCREEN
        if (shouldDraw()) raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
#endif
        // Tracker also beacons its own stats every UPDATE_INTERVAL seconds
        if ((now - lastSentPacketTime) > UPDATE_INTERVAL * 1000UL && targetNodeNum) {
            sendFriendFinderPacket(targetNodeNum, meshtastic_FriendFinder_RequestType_NONE);
        }
        break;
    default: break;
    }

    // Snappy UI
    return 50; // ~20 FPS
}

/* ---------- RX ---------- */
bool FriendFinderModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                                meshtastic_FriendFinder *ff)
{
    const uint32_t from = getFrom(&mp);
    if (from == nodeDB->getNodeNum()) return true;

    const bool hasFF = mp.decoded.has_friend_finder;
    if (!hasFF) return false;

    LOG_INFO("[FriendFinder] RX pkt id=0x%08x from=0x%08x port=%u hop=%u/%u chan=%u",
             (unsigned)mp.id, (unsigned)from, (unsigned)mp.decoded.portnum,
             (unsigned)mp.hop_limit, (unsigned)mp.hop_start, (unsigned)mp.channel);

    if (mp.decoded.payload.size) hexdump("RX raw", mp.decoded.payload.bytes, mp.decoded.payload.size);

    LOG_INFO("[FriendFinder] RX FF type=%s(%d) batt=%u sats=%u lat=%d lon=%d time=%u state=%d pairingOpen=%d",
             ffTypeName(ff->request_type), (int)ff->request_type,
             (unsigned)ff->battery_level, (unsigned)ff->sats_in_view,
             (int)ff->latitude_i, (int)ff->longitude_i, (unsigned)ff->time,
             (int)currentState, (int)pairingWindowOpen);

    switch (ff->request_type) {
    case meshtastic_FriendFinder_RequestType_REQUEST: {
        // A directed request from a friend, or a broadcast request while we are in the pairing window
        bool isDirectedRequest = (findFriend(from) >= 0);
        bool isInPairingWindow = pairingWindowOpen;

        if (isDirectedRequest || isInPairingWindow) {
            // Become tracked by the requester
            targetNodeNum = from;
            currentState  = FriendFinderState::BEING_TRACKED;
            previousDistance = -1.0f; // Reset distance trend
            activateHighGpsMode(); // Activate real-time GPS
            pairingWindowOpen = false;

            // If it's a new friend from a broadcast pairing, create the record now
            if (!isDirectedRequest && isInPairingWindow) {
                uint32_t sess = (uint32_t)random(1, 0x7fffffff);
                uint8_t  sec[16]; for (int i = 0; i < 16; ++i) sec[i] = random(0, 255);
                upsertFriend(from, sess, sec);
                LOG_INFO("[FriendFinder] New friend from pairing window -> ACCEPT");
            } else {
                 LOG_INFO("[FriendFinder] REQUEST from existing friend -> ACCEPT");
            }
            
            sendFriendFinderPacket(from, meshtastic_FriendFinder_RequestType_ACCEPT);
#if HAS_SCREEN
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
            return true;
        }

        // It's a broadcast request and we aren't in a pairing window.
#if HAS_SCREEN
        screen->showSimpleBanner("Hold Pair on both devices", 1200);
#endif
        return true;
    }

    case meshtastic_FriendFinder_RequestType_ACCEPT: {
        // We sent a request and the other person accepted.
        if (currentState == FriendFinderState::AWAITING_RESPONSE && from == targetNodeNum) {
            currentState  = FriendFinderState::TRACKING_TARGET;
            previousDistance = -1.0f; // Reset distance trend
            activateHighGpsMode(); // Activate real-time GPS
            pairingWindowOpen = false;
            lastFriendPacketTime = millis();
            lastFriendData = *ff;

            // If we weren't already friends, save them now.
            if (findFriend(from) < 0) {
                uint32_t sess = (uint32_t)random(1, 0x7fffffff);
                uint8_t  sec[16]; for (int i = 0; i < 16; ++i) sec[i] = random(0, 255);
                upsertFriend(from, sess, sec);
            }

#if HAS_SCREEN
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
        }
        break;
    }

    case meshtastic_FriendFinder_RequestType_END_SESSION: {
        if (from == targetNodeNum &&
            (currentState == FriendFinderState::TRACKING_TARGET ||
             currentState == FriendFinderState::BEING_TRACKED)) {
#if HAS_SCREEN
            screen->showSimpleBanner("Session ended by peer", 1200);
#endif
            endSession(/*notifyPeer*/false);
        }
        break;
    }

    case meshtastic_FriendFinder_RequestType_NONE: {
        // If this is a known friend, store their latest data for quick-start tracking.
        const int friend_idx = findFriend(from);
        if (friend_idx >= 0) {
            friends_[friend_idx].last_data = *ff;
            friends_[friend_idx].last_heard_time = millis();
            LOG_DEBUG("[FriendFinder] Stored background update from friend 0x%08x", from);
        }

        // If this update is for our current tracking target, also update the live session variables.
        if (from == targetNodeNum) {
            lastFriendData = *ff;
            lastFriendPacketTime = millis();
            LOG_DEBUG("[FriendFinder] Update from 0x%08x: batt=%u sats=%u",
                      from, (unsigned)ff->battery_level, (unsigned)ff->sats_in_view);
        }
        break;
    }

    default:
        break;
    }
    return true;
}

/* ---------- TX ---------- */
void FriendFinderModule::sendFriendFinderPacket(uint32_t dst,
                                                meshtastic_FriendFinder_RequestType type,
                                                uint8_t hopLimit)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) {
        LOG_ERROR("[FriendFinder] allocForSending failed");
        return;
    }

    if (hopLimit > 0) p->hop_limit = hopLimit;
    p->want_ack = false;
    p->to = dst;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = FRIEND_FINDER_PORTNUM;
    p->decoded.has_friend_finder = true;

    meshtastic_FriendFinder *fff = &p->decoded.friend_finder;
    *fff = meshtastic_FriendFinder_init_default;
    fff->request_type  = type;

    if (gpsStatus->getHasLock()) {
        fff->latitude_i   = gpsStatus->getLatitude();
        fff->longitude_i  = gpsStatus->getLongitude();
        fff->sats_in_view = gpsStatus->getNumSatellites();
    }
    fff->battery_level = powerStatus->getBatteryChargePercent();
    fff->time          = getValidTime(RTCQualityFromNet);

    // Mirror to payload bytes so Router can carry it uniformly
    {
        pb_ostream_t os = pb_ostream_from_buffer(p->decoded.payload.bytes,
                                                 sizeof(p->decoded.payload.bytes));
        if (pb_encode(&os, meshtastic_FriendFinder_fields, fff)) {
            p->decoded.payload.size = os.bytes_written;
        } else {
            p->decoded.payload.size = 0;
        }
    }

    // Debug exact on-wire bytes
    size_t enc = 0;
    bool ok = pb_get_encoded_size(&enc, meshtastic_FriendFinder_fields, fff);
    LOG_DEBUG("[FriendFinder] TX pkt id=0x%08x to=0x%08x port=%u hop=%u type=%s(%d) encSize=%u ok=%d",
              (unsigned)p->id, (unsigned)dst, (unsigned)p->decoded.portnum,
              (unsigned)p->hop_limit, ffTypeName(type), (int)type, (unsigned)enc, (int)ok);
    if (ok && enc > 0 && enc < 256) {
        uint8_t buf[256];
        pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
        if (pb_encode(&os, meshtastic_FriendFinder_fields, fff)) {
            hexdump("TX hex", buf, os.bytes_written);
        }
    }

    ::service->sendToMesh(p, RX_SRC_LOCAL, false);
    lastSentPacketTime = millis();
}

const char *FriendFinderModule::getNodeName(uint32_t nodeNum)
{
    if (nodeNum == NODENUM_BROADCAST) return "Broadcast";
    meshtastic_NodeInfoLite *info = nodeDB->getMeshNode(nodeNum);
    static char fallback[32]; 

    if (info && info->has_user) {
        if (strlen(info->user.long_name) > 0) return info->user.long_name;
        if (strlen(info->user.short_name) > 0) {
            snprintf(fallback, sizeof(fallback), "!%s", info->user.short_name);
            return fallback;
        }
    }

    snprintf(fallback, sizeof(fallback), "0x%08X", (unsigned)nodeNum);
    return fallback;
}

const char *FriendFinderModule::getShortName(uint32_t nodeNum)
{
    meshtastic_NodeInfoLite *info = nodeDB->getMeshNode(nodeNum);
    static char fallback[5]; // "XXXX\0"

    if (info && info->has_user && strlen(info->user.short_name) > 0) {
        return info->user.short_name;
    }
    
    // Fallback to last 4 hex digits of node number
    snprintf(fallback, sizeof(fallback), "%04X", (unsigned)(nodeNum & 0xFFFF));
    return fallback;
}


void FriendFinderModule::raiseUIEvent(UIFrameEvent::Action a, bool focus)
{
#if HAS_SCREEN
    UIFrameEvent e; e.action = a;
    if (focus) requestFocus();
    notifyObservers(&e);
#endif
}

/* ---------- Drawing ---------- */
#if HAS_SCREEN

// --- helpers (drawing only) ---
void FriendFinderModule::drawMenuList(OLEDDisplay *d, int16_t x, int16_t y, int W, int H,
                         const char* const* rows, int N, int sel, const char *title)
{
    d->setFont(FONT_SMALL);
    const int titleH = FONT_HEIGHT_SMALL;
    const int rowH   = FONT_HEIGHT_SMALL + 2;
    const int top    = y + titleH + 2;

    // Title
    d->drawString(x + 2, y, title);

    // How many rows fit?
    const int maxR  = std::max(1, (H - (top - y)) / rowH);
    const int first = std::max(0, std::min(sel - maxR / 2, N - maxR));

    for (int r = 0; r < maxR && (first + r) < N; ++r) {
        const int yy = top + r * rowH;
        const bool isSel = (first + r == sel);
        if (isSel) d->drawString(x + 0, yy, ">");
        d->drawString(x + 10, yy, rows[first + r]);
    }
}

void FriendFinderModule::drawFriendList(OLEDDisplay *d, int16_t x, int16_t y, int W, int H, int sel)
{
    d->setFont(FONT_SMALL);
    const int titleH = FONT_HEIGHT_SMALL;
    const int rowH   = FONT_HEIGHT_SMALL + 2;
    const int top    = y + titleH + 2;

    // Title
    d->drawString(x + 2, y, "Track a Friend");

    const int friendCount = getUsedFriendsCount();
    const int totalRows = friendCount + 1; // +1 for "Back"

    // How many rows fit?
    const int maxR  = std::max(1, (H - (top - y)) / rowH);
    const int first = std::max(0, std::min(sel - maxR / 2, totalRows - maxR));

    for (int r = 0; r < maxR && (first + r) < totalRows; ++r) {
        const int listIdx = first + r; // 0 is "Back", 1 is first friend, etc.
        const int yy = top + r * rowH;
        const bool isSel = (listIdx == sel);
        if (isSel) d->drawString(x + 0, yy, ">");
        
        if (listIdx == 0) {
            d->drawString(x + 10, yy, "Back");
            continue;
        }

        // It's a friend, get their details
        const int slot = getFriendSlotByListIndex(listIdx);
        if (slot < 0) continue; 

        char lineBuf[48];
        const char *name = getNodeName(friends_[slot].node);
        
        // Calculate and append distance if possible
        char distBuf[16] = "(?\\?)";
        const auto& peerData = friends_[slot].last_data;
        if (gpsStatus->getHasLock() && (peerData.latitude_i != 0 || peerData.longitude_i != 0)) {
            GeoCoord me(gpsStatus->getLatitude(), gpsStatus->getLongitude(), 0);
            GeoCoord fr(peerData.latitude_i, peerData.longitude_i, 0);
            float dist = me.distanceTo(fr);

            if (config.display.units == meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL) {
                float feet = dist * METERS_TO_FEET;
                if (feet < 1000) snprintf(distBuf, sizeof(distBuf), "(%.0fft)", feet);
                else snprintf(distBuf, sizeof(distBuf), "(%.1fmi)", feet / MILES_TO_FEET);
            } else {
                if (dist < 1000) snprintf(distBuf, sizeof(distBuf), "(%.0fm)", dist);
                else snprintf(distBuf, sizeof(distBuf), "(%.1fkm)", dist / 1000.0f);
            }
        }
        
        snprintf(lineBuf, sizeof(lineBuf), "%s %s", name, distBuf);
        d->drawString(x + 10, yy, lineBuf);
    }
}

void FriendFinderModule::drawFriendMap(OLEDDisplay *d, int16_t x, int16_t y, int W, int H)
{
    d->setFont(FONT_SMALL);
    const int16_t cx = x + W / 2;
    const int16_t cy = y + H / 2;
    const int16_t mapRadius = std::min(W, H) / 2 - 2;

    // Title
    d->drawString(x + 2, y, "Friend Map");

    if (!gpsStatus->getHasLock()) {
        d->setTextAlignment(TEXT_ALIGN_CENTER);
        d->drawString(cx, cy - (FONT_HEIGHT_SMALL/2), "Waiting for GPS...");
        return;
    }

    // Find the maximum distance to any friend to set the scale
    float maxDist = 0.0f;
    for (const auto& f : friends_) {
        if (f.used && (f.last_data.latitude_i != 0 || f.last_data.longitude_i != 0)) {
            GeoCoord me(gpsStatus->getLatitude(), gpsStatus->getLongitude(), 0);
            GeoCoord fr(f.last_data.latitude_i, f.last_data.longitude_i, 0);
            maxDist = std::max(maxDist, (float)me.distanceTo(fr));
        }
    }
    if (maxDist < 50.0f) maxDist = 50.0f; // Min scale distance

    // Calculate scale and get heading
    float scale = (float)mapRadius / maxDist;
    float headingRad = (magnetometerModule && magnetometerModule->hasHeading()) 
                     ? (magnetometerModule->getHeading() * M_PI / 180.0f) : 0.0f;

    // Draw scale text
    char scaleBuf[32];
    snprintf(scaleBuf, sizeof(scaleBuf), "%.1fm/px", 1.0f / scale);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 2, y, scaleBuf);

    // Draw North Indicator
    float northAngle = (config.display.compass_north_top == false) ? -headingRad : 0.0f;
    int16_t nx = cx + (mapRadius - FONT_HEIGHT_SMALL) * sinf(northAngle);
    int16_t ny = cy - (mapRadius - FONT_HEIGHT_SMALL) * cosf(northAngle);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(nx, ny - (FONT_HEIGHT_SMALL / 2), "N");

    // Draw self (center dot)
    d->fillCircle(cx, cy, 2);
    d->drawCircle(cx, cy, 2);

    // Draw friends
    GeoCoord me(gpsStatus->getLatitude(), gpsStatus->getLongitude(), 0);
    for (const auto& f : friends_) {
        if (f.used && (f.last_data.latitude_i != 0 || f.last_data.longitude_i != 0)) {
            GeoCoord fr(f.last_data.latitude_i, f.last_data.longitude_i, 0);
            float bearingRad = me.bearingTo(fr);
            float distance = me.distanceTo(fr);
            float screenAngle = bearingRad;
            if (config.display.compass_north_top == false) {
                screenAngle -= headingRad; // Rotate map for "heading up" view
            }
            float screenDist = distance * scale;
            if (screenDist > mapRadius) screenDist = mapRadius; // Clamp to edge

            int16_t friendX = cx + screenDist * sinf(screenAngle);
            int16_t friendY = cy - screenDist * cosf(screenAngle);

            d->drawCircle(friendX, friendY, 2);
            
            if (friendMapNamesVisible) {
                const char* name = getShortName(f.node);
                d->drawString(friendX + 4, friendY - (FONT_HEIGHT_SMALL/2), name);
            }
        }
    }

    // Draw the map menu overlay if visible
    if (friendMapMenuVisible) {
        static const char* rows[NUM_MAP_MENU] = { "Toggle Names", "Back to Map", "Exit" };
        this->drawMenuList(d, x, y, W, H, rows, NUM_MAP_MENU, friendMapMenuIndex, "Map Menu");
    }
}


static inline const char* truncName(const char* s, char* out, size_t outsz, int maxChars)
{
    if (!s) s = "Friend";
    size_t n = strnlen(s, outsz - 1);
    if ((int)n <= maxChars) { strlcpy(out, s, outsz); return out; }
    if (maxChars <= 1) { strlcpy(out, s, outsz); return out; }
    size_t keep = (size_t)maxChars - 1;
    memcpy(out, s, keep);
    out[keep] = '\xEF'; // '…' fallback glyph if needed
    out[keep + 1] = 0;
    return out;
}



void FriendFinderModule::drawSessionPage(OLEDDisplay *d, int16_t x, int16_t y, int W, int H,
                            const char* peerName,
                            const meshtastic_FriendFinder& peerData,
                            bool haveFix,
                            int32_t myLat, int32_t myLon,
                            uint32_t ageSec, uint32_t lastFriendPacketTime)
{
    d->setFont(FONT_SMALL);
    char nameBuf[24];
    char distBuf[16], bearingBuf[16], agoBuf[16], batBuf[16], satsBuf[16];

    truncName(peerName, nameBuf, sizeof(nameBuf), 12);

    bool havePeerPos = (peerData.latitude_i != 0 || peerData.longitude_i != 0);
    bool haveBoth = haveFix && havePeerPos;
    float bearingDeg = 0;
    if (haveBoth) {
        GeoCoord me(myLat, myLon, 0);
        GeoCoord fr(peerData.latitude_i, peerData.longitude_i, 0);
        float currentDistance = me.distanceTo(fr);
        bearingDeg = me.bearingTo(fr) * 180 / PI; if (bearingDeg < 0) bearingDeg += 360;
        
        char trendIndicator[3] = " ";
        if(previousDistance >= 0.0f) {
            if(currentDistance < previousDistance - 1.0f) trendIndicator[0] = 25; // Down arrow
            else if (currentDistance > previousDistance + 1.0f) trendIndicator[0] = 24; // Up arrow
        }
        previousDistance = currentDistance;

        if (config.display.units == meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL) {
            float feet = currentDistance * METERS_TO_FEET;
            if (feet < 1000) snprintf(distBuf, sizeof(distBuf), "%.0fft%s", feet, trendIndicator);
            else snprintf(distBuf, sizeof(distBuf), "%.1fmi%s", feet / MILES_TO_FEET, trendIndicator);
        } else {
            if (currentDistance < 1000) snprintf(distBuf, sizeof(distBuf), "%.0fm%s", currentDistance, trendIndicator);
            else snprintf(distBuf, sizeof(distBuf), "%.1fkm%s", currentDistance / 1000, trendIndicator);
        }
        snprintf(bearingBuf, sizeof(bearingBuf), "%.0f\xB0", bearingDeg); // Degree symbol
    } else {
        strlcpy(distBuf, "--      ", sizeof(distBuf));
        strlcpy(bearingBuf, "--\xB0", sizeof(bearingBuf));
        previousDistance = -1.0f;
    }

    snprintf(batBuf, sizeof(batBuf), "%u%% Bat", (unsigned)peerData.battery_level);
    snprintf(satsBuf, sizeof(satsBuf), "%u Sats", (unsigned)peerData.sats_in_view);

    if (lastFriendPacketTime == 0) {
        strlcpy(agoBuf, "Waiting...", sizeof(agoBuf));
    } else if (ageSec > 999) {
        strlcpy(agoBuf, ">999s ago", sizeof(agoBuf));
    } else {
        snprintf(agoBuf, sizeof(agoBuf), "%lus ago", ageSec);
    }
    
    // --- New Layout ---

    // 1. Header (minimal padding)
    const int headerH = FONT_HEIGHT_SMALL;
    char titleBuf[48];
    snprintf(titleBuf, sizeof(titleBuf), "Tracking: %s", nameBuf);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(x + W / 2, y, titleBuf);
    // d->drawLine(x + 4, y + headerH, x + W - 4, y + headerH);

    // 2. Content Area (Arrow)
    const int footerH = FONT_HEIGHT_SMALL * 2; // Footer is just the two lines of text
    const int contentH = H - headerH - footerH; // Maximized content area
    
    // Shift center point down slightly for a lower-slung arrow
    const int cy = y + headerH + (contentH / 2) + 3;
    const int cx = x + W / 2;

    if (haveBoth) {
        const float headingRad = magnetometerModule && magnetometerModule->hasHeading()
                                ? magnetometerModule->getHeading() * M_PI / 180.0f
                                : 0.0f;

        float arrowTheta = (bearingDeg * PI / 180.0f);
        if (config.display.compass_north_top == false) arrowTheta -= headingRad;

        // Set arrow to fill 100% of the new, larger content area
        const float arrowSize = (float)contentH;
        // Make the arrow very narrow/long to improve directionality
        const float arrowWidth = arrowSize * 0.4f; 

        // Define points for a simple triangle arrow pointing UP (0 rad) relative to origin (0,0)
        float p1x = 0;                    float p1y = -arrowSize / 2.0f; // Tip
        float p2x = -arrowWidth / 2.0f;   float p2y = arrowSize / 2.0f;  // Base-left
        float p3x = arrowWidth / 2.0f;    float p3y = arrowSize / 2.0f;  // Base-right

        // Rotate points
        float cos_a = cosf(arrowTheta);
        float sin_a = sinf(arrowTheta);

        int16_t r_p1x = p1x * cos_a - p1y * sin_a; int16_t r_p1y = p1x * sin_a + p1y * cos_a;
        int16_t r_p2x = p2x * cos_a - p2y * sin_a; int16_t r_p2y = p2x * sin_a + p2y * cos_a;
        int16_t r_p3x = p3x * cos_a - p3y * sin_a; int16_t r_p3y = p3x * sin_a + p3y * cos_a;
        
        // Translate to screen center and draw
        d->fillTriangle(cx + r_p1x, cy + r_p1y, 
                        cx + r_p2x, cy + r_p2y, 
                        cx + r_p3x, cy + r_p3y);

    } else {
        d->setTextAlignment(TEXT_ALIGN_CENTER);
        d->setFont(FONT_LARGE);
        d->drawString(cx, cy - (FONT_HEIGHT_LARGE / 2), "?");
    }

    // 3. Footer Area (Stats)
    const int footerY1 = y + H - (FONT_HEIGHT_SMALL * 2);
    const int footerY2 = y + H - FONT_HEIGHT_SMALL;

    d->setFont(FONT_SMALL);

    // Row 1: Distance | Data Age
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(x + 2, footerY1, distBuf);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 2, footerY1, agoBuf);
    
    // Row 2: Bearing | Peer Battery | Peer Sats
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(x + 2, footerY2, bearingBuf);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(x + W/2, footerY2, batBuf);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 2, footerY2, satsBuf);
}


void FriendFinderModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    const int16_t W = display->getWidth();
    const int16_t H = display->getHeight();

    // Fast in-frame menu
    if (currentState == FriendFinderState::MENU_SELECTION) {
        static const char* rows[NUM_MENU] = {
            "Back/Exit",
            "Start Pairing",
            "Track a Friend",
            "Friend Map",
            "Compass Calibration"
        };
        this->drawMenuList(display, x, y, W, H, rows, NUM_MENU, menuIndex, "Friend Finder");
        return;
    }
    
    if (currentState == FriendFinderState::CALIBRATION_MENU) {
        static const char* rows[NUM_CAL_MENU] = {
            "Back",
            "Figure-8 Cal",
            "Flat-Spin Cal",
            "Set North Here",
            "Clear North Offset",
            "Dump Cal to Log"
        };
        this->drawMenuList(display, x, y, W, H, rows, NUM_CAL_MENU, calibrationMenuIndex, "Compass Calibration");
        return;
    }

    if (currentState == FriendFinderState::FRIEND_MAP) {
        drawFriendMap(display, x, y, W, H);
        return;
    }

    if (currentState == FriendFinderState::FRIEND_LIST) {
        drawFriendList(display, x, y, W, H, friendListIndex);
        return;
    }

    if (currentState == FriendFinderState::FRIEND_LIST_ACTION) {
        static const char *rows[NUM_FRIEND_ACTIONS] = { "Track", "Remove", "Back" };
        const char *friendName = getNodeName(friends_[getFriendSlotByListIndex(friendListIndex)].node);
        this->drawMenuList(display, x, y, W, H, rows, NUM_FRIEND_ACTIONS, overlayIndex, friendName);
        return;
    }   
    
    // In-session Menu
    if (currentState == FriendFinderState::TRACKING_MENU) {
        static const char* rows[NUM_OVERLAY] = {"Stop Tracking", "Back"};
        this->drawMenuList(display, x, y, W, H, rows, NUM_OVERLAY, overlayIndex, "Session Menu");
        return;
    }

    // Pairing wait page (no footer)
    if (currentState == FriendFinderState::AWAITING_RESPONSE) {
        display->setFont(FONT_SMALL);
        display->drawString(x + 2, y, "Friend Finder");
        const int line0 = y + FONT_HEIGHT_SMALL + 2;
        const int32_t remainMs = (int32_t)(pairingWindowOpen ? (pairingWindowExpiresAt - millis()) : 0);
        const int remain = remainMs > 0 ? (remainMs + 999) / 1000 : 0;
        char buf[48];
        snprintf(buf, sizeof(buf), "Requesting… %ds left", remain);
        display->drawString(x + 2, line0, buf);
        display->drawString(x + 2, line0 + FONT_HEIGHT_SMALL + 2, "Waiting for response...");
        return;
    }

    // Unified session page for both roles
    if (currentState == FriendFinderState::TRACKING_TARGET ||
        currentState == FriendFinderState::BEING_TRACKED)
    {
        const char* peerName = getNodeName(targetNodeNum);
        int32_t myLat = gpsStatus->getLatitude();
        int32_t myLon = gpsStatus->getLongitude();
        bool haveFix  = gpsStatus->getHasLock();

        uint32_t ageSec = 0;
        if (lastFriendPacketTime) {
            uint32_t now = millis();
            ageSec = (now >= lastFriendPacketTime) ? ((now - lastFriendPacketTime) / 1000U) : 0;
        }

        this->drawSessionPage(display, x, y, W, H, peerName, lastFriendData,
                        haveFix, myLat, myLon, ageSec, this->lastFriendPacketTime);
    }
}
#endif // HAS_SCREEN

bool FriendFinderModule::shouldDraw()
{
    return currentState == FriendFinderState::TRACKING_TARGET ||
           currentState == FriendFinderState::BEING_TRACKED ||
           currentState == FriendFinderState::AWAITING_RESPONSE ||
           currentState == FriendFinderState::MENU_SELECTION ||
           currentState == FriendFinderState::TRACKING_MENU   ||
           currentState == FriendFinderState::FRIEND_LIST      ||
           currentState == FriendFinderState::FRIEND_LIST_ACTION ||
           currentState == FriendFinderState::CALIBRATION_MENU ||
           currentState == FriendFinderState::FRIEND_MAP;
}