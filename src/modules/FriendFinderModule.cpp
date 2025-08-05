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
#define UPDATE_INTERVAL       30    // seconds
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
void FriendFinderModule::loadFriends() {
    for (auto &f : friends_) f = FriendRecord{0,0,{0},false};

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
        LOG_WARN("[FriendFinder] Unexpected friends blob size=%u, resetting", (unsigned)sz);
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

void FriendFinderModule::startTracking(uint32_t nodeNum)
{
    if (nodeNum == 0 || nodeNum == nodeDB->getNodeNum()) return;

    targetNodeNum = nodeNum;

    // If already friends, start immediately
    if (findFriend(nodeNum) >= 0) {
        LOG_INFO("[FriendFinder] startTracking(): already friends with 0x%08x -> start immediately", nodeNum);
        currentState = FriendFinderState::TRACKING_TARGET;
        previousDistance = -1.0f; // Reset distance trend
        activateHighGpsMode(); // Activate real-time GPS
        lastSentPacketTime = 0; // force quick first beacon
#if HAS_SCREEN
        screen->showSimpleBanner("Tracking started", 1200);
#endif
        sendFriendFinderPacket(nodeNum, meshtastic_FriendFinder_RequestType_NONE, 0);
        return;
    }

    // Otherwise, quick request (peer must be in pairing window)
    currentState  = FriendFinderState::AWAITING_RESPONSE;
    pairingWindowOpen = true;
    pairingWindowExpiresAt = millis() + PAIRING_WINDOW_MS;

#if HAS_SCREEN
    screen->showSimpleBanner("Request sent… Press Pairing on peer", 1500);
#endif
    sendFriendFinderPacket(nodeNum, meshtastic_FriendFinder_RequestType_REQUEST, 0);
    lastSentPacketTime = millis();
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

    const bool btnUp    = ev->inputEvent == INPUT_BROKER_UP;
    const bool btnDown  = ev->inputEvent == INPUT_BROKER_DOWN || ev->inputEvent == INPUT_BROKER_USER_PRESS
                       || ev->inputEvent == INPUT_BROKER_ALT_PRESS;
    const bool btnSel   = ev->inputEvent == INPUT_BROKER_SELECT || ev->inputEvent == INPUT_BROKER_ALT_LONG;
    const bool btnBack  = ev->inputEvent == INPUT_BROKER_BACK || ev->inputEvent == INPUT_BROKER_CANCEL;

    // In-session menu
    if (currentState == FriendFinderState::TRACKING_MENU) {
        if (btnUp)   { overlayIndex = (overlayIndex + NUM_OVERLAY - 1) % NUM_OVERLAY; screen->forceDisplay(); return 1; }
        if (btnDown) { overlayIndex = (overlayIndex + 1) % NUM_OVERLAY; screen->forceDisplay(); return 1; }
        if (btnBack) {
            currentState = previousState; // Go back to tracking view
            screen->forceDisplay();
            return 1;
        }
        if (btnSel) {
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
        if (btnSel) {
            previousState = currentState;
            currentState = FriendFinderState::TRACKING_MENU;
            overlayIndex = 0; // Re-use overlayIndex for menu index
            screen->forceDisplay();
            return 1;
        }
        if (btnBack) { // Allow ending session with BACK button from main tracking screen
            endSession(/*notifyPeer*/true);
            return 1;
        }
        return 0;
    }

    // In-frame main menu (fast)
    if (currentState == FriendFinderState::MENU_SELECTION) {
        if (btnUp)   { menuIndex = (menuIndex + NUM_MENU - 1) % NUM_MENU; screen->forceDisplay(); return 1; }
        if (btnDown) { menuIndex = (menuIndex + 1) % NUM_MENU; screen->forceDisplay(); return 1; }
        if (btnBack) {
            currentState = FriendFinderState::IDLE;
            pairingWindowOpen = false;
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
            return 1;
        }
        if (btnSel) {
            switch (menuIndex) {
            case 0: // Back/Exit
                currentState = FriendFinderState::IDLE;
                pairingWindowOpen = false;
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
                return 1;
            case 1: // Start Pairing
                beginPairing();
                return 1;
            case 2: // Track Friend
            {
#if HAS_SCREEN
                uint32_t firstFriend = 0;
                for (int i = 0; i < MAX_FRIENDS; ++i) if (friends_[i].used) { firstFriend = friends_[i].node; break; }
                if (firstFriend) {
                    startTracking(firstFriend);
                } else {
                    screen->showNodePicker("Track who?", 25000, [this](uint32_t nodenum) {
                        if (nodenum) startTracking(nodenum);
                    });
                }
#endif
                return 1;
            }
            case 3: // List Friends (quick count)
            {
#if HAS_SCREEN
                char buf[96]; int count = 0;
                for (int i = 0; i < MAX_FRIENDS; ++i) if (friends_[i].used) ++count;
                snprintf(buf, sizeof(buf), "Friends saved: %d", count);
                screen->showSimpleBanner(buf, 1200);
#endif
                return 1;
            }
            default: return 1;
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

    if (pairingWindowOpen && (int32_t)(now - pairingWindowExpiresAt) >= 0) {
        pairingWindowOpen = false;
        if (currentState == FriendFinderState::AWAITING_RESPONSE) {
            currentState = FriendFinderState::MENU_SELECTION;
#if HAS_SCREEN
            screen->showSimpleBanner("Pairing timed out", 1200);
#endif
        }
    }

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
        if (findFriend(from) >= 0) {
            targetNodeNum = from;
            currentState  = FriendFinderState::BEING_TRACKED;
            previousDistance = -1.0f; // Reset distance trend
            activateHighGpsMode(); // Activate real-time GPS
            pairingWindowOpen = false;
            LOG_INFO("[FriendFinder] REQUEST from existing friend -> ACCEPT");
            sendFriendFinderPacket(from, meshtastic_FriendFinder_RequestType_ACCEPT);
#if HAS_SCREEN
            // Simply showing the UI is notification enough
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
            return true;
        }

        if (!pairingWindowOpen) {
#if HAS_SCREEN
            screen->showSimpleBanner("Hold Pair on both devices", 1200);
#endif
            return true;
        }

        // Mutual pairing: save friend, accept, and enter session
        targetNodeNum = from;
        currentState  = FriendFinderState::BEING_TRACKED;
        previousDistance = -1.0f; // Reset distance trend
        activateHighGpsMode(); // Activate real-time GPS
        pairingWindowOpen = false;

        uint32_t sess = (uint32_t)random(1, 0x7fffffff);
        uint8_t  sec[16]; for (int i = 0; i < 16; ++i) sec[i] = random(0, 255);
        upsertFriend(from, sess, sec);

        LOG_INFO("[FriendFinder] ACCEPT -> saved friend 0x%08x", (unsigned)from);
        sendFriendFinderPacket(from, meshtastic_FriendFinder_RequestType_ACCEPT);

#if HAS_SCREEN
        // Show the UI instead of just a banner
        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
        break;
    }

    case meshtastic_FriendFinder_RequestType_ACCEPT: {
        if (currentState == FriendFinderState::AWAITING_RESPONSE) {
            targetNodeNum = from;
            currentState  = FriendFinderState::TRACKING_TARGET;
            previousDistance = -1.0f; // Reset distance trend
            activateHighGpsMode(); // Activate real-time GPS
            pairingWindowOpen = false;
            lastFriendPacketTime = millis();
            lastFriendData = *ff;

            // Save friendship for instant future sessions
            uint32_t sess = (uint32_t)random(1, 0x7fffffff);
            uint8_t  sec[16]; for (int i = 0; i < 16; ++i) sec[i] = random(0, 255);
            upsertFriend(from, sess, sec);

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
    if (info && info->has_user && strlen(info->user.long_name) > 0) return info->user.long_name;
    static char fallback[12];
    snprintf(fallback, sizeof(fallback), "!%s", info->user.short_name);
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

// This is now a member function
void FriendFinderModule::drawMenuList(OLEDDisplay *d, int16_t x, int16_t y, int W, int H,
                         const char* const* rows, int N, int sel)
{
    d->setFont(FONT_SMALL);
    const int titleH = FONT_HEIGHT_SMALL;
    const int rowH   = FONT_HEIGHT_SMALL + 2;
    const int top    = y + titleH + 2;

    // Title
    d->drawString(x + 2, y, "Friend Finder");

    // How many rows fit?
    const int maxR  = std::max(1, (H - (top - y)) / rowH);
    const int first = std::max(0, std::min(sel - maxR / 2, N - maxR));

    for (int r = 0; r < maxR && (first + r) < N; ++r) {
        const int yy = top + r * rowH;
        const bool isSel = (first + r == sel);
        // pointer + label (avoid full-screen fill to reduce OLED bandwidth)
        if (isSel) d->drawString(x + 0, yy, ">");
        d->drawString(x + 10, yy, rows[first + r]);
    }
}

// C-style helper for truncating names
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


// This is now a member function to access `this->previousDistance`
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

    // --- Prepare all text strings first ---
    // Peer Name
    truncName(peerName, nameBuf, sizeof(nameBuf), 12);

    // Distance and Bearing
    bool havePeerPos = (peerData.latitude_i != 0 || peerData.longitude_i != 0);
    bool haveBoth = haveFix && havePeerPos;
    float bearingDeg = 0;
    if (haveBoth) {
        GeoCoord me(myLat, myLon, 0);
        GeoCoord fr(peerData.latitude_i, peerData.longitude_i, 0);
        float currentDistance = me.distanceTo(fr);
        bearingDeg = me.bearingTo(fr) * 180 / PI; if (bearingDeg < 0) bearingDeg += 360;
        
        // Prepare trend indicator
        char trendIndicator[3] = " "; // Space, up/down arrow, null terminator
        if(previousDistance >= 0.0f) {
            if(currentDistance < previousDistance - 0.5f) trendIndicator[0] = 25; // Down arrow
            else if (currentDistance > previousDistance + 0.5f) trendIndicator[0] = 24; // Up arrow
        }
        previousDistance = currentDistance; // Update for next frame
        
        if (config.display.units == meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL) {
            float feet = currentDistance * METERS_TO_FEET;
            if (feet < 1000) snprintf(distBuf, sizeof(distBuf), "%.0fft%s", feet, trendIndicator);
            else snprintf(distBuf, sizeof(distBuf), "%.1fmi%s", feet / MILES_TO_FEET, trendIndicator);
        } else {
            if (currentDistance < 1000) snprintf(distBuf, sizeof(distBuf), "%.0fm%s", currentDistance, trendIndicator);
            else snprintf(distBuf, sizeof(distBuf), "%.1fkm%s", currentDistance / 1000, trendIndicator);
        }
        snprintf(bearingBuf, sizeof(bearingBuf), "%.0fdeg", bearingDeg);
    } else {
        strlcpy(distBuf, "Dist --", sizeof(distBuf));
        strlcpy(bearingBuf, "Brg --", sizeof(bearingBuf));
        previousDistance = -1.0f; // Invalidate distance if we lose fix
    }

    // Peer Stats
    snprintf(batBuf, sizeof(batBuf), "%u%% Bat", (unsigned)peerData.battery_level);
    snprintf(satsBuf, sizeof(satsBuf), "%u Sats", (unsigned)peerData.sats_in_view);

    // Last Update Time
    if (lastFriendPacketTime == 0) {
        strlcpy(agoBuf, "waiting...", sizeof(agoBuf));
    } else if (ageSec > 999) {
        strlcpy(agoBuf, ">999s ago", sizeof(agoBuf));
    } else {
        snprintf(agoBuf, sizeof(agoBuf), "%lus ago", ageSec);
    }
    
    // --- Drawing ---
    // Layout Constants
    const int headerH = FONT_HEIGHT_SMALL + 4;
    const int footerH = (FONT_HEIGHT_SMALL * 2) + 6;
    const int contentH = H - headerH - footerH;

    // Header
    char titleBuf[48];
    snprintf(titleBuf, sizeof(titleBuf), "Tracking: %s", nameBuf);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(x + W / 2, y, titleBuf);
    d->drawLine(x + 4, y + FONT_HEIGHT_SMALL + 2, x + W - 4, y + FONT_HEIGHT_SMALL + 2);

    // Arrow (or placeholder) in the main content area
    const int r = std::min(W / 2 - 4, contentH / 2 - 4);
    const int cx = x + W / 2;
    const int cy = y + headerH + contentH / 2;
    if (haveBoth) {
        const float headingRad = magnetometerModule->hasHeading() ? magnetometerModule->getHeading() * M_PI / 180.0f : 0.0f;

        float arrowTheta = (bearingDeg * PI / 180.0f);
        if (config.display.compass_north_top == false) arrowTheta -= headingRad;
        graphics::CompassRenderer::drawNodeHeading(d, cx, cy, r * 1.2f, arrowTheta); // Made arrow slightly larger
    } else {
        d->setTextAlignment(TEXT_ALIGN_CENTER);
        d->setFont(FONT_LARGE);
        d->drawString(cx, cy - (FONT_HEIGHT_LARGE / 2), "?");
        d->setFont(FONT_SMALL);
    }

    // Footer with two rows of data
    const int footerY1 = y + H - (FONT_HEIGHT_SMALL * 2) - 2;
    const int footerY2 = y + H - FONT_HEIGHT_SMALL;

    // Footer Line 1
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(x + 4, footerY1, distBuf);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(x + W/2, footerY1, agoBuf);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 4, footerY1, batBuf);
    
    // Footer Line 2
    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(x + 4, footerY2, bearingBuf);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 4, footerY2, satsBuf);
}

void FriendFinderModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    const int16_t W = display->getWidth();
    const int16_t H = display->getHeight();

    // Fast in-frame menu
    if (currentState == FriendFinderState::MENU_SELECTION) {
        static const char* rows[NUM_MENU] = {"Back/Exit", "Start Pairing", "Track Friend", "List Friends"};
        this->drawMenuList(display, x, y, W, H, rows, NUM_MENU, menuIndex);
        return;
    }
    
    // New In-session Menu
    if (currentState == FriendFinderState::TRACKING_MENU) {
        static const char* rows[NUM_OVERLAY] = {"Stop Tracking", "Back"};
        this->drawMenuList(display, x, y, W, H, rows, NUM_OVERLAY, overlayIndex);
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
        snprintf(buf, sizeof(buf), "Pairing… %ds left", remain);
        display->drawString(x + 2, line0, buf);
        display->drawString(x + 2, line0 + FONT_HEIGHT_SMALL + 2, "Press on BOTH devices");
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
           currentState == FriendFinderState::TRACKING_MENU;
}