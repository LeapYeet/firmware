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
#include "graphics/draw/MenuHandler.h"
#include "main.h"
#include "input/InputBroker.h"
#include <cmath>

#include <algorithm>
#include <cstring>

// nanopb
#include "pb_encode.h"
#include "pb_decode.h"

#define FRIEND_FINDER_PORTNUM meshtastic_PortNum_FRIEND_FINDER_APP
#define UPDATE_INTERVAL       20    // seconds
#define BACKGROUND_UPDATE_INTERVAL 120   // seconds (2 minutes)
#define HIGH_GPS_INTERVAL     2     // seconds
#define DEFAULT_GPS_INTERVAL  300   // 5 minutes

#if defined(ARDUINO_ARCH_ESP32)
  #include <Preferences.h>
  static Preferences g_prefs;
  #define FF_HAVE_NVS 1
#else
  #define FF_HAVE_NVS 0
#endif

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

    if (config.position.gps_update_interval > 0 && config.position.gps_update_interval <= HIGH_GPS_INTERVAL) {
        LOG_WARN("[FriendFinder] GPS interval is low (%d sec), restoring default. Was device rebooted during a session?", config.position.gps_update_interval);
        config.position.gps_update_interval = DEFAULT_GPS_INTERVAL;
        service->reloadConfig(SEGMENT_CONFIG);
    }
}

int FriendFinderModule::getUsedFriendsCount() const {
    int cnt = 0;
    for (const auto &f : friends_) if (f.used) ++cnt;
    return cnt;
}

const FriendFinderModule::FriendRecord& FriendFinderModule::getFriendRecord(int slot) const {
    return friends_[slot];
}

const FriendFinderModule::FriendRecord* FriendFinderModule::getFriendByListIndex(int listIdx) const {
    int slot = getFriendSlotByListIndex(listIdx);
    if (slot < 0) return nullptr;
    return &friends_[slot];
}

int FriendFinderModule::getFriendSlotByListIndex(int listIdx) const {
    if (listIdx <= 0) return -1;
    int i = 1;
    for (int slot = 0; slot < MAX_FRIENDS; ++slot) {
        if (friends_[slot].used) {
            if (i == listIdx) return slot;
            ++i;
        }
    }
    return -1;
}

void FriendFinderModule::removeFriendByListIndex(int listIdx) {
    int slot = getFriendSlotByListIndex(listIdx);
    if (slot < 0) return;
    friends_[slot].used = false;
    saveFriends();
    LOG_INFO("[FriendFinder] Removed friend at slot %d", slot);
}

void FriendFinderModule::loadFriends() {
    for (auto &f : friends_) {
        f = {};
        f.last_data = meshtastic_FriendFinder_init_default;
    }

#if FF_HAVE_NVS
    if (!g_prefs.begin("ffinder", false)) {
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
        if (idx < 0) idx = 0;
    }
    friends_[idx].node = node;
    friends_[idx].session_id = session_id;
    if (secret) memcpy(friends_[idx].secret, secret, 16); else memset(friends_[idx].secret, 0, 16);
    friends_[idx].used = true;
    saveFriends();
    LOG_INFO("[FriendFinder] Saved friend 0x%08x at slot %d", (unsigned)node, idx);
}

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

void FriendFinderModule::launchMenu()
{
#if HAS_SCREEN
    graphics::menuHandler::menuQueue = graphics::menuHandler::friend_finder_base_menu;
    screen->runNow();
    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
}

void FriendFinderModule::setState(FriendFinderState s) {
    currentState = s;
    if (s == FriendFinderState::IDLE) {
        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
    } else {
        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
    }
}

void FriendFinderModule::beginPairing()
{
    pairingWindowOpen = true;
    pairingWindowExpiresAt = millis() + PAIRING_WINDOW_MS;
    currentState = FriendFinderState::PAIRING_DISCOVERY;
    pairingCandidateNodeNum = 0;
    rejectedPeers.clear();

    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);

    sendFriendFinderPacket(NODENUM_BROADCAST, meshtastic_FriendFinder_RequestType_REQUEST, 1);
    lastSentPacketTime = millis();
}

void FriendFinderModule::showConfirmationPrompt(uint32_t fromNode)
{
    if (currentState != FriendFinderState::PAIRING_DISCOVERY) return;

    for (uint32_t rejectedNode : rejectedPeers) {
        if (fromNode == rejectedNode) {
            LOG_DEBUG("[FriendFinder] Ignoring request from already rejected peer 0x%08x", (unsigned)fromNode);
            return;
        }
    }

    LOG_INFO("[FriendFinder] Proposing pair with candidate 0x%08x", (unsigned)fromNode);
    pairingCandidateNodeNum = fromNode;
    currentState = FriendFinderState::AWAITING_CONFIRMATION;

    char msg[64];
    snprintf(msg, sizeof(msg), "Pair with %s?", getShortName(fromNode));

    static const char *options[] = {"No", "Yes"};
    graphics::BannerOverlayOptions bannerOptions;
    bannerOptions.message = msg;
    bannerOptions.optionsArrayPtr = options;
    bannerOptions.optionsCount = 2;
    bannerOptions.durationMs = 15000;
    bannerOptions.bannerCallback = [](int selected) {
        if (!friendFinderModule) return;
        if (selected == 1) { // Yes
            friendFinderModule->acceptPairingRequest();
        } else { // No or timeout
            friendFinderModule->rejectPairingRequest();
        }
    };
    screen->showOverlayBanner(bannerOptions);
}

void FriendFinderModule::acceptPairingRequest()
{
    if (pairingCandidateNodeNum == 0) return;

    LOG_INFO("[FriendFinder] User accepted initial pairing with 0x%08x. Sending ACCEPT and waiting for their ACCEPT.", (unsigned)pairingCandidateNodeNum);

    sendFriendFinderPacket(pairingCandidateNodeNum, meshtastic_FriendFinder_RequestType_ACCEPT);
    currentState = FriendFinderState::AWAITING_FINAL_ACCEPTANCE;
    raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
}

void FriendFinderModule::rejectPairingRequest()
{
    LOG_INFO("[FriendFinder] User rejected pairing or request timed out.");

    if (pairingCandidateNodeNum != 0) {
        sendFriendFinderPacket(pairingCandidateNodeNum, meshtastic_FriendFinder_RequestType_REJECT);
        rejectedPeers.push_back(pairingCandidateNodeNum);
    }

    pairingCandidateNodeNum = 0;
    if (pairingWindowOpen && (int32_t)(millis() - pairingWindowExpiresAt) < 0) {
        currentState = FriendFinderState::PAIRING_DISCOVERY;
        raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
    } else {
        currentState = FriendFinderState::IDLE;
        pairingWindowOpen = false;
        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
        screen->showSimpleBanner("Pairing cancelled", 1200);
    }
}

void FriendFinderModule::completePairing(uint32_t nodeNum)
{
    if (findFriend(nodeNum) < 0) {
        uint32_t sess = (uint32_t)random(1, 0x7fffffff);
        uint8_t  sec[16]; for (int i = 0; i < 16; ++i) sec[i] = random(0, 255);
        upsertFriend(nodeNum, sess, sec);
    }

    sendFriendFinderPacket(nodeNum, meshtastic_FriendFinder_RequestType_NONE);

    pairingWindowOpen = false;
    pairingCandidateNodeNum = 0;
    currentState = FriendFinderState::IDLE;

    char msg[64];
    snprintf(msg, sizeof(msg), "%s Paired!", getShortName(nodeNum));
    screen->showSimpleBanner(msg, 2500);

    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);

    graphics::menuHandler::menuQueue = graphics::menuHandler::friend_finder_base_menu;
    screen->runNow();
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
    sendFriendFinderPacket(nodeNum, meshtastic_FriendFinder_RequestType_REQUEST, 0);
    lastSentPacketTime = millis();
}

void FriendFinderModule::startTracking(uint32_t nodeNum)
{
    if (nodeNum == 0 || nodeNum == nodeDB->getNodeNum()) return;

    targetNodeNum = nodeNum;
    const int friend_idx = findFriend(nodeNum);

    if (friend_idx >= 0) {
        LOG_INFO("[FriendFinder] startTracking(): already friends with 0x%08x -> start immediately", nodeNum);
        currentState = FriendFinderState::TRACKING_TARGET;
        previousDistance = -1.0f;

        lastFriendData = friends_[friend_idx].last_data;
        lastFriendPacketTime = friends_[friend_idx].last_heard_time;

        activateHighGpsMode();
        lastSentPacketTime = 0;
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
    previousDistance = -1.0f;
    restoreNormalGpsMode();
    screen->setFrames(graphics::Screen::FOCUS_DEFAULT);
}

int FriendFinderModule::handleInputEvent(const InputEvent *ev)
{
    if (!ev || !shouldDraw()) return 0;

    if (screen->isOverlayBannerShowing()) {
        return 0;
    }

    const bool isSelect  = ev->inputEvent == INPUT_BROKER_SELECT || ev->inputEvent == INPUT_BROKER_ALT_LONG;
    const bool isBack    = ev->inputEvent == INPUT_BROKER_BACK || ev->inputEvent == INPUT_BROKER_CANCEL;

    if (currentState == FriendFinderState::PAIRING_DISCOVERY ||
        currentState == FriendFinderState::AWAITING_CONFIRMATION ||
        currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE) {
        if (isBack) {
            LOG_INFO("[FriendFinder] User cancelled pairing via back button.");
            currentState = FriendFinderState::IDLE;
            pairingWindowOpen = false;
            pairingCandidateNodeNum = 0;
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
            screen->showSimpleBanner("Pairing cancelled", 1200);
            return 1;
        }
        return 0;
    }

    if (currentState == FriendFinderState::FRIEND_MAP) {
        if (isSelect) {
            graphics::menuHandler::menuQueue = graphics::menuHandler::friend_finder_map_menu;
            screen->runNow();
            return 1;
        }
        if (isBack) {
            setState(FriendFinderState::IDLE);
            return 1;
        }
        return 0;
    }

    if (currentState == FriendFinderState::TRACKING_TARGET ||
        currentState == FriendFinderState::BEING_TRACKED)
    {
        if (isSelect) {
            previousState = currentState;
            graphics::menuHandler::menuQueue = graphics::menuHandler::friend_finder_session_menu;
            screen->runNow();
            return 1;
        }
        if (isBack) {
            endSession(true);
            return 1;
        }
        return 0;
    }

    return 0;
}

int32_t FriendFinderModule::runOnce()
{
    const uint32_t now = millis();

    if (currentState == FriendFinderState::PAIRING_DISCOVERY) {
        if ((now - lastSentPacketTime) > 5000UL) { // Re-broadcast every 5 seconds
            LOG_DEBUG("[FriendFinder] Re-broadcasting pairing request");
            sendFriendFinderPacket(NODENUM_BROADCAST, meshtastic_FriendFinder_RequestType_REQUEST, 1);
        }
    }

    if (currentState == FriendFinderState::FRIEND_MAP) {
#if HAS_SCREEN
        raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
#endif
    }

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
        if (currentState == FriendFinderState::AWAITING_RESPONSE ||
            currentState == FriendFinderState::PAIRING_DISCOVERY ||
            currentState == FriendFinderState::AWAITING_CONFIRMATION ||
            currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE) {
            currentState = FriendFinderState::IDLE;
            pairingCandidateNodeNum = 0;
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
#if HAS_SCREEN
            screen->showSimpleBanner("Pairing timed out", 1200);
#endif
        }
    }

#if HAS_SCREEN
    if (magnetometerModule) {
        const bool calNow = magnetometerModule->isCalibrating();
        if (calNow && !calWasActive) {
            calWasActive = true;
            screen->showSimpleBanner("Move in FIGURE-8", 15000);
        }
        if (!calNow && calWasActive) {
            calWasActive = false;
            screen->showSimpleBanner("Calibration done", 1200);
        }

        const bool flatNow = magnetometerModule->isFlatCalibrating();
        if (flatNow && !flatCalWasActive) {
            flatCalWasActive = true;
            screen->showSimpleBanner("Spin device CLOCKWISE", 15000);
        }
        if (!flatNow && flatCalWasActive) {
            flatCalWasActive = false;
            screen->showSimpleBanner("Calibration done", 1200);
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
        if ((now - lastSentPacketTime) > UPDATE_INTERVAL * 1000UL && targetNodeNum) {
            sendFriendFinderPacket(targetNodeNum, meshtastic_FriendFinder_RequestType_NONE);
        }
        break;
    default: break;
    }

    return 50;
}

bool FriendFinderModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                                meshtastic_FriendFinder *ff)
{
    const uint32_t from = getFrom(&mp);
    if (from == nodeDB->getNodeNum()) return true;

    if (!mp.decoded.has_friend_finder) return false;

    LOG_INFO("[FriendFinder] RX pkt id=0x%08x from=0x%08x to=0x%08x port=%u hop=%u/%u chan=%u",
             (unsigned)mp.id, (unsigned)from, (unsigned)mp.to, (unsigned)mp.decoded.portnum,
             (unsigned)mp.hop_limit, (unsigned)mp.hop_start, (unsigned)mp.channel);

    if (mp.decoded.payload.size) hexdump("RX raw", mp.decoded.payload.bytes, mp.decoded.payload.size);

    LOG_INFO("[FriendFinder] RX FF type=%s(%d) batt=%u sats=%u lat=%d lon=%d time=%u state=%d",
             ffTypeName(ff->request_type), (int)ff->request_type,
             (unsigned)ff->battery_level, (unsigned)ff->sats_in_view,
             (int)ff->latitude_i, (int)ff->longitude_i, (unsigned)ff->time,
             (int)currentState);

    switch (ff->request_type) {
    case meshtastic_FriendFinder_RequestType_REQUEST: {
        if (currentState == FriendFinderState::PAIRING_DISCOVERY) {
            showConfirmationPrompt(from);
            return true;
        }

        bool isDirectedRequestToMe = (mp.to == nodeDB->getNodeNum());
        if (isDirectedRequestToMe && (findFriend(from) >= 0 || currentState == FriendFinderState::AWAITING_RESPONSE)) {
            targetNodeNum = from;
            currentState  = FriendFinderState::BEING_TRACKED;
            activateHighGpsMode();
            pairingWindowOpen = false;

            if (findFriend(from) < 0) {
                 uint32_t sess = (uint32_t)random(1, 0x7fffffff);
                 uint8_t sec[16]; for (int i = 0; i < 16; ++i) sec[i] = random(0, 255);
                 upsertFriend(from, sess, sec);
            }
            LOG_INFO("[FriendFinder] Directed request from 0x%08x -> ACCEPT", from);
            sendFriendFinderPacket(from, meshtastic_FriendFinder_RequestType_ACCEPT);
            sendFriendFinderPacket(from, meshtastic_FriendFinder_RequestType_NONE); // Immediately send our location
#if HAS_SCREEN
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
#endif
            return true;
        }
        return true;
    }

    case meshtastic_FriendFinder_RequestType_ACCEPT: {
        if (currentState == FriendFinderState::AWAITING_RESPONSE && from == targetNodeNum) {
            LOG_INFO("[FriendFinder] Tracking request to 0x%08x was accepted.", (unsigned)from);
            currentState = FriendFinderState::TRACKING_TARGET;
            activateHighGpsMode();
            pairingWindowOpen = false;
            sendFriendFinderPacket(from, meshtastic_FriendFinder_RequestType_NONE); // Immediately send our location
            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
            return true;
        }

        if (currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE && from == pairingCandidateNodeNum) {
            LOG_INFO("[FriendFinder] Received final acceptance from 0x%08x. Pairing complete!", (unsigned)from);
            completePairing(from);
            return true;
        }
        else if (currentState == FriendFinderState::PAIRING_DISCOVERY) {
            LOG_INFO("[FriendFinder] Received an ACCEPT from 0x%08x while in discovery, treating as proposal.", (unsigned)from);
            showConfirmationPrompt(from);
            return true;
        }
        break;
    }

    case meshtastic_FriendFinder_RequestType_REJECT: {
        if (currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE && from == pairingCandidateNodeNum) {
            LOG_INFO("[FriendFinder] Peer 0x%08x rejected the pairing request.", (unsigned)from);
            
            if (pairingWindowOpen && (int32_t)(millis() - pairingWindowExpiresAt) < 0) {
                currentState = FriendFinderState::PAIRING_DISCOVERY;
                pairingCandidateNodeNum = 0;
                screen->showSimpleBanner("Peer rejected pair", 1500);
                raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
            } else {
                currentState = FriendFinderState::IDLE;
                pairingWindowOpen = false;
                pairingCandidateNodeNum = 0;
                screen->showSimpleBanner("Pairing failed", 1200);
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND, false);
            }
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
            endSession(false);
        }
        break;
    }

    case meshtastic_FriendFinder_RequestType_NONE: {
        if (currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE && from == pairingCandidateNodeNum) {
            LOG_INFO("[FriendFinder] Received final pairing confirmation from 0x%08x.", (unsigned)from);
            completePairing(from);
            return true;
        }

        const int friend_idx = findFriend(from);
        if (friend_idx >= 0) {
            friends_[friend_idx].last_data = *ff;
            friends_[friend_idx].last_heard_time = millis();
            LOG_DEBUG("[FriendFinder] Stored background update from friend 0x%08x", from);
        }

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

    {
        pb_ostream_t os = pb_ostream_from_buffer(p->decoded.payload.bytes,
                                                 sizeof(p->decoded.payload.bytes));
        if (pb_encode(&os, meshtastic_FriendFinder_fields, fff)) {
            p->decoded.payload.size = os.bytes_written;
        } else {
            p->decoded.payload.size = 0;
        }
    }

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
    static char fallback[5];

    if (info && info->has_user && strlen(info->user.short_name) > 0) {
        return info->user.short_name;
    }

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

#if HAS_SCREEN

void FriendFinderModule::drawFriendMap(OLEDDisplay *d, int16_t x, int16_t y, int W, int H)
{
    d->setFont(FONT_SMALL);
    const int16_t cx = x + W / 2;
    const int16_t cy = y + H / 2;
    const int16_t mapRadius = std::min(W, H) / 2 - 2;

    // d->drawString(x + 2, y, "Friend Map");

    if (!gpsStatus->getHasLock()) {
        d->setTextAlignment(TEXT_ALIGN_CENTER);
        d->drawString(cx, cy - (FONT_HEIGHT_SMALL/2), "Waiting for GPS...");
        return;
    }

    float maxDist = 0.0f;
    for (const auto& f : friends_) {
        if (f.used && (f.last_data.latitude_i != 0 || f.last_data.longitude_i != 0)) {
            GeoCoord me(gpsStatus->getLatitude(), gpsStatus->getLongitude(), 0);
            GeoCoord fr(f.last_data.latitude_i, f.last_data.longitude_i, 0);
            maxDist = std::max(maxDist, (float)me.distanceTo(fr));
        }
    }
    if (maxDist < 10.0f) maxDist = 10.0f;

    float scale = (float)mapRadius / maxDist;
    float headingRad = (magnetometerModule && magnetometerModule->hasHeading())
                     ? (magnetometerModule->getHeading() * M_PI / 180.0f) : 0.0f;

    char scaleBuf[32];
    snprintf(scaleBuf, sizeof(scaleBuf), "%.1fm/px", 1.0f / scale);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 2, y, scaleBuf);

    float northAngle = (config.display.compass_north_top == false) ? -headingRad : 0.0f;
    int16_t nx = cx - (mapRadius - FONT_HEIGHT_SMALL) * sinf(northAngle);
    int16_t ny = cy + (mapRadius - FONT_HEIGHT_SMALL) * cosf(northAngle);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(nx, ny - (FONT_HEIGHT_SMALL / 2), "");

    d->fillCircle(cx, cy, 2);
    d->drawCircle(cx, cy, 2);

    GeoCoord me(gpsStatus->getLatitude(), gpsStatus->getLongitude(), 0);
    for (const auto& f : friends_) {
        if (f.used && (f.last_data.latitude_i != 0 || f.last_data.longitude_i != 0)) {
            GeoCoord fr(f.last_data.latitude_i, f.last_data.longitude_i, 0);
            float bearingRad = me.bearingTo(fr);
            float distance = me.distanceTo(fr);
            float screenAngle = bearingRad;
            if (config.display.compass_north_top == false) {
                screenAngle -= headingRad;
            }
            float screenDist = distance * scale;
            if (screenDist > mapRadius) screenDist = mapRadius;

            int16_t friendX = cx - screenDist * sinf(screenAngle);
            int16_t friendY = cy + screenDist * cosf(screenAngle);

            d->drawCircle(friendX, friendY, 2);

            if (friendMapNamesVisible) {
                const char* name = getShortName(f.node);
                d->drawString(friendX + 4, friendY - (FONT_HEIGHT_SMALL/2), name);
            }
        }
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
    out[keep] = '\xEF';
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
    char distBuf[16], agoBuf[16], batBuf[16];

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
            if(currentDistance < previousDistance - 1.0f) trendIndicator[0] = 25;
            else if (currentDistance > previousDistance + 1.0f) trendIndicator[0] = 24;
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
    } else {
        strlcpy(distBuf, "--      ", sizeof(distBuf));
        previousDistance = -1.0f;
    }

    snprintf(batBuf, sizeof(batBuf), "%u%% ", (unsigned)peerData.battery_level);

    if (lastFriendPacketTime == 0) {
        strlcpy(agoBuf, "Waiting...", sizeof(agoBuf));
    } else if (ageSec > 999) {
        strlcpy(agoBuf, ">999s ago", sizeof(agoBuf));
    } else {
        snprintf(agoBuf, sizeof(agoBuf), "%lus ago", ageSec);
    }

    const int headerH = FONT_HEIGHT_SMALL;
    char titleBuf[48];
    snprintf(titleBuf, sizeof(titleBuf), "Tracking: %s", nameBuf);
    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(x + W / 2, y, titleBuf);

    const int footerH = FONT_HEIGHT_SMALL;
    const int contentH = H - headerH - footerH;

    const int cy = y + headerH + (contentH / 2) + 3;
    const int cx = x + W / 2;

    const bool haveHeading = (magnetometerModule && magnetometerModule->hasHeading()) && !forceNoMagnetometerView;

    if (haveBoth && haveHeading) {
        const float headingRad = magnetometerModule->getHeading() * M_PI / 180.0f;
        float arrowTheta = (bearingDeg * PI / 180.0f);
        if (config.display.compass_north_top == false) arrowTheta -= headingRad;

        const float arrowSize = (float)contentH * 0.9f;
        const float arrowWidth = arrowSize * 0.4f;

        float p1x = 0;                    float p1y = -arrowSize / 2.0f;
        float p2x = -arrowWidth / 2.0f;   float p2y = arrowSize / 2.0f;
        float p3x = arrowWidth / 2.0f;    float p3y = arrowSize / 2.0f;

        float cos_a = cosf(arrowTheta);
        float sin_a = sinf(arrowTheta);

        int16_t r_p1x = p1x * cos_a - p1y * sin_a; int16_t r_p1y = p1x * sin_a + p1y * cos_a;
        int16_t r_p2x = p2x * cos_a - p2y * sin_a; int16_t r_p2y = p2x * sin_a + p2y * cos_a;
        int16_t r_p3x = p3x * cos_a - p3y * sin_a; int16_t r_p3y = p3x * sin_a + p3y * cos_a;

        d->fillTriangle(cx + r_p1x, cy + r_p1y,
                        cx + r_p2x, cy + r_p2y,
                        cx + r_p3x, cy + r_p3y);

    } else if (haveBoth) {
        d->setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        d->setFont(FONT_LARGE);
        d->drawString(cx, cy, distBuf);
    }
    else {
        d->setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        d->setFont(FONT_SMALL);
        
        if (!haveFix) {
            d->drawString(cx, cy, "Waiting for\nlocal GPS");
        } else if (!havePeerPos) {
            if (lastFriendPacketTime == 0) {
                 d->drawString(cx, cy, "Waiting for\npeer data");
            } else {
                 d->drawString(cx, cy, "Peer has\nno GPS");
            }
        } else {
            d->setFont(FONT_LARGE);
            d->drawString(cx, cy, "?");
        }
    }

    d->setFont(FONT_SMALL);
    const int footerY = y + H - FONT_HEIGHT_SMALL;

    d->setTextAlignment(TEXT_ALIGN_LEFT);
    d->drawString(x + 2, footerY, distBuf);

    d->setTextAlignment(TEXT_ALIGN_CENTER);
    d->drawString(x + W / 2, footerY, batBuf);

    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(x + W - 2, footerY, agoBuf);
}
void FriendFinderModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    const int16_t W = display->getWidth();
    const int16_t H = display->getHeight();

    if (currentState == FriendFinderState::PAIRING_DISCOVERY ||
        currentState == FriendFinderState::AWAITING_CONFIRMATION ||
        currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE) {
        display->setFont(FONT_SMALL);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + W / 2, y, "Friend Finder Pairing");

        const int32_t remainMs = (int32_t)(pairingWindowOpen ? (pairingWindowExpiresAt - millis()) : 0);
        const int remain = remainMs > 0 ? (remainMs + 999) / 1000 : 0;
        char buf[32];
        snprintf(buf, sizeof(buf), "%ds left", remain);
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(x + W - 2, y, buf);

        display->setTextAlignment(TEXT_ALIGN_CENTER);
        if (currentState == FriendFinderState::PAIRING_DISCOVERY) {
            display->drawString(x + W / 2, y + H / 2, "Looking for peers...");
        } else if (currentState == FriendFinderState::AWAITING_CONFIRMATION) {
            display->drawString(x + W / 2, y + H / 2 - (FONT_HEIGHT_SMALL/2), "Found peer!");
            display->drawString(x + W / 2, y + H / 2 + (FONT_HEIGHT_SMALL/2), "Awaiting confirmation...");
        } else { // AWAITING_FINAL_ACCEPTANCE
             display->drawString(x + W / 2, y + H / 2, "Waiting for peer to accept...");
        }
        return;
    }

    if (currentState == FriendFinderState::FRIEND_MAP) {
        drawFriendMap(display, x, y, W, H);
        return;
    }

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
#endif

bool FriendFinderModule::shouldDraw()
{
    return currentState == FriendFinderState::TRACKING_TARGET ||
           currentState == FriendFinderState::BEING_TRACKED ||
           currentState == FriendFinderState::AWAITING_RESPONSE ||
           currentState == FriendFinderState::FRIEND_MAP ||
           currentState == FriendFinderState::PAIRING_DISCOVERY || 
           currentState == FriendFinderState::AWAITING_CONFIRMATION ||
           currentState == FriendFinderState::AWAITING_FINAL_ACCEPTANCE;
}
