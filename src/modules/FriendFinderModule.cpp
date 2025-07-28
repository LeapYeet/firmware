// In src/modules/FriendFinderModule.cpp
#include "FriendFinderModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "GPS.h"
#include "Power.h"
#include "TypeConversions.h"

#include "graphics/Screen.h"
#include "graphics/draw/CompassRenderer.h"
#include "graphics/draw/NotificationRenderer.h"
#include "graphics/TimeFormatters.h"
#include "main.h"                      // globals: screen, router, config, etc.
#include <algorithm>
#include "input/InputBroker.h"


/* -------------------------------------------------------------------------- */
#define FRIEND_FINDER_PORTNUM meshtastic_PortNum_PRIVATE_APP
#define UPDATE_INTERVAL       30       // seconds
#define PAIRING_TIMEOUT       30000    // 30 seconds to wait for a pairing response

/* -------------------------------------------------------------------------- */
FriendFinderModule *friendFinderModule      = nullptr;   // global pointer
FriendFinderModule *FriendFinderModule::instance = nullptr;

/* -------------------------------------------------------------------------- */
/* FriendFinderModule.cpp  – constructor */
FriendFinderModule::FriendFinderModule()
    : ProtobufModule("friendfinder",
                     FRIEND_FINDER_PORTNUM,
                     meshtastic_FriendFinder_fields),
      OSThread("FriendFinder")
{
    instance      = this;
    this->ourPortNum = FRIEND_FINDER_PORTNUM;

    if (inputBroker) {
        inputObserver.observe(inputBroker);   // capture button events
        LOG_INFO("FF: inputBroker observer attached");
    } else {
        LOG_WARN("FF: inputBroker is null – no button events!");
    }
}


/* -------------------------------------------------------------------------- */
void FriendFinderModule::setup()
{
    LOG_INFO("FriendFinderModule: setup()");
}

/* -------------------------------------------------------------------------- */
void FriendFinderModule::launchMenu()
{
#if HAS_SCREEN
    menuIndex    = 0;
    currentState = FriendFinderState::MENU_SELECTION;

    raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, /*focus=*/true);
#endif
}



/* -------------------------------------------------------------------------- */
void FriendFinderModule::beginPairing()
{
    LOG_INFO("FF: Begin pairing, broadcasting request with hopLimit=1");
    currentState = FriendFinderState::AWAITING_RESPONSE; // We are now waiting for someone to accept
    sendFriendFinderPacket(NODENUM_BROADCAST,
                           meshtastic_FriendFinder_RequestType_REQUEST,
                           1); // hopLimit = 1 as requested
    screen->showOverlayBanner("Broadcasting Pair Request...", 5000);
    lastSentPacketTime = millis(); // For timeout tracking
}


/* -------------------------------------------------------------------------- */
int FriendFinderModule::handleInputEvent(const InputEvent *ev)
{
    // Don't handle input if we are waiting for on-screen banner confirmation
    if (currentState != FriendFinderState::MENU_SELECTION)
        return 0;  // let others handle

    /***** DEBUG-ENHANCED INPUT MAPPING  *************************************/
    LOG_INFO("FF key: raw=%d  repeat=%d  idx=%d",
            ev->inputEvent, menuIndex);

    bool up   = ev->inputEvent == INPUT_BROKER_UP;
    bool down = ev->inputEvent == INPUT_BROKER_DOWN  ||
                ev->inputEvent == INPUT_BROKER_USER_PRESS ||
                ev->inputEvent == INPUT_BROKER_ALT_PRESS;
    bool sel  = ev->inputEvent == INPUT_BROKER_SELECT   ||
                ev->inputEvent == INPUT_BROKER_ALT_LONG;
    bool back = ev->inputEvent == INPUT_BROKER_BACK || ev->inputEvent == INPUT_BROKER_CANCEL;

    LOG_DEBUG("FF map: up=%d dn=%d sel=%d back=%d",
            up, down, sel, back);
            
    /* scrolling ------------------------------------------------------ */
    if (up   && menuIndex > 0)              { menuIndex--; screen->forceDisplay(); return 1; }
    if (down && menuIndex < NUM_MENU - 1)   { menuIndex++; screen->forceDisplay(); return 1; }

    /* exit ----------------------------------------------------------- */
    if (back) {
        currentState = FriendFinderState::IDLE;
        raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND);
        return 1;
    }

    /* select --------------------------------------------------------- */
    if (sel) {
        if (menuIndex == 1) beginPairing();
        /* TODO other items */
        return 1;
    }
    return 0;
}



/* -------------------------------------------------------------------------- */
int32_t FriendFinderModule::runOnce()
{
    switch (currentState) {
        case FriendFinderState::AWAITING_RESPONSE:
            if (millis() - lastSentPacketTime > PAIRING_TIMEOUT) {
                LOG_INFO("FF: Pairing request timed out.");
                screen->showOverlayBanner("Pairing timed out", 3000);
                currentState = FriendFinderState::MENU_SELECTION;
            }
            break;

        case FriendFinderState::BEING_TRACKED:
            if (millis() - lastSentPacketTime > UPDATE_INTERVAL * 1000) {
                sendFriendFinderPacket(targetNodeNum,
                                       meshtastic_FriendFinder_RequestType_NONE);
            }
            [[fallthrough]];         // also redraw compass
        case FriendFinderState::TRACKING_TARGET:
#if HAS_SCREEN
            if (shouldDraw())
                raiseUIEvent(UIFrameEvent::Action::REDRAW_ONLY);
#endif
            break;
        default:
            break;
    }
    return 1000;                                    // sleep 1 s
}



/* -------------------------------------------------------------------------- */
bool FriendFinderModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                                meshtastic_FriendFinder *ff)
{
    uint32_t from = getFrom(&mp);
    if (from == nodeDB->getNodeNum()) return true; // Ignore our own packets

    LOG_INFO("FF: Received packet from %#x, type %d, state %d", from, ff->request_type, (int)currentState);

    switch (ff->request_type) {
        case meshtastic_FriendFinder_RequestType_REQUEST:
        {
            // Only respond to requests if we are idle or in the menu. Avoids handling multiple requests.
            if (currentState == FriendFinderState::MENU_SELECTION || currentState == FriendFinderState::IDLE) {
                const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(from);
                char bannerMsg[128];
                snprintf(bannerMsg, sizeof(bannerMsg), "Pair with %s?\nYes\nNo",
                         (node && node->has_user) ? node->user.long_name : "Unknown");

                pendingPairRequestFrom = from;
                currentState = FriendFinderState::AWAITING_CONFIRMATION;

                screen->showOverlayBanner(bannerMsg, 30000, 2, [this](int selected) {
                    if (this->pendingPairRequestFrom != 0) {
                        if (selected == 0) { // Yes
                            LOG_INFO("FF: Accepting pair request from %#x", this->pendingPairRequestFrom);
                            sendFriendFinderPacket(this->pendingPairRequestFrom, meshtastic_FriendFinder_RequestType_ACCEPT);
                            this->targetNodeNum = this->pendingPairRequestFrom;
                            this->currentState = FriendFinderState::BEING_TRACKED; // We are now being tracked
                            screen->showOverlayBanner("Paired! You can be tracked.", 5000);
                            raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true); // Go to compass screen
                        } else { // No or timeout
                            LOG_INFO("FF: Rejecting pair request from %#x", this->pendingPairRequestFrom);
                            sendFriendFinderPacket(this->pendingPairRequestFrom, meshtastic_FriendFinder_RequestType_REJECT);
                            this->currentState = FriendFinderState::IDLE;
                        }
                        this->pendingPairRequestFrom = 0; // Clear the pending request
                    }
                });
            }
            break;
        }

        case meshtastic_FriendFinder_RequestType_ACCEPT:
        {
            // We only care about ACCEPT if we were the one who sent the request
            if (currentState == FriendFinderState::AWAITING_RESPONSE) {
                const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(from);
                char bannerMsg[128];
                snprintf(bannerMsg, sizeof(bannerMsg), "Paired with %s!",
                         (node && node->has_user) ? node->user.long_name : "Unknown");

                screen->showOverlayBanner(bannerMsg, 5000);

                targetNodeNum = from;
                lastFriendPacketTime = millis();
                lastFriendData = *ff;
                currentState = FriendFinderState::TRACKING_TARGET;
                raiseUIEvent(UIFrameEvent::Action::REGENERATE_FRAMESET, true); // Go to compass screen
            }
            break;
        }

        case meshtastic_FriendFinder_RequestType_REJECT:
        {
            if (currentState == FriendFinderState::AWAITING_RESPONSE) {
                const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(from);
                char bannerMsg[128];
                snprintf(bannerMsg, sizeof(bannerMsg), "Pairing rejected by %s",
                         (node && node->has_user) ? node->user.long_name : "Unknown");
                screen->showOverlayBanner(bannerMsg, 5000);
                currentState = FriendFinderState::MENU_SELECTION; // Go back to menu
            }
            break;
        }

        case meshtastic_FriendFinder_RequestType_NONE:
        {
            // This is a regular data update. Only process if it's from our tracked friend.
            if (from == targetNodeNum) {
                lastFriendData       = *ff;
                lastFriendPacketTime = millis();
                LOG_DEBUG("FF: Received location update from %#x", from);
            }
            break;
        }

        default:
            LOG_WARN("FF: Received unknown request type %d", ff->request_type);
            break;
    }
    return true;
}

/* -------------------------------------------------------------------------- */
void FriendFinderModule::sendFriendFinderPacket(
        uint32_t dst, meshtastic_FriendFinder_RequestType type, uint8_t hopLimit)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) { LOG_ERROR("FF: allocForSending failed"); return; }

    if (hopLimit > 0) { // Override default hop limit if specified
        p->hop_limit = hopLimit;
    }

    p->want_ack  = false; // Pairing packets are broadcast, so no ACK
    p->to        = dst;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag; // We are populating the 'decoded' part of the packet

    // Populate the Data struct
    p->decoded.portnum = FRIEND_FINDER_PORTNUM;
    p->decoded.has_friend_finder = true;

    meshtastic_FriendFinder *ff = &p->decoded.friend_finder;
    ff->request_type = type;

    if (gpsStatus->getHasLock()) {
        ff->latitude_i   = gpsStatus->getLatitude();
        ff->longitude_i  = gpsStatus->getLongitude();
        ff->sats_in_view = gpsStatus->getNumSatellites();
    }
    ff->battery_level = powerStatus->getBatteryChargePercent();
    ff->time = getValidTime(RTCQualityFromNet);

    // CORRECTED: The 'false' argument tells the service not to send status to the phone, preventing the crash.
    service->sendToMesh(p, RX_SRC_LOCAL, false);
    
    lastSentPacketTime = millis();
}

const char *FriendFinderModule::getNodeName(uint32_t nodeNum)
{
    if (nodeNum == NODENUM_BROADCAST)
        return "Broadcast";

    meshtastic_NodeInfoLite *info = nodeDB->getMeshNode(nodeNum);
    if (info && info->has_user && strlen(info->user.long_name) > 0) {
        return info->user.long_name;
    }

    // Fallback to hex ID if no name is available
    static char fallback[12];
    snprintf(fallback, sizeof(fallback), "0x%08x", nodeNum);
    return fallback;
}

/* -------------------------------------------------------------------------- */
void FriendFinderModule::raiseUIEvent(UIFrameEvent::Action a, bool focus)
{
#if HAS_SCREEN
    UIFrameEvent e;
    e.action = a;
    if (focus) requestFocus();
    notifyObservers(&e);
#endif
}

/* -------------------------------------------------------------------------- */
#if HAS_SCREEN
bool FriendFinderModule::shouldDraw()
{
    return currentState == FriendFinderState::TRACKING_TARGET ||
           currentState == FriendFinderState::BEING_TRACKED;
}

/* -------------------------------------------------------------------------- */
void FriendFinderModule::drawFrame(OLEDDisplay *display,
                                   OLEDDisplayUiState *state,
                                   int16_t x, int16_t y)
{
    /* ---- MENU SELECTION PAGE ------------------------------------------- */
    if (currentState == FriendFinderState::MENU_SELECTION) {
        display->setFont(FONT_SMALL);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + display->getWidth()/2, y, "Friend Finder");
        display->setTextAlignment(TEXT_ALIGN_LEFT);

        const char *rows[NUM_MENU] = {
            "Back",
            "Start Pairing",
            "Track Friend",
            "Friend Map",
            "List Friends",
            "SOS"
        };

        int rowH     = FONT_HEIGHT_SMALL;
        int top      = y + FONT_HEIGHT_SMALL + 2;
        int visible  = (display->getHeight() - top) / rowH;
        int firstRow = std::max(0, std::min(menuIndex - visible/2, NUM_MENU - visible));

        for (int r = 0; r < visible && firstRow+r < NUM_MENU; ++r) {
            int yy = top + r*rowH;
            bool highlight = (firstRow+r == menuIndex);
            if (highlight) { display->fillRect(x, yy-1, display->getWidth(), rowH); display->setColor(BLACK); }
            display->drawString(x+2, yy, rows[firstRow+r]);
            if (highlight) display->setColor(WHITE);
        }

        if (NUM_MENU > visible) {          // simple scrollbar
            int trackX = display->getWidth() - 4;
            int trackH = visible * rowH;
            display->drawRect(trackX, top, 3, trackH);
            int barH = std::max(4, (trackH * visible) / NUM_MENU);
            int barY = top + (trackH * firstRow) / NUM_MENU;
            display->fillRect(trackX, barY, 3, barH);
        }
        return;
    }

    /* ---- COMPASS / TRACK PAGE ----------------------------------------- */
    if (!shouldDraw()) return;

    // header
    display->setFont(FONT_SMALL);
    display->drawString(x, y, "Friend Finder");

    int line0 = y + FONT_HEIGHT_SMALL + 2;
    char buf[32];

    if (currentState == FriendFinderState::TRACKING_TARGET) {
        if (lastFriendPacketTime == 0) {
            display->drawString(x, line0, "Awaiting data…");
            return;
        }

        int32_t myLat = gpsStatus->getLatitude();
        int32_t myLon = gpsStatus->getLongitude();
        GeoCoord me(myLat, myLon, 0);
        GeoCoord friendLoc(lastFriendData.latitude_i,
                           lastFriendData.longitude_i, 0);

        float meters = me.distanceTo(friendLoc);
        float bearingRad = me.bearingTo(friendLoc);
        float bearingDeg = bearingRad * 180 / PI;
        if (bearingDeg < 0) bearingDeg += 360;

        if (config.display.units ==
            meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL) {
            float feet = meters * METERS_TO_FEET;
            if (feet < 2 * MILES_TO_FEET) snprintf(buf, sizeof(buf), "%.0fft", feet);
            else snprintf(buf, sizeof(buf), "%.1fmi", feet / MILES_TO_FEET);
        } else {
            if (meters < 2000) snprintf(buf, sizeof(buf), "%.0fm", meters);
            else snprintf(buf, sizeof(buf), "%.1fkm", meters/1000);
        }
        display->drawString(x, line0, buf);

        snprintf(buf, sizeof(buf), "Bearing %.0f°", bearingDeg);
        display->drawString(x, line0 + FONT_HEIGHT_SMALL + 2, buf);

        snprintf(buf, sizeof(buf), "Batt %d%%  Sats %d",
                 lastFriendData.battery_level, lastFriendData.sats_in_view);
        display->drawString(x, line0 + 2*(FONT_HEIGHT_SMALL+2), buf);

        // compass
        uint16_t diam = graphics::CompassRenderer::getCompassDiam(
                            display->getWidth(), display->getHeight());
        int16_t cx = x + display->getWidth() - diam/2 - 5;
        int16_t cy = y + display->getHeight()/2;
        display->drawCircle(cx, cy, diam/2);

        float headingRad = screen->hasHeading() ?
                           screen->getHeading() * PI / 180 :
                           screen->estimatedHeading(DegD(myLat), DegD(myLon));
        graphics::CompassRenderer::drawCompassNorth(display, cx, cy,
                                                    headingRad, diam/2);

        float arrowTheta = bearingRad;
        if (!config.display.compass_north_top) arrowTheta -= headingRad;
        graphics::CompassRenderer::drawNodeHeading(display, cx, cy,
                                                   diam, arrowTheta);
    }
    else if (currentState == FriendFinderState::BEING_TRACKED) {
        display->drawString(x, line0, "You are being tracked");
        snprintf(buf, sizeof(buf), "by %s", getNodeName(targetNodeNum));
        display->drawString(x, line0 + FONT_HEIGHT_SMALL + 2, buf);
    }
}
#endif      // HAS_SCREEN