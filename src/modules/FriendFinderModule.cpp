// File: src/modules/FriendFinderModule.cpp
#include "modules/FriendFinderModule.h"

// CHANGED: We now explicitly include every header we need. No more guessing.
#include "main.h"                // For millis()
#include <pb_decode.h>           // For pb_decode and its types
#include <pb_encode.h>           // For pb_encode and its types
#include "RadioInterface.h"      // For the global sendPacket() function and BROADCAST_ADDR
#include "DeviceState.h"         // For the global deviceState object
#include "modules/PositionModule.h" // To get the positionModule pointer

// The TAG is now initialized in the header, so this line is removed.

FriendFinderModule::FriendFinderModule()
    : MeshModule("FriendFinder"),
      mode_(Mode::IDLE),
      lastSendMs_(0),
      lastRecvMs_(0),
      pendingPacket_(nullptr) {}

bool FriendFinderModule::wantPacket(const meshtastic_MeshPacket *packet)
{
    if (packet->decoded.portnum == PORT_FRIEND_FINDER) {
        pendingPacket_ = packet;
        return true;
    }
    return false;
}

void FriendFinderModule::setup()
{
    LOGI(TAG, "FriendFinderModule initialized (IDLE)");
}

void FriendFinderModule::loop()
{
    if (pendingPacket_) {
        handlePacket(pendingPacket_);
        pendingPacket_ = nullptr;
    }

    uint32_t nowMs = millis();
    switch (mode_) {
        case Mode::SEARCHING:
            if (nowMs - lastSendMs_ > 1000) {
                sendHello();
                lastSendMs_ = nowMs;
            }
            break;
        case Mode::TRACKING:
            if (nowMs - lastSendMs_ > 5000) {
                sendStatus();
                lastSendMs_ = nowMs;
            }
            break;
        default:
            break;
    }
}

void FriendFinderModule::handlePacket(const meshtastic_MeshPacket *packet)
{
    meshtastic_FriendFinder msg = meshtastic_FriendFinder_init_default;
    pb_istream_t istream       = pb_istream_from_buffer(packet->decoded.payload.bytes, packet->decoded.payload.size);
    if (!pb_decode(&istream, meshtastic_FriendFinder_fields, &msg)) {
        LOGW(TAG, "PB decode failed: %s", PB_GET_ERROR(&istream));
        return;
    }

    lastRecvMs_ = millis();
    switch (msg.which_payload) {
        case meshtastic_FriendFinder_hello_tag:
            LOGI(TAG, "RX Hello from 0x%08x", msg.node_id);
            mode_ = Mode::TRACKING;
            lastSendMs_ = 0;
            break;
        case meshtastic_FriendFinder_status_tag:
            LOGI(TAG, "RX Status lat=%f lon=%f batt=%u%%",
                 (float)msg.payload.status.lat_e7 / 1e7f,
                 (float)msg.payload.status.lon_e7 / 1e7f,
                 msg.payload.status.batt_pct);
            break;
        case meshtastic_FriendFinder_ack_tag:
            LOGI(TAG, "RX Ack");
            break;
        default:
            break;
    }
}

void FriendFinderModule::sendProtobuf(const meshtastic_FriendFinder &msg)
{
    meshtastic_MeshPacket p;
    p.channel = 0;
    p.to      = BROADCAST_ADDR; // from RadioInterface.h
    p.from    = ourNode->nodeNum;   // from RadioInterface.h
    p.want_ack = false;
    p.id      = 0;
    p.hop_limit = 3;

    // CHANGED: Add a static_cast to fix the portnum type conversion error.
    p.decoded.portnum = static_cast<meshtastic_PortNum>(PORT_FRIEND_FINDER);

    uint8_t buffer[meshtastic_FriendFinder_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (!pb_encode(&stream, meshtastic_FriendFinder_fields, &msg)) {
        LOGW(TAG, "Failed to encode protobuf");
        return;
    }

    p.decoded.payload.size = stream.bytes_written;
    memcpy(p.decoded.payload.bytes, buffer, stream.bytes_written);

    sendPacket(&p); // from RadioInterface.h
}

void FriendFinderModule::startSearch()
{
    mode_ = Mode::SEARCHING;
    lastSendMs_ = 0;
    LOGI(TAG, "Mode -> SEARCHING");
}

void FriendFinderModule::stop()
{
    mode_ = Mode::IDLE;
    LOGI(TAG, "Mode -> IDLE");
}

void FriendFinderModule::sendHello()
{
    meshtastic_FriendFinder msg = meshtastic_FriendFinder_init_default;
    msg.node_id               = ourNode->nodeNum;
    msg.which_payload         = meshtastic_FriendFinder_hello_tag;
    msg.payload.hello.flags   = 0;
    sendProtobuf(msg);
    LOGD(TAG, "TX Hello");
}

void FriendFinderModule::sendStatus()
{
    meshtastic_FriendFinder msg = meshtastic_FriendFinder_init_default;
    msg.node_id               = ourNode->nodeNum;
    msg.which_payload         = meshtastic_FriendFinder_status_tag;

    if (positionModule && positionModule->fixValid()) {
        msg.payload.status.lat_e7 = positionModule->lastGpsLatitude;
        msg.payload.status.lon_e7 = positionModule->lastGpsLongitude;
        msg.payload.status.alt_cm = positionModule->lastGpsAltitude * 100;
        msg.payload.status.sats = positionModule->satsInView;
        msg.payload.status.unix_sec = positionModule->lastGpsFixTime;
    }

    // from DeviceState.h
    msg.payload.status.batt_pct = deviceState.getBatteryLevel();

    sendProtobuf(msg);
    LOGD(TAG, "TX Status");
}