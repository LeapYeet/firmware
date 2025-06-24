#pragma once

#include "mesh/MeshModule.h"
#include "mesh/generated/meshtastic/friend_finder.pb.h"
#include "mesh/mesh-pb-constants.h"

class FriendFinderModule : public MeshModule {
public:
    FriendFinderModule();
    ~FriendFinderModule() override = default;

    void setup() override;
    void loop() override;
    void handlePacket(const meshtastic_MeshPacket *packet);
    bool wantPacket(const meshtastic_MeshPacket *packet) override;

    void startSearch();
    void stop();

    // Define a port number for this module in the private application range
    static const uint8_t PORT_FRIEND_FINDER = meshtastic_PortNum_PRIVATE_APP;

private:
    enum class Mode { IDLE, SEARCHING, TRACKING };
    Mode     mode_;
    uint32_t lastSendMs_;
    uint32_t lastRecvMs_;
    const meshtastic_MeshPacket *pendingPacket_;

    static constexpr const char *TAG = "FriendFinder";

    void sendProtobuf(const meshtastic_FriendFinder &msg);
    void sendHello();
    void sendStatus();
};