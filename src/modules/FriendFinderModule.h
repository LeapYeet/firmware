#pragma once
/**
 * FriendFinderModule — scaffold for “Find-My-Friend” feature.
 * Only fulfils the pure-virtual contract of MeshModule for now.
 */

#include "MeshModule.h"           // Meshtastic core base class

class FriendFinderModule : public MeshModule {
public:
    /**
     * @brief MeshModule requires a name string in its ctor. This is the constructor that the compiler is looking for.
     */
    FriendFinderModule() : MeshModule("FriendFinder") {}

    /**
     * @brief This function is called during device startup.
     */
    void setup() override {}

    /**
     * @brief We ignore every packet until real logic arrives.
     * The signature for wantPacket must match the base class exactly.
     */
    bool wantPacket(const meshtastic_MeshPacket *packet) override { return false; }
};