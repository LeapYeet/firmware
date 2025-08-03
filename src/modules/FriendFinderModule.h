#pragma once

#include "MeshModule.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "Observer.h"
#include "input/InputBroker.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/generated/meshtastic/friendfinder.pb.h"

class MeshService;

enum class FriendFinderState : uint8_t {
    IDLE = 0,
    MENU_SELECTION,
    AWAITING_RESPONSE,
    AWAITING_CONFIRMATION,
    TRACKING_TARGET,
    BEING_TRACKED,
    TRACKING_MENU
};

class FriendFinderModule
  : public ProtobufModule<meshtastic_FriendFinder>,
    public concurrency::OSThread,
    public Observable<const UIFrameEvent *>
{
public:
    FriendFinderModule();

    bool    handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                   meshtastic_FriendFinder *ff) override;
    void    setup() override;
    int32_t runOnce() override;

#if HAS_SCREEN
    bool wantUIFrame() override {
        // Only ask for a UI frame when we actually display something
        return shouldDraw();
    }
    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                   int16_t x, int16_t y) override;
    Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
#endif

    bool interceptingKeyboardInput() override {
        return shouldDraw();
    }

    void launchMenu();
    void beginPairing();

private:
    // -------- persist friends --------
    struct FriendRecord {
        uint32_t node;
        uint32_t session_id;
        uint8_t  secret[16];
        bool     used;
    };
    static constexpr int MAX_FRIENDS = 8;
    FriendRecord friends_[MAX_FRIENDS]{};
    void loadFriends();
    void saveFriends();
    int  findFriend(uint32_t node) const;
    void upsertFriend(uint32_t node, uint32_t session_id, const uint8_t secret[16]);

private:
    FriendFinderState currentState = FriendFinderState::IDLE;
    FriendFinderState previousState = FriendFinderState::IDLE;

    uint32_t          targetNodeNum = 0;
    meshtastic_FriendFinder lastFriendData {};
    uint32_t          lastFriendPacketTime = 0;
    uint32_t          lastSentPacketTime   = 0;

    // Pairing window
    bool              pairingWindowOpen = false;
    uint32_t          pairingWindowExpiresAt = 0;
    static constexpr uint32_t PAIRING_WINDOW_MS = 30000;

    // Main menu
    static constexpr int NUM_MENU = 4; // Back/Exit, Start Pairing, Track Friend, List Friends
    int  menuIndex = 0;

    // Overlay (while tracking)
    static constexpr int NUM_OVERLAY = 2; // Stop Tracking, Back
    int  overlayIndex = 0;

    // Input handling
    CallbackObserver<FriendFinderModule, const InputEvent *> inputObserver {
        this, &FriendFinderModule::handleInputEvent };

    int  handleInputEvent(const InputEvent *ev);

    // Helpers
    void sendFriendFinderPacket(uint32_t dst,
                                meshtastic_FriendFinder_RequestType type,
                                uint8_t hopLimit = 0);
    void startTracking(uint32_t nodeNum);
    void endSession(bool notifyPeer);
    void raiseUIEvent(UIFrameEvent::Action a, bool focus = false);
    bool shouldDraw();
    const char *getNodeName(uint32_t nodeNum);

    static FriendFinderModule *instance;
};

extern FriendFinderModule *friendFinderModule;