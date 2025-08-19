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
    TRACKING_MENU,
    FRIEND_LIST,
    FRIEND_LIST_ACTION,
    CALIBRATION_MENU,
    FRIEND_MAP
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
        // -- New fields for background updates --
        meshtastic_FriendFinder last_data;
        uint32_t last_heard_time; // millis()
    };
    static constexpr int MAX_FRIENDS = 8;
    FriendRecord friends_[MAX_FRIENDS]{};
    void loadFriends();
    void saveFriends();
    int  findFriend(uint32_t node) const;
    void upsertFriend(uint32_t node, uint32_t session_id, const uint8_t secret[16]);
    int getUsedFriendsCount() const;
    int getFriendSlotByListIndex(int listIdx) const;
    void removeFriendAt(int listIdx);


private:
    FriendFinderState currentState = FriendFinderState::IDLE;
    FriendFinderState previousState = FriendFinderState::IDLE;

    uint32_t          targetNodeNum = 0;
    meshtastic_FriendFinder lastFriendData {};
    uint32_t          lastFriendPacketTime = 0;
    uint32_t          lastSentPacketTime   = 0;
    uint32_t          lastBackgroundUpdateTime = 0; // For periodic friend pings

    // Pairing window
    bool              pairingWindowOpen = false;
    uint32_t          pairingWindowExpiresAt = 0;
    static constexpr uint32_t PAIRING_WINDOW_MS = 30000;

    // --- Menu States ---
    static constexpr int NUM_MENU = 5;
    int  menuIndex = 0;
    
    static constexpr int NUM_CAL_MENU = 6;
    int calibrationMenuIndex = 0;

    static constexpr int NUM_OVERLAY = 2; // In-session menu
    int  overlayIndex = 0;
    
    static constexpr int NUM_FRIEND_ACTIONS = 3;
    int friendListIndex = 0;

    // --- Friend Map UI State ---
    bool friendMapNamesVisible = true;
    bool friendMapMenuVisible = false;
    static constexpr int NUM_MAP_MENU = 3; // "Toggle Names", "Back", "Exit"
    int friendMapMenuIndex = 0;

    // GPS high-power mode management
    bool isGpsHighPower = false;
    uint32_t originalGpsUpdateInterval = 0;
    void activateHighGpsMode();
    void restoreNormalGpsMode();
    
    // Distance trend tracking
    float previousDistance = -1.0f;

    // Input handling
    CallbackObserver<FriendFinderModule, const InputEvent *> inputObserver {
        this, &FriendFinderModule::handleInputEvent };

    int  handleInputEvent(const InputEvent *ev);

    // Helpers
    void sendFriendFinderPacket(uint32_t dst,
                                meshtastic_FriendFinder_RequestType type,
                                uint8_t hopLimit = 0);
    void startTracking(uint32_t nodeNum);
    void requestMutualTracking(uint32_t nodeNum);
    void endSession(bool notifyPeer);
    void raiseUIEvent(UIFrameEvent::Action a, bool focus = false);
    bool shouldDraw();
    const char *getNodeName(uint32_t nodeNum);
    const char *getShortName(uint32_t nodeNum);
    
#if HAS_SCREEN
    // Drawing helpers are member functions to access state
    void drawMenuList(OLEDDisplay *d, int16_t x, int16_t y, int W, int H,
                      const char* const* rows, int N, int sel, const char *title);
    void drawFriendList(OLEDDisplay *d, int16_t x, int16_t y, int W, int H, int sel);
    void drawFriendMap(OLEDDisplay *d, int16_t x, int16_t y, int W, int H);
    void drawSessionPage(OLEDDisplay *d, int16_t x, int16_t y, int W, int H,
                         const char* peerName,
                         const meshtastic_FriendFinder& peerData,
                         bool haveFix,
                         int32_t myLat, int32_t myLon,
                         uint32_t ageSec, uint32_t lastFriendPacketTime);
#endif

    static FriendFinderModule *instance;

    // Calibration UI state (simple)
    bool calWasActive = false;     // Figure-8
    bool flatCalWasActive = false; // Flat-spin
};

extern FriendFinderModule *friendFinderModule;