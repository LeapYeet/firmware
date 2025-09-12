#pragma once

#include "MeshModule.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "Observer.h"
#include "input/InputBroker.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/generated/meshtastic/friendfinder.pb.h"

#if HAS_SCREEN
#include "graphics/Screen.h"
#include "graphics/SharedUIDisplay.h"
#endif

#include <vector>

class MeshService;

enum class FriendFinderState : uint8_t {
    IDLE = 0,
    PAIRING_DISCOVERY,
    AWAITING_RESPONSE,
    AWAITING_CONFIRMATION,
    AWAITING_FINAL_ACCEPTANCE,
    TRACKING_TARGET,
    BEING_TRACKED,
    FRIEND_MAP,
    COMPASS_SCREEN,
    TRACKING_SPOOFED_TARGET
};

class FriendFinderModule
  : public ProtobufModule<meshtastic_FriendFinder>,
    public concurrency::OSThread,
    public Observable<const UIFrameEvent *>
{
public:
    FriendFinderModule();

    // ---- Public structs and constants for MenuHandler ----
    struct FriendRecord {
        uint32_t node;
        uint32_t session_id;
        uint8_t  secret[16];
        bool     used;
        meshtastic_FriendFinder last_data;
        uint32_t last_heard_time; // millis()
    };
    static constexpr int MAX_FRIENDS = 8;

    // ---- Public API for MenuHandler ----
    int getUsedFriendsCount() const;
    const FriendRecord& getFriendRecord(int slot) const;
    const FriendRecord* getFriendByListIndex(int listIdx) const;
    void removeFriendByListIndex(int listIdx);
    void setState(FriendFinderState s);
    bool friendMapNamesVisible = true;
    bool forceNoMagnetometerView = false;
    bool spoofModeEnabled = false;

    // ---- Core Module Functions ----
    bool    handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                   meshtastic_FriendFinder *ff) override;
    void    setup() override;
    int32_t runOnce() override;

#if HAS_SCREEN
    bool wantUIFrame() override {
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
    void requestMutualTracking(uint32_t nodeNum);
    void endSession(bool notifyPeer);
    void startSpoofedTracking(int direction);

    // Public for banner callback access
    void acceptPairingRequest();
    void rejectPairingRequest();

private:
    // -------- persist friends --------
    FriendRecord friends_[MAX_FRIENDS]{};
    void loadFriends();
    void saveFriends(); 
    int  findFriend(uint32_t node) const;
    void upsertFriend(uint32_t node, uint32_t session_id, const uint8_t secret[16]);
    int getFriendSlotByListIndex(int listIdx) const;
    
private:
    FriendFinderState currentState = FriendFinderState::IDLE;
    FriendFinderState previousState = FriendFinderState::IDLE;
    int spoofedDirection = 0;
    
    uint32_t          targetNodeNum = 0;
    meshtastic_FriendFinder lastFriendData {};
    uint32_t          lastFriendPacketTime = 0;
    uint32_t          lastSentPacketTime   = 0;
    uint32_t          lastBackgroundUpdateTime = 0;
    uint32_t          lastDebugLogMs = 0; 

    // Pairing window
    bool              pairingWindowOpen = false;
    uint32_t          pairingWindowExpiresAt = 0;
    uint32_t          pairingCandidateNodeNum = 0;
    std::vector<uint32_t> rejectedPeers;
    static constexpr uint32_t PAIRING_WINDOW_MS = 30000;

    // GPS high-power mode management
    bool isGpsHighPower = false;
    uint32_t originalGpsUpdateInterval = 0;
    void activateHighGpsMode();
    void restoreNormalGpsMode();

    void completePairing(uint32_t nodeNum);
    void showConfirmationPrompt(uint32_t fromNode);

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
    void raiseUIEvent(UIFrameEvent::Action a, bool focus = false);
    bool shouldDraw();

#if HAS_SCREEN
    // Drawing helpers are member functions to access state
    void drawFriendMap(OLEDDisplay *d, int16_t x, int16_t y, int W, int H);
    void drawSimpleCompass(OLEDDisplay *d, int16_t x, int16_t y, int W, int H);
    void drawFigure8Cal(OLEDDisplay *d, int16_t x, int16_t y, int W, int H);
    void drawFlatSpinCal(OLEDDisplay *d, int16_t x, int16_t y, int W, int H);
    void drawSessionPage(OLEDDisplay *d, int16_t x, int16_t y, int W, int H,
                         const char* peerName,
                         const meshtastic_FriendFinder& peerData,
                         bool haveFix,
                         int32_t myLat, int32_t myLon,
                         uint32_t ageSec, uint32_t lastFriendPacketTime);
#endif

    static FriendFinderModule *instance;

    // Calibration UI state (simple)
    bool calWasActive = false;
    bool flatCalWasActive = false;

public: // Public helpers for MenuHandler
    const char *getNodeName(uint32_t nodeNum);
    const char *getShortName(uint32_t nodeNum);
};

extern FriendFinderModule *friendFinderModule;