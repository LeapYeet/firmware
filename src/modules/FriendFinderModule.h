// In src/modules/FriendFinderModule.h
#pragma once

#include "MeshModule.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "Observer.h"                         // UIFrameEvent / Observable
#include "input/InputBroker.h"                // InputEvent
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "mesh/generated/meshtastic/friendfinder.pb.h"

// Forward declarations ---------------------------------------------------------
class MeshService;

/*---------------------------------------------------------------------------*/
/* */
/* Friend-Finder state machine                                              */
/* */
/*---------------------------------------------------------------------------*/
enum class FriendFinderState {
    IDLE,               // default Meshtastic carousel
    MENU_SELECTION,     // scrollable FF menu (Back, Start Pairing, …)
    SENDING_REQUEST,    // pairing broadcast sent
    AWAITING_RESPONSE,  // waiting for a friend to accept/reject our request
    AWAITING_CONFIRMATION, // waiting for the local user to confirm a friend's request
    TRACKING_TARGET,    // we are tracking a friend
    BEING_TRACKED       // someone is tracking us
};

/*---------------------------------------------------------------------------*/
/* FriendFinderModule                                                       */
/*---------------------------------------------------------------------------*/
class FriendFinderModule
        : public ProtobufModule<meshtastic_FriendFinder>,
          public concurrency::OSThread,
          public Observable<const UIFrameEvent *>
{
public:
    FriendFinderModule();

    /* — ProtobufModule overrides — */
    bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                meshtastic_FriendFinder *ff) override;

    /* — MeshModule overrides — */
    void    setup()   override;
    int32_t runOnce() override;

    /* — UI frame integration — */
#if HAS_SCREEN
    bool wantUIFrame() override { return shouldDraw() ||
                                          currentState == FriendFinderState::MENU_SELECTION; }
    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                   int16_t x, int16_t y) override;
    Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
#endif

    bool interceptingKeyboardInput() override
    { return currentState == FriendFinderState::MENU_SELECTION || shouldDraw(); }

    /* — Public helpers called from Screen.cpp — */
    void launchMenu();          // open FF main menu
    void beginPairing();        // Start Pairing command action

protected:
    int  handleInputEvent(const InputEvent *ev);

private:
    /* ---------------------------------------------------------------------- */
    MeshService *service = nullptr;

    FriendFinderState currentState = FriendFinderState::IDLE;

    uint32_t          targetNodeNum        = 0;
    uint32_t          pendingPairRequestFrom = 0; // Temp store for the node that sent us a request
    meshtastic_FriendFinder lastFriendData {};
    uint32_t          lastFriendPacketTime = 0;
    uint32_t          lastSentPacketTime   = 0;

    /* scrollable menu bookkeeping */
    static constexpr int NUM_MENU = 6;
    int  menuIndex = 0;

    /* Input observer (buttons/encoder) */
    CallbackObserver<FriendFinderModule, const InputEvent *> inputObserver {
        this, &FriendFinderModule::handleInputEvent };

    /* Helpers */
    void sendFriendFinderPacket(uint32_t dst,
                                meshtastic_FriendFinder_RequestType type,
                                uint8_t hopLimit = 0);
    void raiseUIEvent(UIFrameEvent::Action a, bool focus = false);
    bool shouldDraw();      // compass page active?
    const char *getNodeName(uint32_t nodeNum); // Add this function declaration

    static FriendFinderModule *instance;
};
/* Global pointer so UI / Screen.cpp can talk to us */
extern FriendFinderModule *friendFinderModule;