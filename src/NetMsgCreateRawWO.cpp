#pragma once
#include "NetMsg_callback_decl.h"
#include "ManagerSerializableNetMsgMap.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include <memory>
#include "NetMsg.h"
#include <vector>
#include <memory>
#include <string>
#include "NetMsgCreateRawWO.h"
#include "GLViewSpeedRacer.h"
#include "GLView.h"
#include <new.h>
#include <ManagerGLView.h>
#include <WO.h>

using namespace Aftr;

// Define the NetMsgBlackScreen class
NetMsgMacroDefinition(NetMsgBlackScreen);

// Constructor
NetMsgBlackScreen::NetMsgBlackScreen() : NetMsg() {}

// Destructor
NetMsgBlackScreen::~NetMsgBlackScreen() {}

// Serialize the message (no additional data needed)
bool NetMsgBlackScreen::toStream(NetMessengerStreamBuffer& os) const {
    return true;
}

// Deserialize the message (no additional data needed)
bool NetMsgBlackScreen::fromStream(NetMessengerStreamBuffer& is) {
    return true;
}

// Handle the arrival of the message
void NetMsgBlackScreen::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        glView->showBlackScreen = false; // Remove the black screen in the receiving instance
    }
}

