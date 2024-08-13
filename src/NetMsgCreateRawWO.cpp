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

NetMsgMacroDefinition(NetMsgBlackScreen);

NetMsgBlackScreen::NetMsgBlackScreen() : NetMsg() {}

NetMsgBlackScreen::~NetMsgBlackScreen() {}

bool NetMsgBlackScreen::toStream(NetMessengerStreamBuffer& os) const {
    return true; // No additional data for this message
}

bool NetMsgBlackScreen::fromStream(NetMessengerStreamBuffer& is) {
    return true; // No additional data for this message
}

void NetMsgBlackScreen::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        glView->showBlackScreen = false; // Remove the black screen
    }
}

