#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "ChangeCarSkin.h"


using namespace Aftr;

NetMsgMacroDefinition(NetMsgSwitchCarSkin);

NetMsgSwitchCarSkin::NetMsgSwitchCarSkin() : NetMsg(), skinType("") {}

NetMsgSwitchCarSkin::~NetMsgSwitchCarSkin() {}

bool NetMsgSwitchCarSkin::toStream(NetMessengerStreamBuffer& os) const {
    os << skinType;
    return true;
}

bool NetMsgSwitchCarSkin::fromStream(NetMessengerStreamBuffer& is) {
    is >> skinType;
    return true;
}

void NetMsgSwitchCarSkin::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        if (skinType == "Dodge") {
            glView->spawnPlayer1(); // Function for Dodge cars
        }
        else if (skinType == "Ford") {
            glView->OtherCarSkin1(); // Function for Ford cars
        }
        else if (skinType == "Sports Car") {
            glView->OtherCarSkin2(); // Function for Sports cars
        }
        else if (skinType == "CyberTrunk") {
            glView->OtherCarSkin3(); // Function for Audi R8 cars
        }
    }
}