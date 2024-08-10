#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "ChangePlayer2Skin.h"


using namespace Aftr;


NetMsgMacroDefinition(NetMsgChangeCarSkin);

NetMsgChangeCarSkin::NetMsgChangeCarSkin() : NetMsg(), skinType(""), player("") {}

NetMsgChangeCarSkin::~NetMsgChangeCarSkin() {}

bool NetMsgChangeCarSkin::toStream(NetMessengerStreamBuffer& os) const {
    os << skinType << player;
    return true;
}

bool NetMsgChangeCarSkin::fromStream(NetMessengerStreamBuffer& is) {
    is >> skinType >> player;
    return true;
}

void NetMsgChangeCarSkin::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        if (player == "Player1") {
            if (skinType == "Dodge") {
                glView->hideAllCars();
                glView->spawnPlayer1();
            }
            else if (skinType == "Ford") {
                glView->hideAllCars();
                glView->OtherCarSkin1();
            }
            else if (skinType == "Sports Car") {
                glView->hideAllCars();
                glView->OtherCarSkin2();
            }
            else if (skinType == "CyberTrunk") {
                glView->hideAllCars();
                glView->OtherCarSkin3();
            }
        }
        else if (player == "Player2") {
            if (skinType == "Dodge") {
                glView->hideAllCars2();
                glView->spawnPlayer2();
            }
            else if (skinType == "Ford") {
                glView->hideAllCars2();
                glView->spawnPlayer2Skin1();
            }
            else if (skinType == "Sports Car") {
                glView->hideAllCars2();
                glView->spawnPlayer2Skin2();
            }
            else if (skinType == "CyberTrunk") {
                glView->hideAllCars2();
                glView->spawnPlayer2Skin3();
            }
        }
    }
}