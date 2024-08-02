#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsgSpawnCar.h"


using namespace Aftr;


NetMsgMacroDefinition(NetMsgSpawnCarPlayers);

NetMsgSpawnCarPlayers::NetMsgSpawnCarPlayers() : NetMsg(), carType("") {}

NetMsgSpawnCarPlayers::~NetMsgSpawnCarPlayers() {}

bool NetMsgSpawnCarPlayers::toStream(NetMessengerStreamBuffer& os) const {
    os << carType;
    return true;
}

bool NetMsgSpawnCarPlayers::fromStream(NetMessengerStreamBuffer& is) {
    is >> carType;
    return true;
}

void NetMsgSpawnCarPlayers::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        if (carType == "Player1") {
            glView->spawnPlayer1();
        }
        else if (carType == "Player2") {
            glView->spawnPlayer2();
        }
    }
}