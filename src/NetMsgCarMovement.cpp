#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsgCarMovement.h"


using namespace Aftr;


NetMsgMacroDefinition(NetMsgCarMovementCar2);

NetMsgCarMovementCar2::NetMsgCarMovementCar2()
    : NetMsg(), carMainPosition(0.0f, 0.0f, 0.0f), carRightPosition(0.0f, 0.0f, 0.0f),
    carLeftPosition(0.0f, 0.0f, 0.0f), carDownPosition(0.0f, 0.0f, 0.0f) {}

NetMsgCarMovementCar2::~NetMsgCarMovementCar2() {}

bool NetMsgCarMovementCar2::toStream(NetMessengerStreamBuffer& os) const {
    os << carMainPosition.x << carMainPosition.y << carMainPosition.z;
    os << carRightPosition.x << carRightPosition.y << carRightPosition.z;
    os << carLeftPosition.x << carLeftPosition.y << carLeftPosition.z;
    os << carDownPosition.x << carDownPosition.y << carDownPosition.z;
    return true;
}

bool NetMsgCarMovementCar2::fromStream(NetMessengerStreamBuffer& is) {
    is >> carMainPosition.x >> carMainPosition.y >> carMainPosition.z;
    is >> carRightPosition.x >> carRightPosition.y >> carRightPosition.z;
    is >> carLeftPosition.x >> carLeftPosition.y >> carLeftPosition.z;
    is >> carDownPosition.x >> carDownPosition.y >> carDownPosition.z;
    return true;
}

void NetMsgCarMovementCar2::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        if (glView->carMain) {
            glView->carMain->setPos(carMainPosition);
        }
        if (glView->carRight) {
            glView->carRight->setPos(carRightPosition);
        }
        if (glView->carLeft) {
            glView->carLeft->setPos(carLeftPosition);
        }
        if (glView->carDown) {
            glView->carDown->setPos(carDownPosition);
        }
    }
}
