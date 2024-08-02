#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsgCar.h"


using namespace Aftr;


NetMsgMacroDefinition(NetMsgCarMovement);

NetMsgCarMovement::NetMsgCarMovement()
    : NetMsg(), car_testPosition(0.0f, 0.0f, 0.0f), car_turnPosition(0.0f, 0.0f, 0.0f),
    car_other_sidePosition(0.0f, 0.0f, 0.0f), car_newPosition(0.0f, 0.0f, 0.0f) {}

NetMsgCarMovement::~NetMsgCarMovement() {}

bool NetMsgCarMovement::toStream(NetMessengerStreamBuffer& os) const {
    os << car_testPosition.x << car_testPosition.y << car_testPosition.z;
    os << car_turnPosition.x << car_turnPosition.y << car_turnPosition.z;
    os << car_other_sidePosition.x << car_other_sidePosition.y << car_other_sidePosition.z;
    os << car_newPosition.x << car_newPosition.y << car_newPosition.z;

    return true;
}

bool NetMsgCarMovement::fromStream(NetMessengerStreamBuffer& is) {
    is >> car_testPosition.x >> car_testPosition.y >> car_testPosition.z;
    is >> car_turnPosition.x >> car_turnPosition.y >> car_turnPosition.z;
    is >> car_other_sidePosition.x >> car_other_sidePosition.y >> car_other_sidePosition.z;
    is >> car_newPosition.x >> car_newPosition.y >> car_newPosition.z;

    return true;
}

void NetMsgCarMovement::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        if (glView->car_test) {
            glView->car_test->setPos(car_testPosition);
        }
        if (glView->car_turn) {
            glView->car_turn->setPos(car_turnPosition);
        }
        if (glView->car_other_side) {
            glView->car_other_side->setPos(car_other_sidePosition);
        }
        if (glView->car_new) {
            glView->car_new->setPos(car_newPosition);
        }
    }
}
