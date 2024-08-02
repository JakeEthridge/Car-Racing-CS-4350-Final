#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsgCarVisibility.h"

using namespace Aftr;

NetMsgMacroDefinition(NetMsgCarVisibility);

NetMsgCarVisibility::NetMsgCarVisibility() : NetMsg(), carName("") {}

NetMsgCarVisibility::~NetMsgCarVisibility() {}

bool NetMsgCarVisibility::toStream(NetMessengerStreamBuffer& os) const {
    os << carName;
    return true;
}

bool NetMsgCarVisibility::fromStream(NetMessengerStreamBuffer& is) {
    is >> carName;
    return true;
}

void NetMsgCarVisibility::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        if (carName == "car_test") {
            glView->car_test->isVisible = true;
            glView->car_turn->isVisible = false;
            glView->car_other_side->isVisible = false;
            glView->car_new->isVisible = false;
        }
        else if (carName == "car_turn") {
            glView->car_test->isVisible = false;
            glView->car_turn->isVisible = true;
            glView->car_other_side->isVisible = false;
            glView->car_new->isVisible = false;
        }
        else if (carName == "car_other_side") {
            glView->car_test->isVisible = false;
            glView->car_turn->isVisible = false;
            glView->car_other_side->isVisible = true;
            glView->car_new->isVisible = false;
        }
        else if (carName == "car_new") {
            glView->car_test->isVisible = false;
            glView->car_turn->isVisible = false;
            glView->car_other_side->isVisible = false;
            glView->car_new->isVisible = true;
        }
    }
    if (carName == "carMain") {
        glView->carMain->isVisible = true;
        glView->carRight->isVisible = false;
        glView->carLeft->isVisible = false;
        glView->carDown->isVisible = false;
    }
    else if (carName == "carRight") {
        glView->carMain->isVisible = false;
        glView->carRight->isVisible = true;
        glView->carLeft->isVisible = false;
        glView->carDown->isVisible = false;
    }
    else if (carName == "carLeft") {
        glView->carMain->isVisible = false;
        glView->carRight->isVisible = false;
        glView->carLeft->isVisible = true;
        glView->carDown->isVisible = false;
    }
    else if (carName == "carDown") {
        glView->carMain->isVisible = false;
        glView->carRight->isVisible = false;
        glView->carLeft->isVisible = false;
        glView->carDown->isVisible = true;
    }
}
