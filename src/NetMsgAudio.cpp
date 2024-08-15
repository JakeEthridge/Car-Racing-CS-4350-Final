#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsgAudio.h"


using namespace Aftr;

NetMsgMacroDefinition(NetMsgTimerControl);
 
NetMsgTimerControl::NetMsgTimerControl()
    : NetMsg(), isTimerRunning(false) {}

NetMsgTimerControl::~NetMsgTimerControl() {}

bool NetMsgTimerControl::toStream(NetMessengerStreamBuffer& os) const {
    // Convert bool to int for streaming
    os << (isTimerRunning ? 1 : 0);
    return true;
}


bool NetMsgTimerControl::fromStream(NetMessengerStreamBuffer& is) {
    int temp;
    // Read as int and then convert to bool
    is >> temp;
    isTimerRunning = (temp != 0);
    return true;
}

void NetMsgTimerControl::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        // Update the timer state in the receiving instance
        if (isTimerRunning) {
            glView->timerStartTime = SDL_GetTicks();
            glView->isTimerRunning = true;
            glView->pausedTime = 0;
        }
        else {
            glView->pausedTime = SDL_GetTicks() - glView->timerStartTime;
            glView->isTimerRunning = false;
        }
    }
}

