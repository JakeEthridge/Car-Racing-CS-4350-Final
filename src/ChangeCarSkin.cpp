#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "ChangeCarSkin.h"


using namespace Aftr;

NetMsgMacroDefinition(NetMsgStartToLoading);

NetMsgStartToLoading::NetMsgStartToLoading() : NetMsg() {}

NetMsgStartToLoading::~NetMsgStartToLoading() {}

bool NetMsgStartToLoading::toStream(NetMessengerStreamBuffer& os) const {
    // You can add any data you need to send here
    return true;
}

bool NetMsgStartToLoading::fromStream(NetMessengerStreamBuffer& is) {
    // You can receive any data here if needed
    return true;
}

void NetMsgStartToLoading::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        // Implement what should happen when the message arrives
        // For example, this could start the loading process on another instance
        glView->startLoadingProcess(); // Placeholder function, implement it as needed
    }
}