//#pragma once
//
//#include "NetMsg.h"
//#include "GLViewSpeedRacer.h"
//#include "ManagerGLView.h"
//#include "NetMessengerStreamBuffer.h"
//#include "NetMsgMacroMethods.h"
//#include "NetMsgAudio.h"
//
//
//using namespace Aftr;
//
//
//// Implementation of methods
//NetMsgTimeTrial::NetMsgTimeTrial() : NetMsg(), action(START), resetTime(30.0f) {}
//
//NetMsgTimeTrial::~NetMsgTimeTrial() {}
//
//bool NetMsgTimeTrial::fromStream(NetMessengerStreamBuffer& is) {
//    int actionInt;
//    is >> actionInt;
//    action = static_cast<Action>(actionInt);
//    if (action == RESET) {
//        is >> resetTime;
//    }
//    return true;
//}
//
//bool NetMsgTimeTrial::toStream(NetMessengerStreamBuffer& os) const {
//    os << static_cast<int>(action);
//    if (action == RESET) {
//        os << resetTime;
//    }
//    return true;
//}
//
//
//void NetMsgTimeTrial::onMessageArrived() {
//    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
//    if (glView) {
//        switch (action) {
//        case START:
//            glView->startTimer();
//            break;
//        case PAUSE:
//            glView->pauseTimer();
//            break;
//        case RESET:
//            glView->resetTimer(resetTime);
//            break;
//        }
//    }
//}
//
