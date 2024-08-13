#pragma once

#include "NetMsg.h"
#include "GLViewSpeedRacer.h"
#include "ManagerGLView.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsgAudio.h"


using namespace Aftr;


// Implementation of methods
NetMsgMuteState::NetMsgMuteState() : NetMsg(), isMuted(false) {}

NetMsgMuteState::~NetMsgMuteState() {}

bool NetMsgMuteState::fromStream(NetMessengerStreamBuffer& is) {

    return true;
}


bool NetMsgMuteState::toStream(NetMessengerStreamBuffer& os) const {
    
    return true;
}


void NetMsgMuteState::onMessageArrived() {
    GLViewSpeedRacer* glView = dynamic_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());
    if (glView) {
        glView->soundEngine->stopAllSounds();
        std::string startMenuSoundPath = ManagerEnvironmentConfiguration::getLMM() + "/sounds/StartMenu.wav";
        glView->soundEngine->play2D(startMenuSoundPath.c_str(), true, false, true);
    }
}
