#pragma once

#include "NetMsg.h"
#include <iostream>
#include "WO.h"
#include "GLViewSpeedRacer.h"
#include "ManagerSerializableNetMsgMap.h"
#include "NetMessengerStreamBuffer.h"
#include "NetMsgMacroMethods.h"
#include "NetMsg_callback_decl.h"
#include <memory>
#include "GLView.h"
#include <string>
#ifdef AFTR_CONFIG_USE_BOOST


namespace Aftr {
class NetMsgMuteState : public NetMsg {
public:
    NetMsgMacroDeclaration(NetMsgMuteState);
    bool isMuted;
    int ownerInstanceId; // The ID of the instance that currently owns the track
    NetMsgMuteState();
    ~NetMsgMuteState() override;

    bool toStream(NetMessengerStreamBuffer& os) const override;
    bool fromStream(NetMessengerStreamBuffer& is) override;
    void onMessageArrived() override;
};
};
#endif