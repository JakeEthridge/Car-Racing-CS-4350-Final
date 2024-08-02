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
    class NetMsgSpawnCarPlayers : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgSpawnCarPlayers);

        NetMsgSpawnCarPlayers();
        virtual ~NetMsgSpawnCarPlayers();

        virtual bool toStream(NetMessengerStreamBuffer& os) const override;
        virtual bool fromStream(NetMessengerStreamBuffer& is) override;
        virtual void onMessageArrived() override;

        std::string carType; // "Player1" or "Player2"
    };
};
#endif

