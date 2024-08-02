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
#include <new.h>
#include <string>
#ifdef AFTR_CONFIG_USE_BOOST

namespace Aftr {
    class NetMsgSwitchTerrain : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgSwitchTerrain);

        NetMsgSwitchTerrain();
        virtual ~NetMsgSwitchTerrain();
        virtual bool toStream(NetMessengerStreamBuffer& os) const override;
        virtual bool fromStream(NetMessengerStreamBuffer& is) override;
        virtual void onMessageArrived() override;

        bool useAnotherGrid;
       
        float moveDownAmount;
        float rotateAmount;
        float moveNegativeXAmount;
    };

};
#endif
