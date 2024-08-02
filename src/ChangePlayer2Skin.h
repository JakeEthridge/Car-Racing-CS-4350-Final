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
    class NetMsgChangeCarSkin : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgChangeCarSkin);

        NetMsgChangeCarSkin();
        virtual ~NetMsgChangeCarSkin();
        virtual bool toStream(NetMessengerStreamBuffer& os) const;
        virtual bool fromStream(NetMessengerStreamBuffer& is);
        virtual void onMessageArrived();

        std::string skinType;
        std::string player;
    };
};
#endif
