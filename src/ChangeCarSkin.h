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
    class NetMsgSwitchCarSkin : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgSwitchCarSkin);

        NetMsgSwitchCarSkin();
        virtual ~NetMsgSwitchCarSkin();

        virtual bool toStream(NetMessengerStreamBuffer& os) const override;
        virtual bool fromStream(NetMessengerStreamBuffer& is) override;
        virtual void onMessageArrived() override;

        std::string skinType; // "Dodge" or "Ford"
    };
};
#endif
