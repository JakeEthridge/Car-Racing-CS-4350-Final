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
    class NetMsgTimerControl : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgTimerControl);

        NetMsgTimerControl();
        virtual ~NetMsgTimerControl();

        // Define message data
        bool isTimerRunning;

        // Serialization methods
        virtual bool toStream(NetMessengerStreamBuffer& os) const override;
        virtual bool fromStream(NetMessengerStreamBuffer& is) override;

        // Message handling
        virtual void onMessageArrived() override;
    };

};
#endif
