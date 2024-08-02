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
    class NetMsgCarMovementCar2 : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgCarMovementCar2);

        NetMsgCarMovementCar2();
        virtual ~NetMsgCarMovementCar2();

        virtual bool toStream(NetMessengerStreamBuffer& os) const override;
        virtual bool fromStream(NetMessengerStreamBuffer& is) override;
        virtual void onMessageArrived() override;

        Vector carMainPosition;
        Vector carRightPosition;
        Vector carLeftPosition;
        Vector carDownPosition;
    };
};
#endif

