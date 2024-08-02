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
class NetMsgCarMovement : public NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgCarMovement);

        NetMsgCarMovement();
        virtual ~NetMsgCarMovement();
        virtual bool toStream(NetMessengerStreamBuffer& os) const override;
        virtual bool fromStream(NetMessengerStreamBuffer& is) override;
        virtual void onMessageArrived() override;

        Vector car_testPosition; // Position of car_test to be synchronized
        Vector car_turnPosition; // Position of car_turn to be synchronized
        Vector car_other_sidePosition; // Position of car_other_side to be synchronized
        Vector car_newPosition; // Position of car_new to be synchronized



    };
};
#endif

