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
    class NetMsgTerrainLoaded : public Aftr::NetMsg {
    public:
        NetMsgMacroDeclaration(NetMsgTerrainLoaded);

        NetMsgTerrainLoaded() = default;
        virtual ~NetMsgTerrainLoaded() = default;

        virtual bool toStream(NetMessengerStreamBuffer& os) const override {
            // No additional data to serialize
            return true;
        }

        virtual bool fromStream(NetMessengerStreamBuffer& is) override {
            // No additional data to deserialize
            return true;
        }

        virtual void onMessageArrived() override {
            GLViewSpeedRacer* glView = static_cast<GLViewSpeedRacer*>(ManagerGLView::getGLView());

            // Mark that the other instance has loaded the terrain
            glView->otherInstanceTerrainLoaded = true;
        }

        virtual std::string toString() const override {
            return "NetMsgTerrainLoaded";
        }
    };

};
#endif
