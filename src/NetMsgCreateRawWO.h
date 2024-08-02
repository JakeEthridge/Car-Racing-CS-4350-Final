//#pragma once
//
//#include "NetMsg.h"
//#include "ManagerSerializableNetMsgMap.h"
//#include "NetMessengerStreamBuffer.h"
//#include "NetMsgMacroMethods.h"
//#include "GLViewSpeedracer.h"
//#include "NetMsg_callback_decl.h"
//#include <memory>
//
//#ifdef AFTR_CONFIG_USE_BOOST
//namespace Aftr {
//
//    class NetMsgCreateRawWO : public NetMsg {
//    public:
//        NetMsgMacroDeclaration(NetMsgCreateRawWO);
//
//        NetMsgCreateRawWO();
//        virtual ~NetMsgCreateRawWO();
//        virtual bool toStream(NetMessengerStreamBuffer& os) const;
//        virtual bool fromStream(NetMessengerStreamBuffer& is);
//        virtual void onMessageArrived();
//        virtual std::string toString() const;
//        float rotationX, rotationY, rotationZ; // New rotation fields
//        // Use consistent data types
//        double xPos;
//        double yPos;
//        double zPos;
//        float rotationAngle;
//        double xRot, yRot, zRot; // Rotation variables
//        double x_current = 0;
//        double y_current = 0;
//        double z_current = 0;
//
//        double last_x = 0;
//        double last_y = 0;
//        double last_z = 0;
//
//
//    protected:
//
//    };
//
//
//} // namespace Aftr
//#endif
