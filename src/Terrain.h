#pragma once 

#include "PxPhysicsAPI.h"
#include "Model.h"
#include "WOGridECEFElevation.h"

namespace Aftr {
    class WOGridECEFElevationPhysX : public WOGridECEFElevation {
    public:
        WOGridECEFElevationPhysX();
        virtual ~WOGridECEFElevationPhysX();

        static WOGridECEFElevationPhysX* New(physx::PxPhysics* pxPhysics, physx::PxScene* pxScene, Aftr::VectorD upperLeft, Aftr::VectorD lowerRight, Aftr::VectorD offset, Aftr::VectorD scale, const std::string& elevationPath, const std::string& texturePath);

        virtual void onCreate(physx::PxPhysics* pxPhysics, physx::PxScene* pxScene, Aftr::VectorD upperLeft, Aftr::VectorD lowerRight, Aftr::VectorD offset, Aftr::VectorD scale, const std::string& elevationPath, const std::string& texturePath);

        physx::PxRigidStatic* actor = nullptr;
        

    private:
        float* vertexListCopy = nullptr;
        unsigned int* indiciesCopy = nullptr;
    };
}