#pragma once
#include <WO.h>
#include <string>
#include <memory>
#include "PxPhysicsAPI.h"
#include "ManagerOpenGLState.h" 

class Car : public Aftr::WO {
public:
    virtual ~Car();

    static Car* New(const std::string& modelFileName, Aftr::Vector scale, Aftr::MESH_SHADING_TYPE shadingType, physx::PxPhysics* pxPhysics = nullptr, physx::PxScene* pxScene = nullptr, Aftr::Mat4 pose = Aftr::Mat4());

    float degToRad(float deg);
    physx::PxTransform convertPoseToPxTransform(Aftr::Mat4 pose);

    Aftr::Vector* getPos();
    void setPos(Aftr::Vector pos);
    Aftr::Mat4* getCarPose();
    void setCarPose(Aftr::Mat4 pose);
    Aftr::Vector* getRelativeRotation();
    Aftr::Vector* getGlobalRotation();
    void updatePoseFromPhysicsEngine();
    void rotateCar(float angle, const physx::PxVec3& axis);
    physx::PxVec3 getForwardVector() const;
    virtual void onUpdateWO() override;
    physx::PxRigidDynamic* getRigidDynamic() const { return pxRigidDynamic; }
    physx::PxRigidDynamic* pxRigidDynamic = nullptr;
    void setSpeed(float speed);
protected:
    Aftr::Vector p = Aftr::Vector(10, 15, 4);
    Aftr::Vector* positionInfo = &p;
    Aftr::Mat4 p2 = Aftr::Mat4();
    Aftr::Mat4* poseInfo = &p2;
    Aftr::Vector curr_relativeRotationInfo, prev_relativeRotationInfo;
    Aftr::Vector curr_globalRotationInfo, prev_globalRotationInfo;
    //physx::PxRigidDynamic* pxRigidDynamic = nullptr;
    float currentSpeed;

    Car(const std::string& modelFileName, Aftr::Vector scale, Aftr::MESH_SHADING_TYPE shadingType, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene, Aftr::Mat4 pose);

};