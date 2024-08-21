
#include "Car.h"
#include "Mat4.h"
#include "foundation/PxMat44.h"

using namespace std;
using namespace physx;

Car::~Car()
{
}

Car::Car(const std::string& modelFileName, Aftr::Vector scale, Aftr::MESH_SHADING_TYPE shadingType, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene, Aftr::Mat4 pose)
    : IFace(this), WO() {
    this->Aftr::WO::onCreate(modelFileName, scale, shadingType);
    if (pxPhysics && pxScene) {
        // Create materials
        physx::PxMaterial* gMaterial = pxPhysics->createMaterial(0.5f, 0.5f, 0.6f);
        physx::PxShape* bodyShape = pxPhysics->createShape(physx::PxCapsuleGeometry(2, 4), *gMaterial, true);
        physx::PxTransform bodyTransform = convertPoseToPxTransform(pose);
        pxRigidDynamic = pxPhysics->createRigidDynamic(bodyTransform);
        pxRigidDynamic->attachShape(*bodyShape);
        // Calculate the circumference of the Car's body
        float bodyCircumference = 2 * Aftr::PI * 2;
        // Calculate the leg spacing based on the circumference
        float legSpacing = bodyCircumference / 4;
        // Create capsules for legs
        for (int i = 0; i < 8; ++i) {
            // Calculate the angular displacement for each leg
            float angle = i * Aftr::PI / 4;
            // Calculate the position of the leg relative to the body
            float offsetX = cos(angle) * (2 + legSpacing / 2); // Body radius + half leg spacing
            float offsetY = sin(angle) * (2 + legSpacing / 2);
            // Create leg shape
            physx::PxShape* legShape = pxPhysics->createShape(physx::PxCapsuleGeometry(0.5f, 2.0f), *gMaterial, true);
            // Position each leg relative to the Car's body
            physx::PxTransform legTransform(physx::PxVec3(offsetX, offsetY, -1), physx::PxQuat(angle, physx::PxVec3(0, 0, 1)));
            pxRigidDynamic->attachShape(*legShape);
            legShape->setLocalPose(legTransform);
        }
        pxRigidDynamic->userData = this;
        pxScene->addActor(*pxRigidDynamic);
    }
}

Car* Car::New(const std::string& modelFileName, Aftr::Vector scale, Aftr::MESH_SHADING_TYPE shadingType, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene, Aftr::Mat4 pose)
{
    Car* newCar = new Car(modelFileName, scale, shadingType, pxPhysics, pxScene, pose);
    return newCar;
}

float Car::degToRad(float deg) {
    return deg * Aftr::PI / 180;
}

physx::PxTransform Car::convertPoseToPxTransform(Aftr::Mat4 pose) {
    int k = 0;
    float mat[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mat[k] = pose.at(i, j);
            k++;
        }
    }
    auto t = physx::PxTransform(physx::PxMat44(mat));
    return t;
}

Aftr::Vector* Car::getRelativeRotation()
{
    return &curr_relativeRotationInfo;
}

Aftr::Vector* Car::getGlobalRotation()
{
    return &curr_globalRotationInfo;
}

Aftr::Vector* Car::getPos()
{
    return positionInfo;
}

Aftr::Mat4* Car::getCarPose()
{
    return poseInfo;
}

void Car::setPos(Aftr::Vector pos)
{
    /*   *positionInfo = pos;
       this->setPosition(*positionInfo);
       *poseInfo = this->getPose();*/
    WO::setPosition(pos);
    PxTransform t{ pos.x,pos.y,pos.z };
    pxRigidDynamic->setGlobalPose(t);

}

void Car::setCarPose(Aftr::Mat4 pose)
{
    *poseInfo = pose;
    this->setPose(*poseInfo);
    *positionInfo = this->getPosition();
}

void Car::updatePoseFromPhysicsEngine()
{
    if (this && pxRigidDynamic) {
        if (pxRigidDynamic->isSleeping()) {
            pxRigidDynamic->wakeUp();
        }
        physx::PxTransform pose = pxRigidDynamic->getGlobalPose();
        physx::PxMat44 consPosition(pose);
        Aftr::Mat4 updatedPose;

        for (int i = 0; i < 16; i++) {
            updatedPose[i] = consPosition(i % 4, i / 4);
        }
        this->setCarPose(updatedPose);
    }
}

void Car::onUpdateWO()
{
    WO::onUpdateWO();

    // translation
    this->setPosition(*positionInfo);

    // relative rotations 
    if (prev_relativeRotationInfo != curr_relativeRotationInfo) {
        this->rotateAboutRelX(degToRad(curr_relativeRotationInfo.x - prev_relativeRotationInfo.x));
        this->rotateAboutRelY(degToRad(curr_relativeRotationInfo.y - prev_relativeRotationInfo.y));
        this->rotateAboutRelZ(degToRad(curr_relativeRotationInfo.z - prev_relativeRotationInfo.z));

        *poseInfo = this->getPose();

        prev_relativeRotationInfo = curr_relativeRotationInfo;
    }

    // global rotations
    if (prev_globalRotationInfo != curr_globalRotationInfo) {
        this->rotateAboutGlobalX(degToRad(curr_globalRotationInfo.x - prev_globalRotationInfo.x));
        this->rotateAboutGlobalY(degToRad(curr_globalRotationInfo.y - prev_globalRotationInfo.y));
        this->rotateAboutGlobalZ(degToRad(curr_globalRotationInfo.z - prev_globalRotationInfo.z));

        *poseInfo = this->getPose();

        prev_globalRotationInfo = curr_globalRotationInfo;
    }
    // Detect collision by checking for a sudden stop reduction in speed
    float currentSpeed = pxRigidDynamic->getLinearVelocity().magnitude();
    if (previousVelocity - currentSpeed > 10.0f) // Adjust the threshold as needed
    {
        if (CrashSound) {
            CrashSound->play2D("../../../modules/SpeedRacer/mm/sounds/crash.wav", false); // Play the crash sound once
        }
    }
    previousVelocity = currentSpeed; // Update the previous velocity
}

void Car::rotateCar(float angle, const physx::PxVec3& axis)
{
    if (pxRigidDynamic)
    {
        // Get the current global pose
        physx::PxTransform currentPose = pxRigidDynamic->getGlobalPose();

        // Create a quaternion representing the rotation
        physx::PxQuat rotationQuat(angle, axis);

        // Apply the rotation to the current orientation
        currentPose.q = rotationQuat * currentPose.q;

        // Set the new global pose
        pxRigidDynamic->setGlobalPose(currentPose);
    }
}
physx::PxVec3 Car::getForwardVector() const {
    if (pxRigidDynamic) {
        // The forward vector in local space is typically along the +Z axis
        // but it depends on your specific model's orientation. Here, we assume it is along +Z.
        physx::PxVec3 localForward(0, 1, 0); // Change this if your model's forward direction is different
        physx::PxQuat orientation = pxRigidDynamic->getGlobalPose().q;
        return orientation.rotate(localForward);
    }
    return physx::PxVec3(0, 0, 0);
}
// Implementation of setSpeed
// Implementation of setSpeed
void Car::setSpeed(float speed) {
    this->currentSpeed = speed;
    if (this->pxRigidDynamic) {
        physx::PxVec3 velocity = this->pxRigidDynamic->getLinearVelocity();
        velocity.normalize(); // Keep the direction the same
        this->pxRigidDynamic->setLinearVelocity(velocity * speed);
    }
}

void Car::setPhysicsQuality(float quality) {
    // Adjust physics properties based on the quality value
    // For example, change friction, mass, or damping
    float adjustedFriction = 1.0f - (quality * 0.5f); // Adjust friction based on quality
    float adjustedMass = 100.0f * quality; // Example adjustment for mass

    // Apply these adjustments to the car's physics properties
    physx::PxRigidDynamic* rigidBody = getRigidDynamic();
    if (rigidBody) {
        rigidBody->setMass(adjustedMass);
        // Apply other physics adjustments as needed
    }
}