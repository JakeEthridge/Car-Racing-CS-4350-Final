#include "GLViewSpeedRacer.h"
#include "irrKlang.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "stb/stb_image.h"
#include "NetMsgCreateWO.h"
#include <NetMsg.h>
#include <new.h>
#include "NetMessengerStreamBuffer.h"
#include <NetMessengerClient.h>
#include "NetMsgCreateRawWO.h"
#include "GLView.h"
#include "Terrain.h"
#include <NetMsgTerrainWO.h>
#include "aftrUtilities.h"
#include <NetMsgCar.h>
#include <NetMsgCarVisibility.h>
#include <NetMsgSpawnCar.h>
#include <NetMsgCarMovement.h>
#include <ChangePlayer2Skin.h>
#include <ChangeCarSkin.h>

using namespace Aftr;

using namespace irrklang;

GLViewSpeedRacer* GLViewSpeedRacer::New(const std::vector< std::string >& args, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene)
{
    GLViewSpeedRacer* glv = new GLViewSpeedRacer(args, pxPhysics, pxScene);
    glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
    glv->onCreate();
    return glv;
}

GLViewSpeedRacer::GLViewSpeedRacer(const std::vector< std::string >& args, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene) : GLView(args)
{
    //Initialize any member variables that need to be used inside of LoadMap() here.
    //Note: At this point, the Managers are not yet initialized. The Engine initialization
    //occurs immediately after this method returns (see GLViewATestPhysX::New() for
    //reference). Then the engine invoke's GLView::loadMap() for this module.
    //After loadMap() returns, GLView::onCreate is finally invoked.

    //The order of execution of a module startup:
    //GLView::New() is invoked:
    //    calls GLView::init()
    //       calls GLView::loadMap() (as well as initializing the engine's Managers)
    //    calls GLView::onCreate()

    //GLViewATestPhysX::onCreate() is invoked after this module's LoadMap() is completed.
    this->pxPhysics = pxPhysics;
    this->pxScene = pxScene;
}

void GLViewSpeedRacer::onCreate()
{
    //GLViewATestPhysX::onCreate() is invoked after this module's LoadMap() is completed.
    //At this point, all the managers are initialized. That is, the engine is fully initialized.

    if (this->pe != NULL)
    {
        //optionally, change gravity direction and magnitude here
        //The user could load these values from the module's aftr.conf
        this->pe->setGravityNormalizedVector(Vector(0, 0, -1.0f));
        this->pe->setGravityScalar(Aftr::GRAVITY);
    }
    this->setActorChaseType(STANDARDEZNAV); //Default is STANDARDEZNAV mode
    //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1



}


GLViewSpeedRacer::~GLViewSpeedRacer()
{
    //Implicitly calls GLView::~GLView()
}
void GLViewSpeedRacer::updateWorld() {
    GLView::updateWorld();
    Uint32 currentTime = SDL_GetTicks();

    pxScene->simulate(0.03);

    physx::PxU32 errorState = 0;
    pxScene->fetchResults(true);
    {
        physx::PxU32 numDynamicActors = pxScene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
        physx::PxActor** dynamicActors = new physx::PxActor * [numDynamicActors];
        auto tmp = pxScene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC, dynamicActors, numDynamicActors);

        for (physx::PxU32 i = 0; i < numDynamicActors; i++) {
            physx::PxActor* actor = dynamicActors[i];
            Car* carWO = static_cast<Car*>(actor->userData);
            carWO->updatePoseFromPhysicsEngine();
        }

        Car* visibleCar = followCar1 ? getVisibleCar1() : getVisibleCar2();
        if (visibleCar) {
            physx::PxTransform carTransform = visibleCar->getRigidDynamic()->getGlobalPose();
            physx::PxVec3 carPosition = carTransform.p;

            // Set the camera offset based on which car is visible
            physx::PxVec3 cameraOffset(0, -20, 7);

            if (visibleCar == car_turn || visibleCar == carRight) {
                cameraOffset = physx::PxVec3(0, 40, 7);
            }
            else if (visibleCar == car_other_side || visibleCar == carLeft) {
                cameraOffset = physx::PxVec3(-40, 0, 7);
            }
            else if (visibleCar == car_test || visibleCar == carMain) {
                cameraOffset = physx::PxVec3(40, 0, 5);
            }
            else if (visibleCar == car_new || visibleCar == carDown) {
                cameraOffset = physx::PxVec3(0, -40, 7);
            }
            else {
                // Default camera offset if no car matches
                cameraOffset = physx::PxVec3(0, 0, 7); // or whatever default you want
            }

            physx::PxVec3 desiredCameraPosition = carPosition + cameraOffset;

            Aftr::Vector currentCameraPosition = this->getCamera()->getPosition();
            Aftr::Vector newCameraPosition = Aftr::Vector(
                lerp(currentCameraPosition.x, desiredCameraPosition.x, 0.1f),
                lerp(currentCameraPosition.y, desiredCameraPosition.y, 0.1f),
                lerp(currentCameraPosition.z, desiredCameraPosition.z, 0.1f)
            );

            this->getCamera()->setPosition(newCameraPosition);

            Aftr::Vector currentLookAtPoint = this->getCamera()->getCameraLookAtPoint();
            Aftr::Vector newLookAtPoint = Aftr::Vector(
                lerp(currentLookAtPoint.x, carPosition.x, 0.1f),
                lerp(currentLookAtPoint.y, carPosition.y, 0.1f),
                lerp(currentLookAtPoint.z, carPosition.z, 0.1f)
            );

            this->getCamera()->setCameraLookAtPoint(newLookAtPoint);
        }

        delete[] dynamicActors;
    }
    updateControls();
    auto timeNow = std::chrono::high_resolution_clock::now();
    auto retryAfter = std::chrono::duration_cast<std::chrono::seconds>(timeNow - tcpRetry).count();
    if (!client->isTCPSocketOpen() && retryAfter > 15) {
        tcpRetry = timeNow;
        this->client = NetMessengerClient::New("127.0.0.1", ManagerEnvironmentConfiguration::getVariableValue("NetServerTransmitPort"));
    }
}


void GLViewSpeedRacer::onResizeWindow(GLsizei width, GLsizei height)
{
    GLView::onResizeWindow(width, height); //call parent's resize method.
}


void GLViewSpeedRacer::onMouseDown(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseDown(e);
}


void GLViewSpeedRacer::onMouseUp(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseUp(e);
}


void GLViewSpeedRacer::onMouseMove(const SDL_MouseMotionEvent& e)
{
    GLView::onMouseMove(e);
}

int GLViewSpeedRacer::lapNumber = 1;
int GLViewSpeedRacer::leftKeyPressCount = 0;
Uint32 GLViewSpeedRacer::secondLeftKeyPressTime = 0;
bool GLViewSpeedRacer::updateLapPending = false;

bool GLViewSpeedRacer::isTimerRunning = false;
Uint32 GLViewSpeedRacer::timerStartTime = 0;
int GLViewSpeedRacer::resetTime = 10;
Uint32 GLViewSpeedRacer::pausedTime = 0;


void GLViewSpeedRacer::onKeyDown(const SDL_KeyboardEvent& key) {
    GLView::onKeyDown(key);
    Car* currentCar = nullptr;

    // Determine the currently active car model
    switch (selectedCarModel) {
    case CAR_MODEL_DODGE:
        currentCar = carDodge;
        break;
    case CAR_MODEL_FORD:
        currentCar = carFord;
        break;
        // Add cases for additional models if needed
    }

    // Handle car visibility and position based on key presses
    if (key.keysym.sym == SDLK_DOWN) {
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        car_test->isVisible = true;
        car_turn->isVisible = false;
        car_other_side->isVisible = false;
        car_new->isVisible = false;

        physx::PxVec3 rightVector = car_test->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
        physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount * -1.0f;

        car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovement msg;
            msg.car_testPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.car_turnPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.car_other_sidePosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.car_newPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "car_test";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    if (key.keysym.sym == SDLK_UP) {
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        car_test->isVisible = false;
        car_turn->isVisible = false;
        car_other_side->isVisible = true;
        car_new->isVisible = false;

        physx::PxVec3 rightVector = car_test->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
        physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount;
        car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (this->client) {
            NetMsgCarMovement msg;
            msg.car_other_sidePosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.car_turnPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.car_testPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.car_newPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            this->client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "car_other_side";
            this->client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    if (key.keysym.sym == SDLK_LEFT) {
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        car_test->isVisible = false;
        car_turn->isVisible = false;
        car_other_side->isVisible = false;
        car_new->isVisible = true;

        physx::PxVec3 forward = car_test->getForwardVector();
        physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + forward * moveAmount;
        car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovement msg;
            msg.car_testPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.car_turnPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.car_other_sidePosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.car_newPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "car_new";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    if (key.keysym.sym == SDLK_RIGHT) {
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        car_test->isVisible = false;
        car_turn->isVisible = true;
        car_other_side->isVisible = false;
        car_new->isVisible = false;

        physx::PxVec3 forward = car_test->getForwardVector();
        physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + forward * moveAmount * -1.0f;
        car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovement msg;
            msg.car_testPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.car_turnPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.car_other_sidePosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.car_newPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "car_turn";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    // Handle car visibility and position based on key presses
    if (key.keysym.sym == SDLK_s) { // Down arrow key
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        carMain->isVisible = true;
        carRight->isVisible = false;
        carLeft->isVisible = false;
        carDown->isVisible = false;

        physx::PxVec3 rightVector = carMain->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
        physx::PxVec3 newPos = carMain->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount * -1.0f;
        carRight->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        carMain->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        carLeft->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        carDown->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovementCar2 msg;
            msg.carMainPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.carRightPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.carLeftPosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.carDownPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "carMain";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    if (key.keysym.sym == SDLK_w) { // Up arrow key
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        carMain->isVisible = false;
        carRight->isVisible = false;
        carLeft->isVisible = true;
        carDown->isVisible = false;

        physx::PxVec3 rightVector = carMain->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
        physx::PxVec3 newPos = carMain->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount;
        carRight->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        carMain->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        carLeft->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        carDown->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovementCar2 msg;
            msg.carMainPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.carRightPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.carLeftPosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.carDownPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "carLeft";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    if (key.keysym.sym == SDLK_a) { // Left arrow key
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        carMain->isVisible = false;
        carRight->isVisible = false;
        carLeft->isVisible = false;
        carDown->isVisible = true;

        physx::PxVec3 forward = carMain->getForwardVector();
        physx::PxVec3 newPos = carMain->getRigidDynamic()->getGlobalPose().p + forward * moveAmount;
        carRight->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        carMain->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        carLeft->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        carDown->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovementCar2 msg;
            msg.carMainPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.carRightPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.carLeftPosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.carDownPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "carDown";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }

    if (key.keysym.sym == SDLK_d) { // Right arrow key
        drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        carMain->isVisible = false;
        carRight->isVisible = true;
        carLeft->isVisible = false;
        carDown->isVisible = false;

        physx::PxVec3 forward = carMain->getForwardVector();
        physx::PxVec3 newPos = carMain->getRigidDynamic()->getGlobalPose().p + forward * moveAmount * -1.0f;
        carRight->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        carMain->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        carLeft->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        carDown->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

        // Send the updated positions and visibility to the other instance
        if (client) {
            NetMsgCarMovementCar2 msg;
            msg.carMainPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
            msg.carRightPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
            msg.carLeftPosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
            msg.carDownPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
            client->sendNetMsgSynchronousTCP(msg);

            NetMsgCarVisibility visibilityMsg;
            visibilityMsg.carName = "carRight";
            client->sendNetMsgSynchronousTCP(visibilityMsg);
        }
    }
    if (key.keysym.sym == SDLK_1)
    {
        active_keys[SDLK_1] = true;
    }
    if (key.keysym.sym == SDLK_4)
    {
        active_keys[SDLK_4] = true;
    }
    if (key.keysym.sym == SDLK_3)
    {
        active_keys[SDLK_3] = true;
    }
    if (key.keysym.sym == SDLK_2)
    {
        active_keys[SDLK_2] = true;
    }
    if (key.keysym.sym == SDLK_5)
    {
        active_keys[SDLK_5] = true;
    }
    if (key.keysym.sym == SDLK_6)
    {
        active_keys[SDLK_6] = true;
    }
    // Handle 0 key press to increment the lap number
    if (key.keysym.sym == SDLK_0) {
        lapNumber++;
        if (lapNumber >= 3) {
            // Optional: handle additional logic when the player wins
            std::cout << "Player Wins!" << std::endl;
        }
    }
}

void GLViewSpeedRacer::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
    if (key.keysym.sym == SDLK_DOWN) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_UP) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_LEFT) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_RIGHT) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_s) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_w) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_a) {
        drivingSound->stopAllSounds();
    }
    if (key.keysym.sym == SDLK_d) {
        drivingSound->stopAllSounds();
    }

    if (key.keysym.sym == SDLK_1)
    {
        active_keys[SDLK_1] = false;
    }
    if (key.keysym.sym == SDLK_4)
    {
        active_keys[SDLK_4] = false;
    }
    if (key.keysym.sym == SDLK_3)
    {
        active_keys[SDLK_3] = false;
    }
    if (key.keysym.sym == SDLK_2)
    {
        active_keys[SDLK_2] = false;
    }
    if (key.keysym.sym == SDLK_5)
    {
        active_keys[SDLK_5] = false;
    }
    if (key.keysym.sym == SDLK_6)
    {
        active_keys[SDLK_6] = false;
    }
}



void Aftr::GLViewSpeedRacer::loadMap()
{
    this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
    this->actorLst = new WorldList();
    this->netLst = new WorldList();



    ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = true;
    this->glRenderer->isUsingShadowMapping(false); //set to TRUE to enable shadow mapping, must be using GL 3.2+

    this->cam->setPosition(0, 0, 0);

    std::string track("../../../modules/SpeedRacer/mm/models/Formula_Track.fbx");

    //SkyBox Textures readily available
    std::vector< std::string > skyBoxImageNames; //vector to store texture paths
    skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg");
    //skyBoxImageNames.push_back("../../../modules/SpeedRacer/mm/images/cobblestone_street_night_2k.hdr");
    {
        //Create a light
        float ga = 0.1f; //Global Ambient Light level for this module
        ManagerLight::setGlobalAmbientLight(aftrColor4f(ga, ga, ga, 1.0f));
        WOLight* light = WOLight::New();
        light->isDirectionalLight(true);
        light->setPosition(Vector(0, 0, 100));
        //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
        //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
        light->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat({ 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD));
        light->setLabel("Light");
        worldLst->push_back(light);
    }
    client = NetMessengerClient::New("127.0.0.1", ManagerEnvironmentConfiguration::getVariableValue("NetServerTransmitPort"));
    {
        //Create the SkyBox
        WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
        wo->setPosition(Vector(0, 0, 0));
        wo->setLabel("Sky Box");
        wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        worldLst->push_back(wo);
    }
    // Initialize the sound engine
    soundEngine = irrklang::createIrrKlangDevice();
    if (!soundEngine) {
        // Handle error
    }

    // Load the collision sound
  /*  irrklang::ISoundSource* collisionSoundPath = soundEngine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Collision.wav").c_str());
    soundEngine->addSoundSourceAlias(collisionSoundPath, "collision");
    soundList.push_back("collision");*/

    spawnPlayer1();
    OtherCarSkin1();
    OtherCarSkin2();
    OtherCarSkin3();
    spawnPlayer2();
    spawnPlayer2Skin1();
    spawnPlayer2Skin2();
    spawnPlayer2Skin3();


    // Default song
    //engine->play2D("../../../modules/SpeedRacer/mm/sounds/ride.mp3", true);

    // Add sound sources for grid terrain functions
    irrklang::ISoundSource* gridTerrainSound = soundEngine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "/sounds/nature.wav").c_str());
    soundEngine->addSoundSourceAlias(gridTerrainSound, "gridTerrainSound");
    soundList.push_back("gridTerrainSound");

    irrklang::ISoundSource* anotherGridTerrainSound = soundEngine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "/sounds/").c_str());
    soundEngine->addSoundSourceAlias(anotherGridTerrainSound, "anotherGridTerrainSound");
    soundList.push_back("anotherGridTerrainSound");

    // Initialize SDL_image
    IMG_Init(IMG_INIT_PNG);

    // Load images from library
    std::string startImagePath = "../../../modules/SpeedRacer/mm/images/start_image.png";
    std::string loadingImagePath = "../../../modules/SpeedRacer/mm/images/loading_image.png";

    startImage = loadTexture(startImagePath);
    if (!startImage) {
        printf("Failed to load start image from path: %s\n", startImagePath.c_str());
    }

    loadingImage = loadTexture(loadingImagePath);
    if (!loadingImage) {
        printf("Failed to load loading image from path: %s\n", loadingImagePath.c_str());
    }

    // Load and play the start screen soundtrack
    startScreenSoundtrack = soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/StartMenu.wav").c_str(), true, false, true);
    // Declare a boolean to track the window size state

    WOImGui* gui = WOImGui::New(nullptr);
    gui->setLabel("Switch Terrain");
    gui->subscribe_drawImGuiWidget([this, gui]() {
        // Get the size of the display
        ImVec2 displaySize = ImGui::GetIO().DisplaySize;

        // Calculate the elapsed time since the loading started
        Uint32 currentTime = SDL_GetTicks();
        Uint32 elapsedTime = currentTime - loadingStartTime;

        // Handle the different game states
        if (gameState == START_SCREEN) {
            // Show start screen
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(displaySize.x, displaySize.y), ImGuiCond_Always);
            ImGui::Begin("Start Game", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

            // Display the start image
            if (startImage) {
                ImGui::Image((void*)(intptr_t)startImage, displaySize);
            }
            else {
                ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 50.0f, displaySize.y - 100.0f));
                ImGui::Text("Press Space Bar to Start");
            }

            ImGui::End();

            // Check if the space bar is pressed
            const Uint8* state = SDL_GetKeyboardState(nullptr);
            if (state[SDL_SCANCODE_SPACE]) {
                // Transition to the loading screen
                gameState = LOADING;
                loadingStartTime = SDL_GetTicks();
                isLoading = true;
            }

            return; // Skip the rest of the update while in the start screen
        }
        else if (gameState == LOADING) {
            if (elapsedTime < 10000) { // 10 seconds in milliseconds
                // Show loading screen
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(displaySize.x, displaySize.y), ImGuiCond_Always);
                ImGui::Begin("Loading...", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

                // Display the loading image
                if (loadingImage) {
                    ImGui::Image((void*)(intptr_t)loadingImage, displaySize);
                }
                else {
                    ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 50.0f, displaySize.y - 100.0f));
                    ImGui::Text("Loading...");
                }

                ImGui::End();
                return; // Skip the rest of the update while loading
            }
            else if (isTerrainLoaded()) {
                // Transition to the main GUI after terrain is loaded
                gameState = MAIN_GUI;
                isLoading = false;
            }
        }
        // Draw a black background behind the GUI if showBlackScreen is true
        if (showBlackScreen) {
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(displaySize.x, displaySize.y), ImGuiCond_Always);
            ImGui::Begin("Background", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(0, 0), displaySize, IM_COL32(0, 0, 0, 255));
            ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 100.0f, displaySize.y - 100.0f));
            ImGui::Text("Press Space Bar to Remove Background");
            ImGui::End();

            // Check if the space bar is pressed
            const Uint8* state = SDL_GetKeyboardState(nullptr);
            if (state[SDL_SCANCODE_SPACE]) {
                showBlackScreen = false; // Hide the black screen
                this->spawnPlayer1();
            }
        }
        ImGui::Begin("Racecar Game Control Panel");
        ImGui::Separator();
        ImGui::Text("Lap: %d/3", lapNumber);
        ImGui::Separator();
        if (ImGui::SliderFloat("Car Speed", &moveAmount, 0.1f, 10.0f)) {
        }
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Game Modes")) {
            ImGui::Text("Time Trials Mode");
                // Button to start/pause the timer
                if (ImGui::Button(isTimerRunning ? "Pause Timer" : "Start Timer")) {
                    if (isTimerRunning) {
                        // Pause the timer
                        isTimerRunning = false;
                        pausedTime = SDL_GetTicks() - timerStartTime;
                    }
                    else {
                        // Resume or start the timer
                        timerStartTime = SDL_GetTicks() - pausedTime;
                        isTimerRunning = true;
                    }
                }

                // Display the reset time slider
                ImGui::SliderInt("Reset Time (s)", &resetTime, 10, 60);

                // Display the timer
                if (isTimerRunning) {
                    float elapsedTime = (SDL_GetTicks() - timerStartTime) / 1000.0f;
                    ImGui::Text("Timer: %.2f seconds", elapsedTime);

                    // Check if the elapsed time exceeds resetTime (in seconds)
                    if (elapsedTime > resetTime) {
                        resetCarPosition();

                        // Reset the timer start time
                        timerStartTime = SDL_GetTicks();
                    }
                }
                else {
                    float pausedElapsedTime = pausedTime / 1000.0f;
                    ImGui::Text("Timer: %.2f seconds", pausedElapsedTime);
                }
            
        }
        ImGui::Separator();
        // Main GUI code (this part is executed when gameState is MAIN_GUI)
        static float backgroundVolume = 1.0f;
        static bool wasSwitchToAnotherGridPressed = false;
        static bool wasSwitchToDefaultGridPressed = false;
        if (ImGui::CollapsingHeader("Racetrack Map Selection")) {
            bool isSwitchToAnotherGridPressed = ImGui::Button("Track 1");
            if (isSwitchToAnotherGridPressed && !wasSwitchToAnotherGridPressed) {
                // Reset loading flag for Terrain 1
                terrain1Loaded = false;
                gameState = LOADING;
                loadingStartTime = SDL_GetTicks();

                // Switch to another grid and apply additional transformations
                this->switchTerrain(true);
                this->moveTerrainDown(90.0f);
                this->rotateTerrain(-0.261799f * 6);
                this->moveTerrainNegativeX(50.0f * 11);

                // Stop all sounds and play "Track1.wav"
                soundEngine->stopAllSounds();
                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track1.wav").c_str(), true);

                // Set the camera position
                this->cam->setPosition(0.0f, 0.0f, -500.0f);

                // Reset flag for the default grid
                wasSwitchToDefaultGridPressed = false;

                // Send terrain change message
                sendTerrainChangeMessage(true, 90.0f, -0.261799f * 6, 50.0f * 11);

                // Set the loading state
                isLoading = true;
            }

            // Update flag for button press state
            wasSwitchToAnotherGridPressed = isSwitchToAnotherGridPressed;



            bool isSwitchToDefaultGridPressed = ImGui::Button("Track 2");
            if (isSwitchToDefaultGridPressed && !wasSwitchToDefaultGridPressed) {
                // Set loading state for Terrain 2
                terrain2Loaded = false; // Reset loading flag
                gameState = LOADING;
                loadingStartTime = SDL_GetTicks();

                // Ensure only "Default Grid" is loaded
                this->switchTerrain(false);
                this->moveTerrainDown(90.0f);
                this->rotateTerrain(-0.261799f * 6);
                this->moveTerrainNegativeX(50.0f * 11);

                // Play "nature.wav" after switching to default grid
                soundEngine->stopAllSounds();
                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/nature.wav").c_str(), true);

                // Set the camera position to the Z axis 500 positive
                this->cam->setPosition(0.0f, 0.0f, -500.0f);

                // Reset the flag for the another grid
                wasSwitchToAnotherGridPressed = false;

                sendTerrainChangeMessage(true, 90.0f, -0.261799f * 6, 50.0f * 11);

                // Set the loading state to wait for terrain to load
                isLoading = true;
            }
            wasSwitchToDefaultGridPressed = isSwitchToDefaultGridPressed;
        }
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Player 1 Controls")) {

            //if (ImGui::Button("Select Player1")) {
            //    this->spawnPlayer1();

            //    // Send network message to spawn Player1 in another instance
            //    if (client) {
            //        NetMsgSpawnCarPlayers msg;
            //        msg.carType = "Player1";
            //        client->sendNetMsgSynchronousTCP(msg);
            //    }
            //}

            // Define the available car skins
            static const char* carSkins[] = { "Dodge", "Ford", "Sports Car", "CyberTrunk" };
            static int selectedSkin = 0;

            ImGui::Combo("Select Car Skin", &selectedSkin, carSkins, IM_ARRAYSIZE(carSkins));

            if (ImGui::Button("Switch Player 1 Car")) {
                if (selectedSkin == 0) {
                    hideAllCars();
                    this->spawnPlayer1(); // Switch to Dodge skin locally
                }
                else if (selectedSkin == 1) {
                    hideAllCars();
                    this->OtherCarSkin1(); // Switch to Ford skin locally
                }
                else if (selectedSkin == 2) {
                    hideAllCars();
                    this->OtherCarSkin2(); // Switch to Sports Car skin locally
                }
                else if (selectedSkin == 3) {
                    hideAllCars();
                    this->OtherCarSkin3(); // Switch to CyberTrunk skin locally
                }

                // Send network message to switch car skin in other instances
                if (client) {
                    NetMsgChangeCarSkin msg;
                    switch (selectedSkin) {
                    case 0: msg.skinType = "Dodge"; break;
                    case 1: msg.skinType = "Ford"; break;
                    case 2: msg.skinType = "Sports Car"; break;
                    case 3: msg.skinType = "CyberTrunk"; break;
                    }
                    msg.player = "Player1";
                    client->sendNetMsgSynchronousTCP(msg);
                }
            }
            Uint32 currentTime = SDL_GetTicks();

            // Check for Player 2 speed boost key press
            const Uint8* state = SDL_GetKeyboardState(nullptr);
            if (state[SDL_SCANCODE_RCTRL]) {
                if (canPressPlayer1SpeedButton) {
                    player1SpeedBoostAmount = ((rand() % 3) + 3); // Randomly select speed boost between 3 and 5
                    moveAmount += player1SpeedBoostAmount;
                    isPlayer1SpeedBoostActive = true;
                    player1SpeedBoostStartTime = SDL_GetTicks();
                    lastPlayer1SpeedBoostTime = SDL_GetTicks();
                    canPressPlayer1SpeedButton = false; // Reset the button press availability
                }
            }

            // Timer for Player 2 Speed Boost
            if (isPlayer1SpeedBoostActive) {
                float boostDuration = 5.0f; // Duration of the speed boost in seconds
                float elapsedTime = (currentTime - player1SpeedBoostStartTime) / 1000.0f;
                float remainingTime = boostDuration - elapsedTime;

                if (remainingTime <= 0.0f) {
                    moveAmount = 8.0f; // Reset the speed to default
                    isPlayer1SpeedBoostActive = false;
                    player1SpeedBoostAmount = 0;
                }

                // Display the countdown
                ImGui::Text("Speed Boost Countdown: %.1f seconds", remainingTime);
            }
            else {
                // Display default countdown message when boost is not active
                ImGui::Text("Speed Boost Countdown: N/A");
            }

            // Cooldown for Player 1 Speed Boost
            if (currentTime - lastPlayer1SpeedBoostTime >= 6000) { // 6 seconds cooldown
                canPressPlayer1SpeedButton = true;
            }

            // Display the updated speed
            ImGui::Text("Updated Speed: %.2f", moveAmount);
        }


        ImGui::Separator();
        if (ImGui::CollapsingHeader("Player 2 Controls")) {
            //if (ImGui::Button("Select Player2")) {
            //    this->spawnPlayer2();

            //    // Send network message to spawn Player2 in another instance
            //    if (client) {
            //        NetMsgSpawnCarPlayers msg;
            //        msg.carType = "Player2";
            //        client->sendNetMsgSynchronousTCP(msg);
            //    }
            //}

            // Define the available car skins for Player 2
            static const char* carSkinsPlayer2[] = { "Dodge", "Ford", "Sports Car", "CyberTrunk" };
            static int selectedSkinPlayer2 = 0;

            ImGui::Combo("Select Player 2 Car", &selectedSkinPlayer2, carSkinsPlayer2, IM_ARRAYSIZE(carSkinsPlayer2));

            if (ImGui::Button("Switch Player 2 Car")) {
                if (selectedSkinPlayer2 == 0) {
                    hideAllCars2();
                    this->spawnPlayer2(); // Switch to Dodge skin locally
                }
                else if (selectedSkinPlayer2 == 1) {
                    hideAllCars2(); 
                    this->spawnPlayer2Skin1(); // Switch to Ford skin locally
                }
                else if (selectedSkinPlayer2 == 2) {
                    hideAllCars2(); 
                    this->spawnPlayer2Skin2(); // Switch to Sports Car skin locally
                }
                else if (selectedSkinPlayer2 == 3) {
                    hideAllCars2(); 
                    this->spawnPlayer2Skin3(); // Switch to CyberTrunk skin locally
                }

                // Send network message to switch car skin in other instances
                if (client) {
                    NetMsgChangeCarSkin msg;
                    switch (selectedSkinPlayer2) {
                    case 0: msg.skinType = "Dodge"; break;
                    case 1: msg.skinType = "Ford"; break;
                    case 2: msg.skinType = "Sports Car"; break;
                    case 3: msg.skinType = "CyberTrunk"; break;
                    }
                    msg.player = "Player2";
                    client->sendNetMsgSynchronousTCP(msg);
                }
            }

            Uint32 currentTime = SDL_GetTicks();

            // Check for Player 2 speed boost key press
            const Uint8* state = SDL_GetKeyboardState(nullptr);
            if (state[SDL_SCANCODE_TAB]) {
                if (canPressPlayer2SpeedButton) {
                    player2SpeedBoostAmount = ((rand() % 3) + 3); // Randomly select speed boost between 3 and 5
                    moveAmount += player2SpeedBoostAmount;
                    isPlayer2SpeedBoostActive = true;
                    player2SpeedBoostStartTime = currentTime;
                    lastPlayer2SpeedBoostTime = currentTime;
                    canPressPlayer2SpeedButton = false; // Reset the button press availability
                }
            }

            // Timer for Player 2 Speed Boost
            if (isPlayer2SpeedBoostActive) {
                float boostDuration = 5.0f; // Duration of the speed boost in seconds
                float elapsedTime = (currentTime - player2SpeedBoostStartTime) / 1000.0f;
                float remainingTime = boostDuration - elapsedTime;

                if (remainingTime <= 0.0f) {
                    moveAmount = 8.0f; // Reset the speed to default
                    isPlayer2SpeedBoostActive = false;
                    player2SpeedBoostAmount = 0;
                }

                // Display the countdown
                ImGui::Text("Speed Boost Countdown: %.1f seconds", remainingTime);
            }
            else {
                // Display default countdown message when boost is not active
                ImGui::Text("Speed Boost Countdown: N/A");
            }

            // Cooldown for Player 2 Speed Boost
            if (currentTime - lastPlayer2SpeedBoostTime >= 6000) { // 6 seconds cooldown
                canPressPlayer2SpeedButton = true;
            }

            // Display the updated speed
            ImGui::Text("Updated Speed: %.2f", moveAmount);
        }
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Audio")) {

            // Dropdown menu for selecting music
            static int selectedMusicIndex = 0;
            static const char* musicOptions[] = { "SoundTrack1", "SoundTrack2", "SoundTrack3", "SoundTrack4" };
            if (ImGui::BeginCombo("Select Music", musicOptions[selectedMusicIndex])) {
                for (int i = 0; i < IM_ARRAYSIZE(musicOptions); i++) {
                    bool isSelected = (selectedMusicIndex == i);
                    if (ImGui::Selectable(musicOptions[i], isSelected)) {
                        selectedMusicIndex = i;

                        // Handle music change considering mute state
                        soundEngine->stopAllSounds();
                        if (!GLViewSpeedRacer::isMuted) {
                            switch (selectedMusicIndex) {
                            case 0:
                                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track1.wav").c_str(), true);
                                break;
                            case 1:
                                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track2.wav").c_str(), true);
                                break;
                            case 2:
                                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track3.wav").c_str(), true);
                                break;
                            case 3:
                                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track4.wav").c_str(), true);
                                break;
                            }
                        }
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            // Volume slider
            if (ImGui::SliderFloat("Volume", &backgroundVolume, 0.0f, 1.0f)) {
                soundEngine->setSoundVolume(backgroundVolume);
            }

            // Mute button
            if (ImGui::Button(GLViewSpeedRacer::isMuted ? "Unmute" : "Mute")) {
                GLViewSpeedRacer::isMuted = !GLViewSpeedRacer::isMuted;
                soundEngine->stopAllSounds();
                if (!GLViewSpeedRacer::isMuted) {
                    // Play the currently selected track again
                    switch (selectedMusicIndex) {
                    case 0:
                        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track1.wav").c_str(), true);
                        break;
                    case 1:
                        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track2.wav").c_str(), true);
                        break;
                    case 2:
                        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track3.wav").c_str(), true);
                        break;
                    case 3:
                        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track4.wav").c_str(), true);
                        break;
                    }
                }
            }
        }
        ImGui::Separator();
        // Button to switch camera view
        if (ImGui::Button("Switch Camera View")) {
            followCar1 = !followCar1; // Toggle the followCar1 flag
        }

        ImGui::End();
        });
    this->worldLst->push_back(gui);


    //createSpeedRacerWayPoints();
}



void Aftr::GLViewSpeedRacer::updateGravity(physx::PxVec3 g)
{
    gravity = g;
}

void GLViewSpeedRacer::createSpeedRacerWayPoints()
{
    // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
    WayPointParametersBase params(this);
    params.frequency = 5000;
    params.useCamera = true;
    params.visible = false;
    WOWayPointSpherical* wayPt = WOWayPointSpherical::New(params, 3);
    wayPt->setPosition(Vector(50, 0, 3));
    worldLst->push_back(wayPt);
}
void GLViewSpeedRacer::switchTerrain(bool useAnotherGrid) {
    for (WO* wo : *worldLst) {
        if (wo->getLabel() == "terrain" || wo->getLabel() == "anotherTerrain") {
            worldLst->eraseViaWOptr(wo);
            break;
        }
    }

    if (useAnotherGrid) {
        createAnotherGrid();
        switchSkyBox(1); // Switch to the second skybox
        anotherGridTerrainFunction(); // Play another grid terrain sound
    }
    else {
        createGrid();
        gridTerrainFunction();
        switchSkyBox(0); // Switch to the first skybox
    }

}

void GLViewSpeedRacer::gridTerrainFunction() {
    // Implementation of the grid terrain function
    soundEngine->play2D("gridTerrainSound", true); // Play in loop
}
void GLViewSpeedRacer::anotherGridTerrainFunction() {
    // Implementation of the another grid terrain function
    soundEngine->play2D("anotherGridTerrainSound", true); // Play in loop
}
void GLViewSpeedRacer::createGrid()
{
    std::string elevationPath = ManagerEnvironmentConfiguration::getLMM() + "/images/racetrackSand_heightmap.tiff";
    std::string texturePath = ManagerEnvironmentConfiguration::getLMM() + "/images/racetrackSandMap.png";
    // Define the boundaries of the grid
    float top = 0.15f;
    float bottom = 0.16f;
    float left = -0.15f;
    float right = -0.16f;

    float vert = top - bottom;
    float horz = right - left;


    // Define the offset and scale for the grid
    VectorD offset((top + bottom) / 2, (left + right) / 2, 0);
    // Set up the center of the world and gravity direction
    auto centerOfWorld = offset.toVecS().toECEFfromWGS84();
    auto gravityDirection = -centerOfWorld;
    gravityDirection.normalize();

    VectorD scale = VectorD(1.0f, 1.0f, 1.0f);
    VectorD upperLeft(top, left, 0);
    VectorD lowerRight(bottom, right, 0);

    // Debug print paths
    std::cout << "Elevation Path: " << elevationPath << std::endl;

    // Create the grid using the MyGrid class
    WOGridECEFElevationPhysX* grid = WOGridECEFElevationPhysX::New(pxPhysics, pxScene, upperLeft, lowerRight, offset, scale, elevationPath, texturePath);
    grid->setLabel("terrain");
    grid->useFrustumCulling = false;

    grid->upon_async_model_loaded([this, gravityDirection, centerOfWorld]() {
        updateGravity(physx::PxVec3{ gravityDirection.x * 8, gravityDirection.y, gravityDirection.z });
        this->getCamera()->setCameraAxisOfHorizontalRotationViaMouseMotion(centerOfWorld.toVecS());
        });
    worldLst->push_back((WO*)grid);
}

void GLViewSpeedRacer::createAnotherGrid()
{
    std::string elevationPath = ManagerEnvironmentConfiguration::getLMM() + "/images/racetrackV2_heightmap.tiff";
    std::string texturePath = ManagerEnvironmentConfiguration::getLMM() + "/images/racetrackV2.png";
    float top = 0.15f;
    float bottom = 0.16f;
    float left = -0.15f;
    float right = -0.16f;

    float vert = top - bottom;
    float horz = right - left;

    VectorD offset((top + bottom) / 2, (left + right) / 2, 0);
    auto centerOfWorld = offset.toVecS().toECEFfromWGS84();
    auto gravityDirection = -centerOfWorld;
    gravityDirection.normalize();

    VectorD scale = VectorD(1.0f, 1.0f, 1.0f);
    VectorD upperLeft(top, left, 0);
    VectorD lowerRight(bottom, right, 0);

    WOGridECEFElevationPhysX* grid = WOGridECEFElevationPhysX::New(pxPhysics, pxScene, upperLeft, lowerRight, offset, scale, elevationPath, texturePath);
    grid->setLabel("anotherTerrain");
    grid->useFrustumCulling = false;

    grid->upon_async_model_loaded([this, gravityDirection, centerOfWorld]() {
        updateGravity(physx::PxVec3{ gravityDirection.x * 8, gravityDirection.y, gravityDirection.z });
        this->getCamera()->setCameraAxisOfHorizontalRotationViaMouseMotion(centerOfWorld.toVecS());
        });
    worldLst->push_back((WO*)grid);
}

void GLViewSpeedRacer::switchSkyBox(int index)
{
    // Remove the existing skybox
    for (WO* wo : *worldLst)
    {
        if (wo->getLabel() == "SkyBox")
        {
            worldLst->eraseViaWOptr(wo);
            break;
        }
    }

    // Create a new skybox based on the index
    if (index == 0)
    {
        worldLst->push_back(WOSkyBox::New(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg", this->getCameraPtrPtr()));
    }
    else if (index == 1)
    {
        worldLst->push_back(WOSkyBox::New(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg", this->getCameraPtrPtr()));
    }
}
void GLViewSpeedRacer::rotateTerrain(float angle) {
    for (WO* wo : *worldLst) {
        if (wo->getLabel() == "terrain" || wo->getLabel() == "anotherTerrain") {
            // Apply rotation to the terrain
            wo->getModel()->rotateAboutGlobalY(angle);
            break;
        }
    }
}

void GLViewSpeedRacer::moveTerrainDown(float amount) {
    for (WO* wo : *worldLst) {
        if (wo->getLabel() == "terrain" || wo->getLabel() == "anotherTerrain") {
            // Move the terrain down along the Z-axis
            Vector currentPos = wo->getPosition();
            wo->setPosition(currentPos.x, currentPos.y, currentPos.z - amount);
            std::cout << "Terrain moved down by " << amount << ". New Z position: " << currentPos.z - amount << std::endl;
            break;
        }
    }
}
void GLViewSpeedRacer::moveTerrainUp(float amount) {
    for (WO* wo : *worldLst) {
        if (wo->getLabel() == "terrain" || wo->getLabel() == "anotherTerrain") {
            // Move the terrain up along the Z-axis
            Vector currentPos = wo->getPosition();
            wo->setPosition(currentPos.x, currentPos.y, currentPos.z + amount);
            std::cout << "Terrain moved up by " << amount << ". New Z position: " << currentPos.z + amount << std::endl;
            break;
        }
    }
}
void GLViewSpeedRacer::moveTerrainNegativeX(float amount) {
    for (WO* wo : *worldLst) {
        if (wo->getLabel() == "terrain" || wo->getLabel() == "anotherTerrain") {
            // Move the terrain along the negative X-axis
            Vector currentPos = wo->getPosition();
            wo->setPosition(currentPos.x - amount, currentPos.y, currentPos.z);
            std::cout << "Terrain moved along negative X-axis by " << amount << ". New X position: " << currentPos.x - amount << std::endl;
            break;
        }
    }
}

void GLViewSpeedRacer::spawnPlayer1() {
    std::string Car_Up = ManagerEnvironmentConfiguration::getLMM() + "/models/CarDodgeRight.fbx";
    std::string Car_Right = ManagerEnvironmentConfiguration::getLMM() + "/models/DodgeCharger.fbx";
    std::string Car_Left = ManagerEnvironmentConfiguration::getLMM() + "/models/CarDodgeDown.fbx";
    std::string Car_Down = ManagerEnvironmentConfiguration::getLMM() + "/models/CarDodgeNew.fbx"; // New car model path

    car_test = Car::New(Car_Up, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_test->setPos(Vector(0, 0, 5));
    car_test->setPose(spy_pose);
    car_test->rotateAboutGlobalZ(-4.60f);
    car_test->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_test->isVisible = false;
    worldLst->push_back(car_test);

    car_turn = Car::New(Car_Right, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_turn->setPos(Vector(0, 0, 5));
    car_turn->setPose(spy_pose);
    car_turn->rotateAboutGlobalZ(-4.60f);
    car_turn->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_turn->isVisible = false;
    worldLst->push_back(car_turn);

    car_other_side = Car::New(Car_Left, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_other_side->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of car_test
    car_other_side->setPose(spy_pose);
    car_other_side->rotateAboutGlobalZ(-4.60f);
    car_other_side->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_other_side->isVisible = false;
    worldLst->push_back(car_other_side);

    car_new = Car::New(Car_Down, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_new->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    car_new->setPose(spy_pose);
    car_new->rotateAboutGlobalZ(-4.60f);
    car_new->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_new->isVisible = true;
    worldLst->push_back(car_new);


}
void GLViewSpeedRacer::OtherCarSkin1() {
    std::string Car_Up = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/FordCarDirection.dae";
    std::string Car_Right = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/LowPoly Muscle Cougar xr1970.dae";
    std::string Car_Left = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionOp.dae";
    std::string Car_Down = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae";

    car_test = Car::New(Car_Up, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_test->setPos(Vector(0, 0, 5));
    car_test->setPose(spy_pose);
    car_test->rotateAboutGlobalZ(-4.60f);
    car_test->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_test->isVisible = false;
    worldLst->push_back(car_test);

    car_turn = Car::New(Car_Right, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_turn->setPos(Vector(0, 0, 5));
    car_turn->setPose(spy_pose);
    car_turn->rotateAboutGlobalZ(-4.60f);
    car_turn->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_turn->isVisible = false;
    worldLst->push_back(car_turn);

    car_other_side = Car::New(Car_Left, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_other_side->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of car_test
    car_other_side->setPose(spy_pose);
    car_other_side->rotateAboutGlobalZ(-4.60f);
    car_other_side->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_other_side->isVisible = false;
    worldLst->push_back(car_other_side);

    car_new = Car::New(Car_Down, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_new->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    car_new->setPose(spy_pose);
    car_new->rotateAboutGlobalZ(-4.60f);
    car_new->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_new->isVisible = true;
    worldLst->push_back(car_new);

}
void GLViewSpeedRacer::OtherCarSkin2() {
    std::string Car_Up = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTestRight.fbx";
    std::string Car_Right = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTest.fbx";
    std::string Car_Left = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTestLeft.fbx";
    std::string Car_Down = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTestOtherWay.fbx";

    car_test = Car::New(Car_Up, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_test->setPos(Vector(0, 0, 5));
    car_test->setPose(spy_pose);
    car_test->rotateAboutGlobalZ(-4.60f);
    car_test->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_test->isVisible = false;
    worldLst->push_back(car_test);

    car_turn = Car::New(Car_Right, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_turn->setPos(Vector(0, 0, 5));
    car_turn->setPose(spy_pose);
    car_turn->rotateAboutGlobalZ(-4.60f);
    car_turn->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_turn->isVisible = false;
    worldLst->push_back(car_turn);

    car_other_side = Car::New(Car_Left, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_other_side->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of car_test
    car_other_side->setPose(spy_pose);
    car_other_side->rotateAboutGlobalZ(-4.60f);
    car_other_side->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_other_side->isVisible = false;
    worldLst->push_back(car_other_side);

    car_new = Car::New(Car_Down, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_new->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    car_new->setPose(spy_pose);
    car_new->rotateAboutGlobalZ(-4.60f);
    car_new->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_new->isVisible = true;
    worldLst->push_back(car_new);

}
void GLViewSpeedRacer::OtherCarSkin3() {
    std::string Car_Up = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkFront.fbx";
    std::string Car_Right = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkLeft.fbx";
    std::string Car_Left = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkBack.fbx";
    std::string Car_Down = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkRight.fbx";

    car_test = Car::New(Car_Up, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_test->setPos(Vector(0, 0, 5));
    car_test->setPose(spy_pose);
    car_test->rotateAboutGlobalZ(-4.60f);
    car_test->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_test->isVisible = false;
    worldLst->push_back(car_test);

    car_turn = Car::New(Car_Right, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_turn->setPos(Vector(0, 0, 5));
    car_turn->setPose(spy_pose);
    car_turn->rotateAboutGlobalZ(-4.60f);
    car_turn->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_turn->isVisible = false;
    worldLst->push_back(car_turn);

    car_other_side = Car::New(Car_Left, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_other_side->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of car_test
    car_other_side->setPose(spy_pose);
    car_other_side->rotateAboutGlobalZ(-4.60f);
    car_other_side->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_other_side->isVisible = false;
    worldLst->push_back(car_other_side);

    car_new = Car::New(Car_Down, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_new->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    car_new->setPose(spy_pose);
    car_new->rotateAboutGlobalZ(-4.60f);
    car_new->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    car_new->isVisible = true;
    worldLst->push_back(car_new);

}
void GLViewSpeedRacer::spawnPlayer2() {

    std::string carModelUp = ManagerEnvironmentConfiguration::getLMM() + "/models/CarDodgeRight.fbx";
    std::string carModelRight = ManagerEnvironmentConfiguration::getLMM() + "/models/DodgeCharger.fbx";
    std::string carModelLeft = ManagerEnvironmentConfiguration::getLMM() + "/models/CarDodgeDown.fbx";
    std::string carModelDown = ManagerEnvironmentConfiguration::getLMM() + "/models/CarDodgeNew.fbx";


    // Initialize carMain
    carMain = Car::New(carModelUp, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carMain->setPos(Vector(0, 0, 5));
    carMain->setPose(spy_pose);
    carMain->rotateAboutGlobalZ(-4.60f);
    carMain->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carMain->isVisible = false;
    worldLst->push_back(carMain);

    // Initialize carRight
    carRight = Car::New(carModelRight, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carRight->setPos(Vector(0, 0, 5));
    carRight->setPose(spy_pose);
    carRight->rotateAboutGlobalZ(-4.60f);
    carRight->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carRight->isVisible = false;
    worldLst->push_back(carRight);

    // Initialize carLeft
    carLeft = Car::New(carModelLeft, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carLeft->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of carMain
    carLeft->setPose(spy_pose);
    carLeft->rotateAboutGlobalZ(-4.60f);
    carLeft->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carLeft->isVisible = false;
    worldLst->push_back(carLeft);

    // Initialize carDown
    carDown = Car::New(carModelDown, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carDown->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    carDown->setPose(spy_pose);
    carDown->rotateAboutGlobalZ(-4.60f);
    carDown->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carDown->isVisible = true;
    worldLst->push_back(carDown);
}

void GLViewSpeedRacer::spawnPlayer2Skin1() {

    std::string carModelUp = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/FordCarDirection.dae";
    std::string carModelRight = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/LowPoly Muscle Cougar xr1970.dae";
    std::string carModelLeft = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionOp.dae";
    std::string carModelDown = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae";

    // Initialize carMain
    carMain = Car::New(carModelUp, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carMain->setPos(Vector(0, 0, 5));
    carMain->setPose(spy_pose);
    carMain->rotateAboutGlobalZ(-4.60f);
    carMain->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carMain->isVisible = false;
    worldLst->push_back(carMain);

    // Initialize carRight
    carRight = Car::New(carModelRight, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carRight->setPos(Vector(0, 0, 5));
    carRight->setPose(spy_pose);
    carRight->rotateAboutGlobalZ(-4.60f);
    carRight->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carRight->isVisible = false;
    worldLst->push_back(carRight);

    // Initialize carLeft
    carLeft = Car::New(carModelLeft, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carLeft->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of carMain
    carLeft->setPose(spy_pose);
    carLeft->rotateAboutGlobalZ(-4.60f);
    carLeft->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carLeft->isVisible = false;
    worldLst->push_back(carLeft);

    // Initialize carDown
    carDown = Car::New(carModelDown, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carDown->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    carDown->setPose(spy_pose);
    carDown->rotateAboutGlobalZ(-4.60f);
    carDown->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carDown->isVisible = true;
    worldLst->push_back(carDown);
}

void GLViewSpeedRacer::spawnPlayer2Skin2() {

    std::string carModelUp = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTestRight.fbx";
    std::string carModelRight = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTest.fbx";
    std::string carModelLeft = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTestLeft.fbx";
    std::string carModelDown = ManagerEnvironmentConfiguration::getLMM() + "/models/CarTestOtherWay.fbx";

    // Initialize carMain
    carMain = Car::New(carModelUp, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carMain->setPos(Vector(0, 0, 5));
    carMain->setPose(spy_pose);
    carMain->rotateAboutGlobalZ(-4.60f);
    carMain->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carMain->isVisible = false;
    worldLst->push_back(carMain);

    // Initialize carRight
    carRight = Car::New(carModelRight, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carRight->setPos(Vector(0, 0, 5));
    carRight->setPose(spy_pose);
    carRight->rotateAboutGlobalZ(-4.60f);
    carRight->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carRight->isVisible=false;
    worldLst->push_back(carRight);

    // Initialize carLeft
    carLeft = Car::New(carModelLeft, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carLeft->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of carMain
    carLeft->setPose(spy_pose);
    carLeft->rotateAboutGlobalZ(-4.60f);
    carLeft->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carLeft->isVisible=false;
    worldLst->push_back(carLeft);

    // Initialize carDown
    carDown = Car::New(carModelDown, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carDown->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    carDown->setPose(spy_pose);
    carDown->rotateAboutGlobalZ(-4.60f);
    carDown->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carDown->isVisible = true;
    worldLst->push_back(carDown);
}

void GLViewSpeedRacer::spawnPlayer2Skin3() {
    std::string carModelUp = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkFront.fbx";
    std::string carModelRight = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkLeft.fbx";
    std::string carModelLeft = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkBack.fbx";
    std::string carModelDown = ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunkRight.fbx";


    // Initialize carMain
    carMain = Car::New(carModelUp, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carMain->setPos(Vector(0, 0, 5));
    carMain->setPose(spy_pose);
    carMain->rotateAboutGlobalZ(-4.60f);
    carMain->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carMain->isVisible = false;
    worldLst->push_back(carMain);

    // Initialize carRight
    carRight = Car::New(carModelRight, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carRight->setPos(Vector(0, 0, 5));
    carRight->setPose(spy_pose);
    carRight->rotateAboutGlobalZ(-4.60f);
    carRight->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carRight->isVisible = false;
    worldLst->push_back(carRight);

    // Initialize carLeft
    carLeft = Car::New(carModelLeft, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carLeft->setPos(Vector(0, 0, 5)); // Adjust the position to be on the other side of carMain
    carLeft->setPose(spy_pose);
    carLeft->rotateAboutGlobalZ(-4.60f);
    carLeft->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carLeft->isVisible = false;
    worldLst->push_back(carLeft);

    // Initialize carDown
    carDown = Car::New(carModelDown, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carDown->setPos(Vector(0, 0, 5)); // Adjust the position as needed
    carDown->setPose(spy_pose);
    carDown->rotateAboutGlobalZ(-4.60f);
    carDown->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    carDown->isVisible = true;
    worldLst->push_back(carDown);
}


void GLViewSpeedRacer::setCarSpeed(float speed) {
    car_test->setSpeed(speed);
    car_turn->setSpeed(speed);
    car_other_side->setSpeed(speed);
    car_new->setSpeed(speed);
    carMain->setSpeed(speed);
    carRight->setSpeed(speed);
    carLeft->setSpeed(speed);
    carDown->setSpeed(speed);
}


void GLViewSpeedRacer::loadAssets() {
    isLoading = true;

    // Simulate asset loading with a sleep (replace with actual asset loading)
    SDL_Delay(2000); // Simulate a 2-second loading time

    // Load your assets here
    // Example: load textures, models, sounds, etc.

    isAssetsLoaded = true;
    isLoading = false;
}
SDL_Texture* GLViewSpeedRacer::loadTexture(const std::string& path) {
    SDL_Surface* surface = IMG_Load(path.c_str());
    if (!surface) {
        printf("Unable to load image %s! SDL_image Error: %s\n", path.c_str(), IMG_GetError());
        return nullptr;
    }
    SDL_Window* window = SDL_CreateWindow("SpeedRacer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_OPENGL);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);

    if (!texture) {
        printf("Unable to create texture from %s! SDL Error: %s\n", path.c_str(), SDL_GetError());
    }

    return texture;
}
bool GLViewSpeedRacer::isTerrainLoaded() {
    return terrain1Loaded || terrain2Loaded;
}
void GLViewSpeedRacer::resetCarPosition() {
    // Set the car's position to the initial position
    car_test->setPos(initialCarPosition);
    car_turn->setPos(Aftr::Vector(initialCarPosition.x + 12, initialCarPosition.y, initialCarPosition.z));
    car_other_side->setPos(Aftr::Vector(initialCarPosition.x - 12, initialCarPosition.y, initialCarPosition.z));
    car_new->setPos(Aftr::Vector(initialCarPosition.x, initialCarPosition.y + 12, initialCarPosition.z));

    // Reset visibility states
    car_test->isVisible = true;
    car_turn->isVisible = false;
    car_other_side->isVisible = false;
    car_new->isVisible = false;
}
void GLViewSpeedRacer::updateActiveKeys(SDL_KeyCode keycode, bool state)
{
    active_keys[keycode] = state;
}
void GLViewSpeedRacer::updateControls()
{
    if (active_keys[SDLK_1])
    {
        this->getCamera()->moveRelative(Vector(0, 0, 1) * this->getCamera()->getCameraVelocity() * 75); // Increased speed by a factor of 5
    }

    if (active_keys[SDLK_4])
    {
        this->getCamera()->moveRelative(Vector(0, 0, -1) * this->getCamera()->getCameraVelocity() * 75);
    }
    if (active_keys[SDLK_3])
    {
        this->getCamera()->moveRelative(Vector(1, 0, 0) * this->getCamera()->getCameraVelocity() * 75); // Increased speed by a factor of 5
    }

    if (active_keys[SDLK_2])
    {
        this->getCamera()->moveRelative(Vector(-1, 0, 0) * this->getCamera()->getCameraVelocity() * 75);
    }
    // New key press to move the camera along -X direction and look down the -Z axis
    if (active_keys[SDLK_5])
    {
        // Move the camera in the -X direction
        this->getCamera()->moveRelative(Vector(-1, 0, 0) * this->getCamera()->getCameraVelocity() * 100);

        // Rotate the camera to look down the -Z axis
        this->getCamera()->setCameraLookAtPoint(this->getCamera()->getPosition() + Vector(0, 0, -1));
    }
    if (active_keys[SDLK_6])
    {
        this->getCamera()->moveRelative(Vector(0, 0, 1) * this->getCamera()->getCameraVelocity() * 275); // Increased speed by a factor of 5
    }

}


void GLViewSpeedRacer::sendTerrainChangeMessage(bool useAnotherGrid, float moveDownAmount, float rotateAmount, float moveNegativeXAmount) {
    NetMsgSwitchTerrain msg;
    msg.useAnotherGrid = useAnotherGrid;
    msg.moveDownAmount = moveDownAmount;
    msg.rotateAmount = rotateAmount;
    msg.moveNegativeXAmount = moveNegativeXAmount;
    //
    // Send network message
    client->sendNetMsgSynchronousTCP(msg);
}
void GLViewSpeedRacer::handleCarMovement(int carModel, int keyPress, float moveAmount) {
    Car* currentCar = nullptr;

    switch (carModel) {
    case CAR_MODEL_DODGE:
        currentCar = carDodge;
        break;
    case CAR_MODEL_FORD:
        currentCar = carFord;
        break;
        // Add cases for additional models if needed
    }

    if (!currentCar) return;
    //
    // Handle car visibility and position based on key presses
    switch (keyPress) {
    case SDLK_DOWN:
        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        currentCar->isVisible = true;
        // Apply movement
        {
            physx::PxVec3 rightVector = currentCar->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
            physx::PxVec3 newPos = currentCar->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount * -1.0f;
            currentCar->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        }
        break;
    case SDLK_UP:
        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        currentCar->isVisible = false;
        // Apply movement
        {
            physx::PxVec3 rightVector = currentCar->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
            physx::PxVec3 newPos = currentCar->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount;
            currentCar->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        }
        break;
    case SDLK_LEFT:
        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        // Apply movement
        {
            physx::PxVec3 forward = currentCar->getForwardVector();
            physx::PxVec3 newPos = currentCar->getRigidDynamic()->getGlobalPose().p + forward * moveAmount;
            currentCar->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        }
        break;
    case SDLK_RIGHT:
        soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
        // Apply movement
        {
            physx::PxVec3 forward = currentCar->getForwardVector();
            physx::PxVec3 newPos = currentCar->getRigidDynamic()->getGlobalPose().p + forward * moveAmount * -1.0f;
            currentCar->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        }
        break;
    }
}
void GLViewSpeedRacer::hideAllCars() {
    if (car_test) car_test->isVisible = false;
    if (car_turn) car_turn->isVisible = false;
    if (car_other_side) car_other_side->isVisible = false;
    if (car_new) car_new->isVisible = false;
}
void GLViewSpeedRacer::hideAllCars2() {
    if (carMain) carMain->isVisible = false;
    if (carRight) carRight->isVisible = false;
    if (carLeft) carLeft->isVisible = false;
    if (carDown) carDown->isVisible = false;
}
