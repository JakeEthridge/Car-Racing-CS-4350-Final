
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
#include <NetMsgAudio.h>
#include <NetMsgTerrainLoaded .h>
#include "Car.h"

namespace Aftr {

    bool GLViewSpeedRacer::isMuted = false; // Define static member
    int GLViewSpeedRacer::selectedMusicIndex = 0; // Define static member



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
        spawnPlayer1();
        spawnPlayer2();
        OtherCarSkin2();
        
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

                // Get car transform
                physx::PxTransform carTransform = carWO->getRigidDynamic()->getGlobalPose();
                physx::PxVec3 carPosition = carTransform.p;

                // Restrict car's Z-axis movement
                if (carPosition.z < -5.0f) {
                    // Apply an upward force to lift the car above the Z-axis
                    physx::PxVec3 upwardForce(0.0f, 0.0f, 1.0f); // Adjust the force value as needed
                    carWO->getRigidDynamic()->addForce(upwardForce, physx::PxForceMode::eIMPULSE);
                }
                else if (carPosition.z > 5.0f) {
                    // Adjust car's Z position to be within the allowed range
                    physx::PxVec3 correctedPosition = carPosition;
                    correctedPosition.z = 5.0f; // Set to the maximum allowed Z value
                    carWO->getRigidDynamic()->setGlobalPose(physx::PxTransform(correctedPosition, carTransform.q));
                }
            }

            Car* visibleCar = followCar1 ? getVisibleCar1() : getVisibleCar2();
            physx::PxVec3 cameraOffset(0, -20, 7);

            if (visibleCar) {
                physx::PxTransform carTransform = visibleCar->getRigidDynamic()->getGlobalPose();
                physx::PxVec3 carPosition = carTransform.p;

                // Set the camera offset based on which car is visible
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
            else if (car1 || car2 || car3) {  // Fallback option: Follow car1 if no other car is visible
                Aftr::Vector car1Position = car1->getPosition();
                Aftr::Vector car1CameraOffset = Aftr::Vector(0, -15, 5); // Specific offset for car1

                Aftr::Vector desiredCameraPosition = car1Position + car1CameraOffset;

                Aftr::Vector currentCameraPosition = this->getCamera()->getPosition();
                Aftr::Vector newCameraPosition = Aftr::Vector(
                    lerp(currentCameraPosition.x, desiredCameraPosition.x, 0.1f),
                    lerp(currentCameraPosition.y, desiredCameraPosition.y, 0.1f),
                    lerp(currentCameraPosition.z, desiredCameraPosition.z, 0.1f)
                );

                this->getCamera()->setPosition(newCameraPosition);

                Aftr::Vector currentLookAtPoint = this->getCamera()->getCameraLookAtPoint();
                Aftr::Vector newLookAtPoint = Aftr::Vector(
                    lerp(currentLookAtPoint.x, car1Position.x, 0.1f),
                    lerp(currentLookAtPoint.y, car1Position.y, 0.1f),
                    lerp(currentLookAtPoint.z, car1Position.z, 0.1f)
                );

                this->getCamera()->setCameraLookAtPoint(newLookAtPoint);
            }

            delete[] dynamicActors;
        }

        updateCamera();

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
       
        // Handle car visibility and position based on key presses
        if (key.keysym.sym == SDLK_DOWN) {
            drivingSound->setSoundVolume(0.01f);
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

            // Send the updated positions and visibility to the other instance if network is enabled
            if (isNetworkEnabled && client) {
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
            drivingSound->setSoundVolume(0.01f);
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

            // Send the updated positions and visibility to the other instance if network is enabled
            if (isNetworkEnabled && client) {
                NetMsgCarMovement msg;
                msg.car_other_sidePosition = Aftr::Vector(newPos.x - 12, newPos.y, newPos.z);
                msg.car_turnPosition = Aftr::Vector(newPos.x + 12, newPos.y, newPos.z);
                msg.car_testPosition = Aftr::Vector(newPos.x, newPos.y, newPos.z);
                msg.car_newPosition = Aftr::Vector(newPos.x, newPos.y + 12, newPos.z);
                client->sendNetMsgSynchronousTCP(msg);

                NetMsgCarVisibility visibilityMsg;
                visibilityMsg.carName = "car_other_side";
                client->sendNetMsgSynchronousTCP(visibilityMsg);
            }
        }

        if (key.keysym.sym == SDLK_LEFT) {
            car1->setPosition(Vector(0, 0, -20));
            car3->setPosition(Vector(0, 0, -20));
            drivingSound->setSoundVolume(0.01f);
            drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
            car_test->isVisible = false;
            car_turn->isVisible = false;
            car_other_side->isVisible = false;
            car_new->isVisible = true;
            car1->isVisible = false;
            car3->isVisible = false;

            

            physx::PxVec3 forward = car_test->getForwardVector();
            physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + forward * moveAmount;
            car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
            car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
            car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
            car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

            // Send the updated positions and visibility to the other instance if network is enabled
            if (isNetworkEnabled && client) {
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
            drivingSound->setSoundVolume(0.01f);
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

            // Send the updated positions and visibility to the other instance if network is enabled
            if (isNetworkEnabled && client) {
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
            drivingSound->setSoundVolume(0.01f);
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
            if (isNetworkEnabled && client) {
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
            drivingSound->setSoundVolume(0.01f);
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
            if (isNetworkEnabled && client) {
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
            car2->setPosition(Vector(0, 0, -20));
            drivingSound->setSoundVolume(0.01f);
            drivingSound->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Driving.wav").c_str(), true);
            carMain->isVisible = false;
            carRight->isVisible = false;
            carLeft->isVisible = false;
            carDown->isVisible = true;
            car2->isVisible = false;

            physx::PxVec3 forward = carMain->getForwardVector();
            physx::PxVec3 newPos = carMain->getRigidDynamic()->getGlobalPose().p + forward * moveAmount;
            carRight->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
            carMain->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
            carLeft->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
            carDown->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));

            // Send the updated positions and visibility to the other instance
            if (isNetworkEnabled && client) {
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
            drivingSound->setSoundVolume(0.01f);
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
            if (isNetworkEnabled && client) {
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
            active_keys[SDLK_1] = !active_keys[SDLK_1];
        }
        if (key.keysym.sym == SDLK_4)
        {
            active_keys[SDLK_4] = !active_keys[SDLK_4];
        }
        if (key.keysym.sym == SDLK_3)
        {
            active_keys[SDLK_3] = !active_keys[SDLK_3];
        }
        if (key.keysym.sym == SDLK_2)
        {
            active_keys[SDLK_2] = !active_keys[SDLK_2];
        }
        if (key.keysym.sym == SDLK_5)
        {
            active_keys[SDLK_5] = !active_keys[SDLK_5];
        }
        if (key.keysym.sym == SDLK_6)
        {
            active_keys[SDLK_6] = !active_keys[SDLK_6];
        }
        // Handle 0 key press to increment the lap number
        if (key.keysym.sym == SDLK_LSHIFT) {
            lapNumber++;
            if (lapNumber >= 3) {
                 //handle additional logic when the player wins
                std::cout << "Player 2 Wins!" << std::endl;
            }
        }
        // Handle 0 key press to increment the lap number
        if (key.keysym.sym == SDLK_RSHIFT) {
            lapNumber++;
            if (lapNumber >= 3) {
                 //handle additional logic when the player wins
                std::cout << "Player 1 Wins!" << std::endl;
            }
        }
        if (key.keysym.sym == SDLK_r) {
            lapNumber = 0;
            std::cout << "Lap counter reset to 0" << std::endl;
        }

        if (key.keysym.sym == SDLK_m) {
            soundEngine->stopAllSounds();
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

        OtherCarSkin2();
        car1->isVisible = false;
        spawnPlayer1();
        OtherCarSkin1();
        

        hideAllCars();
        OtherCarSkin3();
        hideAllCars();
        spawnPlayer2();
        car2->isVisible = false;
        hideAllCars2();



        // Load and play the start screen soundtrack
        startScreenSoundtrack = soundEngine->play2D(
            (ManagerEnvironmentConfiguration::getLMM() + "/sounds/StartMenu.wav").c_str(),
            true,  // loop
            false, // stream
            true   // start paused
        );

        // Set the volume to a very low level (e.g., 0.1)
        if (startScreenSoundtrack) {
            startScreenSoundtrack->setVolume(0.1f); // Volume range is from 0.0f (mute) to 1.0f (full volume)
        }

        // Resume playback if needed
        if (startScreenSoundtrack) {
            startScreenSoundtrack->setIsPaused(false);
        }

        // Declare a boolean to track the window size state

        WOImGui* gui = WOImGui::New(nullptr);
        gui->setLabel("Switch Terrain");
        gui->subscribe_drawImGuiWidget([this, gui]() {

            ImVec2 displaySize = ImGui::GetIO().DisplaySize;

           
            ImGuiStyle& style = ImGui::GetStyle();
            style.Colors[ImGuiCol_WindowBg] = ImVec4(0.0f, 0.0f, 0.3f, 1.0f); // Darker blue 
            style.Colors[ImGuiCol_Button] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f); // Black
            style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.2f, 0.2f, 0.2f, 1.0f); // Dark gray 
            style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.1f, 0.1f, 0.1f, 1.0f); // Lighter black 
            style.Colors[ImGuiCol_Text] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // White text

            // Calculate the elapsed time since the loading started
            Uint32 currentTime = SDL_GetTicks();
            Uint32 elapsedTime = currentTime - loadingStartTime;
            float volumeLevel = 0.0f; // Volume starts at zero

            // Handle the different game states
            if (gameState == START_SCREEN) {
                // Show start screen
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(displaySize.x, displaySize.y), ImGuiCond_Always);
                ImGui::Begin("Start Game", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);


                ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 50.0f, displaySize.y - 500.0f));

                ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[1]); 

                // Scale the text to make it larger
                ImGui::SetWindowFontScale(2.0f);

               
                ImGui::Text("SpeedRacer");

                // Reset the font scaling back to normal
                ImGui::SetWindowFontScale(1.0f);

                ImGui::PopFont();


                // Display the start image
                if (startImage) {
                    ImGui::Image((void*)(intptr_t)startImage, displaySize);
                }
                else {
                    ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 50.0f, displaySize.y - 100.0f));
                    ImGui::Text("   Press Space Bar to Start");
                }

               
                ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 60.0f, displaySize.y - 80.0f)); 

               
                ImGui::PushItemWidth(250.0f); 
                ImGui::SliderFloat("Volume", &volumeLevel, 0.0f, 1.0f); // Slider to control volume
                ImGui::PopItemWidth();

              
                ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 40.0f, displaySize.y - 60.0f)); // Adjust to position below the slider

                // Set the button width and height
                ImVec2 buttonSize(225, 0); 

                // Button to toggle network messaging between Single Player and Multiplayer
                if (isNetworkEnabled) {
                    if (ImGui::Button("Multiplayer", buttonSize)) {
                        isNetworkEnabled = false; // Switch to single player mode
                    }
                }
                else {
                    if (ImGui::Button("Single Player", buttonSize)) {
                        isNetworkEnabled = true; // Switch to multiplayer mode
                    }
                }

                // Display the current state of the network messaging below the button
                ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 40.0f, displaySize.y - 40.0f)); 
                ImGui::Text("  Network Messaging: %s", isNetworkEnabled ? "Enabled" : "Disabled");

                if (startScreenSoundtrack) {
                    startScreenSoundtrack->setVolume(volumeLevel); // Update volume based on slider value
                }

                ImGui::End();
            

                // Check if the space bar is pressed
                const Uint8* state = SDL_GetKeyboardState(nullptr);
                if (state[SDL_SCANCODE_SPACE]) {
                    startLoadingProcess();
                    soundEngine->stopAllSounds();
                    // Send the NetMsg for the transition
                    NetMsgStartToLoading msg;
                    client->sendNetMsgSynchronousTCP(msg);
                }


                return; 
            }
            else if (gameState == LOADING) {
                if (elapsedTime < 20000) {
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
                    return; 
                }
                else if (isTerrainLoaded() && otherInstanceTerrainLoaded) {
                   
                    gameState = MAIN_GUI;
                    isLoading = false;
                }
            }
            if (showBlackScreen) {
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(displaySize.x, displaySize.y), ImGuiCond_Always);
                ImGui::Begin("Background", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
                ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(0, 0), displaySize, IM_COL32(0, 0, 0, 255));
                ImGui::SetCursorPos(ImVec2(displaySize.x / 2.0f - 100.0f, displaySize.y - 100.0f));
                ImGui::Text("Please Select a Race Track");
                ImGui::End();
            }
                // Check if the space bar is pressed
                const Uint8* state = SDL_GetKeyboardState(nullptr);
                if (state[SDL_SCANCODE_SPACE]) {
                    this->spawnPlayer1();
                    active_keys[SDLK_1] = !active_keys[SDLK_1];
                }
            
            ImGui::Begin("Racecar Game Control Panel");
            if (ImGui::Button("Toggle Network Messaging")) {
                isNetworkEnabled = !isNetworkEnabled;
            }

            // Display the current state of the network messaging
            ImGui::Text("Network Messaging: %s", isNetworkEnabled ? "Enabled" : "Disabled");
            ImGui::Separator();
            if (ImGui::Button("Start Race")) {
                // Play the start race sound
                soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/start_race.wav").c_str(), false);

            }
            ImGui::Text("Lap: %d/3", lapNumber);
            ImGui::Separator();
            if (ImGui::SliderFloat("Car Speed", &moveAmount, 0.1f, 10.0f)) {
            }
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Game Modes")) {
                ImGui::Text("Time Trials Mode");
                if (ImGui::Button(isTimerRunning ? "Pause Timer" : "Start Timer")) {
                    if (isTimerRunning) {
                        // Pause the timer
                        isTimerRunning = false;
                        pausedTime = SDL_GetTicks() - timerStartTime;

                        // Create and send NetMsgTimerControl with timer paused
                        NetMsgTimerControl msg;
                        msg.isTimerRunning = false;
                        if (isNetworkEnabled) {
                            client->sendNetMsgSynchronousTCP(msg);
                        }
                    }
                    else {
                        // Resume or start the timer
                        timerStartTime = SDL_GetTicks() - pausedTime;
                        isTimerRunning = true;
                        pausedTime = 0;

                        // Create and send NetMsgTimerControl with timer running
                        NetMsgTimerControl msg;
                        msg.isTimerRunning = true;
                        if (isNetworkEnabled) {
                            client->sendNetMsgSynchronousTCP(msg);
                        }
                    }
                }

                // Display the reset time slider
                ImGui::SliderInt("Reset Time (s)", &resetTime, 10, 60);

                // Display the timer
                if (isTimerRunning) {
                    float elapsedTime = (SDL_GetTicks() - timerStartTime) / 1000.0f;
                    ImGui::Text("Timer: %.2f seconds", elapsedTime);

                    // Check if the elapsed time exceeds resetTime
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
            static float backgroundVolume = 0.3f;
            static float backgroundEngine = 0.01f;
            static bool wasSwitchToAnotherGridPressed = false;
            static bool wasSwitchToDefaultGridPressed = false;
            if (ImGui::CollapsingHeader("Racetrack Map Selection")) {
                bool isSwitchToAnotherGridPressed = ImGui::Button("Track 1");
                if (isSwitchToAnotherGridPressed && !wasSwitchToAnotherGridPressed) {
                    // Reset loading flag for Terrain 1
                    terrain1Loaded = false;
                    terrainGridLoaded = false; // Reset terrain grid loading flag
                    gameState = LOADING;
                    loadingStartTime = SDL_GetTicks();

                    // Switch to another grid 
                    this->switchTerrain(true);
                    this->moveTerrainDown(90.0f);
                    this->rotateTerrain(-0.261799f * 6);
                    this->moveTerrainNegativeX(50.0f * 11);
                    this->moveTerrainPositiveX(40.0f);

                   
                    soundEngine->stopAllSounds();
                    //soundEngine->setSoundVolume(0.5f);
                    //soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track1.wav").c_str(), true);

                  
                   

                    // Reset flag for the default grid
                    wasSwitchToDefaultGridPressed = false;
                    showBlackScreen = false; 

                    // Send the network message to remove the black screen
                    NetMsgBlackScreen msg;
                    if (isNetworkEnabled) {
                        client->sendNetMsgSynchronousTCP(msg);
                    }
                   
                    // Send terrain change message
                    sendTerrainChangeMessage(true, 90.0f, -0.261799f * 6, 50.0f * 11, 40.0f);

                    // Set the loading state
                    isLoading = true;
                   
                    active_keys[SDLK_1] = !active_keys[SDLK_1];
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
                    this->moveTerrainPositiveX(40.0f);

                   
                    soundEngine->stopAllSounds();
                    //soundEngine->setSoundVolume(0.5f);
                    //soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/Track3.wav").c_str(), true);

                    // Set the camera position to the Z axis 500 positive
                    this->cam->setPosition(0.0f, 0.0f, -500.0f);

                    // Reset the flag for the another grid
                    wasSwitchToAnotherGridPressed = false;

                    sendTerrainChangeMessage(false, 90.0f, -0.261799f * 6, 50.0f * 11, 40.0f);

                    // Set the loading state to wait for terrain to load
                    isLoading = true;
                }
                wasSwitchToDefaultGridPressed = isSwitchToDefaultGridPressed;
            }
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Player 1 Controls")) {


                // Define the available car skins
                static const char* carSkins[] = { "Dodge", "Ford", "Sports Car", "CyberTrunk" };
                static int selectedSkin = 0;

                ImGui::Combo("Select Car Skin", &selectedSkin, carSkins, IM_ARRAYSIZE(carSkins));

                if (ImGui::Button("Switch Player 1 Car")) {
                    if (selectedSkin == 0) {
                        hideAllCars();
                        this->spawnPlayer1(); // Switch to Dodge skin 
                    }
                    else if (selectedSkin == 1) {
                        hideAllCars();
                        this->OtherCarSkin1(); // Switch to Ford skin 
                    }
                    else if (selectedSkin == 2) {
                        hideAllCars();
                        this->OtherCarSkin2(); // Switch to Sports Car skin 
                    }
                    else if (selectedSkin == 3) {
                        hideAllCars();
                        this->OtherCarSkin3(); // Switch to CyberTrunk skin 
                    }

                   
                    followCar1 = true; // This flag makes the camera follow Player 1

                    // Send network message to switch car skin in other instances
                    if (isNetworkEnabled && client) {
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

                // Timer for Player 1 Speed Boost
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

                    // Automatically switch camera view to follow Player 2
                    followCar1 = false; // This flag makes the camera follow Player 2 (or disable following Player 1)

                    // Send network message to switch car skin in other instances
                    if (isNetworkEnabled && client) {
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
                if (ImGui::SliderFloat("Car Engine Volume", &backgroundEngine, 0.0f, 1.0f)) {
                    drivingSound->setSoundVolume(backgroundEngine);
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
        }
        else {
            createGrid();
            switchSkyBox(0); // Switch to the first skybox
        }

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
            worldLst->push_back(WOSkyBox::New(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg", this->getCameraPtrPtr()));
        }
        else if (index == 1)
        {
            worldLst->push_back(WOSkyBox::New(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg", this->getCameraPtrPtr()));
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
    void GLViewSpeedRacer::moveTerrainPositiveX(float amount) {
        for (WO* wo : *worldLst) {
            if (wo->getLabel() == "terrain" || wo->getLabel() == "anotherTerrain") {
                // Move the terrain along the positive X-axis by the specified amount
                Vector currentPos = wo->getPosition();
                wo->setPosition(currentPos.x + amount, currentPos.y, currentPos.z);
                std::cout << "Terrain moved along positive X-axis by " << 12.0f << ". New X position: " << currentPos.x + 12.0f << std::endl;
                break;
            }
        }
    }

    void GLViewSpeedRacer::spawnPlayer1() {
        std::string Car_Up(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeRight.fbx");
        std::string Car_Right(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDogde.fbx");
        std::string Car_Left(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeDown.fbx");
        std::string Car_Down(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeNew.fbx"); // New car model path
        std::string Car11(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeNew.fbx");
        car_test = Car::New(Car_Up, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
        car_test->setPos(Vector(0, 0, 0));
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
        car_new->isVisible = false;
        worldLst->push_back(car_new);

      
        car1 = WO::New(Car11, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT);
        car1->setPosition(Vector(12, 0, 0));
        car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car1->isVisible = true;
        car1->setLabel("car1");
        worldLst->push_back(car1);
    }
    void GLViewSpeedRacer::OtherCarSkin1() {
        std::string Car_Up(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/FordCarDirection.dae");
        std::string Car_Right(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/LowPoly Muscle Cougar xr1970.dae");
        std::string Car_Left(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionOp.dae");
        std::string Car_Down(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae");

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
        car_new->isVisible = false;
        worldLst->push_back(car_new);

        std::string Car13 = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae";
        car3 = WO::New(Car13, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT);
        car3->setPosition(Vector(0, 0, 5));
        car3->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car3->isVisible = true;
        car3->setLabel("car3");
        worldLst->push_back(car3);

    }
    void GLViewSpeedRacer::OtherCarSkin2() {
        std::string Car_Up(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestRight.fbx");
        std::string Car_Right(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTest.fbx");
        std::string Car_Left(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestLeft.fbx");
        std::string Car_Down(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestOtherWay.fbx");

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
        car_new->isVisible = false;
        worldLst->push_back(car_new);

        std::string Car11(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestOtherWay.fbx");
        car1 = WO::New(Car11, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT);
        car1->setPosition(Vector(12, 0, 0));
        car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car1->isVisible = true;
        car1->setLabel("car1");
        worldLst->push_back(car1);

    }
    void GLViewSpeedRacer::OtherCarSkin3() {
        std::string Car_Up(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkFront.fbx");
        std::string Car_Right(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkLeft.fbx");
        std::string Car_Left(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkBack.fbx");
        std::string Car_Down(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkRight.fbx");

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
        car_new->isVisible = false;
        worldLst->push_back(car_new);

        std::string Car11(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkRight.fbx");
        car1 = WO::New(Car11, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT);
        car1->setPosition(Vector(12, 0, 0));
        car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car1->isVisible = true;
        car1->setLabel("car1");
        worldLst->push_back(car1);
    }
    void GLViewSpeedRacer::spawnPlayer2() {
        std::string carModelUp(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeRight.fbx");
        std::string carModelRight(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDogde.fbx");
        std::string carModelLeft(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeDown.fbx");
        std::string carModelDown(ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeNew.fbx");


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
        carDown->isVisible = false;
        worldLst->push_back(carDown);

        std::string Car22 = ManagerEnvironmentConfiguration::getLMM() + "/models/Dodge/CarDodgeNew.fbx";
        car2 = WO::New(Car22, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT);
        car2->setPosition(Vector(12, 0, 0));
        car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car2->isVisible = true;
        car2->setLabel("car2");
        worldLst->push_back(car2);
    }

    void GLViewSpeedRacer::spawnPlayer2Skin1() {

        std::string carModelUp(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/FordCarDirection.dae");
        std::string carModelRight(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/LowPoly Muscle Cougar xr1970.dae");
        std::string carModelLeft(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionOp.dae");
        std::string carModelDown(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae");

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
        carDown->isVisible = false;
        worldLst->push_back(carDown);

        std::string Car22(ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae");
        car2 = WO::New(Car22, Vector(20, 20, 20), MESH_SHADING_TYPE::mstFLAT);
        car2->setPosition(Vector(12, 0, 0));
        car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car2->isVisible = true;
        car2->setLabel("car2");
        worldLst->push_back(car2);
    }

    void GLViewSpeedRacer::spawnPlayer2Skin2() {

        std::string carModelUp(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestRight.fbx");
        std::string carModelRight(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTest.fbx");
        std::string carModelLeft(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestLeft.fbx");
        std::string carModelDown(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestOtherWay.fbx");

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
        std::string Car23(ManagerEnvironmentConfiguration::getLMM() + "/models/SportCar/CarTestOtherWay.fbx");
        car2 = WO::New(Car23, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT);
        car2->setPosition(Vector(12, 0, 0));
        car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car2->isVisible = true;
        car2->setLabel("car2");
        worldLst->push_back(car2);
    }

    void GLViewSpeedRacer::spawnPlayer2Skin3() {
        std::string carModelUp(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkFront.fbx");
        std::string carModelRight(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkLeft.fbx");
        std::string carModelLeft(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkBack.fbx");
        std::string carModelDown(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkRight.fbx");


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
        carDown->isVisible = false;
        worldLst->push_back(carDown);

        std::string Car22(ManagerEnvironmentConfiguration::getLMM() + "/models/CyberTrunk/CyberTrunkRight.fbx");
        car2 = WO::New(Car22, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT);
        car2->setPosition(Vector(12, 0, 0));
        car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        car2->isVisible = true;
        car2->setLabel("car2");
        worldLst->push_back(car2);
    }


    void GLViewSpeedRacer::resetCarPosition() {
        // Set the car's position to the initial position
        car_test->setPos(initialCarPosition);
        car_turn->setPos(Aftr::Vector(initialCarPosition.x + 12, initialCarPosition.y, initialCarPosition.z));
        car_other_side->setPos(Aftr::Vector(initialCarPosition.x - 12, initialCarPosition.y, initialCarPosition.z));
        car_new->setPos(Aftr::Vector(initialCarPosition.x, initialCarPosition.y + 12, initialCarPosition.z));
        car1->setPosition(Vector(12, 0, 0));

        // Set the new car positions
        carMain->setPos(Aftr::Vector(initialCarPosition.x, initialCarPosition.y, initialCarPosition.z + 5));
        carRight->setPos(Aftr::Vector(initialCarPosition.x + 5, initialCarPosition.y, initialCarPosition.z + 5));
        carLeft->setPos(Aftr::Vector(initialCarPosition.x - 5, initialCarPosition.y, initialCarPosition.z + 5));
        carDown->setPos(Aftr::Vector(initialCarPosition.x, initialCarPosition.y, initialCarPosition.z + 5));
        car2->setPosition(Vector(12, 0, 0));

        // Reset visibility states - Make all objects invisible
        hideAllCarObjects();

        // Optionally: Make some specific cars visible again
        car1->isVisible = false;
        car2->isVisible = false; // Set to false initially, will turn visible when needed
    }

    void GLViewSpeedRacer::hideAllCarObjects() {
        // Make all car objects invisible
        car_test->isVisible = false;
        car_turn->isVisible = false;
        car_other_side->isVisible = false;
        car_new->isVisible = false;
        carMain->isVisible = false;
        carRight->isVisible = false;
        carLeft->isVisible = false;
        carDown->isVisible = false;
        car1->isVisible = false;
        car2->isVisible = false;
    }

    void GLViewSpeedRacer::updateActiveKeys(SDL_KeyCode keycode, bool state)
    {
        active_keys[keycode] = state;
    }
    void GLViewSpeedRacer::updateCamera()
    {
        if (active_keys[SDLK_1])
        {
            this->getCamera()->moveRelative(Vector(0, 0, 1) * this->getCamera()->getCameraVelocity() * 75); // Increased speed by a factor of 5
        }

        if (active_keys[SDLK_4])
        {
            this->getCamera()->moveRelative(Vector(0, -1, 0) * this->getCamera()->getCameraVelocity() * 75);
        }
        if (active_keys[SDLK_5])
        {
            this->getCamera()->moveRelative(Vector(0, 1, 0) * this->getCamera()->getCameraVelocity() * 75);
        }
        if (active_keys[SDLK_3])
        {
            this->getCamera()->moveRelative(Vector(1, 0, 0) * this->getCamera()->getCameraVelocity() * 75); // Increased speed by a factor of 5
        }

        if (active_keys[SDLK_2])
        {
            this->getCamera()->moveRelative(Vector(-1, 0, 0) * this->getCamera()->getCameraVelocity() * 75);
        }
        if (active_keys[SDLK_6]) 
        {
            // Move the camera in the -X direction
            this->getCamera()->moveRelative(Vector(-1, 0, 0) * this->getCamera()->getCameraVelocity() * 100);

            // Rotate the camera to look down the -Z axis
            this->getCamera()->setCameraLookAtPoint(this->getCamera()->getPosition() + Vector(0, 0, -1));

            // Move the camera in the +Z direction with increased speed
            this->getCamera()->moveRelative(Vector(0, 0, 1) * this->getCamera()->getCameraVelocity() * 200);
        }



    }
    void GLViewSpeedRacer::sendTerrainChangeMessage(bool useAnotherGrid, float moveDownAmount, float rotateAmount, float moveNegativeXAmount, float movePositiveXAmount) {
        if (!isNetworkEnabled) {
            return;
        }

        // Existing message setup...
        NetMsgSwitchTerrain msg;
        msg.useAnotherGrid = useAnotherGrid;
        msg.moveDownAmount = moveDownAmount;
        msg.rotateAmount = rotateAmount;
        msg.moveNegativeXAmount = moveNegativeXAmount;
        msg.movePositiveXAmount = movePositiveXAmount;

        // Send network message
        client->sendNetMsgSynchronousTCP(msg);

        // Send the terrain loaded message after the terrain is loaded
        if (isTerrainLoaded()) {
            NetMsgTerrainLoaded terrainLoadedMsg;
            client->sendNetMsgSynchronousTCP(terrainLoadedMsg);
        }
    }
    bool GLViewSpeedRacer::isTerrainLoaded() {
        return terrain1Loaded || terrain2Loaded;
    }
    void GLViewSpeedRacer::hideAllCars() {
        if (car_test) car_test->isVisible = false;
        if (car_turn) car_turn->isVisible = false;
        if (car_other_side) car_other_side->isVisible = false;
        if (car_new) car_new->isVisible = false;
        if (car1) car1->isVisible = false;
        if (car3) car3->isVisible = false;
    }
    void GLViewSpeedRacer::hideAllCars2() {
        if (carMain) carMain->isVisible = false;
        if (carRight) carRight->isVisible = false;
        if (carLeft) carLeft->isVisible = false;
        if (carDown) carDown->isVisible = false;
        if (car2) car2->isVisible = false;
    }
    void GLViewSpeedRacer::startLoadingProcess() {
        // Transition to the loading screen
        gameState = LOADING;
        loadingStartTime = SDL_GetTicks();
        isLoading = true;

    }
}
