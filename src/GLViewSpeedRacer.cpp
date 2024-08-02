#include "GLViewSpeedRacer.h"
#include "irrKlang.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"
//#include "PxScene.h"

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

    // Get the current time in milliseconds
    Uint32 currentTime = SDL_GetTicks();

    // Initialize the timer if it's not set
    if (timer == 0) {
        timer = currentTime; // Initialize the timer
    }

    // Calculate elapsed time in seconds
    float elapsedTime = (currentTime - timer) / 1000.0f;

    // Check if the elapsed time exceeds resetTime (in seconds) and if the car is not in the initial position
    if (elapsedTime > resetTime) {
        if (car_test->getPosition() != initialCarPosition) {
            // Reset the car position
            resetCarPosition();
        }
        // Reset the timer
        timer = currentTime;
    }
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

        Car* visibleCar = getVisibleCar();
        if (visibleCar) {
            physx::PxTransform carTransform = visibleCar->getRigidDynamic()->getGlobalPose();
            physx::PxVec3 carPosition = carTransform.p;

            // Set the camera offset based on which car is visible
            physx::PxVec3 cameraOffset(0, -20, 5);

            if (visibleCar == car_turn) {
                cameraOffset = physx::PxVec3(0, 20, 5);
            }
            if (visibleCar == carRight) {
                cameraOffset = physx::PxVec3(0, 20, 5);
            }
            if (visibleCar == car_other_side) {
                cameraOffset = physx::PxVec3(-20, 0, 5);
            }
            if (visibleCar == carLeft) {
                cameraOffset = physx::PxVec3(-20, 0, 5);
            }
            if (visibleCar == car_test) {
                cameraOffset = physx::PxVec3(20, 0, 5);
            }
            if (visibleCar == carMain) {
                cameraOffset = physx::PxVec3(20, 0, 5);
            }
            if (visibleCar == car_new) {
                cameraOffset = physx::PxVec3(0, -20, 5);
            }
            if (visibleCar == carDown) {
                cameraOffset = physx::PxVec3(0, -20, 5);
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
    //updateControls();
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


void GLViewSpeedRacer::onKeyDown(const SDL_KeyboardEvent& key) {
    GLView::onKeyDown(key);

    // Handle car visibility and position based on key presses
    if (key.keysym.sym == SDLK_DOWN) {
        car_test->isVisible = true;
        car_turn->isVisible = false;
        car_other_side->isVisible = false;
        car_new->isVisible = false;

        physx::PxVec3 rightVector = car_test->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
        physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount * -1.0f;
        car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));
    }

    if (key.keysym.sym == SDLK_UP) {
        car_test->isVisible = false;
        car_turn->isVisible = false;
        car_other_side->isVisible = true;
        car_new->isVisible = false;

        physx::PxVec3 rightVector = car_test->getRigidDynamic()->getGlobalPose().q.rotate(physx::PxVec3(1, 0, 0));
        physx::PxVec3 newPos = car_test->getRigidDynamic()->getGlobalPose().p + rightVector * moveAmount;
        car_turn->setPos(Aftr::Vector(newPos.x + 12, newPos.y, newPos.z));
        car_test->setPos(Aftr::Vector(newPos.x, newPos.y, newPos.z));
        car_other_side->setPos(Aftr::Vector(newPos.x - 12, newPos.y, newPos.z));
        car_new->setPos(Aftr::Vector(newPos.x, newPos.y + 12, newPos.z));
    }

    if (key.keysym.sym == SDLK_LEFT) {
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
    }

    if (key.keysym.sym == SDLK_RIGHT) {
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
    }
    // Handle car visibility and position based on key presses
    if (key.keysym.sym == SDLK_s) { // Down arrow key
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
    }

    if (key.keysym.sym == SDLK_w) { // Up arrow key
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
    }

    if (key.keysym.sym == SDLK_a) { // Left arrow key
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
    }

    if (key.keysym.sym == SDLK_d) { // Right arrow key
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


}

void GLViewSpeedRacer::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
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
    spawnPlayer2();

    // Default song
    engine->play2D("../../../modules/SpeedRacer/mm/sounds/ride.mp3", true);
   //std::string cars("../../../modules/SpeedRacer/mm/models/porsche/Porsche_935_2019.obj");
   //std::string car(ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl");
   std::string cars2("../../../modules/SpeedRacer/mm/models/porsche/low_poly_911.dae");
   std::string dodge("../../../modules/SpeedRacer/mm/models/car2/source/dodge.fbx");
   std::string ford("../../../modules/SpeedRacer/mm/models/ford/LowPoly Muscle Cougar xr1970.dae");

    // Add sound sources for grid terrain functions
    irrklang::ISoundSource* gridTerrainSound = soundEngine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "/sounds/nature.wav").c_str());
    soundEngine->addSoundSourceAlias(gridTerrainSound, "gridTerrainSound");
    soundList.push_back("gridTerrainSound");

    irrklang::ISoundSource* anotherGridTerrainSound = soundEngine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "/sounds/").c_str());
    soundEngine->addSoundSourceAlias(anotherGridTerrainSound, "anotherGridTerrainSound");
    soundList.push_back("anotherGridTerrainSound");
   // Car Objects 1 - 4
   
   WO* car1 = WO::New(ford, Vector(1.0, 1.0, 1.0));
   car1->setPosition(Vector(48, -40, -2));
   car1->isVisible = true;
   car1->rotateAboutGlobalZ(-4.60f);
   car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car1->setLabel("Ford Muscle Cougar");
   worldLst->push_back(car1);
   actorLst->push_back(car1);

    // Initialize SDL_image
    IMG_Init(IMG_INIT_PNG);
   WO* car2 = WO::New(dodge, Vector(.1, .1, .1));
   car2->setPosition(Vector(48, -40, -2));
   car2->isVisible = false;
   car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car2->setLabel("Car2");
   worldLst->push_back(car2);
   actorLst->push_back(car2);

    // Load images using ManagerEnvironmentConfiguration::getLMM() path
    std::string basePath = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/";
    std::string startImagePath = basePath + "start_image.png";
    std::string loadingImagePath = basePath + "loading_image.png";

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

        // Main GUI code (this part is executed when gameState is MAIN_GUI)
        static float backgroundVolume = 1.0f;
        static bool wasSwitchToAnotherGridPressed = false;
        static bool wasSwitchToDefaultGridPressed = false;

        ImGui::Begin("Switch Terrain");
        bool isSwitchToAnotherGridPressed = ImGui::Button("Terrain 1");
        if (isSwitchToAnotherGridPressed && !wasSwitchToAnotherGridPressed) {
            // Set loading state for Terrain 1
            terrain1Loaded = false; // Reset loading flag
            gameState = LOADING;
            loadingStartTime = SDL_GetTicks();

            // Ensure only "Another Grid" is loaded
            this->switchTerrain(true);

            // Apply additional transformations
            this->moveTerrainDown(90.0f);
            this->rotateTerrain(-0.261799f * 6);
            this->moveTerrainNegativeX(50.0f * 11);

            // Play "sandstorm.wav" after switching to another grid
            soundEngine->stopAllSounds();
            soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/").c_str(), true);

            // Set the camera position to the Z axis -500
            this->cam->setPosition(0.0f, 0.0f, -500.0f);

            // Reset the flag for the default grid
            wasSwitchToDefaultGridPressed = false;

            // Send network message to switch terrain
            if (this->client) {
                NetMsgSwitchTerrain msg;
                msg.useAnotherGrid = true;
                this->client->sendNetMsgSynchronousTCP(msg);
            }

            // Set the loading state to wait for terrain to load
            isLoading = true;
        }
        wasSwitchToAnotherGridPressed = isSwitchToAnotherGridPressed;

        bool isSwitchToDefaultGridPressed = ImGui::Button("Terrain 2");
        if (isSwitchToDefaultGridPressed && !wasSwitchToDefaultGridPressed) {
            // Set loading state for Terrain 2
            terrain2Loaded = false; // Reset loading flag
            gameState = LOADING;
            loadingStartTime = SDL_GetTicks();

            // Ensure only "Default Grid" is loaded
            this->switchTerrain(false);

            // Play "nature.wav" after switching to default grid
            soundEngine->stopAllSounds();
            soundEngine->play2D((ManagerEnvironmentConfiguration::getLMM() + "/sounds/nature.wav").c_str(), true);

            // Set the camera position to the Z axis 500 positive
            this->cam->setPosition(400.0f, 0.0f, 500.0f);

            // Reset the flag for the another grid
            wasSwitchToAnotherGridPressed = false;

            // Send network message to switch terrain
            if (this->client) {
                NetMsgSwitchTerrain msg;
                msg.useAnotherGrid = false;
                this->client->sendNetMsgSynchronousTCP(msg);
            }

            // Set the loading state to wait for terrain to load
            isLoading = true;
        }
        wasSwitchToDefaultGridPressed = isSwitchToDefaultGridPressed;

        if (ImGui::Button("Select Player1")) {
            this->spawnPlayer1();
        }
        if (ImGui::Button("Select Player2")) {
            this->spawnPlayer2();
        }
        if (ImGui::SliderFloat("Car Speed", &moveAmount, 0.1f, 10.0f)) {
            // No additional code needed; moveAmount is directly used in onKeyDown
        }
        if (ImGui::SliderFloat("Volume", &backgroundVolume, 0.0f, 1.0f)) {
            soundEngine->setSoundVolume(backgroundVolume);
        }
        // Timer Display and Reset Time Slider
        ImGui::Text("Timer: %.2f seconds", (SDL_GetTicks() - timer) / 1000.0f);
        if (ImGui::SliderInt("Reset Time (s)", &resetTime, 10, 60)) {
            // No conversion needed; resetTime is already in seconds
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
    std::string elevationPath = ManagerEnvironmentConfiguration::getLMM() + "/images/sandbox_heightmap.tif";
    std::string texturePath = ManagerEnvironmentConfiguration::getLMM() + "/images/sandbox.png";
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
    std::string elevationPath = ManagerEnvironmentConfiguration::getLMM() + "/images/racetrack_heightmap.tiff";
    std::string texturePath = ManagerEnvironmentConfiguration::getLMM() + "/images/racetrack.png";
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
    std::string Car_Down = ManagerEnvironmentConfiguration::getLMM() + "/models/CardodgeLeft.fbx"; // New car model path

    car_test = Car::New(Car_Up, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_test->setPos(Vector(0, 0, 0));
    car_test->setPose(spy_pose);
    car_test->rotateAboutGlobalZ(-4.60f);
    car_test->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(car_test);

    car_turn = Car::New(Car_Right, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_turn->setPos(Vector(0, 0, 0));
    car_turn->setPose(spy_pose);
    car_turn->rotateAboutGlobalZ(-4.60f);
    car_turn->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(car_turn);

    car_other_side = Car::New(Car_Left, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_other_side->setPos(Vector(0, 0, 0)); // Adjust the position to be on the other side of car_test
    car_other_side->setPose(spy_pose);
    car_other_side->rotateAboutGlobalZ(-4.60f);
    car_other_side->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(car_other_side);

    car_new = Car::New(Car_Down, Vector(0.1, 0.1, 0.1), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    car_new->setPos(Vector(0, 0, 0)); // Adjust the position as needed
    car_new->setPose(spy_pose);
    car_new->rotateAboutGlobalZ(-4.60f);
    car_new->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(car_new);

}
void GLViewSpeedRacer::spawnPlayer2() {
    std::string carModelUp = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/FordCarDirection.dae";
    std::string carModelRight = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/LowPoly Muscle Cougar xr1970.dae";
    std::string carModelLeft = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionOp.dae";
    std::string carModelDown = ManagerEnvironmentConfiguration::getLMM() + "/models/ford/CarDirectionDown.dae";

    // Initialize carMain
    carMain = Car::New(carModelUp, Vector(5, 5, 5), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carMain->setPos(Vector(0, 0, 0));
    carMain->setPose(spy_pose);
    carMain->rotateAboutGlobalZ(-4.60f);
    carMain->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(carMain);

    // Initialize carRight
    carRight = Car::New(carModelRight, Vector(5, 5, 5), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carRight->setPos(Vector(0, 0, 0));
    carRight->setPose(spy_pose);
    carRight->rotateAboutGlobalZ(-4.60f);
    carRight->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(carRight);

    // Initialize carLeft
    carLeft = Car::New(carModelLeft, Vector(5, 5, 5), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carLeft->setPos(Vector(0, 0, 0)); // Adjust the position to be on the other side of carMain
    carLeft->setPose(spy_pose);
    carLeft->rotateAboutGlobalZ(-4.60f);
    carLeft->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(carLeft);

    // Initialize carDown
    carDown = Car::New(carModelDown, Vector(5, 5, 5), MESH_SHADING_TYPE::mstFLAT, pxPhysics, pxScene, spy_pose);
    carDown->setPos(Vector(0, 0, 0)); // Adjust the position as needed
    carDown->setPose(spy_pose);
    carDown->rotateAboutGlobalZ(-4.60f);
    carDown->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
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
}