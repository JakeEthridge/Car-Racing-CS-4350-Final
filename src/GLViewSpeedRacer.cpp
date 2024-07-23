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

using namespace Aftr;

using namespace irrklang;

GLViewSpeedRacer* GLViewSpeedRacer::New( const std::vector< std::string >& args )
{
   GLViewSpeedRacer* glv = new GLViewSpeedRacer( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewSpeedRacer::GLViewSpeedRacer( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewSpeedRacer::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewSpeedRacer::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewSpeedRacer::onCreate()
{
   //GLViewSpeedRacer::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}


GLViewSpeedRacer::~GLViewSpeedRacer()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewSpeedRacer::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   this->cam->setPosition(car1->getPosition() + Vector(-15, 1, 5));
 
   
}


void GLViewSpeedRacer::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewSpeedRacer::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewSpeedRacer::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewSpeedRacer::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}

NetMessengerClient* client;
void GLViewSpeedRacer::onKeyDown(const SDL_KeyboardEvent& key) {
    GLView::onKeyDown(key);
    if (key.keysym.sym == SDLK_r) { // Reset camera position
        // Get the car's position
        Vector carPosition = car1->getPosition();

        // Get the car's look direction
        Vector carLookDirection = car1->getLookDirection();

        // Define the camera offset relative to the car's position
        Vector cameraOffsetRelativeToCar(10, -10, -10); // Adjust as needed

        // Compute the camera position based on the car's position, look direction, and offset
        Vector cameraPosition = carPosition - carLookDirection * cameraOffsetRelativeToCar;

        // Set the camera position
        this->cam->setPosition(cameraPosition);
        this->cam->rotateAboutGlobalZ(90.0f);
    }


    float moveSpeed = 1.0f; // Adjust as needed
    float turnAngle = 0.1f; // Adjust as needed

    // Set the movement direction based on the pressed keys
    Vector movement(0, 0, 0);

    // Flag variables to control message sending
    bool sendMovementMessage = false;
    bool sendRotationMessage = false;

    if (key.keysym.sym == SDLK_w) { // Move forwards
        // Move the car in the direction it is facing (its look direction)
        movement += car1->getLookDirection() * moveSpeed;
        car1->moveRelative(movement);
        sendMovementMessage = true;
    }

    // Handle continuous turning and send net messages
    if (key.keysym.sym == SDLK_a) { // Turn left
        // Adjust the rotation of the car counterclockwise (left turn)
        car1->rotateAboutGlobalZ(turnAngle);
        sendRotationMessage = true;
    }
    if (key.keysym.sym == SDLK_d) { // Turn right
        // Adjust the rotation of the car clockwise (right turn)
        car1->rotateAboutGlobalZ(-turnAngle);
        sendRotationMessage = true;
        this->cam->setPosition(car1->getPosition() + Vector(-10, 0, 10));
    }

    
   

    // Send network messages if the corresponding flag is set
    if (sendMovementMessage || sendRotationMessage) {
        NetMsgCreateRawWO* netMsg = new NetMsgCreateRawWO();
        if (sendMovementMessage) {
            // Set movement data in the message
            netMsg->xPos = car1->getPosition().x;
            netMsg->yPos = car1->getPosition().y;
            netMsg->zPos = car1->getPosition().z;
        }
        if (sendRotationMessage) {
            // Set rotation data in the message
            netMsg->rotationZ = sendRotationMessage ? (key.keysym.sym == SDLK_a ? turnAngle : -turnAngle) : 0.0f;
        }
        client->sendNetMsgSynchronousTCP(*netMsg);
        delete netMsg;
    }
}




void GLViewSpeedRacer::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);

    // Stop the continuous turning when 'A' or 'D' keys are released
    if (key.keysym.sym == SDLK_a || key.keysym.sym == SDLK_d)
    {
        // Reset the rotation to stop turning
        car1->rotateAboutGlobalZ(0.0f);
    }

    // Optionally, you can perform additional actions based on key releases here
}



void Aftr::GLViewSpeedRacer::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   client = NetMessengerClient::New("127.0.0.1", ManagerEnvironmentConfiguration::getVariableValue("NetServerTransmitPort"));

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string track("../../../modules/SpeedRacer/mm/models/Formula_Track.fbx");
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
   //skyBoxImageNames.push_back("../../../modules/SpeedRacer/mm/images/cobblestone_street_night_2k.hdr");
   {
      //Create a light
      float ga = 0.1f; //Global Ambient Light level for this module
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
      //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );
   }

   {
      //Create the SkyBox
      WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }

   /*{
       WO* wo = WO::New(track, Vector(3, 3, 3), MESH_SHADING_TYPE::mstFLAT);
       wo->setPosition(Vector(0, 0, 0));
       wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
       worldLst->push_back(wo);
   }*/

   /*{
       WONVPhysX* physTrack = WONVPhysX::New(track, Vector(3, 3, 3), MESH_SHADING_TYPE::mstFLAT);
       physTrack->setPosition(Vector(0, 0, 0));
       physTrack->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
       worldLst->push_back(physTrack);
       PxScene::setGravity();
   }*/
  
   
   // Car Models loaded in

   //std::string cars("../../../modules/SpeedRacer/mm/models/porsche/Porsche_935_2019.obj");
   //std::string car(ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl");
   std::string cars2("../../../modules/SpeedRacer/mm/models/porsche/low_poly_911.dae");
   std::string dodge("../../../modules/SpeedRacer/mm/models/car2/source/dodge.fbx");
   std::string ford("../../../modules/SpeedRacer/mm/models/ford/LowPoly Muscle Cougar xr1970.dae");

   // Car Objects 1 - 4
   
   car1 = WO::New(ford, Vector(1.0, 1.0, 1.0));
   car1->setPosition(Vector(48, -40, -2));
   car1->isVisible = true;
   car1->rotateAboutGlobalZ(-4.60f);
   car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car1->setLabel("Ford Muscle Cougar");
   worldLst->push_back(car1);
   actorLst->push_back(car1);

   car2 = WO::New(dodge, Vector(.1, .1, .1));
   car2->setPosition(Vector(48, -40, -2));
   car2->isVisible = false;
   car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car2->setLabel("Car2");
   worldLst->push_back(car2);
   actorLst->push_back(car2);

   car3 = WO::New(cars2, Vector(1, 1, 1));
   car3->setPosition(Vector(6, 0, 1));
   car3->isVisible = false;
   car3->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car3->setLabel("Car3");
   worldLst->push_back(car3);
   actorLst->push_back(car3);

   car4 = WO::New(cars2, Vector(1, 1, 1));
   car4->setPosition(Vector(4, 0, 1));
   car4->isVisible = false;
   car4->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car4->setLabel("Car4");
   worldLst->push_back(car4);
   actorLst->push_back(car4);

   car4 = WO::New(cars2, Vector(1, 1, 1));
   car4->setPosition(Vector(4, 0, 1));
   car4->isVisible = false;
   car4->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car4->setLabel("Car4");
   worldLst->push_back(car4);
   actorLst->push_back(car4);

   



   // Default song
   engine->play2D("../../../modules/SpeedRacer/mm/sounds/ride.mp3", true);

   //Make a Dear Im Gui instance via the WOImGui in the engine... This calls
   //the default Dear ImGui demo that shows all the features... To create your own,
   //inherit from WOImGui and override WOImGui::drawImGui_for_this_frame(...) (among any others you need).
   WOImGui* gui = WOImGui::New( nullptr );
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
       [this, gui]() {
           static WO* focus = car1; // Set the initial focus to car1
           ImVec4 bgColor = ImVec4(0.2f, 0.2f, 0.2f, 1.0f); // Background color for the GUI
           if (ImGui::Begin("Main Menu")) {
               const char* items[] = { car1->getLabel().c_str(), car2->getLabel().c_str(), car3->getLabel().c_str(), car4->getLabel().c_str() }; // List of items including Car4
               static int item_current_idx = 0; // Index of selected item
               const char* songs[] = { "Ride Wit Me", "Astro", "Tokyo Drift", "Bandolero"};
               static int songs_current_idx = 0; // Index of selected item
               

               if (ImGui::BeginCombo("Select Car", items[item_current_idx])) {
                   for (int i = 0; i < IM_ARRAYSIZE(items); i++) {
                       bool is_selected = (item_current_idx == i);
                       if (ImGui::Selectable(items[i], is_selected)) {
                           item_current_idx = i;
                           // Toggle visibility based on selection
                           if (i == 0) {
                               focus = car1;
                               car1->isVisible = true;
                               car2->isVisible = false;
                               car3->isVisible = false;
                               car4->isVisible = false;
                           }
                           else if (i == 1) {
                               focus = car2;
                               car1->isVisible = false;
                               car2->isVisible = true;
                               car3->isVisible = false;
                               car4->isVisible = false;
                           }
                           else if (i == 2) {
                               focus = car3;
                               car1->isVisible = false;
                               car2->isVisible = false;
                               car3->isVisible = true;
                               car4->isVisible = false;
                           }
                           else if (i == 3) {
                               focus = car4;
                               car1->isVisible = false;
                               car2->isVisible = false;
                               car3->isVisible = false;
                               car4->isVisible = true;
                           }
                       }
                       if (is_selected) {
                           ImGui::SetItemDefaultFocus();
                       }
                   }
                   ImGui::EndCombo();
               }

               ImGui::Spacing(); // Add spacing between combo boxes

               if (ImGui::BeginCombo("Select Music", songs[songs_current_idx])) {

                   for (int i = 0; i < IM_ARRAYSIZE(songs); i++) {
                       bool is_selected = (songs_current_idx == i);
                       if (ImGui::Selectable(songs[i], is_selected)) {
                           songs_current_idx = i;
                           if (i == 0) {
                               engine->stopAllSounds();
                               engine->setSoundVolume(1);
                               engine->play2D("../../../modules/SpeedRacer/mm/sounds/ride.mp3", true);
                           }
                           else if (i == 1) {
                               engine->stopAllSounds();
                               engine->setSoundVolume(.5f);
                               engine->play2D("../../../modules/SpeedRacer/mm/sounds/astro.mp3", true);
                           }
                           else if (i == 2) {
                               engine->stopAllSounds();
                               engine->setSoundVolume(1);
                               engine->play2D("../../../modules/SpeedRacer/mm/sounds/tokyo.mp3", true);
                           }
                           else if (i == 3) {
                               engine->stopAllSounds();
                               engine->setSoundVolume(1);
                               engine->play2D("../../../modules/SpeedRacer/mm/sounds/band.mp3", true);
                           }
                       }
                   }
               ImGui::EndCombo();
               }

               
               if (ImGui::SliderFloat("Volume", &volume, 0.0f, 1.0f)) {
                   engine->setSoundVolume(volume);
               }

               static bool on = true;
               if (ImGui::Button("Stop Music")) { 
                   engine->stopAllSounds();
               }
               ImGui::End();
           }
       });
   this->worldLst->push_back( gui );
 

   //createSpeedRacerWayPoints();
}


void GLViewSpeedRacer::createSpeedRacerWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   WOWayPointSpherical* wayPt = WOWayPointSpherical::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}

void GLViewSpeedRacer::carSelection() { 
    
}