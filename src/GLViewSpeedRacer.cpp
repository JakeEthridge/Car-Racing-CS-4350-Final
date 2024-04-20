#include "GLViewSpeedRacer.h"

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

using namespace Aftr;

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
void GLViewSpeedRacer::onKeyDown(const SDL_KeyboardEvent& key)
{
    GLView::onKeyDown(key);

    float moveSpeed = 1.0f; // Adjust as needed
    float turnAngle = 0.1f; // Adjust as needed

    // Set the movement direction based on the pressed keys
    Vector movement(0, 0, 0);

    if (key.keysym.sym == SDLK_w) // Move forwards
    {
        // Move the car in the direction it is facing (its look direction)
        movement += car1->getLookDirection() * moveSpeed;
        car1->moveRelative(movement);
    }
    if (key.keysym.sym == SDLK_s) // Move backwards
    {
        // Move the car backwards along its look direction
        movement -= car1->getLookDirection() * moveSpeed;
        car1->moveRelative(movement);
    }

    // Handle continuous turning
    if (key.keysym.sym == SDLK_a) // Turn left
    {
        // Rotate the car counterclockwise (left turn) around its vertical axis (z-axis)
        car1->rotateAboutGlobalZ(turnAngle);
    }
    if (key.keysym.sym == SDLK_d) // Turn right
    {
        // Rotate the car clockwise (right turn) around its vertical axis (z-axis)
        car1->rotateAboutGlobalZ(-turnAngle);
    }

    // Optionally, you can perform additional actions based on key presses here
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

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
 

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

   { 
      ////Create the infinite grass plane (the floor)
      WO* wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      wo->upon_async_model_loaded( [wo]()
         {
            ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
            grassSkin.getMultiTextureSet().at( 0 ).setTexRepeats( 5.0f );
            grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
            grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
            grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
            grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
         } );
      wo->setLabel( "Grass" );
      worldLst->push_back( wo );
   }

   // Car Models Loaded in

   //std::string cars("../../../modules/SpeedRacer/mm/models/porsche/Porsche_935_2019.obj");
   //std::string nissan("../../../modules/SpeedRacer/mm/models/nissan/NISSAN-GTR.obj");
   //std::string acura("../../../modules/SpeedRacer/mm/models/acura/Acura_NSX_1997.mtl");
   //std::string ferrari("../../../modules/SpeedRacer/mm/models/ferrrari/uploads_files_3433296_Ferrari+Concept1.mtl");
   std::string porsche("../../../modules/SpeedRacer/mm/models/porsche/low_poly_911.dae");

   // 4 car models loaded in from standard list ~ possibly add more

   car1 = WO::New(porsche, Vector(1, 1, 1));
   car1->setPosition(Vector(5, 0, 1));
   car1->isVisible = true;
   car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car1->setLabel("Nissan");
   worldLst->push_back(car1);
   actorLst->push_back(car1);
   
   car2 = WO::New(porsche, Vector(1, 1, 1));
   car2->setPosition(Vector(0, 0, 1));
   car2->isVisible = false;
   car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car2->setLabel("Porsche 911");
   worldLst->push_back(car2);
   actorLst->push_back(car2);

   car3 = WO::New(porsche, Vector(1, 1, 1));
   car3->setPosition(Vector(8, 0, 1));
   car3->isVisible = false;
   car3->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car3->setLabel("Car3");
   worldLst->push_back(car3);
   actorLst->push_back(car3);

   car4 = WO::New(porsche, Vector(1, 1, 1));
   car4->setPosition(Vector(10, 0, 1));
   car4->isVisible = false;
   car4->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car4->setLabel("Car4");
   worldLst->push_back(car4);
   actorLst->push_back(car4);

   
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