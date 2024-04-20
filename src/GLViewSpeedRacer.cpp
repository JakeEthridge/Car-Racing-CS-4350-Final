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
   
   
   // Car Models loaded in

   //std::string cars("../../../modules/SpeedRacer/mm/models/porsche/Porsche_935_2019.obj");
   //std::string car(ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl");
   std::string cars2("../../../modules/SpeedRacer/mm/models/porsche/low_poly_911.dae");

   // Car Objects 1 - 4

   car1 = WO::New(cars2, Vector(1, 1, 1));
   car1->setPosition(Vector(10, 0, 2));
   car1->isVisible = true;
   car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car1->setLabel("Car1");
   worldLst->push_back(car1);
   actorLst->push_back(car1);

   car2 = WO::New(cars2, Vector(1, 1, 1));
   car2->setPosition(Vector(8, 0, 1));
   car2->isVisible = false;
   car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car2->setLabel("Car3");
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

   std::string RaceTrack(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race1 = WO::New(RaceTrack, Vector(2, 2, 2));
   race1->setPosition(Vector(0, 0, 0));
   race1->isVisible = true;
   race1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race1->setLabel("racetrack");
   worldLst->push_back(race1);
   actorLst->push_back(race1);

   std::string RaceTrack2(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race2 = WO::New(RaceTrack2, Vector(2, 2, 2));
   race2->setPosition(Vector(53, 0, 0));
   race2->isVisible = true;
   race2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race2->setLabel("racetrack");
   worldLst->push_back(race2);
   actorLst->push_back(race2);


   std::string RaceTrack3(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race3 = WO::New(RaceTrack3, Vector(2, 2, 2));
   race3->setPosition(Vector(106, 0, 0));
   race3->isVisible = true;
   race3->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race3->setLabel("racetrack");
   worldLst->push_back(race3);
   actorLst->push_back(race3);

   std::string RaceTrack4(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcorner10x10.wrl");
   race4 = WO::New(RaceTrack4, Vector(2, 2, 2));
   race4->setPosition(Vector(140, 0, 0));
   race4->isVisible = true;
   race4->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race4->setLabel("racetrack");
   // Rotate the object 90 degrees around the global Z-axis
   race4->rotateAboutGlobalZ(300.0f);
   worldLst->push_back(race4);
   actorLst->push_back(race4);

   std::string RaceTrack5(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race5 = WO::New(RaceTrack5, Vector(2, 2, 2));
   race5->setPosition(Vector(139, -35, 0));
   race5->isVisible = true;
   race5->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race5->setLabel("racetrack");
   race5->rotateAboutGlobalZ(300.0f);
   worldLst->push_back(race5);
   actorLst->push_back(race5);


   std::string RaceTrack6(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race6 = WO::New(RaceTrack5, Vector(2, 2, 2));
   race6->setPosition(Vector(138, -88, 0));
   race6->isVisible = true;
   race6->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race6->setLabel("racetrack");
   race6->rotateAboutGlobalZ(300.0f);
   worldLst->push_back(race6);
   actorLst->push_back(race6);


   std::string RaceTrack7(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcorner10x10.wrl");
   race7 = WO::New(RaceTrack7, Vector(2, 2, 2));
   race7->setPosition(Vector(137, -122, 0));
   race7->isVisible = true;
   race7->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race7->setLabel("racetrack");
   // Rotate the object 90 degrees around the global Z-axis
   race7->rotateAboutGlobalZ(600.0f);
   worldLst->push_back(race7);
   actorLst->push_back(race7);
   

   std::string RaceTrack8(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race8 = WO::New(RaceTrack8, Vector(2, 2, 2));
   race8->setPosition(Vector(106, -122, 0));
   race8->isVisible = true;
   race8->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race8->setLabel("racetrack");
   worldLst->push_back(race8);
   actorLst->push_back(race8);

   std::string RaceTrack9(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race9 = WO::New(RaceTrack9, Vector(2, 2, 2));
   race9->setPosition(Vector(53, -122, 0));
   race9->isVisible = true;
   race9->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race9->setLabel("racetrack");
   worldLst->push_back(race9);
   actorLst->push_back(race9);

   std::string RaceTrack10(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race10 = WO::New(RaceTrack10, Vector(2, 2, 2));
   race10->setPosition(Vector(0, -122, 0));
   race10->isVisible = true;
   race10->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race10->setLabel("racetrack");
   worldLst->push_back(race10);
   actorLst->push_back(race10);

   std::string RaceTrack11(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcorner10x10.wrl");
   race11 = WO::New(RaceTrack11, Vector(2, 2, 2));
   race11->setPosition(Vector(-36, -122, 0));
   race11->isVisible = true;
   race11->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race11->setLabel("racetrack");
   race11->rotateAboutGlobalZ(900.0f);
   worldLst->push_back(race11);
   actorLst->push_back(race11);
  

   std::string RaceTrack12(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race12 = WO::New(RaceTrack12, Vector(2, 2, 2)); 
   race12->setPosition(Vector(-35, -88, 0));
   race12->isVisible = true;
   race12->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race12->setLabel("racetrack");
   race12->rotateAboutGlobalZ(300.0f);
   worldLst->push_back(race12);
   actorLst->push_back(race12);

   std::string RaceTrack13(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcenterlane26x10.wrl");
   race13 = WO::New(RaceTrack13, Vector(2, 2, 2));
   race13->setPosition(Vector(-34, -35, 0));
   race13->isVisible = true;
   race13->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race13->setLabel("racetrack");
   race13->rotateAboutGlobalZ(300.0f);
   worldLst->push_back(race13);
   actorLst->push_back(race13);


   std::string RaceTrack14(ManagerEnvironmentConfiguration::getSMM() + "/models/roadcorner10x10.wrl");
   race14 = WO::New(RaceTrack14, Vector(2, 2, 2));
   race14->setPosition(Vector(-33, 1, 0));
   race14->isVisible = true;
   race14->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   race14->setLabel("racetrack");
   race14->rotateAboutGlobalZ(1200.0f);
   worldLst->push_back(race14);
   actorLst->push_back(race14);

   // Define parameters for spawning props in a line
   const int numProps = 7; // Number of props to spawn
   const float propSpacing = 21.0f; // Larger spacing between props in the line

   for (int i = 0; i < numProps; ++i) {
       std::string props(ManagerEnvironmentConfiguration::getSMM() + "/models/generic_medium.obj");
       WO* prop = WO::New(props, Vector(1, 1, 1));
       prop->setPosition(Vector(i * propSpacing, 12.5, 1)); // Adjust X position based on index and spacing
       prop->isVisible = true;
       prop->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
       prop->setLabel("prop_" + std::to_string(i));

       // Upon async model loaded, set the material properties to achieve a grey texture
       prop->upon_async_model_loaded([prop]() {
           auto& meshes = prop->getModel()->getModelDataShared()->getModelMeshes();
           for (auto& mesh : meshes) { // Iterate over all meshes
               for (auto& skin : mesh->getSkins()) { // Iterate over all skins in each mesh
                   // Set material properties for grey texture
                   skin.setAmbient(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey ambient color
                   skin.setDiffuse(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey diffuse color
                   skin.setSpecular(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey specular color
                   skin.setSpecularCoefficient(10); // Specular coefficient
               }
           }
           });
       // Rotate the object 180 degrees around the Y-axis

       worldLst->push_back(prop);
       actorLst->push_back(prop);
   }

   std::string props2(ManagerEnvironmentConfiguration::getSMM() + "/models/generic_medium.obj");
   // Upon async model loaded, set the material properties to achieve a grey texture
   prop2->upon_async_model_loaded([prop2]() {
       auto& meshes = prop2->getModel()->getModelDataShared()->getModelMeshes();
       for (auto& mesh : meshes) { // Iterate over all meshes
           for (auto& skin : mesh->getSkins()) { // Iterate over all skins in each mesh
               // Set material properties for grey texture
               skin.setAmbient(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey ambient color
               skin.setDiffuse(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey diffuse color
               skin.setSpecular(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey specular color
               skin.setSpecularCoefficient(10); // Specular coefficient
           }
       }
       });

   worldLst->push_back(prop2);
   actorLst->push_back(prop2);

   std::string props3(ManagerEnvironmentConfiguration::getSMM() + "/models/generic_medium.obj");
   WO* prop3 = WO::New(props3, Vector(1, 1, 1));
   prop3->setPosition(Vector(-37, 6, 1));
   prop3->isVisible = true;
   prop3->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   prop3->setLabel("prop");

   // Upon async model loaded, set the material properties to achieve a grey texture
   prop3->upon_async_model_loaded([prop3]() {
       auto& meshes = prop3->getModel()->getModelDataShared()->getModelMeshes();
       for (auto& mesh : meshes) { // Iterate over all meshes
           for (auto& skin : mesh->getSkins()) { // Iterate over all skins in each mesh
               // Set material properties for grey texture
               skin.setAmbient(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey ambient color
               skin.setDiffuse(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey diffuse color
               skin.setSpecular(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey specular color
               skin.setSpecularCoefficient(10); // Specular coefficient
           }
       }
       });

   // Rotate the object slightly around the Z-axis
   prop3->rotateAboutGlobalZ(0.8f); // Adjust the angle as needed

   worldLst->push_back(prop3);
   actorLst->push_back(prop3);

   //NOT EVENLY NEXT TO ACH OTHER HAVE TO FIX LATER
   const int numProp1s = 6; // Number of props to spawn
   const float propSpacing1 = 21.0f; // Spacing between props
   for (int i = 0; i < numProp1s; ++i) {
       std::string props4(ManagerEnvironmentConfiguration::getSMM() + "/models/generic_medium.obj");
       WO* prop4 = WO::New(props4, Vector(1, 1, 1));
       prop4->setPosition(Vector(-43, -10 - i * propSpacing1, 1)); // Adjust Y position based on index and spacing
       prop4->isVisible = true;
       prop4->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
       prop4->setLabel("prop_" + std::to_string(i));

       // Upon async model loaded, set the material properties to achieve a grey texture
       prop4->upon_async_model_loaded([prop4]() {
           auto& meshes = prop4->getModel()->getModelDataShared()->getModelMeshes();
           for (auto& mesh : meshes) { // Iterate over all meshes
               for (auto& skin : mesh->getSkins()) { // Iterate over all skins in each mesh
                   // Set material properties for grey texture
                   skin.setAmbient(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey ambient color
                   skin.setDiffuse(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey diffuse color
                   skin.setSpecular(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey specular color
                   skin.setSpecularCoefficient(10); // Specular coefficient
               }
           }
           });

       // Rotate the object slightly around the Z-axis
       prop4->rotateAboutGlobalZ(1.5f); // Adjust the angle as needed

       worldLst->push_back(prop4);
       actorLst->push_back(prop4);
   }


   std::string props5(ManagerEnvironmentConfiguration::getSMM() + "/models/generic_medium.obj");
   WO* prop5 = WO::New(props5, Vector(1, 1, 1));
   prop5->setPosition(Vector(-37.5, -130, 1));
   prop5->isVisible = true;
   prop5->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   prop5->setLabel("prop");

   // Upon async model loaded, set the material properties to achieve a grey texture
   prop5->upon_async_model_loaded([prop5]() {
       auto& meshes = prop5->getModel()->getModelDataShared()->getModelMeshes();
       for (auto& mesh : meshes) { // Iterate over all meshes
           for (auto& skin : mesh->getSkins()) { // Iterate over all skins in each mesh
               // Set material properties for grey texture
               skin.setAmbient(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey ambient color
               skin.setDiffuse(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey diffuse color
               skin.setSpecular(aftrColor4f(0.5f, 0.5f, 0.5f, 1.0f)); // Grey specular color
               skin.setSpecularCoefficient(10); // Specular coefficient
           }
       }
       });

   // Rotate the object slightly around the Z-axis
   prop5->rotateAboutGlobalZ(2.1f); // Adjust the angle as needed

   worldLst->push_back(prop5);
   actorLst->push_back(prop5);
   
   prop5->rotateAboutGlobalZ(2.1f); // Adjust the angle as needed

   worldLst->push_back(prop5);
   actorLst->push_back(prop5);


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