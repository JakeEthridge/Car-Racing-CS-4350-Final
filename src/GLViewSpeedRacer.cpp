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


void GLViewSpeedRacer::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   // car accelerator
   if( key.keysym.sym == SDLK_w )
   {
       carSelection();
   }
   // move car left
   if (key.keysym.sym == SDLK_a) {

   }
   //move car right
   if (key.keysym.sym == SDLK_d) {

   }
   // car reverses
   if (key.keysym.sym == SDLK_s) {

   }

}


void GLViewSpeedRacer::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
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

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
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
   // Inside GLViewSpeedRacer::loadMap()
   // Path to the terrain image
   std::string terrainImagePath = ManagerEnvironmentConfiguration::getSMM() + "/images/Woodland.bmp";

   // Load the terrain image file
   FILE* file = std::fopen(terrainImagePath.c_str(), "rb");
   if (!file) {
       std::fprintf(stderr, "Failed to open terrain image file: %s\n", terrainImagePath.c_str());
       return;
   }

   // Load image data from the file
   int width, height, channels;
   unsigned char* imgData = stbi_load_from_file(file, &width, &height, &channels, 0);
   if (imgData == nullptr) {
       std::fprintf(stderr, "Failed to load terrain image\n");
       std::fclose(file);
       return;
   }

   // Create an OpenGL texture for the terrain
   GLuint terrainTextureID;
   glGenTextures(1, &terrainTextureID);
   glBindTexture(GL_TEXTURE_2D, terrainTextureID);

   // Set texture parameters
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   // Upload the texture data from the image
   GLenum format = (channels == 4) ? GL_RGBA : GL_RGB;
   glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, imgData);

   // Generate mipmaps
   glGenerateMipmap(GL_TEXTURE_2D);

   // Unbind the texture
   glBindTexture(GL_TEXTURE_2D, 0);

   // Free the image data
   stbi_image_free(imgData);

   // Close the file
   std::fclose(file);



   //std::string cars("../../../modules/SpeedRacer/mm/models/porsche/Porsche_935_2019.obj");
   //car3 = WO::New(cars, Vector(0.01, 0.01, 0.01));
   //car3->setPosition(Vector(0, 0, 1));
   //car3->isVisible = true;
   //car3->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //car3->setLabel("Car2");
   //worldLst->push_back(car3);
   //actorLst->push_back(car3);

   std::string car(ManagerEnvironmentConfiguration::getSMM() + "/models/model.dae");
   car1 = WO::New(car, Vector(1, 1, 1));
   car1->setPosition(Vector(0, 0, 1));
   car1->isVisible = true;
   car1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car1->setLabel("Car1");
   worldLst->push_back(car1);
   actorLst->push_back(car1);


   // import porsche
   WO* porsche = WO::New("../../../modules/SpeedRacer/mm/models/porsche/Porsche_935_2019.obj");
   porsche->setPosition(Vector(10, 10, 3)); // start position will depend on terrain
   porsche->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   porsche->setLabel("Porsche 911");
   carModels.push_back(porsche);
   

   std::string cars2(ManagerEnvironmentConfiguration::getSMM() + "/models/Ferrari.dae");
   car2 = WO::New(cars2, Vector(1, 1, 1));
   car2->setPosition(Vector(0, 0, 1));
   car2->isVisible = true;
   car2->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   car2->setLabel("Car3");
   worldLst->push_back(car2);
   actorLst->push_back(car2);

   
   //worldLst->push_back(carModels[0]);
   
   //Make a Dear Im Gui instance via the WOImGui in the engine... This calls
   //the default Dear ImGui demo that shows all the features... To create your own,
   //inherit from WOImGui and override WOImGui::drawImGui_for_this_frame(...) (among any others you need).
   WOImGui* gui = WOImGui::New( nullptr );
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
       [this, gui]() {
           static WO* focus = car1; // Set the initial focus to car1
           ImVec4 bgColor = ImVec4(0.2f, 0.2f, 0.2f, 1.0f); // Background color for the GUI
           if (ImGui::Begin("Car Switch")) {
               const char* items[] = { "Car1", "Car2", "Car3" }; // List of items including Car3
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
                           }
                           else if (i == 1) {
                               focus = car2;
                               car1->isVisible = false;
                               car2->isVisible = true;
                               car3->isVisible = false;
                           }
                           else if (i == 2) {
                               focus = car3;
                               car1->isVisible = false;
                               car2->isVisible = false;
                               car3->isVisible = true;
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
 

   createSpeedRacerWayPoints();
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