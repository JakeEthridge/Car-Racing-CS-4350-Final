#pragma once

#include "GLView.h"
#include "irrKlang.h"
#include "Car.h"
#include "GLView.h"
#include "chrono"
#include <NetMessengerClient.h>
#include <NetMsgCreateRawWO.h>
#include "PxPhysicsAPI.h"

namespace Aftr
{
   class Camera;

/**
   \class GLViewSpeedRacer
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewSpeedRacer : public GLView
{
public:
   static GLViewSpeedRacer* New(const std::vector<std::string>& outArgs, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene);
   virtual ~GLViewSpeedRacer();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void createSpeedRacerWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );
   void carSelection();
   WO* car4;
   WO* car3;
   WO* car2;
   WO* car1;
   WO* race1;
   WO* race2;
   WO* race3;
   WO* race4;
   WO* race5;
   WO* race6;
   WO* race7;
   WO* race8;
   WO* race9;
   WO* race10;
   WO* race11;
   WO* race12;
   WO* race13;
   WO* race14;
   WO* race15;
   WO* race16;
   WO* race17;
   WO* prop;
   WO* prop2;
   WO* prop3;
   WO* prop4;
   WO* prop5;
   WO* prop6;
   virtual void updateGravity(physx::PxVec3 g);
   void spawnPlayer1();
   void spawnPlayer2();
   void resetCarPosition();
   bool isTerrainLoaded();
   virtual void createGrid();
   virtual void createAnotherGrid();
   virtual void switchSkyBox(int index);
   virtual void switchTerrain(bool useAnotherGrid);
   virtual void updateActiveKeys(SDL_KeyCode keycode, bool state);
   void gridTerrainFunction();
   void anotherGridTerrainFunction();
   void moveTerrainNegativeX(float amount);
   void moveTerrainUp(float amount);
   void moveTerrainDown(float amount);
   void rotateTerrain(float angle);
   void setCarSpeed(float speed);
   void loadAssets();
   void updateControls();
   Car* getVisibleCar() {
       if (car_test->isVisible) return car_test;
       if (car_turn->isVisible) return car_turn;
       if (car_other_side->isVisible) return car_other_side;
       if (car_new->isVisible) return car_new;
       return nullptr;
   }

   // Simple linear interpolation function
   float lerp(float a, float b, float t) {
       return a + t * (b - a);
   }
protected:
   GLViewSpeedRacer(const std::vector<std::string>& args, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene);
   virtual void onCreate();  
   // IrrKlang Sound Device
   irrklang::ISoundEngine* engine = irrklang::createIrrKlangDevice();
   float volume = 1;

   physx::PxPhysics* pxPhysics = nullptr;
   physx::PxScene* pxScene = nullptr;
   Mat4 spy_pose;
   // Cars
   Car* car_test;
   Car* car_turn;
   Car* car_other_side;
   Car* car_new;
   Car* carMain;
   Car* carRight;
   Car* carLeft;
   Car* carDown;
   float timer;       // Timer to track elapsed time
   Aftr::Vector initialCarPosition; // Initial position of the car
   // Collision detection
   float lastSoundTime; // Time when the sound was last played
   bool isSoundPlaying; // Flag to check if the sound is playing
   const float soundCooldown = 1.0f; // Cooldown period in seconds
   NetMessengerClient* client = nullptr;
   std::chrono::high_resolution_clock::time_point tcpRetry;
   // Other members
   std::vector<Car*> cars; // Vector of car pointers
   float moveAmount; // Amount to move the car

   irrklang::ISoundEngine* soundEngine = irrklang::createIrrKlangDevice();
   std::unordered_map<WO*, irrklang::ISoundSource*> defaultSounds;
   std::vector<std::string> soundList;
   std::map<SDL_KeyCode, bool> active_keys;
   enum GameState { START_SCREEN, LOADING, MAIN_GUI };
   GameState gameState = START_SCREEN;
   Uint32 loadingStartTime = 0;
   bool isLoading = false;
   SDL_Texture* startImage = nullptr;
   SDL_Texture* loadingImage = nullptr;
   // Add your SDL_Renderer pointer
   SDL_Renderer* renderer;
   // Method to load texture
   SDL_Texture* loadTexture(const std::string& path);
   irrklang::ISound* startScreenSoundtrack = nullptr;
   // Member variables to track loading status
   bool terrain1Loaded = false;
   bool terrain2Loaded = false;
   bool isAssetsLoaded; // Flag to indicate assets are loaded
   // Methods to update loading status
   void setTerrain1Loaded(bool status) { terrain1Loaded = status; }
   void setTerrain2Loaded(bool status) { terrain2Loaded = status; }
   physx::PxVec3 gravity = physx::PxVec3(0.0f, 0.0f, -9.8f);
   int resetTime;     // Time limit to reach the destination in seconds
   WO* terrain; // Assume WO is your World Object class for the terrain
   Camera* camera; // Assume Camera is your camera class
   float terrainHeight;
   bool isTopViewActive = false;
   bool isFreeLookActive = false;
};

/** \} */

} //namespace Aftr
