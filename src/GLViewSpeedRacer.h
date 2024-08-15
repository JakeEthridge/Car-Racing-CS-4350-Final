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
        virtual void onResizeWindow(GLsizei width, GLsizei height);
        virtual void onMouseDown(const SDL_MouseButtonEvent& e);
        virtual void onMouseUp(const SDL_MouseButtonEvent& e);
        virtual void onMouseMove(const SDL_MouseMotionEvent& e);
        virtual void onKeyDown(const SDL_KeyboardEvent& key);
        virtual void onKeyUp(const SDL_KeyboardEvent& key);
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
        void OtherCarSkin1();
        void OtherCarSkin2();
        void OtherCarSkin3();
        void sendTerrainChangeMessage(bool useAnotherGrid, float moveDownAmount, float rotateAmount, float moveNegativeXAmount);
        void handleCarMovement(int carModel, int keyPress, float moveAmount);
        void spawnPlayer2Skin1();
        void spawnPlayer2Skin2();
        void spawnPlayer2Skin3();
        void hideAllCars();
        void hideAllCars2();
        void respawnSelectedCar();
  
        bool otherInstanceTerrainLoaded = false; // New variable to track terrain loading in the other instance
        void handleTerrainLoading() {
            if (isTerrainLoaded() && otherInstanceTerrainLoaded) {
                // Transition to the main GUI after terrain is loaded in both instances
                gameState = MAIN_GUI;
                isLoading = false;
            }
        }
        Car* getVisibleCar1() {
            if (car_test->isVisible) return car_test;
            if (car_turn->isVisible) return car_turn;
            if (car_other_side->isVisible) return car_other_side;
            if (car_new->isVisible) return car_new;
            return nullptr;
        }

        Car* getVisibleCar2() {
            if (carMain->isVisible) return carMain;
            if (carRight->isVisible) return carRight;
            if (carLeft->isVisible) return carLeft;
            if (carDown->isVisible) return carDown;
            return nullptr;
        }
        void startLoadingProcess();
        bool isNetworkEnabled; // Flag to enable/disable network messaging
        bool followCar1; // Add this member variable to track the current car group
        // Initialize the static member
        //bool isMuted = false;
        // Cars1
        Car* car_test;
        Car* car_turn;
        Car* car_other_side;
        Car* car_new;

        //car2
        Car* carMain;
        Car* carRight;
        Car* carLeft;
        Car* carDown;
        int selectedSkin = 0; // Default value or initialize as needed
        // Simple linear interpolation function
        float lerp(float a, float b, float t) {
            return a + t * (b - a);
        }
        irrklang::ISoundEngine* soundEngine = irrklang::createIrrKlangDevice();
        irrklang::ISoundEngine* drivingSound = irrklang::createIrrKlangDevice();
        irrklang::ISoundEngine* StartAudio = irrklang::createIrrKlangDevice();
        //bool isSpacePressed = false; // Flag to indicate if the space key is pressed
        bool showBlackScreen = true;
        static bool isMuted; // Define as static
         static int selectedMusicIndex; // Declare static member
         // Timer variables
         static bool isTimerRunning;
         static Uint32 timerStartTime;
         static int resetTime;
         static Uint32 pausedTime;
         bool isFullSize = true;
    protected:
        GLViewSpeedRacer(const std::vector<std::string>& args, physx::PxPhysics* pxPhysics, physx::PxScene* pxScene);
        virtual void onCreate();
        // IrrKlang Sound Device
        irrklang::ISoundEngine* engine = irrklang::createIrrKlangDevice();

        float volume = 1;

        physx::PxPhysics* pxPhysics = nullptr;
        physx::PxScene* pxScene = nullptr;
        Mat4 spy_pose;

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
        std::vector<Car*> spawnedCars; // Vector to keep track of spawned car objects
        Car* selectedCar = nullptr; // Pointer to the currently selected car

        // Variables for collision detection
       

       
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
        WO* terrain; // Assume WO is your World Object class for the terrain
        Camera* camera; // Assume Camera is your camera class
        float terrainHeight;
        bool isTopViewActive = false;
        bool isFreeLookActive = false;
        bool isDrivingSoundPlaying;          // Flag to check if sound is playing
        enum CarType {
            DODGE,
            FORD
        };
        bool showGrids;
        CarType currentCarType = DODGE; // Default to Dodge
        bool usePlayer1 = true; // Start with spawnPlayer1
        float moveAmount = 8.0f; // Default speed for both players
        float player1SpeedBoostAmount = 0.0f; // Amount to boost speed for player 1
        float player2SpeedBoostAmount = 0.0f; // Amount to boost speed for player 2
        bool isPlayer1SpeedBoostActive = false;
        bool isPlayer2SpeedBoostActive = false;
        uint32_t player1SpeedBoostStartTime = 0;
        uint32_t player2SpeedBoostStartTime = 0;
        uint32_t lastPlayer1SpeedBoostTime = 0; // Track the last time the button was pressed
        uint32_t lastPlayer2SpeedBoostTime = 0; // Track the last time the button was pressed
        bool canPressPlayer1SpeedButton = true; // Track if the button can be pressed
        bool canPressPlayer2SpeedButton = true; // Track if the button can be pressed
        // Variables to track laps and timing
        static int lapNumber;
        static int leftKeyPressCount;
        static Uint32 secondLeftKeyPressTime;
        static bool updateLapPending;
        int currentSkinPlayer2 = -1; // Initialize with an invalid value to indicate no skin is currently active



       
        enum CarModel {
            CAR_MODEL_DODGE,
            CAR_MODEL_FORD,
            // Add more models if needed
        };

        CarModel selectedCarModel = CAR_MODEL_DODGE; // Default model
        Car* carDodge = nullptr;
        Car* carFord = nullptr;

    };

    /** \} */

} //namespace Aftr
