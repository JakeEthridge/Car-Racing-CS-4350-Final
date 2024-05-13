#pragma once

#include "GLView.h"
#include "irrKlang.h"

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
   static GLViewSpeedRacer* New( const std::vector< std::string >& outArgs );
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




protected:
   GLViewSpeedRacer( const std::vector< std::string >& args );
   virtual void onCreate();  
   // IrrKlang Sound Device
   irrklang::ISoundEngine* engine = irrklang::createIrrKlangDevice();
   float volume = 1;
};

/** \} */

} //namespace Aftr
