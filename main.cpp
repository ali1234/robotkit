// Robot Kit - main.c

// Copyright 2010 Alistair Buxton <a.j.buxton@gmail.com>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "Ogre.h"
#include "OgreConfigFile.h"
#include "OgreStringConverter.h"
#include "OgreException.h"

#define HIGH_QUALITY 1
#define FRAMEWRITER 0

//Use this define to signify OIS will be used as a DLL
//(so that dll import/export macros are in effect)
#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <CoreFoundation/CoreFoundation.h>

// This function will locate the path to our application on OS X,
// unlike windows you can not rely on the curent working directory
// for locating your configuration files and resources.
std::string macBundlePath()
{
    char path[1024];
    CFBundleRef mainBundle = CFBundleGetMainBundle();
    assert(mainBundle);

    CFURLRef mainBundleURL = CFBundleCopyBundleURL(mainBundle);
    assert(mainBundleURL);

    CFStringRef cfStringRef = CFURLCopyFileSystemPath( mainBundleURL, kCFURLPOSIXPathStyle);
    assert(cfStringRef);

    CFStringGetCString(cfStringRef, path, 1024, kCFStringEncodingASCII);

    CFRelease(mainBundleURL);
    CFRelease(cfStringRef);

    return std::string(path);
}
#endif

#include <btBulletDynamicsCommon.h>

#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"

using namespace Ogre;

#include "parts.h"
#include "hexapod.h"

class WorldFrameListener : public FrameListener, public WindowEventListener, public RenderTargetListener
{
protected:
	SceneNode *cam_node;
	SceneNode *hex_node;
	btDiscreteDynamicsWorld *dynamicsWorld;
	BtOgre::DebugDrawer *mDebugDrawer;
public:

	int frame_number;

	// Constructor takes a RenderWindow because it uses that to determine input context
	WorldFrameListener(RenderWindow* win, SceneNode *c, SceneNode *h, btDiscreteDynamicsWorld *dw, BtOgre::DebugDrawer *mDD, bool bufferedKeys = false, bool bufferedMouse = false,
			     bool bufferedJoy = false ) : frame_number(10000),
		mWindow(win), cam_node(c), hex_node(h), dynamicsWorld(dw), mDebugDrawer(mDD), mNumScreenShots(0),
		mTimeUntilNextToggle(0), mFiltering(TFO_BILINEAR),
		mAniso(1), mSceneDetailIndex(0), mDebugOverlay(0),
		mInputManager(0), mKeyboard(0), mJoy(0)
    {
		using namespace OIS;

		LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
		ParamList pl;
		size_t windowHnd = 0;
		std::ostringstream windowHndStr;

		win->getCustomAttribute("WINDOW", &windowHnd);
		windowHndStr << windowHnd;
		pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

		mInputManager = InputManager::createInputSystem( pl );

		//Create all devices (We only catch joystick exceptions here, as, most people have Key/Mouse)
		//mKeyboard = static_cast<Keyboard*>(mInputManager->createInputObject( OISKeyboard, bufferedKeys ));
		try {
			mJoy = static_cast<JoyStick*>(mInputManager->createInputObject( OISJoyStick, bufferedJoy ));
		}
		catch(...) {
			mJoy = 0;
            LogManager::getSingletonPtr()->logMessage("*** NO Joystick ***");
		}

		//Set initial mouse clipping size

		//Register as a Window listener
		WindowEventUtilities::addWindowEventListener(mWindow, this);
        //Register as render target listener
        mWindow->addListener(this);
        
    }

	//Adjust mouse clipping area

	//Unattach OIS before window shutdown (very important under Linux)
	void windowClosed(RenderWindow* rw)
	{
		//Only close for window that created OIS (the main window in these demos)
		if( rw == mWindow )
		{
			if( mInputManager )
			{
				//mInputManager->destroyInputObject( mKeyboard );
				mInputManager->destroyInputObject( mJoy );

				OIS::InputManager::destroyInputSystem(mInputManager);
				mInputManager = 0;
			}
		}
	}

	~WorldFrameListener()
	{
        mWindow->removeListener(this);
		//Remove ourself as a Window listener
		WindowEventUtilities::removeWindowEventListener(mWindow, this);
		windowClosed(mWindow);
	}

	bool processUnbufferedKeyInput(const FrameEvent& evt)
	{
		using namespace OIS;

		if( mKeyboard->isKeyDown(KC_ESCAPE) || mKeyboard->isKeyDown(KC_Q) )
			return false; // return false to quit

		if( mKeyboard->isKeyDown(KC_T) && mTimeUntilNextToggle <= 0 )
		{
			switch(mFiltering)
			{
			case TFO_BILINEAR:
				mFiltering = TFO_TRILINEAR;
				mAniso = 1;
				break;
			case TFO_TRILINEAR:
				mFiltering = TFO_ANISOTROPIC;
				mAniso = 8;
				break;
			case TFO_ANISOTROPIC:
				mFiltering = TFO_BILINEAR;
				mAniso = 1;
				break;
			default: break;
			}
			MaterialManager::getSingleton().setDefaultTextureFiltering(mFiltering);
			MaterialManager::getSingleton().setDefaultAnisotropy(mAniso);

			mTimeUntilNextToggle = 1;
		}

		if(mKeyboard->isKeyDown(KC_SYSRQ) && mTimeUntilNextToggle <= 0)
		{
			std::ostringstream ss;
			ss << "screenshot_" << ++mNumScreenShots << ".png";
			mWindow->writeContentsToFile(ss.str());
			mTimeUntilNextToggle = 0.5;
			
		}

		// Return true to continue rendering
		return true;
	}

    bool frameStarted(const FrameEvent &evt)
    {
		static float rtime = 0.0001;

		if(mWindow->isClosed())	return false;

		

		//mDebugDrawer->step();

#if FRAMEWRITER
        	float time = 1.0/60.0;
#else
        	float time = 1.0*evt.timeSinceLastFrame;
#endif
		rtime += time;

		Vector3 robot_pos = hex_node->getPosition();

		if(frame_number%600 == 0) {
			float vx = robot_pos.x/rtime;
			float vy = robot_pos.z/rtime;
			printf("Robot velocity = %.1f cm/s\n", 100*sqrt((vx*vx)+(vy*vy)));
		}

		cam_node->yaw(Degree(3.0*time));
		cam_node->translate(-0.002*time,0,0.077*time);

		if(rtime > 5.0) {
			Vector3 d = robot_pos - cam_node->getPosition();
			d.y = 0;
			cam_node->translate(d*time*0.5);
		}				

		dynamicsWorld->stepSimulation(time, 1000, 0.0005);
/**/
		return true;

    }

    bool frameEnded(const FrameEvent &evt) {
#if  FRAMEWRITER
        std::ostringstream ss;
		ss << "/media/Storage/rwtmp/s-000001_" << frame_number << ".png";
		mWindow->writeContentsToFile(ss.str());
		LogManager::getSingletonPtr()->logMessage("Saved Frame "+StringConverter::toString(frame_number));
#endif
		frame_number ++;
#if FRAMEWRITER
        if (frame_number < 10480)
            return true;
        else return false;
#else
		return true;
#endif
    }


protected:
	RenderWindow* mWindow;

	unsigned int mNumScreenShots;
	// just to stop toggles flipping too fast
	Real mTimeUntilNextToggle ;
	TextureFilterOptions mFiltering;
	int mAniso;

	int mSceneDetailIndex ;
	Overlay* mDebugOverlay;

	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Keyboard* mKeyboard;
	OIS::JoyStick* mJoy;
};

void PreTickCallback(btDynamicsWorld *w, btScalar timestep) {
	Hexapod *h = (Hexapod *)w->getWorldUserInfo();
	h->think(timestep);
}

class World
{
public:
	World()	{
		mFrameListener = 0;
		mRoot = 0;
// Provide a nice cross platform solution for locating the configuration files
// On windows files are searched for in the current working directory, on OS X however
// you must provide the full path, the helper function macBundlePath does this for us.
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
		mResourcePath = macBundlePath() + "/Contents/Resources/";
#else
		mResourcePath = "";
#endif
	}

	~World() {
		if (mFrameListener) delete mFrameListener;
		if (mRoot) delete mRoot;

		if (dynamicsWorld) delete dynamicsWorld;
		if (solver) delete solver;
		if (dispatcher) delete dispatcher;
		if (collisionConfiguration) delete collisionConfiguration;
		if (broadphase) delete broadphase;
	}

	void go(void) {
		if (!setup()) return;
		mRoot->startRendering();
		destroyScene();
	}

protected:
	Root *mRoot;
	SceneManager* mSceneMgr;
	WorldFrameListener* mFrameListener;
	RenderWindow* mWindow;
	Ogre::String mResourcePath;
	SceneNode *cam_node;
	Camera *free_camera;
	Hexapod *h;
	SceneNode *hex_node;

	// Bullet things
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
        BtOgre::DebugDrawer* mDebugDrawer;

    // These internal methods package up the stages in the startup process
    /** Sets up the application - returns false if the user chooses to abandon configuration. */
    bool setup(void)
    {

		String pluginsPath;
		// only use plugins.cfg if not static
#ifndef OGRE_STATIC_LIB
		pluginsPath = mResourcePath + "plugins.cfg";
#endif
		
        mRoot = new Root(pluginsPath, 
            mResourcePath + "ogre.cfg", mResourcePath + "Ogre.log");

        setupResources();

        bool carryOn = configure();
        if (!carryOn) return false;

        chooseSceneManager();

        // Set default mipmap level (NB some APIs ignore this)
        TextureManager::getSingleton().setDefaultNumMipmaps(5);
#if HIGH_QUALITY
		MaterialManager::getSingleton().setDefaultTextureFiltering(TFO_ANISOTROPIC);
		MaterialManager::getSingleton().setDefaultAnisotropy(8);
#endif

		// Create any resource listeners (for loading screens)
		createResourceListener();
		// Load resources
		loadResources();

		// Create the scene
        createScene();
        createViewports();
        createFrameListener();
        return true;

    }
    /** Configures the application - returns false if the user chooses to abandon configuration. */
    bool configure(void)
    {
        // Show the configuration dialog and initialise the system
        // You can skip this and use root.restoreConfig() to load configuration
        // settings if you were sure there are valid ones saved in ogre.cfg
        if(mRoot->showConfigDialog())
//	if(mRoot->restoreConfig())
        {
            // If returned true, user clicked OK so initialise
            // Here we choose to let the system create a default rendering window by passing 'true'
            mWindow = mRoot->initialise(true);
            return true;
        }
        else
        {
            return false;
        }
    }

    void chooseSceneManager(void)
    {
        // Create the SceneManager, in this case a generic one
        mSceneMgr = mRoot->createSceneManager(ST_GENERIC, "ExampleSMInstance");
    }

    void destroyScene(void){}    // Optional to override this

    void createViewports(void)
    {
        Viewport *vp;
	vp = mWindow->addViewport(free_camera, 0);
	vp->setDimensions(0.0,0.0,1.0,1.0);
    }

    void createFrameListener(void)
    {
        mFrameListener= new WorldFrameListener(mWindow, cam_node, hex_node, dynamicsWorld, mDebugDrawer);
        mRoot->addFrameListener(mFrameListener);
    }

    void createScene(void)
    {

	// Setup physics world

	// Build the broadphase
	broadphase = new btDbvtBroadphase();
	 
	// Set up the collision configuration and dispatcher
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	 
	// The actual physics solver
	solver = new btSequentialImpulseConstraintSolver;
	 
	// The world.
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	//dynamicsWorld->setGravity(btVector3(0,-9.8,0));
	dynamicsWorld->setGravity(btVector3(0,-9.8,0));

	mDebugDrawer = new BtOgre::DebugDrawer(mSceneMgr->getRootSceneNode(), dynamicsWorld);
	dynamicsWorld->setDebugDrawer(mDebugDrawer);

	// The ground
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
	btDefaultMotionState* groundMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
	btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);

	h = new Hexapod(mSceneMgr, dynamicsWorld);
	hex_node = h->GetSceneNode();

	dynamicsWorld->setInternalTickCallback(PreTickCallback,h,true);

	cam_node = mSceneMgr->getRootSceneNode()->createChildSceneNode();

	free_camera = mSceneMgr->createCamera("FreeCamera");
	free_camera->setPosition(Vector3(0.15,0.4,0.6));

	free_camera->lookAt(0,0.05,0);

	free_camera->setNearClipDistance(0.001);
	free_camera->setFarClipDistance(5);

	cam_node->attachObject(free_camera);

        mSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));
#if HIGH_QUALITY
        mSceneMgr->setShadowTechnique(SHADOWTYPE_STENCIL_ADDITIVE);
#endif

        mSceneMgr->setSkyBox(true, "BlueSkyBox", 10);

	SceneNode * light_node = mSceneMgr->getRootSceneNode()->createChildSceneNode();

        Light *light;

        light = mSceneMgr->createLight("SunLight");
        light->setType(Light::LT_DIRECTIONAL);
	light->setPosition(Vector3(0,1,0));
        light->setDirection(Vector3(0.05, -1.0, 0.05).normalisedCopy());
        light->setDiffuseColour(1.0,1.0,1.0);
        light->setSpecularColour(1.0,1.0,1.0);

	light_node->attachObject(light);





	Plane plane(Vector3::UNIT_Y, 0);
	MeshManager::getSingleton().createPlane("ground",
           ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
           100,100,100,100,true,1,500,500,Vector3::UNIT_Z);

	Entity *ent = mSceneMgr->createEntity("GroundEntity", "ground");
	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);
	ent->setMaterialName("Ground");

     


    }

    /// Method which will define the source of resources (other than current folder)
    void setupResources(void)
    {
        // Load resource paths from config file
        ConfigFile cf;
        cf.load(mResourcePath + "resources.cfg");

        // Go through all sections & settings in the file
        ConfigFile::SectionIterator seci = cf.getSectionIterator();

        String secName, typeName, archName;
        while (seci.hasMoreElements())
        {
            secName = seci.peekNextKey();
            ConfigFile::SettingsMultiMap *settings = seci.getNext();
            ConfigFile::SettingsMultiMap::iterator i;
            for (i = settings->begin(); i != settings->end(); ++i)
            {
                typeName = i->first;
                archName = i->second;
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
                // OS X does not set the working directory relative to the app,
                // In order to make things portable on OS X we need to provide
                // the loading with it's own bundle path location
                ResourceGroupManager::getSingleton().addResourceLocation(
                    String(macBundlePath() + "/" + archName), typeName, secName);
#else
                ResourceGroupManager::getSingleton().addResourceLocation(
                    archName, typeName, secName);
#endif
            }
        }
    }

	/// Optional override method where you can create resource listeners (e.g. for loading screens)
	void createResourceListener(void)
	{

	}

	/// Optional override method where you can perform resource group loading
	/// Must at least do ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	void loadResources(void)
	{
		// Initialise, parse scripts etc
		ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	}


};

#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"

INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
int main(int argc, char **argv)
#endif
{
    // Create application object
    World app;

    try {
        app.go();
    } catch( Exception& e ) {
#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        fprintf(stderr, "An exception has occurred: %s\n",
                e.getFullDescription().c_str());
#endif
    }

    return 0;
}
