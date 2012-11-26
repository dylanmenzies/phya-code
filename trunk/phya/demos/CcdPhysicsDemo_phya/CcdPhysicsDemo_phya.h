/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-20
06 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef CCD_PHYSICS_DEMO_H
#define CCD_PHYSICS_DEMO_H


#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif


#include "LinearMath/btAlignedObjectArray.h"

//Phya
#include "Phya.hpp"
#ifdef MANIFOLDCOLLISION
#define NBODIES 2
#else
#define NBODIES 10  //2 10
#endif

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class CcdPhysicsDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	class	Win32ThreadSupport*		m_threadSupportCollision;
	class	Win32ThreadSupport*		m_threadSupportSolver;
#endif
#endif

	btConstraintSolver*	m_solver;

	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;



	//Phya
	paRndFun* m_rndfun;
	paWavFun* m_wavfun;
	paFunSurface* m_funsurf;			// Function surface.. uses surface functions, combined with
	paFunSurface* m_funsurfn[NBODIES];
	paModalData* m_modes;
	paModalRes* m_res[NBODIES];
	paBody* m_abody[NBODIES];
	int PhyaUpdateCollisions();
	int PhyaUpdateManifoldCollision();
	float m_dt;

	public:

	void	initPhysics();
	void	exitPhysics();

	//Phya
	void    initAudio();
	void    startAudio();
	void    exitAudio();

	virtual ~CcdPhysicsDemo()			// Not currently used in renderer exit.
	{
		exitAudio();		// Phya
		exitPhysics();
	}

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);
	
	void createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos );
	
	static DemoApplication* Create()
	{
		CcdPhysicsDemo* demo = new CcdPhysicsDemo;
		demo->myinit();
		demo->initAudio();
		demo->initPhysics();
		return demo;
	}

};

#endif //CCD_PHYSICS_DEMO_H

