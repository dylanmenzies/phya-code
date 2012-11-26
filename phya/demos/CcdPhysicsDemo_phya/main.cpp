/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
CcdPhysicsDemo_phya
Incorporating Phya integration code.
(c) Dylan Menzies 2001-2009   www.zenprobe.com/phya
*/


#include "CcdPhysicsDemo_phya.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

#include "Phya.hpp"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{
	CcdPhysicsDemo* ccdDemo = new CcdPhysicsDemo();

	ccdDemo->initAudio();					//Phya

	ccdDemo->initPhysics();					//Phya  Also makes body -> audio body links.
	ccdDemo->setCameraDistance(4.0);		//Overide default renderer setting.
	ccdDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

	ccdDemo->startAudio();					//Phya Set audio device and start thread.

	glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",ccdDemo);

	ccdDemo->exitAudio();					//Phya  Kills audio thread nicely, if we get this far.
	delete ccdDemo;
	return 0;

}
