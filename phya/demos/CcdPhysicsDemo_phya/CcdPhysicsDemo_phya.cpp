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
(c) Dylan Menzies 2001-2012
*/




//enable just one, DO_BENCHMARK_PYRAMIDS or DO_WALL
//#define DO_BENCHMARK_PYRAMIDS 1
#define DO_WALL 1

//Note: some of those settings need 'DO_WALL' demo
//#define USE_KINEMATIC_GROUND 1
//#define PRINT_CONTACT_STATISTICS 1
//#define USER_DEFINED_FRICTION_MODEL 1
//#define USE_CUSTOM_NEAR_CALLBACK 1
//#define CENTER_OF_MASS_SHIFT 1
//#define VERBOSE_TIMESTEPPING_CONSOLEOUTPUT 1

//#define USE_PARALLEL_SOLVER 1 //experimental parallel solver
//#define USE_PARALLEL_DISPATCHER 1

//from Bullet 2.68 onwards ODE Quickstep constraint solver is optional part of Bullet, re-distributed under the ZLib license with permission of Russell L. Smith
//#define COMPARE_WITH_QUICKSTEP 1

//int gc = 0;	// Count high-level contact create/destroy.

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "../../Extras/BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#ifdef WIN32
#include "../../Extras/BulletMultiThreaded/Win32ThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //WIN32

#ifdef USE_LIBSPE2
#include "../../Extras/BulletMultiThreaded/SpuLibspe2Support.h"
#endif //USE_LIBSPE2

#ifdef USE_PARALLEL_SOLVER
#include "../../Extras/BulletMultiThreaded/SpuParallelSolver.h"
#include "../../Extras/BulletMultiThreaded/SpuSolverTask/SpuParallellSolverTask.h"
#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER



#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"


//#include "BMF_Api.h"
#include <stdio.h> //printf debugging

static float	gCollisionMargin = 0.05f;
#include "CcdPhysicsDemo_phya.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"



extern ContactAddedCallback			gContactAddedCallback;
extern ContactDestroyedCallback		gContactDestroyedCallback;


btTransform comOffset;
btVector3 comOffsetVec(0,2,0);

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

bool createConstraint = true;//false;



#ifdef _DEBUG
const int gNumObjects = NBODIES;
#else
const int gNumObjects = NBODIES;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cpp
#endif


const int maxNumObjects = 32760;

int	shapeIndex[maxNumObjects];


#define CUBE_HALF_EXTENTS 1.5  //.5

#define EXTRA_HEIGHT -10.f
//GL_LineSegmentShape shapeE(btPoint3(-50,0,0),
//						   btPoint3(50,0,0));


void CcdPhysicsDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector3 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);
			
			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);
#ifdef USER_DEFINED_FRICTION_MODEL	
		///Advanced use: override the friction solver
		body->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

		}
	}
}


////////////////////////////////////

//paFloat gx =0;		//	Track changes in position

//Phya
//?? user data for btPersistentManifold to store manifold-level acontact.
//?? does numContacts always go to zero once before manifold removed - no not for box shape.
paFloat x = 0;
int
CcdPhysicsDemo::PhyaUpdateCollisions()
{
	//input : m_dispatcher, m_dt
	int numManifolds = m_dispatcher->getNumManifolds();
	int i;
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)(contactManifold->getBody0());
		btCollisionObject* obB = (btCollisionObject*)(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();
//		printf("%d\n", numContacts);
//		printf("\n");
//		if (numContacts > 1) numContacts = 1;

//paFloat x=0;
//paFloat impulseSum = 0;



		//// First pass to see if all contacts are new => impact possible
		//// Currently fails, because of runs of single contacts.
		//bool firstContact = true;	// Hypothesis.
		//for (int j=0;j<numContacts;j++)
		//{  //break;
		//	btManifoldPoint& pt = contactManifold->getContactPoint(j);
		//	paContact* ac = (paContact*)(pt.m_userPersistentData);

		//	if (ac) firstContact = false;
		//}

//if (firstContact) 
//printf("%d ",numContacts);
//printf("\n");
//paFloat D = 0;
//paFloat XD = 0;
		for (int j=0;j<numContacts;j++)
		{  //break;
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			paContact* ac = (paContact*)(pt.m_userPersistentData);
			bool firstContact = false;

//printf("%d\n",contactManifold);
//const btVector3 &p = pt.getPositionWorldOnA();
//const btScalar d = pt.getDistance();
//printf("%f	%f	%f	%f %f\n",p.x(), p.y(), p.z(), d, p.x()-x);
//D += pt.m_distance1;
//XD += p.x() * pt.m_distance1;
//x = p.x();

//printf("%d\n", ac);
//{
//const btVector3 &p = pt.getPositionWorldOnB();
//if (firstContact) printf("*");
//printf("%f	%f	%f %d\n", pt.m_appliedImpulse /m_dt, p.x(), ac);
//}
			if (ac == 0)	// Test for new contact.
			{
				// Try new contact.
				ac = paContact::newContact();
				if (ac)
				{
					ac->setBody1((paBody*)obA->getUserPointer());	// body->abody, set up in initPhysics.
					ac->setBody2((paBody*)obB->getUserPointer());
					ac->setUserData((void *)(&pt));					// contact<->acontact cross ref.
					pt.m_userPersistentData = (void *)ac;		
//printf("+ %d\n", ac);
//printf("+ %d %d\n", ac, &pt);	
//gc++;
				}
				firstContact = true;	// Indicates should test for impact below..
			}

			// Update contacts.
//			if (ac || firstContact)		//! if !ac && firstcontact, could also check if any impacts available.
			{
				#define JITTER_VEL_MAX 0.3
				// Update dynamic data  //! Should pretty this up with bt-pa vec ops.
				paGeomCollisionData d;
//				btTransform wt1;
//				btMotionState ((btRigidBody*)obA)->getMotionState()->getWorldTransform(wt1);
//				const btVector3 &p1 = wt1.getOrigin();
				if (( (btRigidBody *)obA)->getActivationState() != ACTIVE_TAG)
				d.body1.isStill = true;
				else
				{
					const btVector3 &p1 = obA->getWorldTransform().getOrigin();
					const btVector3 &v1 = ((btRigidBody *)obA)->getLinearVelocity();
					const btVector3 &a1 = ((btRigidBody *)obA)->getAngularVelocity();

					if (abs(v1.x())+abs(v1.y())+abs(v1.z())
						+abs(a1.x())+abs(a1.y())+abs(a1.z()) < JITTER_VEL_MAX) //! Could be improved.
						d.body1.isStill = true;			// Kill standing jitter from moving contacts. 
					else
					{
						d.body1.isStill = false;
						d.body1.position[0] = p1.x();
						d.body1.position[1] = p1.y();
						d.body1.position[2] = p1.z();
						d.body1.velocity[0] = v1.x();
						d.body1.velocity[1] = v1.y();
						d.body1.velocity[2] = v1.z();
						d.body1.angularVel[0] = a1.x();
						d.body1.angularVel[1] = a1.y();
						d.body1.angularVel[2] = a1.z();
					}
				}

				if (( (btRigidBody *)obB)->getActivationState()  == ISLAND_SLEEPING) //!= ACTIVE_TAG)
				d.body2.isStill = true;
				else
				{
					const btVector3 &p2 = obB->getWorldTransform().getOrigin();
					const btVector3 &v2 = ((btRigidBody *)obB)->getLinearVelocity();
					const btVector3 &a2 = ((btRigidBody *)obB)->getAngularVelocity();

					if (abs(v2.x())+abs(v2.y())+abs(v2.z())+abs(a2.x())
						+abs(a2.y())+abs(a2.z()) < JITTER_VEL_MAX)
						d.body2.isStill = true;			// Kill standing jitter.
					else
					{
						d.body2.isStill = false;
						d.body2.position[0] = p2.x();
						d.body2.position[1] = p2.y();
						d.body2.position[2] = p2.z();
						d.body2.velocity[0] = v2.x();
						d.body2.velocity[1] = v2.y();
		 				d.body2.velocity[2] = v2.z();
						d.body2.angularVel[0] = a2.x();
						d.body2.angularVel[1] = a2.y();
						d.body2.angularVel[2] = a2.z();
					}
				}

				// Calc and pass dynamic data.
				paContactDynamicData cd;
	
				////if (d.body1.isStill && d.body2.isStill)
				////{
				////	// No contact force, avoid uneeded calcs.
				////	cd.contactForce = 0;
				////}
				////else
				{
					const btVector3 &n = pt.m_normalWorldOnB;
					d.normal[0] = n.x();
					d.normal[1] = n.y();
					d.normal[2] = n.z();

					const btVector3 &p = pt.getPositionWorldOnB();
					d.contactPos[0] = p.x();
					d.contactPos[1] = p.y();
					d.contactPos[2] = p.z();
//d.contactPos[0] = d.body2.position[0];
//d.contactPos[1] = d.body2.position[1]-1.5;
//d.contactPos[2] = d.body2.position[2];
//const btVector3 &pA = pt.getPositionWorldOnB();

//x += pt.m_appliedImpulse/m_dt * p.x();
//impulseSum += pt.m_appliedImpulse/m_dt;

					d.calcContactVel = false;
					// Calc contact vel by pos difference.
					// Only useful if contacts persist for several frames,
					// and don't jump. This rules out Bullet.
					if (ac)
					{
						paFloat* lp = ac->getLastPosition();			// Not valid for new contact.
			 			d.contactVel[0] = ( p.x() - lp[0] ) / m_dt;		// 
						d.contactVel[1] = ( p.y() - lp[1] ) / m_dt;
						d.contactVel[2] = ( p.z() - lp[2] ) / m_dt;

// printf("c vel %f\n", paGeomCalcMag(d.contactVel)); // / paGeomCalcMag(d.body2.velocity));
// printf("c pos %f  %f  %f\n", p.x(), p.y(), p.z()); // / paGeomCalcMag(d.body2.velocity));
// x+=p.x();
						lp[0] = (paFloat)p.x();
						lp[1] = (paFloat)p.y();
						lp[2] = (paFloat)p.z();
					}

					paGeomCollisionResult r;
					paGeomCollisionCalc(&d, &r);

//if (r.normalSpeedBody1RelBody2 > 1.0)
//if (pt.m_appliedImpulse)
//printf("ts %f	ns %f   z %f\n", r.tangentSpeedBody1RelBody2, r.normalSpeedBody1RelBody2, d.body2.position[1]);
//printf("C %f %f %f      CM %f %f %f\n", d.contactPos[0], d.contactPos[1], d.contactPos[2], 
//	   d.body2.position[0], d.body2.position[1], d.body2.position[2]);
//if (pt.m_appliedImpulse) printf("z %f\n", 	d.contactPos[1]); //d.body2.position[1]);


					// Experimental options in mapping kinematic parameters onto surface kinematic parameters.
					// Should eventually be contained in generators, and properties set for surfaces.

//					cd.speedContactRelBody1 = r.speedContactRelBody1;
//					cd.speedContactRelBody2 = r.speedContactRelBody2;
					//! Compromise to compensate for contact vel being hard to track directly, because eg Bullet contacts too short lived.
//					cd.speedContactRelBody1 = 6.0;
					cd.speedContactRelBody1 = r.tangentSpeedBody1RelBody2 + r.speedBody1RelBody2;		// 2nd speed is rel CM speed. 
																										//! Const is useful for disks. Note this prevents gens from being quiet, which keeps res s active too.
					cd.speedContactRelBody2 = cd.speedContactRelBody1;



					// Slip speed is the highest quality parameter available, because it is derived from velocities.
//					cd.speedBody1RelBody2 = r.tangentSpeedBody1RelBody2;		// Basic

//					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2
//											+ r.normalSpeedBody1RelBody2*10.0);

					// Boost slip with normal speed, to simulate impact using a contact.
					//! Contact power prop. speed x force. Try lateral force for more accuracy?
					//! These formula should be hidden in the contact generator.
					//! cd.contactForce controls amplitude, cd.speedBody1RelBody2 controls brightness (like hardness effect)
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;



//					cd.contactForce = pt.m_appliedImpulse /m_dt;										// Basic - doesn't work with constant in  cd.speedContactRelBody1 above
//					cd.contactForce = (r.tangentSpeedBody1RelBody2 + r.normalSpeedBody1RelBody2 > 0)	// Fixes above.
//										? pt.m_appliedImpulse /m_dt : 0;
					//					cd.contactForce = 100.0/sqrtf(numContacts);
					cd.contactForce =
										(r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 * 1.0)
											* pt.m_appliedImpulse /m_dt;

//!!! There is a very basic issue here to be resolved: Gain limiting is very desirable to bring out detail, but this can also 
//!!! squash impact amp which are only very short. Solution will involve more sophistocated gain compression.
//!!! Also if impacts are detected separately from contacts, they can have independent gain limiting. New complex stochastic
//!!! impacts will make this attractive.

//					if (cd.contactForce > 0.0 && cd.contactForce < 1.0) cd.contactForce = 1.0;

//if (!firstContact) printf("%f\n", r.speedContactRelBody2);


//printf("%f\n", r.normalVelBody1RelBody2);

					// Impact at contact start doesn't work well if contacts short lived.
					if (0 && firstContact && r.normalVelBody1RelBody2 < -0.2)
					{
						paImpact* ai = paImpact::newImpact();
						if (ai)
						{
							ai->setBody1((paBody*)obA->getUserPointer());
							ai->setBody2((paBody*)obB->getUserPointer());
							paImpactDynamicData id;
							id.impactImpulse = pt.m_appliedImpulse *0.1;	//r.normalSpeedBody1RelBody2; pt.m_appliedImpulse*0.01;    //; // 
							id.relNormalSpeedAtImpact = r.normalSpeedBody1RelBody2;
							id.relTangentSpeedAtImpact = r.tangentSpeedBody1RelBody2;
							ai->setDynamicData(&id);
						}
					}
				}
				// Pass dynamic contact data.

				if (ac && (1 || !firstContact)) ac->setDynamicData(&cd);
			}

			
			//printf("x %f   %d\n", x/numContacts, numContacts); // / paGeomCalcMag(d.body2.velocity));

			//paFloat v = 1.0;
			//contactData.contactForce = 0.005f * v;
			//contactData.speedContactRelBody1 = 0;
			//contactData.speedContactRelBody2 = v;
			//contactData.speedBody1RelBody2 = v;
			//ac->setDynamicData(&contactData);

		}
//printf("\n %f \n\n", XD/D);

//x /= impulseSum;
//printf("%f	%f	%f\n", x, gx, x-gx);
//gx = x;


	}

	return 0;
}







// Version of collision handling for single manifold,
// with one persistent audio contact.
// Used to test wavSurf etc

paFloat g_f = 0;
paFloat g_s = 0;
paFloat g_alpha;
bool g_inContact = false;

//int g_dc;		// debug count

int
CcdPhysicsDemo::PhyaUpdateManifoldCollision()
{
	// input : m_dispatcher, m_dt
	int numManifolds = m_dispatcher->getNumManifolds();
	if (numManifolds > 1) numManifolds = 1;		// Should only be one..
	int i;

	paContactDynamicData cd;
	paGeomCollisionResult r;
	btCollisionObject* obA; 
	btCollisionObject* obB;
	int numContacts = 0;

	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
		obA = (btCollisionObject*)(contactManifold->getBody0());
		obB = (btCollisionObject*)(contactManifold->getBody1());
	
//printf("M %d\n", contactManifold);
		numContacts = contactManifold->getNumContacts();

		cd.contactForce = 0;	// Sum forces from contacts in each manifold.


//g_dc++;

		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			// Update contacts.
			{
				#define JITTER_VEL_MAX 0.3
				// Update dynamic data  //! Should pretty this up with bt-pa vec ops.
				paGeomCollisionData d;

				if (( (btRigidBody *)obA)->getActivationState() != ACTIVE_TAG)
				d.body1.isStill = true;
				else
				{
					const btVector3 &p1 = obA->getWorldTransform().getOrigin();
					const btVector3 &v1 = ((btRigidBody *)obA)->getLinearVelocity();
					const btVector3 &a1 = ((btRigidBody *)obA)->getAngularVelocity();

					if (abs(v1.x())+abs(v1.y())+abs(v1.z())
						+abs(a1.x())+abs(a1.y())+abs(a1.z()) < JITTER_VEL_MAX) //! Could be improved.
						d.body1.isStill = true;			// Kill standing jitter from moving contacts. 
					else
					{
						d.body1.isStill = false;
						d.body1.position[0] = p1.x();
						d.body1.position[1] = p1.y();
						d.body1.position[2] = p1.z();
						d.body1.velocity[0] = v1.x();
						d.body1.velocity[1] = v1.y();
						d.body1.velocity[2] = v1.z();
						d.body1.angularVel[0] = a1.x();
						d.body1.angularVel[1] = a1.y();
						d.body1.angularVel[2] = a1.z();
					}
				}

				if ( ((btRigidBody *)obB)->getActivationState() == ISLAND_SLEEPING ) //!= ACTIVE_TAG)
				d.body2.isStill = true;
				else
				{
					const btVector3 &p2 = obB->getWorldTransform().getOrigin();
					const btVector3 &v2 = ((btRigidBody *)obB)->getLinearVelocity();
					const btVector3 &a2 = ((btRigidBody *)obB)->getAngularVelocity();

					if (abs(v2.x())+abs(v2.y())+abs(v2.z())+abs(a2.x())
						+abs(a2.y())+abs(a2.z()) < JITTER_VEL_MAX)
						d.body2.isStill = true;			// Kill standing jitter.
					else
					{
						d.body2.isStill = false;
						d.body2.position[0] = p2.x();
						d.body2.position[1] = p2.y();
						d.body2.position[2] = p2.z();
						d.body2.velocity[0] = v2.x();
						d.body2.velocity[1] = v2.y();
		 				d.body2.velocity[2] = v2.z();
						d.body2.angularVel[0] = a2.x();
						d.body2.angularVel[1] = a2.y();
						d.body2.angularVel[2] = a2.z();
					}
				}

				// Calc and pass dynamic data.
	
				////if (d.body1.isStill && d.body2.isStill)
				////{
				////	// No contact force, avoid uneeded calcs.
				////	cd.contactForce = 0;
				////}
				////else
				{
					const btVector3 &n = pt.m_normalWorldOnB;
					d.normal[0] = n.x();
					d.normal[1] = n.y();
					d.normal[2] = n.z();

					const btVector3 &p = pt.getPositionWorldOnB();
					d.contactPos[0] = p.x();
					d.contactPos[1] = p.y();
					d.contactPos[2] = p.z();

//x += pt.m_appliedImpulse/m_dt * p.x();
//impulseSum += pt.m_appliedImpulse/m_dt;

					d.calcContactVel = false;
					// Calc contact vel by pos difference.
					// Only useful if contacts persist for several frames,
					// and don't jump. This rules out Bullet.
					//if (ac)
					//{
					//	paFloat* lp = ac->getLastPosition();			// Not valid for new contact.
			 		//	d.contactVel[0] = ( p.x() - lp[0] ) / m_dt;		// 
					//	d.contactVel[1] = ( p.y() - lp[1] ) / m_dt;
					//	d.contactVel[2] = ( p.z() - lp[2] ) / m_dt;


					//	lp[0] = (paFloat)p.x();
					//	lp[1] = (paFloat)p.y();
					//	lp[2] = (paFloat)p.z();
					//}

// x+=p.x();
					paGeomCollisionCalc(&d, &r);


					// Experimental options in mapping kinematic parameters onto surface kinematic parameters.
					// Should eventually be contained in generators, and properties set for surfaces.

//					cd.speedContactRelBody1 = r.speedContactRelBody1;
//					cd.speedContactRelBody2 = r.speedContactRelBody2;
					//! Compromise to compensate for contact vel being hard to track directly, because eg Bullet contacts too short lived.
//					cd.speedContactRelBody1 = 6.0;
					cd.speedContactRelBody1 = r.tangentSpeedBody1RelBody2 + r.speedBody1RelBody2;		// 2nd speed is rel CM speed. 
																										//! Const is useful for disks. Note this prevents gens from being quiet, which keeps res s active too.
					cd.speedContactRelBody2 = cd.speedContactRelBody1;



					// Slip speed is the highest quality parameter available, because it is derived from velocities.
//					cd.speedBody1RelBody2 = r.tangentSpeedBody1RelBody2;		// Basic

//					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2
//											+ r.normalSpeedBody1RelBody2*10.0);

					// Boost slip with normal speed, to simulate impact using a contact.
					//! Contact power prop. speed x force. Try lateral force for more accuracy?
					//! These formula should be hidden in the contact generator.
					//! cd.contactForce controls amplitude, cd.speedBody1RelBody2 controls brightness (like hardness effect)
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;



//					cd.contactForce += pt.m_appliedImpulse /m_dt;										// Basic - doesn't work with constant in  cd.speedContactRelBody1 above
//					cd.contactForce += (r.tangentSpeedBody1RelBody2 + r.normalSpeedBody1RelBody2 > 0)	// Fixes above.
//										? pt.m_appliedImpulse /m_dt : 0;
					//					cd.contactForce = 100.0/sqrtf(numContacts);
					cd.contactForce += (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;


#ifdef PLASTIC
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;
					cd.contactForce += (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;
#endif

#ifdef WATER
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *0.1
											+ r.normalSpeedBody1RelBody2 *1.0
											+ r.speedBody1RelBody2 *10.0
											)
											* pt.m_appliedImpulse /m_dt;
					cd.contactForce += (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;
#endif

#ifdef GRAVEL1
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *10.0
											+ r.speedBody1RelBody2)
											* pt.m_appliedImpulse /m_dt;		//!! if last contact impulse is zero this will clear cd.speedBody1RelBody2. 


//					cd.speedBody1RelBody2 = //(r.tangentSpeedBody1RelBody2 *1.0);
////						r.normalSpeedBody1RelBody2;
//						r.speedBody1RelBody2;
////						* pt.m_appliedImpulse /m_dt;

#endif
#ifdef GRAVEL2
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *10.0
											+ r.speedBody1RelBody2)
											* pt.m_appliedImpulse /m_dt;
//printf("%f %f %f %f\n", r.tangentSpeedBody1RelBody2, r.normalSpeedBody1RelBody2, r.speedBody1RelBody2, pt.m_appliedImpulse );

#endif

#ifdef FOIL
					cd.speedContactRelBody1 = r.speedBody1RelBody2;		// 2nd speed is rel CM speed. 
#endif

#ifdef REEDS
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *10.0
											+ r.speedBody1RelBody2
											+ paGeomCalcMag(d.body2.angularVel) *500.0		// Catch spin. eg cyl spining on point.
											)
											* pt.m_appliedImpulse /m_dt;

					cd.speedContactRelBody2 = cd.speedContactRelBody1 + paGeomCalcMag(d.body2.angularVel);

//printf("%f %f %f %f\n", r.tangentSpeedBody1RelBody2, r.normalSpeedBody1RelBody2, r.speedBody1RelBody2, pt.m_appliedImpulse );

#endif
				}

			}
		}
	}
//printf("%f\n%f\n%f\n\n",cd.contactForce,cd.speedBody1RelBody2,cd.speedContactRelBody2);

//if (cd.contactForce > 0.0) cd.contactForce = 20.0;

		paContact::pool.firstActiveObject();
		paContact* ac = paContact::pool.getNextActiveObject();		// Expect at most one.

  //     	if (!ac)  
		//{
		//	g_alpha = 1.0;		// New contact, make force smoother fast initially;
		//}

		//g_f = g_alpha * cd.contactForce + (1.0f - g_alpha) * g_f;
		//g_alpha -= 0.01;
		//if (g_alpha <= 0.02) g_alpha = 0.02;

		//g_s = g_alpha * cd.speedBody1RelBody2 + (1.0f - g_alpha) * g_s;


		if (!ac && numManifolds>0) // && cd.contactForce > 0)
		{
			ac = paContact::newContact();
			if (ac)
			{
				ac->setBody1((paBody*)obA->getUserPointer());		// body->abody, set up in initPhysics.
				ac->setBody2((paBody*)obB->getUserPointer());
//				ac->setUserData((void *)(&pt));					// contact<->acontact cross ref.
//				pt.m_userPersistentData = (void *)ac;		
			}
		}

//		else if (ac && cd.contactForce == 0) ac->fadeAndDelete();	// Keep one contact alive to make transitions easier to manage.

		if (numContacts == 0)
		{	
			cd.contactForce = 0;
			cd.speedBody1RelBody2 = 0;
			cd.speedContactRelBody1 = 0;
			cd.speedContactRelBody2 = 0;
		}

//printf("%f\n", cd.speedBody1RelBody2);

		if (ac)
		{
//			cd.contactForce = g_f;
			ac->setDynamicData(&cd);
//printf("%f\n", cd.speedBody1RelBody2); //! Fluctuating, why? sounds good. Better to have added controlled random factors.

		}


		if (g_inContact == false  &&  cd.contactForce > 0.0f) // && r.normalVelBody1RelBody2 < -0.2)
		{
			g_inContact = true;

			paImpact* ai = paImpact::newImpact();
//printf("%d\n", ai);
			if (ai)
			{
//printf("n");
				ai->setBody1((paBody*)obA->getUserPointer());
				ai->setBody2((paBody*)obB->getUserPointer());
				paImpactDynamicData id;
				id.impactImpulse = cd.contactForce*0.1;	//!! //r.normalSpeedBody1RelBody2; pt.m_appliedImpulse*0.01;    //; // 
				id.relNormalSpeedAtImpact = r.normalSpeedBody1RelBody2;
				id.relTangentSpeedAtImpact = r.tangentSpeedBody1RelBody2;
				ai->setDynamicData(&id);
			}
		}

		if (numContacts == 0) g_inContact = false;
//		if (cd.contactForce == 0.0f ) g_inContact = false;
		// Pass dynamic contact data.

//x /= impulseSum;
//printf("%f	%f	%f\n", x, gx, x-gx);
//gx = x;



	return 0;
}







//Phya
static bool PhyaContactDestroyedCallback(void* userPersistentData)
{	
	if (userPersistentData != 0)								// paContact may not have been available.
	{
		((paContact*)userPersistentData)->fadeAndDelete();		// Free paContact.
//printf("- %d\n", userPersistentData);
//printf("- %d	%d\n", userPersistentData, ((paContact*)userPersistentData)->getUserData());
//gc--;
	}
	return false; //No friction calc.
}

//static bool PhyaContactAddedCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
//{
//// Currently broken in Bullet??
////printf("\naaaaaaaaaaaaa %x\n",(int)(&cp));
////	cp.m_userPersistentData = (void *)1;
//
//	return false; //No friction calc.
//}
//







//experimental jitter damping (1 = no damping, 0 = total damping once motion below threshold)
extern btScalar gJitterVelocityDampingFactor;

//extern int gNumManifold;
//extern int gOverlappingPairs;
//extern int gTotalContactPoints;

void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


#ifdef USE_KINEMATIC_GROUND
	//btQuaternion kinRotation(btVector3(0,0,1),0.);
	btVector3 kinTranslation(-0.01,0,0);
	//kinematic object
	btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[0];
	//is this a rigidbody with a motionstate? then use the motionstate to update positions!
	if (btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
	{
		btTransform newTrans;
		btRigidBody::upcast(colObj)->getMotionState()->getWorldTransform(newTrans);
		newTrans.getOrigin()+=kinTranslation;
		btRigidBody::upcast(colObj)->getMotionState()->setWorldTransform(newTrans);
	} else
	{
		m_dynamicsWorld->getCollisionObjectArray()[0]->getWorldTransform().getOrigin() += kinTranslation;
	}

#endif //USE_KINEMATIC_GROUND

#ifdef SLEEP
Sleep(5);		// Limit time doing physics/graphics
#endif
	m_dt = getDeltaTimeMicroseconds() * 0.000001f * 2.0f;  //! Time scale factor. Equivalent length scale factor L = T*T. As m_dt incr, num sim steps should incr too.
	
//	printf("m_dt = %f: ",m_dt);
	
	if (m_dynamicsWorld)
	{

//#define FIXED_STEP 1
#ifdef FIXED_STEP
  		m_dynamicsWorld->stepSimulation(1.0f/60.f,0);
	
#else
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			m_dt = 1.0/420.f;


		int numSimSteps = 0;
		numSimSteps = m_dynamicsWorld->stepSimulation(m_dt,maxSimSubSteps);
		
		//Phya
		paLock();
#ifdef MANIFOLDCOLLISION
				PhyaUpdateManifoldCollision();			// Manage single persistent manifold collision.
#else
			PhyaUpdateCollisions();					// Start new contacts, and update existing.
#endif
		paUnlock();

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif
	}
	
#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 
	
	renderme(); 

// Display Contacts

#ifdef DISPLAY_CONTACTS	
{
	int numManifolds = m_dispatcher->getNumManifolds();
	int i;
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();
		paFloat x=0;
// if (numContacts>1) numContacts =1;  // See how much a single persistent contact jumps around.
		for (int j=0;j<numContacts;j++)
		{  //break;
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			glBegin(GL_LINES);
			glColor3f(1, 0, 1);
			
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();

//			glVertex3d(ptA.x(),ptA.y(),ptA.z());
						glVertex3d(0.0,0.0,0.0);

			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}
	}
}
#endif

	//render the graphics objects, with center of mass shift

		updateCamera();



#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	//printf("num manifolds: %i\n",gNumManifold);
	//printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	//printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	//gTotalContactPoints = 0;
	glutSwapBuffers();
}

void CcdPhysicsDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	glutSwapBuffers();
}





///User-defined friction model, the most simple friction model available: no friction

float myFrictionModel(	btRigidBody& body1,	btRigidBody& body2,	btManifoldPoint& contactPoint,	const btContactSolverInfo& solverInfo	)
{
	//don't do any friction
	return 0.f;
}


//Phya
// Pointers (body -> abody) are set in initPhysics
void	CcdPhysicsDemo::initAudio()
{

	paSetnFramesPerSecond(44100);

	m_funsurf = new paFunSurface;			// Function surface.. uses surface functions, combined with

	//m_funsurf->setGainAtRoll(1.0f);						// Boost (or reduce) volume towards rolling.
	//m_funsurf->setGainBreakSlipSpeed(10.0f);
	m_funsurf->setContactAmpMax(200.0);
	m_funsurf->setContactDamping(1.0f);	//1.1						// Factor damping is modified by when in contact.


	m_funsurf->setHardness(100.0f);							// Impulse time = 1/hardness (limited).
	m_funsurf->setImpactAmpMax(1.0);
	m_funsurf->setImpulseToHardnessBreakpoint(0.0);
	m_funsurf->setImpulseToHardnessScale(1000.0);


#ifdef WAVSURF
	m_wavfun = new paWavFun;
	m_wavfun->setInterpOff();
	m_funsurf->setFun(m_wavfun);
	m_funsurf->setRateConstant(1.0f);						// Calibrate rate of surface generation (depends on fun used)
    m_funsurf->setContactDirectGain(1.0);	// 0.0 1000.0
	m_funsurf->setContactGain(0.05);
	m_funsurf->setSystemDecayAlpha(0.05);
	m_funsurf->setCutoffFreqAtRoll(500.0f); //50			// Adjust the rel body vel to filter cutoff mapping.
	m_funsurf->setCutoffFreqRate(40.0f);	//10.0 300.0	// Rate of change of cutoff freq with slip speed.
	m_funsurf->setQuietSpeed(0.00001f);
#elif defined FREEPARTICLESURF
	m_rndfun = new paRndFun;
	m_rndfun->setMin(0.0);
	m_rndfun->setZeroRate(-1.0f);							
	m_funsurf->setFun(m_rndfun);							// Random bump surface texture for gravel impacts
//	m_funsurf->setRateMin(5.0f);
//	m_funsurf->setGainAtRoll(0.0f);
//	m_funsurf->setGainBreakSlipSpeed(10.0f);
#else
	m_rndfun = new paRndFun;								// Bump surface
	m_rndfun->setZeroRate(10.0f);							// Rate of surface returning to zero height, relative to main rate.
	m_funsurf->setFun(m_rndfun);							// Random bump surface texture.
	m_funsurf->setRateMin(5.0f);
	m_funsurf->setRateAtSpeed(100.0f,1.0f);					// Calibrate rate of surface generation (depends on fun used)
	m_funsurf->setContactGain(0.5f);
	// The following define the velocity to surface-filter map.
	// This rises linearly to a maximum frequency value.
	m_funsurf->setContactAmpMax(0.0f);
	m_funsurf->setCutoffFreqAtRoll(50.0f); //50				// Adjust the rel body vel to filter cutoff mapping.
	m_funsurf->setCutoffFreqRate(40.0f);	 //10.0 300.0	// Rate of change of cutoff freq with slip speed.
//	m_funsurf->setCutoffFreqMax(1000.0f);	//10 8000		// Useful for low res without high modes.
//	m_funsurf->setCutoffFreq2AtRoll(50.0f);					// Beef up rolling with optional extra filter layer.
//	m_funsurf->setCutoffFreq2Rate(300.0f);
	m_funsurf->setContactDamping(1.0f);	//1.1 1.3			// Factor damping is modified by when in contact.

	m_funsurf->setImpactGain(0.1f); 
	m_funsurf->setImpactAmpMax(1.0f);
	m_funsurf->setHardness(200.0f);						// Impulse time = 1/hardness (limited).
	m_funsurf->setImpulseToHardnessScale(5.0);			// AKA stiffness
	m_funsurf->setMaxHardness(5000.0);

//	m_funsurf->setSkidThickness(10.0);		//!! get skids working again.
#endif

#ifdef PLASTIC
	if (m_wavfun->readWav("../resource/plastic.wav") == -1)
		m_wavfun->readWav("plastic.wav");
	    m_funsurf->setContactDirectGain(0.2);	// 0.0 1000.0
		m_funsurf->setContactGain(0.05);
		m_funsurf->setSystemDecayAlpha(0.05);
		m_funsurf->setCutoffFreqAtRoll(700.0f); //50			// Adjust the rel body vel to filter cutoff mapping.
		m_funsurf->setCutoffFreqRate(5.0f);	//10.0 300.0	// Rate of change of cutoff freq with slip speed.
		m_funsurf->setImpactGain(0.0);

#endif
#ifdef WATER
	if (m_wavfun->readWav("../resource/water.wav") == -1)
		m_wavfun->readWav("water.wav");
		m_funsurf->setSystemDecayAlpha(0.02);

		m_funsurf->setContactGain(0.00001);
		m_funsurf->setContactAmpMin(0.0001);
		m_funsurf->setContactDirectGain(200.0);	// 0.0 1000.0
		m_funsurf->setContactAmpMax(0.003f);
		m_funsurf->setCutoffFreqAtRoll(200.0f); //50			// Adjust the rel body vel to filter cutoff mapping.
		m_funsurf->setCutoffFreqRate(3.0f);	//10.0 300.0	// Rate of change of cutoff freq with slip speed.
	    m_funsurf->setImpactGain(0.0);
#endif
#ifdef LEAVES
	if (m_wavfun->readWav("../resource/leaves.wav") == -1)
		m_wavfun->readWav("leaves.wav");
		m_funsurf->setSystemDecayAlpha(0.016);
		m_funsurf->setCutoffFreqAtRoll(20000.0f); //50			// Adjust the rel body vel to filter cutoff mapping.
		m_funsurf->setCutoffFreqMax(2000.0f);					// Falling filter!! Opposite of other surfaces.
		m_funsurf->setCutoffFreqRate(2.0f);
		m_funsurf->setContactAmpMax(5.0);
		m_funsurf->setContactGain(0.003);
		m_funsurf->setContactDirectGain(1.0);
		m_funsurf->setImpactGain(0.0);
#endif


#ifdef GRAVEL1
	m_funsurf->setRateAtSpeed(20.0f,1.0f);					// Calibrate rate of surface generation (depends on fun used)
	m_funsurf->setRateMax(60.0f);
	m_funsurf->setRateMin(0.0f);
	m_funsurf->setContactGain(500000.0);
	m_funsurf->setContactAmpMin(4000.0);
	m_funsurf->setContactAmpMax(40000.0);
    m_funsurf->setContactDirectGain(15.0);	// 0.0 1000.0
	m_funsurf->setImpactGain(0.0);
	m_funsurf->setCutoffFreqAtRoll(200.0f); //50			// Adjust the rel body vel to filter cutoff mapping.
	m_funsurf->setCutoffFreqRate(100.0);		//10.0 300.0	// Rate of change of cutoff freq with slip speed.
	m_funsurf->setCutoffFreqMax(10000.0f);					
	m_funsurf->setSystemDecayAlpha(0.01);
#endif
#ifdef GRAVEL2
	m_funsurf->setRateAtSpeed(50.0f,1.0f);					// Calibrate rate of surface generation (depends on fun used)
	m_funsurf->setRateMax(200.0f);
	m_funsurf->setRateMin(0.0f); //20
	m_funsurf->setContactGain(5000.0);
	m_funsurf->setContactAmpMin(5000.0);
	m_funsurf->setContactAmpMax(10000.0);
    m_funsurf->setContactDirectGain(60.0);	//60// 0.0 1000.0	//!Dry instead of direct
	m_funsurf->setImpactGain(0.0);
	m_funsurf->setCutoffFreqAtRoll(800.0f); //50				// Adjust the rel body vel to filter cutoff mapping.
	m_funsurf->setCutoffFreqRate(200.0f);	 //10.0 300.0	// Rate of change of cutoff freq with slip speed.
	m_funsurf->setCutoffFreqMax(4000.0f);					
	m_funsurf->setSystemDecayAlpha(0.04);
	m_funsurf->setContactDamping(1.5);			//! damping doesn't work in simple set up.
//	m_funsurf->setGainAtRoll(100.0f);
//	m_funsurf->setGainBreakSlipSpeed(1.0f);
#endif
#ifdef FOIL
	m_funsurf->setRateAtSpeed(4.0f,1.0f);					// Calibrate rate of surface generation (depends on fun used)
	m_funsurf->setRateMax(40.0f);
	m_rndfun->setMin(0.0);
	m_rndfun->setZeroRate(10000.0f);
	m_funsurf->setContactGain(50.0);
	m_funsurf->setContactAmpMin(5000.0);
	m_funsurf->setContactAmpMax(20000.0);
    m_funsurf->setContactDirectGain(2000.0);	// 0.0 1000.0
	m_funsurf->setImpactGain(0.0);
	m_funsurf->setCutoffFreqAtRoll(4000.0f); //50				// Adjust the rel body vel to filter cutoff mapping.
	m_funsurf->setCutoffFreqRate(15.0f);	 //10.0 300.0	// Rate of change of cutoff freq with slip speed.
	m_funsurf->setCutoffFreqMax(12000.0f);					// Falling filter!! Opposite of other surfaces.
	m_funsurf->setSystemDecayAlpha(0.15);
	m_funsurf->setContactDamping(4.0);			//! damping doesn't work in simple set up.
#endif
#ifdef REEDS
	m_funsurf->setRateAtSpeed(5.0f,1.0f);					// Calibrate rate of surface generation (depends on fun used)
	m_funsurf->setContactGain(10.0);
	m_funsurf->setContactAmpMin(5000.0);
	m_funsurf->setContactAmpMax(20000.0);
    m_funsurf->setContactDirectGain(400.0);	// 0.0 1000.0
	m_funsurf->setCutoffFreqAtRoll(2000.0f); //50				// Adjust the rel body vel to filter cutoff mapping.
	m_funsurf->setCutoffFreqRate(3.0f);	 //10.0 300.0	// Rate of change of cutoff freq with slip speed.
	m_funsurf->setCutoffFreqMax(5000.0f);					// Falling filter!! Opposite of other surfaces.
	m_funsurf->setSystemDecayAlpha(0.01);
	m_funsurf->setContactDamping(1.8);			//! damping doesn't work in simple set up.
#endif



#ifdef IMPACTSAMPLE
//	 ("../../modan/bintop.wav");
	if (m_funsurf->setImpactSample("../resource/impactSample1.wav") == -1)
		m_funsurf->setImpactSample("impactSample1.wav");	// Try local, eg for distrb exe.
	if (m_funsurf->setImpactSample("../resource/impactSample2.wav") == -1)
		m_funsurf->setImpactSample("impactSample2.wav");	// Try local, eg for distrb exe.
	if (m_funsurf->setImpactSample("../resource/impactSample3.wav") == -1)
		m_funsurf->setImpactSample("impactSample3.wav");	// Try local, eg for distrb exe.
	if (m_funsurf->setImpactSample("../resource/impactSample4.wav") == -1)
		m_funsurf->setImpactSample("impactSample4.wav");	// Try local, eg for distrb exe.
	if (m_funsurf->setImpactSample("../resource/impactSample5.wav") == -1)
		m_funsurf->setImpactSample("impactSample5.wav");	// Try local, eg for distrb exe.
	if (m_funsurf->setImpactSample("../resource/impactSample6.wav") == -1)
		m_funsurf->setImpactSample("impactSample6.wav");	// Try local, eg for distrb exe.

	m_funsurf->setImpactGain(0.000001f); 
	m_funsurf->setImpactAmpMax(1.0f);
	m_funsurf->setImpactDirectGain(1000.0f);			//!! No independent control for gain to main resonator.
	m_funsurf->setImpactCrossGain(0.0f);
	m_funsurf->setHardness(10.0f);						// Impulse time = 1/hardness (limited).
	m_funsurf->setImpulseToHardnessScale(5.0);			// AKA stiffness


	//m_funsurf->setContactAmpMax(200.0f);
	//m_funsurf->setContactGain(1.0f);
	//m_funsurf->setHardness(10.0f);						// Impulse time = 1/hardness (limited).
	//m_funsurf->setImpulseToHardnessScale(1.0);			// AKA stiffness
	//m_funsurf->setImpactGain(10000.0f);
	//m_funsurf->setImpactAmpMax(0.1);


#endif







	m_modes = new paModalData;

#ifndef IMPACTSAMPLE
	if (m_modes->read("../resource/eg.md") == -1)
		m_modes->read("eg.md");	// Try local, eg for distrb exe.
#else
	if (m_modes->read("../resource/impactSample.md") == -1)
		m_modes->read("impactSample.md");	// Try local, eg for distrb exe.
#endif

int i;
 
	// Impact cutoffs set to match different base frequencies, for more natural sound
#if NBODIES == 2
	paFloat cutoff[NBODIES] = {-1.0f, -1.0f};
#else
	paFloat cutoff[NBODIES] = {100.0f, 300.0f, 400.0f, 500.0f, 1000.0f, 1000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f};
#endif

	for(i=1; i< NBODIES; i++) {
		paModalRes *mr = m_res[i] = new paModalRes;
		mr->setData(m_modes);
		mr->setQuietLevel(1.0f);		// Determines at what rms envelope level a resonator will be 
										// faded out when no longer in contact, to save cpu.
										// Make bigger to save more cpu, but possibly truncate decays notceably.
								
		mr->setnActiveModes(50);		// Can trade detail for speed.
		mr->setAuxAmpScale(2.0f);
		mr->setAuxDampScale(.5f);

#ifndef IMPACTSAMPLE
		mr->setAuxFreqScale(0.2f + 0.2f*i);		// Give a different base frequency to each object.
#endif
//		mr->setAuxFreqScale(0.5f + 0.2f*i);
//		mr->setAuxFreqScale(0.8f + 0.1f*i);
#ifdef FOIL
//		mr->setAuxFreqScale(3.0f);
//		mr->setAuxDampScale(2.0f);
#endif		
		mr->setMaxContactDamping(3.0);
		m_abody[i] = new paBody;
#ifndef NORES
		m_abody[i]->setRes(mr);			// NB Possible to have several bodies using one res for efficiency.
#endif
		m_funsurfn[i] = new paFunSurface;
		*m_funsurfn[i] = *m_funsurf;		// Copy template.
		m_funsurfn[i]->setCutoffFreqMax(cutoff[i]);
		m_abody[i]->setSurface(m_funsurfn[i]);
//		m_abody[i]->setContactGain(10.0);
//		m_abody[i]->setContactAmpMax(3000.0f);		// Acts like a control signal compressor/limiter. Use to define range of expression.
	}


	//// Create pools and lists that enable efficient
	//// dynamic collision management.

	paBlock::setnMaxFrames(128);  //128							// The size of each audio block.
	paBlock::pool.allocate(NBODIES+5);							// Num resonators+5.
	paContact::pool.allocate(6*NBODIES);
	paImpact::pool.allocate(6*NBODIES);
	paFunSurface::contactGenPool.allocate(12*NBODIES);			// Each Contact and Impact takes 2 contactGens.
	paFunSurface::impactGenPool.allocate(12*NBODIES);			// Each Impact takes 2 impactGens.
	paRes::activeResList.allocate(NBODIES);						// Num resonators.

}

//Phya 
void	CcdPhysicsDemo::startAudio() 
{

	//// Configure simple mono output stream.
	paSetnStreamBufFrames(256); //256 2048			// Num of frames in single write to device.
	paSetnDeviceBufFrames(2048);					// Size of device's internal buffer. (The AIO library used is intended for demonstration, and is not optimized)
	paOpenStream();					

	//// Alternatively use output callbacks to route seperate resonator outputs to a 3d audio system.
//	paSetMultipleOutputCallback( multipleOutputCallback );

	// Phya includes a look-ahead limiter class for preventing clipping.
	// Interface to a master limiter is provided here for convenience.
	// Minimize clipping first, before adding limiting.
	// Per object limiting could be better, and is neccessary for 3D sound.
	// - see MultipleOutputCallback()
#ifdef LIMITER
	paSetLimiter(0.005f, 0.000f, 0.800f);
//	paSetLimiter(0.001f, 0.001f, 0.001f);

#endif


	paInit();								// If just using paTick()directly you need paTickInit() here instead.
#ifdef AUDIOTHREAD
	paStartThread();
#endif
	// Play test impact.

	//m_abody[0]->getRes()->setAuxFreqScale(0.2f);
	//m_abody[0]->getRes()->setAuxAmpScale(0.1f);
	//m_abody[0]->getSurface()->setHardness(100000.0f);  // Impulse time = 1/hardness (limited).

	//{
	//	paLock();
	//	paContact* contact = new paContact;
	//	paImpact* impact = new paImpact;
	//	paContactDynamicData contactData;
	//	paImpactDynamicData impactData;

	//	impact = paImpact::newImpact();
	//	impact->setBody1(m_abody[0]);
	//	impactData.relTangentSpeedAtImpact = 0; // No skid.
	//	impactData.impactImpulse = 1.0;
	//	impact->setDynamicData(&impactData);
	//	paUnlock();
	//}

}




void	CcdPhysicsDemo::initPhysics()
{
// Setup Bullet physics and pointers to Phya bodies

#ifdef _RELEASE
#endif

m_enableshadows=!m_enableshadows;

#ifdef NOHELP
if (m_debugMode & btIDebugDraw::DBG_NoHelpText)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoHelpText);
else
	m_debugMode |= btIDebugDraw::DBG_NoHelpText;
#endif


#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif //
#endif

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	m_collisionShapes.push_back(new btStaticPlaneShape(btVector3(0,1,0),0.5));
#else

	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346
	m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));
#endif

//#define CUBE_HALF_EXTENTS 1.5
#ifdef DO_BENCHMARK_PYRAMIDS
	m_collisionShapes.push_back(new btBoxShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
#else
#ifdef DISK
	m_collisionShapes.push_back(new btCylinderShape (btVector3(2.0*CUBE_HALF_EXTENTS,0.2*CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));	// overlapping disks
#elif defined CAN
	m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));			// cans
#endif
//	m_collisionShapes.push_back(new btCylinderShape (btVector3(2*CUBE_HALF_EXTENTS,2*CUBE_HALF_EXTENTS,2*CUBE_HALF_EXTENTS)));
//	m_collisionShapes.push_back(new btCylinderShape (btVector3(7*CUBE_HALF_EXTENTS,0.5*CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
//	m_collisionShapes.push_back(new btBoxShape (btVector3(7*CUBE_HALF_EXTENTS,0.5*CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
#endif
	



	m_dispatcher=0;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	
#ifdef USE_PARALLEL_DISPATCHER
int maxNumOutstandingTasks = 4;

#ifdef USE_WIN32_THREADING

	m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else

#ifdef USE_LIBSPE2

   spe_program_handle_t * program_handle;
#ifndef USE_CESOF
                        program_handle = spe_image_open ("./spuCollision.elf");
                        if (program_handle == NULL)
                    {
                                perror( "SPU OPEN IMAGE ERROR\n");
                    }
                        else
                        {
                                printf( "IMAGE OPENED\n");
                        }
#else
                        extern spe_program_handle_t spu_program;
                        program_handle = &spu_program;
#endif
        SpuLibspe2Support* threadSupportCollision  = new SpuLibspe2Support( program_handle, maxNumOutstandingTasks);
#endif //USE_LIBSPE2

///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
/// For Unix/Mac someone could implement a pthreads version of btThreadSupportInterface?
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif


	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
//	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#else
	
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

#ifdef USE_CUSTOM_NEAR_CALLBACK
	//this is optional
	m_dispatcher->setNearCallback(customNearCallback);
#endif

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
/// For large worlds or over 16384 objects, use the bt32BitAxisSweep3 broadphase
//	m_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
/// When trying to debug broadphase issues, try to use the btSimpleBroadphase
//	m_broadphase = new btSimpleBroadphase;
	
	//box-box is in Extras/AlternativeCollisionAlgorithms:it requires inclusion of those files

#ifdef COMPARE_WITH_QUICKSTEP
	m_solver = new btOdeQuickstepConstraintSolver();
#else

	
#ifdef USE_PARALLEL_SOLVER

	m_threadSupportSolver = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"solver",
								processSolverTask,
								createSolverLocalStoreMemory,
								maxNumOutstandingTasks));

	m_solver = new btParallelSequentialImpulseSolver(m_threadSupportSolver,maxNumOutstandingTasks);
#else
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
	
#endif //USE_PARALLEL_SOLVER
#endif
		
#ifdef	USER_DEFINED_FRICTION_MODEL
	//user defined friction model is not supported in 'cache friendly' solver yet, so switch to old solver
		m_solver->setSolverMode(btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
#endif //USER_DEFINED_FRICTION_MODEL

		btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
		m_dynamicsWorld = world;


		m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
		m_dynamicsWorld->setGravity(btVector3(0,-10,0));

		

#ifdef USER_DEFINED_FRICTION_MODEL
	{
		//m_solver->setContactSolverFunc(ContactSolverFunc func,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
		m_solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,DEFAULT_CONTACT_SOLVER_TYPE);
		m_solver->SetFrictionSolverFunc(myFrictionModel,DEFAULT_CONTACT_SOLVER_TYPE,USER_CONTACT_SOLVER_TYPE1);
		m_solver->SetFrictionSolverFunc(myFrictionModel,USER_CONTACT_SOLVER_TYPE1,USER_CONTACT_SOLVER_TYPE1);
		//m_physicsEnvironmentPtr->setNumIterations(2);
	}
#endif //USER_DEFINED_FRICTION_MODEL



	btTransform tr;
	tr.setIdentity();

	int i;
	
	for (i=0;i<gNumObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;
		}
		else
			shapeIndex[i] = 0;
	}



	for (i=0;i<gNumObjects;i++)
	{
		btCollisionShape* shape = m_collisionShapes[shapeIndex[i]];
		shape->setMargin(gCollisionMargin);

		bool isDyna = i>0;

		btTransform trans;
		trans.setIdentity();
		
		if (i>0)
		{
			int col = i-gNumObjects/2;
			btVector3 pos(col*2*CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS + EXTRA_HEIGHT,5);
			trans.setOrigin(pos);
		} else
		{
			trans.setOrigin(btVector3(0,EXTRA_HEIGHT-CUBE_HALF_EXTENTS,5));
		}

		float mass = 1.f;

		if (!isDyna)
			mass = 0.f;
	
		btRigidBody* body = localCreateRigidBody(mass,trans,shape);

		///////////////////////////////////////////////////////////////////////////////
		//     
		// Point to Phya body from Bullet body.
		if (i > 0) body->setUserPointer((void*)(m_abody[i]));
		if (i == 0) body->setUserPointer((void*)0);


		// Setup manifold point callbacks.
//		gContactAddedCallback = PhyaContactAddedCallback;		// Not working correctly.
#ifndef MANIFOLDCOLLISON 
		gContactDestroyedCallback = PhyaContactDestroyedCallback;
#endif

//		body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

#ifdef USE_KINEMATIC_GROUND
		if (mass == 0.f)
		{
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
#endif //USE_KINEMATIC_GROUND
		
		
		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		body->setCcdMotionThreshold( CUBE_HALF_EXTENTS );
		//Experimental: better estimation of CCD Time of Impact:
		body->setCcdSweptSphereRadius( 0.2*CUBE_HALF_EXTENTS );

#ifdef USER_DEFINED_FRICTION_MODEL	
		///Advanced use: override the friction solver
		body->m_frictionSolverType = USER_CONTACT_SOLVER_TYPE1;
#endif //USER_DEFINED_FRICTION_MODEL

	}


//	clientResetScene();


}
	





void	CcdPhysicsDemo::exitPhysics()
{


	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;
#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	if (m_threadSupportSolver)
	{
		delete m_threadSupportSolver;
	}
#endif
#endif

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

#ifdef USE_PARALLEL_DISPATCHER
#ifdef WIN32
	if (m_threadSupportCollision)
	{
		delete m_threadSupportCollision;
	}
#endif
#endif

	delete m_collisionConfiguration;

	
}




void	CcdPhysicsDemo::exitAudio()
{

	paStopThread();

	paBlock::pool.deallocate();
	paContact::pool.deallocate();
	paImpact::pool.deallocate();
	paFunSurface::contactGenPool.deallocate();
	paFunSurface::impactGenPool.deallocate();
	paRes::activeResList.deallocate();

	delete m_rndfun;
	delete m_wavfun;
	delete m_funsurf;
	delete m_modes;

	int i;
	for(i=0; i< 3; i++) {
		delete m_res[i];
		delete m_abody[i];
		delete m_funsurfn[i];
	}
}


void CcdPhysicsDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key=='q')
	{
		paStopThread();
	}

	DemoApplication::keyboardCallback(key, x, y);
}