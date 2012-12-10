#define DO_WALL 1

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

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
bool useCompound = false;

const int gNumObjects = NBODIES;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cpp

const int maxNumObjects = 32760;

int	shapeIndex[maxNumObjects];

#define CUBE_HALF_EXTENTS 1.5  //.5

#define EXTRA_HEIGHT -10.f

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
		}
	}
}

paFloat x = 0;
int CcdPhysicsDemo::PhyaUpdateCollisions()
{
	// input : m_dispatcher, m_dt
	int numManifolds = m_dispatcher->getNumManifolds();
	int i;
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
		btCollisionObject* obA = (btCollisionObject*)(contactManifold->getBody0());
		btCollisionObject* obB = (btCollisionObject*)(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();

		for (int j=0;j<numContacts;j++)
		{  //break;
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			paContact* ac = (paContact*)(pt.m_userPersistentData);
			bool firstContact = false;

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
				}
				firstContact = true;	// Indicates should test for impact below..
			}

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
	
				{
					const btVector3 &n = pt.m_normalWorldOnB;
					d.normal[0] = n.x();
					d.normal[1] = n.y();
					d.normal[2] = n.z();

					const btVector3 &p = pt.getPositionWorldOnB();
					d.contactPos[0] = p.x();
					d.contactPos[1] = p.y();
					d.contactPos[2] = p.z();

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

						lp[0] = (paFloat)p.x();
						lp[1] = (paFloat)p.y();
						lp[2] = (paFloat)p.z();
					}

					paGeomCollisionResult r;
					paGeomCollisionCalc(&d, &r);

					cd.speedContactRelBody1 = r.tangentSpeedBody1RelBody2 + r.speedBody1RelBody2;		// 2nd speed is rel CM speed. 
					cd.speedContactRelBody2 = cd.speedContactRelBody1;

					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;

					cd.contactForce =
										(r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 * 1.0)
											* pt.m_appliedImpulse /m_dt;

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
		}
	}

	return 0;
}

paFloat g_f = 0;
paFloat g_s = 0;
paFloat g_alpha;
bool g_inContact = false;

int CcdPhysicsDemo::PhyaUpdateManifoldCollision()
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
	
		numContacts = contactManifold->getNumContacts();

		cd.contactForce = 0;	// Sum forces from contacts in each manifold.

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
				{
					const btVector3 &n = pt.m_normalWorldOnB;
					d.normal[0] = n.x();
					d.normal[1] = n.y();
					d.normal[2] = n.z();

					const btVector3 &p = pt.getPositionWorldOnB();
					d.contactPos[0] = p.x();
					d.contactPos[1] = p.y();
					d.contactPos[2] = p.z();

					d.calcContactVel = false;

					paGeomCollisionCalc(&d, &r);

					cd.speedContactRelBody1 = r.tangentSpeedBody1RelBody2 + r.speedBody1RelBody2;		// 2nd speed is rel CM speed. 
					cd.speedContactRelBody2 = cd.speedContactRelBody1;

					// Boost slip with normal speed, to simulate impact using a contact.
					cd.speedBody1RelBody2 = (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;

					cd.contactForce += (r.tangentSpeedBody1RelBody2 *1.0
											+ r.normalSpeedBody1RelBody2 *1.0)
											* pt.m_appliedImpulse /m_dt;

				}
			}
		}
	}

		paContact::pool.firstActiveObject();
		paContact* ac = paContact::pool.getNextActiveObject();		// Expect at most one.

		if (!ac && numManifolds>0) // && cd.contactForce > 0)
		{
			ac = paContact::newContact();
			if (ac)
			{
				ac->setBody1((paBody*)obA->getUserPointer());		// body->abody, set up in initPhysics.
				ac->setBody2((paBody*)obB->getUserPointer());
			}
		}

		if (numContacts == 0)
		{	
			cd.contactForce = 0;
			cd.speedBody1RelBody2 = 0;
			cd.speedContactRelBody1 = 0;
			cd.speedContactRelBody2 = 0;
		}

		if (ac)
		{
			ac->setDynamicData(&cd);
		}


		if (g_inContact == false  &&  cd.contactForce > 0.0f) // && r.normalVelBody1RelBody2 < -0.2)
		{
			g_inContact = true;

			paImpact* ai = paImpact::newImpact();
			if (ai)
			{
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

	return 0;
}

static bool PhyaContactDestroyedCallback(void* userPersistentData)
{	
	if (userPersistentData != 0)								// paContact may not have been available.
	{
		((paContact*)userPersistentData)->fadeAndDelete();		// Free paContact.
	}
	return false; //No friction calc.
}

//experimental jitter damping (1 = no damping, 0 = total damping once motion below threshold)
extern btScalar gJitterVelocityDampingFactor;

void CcdPhysicsDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	m_dt = getDeltaTimeMicroseconds() * 0.000001f * 2.0f;  //! Time scale factor. Equivalent length scale factor L = T*T. As m_dt incr, num sim steps should incr too.
	
//	printf("m_dt = %f: ",m_dt);
	
	if (m_dynamicsWorld)
	{

//#define FIXED_STEP 1
#ifdef FIXED_STEP
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
#else
			PhyaUpdateCollisions();					// Start new contacts, and update existing.
#endif
		paUnlock();

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
#endif
	}
		
	renderme(); 
	//render the graphics objects, with center of mass shift
	updateCamera();

	glFlush();
	//some additional debugging info

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
	m_funsurf = new paFunSurface;			// Function surface.. uses surface functions, combined with

	//m_funsurf->setGainAtRoll(1.0f);						// Boost (or reduce) volume towards rolling.
	//m_funsurf->setGainBreakSlipSpeed(10.0f);
	m_funsurf->setContactAmpMax(200.0);
	m_funsurf->setContactDamping(1.0f);	//1.1						// Factor damping is modified by when in contact.


	m_funsurf->setHardness(100.0f);							// Impulse time = 1/hardness (limited).
	m_funsurf->setImpactAmpMax(1.0);
	m_funsurf->setImpulseToHardnessBreakpoint(0.0);
	m_funsurf->setImpulseToHardnessScale(1000.0);

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

	m_modes = new paModalData;

#ifndef IMPACTSAMPLE
	if (m_modes->read("../resource/Pickbreak.md") == -1)
		m_modes->read("eg.md");	// Try local, eg for distrb exe.
#endif

int i;
 
	// Impact cutoffs set to match different base frequencies, for more natural sound
#if NBODIES == 2
#else
	paFloat cutoff[NBODIES] = {100.0f, 300.0f, 400.0f, 500.0f, 1000.0f, 1000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f};
#endif

	for(i=1; i< NBODIES; i++) {
		paModalRes *mr = m_res[i] = new paModalRes;
		mr->setData(m_modes);
		mr->setQuietLevel(1.0f);		// Determines at what rms envelope level a resonator will be 
										// faded out when no longer in contact, to save cpu.
										// Make bigger to save more cpu, but possibly truncate decays notceably.
								
		//mr->setnActiveModes(50);		// Can trade detail for speed.
		mr->setAuxAmpScale(2.0f);
		mr->setAuxDampScale(.5f);

		mr->setMaxContactDamping(3.0);
		m_abody[i] = new paBody;

#ifndef NORES
		m_abody[i]->setRes(mr);			// NB Possible to have several bodies using one res for efficiency.
#endif

		m_funsurfn[i] = new paFunSurface;
		*m_funsurfn[i] = *m_funsurf;		// Copy template.
		m_funsurfn[i]->setCutoffFreqMax(cutoff[i]);
		m_abody[i]->setSurface(m_funsurfn[i]);
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
	paSetnFramesPerSecond(44100);

	//// Configure simple mono output stream.
	paSetnStreamBufFrames(256); //256 2048			// Num of frames in single write to device.
	paSetnDeviceBufFrames(2048);					// Size of device's internal buffer. (The AIO library used is intended for demonstration, and is not optimized)
	paOpenStream();					

#ifdef LIMITER
	paSetLimiter(0.005f, 0.000f, 0.800f);
//	paSetLimiter(0.001f, 0.001f, 0.001f);

#endif

	paInit();								// If just using paTick()directly you need paTickInit() here instead.
#ifdef AUDIOTHREAD
	paStartThread();
#endif
}

void	CcdPhysicsDemo::initPhysics()
{

#ifdef _RELEASE
#endif

m_enableshadows=!m_enableshadows;

#ifdef NOHELP
if (m_debugMode & btIDebugDraw::DBG_NoHelpText)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoHelpText);
else
	m_debugMode |= btIDebugDraw::DBG_NoHelpText;
#endif

m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));

//#define CUBE_HALF_EXTENTS 1.5
#ifdef DO_BENCHMARK_PYRAMIDS
#else
#ifdef DISK
	m_collisionShapes.push_back(new btCylinderShape (btVector3(2.0*CUBE_HALF_EXTENTS,0.2*CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));	// overlapping disks
#elif defined CAN
#endif
#endif

	m_dispatcher=0;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	
#ifdef USE_PARALLEL_DISPATCHER
#else
	
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
#ifdef COMPARE_WITH_QUICKSTEP
#else

	
#ifdef USE_PARALLEL_SOLVER
#else
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;
#endif //USE_PARALLEL_SOLVER

#endif

		btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
		m_dynamicsWorld = world;

		m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
		m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	btTransform tr;
	tr.setIdentity();

	int i;
	
	for (i=0;i<gNumObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;//sphere
		}
		else
			shapeIndex[i] = 0;
	}

	if (useCompound)
	{
		btCompoundShape* compoundShape = new btCompoundShape();
		btCollisionShape* oldShape = m_collisionShapes[1];
		m_collisionShapes[1] = compoundShape;
		btVector3 sphereOffset(0,0,2);

		comOffset.setIdentity();

#ifdef CENTER_OF_MASS_SHIFT
#else
		compoundShape->addChildShape(tr,oldShape);
		tr.setOrigin(sphereOffset);
		compoundShape->addChildShape(tr,new btSphereShape(0.9));
#endif
	}

#ifdef DO_WALL

	doll = new RagDoll(m_dynamicsWorld, btVector3(0,2,15), 7.5f, m_abody );

	for (i=0;i<gNumObjects;i++)
	{
		btCollisionShape* shape = m_collisionShapes[shapeIndex[i]];
		shape->setMargin(gCollisionMargin);

		bool isDyna = i>0;

		btTransform trans;
		trans.setIdentity();
		
		if (i>0)
		{
			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}

			btVector3 pos(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,5);

			trans.setOrigin(pos);
		} else
		{
			trans.setOrigin(btVector3(0,EXTRA_HEIGHT-CUBE_HALF_EXTENTS,5));
		}

		float mass = 1.f;

		if (!isDyna)
			mass = 0.f;
	
		btRigidBody* body = localCreateRigidBody(mass,trans,shape);

		//Phya    
		// Point to Phya body from Bullet body.
		//if (i > 1) body->setUserPointer((void*)(m_abody[i]));

		// Setup manifold point callbacks.
//		gContactAddedCallback = PhyaContactAddedCallback;		// Not working correctly.
#ifndef MANIFOLDCOLLISON 
		gContactDestroyedCallback = PhyaContactDestroyedCallback;
#endif
		body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

#ifdef USE_KINEMATIC_GROUND
#endif //USE_KINEMATIC_GROUND
		
		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		body->setCcdMotionThreshold( CUBE_HALF_EXTENTS );
		//Experimental: better estimation of CCD Time of Impact:
		body->setCcdSweptSphereRadius( 0.2*CUBE_HALF_EXTENTS );
	}
#endif
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

	delete doll;
	doll = nullptr;

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;
#ifdef USE_PARALLEL_DISPATCHER
#endif

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

#ifdef USE_PARALLEL_DISPATCHER
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