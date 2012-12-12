#include "swing.h"

Swing::Swing( btDynamicsWorld* ownerWorld,
	const btVector3& jointPosition, 
	btScalar scale_swing, btRigidBody *ground ) : m_ownerWorld (ownerWorld)
{

//	scale_swing = 1.0;
	m_shape = new btCapsuleShape( btScalar(scale_swing * 0.1), btScalar(scale_swing) );

	btTransform offset;
	offset.setIdentity();
//	offset.setOrigin( positionOffset );

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin( btVector3( btScalar(0.0), btScalar(0.0), btScalar(0.0) ) );
	m_body = localCreateRigidBody( btScalar(1.0), offset * transform, m_shape );
	

	m_joint = new btPoint2PointConstraint(*m_body, *ground, 
		btVector3(btScalar(0.), btScalar(0.5 * scale_swing), btScalar(0.)),
		jointPosition
		);

	/* setup the a hinge */
	//btGeneric6DofConstraint * joint6DOF;
	//btTransform localA, localB;
	//bool useLinearReferenceFrameA = true;
	//localA.setIdentity();
	//localB.setIdentity();
	//localA.setOrigin(btVector3(btScalar(0.), btScalar(0.5 * scale_swing), btScalar(0.))); //! hinge relative to capsule centres
	//localB.setOrigin(jointPosition); //! hinge relative to ground
	//m_joint = new btGeneric6DofConstraint(*m_body, *ground, localA, localB, useLinearReferenceFrameA);

//	m_body->setRestitution(2.0f);
//	m_body->setRollingFriction(0.0f);

	m_body->setDamping( 0.1f, 0.7f );
	//m_body->setDeactivationTime( 0.0f );
	m_body->setSleepingThresholds( 0.0f, 0.0f );
	//m_body->setDamping( 0.05f, 0.85f );
	//m_body->setDeactivationTime( 0.8f );
	//m_body->setSleepingThresholds( 1.6f, 2.5f );
	m_body->setActivationState(DISABLE_DEACTIVATION);
	ground->setActivationState(DISABLE_DEACTIVATION);
	ground->setRestitution(0.0f);
	ground->setRollingFriction(0.0f);

	
#ifdef RIGID
	m_joint->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
	m_joint->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
	/* uncomment these to set the movement limits. */
	//joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
	//joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.5f,SIMD_EPSILON,SIMD_PI*0.3f));
#endif
	m_ownerWorld->addConstraint(m_joint, true);
}

Swing::~Swing()
{
	// delete any objects created on the heap here.
	delete m_shape;
	m_shape = nullptr;
	
	delete m_body;
	m_body = nullptr;

	delete m_joint;
	m_joint = nullptr;
}

btRigidBody* Swing::localCreateRigidBody( btScalar mass, const btTransform& startTransform, btCollisionShape* shape )
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}