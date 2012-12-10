#ifndef __SWING_H__
#define __SWING_H__

#include "DemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"

class Swing
{
public:
	Swing(btDynamicsWorld* ownerWorld, const btVector3& positionOffset, btScalar scale_swing, btRigidBody *ground);
	~Swing();
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
protected:
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shape;
	btRigidBody* m_body;
	btTypedConstraint* m_joint;
};

#endif // paired with line 1