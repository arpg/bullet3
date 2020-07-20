#include "btVehicle.h"

//constructor to create a car from an existing rigidbody
btVehicle::btVehicle() : 
	m_chassisObject(NULL),
	m_indexRightAxis(0),
	m_indexUpAxis(1),
	m_indexForwardAxis(2) 
{	
}

btVehicle::~btVehicle()
{
	// delete m_chassisObject;
}

btRigidBody* btVehicle::createLocalRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
	body->setWorldTransform(startTransform);
#endif  //

	body->setUserIndex(-1);
	// m_dynamicsWorld->addRigidBody(body);
	return body;
}


///btActionInterface interface
void btVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar step)
{
	(void)collisionWorld;
	updateVehicle(step);
}

// void btVehicle::debugDraw(btIDebugDraw* debugDrawer)
// {
	// for (int v = 0; v < this->getNumWheels(); v++)
	// {
	// 	btVector3 wheelColor(0, 1, 1);
	// 	if (getWheelInfo(v).m_raycastInfo.m_isInContact)
	// 	{
	// 		wheelColor.setValue(0, 0, 1);
	// 	}
	// 	else
	// 	{
	// 		wheelColor.setValue(1, 0, 1);
	// 	}

	// 	btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

	// 	btVector3 axle = btVector3(
	// 		getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
	// 		getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
	// 		getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

	// 	//debug wheels (cylinders)
	// 	debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
	// 	debugDrawer->drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_contactPointWS, wheelColor);
	// }
// }

btTransform& btVehicle::getChassisWorldTransform() 
{
	/*if (getRigidBody()->getMotionState())
	{
		btTransform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/

	// return getRigidBody()->getCenterOfMassTransform();

	return getChassisObject()->getWorldTransform();
}
btCollisionObject* btVehicle::getChassisObject() 
{
	return m_chassisObject;
}
btRigidBody* btVehicle::getRigidBody()
{
	return getChassisBody();
}
btRigidBody* btVehicle::getChassisBody()
{
	btRigidBody* body = btRigidBody::upcast(getChassisObject());
	if (body)
		return body;
	else
	{
		printf("No rigid body associated with chassis object.");
	}
	return 0;
}

btVector3 btVehicle::getAxisVector(int axis) 
{
	return getChassisWorldTransform().getBasis().getColumn(axis);
}
btVector3 btVehicle::getForwardVector() 
{
	return getAxisVector(getForwardAxis());
}

btVector3 btVehicle::getUpVector() 
{
	return getAxisVector(getUpAxis());
}

btVector3 btVehicle::getRightVector() 
{
	return getAxisVector(getRightAxis());
}

btScalar btVehicle::getCurrentSpeedMS() 
{
	return getRigidBody()->getLinearVelocity().length();
}

btScalar btVehicle::getCurrentSpeedKmHour() 
{
	return btScalar(3.6) * getCurrentSpeedMS();
}

void btVehicle::setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)
{
	m_indexRightAxis = rightIndex;
	m_indexUpAxis = upIndex;
	m_indexForwardAxis = forwardIndex;
}

///backwards compatibility
int btVehicle::getUserConstraintType() const
{
	return m_userConstraintType;
}

void btVehicle::setUserConstraintType(int userConstraintType)
{
	m_userConstraintType = userConstraintType;
};

void btVehicle::setUserConstraintId(int uid)
{
	m_userConstraintId = uid;
}

int btVehicle::getUserConstraintId() const
{
	return m_userConstraintId;
}
