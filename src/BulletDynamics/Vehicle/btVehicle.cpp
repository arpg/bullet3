#include "btVehicle.h"

//constructor to create a car from an existing rigidbody
btVehicle::btVehicle(btCollisionObject* chassisObject) : 
	m_chassisObject(chassisObject),
	m_indexRightAxis(0),
	m_indexUpAxis(1),
	m_indexForwardAxis(2) 
{	
}

btVehicle::~btVehicle()
{
	// delete m_chassisObject;
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
