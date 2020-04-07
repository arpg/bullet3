#ifndef BT_VEHICLE_H
#define BT_VEHICLE_H

#include <algorithm>
#include <vector>
#include <memory>
#include <iostream> // cout
#include <stdio.h>  //printf debugging

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "btWheel.h"

class btVehicle : public btActionInterface
{
protected:
	int m_indexRightAxis;
	int m_indexUpAxis;
	int m_indexForwardAxis;

	btCollisionObject* m_chassisObject;

	///backwards compatibility
	int m_userConstraintType;
	int m_userConstraintId;

public:
	btVehicle(btCollisionObject* chassisObject = new btCollisionObject());
	virtual ~btVehicle();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
	virtual void updateVehicle(btScalar step) = 0;
	virtual void debugDraw(btIDebugDraw* debugDrawer) = 0;

	virtual inline btVector3 getChassisVelocity() { return btTransform(btTransform(btQuaternion(0,0,0,1),getChassisBody()->getLinearVelocity())*getChassisWorldTransform()).getOrigin(); }
	virtual inline btScalar getChassisForwardVelocity() { return getChassisVelocity()[getForwardAxis()]; }  

	virtual btTransform& getChassisWorldTransform();

	virtual btCollisionObject* getChassisObject() ;
	virtual btRigidBody* getRigidBody() ;
	virtual btRigidBody* getChassisBody() ;

	inline int getRightAxis()  { return m_indexRightAxis; }
	inline int getUpAxis()  { return m_indexUpAxis; }
	inline int getForwardAxis()  {	return m_indexForwardAxis; }

	virtual btVector3 getAxisVector(int axis) ;
	virtual btVector3 getForwardVector() ;
	virtual btVector3 getUpVector() ;
	virtual btVector3 getRightVector() ;

	virtual btScalar getCurrentSpeedMS() ;
	virtual btScalar getCurrentSpeedKmHour() ;

	virtual void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex);

	///backwards compatibility
	int getUserConstraintType() const;
	void setUserConstraintType(int );
	void setUserConstraintId(int );
	int getUserConstraintId() const;
};

#endif  //BT_VEHICLE_H
