/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_RAYCASTVEHICLE_OLD_H
#define BT_RAYCASTVEHICLE_OLD_H

#include <algorithm>

#include <vector>
#include <memory>
#include <iostream> // cout
#include <stdio.h>  //printf debugging

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class btVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : public btActionInterface
{
public:
	class btVehicleTuning
	{
	public:
		btVehicleTuning()
			: m_suspensionStiffness(btScalar(5.88)),
			  m_suspensionCompression(btScalar(0.83)),
			  m_suspensionDamping(btScalar(0.88)),
			  m_maxSuspensionTravelCm(btScalar(500.)),
			  m_frictionSlip(btScalar(10.5)),
			  m_maxSuspensionForce(btScalar(6000.))
		{
		}
		btScalar m_suspensionStiffness;
		btScalar m_suspensionCompression;
		btScalar m_suspensionDamping;
		btScalar m_maxSuspensionTravelCm;
		btScalar m_frictionSlip;
		btScalar m_maxSuspensionForce;
	};

	void defaultInit(const btVehicleTuning& tuning);
	
////// wheel vehicles //////
protected:
	int m_indexRightAxis;
	int m_indexUpAxis;
	int m_indexForwardAxis;

	btScalar m_pitchControl;
	btScalar m_steeringValue;
	btScalar m_currentVehicleSpeedKmHour;

	btRigidBody* m_chassisBody;

	///backwards compatibility
	int m_userConstraintType;
	int m_userConstraintId;

public:
	btAlignedObjectArray<btWheelInfo> m_wheelInfo;

	//constructor to create a car from an existing rigidbody
	btRaycastVehicle(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster);

	virtual ~btRaycastVehicle();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step)
	{
		(void)collisionWorld;
		updateVehicle(step);
	}

	virtual void debugDraw(btIDebugDraw* debugDrawer);

	virtual const btTransform& getChassisWorldTransform() const;

	virtual void updateVehicle(btScalar step);

	virtual btScalar getSteeringValue(int wheel) const;
	virtual btScalar getSteeringClampValue(int wheel) const;
	virtual void setSteeringValue(btScalar steering, int wheel);
	virtual void setSteeringValue(btScalar steering, bool isFrontWheelSteer = true);
	virtual void setSteeringClampValue(btScalar steering, int wheel);
	virtual void setSteeringClampValue(btScalar steering, bool isFrontWheelSteer = true);

	virtual btScalar getEngineForce(int wheel) const;
	virtual btScalar getMaxEngineForce(int wheel) const;
	virtual btScalar getMinEngineForce(int wheel) const;
	virtual void setEngineForce(btScalar force, int wheel);
	virtual void setEngineForce(btScalar force, bool isFrontWheelDrive = true);

	virtual void setBrake(btScalar brake, int wheelIndex);
	virtual void setBrake(btScalar brake, bool isFrontWheelBrake = false);

	virtual void setPitchControl(btScalar pitch){ m_pitchControl = pitch; }

	virtual btWheelInfo& addWheel(const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0, const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel);

	inline int getNumWheels() const { return int(m_wheelInfo.size()); }

	const btWheelInfo& getWheelInfo(int index) const;
	btWheelInfo& getWheelInfo(int index);

	int getFrontLeftWheel();
	int getFrontRightWheel();
	int getBackLeftWheel();
	int getBackRightWheel();

	inline btRigidBody* getRigidBody() { return m_chassisBody; }
	const btRigidBody* getRigidBody() const { return m_chassisBody; }

	inline int getRightAxis() const { return m_indexRightAxis; }
	inline int getUpAxis() const { return m_indexUpAxis; }
	inline int getForwardAxis() const { return m_indexForwardAxis; }

	///Worldspace forward vector
	btVector3 getForwardVector() const
	{
		const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 forwardW(
			chassisTrans.getBasis()[0][m_indexForwardAxis],
			chassisTrans.getBasis()[1][m_indexForwardAxis],
			chassisTrans.getBasis()[2][m_indexForwardAxis]);

		return forwardW;
	}

	virtual inline btVector3 getAxisVector(int axis)
	{
		const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 vec(
			chassisTrans.getBasis()[0][axis],
			chassisTrans.getBasis()[1][axis],
			chassisTrans.getBasis()[2][axis]);

		return vec;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	virtual void calcCurrentSpeed();
	btScalar getCurrentSpeedKmHour() const { return m_currentVehicleSpeedKmHour; }

	void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)
	{
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}

////// raycast vehicles //////
private:
	btVehicleRaycaster* m_vehicleRaycaster;

	btAlignedObjectArray<btVector3> m_forwardWS;
	btAlignedObjectArray<btVector3> m_axle;
	btAlignedObjectArray<btScalar> m_forwardImpulse;
	btAlignedObjectArray<btScalar> m_sideImpulse;

public:
	virtual void updateWheelTransformWS(btWheelInfo& wheel, bool interpolatedTransform = true);
	virtual const btTransform& getWheelTransformWS(int wheelIndex) const;
	virtual void updateWheelTransform(btWheelInfo& wheel, bool interpolatedTransform=true);
	virtual void updateWheelTransform(int wheelIndex, bool interpolatedTransform=true);
	virtual void updateAllWheelTransforms(bool interpolatedTransform);

	virtual btScalar rayCast(btWheelInfo& wheel);
	virtual void castAllRays();

	virtual void resetSuspension();

	virtual void applySuspensionForceToChassis(btScalar step, int wheel);
	virtual void applyAllSuspensionForcesToChassis(btScalar step);

	virtual void updateWheelRotationFromChassisInertia(btScalar step, int wheel);
	virtual void updateAllWheelRotationsFromChassisInertia(btScalar step);

	//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	void updateSuspension(btScalar deltaTime);

	void updateFriction(btScalar timeStep);

////// backwards compatibility
	int getUserConstraintType() const { return m_userConstraintType; }
	void setUserConstraintType(int userConstraintType) { m_userConstraintType = userConstraintType; };

	void setUserConstraintId(int uid) { m_userConstraintId = uid; }
	int getUserConstraintId() const { return m_userConstraintId; }
};

class btDefaultVehicleRaycaster : public btVehicleRaycaster
{
	btDynamicsWorld* m_dynamicsWorld;

public:
	btDefaultVehicleRaycaster(btDynamicsWorld* world)
		: m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result);
};

#endif  //BT_RAYCASTVEHICLE_OLD_H
