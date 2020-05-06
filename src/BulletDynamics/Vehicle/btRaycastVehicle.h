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
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Vehicle/btWheelVehicle.h"
#include "btVehicleRaycaster.h"

class btRaycastVehicle : public btWheelVehicle
{
private:
	btVehicleRaycaster* m_vehicleRaycaster;

public:
	// constructor to create a car from an existing rigid body
	btRaycastVehicle(btRigidBody* chassisBody, btVehicleRaycaster* raycaster);

	virtual ~btRaycastVehicle();

	// btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
	virtual void updateVehicle(btScalar step);

	virtual void addWheel(btRigidBody* wheelBody);
	virtual std::string wheel2str(int i, std::string block_prefix, std::string block_suffix, std::string line_prefix, std::string line_suffix);
	virtual std::string constraint2str(int i, std::string block_prefix, std::string block_suffix, std::string line_prefix, std::string line_suffix);
	
	virtual btHinge2Constraint* getConstraint(int constraint);

	virtual void updateWheelTransformWS(btWheel* wheel, bool interpolatedTransform);
	virtual void updateWheelTransform(btWheel* wheel, bool interpolatedTransform);
	virtual void updateWheelTransform(int wheelIndex, bool interpolatedTransform);
	virtual void updateAllWheelTransforms(bool interpolatedTransform);
	virtual btScalar rayCast(btWheel* wheel);
	virtual void castAllRays();

	virtual void resetSuspension();

	virtual void applySuspensionForceToChassis(btScalar step, int wheel);
	virtual void applyAllSuspensionForcesToChassis(btScalar step);

	virtual void updateWheelRotationFromChassisInertia(btScalar timeStep, int wheelIndex);
	virtual void updateAllWheelRotationFromChassisInertia(btScalar timeStep);

	void updateSuspension(btScalar deltaTime);

	void updateFriction(btScalar timeStep);

	virtual void applyForces(btScalar step);
	virtual void applyForcesToWheel(int wheel, btScalar);

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

#endif  //BT_RAYCASTVEHICLE_H
