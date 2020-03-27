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
#ifndef BT_HINGE2VEHICLE_H
#define BT_HINGE2VEHICLE_H

#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"

#define HINGE2_SUSPENSION_IDX 2
#define HINGE2_DRIVE_IDX 3
#define HINGE2_STEERING_IDX 5

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btHinge2Vehicle : public btRaycastVehicle
{
protected:
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btTypedConstraint* m_hinges[4];

	// bool m_useMCLPSolver = false;  //true;

	float m_CFM = 0.9;
	float m_ERP = 0.9;

	virtual inline 
	btHinge2Constraint* getHinge2(int i) { return static_cast<btHinge2Constraint*>(m_hinges[i]); }

public:
	//constructor to create a car from an existing rigidbody
	btHinge2Vehicle(btDiscreteDynamicsWorld* world, const btVehicleTuning& tuning, btRigidBody* chassis);

	virtual ~btHinge2Vehicle();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step)
	{
		// (void)collisionWorld;
		updateVehicle(step);
	}

	///btActionInterface interface
	virtual void debugDraw(btIDebugDraw* debugDrawer);

	virtual void applyForces(bool isFrontWheelDrive = true);

	virtual void updateVehicle(btScalar step);

	virtual btWheelInfo& addWheel(
		const btVector3& connectionPointCS0, 
		const btVector3& wheelDirectionCS0, 
		const btVector3& wheelAxleCS, 
		btScalar suspensionRestLength, 
		btScalar wheelRadius, 
		const btVehicleTuning& tuning, 
		bool isFrontWheel,
		btScalar wheelMass,
		btCollisionShape* wheelShape );

	virtual btRigidBody* createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);


	virtual inline
	void applyEngine(int wheel)
	{
		btHinge2Constraint* hinge = getHinge2(wheel);
		btRigidBody* bodyA = getChassisBody();
		btTransform trA = bodyA->getWorldTransform();
		btVector3 hingeAxisInWorld = trA.getBasis() * hinge->getFrameOffsetA().getBasis().getColumn(2);
		// btVector3 hingeAxisInWorld = hinge->getAxis2();
		// printf("axis %f %f %f\n",hingeAxisInWorld[0],hingeAxisInWorld[1],hingeAxisInWorld[2]);
		hinge->getRigidBodyA().applyTorque(-hingeAxisInWorld * getEngineForce(wheel));
		hinge->getRigidBodyB().applyTorque(hingeAxisInWorld * getEngineForce(wheel));
	}

	virtual inline
	void applySteering(int wheel)
	{
		getHinge2(wheel)->setServoTarget(HINGE2_STEERING_IDX, getSteeringValue(wheel));
	}

	virtual inline
	btJointFeedback* getJointFeedback(int wheel) { return getHinge2(wheel)->getJointFeedback(); }

	btVector3 getTorque(int wheel) { return  getJointFeedback(wheel)->m_appliedTorqueBodyB ; }
	btVector3 getForce(int wheel) { return getJointFeedback(wheel)->m_appliedForceBodyB ; }

	virtual inline 
	btRigidBody* getChassisBody() { return getRigidBody(); }
	virtual inline 
	const btRigidBody* getChassisBody() const { return getRigidBody(); }

	virtual inline 
	btRigidBody* getWheelBody(int wheel) { return &(m_hinges[wheel]->getRigidBodyB()); }
	virtual inline 
	const btRigidBody* getWheelBody(int wheel) const { return &(m_hinges[wheel]->getRigidBodyB()); }

	virtual inline 
	btCollisionShape* getChassisShape() { return getChassisBody()->getCollisionShape(); }
	virtual inline 
	const btCollisionShape* getChassisShape() const { return getChassisBody()->getCollisionShape(); }

	virtual inline 
	btCollisionShape* getWheelShape(int wheel) { return getWheelBody(wheel)->getCollisionShape(); }
	virtual inline 
	const btCollisionShape* getWheelShape(int wheel) const { return getWheelBody(wheel)->getCollisionShape(); }

	virtual inline 
	btScalar getWheelHalfWidth(int wheel) const 
	{ 
		auto shape = (btCylinderShape*)getWheelShape(wheel);
		int idxRadius, idxHeight;
		switch (shape->getUpAxis())  // get indices of radius and height of cylinder
		{
			case 0:  // cylinder is aligned along x
				idxRadius = 1;
				idxHeight = 0;
				break;
			case 2:  // cylinder is aligned along z
				idxRadius = 0;
				idxHeight = 2;
				break;
			default:  // cylinder is aligned along y
				idxRadius = 0;
				idxHeight = 1;
		}

		btAssert( getWheelInfo(wheel).m_wheelsWidth == 2*(shape->getHalfExtentsWithMargin()[idxHeight]) );

		return shape->getHalfExtentsWithMargin()[idxHeight]; 
	}
	virtual inline 
	btScalar getWheelRadius(int wheel) const 
	{ 
		auto shape = (btCylinderShape*)getWheelShape(wheel);
		int idxRadius, idxHeight;
		switch (shape->getUpAxis())  // get indices of radius and height of cylinder
		{
			case 0:  // cylinder is aligned along x
				idxRadius = 1;
				idxHeight = 0;
				break;
			case 2:  // cylinder is aligned along z
				idxRadius = 0;
				idxHeight = 2;
				break;
			default:  // cylinder is aligned along y
				idxRadius = 0;
				idxHeight = 1;
		}

		btAssert( getWheelInfo(wheel).m_wheelsRadius == shape->getHalfExtentsWithMargin()[idxRadius] );

		return shape->getHalfExtentsWithMargin()[idxRadius]; 
	}
};

#endif  //BT_HINGE2VEHICLE_H
