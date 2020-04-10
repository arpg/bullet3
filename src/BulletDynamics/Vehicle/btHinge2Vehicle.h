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

#define HINGE2_LIN_X_IDX 0
#define HINGE2_LIN_Y_IDX 2
#define HINGE2_LIN_Z_IDX 1
#define HINGE2_ANG_X_IDX HINGE2_LIN_X_IDX+3
#define HINGE2_ANG_Y_IDX HINGE2_LIN_Y_IDX+3
#define HINGE2_ANG_Z_IDX HINGE2_LIN_Z_IDX+3

#define HINGE2_SUSPENSION_IDX HINGE2_LIN_Y_IDX
#define HINGE2_STEERING_IDX HINGE2_ANG_Y_IDX
#define HINGE2_DRIVE_IDX HINGE2_ANG_X_IDX

#include "BulletDynamics/Vehicle/btWheelVehicle.h"
#include "BulletDynamics/ConstraintSolver/btHinge2Constraint.h"

class btHinge2Vehicle : public btWheelVehicle
{
protected:
	btAlignedObjectArray<btHinge2Constraint*> m_constraints;

public:
	// struct btHinge2VehicleConfig{
	// 	btScalar cfm=0.9, 
	// 		erp=0.9;

	// } m_config;	

	//constructor to create a car from an existing rigidbody
	btHinge2Vehicle(btRigidBody* chassisBody);

	virtual ~btHinge2Vehicle();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
	virtual void updateVehicle(btScalar step);
	// virtual void debugDraw(btIDebugDraw* debugDrawer) = 0;

	virtual btHinge2Constraint* addWheel(btRigidBody* wheelBody);
	virtual std::string wheel2str(int i, std::string block_prefix, std::string block_suffix, std::string line_prefix, std::string line_suffix);
	virtual std::string constraint2str(int i, std::string block_prefix, std::string block_suffix, std::string line_prefix, std::string line_suffix);
	
	virtual btHinge2Constraint* getConstraint(int constraint);

	virtual void applyForces(btScalar);
	virtual void applyForcesToWheel(int wheel, btScalar);

};

#endif  //BT_HINGE2VEHICLE_H
