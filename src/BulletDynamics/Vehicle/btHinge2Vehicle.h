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
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"

class btSoftRigidDynamicsWorld;

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
	btHinge2Vehicle();

	virtual ~btHinge2Vehicle();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
	virtual void updateVehicle(btScalar step);
	// virtual void debugDraw(btIDebugDraw* debugDrawer) = 0;

	virtual btHinge2Constraint* addWheel(btRigidBody* wheelBody);
	btHinge2Constraint* addWheel2( btRigidBody* wheelBody, 
								  btScalar maxSteer = -1.f, 
								  btScalar maxAngularVelocity = -1.f, 
								  btScalar suspensionStiffness = -1.f, 
								  btScalar suspensionDamping = -1.f, 
								  btScalar maxTravel = -1.f, 
								  btScalar friction = -1.f, 
								  btScalar stallTorque = -1.f	);
	btSoftBody* addSoftWheel();
	virtual std::string wheel2str(int i, std::string block_prefix, std::string block_suffix, std::string line_prefix, std::string line_suffix);
	virtual std::string constraint2str(int i, std::string block_prefix, std::string block_suffix, std::string line_prefix, std::string line_suffix);
	
	void updateConstraints();	
	virtual btHinge2Constraint* getConstraint(int constraint);

	virtual void applyForces(btScalar);
	virtual void applyForcesToWheel(int wheel, btScalar);

};

class btDefaultHinge2Vehicle : public btHinge2Vehicle
{
public:

	btDefaultHinge2Vehicle(
		btScalar _chassisHalfWidth = 0.25f, 
		btScalar _chassisHalfHeight = 0.1f, 
		btScalar _chassisHalfLength = 0.5f, 
		btScalar _chassisMass = 10.f, 
		btScalar _wheelRadius = 0.2f, 
		btScalar _wheelHalfWidth = 0.1f, 
		btScalar _wheelMass = 1.f, 
		btScalar _suspensionStiffness = 200.f, 
		btScalar _suspensionDamping = 2.3f,
		btScalar _maxSteer = 30.f*M_PI/180.f,
		btScalar _maxAngVel = 5.f/0.2f,
		btScalar _maxTravel = 0.1f,
		btScalar _wheelFriction = 1.f,
		btScalar _stallTorque = 20.f,
		btScalar _maxSteeringRate = 20.f
	);
	virtual ~btDefaultHinge2Vehicle() {};
	void spawn(btDiscreteDynamicsWorld* world, btTransform initialPose = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));

	// virtual void updateVehicle(btScalar step) {};
	// virtual void debugDraw(btIDebugDraw* debugDrawer) {};

public:
	btScalar chassisHalfWidth; 
	btScalar chassisHalfHeight; 
	btScalar chassisHalfLength; 
	btScalar chassisMass; 
	btScalar wheelRadius; 
	btScalar wheelHalfWidth; 
	btScalar wheelMass; 
	btScalar suspensionStiffness; 
	btScalar suspensionDamping;
	btScalar maxSteer;
	btScalar maxAngVel;
	btScalar maxTravel;
	btScalar wheelFriction;
	btScalar stallTorque;
	btScalar maxSteeringRate;
};

class btDefaultHinge2VehicleSoft : public btDefaultHinge2Vehicle
{
public:

	btDefaultHinge2VehicleSoft(
		btSoftBodyWorldInfo m_softBodyWorldInfo,
		btScalar _chassisHalfWidth = 0.25f, 
		btScalar _chassisHalfHeight = 0.1f, 
		btScalar _chassisHalfLength = 0.5f, 
		btScalar _chassisMass = 10.f, 
		btScalar _wheelRadius = 0.2f, 
		btScalar _wheelHalfWidth = 0.1f, 
		btScalar _wheelMass = 1.f, 
		btScalar _suspensionStiffness = 200.f, 
		btScalar _suspensionDamping = 2.3f,
		btScalar _maxSteer = 30.f*M_PI/180.f,
		btScalar _maxAngVel = 5.f/0.2f,
		btScalar _maxTravel = 0.1f,
		btScalar _wheelFriction = 1.f,
		btScalar _stallTorque = 20.f,
		btScalar _maxSteeringRate = 20.f
	);
	virtual ~btDefaultHinge2VehicleSoft() {};
	void spawnTorus();
	void spawn(btSoftRigidDynamicsWorld* world, btTransform initialPose = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));

	// virtual void updateVehicle(btScalar step) {};
	// virtual void debugDraw(btIDebugDraw* debugDrawer) {};

public:
	btScalar chassisHalfWidth; 
	btScalar chassisHalfHeight; 
	btScalar chassisHalfLength; 
	btScalar chassisMass; 
	btScalar wheelRadius; 
	btScalar wheelHalfWidth; 
	btScalar wheelMass; 
	btScalar suspensionStiffness; 
	btScalar suspensionDamping;
	btScalar maxSteer;
	btScalar maxAngVel;
	btScalar maxTravel;
	btScalar wheelFriction;
	btScalar stallTorque;
	btScalar maxSteeringRate;
	
	btSoftBodyWorldInfo m_softBodyWorldInfo;
	std::vector<btSoftBody*> m_softWheels;
	std::vector<btTransform> m_wheelTrans;
	std::vector<btRigidBody*> m_rigidWheels;
};
#endif  //BT_HINGE2VEHICLE_H
