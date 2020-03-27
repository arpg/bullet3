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

#include "LinearMath/btVector3.h"
#include "btHinge2Vehicle.h"

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

btHinge2Vehicle::btHinge2Vehicle(btDiscreteDynamicsWorld* world, const btVehicleTuning& tuning, btRigidBody* chassis)
	: m_dynamicsWorld(world),
	  btRaycastVehicle(tuning, chassis, nullptr)
{
	// m_dynamicsWorld = world;
}

btHinge2Vehicle::~btHinge2Vehicle()
{

	// delete wheels and associated hinge constraints
	for (uint i=0; i<getNumWheels(); i++)
	{
		btTypedConstraint* temp = getHinge2(i);
		m_dynamicsWorld->removeConstraint(temp);
		// m_dynamicsWorld->removeRigidBody(&(temp->getRigidBodyB()));
	}

	// delete dynamics world
	// if (m_dynamicsWorld)
	// {
	// 	delete m_dynamicsWorld;
		m_dynamicsWorld = 0;
	// }

}

//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btWheelInfo& btHinge2Vehicle::addWheel(
	const btVector3& connectionPointCS, 
	const btVector3& wheelDirectionCS0, 
	const btVector3& wheelAxleCS, 
	btScalar suspensionRestLength, 
	btScalar wheelRadius, 
	const btVehicleTuning& tuning, 
	bool isFrontWheel,
	btScalar wheelMass,
	btCollisionShape* wheelShape)
{
	btRaycastVehicle::addWheel(
			connectionPointCS, 
			wheelDirectionCS0, 
			wheelAxleCS, 
			suspensionRestLength, 
			wheelRadius, 
			tuning, 
			isFrontWheel );

	btWheelInfo wheel = getWheelInfo(getNumWheels()-1);
	
	wheel.m_wheelsMass = wheelMass;
	wheel.m_engineForce = btScalar(0.);
	wheel.m_maxEngineForce = btScalar(500.);
	wheel.m_minEngineForce = btScalar(-100.);

	// printf("rad %f\n",wheel.m_wheelsRadius);
	// wheel.m_wheelsRadius = 0.2;
	// printf("rad %f\n",wheel.m_wheelsRadius);

	// add hinge constraint
	{
		// chassis
		btRigidBody* pBodyA = getChassisBody();
		pBodyA->setFriction(1.);
		pBodyA->setActivationState(DISABLE_DEACTIVATION);

		// wheel
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(getChassisWorldTransform().getBasis() * connectionPointCS);
		btVector3 anchor = tr.getOrigin();
		btRigidBody* pBodyB = createRigidBody(wheelMass, tr, wheelShape);
		pBodyB->setFriction(wheel.m_frictionSlip);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);

		// add some data to build constraint frames
		// btVector3 parentAxis(0.f, 1.f, 0.f);
		// btVector3 childAxis(1.f, 0.f, 0.f);
		btVector3 parentAxis = getAxisVector(getUpAxis());
		btVector3 childAxis = getAxisVector(getRightAxis());
		btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
		m_hinges[getNumWheels()-1] = pHinge2;
		pHinge2->setJointFeedback(new btJointFeedback());

		//m_guiHelper->get2dCanvasInterface();
		
		// add constraint to world
		m_dynamicsWorld->addConstraint(pHinge2, true);

		// Drive engine.
		pHinge2->enableMotor(HINGE2_DRIVE_IDX, true);
		pHinge2->setMaxMotorForce(HINGE2_DRIVE_IDX, wheel.m_maxEngineForce);
		// pHinge2->setTargetVelocity(HINGE2_DRIVE_IDX, 0);

		// Steering engine.
		pHinge2->enableMotor(HINGE2_STEERING_IDX, true);
		// pHinge2->setMaxMotorForce(5, 1000);
		// pHinge2->setTargetVelocity(5, 0);
		pHinge2->setServo(HINGE2_STEERING_IDX, true);
		pHinge2->setTargetVelocity(HINGE2_STEERING_IDX, 1000);
		pHinge2->setMaxMotorForce(HINGE2_STEERING_IDX, 1000);
		pHinge2->setServoTarget(HINGE2_STEERING_IDX, 0);

		// suspension
		pHinge2->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_SUSPENSION_IDX );
		pHinge2->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_SUSPENSION_IDX );
		pHinge2->setDamping( HINGE2_SUSPENSION_IDX, wheel.m_wheelsDampingCompression, false);
		pHinge2->setStiffness( HINGE2_SUSPENSION_IDX, wheel.m_suspensionStiffness, false);

		// btVector3 linLowerLimits(FLT_MAX,FLT_MAX,FLT_MAX);
		// btVector3 linUpperLimits(FLT_MIN,FLT_MIN,FLT_MIN);
		// linLowerLimits[HINGE2_SUSPENSION_IDX] = -2*2*wheelRadius;
		// linUpperLimits[HINGE2_SUSPENSION_IDX] = 0.9*wheelRadius;
		// pHinge2->setLinearLowerLimit(linLowerLimits);
		// pHinge2->setLinearUpperLimit(linUpperLimits);
		
		// btVector3 angLowerLimits(FLT_MAX,FLT_MAX,FLT_MAX);
		// btVector3 angUpperLimits(FLT_MIN,FLT_MIN,FLT_MIN);
		// // if (i==frontRightWheelIndex || i==frontLeftWheelIndex)
		// // {
		// 	angLowerLimits[HINGE2_STEERING_IDX-3] = -1.f;
		// 	angUpperLimits[HINGE2_STEERING_IDX-3] = 1.f;
		// // }
		// // else
		// // {
		// // 	angLowerLimits[HINGE2_STEERING_IDX-3] = 0.f;
		// // 	angUpperLimits[HINGE2_STEERING_IDX-3] = 0.f;
		// // }
		// pHinge2->setAngularLowerLimit(angLowerLimits);
		// pHinge2->setAngularUpperLimit(angUpperLimits);

		// pHinge2->setDbgDrawSize(btScalar(5.f));

		// pHinge2->setEquilibriumPoint();
	}
}

btRigidBody* btHinge2Vehicle::createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
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

	m_dynamicsWorld->addRigidBody(body);
	return body;
}

void btHinge2Vehicle::applyForces(bool isFrontWheelDrive)
{
	for (uint i=0; i<m_wheelInfo.size(); i++)
	{
		bool isFrontWheel = getWheelInfo(i).m_bIsFrontWheel;
		if (isFrontWheel)
			applySteering(i);
		if (isFrontWheel == isFrontWheelDrive)
		{
			applyEngine(i);
		}
	}
}

void btHinge2Vehicle::updateVehicle(btScalar step)
{
	calcCurrentSpeed();
	
	// applyForces(true);
}

void btHinge2Vehicle::debugDraw(btIDebugDraw* debugDrawer)
{
	for (int v = 0; v < this->getNumWheels(); v++)
	{
		btVector3 wheelColor(0, 1, 1);
		if (getWheelInfo(v).m_raycastInfo.m_isInContact)
		{
			wheelColor.setValue(0, 0, 1);
		}
		else
		{
			wheelColor.setValue(1, 0, 1);
		}

		btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

		btVector3 axle = btVector3(
			getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

		//debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
		debugDrawer->drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_contactPointWS, wheelColor);
	}
}
