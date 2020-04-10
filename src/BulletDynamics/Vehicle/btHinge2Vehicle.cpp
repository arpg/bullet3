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

btHinge2Vehicle::btHinge2Vehicle(btRigidBody* chassisBody)
	: btWheelVehicle(chassisBody)
{
}

btHinge2Vehicle::~btHinge2Vehicle()
{

}

void btHinge2Vehicle::updateAction(btCollisionWorld* collisionWorld, btScalar step)
{
	// (void)collisionWorld;
	updateVehicle(step);
}

void btHinge2Vehicle::updateVehicle(btScalar step)
{
	applyForces(step);
}

btHinge2Constraint* btHinge2Vehicle::addWheel(btRigidBody* wheelBody)
{
	const btVector3& wheelDirectionCS = -getUpVector();
	const btVector3& wheelAxleCS = getRightVector();
	const btVector3& chassisConnectionCS = btTransform(getChassisWorldTransform().inverse() * wheelBody->getWorldTransform()).getOrigin();

	// chassis connection (static parent)
	btRigidBody* bodyA = getChassisBody();
	bodyA->setActivationState(DISABLE_DEACTIVATION);
	btVector3 axisAWS = bodyA->getWorldTransform().getBasis() * -wheelDirectionCS;

	// wheel connection (dynamic child)	
	btRigidBody* bodyB = wheelBody;
	bodyB->setActivationState(DISABLE_DEACTIVATION);
	btVector3 axisBWS = bodyA->getWorldTransform().getBasis() * wheelAxleCS;

	// printf("axis a %f %f %f\n",axisAWS[0],axisAWS[1],axisAWS[2]);
	// printf("axis b %f %f %f\n",axisBWS[0],axisBWS[1],axisBWS[2]);
	// printf("dot %f\n",axisAWS.dot(axisBWS));
	// printf("eps %f\n",FLT_EPSILON);

	btAssert(axisAWS.dot(axisBWS) < FLT_EPSILON );

	btVector3 anchorWS = btTransform(bodyA->getWorldTransform() * btTransform(btQuaternion(0,0,0,1),chassisConnectionCS)).getOrigin();

	btHinge2Constraint* hinge = new btHinge2Constraint(*bodyA, *bodyB, anchorWS, axisAWS, axisBWS);
	m_constraints.push_back(hinge);

	btWheel* wheel = new btWheel(chassisConnectionCS, wheelDirectionCS, wheelAxleCS, bodyB);
	wheel->setMaxVelocity(4/wheel->getRadius());
	m_wheels.push_back(wheel);

	btJointFeedback* fb = new btJointFeedback();
	hinge->setJointFeedback(fb);
	
	// default params
	getChassisBody()->setFriction(1);

	wheelBody->setFriction(1);
	wheelBody->setRollingFriction(0.05);
	wheelBody->setContactStiffnessAndDamping(7500., 1000);

	// Drive engine.
	hinge->enableMotor(HINGE2_DRIVE_IDX, false);
	// hinge->setMaxMotorForce(HINGE2_DRIVE_IDX, maxMotorForce);
	// hinge->setTargetVelocity(HINGE2_DRIVE_IDX, 0);

	// Steering engine.
	hinge->enableMotor(HINGE2_STEERING_IDX, true);
	hinge->setServo(HINGE2_STEERING_IDX, true);
	hinge->setTargetVelocity(HINGE2_STEERING_IDX, 100);
	hinge->setMaxMotorForce(HINGE2_STEERING_IDX, 1000);
	hinge->setServoTarget(HINGE2_STEERING_IDX, 0);

	// vertical suspension
	hinge->enableSpring(HINGE2_SUSPENSION_IDX, true);
	hinge->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_SUSPENSION_IDX );
	hinge->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_SUSPENSION_IDX );
	hinge->setDamping( HINGE2_SUSPENSION_IDX, wheel->getDamping(), false);
	hinge->setStiffness( HINGE2_SUSPENSION_IDX, wheel->getStiffness(), false);

	// horizontal
	hinge->enableSpring(HINGE2_LIN_X_IDX, true);
	hinge->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_LIN_X_IDX );
	hinge->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_LIN_X_IDX );
	hinge->setDamping( HINGE2_LIN_X_IDX, wheel->getDamping(), false);
	hinge->setStiffness( HINGE2_LIN_X_IDX, 1000.f, false);

	// limits
	btVector3 linLowerLimits;
	btVector3 linUpperLimits;
	linLowerLimits[HINGE2_LIN_X_IDX] = -0.05; // suspension horizontal travel limited
	linUpperLimits[HINGE2_LIN_X_IDX] = 0.1;
	linLowerLimits[HINGE2_LIN_Y_IDX] = -wheel->getMaxTravel(); // suspension vertical travel limited
	linUpperLimits[HINGE2_LIN_Y_IDX] = wheel->getMaxTravel();
	linLowerLimits[HINGE2_LIN_Z_IDX] = 0.f; // lock wheel forward/back
	linUpperLimits[HINGE2_LIN_Z_IDX] = 0.f;
	hinge->setLinearLowerLimit(linLowerLimits);
	hinge->setLinearUpperLimit(linUpperLimits);

	btVector3 angLowerLimits;
	btVector3 angUpperLimits;
	angLowerLimits[HINGE2_ANG_X_IDX-3] = -wheel->getMaxVelocity(); // free spin along axle
	angUpperLimits[HINGE2_ANG_X_IDX-3] = wheel->getMaxVelocity();
	angLowerLimits[HINGE2_ANG_Y_IDX-3] = wheel->getMinSteeringAngle(); // steering limited
	angUpperLimits[HINGE2_ANG_Y_IDX-3] = wheel->getMaxSteeringAngle();
	angLowerLimits[HINGE2_ANG_Z_IDX-3] = 0.f; // lock wheel roll
	angUpperLimits[HINGE2_ANG_Z_IDX-3] = 0.f;
	hinge->setAngularLowerLimit(angLowerLimits);
	hinge->setAngularUpperLimit(angUpperLimits);

	hinge->setBreakingImpulseThreshold(INFINITY);

	hinge->setDbgDrawSize(btScalar(5.f));

	hinge->setEquilibriumPoint();

	return hinge;


	// btVector3 fric = static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->getRigidBodyB().getWorldTransform().getBasis().getColumn(2) * 1.0
	// 	+ static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->getRigidBodyB().getWorldTransform().getBasis().getColumn(0) * 1.0;
	// fric.normalize();
	// static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->getRigidBodyB().setAnisotropicFriction(fric);
	// static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->getRigidBodyB().setSpinningFriction(.02);
	// getWheel(i)->getBody()->setRollingFriction(0.05);
}

std::string btHinge2Vehicle::wheel2str(int i, std::string block_prefix="", std::string block_suffix="", std::string line_prefix="", std::string line_suffix="")
{
	return getWheel(i)->str(block_prefix, block_suffix, line_prefix, line_suffix)+constraint2str(i, block_prefix, block_suffix, line_prefix, line_suffix);
}
std::string btHinge2Vehicle::constraint2str(int i, std::string block_prefix="", std::string block_suffix="", std::string line_prefix="", std::string line_suffix="")
{
	btHinge2Constraint* con = getConstraint(i);
	btVector3 linupplim; con->getLinearUpperLimit(linupplim);
	btVector3 linlowlim; con->getLinearLowerLimit(linlowlim);
	btVector3 angupplim; con->getAngularUpperLimit(angupplim);
	btVector3 anglowlim; con->getAngularLowerLimit(anglowlim);
	std::string msg;
	msg += block_prefix;
	msg += line_prefix + 	"anchorA " + 		btWheel::vector2str(con->getAnchor()) 								+ line_suffix +"\n";
	msg += line_prefix + 	"axisA " + 			btWheel::vector2str(con->getAxis1())								+ line_suffix +"\n";
	msg += line_prefix + 	"axisB " + 			btWheel::vector2str(con->getAxis2())								+ line_suffix +"\n";
	msg += line_prefix + 	"linUppLim " + 		btWheel::vector2str(linupplim)										+ line_suffix +"\n";
	msg += line_prefix + 	"linLowLim " + 		btWheel::vector2str(linlowlim)										+ line_suffix +"\n";
	msg += line_prefix + 	"angUppLim " + 		btWheel::vector2str(angupplim)										+ line_suffix +"\n";
	msg += line_prefix + 	"angLowLim " + 		btWheel::vector2str(anglowlim)										+ line_suffix +"\n";
	msg += line_prefix + 	"appliedForceA " +	btWheel::vector2str(con->getJointFeedback()->m_appliedForceBodyA)	+ line_suffix +"\n";
	msg += line_prefix + 	"appliedTorqueA " +	btWheel::vector2str(con->getJointFeedback()->m_appliedForceBodyA)	+ line_suffix +"\n";
	msg += line_prefix + 	"appliedForceB " +	btWheel::vector2str(con->getJointFeedback()->m_appliedForceBodyA)	+ line_suffix +"\n";
	msg += line_prefix + 	"appliedTorqueB " +	btWheel::vector2str(con->getJointFeedback()->m_appliedForceBodyA)	+ line_suffix +"\n";
	msg += block_suffix;
	return msg;
}

btHinge2Constraint* btHinge2Vehicle::getConstraint(int constraint)
{
	btAssert(constraint < m_constraints.size());
	return m_constraints[constraint];
}

void btHinge2Vehicle::applyForces(btScalar step)
{
	for (uint i=0; i<getNumWheels(); i++)
	{
		applyForcesToWheel(i, step);
	}
}

void btHinge2Vehicle::applyForcesToWheel(int i, btScalar step)
{
	btAssert(i < getNumWheels() && i < m_constraints.size());

	// printf("Applying force %f steer %f\n", getWheel(i)->getTorqueForce(), getWheel(i)->getSteeringAngle());

	getConstraint(i)->setServoTarget(HINGE2_STEERING_IDX, getWheel(i)->getSteeringAngle());

	btVector3 torque = getWheel(i)->getWorldTransform().getBasis().getColumn(getWheel(i)->getWidth()) * getWheel(i)->getTorqueForce(); 
	getConstraint(i)->getRigidBodyA().applyTorqueImpulse( -torque );
	getConstraint(i)->getRigidBodyB().applyTorqueImpulse( torque );
}