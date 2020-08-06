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

// #include "LinearMath/btVector3.h"
#include "btHinge2Vehicle.h"

// #include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
// #include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
// #include "LinearMath/btQuaternion.h"
// #include "LinearMath/btMinMax.h"
// #include "LinearMath/btIDebugDraw.h"
// #include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

btHinge2Vehicle::btHinge2Vehicle()
	: btWheelVehicle()
{
}

btHinge2Vehicle::~btHinge2Vehicle()
{
	while (m_constraints.size()>0)
	{
		delete m_constraints[m_constraints.size()-1];
	}
}

void btHinge2Vehicle::updateAction(btCollisionWorld* collisionWorld, btScalar step)
{
	// (void)collisionWorld;
	// updateVehicle(step);
	updateConstraints();
}

void btHinge2Vehicle::updateVehicle(btScalar step)
{
	applyForces(step);
}

btHinge2Constraint* btHinge2Vehicle::addWheel2(btRigidBody* wheelBody, btScalar maxSteering, btScalar maxAngularVelocity, btScalar suspensionStiffness, btScalar suspensionDamping, btScalar maxTravel, btScalar friction, btScalar stallTorque)
{
	btHinge2Constraint* hinge = addWheel(wheelBody);
	btWheel* wheel = getWheel(getNumWheels()-1);

	if(maxSteering>=0) 
	{
		wheel->setSteeringAngle(0);
		wheel->setMaxSteeringAngle(maxSteering); 
		wheel->setMinSteeringAngle(-maxSteering); 
	}
	// else, use default in btWheel constructor

	if(maxAngularVelocity>=0) 
	{
		wheel->setAngularVelocity(0);
		wheel->setMaxAngularVelocity(maxAngularVelocity); 
		wheel->setMinAngularVelocity(-maxAngularVelocity);
	}
	// else, use default in btWheel constructor

	if(suspensionStiffness>=0)
	{  
		wheel->setStiffness(suspensionStiffness); 
	}
	// else, use default in btWheel constructor

	if(suspensionDamping>=0) 
	{  
		wheel->setDamping(suspensionDamping); 
	}
	// else, use default in btWheel constructor

	if(maxTravel>=0) 
		wheel->setMaxTravel(maxTravel); 
	// else, use default in btWheel constructor

	if(friction>=0) 
		wheel->setFriction(friction); 
	// else, use default in btWheel constructor

	if(stallTorque>=0) 
		wheel->setStallTorque(stallTorque); 
	// else, use default in btWheel constructor

	return hinge;
}

btHinge2Constraint* btHinge2Vehicle::addWheel(btRigidBody* wheelBody)
{
	const btVector3& wheelDirectionCS = -getUpVector();
	const btVector3& wheelAxleCS = getRightVector();
	const btVector3& chassisConnectionCS = wheelBody->getWorldTransform().getOrigin() - getChassisWorldTransform().getOrigin(); // assumes wheel body and chassis body are aligned

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
	// wheel->setMaxVelocity(4/wheel->getRadius());
	m_wheels.push_back(wheel);

	btJointFeedback* fb = new btJointFeedback();
	hinge->setJointFeedback(fb);
	
	// default params
	getChassisBody()->setFriction(1);

	wheelBody->setFriction(1);
	wheelBody->setRollingFriction(0.05);
	wheelBody->setContactStiffnessAndDamping(7000., 100);

	// updateConstraints();
	// return hinge;

	// Drive engine.
	hinge->enableMotor(HINGE2_DRIVE_IDX, wheel->getStallTorque()!=0.f);
	hinge->setMaxMotorForce(HINGE2_DRIVE_IDX, wheel->getStallTorque());
	hinge->setTargetVelocity(HINGE2_DRIVE_IDX, -wheel->getAngularVelocity());

	// Steering engine.
	hinge->enableMotor(HINGE2_STEERING_IDX, wheel->getMaxSteeringAngle()!=0.f);
	hinge->setServo(HINGE2_STEERING_IDX, wheel->getMaxSteeringAngle()!=0.f);
	hinge->setTargetVelocity(HINGE2_STEERING_IDX, 10.f);
	hinge->setMaxMotorForce(HINGE2_STEERING_IDX, 10.f);
	hinge->setServoTarget(HINGE2_STEERING_IDX, wheel->getSteeringAngle());

	// vertical suspension
	hinge->enableSpring(HINGE2_SUSPENSION_IDX, true);
	hinge->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_SUSPENSION_IDX );
	hinge->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_SUSPENSION_IDX );
	hinge->setDamping( HINGE2_SUSPENSION_IDX, wheel->getDamping(), false);
	hinge->setStiffness( HINGE2_SUSPENSION_IDX, wheel->getStiffness(), false);

	// horizontal
	hinge->enableSpring(HINGE2_LIN_X_IDX, false);
	// hinge->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_LIN_X_IDX );
	// hinge->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_LIN_X_IDX );
	// hinge->setDamping( HINGE2_LIN_X_IDX, wheel->getDamping(), false);
	// hinge->setStiffness( HINGE2_LIN_X_IDX, 1000.f, false);

	// limits
	btVector3 linLowerLimits;
	btVector3 linUpperLimits;
	linLowerLimits[HINGE2_LIN_X_IDX] = 0.f; // suspension horizontal travel limited
	linUpperLimits[HINGE2_LIN_X_IDX] = 0.f;
	linLowerLimits[HINGE2_LIN_Y_IDX] = -wheel->getMaxTravel(); // suspension vertical travel limited
	linUpperLimits[HINGE2_LIN_Y_IDX] = wheel->getMaxTravel();
	linLowerLimits[HINGE2_LIN_Z_IDX] = 0.f; // lock wheel forward/back
	linUpperLimits[HINGE2_LIN_Z_IDX] = 0.f;
	hinge->setLinearLowerLimit(linLowerLimits);
	hinge->setLinearUpperLimit(linUpperLimits);

	btVector3 angLowerLimits;
	btVector3 angUpperLimits;
	angLowerLimits[HINGE2_ANG_X_IDX-3] = 1.f; // free spin along axle
	angUpperLimits[HINGE2_ANG_X_IDX-3] = -1.f;
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

void btHinge2Vehicle::configureForDriveMode()
{
	btWheelVehicle::configureForDriveMode();
	updateConstraints();
}

void btHinge2Vehicle::updateConstraints()
{
	for (uint i=0; i<getNumWheels(); i++)
	{
		btWheel* wheel = getWheel(i);
		btHinge2Constraint* hinge = getConstraint(i);

		hinge->enableMotor(HINGE2_DRIVE_IDX, wheel->getStallTorque()!=0.f);
		hinge->setMaxMotorForce(HINGE2_DRIVE_IDX, wheel->getStallTorque());
		hinge->setTargetVelocity(HINGE2_DRIVE_IDX, -wheel->getAngularVelocity());

		// Steering engine.
		hinge->enableMotor(HINGE2_STEERING_IDX, wheel->getMaxSteeringAngle()!=0.f);
		hinge->setServo(HINGE2_STEERING_IDX, wheel->getMaxSteeringAngle()!=0.f);
		hinge->setTargetVelocity(HINGE2_STEERING_IDX, wheel->getMaxSteeringRate());
		hinge->setMaxMotorForce(HINGE2_STEERING_IDX, wheel->getStallTorque()/wheel->getRadius());
		hinge->setServoTarget(HINGE2_STEERING_IDX, -wheel->getSteeringAngle());

		// set wheel steering position instantaneously
		// btTransform wheelTranCS = btTransform(btQuaternion(0,0,0,1), wheel->getChassisConnection());
		// btMatrix3x3 yaw;
		// yaw.setEulerZYX(0, 0, -wheel->getSteeringAngle());
		// wheelTranCS.setBasis(yaw);
		// btTransform wheelTranWS = getChassisWorldTransform() * wheelTranCS;
		// wheel->getBody()->setCenterOfMassTransform(wheelTranWS);

		// vertical suspension
		hinge->enableSpring(HINGE2_SUSPENSION_IDX, true);
		hinge->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_SUSPENSION_IDX );
		hinge->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_SUSPENSION_IDX );
		hinge->setDamping( HINGE2_SUSPENSION_IDX, wheel->getDamping(), false);
		hinge->setStiffness( HINGE2_SUSPENSION_IDX, wheel->getStiffness(), false);

		// horizontal
		hinge->enableSpring(HINGE2_LIN_X_IDX, false);
		// hinge->setParam( BT_CONSTRAINT_CFM, 0.9, HINGE2_LIN_X_IDX );
		// hinge->setParam( BT_CONSTRAINT_ERP, 0.9, HINGE2_LIN_X_IDX );
		// hinge->setDamping( HINGE2_LIN_X_IDX, wheel->getDamping(), false);
		// hinge->setStiffness( HINGE2_LIN_X_IDX, wheel->getStiffness(), false);

		// limits
		btVector3 linLowerLimits;
		btVector3 linUpperLimits;
		linLowerLimits[HINGE2_LIN_X_IDX] = 0.f; // suspension horizontal travel limited
		linUpperLimits[HINGE2_LIN_X_IDX] = 0.f;
		linLowerLimits[HINGE2_LIN_Y_IDX] = -wheel->getMaxTravel(); // suspension vertical travel limited
		linUpperLimits[HINGE2_LIN_Y_IDX] = wheel->getMaxTravel();
		linLowerLimits[HINGE2_LIN_Z_IDX] = 0.f; // lock wheel forward/back
		linUpperLimits[HINGE2_LIN_Z_IDX] = 0.f;
		hinge->setLinearLowerLimit(linLowerLimits);
		hinge->setLinearUpperLimit(linUpperLimits);

		btVector3 angLowerLimits;
		btVector3 angUpperLimits;
		angLowerLimits[HINGE2_ANG_X_IDX-3] = 1.f; // free spin along axle
		angUpperLimits[HINGE2_ANG_X_IDX-3] = -1.f;
		angLowerLimits[HINGE2_ANG_Y_IDX-3] = wheel->getMinSteeringAngle(); // steering limited
		angUpperLimits[HINGE2_ANG_Y_IDX-3] = wheel->getMaxSteeringAngle();
		angLowerLimits[HINGE2_ANG_Z_IDX-3] = 0.f; // lock wheel roll
		angUpperLimits[HINGE2_ANG_Z_IDX-3] = 0.f;
		hinge->setAngularLowerLimit(angLowerLimits);
		hinge->setAngularUpperLimit(angUpperLimits);

		// hinge->setBreakingImpulseThreshold(INFINITY);

		// hinge->setDbgDrawSize(btScalar(5.f));

		// hinge->setEquilibriumPoint();
	}
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
	msg += line_prefix + 	"chassis force " +	btWheel::vector2str(con->getJointFeedback()->m_appliedForceBodyA)	+ line_suffix +"\n";
	msg += line_prefix + 	"chassis torque " +	btWheel::vector2str(con->getJointFeedback()->m_appliedTorqueBodyA)	+ line_suffix +"\n";
	msg += line_prefix + 	"wheel force " +	btWheel::vector2str(con->getJointFeedback()->m_appliedForceBodyB)	+ line_suffix +"\n";
	msg += line_prefix + 	"wheel torque " +	btWheel::vector2str(con->getJointFeedback()->m_appliedTorqueBodyB)	+ line_suffix +"\n";
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

	// printf("Applying ang vel %f steer %f\n", getWheel(i)->getAngularVelocity(), getWheel(i)->getSteeringAngle());

	// updateConstraints();

	getConstraint(i)->setServoTarget(HINGE2_STEERING_IDX, getWheel(i)->getSteeringAngle());

	// btVector3 torque = getWheel(i)->getWorldTransform().getBasis().getColumn(getRightAxis()) * getWheel(i)->getMotorTorque() * (1/getWheel(i)->getBody()->getInvMass()); 
	// getConstraint(i)->getRigidBodyA().applyTorqueImpulse( -torque*step ); // chassis body
	// getConstraint(i)->getRigidBodyB().applyTorqueImpulse(  torque*step ); // wheel body

	getConstraint(i)->setTargetVelocity(HINGE2_DRIVE_IDX, -getWheel(i)->getAngularVelocity());

	// updateConstraints();
}

btDefaultHinge2Vehicle::btDefaultHinge2Vehicle(
	btScalar _chassisHalfWidth, 
	btScalar _chassisHalfHeight, 
	btScalar _chassisHalfLength, 
	btScalar _chassisMass, 
	btScalar _wheelRadius, 
	btScalar _wheelHalfWidth, 
	btScalar _wheelMass, 
	btScalar _suspensionStiffness, 
	btScalar _suspensionDamping,
	btScalar _maxSteer,
	btScalar _maxAngVel,
	btScalar _maxTravel,
	btScalar _wheelFriction,
	btScalar _stallTorque,
	btScalar _maxSteeringRate
) : btHinge2Vehicle(),
	chassisHalfWidth(_chassisHalfWidth),
	chassisHalfHeight(_chassisHalfHeight),
	chassisHalfLength(_chassisHalfLength),
	chassisMass(_chassisMass),
	wheelRadius(_wheelRadius), 
	wheelHalfWidth(_wheelHalfWidth),
	wheelMass(_wheelMass),
	suspensionStiffness(_suspensionStiffness),
	suspensionDamping(_suspensionDamping),
	maxSteer(_maxSteer),
	maxAngVel(_maxAngVel),
	maxTravel(_maxTravel),
	wheelFriction(_wheelFriction),
	stallTorque(_stallTorque),
	maxSteeringRate(_maxSteeringRate)
{
}

void btDefaultHinge2Vehicle::spawn(
	btDiscreteDynamicsWorld* world,
	btTransform initialPose
)
{
	btCollisionShape* chassisShape = new btBoxShape(btVector3(chassisHalfLength, chassisHalfWidth, chassisHalfHeight));

	btRigidBody* chassisBody = createLocalRigidBody(chassisMass, initialPose, chassisShape);
	setChassisBody(chassisBody);
	world->addRigidBody(chassisBody);

	world->addAction(this);

	btCollisionShape* wheelShape = new btCylinderShape(btVector3(wheelRadius, wheelHalfWidth, wheelRadius));

	// front right
	btVector3 FRWheelPosCS = btVector3(	 btScalar(chassisHalfLength-wheelRadius), -btScalar(chassisHalfWidth + wheelHalfWidth),	btScalar(-chassisHalfHeight)	);//-cog;
	{
		btVector3 wheelPosCS = FRWheelPosCS;
		// btVector3 wheelPosCS = btVector3(	btScalar(chassisHalfLength-wheelRadius+.2), -btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	);
		btTransform wheelTranWS = getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = createLocalRigidBody(wheelMass, wheelTranWS, wheelShape);
		world->addRigidBody(wheelBody);
		btTypedConstraint* constraint = addWheel2(wheelBody, maxSteer, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, wheelFriction, stallTorque);
		world->addConstraint(constraint, true);
	}
	// front left
	{
		btVector3 wheelPosCS = FRWheelPosCS*btVector3(1,-1,1);
		// btVector3 wheelPosCS = btVector3(	 btScalar(chassisHalfLength-wheelRadius+.2),  btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	);
		btTransform wheelTranWS = getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = createLocalRigidBody(wheelMass, wheelTranWS, wheelShape);
		world->addRigidBody(wheelBody);
		btTypedConstraint* constraint = addWheel2(wheelBody, maxSteer, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, wheelFriction, stallTorque);
		world->addConstraint(constraint, true);
	}
	// back left
	{
		btVector3 wheelPosCS = FRWheelPosCS*btVector3(-1,-1,1);
		// btVector3 wheelPosCS = btVector3(	-btScalar(chassisHalfLength-wheelRadius+.2),  btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	);
		btTransform wheelTranWS = getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = createLocalRigidBody(wheelMass, wheelTranWS, wheelShape);
		world->addRigidBody(wheelBody);
		btTypedConstraint* constraint = addWheel2(wheelBody, 0.f, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, wheelFriction, stallTorque);
		world->addConstraint(constraint, true);
	}
	// back right
	{
		btVector3 wheelPosCS = FRWheelPosCS*btVector3(-1,1,1);
		// btVector3 wheelPosCS = btVector3(	-btScalar(chassisHalfLength-wheelRadius+.2), -btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	);
		btTransform wheelTranWS = getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = createLocalRigidBody(wheelMass, wheelTranWS, wheelShape);
		world->addRigidBody(wheelBody);
		btTypedConstraint* constraint = addWheel2(wheelBody, 0.f, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, wheelFriction, stallTorque);
		world->addConstraint(constraint, true);
	}

	updateConstraints();
}