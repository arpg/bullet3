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

#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

btRaycastVehicle::btRaycastVehicle(btRigidBody* chassisBody, btVehicleRaycaster* raycaster)
	: btWheelVehicle(chassisBody)
 {
	m_vehicleRaycaster = raycaster;	 
 }

btRaycastVehicle::~btRaycastVehicle()
{
}

void btRaycastVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar step)
{
	updateVehicle(step);
}

void btRaycastVehicle::updateVehicle(btScalar step)
{
	updateAllWheelTransforms(false);
	btScalar curr_speed = getCurrentSpeedKmHour();
	updateSuspension(step);
	applyAllSuspensionForcesToChassis(step);
	// updateFriction(step);
	updateAllWheelRotationFromChassisInertia(step);
}

void btRaycastVehicle::addWheel(btRigidBody* wheelBody)
{
	const btVector3& wheelDirectionCS = -getUpVector();
	const btVector3& wheelAxleCS = getRightVector();
	const btVector3& chassisConnectionCS = wheelBody->getWorldTransform().getOrigin() - getChassisWorldTransform().getOrigin(); // assumes wheel and chassis body are aligned

	// chassis connection (static parent)
	btRigidBody* bodyA = getChassisBody();
	bodyA->setActivationState(DISABLE_DEACTIVATION);
	btVector3 axisAWS = bodyA->getWorldTransform().getBasis() * -wheelDirectionCS;

	// wheel connection (dynamic child)
	btRigidBody* bodyB = wheelBody;
	bodyB->setActivationState(DISABLE_DEACTIVATION);
	btVector3 axisBWS = bodyA->getWorldTransform().getBasis() * wheelAxleCS;

	btAssert(axisAWS.dot(axisAWS) < FLT_EPSILON);

	btWheel* wheel = new btWheel(chassisConnectionCS, wheelDirectionCS, wheelAxleCS, bodyB);
	wheel->setMaxVelocity(4/wheel->getRadius());
	m_wheels.push_back(wheel);

	updateWheelTransform(getNumWheels()-1, false);
}

void btRaycastVehicle::updateWheelTransformWS(btWheel* wheel, bool interpolatedTransform)
{
	btTransform chassisTrans = getChassisWorldTransform();
	if (interpolatedTransform && getRigidBody()->getMotionState())
		getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);

	wheel->m_raycastInfo.m_hardPointWS = chassisTrans(wheel->getChassisConnection());
	wheel->m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel->getChassisConnection();
	wheel->m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel->getAxleConnection();
}

void btRaycastVehicle::updateWheelTransform(btWheel* wheel, bool interpolatedTransform)
{
	updateWheelTransformWS(wheel, interpolatedTransform);
	btVector3 up = -wheel->m_raycastInfo.m_wheelDirectionWS;
	const btVector3& right = wheel->m_raycastInfo.m_wheelAxleWS;
	btVector3 fwd = up.cross(right);
	fwd = fwd.normalize();

	// rotate around steering over de wheelAxleWS
	btScalar steering = wheel->getSteeringAngle();

	btQuaternion steeringOrn(up, steering); 
	btMatrix3x3 steeringMat(steeringOrn);

	btQuaternion rotatingOrn(right, -wheel->m_raycastInfo.m_rotation);
	btMatrix3x3 rotatingMat(rotatingOrn);

	btMatrix3x3 basis2;
	basis2[0][m_indexRightAxis] = -right[0];
	basis2[1][m_indexRightAxis] = -right[1];
	basis2[2][m_indexRightAxis] = -right[2];

	basis2[0][m_indexUpAxis] = up[0];
	basis2[1][m_indexUpAxis] = up[1];
	basis2[2][m_indexUpAxis] = up[2];

	basis2[0][m_indexForwardAxis] = fwd[0];
	basis2[1][m_indexForwardAxis] = fwd[1];
	basis2[2][m_indexForwardAxis] = fwd[2];

	// NEED TO CHECK THIS
	wheel->m_raycastInfo.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	wheel->m_raycastInfo.m_worldTransform.setOrigin(wheel->m_raycastInfo.m_hardPointWS + wheel->m_raycastInfo.m_wheelDirectionWS * wheel->m_raycastInfo.m_suspensionLength);
}

void btRaycastVehicle::updateWheelTransform(int wheelIndex, bool interpolatedTransform)
{
	updateWheelTransform(m_wheels[wheelIndex], interpolatedTransform);
}

void btRaycastVehicle::updateAllWheelTransforms(bool interpolatedTransform)
{
	for (int i = 0; i < m_wheels.size(); i++)
	{
		updateWheelTransform(m_wheels[i], interpolatedTransform);
	}
}

btScalar btRaycastVehicle::rayCast(btWheel* wheel)
{
	updateWheelTransformWS(wheel, false);

	btScalar depth = -1;

	btScalar raylen = wheel->getSuspensionRestLength() + wheel->getRadius();

	btVector3 rayvector = wheel->getChassisConnection() * raylen;
	const btVector3& source = wheel->m_raycastInfo.m_hardPointWS;
	wheel->m_raycastInfo.m_contactPointWS = source + rayvector;
	const btVector3& target = wheel->m_raycastInfo.m_contactPointWS;

	btScalar param = btScalar(0.);

	btVehicleRaycaster::btVehicleRaycasterResult rayResults;

	btAssert(m_vehicleRaycaster);

	wheel->m_raycastInfo.m_isInContact = false;
	void* object = m_vehicleRaycaster->castRay(source, target, rayResults);

	wheel->m_raycastInfo.m_groundObject = 0;

	if (object)
	{
		param = rayResults.m_distFraction;
		depth = raylen * rayResults.m_distFraction;
		wheel->m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel->m_raycastInfo.m_isInContact = true;

		wheel->m_raycastInfo.m_groundObject = &getFixedBody();

		btScalar hitDistance = param * raylen;
		wheel->m_raycastInfo.m_suspensionLength = hitDistance - wheel->getRadius();
		// clamp on max suspension travel

		btScalar minSuspensionLength = wheel->getSuspensionRestLength() - wheel->getMaxTravel();
		btScalar maxSuspensionLength = wheel->getSuspensionRestLength() + wheel->getMaxTravel();
		if (wheel->m_raycastInfo.m_suspensionLength < minSuspensionLength)
		{
			wheel->m_raycastInfo.m_suspensionLength = minSuspensionLength;
		}
		if (wheel->m_raycastInfo.m_suspensionLength > maxSuspensionLength)
		{
			wheel->m_raycastInfo.m_suspensionLength = maxSuspensionLength;
		}

		wheel->m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;
		btScalar denominator = wheel->m_raycastInfo.m_contactNormalWS.dot(wheel->m_raycastInfo.m_wheelDirectionWS);

		btVector3 chassis_velocity_at_contactPoint;	
		btVector3 relpos = wheel->m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btScalar projVel = wheel->m_raycastInfo.m_contactPointWS.dot(chassis_velocity_at_contactPoint);

		if (denominator >= btScalar(-0.1))
		{
			wheel->m_raycastInfo.m_suspensionRelativeVelocity = btScalar(0.0);
			wheel->m_raycastInfo.m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
		}
		else
		{
			btScalar inv = btScalar(-1.) / denominator;
			wheel->m_raycastInfo.m_suspensionRelativeVelocity = projVel * inv;
			wheel->m_raycastInfo.m_clippedInvContactDotSuspension = inv;
		}
	}
	else
	{
		// put wheel info as in rest position
		wheel->m_raycastInfo.m_suspensionLength = wheel->getSuspensionRestLength();
		wheel->m_raycastInfo.m_suspensionRelativeVelocity = btScalar(0.0);
		wheel->m_raycastInfo.m_contactNormalWS = -wheel->m_raycastInfo.m_wheelDirectionWS;
		wheel->m_raycastInfo.m_clippedInvContactDotSuspension = btScalar(1.0);
	}
	
	return depth;
}
	
void btRaycastVehicle::castAllRays()
{
	for (int i = 0; i < getNumWheels(); i++)
	{
		rayCast(m_wheels[i]);
	}
}

void btRaycastVehicle::resetSuspension()
{
	for (int i = 0; i < m_wheels.size(); i++)
	{
		btWheel* wheel = m_wheels[i];
		wheel->m_raycastInfo.m_suspensionLength = wheel->getSuspensionRestLength();
		wheel->m_raycastInfo.m_suspensionRelativeVelocity = btScalar(1.0);

		wheel->m_raycastInfo.m_contactNormalWS = -wheel->m_raycastInfo.m_wheelDirectionWS;
		wheel->m_raycastInfo.m_clippedInvContactDotSuspension = btScalar(1.0);
	}
}

void btRaycastVehicle::applySuspensionForceToChassis(btScalar step, int wheelIndex)
{
	btAssert(iwheel < m_wheels.size());

	// apply suspension force
	btWheel* wheel = m_wheels[wheelIndex];

	btScalar suspensionForce = wheel->m_raycastInfo.m_wheelSuspensionForce;

	if (suspensionForce > wheel->getMaxSuspensionForce())
	{
		suspensionForce = wheel->getMaxSuspensionForce();
	}
	btVector3 impulse = wheel->m_raycastInfo.m_contactNormalWS * suspensionForce * step;
	btVector3 relpos = wheel->m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
	
	getRigidBody()->applyImpulse(impulse, relpos);
}

void btRaycastVehicle::applyAllSuspensionForcesToChassis(btScalar step)
{
	for (int i = 0; i < m_wheels.size(); i++)
	{
		applySuspensionForceToChassis(step, i);
	}
}

void btRaycastVehicle::updateWheelRotationFromChassisInertia(btScalar timeStep, int wheelIndex)
{
	btAssert(wheelIndex < m_wheels.size());
	btWheel* wheel = m_wheels[wheelIndex];
	btVector3 relpos = wheel->m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
	btVector3 vel = getRigidBody()->getVelocityInLocalPoint(relpos);

	if (wheel->m_raycastInfo.m_isInContact)
	{
		// calc forward unit vector of chassis (i.e. direction of travel, not nec. same as wheel heading)
		const btTransform& chassisWorldTransform = getChassisWorldTransform();

		btVector3 fwd(chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
					  chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
					  chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

		// calc forward vector projected to plane parallel to contact surface (e.g. ground)
		// this basically cancels the wheel's velocity in the direction of the contact surface due to assumed rigid collision
		btScalar proj = fwd.dot(wheel->m_raycastInfo.m_contactNormalWS);
		fwd -= wheel->m_raycastInfo.m_contactNormalWS * proj;

		// calc velocity of wheel (connection point) in direction of travel
		btScalar lin_vel = fwd.dot(vel);

		// convert projected linear velocity of wheel to angular velocity
		btScalar ang_vel = lin_vel / wheel->getRadius();

		// multiply timeStep to get change in rotation
		wheel->m_raycastInfo.m_deltaRotation = ang_vel * timeStep;
		wheel->m_raycastInfo.m_rotation += wheel->m_raycastInfo.m_deltaRotation;
	}
	else
	{
		wheel->m_raycastInfo.m_deltaRotation *= btScalar(0.99);
		wheel->m_raycastInfo.m_rotation += wheel->m_raycastInfo.m_deltaRotation;
	}	
}

void btRaycastVehicle::updateAllWheelRotationFromChassisInertia(btScalar timeStep)
{
	for (int i = 0; i < m_wheels.size(); i++)
	{
		updateWheelRotationFromChassisInertia(timeStep, i);
	}
}

void btRaycastVehicle::updateSuspension(btScalar deltaTime)
{
	(void) deltaTime;

	btScalar chassisMass = 1 / getChassisBody()->getInvMass();

	for (int i = 0; i < getNumWheels(); i++)
	{
		btWheel* wheel = m_wheels[i];

		if (wheel->m_raycastInfo.m_isInContact)
		{
			btScalar force;
			// Spring
			{
				btScalar susp_length = wheel->getSuspensionRestLength();
				btScalar current_length = wheel->m_raycastInfo.m_suspensionLength;
				btScalar length_diff = susp_length - current_length;
				force = wheel->getStiffness() * length_diff * wheel->m_raycastInfo.m_clippedInvContactDotSuspension;
			}
			// Damper
			{
				btScalar projected_rel_vel = wheel->m_raycastInfo.m_suspensionRelativeVelocity;
				{
					btScalar susp_damping;
					if (projected_rel_vel < btScalar(0.0))
					{
						susp_damping = wheel->getDamping();
					}
					else
					{
						susp_damping = wheel->getCompression();
					}
					force -= susp_damping * projected_rel_vel;
				}
			}
			// Result
			wheel->m_raycastInfo.m_wheelSuspensionForce = force * chassisMass;
			if (wheel->m_raycastInfo.m_wheelSuspensionForce < btScalar(0.))
			{
				wheel->m_raycastInfo.m_wheelSuspensionForce = btScalar(0.0);
			}
		}
		else
		{
			wheel->m_raycastInfo.m_wheelSuspensionForce = btScalar(0.);
		}	
	}
}

// void btRaycastVehicle::updateFriction(btScalar timeStep)
// {
// }

void btRaycastVehicle::applyForces(btScalar step)
{
	for (uint i = 0; i < getNumWheels(); i++)
	{
		applyForcesToWheel(i, step);
	}
}

void btRaycastVehicle::applyForcesToWheel(int i, btScalar step)
{
	btAssert(i < getNumWheels() && i < m_constraints.size());

	
}