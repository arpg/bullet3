#include "BulletDynamics/Vehicle/btWheelVehicle.h"

// void* btDefaultVehicleRaycaster::castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result)
// {
// 	//	RayResultCallback& resultCallback;

// 	btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);

// 	m_dynamicsWorld->rayTest(from, to, rayCallback);

// 	if (rayCallback.hasHit())
// 	{
// 		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
// 		if (body && body->hasContactResponse())
// 		{
// 			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
// 			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
// 			result.m_hitNormalInWorld.normalize();
// 			result.m_distFraction = rayCallback.m_closestHitFraction;
// 			return (void*)body;
// 		}
// 	}
// 	return 0;
// }

/////////////////////////////////////////////////////////////////

btWheelVehicle::btWheelVehicle() 
	: btVehicle(),
	  m_driveMode(DIFF)
{	
	configureForDriveMode();
}


btWheelVehicle::~btWheelVehicle()
{
	while (getNumWheels())
	{
		delete m_wheels[getNumWheels()-1];
	}
}

// void btWheelVehicle::updateVehicle(btScalar step)
// {
	
// }

void btWheelVehicle::debugDraw(btIDebugDraw* debugDrawer)
{
	for (int v = 0; v < this->getNumWheels(); v++)
	{
		btVector3 wheelColor(0, 1, 1);
		// if (getWheel(v)->isInContact())
		// {
			// wheelColor.setValue(0, 0, 1);
		// }
		// else
		// {
		// 	wheelColor.setValue(1, 0, 1);
		// }

		btVector3 wheelPosWS = getWheel(v)->getWorldTransform().getOrigin();

		btVector3 axle = btVector3(
			getWheel(v)->getWorldTransform().getBasis()[0][getRightAxis()],
			getWheel(v)->getWorldTransform().getBasis()[1][getRightAxis()],
			getWheel(v)->getWorldTransform().getBasis()[2][getRightAxis()]);

		//debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
		// debugDrawer->drawLine(wheelPosWS, getWheel(v)->m_raycastInfo.m_contactPointWS, wheelColor);
	}
}

// void btWheelVehicle::spawn(btDynamicsWorld* world, btTransform initialPose)
// {
// 	// m_raycaster = new btDefaultVehicleRaycaster(world);

// 	btVehicle::spawn(world, initialPose);
// 	for(uint i=0; i<getNumWheels(); i++)
// 	{
// 		getWheel(i)->spawn(world);
// 	}
// }

void btWheelVehicle::setDriveMode(DriveMode mode)
{
	if (mode == m_driveMode) return;
	m_driveMode = mode;
	configureForDriveMode();
}

void btWheelVehicle::resetSuspension()
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		btTransform wheelTranCS = btTransform(btQuaternion(0,0,0,1), getWheel(i)->getChassisConnection());
		btTransform wheelTranWS = getChassisWorldTransform() * wheelTranCS;
		getWheel(i)->getRigidBody()->setCenterOfMassTransform(wheelTranWS);
	}
}

void btWheelVehicle::addWheel(const btVector3& chassisConnectionCS, btScalar width, btScalar radius)
{
	const btVector3 wheelDirectionCS = -getUpVector();
	const btVector3 wheelAxleCS = getRightVector();
	btWheel wheel(chassisConnectionCS, wheelDirectionCS, wheelAxleCS, width, radius);
	m_wheels.push_back(&wheel);
	// return wheel;
}
btWheel* btWheelVehicle::getWheel(int wheel)
{
	btAssert(wheel < getNumWheels());
	return m_wheels[wheel];
}

void btWheelVehicle::setEnabledAngularVelocity(btScalar val)
{
	// printf("Setting ang vel %f rad/s\n", val);
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isMotorEnabled()) // not necessary but good habit
		{
			getWheel(i)->setAngularVelocity(val);
		}
	}
}
void btWheelVehicle::setEnabledLinearVelocity(btScalar val)
{
	// printf("Setting lin vel %f m/s\n", val);
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isMotorEnabled()) // not necessary but good habit
		{
			getWheel(i)->setAngularVelocity(val / getWheel(i)->getRadius());
		}
	}
}
void btWheelVehicle::setEnabledAngularAcceleration(btScalar accel, btScalar dt)
{
	// printf("Setting ang accel %f over %f sec\n", accel, dt);
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isMotorEnabled()) // not necessary but good habit
		{
			getWheel(i)->setAngularVelocity( accel * dt + getWheel(i)->getAngularVelocity() );
		}
	}
}
void btWheelVehicle::setEnabledSteeringAngle(btScalar val)
{
	// printf("Setting steering %f rad\n", val);

	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isSteeringEnabled())
		{
			getWheel(i)->setSteeringAngle(val);
		}
	}
}

void btWheelVehicle::setEnabledVelocity(btScalar yaw_vel, btScalar forw_vel)
{
	// printf("Setting yaw vel %f rad/s", val);

	switch (m_driveMode)
	{
		case DriveMode::DIFF :
		{
			// printf(" in diff mode\n");
			for(uint i=0; i<getNumWheels(); i++)
			{
				btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(i)->getWorldTransform();
      			btScalar wheel_base = 2*fabs(wheelTranCS.getOrigin()[getForwardAxis()]);
				btScalar ang_vel_sign = -1*wheelTranCS.getOrigin()[getRightAxis()];
				ang_vel_sign /= fabs(ang_vel_sign);
      			btScalar ang_vel = (forw_vel + ang_vel_sign*yaw_vel*wheel_base/2.0); 
				getWheel(i)->setAngularVelocity(ang_vel/getWheel(i)->getRadius());
			}
			break;
		}
		case DriveMode::ACKERMANN :
		{
			// printf(" in ackermann mode\n");
			btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(0)->getWorldTransform();
			btScalar wheel_base = 2*fabs(wheelTranCS.getOrigin()[getForwardAxis()]);
			btScalar steering_angle = atan2(yaw_vel * wheel_base, getCurrentSpeedMS());
			// printf("\tenabled steering %f rad\n", steering_angle);
			setEnabledSteeringAngle(steering_angle);
			setEnabledLinearVelocity(forw_vel);
			break;
		}
		case DriveMode::DUAL_ACKERMANN :
		{
			// printf(" in dual ackermann mode\n");
			for(uint i=0; i<getNumWheels(); i++)
			{
				btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(i)->getWorldTransform();
				btScalar wheel_base = wheelTranCS.getOrigin()[getForwardAxis()];
				btScalar steering_angle = atan2(yaw_vel * wheel_base, getCurrentSpeedMS());
				// setEnabledSteeringAngle(atan2(val * wheel_base, getCurrentSpeedMS()));
				// printf("\twheel %d steering %f rad\n", i, steering_angle);
				getWheel(i)->setSteeringAngle(steering_angle);
			}
			setEnabledLinearVelocity(forw_vel);
			break;
		}
		default:
		{
			break;
		}
	}
}

void btWheelVehicle::setAllFriction(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setFriction(val);
	}
}
void btWheelVehicle::setAllStiffness(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setStiffness(val);
	}
}
void btWheelVehicle::setAllDamping(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setDamping(val);
	}
}
void btWheelVehicle::setAllMaxTravel(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMaxTravel(val);
	}
}

void btWheelVehicle::setAllMinAngularVelocity(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMinAngularVelocity(val);
	}
}
void btWheelVehicle::setAllMaxAngularVelocity(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMaxAngularVelocity(val);
	}
}

void btWheelVehicle::setAllMinSteeringAngle(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMinSteeringAngle(val);
	}
}
void btWheelVehicle::setAllMaxSteeringAngle(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMaxSteeringAngle(val);
	}
}

std::vector<btWheel*>& btWheelVehicle::getEnabledMotorWheels()
{
	static std::vector<btWheel*> wheels;
	wheels.clear();
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isMotorEnabled())
		{
			wheels.push_back(getWheel(i));
		}
	}
	return wheels;
}

std::vector<btWheel*>& btWheelVehicle::getEnabledSteeringWheels()
{
	static std::vector<btWheel*> wheels;
	wheels.clear();
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isSteeringEnabled())
		{
			wheels.push_back(getWheel(i));
		}
	}
	return wheels;
}

btScalar btWheelVehicle::getEnabledSteeringAngle()
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isSteeringEnabled())
		{
			return getWheel(i)->getSteeringAngle();
		}
	}
	return 0.f;
}

void btWheelVehicle::configureForDriveMode()
{
	switch (m_driveMode)
	{
		case DriveMode::DIFF:
			for(uint i=0; i<getNumWheels(); i++)
			{
				// getWheel(i)->setSteeringAngle(0);
				getWheel(i)->setMinSteeringAngle(0);
				getWheel(i)->setMaxSteeringAngle(0);
			}
			break;

		case DriveMode::ACKERMANN:
			for(uint i=0; i<getNumWheels(); i++)
			{
				// getWheel(i)->setSteeringAngle(0);
				btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(i)->getWorldTransform();
				if (wheelTranCS.getOrigin()[getForwardAxis()] > 0) // front wheel
				{
					getWheel(i)->setMinSteeringAngle(-0.4);
					getWheel(i)->setMaxSteeringAngle(0.4);
				}
				else // back wheel
				{
					getWheel(i)->setMinSteeringAngle(0);
					getWheel(i)->setMaxSteeringAngle(0);
				}
			}
			break;

		case DriveMode::DUAL_ACKERMANN:
			for(uint i=0; i<getNumWheels(); i++)
			{
				// getWheel(i)->setSteeringAngle(0);
				getWheel(i)->setMinSteeringAngle(-0.4);
				getWheel(i)->setMaxSteeringAngle(0.4);
				
			}
			break;

		case DriveMode::REAR_ACKERMANN:
			for(uint i=0; i<getNumWheels(); i++)
			{
				// getWheel(i)->setSteeringAngle(0);
				btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(i)->getWorldTransform();
				if (wheelTranCS.getOrigin()[getForwardAxis()] < 0) // rear wheel
				{
					getWheel(i)->setMinSteeringAngle(-0.4);
					getWheel(i)->setMaxSteeringAngle(0.4);
				}
				else // front wheel
				{
					getWheel(i)->setMinSteeringAngle(0);
					getWheel(i)->setMaxSteeringAngle(0);
				}
			}
			break;

		default:
			break;
	}
}
