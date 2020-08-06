#include "btWheelVehicle.h"
class btWheelVehicle;

btWheelVehicle::btWheelVehicle()
	: btVehicle(),
	  m_driveMode(ACKERMANN)
{	
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
	// for (int v = 0; v < this->getNumWheels(); v++)
	// {
	// 	btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

	// 	btVector3 axle = btVector3(
	// 		getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
	// 		getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
	// 		getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

	// 	//debug wheels (cylinders)
	// 	debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
	// 	debugDrawer->drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_contactPointWS, wheelColor);
	// }
}

void btWheelVehicle::setDriveMode(DriveMode mode)
{
	m_driveMode = mode;
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
void btWheelVehicle::setEnabledYawVelocity(btScalar val)
{
	printf("Setting yaw vel %f rad/s", val);

	configureForDriveMode();

	switch (m_driveMode)
	{
		case DriveMode::DIFF :
		{
			printf(" in diff mode\n");
			for(uint i=0; i<getNumWheels(); i++)
			{
				btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(i)->getWorldTransform();
				btScalar lin_vel = getCurrentSpeedMS() - wheelTranCS.getOrigin()[getRightAxis()] * val;
				btScalar ang_vel = lin_vel / getWheel(i)->getRadius();
				printf("\twheel %d ang vel %f rad/s\n", i, ang_vel);
				getWheel(i)->setAngularVelocity(ang_vel);
			}
			break;
		}
		case DriveMode::ACKERMANN :
		{
			printf(" in ackermann mode\n");
			btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(0)->getWorldTransform();
			btScalar wheel_base = 2*fabs(wheelTranCS.getOrigin()[getForwardAxis()]);
			btScalar steering_angle = atan2(val * wheel_base, getCurrentSpeedMS());
			printf("\tenabled steering %f rad\n", steering_angle);
			setEnabledSteeringAngle(steering_angle);
			break;
		}
		case DriveMode::DUAL_ACKERMANN :
		{
			printf(" in dual ackermann mode\n");
			for(uint i=0; i<getNumWheels(); i++)
			{
				btTransform wheelTranCS = getChassisWorldTransform().inverse() * getWheel(i)->getWorldTransform();
				btScalar wheel_base = wheelTranCS.getOrigin()[getForwardAxis()];
				btScalar steering_angle = atan2(val * wheel_base, getCurrentSpeedMS());
				// setEnabledSteeringAngle(atan2(val * wheel_base, getCurrentSpeedMS()));
				printf("\twheel %d steering %f rad\n", i, steering_angle);
				getWheel(i)->setSteeringAngle(steering_angle);
			}
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

		default:
			break;
	}
}