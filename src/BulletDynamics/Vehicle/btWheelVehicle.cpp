#include "btWheelVehicle.h"
class btWheelVehicle;

btWheelVehicle::btWheelVehicle()
	: btVehicle()
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
	printf("Setting ang vel %f rad/s\n", val);
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
	printf("Setting lin vel %f m/s\n", val);
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
	printf("Setting ang accel %f over %f sec\n", accel, dt);
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
	printf("Setting steering %f rad\n", val);
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isSteeringEnabled())
		{
			getWheel(i)->setSteeringAngle(val);
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
void btWheelVehicle::btWheelVehicle::setAllStiffness(btScalar val)
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
