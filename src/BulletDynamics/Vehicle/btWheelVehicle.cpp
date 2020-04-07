#include "btWheelVehicle.h"
class btWheelVehicle;

btWheelVehicle::btWheelVehicle(btCollisionObject* chassisObject) : 
	btVehicle(chassisObject)
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

void btWheelVehicle::setEnabledMotorForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isMotorEnabled()) // not necessary but good habit
		{
			getWheel(i)->setMotorForce(val);
		}
	}
}
void btWheelVehicle::setEnabledTorqueForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isMotorEnabled()) // not necessary but good habit
		{
			getWheel(i)->setTorqueForce(val);
		}
	}
}
void btWheelVehicle::setEnabledBrakeForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isBrakeEnabled())
		{
			getWheel(i)->setBrakeForce(val);
		}
	}
}
void btWheelVehicle::setEnabledSteeringAngle(btScalar val)
{
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

void btWheelVehicle::setAllMinMotorForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMinMotorForce(val);
	}
}
void btWheelVehicle::setAllMaxMotorForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMaxMotorForce(val);
	}
}

void btWheelVehicle::setAllMinBrakeForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMinBrakeForce(val);
	}
}
void btWheelVehicle::setAllMaxBrakeForce(btScalar val)
{
	for(uint i=0; i<getNumWheels(); i++)
	{
		getWheel(i)->setMaxBrakeForce(val);
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

std::vector<btWheel*>& btWheelVehicle::getEnabledBrakeWheels()
{
	static std::vector<btWheel*> wheels;
	wheels.clear();
	for(uint i=0; i<getNumWheels(); i++)
	{
		if (getWheel(i)->isBrakeEnabled())
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
}
