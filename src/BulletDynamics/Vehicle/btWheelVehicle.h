#ifndef BT_WHEELVEHICLE_H
#define BT_WHEELVEHICLE_H

#include "BulletDynamics/Vehicle/btVehicle.h"
#include "btWheel.h"

class btWheelVehicle : public btVehicle
{
protected:
	btAlignedObjectArray<btWheel*> m_wheels;

	virtual void updateVehicle(btScalar step) = 0;
	virtual void debugDraw(btIDebugDraw* debugDrawer);

public:
	btWheelVehicle(btCollisionObject* chassisObject = new btCollisionObject());
	~btWheelVehicle();

	virtual btWheel* getWheel(int wheel);
	inline virtual int getNumWheels() { return (int)m_wheels.size(); }
	virtual void addWheel(const btVector3& chassisConnectionCS, btScalar width, btScalar radius);

	virtual void setEnabledMotorForce(btScalar);
	virtual void setEnabledTorqueForce(btScalar);
	virtual void setEnabledBrakeForce(btScalar);
	virtual void setEnabledSteeringAngle(btScalar);

	virtual void setAllFriction(btScalar);
	virtual void setAllStiffness(btScalar);
	virtual void setAllDamping(btScalar);
	virtual void setAllMaxTravel(btScalar);
	
	virtual void setAllMinMotorForce(btScalar);
	virtual void setAllMaxMotorForce(btScalar);

	virtual void setAllMinBrakeForce(btScalar);
	virtual void setAllMaxBrakeForce(btScalar);

	virtual void setAllMinSteeringAngle(btScalar);
	virtual void setAllMaxSteeringAngle(btScalar);

	virtual std::vector<btWheel*>& getEnabledMotorWheels();
	virtual std::vector<btWheel*>& getEnabledBrakeWheels();
	virtual std::vector<btWheel*>& getEnabledSteeringWheels();

	virtual btScalar getEnabledSteeringAngle();
};

#endif  //BT_WHEELVEHICLE_H
