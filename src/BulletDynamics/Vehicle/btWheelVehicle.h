#ifndef BT_WHEELVEHICLE_H
#define BT_WHEELVEHICLE_H

#include "BulletDynamics/Vehicle/btVehicle.h"
#include "btWheel.h"

class btWheelVehicle : public btVehicle
{
public:
	enum DriveMode{ DIFF=0, ACKERMANN=1, DUAL_ACKERMANN=2};

protected:
	btAlignedObjectArray<btWheel*> m_wheels;

	btWheelVehicle::DriveMode m_driveMode;

	virtual void configureForDriveMode();

	virtual void updateVehicle(btScalar step) = 0;
	virtual void debugDraw(btIDebugDraw* debugDrawer);

public:
	btWheelVehicle();
	~btWheelVehicle();

    void setDriveMode(DriveMode);

	void resetSuspension();

	virtual btWheel* getWheel(int wheel);
	inline virtual int getNumWheels() { return (int)m_wheels.size(); }
	virtual void addWheel(const btVector3& chassisConnectionCS, btScalar width, btScalar radius);

	virtual void setEnabledAngularVelocity(btScalar);
	virtual void setEnabledLinearVelocity(btScalar);
	virtual void setEnabledAngularAcceleration(btScalar accel, btScalar dt);
	virtual void setEnabledSteeringAngle(btScalar);
	virtual void setEnabledYawVelocity(btScalar);

	virtual void setAllFriction(btScalar);
	virtual void setAllStiffness(btScalar);
	virtual void setAllDamping(btScalar);
	virtual void setAllMaxTravel(btScalar);
	
	virtual void setAllMinAngularVelocity(btScalar);
	virtual void setAllMaxAngularVelocity(btScalar);

	virtual void setAllMinSteeringAngle(btScalar);
	virtual void setAllMaxSteeringAngle(btScalar);

	virtual std::vector<btWheel*>& getEnabledMotorWheels();
	virtual std::vector<btWheel*>& getEnabledSteeringWheels();

	virtual btScalar getEnabledSteeringAngle();

};

#endif  //BT_WHEELVEHICLE_H
