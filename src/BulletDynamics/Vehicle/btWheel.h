#ifndef BT_WHEEL_H
#define BT_WHEEL_H

#include <string>
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletCollisionCommon.h"
// #include "Bullet3Common/b3Logging.h"

class btWheel
{
protected:
	// kinematics
	const btVector3 m_chassisConnectionCS; // position of wheels relative to chassis
	const btVector3 m_wheelDirectionCS; // direction of wheels relative to chassis (chassis down)
	const btVector3 m_wheelAxleCS; // relative to chassis
	btCollisionObject* m_collisionObject;

	// dynamics
	btScalar m_friction;
	btScalar m_suspensionStiffness;
	btScalar m_suspensionDamping;
	btScalar m_maxSuspensionTravel;

	// control
	bool m_motorEnabled;
	bool m_brakeEnabled;
	bool m_steeringEnabled;

	btScalar m_motorForce; // positive speeds forward
	btScalar m_maxMotorForce;
	btScalar m_minMotorForce;

	btScalar m_brakeForce; // positive slows downs
	btScalar m_maxBrakeForce;
	btScalar m_minBrakeForce;

	btScalar m_steeringAngle; // positive is to the right
	btScalar m_minSteeringAngle;
	btScalar m_maxSteeringAngle;

	btScalar m_maxVelocity; // angular velocity limit along radial axis

public:
	btWheel(const btVector3& chassisConnectionCS, const btVector3& wheelDirectionCS, const btVector3& wheelAxleCS, btCollisionObject* object) :
		m_chassisConnectionCS(chassisConnectionCS),
		m_wheelDirectionCS(wheelDirectionCS),
		m_wheelAxleCS(wheelAxleCS),
		m_collisionObject(object) 
	{
		m_friction = 1.0f;
		m_suspensionStiffness = 200.f;
		m_suspensionDamping = 2.3f;
		m_maxSuspensionTravel = 0.1f;

		m_motorEnabled = false;
		m_brakeEnabled = false;
		m_steeringEnabled = false;

		m_motorForce = 0.f;
		m_maxMotorForce = 200.f;
		m_minMotorForce = -200.f;

		m_brakeForce = 0.f;
		m_maxBrakeForce = 100.f;
		m_minBrakeForce = 0.f;

		m_steeringAngle = 0.f;
		m_minSteeringAngle = -0.5f;
		m_maxSteeringAngle = 0.5f;

		m_maxVelocity = 5*2*M_PI;
	}	

	btWheel(const btVector3& chassisConnectionCS, const btVector3& wheelDirectionCS, const btVector3& wheelAxleCS, btScalar width, btScalar radius)
	{
		btCollisionObject* object = new btCollisionObject();
		btCylinderShape* shape;
		switch ((int)getWidthAxis())
		{
			case 0:
				shape = new btCylinderShapeX(btVector3(width, radius, radius));
				break;
			case 2:
				shape = new btCylinderShapeZ(btVector3(radius, radius, width));
				break;
			default:
				printf("ERROR\n");
				break;
		} 
		object->setCollisionShape(shape);
		btWheel(chassisConnectionCS, wheelDirectionCS, wheelAxleCS, object);
	}	

	~btWheel()
	{
		delete m_collisionObject;
	}

	// getters
	inline virtual btVector3 getChassisConnection() { return m_chassisConnectionCS; }

	inline virtual btCollisionObject* getObject() const { return m_collisionObject; }
	inline virtual btCollisionShape* getShape() const { return getObject()->getCollisionShape(); }
	inline virtual btRigidBody* getBody() const
	{
		btRigidBody* body = btRigidBody::upcast(getObject());
		if (body)
			return body;
		// else
		// {
		// 	b3Printf("No rigid body associated with wheel shape.")
		// }
		return 0;
	}

	inline virtual btVector3 getAxisVector(int axis) 
	{
		return getWorldTransform().getBasis().getColumn(axis);
	}

	inline virtual btScalar getWidthAxis() { return ((const btCylinderShape*)getShape())->getUpAxis(); }
	inline virtual btScalar getRadiusAxis() { return ((int)getWidthAxis() + 1)%3; }

	inline virtual btScalar getWidth() { return ((const btCylinderShape*)getShape())->getHalfExtentsWithoutMargin()[(int)getWidthAxis()]; }
	inline virtual btScalar getRadius() { return ((const btCylinderShape*)getShape())->getHalfExtentsWithoutMargin()[(int)getRadiusAxis()]; }
	
	inline virtual const btTransform getWorldTransform() { return getObject()->getWorldTransform(); }

	inline virtual btVector3 getWheelVelocity() { return btTransform(btTransform(btQuaternion(0,0,0,1),getBody()->getAngularVelocity())*getWorldTransform()).getOrigin(); }
	inline virtual btScalar getRadialVelocity() { return getWheelVelocity()[(int)getRadiusAxis()]; }

	inline virtual btScalar getFriction() { return m_friction; }
	inline virtual btScalar getStiffness() { return m_suspensionStiffness; }
	inline virtual btScalar getDamping() { return m_suspensionDamping; }
	inline virtual btScalar getMaxTravel() { return m_maxSuspensionTravel; }

	inline virtual bool isMotorEnabled() { return m_motorEnabled; }
	inline virtual bool isBrakeEnabled() { return m_brakeEnabled; }
	inline virtual bool isSteeringEnabled() { return m_steeringEnabled; }

	inline virtual btScalar getMotorForce() { return m_motorForce; }
	inline virtual btScalar getMinMotorForce() { return m_minMotorForce; }
	inline virtual btScalar getMaxMotorForce() { return m_maxMotorForce; }

	inline virtual btScalar getTorqueForce() { return getMotorForce() / getRadius(); }

	inline virtual btScalar getBrakeForce() { return m_brakeForce; }
	inline virtual btScalar getMinBrakeForce() { return m_minBrakeForce; }
	inline virtual btScalar getMaxBrakeForce() { return m_maxBrakeForce; }

	inline virtual btScalar getSteeringAngle() { return m_steeringAngle; }
	inline virtual btScalar getMinSteeringAngle() { return m_minSteeringAngle; }
	inline virtual btScalar getMaxSteeringAngle() { return m_maxSteeringAngle; }

	inline virtual btScalar getMaxVelocity() { return m_maxVelocity; }

	// setters
	// inline virtual void setShape(btCollisionShape* shape) { m_shape = shape; }

	inline virtual void setFriction(btScalar val) { m_friction = val; }
	inline virtual void setStiffness(btScalar val) { m_suspensionStiffness = val; }
	inline virtual void setDamping(btScalar val) { m_suspensionDamping = val; }
	inline virtual void setMaxTravel(btScalar val) { m_maxSuspensionTravel = val; }

	inline virtual void setMotorEnabled(bool enabled) { m_motorEnabled = enabled; }
	inline virtual void setBrakeEnabled(bool enabled) { m_brakeEnabled = enabled; }
	inline virtual void setSteeringEnabled(bool enabled) { m_steeringEnabled = enabled; }

	// If motor not enabled, keep motor force same as previous. This lets us lock forces/steering angles to play with, but these values still default to disabled and 0.
	inline virtual void setMotorForce(btScalar val) { m_motorForce = std::min(std::max( (isMotorEnabled() ? val : m_motorForce) , getMinMotorForce()), getMaxMotorForce()); printf("Motor set to %f\n",m_motorForce); }
	inline virtual void setMinMotorForce(btScalar val) { m_minMotorForce = val; m_motorForce = std::min(std::max( m_motorForce , getMinMotorForce()), getMaxMotorForce()); }
	inline virtual void setMaxMotorForce(btScalar val) { m_maxMotorForce = val; m_motorForce = std::min(std::max( m_motorForce , getMinMotorForce()), getMaxMotorForce()); }

	inline virtual void setTorqueForce(btScalar val) { setMotorForce(val * getRadius()); }

	inline virtual void setBrakeForce(btScalar val) { m_brakeForce = std::min(std::max( (isBrakeEnabled() ? val : m_brakeForce) , getMinBrakeForce()), getMaxBrakeForce()); printf("Brake set to %f\n",m_brakeForce); }
	inline virtual void setMinBrakeForce(btScalar val) { m_minBrakeForce = val; m_brakeForce = std::min(std::max( m_brakeForce , getMinBrakeForce()), getMaxBrakeForce()); }
	inline virtual void setMaxBrakeForce(btScalar val) { m_maxBrakeForce = val; m_brakeForce = std::min(std::max( m_brakeForce , getMinBrakeForce()), getMaxBrakeForce()); }

	inline virtual void setSteeringAngle(btScalar val) { m_steeringAngle = std::min(std::max( (isSteeringEnabled() ? val : m_steeringAngle), getMinSteeringAngle()), getMaxSteeringAngle()); printf("Steering set to %f\n",m_steeringAngle); }
	inline virtual void setMinSteeringAngle(btScalar val) { m_minSteeringAngle = val; m_steeringAngle = std::min(std::max( m_steeringAngle , getMinSteeringAngle()), getMaxSteeringAngle()); }
	inline virtual void setMaxSteeringAngle(btScalar val) { m_maxSteeringAngle = val; m_steeringAngle = std::min(std::max( m_steeringAngle , getMinSteeringAngle()), getMaxSteeringAngle()); }

	inline virtual void setMaxVelocity(btScalar val) { m_maxVelocity = val; }

	// btVector3 fric = static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->getRigidBodyB().getWorldTransform().getBasis().getColumn(2) * 1.0
	// 	+ static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->getRigidBodyB().getWorldTransform().getBasis().getColumn(0) * 1.0;
	// fric.normalize();
	// static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->getRigidBodyB().setAnisotropicFriction(fric);
	// static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->getRigidBodyB().setSpinningFriction(.02);
	// getWheel(i)->getBody()->setRollingFriction(0.05);

	virtual inline
	std::string str(std::string block_prefix="", std::string block_suffix="", std::string line_prefix="", std::string line_suffix="")
	{	
		std::string msg;
		msg += block_prefix;
		msg += line_prefix + 	"chassisConnection " + 		vector2str(m_chassisConnectionCS) 		+ line_suffix +"\n";
		msg += line_prefix + 	"wheelDirection " + 		vector2str(m_wheelDirectionCS)			+ line_suffix +"\n";
		msg += line_prefix + 	"wheelAxle " + 				vector2str(m_wheelAxleCS)				+ line_suffix +"\n";
		msg += line_prefix + 	"friction " + 				std::to_string(getFriction())			+ line_suffix +"\n";
		msg += line_prefix + 	"suspensionStiffness " + 	std::to_string(getStiffness())			+ line_suffix +"\n";
		msg += line_prefix + 	"suspensionDamping " + 		std::to_string(getDamping())			+ line_suffix +"\n";
		msg += line_prefix + 	"suspensionMaxTravel " + 	std::to_string(getMaxTravel())			+ line_suffix +"\n";
		msg += line_prefix + 	"motorEnabled " + 			std::to_string(isMotorEnabled())		+ line_suffix +"\n";
		msg += line_prefix + 	"brakeEnabled " + 			std::to_string(isBrakeEnabled())		+ line_suffix +"\n";
		msg += line_prefix + 	"steeringEnabled " + 		std::to_string(isSteeringEnabled())		+ line_suffix +"\n";
		msg += line_prefix + 	"motorForce " + 			std::to_string(getMotorForce())			+ line_suffix +"\n";
		msg += line_prefix + 	"maxMotorForce " + 			std::to_string(getMaxMotorForce())		+ line_suffix +"\n";
		msg += line_prefix + 	"minMotorForce " + 			std::to_string(getMinMotorForce())		+ line_suffix +"\n";
		msg += line_prefix + 	"brakeForce " + 			std::to_string(getBrakeForce())			+ line_suffix +"\n";
		msg += line_prefix + 	"maxBrakeForce " + 			std::to_string(getMaxBrakeForce())		+ line_suffix +"\n";
		msg += line_prefix + 	"minBrakeForce " + 			std::to_string(getMinBrakeForce())		+ line_suffix +"\n";
		msg += line_prefix + 	"steeringAngle " + 			std::to_string(getSteeringAngle())		+ line_suffix +"\n";
		msg += line_prefix + 	"maxSteeringAngle " + 		std::to_string(getMaxSteeringAngle())	+ line_suffix +"\n";
		msg += line_prefix + 	"minSteeringAngle " + 		std::to_string(getMinSteeringAngle())	+ line_suffix +"\n";
		msg += block_suffix;
		return msg;
	}
	static inline std::string vector2str(btVector3 v)
	{
		return std::to_string(v[0])+" "+std::to_string(v[1])+" "+std::to_string(v[2]);
	}

};

#endif  //BT_WHEEL_INFO_H
