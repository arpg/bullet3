#ifndef BT_WHEEL_H
#define BT_WHEEL_H

#include <string>
#include <iostream>
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

	btScalar m_angularVelocity; // positive speeds forward
	btScalar m_maxAngularVelocity;
	btScalar m_minAngularVelocity;

	btScalar m_steeringAngle; // positive is to the left
	btScalar m_minSteeringAngle;
	btScalar m_maxSteeringAngle;
	
	btScalar m_stallTorque;

	btScalar m_maxSteeringRate;

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

		m_angularVelocity = 0.f;
		m_maxAngularVelocity = 0.f;
		m_minAngularVelocity = -0.f;

		m_steeringAngle = 0.f;
		m_minSteeringAngle = -0.f;
		m_maxSteeringAngle = 0.f;

		m_stallTorque = INFINITY;

		m_maxSteeringRate = FLT_MAX;
	}	

	btWheel(const btVector3& chassisConnectionCS, const btVector3& wheelDirectionCS, const btVector3& wheelAxleCS, btScalar halfWidth, btScalar radius)
	{
		btCollisionObject* object = new btCollisionObject();
		btCylinderShape* shape;
		switch ((int)getWidthAxis())
		{
			case 0:
				shape = new btCylinderShapeX(btVector3(halfWidth, radius, radius));
				break;
			case 2:
				shape = new btCylinderShapeZ(btVector3(radius, radius, halfWidth));
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

	inline virtual btScalar getWidth() { return 2*((const btCylinderShape*)getShape())->getHalfExtentsWithoutMargin()[(int)getWidthAxis()]; }
	inline virtual btScalar getRadius() { return ((const btCylinderShape*)getShape())->getHalfExtentsWithoutMargin()[(int)getRadiusAxis()]; }
	
	inline virtual const btTransform getWorldTransform() { return getObject()->getWorldTransform(); }

	inline virtual btVector3 getWheelVelocity() { return btTransform(btTransform(btQuaternion(0,0,0,1),getBody()->getAngularVelocity())*getWorldTransform()).getOrigin(); }
	inline virtual btScalar getRadialVelocity() { return getWheelVelocity()[(int)getRadiusAxis()]; }

	inline virtual btScalar getFriction() { return m_friction; }
	inline virtual btScalar getStiffness() { return m_suspensionStiffness; }
	inline virtual btScalar getDamping() { return m_suspensionDamping; }
	inline virtual btScalar getMaxTravel() { return m_maxSuspensionTravel; }

	inline virtual bool isMotorEnabled() { return getStallTorque()!=0; }
	inline virtual bool isSteeringEnabled() { return getMaxSteeringAngle()!=0 || getMinSteeringAngle()!=0; }

	inline virtual btScalar getAngularVelocity() { return m_angularVelocity; }
	inline virtual btScalar getMinAngularVelocity() { return m_minAngularVelocity; }
	inline virtual btScalar getMaxAngularVelocity() { return m_maxAngularVelocity; }

	inline virtual btScalar getLinearVelocity() { return getAngularVelocity() * (getRadius() ); }

	inline virtual btScalar getSteeringAngle() { return m_steeringAngle; }
	inline virtual btScalar getMinSteeringAngle() { return m_minSteeringAngle; }
	inline virtual btScalar getMaxSteeringAngle() { return m_maxSteeringAngle; }

	inline virtual btScalar getStallTorque() { return m_stallTorque; }

	inline virtual btScalar getMaxSteeringRate() { return m_maxSteeringRate; }

	// setters
	// inline virtual void setShape(btCollisionShape* shape) { m_shape = shape; }

	inline virtual void setFriction(btScalar val) { m_friction = val; }
	inline virtual void setStiffness(btScalar val) { m_suspensionStiffness = val; }
	inline virtual void setDamping(btScalar val) { m_suspensionDamping = val; }
	inline virtual void setMaxTravel(btScalar val) { m_maxSuspensionTravel = val; }

	// If motor not enabled, keep motor force same as previous. This lets us lock forces/steering angles to play with, but these values still default to disabled and 0.
	inline virtual void setAngularVelocity(btScalar val) { m_angularVelocity = std::min(std::max( (isMotorEnabled() ? val : m_angularVelocity) , getMinAngularVelocity()), getMaxAngularVelocity()); }
	inline virtual void setMinAngularVelocity(btScalar val) { m_minAngularVelocity = val; m_angularVelocity = std::min(std::max( m_angularVelocity , getMinAngularVelocity()), getMaxAngularVelocity()); }
	inline virtual void setMaxAngularVelocity(btScalar val) { m_maxAngularVelocity = val; m_angularVelocity = std::min(std::max( m_angularVelocity , getMinAngularVelocity()), getMaxAngularVelocity()); }

	inline virtual void setSteeringAngle(btScalar val) { m_steeringAngle = std::min(std::max( (isSteeringEnabled() ? val : m_steeringAngle), getMinSteeringAngle()), getMaxSteeringAngle()); }
	inline virtual void setMinSteeringAngle(btScalar val) { m_minSteeringAngle = val; m_steeringAngle = std::min(std::max( m_steeringAngle , getMinSteeringAngle()), getMaxSteeringAngle()); }
	inline virtual void setMaxSteeringAngle(btScalar val) { m_maxSteeringAngle = val; m_steeringAngle = std::min(std::max( m_steeringAngle , getMinSteeringAngle()), getMaxSteeringAngle()); }

	inline virtual void setStallTorque(btScalar val) { m_stallTorque = val; }

	inline virtual void setMaxSteeringRate(btScalar val) { m_maxSteeringRate = val; }

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
		msg += line_prefix + 	"steeringEnabled " + 		std::to_string(isSteeringEnabled())		+ line_suffix +"\n";
		msg += line_prefix + 	"angularVelocity " + 		std::to_string(getAngularVelocity())	+ line_suffix +"\n";
		msg += line_prefix + 	"maxAngularVelocity " + 	std::to_string(getMaxAngularVelocity())	+ line_suffix +"\n";
		msg += line_prefix + 	"minAngularVelocity " + 	std::to_string(getMinAngularVelocity())	+ line_suffix +"\n";
		msg += line_prefix + 	"steeringAngle " + 			std::to_string(getSteeringAngle())		+ line_suffix +"\n";
		msg += line_prefix + 	"maxSteeringAngle " + 		std::to_string(getMaxSteeringAngle())	+ line_suffix +"\n";
		msg += line_prefix + 	"minSteeringAngle " + 		std::to_string(getMinSteeringAngle())	+ line_suffix +"\n";
		msg += line_prefix + 	"stallTorque " + 			std::to_string(getStallTorque())		+ line_suffix +"\n";
		msg += line_prefix + 	"maxSteeringRate " + 		std::to_string(getMaxSteeringRate())		+ line_suffix +"\n";
		msg += block_suffix;
		return msg;
	}
	static inline std::string vector2str(btVector3 v)
	{
		return std::to_string(v[0])+" "+std::to_string(v[1])+" "+std::to_string(v[2]);
	}

};

#endif  //BT_WHEEL_INFO_H
