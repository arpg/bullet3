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
#ifndef BT_WHEEL_INFO_H
#define BT_WHEEL_INFO_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

struct btWheelContactPoint
{
	btRigidBody* m_body0;
	btRigidBody* m_body1;
	btVector3 m_frictionPositionWorld;
	btVector3 m_frictionDirectionWorld;
	btScalar m_jacDiagABInv;
	btScalar m_maxImpulse;

	btWheelContactPoint(btRigidBody* body0, btRigidBody* body1, const btVector3& frictionPosWorld, const btVector3& frictionDirectionWorld, btScalar maxImpulse)
		: m_body0(body0),
		  m_body1(body1),
		  m_frictionPositionWorld(frictionPosWorld),
		  m_frictionDirectionWorld(frictionDirectionWorld),
		  m_maxImpulse(maxImpulse)
	{
		btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		btScalar relaxation = 1.f;
		m_jacDiagABInv = relaxation / (denom0 + denom1);
	}
};

struct btWheelInfoConstructionInfo
{
	btVector3 m_chassisConnectionCS;
	btVector3 m_wheelDirectionCS;
	btVector3 m_wheelAxleCS;
	btScalar m_suspensionRestLength;
	btScalar m_maxSuspensionTravelCm;
	btScalar m_wheelRadius;

	btScalar m_suspensionStiffness;
	btScalar m_wheelsDampingCompression;
	btScalar m_wheelsDampingRelaxation;
	btScalar m_frictionSlip;
	btScalar m_maxSuspensionForce;
	bool m_bIsFrontWheel;
};

/// btWheelInfo contains information per wheel about friction and suspension.
struct btWheelInfo
{
	struct RaycastInfo
	{
		//set by raycaster
		btVector3 m_contactNormalWS;  //contactnormal
		btVector3 m_contactPointWS;   //raycast hitpoint
		btScalar m_suspensionLength;
		btVector3 m_hardPointWS;       //raycast starting point
		btVector3 m_wheelDirectionWS;  //direction in worldspace
		btVector3 m_wheelAxleWS;       // axle in worldspace
		bool m_isInContact;
		void* m_groundObject;  //could be general void* ptr
	};

	RaycastInfo m_raycastInfo;

	btTransform m_worldTransform;

	btVector3 m_chassisConnectionPointCS;  //const
	btVector3 m_wheelDirectionCS;          //const
	btVector3 m_wheelAxleCS;               // const or modified by steering
	btScalar m_suspensionRestLength1;      //const
	btScalar m_maxSuspensionTravelCm;
	btScalar getSuspensionRestLength() const;
	btScalar m_suspensionStiffness;       //const
	btScalar m_wheelsDampingCompression;  //const
	btScalar m_wheelsDampingRelaxation;   //const
	btScalar m_frictionSlip;
	btScalar m_rotation;
	btScalar m_deltaRotation;
	btScalar m_rollInfluence;
	btScalar m_maxSuspensionForce;

	btScalar m_wheelsRadius;              //const
	btScalar m_wheelsMass;
	btScalar m_wheelsWidth;

	btScalar m_steering;
	btScalar m_steeringClamp;

	btScalar m_engineForce;
	btScalar m_maxEngineForce;
	btScalar m_minEngineForce;

	btScalar m_brake;

	bool m_bIsFrontWheel;

	void* m_clientInfo;  //can be used to store pointer to sync transforms...

	btWheelInfo() {}

	btWheelInfo(btWheelInfoConstructionInfo& ci)

	{
		m_suspensionRestLength1 = ci.m_suspensionRestLength;
		m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

		m_suspensionStiffness = ci.m_suspensionStiffness;
		m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
		m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
		m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
		m_wheelDirectionCS = ci.m_wheelDirectionCS;
		m_wheelAxleCS = ci.m_wheelAxleCS;
		m_frictionSlip = ci.m_frictionSlip;
		m_rotation = btScalar(0.);
		m_deltaRotation = btScalar(0.);
		m_rollInfluence = btScalar(0.1);
		m_bIsFrontWheel = ci.m_bIsFrontWheel;
		m_maxSuspensionForce = ci.m_maxSuspensionForce;

		m_wheelsRadius = ci.m_wheelRadius;
		m_wheelsMass = btScalar(1.);;
		m_wheelsWidth = btScalar(1.);;

		m_steering = btScalar(0.);
		m_steeringClamp = btScalar(1.);

		m_engineForce = btScalar(0.);
		m_maxEngineForce = btScalar(500.);
		m_minEngineForce = btScalar(-500.);

		m_brake = btScalar(0.);
	}

	void updateWheel(const btRigidBody& chassis, RaycastInfo& raycastInfo);

	btScalar m_clippedInvContactDotSuspension;
	btScalar m_suspensionRelativeVelocity;
	//calculated by suspension
	btScalar m_wheelsSuspensionForce;
	btScalar m_skidInfo;
};

#endif  //BT_WHEEL_INFO_H
