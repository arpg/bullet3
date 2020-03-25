/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2015 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <vector>
#include <memory>
#include <iostream> // cout
#include <stdio.h>  //printf debugging

#include "Hinge2Vehicle.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

class btVehicleTuning;

class btCollisionShape;

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
// #include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

#define linVel2WheelVel(radius,linvel) linvel/radius

class Hinge2Vehicle : public CommonRigidBodyBase
{
public:
	// static float calcQuadBezier(float* pts, float tau);
	// static btVector3 calcCubicBezier(btVector3* pts, float tau);
	// static float calcCubicBezier(float* pts, float tau);
	// static btVector3 calcQuadBezier(btVector3* pts, float tau);
	// static float calcQuadBezier(float* pts, float tau);
	// static btVector3 calcCubicBezier(btVector3* pts, float tau);
	// static float calcCubicBezier(float* pts, float tau);
	template <typename T> 
	static T calcLinearBezier(T* pts, float tau);
	template <typename T> 
	static T calcNDBezier(T* pts, float tau, int degree);

	/* extra stuff*/
	btVector3 m_cameraPosition;

	btRigidBody* m_gndBody;
	btRigidBody* m_carChassis;
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;
	// int m_wheelInstances[4];

	// std::vector<btHinge2Constraint*> m_wheels;
	btTypedConstraint* m_wheels[4];
	void setMotorTargets();

	bool m_useDefaultCamera;
	//----------------------------

	class btTriangleIndexVertexArray* m_indexVertexArrays;

	btVector3* m_vertices;

	btCollisionShape* m_wheelShape;

	float m_cameraHeight;

	float m_minCameraDistance;
	float m_maxCameraDistance;

	Hinge2Vehicle(struct GUIHelperInterface* helper);

	virtual ~Hinge2Vehicle();

	virtual void stepSimulation(float deltaTime);

	virtual void resetForklift();

	virtual void clientResetScene();

	virtual void displayCallback();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	virtual bool keyboardCallback(int key, int state);

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);

	void initPhysics();
	void exitPhysics();

	virtual void resetCamera()
	{
		float dist = 8;
		float pitch = -32;
		float yaw = -45;
		float targetPos[3] = {0,0,2};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	/*static DemoApplication* Create()
	{
		Hinge2Vehicle* demo = new Hinge2Vehicle();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	*/
};

static const int rightIndex = 0;
static const int upIndex = 1;
static const int forwardIndex = 2;

static const int frontRightWheelIndex = 0;
static const int frontLeftWheelIndex = 1;
static const int backLeftWheelIndex = 2;
static const int backRightWheelIndex = 3;

// 6 DOF hinge constraint indices
// const int strafeIndex = 0; // lin along x
// const int lungeIndex = 2; // lin along z
static const int suspensionIndex = 2; // lin along y
static const int driveIndex = 3; // rot along x (right, principle axis) 
// const int wobbleIndex = 4; // rot along z (forward)
static const int steeringIndex = 5; // rot along y (up)

static bool useMCLPSolver = false;  //true;

//const int maxProxies = 32766;
//const int maxOverlap = 65535;

static float gTargetVelocity = 0.f;
// float gEngineForce = 0.f;
// float gBreakingForce = 100.f;

static float maxTargetVelocity = 4.f;
// static float deltaTargetVelocity = 0.2f;
static float maxMotorForce = 500.f;
// float maxEngineForce = 50.f;  //this should be engine/velocity dependent
// float maxBreakingForce = 100.f;
// float defaultBreakingForce = 10.f;
// btScalar maxMotorImpulse = 400.f;
	
static float gVehicleSteering = 0.f;
static float steeringIncrement = 0.04f;
static float steeringClamp = 1.0f;

// float wheelRadius = 0.5f;
// float wheelWidth = 0.4f;
static float wheelRadius = 0.075f;
static float wheelWidth = 0.05f;

//float	wheelFriction = 1000;//BT_LARGE_FLOAT;
static float suspensionStiffness = 700.f;
static float suspensionDamping = 10.3f;
//float	suspensionCompression = 4.4f;
//float	rollInfluence = 0.1f;//1.0f;
//btScalar suspensionRestLength(0.6);

static float chassisHalfWidth = 0.1;
static float chassisHalfHeight = 0.05;
static float chassisHalfLength = 0.4;

static float fps = 0.f;

static btScalar cfm = 0.9f;
static btScalar erp = 0.9f;

// static float throttle_tau;
static bool throttle_up = false;
static bool throttle_down = false;
static bool steer_left = false;
static bool steer_right = false;

static std::vector<float> vel_profile = {0, 0.5*maxTargetVelocity, maxTargetVelocity}; 

static btVector3 cog(0,0,-0.2);

////////////////////////////////////

Hinge2Vehicle::Hinge2Vehicle(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper),
	  m_carChassis(0),
	  m_guiHelper(helper),
	  m_indexVertexArrays(0),
	  m_vertices(0),
	  m_cameraHeight(4.f),
	  m_minCameraDistance(3.f),
	  m_maxCameraDistance(10.f)
{
	helper->setUpAxis(upIndex);

	m_wheelShape = 0;
	m_cameraPosition = btVector3(30, 30, 30);
	m_useDefaultCamera = false;
}

void Hinge2Vehicle::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				m_dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
			m_dynamicsWorld->removeRigidBody(body);
		}
		else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	delete m_wheelShape;
	m_wheelShape = 0;

	//delete solver
	delete m_solver;
	m_solver = 0;

	//delete broadphase
	delete m_broadphase;
	m_broadphase = 0;

	//delete dispatcher
	delete m_dispatcher;
	m_dispatcher = 0;

	delete m_collisionConfiguration;
	m_collisionConfiguration = 0;

	// for (int j = 0; j < 4; j++)
	// {
	// 	m_dynamicsWorld->removeConstraint(m_wheels[j]);
	// 	delete m_wheels[i];
	// 	m_wheels[i] = 0;
	// }
	// m_wheels.clear();
}

Hinge2Vehicle::~Hinge2Vehicle()
{
	//exitPhysics();
}

static void preTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
	Hinge2Vehicle* info = (Hinge2Vehicle*)world->getWorldUserInfo();
	info->setMotorTargets();
	fps = 1/timeStep;
}

// static void postTickCallback(btDynamicsWorld* world, btScalar timeStep)
// {
// 	throttle_up = false;
// 	throttle_down = false;
// 	steer_left = false;
// 	steer_right = false;
// }

void Hinge2Vehicle::initPhysics()
{
	{  // create a slider to change the number of pendula
		SliderParams slider("CFM", &cfm);
		slider.m_minVal = 0;
		slider.m_maxVal = 3;
		slider.m_clampToIntegers = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{  // create a slider to change the number of pendula
		SliderParams slider("ERP", &erp);
		slider.m_minVal = 0;
		slider.m_maxVal = 3;
		slider.m_clampToIntegers = false;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	m_guiHelper->setUpAxis(upIndex);

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_broadphase = new btAxisSweep3(worldMin, worldMax);
	if (useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		m_solver = sol;
	}
	else
	{
		m_solver = new btSequentialImpulseConstraintSolver();
	}
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	if (useMCLPSolver)
	{
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;  //for direct solver it is better to have a small A matrix
	}
	else
	{
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
	m_dynamicsWorld->setGravity(btVector3(0,-9.8,0));
	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	m_dynamicsWorld->setInternalTickCallback(preTickCallback, this, true);
	// m_dynamicsWorld->setInternalTickCallback(postTickCallback, this, false);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	// add ground
	btCompoundShape* groundCompound = new btCompoundShape();
	m_collisionShapes.push_back(groundCompound);
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
		m_collisionShapes.push_back(groundShape);
		
		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0, -3, 0));

		groundCompound->addChildShape(localTrans, groundShape);
	}
	// add box obstacles
	{
		btCollisionShape* shape = new btBoxShape(btVector3(5, 1, 1));
		m_collisionShapes.push_back(shape);
		
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(-5.0-0.5, 0, 0+1));
		groundCompound->addChildShape(tr, shape);
	}
	// add ramp
	{
		btVector3 p_bbl(5.0, 	0, 		0); // -y z x
		btVector3 p_bbr(-5.0, 	0, 		0);
		btVector3 p_bfr(-5.0, 	0, 		4.0);
		btVector3 p_bfl(5.0, 	0, 		4.0);
		btVector3 p_tfl(5.0, 	2.0, 	4.0);
		btVector3 p_tfr(-5.0, 	2.0, 	4.0);
    	btTriangleMesh *mesh = new btTriangleMesh();
		mesh->addTriangle(p_bbl, p_bbr, p_bfr);
		mesh->addTriangle(p_bbl, p_bfr, p_bfl);
		mesh->addTriangle(p_bbl, p_bbr, p_tfr);
		mesh->addTriangle(p_bbl, p_tfr, p_tfl);
		mesh->addTriangle(p_bbl, p_tfl, p_bfl);
		mesh->addTriangle(p_bbr, p_bfr, p_tfr);
		mesh->addTriangle(p_bfl, p_tfr, p_bfr);
		mesh->addTriangle(p_bfl, p_tfl, p_tfr);
    	btCollisionShape* shape = new btBvhTriangleMeshShape(mesh,true,true);
		m_collisionShapes.push_back(shape);

		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(5.0+0.5, 0, 0));
		groundCompound->addChildShape(tr, shape);
	}
	btTransform tr;
	tr.setIdentity();
	// tr.setOrigin(btVector3(0, -3, 0));
	m_gndBody = localCreateRigidBody(0, tr, groundCompound);
	m_gndBody->setFriction(1);

	// add chassis
	btCollisionShape* chassisShape = new btBoxShape(btVector3(chassisHalfWidth, chassisHalfHeight, chassisHalfLength)); // width (z), height (-y), length (x)
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 0, 0)-cog);
	compound->addChildShape(localTrans, chassisShape);

	const btScalar FALLHEIGHT = 1;
	tr.setOrigin(btVector3(0, FALLHEIGHT, 0));

	const btScalar chassisMass = 12.0f;
	const btScalar wheelMass = 1.0f;
	m_carChassis = localCreateRigidBody(chassisMass, tr, compound);  //chassisShape);

	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

	btVector3 wheelPos[4];
	wheelPos[frontRightWheelIndex] = 	btVector3(	-btScalar(chassisHalfWidth + wheelWidth),	btScalar(FALLHEIGHT-0-chassisHalfHeight),	btScalar(chassisHalfLength-wheelRadius)		)-cog;
	wheelPos[frontLeftWheelIndex] = 	btVector3(	btScalar(chassisHalfWidth + wheelWidth),	btScalar(FALLHEIGHT-0-chassisHalfHeight),	btScalar(chassisHalfLength-wheelRadius)		)-cog;
	wheelPos[backLeftWheelIndex] = 		btVector3(	btScalar(chassisHalfWidth + wheelWidth),	btScalar(FALLHEIGHT-0-chassisHalfHeight),	-btScalar(chassisHalfLength-wheelRadius)	)-cog;
	wheelPos[backRightWheelIndex] = 	btVector3(	-btScalar(chassisHalfWidth + wheelWidth),	btScalar(FALLHEIGHT-0-chassisHalfHeight),	-btScalar(chassisHalfLength-wheelRadius)	)-cog;

	for (int i = 0; i < 4; i++)
	{
		// create a Hinge2 joint
		// create two rigid bodies
		// static bodyA (parent) on top:

		btRigidBody* pBodyA = this->m_carChassis;
		pBodyA->setActivationState(DISABLE_DEACTIVATION);

		// dynamic bodyB (child) below it :
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(wheelPos[i]);
		btVector3 anchor = tr.getOrigin();

		// if (i==frontLeftWheelIndex || i==backLeftWheelIndex)
		// {
		// 	anchor += btVector3(-0.05f,0.f,0.f);
		// }
		// else
		// {
		// 	anchor += btVector3(0.05f,0.f,0.f);
		// }
		
		btRigidBody* pBodyB = createRigidBody(wheelMass, tr, m_wheelShape);
		pBodyB->setFriction(1);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);

		// add some data to build constraint frames
		btVector3 parentAxis(0.f, 1.f, 0.f);
		btVector3 childAxis(1.f, 0.f, 0.f);
		btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
		m_wheels[i] = pHinge2;

		//m_guiHelper->get2dCanvasInterface();
		
		// add constraint to world
		m_dynamicsWorld->addConstraint(pHinge2, true);

		// Drive engine.
		pHinge2->enableMotor(driveIndex, true);
		pHinge2->setMaxMotorForce(driveIndex, maxMotorForce);
		pHinge2->setTargetVelocity(driveIndex, 0);

		// Steering engine.
		pHinge2->enableMotor(steeringIndex, true);
		// pHinge2->setMaxMotorForce(5, 1000);
		// pHinge2->setTargetVelocity(5, 0);
		pHinge2->setServo(steeringIndex, true);
		pHinge2->setTargetVelocity(steeringIndex, 100);
		pHinge2->setMaxMotorForce(steeringIndex, 1000);
		pHinge2->setServoTarget(steeringIndex, 0);

		// suspension
		pHinge2->setParam( BT_CONSTRAINT_CFM, cfm, suspensionIndex );
		pHinge2->setParam( BT_CONSTRAINT_ERP, erp, suspensionIndex );
		pHinge2->setDamping( suspensionIndex, suspensionDamping, false);
		pHinge2->setStiffness( suspensionIndex, suspensionStiffness, false);

		// btVector3 linLowerLimits(FLT_MAX,FLT_MAX,FLT_MAX);
		// btVector3 linUpperLimits(FLT_MIN,FLT_MIN,FLT_MIN);
		// linLowerLimits[suspensionIndex] = -2*2*wheelRadius;
		// linUpperLimits[suspensionIndex] = 0.9*wheelRadius;
		// pHinge2->setLinearLowerLimit(linLowerLimits);
		// pHinge2->setLinearUpperLimit(linUpperLimits);
		
		// btVector3 angLowerLimits(FLT_MAX,FLT_MAX,FLT_MAX);
		// btVector3 angUpperLimits(FLT_MIN,FLT_MIN,FLT_MIN);
		// // if (i==frontRightWheelIndex || i==frontLeftWheelIndex)
		// // {
		// 	angLowerLimits[steeringIndex-3] = -1.f;
		// 	angUpperLimits[steeringIndex-3] = 1.f;
		// // }
		// // else
		// // {
		// // 	angLowerLimits[steeringIndex-3] = 0.f;
		// // 	angUpperLimits[steeringIndex-3] = 0.f;
		// // }
		// pHinge2->setAngularLowerLimit(angLowerLimits);
		// pHinge2->setAngularUpperLimit(angUpperLimits);

		pHinge2->setDbgDrawSize(btScalar(5.f));
	
		// pHinge2->setEquilibriumPoint();
	}

	resetForklift();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Hinge2Vehicle::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
}

//to be implemented by the demo
void Hinge2Vehicle::renderScene()
{
	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

	m_guiHelper->render(m_dynamicsWorld);

	btVector3 wheelColor(1, 0, 0);

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);
}

// void Hinge2Vehicle::setMotorTargets()
// {
// 	static btScalar last_steer_ang;
// 	static btScalar steer_ang;
// 	static uint neg_count;
// 	m_wheels[frontRightWheelIndex]->getRigidBodyB().getOrientation().getEulerZYX(*(new btScalar()), steer_ang, *(new btScalar()));
// 	std::cout << "steer " << std::to_string(steer_ang) << std::endl;
// 	if (last_steer_ang*steer_ang<0)
// 	{
// 		neg_count++;
// 		if (neg_count>5)
// 		{
// 			gVehicleSteering = 0;
// 			neg_count = 0;
// 		}
// 	}
// 	else
// 	{
// 		neg_count--;
// 	}
	
// 	static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->setTargetVelocity(driveIndex, -gTargetVelocity);
// 	static_cast<btHinge2Constraint*>(m_wheels[backRightWheelIndex])->setTargetVelocity(driveIndex, -gTargetVelocity);
// 	static_cast<btHinge2Constraint*>(m_wheels[frontLeftWheelIndex])->setTargetVelocity(driveIndex, -gTargetVelocity);
// 	static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->setTargetVelocity(driveIndex, -gTargetVelocity);
// 	static_cast<btHinge2Constraint*>(m_wheels[frontLeftWheelIndex])->setTargetVelocity(steeringIndex, -gVehicleSteering);
// 	static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->setTargetVelocity(steeringIndex, -gVehicleSteering);

// 	last_steer_ang = steer_ang;
// }

// btVector3 Hinge2Vehicle::calcQuadBezier(btVector3* pts, float tau)
// {
// 	assert(tau>=0 && tau<=1);
// 	return ( (1-tau)*((1-tau)*pts[0]+tau*pts[1]) + tau*((1-tau)*pts[1] +tau*pts[2]) );
// }

// btVector3 Hinge2Vehicle::calcCubicBezier(btVector3* pts, float tau)
// {
// 	assert(tau>=0 && tau<=1);
// 	btVector3 b1 = calcQuadBezier(pts,tau);
// 	btVector3 b2 = calcQuadBezier(pts+1,tau);
// 	return ( (1-tau)*b1 + tau*b2 );
// }

// float Hinge2Vehicle::calcQuadBezier(float* pts, float tau)
// {
// 	assert(tau>=0 && tau<=1);
// 	return ( (1-tau)*((1-tau)*pts[0]+tau*pts[1]) + tau*((1-tau)*pts[1] +tau*pts[2]) );
// }

// float Hinge2Vehicle::calcCubicBezier(float* pts, float tau)
// {
// 	assert(tau>=0 && tau<=1);
// 	float b1 = calcQuadBezier(pts,tau);
// 	float b2 = calcQuadBezier(pts+1,tau);
// 	return ( (1-tau)*b1 + tau*b2 );
// }

template <typename T>
T Hinge2Vehicle::calcLinearBezier(T* pts, float tau)
{
	return (1-tau)*pts[0] + tau*pts[1];
}

template <typename T>
T Hinge2Vehicle::calcNDBezier(T* pts, float tau, int degree)
{
	assert(tau>=0 && tau<=1);
	assert(degree>=0);
	
	T b1, b2;
	if (degree==1)
	{
		return calcLinearBezier<T>(pts, tau);
	}
	else
	{
		b1 = calcNDBezier<T>(pts,tau, degree-1);
		b2 = calcNDBezier<T>(pts+1,tau, degree-1);
	}
	return ( (1-tau)*b1 + tau*b2 );
}

void Hinge2Vehicle::setMotorTargets()
{
	static float throttle_tau;
	if (throttle_up)
	{
		// if (gTargetVelocity<0)
		// {
		// 	throttle_tau = 0;
		// }
		// else
		{
			throttle_tau+=0.1;
			throttle_tau = std::min(throttle_tau,1.f);
		}
		// gTargetVelocity = linVel2WheelVel(wheelRadius, calcCubicBezier(&vel_profile[0],throttle_tau));
		gTargetVelocity = calcNDBezier<float>(&vel_profile[0],throttle_tau,vel_profile.size()-1);

		printf("throttle up %f\n",throttle_tau);
	}
	else if (throttle_down)
	{
		// if (gTargetVelocity>0)
		// {
		// 	throttle_tau = 0;
		// }
		// else
		{
			throttle_tau-=0.1;
			throttle_tau = std::max(throttle_tau,-1.f);
		}
		// gTargetVelocity = -linVel2WheelVel(wheelRadius, calcCubicBezier(&vel_profile[0],throttle_tau));
		gTargetVelocity = -calcNDBezier<float>(&vel_profile[0],-throttle_tau,vel_profile.size()-1);
		
		printf("throttle down %f\n",throttle_tau);
	}
	else
	{
		throttle_tau = 0;
		gTargetVelocity = 0;

		printf("throttle off %f\n",throttle_tau);
	}
	printf("Target vel: %f\n",gTargetVelocity);

	// print vel profile
	// float t = 0.;
	// while (t<1.1)
	// {
	// 	printf("Target vel at %f: %f\n",t,calcNDBezier<float>(&vel_profile[0], t, vel_profile.size()-1));
	// 	t+=.1;
	// }	
	
	if (steer_left)
	{
		gVehicleSteering -= 0.025;
		gVehicleSteering = std::max(gVehicleSteering,-0.5f);

		printf("steer left %f\n",gVehicleSteering);
	}
	else if (steer_right)
	{
		gVehicleSteering += 0.025;
		gVehicleSteering = std::min(gVehicleSteering,0.5f);

		printf("steer right %f\n",gVehicleSteering);
	}
	else
	{
		gVehicleSteering *= 0.96;

		printf("steer straight %f\n",gVehicleSteering);
	}

	static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->setTargetVelocity(driveIndex, -linVel2WheelVel(wheelRadius,gTargetVelocity));
	static_cast<btHinge2Constraint*>(m_wheels[backRightWheelIndex])->setTargetVelocity(driveIndex, -linVel2WheelVel(wheelRadius,gTargetVelocity));
	static_cast<btHinge2Constraint*>(m_wheels[frontLeftWheelIndex])->setTargetVelocity(driveIndex, -linVel2WheelVel(wheelRadius,gTargetVelocity));
	static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->setTargetVelocity(driveIndex, -linVel2WheelVel(wheelRadius,gTargetVelocity));

	static_cast<btHinge2Constraint*>(m_wheels[backLeftWheelIndex])->setServoTarget(steeringIndex, 0);
	static_cast<btHinge2Constraint*>(m_wheels[backRightWheelIndex])->setServoTarget(steeringIndex, 0);
	static_cast<btHinge2Constraint*>(m_wheels[frontLeftWheelIndex])->setServoTarget(steeringIndex, gVehicleSteering);
	static_cast<btHinge2Constraint*>(m_wheels[frontRightWheelIndex])->setServoTarget(steeringIndex, gVehicleSteering);
}

void Hinge2Vehicle::stepSimulation(float deltaTime)
{
	printf("FPS: %f\n",1/deltaTime);

	float dt = deltaTime;

	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = 2;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

		if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
		{
			btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
			int numFallbacks = sol->getNumFallbacks();
			if (numFallbacks)
			{
				static int totalFailures = 0;
				totalFailures += numFallbacks;
				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
			}
			sol->setNumFallbacks(0);
		}

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
			}
			else
			{
				printf("Simulated (%i) steps\n", numSimSteps);
			}
		}
#endif  //VERBOSE_FEEDBACK
	}
	
	// float vel = pHinge2->getRigidBodyA().getLinearVelocity().norm();
	// std::cout << "vel " << std::to_string(vel) << std::endl;
}

void Hinge2Vehicle::displayCallback(void)
{
	//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	//	glFlush();
	//	glutSwapBuffers();
}

void Hinge2Vehicle::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void Hinge2Vehicle::resetForklift()
{
	gVehicleSteering = 0.f;
	// gBreakingForce = defaultBreakingForce;
	// gEngineForce = 0.f;

	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
	m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
}

bool Hinge2Vehicle::keyboardCallback(int key, int state)
{
	bool handled = false;
	bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

	if (state) // key down
	{
		{
			switch (key)
			{
				case 'j':
				case B3G_LEFT_ARROW:
				{
					// handled = true;
					// gVehicleSteering += steeringIncrement;
					// if (gVehicleSteering > steeringClamp)
					// 	gVehicleSteering = steeringClamp;
					
					// if (gVehicleSteering<0.f)
					// 	gVehicleSteering = 0;
					// else
					// 	gVehicleSteering = 0.5f;

					steer_left = true;

					break;
				}
				case 'l':
				case B3G_RIGHT_ARROW:
				{
					// handled = true;
					// gVehicleSteering -= steeringIncrement;
					// if (gVehicleSteering < -steeringClamp)
					// 	gVehicleSteering = -steeringClamp;

					// if (gVehicleSteering>0.f)
					// 	gVehicleSteering = 0;
					// else
					// 	gVehicleSteering = -0.5f;

					steer_right = true;

					break;
				}
				case 'i':
				case B3G_UP_ARROW:
				{
					// handled = true;
					// gEngineForce = maxEngineForce;
					// gBreakingForce = 0.f;

					// if (gTargetVelocity<0.f)
					// 	gTargetVelocity = 0;
					// else
					// 	gTargetVelocity = linVel2WheelVel(wheelRadius,maxTargetVelocity);

					throttle_up = true;

					break;
				}
				case 'k':
				case B3G_DOWN_ARROW:
				{
					// handled = true;
					// gEngineForce = -maxEngineForce;
					// gBreakingForce = 0.f;
					// gTargetVelocity = -maxTargetVelocity;

					// if (gTargetVelocity>0.f)
					// 	gTargetVelocity = 0;
					// else
					// 	gTargetVelocity = linVel2WheelVel(wheelRadius,-maxTargetVelocity);
					
					throttle_down = true;
					
					break;
				}

				case B3G_F7:
				{
					handled = true;
					btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
					world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
					printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
					break;
				}
				case B3G_F6:
				{
					handled = true;
					//switch solver (needs demo restart)
					useMCLPSolver = !useMCLPSolver;
					printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

					delete m_solver;
					if (useMCLPSolver)
					{
						btDantzigSolver* mlcp = new btDantzigSolver();
						//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
						btMLCPSolver* sol = new btMLCPSolver(mlcp);
						m_solver = sol;
					}
					else
					{
						m_solver = new btSequentialImpulseConstraintSolver();
					}

					m_dynamicsWorld->setConstraintSolver(m_solver);

					//exitPhysics();
					//initPhysics();
					break;
				}

				case B3G_F5:
					handled = true;
					m_useDefaultCamera = !m_useDefaultCamera;
					break;

				default:
					break;
			}
		}
	}
	else // key up
	{
		{
			switch (key)
			{
				case 'j':
				case B3G_LEFT_ARROW:
				{
				// 	handled = true;
				// 	gVehicleSteering += steeringIncrement;
				// 	if (gVehicleSteering > steeringClamp)
				// 		gVehicleSteering = steeringClamp;

					steer_left = false;
					steer_right = false;

					break;
				}
				case 'l':
				case B3G_RIGHT_ARROW:
				{
				// 	handled = true;
				// 	gVehicleSteering -= steeringIncrement;
				// 	if (gVehicleSteering < -steeringClamp)
				// 		gVehicleSteering = -steeringClamp;

					steer_left = false;
					steer_right = false;

					break;
				}
				case 'i':
				case B3G_UP_ARROW:
				{
				// 	handled = true;
				// 	// gEngineForce = maxEngineForce;
				// 	// gBreakingForce = 0.f;
				// 	gTargetVelocity = 0.f;

					throttle_up = false;
					throttle_down = false;

					break;
				}
				case 'k':
				case B3G_DOWN_ARROW:
				{
				// 	handled = true;
				// 	// gEngineForce = -maxEngineForce;
				// 	// gBreakingForce = 0.f;
				// 	gTargetVelocity = 0.f;

					throttle_up = false;
					throttle_down = false;

					break;
				}
					
				default:
					break;
			}
		}
	}
	return true;
	return handled;
}

void Hinge2Vehicle::specialKeyboardUp(int key, int x, int y)
{
}

void Hinge2Vehicle::specialKeyboard(int key, int x, int y)
{
}

btRigidBody* Hinge2Vehicle::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
	body->setWorldTransform(startTransform);
#endif  //

	m_dynamicsWorld->addRigidBody(body);
	return body;
}

CommonExampleInterface* Hinge2VehicleCreateFunc(struct CommonExampleOptions& options)
{
	return new Hinge2Vehicle(options.m_guiHelper);
}
