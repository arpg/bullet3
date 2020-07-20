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

#include "Hinge2Vehicle.h"
#include <string>
// #include "LinearMath/btBezier.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "BulletDynamics/Vehicle/btHinge2Vehicle.h"


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
	btHinge2Vehicle* m_vehicle;
	btRigidBody* m_gndBody;
	GUIHelperInterface* m_guiHelper;

	btScalar m_vehicleForce;
	btScalar m_vehicleSteering;

	Hinge2Vehicle(struct GUIHelperInterface* helper);

	virtual ~Hinge2Vehicle();

	// virtual void stepSimulation(float deltaTime);

	virtual void resetForklift();

	virtual void clientResetScene();

	virtual void displayCallback();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	virtual bool keyboardCallback(int key, int state);
	void setTargets();

	virtual void renderScene();

	void spawnEnvironment();
	void spawnVehicle(btVector3 chassisHalfExtents, btScalar chassisMass, btScalar wheelRadius, btScalar wheelHalfWidth, btScalar wheelMass, btScalar suspensionStiffness, btScalar suspensionDamping);
	void spawnCar();
	void spawnCarPlannerCar();
	void spawnBuggie();

	void initPhysics();
	void exitPhysics();

	virtual void resetCamera();

};

static bool useMCLPSolver = false;

static int rightIndex = 1;
static int upIndex = 2;
static int forwardIndex = 0;

// static float throttle_tau;
static bool throttle_up = false;
static bool throttle_down = false;
static bool steer_left = false;
static bool steer_right = false;
static bool space = false;

// static std::vector<float> vel_profile = {0, 0.5*maxTargetVelocity, maxTargetVelocity}; 
////////////////////////////////////

Hinge2Vehicle::Hinge2Vehicle(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper),
	  m_guiHelper(helper)
{
}

Hinge2Vehicle::~Hinge2Vehicle()
{
}

static void preTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
	Hinge2Vehicle* info = (Hinge2Vehicle*)world->getWorldUserInfo();
	info->setTargets();
	// fps = 1/timeStep;

	std::string prtmsg;
	for (int i = 0; i < info->m_vehicle->getNumWheels(); i++)
	{
		prtmsg += "Wheel "+std::to_string(i)+":\n";
		prtmsg += info->m_vehicle->wheel2str(i,"","","\t","");
	}
	std::cout << prtmsg;
	std::cout << "Speed " << std::to_string(info->m_vehicle->getCurrentSpeedMS()) << " m/s" << std::endl;
}	

void Hinge2Vehicle::initPhysics()
{
	m_guiHelper->setUpAxis(upIndex);

	createEmptyDynamicsWorld();
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
	m_dynamicsWorld->setGravity(btVector3(0,0,-9.8));
	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	m_dynamicsWorld->setInternalTickCallback(preTickCallback, this, true);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	spawnEnvironment();

	// spawnCar();
	spawnCarPlannerCar();
	// spawnBuggie();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Hinge2Vehicle::exitPhysics()
{
}

void Hinge2Vehicle::spawnEnvironment()
{
	// add ground
	btCompoundShape* groundCompound = new btCompoundShape();
	m_collisionShapes.push_back(groundCompound);
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(50, 50, .5));
		m_collisionShapes.push_back(groundShape);
		
		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0, 0, -.5));

		groundCompound->addChildShape(localTrans, groundShape);
	}
	// add box obstacles
	{
		btCollisionShape* shape = new btBoxShape(btVector3(.5, 5, .5));
		m_collisionShapes.push_back(shape);
		
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0.5, -6, 0.5));
		groundCompound->addChildShape(tr, shape);
	}
	// add ramp
	{
		btVector3 p_bbl(0,		-5.0, 	0); 
		btVector3 p_bbr(0,		5.0, 	0);
		btVector3 p_bfr(4.0,	5.0, 	0);
		btVector3 p_bfl(4.0,	-5.0, 	0);
		btVector3 p_tfl(4.0,	-5.0, 	2.0);
		btVector3 p_tfr(4.0,	5.0, 	2.0);
    	btTriangleMesh *mesh = new btTriangleMesh();
		mesh->addTriangle(p_bbl, p_bbr, p_bfr);
		mesh->addTriangle(p_bbl, p_bfr, p_bfl);
		mesh->addTriangle(p_bbl, p_tfr, p_bbr);
		mesh->addTriangle(p_bbl, p_tfl, p_tfr);
		mesh->addTriangle(p_bbl, p_bfl, p_tfl);
		mesh->addTriangle(p_bbr, p_tfr, p_bfr);
		mesh->addTriangle(p_bfl, p_bfr, p_tfr);
		mesh->addTriangle(p_bfl, p_tfr, p_tfl);
    	btCollisionShape* shape = new btBvhTriangleMeshShape(mesh,true,true);
		m_collisionShapes.push_back(shape);

		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 6, 0));
		groundCompound->addChildShape(tr, shape);
	}
	btTransform tr;
	tr.setIdentity();
	// tr.setOrigin(btVector3(0, -3, 0));
	m_gndBody = createRigidBody(0, tr, groundCompound);
	m_gndBody->setFriction(1);
}


void Hinge2Vehicle::spawnVehicle(btVector3 chassisHalfExtents, btScalar chassisMass, btScalar wheelRadius, btScalar wheelHalfWidth, btScalar wheelMass, btScalar suspensionStiffness, btScalar suspensionDamping)
{
	static std::string prtmsg = "Spawning vehicle:\n";
	static float chassisHalfWidth = chassisHalfExtents[0];
	static float chassisHalfHeight = chassisHalfExtents[1];
	static float chassisHalfLength = chassisHalfExtents[2]-.2;

	prtmsg += "\tchassisMass: "+std::to_string(chassisMass)+"\n";
	prtmsg += "\tchassisHalfWidth: "+std::to_string(chassisHalfWidth)+"\n";
	prtmsg += "\tchassisHalfHeight: "+std::to_string(chassisHalfHeight)+"\n";
	prtmsg += "\tchassisHalfLength: "+std::to_string(chassisHalfLength)+"\n";

	// add chassis
	btCollisionShape* chassisShape = new btBoxShape(btVector3(chassisHalfLength, chassisHalfWidth, chassisHalfHeight)); // width (z), height (-y), length (x)
	// m_collisionShapes.push_back(chassisShape);

	// btCompoundShape* chassisShape = new btCompoundShape();
	// m_collisionShapes.push_back(chassisShape);
	// btTransform localTrans;
	// localTrans.setIdentity();
	btVector3 cog(0,0,0.);
	// //localTrans effectively shifts the center of mass with respect to the chassis
	// localTrans.setOrigin(btVector3(0, 0, 0)-cog);
	// chassisShape->addChildShape(localTrans, chassisShape);

	prtmsg += "\tCOG: "+std::to_string(cog[0])+" "+std::to_string(cog[1])+" "+std::to_string(cog[2])+"\n";

	btScalar FALLHEIGHT = 5;
	btTransform chassisTran;
	chassisTran.setIdentity();
	btVector3 pos(-3, 0, FALLHEIGHT);
	chassisTran.setOrigin(pos);

	// btScalar chassisMass = 80.0f;
	btRigidBody* chassisBody = btVehicle::createLocalRigidBody(wheelMass, chassisTran, chassisShape);
	m_dynamicsWorld->addRigidBody(chassisBody);

	m_vehicle = new btHinge2Vehicle();
	m_vehicle->setChassisBody(chassisBody);
	m_dynamicsWorld->addVehicle(m_vehicle);
	m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

	pos = m_vehicle->getChassisWorldTransform().getOrigin();
	prtmsg += "\tchassisPos: "+std::to_string(pos[0])+" "+std::to_string(pos[1])+" "+std::to_string(pos[2])+"\n";

	// add wheels
	// btVector3 wheelPosCS[4];
	// wheelPosCS[0] = btVector3(	-btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight),	 btScalar(chassisHalfLength-wheelRadius+.2)	)-cog; // front right
	// wheelPosCS[1] = btVector3(	 btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight),	 btScalar(chassisHalfLength-wheelRadius+.2)	)-cog; // front left
	// wheelPosCS[2] = btVector3(	 btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight),	-btScalar(chassisHalfLength-wheelRadius+.2)	)-cog; // back left
	// wheelPosCS[3] = btVector3(	-btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight),	-btScalar(chassisHalfLength-wheelRadius+.2)	)-cog; // back right

	btCollisionShape* wheelShape = new btCylinderShape(btVector3(wheelRadius, wheelHalfWidth, wheelRadius));
	// m_collisionShapes.push_back(wheelShape);

	btVector4 wheelColor(1,0,0,1);

	btScalar maxSteer = 30.f*B3_RADS_PER_DEG;
	btScalar maxLinVel = 10.f;
	btScalar maxAngVel = maxLinVel/(wheelRadius);
	btScalar maxTravel = 0.1f;
	btScalar friction = 1.f;
	btScalar stallTorque = 20.f;

	// front right
	{
		btVector3 wheelPosCS = btVector3(	btScalar(chassisHalfLength-wheelRadius+.2),		-btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	)-cog;
		btTransform wheelTranWS = m_vehicle->getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = btVehicle::createLocalRigidBody(wheelMass, wheelTranWS, wheelShape, wheelColor);
		m_dynamicsWorld->addRigidBody(wheelBody);
		btTypedConstraint* constraint = m_vehicle->addWheel2(wheelBody, maxSteer, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, friction, stallTorque);
		m_dynamicsWorld->addConstraint(constraint, true);
	}
	// front left
	{
		btVector3 wheelPosCS = btVector3(	btScalar(chassisHalfLength-wheelRadius+.2),		btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	)-cog; 
		btTransform wheelTranWS = m_vehicle->getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = btVehicle::createLocalRigidBody(wheelMass, wheelTranWS, wheelShape, wheelColor);
		m_dynamicsWorld->addRigidBody(wheelBody);
		btTypedConstraint* constraint = m_vehicle->addWheel2(wheelBody, maxSteer, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, friction, stallTorque);
		m_dynamicsWorld->addConstraint(constraint, true);
	}
	// back left
	{
		btVector3 wheelPosCS = btVector3(	-btScalar(chassisHalfLength-wheelRadius+.2),		btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	)-cog; 
		btTransform wheelTranWS = m_vehicle->getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = btVehicle::createLocalRigidBody(wheelMass, wheelTranWS, wheelShape, wheelColor);
		m_dynamicsWorld->addRigidBody(wheelBody);
		btTypedConstraint* constraint = m_vehicle->addWheel2(wheelBody, 0.f, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, friction, stallTorque);
		m_dynamicsWorld->addConstraint(constraint, true);
	}
	// back right
	{
		btVector3 wheelPosCS = btVector3(	-btScalar(chassisHalfLength-wheelRadius+.2),		-btScalar(chassisHalfWidth + wheelHalfWidth+.1),	btScalar(-chassisHalfHeight)	)-cog; 
		btTransform wheelTranWS = m_vehicle->getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS);
		btRigidBody* wheelBody = btVehicle::createLocalRigidBody(wheelMass, wheelTranWS, wheelShape, wheelColor);
		m_dynamicsWorld->addRigidBody(wheelBody);
		btTypedConstraint* constraint = m_vehicle->addWheel2(wheelBody, 0.f, maxAngVel, suspensionStiffness, suspensionDamping, maxTravel, friction, stallTorque);
		m_dynamicsWorld->addConstraint(constraint, true);
	}
	m_vehicle->updateConstraints();

	// for (int i = 0; i < 4; i++)
	// {
	// 	btTransform wheelTranWS = m_vehicle->getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS[i]);
	// 	btRigidBody* wheelBody = createRigidBody(wheelMass, wheelTranWS, wheelShape, wheelColor);

	// 	prtmsg += "Adding wheel "+std::to_string(i)+":\n";

	// 	btTypedConstraint* constraint = m_vehicle->addWheel(wheelBody);
	// 	// btTypedConstraint* constraint = m_vehicle->addWheel2(wheelBody, maxSteer, maxTorque, maxBrake, suspensionStiffness, suspensionDamping, maxTravel, friction, maxLinVelocity);
	// 	m_dynamicsWorld->addConstraint(constraint, true);

	// 	// m_vehicle->getWheel(i)->setDamping(0.f);
	// 	// m_vehicle->getWheel(i)->setStiffness(0.f);

	// 	// if (i<2) // front wheels
	// 	// {
	// 	// 	m_vehicle->getWheel(i)->setSteeringEnabled(true);
	// 	// }
	// 	// else
	// 	// {
	// 		// m_vehicle->getWheel(i)->setMotorEnabled(true);
	// 		// m_vehicle->getWheel(i)->setBrakeEnabled(true);
	// 	// }
		
	// 	prtmsg += m_vehicle->wheel2str(i,"","","\t","");
	// }
	// std::cout << prtmsg;
}


void Hinge2Vehicle::spawnCar()
{
	static float wheelRadius = 0.075f;
	static float wheelHalfWidth = 0.05f;
	static float wheelMass = 1.0f;

	static float suspensionStiffness = 700.f;
	static float suspensionDamping = 30.f;

	static float chassisHalfWidth = 0.1;
	static float chassisHalfHeight = 0.05;
	static float chassisHalfLength = 0.4;
	static float chassisMass = 10.f;

	spawnVehicle(btVector3(chassisHalfWidth, chassisHalfHeight, chassisHalfLength), chassisMass, wheelRadius, wheelHalfWidth, wheelMass, suspensionStiffness, suspensionDamping);
}

void Hinge2Vehicle::spawnCarPlannerCar()
{
	// static float wheelRadius = 0.04f;
	// static float wheelHalfWidth = 0.025f;
	// static float wheelMass = 0.01f;

	// static float suspensionStiffness = 120.f;
	// static float suspensionDamping = 10.f;

	// static float chassisHalfWidth = 0.21;
	// static float chassisHalfHeight = 0.03;
	// static float chassisHalfLength = 0.27+wheelRadius;
	// static float chassisMass = 0.473f;

	static float wheelRadius = 0.2f;
	static float wheelHalfWidth = 0.1f;
	static float wheelMass = 1.0f;

	static float suspensionStiffness = 200.f;
	static float suspensionDamping = 2.3f;

	static float chassisHalfWidth = 0.21;
	static float chassisHalfHeight = 0.03;
	static float chassisHalfLength = 0.37+wheelRadius;
	static float chassisMass = 20.473f;

	static float maxSteer = 30*B3_RADS_PER_DEG;
	static float maxLinVel = 5.;
	static float maxAngVel = maxLinVel/wheelRadius;
	static float maxTravel = 0.1f;
	static float wheelFriction = 1.f;
	static float stallTorque = 20.f;
	
	btVector3 initialPos(-3,0,5);
	btQuaternion initialRot(0,0,0,1);
	btTransform initialPose(initialRot,initialPos);

	// spawnVehicle(btVector3(chassisHalfWidth, chassisHalfHeight, chassisHalfLength), chassisMass, wheelRadius, wheelHalfWidth, wheelMass, suspensionStiffness, suspensionDamping);
	m_vehicle = new btDefaultHinge2Vehicle(chassisHalfWidth,
									 chassisHalfHeight,
									 chassisHalfLength,
									 chassisMass,
									 wheelRadius,
									 wheelHalfWidth,
									 wheelMass,
									 suspensionStiffness,
									 suspensionDamping,
									 maxSteer,
									 maxAngVel,
									 maxTravel,
									 wheelFriction,
									 stallTorque);
	m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);
	dynamic_cast<btDefaultHinge2Vehicle*>(m_vehicle)->spawn(m_dynamicsWorld, initialPose);
	// m_collisionShapes.push_back(m_vehicle->getChassisBody()->getCollisionShape());
	// m_collisionShapes.push_back(m_vehicle->getWheel(0)->getShape());
	// m_dynamicsWorld->addVehicle(m_vehicle);
}

void Hinge2Vehicle::spawnBuggie()
{
	static float wheelRadius = 0.2f;
	static float wheelHalfWidth = 0.2f;
	static float wheelMass = 5.0f;
	
	static float suspensionStiffness = 700.f;
	static float suspensionDamping = 20.f;
	
	static float chassisHalfWidth = 0.5;
	static float chassisHalfHeight = 0.25;
	static float chassisHalfLength = 1.;
	static float chassisMass = 80.f;

	spawnVehicle(btVector3(chassisHalfWidth, chassisHalfHeight, chassisHalfLength), chassisMass, wheelRadius, wheelHalfWidth, wheelMass, suspensionStiffness, suspensionDamping);
}

//to be implemented by the demo
void Hinge2Vehicle::renderScene()
{

	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

	resetCamera();

	m_guiHelper->render(m_dynamicsWorld);

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);
}

void Hinge2Vehicle::resetCamera()
{
	float dist = 6;
	float pitch = -32;
	float yaw = -90;
	float des_yaw = 0;
	float targetPos[3] = {2,0,0};

	btTransform target = m_vehicle->getChassisBody()->getWorldTransform();
	// target = target*btTransform(btQuaternion(0,0,1,0),btVector3());
	// target = target*btTransform(btQuaternion(target.getBasis().getColumn(0),-32*B3_RADS_PER_DEG),btVector3());
	targetPos[0] = target.getOrigin()[0];
	targetPos[1] = target.getOrigin()[1];
	targetPos[2] = target.getOrigin()[2];
	btScalar qy,qp,qr;
	target.getRotation().getEulerZYX(qy,qp,qr);
	yaw += qy*B3_DEGS_PER_RAD;

	printf("x %f y %f z %f r %f p %f y %f\n", targetPos[0], targetPos[1], targetPos[2], qr, qp, qy);

	// if (yaw > 90)
	// 	yaw = 180-yaw;
	// if (yaw < -90)
	// 	yaw = -180-yaw;
	yaw += des_yaw;
	// printf("yaw %f\n",yaw);
	m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
}

void Hinge2Vehicle::resetForklift()
{
	// gVehicleSteering = 0.f;
	// // gBreakingForce = defaultBreakingForce;
	// // gEngineForce = 0.f;

	// m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	// m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
	// m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_vehicle->getChassisBody()->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
}

// void Hinge2Vehicle::stepSimulation(float step)
// {
// 	if (m_dynamicsWorld)
// 	{
// 		//during idle mode, just run 1 simulation step maximum
// 		int maxSimSubSteps = 2;

// 		int numSimSteps;
// 		numSimSteps = m_dynamicsWorld->stepSimulation(step, maxSimSubSteps);

// 		if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
// 		{
// 			btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
// 			int numFallbacks = sol->getNumFallbacks();
// 			if (numFallbacks)
// 			{
// 				static int totalFailures = 0;
// 				totalFailures += numFallbacks;
// 				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
// 			}
// 			sol->setNumFallbacks(0);
// 		}

// //#define VERBOSE_FEEDBACK
// #ifdef VERBOSE_FEEDBACK
// 		if (!numSimSteps)
// 			printf("Interpolated transforms\n");
// 		else
// 		{
// 			if (numSimSteps > maxSimSubSteps)
// 			{
// 				//detect dropping frames
// 				printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
// 			}
// 			else
// 			{
// 				printf("Simulated (%i) steps\n", numSimSteps);
// 			}
// 		}
// #endif  //VERBOSE_FEEDBACK
// 	}
// }

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

bool Hinge2Vehicle::keyboardCallback(int key, int state)
{
	bool handled = false;
	bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);
	// printf("******* %d",key);

	if (state) // key down
	{
		{
			switch (key)
			{
				case 32:
					space = true;
					break;
				case 'j':
				case B3G_LEFT_ARROW:
				{
					steer_left = true;
					break;
				}
				case 'l':
				case B3G_RIGHT_ARROW:
				{
					steer_right = true;
					break;
				}
				case 'i':
				case B3G_UP_ARROW:
				{
					throttle_up = true;
					break;
				}
				case 'k':
				case B3G_DOWN_ARROW:
				{
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
					// m_useDefaultCamera = !m_useDefaultCamera;
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
				case 32:
					space = false;
					break;

				case 'j':
				case B3G_LEFT_ARROW:
				{
					steer_left = false;
					steer_right = false;
					break;
				}
				case 'l':
				case B3G_RIGHT_ARROW:
				{
					steer_left = false;
					steer_right = false;
					break;
				}
				case 'i':
				case B3G_UP_ARROW:
				{
					throttle_up = false;
					throttle_down = false;
					break;
				}
				case 'k':
				case B3G_DOWN_ARROW:
				{
					throttle_up = false;
					throttle_down = false;
					break;
				}
					
				default:
					break;
			}
		}
	}
	setTargets();

	return true;
	return handled;
}

void Hinge2Vehicle::setTargets()
{
	static btScalar vel_tau;
	float vel_max = 6.f;
	float vel_diff = 0.05f;

	if (throttle_up)
	{
		vel_tau += vel_diff*vel_max;
		vel_tau = std::min((float)vel_tau, vel_max);
	}
	else if (throttle_down)
	{
		vel_tau += -vel_diff*vel_max;
		vel_tau = std::max((float)vel_tau, -vel_max);
	}
	else //if (!throttle_down && !throttle_up)
	{
		vel_tau = 0.f;
	}
	// btScalar maxTargetVelocity = 4.f;
	// std::vector<float> vel_profile = {0, 0.5*maxTargetVelocity, maxTargetVelocity}; 
	// btScalar targetVelocity = (vel_tau<0 ? -1 : 1)*btBezier::calcBezier<float>(&(vel_profile[0]), vel_profile.size()-1, fabs(vel_tau));
	// for (uint i=0; i<m_vehicle->getNumWheels(); i++)
	// {
	// 	if (m_vehicle->getWheel(i)->isMotorEnabled()) // not necessary but good habit
	// 	{
	// 		btScalar currVel = m_vehicle->getWheel(i)->getRadialVelocity();
	// 		btScalar targetForce;
	// 		if (currVel > (targetVelocity/m_vehicle->getWheel(i)->getRadius()))
	// 			targetForce = 0.01;
	// 		else if (currVel < -(targetVelocity/m_vehicle->getWheel(i)->getRadius()))
	// 			targetForce = -0.01;
	// 		else
	// 		{
	// 			if (vel_tau>0.0)
	// 				targetForce = 100.f;
	// 			else if (vel_tau<0.0)
	// 				targetForce = -100.f;
	// 			else
	// 				targetForce = 0.0;
				
	// 		}
			
	// 		m_vehicle->getWheel(i)->setMotorTorque(targetForce);
	// 	}
	// }
	
	
	
	// btScalar targetForce;
	// if (0.5*(m_vehicle->getWheel > targetVelocity))
	// 	targetForce = -vel_tau*100.f;
	// else if (m_vehicle->getChassisForwardVelocity() < targetVelocity)
	// 	targetForce = vel_tau*100.f;

	m_vehicle->setEnabledLinearVelocity(vel_tau);

	btScalar targetSteering;
	if (steer_left)
	{
		targetSteering = m_vehicle->getEnabledSteeringAngle() - 0.025;
	}
	else if (steer_right)
	{
		targetSteering = m_vehicle->getEnabledSteeringAngle() + 0.025;
	}
	else
	{
		targetSteering = m_vehicle->getEnabledSteeringAngle() * 0.96;
	}
	m_vehicle->setEnabledSteeringAngle(targetSteering);

	// printf("Setting force %f steer %f\n", targetForce, targetSteering);

	// m_vehicle->applyForces();	
}

void Hinge2Vehicle::specialKeyboardUp(int key, int x, int y)
{
}

void Hinge2Vehicle::specialKeyboard(int key, int x, int y)
{
}

CommonExampleInterface* Hinge2VehicleCreateFunc(struct CommonExampleOptions& options)
{
	return new Hinge2Vehicle(options.m_guiHelper);
}


