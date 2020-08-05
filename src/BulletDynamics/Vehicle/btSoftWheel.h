#ifndef BT_SOFTWHEEL_H
#define BT_SOFTWHEEL_H

#include <string>
#include <iostream>

#include "BulletDynamics/Vehicle/btWheel.h"

// #include "BulletSoftBody/btSoftBodyHelpers.h"
// #include "BulletSoftBody/btSoftBody.h"
// #include "TorusMesh.h"

class btSoftWheel : public btWheel
{
protected:
public:
    btSoftWheel(const btVector3& chassisConnectionCS, const btVector3& wheelDirectionCS, const btVector3& wheelAxleCS, btCollisionObject* object) :
        btWheel(chassisConnectionCS, wheelDirectionCS, wheelAxleCS, object)
    {
    }

    ~btSoftWheel()
    {
        delete m_collisionObject;
    }

    static btSoftBody* initSoftTorus(btSoftBodyWorldInfo m_softBodyWorldInfo, btVector3 pos, btVector3 a);
    static void spawnSoftTorus();
private:

};

// btSoftBody* btSoftWheel::initSoftTorus(btSoftBodyWorldInfo m_softBodyWorldInfo, btVector3 pos, btVector3 a)
// {
//     btSoftBody* softWheel = btSoftBodyHelpers::CreateFromTriMesh(m_softBodyWorldInfo, gVertices, &gIndices[0][0], NUM_TRIANGLES);
//     btSoftBody::Material* pm = softWheel->appendMaterial();
//     pm->m_kLST = 0.1;
//     pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
// 	softWheel->generateBendingConstraints(2, pm);
// 	softWheel->m_cfg.piterations = 2;
// 	softWheel->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;
// 	softWheel->m_cfg.kDF = 1;
// 	softWheel->randomizeConstraints();
//     btVector3 s(0.1, 0.3, 0.1);
// 	softWheel->scale(s);
// 	softWheel->rotate(btQuaternion(a[0], a[1], a[2]));
// 	softWheel->translate(pos);
// 	softWheel->setTotalMass(2, true);
// 	softWheel->generateClusters(64);
// 	softWheel->setPose(false, true);
	
//     return softWheel;
// }

// void btSoftWheel::spawnSoftTorus()
// {

// }


#endif // BT_SOFT_WHEEL_H