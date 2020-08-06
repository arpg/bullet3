#include "btWheel.h"

void btWheel::castRay(btVehicleRaycaster* vehicleRaycaster, btVehicleRaycaster::btVehicleRaycasterResult& rayResults)
{
	btAssert(vehicleRaycaster);

	const btVector3& source = getWorldTransform().getOrigin();
	btScalar raylen = getRadius();
    // raylen += 0.5;
    // raylen *= 1.1;
    const btVector3& raydir = -getWheelDirectionWS();
	const btVector3& rayvector = raydir * raylen;
	const btVector3& target = source + rayvector;
	// btVehicleRaycaster::btVehicleRaycasterResult rayResults;

    // void* object = vehicleRaycaster->castRay(source, target, rayResults);
    vehicleRaycaster->castRay(source, target, rayResults);
    std::cout << "***castwheel " 
        << "\n " << std::to_string(rayResults.m_hitPointInWorld[0]) 
        << " " << std::to_string(rayResults.m_hitPointInWorld[1]) 
        << " " << std::to_string(rayResults.m_hitPointInWorld[2])
        << "\n " << std::to_string(source[0])  
        << " " << std::to_string(source[1])  
        << " " << std::to_string(source[2])  
        << "\n " << std::to_string(target[0])  
        << " " << std::to_string(target[1])  
        << " " << std::to_string(target[2])  
        // << " " << std::to_string(rayResults.m_distFraction) 
        << std::endl;

    return;
}

bool btWheel::isInContact(btVehicleRaycaster* vehicleRaycaster)
{
    btVehicleRaycaster::btVehicleRaycasterResult rayResults;
    castRay(vehicleRaycaster, rayResults);    
    // std::cout << "***castwheel " << std::to_string(rayResults.m_hitPointInWorld[0]) << " " << std::to_string(rayResults.m_hitPointInWorld[1]) << " " << std::to_string(rayResults.m_hitPointInWorld[2]) << " " << std::to_string(rayResults.m_distFraction) << std::endl;

    if (rayResults.m_distFraction!=-1.f)
    {
        return true;
    }
    return false;
}

// bool btWheel::isInContact(btDynamicsWorld* world)
// {
//     world->
//     if (rayResults.m_distFraction!=-1.f)
//     {
//         return true;
//     }
//     return false;
// }