//**********************************************************************************
// STEAMiE's Entry Point.
//**********************************************************************************

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "GLViewSpeedRacer.h" //GLView subclass instantiated to drive this simulation
#include "PxPhysicsAPI.h"

/**
   This creates a GLView subclass instance and begins the GLView's main loop.
   Each iteration of this loop occurs when a reset request is received. A reset
   request causes the entire GLView to be destroyed (since its exits scope) and
   begin again (simStatus == -1). This loop exits when a request to exit the
   application is received (simStatus == 0 ).
*/
int main(int argc, char* argv[])
{
    std::vector< std::string > args{ argv, argv + argc }; ///< Command line arguments passed via argc and argv, reserved to size of argc
    int simStatus = 0;

    physx::PxDefaultAllocator pxAllocator;
    physx::PxDefaultErrorCallback pxError;
    physx::PxFoundation* pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, pxAllocator, pxError);

    physx::PxPvd* gPvd = physx::PxCreatePvd(*pxFoundation);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10000);
    gPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);

    physx::PxPhysics* pxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *pxFoundation, physx::PxTolerancesScale(), true, gPvd);

    physx::PxSceneDesc pxSceneDesc(pxPhysics->getTolerancesScale());
    pxSceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
    pxSceneDesc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    pxSceneDesc.flags = pxSceneDesc.flags | physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS;
    pxSceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, -9.8f);

    physx::PxScene* pxScene = pxPhysics->createScene(pxSceneDesc);
    pxScene->setFlag(physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS, true);

    do
    {
        std::unique_ptr< Aftr::GLViewSpeedRacer > glView(Aftr::GLViewSpeedRacer::New(args, pxPhysics, pxScene));
        simStatus = glView->startWorldSimulationLoop(); // Runs until simulation exits or requests a restart (values 0 or -1, respectively)
    } while (simStatus != 0);

    std::cout << "Exited AftrBurner Engine Normally..." << std::endl;
    return 0;
}

