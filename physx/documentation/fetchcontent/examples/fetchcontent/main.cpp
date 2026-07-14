#include "PxConfig.h"
#include "PxPhysicsAPI.h"
#include <iostream>

using namespace physx;

int main() {
    std::cout << "Starting PhysX up..." << std::endl;
    PxDefaultAllocator allocator;
    PxDefaultErrorCallback error_callback;
    auto foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, error_callback);
    if (!foundation) {
        std::cerr << "PxCreateFoundation failed!" << std::endl;
        return 1;
    }

    PxCudaContextManager* cudaContextManager = nullptr;

#if PX_SUPPORT_GPU_PHYSX
    PxCudaContextManagerDesc cudaContextManagerDesc;
    cudaContextManager = PxCreateCudaContextManager(*foundation, cudaContextManagerDesc, PxGetProfilerCallback());

    if(cudaContextManager && cudaContextManager->contextIsValid()) {
        std::cout << "GPU acceleration is available" << std::endl;
    } else {
        std::cout << "GPU acceleration is not available" << std::endl;
    }
#else
    std::cout << "Built without GPU support (CPU-only mode)" << std::endl;
#endif

    PxTolerancesScale tolerancesScale;
    PxSceneDesc sceneDesc(tolerancesScale);
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

    auto dispatcher = PxDefaultCpuDispatcherCreate(4);
    if (!dispatcher) {
        std::cerr << "PxDefaultCpuDispatcherCreate failed!" << std::endl;
#if PX_SUPPORT_GPU_PHYSX
        PX_RELEASE(cudaContextManager);
#endif
        PX_RELEASE(foundation);
        return 1;
    }
    sceneDesc.cpuDispatcher = dispatcher;
    sceneDesc.filterShader  = PxDefaultSimulationFilterShader;

#if PX_SUPPORT_GPU_PHYSX
    if(cudaContextManager && cudaContextManager->contextIsValid()) {
        sceneDesc.cudaContextManager = cudaContextManager;
        sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
        sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
    }
#endif

    auto physics_sdk = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolerancesScale, true, nullptr);
    if (!physics_sdk) {
        std::cerr << "PxCreatePhysics failed!" << std::endl;
#if PX_SUPPORT_GPU_PHYSX
        PX_RELEASE(cudaContextManager);
#endif
        PX_RELEASE(dispatcher);
        PX_RELEASE(foundation);
        return 1;
    }

    auto scene = physics_sdk->createScene(sceneDesc);
    if (!scene) {
        std::cerr << "createScene failed!" << std::endl;
#if PX_SUPPORT_GPU_PHYSX
        PX_RELEASE(cudaContextManager);
#endif
        PX_RELEASE(physics_sdk);
        PX_RELEASE(dispatcher);
        PX_RELEASE(foundation);
        return 1;
    }

    std::cout << "PhysX set up" << std::endl;

    // Create a simple dynamic box to verify simulation works
    PxMaterial* material = physics_sdk->createMaterial(0.5f, 0.5f, 0.6f);
    PxShape* shape = physics_sdk->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material);
    PxRigidDynamic* body = physics_sdk->createRigidDynamic(PxTransform(PxVec3(0.0f, 10.0f, 0.0f)));
    body->attachShape(*shape);
    PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
    scene->addActor(*body);

    // Simulate a few steps and verify the box falls under gravity
    float initialY = body->getGlobalPose().p.y;
    for (int i = 0; i < 10; i++) {
        scene->simulate(1.0f / 60.0f);
        scene->fetchResults(true);
    }
    float finalY = body->getGlobalPose().p.y;
    std::cout << "Box position: y=" << initialY << " -> y=" << finalY << std::endl;

    if (finalY < initialY) {
        std::cout << "Simulation verified: gravity is working" << std::endl;
    } else {
        std::cerr << "Simulation error: box did not fall" << std::endl;
    }

    // Release resources
    PX_RELEASE(material);
    PX_RELEASE(shape);
    PX_RELEASE(scene);
    PX_RELEASE(dispatcher);
    PX_RELEASE(physics_sdk);
#if PX_SUPPORT_GPU_PHYSX
    PX_RELEASE(cudaContextManager);
#endif
    PX_RELEASE(foundation);

    std::cout << "Shutting down..." << std::endl;
    return 0;
}
