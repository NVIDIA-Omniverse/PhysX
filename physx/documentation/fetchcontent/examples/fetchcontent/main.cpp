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

    // Cook a convex mesh at runtime. The cooking entry points (PxCookConvexMesh, ...)
    // are exported by PhysXCommon, so this also verifies that the linked PhysX
    // libraries expose them.
    const PxVec3 convexVertices[] = {
        PxVec3( 0.0f,  1.0f,  0.0f), PxVec3( 0.0f, -1.0f,  0.0f),
        PxVec3( 1.0f,  0.0f,  0.0f), PxVec3(-1.0f,  0.0f,  0.0f),
        PxVec3( 0.0f,  0.0f,  1.0f), PxVec3( 0.0f,  0.0f, -1.0f)
    };
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count  = sizeof(convexVertices) / sizeof(convexVertices[0]);
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data   = convexVertices;
    convexDesc.flags         = PxConvexFlag::eCOMPUTE_CONVEX;

    PxCookingParams cookingParams(tolerancesScale);
    PxDefaultMemoryOutputStream cookedData;
    PxConvexMesh* convexMesh = nullptr;
    if (PxCookConvexMesh(cookingParams, convexDesc, cookedData)) {
        PxDefaultMemoryInputData cookedInput(cookedData.getData(), cookedData.getSize());
        convexMesh = physics_sdk->createConvexMesh(cookedInput);
    }
    if (!convexMesh) {
        std::cerr << "Convex mesh cooking failed!" << std::endl;
        PX_RELEASE(scene);
        PX_RELEASE(physics_sdk);
        PX_RELEASE(dispatcher);
#if PX_SUPPORT_GPU_PHYSX
        PX_RELEASE(cudaContextManager);
#endif
        PX_RELEASE(foundation);
        return 1;
    }
    std::cout << "Cooked convex mesh: " << convexMesh->getNbVertices() << " vertices, "
              << convexMesh->getNbPolygons() << " polygons" << std::endl;

    // Create a dynamic box and a dynamic body using the cooked convex mesh to verify simulation works
    PxMaterial* material = physics_sdk->createMaterial(0.5f, 0.5f, 0.6f);
    PxShape* shape = physics_sdk->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material);
    PxRigidDynamic* body = physics_sdk->createRigidDynamic(PxTransform(PxVec3(0.0f, 10.0f, 0.0f)));
    body->attachShape(*shape);
    PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
    scene->addActor(*body);

    PxShape* convexShape = physics_sdk->createShape(PxConvexMeshGeometry(convexMesh), *material);
    PxRigidDynamic* convexBody = physics_sdk->createRigidDynamic(PxTransform(PxVec3(3.0f, 10.0f, 0.0f)));
    convexBody->attachShape(*convexShape);
    PxRigidBodyExt::updateMassAndInertia(*convexBody, 10.0f);
    scene->addActor(*convexBody);

    // Simulate a few steps and verify both bodies fall under gravity
    float initialY = body->getGlobalPose().p.y;
    float initialConvexY = convexBody->getGlobalPose().p.y;
    for (int i = 0; i < 10; i++) {
        scene->simulate(1.0f / 60.0f);
        scene->fetchResults(true);
    }
    float finalY = body->getGlobalPose().p.y;
    float finalConvexY = convexBody->getGlobalPose().p.y;
    std::cout << "Box position: y=" << initialY << " -> y=" << finalY << std::endl;
    std::cout << "Convex mesh position: y=" << initialConvexY << " -> y=" << finalConvexY << std::endl;

    int result = 0;
    if (finalY < initialY && finalConvexY < initialConvexY) {
        std::cout << "Simulation verified: gravity is working" << std::endl;
    } else {
        std::cerr << "Simulation error: bodies did not fall" << std::endl;
        result = 1;
    }

    // Release resources
    PX_RELEASE(material);
    PX_RELEASE(shape);
    PX_RELEASE(convexShape);
    PX_RELEASE(convexMesh);
    PX_RELEASE(scene);
    PX_RELEASE(dispatcher);
    PX_RELEASE(physics_sdk);
#if PX_SUPPORT_GPU_PHYSX
    PX_RELEASE(cudaContextManager);
#endif
    PX_RELEASE(foundation);

    std::cout << "Shutting down..." << std::endl;
    return result;
}
