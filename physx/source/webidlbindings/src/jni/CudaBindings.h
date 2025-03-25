#ifndef CUDA_BINDINGS_H
#define CUDA_BINDINGS_H

// CUDA support is always disabled since PhysX 5.6.0 because apparently this now requires an
// extra CUDA toolkit which is annoying to set up (especially in GitHub workflow)
#if FALSE
//#if !(defined(__EMSCRIPTEN__) || defined(__APPLE__) || defined(__ANDROID__))

#include "cudamanager/PxCudaContext.h"
#include "extensions/PxParticleExt.h"
#include "extensions/PxParticleClothCooker.h"
#include "PxPhysicsAPI.h"
#include "PxParticleGpu.h"

struct PxCudaTopLevelFunctions {

    static int GetSuggestedCudaDeviceOrdinal(physx::PxFoundation& foundation) {
        return PxGetSuggestedCudaDeviceOrdinal(foundation.getErrorCallback());
    }

    static physx::PxCudaContextManager* CreateCudaContextManager(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc) {
        return PxCreateCudaContextManager(foundation, desc);
    }

    // cloth particle functions

    static physx::PxParticleClothPreProcessor* CreateParticleClothPreProcessor(physx::PxCudaContextManager* cudaContextManager) {
        return PxCreateParticleClothPreProcessor(cudaContextManager);
    }

    static physx::ExtGpu::PxParticleClothBufferHelper* CreateParticleClothBufferHelper(
        const physx::PxU32 maxCloths,
        const physx::PxU32 maxTriangles,
        const physx::PxU32 maxSprings,
        const physx::PxU32 maxParticles,
        physx::PxCudaContextManager *cudaContextManager
    ) {
        return physx::ExtGpu::PxCreateParticleClothBufferHelper(maxCloths, maxTriangles, maxSprings, maxParticles, cudaContextManager);
    }

    static physx::ExtGpu::PxParticleClothCooker* CreateParticleClothCooker(
        physx::PxU32 vertexCount, physx::PxVec4 *inVertices,
        physx::PxU32 triangleIndexCount, physx::PxU32 *inTriangleIndices,
        physx::PxU32 constraintTypeFlags = physx::ExtGpu::PxParticleClothConstraint::eTYPE_ALL,
        physx::PxVec3 verticalDirection = physx::PxVec3(0.0f, 1.0f, 0.0f),
        physx::PxReal bendingConstraintMaxAngle = 20.0f / 360.0f * physx::PxTwoPi
    ) {
        return physx::PxCreateParticleClothCooker(vertexCount, inVertices, triangleIndexCount, inTriangleIndices, constraintTypeFlags, verticalDirection, bendingConstraintMaxAngle);
    }

    static physx::PxParticleClothBuffer* CreateAndPopulateParticleClothBuffer(
        const physx::ExtGpu::PxParticleBufferDesc &desc,
        const physx::PxParticleClothDesc &clothDesc,
        physx::PxPartitionedParticleCloth &output,
        physx::PxCudaContextManager *cudaContextManager
    ) {
        return physx::ExtGpu::PxCreateAndPopulateParticleClothBuffer(desc, clothDesc, output, cudaContextManager);
    }

    // fluid particle functions

    static physx::PxParticleBuffer* CreateAndPopulateParticleBuffer(const physx::ExtGpu::PxParticleBufferDesc &desc, physx::PxCudaContextManager *cudaContextManager) {
        return physx::ExtGpu::PxCreateAndPopulateParticleBuffer(desc, cudaContextManager);
    }

    static physx::PxParticleAndDiffuseBuffer* CreateAndPopulateParticleAndDiffuseBuffer(const physx::ExtGpu::PxParticleAndDiffuseBufferDesc &desc, physx::PxCudaContextManager *cudaContextManager) {
        return physx::ExtGpu::PxCreateAndPopulateParticleAndDiffuseBuffer(desc, cudaContextManager);
    }

    // memory util functions
    
    static physx::PxU32* allocPinnedHostBufferPxU32(physx::PxCudaContextManager *cudaContextManager, physx::PxU32 numElements) {
        return cudaContextManager->allocPinnedHostBuffer<physx::PxU32>(numElements);
    }

    static physx::PxVec4* allocPinnedHostBufferPxVec4(physx::PxCudaContextManager *cudaContextManager, physx::PxU32 numElements) {
        return cudaContextManager->allocPinnedHostBuffer<physx::PxVec4>(numElements);
    }

    static void freePinnedHostBufferPxU32(physx::PxCudaContextManager* cudaContextManager, physx::PxU32* buffer) {
        cudaContextManager->freePinnedHostBuffer(buffer);
    }

    static void freePinnedHostBufferPxVec4(physx::PxCudaContextManager* cudaContextManager, physx::PxVec4* buffer) {
        cudaContextManager->freePinnedHostBuffer(buffer);
    }
    
    static physx::PxU64 pxU32deviceptr(void* pxU32data) { return CUdeviceptr(pxU32data); }

    static physx::PxU64 pxVec4deviceptr(physx::PxVec4* pxVec4data) { return CUdeviceptr(pxVec4data); }
};

typedef physx::PxParticleBufferFlag::Enum PxParticleBufferFlagEnum;
typedef physx::PxParticleFlag::Enum PxParticleFlagEnum;
typedef physx::PxParticlePhaseFlag::Enum PxParticlePhaseFlagEnum;
typedef physx::PxParticleSolverType::Enum PxParticleSolverTypeEnum;
typedef physx::ExtGpu::PxParticleClothConstraint PxParticleClothConstraintEnum;

typedef physx::PxGpuMirroredPointer<physx::PxGpuParticleSystem> PxGpuMirroredGpuParticleSystemPointer;

typedef PxArrayExt<physx::PxParticleRigidFilterPair> PxArray_PxParticleRigidFilterPair;
typedef PxArrayExt<physx::PxParticleSpring> PxArray_PxParticleSpring;

// deprecated std::vector style types
typedef PxArrayExt<physx::PxParticleRigidFilterPair> Vector_PxParticleRigidFilterPair;
typedef PxArrayExt<physx::PxParticleSpring> Vector_PxParticleSpring;

#endif

#endif