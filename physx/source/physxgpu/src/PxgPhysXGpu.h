// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_PHYSX_GPU_H
#define PXG_PHYSX_GPU_H

#include "geometry/PxGeometry.h" //just for PxGeometryType::Enum
#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "PxgHeapMemAllocator.h"
#include "PxsTransformCache.h"
#include "PxPhysXGpu.h"

#include "CmIDPool.h"
#include "CudaKernelWrangler.h"

namespace physx
{

struct PxvSimStats;
class PxgCudaKernelWranglerManager;

class PxgPhysXGpu : public PxPhysXGpu, public PxUserAllocated
{
public:
	//PxPhysXGpu implementation
	virtual	void									release()	PX_OVERRIDE;

	virtual PxsParticleBuffer*						createParticleBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
	virtual PxsParticleAndDiffuseBuffer*			createParticleAndDiffuseBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
	virtual PxsParticleClothBuffer*					createParticleClothBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
	virtual PxsParticleRigidBuffer*					createParticleRigidBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumRigids, PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;

	virtual PxsMemoryManager*						createGpuMemoryManager(PxCudaContextManager* cudaContextManager)	PX_OVERRIDE;
	
	virtual PxsHeapMemoryAllocatorManager*			createGpuHeapMemoryAllocatorManager(const PxU32 heapCapacity,
														PxsMemoryManager* memoryManager,
														const PxU32 gpuComputeVersion)	PX_OVERRIDE;

	virtual PxsKernelWranglerManager*				getGpuKernelWranglerManager(PxCudaContextManager* cudaContextManager)	PX_OVERRIDE;

	virtual Bp::BroadPhase*							createGpuBroadPhase(PxsKernelWranglerManager* gpuKernelWrangler,
														PxCudaContextManager* cudaContextManager,
														const PxU32 gpuComputeVersion,
														const PxGpuDynamicsMemoryConfig& config,
														PxsHeapMemoryAllocatorManager* heapMemoryManager, PxU64 contextID)	PX_OVERRIDE;

	virtual Bp::AABBManagerBase*					createGpuAABBManager(PxsKernelWranglerManager* gpuKernelWrangler,
														PxCudaContextManager* cudaContextManager,
														const PxU32 gpuComputeVersion,
														const PxGpuDynamicsMemoryConfig& config,
														PxsHeapMemoryAllocatorManager* heapMemoryManager,
														Bp::BroadPhase& bp,
														Bp::BoundsArray& boundsArray,
														PxFloatArrayPinned& contactDistance,
														PxU32 maxNbAggregates, PxU32 maxNbShapes,
														PxVirtualAllocator& allocator,
														PxU64 contextID,
														PxPairFilteringMode::Enum kineKineFilteringMode,
														PxPairFilteringMode::Enum staticKineFilteringMode)	PX_OVERRIDE;

	virtual Bp::BoundsArray* 						createGpuBounds(PxVirtualAllocator& allocator)  PX_OVERRIDE;

	virtual PxvNphaseImplementationContext*			createGpuNphaseImplementationContext(PxsContext& context,
														PxsKernelWranglerManager* gpuKernelWrangler,
														PxvNphaseImplementationFallback* fallbackForUnsupportedCMs,
														const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase,
														void* patchStreamBase, void* forceAndIndiceStreamBase,
														PxBoundsArrayPinned& bounds, IG::IslandSim* islandSim,
														physx::Dy::Context* dynamicsContext, const PxU32 gpuComputeVersion,
														PxsHeapMemoryAllocatorManager* heapMemoryManager, bool useGPUBP)	PX_OVERRIDE;
	virtual PxsSimulationController*				createGpuSimulationController(PxsKernelWranglerManager* gpuWranglerManagers, 
														PxCudaContextManager* cudaContextManager,
														Dy::Context* dynamicContext, PxvNphaseImplementationContext* npContext, Bp::BroadPhase* bp, 
														bool useGpuBroadphase,
														PxsSimulationControllerCallback* callback, PxU32 gpuComputeVersion,
														PxsHeapMemoryAllocatorManager* heapMemoryManager, PxU32 maxSoftBodyContacts,
														PxU32 maxFemClothContacts, PxU32 maxParticleContacts,
														PxU32 collisionStackSizeBytes, bool enableBodyAccelerations)	PX_OVERRIDE;
	virtual Dy::Context*							createGpuDynamicsContext(Cm::FlushPool& taskPool, PxsKernelWranglerManager* gpuKernelWragler, 
														PxCudaContextManager* cudaContextManager,
														const PxGpuDynamicsMemoryConfig& config, IG::SimpleIslandManager& islandManager, const PxU32 maxNumPartitions, const PxU32 maxNumStaticPartitions,
														const bool enableStabilization, const bool useEnhancedDeterminism, const PxReal maxBiasCoefficient,
														const PxU32 gpuComputeVersion, PxvSimStats& simStats, PxsHeapMemoryAllocatorManager* heapMemoryManager,
														const bool frictionEveryIteration, const bool externalForcesEveryTgsIterationEnabled, PxSolverType::Enum solverType,
														const PxReal lengthScale, bool enableDirectGPUAPI, PxU64 contextID, bool isResidualReportingEnabled) PX_OVERRIDE;
	
	//internals
		
	static PxgPhysXGpu&								getInstance();

private:
	PxgPhysXGpu()			{}
	virtual ~PxgPhysXGpu()	{}

	static PxgPhysXGpu* sInstance;
	typedef PxHashMap<PxCudaContextManager*, PxgCudaKernelWranglerManager*> KernelWranglerMap;
	KernelWranglerMap mKernelWranglerInstances;
};

}

void PxgSetPhysXGpuDelayLoadHook(const physx::PxDelayLoadHook* hook);

#endif // PXG_PHYSX_GPU_H
