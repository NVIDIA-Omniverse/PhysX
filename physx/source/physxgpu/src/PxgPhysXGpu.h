// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_PHYSX_GPU_H
#define PXG_PHYSX_GPU_H

#include "geometry/PxGeometry.h" //just for PxGeometryType::Enum
#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
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

	virtual PxsParticleBuffer*						createParticleBuffer(PxU32 maxNumParticles, PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
	virtual PxsParticleAndDiffuseBuffer*			createParticleAndDiffuseBuffer(PxU32 maxNumParticles, PxU32 maxDiffuseParticles, PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;

	virtual PxsMemoryManager*						createGpuMemoryManager(PxCudaContextManager* cudaContextManager)	PX_OVERRIDE;
	
	virtual PxsHeapMemoryAllocatorManager*			createGpuHeapMemoryAllocatorManager(const PxU32 heapCapacity,
														PxsMemoryManager* memoryManager,
														const PxU32 gpuComputeVersion)	PX_OVERRIDE;

	virtual PxsKernelWranglerManager*				getGpuKernelWranglerManager(PxCudaContextManager* cudaContextManager)	PX_OVERRIDE;

	virtual Bp::BroadPhase*							createGpuBroadPhase(const PxGpuBroadPhaseDesc& desc,
														PxsKernelWranglerManager* gpuKernelWrangler,
														PxCudaContextManager* cudaContextManager,
														PxU32 gpuComputeVersion,
														const PxGpuDynamicsMemoryConfig& config,
														PxsHeapMemoryAllocatorManager& heapMemoryManager, PxU64 contextID)	PX_OVERRIDE;

	virtual Bp::AABBManagerBase*					createGpuAABBManager(PxsKernelWranglerManager* gpuKernelWrangler,
														PxCudaContextManager* cudaContextManager,
														const PxU32 gpuComputeVersion,
														const PxGpuDynamicsMemoryConfig& config,
														PxsHeapMemoryAllocatorManager& heapMemoryManager,
														Bp::BroadPhase& bp,
														Bp::BoundsArray& boundsArray,
														Cm::PinnableArray<PxReal>& contactDistance,
														PxU32 maxNbAggregates, PxU32 maxNbShapes,
														PxU64 contextID,
														PxPairFilteringMode::Enum kineKineFilteringMode,
														PxPairFilteringMode::Enum staticKineFilteringMode)	PX_OVERRIDE;

	virtual Bp::BoundsArray* 						createGpuBounds(Cm::VirtualAllocatorCallback& allocator)  PX_OVERRIDE;

	virtual PxvNphaseImplementationContext*			createGpuNphaseImplementationContext(PxsContext& context,
														PxsKernelWranglerManager* gpuKernelWrangler,
														PxvNphaseImplementationFallback* fallbackForUnsupportedCMs,
														const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase,
														void* patchStreamBase, void* forceAndIndiceStreamBase,
														Bp::BoundsArray& bounds, IG::IslandSim* islandSim,
														physx::Dy::Context* dynamicsContext, const PxU32 gpuComputeVersion,
														PxsHeapMemoryAllocatorManager& heapMemoryManager, bool useGPUBP)	PX_OVERRIDE;

	virtual PxsSimulationController*				createGpuSimulationController(PxsKernelWranglerManager* gpuWranglerManagers, 
														PxCudaContextManager* cudaContextManager,
														Dy::Context* dynamicContext, PxvNphaseImplementationContext* npContext, Bp::BroadPhase* bp, 
														bool useGpuBroadphase,
														PxsSimulationControllerCallback* callback, PxU32 gpuComputeVersion,
														PxsHeapMemoryAllocatorManager& heapMemoryManager, PxU32 maxDeformableVolumeContacts,
														PxU32 maxDeformableSurfaceContacts, PxU32 maxParticleContacts,
														PxU32 collisionStackSizeBytes, bool enableBodyAccelerations)	PX_OVERRIDE;

	virtual Dy::Context*							createGpuDynamicsContext(Cm::FlushPool& taskPool, PxsKernelWranglerManager* gpuKernelWragler, 
														PxCudaContextManager* cudaContextManager,
														const PxGpuDynamicsMemoryConfig& config, IG::SimpleIslandManager& islandManager, PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions, PxReal maxBiasCoefficient,
														PxU32 gpuComputeVersion, PxvSimStats& simStats, PxsHeapMemoryAllocatorManager& heapMemoryManager, PxSolverType::Enum solverType,
														PxReal lengthScale, PxU64 contextID, PxSceneFlags sceneFlags) PX_OVERRIDE;
	
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
