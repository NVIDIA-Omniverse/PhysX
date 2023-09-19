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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_PHYSX_GPU_H
#define PX_PHYSX_GPU_H

#include "task/PxTask.h"

#include "foundation/PxPinnedArray.h"
#include "common/PxPhysXCommonConfig.h"
#include "PxSceneDesc.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxSparseGridParams.h"

namespace physx
{

class PxFoundation;
class PxCudaContextManagerDesc;
class PxvNphaseImplementationContext;
class PxsContext;
class PxsKernelWranglerManager;
class PxvNphaseImplementationFallback;
struct PxgDynamicsMemoryConfig;
class PxsMemoryManager;
class PxsHeapMemoryAllocatorManager;
class PxsSimulationController;
class PxsSimulationControllerCallback;
class PxDelayLoadHook;
class PxParticleBuffer;
class PxParticleAndDiffuseBuffer;
class PxParticleClothBuffer;
class PxParticleRigidBuffer;

class PxIsosurfaceExtractor;
class PxSparseGridIsosurfaceExtractor;
struct PxIsosurfaceParams;

class PxAnisotropyGenerator;
class PxSmoothedPositionGenerator;
class PxParticleNeighborhoodProvider;
class PxPhysicsGpu;

struct PxvSimStats;

namespace Bp
{
	class BroadPhase;
	class AABBManagerBase;
	class BoundsArray;
}

namespace Dy
{
	class Context;
}

namespace IG
{
	class IslandSim;
	class SimpleIslandManager;
}

namespace Cm
{
	class FlushPool;
}

/**
\brief Interface to create and run CUDA enabled PhysX features.

The methods of this interface are expected not to be called concurrently. 
Also they are expected to not be called concurrently with any tasks spawned before the end pipeline ... TODO make clear.
*/
class PxPhysXGpu
{
protected:
	virtual								~PxPhysXGpu()	{}
										PxPhysXGpu()	{}
public:

	/**
	\brief Closes this instance of the interface.
	*/
	virtual		void					release() = 0;

	virtual PxParticleBuffer* createParticleBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxCudaContextManager* cudaContextManager, PxU64* memStat, void (*onParticleBufferRelease)(PxParticleBuffer* buffer)) = 0;
	virtual PxParticleClothBuffer* createParticleClothBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager* cudaContextManager, PxU64* memStat, void (*onParticleBufferRelease)(PxParticleBuffer* buffer)) = 0;
	virtual PxParticleRigidBuffer* createParticleRigidBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumRigids, PxCudaContextManager* cudaContextManager, PxU64* memStat, void (*onParticleBufferRelease)(PxParticleBuffer* buffer)) = 0;
	virtual PxParticleAndDiffuseBuffer* createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager* cudaContextManager, PxU64* memStat, void (*onParticleBufferRelease)(PxParticleBuffer* buffer)) = 0;
	
	/**
	Create GPU memory manager.
	*/
	virtual PxsMemoryManager* createGpuMemoryManager(PxCudaContextManager* cudaContextManager) = 0;

	virtual PxsHeapMemoryAllocatorManager* createGpuHeapMemoryAllocatorManager(
		const PxU32 heapCapacity, 
		PxsMemoryManager* memoryManager,
		const PxU32 gpuComputeVersion) = 0;

	/** 
	Create GPU kernel wrangler manager. If a kernel wrangler manager already exists, then that one will be returned.
	The kernel wrangler manager should not be deleted. It will automatically be deleted when the PxPhysXGpu singleton gets released.
	*/
	virtual PxsKernelWranglerManager* getGpuKernelWranglerManager(
		PxCudaContextManager* cudaContextManager) = 0;

	/**
	Create GPU broadphase.
	*/
	virtual Bp::BroadPhase* createGpuBroadPhase(
		PxsKernelWranglerManager* gpuKernelWrangler,
		PxCudaContextManager* cudaContextManager,
		const PxU32 gpuComputeVersion,
		const PxgDynamicsMemoryConfig& config,
		PxsHeapMemoryAllocatorManager* heapMemoryManager, PxU64 contextID) = 0;


	/**
	Create GPU aabb manager.
	*/
	virtual Bp::AABBManagerBase* createGpuAABBManager(
		PxsKernelWranglerManager* gpuKernelWrangler,
		PxCudaContextManager* cudaContextManager,
		const PxU32 gpuComputeVersion,
		const PxgDynamicsMemoryConfig& config,
		PxsHeapMemoryAllocatorManager* heapMemoryManager,
		Bp::BroadPhase& bp, 
		Bp::BoundsArray& boundsArray, 
		PxFloatArrayPinned& contactDistance,
		PxU32 maxNbAggregates, PxU32 maxNbShapes,
		PxVirtualAllocator& allocator, 
		PxU64 contextID,
		PxPairFilteringMode::Enum kineKineFilteringMode, 
		PxPairFilteringMode::Enum staticKineFilteringMode) = 0;

	/**
	Create GPU narrow phase context.
	*/
	virtual PxvNphaseImplementationContext* createGpuNphaseImplementationContext(PxsContext& context,
		PxsKernelWranglerManager* gpuKernelWrangler,
		PxvNphaseImplementationFallback* fallbackForUnsupportedCMs,
		const PxgDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase, void* patchStreamBase, void* forceAndIndiceStreamBase,
		PxBoundsArrayPinned& bounds, IG::IslandSim* islandSim,
		physx::Dy::Context* dynamicsContext, const PxU32 gpuComputeVersion, PxsHeapMemoryAllocatorManager* heapMemoryManager,
		bool useGpuBP) = 0;

	/**
	Create GPU simulation controller.
	*/
	virtual PxsSimulationController* createGpuSimulationController(PxsKernelWranglerManager* gpuWranglerManagers, 
		PxCudaContextManager* cudaContextManager,
		Dy::Context* dynamicContext, PxvNphaseImplementationContext* npContext, Bp::BroadPhase* bp, 
		const bool useGpuBroadphase, IG::SimpleIslandManager* simpleIslandSim,
		PxsSimulationControllerCallback* callback, const PxU32 gpuComputeVersion, PxsHeapMemoryAllocatorManager* heapMemoryManager,
		const PxU32 maxSoftBodyContacts, const PxU32 maxFemClothContacts, const PxU32 maxParticleContacts, const PxU32 maxHairContacts) = 0;

	/**
	Create GPU dynamics context.
	*/
	virtual Dy::Context* createGpuDynamicsContext(Cm::FlushPool& taskPool, PxsKernelWranglerManager* gpuKernelWragler, 
		PxCudaContextManager* cudaContextManager, 
		const PxgDynamicsMemoryConfig& config, IG::SimpleIslandManager* islandManager, const PxU32 maxNumPartitions, const PxU32 maxNumStaticPartitions,
		const bool enableStabilization, const bool useEnhancedDeterminism, const PxReal maxBiasCoefficient,
		const PxU32 gpuComputeVersion, PxvSimStats& simStats, PxsHeapMemoryAllocatorManager* heapMemoryManager,
		const bool frictionEveryIteration, PxSolverType::Enum solverType, const PxReal lengthScale, bool enableDirectGPUAPI) = 0;
};

}

/**
Create PxPhysXGpu interface class.
*/
PX_C_EXPORT PX_PHYSX_GPU_API physx::PxPhysXGpu* PX_CALL_CONV PxCreatePhysXGpu();

/**
Create a cuda context manager. Set launchSynchronous to true for Cuda to report the actual point of failure.
*/
PX_C_EXPORT PX_PHYSX_GPU_API physx::PxCudaContextManager* PX_CALL_CONV PxCreateCudaContextManager(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc, physx::PxProfilerCallback* profilerCallback = NULL, bool launchSynchronous = false);

/**
Set profiler callback.
*/
PX_C_EXPORT PX_PHYSX_GPU_API void PX_CALL_CONV PxSetPhysXGpuProfilerCallback(physx::PxProfilerCallback* profilerCallback);

/**
Query the device ordinal - depends on control panel settings.
*/
PX_C_EXPORT PX_PHYSX_GPU_API int PX_CALL_CONV PxGetSuggestedCudaDeviceOrdinal(physx::PxErrorCallback& errc);

// Implementation of the corresponding functions from PxGpu.h/cpp in the GPU shared library
PX_C_EXPORT PX_PHYSX_GPU_API void PX_CALL_CONV PxGpuCudaRegisterFunction(int moduleIndex, const char* functionName);
PX_C_EXPORT PX_PHYSX_GPU_API void** PX_CALL_CONV PxGpuCudaRegisterFatBinary(void* fatBin);
#if PX_SUPPORT_GPU_PHYSX
PX_C_EXPORT PX_PHYSX_GPU_API physx::PxKernelIndex* PX_CALL_CONV PxGpuGetCudaFunctionTable();
PX_C_EXPORT PX_PHYSX_GPU_API physx::PxU32 PX_CALL_CONV PxGpuGetCudaFunctionTableSize();
PX_C_EXPORT PX_PHYSX_GPU_API void** PX_CALL_CONV PxGpuGetCudaModuleTable();
PX_C_EXPORT PX_PHYSX_GPU_API physx::PxU32 PX_CALL_CONV PxGpuGetCudaModuleTableSize();
PX_C_EXPORT PX_PHYSX_GPU_API physx::PxPhysicsGpu* PX_CALL_CONV PxGpuCreatePhysicsGpu();

#endif

#endif // PX_PHYSX_GPU_H
