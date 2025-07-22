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

#include "PxgPhysXGpu.h"
#include "PxgMemoryManager.h"
#include "PxgHeapMemAllocator.h"
#include "PxgKernelWrangler.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgAABBManager.h"
#include "PxgNphaseImplementationContext.h"
#include "PxgSimulationController.h"
#include "PxgArticulationCore.h"
#include "PxgDynamicsContext.h"
#include "PxgTGSDynamicsContext.h"
#include "CudaKernelWrangler.h"
#include "CudaContextManager.h"
#include "PhysXDeviceSettings.h"
#include "PxgBroadPhase.h"
#include "PxgCommon.h"
#include "PxgNarrowphase.h"
#include "PxgSolver.h"
#include "foundation/PxFoundation.h"
#include "PxgPBDParticleSystemCore.h"
#include "PxgArrayConverter.h"
#include "PxgSDFBuilder.h"
#include "PxgDeformableSkinning.h"
#include "PxsTransformCache.h"

#include "PxgKernelLauncher.h"
#include "PxgIsosurfaceExtraction.h"
#include "PxgAnisotropy.h"
#include "PxgSmoothing.h"
#include "PxgParticleNeighborhoodProvider.h"
#include "gpu/PxPhysicsGpu.h"

using namespace physx;

PxgPhysXGpu* PxgPhysXGpu::sInstance = NULL;

//----------------------------------------------------------------------------//
void PxGpuReleasePhysicsGpu();

void PxgPhysXGpu::release()
{
	for(KernelWranglerMap::Iterator iter = mKernelWranglerInstances.getIterator(); !iter.done(); ++iter)
	{
		if(iter->second)
		{
			PX_DELETE(iter->second);
		}
	}

	mKernelWranglerInstances.clear();

	PX_DELETE_THIS;
	sInstance = NULL;

	PxGpuReleasePhysicsGpu();
}

//----------------------------------------------------------------------------//

PxgPhysXGpu& PxgPhysXGpu::getInstance()
{
	if (!PxgPhysXGpu::sInstance)
		PxgPhysXGpu::sInstance = PX_NEW(PxgPhysXGpu);

	return *PxgPhysXGpu::sInstance;
}

//----------------------------------------------------------------------------//

PxsParticleBuffer* PxgPhysXGpu::createParticleBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxCudaContextManager& cudaContextManager)
{
	PxgParticleBuffer* particleBuffer =  PX_NEW(PxgParticleBuffer)(maxNumParticles, maxVolumes, cudaContextManager);
	return static_cast<PxsParticleBuffer*>(particleBuffer);
}

PxsParticleAndDiffuseBuffer* PxgPhysXGpu::createParticleAndDiffuseBuffer(const PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager& cudaContextManager)
{
	PxgParticleAndDiffuseBuffer* diffuseBuffer = PX_NEW(PxgParticleAndDiffuseBuffer)(maxParticles, maxVolumes, maxDiffuseParticles, cudaContextManager);
	return static_cast<PxsParticleAndDiffuseBuffer*>(diffuseBuffer);
}

PxsParticleClothBuffer* PxgPhysXGpu::createParticleClothBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager& cudaContextManager)
{
	PxgParticleClothBuffer* clothBuffer = PX_NEW(PxgParticleClothBuffer)(maxNumParticles, maxVolumes, maxNumCloths, maxNumTriangles, maxNumSprings, cudaContextManager);
	return static_cast<PxsParticleClothBuffer*>(clothBuffer);
}

PxsParticleRigidBuffer* PxgPhysXGpu::createParticleRigidBuffer(PxU32 maxNumParticles, PxU32 maxVolumes, PxU32 maxNumRigids, PxCudaContextManager& cudaContextManager)
{
	PxgParticleRigidBuffer* rigidBuffer = PX_NEW(PxgParticleRigidBuffer)(maxNumParticles, maxVolumes, maxNumRigids, cudaContextManager);
	return static_cast<PxsParticleRigidBuffer*>(rigidBuffer);
}

PxsMemoryManager* PxgPhysXGpu::createGpuMemoryManager(PxCudaContextManager* cudaContextManager)
{
	return createPxgMemoryManager(cudaContextManager);
}

//----------------------------------------------------------------------------//

PxsHeapMemoryAllocatorManager* PxgPhysXGpu::createGpuHeapMemoryAllocatorManager(const PxU32 heapCapacity, PxsMemoryManager* memoryManager, const PxU32 /*gpuComputeVersion*/)
{
	return PX_NEW(PxgHeapMemoryAllocatorManager)(heapCapacity, memoryManager);
}

//----------------------------------------------------------------------------//

PxsKernelWranglerManager* PxgPhysXGpu::getGpuKernelWranglerManager(PxCudaContextManager* cudaContextManager)
{
	if (!cudaContextManager)
		return NULL;


	const KernelWranglerMap::Entry* entry = mKernelWranglerInstances.find(cudaContextManager);
	if(entry)
		return entry->second;

	PxgCudaKernelWranglerManager* wrangler = PX_NEW(PxgCudaKernelWranglerManager)(*cudaContextManager, *PxGetErrorCallback());
	mKernelWranglerInstances.insert(cudaContextManager, wrangler);
	return wrangler;
}

//----------------------------------------------------------------------------//

Bp::BroadPhase* PxgPhysXGpu::createGpuBroadPhase(const PxGpuBroadPhaseDesc& desc, PxsKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
	PxU32 gpuComputeVersion, const PxGpuDynamicsMemoryConfig& config, 
	PxsHeapMemoryAllocatorManager* heapMemoryManager, PxU64 contextID)
{
	if (gpuComputeVersion == 0)
	{
		return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgCudaBroadPhaseSap), "PxgCudaBroadPhaseSap"), PxgCudaBroadPhaseSap)(desc,
			static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), cudaContextManager, config, static_cast<PxgHeapMemoryAllocatorManager*>(heapMemoryManager), contextID);
	}
	else
	{
		return NULL;
	}
}

//----------------------------------------------------------------------------//

Bp::AABBManagerBase* PxgPhysXGpu::createGpuAABBManager(
	PxsKernelWranglerManager* gpuKernelWrangler,
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
	PxPairFilteringMode::Enum staticKineFilteringMode)
{
	if (gpuComputeVersion == 0)
	{
		return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgAABBManager), "PxgAABBManager"), PxgAABBManager)(
			static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), cudaContextManager, static_cast<PxgHeapMemoryAllocatorManager*>(heapMemoryManager),
			config, bp, boundsArray, contactDistance, maxNbAggregates,maxNbShapes, allocator, contextID, kineKineFilteringMode, staticKineFilteringMode);
	}
	else
	{
		return NULL;
	}
}

//----------------------------------------------------------------------------//

Bp::BoundsArray* PxgPhysXGpu::createGpuBounds( PxVirtualAllocator& allocator)
{
	return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgBoundsArray), "PxgBoundsArray"), PxgBoundsArray)(allocator);
}

//----------------------------------------------------------------------------//

PxvNphaseImplementationContext* PxgPhysXGpu::createGpuNphaseImplementationContext(PxsContext& context,
	PxsKernelWranglerManager* gpuKernelWrangler,
	PxvNphaseImplementationFallback* fallbackForUnsupportedCMs,
	const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig,
	void* contactStreamBase, void* patchStreamBase, void* forceAndIndiceStreamBase,
	PxBoundsArrayPinned& bounds, IG::IslandSim* islandSim, Dy::Context* dynamicsContext, 
	const PxU32 /*gpuComputeVersion*/, PxsHeapMemoryAllocatorManager* heapMemoryManager, bool useGPUBP)
{
	return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgNphaseImplementationContext), "PxgNphaseImplementationContext"), PxgNphaseImplementationContext)(context, gpuKernelWrangler, fallbackForUnsupportedCMs, gpuDynamicsConfig, contactStreamBase, patchStreamBase, forceAndIndiceStreamBase,
		bounds, islandSim, dynamicsContext, static_cast<PxgHeapMemoryAllocatorManager*>(heapMemoryManager), useGPUBP);
}

//----------------------------------------------------------------------------//

PxsSimulationController* PxgPhysXGpu::createGpuSimulationController(PxsKernelWranglerManager* gpuWranglerManagers, PxCudaContextManager* cudaContextManager, 
	Dy::Context* dynamicContext, PxvNphaseImplementationContext* npContext,
	Bp::BroadPhase* bp, const bool useGpuBroadphase, PxsSimulationControllerCallback* callback, 
	const PxU32 /*gpuComputeVersion*/, PxsHeapMemoryAllocatorManager* heapMemoryManager, const PxU32 maxSoftBodyContacts, const PxU32 maxFemClothContacts,
	const PxU32 maxParticleContacts, const PxU32 collisionStackSizeBytes, bool enableBodyAccelerations)
{
	return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgSimulationController), "PxgSimulationController"),
		PxgSimulationController)(gpuWranglerManagers, cudaContextManager, static_cast<PxgDynamicsContext*>(dynamicContext), static_cast<PxgNphaseImplementationContext*>(npContext),
		bp, useGpuBroadphase, callback, static_cast<PxgHeapMemoryAllocatorManager*>(heapMemoryManager), maxSoftBodyContacts, 
			maxFemClothContacts, maxParticleContacts, collisionStackSizeBytes, enableBodyAccelerations);
}

//----------------------------------------------------------------------------//

Dy::Context* PxgPhysXGpu::createGpuDynamicsContext(Cm::FlushPool& taskPool, PxsKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, 
	const PxGpuDynamicsMemoryConfig& config, IG::SimpleIslandManager& islandManager, PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions, bool enableStabilization, 
	bool useEnhancedDeterminism, PxReal maxBiasCoefficient, PxU32 /*gpuComputeVersion*/, PxvSimStats& simStats, PxsHeapMemoryAllocatorManager* heapMemoryManager,
	bool frictionEveryIteration, bool externalForcesEveryTgsIterationEnabled, PxSolverType::Enum solverType, PxReal lengthScale, bool enableDirectGPUAPI, PxU64 contextID, bool isResidualReportingEnabled)
{
	if(solverType == PxSolverType::eTGS)
		return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgTGSDynamicsContext), "PxgDynamicsContext"), PxgTGSDynamicsContext)(taskPool, gpuKernelWrangler, cudaContextManager, config, islandManager,
			maxNumPartitions, maxNumStaticPartitions, enableStabilization, useEnhancedDeterminism, maxBiasCoefficient, simStats, 
			static_cast<PxgHeapMemoryAllocatorManager*>(heapMemoryManager), externalForcesEveryTgsIterationEnabled, lengthScale, enableDirectGPUAPI, contextID, isResidualReportingEnabled);
	else
		return PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgDynamicsContext), "PxgDynamicsContext"), PxgDynamicsContext)(taskPool, gpuKernelWrangler, cudaContextManager, config, islandManager,
			maxNumPartitions, maxNumStaticPartitions, enableStabilization, useEnhancedDeterminism, maxBiasCoefficient, simStats, 
			static_cast<PxgHeapMemoryAllocatorManager*>(heapMemoryManager), frictionEveryIteration, lengthScale, enableDirectGPUAPI, contextID, isResidualReportingEnabled);
}

//----------------------------------------------------------------------------//
// Implementation of exported functions
//----------------------------------------------------------------------------//

PxPhysXGpu* PxCreatePhysXGpu()
{
	createPxgBroadphase();
	createPxgCommon();
	createPxgNarrowphase();
	createPxgSimulationController();
	createPxgArticulation();
	createPxgSolver();
	return &PxgPhysXGpu::getInstance();
}

//----------------------------------------------------------------------------//

physx::PxCudaContextManager* PxCreateCudaContextManager(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc, physx::PxProfilerCallback* profilerCallback, bool launchSynchronous)
{
	//this is only necessary for static lib configs with PhysXGpu still being a shared lib. 
	PxSetFoundationInstance(foundation);

	PxSetProfilerCallback(profilerCallback);
	
	return physx::createCudaContextManager(desc, foundation.getErrorCallback(), launchSynchronous);
}

//----------------------------------------------------------------------------//

void PxSetPhysXGpuProfilerCallback(physx::PxProfilerCallback* profilerCallback)
{
	PxSetProfilerCallback(profilerCallback);
}

//----------------------------------------------------------------------------//

void PxSetPhysXGpuFoundationInstance(physx::PxFoundation& foundation)
{
	PxSetFoundationInstance(foundation);
}

//----------------------------------------------------------------------------//

int PxGetSuggestedCudaDeviceOrdinal(physx::PxErrorCallback& errc)
{
	return physx::PhysXDeviceSettings::getSuggestedCudaDeviceOrdinal(errc);
}

//----------------------------------------------------------------------------//
static const PxU32 s_maxNumFunctions = 1024; // Max number of kernel entry points
static PxKernelIndex s_FunctionTable[s_maxNumFunctions];
static PxU32 s_numFunctions = 0;

void PxGpuCudaRegisterFunction(int moduleIndex, const char* functionName)
{
	if(s_numFunctions < s_maxNumFunctions)
	{
		s_FunctionTable[s_numFunctions].moduleIndex = moduleIndex;
		s_FunctionTable[s_numFunctions].functionName = functionName;
		s_numFunctions++;
	}
	else
	{
		if(PxIsFoundationValid()) // error callback requires foundation
			PxGetErrorCallback()->reportError(PxErrorCode::eINTERNAL_ERROR, "Too many cuda kernels registered. Increase maxNumFunctions limit.", PX_FL);
		else
			fprintf(stderr, "Too many cuda kernels registered. Increase maxNumFunctions limit. (%s:%i)\n", PX_FL);
	}
}

PxKernelIndex* PxGpuGetCudaFunctionTable()
{
	return s_FunctionTable;
}

PxU32 PxGpuGetCudaFunctionTableSize()
{
	return s_numFunctions;
}

static PxU32 s_numModules = 0;
static const PxU32 s_maxNumModules = 128; // max number of *.cu files
static void* s_moduleTable[s_maxNumModules];

void** PxGpuCudaRegisterFatBinary(void* fatBin)
{
	//HACK to get real fatbin in CUDA 4.0
	struct CUIfatbinStruct
	{
		int magic;
		int version;
		void *fatbinArray;
		char *fatbinFile;
	};
	const CUIfatbinStruct *fatbinStruct = (const CUIfatbinStruct *)fatBin;
	if (fatbinStruct->magic == 0x466243B1)
	{
		fatBin = fatbinStruct->fatbinArray;
	}


	if (s_numModules < s_maxNumModules)
	{
		s_moduleTable[s_numModules] = fatBin;

		// what we return here will be the value that is passed in as moduleIndex
		// in PxGpuCudaRegisterFunction. We simply use the index into s_moduleTable
		// as a handle
		return (void**)(size_t) s_numModules++;
	}
	else
	{
		if(PxIsFoundationValid()) // error callback requires foundation
			PxGetErrorCallback()->reportError(PxErrorCode::eINTERNAL_ERROR, "Too many cuda modules registered. Increase maxNumModules limit.", PX_FL);
		else
			fprintf(stderr, "Too many cuda modules registered. Increase maxNumModules limit. (%s:%i)\n", PX_FL);

	}
	return NULL;
}

void** PxGpuGetCudaModuleTable()
{
	return s_moduleTable;
}

PxU32 PxGpuGetCudaModuleTableSize()
{
	return s_numModules;
}




class PxgPhysicsGpu : public PxPhysicsGpu, public PxUserAllocated
{
public:
	PxgPhysicsGpu() {}

	virtual PxIsosurfaceExtractor* createDenseGridIsosurfaceExtractor(PxCudaContextManager* cudaContextManager, const PxBounds3& worldBounds,
		PxReal cellSize, const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles);

	virtual PxSparseGridIsosurfaceExtractor* createSparseGridIsosurfaceExtractor(PxCudaContextManager* cudaContextManager, const PxSparseGridParams& sparseGridParams,
		const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles);

	virtual PxAnisotropyGenerator* createAnisotropyGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles,
		PxReal anisotropyScale, PxReal minAnisotropy, PxReal maxAnisotropy);

	virtual PxSmoothedPositionGenerator* createSmoothedPositionGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles, PxReal smoothingStrength);

	virtual PxParticleNeighborhoodProvider* createParticleNeighborhoodProvider(PxCudaContextManager* cudaContextManager, const PxU32 maxNumParticles,
		const PxReal cellSize, const PxU32 maxNumSparseGridCells);

	virtual PxArrayConverter* createArrayConverter(PxCudaContextManager* cudaContextManager);

	virtual PxSDFBuilder* createSDFBuilder(PxCudaContextManager* cudaContextManager);

	virtual PxgDeformableSkinning* createDeformableSkinning(PxCudaContextManager* cudaContextManager);

	virtual void release();

	virtual ~PxgPhysicsGpu() {}

	static PxgPhysicsGpu* sInstance;
};

PxgPhysicsGpu* PxgPhysicsGpu::sInstance = NULL;

PxIsosurfaceExtractor* PxgPhysicsGpu::createDenseGridIsosurfaceExtractor(PxCudaContextManager* cudaContextManager, const PxBounds3& worldBounds,
	PxReal cellSize, const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgDenseGridIsosurfaceExtractor)(kernelLauncher, worldBounds, cellSize, isosurfaceParams, maxNumParticles, maxNumVertices, maxNumTriangles);
}

PxSparseGridIsosurfaceExtractor* PxgPhysicsGpu::createSparseGridIsosurfaceExtractor(PxCudaContextManager* cudaContextManager, const PxSparseGridParams& sparseGridParams,
	const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgSparseGridIsosurfaceExtractor)(kernelLauncher, sparseGridParams, isosurfaceParams, maxNumParticles, maxNumVertices, maxNumTriangles);
}

PxAnisotropyGenerator* PxgPhysicsGpu::createAnisotropyGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles, PxReal anisotropyScale, PxReal minAnisotropy, PxReal maxAnisotropy)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgAnisotropyGenerator)(kernelLauncher, maxNumParticles, anisotropyScale, minAnisotropy, maxAnisotropy);
}

PxSmoothedPositionGenerator* PxgPhysicsGpu::createSmoothedPositionGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles, PxReal smoothingStrength)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgSmoothedPositionGenerator)(kernelLauncher, maxNumParticles, smoothingStrength);
}

PxParticleNeighborhoodProvider* PxgPhysicsGpu::createParticleNeighborhoodProvider(PxCudaContextManager* cudaContextManager, const PxU32 maxNumParticles, const PxReal particleContactOffset, const PxU32 maxNumSparseGridCells)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgParticleNeighborhoodProvider)(kernelLauncher, maxNumParticles, particleContactOffset, maxNumSparseGridCells);
}

PxArrayConverter* PxgPhysicsGpu::createArrayConverter(PxCudaContextManager* cudaContextManager)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgArrayConverter)(kernelLauncher);
}

PxSDFBuilder* PxgPhysicsGpu::createSDFBuilder(PxCudaContextManager* cudaContextManager)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgSDFBuilder)(kernelLauncher);
}

PxgDeformableSkinning* PxgPhysicsGpu::createDeformableSkinning(PxCudaContextManager* cudaContextManager)
{
	PxgCudaKernelWranglerManager* wrangler = static_cast<PxgCudaKernelWranglerManager*>(PxCreatePhysXGpu()->getGpuKernelWranglerManager(cudaContextManager));
	PxgKernelLauncher kernelLauncher(cudaContextManager, wrangler);
	return PX_NEW(PxgDeformableSkinning)(kernelLauncher);
}

void PxgPhysicsGpu::release()
{
	PX_FREE_THIS;
}

PxPhysicsGpu* PxGpuCreatePhysicsGpu()
{
	if (!PxgPhysicsGpu::sInstance)
		PxgPhysicsGpu::sInstance = PX_NEW(PxgPhysicsGpu);

	return PxgPhysicsGpu::sInstance;
}

void PxGpuReleasePhysicsGpu()
{
	if (PxgPhysicsGpu::sInstance)	
	{
		PxgPhysicsGpu::sInstance->release();
		PxgPhysicsGpu::sInstance = NULL;
	}
}

//----------------------------------------------------------------------------//
