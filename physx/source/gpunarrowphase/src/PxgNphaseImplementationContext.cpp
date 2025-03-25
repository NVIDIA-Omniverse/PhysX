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

#include "common/PxProfileZone.h"
#include "cudamanager/PxCudaContext.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxPreprocessor.h"

#include "PxsContext.h"
#include "PxsSimpleIslandManager.h"
#include "PxsMaterialManager.h"
#include "CmFlushPool.h"
#include "foundation/PxArray.h"
#include "foundation/PxAllocator.h"

#include "PxvDynamics.h"			// for PxsBodyCore
#include "PxvGeometry.h"			// for PxsShapeCore		
#include "mesh/GuTriangleMesh.h"
#include "hf/GuHeightField.h"
#include "PxgNphaseImplementationContext.h"
#include "PxgSolverCore.h"

//FDTODO: those incudes should eventually disappear as we get rid of the copy-pasted stuff
#include "PxgDynamicsContext.h"
#include "PxgKernelWrangler.h"
#include "PxgSimulationCore.h"
#include "PxgParticleSystemCore.h"
#include "PxgArticulationCore.h"

#include "cudamanager/PxCudaContextManager.h"

#include "GuConvexGeometry.h"
#include "GuConvexSupport.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#endif
#include "convexNpCommon.h"

using namespace physx;
using namespace Gu;

#define PXG_ENABLE_GPU_NP 1

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

void PxgCMGpuDiscreteUpdateBase::processContactManagers(PxgContactManagers& managers, PxgGpuContactManagers& managersGPU, GPU_BUCKET_ID::Enum type)
{
	PxRenderOutput renderOutput(mContext->mContext.getRenderBuffer());

	managersGPU.mLostAndTotalReportedPairsCountPinned->y = 0;
	managersGPU.mLostAndTotalReportedPairsCountPinned->x = 0;

	const PxU32 numTests = managers.mCpuContactManagerMapping.size();

	if (numTests == 0)
		return;

	PxU32 maxPatches = 1;

	switch (type)
	{
	case GPU_BUCKET_ID::eConvex:
		mContext->mGpuNarrowphaseCore->testSDKConvexConvexGjkEpaGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput
		);
		break;
	case GPU_BUCKET_ID::eConvexPlane:
		mContext->mGpuNarrowphaseCore->testSDKConvexPlaneGjkEpaGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput
		);
		break;
	case GPU_BUCKET_ID::eConvexTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKConvexTriMeshSATGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
		maxPatches = PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS;
		break;
	}
	case GPU_BUCKET_ID::eConvexHeightfield:
	{
		mContext->mGpuNarrowphaseCore->testSDKConvexHeightfieldGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
		maxPatches = PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS;
		break;
	}
	case GPU_BUCKET_ID::eConvexCorePlane:
		mContext->mGpuNarrowphaseCore->testSDKConvexCorePlaneGjkEpaGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput);
		break;
	case GPU_BUCKET_ID::eConvexCoreConvex:
		mContext->mGpuNarrowphaseCore->testSDKConvexCoreConvexGjkEpaGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput);
		break;
	case GPU_BUCKET_ID::eConvexCoreTrimesh:
		mContext->mGpuNarrowphaseCore->testSDKConvexCoreTrimeshGjkEpaGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput);
		maxPatches = Gu::Contact::MAX_PATCHES;
		break;
	case GPU_BUCKET_ID::eConvexCoreTetmesh:
		mContext->mGpuNarrowphaseCore->testSDKConvexCoreTetmeshGjkEpaGpu(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	case GPU_BUCKET_ID::eConvexCoreClothmesh:
		mContext->mGpuNarrowphaseCore->testSDKConvexCoreClothmeshGjkEpaGpu(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	case GPU_BUCKET_ID::eSphere:
	{
		mContext->mGpuNarrowphaseCore->testSDKSphereGpu(managersGPU, managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput
		);
		break;
	}
	case GPU_BUCKET_ID::eBoxBox:
	{
		mContext->mGpuNarrowphaseCore->testSDKBoxBoxGpu(managersGPU, managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize,
			&renderOutput
		);
		break;
	}
	case GPU_BUCKET_ID::eSphereTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKSphereTriMeshSATGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
		maxPatches = PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS;
		break;
	}
	case GPU_BUCKET_ID::eSphereHeightfield:
	{
		mContext->mGpuNarrowphaseCore->testSDKSphereHeightfieldGpu(managersGPU,
			mContext->mContext.getCreateAveragePoint(), managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
		maxPatches = PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS;
		break;
	}
	case GPU_BUCKET_ID::eTrianglePlane:
	{
		mContext->mGpuNarrowphaseCore->testSDKTriMeshPlaneGpu(managersGPU,
			managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
		break;
	}

	case GPU_BUCKET_ID::eTriangleHeightfield:
	{
		mContext->mGpuNarrowphaseCore->testSDKTriMeshHeightfieldGpu(managersGPU,
			managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
			maxPatches = MAX_MESH_MESH_PATCHES;
		
		break;
	}
	case GPU_BUCKET_ID::eTriangleTriangle:
	{
		mContext->mGpuNarrowphaseCore->testSDKTriMeshTriMeshGpu(managersGPU,
			managers.mCpuContactManagerMapping.size(),
			mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mContactStreamPool->mDataStream,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStream,
			mContext->mContext.mPatchStreamPool->mDataStreamSize, mContext->mContext.mContactStreamPool->mDataStreamSize,
			mContext->mContext.mForceAndIndiceStreamPool->mDataStreamSize);
		maxPatches = MAX_MESH_MESH_PATCHES;
		break;
	}
	case GPU_BUCKET_ID::eSoftbody:
	{
		mContext->mGpuNarrowphaseCore->testSDKSoftbody(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eSoftbodies:
	{
		mContext->mGpuNarrowphaseCore->testSDKSoftbodies(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eSoftbodyFemCloth:
	{
		mContext->mGpuNarrowphaseCore->testSDKSoftbodyCloth(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eSoftbodyTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKSoftbodyTrimesh(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eSoftbodySdfTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKSoftbodySdfTrimesh(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eSoftbodyHeightfield:
	{
		mContext->mGpuNarrowphaseCore->testSDKSoftbodyHeightfield(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eFemClothSphere:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothSphere(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eFemClothPlane:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothPlane(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eFemClothBox:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothBox(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eFemClothConvexes:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothConvexes(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eFemClothes:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothCloth(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eFemClothTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothTrimesh(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);

		break;
	}
	case GPU_BUCKET_ID::eFemClothSdfTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothSdfTrimesh(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);

		break;
	}
	case GPU_BUCKET_ID::eFemClothHeightfield:
	{
		mContext->mGpuNarrowphaseCore->testSDKFemClothHeightfield(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);

		break;
	}
	case GPU_BUCKET_ID::eConvexParticle:
	{
		mContext->mGpuNarrowphaseCore->testSDKConvexParticle(managersGPU,
			managers.mCpuContactManagerMapping.size());
		break;
	}
	case GPU_BUCKET_ID::eParticlesystems:
	{
		mContext->mGpuNarrowphaseCore->testSDKParticleSystemGpu(managersGPU,
			managers.mCpuContactManagerMapping.size());
		break;
	}
	case GPU_BUCKET_ID::eParticlesystemSoftbody:
	{
		mContext->mGpuNarrowphaseCore->testSDKParticleSoftbody(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eParticlesystemFemCloth:
	{
		mContext->mGpuNarrowphaseCore->testSDKParticleFemCloth(managersGPU,
			managers.mCpuContactManagerMapping.size(), &renderOutput);
		break;
	}
	case GPU_BUCKET_ID::eParticlesystemTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKParticleTriMesh(managersGPU,
			managers.mCpuContactManagerMapping.size());
		break;
	}
	case GPU_BUCKET_ID::eParticlesystemSdfTrimesh:
	{
		mContext->mGpuNarrowphaseCore->testSDKParticleSdfTriMesh(managersGPU,
			managers.mCpuContactManagerMapping.size());
		break;
	}
	case GPU_BUCKET_ID::eParticlesystemHeightfield:
	{
		mContext->mGpuNarrowphaseCore->testSDKParticleHeightfield(managersGPU,
			managers.mCpuContactManagerMapping.size());
		break;
	}
	case GPU_BUCKET_ID::eFallback:
	case GPU_BUCKET_ID::eCount:
	default:
		break;
	}

	mContext->mGpuNarrowphaseCore->updateFrictionPatches(managersGPU, managers.mCpuContactManagerMapping.size(),
		mContext->mContext.mPatchStreamPool->mDataStream, mContext->mContext.mFrictionPatchStreamPool->mDataStream);

	mContext->mMaxPatches = PxMax(mContext->mMaxPatches, maxPatches);
}

void PxgCMGpuDiscreteUpdateTask::runInternal()
{
	mContext->removeLostPairs();

	mPostBroadPhaseTask->removeReference();

	PxgGpuNarrowphaseCore* npCore = mContext->mGpuNarrowphaseCore;

	mContext->mGpuNarrowphaseCore->syncNotRigidWithNp();

	mContext->mMaxPatches = 1;

	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		GPU_BUCKET_ID::Enum type = GPU_BUCKET_ID::Enum(i);
		processContactManagers(npCore->getExistingContactManagers(type),
			npCore->getExistingGpuContactManagers(type), type);
	}
}

void PxgCMGpuDiscreteSecondPassUpdateTask::runInternal()
{
	PxgGpuNarrowphaseCore* npCore = mContext->mGpuNarrowphaseCore;

	npCore->prepareTempContactManagers();

	npCore->syncNotRigidWithNp();

	PxU32 numCpuContactManagerMappings = 0;

	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
	{
		GPU_BUCKET_ID::Enum type = GPU_BUCKET_ID::Enum(i);

		PxgNewContactManagers& newContactManger = npCore->getNewContactManagers(type);

		processContactManagers(newContactManger,
			npCore->getNewGpuContactManagers(type),
			GPU_BUCKET_ID::Enum(i));

		numCpuContactManagerMappings += newContactManger.mCpuContactManagerMapping.size();
	}

	if(numCpuContactManagerMappings > 0)
	{
		mContext->mGpuNarrowphaseCore->pushBuffer();
	}
}

PxgNphaseImplementationContext::PxgNphaseImplementationContext(
	PxsContext& context, PxsKernelWranglerManager* gpuKernelWrangler, PxvNphaseImplementationFallback* fallbackForUnsupportedCMs,
	const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase,  void* patchStreamBase, void* forceAndIndiceStreamBase,
	PxBoundsArrayPinned& bounds,  IG::IslandSim* islandSim, physx::Dy::Context* dynamicsContext, PxgHeapMemoryAllocatorManager* heapMemoryManager, bool useGPUBP) :
	PxvNphaseImplementationContext	(context),
	mFallbackForUnsupportedCMs		(fallbackForUnsupportedCMs),
	mUpdateCMsFirstPassTask			(this),
	mUpdateCMsSecondPassTask		(this),
	mUpdateFallbackPairs			(this),
	mContactManagerOutputs			(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
	mTotalNbPairs					(0),
	mBounds							(bounds),
	mUseGPUBP						(useGPUBP)
{
#if PX_CHECKED
	if(!context.getPCM())
		outputError<PxErrorCode::ePERF_WARNING>(__LINE__, "Attempting to use GPU rigid body pipeline without using PCM contact gen! All contact gen will be performed on CPU.");
#endif
	{
		mGpuDynamicContext = static_cast<PxgGpuContext*>(dynamicsContext);
		CUstream stream = mGpuDynamicContext->getGpuSolverCore()->getStream();
		mGpuNarrowphaseCore = PX_NEW(PxgGpuNarrowphaseCore)(static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), context.getCudaContextManager(), gpuDynamicsConfig,
															contactStreamBase, patchStreamBase, forceAndIndiceStreamBase, islandSim, stream, heapMemoryManager, this);

		mGpuDynamicContext->mGpuNpCore = mGpuNarrowphaseCore;
		mGpuNarrowphaseCore->mGpuContext = mGpuDynamicContext;
		
		for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
		{
			mRecordedGpuPairCount[i] = mNbPairCount[i] = 0;
		}
	}
	mMaxPatches = 1;
};

PxgNphaseImplementationContext::~PxgNphaseImplementationContext()
{
	PxCudaContextManager* contMan = mContext.getCudaContextManager();
	contMan->acquireContext();
	PX_DELETE(mGpuNarrowphaseCore);
	contMan->releaseContext();
}

void PxgCMGpuDiscreteUpdateFallbackTask::runInternal()
{
	mContext->updateContactManagersFallback(mDt, mCont);
}

void PxgNphaseImplementationContext::removeLostPairs()
{
	mGpuNarrowphaseCore->removeLostPairs();

	//mGpuNarrowphaseCore->removeLostPairsConvexTriMesh(mContext.getEventProfiler());
}

void PxgNphaseImplementationContext::prepareTempContactManagers(PxBaseTask* continuation)
{
	mGpuNarrowphaseCore->prepareTempContactManagersTasks(mContext.getTaskPool(), continuation);
}

void PxgNphaseImplementationContext::updateContactManagersFallback(PxReal dt, PxBaseTask* continuation)
{
	mFallbackForUnsupportedCMs->removeContactManagersFallback(mContactManagerOutputs.begin());
	mFallbackForUnsupportedCMs->processContactManager(dt, mContactManagerOutputs.begin(), continuation);
}

void PxgNphaseImplementationContext::updateNarrowPhaseShape()
{
	//run in npstream
	mGpuNarrowphaseCore->prepareGpuNarrowphase(mContext.getTransformCache(), mContext.getContactDistances(), 
		mHasContactDistanceChanged);
}

void PxgNphaseImplementationContext::updateContactManager(PxReal dt, bool hasContactDistanceChanged, PxBaseTask* continuation,
	PxBaseTask* firstPassNpContinuation, Cm::FanoutTask* updateBoundAndShapeTask)
{
	PX_PROFILE_ZONE("Sim.queueNarrowPhase", 0);

	mHasContactDistanceChanged = hasContactDistanceChanged;

	mContext.clearManagerTouchEvents();

	mUpdateFallbackPairs.setContinuation(continuation);
	mUpdateFallbackPairs.setDt(dt);
	mUpdateFallbackPairs.removeReference();

#if PX_ENABLE_SIM_STATS
	mContext.mSimStats.mNbDiscreteContactPairsTotal = 0;
	mContext.mSimStats.mNbDiscreteContactPairsWithCacheHits = 0;
	mContext.mSimStats.mNbDiscreteContactPairsWithContacts = 0;
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	
	mContext.mMaxPatches = 0;
			
	mUpdateCMsFirstPassTask.setDt(dt);
	mUpdateCMsFirstPassTask.setFirstPassContinuation(firstPassNpContinuation);
	mUpdateCMsFirstPassTask.setContinuation(continuation);

	updateBoundAndShapeTask->addDependent(mUpdateCMsFirstPassTask);
	updateBoundAndShapeTask->removeReference();
}

void PxgNphaseImplementationContext::postBroadPhaseUpdateContactManager(PxBaseTask* continuation)
{
	continuation->addReference();
	mUpdateCMsFirstPassTask.setPostBroadPhaseTask(continuation);
	mUpdateCMsFirstPassTask.removeReference();
}

void PxgNphaseImplementationContext::secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.queueNarrowPhase", 0);	
	mUpdateCMsSecondPassTask.setContinuation(continuation);
	mUpdateCMsSecondPassTask.setDt(dt);

	prepareTempContactManagers(&mUpdateCMsSecondPassTask);

	mUpdateCMsSecondPassTask.removeReference();

	mFallbackForUnsupportedCMs->processContactManagerSecondPass(dt, continuation);
}

void PxgNphaseImplementationContext::fetchUpdateContactManager()
{
	mergeContactManagers(NULL);
}

void PxgNphaseImplementationContext::destroy()
{
	this->~PxgNphaseImplementationContext();
	PX_FREE_THIS;
}

void PxgNphaseImplementationContext::processResults()
{
	{
		PX_PROFILE_ZONE("GpuNarrowPhase.processResults", 0);

		PxcNpThreadContext* PX_RESTRICT threadContext = mContext.getNpThreadContext(); 
	
		threadContext->mPCM = mContext.getPCM();
		threadContext->mCreateAveragePoint = mContext.getCreateAveragePoint();
		threadContext->mContactCache = mContext.getContactCacheFlag();
		threadContext->mTransformCache = &(mContext.getTransformCache());
		// PT: TODO: we don't setup the contact distances here?

		PxU32 newTouchCMCount = 0, lostTouchCMCount = 0;
		PxU32 nbConvexConvex = 0;
		PxU32 nbConvexPlane = 0;
		PxU32 nbConvexTriMesh = 0;
		PxU32 nbConvexHf = 0;
		PxU32 nbSphere = 0;
		PxU32 nbSphereTriMesh = 0;
		PxU32 nbSphereHf = 0;
		PxU32 nbTrianglePlane = 0;
		PxU32 nbTriangleHf = 0;

		PxU32 maxPatches = threadContext->mMaxPatches;
		{
			nbConvexConvex = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eConvex]->mContactManagers.mCpuContactManagerMapping.size();
			nbConvexPlane = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eConvexPlane]->mContactManagers.mCpuContactManagerMapping.size();
			nbConvexTriMesh = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eConvexTrimesh]->mContactManagers.mCpuContactManagerMapping.size();
			nbConvexHf = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eConvexHeightfield]->mContactManagers.mCpuContactManagerMapping.size();

			nbSphere = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eSphere]->mContactManagers.mCpuContactManagerMapping.size();
			nbSphereTriMesh = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eSphereTrimesh]->mContactManagers.mCpuContactManagerMapping.size();
			nbSphereHf = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eSphereHeightfield]->mContactManagers.mCpuContactManagerMapping.size();
		
			nbTrianglePlane = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eTrianglePlane]->mContactManagers.mCpuContactManagerMapping.size();
			nbTriangleHf = mGpuNarrowphaseCore->mContactManagers[GPU_BUCKET_ID::eTriangleHeightfield]->mContactManagers.mCpuContactManagerMapping.size();

			PxBitMap& localChangeTouchCM = threadContext->getLocalChangeTouch();
				
			//KS - convex-convex can only produce a maximum of 1 contact patch and convex-mesh produces a max of 4. Mesh-mesh can produce a maximum of
			//32 patches...
			maxPatches = PxMax(maxPatches, mMaxPatches);

			//Only process the array of found/lost touch events (let's confirm that this works...)
			PxPinnedArray<PxsContactManager*>& lostFoundIndIterator = mGpuNarrowphaseCore->getLostFoundPairsCms();
			PxPinnedArray<PxsContactManagerOutputCounts>& itChangedOutputs = mGpuNarrowphaseCore->getLostFoundPairsOutput();

			PxU32 nbLostFoundPairs = mGpuNarrowphaseCore->getTotalNbLostFoundPairs();

			// jcarius: Defensive coding added for OM-119911. In theory this should not be needed, as these counts
			// should never get into an invalid state. We could/should remove this eventually.
			// ### DEFENSIVE
			PX_ASSERT(nbLostFoundPairs <= lostFoundIndIterator.size());
			PX_ASSERT(nbLostFoundPairs <= itChangedOutputs.size());
			if(nbLostFoundPairs > lostFoundIndIterator.size() || nbLostFoundPairs > itChangedOutputs.size())
			{
				outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "PxgNphaseImplementationContext::processResults: Inconsistent pair numbers!");
				nbLostFoundPairs = PxMin(lostFoundIndIterator.size(), itChangedOutputs.size());
			}

			PxU32 maxCmIndx = mContext.getContactManagerPool().getMaxUsedIndex();
			localChangeTouchCM.resize(maxCmIndx);

			//KS - TODO - let's remove this loop entirely in the near future!
			for(PxU32 a = 0; a < nbLostFoundPairs; ++a)
			{
				PxsContactManager* cm = lostFoundIndIterator[a];

				//PX_ASSERT(cm->getWorkUnit().statusFlags != itChangedOutputs[a].statusFlag);

				cm->getWorkUnit().mStatusFlags = itChangedOutputs[a].statusFlag; //KS - cache status flag in PxcNpWorkUnit to simplify logic elsewhere....

				if(itChangedOutputs[a].nbPatches)
					newTouchCMCount++;
				else
					lostTouchCMCount++;

				localChangeTouchCM.growAndSet(cm->getIndex());
			}
		}

#if PX_ENABLE_SIM_STATS
		threadContext->mDiscreteContactPairs[PxGeometryType::eSPHERE][PxGeometryType::eSPHERE] += nbSphere;
		threadContext->mDiscreteContactPairs[PxGeometryType::eCONVEXMESH][PxGeometryType::eCONVEXMESH]+= nbConvexConvex;
		threadContext->mDiscreteContactPairs[PxGeometryType::ePLANE][PxGeometryType::eCONVEXMESH] += nbConvexPlane;
		threadContext->mDiscreteContactPairs[PxGeometryType::eCONVEXMESH][PxGeometryType::eTRIANGLEMESH]+= nbConvexTriMesh;
		threadContext->mDiscreteContactPairs[PxGeometryType::eCONVEXMESH][PxGeometryType::eHEIGHTFIELD] += nbConvexHf;
		threadContext->mDiscreteContactPairs[PxGeometryType::eSPHERE][PxGeometryType::eTRIANGLEMESH] += nbSphereTriMesh;
		threadContext->mDiscreteContactPairs[PxGeometryType::eSPHERE][PxGeometryType::eHEIGHTFIELD] += nbSphereHf;
		threadContext->mDiscreteContactPairs[PxGeometryType::ePLANE][PxGeometryType::eTRIANGLEMESH] += nbTrianglePlane;
		threadContext->mDiscreteContactPairs[PxGeometryType::eTRIANGLEMESH][PxGeometryType::eHEIGHTFIELD] += nbTriangleHf;
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		threadContext->addLocalNewTouchCount(newTouchCMCount);
		threadContext->addLocalLostTouchCount(lostTouchCMCount);

		threadContext->mMaxPatches = maxPatches;

		mContext.putNpThreadContext(threadContext);
	}
}

void PxgNphaseImplementationContext::appendContactManagers()
{
}

PxsContactManagerOutputCounts* PxgNphaseImplementationContext::getLostFoundPatchOutputCounts()
{
	return mGpuNarrowphaseCore->mLostFoundPairsOutputData.begin() + mGpuNarrowphaseCore->mTotalLostFoundPairs;
}

PxsContactManager** PxgNphaseImplementationContext::getLostFoundPatchManagers()
{
	return mGpuNarrowphaseCore->mLostFoundPairsCms.begin() + mGpuNarrowphaseCore->mTotalLostFoundPairs;
}

PxU32 PxgNphaseImplementationContext::getNbLostFoundPatchManagers()
{
	return mGpuNarrowphaseCore->mTotalLostFoundPatches;
}

void PxgNphaseImplementationContext::mergeContactManagers(PxBaseTask* /*continuation*/)
{
	mContext.getCudaContextManager()->acquireContext();

	if (mContactManagerOutputs.capacity() < mTotalNbPairs)
	{
		PX_PROFILE_ZONE("GpuNarrowPhase.allocateManagerOutputs", 0);
		mContactManagerOutputs.reserve(PxMax(mTotalNbPairs, mContactManagerOutputs.capacity() * 2));
	}
	mContactManagerOutputs.forceSize_Unsafe(mTotalNbPairs);

	mFallbackForUnsupportedCMs->appendContactManagersFallback(mContactManagerOutputs.begin());

	PxMemCopy(mRecordedGpuPairCount, mNbPairCount, sizeof(PxU32) * GPU_BUCKET_ID::eCount);

	const PxU32 nbFallbackPairs = mNbPairCount[GPU_BUCKET_ID::eFallback];
	mGpuNarrowphaseCore->fetchNarrowPhaseResults(mContext.mContactStreamPool, mContext.mPatchStreamPool, mContext.mForceAndIndiceStreamPool, 
		mContactManagerOutputs.begin(),
		mFallbackForUnsupportedCMs->getShapeInteractionsGPU(),
		mFallbackForUnsupportedCMs->getRestDistancesGPU(), 
		mFallbackForUnsupportedCMs->getTorsionalDataGPU(),
		nbFallbackPairs, mFallbackForUnsupportedCMs->getLostFoundPatchOutputCounts(), 
		mFallbackForUnsupportedCMs->getLostFoundPatchManagers(), mFallbackForUnsupportedCMs->getNbLostFoundPatchManagers());

	// AD: need to make sure we're safe if fetchNarrowphaseResults did not run completely because of abort.
	if (mGpuNarrowphaseCore->mCudaContext->isInAbortMode())
	{
		// We have some coupling issues here: we index into this array with mNpIndex, which is never bounds-checked
		// so if we force the size to 0 here, we will still access the memory no matter what. Because we keep passing
		// this thing around as a pointer. This needs to be guarded in all places where we get a contactOutputIterator.
		// we cleared all the data to 0 inside this function, the same as we are doing for contactManagerOutputs that overflow
		// the contact stream. So we are able to handle things being NULL in there. I really hope this is the case.
		mContactManagerOutputs.forceSize_Unsafe(0);
		mTotalNbPairs = 0;
	}

	bool contactsOverflown = mContext.mContactStreamPool->isOverflown(), forcesOverflown = mContext.mForceAndIndiceStreamPool->isOverflown(),
		patchesOverflown = mContext.mPatchStreamPool->isOverflown();

	if (contactsOverflown || forcesOverflown || patchesOverflown)
	{
		PxU8* contactLimit = mContext.mContactStreamPool->mDataStream + mContext.mContactStreamPool->mSharedDataIndexGPU;
		PxU8* forceLimit = mContext.mForceAndIndiceStreamPool->mDataStream + mContext.mForceAndIndiceStreamPool->mSharedDataIndexGPU;
		PxU8* patchLimit = mContext.mPatchStreamPool->mDataStream + mContext.mPatchStreamPool->mSharedDataIndexGPU;
		
		for (PxU32 i = 0; i < nbFallbackPairs; ++i)
		{
			PxsContactManagerOutput* output = mContactManagerOutputs.begin() + i;

			if ((output->nbContacts && (output->contactPoints < contactLimit)) || 
				(output->nbContacts && ((PxU8*) output->contactForces < forceLimit)) ||
				(output->nbPatches && (output->contactPatches < patchLimit)))
			{
				output->nbContacts = 0;
				output->nbPatches = 0;
				output->contactForces = NULL;
				output->contactPatches = NULL;
				output->contactPoints = NULL;
			}
		}

		if (contactsOverflown)
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Contact buffer overflow detected, please increase its size in the scene desc!\n");
		else if (patchesOverflown)
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Patch buffer overflow detected, please increase its size in the scene desc!\n");
		else if (forcesOverflown)
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Force buffer overflow detected, please increase its size in the scene desc!\n");

	}

	//mGpuNarrowphaseCore->appendContactManagers(mContext.getEventProfiler(), outputs, mNbFallbackPairs);

	//mContactManagerOutputs.unmap();
	//mGpuNarrowphaseCore->appendContactManagersConvexTriMesh(mContext.getEventProfiler());
	//mGpuNarrowphaseCore->waitAndResetCopyQueues();

	processResults();
	mContext.getCudaContextManager()->releaseContext();
}

PxsContactManagerOutput* PxgNphaseImplementationContext::getGPUContactManagerOutputBase()
{
	return mGpuNarrowphaseCore->getGPUContactManagerOutputBase();
}

PxReal* PxgNphaseImplementationContext::getGPURestDistances()
{
	return mGpuNarrowphaseCore->getGPURestDistances();
}
Sc::ShapeInteraction** PxgNphaseImplementationContext::getGPUShapeInteractions()
{
	return mGpuNarrowphaseCore->getGPUShapeInteractions();
}

PxsTorsionalFrictionData* PxgNphaseImplementationContext::getGPUTorsionalData()
{
	return mGpuNarrowphaseCore->getGPUTorsionalData();
}

static void prepGpuContactManagerOutput(PxsContactManagerOutput& output, const PxcNpWorkUnit& workUnit, PxU32 patchCount, PxI32 touching)
{
	output.nbPatches = PxTo8(patchCount);
	output.prevPatches = 0;
	output.nbContacts = 0;
	output.contactForces = NULL;
	output.contactPatches = NULL;
	output.contactPoints = NULL;
	output.frictionPatches = NULL;
	output.flags = workUnit.mFlags;

	PxU8 statusFlag = 0;

	if(workUnit.mFlags & PxcNpWorkUnitFlag::eOUTPUT_CONSTRAINTS)
		statusFlag |= PxsContactManagerStatusFlag::eREQUEST_CONSTRAINTS;

	const bool staticOrKinematic = workUnit.mFlags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR || 
		!(workUnit.mFlags & (PxcNpWorkUnitFlag::eDYNAMIC_BODY0 | PxcNpWorkUnitFlag::eDYNAMIC_BODY1 | PxcNpWorkUnitFlag::eSOFT_BODY));

	//This flag is used by the particle system to figure out whether it can output simple or complex (2-way) collision structures
	if (staticOrKinematic)
		statusFlag |= PxsContactManagerStatusFlag::eSTATIC_OR_KINEMATIC;

	if (touching > 0)
	{
		statusFlag |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	}
	else if (touching < 0)
	{
		statusFlag |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;
	}

	output.statusFlag = statusFlag;
}

void PxgNphaseImplementationContext::preallocateNewBuffers(PxU32 nbNewPairs, PxU32 maxIndex)
{
	if (maxIndex >= mGpuContactManagerBitMap[0].size())
	{
		PxU32 newSize = (2 * (maxIndex + 256));
		for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
		{
			mGpuContactManagerBitMap[i].resize(newSize);
		}
	}

	mGpuNarrowphaseCore->preallocateNewBuffers(nbNewPairs);
}

static PX_FORCE_INLINE bool isDynamicMesh(const PxsShapeCore* shapeCore)
{
	const PxGeometry& geom = shapeCore->mGeometry.getGeometry();
	const Gu::TriangleMesh* mesh = static_cast<const Gu::TriangleMesh*>(static_cast<const PxTriangleMeshGeometry&>(geom).triangleMesh);
	return mesh->getSdfDataFast().mSdf != NULL;
}

static PX_FORCE_INLINE bool isGpuMesh(PxgGpuNarrowphaseCore* narrowPhaseCore, const PxsShapeCore* shapeCore)
{
	const PxGeometry& geom = shapeCore->mGeometry.getGeometry();
	return narrowPhaseCore->isMeshGPUCompatible(static_cast<const PxTriangleMeshGeometryLL&>(geom));
}

PX_FORCE_INLINE void PxgNphaseImplementationContext::registerContactManagerInternal(PxsContactManager* cm, const PxcNpWorkUnit& workUnit, PxU32 patchCount,
	PxI32 touching, const Sc::ShapeInteraction* shapeInteraction, GPU_BUCKET_ID::Enum bucketId)
{
	PxsContactManagerOutput output;
	prepGpuContactManagerOutput(output, workUnit, patchCount, touching);

	mGpuNarrowphaseCore->registerContactManager(cm, shapeInteraction, output, bucketId);

	mGpuContactManagerBitMap[bucketId].set(cm->getIndex());
	mNbPairCount[bucketId]++;
}

namespace
{
	// PT: using a struct to limit the amount of refactoring when we add a parameter, and the amount of PX_UNUSED we have to write.
	// For now we don't try to be clever, we just pass all that data to all the functions.
	struct gNpParams
	{
		PxgGpuNarrowphaseCore*	mGpuNPCore;
		const PxsShapeCore*		mShapeCore0;
		const PxsShapeCore*		mShapeCore1;
		PxGeometryType::Enum	mType0;
		PxGeometryType::Enum	mType1;
		bool					mIsRigid0;
		bool					mIsRigid1;

		PX_FORCE_INLINE	bool	isMeshGPUCompatible0()	const
		{
			return mGpuNPCore->isMeshGPUCompatible(mShapeCore0->mGeometry.get<const PxTriangleMeshGeometryLL>());
		}

		PX_FORCE_INLINE	bool	isMeshGPUCompatible1()	const
		{
			return mGpuNPCore->isMeshGPUCompatible(mShapeCore1->mGeometry.get<const PxTriangleMeshGeometryLL>());
		}
	};
}

static PX_FORCE_INLINE bool isConvexCoreGpuCompatible(const PxsShapeCore* shapeCore)
{
	return Gu::isGPUCompatible(shapeCore->mGeometry.get<const PxConvexCoreGeometry>());
}

static PX_FORCE_INLINE bool isConvexGpuCompatible(const PxsShapeCore* shapeCore)
{
	return shapeCore->mGeometry.get<const PxConvexMeshGeometryLL>().gpuCompatible;
}

typedef GPU_BUCKET_ID::Enum (*internalMethod)(const gNpParams&);

// PT: generic function when the pair of types unconditionally maps to a bucket
static PX_FORCE_INLINE GPU_BUCKET_ID::Enum bucket(const gNpParams&, GPU_BUCKET_ID::Enum id)
{
	return id;
}

static GPU_BUCKET_ID::Enum cndCasePlaneConvexCore(const gNpParams& p)
{
	PX_ASSERT(p.mType0 == PxGeometryType::ePLANE);
	PX_ASSERT(p.mType1 == PxGeometryType::eCONVEXCORE);

	if (isConvexCoreGpuCompatible(p.mShapeCore1))
		return GPU_BUCKET_ID::eConvexCorePlane;

	return GPU_BUCKET_ID::eFallback;
}

// PT: then conditional functions where things aren't so simple

static GPU_BUCKET_ID::Enum cndCasePlaneConvex(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::ePLANE);
	PX_ASSERT(p.mType1==PxGeometryType::eCONVEXMESH);

	if(isConvexGpuCompatible(p.mShapeCore1))
		return GPU_BUCKET_ID::eConvexPlane;
	
	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCasePrimitiveConvex(const gNpParams& p)
{
	PX_ASSERT(p.mType0<=PxGeometryType::eCONVEXMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eCONVEXMESH);

	const bool isShape0Primitive =  (p.mShapeCore0->mGeometry.getType() < PxGeometryType::eCONVEXMESH);
	const bool isShape0GpuCompatible = isShape0Primitive || isConvexGpuCompatible(p.mShapeCore0);
	const bool isShape1GpuCompatible = isConvexGpuCompatible(p.mShapeCore1);

	if(isShape0GpuCompatible && isShape1GpuCompatible)
		return GPU_BUCKET_ID::eConvex;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexCoreConvex(const gNpParams& p)
{
	PX_ASSERT(p.mType0 <= PxGeometryType::eCONVEXMESH && p.mType0 != PxGeometryType::ePLANE);
	PX_ASSERT(p.mType1 <= PxGeometryType::eCONVEXMESH && p.mType1 != PxGeometryType::ePLANE);

	if (p.mType0 == PxGeometryType::eCONVEXCORE && !isConvexCoreGpuCompatible(p.mShapeCore0))
		return GPU_BUCKET_ID::eFallback;

	if (p.mType1 == PxGeometryType::eCONVEXCORE && !isConvexCoreGpuCompatible(p.mShapeCore1))
		return GPU_BUCKET_ID::eFallback;

	if (p.mType1 == PxGeometryType::eCONVEXMESH && !isConvexGpuCompatible(p.mShapeCore1))
		return GPU_BUCKET_ID::eFallback;

	return GPU_BUCKET_ID::eConvexCoreConvex;
}

static GPU_BUCKET_ID::Enum cndCaseConvexCoreTrimesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0 == PxGeometryType::eCONVEXCORE);
	PX_ASSERT(p.mType1 == PxGeometryType::eTRIANGLEMESH);

	if (!p.mIsRigid1) // a cloth
	{

		if (isConvexCoreGpuCompatible(p.mShapeCore0))
			return GPU_BUCKET_ID::eConvexCoreClothmesh;

		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible convex core geometry is not able to collide with cloth.\n");
		return GPU_BUCKET_ID::eFallback;
	}

	if (isConvexCoreGpuCompatible(p.mShapeCore0) && p.isMeshGPUCompatible1())
		return GPU_BUCKET_ID::eConvexCoreTrimesh;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexCoreHeightfield(const gNpParams& p)
{
	PX_ASSERT(p.mType0 == PxGeometryType::eCONVEXCORE);
	PX_ASSERT(p.mType1 == PxGeometryType::eHEIGHTFIELD);

	PX_UNUSED(p);
	//if (isConvexCoreGpuCompatible(p.mShapeCore0) && p.isMeshGPUCompatible1())
	//	return GPU_BUCKET_ID::eConvexCoreHeightfield;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexCoreTetra(const gNpParams& p)
{
	PX_ASSERT(p.mType0 == PxGeometryType::eCONVEXCORE);
	PX_ASSERT(p.mType1 == PxGeometryType::eTETRAHEDRONMESH);

	if (isConvexCoreGpuCompatible(p.mShapeCore0))
		return GPU_BUCKET_ID::eConvexCoreTetmesh;

	outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible convex core geometry is not able to collide with deformable volume tetrahedron mesh.\n");

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexCoreParticle(const gNpParams& p)
{
	PX_ASSERT(p.mType0 == PxGeometryType::eCONVEXCORE);
	PX_ASSERT(p.mType1 == PxGeometryType::ePARTICLESYSTEM);

	if (isConvexCoreGpuCompatible(p.mShapeCore0))
		return GPU_BUCKET_ID::eParticlesystems;

	outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible convex core geometry is not able to collide with particle system.\n");

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexCoreNotImplemented(const gNpParams& p)
{
	PX_UNUSED(p);
	PX_ASSERT(p.mType0 == PxGeometryType::eCONVEXCORE || p.mType1 == PxGeometryType::eCONVEXCORE);
	outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Not implemented yet.\n");
	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseSphereMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eSPHERE);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	const bool isCloth1 = !p.mIsRigid1;
	if(isCloth1)
		return GPU_BUCKET_ID::eFemClothSphere;

	if(p.isMeshGPUCompatible1())
		return GPU_BUCKET_ID::eSphereTrimesh;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCasePlaneMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::ePLANE);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	const bool isCloth1 = !p.mIsRigid1;
	return isCloth1 ? GPU_BUCKET_ID::eFemClothPlane : GPU_BUCKET_ID::eTrianglePlane;
}

static GPU_BUCKET_ID::Enum cndCaseCapsuleMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eCAPSULE);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	const bool isCloth1 = !p.mIsRigid1;
	if(isCloth1)
		return GPU_BUCKET_ID::eFemClothSphere;

	if(p.isMeshGPUCompatible1())
		return GPU_BUCKET_ID::eSphereTrimesh;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseBoxMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eBOX);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	const bool isCloth1 = !p.mIsRigid1;
	if(isCloth1)
		return GPU_BUCKET_ID::eFemClothBox;

	if(p.isMeshGPUCompatible1())
		return GPU_BUCKET_ID::eConvexTrimesh;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	const bool type0supported = isConvexGpuCompatible(p.mShapeCore0);
	const bool isCloth1 = !p.mIsRigid1;
	if(isCloth1)
	{
		if (type0supported)
		{
			return GPU_BUCKET_ID::eFemClothConvexes;
		}
		else
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible convex mesh is not able to collide with cloth.\n");
			return GPU_BUCKET_ID::eFallback;
		}
	}

	if(type0supported && p.isMeshGPUCompatible1())
		return GPU_BUCKET_ID::eConvexTrimesh;

	return GPU_BUCKET_ID::eFallback;
}

static PX_FORCE_INLINE GPU_BUCKET_ID::Enum cndCaseMeshMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	// PT: TODO: this returns true if a SDF is present, which is a bit confusing - can't we use SDFs for static meshes?
	const bool isDynamic0 = isDynamicMesh(p.mShapeCore0);
	const bool isDynamic1 = isDynamicMesh(p.mShapeCore1);

	const bool isRigidBodyMesh0 = p.mIsRigid0 && isGpuMesh(p.mGpuNPCore, p.mShapeCore0);
	const bool isRigidBodyMesh1 = p.mIsRigid1 && isGpuMesh(p.mGpuNPCore, p.mShapeCore1);

	const bool isValidTriTriPair = isRigidBodyMesh0 && isRigidBodyMesh1 && (isDynamic0 || isDynamic1);
	if(isValidTriTriPair)
		return GPU_BUCKET_ID::eTriangleTriangle;

	/////

	const bool isCloth0 = !p.mIsRigid0;
	const bool isCloth1 = !p.mIsRigid1;
	if(isCloth0 &&  isCloth1)
		return GPU_BUCKET_ID::eFemClothes;

	/////

	if(isCloth1)
	{
		const bool isRigidBodySDFMesh0 = isRigidBodyMesh0 && isDynamic0;
		if(isRigidBodySDFMesh0)
			return GPU_BUCKET_ID::eFemClothSdfTrimesh;
	}

	if(isCloth0)
	{
		const bool isRigidBodySDFMesh1 = isRigidBodyMesh1 && isDynamic1;
		if(isRigidBodySDFMesh1)
			return GPU_BUCKET_ID::eFemClothSdfTrimesh;
	}

	/////

	// PT: TODO: copied from original code, maybe could be simplified
	if((!isCloth0 && isCloth1) || (!isCloth1 && isCloth0))
	{
		if((!isCloth0 && p.mGpuNPCore->isClothMeshGPUCompatible(p.mShapeCore1->mGeometry.get<const PxTriangleMeshGeometryLL>())) ||
			(!isCloth1 && p.mGpuNPCore->isClothMeshGPUCompatible(p.mShapeCore0->mGeometry.get<const PxTriangleMeshGeometryLL>())))
		{
			return GPU_BUCKET_ID::eFemClothTrimesh;
		}
		else
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible triangle mesh is not able to collide with cloth.\n");
	}

	return GPU_BUCKET_ID::eFallback;
}

static PX_FORCE_INLINE GPU_BUCKET_ID::Enum cndCaseMeshCustom(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eCUSTOM);

	const bool isCloth0 = !p.mIsRigid0;
	
	//Cloth currently cannot collide against a custom geometry - there is a filter in filterRbCollisionPairShared
	PX_ASSERT(!isCloth0);
	PX_UNUSED(isCloth0);

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexHeightfield(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eHEIGHTFIELD);

	if(isConvexGpuCompatible(p.mShapeCore0))
		return GPU_BUCKET_ID::eConvexHeightfield;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCasePrimitiveParticle(const gNpParams& p)
{
	PX_UNUSED(p);
	PX_ASSERT(p.mType0<PxGeometryType::eCONVEXMESH);
	PX_ASSERT(p.mType1==PxGeometryType::ePARTICLESYSTEM);
	return GPU_BUCKET_ID::eParticlesystems;
}

static GPU_BUCKET_ID::Enum cndCaseConvexParticle(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(p.mType1==PxGeometryType::ePARTICLESYSTEM);

	if(isConvexGpuCompatible(p.mShapeCore0))
		return GPU_BUCKET_ID::eConvexParticle;
	else
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible convex mesh is not able to collide with particle system.\n");

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseParticleTetra(const gNpParams& p)
{
	PX_UNUSED(p);
	PX_ASSERT(p.mType0==PxGeometryType::ePARTICLESYSTEM);
	PX_ASSERT(p.mType1==PxGeometryType::eTETRAHEDRONMESH);
	return GPU_BUCKET_ID::eParticlesystemSoftbody;
}

static GPU_BUCKET_ID::Enum cndCaseParticleMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::ePARTICLESYSTEM);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	// PT: copied from original code but don't ask me
	const bool isRigidBodyMesh = p.mIsRigid1 && isGpuMesh(p.mGpuNPCore, p.mShapeCore1);
	const bool isRigidBodySDFMesh = isRigidBodyMesh && isDynamicMesh(p.mShapeCore1);

	if(p.mGpuNPCore->isClothMeshGPUCompatible(p.mShapeCore1->mGeometry.get<const PxTriangleMeshGeometryLL>()))
	{
		GPU_BUCKET_ID::Enum bucketId = GPU_BUCKET_ID::eParticlesystemFemCloth;
		const bool isCloth1 = !p.mIsRigid1;
		if(isCloth1)
			bucketId = GPU_BUCKET_ID::eParticlesystemFemCloth;				
		else if (p.mShapeCore1->mGeometry.get<const PxTriangleMeshGeometryLL>().materialsLL.numIndices <= 1) // rigid body
		{
			if (isRigidBodySDFMesh)
				bucketId = GPU_BUCKET_ID::eParticlesystemSdfTrimesh;
			else
				bucketId = GPU_BUCKET_ID::eParticlesystemTrimesh;
		}
		return bucketId;
	}
	else
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible triangle mesh is not able to collide with particle system.\n");
	}
	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseParticleCustom(const gNpParams& p)
{
	PX_UNUSED(p);
	PX_ASSERT(p.mType0==PxGeometryType::ePARTICLESYSTEM);
	PX_ASSERT(p.mType1==PxGeometryType::eCUSTOM);
	//Error message already generated in filterRbCollisionPairShared
	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseConvexTetra(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eTETRAHEDRONMESH);

	if(!isConvexGpuCompatible(p.mShapeCore0))
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible convex mesh is not able to collide with deformable volume tetrahedron mesh.\n");
		return GPU_BUCKET_ID::eFallback;
	}
	return GPU_BUCKET_ID::eSoftbody;
}

static GPU_BUCKET_ID::Enum cndCaseTetraMesh(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eTETRAHEDRONMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eTRIANGLEMESH);

	const bool isRigidBodyMesh = p.mIsRigid1 && isGpuMesh(p.mGpuNPCore, p.mShapeCore1);
	const bool isRigidBodySDFMesh = isRigidBodyMesh && isDynamicMesh(p.mShapeCore1);

	if(p.mGpuNPCore->isClothMeshGPUCompatible(p.mShapeCore1->mGeometry.get<const PxTriangleMeshGeometryLL>()))
	{
		const bool isCloth1 = !p.mIsRigid1;
		if(isCloth1)
		{
			return GPU_BUCKET_ID::eSoftbodyFemCloth;
		}
		else if(p.mShapeCore1->mGeometry.get<const PxTriangleMeshGeometryLL>().materialsLL.numIndices <= 1) // rigid body
		{
			if (isRigidBodySDFMesh)
				return GPU_BUCKET_ID::eSoftbodySdfTrimesh;
			else
				return GPU_BUCKET_ID::eSoftbodyTrimesh;
		}
	}
	else
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Non-GPU-compatible triangle mesh is not able to collide with deformable volume tetrahedron mesh.\n");
	}
	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseTetraCustom(const gNpParams& p)
{
	PX_UNUSED(p);
	PX_ASSERT(p.mType0==PxGeometryType::eTETRAHEDRONMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eCUSTOM);
	//Error message already generated in filterRbCollisionPairShared
	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum cndCaseMeshHeightfield(const gNpParams& p)
{
	PX_ASSERT(p.mType0==PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(p.mType1==PxGeometryType::eHEIGHTFIELD);

	const bool isCloth0 = !p.mIsRigid0;
	if(isCloth0)
		return GPU_BUCKET_ID::eFemClothHeightfield;

	return GPU_BUCKET_ID::eFallback;
}

static GPU_BUCKET_ID::Enum caseSphereSphere(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSphere);				}
static GPU_BUCKET_ID::Enum caseSpherePlane(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSphere);				}
static GPU_BUCKET_ID::Enum caseSphereCapsule(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eSphere);				}
static GPU_BUCKET_ID::Enum caseSphereBox(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSphere);				}
static GPU_BUCKET_ID::Enum caseSphereConvex(const gNpParams& p)			{ return cndCasePrimitiveConvex(p);						}
static GPU_BUCKET_ID::Enum caseSphereParticle(const gNpParams& p)		{ return cndCasePrimitiveParticle(p);					}
static GPU_BUCKET_ID::Enum caseSphereTetra(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSoftbody);			}
static GPU_BUCKET_ID::Enum caseSphereMesh(const gNpParams& p)			{ return cndCaseSphereMesh(p);							}
static GPU_BUCKET_ID::Enum caseSphereHeightfield(const gNpParams& p)	{ return bucket(p, GPU_BUCKET_ID::eSphereHeightfield);	}
static GPU_BUCKET_ID::Enum caseSphereCustom(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eFallback);			}

static GPU_BUCKET_ID::Enum casePlanePlane(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eFallback);		}
static GPU_BUCKET_ID::Enum casePlaneCapsule(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSphere);			}
static GPU_BUCKET_ID::Enum casePlaneBox(const gNpParams& p)				{ return bucket(p, GPU_BUCKET_ID::eConvexPlane);	}
static GPU_BUCKET_ID::Enum casePlaneConvexCore(const gNpParams& p)		{ return cndCasePlaneConvexCore(p);					}
static GPU_BUCKET_ID::Enum casePlaneConvex(const gNpParams& p)			{ return cndCasePlaneConvex(p);						}
static GPU_BUCKET_ID::Enum casePlaneParticle(const gNpParams& p)		{ return cndCasePrimitiveParticle(p);				}
static GPU_BUCKET_ID::Enum casePlaneTetra(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSoftbody);		}
static GPU_BUCKET_ID::Enum casePlaneMesh(const gNpParams& p)			{ return cndCasePlaneMesh(p);						}
static GPU_BUCKET_ID::Enum casePlaneHeightfield(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eFallback);		}
static GPU_BUCKET_ID::Enum casePlaneCustom(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eFallback);		}

static GPU_BUCKET_ID::Enum caseCapsuleCapsule(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eSphere);				}
static GPU_BUCKET_ID::Enum caseCapsuleBox(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eConvex);				}
static GPU_BUCKET_ID::Enum caseCapsuleConvex(const gNpParams& p)		{ return cndCasePrimitiveConvex(p);						}
static GPU_BUCKET_ID::Enum caseCapsuleParticle(const gNpParams& p)		{ return cndCasePrimitiveParticle(p);					}
static GPU_BUCKET_ID::Enum caseCapsuleTetra(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSoftbody);			}
static GPU_BUCKET_ID::Enum caseCapsuleMesh(const gNpParams& p)			{ return cndCaseCapsuleMesh(p);							}
static GPU_BUCKET_ID::Enum caseCapsuleHeightfield(const gNpParams& p)	{ return bucket(p, GPU_BUCKET_ID::eSphereHeightfield);	}
static GPU_BUCKET_ID::Enum caseCapsuleCustom(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eFallback);			}

static GPU_BUCKET_ID::Enum caseBoxBox(const gNpParams& p)				{ return bucket(p, GPU_BUCKET_ID::eBoxBox);				}
static GPU_BUCKET_ID::Enum caseBoxConvex(const gNpParams& p)			{ return cndCasePrimitiveConvex(p);						}
static GPU_BUCKET_ID::Enum caseBoxParticle(const gNpParams& p)			{ return cndCasePrimitiveParticle(p);					}
static GPU_BUCKET_ID::Enum caseBoxTetra(const gNpParams& p)				{ return bucket(p, GPU_BUCKET_ID::eSoftbody);			}
static GPU_BUCKET_ID::Enum caseBoxMesh(const gNpParams& p)				{ return cndCaseBoxMesh(p);								}
static GPU_BUCKET_ID::Enum caseBoxHeightfield(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eConvexHeightfield);	}
static GPU_BUCKET_ID::Enum caseBoxCustom(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eFallback);			}

static GPU_BUCKET_ID::Enum caseConvexCoreConvex(const gNpParams& p)		{ return cndCaseConvexCoreConvex(p);			}
static GPU_BUCKET_ID::Enum caseConvexCoreTrimesh(const gNpParams& p)	{ return cndCaseConvexCoreTrimesh(p);			}
static GPU_BUCKET_ID::Enum caseConvexCoreHeightfield(const gNpParams& p){ return cndCaseConvexCoreHeightfield(p);		}
static GPU_BUCKET_ID::Enum caseConvexCoreTetra(const gNpParams& p)		{ return cndCaseConvexCoreTetra(p);				}
static GPU_BUCKET_ID::Enum caseConvexCoreParticle(const gNpParams& p)	{ return cndCaseConvexCoreParticle(p);			}
static GPU_BUCKET_ID::Enum caseConvexCoreNotImplemented(const gNpParams& p){ return cndCaseConvexCoreNotImplemented(p);	}

static GPU_BUCKET_ID::Enum caseConvexConvex(const gNpParams& p)			{ return cndCasePrimitiveConvex(p);				}
static GPU_BUCKET_ID::Enum caseConvexParticle(const gNpParams& p)		{ return cndCaseConvexParticle(p);				}
static GPU_BUCKET_ID::Enum caseConvexTetra(const gNpParams& p)			{ return cndCaseConvexTetra(p);					}
static GPU_BUCKET_ID::Enum caseConvexMesh(const gNpParams& p)			{ return cndCaseConvexMesh(p);					}
static GPU_BUCKET_ID::Enum caseConvexHeightfield(const gNpParams& p)	{ return cndCaseConvexHeightfield(p);			}
static GPU_BUCKET_ID::Enum caseConvexCustom(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eFallback);	}

static GPU_BUCKET_ID::Enum caseParticleParticle(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eFallback);					}
static GPU_BUCKET_ID::Enum caseParticleTetra(const gNpParams& p)		{ return cndCaseParticleTetra(p);								}
static GPU_BUCKET_ID::Enum caseParticleMesh(const gNpParams& p)			{ return cndCaseParticleMesh(p);								}
static GPU_BUCKET_ID::Enum caseParticleHeightfield(const gNpParams& p)	{ return bucket(p, GPU_BUCKET_ID::eParticlesystemHeightfield);	}
static GPU_BUCKET_ID::Enum caseParticleCustom(const gNpParams& p)		{ return cndCaseParticleCustom(p);								}

static GPU_BUCKET_ID::Enum caseTetraTetra(const gNpParams& p)			{ return bucket(p, GPU_BUCKET_ID::eSoftbodies);				}
static GPU_BUCKET_ID::Enum caseTetraMesh(const gNpParams& p)			{ return cndCaseTetraMesh(p);								}
static GPU_BUCKET_ID::Enum caseTetraHeightfield(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eSoftbodyHeightfield);	}
static GPU_BUCKET_ID::Enum caseTetraCustom(const gNpParams& p)			{ return cndCaseTetraCustom(p);								}

static GPU_BUCKET_ID::Enum caseMeshMesh(const gNpParams& p)				{ return cndCaseMeshMesh(p);		}
static GPU_BUCKET_ID::Enum caseMeshHeightfield(const gNpParams& p)		{ return cndCaseMeshHeightfield(p);	}
static GPU_BUCKET_ID::Enum caseMeshCustom(const gNpParams& p)			{ return cndCaseMeshCustom(p);		}

static GPU_BUCKET_ID::Enum caseHeightfieldHeightfield(const gNpParams& p)	{ return bucket(p, GPU_BUCKET_ID::eFallback);	}
static GPU_BUCKET_ID::Enum caseHeightfieldCustom(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eFallback);	}

static GPU_BUCKET_ID::Enum caseCustomCustom(const gNpParams& p)		{ return bucket(p, GPU_BUCKET_ID::eFallback);	}

static internalMethod gTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		caseSphereSphere,		//PxGeometryType::eSPHERE
		caseSpherePlane,		//PxGeometryType::ePLANE
		caseSphereCapsule,		//PxGeometryType::eCAPSULE
		caseSphereBox,			//PxGeometryType::eBOX
		caseConvexCoreConvex,	//PxGeometryType::eCONVEXCORE
		caseSphereConvex,		//PxGeometryType::eCONVEXMESH
		caseSphereParticle,		//PxGeometryType::ePARTICLESYSTEM
		caseSphereTetra,		//PxGeometryType::eTETRAHEDRONMESH
		caseSphereMesh,			//PxGeometryType::eTRIANGLEMESH
		caseSphereHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseSphereCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePLANE
	{
		NULL,					//PxGeometryType::eSPHERE
		casePlanePlane,			//PxGeometryType::ePLANE
		casePlaneCapsule,		//PxGeometryType::eCAPSULE
		casePlaneBox,			//PxGeometryType::eBOX
		casePlaneConvexCore,	//PxGeometryType::eCONVEXCORE
		casePlaneConvex,		//PxGeometryType::eCONVEXMESH
		casePlaneParticle,		//PxGeometryType::ePARTICLESYSTEM
		casePlaneTetra,			//PxGeometryType::eTETRAHEDRONMESH
		casePlaneMesh,			//PxGeometryType::eTRIANGLEMESH
		casePlaneHeightfield,	//PxGeometryType::eHEIGHTFIELD
		casePlaneCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCAPSULE
	{
		NULL,					//PxGeometryType::eSPHERE
		NULL,					//PxGeometryType::ePLANE
		caseCapsuleCapsule,		//PxGeometryType::eCAPSULE
		caseCapsuleBox,			//PxGeometryType::eBOX
		caseConvexCoreConvex,	//PxGeometryType::eCONVEXCORE
		caseCapsuleConvex,		//PxGeometryType::eCONVEXMESH
		caseCapsuleParticle,	//PxGeometryType::ePARTICLESYSTEM
		caseCapsuleTetra,		//PxGeometryType::eTETRAHEDRONMESH
		caseCapsuleMesh,		//PxGeometryType::eTRIANGLEMESH
		caseCapsuleHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseCapsuleCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eBOX
	{
		NULL,				//PxGeometryType::eSPHERE
		NULL,				//PxGeometryType::ePLANE
		NULL,				//PxGeometryType::eCAPSULE
		caseBoxBox,			//PxGeometryType::eBOX
		caseConvexCoreConvex,//PxGeometryType::eCONVEXCORE
		caseBoxConvex,		//PxGeometryType::eCONVEXMESH
		caseBoxParticle,	//PxGeometryType::ePARTICLESYSTEM
		caseBoxTetra,		//PxGeometryType::eTETRAHEDRONMESH
		caseBoxMesh,		//PxGeometryType::eTRIANGLEMESH
		caseBoxHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseBoxCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXCORE
	{
		NULL,						//PxGeometryType::eSPHERE
		NULL,						//PxGeometryType::ePLANE
		NULL,						//PxGeometryType::eCAPSULE
		NULL,						//PxGeometryType::eBOX
		caseConvexCoreConvex,		//PxGeometryType::eCONVEXCORE
		caseConvexCoreConvex,		//PxGeometryType::eCONVEXMESH
		caseConvexCoreParticle,		//PxGeometryType::ePARTICLESYSTEM
		caseConvexCoreTetra,		//PxGeometryType::eTETRAHEDRONMESH
		caseConvexCoreTrimesh,		//PxGeometryType::eTRIANGLEMESH
		caseConvexCoreHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseConvexCoreNotImplemented,//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXMESH
	{
		NULL,					//PxGeometryType::eSPHERE
		NULL,					//PxGeometryType::ePLANE
		NULL,					//PxGeometryType::eCAPSULE
		NULL,					//PxGeometryType::eBOX
		NULL,					//PxGeometryType::eCONVEXCORE
		caseConvexConvex,		//PxGeometryType::eCONVEXMESH
		caseConvexParticle,		//PxGeometryType::ePARTICLESYSTEM
		caseConvexTetra,		//PxGeometryType::eTETRAHEDRONMESH
		caseConvexMesh,			//PxGeometryType::eTRIANGLEMESH
		caseConvexHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseConvexCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePARTICLESYSTEM
	{
		NULL,						//PxGeometryType::eSPHERE
		NULL,						//PxGeometryType::ePLANE
		NULL,						//PxGeometryType::eCAPSULE
		NULL,						//PxGeometryType::eBOX
		NULL,						//PxGeometryType::eCONVEXCORE
		NULL,						//PxGeometryType::eCONVEXMESH
		caseParticleParticle,		//PxGeometryType::ePARTICLESYSTEM
		caseParticleTetra,			//PxGeometryType::eTETRAHEDRONMESH
		caseParticleMesh,			//PxGeometryType::eTRIANGLEMESH
		caseParticleHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseParticleCustom,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTETRAHEDRONMESH
	{
		NULL,					//PxGeometryType::eSPHERE
		NULL,					//PxGeometryType::ePLANE
		NULL,					//PxGeometryType::eCAPSULE
		NULL,					//PxGeometryType::eBOX
		NULL,					//PxGeometryType::eCONVEXCORE
		NULL,					//PxGeometryType::eCONVEXMESH
		NULL,					//PxGeometryType::ePARTICLESYSTEM
		caseTetraTetra,			//PxGeometryType::eTETRAHEDRONMESH
		caseTetraMesh,			//PxGeometryType::eTRIANGLEMESH
		caseTetraHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseTetraCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		NULL,					//PxGeometryType::eSPHERE
		NULL,					//PxGeometryType::ePLANE
		NULL,					//PxGeometryType::eCAPSULE
		NULL,					//PxGeometryType::eBOX
		NULL,					//PxGeometryType::eCONVEXCORE
		NULL,					//PxGeometryType::eCONVEXMESH
		NULL,					//PxGeometryType::ePARTICLESYSTEM
		NULL,					//PxGeometryType::eTETRAHEDRONMESH
		caseMeshMesh,			//PxGeometryType::eTRIANGLEMESH
		caseMeshHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseMeshCustom,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		NULL,						//PxGeometryType::eSPHERE
		NULL,						//PxGeometryType::ePLANE
		NULL,						//PxGeometryType::eCAPSULE
		NULL,						//PxGeometryType::eBOX
		NULL,						//PxGeometryType::eCONVEXCORE
		NULL,						//PxGeometryType::eCONVEXMESH
		NULL,						//PxGeometryType::ePARTICLESYSTEM
		NULL,						//PxGeometryType::eTETRAHEDRONMESH
		NULL,						//PxGeometryType::eTRIANGLEMESH
		caseHeightfieldHeightfield,	//PxGeometryType::eHEIGHTFIELD
		caseHeightfieldCustom,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCUSTOM
	{
		NULL,				//PxGeometryType::eSPHERE
		NULL,				//PxGeometryType::ePLANE
		NULL,				//PxGeometryType::eCAPSULE
		NULL,				//PxGeometryType::eBOX
		NULL,				//PxGeometryType::eCONVEXCORE
		NULL,				//PxGeometryType::eCONVEXMESH
		NULL,				//PxGeometryType::ePARTICLESYSTEM
		NULL,				//PxGeometryType::eTETRAHEDRONMESH
		NULL,				//PxGeometryType::eTRIANGLEMESH
		NULL,				//PxGeometryType::eHEIGHTFIELD
		caseCustomCustom,	//PxGeometryType::eCUSTOM
	},
};
PX_COMPILE_TIME_ASSERT(sizeof(gTable) / sizeof(gTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

void PxgNphaseImplementationContext::registerContactManager(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxI32 touching, PxU32 patchCount)
{
	mTotalNbPairs++;

	if(cm->getIndex() >= mGpuContactManagerBitMap[0].size())
	{
		const PxU32 newSize = (2*cm->getIndex()+256)&~255;
		for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
		{
			mGpuContactManagerBitMap[i].resize(newSize);
		}
	}

	///////////////////////////

	PxcNpWorkUnit& workUnit = cm->getWorkUnit();

	const PxsShapeCore* shapeCore0 = workUnit.getShapeCore0();
	const PxsShapeCore* shapeCore1 = workUnit.getShapeCore1();

	PX_ASSERT(shapeCore0);
	PX_ASSERT(shapeCore1);

	PxGeometryType::Enum type0 = shapeCore0->mGeometry.getType();
	PxGeometryType::Enum type1 = shapeCore1->mGeometry.getType();

	bool isRigid0 = workUnit.mRigidCore0 ? true : false;
	bool isRigid1 = workUnit.mRigidCore1 ? true : false;

	const bool flip = (type1 < type0);
	if(flip)
	{
		PxSwap(type0, type1);
		PxSwap(shapeCore0, shapeCore1);
		PxSwap(isRigid0, isRigid1);
	}

	///////////////////////////

	GPU_BUCKET_ID::Enum bucketId = GPU_BUCKET_ID::eFallback;

	const bool hasModification = (workUnit.mFlags & PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT) != 0;
	const bool gpuCollision = mContext.getPCM() && !hasModification && PXG_ENABLE_GPU_NP && (workUnit.mFlags & PxcNpWorkUnitFlag::eDETECT_DISCRETE_CONTACT);
	if(gpuCollision)
	{
		const internalMethod func = gTable[type0][type1];
		PX_ASSERT(func);

		gNpParams p;
		p.mGpuNPCore	= mGpuNarrowphaseCore;
		p.mShapeCore0	= shapeCore0;
		p.mShapeCore1	= shapeCore1;
		p.mType0		= type0;
		p.mType1		= type1;
		p.mIsRigid0		= isRigid0;
		p.mIsRigid1		= isRigid1;
		bucketId = func(p);
	}

	if(bucketId!=GPU_BUCKET_ID::eFallback)
	{
		registerContactManagerInternal(cm, workUnit, patchCount, touching, shapeInteraction, bucketId);
	}
	else
	{
		mFallbackForUnsupportedCMs->registerContactManager(cm, shapeInteraction, touching, patchCount);
		mGpuContactManagerBitMap[GPU_BUCKET_ID::eFallback].set(cm->getIndex());
		mNbPairCount[GPU_BUCKET_ID::eFallback]++;
	}
}

void PxgNphaseImplementationContext::unregisterContactManager(PxsContactManager* cm)
{
	mTotalNbPairs--;

	if (mGpuContactManagerBitMap[0].test(cm->getIndex()))
	{
		mFallbackForUnsupportedCMs->unregisterContactManagerFallback(cm, mContactManagerOutputs.begin());
		mGpuContactManagerBitMap[0].reset(cm->getIndex());
		mNbPairCount[0]--;
	}
	else
	{
		for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
		{
			if (mGpuContactManagerBitMap[i].test(cm->getIndex()))
			{
				mGpuNarrowphaseCore->unregisterContactManager(cm, i);
				mGpuContactManagerBitMap[i].reset(cm->getIndex());
				mNbPairCount[i]--;
				break;
			}
		}
	}
}

void PxgNphaseImplementationContext::refreshContactManager(PxsContactManager* cm)
{
	PxcNpWorkUnit& workUnit = cm->getWorkUnit();
	PxgContactManagerInput input;
	input.shapeRef0 = mGpuNarrowphaseCore->getShapeIndex(*workUnit.getShapeCore0());
	input.shapeRef1 = mGpuNarrowphaseCore->getShapeIndex(*workUnit.getShapeCore1());
	input.transformCacheRef0 = workUnit.mTransformCache0;
	input.transformCacheRef1 = workUnit.mTransformCache1;

	if (mGpuContactManagerBitMap[0].test(cm->getIndex()))
	{
		mFallbackForUnsupportedCMs->refreshContactManagerFallback(cm, mContactManagerOutputs.begin());
	}
	else
	{
		PxU32 offset = 0;
		for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eCount; ++i)
		{
			offset += mRecordedGpuPairCount[i - 1];
			if (mGpuContactManagerBitMap[i].test(cm->getIndex()))
			{
				mGpuNarrowphaseCore->refreshContactManager(cm, mContactManagerOutputs.begin() + offset, input, i);
				break;
			}
		}
	}
}

PxsContactManagerOutput& PxgNphaseImplementationContext::getNewContactManagerOutput(PxU32 npId)
{
	PX_ASSERT(npId & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK);

	PxU32 bucketId = PxsContactManagerBase::computeBucketIndexFromId(npId);

	for (PxU32 i = GPU_BUCKET_ID::eConvex; i < GPU_BUCKET_ID::eTriangleTriangle; ++i)
	{
		PxgNewContactManagers& gpuManagers = mGpuNarrowphaseCore->getNewContactManagers(GPU_BUCKET_ID::Enum(i));
		if(gpuManagers.mBucketId == bucketId)
			return gpuManagers.mGpuOutputContactManagers[PxsContactManagerBase::computeIndexFromId(npId & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
	}

	return mFallbackForUnsupportedCMs->getNewContactManagerOutput(npId);
}

void PxgNphaseImplementationContext::registerShape(const PxNodeIndex& nodeIndex, const PxsShapeCore& shapeCore, const PxU32 transformCacheID, PxActor* actor, const bool isFemCloth)
{
	mGpuNarrowphaseCore->registerShape(nodeIndex, shapeCore, transformCacheID, isFemCloth, actor);
}

void PxgNphaseImplementationContext::updateShapeMaterial(const PxsShapeCore& shapeCore)
{
	mGpuNarrowphaseCore->updateShapeMaterial(shapeCore);
}

void PxgNphaseImplementationContext::unregisterShape(const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isFemCloth)
{
	mGpuNarrowphaseCore->unregisterShape(shapeCore, transformCacheID, isFemCloth);
}

void PxgNphaseImplementationContext::registerAggregate(const PxU32 transformCacheID)
{
	mGpuNarrowphaseCore->registerAggregate(transformCacheID);
}

void PxgNphaseImplementationContext::registerMaterial(const PxsMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->registerMaterial(materialCore);
}

void PxgNphaseImplementationContext::updateMaterial(const PxsMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->updateMaterial(materialCore);
}

void PxgNphaseImplementationContext::unregisterMaterial(const PxsMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->unregisterMaterial(materialCore);
}

void PxgNphaseImplementationContext::registerMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->registerFEMMaterial(materialCore);
}

void PxgNphaseImplementationContext::updateMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->updateFEMMaterial(materialCore);
}

void PxgNphaseImplementationContext::unregisterMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->unregisterFEMMaterial(materialCore);
}

void PxgNphaseImplementationContext::registerMaterial(const PxsDeformableVolumeMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->registerFEMMaterial(materialCore);
}

void PxgNphaseImplementationContext::updateMaterial(const PxsDeformableVolumeMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->updateFEMMaterial(materialCore);
}

void PxgNphaseImplementationContext::unregisterMaterial(const PxsDeformableVolumeMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->unregisterFEMMaterial(materialCore);
}

void PxgNphaseImplementationContext::registerMaterial(const PxsPBDMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->registerParticleMaterial(materialCore);
}

void PxgNphaseImplementationContext::updateMaterial(const PxsPBDMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->updateParticleMaterial(materialCore);
}

void PxgNphaseImplementationContext::unregisterMaterial(const PxsPBDMaterialCore& materialCore)
{
	mGpuNarrowphaseCore->unregisterParticleMaterial(materialCore);
}

void PxgNphaseImplementationContext::startNarrowPhaseTasks()
{
	PxU32 numContactManagers = 0;
	for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
	{
		numContactManagers += mGpuNarrowphaseCore->mContactManagers[i]->mContactManagers.mCpuContactManagerMapping.size();
	}
	
	if(numContactManagers > 0)
	{
		mGpuNarrowphaseCore->pushBuffer();
	}
}

PxReal PxgNphaseImplementationContext::getToleranceLength()
{
	return mContext.getToleranceLength();
}

PxsContactManagerOutputIterator PxgNphaseImplementationContext::getContactManagerOutputs()
{
	PxU32 offsets[GPU_BUCKET_ID::eCount * 2];

	PxU32 count = 0;
	for (PxU32 i = 0; i < GPU_BUCKET_ID::eCount; ++i)
	{
		offsets[i] = count;
		count += mRecordedGpuPairCount[i];
	}

	//Remove fallback pairs from the GPU indexing
	offsets[GPU_BUCKET_ID::eCount] = count - mRecordedGpuPairCount[0];

	count = 0;
	for (PxU32 i = 1; i < GPU_BUCKET_ID::eCount; ++i)
	{
		offsets[GPU_BUCKET_ID::eCount+i] = count;
		count += mRecordedGpuPairCount[i];
	}

	
	return PxsContactManagerOutputIterator(offsets, GPU_BUCKET_ID::eCount * 2, mContactManagerOutputs.begin());
}

void PxgNphaseImplementationContext::acquireContext() { mGpuNarrowphaseCore->acquireContext(); }
void PxgNphaseImplementationContext::releaseContext() { mGpuNarrowphaseCore->releaseContext();  }
