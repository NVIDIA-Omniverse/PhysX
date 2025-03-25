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

#ifndef PXG_NPHASE_IMPLEMENTATION_CONTEXT_H
#define PXG_NPHASE_IMPLEMENTATION_CONTEXT_H

#include "CmTask.h"
#include "foundation/PxMutex.h"

#include "PxvNphaseImplementationContext.h"
#include "PxgNarrowphaseCore.h"
#include "PxgSimulationCore.h"
#include "foundation/PxHashMap.h"

#include "PxsContactManagerState.h"
#include "PxgContactManager.h"
#include "PxgPersistentContactManifold.h"
#include "PxgHeapMemAllocator.h"

namespace physx
{
	struct PxcNpWorkUnit;
	class PxgNphaseImplementationContext;
	class PxgGpuContext;
	class PxgParticleSystemCore;
	class PxsKernelWranglerManager;

	namespace Bp
	{
		class BpNonVirtualMemAllocator;
	}

	namespace Dy
	{
		class Context;
	}
	
	class PxgCMGpuDiscreteUpdateBase: public Cm::Task
	{
	private:
		PX_NOCOPY(PxgCMGpuDiscreteUpdateBase)

	public:
		PxgCMGpuDiscreteUpdateBase(PxgNphaseImplementationContext* context):
		   Cm::Task(0), mDt(FLT_MAX), mContext(context)
		{
		}

		virtual ~PxgCMGpuDiscreteUpdateBase()
		{}

		void setDt(PxReal dt)
		{
			mDt = dt;
		}

		void processContactManagers(PxgContactManagers& managers, PxgGpuContactManagers& managersGPU, GPU_BUCKET_ID::Enum type);

	protected:
		PxReal							mDt;		//we could probably retrieve from context to save space?
		PxgNphaseImplementationContext*	mContext;
	};

	class PxgCMGpuDiscreteUpdateFallbackTask : public Cm::Task
	{
	public:
		PxgCMGpuDiscreteUpdateFallbackTask(PxgNphaseImplementationContext* context) :
			Cm::Task(0), mDt(FLT_MAX), mContext(context)
		{
		}

		virtual ~PxgCMGpuDiscreteUpdateFallbackTask()
		{}

		void setDt(PxReal dt)
		{
			mDt = dt;
		}

		virtual const char* getName() const
		{
			return "PxgCMGpuDiscreteUpdateFallbackTask";
		}

		void runInternal();

	protected:
		PxReal							mDt;
		PxgNphaseImplementationContext*	mContext;
	};

	class PxgCMGpuDiscreteUpdateTask : public PxgCMGpuDiscreteUpdateBase
	{
		PxBaseTask* mFirstPassNpContinuation;
		PxBaseTask* mPostBroadPhaseTask;
	public:
		PxgCMGpuDiscreteUpdateTask(PxgNphaseImplementationContext* context) : PxgCMGpuDiscreteUpdateBase(context), mFirstPassNpContinuation(NULL)
		{
			mPostBroadPhaseTask = NULL;
		}

		virtual ~PxgCMGpuDiscreteUpdateTask()
		{
		}

		virtual void setPostBroadPhaseTask(PxBaseTask* postBroadPhaseTask) 
		{
			mPostBroadPhaseTask = postBroadPhaseTask;
		}

		virtual void release()
		{
			if(mFirstPassNpContinuation)
				mFirstPassNpContinuation->removeReference();
			PxgCMGpuDiscreteUpdateBase::release();
		}

		void setFirstPassContinuation(PxBaseTask* firstPassContinuation) { mFirstPassNpContinuation = firstPassContinuation; }

		void runInternal();

		virtual const char* getName() const
		{
			return "PxgNphaseImplementationContext.firstContactManagerDiscreteUpdate";
		}
	};

	class PxgCMGpuDiscreteSecondPassUpdateTask : public PxgCMGpuDiscreteUpdateBase
	{
	public:
		PxgCMGpuDiscreteSecondPassUpdateTask(PxgNphaseImplementationContext* context): PxgCMGpuDiscreteUpdateBase(context)
		{
		}

		virtual ~PxgCMGpuDiscreteSecondPassUpdateTask()
		{}

		void runInternal();

		virtual const char* getName() const
		{
			return "PxgNphaseImplementationContext.secondContactManagerDiscreteUpdate";
		}
		
	protected:
	};

	class PxgNphaseImplementationContext : public PxvNphaseImplementationContext
	{
		private:
												PX_NOCOPY(PxgNphaseImplementationContext)
	public:
		
		PxgNphaseImplementationContext(PxsContext& context, PxsKernelWranglerManager* gpuKernelWrangler, PxvNphaseImplementationFallback* fallbackForUnsupportedCMs,
			const PxGpuDynamicsMemoryConfig& gpuDynamicsConfig, void* contactStreamBase, void* patchStreamBase, void* forceAndIndiceStreamBase,
			PxBoundsArrayPinned& bounds, IG::IslandSim* islandSim, 
			physx::Dy::Context* dynamicsContext, PxgHeapMemoryAllocatorManager* heapMemoryManager, 
			bool useGPUNP);

		virtual ~PxgNphaseImplementationContext();
	
		virtual void				destroy()	PX_OVERRIDE PX_FINAL;
		virtual void				updateContactManager(PxReal dt, bool hasContactDistanceChanged, PxBaseTask* continuation, PxBaseTask* firstPassNpContinuation, Cm::FanoutTask* updateBoundAndShapeTask)	PX_OVERRIDE PX_FINAL;
				void				updateContactManagersFallback(PxReal dt, PxBaseTask* continuation);
		virtual void				postBroadPhaseUpdateContactManager(PxBaseTask* continuation)	PX_OVERRIDE PX_FINAL;
		virtual void				secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)	PX_OVERRIDE PX_FINAL;
		virtual void				fetchUpdateContactManager()	PX_OVERRIDE PX_FINAL;

		virtual void				preallocateNewBuffers(PxU32 nbNewPairs, PxU32 maxIndex)	PX_OVERRIDE PX_FINAL;

		virtual void				registerContactManager(PxsContactManager* cm, const Sc::ShapeInteraction* shapeInteraction, PxI32 touching, PxU32 numPatches)	PX_OVERRIDE PX_FINAL;
		virtual void				unregisterContactManager(PxsContactManager* cm)	PX_OVERRIDE PX_FINAL;
		virtual void				refreshContactManager(PxsContactManager* cm)	PX_OVERRIDE PX_FINAL;

		virtual void				registerShape(const PxNodeIndex& nodeIndex, const PxsShapeCore& shapeCore, const PxU32 transformCacheID, PxActor* actor, const bool isFemCloth)	PX_OVERRIDE PX_FINAL;
		virtual void				updateShapeMaterial(const PxsShapeCore& shapeCore)	PX_OVERRIDE PX_FINAL;
		virtual void				unregisterShape(const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isFemCloth)	PX_OVERRIDE PX_FINAL;

		virtual void				registerAggregate(const PxU32 transformCacheID)	PX_OVERRIDE PX_FINAL;

		virtual void				registerMaterial(const PxsMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				updateMaterial(const PxsMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				unregisterMaterial(const PxsMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;

		virtual void				registerMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				updateMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				unregisterMaterial(const PxsDeformableSurfaceMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;

		virtual void				registerMaterial(const PxsDeformableVolumeMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				updateMaterial(const PxsDeformableVolumeMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				unregisterMaterial(const PxsDeformableVolumeMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;

		virtual void				registerMaterial(const PxsPBDMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				updateMaterial(const PxsPBDMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;
		virtual void				unregisterMaterial(const PxsPBDMaterialCore& materialCore)	PX_OVERRIDE PX_FINAL;

		virtual PxsContactManagerOutput& getNewContactManagerOutput(PxU32 npId)	PX_OVERRIDE PX_FINAL;

		virtual void				appendContactManagers()	PX_OVERRIDE PX_FINAL;

		virtual PxsContactManagerOutputIterator	getContactManagerOutputs()	PX_OVERRIDE PX_FINAL;

		virtual void				setContactModifyCallback(PxContactModifyCallback* callback) PX_OVERRIDE PX_FINAL	{ mFallbackForUnsupportedCMs->setContactModifyCallback(callback); }

		void						updateNarrowPhaseShape();

		void						mergeContactManagers(PxBaseTask* continuation);

		void						processResults();

		virtual	void				startNarrowPhaseTasks()	PX_OVERRIDE PX_FINAL;

		PxReal						getToleranceLength();

		PxgGpuNarrowphaseCore*		getGpuNarrowphaseCore() { return mGpuNarrowphaseCore; }

		PxgSimulationCore*			getSimulationCore() { return mGpuSimulationCore; }
		void						setSimulationCore(PxgSimulationCore* sc) { mGpuSimulationCore = sc; }

		virtual void				acquireContext()	PX_OVERRIDE PX_FINAL;
		virtual void				releaseContext()	PX_OVERRIDE PX_FINAL;

		virtual void				lock()		PX_OVERRIDE PX_FINAL	{ mFallbackForUnsupportedCMs->lock();}
		virtual void				unlock()	PX_OVERRIDE PX_FINAL	{ mFallbackForUnsupportedCMs->unlock(); }
	
		PxsContext&								getContext() { return mContext; }

		virtual PxsContactManagerOutputCounts*	getLostFoundPatchOutputCounts()	PX_OVERRIDE PX_FINAL;
		virtual PxsContactManager**				getLostFoundPatchManagers()		PX_OVERRIDE PX_FINAL;
		virtual PxU32							getNbLostFoundPatchManagers()	PX_OVERRIDE PX_FINAL;

		virtual PxsContactManagerOutput*		getGPUContactManagerOutputBase()	PX_OVERRIDE PX_FINAL;
		virtual PxReal*							getGPURestDistances()				PX_OVERRIDE PX_FINAL;
		virtual Sc::ShapeInteraction**			getGPUShapeInteractions()			PX_OVERRIDE PX_FINAL;
		virtual PxsTorsionalFrictionData*		getGPUTorsionalData()				PX_OVERRIDE PX_FINAL;

	protected:

		void						prepareTempContactManagers(PxBaseTask* continuation);

		void						removeLostPairs();

		void						registerContactManagerInternal(PxsContactManager* cm, const PxcNpWorkUnit& workUnit, PxU32 patchCount, PxI32 touching,
																   const Sc::ShapeInteraction* shapeInteraction, GPU_BUCKET_ID::Enum bucketId);
			
		PxvNphaseImplementationFallback*		mFallbackForUnsupportedCMs;				

		//those arrays logically belongs to the task, but we keep it here to avoid reallocations each frame
		friend class PxgCMGpuDiscreteUpdateBase;
		friend class PxgCMGpuDiscreteUpdateTask;
		friend class PxgCMGpuDiscreteSecondPassUpdateTask;
		friend class PxgCMGpuDiscreteUpdateConvexTriMeshTask;
		friend class PxgCMGpuDiscreteSecondPassUpdateConvexTriMeshTask;
		
		PxgCMGpuDiscreteUpdateTask				mUpdateCMsFirstPassTask;
		PxgCMGpuDiscreteSecondPassUpdateTask	mUpdateCMsSecondPassTask;
		PxgCMGpuDiscreteUpdateFallbackTask		mUpdateFallbackPairs;

		PxPinnedArray<PxsContactManagerOutput>	mContactManagerOutputs;
		
		PxBitMap								mGpuContactManagerBitMap[GPU_BUCKET_ID::eCount];
		PxU32									mRecordedGpuPairCount[GPU_BUCKET_ID::eCount];
		PxU32									mNbPairCount[GPU_BUCKET_ID::eCount];

		PxgGpuNarrowphaseCore*					mGpuNarrowphaseCore;
		PxgSimulationCore*						mGpuSimulationCore;
		PxgGpuContext*							mGpuDynamicContext;

		PxU32									mTotalNbPairs;

		PxBoundsArrayPinned&					mBounds;

		bool									mHasContactDistanceChanged;
		bool									mUseGPUBP;

		PxU32									mMaxPatches;
	};
}

#endif
