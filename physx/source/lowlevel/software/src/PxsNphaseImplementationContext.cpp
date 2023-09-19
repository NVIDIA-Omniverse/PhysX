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
       
#include "PxsContext.h"
#include "CmFlushPool.h"
#include "PxsSimpleIslandManager.h"
#include "common/PxProfileZone.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#endif

#include "PxsContactManagerState.h"

#include "PxsNphaseImplementationContext.h"
#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "PxvGlobals.h"

#include "PxcNpContactPrepShared.h"

using namespace physx;

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif

class PxsCMUpdateTask : public Cm::Task
{
public:

	static const PxU32 BATCH_SIZE = 128;
	//static const PxU32 BATCH_SIZE = 32;

	PxsCMUpdateTask(PxsContext* context, PxReal dt, PxsContactManager** cmArray, PxsContactManagerOutput* cmOutputs, Gu::Cache* caches, PxU32 cmCount, PxContactModifyCallback* callback) :
			Cm::Task	(context->getContextId()),
			mCmArray	(cmArray),
			mCmOutputs	(cmOutputs),
			mCaches		(caches),
			mContext	(context),
			mCallback	(callback),
			mCmCount	(cmCount),
			mDt			(dt),
			mNbPatchChanged(0)			
	{
	}

	virtual void release();

	/*PX_FORCE_INLINE void insert(PxsContactManager* cm)
	{
		PX_ASSERT(mCmCount < BATCH_SIZE);
		mCmArray[mCmCount++]=cm;
	}*/

public:	
	//PxsContactManager*	mCmArray[BATCH_SIZE];
	PxsContactManager**			mCmArray;
	PxsContactManagerOutput*	mCmOutputs;
	Gu::Cache*					mCaches;
	PxsContext*					mContext;
	PxContactModifyCallback*	mCallback;
	PxU32						mCmCount;
	PxReal						mDt;		//we could probably retrieve from context to save space?
	PxU32						mNbPatchChanged;

	PxsContactManagerOutputCounts	mPatchChangedOutputCounts[BATCH_SIZE];
	PxsContactManager*				mPatchChangedCms[BATCH_SIZE];
};
#if PX_VC
#pragma warning(pop)
#endif

void PxsCMUpdateTask::release()
{
	// We used to do Task::release(); here before fixing DE1106 (xbox pure virtual crash)
	// Release in turn causes the dependent tasks to start running
	// The problem was that between the time release was called and by the time we got to the destructor
	// The task chain would get all the way to scene finalization code which would reset the allocation pool
	// And a new task would get allocated at the same address, then we would invoke the destructor on that freshly created task
	// This could potentially cause any number of other problems, it is suprising that it only manifested itself
	// as a pure virtual crash
	PxBaseTask* saveContinuation = mCont;
	this->~PxsCMUpdateTask();
	if (saveContinuation)
		saveContinuation->removeReference();
}

static const bool gUseNewTaskAllocationScheme = false;

class PxsCMDiscreteUpdateTask : public PxsCMUpdateTask
{
public:
	PxsCMDiscreteUpdateTask(PxsContext* context, PxReal dt, PxsContactManager** cms, PxsContactManagerOutput* cmOutputs, Gu::Cache* caches, PxU32 nbCms,
		PxContactModifyCallback* callback):
	  PxsCMUpdateTask(context, dt, cms, cmOutputs, caches, nbCms, callback)
	{}

	virtual ~PxsCMDiscreteUpdateTask()
	{}

	void runModifiableContactManagers(PxU32* modifiableIndices, PxU32 nbModifiableManagers, PxcNpThreadContext& threadContext, PxU32& maxPatches_)
	{
		PX_ASSERT(nbModifiableManagers != 0);

		PxU32 maxPatches = maxPatches_;
		
		class PxcContactSet: public PxContactSet
		{
		public:
			PxcContactSet(PxU32 count, PxModifiableContact *contacts)
			{
				mContacts = contacts;
				mCount = count;
			}
			PxModifiableContact*	getContacts()	{ return mContacts; }
			PxU32					getCount()		{ return mCount; }	
		};

		if(mCallback)
		{
			PX_ALLOCA(mModifiablePairArray, PxContactModifyPair, nbModifiableManagers);

			PxsTransformCache& transformCache = mContext->getTransformCache();
		
			for(PxU32 i = 0; i < nbModifiableManagers; ++i)
			{
				PxU32 index = modifiableIndices[i];
				PxsContactManager& cm = *mCmArray[index];
	
				PxsContactManagerOutput& output = mCmOutputs[index];
	
				PxU32 count = output.nbContacts;
	
				if(count)
				{
					PxContactModifyPair& p = mModifiablePairArray[i];
					PxcNpWorkUnit &unit = cm.getWorkUnit();

					p.shape[0] = gPxvOffsetTable.convertPxsShape2Px(unit.shapeCore0);
					p.shape[1] = gPxvOffsetTable.convertPxsShape2Px(unit.shapeCore1);
	
					p.actor[0] = unit.flags & (PxcNpWorkUnitFlag::eDYNAMIC_BODY0 | PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? gPxvOffsetTable.convertPxsRigidCore2PxRigidBody(unit.rigidCore0)
						: gPxvOffsetTable.convertPxsRigidCore2PxRigidStatic(unit.rigidCore0);

					p.actor[1] = unit.flags & (PxcNpWorkUnitFlag::eDYNAMIC_BODY1 | PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? gPxvOffsetTable.convertPxsRigidCore2PxRigidBody(unit.rigidCore1)
						: gPxvOffsetTable.convertPxsRigidCore2PxRigidStatic(unit.rigidCore1);
	
					p.transform[0] = transformCache.getTransformCache(unit.mTransformCache0).transform;
					p.transform[1] = transformCache.getTransformCache(unit.mTransformCache1).transform;
	
					PxModifiableContact* contacts = reinterpret_cast<PxModifiableContact*>(output.contactPoints);
					static_cast<PxcContactSet&>(p.contacts) = PxcContactSet(count, contacts);
	
					PxReal mi0 = unit.flags & (PxcNpWorkUnitFlag::eDYNAMIC_BODY0 | PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? static_cast<const PxsBodyCore*>(unit.rigidCore0)->maxContactImpulse : PX_MAX_F32;
					PxReal mi1 = unit.flags & (PxcNpWorkUnitFlag::eDYNAMIC_BODY1 | PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? static_cast<const PxsBodyCore*>(unit.rigidCore1)->maxContactImpulse : PX_MAX_F32;
					PxReal maxImpulse = PxMin(mi0, mi1);
					for (PxU32 j = 0; j < count; j++)
						contacts[j].maxImpulse = maxImpulse;
	
		#if PX_ENABLE_SIM_STATS
					PxU8 gt0 = PxTo8(unit.geomType0), gt1 = PxTo8(unit.geomType1);
					threadContext.mModifiedContactPairs[PxMin(gt0, gt1)][PxMax(gt0, gt1)]++;
		#else
					PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
		#endif
				}
			}
	
			mCallback->onContactModify(mModifiablePairArray, nbModifiableManagers);
		}
	
		for(PxU32 i = 0; i < nbModifiableManagers; ++i)
		{
			PxU32 index = modifiableIndices[i];
			PxsContactManager& cm = *mCmArray[index];
	
			//Loop through the contacts in the contact stream and update contact count!
	
			PxU32 numContacts = 0;
			PxcNpWorkUnit& unit = cm.getWorkUnit();
			PxsContactManagerOutput& output = mCmOutputs[index];
	
			PxU32 numPatches = output.nbPatches;
		
			if (output.nbContacts)
			{
				//PxU8* compressedContacts = cm.getWorkUnit().compressedContacts;
				//PxModifyContactHeader* header = reinterpret_cast<PxModifyContactHeader*>(compressedContacts);
				PxContactPatch* patches = reinterpret_cast<PxContactPatch*>(output.contactPatches);
				PxModifiableContact* points = reinterpret_cast<PxModifiableContact*>(output.contactPoints);

				if (patches->internalFlags & PxContactPatch::eREGENERATE_PATCHES)
				{
					//Some data was modified that must trigger patch re-generation...
					for (PxU8 k = 0; k < numPatches; ++k)
					{
						PxU32 startIndex = patches[k].startContactIndex;

						patches[k].normal = points[startIndex].normal;
						patches[k].dynamicFriction = points[startIndex].dynamicFriction;
						patches[k].staticFriction = points[startIndex].staticFriction;
						patches[k].restitution = points[startIndex].restitution;

						for (PxU32 j = 1; j < patches[k].nbContacts; ++j)
						{
							if (points[startIndex].normal.dot(points[j + startIndex].normal) < PXC_SAME_NORMAL
								&& points[j + startIndex].maxImpulse > 0.f) //TODO - this needs extending for material indices but we don't support modifying those yet
							{
								//The points are now in a separate friction patch...
								// Shift all patches above index k up by one to make space
								for (PxU32 c = numPatches - 1; c > k; --c)
								{
									patches[c + 1] = patches[c];
								}
								numPatches++;
								patches[k + 1].materialFlags = patches[k].materialFlags;
								patches[k + 1].internalFlags = patches[k].internalFlags;
								patches[k + 1].startContactIndex = PxTo8(j + startIndex);
								patches[k + 1].nbContacts = PxTo8(patches[k].nbContacts - j);
								//Fill in patch information now that final patches are available
								patches[k].nbContacts = PxU8(j);
								break; // we're done with all contacts in patch k because the remaining were just transferrred to patch k+1
							}
						}
					}
				}

				maxPatches = PxMax(maxPatches, PxU32(numPatches));

				output.nbPatches = PxU8(numPatches);

				for (PxU32 a = 0; a < output.nbContacts; ++a)
				{
					numContacts += points[a].maxImpulse != 0.f;
				}
			}

			if (!numContacts)
			{
				output.nbPatches = 0;
				output.nbContacts = 0;
			}

			if(output.nbPatches != output.prevPatches)
			{
				mPatchChangedCms[mNbPatchChanged] = &cm;
				PxsContactManagerOutputCounts& counts = mPatchChangedOutputCounts[mNbPatchChanged++];
				counts.nbPatches = PxU8(numPatches);
				counts.prevPatches = output.prevPatches;
				counts.statusFlag = output.statusFlag;
				//counts.nbContacts = output.nbContacts16;
			}
	
			if(!numContacts)
			{	
				//KS - we still need to retain the patch count from the previous frame to detect found/lost events...
				unit.clearCachedState();
				continue;
			}
	
			if(threadContext.mContactStreamPool)
			{
				//We need to allocate a new structure inside the contact stream pool
	
				PxU32 patchSize = output.nbPatches * sizeof(PxContactPatch);
				PxU32 contactSize = output.nbContacts * sizeof(PxExtendedContact);
	
				/*PxI32 increment = (PxI32)(patchSize + contactSize);
				PxI32 index = PxAtomicAdd(&mContactStreamPool->mSharedContactIndex, increment) - increment;
				PxU8* address = mContactStreamPool->mContactStream + index;*/
				bool isOverflown = false;
	
				PxI32 contactIncrement = PxI32(contactSize);
				PxI32 contactIndex = PxAtomicAdd(&threadContext.mContactStreamPool->mSharedDataIndex, contactIncrement);
				
				if (threadContext.mContactStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Contact buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
							
				PxU8* contactAddress = threadContext.mContactStreamPool->mDataStream  + threadContext.mContactStreamPool->mDataStreamSize - contactIndex;
	
				PxI32 patchIncrement = PxI32(patchSize);
				PxI32 patchIndex = PxAtomicAdd(&threadContext.mPatchStreamPool->mSharedDataIndex, patchIncrement);
				
				if (threadContext.mPatchStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Patch buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
				
				PxU8* patchAddress = threadContext.mPatchStreamPool->mDataStream + threadContext.mPatchStreamPool->mDataStreamSize - patchIndex;
	
				PxU32 internalFlags = reinterpret_cast<PxContactPatch*>(output.contactPatches)->internalFlags;
	
				PxI32 increment2 = PxI32(output.nbContacts * sizeof(PxReal));
				PxI32 index2 = PxAtomicAdd(&threadContext.mForceAndIndiceStreamPool->mSharedDataIndex, increment2);
				
				if (threadContext.mForceAndIndiceStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Force buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
	
				if (isOverflown)
				{
					output.contactPoints = NULL;
					output.contactPatches = NULL;
					output.contactForces = NULL;
				
					output.nbContacts = output.nbPatches = 0;
				}
				else
				{
					output.contactForces = reinterpret_cast<PxReal*>(threadContext.mForceAndIndiceStreamPool->mDataStream + threadContext.mForceAndIndiceStreamPool->mDataStreamSize - index2);
				
					PxMemZero(output.contactForces, sizeof(PxReal) * output.nbContacts);
					
					PxExtendedContact* contacts = reinterpret_cast<PxExtendedContact*>(contactAddress);
					PxMemCopy(patchAddress, output.contactPatches, sizeof(PxContactPatch) * output.nbPatches);
	
					PxContactPatch* newPatches = reinterpret_cast<PxContactPatch*>(patchAddress);
	
					internalFlags |= PxContactPatch::eCOMPRESSED_MODIFIED_CONTACT;
	
					for(PxU32 a = 0; a < output.nbPatches; ++a)
					{
						newPatches[a].internalFlags = PxU8(internalFlags);
					}
	
					//KS - only the first patch will have mass modification properties set. For the GPU solver, this must be propagated to the remaining patches
					for(PxU32 a = 1; a < output.nbPatches; ++a)
					{
						newPatches[a].mMassModification = newPatches->mMassModification;
					}
	
					PxModifiableContact* sourceContacts = reinterpret_cast<PxModifiableContact*>(output.contactPoints);
	
					for(PxU32 a = 0; a < output.nbContacts; ++a)
					{
						PxExtendedContact& contact = contacts[a];
						PxModifiableContact& srcContact = sourceContacts[a];
						contact.contact = srcContact.contact;
						contact.separation = srcContact.separation;
						contact.targetVelocity = srcContact.targetVelocity;
						contact.maxImpulse = srcContact.maxImpulse;
					}
	
					output.contactPatches = patchAddress;
					output.contactPoints = reinterpret_cast<PxU8*>(contacts);
				}
			}
		}
		maxPatches_ = maxPatches;
	}

	template < void (*NarrowPhase)(PxcNpThreadContext&, const PxcNpWorkUnit&, Gu::Cache&, PxsContactManagerOutput&, PxU64)>
	void processCms(PxcNpThreadContext* threadContext)
	{
		const PxU64 contextID = mContext->getContextId();
		//PX_PROFILE_ZONE("processCms", mContext->getContextId());

		// PT: use local variables to avoid reading class members N times, if possible
		const PxU32 nb = mCmCount;
		PxsContactManager** PX_RESTRICT cmArray = mCmArray;

		PxU32 maxPatches = threadContext->mMaxPatches;

		PxU32 newTouchCMCount = 0, lostTouchCMCount = 0;
		PxBitMap& localChangeTouchCM = threadContext->getLocalChangeTouch();

		PX_ALLOCA(modifiableIndices, PxU32, nb);
		PxU32 modifiableCount = 0;

		for(PxU32 i=0;i<nb;i++)
		{
			const PxU32 prefetch1 = PxMin(i + 1, nb - 1);
			const PxU32 prefetch2 = PxMin(i + 2, nb - 1);

			PxPrefetchLine(cmArray[prefetch2]);
			PxPrefetchLine(&mCmOutputs[prefetch2]);
			PxPrefetchLine(cmArray[prefetch1]->getWorkUnit().shapeCore0);
			PxPrefetchLine(cmArray[prefetch1]->getWorkUnit().shapeCore1);
			PxPrefetchLine(&threadContext->mTransformCache->getTransformCache(cmArray[prefetch1]->getWorkUnit().mTransformCache0));
			PxPrefetchLine(&threadContext->mTransformCache->getTransformCache(cmArray[prefetch1]->getWorkUnit().mTransformCache1));

			PxsContactManager* const cm = cmArray[i];			

			if(cm)
			{
				PxsContactManagerOutput& output = mCmOutputs[i];
				PxcNpWorkUnit& unit = cm->getWorkUnit();

				output.prevPatches = output.nbPatches;

				PxU8 oldStatusFlag = output.statusFlag;

				PxU8 oldTouch = PxTo8(oldStatusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH);

				Gu::Cache& cache = mCaches[i];

				NarrowPhase(*threadContext, unit, cache, output, contextID);
				
				PxU16 newTouch = PxTo8(output.statusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH);
				
				bool modifiable = output.nbPatches != 0 && unit.flags & PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT;

				if(modifiable)
				{
					modifiableIndices[modifiableCount++] = i;
				}
				else
				{
					maxPatches = PxMax(maxPatches, PxTo32(output.nbPatches));

					if(output.prevPatches != output.nbPatches)
					{
						mPatchChangedCms[mNbPatchChanged] = cm;
						PxsContactManagerOutputCounts& counts = mPatchChangedOutputCounts[mNbPatchChanged++];
						counts.nbPatches = output.nbPatches;
						counts.prevPatches = output.prevPatches;
						counts.statusFlag = output.statusFlag;
						//counts.nbContacts = output.nbContacts;
					}
				}

				if (newTouch ^ oldTouch)
				{
					unit.statusFlags = PxU8(output.statusFlag | (unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH));  //KS - todo - remove the need to access the work unit at all!
					localChangeTouchCM.growAndSet(cmArray[i]->getIndex());
					if(newTouch)
						newTouchCMCount++;
					else
						lostTouchCMCount++;
				}
				else if (!(oldStatusFlag&PxsContactManagerStatusFlag::eTOUCH_KNOWN))
				{
					unit.statusFlags = PxU8(output.statusFlag | (unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH));  //KS - todo - remove the need to access the work unit at all!
				}
			}
		}

		if(modifiableCount)
			runModifiableContactManagers(modifiableIndices, modifiableCount, *threadContext, maxPatches);

		threadContext->addLocalNewTouchCount(newTouchCMCount);
		threadContext->addLocalLostTouchCount(lostTouchCMCount);

		threadContext->mMaxPatches = maxPatches;
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.narrowPhase", mContext->getContextId());

		PxcNpThreadContext* PX_RESTRICT threadContext = mContext->getNpThreadContext(); 
	
		threadContext->mDt = mDt;

		const bool pcm = mContext->getPCM();
		threadContext->mPCM = pcm;
		threadContext->mCreateAveragePoint = mContext->getCreateAveragePoint();
		threadContext->mContactCache = mContext->getContactCacheFlag();
		threadContext->mTransformCache = &mContext->getTransformCache();
		threadContext->mContactDistances = mContext->getContactDistances();

		if(pcm)
			processCms<PxcDiscreteNarrowPhasePCM>(threadContext);
		else
			processCms<PxcDiscreteNarrowPhase>(threadContext);

		mContext->putNpThreadContext(threadContext);
	}

	virtual const char* getName() const
	{
		return "PxsContext.contactManagerDiscreteUpdate";
	}
};

static void processContactManagers(PxsContext& context, PxsContactManagers& narrowPhasePairs, PxArray<PxsCMDiscreteUpdateTask*>& tasks, PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation, PxContactModifyCallback* modifyCallback)
{
	Cm::FlushPool& taskPool = context.getTaskPool();

	//Iterate all active contact managers
	taskPool.lock();
	/*const*/ PxU32 nbCmsToProcess = narrowPhasePairs.mContactManagerMapping.size();

	// PT: TASK-CREATION TAG
	if(!gUseNewTaskAllocationScheme)
	{
		for(PxU32 a=0; a<nbCmsToProcess;)
		{
			void* ptr = taskPool.allocateNotThreadSafe(sizeof(PxsCMDiscreteUpdateTask));
			PxU32 nbToProcess = PxMin(nbCmsToProcess - a, PxsCMUpdateTask::BATCH_SIZE);
			PxsCMDiscreteUpdateTask* task = PX_PLACEMENT_NEW(ptr, PxsCMDiscreteUpdateTask)(&context, dt, narrowPhasePairs.mContactManagerMapping.begin() + a, 
				cmOutputs + a, narrowPhasePairs.mCaches.begin() + a, nbToProcess, modifyCallback);

			a += nbToProcess;

			task->setContinuation(continuation);
			task->removeReference();

			tasks.pushBack(task);
		}
	}
	else
	{
		// PT:
		const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();

		PxU32 nbPerTask;
		if(numCpuTasks)
			nbPerTask = nbCmsToProcess > numCpuTasks ? nbCmsToProcess / numCpuTasks : nbCmsToProcess;
		else
			nbPerTask = nbCmsToProcess;

		// PT: we need to respect that limit even with a single thread, because of hardcoded buffer limits in ScAfterIntegrationTask.
		if(nbPerTask>PxsCMUpdateTask::BATCH_SIZE)
			nbPerTask = PxsCMUpdateTask::BATCH_SIZE;

		PxU32 start = 0;
		while(nbCmsToProcess)
		{
			const PxU32 nb = nbCmsToProcess < nbPerTask ? nbCmsToProcess : nbPerTask;

				void* ptr = taskPool.allocateNotThreadSafe(sizeof(PxsCMDiscreteUpdateTask));
				PxsCMDiscreteUpdateTask* task = PX_PLACEMENT_NEW(ptr, PxsCMDiscreteUpdateTask)(&context, dt, narrowPhasePairs.mContactManagerMapping.begin() + start,
					cmOutputs + start, narrowPhasePairs.mCaches.begin() + start, nb, modifyCallback);

				task->setContinuation(continuation);
				task->removeReference();

				tasks.pushBack(task);

			start += nb;
			nbCmsToProcess -= nb;
		}
	}
	taskPool.unlock();
}

void PxsNphaseImplementationContext::processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation)
{
	processContactManagers(mContext, mNarrowPhasePairs, mCmTasks, dt, cmOutputs, continuation, mModifyCallback);
}

void PxsNphaseImplementationContext::processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation)
{
	processContactManagers(mContext, mNewNarrowPhasePairs, mCmTasks, dt, mNewNarrowPhasePairs.mOutputContactManagers.begin(), continuation, mModifyCallback);
}

void PxsNphaseImplementationContext::updateContactManager(PxReal dt, bool /*hasContactDistanceChanged*/, PxBaseTask* continuation, 
	PxBaseTask* firstPassNpContinuation, Cm::FanoutTask* /*updateBoundAndShapeTask*/)
{
	PX_PROFILE_ZONE("Sim.queueNarrowPhase", mContext.mContextID);

	firstPassNpContinuation->removeReference();

	mContext.clearManagerTouchEvents();

#if PX_ENABLE_SIM_STATS
	mContext.mSimStats.mNbDiscreteContactPairsTotal = 0;
	mContext.mSimStats.mNbDiscreteContactPairsWithCacheHits = 0;
	mContext.mSimStats.mNbDiscreteContactPairsWithContacts = 0;
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	//KS - temporarily put this here. TODO - move somewhere better
	mContext.mTotalCompressedCacheSize = 0;
	mContext.mMaxPatches = 0;
	
	processContactManager(dt, mNarrowPhasePairs.mOutputContactManagers.begin(), continuation);
}

void PxsNphaseImplementationContext::secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.queueNarrowPhase", mContext.mContextID);

	processContactManagerSecondPass(dt, continuation);		
}

void PxsNphaseImplementationContext::destroy()
{
	this->~PxsNphaseImplementationContext();
	PX_FREE_THIS;
}

/*void PxsNphaseImplementationContext::registerContactManagers(PxsContactManager** cms, Sc::ShapeInteraction** shapeInteractions, PxU32 nbContactManagers, PxU32 maxContactManagerId)
{
	PX_UNUSED(maxContactManagerId);
	for (PxU32 a = 0; a < nbContactManagers; ++a)
	{
		registerContactManager(cms[a], shapeInteractions[a], 0, 0);
	}
}*/

void PxsNphaseImplementationContext::registerContactManager(PxsContactManager* cm, Sc::ShapeInteraction* shapeInteraction, PxI32 touching, PxU32 patchCount)
{
	PX_ASSERT(cm);

	PxcNpWorkUnit& workUnit = cm->getWorkUnit();
	PxsContactManagerOutput output;

	PX_ASSERT(workUnit.geomType0<PxGeometryType::eGEOMETRY_COUNT);
	PX_ASSERT(workUnit.geomType1<PxGeometryType::eGEOMETRY_COUNT);
	const PxGeometryType::Enum geomType0 = PxGeometryType::Enum(workUnit.geomType0);
	const PxGeometryType::Enum geomType1 = PxGeometryType::Enum(workUnit.geomType1);

	Gu::Cache cache;
	mContext.createCache(cache, geomType0, geomType1);

	PxMemZero(&output, sizeof(output));
	output.nbPatches = PxTo8(patchCount);

	if(workUnit.flags & PxcNpWorkUnitFlag::eOUTPUT_CONSTRAINTS)
		output.statusFlag |= PxsContactManagerStatusFlag::eREQUEST_CONSTRAINTS;

	if (touching > 0)
		output.statusFlag |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else if (touching < 0)
		output.statusFlag |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	output.statusFlag |= PxsContactManagerStatusFlag::eDIRTY_MANAGER;

	if (workUnit.statusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
		workUnit.statusFlags |= PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH;

	output.flags = workUnit.flags;

	mNewNarrowPhasePairs.mOutputContactManagers.pushBack(output);
	mNewNarrowPhasePairs.mCaches.pushBack(cache);
	mNewNarrowPhasePairs.mContactManagerMapping.pushBack(cm);

	if(mGPU)
	{
		mNewNarrowPhasePairs.mShapeInteractions.pushBack(shapeInteraction);
		mNewNarrowPhasePairs.mRestDistances.pushBack(cm->getRestDistance());
		mNewNarrowPhasePairs.mTorsionalProperties.pushBack(PxsTorsionalFrictionData(workUnit.mTorsionalPatchRadius, workUnit.mMinTorsionalPatchRadius));
	}

	PxU32 newSz = mNewNarrowPhasePairs.mOutputContactManagers.size();
	workUnit.mNpIndex = mNewNarrowPhasePairs.computeId(newSz - 1) | PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;
}

void PxsNphaseImplementationContext::removeContactManagersFallback(PxsContactManagerOutput* cmOutputs)
{
	if (mRemovedContactManagers.size())
	{
		lock();
		PxSort(mRemovedContactManagers.begin(), mRemovedContactManagers.size(), PxGreater<PxU32>());

		for (PxU32 a = 0; a < mRemovedContactManagers.size(); ++a)
		{
#if PX_DEBUG
			if (a > 0)
				PX_ASSERT(mRemovedContactManagers[a] < mRemovedContactManagers[a - 1]);
#endif
			unregisterContactManagerInternal(mRemovedContactManagers[a], mNarrowPhasePairs, cmOutputs);
		}

		mRemovedContactManagers.forceSize_Unsafe(0);
		unlock();
	}
}

void PxsNphaseImplementationContext::unregisterContactManager(PxsContactManager* cm)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();

	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);

	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		unregisterAndForceSize(mNarrowPhasePairs, index);
	}
	else
	{
		//KS - the index in the "new" list will be the index 
		unregisterAndForceSize(mNewNarrowPhasePairs, index);
	}
}

void PxsNphaseImplementationContext::refreshContactManager(PxsContactManager* cm)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);
	PxsContactManagerOutput output;
	Sc::ShapeInteraction* interaction;
	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		output = mNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(index)];
		interaction = mGPU ? mNarrowPhasePairs.mShapeInteractions[PxsContactManagerBase::computeIndexFromId(index)] : cm->getShapeInteraction();
		unregisterAndForceSize(mNarrowPhasePairs, index);
	}
	else
	{
		output = mNewNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
		interaction = mGPU ? mNewNarrowPhasePairs.mShapeInteractions[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))] : cm->getShapeInteraction();
		//KS - the index in the "new" list will be the index 
		unregisterAndForceSize(mNewNarrowPhasePairs, index);
	}
	PxI32 touching = 0;
	if(output.statusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH)
		touching = 1;
	else if (output.statusFlag & PxsContactManagerStatusFlag::eHAS_NO_TOUCH)
		touching = -1;
	registerContactManager(cm, interaction, touching, output.nbPatches);
}

void PxsNphaseImplementationContext::unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* /*cmOutputs*/)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);

	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		mRemovedContactManagers.pushBack(index);
	}
	else
	{
		//KS - the index in the "new" list will be the index 
		unregisterAndForceSize(mNewNarrowPhasePairs, index);
	}
}

void PxsNphaseImplementationContext::refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);

	PxsContactManagerOutput output;
	Sc::ShapeInteraction* interaction;
	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		output = cmOutputs[PxsContactManagerBase::computeIndexFromId(index)];
		interaction = mGPU ? mNarrowPhasePairs.mShapeInteractions[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))] : cm->getShapeInteraction();
		//unregisterContactManagerInternal(index, mNarrowPhasePairs, cmOutputs);
		unregisterContactManagerFallback(cm, cmOutputs);
	}
	else
	{
		//KS - the index in the "new" list will be the index 
		output = mNewNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
		interaction = mGPU ? mNewNarrowPhasePairs.mShapeInteractions[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))] : cm->getShapeInteraction();
		unregisterAndForceSize(mNewNarrowPhasePairs, index);
	}

	PxI32 touching = 0;
	if(output.statusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH)
	{
		touching = 1;
		unit.statusFlags |= PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH;
	}
	else if (output.statusFlag & PxsContactManagerStatusFlag::eHAS_NO_TOUCH)
		touching = -1;
	registerContactManager(cm, interaction, touching, output.nbPatches);	
}

void PxsNphaseImplementationContext::appendContactManagers()
{
	//Copy new pairs to end of old pairs. Clear new flag, update npIndex on CM and clear the new pair buffer
	const PxU32 existingSize = mNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 nbToAdd = mNewNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 newSize = existingSize + nbToAdd;
	
	if(newSize > mNarrowPhasePairs.mContactManagerMapping.capacity())
	{
		PxU32 newSz = PxMax(256u, PxMax(mNarrowPhasePairs.mContactManagerMapping.capacity()*2, newSize));

		mNarrowPhasePairs.mContactManagerMapping.reserve(newSz);
		mNarrowPhasePairs.mOutputContactManagers.reserve(newSz);
		mNarrowPhasePairs.mCaches.reserve(newSz);
		if(mGPU)
		{
			mNarrowPhasePairs.mShapeInteractions.reserve(newSz);
			mNarrowPhasePairs.mRestDistances.reserve(newSz);
			mNarrowPhasePairs.mTorsionalProperties.reserve(newSz);
		}
	}

	mNarrowPhasePairs.mContactManagerMapping.forceSize_Unsafe(newSize);
	mNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(newSize);
	mNarrowPhasePairs.mCaches.forceSize_Unsafe(newSize);
	if(mGPU)
	{
		mNarrowPhasePairs.mShapeInteractions.forceSize_Unsafe(newSize);
		mNarrowPhasePairs.mRestDistances.forceSize_Unsafe(newSize);
		mNarrowPhasePairs.mTorsionalProperties.forceSize_Unsafe(newSize);
	}

	PxMemCopy(mNarrowPhasePairs.mContactManagerMapping.begin() + existingSize, mNewNarrowPhasePairs.mContactManagerMapping.begin(), sizeof(PxsContactManager*)*nbToAdd);
	PxMemCopy(mNarrowPhasePairs.mOutputContactManagers.begin() + existingSize, mNewNarrowPhasePairs.mOutputContactManagers.begin(), sizeof(PxsContactManagerOutput)*nbToAdd);
	PxMemCopy(mNarrowPhasePairs.mCaches.begin() + existingSize, mNewNarrowPhasePairs.mCaches.begin(), sizeof(Gu::Cache)*nbToAdd);
	if(mGPU)
	{
		PxMemCopy(mNarrowPhasePairs.mShapeInteractions.begin() + existingSize, mNewNarrowPhasePairs.mShapeInteractions.begin(), sizeof(Sc::ShapeInteraction*)*nbToAdd);
		PxMemCopy(mNarrowPhasePairs.mRestDistances.begin() + existingSize, mNewNarrowPhasePairs.mRestDistances.begin(), sizeof(PxReal)*nbToAdd);
		PxMemCopy(mNarrowPhasePairs.mTorsionalProperties.begin() + existingSize, mNewNarrowPhasePairs.mTorsionalProperties.begin(), sizeof(PxsTorsionalFrictionData)*nbToAdd);
	}

	PxU32* edgeNodeIndices = mIslandSim->getEdgeNodeIndexPtr();

	for(PxU32 a = 0; a < mNewNarrowPhasePairs.mContactManagerMapping.size(); ++a)
	{
		PxsContactManager* cm = mNewNarrowPhasePairs.mContactManagerMapping[a];
		PxcNpWorkUnit& unit = cm->getWorkUnit();
		unit.mNpIndex = mNarrowPhasePairs.computeId(existingSize + a);

		if(unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH)
		{
			unit.statusFlags &= (~PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH);
			if(!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
			{
				PartitionEdge* partitionEdge = mIslandSim->getFirstPartitionEdge(unit.mEdgeIndex);

				while(partitionEdge)
				{
					edgeNodeIndices[partitionEdge->mUniqueIndex] = unit.mNpIndex;
					partitionEdge = partitionEdge->mNextPatch;
				}
			}
		}
	}

	mNewNarrowPhasePairs.clear();

	appendNewLostPairs();
}

void PxsNphaseImplementationContext::appendContactManagersFallback(PxsContactManagerOutput* cmOutputs)
{
	PX_PROFILE_ZONE("PxsNphaseImplementationContext.appendContactManagersFallback", mContext.mContextID);

	//Copy new pairs to end of old pairs. Clear new flag, update npIndex on CM and clear the new pair buffer
	const PxU32 existingSize = mNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 nbToAdd = mNewNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 newSize =existingSize + nbToAdd;
	
	if(newSize > mNarrowPhasePairs.mContactManagerMapping.capacity())
	{
		PxU32 newSz = PxMax(mNarrowPhasePairs.mContactManagerMapping.capacity()*2, newSize);

		mNarrowPhasePairs.mContactManagerMapping.reserve(newSz);
		mNarrowPhasePairs.mCaches.reserve(newSz);
		if(mGPU)
		{
			mNarrowPhasePairs.mShapeInteractions.reserve(newSz);
			mNarrowPhasePairs.mRestDistances.reserve(newSz);
			mNarrowPhasePairs.mTorsionalProperties.reserve(newSz);
		}
	}

	mNarrowPhasePairs.mContactManagerMapping.forceSize_Unsafe(newSize);
	mNarrowPhasePairs.mCaches.forceSize_Unsafe(newSize);
	if(mGPU)
	{
		mNarrowPhasePairs.mShapeInteractions.forceSize_Unsafe(newSize);
		mNarrowPhasePairs.mRestDistances.forceSize_Unsafe(newSize);
		mNarrowPhasePairs.mTorsionalProperties.forceSize_Unsafe(newSize);
	}

	PxMemCopy(mNarrowPhasePairs.mContactManagerMapping.begin() + existingSize, mNewNarrowPhasePairs.mContactManagerMapping.begin(), sizeof(PxsContactManager*)*nbToAdd);
	PxMemCopy(cmOutputs + existingSize, mNewNarrowPhasePairs.mOutputContactManagers.begin(), sizeof(PxsContactManagerOutput)*nbToAdd);
	PxMemCopy(mNarrowPhasePairs.mCaches.begin() + existingSize, mNewNarrowPhasePairs.mCaches.begin(), sizeof(Gu::Cache)*nbToAdd);
	if(mGPU)
	{
		PxMemCopy(mNarrowPhasePairs.mShapeInteractions.begin() + existingSize, mNewNarrowPhasePairs.mShapeInteractions.begin(), sizeof(Sc::ShapeInteraction*)*nbToAdd);
		PxMemCopy(mNarrowPhasePairs.mRestDistances.begin() + existingSize, mNewNarrowPhasePairs.mRestDistances.begin(), sizeof(PxReal)*nbToAdd);
		PxMemCopy(mNarrowPhasePairs.mTorsionalProperties.begin() + existingSize, mNewNarrowPhasePairs.mTorsionalProperties.begin(), sizeof(PxsTorsionalFrictionData)*nbToAdd);
	}

	PxU32* edgeNodeIndices = mIslandSim->getEdgeNodeIndexPtr();

	for(PxU32 a = 0; a < mNewNarrowPhasePairs.mContactManagerMapping.size(); ++a)
	{
		PxsContactManager* cm = mNewNarrowPhasePairs.mContactManagerMapping[a];
		PxcNpWorkUnit& unit = cm->getWorkUnit();
		unit.mNpIndex = mNarrowPhasePairs.computeId(existingSize + a);

		if(unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH)
		{
			unit.statusFlags &= (~PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH);
			if(!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
			{
				PartitionEdge* partitionEdge = mIslandSim->getFirstPartitionEdge(unit.mEdgeIndex);

				while(partitionEdge)
				{
					edgeNodeIndices[partitionEdge->mUniqueIndex] = unit.mNpIndex;
					partitionEdge = partitionEdge->mNextPatch;
				}
			}
		}
	}

	mNewNarrowPhasePairs.clear();

	appendNewLostPairs();
}

void PxsNphaseImplementationContext::appendNewLostPairs()
{
	mCmFoundLostOutputCounts.forceSize_Unsafe(0);
	mCmFoundLost.forceSize_Unsafe(0);
	PxU32 count = 0;
	for (PxU32 i = 0, taskSize = mCmTasks.size(); i < taskSize; ++i)
	{
		PxsCMDiscreteUpdateTask* task = mCmTasks[i];

		const PxU32 patchCount = task->mNbPatchChanged;

		if (patchCount)
		{
			const PxU32 newSize = mCmFoundLostOutputCounts.size() + patchCount;

			//KS - TODO - consider switching to 2x loops to avoid frequent allocations. However, we'll probably only grow this rarely,
			//so may not be worth the effort!
			if (mCmFoundLostOutputCounts.capacity() < newSize)
			{
				//Allocate more memory!!!
				const PxU32 newCapacity = PxMax(mCmFoundLostOutputCounts.capacity() * 2, newSize);
				mCmFoundLostOutputCounts.reserve(newCapacity);
				mCmFoundLost.reserve(newCapacity);
			}
	
			mCmFoundLostOutputCounts.forceSize_Unsafe(newSize);
			mCmFoundLost.forceSize_Unsafe(newSize);

			PxMemCopy(&mCmFoundLost[count], task->mPatchChangedCms, sizeof(PxsContactManager*)*patchCount);
			PxMemCopy(&mCmFoundLostOutputCounts[count], task->mPatchChangedOutputCounts, sizeof(PxsContactManagerOutputCounts)*patchCount);
			count += patchCount;
		}
	}

	mCmTasks.forceSize_Unsafe(0);
}

void PxsNphaseImplementationContext::unregisterContactManagerInternal(PxU32 npIndex, PxsContactManagers& managers, PxsContactManagerOutput* cmOutputs)
{
//	PX_PROFILE_ZONE("unregisterContactManagerInternal", 0);

	//TODO - remove this element from the list.
	const PxU32 index = PxsContactManagerBase::computeIndexFromId((npIndex & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK)));

	//Now we replace-with-last and remove the elements...

	const PxU32 replaceIndex = managers.mContactManagerMapping.size()-1;

	PxsContactManager* replaceManager = managers.mContactManagerMapping[replaceIndex];

	mContext.destroyCache(managers.mCaches[index]);

	managers.mContactManagerMapping[index] = replaceManager;
	managers.mCaches[index] = managers.mCaches[replaceIndex];
	cmOutputs[index] = cmOutputs[replaceIndex];
	if(mGPU)
	{
		managers.mShapeInteractions[index] = managers.mShapeInteractions[replaceIndex];
		managers.mRestDistances[index] = managers.mRestDistances[replaceIndex];
		managers.mTorsionalProperties[index] = managers.mTorsionalProperties[replaceIndex];
	}
	managers.mCaches[replaceIndex].reset();
	
	PxcNpWorkUnit& replaceUnit = replaceManager->getWorkUnit();
	replaceUnit.mNpIndex = npIndex;
	if(replaceUnit.statusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
	{
		if(!(replaceUnit.flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
		{
			PxU32* edgeNodeIndices = mIslandSim->getEdgeNodeIndexPtr();

			PartitionEdge* partitionEdge = mIslandSim->getFirstPartitionEdge(replaceUnit.mEdgeIndex);
			while(partitionEdge)
			{
				edgeNodeIndices[partitionEdge->mUniqueIndex] = replaceUnit.mNpIndex;
				partitionEdge = partitionEdge->mNextPatch;
			}
		}
	}

	managers.mContactManagerMapping.forceSize_Unsafe(replaceIndex);
	managers.mCaches.forceSize_Unsafe(replaceIndex);
	if(mGPU)
	{
		managers.mShapeInteractions.forceSize_Unsafe(replaceIndex);
		managers.mRestDistances.forceSize_Unsafe(replaceIndex);
		managers.mTorsionalProperties.forceSize_Unsafe(replaceIndex);
	}
}

PxsContactManagerOutput& PxsNphaseImplementationContext::getNewContactManagerOutput(PxU32 npId)
{
	PX_ASSERT(npId & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK);
	return mNewNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(npId & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
}

PxsContactManagerOutputIterator PxsNphaseImplementationContext::getContactManagerOutputs()
{
	PxU32 offsets[1] = {0};
	return PxsContactManagerOutputIterator(offsets, 1, mNarrowPhasePairs.mOutputContactManagers.begin());
}

PxvNphaseImplementationContextUsableAsFallback* physx::createNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, PxVirtualAllocatorCallback* allocator, bool gpuDynamics)
{
	// PT: TODO: remove useless placement new

	PxsNphaseImplementationContext* npImplContext = reinterpret_cast<PxsNphaseImplementationContext*>(
		PX_ALLOC(sizeof(PxsNphaseImplementationContext), "PxsNphaseImplementationContext"));

	if(npImplContext)
		npImplContext = PX_PLACEMENT_NEW(npImplContext, PxsNphaseImplementationContext)(context, islandSim, allocator, 0, gpuDynamics);

	return npImplContext;
}

