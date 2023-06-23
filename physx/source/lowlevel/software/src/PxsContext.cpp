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

#include "common/PxProfileZone.h"
#include "PxvConfig.h"
#include "PxcContactCache.h"
#include "PxsRigidBody.h"
#include "PxsContactManager.h"
#include "PxsContext.h"
#include "PxPhysXConfig.h"

#include "foundation/PxBitMap.h"
#include "CmFlushPool.h"

#include "PxsMaterialManager.h"
#include "PxSceneDesc.h"
#include "PxsCCD.h"
#include "PxvGeometry.h"
#include "PxvManager.h"
#include "PxsSimpleIslandManager.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#endif

#include "PxcNpContactPrepShared.h"
#include "PxcNpCache.h"

using namespace physx;

PxsContext::PxsContext(const PxSceneDesc& desc, PxTaskManager* taskManager, Cm::FlushPool& taskPool, PxCudaContextManager* cudaContextManager, PxU32 poolSlabSize, PxU64 contextID) :
	mNpThreadContextPool			(this),
	mContactManagerPool				("mContactManagerPool", this, poolSlabSize),
	mManifoldPool					("mManifoldPool", poolSlabSize),
	mSphereManifoldPool				("mSphereManifoldPool", poolSlabSize),
	mContactModifyCallback			(NULL),
	mNpImplementationContext		(NULL),
	mNpFallbackImplementationContext(NULL),
	mTaskManager					(taskManager),
	mTaskPool						(taskPool),
	mCudaContextManager				(cudaContextManager),
	mPCM							(desc.flags & PxSceneFlag::eENABLE_PCM),
	mContactCache					(false),
	mCreateAveragePoint				(desc.flags & PxSceneFlag::eENABLE_AVERAGE_POINT),
	mContextID						(contextID)
{
	clearManagerTouchEvents();
	mVisualizationCullingBox.setEmpty();

	PxMemZero(mVisualizationParams, sizeof(PxReal) * PxVisualizationParameter::eNUM_VALUES);

	mNpMemBlockPool.init(desc.nbContactDataBlocks, desc.maxNbContactDataBlocks);
}

PxsContext::~PxsContext()
{
	PX_DELETE(mTransformCache);
	mContactManagerPool.destroy(); //manually destroy the contact manager pool, otherwise pool deletion order is random and we can get into trouble with references into other pools needed during destruction.
}

// =========================== Create methods
namespace physx
{
	bool gEnablePCMCaching[][PxGeometryType::eGEOMETRY_COUNT] =
	{
		//eSPHERE,
		{
			false,				//eSPHERE
			false,				//ePLANE
			false,				//eCAPSULE
			false,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			true,				//eTRIANGLEMESH
			true,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//ePLANE
		{
			false,				//eSPHERE
			false,				//ePLANE
			true,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			false,				//eTRIANGLEMESH
			false,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//eCAPSULE,
		{
			false,				//eSPHERE
			true,				//ePLANE
			false,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			true,				//eTRIANGLEMESH
			true,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//eBOX,
		{
			false,				//eSPHERE
			true,				//ePLANE
			true,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			true,				//eTRIANGLEMESH
			true,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//eCONVEXMESH,
		{
			true,				//eSPHERE
			true,				//ePLANE
			true,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			true,				//eTRIANGLEMESH
			true,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//ePARTICLESYSTEM
		{
			false,				//eSPHERE
			false,				//ePLANE
			false,				//eCAPSULE
			false,				//eBOX
			false,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			false,				//eSOFTBODY,
			false,				//eTRIANGLEMESH
			false,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			false,				//eCUSTOM
		},

		//eSOFTBODY
		{
			false,				//eSPHERE
			false,				//ePLANE
			false,				//eCAPSULE
			false,				//eBOX
			false,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			false,				//eSOFTBODY,
			false,				//eTRIANGLEMESH
			false,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			false,				//eCUSTOM
		},

		//eTRIANGLEMESH,
		{
			true,				//eSPHERE
			false,				//ePLANE
			true,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			false,				//eTRIANGLEMESH
			false,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//eHEIGHTFIELD,
		{
			true,				//eSPHERE
			false,				//ePLANE
			true,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			true,				//eSOFTBODY,
			false,				//eTRIANGLEMESH
			false,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		},

		//eHAIRSYSTEM
		{
			false,				//eSPHERE
			false,				//ePLANE
			false,				//eCAPSULE
			false,				//eBOX
			false,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			false,				//eSOFTBODY,
			false,				//eTRIANGLEMESH
			false,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			false,				//eCUSTOM
		},

		//eCUSTOM,
		{
			true,				//eSPHERE
			true,				//ePLANE
			true,				//eCAPSULE
			true,				//eBOX
			true,				//eCONVEXMESH
			false,				//ePARTICLESYSTEM
			false,				//eSOFTBODY,
			true,				//eTRIANGLEMESH
			true,				//eHEIGHTFIELD
			false,				//eHAIRSYSTEM
			true,				//eCUSTOM
		}
	};
	PX_COMPILE_TIME_ASSERT(sizeof(gEnablePCMCaching) / sizeof(gEnablePCMCaching[0]) == PxGeometryType::eGEOMETRY_COUNT);
}

void PxsContext::createTransformCache(PxVirtualAllocatorCallback& allocatorCallback)
{
	mTransformCache = PX_NEW(PxsTransformCache)(allocatorCallback);
}

PxsContactManager* PxsContext::createContactManager(PxsContactManager* contactManager, bool useCCD)
{
	PxsContactManager* cm = contactManager? contactManager : mContactManagerPool.get();
	if(cm)
	{
		cm->getWorkUnit().clearCachedState();

		if(!contactManager)
			setActiveContactManager(cm, useCCD);
	}
	else
	{
		PX_WARN_ONCE("Reached limit of contact pairs.");
	}

	return cm;
}

void PxsContext::createCache(Gu::Cache& cache, PxGeometryType::Enum geomType0, PxGeometryType::Enum geomType1)
{
	if(mPCM)
	{
		if(gEnablePCMCaching[geomType0][geomType1])
		{
			if(geomType0 <= PxGeometryType::eCONVEXMESH && geomType1 <= PxGeometryType::eCONVEXMESH)
			{
				if(geomType0 == PxGeometryType::eSPHERE || geomType1 == PxGeometryType::eSPHERE)
				{
					Gu::PersistentContactManifold* manifold = mSphereManifoldPool.allocate();
					PX_PLACEMENT_NEW(manifold, Gu::SpherePersistentContactManifold());
					cache.setManifold(manifold);
				}
				else
				{
					Gu::PersistentContactManifold* manifold = mManifoldPool.allocate();
					PX_PLACEMENT_NEW(manifold, Gu::LargePersistentContactManifold());
					cache.setManifold(manifold);

				}
				cache.getManifold().clearManifold();
			}
			else
			{
				//ML: raised 1 to indicate the manifold is multiManifold which is for contact gen in mesh/height field
				//cache.manifold = 1;
				cache.setMultiManifold(NULL);
			}
		}
		else
		{
			//cache.manifold =  0;
			cache.mCachedData = NULL;
			cache.mManifoldFlags = 0;
		}			
	}
}

void PxsContext::destroyContactManager(PxsContactManager* cm)
{
	const PxU32 idx = cm->getIndex();
	if(cm->getCCD())
		mActiveContactManagersWithCCD.growAndReset(idx);
	//mActiveContactManager.growAndReset(idx);
	mContactManagerTouchEvent.growAndReset(idx);
	mContactManagerPool.put(cm);
}

void PxsContext::destroyCache(Gu::Cache& cache)
{
	if(cache.isManifold())
	{
		if(!cache.isMultiManifold())
		{
			Gu::PersistentContactManifold& manifold = cache.getManifold();
			if(manifold.mCapacity == GU_SPHERE_MANIFOLD_CACHE_SIZE)
				mSphereManifoldPool.deallocate(static_cast<Gu::SpherePersistentContactManifold*>(&manifold));
			else
				mManifoldPool.deallocate(static_cast<Gu::LargePersistentContactManifold*>(&manifold));
		}
		cache.mCachedData = NULL;
		cache.mManifoldFlags = 0;
	}
}

void PxsContext::setScratchBlock(void* addr, PxU32 size)
{
	mScratchAllocator.setBlock(addr, size);
}

void PxsContext::shiftOrigin(const PxVec3& shift)
{
	// transform cache
	mTransformCache->shiftTransforms(-shift);

#if 0
	if (getContactCacheFlag())
	{
		//Iterate all active contact managers
		PxBitMap::Iterator it(mActiveContactManager);
		PxU32 index = it.getNext();
		while(index != PxBitMap::Iterator::DONE)
		{
			PxsContactManager* cm = mContactManagerPool.findByIndexFast(index);

			PxcNpWorkUnit& npwUnit = cm->getWorkUnit();

			// contact cache
			if(!npwUnit.pairCache.isManifold())
			{
				PxU8* contactCachePtr = npwUnit.pairCache.mCachedData;
				if (contactCachePtr)
				{
					PxcLocalContactsCache* lcc;
					PxU8* contacts = PxcNpCacheRead(npwUnit.pairCache, lcc);
#ifdef _DEBUG
					PxcLocalContactsCache testCache;
					PxU32 testBytes;
					const PxU8* testPtr = PxcNpCacheRead2(npwUnit.pairCache, testCache, testBytes);
#endif
					lcc->mTransform0.p -= shift;
					lcc->mTransform1.p -= shift;
				
					const PxU32 nbContacts = lcc->mNbCachedContacts;
					const bool sameNormal = lcc->mSameNormal;
					const bool useFaceIndices = lcc->mUseFaceIndices;
				
					for(PxU32 i=0; i < nbContacts; i++)
					{
						if (i != nbContacts-1)
							PxPrefetchLine(contacts, 128);

						if(!i || !sameNormal)
							contacts += sizeof(PxVec3);

						PxVec3* cachedPoint	= reinterpret_cast<PxVec3*>(contacts);
						*cachedPoint -= shift;
						contacts += sizeof(PxVec3);
						contacts += sizeof(PxReal);

						if(useFaceIndices)
							contacts += 2 * sizeof(PxU32);
					}
#ifdef _DEBUG
					PX_ASSERT(contacts == (testPtr + testBytes));
#endif
				}
			}

			index = it.getNext();
		}

	}
#endif

	// adjust visualization culling box
	if(!mVisualizationCullingBox.isEmpty())
	{
		mVisualizationCullingBox.minimum -= shift;
		mVisualizationCullingBox.maximum -= shift;
	}
}

void PxsContext::swapStreams()
{
	mNpMemBlockPool.swapNpCacheStreams();
}

void PxsContext::mergeCMDiscreteUpdateResults(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.narrowPhaseMerge", mContextID);

	this->mNpImplementationContext->appendContactManagers();

	//Note: the iterator extracts all the items and returns them to the cache on destruction(for thread safety).
	PxcThreadCoherentCacheIterator<PxcNpThreadContext, PxcNpContext> threadContextIt(mNpThreadContextPool);

	for(PxcNpThreadContext* threadContext = threadContextIt.getNext(); threadContext; threadContext = threadContextIt.getNext())
	{
		mCMTouchEventCount[PXS_LOST_TOUCH_COUNT] += threadContext->getLocalLostTouchCount();
		mCMTouchEventCount[PXS_NEW_TOUCH_COUNT] += threadContext->getLocalNewTouchCount();

#if PX_ENABLE_SIM_STATS
		for(PxU32 i=0;i<PxGeometryType::eGEOMETRY_COUNT;i++)
		{
	#if PX_DEBUG
			for(PxU32 j=0; j<i; j++)
				PX_ASSERT(!threadContext->mDiscreteContactPairs[i][j]);
	#endif
			for(PxU32 j=i; j<PxGeometryType::eGEOMETRY_COUNT; j++)
			{
				const PxU32 nb = threadContext->mDiscreteContactPairs[i][j];
				const PxU32 nbModified = threadContext->mModifiedContactPairs[i][j];
				mSimStats.mNbDiscreteContactPairs[i][j] += nb;
				mSimStats.mNbModifiedContactPairs[i][j] += nbModified;
				mSimStats.mNbDiscreteContactPairsTotal += nb;
			}
		}

		mSimStats.mNbDiscreteContactPairsWithCacheHits += threadContext->mNbDiscreteContactPairsWithCacheHits;
		mSimStats.mNbDiscreteContactPairsWithContacts += threadContext->mNbDiscreteContactPairsWithContacts;

		mSimStats.mTotalCompressedContactSize += threadContext->mCompressedCacheSize;
		//KS - this data is not available yet
		//mSimStats.mTotalConstraintSize += threadContext->mConstraintSize;
		threadContext->clearStats();
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
		mContactManagerTouchEvent.combineInPlace<PxBitMap::OR>(threadContext->getLocalChangeTouch());
		//mContactManagerPatchChangeEvent.combineInPlace<PxBitMap::OR>(threadContext->getLocalPatchChangeMap());
		mTotalCompressedCacheSize += threadContext->mTotalCompressedCacheSize;
		mMaxPatches = PxMax(mMaxPatches, threadContext->mMaxPatches);

		threadContext->mTotalCompressedCacheSize = threadContext->mMaxPatches = 0;
	}
}

void PxsContext::updateContactManager(PxReal dt, bool hasBoundsArrayChanged, bool hasContactDistanceChanged, PxBaseTask* continuation, PxBaseTask* firstPassContinuation,
	Cm::FanoutTask* updateBoundAndShapeTask)
{
	PX_ASSERT(mNpImplementationContext);
	return mNpImplementationContext->updateContactManager(dt, hasBoundsArrayChanged, hasContactDistanceChanged, continuation, 
		firstPassContinuation, updateBoundAndShapeTask);
}

void PxsContext::secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)
{
	PX_ASSERT(mNpImplementationContext);
	mNpImplementationContext->secondPassUpdateContactManager(dt, continuation);
}

void PxsContext::fetchUpdateContactManager()
{
	PX_ASSERT(mNpImplementationContext);
	mNpImplementationContext->fetchUpdateContactManager();
	mergeCMDiscreteUpdateResults(NULL);
}

void PxsContext::resetThreadContexts()
{
	//Note: the iterator extracts all the items and returns them to the cache on destruction(for thread safety).
	PxcThreadCoherentCacheIterator<PxcNpThreadContext, PxcNpContext> threadContextIt(mNpThreadContextPool);
	PxcNpThreadContext* threadContext = threadContextIt.getNext();

	while(threadContext != NULL)
	{
		threadContext->reset(mContactManagerTouchEvent.size());
		threadContext = threadContextIt.getNext();
	}
}

bool PxsContext::getManagerTouchEventCount(int* newTouch, int* lostTouch, int* ccdTouch) const
{
	if(newTouch)
		*newTouch = int(mCMTouchEventCount[PXS_NEW_TOUCH_COUNT]);

	if(lostTouch)
		*lostTouch = int(mCMTouchEventCount[PXS_LOST_TOUCH_COUNT]);

	if(ccdTouch)
		*ccdTouch = int(mCMTouchEventCount[PXS_CCD_RETOUCH_COUNT]);

	return true;
}

bool PxsContext::fillManagerTouchEvents(PxvContactManagerTouchEvent* newTouch, PxI32& newTouchCount, PxvContactManagerTouchEvent* lostTouch, PxI32& lostTouchCount,
										 PxvContactManagerTouchEvent* ccdTouch, PxI32& ccdTouchCount)
{
	const PxvContactManagerTouchEvent* newTouchStart = newTouch;
	const PxvContactManagerTouchEvent* lostTouchStart = lostTouch;
	const PxvContactManagerTouchEvent* ccdTouchStart = ccdTouch;

	const PxvContactManagerTouchEvent* newTouchEnd = newTouch + newTouchCount;
	const PxvContactManagerTouchEvent* lostTouchEnd = lostTouch + lostTouchCount;
	const PxvContactManagerTouchEvent* ccdTouchEnd = ccdTouch + ccdTouchCount;

	PX_UNUSED(newTouchEnd);
	PX_UNUSED(lostTouchEnd);
	PX_UNUSED(ccdTouchEnd);
	
	PxU32 index;
	PxBitMap::Iterator it(mContactManagerTouchEvent);

	while((index = it.getNext()) != PxBitMap::Iterator::DONE)
	{
		PxsContactManager* cm = mContactManagerPool.findByIndexFast(index);

		if(cm->getTouchStatus())
		{
			if(!cm->getHasCCDRetouch())
			{
				PX_ASSERT(newTouch < newTouchEnd);
				newTouch->setCMTouchEventUserData(cm->getShapeInteraction());
				newTouch++;
			}
			else
			{
				PX_ASSERT(ccdTouch);
				PX_ASSERT(ccdTouch < ccdTouchEnd);
				ccdTouch->setCMTouchEventUserData(cm->getShapeInteraction());
				cm->clearCCDRetouch();
				ccdTouch++;
			}
		}
		else
		{
			PX_ASSERT(lostTouch < lostTouchEnd);
			lostTouch->setCMTouchEventUserData(cm->getShapeInteraction());
			lostTouch++;
		}
	}

	newTouchCount = PxI32(newTouch - newTouchStart);
	lostTouchCount = PxI32(lostTouch - lostTouchStart);
	ccdTouchCount = PxI32(ccdTouch - ccdTouchStart);
	return true;
}

void PxsContext::beginUpdate()
{
#if PX_ENABLE_SIM_STATS
	mSimStats.clearAll();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
}

