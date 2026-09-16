// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_NP_THREAD_CONTEXT_H
#define PXC_NP_THREAD_CONTEXT_H

#include "geometry/PxGeometry.h"
#include "geomutils/PxContactBuffer.h"
#include "common/PxRenderOutput.h"

#include "CmRenderBuffer.h"

#include "PxPhysXConfig.h"
#include "CmScaling.h"
#include "PxcNpCacheStreamPair.h"
#include "PxcConstraintBlockStream.h"
#include "PxcThreadCoherentCache.h"
#include "PxcScratchAllocator.h"
#include "foundation/PxBitMap.h"
#include "../pcm/GuPersistentContactManifold.h"
#include "../contact/GuContactMethodImpl.h"

namespace physx
{

class PxsTransformCache;
class PxsMaterialManager;

namespace Sc
{
	class BodySim;
}
   
/*!
Per-thread context used by contact generation routines.
*/

struct PxcDataStreamPool
{
	PxU8* mDataStream;
	PxI32 mSharedDataIndex;
	PxU32 mDataStreamSize;
	PxU32 mSharedDataIndexGPU;

	bool isOverflown()	const
	{
		//FD: my expectaton is that reading those variables is atomic, shared indices are non-decreasing, 
		//so we can only get a false overflow alert because of concurrency issues, which is not a big deal as it means 
		//it did overflow a bit later  
		return (mSharedDataIndex + mSharedDataIndexGPU) > mDataStreamSize;
	}
};

struct PxcNpContext
{
	private:
												PX_NOCOPY(PxcNpContext)
	public:

												PxcNpContext() :
													mNpMemBlockPool			(mScratchAllocator),
													mMeshContactMargin		(0.0f),
													mToleranceLength		(0.0f),
													mContactStreamPool		(NULL),
													mPatchStreamPool		(NULL),
													mForceAndIndiceStreamPool(NULL),
													mFrictionPatchStreamPool(NULL),
													mMaterialManager		(NULL)
												{
												}

					PxcScratchAllocator			mScratchAllocator;
					PxcNpMemBlockPool			mNpMemBlockPool;
					PxReal						mMeshContactMargin;
					PxReal						mToleranceLength;
					Cm::RenderBuffer			mRenderBuffer;
					PxcDataStreamPool*			mContactStreamPool;
					PxcDataStreamPool*			mPatchStreamPool;
					PxcDataStreamPool*			mForceAndIndiceStreamPool;
					PxcDataStreamPool*			mFrictionPatchStreamPool;
					PxsMaterialManager*			mMaterialManager;

	PX_FORCE_INLINE	PxReal						getToleranceLength()		const	{ return mToleranceLength;					}
	PX_FORCE_INLINE	void						setToleranceLength(PxReal x)		{ mToleranceLength = x;						}
	PX_FORCE_INLINE	PxReal						getMeshContactMargin()		const	{ return mMeshContactMargin;				}
	PX_FORCE_INLINE	void						setMeshContactMargin(PxReal x)		{ mMeshContactMargin = x;					}

	PX_FORCE_INLINE	PxcNpMemBlockPool&			getNpMemBlockPool()					{ return mNpMemBlockPool;					}
	PX_FORCE_INLINE	const PxcNpMemBlockPool&	getNpMemBlockPool()			const	{ return mNpMemBlockPool;					}
	PX_FORCE_INLINE void						setMaterialManager(PxsMaterialManager* m){ mMaterialManager = m;				}
	PX_FORCE_INLINE PxsMaterialManager*			getMaterialManager() const			{ return mMaterialManager;					}
};

class PxcNpThreadContext : public PxcThreadCoherentCache<PxcNpThreadContext, PxcNpContext>::EntryBase
{
												PX_NOCOPY(PxcNpThreadContext)
public:
												PxcNpThreadContext(PxcNpContext* params);
												~PxcNpThreadContext();

#if PX_ENABLE_SIM_STATS
					void						clearStats();
#else
					PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	PX_FORCE_INLINE void						addLocalNewTouchCount(PxU32 newTouchCMCount)	{ mLocalNewTouchCount += newTouchCMCount;	}
	PX_FORCE_INLINE void						addLocalLostTouchCount(PxU32 lostTouchCMCount)	{ mLocalLostTouchCount += lostTouchCMCount;	}
	PX_FORCE_INLINE PxU32						getLocalNewTouchCount()					const	{ return mLocalNewTouchCount;				}
	PX_FORCE_INLINE PxU32						getLocalLostTouchCount()				const	{ return mLocalLostTouchCount;				}

	PX_FORCE_INLINE PxBitMap&					getLocalChangeTouch()							{ return mLocalChangeTouch;					}

					void						reset(PxU32 cmCount);
	// debugging
					PxRenderOutput 				mRenderOutput;

	// dsequeira: Need to think about this block pool allocation a bit more. Ideally we'd be 
	// taking blocks from a single pool, except that we want to be able to selectively reclaim
	// blocks if the user needs to defragment, depending on which artifacts they're willing
	// to tolerate, such that the blocks we don't reclaim are contiguous.
#if PX_ENABLE_SIM_STATS
					PxU32						mDiscreteContactPairs	[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
					PxU32						mModifiedContactPairs	[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
#else
					PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
					PxcContactBlockStream 		mContactBlockStream;		// constraint block pool
					PxcNpCacheStreamPair		mNpCacheStreamPair;			// narrow phase pairwise data cache

	// Everything below here is scratch state. Most of it can even overlap.

	// temporary contact buffer
					PxContactBuffer				mContactBuffer;    

	PX_ALIGN(16, Gu::MultiplePersistentContactManifold		mTempManifold); 

					Gu::NarrowPhaseParams		mNarrowPhaseParams;

	// DS: this stuff got moved here from the PxcNpPairContext. As Pierre says:
	////////// PT: those members shouldn't be there in the end, it's not necessary
					PxsTransformCache*			mTransformCache;
					const PxReal*				mContactDistances;
					bool						mPCM;
					bool						mContactCache;
					bool						mCreateAveragePoint;	// flag to enforce whether we create average points
#if PX_ENABLE_SIM_STATS
					PxU32						mCompressedCacheSize;
					PxU32						mNbDiscreteContactPairsWithCacheHits;
					PxU32						mNbDiscreteContactPairsWithContacts;
#else
					PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
					PxReal						mDt; // AP: still needed for ccd
					PxU32						mCCDPass;
					PxU32						mCCDFaceIndex;

					PxU32						mMaxPatches;

					PxcDataStreamPool*			mContactStreamPool;
					PxcDataStreamPool*			mPatchStreamPool;
					PxcDataStreamPool*			mForceAndIndiceStreamPool; //this stream is used to store the force buffer and triangle index if we are performing mesh/heightfield contact gen
					PxcDataStreamPool*			mFrictionPatchStreamPool;
					PxsMaterialManager*			mMaterialManager;

private:
		// change touch handling.
					PxBitMap					mLocalChangeTouch;
					PxU32						mLocalNewTouchCount;
					PxU32						mLocalLostTouchCount;
};

}

#endif
