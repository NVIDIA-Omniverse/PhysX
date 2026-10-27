// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_CONTEXT_H
#define PXS_CONTEXT_H

#include "CmPinnableArray.h"
#include "foundation/PxPool.h"
#include "PxVisualizationParameter.h"
#include "PxSceneDesc.h"

#include "common/PxRenderOutput.h"

#include "CmPool.h"

#include "PxvNphaseImplementationContext.h"
#include "PxvSimStats.h"
#include "PxsContactManager.h"
#include "PxcNpBatch.h"
#include "PxcConstraintBlockStream.h"
#include "PxcNpCacheStreamPair.h"
#include "PxcNpMemBlockPool.h"
#include "CmUtils.h"
#include "CmTask.h"

#include "PxContactModifyCallback.h"

#include "PxsTransformCache.h"
#include "GuPersistentContactManifold.h"
#include "PxcNpThreadContext.h"

namespace physx
{
#if PX_SUPPORT_GPU_PHYSX
class PxCudaContextManager;
#endif
class PxsRigidBody;
struct PxcConstraintBlock;
class PxsMaterialManager;
class PxsCCDContext;
struct PxsContactManagerOutput;
struct PxvContactManagerTouchEvent;
	
namespace Cm
{
	class FlushPool;
}

namespace IG
{
	typedef PxU32 EdgeIndex;
}

enum PxsTouchEventCount
{
	PXS_LOST_TOUCH_COUNT,
	PXS_NEW_TOUCH_COUNT,
	PXS_CCD_RETOUCH_COUNT, 	// pairs that are touching at a CCD pass and were touching at discrete collision or at a previous CCD pass already
							// (but they could have lost touch in between)
	PXS_TOUCH_EVENT_COUNT
};

class PxsContext : public PxUserAllocated, public PxcNpContext
{
												PX_NOCOPY(PxsContext)
public:
												PxsContext(const PxSceneDesc& desc, PxTaskManager*, Cm::FlushPool&, PxCudaContextManager*, PxU32 poolSlabSize, PxU64 contextID);
												~PxsContext();

					void						createTransformCache(Cm::VirtualAllocatorCallback& allocator, Cm::PinnableAllocatorFallback::Enum fallback);

					PxsContactManager*			createContactManager(PxsContactManager* contactManager, bool useCCD);
					void						createCache(Gu::Cache& cache, PxGeometryType::Enum geomType0, PxGeometryType::Enum geomType1);
					void						destroyCache(Gu::Cache& cache);
					void						destroyContactManager(PxsContactManager* cm);

	PX_FORCE_INLINE	PxU64						getContextId() const { return mContextID; }

	// Collision properties
	PX_FORCE_INLINE	PxContactModifyCallback*	getContactModifyCallback()						const		{ return mContactModifyCallback;	}
	PX_FORCE_INLINE	void						setContactModifyCallback(PxContactModifyCallback* c)		{ mContactModifyCallback = c; mNpImplementationContext->setContactModifyCallback(c);}


    // resource-related
					void						setScratchBlock(void* addr, PxU32 size);

	PX_FORCE_INLINE	void						setContactDistance(const Cm::PinnableArray<PxReal>* contactDistances)	{ mContactDistances = contactDistances;	}

	// Task-related
					void						updateContactManager(PxReal dt, bool hasContactDistanceChanged, PxBaseTask* continuation, 
																	PxBaseTask* firstPassContinuation, Cm::FanoutTask* updateBoundAndShapeTask);
					void						secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation);
					void						fetchUpdateContactManager();
					void						swapStreams();
						
					void						resetThreadContexts();

	// Manager status change
					bool						getManagerTouchEventCount(PxU32* newTouch, PxU32* lostTouch, PxU32* ccdTouch) const;
					void						fillManagerTouchEvents(
												PxArray<PxvContactManagerTouchEvent>& newTouchEvents,
												PxArray<PxvContactManagerTouchEvent>& lostTouchEvents,
												PxArray<PxvContactManagerTouchEvent>* ccdTouchEvents);

					void						beginUpdate();

	// PX_ENABLE_SIM_STATS
	PX_FORCE_INLINE	PxvSimStats&				getSimStats()						{ return mSimStats;													}
	PX_FORCE_INLINE	const PxvSimStats&			getSimStats()				const	{ return mSimStats;													}

	PX_FORCE_INLINE	Cm::FlushPool&				getTaskPool()				const	{ return mTaskPool;													}
	PX_FORCE_INLINE	PxRenderBuffer&				getRenderBuffer()					{ return mRenderBuffer;												}

	PX_FORCE_INLINE	PxReal						getRenderScale()			const	{ return mVisualizationParams[PxVisualizationParameter::eSCALE];	}
	PX_FORCE_INLINE	PxReal						getVisualizationParameter(PxVisualizationParameter::Enum param)	const
												{
													PX_ASSERT(param < PxVisualizationParameter::eNUM_VALUES);

													return mVisualizationParams[param];
												}

	PX_FORCE_INLINE	void						setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)
												{
													PX_ASSERT(param < PxVisualizationParameter::eNUM_VALUES);
													PX_ASSERT(value >= 0.0f);

													mVisualizationParams[param] = value;
												}

	PX_FORCE_INLINE	void						setVisualizationCullingBox(const PxBounds3& box)	{ mVisualizationCullingBox = box;	}
	PX_FORCE_INLINE	const PxBounds3&			getVisualizationCullingBox()				const	{ return mVisualizationCullingBox;	}

	PX_FORCE_INLINE	bool						getPCM()					const	{ return mPCM;					}
	PX_FORCE_INLINE	bool						getContactCacheFlag()		const	{ return mContactCache;			}
	PX_FORCE_INLINE	bool						getCreateAveragePoint()		const	{ return mCreateAveragePoint;	}
	PX_FORCE_INLINE	bool						getCCDFlag()				const	{ return mCCD;					}

	// general stuff
					void						shiftOrigin(const PxVec3& shift);

	PX_FORCE_INLINE	void						setPCM(bool enabled)					{ mPCM = enabled;				}
	PX_FORCE_INLINE	void						setContactCache(bool enabled)			{ mContactCache = enabled;		}

	PX_FORCE_INLINE	PxcScratchAllocator&		getScratchAllocator()					{ return mScratchAllocator;		}
	PX_FORCE_INLINE PxsTransformCache&			getTransformCache()						{ return *mTransformCache;		}
	PX_FORCE_INLINE const PxReal*				getContactDistances()		const		{ return mContactDistances->begin(); }

	PX_FORCE_INLINE	PxvNphaseImplementationContext*	getNphaseImplementationContext()			const							{ return mNpImplementationContext;			}
	PX_FORCE_INLINE	void							setNphaseImplementationContext(PxvNphaseImplementationContext* ctx)			{ mNpImplementationContext = ctx;			}

	PX_FORCE_INLINE	PxvNphaseImplementationContext*	getNphaseFallbackImplementationContext()	const							{ return mNpFallbackImplementationContext;	}
	PX_FORCE_INLINE	void							setNphaseFallbackImplementationContext(PxvNphaseImplementationContext* ctx)	{ mNpFallbackImplementationContext = ctx;	}

	PX_FORCE_INLINE	PxU32							getMaxPatchCount() const				{ return mMaxPatches; }

	PX_FORCE_INLINE	PxcNpThreadContext*			getNpThreadContext()
	{
		// We may want to conditional compile to exclude this on single threaded implementations
		// if it is determined to be a performance hit.
		return mNpThreadContextPool.get();
	}

	PX_FORCE_INLINE	void						putNpThreadContext(PxcNpThreadContext* threadContext)
																						{ mNpThreadContextPool.put(threadContext);	}
	PX_FORCE_INLINE PxMutex&					getLock()								{ return mLock;					}

	PX_FORCE_INLINE	PxTaskManager&				getTaskManager() 
												{ 
													PX_ASSERT(mTaskManager);
													return *mTaskManager; 
												}

	PX_FORCE_INLINE PxCudaContextManager*		getCudaContextManager()
												{
													return mCudaContextManager;
												}

	PX_FORCE_INLINE	void						clearManagerTouchEvents();

	PX_FORCE_INLINE Cm::PoolList<PxsContactManager>& getContactManagerPool()
	{
		return mContactManagerPool;
	}

	PX_FORCE_INLINE void setActiveContactManager(const PxsContactManager* manager, PxIntBool useCCD)
	{
		/*const PxU32 index = manager->getIndex();
		if(index >= mActiveContactManager.size())
		{
			const PxU32 newSize = (2 * index + 256)&~255;
			mActiveContactManager.resize(newSize);
		}
		mActiveContactManager.set(index);*/

		//Record any pairs that have CCD enabled!
		if(useCCD)
		{
			const PxU32 index = manager->getIndex();
			if(index >= mActiveContactManagersWithCCD.size())
			{
				const PxU32 newSize = (2 * index + 256)&~255;
				mActiveContactManagersWithCCD.resize(newSize);
			}
			mActiveContactManagersWithCCD.set(index);
		}
	}

private:
						void					mergeCMDiscreteUpdateResults(PxBaseTask* continuation);
							
	// Threading
	PxcThreadCoherentCache<PxcNpThreadContext, PxcNpContext>
												mNpThreadContextPool;

	// Contact managers
	Cm::PoolList<PxsContactManager>				mContactManagerPool;
	PxPool<Gu::LargePersistentContactManifold>	mManifoldPool;
	PxPool<Gu::SpherePersistentContactManifold>	mSphereManifoldPool;
	
	PxBitMap									mActiveContactManagersWithCCD; //KS - adding to filter any pairs that had a touch
	PxBitMap									mContactManagersWithCCDTouch; //KS - adding to filter any pairs that had a touch
	PxBitMap									mContactManagerTouchEvent;

	PxU32										mCMTouchEventCount[PXS_TOUCH_EVENT_COUNT];

	PxMutex										mLock;

	PxContactModifyCallback*					mContactModifyCallback;

	// narrowphase platform-dependent implementations support
	PxvNphaseImplementationContext*				mNpImplementationContext;
	PxvNphaseImplementationContext*				mNpFallbackImplementationContext;
	
	// debug rendering (CS TODO: MS would like to have these wrapped into a class)
					PxReal						mVisualizationParams[PxVisualizationParameter::eNUM_VALUES];

					PxBounds3					mVisualizationCullingBox;

					PxTaskManager*				mTaskManager;
					Cm::FlushPool&				mTaskPool;

					PxCudaContextManager*		mCudaContextManager;

						// PX_ENABLE_SIM_STATS
					PxvSimStats					mSimStats;
					bool						mPCM;
					bool						mContactCache;
					const bool					mCreateAveragePoint;
					const bool					mCCD;

					PxsTransformCache*					mTransformCache;
					const Cm::PinnableArray<PxReal>*	mContactDistances;
					PxU32								mMaxPatches;
					const PxU64							mContextID;

					friend class PxsCCDContext;
					friend class PxsNphaseImplementationContext;
					friend class PxgNphaseImplementationContext; //FDTODO ideally it shouldn't be here..
};

PX_FORCE_INLINE void PxsContext::clearManagerTouchEvents()
{
	mContactManagerTouchEvent.clear();
	for(PxU32 i = 0; i < PXS_TOUCH_EVENT_COUNT; ++i)
	{
		mCMTouchEventCount[i] = 0;
	}
}

}

#endif
