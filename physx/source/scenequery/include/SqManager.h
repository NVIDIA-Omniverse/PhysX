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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SQ_MANAGER_H
#define SQ_MANAGER_H

// PT: SQ-API LEVEL 2 (Level 1 = SqPruner.h)
// PT: this file is part of a "high-level" set of files within Sq. The SqPruner API doesn't rely on them.
// PT: this should really be at Np level but moving it to Sq allows us to share it.

#include "common/PxPhysXCommonConfig.h"

#include "foundation/PxBitMap.h"
#include "foundation/PxArray.h"
#include "SqPruner.h"
#include "geometry/PxGeometryHelpers.h"

namespace physx
{
namespace Sq
{
	// PrunerManager-level adapter
	class Adapter
	{
		public:
						Adapter()	{}
		virtual			~Adapter()	{}

		// Retrieves the PxGeometry associated with a given PrunerPayload. This will be called by
		// the PrunerManager class when computing bounds.
		virtual	const PxGeometry&	getGeometry(const Gu::PrunerPayload& payload)	const	= 0;
	};

	// PT: extended pruner structure. We might want to move the additional data to the pruner itself later.
	struct PrunerExt : public PxUserAllocated
	{
//		private:
												PrunerExt();
												~PrunerExt();

						void					init(Gu::Pruner* pruner);
						void					flushMemory();
						void					preallocate(PxU32 nbShapes);

						void					addToDirtyList(Gu::PrunerHandle handle, bool dynamic, const PxTransform& transform);
						void					removeFromDirtyList(Gu::PrunerHandle handle);
						bool					processDirtyList(PxU32 index, const Adapter& adapter, float inflation);
//						void					growDirtyList(Gu::PrunerHandle handle);

		PX_FORCE_INLINE	Gu::Pruner*				pruner()			{ return mPruner;	}
		PX_FORCE_INLINE	const Gu::Pruner*		pruner()	const	{ return mPruner;	}

						Gu::Pruner*				mPruner;
						PxBitMap				mDirtyMap;
					PxArray<Gu::PrunerHandle>	mDirtyList;
						bool					mDirtyStatic;	// true if dirty list contains a static

						PX_NOCOPY(PrunerExt)

		friend class PrunerManager;
	};
}
}

#include "foundation/PxHashSet.h"
namespace physx
{
namespace Sq
{
	class CompoundPruner;
	typedef PxPair<PrunerCompoundId, Gu::PrunerHandle>	CompoundPair;
	typedef PxCoalescedHashSet<CompoundPair >			CompoundPrunerSet;
	// AB: extended compound pruner structure, buffers compound shape changes and flushes them.
	struct CompoundPrunerExt : public PxUserAllocated
	{
//		private:
												CompoundPrunerExt();
												~CompoundPrunerExt();

						void					flushMemory();
						void					preallocate(PxU32 nbShapes);
						void					flushShapes(const Adapter& adapter, float inflation);
						void					addToDirtyList(PrunerCompoundId compoundId, Gu::PrunerHandle handle, const PxTransform& transform);
						void					removeFromDirtyList(PrunerCompoundId compoundId, Gu::PrunerHandle handle);						

		PX_FORCE_INLINE	const CompoundPruner*	pruner()	const	{ return mPruner;	}
		PX_FORCE_INLINE	CompoundPruner*			pruner()			{ return mPruner;	}

						CompoundPruner*			mPruner;						
						CompoundPrunerSet		mDirtyList;

						PX_NOCOPY(CompoundPrunerExt)

		friend class PrunerManager;
	};
}
}

#include "foundation/PxMutex.h"
#include "SqPrunerData.h"

namespace physx
{
class PxRenderOutput;
class PxBVH;
class PxSceneLimits;	// PT: TODO: decouple from PxSceneLimits

namespace Sq
{
	class PrunerManager : public PxUserAllocated
	{
	public:
														PrunerManager(PxU64 contextID, Gu::Pruner* staticPruner, Gu::Pruner* dynamicPruner,
															PxU32 dynamicTreeRebuildRateHint, float inflation,
															const PxSceneLimits& limits, const Adapter& adapter);
														~PrunerManager();

						PrunerData						addPrunerShape(const Gu::PrunerPayload& payload, bool dynamic, PrunerCompoundId compoundId, const PxBounds3& bounds, const PxTransform& transform, bool hasPruningStructure=false);
						void							addCompoundShape(const PxBVH& bvh, PrunerCompoundId compoundId, const PxTransform& compoundTransform, PrunerData* prunerData, const Gu::PrunerPayload* payloads, const PxTransform* transforms, bool isDynamic);
						void							markForUpdate(PrunerCompoundId compoundId, PrunerData s, const PxTransform& transform);
						void							removePrunerShape(PrunerCompoundId compoundId, PrunerData shapeData, Gu::PrunerPayloadRemovalCallback* removalCallback);

		PX_FORCE_INLINE	const Gu::Pruner*				getPruner(PruningIndex::Enum index)			const	{ return mPrunerExt[index].mPruner;		}
		PX_FORCE_INLINE	Gu::Pruner*						getPruner(PruningIndex::Enum index)					{ return mPrunerExt[index].mPruner;		}
		PX_FORCE_INLINE	const CompoundPruner*			getCompoundPruner()							const	{ return mCompoundPrunerExt.mPruner;	}
		PX_FORCE_INLINE	PxU64							getContextId()								const	{ return mContextID;					}

						void							preallocate(PxU32 prunerIndex, PxU32 nbShapes);

						void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint);
		PX_FORCE_INLINE	PxU32							getDynamicTreeRebuildRateHint()				const	{ return mRebuildRateHint;				}
						
						void							flushUpdates();
						void							forceRebuildDynamicTree(PxU32 prunerIndex);

						void							updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform);
						void							removeCompoundActor(PrunerCompoundId compoundId, Gu::PrunerPayloadRemovalCallback* removalCallback);

						void*							prepareSceneQueriesUpdate(PruningIndex::Enum index);
						void							sceneQueryBuildStep(void* handle);

						void							sync(const Gu::PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices);
						void							afterSync(bool buildStep, bool commit);
						void							shiftOrigin(const PxVec3& shift);
						void							visualize(PxU32 prunerIndex, PxRenderOutput& out)	const;

						void							flushMemory();
		PX_FORCE_INLINE PxU32							getStaticTimestamp()	const	{ return mStaticTimestamp;	}
		PX_FORCE_INLINE const Adapter&					getAdapter()			const	{ return mAdapter;			}
	private:
						const Adapter&					mAdapter;
						PrunerExt						mPrunerExt[PruningIndex::eCOUNT];
						CompoundPrunerExt				mCompoundPrunerExt;

						const PxU64						mContextID;
						PxU32							mStaticTimestamp;
						PxU32							mRebuildRateHint;
						const float						mInflation;	// SQ_PRUNER_EPSILON

						PxMutex							mSQLock;  // to make sure only one query updates the dirty pruner structure if multiple queries run in parallel

						volatile bool					mPrunerNeedsUpdating;

						void							flushShapes();
		PX_FORCE_INLINE void							invalidateStaticTimestamp()		{ mStaticTimestamp++;		}

						PX_NOCOPY(PrunerManager)
	};
}
}

#endif
