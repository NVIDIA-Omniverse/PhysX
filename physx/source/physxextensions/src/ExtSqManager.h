// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_SQ_MANAGER_H
#define EXT_SQ_MANAGER_H

#include "common/PxPhysXCommonConfig.h"

#include "foundation/PxBitMap.h"
#include "foundation/PxArray.h"
#include "SqPruner.h"
#include "SqManager.h"

#include "foundation/PxHashSet.h"
namespace physx
{
namespace Sq
{
	class CompoundPruner;
}
}

#include "foundation/PxMutex.h"

namespace physx
{
class PxRenderOutput;
class PxBVH;
class PxSceneLimits;

namespace Gu
{
	class BVH;
}

namespace Sq
{
	// PT: this is a customized version of physx::Sq::PrunerManager that supports more than 2 hardcoded pruners.
	// It might not be possible to support the whole PxSceneQuerySystem API with an arbitrary number of pruners.
	class ExtPrunerManager : public PxUserAllocated
	{
	public:
														ExtPrunerManager(PxU64 contextID, float inflation, const Adapter& adapter, bool usesTreeOfPruners);
														~ExtPrunerManager();

						PxU32							addPruner(Gu::Pruner* pruner, PxU32 preallocated);

						Gu::PrunerHandle				addPrunerShape(const Gu::PrunerPayload& payload, PxU32 prunerIndex, bool dynamic, PrunerCompoundId compoundId, const PxBounds3& bounds, const PxTransform& transform, bool hasPruningStructure=false);
						void							addCompoundShape(const PxBVH& bvh, PrunerCompoundId compoundId, const PxTransform& compoundTransform, Gu::PrunerHandle* prunerHandle, const Gu::PrunerPayload* payloads, const PxTransform* transforms, bool isDynamic);
						void							markForUpdate(PxU32 prunerIndex, bool dynamic, PrunerCompoundId compoundId, Gu::PrunerHandle shapeHandle, const PxTransform& transform);
						void							removePrunerShape(PxU32 prunerIndex, bool dynamic, PrunerCompoundId compoundId, Gu::PrunerHandle shapeHandle, Gu::PrunerPayloadRemovalCallback* removalCallback);

		PX_FORCE_INLINE	PxU32							getNbPruners()								const	{ return mPrunerExt.size();				}
		PX_FORCE_INLINE	const Gu::Pruner*				getPruner(PxU32 index)						const	{ return mPrunerExt[index]->mPruner;	}
		PX_FORCE_INLINE	Gu::Pruner*						getPruner(PxU32 index)								{ return mPrunerExt[index]->mPruner;	}
		PX_FORCE_INLINE	const CompoundPruner*			getCompoundPruner()							const	{ return mCompoundPrunerExt.mPruner;	}
		PX_FORCE_INLINE	PxU64							getContextId()								const	{ return mContextID;					}

						void							preallocate(PxU32 prunerIndex, PxU32 nbShapes);

						void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint);
		PX_FORCE_INLINE	PxU32							getDynamicTreeRebuildRateHint()				const	{ return mRebuildRateHint;				}
						
						void							flushUpdates();
						void							forceRebuildDynamicTree(PxU32 prunerIndex);

						void							updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform);
						void							removeCompoundActor(PrunerCompoundId compoundId, Gu::PrunerPayloadRemovalCallback* removalCallback);

						void*							prepareSceneQueriesUpdate(PxU32 prunerIndex);
						void							sceneQueryBuildStep(void* handle);

						void							sync(PxU32 prunerIndex, const Gu::PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices);
						void							afterSync(bool buildStep, bool commit);
						void							shiftOrigin(const PxVec3& shift);
						void							visualize(PxU32 prunerIndex, PxRenderOutput& out)	const;

						void							flushMemory();
		PX_FORCE_INLINE PxU32							getStaticTimestamp()	const	{ return mStaticTimestamp;	}
		PX_FORCE_INLINE const Adapter&					getAdapter()			const	{ return mAdapter;			}
		PX_FORCE_INLINE	const Gu::BVH*					getTreeOfPruners()		const	{ return mTreeOfPruners;	}

						PxU32							startCustomBuildstep();
						void							customBuildstep(PxU32 index);
						void							finishCustomBuildstep();

						void							createTreeOfPruners();
	private:
						const Adapter&					mAdapter;
						PxArray<PrunerExt*>				mPrunerExt;
						CompoundPrunerExt				mCompoundPrunerExt;

						Gu::BVH*						mTreeOfPruners;

						const PxU64						mContextID;
						PxU32							mStaticTimestamp;
						PxU32							mRebuildRateHint;
						const float						mInflation;	// SQ_PRUNER_EPSILON

						PxMutex							mSQLock;  // to make sure only one query updates the dirty pruner structure if multiple queries run in parallel

						volatile bool					mPrunerNeedsUpdating;
						volatile bool					mTimestampNeedsUpdating;
						const bool						mUsesTreeOfPruners;

						void							flushShapes();
		PX_FORCE_INLINE void							invalidateStaticTimestamp()		{ mStaticTimestamp++;		}

						PX_NOCOPY(ExtPrunerManager)
	};
}
}

#endif
