// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_COMPOUND_PRUNING_POOL_H
#define SQ_COMPOUND_PRUNING_POOL_H

#include "SqPruner.h"
#include "foundation/PxArray.h"
#include "GuPrunerMergeData.h"
#include "GuIncrementalAABBTree.h"
#include "GuAABBTreeBounds.h"

namespace physx
{
namespace Gu
{
	class PruningPool;
}

namespace Sq
{

	///////////////////////////////////////////////////////////////////////////////////////////////

	typedef PxArray<Gu::IncrementalAABBTreeNode*>	UpdateMap;

	///////////////////////////////////////////////////////////////////////////////////////////////

	class CompoundTree
	{
	public:
		void	updateObjectAfterManualBoundsUpdates(Gu::PrunerHandle handle);
		void	removeObject(Gu::PrunerHandle handle, Gu::PrunerPayloadRemovalCallback* removalCallback);
		bool	addObject(Gu::PrunerHandle& result, const PxBounds3& bounds, const Gu::PrunerPayload& data, const PxTransform& transform);

	private:
		void	updateMapping(const Gu::PoolIndex poolIndex, Gu::IncrementalAABBTreeNode* node, const Gu::NodeList& changedLeaves);

	public:
		Gu::IncrementalAABBTree*	mTree;
		Gu::PruningPool*			mPruningPool;
		UpdateMap*					mUpdateMap;
		PxTransform					mGlobalPose;
		PxCompoundPrunerQueryFlags	mFlags;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////

	class CompoundTreePool
	{
	public:
													CompoundTreePool(PxU64 contextID);
													~CompoundTreePool();

						void						preallocate(PxU32 newCapacity);

						Gu::PoolIndex				addCompound(Gu::PrunerHandle* results, const Gu::BVH& bvh, const PxBounds3& compoundBounds, const PxTransform& transform, bool isDynamic, const Gu::PrunerPayload* data, const PxTransform* transforms);
						Gu::PoolIndex				removeCompound(Gu::PoolIndex index, Gu::PrunerPayloadRemovalCallback* removalCallback);

						void						shiftOrigin(const PxVec3& shift);

		PX_FORCE_INLINE const Gu::AABBTreeBounds&	getCurrentAABBTreeBounds()	const	{ return mCompoundBounds;				}
		PX_FORCE_INLINE const PxBounds3*			getCurrentCompoundBounds()	const	{ return mCompoundBounds.getBounds();	}
		PX_FORCE_INLINE PxBounds3*					getCurrentCompoundBounds()			{ return mCompoundBounds.getBounds();	}

		PX_FORCE_INLINE const CompoundTree*			getCompoundTrees()			const	{ return mCompoundTrees;				}
		PX_FORCE_INLINE CompoundTree*				getCompoundTrees()					{ return mCompoundTrees;				}

		PX_FORCE_INLINE PxU32						getNbObjects()				const	{ return mNbObjects;					}

	private:
						bool						resize(PxU32 newCapacity);

						PxU32						mNbObjects;			//!< Current number of objects
						PxU32						mMaxNbObjects;		//!< Max. number of objects (capacity for mWorldBoxes, mObjects)

		//!< these arrays are parallel
						Gu::AABBTreeBounds			mCompoundBounds;	//!< List of compound world boxes, stores mNbObjects, capacity=mMaxNbObjects		
						CompoundTree*				mCompoundTrees;

						PxU64						mContextID;
	};

}
}

#endif
