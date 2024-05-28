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

#ifndef BP_BROADPHASE_UPDATE_H
#define BP_BROADPHASE_UPDATE_H

#include "BpFiltering.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxUnionCast.h"

namespace physx
{
namespace Bp
{
typedef PxU32 ShapeHandle;
typedef PxU32 BpHandle;
#define BP_INVALID_BP_HANDLE	0x3fffffff

class BroadPhase;

class BroadPhaseUpdateData
{
public:

     /**
	 \brief A structure detailing the changes to the collection of aabbs, whose overlaps are computed in the broadphase.
	 The structure consists of per-object arrays of object bounds and object groups, and three arrays that index
	 into the per-object arrays, denoting the bounds which are to be created, updated and removed in the broad phase.

	 * each entry in the object arrays represents the same shape or aggregate from frame to frame.
	 * each entry in an index array must be less than the capacity of the per-object arrays.
	 * no index value may appear in more than one index array, and may not occur more than once in that array.

	 An index value is said to be "in use" if it has appeared in a created list in a previous update, and has not
	 since occurred in a removed list.

	 \param[in] created an array of indices describing the bounds that must be inserted into the broadphase.
	 Each index in the array must not be in use.

	 \param[in] updated an array of indices (referencing the boxBounds and boxGroups arrays) describing the bounds
	 that have moved since the last broadphase update. Each index in the array must be in use, and each object
	 whose index is in use and whose AABB has changed must appear in the update list.

	 \param[in] removed an array of indices describing the bounds that must be removed from the broad phase. Each index in
	 the array must be in use.

	 \param[in] boxBounds an array of bounds coordinates for the AABBs to be processed by the broadphase.

	 An entry is valid if its values are integer bitwise representations of floating point numbers that satisfy max>min in each dimension,
	 along with a further rule that minima(maxima) must have even(odd) values. 

	 Each entry whose index is either in use or appears in the created array must be valid. An entry whose index is either not in use or
	 appears in the removed array need not be valid.

	 \param[in]  boxGroups an array of group ids, one for each bound, used for pair filtering.  Bounds with the same group id will not be
	 reported as overlap pairs by the broad phase.  Zero is reserved for static bounds.

	 Entries in this array are immutable: the only way to change the group of an object is to remove it from the broad phase and reinsert
	 it at a different index (recall that each index must appear at most once in the created/updated/removed lists).

	 \param[in]  boxesCapacity the length of the boxBounds and boxGroups arrays.

	 \see BroadPhase::update
	 */
	BroadPhaseUpdateData(
		const ShapeHandle* created, PxU32 createdSize, 
		const ShapeHandle* updated, PxU32 updatedSize, 
		const ShapeHandle* removed, PxU32 removedSize, 
		const PxBounds3* boxBounds, const Bp::FilterGroup::Enum* boxGroups, const PxReal* boxContactDistances, PxU32 boxesCapacity,
		const BpFilter& filter,
		bool stateChanged,
		bool gpuStateChanged
	) :
		mCreated		(created),
		mCreatedSize	(createdSize),
		mUpdated		(updated),
		mUpdatedSize	(updatedSize),
		mRemoved		(removed),
		mRemovedSize	(removedSize),
		mBoxBounds		(boxBounds),
		mBoxGroups		(boxGroups),
		mBoxDistances	(boxContactDistances),
		mBoxesCapacity	(boxesCapacity),
		mFilter			(filter),
		mStateChanged	(stateChanged),
		mGpuStateChanged(gpuStateChanged)
	{
	}

	BroadPhaseUpdateData(const BroadPhaseUpdateData& other) :
		mCreated		(other.mCreated),
		mCreatedSize	(other.mCreatedSize),
		mUpdated		(other.mUpdated),
		mUpdatedSize	(other.mUpdatedSize),
		mRemoved		(other.mRemoved),
		mRemovedSize	(other.mRemovedSize),
		mBoxBounds		(other.mBoxBounds),
		mBoxGroups		(other.mBoxGroups),
		mBoxDistances	(other.mBoxDistances),
		mBoxesCapacity	(other.mBoxesCapacity),
		mFilter			(other.mFilter),
		mStateChanged	(other.mStateChanged),
		mGpuStateChanged(other.mGpuStateChanged)
	{
	}

	BroadPhaseUpdateData& operator=(const BroadPhaseUpdateData& other);

	PX_FORCE_INLINE	const ShapeHandle*				getCreatedHandles()		const { return mCreated;			}
	PX_FORCE_INLINE	PxU32							getNumCreatedHandles()	const { return mCreatedSize;		}

	PX_FORCE_INLINE	const ShapeHandle*				getUpdatedHandles()		const { return mUpdated;			}
	PX_FORCE_INLINE	PxU32							getNumUpdatedHandles()	const { return mUpdatedSize;		}

	PX_FORCE_INLINE	const ShapeHandle*				getRemovedHandles()		const { return mRemoved;			}
	PX_FORCE_INLINE	PxU32							getNumRemovedHandles()	const { return mRemovedSize;		}

	PX_FORCE_INLINE	const PxBounds3*				getAABBs()				const { return mBoxBounds;			}
	PX_FORCE_INLINE	const Bp::FilterGroup::Enum*	getGroups()				const { return mBoxGroups;			}
	PX_FORCE_INLINE	const PxReal*					getContactDistance()	const { return mBoxDistances;		}
	PX_FORCE_INLINE	PxU32							getCapacity()			const { return mBoxesCapacity;		}

	PX_FORCE_INLINE	const BpFilter&					getFilter()				const { return mFilter;				}

	PX_FORCE_INLINE	bool							getStateChanged()		const { return mStateChanged;		}
	PX_FORCE_INLINE	bool							getGpuStateChanged()	const { return mGpuStateChanged;	}

#if PX_CHECKED
	static bool isValid(const BroadPhaseUpdateData& updateData, const BroadPhase& bp, const bool skipBoundValidation, PxU64 contextID);
	bool isValid(const bool skipBoundValidation) const;
#endif

private:

	const ShapeHandle*				mCreated;
	const PxU32						mCreatedSize;

	const ShapeHandle*				mUpdated;
	const PxU32						mUpdatedSize;

	const ShapeHandle*				mRemoved;
	const PxU32						mRemovedSize;

	const PxBounds3*				mBoxBounds;
	const Bp::FilterGroup::Enum*	mBoxGroups;
	const PxReal*					mBoxDistances;
	const PxU32						mBoxesCapacity;

	const BpFilter&					mFilter;

	const bool						mStateChanged;
	const bool						mGpuStateChanged;
};

} //namespace Bp

} //namespace physx

#endif
