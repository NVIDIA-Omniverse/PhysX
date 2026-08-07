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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "BpBroadPhase.h"
#include "common/PxProfileZone.h"
#include "foundation/PxBitMap.h"

using namespace physx;
using namespace Bp;

#if PX_CHECKED
static BroadPhaseUpdateError::Enum testHandles(PxU32 size, const BpHandle* handles, const PxU32 capacity, const Bp::FilterGroup::Enum* groups, const PxBounds3* bounds, PxBitMap& bitmap)
{
	if(!handles && size)
		return BroadPhaseUpdateError::eMISSING_DATA;

/*	ValType minVal=0;
	ValType maxVal=0xffffffff;*/

	for(PxU32 i=0;i<size;i++)
	{
		const BpHandle h = handles[i];

		if(h>=capacity)
			return BroadPhaseUpdateError::eOUT_OF_BOUNDS;

		// Array in ascending order of id.
		if(i>0 && (h < handles[i-1]))
			return BroadPhaseUpdateError::eINVALID_ORDER;

		if(groups && groups[h]==FilterGroup::eINVALID)
			return BroadPhaseUpdateError::eINVALID_GROUP;

		bitmap.set(h);

		if(bounds)
		{
			if(!bounds[h].isFinite())
				return BroadPhaseUpdateError::eNON_FINITE_BOUNDS;

			for(PxU32 j=0;j<3;j++)
			{
				// Max must be greater than min
				if(bounds[h].minimum[j] > bounds[h].maximum[j] && !(bounds[h].minimum[j] == PX_MAX_BOUNDS_EXTENTS && bounds[h].maximum[j] == -PX_MAX_BOUNDS_EXTENTS))
					return BroadPhaseUpdateError::eINVALID_BOUNDS;
#if 0
				//Bounds have an upper limit.
				if(bounds[created[i]].getMax(j)>=maxVal)
					return BroadPhaseUpdateError::eINVALID;

				//Bounds have a lower limit.
				if(bounds[created[i]].getMin(j)<=minVal)
					return BroadPhaseUpdateError::eINVALID;

				//Max must be odd.
				if(4 != (bounds[created[i]].getMax(j) & 4))
					return BroadPhaseUpdateError::eINVALID;

				//Min must be even.
				if(0 != (bounds[created[i]].getMin(j) & 4))
					return BroadPhaseUpdateError::eINVALID;
#endif
			}
		}
	}
	return BroadPhaseUpdateError::eNO_ERROR;
}

// PT: test that incoming set of handles does not already appear in passed bitmap
static bool testBitmap(const PxBitMap& bitmap, PxU32 size, const BpHandle* handles)
{
	while(size--)
	{
		const BpHandle h = *handles++;
		if(bitmap.test(h))
			return false;
	}
	return true;
}

static BroadPhaseUpdateError::Enum outputErrorMessage(BroadPhaseUpdateError::Enum error)
{
	PxFoundation& foundation = PxGetFoundation();

	switch(error)
	{
		case BroadPhaseUpdateError::eNO_ERROR:
			break;

		case BroadPhaseUpdateError::eINVALID:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData\n");
			break;

		case BroadPhaseUpdateError::eMISSING_DATA:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - missing data\n");
			break;

		case BroadPhaseUpdateError::eOUT_OF_BOUNDS:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - out of bounds indices\n");
			break;

		case BroadPhaseUpdateError::eINVALID_ORDER:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - invalid order\n");
			break;

		case BroadPhaseUpdateError::eINVALID_GROUP:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - invalid groups\n");
			break;

		case BroadPhaseUpdateError::eNON_FINITE_BOUNDS:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - non-finite bounds\n");
			break;

		case BroadPhaseUpdateError::eINVALID_BOUNDS:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - invalid bounds\n");
			break;

		case BroadPhaseUpdateError::eCREATED_AND_UPDATED:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - created and updated\n");
			break;

		case BroadPhaseUpdateError::eCREATED_AND_REMOVED:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - created and removed\n");
			break;

		case BroadPhaseUpdateError::eUPDATED_AND_REMOVED:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - updated and removed\n");
			break;

		case BroadPhaseUpdateError::eALREADY_ADDED:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - already added\n");
			break;

		case BroadPhaseUpdateError::eALREADY_REMOVED:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - already removed\n");
			break;

		case BroadPhaseUpdateError::eNOT_IN_DATABASE:
			foundation.error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Illegal BroadPhaseUpdateData - not in database\n");
			break;
	}

	return error;
}

BroadPhaseUpdateError::Enum BroadPhaseUpdateData::isValid(PxU64 contextID, const BroadPhase* bp, bool skipBoundValidation) const 
{
	PX_PROFILE_ZONE("BroadPhaseUpdateData::isValid", contextID);

	// PT: these bitmaps, filled by the testHandles functions, will contain the set of incoming created/updated/removed objects.
	const PxU32 boxesCapacity = getCapacity();
	PxBitMap createdBitmap;	createdBitmap.resizeAndClear(boxesCapacity);
	PxBitMap updatedBitmap;	updatedBitmap.resizeAndClear(boxesCapacity);
	PxBitMap removedBitmap;	removedBitmap.resizeAndClear(boxesCapacity);

	{
		const PxBounds3* bounds = skipBoundValidation ? NULL : getAABBs();
		const Bp::FilterGroup::Enum* groups = getGroups();

		BroadPhaseUpdateError::Enum state;

		state = testHandles(getNumCreatedHandles(), getCreatedHandles(), boxesCapacity, groups, bounds, createdBitmap);
		if(state != BroadPhaseUpdateError::eNO_ERROR)
			return outputErrorMessage(state);

		state = testHandles(getNumUpdatedHandles(), getUpdatedHandles(), boxesCapacity, groups, bounds, updatedBitmap);
		if(state != BroadPhaseUpdateError::eNO_ERROR)
			return outputErrorMessage(state);

		state = testHandles(getNumRemovedHandles(), getRemovedHandles(), boxesCapacity, NULL, NULL, removedBitmap);
		if(state != BroadPhaseUpdateError::eNO_ERROR)
			return outputErrorMessage(state);
	}

	{
		// PT: created/updated - test that updated objects did not also appear in the set of created objects.
		if(!testBitmap(createdBitmap, getNumUpdatedHandles(), getUpdatedHandles()))
			return outputErrorMessage(BroadPhaseUpdateError::eCREATED_AND_UPDATED);

		// PT: created/removed - test that removed objects did not also appear in the set of created objects.
		if(!testBitmap(createdBitmap, getNumRemovedHandles(), getRemovedHandles()))
			return outputErrorMessage(BroadPhaseUpdateError::eCREATED_AND_REMOVED);

		// PT: updated/removed - test that removed objects did not also appear in the set of updated objects.
		if(!testBitmap(updatedBitmap, getNumRemovedHandles(), getRemovedHandles()))
			return outputErrorMessage(BroadPhaseUpdateError::eUPDATED_AND_REMOVED);
	}

	if(bp)
		return outputErrorMessage(bp->isValid(*this));

	return BroadPhaseUpdateError::eNO_ERROR;
}
#endif
