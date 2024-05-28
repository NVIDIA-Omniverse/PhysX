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

#ifndef SQ_PRUNER_H
#define SQ_PRUNER_H

#include "foundation/PxBounds3.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxFlags.h"
#include "GuPruner.h"
#include "SqTypedef.h"

namespace physx
{	
namespace Gu
{
	class BVH;
}
namespace Sq
{

/**
\brief Compound-pruner-specific flags for scene queries.
*/
struct PxCompoundPrunerQueryFlag
{
	enum Enum
	{
		eSTATIC		= (1<<0),	//!< Traverse static compounds
		eDYNAMIC	= (1<<1),	//!< Traverse dynamic compounds
	};
};

/**
\brief Flags typedef for the set of bits defined in PxCompoundPrunerQueryFlag.
*/
typedef PxFlags<PxCompoundPrunerQueryFlag::Enum,PxU32> PxCompoundPrunerQueryFlags;
PX_FLAGS_OPERATORS(PxCompoundPrunerQueryFlag::Enum,PxU32)

struct CompoundPrunerRaycastCallback
{
					CompoundPrunerRaycastCallback()		{}
    virtual			~CompoundPrunerRaycastCallback()	{}

	virtual bool	invoke(PxReal& distance, PxU32 primIndex, const Gu::PrunerPayload* payloads, const PxTransform* transforms, const PxTransform* compoundPose) = 0;
};

struct CompoundPrunerOverlapCallback
{
					CompoundPrunerOverlapCallback()		{}
    virtual			~CompoundPrunerOverlapCallback()	{}

	virtual bool	invoke(PxU32 primIndex, const Gu::PrunerPayload* payloads, const PxTransform* transforms, const PxTransform* compoundPose) = 0;
};

//////////////////////////////////////////////////////////////////////////
/**
*	Pruner holding compound objects
*/
//////////////////////////////////////////////////////////////////////////
class CompoundPruner : public Gu::BasePruner
{
	public:
	virtual							~CompoundPruner() {}

	/**
	\brief		Adds compound to the pruner.
	\param		results		[out]	an array for resulting handles
	\param		bvh			[in]	BVH
	\param		compoundId	[in]	compound id
	\param		transform	[in]	compound transform
	\param		data		[in]	an array of object data

	\return		true if success, false if internal allocation failed. The first failing add results in a INVALID_PRUNERHANDLE.
	
	Handles are usable as indices. Each handle is either be a recycled handle returned by the client via removeObjects(),
	or a fresh handle that is either zero, or one greater than the last fresh handle returned.
	*/
	virtual bool					addCompound(Gu::PrunerHandle* results, const Gu::BVH& bvh, PrunerCompoundId compoundId, const PxTransform& transform, bool isDynamic, const Gu::PrunerPayload* data, const PxTransform* transforms) = 0;

	/**
	Removes compound from the pruner.
	\param		compoundId	[in]	compound to remove
	*/
	virtual bool					removeCompound(PrunerCompoundId compoundId, Gu::PrunerPayloadRemovalCallback* removalCallback) = 0;

	/**
	Updates compound object
	\param		compoundId	[in]	compound to update
	\param		transform	[in]	compound transformation
	*/
	virtual bool					updateCompound(PrunerCompoundId compoundId, const PxTransform& transform) = 0;

	/**
	Updates object after manually updating their bounds via "getPayload" calls.
	\param		compoundId	[in]	compound that the object belongs to
	\param		handle		[in]	the object to update
	*/
	virtual void					updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const Gu::PrunerHandle handle) = 0;

	/**
	Removes object from compound pruner.
	\param		compoundId	[in]	compound that the object belongs to	 
	\param		handle		[in]	the object to remove
	*/
	virtual void					removeObject(PrunerCompoundId compoundId, const Gu::PrunerHandle handle, Gu::PrunerPayloadRemovalCallback* removalCallback) = 0;

	/**
	\brief		Adds object to the pruner.
	\param		compoundId	[in]	compound that the object belongs to
	\param		result		[out]	an array for resulting handles
	\param		bounds		[in]	an array of bounds. These bounds are used as-is so they should be pre-inflated if inflation is needed.
	\param		userData	[in]	an array of object data
	
	\return		true if success, false if internal allocation failed. The first failing add results in a INVALID_PRUNERHANDLE.
	*/
	virtual bool					addObject(PrunerCompoundId compoundId, Gu::PrunerHandle& result, const PxBounds3& bounds, const Gu::PrunerPayload userData, const PxTransform& transform) = 0;

	/**
	 *	Query functions
	 *  
	 *	Note: return value may disappear if PrunerCallback contains the necessary information
	 *			currently it is still used for the dynamic pruner internally (to decide if added objects must be queried)
	 */
	virtual	bool					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const = 0;
	virtual	bool					overlap(const Gu::ShapeData& queryVolume, CompoundPrunerOverlapCallback&, PxCompoundPrunerQueryFlags flags) const = 0;
	virtual	bool					sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const = 0;

	/**
	\brief	Retrieves the object's payload and data associated with the handle.

	This function returns the payload associated with a given handle. Additionally it can return the
	destination addresses for the object's bounds & transform. The user can then write the new bounds
	and transform there, before eventually calling updateObjects().

	\param[in]	handle		Object handle (initially returned by addObjects())
	\param[in]	compoundId	The compound id
	\param[out]	data		Optional location where to store the internal data associated with the payload.

	\return	The payload associated with the given handle.
	*/
	virtual const Gu::PrunerPayload&	getPayloadData(Gu::PrunerHandle handle, PrunerCompoundId compoundId, Gu::PrunerPayloadData* data) const = 0;

	/**
	\brief	Preallocate space

	\param[in]	nbEntries	The number of entries to preallocate space for
	*/
	virtual void					preallocate(PxU32 nbEntries) = 0;

	// PT: beware, shape transform
	virtual bool					setTransform(Gu::PrunerHandle handle, PrunerCompoundId compoundId, const PxTransform& transform)	 = 0;

	// PT: beware, actor transform
	virtual	const PxTransform&		getTransform(PrunerCompoundId compoundId)	const	= 0;

	virtual	void					visualizeEx(PxRenderOutput& out, PxU32 color, bool drawStatic, bool drawDynamic) const	= 0;
};

}
}

#endif
