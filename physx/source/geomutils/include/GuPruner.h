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

#ifndef GU_PRUNER_H
#define GU_PRUNER_H

#include "foundation/PxUserAllocated.h"
#include "foundation/PxTransform.h"
#include "GuPrunerPayload.h"
#include "GuPrunerTypedef.h"

namespace physx
{
	class PxRenderOutput;
	class PxBounds3;

namespace Gu
{
	class ShapeData;

	struct PrunerRaycastCallback
	{
						PrunerRaycastCallback()		{}
		virtual			~PrunerRaycastCallback()	{}

		virtual bool	invoke(PxReal& distance, PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms) = 0;
	};

	struct PrunerOverlapCallback
	{
						PrunerOverlapCallback()		{}
		virtual			~PrunerOverlapCallback()	{}

		virtual bool	invoke(PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms) = 0;
	};

	class BasePruner : public PxUserAllocated
	{
		public:
										BasePruner()	{}
		virtual							~BasePruner()	{}

		// shift the origin of the pruner objects
		virtual void					shiftOrigin(const PxVec3& shift) = 0;

		// additional 'internal' interface		
		virtual	void					visualize(PxRenderOutput&, PxU32, PxU32) const {}
	};

	class Pruner : public BasePruner
	{
		public:
										Pruner()	{}
		virtual							~Pruner()	{}

		/**
		\brief	Adds objects to the pruner.

		\param[out]	results				Returned handles for added objects
		\param[in]	bounds				Bounds of added objects. These bounds are used as-is so they should be pre-inflated if inflation is needed.
		\param[in]	data				Payloads for added objects.
		\param[in]	transforms			Transforms of added objects.
		\param[in]	count				Number of objects in the arrays
		\param[in]	hasPruningStructure	True if added objects have pruning structure. The structure will be merged later, adding the objects will not invalidate the pruner. 

		\return		true if success, false if internal allocation failed. The first failing add results in a INVALID_PRUNERHANDLE.

		\see PxPruningStructure
		*/
		virtual bool					addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* data, const PxTransform* transforms, PxU32 count, bool hasPruningStructure) = 0;

		/**
		\brief	Removes objects from the pruner.

		\param[in]	handles				The objects to remove
		\param[in]	count				The number of objects to remove
		\param[in]	removalCallback		Optional callback, called for each removed object (giving access to its payload for keeping external structures in sync)
		*/
		virtual void					removeObjects(const PrunerHandle* handles, PxU32 count, PrunerPayloadRemovalCallback* removalCallback) = 0;

		/**
		\brief	Updates objects with new bounds & transforms.

		There are two ways to use this function:

		1) manual bounds update: you can manually update the bounds via "getPayloadData" calls prior to calling "updateObjects".
		In this case "updateObjects" only notifies the system that the data for these objects has changed. In this mode the
		"inflation", "boundsIndices", "newBounds" and "newTransforms" parameters should remain null.

		2) synchronization mode: in this case the new bounds (and optionally the new transforms) have been computed by an
		external source and "updateObjects" tells the system to update its data from passed buffers. The new bounds are
		always inflated by the "inflation" parameter while being copied. "boundsIndices" is an optional remap table, allowing
		this call to only update a subset of the existing bounds (i.e. the updated bounds don't have to be first copied to a
		separate contiguous buffer).

		\param[in]	handles			The objects to update
		\param[in]	count			The number of objects to update
		\param[in]	inflation		Bounds inflation value
		\param[in]	boundsIndices	The indices of the bounds in the bounds array (or NULL)
		\param[in]	newBounds		Updated bounds array (or NULL)
		\param[in]	newTransforms	Updated transforms array (or NULL)
		*/
		virtual void					updateObjects(const PrunerHandle* handles, PxU32 count, float inflation=0.0f, const PxU32* boundsIndices=NULL, const PxBounds3* newBounds=NULL, const PxTransform32* newTransforms=NULL) = 0;

		/**
		\brief	Gets rid of internal accel struct.	 
		*/
		virtual void					purge() = 0;

		/**
		\brief	Makes the queries consistent with previous changes.

		This function must be called before starting queries on an updated Pruner and assert otherwise.
		*/
		virtual void					commit() = 0;

		/**
		\brief	Merges pruning structure to current pruner, parameters may differ for each pruner implementation.

		\param[in]	mergeParams		Implementation-dependent merge data
		*/
		virtual void					merge(const void* mergeParams) = 0;

		/**
		 *	Query functions
		 *  
		 *	Note: return value may disappear if PrunerCallback contains the necessary information
		 *			currently it is still used for the dynamic pruner internally (to decide if added objects must be queried)
		 */
		virtual	bool					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback&) const = 0;
		virtual	bool					overlap(const Gu::ShapeData& queryVolume, PrunerOverlapCallback&) const = 0;
		virtual	bool					sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback&) const = 0;

		/**
		\brief	Retrieves the object's payload and data associated with the handle.

		This function returns the payload associated with a given handle. Additionally it can return the
		destination addresses for the object's bounds & transform. The user can then write the new bounds
		and transform there, before eventually calling updateObjects().

		\param[in]	handle		Object handle (initially returned by addObjects())
		\param[out]	data		Optional location where to store the internal data associated with the payload.

		\return	The payload associated with the given handle.
		*/
		virtual const PrunerPayload&	getPayloadData(PrunerHandle handle, PrunerPayloadData* data=NULL) const = 0;

		/**
		\brief	Preallocate space

		\param[in]	nbEntries	The number of entries to preallocate space for
		*/
		virtual void					preallocate(PxU32 nbEntries) = 0;

		/**
		\brief	Sets object's transform

		\note	This is equivalent to retrieving the transform's address with "getPayloadData" and writing
		the transform there.

		\param[in]	handle		Object handle (initially returned by addObjects())
		\param[in]	transform	New transform

		\return	True if success
		*/
		virtual bool					setTransform(PrunerHandle handle, const PxTransform& transform)		 = 0;

		// PT: from the SQ branch, maybe temporary, unclear if a getType() function would be better etc
		virtual	bool					isDynamic()	const	{ return false;	}

		virtual	void					getGlobalBounds(PxBounds3&)	const	= 0;
	};

	/**
	*	Pruner building accel structure over time base class
	*/
	class DynamicPruner : public Pruner
	{
		public:

		/**
		 * sets the rebuild hint rate used for step building the accel structure.
		 */
		virtual void					setRebuildRateHint(PxU32 nbStepsForRebuild) = 0;	

		/** 
		 * Steps the accel structure build.
		 * synchronousCall specifies if initialization can happen. It should not initialize build when called from a different thread
		 * returns true if finished
		 */
		virtual bool					buildStep(bool synchronousCall = true) = 0;

		/** 
		 * Prepares new tree build
		 * returns true if new tree is needed
		 */
		virtual bool					prepareBuild() = 0;	
	};
}
}

#endif
