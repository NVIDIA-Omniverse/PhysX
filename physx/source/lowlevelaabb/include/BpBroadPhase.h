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

#ifndef BP_BROADPHASE_H
#define BP_BROADPHASE_H

#include "foundation/PxUserAllocated.h"
#include "PxBroadPhase.h"
#include "BpBroadPhaseUpdate.h"

namespace physx
{
	class PxcScratchAllocator;
	class PxBaseTask;

namespace Bp
{

class BroadPhaseUpdateData;

/**
\brief Base broad phase class.  Functions only relevant to MBP.
*/
class BroadPhaseBase : public PxBroadPhaseRegions, public PxUserAllocated
{
	public:
				BroadPhaseBase()	{}
	virtual		~BroadPhaseBase()	{}

	/**
	\brief Gets broad-phase caps.

	\param	caps	[out] Broad-phase caps
	*/
	virtual	void	getCaps(PxBroadPhaseCaps& caps)	const 
	{
		caps.mMaxNbRegions = 0;
	}

	// PxBroadPhaseRegions
	virtual	PxU32			getNbRegions()														const	PX_OVERRIDE	{ return 0;			}
	virtual	PxU32			getRegions(PxBroadPhaseRegionInfo*, PxU32, PxU32)					const	PX_OVERRIDE	{ return 0;			}
	virtual	PxU32			addRegion(const PxBroadPhaseRegion&, bool, const PxBounds3*, const PxReal*)	PX_OVERRIDE	{ return 0xffffffff;}
	virtual	bool			removeRegion(PxU32)															PX_OVERRIDE	{ return false;		}
	virtual	PxU32			getNbOutOfBoundsObjects()											const	PX_OVERRIDE	{ return 0;			}
	virtual	const PxU32*	getOutOfBoundsObjects()												const	PX_OVERRIDE	{ return NULL;		}
	//~PxBroadPhaseRegions
};

/*
\brief Structure used to report created and deleted broadphase pairs
\note The indices mVolA and mVolB correspond to the bounds indices 
BroadPhaseUpdateData::mCreated used by BroadPhase::update
\see BroadPhase::getCreatedPairs, BroadPhase::getDeletedPairs
*/
struct BroadPhasePair
{
	BroadPhasePair(ShapeHandle volA, ShapeHandle volB) :
		mVolA		(PxMin(volA, volB)),
		mVolB		(PxMax(volA, volB))
	{
	}
	BroadPhasePair() :
		mVolA		(BP_INVALID_BP_HANDLE),
		mVolB		(BP_INVALID_BP_HANDLE)
	{
	}

	ShapeHandle		mVolA;		// NB: mVolA < mVolB
	ShapeHandle		mVolB;
};

class BroadPhase : public BroadPhaseBase
{
public:

	/**
	\brief Instantiate a BroadPhase instance.
	\param[in] bpType - the bp type (either mbp or sap).  This is typically specified in PxSceneDesc. 
	\param[in] maxNbRegions is the expected maximum number of broad-phase regions.
	\param[in] maxNbBroadPhaseOverlaps is the expected maximum number of broad-phase overlaps.
	\param[in] maxNbStaticShapes is the expected maximum number of static shapes.
	\param[in] maxNbDynamicShapes is the expected maximum number of dynamic shapes.
	\param[in] contextID is the context ID parameter sent to the profiler
	\return The instantiated BroadPhase.
	\note maxNbRegions is only used if mbp is the chosen broadphase (PxBroadPhaseType::eMBP)
	\note maxNbRegions, maxNbBroadPhaseOverlaps, maxNbStaticShapes and maxNbDynamicShapes are typically specified in PxSceneLimits
	*/
	static BroadPhase* create(
		const PxBroadPhaseType::Enum bpType,
		const PxU32 maxNbRegions,
		const PxU32 maxNbBroadPhaseOverlaps,
		const PxU32 maxNbStaticShapes,
		const PxU32 maxNbDynamicShapes,
		PxU64 contextID);

	virtual	PxBroadPhaseType::Enum	getType() const = 0;

	/**
	\brief Shutdown of the broadphase.
	*/
	virtual	void	release() = 0;

	/**
	\brief Updates the broadphase and computes the lists of created/deleted pairs.

	\param[in] scratchAllocator - a PxcScratchAllocator instance used for temporary memory allocations.
	This must be non-null.

	\param[in] updateData a description of changes to the collection of aabbs since the last broadphase update.
	The changes detail the indices of the bounds that have been added/updated/removed as well as an array of all
	bound coordinates and an array of group ids used to filter pairs with the same id.
	\see BroadPhaseUpdateData

	\param[in] continuation the task that is in the queue to be executed immediately after the broadphase has completed its update. NULL is not supported.

	\note In PX_CHECKED and PX_DEBUG build configurations illegal input data (that does not conform to the BroadPhaseUpdateData specifications) triggers 
	a special code-path that entirely bypasses the broadphase and issues a warning message to the error stream.  No guarantees can be made about the 
	correctness/consistency of broadphase behavior with illegal input data in PX_RELEASE and PX_PROFILE configs because validity checks are not active 
	in these builds.
	*/
	virtual	void	update(PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation) = 0;

	/**
	\brief prepare broad phase data.
	*/
	virtual	void	preBroadPhase(const Bp::BroadPhaseUpdateData& updateData) = 0;

	/**
	\brief Fetch the results of any asynchronous broad phase work.
	*/
	virtual	void	fetchBroadPhaseResults() = 0;

	/*
	\brief	Get created pairs.

	Note that each overlap pair is reported only on the frame when the overlap first occurs. The overlap persists
	until the pair appears in the list of deleted pairs or either of the bounds in the pair is removed from the broadphase.  
	A created overlap must involve at least one of the bounds of the overlap pair appearing in either the created or updated list.
	It is impossible for the same pair to appear simultaneously in the list of created and deleted overlap pairs.
	An overlap is defined as a pair of bounds that overlap on all three axes; that is when maxA > minB and maxB > minA for all three axes. 

	\param	nbCreatedPairs [out]	The number of created aabb overlap pairs computed in the execution of update() that has just completed.
	\return	The array of created aabb overlap pairs computed in the execution of update() that has just completed.
	*/
	virtual	const BroadPhasePair*	getCreatedPairs(PxU32& nbCreatedPairs)	const	= 0;

	/**
	\brief	Get deleted pairs.

	Note that a deleted pair can only be reported if that pair has already appeared in the list of created pairs in an earlier update.
	A lost overlap occurs when a pair of bounds previously overlapped on all three axes but have now separated on at least one axis.
	A lost overlap must involve at least one of the bounds of the lost overlap pair appearing in the updated list.
	Lost overlaps arising from removal of bounds from the broadphase do not appear in the list of deleted pairs.
	It is impossible for the same pair to appear simultaneously in the list of created and deleted pairs.

	\param	nbDeletedPairs [out]	The number of deleted overlap pairs computed in the execution of update() that has just completed.
	\return	The array of deleted overlap pairs computed in the execution of update() that has just completed.
	*/
	virtual	const BroadPhasePair*	getDeletedPairs(PxU32& nbDeletedPairs)	const	= 0;

	/**
	\brief After the broadphase has completed its update() function and the created/deleted pairs have been queried
	with getCreatedPairs/getDeletedPairs it is desirable to free any memory that was temporarily acquired for the update but is 
	is no longer required post-update.  This can be achieved with the function freeBuffers().
	*/
	virtual	void	freeBuffers()	= 0;

	/**
	\brief Adjust internal structures after all bounds have been adjusted due to a scene origin shift.
	*/
	virtual void	shiftOrigin(const PxVec3& shift, const PxBounds3* boundsArray, const PxReal* contactDistances) = 0;

#if PX_CHECKED
	/**
	\brief Test that the created/updated/removed lists obey the rules that 
	1. object ids can only feature in the created list if they have never been previously added or if they were previously removed.
	2. object ids can only be added to the updated list if they have been previously added without being removed.
	3. objects ids can only be added to the removed list if they have been previously added without being removed.
	*/
	virtual bool	isValid(const BroadPhaseUpdateData& updateData) const = 0;
#endif
};

} //namespace Bp

} //namespace physx

#endif //BP_BROADPHASE_H
