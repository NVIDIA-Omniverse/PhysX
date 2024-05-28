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

#ifndef BP_BROADPHASE_MBP_H
#define BP_BROADPHASE_MBP_H

#include "BpBroadPhase.h"
#include "BpBroadPhaseMBPCommon.h"
#include "foundation/PxArray.h"

namespace internalMBP
{
	class MBP;
}

namespace physx
{
namespace Bp
{
	class BroadPhaseMBP : public BroadPhase
	{
											PX_NOCOPY(BroadPhaseMBP)
		public:
											BroadPhaseMBP(	PxU32 maxNbRegions,
															PxU32 maxNbBroadPhaseOverlaps,
															PxU32 maxNbStaticShapes,
															PxU32 maxNbDynamicShapes,
															PxU64 contextID);
		virtual								~BroadPhaseMBP();

	// BroadPhaseBase
		virtual	void						getCaps(PxBroadPhaseCaps& caps)														const	PX_OVERRIDE	PX_FINAL;
	//~BroadPhaseBase

	// PxBroadPhaseRegions
		virtual	PxU32						getNbRegions()																		const	PX_OVERRIDE	PX_FINAL;
		virtual	PxU32						getRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE	PX_FINAL;
		virtual	PxU32						addRegion(const PxBroadPhaseRegion& region, bool populateRegion, const PxBounds3* boundsArray, const PxReal* contactDistance)	PX_OVERRIDE	PX_FINAL;
		virtual	bool						removeRegion(PxU32 handle)			PX_OVERRIDE	PX_FINAL;
		virtual	PxU32						getNbOutOfBoundsObjects()	const	PX_OVERRIDE	PX_FINAL;
		virtual	const PxU32*				getOutOfBoundsObjects()		const	PX_OVERRIDE	PX_FINAL;
	//~PxBroadPhaseRegions

	// BroadPhase
		virtual	PxBroadPhaseType::Enum		getType()					const	PX_OVERRIDE	PX_FINAL	{ return PxBroadPhaseType::eMBP;	}
		virtual	void						release()							PX_OVERRIDE	PX_FINAL	{ PX_DELETE_THIS;					}
		virtual	void						update(PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation)	PX_OVERRIDE;
		virtual	void						preBroadPhase(const Bp::BroadPhaseUpdateData&) PX_OVERRIDE	PX_FINAL	{}
		virtual void						fetchBroadPhaseResults()		PX_OVERRIDE	PX_FINAL	{}
		virtual const BroadPhasePair*		getCreatedPairs(PxU32&)	const	PX_OVERRIDE	PX_FINAL;
		virtual const BroadPhasePair*		getDeletedPairs(PxU32&)	const	PX_OVERRIDE	PX_FINAL;
		virtual void						freeBuffers()					PX_OVERRIDE	PX_FINAL;
		virtual void						shiftOrigin(const PxVec3& shift, const PxBounds3* boundsArray, const PxReal* contactDistances)	PX_OVERRIDE	PX_FINAL;
#if PX_CHECKED
		virtual bool						isValid(const BroadPhaseUpdateData& updateData)	const	PX_OVERRIDE	PX_FINAL;
#endif
	//~BroadPhase

				internalMBP::MBP*			mMBP;		// PT: TODO: aggregate

				MBP_Handle*					mMapping;
				PxU32						mCapacity;
				PxArray<BroadPhasePair>		mCreated;
				PxArray<BroadPhasePair>		mDeleted;

				const Bp::FilterGroup::Enum*mGroups;
				const BpFilter*				mFilter;

				const PxU64					mContextID;

				void						setUpdateData(const BroadPhaseUpdateData& updateData);
				void						addObjects(const BroadPhaseUpdateData& updateData);
				void						removeObjects(const BroadPhaseUpdateData& updateData);
				void						updateObjects(const BroadPhaseUpdateData& updateData);

				void						update();
				void						postUpdate();
				void						allocateMappingArray(PxU32 newCapacity);

				PxU32						getCurrentNbPairs()	const;
	};

} //namespace Bp

} //namespace physx

#endif // BP_BROADPHASE_MBP_H
