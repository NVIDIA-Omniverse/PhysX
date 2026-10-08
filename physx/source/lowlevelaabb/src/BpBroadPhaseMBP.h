// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
		virtual	void						update(PxU32 numCpuTasks, Cm::FlushPool* flushPool, PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation)	PX_OVERRIDE;
		virtual	void						preBroadPhase(const Bp::BroadPhaseUpdateData&) PX_OVERRIDE	PX_FINAL	{}
		virtual void						fetchBroadPhaseResults()		PX_OVERRIDE	PX_FINAL	{}
		virtual const BroadPhasePair*		getCreatedPairs(PxU32&)	const	PX_OVERRIDE	PX_FINAL;
		virtual const BroadPhasePair*		getDeletedPairs(PxU32&)	const	PX_OVERRIDE	PX_FINAL;
		virtual void						freeBuffers()					PX_OVERRIDE	PX_FINAL;
		virtual void						shiftOrigin(const PxVec3& shift, const PxBounds3* boundsArray, const PxReal* contactDistances)	PX_OVERRIDE	PX_FINAL;
#if PX_CHECKED
		virtual BroadPhaseUpdateError::Enum	isValid(const BroadPhaseUpdateData& updateData)	const	PX_OVERRIDE	PX_FINAL;
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
