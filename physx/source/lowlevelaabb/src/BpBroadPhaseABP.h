// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BP_BROADPHASE_ABP_H
#define BP_BROADPHASE_ABP_H

#include "foundation/PxArray.h"
#include "BpBroadPhase.h"
#include "PxPhysXConfig.h"
#include "BpBroadPhaseUpdate.h"

#define ABP_MT2

namespace internalABP{
	class ABP;
}

namespace physx
{
namespace Bp
{
	class BroadPhaseABP : public BroadPhase
	{
											PX_NOCOPY(BroadPhaseABP)
		public:
											BroadPhaseABP(	PxU32 maxNbBroadPhaseOverlaps,
															PxU32 maxNbStaticShapes,
															PxU32 maxNbDynamicShapes,
															PxU64 contextID,
															bool enableMT);
		virtual								~BroadPhaseABP();

	// BroadPhase
		virtual	PxBroadPhaseType::Enum		getType()					const	PX_OVERRIDE	PX_FINAL	{ return mEnableMT ? PxBroadPhaseType::ePABP : PxBroadPhaseType::eABP;	}
		virtual	void						release()							PX_OVERRIDE	PX_FINAL	{ PX_DELETE_THIS;														}
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

		internalABP::ABP*					mABP;		// PT: TODO: aggregate
				PxU32						mNbAdded;
				PxU32						mNbUpdated;
				PxU32						mNbRemoved;
				const BpHandle*				mCreatedHandles;
				const BpHandle*				mUpdatedHandles;
				const BpHandle*				mRemovedHandles;
				PxArray<BroadPhasePair>		mCreated;
				PxArray<BroadPhasePair>		mDeleted;

				const Bp::FilterGroup::Enum*mGroups;
				const BpFilter*				mFilter;

				const PxU64					mContextID;
				const bool					mEnableMT;

				void						addObjects();
				void						removeObjects();
				void						updateObjects();
				void						setUpdateData();
	};

} //namespace Bp

} //namespace physx

#endif // BP_BROADPHASE_ABP_H
