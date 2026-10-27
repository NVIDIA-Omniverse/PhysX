// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_SCENE_QUERIES_H
#define NP_SCENE_QUERIES_H

#include "PxSceneQueryDesc.h"

#include "SqQuery.h"

#include "ScSqBoundsSync.h"
#if PX_SUPPORT_PVD
	#include "NpPvdSceneQueryCollector.h"
	#include "NpPvdSceneClient.h"
#endif

#include "PxSceneQuerySystem.h"

#include "NpBounds.h"	// PT: for SQ_PRUNER_EPSILON

namespace physx
{
	class PxScene;
	class PxSceneDesc;

namespace Vd
{
	class PvdSceneClient;
}

class NpSceneQueries : public Sc::SqBoundsSync
#if PX_SUPPORT_PVD
	, public Sq::PVDCapture
#endif
{
												PX_NOCOPY(NpSceneQueries)
	public:
	// PT: TODO: use PxSceneQueryDesc here, but we need some SQ-specific "scene limits"
												NpSceneQueries(const PxSceneDesc& desc, Vd::PvdSceneClient* pvd, PxU64 contextID);
												~NpSceneQueries();

	PX_FORCE_INLINE	PxSceneQuerySystem&			getSQAPI()			{ PX_ASSERT(mSQ);	return *mSQ;	}
	PX_FORCE_INLINE	const PxSceneQuerySystem&	getSQAPI()	const	{ PX_ASSERT(mSQ);	return *mSQ;	}

	protected:
	// SqBoundsSync
	virtual			void						sync(PxU32 prunerIndex, const ScPrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds,
													const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)	PX_OVERRIDE;
	//~SqBoundsSync

	public:
					PxSceneQuerySystem*			mSQ;

#if PX_SUPPORT_PVD
					Vd::PvdSceneClient*			mPVDClient;
					//Scene query and hits for pvd, collected in current frame
			mutable Vd::PvdSceneQueryCollector	mSingleSqCollector;
	PX_FORCE_INLINE	Vd::PvdSceneQueryCollector&	getSingleSqCollector()	const	{ return mSingleSqCollector;	}

	// PVDCapture
	virtual			bool						transmitSceneQueries() PX_OVERRIDE;
	virtual			void						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits) PX_OVERRIDE;
	virtual			void						sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits) PX_OVERRIDE;
	virtual			void						overlap(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData) PX_OVERRIDE;
	//~PVDCapture
#endif // PX_SUPPORT_PVD
};

} // namespace physx, sq

#endif
