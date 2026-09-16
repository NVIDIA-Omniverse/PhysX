// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_QUERY_H
#define SQ_QUERY_H

// PT: SQ-API LEVEL 3 (Level 1 = SqPruner.h, Level 2 = SqManager/SqPrunerData)
// PT: this file is part of a "high-level" set of files within Sq. The SqPruner API doesn't rely on them.
// PT: this should really be at Np level but moving it to Sq allows us to share it.

#include "foundation/PxSimpleTypes.h"
#include "geometry/PxGeometryQueryFlags.h"

#include "SqManager.h"
#include "PxQueryReport.h"
#include "GuCachedFuncs.h"

namespace physx
{
class PxGeometry;
struct PxQueryFilterData;
struct PxFilterData;
class PxQueryFilterCallback;

namespace Sq
{
	struct MultiQueryInput;

	class PVDCapture
	{
		public:
						PVDCapture()	{}
		virtual			~PVDCapture()	{}

		virtual	bool	transmitSceneQueries()		= 0;

		virtual	void	raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits)								= 0;
		virtual	void	sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits)	= 0;
		virtual	void	overlap(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData)															= 0;
	};

	// SceneQueries-level adapter. Augments the PrunerManager-level adapter with functions needed to perform queries.
	class QueryAdapter : public Adapter
	{
		public:
									QueryAdapter()	{}
		virtual						~QueryAdapter()	{}

		// PT: TODO: decouple from PxQueryCache?
		virtual	Gu::PrunerHandle	findPrunerHandle(const PxQueryCache& cache, PrunerCompoundId& compoundId, PxU32& prunerIndex)	const	= 0;

		// PT: TODO: return reference? but this version is at least consistent with getActorShape
		virtual	void				getFilterData(const Gu::PrunerPayload& payload, PxFilterData& filterData)	const	= 0;
		virtual	void				getActorShape(const Gu::PrunerPayload& payload, PxActorShape& actorShape)	const	= 0;
	};

}

	class SceneQueries
	{
													PX_NOCOPY(SceneQueries)
		public:
													SceneQueries(Sq::PVDCapture* pvd, PxU64 contextID, Gu::Pruner* staticPruner, Gu::Pruner* dynamicPruner,
														PxU32 dynamicTreeRebuildRateHint, float inflation,
														const PxSceneLimits& limits, const Sq::QueryAdapter& adapter);
													~SceneQueries();

		PX_FORCE_INLINE	Sq::PrunerManager&			getPrunerManagerFast()			{ return mSQManager;	}
		PX_FORCE_INLINE	const Sq::PrunerManager&	getPrunerManagerFast()	const	{ return mSQManager;	}

		template<typename QueryHit>
						bool						multiQuery(
														const Sq::MultiQueryInput& in,
														PxHitCallback<QueryHit>& hits, PxHitFlags hitFlags, const PxQueryCache* cache,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) const;

						bool						_raycast(
														const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxRaycastCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags) const;

						bool						_sweep(
														const PxGeometry& geometry, const PxTransform& pose,	// GeomObject data
														const PxVec3& unitDir, const PxReal distance,			// Ray data
														PxSweepCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const;

						bool						_overlap(
														const PxGeometry& geometry, const PxTransform& transform,	// GeomObject data
														PxOverlapCallback& hitCall, 
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags) const;

		PX_FORCE_INLINE	PxU64						getContextId()			const	{ return mSQManager.getContextId();	}
						Sq::PrunerManager			mSQManager;
		public:
						Gu::CachedFuncs				mCachedFuncs;

						Sq::PVDCapture*				mPVD;
	};

#if PX_SUPPORT_EXTERN_TEMPLATE
	//explicit template instantiation declaration
	extern template
	bool SceneQueries::multiQuery<PxRaycastHit>(const Sq::MultiQueryInput&, PxHitCallback<PxRaycastHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;

	extern template
	bool SceneQueries::multiQuery<PxOverlapHit>(const Sq::MultiQueryInput&, PxHitCallback<PxOverlapHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;

	extern template
	bool SceneQueries::multiQuery<PxSweepHit>(const Sq::MultiQueryInput&, PxHitCallback<PxSweepHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;
#endif

}

#endif
