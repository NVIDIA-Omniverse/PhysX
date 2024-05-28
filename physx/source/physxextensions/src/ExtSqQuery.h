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

#ifndef EXT_SQ_QUERY_H
#define EXT_SQ_QUERY_H

#include "foundation/PxSimpleTypes.h"
#include "geometry/PxGeometryQueryFlags.h"

#include "ExtSqManager.h"
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
	struct ExtMultiQueryInput;

	class ExtPVDCapture
	{
		public:
						ExtPVDCapture()		{}
		virtual			~ExtPVDCapture()	{}

		virtual	bool	transmitSceneQueries()		= 0;

		virtual	void	raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits)								= 0;
		virtual	void	sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits)	= 0;
		virtual	void	overlap(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData)															= 0;
	};

	// SceneQueries-level adapter. Augments the PrunerManager-level adapter with functions needed to perform queries.
	class ExtQueryAdapter : public Adapter
	{
		public:
									ExtQueryAdapter()	{}
		virtual						~ExtQueryAdapter()	{}

		// PT: TODO: decouple from PxQueryCache?
		virtual	Gu::PrunerHandle	findPrunerHandle(const PxQueryCache& cache, PrunerCompoundId& compoundId, PxU32& prunerIndex)	const	= 0;

		// PT: TODO: return reference? but this version is at least consistent with getActorShape
		virtual	void				getFilterData(const Gu::PrunerPayload& payload, PxFilterData& filterData)	const	= 0;
		virtual	void				getActorShape(const Gu::PrunerPayload& payload, PxActorShape& actorShape)	const	= 0;

		// PT: new for this customized version: a function to perform per-pruner filtering
		virtual	bool				processPruner(PxU32 prunerIndex, const PxQueryThreadContext* context, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall)	const	= 0;
	};
}

	// PT: this is a customized version of physx::Sq::SceneQueries that supports more than 2 hardcoded pruners.
	// It might not be possible to support the whole PxSceneQuerySystem API with an arbitrary number of pruners.
	class ExtSceneQueries
	{
													PX_NOCOPY(ExtSceneQueries)
		public:
													ExtSceneQueries(Sq::ExtPVDCapture* pvd, PxU64 contextID,
														float inflation, const Sq::ExtQueryAdapter& adapter, bool usesTreeOfPruners);
													~ExtSceneQueries();

		PX_FORCE_INLINE	Sq::ExtPrunerManager&		getPrunerManagerFast()			{ return mSQManager;	}
		PX_FORCE_INLINE	const Sq::ExtPrunerManager&	getPrunerManagerFast()	const	{ return mSQManager;	}

		template<typename QueryHit>
						bool						multiQuery(
														const Sq::ExtMultiQueryInput& in,
														PxHitCallback<QueryHit>& hits, PxHitFlags hitFlags, const PxQueryCache*,
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
						Sq::ExtPrunerManager		mSQManager;
		public:
						Gu::CachedFuncs				mCachedFuncs;

						Sq::ExtPVDCapture*			mPVD;
	};

#if PX_SUPPORT_EXTERN_TEMPLATE
	//explicit template instantiation declaration
	extern template
	bool ExtSceneQueries::multiQuery<PxRaycastHit>(const Sq::ExtMultiQueryInput&, PxHitCallback<PxRaycastHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;

	extern template
	bool ExtSceneQueries::multiQuery<PxOverlapHit>(const Sq::ExtMultiQueryInput&, PxHitCallback<PxOverlapHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;

	extern template
	bool ExtSceneQueries::multiQuery<PxSweepHit>(const Sq::ExtMultiQueryInput&, PxHitCallback<PxSweepHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;
#endif

}

#endif
