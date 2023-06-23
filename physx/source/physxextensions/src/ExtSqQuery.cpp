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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ExtSqQuery.h"

using namespace physx;
using namespace Sq;

#include "common/PxProfileZone.h"
#include "foundation/PxFPU.h"
#include "GuBounds.h"
#include "GuIntersectionRayBox.h"
#include "GuIntersectionRay.h"
#include "GuBVH.h"
#include "geometry/PxGeometryQuery.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxTriangleMeshGeometry.h"
//#include "geometry/PxBVH.h"

#include "PxQueryFiltering.h"
#include "PxRigidActor.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

// PT: this is a customized version of physx::Sq::SceneQueries that supports more than 2 hardcoded pruners.
// It might not be possible to support the whole PxSceneQuerySystem API with an arbitrary number of pruners.

// See #MODIFIED tag for what changed in this file compared to the initial code in SqQuery.cpp

static PX_FORCE_INLINE void copy(PxRaycastHit* PX_RESTRICT dest, const PxRaycastHit* PX_RESTRICT src)
{
	dest->faceIndex	= src->faceIndex;
	dest->flags		= src->flags;
	dest->position	= src->position;
	dest->normal	= src->normal;
	dest->distance	= src->distance;
	dest->u			= src->u;
	dest->v			= src->v;
	dest->actor		= src->actor;
	dest->shape		= src->shape;
}

static PX_FORCE_INLINE void copy(PxSweepHit* PX_RESTRICT dest, const PxSweepHit* PX_RESTRICT src)
{
	dest->faceIndex	= src->faceIndex;
	dest->flags		= src->flags;
	dest->position	= src->position;
	dest->normal	= src->normal;
	dest->distance	= src->distance;
	dest->actor		= src->actor;
	dest->shape		= src->shape;
}

static PX_FORCE_INLINE void copy(PxOverlapHit* PX_RESTRICT dest, const PxOverlapHit* PX_RESTRICT src)
{
	dest->faceIndex	= src->faceIndex;
	dest->actor		= src->actor;
	dest->shape		= src->shape;
}

// these partial template specializations are used to generalize the query code to be reused for all permutations of
// hit type=(raycast, overlap, sweep) x query type=(ANY, SINGLE, MULTIPLE)
template <typename HitType> struct HitTypeSupport { enum { IsRaycast = 0, IsSweep = 0, IsOverlap = 0 }; };
template <> struct HitTypeSupport<PxRaycastHit>
{
	enum { IsRaycast = 1, IsSweep = 0, IsOverlap = 0 };
	static PX_FORCE_INLINE PxReal getDistance(const PxQueryHit& hit) { return static_cast<const PxRaycastHit&>(hit).distance; }
};
template <> struct HitTypeSupport<PxSweepHit>
{
	enum { IsRaycast = 0, IsSweep = 1, IsOverlap = 0 };
	static PX_FORCE_INLINE PxReal getDistance(const PxQueryHit& hit) { return static_cast<const PxSweepHit&>(hit).distance; }
};
template <> struct HitTypeSupport<PxOverlapHit>
{
	enum { IsRaycast = 0, IsSweep = 0, IsOverlap = 1 };
	static PX_FORCE_INLINE PxReal getDistance(const PxQueryHit&) { return -1.0f; }
};

#define HITDIST(hit) HitTypeSupport<HitType>::getDistance(hit)

template<typename HitType>
static PxU32 clipHitsToNewMaxDist(HitType* ppuHits, PxU32 count, PxReal newMaxDist)
{
	PxU32 i=0;
	while(i!=count)
	{
		if(HITDIST(ppuHits[i]) > newMaxDist)
			ppuHits[i] = ppuHits[--count];
		else
			i++;
	}
	return count;
}

namespace physx
{
	namespace Sq
	{
	struct ExtMultiQueryInput
	{
		const PxVec3* rayOrigin; // only valid for raycasts
		const PxVec3* unitDir; // only valid for raycasts and sweeps
		PxReal maxDistance; // only valid for raycasts and sweeps
		const PxGeometry* geometry; // only valid for overlaps and sweeps
		const PxTransform* pose; // only valid for overlaps and sweeps
		PxReal inflation; // only valid for sweeps

		// Raycast constructor
		ExtMultiQueryInput(const PxVec3& aRayOrigin, const PxVec3& aUnitDir, PxReal aMaxDist)
		{
			rayOrigin = &aRayOrigin;
			unitDir = &aUnitDir;
			maxDistance = aMaxDist;
			geometry = NULL;
			pose = NULL;
			inflation = 0.0f;
		}

		// Overlap constructor
		ExtMultiQueryInput(const PxGeometry* aGeometry, const PxTransform* aPose)
		{
			geometry = aGeometry;
			pose = aPose;
			inflation = 0.0f;
			rayOrigin = unitDir = NULL;
		}

		// Sweep constructor
		ExtMultiQueryInput(
			const PxGeometry* aGeometry, const PxTransform* aPose,
			const PxVec3& aUnitDir, const PxReal aMaxDist, const PxReal aInflation)
		{
			rayOrigin = NULL;
			maxDistance = aMaxDist;
			unitDir = &aUnitDir;
			geometry = aGeometry;
			pose = aPose;
			inflation = aInflation;
		}

		PX_FORCE_INLINE const PxVec3& getDir() const { PX_ASSERT(unitDir); return *unitDir; }
		PX_FORCE_INLINE const PxVec3& getOrigin() const { PX_ASSERT(rayOrigin); return *rayOrigin; }
	};

	}
}

// performs a single geometry query for any HitType (PxSweepHit, PxOverlapHit, PxRaycastHit)
template<typename HitType>
struct ExtGeomQueryAny
{
	static PX_FORCE_INLINE PxU32 geomHit(
		const CachedFuncs& funcs, const ExtMultiQueryInput& input, const Gu::ShapeData* sd,
		const PxGeometry& sceneGeom, const PxTransform& pose, PxHitFlags hitFlags,
		PxU32 maxHits, HitType* hits, const PxReal shrunkMaxDistance, const PxBounds3* precomputedBounds,
		PxQueryThreadContext* context)
	{
		using namespace Gu;

		const PxGeometry& geom0 = *input.geometry;
		const PxTransform& pose0 = *input.pose;
		const PxGeometry& geom1 = sceneGeom;
		const PxTransform& pose1 = pose;

		// Handle raycasts
		if(HitTypeSupport<HitType>::IsRaycast)
		{
			// the test for mesh AABB is archived in //sw/physx/dev/apokrovsky/graveyard/sqMeshAABBTest.cpp
			// TODO: investigate performance impact (see US12801)
			PX_CHECK_AND_RETURN_VAL(input.getDir().isFinite(), "PxScene::raycast(): rayDir is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(input.getOrigin().isFinite(), "PxScene::raycast(): rayOrigin is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "PxScene::raycast(): pose is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(shrunkMaxDistance >= 0.0f, "PxScene::raycast(): maxDist is negative.", 0);
			PX_CHECK_AND_RETURN_VAL(PxIsFinite(shrunkMaxDistance), "PxScene::raycast(): maxDist is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(PxAbs(input.getDir().magnitudeSquared()-1)<1e-4f,
				"PxScene::raycast(): ray direction must be unit vector.", 0);

			// PT: TODO: investigate perf difference
			const RaycastFunc func = funcs.mCachedRaycastFuncs[geom1.getType()];
			return func(geom1, pose1, input.getOrigin(), input.getDir(), shrunkMaxDistance,
						hitFlags, maxHits, reinterpret_cast<PxGeomRaycastHit*>(hits), sizeof(PxRaycastHit), context);
		}
		// Handle sweeps
		else if(HitTypeSupport<HitType>::IsSweep)
		{
			PX_ASSERT(precomputedBounds != NULL);
			PX_ASSERT(sd != NULL);
			// b0 = query shape bounds
			// b1 = scene shape bounds
			// AP: Here we clip the sweep to bounds with sum of extents. This is needed for GJK stability.
			// because sweep is equivalent to a raycast vs a scene shape with inflated bounds.
			// This also may (or may not) provide an optimization for meshes because top level of rtree has multiple boxes
			// and there is no bounds test for the whole mesh elsewhere
			PxBounds3 b0 = *precomputedBounds, b1;
			// compute the scene geometry bounds
			// PT: TODO: avoid recomputing the bounds here
			Gu::computeBounds(b1, sceneGeom, pose, 0.0f, 1.0f);
			const PxVec3 combExt = (b0.getExtents() + b1.getExtents())*1.01f;

			PxF32 tnear, tfar;
			if(!intersectRayAABB2(-combExt, combExt, b0.getCenter() - b1.getCenter(), input.getDir(), shrunkMaxDistance, tnear, tfar)) // returns (tnear<tfar)
				if(tnear>tfar) // this second test is needed because shrunkMaxDistance can be 0 for 0 length sweep
					return 0;
			PX_ASSERT(input.getDir().isNormalized());
			// tfar is now the t where the ray exits the AABB. input.getDir() is normalized

			const PxVec3& unitDir = input.getDir();
			PxSweepHit& sweepHit = reinterpret_cast<PxSweepHit&>(hits[0]);
			
			// if we don't start inside the AABB box, offset the start pos, because of precision issues with large maxDist
			const bool offsetPos = (tnear > GU_RAY_SURFACE_OFFSET);
			const PxReal offset = offsetPos ? (tnear - GU_RAY_SURFACE_OFFSET) : 0.0f;
			const PxVec3 offsetVec(offsetPos ? (unitDir*offset) : PxVec3(0.0f));
			// we move the geometry we sweep against, so that we avoid the Gu::Capsule/Box recomputation
			const PxTransform pose1Offset(pose1.p - offsetVec, pose1.q);
            
			const PxReal distance = PxMin(tfar, shrunkMaxDistance) - offset;
			const PxReal inflation = input.inflation;
			PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "PxScene::sweep(): pose0 is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(pose1Offset.isValid(), "PxScene::sweep(): pose1 is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(unitDir.isFinite(), "PxScene::sweep(): unitDir is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(PxIsFinite(distance), "PxScene::sweep(): distance is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL((distance >= 0.0f && !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP)) || distance > 0.0f,
				"PxScene::sweep(): sweep distance must be >=0 or >0 with eASSUME_NO_INITIAL_OVERLAP.", 0);

			PxU32 retVal = 0;
			const GeomSweepFuncs& sf = funcs.mCachedSweepFuncs;
			switch(geom0.getType())
			{
				case PxGeometryType::eSPHERE:
				{
					const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
					const PxCapsuleGeometry capsuleGeom(sphereGeom.radius, 0.0f);
					const Capsule worldCapsule(pose0.p, pose0.p, sphereGeom.radius); // AP: precompute?
					const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
					const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, capsuleGeom, pose0, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation, context));
				}
				break;

				case PxGeometryType::eCAPSULE:
				{
					const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
					const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, static_cast<const PxCapsuleGeometry&>(geom0), pose0, sd->getGuCapsule(), unitDir, distance, sweepHit, hitFlags, inflation, context));
				}
				break;
	
				case PxGeometryType::eBOX:
				{
					const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
					const SweepBoxFunc func = precise ? sf.preciseBoxMap[geom1.getType()] : sf.boxMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, static_cast<const PxBoxGeometry&>(geom0), pose0, sd->getGuBox(), unitDir, distance, sweepHit, hitFlags, inflation, context));
				}
				break;
	
				case PxGeometryType::eCONVEXMESH:
				{
					const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
					const SweepConvexFunc func = sf.convexMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, convexGeom, pose0, unitDir, distance, sweepHit, hitFlags, inflation, context));
				}
				break;
				default:
					outputError<physx::PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxScene::sweep(): first geometry object parameter must be sphere, capsule, box or convex geometry.");
				break;
			}
			if (retVal)
			{
				// we need to offset the distance back
				sweepHit.distance += offset;
				// we need to offset the hit position back as we moved the geometry we sweep against
				sweepHit.position += offsetVec;
			}
			return retVal;
		}
		// Handle overlaps
		else if(HitTypeSupport<HitType>::IsOverlap)
		{
			const GeomOverlapTable* overlapFuncs = funcs.mCachedOverlapFuncs;
			return PxU32(Gu::overlap(geom0, pose0, geom1, pose1, overlapFuncs, context));
		}
		else
		{
			PX_ALWAYS_ASSERT_MESSAGE("Unexpected template expansion in GeomQueryAny::geomHit");
			return 0;
		}
	}
};

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE bool applyFilterEquation(const ExtQueryAdapter& adapter, const PrunerPayload& payload, const PxFilterData& queryFd)
{
	// if the filterData field is non-zero, and the bitwise-AND value of filterData AND the shape's
	// queryFilterData is zero, the shape is skipped.
	if(queryFd.word0 | queryFd.word1 | queryFd.word2 | queryFd.word3)
	{
		// PT: TODO: revisit this, there's an obvious LHS here otherwise
		// We could maybe make this more flexible and let the user do the filtering
//		const PxFilterData& objFd = adapter.getFilterData(payload);
		PxFilterData objFd;
		adapter.getFilterData(payload, objFd);

		const PxU32 keep = (queryFd.word0 & objFd.word0) | (queryFd.word1 & objFd.word1) | (queryFd.word2 & objFd.word2) | (queryFd.word3 & objFd.word3);
		if(!keep)
			return false;
	}
	return true;
}

static PX_FORCE_INLINE bool applyAllPreFiltersSQ(
	const ExtQueryAdapter& adapter, const PrunerPayload& payload, const PxActorShape& as, PxQueryHitType::Enum& shapeHitType, const PxQueryFlags& inFilterFlags,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	PxHitFlags& queryFlags/*, PxU32 maxNbTouches*/)
{
	// #MODIFIED
	// PT: we have to do the static / dynamic filtering here now, because we're operating on N pruners
	// and we don't know which one(s) are "static", which ones are "dynamic", and which ones are a mix of both.
	const bool doStatics = filterData.flags & PxQueryFlag::eSTATIC;
	const bool doDynamic = filterData.flags & PxQueryFlag::eDYNAMIC;
	const PxType actorType = as.actor->getConcreteType();
	const bool isStatic = (actorType == PxConcreteType::eRIGID_STATIC);
	if(isStatic && !doStatics)
		return false;
	if(!isStatic && !doDynamic)
		return false;
	//~#MODIFIED

	if(!(filterData.flags & PxQueryFlag::eBATCH_QUERY_LEGACY_BEHAVIOUR) && !applyFilterEquation(adapter, payload, filterData.data))
		return false;

	if((inFilterFlags & PxQueryFlag::ePREFILTER) && (filterCall))
	{
		PxHitFlags outQueryFlags = queryFlags;

		if(filterCall)
			shapeHitType = filterCall->preFilter(filterData.data, as.shape, as.actor, outQueryFlags);

		// AP: at this point the callback might return eTOUCH but the touch buffer can be empty, the hit will be discarded
		//PX_CHECK_MSG(hitType == PxQueryHitType::eTOUCH ? maxNbTouches > 0 : true,
		//	"SceneQuery: preFilter returned eTOUCH but empty touch buffer was provided, hit discarded.");

		queryFlags = (queryFlags & ~PxHitFlag::eMODIFIABLE_FLAGS) | (outQueryFlags & PxHitFlag::eMODIFIABLE_FLAGS);

		if(shapeHitType == PxQueryHitType::eNONE)
			return false;
	}
	// test passed, continue to return as;
	return true;
}

static PX_NOINLINE void computeCompoundShapeTransform(PxTransform* PX_RESTRICT transform, const PxTransform* PX_RESTRICT compoundPose, const PxTransform* PX_RESTRICT transforms, PxU32 primIndex)
{
	// PT:: tag: scalar transform*transform
	*transform = (*compoundPose) * transforms[primIndex];
}

// struct to access protected data members in the public PxHitCallback API
template<typename HitType>
struct ExtMultiQueryCallback : public PrunerRaycastCallback, public PrunerOverlapCallback, public CompoundPrunerRaycastCallback, public CompoundPrunerOverlapCallback
{
	const ExtSceneQueries&		mScene;
	const ExtMultiQueryInput&	mInput;
	PxHitCallback<HitType>&		mHitCall;
	const PxHitFlags			mHitFlags;
	const PxQueryFilterData&	mFilterData;
	PxQueryFilterCallback*		mFilterCall;
	PxReal						mShrunkDistance;
	const PxHitFlags			mMeshAnyHitFlags;
	bool						mReportTouchesAgain;
	bool						mFarBlockFound; // this is to prevent repeated searches for far block
	const bool					mNoBlock;
	const bool					mAnyHit;

	// The reason we need these bounds is because we need to know combined(inflated shape) bounds to clip the sweep path
	// to be tolerable by GJK precision issues. This test is done for (queryShape vs touchedShapes)
	// So it makes sense to cache the bounds for sweep query shape, otherwise we'd have to recompute them every time
	// Currently only used for sweeps.
	const PxBounds3*			mQueryShapeBounds;
	const ShapeData*			mShapeData;
	PxTransform					mCompoundShapeTransform;

	ExtMultiQueryCallback(
		const ExtSceneQueries& scene, const ExtMultiQueryInput& input, bool anyHit, PxHitCallback<HitType>& hitCall, PxHitFlags hitFlags,
		const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall, PxReal shrunkDistance) :
			mScene				(scene),
			mInput				(input),
			mHitCall			(hitCall),
			mHitFlags			(hitFlags),
			mFilterData			(filterData),
			mFilterCall			(filterCall),
			mShrunkDistance		(shrunkDistance),
			mMeshAnyHitFlags	((hitFlags.isSet(PxHitFlag::eMESH_ANY) || anyHit) ? PxHitFlag::eMESH_ANY : PxHitFlag::Enum(0)),
			mReportTouchesAgain	(true),
			mFarBlockFound		(filterData.flags & PxQueryFlag::eNO_BLOCK),
			mNoBlock			(filterData.flags & PxQueryFlag::eNO_BLOCK),
			mAnyHit				(anyHit),
			mQueryShapeBounds	(NULL),
			mShapeData			(NULL)
	{
	}

	bool processTouchHit(const HitType& hit, PxReal& aDist)
#if PX_WINDOWS_FAMILY
		PX_RESTRICT
#endif
	{
		// -------------------------- handle eTOUCH hits ---------------------------------
		// for qType=multiple, store the hit. For other qTypes ignore it.
		// <= is important for initially overlapping sweeps
		#if PX_CHECKED
		if(mHitCall.maxNbTouches == 0 && !mFilterData.flags.isSet(PxQueryFlag::eRESERVED))
			// issue a warning if eTOUCH was returned by the prefilter, we have 0 touch buffer and not a batch query
			// not doing for BQ because the touches buffer can be overflown and thats ok by spec
			// eRESERVED to avoid a warning from nested callback (closest blocking hit recursive search)
			outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "User filter returned PxQueryHitType::eTOUCH but the touches buffer was empty. Hit was discarded.");
		#endif

		if(mHitCall.maxNbTouches && mReportTouchesAgain && HITDIST(hit) <= mShrunkDistance)
		{
			// Buffer full: need to find the closest blocking hit, clip touch hits and flush the buffer
			if(mHitCall.nbTouches == mHitCall.maxNbTouches)
			{
				// issue a second nested query just looking for the closest blocking hit
				// could do better perf-wise by saving traversal state (start looking for blocking from this point)
				// but this is not a perf critical case because users can provide a bigger buffer
				// that covers non-degenerate cases
				// far block search doesn't apply to overlaps because overlaps don't work with blocking hits
				if(HitTypeSupport<HitType>::IsOverlap == 0)
				{
					// AP: the use of eRESERVED is a bit tricky, see other comments containing #LABEL1
					PxQueryFilterData fd1 = mFilterData; fd1.flags |= PxQueryFlag::eRESERVED;
					PxHitBuffer<HitType> buf1; // create a temp callback buffer for a single blocking hit
					if(!mFarBlockFound && mHitCall.maxNbTouches > 0 && mScene.ExtSceneQueries::multiQuery<HitType>(mInput, buf1, mHitFlags, NULL, fd1, mFilterCall))
					{
						mHitCall.block = buf1.block;
						mHitCall.hasBlock = true;
						mHitCall.nbTouches =
							clipHitsToNewMaxDist<HitType>(mHitCall.touches, mHitCall.nbTouches, HITDIST(buf1.block));
						mShrunkDistance = HITDIST(buf1.block);
						aDist = mShrunkDistance;
					}
					mFarBlockFound = true;
				}
				if(mHitCall.nbTouches == mHitCall.maxNbTouches)
				{
					mReportTouchesAgain = mHitCall.processTouches(mHitCall.touches, mHitCall.nbTouches);
					if(!mReportTouchesAgain)
						return false; // optimization - buffer is full 
					else
						mHitCall.nbTouches = 0; // reset nbTouches so we can continue accumulating again
				}
			}

			//if(hitCall.nbTouches < hitCall.maxNbTouches) // can be true if maxNbTouches is 0
			mHitCall.touches[mHitCall.nbTouches++] = hit;
		} // if(hitCall.maxNbTouches && reportTouchesAgain && HITDIST(hit) <= shrunkDistance)

		return true;
	}

	template<const bool isCached>	// is this call coming as a callback from the pruner or a single item cached callback?
	bool _invoke(PxReal& aDist, PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms, const PxTransform* compoundPose)
#if PX_WINDOWS_FAMILY
		PX_RESTRICT
#endif
	{
		PX_ASSERT(payloads);
		const PrunerPayload& payload = payloads[primIndex];
		const ExtQueryAdapter& adapter = static_cast<const ExtQueryAdapter&>(mScene.mSQManager.getAdapter());

		PxActorShape actorShape;
		adapter.getActorShape(payload, actorShape);

		const PxQueryFlags filterFlags = mFilterData.flags;

		// for no filter callback, default to eTOUCH for MULTIPLE, eBLOCK otherwise
		// also always treat as eBLOCK if currently tested shape is cached
		// Using eRESERVED flag as a special condition to default to eTOUCH hits while only looking for a single blocking hit
		// from a nested query (see other comments containing #LABEL1)
		PxQueryHitType::Enum shapeHitType =
			((mHitCall.maxNbTouches || (mFilterData.flags & PxQueryFlag::eRESERVED)) && !isCached)
				? PxQueryHitType::eTOUCH
				: PxQueryHitType::eBLOCK;

		// apply pre-filter
		PxHitFlags filteredHitFlags = mHitFlags;
		if(!isCached) // don't run filters on single item cache
		{
			if(!applyAllPreFiltersSQ(adapter, payload, actorShape, shapeHitType/*in&out*/, filterFlags, mFilterData, mFilterCall, filteredHitFlags/*, mHitCall.maxNbTouches*/))
				return true; // skip this shape from reporting if prefilter said to do so

//			if(shapeHitType == PxQueryHitType::eNONE)
//				return true;
		}

		const PxGeometry& shapeGeom = adapter.getGeometry(payload);

		PX_ASSERT(transforms);
		const PxTransform* shapeTransform;
		if(!compoundPose)
		{
			shapeTransform = transforms + primIndex;
		}
		else
		{
			computeCompoundShapeTransform(&mCompoundShapeTransform, compoundPose, transforms, primIndex);
			shapeTransform = &mCompoundShapeTransform;
		}

		const PxU32 tempCount = 1;
		HitType tempBuf[tempCount];

		// Here we decide whether to use the user provided buffer in place or a local stack buffer
		// see if we have more room left in the callback results buffer than in the parent stack buffer
		// if so get subHits in-place in the hit buffer instead of the parent stack buffer
		// nbTouches is the number of accumulated touch hits so far
		// maxNbTouches is the size of the user buffer
		PxU32 maxSubHits1;
		HitType* subHits1;
		if(mHitCall.nbTouches >= mHitCall.maxNbTouches)
		// if there's no room left in the user buffer, use a stack buffer
		{
			// tried using 64 here - causes check stack code to get generated on xbox, perhaps because of guard page
			// need this buffer in case the input buffer is full but we still want to correctly merge results from later hits
			maxSubHits1 = tempCount;
			subHits1 = reinterpret_cast<HitType*>(tempBuf);
		}
		else
		{
			maxSubHits1 = mHitCall.maxNbTouches - mHitCall.nbTouches;	// how much room is left in the user buffer
			subHits1 = mHitCall.touches + mHitCall.nbTouches;			// pointer to the first free hit in the user buffer
		}

		// call the geometry specific intersection template
		const PxU32 nbSubHits = ExtGeomQueryAny<HitType>::geomHit(
			mScene.mCachedFuncs, mInput, mShapeData, shapeGeom,
			*shapeTransform, filteredHitFlags | mMeshAnyHitFlags,
			maxSubHits1, subHits1, mShrunkDistance, mQueryShapeBounds, &mHitCall);

		// ------------------------- iterate over geometry subhits -----------------------------------
		for (PxU32 iSubHit = 0; iSubHit < nbSubHits; iSubHit++)
		{
			HitType& hit = subHits1[iSubHit];
			hit.actor = actorShape.actor;
			hit.shape = actorShape.shape;

			// some additional processing only for sweep hits with initial overlap
			if(HitTypeSupport<HitType>::IsSweep && HITDIST(hit) == 0.0f && !(filteredHitFlags & PxHitFlag::eMTD))
				// PT: necessary as some leaf routines are called with reversed params, thus writing +unitDir there.
				// AP: apparently still necessary to also do in Gu because Gu can be used standalone (without SQ)
				reinterpret_cast<PxSweepHit&>(hit).normal = -mInput.getDir();

			// start out with hitType for this cached shape set to a pre-filtered hit type
			PxQueryHitType::Enum hitType = shapeHitType;

			// run the post-filter if specified in filterFlags and filterCall is non-NULL
			if(!isCached && mFilterCall && (filterFlags & PxQueryFlag::ePOSTFILTER))
			{
				//if(mFilterCall)
					hitType = mFilterCall->postFilter(mFilterData.data, hit, hit.shape, hit.actor);
			}

			// early out on any hit if eANY_HIT was specified, regardless of hit type
			if(mAnyHit && hitType != PxQueryHitType::eNONE)
			{
				// block or touch qualifies for qType=ANY type hit => return it as blocking according to spec. Ignore eNONE.
				//mHitCall.block = hit;
				copy(&mHitCall.block, &hit);
				mHitCall.hasBlock = true;
				return false; // found a hit for ANY qType, can early exit now
			}

			if(mNoBlock && hitType==PxQueryHitType::eBLOCK)
				hitType = PxQueryHitType::eTOUCH;

			PX_WARN_ONCE_IF(HitTypeSupport<HitType>::IsOverlap && hitType == PxQueryHitType::eBLOCK, 
				"eBLOCK returned from user filter for overlap() query. This may cause undesired behavior. "
				"Consider using PxQueryFlag::eNO_BLOCK for overlap queries.");

			if(hitType == PxQueryHitType::eTOUCH)
			{
				if(!processTouchHit(hit, aDist))
					return false;
			} // if(hitType == PxQueryHitType::eTOUCH)
			else if(hitType == PxQueryHitType::eBLOCK)
			{
				// -------------------------- handle eBLOCK hits ----------------------------------
				// only eBLOCK qualifies as a closest hit candidate => compare against best distance and store
				// <= is needed for eTOUCH hits to be recorded correctly vs same eBLOCK distance for overlaps
				if(HITDIST(hit) <= mShrunkDistance)
				{
					if(HitTypeSupport<HitType>::IsOverlap == 0)
					{
						mShrunkDistance = HITDIST(hit);
						aDist = mShrunkDistance;
					}
					//mHitCall.block = hit;
					copy(&mHitCall.block, &hit);
					mHitCall.hasBlock = true;
				}
			} // if(hitType == eBLOCK)
			else {
				PX_ASSERT(hitType == PxQueryHitType::eNONE);
			}
		} // for iSubHit
		return true;
	}

	virtual bool	invoke(PxReal& aDist, PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms)
	{
		return _invoke<false>(aDist, primIndex, payloads, transforms, NULL);
	}

	virtual bool	invoke(PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms)
	{
		float unused = 0.0f;
		return _invoke<false>(unused, primIndex, payloads, transforms, NULL);
	}

	virtual bool	invoke(PxReal& aDist, PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms, const PxTransform* compoundPose)
	{
		return _invoke<false>(aDist, primIndex, payloads, transforms, compoundPose);
	}

	virtual bool	invoke(PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms, const PxTransform* compoundPose)
	{
		float unused = 0.0f;
		return _invoke<false>(unused, primIndex, payloads, transforms, compoundPose);
	}

private:
	ExtMultiQueryCallback<HitType>& operator=(const ExtMultiQueryCallback<HitType>&);
};

//========================================================================================================================
#if PX_SUPPORT_PVD
template<typename HitType>
struct ExtCapturePvdOnReturn : public PxHitCallback<HitType>
{
	// copy the arguments of multiQuery into a struct, this is strictly for PVD recording
	const ExtSceneQueries*		mSQ;
	const ExtMultiQueryInput&	mInput;
	const PxQueryFilterData&	mFilterData;
	PxArray<HitType>			mAllHits;
	PxHitCallback<HitType>&		mParentCallback;

	ExtCapturePvdOnReturn(
		const ExtSceneQueries* sq, const ExtMultiQueryInput& input,
		const PxQueryFilterData& filterData,
		PxHitCallback<HitType>& parentCallback) :
		PxHitCallback<HitType>	(parentCallback.touches, parentCallback.maxNbTouches),
		mSQ						(sq),
		mInput					(input),
		mFilterData				(filterData),
		mParentCallback			(parentCallback)
	{}

	virtual PxAgain processTouches(const HitType* hits, PxU32 nbHits)
	{
		const PxAgain again = mParentCallback.processTouches(hits, nbHits);
		for(PxU32 i=0; i<nbHits; i++)
			mAllHits.pushBack(hits[i]);
		return again;
	}

	~ExtCapturePvdOnReturn()
	{
		ExtPVDCapture* pvd = mSQ->mPVD;
		if(!pvd || !pvd->transmitSceneQueries())
			return;

		if(mParentCallback.nbTouches)
		{
			for(PxU32 i = 0; i < mParentCallback.nbTouches; i++)
				mAllHits.pushBack(mParentCallback.touches[i]);
		}

		if(mParentCallback.hasBlock)
			mAllHits.pushBack(mParentCallback.block);

		// PT: TODO: why do we need reinterpret_casts below?
		if(HitTypeSupport<HitType>::IsRaycast)
			pvd->raycast(mInput.getOrigin(), mInput.getDir(), mInput.maxDistance, reinterpret_cast<PxRaycastHit*>(mAllHits.begin()), mAllHits.size(), mFilterData, this->maxNbTouches!=0);
		else if(HitTypeSupport<HitType>::IsOverlap)
			pvd->overlap(*mInput.geometry, *mInput.pose, reinterpret_cast<PxOverlapHit*>(mAllHits.begin()), mAllHits.size(), mFilterData);
		else if(HitTypeSupport<HitType>::IsSweep)
			pvd->sweep	(*mInput.geometry, *mInput.pose, mInput.getDir(), mInput.maxDistance, reinterpret_cast<PxSweepHit*>(mAllHits.begin()), mAllHits.size(), mFilterData, this->maxNbTouches!=0);
	}

private:
	ExtCapturePvdOnReturn<HitType>& operator=(const ExtCapturePvdOnReturn<HitType>&);
};
#endif // PX_SUPPORT_PVD

//========================================================================================================================
template<typename HitType>
struct ExtIssueCallbacksOnReturn
{
	PxHitCallback<HitType>& hits;
	bool again;	// query was stopped by previous processTouches. This means that nbTouches is still non-zero
				// but we don't need to issue processTouches again
	PX_FORCE_INLINE ExtIssueCallbacksOnReturn(PxHitCallback<HitType>& aHits) : hits(aHits)
	{
		again = true;
	}

	~ExtIssueCallbacksOnReturn()
	{
		if(again)
			// only issue processTouches if query wasn't stopped
			// this is because nbTouches doesn't get reset to 0 in this case (according to spec)
			// and the touches in touches array were already processed by the callback
		{
			if(hits.hasBlock && hits.nbTouches)
				hits.nbTouches = clipHitsToNewMaxDist<HitType>(hits.touches, hits.nbTouches, HITDIST(hits.block));
			if(hits.nbTouches)
			{
				bool again_ = hits.processTouches(hits.touches, hits.nbTouches);
				if(again_)
					hits.nbTouches = 0;
			}
		}
		hits.finalizeQuery();
	}

private:
	ExtIssueCallbacksOnReturn<HitType>& operator=(const ExtIssueCallbacksOnReturn<HitType>&);
};

#undef HITDIST

//========================================================================================================================

template<typename HitType>
static bool doQueryVsCached(const PrunerHandle handle, PxU32 prunerIndex, const PrunerCompoundId cachedCompoundId, const ExtPrunerManager& manager, ExtMultiQueryCallback<HitType>& pcb, const ExtMultiQueryInput& input);

static PX_FORCE_INLINE PxCompoundPrunerQueryFlags convertFlags(PxQueryFlags	inFlags)
{
	PxCompoundPrunerQueryFlags outFlags(0);
	if(inFlags.isSet(PxQueryFlag::eSTATIC))
		outFlags.raise(PxCompoundPrunerQueryFlag::eSTATIC);
	if(inFlags.isSet(PxQueryFlag::eDYNAMIC))
		outFlags.raise(PxCompoundPrunerQueryFlag::eDYNAMIC);
	return outFlags;
}

// #MODIFIED
static PX_FORCE_INLINE bool prunerFilter(const ExtQueryAdapter& adapter, PxU32 prunerIndex, const PxQueryThreadContext* context, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall)
{
	// PT: the internal PhysX code can skip an entire pruner by just testing one query flag, since there is a direct
	// mapping between the static/dynamic flags and the static/dynamic pruners. This is not the case here anymore,
	// so instead we call a user-provided callback to validate processing each pruner.
	return adapter.processPruner(prunerIndex, context, filterData, filterCall);
}
//~#MODIFIED

// PT: the following local callbacks are for the "tree of pruners"
template<typename HitType>
struct LocalBaseCallback
{
	LocalBaseCallback(ExtMultiQueryCallback<HitType>& pcb, const Sq::ExtPrunerManager& manager, const ExtQueryAdapter& adapter, PxHitCallback<HitType>& hits, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) :
		mPCB		(pcb),
		mSQManager	(manager),
		mAdapter	(adapter),
		mHits		(hits),
		mFilterData	(filterData),
		mFilterCall	(filterCall)
	{}

	ExtMultiQueryCallback<HitType>&	mPCB;
	const Sq::ExtPrunerManager&		mSQManager;
	const ExtQueryAdapter&			mAdapter;
	PxHitCallback<HitType>&			mHits;
	const PxQueryFilterData&		mFilterData;
	PxQueryFilterCallback*			mFilterCall;

	PX_FORCE_INLINE	const Pruner* filtering(PxU32 prunerIndex)
	{
		if(!prunerFilter(mAdapter, prunerIndex, &mHits, mFilterData, mFilterCall))
			return NULL;

		return mSQManager.getPruner(prunerIndex);
	}

	PX_NOCOPY(LocalBaseCallback)
};

template<typename HitType>
struct LocalRaycastCallback : LocalBaseCallback<HitType>,  PxBVH::RaycastCallback
{
	LocalRaycastCallback(const ExtMultiQueryInput& input, ExtMultiQueryCallback<HitType>& pcb, const Sq::ExtPrunerManager& manager, const ExtQueryAdapter& adapter, PxHitCallback<HitType>& hits, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) :
		LocalBaseCallback<HitType>(pcb, manager, adapter, hits, filterData, filterCall), mInput(input)	{}

	virtual bool	reportHit(PxU32 boundsIndex, PxReal& distance)
	{
		const Pruner* pruner = LocalBaseCallback<HitType>::filtering(boundsIndex);
		if(!pruner)
			return true;
		return pruner->raycast(mInput.getOrigin(), mInput.getDir(), distance, this->mPCB);
	}

	const ExtMultiQueryInput&	mInput;

	PX_NOCOPY(LocalRaycastCallback)
};

template<typename HitType>
struct LocalOverlapCallback : LocalBaseCallback<HitType>,  PxBVH::OverlapCallback
{
	LocalOverlapCallback(const ShapeData& shapeData, ExtMultiQueryCallback<HitType>& pcb, const Sq::ExtPrunerManager& manager, const ExtQueryAdapter& adapter, PxHitCallback<HitType>& hits, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) :
		LocalBaseCallback<HitType>(pcb, manager, adapter, hits, filterData, filterCall), mShapeData(shapeData)	{}

	virtual bool	reportHit(PxU32 boundsIndex)
	{
		const Pruner* pruner = LocalBaseCallback<HitType>::filtering(boundsIndex);
		if(!pruner)
			return true;
		return pruner->overlap(mShapeData, this->mPCB);
	}

	const ShapeData&	mShapeData;

	PX_NOCOPY(LocalOverlapCallback)
};

template<typename HitType>
struct LocalSweepCallback : LocalBaseCallback<HitType>,  PxBVH::RaycastCallback
{
	LocalSweepCallback(const ShapeData& shapeData, const PxVec3& dir, ExtMultiQueryCallback<HitType>& pcb, const Sq::ExtPrunerManager& manager, const ExtQueryAdapter& adapter, PxHitCallback<HitType>& hits, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) :
		LocalBaseCallback<HitType>(pcb, manager, adapter, hits, filterData, filterCall), mShapeData(shapeData), mDir(dir)	{}

	virtual bool	reportHit(PxU32 boundsIndex, PxReal& distance)
	{
		const Pruner* pruner = LocalBaseCallback<HitType>::filtering(boundsIndex);
		if(!pruner)
			return true;
		return pruner->sweep(mShapeData, mDir, distance, this->mPCB);
	}

	const ShapeData&	mShapeData;
	const PxVec3&		mDir;

	PX_NOCOPY(LocalSweepCallback)
};

// PT: TODO: revisit error messages without breaking UTs
template<typename HitType>
bool ExtSceneQueries::multiQuery(
	const ExtMultiQueryInput& input, PxHitCallback<HitType>& hits, PxHitFlags hitFlags, const PxQueryCache* cache,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) const
{
	const bool anyHit = (filterData.flags & PxQueryFlag::eANY_HIT) == PxQueryFlag::eANY_HIT;

	if(HitTypeSupport<HitType>::IsRaycast == 0)
	{
		PX_CHECK_AND_RETURN_VAL(input.pose != NULL, "NpSceneQueries::overlap/sweep pose is NULL.", 0);
		PX_CHECK_AND_RETURN_VAL(input.pose->isValid(), "NpSceneQueries::overlap/sweep pose is not valid.", 0);
	}
	else
	{
		PX_CHECK_AND_RETURN_VAL(input.getOrigin().isFinite(), "NpSceneQueries::raycast pose is not valid.", 0);
	}

	if(HitTypeSupport<HitType>::IsOverlap == 0)
	{
		PX_CHECK_AND_RETURN_VAL(input.getDir().isFinite(), "NpSceneQueries multiQuery input check: unitDir is not valid.", 0);
		PX_CHECK_AND_RETURN_VAL(input.getDir().isNormalized(), "NpSceneQueries multiQuery input check: direction must be normalized", 0);
	}

	if(HitTypeSupport<HitType>::IsRaycast)
	{
		PX_CHECK_AND_RETURN_VAL(input.maxDistance > 0.0f, "NpSceneQueries::multiQuery input check: distance cannot be negative or zero", 0);
	}

	if(HitTypeSupport<HitType>::IsOverlap && !anyHit)
	{
		PX_CHECK_AND_RETURN_VAL(hits.maxNbTouches > 0, "PxScene::overlap() calls without eANY_HIT flag require a touch hit buffer for return results.", 0);
	}

	if(HitTypeSupport<HitType>::IsSweep)
	{
		PX_CHECK_AND_RETURN_VAL(input.maxDistance >= 0.0f, "NpSceneQueries multiQuery input check: distance cannot be negative", 0);
		PX_CHECK_AND_RETURN_VAL(input.maxDistance != 0.0f || !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP),
			"NpSceneQueries multiQuery input check: zero-length sweep only valid without the PxHitFlag::eASSUME_NO_INITIAL_OVERLAP flag", 0);
	}

	PX_CHECK_MSG(!cache || (cache && cache->shape && cache->actor), "Raycast cache specified but shape or actor pointer is NULL!");
	PrunerCompoundId cachedCompoundId = INVALID_COMPOUND_ID;
	// PT: this is similar to the code in the SqRefFinder so we could share that code maybe. But here we later retrieve the payload from the PrunerData,
	// i.e. we basically go back to the same pointers we started from. I suppose it's to make sure they get properly invalidated when an object is deleted etc,
	// but we could still probably find a more efficient way to do that here. Isn't it exactly why we had the Signature class initially?
	//
	// how can this work anyway? if the actor has been deleted the lookup won't work either => doc says it's up to users to manage that....
	const ExtQueryAdapter& adapter = static_cast<const ExtQueryAdapter&>(mSQManager.getAdapter());
	PxU32 prunerIndex = 0xffffffff;
	const PrunerHandle cacheData = cache ? adapter.findPrunerHandle(*cache, cachedCompoundId, prunerIndex) : INVALID_PRUNERHANDLE;

	// this function is logically const for the SDK user, as flushUpdates() will not have an API-visible effect on this object
	// internally however, flushUpdates() changes the states of the Pruners in mSQManager
	// because here is the only place we need this, const_cast instead of making SQM mutable
	const_cast<ExtSceneQueries*>(this)->mSQManager.flushUpdates();

#if PX_SUPPORT_PVD
	ExtCapturePvdOnReturn<HitType> pvdCapture(this, input, filterData, hits);
#endif

	ExtIssueCallbacksOnReturn<HitType> cbr(hits); // destructor will execute callbacks on return from this function
	hits.hasBlock = false;
	hits.nbTouches = 0;

	PxReal shrunkDistance = HitTypeSupport<HitType>::IsOverlap ? PX_MAX_REAL : input.maxDistance; // can be progressively shrunk as we go over the list of shapes
	if(HitTypeSupport<HitType>::IsSweep)
		shrunkDistance = PxMin(shrunkDistance, PX_MAX_SWEEP_DISTANCE);

	ExtMultiQueryCallback<HitType> pcb(*this, input, anyHit, hits, hitFlags, filterData, filterCall, shrunkDistance);

	if(cacheData!=INVALID_PRUNERHANDLE && hits.maxNbTouches == 0) // don't use cache for queries that can return touch hits
	{
		if(!doQueryVsCached(cacheData, prunerIndex, cachedCompoundId, mSQManager, pcb, input))
			return hits.hasAnyHits();
	}

	const PxU32 nbPruners = mSQManager.getNbPruners();
	const CompoundPruner* compoundPruner = mSQManager.getCompoundPruner();

	const PxCompoundPrunerQueryFlags compoundPrunerQueryFlags = convertFlags(filterData.flags);

	const BVH* treeOfPruners = mSQManager.getTreeOfPruners();

	if(HitTypeSupport<HitType>::IsRaycast)
	{
		// #MODIFIED
		bool again = true;
		if(treeOfPruners)
		{
			LocalRaycastCallback<HitType> prunerRaycastCB(input, pcb, mSQManager, adapter, hits, filterData, filterCall);
			again = treeOfPruners->raycast(input.getOrigin(), input.getDir(), pcb.mShrunkDistance, prunerRaycastCB, PxGeometryQueryFlag::Enum(0));
			if(!again)
			{
				cbr.again = again; // update the status to avoid duplicate processTouches()
				return hits.hasAnyHits();
			}
		}
		else
		{
			for(PxU32 i=0;i<nbPruners;i++)
			{
				if(prunerFilter(adapter, i, &hits, filterData, filterCall))
				{
					const Pruner* pruner = mSQManager.getPruner(i);
					again = pruner->raycast(input.getOrigin(), input.getDir(), pcb.mShrunkDistance, pcb);
					if(!again)
					{
						cbr.again = again; // update the status to avoid duplicate processTouches()
						return hits.hasAnyHits();
					}
				}
			}
		}
		//~#MODIFIED

		if(again && compoundPruner)
			again = compoundPruner->raycast(input.getOrigin(), input.getDir(), pcb.mShrunkDistance, pcb, compoundPrunerQueryFlags);

		cbr.again = again; // update the status to avoid duplicate processTouches()
		return hits.hasAnyHits();
	}
	else if(HitTypeSupport<HitType>::IsOverlap)
	{
		PX_ASSERT(input.geometry);

		const ShapeData sd(*input.geometry, *input.pose, input.inflation);
		pcb.mShapeData = &sd;

		// #MODIFIED
		bool again = true;
		if(treeOfPruners)
		{
			LocalOverlapCallback<HitType> prunerOverlapCB(sd, pcb, mSQManager, adapter, hits, filterData, filterCall);
			again = treeOfPruners->overlap(*input.geometry, *input.pose, prunerOverlapCB, PxGeometryQueryFlag::Enum(0));
			if(!again)
			{
				cbr.again = again; // update the status to avoid duplicate processTouches()
				return hits.hasAnyHits();
			}
		}
		else
		{
			for(PxU32 i=0;i<nbPruners;i++)
			{
				if(prunerFilter(adapter, i, &hits, filterData, filterCall))
				{
					const Pruner* pruner = mSQManager.getPruner(i);
					again = pruner->overlap(sd, pcb);
					if(!again)
					{
						cbr.again = again; // update the status to avoid duplicate processTouches()
						return hits.hasAnyHits();
					}
				}
			}
		}
		//~#MODIFIED

		if(again && compoundPruner)
			again = compoundPruner->overlap(sd, pcb, compoundPrunerQueryFlags);

		cbr.again = again; // update the status to avoid duplicate processTouches()
		return hits.hasAnyHits();
	}
	else
	{
		PX_ASSERT(HitTypeSupport<HitType>::IsSweep);
		PX_ASSERT(input.geometry);

		const ShapeData sd(*input.geometry, *input.pose, input.inflation);
		pcb.mQueryShapeBounds = &sd.getPrunerInflatedWorldAABB();
		pcb.mShapeData = &sd;

		// #MODIFIED
		bool again = true;
		if(treeOfPruners)
		{
			LocalSweepCallback<HitType> prunerSweepCB(sd, input.getDir(), pcb, mSQManager, adapter, hits, filterData, filterCall);
			again = treeOfPruners->sweep(*input.geometry, *input.pose, input.getDir(), pcb.mShrunkDistance, prunerSweepCB, PxGeometryQueryFlag::Enum(0));
			if(!again)
			{
				cbr.again = again; // update the status to avoid duplicate processTouches()
				return hits.hasAnyHits();
			}
		}
		else
		{
			for(PxU32 i=0;i<nbPruners;i++)
			{
				if(prunerFilter(adapter, i, &hits, filterData, filterCall))
				{
					const Pruner* pruner = mSQManager.getPruner(i);
					again = pruner->sweep(sd, input.getDir(), pcb.mShrunkDistance, pcb);
					if(!again)
					{
						cbr.again = again; // update the status to avoid duplicate processTouches()
						return hits.hasAnyHits();
					}
				}
			}
		}
		//~#MODIFIED

		if(again && compoundPruner)
			again = compoundPruner->sweep(sd, input.getDir(), pcb.mShrunkDistance, pcb, compoundPrunerQueryFlags);
		
		cbr.again = again; // update the status to avoid duplicate processTouches()
		return hits.hasAnyHits();
	}
}

//explicit template instantiation
template bool ExtSceneQueries::multiQuery<PxRaycastHit>(const ExtMultiQueryInput&, PxHitCallback<PxRaycastHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const; 
template bool ExtSceneQueries::multiQuery<PxOverlapHit>(const ExtMultiQueryInput&, PxHitCallback<PxOverlapHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;
template bool ExtSceneQueries::multiQuery<PxSweepHit>(const ExtMultiQueryInput&, PxHitCallback<PxSweepHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*) const;

///////////////////////////////////////////////////////////////////////////////

bool ExtSceneQueries::_raycast(
	const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
	PxHitCallback<PxRaycastHit>& hits, PxHitFlags hitFlags, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, PxGeometryQueryFlags flags) const
{
	PX_PROFILE_ZONE("SceneQuery.raycast", getContextId());
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)

	ExtMultiQueryInput input(origin, unitDir, distance);
	return multiQuery<PxRaycastHit>(input, hits, hitFlags, cache, filterData, filterCall);
}

//////////////////////////////////////////////////////////////////////////

bool ExtSceneQueries::_overlap(
	const PxGeometry& geometry, const PxTransform& pose, PxOverlapCallback& hits,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, PxGeometryQueryFlags flags) const
{
	PX_PROFILE_ZONE("SceneQuery.overlap", getContextId());
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)

	ExtMultiQueryInput input(&geometry, &pose);
	return multiQuery<PxOverlapHit>(input, hits, PxHitFlags(), cache, filterData, filterCall);
}

///////////////////////////////////////////////////////////////////////////////

bool ExtSceneQueries::_sweep(
	const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
	PxHitCallback<PxSweepHit>& hits, PxHitFlags hitFlags, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const
{
	PX_PROFILE_ZONE("SceneQuery.sweep", getContextId());
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)

#if PX_CHECKED
	if(!PxGeometryQuery::isValid(geometry))
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Provided geometry is not valid");
#endif

	if((hitFlags & PxHitFlag::ePRECISE_SWEEP) && (hitFlags & PxHitFlag::eMTD))
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, " Precise sweep doesn't support MTD. Perform MTD with default sweep");
		hitFlags &= ~PxHitFlag::ePRECISE_SWEEP;
	}

	if((hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP) && (hitFlags & PxHitFlag::eMTD))
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, " eMTD cannot be used in conjunction with eASSUME_NO_INITIAL_OVERLAP. eASSUME_NO_INITIAL_OVERLAP will be ignored");
		hitFlags &= ~PxHitFlag::eASSUME_NO_INITIAL_OVERLAP;
	}

	PxReal realInflation = inflation;
	if((hitFlags & PxHitFlag::ePRECISE_SWEEP)&& inflation > 0.f)
	{
		realInflation = 0.f;
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, " Precise sweep doesn't support inflation, inflation will be overwritten to be zero");
	}
	ExtMultiQueryInput input(&geometry, &pose, unitDir, distance, realInflation);
	return multiQuery<PxSweepHit>(input, hits, hitFlags, cache, filterData, filterCall);
}

///////////////////////////////////////////////////////////////////////////////

template<typename HitType>
static bool doQueryVsCached(const PrunerHandle handle, PxU32 prunerIndex, const PrunerCompoundId cachedCompoundId, const ExtPrunerManager& manager, ExtMultiQueryCallback<HitType>& pcb, const ExtMultiQueryInput& input)
{
	// this block is only executed for single shape cache
	const PrunerPayload* payloads;
	const PxTransform* compoundPosePtr;

	PxTransform* transform;
	PxTransform compoundPose;
	if(cachedCompoundId == INVALID_COMPOUND_ID)
	{
		const Pruner* pruner = manager.getPruner(PruningIndex::Enum(prunerIndex));
		PX_ASSERT(pruner);

		PrunerPayloadData ppd;
		const PrunerPayload& cachedPayload = pruner->getPayloadData(handle, &ppd);

		payloads = &cachedPayload;
		compoundPosePtr = NULL;
		transform = ppd.mTransform;
	}
	else
	{
		const CompoundPruner* pruner = manager.getCompoundPruner();
		PX_ASSERT(pruner);

		PrunerPayloadData ppd;
		const PrunerPayload& cachedPayload = pruner->getPayloadData(handle, cachedCompoundId, &ppd);

		compoundPose = pruner->getTransform(cachedCompoundId);

		payloads = &cachedPayload;
		compoundPosePtr = &compoundPose;
		transform = ppd.mTransform;
	}
	PxReal dummyDist;
	
	bool againAfterCache;
	if(HitTypeSupport<HitType>::IsSweep)
	{
		// AP: for sweeps we cache the bounds because we need to know them for the test to clip the sweep to bounds
		// otherwise GJK becomes unstable. The bounds can be used multiple times so this is an optimization.
		const ShapeData sd(*input.geometry, *input.pose, input.inflation);
		pcb.mQueryShapeBounds = &sd.getPrunerInflatedWorldAABB();
		pcb.mShapeData = &sd;
//		againAfterCache = pcb.invoke(dummyDist, 0);
		againAfterCache = pcb.template _invoke<true>(dummyDist, 0, payloads, transform, compoundPosePtr);
		pcb.mQueryShapeBounds = NULL;
		pcb.mShapeData = NULL;
	} else
//		againAfterCache = pcb.invoke(dummyDist, 0);
		againAfterCache = pcb.template _invoke<true>(dummyDist, 0, payloads, transform, compoundPosePtr);

	return againAfterCache;
}

///////////////////////////////////////////////////////////////////////////////

ExtSceneQueries::ExtSceneQueries(ExtPVDCapture* pvd, PxU64 contextID, float inflation, const ExtQueryAdapter& adapter, bool usesTreeOfPruners) :
	mSQManager	(contextID, inflation, adapter, usesTreeOfPruners),
	mPVD		(pvd)
{
}

ExtSceneQueries::~ExtSceneQueries()
{
}

