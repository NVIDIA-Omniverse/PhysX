// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpPvdSceneQueryCollector.h"
#include "NpPvdSceneClient.h"

#if PX_SUPPORT_PVD
using namespace physx;
using namespace Vd;

static const char* gName_PvdRaycast[2]			= { "SceneQueries.Raycasts",		"BatchedQueries.Raycasts" };
static const char* gName_PvdSweep[2]			= { "SceneQueries.Sweeps",			"BatchedQueries.Sweeps" };
static const char* gName_PvdOverlap[2]			= { "SceneQueries.Overlaps",		"BatchedQueries.Overlaps" };
static const char* gName_PvdSqHit[2]			= { "SceneQueries.Hits",			"BatchedQueries.Hits" };
static const char* gName_PxTransform[2]			= { "SceneQueries.PoseList",		"BatchedQueries.PoseList" };
static const char* gName_PxFilterData[2]		= { "SceneQueries.FilterDataList",	"BatchedQueries.FilterDataList" };
static const char* gName_PxGeometryHolder[2]	= { "SceneQueries.GeometryList",	"BatchedQueries.GeometryList" };

PvdSceneQueryCollector::PvdSceneQueryCollector(Vd::PvdSceneClient* pvd, bool isBatched) :
	mAccumulatedRaycastQueries	(gName_PvdRaycast),
	mAccumulatedSweepQueries	(gName_PvdSweep),
	mAccumulatedOverlapQueries	(gName_PvdOverlap),
	mPvdSqHits					(gName_PvdSqHit),
	mPoses						(gName_PxTransform),
	mFilterData					(gName_PxFilterData),
	mPVD						(pvd),
	mGeometries0				(gName_PxGeometryHolder),
	mGeometries1				(gName_PxGeometryHolder),
	mInUse						(0),
	mIsBatched					(isBatched)
{
}

void PvdSceneQueryCollector::release()
{
	physx::pvdsdk::PvdDataStream* stream = mPVD->getDataStream();
	if(stream && stream->isConnected())
	{
		const PxArray<PxGeometryHolder>& geoms = getPrevFrameGeometries();
		for(PxU32 k=0; k<geoms.size(); ++k)
			stream->destroyInstance(&geoms[k]);

		clearGeometryArrays();
	}
}

template<class SDKHitType, class PvdHitType>
static void accumulate(PvdHitType& query, PxArray<PvdHitType>& accumulated, const char* arrayName, PxArray<PvdSqHit>& dst, const SDKHitType* src, PxU32 nb, const PxQueryFilterData& fd)
{
	query.mFilterFlags = fd.flags;
	query.mHits = PvdReference(arrayName, dst.size(), nb);

	PX_ASSERT(PxU32(-1) != nb);
	for(PxU32 i=0; i<nb; i++)
		dst.pushBack(PvdSqHit(src[i]));

	accumulated.pushBack(query);
}

static PX_FORCE_INLINE void clampNbHits(PxU32& hitsNum, const PxQueryFilterData& fd, bool multipleHits)
{
	if((fd.flags & PxQueryFlag::eANY_HIT) || !multipleHits)
		hitsNum = hitsNum > 0 ? 1u : 0;
}

template<class Type> static void pushBackT(PxArray<Type>& array, const Type& item, PvdReference& ref, const char* arrayName)
{
	ref = PvdReference(arrayName, array.size(), 1);
	array.pushBack(item);
}

void PvdSceneQueryCollector::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& fd, bool multipleHits)
{
	PxMutex::ScopedLock lock(mMutex);

	PvdRaycast raycastQuery;
	raycastQuery.mOrigin		= origin;
	raycastQuery.mUnitDir		= unitDir;
	raycastQuery.mDistance		= distance;
	raycastQuery.mFilterData	= fd.data;
	if(fd.flags & PxQueryFlag::eANY_HIT)	raycastQuery.mType = QueryID::QUERY_RAYCAST_ANY_OBJECT;
	else if(multipleHits)					raycastQuery.mType = QueryID::QUERY_RAYCAST_ALL_OBJECTS;
	else									raycastQuery.mType = QueryID::QUERY_RAYCAST_CLOSEST_OBJECT;
	clampNbHits(hitsNum, fd, multipleHits);

	accumulate(raycastQuery, mAccumulatedRaycastQueries, getArrayName(mPvdSqHits), mPvdSqHits, hit, hitsNum, fd);
}

void PvdSceneQueryCollector::sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& fd, bool multipleHits)
{
	PxMutex::ScopedLock lock(mMutex);

	PvdSweep sweepQuery;
	pushBackT(getGeometries(mInUse), PxGeometryHolder(geometry), sweepQuery.mGeometries, getArrayName(getGeometries(mInUse)));	// PT: TODO: optimize this. We memcopy once to the stack, then again to the array....
	pushBackT(mPoses, pose, sweepQuery.mPoses, getArrayName(mPoses));
	pushBackT(mFilterData, fd.data, sweepQuery.mFilterData, getArrayName(mFilterData));

	const PxGeometryType::Enum type = geometry.getType();	// PT: TODO: QueryID::QUERY_LINEAR_xxx_SWEEP_ALL_OBJECTS are never used!
	if(type==PxGeometryType::eBOX)												sweepQuery.mType = QueryID::QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT;
	else if(type==PxGeometryType::eSPHERE || type==PxGeometryType::eCAPSULE)	sweepQuery.mType = QueryID::QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT;
	else if(type==PxGeometryType::eCONVEXMESH)									sweepQuery.mType = QueryID::QUERY_LINEAR_CONVEX_SWEEP_CLOSEST_OBJECT;
	else																		PX_ASSERT(0);
	sweepQuery.mUnitDir		= unitDir;
	sweepQuery.mDistance	= distance;
	clampNbHits(hitsNum, fd, multipleHits);

	accumulate(sweepQuery, mAccumulatedSweepQueries, getArrayName(mPvdSqHits), mPvdSqHits, hit, hitsNum, fd);
}

void PvdSceneQueryCollector::overlapMultiple(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& fd)
{
	PxMutex::ScopedLock lock(mMutex);

	PvdOverlap overlapQuery;
	pushBackT(getGeometries(mInUse), PxGeometryHolder(geometry), overlapQuery.mGeometries, getArrayName(getGeometries(mInUse)));	// PT: TODO: optimize this. We memcopy once to the stack, then again to the array....

	const PxGeometryType::Enum type = geometry.getType();
	if(type==PxGeometryType::eBOX)				overlapQuery.mType = pose.q.isIdentity() ? QueryID::QUERY_OVERLAP_AABB_ALL_OBJECTS : QueryID::QUERY_OVERLAP_OBB_ALL_OBJECTS;
	else if(type==PxGeometryType::eSPHERE)		overlapQuery.mType = QueryID::QUERY_OVERLAP_SPHERE_ALL_OBJECTS;
	else if(type==PxGeometryType::eCAPSULE)		overlapQuery.mType = QueryID::QUERY_OVERLAP_CAPSULE_ALL_OBJECTS;
	else if(type==PxGeometryType::eCONVEXMESH)	overlapQuery.mType = QueryID::QUERY_OVERLAP_CONVEX_ALL_OBJECTS;
	else										PX_ASSERT(0);
	overlapQuery.mPose			= pose;
	overlapQuery.mFilterData	= fd.data;

	accumulate(overlapQuery, mAccumulatedOverlapQueries, getArrayName(mPvdSqHits), mPvdSqHits, hit, hitsNum, fd);
}
#endif
