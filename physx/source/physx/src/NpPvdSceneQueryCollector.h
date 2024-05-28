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

#ifndef NP_PVD_SCENEQUERYCOLLECTOR_H
#define NP_PVD_SCENEQUERYCOLLECTOR_H

#include "geometry/PxGeometryHelpers.h"
#include "PxFiltering.h"
#include "PxQueryReport.h"
#include "PxQueryFiltering.h"

#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"

#if PX_SUPPORT_PVD

namespace physx
{
namespace Vd
{
	class PvdSceneClient;

	struct QueryID { enum Enum {
		QUERY_RAYCAST_ANY_OBJECT,
		QUERY_RAYCAST_CLOSEST_OBJECT,
		QUERY_RAYCAST_ALL_OBJECTS,

		QUERY_OVERLAP_SPHERE_ALL_OBJECTS,
		QUERY_OVERLAP_AABB_ALL_OBJECTS,
		QUERY_OVERLAP_OBB_ALL_OBJECTS,
		QUERY_OVERLAP_CAPSULE_ALL_OBJECTS,
		QUERY_OVERLAP_CONVEX_ALL_OBJECTS,

		QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT,
		QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT,
		QUERY_LINEAR_CONVEX_SWEEP_CLOSEST_OBJECT
	}; };

struct PvdReference
{
	PX_FORCE_INLINE PvdReference()																													{}
	PX_FORCE_INLINE PvdReference(const char* arrayName, PxU32 baseIndex, PxU32 count) : mArrayName(arrayName), mBaseIndex(baseIndex), mCount(count)	{}

	const char*		mArrayName;
	PxU32			mBaseIndex;
	PxU32			mCount;
};

struct PvdRaycast
{
	PxU32			mType;
	PxFilterData	mFilterData;
	PxU32			mFilterFlags;
	PxVec3			mOrigin;
	PxVec3			mUnitDir;
	PxReal			mDistance;
	PvdReference	mHits;
};

struct PvdOverlap
{
	PxU32			mType;
	PxFilterData	mFilterData;
	PxU32			mFilterFlags;
	PxTransform		mPose;
	PvdReference	mGeometries;
	PvdReference	mHits;
};

struct PvdSweep
{
	PxU32			mType;
	PxU32			mFilterFlags;
	PxVec3			mUnitDir;
	PxReal			mDistance;
	PvdReference	mGeometries;
	PvdReference	mPoses;
	PvdReference	mFilterData;
	PvdReference	mHits;
};

struct PvdSqHit
{
	const void*		mShape;
	const void*		mActor;
	PxU32			mFaceIndex;
	PxU32			mFlags;
	PxVec3			mImpact;
	PxVec3			mNormal;
	PxF32			mDistance;
	PxF32			mU;
	PxF32			mV;

	PvdSqHit()
	{
		setDefaults(0xFFFFffff, NULL, NULL);
	}

	explicit PvdSqHit(const PxOverlapHit& hit)
	{
		setDefaults(hit.faceIndex, hit.shape, hit.actor);
	}

	explicit PvdSqHit(const PxRaycastHit& hit)
	{
		setDefaults(hit.faceIndex, hit.shape, hit.actor);

		mImpact = hit.position;
		mNormal = hit.normal;
		mDistance = hit.distance;
		mFlags = hit.flags;
		mU = hit.u;
		mV = hit.v;
	}

	explicit PvdSqHit(const PxSweepHit& hit)
	{
		setDefaults(hit.faceIndex, hit.shape, hit.actor);

		mImpact = hit.position;
		mNormal = hit.normal;
		mDistance = hit.distance;
		mFlags = hit.flags;
	}

  private:
	void setDefaults(PxU32 faceIndex, const void* shape, const void* actor)
	{
		mShape = shape;
		mActor = actor;
		mFaceIndex = faceIndex;
		mFlags = 0;
		mImpact = mNormal = PxVec3(0.0f);
		mDistance = mU = mV = 0.0f;
	}
};

template <class T>
class NamedArray : public PxArray<T>
{
	public:
		NamedArray(const char* names[2]) { mNames[0] = names[0]; mNames[1] = names[1]; }

	const char*	mNames[2];
};

class PvdSceneQueryCollector
{
	PX_NOCOPY(PvdSceneQueryCollector)
public:
	PvdSceneQueryCollector(Vd::PvdSceneClient* pvd, bool isBatched);
	~PvdSceneQueryCollector()	{}

	void clear()
	{
		PxMutex::ScopedLock lock(mMutex);

		mAccumulatedRaycastQueries.clear();
		mAccumulatedOverlapQueries.clear();
		mAccumulatedSweepQueries.clear();
		mPvdSqHits.clear();
		mPoses.clear();
		mFilterData.clear();
	}

	void clearGeometryArrays()
	{
		mGeometries0.clear();
		mGeometries1.clear();
	}

	void release();

	void raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits);
	void sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits);
	void overlapMultiple(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData);

	PX_FORCE_INLINE	PxMutex&							getLock()												{ return mMutex;								}

	template <class T>
	PX_FORCE_INLINE	const char*							getArrayName(const NamedArray<T>& namedArray)	const	{ return namedArray.mNames[mIsBatched];			}

	PX_FORCE_INLINE	const NamedArray<PxGeometryHolder>&	getGeometries(PxU32 index)						const	{ return index ? mGeometries1 : mGeometries0;	}
	PX_FORCE_INLINE	NamedArray<PxGeometryHolder>&		getGeometries(PxU32 index)								{ return index ? mGeometries1 : mGeometries0;	}

	PX_FORCE_INLINE	const NamedArray<PxGeometryHolder>& getCurrentFrameGeometries()						const	{ return getGeometries(mInUse);					}
	PX_FORCE_INLINE	const NamedArray<PxGeometryHolder>& getPrevFrameGeometries()						const	{ return getGeometries(mInUse ^ 1);				}

	void prepareNextFrameGeometries()
	{
		mInUse ^= 1;
		getGeometries(mInUse).clear();
	}

	NamedArray<PvdRaycast>		mAccumulatedRaycastQueries;
	NamedArray<PvdSweep>		mAccumulatedSweepQueries;
	NamedArray<PvdOverlap>		mAccumulatedOverlapQueries;
	NamedArray<PvdSqHit>		mPvdSqHits;
	NamedArray<PxTransform>		mPoses;
	NamedArray<PxFilterData>	mFilterData;

private:
	Vd::PvdSceneClient*			mPVD;
	PxMutex						mMutex;
	NamedArray<PxGeometryHolder>mGeometries0;
	NamedArray<PxGeometryHolder>mGeometries1;
	PxU32						mInUse;
	const bool					mIsBatched;
};
}
}

#endif // PX_SUPPORT_PVD

#endif // NP_PVD_SCENEQUERYCOLLECTOR_H
