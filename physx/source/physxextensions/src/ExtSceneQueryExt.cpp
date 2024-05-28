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

#include "extensions/PxSceneQueryExt.h"
#include "geometry/PxGeometryHelpers.h"
#include "foundation/PxAllocatorCallback.h"
#include "CmUtils.h"
#include "foundation/PxAllocator.h"

using namespace physx;

bool PxSceneQueryExt::raycastAny(	const PxScene& scene,
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryHit& hit, const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= PxQueryFlag::eANY_HIT;
	PxRaycastBuffer buf;
	scene.raycast(origin, unitDir, distance, buf, PxHitFlag::eANY_HIT, fdAny, filterCall, cache);
	hit = buf.block;
	return buf.hasBlock;
}

bool PxSceneQueryExt::raycastSingle(const PxScene& scene,
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryFlags outputFlags, PxRaycastHit& hit,
									const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache)
{
	PxRaycastBuffer buf;
	PxQueryFilterData fd1 = filterData;
	scene.raycast(origin, unitDir, distance, buf, outputFlags, fd1, filterCall, cache);
	hit = buf.block;
	return buf.hasBlock;
}

PxI32 PxSceneQueryExt::raycastMultiple(	const PxScene& scene,
										const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
										PxSceneQueryFlags outputFlags,
										PxRaycastHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
										const PxSceneQueryFilterData& filterData,
										PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache)
{
	PxRaycastBuffer buf(hitBuffer, hitBufferSize);
	PxQueryFilterData fd1 = filterData;
	scene.raycast(origin, unitDir, distance, buf, outputFlags, fd1, filterCall, cache);
	blockingHit = buf.hasBlock;
	if(blockingHit)
	{
		if(buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return PxI32(buf.nbTouches+1);
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return PxI32(buf.nbTouches);
}

bool PxSceneQueryExt::sweepAny(	const PxScene& scene,
								const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
								PxSceneQueryFlags queryFlags,
								PxSceneQueryHit& hit,
								const PxSceneQueryFilterData& filterData,
								PxSceneQueryFilterCallback* filterCall,
								const PxSceneQueryCache* cache,
								PxReal inflation)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= PxQueryFlag::eANY_HIT;
	PxSweepBuffer buf;
	scene.sweep(geometry, pose, unitDir, distance, buf, queryFlags|PxHitFlag::eANY_HIT, fdAny, filterCall, cache, inflation);
	hit = buf.block;
	return buf.hasBlock;
}

bool PxSceneQueryExt::sweepSingle(	const PxScene& scene,
									const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryFlags outputFlags,
									PxSweepHit& hit,
									const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall,
									const PxSceneQueryCache* cache,
									PxReal inflation)
{
	PxSweepBuffer buf;
	PxQueryFilterData fd1 = filterData;
	scene.sweep(geometry, pose, unitDir, distance, buf, outputFlags, fd1, filterCall, cache, inflation);
	hit = buf.block;
	return buf.hasBlock;
}

PxI32 PxSceneQueryExt::sweepMultiple(	const PxScene& scene,
										const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
										PxSceneQueryFlags outputFlags, PxSweepHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
										const PxSceneQueryFilterData& filterData,
										PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache,
										PxReal inflation)
{
	PxQueryFilterData fd1 = filterData;
	PxSweepBuffer buf(hitBuffer, hitBufferSize);
	scene.sweep(geometry, pose, unitDir, distance, buf, outputFlags, fd1, filterCall, cache, inflation);
	blockingHit = buf.hasBlock;
	if(blockingHit)
	{
		if(buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return PxI32(buf.nbTouches+1);
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return PxI32(buf.nbTouches);
}

PxI32 PxSceneQueryExt::overlapMultiple(	const PxScene& scene,
										const PxGeometry& geometry, const PxTransform& pose,
										PxOverlapHit* hitBuffer, PxU32 hitBufferSize,
										const PxSceneQueryFilterData& filterData,
										PxSceneQueryFilterCallback* filterCall)
{
	PxQueryFilterData fd1 = filterData;
	fd1.flags |= PxQueryFlag::eNO_BLOCK;
	PxOverlapBuffer buf(hitBuffer, hitBufferSize);
	scene.overlap(geometry, pose, buf, fd1, filterCall);
	if(buf.hasBlock)
	{
		if(buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return PxI32(buf.nbTouches+1);
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return PxI32(buf.nbTouches);
}

bool PxSceneQueryExt::overlapAny(	const PxScene& scene,
									const PxGeometry& geometry, const PxTransform& pose,
									PxOverlapHit& hit,
									const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= (PxQueryFlag::eANY_HIT | PxQueryFlag::eNO_BLOCK);
	PxOverlapBuffer buf;
	scene.overlap(geometry, pose, buf, fdAny, filterCall);
	hit = buf.block;
	return buf.hasBlock;
}

namespace
{
	struct Raycast
	{
		PxVec3 origin;
		PxVec3 unitDir;
		PxReal distance;
		PxHitFlags hitFlags;
		PxQueryFilterData filterData;
		const PxQueryCache* cache;
	};
	struct Sweep
	{
		PxGeometryHolder geometry;
		PxTransform pose;
		PxVec3 unitDir;
		PxReal distance;
		PxHitFlags hitFlags;
		PxQueryFilterData filterData;
		const PxQueryCache* cache;
		PxReal inflation;
	};
	struct Overlap
	{
		PxGeometryHolder geometry;
		PxTransform pose;
		PxQueryFilterData filterData;
		const PxQueryCache* cache;
	};
}

template<typename HitType>
struct NpOverflowBuffer : PxHitBuffer<HitType>
{
	bool overflow;
	bool processCalled;
	PxU32 saveNbTouches;
	NpOverflowBuffer(HitType* hits, PxU32 count) : PxHitBuffer<HitType>(hits, count), overflow(false), processCalled(false), saveNbTouches(0)
	{
	}

	virtual PxAgain processTouches(const HitType* /*hits*/, PxU32 /*count*/)
	{
		if (processCalled)
			return false;
		saveNbTouches = this->nbTouches;
		processCalled = true;
		return true;
	}

	virtual void finalizeQuery()
	{
		if (processCalled)
		{
			overflow = (this->nbTouches > 0);
			this->nbTouches = saveNbTouches;
		}
	}
};


class ExtBatchQuery : public PxBatchQueryExt
{
	PX_NOCOPY(ExtBatchQuery)
public:

	ExtBatchQuery(
		const PxScene& scene,
		PxQueryFilterCallback* queryFilterCallback,
		PxRaycastBuffer* raycastBuffers, Raycast* raycastQueries, const PxU32 maxNbRaycasts, PxRaycastHit* raycastTouches, const PxU32 maxNbRaycastTouches,
		PxSweepBuffer* sweepBuffers, Sweep* sweepQueries, const PxU32 maxNbSweeps, PxSweepHit* sweepTouches, const PxU32 maxNbSweepTouches,
		PxOverlapBuffer* overlapBuffers, Overlap* overlapQueries, const PxU32 maxNbOverlaps, PxOverlapHit* overlapTouches, const PxU32 maxNbOverlapTouches);

	~ExtBatchQuery() {}

	virtual void release();

	virtual PxRaycastBuffer* raycast(
		const PxVec3& origin, const PxVec3& unitDir, const PxReal distance, const PxU16 maxNbTouches,
		PxHitFlags hitFlags = PxHitFlags(PxHitFlag::eDEFAULT),
		const PxQueryFilterData& filterData = PxQueryFilterData(),
		const PxQueryCache* cache = NULL);

	virtual PxSweepBuffer* sweep(
		const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance, const PxU16 maxNbTouches,
		PxHitFlags hitFlags = PxHitFlags(PxHitFlag::eDEFAULT),
		const PxQueryFilterData& filterData = PxQueryFilterData(),
		const PxQueryCache* cache = NULL,
		const PxReal inflation = 0.f);

	virtual PxOverlapBuffer* overlap(
		const PxGeometry& geometry, const PxTransform& pose, PxU16 maxNbTouches = 0,
		const PxQueryFilterData& filterData = PxQueryFilterData(),
		const PxQueryCache* cache = NULL);

	virtual void execute();

private:

	template<typename HitType, typename QueryType> struct Query
	{
		PxHitBuffer<HitType>* mBuffers;
		QueryType* mQueries;
		PxU32 mMaxNbBuffers;
		HitType* mTouches;
		PxU32 mMaxNbTouches;

		PxU32 mBufferTide;

		Query()
			: mBuffers(NULL),
			mQueries(NULL),
			mMaxNbBuffers(0),
			mTouches(NULL),
			mMaxNbTouches(0),
			mBufferTide(0)
		{
		}

		Query(PxHitBuffer<HitType>* buffers, QueryType* queries, const PxU32 maxNbBuffers, HitType* touches, const PxU32 maxNbTouches)
			: mBuffers(buffers),
			mQueries(queries),
			mMaxNbBuffers(maxNbBuffers),
			mTouches(touches),
			mMaxNbTouches(maxNbTouches),
			mBufferTide(0)
		{
			for (PxU32 i = 0; i < mMaxNbBuffers; i++)
			{
				mBuffers[i].hasBlock = false;
				mBuffers[i].nbTouches = 0;
			}
		}

		PxHitBuffer<HitType>* addQuery(const QueryType& query, const PxU32 maxNbTouches)
		{
			if ((mBufferTide + 1) > mMaxNbBuffers)
			{
				//Ran out of queries.
				return NULL;
			}

			PxHitBuffer<HitType>* buffer = mBuffers + mBufferTide;
			buffer->touches = NULL;
			buffer->maxNbTouches = maxNbTouches;
			buffer->hasBlock = false;
			buffer->nbTouches = 0xffffffff;

			mQueries[mBufferTide] = query;

			mBufferTide++;

			return buffer;
		}

		static void performQuery(const PxScene& scene, const Raycast& query, NpOverflowBuffer<PxRaycastHit>& hitBuffer, PxQueryFilterCallback* qfcb)
		{
			scene.raycast(
				query.origin, query.unitDir, query.distance,
				hitBuffer,
				query.hitFlags,
				query.filterData, qfcb,
				query.cache);
		}

		static void performQuery(const PxScene& scene, const Sweep& query, NpOverflowBuffer<PxSweepHit>& hitBuffer, PxQueryFilterCallback* qfcb)
		{
			scene.sweep(
				query.geometry.any(), query.pose, query.unitDir, query.distance,
				hitBuffer,
				query.hitFlags,
				query.filterData, qfcb,
				query.cache,
				query.inflation);
		}

		static void performQuery(const PxScene& scene, const Overlap& query, NpOverflowBuffer<PxOverlapHit>& hitBuffer, PxQueryFilterCallback* qfcb)
		{
			scene.overlap(
				query.geometry.any(), query.pose,
				hitBuffer,
				query.filterData, qfcb,
				query.cache);
		}

		void execute(const PxScene& scene, PxQueryFilterCallback* qfcb)
		{
			PxU32 touchesTide = 0;
			for (PxU32 i = 0; i < mBufferTide; i++)
			{
				PX_ASSERT(0xffffffff == mBuffers[i].nbTouches);
				PX_ASSERT(0xffffffff != mBuffers[i].maxNbTouches);
				PX_ASSERT(!mBuffers[i].touches);

				bool noTouchesRemaining = false;
				if (mBuffers[i].maxNbTouches > 0)
				{
					if (touchesTide >= mMaxNbTouches)
					{
						//No resources left.
						mBuffers[i].maxNbTouches = 0;
						mBuffers[i].touches = NULL;
						noTouchesRemaining = true;
					}
					else if ((touchesTide + mBuffers[i].maxNbTouches) > mMaxNbTouches)
					{
						//Some resources left but not enough to match requested number.
						//This might be enough but it depends on the number of hits generated by the query.
						mBuffers[i].maxNbTouches = mMaxNbTouches - touchesTide;
						mBuffers[i].touches = mTouches + touchesTide;
					}
					else
					{
						//Enough resources left to match request.
						mBuffers[i].touches = mTouches + touchesTide;
					}
				}

				bool overflow = false;
				{
					PX_ALIGN(16, NpOverflowBuffer<HitType> overflowBuffer)(mBuffers[i].touches, mBuffers[i].maxNbTouches);
					performQuery(scene, mQueries[i], overflowBuffer, qfcb);
					overflow = overflowBuffer.overflow || noTouchesRemaining;
					mBuffers[i].hasBlock = overflowBuffer.hasBlock;
					mBuffers[i].block = overflowBuffer.block;
					mBuffers[i].nbTouches = overflowBuffer.nbTouches;
				}

				if(overflow)
				{
					mBuffers[i].maxNbTouches = 0xffffffff;
				}
				touchesTide += mBuffers[i].nbTouches;
			}

			mBufferTide = 0;
		}
	};

	const PxScene& mScene;
	PxQueryFilterCallback* mQueryFilterCallback;

	Query<PxRaycastHit, Raycast> mRaycasts;
	Query<PxSweepHit, Sweep> mSweeps;
	Query<PxOverlapHit, Overlap> mOverlaps;
};

template<typename HitType>
class ExtBatchQueryDesc
{
public:
	ExtBatchQueryDesc(const PxU32 maxNbResults, const PxU32 maxNbTouches)
		: mResults(NULL),
		  mMaxNbResults(maxNbResults),
		  mTouches(NULL),
		  mMaxNbTouches(maxNbTouches)
	{
	}
	ExtBatchQueryDesc(PxHitBuffer<HitType>* results, const PxU32 maxNbResults, HitType* touches, PxU32 maxNbTouches)
		: mResults(results),
		  mMaxNbResults(maxNbResults),
		  mTouches(touches),
		  mMaxNbTouches(maxNbTouches)
	{
	}

	PX_FORCE_INLINE PxHitBuffer<HitType>* getResults() const { return mResults; }
	PX_FORCE_INLINE PxU32 getNbResults() const { return mMaxNbResults; }
	PX_FORCE_INLINE HitType* getTouches() const { return mTouches; }
	PX_FORCE_INLINE PxU32 getNbTouches() const { return mMaxNbTouches; }

private:
	PxHitBuffer<HitType>* mResults;
	PxU32 mMaxNbResults;
	HitType* mTouches;
	PxU32 mMaxNbTouches;
};

template <typename HitType, typename QueryType>
PxU32 computeByteSize(const ExtBatchQueryDesc<HitType>& queryDesc)
{
	PxU32 byteSize = 0;
	if (queryDesc.getNbResults() > 0)
	{
		byteSize += sizeof(QueryType)*queryDesc.getNbResults();
		if (!queryDesc.getResults())
		{
			byteSize += sizeof(PxHitBuffer<HitType>)*queryDesc.getNbResults() + sizeof(HitType)*queryDesc.getNbTouches();
		}
	}
	return byteSize;
}

template <typename HitType, typename QueryType> PxU8* parseDesc
(PxU8* bufIn, const ExtBatchQueryDesc<HitType>& queryDesc,
	PxHitBuffer<HitType>*& results,
	QueryType*& queries,
	PxU32& maxBufferSize,
	HitType*& touches,
	PxU32& maxNbTouches)
{
	PxU8* bufOut = bufIn;

	results = queryDesc.getResults();
	queries = NULL;
	maxBufferSize = queryDesc.getNbResults();
	touches = queryDesc.getTouches();
	maxNbTouches = queryDesc.getNbTouches();

	if (maxBufferSize > 0)
	{
		queries = reinterpret_cast<QueryType*>(bufOut);
		bufOut += sizeof(QueryType)*maxBufferSize;

		if (!results)
		{
			results = reinterpret_cast<PxHitBuffer<HitType>*>(bufOut);
			for (PxU32 i = 0; i < maxBufferSize; i++)
			{
				PX_PLACEMENT_NEW(results + i, PxHitBuffer<HitType>);
			}
			bufOut += sizeof(PxHitBuffer<HitType>)*maxBufferSize;

			if (maxNbTouches > 0)
			{
				touches = reinterpret_cast<HitType*>(bufOut);
				bufOut += sizeof(HitType)*maxNbTouches;
			}
		}
	}

	return bufOut;
}

PxBatchQueryExt* create
(const PxScene& scene, PxQueryFilterCallback* queryFilterCallback,
 const ExtBatchQueryDesc<PxRaycastHit>& raycastDesc, const ExtBatchQueryDesc<PxSweepHit>& sweepDesc, const ExtBatchQueryDesc<PxOverlapHit>& overlapDesc)
{
	const PxU32 byteSize =
		sizeof(ExtBatchQuery) +
		computeByteSize<PxRaycastHit, Raycast>(raycastDesc) +
		computeByteSize<PxSweepHit, Sweep>(sweepDesc) +
		computeByteSize<PxOverlapHit, Overlap>(overlapDesc);

	PxAllocatorCallback& allocator = *PxGetAllocatorCallback();

	PxU8* buf = reinterpret_cast<PxU8*>(allocator.allocate(byteSize, "NpBatchQueryExt", PX_FL));
	PX_CHECK_AND_RETURN_NULL(buf, "PxCreateBatchQueryExt - alllocation failed");
	ExtBatchQuery* bq = reinterpret_cast<ExtBatchQuery*>(buf);
	buf += sizeof(ExtBatchQuery);

	PxHitBuffer<PxRaycastHit>* raycastBuffers = NULL;
	Raycast* raycastQueries = NULL;
	PxU32 maxNbRaycasts = 0;
	PxRaycastHit* raycastTouches = NULL;
	PxU32 maxNbRaycastTouches = 0;
	buf = parseDesc<PxRaycastHit, Raycast>(buf, raycastDesc, raycastBuffers, raycastQueries, maxNbRaycasts, raycastTouches, maxNbRaycastTouches);

	PxHitBuffer<PxSweepHit>* sweepBuffers = NULL;
	Sweep* sweepQueries = NULL;
	PxU32 maxNbSweeps = 0;
	PxSweepHit* sweepTouches = NULL;
	PxU32 maxNbSweepTouches = 0;
	buf = parseDesc<PxSweepHit, Sweep>(buf, sweepDesc, sweepBuffers, sweepQueries, maxNbSweeps, sweepTouches, maxNbSweepTouches);

	PxHitBuffer<PxOverlapHit>* overlapBuffers = NULL;
	Overlap* overlapQueries = NULL;
	PxU32 maxNbOverlaps = 0;
	PxOverlapHit* overlapTouches = NULL;
	PxU32 maxNbOverlapTouches = 0;
	buf = parseDesc<PxOverlapHit, Overlap>(buf, overlapDesc, overlapBuffers, overlapQueries, maxNbOverlaps, overlapTouches, maxNbOverlapTouches);

	PX_ASSERT((reinterpret_cast<PxU8*>(bq) + byteSize) == buf);

	PX_PLACEMENT_NEW(bq, ExtBatchQuery)(
		scene, queryFilterCallback,
		raycastBuffers, raycastQueries, maxNbRaycasts, raycastTouches, maxNbRaycastTouches,
		sweepBuffers, sweepQueries, maxNbSweeps, sweepTouches, maxNbSweepTouches,
		overlapBuffers, overlapQueries, maxNbOverlaps, overlapTouches, maxNbOverlapTouches);

	return bq;
}

PxBatchQueryExt* physx::PxCreateBatchQueryExt(
	const PxScene& scene, PxQueryFilterCallback* queryFilterCallback,
	const PxU32 maxNbRaycasts, const PxU32 maxNbRaycastTouches,
	const PxU32 maxNbSweeps, const PxU32 maxNbSweepTouches,
	const PxU32 maxNbOverlaps, const PxU32 maxNbOverlapTouches)
{
	PX_CHECK_AND_RETURN_NULL(!((0 != maxNbRaycastTouches) && (0 == maxNbRaycasts)),
		"PxCreateBatchQueryExt - maxNbRaycastTouches is non-zero but maxNbRaycasts is zero");
	PX_CHECK_AND_RETURN_NULL(!((0 != maxNbSweepTouches) && (0 == maxNbSweeps)),
		"PxCreateBatchQueryExt - maxNbSweepTouches is non-zero but maxNbSweeps is zero");
	PX_CHECK_AND_RETURN_NULL(!((0 != maxNbOverlapTouches) && (0 == maxNbOverlaps)),
		"PxCreateBatchQueryExt - maxNbOverlaps is non-zero but maxNbOverlaps is zero");

	return create(scene, queryFilterCallback,
		ExtBatchQueryDesc<PxRaycastHit>(maxNbRaycasts, maxNbRaycastTouches),
		ExtBatchQueryDesc<PxSweepHit>(maxNbSweeps, maxNbSweepTouches),
		ExtBatchQueryDesc<PxOverlapHit>(maxNbOverlaps, maxNbOverlapTouches));
}

PxBatchQueryExt* physx::PxCreateBatchQueryExt(
	const PxScene& scene, PxQueryFilterCallback* queryFilterCallback,
	PxRaycastBuffer* raycastBuffers, const PxU32 maxNbRaycasts, PxRaycastHit* raycastTouches, const PxU32 maxNbRaycastTouches,
	PxSweepBuffer* sweepBuffers, const PxU32 maxNbSweeps, PxSweepHit* sweepTouches, const PxU32 maxNbSweepTouches,
	PxOverlapBuffer* overlapBuffers, const PxU32 maxNbOverlaps, PxOverlapHit* overlapTouches, const PxU32 maxNbOverlapTouches)
{
	PX_CHECK_AND_RETURN_NULL(!(!raycastTouches && (maxNbRaycastTouches != 0)),
		"PxCreateBatchQueryExt - maxNbRaycastTouches > 0 but raycastTouches is NULL");
	PX_CHECK_AND_RETURN_NULL(!(!raycastBuffers && (maxNbRaycasts != 0)),
		"PxCreateBatchQueryExt - maxNbRaycasts > 0 but raycastBuffers is NULL");
	PX_CHECK_AND_RETURN_NULL(!(!raycastBuffers && raycastTouches),
		"PxCreateBatchQueryExt - raycastBuffers is NULL but raycastTouches is non-NULL");

	PX_CHECK_AND_RETURN_NULL(!(!sweepTouches && (maxNbSweepTouches != 0)),
		"PxCreateBatchQueryExt - maxNbSweepTouches > 0 but sweepTouches is NULL");
	PX_CHECK_AND_RETURN_NULL(!(!sweepBuffers && (maxNbSweeps != 0)),
		"PxCreateBatchQueryExt - maxNbSweeps > 0 but sweepBuffers is NULL");
	PX_CHECK_AND_RETURN_NULL(!(!sweepBuffers && sweepTouches),
		"PxCreateBatchQueryExt - sweepBuffers is NULL but sweepTouches is non-NULL");

	PX_CHECK_AND_RETURN_NULL(!(!overlapTouches && (maxNbOverlapTouches != 0)),
		"PxCreateBatchQueryExt - maxNbOverlapTouches > 0 but overlapTouches is NULL");
	PX_CHECK_AND_RETURN_NULL(!(!overlapBuffers && (maxNbOverlaps != 0)),
		"PxCreateBatchQueryExt - maxNbOverlaps > 0 but overlapBuffers is NULL");
	PX_CHECK_AND_RETURN_NULL(!(!overlapBuffers && overlapTouches),
		"PxCreateBatchQueryExt - overlapBuffers is NULL but overlapTouches is non-NULL");

	return create(scene, queryFilterCallback, 
		ExtBatchQueryDesc<PxRaycastHit>(raycastBuffers, maxNbRaycasts, raycastTouches, maxNbRaycastTouches),
		ExtBatchQueryDesc<PxSweepHit>(sweepBuffers, maxNbSweeps, sweepTouches, maxNbSweepTouches),
		ExtBatchQueryDesc<PxOverlapHit>(overlapBuffers, maxNbOverlaps, overlapTouches, maxNbOverlapTouches));
}
 

ExtBatchQuery::ExtBatchQuery
(const PxScene& scene, PxQueryFilterCallback* queryFilterCallback,
 PxRaycastBuffer* raycastBuffers, Raycast* raycastQueries, const PxU32 maxNbRaycasts, PxRaycastHit* raycastTouches, const PxU32 maxNbRaycastTouches,
 PxSweepBuffer* sweepBuffers, Sweep* sweepQueries, const PxU32 maxNbSweeps, PxSweepHit* sweepTouches, const PxU32 maxNbSweepTouches,
 PxOverlapBuffer* overlapBuffers, Overlap* overlapQueries, const PxU32 maxNbOverlaps, PxOverlapHit* overlapTouches, const PxU32 maxNbOverlapTouches)
	: mScene(scene),
	  mQueryFilterCallback(queryFilterCallback)
{
	typedef Query<PxRaycastHit, Raycast> QueryRaycast;
	typedef Query<PxSweepHit, Sweep> QuerySweep;
	typedef Query<PxOverlapHit, Overlap> QueryOverlap;
	PX_PLACEMENT_NEW(&mRaycasts, QueryRaycast)(raycastBuffers, raycastQueries, maxNbRaycasts, raycastTouches, maxNbRaycastTouches);
	PX_PLACEMENT_NEW(&mSweeps, QuerySweep)(sweepBuffers, sweepQueries, maxNbSweeps, sweepTouches, maxNbSweepTouches);
	PX_PLACEMENT_NEW(&mOverlaps, QueryOverlap)(overlapBuffers, overlapQueries, maxNbOverlaps, overlapTouches, maxNbOverlapTouches);
}

void ExtBatchQuery::release()
{
	PxGetAllocatorCallback()->deallocate(this);
}

PxRaycastBuffer* ExtBatchQuery::raycast
(const PxVec3& origin, const PxVec3& unitDir, const PxReal distance, 
 const PxU16 maxNbTouches,
 PxHitFlags hitFlags,
 const PxQueryFilterData& filterData,
 const PxQueryCache* cache)
{
	const PxQueryFilterData qfd(filterData.data, filterData.flags | PxQueryFlag::eBATCH_QUERY_LEGACY_BEHAVIOUR);
	const Raycast raycast = { origin, unitDir, distance, hitFlags, qfd, cache };
	PxRaycastBuffer* buffer = mRaycasts.addQuery(raycast, maxNbTouches);
	PX_CHECK_MSG(buffer, "PxBatchQueryExt::raycast - number of raycast() calls exceeds maxNbRaycasts. query discarded");
	return buffer;
}

PxSweepBuffer* ExtBatchQuery::sweep
(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance, 
 const PxU16 maxNbTouches,
 PxHitFlags hitFlags,
 const PxQueryFilterData& filterData,
 const PxQueryCache* cache, 
 const PxReal inflation)
{
	const PxQueryFilterData qfd(filterData.data, filterData.flags | PxQueryFlag::eBATCH_QUERY_LEGACY_BEHAVIOUR);
	const Sweep sweep = { geometry, pose, unitDir, distance, hitFlags, qfd, cache, inflation};
	PxSweepBuffer* buffer = mSweeps.addQuery(sweep, maxNbTouches);
	PX_CHECK_MSG(buffer, "PxBatchQueryExt::sweep - number of sweep() calls exceeds maxNbSweeps. query discarded");
	return buffer;
}

PxOverlapBuffer* ExtBatchQuery::overlap
(const PxGeometry& geometry, const PxTransform& pose, PxU16 maxNbTouches,
 const PxQueryFilterData& filterData,
 const PxQueryCache* cache)
{
	const PxQueryFilterData qfd(filterData.data, filterData.flags | PxQueryFlag::eBATCH_QUERY_LEGACY_BEHAVIOUR);
	const Overlap overlap = { geometry, pose, qfd, cache};
	PxOverlapBuffer* buffer = mOverlaps.addQuery(overlap, maxNbTouches);
	PX_CHECK_MSG(buffer, "PxBatchQueryExt::overlap - number of overlap() calls exceeds maxNbOverlaps. query discarded");
	return buffer;
}

void ExtBatchQuery::execute()
{
	mRaycasts.execute(mScene, mQueryFilterCallback);
	mSweeps.execute(mScene, mQueryFilterCallback);
	mOverlaps.execute(mScene, mQueryFilterCallback);
}
