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

#ifndef GU_QUERY_SYSTEM_H
#define GU_QUERY_SYSTEM_H

#include "foundation/PxUserAllocated.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"
#include "foundation/PxBounds3.h"
#include "GuPruner.h"
#include "GuActorShapeMap.h"

namespace physx
{
	class PxGeometry;

namespace Gu
{
	class BVH;

	class Adapter
	{
		public:
									Adapter()	{}
		virtual						~Adapter()	{}

		virtual	const PxGeometry&	getGeometry(const PrunerPayload& payload)	const	= 0;
	};

	class PrunerFilter
	{
		public:
									PrunerFilter()	{}
		virtual						~PrunerFilter()	{}

		virtual	bool				processPruner(PxU32 prunerIndex/*, const PxQueryThreadContext* context*/)	const	= 0;
	};

	typedef PxU32	PrunerInfo;
	PX_FORCE_INLINE PrunerInfo		createPrunerInfo(PxU32 prunerIndex, bool isDynamic)		{ return (prunerIndex << 1) | PxU32(isDynamic);	}
	PX_FORCE_INLINE PxU32			getPrunerIndex(PrunerInfo info)							{ return PxU32(info)>>1;						}
	PX_FORCE_INLINE PxU32			getDynamic(PrunerInfo info)								{ return PxU32(info) & 1;						}

	PX_FORCE_INLINE ActorShapeData	createActorShapeData(PrunerInfo info, PrunerHandle h)	{ return (ActorShapeData(h) << 32) | ActorShapeData(info);	}
	PX_FORCE_INLINE PrunerInfo		getPrunerInfo(ActorShapeData data)						{ return PrunerInfo(data);									}
	PX_FORCE_INLINE PrunerHandle	getPrunerHandle(ActorShapeData data)					{ return PrunerHandle(data >> 32);							}

	#define INVALID_ACTOR_SHAPE_DATA	PxU64(-1)

	class QuerySystem : public PxUserAllocated
	{
		public:	// PT: TODO: public only to implement checkPrunerIndex easily, revisit this
		struct PrunerExt : public PxUserAllocated
		{
									PrunerExt(Pruner* pruner, PxU32 preallocated);
									~PrunerExt();

			void					flushMemory();

			void					addToDirtyList(PrunerHandle handle, PxU32 dynamic, const PxTransform& transform, const PxBounds3* userBounds=NULL);
			void					removeFromDirtyList(PrunerHandle handle);
			bool					processDirtyList(const Adapter& adapter, float inflation);

			Pruner*					mPruner;
			PxBitMap				mDirtyMap;
			PxArray<PrunerHandle>	mDirtyList;
			PxU32					mNbStatic;		// nb static objects in pruner
			PxU32					mNbDynamic;		// nb dynamic objects in pruner
			bool					mDirtyStatic;	// true if dirty list contains a static

			struct Data
			{
				PxTransform	mPose;
				PxBounds3	mBounds;
			};
			PxArray<Data>			mDirtyData;

			PX_NOCOPY(PrunerExt)
		};

	public:
		PX_PHYSX_COMMON_API							QuerySystem(PxU64 contextID, float inflation, const Adapter& adapter, bool usesTreeOfPruners=false);
		PX_PHYSX_COMMON_API							~QuerySystem();

		PX_FORCE_INLINE		PxU64					getContextId()						const	{ return mContextID;		}
		PX_FORCE_INLINE		const Adapter&			getAdapter()						const	{ return mAdapter;			}
		PX_FORCE_INLINE		PxU32					getStaticTimestamp()				const	{ return mStaticTimestamp;	}

		PX_PHYSX_COMMON_API	PxU32					addPruner(Pruner* pruner, PxU32 preallocated);
		PX_PHYSX_COMMON_API	void					removePruner(PxU32 prunerIndex);
		PX_FORCE_INLINE		PxU32					getNbPruners()						const	{ return mPrunerExt.size();				}
		PX_FORCE_INLINE		const Pruner*			getPruner(PxU32 index)				const	{ return mPrunerExt[index]->mPruner;	}
		PX_FORCE_INLINE		Pruner*					getPruner(PxU32 index)						{ return mPrunerExt[index]->mPruner;	}

		PX_PHYSX_COMMON_API	ActorShapeData			addPrunerShape(const PrunerPayload& payload, PxU32 prunerIndex, bool dynamic, const PxTransform& transform, const PxBounds3* userBounds=NULL);
		PX_PHYSX_COMMON_API	void					removePrunerShape(ActorShapeData data, PrunerPayloadRemovalCallback* removalCallback);
		PX_PHYSX_COMMON_API	void					updatePrunerShape(ActorShapeData data, bool immediately, const PxTransform& transform, const PxBounds3* userBounds=NULL);

		PX_PHYSX_COMMON_API	const PrunerPayload&	getPayloadData(ActorShapeData data, PrunerPayloadData* ppd=NULL)	const;

		PX_PHYSX_COMMON_API	void					commitUpdates();
		PX_PHYSX_COMMON_API	void					update(bool buildStep, bool commit);
		PX_PHYSX_COMMON_API	void					sync(PxU32 prunerIndex, const PrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count);

		PX_PHYSX_COMMON_API	void					flushMemory();

		PX_PHYSX_COMMON_API	void					raycast(const PxVec3& origin, const PxVec3& unitDir, float& inOutDistance, PrunerRaycastCallback& cb, const PrunerFilter* prunerFilter)			const;
		PX_PHYSX_COMMON_API	void					overlap(const ShapeData& queryVolume, PrunerOverlapCallback& cb, const PrunerFilter* prunerFilter)												const;
		PX_PHYSX_COMMON_API	void					sweep(const ShapeData& queryVolume, const PxVec3& unitDir, float& inOutDistance, PrunerRaycastCallback& cb, const PrunerFilter* prunerFilter)	const;

							PxU32					startCustomBuildstep();
							void					customBuildstep(PxU32 index);
							void					finishCustomBuildstep();

							void					createTreeOfPruners();
	private:
							const Adapter&			mAdapter;
							PxArray<PrunerExt*>		mPrunerExt;
							PxArray<PxU32>			mDirtyPruners;
							PxArray<PxU32>			mFreePruners;

							Gu::BVH*				mTreeOfPruners;

							const PxU64				mContextID;
							PxU32					mStaticTimestamp;
							const float				mInflation;	// SQ_PRUNER_EPSILON

							PxMutex					mSQLock;  // to make sure only one query updates the dirty pruner structure if multiple queries run in parallel

							volatile bool			mPrunerNeedsUpdating;
							volatile bool			mTimestampNeedsUpdating;
							const bool				mUsesTreeOfPruners;

							void					processDirtyLists();
		PX_FORCE_INLINE		void					invalidateStaticTimestamp()		{ mStaticTimestamp++;		}

							PX_NOCOPY(QuerySystem)
	};
}
}


#include "geometry/PxGeometryHit.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "GuCachedFuncs.h"
#include "GuCapsule.h"
#include "GuBounds.h"

#if PX_VC
#pragma warning(disable: 4355 )	// "this" used in base member initializer list
#endif

namespace physx
{
namespace Gu
{
	// PT: TODO: use templates instead of v-calls?

	// PT: we decouple the filter callback from the rest, so that the same filter callback can easily be reused for all pruner queries.
	// This combines the pre-filter callback and fetching the payload's geometry in a single call. Return null to ignore that object.
	struct PrunerFilterCallback
	{
		virtual	~PrunerFilterCallback()	{}

		// Query's hit flags can be tweaked per object. (Note that 'hitFlags' is unused for overlaps though)
		virtual	const PxGeometry*	validatePayload(const PrunerPayload& payload, PxHitFlags& hitFlags) = 0;
	};

	struct DefaultPrunerRaycastCallback : public PrunerRaycastCallback, public PxRaycastThreadContext
	{
		PxRaycastThreadContext*	mContext;
		PrunerFilterCallback&	mFilterCB;
		const GeomRaycastTable&	mCachedRaycastFuncs;
		const PxVec3&			mOrigin;
		const PxVec3&			mDir;
		PxGeomRaycastHit*		mLocalHits;
		const PxU32				mMaxLocalHits;
		const PxHitFlags		mHitFlags;
		PxGeomRaycastHit		mClosestHit;
		PrunerPayload			mClosestPayload;
		bool					mFoundHit;
		const bool				mAnyHit;

		DefaultPrunerRaycastCallback(PrunerFilterCallback& filterCB, const GeomRaycastTable& funcs, const PxVec3& origin, const PxVec3& dir, float distance, PxU32 maxLocalHits, PxGeomRaycastHit* localHits, PxHitFlags hitFlags, bool anyHit, PxRaycastThreadContext* context=NULL) :
			mContext			(context ? context : this),
			mFilterCB			(filterCB),
			mCachedRaycastFuncs	(funcs),
			mOrigin				(origin),
			mDir				(dir),
			mLocalHits			(localHits),
			mMaxLocalHits		(maxLocalHits),
			mHitFlags			(hitFlags),
			mFoundHit			(false),
			mAnyHit				(anyHit)
		{
			mClosestHit.distance = distance;
		}

		virtual	bool	reportHits(const PrunerPayload& /*payload*/, PxU32 /*nbHits*/, PxGeomRaycastHit* /*hits*/)
		{
			return true;
		}

		virtual bool	invoke(PxReal& aDist, PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms)	PX_OVERRIDE PX_FINAL
		{
			PX_ASSERT(payloads && transforms);

			const PrunerPayload& payload = payloads[primIndex];

			PxHitFlags filteredHitFlags = mHitFlags;
			const PxGeometry* shapeGeom = mFilterCB.validatePayload(payload, filteredHitFlags);
			if(!shapeGeom)
				return true;

			const RaycastFunc func = mCachedRaycastFuncs[shapeGeom->getType()];
			const PxU32 nbHits = func(*shapeGeom, transforms[primIndex], mOrigin, mDir, aDist, filteredHitFlags, mMaxLocalHits, mLocalHits, sizeof(PxGeomRaycastHit), mContext);
			if(!nbHits || !reportHits(payload, nbHits, mLocalHits))
				return true;

			const PxGeomRaycastHit& localHit = mLocalHits[0];
			if(localHit.distance < mClosestHit.distance)
			{
				mFoundHit = true;
				if(mAnyHit)
					return false;

				aDist = localHit.distance;
				mClosestHit = localHit;
				mClosestPayload = payload;
			}
			return true;
		}

		PX_NOCOPY(DefaultPrunerRaycastCallback)
	};

	struct DefaultPrunerRaycastAnyCallback : public DefaultPrunerRaycastCallback
	{
		PxGeomRaycastHit	mLocalHit;

		DefaultPrunerRaycastAnyCallback(PrunerFilterCallback& filterCB, const GeomRaycastTable& funcs, const PxVec3& origin, const PxVec3& dir, float distance) :
			DefaultPrunerRaycastCallback	(filterCB, funcs, origin, dir, distance, 1, &mLocalHit, PxHitFlag::eANY_HIT, true)	{}
	};

	struct DefaultPrunerRaycastClosestCallback : public DefaultPrunerRaycastCallback
	{
		PxGeomRaycastHit	mLocalHit;

		DefaultPrunerRaycastClosestCallback(PrunerFilterCallback& filterCB, const GeomRaycastTable& funcs, const PxVec3& origin, const PxVec3& dir, float distance, PxHitFlags hitFlags) :
			DefaultPrunerRaycastCallback	(filterCB, funcs, origin, dir, distance, 1, &mLocalHit, hitFlags, false)	{}
	};

	struct DefaultPrunerOverlapCallback : public PrunerOverlapCallback, public PxOverlapThreadContext
	{
		PxOverlapThreadContext*	mContext;
		PrunerFilterCallback&	mFilterCB;
		const GeomOverlapTable*	mCachedFuncs;
		const PxGeometry&		mGeometry;
		const PxTransform&		mPose;
		PxHitFlags				mUnused;

		DefaultPrunerOverlapCallback(PrunerFilterCallback& filterCB, const GeomOverlapTable* funcs, const PxGeometry& geometry, const PxTransform& pose, PxOverlapThreadContext* context=NULL) :
			mContext		(context ? context : this),
			mFilterCB		(filterCB),
			mCachedFuncs	(funcs),
			mGeometry		(geometry),
			mPose			(pose)
		{
		}

		virtual	bool	reportHit(const PrunerPayload& /*payload*/)
		{
			return true;
		}

		virtual bool	invoke(PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms)	PX_OVERRIDE PX_FINAL
		{
			PX_ASSERT(payloads && transforms);

			const PrunerPayload& payload = payloads[primIndex];

			const PxGeometry* shapeGeom = mFilterCB.validatePayload(payload, mUnused);
			if(!shapeGeom || !Gu::overlap(mGeometry, mPose, *shapeGeom, transforms[primIndex], mCachedFuncs, mContext))
				return true;

			return reportHit(payload);
		}

		PX_NOCOPY(DefaultPrunerOverlapCallback)
	};

	struct BoxShapeCast
	{
		static PX_FORCE_INLINE PxU32 sweep(	const GeomSweepFuncs& sf, const PxGeometry& geom, const PxTransform& pose,
											const PxGeometry& queryGeom, const PxTransform& queryPose, const ShapeData& queryVolume,
											const PxVec3& unitDir, PxReal distance, PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, PxReal inflation, PxSweepThreadContext* context)
		{
			PX_ASSERT(queryGeom.getType()==PxGeometryType::eBOX);
			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepBoxFunc func = precise ? sf.preciseBoxMap[geom.getType()] : sf.boxMap[geom.getType()];
			return PxU32(func(geom, pose, static_cast<const PxBoxGeometry&>(queryGeom), queryPose, queryVolume.getGuBox(), unitDir, distance, sweepHit, hitFlags, inflation, context));
		}
	};

	struct SphereShapeCast
	{
		static PX_FORCE_INLINE PxU32 sweep(	const GeomSweepFuncs& sf, const PxGeometry& geom, const PxTransform& pose,
											const PxGeometry& queryGeom, const PxTransform& queryPose, const ShapeData& /*queryVolume*/,
											const PxVec3& unitDir, PxReal distance, PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, PxReal inflation, PxSweepThreadContext* context)
		{
			PX_ASSERT(queryGeom.getType()==PxGeometryType::eSPHERE);
			// PT: we don't use sd.getGuSphere() here because PhysX doesn't expose a set of 'SweepSphereFunc' functions,
			// we have to go through a capsule (which is then seen as a sphere internally when the half-length is zero).
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(queryGeom);
			const PxCapsuleGeometry capsuleGeom(sphereGeom.radius, 0.0f);
			const Capsule worldCapsule(queryPose.p, queryPose.p, sphereGeom.radius); // AP: precompute?
			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom.getType()] : sf.capsuleMap[geom.getType()];
			return PxU32(func(geom, pose, capsuleGeom, queryPose, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation, context));
		}
	};

	struct CapsuleShapeCast
	{
		static PX_FORCE_INLINE PxU32 sweep(	const GeomSweepFuncs& sf, const PxGeometry& geom, const PxTransform& pose,
											const PxGeometry& queryGeom, const PxTransform& queryPose, const ShapeData& queryVolume,
											const PxVec3& unitDir, PxReal distance, PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, PxReal inflation, PxSweepThreadContext* context)
		{
			PX_ASSERT(queryGeom.getType()==PxGeometryType::eCAPSULE);
			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom.getType()] : sf.capsuleMap[geom.getType()];
			return PxU32(func(geom, pose, static_cast<const PxCapsuleGeometry&>(queryGeom), queryPose, queryVolume.getGuCapsule(), unitDir, distance, sweepHit, hitFlags, inflation, context));
		}
	};

	struct ConvexShapeCast
	{
		static PX_FORCE_INLINE PxU32 sweep(	const GeomSweepFuncs& sf, const PxGeometry& geom, const PxTransform& pose,
											const PxGeometry& queryGeom, const PxTransform& queryPose, const ShapeData& /*queryVolume*/,
											const PxVec3& unitDir, PxReal distance, PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, PxReal inflation, PxSweepThreadContext* context)
		{
			PX_ASSERT(queryGeom.getType()==PxGeometryType::eCONVEXMESH);
			const SweepConvexFunc func = sf.convexMap[geom.getType()];
			return PxU32(func(geom, pose, static_cast<const PxConvexMeshGeometry&>(queryGeom), queryPose, unitDir, distance, sweepHit, hitFlags, inflation, context));
		}
	};

	struct DefaultPrunerSweepCallback : public PrunerRaycastCallback, public PxSweepThreadContext
	{
		virtual	bool	reportHit(const PrunerPayload& /*payload*/, PxGeomSweepHit& /*hit*/)
		{
			return true;
		}
	};

	template<class ShapeCast>
	struct DefaultPrunerSweepCallbackT : public DefaultPrunerSweepCallback
	{
		PxSweepThreadContext*	mContext;
		PrunerFilterCallback&	mFilterCB;
		const GeomSweepFuncs&	mCachedFuncs;
		const PxGeometry&		mGeometry;
		const PxTransform&		mPose;
		const ShapeData&		mQueryVolume;
		const PxVec3&			mDir;
		PxGeomSweepHit			mLocalHit;
		const PxHitFlags		mHitFlags;
		PxGeomSweepHit			mClosestHit;
		PrunerPayload			mClosestPayload;
		bool					mFoundHit;
		const bool				mAnyHit;

		DefaultPrunerSweepCallbackT(PrunerFilterCallback& filterCB, const GeomSweepFuncs& funcs,
									const PxGeometry& geometry, const PxTransform& pose, const ShapeData& queryVolume,
									const PxVec3& dir, float distance, PxHitFlags hitFlags, bool anyHit, PxSweepThreadContext* context=NULL) :
			mContext		(context ? context : this),
			mFilterCB		(filterCB),
			mCachedFuncs	(funcs),
			mGeometry		(geometry),
			mPose			(pose),
			mQueryVolume	(queryVolume),
			mDir			(dir),
			mHitFlags		(hitFlags),
			mFoundHit		(false),
			mAnyHit			(anyHit)
		{
			mClosestHit.distance = distance;
		}

		virtual bool	invoke(PxReal& aDist, PxU32 primIndex, const PrunerPayload* payloads, const PxTransform* transforms)	PX_OVERRIDE PX_FINAL
		{
			PX_ASSERT(payloads && transforms);

			const PrunerPayload& payload = payloads[primIndex];

			PxHitFlags filteredHitFlags = mHitFlags;
			const PxGeometry* shapeGeom = mFilterCB.validatePayload(payload, filteredHitFlags);
			if(!shapeGeom)
				return true;

			// PT: ### TODO: missing bit from PhysX version here

			const float inflation = 0.0f;	// ####
			const PxU32 retVal = ShapeCast::sweep(mCachedFuncs, *shapeGeom, transforms[primIndex], mGeometry, mPose, mQueryVolume, mDir, aDist, mLocalHit, filteredHitFlags, inflation, mContext);

			if(!retVal || !reportHit(payload, mLocalHit))
				return true;

			if(mLocalHit.distance < mClosestHit.distance)
			{
				mFoundHit = true;
				if(mAnyHit)
					return false;

				aDist = mLocalHit.distance;
				mClosestHit = mLocalHit;
				mClosestPayload = payload;
			}
			return true;
		}

		PX_NOCOPY(DefaultPrunerSweepCallbackT)
	};

typedef DefaultPrunerSweepCallbackT<SphereShapeCast>	DefaultPrunerSphereSweepCallback;
typedef DefaultPrunerSweepCallbackT<BoxShapeCast>		DefaultPrunerBoxSweepCallback;
typedef DefaultPrunerSweepCallbackT<CapsuleShapeCast>	DefaultPrunerCapsuleSweepCallback;
typedef DefaultPrunerSweepCallbackT<ConvexShapeCast>	DefaultPrunerConvexSweepCallback;

}
}

#endif
