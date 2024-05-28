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

#include "NpSceneQueries.h"

#include "common/PxProfileZone.h"
#include "GuBounds.h"
#include "CmTransformUtils.h"

#include "NpShape.h"
#include "NpActor.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "GuActorShapeMap.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

#if PX_SUPPORT_PVD
	#include "NpPvdSceneQueryCollector.h"
#endif

PX_IMPLEMENT_OUTPUT_ERROR

static PX_FORCE_INLINE NpShape* getShapeFromPayload(const PrunerPayload& payload)
{
	return reinterpret_cast<NpShape*>(payload.data[0]);
}

static PX_FORCE_INLINE NpActor* getActorFromPayload(const PrunerPayload& payload)
{
	return reinterpret_cast<NpActor*>(payload.data[1]);
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE ActorShapeData	createActorShapeData(PrunerData data, PrunerCompoundId id)	{ return (ActorShapeData(id) << 32) | ActorShapeData(data);	}
static PX_FORCE_INLINE PrunerData		getPrunerData(ActorShapeData data)							{ return PrunerData(data);									}
static PX_FORCE_INLINE PrunerCompoundId	getCompoundID(ActorShapeData data)							{ return PrunerCompoundId(data >> 32);						}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class NpSqAdapter : public QueryAdapter
	{
		public:
						NpSqAdapter()	{}
		virtual			~NpSqAdapter()	{}

		// Adapter
		virtual	const PxGeometry&	getGeometry(const PrunerPayload& payload)	const;
		//~Adapter

		// QueryAdapter
		virtual	PrunerHandle		findPrunerHandle(const PxQueryCache& cache, PrunerCompoundId& compoundId, PxU32& prunerIndex)	const;
		virtual	void				getFilterData(const PrunerPayload& payload, PxFilterData& filterData)							const;
		virtual	void				getActorShape(const PrunerPayload& payload, PxActorShape& actorShape)							const;
		//~QueryAdapter

				ActorShapeMap		mDatabase;
	};
}

PrunerHandle NpSqAdapter::findPrunerHandle(const PxQueryCache& cache, PrunerCompoundId& compoundId, PxU32& prunerIndex)	const
{
	const NpActor& npActor = NpActor::getFromPxActor(*cache.actor);

	const PxU32 actorIndex = npActor.getBaseIndex();
	PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

	const ActorShapeData actorShapeData = mDatabase.find(actorIndex, &npActor, static_cast<NpShape*>(cache.shape));

	const PrunerData prunerData = getPrunerData(actorShapeData);
	compoundId = getCompoundID(actorShapeData);

	prunerIndex = getPrunerIndex(prunerData);
	return getPrunerHandle(prunerData);
}

void NpSqAdapter::getFilterData(const PrunerPayload& payload, PxFilterData& filterData) const
{
	NpShape* npShape = getShapeFromPayload(payload);
	filterData = npShape->getQueryFilterData();
}

void NpSqAdapter::getActorShape(const PrunerPayload& payload, PxActorShape& actorShape) const
{
	NpShape* npShape = getShapeFromPayload(payload);
	NpActor* npActor = getActorFromPayload(payload);
	actorShape.actor = static_cast<PxRigidActor*>(static_cast<const Sc::RigidCore&>(npActor->getActorCore()).getPxActor());
	actorShape.shape = npShape;
	PX_ASSERT(actorShape.shape == npShape->getCore().getPxShape());
}

const PxGeometry& NpSqAdapter::getGeometry(const PrunerPayload& payload) const
{
	NpShape* npShape = getShapeFromPayload(payload);
	return npShape->getCore().getGeometry();
}

#if PX_SUPPORT_PVD
bool NpSceneQueries::transmitSceneQueries()
{
	if(!mPVDClient)
		return false;
	if(!(mPVDClient->checkPvdDebugFlag() && (mPVDClient->getScenePvdFlagsFast() & PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES)))
		return false;
	return true;
}

void NpSceneQueries::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits)
{
	mSingleSqCollector.raycast(origin, unitDir, distance, hit, hitsNum, filterData, multipleHits);
}

void NpSceneQueries::sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits)
{
	mSingleSqCollector.sweep(geometry, pose, unitDir, distance, hit, hitsNum, filterData, multipleHits);
}

void NpSceneQueries::overlap(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData)
{
	mSingleSqCollector.overlapMultiple(geometry, pose, hit, hitsNum, filterData);
}
#endif

static PX_FORCE_INLINE void setPayload(PrunerPayload& pp, const NpShape* npShape, const NpActor* npActor)
{
	pp.data[0] = size_t(npShape);
	pp.data[1] = size_t(npActor);
}
static PX_FORCE_INLINE bool isDynamicActor(const PxRigidActor& actor)
{
	const PxType actorType = actor.getConcreteType();
	return actorType != PxConcreteType::eRIGID_STATIC;
}

namespace
{
	struct DatabaseCleaner : PrunerPayloadRemovalCallback
	{
		DatabaseCleaner(NpSqAdapter& adapter) : mAdapter(adapter){}

		virtual void	invoke(PxU32 nbRemoved, const PrunerPayload* removed)	PX_OVERRIDE PX_FINAL
		{
			PxU32 actorIndex = NP_UNUSED_BASE_INDEX;
			const NpActor* cachedActor = NULL;

			while(nbRemoved--)
			{
				const PrunerPayload& payload = *removed++;

				const NpActor* npActor = getActorFromPayload(payload);

				if(npActor!=cachedActor)
				{
					actorIndex = npActor->getBaseIndex();
					cachedActor = npActor;
				}
				PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

				bool status = mAdapter.mDatabase.remove(actorIndex, npActor, getShapeFromPayload(payload), NULL);
				PX_ASSERT(status);
				PX_UNUSED(status);
			}
		}
		NpSqAdapter&	mAdapter;

		PX_NOCOPY(DatabaseCleaner)
	};
}

namespace
{
	class InternalPxSQ : public PxSceneQuerySystem, public PxUserAllocated
	{
		public:
										InternalPxSQ(const PxSceneDesc& desc, PVDCapture* pvd, PxU64 contextID, Pruner* staticPruner, Pruner* dynamicPruner) :
											mQueries(pvd, contextID, staticPruner, dynamicPruner, desc.dynamicTreeRebuildRateHint, SQ_PRUNER_EPSILON, desc.limits, mAdapter),
											mUpdateMode	(desc.sceneQueryUpdateMode),
											mRefCount	(1)
										{}
		virtual							~InternalPxSQ(){}

		PX_FORCE_INLINE	Sq::PrunerManager&			SQ()				{ return mQueries.mSQManager;	}
		PX_FORCE_INLINE	const Sq::PrunerManager&	SQ()	const		{ return mQueries.mSQManager;	}

		virtual		void				release()
		{
			mRefCount--;
			if(!mRefCount)
				PX_DELETE_THIS;
		}

		virtual		void				acquireReference()
		{
			mRefCount++;
		}

		virtual		void				preallocate(PxU32 prunerIndex, PxU32 nbShapes)
		{
			SQ().preallocate(prunerIndex, nbShapes);
		}

		// PT: TODO: returning PxSQShapeHandle means we have to store them in PhysX, inside the shape manager's mSceneQueryData array. But if we'd change the API here
		// and also pass the actor/shape couple instead of cached PxSQShapeHandle to functions like removeSQShape, it would simplify the internal PhysX code and truly
		// decouple the SQ parts from the rest. It is unclear what the consequences would be on performance: on one hand the PxSQ implementation would need a
		// hashmap or something to remap actor/shape to the SQ data, on the other hand the current code for that in PhysX is only fast for non-shared shapes.
		// (see findSceneQueryData)
		//
		// Another appealing side-effect here is that there probably wouldn't be a "compound ID" anymore: we just pass the actor/shape couple and it's up to
		// the implementation to know that this actor was added "as a compound" or not. Again, consequences on the code are unknown. We might have to just try.
		//
		// It might become quite tedious for the sync function though, since that one caches *PxSQPrunerHandles*. We don't want to do the "ref finding" equivalent each
		// frame for each shape, so some kind of cache is needed. That probably means these cached items must appear and be handled on the PhysX/internal side of
		// the API. That being said and as noted already in another part of the code:
		// PT: TODO: this is similar to findPrunerData in QueryAdapter. Maybe we could unify these.
		// => so the ref-finding code could stay, and just reuse findPrunerData (like the query cache). It wouldn't fully decouple the internal PhysX code from SQ,
		// since we'd still store an array of "PrunerData" internally. *BUT* we could still drop mSceneQueryData. So, still worth trying.
		//
		// The nail in the coffin for this idea though is that we still need to provide an internal implementation of PxSQ, and that one does use mSceneQueryData.
		// So we're struck with this array, unless we decide that there is NO internal implementation, and it's all moved to Extensions all the time.
		//
		// If we move the implementation to Extension, suddenly we need to link to SceneQuery.lib, which was not the case previously. At this point it becomes
		// appealing to just move the Sq code to Gu: it solves the link errors, we could finally include it from Sc, we'd get rid of cross-DLL calls between
		// Sq and Gu, etc
		virtual		void	addSQShape(	const PxRigidActor& actor, const PxShape& shape, const PxBounds3& bounds,
										const PxTransform& transform, const PxSQCompoundHandle* compoundHandle, bool hasPruningStructure)
		{
			const NpShape& npShape = static_cast<const NpShape&>(shape);
			const NpActor& npActor = NpActor::getFromPxActor(actor);

			PrunerPayload payload;
			setPayload(payload, &npShape, &npActor);

			const PrunerCompoundId pcid = compoundHandle ? PrunerCompoundId(*compoundHandle) : INVALID_COMPOUND_ID;

			const PrunerData prunerData = SQ().addPrunerShape(payload, isDynamicActor(actor), pcid, bounds, transform, hasPruningStructure);

			const PxU32 actorIndex = npActor.getBaseIndex();
			PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

			mAdapter.mDatabase.add(actorIndex, &npActor, &npShape, createActorShapeData(prunerData, pcid));
		}

		virtual	void						removeSQShape(const PxRigidActor& actor, const PxShape& shape)
		{
			const NpActor& npActor = NpActor::getFromPxActor(actor);
			const NpShape& npShape = static_cast<const NpShape&>(shape);

			const PxU32 actorIndex = npActor.getBaseIndex();
			PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

			ActorShapeData actorShapeData;
			mAdapter.mDatabase.remove(actorIndex, &npActor, &npShape, &actorShapeData);

			const PrunerData data = getPrunerData(actorShapeData);
			const PrunerCompoundId compoundId = getCompoundID(actorShapeData);

			SQ().removePrunerShape(compoundId, data, NULL);
		}

		virtual		void				updateSQShape(const PxRigidActor& actor, const PxShape& shape, const PxTransform& transform)
		{
			const NpActor& npActor = NpActor::getFromPxActor(actor);
			const NpShape& npShape = static_cast<const NpShape&>(shape);

			const PxU32 actorIndex = npActor.getBaseIndex();
			PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

			const ActorShapeData actorShapeData = mAdapter.mDatabase.find(actorIndex, &npActor, &npShape);

			const PrunerData shapeHandle = getPrunerData(actorShapeData);
			const PrunerCompoundId pcid = getCompoundID(actorShapeData);

			SQ().markForUpdate(pcid, shapeHandle, transform);
		}

		virtual		PxSQCompoundHandle	addSQCompound(const PxRigidActor& actor, const PxShape** shapes, const PxBVH& pxbvh, const PxTransform* transforms)
		{
			const BVH& bvh = static_cast<const BVH&>(pxbvh);

			const PxU32 numSqShapes = bvh.BVH::getNbBounds();

			const NpActor& npActor = NpActor::getFromPxActor(actor);

			PX_ALLOCA(payloads, PrunerPayload, numSqShapes);
			for(PxU32 i=0; i<numSqShapes; i++)
				setPayload(payloads[i], static_cast<const NpShape*>(shapes[i]), &npActor);

			const PxU32 actorIndex = npActor.getBaseIndex();
			PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

			PX_ALLOCA(shapeHandles, PrunerData, numSqShapes);
			SQ().addCompoundShape(bvh, PrunerCompoundId(actorIndex), actor.getGlobalPose(), shapeHandles, payloads, transforms, isDynamicActor(actor));

			for(PxU32 i=0; i<numSqShapes; i++)
			{
				// PT: TODO: actorIndex is now redundant!
				mAdapter.mDatabase.add(actorIndex, &npActor, static_cast<const NpShape*>(shapes[i]), createActorShapeData(shapeHandles[i], actorIndex));
			}

			return PxSQCompoundHandle(actorIndex);
		}

		virtual		void				removeSQCompound(PxSQCompoundHandle compoundHandle)
		{
			DatabaseCleaner cleaner(mAdapter);
			SQ().removeCompoundActor(PrunerCompoundId(compoundHandle), &cleaner);
		}

		virtual		void				updateSQCompound(PxSQCompoundHandle compoundHandle, const PxTransform& compoundTransform)
		{
			SQ().updateCompoundActor(PrunerCompoundId(compoundHandle), compoundTransform);
		}

		virtual	void							flushUpdates()														{ SQ().flushUpdates();														}
		virtual	void							flushMemory()														{ SQ().flushMemory();														}
		virtual	void							visualize(PxU32 prunerIndex, PxRenderOutput& out)			const	{ SQ().visualize(prunerIndex, out);										}
		virtual	void							shiftOrigin(const PxVec3& shift)									{ SQ().shiftOrigin(shift);													}
		virtual	PxSQBuildStepHandle				prepareSceneQueryBuildStep(PxU32 prunerIndex)						{ return SQ().prepareSceneQueriesUpdate(PruningIndex::Enum(prunerIndex));	}
		virtual	void							sceneQueryBuildStep(PxSQBuildStepHandle handle)						{ SQ().sceneQueryBuildStep(handle);										}
		virtual	void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint)			{ SQ().setDynamicTreeRebuildRateHint(dynTreeRebuildRateHint);				}
		virtual	PxU32							getDynamicTreeRebuildRateHint()								const	{ return SQ().getDynamicTreeRebuildRateHint();								}
		virtual	void							forceRebuildDynamicTree(PxU32 prunerIndex)							{ SQ().forceRebuildDynamicTree(prunerIndex);								}
		virtual	PxSceneQueryUpdateMode::Enum	getUpdateMode()												const	{ return mUpdateMode;														}
		virtual	void							setUpdateMode(PxSceneQueryUpdateMode::Enum mode)					{ mUpdateMode = mode;														}
		virtual	PxU32							getStaticTimestamp()										const	{ return SQ().getStaticTimestamp();										}

		virtual	void							finalizeUpdates()
		{
			switch(mUpdateMode)
			{
				case PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED:		SQ().afterSync(true, true);	break;
				case PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED:	SQ().afterSync(true, false);	break;
				case PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED:	SQ().afterSync(false, false);	break;
			}
		}

		virtual		void				merge(const PxPruningStructure& pxps)
		{
			Pruner* staticPruner = SQ().getPruner(PruningIndex::eSTATIC);
			if(staticPruner)
				staticPruner->merge(pxps.getStaticMergeData());

			Pruner* dynamicPruner = SQ().getPruner(PruningIndex::eDYNAMIC);
			if(dynamicPruner)
				dynamicPruner->merge(pxps.getDynamicMergeData());
		}

		virtual		bool				raycast(	const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
													PxRaycastCallback& hitCall, PxHitFlags hitFlags,
													const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
													const PxQueryCache* cache, PxGeometryQueryFlags flags) const
		{
			return mQueries._raycast(origin, unitDir, distance, hitCall, hitFlags, filterData, filterCall, cache, flags);
		}

		virtual		bool				sweep(	const PxGeometry& geometry, const PxTransform& pose,
												const PxVec3& unitDir, const PxReal distance,
												PxSweepCallback& hitCall, PxHitFlags hitFlags,
												const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
												const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const

		{
			return mQueries._sweep(geometry, pose, unitDir, distance, hitCall, hitFlags, filterData, filterCall, cache, inflation, flags);
		}

		virtual		bool				overlap(	const PxGeometry& geometry, const PxTransform& transform,
													PxOverlapCallback& hitCall, 
													const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
													const PxQueryCache* cache, PxGeometryQueryFlags flags) const
		{
			return mQueries._overlap( geometry, transform, hitCall, filterData, filterCall, cache, flags);
		}

		virtual	PxSQPrunerHandle		getHandle(const PxRigidActor& actor, const PxShape& shape, PxU32& prunerIndex)	const
		{
			const NpActor& npActor = NpActor::getFromPxActor(actor);
			const NpShape& npShape = static_cast<const NpShape&>(shape);

			const PxU32 actorIndex = npActor.getBaseIndex();
			PX_ASSERT(actorIndex!=NP_UNUSED_BASE_INDEX);

			const ActorShapeData actorShapeData = mAdapter.mDatabase.find(actorIndex, &npActor, &npShape);

			const PrunerData prunerData = getPrunerData(actorShapeData);

			prunerIndex = getPrunerIndex(prunerData);

			return PxSQPrunerHandle(getPrunerHandle(prunerData));
		}

		virtual		void				sync(PxU32 prunerIndex, const PxSQPrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds, const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)
		{
			PX_ASSERT(prunerIndex==PruningIndex::eDYNAMIC);
			if(prunerIndex==PruningIndex::eDYNAMIC)
				SQ().sync(handles, boundsIndices, bounds, transforms, count, ignoredIndices);
		}

					SceneQueries					mQueries;
					NpSqAdapter						mAdapter;
					PxSceneQueryUpdateMode::Enum	mUpdateMode;
					PxU32							mRefCount;
	};
}

#include "SqFactory.h"

static CompanionPrunerType getCompanionType(PxDynamicTreeSecondaryPruner::Enum type)
{
	switch(type)
	{
		case PxDynamicTreeSecondaryPruner::eNONE:			return COMPANION_PRUNER_NONE;
		case PxDynamicTreeSecondaryPruner::eBUCKET:			return COMPANION_PRUNER_BUCKET;
		case PxDynamicTreeSecondaryPruner::eINCREMENTAL:	return COMPANION_PRUNER_INCREMENTAL;
		case PxDynamicTreeSecondaryPruner::eBVH:			return COMPANION_PRUNER_AABB_TREE;
		case PxDynamicTreeSecondaryPruner::eLAST:			return COMPANION_PRUNER_NONE;
	}
	return COMPANION_PRUNER_NONE;
}

static BVHBuildStrategy getBuildStrategy(PxBVHBuildStrategy::Enum bs)
{
	switch(bs)
	{
		case PxBVHBuildStrategy::eFAST:		return BVH_SPLATTER_POINTS;
		case PxBVHBuildStrategy::eDEFAULT:	return BVH_SPLATTER_POINTS_SPLIT_GEOM_CENTER;
		case PxBVHBuildStrategy::eSAH:		return BVH_SAH;
		case PxBVHBuildStrategy::eLAST:		return BVH_SPLATTER_POINTS;
	}
	return BVH_SPLATTER_POINTS;
}

static Pruner* create(PxPruningStructureType::Enum type, PxU64 contextID, PxDynamicTreeSecondaryPruner::Enum secondaryType, PxBVHBuildStrategy::Enum buildStrategy, PxU32 nbObjectsPerNode)
{
	// PT: to force testing the bucket pruner
//	return createBucketPruner(contextID);
//	return createIncrementalPruner(contextID);

	const CompanionPrunerType cpType = getCompanionType(secondaryType);
	const BVHBuildStrategy bs = getBuildStrategy(buildStrategy);

	Pruner* pruner = NULL;
	switch(type)
	{
		case PxPruningStructureType::eNONE:					{ pruner = createBucketPruner(contextID);										break;	}
		case PxPruningStructureType::eDYNAMIC_AABB_TREE:	{ pruner = createAABBPruner(contextID, true, cpType, bs, nbObjectsPerNode);		break;	}
		case PxPruningStructureType::eSTATIC_AABB_TREE:		{ pruner = createAABBPruner(contextID, false, cpType, bs, nbObjectsPerNode);	break;	}
		// PT: for tests
		case PxPruningStructureType::eLAST:					{ pruner = createIncrementalPruner(contextID);									break;	}
//		case PxPruningStructureType::eLAST:					break;
	}
	return pruner;
}

static PxSceneQuerySystem* getPxSQ(const PxSceneDesc& desc, PVDCapture* pvd, PxU64 contextID)
{
	if(desc.sceneQuerySystem)
	{
		desc.sceneQuerySystem->acquireReference();
		return desc.sceneQuerySystem;
	}
	else
	{
		Pruner* staticPruner = create(desc.staticStructure, contextID, desc.dynamicTreeSecondaryPruner, desc.staticBVHBuildStrategy, desc.staticNbObjectsPerNode);
		Pruner* dynamicPruner = create(desc.dynamicStructure, contextID, desc.dynamicTreeSecondaryPruner, desc.dynamicBVHBuildStrategy, desc.dynamicNbObjectsPerNode);
		return PX_NEW(InternalPxSQ)(desc, pvd, contextID, staticPruner, dynamicPruner);
	}
}

#if PX_SUPPORT_PVD
	#define PVD_PARAM	this
#else
	#define PVD_PARAM	NULL
#endif

NpSceneQueries::NpSceneQueries(const PxSceneDesc& desc, Vd::PvdSceneClient* pvd, PxU64 contextID) :
	mSQ					(getPxSQ(desc, PVD_PARAM, contextID))
#if PX_SUPPORT_PVD
	// PT: warning, pvd object not created yet at this point
	,mPVDClient			(pvd)
    ,mSingleSqCollector	(pvd, false)
#endif
{
	PX_UNUSED(pvd);
	PX_UNUSED(contextID);
}

NpSceneQueries::~NpSceneQueries()
{
	if(mSQ)
	{
		mSQ->release();
		mSQ = NULL;
	}
}

void NpSceneQueries::sync(PxU32 prunerIndex, const ScPrunerHandle* handles, const PxU32* indices, const PxBounds3* bounds,
							const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)
{
	mSQ->sync(prunerIndex, handles, indices, bounds, transforms, count, ignoredIndices);
}


#include "NpScene.h"

// PT: TODO: eventually move NP_READ_CHECK to internal PxSQ version ?

bool NpScene::raycast(
	const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
	PxHitCallback<PxRaycastHit>& hits, PxHitFlags hitFlags, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, PxGeometryQueryFlags flags) const
{
	NP_READ_CHECK(this);
	return mNpSQ.mSQ->raycast(origin, unitDir, distance, hits, hitFlags, filterData, filterCall, cache, flags);
}

bool NpScene::overlap(
	const PxGeometry& geometry, const PxTransform& pose, PxOverlapCallback& hits,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, PxGeometryQueryFlags flags) const
{
	NP_READ_CHECK(this);
	return mNpSQ.mSQ->overlap(geometry, pose, hits, filterData, filterCall, cache, flags);
}

bool NpScene::sweep(
	const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
	PxHitCallback<PxSweepHit>& hits, PxHitFlags hitFlags, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const
{
	NP_READ_CHECK(this);
	return mNpSQ.mSQ->sweep(geometry, pose, unitDir, distance, hits, hitFlags, filterData, filterCall, cache, inflation, flags);
}

void NpScene::setUpdateMode(PxSceneQueryUpdateMode::Enum updateMode)
{
	NP_WRITE_CHECK(this);

	getSQAPI().setUpdateMode(updateMode);
	updatePvdProperties();
}

PxSceneQueryUpdateMode::Enum NpScene::getUpdateMode() const
{
	NP_READ_CHECK(this);
	return getSQAPI().getUpdateMode();
}

void NpScene::setDynamicTreeRebuildRateHint(PxU32 dynamicTreeRebuildRateHint)
{
	NP_WRITE_CHECK(this);
	PX_CHECK_AND_RETURN((dynamicTreeRebuildRateHint >= 4), "PxScene::setDynamicTreeRebuildRateHint(): Param has to be >= 4!");

	getSQAPI().setDynamicTreeRebuildRateHint(dynamicTreeRebuildRateHint);
	updatePvdProperties();
}

PxU32 NpScene::getDynamicTreeRebuildRateHint() const
{
	NP_READ_CHECK(this);
	return getSQAPI().getDynamicTreeRebuildRateHint();
}

void NpScene::forceRebuildDynamicTree(PxU32 prunerIndex)
{
	PX_PROFILE_ZONE("API.forceDynamicTreeRebuild", getContextId());
	NP_WRITE_CHECK(this);

	PX_SIMD_GUARD;
	getSQAPI().forceRebuildDynamicTree(prunerIndex);
}

PxU32 NpScene::getStaticTimestamp() const
{
	return getSQAPI().getStaticTimestamp();
}

PxPruningStructureType::Enum NpScene::getStaticStructure() const
{
	return mPrunerType[0];
}

PxPruningStructureType::Enum NpScene::getDynamicStructure() const
{
	return mPrunerType[1];
}

void NpScene::flushUpdates()
{
	PX_PROFILE_ZONE("API.flushQueryUpdates", getContextId());
	NP_WRITE_CHECK(this);

	PX_SIMD_GUARD;

	getSQAPI().flushUpdates();
}

namespace
{
	class SqRefFinder: public Sc::SqRefFinder
	{
		PX_NOCOPY(SqRefFinder)
	public:
		SqRefFinder(const PxSceneQuerySystem& pxsq) : mPXSQ(pxsq)	{}
		const PxSceneQuerySystem&	mPXSQ;

		virtual	ScPrunerHandle find(const PxRigidBody* body, const PxShape* shape, PxU32& prunerIndex)
		{
			return mPXSQ.getHandle(*body, *shape, prunerIndex);
		}
	};
}

void NpScene::syncSQ()
{
	PxSceneQuerySystem& pm = getSQAPI();

	{
		const PxU32 numBodies = mScene.getNumActiveCompoundBodies();
		const Sc::BodyCore*const* bodies = mScene.getActiveCompoundBodiesArray();

		// PT: we emulate "getGlobalPose" here by doing the equivalent matrix computation directly.
		// This works because the code is the same for rigid dynamic & articulation links.
		// PT: TODO: SIMD
		for(PxU32 i = 0; i < numBodies; i++)
		{
			// PT: we don't have access to Np from here so we have to go through Px, which is a bit ugly.
			// If this creates perf issues an alternative would be to just store the ID along with the body
			// pointer in the active compound bodies array.
			PxActor* actor = bodies[i]->getPxActor();
			PX_ASSERT(actor);

			const PxU32 id = static_cast<PxRigidActor*>(actor)->getInternalActorIndex();
			PX_ASSERT(id!=0xffffffff);

			pm.updateSQCompound(PxSQCompoundHandle(id), bodies[i]->getBody2World() * bodies[i]->getBody2Actor().getInverse());
		}
	}

	SqRefFinder sqRefFinder(pm);
	mScene.syncSceneQueryBounds(mNpSQ, sqRefFinder);

	pm.finalizeUpdates();
}

void NpScene::forceSceneQueryRebuild()
{
	// PT: what is this function anyway? What's the difference between this and forceDynamicTreeRebuild ? Why is the implementation different?
	syncSQ();
}

void NpScene::sceneQueriesStaticPrunerUpdate(PxBaseTask* )
{
	PX_PROFILE_ZONE("SceneQuery.sceneQueriesStaticPrunerUpdate", getContextId());
	// run pruner build only, this will build the new tree only, no commit happens
	getSQAPI().sceneQueryBuildStep(mStaticBuildStepHandle);
}

void NpScene::sceneQueriesDynamicPrunerUpdate(PxBaseTask*)
{
	PX_PROFILE_ZONE("SceneQuery.sceneQueriesDynamicPrunerUpdate", getContextId());
	// run pruner build only, this will build the new tree only, no commit happens
	getSQAPI().sceneQueryBuildStep(mDynamicBuildStepHandle);
}

void NpScene::sceneQueriesUpdate(PxBaseTask* completionTask, bool controlSimulation)
{
	PX_SIMD_GUARD;

	PxSQBuildStepHandle runUpdateTasksStatic = NULL;
	PxSQBuildStepHandle runUpdateTasksDynamic = NULL;
	{
		// write guard must end before scene queries tasks kicks off worker threads
		NP_WRITE_CHECK(this);

		PX_PROFILE_START_CROSSTHREAD("Basic.sceneQueriesUpdate", getContextId());

		if(mSQUpdateRunning)
		{
			//fetchSceneQueries doesn't get called
			outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchSceneQueries was not called!");
			return;
		}
	
		PxSceneQuerySystem& pxsq = getSQAPI();

		// flush scene queries updates
		pxsq.flushUpdates();

		// prepare scene queries for build - copy bounds
		runUpdateTasksStatic = pxsq.prepareSceneQueryBuildStep(PX_SCENE_PRUNER_STATIC);
		runUpdateTasksDynamic = pxsq.prepareSceneQueryBuildStep(PX_SCENE_PRUNER_DYNAMIC);
		mStaticBuildStepHandle = runUpdateTasksStatic;
		mDynamicBuildStepHandle = runUpdateTasksDynamic;
		mSQUpdateRunning = true;
	}

	{
		PX_PROFILE_ZONE("Sim.sceneQueriesTaskSetup", getContextId());

		if (controlSimulation)
		{
			{
				PX_PROFILE_ZONE("Sim.resetDependencies", getContextId());
				// Only reset dependencies, etc if we own the TaskManager. Will be false
				// when an NpScene is controlled by an APEX scene.
				mTaskManager->resetDependencies();
			}
			mTaskManager->startSimulation();
		}

		mSceneQueriesCompletion.setContinuation(*mTaskManager, completionTask);
		if(runUpdateTasksStatic)
			mSceneQueriesStaticPrunerUpdate.setContinuation(&mSceneQueriesCompletion);
		if(runUpdateTasksDynamic)
			mSceneQueriesDynamicPrunerUpdate.setContinuation(&mSceneQueriesCompletion);

		mSceneQueriesCompletion.removeReference();
		if(runUpdateTasksStatic)
			mSceneQueriesStaticPrunerUpdate.removeReference();
		if(runUpdateTasksDynamic)
			mSceneQueriesDynamicPrunerUpdate.removeReference();
	}
}

bool NpScene::checkSceneQueriesInternal(bool block)
{
	PX_PROFILE_ZONE("Basic.checkSceneQueries", getContextId());
	return mSceneQueriesDone.wait(block ? PxSync::waitForever : 0);
}

bool NpScene::checkQueries(bool block)
{
	return checkSceneQueriesInternal(block);
}

bool NpScene::fetchQueries(bool block)
{
	if(!mSQUpdateRunning)
	{
		//fetchSceneQueries doesn't get called
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxScene::fetchQueries: fetchQueries() called illegally! It must be called after sceneQueriesUpdate()");
		return false;
	}

	if(!checkSceneQueriesInternal(block))
		return false;

	{
		PX_SIMD_GUARD;

		NP_WRITE_CHECK(this);

		// we use cross thread profile here, to show the event in cross thread view
		// PT: TODO: why do we want to show it in the cross thread view?
		PX_PROFILE_START_CROSSTHREAD("Basic.fetchQueries", getContextId());

		// flush updates and commit if work is done
		getSQAPI().flushUpdates();
	
		PX_PROFILE_STOP_CROSSTHREAD("Basic.fetchQueries", getContextId());
		PX_PROFILE_STOP_CROSSTHREAD("Basic.sceneQueriesUpdate", getContextId());

		mSceneQueriesDone.reset();
		mSQUpdateRunning = false;
	}
	return true;
}
