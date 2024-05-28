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

#include "extensions/PxCustomSceneQuerySystem.h"
#include "extensions/PxShapeExt.h"
#include "foundation/PxAlloca.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxUserAllocated.h"
#include "geometry/PxBVH.h"
#include "GuActorShapeMap.h"
#include "ExtSqQuery.h"
#include "SqFactory.h"
#include "PxRigidActor.h"
#include "PxPruningStructure.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

// PT: this customized version uses:
// - a modified version of Sq::PrunerManager, named Sq::ExtPrunerManager, located in ExtSqManager.cpp
// - a modified version of Sq::SceneQueries, named Sq::ExtSceneQueries, located in ExtSqQuery.cpp
//
// Sq::PrunerManager and Sq::SceneQueries live in the SceneQuery lib, and are used by PhysX internally
// to implement the regular SQ system.
//
// Sq::ExtPrunerManager and Sq::ExtSceneQueries live in the Extensions lib, and are not used by the
// regular PhysX SQ system. They are examples of how the default code can be customized.
//

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
//	if(0)
//		return createIncrementalPruner(contextID);

	const CompanionPrunerType cpType = getCompanionType(secondaryType);
	const BVHBuildStrategy bs = getBuildStrategy(buildStrategy);

	Pruner* pruner = NULL;
	switch(type)
	{
		case PxPruningStructureType::eNONE:					{ pruner = createBucketPruner(contextID);										break;	}
		case PxPruningStructureType::eDYNAMIC_AABB_TREE:	{ pruner = createAABBPruner(contextID, true, cpType, bs, nbObjectsPerNode);		break;	}
		case PxPruningStructureType::eSTATIC_AABB_TREE:		{ pruner = createAABBPruner(contextID, false, cpType, bs, nbObjectsPerNode);	break;	}
		case PxPruningStructureType::eLAST:					break;
	}
	return pruner;
}

#define EXT_PRUNER_EPSILON	0.005f

// PT: in this external implementation we'll use Px pointers instead of Np pointers in the payload.
static PX_FORCE_INLINE void setPayload(PrunerPayload& pp, const PxShape* shape, const PxRigidActor* actor)
{
	pp.data[0] = size_t(shape);
	pp.data[1] = size_t(actor);
}

static PX_FORCE_INLINE PxShape* getShapeFromPayload(const PrunerPayload& payload)
{
	return reinterpret_cast<PxShape*>(payload.data[0]);
}

static PX_FORCE_INLINE PxRigidActor* getActorFromPayload(const PrunerPayload& payload)
{
	return reinterpret_cast<PxRigidActor*>(payload.data[1]);
}

static PX_FORCE_INLINE bool isDynamicActor(const PxRigidActor& actor)
{
	const PxType actorType = actor.getConcreteType();
	return actorType != PxConcreteType::eRIGID_STATIC;
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE ActorShapeData	createActorShapeData(PrunerHandle h, PrunerCompoundId id)	{ return (ActorShapeData(id) << 32) | ActorShapeData(h);	}
static PX_FORCE_INLINE PrunerHandle		getPrunerHandle(ActorShapeData data)						{ return PrunerHandle(data);								}
static PX_FORCE_INLINE PrunerCompoundId	getCompoundID(ActorShapeData data)							{ return PrunerCompoundId(data >> 32);						}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class ExtSqAdapter : public ExtQueryAdapter
	{
									PX_NOCOPY(ExtSqAdapter)
		public:
									ExtSqAdapter(const PxCustomSceneQuerySystemAdapter&	adapter) : mUserAdapter(adapter)/*, mFilterData(NULL)*/	{}
		virtual						~ExtSqAdapter()	{}

		// Adapter
		virtual	const PxGeometry&	getGeometry(const PrunerPayload& payload)	const;
		//~Adapter

		// ExtQueryAdapter
		virtual	PrunerHandle		findPrunerHandle(const PxQueryCache& cache, PrunerCompoundId& compoundId, PxU32& prunerIndex)	const;
		virtual	void				getFilterData(const PrunerPayload& payload, PxFilterData& filterData)							const;
		virtual	void				getActorShape(const PrunerPayload& payload, PxActorShape& actorShape)							const;
		virtual	bool				processPruner(PxU32 prunerIndex, const PxQueryThreadContext* context, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall)	const;
		//~ExtQueryAdapter

				const PxCustomSceneQuerySystemAdapter&	mUserAdapter;
				ActorShapeMap							mDatabase;
				const PxQueryFilterData*				mFilterData;

	};
}

const PxGeometry& ExtSqAdapter::getGeometry(const PrunerPayload& payload) const
{
	PxShape* shape = getShapeFromPayload(payload);
	return shape->getGeometry();
}

PrunerHandle ExtSqAdapter::findPrunerHandle(const PxQueryCache& cache, PrunerCompoundId& compoundId, PxU32& prunerIndex) const
{
	const PxU32 actorIndex = cache.actor->getInternalActorIndex();
	PX_ASSERT(actorIndex!=0xffffffff);

	const ActorShapeData actorShapeData = mDatabase.find(actorIndex, cache.actor, cache.shape);

	compoundId = getCompoundID(actorShapeData);

	prunerIndex = mUserAdapter.getPrunerIndex(*cache.actor, *cache.shape);

	return getPrunerHandle(actorShapeData);
}

void ExtSqAdapter::getFilterData(const PrunerPayload& payload, PxFilterData& filterData) const
{
	PxShape* shape = getShapeFromPayload(payload);
	filterData = shape->getQueryFilterData();
}

void ExtSqAdapter::getActorShape(const PrunerPayload& payload, PxActorShape& actorShape) const
{
	actorShape.actor = getActorFromPayload(payload);
	actorShape.shape = getShapeFromPayload(payload);
}

bool ExtSqAdapter::processPruner(PxU32 prunerIndex, const PxQueryThreadContext* context, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) const
{
	return mUserAdapter.processPruner(prunerIndex, context, filterData, filterCall);
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class CustomPxSQ : public PxCustomSceneQuerySystem, public PxUserAllocated
	{
		public:
												CustomPxSQ(const PxCustomSceneQuerySystemAdapter& adapter, ExtPVDCapture* pvd, PxU64 contextID,
													PxSceneQueryUpdateMode::Enum mode, bool usesTreeOfPruners) :
													mExtAdapter	(adapter),
													mQueries	(pvd, contextID, EXT_PRUNER_EPSILON, mExtAdapter, usesTreeOfPruners),
													mUpdateMode	(mode),
													mRefCount	(1)
													{}
		virtual									~CustomPxSQ()	{}

		virtual	void							release();
		virtual	void							acquireReference();
		virtual	void							preallocate(PxU32 prunerIndex, PxU32 nbShapes)										{ SQ().preallocate(prunerIndex, nbShapes);						}
		virtual	void							addSQShape(	const PxRigidActor& actor, const PxShape& shape, const PxBounds3& bounds,
															const PxTransform& transform, const PxSQCompoundHandle* compoundHandle, bool hasPruningStructure);
		virtual	void							removeSQShape(const PxRigidActor& actor, const PxShape& shape);
		virtual	void							updateSQShape(const PxRigidActor& actor, const PxShape& shape, const PxTransform& transform);
		virtual	PxSQCompoundHandle				addSQCompound(const PxRigidActor& actor, const PxShape** shapes, const PxBVH& pxbvh, const PxTransform* transforms);
		virtual	void							removeSQCompound(PxSQCompoundHandle compoundHandle);
		virtual	void							updateSQCompound(PxSQCompoundHandle compoundHandle, const PxTransform& compoundTransform);
		virtual	void							flushUpdates()																		{ SQ().flushUpdates();											}
		virtual	void							flushMemory()																		{ SQ().flushMemory();											}
		virtual	void							visualize(PxU32 prunerIndex, PxRenderOutput& out)		const						{ SQ().visualize(prunerIndex, out);							}
		virtual	void							shiftOrigin(const PxVec3& shift)													{ SQ().shiftOrigin(shift);										}
		virtual	PxSQBuildStepHandle				prepareSceneQueryBuildStep(PxU32 prunerIndex);
		virtual	void							sceneQueryBuildStep(PxSQBuildStepHandle handle);
		virtual	void							finalizeUpdates();
		virtual	void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint)							{ SQ().setDynamicTreeRebuildRateHint(dynTreeRebuildRateHint);	}
		virtual	PxU32							getDynamicTreeRebuildRateHint()							const						{ return SQ().getDynamicTreeRebuildRateHint();					}
		virtual	void							forceRebuildDynamicTree(PxU32 prunerIndex)											{ SQ().forceRebuildDynamicTree(prunerIndex);					}
		virtual	PxSceneQueryUpdateMode::Enum	getUpdateMode()											const						{ return mUpdateMode;											}
		virtual	void							setUpdateMode(PxSceneQueryUpdateMode::Enum mode)									{ mUpdateMode = mode;											}
		virtual	PxU32							getStaticTimestamp()									const						{ return SQ().getStaticTimestamp();							}
		virtual	void							merge(const PxPruningStructure& pxps);
		virtual	bool							raycast(const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
														PxRaycastCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags)	const;
		virtual	bool							sweep(	const PxGeometry& geometry, const PxTransform& pose,
														const PxVec3& unitDir, const PxReal distance,
														PxSweepCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags)	const;
		virtual	bool							overlap(const PxGeometry& geometry, const PxTransform& transform,
														PxOverlapCallback& hitCall, 
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, PxGeometryQueryFlags flags)	const;
		virtual	PxSQPrunerHandle				getHandle(const PxRigidActor& actor, const PxShape& shape, PxU32& prunerIndex)	const;
		virtual	void							sync(PxU32 prunerIndex, const PxSQPrunerHandle* handles, const PxU32* indices, const PxBounds3* bounds,
													const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices);

		virtual	PxU32							addPruner(PxPruningStructureType::Enum primaryType, PxDynamicTreeSecondaryPruner::Enum secondaryType, PxU32 preallocated);
		virtual	PxU32							startCustomBuildstep();
		virtual	void							customBuildstep(PxU32 index);
		virtual	void							finishCustomBuildstep();

		PX_FORCE_INLINE	ExtPrunerManager&		SQ()			{ return mQueries.mSQManager;	}
		PX_FORCE_INLINE	const ExtPrunerManager&	SQ()	const	{ return mQueries.mSQManager;	}

				ExtSqAdapter					mExtAdapter;
				ExtSceneQueries					mQueries;
				PxSceneQueryUpdateMode::Enum	mUpdateMode;
				PxU32							mRefCount;
	};
}

///////////////////////////////////////////////////////////////////////////////

void addExternalSQ(PxSceneQuerySystem* added);
void removeExternalSQ(PxSceneQuerySystem* removed);

void CustomPxSQ::release()
{
	mRefCount--;
	if(!mRefCount)
	{
		removeExternalSQ(this);
		PX_DELETE_THIS;
	}
}

void CustomPxSQ::acquireReference()
{
	mRefCount++;
}

void CustomPxSQ::addSQShape(const PxRigidActor& actor, const PxShape& shape, const PxBounds3& bounds, const PxTransform& transform, const PxSQCompoundHandle* compoundHandle, bool hasPruningStructure)
{
	PrunerPayload payload;
	setPayload(payload, &shape, &actor);

	const bool isDynamic = isDynamicActor(actor);
	const PxU32 prunerIndex = mExtAdapter.mUserAdapter.getPrunerIndex(actor, shape);

	const PrunerCompoundId cid = compoundHandle ? PrunerCompoundId(*compoundHandle) : INVALID_COMPOUND_ID;
	const PrunerHandle shapeHandle = SQ().addPrunerShape(payload, prunerIndex, isDynamic, cid, bounds, transform, hasPruningStructure);

	const PxU32 actorIndex = actor.getInternalActorIndex();
	PX_ASSERT(actorIndex!=0xffffffff);
	mExtAdapter.mDatabase.add(actorIndex, &actor, &shape, createActorShapeData(shapeHandle, cid));
}

namespace
{
	struct DatabaseCleaner : PrunerPayloadRemovalCallback
	{
		DatabaseCleaner(ExtSqAdapter& adapter) : mAdapter(adapter){}

		virtual void	invoke(PxU32 nbRemoved, const PrunerPayload* removed)	PX_OVERRIDE PX_FINAL
		{
			PxU32 actorIndex = 0xffffffff;
			const PxRigidActor* cachedActor = NULL;

			while(nbRemoved--)
			{
				const PrunerPayload& payload = *removed++;

				const PxRigidActor* actor = getActorFromPayload(payload);

				if(actor!=cachedActor)
				{
					actorIndex = actor->getInternalActorIndex();
					cachedActor = actor;
				}
				PX_ASSERT(actorIndex!=0xffffffff);

				bool status = mAdapter.mDatabase.remove(actorIndex, actor, getShapeFromPayload(payload), NULL);
				PX_ASSERT(status);
				PX_UNUSED(status);
			}
		}
		ExtSqAdapter&	mAdapter;

		PX_NOCOPY(DatabaseCleaner)
	};
}

void CustomPxSQ::removeSQShape(const PxRigidActor& actor, const PxShape& shape)
{
	const bool isDynamic = isDynamicActor(actor);
	const PxU32 prunerIndex = mExtAdapter.mUserAdapter.getPrunerIndex(actor, shape);

	const PxU32 actorIndex = actor.getInternalActorIndex();
	PX_ASSERT(actorIndex!=0xffffffff);

	ActorShapeData actorShapeData;
	mExtAdapter.mDatabase.remove(actorIndex, &actor, &shape, &actorShapeData);

	const PrunerHandle shapeHandle = getPrunerHandle(actorShapeData);
	const PrunerCompoundId compoundId = getCompoundID(actorShapeData);

	SQ().removePrunerShape(prunerIndex, isDynamic, compoundId, shapeHandle, NULL);
}

void CustomPxSQ::updateSQShape(const PxRigidActor& actor, const PxShape& shape, const PxTransform& transform)
{
	const bool isDynamic = isDynamicActor(actor);
	const PxU32 prunerIndex = mExtAdapter.mUserAdapter.getPrunerIndex(actor, shape);

	const PxU32 actorIndex = actor.getInternalActorIndex();
	PX_ASSERT(actorIndex!=0xffffffff);

	const ActorShapeData actorShapeData = mExtAdapter.mDatabase.find(actorIndex, &actor, &shape);

	const PrunerHandle shapeHandle = getPrunerHandle(actorShapeData);
	const PrunerCompoundId cid = getCompoundID(actorShapeData);

	SQ().markForUpdate(prunerIndex, isDynamic, cid, shapeHandle, transform);
}

PxSQCompoundHandle CustomPxSQ::addSQCompound(const PxRigidActor& actor, const PxShape** shapes, const PxBVH& bvh, const PxTransform* transforms)
{
	const PxU32 numSqShapes = bvh.getNbBounds();

	PX_ALLOCA(payloads, PrunerPayload, numSqShapes);
	for(PxU32 i=0; i<numSqShapes; i++)
		setPayload(payloads[i], shapes[i], &actor);

	const PxU32 actorIndex = actor.getInternalActorIndex();
	PX_ASSERT(actorIndex!=0xffffffff);

	PX_ALLOCA(shapeHandles, PrunerHandle, numSqShapes);
	SQ().addCompoundShape(bvh, actorIndex, actor.getGlobalPose(), shapeHandles, payloads, transforms, isDynamicActor(actor));

	for(PxU32 i=0; i<numSqShapes; i++)
	{
		// PT: TODO: actorIndex is now redundant!
		mExtAdapter.mDatabase.add(actorIndex, &actor, shapes[i], createActorShapeData(shapeHandles[i], actorIndex));
	}

	return PxSQCompoundHandle(actorIndex);
}

void CustomPxSQ::removeSQCompound(PxSQCompoundHandle compoundHandle)
{
	DatabaseCleaner cleaner(mExtAdapter);
	SQ().removeCompoundActor(PrunerCompoundId(compoundHandle), &cleaner);
}

void CustomPxSQ::updateSQCompound(PxSQCompoundHandle compoundHandle, const PxTransform& compoundTransform)
{
	SQ().updateCompoundActor(PrunerCompoundId(compoundHandle), compoundTransform);
}

PxSQBuildStepHandle CustomPxSQ::prepareSceneQueryBuildStep(PxU32 prunerIndex)
{
	return SQ().prepareSceneQueriesUpdate(prunerIndex);
}

void CustomPxSQ::sceneQueryBuildStep(PxSQBuildStepHandle handle)
{
	SQ().sceneQueryBuildStep(handle);
}

void CustomPxSQ::finalizeUpdates()
{
	switch(mUpdateMode)
	{
		case PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED:		SQ().afterSync(true, true);	break;
		case PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED:	SQ().afterSync(true, false);	break;
		case PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED:	SQ().afterSync(false, false);	break;
	}
}

void CustomPxSQ::merge(const PxPruningStructure& /*pxps*/)
{
	PX_ASSERT(!"Not supported by this custom SQ system");

	// PT: PxPruningStructure only knows about the regular static/dynamic pruners, so it is not
	// compatible with this custom version.
}

bool CustomPxSQ::raycast(	const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
						PxRaycastCallback& hitCall, PxHitFlags hitFlags,
						const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
						const PxQueryCache* cache, PxGeometryQueryFlags flags) const
{
	return mQueries._raycast(origin, unitDir, distance, hitCall, hitFlags, filterData, filterCall, cache, flags);
}

bool CustomPxSQ::sweep(	const PxGeometry& geometry, const PxTransform& pose,
						const PxVec3& unitDir, const PxReal distance,
						PxSweepCallback& hitCall, PxHitFlags hitFlags,
						const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
						const PxQueryCache* cache, const PxReal inflation, PxGeometryQueryFlags flags) const
{
	return mQueries._sweep(geometry, pose, unitDir, distance, hitCall, hitFlags, filterData, filterCall, cache, inflation, flags);
}

bool CustomPxSQ::overlap(	const PxGeometry& geometry, const PxTransform& transform,
						PxOverlapCallback& hitCall, 
						const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
						const PxQueryCache* cache, PxGeometryQueryFlags flags) const
{
	return mQueries._overlap( geometry, transform, hitCall, filterData, filterCall, cache, flags);
}

PxSQPrunerHandle CustomPxSQ::getHandle(const PxRigidActor& actor, const PxShape& shape, PxU32& prunerIndex) const
{
	const PxU32 actorIndex = actor.getInternalActorIndex();
	PX_ASSERT(actorIndex!=0xffffffff);

	const ActorShapeData actorShapeData = mExtAdapter.mDatabase.find(actorIndex, &actor, &shape);

	prunerIndex = mExtAdapter.mUserAdapter.getPrunerIndex(actor, shape);

	return PxSQPrunerHandle(getPrunerHandle(actorShapeData));
}

void CustomPxSQ::sync(PxU32 prunerIndex, const PxSQPrunerHandle* handles, const PxU32* indices, const PxBounds3* bounds,
					const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)
{
	SQ().sync(prunerIndex, handles, indices, bounds, transforms, count, ignoredIndices);
}

PxU32 CustomPxSQ::addPruner(PxPruningStructureType::Enum primaryType, PxDynamicTreeSecondaryPruner::Enum secondaryType, PxU32 preallocated)
{
	Pruner* pruner = create(primaryType, mQueries.getContextId(), secondaryType, PxBVHBuildStrategy::eFAST, 4);
	return mQueries.mSQManager.addPruner(pruner, preallocated);
}

PxU32 CustomPxSQ::startCustomBuildstep()
{
	return SQ().startCustomBuildstep();
}

void CustomPxSQ::customBuildstep(PxU32 index)
{
	SQ().customBuildstep(index);
}

void CustomPxSQ::finishCustomBuildstep()
{
	SQ().finishCustomBuildstep();
}

///////////////////////////////////////////////////////////////////////////////

PxCustomSceneQuerySystem* physx::PxCreateCustomSceneQuerySystem(PxSceneQueryUpdateMode::Enum sceneQueryUpdateMode, PxU64 contextID, const PxCustomSceneQuerySystemAdapter& adapter, bool usesTreeOfPruners)
{
	ExtPVDCapture* pvd = NULL;
	CustomPxSQ* pxsq = PX_NEW(CustomPxSQ)(adapter, pvd, contextID, sceneQueryUpdateMode, usesTreeOfPruners);

	addExternalSQ(pxsq);

	return pxsq;
}

///////////////////////////////////////////////////////////////////////////////
