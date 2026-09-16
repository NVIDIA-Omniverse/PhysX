// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-28
 */
/**
 * @implements REQ-PARSE-BODY-001
 * @covers AC-6
 */
/**
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-5
 */
/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-10
 */
/**
 * @implements REQ-PARSE-FEED-003
 * @covers AC-12
 */

#include <cstring>
#include <vector>

#include <PxPhysicsAPI.h>
#include <PhysXTools.h>
#include <PhysXScene.h>
#include <OmniPhysX.h>
#include <Setup.h>

#include "InternalActor.h"
#include "InternalScene.h"

#include <omni/physics/parse/KnownTokens.h>

#include <usdLoad/LoadUsd.h>

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;

InternalActor::InternalActor(PhysXScene* ps,
                             bool dynamicActor,
                             const ObjectInstance* instance,
                             bool localSpaceVelocities,
                             omni::physics::parse::ObjectKey key)
    : mKey(key),
      mActor(nullptr),
      mID(-1),
      mFlags(0),
      mSurfaceVelocityLocalSpace(true),
      mSurfaceVelocityEnabled(false),
      mSplinesSurfaceVelocityEnabled(false),
      mSplinesCurve(nullptr),
      mSolveContactEnabled(true),
      mPhysXScene(ps),
      mMirrorSharedCollection(nullptr),
      mMirrorMemsize(0),
      mMirrorMemory(nullptr)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (instance)
    {
        // Instanced actors are only created on the (single-threaded) non-replicator
        // path, so minting the instancer key here is safe.
        AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        // A missing AttachedStage here is a fatal setup error, not a case to
        // silently fall back to direct USD for.
        CARB_ASSERT(as);
        if (as)
            mInstanceKey = instance->instancerKey;
        mInstanceIndex = instance->index;

        if (instance->hasProtoTransformInverse)
        {
            mProtoTransformInverse = toPxMat44d(instance->protoTransformInverse);
        }
        else
        {
            const omni::physics::parse::ObjectKey protoKey = instance->protoKey;
            if (as && as->getSource() && as->getSource()->exists(protoKey))
            {
                // World transforms via the source only (per-call at Default; the engine
                // xform cache will be reintroduced as a source-side cache later).
                const PxMat44d protoWorld =
                    getWorldTransform(*as, protoKey, omni::physics::parse::ReadTime::defaultTime());
                const PxMat44d instancerWorld = getWorldTransform(
                    *as, instance->instancerKey, omni::physics::parse::ReadTime::defaultTime());
                // Bit-exact Gf::RemoveScaleShear() (gfmath::removeScaleShearGf), not
                // the PhysX-native omni::physx::removeScaleShear: protoWorld and
                // instancerWorld are world transforms of a PointInstancer prototype
                // and its instancer, so non-uniform scale meeting rotation anywhere
                // in either ancestor chain produces sheared input, and the two
                // decompositions disagree on that input (see MatrixTools.h). This
                // keeps every instance's placement numerically unchanged from the
                // pre-port behaviour.
                const PxMat44d proto = gfmath::removeScaleShearGf(protoWorld);
                const PxMat44d instancer = gfmath::removeScaleShearGf(instancerWorld);
                // Was, in Gf (row-vector) order: (proto * instancer^-1)^-1.
                // Gf A * B is PhysX B * A on the same sixteen doubles, so
                // proto * instancer^-1  ->  instancer^-1 * proto, and inverting
                // that gives proto^-1 * instancer.
                mProtoTransformInverse = affineInverse(proto) * instancer;
            }
            else
            {
                mProtoTransformInverse = PxMat44d(PxIdentity);
            }
        }
        // Store the instancer's authored initial transforms, so a later reset can
        // restore them. This is authored input for the write-back
        // (InternalPhysXDatabase::resetStartProperties).
        if (as && as->getSource() && as->getSource()->isA(mInstanceKey, as->getKnownTokens().pointInstancerType))
        {
            const omni::physics::parse::KnownTokens& tok = as->getKnownTokens();
            TransformsInstanceMap::const_iterator fit = db.mInitialPointInstancerTransforms.find(mInstanceKey);
            if (fit == db.mInitialPointInstancerTransforms.end())
            {
                db.mInitialTransformsStored = true;
                InitialInstancerData& initialData = db.mInitialPointInstancerTransforms[mInstanceKey];

                // Promote a purely time-sampled attribute's earliest sample to Default
                // first: the read below resolves at Default, matching the old
                // attr.Get()-then-EarliestTime()-fallback behavior without needing a
                // dedicated earliest-sample read mode.
                omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite();
                if (dw)
                {
                    dw->promoteEarliestSampleToDefault(mInstanceKey, tok.positions);
                    dw->promoteEarliestSampleToDefault(mInstanceKey, tok.orientations);
                    dw->promoteEarliestSampleToDefault(mInstanceKey, tok.scales);
                    dw->promoteEarliestSampleToDefault(mInstanceKey, tok.velocities);
                    dw->promoteEarliestSampleToDefault(mInstanceKey, tok.angularVelocities);
                }

                // Quaternion arrays already resolve xyzw (PxQuat order) through
                // getArrayValue's fillArray decode ladder, whatever the backing type.
                getArrayValue(*as, mInstanceKey, tok.positions,
                              omni::physics::parse::ReadTime::defaultTime(), initialData.positions);
                getArrayValue(*as, mInstanceKey, tok.orientations,
                              omni::physics::parse::ReadTime::defaultTime(), initialData.orientations);
                getArrayValue(*as, mInstanceKey, tok.scales,
                              omni::physics::parse::ReadTime::defaultTime(), initialData.scales);
                getArrayValue(*as, mInstanceKey, tok.velocities,
                              omni::physics::parse::ReadTime::defaultTime(), initialData.velocities);
                getArrayValue(*as, mInstanceKey, tok.angularVelocities,
                              omni::physics::parse::ReadTime::defaultTime(), initialData.angularVelocities);
            }
        }
    }
    else
    {
        mInstanceIndex = kInvalidUint32_t;

        // Nested-rigid-body detection: is `key` a descendant of another enabled rigid
        // body, and if so, is it actually composed under that ancestor's transform (no
        // resetXformStack break in between)? Walks ancestors via IPhysicsSource::getParent.
        AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        if (as)
        {
            if (const omni::physics::parse::IPhysicsSource* src = as->getSource())
            {
                // getKnownTokens() is interned once per attach on the serial setup thread, so
                // it is safe to read from a replicator worker thread. A fresh
                // KnownTokens::intern(*src) here would instead mutate the source's shared token
                // tables with no lock, racing sibling workers.
                const omni::physics::parse::KnownTokens& tok = as->getKnownTokens();

                // The ancestor chain first (getParent is a path operation), then ONE batched
                // existence query for the whole chain instead of one round trip per level; both
                // walks below stop at the first ancestor that is not live, as before.
                std::vector<omni::physics::parse::ObjectKey> chain;
                for (omni::physics::parse::ObjectKey k = src->getParent(key); k.valid(); k = src->getParent(k))
                    chain.push_back(k);
                std::vector<bool> live;
                src->existsBatch(chain, live);
                size_t liveDepth = 0;
                while (liveDepth < chain.size() && live[liveDepth])
                    ++liveDepth;

                bool bodyParentFound = false;
                size_t bodyParentDepth = 0;
                for (size_t depth = 0; depth < liveDepth; ++depth)
                {
                    const omni::physics::parse::ObjectKey bodyParentKey = chain[depth];
                    if (src->hasSchema(bodyParentKey, tok.physicsRigidBodyAPI))
                    {
                        // UsdPhysicsRigidBodyAPI's rigidBodyEnabled schema default is true;
                        // getAttribute leaves a preseeded out-param untouched on a miss (see
                        // IPhysicsSource.h's "read with default" idiom), matching a real
                        // UsdAttribute::Get() resolving the schema fallback when unauthored.
                        bool bodyEnabled = true;
                        src->getAttribute(bodyParentKey, tok.physicsRigidBodyEnabled, bodyEnabled);
                        if (bodyEnabled)
                        {
                            bodyParentFound = true;
                            bodyParentDepth = depth;
                            break;
                        }
                    }
                }

                if (bodyParentFound)
                {
                    bool hasResetXformStack = false;
                    for (size_t depth = 0; depth < bodyParentDepth; ++depth)
                    {
                        bool resetsXformStack = false;
                        getLocalTransform(*as, chain[depth], omni::physics::parse::ReadTime::defaultTime(), resetsXformStack);
                        if (resetsXformStack)
                        {
                            hasResetXformStack = true;
                            break;
                        }
                    }
                    if (!hasResetXformStack)
                    {
                        db.setNestedBodiesUsed(true);
                    }
                }
            }
        }

        // store initial conditions for reset, and prepare the attributes for write, non instanced, instanced do have own buffers.
        // (The sink resolves the parent frame per-frame, so no parent caching is needed here.)
        if (dynamicActor)
        {
            initializeDynamicActor();
        }
    }

    if (localSpaceVelocities)
    {
        mFlags |= InternalActorFlag::eLOCALSPACE_VELOCITIES;
    }
}

InternalActor::~InternalActor() = default;

void InternalActor::switchFromKinematic()
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    ActorInitialDataMap::iterator initFit = db.mInitialActorDataMap.find(mKey);
    if (initFit == db.mInitialActorDataMap.end())
    {
        initializeDynamicActor(true);
    }
}

void InternalActor::initializeDynamicActor(bool runtimeInitialization)
{
    const bool updateUSD = OmniPhysX::getInstance().getCachedSettings().updateToUsd &&
        !(SimulationCallbacks::getSimulationCallbacks()->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE));

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    AttachedStage* attachedStage = updateUSD ? UsdLoad::getUsdLoad()->getActiveAttachedStage() : nullptr;
    omni::physics::parse::IPhysicsDataWrite* dw = attachedStage ? attachedStage->getDataWrite() : nullptr;

    if (updateUSD && dw)
    {
        ActorInitialDataMap::iterator initFit = db.mInitialActorDataMap.find(mKey);
        if (initFit == db.mInitialActorDataMap.end())
        {
            db.mInitialTransformsStored = true;
            ActorInitialData& initialData = db.mInitialActorDataMap[mKey];

            dw->storeXformOpReset(mKey);

            const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource();
            if (source)
            {
                // Use the attach's already-interned token cache, not a fresh
                // KnownTokens::intern(): this can run on a replicator worker thread.
                const omni::physics::parse::KnownTokens& tok = attachedStage->getKnownTokens();

                source->getAttribute(mKey, tok.physicsVelocity, initialData.velocity);
                source->getAttribute(mKey, tok.physicsAngularVelocity, initialData.angularVelocity);
            }
            initialData.velocityWritten = false;
            initialData.angularVelocityWritten = false;
        }
    }

    // Non-instancer dynamic bodies (including nested ones) route transform
    // write-back through IPhysicsDataWrite. prepareTransformWrite either keeps
    // the stack in XformCommonAPI form (when that mode is enabled) or normalizes
    // it to scale/orient/translate; either way the sink owns the world->local
    // conversion + residual extra-transform, so nothing is authored inline here.
    // The sink authors a batch ancestor-first, so nested bodies see fresh parents.
    if (updateUSD && attachedStage)
    {
        omni::physics::parse::IPhysicsDataWrite* dw = attachedStage->getDataWrite();

        bool prepared = false;
        if (dw)
        {
            dw->setUpdateToUsdUsingXformCommonAPI(
                OmniPhysX::getInstance().getCachedSettings().updateToUsdUsingXformCommonAPI);

            // The one-time load-time xform-op normalization authors new xformOp
            // attributes and rewrites xformOpOrder at whatever destination
            // prepareTransformWrite is given. When a simulation output layer is
            // active these edits must land on it -- exactly as the per-frame
            // transform writes do (InternalScene::updateRenderTransforms) -- so
            // they live in the transient simulation layer and vanish on Stop.
            // Without this scoping the user's authoring layer is permanently
            // rewritten for any body with a non-canonical xform stack (issue #8).
            // rawLayer() is a non-owning peek: OmniPhysX's own reference keeps the
            // layer alive for this call, and a USD sink's prepareTransformWrite takes
            // its own ref-counting reference for its edit context. A backend with no
            // destination-override concept ignores it.
            const SimulationLayerHandle simLayer = OmniPhysX::getInstance().getSimulationLayer();
            dw->prepareTransformWrite(&mKey, 1, &prepared, simLayer.rawLayer());
        }

        if (prepared)
            mFlags |= InternalActorFlag::eUSE_DATAWRITE_SINK;
        else
            mFlags |= InternalActorFlag::eSKIP_UPDATE_TRANSFORM;
    }
}

void InternalActor::enableSurfaceVelocity(bool enable, ::physx::PxRigidActor& actor)
{
    if (enable && !mSurfaceVelocityEnabled)
    {
        if (actor.getScene())
            actor.getScene()->resetFiltering(actor);
        PxShape* shapePtr = nullptr;
        for (uint32_t i = 0; i < actor.getNbShapes(); i++)
        {
            actor.getShapes(&shapePtr, 1, i);
            PxFilterData fd = shapePtr->getSimulationFilterData();
            fd.word3 |= CONTACT_MODIFY_SURFACE_VELOCITY;
            shapePtr->setSimulationFilterData(fd);
        }
        mSurfaceVelocityEnabled = enable;
    }
    else if (!enable && mSurfaceVelocityEnabled)
    {
        if(actor.getScene())
            actor.getScene()->resetFiltering(actor);
        PxShape* shapePtr = nullptr;
        for (uint32_t i = 0; i < actor.getNbShapes(); i++)
        {
            actor.getShapes(&shapePtr, 1, i);
            PxFilterData fd = shapePtr->getSimulationFilterData();
            fd.word3 &= ~CONTACT_MODIFY_SURFACE_VELOCITY;
            shapePtr->setSimulationFilterData(fd);
        }
        mSurfaceVelocityEnabled = enable;
    }
}

void InternalActor::copySurfaceVelocityState(const InternalActor& source,
                                             ::physx::PxRigidActor& cloneActor,
                                             const ::physx::PxTransform& clonePivotPose)
{
    (void)cloneActor; // retained in the signature for symmetry with peer copy helpers
    mSurfaceVelocity = source.mSurfaceVelocity;
    mSurfaceVelocityAuthored = source.mSurfaceVelocityAuthored;
    mSurfaceAngularVelocity = source.mSurfaceAngularVelocity;
    mSurfaceVelocityLocalSpace = source.mSurfaceVelocityLocalSpace;
    mSurfaceAngularVelocityPivot = clonePivotPose;
    mSurfaceVelocityEnabled = source.mSurfaceVelocityEnabled;
}

void InternalActor::enableSplineSurfaceVelocity(bool enable,
                                                ::physx::PxRigidActor& actor,
                                                const AttachedStage& attachedStage,
                                                omni::physics::parse::ObjectKey splinesCurveKey)
{
    if (enable && !mSplinesSurfaceVelocityEnabled)
    {
        if (actor.getScene())
            actor.getScene()->resetFiltering(actor);
        PxShape* shapePtr = nullptr;
        for (uint32_t i = 0; i < actor.getNbShapes(); i++)
        {
            actor.getShapes(&shapePtr, 1, i);
            PxFilterData fd = shapePtr->getSimulationFilterData();
            fd.word3 |= CONTACT_MODIFY_SURFACE_VELOCITY;
            shapePtr->setSimulationFilterData(fd);
        }

        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        bool added = false;
        mSplinesCurve = db.addSplinesCurve(attachedStage, splinesCurveKey, added);
        if (mSplinesCurve)
        {
            mSplinesSurfaceVelocityEnabled = enable;
            if (added)
            {
                // compute the relative pose
                const PxTransform actorGlobalPose = actor.getGlobalPose();
                const PxTransform splineGlobalPose = toTransform(getWorldTransform(
                    attachedStage, splinesCurveKey, omni::physics::parse::ReadTime::defaultTime()));
                mSplineLocalSpace = actorGlobalPose.getInverse() * splineGlobalPose;
            }
        }
    }
    else if (!enable && mSplinesSurfaceVelocityEnabled)
    {
        if (actor.getScene())
            actor.getScene()->resetFiltering(actor);
        PxShape* shapePtr = nullptr;
        for (uint32_t i = 0; i < actor.getNbShapes(); i++)
        {
            actor.getShapes(&shapePtr, 1, i);
            PxFilterData fd = shapePtr->getSimulationFilterData();
            fd.word3 &= ~CONTACT_MODIFY_SURFACE_VELOCITY;
            shapePtr->setSimulationFilterData(fd);
        }
        mSplinesCurve = nullptr;
        mSplinesSurfaceVelocityEnabled = enable;
    }
}

void InternalActor::enableContactSolve(bool enable, ::physx::PxRigidActor* actor)
{
    if (enable && !mSolveContactEnabled)
    {
        if (actor->getScene())
            actor->getScene()->resetFiltering(*actor);
        PxShape* shapePtr = nullptr;
        for (uint32_t i = 0; i < actor->getNbShapes(); i++)
        {
            actor->getShapes(&shapePtr, 1, i);
            PxFilterData fd = shapePtr->getSimulationFilterData();
            fd.word3 &= ~CONTACT_SOLVE_DISABLE;
            shapePtr->setSimulationFilterData(fd);
        }
        mSolveContactEnabled = enable;
    }
    else if (!enable && mSolveContactEnabled)
    {
        if (actor->getScene())
            actor->getScene()->resetFiltering(*actor);
        PxShape* shapePtr = nullptr;
        for (uint32_t i = 0; i < actor->getNbShapes(); i++)
        {
            actor->getShapes(&shapePtr, 1, i);
            PxFilterData fd = shapePtr->getSimulationFilterData();
            fd.word3 |= CONTACT_SOLVE_DISABLE;
            shapePtr->setSimulationFilterData(fd);
        }
        mSolveContactEnabled = enable;
    }
}
