// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>
#include <PhysXTools.h>
#include <PhysXScene.h>
#include <OmniPhysX.h>
#include <Setup.h>

#include "InternalTools.h"
#include "InternalActor.h"
#include "InternalScene.h"

#include <usdLoad/LoadUsd.h>
#include <UsdPhysicsDataWrite.h>

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace PXR_NS;
using namespace carb;
using namespace ::physx;

static GfMatrix4d toGfMatrix4d(const omni::physics::parse::Matrix4d& matrix)
{
    return GfMatrix4d(matrix.data[0], matrix.data[1], matrix.data[2], matrix.data[3],
                      matrix.data[4], matrix.data[5], matrix.data[6], matrix.data[7],
                      matrix.data[8], matrix.data[9], matrix.data[10], matrix.data[11],
                      matrix.data[12], matrix.data[13], matrix.data[14], matrix.data[15]);
}

InternalActor::InternalActor(PhysXScene* ps,
                             const PXR_NS::SdfPath& primKey,
                             const UsdPrim& prim,
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
        const UsdPrim instancerPrim = prim.GetStage()->GetPrimAtPath(instance->instancerPath);
        AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        // A missing AttachedStage here is a fatal setup error, not a case to
        // silently fall back to direct USD for.
        CARB_ASSERT(as);
        if (as)
            mInstanceKey = as->keyFor(instance->instancerPath);
        mInstanceIndex = instance->index;

        if (instance->hasProtoTransformInverse)
        {
            mProtoTransformInverse = toGfMatrix4d(instance->protoTransformInverse);
        }
        else
        {
            const omni::physics::parse::ObjectKey protoKey = as ? as->keyFor(instance->protoPath) : omni::physics::parse::ObjectKey{};
            if (as && as->getSource() && as->getSource()->exists(protoKey))
            {
                // World transforms via the source only (per-call at Default; the engine
                // xform cache will be reintroduced as a source-side cache later).
                const GfMatrix4d protoWorld = getWorldTransform(*as, protoKey, UsdTimeCode::Default());
                const GfMatrix4d instancerWorld = getWorldTransform(*as, as->keyFor(instance->instancerPath), UsdTimeCode::Default());
                mProtoTransformInverse = protoWorld.RemoveScaleShear();
                GfMatrix4d instancerMatrix = instancerWorld.RemoveScaleShear();
                mProtoTransformInverse = (mProtoTransformInverse * instancerMatrix.GetInverse()).GetInverse();
            }
            else
            {
                mProtoTransformInverse = GfMatrix4d(1.0);
            }
        }
        // store initial data
        if (instancerPrim.IsA<UsdGeomPointInstancer>())
        {
            UsdGeomPointInstancer instancer(instancerPrim);
            TransformsInstanceMap::const_iterator fit = db.mInitialPointInstancerTransforms.find(mInstanceKey);
            if (fit == db.mInitialPointInstancerTransforms.end())
            {
                db.mInitialTransformsStored = true;
                InitialInstancerData& initialData = db.mInitialPointInstancerTransforms[mInstanceKey];
                if (!instancer.GetPositionsAttr().Get(&initialData.positions))
                    instancer.GetPositionsAttr().Get(&initialData.positions, UsdTimeCode::EarliestTime());
                if (!instancer.GetOrientationsAttr().Get(&initialData.orientations))
                    instancer.GetOrientationsAttr().Get(&initialData.orientations, UsdTimeCode::EarliestTime());

                if (!instancer.GetScalesAttr().Get(&initialData.scales))
                    instancer.GetScalesAttr().Get(&initialData.scales, UsdTimeCode::EarliestTime());

                if (!instancer.GetVelocitiesAttr().Get(&initialData.velocities))
                    instancer.GetVelocitiesAttr().Get(&initialData.velocities, UsdTimeCode::EarliestTime());
                if (!instancer.GetAngularVelocitiesAttr().Get(&initialData.angularVelocities))
                    instancer.GetAngularVelocitiesAttr().Get(&initialData.angularVelocities, UsdTimeCode::EarliestTime());
            }
        }
    }
    else
    {
        mInstanceIndex = kInvalidUint32_t;

        // check if the body is nested
        if (prim)
        {
            bool bodyParentFound = false;
            UsdPrim bodyParent = prim.GetParent();
            while (bodyParent != prim.GetStage()->GetPseudoRoot())
            {
                const UsdPhysicsRigidBodyAPI rboAPI(bodyParent);
                if (rboAPI)
                {
                    bool bodyEnabled = false;
                    rboAPI.GetRigidBodyEnabledAttr().Get(&bodyEnabled);
                    if (bodyEnabled)
                    {
                        bodyParentFound = true;
                        break;
                    }
                }
                bodyParent = bodyParent.GetParent();
            }


            if (bodyParentFound)
            {
                bool hasResetXformStack = false;
                UsdPrim parent = prim;
                while (parent != prim.GetStage()->GetPseudoRoot() && parent != bodyParent)
                {
                    UsdGeomXformable xformable(parent);
                    if (xformable && xformable.GetResetXformStack())
                    {
                        hasResetXformStack = true;
                        break;
                    }
                    parent = parent.GetParent();
                }
                if (!hasResetXformStack)
                {
                    db.setNestedBodiesUsed(true);
                }
            }
        }

        const uint32_t simulationFlags = SimulationCallbacks::getSimulationCallbacks()->getSimulationFlags(primKey);
        if (simulationFlags & GlobalSimulationFlag::eNOTIFY_UPDATE)
        {
            mFlags |= InternalActorFlag::eNOTIFY_TRANSFORM;
        }
        if (simulationFlags & GlobalSimulationFlag::eSKIP_WRITE)
        {
            mFlags |= InternalActorFlag::eSKIP_UPDATE_TRANSFORM;
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
    omni::physics::usd::UsdPhysicsDataWrite* usdDataWrite =
        attachedStage ? omni::physics::usd::asUsdDataWrite(attachedStage->getDataWrite()) : nullptr;

    if (updateUSD && usdDataWrite)
    {
        ActorInitialDataMap::iterator initFit = db.mInitialActorDataMap.find(mKey);
        if (initFit == db.mInitialActorDataMap.end())
        {
            db.mInitialTransformsStored = true;
            ActorInitialData& initialData = db.mInitialActorDataMap[mKey];

            UsdPrim prim = usdDataWrite->usdPrimForWrite(mKey);
            if (prim)
            {
                UsdGeomXformable xformable(prim);
                if (xformable)
                    initialData.xformOpStorage.store(xformable);
            }

            const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource();
            if (source)
            {
                carb::Float3 velocity;
                if (source->getAttribute(mKey, source->internToken(UsdPhysicsTokens->physicsVelocity.GetString()),
                                         velocity))
                {
                    initialData.velocity = PXR_NS::GfVec3f(velocity.x, velocity.y, velocity.z);
                }
                carb::Float3 angularVelocity;
                if (source->getAttribute(mKey, source->internToken(UsdPhysicsTokens->physicsAngularVelocity.GetString()),
                                         angularVelocity))
                {
                    initialData.angularVelocity =
                        PXR_NS::GfVec3f(angularVelocity.x, angularVelocity.y, angularVelocity.z);
                }
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
            if (usdDataWrite)
            {
                usdDataWrite->setUpdateToUsdUsingXformCommonAPI(
                    OmniPhysX::getInstance().getCachedSettings().updateToUsdUsingXformCommonAPI);
            }

            dw->prepareTransformWrite(&mKey, 1, &prepared);
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
                const GfMatrix4d splineGlobalPosePxr =
                    getWorldTransform(attachedStage, splinesCurveKey, UsdTimeCode::Default());
                const PxTransform splineGlobalPose = toPhysX(splineGlobalPosePxr);
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
