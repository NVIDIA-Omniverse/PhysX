// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;

InternalActor::InternalActor(PhysXScene* ps,
                             const pxr::SdfPath& primPath,
                             const UsdPrim& prim,
                             bool dynamicActor,
                             const ObjectInstance* instance,
                             bool localSpaceVelocities)
    : mPrim(prim),
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
        mInstancePrim = prim.GetStage()->GetPrimAtPath(instance->instancerPath);
        mInstanceIndex = instance->index;

        const UsdPrim protoPrim = prim.GetStage()->GetPrimAtPath(instance->protoPath);
        if (protoPrim)
        {
            mProtoTransformInverse = db.mXformCache.GetLocalToWorldTransform(protoPrim).RemoveScaleShear();
            GfMatrix4d instancerMatrix = db.mXformCache.GetLocalToWorldTransform(mInstancePrim).RemoveScaleShear();
            mProtoTransformInverse = (mProtoTransformInverse * instancerMatrix.GetInverse()).GetInverse();
        }
        else
        {
            mProtoTransformInverse = GfMatrix4d(1.0);
        }
        // store initial data
        if (mInstancePrim.IsA<UsdGeomPointInstancer>())
        {
            UsdGeomPointInstancer instancer(mInstancePrim);
            TransformsInstanceMap::const_iterator fit = db.mInitialPointInstancerTransforms.find(instance->instancerPath);
            if (fit == db.mInitialPointInstancerTransforms.end())
            {
                db.mInitialTransformsStored = true;
                InitialInstancerData& initialData = db.mInitialPointInstancerTransforms[instance->instancerPath];                
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

        // store initial conditions for reset, and prepare the attributes for write, non instanced, instanced do have own buffers
        if (dynamicActor)
        {
            initializeDynamicActor();
        }

        const uint32_t simulationFlags = SimulationCallbacks::getSimulationCallbacks()->getSimulationFlags(primPath);
        if (simulationFlags & GlobalSimulationFlag::eNOTIFY_UPDATE)
        {
            mFlags |= InternalActorFlag::eNOTIFY_TRANSFORM;
        }
        if (simulationFlags & GlobalSimulationFlag::eSKIP_WRITE)
        {
            mFlags |= InternalActorFlag::eSKIP_UPDATE_TRANSFORM;
        }

        if (prim && getParentXform(prim, mParentXformPrim))
        {
            mFlags |= InternalActorFlag::eHAS_PARENT_XFORM;
            mFlags |= InternalActorFlag::ePARENT_XFORM_DIRTY;
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
    ActorInitialDataMap::iterator initFit = db.mInitialActorDataMap.find(mPrim.GetPrimPath());
    if (initFit == db.mInitialActorDataMap.end())
    {
        initializeDynamicActor();
    }
}

void InternalActor::initializeDynamicActor()
{
    const bool useFabric = OmniPhysX::getInstance().getCachedSettings().fabricEnabled;
    const bool updateUSD = OmniPhysX::getInstance().getCachedSettings().updateToUsd &&
        !(SimulationCallbacks::getSimulationCallbacks()->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE))
        && !useFabric;

    if (updateUSD && mPrim)
    {
        static TfToken gTranslate("xformOp:translate");
        static TfToken gOrient("xformOp:orient");
        mXformOpTranslatePath = mPrim.GetPrimPath().AppendProperty(gTranslate);
        mXformOpOrientPath = mPrim.GetPrimPath().AppendProperty(gOrient);

        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        ActorInitialDataMap::iterator initFit = db.mInitialActorDataMap.find(mPrim.GetPrimPath());
        if (initFit == db.mInitialActorDataMap.end())
        {
            db.mInitialTransformsStored = true;
            ActorInitialData& initialData = db.mInitialActorDataMap[mPrim.GetPrimPath()];

            initialData.xformOpStorage.store(pxr::UsdGeomXformable(mPrim));

            // check if attributes are written
            initialData.velocityWritten = false;
            initialData.angularVelocityWritten = false;

            const SdfLayerHandle layer = mPrim.GetStage()->GetEditTarget().GetLayer();

            pxr::UsdAttribute velAttr = mPrim.GetAttribute(UsdPhysicsTokens->physicsVelocity);
            if (velAttr)
            {
                velAttr.Get(&initialData.velocity);
                if (layer->GetAttributeAtPath(velAttr.GetPath()) != nullptr)
                {
                    initialData.velocityWritten = true;
                }
            }
            pxr::UsdAttribute angVelAttr = mPrim.GetAttribute(UsdPhysicsTokens->physicsAngularVelocity);
            if (angVelAttr)
            {
                angVelAttr.Get(&initialData.angularVelocity);
                if (layer->GetAttributeAtPath(angVelAttr.GetPath()) != nullptr)
                {
                    initialData.angularVelocityWritten = true;
                }
            }
        }
    }

    const bool updateUSDVelocities = OmniPhysX::getInstance().getCachedSettings().updateVelocitiesToUsd;
    if (updateUSDVelocities && mPrim)
    {
        mVelocityPath = mPrim.GetPrimPath().AppendProperty(pxr::UsdPhysicsTokens.Get()->physicsVelocity);
        mAngularVelocityPath = mPrim.GetPrimPath().AppendProperty(pxr::UsdPhysicsTokens.Get()->physicsAngularVelocity);

        UsdPhysicsRigidBodyAPI bodyApi = UsdPhysicsRigidBodyAPI::Get(mPrim.GetStage(), mPrim.GetPrimPath());
        if (!bodyApi.GetVelocityAttr() || !bodyApi.GetVelocityAttr().HasValue())
        {
            bodyApi.CreateVelocityAttr().Set(GfVec3f(0.0f));
        }
        if (!bodyApi.GetAngularVelocityAttr() || !bodyApi.GetAngularVelocityAttr().HasValue())
        {
            bodyApi.CreateAngularVelocityAttr().Set(GfVec3f(0.0f));
        }
    }

    if (updateUSD && mPrim)
    {
        bool preTransform = false;
        bool postTransform = false;        

        if (OmniPhysX::getInstance().getSimulationLayer())
        {
            pxr::UsdEditContext editContext(mPrim.GetStage(), UsdEditTarget(OmniPhysX::getInstance().getSimulationLayer()));
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(mPrim, &mExtraTransfInv, &preTransform, &mExtraTransfInv, &postTransform))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a rigid body: %s, RigidBody wont get transformation updates.",
                    mPrim.GetPrimPath().GetText());
                mFlags |= InternalActorFlag::eSKIP_UPDATE_TRANSFORM;
            }
        }
        else
        {
            // if we fail to setup transformations skip the prim for update
            if (!setupTransformOpsAsScaleOrientTranslate(mPrim, &mExtraTransfInv, &preTransform, &mExtraTransfInv, &postTransform))
            {
                CARB_LOG_WARN("Failed to create transformation stack (position, quat, scale) on a rigid body: %s", mPrim.GetPrimPath().GetText());
                mFlags |= InternalActorFlag::eSKIP_UPDATE_TRANSFORM;
            }
        }

        if (!(mFlags & InternalActorFlag::eSKIP_UPDATE_TRANSFORM))
        {
            SdfPrimSpecHandle primSpec = SdfCreatePrimInLayer(mPrim.GetStage()->GetEditTarget().GetLayer(), mPrim.GetPrimPath());
            if (primSpec)
            {
                static TfToken gTranslate("xformOp:translate");
                pxr::SdfPath attributePath = mPrim.GetPrimPath().AppendProperty(gTranslate);
                SdfAttributeSpecHandle posAttr = primSpec->GetAttributeAtPath(attributePath);
                static TfToken gOrient("xformOp:orient");
                attributePath = mPrim.GetPrimPath().AppendProperty(gOrient);
                SdfAttributeSpecHandle orAttr = primSpec->GetAttributeAtPath(attributePath);
                static TfType quatf = TfType::Find<GfQuatf>();

                if (posAttr && orAttr && orAttr.GetSpec().GetValueType() == quatf)
                {                    
                    mFlags |= InternalActorFlag::eFAST_TRANSFORM;
                }
            }
        }

        if (preTransform || postTransform)
        {
            mFlags |= InternalActorFlag::eHAS_EXTRA_TRANSFORM;
            mExtraTransfInv = mExtraTransfInv.GetInverse();
            if (preTransform)
                mFlags |= InternalActorFlag::eEXTRA_TRANSFORM_PRE_OP;
        }
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

void InternalActor::enableSplineSurfaceVelocity(bool enable,
                                                ::physx::PxRigidActor& actor,
                                                const pxr::UsdGeomBasisCurves& splinesCurvePrim)
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
        mSplinesCurve = db.addSplinesCurve(splinesCurvePrim, added);
        if (mSplinesCurve)
        {
            mSplinesSurfaceVelocityEnabled = enable;
            if (added)
            {
                // compute the relative pose
                const PxTransform actorGlobalPose = actor.getGlobalPose();
                const GfMatrix4d splineGlobalPosePxr = splinesCurvePrim.ComputeLocalToWorldTransform(UsdTimeCode::Default());
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
