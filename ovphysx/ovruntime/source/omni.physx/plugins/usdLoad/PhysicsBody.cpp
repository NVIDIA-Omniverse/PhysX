// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-BODY-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-BODY-002
 * @covers AC-2
 */

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>

#include <omni/physx/IPhysxSettings.h>

#include <physxSchema/tokens.h>
#include <omniUsdPhysicsDeformableSchema/tokens.h>


#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXTools.h>
#include <OmniPhysX.h>
#include <ChangeRegister.h>
#include <CookingDataAsync.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "Mass.h"
#include "Material.h"
#include "CollisionGroup.h"
#include "AttributeHelpers.h"

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include "UsdSource.h"

#include <pxr/usd/usdPhysics/tokens.h>


using namespace PXR_NS;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

void setToDefault(PhysxForceDesc& desc)
{
    desc.enabled = false;
    desc.accelerationMode = true;
    desc.force = { 0.0f, 0.0f, 0.0f };
    desc.torque = { 0.0f, 0.0f, 0.0f };
    desc.worldFrame = false;
    desc.body = kInvalidObjectId;
    desc.scene = kInvalidObjectId;
    desc.localRot = { 0.0f, 0.0f, 0.0f, 1.0f };
}

PhysxForceDesc* parsePhysxForce(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    PhysxForceDesc* desc = ICE_PLACEMENT_NEW(PhysxForceDesc)();
    setToDefault(*desc);

    // Fully source-routed (by ObjectKey): applied-API gate, attribute reads,
    // world transform, and the ancestor xform-op time-sample scan — no UsdPrim.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src && internal::hasAppliedSchema<PhysxSchemaPhysxForceAPI>(*src, key))
    {
        getAttribute(attachedStage, desc->enabled, key, PhysxSchemaTokens->physxForceForceEnabled, updatePhysxForceEnabled);
        getAttribute(attachedStage, desc->worldFrame, key, PhysxSchemaTokens->physxForceWorldFrameEnabled, updatePhysxForceWorldFrameEnabled);

        GfVec3f val{ 0.f };
        getAttribute(attachedStage, val, key, PhysxSchemaTokens->physxForceForce, updatePhysxForce);
        GfVec3ToFloat3(val, desc->force);

        getAttribute(attachedStage, val, key, PhysxSchemaTokens->physxForceTorque, updatePhysxTorque);
        GfVec3ToFloat3(val, desc->torque);

        TfToken mode;
        getAttribute(attachedStage, mode, key, PhysxSchemaTokens->physxForceMode, updatePhysxForceMode);
        desc->accelerationMode = (mode == PhysxSchemaTokens->acceleration);

        // EarliestTime() matches the legacy load-time xform cache
        // (UsdGeomXformCache(UsdTimeCode::EarliestTime())): for a force whose
        // xform op carries time samples but no authored default, this reads the
        // first keyframe rather than the op default/identity.
        const GfMatrix4d pose = internal::getWorldTransform(attachedStage, key, UsdTimeCode::EarliestTime());
        GfVec3ToFloat3(pose.ExtractTranslation(), desc->worldPos);

        if (src->mightWorldTransformBeTimeVarying(key))
        {
            attachedStage.getAnimatedKinematicBodies()[attachedStage.pathFor(key)] = key;
        }
    }

    return desc;
}

SdfPath getRigidBodySimulationOwner(AttachedStage& attachedStage, const SdfPath& bodyPath)
{
    if (bodyPath == SdfPath())
        return SdfPath();

    // physics:simulationOwner is the same relationship whether declared by
    // UsdPhysicsRigidBodyAPI or UsdPhysicsCollisionAPI, so a single source read
    // of the relationship covers both branches of the former API-gated logic.
    SdfPathVector owners;
    omni::physx::internal::getRelationshipValue(
        attachedStage, attachedStage.keyFor(bodyPath), UsdPhysicsTokens->physicsSimulationOwner, owners);
    if (!owners.empty())
    {
        return owners[0];
    }
    return SdfPath();
}

void finalizePhysxForce(AttachedStage& attachedStage, omni::physics::parse::ObjectKey forceKey, PhysxForceDesc& desc)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;

    // search if the force belongs to some dynamic body — walk ancestors via the
    // source (no UsdPrim).
    const omni::physics::parse::ObjectKey root = src->getRootKey();
    ObjectId bodyId = kInvalidObjectId;
    omni::physics::parse::ObjectKey parent = forceKey;
    while (parent.valid() && parent != root)
    {
        const SdfPath parentPath = attachedStage.pathFor(parent);
        bodyId = attachedStage.getObjectDatabase()->findEntry(parentPath, eBody);
        if (bodyId != kInvalidObjectId)
        {
            break;
        }

        bodyId = attachedStage.getObjectDatabase()->findEntry(parentPath, eArticulationLink);
        if (bodyId != kInvalidObjectId)
        {
            break;
        }
        parent = src->getParent(parent);
    }

    if (bodyId != kInvalidObjectId)
    {
        desc.body = bodyId;
        if (internal::hasAppliedSchema<UsdPhysicsRigidBodyAPI>(*src, parent))
        {
            SdfPathVector owners;
            omni::physx::internal::getRelationshipValue(
                attachedStage, parent, UsdPhysicsTokens->physicsSimulationOwner, owners);
            if (!owners.empty())
            {
                const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(owners[0], eScene);
                desc.scene = entry;
            }

            // Local rotation between the force prim and its body ancestor,
            // composed from the two source-routed world transforms
            // (rel = childWorld * parentWorld^-1); only the rotation is used.
            // EarliestTime() matches the legacy load-time xform cache (see
            // parsePhysxForce); caching can be reintroduced source-side.
            const GfMatrix4d childWorld =
                internal::getWorldTransform(attachedStage, forceKey, UsdTimeCode::EarliestTime());
            const GfMatrix4d parentWorld =
                internal::getWorldTransform(attachedStage, parent, UsdTimeCode::EarliestTime());
            const GfMatrix4d rel = childWorld * parentWorld.GetInverse();
            const GfTransform tr(rel);
            const GfQuatf localRot = GfQuatf(tr.GetRotation().GetQuat());
            GfQuatToFloat4(localRot, desc.localRot);
        }
    }
}

ObjectId getRigidBody(AttachedStage& attachedStage, const SdfPath& shapeKey, PhysxShapeDesc& shapeDesc)
{
    if (shapeDesc.rigidBody.valid())
        return attachedStage.getObjectDatabase()->findEntry(attachedStage.pathFor(shapeDesc.rigidBody), eBody);
    else
    {
        // Walk ancestors through the source (no UsdPrim) looking for the nearest
        // body / articulation-link entry.
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src)
            return kInvalidObjectId;
        const omni::physics::parse::ObjectKey root = src->getRootKey();
        omni::physics::parse::ObjectKey parent = attachedStage.keyFor(shapeKey);
        while (parent.valid() && parent != root)
        {
            const SdfPath parentPath = attachedStage.pathFor(parent);
            ObjectId bodyId = attachedStage.getObjectDatabase()->findEntry(parentPath, eBody);
            if (bodyId != kInvalidObjectId)
            {
                shapeDesc.rigidBody = parent;
                return bodyId;
            }
            bodyId = attachedStage.getObjectDatabase()->findEntry(parentPath, eArticulationLink);
            if (bodyId != kInvalidObjectId)
            {
                shapeDesc.rigidBody = parent;
                return bodyId;
            }

            parent = src->getParent(parent);
        }
        return kInvalidObjectId;
    }
}

PhysxRigidBodyDesc* createStaticBody()
{
    StaticPhysxRigidBodyDesc* desc = ICE_PLACEMENT_NEW(StaticPhysxRigidBodyDesc)();

    return desc;
}


void finalizeRigidBody(AttachedStage& attachedStage, BodyDescAndColliders& bodyAndColliders)
{
    for (const SdfPath& collisionPath : bodyAndColliders.collisions)
    {
        if (collisionPath != SdfPath())
        {
            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(collisionPath);
            if (entries && !entries->empty())
            {
                auto it = entries->begin();
                while (it != entries->end())
                {
                    if (it->first == eShape)
                    {
                        const ObjectId collisionId = it->second;
                        if (collisionId != kInvalidObjectId)
                        {
                            bodyAndColliders.desc->shapes.push_back(collisionId);
                        }
                    }
                    it++;
                }

            }
        }
    }
}

void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxDeformableBodyDesc& desc)
{
    const float metersPerUnit = units.metersPerUnit;
    float tolerancesSpeed = float(10.0f / metersPerUnit);

    desc.sceneId = kInvalidObjectId;
    desc.simMeshMaterial = kInvalidObjectId;
    desc.transform = GfMatrix4d(1.0f);
    desc.bodyEnabled = false;
    desc.kinematicBody = false;
    desc.startsAsleep = false;
    desc.mass = -1.0f;
    desc.enableSpeculativeCCD = false;
    desc.selfCollision = false;
    desc.disableGravity = false;
    desc.sleepThreshold = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.linearDamping = 0.005f;
    desc.maxLinearVelocity = SQRT_FLT_MAX;
    desc.settlingThreshold = float(0.1f / metersPerUnit);
    desc.settlingDamping = 10.0f;
    desc.maxDepenetrationVelocity = float(3.0f / metersPerUnit);
    desc.contactOffset = -1.0f;
    desc.restOffset = float(0.02f / metersPerUnit);
    desc.selfCollisionFilterDistance = -1.0f;
    desc.solverPositionIterationCount = 16;
    desc.hasAutoAPI = false;
    desc.isAutoMeshSimplificationEnabled = false;
    desc.isAutoRemeshingEnabled = false;
    desc.hasAutoForceConforming = false;
    desc.autoRemeshingResolution = 0;
    desc.autoTriangleTargetCount = 0;
    desc.simMeshLeftHandedOrientation = false;
    desc.collisionMeshLeftHandedOrientation = false;
}

void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxVolumeDeformableBodyDesc& desc)
{
    setToDefault(units, static_cast<PhysxDeformableBodyDesc&>(desc));

    desc.isAutoHexahedralMeshEnabled = false;
    desc.autoHexahedralResolution = 0;
}

void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxSurfaceDeformableBodyDesc& desc)
{
    setToDefault(units, static_cast<PhysxDeformableBodyDesc&>(desc));

    desc.restBendAnglesDefault = OmniUsdPhysicsDeformableSchemaTokens->flatDefault;
    desc.collisionPairUpdateFrequency = 1;
    desc.collisionIterationMultiplier = 1;
}

const SdfPath* findFirstNonEmptyPath(const SdfPathVector& paths)
{
    auto it = std::find_if(paths.begin(), paths.end(), [](const SdfPath path) { return !path.IsEmpty(); });
    return (it != paths.end()) ? &(*it) : nullptr;
}

const TfToken* findFirstNonEmptyToken(const TfTokenVector& tokens)
{
    auto it = std::find_if(tokens.begin(), tokens.end(), [](const TfToken token) { return !token.IsEmpty(); });
    return (it != tokens.end()) ? &(*it) : nullptr;
}


void finalizeDeformableBody(AttachedStage& attachedStage, PhysxDeformableBodyDesc* desc, const SdfPath simMeshMaterial)
{
    ObjectCategory type;
    if (desc->type == eVolumeDeformableBody)
    {
        type = eDeformableMaterial;
    }
    else if (desc->type == eSurfaceDeformableBody)
    {
        type = eSurfaceDeformableMaterial;
    }
    else
    {
        return;
    }

    desc->simMeshMaterial = getMaterial(attachedStage, simMeshMaterial, type);

    registerDeformablePoseChangeParams(attachedStage, desc->simMeshBindPoseToken);
    registerDeformablePoseChangeParams(attachedStage, desc->collisionMeshBindPoseToken);
    for (const TfToken instanceToken : desc->skinGeomBindPoseTokens)
    {
        registerDeformablePoseChangeParams(attachedStage, instanceToken);
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
