// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */

/**
 * @implements REQ-PARSE-BODY-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-BODY-002
 * @covers AC-2
 */

#include <carb/Types.h>
#include <common/foundation/Allocator.h>
#include <omni/physx/IPhysxSettings.h>

#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXTools.h>
#include <OmniPhysX.h>
#include <ChangeRegister.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "Mass.h"
#include "Material.h"
#include "CollisionGroup.h"
#include "AttributeHelpers.h"

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

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
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (src && src->hasSchema(key, tok.physxForceAPI))
    {
        const omni::physics::parse::ReadTime readTime = omni::physics::parse::ReadTime::defaultTime();
        getAttribute(attachedStage, desc->enabled, key, tok.physxForceForceEnabled, readTime, updatePhysxForceEnabled);
        getAttribute(attachedStage, desc->worldFrame, key, tok.physxForceWorldFrameEnabled, readTime, updatePhysxForceWorldFrameEnabled);

        carb::Float3 val{ 0.f, 0.f, 0.f };
        getAttribute(attachedStage, val, key, tok.physxForceForce, readTime, updatePhysxForce);
        desc->force = val;

        // `val` is deliberately NOT reset between the two reads: the Gf form
        // this replaced reused the same variable, so a failed torque read
        // carries the force value into desc->torque. Preserved verbatim.
        getAttribute(attachedStage, val, key, tok.physxForceTorque, readTime, updatePhysxTorque);
        desc->torque = val;

        // TokenId-typed read (ADR-0018): the mode enum's string value is compared as a
        // TokenId rather than round-tripping through TfToken/tfTokenFor.
        omni::physics::parse::TokenId modeTok{};
        getAttribute(attachedStage, modeTok, key, tok.physxForceMode, readTime, updatePhysxForceMode);
        desc->accelerationMode = (modeTok == tok.acceleration);

        // EarliestTime() matches the legacy load-time xform cache
        // (UsdGeomXformCache(UsdTimeCode::EarliestTime())): for a force whose
        // xform op carries time samples but no authored default, this reads the
        // first keyframe rather than the op default/identity. The 2-arg
        // getWorldTransform overload is the source's cached EarliestTime read
        // (PhysXTools.h) -- pxr-free and the same load-time result.
        const ::physx::PxMat44d pose = internal::getWorldTransform(attachedStage, key);
        const ::physx::PxVec3d worldPos = pose.getPosition();
        desc->worldPos = { float(worldPos.x), float(worldPos.y), float(worldPos.z) };

        if (src->mightWorldTransformBeTimeVarying(key))
        {
            attachedStage.getAnimatedKinematicBodies().insert(key);
        }
    }

    return desc;
}

omni::physics::parse::ObjectKey getRigidBodySimulationOwner(AttachedStage& attachedStage, omni::physics::parse::ObjectKey bodyKey)
{
    if (!bodyKey.valid())
        return {};

    // physics:simulationOwner is the same relationship whether declared by
    // UsdPhysicsRigidBodyAPI or UsdPhysicsCollisionAPI, so a single source read
    // of the relationship covers both branches of the former API-gated logic.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    std::vector<omni::physics::parse::ObjectKey> owners;
    omni::physx::internal::getRelationshipValue(attachedStage, bodyKey, tok.physicsSimulationOwner, owners);
    if (!owners.empty())
    {
        return owners[0];
    }
    return {};
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
        bodyId = attachedStage.getObjectDatabase()->findEntry(parent, eBody);
        if (bodyId != kInvalidObjectId)
        {
            break;
        }

        bodyId = attachedStage.getObjectDatabase()->findEntry(parent, eArticulationLink);
        if (bodyId != kInvalidObjectId)
        {
            break;
        }
        parent = src->getParent(parent);
    }

    if (bodyId != kInvalidObjectId)
    {
        desc.body = bodyId;
        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
        if (src->hasSchema(parent, tok.physicsRigidBodyAPI))
        {
            std::vector<omni::physics::parse::ObjectKey> owners;
            omni::physx::internal::getRelationshipValue(attachedStage, parent, tok.physicsSimulationOwner, owners);
            if (!owners.empty())
            {
                const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(owners[0], eScene);
                desc.scene = entry;
            }

            // Local rotation between the force prim and its body ancestor,
            // composed from the two source-routed world transforms
            // (rel = childWorld * parentWorld^-1); only the rotation is used.
            // EarliestTime() matches the legacy load-time xform cache (see
            // parsePhysxForce); caching can be reintroduced source-side. The
            // 2-arg getWorldTransform overload is the source's cached
            // EarliestTime read (PhysXTools.h) -- pxr-free, same result.
            const ::physx::PxMat44d childWorld = internal::getWorldTransform(attachedStage, forceKey);
            const ::physx::PxMat44d parentWorld = internal::getWorldTransform(attachedStage, parent);
            // childWorld * parentWorld^-1 in the USD row-vector convention is the
            // reversed product under PhysX's column-vector convention.
            const ::physx::PxMat44d rel = affineInverse(parentWorld) * childWorld;
            desc.localRot = toFloat4(toTransform(rel).q.getNormalized());
        }
    }
}

ObjectId getRigidBody(AttachedStage& attachedStage, omni::physics::parse::ObjectKey shapeKey, PhysxShapeDesc& shapeDesc)
{
    if (shapeDesc.rigidBody.valid())
        return attachedStage.getObjectDatabase()->findEntry(shapeDesc.rigidBody, eBody);
    else
    {
        // Walk ancestors through the source (no UsdPrim) looking for the nearest
        // body / articulation-link entry.
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src)
            return kInvalidObjectId;
        const omni::physics::parse::ObjectKey root = src->getRootKey();
        omni::physics::parse::ObjectKey parent = shapeKey;
        while (parent.valid() && parent != root)
        {
            ObjectId bodyId = attachedStage.getObjectDatabase()->findEntry(parent, eBody);
            if (bodyId != kInvalidObjectId)
            {
                shapeDesc.rigidBody = parent;
                return bodyId;
            }
            bodyId = attachedStage.getObjectDatabase()->findEntry(parent, eArticulationLink);
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
    for (const omni::physics::parse::ObjectKey collisionKey : bodyAndColliders.collisions)
    {
        if (collisionKey.valid())
        {
            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(collisionKey);
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
    desc.transform = omni::physics::parse::Matrix4d{}; // identity (Matrix4d's in-class default)
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

void setToDefault(const omni::physics::parse::SourceUnits& units,
                  const omni::physics::parse::IPhysicsSource& source,
                  PhysxSurfaceDeformableBodyDesc& desc)
{
    setToDefault(units, static_cast<PhysxDeformableBodyDesc&>(desc));

    omni::physics::parse::KnownTokens tok;
    tok.intern(source);
    desc.restBendAnglesDefault = tok.flatDefault;
    desc.collisionPairUpdateFrequency = 1;
    desc.collisionIterationMultiplier = 1;
}

void finalizeDeformableBody(AttachedStage& attachedStage, PhysxDeformableBodyDesc* desc, omni::physics::parse::ObjectKey simMeshMaterial)
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

    // simMeshBindPoseToken/collisionMeshBindPoseToken/skinGeomBindPoseTokens
    // are TokenIds (ADR-0019 increment 7); registerDeformablePoseChangeParams
    // takes a plain instance-name string, so bridge via the source.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src)
    {
        registerDeformablePoseChangeParams(attachedStage, std::string(src->tokenToString(desc->simMeshBindPoseToken)));
        registerDeformablePoseChangeParams(attachedStage, std::string(src->tokenToString(desc->collisionMeshBindPoseToken)));
        for (const omni::physics::parse::TokenId instanceToken : desc->skinGeomBindPoseTokens)
        {
            registerDeformablePoseChangeParams(attachedStage, std::string(src->tokenToString(instanceToken)));
        }
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
