// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include <common/utilities/Utilities.h>

#include "RigidBody.h"
#include "FilteredPairs.h"
#include "UsdLoad.h"

using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{

bool isEnabledBody(const UsdPrim& usdPrim, const BodyMap& bodyMap, bool& physicsAPIFound, ObjectType::Enum type)
{
    BodyMap::const_iterator it = bodyMap.find(usdPrim.GetPrimPath());
    if (it != bodyMap.end() && (type == ObjectType::eUndefined || it->second->type == type))
    {
        switch (it->second->type)
        {
        case ObjectType::eRigidBody:
            physicsAPIFound = true;
            return ((RigidBodyDesc*)it->second)->rigidBodyEnabled;
        case ObjectType::eVolumeDeformableBody:
        case ObjectType::eSurfaceDeformableBody:
        case ObjectType::eCurvesDeformableBody:
            physicsAPIFound = true;
            return ((BodyDesc*)it->second)->bodyEnabled;
        }
    }

    physicsAPIFound = false;
    return false;
}

bool hasEnabledBodyParent(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, const BodyMap& bodyMap, UsdPrim& bodyPrim, ObjectType::Enum type)
{
    bool physicsAPIFound = false;
    UsdPrim parent = usdPrim;
    while (parent != stage->GetPseudoRoot())
    {
        if (isEnabledBody(parent, bodyMap, physicsAPIFound, type))
        {
            bodyPrim = parent;
            return true;
        }

        if (physicsAPIFound)
        {
            bodyPrim = parent;
            return false;
        }

        parent = parent.GetParent();
    }
    return false;
}

pxr::SdfPath getRigidBody(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, const BodyMap& bodyMap)
{
    UsdPrim bodyPrim = UsdPrim();
    RigidBodyDesc* body = nullptr;
    if (hasEnabledBodyParent(stage, usdPrim, bodyMap, bodyPrim, ObjectType::eRigidBody))
    {
       return bodyPrim.GetPrimPath();
    }
    else
    {
        // collision does not have a dynamic body parent, it is considered a static collision        
        if (bodyPrim == UsdPrim())
        {
            return SdfPath();
        }
        else
        {
            return bodyPrim.GetPrimPath();
        }
    }
}

void getRigidBodyTransformation(UsdGeomXformCache& xfCache, const UsdPrim& bodyPrim, RigidBodyDesc& desc)
{
    GfMatrix4d mat = xfCache.GetLocalToWorldTransform(bodyPrim);
    const GfTransform tr(mat);
    const GfVec3d pos = tr.GetTranslation();
    const GfQuatd rot = tr.GetRotation().GetQuat();
    const GfVec3d sc = tr.GetScale();

    if (!scaleIsUniform(sc[0], sc[1], sc[2]) && tr.GetScaleOrientation().GetQuaternion() != GfQuaternion::GetIdentity())
    {
        CARB_LOG_WARN("ScaleOrientation is not supported for rigid bodies, prim path: %s. You may ignore this if the scale is close to uniform.", bodyPrim.GetPrimPath().GetText());
    }

    desc.position = GfVec3f(pos);
    desc.rotation = GfQuatf(rot);
    desc.scale = GfVec3f(sc);
}

RigidBodyDesc* parseRigidBody(const UsdStageWeakPtr stage, UsdGeomXformCache& xfCache, const UsdPrim& bodyPrim, const BodyMap& bodyMap, uint64_t primTypes)
{
    //not using UsdPhysicsBodyAPI for backwards compatiblity
    const UsdPhysicsRigidBodyAPI rigidBodyAPI(bodyPrim);

    if (!(primTypes & PrimType::eUsdGeomXformable))
    {
        CARB_LOG_ERROR("RigidBodyAPI applied to a non-xformable primitive. (%s)", bodyPrim.GetPrimPath().GetText());
        return nullptr;
    }

    CARB_ASSERT(rigidBodyAPI);

    RigidBodyDesc* desc = new RigidBodyDesc();

    // transformation
    getRigidBodyTransformation(xfCache, bodyPrim, *desc);

    // filteredPairs
    parseFilteredPairs(stage, bodyPrim, desc->filteredCollisions);

    // velocity
    rigidBodyAPI.GetVelocityAttr().Get(&desc->linearVelocity);
    rigidBodyAPI.GetAngularVelocityAttr().Get(&desc->angularVelocity);

    // rigid body flags
    rigidBodyAPI.GetRigidBodyEnabledAttr().Get(&desc->rigidBodyEnabled);

    // body flags
    rigidBodyAPI.GetKinematicEnabledAttr().Get(&desc->kinematicBody);
    rigidBodyAPI.GetStartsAsleepAttr().Get(&desc->startsAsleep);

    // simulation owner
    const UsdRelationship ownerRel = rigidBodyAPI.GetSimulationOwnerRel();
    if (ownerRel)
    {
        SdfPathVector owners;
        ownerRel.GetTargets(&owners);
        if (!owners.empty())
        {
            desc->simulationOwners = owners;
        }
    }

    // Ensure if we have a hierarchical parent that has an enabled rigid body,
    // that we also have a reset xform stack, otherwise we should log an error.
    UsdPrim bodyParent = UsdPrim();
    if (hasEnabledBodyParent(stage, bodyPrim, bodyMap, bodyParent, ObjectType::eRigidBody))
    {
        bool hasResetXformStack = false;
        UsdPrim parent = bodyPrim;
        while (parent != stage->GetPseudoRoot() && parent != bodyParent)
        {
            if (xfCache.GetResetXformStack(parent))
            {
                hasResetXformStack = true;
                break;
            }
            parent = parent.GetParent();
        }                   
        if (!hasResetXformStack)
        {
            CARB_LOG_ERROR("Rigid Body of (%s) missing xformstack reset when child of another enabled rigid body (%s) in hierarchy. "
                           "Simulation of multiple RigidBodyAPI's in a hierarchy will cause unpredicted results. "
                           "Please fix the hierarchy or use XformStack reset.",
                            bodyPrim.GetPrimPath().GetText(),
                            bodyParent.GetPrimPath().GetText());
            delete desc;
            return nullptr;
        }
    }

    return desc;
}

} // namespace schema
} // namespace physics
} // namespace omni
