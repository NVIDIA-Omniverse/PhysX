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
#include <common/utilities/UsdMaterialParsing.h>

#include "DeformableBody.h"
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
            pxr::UsdAttribute getPosePurposesAttr(pxr::UsdPrim posePrim, pxr::TfToken instanceName)
            {
                pxr::TfToken attrName = pxr::UsdSchemaRegistry::MakeMultipleApplyNameInstance(OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, instanceName);
                return posePrim.GetAttribute(attrName);
            }

            pxr::TfToken getPoseNameFromPurpose(const UsdPrim prim, const pxr::TfToken posePurposeToken)
            {
                TfTokenVector allAPIs = prim.GetAppliedSchemas();

                TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
                TfToken poseTypeName = UsdSchemaRegistry::GetAPISchemaTypeName(poseType);

                for (const auto& api : allAPIs)
                {
                    std::pair<TfToken, TfToken> typeNameAndInstance = UsdSchemaRegistry::GetTypeNameAndInstance(api);
                    if (typeNameAndInstance.first == poseTypeName)
                    {
                        VtArray<TfToken> candTokens;
                        getPosePurposesAttr(prim, typeNameAndInstance.second).Get(&candTokens);
                        for (const TfToken candToken : candTokens)
                        {
                            if (candToken == posePurposeToken)
                            {
                                return typeNameAndInstance.second;
                            }
                        }
                    }
                }

                return pxr::TfToken();
            }

            bool isSimMesh(ObjectType::Enum& type, const UsdPrim prim)
            {
                TfType volumeType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
                TfType surfaceType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
                TfType curvesType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->CurvesDeformableSimAPI);

                type = ObjectType::eUndefined;
                if (prim.HasAPI(volumeType))
                {
                    bool isTetMesh = prim.IsA<UsdGeomTetMesh>();
                    if (!isTetMesh)
                    {
                        CARB_LOG_ERROR(
                            "VolumeDeformableSimAPI applied to non-tetmesh. (%s)", prim.GetPrimPath().GetText());
                        return false;
                    }
                    type = ObjectType::eVolumeDeformableBody;
                }
                else if (prim.HasAPI(surfaceType))
                {
                    bool isTriMesh = prim.IsA<UsdGeomMesh>();
                    if (!isTriMesh)
                    {
                        CARB_LOG_ERROR(
                            "SurfaceDeformableSimAPI applied to non-trimesh. (%s)", prim.GetPrimPath().GetText());
                        return false;
                    }
                    type = ObjectType::eSurfaceDeformableBody;
                }
                else if (prim.HasAPI(curvesType))
                {
                    bool isCurves = prim.IsA<UsdGeomBasisCurves>();
                    if (!isCurves)
                    {
                        CARB_LOG_ERROR(
                            "CurvesDeformableSimAPI applied to non-curves prim. (%s)", prim.GetPrimPath().GetText());
                        return false;
                    }
                    type = ObjectType::eCurvesDeformableBody;
                }
                return type != ObjectType::eUndefined;
            }

            bool isPointBased(const UsdPrim prim)
            {
                return prim.IsA<UsdGeomPointBased>();
            }

            bool isEnabledCollision(const UsdPrim prim)
            {
                UsdPhysicsCollisionAPI collisionAPI(prim);
                if (collisionAPI)
                {
                    bool isEnabled;
                    collisionAPI.GetCollisionEnabledAttr().Get(&isEnabled);
                    return isEnabled;
                }
                return false;
            }

            SdfPath getDeformableMaterialBinding(const UsdStageWeakPtr stage, const SdfPath primPath, const ObjectType::Enum deformableType)
            {
                UsdPrim prim = stage->GetPrimAtPath(primPath);
                if (prim)
                {
                    const SdfPath& matPath = usdmaterialutils::getMaterialBinding(prim);
                    bool needSurfaceMat = (deformableType == ObjectType::eSurfaceDeformableBody);
                    bool needCurvesMat = (deformableType == ObjectType::eCurvesDeformableBody);

                    if (!matPath.IsEmpty())
                    {
                        const UsdPrim matPrim = stage->GetPrimAtPath(matPath);
                        if (matPrim)
                        {
                            TfType matType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);
                            TfType surfaceMatType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableMaterialAPI);
                            TfType curvesMatType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->CurvesDeformableMaterialAPI);
                            if (needSurfaceMat && matPrim.HasAPI(surfaceMatType))
                            {
                                return matPath;
                            }
                            else if (needCurvesMat && matPrim.HasAPI(curvesMatType))
                            {
                                return matPath;
                            }
                            else if (matPrim.HasAPI(matType))
                            {
                                return matPath;
                            }
                        }
                    }
                }
                return SdfPath();
            }

            DeformableBodyDesc* parseDeformableBody(const UsdStageWeakPtr stage, UsdGeomXformCache& xfCache, const UsdPrim& bodyPrim, const BodyMap& bodyMap, uint64_t primTypes)
            {
                static const uint64_t geomTypes = PrimType::eUsdGeomTetMesh | PrimType::eUsdGeomMesh | PrimType::eUsdGeomBasisCurves;

                bool validGeomTypes = (primTypes & geomTypes) > 0;
                bool validRootTypes = (primTypes & PrimType::eUsdGeomGprim) == 0 && (primTypes & PrimType::eUsdGeomImageable) > 0;
                if (!validGeomTypes && !validRootTypes)
                {
                    CARB_LOG_ERROR("DeformableBodyAPI can only be applied to Mesh, TetMesh or BasisCurves for single node deformables "
                        "or non-Gprim Imageable for hierarchical setups. (%s)", bodyPrim.GetPrimPath().GetText());
                    return nullptr;
                }

                DeformableBodyDesc* desc = new DeformableBodyDesc();

                // transformation
                GfMatrix4d mat = xfCache.GetLocalToWorldTransform(bodyPrim);
                desc->transform = mat;

                TfType deformableBodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
                if (bodyPrim.HasAPI(deformableBodyType))
                {
                    // enabled flag
                    bodyPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->deformableBodyEnabled).Get(&desc->bodyEnabled);

                    // total mass
                    bodyPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->mass).Get(&desc->mass);
                }

                // filteredPairs
                parseFilteredPairs(stage, bodyPrim, desc->filteredCollisions);

                TfType bodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->BodyAPI);
                if (bodyPrim.HasAPI(bodyType))
                {
                    // body flags
                    bodyPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->kinematicEnabled).Get(&desc->kinematicBody);
                    bodyPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->startsAsleep).Get(&desc->startsAsleep);

                    // simulation owner
                    const UsdRelationship ownerRel = bodyPrim.GetRelationship(OmniPhysicsDeformableAttrTokens->simulationOwner);
                    if (ownerRel)
                    {
                        SdfPathVector owners;
                        ownerRel.GetTargets(&owners);
                        if (!owners.empty())
                        {
                            desc->simulationOwners = owners;
                        }
                    }
                }

                // Ensure if we have a hierarchical parent that has an enabled rigid body,
                // that we also have a reset xform stack, otherwise we should log an error.
                UsdPrim bodyParent = UsdPrim();
                if (hasEnabledBodyParent(stage, bodyPrim, bodyMap, bodyParent, ObjectType::eRigidBody))
                {
                    bool hasResetXformStack = xfCache.GetResetXformStack(bodyPrim);
                    if (!hasResetXformStack)
                    {
                        CARB_LOG_ERROR("Deformable Body of (%s) missing xformstack reset when child of rigid body (%s) in hierarchy. "
                            "Simulation of deformable body under RigidBodyAPI's in a hierarchy will cause unpredicted results. "
                            "Please fix the hierarchy or use XformStack reset.",
                            bodyPrim.GetPrimPath().GetText(),
                            bodyParent.GetPrimPath().GetText());
                        delete desc;
                        return nullptr;
                    }
                }

                UsdPrim simMeshPrim;
                ObjectType::Enum simMeshType = ObjectType::eUndefined;

                // process hierarchie including bodyPrim
                UsdPrimRange prims(bodyPrim, pxr::UsdPrimAllPrimsPredicate);
                for (pxr::UsdPrimRange::const_iterator it = prims.begin(); it != prims.end(); ++it)
                {
                    UsdPrim prim = *it;
                    if (!isPointBased(prim))
                    {
                        continue;
                    }
                    if (prim != bodyPrim && xfCache.GetResetXformStack(prim))
                    {
                        it.PruneChildren();
                        continue;
                    }
                    ObjectType::Enum type;
                    bool isSim = isSimMesh(type, prim);
                    bool isColl = isEnabledCollision(prim);
                    if (isSim)
                    {
                        if (simMeshPrim.IsValid())
                        {
                            CARB_LOG_ERROR("Multiple deformable simulation meshes with VolumeDeformableSimAPI/SurfaceDeformableSimAPI found at (%s).",
                                           bodyPrim.GetPrimPath().GetText());
                            delete desc;
                            return nullptr;
                        }
                        if (prim != bodyPrim && prim.GetParent() != bodyPrim)
                        {
                            CARB_LOG_ERROR("VolumeDeformableSimAPI/SurfaceDeformableSimAPI found that is neither on prim with UsdPhysicsDeformableBody "
                                           "nor an immediate child, (%s).",
                                           bodyPrim.GetPrimPath().GetText());
                            delete desc;
                            return nullptr;
                        }
                        simMeshPrim = prim;
                        simMeshType = type;

                        pxr::TfToken bindPoseToken = getPoseNameFromPurpose(simMeshPrim, OmniPhysicsDeformableAttrTokens->bindPose);
                        desc->simMeshBindPoseToken = bindPoseToken;
                    }
                    if (isColl)
                    {
                        // we just ignore CollisionAPI simulation owners,
                        // according to the standard schema it should always be subordinate
                        // to the BodyAPI simulation owner.

                        desc->collisionGeomPaths.push_back(prim.GetPath());
                        pxr::TfToken bindPoseToken = getPoseNameFromPurpose(prim, OmniPhysicsDeformableAttrTokens->bindPose);
                        desc->collisionGeomBindPoseTokens.push_back(bindPoseToken);

                        pxr::TfToken selfCollisionFilterPoseToken = getPoseNameFromPurpose(prim, OmniPhysicsDeformableAttrTokens->selfCollisionFilterPose);
                        desc->collisionGeomSelfCollisionFilterPoseTokens.push_back(selfCollisionFilterPoseToken);

                        // filteredPairs
                        pxr::SdfPathVector filteredPairs;
                        parseFilteredPairs(stage, prim, filteredPairs);
                        desc->filteredCollisions.insert(desc->filteredCollisions.end(),
                            filteredPairs.begin(), filteredPairs.end());
                    }
                    if (!isSim && !isColl)
                    {
                        desc->skinGeomPaths.push_back(prim.GetPath());
                        pxr::TfToken bindPoseToken = getPoseNameFromPurpose(prim, OmniPhysicsDeformableAttrTokens->bindPose);
                        desc->skinGeomBindPoseTokens.push_back(bindPoseToken);
                    }
                }

                if (!simMeshPrim)
                {
                    CARB_LOG_ERROR("Deformable simulation mesh is not valid (%s).",
                        bodyPrim.GetPrimPath().GetText());
                    delete desc;
                    return nullptr;
                }
                desc->simMeshPath = simMeshPrim.GetPath();

                // materials provided through sim mesh (or any parent) provides material default.
                desc->simMeshMaterialPath = getDeformableMaterialBinding(stage, desc->simMeshPath, simMeshType);

                // materials coming though non sim geometry can serve as hints for localized materials if supported.
                // collision materials
                for (SdfPath collisionGeomPath : desc->collisionGeomPaths)
                {
                    SdfPath matPath = getDeformableMaterialBinding(stage, collisionGeomPath, simMeshType);
                    desc->collisionGeomMaterialPaths.push_back(matPath);
                }
                // skin materials
                for (SdfPath skinGeomPath : desc->skinGeomPaths)
                {
                    SdfPath matPath = getDeformableMaterialBinding(stage, skinGeomPath, simMeshType);
                    desc->skinGeomMaterialPaths.push_back(matPath);
                }

                // set type
                desc->type = simMeshType;

                return desc;
            }

        } // namespace schema
    } // namespace physics
} // namespace omni
