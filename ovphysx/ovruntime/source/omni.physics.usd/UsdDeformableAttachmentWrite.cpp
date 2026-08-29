// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <omni/physics/usd/UsdDeformableAttachmentWrite.h>

#include <omniUsdPhysicsDeformableSchema/tokens.h>

#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/schemaRegistry.h>

#include <vector>

using namespace PXR_NS;

namespace omni::physics::usd
{

namespace
{
template <typename T>
void setAttr(const UsdPrim& prim, const TfToken& attr, const T& value)
{
    if (UsdAttribute a = prim.GetAttribute(attr))
        a.Set(value);
}
} // namespace

void writeVtxTetAttachment(const UsdStageWeakPtr& stage,
                           const SdfPath& attachmentPath,
                           const VtArray<int32_t>& vtxIndicesSrc0,
                           const VtArray<int32_t>& tetIndicesSrc1,
                           const VtArray<GfVec3f>& tetCoordsSrc1,
                           bool enabled)
{
    if (!stage)
        return;
    const UsdPrim prim = stage->GetPrimAtPath(attachmentPath);
    if (!prim)
        return;
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsVtxIndicesSrc0, vtxIndicesSrc0);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsTetIndicesSrc1, tetIndicesSrc1);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsTetCoordsSrc1, tetCoordsSrc1);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsAttachmentEnabled, enabled);
}

void writeVtxXformAttachment(const UsdStageWeakPtr& stage,
                             const SdfPath& attachmentPath,
                             const VtArray<int32_t>& vtxIndicesSrc0,
                             const VtArray<GfVec3f>& localPositionsSrc1,
                             bool enabled)
{
    if (!stage)
        return;
    const UsdPrim prim = stage->GetPrimAtPath(attachmentPath);
    if (!prim)
        return;
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsVtxIndicesSrc0, vtxIndicesSrc0);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsLocalPositionsSrc1, localPositionsSrc1);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsAttachmentEnabled, enabled);
}

void writeElementCollisionFilter(const UsdStageWeakPtr& stage,
                                 const SdfPath& filterPath,
                                 const VtArray<uint32_t>& groupElemCounts0,
                                 const VtArray<uint32_t>& groupElemIndices0,
                                 const VtArray<uint32_t>& groupElemCounts1,
                                 const VtArray<uint32_t>& groupElemIndices1,
                                 bool enabled)
{
    if (!stage)
        return;
    const UsdPrim prim = stage->GetPrimAtPath(filterPath);
    if (!prim)
        return;
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsGroupElemCounts0, groupElemCounts0);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsGroupElemIndices0, groupElemIndices0);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsGroupElemCounts1, groupElemCounts1);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsGroupElemIndices1, groupElemIndices1);
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsFilterEnabled, enabled);
}

namespace
{
void setTargets(const UsdPrim& prim, const TfToken& rel, const SdfPath& target)
{
    if (UsdRelationship r = prim.GetRelationship(rel))
        r.SetTargets({ target });
}
} // namespace

void defineAttachmentPrim(const UsdStageWeakPtr& stage,
                          const SdfPath& primPath,
                          const TfToken& attachmentTypeName,
                          const SdfPath& src0Path,
                          const SdfPath& src1Path)
{
    if (!stage)
        return;
    const UsdPrim prim = stage->DefinePrim(primPath, attachmentTypeName);
    if (!prim)
        return;
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsAttachmentEnabled, false);
    setTargets(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSrc0, src0Path);
    setTargets(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSrc1, src1Path);
}

void defineElementCollisionFilterPrim(const UsdStageWeakPtr& stage,
                                      const SdfPath& primPath,
                                      const SdfPath& src0Path,
                                      const SdfPath& src1Path)
{
    if (!stage)
        return;
    const UsdPrim prim = stage->DefinePrim(primPath, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsElementCollisionFilter);
    if (!prim)
        return;
    setAttr(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsFilterEnabled, false);
    setTargets(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSrc0, src0Path);
    setTargets(prim, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSrc1, src1Path);
}

void disableAttachmentsAndFilters(const UsdStageWeakPtr& stage, const SdfPath& autoAttachmentPath)
{
    if (!stage)
        return;
    const UsdPrim autoAttachmentPrim = stage->GetPrimAtPath(autoAttachmentPath);
    if (!autoAttachmentPrim)
        return;
    const TfType attachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsAttachment);
    const TfType filterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsElementCollisionFilter);
    for (const UsdPrim& child : autoAttachmentPrim.GetChildren())
    {
        if (child.IsA(attachmentType))
            setAttr(child, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsAttachmentEnabled, false);
        else if (child.IsA(filterType))
            setAttr(child, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsFilterEnabled, false);
    }
}

void removeAttachmentsAndFilters(const UsdStageWeakPtr& stage, const SdfPath& autoAttachmentPath)
{
    if (!stage)
        return;
    const UsdPrim autoAttachmentPrim = stage->GetPrimAtPath(autoAttachmentPath);
    if (!autoAttachmentPrim)
        return;
    const TfType attachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsAttachment);
    const TfType filterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsElementCollisionFilter);
    std::vector<SdfPath> toRemove;
    for (const UsdPrim& child : autoAttachmentPrim.GetChildren())
    {
        if (child.IsA(attachmentType) || child.IsA(filterType))
            toRemove.push_back(child.GetPath());
    }
    for (const SdfPath& path : toRemove)
        stage->RemovePrim(path);
}

} // namespace omni::physics::usd
