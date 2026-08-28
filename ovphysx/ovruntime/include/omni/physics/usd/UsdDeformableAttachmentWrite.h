// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

#include <cstdint>

// USD-specific write-back for auto-generated deformable attachment data.
//
// This lives in the USD library by design. The attachment *generation*
// (tet/triangle finders, geometry sampling, culling) stays in omni.physx;
// only the USD *authoring* of the computed result lives here, so omni.physx
// holds no direct UsdAttribute writes for the auto-attachment pipeline. It is
// deliberately NOT a generic data sink (cf. IPhysicsDataWrite) — these are
// purpose-built for the OmniPhysics deformable-attachment schema.
namespace omni::physics::usd
{

// VtxTetAttachment: barycentric attachment of src0 vertices to src1 tets.
void writeVtxTetAttachment(const PXR_NS::UsdStageWeakPtr& stage,
                           const PXR_NS::SdfPath& attachmentPath,
                           const PXR_NS::VtArray<int32_t>& vtxIndicesSrc0,
                           const PXR_NS::VtArray<int32_t>& tetIndicesSrc1,
                           const PXR_NS::VtArray<PXR_NS::GfVec3f>& tetCoordsSrc1,
                           bool enabled);

// VtxXformAttachment: src0 vertices pinned to local positions on an xformable.
void writeVtxXformAttachment(const PXR_NS::UsdStageWeakPtr& stage,
                             const PXR_NS::SdfPath& attachmentPath,
                             const PXR_NS::VtArray<int32_t>& vtxIndicesSrc0,
                             const PXR_NS::VtArray<PXR_NS::GfVec3f>& localPositionsSrc1,
                             bool enabled);

// ElementCollisionFilter: per-side element-group counts/indices.
void writeElementCollisionFilter(const PXR_NS::UsdStageWeakPtr& stage,
                                 const PXR_NS::SdfPath& filterPath,
                                 const PXR_NS::VtArray<uint32_t>& groupElemCounts0,
                                 const PXR_NS::VtArray<uint32_t>& groupElemIndices0,
                                 const PXR_NS::VtArray<uint32_t>& groupElemCounts1,
                                 const PXR_NS::VtArray<uint32_t>& groupElemIndices1,
                                 bool enabled);

// --- Setup-time prim authoring (creation + relationship targets) ---------

// Define a deformable-attachment child prim of the given typed schema
// (VtxXform/VtxVtx/VtxTri/VtxTet), start it disabled, and point its
// src0/src1 relationships at the two attachable prims.
void defineAttachmentPrim(const PXR_NS::UsdStageWeakPtr& stage,
                          const PXR_NS::SdfPath& primPath,
                          const PXR_NS::TfToken& attachmentTypeName,
                          const PXR_NS::SdfPath& src0Path,
                          const PXR_NS::SdfPath& src1Path);

// Define an ElementCollisionFilter child prim, start it disabled, and
// point its src0/src1 relationships at the two collider prims.
void defineElementCollisionFilterPrim(const PXR_NS::UsdStageWeakPtr& stage,
                                      const PXR_NS::SdfPath& primPath,
                                      const PXR_NS::SdfPath& src0Path,
                                      const PXR_NS::SdfPath& src1Path);

// Disable every Attachment/ElementCollisionFilter child of the auto-
// attachment prim (sets attachmentEnabled / filterEnabled to false).
void disableAttachmentsAndFilters(const PXR_NS::UsdStageWeakPtr& stage,
                                  const PXR_NS::SdfPath& autoAttachmentPath);

// Remove every Attachment/ElementCollisionFilter child of the auto-
// attachment prim.
void removeAttachmentsAndFilters(const PXR_NS::UsdStageWeakPtr& stage,
                                 const PXR_NS::SdfPath& autoAttachmentPath);

} // namespace omni::physics::usd
