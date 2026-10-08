// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-ATTRS-001
 * @covers AC-10, AC-14
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-5
 */

#pragma once

#include <PxMaterial.h>
#include <PxShape.h>
#include <foundation/PxSimpleTypes.h>

#include <algorithm>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{

// Per-shape output-read columns shared by the rigid-body and articulation views. The clamp, the
// first-material rule and the null handling live here so the two readers cannot drift apart.
enum class OvStageShapeProperty
{
    eStaticFriction,
    eDynamicFriction,
    eRestitution,
    eContactOffset,
    eRestOffset,
    // "Not a per-shape column": lets a caller's attribute table carry this as a field instead of a
    // second list of which rows are per-shape. Filtered out by the reader before any gather.
    eNone
};

// Whether a property is read off the shape's MATERIAL rather than the shape. A material property
// missing from this list reintroduces a per-column getMaterials() pass (REQ-READ-ATTRS-001 AC-12).
inline bool ovStageShapePropertyNeedsMaterial(OvStageShapeProperty property)
{
    switch (property)
    {
    case OvStageShapeProperty::eStaticFriction:
    case OvStageShapeProperty::eDynamicFriction:
    case OvStageShapeProperty::eRestitution:
        return true;
    case OvStageShapeProperty::eContactOffset:
    case OvStageShapeProperty::eRestOffset:
    case OvStageShapeProperty::eNone:
        return false;
    }
    return false;
}

// Label for a check that concerns the whole set of per-shape columns rather than any single one,
// such as the record index check.
inline const char* ovStageShapePropertySetLabel()
{
    return "per-shape columns";
}

// Attribute name for the check helpers' messages, so two refused columns do not log the same line.
inline const char* ovStageShapePropertyLabel(OvStageShapeProperty property)
{
    switch (property)
    {
    case OvStageShapeProperty::eStaticFriction:  return "static friction";
    case OvStageShapeProperty::eDynamicFriction: return "dynamic friction";
    case OvStageShapeProperty::eRestitution:     return "restitution";
    case OvStageShapeProperty::eContactOffset:   return "contact offset";
    case OvStageShapeProperty::eRestOffset:      return "rest offset";
    // Unreachable with a valid column: callers reject eNone before asking for a label.
    case OvStageShapeProperty::eNone:            return ovStageShapePropertySetLabel();
    }
    return ovStageShapePropertySetLabel();
}

// One requested per-shape column: which property, and the zero-initialized row it fills.
struct OvStageShapeColumn
{
    OvStageShapeProperty property;
    float* dst;
};

// The count a reader may index: the declared count clamped against the vector backing it. Also what
// `shapeCount` reports, so the number a consumer uses to find where the real values stop is the same
// number that bounded the write -- padding cannot mark the end on its own, since 0.0 is a legal offset.
inline ::physx::PxU32 effectiveOvStageShapeCount(::physx::PxU32 declaredCount,
                                                 const std::vector<::physx::PxShape*>& shapes)
{
    return std::min(declaredCount, static_cast<::physx::PxU32>(shapes.size()));
}

// The SET form: one walk of a body's shapes fills every requested column, resolving the material once
// per shape rather than once per shape per column (REQ-READ-ATTRS-001 AC-12), and only when some
// requested column needs it -- a contactOffset + restOffset read touches no material at all.
//
// The caller owns tensor validation and supplies zero-initialized rows. Only real shapes are written,
// so missing shapes/materials and the padded tail remain deterministic zeros.
inline void gatherOvStageShapeRows(const OvStageShapeColumn* columns,
                                   ::physx::PxU32 numColumns,
                                   ::physx::PxU32 width,
                                   ::physx::PxU32 declaredCount,
                                   const std::vector<::physx::PxShape*>& shapes)
{
    if (!columns || numColumns == 0)
    {
        return;
    }

    bool anyNeedsMaterial = false;
    for (::physx::PxU32 c = 0; c < numColumns && !anyNeedsMaterial; ++c)
    {
        anyNeedsMaterial = ovStageShapePropertyNeedsMaterial(columns[c].property);
    }

    const ::physx::PxU32 shapeCount = std::min(effectiveOvStageShapeCount(declaredCount, shapes), width);
    for (::physx::PxU32 shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex)
    {
        ::physx::PxShape* const shape = shapes[shapeIndex];
        if (!shape)
            continue;

        // A multi-material shape reports material 0, matching TensorBindingsAPI. Null is not an error:
        // the material columns keep their zero for this shape, the offset columns are unaffected.
        ::physx::PxMaterial* material = nullptr;
        if (anyNeedsMaterial && shape->getMaterials(&material, 1) == 0)
        {
            material = nullptr;
        }

        for (::physx::PxU32 c = 0; c < numColumns; ++c)
        {
            float& value = columns[c].dst[shapeIndex];
            // Every enumerator is listed rather than closed with a default: an unlisted property must
            // leave the caller's zero, not fall into the material arm. -Wno-switch is set tree-wide.
            switch (columns[c].property)
            {
            case OvStageShapeProperty::eContactOffset:
                value = shape->getContactOffset();
                break;
            case OvStageShapeProperty::eRestOffset:
                value = shape->getRestOffset();
                break;
            case OvStageShapeProperty::eStaticFriction:
                if (material)
                    value = material->getStaticFriction();
                break;
            case OvStageShapeProperty::eDynamicFriction:
                if (material)
                    value = material->getDynamicFriction();
                break;
            case OvStageShapeProperty::eRestitution:
                if (material)
                    value = material->getRestitution();
                break;
            case OvStageShapeProperty::eNone:
                break; // leaves the caller's zero
            }
        }
    }
}

// One-column convenience form; delegates so the clamp, the null-shape skip and the first-material
// rule have a single implementation.
inline void gatherOvStageShapeRow(OvStageShapeProperty property,
                                  float* dst,
                                  ::physx::PxU32 width,
                                  ::physx::PxU32 declaredCount,
                                  const std::vector<::physx::PxShape*>& shapes)
{
    const OvStageShapeColumn column{ property, dst };
    gatherOvStageShapeRows(&column, 1, width, declaredCount, shapes);
}

} // namespace tensors
} // namespace physx
} // namespace omni
