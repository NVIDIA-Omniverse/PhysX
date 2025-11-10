// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <internal/Internal.h>
#include "foundation/PxTransform.h"

namespace physx
{
class PxGeometry;
}

namespace omni
{
namespace physx
{

struct AttachmentActorTypeDeprecated
{
    enum Enum
    {
        eINVALID = 0,
        eDEFORMABLE_BODY = 1 << 0,
        eDEFORMABLE_SURFACE = 1 << 1,
        ePARTICLE_CLOTH = 1 << 2,
        eRIGID = 1 << 3,
        eWORLD = 1 << 4,

        eDEFORMABLE = eDEFORMABLE_BODY | eDEFORMABLE_SURFACE | ePARTICLE_CLOTH
    };
};

struct DeformableMeshInfoDeprecated
{
    DeformableMeshInfoDeprecated() : type(AttachmentActorTypeDeprecated::eINVALID)
    {
    }

    AttachmentActorTypeDeprecated::Enum type;
    std::vector<uint32_t> indices;
    std::vector<carb::Float3> positions;
    std::vector<carb::Float3> restPositions;
    usdparser::MeshKey meshCrc;
    pxr::SdfPath scenePath;
};

typedef void (*getGeometryInfoCallbackDeprecated)(const ::physx::PxGeometry& geom,
                                                  const ::physx::PxTransform& geomPos,
                                                  const ::physx::PxVec3& scale,
                                                  const ::physx::PxTransform* shapeTransform,
                                                  void* userData);

bool computeAttachmentPointsDeprecated(const pxr::SdfPath& attachmentPath);
bool computeAttachmentPointsDeprecated(const pxr::SdfPath& deformablePath,
                                       const pxr::SdfPath& rigidBodyPath,
                                       const pxr::SdfPath& attachmentPath);

bool getDeformableMeshInfoDeprecated(const pxr::SdfPath& deformablePath,
                                     DeformableMeshInfoDeprecated& deformableMeshInfo);

void parseRigidBodyShapesDeprecated(const pxr::UsdPrim& rigidBodyPrim,
                                    const usdparser::PhysxShapeDesc* desc,
                                    getGeometryInfoCallbackDeprecated callbackFn,
                                    void* userData);

} // namespace physx
} // namespace omni
