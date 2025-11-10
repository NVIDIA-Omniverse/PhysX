// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Types.h>
#include <PxPhysicsAPI.h>
#include <private/omni/physx/PhysxUsd.h>

#include "PhysXDefines.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "ConeCylinderConvexMesh.h"

using namespace omni::physx;
using namespace physx;
using namespace omni::physx::usdparser;

PxConvexMesh* omni::physx::createCylinderConvexMesh(const float width, const float radius, const uint32_t numCirclePoints,
    enum omni::physx::usdparser::Axis axis)
{
#define MAX_NUM_VERTS_IN_CIRCLE 64
    CARB_ASSERT(numCirclePoints <= MAX_NUM_VERTS_IN_CIRCLE);
    PxVec3 verts[2 * MAX_NUM_VERTS_IN_CIRCLE];
    PxU32 numVerts = 2 * numCirclePoints;
    const PxF32 dtheta = 2 * PxPi / (1.0f * numCirclePoints);

    PxU32 axisIndex;
    PxU32 otherIndex0;
    PxU32 otherIndex1;
    if (axis == eX)
    {
        axisIndex = 0;
        otherIndex0 = 1;
        otherIndex1 = 2;
    }
    else if (axis == eY)
    {
        axisIndex = 1;
        otherIndex0 = 0;
        otherIndex1 = 2;
    }
    else
    {
        axisIndex = 2;
        otherIndex0 = 0;
        otherIndex1 = 1;
    }

    for (PxU32 i = 0; i < numCirclePoints; i++)
    {
        const PxF32 theta = dtheta * i;
        const PxF32 cosTheta = radius * PxCos(theta);
        const PxF32 sinTheta = radius * PxSin(theta);

        PxVec3& vt0 = verts[2 * i + 0];
        vt0[axisIndex] = -0.5f * width;
        vt0[otherIndex0] = cosTheta;
        vt0[otherIndex1] = sinTheta;

        PxVec3& vt1 = verts[2 * i + 1];
        vt1[axisIndex] = +0.5f * width;
        vt1[otherIndex0] = cosTheta;
        vt1[otherIndex1] = sinTheta;
    }

    // Create descriptor for convex mesh
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = numVerts;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = verts;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxConvexMesh* convexMesh = NULL;
    PxDefaultMemoryOutputStream buf;
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxConvexMeshCookingResult::Enum condition;
    if (PxCookConvexMesh(physxSetup.getDefaultCookingParams(), convexDesc, buf, &condition))
    {
        if (condition == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
            CARB_LOG_INFO("CreateCylinderConvexMesh: polygon limit reached, resulting convex hull may be suboptimal.");
        PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
        convexMesh = physxSetup.getPhysics()->createConvexMesh(id);
    }

    return convexMesh;
}

PxConvexMesh* omni::physx::createConeConvexMesh(const float width, const float radius, const uint32_t numCirclePoints,
    enum omni::physx::usdparser::Axis axis)
{
#define MAX_NUM_VERTS_IN_CIRCLE 64
    CARB_ASSERT(numCirclePoints <= MAX_NUM_VERTS_IN_CIRCLE);
    PxVec3 verts[MAX_NUM_VERTS_IN_CIRCLE + 1];
    PxU32 numVerts = numCirclePoints + 1;
    const PxF32 dtheta = 2 * PxPi / (1.0f * numCirclePoints);

    PxU32 axisIndex;
    PxU32 otherIndex0;
    PxU32 otherIndex1;
    if (axis == eX)
    {
        axisIndex = 0;
        otherIndex0 = 1;
        otherIndex1 = 2;
    }
    else if (axis == eY)
    {
        axisIndex = 1;
        otherIndex0 = 0;
        otherIndex1 = 2;
    }
    else
    {
        axisIndex = 2;
        otherIndex0 = 0;
        otherIndex1 = 1;
    }

    for (PxU32 i = 0; i < numCirclePoints; i++)
    {
        const PxF32 theta = dtheta * i;
        const PxF32 cosTheta = radius * PxCos(theta);
        const PxF32 sinTheta = radius * PxSin(theta);

        PxVec3& vt = verts[i];
        vt[axisIndex] = -0.5f * width;
        vt[otherIndex0] = cosTheta;
        vt[otherIndex1] = sinTheta;
    }
    PxVec3& vt = verts[numCirclePoints];
    vt[axisIndex] = 0.5f * width;
    vt[otherIndex0] = 0;
    vt[otherIndex1] = 0;

    // Create descriptor for convex mesh
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = numVerts;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = verts;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxConvexMesh* convexMesh = NULL;
    PxDefaultMemoryOutputStream buf;
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxConvexMeshCookingResult::Enum condition;
    if (PxCookConvexMesh(physxSetup.getDefaultCookingParams(), convexDesc, buf, &condition))
    {
        if (condition == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
            CARB_LOG_INFO("CreateConeConvexMesh: polygon limit reached, resulting convex hull may be suboptimal.");
        PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
        convexMesh = physxSetup.getPhysics()->createConvexMesh(id);
    }

    return convexMesh;
}

::physx::PxVec3 omni::physx::getConeOrCylinderScale(const ::physx::PxF32 halfWidth, const ::physx::PxF32 radius,
    enum omni::physx::usdparser::Axis axis)
{
    PxVec3 scale;
    if (axis == eX)
    {
        scale.x = halfWidth;
        scale.y = radius;
        scale.z = radius;
    }
    else if (axis == eY)
    {
        scale.x = radius;
        scale.y = halfWidth;
        scale.z = radius;
    }
    else
    {
        scale.x = radius;
        scale.y = radius;
        scale.z = halfWidth;
    }

    return scale;
}

void omni::physx::getConeOrCylinderSize(const ::physx::PxVec3& scale, enum omni::physx::usdparser::Axis axis, ::physx::PxF32& halfWidth, ::physx::PxF32& radius)
{
    if (axis == eX)
    {
        halfWidth = scale.x;
        radius = scale.y;
    }
    else if (axis == eY)
    {
        radius = scale.x;
        halfWidth = scale.y;
    }
    else
    {
        radius = scale.x;
        halfWidth = scale.z;
    }
}
