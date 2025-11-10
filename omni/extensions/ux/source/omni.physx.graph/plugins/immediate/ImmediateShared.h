// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include <PxPhysicsAPI.h>
#include "ImmediateMeshCache.h"
#include <common/foundation/TypeCast.h>

namespace carb
{
namespace tasking
{
struct ITasking;
}
} // namespace carb
namespace omni
{
namespace physx
{
namespace graph
{
// These are the globals shared between all Immediate Nodes
struct ImmediateShared
{
    ::physx::PxDefaultAllocator pxAllocator;
    ::physx::PxDefaultErrorCallback pxErrorCallback;

    ::physx::PxFoundation* pxFoundation = nullptr;
    carb::tasking::ITasking* carbTasking = nullptr;

    int refCount = 0;
    bool singleThreaded = false;
    uint32_t numberOfTasks = 24;

    ImmediateMeshCache immediateMeshCache;

    bool initialize();
    void release();

    template <typename T>
    static bool parseBoundsFromAttributes(::physx::PxBounds3& bounds, const T& boxMinAttr, const T& boxMaxAttr);

    // Parses a bounding box transform and applies it to an input bounds struct
    template <typename T>
    static bool parseAndTransformBoundingBoxFromAttribute(::physx::PxBounds3& bounds, const T& boxTransformAttr);
};

// Parses the axis aligned bounding box coming from two attributes specifying the min and max corners
template <typename T>
bool ImmediateShared::parseBoundsFromAttributes(::physx::PxBounds3& bounds, const T& boxMinAttr, const T& boxMaxAttr)
{
    const auto boxMinFloat = boxMinAttr.template getCpu<float[3]>();
    const auto boxMaxFloat = boxMaxAttr.template getCpu<float[3]>();
    const auto boxMinDouble = boxMinAttr.template getCpu<double[3]>();
    const auto boxMaxDouble = boxMaxAttr.template getCpu<double[3]>();
    if (boxMinDouble && boxMaxDouble)
    {
        bounds.minimum.x = static_cast<float>(boxMinDouble[0]);
        bounds.minimum.y = static_cast<float>(boxMinDouble[1]);
        bounds.minimum.z = static_cast<float>(boxMinDouble[2]);
        bounds.maximum.x = static_cast<float>(boxMaxDouble[0]);
        bounds.maximum.y = static_cast<float>(boxMaxDouble[1]);
        bounds.maximum.z = static_cast<float>(boxMaxDouble[2]);
        return true;
    }
    else if (boxMinFloat && boxMaxFloat)
    {
        memcpy(&bounds.minimum, *boxMinFloat, sizeof(::physx::PxVec3));
        memcpy(&bounds.maximum, *boxMaxFloat, sizeof(::physx::PxVec3));
        return true;
    }
    else
    {
        return false;
    }
}

// Parses a bounding box transform and applies it to an input bounds struct
template <typename T>
bool ImmediateShared::parseAndTransformBoundingBoxFromAttribute(::physx::PxBounds3& bounds, const T& boxTransformAttr)
{
    pxr::GfMatrix4d gfWorldMatrix(1.0);
    const auto worldMatrixFloat = boxTransformAttr.template getCpu<float[16]>();
    const auto worldMatrixDouble = boxTransformAttr.template getCpu<double[16]>();
    if (worldMatrixDouble)
    {
        memcpy(&gfWorldMatrix, *worldMatrixDouble, sizeof(double) * 16);
    }
    else if (worldMatrixFloat)
    {
        double* mat = gfWorldMatrix.data();
        for (int i = 0; i < 16; ++i)
        {
            mat[i] = worldMatrixFloat[i];
        }
    }
    else
    {
        return false;
    }
    pxr::GfBBox3d bbox = pxr::GfBBox3d(
        pxr::GfRange3d(omni::physx::toVec3d(bounds.minimum), omni::physx::toVec3d(bounds.maximum)), gfWorldMatrix);
    pxr::GfRange3d range = bbox.ComputeAlignedRange();
    const pxr::GfVec3d min = range.GetMin();
    const pxr::GfVec3d max = range.GetMax();
    bounds.minimum = omni::physx::toPhysX(min);
    bounds.maximum = omni::physx::toPhysX(max);
    return true;
}

} // namespace graph
} // namespace physx
} // namespace omni
