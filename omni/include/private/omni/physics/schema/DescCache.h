// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "IUsdPhysicsListener.h"

#include <map>
#include <mutex>

namespace omni
{
namespace physics
{
namespace schema
{

struct PrimType
{
    enum Enum
    {
        eUsdXform = 1 << 0,
        eUsdGeomXformable = 1 << 1,
        eUsdGeomPointInstancer = 1 << 2,
        eUsdPhysicsScene = 1 << 3,
        eUsdPhysicsCollisionGroup = 1 << 4,
        eUsdPhysicsJoint = 1 << 5,
        eUsdPhysicsRevoluteJoint = 1 << 6,
        eUsdPhysicsFixedJoint = 1 << 7,
        eUsdPhysicsPrismaticJoint = 1 << 8,
        eUsdPhysicsSphericalJoint = 1 << 9,
        eUsdPhysicsDistanceJoint = 1 << 10,
        eUsdGeomImageable = 1 << 11,
        eUsdGeomGprim = 1 << 12,
        eUsdGeomMesh = 1 << 13,
        eUsdGeomTetMesh = 1 << 14,
        eUsdGeomBasisCurves = 1 << 15,
        eUsdPhysicsAttachment = 1 << 16,
        eUsdPhysicsVtxVtxAttachment = 1 << 17,
        eUsdPhysicsVtxTriAttachment = 1 << 18,
        eUsdPhysicsVtxTetAttachment = 1 << 19,
        eUsdPhysicsVtxCrvAttachment = 1 << 20,
        eUsdPhysicsVtxXformAttachment = 1 << 21,
        eUsdPhysicsTetXformAttachment = 1 << 22,
        eUsdPhysicsTriTriAttachment = 1 << 23,
        eUsdPhysicsElementCollisionFilter = 1 << 24,
        eLast = 1 << 25
    };
};

using PathObjectMap = std::unordered_map<pxr::SdfPath, const ShapeDesc*, pxr::SdfPath::Hash>;
using PrimTypesMap = pxr::TfHashMap<pxr::TfType, uint64_t, pxr::TfHash>;

// Desc cache
class DescCache
{
public:
    void clear()
    {
        for (PathObjectMap::reference ref : mPathObjectMap)
        {
            delete ref.second;
        }
        mPathObjectMap.clear();
        mPrimTypesMap.clear();
    }

    void addDesc(const pxr::SdfPath& path, const ShapeDesc* desc)
    {
        mPathObjectMap[path] = desc;
    }

    const ShapeDesc* getDesc(const pxr::SdfPath& path) const
    {
        PathObjectMap::const_iterator fit = mPathObjectMap.find(path);
        if (fit != mPathObjectMap.end())
            return fit->second;
        else
            return nullptr;
    }

    bool getPrimTypes(const pxr::TfType& primType, uint64_t& types) const
    {
        PrimTypesMap::const_iterator fit = mPrimTypesMap.find(primType);
        if (fit != mPrimTypesMap.end())
        {
            types = fit->second;
            return true;
        }
        else
        {
            return false;
        }
    }

    void addPrimTypes(const pxr::TfType& primType, uint64_t types)
    {
        mPrimTypesMap[primType] = types;
    }

private:
    PathObjectMap mPathObjectMap;
    PrimTypesMap mPrimTypesMap;
};

} // namespace schema
} // namespace physics
} // namespace omni
