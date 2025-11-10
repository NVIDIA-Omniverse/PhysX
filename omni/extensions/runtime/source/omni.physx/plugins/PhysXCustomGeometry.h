// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysxCustomGeometry.h>
#include <common/foundation/Allocator.h>

#include <private/omni/physx/PhysxUsd.h>

#include <map>

namespace omni
{
namespace physx
{
struct CustomGeometryInfo
{
    pxr::TfToken customGeomtryAPIToken;
    ICustomGeometryCallback customGeometryCb;
    ::physx::PxCustomGeometry::Type* typeId;
};


class CustomPhysXGeometryCallback : public ::physx::PxCustomGeometry::Callbacks, public Allocateable
{
public:
    CustomPhysXGeometryCallback(const pxr::SdfPath& path,
                                const usdparser::CustomPhysxShapeDesc& shapeDesc,
                                const CustomGeometryInfo& customGeometryInfo,
                                void* userObject);

    void release();

    // PxCustomGeometry::Callbacks boilerplate
    virtual ::physx::PxCustomGeometry::Type getCustomType() const
    {
        return *mCustomGeometryInfo.typeId;
    }

    ::physx::PxBounds3 getLocalBounds(const ::physx::PxGeometry& geometry) const;

    bool generateContacts(const ::physx::PxGeometry& geom0,
                          const ::physx::PxGeometry& geom1,
                          const ::physx::PxTransform& pose0,
                          const ::physx::PxTransform& pose1,
                          const float contactDistance,
                          const float meshContactMargin,
                          const float toleranceLength,
                          ::physx::PxContactBuffer& contactBuffer) const;

    ::physx::PxU32 raycast(const ::physx::PxVec3& origin,
                           const ::physx::PxVec3& unitDir,
                           const ::physx::PxGeometry& geom,
                           const ::physx::PxTransform& pose,
                           ::physx::PxReal maxDist,
                           ::physx::PxHitFlags hitFlags,
                           ::physx::PxU32 maxHits,
                           ::physx::PxGeomRaycastHit* rayHits,
                           ::physx::PxU32 stride,
                           ::physx::PxRaycastThreadContext* threadContext) const;
    bool overlap(const ::physx::PxGeometry& geom0,
                 const ::physx::PxTransform& pose0,
                 const ::physx::PxGeometry& geom1,
                 const ::physx::PxTransform& pose1,
                 ::physx::PxOverlapThreadContext* threadContext) const;
    bool sweep(const ::physx::PxVec3& unitDir,
               const ::physx::PxReal maxDist,
               const ::physx::PxGeometry& geom0,
               const ::physx::PxTransform& pose0,
               const ::physx::PxGeometry& geom1,
               const ::physx::PxTransform& pose1,
               ::physx::PxGeomSweepHit& sweepHit,
               ::physx::PxHitFlags hitFlags,
               const ::physx::PxReal inflation,
               ::physx::PxSweepThreadContext* threadContext) const;

    void visualize(const ::physx::PxGeometry& geometry,
                   ::physx::PxRenderOutput& out,
                   const ::physx::PxTransform& absPose,
                   const ::physx::PxBounds3& cullbox) const;
    void computeMassProperties(const ::physx::PxGeometry& geometry, ::physx::PxMassProperties& massProperties) const;
    bool usePersistentContactManifold(const ::physx::PxGeometry& geometry, ::physx::PxReal& breakingThreshold) const;

    ~CustomPhysXGeometryCallback()
    {
    }

    void* getUserObject() const
    {
        return mUserObject;
    }

private:
    pxr::SdfPath mCustomGeometryPath;
    CustomGeometryInfo mCustomGeometryInfo;
    void* mUserObject;
};

using CustomGeometryRegistryMap = std::unordered_map<size_t, CustomGeometryInfo>;
using CustomGeometryTypeMap = std::unordered_map<pxr::TfToken, CustomGeometryInfo, pxr::TfToken::HashFunctor>;
using CustomGeometryMap = std::unordered_map<pxr::SdfPath, CustomPhysXGeometryCallback*, pxr::SdfPath::Hash>;

class PhysXCustomGeometryManager
{
public:
    PhysXCustomGeometryManager();
    ~PhysXCustomGeometryManager();

    size_t registerCustomGeometry(const pxr::TfToken& customGeometryAPIToken, ICustomGeometryCallback& geometryCallback);
    void unregisterCustomGeometry(size_t id);

    void clear()
    {
        for (CustomGeometryMap::reference ref : mCustomGeometryMap)
        {
            delete ref.second;
        }
        mCustomGeometryMap.clear();
    }

    CustomPhysXGeometryCallback* createCustomGeometry(const pxr::SdfPath& primPath,
                                                      const usdparser::CustomPhysxShapeDesc& customShapeDesc);

    void removeCustomGeometry(const pxr::SdfPath& primPath);

    const CustomGeometryTypeMap& getCustomGeometryTypeMap() const
    {
        return mCustomGeometryTypeMap;
    }

    bool isKnownType(const ::physx::PxCustomGeometry::Type& customType) const
    {
        for (CustomGeometryTypeMap::const_reference ref : mCustomGeometryTypeMap)
        {
            if (*ref.second.typeId == customType)
                return true;
        }
        return false;
    }

private:
    size_t mGeometryRegistryCounter;
    CustomGeometryRegistryMap mCustomGeometryRegistryMap;
    CustomGeometryTypeMap mCustomGeometryTypeMap;
    CustomGeometryMap mCustomGeometryMap;
};
} // namespace physx
} // namespace omni
