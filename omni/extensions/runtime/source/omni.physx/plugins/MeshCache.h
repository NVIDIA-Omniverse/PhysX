// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <omni/physx/IPhysx.h>
#include <PxPhysicsAPI.h>
#include <private/omni/physx/PhysxUsd.h>

#include <map>
#include <unordered_map>
#include <vector>
#include <gsl/span>

#include <omni/physx/IPhysxCooking.h> // PhysXCookedDataSpan

namespace omni
{
namespace physx
{

using ConvexMeshVector = std::vector<::physx::PxConvexMesh*>;
using ConvexMeshMap =
    std::unordered_map<omni::physx::usdparser::MeshKey, ::physx::PxConvexMesh*, omni::physx::usdparser::MeshKeyHash>;
using TriangleMeshMap =
    std::unordered_map<omni::physx::usdparser::MeshKey, ::physx::PxTriangleMesh*, omni::physx::usdparser::MeshKeyHash>;
using ConvexDecompositionMap =
    std::unordered_map<omni::physx::usdparser::MeshKey, ConvexMeshVector, omni::physx::usdparser::MeshKeyHash>;
using ConvexMeshDataMap = std::multimap<::physx::PxConvexMesh*, ConvexMeshData>;
using TriMeshRemapTable = std::unordered_map<const ::physx::PxTriangleMesh*, const uint32_t*>;
using SphereFillMap = std::unordered_map<omni::physx::usdparser::MeshKey,
                                         omni::physx::usdparser::SpherePointsPhysxShapeDesc*,
                                         omni::physx::usdparser::MeshKeyHash>;

struct CollisionRepresentation;
class MeshCache
{
public:
    MeshCache()
    {
    }

    ~MeshCache()
    {
        release();
    }

    bool hasConvexMesh(const omni::physx::usdparser::MeshKey& crc) const
    {
        bool ret = false;

        ConvexMeshMap::const_iterator it = mConvexMeshMap.find(crc);
        if (it != mConvexMeshMap.end())
        {
            ret = true;
        }

        return ret;
    }

    bool hasTriangleMesh(const omni::physx::usdparser::MeshKey& crc) const
    {
        bool ret = false;
        TriangleMeshMap::const_iterator it = mTriangleMeshMap.find(crc);
        if (it != mTriangleMeshMap.end())
        {
            ret = true;
        }
        return ret;
    }

    bool hasConvexDecomposition(const omni::physx::usdparser::MeshKey& crc) const
    {
        bool ret = false;

        ConvexDecompositionMap::const_iterator found = mConvexDecompositionMap.find(crc);
        if (found != mConvexDecompositionMap.end())
        {
            ret = true;
        }

        return ret;
    }

    ::physx::PxConvexMesh* getConvexMesh(const omni::physx::usdparser::MeshKey& crc) const
    {
        ConvexMeshMap::const_iterator it = mConvexMeshMap.find(crc);
        if (it != mConvexMeshMap.end())
            return it->second;
        return nullptr;
    }

    ConvexMeshPolygon* getConvexMeshPolygons(::physx::PxConvexMesh* mesh);

    void addConvexMesh(const omni::physx::usdparser::MeshKey& crc, ::physx::PxConvexMesh* mesh);

    ::physx::PxTriangleMesh* getTriangleMesh(const omni::physx::usdparser::MeshKey& crc) const
    {
        TriangleMeshMap::const_iterator it = mTriangleMeshMap.find(crc);
        if (it != mTriangleMeshMap.end())
            return it->second;
        return nullptr;
    }

    void addTriangleMesh(const omni::physx::usdparser::MeshKey& crc,
                         ::physx::PxTriangleMesh* mesh,
                         pxr::UsdPrim* prim,
                         gsl::span<const uint32_t>* inputTrianglesFaceMapping,
                         bool computeRemapTable)
    {
        mesh->acquireReference();
        mTriangleMeshMap[crc] = mesh;
        if (inputTrianglesFaceMapping && !inputTrianglesFaceMapping->empty())
        {
            uint32_t* triangleFaceMapping = new uint32_t[inputTrianglesFaceMapping->size()];
            memcpy(triangleFaceMapping, inputTrianglesFaceMapping->data(),
                   sizeof(uint32_t) * inputTrianglesFaceMapping->size());
            mTriMeshRemapTable[mesh] = triangleFaceMapping;
        }
        else
        {
            CARB_ASSERT(!computeRemapTable || prim);
            mTriMeshRemapTable[mesh] = (computeRemapTable ? getRemapTable(*prim) : nullptr);
        }
    }

    const uint32_t* getTriangleMeshFaceMap(const ::physx::PxTriangleMesh* triMesh) const
    {
        TriMeshRemapTable::const_iterator fit = mTriMeshRemapTable.find(triMesh);
        if (fit != mTriMeshRemapTable.end())
            return fit->second;
        return nullptr;
    }

    const ConvexDecompositionMap& getConvexDecompositionMap() const
    {
        return mConvexDecompositionMap;
    }

    const ConvexMeshDataMap& getConvexMeshDataMap() const
    {
        return mConvexMeshDataMap;
    }

    void addConvexDecomposition(const omni::physx::usdparser::MeshKey& crc, const ConvexMeshVector& meshes);

    void release();

    static const uint32_t* getRemapTable(const pxr::UsdPrim& prim);

    const SphereFillMap& getSphereFillMap() const
    {
        return mSphereFillMap;
    }

    const omni::physx::usdparser::SpherePointsPhysxShapeDesc* addSphereFill(
        const omni::physx::usdparser::MeshKey& crc, const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc)
    {
        omni::physx::usdparser::SpherePointsPhysxShapeDesc* d = new omni::physx::usdparser::SpherePointsPhysxShapeDesc;
        *d = desc;
        mSphereFillMap[crc] = d;
        return d;
    }

    const omni::physx::usdparser::SpherePointsPhysxShapeDesc* getSphereFill(const omni::physx::usdparser::MeshKey& crc) const
    {
        SphereFillMap::const_iterator it = mSphereFillMap.find(crc);
        if (it != mSphereFillMap.end())
            return it->second;
        return nullptr;
    }

    bool createRuntimeTriangleMesh(::physx::PxPhysics& physics,
                                   usdparser::MeshKey meshCRC,
                                   bool useOriginalTriangles,
                                   omni::physx::PhysxCookedDataSpan inputCookedData,
                                   gsl::span<const uint32_t> inputTrianglesFaceMapping,
                                   ::physx::PxTriangleMesh** returnedMesh = nullptr);

    bool createRuntimeSphereFill(::physx::PxPhysics& physics,
                                 usdparser::MeshKey crc,
                                 omni::physx::PhysxCookedDataSpan inputCookedData,
                                 const omni::physx::usdparser::SpherePointsPhysxShapeDesc** returnedMesh);

    bool createRuntimeConvexMesh(::physx::PxPhysics& physics,
                                 usdparser::MeshKey crc,
                                 omni::physx::PhysxCookedDataSpan inputCookedData,
                                 ::physx::PxConvexMesh** returnedMesh);

    bool createRuntimeConvexDecomposition(::physx::PxPhysics& physics,
                                          usdparser::MeshKey crc,
                                          const omni::physx::PhysxCookedDataSpan* inputCookedData,
                                          uint64_t inputCookedDataNumber,
                                          std::vector<::physx::PxConvexMesh*>* returnedMeshes);

    omni::physx::CollisionRepresentation* getCollisionRepresentation(usdparser::MeshKey crc,
                                                                     const omni::physx::usdparser::PhysxShapeDesc& desc);

private:
    ConvexMeshMap mConvexMeshMap;
    TriangleMeshMap mTriangleMeshMap;
    ConvexDecompositionMap mConvexDecompositionMap;
    ConvexMeshDataMap mConvexMeshDataMap;
    TriMeshRemapTable mTriMeshRemapTable;
    SphereFillMap mSphereFillMap;
};

MeshCache* getMeshCache();
void releaseMeshCache();
} // namespace physx
} // namespace omni
