// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CACHE-ATTACH-001
// @covers AC-7

#pragma once

#include <omni/physx/IPhysx.h>
#include <PxPhysicsAPI.h>
#include <private/omni/physx/PhysxUsd.h>

#include <carb/tasking/TaskingUtils.h>
#include <atomic>
#include <mutex>
#include <map>
#include <memory>
#include <cstring>
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
// @implements REQ-ATTACH-SERVICES-001
// @covers AC-3
// Entries are immutable after publication. Lookup containers are synchronized;
// returned data stays borrowed until the externally serialized runtime cache reset.
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
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
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
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
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
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
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
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        ConvexMeshMap::const_iterator it = mConvexMeshMap.find(crc);
        if (it != mConvexMeshMap.end())
            return it->second;
        return nullptr;
    }

    ConvexMeshPolygon* getConvexMeshPolygons(::physx::PxConvexMesh* mesh);

    void addConvexMesh(const omni::physx::usdparser::MeshKey& crc, ::physx::PxConvexMesh* mesh);

    ::physx::PxTriangleMesh* getTriangleMesh(const omni::physx::usdparser::MeshKey& crc) const
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        TriangleMeshMap::const_iterator it = mTriangleMeshMap.find(crc);
        if (it != mTriangleMeshMap.end())
            return it->second;
        return nullptr;
    }

    void addTriangleMesh(const omni::physx::usdparser::MeshKey& crc,
                         ::physx::PxTriangleMesh* mesh,
                         gsl::span<const uint32_t>* inputTrianglesFaceMapping,
                         bool computeRemapTable)
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        // The map takes the mesh before its reference: if the insertion throws,
        // the cache holds no reference it cannot find again.
        mTriangleMeshMap[crc] = mesh;
        mesh->acquireReference();
        if (inputTrianglesFaceMapping && !inputTrianglesFaceMapping->empty())
        {
            uint32_t* triangleFaceMapping = new uint32_t[inputTrianglesFaceMapping->size()];
            memcpy(triangleFaceMapping, inputTrianglesFaceMapping->data(),
                   sizeof(uint32_t) * inputTrianglesFaceMapping->size());
            mTriMeshRemapTable[mesh] = triangleFaceMapping;
        }
        else
        {
            mTriMeshRemapTable[mesh] = nullptr;
        }
    }

    // A triangle mesh restored from a compiled bundle carries its face map in the
    // bundle. Its owner forgets the map before releasing the mesh: entries are
    // keyed by address.
    void restoreTriangleMeshFaceMap(const ::physx::PxTriangleMesh* mesh, const uint32_t* faces, size_t count)
    {
        std::unique_ptr<uint32_t[]> copy(new uint32_t[count]);
        memcpy(copy.get(), faces, sizeof(uint32_t) * count);
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        const uint32_t*& entry = mTriMeshRemapTable[mesh];
        delete[] entry;
        entry = copy.release();
    }

    void forgetTriangleMeshFaceMap(const ::physx::PxTriangleMesh* mesh)
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        const TriMeshRemapTable::iterator found = mTriMeshRemapTable.find(mesh);
        if (found == mTriMeshRemapTable.end())
            return;
        delete[] found->second;
        mTriMeshRemapTable.erase(found);
    }

    const uint32_t* getTriangleMeshFaceMap(const ::physx::PxTriangleMesh* triMesh) const
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        TriMeshRemapTable::const_iterator fit = mTriMeshRemapTable.find(triMesh);
        if (fit != mTriMeshRemapTable.end())
            return fit->second;
        return nullptr;
    }

    // The compiled stage cache stores a triangle mesh as the cooked stream it was
    // created from: PhysX binary serialization drops part of a mesh cooked with GPU
    // data (see cache::encodeCollection). Once an attachment that can store has
    // been configured, each triangle mesh created from cooked data keeps a copy of
    // its stream until release(). A mesh created before has none, and a store
    // that needs it bypasses. Tests pass false to create such a mesh.
    void retainTriangleMeshCookedData(bool retain = true)
    {
        mRetainTriangleMeshCookedData.store(retain, std::memory_order_relaxed);
    }

    std::shared_ptr<const std::vector<uint8_t>> getTriangleMeshCookedData(const ::physx::PxTriangleMesh* mesh) const
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        const auto found = mTriMeshCookedData.find(mesh);
        return found == mTriMeshCookedData.end() ? nullptr : found->second;
    }

    ConvexMeshVector getConvexDecomposition(const omni::physx::usdparser::MeshKey& crc) const
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        const auto found = mConvexDecompositionMap.find(crc);
        return found == mConvexDecompositionMap.end() ? ConvexMeshVector{} : found->second;
    }

    bool getConvexMeshData(::physx::PxConvexMesh* mesh, ConvexMeshData& data) const
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        const auto found = mConvexMeshDataMap.find(mesh);
        if (found == mConvexMeshDataMap.end())
            return false;
        data = found->second;
        return true;
    }

    void addConvexDecomposition(const omni::physx::usdparser::MeshKey& crc, const ConvexMeshVector& meshes);

    void release();

    const omni::physx::usdparser::SpherePointsPhysxShapeDesc* addSphereFill(
        const omni::physx::usdparser::MeshKey& crc, const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc)
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
        omni::physx::usdparser::SpherePointsPhysxShapeDesc* d = new omni::physx::usdparser::SpherePointsPhysxShapeDesc;
        *d = desc;
        mSphereFillMap[crc] = d;
        return d;
    }

    const omni::physx::usdparser::SpherePointsPhysxShapeDesc* getSphereFill(const omni::physx::usdparser::MeshKey& crc) const
    {
        std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(mMutex);
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
    mutable carb::tasking::RecursiveMutexWrapper mMutex;
    ConvexMeshMap mConvexMeshMap;
    TriangleMeshMap mTriangleMeshMap;
    ConvexDecompositionMap mConvexDecompositionMap;
    ConvexMeshDataMap mConvexMeshDataMap;
    TriMeshRemapTable mTriMeshRemapTable;
    std::unordered_map<const ::physx::PxTriangleMesh*, std::shared_ptr<const std::vector<uint8_t>>> mTriMeshCookedData;
    std::atomic<bool> mRetainTriangleMeshCookedData{ false };
    SphereFillMap mSphereFillMap;
};

MeshCache* getMeshCache();
void releaseMeshCache();
} // namespace physx
} // namespace omni
