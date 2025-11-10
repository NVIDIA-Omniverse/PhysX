// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/extras/Hash.h>


namespace omni
{
namespace physx
{
namespace usdparser
{

// This class represents a unique 'key' or fingerprint of a mesh
// This key will uniquely identify a mesh with zero chance of
// collisions with any other mesh. The key is comprised of a single
// 128 bit hash of the mesh data. The mesh data refers to
// the hash of the vertices, indices, the maximum convex hull
// count (for convex decompositions of triangle meshes) as well
// as the sign scale used for this mesh
class MeshKey
{
public:
    // Assignment operator
    void operator=(const MeshKey& k)
    {
        m_hash.d[0] = k.m_hash.d[0];
        m_hash.d[1] = k.m_hash.d[1];
    }
    // The not equal operator for 'MeshKey'
    bool operator!=(const MeshKey& k) const
    {
        return !(k == *this);
    }

    // The equality operator for MeshKey
    bool operator==(const MeshKey& k) const
    {
        return m_hash.d[0] == k.m_hash.d[0] && m_hash.d[1] == k.m_hash.d[1];
    }

    carb::extras::hash128_t getFullHash(void) const
    {
        return m_hash;
    }

    uint64_t getHashIndex(void) const
    {
        uint64_t hash = (uint64_t)m_hash.d[0];
        hash ^= (uint64_t)m_hash.d[1];
        return (hash);
    }

    // Operator to return the hash of the key
    size_t operator()(const MeshKey& k) const
    {
        return static_cast<size_t>(k.getHashIndex());
    }

    // Operator to return the less than result against another key; used by std::multi_map
    bool operator<(const MeshKey& k) const
    {
        int ret = memcmp(m_hash.d, k.m_hash.d, sizeof(m_hash.d));
        return ret < 0;
    }

    // Differentiates between a raw cooked triangle mesh from a convex decomposition of a triangle mesh
    void setMaxHullCount(uint32_t maxHullCount)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&maxHullCount, sizeof(maxHullCount), m_hash);
    }

    void setMaxSpheres(uint32_t maxSpheres)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&maxSpheres, sizeof(maxSpheres), m_hash);
    }

    void setSeedCount(uint32_t seedCount)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&seedCount, sizeof(seedCount), m_hash);
    }

    void setFillMode(uint32_t fillMode)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&fillMode, sizeof(fillMode), m_hash);
    }

    void setMaxHullVertices(uint32_t maxHullVertices)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&maxHullVertices, sizeof(maxHullVertices), m_hash);
    }

    void setErrorPercentage(float errorPercent)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&errorPercent, sizeof(errorPercent), m_hash);
    }

    void setMinThickness(float minThickness)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&minThickness, sizeof(minThickness), m_hash);
    }

    void setTriangleMeshMode(uint32_t mode)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&mode, sizeof(mode), m_hash);
    }

    void setRightHandedOrientation(bool state)
    {
        uint32_t value = state ? 1 : 0;
        m_hash = carb::extras::fnv128hash((const uint8_t*)&value, sizeof(value), m_hash);
    }

    void setUseShrinkwrap(bool state)
    {
        uint32_t value = state ? 1 : 0;
        m_hash = carb::extras::fnv128hash((const uint8_t*)&value, sizeof(value), m_hash);
    }

    void setUseMeshSimplification(bool state)
    {
        uint32_t value = state ? 1 : 0;
        m_hash = carb::extras::fnv128hash((const uint8_t*)&value, sizeof(value), m_hash);
    }

    void setSimplificationMetric(float metric)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&metric, sizeof(metric), m_hash);
    }

    void setWeldTolerance(float tolerance)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&tolerance, sizeof(tolerance), m_hash);
    }

    void setCookedDataVersion(uint32_t v)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&v, sizeof(v), m_hash);
    }

    // Differentiates between a raw cooked triangle mesh from a convex decomposition of a triangle mesh
    void setVoxelResolution(uint32_t voxelResolution)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&voxelResolution, sizeof(voxelResolution), m_hash);
    }

    // Compute the 128 bit hash for the vertices and vertex count and accumulate the hash
    void computeVerticesHash(uint32_t vertexCount, const float* vertices)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&vertexCount, sizeof(vertexCount), m_hash);
        m_hash = carb::extras::fnv128hash((const uint8_t*)vertices, sizeof(float) * 3 * vertexCount, m_hash);
    }

    // Compute the 128 bit hash for the indices (and indices count) and add it to the hash
    void computeIndicesHash(uint32_t indicesCount, const uint32_t* indices)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&indicesCount, sizeof(indicesCount), m_hash);
        m_hash = carb::extras::fnv128hash((const uint8_t*)indices, sizeof(uint32_t) * indicesCount, m_hash);
    }

    // Compute the 128 bit hash for the face indexes and count and add it to the hash
    void computeFacesHash(uint32_t faceCount, const uint32_t* faces)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&faceCount, sizeof(faceCount), m_hash);
        m_hash = carb::extras::fnv128hash((const uint8_t*)faces, sizeof(uint32_t) * faceCount, m_hash);
    }

    // Compute the 128 bit hash for the face hole indexes and count and add it to the hash
    void computeFaceHolesHash(uint32_t holesCount, const uint32_t* holes)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&holesCount, sizeof(holesCount), m_hash);
        m_hash = carb::extras::fnv128hash((const uint8_t*)holes, sizeof(uint32_t) * holesCount, m_hash);
    }

    // Compute the 128 bit hash for the face hole indexes and count and add it to the hash
    void computeFaceMaterialsHash(uint32_t faceMaterialsHash, const uint16_t* materials)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&faceMaterialsHash, sizeof(faceMaterialsHash), m_hash);
        m_hash = carb::extras::fnv128hash((const uint8_t*)materials, sizeof(uint16_t) * faceMaterialsHash, m_hash);
    }

    // Add the sign scale to the hash
    void setSignScale(const float signScale[3])
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)signScale, sizeof(float) * 3, m_hash);
    }

    // Add arbitrary data to hash
    void setMiscData(const uint8_t* data, size_t size)
    {
        m_hash = carb::extras::fnv128hash(data, size, m_hash);
    }

    void setMeshKey(const MeshKey& meshKey)
    {
        m_hash = carb::extras::fnv128hash((const uint8_t*)&meshKey.m_hash, sizeof(m_hash), m_hash);
    }

    void setComputeGPUData(bool state)
    {
        uint32_t value = state ? 1 : 0;
        m_hash = carb::extras::fnv128hash((const uint8_t*)&value, sizeof(value), m_hash);
    }

private:
    carb::extras::hash128_t m_hash{ 0, 0 }; // the hash for this mesh
};

// This class defines the custom hash function to
// be used with the STL containers. It converts a 'MeshKey'
// into a size_t which is used by the STL as a unique
// hash signature. However, the == operator on 'MeshKey'
// is still used to still fully uniquely identify the
// 'MeshKey' as being unique from any other.
// Example: std::unordered_set< MeshKey, MeshKeyHash >
class MeshKeyHash
{
public:
    size_t operator()(const MeshKey& k) const
    {
        return static_cast<size_t>(k.getHashIndex());
    }
};

} // namespace usdparser
} // namespace physx
} // namespace omni
