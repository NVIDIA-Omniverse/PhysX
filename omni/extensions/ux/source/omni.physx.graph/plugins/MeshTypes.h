// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include <PxPhysicsAPI.h>
#include <gsl/span>
#include <omni/physx/MeshKey.h>
#include <omni/physx/IPhysxCookingService.h> // PhysxCookingMeshView

// MeshKey is just a 128 bit hash to identify a Mesh from its vertices/faces etc.
typedef omni::physx::usdparser::MeshKey MeshKey;
typedef omni::physx::usdparser::MeshKeyHash MeshKeyHash;


// MeshView is a read only window on some arrays that describe points, indices and faces of mesh
// It additionally keeps track of sign scale and mesh orientation
struct MeshView
{
    omni::physx::PhysxCookingMeshView cookingMeshView;
    carb::Float3 signScale = { 1.0f, 1.0f, 1.0f };
    uint64_t primStageId = 0;
    uint64_t primId = 0;
    uint64_t primTokenId = 0;
    bool usePrimID = false;
};

// MeshFacesToTrianglesMapping is holding a triangles to faces mapping array after triangulating a mesh
struct MeshFacesToTrianglesMapping
{
    std::vector<uint32_t> trianglesToFacesMapping; // maps triangles to original faces in the mesh
};

// A pair of MeshKeys identifying the mesh itself and mesh with its collision approximation
struct MeshHashes
{
    MeshKey meshKey; // Hashes mesh data before triangulation
    MeshKey cookedDataCRC; // Hashes mesh data before triangulation, collision approximation (+ parameters)
};

// Parameters to indicate how the cooking system should be cooking the data
struct MeshCollisionParameters
{
    double metersPerUnit = 1.0;
    uint64_t collisionApproximation = 0; // Token of the approximation ('triangleMesh', 'convexHull' etc.)
};

// MeshDefinitionView is a MeshView with already computed hashes and collision parameters
struct MeshDefinitionView
{
    MeshHashes meshHashes;
    MeshView meshView;
    MeshCollisionParameters meshCollision;
};

// MeshInputView is a MeshView with its transform and collision parameters.
// It additionally tracks a stage and a path as handles / tokens.
// Path and stage are not used internally to the system to read any data, as every input is always
// specified through a MeshView and how to read from the source is a duty of the caller.
// They are effectively 'userData' for callers that need storing such data to identify their input meshes
// at later stages of some algorithms, for example when running a BVH to find overlap pairs etc.
struct MeshInputView
{
    pxr::GfMatrix4d worldMatrix; // World matrix (with scale)
    MeshView meshView;
    MeshKey meshKey; // Hashes mesh data before triangulation
    MeshCollisionParameters meshCollision;
};

// MeshGeometryData holds world transform (without scale) and the geometry created from cooked data (with scale)
struct MeshGeometryData
{
    physx::PxTriangleMeshGeometry triangleMeshGeometry;
    physx::PxConvexMeshGeometry convexMeshGeometry;
    MeshFacesToTrianglesMapping* meshFacesToTrianglesMapping = nullptr;
    physx::PxTransform worldTransform; // World Transform (without scale)
    pxr::GfMatrix4d worldMatrix; // World matrix (with scale)
    bool isValid = false;
};

// MeshCookedData holds the actual Px**Meshes
struct MeshCookedData
{
    MeshHashes meshHashes;
    physx::PxTriangleMesh* pxTriangleMesh = nullptr;
    physx::PxConvexMesh* pxConvexMesh = nullptr;
    MeshFacesToTrianglesMapping* meshFacesToTrianglesMapping = nullptr;
    physx::PxU32 uniqueMeshIndex = 0xffffffff;
    physx::PxIntBool isValid = 0;
};
