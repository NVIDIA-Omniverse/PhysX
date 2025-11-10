// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "ArticulationMetatype.h"
#include "PhysicsTypes.h"

#include <carb/Types.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::DofMotion;

struct Subspace
{
    carb::Float3 origin{ 0.0f, 0.0f, 0.0f };
};

enum class RigidBodyType : uint8_t
{
    eRigidDynamic,
    eArticulationLink,
    eInvalid = 0xff,
};

struct RigidBodyEntry
{
    ::physx::PxRigidBody* body = nullptr;

    RigidBodyType type = RigidBodyType::eInvalid;

    pxr::SdfPath path;

    Subspace* subspace = nullptr;

    ::physx::PxU32 numShapes = 0;
    std::vector<::physx::PxShape*> shapes;
};

struct SoftBodyEntry
{
    ::physx::PxSoftBody* body = nullptr;

    pxr::SdfPath path;

    Subspace* subspace = nullptr;
    std::vector<::physx::PxMat33> simRestPoses;
    std::vector<::physx::PxU32> simIndices;
};

struct SoftBodyMaterialEntry
{
    ::physx::PxFEMSoftBodyMaterial* femSoftBodyMaterial = nullptr;
    pxr::SdfPath path;
};

struct DeformableBodyEntry
{
    ::physx::PxDeformableBody* body = nullptr;

    pxr::SdfPath path;
    pxr::SdfPath simMeshPath;
    pxr::SdfPath collMeshPath;

    Subspace* subspace = nullptr;
    std::vector<::physx::PxU32> simIndices;
    std::vector<::physx::PxVec3> restPositions;
    std::vector<::physx::PxU32> collIndices;
};

struct DeformableMaterialEntry
{
    ::physx::PxDeformableMaterial* material = nullptr;
    pxr::SdfPath path;
};

struct ParticleSystemEntry
{
    ::physx::PxPBDParticleSystem* particleSystem = nullptr;
    pxr::SdfPath path;
};

struct SdfShapeEntry
{
    pxr::SdfPath path;
    Subspace* subspace = nullptr;
    ::physx::PxU32 numSamplePoints = 0;
    ::physx::PxShape* shape;
};

struct ParticleClothEntry
{
    ::physx::PxParticleClothBuffer* cloth = nullptr;
    ::physx::PxPBDParticleSystem* particleSystem = nullptr;

    pxr::SdfPath path;

    Subspace* subspace = nullptr;
};

struct ParticleMaterialEntry
{
    ::physx::PxPBDMaterial* pbdMaterial = nullptr;
    pxr::SdfPath path;
};

struct DofImpl
{
    ::physx::PxArticulationJointReducedCoordinate* joint = nullptr;
    ::physx::PxArticulationAxis::Enum axis = ::physx::PxArticulationAxis::eTWIST;
    ::physx::PxU32 driveType = 0;
};

// represents an articulation instance
struct ArticulationEntry
{
    // common type info that can be shared by multiple articulations
    const ArticulationMetatype* metatype = nullptr;

    ::physx::PxArticulationReducedCoordinate* arti = nullptr;

    pxr::SdfPath path;

    Subspace* subspace = nullptr;

    ::physx::PxU32 numLinks = 0;
    ::physx::PxU32 numDofs = 0;
    ::physx::PxU32 numShapes = 0;
    ::physx::PxU32 numFixedTendons = 0;
    ::physx::PxU32 numSpatialTendons = 0;

    std::vector<::physx::PxArticulationLink*> links;
    std::vector<::physx::PxU32> dofStarts;
    std::vector<FreeD6RotationAxesFlags> freeD6Axes;
    std::vector<DofImpl> dofImpls;
    std::vector<::physx::PxShape*> shapes;
    std::vector<::physx::PxArticulationFixedTendon*> fixedTendons;
    std::vector<::physx::PxArticulationSpatialTendon*> spatialTendons;
    std::vector<::physx::PxQuat> incomingJointPhysxToUsdRotations; // per-link rotation that was applied to the incoming
                                                                   // joint frame during USD parsing.
    std::vector<::physx::PxTransform> jointChild; // incoming joint child transform
    std::vector<::physx::PxTransform> jointParent; // incoming joint parent transform
    std::vector<::physx::PxU32> parentIndices; // parent indices
    std::vector<bool> isIncomingJointBody0Parent; // whether USD structure is similar to physx SDK
};

inline DofMotion fromPhysx(::physx::PxArticulationMotion::Enum m)
{
    if (m == ::physx::PxArticulationMotion::eFREE)
    {
        return omni::physics::tensors::DofMotion::eFree;
    }
    else if (m == ::physx::PxArticulationMotion::eLIMITED)
    {
        return omni::physics::tensors::DofMotion::eLimited;
    }
    else if (m == ::physx::PxArticulationMotion::eLOCKED)
    {
        return omni::physics::tensors::DofMotion::eLocked;
    }
    else
    {
        return omni::physics::tensors::DofMotion::eInvalid;
    }
}

struct RigidContactSensorEntry
{
    pxr::SdfPath path;
    std::vector<pxr::SdfPath> filterPaths;

    uint64_t referentId = 0; // USD path alias

    ::physx::PxArticulationLink* link = nullptr; // set if referent is an articulation link
    ::physx::PxRigidDynamic* rd = nullptr; // set if referent is a rigid dynamic
    ::physx::PxShape* shape = nullptr; // set if referent is a rigid shape
    uint32_t nameID = 0xffffffff;
    std::unordered_map<uint64_t, uint32_t> filterIndexMap; // maps USD path to filter index

    Subspace* subspace = nullptr;
};

// asInt() is the same as SdfPath::_AsInt()
// asInt(a)==asInt(b) <=> a is same path as b,
// which is how SdfPath::operator== is currently defined.
// If USD changes sizeof(pxr::SdfPath), we will need to change
inline uint64_t asInt(const pxr::SdfPath& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");
    uint64_t ret;
    std::memcpy(&ret, &path, sizeof(pxr::SdfPath));
    return ret;
}

inline const pxr::SdfPath& intToPath(const uint64_t& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");
    return reinterpret_cast<const pxr::SdfPath&>(path);
}

} // namespace tensors
} // namespace physx
} // namespace omni
