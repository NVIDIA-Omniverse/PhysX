// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-2
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#include <omni/physics/parse/Handles.h>

#include "tensors/ArticulationMetatype.h"
#include "tensors/PhysicsTypes.h"

#include <carb/Types.h>

#include <foundation/PxMat44.h> // PxMat44d: the instancer frames (ADR-0001 section 8)

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physx
{
namespace internal
{
class InternalScene;
class InternalVehicle;
} // namespace internal
} // namespace physx
} // namespace omni

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

    // Source-native path string (ObjectKey-derived; not necessarily a resolvable
    // pxr::SdfPath under the ovstage backend).
    std::string path;

    Subspace* subspace = nullptr;

    ::physx::PxU32 numShapes = 0;
    std::vector<::physx::PxShape*> shapes;
};

// One instance of a point-instancer prototype. The body is a plain PxRigidDynamic in the scene, so it
// resolves through the same superset row map every other rigid read uses.
struct PointInstance
{
    ::physx::PxRigidDynamic* body = nullptr;

    // Slot in the instancer's instance array. The emitted column is the full array placed by index,
    // so this is a scatter target, not a sequence position.
    ::physx::PxU32 index = 0;

    // The prototype's own transform, inverted. Constant while the instance exists; instances of one
    // prototype each carry an identical copy.
    ::physx::PxMat44d protoInverse{ ::physx::PxIdentity };
};

// A point instancer and its instances, assembled by the caller rather than by walking PxScene: an
// instancer is not a PhysX object, and the instancer <-> instance relationship lives in the object
// database (InternalActor::mInstanceKey / mInstanceIndex / mProtoTransformInverse), which this layer
// does not see.
//
// No `path` (this view fills columns by index and never names a prim) and no `subspace`: an instance
// pose is expressed in its instancer's frame, so dividing out the instancer's world transform already
// accounts for the subspace origin -- subtracting it again would apply the offset twice.
struct PointInstancerEntry
{
    // World transform of the instancer prim, inverted -- the frame instance poses are expressed in.
    //
    // Recomputed by the caller per read, never cached: it comes from a stage query, so a moving
    // instancer changes it without creating or retiring any PhysX record, and the object-lifetime
    // epoch cannot see it. Caching behind that epoch would serve stale positions with no error.
    ::physx::PxMat44d worldInverse{ ::physx::PxIdentity };

    // Instance-array length to emit: max index + 1, not instances.size(). The column is the full
    // array placed by index, so a sparse instancer still publishes every slot.
    //
    // Also the bound both column fills reject against, and the reader must size each instancer's
    // destination sub-range from the same quantity -- otherwise a write lands silently inside the
    // next instancer's range. The reader sets both from one `acc.maxIndex` on adjacent lines; there
    // is no check, so keep them adjacent.
    ::physx::PxU32 arrayLength = 0;

    std::vector<PointInstance> instances;
};

struct DeformableBodyEntry
{
    ::physx::PxDeformableBody* body = nullptr;

    std::string path;
    std::string simMeshPath;
    std::string collMeshPath;

    Subspace* subspace = nullptr;
    std::vector<::physx::PxU32> simIndices;
    std::vector<::physx::PxVec3> restPositions;
    std::vector<::physx::PxU32> collIndices;
};

struct DeformableMaterialEntry
{
    ::physx::PxDeformableMaterial* material = nullptr;
    std::string path;
    bool isSurface = false; // true when material is PxDeformableSurfaceMaterial
};

struct SdfShapeEntry
{
    std::string path;
    Subspace* subspace = nullptr;
    ::physx::PxU32 numSamplePoints = 0;
    ::physx::PxShape* shape;
};

struct DofImpl
{
    ::physx::PxArticulationJointReducedCoordinate* joint = nullptr;
    ::physx::PxArticulationAxis::Enum axis = ::physx::PxArticulationAxis::eTWIST;
    ::physx::PxU32 isEnvelopeUsed = 0;
    std::string path;
};

// One vehicle in a superset view. A vehicle is not a PhysX object: PxRigidDynamic is only its
// chassis, and the wheel state (suspension travel, steer, per-wheel local pose) lives in omni's
// InternalVehicle, so the entry holds that and reaches through it.
//
// Host-only by necessity: suspension and sticky-tire constraints are custom PxConstraints with a CPU
// solver-prep function, which PhysX refuses on a scene with PxSceneFlag::eENABLE_DIRECT_GPU_API, so
// vehicle state can never be device-resident. No `path`: rows map to the wheel prims, which the
// reader carries as ObjectKeys.
struct VehicleEntry
{
    omni::physx::internal::InternalVehicle* vehicle = nullptr;
    ::physx::PxRigidDynamic* actor = nullptr; // the chassis
    ::physx::PxU32 numWheels = 0;
};

// represents an articulation instance
struct ArticulationEntry
{
    // common type info that can be shared by multiple articulations
    const ArticulationMetatype* metatype = nullptr;

    ::physx::PxArticulationReducedCoordinate* arti = nullptr;

    std::string path;

    Subspace* subspace = nullptr;

    ::physx::PxU32 numLinks = 0;
    ::physx::PxU32 numDofs = 0;
    ::physx::PxU32 numShapes = 0;
    ::physx::PxU32 numFixedTendons = 0;
    ::physx::PxU32 numSpatialTendons = 0;

    std::vector<::physx::PxArticulationLink*> links;
    std::vector<std::string> linkPaths;
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
    // Source-native display path; NOT the encoding used for contact matching below.
    std::string path;
    std::vector<std::string> filterPaths;
    // Canonical source identities aligned with filterPaths. Keep these through
    // view construction so backend consumers do not have to reconstruct keys
    // from the legacy contact-report ids below.
    std::vector<omni::physics::parse::ObjectKey> filterKeys;

    // referentId/filterIndexMap intentionally stay in the legacy ObjectKey.handle
    // encoding (see asInt()/keyFromLegacyId() below) rather than becoming a distinct
    // ObjectKey-typed field, because they must match ContactEventHeader/ContactData's
    // actor/collider fields (ContactReport.cpp), which use the same legacy uint64_t id
    // for wire compatibility with existing consumers.
    // Retyping these without also retyping ContactEvent.h is a real, separate,
    // riskier follow-up, not part of this identity/display-field cleanup.
    uint64_t referentId = 0; // legacy ObjectKey.handle-encoded identity, matches contact headers

    ::physx::PxArticulationLink* link = nullptr; // set if referent is an articulation link
    ::physx::PxRigidDynamic* rd = nullptr; // set if referent is a rigid dynamic
    ::physx::PxShape* shape = nullptr; // set if referent is a rigid shape
    uint32_t nameID = 0xffffffff;
    std::unordered_map<uint64_t, uint32_t> filterIndexMap; // legacy ObjectKey.handle-keyed, see referentId above

    Subspace* subspace = nullptr;
};

// PointSetReadEntry deliberately lives in tensors/gpu/GpuPointSetReadView.h, not here: this file is
// USD-coupled through PXR_NS::SdfPath and that header must stay USD-free.

// asInt()/keyFromLegacyId() are kept ONLY for the legacy contact-header identity
// encoding above (RigidContactSensorEntry::referentId/filterIndexMap and their
// consumers in CpuSimulationData.cpp/CpuRigidContactView.cpp/GpuRigidContactView.cpp/
// GpuSimulationData.cpp/BaseRigidContactView.cpp) -- NOT used for this file's other,
// std::string-typed identity/display path fields. The legacy uint64_t id IS the raw
// omni::physics::parse::ObjectKey handle directly, both configs now (formerly a
// USD-only asInt(SdfPath) bit-cast; see ContactReport.cpp's keyToLegacyPathInt for the
// matching wire-encode). Callers that need a display string for such an id must resolve
// it through AttachedStage::textFor(ObjectKey), not a path-decode of the id itself.
inline uint64_t asInt(omni::physics::parse::ObjectKey key)
{
    return key.handle;
}

inline omni::physics::parse::ObjectKey keyFromLegacyId(uint64_t id)
{
    return omni::physics::parse::ObjectKey{ id };
}

} // namespace tensors
} // namespace physx
} // namespace omni
