// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-NATIVE-SCHEMA-001
// @covers AC-3
// @implements REQ-COMP-JOINT-001
// @covers AC-1 AC-2 AC-3
// @implements REQ-COMP-GEOMETRY-001
// @covers AC-1 AC-3
#pragma once
// C++17, exact matching SDK/compiler/runtime required. No USD or Python types.
#include <ovphysx_companion/native_compatibility.h>
#include <ovphysx_companion/native_exports.h>
#include <PxPhysicsAPI.h>
#include <geometry/PxGeometryQuery.h>
#include <extensions/PxMassProperties.h>
#include <omni/physx/NativeParameters.h>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace ovphysx::companion::native
{
using omni::physx::native::ParameterKind;
using omni::physx::native::ParameterSpec;
using omni::physx::native::ParameterValue;

// A read-only view valid only during create; copy values needed by later callbacks.
struct ObjectContext
{
    uint64_t attachment;
    uint64_t key;
    const ParameterValue* parameters;
    uint32_t parameterCount;
};

// Supported non-inline PhysX operations resolve through the owning runtime.
// Do not link PhysX static libraries into a plugin or create a second foundation.
struct HostApi
{
    uint32_t size;
    uint32_t version;
    decltype(&physx::PxGeometryQuery::raycast) raycast;
    decltype(&physx::PxGeometryQuery::overlap) overlap;
    decltype(&physx::PxGeometryQuery::sweep) sweep;
};

// Every non-null create result receives one release, including rollback. Callbacks
// must not throw or block on physics/owner operations. No interpreter callbacks.
// Returned shader data has dataSize bytes and dataAlignment alignment, is owned
// by the object until release, and is copied by the runtime before solver work.
struct JointCallbacks
{
    void* (*create)(const ObjectContext&, physx::PxRigidActor*, const physx::PxTransform&,
                    physx::PxRigidActor*, const physx::PxTransform&, uint32_t& flags, void* context) noexcept = nullptr;
    void (*release)(void* object, void* context) noexcept = nullptr;
    void* (*prepare)(void* object) noexcept = nullptr;
    const void* (*constantBlock)(void* object) noexcept = nullptr;
    void (*comShift)(void* object, uint32_t actor) noexcept = nullptr;
    void (*originShift)(void* object, const physx::PxVec3&) noexcept = nullptr;
    physx::PxConstraintSolverPrep solverPrep = nullptr;
    uint32_t dataSize = 0;
    uint32_t dataAlignment = 16;
    // Optional explicit refresh at an idle owner. On rejection, leave state unchanged.
    bool (*refresh)(const ObjectContext&, void* object, void* context) noexcept = nullptr;
};

struct GeometryCallbacks
{
    void* (*create)(const ObjectContext&, const physx::PxCustomGeometry::Type&, void* context) noexcept = nullptr;
    void (*release)(void* object, void* context) noexcept = nullptr;
    physx::PxBounds3 (*bounds)(const physx::PxGeometry&, void*) = nullptr;
    bool (*contacts)(const physx::PxGeometry&, const physx::PxGeometry&, const physx::PxTransform&,
                     const physx::PxTransform&, float, float, float, physx::PxContactBuffer&, void*, void*) = nullptr;
    void (*mass)(const physx::PxGeometry&, physx::PxMassProperties&, void*) = nullptr;
    bool (*manifold)(const physx::PxGeometry&, float&, void*) = nullptr;
    uint32_t (*raycast)(const physx::PxVec3&, const physx::PxVec3&, const physx::PxGeometry&,
                        const physx::PxTransform&, float, physx::PxHitFlags, uint32_t,
                        physx::PxGeomRaycastHit*, uint32_t, physx::PxRaycastThreadContext*, void*) = nullptr;
    bool (*overlap)(const physx::PxGeometry&, const physx::PxTransform&, const physx::PxGeometry&,
                    const physx::PxTransform&, physx::PxOverlapThreadContext*, void*) = nullptr;
    bool (*sweep)(const physx::PxVec3&, float, const physx::PxGeometry&, const physx::PxTransform&,
                  const physx::PxGeometry&, const physx::PxTransform&, physx::PxGeomSweepHit&,
                  physx::PxHitFlags, float, physx::PxSweepThreadContext*, void*) = nullptr;
    void (*visualize)(const physx::PxGeometry&, const physx::PxTransform&, physx::PxRenderOutput&,
                      const physx::PxBounds3&, void*) = nullptr;
};

enum class Kind : uint32_t { Joint = 1, Geometry = 2 };
// Export one constant data symbol named ovphysx_companion_plugin_v1. One module
// registers one schema. start is called only after complete descriptor validation;
// stop runs after retirement, before dlclose. Module constructors must be inert.
// A null start result declines registration (no stop); otherwise stop runs once.
struct Plugin
{
    uint32_t size = sizeof(Plugin);
    uint32_t version = 1;
    Compatibility compatibility;
    Kind kind = Kind::Joint;
    uint32_t reserved = 0;
    const char* schema = nullptr;
    const ParameterSpec* parameters = nullptr;
    uint32_t parameterCount = 0;
    uint32_t capabilities = 0; // v1: reserved, must be zero
    void* (*start)(const HostApi&) noexcept = nullptr;
    void (*stop)(void*) noexcept = nullptr;
    JointCallbacks joint;
    GeometryCallbacks geometry;
};
static_assert(std::is_standard_layout_v<Plugin>);
}
