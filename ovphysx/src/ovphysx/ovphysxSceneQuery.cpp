// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Scene query API: raycast, sweep, and overlap against the physics scene.
// Results are stored in an internal hit buffer per instance.

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"  // g_sidecarEncodeSdfPath
#include <omni/physx/PhysXRuntime.h>

#include <omni/physx/IPhysxSceneQuery.h>
#include <carb/Framework.h>

#include <cmath>
#include <cstddef>

static_assert(sizeof(ovphysx_scene_query_hit_t) == 64,
    "ovphysx_scene_query_hit_t size changed -- update ctypes mirror in _bindings.py");
static_assert(offsetof(ovphysx_scene_query_hit_t, collision) == 0, "collision offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, rigid_body) == 8, "rigid_body offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, proto_index) == 16, "proto_index offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, normal) == 20, "normal offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, position) == 32, "position offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, distance) == 44, "distance offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, face_index) == 48, "face_index offset");
static_assert(offsetof(ovphysx_scene_query_hit_t, material) == 56, "material offset");

// Sidecar encode-sdf-path atomic owned here next to its consumer; loader
// writes it during loadInternalSidecar() via the extern in
// ovphysxInternalInterop.h. (g_sidecarGetPhysXPtr lives in ovphysxPhysXInterop.cpp.)
std::atomic<OvphysxSidecarEncodeSdfPathFn> g_sidecarEncodeSdfPath{nullptr};

namespace {

// SdfPath encoding is resolved in the internal sidecar (which links USD) and
// published by the sidecar loader via g_sidecarEncodeSdfPath.

uint64_t encodeSdfPath(const char* prim_path)
{
    auto fn = g_sidecarEncodeSdfPath.load(std::memory_order_acquire);
    return fn ? fn(prim_path) : 0;
}

// ---- Hit conversion helpers ----

ovphysx_scene_query_hit_t convertLocationHit(const omni::physx::SceneQueryHitLocation& src)
{
    ovphysx_scene_query_hit_t h{};
    h.collision   = src.collision;
    h.rigid_body  = src.rigidBody;
    h.proto_index = src.protoIndex;
    h.normal[0]   = src.normal.x;
    h.normal[1]   = src.normal.y;
    h.normal[2]   = src.normal.z;
    h.position[0] = src.position.x;
    h.position[1] = src.position.y;
    h.position[2] = src.position.z;
    h.distance    = src.distance;
    h.face_index  = src.faceIndex;
    h.material    = src.material;
    return h;
}

ovphysx_scene_query_hit_t convertOverlapHit(const omni::physx::OverlapHit& src)
{
    ovphysx_scene_query_hit_t h{};
    h.collision   = src.collision;
    h.rigid_body  = src.rigidBody;
    h.proto_index = src.protoIndex;
    return h;
}

// ---- Common validation ----

// Validate a SHAPE geometry's prim_path and encode it via the sidecar.
// Returns success() with out_encoded set on success, or a specific error result.
ovphysx_result_t encodeShapePrimPath(const ovphysx_string_t& prim_path, uint64_t& out_encoded)
{
    if (!isValid(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "shape prim_path is NULL or empty");
    if (hasEmbeddedNul(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "shape prim_path contains an embedded NUL byte");

    const std::string prim_path_str = toStdString(prim_path);
    out_encoded = encodeSdfPath(prim_path_str.c_str());
    if (out_encoded == 0)
        return set_error(OVPHYSX_API_ERROR, "failed to encode prim path (internal sidecar not loaded?)");
    return success();
}

ovphysx_result_t validateSceneQueryArgs(
    ovphysx_handle_t handle,
    const ovphysx_scene_query_hit_t** out_hits,
    uint32_t* out_count,
    InstanceData** out_instance)
{
    if (!out_hits || !out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "output pointer is NULL");
    *out_hits = nullptr;
    *out_count = 0;

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    *out_instance = instance;
    return success();
}

omni::physx::IPhysxSceneQuery* acquireSceneQuery()
{
    return omni::physx::runtime::tryGetPhysxSceneQueryInterface();
}

} // anonymous namespace


// ---- Raycast ----

OVPHYSX_API ovphysx_result_t ovphysx_raycast(
    ovphysx_handle_t handle,
    const float origin[3],
    const float direction[3],
    float distance,
    bool both_sides,
    ovphysx_scene_query_mode_t mode,
    const ovphysx_scene_query_hit_t** out_hits,
    uint32_t* out_count)
{
    InstanceData* instance = nullptr;
    {
        auto check = validateSceneQueryArgs(handle, out_hits, out_count, &instance);
        if (check.status != OVPHYSX_API_SUCCESS)
            return check;
    }

    if (!origin || !direction)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "origin or direction is NULL");

    if (distance < 0.0f || !std::isfinite(distance))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "distance must be finite and >= 0");

    auto* sq = acquireSceneQuery();
    if (!sq)
        return set_error(OVPHYSX_API_ERROR, "IPhysxSceneQuery interface not available");

    carb::Float3 o{origin[0], origin[1], origin[2]};
    carb::Float3 d{direction[0], direction[1], direction[2]};

    auto& buf = instance->sceneQueryHitBuffer;
    buf.clear();

    switch (mode)
    {
    case OVPHYSX_SCENE_QUERY_MODE_CLOSEST:
    {
        omni::physx::RaycastHit hit{};
        if (sq->raycastClosest(o, d, distance, hit, both_sides))
            buf.push_back(convertLocationHit(hit));
        break;
    }
    case OVPHYSX_SCENE_QUERY_MODE_ANY:
    {
        if (sq->raycastAny(o, d, distance, both_sides))
        {
            ovphysx_scene_query_hit_t h{};
            buf.push_back(h);
        }
        break;
    }
    case OVPHYSX_SCENE_QUERY_MODE_ALL:
    {
        sq->raycastAll(o, d, distance,
            [&buf](const omni::physx::RaycastHit& hit) -> bool {
                buf.push_back(convertLocationHit(hit));
                return true;
            },
            both_sides);
        break;
    }
    default:
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid scene query mode");
    }

    *out_hits = buf.data();
    *out_count = static_cast<uint32_t>(buf.size());
    return success();
}


// ---- Sweep ----

OVPHYSX_API ovphysx_result_t ovphysx_sweep(
    ovphysx_handle_t handle,
    const ovphysx_scene_query_geometry_desc_t* geometry,
    const float direction[3],
    float distance,
    bool both_sides,
    ovphysx_scene_query_mode_t mode,
    const ovphysx_scene_query_hit_t** out_hits,
    uint32_t* out_count)
{
    InstanceData* instance = nullptr;
    {
        auto check = validateSceneQueryArgs(handle, out_hits, out_count, &instance);
        if (check.status != OVPHYSX_API_SUCCESS)
            return check;
    }

    if (!geometry || !direction)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "geometry or direction is NULL");

    if (distance < 0.0f || !std::isfinite(distance))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "distance must be finite and >= 0");

    auto* sq = acquireSceneQuery();
    if (!sq)
        return set_error(OVPHYSX_API_ERROR, "IPhysxSceneQuery interface not available");

    carb::Float3 dir{direction[0], direction[1], direction[2]};
    auto& buf = instance->sceneQueryHitBuffer;
    buf.clear();

    switch (geometry->type)
    {
    case OVPHYSX_SCENE_QUERY_GEOMETRY_SPHERE:
    {
        float r = geometry->sphere.radius;
        carb::Float3 pos{geometry->sphere.position[0],
                         geometry->sphere.position[1],
                         geometry->sphere.position[2]};
        switch (mode)
        {
        case OVPHYSX_SCENE_QUERY_MODE_CLOSEST:
        {
            omni::physx::SweepHit hit{};
            if (sq->sweepSphereClosest(r, pos, dir, distance, hit, both_sides))
                buf.push_back(convertLocationHit(hit));
            break;
        }
        case OVPHYSX_SCENE_QUERY_MODE_ANY:
        {
            if (sq->sweepSphereAny(r, pos, dir, distance, both_sides))
            {
                ovphysx_scene_query_hit_t h{};
                buf.push_back(h);
            }
            break;
        }
        case OVPHYSX_SCENE_QUERY_MODE_ALL:
        {
            sq->sweepSphereAll(r, pos, dir, distance,
                [&buf](const omni::physx::SweepHit& hit) -> bool {
                    buf.push_back(convertLocationHit(hit));
                    return true;
                },
                both_sides);
            break;
        }
        default:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid scene query mode");
        }
        break;
    }
    case OVPHYSX_SCENE_QUERY_GEOMETRY_BOX:
    {
        carb::Float3 halfExt{geometry->box.half_extent[0],
                             geometry->box.half_extent[1],
                             geometry->box.half_extent[2]};
        carb::Float3 pos{geometry->box.position[0],
                         geometry->box.position[1],
                         geometry->box.position[2]};
        carb::Float4 rot{geometry->box.rotation[0],
                         geometry->box.rotation[1],
                         geometry->box.rotation[2],
                         geometry->box.rotation[3]};
        switch (mode)
        {
        case OVPHYSX_SCENE_QUERY_MODE_CLOSEST:
        {
            omni::physx::SweepHit hit{};
            if (sq->sweepBoxClosest(halfExt, pos, rot, dir, distance, hit, both_sides))
                buf.push_back(convertLocationHit(hit));
            break;
        }
        case OVPHYSX_SCENE_QUERY_MODE_ANY:
        {
            if (sq->sweepBoxAny(halfExt, pos, rot, dir, distance, both_sides))
            {
                ovphysx_scene_query_hit_t h{};
                buf.push_back(h);
            }
            break;
        }
        case OVPHYSX_SCENE_QUERY_MODE_ALL:
        {
            sq->sweepBoxAll(halfExt, pos, rot, dir, distance,
                [&buf](const omni::physx::SweepHit& hit) -> bool {
                    buf.push_back(convertLocationHit(hit));
                    return true;
                },
                both_sides);
            break;
        }
        default:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid scene query mode");
        }
        break;
    }
    case OVPHYSX_SCENE_QUERY_GEOMETRY_SHAPE:
    {
        uint64_t encoded = 0;
        auto shape_check = encodeShapePrimPath(geometry->shape.prim_path, encoded);
        if (shape_check.status != OVPHYSX_API_SUCCESS)
            return shape_check;

        switch (mode)
        {
        case OVPHYSX_SCENE_QUERY_MODE_CLOSEST:
        {
            omni::physx::SweepHit hit{};
            if (sq->sweepShapeClosest(encoded, dir, distance, hit, both_sides))
                buf.push_back(convertLocationHit(hit));
            break;
        }
        case OVPHYSX_SCENE_QUERY_MODE_ANY:
        {
            if (sq->sweepShapeAny(encoded, dir, distance, both_sides))
            {
                ovphysx_scene_query_hit_t h{};
                buf.push_back(h);
            }
            break;
        }
        case OVPHYSX_SCENE_QUERY_MODE_ALL:
        {
            sq->sweepShapeAll(encoded, dir, distance,
                [&buf](const omni::physx::SweepHit& hit) -> bool {
                    buf.push_back(convertLocationHit(hit));
                    return true;
                },
                both_sides);
            break;
        }
        default:
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid scene query mode");
        }
        break;
    }
    default:
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid geometry type");
    }

    *out_hits = buf.data();
    *out_count = static_cast<uint32_t>(buf.size());
    return success();
}


// ---- Overlap ----

OVPHYSX_API ovphysx_result_t ovphysx_overlap(
    ovphysx_handle_t handle,
    const ovphysx_scene_query_geometry_desc_t* geometry,
    ovphysx_scene_query_mode_t mode,
    const ovphysx_scene_query_hit_t** out_hits,
    uint32_t* out_count)
{
    InstanceData* instance = nullptr;
    {
        auto check = validateSceneQueryArgs(handle, out_hits, out_count, &instance);
        if (check.status != OVPHYSX_API_SUCCESS)
            return check;
    }

    if (!geometry)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "geometry is NULL");

    auto* sq = acquireSceneQuery();
    if (!sq)
        return set_error(OVPHYSX_API_ERROR, "IPhysxSceneQuery interface not available");

    if (mode != OVPHYSX_SCENE_QUERY_MODE_CLOSEST &&
        mode != OVPHYSX_SCENE_QUERY_MODE_ANY &&
        mode != OVPHYSX_SCENE_QUERY_MODE_ALL)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid scene query mode");

    auto& buf = instance->sceneQueryHitBuffer;
    buf.clear();

    // Overlap has no geometric "closest" concept, so CLOSEST is treated as ALL.
    // Only ANY changes behaviour (single boolean check instead of full enumeration).
    bool anyHit = (mode == OVPHYSX_SCENE_QUERY_MODE_ANY);

    auto overlapReport = [&buf](const omni::physx::OverlapHit& hit) -> bool {
        buf.push_back(convertOverlapHit(hit));
        return true;
    };

    switch (geometry->type)
    {
    case OVPHYSX_SCENE_QUERY_GEOMETRY_SPHERE:
    {
        float r = geometry->sphere.radius;
        carb::Float3 pos{geometry->sphere.position[0],
                         geometry->sphere.position[1],
                         geometry->sphere.position[2]};
        if (anyHit)
        {
            if (sq->overlapSphereAny(r, pos))
            {
                ovphysx_scene_query_hit_t h{};
                buf.push_back(h);
            }
        }
        else
        {
            sq->overlapSphere(r, pos, overlapReport, false);
        }
        break;
    }
    case OVPHYSX_SCENE_QUERY_GEOMETRY_BOX:
    {
        carb::Float3 halfExt{geometry->box.half_extent[0],
                             geometry->box.half_extent[1],
                             geometry->box.half_extent[2]};
        carb::Float3 pos{geometry->box.position[0],
                         geometry->box.position[1],
                         geometry->box.position[2]};
        carb::Float4 rot{geometry->box.rotation[0],
                         geometry->box.rotation[1],
                         geometry->box.rotation[2],
                         geometry->box.rotation[3]};
        if (anyHit)
        {
            if (sq->overlapBoxAny(halfExt, pos, rot))
            {
                ovphysx_scene_query_hit_t h{};
                buf.push_back(h);
            }
        }
        else
        {
            sq->overlapBox(halfExt, pos, rot, overlapReport, false);
        }
        break;
    }
    case OVPHYSX_SCENE_QUERY_GEOMETRY_SHAPE:
    {
        uint64_t encoded = 0;
        auto shape_check = encodeShapePrimPath(geometry->shape.prim_path, encoded);
        if (shape_check.status != OVPHYSX_API_SUCCESS)
            return shape_check;

        if (anyHit)
        {
            if (sq->overlapShapeAny(encoded))
            {
                ovphysx_scene_query_hit_t h{};
                buf.push_back(h);
            }
        }
        else
        {
            sq->overlapShape(encoded, overlapReport, false);
        }
        break;
    }
    default:
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid geometry type");
    }

    *out_hits = buf.data();
    *out_count = static_cast<uint32_t>(buf.size());
    return success();
}
