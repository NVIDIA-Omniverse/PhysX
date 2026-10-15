// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Test helpers for the OVPHYSX_QUERY_TARGET_DEBUG_RENDER session. The removed
// public debug-render entry points are not called here; these wrappers write
// and read the current session attributes.

#pragma once

#include "ovphysx/ovphysx.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace {

ovphysx_result_t debugRenderFinish(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_result_t status{};
    status.status = task.status;
    if (task.status != OVPHYSX_API_SUCCESS)
        return status;
    ovphysx_wait_task_result_t waited{};
    status = ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &waited);
    ovphysx_destroy_wait_task_result(&waited);
    return status;
}

const ovx_string_or_token_t* debugRenderParamName(uint32_t param)
{
    switch (param)
    {
    case OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES: return &OVPHYSX_NAME_DEBUG_WORLD_AXES;
    case OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES: return &OVPHYSX_NAME_DEBUG_BODY_AXES;
    case OVPHYSX_DEBUG_RENDER_PARAM_BODY_MASS_AXES: return &OVPHYSX_NAME_DEBUG_BODY_MASS_AXES;
    case OVPHYSX_DEBUG_RENDER_PARAM_BODY_LINEAR_VELOCITY: return &OVPHYSX_NAME_DEBUG_BODY_LINEAR_VELOCITY;
    case OVPHYSX_DEBUG_RENDER_PARAM_BODY_ANGULAR_VELOCITY: return &OVPHYSX_NAME_DEBUG_BODY_ANGULAR_VELOCITY;
    case OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT: return &OVPHYSX_NAME_DEBUG_CONTACT_POINT;
    case OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL: return &OVPHYSX_NAME_DEBUG_CONTACT_NORMAL;
    case OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_ERROR: return &OVPHYSX_NAME_DEBUG_CONTACT_ERROR;
    case OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_IMPULSE: return &OVPHYSX_NAME_DEBUG_CONTACT_IMPULSE;
    case OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_POINT: return &OVPHYSX_NAME_DEBUG_FRICTION_POINT;
    case OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_NORMAL: return &OVPHYSX_NAME_DEBUG_FRICTION_NORMAL;
    case OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_IMPULSE: return &OVPHYSX_NAME_DEBUG_FRICTION_IMPULSE;
    case OVPHYSX_DEBUG_RENDER_PARAM_ACTOR_AXES: return &OVPHYSX_NAME_DEBUG_ACTOR_AXES;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AABBS: return &OVPHYSX_NAME_DEBUG_COLLISION_AABBS;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_SHAPES: return &OVPHYSX_NAME_DEBUG_COLLISION_SHAPES;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AXES: return &OVPHYSX_NAME_DEBUG_COLLISION_AXES;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_COMPOUNDS: return &OVPHYSX_NAME_DEBUG_COLLISION_COMPOUNDS;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_FACE_NORMALS: return &OVPHYSX_NAME_DEBUG_COLLISION_FACE_NORMALS;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_EDGES: return &OVPHYSX_NAME_DEBUG_COLLISION_EDGES;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_STATIC_PRUNER: return &OVPHYSX_NAME_DEBUG_COLLISION_STATIC_PRUNER;
    case OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_DYNAMIC_PRUNER: return &OVPHYSX_NAME_DEBUG_COLLISION_DYNAMIC_PRUNER;
    case OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LOCAL_FRAMES: return &OVPHYSX_NAME_DEBUG_JOINT_LOCAL_FRAMES;
    case OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LIMITS: return &OVPHYSX_NAME_DEBUG_JOINT_LIMITS;
    case OVPHYSX_DEBUG_RENDER_PARAM_CULL_BOX: return &OVPHYSX_NAME_DEBUG_CULL_BOX;
    case OVPHYSX_DEBUG_RENDER_PARAM_MBP_REGIONS: return &OVPHYSX_NAME_DEBUG_MBP_REGIONS;
    case OVPHYSX_DEBUG_RENDER_PARAM_SIMULATION_MESH: return &OVPHYSX_NAME_DEBUG_SIMULATION_MESH;
    case OVPHYSX_DEBUG_RENDER_PARAM_SDF: return &OVPHYSX_NAME_DEBUG_SDF;
    default: return nullptr;
    }
}

struct DebugRenderQuery
{
    ovphysx_instance_handle_t handle = 0;
    ovphysx_query_handle_t query = 0;
    ovphysx_result_t status{};

    explicit DebugRenderQuery(ovphysx_instance_handle_t instance)
        : handle(instance)
    {
        status = debugRenderFinish(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr));
    }

    ~DebugRenderQuery()
    {
        if (query)
            (void)debugRenderFinish(handle, ovphysx_release_query(handle, query));
    }
};

ovphysx_result_t debugRenderWriteFloat(ovphysx_instance_handle_t handle,
                                           const ovx_string_or_token_t* attribute,
                                           float value)
{
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    ovphysx_write_handle_t write = 0;
    ovphysx_result_t status = debugRenderFinish(handle, ovphysx_write(handle, query.query, attribute, 1, &write, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    ovphysx_fetched_write_group_t fetched{};
    status = debugRenderFinish(handle, ovphysx_fetch_write_next(handle, write, &fetched));
    const ovstage_map_group_t* group = fetched.group;
    if (status.status == OVPHYSX_API_SUCCESS && group && group->data.tensors && group->data.tensors[0].data)
        *static_cast<float*>(group->data.tensors[0].data) = value;
    else
        status.status = OVPHYSX_API_ERROR;
    if (status.status == OVPHYSX_API_SUCCESS)
    {
        ovstage_cuda_sync_t sync{};
        status = debugRenderFinish(handle, ovphysx_commit_group(handle, write, group, sync));
    }
    (void)debugRenderFinish(handle, ovphysx_release_write(handle, write));
    return status;
}

ovphysx_result_t debug_render_enable(ovphysx_instance_handle_t handle, bool enable)
{
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    ovphysx_write_handle_t write = 0;
    ovphysx_result_t status =
        debugRenderFinish(handle, ovphysx_write(handle, query.query, &OVPHYSX_NAME_DEBUG_ENABLED, 1, &write, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    ovphysx_fetched_write_group_t fetched{};
    status = debugRenderFinish(handle, ovphysx_fetch_write_next(handle, write, &fetched));
    const ovstage_map_group_t* group = fetched.group;
    if (status.status == OVPHYSX_API_SUCCESS && group && group->data.tensors && group->data.tensors[0].data)
        *static_cast<uint8_t*>(group->data.tensors[0].data) = enable ? 1 : 0;
    else
        status.status = OVPHYSX_API_ERROR;
    if (status.status == OVPHYSX_API_SUCCESS)
    {
        ovstage_cuda_sync_t sync{};
        status = debugRenderFinish(handle, ovphysx_commit_group(handle, write, group, sync));
    }
    (void)debugRenderFinish(handle, ovphysx_release_write(handle, write));
    return status;
}

ovphysx_result_t debug_render_set_parameter(ovphysx_instance_handle_t handle, uint32_t param, float value)
{
    ovphysx_result_t status{};
    const ovx_string_or_token_t* name = debugRenderParamName(param);
    if (!name)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    return debugRenderWriteFloat(handle, name, value);
}

ovphysx_result_t debug_render_get_parameter(ovphysx_instance_handle_t handle, uint32_t param, float* out_value)
{
    ovphysx_result_t status{};
    if (!out_value || !debugRenderParamName(param))
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    *out_value = 0.0f;
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    const ovx_string_or_token_t* attribute = debugRenderParamName(param);
    ovphysx_read_handle_t read = 0;
    status = debugRenderFinish(handle, ovphysx_read(handle, query.query, attribute, 1, &read, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    const ovstage_read_group_t* group = nullptr;
    status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &group));
    if (status.status == OVPHYSX_API_SUCCESS && group && group->data.tensors && group->data.tensors[0].data)
        *out_value = *static_cast<const float*>(group->data.tensors[0].data);
    else
        status.status = OVPHYSX_API_ERROR;
    if (group)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, group->read_group_id));
    (void)debugRenderFinish(handle, ovphysx_release_read(handle, read));
    return status;
}

ovphysx_result_t debug_render_set_scale(ovphysx_instance_handle_t handle, float scale)
{
    ovphysx_result_t status{};
    if (!std::isfinite(scale) || scale < 0.0f)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    return debugRenderWriteFloat(handle, &OVPHYSX_NAME_DEBUG_SCALE, scale);
}

ovphysx_result_t debug_render_get_scale(ovphysx_instance_handle_t handle, float* out_scale)
{
    ovphysx_result_t status{};
    if (!out_scale)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    ovphysx_read_handle_t read = 0;
    status = debugRenderFinish(handle, ovphysx_read(handle, query.query, &OVPHYSX_NAME_DEBUG_SCALE, 1, &read, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    const ovstage_read_group_t* group = nullptr;
    status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &group));
    if (status.status == OVPHYSX_API_SUCCESS && group && group->data.tensors && group->data.tensors[0].data)
        *out_scale = *static_cast<const float*>(group->data.tensors[0].data);
    else
        status.status = OVPHYSX_API_ERROR;
    if (group)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, group->read_group_id));
    (void)debugRenderFinish(handle, ovphysx_release_read(handle, read));
    return status;
}

ovphysx_result_t debug_render_set_culling_box(ovphysx_instance_handle_t handle, const float min3[3], const float max3[3])
{
    ovphysx_result_t status{};
    if (!min3 || !max3)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    ovphysx_write_handle_t write = 0;
    status = debugRenderFinish(handle, ovphysx_write(handle, query.query, &OVPHYSX_NAME_DEBUG_CULLING_BOX, 1, &write, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    ovphysx_fetched_write_group_t fetched{};
    status = debugRenderFinish(handle, ovphysx_fetch_write_next(handle, write, &fetched));
    const ovstage_map_group_t* group = fetched.group;
    if (status.status == OVPHYSX_API_SUCCESS && group && group->data.tensors && group->data.tensors[0].data &&
        group->data.tensors[0].shape)
    {
        float* box = static_cast<float*>(group->data.tensors[0].data);
        box[0] = 1.0f;
        std::memcpy(box + 1, min3, 3 * sizeof(float));
        std::memcpy(box + 4, max3, 3 * sizeof(float));
        ovstage_cuda_sync_t sync{};
        status = debugRenderFinish(handle, ovphysx_commit_group(handle, write, group, sync));
    }
    else
        status.status = OVPHYSX_API_ERROR;
    (void)debugRenderFinish(handle, ovphysx_release_write(handle, write));
    return status;
}

ovphysx_result_t debug_render_set_scope_tokens(ovphysx_instance_handle_t handle, const ovx_primpath_t* tokens, uint32_t count)
{
    ovphysx_result_t status{};
    if (count > 0 && !tokens)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    ovphysx_write_handle_t write = 0;
    status = debugRenderFinish(handle, ovphysx_write(handle, query.query, &OVPHYSX_NAME_DEBUG_SCOPE, 1, &write, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    ovphysx_fetched_write_group_t fetched{};
    status = debugRenderFinish(handle, ovphysx_fetch_write_next(handle, write, &fetched));
    const ovstage_map_group_t* group = fetched.group;
    if (status.status == OVPHYSX_API_SUCCESS && group && group->data.tensors && group->data.tensors[0].shape)
    {
        auto* slots = static_cast<uint64_t*>(group->data.tensors[0].data);
        slots[0] = count;
        if (count > 0 && count <= 4096u)
            std::memcpy(slots + 1, tokens, count * sizeof(uint64_t));
        ovstage_cuda_sync_t sync{};
        status = debugRenderFinish(handle, ovphysx_commit_group(handle, write, group, sync));
    }
    else
        status.status = OVPHYSX_API_ERROR;
    (void)debugRenderFinish(handle, ovphysx_release_write(handle, write));
    return status;
}

std::vector<ovphysx_debug_point_t> g_debugRenderPoints;
std::vector<ovphysx_debug_line_t> g_debugRenderLines;
std::vector<ovphysx_debug_triangle_t> g_debugRenderTriangles;

ovphysx_result_t debug_render_get_points(ovphysx_instance_handle_t handle, const ovphysx_debug_point_t** out_points, uint32_t* out_count)
{
    ovphysx_result_t status{};
    if (!out_points || !out_count)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    *out_points = nullptr;
    *out_count = 0;
    g_debugRenderPoints.clear();
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    const ovx_string_or_token_t attributes[] = {
        OVPHYSX_NAME_DEBUG_POINT_POSITION,
        OVPHYSX_NAME_DEBUG_POINT_COLOR,
    };
    ovphysx_read_handle_t read = 0;
    status = debugRenderFinish(handle, ovphysx_read(handle, query.query, attributes, 2, &read, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    const ovstage_read_group_t* position = nullptr;
    const ovstage_read_group_t* color = nullptr;
    status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &position));
    if (status.status == OVPHYSX_API_SUCCESS)
        status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &color));
    if (status.status == OVPHYSX_API_SUCCESS && position && color && position->data.tensors && position->data.tensors[0].shape)
    {
        const int64_t count = position->data.tensors[0].shape[0];
        const float* xyz = static_cast<const float*>(position->data.tensors[0].data);
        const uint32_t* colors = color->data.tensors ? static_cast<const uint32_t*>(color->data.tensors[0].data) : nullptr;
        g_debugRenderPoints.resize(static_cast<size_t>(count));
        for (int64_t i = 0; i < count; ++i)
        {
            if (xyz)
                std::memcpy(g_debugRenderPoints[static_cast<size_t>(i)].pos, xyz + i * 3, 3 * sizeof(float));
            g_debugRenderPoints[static_cast<size_t>(i)].color = colors ? colors[i] : 0;
        }
    }
    if (position)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, position->read_group_id));
    if (color)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, color->read_group_id));
    (void)debugRenderFinish(handle, ovphysx_release_read(handle, read));
    if (!g_debugRenderPoints.empty())
    {
        *out_points = g_debugRenderPoints.data();
        *out_count = static_cast<uint32_t>(g_debugRenderPoints.size());
    }
    return status;
}

ovphysx_result_t debug_render_get_lines(ovphysx_instance_handle_t handle, const ovphysx_debug_line_t** out_lines, uint32_t* out_count)
{
    ovphysx_result_t status{};
    if (!out_lines || !out_count)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    *out_lines = nullptr;
    *out_count = 0;
    g_debugRenderLines.clear();
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    const ovx_string_or_token_t attributes[] = {
        OVPHYSX_NAME_DEBUG_LINE_POSITION,
        OVPHYSX_NAME_DEBUG_LINE_COLOR,
    };
    ovphysx_read_handle_t read = 0;
    status = debugRenderFinish(handle, ovphysx_read(handle, query.query, attributes, 2, &read, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    const ovstage_read_group_t* position = nullptr;
    const ovstage_read_group_t* color = nullptr;
    status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &position));
    if (status.status == OVPHYSX_API_SUCCESS)
        status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &color));
    if (status.status == OVPHYSX_API_SUCCESS && position && color && position->data.tensors && position->data.tensors[0].shape)
    {
        const int64_t count = position->data.tensors[0].shape[0];
        const float* xyz = static_cast<const float*>(position->data.tensors[0].data);
        const uint32_t* colors = color->data.tensors ? static_cast<const uint32_t*>(color->data.tensors[0].data) : nullptr;
        g_debugRenderLines.resize(static_cast<size_t>(count));
        for (int64_t i = 0; i < count; ++i)
        {
            ovphysx_debug_line_t& line = g_debugRenderLines[static_cast<size_t>(i)];
            if (xyz)
            {
                std::memcpy(line.pos0, xyz + i * 6, 3 * sizeof(float));
                std::memcpy(line.pos1, xyz + i * 6 + 3, 3 * sizeof(float));
            }
            line.color0 = colors ? colors[i * 2] : 0;
            line.color1 = colors ? colors[i * 2 + 1] : 0;
        }
    }
    if (position)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, position->read_group_id));
    if (color)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, color->read_group_id));
    (void)debugRenderFinish(handle, ovphysx_release_read(handle, read));
    if (!g_debugRenderLines.empty())
    {
        *out_lines = g_debugRenderLines.data();
        *out_count = static_cast<uint32_t>(g_debugRenderLines.size());
    }
    return status;
}

ovphysx_result_t debug_render_get_triangles(ovphysx_instance_handle_t handle, const ovphysx_debug_triangle_t** out_triangles, uint32_t* out_count)
{
    ovphysx_result_t status{};
    if (!out_triangles || !out_count)
    {
        status.status = OVPHYSX_API_INVALID_ARGUMENT;
        return status;
    }
    *out_triangles = nullptr;
    *out_count = 0;
    g_debugRenderTriangles.clear();
    DebugRenderQuery query(handle);
    if (query.status.status != OVPHYSX_API_SUCCESS)
        return query.status;
    const ovx_string_or_token_t attributes[] = {
        OVPHYSX_NAME_DEBUG_TRIANGLE_POSITION,
        OVPHYSX_NAME_DEBUG_TRIANGLE_COLOR,
    };
    ovphysx_read_handle_t read = 0;
    status = debugRenderFinish(handle, ovphysx_read(handle, query.query, attributes, 2, &read, nullptr));
    if (status.status != OVPHYSX_API_SUCCESS)
        return status;
    const ovstage_read_group_t* position = nullptr;
    const ovstage_read_group_t* color = nullptr;
    status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &position));
    if (status.status == OVPHYSX_API_SUCCESS)
        status = debugRenderFinish(handle, ovphysx_fetch_read_next(handle, read, &color));
    if (status.status == OVPHYSX_API_SUCCESS && position && color && position->data.tensors && position->data.tensors[0].shape)
    {
        const int64_t count = position->data.tensors[0].shape[0];
        const float* xyz = static_cast<const float*>(position->data.tensors[0].data);
        const uint32_t* colors = color->data.tensors ? static_cast<const uint32_t*>(color->data.tensors[0].data) : nullptr;
        g_debugRenderTriangles.resize(static_cast<size_t>(count));
        for (int64_t i = 0; i < count; ++i)
        {
            ovphysx_debug_triangle_t& triangle = g_debugRenderTriangles[static_cast<size_t>(i)];
            if (xyz)
            {
                std::memcpy(triangle.pos0, xyz + i * 9, 3 * sizeof(float));
                std::memcpy(triangle.pos1, xyz + i * 9 + 3, 3 * sizeof(float));
                std::memcpy(triangle.pos2, xyz + i * 9 + 6, 3 * sizeof(float));
            }
            triangle.color0 = colors ? colors[i * 3] : 0;
            triangle.color1 = colors ? colors[i * 3 + 1] : 0;
            triangle.color2 = colors ? colors[i * 3 + 2] : 0;
        }
    }
    if (position)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, position->read_group_id));
    if (color)
        (void)debugRenderFinish(handle, ovphysx_release_group(handle, read, color->read_group_id));
    (void)debugRenderFinish(handle, ovphysx_release_read(handle, read));
    if (!g_debugRenderTriangles.empty())
    {
        *out_triangles = g_debugRenderTriangles.data();
        *out_count = static_cast<uint32_t>(g_debugRenderTriangles.size());
    }
    return status;
}

} // namespace
