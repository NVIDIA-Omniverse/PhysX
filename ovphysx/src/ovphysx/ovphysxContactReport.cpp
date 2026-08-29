// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause


// Pull-based contact report: exposes the Omni PhysX runtime's collected
// contact data through the ovphysx C API as typed struct pointers.

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/ContactEvent.h>

#include <cstddef>

// Compile-time ABI guards: ovphysx defines its own C structs that must be
// layout-identical to the internal omni::physx structs so we can reinterpret_cast
// between them. Size checks catch added/removed fields; offset checks catch
// reordering or type-width changes (e.g. the long -> int64_t stageId migration).
static_assert(sizeof(ovphysx_contact_event_header_t) == sizeof(omni::physx::ContactEventHeader),
    "ovphysx_contact_event_header_t size mismatch -- update ovphysx_types.h to match ContactEvent.h");
static_assert(sizeof(ovphysx_contact_point_t) == sizeof(omni::physx::ContactData),
    "ovphysx_contact_point_t size mismatch -- update ovphysx_types.h to match ContactEvent.h");

static_assert(offsetof(ovphysx_contact_event_header_t, type) == offsetof(omni::physx::ContactEventHeader, type),
    "ovphysx_contact_event_header_t::type offset mismatch");
static_assert(offsetof(ovphysx_contact_event_header_t, stageId) == offsetof(omni::physx::ContactEventHeader, stageId),
    "ovphysx_contact_event_header_t::stageId offset mismatch");
static_assert(offsetof(ovphysx_contact_event_header_t, actor0) == offsetof(omni::physx::ContactEventHeader, actor0),
    "ovphysx_contact_event_header_t::actor0 offset mismatch");
static_assert(offsetof(ovphysx_contact_event_header_t, numContactData) == offsetof(omni::physx::ContactEventHeader, numContactData),
    "ovphysx_contact_event_header_t::numContactData offset mismatch");
static_assert(offsetof(ovphysx_contact_event_header_t, protoIndex1) == offsetof(omni::physx::ContactEventHeader, protoIndex1),
    "ovphysx_contact_event_header_t::protoIndex1 offset mismatch");

static_assert(offsetof(ovphysx_contact_point_t, position) == offsetof(omni::physx::ContactData, position),
    "ovphysx_contact_point_t::position offset mismatch");
static_assert(offsetof(ovphysx_contact_point_t, normal) == offsetof(omni::physx::ContactData, normal),
    "ovphysx_contact_point_t::normal offset mismatch");
static_assert(offsetof(ovphysx_contact_point_t, impulse) == offsetof(omni::physx::ContactData, impulse),
    "ovphysx_contact_point_t::impulse offset mismatch");
static_assert(offsetof(ovphysx_contact_point_t, separation) == offsetof(omni::physx::ContactData, separation),
    "ovphysx_contact_point_t::separation offset mismatch");
static_assert(offsetof(ovphysx_contact_point_t, material1) == offsetof(omni::physx::ContactData, material1),
    "ovphysx_contact_point_t::material1 offset mismatch");

static_assert(sizeof(ovphysx_friction_anchor_t) == sizeof(omni::physx::FrictionAnchor),
    "ovphysx_friction_anchor_t size mismatch -- update ovphysx_types.h to match ContactEvent.h");
static_assert(offsetof(ovphysx_friction_anchor_t, position) == offsetof(omni::physx::FrictionAnchor, position),
    "ovphysx_friction_anchor_t::position offset mismatch");
static_assert(offsetof(ovphysx_friction_anchor_t, impulse) == offsetof(omni::physx::FrictionAnchor, impulse),
    "ovphysx_friction_anchor_t::impulse offset mismatch");


OVPHYSX_API ovphysx_result_t ovphysx_get_contact_report(
    ovphysx_handle_t handle,
    const ovphysx_contact_event_header_t** out_event_headers,
    uint32_t* out_num_event_headers,
    const ovphysx_contact_point_t** out_contact_data,
    uint32_t* out_num_contact_data,
    const ovphysx_friction_anchor_t** out_friction_anchors,
    uint32_t* out_num_friction_anchors)
{
    if (!out_event_headers || !out_num_event_headers || !out_contact_data || !out_num_contact_data)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "output pointer is NULL");

    *out_event_headers = nullptr;
    *out_num_event_headers = 0;
    *out_contact_data = nullptr;
    *out_num_contact_data = 0;
    if (out_friction_anchors)
        *out_friction_anchors = nullptr;
    if (out_num_friction_anchors)
        *out_num_friction_anchors = 0;

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim)
        return set_error(OVPHYSX_API_ERROR, "IPhysxSimulation runtime interface not available");

    const omni::physx::ContactEventHeader* headers = nullptr;
    const omni::physx::ContactData* data = nullptr;
    uint32_t numContactData = 0;
    const omni::physx::FrictionAnchor* frictionAnchors = nullptr;
    uint32_t numFrictionAnchors = 0;

    uint32_t numHeaders = physxSim->getFullContactReport(
        &headers, &data, numContactData, &frictionAnchors, numFrictionAnchors);

    *out_event_headers = reinterpret_cast<const ovphysx_contact_event_header_t*>(headers);
    *out_num_event_headers = numHeaders;
    *out_contact_data = reinterpret_cast<const ovphysx_contact_point_t*>(data);
    *out_num_contact_data = numContactData;

    if (out_friction_anchors)
        *out_friction_anchors = reinterpret_cast<const ovphysx_friction_anchor_t*>(frictionAnchors);
    if (out_num_friction_anchors)
        *out_num_friction_anchors = numFrictionAnchors;

    return success();
}
