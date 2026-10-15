// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-SESSION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-3 AC-5 AC-7
 *
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-CAPI-OBJECTKEY-001
 * @covers AC-1 AC-4
 *
 * The public pointer contract is the physxPointer column (REQ-CAPI-PHYSXPTR-001).
 * This file keeps the sidecar resolver for object-key lookup.
 */

// Sidecar interop: raw PhysX object lookup and ObjectKey resolution.
// Bridges omni::physx types into C-ABI values usable by the main lib.

#include "internal/sidecar/ovphysxInternalInterop.h"
#include "ovphysx/ovphysx_legacy_types.h"
#include "ovphysx/ovphysx_types.h"
#include "ovphysxInternalPhysXAccess.hpp"

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IOvxPhysicsRead.h> // ovstage-native output read symbols + types
#include <omni/physx/IOvxPhysicsWrite.h> // ovstage-native write symbols + types (ADR-0012)
#include <omni/physx/IPhysxVisualization.h>
#include <PxArticulationReducedCoordinate.h>

#include <cstddef>      // offsetof (debug-render struct layout guards)
#include <type_traits>  // is_same (debug-render per-field type guards)
#include <exception>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <string>
#include <unordered_map>

#if !defined(_WIN32)
#    include <dlfcn.h>
#else
#    include <windows.h>
#endif

// Compile-time guards: ovphysx_physx_type_t enum values must stay aligned with
// the internal omni::physx::PhysXType values so that ovphysx_internal_get_physx_ptr
// can forward the integer directly without a translation table.
static_assert(OVPHYSX_PHYSX_TYPE_SCENE           == omni::physx::ePTScene,          "ovphysx_physx_type_t / PhysXType drift: SCENE");
static_assert(OVPHYSX_PHYSX_TYPE_MATERIAL        == omni::physx::ePTMaterial,       "ovphysx_physx_type_t / PhysXType drift: MATERIAL");
static_assert(OVPHYSX_PHYSX_TYPE_SHAPE           == omni::physx::ePTShape,          "ovphysx_physx_type_t / PhysXType drift: SHAPE");
static_assert(OVPHYSX_PHYSX_TYPE_COMPOUND_SHAPE  == omni::physx::ePTCompoundShape,  "ovphysx_physx_type_t / PhysXType drift: COMPOUND_SHAPE");
static_assert(OVPHYSX_PHYSX_TYPE_ACTOR           == omni::physx::ePTActor,          "ovphysx_physx_type_t / PhysXType drift: ACTOR");
static_assert(OVPHYSX_PHYSX_TYPE_JOINT           == omni::physx::ePTJoint,          "ovphysx_physx_type_t / PhysXType drift: JOINT");
static_assert(OVPHYSX_PHYSX_TYPE_CUSTOM_JOINT    == omni::physx::ePTCustomJoint,    "ovphysx_physx_type_t / PhysXType drift: CUSTOM_JOINT");
static_assert(OVPHYSX_PHYSX_TYPE_ARTICULATION    == omni::physx::ePTArticulation,   "ovphysx_physx_type_t / PhysXType drift: ARTICULATION");
static_assert(OVPHYSX_PHYSX_TYPE_LINK            == omni::physx::ePTLink,           "ovphysx_physx_type_t / PhysXType drift: LINK");
static_assert(OVPHYSX_PHYSX_TYPE_LINK_JOINT      == omni::physx::ePTLinkJoint,      "ovphysx_physx_type_t / PhysXType drift: LINK_JOINT");
static_assert(OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM == omni::physx::ePTParticleSystem, "ovphysx_physx_type_t / PhysXType drift: PARTICLE_SYSTEM");
static_assert(OVPHYSX_PHYSX_TYPE_PARTICLE_SET    == omni::physx::ePTParticleSet,    "ovphysx_physx_type_t / PhysXType drift: PARTICLE_SET");
static_assert(OVPHYSX_PHYSX_TYPE_PHYSICS         == omni::physx::ePTPhysics,        "ovphysx_physx_type_t / PhysXType drift: PHYSICS");

static_assert(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK  == omni::physx::kOvxArticulationLink,  "sim object / OvxObjectType drift: LINK");
static_assert(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT == omni::physx::kOvxArticulationJoint, "sim object / OvxObjectType drift: JOINT");
static_assert(OVPHYSX_QUERY_TARGET_PARTICLE_SET       == omni::physx::kOvxParticleSet,       "sim object / OvxObjectType drift: PARTICLE_SET");
static_assert(OVPHYSX_QUERY_TARGET_ARTICULATION       == omni::physx::kOvxArticulation,      "sim object / OvxObjectType drift: ARTICULATION");
static_assert(OVPHYSX_QUERY_TARGET_SCENE              == omni::physx::kOvxScene,             "sim object / OvxObjectType drift: SCENE");
static_assert(OVPHYSX_QUERY_TARGET_PHYSX_MATERIAL     == omni::physx::kOvxPhysxMaterial,     "sim object / OvxObjectType drift: MATERIAL");
static_assert(OVPHYSX_QUERY_TARGET_SHAPE              == omni::physx::kOvxShape,             "sim object / OvxObjectType drift: SHAPE");
static_assert(OVPHYSX_QUERY_TARGET_COMPOUND_SHAPE     == omni::physx::kOvxCompoundShape,     "sim object / OvxObjectType drift: COMPOUND_SHAPE");
static_assert(OVPHYSX_QUERY_TARGET_RIGID_ACTOR        == omni::physx::kOvxRigidActor,        "sim object / OvxObjectType drift: ACTOR");
static_assert(OVPHYSX_QUERY_TARGET_JOINT              == omni::physx::kOvxJoint,             "sim object / OvxObjectType drift: STANDALONE_JOINT");
static_assert(OVPHYSX_QUERY_TARGET_CUSTOM_JOINT       == omni::physx::kOvxCustomJoint,       "sim object / OvxObjectType drift: CUSTOM_JOINT");
static_assert(OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM    == omni::physx::kOvxParticleSystem,    "sim object / OvxObjectType drift: PARTICLE_SYSTEM");
static_assert(OVPHYSX_QUERY_TARGET_PHYSICS            == omni::physx::kOvxPhysics,           "sim object / OvxObjectType drift: PHYSICS");

// Compile-time guards: the ovphysx_debug_*_t C structs must stay byte-for-byte
// layout-compatible with omni::physx::DebugPoint / DebugLine / DebugTriangle because
// the debug-render getters reinterpret the omni.physx buffer without a copy
// (see ovphysxPhysXInterop.cpp). Size checks catch added or removed fields, offset
// checks catch reordering, and the is_same guards below catch an equal-width type
// swap (e.g. uint32 colour -> float) that sizeof/offsetof alone would miss.
static_assert(sizeof(ovphysx_debug_point_t) == sizeof(omni::physx::DebugPoint), "ovphysx_debug_point_t / DebugPoint size drift");
static_assert(offsetof(ovphysx_debug_point_t, pos)   == offsetof(omni::physx::DebugPoint, mPos),   "debug_point pos offset drift");
static_assert(offsetof(ovphysx_debug_point_t, color) == offsetof(omni::physx::DebugPoint, mColor), "debug_point color offset drift");

static_assert(sizeof(ovphysx_debug_line_t) == sizeof(omni::physx::DebugLine), "ovphysx_debug_line_t / DebugLine size drift");
static_assert(offsetof(ovphysx_debug_line_t, pos0)   == offsetof(omni::physx::DebugLine, mPos0),   "debug_line pos0 offset drift");
static_assert(offsetof(ovphysx_debug_line_t, color0) == offsetof(omni::physx::DebugLine, mColor0), "debug_line color0 offset drift");
static_assert(offsetof(ovphysx_debug_line_t, pos1)   == offsetof(omni::physx::DebugLine, mPos1),   "debug_line pos1 offset drift");
static_assert(offsetof(ovphysx_debug_line_t, color1) == offsetof(omni::physx::DebugLine, mColor1), "debug_line color1 offset drift");

static_assert(sizeof(ovphysx_debug_triangle_t) == sizeof(omni::physx::DebugTriangle), "ovphysx_debug_triangle_t / DebugTriangle size drift");
static_assert(offsetof(ovphysx_debug_triangle_t, pos0)   == offsetof(omni::physx::DebugTriangle, mPos0),   "debug_triangle pos0 offset drift");
static_assert(offsetof(ovphysx_debug_triangle_t, color0) == offsetof(omni::physx::DebugTriangle, mColor0), "debug_triangle color0 offset drift");
static_assert(offsetof(ovphysx_debug_triangle_t, pos1)   == offsetof(omni::physx::DebugTriangle, mPos1),   "debug_triangle pos1 offset drift");
static_assert(offsetof(ovphysx_debug_triangle_t, color1) == offsetof(omni::physx::DebugTriangle, mColor1), "debug_triangle color1 offset drift");
static_assert(offsetof(ovphysx_debug_triangle_t, pos2)   == offsetof(omni::physx::DebugTriangle, mPos2),   "debug_triangle pos2 offset drift");
static_assert(offsetof(ovphysx_debug_triangle_t, color2) == offsetof(omni::physx::DebugTriangle, mColor2), "debug_triangle color2 offset drift");

// Per-field type guards: pin the omni-side field types so an equal-width substitution
// (which size/offset checks cannot see) fails the build. carb::Float3 is itself pinned
// to three floats so a Float3 component flip (float -> int) is caught too.
static_assert(std::is_same<decltype(carb::Float3::x), float>::value, "carb::Float3 component is no longer float");
static_assert(sizeof(omni::physx::DebugPoint) == sizeof(ovphysx_debug_point_t), "DebugPoint layout drift");
static_assert(sizeof(omni::physx::DebugLine) == sizeof(ovphysx_debug_line_t), "DebugLine layout drift");
static_assert(sizeof(omni::physx::DebugTriangle) == sizeof(ovphysx_debug_triangle_t), "DebugTriangle layout drift");
static_assert(std::is_same<decltype(omni::physx::DebugPoint::mPos), carb::Float3>::value,     "DebugPoint::mPos type drift");
static_assert(std::is_same<decltype(omni::physx::DebugPoint::mColor), uint32_t>::value,       "DebugPoint::mColor type drift");
static_assert(std::is_same<decltype(omni::physx::DebugLine::mPos0), carb::Float3>::value,     "DebugLine::mPos0 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugLine::mColor0), uint32_t>::value,       "DebugLine::mColor0 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugLine::mPos1), carb::Float3>::value,     "DebugLine::mPos1 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugLine::mColor1), uint32_t>::value,       "DebugLine::mColor1 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugTriangle::mPos0), carb::Float3>::value, "DebugTriangle::mPos0 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugTriangle::mColor0), uint32_t>::value,   "DebugTriangle::mColor0 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugTriangle::mPos1), carb::Float3>::value, "DebugTriangle::mPos1 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugTriangle::mColor1), uint32_t>::value,   "DebugTriangle::mColor1 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugTriangle::mPos2), carb::Float3>::value, "DebugTriangle::mPos2 type drift");
static_assert(std::is_same<decltype(omni::physx::DebugTriangle::mColor2), uint32_t>::value,   "DebugTriangle::mColor2 type drift");

// The public ovphysx_debug_render_parameter_t enum must mirror the internal
// omni::physx::PhysXVisualizationParameter values one-for-one. The forward in
// internal_set_visualization_parameter is a raw static_cast<PhysXVisualizationParameter>
// with no translation table, so every enumerator is asserted. A reorder inside the
// range that preserved count and endpoints would otherwise mis-map.
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_NONE                     == (int)omni::physx::eNone,                  "debug_render_parameter drift: NONE");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES               == (int)omni::physx::eWorldAxes,             "debug_render_parameter drift: WORLD_AXES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES                == (int)omni::physx::eBodyAxes,              "debug_render_parameter drift: BODY_AXES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_BODY_MASS_AXES           == (int)omni::physx::eBodyMassAxes,          "debug_render_parameter drift: BODY_MASS_AXES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_BODY_LINEAR_VELOCITY     == (int)omni::physx::eBodyLinearVelocity,    "debug_render_parameter drift: BODY_LINEAR_VELOCITY");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_BODY_ANGULAR_VELOCITY    == (int)omni::physx::eBodyAngularVelocity,   "debug_render_parameter drift: BODY_ANGULAR_VELOCITY");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT            == (int)omni::physx::eContactPoint,          "debug_render_parameter drift: CONTACT_POINT");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL           == (int)omni::physx::eContactNormal,         "debug_render_parameter drift: CONTACT_NORMAL");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_ERROR            == (int)omni::physx::eContactError,          "debug_render_parameter drift: CONTACT_ERROR");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_IMPULSE          == (int)omni::physx::eContactImpulse,        "debug_render_parameter drift: CONTACT_IMPULSE");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_POINT           == (int)omni::physx::eFrictionPoint,         "debug_render_parameter drift: FRICTION_POINT");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_NORMAL          == (int)omni::physx::eFrictionNormal,        "debug_render_parameter drift: FRICTION_NORMAL");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_IMPULSE         == (int)omni::physx::eFrictionImpulse,       "debug_render_parameter drift: FRICTION_IMPULSE");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_ACTOR_AXES               == (int)omni::physx::eActorAxes,             "debug_render_parameter drift: ACTOR_AXES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AABBS          == (int)omni::physx::eCollisionAABBs,        "debug_render_parameter drift: COLLISION_AABBS");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_SHAPES         == (int)omni::physx::eCollisionShapes,       "debug_render_parameter drift: COLLISION_SHAPES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AXES           == (int)omni::physx::eCollisionAxes,         "debug_render_parameter drift: COLLISION_AXES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_COMPOUNDS      == (int)omni::physx::eCollisionCompounds,    "debug_render_parameter drift: COLLISION_COMPOUNDS");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_FACE_NORMALS   == (int)omni::physx::eCollisionFaceNormals,  "debug_render_parameter drift: COLLISION_FACE_NORMALS");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_EDGES          == (int)omni::physx::eCollisionEdges,        "debug_render_parameter drift: COLLISION_EDGES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_STATIC_PRUNER  == (int)omni::physx::eCollisionStaticPruner, "debug_render_parameter drift: COLLISION_STATIC_PRUNER");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_DYNAMIC_PRUNER == (int)omni::physx::eCollisionDynamicPruner,"debug_render_parameter drift: COLLISION_DYNAMIC_PRUNER");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LOCAL_FRAMES       == (int)omni::physx::eJointLocalFrames,      "debug_render_parameter drift: JOINT_LOCAL_FRAMES");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LIMITS             == (int)omni::physx::eJointLimits,           "debug_render_parameter drift: JOINT_LIMITS");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_CULL_BOX                 == (int)omni::physx::eCullBox,               "debug_render_parameter drift: CULL_BOX");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_MBP_REGIONS              == (int)omni::physx::eMBPRegions,            "debug_render_parameter drift: MBP_REGIONS");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_SIMULATION_MESH          == (int)omni::physx::eSimulationMesh,        "debug_render_parameter drift: SIMULATION_MESH");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_SDF                      == (int)omni::physx::eSDF,                   "debug_render_parameter drift: SDF");
static_assert((int)OVPHYSX_DEBUG_RENDER_PARAM_COUNT                    == (int)omni::physx::ePhysXNumValues,        "debug_render_parameter COUNT must equal ePhysXNumValues");

// The public ovphysx output-read surface uses ovstage's own types directly
// (ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t), so there
// is no ovphysx mirror enum to keep in lock-step.

// The runtime's OvxReadStatus must map cleanly onto the 1 / 0 / -1 convention
// ovphysx_internal_fetch_read_next returns.
static_assert((int)omni::physx::kOvxReadStatusOk != (int)omni::physx::kOvxReadStatusEndOfIteration, "OvxReadStatus ok/eof collision");
static_assert((int)omni::physx::kOvxReadStatusError != (int)omni::physx::kOvxReadStatusOk, "OvxReadStatus ok/error collision");

// try/catch guards the extern "C" boundary so C++ exceptions do not escape into C callers.
// The enum range is not validated here because PhysX returns nullptr for unknown types.
// The selector/type relationship is validated below as defense in depth.
OVPHYSX_INTERNAL_API void* ovphysx_internal_get_physx_ptr(uint64_t attach, const char* prim_path, int physx_type)
{
    if (!prim_path)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr called with null prim_path");
        return nullptr;
    }

    const bool physics_lookup = physx_type == static_cast<int>(OVPHYSX_PHYSX_TYPE_PHYSICS);
    const bool empty_path = prim_path[0] == '\0';
    if (physics_lookup != empty_path)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr selector/type mismatch");
        return nullptr;
    }

    try
    {
        omni::physx::IPhysx* physx = ovphysx::internal::sidecar::tryGetInjectedPhysxInterface();
        if (!physx)
        {
            CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr failed to resolve IPhysx runtime interface");
            return nullptr;
        }

        // The runtime maps the invalid key to the empty path reserved for ePTPhysics.
        omni::physics::parse::ObjectKey key{};
        if (!physics_lookup)
        {
            key = physx->resolveObjectKey(attach, prim_path);
            if (!key.valid())
            {
                CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr could not resolve ObjectKey for '%s'", prim_path);
                return nullptr;
            }
        }

        return physx->getPhysXPtr(attach, key, static_cast<omni::physx::PhysXType>(physx_type));
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr caught exception: %s", e.what());
        return nullptr;
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr caught unknown exception");
        return nullptr;
    }
}

// ---- PhysX debug-visualization bridge -> omni::physx::IPhysxVisualization ----
// The sidecar resolves the interface from the owner-injected static runtime
// accessor. All functions are no-ops (and never throw across the C ABI) when
// the interface is unavailable.

// NOTE: the debug-viz master scale eSCALE = viewportGizmoScale * visualizationScale.
// omni::physx treats a non-positive (unset) viewport gizmo scale, the common case in
// headless hosts, as 1.0 when computing eSCALE (PhysXDebugVisualization.cpp). This
// sidecar therefore does not seed the persistent /persistent/app/viewport/gizmo/scale
// setting, which is a process-global side effect ovphysx should not own.

OVPHYSX_INTERNAL_API void ovphysx_internal_enable_visualization(uint64_t attach, bool enable)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->enableVisualization(attach, enable);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: enable_visualization threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter(uint64_t attach, uint32_t param, bool on)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->setVisualizationParameter(attach, static_cast<omni::physx::PhysXVisualizationParameter>(param), on);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_parameter threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter_value(uint64_t attach, uint32_t param, float value)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        // ABI-appended member: an older omni.physx predating the API leaves
        // the pointer unset, same guard as the scope setter.
        if (visualization && visualization->setVisualizationParameterValue)
        {
            visualization->setVisualizationParameterValue(attach, static_cast<omni::physx::PhysXVisualizationParameter>(param), value);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_parameter_value threw");
    }
}

OVPHYSX_INTERNAL_API bool ovphysx_internal_set_visualization_scope_tokens(uint64_t attach, const ovx_primpath_t* tokens, uint32_t count)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (!visualization || !visualization->setVisualizationScopeTokens)
            return false;
        return visualization->setVisualizationScopeTokens(attach, tokens, count);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_scope_tokens threw");
        return false;
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_scale(uint64_t attach, float scale)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->setVisualizationScale(attach, scale);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_scale threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_culling_box(uint64_t attach, const float* min3, const float* max3)
{
    if (!min3 || !max3)
        return;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->setVisualizationCullingBox(attach, carb::Float3{ min3[0], min3[1], min3[2] },
                                                        carb::Float3{ max3[0], max3[1], max3[2] });
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_culling_box threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_points(uint64_t attach, const void** out, uint32_t* count)
{
    if (out) *out = nullptr;
    if (count) *count = 0;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            if (out) *out = visualization->getPoints(attach);
            if (count) *count = visualization->getNbPoints(attach);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: get_debug_points threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_lines(uint64_t attach, const void** out, uint32_t* count)
{
    if (out) *out = nullptr;
    if (count) *count = 0;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            if (out) *out = visualization->getLines(attach);
            if (count) *count = visualization->getNbLines(attach);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: get_debug_lines threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_triangles(uint64_t attach, const void** out, uint32_t* count)
{
    if (out) *out = nullptr;
    if (count) *count = 0;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            if (out) *out = visualization->getTriangles(attach);
            if (count) *count = visualization->getNbTriangles(attach);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: get_debug_triangles threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_debug_render_apply(
    const ovphysx_internal_debug_render_state_t* state)
{
    if (!state)
        return;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (!visualization || !visualization->applyDebugRenderForAttach)
            return;
        omni::physx::DebugRenderAttachRequest request{};
        request.attachHandle = state->attach_handle;
        request.enabled = state->enabled != 0;
        request.scale = state->scale;
        request.params = state->params;
        request.paramCount = OVPHYSX_DEBUG_RENDER_PARAM_COUNT;
        request.hasCullingBox = state->has_culling_box != 0;
        for (int axis = 0; axis < 3; ++axis)
        {
            request.cullMin[axis] = state->cull_min[axis];
            request.cullMax[axis] = state->cull_max[axis];
        }
        request.scopeTokens = reinterpret_cast<const ovx_primpath_t*>(state->scope_tokens);
        request.scopeCount = state->scope_count;
        visualization->applyDebugRenderForAttach(&request);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: debug_render_apply threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_debug_render_copy(
    uint64_t attach_handle,
    void* points, uint32_t* point_count,
    void* lines, uint32_t* line_count,
    void* triangles, uint32_t* triangle_count)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (!visualization || !visualization->copyDebugPrimitivesForAttach)
        {
            if (point_count) *point_count = 0;
            if (line_count) *line_count = 0;
            if (triangle_count) *triangle_count = 0;
            return;
        }
        visualization->copyDebugPrimitivesForAttach(
            attach_handle,
            static_cast<omni::physx::DebugPoint*>(points), point_count,
            static_cast<omni::physx::DebugLine*>(lines), line_count,
            static_cast<omni::physx::DebugTriangle*>(triangles), triangle_count);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: debug_render_copy threw");
        if (point_count) *point_count = 0;
        if (line_count) *line_count = 0;
        if (triangle_count) *triangle_count = 0;
    }
}

// ============================================================================
// Physics output read (ADR-0007)
//
// The runtime exports the ovstage-native read entry points as standalone
// (non-carb) symbols (omni::physx::ovx*), injected here by the main library at
// load. The sidecar owns the read-session bookkeeping and hands back each
// ovstage_read_group_t verbatim. The public surface is ovstage's own type, with
// no ovphysx mirror and no translation.
// ============================================================================

namespace
{
// Read entry points injected by ovphysx_internal_set_ovx_read_accessors().
struct OvxReadFns
{
    decltype(&omni::physx::ovxQuery)            query = nullptr;
    decltype(&omni::physx::ovxQuerySelected)    querySelected = nullptr;
    decltype(&omni::physx::ovxReadAttributes)   read = nullptr;
    decltype(&omni::physx::ovxFetchReadNext)    fetch = nullptr;
    decltype(&omni::physx::ovxReleaseGroup)     releaseGroup = nullptr;
    decltype(&omni::physx::ovxReleaseRead)      releaseRead = nullptr;
    decltype(&omni::physx::ovxReleaseQuery)     releaseQuery = nullptr;
    bool ready() const
    {
        return query && querySelected && read && fetch && releaseGroup &&
               releaseRead && releaseQuery;
    }
};

// Populated once by ovphysx_internal_set_ovx_read_accessors(), which the main
// library's sidecar loader calls at startup with the addresses of its own
// statically linked omni::physx::ovx* read entry points. The sidecar cannot resolve
// them itself: the owner's symbols are hidden and the static layout has no separate
// libomni.physx.plugin.so to dlopen, so the same handshake as
// ovphysx_internal_set_physx_runtime_accessors is used.
OvxReadFns g_ovxRead;

OvxReadFns& ovxReadFns()
{
    return g_ovxRead;
}

// The write half (ADR-0012), injected by the same handshake and for the same reason.
struct OvxWriteFns
{
    decltype(&omni::physx::ovxWriteAttributes) write = nullptr;
    decltype(&omni::physx::ovxFetchWriteNext) fetch = nullptr;
    decltype(&omni::physx::ovxCommitGroup)    commit = nullptr;
    decltype(&omni::physx::ovxReleaseWrite)   releaseWrite = nullptr;
};

OvxWriteFns g_ovxWrite;

// The contact query (ADR-0044), injected by its own setter. Not part of OvxReadFns::ready(): an owner that
// predates it still serves every other read.
decltype(&omni::physx::ovxQueryContacts) g_ovxQueryContacts = nullptr;

OvxWriteFns& ovxWriteFns()
{
    return g_ovxWrite;
}

// Per-read-session storage: live group descriptors retained by read_group_id. The
// node-based map gives each producer-owned `const ovstage_read_group_t*` a stable
// address until release_group. Numeric tensors, maps, mask, and completion event
// point into the runtime read session and remain valid until release_read.
struct ReadSession
{
    std::unordered_map<uint64_t, ovstage_read_group_t> liveGroups; // read_group_id -> group
    std::vector<ovphysx_read_attribute_result_t> attributes;
};

std::mutex g_readMutex;
std::unordered_map<uint64_t, ReadSession> g_readSessions; // read -> session
std::mutex g_writeMutex;
std::unordered_map<uint64_t, std::vector<ovphysx_write_attribute_result_t>> g_writeResults;

} // namespace

// Owner-injected addresses of the statically linked omni::physx::ovx* read entry
// points. Passed as void* so the shared sidecar header stays free of the ovx/ovstage
// read types, and cast back to the decltype pointer types here. See g_ovxRead.
// Declared extern "C" so the loader can resolve it by plain name via dlsym/GetProcAddress.
// This file includes ovphysxInternalInterop.h rather than ovphysxInternal.h, where the
// C declaration lives, so the linkage has to be stated here too.
extern "C" OVPHYSX_INTERNAL_API void ovphysx_internal_set_ovx_read_accessors(
    void* query, void* querySelected, void* read,
    void* fetch, void* releaseGroup, void* releaseRead, void* releaseQuery)
{
    g_ovxRead.query            = reinterpret_cast<decltype(g_ovxRead.query)>(query);
    g_ovxRead.querySelected    = reinterpret_cast<decltype(g_ovxRead.querySelected)>(querySelected);
    g_ovxRead.read             = reinterpret_cast<decltype(g_ovxRead.read)>(read);
    g_ovxRead.fetch            = reinterpret_cast<decltype(g_ovxRead.fetch)>(fetch);
    g_ovxRead.releaseGroup     = reinterpret_cast<decltype(g_ovxRead.releaseGroup)>(releaseGroup);
    g_ovxRead.releaseRead      = reinterpret_cast<decltype(g_ovxRead.releaseRead)>(releaseRead);
    g_ovxRead.releaseQuery     = reinterpret_cast<decltype(g_ovxRead.releaseQuery)>(releaseQuery);
}

extern "C" OVPHYSX_INTERNAL_API void ovphysx_internal_set_ovx_contact_accessors(void* queryContacts)
{
    g_ovxQueryContacts = reinterpret_cast<decltype(g_ovxQueryContacts)>(queryContacts);
}

extern "C" OVPHYSX_INTERNAL_API void ovphysx_internal_set_ovx_write_accessors(
    void* write, void* fetch, void* commit, void* releaseWrite)
{
    g_ovxWrite.write        = reinterpret_cast<decltype(g_ovxWrite.write)>(write);
    g_ovxWrite.fetch        = reinterpret_cast<decltype(g_ovxWrite.fetch)>(fetch);
    g_ovxWrite.commit       = reinterpret_cast<decltype(g_ovxWrite.commit)>(commit);
    g_ovxWrite.releaseWrite = reinterpret_cast<decltype(g_ovxWrite.releaseWrite)>(releaseWrite);
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_write_attribute(uint64_t query,
    const ovx_string_or_token_t* attributes, size_t attribute_count, ovphysx_write_result_t* out_summary)
{
    if (out_summary)
        *out_summary = {};
    uint64_t opened = 0;
    try
    {
        OvxWriteFns& f = ovxWriteFns();
        if (!f.write || !f.releaseWrite)
            return 0;
        std::unordered_map<uint64_t, std::vector<ovphysx_write_attribute_result_t>>::node_type ownership;
        if (out_summary)
        {
            std::lock_guard<std::mutex> lock(g_writeMutex);
            g_writeResults.try_emplace(0);
            ownership = g_writeResults.extract(0);
            ownership.mapped().resize(attribute_count);
        }
        omni::physx::OvxWriteResult result{};
        opened = f.write(query, attributes, attribute_count, out_summary ? &result : nullptr);
        if (!opened)
            return 0;
        if (out_summary)
        {
            if (result.attributeCount != attribute_count || (attribute_count && !result.attributes))
                throw std::runtime_error("Write summary disagrees with the request");
            for (size_t i = 0; i < attribute_count; ++i)
                ownership.mapped()[i] = {result.attributes[i].attribute, result.attributes[i].groupCount};
            ownership.key() = opened;
            std::lock_guard<std::mutex> lock(g_writeMutex);
            const std::unordered_map<uint64_t, std::vector<ovphysx_write_attribute_result_t>>::insert_return_type inserted =
                g_writeResults.insert(std::move(ownership));
            if (!inserted.inserted)
            {
                opened = 0; // The existing session owns this identity and must not be unwound.
                throw std::runtime_error("Duplicate write session identity");
            }
            *out_summary = {result.totalPrimCount, attribute_count, inserted.position->second.data()};
        }
        return opened;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Write creation failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Write creation failed with an exception");
    }
    if (opened)
    {
        try { ovxWriteFns().releaseWrite(opened); } catch (...) {}
        std::lock_guard<std::mutex> lock(g_writeMutex);
        g_writeResults.erase(opened);
    }
    return 0;
}

OVPHYSX_INTERNAL_API int ovphysx_internal_fetch_write_next(uint64_t write, ovphysx_fetched_write_group_t* out_group)
{
    if (!out_group)
        return -1;
    *out_group = {nullptr, SIZE_MAX};
    try
    {
        OvxWriteFns& f = ovxWriteFns();
        if (!f.fetch)
            return -1;
        // Preserve the original producer address: it is the commit identity.
        omni::physx::OvxFetchedWriteGroup fetched{};
        const omni::physx::OvxWriteStatus status = f.fetch(write, &fetched);
        if (status == omni::physx::kOvxWriteStatusOk)
        {
            *out_group = {fetched.group, fetched.attributeIndex};
            return 1;
        }
        return status == omni::physx::kOvxWriteStatusEndOfIteration ? 0 : -1;
    }
    catch (...)
    {
        return -1;
    }
}

OVPHYSX_INTERNAL_API int ovphysx_internal_commit_group(uint64_t write,
                                                       const ovstage_map_group_t* group,
                                                       ovstage_cuda_sync_t write_done_sync,
                                                       int32_t* out_failure)
{
    // out_failure is left untouched on the paths below that never reach the runtime. A missing
    // accessor or an escaping exception is neither "not live" nor "the publish failed", so the
    // caller reports the absence of a reason rather than inventing one.
    try
    {
        OvxWriteFns& f = ovxWriteFns();
        if (!f.commit)
            return 0;
        omni::physx::OvxCommitFailure why = omni::physx::kOvxCommitFailureNone;
        const bool ok = f.commit(write, group, write_done_sync, &why);
        if (!ok && out_failure)
            *out_failure = static_cast<int32_t>(why);
        return ok ? 1 : 0;
    }
    catch (...)
    {
        return 0;
    }
}

OVPHYSX_INTERNAL_API int ovphysx_internal_release_write(uint64_t write)
{
    try
    {
        OvxWriteFns& functions = ovxWriteFns();
        if (!functions.releaseWrite)
        {
            CARB_LOG_ERROR("Write release API is unavailable");
            return -1;
        }
        const bool released = functions.releaseWrite(write);
        std::lock_guard<std::mutex> lock(g_writeMutex);
        g_writeResults.erase(write);
        return released ? 1 : 0;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Write release failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Write release failed with an exception");
    }
    return -1;
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query(uint64_t attach, uint32_t object_type, uint32_t scope,
                                                                  ovstage_query_result_t* out_summary)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.query)
            return 0;
        return f.query(attach, object_type, scope, out_summary);
    }
    catch (...)
    {
        return 0;
    }
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query_selected(uint64_t attach, uint32_t object_type,
                                                                     const ovx_primpath_t* primPaths,
                                                                     size_t primPathCount,
                                                                     const ovx_string_t* pattern,
                                                                     ovstage_query_result_t* out_summary)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.querySelected)
            return 0;
        return f.querySelected(attach, object_type, primPaths, primPathCount, pattern, out_summary);
    }
    catch (...)
    {
        return 0;
    }
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query_contacts(uint64_t attach,
                                                                     const void* selections,
                                                                     size_t selectionCount,
                                                                     uint32_t filterCount,
                                                                     ovstage_query_result_t* out_summary)
{
    try
    {
        if (!g_ovxQueryContacts)
            return 0;
        return g_ovxQueryContacts(attach, static_cast<const omni::physx::OvxContactSensorSelection*>(selections),
                                  selectionCount, filterCount, out_summary);
    }
    catch (...)
    {
        return 0;
    }
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_read_outputs(uint64_t query,
    const ovx_string_or_token_t* attributes, size_t attribute_count, ovphysx_read_result_t* out_summary)
{
    if (out_summary)
        *out_summary = {};
    uint64_t opened = 0;
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.read || !f.releaseRead)
            return 0;
        std::unordered_map<uint64_t, ReadSession>::node_type ownership;
        {
            std::lock_guard<std::mutex> lock(g_readMutex);
            g_readSessions.try_emplace(0);
            ownership = g_readSessions.extract(0);
        }
        if (out_summary)
            ownership.mapped().attributes.resize(attribute_count);
        omni::physx::OvxReadResult result{};
        opened = f.read(query, attribute_count ? attributes : nullptr, attribute_count,
                        out_summary ? &result : nullptr);
        if (!opened)
            return 0;
        if (out_summary)
        {
            if (result.attributeCount != attribute_count || (attribute_count && !result.attributes))
                throw std::runtime_error("Read summary disagrees with the request");
            for (size_t i = 0; i < attribute_count; ++i)
                ownership.mapped().attributes[i] = {result.attributes[i].attribute,
                    result.attributes[i].groupCount, result.attributes[i].omissionReasons};
        }
        ownership.key() = opened;
        std::lock_guard<std::mutex> lock(g_readMutex);
        const std::unordered_map<uint64_t, ReadSession>::insert_return_type inserted =
            g_readSessions.insert(std::move(ownership));
        if (!inserted.inserted)
        {
            opened = 0; // The existing session owns this identity and must not be unwound.
            throw std::runtime_error("Duplicate read session identity");
        }
        if (out_summary)
            *out_summary = {result.totalPrimCount, attribute_count, inserted.position->second.attributes.data()};
        return opened;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Read creation failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Read creation failed with an exception");
    }
    if (opened)
    {
        try { ovxReadFns().releaseRead(opened); } catch (...) {}
        std::lock_guard<std::mutex> lock(g_readMutex);
        g_readSessions.erase(opened);
    }
    return 0;
}

OVPHYSX_INTERNAL_API int ovphysx_internal_fetch_read_next(uint64_t read, const ovstage_read_group_t** out_group)
{
    if (!out_group)
        return -1;
    *out_group = nullptr;
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.fetch)
            return -1;

        ovstage_read_group_t g{};
        const omni::physx::OvxReadStatus st = f.fetch(read, &g);
        if (st == omni::physx::kOvxReadStatusEndOfIteration)
            return 0; // end of iteration (not an error)
        if (st != omni::physx::kOvxReadStatusOk)
            return -1; // fetch error, distinct from end of iteration

        std::lock_guard<std::mutex> lk(g_readMutex);
        const auto it = g_readSessions.find(read);
        if (it == g_readSessions.end())
            return -1;

        // Retain the group descriptor and stage-derived prim list until release_group,
        // and hand back a stable address. Numeric storage remains read-session-owned.
        ovstage_read_group_t& kept = (it->second.liveGroups[g.read_group_id] = g);
        *out_group = &kept;
        return 1;
    }
    catch (...)
    {
        return -1;
    }
}

OVPHYSX_INTERNAL_API int ovphysx_internal_release_group(uint64_t read, ovstage_read_group_id_t group_id)
{
    try
    {
        OvxReadFns& functions = ovxReadFns();
        std::lock_guard<std::mutex> lock(g_readMutex);
        const auto session = g_readSessions.find(read);
        if (session == g_readSessions.end())
            return 1;
        const auto group = session->second.liveGroups.find(group_id);
        if (group == session->second.liveGroups.end())
            return 1;
        if (!functions.releaseGroup)
        {
            CARB_LOG_ERROR("Read group release API is unavailable");
            return -1;
        }
        if (!functions.releaseGroup(read, &group->second))
            return -1;
        session->second.liveGroups.erase(group);
        return 1;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Read group release failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Read group release failed with an exception");
    }
    return -1;
}

OVPHYSX_INTERNAL_API int ovphysx_internal_release_read(uint64_t read)
{
    try
    {
        OvxReadFns& functions = ovxReadFns();
        if (!functions.releaseRead)
        {
            CARB_LOG_ERROR("Read release API is unavailable");
            return -1;
        }
        const bool released = functions.releaseRead(read);
        // A reported runtime failure may quarantine storage, but still ends
        // the session and every descriptor borrowed from it.
        std::lock_guard<std::mutex> lock(g_readMutex);
        g_readSessions.erase(read);
        return released ? 1 : 0;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Read release failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Read release failed with an exception");
    }
    return -1;
}

OVPHYSX_INTERNAL_API int ovphysx_internal_release_query(uint64_t query)
{
    try
    {
        OvxReadFns& functions = ovxReadFns();
        if (!functions.releaseQuery)
        {
            CARB_LOG_ERROR("Query release API is unavailable");
            return -1;
        }
        return functions.releaseQuery(query) ? 1 : -1;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Query release failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Query release failed with an exception");
    }
    return -1;
}
