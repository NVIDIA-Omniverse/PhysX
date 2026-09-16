// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OBJECTKEY-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-CAPI-PHYSXPTR-001
 * @covers AC-1 AC-2
 */

// Sidecar interop: raw PhysX object lookup and ObjectKey resolution.
// Bridges omni::physx types into C-ABI values usable by the main lib.

#include "internal/sidecar/ovphysxInternalInterop.h"
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
OVPHYSX_INTERNAL_API void* ovphysx_internal_get_physx_ptr(
    const char* prim_path, int physx_type)
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
            key = physx->resolveObjectKey(prim_path);
            if (!key.valid())
            {
                CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr could not resolve ObjectKey for '%s'", prim_path);
                return nullptr;
            }
        }

        return physx->getPhysXPtr(key, static_cast<omni::physx::PhysXType>(physx_type));
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

OVPHYSX_INTERNAL_API void ovphysx_internal_enable_visualization(bool enable)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->enableVisualization(enable);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: enable_visualization threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter(uint32_t param, bool on)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->setVisualizationParameter(
                static_cast<omni::physx::PhysXVisualizationParameter>(param), on);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_parameter threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter_value(uint32_t param, float value)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        // ABI-appended member: an older omni.physx predating the API leaves
        // the pointer unset, same guard as the scope setter.
        if (visualization && visualization->setVisualizationParameterValue)
        {
            visualization->setVisualizationParameterValue(
                static_cast<omni::physx::PhysXVisualizationParameter>(param), value);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_parameter_value threw");
    }
}

OVPHYSX_INTERNAL_API bool ovphysx_internal_set_visualization_scope_tokens(
    const ovx_primpath_t* tokens, uint32_t count)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (!visualization || !visualization->setVisualizationScopeTokens)
            return false;
        return visualization->setVisualizationScopeTokens(tokens, count);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_scope_tokens threw");
        return false;
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_scale(float scale)
{
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->setVisualizationScale(scale);
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_scale threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_culling_box(
    const float* min3, const float* max3)
{
    if (!min3 || !max3)
        return;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            visualization->setVisualizationCullingBox(carb::Float3{ min3[0], min3[1], min3[2] },
                                                        carb::Float3{ max3[0], max3[1], max3[2] });
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: set_visualization_culling_box threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_points(const void** out, uint32_t* count)
{
    if (out) *out = nullptr;
    if (count) *count = 0;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            if (out) *out = visualization->getPoints();
            if (count) *count = visualization->getNbPoints();
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: get_debug_points threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_lines(const void** out, uint32_t* count)
{
    if (out) *out = nullptr;
    if (count) *count = 0;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            if (out) *out = visualization->getLines();
            if (count) *count = visualization->getNbLines();
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: get_debug_lines threw");
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_triangles(const void** out, uint32_t* count)
{
    if (out) *out = nullptr;
    if (count) *count = 0;
    try
    {
        omni::physx::IPhysxVisualization* visualization =
            ovphysx::internal::sidecar::tryGetInjectedPhysxVisualizationInterface();
        if (visualization)
        {
            if (out) *out = visualization->getTriangles();
            if (count) *count = visualization->getNbTriangles();
        }
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: get_debug_triangles threw");
    }
}

// OMPE-94459: calls PxArticulationReducedCoordinate::updateKinematic on the
// articulation at the given prim path. The flags bitmask matches
// ovphysx_articulation_kinematic_flag_t (POSITION=1, VELOCITY=2).
OVPHYSX_INTERNAL_API bool ovphysx_internal_update_kinematic(
    const char* prim_path, uint32_t flags)
{
    if (!prim_path || prim_path[0] == '\0')
        return false;
    try
    {
        omni::physx::IPhysx* physx = ovphysx::internal::sidecar::tryGetInjectedPhysxInterface();
        if (!physx)
            return false;
        omni::physics::parse::ObjectKey key = physx->resolveObjectKey(prim_path);
        if (!key.valid())
            return false;
        void* p = physx->getPhysXPtr(key, omni::physx::ePTArticulation);
        if (!p)
            return false;
        auto* arti = static_cast<::physx::PxArticulationReducedCoordinate*>(p);
        ::physx::PxArticulationKinematicFlags pxFlags;
        if (flags & OVPHYSX_ARTICULATION_KINEMATIC_POSITION) pxFlags |= ::physx::PxArticulationKinematicFlag::ePOSITION;
        if (flags & OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY) pxFlags |= ::physx::PxArticulationKinematicFlag::eVELOCITY;
        if (!pxFlags)
            return true;  // nothing to do, but path resolved
        arti->updateKinematic(pxFlags);
        return true;
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_update_kinematic exception: %s", e.what());
        return false;
    }
    catch (...)
    {
        return false;
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
    decltype(&omni::physx::ovxFetchQueryResult) fetchQueryResult = nullptr;
    decltype(&omni::physx::ovxQueryDictionary)  dict = nullptr;
    decltype(&omni::physx::ovxReadAttributes)   read = nullptr;
    decltype(&omni::physx::ovxFetchReadNext)    fetch = nullptr;
    decltype(&omni::physx::ovxReleaseGroup)     releaseGroup = nullptr;
    decltype(&omni::physx::ovxReleaseRead)      releaseRead = nullptr;
    decltype(&omni::physx::ovxReleaseQuery)     releaseQuery = nullptr;
    bool ready() const
    {
        return query && fetchQueryResult && dict && read && fetch && releaseGroup && releaseRead && releaseQuery;
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
    decltype(&omni::physx::ovxWriteAttribute) write = nullptr;
    decltype(&omni::physx::ovxFetchWriteNext) fetch = nullptr;
    decltype(&omni::physx::ovxCommitGroup)    commit = nullptr;
    decltype(&omni::physx::ovxReleaseWrite)   releaseWrite = nullptr;
};

OvxWriteFns g_ovxWrite;

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
};

std::mutex g_readMutex;
std::unordered_map<uint64_t, ReadSession> g_readSessions; // read -> session

void registerReadSession(uint64_t read)
{
    if (!read)
        return;
    std::lock_guard<std::mutex> lk(g_readMutex);
    g_readSessions[read] = ReadSession{};
}
} // namespace

// Owner-injected addresses of the statically linked omni::physx::ovx* read entry
// points. Passed as void* so the shared sidecar header stays free of the ovx/ovstage
// read types, and cast back to the decltype pointer types here. See g_ovxRead.
// Declared extern "C" so the loader can resolve it by plain name via dlsym/GetProcAddress.
// This file includes ovphysxInternalInterop.h rather than ovphysxInternal.h, where the
// C declaration lives, so the linkage has to be stated here too.
extern "C" OVPHYSX_INTERNAL_API void ovphysx_internal_set_ovx_read_accessors(
    void* query, void* fetchQueryResult, void* dict, void* read,
    void* fetch, void* releaseGroup, void* releaseRead, void* releaseQuery)
{
    g_ovxRead.query            = reinterpret_cast<decltype(g_ovxRead.query)>(query);
    g_ovxRead.fetchQueryResult = reinterpret_cast<decltype(g_ovxRead.fetchQueryResult)>(fetchQueryResult);
    g_ovxRead.dict             = reinterpret_cast<decltype(g_ovxRead.dict)>(dict);
    g_ovxRead.read             = reinterpret_cast<decltype(g_ovxRead.read)>(read);
    g_ovxRead.fetch            = reinterpret_cast<decltype(g_ovxRead.fetch)>(fetch);
    g_ovxRead.releaseGroup     = reinterpret_cast<decltype(g_ovxRead.releaseGroup)>(releaseGroup);
    g_ovxRead.releaseRead      = reinterpret_cast<decltype(g_ovxRead.releaseRead)>(releaseRead);
    g_ovxRead.releaseQuery     = reinterpret_cast<decltype(g_ovxRead.releaseQuery)>(releaseQuery);
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
                                                               const ovx_string_or_token_t* attribute)
{
    try
    {
        OvxWriteFns& f = ovxWriteFns();
        if (!f.write)
            return 0;
        return f.write(query, attribute);
    }
    catch (...)
    {
        return 0;
    }
}

OVPHYSX_INTERNAL_API int ovphysx_internal_fetch_write_next(uint64_t write, const ovstage_map_group_t** out_group)
{
    if (!out_group)
        return -1;
    *out_group = nullptr;
    try
    {
        OvxWriteFns& f = ovxWriteFns();
        if (!f.fetch)
            return -1;
        // The group pointer is handed back unchanged. A map group carries no id, so its address is
        // the commit identity. Copying it into sidecar-side storage, as the read does with its
        // groups, would hand the caller an address the runtime cannot recognise on commit.
        const omni::physx::OvxWriteStatus st = f.fetch(write, out_group);
        if (st == omni::physx::kOvxWriteStatusOk)
            return 1;
        if (st == omni::physx::kOvxWriteStatusEndOfIteration)
            return 0;
        return -1;
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

OVPHYSX_INTERNAL_API void ovphysx_internal_release_write(uint64_t write)
{
    try
    {
        OvxWriteFns& f = ovxWriteFns();
        if (f.releaseWrite)
            f.releaseWrite(write);
    }
    catch (...)
    {
    }
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query(uint32_t object_type, uint32_t scope)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.query)
            return 0;
        return f.query(object_type, scope);
    }
    catch (...)
    {
        return 0;
    }
}

OVPHYSX_INTERNAL_API int ovphysx_internal_fetch_query_result(uint64_t query, ovstage_query_result_t* out_result)
{
    if (!out_result)
        return -1;
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.fetchQueryResult)
            return -1;
        // ovxFetchQueryResult fills ovstage's own type, so it is passed straight through.
        // The attributes array is owned by the query and valid until release_query.
        *out_result = ovstage_query_result_t{};
        if (!f.fetchQueryResult(query, out_result))
            return 0;
        return 1;
    }
    catch (...)
    {
        return -1;
    }
}

OVPHYSX_INTERNAL_API void* ovphysx_internal_query_dictionary(uint64_t query)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.dict)
            return nullptr;
        return reinterpret_cast<void*>(f.dict(query));
    }
    catch (...)
    {
        return nullptr;
    }
}

OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_read_outputs(uint64_t query,
                                                            const ovx_string_or_token_t* attributes,
                                                            size_t attribute_count)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (!f.read)
            return 0;
        // Attributes are already ovstage's ovx_string_or_token_t and are forwarded verbatim.
        const uint64_t r = f.read(query, attribute_count ? attributes : nullptr, attribute_count);
        registerReadSession(r);
        return r;
    }
    catch (...)
    {
        return 0;
    }
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

OVPHYSX_INTERNAL_API void ovphysx_internal_release_group(uint64_t read, ovstage_read_group_id_t group_id)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        std::lock_guard<std::mutex> lk(g_readMutex);
        const auto it = g_readSessions.find(read);
        if (it == g_readSessions.end())
            return;
        auto& live = it->second.liveGroups;
        const auto gi = live.find(group_id);
        if (gi == live.end())
            return; // already released or unknown, idempotent
        if (f.releaseGroup)
            f.releaseGroup(read, &gi->second);
        live.erase(gi);
    }
    catch (...)
    {
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_release_read(uint64_t read)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        // ovxReleaseRead tears down the whole read session including any groups it
        // still owns, so retained groups need no individual release.
        if (f.releaseRead)
            f.releaseRead(read);
        std::lock_guard<std::mutex> lk(g_readMutex);
        g_readSessions.erase(read);
    }
    catch (...)
    {
    }
}

OVPHYSX_INTERNAL_API void ovphysx_internal_release_query(uint64_t query)
{
    try
    {
        OvxReadFns& f = ovxReadFns();
        if (f.releaseQuery)
            f.releaseQuery(query);
    }
    catch (...)
    {
    }
}
