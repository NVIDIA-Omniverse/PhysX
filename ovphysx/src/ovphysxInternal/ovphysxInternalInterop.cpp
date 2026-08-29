// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Sidecar interop: raw PhysX object pointer lookup and SdfPath encoding.
// Bridges omni::physx / USD types into C-ABI values usable by the main lib.

#include "internal/sidecar/ovphysxInternalInterop.h"
#include "internal/sidecar/ovphysxInternalUtil.hpp"
#include "ovphysx/ovphysx_types.h"
#include "ovphysxInternalPhysXAccess.hpp"

#include <pxr/usd/sdf/path.h>  // must precede IPhysx.h: defines PXR_NS used by struct decls

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IOvxPhysicsRead.h> // ovstage-native output read symbols + types
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
// layout-compatible with omni::physx::DebugPoint / DebugLine / DebugTriangle so the
// debug-render getters hand back the omni.physx buffer via zero-copy reinterpret
// (see ovphysxPhysXInterop.cpp). Size checks catch added/removed fields, offset checks
// catch reordering, and the is_same guards below catch an equal-width type swap
// (e.g. uint32 colour -> float) that sizeof/offsetof alone would miss.
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
// omni::physx::PhysXVisualizationParameter values one-for-one: the forward in
// internal_set_visualization_parameter is a raw static_cast<PhysXVisualizationParameter>
// with no translation table, so EVERY enumerator is asserted (not just spot-checks) --
// an intra-range reorder that preserved count and endpoints would otherwise mis-map.
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

// The public ovphysx output-read surface speaks ovstage's own types directly
// (ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t), so there
// is no ovphysx mirror enum to keep in lock-step.

// Conformance check (#10): the runtime's OvxReadStatus must map cleanly onto the
// 1 / 0 / -1 convention ovphysx_internal_fetch_read_next returns.
static_assert((int)omni::physx::kOvxReadStatusOk != (int)omni::physx::kOvxReadStatusEndOfIteration, "OvxReadStatus ok/eof collision");
static_assert((int)omni::physx::kOvxReadStatusError != (int)omni::physx::kOvxReadStatusOk, "OvxReadStatus ok/error collision");

// try/catch guards the extern "C" boundary -- C++ exceptions must not escape into C callers.
// physx_type is not validated here; PhysX internally returns nullptr for unknown types.
OVPHYSX_INTERNAL_API void* ovphysx_internal_get_physx_ptr(
    const char* prim_path, int physx_type)
{
    if (!prim_path || prim_path[0] == '\0')
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr called with null or empty prim_path");
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

        PXR_NS::SdfPath path(prim_path);
        if (path.IsEmpty())
        {
            CARB_LOG_ERROR("Internal sidecar: ovphysx_internal_get_physx_ptr got invalid SdfPath from '%s'", prim_path);
            return nullptr;
        }

        return physx->getPhysXPtr(path, static_cast<omni::physx::PhysXType>(physx_type));
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
// A non-positive (unset) viewport gizmo scale (the common case in headless / minimal
// hosts) is treated as 1.0 inside omni::physx when computing eSCALE
// (PhysXDebugVisualization.cpp), so this sidecar does not seed the persistent
// /persistent/app/viewport/gizmo/scale setting -- a process-global side effect
// ovphysx should not own.

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

// OMPE-94459 (_KINEMATIC_UPDATE_NOOP fix): call
// PxArticulationReducedCoordinate::updateKinematic on the articulation at
// the given prim path. The flags bitmask matches
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
        PXR_NS::SdfPath path(prim_path);
        if (path.IsEmpty())
            return false;
        void* p = physx->getPhysXPtr(path, omni::physx::ePTArticulation);
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

OVPHYSX_INTERNAL_API uint64_t ovphysx_encode_sdf_path(const char* prim_path)
{
    if (!prim_path || prim_path[0] == '\0')
        return 0;

    try
    {
        PXR_NS::SdfPath path(prim_path);
        if (path.IsEmpty())
        {
            CARB_LOG_ERROR("Internal sidecar: ovphysx_encode_sdf_path got invalid SdfPath from '%s'", prim_path);
            return 0;
        }
        return ovphysx::internal::sdfPathToInt(path);
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_encode_sdf_path caught exception: %s", e.what());
        return 0;
    }
    catch (...)
    {
        CARB_LOG_ERROR("Internal sidecar: ovphysx_encode_sdf_path caught unknown exception");
        return 0;
    }
}

// ============================================================================
// Physics output read (ADR-0007)
//
// The runtime plugin exports the ovstage-native read entry points as standalone
// (non-carb) symbols (omni::physx::ovx*). They are resolved here by name from the
// already-loaded plugin module. The sidecar owns the read-session bookkeeping and
// hands back each ovstage_read_group_t verbatim (the public surface IS ovstage's
// type — no ovphysx mirror, no translation).
// ============================================================================

namespace
{
// Resolved plugin read symbols (lazy; retried until the plugin is loaded).
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
// library's sidecar loader calls at startup with the addresses of ITS OWN
// statically-linked omni::physx::ovx* read entry points. The sidecar cannot reach
// them on its own: it has no OvruntimePhysX of its own to link, the owner library's
// symbols are hidden (not exported on its dynamic table), and there is no separate
// libomni.physx.plugin.so in the static layout -- dlopen("libomni.physx.plugin.so",
// RTLD_NOLOAD)+dlsym would silently get a null handle, so injection from the owner
// (same handshake as ovphysx_internal_set_physx_runtime_accessors) is used instead.
OvxReadFns g_ovxRead;

OvxReadFns& ovxReadFns()
{
    return g_ovxRead;
}

// Per-read-session storage: the live ovstage groups retained by read_group_id.
// Retaining each fetched ovstage_read_group_t keeps its borrowed tensors /
// index_map / prim_list valid until ovphysx_internal_release_group, AND gives us a
// stable address to hand back as the producer-owned `const ovstage_read_group_t*`
// (no ovphysx mirror struct anymore — the public surface IS ovstage's type). The
// node-based map keeps &liveGroups[id] stable until release. (The ovstage read
// contract makes each group independently valid until its own release — fetching
// more does not invalidate earlier ones.)
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

// Owner-injected addresses of the statically-linked omni::physx::ovx* read entry
// points. Passed as void* so the shared sidecar header stays free of the ovx/ovstage
// read types; cast back to the resolved decltype pointer types here. See g_ovxRead.
// extern "C" so the loader can resolve it by its plain name via dlsym/GetProcAddress
// (this .cpp includes ovphysxInternalInterop.h, not ovphysxInternal.h where the C
// declaration lives, so the linkage must be stated here too).
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
        // ovxFetchQueryResult fills ovstage's own type; pass it straight through
        // (the attributes array is owned by the query, valid until release_query).
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
        // Attributes are already ovstage's ovx_string_or_token_t — forward verbatim.
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
            return -1; // genuine fetch error — distinct from EOF

        std::lock_guard<std::mutex> lk(g_readMutex);
        const auto it = g_readSessions.find(read);
        if (it == g_readSessions.end())
            return -1;

        // Retain the ovstage group so its borrowed storage stays valid until
        // ovphysx_internal_release_group, and hand back its stable address — the
        // public surface IS ovstage_read_group_t, so there is nothing to translate.
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
            return; // already released / unknown — idempotent
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
        // still owns, so we do not need to release retained groups individually.
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
