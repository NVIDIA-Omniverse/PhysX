// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// End-to-end debug-render EMISSION tests: load a real dynamics scene, step the simulation,
// and verify that each drawable PhysX visualization parameter exports render-buffer
// primitives. This complements test_debug_render.cpp (which covers the C-API contract on a
// bare, stage-less instance -- validation, cached getters, no-stage errors). Here we attach
// a stage and step, so we exercise the full pipeline: set_parameter -> eVISUALIZATION flags
// -> NpScene::visualize() -> getRenderBuffer -> the ovphysx get_points/lines/triangles.
//
// There is no viewport in the test process, so the omni.physx fix that clamps an unset
// viewport gizmo scale to 1.0 is what makes eSCALE non-zero and lets viz emit at all.
//
// NOT every one of the 27 parameters can export in a single stock CPU scene: some need
// scene features the falling-boxes scene lacks (joints/articulations, SDF colliders, an MBP
// broadphase, a set culling box). The DebugRenderE2E suite asserts the strong baseline that
// MUST export for a dynamic rigid-body scene plus dedicated scenes per feature, and prints
// per-parameter matrices so any regression is caught and any conditional non-emitter is
// visible rather than silently skipped.
//
// Two parameters cannot emit at all and are deliberately not asserted:
//  - MBP_REGIONS: implemented in the SDK (NpDebugViz), but MBP broadphase regions cannot be
//    authored through ovphysx or USD physics, so there is never a region to draw (see the
//    skip in MBPRegionsExportPrimitives).
//  - SIMULATION_MESH: dead parameter in this PhysX generation -- nothing in the SDK reads
//    PxVisualizationParameter::eSIMULATION_MESH (deformable visualization is an open TODO in
//    NpDebugViz.cpp), so no scene, CPU or GPU, can make it export.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovx/path_dictionary/path_dictionary.h>

#include <vector>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>

using namespace test_utils;

namespace
{
bool e2e_wait(ovphysx_handle_t h, ovphysx_op_index_t op)
{
    ovphysx_op_wait_result_t wr{};
    ovphysx_result_t r = ovphysx_wait_op(h, op, 10'000'000'000ULL, &wr);
    ovphysx_destroy_wait_result(&wr);
    return r.status == OVPHYSX_API_SUCCESS;
}

bool e2e_step(ovphysx_handle_t h, float dt)
{
    ovphysx_enqueue_result_t r = ovphysx_step(h, dt);
    return r.status == OVPHYSX_API_SUCCESS && e2e_wait(h, r.op_index);
}

bool e2e_intern_path(ovphysx_handle_t h, const char* path, ovx_primpath_t* out_path)
{
    if (!path || !out_path)
        return false;

    std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
    const auto it = test_utils::ovstage_test_attachments().find(h);
    if (it == test_utils::ovstage_test_attachments().end() ||
        it->second.empty() || !it->second.back().stage)
        return false;

    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(it->second.back().stage);
    if (!dictionary)
        return false;
    const ovx_string_t string_path{path, std::strlen(path)};
    const ovx_api_result_t result =
        path_dictionary_create_paths_from_strings(dictionary, &string_path, 1u, out_path);
    const bool ok = result.status == OVX_API_SUCCESS && *out_path != OVX_INVALID_PRIMPATH;
    if (result.error.ptr)
        path_dictionary_release_error(dictionary, result.error);
    return ok;
}

// Total render-buffer primitives (points + lines + triangles) currently exported.
uint32_t e2e_total_prims(ovphysx_handle_t h)
{
    const ovphysx_debug_point_t* p = nullptr;
    const ovphysx_debug_line_t* l = nullptr;
    const ovphysx_debug_triangle_t* t = nullptr;
    uint32_t np = 0, nl = 0, nt = 0;
    ovphysx_debug_render_get_points(h, &p, &np);
    ovphysx_debug_render_get_lines(h, &l, &nl);
    ovphysx_debug_render_get_triangles(h, &t, &nt);
    return np + nl + nt;
}

// One body's world position through a per-path tensor binding (the
// test_clone.cpp idiom). Returns false on any API failure.
bool e2e_read_position(ovphysx_handle_t h, const char* prim_path, float* out_pos)
{
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = test_utils::make_ovx_string(prim_path);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t binding = 0;
    if (ovphysx_create_tensor_binding(h, &desc, &binding).status != OVPHYSX_API_SUCCESS)
        return false;
    float pose[7] = {};
    int64_t shape[] = { 1, 7 };
    DLTensor tensor{};
    tensor.data = pose;
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.dtype = { kDLFloat, 32, 1 };
    tensor.device = { kDLCPU, 0 };
    const ovphysx_result_t r = ovphysx_read_tensor_binding(h, binding, &tensor);
    ovphysx_destroy_tensor_binding(h, binding);
    if (r.status != OVPHYSX_API_SUCCESS)
        return false;
    out_pos[0] = pose[0];
    out_pos[1] = pose[1];
    out_pos[2] = pose[2];
    return true;
}

float e2e_dist3(const float* a, const float* b)
{
    const float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct DebugRenderE2E : public PhysXTestFixture
{
    // The viz mask, scope and scale are process-global. Reset them here so a
    // mid-test ASSERT_ cannot leak state into the next test; the trailing
    // per-test cleanups remain as harmless documentation of what each test set.
    void TearDown() override
    {
        ovphysx_debug_render_set_scope_tokens(m_handle, nullptr, 0u);
        for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
            ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
        ovphysx_debug_render_enable(m_handle, false);
        PhysXTestFixture::TearDown();
    }
};
} // namespace

TEST_F(DebugRenderE2E, DrawableParamsExportPrimitives)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // A few steps so the boxes are falling -> linear-velocity arrows have length.
    for (int i = 0; i < 3; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    // Sweep every drawable parameter EXCLUSIVELY (one at a time) and record whether it
    // exports primitives. Early params observe falling bodies (velocity); later params
    // observe landing/settling (contacts) as the sweep advances the sim.
    const int N = OVPHYSX_DEBUG_RENDER_PARAM_COUNT;
    std::vector<uint32_t> total(N, 0u);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < N; ++p)
    {
        ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 1.0f);
        for (int i = 0; i < 2; ++i)
            ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed at param " << p;
        total[p] = e2e_total_prims(m_handle);
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    }

    static const char* kNames[OVPHYSX_DEBUG_RENDER_PARAM_COUNT] = {
        "NONE", "WorldAxes", "BodyAxes", "BodyMassAxes", "BodyLinVel", "BodyAngVel",
        "ContactPoint", "ContactNormal", "ContactError", "ContactImpulse",
        "FrictionPoint", "FrictionNormal", "FrictionImpulse", "ActorAxes",
        "CollisionAABBs", "CollisionShapes", "CollisionAxes", "CollisionCompounds",
        "CollisionFaceNormals", "CollisionEdges", "CollisionStaticPruner",
        "CollisionDynamicPruner", "JointLocalFrames", "JointLimits", "CullBox",
        "MBPRegions", "SimulationMesh", "SDF"
    };
    std::printf("[DebugRenderE2E] boxes_falling per-parameter export matrix:\n");
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < N; ++p)
        std::printf("  %2d %-22s : %-8s (%u prims)\n", p, kNames[p],
                    total[p] > 0u ? "EXPORTS" : "none", total[p]);

    // Baseline: parameters that MUST export for a dynamic rigid-body scene regardless of
    // solver/broadphase. A regression that stops any of these exporting fails here.
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES], 0u) << "World axes";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES], 0u) << "Body axes";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_BODY_MASS_AXES], 0u) << "Body mass axes";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_ACTOR_AXES], 0u) << "Actor axes";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AABBS], 0u) << "Collision AABBs";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_SHAPES], 0u) << "Collision shapes";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_BODY_LINEAR_VELOCITY], 0u)
        << "Body linear velocity (boxes are falling, so speed > 0)";
    // These four emit structurally for any dynamic rigid-body scene too (the arrow/axis
    // primitives are drawn per body/shape whenever the parameter is on, regardless of
    // magnitudes), so assert them as regression baseline as well.
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_BODY_ANGULAR_VELOCITY], 0u) << "Body angular velocity";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_AXES], 0u) << "Collision (shape) axes";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_STATIC_PRUNER], 0u) << "Static pruner structure";
    EXPECT_GT(total[OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_DYNAMIC_PRUNER], 0u) << "Dynamic pruner structure";
}


TEST_F(DebugRenderE2E, JointParamsExportPrimitives)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/Ant.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Warm up: let the ant articulation settle and start moving.
    for (int i = 0; i < 3; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    // Test JOINT_LOCAL_FRAMES parameter (22).
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LOCAL_FRAMES, 1.0f);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed for JOINT_LOCAL_FRAMES";
    uint32_t joint_local_frames_prims = e2e_total_prims(m_handle);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LOCAL_FRAMES, 0.0f);

    // Test JOINT_LIMITS parameter (23).
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LIMITS, 1.0f);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed for JOINT_LIMITS";
    uint32_t joint_limits_prims = e2e_total_prims(m_handle);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LIMITS, 0.0f);

    std::printf("[DebugRenderE2E] Ant articulation joint parameters:\n");
    std::printf("  %2d %-22s : %-8s (%u prims)\n", OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LOCAL_FRAMES,
                "JointLocalFrames", joint_local_frames_prims > 0u ? "EXPORTS" : "none", joint_local_frames_prims);
    std::printf("  %2d %-22s : %-8s (%u prims)\n", OVPHYSX_DEBUG_RENDER_PARAM_JOINT_LIMITS,
                "JointLimits", joint_limits_prims > 0u ? "EXPORTS" : "none", joint_limits_prims);

    // Assertions: Ant.usda has multiple revolute joints with lowerLimit defined,
    // so both parameters should export visualization primitives.
    EXPECT_GT(joint_local_frames_prims, 0u)
        << "Joint local frames (Ant has multiple articulated joints with local frames)";
    EXPECT_GT(joint_limits_prims, 0u)
        << "Joint limits (Ant revolute joints define lowerLimit, so limits are visualizable)";
}




TEST_F(DebugRenderE2E, TriangleMeshCollisionsExportPrimitives)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/trimesh_ground_collisions.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // A few steps so the box collides with the trimesh ground -> collision normals/edges are visible.
    for (int i = 0; i < 5; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    // Test COLLISION_FACE_NORMALS (param 18)
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_FACE_NORMALS, 1.0f);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "face normals step failed";
    uint32_t face_normals_total = e2e_total_prims(m_handle);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_FACE_NORMALS, 0.0f);
    EXPECT_GT(face_normals_total, 0u)
        << "COLLISION_FACE_NORMALS should export primitives for triangle-mesh collider";

    // Test COLLISION_EDGES (param 19)
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_EDGES, 1.0f);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "edges step failed";
    uint32_t edges_total = e2e_total_prims(m_handle);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_EDGES, 0.0f);
    EXPECT_GT(edges_total, 0u)
        << "COLLISION_EDGES should export primitives for triangle-mesh collider";

    std::printf("[DebugRenderE2E] trimesh_ground_collisions per-parameter export:\n");
    std::printf("  %2d %-22s : %-8s (%u prims)\n", OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_FACE_NORMALS, "CollisionFaceNormals",
                face_normals_total > 0u ? "EXPORTS" : "none", face_normals_total);
    std::printf("  %2d %-22s : %-8s (%u prims)\n", OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_EDGES, "CollisionEdges",
                edges_total > 0u ? "EXPORTS" : "none", edges_total);
}


TEST_F(DebugRenderE2E, CompoundShapesExportPrimitives)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/compound_shapes.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Step a few times to let physics settle
    for (int i = 0; i < 3; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    // Enable only COLLISION_COMPOUNDS to isolate the parameter
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(
        m_handle, OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_COMPOUNDS, 1.0f);

    // Step to trigger visualization
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed";

    uint32_t total = e2e_total_prims(m_handle);
    std::printf("[DebugRenderE2E] Compound shapes COLLISION_COMPOUNDS test: %u prims exported\n",
                total);

    // Verify that compound shapes visualization exports primitives
    EXPECT_GT(total, 0u) << "Collision compounds (compound AABBs) should export primitives";

    // The parameter mask is process-global (ovphysx cache + omni.physx visMask) and
    // survives the fixture's reset_stage: disable what this test enabled so later
    // tests' enable(true) does not re-apply a leaked bit to their fresh scene.
    ovphysx_debug_render_set_parameter(
        m_handle, OVPHYSX_DEBUG_RENDER_PARAM_COLLISION_COMPOUNDS, 0.0f);
}


TEST_F(DebugRenderE2E, MBPRegionsExportPrimitives)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_mbp.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Warm up simulation a couple steps for MBP to organize regions
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    // Enable MBP_REGIONS visualization exclusively
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_MBP_REGIONS, 1.0f);

    // Step to allow visualization to emit
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed";

    uint32_t total = e2e_total_prims(m_handle);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_MBP_REGIONS, 0.0f);
    std::printf("[DebugRenderE2E] MBP_REGIONS exports %u primitives\n", total);
    // MBP_REGIONS visualizes the broadphase's registered regions. MBP regions must be
    // defined explicitly, but ovphysx exposes no broadphase-region API and USD physics has
    // no schema to declare them, so with broadphaseType="MBP" alone there are no regions to
    // draw. This is a coverage limitation (would need a new region API), not a viz defect --
    // skip when none are produced, and assert export if a build ever does register regions.
    if (total == 0u)
        GTEST_SKIP() << "MBP_REGIONS has no regions to draw: ovphysx exposes no broadphase-"
                        "region API and USD cannot declare MBP regions (coverage limitation).";
    SUCCEED() << "MBP_REGIONS exported " << total << " primitives";
}


TEST_F(DebugRenderE2E, CullBoxExportsWhenSet)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Warm up the simulation with a few steps.
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    // Set a culling box around the scene (all boxes are within ~(-30, -30, -10) to (30, 30, 20)).
    // CULL_BOX visualization only emits when a culling box is explicitly set.
    float min_box[3] = {-100.0f, -100.0f, -100.0f};
    float max_box[3] = {100.0f, 100.0f, 100.0f};
    ASSERT_EQ(ovphysx_debug_render_set_culling_box(m_handle, min_box, max_box).status,
              OVPHYSX_API_SUCCESS) << "set_culling_box failed";

    // Enable debug render and CULL_BOX parameter exclusively.
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CULL_BOX, 1.0f);

    // Step the simulation so visualization captures the culling box.
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed";

    // Verify that CULL_BOX exports render primitives.
    uint32_t total_prims = e2e_total_prims(m_handle);
    EXPECT_GT(total_prims, 0u) << "CULL_BOX should export primitives when a culling box is set";

    // Same leak hygiene as every sibling test: the mask is process-global, so a
    // leaked CULL_BOX bit would re-arm on later tests' enable(true).
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CULL_BOX, 0.0f);
}


TEST_F(DebugRenderE2E, ContactAndFrictionParamsExport)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Settle the boxes onto the ground so there is SUSTAINED contact -- contact/friction
    // visualization only emits while bodies are actually touching.
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "settle step failed";

    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    struct ContactVizParam
    {
        ovphysx_debug_render_parameter_t param;
        const char* name;
    };
    const ContactVizParam params[] = {
        { OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, "ContactPoint" },
        { OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, "ContactNormal" },
        { OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_ERROR, "ContactError" },
        { OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_IMPULSE, "ContactImpulse" },
        { OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_POINT, "FrictionPoint" },
        { OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_NORMAL, "FrictionNormal" },
        { OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_IMPULSE, "FrictionImpulse" }
    };
    // On the CPU pipeline (this suite runs in the cpu pass, OVPHYSX_DISABLE_GPU=1) the
    // contact/friction viz is drawn by ShapeInteraction::visualize from the host streams
    // and each parameter emits for settled resting boxes (the draw is gated on the
    // PARAMETER, not on the physical magnitude, so even ~0 rest impulses draw their
    // segments). Assert each parameter individually: a regression that silently kills
    // any one of them must fail here, not skip.
    for (const ContactVizParam& pp : params)
    {
        ovphysx_debug_render_set_parameter(m_handle, pp.param, 1.0f);
        for (int i = 0; i < 2; ++i)
            ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed at contact param " << pp.name;
        uint32_t t = e2e_total_prims(m_handle);
        std::printf("  %2d %-22s : %u prims\n", pp.param, pp.name, t);
        ovphysx_debug_render_set_parameter(m_handle, pp.param, 0.0f);
        EXPECT_GT(t, 0u) << pp.name << " must emit for settled boxes in sustained ground contact";
    }
}


// Per-parameter float VALUES: each PhysX visualization parameter is a magnitude
// multiplied by eSCALE (the bool setter maps to 1.0/0.0); the value entry point
// lets hosts scale semantic groups of gizmos independently. Pin the contract on
// the CPU pass: contact-normal segment length tracks the parameter value.
TEST_F(DebugRenderE2E, ParameterValueScalesMagnitude)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, 1.0f);
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "settle step failed";

    auto max_line_len = [this]() {
        const ovphysx_debug_line_t* ls = nullptr;
        uint32_t n = 0;
        ovphysx_debug_render_get_lines(m_handle, &ls, &n);
        float best = 0.0f;
        for (uint32_t i = 0; i < n; ++i) {
            const float dx = ls[i].pos1[0] - ls[i].pos0[0];
            const float dy = ls[i].pos1[1] - ls[i].pos0[1];
            const float dz = ls[i].pos1[2] - ls[i].pos0[2];
            const float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 > best) best = d2;
        }
        return std::sqrt(best);
    };
    const float len1 = max_line_len();
    ASSERT_GT(len1, 0.0f) << "contact normals must emit for settled boxes";

    // Triple the CONTACT_NORMAL size.
    ASSERT_EQ(ovphysx_debug_render_set_parameter(
                  m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, 3.0f).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed after value change";
    const float len3 = max_line_len();
    std::printf("[DebugRenderE2E::ParamValue] max normal len at value 1: %f, at value 3: %f\n",
                double(len1), double(len3));
    EXPECT_NEAR(len3 / len1, 3.0f, 0.1f)
        << "contact-normal length must track the parameter value";

    // Invalid arguments are rejected.
    EXPECT_EQ(ovphysx_debug_render_set_parameter(m_handle, 0u, 1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(
                  m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, -1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);

    // Suite-shared state hygiene.
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, 0.0f);
}


TEST_F(DebugRenderE2E, TokenScopeUsesExactInternedMembership)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(
        m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_debug_render_set_parameter(
                  m_handle,
                  OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES,
                  1.0f).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f));
    const uint32_t full = e2e_total_prims(m_handle);
    ASSERT_GT(full, 0u);

    ovx_primpath_t cube_tokens[2]{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/World/Cube1", &cube_tokens[0]));
    ASSERT_TRUE(e2e_intern_path(m_handle, "/World/Cube1", &cube_tokens[1]));
    EXPECT_NE(cube_tokens[0], OVX_INVALID_PRIMPATH);
    EXPECT_EQ(cube_tokens[0], cube_tokens[1]);

    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(
                  m_handle, cube_tokens, 1u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f));
    const uint32_t cube_only = e2e_total_prims(m_handle);
    EXPECT_GT(cube_only, 0u);
    EXPECT_LT(cube_only, full);

    ovx_primpath_t world_token{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/World", &world_token));
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(
                  m_handle, &world_token, 1u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f));
    EXPECT_EQ(e2e_total_prims(m_handle), 0u);

    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(
                  m_handle, nullptr, 0u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f));
    EXPECT_EQ(e2e_total_prims(m_handle), full);
}


TEST_F(DebugRenderE2E, ScopeRestrictsContactViz)
{
#ifndef OVPHYSX_TEST_PHYSXSDK_CONTACT_SCOPE_VIZ
    GTEST_SKIP() << "prebuilt PhysX SDK predates the contact-viz scope gate "
                    "(Sc::ShapeInteraction::visualize either-side rule); asserted on source builds";
#endif
    // Contact primitives are PAIR products emitted by ShapeInteraction::visualize,
    // not per-actor draws, so they need their own scope gate (either-side rule: the
    // pair draws while at least one side keeps its actor+shape eVISUALIZATION
    // flags). Pinned with CONTACT_POINT/NORMAL on resting boxes: scoped to one cube
    // its ground contacts survive, an out-of-scope path silences everything, and
    // clearing restores emission.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 1.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, 1.0f);
    // Land the boxes but stay well inside the wake window: sleeping pairs stop
    // emitting contacts regardless of scope, which would fake a positive.
    for (int i = 0; i < 60; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "settle step failed";
    const uint32_t full = e2e_total_prims(m_handle);
    if (full == 0u)
        GTEST_SKIP() << "no contact prims emitted (SDK built without GPU_NP_VISUALIZATION); "
                        "the scope gate is asserted where contacts emit";

    // One cube in scope: its pair with the (out-of-scope) ground keeps drawing
    // via the either-side rule, everything else goes quiet.
    ovx_primpath_t one_cube_token{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/World/Cube1", &one_cube_token));
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, &one_cube_token, 1u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after scoping failed";
    const uint32_t scoped = e2e_total_prims(m_handle);
    std::printf("[DebugRenderE2E::ContactScope] contact prims full=%u scoped(one cube)=%u\n",
                full, scoped);
    EXPECT_GT(scoped, 0u) << "the scoped cube must keep its ground contacts (either-side rule)";
    EXPECT_LT(scoped, full) << "scoping to one cube must shrink the contact emission";

    // Nothing in scope: contact viz must go silent entirely.
    ovx_primpath_t nothing_token{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/None", &nothing_token));
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, &nothing_token, 1u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after empty scope failed";
    EXPECT_EQ(e2e_total_prims(m_handle), 0u) << "no pair has a side in scope -> no contact prims";

    // Clearing restores the pairs (counts can drift a frame after resting
    // contacts resettle, so assert emission, not the exact number).
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, nullptr, 0u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after clearing scope failed";
    const uint32_t restored = e2e_total_prims(m_handle);
    EXPECT_GT(restored, scoped) << "clearing the scope must restore the out-of-scope pairs";

    // Suite-shared state hygiene (the scope is process-global like the mask).
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, 0.0f);
}


TEST_F(DebugRenderE2E, ScopeClearsAcrossStageReplacement)
{
    // Prim-path handles belong to one OVStage dictionary. Detach clears the
    // process-global scope before that dictionary can die, so a replacement
    // stage starts unscoped and cannot accidentally reuse stale numeric IDs.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "first attach failed";
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 1.0f);
    for (int i = 0; i < 3; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";
    const uint32_t full = e2e_total_prims(m_handle);
    ASSERT_GT(full, 0u) << "body axes must emit unscoped";
    ovx_primpath_t one_cube_token{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/World/Cube1", &one_cube_token));
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, &one_cube_token, 1u).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after scoping failed";
    const uint32_t scoped = e2e_total_prims(m_handle);
    EXPECT_GT(scoped, 0u);
    EXPECT_LT(scoped, full);

    // Fresh attach. Scene parameters died with the scene, so re-apply the
    // scale and parameter without reusing the old dictionary's path handle.
    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(e2e_wait(m_handle, reset.op_index)) << "reset wait failed";
    }
    test_utils::destroy_ovstage_test_attachments(m_handle);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "second attach failed";
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 1.0f);
    for (int i = 0; i < 3; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "post-reattach step failed";
    const uint32_t replacement = e2e_total_prims(m_handle);
    std::printf("[DebugRenderE2E::ScopeLifetime] full=%u scoped=%u replacement=%u\n",
                full, scoped, replacement);
    EXPECT_EQ(replacement, full)
        << "a replacement Stage must start with no stale path-handle scope";
}


// -------------------------------------------------------------------------
// GPU pass: SDF collider debug-viz emission.
//
// SDF cooking wants a CUDA context. In the cpu pass (OVPHYSX_DISABLE_GPU=1,
// GPU plugin dir off PATH on Windows) the CI runners fail PxCudaContextManager
// creation during the ovstage re-ingest and the cook task dies with
// bad_function_call, hanging ovphysx_update_from_ovstage until the pass
// timeout (observed on windows AND linux CI; not reproducible locally, where
// the CUDA-less cook falls back to CPU cleanly). This suite is named *GpuTest*
// so scripts/test_cpp.cmake routes it to the gpu pass: own process, GPU
// plugins on PATH, OVPHYSX_TEST_REQUIRE_CUDA=1 runners -- the environment
// where the cook path is known-good. Single long-lived instance per suite
// (Carbonite/Python cannot re-init in-process; same as TensorBindingGpuTest).
// Each attach phase prints before it runs so a CI hang names its exact site.
// -------------------------------------------------------------------------
class SdfDebugRenderGpuTest : public ::testing::Test
{
    static ovphysx_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
#if !OVPHYSX_ENABLE_GPU_TESTS
        s_skipReason = "GPU tests disabled at compile time";
        return;
#endif
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_result_t create_result = ovphysx_create_instance(&args, &s_handle);
        if (create_result.status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_string_t last_err = ovphysx_get_last_error();
            s_skipReason = last_err.length > 0 ? std::string(last_err.ptr, last_err.length)
                                               : "Failed to create instance";
            s_handle = 0;
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle != 0)
        {
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0)
            {
                ovphysx_op_wait_result_t wait_result{};
                ovphysx_wait_op(s_handle, reset_result.op_index, 10'000'000'000ULL, &wait_result);
                ovphysx_destroy_wait_result(&wait_result);
            }
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    void SetUp() override
    {
        if (s_handle == 0)
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "instance unavailable (OVPHYSX_TEST_REQUIRE_CUDA=1): " << s_skipReason;
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        m_handle = 0;
    }
};

ovphysx_handle_t SdfDebugRenderGpuTest::s_handle = 0;
std::string SdfDebugRenderGpuTest::s_skipReason;

TEST_F(SdfDebugRenderGpuTest, SDFExportsGeometry)
{
    std::printf("[SdfDebugRenderGpuTest] phase: attach_usd_with_ovstage\n");
    std::fflush(stdout);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/sdf_collision_scene.usda"))
        << "attach_usd_with_ovstage failed";

    std::printf("[SdfDebugRenderGpuTest] phase: set_scale + warm-up steps\n");
    std::fflush(stdout);
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // A couple of steps so the SDF-collider body settles; gives PhysX time to finalize
    // SDF cooking and any collision detection.
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "warm-up step failed";

    std::printf("[SdfDebugRenderGpuTest] phase: enable SDF param + step\n");
    std::fflush(stdout);
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_SDF, 1.0f);
    ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "SDF step failed";

    uint32_t total = e2e_total_prims(m_handle);
    std::printf("[SdfDebugRenderGpuTest] SDF parameter: %u prims\n", total);

    // SDF should emit a dense point cloud sampling the cooked field around the
    // SdfGeom cube. This is a strong assertion: if SDF cooking or eSDF visualization
    // regresses, this test will catch it.
    EXPECT_GT(total, 0u) << "SDF param should emit dense sample points when a cooked SDF collider is present";
}
// -------------------------------------------------------------------------
// GPU pass: friction-anchor emission under the direct-GPU API.
//
// Friction anchors (eFRICTION_POINT / eFRICTION_NORMAL / eFRICTION_IMPULSE) are a
// solver product. Under eENABLE_DIRECT_GPU_API the CPU contact viz early-outs, so
// their emission comes from PxgGpuNarrowphaseCore::drawFrictionAnchors (fetchResults
// time, from the GPU solver friction-patch stream). This suite creates a DirectGPU
// instance exactly like TensorBindingGpuTest (/physics/suppressReadback +
// /physics/suppressFabricUpdate before plugin load) and asserts that settled boxes
// in sustained ground contact emit friction primitives.
// -------------------------------------------------------------------------
class DirectGpuVizGpuTest : public ::testing::Test
{
    static ovphysx_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
#if !OVPHYSX_ENABLE_GPU_TESTS
        s_skipReason = "GPU tests disabled at compile time";
        return;
#endif
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t direct_gpu_entries[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"),
                                           OVPHYSX_LITERAL("true")),
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressFabricUpdate"),
                                           OVPHYSX_LITERAL("true")),
        };
        args.config_entries = direct_gpu_entries;
        args.config_entry_count = sizeof(direct_gpu_entries) / sizeof(direct_gpu_entries[0]);
        ovphysx_result_t create_result = ovphysx_create_instance(&args, &s_handle);
        if (create_result.status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_string_t last_err = ovphysx_get_last_error();
            s_skipReason = last_err.length > 0 ? std::string(last_err.ptr, last_err.length)
                                               : "Failed to create DirectGPU instance";
            s_handle = 0;
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle != 0)
        {
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0)
            {
                ovphysx_op_wait_result_t wait_result{};
                ovphysx_wait_op(s_handle, reset_result.op_index, 10'000'000'000ULL, &wait_result);
                ovphysx_destroy_wait_result(&wait_result);
            }
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    void SetUp() override
    {
        if (s_handle == 0)
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "instance unavailable (OVPHYSX_TEST_REQUIRE_CUDA=1): " << s_skipReason;
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        // Process-global viz state hygiene, exception-safe (see DebugRenderE2E).
        if (m_handle != 0)
        {
            ovphysx_debug_render_set_scope_tokens(m_handle, nullptr, 0u);
            for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
                ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
            ovphysx_debug_render_enable(m_handle, false);
        }
        // Reset the stage between tests (same pattern as TensorBindingGpuTest): the
        // instance is suite-shared, and a second attach on a populated stage fails.
        if (s_handle != 0)
        {
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0)
            {
                ovphysx_op_wait_result_t wait_result{};
                ovphysx_wait_op(s_handle, reset_result.op_index, 10'000'000'000ULL, &wait_result);
                ovphysx_destroy_wait_result(&wait_result);
            }
        }
        m_handle = 0;
    }
};

ovphysx_handle_t DirectGpuVizGpuTest::s_handle = 0;
std::string DirectGpuVizGpuTest::s_skipReason;

TEST_F(DirectGpuVizGpuTest, FrictionParamsExportPrimitives)
{
    std::printf("[DirectGpuVizGpuTest] phase: attach_usd_with_ovstage\n");
    std::fflush(stdout);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Settle the boxes into SUSTAINED ground contact: friction anchors only exist for
    // pairs the solver keeps active friction patches for.
    std::printf("[DirectGpuVizGpuTest] phase: settle steps\n");
    std::fflush(stdout);
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "settle step failed";

    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);

    struct FrictionVizParam
    {
        ovphysx_debug_render_parameter_t param;
        const char* name;
    };
    const FrictionVizParam kFriction[] = {
        { OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_POINT, "FrictionPoint" },
        { OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_NORMAL, "FrictionNormal" },
        { OVPHYSX_DEBUG_RENDER_PARAM_FRICTION_IMPULSE, "FrictionImpulse" },
    };
    uint32_t grand = 0;
    for (const FrictionVizParam& f : kFriction)
    {
        ovphysx_debug_render_set_parameter(m_handle, f.param, 1.0f);
        for (int i = 0; i < 2; ++i)
            ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed for " << f.name;
        const uint32_t t = e2e_total_prims(m_handle);
        grand += t;
        std::printf("[DirectGpuVizGpuTest] %-16s: %u prims\n", f.name, t);
        std::fflush(stdout);
        ovphysx_debug_render_set_parameter(m_handle, f.param, 0.0f);
    }

    // Triage probe (prints always): contact-point viz shares the NP-end capture
    // infrastructure (drawNewStreamContacts) while friction anchors draw at
    // fetchResults from the solver's friction patches. Contacts > 0 with
    // friction == 0 means the NP-end pair outputs are healthy and the
    // fetch-time friction path is the suspect; contacts == 0 means the pairs
    // produced no narrowphase output at all at measure time.
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 1.0f);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed for ContactPoint probe";
    const uint32_t contactPrims = e2e_total_prims(m_handle);
    std::printf("[DirectGpuVizGpuTest] ContactPoint    : %u prims (triage probe)\n", contactPrims);
    std::fflush(stdout);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 0.0f);
    EXPECT_GT(contactPrims, 0u) << "settled boxes must export contact points under the "
                                   "direct-GPU API (NP-end capture health check)";

    // Settled boxes rest on the ground through persistent friction anchors, so the
    // point/normal params must emit. (Impulse can legitimately be ~0 at rest and is
    // reported above but not asserted.)
#ifdef OVPHYSX_TEST_PHYSXGPU_DEBUG_VIZ
    EXPECT_GT(grand, 0u) << "friction params should export primitives for settled boxes "
                            "in sustained ground contact under the direct-GPU API";
#else
    // The friction-anchor emission lives in the PhysXGpu module and ships with THIS
    // MR: a prebuilt physxsdk package (what CI links; compiling the SDK's CUDA from
    // source is a multi-hour build the test jobs do not do) only gains it after the
    // MR merges and the package pin rolls. Until then the prebuilt module silently
    // no-ops the draw hook, so emission cannot be asserted here -- it IS asserted on
    // source-built (devphysx + CUDA) configurations via the branch above.
    if (grand == 0u)
        GTEST_SKIP() << "prebuilt PhysXGpu predates the DirectGPU friction-anchor emission "
                        "(hard-asserted when the loaded PhysXGpu has the feature: source-built or a new-enough package)";
    SUCCEED() << "friction params exported " << grand << " primitives (prebuilt PhysXGpu already has the feature)";
#endif
}
// GPU-bucket trimesh pairs under the direct-GPU API: convex-vs-triangle-mesh contact
// gen runs in the GPU buckets (convexMeshMidphase writes the same compressed streams),
// so drawNewStreamContacts should emit their contact points/normals like convex-convex.
// This pins that empirically. Known remaining gap (documented in drawNewStreamContacts):
// pairs that FALL BACK to CPU contact gen under DirectGPU are drawn by neither path.
TEST_F(DirectGpuVizGpuTest, TrimeshContactParamsExport)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/trimesh_ground_collisions.usda"))
        << "attach_usd_with_ovstage failed";

    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);

    // Let the box land on the triangle-mesh ground -> sustained convex-trimesh contact.
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "settle step failed";

    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 1.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_NORMAL, 1.0f);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step failed";

    const uint32_t total = e2e_total_prims(m_handle);
    std::printf("[DirectGpuVizGpuTest] trimesh contact point+normal: %u prims\n", total);
    std::fflush(stdout);
    EXPECT_GT(total, 0u) << "convex-vs-trimesh contacts should emit under the direct-GPU API "
                            "(GPU bucket pairs flow through drawNewStreamContacts)";
}

// The visualization scope must also govern the DirectGPU contact emission: the
// NP-end capture (drawNewStreamContacts) gates each GPU pair on its shapes'
// eVISUALIZATION flags (isPairShapeVisualized), the low-level view of the same
// actor+shape authoring the CPU gate in ShapeInteraction::visualize checks.
// DirectGPU sibling of DebugRenderE2E.ScopeRestrictsContactViz.
TEST_F(DirectGpuVizGpuTest, ScopeRestrictsContactViz)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "attach_usd_with_ovstage failed";
    ASSERT_EQ(ovphysx_debug_render_set_scale(m_handle, 1.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_debug_render_enable(m_handle, true).status, OVPHYSX_API_SUCCESS);
    for (int p = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES; p < OVPHYSX_DEBUG_RENDER_PARAM_COUNT; ++p)
        ovphysx_debug_render_set_parameter(m_handle, (ovphysx_debug_render_parameter_t)p, 0.0f);
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 1.0f);
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "settle step failed";
    const uint32_t full = e2e_total_prims(m_handle);
#ifndef OVPHYSX_TEST_PHYSXGPU_DEBUG_VIZ
    // The DirectGPU contact SCOPE GATE (isPairShapeVisualized at the NP-end capture)
    // ships in the PhysXGpu module with THIS MR. A prebuilt physxsdk package (what CI
    // links) already carries the contact emission (so full > 0) but NOT
    // this MR's scope gate, so scoping cannot shrink the capture until the package pin
    // rolls. Assert the scope behavior only where OVPHYSX_TEST_PHYSXGPU_DEBUG_VIZ is
    // defined (source-built devphysx + CUDA, or a package passing the marker probe);
    // the friction DirectGPU test gates the same way. Without it the
    // runtime full==0 check below cannot tell the scope-gate sub-feature apart from
    // the contact emission it already has.
    GTEST_SKIP() << "prebuilt PhysXGpu predates the DirectGPU contact scope gate "
                    "(asserted on source-built configurations)";
#endif
    if (full == 0u)
        GTEST_SKIP() << "no contacts emitted (prebuilt PhysXGpu predates the DirectGPU contact emission)";

    ovx_primpath_t one_cube_token{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/World/Cube1", &one_cube_token));
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, &one_cube_token, 1u).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after scoping failed";
    const uint32_t scoped = e2e_total_prims(m_handle);
    std::printf("[DirectGpuVizGpuTest] contact prims full=%u scoped(one cube)=%u\n", full, scoped);
    std::fflush(stdout);
    EXPECT_GT(scoped, 0u) << "the scoped cube must keep its ground contacts (either-side rule)";
    EXPECT_LT(scoped, full) << "scoping to one cube must shrink the GPU contact capture";

    ovx_primpath_t nothing_token{};
    ASSERT_TRUE(e2e_intern_path(m_handle, "/None", &nothing_token));
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, &nothing_token, 1u).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after empty scope failed";
    EXPECT_EQ(e2e_total_prims(m_handle), 0u) << "no pair has a side in scope -> no captured contacts";

    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(m_handle, nullptr, 0u).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 2; ++i)
        ASSERT_TRUE(e2e_step(m_handle, 1.0f / 60.0f)) << "step after clearing scope failed";
    EXPECT_GT(e2e_total_prims(m_handle), scoped) << "clearing the scope must restore the out-of-scope pairs";

    // Suite-shared state hygiene (the scope is process-global like the mask).
    ovphysx_debug_render_set_parameter(m_handle, OVPHYSX_DEBUG_RENDER_PARAM_CONTACT_POINT, 0.0f);
}
