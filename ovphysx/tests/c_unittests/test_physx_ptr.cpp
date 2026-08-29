// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Tests for ovphysx_get_physx_ptr(handle, prim_path, physx_type, out_ptr).
//
// Coverage split:
//   - ABI / argument-validation tests: null prim_path, null out_ptr,
//     empty prim_path, invalid instance handle.
//   - Happy-path lookups per ovphysx_physx_type_t:
//       * CPU fixture: SCENE, ACTOR, ARTICULATION, LINK, LINK_JOINT,
//         standalone JOINT.
//       * GPU fixture (compiled in iff OVPHYSX_ENABLE_GPU_TESTS, skipped at
//         runtime if no CUDA): PARTICLE_SYSTEM, PARTICLE_SET. Particles are
//         GPU-only in the omni.physx runtime.
//   - Type-mismatch lookups: JOINT on an articulation joint, PARTICLE_*
//     on a rigid-body scene -- both return NOT_FOUND, not crash.
//   - Pointer identity properties: stable across ovphysx_step(), distinct
//     between distinct prims of the same type.
//   - Lifecycle: pointers become unreachable after ovphysx_reset_stage(); out_ptr
//     is not clobbered on failure.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

using namespace test_utils;

// Local helpers (same pattern as test_tensor_binding.cpp / test_clone.cpp).
namespace {

bool wait_op_success(ovphysx_handle_t handle, ovphysx_op_index_t op_index,
                     uint64_t timeout_ns = 10'000'000'000ULL) {
    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_op(handle, op_index, timeout_ns, &wait_result);
    ovphysx_destroy_wait_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS;
}

bool load_usd_and_wait(ovphysx_handle_t handle, const char* usd_path,
                       ovphysx_usd_handle_t& out_handle) {
    out_handle = 1;
    return attach_usd_with_ovstage(handle, usd_path);
}

bool step_and_wait(ovphysx_handle_t handle, float dt) {
    ovphysx_enqueue_result_t res = ovphysx_step(handle, dt);
    return res.status == OVPHYSX_API_SUCCESS && wait_op_success(handle, res.op_index);
}

ovphysx_result_t get_ptr(ovphysx_handle_t handle, const char* path,
                         ovphysx_physx_type_t type, void*& out) {
    out = nullptr;
    return ovphysx_get_physx_ptr(handle, make_ovx_string(path), type, &out);
}

} // namespace

// ============================================================================
// Argument-validation tests (no USD load required)
// ============================================================================

TEST_F(PhysXTestFixture, GetPhysxPtrNullPrimPath) {
    void* ptr = nullptr;
    ovphysx_result_t r = ovphysx_get_physx_ptr(
        m_handle, make_ovx_string(nullptr), OVPHYSX_PHYSX_TYPE_SCENE, &ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(PhysXTestFixture, GetPhysxPtrEmptyPrimPath) {
    void* ptr = nullptr;
    ovphysx_result_t r = ovphysx_get_physx_ptr(
        m_handle, make_ovx_string(""), OVPHYSX_PHYSX_TYPE_SCENE, &ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// A prim path carrying an embedded NUL byte must be rejected outright, not
// silently truncated to a shorter (possibly valid) path. Regression for
// NVBugs 6433621.
TEST_F(PhysXTestFixture, GetPhysxPtrEmbeddedNulPrimPath) {
    std::string storage;
    ovphysx_string_t path = make_ovx_string_bytes(
        std::string("/World/physicsScene") + '\0' + "GARBAGE", storage);
    void* ptr = nullptr;
    ovphysx_result_t r = ovphysx_get_physx_ptr(
        m_handle, path, OVPHYSX_PHYSX_TYPE_SCENE, &ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ptr, nullptr);
}

TEST_F(PhysXTestFixture, GetPhysxPtrNullOutPtr) {
    ovphysx_result_t r = ovphysx_get_physx_ptr(
        m_handle, make_ovx_string("/World/physicsScene"), OVPHYSX_PHYSX_TYPE_SCENE, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(PhysXTestFixture, GetPhysxPtrInvalidInstanceHandle) {
    void* ptr = nullptr;
    ovphysx_result_t r = ovphysx_get_physx_ptr(
        OVPHYSX_INVALID_HANDLE, make_ovx_string("/World/physicsScene"), OVPHYSX_PHYSX_TYPE_SCENE, &ptr);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// ============================================================================
// Happy-path matrix — rigid-body scene
// ============================================================================

class PhysxPtrRigidBodyTest : public PhysXTestFixture {
protected:
    void SetUp() override {
        PhysXTestFixture::SetUp();
        ovphysx_usd_handle_t usd = 0;
        ASSERT_TRUE(load_usd_and_wait(
            m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd))
            << "Failed to load rigid-body scene";
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrRigidBodyTest, SceneNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/physicsScene",
                                 OVPHYSX_PHYSX_TYPE_SCENE, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

TEST_F(PhysxPtrRigidBodyTest, SceneStableAcrossSteps) {
    void* before = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/physicsScene",
                      OVPHYSX_PHYSX_TYPE_SCENE, before).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(before, nullptr);

    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }

    void* after = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/physicsScene",
                      OVPHYSX_PHYSX_TYPE_SCENE, after).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(before, after) << "PxScene pointer must be stable across steps";
}

TEST_F(PhysxPtrRigidBodyTest, ActorNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/Cube1",
                                 OVPHYSX_PHYSX_TYPE_ACTOR, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

// Asking for a particle type on a non-particle scene must report NOT_FOUND
// without crashing. The output pointer is left as the caller-initialized
// value (nullptr here).
TEST_F(PhysxPtrRigidBodyTest, ParticleSystemNotFoundOnRigidBodyScene) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/Cube1",
                                 OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ptr, nullptr);
}

TEST_F(PhysxPtrRigidBodyTest, ParticleSetNotFoundOnRigidBodyScene) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/Cube1",
                                 OVPHYSX_PHYSX_TYPE_PARTICLE_SET, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ptr, nullptr);
}

TEST_F(PhysxPtrRigidBodyTest, NonExistentPathReturnsNotFound) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/NoSuchPrim_xyz123",
                                 OVPHYSX_PHYSX_TYPE_ACTOR, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ptr, nullptr);
}

// After reset(), the prim no longer exists on a (now-detached) stage.
// ovphysx_get_physx_ptr must report a non-SUCCESS status, not crash, and
// not return a stale pointer.
TEST_F(PhysxPtrRigidBodyTest, PointerInvalidAfterReset) {
    void* before = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/Cube1",
                      OVPHYSX_PHYSX_TYPE_ACTOR, before).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(before, nullptr);

    ovphysx_enqueue_result_t reset_res = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_op_success(m_handle, reset_res.op_index));

    // out_ptr must NOT receive a stale pointer on failure. The C API contract
    // is: on the "no USD stage loaded" error path, *out_ptr is left untouched
    // (validateInteropArgs returns before any write). Initialize to nullptr and
    // assert it stayed there — if the API ever leaked the prior pointer through
    // the out param, this test would catch it.
    void* after = nullptr;
    ovphysx_result_t r = ovphysx_get_physx_ptr(
        m_handle, make_ovx_string("/World/Cube1"), OVPHYSX_PHYSX_TYPE_ACTOR, &after);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(after, nullptr);
}

// ============================================================================
// Happy-path matrix — articulation scene
// ============================================================================

class PhysxPtrArticulationTest : public PhysXTestFixture {
protected:
    void SetUp() override {
        PhysXTestFixture::SetUp();
        ovphysx_usd_handle_t usd = 0;
        ASSERT_TRUE(load_usd_and_wait(
            m_handle, "tests/data/two_articulations.usda", usd))
            << "Failed to load articulation scene";
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrArticulationTest, ArticulationNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/articulation",
                                 OVPHYSX_PHYSX_TYPE_ARTICULATION, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

TEST_F(PhysxPtrArticulationTest, ArticulationStablePointer) {
    void* a = nullptr;
    void* b = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/articulation",
                      OVPHYSX_PHYSX_TYPE_ARTICULATION, a).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(get_ptr(m_handle, "/World/articulation",
                      OVPHYSX_PHYSX_TYPE_ARTICULATION, b).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(a, b);
}

TEST_F(PhysxPtrArticulationTest, DistinctArticulationsHaveDistinctPointers) {
    void* a = nullptr;
    void* b = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/articulation",
                      OVPHYSX_PHYSX_TYPE_ARTICULATION, a).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(get_ptr(m_handle, "/World/articulation2",
                      OVPHYSX_PHYSX_TYPE_ARTICULATION, b).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NE(a, b);
}

TEST_F(PhysxPtrArticulationTest, LinkNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle,
                                 "/World/articulation/articulationLink0",
                                 OVPHYSX_PHYSX_TYPE_LINK, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

TEST_F(PhysxPtrArticulationTest, DistinctLinksHaveDistinctPointers) {
    void* a = nullptr;
    void* b = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/articulation/articulationLink0",
                      OVPHYSX_PHYSX_TYPE_LINK, a).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(get_ptr(m_handle, "/World/articulation/articulationLink1",
                      OVPHYSX_PHYSX_TYPE_LINK, b).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NE(a, b);
}

// Articulation joints are PxArticulationJointReducedCoordinate; querying
// them via the plain JOINT (PxJoint) type returns NOT_FOUND.
TEST_F(PhysxPtrArticulationTest, JointTypeOnArticulationJointReturnsNotFound) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(
        m_handle, "/World/articulation/articulatedRevoluteJoint1",
        OVPHYSX_PHYSX_TYPE_JOINT, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ptr, nullptr);
}

// Same prim, LINK_JOINT type: returns the
// PxArticulationJointReducedCoordinate.
TEST_F(PhysxPtrArticulationTest, LinkJointOnArticulationJointNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(
        m_handle, "/World/articulation/articulatedRevoluteJoint1",
        OVPHYSX_PHYSX_TYPE_LINK_JOINT, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

// ============================================================================
// Happy-path matrix — standalone (non-articulation) joint
//
// revolute_joint_scene.usda has /World/revoluteJoint, a PhysicsRevoluteJoint
// outside any articulation. This is a PxJoint, so OVPHYSX_PHYSX_TYPE_JOINT
// must resolve to a non-null pointer.
// ============================================================================

class PhysxPtrStandaloneJointTest : public PhysXTestFixture {
protected:
    void SetUp() override {
        PhysXTestFixture::SetUp();
        ovphysx_usd_handle_t usd = 0;
        ASSERT_TRUE(load_usd_and_wait(
            m_handle, "tests/data/revolute_joint_scene.usda", usd))
            << "Failed to load standalone-joint scene";
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrStandaloneJointTest, JointNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/revoluteJoint",
                                 OVPHYSX_PHYSX_TYPE_JOINT, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

TEST_F(PhysxPtrStandaloneJointTest, JointStablePointer) {
    void* a = nullptr;
    void* b = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/revoluteJoint",
                      OVPHYSX_PHYSX_TYPE_JOINT, a).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(get_ptr(m_handle, "/World/revoluteJoint",
                      OVPHYSX_PHYSX_TYPE_JOINT, b).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(a, b);
}

// ============================================================================
// Happy-path matrix — particle scene (GPU-only)
//
// Particles are GPU-only on the omni.physx runtime side: the load path checks
// `physxScene->isFullGpuPipelineAvailable()` and refuses to register particle
// objects on a CPU instance. The PhysXTestFixture's shared instance is CPU,
// so these tests get their own GPU instance.
//
// Builds compiled with -DOVPHYSX_ENABLE_GPU_TESTS=0 skip both tests at compile
// time. At runtime, if GPU/CUDA isn't available the suite SetUp leaves the
// handle null and each test is skipped with a clear reason.
// ============================================================================

#if OVPHYSX_ENABLE_GPU_TESTS

class PhysxPtrParticleGpuTest : public ::testing::Test {
    static ovphysx_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_handle_t m_handle = 0;

    static void SetUpTestSuite() {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        ovphysx_result_t create_result = ovphysx_create_instance(&args, &s_handle);
        if (create_result.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t last_err = ovphysx_get_last_error();
            s_skipReason = last_err.length > 0
                ? std::string(last_err.ptr, last_err.length)
                : std::string("Failed to create GPU instance");
            s_handle = 0;
            return;
        }

        ovphysx_usd_handle_t usd = 0;
        if (!load_usd_and_wait(s_handle, "tests/data/particles_simple.usda", usd)) {
            s_skipReason = "Failed to load particles_simple.usda on GPU instance";
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            return;
        }
        if (!step_and_wait(s_handle, 1.0f / 60.0f)) {
            s_skipReason = "Step failed on particle scene";
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            return;
        }
    }

    static void TearDownTestSuite() {
        if (s_handle != 0) {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    void SetUp() override {
        if (s_handle == 0) {
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }
};

ovphysx_handle_t PhysxPtrParticleGpuTest::s_handle = 0;
std::string PhysxPtrParticleGpuTest::s_skipReason;

TEST_F(PhysxPtrParticleGpuTest, ParticleSystemNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/particleSystem",
                                 OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

TEST_F(PhysxPtrParticleGpuTest, ParticleSystemStablePointer) {
    void* a = nullptr;
    void* b = nullptr;
    ASSERT_EQ(get_ptr(m_handle, "/World/particleSystem",
                      OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM, a).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(get_ptr(m_handle, "/World/particleSystem",
                      OVPHYSX_PHYSX_TYPE_PARTICLE_SYSTEM, b).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(a, b);
}

TEST_F(PhysxPtrParticleGpuTest, ParticleSetNonZero) {
    void* ptr = nullptr;
    ovphysx_result_t r = get_ptr(m_handle, "/World/particles",
                                 OVPHYSX_PHYSX_TYPE_PARTICLE_SET, ptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ptr, nullptr);
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
