// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// physxPointer is a read-only host uint64 column. A missing object is absent
// (no row). A query type without the column yields no group. The address is
// stable across step and cannot be reacquired after reset or detach.

/**
 * @implements REQ-CAPI-PHYSXPTR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <string>

using namespace test_utils;

namespace {

bool wait_task_success(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
                       uint64_t timeout_ns = 10'000'000'000ULL)
{
    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_task(handle, task, timeout_ns, &wait_result);
    ovphysx_destroy_wait_task_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS;
}

bool step_and_wait(ovphysx_instance_handle_t handle, float dt)
{
    ovphysx_enqueue_result_t res = ovphysx_step(handle, dt);
    return res.status == OVPHYSX_API_SUCCESS && wait_task_success(handle, res);
}

} // namespace

TEST_F(PhysXTestFixture, PhysicsPointerRequiresAnAttachedStage)
{
    EXPECT_EQ(read_physics_singleton(m_handle), nullptr);
}

class PhysxPtrRigidBodyTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrRigidBodyTest, PhysicsSingletonIsOneRootPathRow)
{
    const std::map<std::string, void*> rows = read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_PHYSICS);
    ASSERT_EQ(rows.size(), 1u);
    ASSERT_NE(rows.begin()->second, nullptr);
    EXPECT_EQ(rows.begin()->first, "/");

    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    EXPECT_EQ(read_physics_singleton(m_handle), rows.begin()->second);
}

TEST_F(PhysxPtrRigidBodyTest, SceneAndActorPointers)
{
    EXPECT_NE(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene"), nullptr);
    EXPECT_NE(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"), nullptr);
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/NoSuchPrim_xyz123"), nullptr);
}

TEST_F(PhysxPtrRigidBodyTest, SceneStableAcrossSteps)
{
    void* before = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene");
    ASSERT_NE(before, nullptr);
    for (int i = 0; i < 10; ++i)
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene"), before);
}

TEST_F(PhysxPtrRigidBodyTest, DistinctActorsHaveDistinctPointers)
{
    void* a = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    void* b = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NE(a, b);
}

TEST_F(PhysxPtrRigidBodyTest, ShapeColumnIsNonEmpty)
{
    EXPECT_FALSE(read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_SHAPE).empty());
}

TEST_F(PhysxPtrRigidBodyTest, RigidBodyQueryHasNoPointerColumn)
{
    EXPECT_TRUE(read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY).empty());
}

TEST_F(PhysxPtrRigidBodyTest, ParticleQueriesAreEmptyOnARigidScene)
{
    EXPECT_TRUE(read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM).empty());
    EXPECT_TRUE(read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET).empty());
    EXPECT_TRUE(read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_CUSTOM_JOINT).empty());
}

TEST_F(PhysxPtrRigidBodyTest, PointersAreAbsentAfterResetAndDetach)
{
    ASSERT_NE(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"), nullptr);
    ASSERT_NE(read_physics_singleton(m_handle), nullptr);

    ovphysx_enqueue_result_t reset_res = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset_res.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, reset_res));
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"), nullptr);
    EXPECT_EQ(read_physics_singleton(m_handle), nullptr);

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    ASSERT_NE(read_physics_singleton(m_handle), nullptr);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(read_physics_singleton(m_handle), nullptr);
}

class PhysxPtrMaterialTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/kinematic_support.usda"));
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrMaterialTest, MaterialPointer)
{
    EXPECT_NE(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_PHYSX_MATERIAL, "/World/HighFriction"), nullptr);
}

class PhysxPtrArticulationTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"));
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrArticulationTest, ArticulationLinkAndJointPointers)
{
    void* articulation = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, "/World/articulation");
    void* other = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, "/World/articulation2");
    ASSERT_NE(articulation, nullptr);
    ASSERT_NE(other, nullptr);
    EXPECT_NE(articulation, other);
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, "/World/articulation"), articulation);

    void* link0 = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, "/World/articulation/articulationLink0");
    void* link1 = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, "/World/articulation/articulationLink1");
    ASSERT_NE(link0, nullptr);
    ASSERT_NE(link1, nullptr);
    EXPECT_NE(link0, link1);

    const char* joint = "/World/articulation/articulatedRevoluteJoint1";
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_JOINT, joint), nullptr);
    EXPECT_NE(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, joint), nullptr);
}

class PhysxPtrStandaloneJointTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/revolute_joint_scene.usda"));
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrStandaloneJointTest, JointPointerIsStable)
{
    void* joint = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_JOINT, "/World/revoluteJoint");
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_JOINT, "/World/revoluteJoint"), joint);
}

class PhysxPtrCompoundShapeTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/convex_decomposition_shape.usda"));
        ASSERT_TRUE(step_and_wait(m_handle, 1.0f / 60.0f));
    }
};

TEST_F(PhysxPtrCompoundShapeTest, CompoundShapeColumnIsNonEmpty)
{
    EXPECT_FALSE(read_physx_pointers(m_handle, OVPHYSX_QUERY_TARGET_COMPOUND_SHAPE).empty());
}

#if OVPHYSX_ENABLE_GPU_TESTS

class PhysxPtrParticleGpuTest : public ::testing::Test
{
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
        ovphysx_result_t create_result = ovphysx_create_instance(&s_handle);
        bool cpuOnly = false;
        if (create_result.status == OVPHYSX_API_SUCCESS &&
            ovphysx_get_cpu_mode(&cpuOnly).status == OVPHYSX_API_SUCCESS && cpuOnly)
        {
            s_skipReason = "particles require GPU";
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            return;
        }
        if (create_result.status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_string_t last_err = ovphysx_get_last_error();
            s_skipReason = last_err.length > 0 ? std::string(last_err.ptr, last_err.length)
                                               : std::string("Failed to create GPU instance");
            s_handle = 0;
            return;
        }
        if (!attach_usd_with_ovstage(s_handle, "tests/data/particles_simple.usda") ||
            !step_and_wait(s_handle, 1.0f / 60.0f))
        {
            s_skipReason = "Failed to load or step particles_simple.usda";
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle != 0)
        {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    void SetUp() override
    {
        if (s_handle == 0)
            GTEST_SKIP() << s_skipReason;
        m_handle = s_handle;
    }
};

ovphysx_instance_handle_t PhysxPtrParticleGpuTest::s_handle = 0;
std::string PhysxPtrParticleGpuTest::s_skipReason;

TEST_F(PhysxPtrParticleGpuTest, ParticleSystemAndSetPointers)
{
    void* system = read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM, "/World/particleSystem");
    ASSERT_NE(system, nullptr);
    EXPECT_EQ(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM, "/World/particleSystem"), system);
    EXPECT_NE(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/particles"), nullptr);
}

#endif
