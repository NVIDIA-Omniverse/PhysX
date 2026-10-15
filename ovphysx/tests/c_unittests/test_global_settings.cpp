// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Tests for:
//   scoped configuration and legacy typed config compatibility

/**
 * @implements REQ-CAPI-STRING-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysxTestHelpers.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include "ovphysx_test_utils.h"
#include <PxShape.h>
#include <cstring>
#include <string>
#include <iostream>
#include <vector>

using namespace test_utils;

namespace
{
class ScopedCookedCachePathRestore
{
public:
    explicit ScopedCookedCachePathRestore(const std::string& value)
        : m_value(value)
    {
    }

    ~ScopedCookedCachePathRestore()
    {
        const ovphysx_string_t value = { m_value.data(), m_value.size() };
        (void)ovphysx_set_raw_string_setting_for_test_internal(
            "/UJITSO/datastore/localCachePath", value.ptr, value.length);
    }

private:
    std::string m_value;
};
}

static bool wait_gs_op(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_wait_task_result_t wr{};
    ovphysx_result_t r = ovphysx_wait_task(handle, task, 10'000'000'000ULL, &wr);
    ovphysx_destroy_wait_task_result(&wr);
    return r.status == OVPHYSX_API_SUCCESS;
}

// ============================================================================
// Typed Config API Tests
// ============================================================================

TEST(ScopedConfig, KeyRangesDoNotOverlap)
{
    EXPECT_LE(
        static_cast<uint32_t>(OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE),
        0x0000ffffu);
    EXPECT_GE(static_cast<uint32_t>(OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS), 0x00010000u);
}

TEST(ScopedConfig, GlobalTypeRangeAndFailureAtomicity)
{
    ovphysx_config_value_t value = ovphysx_config_value_int64(64);
    ASSERT_EQ(
        ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB, value).status,
        OVPHYSX_API_SUCCESS);

    ovphysx_config_value_t out = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &out).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(out.type, OVPHYSX_CONFIG_VALUE_INT64);
    EXPECT_EQ(out.value.int64_value, 64);

    EXPECT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB,
            ovphysx_config_value_bool(true)).status,
        OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB,
            ovphysx_config_value_int64(static_cast<int64_t>(INT32_MAX) + 1)).status,
        OVPHYSX_API_INVALID_ARGUMENT);

    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out.value.int64_value, 64);
}

TEST(ScopedConfig, CrossScopeCastIsRejected)
{
    EXPECT_EQ(
        ovphysx_set_global_config(
            static_cast<ovphysx_global_config_key_t>(OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS),
            ovphysx_config_value_string(OVPHYSX_LITERAL("0"))).status,
        OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ScopedConfig, InstanceStringIsCopiedAndUsesCallerBuffer)
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);

    ovphysx_config_value_t out = {};
    out.type = OVPHYSX_CONFIG_VALUE_STRING;
    EXPECT_EQ(
        ovphysx_get_config(handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, &out).status,
        OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(
        ovphysx_set_config(
            handle,
            static_cast<ovphysx_config_key_t>(
                OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING),
            ovphysx_config_value_bool(true)).status,
        OVPHYSX_API_INVALID_ARGUMENT);

    char source[] = "0";
    ASSERT_EQ(
        ovphysx_set_config(
            handle,
            OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
            ovphysx_config_value_string({source, 1})).status,
        OVPHYSX_API_SUCCESS);
    source[0] = '7';

    EXPECT_EQ(
        ovphysx_get_config(handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, &out).status,
        OVPHYSX_API_BUFFER_TOO_SMALL);
    ASSERT_EQ(out.value.string_value.length, 2u);

    char buffer[2] = {};
    out.value.string_value = {buffer, sizeof(buffer)};
    ASSERT_EQ(
        ovphysx_get_config(handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(std::string(buffer, out.value.string_value.length), "0");

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, InstanceWriteAfterAttachPreservesPriorValue)
{
    destroySharedCpuInstance();

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(
        ovphysx_set_config(
            handle,
            OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
            ovphysx_config_value_string(OVPHYSX_LITERAL("0"))).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(
        handle,
        OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));

    EXPECT_EQ(
        ovphysx_set_config(
            handle,
            OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
            ovphysx_config_value_string(OVPHYSX_LITERAL("-1"))).status,
        OVPHYSX_API_INVALID_STATE);

    char buffer[2] = {};
    ovphysx_config_value_t out = {};
    out.type = OVPHYSX_CONFIG_VALUE_STRING;
    out.value.string_value = {buffer, sizeof(buffer)};
    ASSERT_EQ(
        ovphysx_get_config(handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(std::string(buffer, out.value.string_value.length), "0");

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, LegacyCreateValueIsVisibleThroughScopedGetter)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.active_cuda_gpus = OVPHYSX_LITERAL("-1");

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(
        ovphysx_create_instance_with_args(&args, &handle).status,
        OVPHYSX_API_SUCCESS);

    char buffer[3] = {};
    ovphysx_config_value_t out = {};
    out.type = OVPHYSX_CONFIG_VALUE_STRING;
    out.value.string_value = {buffer, sizeof(buffer)};
    ASSERT_EQ(
        ovphysx_get_config(handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(std::string(buffer, out.value.string_value.length), "-1");

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, LegacyEntryUpdatesRetainedGlobalValue)
{
    destroySharedCpuInstance();
    ASSERT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(4)).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(
        ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(6)).status,
        OVPHYSX_API_SUCCESS);

    ovphysx_instance_handle_t first = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&first).status, OVPHYSX_API_SUCCESS);
    ovphysx_config_value_t out = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out.value.int64_value, 6);
    ASSERT_EQ(ovphysx_destroy_instance(first).status, OVPHYSX_API_SUCCESS);

    ovphysx_instance_handle_t second = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&second).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out.value.int64_value, 6);
    ASSERT_EQ(ovphysx_destroy_instance(second).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(
        ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(8)).status,
        OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, CarboniteNumThreadsPathWithOrWithoutSlashSurvivesNextCreate)
{
    destroySharedCpuInstance();
    ovphysx_instance_handle_t warmup = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&warmup).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(warmup).status, OVPHYSX_API_SUCCESS);

    const char* paths[] = {"persistent/physics/numThreads", "/persistent/physics/numThreads"};
    const char* values[] = {"6", "7"};
    const int64_t expected[] = {6, 7};
    for (int i = 0; i < 2; ++i)
    {
        ASSERT_EQ(
            ovphysx_set_global_config(
                OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(4)).status,
            OVPHYSX_API_SUCCESS);
        const ovphysx_string_t path = {paths[i], std::strlen(paths[i])};
        const ovphysx_string_t value = {values[i], std::strlen(values[i])};
        ASSERT_EQ(
            ovphysx_set_global_config_entry(
                ovphysx_config_entry_carbonite(path, value)).status,
            OVPHYSX_API_SUCCESS);

        ovphysx_config_value_t out = {};
        ASSERT_EQ(
            ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
            OVPHYSX_API_SUCCESS);
        EXPECT_EQ(out.value.int64_value, expected[i]);

        ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
        ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(
            ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
            OVPHYSX_API_SUCCESS);
        EXPECT_EQ(out.value.int64_value, expected[i]);
        ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    }

    ASSERT_EQ(
        ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(8)).status,
        OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, LegacyNumThreadsRejectsOutOfRange)
{
    destroySharedCpuInstance();
    ovphysx_instance_handle_t warmup = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&warmup).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(warmup).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(4)).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(
        ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(256)).status,
        OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_config_value_t out = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out.value.int64_value, 4);

    ASSERT_EQ(
        ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(8)).status,
        OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, LegacyNumThreadsRejectedWhileStageAttached)
{
    ovphysx_config_value_t current = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &current).status,
        OVPHYSX_API_SUCCESS);
    const int64_t prior = current.value.int64_value;
    const int32_t rejected = prior == 0 ? 1 : 0;

    destroySharedCpuInstance();
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(
        handle,
        OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));

    EXPECT_EQ(
        ovphysx_set_global_config_entry(ovphysx_config_entry_num_threads(rejected)).status,
        OVPHYSX_API_INVALID_STATE);
    ovphysx_config_value_t out = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out.value.int64_value, prior);

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, LegacyProfilerBackendRejectedAfterCreate)
{
    destroySharedCpuInstance();
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(
        ovphysx_set_global_config_entry(
            ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NVTX)).status,
        OVPHYSX_API_INVALID_STATE);
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

TEST(ScopedConfig, CustomConeGeometryInvertsApproximateSetting)
{
    destroySharedCpuInstance();
    ASSERT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY,
            ovphysx_config_value_bool(false)).status,
        OVPHYSX_API_SUCCESS);

    bool approximate = false;
    ASSERT_EQ(
        ovphysx_test_get_settings_bool("/physics/collisionApproximateCones", &approximate).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(approximate);

    ovphysx_config_value_t out = {};
    ASSERT_EQ(
        ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(out.value.bool_value);

    bool legacy = true;
    ASSERT_EQ(
        ovphysx_get_global_config_bool(
            OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, &legacy).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(legacy);

    ASSERT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY,
            ovphysx_config_value_bool(true)).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(
        ovphysx_test_get_settings_bool("/physics/collisionApproximateCones", &approximate).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(approximate);
    ASSERT_EQ(
        ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(out.value.bool_value);
}

class CollisionGeometryConfigTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        ASSERT_EQ(ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, &m_cone).status,
            OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY, &m_cylinder).status,
            OVPHYSX_API_SUCCESS);
        m_saved = true;
        PhysXTestFixture::SetUp();
    }

    void TearDown() override
    {
        PhysXTestFixture::TearDown();
        if (m_saved)
        {
            EXPECT_EQ(ovphysx_set_global_config(
                OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY, m_cone).status,
                OVPHYSX_API_SUCCESS);
            EXPECT_EQ(ovphysx_set_global_config(
                OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY, m_cylinder).status,
                OVPHYSX_API_SUCCESS);
        }
    }

private:
    ovphysx_config_value_t m_cone{};
    ovphysx_config_value_t m_cylinder{};
    bool m_saved = false;
};

TEST_F(CollisionGeometryConfigTest, SelectsConeAndCylinderShapeGeometry)
{
    for (bool coneCustom : { true, false })
    {
        SCOPED_TRACE(coneCustom);
        ASSERT_EQ(ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY,
            ovphysx_config_value_bool(coneCustom)).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY,
            ovphysx_config_value_bool(!coneCustom)).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/cone_and_cylinder.usda"));

        physx::PxShape* cone = static_cast<physx::PxShape*>(
            read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SHAPE, "/World/Cone"));
        physx::PxShape* cylinder = static_cast<physx::PxShape*>(
            read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SHAPE, "/World/Cylinder"));
        ASSERT_NE(cone, nullptr);
        ASSERT_NE(cylinder, nullptr);
        EXPECT_EQ(cone->getGeometry().getType(), coneCustom ?
            physx::PxGeometryType::eCONVEXCORE : physx::PxGeometryType::eCONVEXMESH);
        EXPECT_EQ(cylinder->getGeometry().getType(), coneCustom ?
            physx::PxGeometryType::eCONVEXMESH : physx::PxGeometryType::eCONVEXCORE);

        ASSERT_TRUE(destroy_ovstage_test_attachments(m_handle));
    }
}

TEST(ScopedConfig, NumThreadsRejectedWhileStageAttached)
{
    ovphysx_config_value_t current = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &current).status,
        OVPHYSX_API_SUCCESS);
    const int64_t prior = current.value.int64_value;
    const int64_t rejected = prior == 0 ? 1 : 0;

    destroySharedCpuInstance();
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(
        handle,
        OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda"));

    EXPECT_EQ(
        ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(rejected)).status,
        OVPHYSX_API_INVALID_STATE);
    ovphysx_config_value_t out = {};
    ASSERT_EQ(
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &out).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out.value.int64_value, prior);

    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

// set_global_config with a typed bool entry must succeed.
TEST(TypedConfig, SetBoolSucceeds)
{
    ovphysx_result_t r = ovphysx_set_global_config_entry(
        ovphysx_config_entry_disable_contact_processing(false));
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS)
        << "set_global_config(bool) must succeed for a known-writable key";
}

// Bool roundtrip: set then get.
TEST(TypedConfig, BoolRoundTrip)
{
    ovphysx_result_t r = ovphysx_set_global_config_entry(
        ovphysx_config_entry_disable_contact_processing(true));
    if (r.status != OVPHYSX_API_SUCCESS)
        GTEST_SKIP() << "Setting not writable; skipping round-trip test";

    bool out = false;
    r = ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, &out);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(out) << "Bool value must round-trip correctly";

    // Toggle back
    r = ovphysx_set_global_config_entry(
        ovphysx_config_entry_disable_contact_processing(false));
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, &out);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(out) << "Bool value must round-trip correctly after toggle";
}

// Int32 roundtrip: set num_threads then get.
TEST(TypedConfig, Int32RoundTrip)
{
    ovphysx_result_t r = ovphysx_set_global_config_entry(
        ovphysx_config_entry_num_threads(8));
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    int32_t out = 0;
    r = ovphysx_get_global_config_int32(OVPHYSX_CONFIG_NUM_THREADS, &out);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out, 8) << "Int32 value must round-trip correctly";
}

// The ovstage read-buffer pool budget key: the typed entry helper and the getter map to
// /physics/ovstageReadPoolMaxMB. Only the config surface is asserted here. The retention
// behaviour (default, 0/negative disables) belongs to REQ-READ-POOL-001 in the runtime.
// 0 is a valid value (disables the pool), so it round-trips like any other.
TEST(TypedConfig, OvstageReadPoolMaxMbRoundTrip)
{
    int32_t original = 0;
    ASSERT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &original).status,
              OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_ovstage_read_pool_max_mb(64)).status,
              OVPHYSX_API_SUCCESS);
    int32_t out = 0;
    ASSERT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &out).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out, 64) << "pool budget must round-trip through /physics/ovstageReadPoolMaxMB";

    ASSERT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_ovstage_read_pool_max_mb(0)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &out).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out, 0) << "0 disables the pool and is an accepted, round-tripping value";

    ASSERT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_ovstage_read_pool_max_mb(original)).status,
              OVPHYSX_API_SUCCESS);
}

// Raw Carbonite setting override roundtrip.
TEST(TypedConfig, CarboniteDirectOverride)
{
    ovphysx_result_t r = ovphysx_set_global_config_entry(
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/disableContactProcessing"),
            OVPHYSX_LITERAL("true")));
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Verify via the typed getter (the raw Carbonite override writes to
    // the same Carbonite path as the typed bool key).
    bool out = false;
    r = ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, &out);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(out) << "Raw Carbonite override must write to the correct path";
}

// Null out_value for get_global_config_bool must return error.
TEST(TypedConfig, GetBoolNullOut)
{
    ovphysx_result_t r = ovphysx_get_global_config_bool(
        OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, nullptr);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Null out_value must produce an error";
}

// Invalid enum key for get_global_config_bool must return error.
TEST(TypedConfig, GetBoolInvalidKey)
{
    bool out = false;
    ovphysx_result_t r = ovphysx_get_global_config_bool(
        static_cast<ovphysx_config_bool_t>(999), &out);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Invalid enum key must produce an error";
}

// Float: there are no valid keys, so any key must return an error.
TEST(TypedConfig, FloatInvalidKeyReturnsError)
{
    float out = 0.0f;
    ovphysx_result_t r = ovphysx_get_global_config_float(
        static_cast<ovphysx_config_float_t>(0), &out);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "No float keys exist; any key must produce an error";
}

// String: an out-of-range key must return error.
TEST(TypedConfig, StringInvalidKeyReturnsError)
{
    char buf[256] = {};
    ovphysx_string_t value_out = {buf, sizeof(buf)};
    size_t required = 0;
    ovphysx_result_t r = ovphysx_get_global_config_string(
        static_cast<ovphysx_config_string_t>(OVPHYSX_CONFIG_STRING_COUNT), &value_out, &required);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Out-of-range string key must produce an error";
}

TEST(TypedConfig, StringOutputRejectsMissingBuffer)
{
    ovphysx_string_t value = {nullptr, 0};
    size_t requiredSize = 0;
    EXPECT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &value, &requiredSize).status,
        OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(TypedConfig, StringOutputPreservesCapacityForRetry)
{
    destroySharedCpuInstance();
    char priorProbe[1] = {};
    ovphysx_string_t priorValue = { priorProbe, sizeof(priorProbe) };
    size_t priorRequiredSize = 0;
    ovphysx_result_t priorResult = ovphysx_get_global_config_string(
        OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &priorValue, &priorRequiredSize);
    ASSERT_TRUE(
        priorResult.status == OVPHYSX_API_SUCCESS ||
        priorResult.status == OVPHYSX_API_BUFFER_TOO_SMALL);

    std::string prior;
    if (priorResult.status == OVPHYSX_API_SUCCESS)
    {
        prior.assign(priorValue.ptr, priorValue.length);
    }
    else
    {
        std::vector<char> priorBuffer(priorRequiredSize);
        priorValue = { priorBuffer.data(), priorBuffer.size() };
        ASSERT_EQ(
            ovphysx_get_global_config_string(
                OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY,
                &priorValue,
                &priorRequiredSize).status,
            OVPHYSX_API_SUCCESS);
        prior.assign(priorValue.ptr, priorValue.length);
    }
    const ScopedCookedCachePathRestore restore(prior);

    static constexpr char expected[] = "ovphysx-config-contract";
    ASSERT_TRUE(ovphysx_set_raw_string_setting_for_test_internal(
        "/UJITSO/datastore/localCachePath", expected, sizeof(expected) - 1));

    char fullBuffer[sizeof(expected)] = {};
    ovphysx_string_t fullValue = { fullBuffer, sizeof(fullBuffer) };
    size_t requiredSize = 0;
    ASSERT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &fullValue, &requiredSize).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(fullValue.length, sizeof(expected) - 1);
    EXPECT_EQ(requiredSize, sizeof(expected));
    EXPECT_EQ(fullValue.ptr[fullValue.length], '\0');
    EXPECT_EQ(std::string(fullValue.ptr, fullValue.length), expected);

    char smallBuffer[5] = {};
    ovphysx_string_t smallValue = { smallBuffer, sizeof(smallBuffer) };
    requiredSize = 0;
    EXPECT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &smallValue, &requiredSize).status,
        OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(smallValue.length, sizeof(smallBuffer));
    EXPECT_EQ(smallValue.ptr[sizeof(smallBuffer) - 1], '\0');
    EXPECT_EQ(requiredSize, sizeof(expected));

    char oneByteBuffer[1] = { 'x' };
    ovphysx_string_t oneByteValue = { oneByteBuffer, sizeof(oneByteBuffer) };
    EXPECT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &oneByteValue, &requiredSize).status,
        OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(oneByteValue.length, sizeof(oneByteBuffer));
    EXPECT_EQ(oneByteValue.ptr[0], '\0');
    EXPECT_EQ(requiredSize, sizeof(expected));

    std::vector<char> retryBuffer(requiredSize);
    ovphysx_string_t retryValue = { retryBuffer.data(), retryBuffer.size() };
    ASSERT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &retryValue, &requiredSize).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(retryValue.length, sizeof(expected) - 1);
    EXPECT_EQ(retryValue.ptr[retryValue.length], '\0');
    EXPECT_EQ(std::string(retryValue.ptr, retryValue.length), expected);

}

#if SIZE_MAX > UINT32_MAX
TEST(TypedConfig, StringOutputRejectsCapacityAboveUint32)
{
    char storage = '\0';
    ovphysx_string_t value = {&storage, static_cast<size_t>(UINT32_MAX) + 1};
    size_t requiredSize = 0;
    EXPECT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY, &value, &requiredSize).status,
        OVPHYSX_API_INVALID_ARGUMENT);
}
#endif

// Null out_value for get_global_config_int32 must return error.
TEST(TypedConfig, GetInt32NullOut)
{
    ovphysx_result_t r = ovphysx_get_global_config_int32(
        OVPHYSX_CONFIG_NUM_THREADS, nullptr);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Null out_value must produce an error";
}
