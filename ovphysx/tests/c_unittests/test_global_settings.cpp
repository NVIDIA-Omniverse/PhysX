// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Tests for:
//   ovphysx_set_global_config / ovphysx_get_global_config_*  (typed config API)

/**
 * @implements REQ-CAPI-STRING-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-2
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <cstring>
#include <string>
#include <iostream>
#include <vector>

using namespace test_utils;

namespace
{
class ScopedRecordingDirectoryRestore
{
public:
    explicit ScopedRecordingDirectoryRestore(const std::string& value)
        : m_value(value)
    {
    }

    ~ScopedRecordingDirectoryRestore()
    {
        const ovphysx_string_t value = { m_value.data(), m_value.size() };
        (void)ovphysx_set_global_config(
            ovphysx_config_entry_omnipvd_ovd_recording_directory(value));
    }

private:
    std::string m_value;
};
}

static bool wait_gs_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index)
{
    ovphysx_op_wait_result_t wr{};
    ovphysx_result_t r = ovphysx_wait_op(handle, op_index, 10'000'000'000ULL, &wr);
    ovphysx_destroy_wait_result(&wr);
    return r.status == OVPHYSX_API_SUCCESS;
}

// ============================================================================
// Typed Config API Tests
// ============================================================================

// set_global_config with a typed bool entry must succeed.
TEST(TypedConfig, SetBoolSucceeds)
{
    ovphysx_result_t r = ovphysx_set_global_config(
        ovphysx_config_entry_disable_contact_processing(false));
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS)
        << "set_global_config(bool) must succeed for a known-writable key";
}

// Bool roundtrip: set then get.
TEST(TypedConfig, BoolRoundTrip)
{
    ovphysx_result_t r = ovphysx_set_global_config(
        ovphysx_config_entry_disable_contact_processing(true));
    if (r.status != OVPHYSX_API_SUCCESS)
        GTEST_SKIP() << "Setting not writable; skipping round-trip test";

    bool out = false;
    r = ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, &out);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(out) << "Bool value must round-trip correctly";

    // Toggle back
    r = ovphysx_set_global_config(
        ovphysx_config_entry_disable_contact_processing(false));
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_get_global_config_bool(OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING, &out);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(out) << "Bool value must round-trip correctly after toggle";
}

// Int32 roundtrip: set num_threads then get.
TEST(TypedConfig, Int32RoundTrip)
{
    ovphysx_result_t r = ovphysx_set_global_config(
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

    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_ovstage_read_pool_max_mb(64)).status,
              OVPHYSX_API_SUCCESS);
    int32_t out = 0;
    ASSERT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &out).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out, 64) << "pool budget must round-trip through /physics/ovstageReadPoolMaxMB";

    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_ovstage_read_pool_max_mb(0)).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, &out).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(out, 0) << "0 disables the pool and is an accepted, round-tripping value";

    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_ovstage_read_pool_max_mb(original)).status,
              OVPHYSX_API_SUCCESS);
}

TEST(TypedConfig, CreateOnlyKeysCannotMutateLiveRuntime)
{
    destroySharedCpuInstance();
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_output_enabled(false)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL(""))).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(
        ovphysx_set_global_config(ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_FILE_NAME)).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL(""))).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_port(1234)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_timeout_ms(0)).status, OVPHYSX_API_SUCCESS);

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    EXPECT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_port(4321)).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(
        ovphysx_set_global_config(ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/omniPvdTransport"), OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME)).status,
        OVPHYSX_API_ERROR);

    int32_t port = 0;
    EXPECT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OMNIPVD_TCP_PORT, &port).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(port, 1234);
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_port(0)).status, OVPHYSX_API_SUCCESS);
}

// Raw Carbonite setting override roundtrip.
TEST(TypedConfig, CarboniteDirectOverride)
{
    ovphysx_result_t r = ovphysx_set_global_config(
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
            OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &value, &requiredSize).status,
        OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(TypedConfig, StringOutputPreservesCapacityForRetry)
{
    destroySharedCpuInstance();
    char priorProbe[1] = {};
    ovphysx_string_t priorValue = { priorProbe, sizeof(priorProbe) };
    size_t priorRequiredSize = 0;
    ovphysx_result_t priorResult = ovphysx_get_global_config_string(
        OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &priorValue, &priorRequiredSize);
    if (priorResult.status == OVPHYSX_API_NOT_FOUND)
        GTEST_SKIP() << "Recording-directory setting is absent; preserving that state without mutation";
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
                OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY,
                &priorValue,
                &priorRequiredSize).status,
            OVPHYSX_API_SUCCESS);
        prior.assign(priorValue.ptr, priorValue.length);
    }
    const ScopedRecordingDirectoryRestore restore(prior);

    static constexpr char expected[] = "ovphysx-config-contract";
    ASSERT_EQ(
        ovphysx_set_global_config(
            ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL(expected))).status,
        OVPHYSX_API_SUCCESS);

    char fullBuffer[sizeof(expected)] = {};
    ovphysx_string_t fullValue = { fullBuffer, sizeof(fullBuffer) };
    size_t requiredSize = 0;
    ASSERT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &fullValue, &requiredSize).status,
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
            OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &smallValue, &requiredSize).status,
        OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(smallValue.length, sizeof(smallBuffer));
    EXPECT_EQ(smallValue.ptr[sizeof(smallBuffer) - 1], '\0');
    EXPECT_EQ(requiredSize, sizeof(expected));

    char oneByteBuffer[1] = { 'x' };
    ovphysx_string_t oneByteValue = { oneByteBuffer, sizeof(oneByteBuffer) };
    EXPECT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &oneByteValue, &requiredSize).status,
        OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(oneByteValue.length, sizeof(oneByteBuffer));
    EXPECT_EQ(oneByteValue.ptr[0], '\0');
    EXPECT_EQ(requiredSize, sizeof(expected));

    std::vector<char> retryBuffer(requiredSize);
    ovphysx_string_t retryValue = { retryBuffer.data(), retryBuffer.size() };
    ASSERT_EQ(
        ovphysx_get_global_config_string(
            OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &retryValue, &requiredSize).status,
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
            OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY, &value, &requiredSize).status,
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
