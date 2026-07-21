// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Tests for:
//   ovphysx_set_global_config / ovphysx_get_global_config_*  (typed config API)

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <cstring>
#include <string>
#include <iostream>

using namespace test_utils;


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

// Float: no valid keys yet — any key must return error.
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

// Null out_value for get_global_config_int32 must return error.
TEST(TypedConfig, GetInt32NullOut)
{
    ovphysx_result_t r = ovphysx_get_global_config_int32(
        OVPHYSX_CONFIG_NUM_THREADS, nullptr);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Null out_value must produce an error";
}
