// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PARTIALLY DEPRECATED (tensor-binding-deprecation): AC-5 instruments the four tensor-binding entry
// points, so run_workflow's binding read and that binding-entry-point NVTX coverage retire with the
// binding (AC-5 drops them). The NVTX opt-in and side-effect-freedom tests stay. Move
// run_workflow's read to a session read at removal.

/**
 * @implements REQ-CAPI-NVTX-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

// Tests for the NVTX profiling opt-in.
//
// The gate is resolved during ovphysx_create_instance(), so these tests use
// standalone TEST() (not PhysXTestFixture) to control the full instance
// lifecycle, the same way the OmniPVD recording tests do.
//
// Whether an NVTX consumer records the ranges cannot be observed from inside the
// process, so what is asserted here is the opt-in and its side-effect freedom.
// The resulting Nsight Systems timeline is verified manually.
//
// AC-5 coverage is partial by design: run_workflow exercises ovphysx_step,
// ovphysx_wait_op, ovphysx_attach_ovstage, ovphysx_create_tensor_binding and
// ovphysx_read_tensor_binding. The remaining entry points AC-5 lists are verified
// by capture rather than here (TEST-CAPI-NVTX-001).

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

using namespace test_utils;

namespace
{

const char* const kEnvVar = "OVPHYSX_NVTX";

void set_env(const char* value)
{
#if defined(_WIN32)
    _putenv_s(kEnvVar, value);
#else
    setenv(kEnvVar, value, 1);
#endif
}

void unset_env()
{
#if defined(_WIN32)
    _putenv_s(kEnvVar, "");
#else
    unsetenv(kEnvVar);
#endif
}

bool nvtx_setting()
{
    bool value = true;
    ovphysx_result_t r = ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, &value);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << "reading /physics/nvtxEnabled must succeed";
    return value;
}

// Leaves the process in the shipped default state: the setting is global, so a
// test that turned NVTX on has to turn it back off for the tests that follow.
class NvtxProfiling : public ::testing::Test
{
protected:
    void SetUp() override
    {
        unset_env();
        reset_setting();
    }

    void TearDown() override
    {
        unset_env();
        reset_setting();
    }

    // Best effort by design. The Carbonite settings interface only exists once an
    // instance has been created somewhere in this process, and before that point
    // nothing can have enabled NVTX, so there is nothing to clear. Asserting here
    // would make these tests pass only when some earlier test in the same process
    // happened to create an instance first.
    static void reset_setting()
    {
        ovphysx_set_global_config(ovphysx_config_entry_nvtx_enabled(false));
    }
};

// Creates an instance, attaches a falling-boxes stage, steps it a fixed number
// of times and reads the resulting body poses back through a tensor binding.
// This walks every entry point the requirement lists as instrumented.
::testing::AssertionResult run_workflow(const ovphysx_config_entry_t* entries,
                                       uint32_t entry_count,
                                       std::vector<float>& out_poses)
{
    out_poses.clear();

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries;
    args.config_entry_count = entry_count;

    ovphysx_handle_t handle = 0;
    ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
    if (r.status != OVPHYSX_API_SUCCESS || handle == 0)
        return ::testing::AssertionFailure() << "Failed to create PhysX instance";

    if (!attach_usd_with_ovstage(handle, "tests/data/boxes_falling_on_groundplane.usda"))
    {
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Failed to attach the test stage";
    }

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_handle_t binding = 0;
    r = ovphysx_create_tensor_binding(handle, &desc, &binding);
    if (r.status != OVPHYSX_API_SUCCESS)
    {
        destroy_ovstage_test_attachments(handle);
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Failed to create the pose binding";
    }

    ovphysx_tensor_spec_t spec{};
    r = ovphysx_get_tensor_binding_spec(handle, binding, &spec);
    if (r.status != OVPHYSX_API_SUCCESS || spec.ndim != 2 || spec.shape[0] <= 0)
    {
        destroy_ovstage_test_attachments(handle);
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Unexpected pose binding spec";
    }

    // Async step plus explicit wait, so ovphysx_step and ovphysx_wait_op are both
    // exercised rather than only the synchronous convenience wrapper.
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 10; ++i)
    {
        ovphysx_enqueue_result_t enqueued = ovphysx_step(handle, dt);
        if (enqueued.status != OVPHYSX_API_SUCCESS)
        {
            destroy_ovstage_test_attachments(handle);
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Step " << i << " failed to enqueue";
        }
        if (enqueued.op_index != 0 && !waitForOperationSuccess(handle, enqueued.op_index, 10'000'000'000ULL))
        {
            destroy_ovstage_test_attachments(handle);
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Step " << i << " failed to complete";
        }
    }

    const size_t element_count = static_cast<size_t>(spec.shape[0]) * static_cast<size_t>(spec.shape[1]);
    std::vector<float> poses(element_count, 0.0f);
    DLTensor* dst = make_float32_tensor(poses, { spec.shape[0], spec.shape[1] });
    r = ovphysx_read_tensor_binding(handle, binding, dst);
    if (r.status == OVPHYSX_API_SUCCESS)
    {
        const float* data = static_cast<const float*>(dst->data);
        out_poses.assign(data, data + element_count);
    }
    free_tensor(dst);

    ovphysx_destroy_tensor_binding(handle, binding);
    destroy_ovstage_test_attachments(handle);
    ovphysx_destroy_instance(handle);

    if (out_poses.empty())
        return ::testing::AssertionFailure() << "Failed to read body poses back";

    return ::testing::AssertionSuccess();
}

} // namespace

// AC-1: no environment variable and no config entry means no NVTX.
TEST_F(NvtxProfiling, DisabledByDefault)
{
    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_FALSE(nvtx_setting())
        << "instance creation must not enable NVTX when nothing asked for it";
}

// AC-2: the typed config entry enables it and is observable through the getter.
TEST_F(NvtxProfiling, ConfigEntryEnables)
{
    const ovphysx_config_entry_t entries[] = { ovphysx_config_entry_nvtx_enabled(true) };

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(entries, 1, poses));
    EXPECT_TRUE(nvtx_setting()) << "the config entry must enable NVTX";
}

// AC-3: the environment variable enables it without any config entry, and is
// written through to the setting so the omni.physx runtime sees the same answer.
TEST_F(NvtxProfiling, EnvironmentVariableEnables)
{
    // The disabled starting point comes from the fixture. It is not re-checked here
    // because reading the setting requires an instance to exist, and this test has
    // to work as the first one in a process. DisabledByDefault covers the default.
    set_env("1");

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_TRUE(nvtx_setting()) << "OVPHYSX_NVTX must be written through to /physics/nvtxEnabled";
}

// AC-3: a value that reads as false does not enable it.
TEST_F(NvtxProfiling, EnvironmentVariableFalseDoesNotEnable)
{
    set_env("0");

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_FALSE(nvtx_setting()) << "OVPHYSX_NVTX=0 must leave NVTX disabled";
}

// AC-1 stickiness and AC-3 symmetry. /physics/nvtxEnabled outlives the instance
// that set it, so an explicitly false environment variable has to be able to turn
// it back off. Writing only the true case would strand the process as enabled.
TEST_F(NvtxProfiling, EnvironmentVariableFalseTurnsOffStickySetting)
{
    const ovphysx_config_entry_t entries[] = { ovphysx_config_entry_nvtx_enabled(true) };
    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(entries, 1, poses));
    ASSERT_TRUE(nvtx_setting()) << "the config entry must leave the sticky setting enabled";

    // Second instance: no config entry, only the falsey environment variable.
    set_env("0");
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_FALSE(nvtx_setting()) << "OVPHYSX_NVTX=0 must turn a sticky enabled setting back off";
}

// AC-4 and AC-5: the ranges are side-effect free. Every instrumented entry point
// runs in both configurations (run_workflow walks them) and the simulation
// results have to match.
TEST_F(NvtxProfiling, EnabledRunMatchesDisabledRun)
{
    std::vector<float> disabled_poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, disabled_poses));

    const ovphysx_config_entry_t entries[] = { ovphysx_config_entry_nvtx_enabled(true) };
    std::vector<float> enabled_poses;
    ASSERT_TRUE(run_workflow(entries, 1, enabled_poses));

    ASSERT_EQ(disabled_poses.size(), enabled_poses.size())
        << "the same stage must produce the same number of pose components";
    for (size_t i = 0; i < disabled_poses.size(); ++i)
    {
        EXPECT_NEAR(disabled_poses[i], enabled_poses[i], 1e-5f)
            << "pose component " << i << " differs with NVTX enabled";
    }
}
