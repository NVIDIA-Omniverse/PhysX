// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysx/version.h"
#include "ovphysx_test_utils.h"
#include "ovphysxTestHelpers.h"
#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

// Note: This test is also used to trigger extension pre-caching during build.
TEST_F(PhysXTestFixture, PhysXInstantiation) {
    // Fixture does everything.
    EXPECT_TRUE(true);
}

// ============================================================================
// Version API Tests
// ============================================================================

// ovphysx_get_version fills all three components without crashing.
TEST(VersionAPI, GetVersionFillsAllComponents)
{
    uint32_t major = 0xFFFFFFFF, minor = 0xFFFFFFFF, patch = 0xFFFFFFFF;
    ovphysx_get_version(&major, &minor, &patch);

    // At least one component must have been written (sentinel values cleared)
    EXPECT_TRUE(major != 0xFFFFFFFF || minor != 0xFFFFFFFF || patch != 0xFFFFFFFF)
        << "ovphysx_get_version must update at least one out parameter";

    std::cout << "  ovphysx version: " << major << "." << minor << "." << patch << std::endl;
}

// ovphysx_get_version_string returns a non-null, non-empty string containing a digit.
TEST(VersionAPI, GetVersionStringIsNonEmpty)
{
    const char* version_str = ovphysx_get_version_string();
    ASSERT_NE(version_str, nullptr) << "ovphysx_get_version_string must return non-null";

    const ovphysx_string_t version = ovphysx_cstr(version_str);
    const size_t len = version.length;
    EXPECT_GT(len, 0u) << "Version string must not be empty";

    bool has_digit = false;
    for (size_t i = 0; i < len; ++i)
    {
        if (version_str[i] >= '0' && version_str[i] <= '9')
        {
            has_digit = true;
            break;
        }
    }
    EXPECT_TRUE(has_digit) << "Version string must contain at least one digit";

    std::cout << "  ovphysx version string: " << version_str << std::endl;
}

// Runtime and compile-time versions must agree (prevents stale-header issues).
TEST(VersionAPI, RuntimeVersionMatchesCompileTimeVersion)
{
    uint32_t rt_major = 0, rt_minor = 0, rt_patch = 0;
    ovphysx_get_version(&rt_major, &rt_minor, &rt_patch);

    EXPECT_EQ(rt_major, static_cast<uint32_t>(OVPHYSX_VERSION_MAJOR))
        << "Runtime major version must match compile-time OVPHYSX_VERSION_MAJOR";
    EXPECT_EQ(rt_minor, static_cast<uint32_t>(OVPHYSX_VERSION_MINOR))
        << "Runtime minor version must match compile-time OVPHYSX_VERSION_MINOR";
    EXPECT_EQ(rt_patch, static_cast<uint32_t>(OVPHYSX_VERSION_PATCH))
        << "Runtime patch version must match compile-time OVPHYSX_VERSION_PATCH";
}

TEST(CreateInstanceValidation, RejectsNullConfigEntriesWithNonzeroCount)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entry_count = 1;
    const ovphysx_handle_t sentinel = UINT64_MAX;
    ovphysx_handle_t handle = sentinel;

    const ovphysx_result_t result = ovphysx_create_instance(&args, &handle);

    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(handle, sentinel);
}

// ============================================================================
// active_cuda_gpus Parse Validation
// ============================================================================
// These tests exercise the active_cuda_gpus parser via the public C API.
// Parse errors are returned before runtime initialization, so no GPU is needed.

// Exercises the active_cuda_gpus parser without requiring CUDA. The parser runs
// early (before attach), so this covers parse validation. Invalid inputs fail at
// create time and valid inputs proceed through instance creation.
static ovphysx_api_status_t try_create_with_gpus(const char* gpus)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.active_cuda_gpus = ovphysx_cstr(gpus);
    ovphysx_handle_t handle = 0;
    ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
    if (result.status == OVPHYSX_API_SUCCESS && handle)
        ovphysx_destroy_instance(handle);
    return result.status;
}

TEST(ActiveCudaGpusParsing, InvalidFormat_NonNumeric)
{
    EXPECT_EQ(try_create_with_gpus("abc"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_DuplicateOrdinals)
{
    EXPECT_EQ(try_create_with_gpus("0,0"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_NegativeMixedWithPositive)
{
    EXPECT_EQ(try_create_with_gpus("-1,0"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_EmptyBetweenCommas)
{
    EXPECT_EQ(try_create_with_gpus("0,,1"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_LetterAfterDigit)
{
    EXPECT_EQ(try_create_with_gpus("0a"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_TrailingComma)
{
    EXPECT_EQ(try_create_with_gpus("0,"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_OrdinalOverflow)
{
    EXPECT_EQ(try_create_with_gpus("9999999999"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_NegativeOtherThanMinusOne)
{
    EXPECT_EQ(try_create_with_gpus("-2"), OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(try_create_with_gpus("-5"), OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(try_create_with_gpus("-100"), OVPHYSX_API_INVALID_ARGUMENT);
}

// Valid formats must pass parsing and succeed with CPU device.
TEST(ActiveCudaGpusParsing, ValidFormat_SingleOrdinalZero)
{
    EXPECT_EQ(try_create_with_gpus("0"), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusParsing, ValidFormat_MinusOne_PhysXAutoSelect)
{
    EXPECT_EQ(try_create_with_gpus("-1"), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusParsing, ValidFormat_SingleDigitOrdinal)
{
    EXPECT_EQ(try_create_with_gpus("2"), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusParsing, ValidFormat_WhitespaceAroundOrdinals)
{
    EXPECT_EQ(try_create_with_gpus(" 0 "), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusAttachTest, DirectOvstageAttachPropagatesExplicitOrdinal)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.active_cuda_gpus = OVPHYSX_LITERAL("0");
    const ovphysx_config_entry_t staleMultiGpuMode =
        ovphysx_config_entry_scene_multi_gpu_mode(1);
    args.config_entries = &staleMultiGpuMode;
    args.config_entry_count = 1;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    const ovphysx_result_t createResult = ovphysx_create_instance(&args, &handle);
    if (createResult.status != OVPHYSX_API_SUCCESS)
    {
        ADD_FAILURE() << "Failed to create an instance with active_cuda_gpus=0";
        return;
    }

    int32_t attachCudaSelector = 0;
    if (!ovphysx_get_attach_cuda_selector_for_test_internal(&attachCudaSelector))
    {
        ADD_FAILURE() << "Attach-time CUDA selector is unavailable";
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
        return;
    }
    EXPECT_EQ(attachCudaSelector, -1);

    int32_t multiGpuMode = -1;
    EXPECT_EQ(ovphysx_get_global_config_int32(
                  OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, &multiGpuMode).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(multiGpuMode, 1);

    const bool attached = test_utils::attach_usd_with_ovstage(
        handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda");
    EXPECT_TRUE(attached);
    if (attached)
    {
        EXPECT_TRUE(ovphysx_get_attach_cuda_selector_for_test_internal(&attachCudaSelector));
        EXPECT_EQ(attachCudaSelector, 0);
        EXPECT_EQ(ovphysx_get_global_config_int32(
                      OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, &multiGpuMode).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_EQ(multiGpuMode, 0);
    }

    if (attached)
    {
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
    }
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

// ============================================================================
// Process-wide CPU-only mode query (REQ-CAPI-CPU-001)
// ============================================================================

TEST(CpuModeAPI, GetCpuModeRejectsNull)
{
    EXPECT_EQ(ovphysx_get_cpu_mode(nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(CpuModeAPI, GetCpuModeReportsEffectivePolicy)
{
    bool cpuOnly = false;
    ASSERT_EQ(ovphysx_get_cpu_mode(&cpuOnly).status, OVPHYSX_API_SUCCESS);

    // The CPU CTest pass typically sets OVPHYSX_DISABLE_GPU=1. When that env
    // var is present, hard CPU-only mode must be reported as true. When it is
    // absent, the getter may still be true if ovphysx_set_cpu_mode(true) was
    // applied earlier in the process (sticky), so only the env implication is
    // asserted. Default-false (neither input active) is covered by
    // CpuNoCudaContextGpuTest and the Python lifecycle probe for
    // OVPHYSX_DISABLE_GPU latching.
    if (std::getenv("OVPHYSX_DISABLE_GPU") != nullptr)
    {
        EXPECT_TRUE(cpuOnly)
            << "OVPHYSX_DISABLE_GPU is set; get_cpu_mode must report true";
    }
}

namespace {

struct ScopedLogCapture
{
    ScopedLogCapture() : originalLevel(ovphysx_get_log_level())
    {
        ovphysx_set_log_level(OVPHYSX_LOG_INFO);
    }

    ~ScopedLogCapture()
    {
        ovphysx_log_capture_stop();
        ovphysx_set_log_level(originalLevel);
    }

    uint32_t originalLevel;
};

// Carbonite settings are process-global and outlive the instance that set them.
// A case that turns DirectGPU on must turn it off again. Its only user runs alone
// in its own gtest pass, so this guard protects a case added to that pass later.
struct ScopedDirectGpu
{
    ~ScopedDirectGpu()
    {
        EXPECT_EQ(ovphysx_set_global_config(ovphysx_config_entry_carbonite(
                      OVPHYSX_LITERAL("/physics/suppressReadback"),
                      OVPHYSX_LITERAL("false"))).status,
                  OVPHYSX_API_SUCCESS);
    }
};

} // namespace

// Empty active_cuda_gpus is "no OVPhysX ordinal override" (ADR-0011), while
// "-1" is explicit PhysX automatic selection. The create INFO line must not
// label empty as "auto". Runs in the isolated ActiveCudaGpusAttachTest pass
// (no OVPHYSX_DISABLE_GPU) so the ordinal field is not forced to "inactive".
TEST(ActiveCudaGpusAttachTest, CreateLogEmptyVsMinusOneAcrossRecreate)
{
    ScopedLogCapture logCapture;
    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);

    ovphysx_create_args emptyArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&emptyArgs, &handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "active_cuda_gpus=no_override"))
        << "empty active_cuda_gpus must log no_override, not auto";
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "active_cuda_gpus=auto"));
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    handle = OVPHYSX_INVALID_HANDLE;

    ovphysx_log_capture_stop();
    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);

    ovphysx_create_args autoArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    autoArgs.active_cuda_gpus = OVPHYSX_LITERAL("-1");
    ASSERT_EQ(ovphysx_create_instance(&autoArgs, &handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "active_cuda_gpus=-1"))
        << "explicit -1 must log the ordinal, not no_override/auto";
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    handle = OVPHYSX_INVALID_HANDLE;

    ovphysx_log_capture_stop();
    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_create_instance(&emptyArgs, &handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "active_cuda_gpus=no_override"))
        << "empty recreate after -1 must still log create-args intent as no_override";
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}

// A DirectGPU rigid-body pose read into a host buffer while the simulation runs
// on a non-zero CUDA ordinal. The staging buffer must be allocated in the
// simulation's CUDA context. If the caller's context is on another device, the
// allocation succeeds and the DirectGPU gather faults. If no context is current,
// the allocation itself fails.
// Runs in its own gtest pass (scripts/test_cpp.cmake). PhysX latches its CUDA
// device at the first GPU attach in the process. An earlier attach on ordinal 0
// pins the device, and the later /physics/cudaDevice = 1 write does not move it.
// The case therefore checks first that it is the only test in the process and
// skips when it is not, so a wrong gtest filter cannot make it a silent
// ordinal-0 run.
TEST(ActiveCudaGpusAttachTest, DirectGpuHostReadOnNonZeroOrdinal)
{
    // Process isolation is the case's precondition, not something it can arrange, so
    // it is checked before anything is created: a wrongly grouped run then leaves no
    // instance, no probe and no DirectGPU setting behind. test_to_run_count() reports
    // the post-filter selection for this process, so the dedicated pass sees 1.
    if (::testing::UnitTest::GetInstance()->test_to_run_count() > 1)
    {
        GTEST_SKIP() << "needs its own process: PhysX latches its CUDA device at the first GPU "
                        "attach, so this case only sees ordinal 1 in the dedicated "
                        "cuda-selection-nonzero-ordinal pass (scripts/test_cpp.cmake)";
    }

    // create_instance validates active_cuda_gpus itself and rejects an ordinal the
    // machine does not have, so the device count has to be known before ordinal 1 is
    // requested. The CUDA shim only becomes queryable once an instance has initialized
    // the runtime, so the count is probed through a throwaway default instance: it
    // overrides no ordinal and never attaches, so skipping from here still leaves no
    // ordinal selected process-wide.
    bool haveTwoDevices = false;
    {
        ovphysx_create_args probeArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_handle_t probeHandle = OVPHYSX_INVALID_HANDLE;
        ASSERT_EQ(ovphysx_create_instance(&probeArgs, &probeHandle).status, OVPHYSX_API_SUCCESS);

        omni::physx::IOptionalCuda* probeCuda = ovphysx::test_cuda::getCuda();
        int deviceCount = 0;
        haveTwoDevices = probeCuda && probeCuda->cudaAvailable() &&
                         probeCuda->deviceGetCount(&deviceCount, nullptr) && deviceCount >= 2;

        ASSERT_EQ(ovphysx_destroy_instance(probeHandle).status, OVPHYSX_API_SUCCESS);
    }
    if (!haveTwoDevices)
    {
        GTEST_SKIP() << "needs at least two CUDA devices to select a non-zero ordinal";
    }

    ScopedDirectGpu directGpuSetting;

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.active_cuda_gpus = OVPHYSX_LITERAL("1");
    const ovphysx_config_entry_t directGpu = ovphysx_config_entry_carbonite(
        OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true"));
    args.config_entries = &directGpu;
    args.config_entry_count = 1;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_SUCCESS);

    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(
        handle, OVPHYSX_SOURCE_DIR "/tests/data/boxes_falling_on_groundplane_gpu.usda"));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);

    // The staging path only runs for a GPU-resident binding. A binding on CPU would
    // make this a same-device host read and would exercise none of the fix.
    DLDevice bindingDevice{ kDLExtDev, -1 };
    ASSERT_EQ(ovphysx_get_tensor_binding_native_device(handle, binding, &bindingDevice).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(bindingDevice.device_type, kDLCUDA);
    ASSERT_EQ(bindingDevice.device_id, 1)
        << "binding landed on ordinal " << bindingDevice.device_id
        << "; PhysX latches its CUDA device at the first GPU attach in the process, so this "
           "case must run in its own gtest pass (cuda-selection-nonzero-ordinal in "
           "scripts/test_cpp.cmake), not alongside other ActiveCudaGpusAttachTest cases";

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(spec.shape[0], 0) << "no rigid bodies matched; the read would be a no-op";

    const size_t total = static_cast<size_t>(spec.shape[0]) * static_cast<size_t>(spec.shape[1]);
    std::vector<float> host_poses(total, std::numeric_limits<float>::quiet_NaN());
    int64_t shape[2] = { spec.shape[0], spec.shape[1] };

    DLTensor host_tensor{};
    host_tensor.data = host_poses.data();
    host_tensor.device = { kDLCPU, 0 };
    host_tensor.ndim = 2;
    host_tensor.dtype = { kDLFloat, 32, 1 };
    host_tensor.shape = shape;

    // Whether the read inherits the simulation's context depends on what else the
    // process has done with CUDA. Detaching removes that dependency and makes the
    // case deterministic.
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ovphysx::test_cuda::ScopedCudaContextDetach detach(cuda);

    const ovphysx_result_t readResult = ovphysx_read_tensor_binding(handle, binding, &host_tensor);

    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda))
        << "read_tensor_binding left a CUDA context pushed on the caller's thread";

    EXPECT_TRUE(detach.restore()) << "failed to restore the caller's CUDA context stack";

    EXPECT_EQ(readResult.status, OVPHYSX_API_SUCCESS)
        << "DirectGPU host read on ordinal 1 failed: "
        << std::string(ovphysx_get_last_error().ptr, ovphysx_get_last_error().length);

    if (readResult.status == OVPHYSX_API_SUCCESS)
    {
        for (size_t i = 0; i < total; ++i)
            EXPECT_TRUE(std::isfinite(host_poses[i])) << "pose component " << i << " was not written";
    }

    EXPECT_EQ(ovphysx_destroy_tensor_binding(handle, binding).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}
