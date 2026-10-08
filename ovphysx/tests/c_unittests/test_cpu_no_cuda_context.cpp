// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PARTIALLY DEPRECATED (tensor-binding-deprecation): only the ovphysx_read_tensor_binding
// CUDA-tensor rejection arm is binding-specific (bindings take a user tensor, sessions do not)
// and retires with the binding. The CPU-mode / no-CUDA-context contract (set/get_cpu_mode,
// create_instance, the process_cpu_only log line) is the subject of this test and stays.

// CpuNoCudaContextGpuTest: on a GPU-capable box, calling ovphysx_set_cpu_mode(true)
// before creating an instance and stepping must not open a CUDA context. Device
// mode is process-wide, not a per-instance create-arg, so once it is set no
// instance may touch the CUDA driver regardless of which USD stages are loaded.
//
// Measurement: the CUDA driver primary-context active flag for device 0, read via
// dlopen so the test needs no build-time CUDA dependency and skips on CPU-only
// runners. PxCreateCudaContextManager retains the primary context, so its
// creation flips this flag 0 -> 1.
//
// Routing: runs in its own isolated gtest pass (scripts/test_cpp.cmake:
// CpuNoCudaContextGpuTest.*) so no earlier GPU test has already activated the
// context. Self-skips if the context is already active.

#include <ovphysx/ovphysx.h>
#include <gtest/gtest.h>

#include "ovphysx_test_utils.h"
#include "test_utilities.h"

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif
#include <cstring>
#include <cstdlib>

namespace {

// CpuNoCudaContextGpuTest suffix keeps this class out of the CPU pass
// (filter: -*GpuTest*) and the main GPU pass explicitly excludes it.
TEST(CpuNoCudaContextGpuTest, CpuOnlyFlagCreatesNoCudaContextOnGpuBox)
{
#if defined(_WIN32)
    HMODULE cudalib = LoadLibraryA("nvcuda.dll");
#else
    void* cudalib = dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!cudalib)
        cudalib = dlopen("libcuda.so", RTLD_NOW | RTLD_LOCAL);
#endif
    if (!cudalib)
        GTEST_SKIP() << "CUDA driver not present -- not a GPU box (contract is GPU-box-only)";

    auto loadSym = [&](const char* name) -> void* {
#if defined(_WIN32)
        return reinterpret_cast<void*>(GetProcAddress(cudalib, name));
#else
        return dlsym(cudalib, name);
#endif
    };

    using cuInit_t                      = int (*)(unsigned int);
    using cuDeviceGetCount_t            = int (*)(int*);
    using cuDeviceGet_t                 = int (*)(int*, int);
    using cuDevicePrimaryCtxGetState_t  = int (*)(int, unsigned int*, int*);

    auto cuInit                     = reinterpret_cast<cuInit_t>(loadSym("cuInit"));
    auto cuDeviceGetCount           = reinterpret_cast<cuDeviceGetCount_t>(loadSym("cuDeviceGetCount"));
    auto cuDeviceGet                = reinterpret_cast<cuDeviceGet_t>(loadSym("cuDeviceGet"));
    auto cuDevicePrimaryCtxGetState = reinterpret_cast<cuDevicePrimaryCtxGetState_t>(
                                          loadSym("cuDevicePrimaryCtxGetState"));

    ASSERT_TRUE(cuInit && cuDeviceGetCount && cuDeviceGet && cuDevicePrimaryCtxGetState)
        << "CUDA driver present but expected symbols missing";

    if (cuInit(0) != 0)
        GTEST_SKIP() << "cuInit failed -- no usable CUDA driver";
    int devCount = 0;
    if (cuDeviceGetCount(&devCount) != 0 || devCount == 0)
        GTEST_SKIP() << "no CUDA devices -- not a GPU box";

    int dev = 0;
    ASSERT_EQ(cuDeviceGet(&dev, 0), 0) << "cuDeviceGet(0) failed";

    auto primaryActive = [&]() -> int {
        unsigned int flags = 0; int active = -1;
        if (cuDevicePrimaryCtxGetState(dev, &flags, &active) != 0) return -1;
        return active;
    };

    // Fresh process: context must not be active yet.
    const int activeBefore = primaryActive();
    if (activeBefore != 0)
        GTEST_SKIP() << "primary CUDA context already active (=" << activeBefore
                     << ") before the test; run CpuNoCudaContextGpuTest.* isolated";

    // Fresh process (this pass does not set OVPHYSX_DISABLE_GPU): hard CPU-only
    // must report false before set_cpu_mode (REQ-CAPI-CPU-001 AC-1 inactive case).
    // test_main already called ovphysx_initialize(), which latched the unset env.
    {
        bool cpuOnlyBefore = true;
        ASSERT_EQ(ovphysx_get_cpu_mode(&cpuOnlyBefore).status, OVPHYSX_API_SUCCESS);
        if (std::getenv("OVPHYSX_DISABLE_GPU") == nullptr)
        {
            EXPECT_FALSE(cpuOnlyBefore)
                << "get_cpu_mode must report false when neither set_cpu_mode nor "
                   "OVPHYSX_DISABLE_GPU is active";
        }
    }

    // Set process-wide CPU-only mode before creating any instance.
    ASSERT_EQ(ovphysx_set_cpu_mode(true).status, OVPHYSX_API_SUCCESS)
        << "ovphysx_set_cpu_mode(true) failed";

    bool cpuOnly = false;
    ASSERT_EQ(ovphysx_get_cpu_mode(&cpuOnly).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(cpuOnly)
        << "ovphysx_get_cpu_mode must report true after set_cpu_mode(true)";

    // Verify that the explicit API activates the same TensorBinding policy as
    // OVPHYSX_DISABLE_GPU before any instance or CUDA interface is needed.
    float dummy = 0.0f;
    int64_t tensorShape[1] = {1};
    DLTensor fakeCudaTensor{};
    fakeCudaTensor.data = &dummy;
    fakeCudaTensor.ndim = 1;
    fakeCudaTensor.dtype = {kDLFloat, 32, 1};
    fakeCudaTensor.shape = tensorShape;
    const DLDeviceType cudaDeviceTypes[] = {kDLCUDA, kDLCUDAManaged};
    for (DLDeviceType deviceType : cudaDeviceTypes)
    {
        fakeCudaTensor.device = {deviceType, 0};
        EXPECT_EQ(ovphysx_read_tensor_binding(OVPHYSX_INVALID_HANDLE, 0, &fakeCudaTensor).status,
                  OVPHYSX_API_DEVICE_MISMATCH);
    }
    ASSERT_EQ(primaryActive(), 0)
        << "CPU-only TensorBinding rejection activated the primary CUDA context";

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t h = 0;
    ASSERT_EQ(ovphysx_create_instance(&args, &h).status, OVPHYSX_API_SUCCESS)
        << "create_instance failed";
    ASSERT_EQ(primaryActive(), 0)
        << "create_instance under CPU-only mode activated the primary CUDA context";

    // Log capture needs Carbonite (initialized by the first create). Recreate once
    // under INFO capture to assert the create-time process_cpu_only line (AC-3).
    ASSERT_EQ(ovphysx_destroy_instance(h).status, OVPHYSX_API_SUCCESS);
    h = 0;
    {
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
        ScopedLogCapture logCapture;
        ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_create_instance(&args, &h).status, OVPHYSX_API_SUCCESS)
            << "recreate_instance failed";
        EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "process_cpu_only=true"))
            << "create INFO line must name hard CPU-only policy";
    }
    ASSERT_EQ(primaryActive(), 0)
        << "recreate under CPU-only mode activated the primary CUDA context";

    // Load a GPU USD stage. Process-wide CPU-only mode was set before create, so
    // the stage content is irrelevant and no CUDA context may be activated.
    const char* assetPath = "tests/data/boxes_falling_on_groundplane_gpu.usda";
    ASSERT_TRUE(test_utils::register_physx_schemas_with_ovstage());
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-cpu-only-test-stage";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    if (primaryActive() != 0)
    {
        ovstage_destroy_instance(stage);
        ovphysx_destroy_instance(h);
        GTEST_SKIP() << "ovstage_create_instance activates the CUDA primary context before "
                        "ovphysx receives the stage, so this release cannot isolate the "
                        "ovphysx CPU-only no-driver-touch contract";
    }

    const ovx_string_t path{ assetPath, std::strlen(assetPath) };
    const ovstage_population_enqueue_result_t enqueue = ovstage_population_open_usd_from_file(
        stage, path, 1, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
    ASSERT_EQ(enqueue.status, OVSTAGE_OK);

    ovstage_population_op_wait_result_t populationWait{};
    ASSERT_EQ(ovstage_population_wait_op(
                  stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &populationWait),
              OVSTAGE_OK);

    ovstage_write_floor_desc_t floorDesc{};
    floorDesc.ordinal = 1;
    floorDesc.scope = OVSTAGE_SCOPE_ALL;
    const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floorDesc);
    ASSERT_EQ(floor.status, OVSTAGE_OK);
    ovstage_op_wait_result_t floorWait{};
    ASSERT_EQ(
        ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &floorWait),
        OVSTAGE_OK);
    ASSERT_EQ(floorWait.error_op_id_count, 0u);
    ASSERT_EQ(ovstage_release_op(stage, floor.op_index), OVSTAGE_OK);

    ASSERT_EQ(ovphysx_attach_ovstage(h, stage, 1).status, OVPHYSX_API_SUCCESS);

    for (int s = 0; s < 3; ++s)
    {
        ovphysx_enqueue_result_t st = ovphysx_step(h, 1.f / 60.f);
        ASSERT_EQ(st.status, OVPHYSX_API_SUCCESS);
        ovphysx_op_wait_result_t sw{};
        ovphysx_wait_op(h, st.op_index, 60'000'000'000ULL, &sw);
        ovphysx_destroy_wait_result(&sw);
    }

    const int activeAfter = primaryActive();

    (void)ovphysx_detach_ovstage(h);
    ovstage_destroy_instance(stage);
    ovphysx_destroy_instance(h);

    EXPECT_EQ(activeAfter, 0)
        << "An instance created under ovphysx_set_cpu_mode(true) activated the CUDA primary "
           "context -- the no-driver-touch contract is broken. CPU-only mode must never touch CUDA.";
}

} // anonymous namespace
