// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ovphysx_evaluate_sdf_shapes at the C boundary (ADR-0048): malformed tensors, more than 65535 shapes and an unowned
// query are refused before a task is created; a CPU instance, and a query kept across detach_ovstage, fail the task naming
// the cause; and on a DirectGPU instance sdf_cube.usda's cube reads negative just inside its +x face and positive just
// outside, with the gradient pointing out, as the SDF view's value test reads it.

/**
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1 AC-2
 * @maps_to TEST-CAPI-SDF-001
 */

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>
#include <ovphysx/ovphysx_geometry_query.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>
#include <cudamanager/PxCudaContextManager.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <string>

using namespace test_utils;

namespace
{
// Nothing outstanding on the points: the handoff of a caller that finished writing them.
constexpr ovstage_cuda_sync_t kNoHandoff{};

// Counts the auto-warmup warnings logged while it lives -- each one a simulation step the evaluation took -- through
// the process log callback, and restores the default logging after.
struct WarmupWarnings
{
    std::atomic<int> count{ 0 };

    WarmupWarnings()
    {
        EXPECT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_WARNING, nullptr, &WarmupWarnings::onLog, this).status,
                  OVPHYSX_API_SUCCESS);
    }
    ~WarmupWarnings()
    {
        ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE);
        ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    }
    WarmupWarnings(const WarmupWarnings&) = delete;
    WarmupWarnings& operator=(const WarmupWarnings&) = delete;

    int seen()
    {
        EXPECT_EQ(ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE).status, OVPHYSX_API_SUCCESS);
        return count.load();
    }

    static void onLog(ovphysx_log_level_t, ovphysx_string_t message, ovphysx_string_t, double, void* data)
    {
        if (message.ptr && std::string(message.ptr, message.length).find("Auto-warmup") != std::string::npos)
            ++static_cast<WarmupWarnings*>(data)->count;
    }
};

// A synchronous refusal of `status`, with no task issued, so there is nothing to wait on.
::testing::AssertionResult refusedBeforeATask(const ovphysx_enqueue_result_t& receipt,
                                              ovphysx_api_status_t status = OVPHYSX_API_INVALID_ARGUMENT)
{
    if (receipt.status == status && receipt.op_index == 0u)
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << "status " << receipt.status << ", op_index " << receipt.op_index;
}

// A tensor descriptor over `data`, never dereferenced by the checks it is handed to: rank 3, `shape`.
DLTensor tensorOf(void* data, int64_t* shape, uint8_t bits = 32, DLDeviceType device = kDLCUDA, int32_t ordinal = 0)
{
    DLTensor t{};
    t.data = data;
    t.device = { device, ordinal };
    t.ndim = 3;
    t.dtype = { kDLFloat, bits, 1 };
    t.shape = shape;
    return t;
}

struct LiveSdfQuery
{
    ovphysx_instance_handle_t handle = 0;
    ovphysx_query_handle_t query = 0;
    uint64_t rows = 0; // total_prim_count of the creation summary

    ~LiveSdfQuery()
    {
        if (handle != 0 && query != 0)
            complete_task(handle, ovphysx_release_query(handle, query));
    }
};

// An SDF_SHAPE query over `pattern`, opened and waited, with its row count from the creation summary.
bool openSdfQuery(ovphysx_instance_handle_t handle, const char* pattern, LiveSdfQuery& live)
{
    live.handle = handle;
    ovphysx_query_selection_t selection{};
    selection.pattern = { pattern, std::char_traits<char>::length(pattern) };
    ovstage_query_result_t summary{};
    const ovphysx_enqueue_result_t opened =
        ovphysx_query_selection(handle, OVPHYSX_QUERY_TARGET_SDF_SHAPE, &selection, &live.query, &summary);
    if (opened.status != OVPHYSX_API_SUCCESS || !waitForTaskSuccess(handle, opened) || live.query == 0)
        return false;
    live.rows = summary.total_prim_count;
    return true;
}
} // namespace

class SdfShapesTest : public PhysXTestFixture
{
};

// Every malformed tensor is a synchronous INVALID_ARGUMENT, before the query is even looked at.
TEST_F(SdfShapesTest, MalformedTensorsAreRefusedBeforeATask)
{
    float storage[64] = {};
    int64_t points3[3] = { 1, 2, 3 };
    int64_t out4[3] = { 1, 2, 4 };
    const DLTensor points = tensorOf(storage, points3);
    DLTensor out = tensorOf(storage, out4);
    const ovphysx_query_handle_t noQuery = 0;

    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, nullptr, &out, kNoHandoff)));
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &points, nullptr, kNoHandoff)));
    const DLTensor noData = tensorOf(nullptr, points3);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &noData, &out, kNoHandoff)));
    int64_t wrongLanes[3] = { 1, 2, 4 };
    const DLTensor fourLanePoints = tensorOf(storage, wrongLanes);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &fourLanePoints, &out, kNoHandoff)));
    DLTensor threeLaneOut = tensorOf(storage, points3);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &points, &threeLaneOut, kNoHandoff)));
    const DLTensor doublePoints = tensorOf(storage, points3, 64);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &doublePoints, &out, kNoHandoff)));
    const DLTensor hostPoints = tensorOf(storage, points3, 32, kDLCPU);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &hostPoints, &out, kNoHandoff)));
    int64_t otherN[3] = { 2, 2, 4 };
    DLTensor otherNOut = tensorOf(storage, otherN);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &points, &otherNOut, kNoHandoff)));
    int64_t otherQ[3] = { 1, 3, 4 };
    DLTensor otherQOut = tensorOf(storage, otherQ);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &points, &otherQOut, kNoHandoff)));
    int64_t noPoints3[3] = { 1, 0, 3 };
    int64_t noPoints4[3] = { 1, 0, 4 };
    const DLTensor zeroQPoints = tensorOf(storage, noPoints3);
    DLTensor zeroQOut = tensorOf(storage, noPoints4);
    EXPECT_TRUE(
        refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &zeroQPoints, &zeroQOut, kNoHandoff)));
    DLTensor otherDeviceOut = tensorOf(storage, out4, 32, kDLCUDA, 1);
    EXPECT_TRUE(
        refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &points, &otherDeviceOut, kNoHandoff)));
    // N x Q past what PhysX's 32-bit point indexing reaches.
    int64_t tooManyPoints3[3] = { 2, int64_t(1) << 30, 3 };
    int64_t tooManyPoints4[3] = { 2, int64_t(1) << 30, 4 };
    const DLTensor manyPoints = tensorOf(storage, tooManyPoints3);
    DLTensor manyOut = tensorOf(storage, tooManyPoints4);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &manyPoints, &manyOut, kNoHandoff)));
    // One evaluation addresses at most 65535 shapes, a PhysX launch limit.
    int64_t tooMany3[3] = { 65536, 1, 3 };
    int64_t tooMany4[3] = { 65536, 1, 4 };
    const DLTensor tooManyPoints = tensorOf(storage, tooMany3);
    DLTensor tooManyOut = tensorOf(storage, tooMany4);
    EXPECT_TRUE(
        refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, noQuery, &tooManyPoints, &tooManyOut, kNoHandoff)));
}

// Well-formed tensors with a query this instance does not own: NOT_FOUND, before a task.
TEST_F(SdfShapesTest, UnownedQueryIsRefusedBeforeATask)
{
    float storage[16] = {};
    int64_t points3[3] = { 1, 2, 3 };
    int64_t out4[3] = { 1, 2, 4 };
    const DLTensor points = tensorOf(storage, points3);
    DLTensor out = tensorOf(storage, out4);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_evaluate_sdf_shapes(m_handle, 0x5DF, &points, &out, kNoHandoff),
                                   OVPHYSX_API_NOT_FOUND));
}

// On a CPU instance the cube is still an SDF_SHAPE row, and the evaluation task fails naming the DirectGPU
// requirement, as the SDF view refused a CPU instance -- before the warmup, so the refused call steps nothing.
TEST_F(SdfShapesTest, CpuInstanceFailsTheTaskNamingDirectGpu)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/sdf_cube.usda")) << "Failed to attach ovstage USD";
    LiveSdfQuery live;
    ASSERT_TRUE(openSdfQuery(m_handle, "/World/Cube", live));
    EXPECT_EQ(live.rows, 1u);

    float storage[16] = {};
    int64_t points3[3] = { 1, 2, 3 };
    int64_t out4[3] = { 1, 2, 4 };
    const DLTensor points = tensorOf(storage, points3);
    DLTensor out = tensorOf(storage, out4);
    WarmupWarnings warmups;
    const ovphysx_result_t failed =
        complete_task(m_handle, ovphysx_evaluate_sdf_shapes(m_handle, live.query, &points, &out, kNoHandoff));
    EXPECT_EQ(failed.status, OVPHYSX_API_ERROR);
    EXPECT_NE(last_task_error().find("requires a DirectGPU scene"), std::string::npos) << last_task_error();
    EXPECT_EQ(warmups.seen(), 0) << "a refused evaluation stepped the simulation";
}

// A query kept across detach_ovstage is stale: after the stage is attached again, its evaluation task fails naming
// the cause, without warming the new stage up, and the query can still be released.
TEST_F(SdfShapesTest, StaleQueryFailsTheTaskNamingTheCause)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/sdf_cube.usda")) << "Failed to attach ovstage USD";
    LiveSdfQuery live;
    ASSERT_TRUE(openSdfQuery(m_handle, "/World/Cube", live));
    const ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, reset));
    }
    ASSERT_TRUE(destroy_ovstage_test_attachments(m_handle));
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/sdf_cube.usda")) << "Failed to attach ovstage USD";

    float storage[16] = {};
    int64_t points3[3] = { 1, 2, 3 };
    int64_t out4[3] = { 1, 2, 4 };
    const DLTensor points = tensorOf(storage, points3);
    DLTensor out = tensorOf(storage, out4);
    WarmupWarnings warmups;
    const ovphysx_result_t failed =
        complete_task(m_handle, ovphysx_evaluate_sdf_shapes(m_handle, live.query, &points, &out, kNoHandoff));
    EXPECT_EQ(failed.status, OVPHYSX_API_ERROR);
    EXPECT_NE(last_task_error().find("no longer attached"), std::string::npos) << last_task_error();
    EXPECT_EQ(warmups.seen(), 0) << "a refused evaluation stepped the new stage";
}

#if OVPHYSX_ENABLE_GPU_TESTS
class SdfShapesGpuTest : public ::testing::Test
{
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t config[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true")),
        };
        args.config_entries = config;
        args.config_entry_count = 1;
        const ovphysx_result_t created = ovphysx_create_instance_with_args(&args, &s_handle);
        if (created.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            s_skipReason.assign(error.ptr ? error.ptr : "", error.length);
            s_handle = 0;
        }
        else if (!ovphysx::test_cuda::cudaAvailable())
        {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            s_skipReason = "CUDA not available";
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle != 0)
            ovphysx_destroy_instance(s_handle);
        s_handle = 0;
    }

    void SetUp() override
    {
        if (s_handle == 0)
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "GPU required: " << s_skipReason;
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        if (m_handle != 0)
        {
            const ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
            EXPECT_EQ(reset.status, OVPHYSX_API_SUCCESS);
            if (reset.op_index != 0)
            {
                EXPECT_TRUE(waitForTaskSuccess(m_handle, reset));
            }
            EXPECT_TRUE(destroy_ovstage_test_attachments(m_handle));
        }
    }
};

ovphysx_instance_handle_t SdfShapesGpuTest::s_handle = 0;
std::string SdfShapesGpuTest::s_skipReason;

// The SDF view's value test through the successor: one point just inside the cube's +x face and one just
// outside, in the cube's mesh-local frame. The first evaluation warms the scene up itself. N other than the
// query's row count fails the task, naming the count.
TEST_F(SdfShapesGpuTest, EvaluatesTheCubeAroundItsFace)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/sdf_cube.usda")) << "Failed to attach ovstage USD";
    LiveSdfQuery live;
    ASSERT_TRUE(openSdfQuery(m_handle, "/World/Cube", live));
    ASSERT_EQ(live.rows, 1u);

    physx::PxScene* const scene =
        static_cast<physx::PxScene*>(read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/PhysicsScene"));
    ASSERT_NE(scene, nullptr);
    physx::PxCudaContextManager* const manager = scene->getCudaContextManager();
    ASSERT_NE(manager, nullptr);
    physx::PxCudaContext* const cuda = manager->getCudaContext();
    const int ordinal = static_cast<int>(manager->getDevice());

    const float hostPoints[6] = { 0.45f, 0.0f, 0.0f, 0.55f, 0.0f, 0.0f };
    float hostOut[8] = {};
    CUdeviceptr pointsDevice = 0;
    CUdeviceptr outDevice = 0;
    {
        physx::PxScopedCudaLock lock(*manager);
        ASSERT_EQ(cuda->memAlloc(&pointsDevice, sizeof(hostPoints)), 0);
        ASSERT_EQ(cuda->memAlloc(&outDevice, sizeof(hostOut)), 0);
        ASSERT_EQ(cuda->memcpyHtoD(pointsDevice, hostPoints, sizeof(hostPoints)), 0);
    }
    int64_t points3[3] = { 1, 2, 3 };
    int64_t out4[3] = { 1, 2, 4 };
    const DLTensor points = tensorOf(reinterpret_cast<void*>(pointsDevice), points3, 32, kDLCUDA, ordinal);
    DLTensor out = tensorOf(reinterpret_cast<void*>(outDevice), out4, 32, kDLCUDA, ordinal);
    {
        // The control for the refusal tests' count: an accepted first evaluation does warm the scene up.
        WarmupWarnings warmups;
        const ovphysx_result_t evaluated =
            complete_task(m_handle, ovphysx_evaluate_sdf_shapes(m_handle, live.query, &points, &out, kNoHandoff));
        EXPECT_EQ(evaluated.status, OVPHYSX_API_SUCCESS) << last_task_error();
        EXPECT_EQ(warmups.seen(), 1);
    }

    int64_t twoRows3[3] = { 2, 1, 3 };
    int64_t twoRows4[3] = { 2, 1, 4 };
    const DLTensor twoRowPoints = tensorOf(reinterpret_cast<void*>(pointsDevice), twoRows3, 32, kDLCUDA, ordinal);
    DLTensor twoRowOut = tensorOf(reinterpret_cast<void*>(outDevice), twoRows4, 32, kDLCUDA, ordinal);
    const ovphysx_result_t mismatched =
        complete_task(m_handle,
                      ovphysx_evaluate_sdf_shapes(m_handle, live.query, &twoRowPoints, &twoRowOut, kNoHandoff));
    EXPECT_EQ(mismatched.status, OVPHYSX_API_ERROR);
    EXPECT_NE(last_task_error().find("the query has 1 SDF shapes"), std::string::npos) << last_task_error();

    {
        physx::PxScopedCudaLock lock(*manager);
        EXPECT_EQ(cuda->memcpyDtoH(hostOut, outDevice, sizeof(hostOut)), 0);
    }
    EXPECT_LT(hostOut[3], 0.0f) << "inside the +x face";
    EXPECT_GT(hostOut[7], 0.0f) << "outside the +x face";
    EXPECT_GT(hostOut[0], 0.5f) << "gradient points out of the +x face";
    EXPECT_GT(hostOut[4], 0.5f) << "gradient points out of the +x face";

    // The handoff's API shape: a producer stream and its completion event are accepted and honoured, and the points
    // copied on that stream evaluate as written. A pageable copy lands before the call, so this cannot tell a waited
    // handoff from an ignored one; the Python test holds the producer stream back to prove the wait (this driver
    // wrapper exposes no host-function launch).
    constexpr unsigned int kNonBlockingStream = 0x1; // CU_STREAM_NON_BLOCKING
    constexpr unsigned int kNoTimingEvent = 0x2; // CU_EVENT_DISABLE_TIMING
    const float swappedPoints[6] = { 0.55f, 0.0f, 0.0f, 0.45f, 0.0f, 0.0f };
    float handedOff[8] = {};
    CUstream producer = nullptr;
    CUevent written = nullptr;
    {
        physx::PxScopedCudaLock lock(*manager);
        ASSERT_EQ(cuda->streamCreate(&producer, kNonBlockingStream), 0);
        ASSERT_EQ(cuda->eventCreate(&written, kNoTimingEvent), 0);
        ASSERT_EQ(cuda->memcpyHtoDAsync(pointsDevice, swappedPoints, sizeof(swappedPoints), producer), 0);
        ASSERT_EQ(cuda->eventRecord(written, producer), 0);
    }
    ovstage_cuda_sync_t handoff{};
    handoff.stream = reinterpret_cast<uintptr_t>(producer);
    handoff.wait_event = reinterpret_cast<uintptr_t>(written);
    const ovphysx_result_t afterProducer =
        complete_task(m_handle, ovphysx_evaluate_sdf_shapes(m_handle, live.query, &points, &out, handoff));
    EXPECT_EQ(afterProducer.status, OVPHYSX_API_SUCCESS) << last_task_error();
    {
        physx::PxScopedCudaLock lock(*manager);
        EXPECT_EQ(cuda->memcpyDtoH(handedOff, outDevice, sizeof(handedOff)), 0);
        cuda->eventDestroy(written);
        cuda->streamDestroy(producer);
        cuda->memFree(pointsDevice);
        cuda->memFree(outDevice);
    }
    EXPECT_GT(handedOff[3], 0.0f) << "the producer's first point is outside the +x face";
    EXPECT_LT(handedOff[7], 0.0f) << "the producer's second point is inside the +x face";
}
#endif // OVPHYSX_ENABLE_GPU_TESTS
