// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-3 AC-6 AC-7 AC-8
 *
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-11 AC-12
 *
 * @implements REQ-CAPI-CPU-001
 * @covers AC-5
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-16
 *
 * @implements REQ-CAPI-THREADING-001
 * @covers AC-6
 *
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-6
 *
 * TEST-CAPI-ASYNC-005: ovphysx_attach_ovstage and ovphysx_detach_ovstage queue one job
 * each on the instance worker, lifecycle work of different instances never overlaps,
 * attach and detach run synchronously while an OmniPVD recording is active, and the
 * worker uses CUDA only as the submitting thread allows.
 */

// Attach and detach as worker jobs: both return while the worker is held, a chain of long
// calls and a query needs no wait in between, a detach and the attach after it run in
// order, and two threads cycling attach and detach on their own instances never deadlock.
// While a recording is active, attach and detach run on the calling thread after every
// instance's earlier work (in their own processes, which a recording capability needs).
// A GPU attach runs in the CUDA context current on the submitting thread, which the worker
// does not keep, and in hard CPU-only mode the workers make no CUDA call.

#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "ovphysx_test_utils.h"
#include "session_test_helpers.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>
#include <ovphysx_companion/companion.h>
#include <carb/extras/ScopeExit.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace
{

constexpr const char* kBoxes = "tests/data/boxes_falling_on_groundplane.usda";
constexpr const char* kBoxesGpu = "tests/data/boxes_falling_on_groundplane_gpu.usda";
constexpr const char* kEnvs = "tests/data/basic_simulation.usda";
constexpr const char* kTables = "/World/envs/env*/table";
constexpr const char* kGpuScene = "/World/physicsScene";
constexpr float kDt = 1.0f / 60.0f;

class WorkerHold
{
public:
    explicit WorkerHold(ovphysx_instance_handle_t handle) : m_handle(handle)
    {
        held = ovphysx_set_worker_hold_for_test_internal(m_handle, true);
    }
    ~WorkerHold() { release(); }
    void release()
    {
        if (held)
            ovphysx_set_worker_hold_for_test_internal(m_handle, false);
        held = false;
    }
    bool held = false;

private:
    ovphysx_instance_handle_t m_handle;
};

ovphysx_task_status_t statusOf(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_NOT_FOUND;
    EXPECT_EQ(ovphysx_query_task_status(handle, task, &status).status, OVPHYSX_API_SUCCESS);
    return status;
}

bool attachedTo(ovphysx_instance_handle_t handle, const void* stage)
{
    bool attached = false;
    EXPECT_EQ(ovphysx_is_stage_attached(handle, stage, &attached).status, OVPHYSX_API_SUCCESS);
    return attached;
}

// Collects every task of handle and reports whether none failed.
bool waitAllSucceeded(ovphysx_instance_handle_t handle)
{
    ovphysx_wait_task_result_t all{};
    const ovphysx_api_status_t status = ovphysx_wait_all(handle, OVPHYSX_TIMEOUT_INFINITE, &all).status;
    const bool succeeded = status == OVPHYSX_API_SUCCESS && all.num_errors == 0;
    for (size_t i = 0; i < all.num_errors; ++i)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, all.error_tasks[i]);
        ADD_FAILURE() << "task " << all.error_tasks[i].op_index << " failed: "
                      << std::string(error.ptr ? error.ptr : "", error.length);
    }
    ovphysx_destroy_wait_task_result(&all);
    return succeeded;
}

// An instance the test owns: destroyed with every Stage the test registry holds for it.
struct OwnedInstance
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ~OwnedInstance()
    {
        if (handle == OVPHYSX_INVALID_HANDLE)
            return;
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    }
};

// A Stage populated from a USD file and sealed at ordinal 1, for a test that attaches it
// itself. Unless handed to the test registry of an instance (see registerWith()), it is
// destroyed after a detach from the instance it was attached to.
class SealedStage
{
public:
    SealedStage(const char* usdPath, const char* name)
    {
        if (!test_utils::register_physx_schemas_with_ovstage())
            return;
        ovstage_instance_desc_t desc{};
        desc.name = name;
        if (ovstage_create_instance(&desc, &m_stage) != OVSTAGE_OK)
        {
            m_stage = nullptr;
            return;
        }
        const ovx_string_t path{ usdPath, std::strlen(usdPath) };
        const ovstage_population_enqueue_result_t populate =
            ovphysx_sample_open_usd_from_file(m_stage, path, 1, 0.0, /*with_rendering=*/true);
        ovstage_population_op_wait_result_t populated{};
        bool ok = populate.status == OVSTAGE_OK &&
                  ovstage_population_wait_op(m_stage, populate.op_index, OVSTAGE_TIMEOUT_INFINITE, &populated) ==
                      OVSTAGE_OK;
        ovstage_write_floor_desc_t floorDesc{};
        floorDesc.ordinal = 1;
        floorDesc.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t floor = ok ? ovstage_advance_write_floor(m_stage, &floorDesc)
                                                  : ovstage_enqueue_result_t{};
        ovstage_op_wait_result_t sealed{};
        ok = ok && floor.status == OVSTAGE_OK &&
             ovstage_wait_op(m_stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, &sealed) == OVSTAGE_OK &&
             sealed.error_op_id_count == 0 && ovstage_release_op(m_stage, floor.op_index) == OVSTAGE_OK;
        if (!ok)
        {
            ovstage_destroy_instance(m_stage);
            m_stage = nullptr;
        }
    }
    ~SealedStage()
    {
        if (!m_stage)
            return;
        if (m_attachedTo != OVPHYSX_INVALID_HANDLE)
        {
            // Earlier work decides the attachment; a Stage that stays attached is leaked
            // rather than destroyed under its attachment.
            ovphysx_wait_task_result_t all{};
            ovphysx_wait_all(m_attachedTo, OVPHYSX_TIMEOUT_INFINITE, &all);
            ovphysx_destroy_wait_task_result(&all);
            bool attached = false;
            if (ovphysx_is_stage_attached(m_attachedTo, m_stage, &attached).status != OVPHYSX_API_SUCCESS ||
                (attached &&
                 test_utils::complete_task(m_attachedTo, ovphysx_detach_ovstage(m_attachedTo)).status !=
                     OVPHYSX_API_SUCCESS))
                return;
        }
        ovstage_destroy_instance(m_stage);
    }
    SealedStage(const SealedStage&) = delete;
    SealedStage& operator=(const SealedStage&) = delete;

    ovstage_instance_t* get() const { return m_stage; }
    bool valid() const { return m_stage != nullptr; }
    // The instance the test submitted an attach of this Stage to.
    void submittedTo(ovphysx_instance_handle_t handle) { m_attachedTo = handle; }
    // Hands the Stage to the test registry of handle: session reads use its dictionary,
    // and destroy_ovstage_test_attachments() detaches and destroys it.
    void registerWith(ovphysx_instance_handle_t handle)
    {
        std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
        test_utils::ovstage_test_attachments()[handle].push_back({ m_stage, 1 });
        m_stage = nullptr;
    }

private:
    ovstage_instance_t* m_stage = nullptr;
    ovphysx_instance_handle_t m_attachedTo = OVPHYSX_INVALID_HANDLE;
};

// The world position of every table body, keyed by body path.
session_test::Column tablePositions(ovphysx_instance_handle_t handle)
{
    session_test::Column positions;
    EXPECT_TRUE(
        session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, positions, kTables));
    return positions;
}

class WorkerLifecycle : public ::testing::Test
{
protected:
    // The process's shared CPU instance, created on first use: the recorded cases run in
    // processes of their own whose first instance must be created recording-capable.
    ovphysx_instance_handle_t shared()
    {
        if (m_shared == 0)
        {
            EXPECT_TRUE(ensureSharedCpuInstance());
            m_shared = sharedCpuInstance();
            // Start from an empty task table.
            ovphysx_wait_task_result_t residual{};
            ovphysx_wait_all(m_shared, OVPHYSX_TIMEOUT_INFINITE, &residual);
            ovphysx_destroy_wait_task_result(&residual);
        }
        return m_shared;
    }

    void TearDown() override
    {
        if (m_shared != 0)
        {
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_shared));
        }
    }

private:
    ovphysx_instance_handle_t m_shared = 0;
};

TEST_F(WorkerLifecycle, SelectedExecutionModeTakesEffect)
{
    const std::string mode = ovphysxTestExecution();
    if (mode.empty())
        GTEST_SKIP() << "default execution mode has no private override";

    const ovphysx_instance_handle_t handle = shared();
    bool workerExited = false;
    ASSERT_TRUE(ovphysx_worker_exited_for_test_internal(handle, &workerExited));
    if (mode == "noworker")
    {
        EXPECT_TRUE(workerExited);
        return;
    }

    ASSERT_EQ(mode, "queue");
    EXPECT_FALSE(workerExited);
    ASSERT_TRUE(ovphysx_hold_worker_after_next_submit_for_test_internal(handle));
    struct ReleaseWorker
    {
        ovphysx_instance_handle_t handle;
        ~ReleaseWorker()
        {
            if (handle != OVPHYSX_INVALID_HANDLE)
                ovphysx_set_worker_hold_for_test_internal(handle, false);
        }
    } release{ handle };
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr);
    ASSERT_EQ(opened.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(handle, opened), OVPHYSX_TASK_STATUS_PENDING)
        << "the queue override did not queue an idle short call";
    ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(handle, false));
    release.handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_TRUE(waitForTaskSuccess(handle, opened));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, query)));
}

TEST_F(WorkerLifecycle, AttachReturnsWhilePending)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = shared();
    SealedStage stage(kBoxes, "worker-lifecycle-pending");
    ASSERT_TRUE(stage.valid());
    {
        WorkerHold hold(handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(handle, stage.get(), 1);
        ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
        stage.submittedTo(handle);
        EXPECT_NE(attach.op_index, 0u);
        EXPECT_EQ(attach.instance, handle);
        EXPECT_EQ(statusOf(handle, attach), OVPHYSX_TASK_STATUS_PENDING);
        EXPECT_FALSE(attachedTo(handle, stage.get())) << "the snapshot reports only an attach that ran";
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(handle, attach));
    }
    EXPECT_TRUE(attachedTo(handle, stage.get()));
    {
        WorkerHold hold(handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(handle);
        ASSERT_EQ(detach.status, OVPHYSX_API_SUCCESS);
        EXPECT_NE(detach.op_index, 0u);
        EXPECT_EQ(statusOf(handle, detach), OVPHYSX_TASK_STATUS_PENDING);
        EXPECT_TRUE(attachedTo(handle, stage.get())) << "a queued detach has not run yet";
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(handle, detach));
    }
    EXPECT_FALSE(attachedTo(handle, nullptr));
}

TEST_F(WorkerLifecycle, AcceptedAttachKeepsLaterCompanionOperationBusy)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = shared();
    SealedStage stage(kBoxes, "worker-lifecycle-companion-admission");
    ASSERT_TRUE(stage.valid());
    WorkerHold hold(handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(handle, stage.get(), 1);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    stage.submittedTo(handle);
    ovphysx_companion_api_t companion{};
    ASSERT_EQ(ovphysx_companion_acquire(1, sizeof(companion), &companion), OVPHYSX_COMPANION_OK);
    ovphysx_companion_scene_statistics_t statistics{};
    EXPECT_EQ(companion.scene_statistics(0, "/World", sizeof(statistics), &statistics), OVPHYSX_COMPANION_BUSY);
    hold.release();
    EXPECT_TRUE(waitForTaskSuccess(handle, attach));
    EXPECT_EQ(companion.scene_statistics(0, "/World", sizeof(statistics), &statistics),
              OVPHYSX_COMPANION_NOT_FOUND);
}

// attach, update, clone, step and a query submitted back to back run in submission order:
// each job sees the attachment the attach job made, and one wait_all collects them all.
TEST_F(WorkerLifecycle, OrderedChainWithoutWaits)
{
    const ovphysx_instance_handle_t handle = shared();
    SealedStage stage(kEnvs, "worker-lifecycle-chain");
    ASSERT_TRUE(stage.valid());

    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(handle, stage.get(), 1);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    stage.submittedTo(handle);
    const ovphysx_enqueue_result_t update = ovphysx_update_from_ovstage(handle, ovstage_ordinal_range_t{ 1, 1, true });
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/envs/env1");
    const ovphysx_enqueue_result_t clone =
        ovphysx_clone(handle, OVPHYSX_LITERAL("/World/envs/env0"), &target, 1, nullptr, nullptr);
    const ovphysx_enqueue_result_t step = ovphysx_step(handle, kDt);
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr);
    for (const ovphysx_enqueue_result_t& task : { update, clone, step, opened })
    {
        ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    }
    EXPECT_LT(attach.op_index, update.op_index);
    EXPECT_LT(update.op_index, clone.op_index);
    EXPECT_LT(clone.op_index, step.op_index);
    EXPECT_LT(step.op_index, opened.op_index);
    // The query is queued behind the long calls.
    EXPECT_TRUE(waitAllSucceeded(handle));
    EXPECT_NE(query, 0u);

    stage.registerWith(handle);
    const session_test::Column positions = tablePositions(handle);
    EXPECT_EQ(positions.count("/World/envs/env0/table"), 1u);
    EXPECT_EQ(positions.count("/World/envs/env1/table"), 1u) << "the clone ran after the attach";
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, query)));
}

// The number of threads of this process, or -1 where it cannot be read.
int processThreadCount()
{
#if defined(__linux__)
    std::error_code error;
    int count = 0;
    for (std::filesystem::directory_iterator task("/proc/self/task", error), end; !error && task != end;
         task.increment(error))
        ++count;
    return error ? -1 : count;
#else
    return -1;
#endif
}

// An instance created while OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION is false starts no
// thread, and each of its calls has finished when it returns: attach, update, clone, step,
// a job and detach return finished tasks, the job ran on the calling thread, and the waits
// only collect them. An instance created while it is true starts one thread; its calls
// return while their jobs are pending (AttachReturnsWhilePending, WorkerStep.ReturnsWhilePending).
TEST_F(WorkerLifecycle, InstanceWithoutAWorkerThreadFinishesEachCallBeforeItReturns)
{
    shared(); // The runtime is up: creating an instance starts no thread but its worker.
    const int threads = processThreadCount();
    OwnedInstance withWorker;
    OwnedInstance withoutWorker;
    ASSERT_EQ(createInstanceWithWorkerThread(true, &withWorker.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(createInstanceWithWorkerThread(false, &withoutWorker.handle).status, OVPHYSX_API_SUCCESS);
    if (threads >= 0)
    {
        EXPECT_EQ(processThreadCount(), threads + 1) << "only the instance with a worker thread started one";
    }

    const ovphysx_instance_handle_t handle = withoutWorker.handle;
    SealedStage stage(kEnvs, "worker-lifecycle-without-worker");
    ASSERT_TRUE(stage.valid());
    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(handle, stage.get(), 1);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    stage.submittedTo(handle);
    EXPECT_EQ(statusOf(handle, attach), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_TRUE(attachedTo(handle, stage.get()));

    ovphysx_string_t target = OVPHYSX_LITERAL("/World/envs/env1");
    RanOn job;
    const ovphysx_enqueue_result_t tasks[] = {
        ovphysx_update_from_ovstage(handle, ovstage_ordinal_range_t{ 1, 1, true }),
        ovphysx_clone(handle, OVPHYSX_LITERAL("/World/envs/env0"), &target, 1, nullptr, nullptr),
        ovphysx_step(handle, kDt),
        ovphysx_submit_test_callback_job_for_test_internal(handle, &RanOn::record, &job),
    };
    for (const ovphysx_enqueue_result_t& task : tasks)
    {
        ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(statusOf(handle, task), OVPHYSX_TASK_STATUS_SUCCEEDED);
    }
    EXPECT_TRUE(job.onThisThread());
    EXPECT_TRUE(waitAllSucceeded(handle));
    stage.registerWith(handle);
    EXPECT_EQ(tablePositions(handle).count("/World/envs/env1/table"), 1u) << "the clone ran";

    const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(handle);
    ASSERT_EQ(detach.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(handle, detach), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_TRUE(waitForTaskSuccess(handle, detach));
    EXPECT_FALSE(attachedTo(handle, nullptr));
}

// An instance created with a worker thread, under OVPHYSX_TEST_EXECUTION=noworker too, runs
// a queued job on that thread rather than on a thread that waits for it with a deadline,
// and destroying the instance afterwards succeeds.
TEST_F(WorkerLifecycle, QueuedJobRunsOnTheWorkerThread)
{
    shared(); // The runtime is up.
    OwnedInstance withWorker;
    ASSERT_EQ(createInstanceWithWorkerThread(true, &withWorker.handle).status, OVPHYSX_API_SUCCESS);
    RanOn job;
    const ovphysx_enqueue_result_t task =
        ovphysx_submit_test_callback_job_for_test_internal(withWorker.handle, &RanOn::record, &job);
    ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(withWorker.handle, task, kWorkerRunsIt));
    EXPECT_TRUE(job.ran.load());
    EXPECT_FALSE(job.onThisThread());
}

// A detach and an attach of another Stage submitted back to back, both behind a held
// worker, run in submission order.
TEST_F(WorkerLifecycle, DetachThenAttachWithoutWait)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = shared();
    SealedStage first(kBoxes, "worker-lifecycle-first");
    SealedStage second(kEnvs, "worker-lifecycle-second");
    ASSERT_TRUE(first.valid() && second.valid());
    ASSERT_TRUE(test_utils::complete_task(handle, ovphysx_attach_ovstage(handle, first.get(), 1)).status ==
                OVPHYSX_API_SUCCESS);
    first.submittedTo(handle);
    {
        WorkerHold hold(handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(handle);
        const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(handle, second.get(), 1);
        ASSERT_EQ(detach.status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS) << "the already-attached check belongs to the attach job";
        second.submittedTo(handle);
        EXPECT_LT(detach.op_index, attach.op_index);
        EXPECT_EQ(statusOf(handle, detach), OVPHYSX_TASK_STATUS_PENDING);
        EXPECT_EQ(statusOf(handle, attach), OVPHYSX_TASK_STATUS_PENDING);
        hold.release();
        EXPECT_TRUE(waitAllSucceeded(handle));
    }
    EXPECT_FALSE(attachedTo(handle, first.get()));
    EXPECT_TRUE(attachedTo(handle, second.get()));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_step(handle, kDt)));
}

// Two threads cycle attach, step and detach on their own instances without waiting in
// between: lifecycle jobs of the two instances never overlap or deadlock, and every task
// succeeds.
TEST_F(WorkerLifecycle, ConcurrentLifecycleOnTwoInstances)
{
    OwnedInstance instances[2];
    for (OwnedInstance& instance : instances)
        ASSERT_EQ(ovphysx_create_instance(&instance.handle).status, OVPHYSX_API_SUCCESS);
    std::atomic<int> failedRounds{ 0 };
    auto cycle = [&failedRounds](ovphysx_instance_handle_t handle, int index) {
        constexpr int kRounds = 6;
        for (int round = 0; round < kRounds; ++round)
        {
            const std::string name = "worker-lifecycle-concurrent-" + std::to_string(index);
            SealedStage stage(round % 2 ? kBoxes : kEnvs, name.c_str());
            if (!stage.valid())
            {
                failedRounds.fetch_add(1);
                continue;
            }
            const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(handle, stage.get(), 1);
            stage.submittedTo(handle);
            const ovphysx_enqueue_result_t step = ovphysx_step(handle, kDt);
            const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(handle);
            ovphysx_wait_task_result_t all{};
            const ovphysx_api_status_t waited = ovphysx_wait_all(handle, OVPHYSX_TIMEOUT_INFINITE, &all).status;
            const bool ok = attach.status == OVPHYSX_API_SUCCESS && step.status == OVPHYSX_API_SUCCESS &&
                            detach.status == OVPHYSX_API_SUCCESS && waited == OVPHYSX_API_SUCCESS &&
                            all.num_errors == 0;
            ovphysx_destroy_wait_task_result(&all);
            if (!ok)
                failedRounds.fetch_add(1);
        }
    };
    // SealedStage registers the schemas through a helper that is not thread-safe, so
    // register them here, before the threads build their Stages.
    ASSERT_TRUE(test_utils::register_physx_schemas_with_ovstage());
    std::thread first(cycle, instances[0].handle, 0);
    std::thread second(cycle, instances[1].handle, 1);
    first.join();
    second.join();
    EXPECT_EQ(failedRounds.load(), 0);
}

// In hard CPU-only mode (the cpu pass sets OVPHYSX_DISABLE_GPU) the worker of a new
// instance runs attach, a whole step and detach without one CUDA call, its first job
// included. The primary-context check of CpuNoCudaContextGpuTest cannot run in that pass,
// where ovstage activates the primary context first.
TEST_F(WorkerLifecycle, CpuOnlyWorkerMakesNoCudaCall)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    bool cpuOnly = false;
    ASSERT_EQ(ovphysx_get_cpu_mode(&cpuOnly).status, OVPHYSX_API_SUCCESS);
    if (!cpuOnly)
        GTEST_SKIP() << "needs hard CPU-only mode";
    OwnedInstance instance;
    ASSERT_EQ(ovphysx_create_instance(&instance.handle).status, OVPHYSX_API_SUCCESS);
    SealedStage stage(kBoxes, "worker-lifecycle-cpu-only");
    ASSERT_TRUE(stage.valid());

    const uint64_t before = ovphysx_worker_cuda_calls_for_test_internal();
    {
        WorkerHold hold(instance.handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(instance.handle, stage.get(), 1);
        stage.submittedTo(instance.handle);
        const ovphysx_enqueue_result_t step = ovphysx_step(instance.handle, kDt);
        const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(instance.handle);
        for (const ovphysx_enqueue_result_t& task : { attach, step, detach })
        {
            ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
            EXPECT_EQ(statusOf(instance.handle, task), OVPHYSX_TASK_STATUS_PENDING) << "all three run on the worker";
        }
        hold.release();
        EXPECT_TRUE(waitAllSucceeded(instance.handle));
    }
    EXPECT_EQ(ovphysx_worker_cuda_calls_for_test_internal(), before);
}

// ---------------------------------------------------------------------------
// Recorded lifecycle calls. Each case runs in a process of its own (scripts/test_cpp.cmake):
// recording needs a capability set before the process creates its first instance.
// ---------------------------------------------------------------------------

struct RecordingDirectory
{
    explicit RecordingDirectory(const char* name)
        : path(std::filesystem::temp_directory_path() /
               (std::string(name) + "_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
    {
        std::error_code ignored;
        std::filesystem::create_directories(path, ignored);
    }
    ~RecordingDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    std::filesystem::path path;
};

// Sets the recording capability, which must happen before the process creates its first
// instance and cannot change later, once per process (so a repeated case still runs).
::testing::AssertionResult makeRecordingCapable()
{
    static bool configured = false;
    if (configured)
        return ::testing::AssertionSuccess();
    const ovphysx_result_t result =
        ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, ovphysx_config_value_bool(true));
    if (result.status != OVPHYSX_API_SUCCESS)
        return ::testing::AssertionFailure() << "the recording capability is not settable: status " << result.status;
    configured = true;
    return ::testing::AssertionSuccess();
}

ovphysx_result_t startFileRecording(ovphysx_instance_handle_t handle, const std::filesystem::path& path)
{
    const std::string text = path.u8string();
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE, ovphysx_cstr(text.c_str()), OVPHYSX_LITERAL(""), 0, 0,
    };
    return ovphysx_ovd_start_recording(handle, &destination, nullptr);
}

bool recordingOf(ovphysx_instance_handle_t handle)
{
    bool recording = false;
    EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    return recording;
}

uintmax_t fileSize(const std::filesystem::path& path)
{
    std::error_code error;
    const uintmax_t size = std::filesystem::file_size(path, error);
    return error ? 0 : size;
}

// While the owner records, its detach runs on the calling thread: it first lets a peer's
// step, queued behind the peer's held worker, finish without collecting it, then tears
// down, finalizes the recording and returns a finished task. A detach job could not do
// this: finalizing needs every instance's lane, which a job holding its own must never
// wait for. The peer has a worker thread in every test execution mode.
TEST_F(WorkerLifecycle, RecordedDetachIsSynchronousAndDrainsPeers)
{
    ASSERT_TRUE(makeRecordingCapable());
    RecordingDirectory directory("ovphysx_worker_lifecycle_recorded_detach");
    const std::filesystem::path capture = directory.path / "owner.ovd";
    OwnedInstance owner;
    OwnedInstance peer;
    ASSERT_EQ(ovphysx_create_instance(&owner.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(createInstanceWithWorkerThread(true, &peer.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(owner.handle, kBoxes, 1, "recorded-owner"));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(peer.handle, kBoxes, 1, "recorded-peer"));
    ASSERT_EQ(ovphysx_step_sync(owner.handle, kDt).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(peer.handle, kDt).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(startFileRecording(owner.handle, capture).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(owner.handle, kDt).status, OVPHYSX_API_SUCCESS);

    WorkerHold hold(peer.handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t peerStep = ovphysx_step(peer.handle, kDt);
    ASSERT_EQ(peerStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(statusOf(peer.handle, peerStep), OVPHYSX_TASK_STATUS_PENDING);
    std::thread releaser([&hold] {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        hold.release();
    });
    const auto submitted = std::chrono::steady_clock::now();
    const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(owner.handle);
    const auto returned = std::chrono::steady_clock::now();
    releaser.join();

    ASSERT_EQ(detach.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(detach.op_index, 0u);
    EXPECT_GE(returned - submitted, std::chrono::milliseconds(150)) << "the detach waits for the peer's queued step";
    EXPECT_EQ(statusOf(owner.handle, detach), OVPHYSX_TASK_STATUS_SUCCEEDED) << "a recorded detach returns a finished task";
    EXPECT_EQ(statusOf(peer.handle, peerStep), OVPHYSX_TASK_STATUS_SUCCEEDED)
        << "the peer's step ran first and stays uncollected";
    EXPECT_FALSE(attachedTo(owner.handle, nullptr));
    EXPECT_FALSE(recordingOf(owner.handle)) << "the detach finalized the owner's recording before it returned";
    EXPECT_GT(fileSize(capture), 0u);

    ovphysx_wait_task_result_t waited{};
    EXPECT_EQ(ovphysx_wait_task(peer.handle, peerStep, OVPHYSX_TIMEOUT_POLL, &waited).status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&waited);
    EXPECT_EQ(ovphysx_wait_task(peer.handle, peerStep, OVPHYSX_TIMEOUT_POLL, &waited).status, OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&waited);
    EXPECT_TRUE(waitForTaskSuccess(owner.handle, detach, OVPHYSX_TIMEOUT_POLL));

    // The finalized stream frees the recording: the peer can start its own.
    ASSERT_EQ(startFileRecording(peer.handle, directory.path / "peer.ovd").status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(peer.handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_ovd_stop_recording(peer.handle).status, OVPHYSX_API_SUCCESS);
}

// A stage-less peer records the only attachment, A's. A's detach finalizes the recording
// as the last live attachment, and B's attach submitted right after it must not run
// first: the recorded detach runs on the calling thread and has finalized the recording
// when it returns, so the attach that follows is an ordinary job. B has a worker thread
// in every test execution mode.
TEST_F(WorkerLifecycle, RecordedDetachThenAttachKeepsOrder)
{
    ASSERT_TRUE(makeRecordingCapable());
    RecordingDirectory directory("ovphysx_worker_lifecycle_recorded_order");
    const std::filesystem::path capture = directory.path / "peer.ovd";
    OwnedInstance a;
    OwnedInstance b;
    OwnedInstance recorder;
    ASSERT_EQ(ovphysx_create_instance(&a.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(createInstanceWithWorkerThread(true, &b.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(&recorder.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(a.handle, kBoxes, 1, "recorded-order-a"));
    ASSERT_EQ(ovphysx_step_sync(a.handle, kDt).status, OVPHYSX_API_SUCCESS);
    SealedStage stageB(kEnvs, "recorded-order-b");
    ASSERT_TRUE(stageB.valid());
    ASSERT_EQ(startFileRecording(recorder.handle, capture).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(a.handle, kDt).status, OVPHYSX_API_SUCCESS);

    const ovphysx_enqueue_result_t detach = ovphysx_detach_ovstage(a.handle);
    ASSERT_EQ(detach.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(a.handle, detach), OVPHYSX_TASK_STATUS_SUCCEEDED) << "a recorded detach returns a finished task";
    EXPECT_FALSE(recordingOf(recorder.handle)) << "the detach finalized the recording before it returned";
    ovphysx_enqueue_result_t attach{};
    {
        // Behind B's held worker an attach job stays queued. The hold ends on its own, so
        // an attach that wrongly ran on this thread, after every lane, could not hang.
        WorkerHold hold(b.handle);
        ASSERT_TRUE(hold.held);
        std::thread releaser([&hold] {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            hold.release();
        });
        attach = ovphysx_attach_ovstage(b.handle, stageB.get(), 1);
        const ovphysx_task_status_t status = statusOf(b.handle, attach);
        releaser.join();
        ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
        stageB.submittedTo(b.handle);
        EXPECT_EQ(status, OVPHYSX_TASK_STATUS_PENDING) << "the recording had ended, so the attach is an ordinary job";
    }
    EXPECT_TRUE(waitForTaskSuccess(b.handle, attach));
    EXPECT_TRUE(waitForTaskSuccess(a.handle, detach, OVPHYSX_TIMEOUT_POLL));
    EXPECT_GT(fileSize(capture), 0u);

    // The finalized stream frees the recording: B can record its new attachment.
    ASSERT_EQ(startFileRecording(b.handle, directory.path / "b.ovd").status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(b.handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_ovd_stop_recording(b.handle).status, OVPHYSX_API_SUCCESS);
}

#if OVPHYSX_ENABLE_GPU_TESTS

// The CUDA driver entry points these cases need beyond IOptionalCuda, loaded at run time as
// test_cpu_no_cuda_context.cpp does, so the test binary links no CUDA library.
struct CudaDriver
{
    int (*cuInit)(unsigned int) = nullptr;
    int (*cuDeviceGet)(int*, int) = nullptr;
    int (*cuDevicePrimaryCtxRetain)(void**, int) = nullptr;
    int (*cuDevicePrimaryCtxRelease)(int) = nullptr;
    int (*cuStreamBeginCapture)(void*, int) = nullptr;
    int (*cuStreamEndCapture)(void*, void**) = nullptr;
    int (*cuGraphInstantiateWithFlags)(void**, void*, unsigned long long) = nullptr;
    int (*cuGraphLaunch)(void*, void*) = nullptr;
    int (*cuGraphExecDestroy)(void*) = nullptr;
    int (*cuGraphDestroy)(void*) = nullptr;

    bool load()
    {
#if defined(_WIN32)
        HMODULE library = LoadLibraryA("nvcuda.dll");
        auto symbol = [library](const char* name) {
            return library ? reinterpret_cast<void*>(GetProcAddress(library, name)) : nullptr;
        };
#else
        void* library = dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
        if (!library)
            library = dlopen("libcuda.so", RTLD_NOW | RTLD_LOCAL);
        auto symbol = [library](const char* name) { return library ? dlsym(library, name) : nullptr; };
#endif
        cuInit = reinterpret_cast<decltype(cuInit)>(symbol("cuInit"));
        cuDeviceGet = reinterpret_cast<decltype(cuDeviceGet)>(symbol("cuDeviceGet"));
        cuDevicePrimaryCtxRetain = reinterpret_cast<decltype(cuDevicePrimaryCtxRetain)>(symbol("cuDevicePrimaryCtxRetain"));
        cuDevicePrimaryCtxRelease =
            reinterpret_cast<decltype(cuDevicePrimaryCtxRelease)>(symbol("cuDevicePrimaryCtxRelease_v2"));
        if (!cuDevicePrimaryCtxRelease)
            cuDevicePrimaryCtxRelease =
                reinterpret_cast<decltype(cuDevicePrimaryCtxRelease)>(symbol("cuDevicePrimaryCtxRelease"));
        cuStreamBeginCapture = reinterpret_cast<decltype(cuStreamBeginCapture)>(symbol("cuStreamBeginCapture_v2"));
        cuStreamEndCapture = reinterpret_cast<decltype(cuStreamEndCapture)>(symbol("cuStreamEndCapture"));
        cuGraphInstantiateWithFlags =
            reinterpret_cast<decltype(cuGraphInstantiateWithFlags)>(symbol("cuGraphInstantiateWithFlags"));
        cuGraphLaunch = reinterpret_cast<decltype(cuGraphLaunch)>(symbol("cuGraphLaunch"));
        cuGraphExecDestroy = reinterpret_cast<decltype(cuGraphExecDestroy)>(symbol("cuGraphExecDestroy"));
        cuGraphDestroy = reinterpret_cast<decltype(cuGraphDestroy)>(symbol("cuGraphDestroy"));
        return cuInit && cuDeviceGet && cuDevicePrimaryCtxRetain && cuDevicePrimaryCtxRelease &&
               cuStreamBeginCapture && cuStreamEndCapture && cuGraphInstantiateWithFlags && cuGraphLaunch &&
               cuGraphExecDestroy && cuGraphDestroy && cuInit(0) == 0;
    }
};

// Device 0's primary context, retained for the scope.
struct PrimaryContext
{
    explicit PrimaryContext(CudaDriver& driver) : m_driver(driver)
    {
        if (m_driver.cuDeviceGet(&m_device, 0) == 0 && m_driver.cuDevicePrimaryCtxRetain(&m_context, m_device) == 0)
            context = reinterpret_cast<uintptr_t>(m_context);
    }
    ~PrimaryContext()
    {
        if (context)
            m_driver.cuDevicePrimaryCtxRelease(m_device);
    }
    uintptr_t context = 0;

private:
    CudaDriver& m_driver;
    int m_device = 0;
    void* m_context = nullptr;
};

// Sets a process-wide boolean Carbonite setting for the scope and restores its value after.
struct BoolSettingScope
{
    BoolSettingScope(const char* setting, bool value) : path(setting)
    {
        bool current = false;
        previous = ovphysx_test_get_settings_bool(path, &current).status == OVPHYSX_API_SUCCESS && current;
        applied = write(value);
    }
    ~BoolSettingScope()
    {
        if (applied)
            write(previous);
    }
    bool write(bool value) const
    {
        return value ? ovphysx_set_raw_string_setting_for_test_internal(path, "true", 4)
                     : ovphysx_set_raw_string_setting_for_test_internal(path, "false", 5);
    }
    const char* path;
    bool previous = false;
    bool applied = false;
};

bool skipWithoutCuda(omni::physx::IOptionalCuda* cuda)
{
    return !cuda || !cuda->cudaAvailable();
}

// A synchronous attach on an instance without a worker keeps using the calling thread's
// already-current CUDA context. It probes that context once at submission, but does not
// push and pop it as a queued worker job would.
TEST_F(WorkerLifecycle, ActiveContextAttachWithoutWorkerGpuTest)
{
    OwnedInstance gpu;
    ASSERT_EQ(createInstanceWithWorkerThread(false, &gpu.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for the workerless active-context attach";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load()) << "CUDA driver entry points are missing";
    PrimaryContext primary(driver);
    ASSERT_NE(primary.context, 0u);
    SealedStage stage(kBoxesGpu, "worker-lifecycle-workerless-active-context");
    ASSERT_TRUE(stage.valid());
    BoolSettingScope activeContext("/persistent/physics/useActiveCudaContext", true);
    ASSERT_TRUE(activeContext.applied);

    ovphysx::test_cuda::ScopedCudaContextDetach callerStack(cuda);
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    ASSERT_TRUE(cuda->ctxPushCurrent(primary.context, nullptr));
    const uint64_t callsBefore = ovphysx_worker_cuda_calls_for_test_internal();
    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(gpu.handle, stage.get(), 1);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    stage.registerWith(gpu.handle);
    EXPECT_EQ(statusOf(gpu.handle, attach), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_EQ(ovphysx_worker_cuda_calls_for_test_internal() - callsBefore, 1u)
        << "a workerless attach must not reproduce its caller's context as a worker would";
    uintptr_t current = 0;
    ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
    EXPECT_EQ(current, primary.context);

    ASSERT_EQ(ovphysx_step_sync(gpu.handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(session_test::cudaContextOf(gpu.handle, kGpuScene, OVPHYSX_PHYSX_TYPE_SCENE), primary.context);
    ASSERT_TRUE(cuda->ctxPopCurrent(nullptr, nullptr));
    EXPECT_TRUE(callerStack.restore());
}

// An active-context attach creates its GPU scene in the CUDA context current on the
// submitting thread when the call is made, although the job runs later on the instance
// worker and the thread has changed its context meanwhile; the submitting thread's context
// stays as the thread left it. The worker keeps no context bound afterwards: a second
// attach submitted without a current context finds none on the worker either, so its
// scene falls back to the CPU.
TEST_F(WorkerLifecycle, ActiveContextAttachGpuTest)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    // The runtime's CUDA interface exists once an instance does.
    OwnedInstance gpu;
    ASSERT_EQ(ovphysx_create_instance(&gpu.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for the active-context attach";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load()) << "CUDA driver entry points are missing";
    PrimaryContext primary(driver);
    ASSERT_NE(primary.context, 0u);
    // Both Stages exist before the test controls the calling thread's CUDA context.
    SealedStage first(kBoxesGpu, "worker-lifecycle-active-context");
    SealedStage second(kBoxesGpu, "worker-lifecycle-no-context");
    ASSERT_TRUE(first.valid() && second.valid());
    BoolSettingScope activeContext("/persistent/physics/useActiveCudaContext", true);
    ASSERT_TRUE(activeContext.applied);

    ovphysx::test_cuda::ScopedCudaContextDetach callerStack(cuda);
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    ASSERT_TRUE(cuda->ctxPushCurrent(primary.context, nullptr));
    const uint64_t callsBefore = ovphysx_worker_cuda_calls_for_test_internal();
    ovphysx_enqueue_result_t attach{};
    {
        WorkerHold hold(gpu.handle);
        ASSERT_TRUE(hold.held);
        attach = ovphysx_attach_ovstage(gpu.handle, first.get(), 1);
        ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
        first.registerWith(gpu.handle);
        EXPECT_EQ(statusOf(gpu.handle, attach), OVPHYSX_TASK_STATUS_PENDING);
        // The job runs in the context captured at submission, whatever the thread does next.
        ASSERT_TRUE(cuda->ctxPopCurrent(nullptr, nullptr));
        ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    }
    ASSERT_TRUE(waitForTaskSuccess(gpu.handle, attach));
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda)) << "the attach changed the calling thread's context";
    EXPECT_GE(ovphysx_worker_cuda_calls_for_test_internal() - callsBefore, 3u)
        << "capture at submission, then push and pop on the worker";
    ASSERT_EQ(ovphysx_step_sync(gpu.handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(session_test::cudaContextOf(gpu.handle, kGpuScene, OVPHYSX_PHYSX_TYPE_SCENE), primary.context)
        << "the GPU scene runs in the submitting thread's context";

    ASSERT_EQ(test_utils::complete_task(gpu.handle, ovphysx_detach_ovstage(gpu.handle)).status, OVPHYSX_API_SUCCESS);
    {
        WorkerHold hold(gpu.handle);
        ASSERT_TRUE(hold.held);
        ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
        const ovphysx_enqueue_result_t reattach = ovphysx_attach_ovstage(gpu.handle, second.get(), 1);
        ASSERT_EQ(reattach.status, OVPHYSX_API_SUCCESS);
        second.registerWith(gpu.handle);
        EXPECT_EQ(statusOf(gpu.handle, reattach), OVPHYSX_TASK_STATUS_PENDING);
        // The infinite wait takes the job while the worker is held. Its new, nested
        // context stack must not replace the absence captured at submission.
        ASSERT_TRUE(cuda->ctxPushCurrent(primary.context, nullptr));
        CARB_SCOPE_EXIT { EXPECT_TRUE(cuda->ctxPopCurrent(nullptr, nullptr)); };
        {
            ASSERT_TRUE(cuda->ctxPushCurrent(primary.context, nullptr));
            CARB_SCOPE_EXIT { EXPECT_TRUE(cuda->ctxPopCurrent(nullptr, nullptr)); };
            ASSERT_TRUE(waitForTaskSuccess(gpu.handle, reattach));
            uintptr_t current = 0;
            ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
            EXPECT_EQ(current, primary.context);
        }
        uintptr_t current = 0;
        ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
        EXPECT_EQ(current, primary.context) << "the wait lost a nested caller context";
    }
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    ASSERT_EQ(ovphysx_step_sync(gpu.handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(session_test::cudaContextOf(gpu.handle, kGpuScene, OVPHYSX_PHYSX_TYPE_SCENE), 0u)
        << "attach used the waiting thread's context instead of the submitted absence";
    EXPECT_TRUE(callerStack.restore());
}

// Run alone to create the first device-ordinal context manager in the process.
// The runtime may bind that context while the submitted empty stack is active;
// it must not remain hidden beneath the waiting thread's restored contexts.
TEST_F(WorkerLifecycle, NoContextOrdinalAttachRestoresCallerGpuTest)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnedInstance gpu;
    ASSERT_EQ(ovphysx_create_instance(&gpu.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for the device-ordinal attach";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load());
    PrimaryContext primary(driver);
    ASSERT_NE(primary.context, 0u);
    SealedStage stage(kBoxesGpu, "worker-lifecycle-ordinal-context");
    ASSERT_TRUE(stage.valid());
    BoolSettingScope activeContext("/persistent/physics/useActiveCudaContext", false);
    ASSERT_TRUE(activeContext.applied);
    ASSERT_EQ(ovphysx_set_config(gpu.handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
                                ovphysx_config_value_string({"0", 1})).status, OVPHYSX_API_SUCCESS);
    ovphysx::test_cuda::ScopedCudaContextDetach callerStack(cuda);
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    {
        WorkerHold hold(gpu.handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(gpu.handle, stage.get(), 1);
        ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
        stage.registerWith(gpu.handle);
        ASSERT_TRUE(cuda->ctxPushCurrent(primary.context, nullptr));
        CARB_SCOPE_EXIT { EXPECT_TRUE(cuda->ctxPopCurrent(nullptr, nullptr)); };
        {
            ASSERT_TRUE(cuda->ctxPushCurrent(primary.context, nullptr));
            CARB_SCOPE_EXIT { EXPECT_TRUE(cuda->ctxPopCurrent(nullptr, nullptr)); };
            ASSERT_TRUE(waitForTaskSuccess(gpu.handle, attach));
            uintptr_t current = 0;
            ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
            EXPECT_EQ(current, primary.context);
        }
        uintptr_t current = 0;
        ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
        EXPECT_EQ(current, primary.context);
    }
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda))
        << "attachment left a context underneath the caller's restored stack";
    ASSERT_EQ(ovphysx_step_sync(gpu.handle, kDt).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(session_test::cudaContextOf(gpu.handle, kGpuScene, OVPHYSX_PHYSX_TYPE_SCENE), primary.context);
    EXPECT_TRUE(callerStack.restore());
}

// A wait inside a CUDA graph capture starts no work. While a capture of a blocking stream
// holds the legacy default stream of the waiting thread's context, a wait without a deadline
// leaves the job it waits for to the worker, here held until another thread releases it, and
// the capture stays valid. Without a capture the same wait runs such a job itself.
TEST_F(WorkerLifecycle, WaitInsideACaptureLeavesTheJobToTheWorkerGpuTest)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    // The runtime's CUDA interface exists once an instance does.
    OwnedInstance instance;
    ASSERT_EQ(ovphysx_create_instance(&instance.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for a wait inside a capture";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load()) << "CUDA driver entry points are missing";
    PrimaryContext primary(driver);
    ASSERT_NE(primary.context, 0u);
    ovphysx::test_cuda::ScopedCudaContextPush push(cuda, primary.context);
    ASSERT_TRUE(push.ok());
    uintptr_t stream = 0;
    ASSERT_TRUE(cuda->streamCreate(&stream, 0u /* blocking */, nullptr));

    RanOn inCapture;
    void* graph = nullptr;
    {
        WorkerHold hold(instance.handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t task =
            ovphysx_submit_test_callback_job_for_test_internal(instance.handle, &RanOn::record, &inCapture);
        ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(driver.cuStreamBeginCapture(reinterpret_cast<void*>(stream), 0 /* global */), 0);
        std::thread releaser([&hold] {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            hold.release();
        });
        ovphysx_wait_task_result_t result{};
        const ovphysx_api_status_t waited =
            ovphysx_wait_task(instance.handle, task, OVPHYSX_TIMEOUT_INFINITE, &result).status;
        ovphysx_destroy_wait_task_result(&result);
        releaser.join();
        EXPECT_EQ(driver.cuStreamEndCapture(reinterpret_cast<void*>(stream), &graph), 0)
            << "the capture was invalidated";
        EXPECT_EQ(waited, OVPHYSX_API_SUCCESS);
    }
    EXPECT_TRUE(inCapture.ran.load());
    EXPECT_NE(inCapture.thread, std::this_thread::get_id()) << "the wait inside the capture ran the job";
    if (graph)
    {
        EXPECT_EQ(driver.cuGraphDestroy(graph), 0);
    }

    RanOn outside;
    {
        WorkerHold hold(instance.handle);
        ASSERT_TRUE(hold.held);
        EXPECT_TRUE(waitForTaskSuccess(instance.handle, ovphysx_submit_test_callback_job_for_test_internal(
                                                            instance.handle, &RanOn::record, &outside)));
    }
    EXPECT_EQ(outside.thread, std::this_thread::get_id()) << "without a capture the wait runs the job";
    EXPECT_TRUE(cuda->streamDestroy(stream, nullptr));
}

// The legacy-stream check does not see a capture of a non-blocking stream, so a wait inside
// one runs the unstarted job itself. It runs it in relaxed capture mode: a CUDA call that
// the capture's global mode refuses (here an allocation) succeeds and leaves the capture
// intact.
TEST_F(WorkerLifecycle, WaitInsideANonBlockingCaptureRunsTheJobRelaxedGpuTest)
{
    OwnedInstance instance;
    ASSERT_EQ(ovphysx_create_instance(&instance.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for a wait inside a capture";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load()) << "CUDA driver entry points are missing";
    PrimaryContext primary(driver);
    ASSERT_NE(primary.context, 0u);
    ovphysx::test_cuda::ScopedCudaContextPush push(cuda, primary.context);
    ASSERT_TRUE(push.ok());
    uintptr_t stream = 0;
    ASSERT_TRUE(cuda->streamCreate(&stream, 1u /* non-blocking */, nullptr));

    struct Allocation
    {
        omni::physx::IOptionalCuda* cuda = nullptr;
        std::thread::id thread;
        uintptr_t pointer = 0;
        int status = -1;
        static void run(void* user)
        {
            Allocation& self = *static_cast<Allocation*>(user);
            self.thread = std::this_thread::get_id();
            self.cuda->memAlloc(&self.pointer, 256, &self.status);
        }
    } allocation;
    allocation.cuda = cuda;
    void* graph = nullptr;
    {
        WorkerHold hold(instance.handle);
        ASSERT_TRUE(hold.held);
        const ovphysx_enqueue_result_t task =
            ovphysx_submit_test_callback_job_for_test_internal(instance.handle, &Allocation::run, &allocation);
        ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(driver.cuStreamBeginCapture(reinterpret_cast<void*>(stream), 0 /* global */), 0);
        EXPECT_TRUE(waitForTaskSuccess(instance.handle, task, OVPHYSX_TIMEOUT_INFINITE));
        EXPECT_EQ(driver.cuStreamEndCapture(reinterpret_cast<void*>(stream), &graph), 0)
            << "the capture was invalidated";
    }
    EXPECT_EQ(allocation.thread, std::this_thread::get_id()) << "the wait did not run the job";
    EXPECT_EQ(allocation.status, 0) << "the capture refused the job's allocation";
    if (allocation.pointer)
    {
        EXPECT_TRUE(cuda->memFree(allocation.pointer, nullptr));
    }
    if (graph)
    {
        EXPECT_EQ(driver.cuGraphDestroy(graph), 0);
    }
    EXPECT_TRUE(cuda->streamDestroy(stream, nullptr));
}

// A graph replay does not repeat ovphysx_update_from_ovstage, so a call inside a capture of a
// blocking stream is refused at submission, without a task, and the capture stays valid. The same
// call outside the capture is accepted; its task fails because no stage is attached.
TEST_F(WorkerLifecycle, UpdateFromOvstageInsideACaptureIsRefusedGpuTest)
{
    // The runtime's CUDA interface exists once an instance does.
    OwnedInstance instance;
    ASSERT_EQ(ovphysx_create_instance(&instance.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for an update inside a capture";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load()) << "CUDA driver entry points are missing";
    PrimaryContext primary(driver);
    ASSERT_NE(primary.context, 0u);
    ovphysx::test_cuda::ScopedCudaContextPush push(cuda, primary.context);
    ASSERT_TRUE(push.ok());
    uintptr_t stream = 0;
    ASSERT_TRUE(cuda->streamCreate(&stream, 0u /* blocking */, nullptr));

    ovstage_ordinal_range_t range{};
    range.end_ordinal = 1;
    void* graph = nullptr;
    ASSERT_EQ(driver.cuStreamBeginCapture(reinterpret_cast<void*>(stream), 0 /* global */), 0);
    const ovphysx_enqueue_result_t refused = ovphysx_update_from_ovstage(instance.handle, range);
    const ovphysx_string_t error = ovphysx_get_last_error();
    const std::string message = error.ptr ? std::string(error.ptr, error.length) : std::string();
    EXPECT_EQ(driver.cuStreamEndCapture(reinterpret_cast<void*>(stream), &graph), 0) << "the capture was invalidated";
    EXPECT_EQ(refused.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(refused.op_index, 0u);
    EXPECT_NE(message.find("CUDA graph capture"), std::string::npos) << message;
    if (graph)
    {
        EXPECT_EQ(driver.cuGraphDestroy(graph), 0);
    }

    const ovphysx_enqueue_result_t accepted = ovphysx_update_from_ovstage(instance.handle, range);
    ASSERT_EQ(accepted.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(instance.handle, accepted).status, OVPHYSX_API_ERROR);
    EXPECT_TRUE(cuda->streamDestroy(stream, nullptr));
}

// Fills every component of a write group's device tensors with xyz, row by row.
bool fillPositions(const ovstage_map_group_t& group, const double (&xyz)[3], uintptr_t context)
{
    for (uint32_t index = 0; index < group.data.tensor_count; ++index)
    {
        const DLTensor& tensor = group.data.tensors[index];
        const size_t width = session_test::detail::componentBytes(tensor.dtype);
        const size_t components = session_test::detail::elementCount(tensor);
        std::vector<unsigned char> bytes(components * width);
        for (size_t i = 0; i < components; ++i)
            session_test::detail::encode(tensor.dtype, xyz[i % 3], bytes.data() + i * width);
        if (!session_test::detail::deviceUpload(tensor, bytes, context))
            return false;
    }
    return true;
}

// Every body of the instance at xyz.
::testing::AssertionResult bodiesAt(ovphysx_instance_handle_t handle, const double (&xyz)[3], uintptr_t context)
{
    session_test::Column positions;
    if (!session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, positions, nullptr,
                                  context))
        return ::testing::AssertionFailure() << "the position read failed";
    if (positions.empty())
        return ::testing::AssertionFailure() << "no rigid bodies";
    for (const std::pair<const std::string, std::vector<double>>& body : positions)
        for (size_t axis = 0; axis < 3; ++axis)
            if (body.second.size() != 3 || std::abs(body.second[axis] - xyz[axis]) > 1e-3)
                return ::testing::AssertionFailure() << body.first << " axis " << axis << " is not at " << xyz[axis];
    return ::testing::AssertionSuccess();
}

// Two GPU instances share one pooled CUDA context. With every task of both collected, a
// stream-ordered commit of A is captured into a CUDA graph in global capture mode while B
// stays idle: neither instance's worker invalidates the capture, and each replay of the
// graph applies the group's contents at that time.
TEST_F(WorkerLifecycle, TwoInstancesOneContextCaptureGpuTest)
{
    // The runtime's CUDA interface exists once an instance does.
    OwnedInstance a;
    OwnedInstance b;
    ASSERT_EQ(ovphysx_create_instance(&a.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(&b.handle).status, OVPHYSX_API_SUCCESS);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    if (skipWithoutCuda(cuda))
    {
        if (ovphysxTestRequireCuda())
            FAIL() << "CUDA is required for the capture";
        GTEST_SKIP() << "CUDA is unavailable";
    }
    CudaDriver driver;
    ASSERT_TRUE(driver.load()) << "CUDA driver entry points are missing";
    // DirectGPU keeps write groups in device memory, which a stream-ordered commit consumes.
    BoolSettingScope directGpu("/physics/suppressReadback", true);
    ASSERT_TRUE(directGpu.applied);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(a.handle, kBoxesGpu, 1, "capture-a"));
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(b.handle, kBoxesGpu, 1, "capture-b"));
    ASSERT_EQ(ovphysx_warmup(a.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_warmup(b.handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t context = session_test::cudaContextOf(a.handle, kGpuScene, OVPHYSX_PHYSX_TYPE_SCENE);
    ASSERT_NE(context, 0u);
    ASSERT_EQ(session_test::cudaContextOf(b.handle, kGpuScene, OVPHYSX_PHYSX_TYPE_SCENE), context)
        << "both instances must share one CUDA context";

    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        a.handle, ovphysx_query(a.handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_token_or_string_t position = session_test::detail::attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t write = 0;
    ASSERT_TRUE(waitForTaskSuccess(a.handle, ovphysx_write(a.handle, query, &position, 1, &write, nullptr)));
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_TRUE(waitForTaskSuccess(a.handle, ovphysx_fetch_write_next(a.handle, write, &fetched)));
    ASSERT_NE(fetched.group, nullptr);
    const ovstage_map_group_t& group = *fetched.group;
    ASSERT_GT(group.data.tensor_count, 0u);
    ASSERT_EQ(group.data.tensors[0].device.device_type, kDLCUDA) << "DirectGPU write groups live on the device";

    uintptr_t stream = 0;
    {
        ovphysx::test_cuda::ScopedCudaContextPush push(cuda, context);
        ASSERT_TRUE(push.ok());
        ASSERT_TRUE(cuda->streamCreate(&stream, 1u /* non-blocking */, nullptr));
    }
    const double first[3] = { 5.0, 0.5, 7.0 };
    const double second[3] = { -3.0, 1.5, 9.0 };
    ASSERT_TRUE(fillPositions(group, first, context));
    // Every task of both instances is collected before the capture, as the capture
    // contract requires; B stays idle until it ends.
    EXPECT_TRUE(waitAllSucceeded(a.handle));
    EXPECT_TRUE(waitAllSucceeded(b.handle));

    void* graph = nullptr;
    void* exec = nullptr;
    ovphysx_enqueue_result_t commit{};
    {
        ovphysx::test_cuda::ScopedCudaContextPush push(cuda, context);
        ASSERT_TRUE(push.ok());
        ASSERT_EQ(driver.cuStreamBeginCapture(reinterpret_cast<void*>(stream), 0 /* global */), 0);
        commit = ovphysx_commit_group(a.handle, write, &group, ovstage_cuda_sync_t{ stream, 0 });
        const int ended = driver.cuStreamEndCapture(reinterpret_cast<void*>(stream), &graph);
        ASSERT_EQ(commit.status, OVPHYSX_API_SUCCESS)
            << std::string(ovphysx_get_last_error().ptr, ovphysx_get_last_error().length);
        ASSERT_EQ(ended, 0) << "the capture was invalidated";
        ASSERT_NE(graph, nullptr);
        ASSERT_EQ(driver.cuGraphInstantiateWithFlags(&exec, graph, 0), 0);
    }
    EXPECT_TRUE(waitForTaskSuccess(a.handle, commit));

    for (const double (*values)[3] : { &first, &second })
    {
        ASSERT_TRUE(fillPositions(group, *values, context));
        {
            ovphysx::test_cuda::ScopedCudaContextPush push(cuda, context);
            ASSERT_TRUE(push.ok());
            ASSERT_EQ(driver.cuGraphLaunch(exec, reinterpret_cast<void*>(stream)), 0);
            ASSERT_TRUE(cuda->streamSynchronize(stream, nullptr));
        }
        EXPECT_TRUE(bodiesAt(a.handle, *values, context)) << "the replayed commit did not apply the group";
    }

    {
        ovphysx::test_cuda::ScopedCudaContextPush push(cuda, context);
        EXPECT_EQ(driver.cuGraphExecDestroy(exec), 0);
        EXPECT_EQ(driver.cuGraphDestroy(graph), 0);
        EXPECT_TRUE(cuda->streamDestroy(stream, nullptr));
    }
    EXPECT_TRUE(waitForTaskSuccess(a.handle, ovphysx_step(a.handle, kDt)));
    EXPECT_TRUE(waitForTaskSuccess(b.handle, ovphysx_step(b.handle, kDt)));
    EXPECT_TRUE(waitForTaskSuccess(a.handle, ovphysx_release_write(a.handle, write)));
    EXPECT_TRUE(waitForTaskSuccess(a.handle, ovphysx_release_query(a.handle, query)));
}

#endif // OVPHYSX_ENABLE_GPU_TESTS

} // namespace
