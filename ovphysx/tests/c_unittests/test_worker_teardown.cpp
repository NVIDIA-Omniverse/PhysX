// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-TEARDOWN-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-6 AC-7
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-11
 *
 * TEST-CAPI-TEARDOWN-001: destroy and shutdown stop the instance worker.
 */

// Destroy and shutdown of an instance whose worker has running and queued work. A
// retryable refusal leaves the queue as it was. Destroy waits only for the running job,
// cancels every queued job, joins the worker and tears the attachment down under the
// lifecycle lock; it also completes from a Carbonite task while lane waiters fill the
// pool. Shutdown does the same for the worker of every live instance. Neither runs on an
// instance's own worker thread.

#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_config.h>
#include <ovphysx_companion/companion.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace
{

constexpr const char* kBoxes = "tests/data/boxes_falling_on_groundplane.usda";
constexpr float kDt = 1.0f / 60.0f;

ovphysx_task_status_t statusOf(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_NOT_FOUND;
    EXPECT_EQ(ovphysx_query_task_status(handle, task, &status).status, OVPHYSX_API_SUCCESS);
    return status;
}

std::string lastError()
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    return std::string(error.ptr ? error.ptr : "", error.length);
}

bool registered(ovphysx_instance_handle_t handle)
{
    bool attached = false;
    return ovphysx_is_stage_attached(handle, nullptr, &attached).status == OVPHYSX_API_SUCCESS;
}

// A job body that reports it started, then blocks until the test releases it (at most
// ten seconds, so a failing test cannot wedge the worker). Declare it before the instance
// that runs it: its destructor releases the job and waits for it to return.
struct BlockingJob
{
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool released = false;
    bool done = false;
    std::atomic<int64_t> finishedNs{0};

    ~BlockingJob()
    {
        std::unique_lock<std::mutex> lock(mutex);
        released = true;
        condition.notify_all();
        if (entered)
            condition.wait_for(lock, std::chrono::seconds(15), [this] { return done; });
    }
    static void run(void* user)
    {
        BlockingJob& job = *static_cast<BlockingJob*>(user);
        std::unique_lock<std::mutex> lock(job.mutex);
        job.entered = true;
        job.condition.notify_all();
        job.condition.wait_for(lock, std::chrono::seconds(10), [&job] { return job.released; });
        job.finishedNs = std::chrono::steady_clock::now().time_since_epoch().count();
        job.done = true;
        job.condition.notify_all();
    }
    bool waitEntered()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, std::chrono::seconds(10), [this] { return entered; });
    }
    void release()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            released = true;
        }
        condition.notify_all();
    }
};

void markRan(void* user)
{
    static_cast<std::atomic<bool>*>(user)->store(true);
}

// An instance the test owns. Unless the test destroyed it, it is destroyed at scope exit
// with its worker released, so no held worker outlives a failed test: a later call that
// waits for every instance would wait for it forever.
struct OwnedInstance
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ~OwnedInstance()
    {
        if (handle == OVPHYSX_INVALID_HANDLE)
            return;
        ovphysx_set_worker_hold_for_test_internal(handle, false);
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    }
    // The test destroyed the instance: free the Stages the test registry holds for it.
    void destroyed()
    {
        test_utils::destroy_ovstage_test_stages(handle);
        handle = OVPHYSX_INVALID_HANDLE;
    }
};

// A Stage populated from a USD file and sealed at ordinal 1, for a test that submits its
// attach itself. Declare it before the instances it is attached to.
class SealedStage
{
public:
    explicit SealedStage(const char* name)
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
        const ovx_string_t path{ kBoxes, std::strlen(kBoxes) };
        const ovstage_population_enqueue_result_t populate =
            ovphysx_sample_open_usd_from_file(m_stage, path, 1, 0.0, /*with_rendering=*/true);
        ovstage_population_op_wait_result_t populated{};
        bool ok = populate.status == OVSTAGE_OK &&
                  ovstage_population_wait_op(m_stage, populate.op_index, OVSTAGE_TIMEOUT_INFINITE, &populated) ==
                      OVSTAGE_OK;
        ovstage_write_floor_desc_t floorDesc{};
        floorDesc.ordinal = 1;
        floorDesc.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t floor =
            ok ? ovstage_advance_write_floor(m_stage, &floorDesc) : ovstage_enqueue_result_t{};
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
        if (m_stage)
            ovstage_destroy_instance(m_stage);
    }
    SealedStage(const SealedStage&) = delete;
    SealedStage& operator=(const SealedStage&) = delete;

    ovstage_instance_t* get() const { return m_stage; }
    bool valid() const { return m_stage != nullptr; }

private:
    ovstage_instance_t* m_stage = nullptr;
};

// Holds the process-wide lifecycle lock on a helper thread until released.
class LifecycleLockHolder
{
public:
    LifecycleLockHolder()
        : m_thread([this] { ovphysx_with_lifecycle_lock_for_test_internal(&LifecycleLockHolder::hold, this); })
    {
    }
    ~LifecycleLockHolder()
    {
        release();
        m_thread.join();
    }
    bool waitHeld()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_condition.wait_for(lock, std::chrono::seconds(10), [this] { return m_held; });
    }
    void release()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_released = true;
        }
        m_condition.notify_all();
    }

private:
    static void hold(void* user)
    {
        LifecycleLockHolder& self = *static_cast<LifecycleLockHolder*>(user);
        std::unique_lock<std::mutex> lock(self.m_mutex);
        self.m_held = true;
        self.m_condition.notify_all();
        self.m_condition.wait_for(lock, std::chrono::seconds(10), [&self] { return self.m_released; });
    }

    std::mutex m_mutex;
    std::condition_variable m_condition;
    bool m_held = false;
    bool m_released = false;
    std::thread m_thread;
};

// What a job on the instance's own worker observed when it called destroy and shutdown.
struct OwnWorkerProbe
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    bool callShutdown = false;
    ovphysx_api_status_t destroy = OVPHYSX_API_SUCCESS;
    std::string destroyError;
    ovphysx_api_status_t shutdown = OVPHYSX_API_SUCCESS;
    std::string shutdownError;

    static void run(void* user)
    {
        OwnWorkerProbe& probe = *static_cast<OwnWorkerProbe*>(user);
        probe.destroy = ovphysx_destroy_instance(probe.handle).status;
        probe.destroyError = lastError();
        if (probe.callShutdown)
        {
            probe.shutdown = ovphysx_shutdown().status;
            probe.shutdownError = lastError();
        }
    }
};

} // namespace

// A refusal before any wait leaves running and queued work as it was: the running job
// finishes and the queued jobs run once the worker continues.
TEST(WorkerTeardown, RetryableDestroyKeepsQueuedWork)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    BlockingJob running;
    std::atomic<bool> secondRan{false};
    std::atomic<bool> thirdRan{false};
    OwnedInstance owner;
    ASSERT_EQ(ovphysx_create_instance(&owner.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(owner.handle, kBoxes, 1, "worker-teardown-retry"));

    ovphysx_companion_api_t companion{};
    ASSERT_EQ(ovphysx_companion_acquire(OVPHYSX_COMPANION_ABI_VERSION, sizeof(companion), &companion),
              OVPHYSX_COMPANION_OK);
    uint64_t session = 0;
    ASSERT_EQ(companion.open_session(owner.handle, &session), OVPHYSX_COMPANION_OK);
    struct CloseSession
    {
        ovphysx_companion_api_t& api;
        uint64_t session;
        ~CloseSession() { EXPECT_EQ(api.close_session(session), OVPHYSX_COMPANION_OK); }
    } closeSession{ companion, session };

    const ovphysx_enqueue_result_t first =
        ovphysx_submit_test_callback_job_for_test_internal(owner.handle, &BlockingJob::run, &running);
    ASSERT_EQ(first.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(running.waitEntered());
    const ovphysx_enqueue_result_t second =
        ovphysx_submit_test_callback_job_for_test_internal(owner.handle, &markRan, &secondRan);
    const ovphysx_enqueue_result_t third =
        ovphysx_submit_test_callback_job_for_test_internal(owner.handle, &markRan, &thirdRan);
    ASSERT_EQ(second.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(third.status, OVPHYSX_API_SUCCESS);

    // The companion lease refuses destruction while the first job still runs.
    EXPECT_EQ(ovphysx_destroy_instance(owner.handle).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_NE(lastError().find("close companion sessions"), std::string::npos) << lastError();
    EXPECT_EQ(statusOf(owner.handle, first), OVPHYSX_TASK_STATUS_RUNNING);
    EXPECT_EQ(statusOf(owner.handle, second), OVPHYSX_TASK_STATUS_PENDING);
    EXPECT_EQ(statusOf(owner.handle, third), OVPHYSX_TASK_STATUS_PENDING);

    running.release();
    EXPECT_TRUE(waitForTaskSuccess(owner.handle, first));
    EXPECT_TRUE(waitForTaskSuccess(owner.handle, second));
    EXPECT_TRUE(waitForTaskSuccess(owner.handle, third));
    EXPECT_TRUE(secondRan.load());
    EXPECT_TRUE(thirdRan.load());
    bool attached = false;
    EXPECT_EQ(ovphysx_is_stage_attached(owner.handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attached) << "a refused destroy keeps the attachment";
}

// Queued attach and clone jobs never start: destroy cancels them with everything queued
// behind them, and their receipts are gone with the instance.
TEST(WorkerTeardown, DestroyCancelsQueuedLifecycleJobs)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    SealedStage stage("worker-teardown-cancel");
    ASSERT_TRUE(stage.valid());
    std::atomic<bool> probeRan{false};
    OwnedInstance victim;
    ASSERT_EQ(ovphysx_create_instance(&victim.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(victim.handle, true));

    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(victim.handle, stage.get(), 1);
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/Cube1Copy");
    const ovphysx_enqueue_result_t clone =
        ovphysx_clone(victim.handle, OVPHYSX_LITERAL("/World/Cube1"), &target, 1, nullptr, nullptr);
    const ovphysx_enqueue_result_t probe =
        ovphysx_submit_test_callback_job_for_test_internal(victim.handle, &markRan, &probeRan);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(clone.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(probe.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(victim.handle, attach), OVPHYSX_TASK_STATUS_PENDING);
    EXPECT_EQ(statusOf(victim.handle, clone), OVPHYSX_TASK_STATUS_PENDING);

    // Destroy needs no progress from the held worker: it runs nothing more.
    ASSERT_EQ(ovphysx_destroy_instance(victim.handle).status, OVPHYSX_API_SUCCESS);
    const ovphysx_instance_handle_t gone = victim.handle;
    victim.destroyed();
    EXPECT_FALSE(probeRan.load()) << "queued jobs are cancelled, not run";
    for (const ovphysx_enqueue_result_t& task : { attach, clone, probe })
    {
        ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_PENDING;
        EXPECT_EQ(ovphysx_query_task_status(gone, task, &status).status, OVPHYSX_API_NOT_FOUND);
        EXPECT_EQ(status, OVPHYSX_TASK_STATUS_NOT_FOUND);
        ovphysx_wait_task_result_t result{};
        EXPECT_EQ(ovphysx_wait_task(gone, task, OVPHYSX_TIMEOUT_INFINITE, &result).status, OVPHYSX_API_NOT_FOUND);
        ovphysx_destroy_wait_task_result(&result);
    }

    // The Stage the cancelled attach named is free for another instance.
    OwnedInstance peer;
    ASSERT_EQ(ovphysx_create_instance(&peer.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(peer.handle, ovphysx_attach_ovstage(peer.handle, stage.get(), 1)));
    EXPECT_TRUE(waitForTaskSuccess(peer.handle, ovphysx_detach_ovstage(peer.handle)));
}

// Destroy waits for the job that is running, and only for it: the jobs queued behind it
// never run.
TEST(WorkerTeardown, DestroyWaitsForRunningJobOnly)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    BlockingJob running;
    std::atomic<bool> ran[3] = {};
    OwnedInstance victim;
    ASSERT_EQ(ovphysx_create_instance(&victim.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_submit_test_callback_job_for_test_internal(victim.handle, &BlockingJob::run, &running).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(running.waitEntered());
    for (std::atomic<bool>& flag : ran)
        ASSERT_EQ(ovphysx_submit_test_callback_job_for_test_internal(victim.handle, &markRan, &flag).status,
                  OVPHYSX_API_SUCCESS);

    // The running job ends 300 ms after destroy starts waiting for it.
    std::thread releaser([&running] {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        running.release();
    });
    const ovphysx_api_status_t destroyStatus = ovphysx_destroy_instance(victim.handle).status;
    const int64_t returnedNs = std::chrono::steady_clock::now().time_since_epoch().count();
    releaser.join();
    EXPECT_EQ(destroyStatus, OVPHYSX_API_SUCCESS);
    if (destroyStatus == OVPHYSX_API_SUCCESS)
        victim.destroyed();
    EXPECT_NE(running.finishedNs.load(), 0) << "the running job did not finish";
    EXPECT_GE(returnedNs, running.finishedNs.load()) << "destroy returned before the running job finished";
    for (const std::atomic<bool>& flag : ran)
        EXPECT_FALSE(flag.load()) << "a queued job ran";
}

// Output cleanup spans several fixed-size batches without allocating after destroy commits.
TEST(WorkerTeardown, DestroyCleansMoreThanOneBatchOfOutputHandles)
{
    constexpr size_t kHandleCount = 40;
    OwnedInstance victim;
    ASSERT_EQ(ovphysx_create_instance(&victim.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(victim.handle, kBoxes));

    std::array<ovphysx_query_handle_t, kHandleCount> queries{};
    for (ovphysx_query_handle_t& query : queries)
    {
        ASSERT_TRUE(waitForTaskSuccess(
            victim.handle,
            ovphysx_query(victim.handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
        ASSERT_NE(query, 0u);
    }
    ASSERT_EQ(ovphysx_destroy_instance(victim.handle).status, OVPHYSX_API_SUCCESS);
    victim.destroyed();
}

// Destroy tears its attachment down under the lifecycle lock that attach and detach jobs
// take, so it never overlaps another instance's lifecycle job: while the lock is held,
// both wait, and both complete once it is released.
TEST(WorkerTeardown, DestroyTakesTheLifecycleLock)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    SealedStage stage("worker-teardown-lifecycle-b");
    ASSERT_TRUE(stage.valid());
    OwnedInstance victim;
    ASSERT_EQ(ovphysx_create_instance(&victim.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(victim.handle, kBoxes, 1, "worker-teardown-lifecycle-a"));
    OwnedInstance peer;
    ASSERT_EQ(ovphysx_create_instance(&peer.handle).status, OVPHYSX_API_SUCCESS);

    ovphysx_enqueue_result_t attach{};
    std::atomic<bool> destroyed{false};
    ovphysx_api_status_t destroyStatus = OVPHYSX_API_ERROR;
    {
        LifecycleLockHolder lock;
        ASSERT_TRUE(lock.waitHeld());
        attach = ovphysx_attach_ovstage(peer.handle, stage.get(), 1);
        EXPECT_EQ(attach.status, OVPHYSX_API_SUCCESS);
        std::thread destroyer([&] {
            destroyStatus = ovphysx_destroy_instance(victim.handle).status;
            destroyed = true;
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        EXPECT_FALSE(destroyed.load()) << "destroy tore the attachment down without the lifecycle lock";
        if (attach.status == OVPHYSX_API_SUCCESS)
        {
            EXPECT_EQ(statusOf(peer.handle, attach), OVPHYSX_TASK_STATUS_RUNNING)
                << "the attach job waits for the lifecycle lock";
        }
        lock.release();
        destroyer.join();
    }
    EXPECT_EQ(destroyStatus, OVPHYSX_API_SUCCESS);
    if (destroyStatus == OVPHYSX_API_SUCCESS)
        victim.destroyed();
    if (attach.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_TRUE(waitForTaskSuccess(peer.handle, attach));
        EXPECT_TRUE(waitForTaskSuccess(peer.handle, ovphysx_detach_ovstage(peer.handle)));
    }
}

// Destroy from a job on the instance's own worker would wait for that worker; it is
// refused and changes nothing. (Shutdown is refused there too; see the isolated case below.)
TEST(WorkerTeardown, DestroyFromOwnWorkerCallbackRefused)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnWorkerProbe probe;
    OwnedInstance owner;
    ASSERT_EQ(ovphysx_create_instance(&owner.handle).status, OVPHYSX_API_SUCCESS);
    probe.handle = owner.handle;
    EXPECT_TRUE(waitForTaskSuccess(
        owner.handle, ovphysx_submit_test_callback_job_for_test_internal(owner.handle, &OwnWorkerProbe::run, &probe),
        kWorkerRunsIt));
    EXPECT_EQ(probe.destroy, OVPHYSX_API_INVALID_STATE);
    EXPECT_NE(probe.destroyError.find("own worker thread"), std::string::npos) << probe.destroyError;
    EXPECT_TRUE(waitForTaskSuccess(owner.handle, ovphysx_submit_test_job_for_test_internal(owner.handle, nullptr, 0)))
        << "the refused destroy changed the instance";
}

// More Carbonite tasks than pool threads wait for a busy lane in a synchronous call, and
// one more task destroys another instance meanwhile: waiting tasks release their threads,
// so the destroying task runs and finishes, and every call succeeds once the lane frees.
TEST(WorkerTeardown, SaturatedPoolLaneAndDestroy)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnedInstance owner;
    ASSERT_EQ(ovphysx_create_instance(&owner.handle).status, OVPHYSX_API_SUCCESS);
    OwnedInstance victim;
    ASSERT_EQ(ovphysx_create_instance(&victim.handle).status, OVPHYSX_API_SUCCESS);
    uint32_t waiters = 0;
    EXPECT_TRUE(ovphysx_lane_waits_and_destroy_from_saturated_carbonite_pool_for_test_internal(
        owner.handle, victim.handle, &waiters));
    EXPECT_GT(waiters, 2u);
    EXPECT_FALSE(registered(victim.handle)) << "the destroying task did not destroy the instance";
    if (!registered(victim.handle))
        victim.destroyed();
}

// Runs in its own process (scripts/test_cpp.cmake), which owns initialize and shutdown.
// Shutdown on an instance's worker is refused. Otherwise, with live instances, shutdown
// stops their workers before it returns: every queued job, a step included, is cancelled
// and each worker is joined. The instances stay registered: their receipts keep their
// outcomes, new jobs are rejected, and destruction still works, also for an instance
// without a worker thread. The others have one in every test execution mode.
TEST(WorkerTeardown, ShutdownJoinsWorkersWithLiveInstances)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t idle = OVPHYSX_INVALID_HANDLE;
    ovphysx_instance_handle_t stepping = OVPHYSX_INVALID_HANDLE;
    ovphysx_instance_handle_t withoutWorker = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(createInstanceWithWorkerThread(true, &idle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(createInstanceWithWorkerThread(true, &stepping).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(createInstanceWithWorkerThread(false, &withoutWorker).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t finished = ovphysx_submit_test_job_for_test_internal(withoutWorker, nullptr, 0);
    ASSERT_EQ(finished.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(stepping, kBoxes, 1, "worker-teardown-shutdown"));
    ASSERT_EQ(ovphysx_step_sync(stepping, kDt).status, OVPHYSX_API_SUCCESS);

    OwnWorkerProbe probe;
    probe.handle = idle;
    probe.callShutdown = true;
    EXPECT_TRUE(waitForTaskSuccess(
        idle, ovphysx_submit_test_callback_job_for_test_internal(idle, &OwnWorkerProbe::run, &probe), kWorkerRunsIt));
    EXPECT_EQ(probe.destroy, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.shutdown, OVPHYSX_API_INVALID_STATE);
    EXPECT_NE(probe.shutdownError.find("worker thread"), std::string::npos) << probe.shutdownError;

    std::atomic<bool> ran[2] = {};
    ovphysx_enqueue_result_t queued[2] = {};
    ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(idle, true));
    for (int i = 0; i < 2; ++i)
    {
        queued[i] = ovphysx_submit_test_callback_job_for_test_internal(idle, &markRan, &ran[i]);
        ASSERT_EQ(queued[i].status, OVPHYSX_API_SUCCESS);
    }
    ASSERT_TRUE(ovphysx_hold_worker_after_next_submit_for_test_internal(stepping));
    const ovphysx_enqueue_result_t step = ovphysx_step(stepping, kDt);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statusOf(stepping, step), OVPHYSX_TASK_STATUS_PENDING);

    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    bool allExited = true;
    for (const ovphysx_instance_handle_t handle : { idle, stepping })
    {
        bool exited = false;
        EXPECT_TRUE(ovphysx_worker_exited_for_test_internal(handle, &exited));
        EXPECT_TRUE(exited) << "shutdown returned with a running worker";
        allExited = allExited && exited;
    }
    // A worker that is still held would leave the waits below waiting forever.
    ASSERT_TRUE(allExited);
    EXPECT_TRUE(waitForTaskFailure(stepping, step, "ovphysx_shutdown stopped the instance worker"));
    for (int i = 0; i < 2; ++i)
    {
        EXPECT_FALSE(ran[i].load());
        EXPECT_TRUE(waitForTaskFailure(idle, queued[i], "ovphysx_shutdown stopped the instance worker"));
    }
    for (const ovphysx_instance_handle_t handle : { idle, withoutWorker })
    {
        const ovphysx_enqueue_result_t late = ovphysx_submit_test_job_for_test_internal(handle, nullptr, 0);
        EXPECT_EQ(late.status, OVPHYSX_API_INVALID_STATE);
        EXPECT_EQ(late.op_index, 0u);
    }
    EXPECT_TRUE(waitForTaskSuccess(withoutWorker, finished, OVPHYSX_TIMEOUT_POLL));
    // A short call is refused at submission too, with the same status, whether it would run
    // on the calling thread or queue: the instance's worker stopped.
    for (const bool busy : { false, true })
    {
        if (busy)
        {
            ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(stepping, true));
        }
        ovphysx_query_handle_t lateQuery = 1;
        const ovphysx_enqueue_result_t lateShort =
            ovphysx_query(stepping, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &lateQuery, nullptr);
        EXPECT_EQ(lateShort.status, OVPHYSX_API_INVALID_STATE) << (busy ? "busy" : "idle");
        EXPECT_EQ(lateShort.op_index, 0u);
        EXPECT_EQ(lateQuery, 0u);
        if (busy)
        {
            ASSERT_TRUE(ovphysx_set_worker_hold_for_test_internal(stepping, false));
        }
    }
    // So is a legacy binding call, which takes the lane: no engine code runs after shutdown.
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t binding = 0;
    EXPECT_EQ(ovphysx_create_tensor_binding(stepping, &desc, &binding).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(binding, 0u);

    EXPECT_EQ(ovphysx_destroy_instance(stepping).status, OVPHYSX_API_SUCCESS);
    test_utils::destroy_ovstage_test_stages(stepping);
    EXPECT_EQ(ovphysx_destroy_instance(idle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_instance(withoutWorker).status, OVPHYSX_API_SUCCESS);
    // Balance the runtime reference those destroys released.
    EXPECT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}
