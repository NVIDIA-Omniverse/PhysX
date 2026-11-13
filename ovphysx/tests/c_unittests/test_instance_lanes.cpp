// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LANE-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-9 AC-12
 *
 * TEST-CAPI-LANE-001 and TEST-CAPI-REENTRY-001: calls that run engine code on the
 * calling thread wait for work queued on the instance worker instead of being
 * rejected, and calls made inside callbacks are refused instead of blocking.
 */

// Lanes from the public API: a call that runs engine code on the calling thread
// waits for jobs queued on its instance's worker, a process-wide call waits for
// every instance's worker, and a call made inside a log or object-change callback
// that would wait is refused while other threads keep going.

#include "gated_simulation.h"
#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "ovphysx_test_utils.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr uint32_t kJobMs = 200;
constexpr const char* kBoxes = "tests/data/boxes_falling_on_groundplane.usda";

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
        if (held.exchange(false))
            ovphysx_set_worker_hold_for_test_internal(m_handle, false);
    }
    std::atomic<bool> held{false};

private:
    ovphysx_instance_handle_t m_handle;
};

ovphysx_task_status_t statusOf(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_task_status_t status = OVPHYSX_TASK_STATUS_NOT_FOUND;
    EXPECT_EQ(ovphysx_query_task_status(handle, task, &status).status, OVPHYSX_API_SUCCESS);
    return status;
}

void drainTasks(ovphysx_instance_handle_t handle)
{
    ovphysx_wait_task_result_t residual{};
    ovphysx_wait_all(handle, OVPHYSX_TIMEOUT_INFINITE, &residual);
    ovphysx_destroy_wait_task_result(&residual);
}

struct OwnedInstance
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ~OwnedInstance()
    {
        if (handle == OVPHYSX_INVALID_HANDLE)
            return;
        test_utils::destroy_ovstage_test_attachments(handle);
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
        test_utils::destroy_ovstage_test_stages(handle);
    }
};

// Holds jobOwner's worker, queues a sleeping job on it and makes call on a helper
// thread. The call must not return while the job is held, and the job must have
// succeeded when it returns: the call waited for the job instead of overtaking or
// rejecting it. call returns its status, which must be SUCCESS.
template <typename Call>
::testing::AssertionResult waitsForQueuedJob(ovphysx_instance_handle_t jobOwner, Call&& call)
{
    WorkerHold hold(jobOwner);
    if (!hold.held)
        return ::testing::AssertionFailure() << "the worker hold is unavailable";
    const ovphysx_enqueue_result_t job = ovphysx_submit_test_job_for_test_internal(jobOwner, nullptr, kJobMs);
    if (job.status != OVPHYSX_API_SUCCESS)
        return ::testing::AssertionFailure() << "the test job was rejected: " << job.status;

    std::atomic<bool> returned{false};
    ovphysx_api_status_t callStatus = OVPHYSX_API_ERROR;
    ovphysx_task_status_t jobAtReturn = OVPHYSX_TASK_STATUS_NOT_FOUND;
    std::thread caller([&] {
        callStatus = call();
        jobAtReturn = statusOf(jobOwner, job);
        returned = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const bool returnedWhileHeld = returned.load();
    hold.release();
    caller.join();
    const ::testing::AssertionResult collected = waitForTaskSuccess(jobOwner, job, OVPHYSX_TIMEOUT_INFINITE);

    if (returnedWhileHeld)
        return ::testing::AssertionFailure() << "the call returned before the queued job ran";
    if (jobAtReturn != OVPHYSX_TASK_STATUS_SUCCEEDED)
        return ::testing::AssertionFailure() << "the queued job was not finished when the call returned: "
                                             << jobAtReturn;
    if (callStatus != OVPHYSX_API_SUCCESS)
        return ::testing::AssertionFailure() << "the call failed with status " << callStatus;
    return collected;
}

// Whether flag turns true within 10 s.
bool turnsTrue(const std::atomic<bool>& flag)
{
    for (int i = 0; i < 10000 && !flag.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return flag.load();
}

// A peer instance whose detach job is held inside the runtime detach: the runtime delivers
// the peer's object-change callback there, on the peer's worker, and the callback blocks
// until release(). Meanwhile the job holds the peer's lane, the lifecycle lock and the lock
// that attach and detach take around the runtime.
class HeldDetach
{
public:
    HeldDetach()
    {
        EXPECT_EQ(ovphysx_create_instance(&m_peer.handle).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(test_utils::attach_usd_with_ovstage(m_peer.handle, kBoxes));
        EXPECT_EQ(ovphysx_step_sync(m_peer.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ovphysx_object_change_callbacks_t callbacks{};
        callbacks.user_data = this;
        callbacks.on_all_objects_destroyed = [](void* data) { static_cast<HeldDetach*>(data)->block(); };
        EXPECT_EQ(ovphysx_subscribe_object_changes(m_peer.handle, &callbacks, &m_subscription).status,
                  OVPHYSX_API_SUCCESS);
        m_detach = ovphysx_detach_ovstage(m_peer.handle);
        EXPECT_EQ(m_detach.status, OVPHYSX_API_SUCCESS);
        std::unique_lock<std::mutex> lock(m_mutex);
        m_held = m_changed.wait_for(lock, std::chrono::seconds(30), [this] { return m_entered; });
    }
    ~HeldDetach()
    {
        release();
        if (m_subscription != OVPHYSX_INVALID_SUBSCRIPTION_ID)
            ovphysx_unsubscribe_object_changes(m_peer.handle, m_subscription);
    }
    bool held() const { return m_held; }
    void release()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_released = true;
        }
        m_changed.notify_all();
        if (m_detach.op_index)
        {
            EXPECT_TRUE(waitForTaskSuccess(m_peer.handle, m_detach, OVPHYSX_TIMEOUT_INFINITE));
            m_detach = {};
        }
    }

private:
    void block()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_entered = true;
        m_changed.notify_all();
        m_changed.wait(lock, [this] { return m_released; });
    }

    OwnedInstance m_peer;
    ovphysx_subscription_id_t m_subscription = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    ovphysx_enqueue_result_t m_detach{};
    std::mutex m_mutex;
    std::condition_variable m_changed;
    bool m_entered = false;
    bool m_released = false;
    bool m_held = false;
};

class Lanes : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        // The CPU instance is shared across suites; start from an empty task table.
        drainTasks(m_handle);
    }

    void loadScene(ovphysx_instance_handle_t handle)
    {
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, kBoxes));
        ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    }

    void openQuery(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, ovphysx_query_handle_t& query)
    {
        ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)));
        ASSERT_NE(query, 0u);
    }
};

TEST_F(Lanes, ShortCallWaitsBehindWorkerJob)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene(m_handle);

    // A session call that runs on the calling thread, the articulation update, waits;
    // short async calls such as ovphysx_query are queued instead (WorkerShortCalls).
    ovphysx_query_handle_t articulations = 0;
    openQuery(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, articulations);
    ovphysx_enqueue_result_t update{};
    EXPECT_TRUE(waitsForQueuedJob(m_handle, [&] {
        update = ovphysx_update_articulations_kinematic(m_handle, articulations,
                                                        OVPHYSX_ARTICULATION_KINEMATIC_POSITION,
                                                        ovstage_cuda_sync_t{0, 0});
        return update.status;
    }));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, update));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, articulations)));

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube1");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t binding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);
    DLTensor* pose = test_utils::make_float32_tensor(std::vector<float>(7, 0.0f), {1, 7});
    EXPECT_TRUE(waitsForQueuedJob(m_handle, [&] { return ovphysx_read_tensor_binding(m_handle, binding, pose).status; }));
    test_utils::free_tensor(pose);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);

    EXPECT_TRUE(waitsForQueuedJob(m_handle, [&] { return ovphysx_step_sync(m_handle, 1.0f / 60.0f).status; }));

    ovphysx_object_change_callbacks_t callbacks{};
    callbacks.on_object_created = [](ovphysx_string_t, ovphysx_physx_type_t, void*) {};
    ovphysx_subscription_id_t subscription = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    EXPECT_TRUE(waitsForQueuedJob(m_handle, [&] {
        return ovphysx_subscribe_object_changes(m_handle, &callbacks, &subscription).status;
    }));
    EXPECT_EQ(ovphysx_unsubscribe_object_changes(m_handle, subscription).status, OVPHYSX_API_SUCCESS);

    // Instance configuration changes only before attach, so use a fresh instance.
    OwnedInstance detached;
    ASSERT_EQ(ovphysx_create_instance(&detached.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitsForQueuedJob(detached.handle, [&] {
        return ovphysx_set_config(detached.handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
                                  ovphysx_config_value_string(OVPHYSX_LITERAL(""))).status;
    }));
}

TEST_F(Lanes, OutputNotRejectedWhileWorkerRuns)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene(m_handle);
    physx::PxScene* const scene = static_cast<physx::PxScene*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene"));
    physx::PxRigidDynamic* const actor = static_cast<physx::PxRigidDynamic*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(scene, nullptr);
    ASSERT_NE(actor, nullptr);
    ovphysx_query_handle_t articulations = 0;
    openQuery(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, articulations);
    // A real step job, held inside its simulation, so the runtime is executing for
    // this attachment when the output call arrives.
    test_utils::GatedSimulationScope gate(m_handle, scene, actor);
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
    hold.release();
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    gate.set_pending_task(step);
    gate.start_watchdog();
    ASSERT_TRUE(gate.wait_until_entered()) << "the step job never reached its simulation";
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_RUNNING);

    std::atomic<bool> opened{false};
    std::thread opener([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        opened = true;
        gate.open_and_join_watchdog();
    });
    // A short async call is queued behind the running step, not rejected busy.
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t created =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr);
    const bool queryReturnedAfterStep = opened.load();
    EXPECT_EQ(created.status, OVPHYSX_API_SUCCESS) << "the output call must be queued, not report the runtime busy";
    EXPECT_NE(created.op_index, 0u);
    // A session call that runs on the calling thread waits for the step instead; the
    // runtime-busy probe after its lane is then clear.
    const ovphysx_enqueue_result_t updated = ovphysx_update_articulations_kinematic(
        m_handle, articulations, OVPHYSX_ARTICULATION_KINEMATIC_POSITION, ovstage_cuda_sync_t{0, 0});
    const bool returnedAfterStep = opened.load();
    opener.join();

    EXPECT_FALSE(queryReturnedAfterStep) << "the short call waited for the step job";
    EXPECT_TRUE(returnedAfterStep) << "the articulation update ran before the step job finished";
    EXPECT_EQ(updated.status, OVPHYSX_API_SUCCESS) << "the output call must wait, not report the runtime busy";
    EXPECT_NE(updated.op_index, 0u);
    EXPECT_EQ(statusOf(m_handle, step), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_EQ(statusOf(m_handle, created), OVPHYSX_TASK_STATUS_SUCCEEDED);
    EXPECT_EQ(gate.finish(), OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, created));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, updated));
    ASSERT_NE(query, 0u);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, articulations)));
}

// Submitting an attach or detach never waits for another instance's attach or detach that
// is running in the runtime.
TEST_F(Lanes, LifecycleSubmissionWaitsForNoPeerLifecycleJob)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnedInstance other;
    ASSERT_EQ(ovphysx_create_instance(&other.handle).status, OVPHYSX_API_SUCCESS);
    HeldDetach held;
    ASSERT_TRUE(held.held()) << "the peer's detach never reached the runtime";

    std::atomic<bool> returned{false};
    ovphysx_enqueue_result_t detach{};
    std::thread caller([&] {
        detach = ovphysx_detach_ovstage(other.handle);
        returned = true;
    });
    EXPECT_TRUE(turnsTrue(returned)) << "the detach submission waited for the peer's running detach";
    held.release();
    caller.join();
    EXPECT_TRUE(waitForTaskSuccess(other.handle, detach, OVPHYSX_TIMEOUT_INFINITE));
}

// An instance with no attachment has no runtime work of its own to reenter: its output calls
// run while another instance's detach holds the runtime's admission for calls on no
// attachment.
TEST_F(Lanes, UnattachedInstanceOutputRunsWhileAPeerDetaches)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnedInstance idle;
    ASSERT_EQ(ovphysx_create_instance(&idle.handle).status, OVPHYSX_API_SUCCESS);
    HeldDetach held;
    ASSERT_TRUE(held.held()) << "the peer's detach never reached the runtime";

    std::atomic<bool> returned{false};
    ovphysx_enqueue_result_t opened{};
    ovphysx_enqueue_result_t released{};
    std::thread caller([&] {
        ovphysx_query_handle_t query = 0;
        opened = ovphysx_query(idle.handle, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr);
        EXPECT_TRUE(waitForTaskSuccess(idle.handle, opened));
        ovphysx_read_handle_t read = 0;
        EXPECT_TRUE(waitForTaskSuccess(idle.handle,
            ovphysx_read(idle.handle, query, &OVPHYSX_NAME_DEBUG_SCALE, 1,
                         ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
        const ovstage_read_group_t* group = nullptr;
        EXPECT_TRUE(waitForTaskSuccess(idle.handle, ovphysx_fetch_read_next(idle.handle, read, &group)));
        EXPECT_NE(group, nullptr);
        if (group)
        {
            EXPECT_TRUE(waitForTaskSuccess(idle.handle,
                ovphysx_release_group(idle.handle, read, group->read_group_id)));
        }
        EXPECT_TRUE(waitForTaskSuccess(idle.handle, ovphysx_release_read(idle.handle, read)));

        ovphysx_write_handle_t write = 0;
        EXPECT_TRUE(waitForTaskSuccess(idle.handle,
            ovphysx_write(idle.handle, query, &OVPHYSX_NAME_DEBUG_SCALE, 1, &write, nullptr)));
        ovphysx_fetched_write_group_t written{};
        EXPECT_TRUE(waitForTaskSuccess(idle.handle, ovphysx_fetch_write_next(idle.handle, write, &written)));
        EXPECT_NE(written.group, nullptr);
        if (written.group)
        {
            EXPECT_EQ(written.group->data.tensor_count, 1u);
            if (written.group->data.tensor_count == 1)
            {
                EXPECT_NE(written.group->data.tensors[0].data, nullptr);
                if (written.group->data.tensors[0].data)
                {
                    *static_cast<float*>(written.group->data.tensors[0].data) = 0.25f;
                    EXPECT_TRUE(waitForTaskSuccess(idle.handle,
                        ovphysx_commit_group(idle.handle, write, written.group, ovstage_cuda_sync_t{})));
                }
            }
        }
        EXPECT_TRUE(waitForTaskSuccess(idle.handle, ovphysx_release_write(idle.handle, write)));
        released = ovphysx_release_query(idle.handle, query);
        returned = true;
    });
    EXPECT_TRUE(turnsTrue(returned)) << "the output calls waited for the peer's detach";
    held.release();
    caller.join();
    EXPECT_TRUE(waitForTaskSuccess(idle.handle, released));
}

// A debug read opened without a stage remains a host snapshot after its instance
// attaches. Fetch/release must still progress while a different instance detaches.
TEST_F(Lanes, UnattachedReadSnapshotProgressesAfterAttach)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnedInstance other;
    ASSERT_EQ(ovphysx_create_instance(&other.handle).status, OVPHYSX_API_SUCCESS);
    ovphysx_query_handle_t query = 0;
    openQuery(other.handle, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, query);
    ovphysx_read_handle_t read = 0;
    ASSERT_TRUE(waitForTaskSuccess(other.handle,
        ovphysx_read(other.handle, query, &OVPHYSX_NAME_DEBUG_SCALE, 1,
                     ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
    loadScene(other.handle);
    HeldDetach held;
    ASSERT_TRUE(held.held()) << "the peer's detach never reached the runtime";

    std::atomic<bool> returned{false};
    std::thread caller([&] {
        const ovstage_read_group_t* group = nullptr;
        EXPECT_TRUE(waitForTaskSuccess(other.handle, ovphysx_fetch_read_next(other.handle, read, &group)));
        EXPECT_NE(group, nullptr);
        if (group)
        {
            EXPECT_TRUE(waitForTaskSuccess(other.handle,
                ovphysx_release_group(other.handle, read, group->read_group_id)));
        }
        EXPECT_TRUE(waitForTaskSuccess(other.handle, ovphysx_release_read(other.handle, read)));
        returned = true;
    });
    EXPECT_TRUE(turnsTrue(returned)) << "the retained snapshot waited for the peer's detach";
    held.release();
    caller.join();
    EXPECT_TRUE(waitForTaskSuccess(other.handle, ovphysx_release_query(other.handle, query)));
}

TEST_F(Lanes, ProcessWideWaitsForPeerWorker)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    OwnedInstance peer;
    ASSERT_EQ(ovphysx_create_instance(&peer.handle).status, OVPHYSX_API_SUCCESS);

    // Rewrite the current value, so the shared process configuration is unchanged.
    ovphysx_config_value_t current{};
    ASSERT_EQ(ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING, &current).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitsForQueuedJob(peer.handle, [&] {
        return ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING, current).status;
    }));
}

// The recording status answers from ovphysx's own record of the recording: it waits for no
// instance's work.
TEST_F(Lanes, RecordingStatusWaitsForNoInstanceWork)
{
    OwnedInstance peer;
    ASSERT_EQ(ovphysx_create_instance(&peer.handle).status, OVPHYSX_API_SUCCESS);
    WorkerHold hold(peer.handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t job = ovphysx_submit_test_job_for_test_internal(peer.handle, nullptr, kJobMs);
    ASSERT_EQ(job.status, OVPHYSX_API_SUCCESS);

    std::atomic<bool> returned{false};
    ovphysx_api_status_t status = OVPHYSX_API_ERROR;
    bool recording = true;
    std::thread caller([&] {
        status = ovphysx_ovd_is_recording(m_handle, &recording).status;
        returned = true;
    });
    EXPECT_TRUE(turnsTrue(returned)) << "the status query waited for the peer's queued job";
    hold.release();
    caller.join();
    EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_TRUE(waitForTaskSuccess(peer.handle, job, OVPHYSX_TIMEOUT_INFINITE));
}

// A recording transition takes companion admission only after every instance's earlier work
// finished: two that wait behind a peer's queued job on two threads both wait, and neither
// refuses the other as a transition in progress.
TEST_F(Lanes, RecordingTransitionsWaitingForWorkRefuseNoOtherTransition)
{
    OwnedInstance peer;
    ASSERT_EQ(ovphysx_create_instance(&peer.handle).status, OVPHYSX_API_SUCCESS);
    WorkerHold hold(peer.handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t job = ovphysx_submit_test_job_for_test_internal(peer.handle, nullptr, kJobMs);
    ASSERT_EQ(job.status, OVPHYSX_API_SUCCESS);

    std::string errors[2];
    std::thread stops[2];
    for (int i = 0; i < 2; ++i)
    {
        stops[i] = std::thread([&, i] {
            // Nothing records, so each stop fails after the wait, with its own text.
            EXPECT_EQ(ovphysx_ovd_stop_recording(m_handle).status, OVPHYSX_API_INVALID_STATE);
            const ovphysx_string_t error = ovphysx_get_last_error();
            errors[i].assign(error.ptr ? error.ptr : "", error.length);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    hold.release();
    for (std::thread& stop : stops)
        stop.join();
    for (const std::string& error : errors)
        EXPECT_EQ(error, "stop_recording: no recording is active");
    EXPECT_TRUE(waitForTaskSuccess(peer.handle, job, OVPHYSX_TIMEOUT_INFINITE));
}

TEST_F(Lanes, KinematicUpdateAfterUnwaitedStep)
{
    loadScene(m_handle);
    ovphysx_query_handle_t articulations = 0;
    openQuery(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, articulations);

    // The step is dispatched and not waited for. The update's query-type probe needs an
    // idle instance, so it must run only after the step finished.
    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t update = ovphysx_update_articulations_kinematic(
        m_handle, articulations, OVPHYSX_ARTICULATION_KINEMATIC_POSITION, ovstage_cuda_sync_t{0, 0});
    EXPECT_EQ(update.status, OVPHYSX_API_SUCCESS) << std::string(ovphysx_get_last_error().ptr,
                                                                 ovphysx_get_last_error().length);
    EXPECT_NE(update.op_index, 0u);
    if (update.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_TRUE(waitForTaskSuccess(m_handle, update));
    }
    EXPECT_TRUE(waitForTaskSuccess(m_handle, step));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, articulations)));
}

// What a callback observed while it ran.
struct CallbackProbe
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_instance_handle_t victim = OVPHYSX_INVALID_HANDLE;
    ovphysx_instance_handle_t peer = OVPHYSX_INVALID_HANDLE;
    ovphysx_enqueue_result_t done{};
    std::atomic<bool> called{false};
    ovphysx_api_status_t wait = OVPHYSX_API_SUCCESS;
    std::string waitError;
    ovphysx_api_status_t stepSync = OVPHYSX_API_SUCCESS;
    ovphysx_enqueue_result_t query{OVPHYSX_API_SUCCESS, 1, 1};
    ovphysx_query_handle_t queryHandle = 1;
    ovphysx_enqueue_result_t clone{OVPHYSX_API_SUCCESS, 1, 1};
    ovphysx_enqueue_result_t update{OVPHYSX_API_SUCCESS, 1, 1};
    ovphysx_api_status_t destroy = OVPHYSX_API_SUCCESS;
    ovphysx_api_status_t getConfig = OVPHYSX_API_INVALID_STATE;
    ovphysx_api_status_t isAttached = OVPHYSX_API_INVALID_STATE;
    bool attached = false;
    ovphysx_api_status_t otherThread = OVPHYSX_API_ERROR;
    std::thread later;
    ovphysx_api_status_t laterStatus = OVPHYSX_API_ERROR;
};

// The calls every callback makes. Each blocking call must be refused at once.
void probeBlockingCalls(CallbackProbe& probe)
{
    ovphysx_wait_task_result_t waited{};
    probe.wait = ovphysx_wait_task(probe.handle, probe.done, OVPHYSX_TIMEOUT_INFINITE, &waited).status;
    const ovphysx_string_t waitError = ovphysx_get_last_error();
    probe.waitError.assign(waitError.ptr ? waitError.ptr : "", waitError.length);
    ovphysx_destroy_wait_task_result(&waited);
    probe.stepSync = ovphysx_step_sync(probe.handle, 1.0f / 60.0f).status;
    probe.query = ovphysx_query(probe.handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL,
                                &probe.queryHandle, nullptr);
    // Long calls never wait, but they queue instance work, so they are refused too.
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/CallbackCloneTarget");
    probe.clone = ovphysx_clone(probe.handle, OVPHYSX_LITERAL("/World/Cube1"), &target, 1, nullptr, nullptr);
    probe.update = ovphysx_update_from_ovstage(probe.handle, ovstage_ordinal_range_t{1, 1, true});
    probe.destroy = ovphysx_destroy_instance(probe.victim).status;
    ovphysx_config_value_t value{};
    probe.getConfig = ovphysx_get_config(probe.handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, &value).status;
    probe.isAttached = ovphysx_is_stage_attached(probe.handle, nullptr, &probe.attached).status;
}

void expectRefusedInCallback(const CallbackProbe& probe)
{
    EXPECT_TRUE(probe.called.load());
    EXPECT_EQ(probe.wait, OVPHYSX_API_INVALID_STATE);
    EXPECT_NE(probe.waitError.find("callback"), std::string::npos) << probe.waitError;
    EXPECT_EQ(probe.stepSync, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.query.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.query.op_index, 0u);
    EXPECT_EQ(probe.query.instance, 0u);
    EXPECT_EQ(probe.queryHandle, 0u);
    for (const ovphysx_enqueue_result_t& longCall : {probe.clone, probe.update})
    {
        EXPECT_EQ(longCall.status, OVPHYSX_API_INVALID_STATE);
        EXPECT_EQ(longCall.op_index, 0u);
        EXPECT_EQ(longCall.instance, 0u);
    }
    EXPECT_EQ(probe.destroy, OVPHYSX_API_INVALID_STATE);
    EXPECT_NE(probe.getConfig, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.isAttached, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(probe.attached);
}

TEST_F(Lanes, CallbackRefusesBlockingCalls)
{
    loadScene(m_handle);
    OwnedInstance victim;
    ASSERT_EQ(ovphysx_create_instance(&victim.handle).status, OVPHYSX_API_SUCCESS);
    ovphysx_query_handle_t query = 0;
    openQuery(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, query);
    const ovx_token_or_string_t mass{0, {OVPHYSX_ATTR_MASS, sizeof(OVPHYSX_ATTR_MASS) - 1}};
    // Open the read backend before the callback exists, so the other client thread does
    // not log initialization diagnostics while the callback holds the log dispatcher.
    ovphysx_read_handle_t warmup = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        m_handle, ovphysx_read(m_handle, query, &mass, 1, ovstage_cuda_sync_t{ 0, 0 }, &warmup, nullptr)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, warmup)));

    // Log callback, delivered on a helper thread.
    {
        struct LogProbe
        {
            CallbackProbe probe;
            ovphysx_query_handle_t query = 0;
            ovx_token_or_string_t mass{};
        } state;
        state.probe.handle = m_handle;
        state.probe.victim = victim.handle;
        state.probe.done = ovphysx_release_query(m_handle, 0);
        ASSERT_EQ(state.probe.done.status, OVPHYSX_API_SUCCESS);
        state.query = query;
        state.mass = mass;
        struct Restore
        {
            ~Restore() { ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr); }
        } restore;
        ASSERT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_WARNING, nullptr,
            [](ovphysx_log_level_t, ovphysx_string_t, ovphysx_string_t, double, void* data) {
                LogProbe& state = *static_cast<LogProbe*>(data);
                if (state.probe.called.exchange(true))
                    return;
                probeBlockingCalls(state.probe);
                // Another client thread keeps using the instance while the callback runs.
                std::thread client([&] {
                    ovphysx_read_handle_t read = 0;
                    state.probe.otherThread = test_utils::complete_task(
                        state.probe.handle,
                        ovphysx_read(state.probe.handle, state.query, &state.mass, 1,
                                    ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)).status;
                    if (read)
                        test_utils::complete_task(state.probe.handle, ovphysx_release_read(state.probe.handle, read));
                });
                client.join();
            }, &state).status, OVPHYSX_API_SUCCESS);
        std::thread emitter([] { ovphysx_log_emit_test_messages(); });
        emitter.join();
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
        expectRefusedInCallback(state.probe);
        EXPECT_EQ(state.probe.otherThread, OVPHYSX_API_SUCCESS);
        // The refused wait left the receipt uncollected.
        EXPECT_TRUE(waitForTaskSuccess(m_handle, state.probe.done));
    }

    // Object-change callback, delivered inside a detach job on the instance's worker,
    // which holds the instance's lane.
    {
        OwnedInstance owner;
        ASSERT_EQ(ovphysx_create_instance(&owner.handle).status, OVPHYSX_API_SUCCESS);
        loadScene(owner.handle);
        struct ObjectProbe
        {
            CallbackProbe probe;
            ovphysx_query_handle_t peerQuery = 0;
        } state;
        state.probe.handle = owner.handle;
        state.probe.victim = victim.handle;
        state.probe.peer = m_handle;
        state.probe.done = ovphysx_release_query(owner.handle, 0);
        ASSERT_EQ(state.probe.done.status, OVPHYSX_API_SUCCESS);
        ovphysx_object_change_callbacks_t callbacks{};
        callbacks.user_data = &state;
        callbacks.on_all_objects_destroyed = [](void* data) {
            ObjectProbe& state = *static_cast<ObjectProbe*>(data);
            if (state.probe.called.exchange(true))
                return;
            probeBlockingCalls(state.probe);
            // A call on another instance from another thread runs while the callback does.
            std::thread client([&] {
                state.probe.otherThread = test_utils::complete_task(
                    state.probe.peer,
                    ovphysx_query(state.probe.peer, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL,
                                  &state.peerQuery, nullptr)).status;
            });
            client.join();
            // A call on this instance from another thread waits for the detach, then runs.
            state.probe.later = std::thread([&] {
                state.probe.laterStatus = ovphysx_set_config(
                    state.probe.handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
                    ovphysx_config_value_string(OVPHYSX_LITERAL(""))).status;
            });
        };
        ovphysx_subscription_id_t subscription = OVPHYSX_INVALID_SUBSCRIPTION_ID;
        ASSERT_EQ(ovphysx_subscribe_object_changes(owner.handle, &callbacks, &subscription).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(waitForTaskSuccess(owner.handle, ovphysx_detach_ovstage(owner.handle)));
        if (state.probe.later.joinable())
            state.probe.later.join();
        EXPECT_EQ(ovphysx_unsubscribe_object_changes(owner.handle, subscription).status, OVPHYSX_API_SUCCESS);
        expectRefusedInCallback(state.probe);
        EXPECT_EQ(state.probe.otherThread, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(state.probe.laterStatus, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(waitForTaskSuccess(owner.handle, state.probe.done));
        if (state.peerQuery)
        {
            EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, state.peerQuery)));
        }
    }

    // The refused destroys left the instance intact.
    bool attached = true;
    EXPECT_EQ(ovphysx_is_stage_attached(victim.handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attached);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

} // namespace
