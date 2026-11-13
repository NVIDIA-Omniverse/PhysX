// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-5 AC-13 AC-14
 *
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-7
 *
 * TEST-CAPI-ASYNC-007: query, read and write handles exist when the call that opens them
 * returns, on both venues, and short calls run only against the attachment they were
 * submitted for.
 */

// Handles at submission: a query, read or write call writes its handle before it returns,
// also when it is queued, so later calls can name the handle at once. Handles come from
// one process-wide counter and are never handed out twice; a handle of another instance
// is rejected when a call is submitted; a call that names the handle of a failed producer
// fails its own task, naming that producer's task; a released handle is gone for calls
// submitted after the release, and releasing a pending or failed handle succeeds.
// Attachment generations: a short call queued behind an attach or detach runs against the
// attachment it made; if that attach or detach failed, calls submitted before its failure
// was collected fail, and calls submitted after see the attachment that exists.

#include "global_test_environment.h"
#include "ovphysxTestHelpers.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_geometry_query.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

// Private binding ABI; deliberately absent from the public headers.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_is_query_valid(
    ovphysx_instance_handle_t handle, ovphysx_query_handle_t query, bool* out_valid);

namespace
{

constexpr const char* kBoxes = "tests/data/boxes_falling_on_groundplane.usda";
constexpr const char* kEnvs = "tests/data/basic_simulation.usda";
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

// The status of task once it finished, observed without collecting it.
ovphysx_task_status_t statusWhenFinished(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    ovphysx_task_status_t status = statusOf(handle, task);
    while ((status == OVPHYSX_TASK_STATUS_PENDING || status == OVPHYSX_TASK_STATUS_RUNNING) &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        status = statusOf(handle, task);
    }
    return status;
}

// Collects every task of the instance; failures are tolerated.
void drainTasks(ovphysx_instance_handle_t handle)
{
    ovphysx_wait_task_result_t residual{};
    ovphysx_wait_all(handle, OVPHYSX_TIMEOUT_INFINITE, &residual);
    ovphysx_destroy_wait_task_result(&residual);
}

ovx_token_or_string_t attributeName(const char* name)
{
    ovx_token_or_string_t attribute{};
    attribute.string = {name, std::strlen(name)};
    return attribute;
}

// A token no path dictionary resolves. Its string is NULL, so the token names the attribute,
// and a read or a write that requests it is accepted and its task fails.
ovx_token_or_string_t unresolvableAttribute()
{
    ovx_token_or_string_t attribute{};
    attribute.token = ovx_token_t{std::numeric_limits<uint64_t>::max() - 1};
    return attribute;
}

std::string lastError()
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    return std::string(error.ptr ? error.ptr : "", error.length);
}

// Collects task, which must have failed, and returns its failure text (empty when it did
// not fail). The text is owned by the wait result, so it is copied before that goes.
std::string collectFailure(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx_wait_task_result_t result{};
    const ovphysx_api_status_t status = ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &result).status;
    std::string text;
    if (status == OVPHYSX_API_ERROR)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, task);
        text.assign(error.ptr ? error.ptr : "", error.length);
    }
    ovphysx_destroy_wait_task_result(&result);
    return text;
}

// A queued call: accepted with a receipt and still pending while the worker is held.
::testing::AssertionResult queued(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS || task.op_index == 0 || task.instance != handle)
        return ::testing::AssertionFailure() << "rejected with status " << task.status << ": " << lastError();
    const ovphysx_task_status_t status = statusOf(handle, task);
    if (status != OVPHYSX_TASK_STATUS_PENDING)
        return ::testing::AssertionFailure() << "task " << task.op_index << " is " << status << ", not pending";
    return ::testing::AssertionSuccess();
}

// A call that ran on the calling thread: its task finished before the call returned.
::testing::AssertionResult ranInline(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS || task.op_index == 0)
        return ::testing::AssertionFailure() << "rejected with status " << task.status << ": " << lastError();
    const ovphysx_task_status_t status = statusOf(handle, task);
    if (status != OVPHYSX_TASK_STATUS_SUCCEEDED)
        return ::testing::AssertionFailure() << "task " << task.op_index << " is " << status << ", not finished";
    return ::testing::AssertionSuccess();
}

// A rejection: no receipt, the given status, and a last error that contains text.
::testing::AssertionResult rejectedWith(ovphysx_enqueue_result_t task, ovphysx_api_status_t status, const char* text)
{
    if (task.status != status || task.op_index != 0)
        return ::testing::AssertionFailure() << "status " << task.status << " task " << task.op_index
                                             << ", not a rejection with status " << status;
    const std::string error = lastError();
    if (error.find(text) == std::string::npos)
        return ::testing::AssertionFailure() << "rejected with \"" << error << "\", not \"" << text << "\"";
    return ::testing::AssertionSuccess();
}

// The failure a call reports when a handle it names came from a failed producer.
std::string failedInputText(const char* api, ovphysx_enqueue_result_t call, const char* kind, uint64_t handle,
                            ovphysx_enqueue_result_t producer)
{
    return std::string(api) + " (task " + std::to_string(call.op_index) + "): input " + kind + " " +
           std::to_string(handle) + " came from failed task " + std::to_string(producer.op_index);
}

// The failure a call reports when the attach or detach submitted before it failed.
std::string attachmentChangedText(const char* api, ovphysx_enqueue_result_t call, ovphysx_enqueue_result_t lifecycle)
{
    return std::string(api) + " (task " + std::to_string(call.op_index) + "): attach or detach task " +
           std::to_string(lifecycle.op_index) +
           " failed, so the attachment this call was submitted for does not exist; collect that task, then "
           "submit again";
}

// An instance a test owns, created in the constructor and destroyed with every Stage the
// test registry holds for it.
struct OwnedInstance
{
    OwnedInstance() { EXPECT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS); }
    ~OwnedInstance()
    {
        if (handle == OVPHYSX_INVALID_HANDLE)
            return;
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    }
    OwnedInstance(const OwnedInstance&) = delete;
    OwnedInstance& operator=(const OwnedInstance&) = delete;
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
};

// A Stage populated from a USD file and sealed at ordinal 1, for a test that attaches it
// itself. It is destroyed after a detach from the instance an attach of it was submitted
// to, once that instance's earlier work ran.
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
        const ovx_string_t path{usdPath, std::strlen(usdPath)};
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
            drainTasks(m_attachedTo);
            bool attached = false;
            if (ovphysx_is_stage_attached(m_attachedTo, m_stage, &attached).status != OVPHYSX_API_SUCCESS ||
                (attached && test_utils::complete_task(m_attachedTo, ovphysx_detach_ovstage(m_attachedTo)).status !=
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

private:
    ovstage_instance_t* m_stage = nullptr;
    ovphysx_instance_handle_t m_attachedTo = OVPHYSX_INVALID_HANDLE;
};

class HandlesAtSubmit : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        // The CPU instance is shared across suites; start from an empty task table.
        drainTasks(m_handle);
    }

    void loadScene()
    {
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kBoxes));
        ASSERT_EQ(ovphysx_step_sync(m_handle, kDt).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_query_handle_t openQuery(ovphysx_instance_handle_t handle,
                                     ovphysx_query_target_t type = OVPHYSX_QUERY_TARGET_RIGID_BODY)
    {
        ovphysx_query_handle_t query = 0;
        EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)));
        EXPECT_NE(query, 0u);
        return query;
    }

    ovphysx_read_handle_t openRead(ovphysx_query_handle_t query)
    {
        const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
        ovphysx_read_handle_t read = 0;
        EXPECT_TRUE(waitForTaskSuccess(
            m_handle, ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)));
        EXPECT_NE(read, 0u);
        return read;
    }

    ovphysx_write_handle_t openWrite(ovphysx_query_handle_t query)
    {
        const ovx_token_or_string_t velocity = attributeName(OVPHYSX_ATTR_LINEAR_VELOCITY);
        ovphysx_write_handle_t write = 0;
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr)));
        EXPECT_NE(write, 0u);
        return write;
    }
};

// Behind held work a query, a read and a write return their handles at once, and later
// calls name them while their producers are still queued: the whole chain runs in order
// once the worker runs. Results other than handles (summaries, groups) are written when
// the calls run.
TEST_F(HandlesAtSubmit, QueuedProducersReturnHandlesThatLaterCallsUseAtOnce)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t summary{};
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, &summary);
    ASSERT_TRUE(queued(m_handle, opened));
    ASSERT_NE(query, 0u) << "the handle is written when the call returns";
    EXPECT_EQ(summary.total_prim_count, 0u) << "the summary is written when the query runs";

    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    const ovphysx_enqueue_result_t readOpened =
        ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    ASSERT_TRUE(queued(m_handle, readOpened));
    ASSERT_NE(read, 0u);
    const ovstage_read_group_t* group = nullptr;
    const ovphysx_enqueue_result_t readFetched = ovphysx_fetch_read_next(m_handle, read, &group);
    ASSERT_TRUE(queued(m_handle, readFetched));
    EXPECT_EQ(group, nullptr) << "a group is written when the fetch runs";

    const ovx_token_or_string_t velocity = attributeName(OVPHYSX_ATTR_LINEAR_VELOCITY);
    ovphysx_write_handle_t write = 0;
    const ovphysx_enqueue_result_t writeOpened = ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr);
    ASSERT_TRUE(queued(m_handle, writeOpened));
    ASSERT_NE(write, 0u);
    ovphysx_fetched_write_group_t writeGroup{};
    const ovphysx_enqueue_result_t writeFetched = ovphysx_fetch_write_next(m_handle, write, &writeGroup);
    ASSERT_TRUE(queued(m_handle, writeFetched));

    const ovphysx_enqueue_result_t writeReleased = ovphysx_release_write(m_handle, write);
    const ovphysx_enqueue_result_t readReleased = ovphysx_release_read(m_handle, read);
    const ovphysx_enqueue_result_t queryReleased = ovphysx_release_query(m_handle, query);
    for (const ovphysx_enqueue_result_t& task : {writeReleased, readReleased, queryReleased})
        EXPECT_TRUE(queued(m_handle, task));
    EXPECT_NE(query, read);
    EXPECT_NE(read, write);
    EXPECT_NE(query, write);

    hold.release();
    for (const ovphysx_enqueue_result_t& task :
         {opened, readOpened, readFetched, writeOpened, writeFetched, writeReleased, readReleased, queryReleased})
        EXPECT_TRUE(waitForTaskSuccess(m_handle, task));
    EXPECT_GT(summary.total_prim_count, 0u);
    EXPECT_NE(group, nullptr);
    EXPECT_NE(writeGroup.group, nullptr);

    // Released, the handles are gone: calls that name them are rejected as for unknown
    // handles.
    ovphysx_read_handle_t again = 7;
    EXPECT_TRUE(rejectedWith(
        ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &again, nullptr),
        OVPHYSX_API_NOT_FOUND,
                             "query is not owned by this instance"));
    EXPECT_EQ(again, 0u);
    EXPECT_TRUE(rejectedWith(ovphysx_fetch_read_next(m_handle, read, &group), OVPHYSX_API_NOT_FOUND,
                             "read is not owned by this instance"));
}

// Another instance's query, read and write are rejected with NOT_FOUND when a call is
// submitted, whether the call would run on the calling thread or queue, and also while
// the producer of the handle is still queued. The owner keeps using them.
TEST_F(HandlesAtSubmit, ForeignHandlesAreRejectedAtSubmissionOnBothVenues)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery(m_handle);
    const ovphysx_read_handle_t read = openRead(query);
    const ovphysx_write_handle_t write = openWrite(query);
    OwnedInstance peer;
    ASSERT_NE(peer.handle, OVPHYSX_INVALID_HANDLE);

    WorkerHold ownerHold(m_handle);
    ASSERT_TRUE(ownerHold.held);
    ovphysx_query_handle_t pendingQuery = 0;
    const ovphysx_enqueue_result_t pendingOpened =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &pendingQuery, nullptr);
    ASSERT_TRUE(queued(m_handle, pendingOpened));
    ASSERT_NE(pendingQuery, 0u);

    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovstage_map_group_t bogus{};
    for (const bool busy : {false, true})
    {
        SCOPED_TRACE(busy ? "peer busy" : "peer idle");
        WorkerHold peerHold(peer.handle);
        if (!busy)
            peerHold.release();
        ovphysx_read_handle_t foreignRead = 7;
        EXPECT_TRUE(rejectedWith(ovphysx_read(peer.handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 },
                                             &foreignRead, nullptr),
                                 OVPHYSX_API_NOT_FOUND, "query is not owned by this instance"));
        EXPECT_EQ(foreignRead, 0u);
        EXPECT_TRUE(rejectedWith(ovphysx_read(peer.handle, pendingQuery, &position, 1, ovstage_cuda_sync_t{ 0, 0 },
                                             &foreignRead, nullptr),
                                 OVPHYSX_API_NOT_FOUND, "query is not owned by this instance"))
            << "a handle whose producer is queued belongs to its instance already";
        ovphysx_write_handle_t foreignWrite = 7;
        EXPECT_TRUE(rejectedWith(ovphysx_write(peer.handle, query, &position, 1, &foreignWrite, nullptr),
                                 OVPHYSX_API_NOT_FOUND, "query is not owned by this instance"));
        EXPECT_EQ(foreignWrite, 0u);
        const ovstage_read_group_t* group = nullptr;
        EXPECT_TRUE(rejectedWith(ovphysx_fetch_read_next(peer.handle, read, &group), OVPHYSX_API_NOT_FOUND,
                                 "read is not owned by this instance"));
        ovphysx_fetched_write_group_t writeGroup{};
        EXPECT_TRUE(rejectedWith(ovphysx_fetch_write_next(peer.handle, write, &writeGroup), OVPHYSX_API_NOT_FOUND,
                                 "write is not owned by this instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_commit_group(peer.handle, write, &bogus, ovstage_cuda_sync_t{0, 0}),
                                 OVPHYSX_API_NOT_FOUND, "write is not owned by this instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_release_group(peer.handle, read, ovstage_read_group_id_t{ 1 }),
                                 OVPHYSX_API_NOT_FOUND,
                                 "read belongs to another instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_release_read(peer.handle, read), OVPHYSX_API_NOT_FOUND,
                                 "Resource belongs to another instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_release_write(peer.handle, write), OVPHYSX_API_NOT_FOUND,
                                 "Resource belongs to another instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_release_query(peer.handle, query), OVPHYSX_API_NOT_FOUND,
                                 "Resource belongs to another instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_release_query(peer.handle, pendingQuery), OVPHYSX_API_NOT_FOUND,
                                 "Resource belongs to another instance"));
        if (!busy)
        {
            // Calling-thread calls; behind held work they would wait for it.
            EXPECT_TRUE(rejectedWith(ovphysx_update_articulations_kinematic(peer.handle, query, 0,
                                                                            ovstage_cuda_sync_t{0, 0}),
                                     OVPHYSX_API_NOT_FOUND, "query is not owned by this instance"));
            bool valid = true;
            EXPECT_EQ(ovphysx_internal_is_query_valid(peer.handle, query, &valid).status, OVPHYSX_API_NOT_FOUND);
            EXPECT_FALSE(valid);
        }
    }
    drainTasks(peer.handle);

    ownerHold.release();
    EXPECT_TRUE(waitForTaskSuccess(m_handle, pendingOpened));
    const ovstage_read_group_t* group = nullptr;
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
    EXPECT_NE(group, nullptr) << "the owner's read is untouched";
    bool valid = false;
    EXPECT_EQ(ovphysx_internal_is_query_valid(m_handle, query, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(valid);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, pendingQuery)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// Unknown handles keep their rule on both venues: a release succeeds as a no-op (it still
// returns a task), a call that uses the handle is rejected with NOT_FOUND. A handle of
// another kind is unknown to a call of this kind: releasing a read as a query or a write
// leaves the read open.
TEST_F(HandlesAtSubmit, UnknownHandlesKeepTheReleaseRuleOnBothVenues)
{
    loadScene();
    constexpr uint64_t kUnknown = 0xDEAD;
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovstage_map_group_t bogus{};
    for (const bool busy : {false, true})
    {
        SCOPED_TRACE(busy ? "busy" : "idle");
        WorkerHold hold(m_handle);
        if (!busy)
            hold.release();
        for (const ovphysx_enqueue_result_t& release :
             {ovphysx_release_query(m_handle, kUnknown), ovphysx_release_read(m_handle, kUnknown),
              ovphysx_release_write(m_handle, kUnknown),
              ovphysx_release_group(m_handle, kUnknown, ovstage_read_group_id_t{ 1 })})
        {
            EXPECT_EQ(release.status, OVPHYSX_API_SUCCESS);
            EXPECT_NE(release.op_index, 0u);
        }
        ovphysx_read_handle_t read = 7;
        EXPECT_TRUE(rejectedWith(ovphysx_read(m_handle, kUnknown, &position, 1, ovstage_cuda_sync_t{ 0, 0 },
                                             &read, nullptr),
                                 OVPHYSX_API_NOT_FOUND, "query is not owned by this instance"));
        EXPECT_EQ(read, 0u);
        ovphysx_write_handle_t write = 7;
        EXPECT_TRUE(rejectedWith(ovphysx_write(m_handle, kUnknown, &position, 1, &write, nullptr),
                                 OVPHYSX_API_NOT_FOUND, "query is not owned by this instance"));
        EXPECT_EQ(write, 0u);
        const ovstage_read_group_t* group = nullptr;
        EXPECT_TRUE(rejectedWith(ovphysx_fetch_read_next(m_handle, kUnknown, &group), OVPHYSX_API_NOT_FOUND,
                                 "read is not owned by this instance"));
        ovphysx_fetched_write_group_t writeGroup{};
        EXPECT_TRUE(rejectedWith(ovphysx_fetch_write_next(m_handle, kUnknown, &writeGroup), OVPHYSX_API_NOT_FOUND,
                                 "write is not owned by this instance"));
        EXPECT_TRUE(rejectedWith(ovphysx_commit_group(m_handle, kUnknown, &bogus, ovstage_cuda_sync_t{0, 0}),
                                 OVPHYSX_API_NOT_FOUND, "write is not owned by this instance"));
        hold.release();
        ovphysx_wait_task_result_t all{};
        EXPECT_EQ(ovphysx_wait_all(m_handle, OVPHYSX_TIMEOUT_INFINITE, &all).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(all.num_errors, 0u);
        ovphysx_destroy_wait_task_result(&all);
    }

    const ovphysx_query_handle_t query = openQuery(m_handle);
    const ovphysx_read_handle_t read = openRead(query);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, read)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, read)));
    const ovstage_read_group_t* group = nullptr;
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
    EXPECT_NE(group, nullptr) << "the read was not released by a release of another kind";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A producer whose task fails still returned its handle. A call that names it is
// accepted and fails its own task, naming the producer's task, on both venues; releasing
// the failed handle succeeds and removes it. That holds for a read over a failed query, a
// fetch of a failed read, and the articulation update, which runs on the calling thread.
TEST_F(HandlesAtSubmit, CallsThatNameAFailedProducersHandleFailTheirTask)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t query = openQuery(m_handle);
    const ovx_token_or_string_t unresolvable = unresolvableAttribute();

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    ovphysx_read_handle_t read = 0;
    const ovphysx_enqueue_result_t readOpened =
        ovphysx_read(m_handle, query, &unresolvable, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    ASSERT_TRUE(queued(m_handle, readOpened));
    ASSERT_NE(read, 0u);
    const ovstage_read_group_t* group = nullptr;
    const ovphysx_enqueue_result_t queuedFetch = ovphysx_fetch_read_next(m_handle, read, &group);
    ASSERT_TRUE(queued(m_handle, queuedFetch));
    hold.release();
    EXPECT_FALSE(collectFailure(m_handle, readOpened).empty());
    EXPECT_EQ(collectFailure(m_handle, queuedFetch),
              failedInputText("ovphysx_fetch_read_next", queuedFetch, "read", read, readOpened));
    EXPECT_EQ(group, nullptr);

    // The instance is idle now, so the same call runs on the calling thread and fails the
    // same way.
    const ovphysx_enqueue_result_t inlineFetch = ovphysx_fetch_read_next(m_handle, read, &group);
    ASSERT_EQ(inlineFetch.status, OVPHYSX_API_SUCCESS) << lastError();
    if (ovphysxIdleCallsRunOnCaller())
    {
        EXPECT_EQ(statusOf(m_handle, inlineFetch), OVPHYSX_TASK_STATUS_FAILED);
    }
    EXPECT_EQ(collectFailure(m_handle, inlineFetch),
              failedInputText("ovphysx_fetch_read_next", inlineFetch, "read", read, readOpened));

    // A producer that fails on the calling thread returns its handle as well.
    ovphysx_read_handle_t inlineRead = 0;
    const ovphysx_enqueue_result_t inlineOpened =
        ovphysx_read(m_handle, query, &unresolvable, 1, ovstage_cuda_sync_t{ 0, 0 }, &inlineRead, nullptr);
    ASSERT_EQ(inlineOpened.status, OVPHYSX_API_SUCCESS) << lastError();
    EXPECT_NE(inlineRead, 0u);
    EXPECT_FALSE(collectFailure(m_handle, inlineOpened).empty());
    const ovphysx_enqueue_result_t fetchOfInline = ovphysx_fetch_read_next(m_handle, inlineRead, &group);
    EXPECT_EQ(collectFailure(m_handle, fetchOfInline),
              failedInputText("ovphysx_fetch_read_next", fetchOfInline, "read", inlineRead, inlineOpened));

    // Releasing a failed handle succeeds and removes it.
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, inlineRead)));
    EXPECT_TRUE(rejectedWith(ovphysx_fetch_read_next(m_handle, read, &group), OVPHYSX_API_NOT_FOUND,
                             "read is not owned by this instance"));

    // A query whose producer failed: queued behind a detach, it finds no stage. A read over
    // it, queued behind it, names a failed producer.
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_query_handle_t lost = 0;
    ovphysx_read_handle_t readOfLost = 0;
    ovphysx_enqueue_result_t detached{};
    ovphysx_enqueue_result_t lostOpened{};
    ovphysx_enqueue_result_t readOfLostOpened{};
    {
        WorkerHold detachHold(m_handle);
        ASSERT_TRUE(detachHold.held);
        detached = ovphysx_detach_ovstage(m_handle);
        ASSERT_TRUE(queued(m_handle, detached));
        lostOpened = ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &lost, nullptr);
        ASSERT_TRUE(queued(m_handle, lostOpened));
        ASSERT_NE(lost, 0u);
        readOfLostOpened =
            ovphysx_read(m_handle, lost, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &readOfLost, nullptr);
        ASSERT_TRUE(queued(m_handle, readOfLostOpened));
        ASSERT_NE(readOfLost, 0u);
    }
    EXPECT_TRUE(waitForTaskSuccess(m_handle, detached));
    EXPECT_TRUE(waitForTaskFailure(m_handle, lostOpened, "attach an ovstage Stage first"));
    EXPECT_EQ(collectFailure(m_handle, readOfLostOpened),
              failedInputText("ovphysx_read", readOfLostOpened, "query", lost, lostOpened));
    // The articulation update runs on the calling thread after earlier work: accepted,
    // it fails its task the same way.
    const ovphysx_enqueue_result_t update = ovphysx_update_articulations_kinematic(
        m_handle, lost, OVPHYSX_ARTICULATION_KINEMATIC_POSITION, ovstage_cuda_sync_t{0, 0});
    ASSERT_EQ(update.status, OVPHYSX_API_SUCCESS) << lastError();
    EXPECT_EQ(collectFailure(m_handle, update),
              failedInputText("ovphysx_update_articulations_kinematic", update, "query", lost, lostOpened));
    bool valid = true;
    EXPECT_EQ(ovphysx_internal_is_query_valid(m_handle, lost, &valid).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(valid) << "a query whose producer failed was never opened";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, readOfLost)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, lost)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

// A release of a handle whose producer is still queued is accepted at once and runs after
// that producer: calls submitted after the release are rejected as for an unknown handle,
// and a second release is a successful no-op. The release of a failed handle queued behind
// its producer succeeds too.
TEST_F(HandlesAtSubmit, ReleasesOfPendingAndFailedHandlesRunAfterTheirProducers)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    const ovphysx_query_handle_t base = openQuery(m_handle);
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    const ovx_token_or_string_t unresolvable = unresolvableAttribute();

    WorkerHold hold(m_handle);
    ASSERT_TRUE(hold.held);
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr);
    ASSERT_TRUE(queued(m_handle, opened));
    const ovphysx_enqueue_result_t released = ovphysx_release_query(m_handle, query);
    ASSERT_TRUE(queued(m_handle, released));
    ovphysx_read_handle_t read = 7;
    EXPECT_TRUE(rejectedWith(
        ovphysx_read(m_handle, query, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr),
        OVPHYSX_API_NOT_FOUND,
                             "query is not owned by this instance"));
    EXPECT_EQ(read, 0u);
    const ovphysx_enqueue_result_t releasedAgain = ovphysx_release_query(m_handle, query);
    ASSERT_TRUE(queued(m_handle, releasedAgain));

    ovphysx_read_handle_t failedRead = 0;
    const ovphysx_enqueue_result_t failedOpened =
        ovphysx_read(m_handle, base, &unresolvable, 1, ovstage_cuda_sync_t{ 0, 0 }, &failedRead, nullptr);
    ASSERT_TRUE(queued(m_handle, failedOpened));
    const ovphysx_enqueue_result_t failedReleased = ovphysx_release_read(m_handle, failedRead);
    ASSERT_TRUE(queued(m_handle, failedReleased));
    hold.release();

    EXPECT_TRUE(waitForTaskSuccess(m_handle, opened));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, released));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, releasedAgain));
    EXPECT_FALSE(collectFailure(m_handle, failedOpened).empty());
    EXPECT_TRUE(waitForTaskSuccess(m_handle, failedReleased));
    bool valid = true;
    EXPECT_EQ(ovphysx_internal_is_query_valid(m_handle, query, &valid).status, OVPHYSX_API_NOT_FOUND)
        << "the query was opened, then released";
    const ovstage_read_group_t* group = nullptr;
    EXPECT_TRUE(rejectedWith(ovphysx_fetch_read_next(m_handle, failedRead, &group), OVPHYSX_API_NOT_FOUND,
                             "read is not owned by this instance"));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, base)));
}

// Handles come from one process-wide counter: across instances, kinds and venues, and
// after releases, every handle is larger than every handle before it, so none is reused.
TEST_F(HandlesAtSubmit, HandlesAreNeverReused)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    loadScene();
    OwnedInstance peer;
    ASSERT_NE(peer.handle, OVPHYSX_INVALID_HANDLE);
    std::vector<uint64_t> minted;
    for (int round = 0; round < 20; ++round)
    {
        const ovphysx_query_handle_t query = openQuery(m_handle);
        const ovphysx_read_handle_t read = openRead(query);
        // A debug-render query needs no stage.
        const ovphysx_query_handle_t peerQuery = openQuery(peer.handle, OVPHYSX_QUERY_TARGET_DEBUG_RENDER);
        minted.insert(minted.end(), {query, read, peerQuery});
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
        EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
        EXPECT_TRUE(waitForTaskSuccess(peer.handle, ovphysx_release_query(peer.handle, peerQuery)));

        WorkerHold hold(m_handle);
        ASSERT_TRUE(hold.held);
        ovphysx_query_handle_t queuedQuery = 0;
        const ovphysx_enqueue_result_t opened =
            ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &queuedQuery, nullptr);
        ASSERT_TRUE(queued(m_handle, opened));
        minted.push_back(queuedQuery);
        const ovphysx_enqueue_result_t released = ovphysx_release_query(m_handle, queuedQuery);
        hold.release();
        EXPECT_TRUE(waitForTaskSuccess(m_handle, opened));
        EXPECT_TRUE(waitForTaskSuccess(m_handle, released));
    }
    for (size_t i = 1; i < minted.size(); ++i)
        EXPECT_GT(minted[i], minted[i - 1]) << "handle " << i;
}

// Once every handle was used, a producer is rejected with ERROR on both venues: no task,
// no handle and nothing queued. The test reaches the end of the handle space through a
// test gate and restores the counter afterwards.
TEST_F(HandlesAtSubmit, ExhaustedHandlesRejectProducersOnBothVenues)
{
    loadScene();
    const ovphysx_query_handle_t base = openQuery(m_handle);
    // Restores the counter once, also when an assertion ends the test early.
    struct RestoreNextHandle
    {
        uint64_t next;
        bool restored = false;
        void now()
        {
            if (!restored)
                ovphysx_exchange_next_output_handle_for_test_internal(next);
            restored = true;
        }
        ~RestoreNextHandle() { now(); }
    } restore{ovphysx_exchange_next_output_handle_for_test_internal(UINT64_MAX)};
    ASSERT_NE(restore.next, 0u);

    ovphysx_query_handle_t last = 0;
    ASSERT_TRUE(waitForTaskSuccess(
        m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &last, nullptr)));
    EXPECT_EQ(last, UINT64_MAX) << "the last handle is still handed out";

    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    for (const bool busy : {false, true})
    {
        SCOPED_TRACE(busy ? "busy" : "idle");
        WorkerHold hold(m_handle);
        if (!busy)
            hold.release();
        // Two queued no-op releases around the rejected calls: their receipts are adjacent,
        // so nothing was queued in between.
        const ovphysx_enqueue_result_t before = ovphysx_release_group(m_handle, m_handle, ovstage_read_group_id_t{ 1 });
        ovphysx_query_handle_t query = 7;
        EXPECT_TRUE(rejectedWith(
            ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr),
            OVPHYSX_API_ERROR, "ovphysx_query: output handles are exhausted"));
        EXPECT_EQ(query, 0u);
        ovphysx_read_handle_t read = 7;
        EXPECT_TRUE(rejectedWith(
            ovphysx_read(m_handle, base, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr),
            OVPHYSX_API_ERROR,
                                 "ovphysx_read: output handles are exhausted"));
        EXPECT_EQ(read, 0u);
        const ovphysx_enqueue_result_t after = ovphysx_release_group(m_handle, m_handle, ovstage_read_group_id_t{ 1 });
        EXPECT_EQ(after.op_index, before.op_index + 1);
        hold.release();
        drainTasks(m_handle);
    }

    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, last)));
    restore.now();
    const ovphysx_query_handle_t again = openQuery(m_handle);
    EXPECT_EQ(again, restore.next) << "minting resumes where it was";
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, again)));
    EXPECT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, base)));
}

// Attachment generations, on instances of their own so that a failed attach never
// reaches another suite.
class AttachmentGenerations : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_NE(m_instance.handle, OVPHYSX_INVALID_HANDLE);
    }

    ovphysx_enqueue_result_t query(ovphysx_query_target_t type, ovphysx_query_handle_t* out)
    {
        return ovphysx_query(m_instance.handle, type, OVPHYSX_SCOPE_ALL, out, nullptr);
    }

    OwnedInstance m_instance;
};

// A query and a read submitted right after an attach, with no wait in between, run
// against the attachment that attach made.
TEST_F(AttachmentGenerations, CallsRightAfterAnAttachUseItsAttachment)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = m_instance.handle;
    SealedStage stage(kBoxes, "generations-attach-query");
    ASSERT_TRUE(stage.valid());
    WorkerHold hold(handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t attached = ovphysx_attach_ovstage(handle, stage.get(), 1);
    ASSERT_TRUE(queued(handle, attached));
    stage.submittedTo(handle);
    ovphysx_query_handle_t bodies = 0;
    const ovphysx_enqueue_result_t opened = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, &bodies);
    ASSERT_TRUE(queued(handle, opened));
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    const ovphysx_enqueue_result_t readOpened =
        ovphysx_read(handle, bodies, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    ASSERT_TRUE(queued(handle, readOpened));
    const ovstage_read_group_t* group = nullptr;
    const ovphysx_enqueue_result_t fetched = ovphysx_fetch_read_next(handle, read, &group);
    ASSERT_TRUE(queued(handle, fetched));
    hold.release();
    for (const ovphysx_enqueue_result_t& task : {attached, opened, readOpened, fetched})
        EXPECT_TRUE(waitForTaskSuccess(handle, task));
    EXPECT_NE(group, nullptr);
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_read(handle, read)));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, bodies)));
}

// An attach that fails when it runs (its Stage is attached to another instance) leaves no
// attachment. Calls submitted before its failure is collected fail, queued or on the idle
// instance; once it is collected, a debug-render query, which needs no stage, opens.
TEST_F(AttachmentGenerations, CallsAfterAFailedAttachFailUntilItsFailureIsCollected)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = m_instance.handle;
    OwnedInstance other;
    ASSERT_NE(other.handle, OVPHYSX_INVALID_HANDLE);
    SealedStage stage(kBoxes, "generations-failed-attach");
    ASSERT_TRUE(stage.valid());
    ASSERT_TRUE(waitForTaskSuccess(other.handle, ovphysx_attach_ovstage(other.handle, stage.get(), 1)));
    stage.submittedTo(other.handle);

    ovphysx_query_handle_t early = 0;
    ovphysx_enqueue_result_t attached{};
    ovphysx_enqueue_result_t earlyOpened{};
    {
        WorkerHold hold(handle);
        ASSERT_TRUE(hold.held);
        attached = ovphysx_attach_ovstage(handle, stage.get(), 1);
        ASSERT_TRUE(queued(handle, attached));
        earlyOpened = query(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, &early);
        ASSERT_TRUE(queued(handle, earlyOpened));
        ASSERT_NE(early, 0u);
    }
    ASSERT_EQ(statusWhenFinished(handle, attached), OVPHYSX_TASK_STATUS_FAILED);
    EXPECT_EQ(collectFailure(handle, earlyOpened), attachmentChangedText("ovphysx_query", earlyOpened, attached));

    // Not collected yet: a call on the now idle instance fails the same way.
    ovphysx_query_handle_t late = 0;
    const ovphysx_enqueue_result_t lateOpened = query(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, &late);
    ASSERT_EQ(lateOpened.status, OVPHYSX_API_SUCCESS) << lastError();
    EXPECT_EQ(collectFailure(handle, lateOpened), attachmentChangedText("ovphysx_query", lateOpened, attached));

    EXPECT_TRUE(waitForTaskFailure(handle, attached, "attach_ovstage"));
    ovphysx_query_handle_t collected = 0;
    const ovphysx_enqueue_result_t collectedOpened = query(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, &collected);
    if (ovphysxIdleCallsRunOnCaller())
    {
        EXPECT_TRUE(ranInline(handle, collectedOpened));
    }
    EXPECT_TRUE(waitForTaskSuccess(handle, collectedOpened));
    bool attachedNow = true;
    EXPECT_EQ(ovphysx_is_stage_attached(handle, nullptr, &attachedNow).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(attachedNow);
    for (const ovphysx_query_handle_t released : {early, late, collected})
        EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, released)));
}

// Calls queued behind a detach that succeeds see no stage: a query of bodies fails as it
// would on the calling thread, and a debug-render query, which needs no stage, opens.
TEST_F(AttachmentGenerations, CallsAfterADetachSeeNoStage)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = m_instance.handle;
    SealedStage stage(kBoxes, "generations-detach");
    ASSERT_TRUE(stage.valid());
    ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_attach_ovstage(handle, stage.get(), 1)));
    stage.submittedTo(handle);
    WorkerHold hold(handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t detached = ovphysx_detach_ovstage(handle);
    ASSERT_TRUE(queued(handle, detached));
    ovphysx_query_handle_t bodies = 0;
    const ovphysx_enqueue_result_t bodiesOpened = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, &bodies);
    ASSERT_TRUE(queued(handle, bodiesOpened));
    ovphysx_query_handle_t debug = 0;
    const ovphysx_enqueue_result_t debugOpened = query(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, &debug);
    ASSERT_TRUE(queued(handle, debugOpened));
    hold.release();
    EXPECT_TRUE(waitForTaskSuccess(handle, detached));
    EXPECT_TRUE(waitForTaskFailure(handle, bodiesOpened, "attach an ovstage Stage first"));
    EXPECT_TRUE(waitForTaskSuccess(handle, debugOpened));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, debug)));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, bodies)));
}

// With Stage A attached, an attach of Stage B fails when it runs. Queries submitted before
// that failure is collected fail, on the idle instance or queued, also one that runs only
// after the collection; queries submitted after the collection run against A.
TEST_F(AttachmentGenerations, AFailedAttachLeavesTheEarlierAttachmentToLaterCalls)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << kNoWorkerThread;
    const ovphysx_instance_handle_t handle = m_instance.handle;
    SealedStage first(kBoxes, "generations-first");
    SealedStage second(kBoxes, "generations-second");
    ASSERT_TRUE(first.valid() && second.valid());
    ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_attach_ovstage(handle, first.get(), 1)));
    first.submittedTo(handle);
    // With the initial scene parse done, a query on the idle instance runs on the calling thread.
    ASSERT_EQ(ovphysx_step_sync(handle, kDt).status, OVPHYSX_API_SUCCESS);

    const ovphysx_enqueue_result_t attachedSecond = ovphysx_attach_ovstage(handle, second.get(), 1);
    ASSERT_EQ(attachedSecond.status, OVPHYSX_API_SUCCESS) << "the already-attached check belongs to the attach job";
    second.submittedTo(handle);
    ASSERT_EQ(statusWhenFinished(handle, attachedSecond), OVPHYSX_TASK_STATUS_FAILED);

    // On the idle instance: the task has failed when the call returns.
    ovphysx_query_handle_t idle = 0;
    const ovphysx_enqueue_result_t idleOpened = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, &idle);
    ASSERT_EQ(idleOpened.status, OVPHYSX_API_SUCCESS) << lastError();
    if (ovphysxIdleCallsRunOnCaller())
    {
        EXPECT_EQ(statusOf(handle, idleOpened), OVPHYSX_TASK_STATUS_FAILED);
    }
    EXPECT_EQ(collectFailure(handle, idleOpened), attachmentChangedText("ovphysx_query", idleOpened, attachedSecond));

    // Queued: one runs before the collection, one is submitted before it and runs after it,
    // and one is submitted after it, with a read over it.
    ovphysx_query_handle_t queuedBefore = 0;
    ovphysx_query_handle_t admittedBefore = 0;
    ovphysx_query_handle_t submittedAfter = 0;
    ovphysx_read_handle_t read = 0;
    ovphysx_enqueue_result_t queuedBeforeOpened{};
    {
        WorkerHold hold(handle);
        ASSERT_TRUE(hold.held);
        queuedBeforeOpened = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, &queuedBefore);
        ASSERT_TRUE(queued(handle, queuedBeforeOpened));
    }
    EXPECT_EQ(collectFailure(handle, queuedBeforeOpened),
              attachmentChangedText("ovphysx_query", queuedBeforeOpened, attachedSecond));
    WorkerHold hold(handle);
    ASSERT_TRUE(hold.held);
    const ovphysx_enqueue_result_t admittedBeforeOpened = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, &admittedBefore);
    ASSERT_TRUE(queued(handle, admittedBeforeOpened));
    EXPECT_TRUE(waitForTaskFailure(handle, attachedSecond, "a stage is already attached"));
    const ovphysx_enqueue_result_t submittedAfterOpened = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, &submittedAfter);
    ASSERT_TRUE(queued(handle, submittedAfterOpened));
    const ovx_token_or_string_t position = attributeName(OVPHYSX_ATTR_POSITION);
    const ovphysx_enqueue_result_t readOpened =
        ovphysx_read(handle, submittedAfter, &position, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr);
    ASSERT_TRUE(queued(handle, readOpened));
    hold.release();
    EXPECT_EQ(collectFailure(handle, admittedBeforeOpened),
              attachmentChangedText("ovphysx_query", admittedBeforeOpened, attachedSecond));
    EXPECT_TRUE(waitForTaskSuccess(handle, submittedAfterOpened));
    EXPECT_TRUE(waitForTaskSuccess(handle, readOpened));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_read(handle, read)));
    bool attachedFirst = false;
    EXPECT_EQ(ovphysx_is_stage_attached(handle, first.get(), &attachedFirst).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attachedFirst);
    for (const ovphysx_query_handle_t released : {idle, queuedBefore, admittedBefore, submittedAfter})
        EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, released)));
}

// Updates, clones and steps follow the same rule. With Stage A attached, an attach of Stage
// B fails; an update and a clone submitted before that failure is collected fail naming it,
// and run against A once it is collected. So does a step started on the idle calling thread
// after a second failed attach.
TEST_F(AttachmentGenerations, LongCallsAfterAFailedAttachFailUntilItsFailureIsCollected)
{
    const ovphysx_instance_handle_t handle = m_instance.handle;
    SealedStage first(kEnvs, "generations-long-first");
    SealedStage second(kEnvs, "generations-long-second");
    ASSERT_TRUE(first.valid() && second.valid());
    ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_attach_ovstage(handle, first.get(), 1)));
    first.submittedTo(handle);

    const ovphysx_enqueue_result_t failedAttach = ovphysx_attach_ovstage(handle, second.get(), 1);
    ASSERT_EQ(failedAttach.status, OVPHYSX_API_SUCCESS);
    second.submittedTo(handle);
    ASSERT_EQ(statusWhenFinished(handle, failedAttach), OVPHYSX_TASK_STATUS_FAILED);
    const ovstage_ordinal_range_t range{1, 1, true};
    ovphysx_string_t target = OVPHYSX_LITERAL("/World/envs/env1");
    const ovphysx_string_t source = OVPHYSX_LITERAL("/World/envs/env0");
    const ovphysx_enqueue_result_t update = ovphysx_update_from_ovstage(handle, range);
    const ovphysx_enqueue_result_t clone = ovphysx_clone(handle, source, &target, 1, nullptr, nullptr);
    EXPECT_EQ(collectFailure(handle, update),
              attachmentChangedText("ovphysx_update_from_ovstage", update, failedAttach));
    EXPECT_EQ(collectFailure(handle, clone), attachmentChangedText("ovphysx_clone", clone, failedAttach));
    EXPECT_TRUE(waitForTaskFailure(handle, failedAttach, "a stage is already attached"));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_update_from_ovstage(handle, range)));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_clone(handle, source, &target, 1, nullptr, nullptr)));

    // The same holds for a step on an instance whose scene is parsed.
    ASSERT_EQ(ovphysx_step_sync(handle, kDt).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t failedAgain = ovphysx_attach_ovstage(handle, second.get(), 1);
    ASSERT_EQ(statusWhenFinished(handle, failedAgain), OVPHYSX_TASK_STATUS_FAILED);
    const ovphysx_enqueue_result_t step = ovphysx_step(handle, kDt);
    EXPECT_EQ(collectFailure(handle, step), attachmentChangedText("ovphysx_step", step, failedAgain));
    EXPECT_TRUE(waitForTaskFailure(handle, failedAgain, "a stage is already attached"));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_step(handle, kDt)));
}

// The articulation update and the SDF evaluation, which run on the calling thread after
// earlier work, follow the same rule: submitted after a failed attach whose failure is not
// collected, their tasks fail naming it; once it is collected the update runs against the
// attachment that exists. The SDF tensors are described on a CUDA device but never read.
TEST_F(AttachmentGenerations, CallingThreadCallsAfterAFailedAttachFailUntilItsFailureIsCollected)
{
    const ovphysx_instance_handle_t handle = m_instance.handle;
    SealedStage first(kBoxes, "generations-calling-thread-first");
    SealedStage second(kBoxes, "generations-calling-thread-second");
    ASSERT_TRUE(first.valid() && second.valid());
    ASSERT_TRUE(waitForTaskSuccess(handle, ovphysx_attach_ovstage(handle, first.get(), 1)));
    first.submittedTo(handle);
    ASSERT_EQ(ovphysx_step_sync(handle, kDt).status, OVPHYSX_API_SUCCESS);
    ovphysx_query_handle_t articulations = 0;
    ASSERT_TRUE(waitForTaskSuccess(handle, query(OVPHYSX_QUERY_TARGET_ARTICULATION, &articulations)));

    const ovphysx_enqueue_result_t failedAttach = ovphysx_attach_ovstage(handle, second.get(), 1);
    ASSERT_EQ(failedAttach.status, OVPHYSX_API_SUCCESS);
    second.submittedTo(handle);
    const ovphysx_enqueue_result_t update = ovphysx_update_articulations_kinematic(
        handle, articulations, OVPHYSX_ARTICULATION_KINEMATIC_POSITION, ovstage_cuda_sync_t{0, 0});
    ASSERT_EQ(update.status, OVPHYSX_API_SUCCESS) << lastError();
    EXPECT_EQ(collectFailure(handle, update),
              attachmentChangedText("ovphysx_update_articulations_kinematic", update, failedAttach));

    alignas(16) float storage[4] = {};
    int64_t pointsShape[3] = {1, 1, 3};
    int64_t outShape[3] = {1, 1, 4};
    DLTensor points{};
    points.data = storage;
    points.device = DLDevice{kDLCUDA, 0};
    points.ndim = 3;
    points.dtype = DLDataType{kDLFloat, 32, 1};
    points.shape = pointsShape;
    DLTensor out = points;
    out.shape = outShape;
    const ovphysx_enqueue_result_t sdf =
        ovphysx_evaluate_sdf_shapes(handle, articulations, &points, &out, ovstage_cuda_sync_t{0, 0});
    ASSERT_EQ(sdf.status, OVPHYSX_API_SUCCESS) << lastError();
    EXPECT_EQ(collectFailure(handle, sdf), attachmentChangedText("ovphysx_evaluate_sdf_shapes", sdf, failedAttach));

    EXPECT_TRUE(waitForTaskFailure(handle, failedAttach, "a stage is already attached"));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_update_articulations_kinematic(
                                               handle, articulations, OVPHYSX_ARTICULATION_KINEMATIC_POSITION,
                                               ovstage_cuda_sync_t{0, 0})));
    EXPECT_TRUE(waitForTaskSuccess(handle, ovphysx_release_query(handle, articulations)));
}

} // namespace
