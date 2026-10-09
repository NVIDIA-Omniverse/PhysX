// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-1 AC-3 AC-4 AC-5a AC-6 AC-7 AC-8
 *
 * TEST-CAPI-WRITE-001: the public app -> physics C write surface of ADR-0012, that is
 * ovphysx_write / ovphysx_fetch_write_next / ovphysx_commit_group / ovphysx_release_write.
 *
 * Argument, handle and lifecycle validation is C-first (AC-8): every reject case asserts a
 * guarantee the C API makes so every frontend inherits it. The session-open, iteration,
 * commit-identity and discard-on-release cases exercise the live write path end to end.
 *
 * Cases covered elsewhere:
 *   - Python surface (AC-9): pytest in tests/python_tests.
 *   - Device residency and the cuda-sync handoff: TEST-INPUT-DEVICE-001, GPU fixture.
 *   - Scatter correctness: TEST-INPUT-CORE-001, an ovruntime doctest over the backend.
 */

#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <string>
#include <vector>

namespace
{

// A syntactically valid attribute. Whether the queried type accepts it is a separate
// check the session makes. These cases only need a non-null pointer.
ovx_string_or_token_t makeAttr(const char* name)
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = std::strlen(name);
    return a;
}

ovstage_cuda_sync_t noSync()
{
    ovstage_cuda_sync_t s{};
    s.stream = 0;
    s.wait_event = 0;
    return s;
}

// Element count of a DLTensor: product of shape dims times dtype.lanes.
// prims.count * lanes is wrong for a [N, 3] position column with lanes=1.
size_t tensorFloatCount(const DLTensor& t)
{
    size_t n = t.dtype.lanes ? static_cast<size_t>(t.dtype.lanes) : 1;
    if (t.shape != nullptr)
    {
        for (int32_t d = 0; d < t.ndim; ++d)
            n *= static_cast<size_t>(t.shape[d]);
    }
    return n;
}

void stepN(ovphysx_handle_t handle, int n)
{
    for (int i = 0; i < n; ++i)
        ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
}

} // namespace

using namespace test_utils;

class WriteApiTest : public PhysXTestFixture
{
protected:
    // A session's groups are planned from the query's matched prims, so the cases below need a real
    // attached scene. An unattached fixture fails at ovphysx_query, before the write is reached.
    // These cases step once so a subsequent read has groups. This fixture is CPU;
    // a pre-step write would apply (AC-5a). DirectGPU refusal is covered in Python.
    void attachAndStep()
    {
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
            << "Failed to attach ovstage USD";
        stepN(m_handle, 1);
    }
};

// --- Argument validation (REQ AC-7, AC-8) -------------------------------------
// C-first, so every frontend inherits these and none may add a check the C API
// does not make.

TEST_F(WriteApiTest, WriteRejectsNullOutParam)
{
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    const ovphysx_result_t r = ovphysx_write(m_handle, /*query*/ 1, &attr, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(WriteApiTest, WriteRejectsNullAttribute)
{
    // One attribute per session, so this is required rather than a count that may be 0:
    // ovstage_map_group_t has no attribute field to label emitted groups with.
    ovphysx_write_handle_t w = 12345;
    const ovphysx_result_t r = ovphysx_write(m_handle, /*query*/ 1, nullptr, &w);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, 0u) << "out_write must be cleared before any early return";
}

TEST_F(WriteApiTest, WriteRejectsUnknownInstanceHandle)
{
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 12345;
    const ovphysx_result_t r = ovphysx_write(/*handle*/ 0, /*query*/ 1, &attr, &w);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, 0u);
}

TEST_F(WriteApiTest, FetchRejectsNullOutGroup)
{
    const ovphysx_result_t r = ovphysx_fetch_write_next(m_handle, /*write*/ 1, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(WriteApiTest, FetchClearsOutGroupBeforeReturning)
{
    // Through uintptr_t: 0xdeadbeef is a 32-bit unsigned int, and casting it straight to a
    // 64-bit pointer is MSVC C4312, which this build promotes to an error.
    const ovstage_map_group_t* g =
        reinterpret_cast<const ovstage_map_group_t*>(static_cast<uintptr_t>(0xdeadbeefu));
    ovphysx_fetch_write_next(m_handle, /*write*/ 999, &g);
    EXPECT_EQ(g, nullptr) << "a caller must never see a stale pointer on a failed fetch";
}

TEST_F(WriteApiTest, CommitRejectsNullGroup)
{
    const ovphysx_result_t r = ovphysx_commit_group(m_handle, /*write*/ 1, nullptr, noSync());
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// AC-7: unknown handles split by whether the call mutates. Only teardown is idempotent.
TEST_F(WriteApiTest, ReleaseIsIdempotentForAnUnknownHandle)
{
    EXPECT_EQ(ovphysx_release_write(m_handle, /*write*/ 999).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_release_write(m_handle, /*write*/ 999).status, OVPHYSX_API_SUCCESS);
}

TEST_F(WriteApiTest, CommitIsNotIdempotentForAnUnknownHandle)
{
    // Commit IS the mutation, so a success return would tell the caller state was
    // published when nothing was. Deliberately unlike release.
    ovstage_map_group_t bogus{};
    const ovphysx_result_t r = ovphysx_commit_group(m_handle, /*write*/ 999, &bogus, noSync());
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// --- Session behaviour --------------------------------------------------------

TEST_F(WriteApiTest, OpenSessionOverAQuery)
{
    attachAndStep();

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);

    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    const ovphysx_result_t r = ovphysx_write(m_handle, q, &attr, &w);

    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << "REQ AC-1: a session opens over the query";
    EXPECT_NE(w, 0u);

    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
}

TEST_F(WriteApiTest, RejectedAttributePreservesRuntimeDiagnostic)
{
    attachAndStep();
    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);

    for (const char* name : { "notAnAttribute", "points", "inverseMass", "linearAcceleration" })
    {
        const ovx_string_or_token_t attr = makeAttr(name);
        ovphysx_write_handle_t w = 12345;
        EXPECT_NE(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(w, 0u);
        const ovphysx_string_t error = ovphysx_get_last_error();
        const std::string message(error.ptr ? error.ptr : "", error.ptr ? error.length : 0);
        EXPECT_NE(message.find(name), std::string::npos) << message;
        EXPECT_NE(message.find("rigid"), std::string::npos) << message;
        EXPECT_NE(message.find("do not accept"), std::string::npos) << message;
    }

    const ovx_string_or_token_t position = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    EXPECT_EQ(ovphysx_write(m_handle, q, &position, &w).status, OVPHYSX_API_SUCCESS);
    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
}

TEST_F(WriteApiTest, RejectedAttributeSurvivesLogCallbackError)
{
    attachAndStep();
    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);

    struct CallbackState
    {
        std::promise<int32_t> result;
        bool completed = false;
    } callbackState;
    std::future<int32_t> callbackResult = callbackState.result.get_future();
    const uint32_t previousLevel = ovphysx_get_log_level();
    struct RestoreLogging
    {
        uint32_t previousLevel;
        ~RestoreLogging()
        {
            ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
            ovphysx_set_log_level(previousLevel);
        }
    } restoreLogging{ previousLevel };
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_ERROR).status, OVPHYSX_API_SUCCESS);
    const ovphysx_log_callback_t callback = [](ovphysx_log_level_t, ovphysx_string_t message,
                                              ovphysx_string_t, double, void* data) {
        CallbackState* state = static_cast<CallbackState*>(data);
        if (!state->completed && message.ptr &&
            std::string(message.ptr, message.length).find("notAnAttribute") != std::string::npos)
        {
            // This configuration call rejects callback reentry without acquiring a scene lock.
            state->result.set_value(ovphysx_set_log_level(OVPHYSX_LOG_ERROR).status);
            state->completed = true;
        }
    };
    ASSERT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_ERROR, nullptr, callback, &callbackState).status,
              OVPHYSX_API_SUCCESS);

    const ovx_string_or_token_t attr = makeAttr("notAnAttribute");
    ovphysx_write_handle_t w = 12345;
    EXPECT_NE(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(w, 0u);
    const ovphysx_string_t error = ovphysx_get_last_error();
    const std::string message(error.ptr ? error.ptr : "", error.ptr ? error.length : 0);
    // The public flush only covers accepted callbacks, not Carbonite's upstream async buffer.
    ASSERT_EQ(callbackResult.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(callbackResult.get(), OVPHYSX_API_ERROR);
    EXPECT_NE(message.find("notAnAttribute"), std::string::npos) << message;
    EXPECT_NE(message.find("rigid"), std::string::npos) << message;
    EXPECT_EQ(message.find("within a log callback"), std::string::npos) << message;
    ovphysx_release_query(m_handle, q);
}

// AC-1: one attribute per session. Writing position and orientation over one prim set is
// two sessions, not one call, because ovstage_map_group_t carries no attribute field.
TEST_F(WriteApiTest, TwoAttributesMeanTwoSessions)
{
    attachAndStep();

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);

    for (const char* name : { OVPHYSX_ATTR_POSITION, OVPHYSX_ATTR_ORIENTATION })
    {
        const ovx_string_or_token_t attr = makeAttr(name);
        ovphysx_write_handle_t w = 0;
        EXPECT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS) << name;
        ovphysx_release_write(m_handle, w);
    }
    ovphysx_release_query(m_handle, q);
}

// AC-7: END_OF_ITERATION is the only non-error exit, and is distinguishable from failure.
TEST_F(WriteApiTest, IterationEndsWithEndOfIteration)
{
    attachAndStep();

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    ASSERT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);

    const ovstage_map_group_t* g = nullptr;
    ovphysx_result_t r;
    int guard = 0;
    while ((r = ovphysx_fetch_write_next(m_handle, w, &g)).status == OVPHYSX_API_SUCCESS)
    {
        ASSERT_NE(g, nullptr);
        ASSERT_LT(++guard, 10000) << "iteration did not terminate";
        EXPECT_EQ(ovphysx_commit_group(m_handle, w, g, noSync()).status, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(r.status, OVPHYSX_API_END_OF_ITERATION);
    EXPECT_EQ(g, nullptr);

    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
}

// AC-3: the pointer is the commit identity, and addresses are never recycled between
// groups. That is what lets a committed group be told apart from a live one.
TEST_F(WriteApiTest, GroupAddressesAreNeverRecycled)
{
    attachAndStep();

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    ASSERT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);

    std::vector<const ovstage_map_group_t*> seen;
    const ovstage_map_group_t* g = nullptr;
    while (ovphysx_fetch_write_next(m_handle, w, &g).status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_EQ(std::find(seen.begin(), seen.end(), g), seen.end())
            << "group address reused within a session";
        seen.push_back(g);
        ovphysx_commit_group(m_handle, w, g, noSync());
    }

    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
}

// AC-3, AC-7: committing the same group twice fails the second time. Not idempotent.
TEST_F(WriteApiTest, DoubleCommitIsRejected)
{
    attachAndStep();

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    ASSERT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);

    const ovstage_map_group_t* g = nullptr;
    ASSERT_EQ(ovphysx_fetch_write_next(m_handle, w, &g).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_commit_group(m_handle, w, g, noSync()).status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(ovphysx_commit_group(m_handle, w, g, noSync()).status, OVPHYSX_API_SUCCESS);

    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
}

// AC-4: a further fetch does not invalidate an earlier uncommitted group.
TEST_F(WriteApiTest, FetchDoesNotInvalidateAnUncommittedGroup)
{
    attachAndStep();

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    ASSERT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);

    const ovstage_map_group_t* first = nullptr;
    const ovstage_map_group_t* second = nullptr;
    ASSERT_EQ(ovphysx_fetch_write_next(m_handle, w, &first).status, OVPHYSX_API_SUCCESS);
    if (ovphysx_fetch_write_next(m_handle, w, &second).status == OVPHYSX_API_SUCCESS)
    {
        // The earlier group must still be committable after the later fetch.
        EXPECT_EQ(ovphysx_commit_group(m_handle, w, first, noSync()).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(ovphysx_commit_group(m_handle, w, second, noSync()).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
}

// AC-4, AC-6: release discards anything uncommitted rather than publishing it, so a caller that
// abandons mid-fill cannot leak uninitialized data into the solver. Asserted in its observable form
// (read the covered prims back and check the sentinel never appears) because the internal
// "was it committed" flag would pass even if release published anyway.
TEST_F(WriteApiTest, ReleaseDiscardsUncommittedGroups)
{
    attachAndStep();

    // Reads a column to compare against before the abandoned write.
    auto readPositions = [&]()
    {
        std::vector<float> out;
        ovphysx_query_handle_t rq = 0;
        EXPECT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &rq).status,
                  OVPHYSX_API_SUCCESS);
        const ovx_string_or_token_t a = makeAttr(OVPHYSX_ATTR_POSITION);
        ovphysx_read_handle_t rh = 0;
        EXPECT_EQ(ovphysx_read(m_handle, rq, &a, 1, &rh).status, OVPHYSX_API_SUCCESS);
        const ovstage_read_group_t* g = nullptr;
        while (ovphysx_fetch_read_next(m_handle, rh, &g).status == OVPHYSX_API_SUCCESS)
        {
            if (g->data.tensors && g->data.tensor_count > 0 &&
                g->data.tensors[0].device.device_type == kDLCPU)
            {
                const DLTensor& t = g->data.tensors[0];
                const size_t n = size_t(g->prims.count) * (t.dtype.lanes ? t.dtype.lanes : 1);
                const float* p = static_cast<const float*>(t.data);
                out.insert(out.end(), p, p + n);
            }
            ovphysx_release_group(m_handle, rh, g->read_group_id);
        }
        ovphysx_release_read(m_handle, rh);
        ovphysx_release_query(m_handle, rq);
        return out;
    };

    const std::vector<float> before = readPositions();
    ASSERT_FALSE(before.empty()) << "need a host column to compare against";

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    ASSERT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);

    const ovstage_map_group_t* g = nullptr;
    ASSERT_EQ(ovphysx_fetch_write_next(m_handle, w, &g).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(g, nullptr);
    // Fill with a sentinel no simulation would produce, then release without committing.
    const DLTensor& t = g->data.tensors[0];
    if (t.device.device_type == kDLCPU)
    {
        const size_t n = size_t(g->prims.count) * (t.dtype.lanes ? t.dtype.lanes : 1);
        std::fill_n(static_cast<float*>(t.data), n, -12345.0f);
    }
    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);

    const std::vector<float> after = readPositions();
    ASSERT_EQ(after.size(), before.size());
    for (size_t i = 0; i < after.size(); ++i)
    {
        EXPECT_NE(after[i], -12345.0f) << "the sentinel from an abandoned group reached the solver, at " << i;
        EXPECT_FLOAT_EQ(after[i], before[i]) << "an uncommitted group changed prim data, at " << i;
    }
}

// AC-5a: on CPU a pre-step write commits and is applied; it does not auto-warm.
TEST_F(WriteApiTest, PreStepWriteAppliesOnCpu)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to attach ovstage USD";

    ovphysx_query_handle_t q = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_POSITION);
    ovphysx_write_handle_t w = 0;
    ASSERT_EQ(ovphysx_write(m_handle, q, &attr, &w).status, OVPHYSX_API_SUCCESS);

    std::vector<float> written;
    const ovstage_map_group_t* g = nullptr;
    while (ovphysx_fetch_write_next(m_handle, w, &g).status == OVPHYSX_API_SUCCESS)
    {
        ASSERT_NE(g, nullptr);
        ASSERT_TRUE(g->data.tensors && g->data.tensor_count > 0);
        const DLTensor& t = g->data.tensors[0];
        ASSERT_EQ(t.device.device_type, kDLCPU);
        const size_t n = tensorFloatCount(t);
        ASSERT_GT(n, 0u);
        float* p = static_cast<float*>(t.data);
        for (size_t i = 0; i < n; ++i)
            p[i] = 100.0f + static_cast<float>(written.size() + i);
        written.insert(written.end(), p, p + n);
        ASSERT_EQ(ovphysx_commit_group(m_handle, w, g, noSync()).status, OVPHYSX_API_SUCCESS)
            << "CPU pre-step commit must apply, not refuse";
    }
    ovphysx_release_write(m_handle, w);
    ovphysx_release_query(m_handle, q);
    ASSERT_FALSE(written.empty());

    std::vector<float> after;
    ovphysx_query_handle_t rq = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &rq).status,
              OVPHYSX_API_SUCCESS);
    ovphysx_read_handle_t rh = 0;
    ASSERT_EQ(ovphysx_read(m_handle, rq, &attr, 1, &rh).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* rg = nullptr;
    while (ovphysx_fetch_read_next(m_handle, rh, &rg).status == OVPHYSX_API_SUCCESS)
    {
        if (rg->data.tensors && rg->data.tensor_count > 0 &&
            rg->data.tensors[0].device.device_type == kDLCPU)
        {
            const DLTensor& t = rg->data.tensors[0];
            const size_t n = tensorFloatCount(t);
            const float* p = static_cast<const float*>(t.data);
            after.insert(after.end(), p, p + n);
        }
        ovphysx_release_group(m_handle, rh, rg->read_group_id);
    }
    ovphysx_release_read(m_handle, rh);
    ovphysx_release_query(m_handle, rq);

    ASSERT_EQ(after.size(), written.size());
    for (size_t i = 0; i < after.size(); ++i)
        EXPECT_FLOAT_EQ(after[i], written[i]) << "CPU pre-step write did not land at " << i;
}
