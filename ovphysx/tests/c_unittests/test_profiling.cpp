// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-PROFILER-002
 * @covers AC-1 AC-2 AC-5 AC-6 AC-10 AC-11
 */

// Tests for the profiler backend selection.
//
// The backend is bound during ovphysx_create_instance(), so these tests use
// standalone TEST() (not PhysXTestFixture) to control the full instance lifecycle,
// the same way the OmniPVD recording tests do.
//
// Whether a consumer records the zones cannot be observed from inside the process for
// the NVTX backend, so what is asserted there is the selection, the loader's own
// account of which backend bound, and side-effect freedom. The trace backend is the
// one case with a checkable artifact, and it gets an isolated process for it.
//
// The trace assertion below exercises a representative previously-uninstrumented
// entry point and the simulation workflow. Exhaustive public-API coverage is enforced
// by test_api_profile_zones.py -- see TEST-CAPI-PROFILER-001.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "ovphysx_test_utils.h"
#include "test_utilities.h"

#include <ovstage/ovx_path_dictionary.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace test_utils;

namespace
{

const char* const kEnvBackend = "OVPHYSX_PROFILER";
const char* const kEnvOutputPath = "OVPHYSX_PROFILER_OUTPUT";

void set_env(const char* name, const char* value)
{
#if defined(_WIN32)
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

void unset_env(const char* name)
{
#if defined(_WIN32)
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
}

int32_t backend_setting()
{
    int32_t value = -1;
    ovphysx_result_t r = ovphysx_get_global_config_int32(OVPHYSX_CONFIG_PROFILER_BACKEND, &value);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << "reading /physics/profilerBackend must succeed";
    return value;
}

// Captures the loader's own account of which Carbonite profiler backend the process
// ended up with. That line is the only in-process evidence that the backend bound --
// whether zones reach a consumer is not observable from here -- and it is INFO, while
// the process default threshold is WARNING.
class ScopedLogCapture
{
public:
    ScopedLogCapture()
    {
        ovphysx_set_log_level(OVPHYSX_LOG_INFO);
        mActive = ovphysx_log_capture_start().status == OVPHYSX_API_SUCCESS;
    }

    ~ScopedLogCapture()
    {
        if (mActive)
        {
            ovphysx_log_capture_stop();
        }
        ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
    }

    ScopedLogCapture(const ScopedLogCapture&) = delete;
    ScopedLogCapture& operator=(const ScopedLogCapture&) = delete;

    bool active() const
    {
        return mActive;
    }

private:
    bool mActive = false;
};

// Substrings of the outcomes the loader reports. Kept short so a reworded log line
// does not fail the test for the wrong reason.
const char* const kPublishing = "Profile zones are publishing to the";
const char* const kAnotherBackendActive = "profile zones go there rather than to the selected backend";
const char* const kNoneWhileBound = "instance asked for none and the zones keep publishing";
const char* const kTraceDestinationFixed = "chrome trace destination is already set to";

size_t count_complete_trace_events(const std::string& contents, const std::string& name)
{
    struct ObjectFields
    {
        bool hasTargetName = false;
        bool hasCompletePhase = false;
        bool hasDuration = false;
    };

    const std::string nameField = "\"name\":\"" + name + "\"";
    const std::string completePhaseField = "\"ph\":\"X\"";
    const std::string durationField = "\"dur\":";
    size_t count = 0;
    std::vector<ObjectFields> objects;
    bool inString = false;
    bool escaped = false;
    for (size_t i = 0; i < contents.size(); ++i)
    {
        const char c = contents[i];
        if (escaped)
        {
            escaped = false;
            continue;
        }
        if (inString)
        {
            if (c == '\\')
                escaped = true;
            else if (c == '"')
                inString = false;
            continue;
        }
        if (c == '"')
        {
            if (!objects.empty())
            {
                ObjectFields& object = objects.back();
                if (contents.compare(i, nameField.size(), nameField) == 0)
                    object.hasTargetName = true;
                else if (contents.compare(i, completePhaseField.size(), completePhaseField) == 0)
                    object.hasCompletePhase = true;
                else if (contents.compare(i, durationField.size(), durationField) == 0)
                    object.hasDuration = true;
            }
            inString = true;
        }
        else if (c == '{')
        {
            objects.emplace_back();
        }
        else if (c == '}' && !objects.empty())
        {
            const ObjectFields object = objects.back();
            objects.pop_back();
            if (object.hasTargetName && object.hasCompletePhase && object.hasDuration)
                ++count;
        }
    }
    return count;
}

TEST(ProfilerTraceParser, CountsEventsInObjectWrappedTrace)
{
    const std::string contents =
        R"({"traceEvents":[{"name":"target","ph":"X","dur":1},{"name":"target","ph":"X","dur":2},{"name":"target","ph":"B","args":{"ph":"X","dur":3}}]})";
    EXPECT_EQ(count_complete_trace_events(contents, "target"), 2u);
}

// Creates an instance with the given backend selection and destroys it again, which is
// all the binding needs: it runs inside instance creation.
//
// `output_path` is only meaningful for the trace backend. Pass one whenever TRACE is
// selected in a process that keeps other instances alive: the file is written at
// last-loader shutdown, so an unconfigured path would drop a ct-profile file into the
// working directory at process exit.
::testing::AssertionResult create_and_destroy(ovphysx_profiler_backend_t selected,
                                              const std::string& output_path = std::string())
{
    std::vector<ovphysx_config_entry_t> entries;
    entries.push_back(ovphysx_config_entry_profiler_backend(selected));
    if (!output_path.empty())
    {
        entries.push_back(ovphysx_config_entry_profiler_output_path(
            ovphysx_string_t{ output_path.c_str(), output_path.size() }));
    }

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries.data();
    args.config_entry_count = static_cast<uint32_t>(entries.size());

    ovphysx_instance_handle_t handle = 0;
    const ovphysx_result_t created = ovphysx_create_instance_with_args(&args, &handle);
    if (created.status != OVPHYSX_API_SUCCESS || handle == 0)
        return ::testing::AssertionFailure() << "Failed to create PhysX instance";

    ovphysx_destroy_instance(handle);
    return ::testing::AssertionSuccess();
}

// Leaves the process in the shipped default state: the selection is global, so a test
// that turned profiling on has to turn it back off for the tests that follow.
class ProfilerBackend : public ::testing::Test
{
protected:
    void SetUp() override
    {
        unset_env(kEnvBackend);
        unset_env(kEnvOutputPath);
        reset_selection();
    }

    void TearDown() override
    {
        unset_env(kEnvBackend);
        unset_env(kEnvOutputPath);
        reset_selection();
    }

    // Best effort by design. The Carbonite settings interface only exists once an
    // instance has been created somewhere in this process, and before that point
    // nothing can have selected a backend, so there is nothing to clear. After the
    // first instance, the startup-only setter is rejected; a create-time entry is
    // what puts the sticky setting back to none for the next case.
    static void reset_selection()
    {
        const ovphysx_result_t set = ovphysx_set_global_config_entry(
            ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NONE));
        if (set.status != OVPHYSX_API_INVALID_STATE)
            return;

        const ovphysx_config_entry_t entry =
            ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NONE);
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        args.config_entries = &entry;
        args.config_entry_count = 1;
        ovphysx_instance_handle_t handle = 0;
        if (ovphysx_create_instance_with_args(&args, &handle).status == OVPHYSX_API_SUCCESS)
            ovphysx_destroy_instance(handle);
    }
};

// Every rigid body's pose (position xyz, then orientation xyzw) read through the session API, flattened
// in prim-path order so two runs compare component for component whatever order the read emits its rows
// in. The falling-boxes scene runs on CPU, so every column is host memory.
bool read_rigid_body_poses(ovphysx_instance_handle_t handle, std::vector<float>& out)
{
    out.clear();
    ovphysx_query_handle_t query = 0;
    if (complete_task(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)).status !=
        OVPHYSX_API_SUCCESS)
        return false;
    path_dictionary_instance_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    bool ok = dictionary != nullptr;
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictionary);
    const ovx_string_or_token_t attributes[] = {
        { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
        { 0, { OVPHYSX_ATTR_ORIENTATION, sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
    };
    ovphysx_read_handle_t read = 0;
    ok = ok && complete_task(handle, ovphysx_read(handle, query, attributes, 2, &read, nullptr)).status == OVPHYSX_API_SUCCESS;
    std::map<std::string, std::array<float, 7>> poses;
    while (ok)
    {
        const ovstage_read_group_t* group = nullptr;
        ok = complete_task(handle, ovphysx_fetch_read_next(handle, read, &group)).status == OVPHYSX_API_SUCCESS;
        if (!ok || !group)
            break;
        const DLTensor* tensor = group->data.tensor_count ? &group->data.tensors[0] : nullptr;
        const uint32_t lanes = tensor ? tensor->dtype.lanes : 0;
        const ovx_primpath_t* paths = nullptr;
        size_t path_count = 0;
        ok = tensor && tensor->data && tensor->device.device_type == kDLCPU && (lanes == 3 || lanes == 4) &&
             ovx_path_dictionary_get_paths(dict, group->prims.list, &paths, &path_count) == OVX_OK;
        const float* values =
            ok ? reinterpret_cast<const float*>(static_cast<const char*>(tensor->data) + tensor->byte_offset) : nullptr;
        for (uint32_t row = 0; ok && row < group->prims.count; ++row)
        {
            const uint32_t index = group->prims.index_map ? group->prims.index_map[row] : group->prims.offset + row;
            ovx_string_t path{};
            ok = index < path_count && ovx_path_dictionary_path_to_string(dict, paths[index], &path) == OVX_OK;
            if (ok)
                std::copy_n(values + row * lanes, lanes,
                            poses[std::string(path.ptr, path.length)].begin() + (lanes == 3 ? 0 : 3));
        }
        ok = complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)).status ==
                 OVPHYSX_API_SUCCESS &&
             ok;
    }
    if (read != 0)
        ok = complete_task(handle, ovphysx_release_read(handle, read)).status == OVPHYSX_API_SUCCESS && ok;
    ok = complete_task(handle, ovphysx_release_query(handle, query)).status == OVPHYSX_API_SUCCESS && ok;
    for (const std::pair<const std::string, std::array<float, 7>>& entry : poses)
        out.insert(out.end(), entry.second.begin(), entry.second.end());
    return ok && !out.empty();
}

// Creates an instance, attaches a falling-boxes stage, steps it a fixed number of times
// and reads the resulting body poses back through the session API. This walks every
// entry point the requirement lists as instrumented.
::testing::AssertionResult run_workflow(const ovphysx_config_entry_t* entries,
                                       uint32_t entry_count,
                                       std::vector<float>& out_poses,
                                       bool exerciseNestedLifecycle = false)
{
    out_poses.clear();

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries;
    args.config_entry_count = entry_count;

    ovphysx_instance_handle_t handle = 0;
    ovphysx_result_t r = ovphysx_create_instance_with_args(&args, &handle);
    if (r.status != OVPHYSX_API_SUCCESS || handle == 0)
        return ::testing::AssertionFailure() << "Failed to create PhysX instance";

    // Called after instance creation so the selected profiler is already bound. This
    // is intentionally outside the simulation workflow's required calls: it proves a
    // formerly-uninstrumented process-level API reaches the same trace.
    if (ovphysx_get_version_string() == nullptr)
    {
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Failed to read the ovphysx version";
    }

    // The first creation binds the selected backend and the final destruction releases
    // it. Exercise another lifecycle entirely inside that epoch when requested, so
    // both lifecycle boundary zones can be observed as complete trace events.
    if (exerciseNestedLifecycle)
    {
        ovphysx_instance_handle_t nested = 0;
        r = ovphysx_create_instance(&nested);
        if (r.status != OVPHYSX_API_SUCCESS || nested == 0)
        {
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Failed to create the nested PhysX instance";
        }
        r = ovphysx_destroy_instance(nested);
        if (r.status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Failed to destroy the nested PhysX instance";
        }
    }

    if (!attach_usd_with_ovstage(handle, "tests/data/boxes_falling_on_groundplane.usda"))
    {
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Failed to attach the test stage";
    }

    // Async step plus explicit wait, so ovphysx_step and ovphysx_wait_task are both
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
        if (enqueued.op_index != 0 && !waitForTaskSuccess(handle, enqueued, 10'000'000'000ULL))
        {
            destroy_ovstage_test_attachments(handle);
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Step " << i << " failed to complete";
        }
    }

    const bool read_ok = read_rigid_body_poses(handle, out_poses);
    destroy_ovstage_test_attachments(handle);
    ovphysx_destroy_instance(handle);

    if (!read_ok)
        return ::testing::AssertionFailure() << "read_rigid_body_poses reported failure";
    if (out_poses.empty())
        return ::testing::AssertionFailure() << "Failed to read body poses back";

    return ::testing::AssertionSuccess();
}

} // namespace

// AC-1: no environment variable and no config entry means no profiler.
TEST_F(ProfilerBackend, DisabledByDefault)
{
    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NONE)
        << "instance creation must not select a backend when nothing asked for one";
}

// AC-2: the typed config entry selects a backend and is observable through the getter.
TEST_F(ProfilerBackend, ConfigEntrySelectsNvtx)
{
    const ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NVTX)
    };

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(entries, 1, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX) << "the config entry must select NVTX";
}

// AC-3: the environment variable selects a backend without any config entry, by name.
TEST_F(ProfilerBackend, EnvironmentVariableSelects)
{
    // The unselected starting point comes from the fixture; it is not re-checked here
    // because reading the setting requires an instance to exist, and this test has to
    // work as the first one in a process. DisabledByDefault covers the default.
    set_env(kEnvBackend, "nvtx");

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX)
        << "OVPHYSX_PROFILER must be written through to /physics/profilerBackend";
}

// AC-3: names are case-insensitive, so a shell that upper-cases does not silently
// disable profiling.
TEST_F(ProfilerBackend, EnvironmentVariableIsCaseInsensitive)
{
    set_env(kEnvBackend, "NVTX");

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX);
}

// AC-3: an unrecognized name contributes nothing. With nothing else selected that
// leaves profiling off.
TEST_F(ProfilerBackend, UnknownEnvironmentValueSelectsNothing)
{
    set_env(kEnvBackend, "tracy");

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NONE)
        << "an unknown backend name must not select anything";
}

// AC-3: and it does not clobber a selection that is already in place. A typo in the
// environment must not silently switch off profiling the config entry asked for --
// which is the case the fixture's clean starting point hides.
TEST_F(ProfilerBackend, UnknownEnvironmentValueLeavesAnExistingSelectionAlone)
{
    const ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NVTX)
    };
    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(entries, 1, poses));
    ASSERT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX);

    set_env(kEnvBackend, "tracy");
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX)
        << "an unknown backend name must leave the sticky selection in place";
}

// AC-1 stickiness and AC-3 symmetry. /physics/profilerBackend outlives the instance
// that set it, so an explicit "none" has to be able to turn it back off; writing only
// the selecting cases would strand the process as enabled.
TEST_F(ProfilerBackend, EnvironmentVariableNoneTurnsOffStickySelection)
{
    const ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NVTX)
    };
    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(entries, 1, poses));
    ASSERT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX)
        << "the config entry must leave the sticky selection in place";

    // Second instance: no config entry, only the explicit environment variable.
    set_env(kEnvBackend, "none");
    ASSERT_TRUE(run_workflow(nullptr, 0, poses));
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NONE)
        << "OVPHYSX_PROFILER=none must turn a sticky selection back off";
}

// REQ-CAPI-PROFILER-002 AC-1: with nothing selected, no Carbonite profiler is bound, so
// neither outcome is reported and the profile zones keep reaching nobody -- the
// behavior of a build without this sink.
TEST_F(ProfilerBackend, CarbBackendStaysUnboundWhenDisabled)
{
    // Capture needs a framework, which means an instance must already exist. The shared
    // one also puts the process in the state the next case cares about: the PhysX
    // runtime is started before the instance under test is created.
    ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create the shared CPU instance";

    const ScopedLogCapture capture;
    ASSERT_TRUE(capture.active()) << "log capture must start";

    ASSERT_TRUE(create_and_destroy(OVPHYSX_PROFILER_BACKEND_NONE));

    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, kPublishing))
        << "no profiler backend may be bound while nothing is selected";
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, kAnotherBackendActive))
        << "with nothing bound there is no backend outcome to report";
}

// REQ-CAPI-PROFILER-002 AC-2 and AC-6: the selection is read before the loader's "runtime
// already started" early return, so an instance that selects a backend in a process
// where an earlier one left it off still binds -- and the loader says which profiler
// the framework actually bound.
//
// Uses TRACE rather than NVTX deliberately. Per AC-9 the NVTX backend is optional at
// build time: a dependency package without carb.profiler-nvtx.static produces a build
// where selecting NVTX warns and binds nothing, which is a supported configuration and
// would fail the assertion below. The trace backend is a shared plugin in the shipped
// plugin set and is always available, so it gives this coverage unconditionally.
TEST_F(ProfilerBackend, CarbBackendBindsOnALaterInstance)
{
    ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create the shared CPU instance";

    const std::string tracePath =
        (std::filesystem::temp_directory_path() / "ovphysx_test_bind_probe.json").string();

    const ScopedLogCapture capture;
    ASSERT_TRUE(capture.active()) << "log capture must start";

    ASSERT_TRUE(create_and_destroy(OVPHYSX_PROFILER_BACKEND_TRACE, tracePath));

    // One of the two outcomes must be reported: either the backend bound, or something
    // else already owned the process profiler. Silence would mean binding never ran,
    // which is the regression this case exists for -- it is what placing the call after
    // the early return looks like from here.
    const bool publishing = ovphysx_log_capture_find(OVPHYSX_LOG_INFO, kPublishing);
    const bool otherBackend = ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, kAnotherBackendActive);
    EXPECT_TRUE(publishing || otherBackend)
        << "selecting a backend on a later instance must still attempt to bind it";

    // In the test binary nothing else binds a Carbonite profiler, so the selected
    // backend is expected to be the one that won.
    EXPECT_FALSE(otherBackend) << "no other Carbonite profiler backend is expected in this process";
}

// REQ-CAPI-PROFILER-002 AC-11: asking for NONE while a backend is bound neither releases it
// nor silences the zones mid-epoch -- an instance still simulating would lose them. The
// instance that asked is told which backend is bound instead.
TEST_F(ProfilerBackend, SelectingNoneWhileBoundKeepsTheBoundBackend)
{
    ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create the shared CPU instance";

    const std::string tracePath =
        (std::filesystem::temp_directory_path() / "ovphysx_test_none_while_bound.json").string();

    // Get a backend bound and keep this instance alive, so the creation below runs
    // inside an open epoch. Which backend ends up bound depends on what an earlier case
    // in this process bound first, and the assertion does not depend on which it is --
    // only that one is bound. The output path matters only if this selection is the one
    // that binds: without it the trace would land in the working directory at exit.
    std::vector<ovphysx_config_entry_t> entries;
    entries.push_back(ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_TRACE));
    entries.push_back(
        ovphysx_config_entry_profiler_output_path(ovphysx_string_t{ tracePath.c_str(), tracePath.size() }));

    ovphysx_create_args boundArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    boundArgs.config_entries = entries.data();
    boundArgs.config_entry_count = static_cast<uint32_t>(entries.size());

    ovphysx_instance_handle_t bound = 0;
    ASSERT_EQ(ovphysx_create_instance_with_args(&boundArgs, &bound).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(bound, 0u);

    {
        const ScopedLogCapture capture;
        ASSERT_TRUE(capture.active()) << "log capture must start";

        ASSERT_TRUE(create_and_destroy(OVPHYSX_PROFILER_BACKEND_NONE));

        EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, kNoneWhileBound))
            << "an instance selecting none while a backend is bound must be told which one is bound";
    }

    ovphysx_destroy_instance(bound);
}

// REQ-CAPI-PROFILER-002 AC-5: binding is idempotent across instances and never turns a
// profiling opt-in into a failed workload.
TEST_F(ProfilerBackend, CarbBackendBindingIsIdempotent)
{
    ASSERT_TRUE(create_and_destroy(OVPHYSX_PROFILER_BACKEND_NVTX));
    ASSERT_TRUE(create_and_destroy(OVPHYSX_PROFILER_BACKEND_NVTX))
        << "a second instance must not fail because the backend is already bound";
    EXPECT_EQ(backend_setting(), OVPHYSX_PROFILER_BACKEND_NVTX);
}

// Same fixture, separate suite so scripts/test_cpp.cmake can give the case below its
// own process.
using ProfilerSideEffects = ProfilerBackend;

// AC-4 and AC-5: the zones are side-effect free. Every instrumented entry point runs in
// both configurations (run_workflow walks them) and the simulation results have to
// match.
//
// Isolated because the unprofiled arm has to be genuinely unprofiled. Binding is
// released only when the last loader shuts down (REQ-CAPI-PROFILER-002 AC-4), and the
// shared process keeps an instance alive throughout, so a backend bound by any earlier
// case stays bound -- and zones record whenever a profiler is bound, whatever this
// instance selected. In that process both arms would be profiled and the comparison
// would prove nothing.
TEST_F(ProfilerSideEffects, EnabledRunMatchesDisabledRun)
{
    std::vector<float> disabled_poses;
    ASSERT_TRUE(run_workflow(nullptr, 0, disabled_poses));

    const ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NVTX)
    };
    std::vector<float> enabled_poses;
    ASSERT_TRUE(run_workflow(entries, 1, enabled_poses));

    ASSERT_EQ(disabled_poses.size(), enabled_poses.size())
        << "the same stage must produce the same number of pose components";
    for (size_t i = 0; i < disabled_poses.size(); ++i)
    {
        EXPECT_NEAR(disabled_poses[i], enabled_poses[i], 1e-5f)
            << "pose component " << i << " differs with profiling enabled";
    }
}

// REQ-CAPI-PROFILER-002 AC-10: the trace backend writes its chrome trace, and it writes it
// when the last instance shuts down rather than earlier.
//
// Runs in its own process (see scripts/test_cpp.cmake). The file is written by
// CARB_PROFILE_SHUTDOWN() on the last-loader path, so any other live instance in the
// process -- the shared CPU instance the other cases use -- would defer the write past
// the end of the test. This case therefore never calls ensureSharedCpuInstance().
TEST(ProfilerTraceBackend, WritesChromeTraceOnLastInstanceShutdown)
{
    const std::filesystem::path tracePath =
        std::filesystem::temp_directory_path() / "ovphysx_test_profiler_trace.json";

    std::error_code ignored;
    std::filesystem::remove(tracePath, ignored);
    ASSERT_FALSE(std::filesystem::exists(tracePath)) << "stale trace from a previous run";

    const std::string tracePathString = tracePath.string();
    const ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_TRACE),
        ovphysx_config_entry_profiler_output_path(
            ovphysx_string_t{ tracePathString.c_str(), tracePathString.size() }),
    };

    std::vector<float> poses;
    ASSERT_TRUE(run_workflow(entries, 2, poses, true));

    // run_workflow destroyed the only instance, so the last-loader shutdown has run.
    ASSERT_TRUE(std::filesystem::exists(tracePath))
        << "the trace backend must write " << tracePathString << " when the last instance shuts down";
    EXPECT_GT(std::filesystem::file_size(tracePath), 0u) << "the trace must not be empty";

    // Enough to prove it is the chrome trace and that our own zones reached it, without
    // pulling in a JSON parser: the format is a JSON array of event objects, and
    // the step entry point is instrumented on every path this workflow takes.
    std::ifstream trace(tracePath, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(trace)), std::istreambuf_iterator<char>());

    // The head of the file, so a failure says what was actually written instead of only
    // that a substring was missing -- gzip output and an empty event array look the same
    // through a find() that returns npos.
    const std::string preview = contents.substr(0, 200);

    // Structural check before content: a chrome trace is a JSON array (or an object
    // with "traceEvents") of event objects carrying a "ph" phase and a "name". Balance
    // the braces and count events rather than pulling in a JSON parser, which is enough
    // to tell a real trace from a truncated or compressed file.
    ASSERT_FALSE(contents.empty()) << "trace is empty";
    const size_t firstNonSpace = contents.find_first_not_of(" \t\r\n");
    ASSERT_NE(firstNonSpace, std::string::npos) << "trace is only whitespace";
    EXPECT_TRUE(contents[firstNonSpace] == '[' || contents[firstNonSpace] == '{')
        << "trace does not start as JSON; file starts: " << preview;

    int depth = 0;
    int minDepth = 0;
    size_t eventCount = 0;
    bool inString = false;
    bool escaped = false;
    for (const char c : contents)
    {
        if (escaped)
        {
            escaped = false;
            continue;
        }
        if (inString)
        {
            if (c == '\\')
                escaped = true;
            else if (c == '"')
                inString = false;
            continue;
        }
        if (c == '"')
            inString = true;
        else if (c == '{' || c == '[')
            ++depth;
        else if (c == '}' || c == ']')
        {
            --depth;
            minDepth = std::min(minDepth, depth);
        }
    }
    EXPECT_EQ(depth, 0) << "trace JSON is unbalanced (truncated write?); file starts: " << preview;
    EXPECT_EQ(minDepth, 0) << "trace JSON closes more than it opens; file starts: " << preview;
    EXPECT_FALSE(inString) << "trace JSON ends inside a string literal";

    for (size_t at = contents.find("\"ph\""); at != std::string::npos; at = contents.find("\"ph\"", at + 1))
    {
        ++eventCount;
    }
    EXPECT_GT(eventCount, 0u) << "no chrome-trace phase fields found; file starts: " << preview;

    // One zone from each of the three groups the docs promise, so this notices a
    // routing regression that silences a whole group rather than only the API zones:
    // the C API entry points, the runtime's host-side work, and the PhysX SDK.
    EXPECT_NE(contents.find("ovphysx_step"), std::string::npos)
        << "the ovphysx entry-point zones did not reach the trace";
    EXPECT_NE(contents.find("ovphysx_get_version_string"), std::string::npos)
        << "a process-level ovphysx entry-point zone did not reach the trace";
    EXPECT_EQ(count_complete_trace_events(contents, "ovphysx_create_instance"), 1u)
        << "the nested instance creation must be a complete trace event";
    EXPECT_EQ(count_complete_trace_events(contents, "ovphysx_destroy_instance"), 2u)
        << "both nested and final instance destruction must be complete trace events";
    EXPECT_NE(contents.find("Physics Load Stage"), std::string::npos)
        << "the runtime's host-side zones did not reach the trace";
    EXPECT_NE(contents.find("Sim."), std::string::npos)
        << "the PhysX SDK zones did not reach the trace (is the capture mask admitting bit 8?)";

    if (!::testing::Test::HasFailure())
    {
        std::filesystem::remove(tracePath, ignored);
    }
}

// REQ-CAPI-PROFILER-002 AC-10: the trace destination is fixed on the plugin before it
// starts, so a second instance in the same epoch asking for a different file does not
// get one and is told so, rather than being left to look for a trace that was never
// going to be written there.
//
// In the isolated process for the same reason the case above is, plus one of its own:
// this needs TRACE to be the backend that actually bound, and in the shared process a
// case that selects NVTX binds it first and holds it for the whole run.
TEST(ProfilerTraceBackend, SecondDestinationInTheSameEpochIsReported)
{
    const std::filesystem::path boundPath =
        std::filesystem::temp_directory_path() / "ovphysx_test_trace_bound_destination.json";
    const std::filesystem::path refusedPath =
        std::filesystem::temp_directory_path() / "ovphysx_test_trace_refused_destination.json";

    std::error_code ignored;
    std::filesystem::remove(boundPath, ignored);
    std::filesystem::remove(refusedPath, ignored);

    const std::string boundPathString = boundPath.string();
    std::vector<ovphysx_config_entry_t> entries;
    entries.push_back(ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_TRACE));
    entries.push_back(ovphysx_config_entry_profiler_output_path(
        ovphysx_string_t{ boundPathString.c_str(), boundPathString.size() }));

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries.data();
    args.config_entry_count = static_cast<uint32_t>(entries.size());

    // Held alive, so the creation below runs inside this epoch instead of starting one.
    ovphysx_instance_handle_t bound = 0;
    ASSERT_EQ(ovphysx_create_instance_with_args(&args, &bound).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(bound, 0u);

    {
        const ScopedLogCapture capture;
        ASSERT_TRUE(capture.active()) << "log capture must start";

        ASSERT_TRUE(create_and_destroy(OVPHYSX_PROFILER_BACKEND_TRACE, refusedPath.string()));

        EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, kTraceDestinationFixed))
            << "a second trace destination in the same epoch must be reported as ignored";
    }

    EXPECT_FALSE(std::filesystem::exists(refusedPath)) << "the refused destination must not receive a trace";

    // Ends the epoch, which writes the trace to the bound destination.
    ovphysx_destroy_instance(bound);

    EXPECT_TRUE(std::filesystem::exists(boundPath))
        << "the trace must be written to the destination that was bound first";
    EXPECT_FALSE(std::filesystem::exists(refusedPath)) << "the refused destination must stay unwritten";

    std::filesystem::remove(boundPath, ignored);
    std::filesystem::remove(refusedPath, ignored);
    ovphysx_set_global_config_entry(ovphysx_config_entry_profiler_backend(OVPHYSX_PROFILER_BACKEND_NONE));
}
