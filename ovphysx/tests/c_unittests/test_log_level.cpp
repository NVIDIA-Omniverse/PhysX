// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-STRING-001
 * @covers AC-1 AC-4
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx_test_utils.h"
#include "global_test_environment.h"  // shared CPU instance (ensureSharedCpuInstance/sharedCpuInstance)
#include "test_utilities.h"

#include <cstring>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// Helpers
// ============================================================================

// Shared instance created once in the first test that needs it. All log level
// tests share it because Carbonite is a singleton. The ovphysx source log level
// is process-scoped, not per-instance.
static ovphysx_handle_t g_handle = 0;

static void ensureInstance()
{
    if (g_handle != 0)
        return;

    // Alias the process-global shared CPU instance, owned and destroyed by
    // PhysXShutdownEnvironment in test_main.cpp. A separate instance-owning
    // Environment here would destroy its handle only after ovphysx_shutdown()
    // has run (gtest tears environments down in reverse registration order, and
    // a static-init Environment registers before main()'s), so shutdown would
    // report the instance as leaked.
    ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create shared CPU instance";
    g_handle = sharedCpuInstance();
}

class ScopedLogLevelReset
{
public:
    ~ScopedLogLevelReset()
    {
        (void)ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
    }
};

class ScopedHostLogEnabled
{
public:
    explicit ScopedHostLogEnabled(bool enabled)
        : m_previous(ovphysx_log_get_global_enabled_for_test())
    {
        ovphysx_log_set_global_enabled_for_test(enabled);
    }

    ~ScopedHostLogEnabled()
    {
        ovphysx_log_set_global_enabled_for_test(m_previous);
    }

private:
    bool m_previous;
};

class ScopedNativeLogCapture
{
public:
    ~ScopedNativeLogCapture()
    {
        if (m_active)
            ovphysx_log_capture_stop();
    }

    bool start()
    {
        if (m_active)
            return false;
        m_active = ovphysx_log_capture_start().status == OVPHYSX_API_SUCCESS;
        return m_active;
    }

private:
    bool m_active = false;
};

class ScopedUsdAttachment
{
public:
    explicit ScopedUsdAttachment(ovphysx_handle_t handle) : m_handle(handle)
    {
    }

    ~ScopedUsdAttachment()
    {
        if (m_attached)
            (void)test_utils::destroy_ovstage_test_attachments(m_handle);
    }

    bool attach(const char* path)
    {
        if (m_attached)
            return false;
        m_attached = test_utils::attach_usd_with_ovstage(m_handle, path);
        return m_attached;
    }

private:
    ovphysx_handle_t m_handle;
    bool m_attached = false;
};

// Clean up the shared instance and restore default log level after all tests.
class LogTestCleanup : public ::testing::Environment
{
public:
    void TearDown() override
    {
        // g_handle aliases the shared CPU instance, which PhysXShutdownEnvironment
        // destroys, so it must not be destroyed here. Drop the alias and restore
        // the default log level so later suites are unaffected.
        g_handle = 0;
        ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
    }
};

static ::testing::Environment* const g_logTestCleanup =
    ::testing::AddGlobalTestEnvironment(new LogTestCleanup());

// ============================================================================
// Log level: ovphysx_set/get_log_level round-trip
// ============================================================================

TEST(LogLevel, InvalidLevelRejectedNoStateChange)
{
    uint32_t original = ovphysx_get_log_level();

    ovphysx_result_t r = ovphysx_set_log_level(999);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "Invalid level should return INVALID_ARGUMENT";

    EXPECT_EQ(ovphysx_get_log_level(), original)
        << "Invalid level should not change the current level";
}

// Smoke test. Console output cannot be verified in a unit test, so this only
// checks that the call succeeds before and after instance creation.
TEST(LogLevel, EnableDefaultLogOutputSmoke)
{
    // Before Carbonite init.
    ovphysx_result_t r = ovphysx_enable_default_log_output(false);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_enable_default_log_output(true);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    // After Carbonite init.
    r = ovphysx_enable_default_log_output(false);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_enable_default_log_output(true);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

TEST(LogLevel, DefaultIsWarning)
{
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_WARNING));
}

TEST(LogLevel, SetGetRoundTrip)
{
    uint32_t original = ovphysx_get_log_level();

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_VERBOSE));

    ovphysx_set_log_level(OVPHYSX_LOG_ERROR);
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_ERROR));

    ovphysx_set_log_level(original);
}

TEST(LogLevel, DefaultRestoresWarning)
{
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_DEFAULT).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_WARNING));
}

// ============================================================================
// Log level filtering: verify message suppression at each threshold
//
// Each test sets a threshold, emits test messages at all levels, and checks
// that only messages at or above the threshold are captured.
// ============================================================================

TEST(LogLevel, WarningLevelSuppressesInfoAndVerbose)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);

    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // ERROR and WARNING should pass through
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] ERROR test message"));
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));

    // INFO and VERBOSE should be suppressed
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "[LogTest] INFO test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_VERBOSE, "[LogTest] VERBOSE test message"));

    ovphysx_log_capture_stop();
}

TEST(LogLevel, InfoLevelAllowsInfoSuppressesVerbose)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_INFO);

    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // ERROR, WARNING, and INFO should pass through
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] ERROR test message"));
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "[LogTest] INFO test message"));

    // VERBOSE should be suppressed
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_VERBOSE, "[LogTest] VERBOSE test message"));

    ovphysx_log_capture_stop();
}

TEST(LogLevel, ErrorLevelSuppressesWarningAndBelow)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_ERROR);

    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // Only ERROR should pass through
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] ERROR test message"));

    // WARNING, INFO, VERBOSE should be suppressed
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "[LogTest] INFO test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_VERBOSE, "[LogTest] VERBOSE test message"));

    ovphysx_log_capture_stop();
}

TEST(LogLevel, VerboseLevelAllowsAll)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // All levels should pass through at VERBOSE threshold
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] ERROR test message"));
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "[LogTest] INFO test message"));
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_VERBOSE, "[LogTest] VERBOSE test message"));

    ovphysx_log_capture_stop();
}

TEST(LogLevel, NoneLevelSuppressesAllOvphysxSourceMessages)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_NONE).status, OVPHYSX_API_SUCCESS);
    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ovphysx_log_emit_test_messages();

    // NONE suppresses every ovphysx-source severity, including Carbonite Fatal.
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] ERROR test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "[LogTest] INFO test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_VERBOSE, "[LogTest] VERBOSE test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] FATAL test message"));

    ovphysx_log_capture_stop();

    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_WARNING).status, OVPHYSX_API_SUCCESS);
    r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ovphysx_log_emit_test_messages();
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));
    ovphysx_log_capture_stop();
}

TEST(LogLevel, OwnedSidecarSourceFollowsLevel)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    const ovphysx_string_t invalidPath = ovphysx_cstr("not a valid SdfPath");
    void* ptr = nullptr;
    ScopedLogLevelReset levelGuard;

    {
        ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_INFO).status, OVPHYSX_API_SUCCESS);
        ScopedNativeLogCapture capture;
        ASSERT_TRUE(capture.start());
        ScopedUsdAttachment attachment(g_handle);
        ASSERT_TRUE(attachment.attach("tests/data/minimal_scene.usda"));
        EXPECT_NE(
            ovphysx_get_physx_ptr(g_handle, invalidPath, OVPHYSX_PHYSX_TYPE_SCENE, &ptr).status,
            OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(ovphysx_log_capture_find(
            OVPHYSX_LOG_ERROR, "Internal sidecar: ovphysx_internal_get_physx_ptr could not resolve ObjectKey"));
    }

    {
        ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_NONE).status, OVPHYSX_API_SUCCESS);
        ScopedNativeLogCapture capture;
        ASSERT_TRUE(capture.start());
        ScopedUsdAttachment attachment(g_handle);
        ASSERT_TRUE(attachment.attach("tests/data/minimal_scene.usda"));
        ptr = nullptr;
        EXPECT_NE(
            ovphysx_get_physx_ptr(g_handle, invalidPath, OVPHYSX_PHYSX_TYPE_SCENE, &ptr).status,
            OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(ovphysx_log_capture_find(
            OVPHYSX_LOG_ERROR, "Internal sidecar: ovphysx_internal_get_physx_ptr could not resolve ObjectKey"));
    }
}

TEST(LogLevel, CaptureCountMatchesEmittedMessages)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // Each level should have at least 1 message from emit_test_messages
    EXPECT_GE(ovphysx_log_capture_count(OVPHYSX_LOG_ERROR), 1u);
    EXPECT_GE(ovphysx_log_capture_count(OVPHYSX_LOG_WARNING), 1u);
    EXPECT_GE(ovphysx_log_capture_count(OVPHYSX_LOG_INFO), 1u);
    EXPECT_GE(ovphysx_log_capture_count(OVPHYSX_LOG_VERBOSE), 1u);

    ovphysx_log_capture_stop();
}

// ============================================================================
// Single callback registration, metadata, filtering, and lifecycle
// ============================================================================

struct CallbackCapture
{
    std::mutex mutex;
    struct Message
    {
        ovphysx_log_level_t level;
        std::string message;
        std::string channel;
        double timestamp;
        bool messageTerminated;
        bool channelTerminated;
    };
    std::vector<Message> messages;
};

class ScopedLogCallbackDisable
{
public:
    ~ScopedLogCallbackDisable()
    {
        (void)ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    }
};

static void testLogCallback(
    ovphysx_log_level_t level,
    ovphysx_string_t message,
    ovphysx_string_t channel,
    double timestamp,
    void* user_data)
{
    CallbackCapture* cap = static_cast<CallbackCapture*>(user_data);
    std::lock_guard<std::mutex> g(cap->mutex);
    cap->messages.push_back({
        level,
        std::string(message.ptr ? message.ptr : "", message.length),
        std::string(channel.ptr ? channel.ptr : "", channel.length),
        timestamp,
        message.ptr && message.ptr[message.length] == '\0',
        channel.ptr && channel.ptr[channel.length] == '\0',
    });
}

TEST(LogCallback, RuntimeSourceFollowsObservedSourcePolicyAtNone)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ScopedLogLevelReset levelGuard;
    CallbackCapture capture;
    ScopedLogCallbackDisable callbackGuard;
    const char* const runtimeDiagnostic = "possibly invalid inertia tensor";
    const char* const irrelevantDeformableDiagnostic =
        "InternalScene::updateDeformableTransforms: CUDA context unavailable, skipping.";
    // CPU instances never create volume deformables (GPU-pipeline gate), so the
    // sim-output CUDA-unavailable warning is not reachable here. The load-time
    // GPU-only error is the observable proof that the deformable fixture ran.
    const char* const deformableGpuOnlyDiagnostic = "Deformable Body feature is only supported on GPU";
    std::string baselineChannel;
    std::string noneChannel;
    bool baselineFound = false;
    bool noneFound = false;
    bool irrelevantDeformableWarningFound = false;
    bool noneDeformableGpuOnlyFound = false;

    ASSERT_EQ(
        ovphysx_set_log_callback(OVPHYSX_LOG_VERBOSE, nullptr, testLogCallback, &capture).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_WARNING).status, OVPHYSX_API_SUCCESS);
    {
        ScopedUsdAttachment attachment(g_handle);
        ASSERT_TRUE(attachment.attach("tests/data/minimal_scene.usda"));
        ASSERT_EQ(ovphysx_step_sync(g_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    }
    {
        std::lock_guard<std::mutex> guard(capture.mutex);
        for (const CallbackCapture::Message& message : capture.messages)
        {
            // Keep the first match: the NONE pass below breaks on its first
            // match, so both channels must be sampled the same way.
            if (!baselineFound && message.level == OVPHYSX_LOG_WARNING &&
                message.message.find(runtimeDiagnostic) != std::string::npos)
            {
                baselineFound = true;
                baselineChannel = message.channel;
            }
            if (message.message.find(irrelevantDeformableDiagnostic) != std::string::npos)
                irrelevantDeformableWarningFound = true;
        }
        capture.messages.clear();
    }
    ASSERT_TRUE(baselineFound);
    EXPECT_FALSE(irrelevantDeformableWarningFound);
    const bool controlledSource =
        baselineChannel == "omni_physx_sdk" ||
        baselineChannel == "omni.physx" ||
        baselineChannel == "ovphysx_internal";

    bool deformableCudaWarningFound = false;
    {
        ScopedUsdAttachment attachment(g_handle);
        ASSERT_TRUE(attachment.attach("tests/data/volume_deformable_simple.usda"));
        ASSERT_EQ(ovphysx_step_sync(g_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    }
    {
        std::lock_guard<std::mutex> guard(capture.mutex);
        for (const CallbackCapture::Message& message : capture.messages)
        {
            if (message.message.find(irrelevantDeformableDiagnostic) != std::string::npos)
                deformableCudaWarningFound = true;
        }
        capture.messages.clear();
    }
    EXPECT_FALSE(deformableCudaWarningFound);

    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_NONE).status, OVPHYSX_API_SUCCESS);
    {
        ScopedUsdAttachment attachment(g_handle);
        ASSERT_TRUE(attachment.attach("tests/data/minimal_scene.usda"));
        ASSERT_EQ(ovphysx_step_sync(g_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    }
    {
        ScopedUsdAttachment attachment(g_handle);
        ASSERT_TRUE(attachment.attach("tests/data/volume_deformable_simple.usda"));
        ASSERT_EQ(ovphysx_step_sync(g_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    }
    {
        std::lock_guard<std::mutex> guard(capture.mutex);
        for (const CallbackCapture::Message& message : capture.messages)
        {
            if (message.level == OVPHYSX_LOG_WARNING &&
                message.message.find(runtimeDiagnostic) != std::string::npos)
            {
                noneFound = true;
                noneChannel = message.channel;
            }
            if (message.channel == "omni_physx_sdk" ||
                message.channel == "omni.physx" ||
                message.channel == "ovphysx_internal")
            {
                if (message.message.find(deformableGpuOnlyDiagnostic) != std::string::npos ||
                    message.message.find(irrelevantDeformableDiagnostic) != std::string::npos)
                {
                    noneDeformableGpuOnlyFound = true;
                }
            }
        }
    }
    EXPECT_EQ(noneFound, !controlledSource);
    if (noneFound)
    {
        EXPECT_EQ(noneChannel, baselineChannel);
    }
    EXPECT_FALSE(noneDeformableGpuOnlyFound)
        << "NONE forwarded a deformable diagnostic from a named target source";
}

TEST(LogCallback, WarningDoesNotOverrideHostGlobalDisable)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ScopedLogLevelReset levelGuard;

    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_WARNING).status, OVPHYSX_API_SUCCESS);
    {
        ScopedHostLogEnabled hostDisabled(false);
        ASSERT_FALSE(ovphysx_log_get_global_enabled_for_test());
        ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_WARNING).status, OVPHYSX_API_SUCCESS);
        ASSERT_FALSE(ovphysx_log_get_global_enabled_for_test())
            << "ovphysx_set_log_level(WARNING) re-enabled process-global logging";
    }
}

TEST(LogCallback, SetReceivesMetadataAndNullTerminatedStrings)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);

    {
        std::lock_guard<std::mutex> g(capture.mutex);
        bool foundError = false, foundWarning = false, foundInfo = false, foundVerbose = false;
        for (const CallbackCapture::Message& message : capture.messages)
        {
            EXPECT_GT(message.timestamp, 0.0);
            EXPECT_FALSE(message.channel.empty());
            EXPECT_TRUE(message.messageTerminated);
            EXPECT_TRUE(message.channelTerminated);
            if (message.level == OVPHYSX_LOG_ERROR && message.message.find("[LogTest] ERROR") != std::string::npos)
                foundError = true;
            if (message.level == OVPHYSX_LOG_WARNING && message.message.find("[LogTest] WARNING") != std::string::npos)
                foundWarning = true;
            if (message.level == OVPHYSX_LOG_INFO && message.message.find("[LogTest] INFO") != std::string::npos)
                foundInfo = true;
            if (message.level == OVPHYSX_LOG_VERBOSE && message.message.find("[LogTest] VERBOSE") != std::string::npos)
                foundVerbose = true;
        }
        EXPECT_TRUE(foundError);
        EXPECT_TRUE(foundWarning);
        EXPECT_TRUE(foundInfo);
        EXPECT_TRUE(foundVerbose);
    }

    r = ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

TEST(LogCallback, SetReplacesAndNullDisables)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    CallbackCapture first;
    CallbackCapture second;
    ovphysx_result_t r = ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, testLogCallback, &first);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;
    r = ovphysx_set_log_callback(OVPHYSX_LOG_VERBOSE, nullptr, testLogCallback, &second);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);

    {
        std::lock_guard<std::mutex> firstLock(first.mutex);
        EXPECT_TRUE(first.messages.empty());
    }
    {
        std::lock_guard<std::mutex> secondLock(second.mutex);
        EXPECT_FALSE(second.messages.empty());
    }

    r = ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    {
        std::lock_guard<std::mutex> secondLock(second.mutex);
        second.messages.clear();
    }
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    {
        std::lock_guard<std::mutex> secondLock(second.mutex);
        EXPECT_TRUE(second.messages.empty());
    }
}

TEST(LogCallback, ChannelFilterLongestPrefixWins)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);
    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);

    std::string emittedChannel;
    {
        std::lock_guard<std::mutex> lock(capture.mutex);
        ASSERT_FALSE(capture.messages.empty());
        for (const CallbackCapture::Message& message : capture.messages)
        {
            if (message.message.find("[LogTest]") != std::string::npos)
            {
                emittedChannel = message.channel;
                break;
            }
        }
        ASSERT_FALSE(emittedChannel.empty());
        capture.messages.clear();
    }

    const std::string shortPrefix = emittedChannel.substr(0, 1);
    std::string filterText =
        " " + shortPrefix + " = none, " + emittedChannel + " = none, " +
        emittedChannel + " = VeRbOsE ";
    const ovphysx_string_t filter = {filterText.data(), filterText.size()};
    r = ovphysx_set_log_callback(OVPHYSX_LOG_NONE, &filter, testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    filterText.assign(filterText.size(), 'X');
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    {
        std::lock_guard<std::mutex> lock(capture.mutex);
        ASSERT_FALSE(capture.messages.empty());
        bool foundError = false;
        bool foundWarning = false;
        bool foundInfo = false;
        bool foundVerbose = false;
        for (const CallbackCapture::Message& message : capture.messages)
        {
            EXPECT_EQ(message.channel, emittedChannel);
            if (message.message.find("[LogTest] ERROR") != std::string::npos)
                foundError = true;
            if (message.message.find("[LogTest] WARNING") != std::string::npos)
                foundWarning = true;
            if (message.message.find("[LogTest] INFO") != std::string::npos)
                foundInfo = true;
            if (message.message.find("[LogTest] VERBOSE") != std::string::npos)
                foundVerbose = true;
        }
        EXPECT_TRUE(foundError);
        EXPECT_TRUE(foundWarning);
        EXPECT_TRUE(foundInfo);
        EXPECT_TRUE(foundVerbose);
    }
    r = ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

struct ReentrantStatuses
{
    int32_t setCallback = -1;
    int32_t setLevel = -1;
    int32_t defaultOutput = -1;
    int32_t flush = -1;
    int32_t shutdown = -1;
};

static void tryReentrantCalls(
    ovphysx_log_level_t,
    ovphysx_string_t,
    ovphysx_string_t,
    double,
    void* user_data)
{
    ReentrantStatuses* statuses = static_cast<ReentrantStatuses*>(user_data);
    if (statuses->setCallback == -1)
    {
        statuses->setCallback = ovphysx_set_log_callback(
            OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr).status;
        statuses->setLevel = ovphysx_set_log_level(OVPHYSX_LOG_ERROR).status;
        statuses->defaultOutput = ovphysx_enable_default_log_output(false).status;
        statuses->flush = ovphysx_flush_log(0).status;
        statuses->shutdown = ovphysx_shutdown().status;
    }
}

TEST(LogCallback, ReentrantConfigurationReturnsError)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);
    ReentrantStatuses statuses;
    ovphysx_result_t r = ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, tryReentrantCalls, &statuses);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(statuses.setCallback, OVPHYSX_API_ERROR);
    EXPECT_EQ(statuses.setLevel, OVPHYSX_API_ERROR);
    EXPECT_EQ(statuses.defaultOutput, OVPHYSX_API_ERROR);
    EXPECT_EQ(statuses.flush, OVPHYSX_API_ERROR);
    EXPECT_EQ(statuses.shutdown, OVPHYSX_API_ERROR);
    r = ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

TEST(LogCallback, InvalidFilterPreservesRegistration)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);
    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;
    const ovphysx_string_t invalidFilter = OVPHYSX_LITERAL("missing_level");
    r = ovphysx_set_log_callback(OVPHYSX_LOG_VERBOSE, &invalidFilter, testLogCallback, &capture);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    {
        std::lock_guard<std::mutex> lock(capture.mutex);
        EXPECT_FALSE(capture.messages.empty());
    }
    r = ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
}

struct BlockingCallbackState
{
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false;
    bool release = false;
    std::atomic<int32_t> active{0};
    std::atomic<int32_t> maxActive{0};
};

static void blockingLogCallback(
    ovphysx_log_level_t,
    ovphysx_string_t,
    ovphysx_string_t,
    double,
    void* user_data)
{
    BlockingCallbackState* state = static_cast<BlockingCallbackState*>(user_data);
    const int32_t active = state->active.fetch_add(1) + 1;
    int32_t previousMax = state->maxActive.load();
    while (active > previousMax && !state->maxActive.compare_exchange_weak(previousMax, active))
    {
    }
    std::unique_lock<std::mutex> lock(state->mutex);
    state->entered = true;
    state->cv.notify_all();
    state->cv.wait(lock, [state] { return state->release; });
    state->active.fetch_sub(1);
}

TEST(LogCallback, ConcurrentProducersAreSerializedAndFlushIsBounded)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status, OVPHYSX_API_SUCCESS);
    BlockingCallbackState state;
    ASSERT_EQ(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, blockingLogCallback, &state).status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;

    std::thread first([] { ovphysx_log_emit_test_messages(); });
    bool entered = false;
    {
        std::unique_lock<std::mutex> lock(state.mutex);
        entered = state.cv.wait_for(
            lock, std::chrono::seconds(5), [&state] { return state.entered; });
        if (!entered)
            state.release = true;
    }
    if (!entered)
    {
        state.cv.notify_all();
        first.join();
        FAIL() << "Callback did not enter within five seconds";
        return;
    }
    std::thread second([] { ovphysx_log_emit_test_messages(); });

    EXPECT_EQ(ovphysx_flush_log(0).status, OVPHYSX_API_TIMEOUT);
    const ovphysx_string_t timeoutError = ovphysx_get_last_error();
    EXPECT_NE(std::string(timeoutError.ptr, timeoutError.length).find("Timed out"), std::string::npos);
    EXPECT_EQ(ovphysx_flush_log(0).status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(ovphysx_flush_log(1000000).status, OVPHYSX_API_TIMEOUT);
    EXPECT_EQ(state.maxActive.load(), 1);

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.release = true;
    }
    state.cv.notify_all();
    first.join();
    second.join();
    EXPECT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(state.maxActive.load(), 1);
}

TEST(LogCallback, ReplacementWaitsForAcceptedCallback)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status, OVPHYSX_API_SUCCESS);
    BlockingCallbackState state;
    ASSERT_EQ(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, nullptr, blockingLogCallback, &state).status, OVPHYSX_API_SUCCESS);
    ScopedLogCallbackDisable callbackGuard;

    std::thread producer([] { ovphysx_log_emit_test_messages(); });
    bool entered = false;
    {
        std::unique_lock<std::mutex> lock(state.mutex);
        entered = state.cv.wait_for(
            lock, std::chrono::seconds(5), [&state] { return state.entered; });
        if (!entered)
            state.release = true;
    }
    if (!entered)
    {
        state.cv.notify_all();
        producer.join();
        FAIL() << "Callback did not enter within five seconds";
        return;
    }

    std::promise<void> replacementStarted;
    std::future<void> replacementStartedFuture = replacementStarted.get_future();
    std::future<ovphysx_result_t> replacement = std::async(std::launch::async, [&replacementStarted] {
        replacementStarted.set_value();
        return ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    });
    replacementStartedFuture.wait();
    EXPECT_EQ(replacement.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.release = true;
    }
    state.cv.notify_all();
    producer.join();
    EXPECT_EQ(replacement.get().status, OVPHYSX_API_SUCCESS);
}

TEST(LogCallback, LengthLimitedFilterAndNullPointerValidation)
{
    const char filterBytes[] = { 'o', 'm', 'n', 'i', '=', 'n', 'o', 'n', 'e', 'X' };
    const ovphysx_string_t filter = { filterBytes, 9 };
    CallbackCapture capture;
    ScopedLogCallbackDisable callbackGuard;
    EXPECT_EQ(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, &filter, testLogCallback, &capture).status, OVPHYSX_API_SUCCESS);

    const ovphysx_string_t invalid = { nullptr, 1 };
    EXPECT_EQ(ovphysx_set_log_callback(
        OVPHYSX_LOG_VERBOSE, &invalid, testLogCallback, &capture).status,
        OVPHYSX_API_INVALID_ARGUMENT);
}
