// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx_test_utils.h"
#include "global_test_environment.h"  // shared CPU instance (ensureSharedCpuInstance/sharedCpuInstance)

#include <cstring>
#include <mutex>
#include <string>
#include <vector>

// ============================================================================
// Helpers
// ============================================================================

// Shared instance created once in the first test that needs it.
// We reuse the same instance for all log level tests because Carbonite is
// a singleton -- the log level is global, not per-instance.
static ovphysx_handle_t g_handle = 0;

static void ensureInstance()
{
    if (g_handle != 0)
        return;

    // Alias the process-global shared CPU instance (owned and destroyed by
    // PhysXShutdownEnvironment in test_main.cpp) instead of creating a separate
    // instance. A second instance-owning Environment here destroyed its handle
    // only AFTER PhysXShutdownEnvironment had already run ovphysx_shutdown()
    // (gtest tears environments down in reverse registration order, and the
    // static-init Environment registers before main()'s), so shutdown saw it as
    // a leaked instance.
    ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create shared CPU instance";
    g_handle = sharedCpuInstance();
}

// Clean up the shared instance and restore default log level after all tests.
class LogTestCleanup : public ::testing::Environment
{
public:
    void TearDown() override
    {
        // g_handle aliases the shared CPU instance, which PhysXShutdownEnvironment
        // destroys -- do NOT destroy it here (that would double-destroy and, due
        // to teardown ordering, ran after shutdown). Just drop the alias and
        // restore the default log level so later suites are unaffected.
        g_handle = 0;
        ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
    }
};

static ::testing::Environment* const g_logTestCleanup =
    ::testing::AddGlobalTestEnvironment(new LogTestCleanup());

// ============================================================================
// Log level: ovphysx_set/get_log_level round-trip
// ============================================================================

// ---------------------------------------------------------------------------
// Test: Invalid log level is rejected with no state change
// ---------------------------------------------------------------------------
TEST(LogLevel, InvalidLevelRejectedNoStateChange)
{
    uint32_t original = ovphysx_get_log_level();

    ovphysx_result_t r = ovphysx_set_log_level(999);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "Invalid level should return INVALID_ARGUMENT";

    // Level should remain unchanged after rejection
    EXPECT_EQ(ovphysx_get_log_level(), original)
        << "Invalid level should not change the current level";
}

// ---------------------------------------------------------------------------
// Test: ovphysx_enable_default_log_output does not crash and is callable
// before and after instance creation (smoke test -- actual console output
// cannot be easily verified in a unit test, but we can confirm no error).
// ---------------------------------------------------------------------------
TEST(LogLevel, EnableDefaultLogOutputSmoke)
{
    // Should return SUCCESS before Carbonite init
    ovphysx_result_t r = ovphysx_enable_default_log_output(false);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_enable_default_log_output(true);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    // Should return SUCCESS after Carbonite init
    r = ovphysx_enable_default_log_output(false);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_enable_default_log_output(true);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: Default global log level should be WARNING
// ---------------------------------------------------------------------------
TEST(LogLevel, DefaultIsWarning)
{
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_WARNING));
}

// ---------------------------------------------------------------------------
// Test: set/get round-trip through multiple levels
// ---------------------------------------------------------------------------
TEST(LogLevel, SetGetRoundTrip)
{
    uint32_t original = ovphysx_get_log_level();

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_VERBOSE));

    ovphysx_set_log_level(OVPHYSX_LOG_ERROR);
    EXPECT_EQ(ovphysx_get_log_level(), static_cast<uint32_t>(OVPHYSX_LOG_ERROR));

    ovphysx_set_log_level(original);
}

// ============================================================================
// Log level filtering: verify message suppression at each threshold
//
// Each test sets a threshold, emits test messages at all levels, and checks
// that only messages at or above the threshold are captured.
// ============================================================================

// ---------------------------------------------------------------------------
// Test: WARNING level (default) suppresses INFO and VERBOSE
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Test: INFO level allows INFO but suppresses VERBOSE
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Test: ERROR level suppresses everything except errors
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Test: VERBOSE level allows everything
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Test: NONE level suppresses all messages
// ---------------------------------------------------------------------------
TEST(LogLevel, NoneLevelSuppressesAll)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_NONE);

    ovphysx_result_t r = ovphysx_log_capture_start();
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // Nothing should pass through at NONE threshold
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "[LogTest] ERROR test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "[LogTest] WARNING test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, "[LogTest] INFO test message"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_VERBOSE, "[LogTest] VERBOSE test message"));

    ovphysx_log_capture_stop();

    // Restore default so subsequent tests aren't affected
    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
}

// ---------------------------------------------------------------------------
// Test: Capture count reflects the right number of messages
// ---------------------------------------------------------------------------
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
// Callback registration and broadcasting
//
// These tests verify that user callbacks receive the expected messages,
// that unregistering stops delivery, and that multiple callbacks broadcast.
// ============================================================================

// Thread-safe accumulator for messages received by a test callback.
struct CallbackCapture
{
    std::mutex mutex;
    std::vector<std::pair<uint32_t, std::string>> messages;
};

static void testLogCallback(uint32_t level, const char* message, void* user_data)
{
    auto* cap = static_cast<CallbackCapture*>(user_data);
    std::lock_guard<std::mutex> g(cap->mutex);
    cap->messages.emplace_back(level, message ? message : "");
}

// ---------------------------------------------------------------------------
// Test: Register a callback and verify it receives all four test messages
// ---------------------------------------------------------------------------
TEST(LogCallback, RegisterAndReceiveMessages)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_register_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // Verify callback received one message at each level
    {
        std::lock_guard<std::mutex> g(capture.mutex);
        bool foundError = false, foundWarning = false, foundInfo = false, foundVerbose = false;
        for (const auto& m : capture.messages)
        {
            if (m.first == OVPHYSX_LOG_ERROR && m.second.find("[LogTest] ERROR") != std::string::npos)
                foundError = true;
            if (m.first == OVPHYSX_LOG_WARNING && m.second.find("[LogTest] WARNING") != std::string::npos)
                foundWarning = true;
            if (m.first == OVPHYSX_LOG_INFO && m.second.find("[LogTest] INFO") != std::string::npos)
                foundInfo = true;
            if (m.first == OVPHYSX_LOG_VERBOSE && m.second.find("[LogTest] VERBOSE") != std::string::npos)
                foundVerbose = true;
        }
        EXPECT_TRUE(foundError) << "Callback did not receive ERROR message";
        EXPECT_TRUE(foundWarning) << "Callback did not receive WARNING message";
        EXPECT_TRUE(foundInfo) << "Callback did not receive INFO message";
        EXPECT_TRUE(foundVerbose) << "Callback did not receive VERBOSE message";
    }

    r = ovphysx_unregister_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: After unregister, the callback should receive no more messages
// ---------------------------------------------------------------------------
TEST(LogCallback, UnregisterStopsMessages)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_register_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Unregister immediately
    r = ovphysx_unregister_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Clear any messages received during the brief registration window
    {
        std::lock_guard<std::mutex> g(capture.mutex);
        capture.messages.clear();
    }

    ovphysx_log_emit_test_messages();

    // Capture should be completely empty after unregister
    {
        std::lock_guard<std::mutex> g(capture.mutex);
        EXPECT_TRUE(capture.messages.empty()) << "Callback received messages after unregister";
    }
}

// ---------------------------------------------------------------------------
// Test: Two callbacks registered simultaneously both receive messages
// ---------------------------------------------------------------------------
TEST(LogCallback, MultipleCallbacks)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);

    CallbackCapture capture1, capture2;
    ovphysx_result_t r;

    r = ovphysx_register_log_callback(testLogCallback, &capture1);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_register_log_callback(testLogCallback, &capture2);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // Both callbacks should receive the WARNING test message
    auto hasWarning = [](const CallbackCapture& cap) {
        for (const auto& m : cap.messages)
            if (m.first == OVPHYSX_LOG_WARNING && m.second.find("[LogTest] WARNING") != std::string::npos)
                return true;
        return false;
    };

    {
        std::lock_guard<std::mutex> g1(capture1.mutex);
        EXPECT_TRUE(hasWarning(capture1)) << "Callback 1 didn't receive WARNING";
    }
    {
        std::lock_guard<std::mutex> g2(capture2.mutex);
        EXPECT_TRUE(hasWarning(capture2)) << "Callback 2 didn't receive WARNING";
    }

    r = ovphysx_unregister_log_callback(testLogCallback, &capture1);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_unregister_log_callback(testLogCallback, &capture2);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: Changing log level after a callback is registered takes effect
// ---------------------------------------------------------------------------
TEST(LogCallback, LogLevelChangeAfterRegisterTakesEffect)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    // Start at VERBOSE -- callback should receive all messages
    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_register_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // Verify all levels arrived
    {
        std::lock_guard<std::mutex> g(capture.mutex);
        bool foundInfo = false;
        for (const auto& m : capture.messages)
            if (m.second.find("[LogTest] INFO") != std::string::npos)
                foundInfo = true;
        EXPECT_TRUE(foundInfo) << "At VERBOSE level, callback should receive INFO";
    }

    // Now tighten the level to ERROR while the callback is still registered
    ovphysx_set_log_level(OVPHYSX_LOG_ERROR);

    {
        std::lock_guard<std::mutex> g(capture.mutex);
        capture.messages.clear();
    }

    ovphysx_log_emit_test_messages();

    // Only ERROR should arrive now
    {
        std::lock_guard<std::mutex> g(capture.mutex);
        bool foundError = false, foundWarning = false, foundInfo = false, foundVerbose = false;
        for (const auto& m : capture.messages)
        {
            if (m.second.find("[LogTest] ERROR") != std::string::npos)
                foundError = true;
            if (m.second.find("[LogTest] WARNING") != std::string::npos)
                foundWarning = true;
            if (m.second.find("[LogTest] INFO") != std::string::npos)
                foundInfo = true;
            if (m.second.find("[LogTest] VERBOSE") != std::string::npos)
                foundVerbose = true;
        }
        EXPECT_TRUE(foundError) << "Callback should still receive ERROR";
        EXPECT_FALSE(foundWarning) << "After tightening to ERROR, WARNING should be suppressed";
        EXPECT_FALSE(foundInfo) << "After tightening to ERROR, INFO should be suppressed";
        EXPECT_FALSE(foundVerbose) << "After tightening to ERROR, VERBOSE should be suppressed";
    }

    r = ovphysx_unregister_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
}

// ---------------------------------------------------------------------------
// Test: Registering the same fn+user_data pair twice returns an error
// ---------------------------------------------------------------------------
TEST(LogCallback, DuplicateRegisterReturnsError)
{
    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_register_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Second registration of the same fn+user_data should fail
    r = ovphysx_register_log_callback(testLogCallback, &capture);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT) << "Duplicate register should be rejected";

    // Clean up
    r = ovphysx_unregister_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: Same function with different user_data is allowed (not a duplicate)
// ---------------------------------------------------------------------------
TEST(LogCallback, SameFnDifferentUserDataAllowed)
{
    CallbackCapture capture1, capture2;
    ovphysx_result_t r;

    r = ovphysx_register_log_callback(testLogCallback, &capture1);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Same fn, different user_data -- should succeed
    r = ovphysx_register_log_callback(testLogCallback, &capture2);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Same fn with different user_data should be allowed";

    // Clean up both
    r = ovphysx_unregister_log_callback(testLogCallback, &capture1);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_unregister_log_callback(testLogCallback, &capture2);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: Calling register from within a callback returns an error
// ---------------------------------------------------------------------------

static void tryRegisterFromCallback(uint32_t level, const char* message, void* user_data)
{
    auto* status_out = static_cast<int32_t*>(user_data);
    if (*status_out == -1) // only attempt once
    {
        CallbackCapture dummy;
        ovphysx_result_t r = ovphysx_register_log_callback(testLogCallback, &dummy);
        *status_out = r.status;
    }
}

TEST(LogCallback, RegisterFromCallbackReturnsError)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    int32_t registerStatus = -1; // sentinel: not attempted yet
    ovphysx_result_t r = ovphysx_register_log_callback(tryRegisterFromCallback, &registerStatus);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // The callback attempted to register -- should have been rejected
    EXPECT_EQ(registerStatus, OVPHYSX_API_ERROR)
        << "Calling register from within a callback should return OVPHYSX_API_ERROR";

    r = ovphysx_unregister_log_callback(tryRegisterFromCallback, &registerStatus);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
}

// ---------------------------------------------------------------------------
// Test: Calling unregister from within a callback returns an error
// ---------------------------------------------------------------------------

static void tryUnregisterFromCallback(uint32_t level, const char* message, void* user_data)
{
    auto* status_out = static_cast<int32_t*>(user_data);
    if (*status_out == -1) // only attempt once
    {
        ovphysx_result_t r = ovphysx_unregister_log_callback(tryUnregisterFromCallback, status_out);
        *status_out = r.status;
    }
}

TEST(LogCallback, UnregisterFromCallbackReturnsError)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

    int32_t unregisterStatus = -1; // sentinel: not attempted yet
    ovphysx_result_t r = ovphysx_register_log_callback(tryUnregisterFromCallback, &unregisterStatus);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    // The callback attempted to unregister itself -- should have been rejected
    EXPECT_EQ(unregisterStatus, OVPHYSX_API_ERROR)
        << "Calling unregister from within a callback should return OVPHYSX_API_ERROR";

    // Callback should still be registered since in-callback unregister was rejected
    r = ovphysx_unregister_log_callback(tryUnregisterFromCallback, &unregisterStatus);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
}

// ---------------------------------------------------------------------------
// Test: Registering a NULL callback returns an error
// ---------------------------------------------------------------------------
TEST(LogCallback, NullCallbackReturnsInvalidArgument)
{
    ovphysx_result_t r = ovphysx_register_log_callback(nullptr, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "NULL fn should return INVALID_ARGUMENT";
}

// ---------------------------------------------------------------------------
// Test: Unregistering a callback that was never registered returns an error
// ---------------------------------------------------------------------------
TEST(LogCallback, UnregisterNonexistentReturnsError)
{
    ovphysx_result_t r = ovphysx_unregister_log_callback(testLogCallback, nullptr);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: Callback respects the global log level (ERROR only)
// ---------------------------------------------------------------------------
TEST(LogCallback, CallbackRespectsLogLevel)
{
    ensureInstance();
    ASSERT_NE(g_handle, 0u);

    // Set threshold to ERROR -- only ERROR messages should reach the callback
    ovphysx_set_log_level(OVPHYSX_LOG_ERROR);

    CallbackCapture capture;
    ovphysx_result_t r = ovphysx_register_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    ovphysx_log_emit_test_messages();

    {
        std::lock_guard<std::mutex> g(capture.mutex);
        bool foundError = false, foundWarning = false, foundInfo = false;
        for (const auto& m : capture.messages)
        {
            if (m.second.find("[LogTest] ERROR") != std::string::npos)
                foundError = true;
            if (m.second.find("[LogTest] WARNING") != std::string::npos)
                foundWarning = true;
            if (m.second.find("[LogTest] INFO") != std::string::npos)
                foundInfo = true;
        }
        EXPECT_TRUE(foundError) << "Callback should receive ERROR at ERROR level";
        EXPECT_FALSE(foundWarning) << "Callback should NOT receive WARNING at ERROR level";
        EXPECT_FALSE(foundInfo) << "Callback should NOT receive INFO at ERROR level";
    }

    r = ovphysx_unregister_log_callback(testLogCallback, &capture);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Restore default
    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
}
