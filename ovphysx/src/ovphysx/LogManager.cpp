// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// LogManager -- global log level, user callback registration, and test capture.
//
// Manages a single Logger2 for user callbacks and a separate one for test
// capture. Both are registered with Carbonite's ILogging when available.

#include "LogManager.hpp"
#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <carb/logging/Logger.h>
#include <carb/logging/Log.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ============================================================================
// Carbonite level mapping (shared between this file and CarboniteLoader)
// ============================================================================

namespace
{

// Carbonite uses higher = more severe:
// kLevelVerbose(-2) < kLevelInfo(-1) < kLevelWarn(0) < kLevelError(1) < kLevelFatal(2)

int32_t toCarbLogLevel(uint32_t ovphysxLevel)
{
    switch (ovphysxLevel)
    {
        case OVPHYSX_LOG_NONE:    return carb::logging::kLevelFatal;
        case OVPHYSX_LOG_ERROR:   return carb::logging::kLevelError;
        case OVPHYSX_LOG_WARNING: return carb::logging::kLevelWarn;
        case OVPHYSX_LOG_INFO:    return carb::logging::kLevelInfo;
        case OVPHYSX_LOG_VERBOSE: return carb::logging::kLevelVerbose;
        default:                  return carb::logging::kLevelWarn;
    }
}

uint32_t fromCarbLogLevel(int32_t carbLevel)
{
    switch (carbLevel)
    {
        case carb::logging::kLevelFatal:   return OVPHYSX_LOG_ERROR;   // NONE mapped to Fatal; round-trip as ERROR
        case carb::logging::kLevelError:   return OVPHYSX_LOG_ERROR;
        case carb::logging::kLevelWarn:    return OVPHYSX_LOG_WARNING;
        case carb::logging::kLevelInfo:    return OVPHYSX_LOG_INFO;
        case carb::logging::kLevelVerbose: return OVPHYSX_LOG_VERBOSE;
        default:                           return OVPHYSX_LOG_VERBOSE;
    }
}

// ============================================================================
// Global state
// ============================================================================

std::atomic<uint32_t> g_logLevel{OVPHYSX_LOG_WARNING};
// Never reset to false: Carbonite is process-lifetime once initialized
// (CarboniteLoader::g_bootstrapDone follows the same pattern).  This means
// getLogging() remains valid for the rest of the process, so callers of
// ovphysx_set_log_level() after init can safely update the threshold.
std::atomic<bool> g_carboniteReady{false};
// Controls whether Carbonite's built-in console logger is active.
// True by default (matches Carbonite's initial state); user may disable to
// avoid doubled output when a custom callback also writes to the console.
// Stored eagerly and applied when Carbonite initializes (or immediately if
// already initialized).  Uses exchange() to detect actual transitions and
// avoid double add/remove of the default logger.
std::atomic<bool> g_defaultOutputEnabled{true};

// ============================================================================
// User callback broadcasting
// ============================================================================

struct CallbackEntry
{
    ovphysx_log_fn fn;
    void* user_data;
};

std::mutex g_callbackMutex;
std::vector<CallbackEntry> g_callbacks;      // guarded by g_callbackMutex
std::atomic<uint32_t> g_dispatchCount{0};    // total in-flight dispatches across all threads;
                                             // used by unregister to spin-wait until safe
thread_local bool t_inDispatch = false;      // per-thread flag to detect same-thread re-entrancy;
                                             // rejects register/unregister called from within a callback

struct UserCallbackLogger : public carb::logging::Logger2
{
    void handleMessage(const carb::logging::LogMessage& msg) override
    {
        uint32_t level = fromCarbLogLevel(msg.level);
        // Snapshot the callback list under the lock, then release before
        // invoking callbacks.  This prevents deadlock if a callback
        // (directly or indirectly) triggers another CARB_LOG_* call that
        // re-enters handleMessage() on the same thread.
        //
        // g_dispatchCount is incremented while holding the mutex so that
        // ovphysx_unregister_log_callback() (which spin-waits on the
        // counter after releasing the mutex) cannot observe count==0
        // between the snapshot copy and the first callback invocation.
        std::vector<CallbackEntry> snapshot;
        {
            std::lock_guard<std::mutex> g(g_callbackMutex);
            snapshot = g_callbacks;
            g_dispatchCount.fetch_add(1, std::memory_order_release);
        }

        bool prevInDispatch = t_inDispatch;
        t_inDispatch = true;

        for (const auto& cb : snapshot)
        {
            cb.fn(level, msg.message, cb.user_data);
        }

        t_inDispatch = prevInDispatch;
        g_dispatchCount.fetch_sub(1, std::memory_order_release);
    }
};

static UserCallbackLogger g_userLogger;
static bool g_userLoggerRegistered = false;

void ensureUserLoggerRegistered()
{
    // Must be called with g_callbackMutex held or from single-threaded context.
    if (!g_userLoggerRegistered && g_carboniteReady.load(std::memory_order_acquire))
    {
        if (auto* logging = carb::logging::getLogging())
        {
            logging->addLogger(&g_userLogger);
            g_userLoggerRegistered = true;
        }
    }
}

void ensureUserLoggerUnregistered()
{
    // Must be called with g_callbackMutex held or from single-threaded context.
    if (g_userLoggerRegistered)
    {
        if (auto* logging = carb::logging::getLogging())
            logging->removeLogger(&g_userLogger);
        g_userLoggerRegistered = false;
    }
}

// ============================================================================
// Test capture Logger2
// ============================================================================

struct LogCaptureLogger : public carb::logging::Logger2
{
    void handleMessage(const carb::logging::LogMessage& msg) override
    {
        std::lock_guard<std::mutex> g(m_mutex);
        uint32_t level = fromCarbLogLevel(msg.level);
        m_messages[level].push_back(msg.message);
    }

    bool find(uint32_t level, const char* substring) const
    {
        std::lock_guard<std::mutex> g(m_mutex);
        auto it = m_messages.find(level);
        if (it == m_messages.end()) return false;
        for (const auto& m : it->second)
            if (m.find(substring) != std::string::npos)
                return true;
        return false;
    }

    uint32_t count(uint32_t level) const
    {
        std::lock_guard<std::mutex> g(m_mutex);
        auto it = m_messages.find(level);
        return it != m_messages.end() ? static_cast<uint32_t>(it->second.size()) : 0;
    }

    void clear()
    {
        std::lock_guard<std::mutex> g(m_mutex);
        m_messages.clear();
    }

private:
    mutable std::mutex m_mutex;
    std::unordered_map<uint32_t, std::vector<std::string>> m_messages;
};

static LogCaptureLogger* g_captureLogger = nullptr;

} // anonymous namespace

// ============================================================================
// Internal API (called by CarboniteLoader)
// ============================================================================

namespace ovphysx
{

void onCarboniteLoggingReady()
{
    g_carboniteReady.store(true, std::memory_order_release);

    if (auto* logging = carb::logging::getLogging())
    {
        // Apply the stored log level threshold
        logging->setLevelThreshold(toCarbLogLevel(g_logLevel.load(std::memory_order_acquire)));

        // Apply stored default-output preference (user may have called
        // ovphysx_enable_default_log_output(false) before Carbonite init).
        // Carbonite starts with the default logger registered, which matches
        // g_defaultOutputEnabled == true, so we only need to act when false.
        if (!g_defaultOutputEnabled.load(std::memory_order_acquire))
        {
            if (auto* defLogger = logging->getDefaultLogger())
                logging->removeLogger(defLogger->getLogger());
        }
    }

    // Register user callback logger if any callbacks were registered pre-init
    std::lock_guard<std::mutex> g(g_callbackMutex);
    if (!g_callbacks.empty())
        ensureUserLoggerRegistered();
}

} // namespace ovphysx

// ============================================================================
// Public C API -- Log level
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_set_log_level(uint32_t level)
{
    if (level > OVPHYSX_LOG_NONE)
    {
        CARB_LOG_WARN("ovphysx_set_log_level: invalid level %u (max %u); level not changed",
                      static_cast<unsigned>(level), static_cast<unsigned>(OVPHYSX_LOG_NONE));
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Level out of range (0-4); no change applied");
    }

    g_logLevel.store(level, std::memory_order_release);

    // If Carbonite is already initialized, apply immediately
    if (g_carboniteReady.load(std::memory_order_acquire))
    {
        if (auto* logging = carb::logging::getLogging())
            logging->setLevelThreshold(toCarbLogLevel(level));
    }

    return success();
}

extern "C" OVPHYSX_API uint32_t ovphysx_get_log_level(void)
{
    return g_logLevel.load(std::memory_order_acquire);
}

// ============================================================================
// Public C API -- Default console output
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_enable_default_log_output(bool enable)
{
    // exchange() returns the previous value; only act on actual transitions
    bool wasEnabled = g_defaultOutputEnabled.exchange(enable, std::memory_order_acq_rel);

    if (wasEnabled != enable && g_carboniteReady.load(std::memory_order_acquire))
    {
        auto* logging = carb::logging::getLogging();
        if (logging)
        {
            auto* defLogger = logging->getDefaultLogger();
            if (defLogger)
            {
                if (enable)
                    logging->addLogger(defLogger->getLogger());
                else
                    logging->removeLogger(defLogger->getLogger());
            }
        }
    }

    return success();
}

// ============================================================================
// Public C API -- Callback registration
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_register_log_callback(ovphysx_log_fn fn, void* user_data)
{
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot register log callback from within a callback");

    if (!fn)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "fn must not be NULL");

    std::lock_guard<std::mutex> g(g_callbackMutex);

    // Reject duplicate registration of the same fn+user_data pair
    for (const auto& cb : g_callbacks)
    {
        if (cb.fn == fn && cb.user_data == user_data)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Callback already registered with the same fn and user_data");
    }

    g_callbacks.push_back({ fn, user_data });
    ensureUserLoggerRegistered();
    return success();
}

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_unregister_log_callback(ovphysx_log_fn fn, void* user_data)
{
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot unregister log callback from within a callback");

    bool found = false;
    {
        std::lock_guard<std::mutex> g(g_callbackMutex);
        for (auto it = g_callbacks.begin(); it != g_callbacks.end(); ++it)
        {
            if (it->fn == fn && it->user_data == user_data)
            {
                g_callbacks.erase(it);
                if (g_callbacks.empty())
                    ensureUserLoggerUnregistered();
                found = true;
                break;
            }
        }
    }

    if (!found)
        return set_error(OVPHYSX_API_ERROR, "Callback not found");

    // Spin-wait for in-flight dispatches to complete.  After this loop the
    // callback is guaranteed to not be running on any thread, so the caller
    // may safely destroy the callback context.  The mutex is NOT held here
    // to avoid deadlock when a callback (or code it calls) re-enters
    // handleMessage() on another thread and tries to take the mutex for its
    // snapshot.
    while (g_dispatchCount.load(std::memory_order_acquire) > 0)
        std::this_thread::yield();

    return success();
}

// ============================================================================
// Public C API -- Test capture
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_log_capture_start(void)
{
    if (g_captureLogger)
        return success(); // already capturing

    auto* logging = carb::logging::getLogging();
    if (!logging)
        return set_error(OVPHYSX_API_ERROR, "Carbonite logging not available");

    g_captureLogger = new LogCaptureLogger();
    logging->addLogger(g_captureLogger);
    return success();
}

extern "C" OVPHYSX_API void ovphysx_log_capture_stop(void)
{
    if (!g_captureLogger)
        return;

    if (auto* logging = carb::logging::getLogging())
        logging->removeLogger(g_captureLogger);

    delete g_captureLogger;
    g_captureLogger = nullptr;
}

extern "C" OVPHYSX_API bool ovphysx_log_capture_find(uint32_t level, const char* substring)
{
    if (!g_captureLogger || !substring)
        return false;
    return g_captureLogger->find(level, substring);
}

extern "C" OVPHYSX_API uint32_t ovphysx_log_capture_count(uint32_t level)
{
    if (!g_captureLogger)
        return 0;
    return g_captureLogger->count(level);
}

extern "C" OVPHYSX_API void ovphysx_log_emit_test_messages(void)
{
    CARB_LOG_ERROR("[LogTest] ERROR test message");
    CARB_LOG_WARN("[LogTest] WARNING test message");
    CARB_LOG_INFO("[LogTest] INFO test message");
    CARB_LOG_VERBOSE("[LogTest] VERBOSE test message");
}
