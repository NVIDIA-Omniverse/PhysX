// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-STRING-001
 * @covers AC-4
 *
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-2
 */

// LogManager: global log level, one application callback, and test capture.
//
// Manages a single Logger2 for user callbacks and a separate one for test
// capture. Both are registered with Carbonite's ILogging when available.

#include "LogManager.hpp"
#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <carb/logging/Logger.h>
#include <carb/logging/Log.h>
#include <omni/log/ILog.h>
#include <omni/log/LogChannel.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Ovruntime's default channel is defined in PhysX.cpp. This TU cannot use
// OMNI_LOG_DEFAULT_CHANNEL: ovphysx is built with CARB_STATIC_PLUGIN_BUILD and
// CARB_PLUGIN_CLIENT_NAME=omni_physx_sdk, so that macro expands to
// kDefaultChannel_omni_physx_sdk. Ovruntime is not, so its default channel is
// kDefaultChannel ("omni.physx").
OMNI_LOG_DECLARE_CHANNEL(kDefaultChannel)

// ============================================================================
// Carbonite level mapping
// ============================================================================

namespace
{

// Carbonite uses higher = more severe:
// kLevelVerbose(-2) < kLevelInfo(-1) < kLevelWarn(0) < kLevelError(1) < kLevelFatal(2)

int32_t toCarbLogLevel(uint32_t ovphysxLevel)
{
    switch (ovphysxLevel)
    {
        case OVPHYSX_LOG_DEFAULT: return carb::logging::kLevelWarn;
        case OVPHYSX_LOG_ERROR:   return carb::logging::kLevelError;
        case OVPHYSX_LOG_WARNING: return carb::logging::kLevelWarn;
        case OVPHYSX_LOG_INFO:    return carb::logging::kLevelInfo;
        case OVPHYSX_LOG_VERBOSE: return carb::logging::kLevelVerbose;
        default:                  return carb::logging::kLevelWarn;
    }
}

void setPhysxRuntimeLogLevel(int32_t level)
{
    kDefaultChannel.level = level;
}

void applyCarboniteLogLevel(carb::logging::ILogging* logging, uint32_t level)
{
    const char* const controlledSources[] = {
        g_carbClientName.c_str(),
        kDefaultChannel.name, // ovruntime's default channel, "omni.physx"
        "ovphysx_internal",
    };
    for (const char* source : controlledSources)
    {
        if (level == OVPHYSX_LOG_NONE)
        {
            logging->setLogEnabledForSource(
                source, carb::logging::LogSettingBehavior::eOverride, false);
            continue;
        }

        logging->setLevelThresholdForSource(
            source,
            carb::logging::LogSettingBehavior::eOverride,
            toCarbLogLevel(level));
        logging->setLogEnabledForSource(
            source, carb::logging::LogSettingBehavior::eInherit, true);
    }

    // registerSource() owns kDefaultChannel.level through
    // setPhysxRuntimeLogLevel(). Do not assign it here: Carbonite publishes the
    // effective level, including its disabled sentinel when process-global
    // logging is disabled. Overwriting that value would let this library
    // re-enable OMNI_LOG_* call-site gates behind the host's back.
}

uint32_t fromCarbLogLevel(int32_t carbLevel)
{
    switch (carbLevel)
    {
        case carb::logging::kLevelFatal:   return OVPHYSX_LOG_ERROR;
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

// The public logging contract is process-wide, so the level, sole callback,
// and Carbonite logger registration are necessarily module state. Lock order:
// readiness/control, then user-logger registration, then callback state, then
// the registration drain and dispatch acceptance mutexes. Never hold callback
// state or registration drain locks while invoking user code or waiting for a
// prior registration to drain.

std::atomic<uint32_t> g_logLevel{OVPHYSX_LOG_WARNING};
// Never reset to false: Carbonite is process-lifetime once initialized
// (CarboniteLoader::g_bootstrapDone follows the same pattern). getLogging()
// therefore remains valid for the rest of the process, so callers of
// ovphysx_set_log_level() after init can safely update the threshold.
std::atomic<bool> g_carboniteReady{false};
// Controls whether Carbonite's built-in console logger is active. True by
// default, matching Carbonite's initial state. The user may disable it to
// avoid doubled output when a custom callback also writes to the console.
// Stored eagerly and applied when Carbonite initializes (or immediately if
// already initialized).
bool g_defaultOutputEnabled = true;

// ============================================================================
// Application callback
// ============================================================================

struct ChannelRule
{
    std::string channel;
    ovphysx_log_level_t minSeverity;
};

struct CallbackRegistration
{
    ovphysx_log_callback_t callback = nullptr;
    void* user_data = nullptr;
    ovphysx_log_level_t minSeverity = OVPHYSX_LOG_WARNING;
    std::vector<ChannelRule> channelRules;
    std::mutex invokeMutex;
    std::mutex drainMutex;
    std::condition_variable drainCv;
    uint64_t inFlight = 0;
};

std::mutex g_callbackMutex;
std::condition_variable g_callbackTransitionCv;
std::shared_ptr<CallbackRegistration> g_callback;
bool g_callbackTransition = false;
std::mutex g_carboniteReadyControlMutex;

struct DispatchNode
{
    uint64_t ticket = 0;
    bool complete = false;
    std::shared_ptr<DispatchNode> next;
};

// Each accepted delivery owns one node until the contiguous completion prefix
// advances past it. This allocation is the bounded bookkeeping cost that lets
// flush observe an exact point-in-time barrier even when registrations overlap,
// without holding a state mutex while application code runs.
std::mutex g_dispatchMutex;
std::condition_variable g_dispatchCv;
uint64_t g_dispatchAccepted = 0;
uint64_t g_dispatchCompleted = 0;
std::shared_ptr<DispatchNode> g_dispatchHead;
DispatchNode* g_dispatchTail = nullptr;
thread_local bool t_inDispatch = false;

bool isValidLogLevel(ovphysx_log_level_t level)
{
    return level >= OVPHYSX_LOG_DEFAULT && level <= OVPHYSX_LOG_NONE;
}

ovphysx_log_level_t effectiveLogLevel(ovphysx_log_level_t level)
{
    return level == OVPHYSX_LOG_DEFAULT ? OVPHYSX_LOG_WARNING : level;
}

bool channelMatches(const std::string& rule, const std::string& channel)
{
    return channel.compare(0, rule.size(), rule) == 0;
}

bool parseLogLevel(const std::string& text, ovphysx_log_level_t* outLevel)
{
    std::string lowered;
    lowered.reserve(text.size());
    for (char value : text)
    {
        if (value >= 'A' && value <= 'Z')
            value = static_cast<char>(value + ('a' - 'A'));
        lowered.push_back(value);
    }

    if (lowered == "default") *outLevel = OVPHYSX_LOG_DEFAULT;
    else if (lowered == "verbose") *outLevel = OVPHYSX_LOG_VERBOSE;
    else if (lowered == "info") *outLevel = OVPHYSX_LOG_INFO;
    else if (lowered == "warning" || lowered == "warn") *outLevel = OVPHYSX_LOG_WARNING;
    else if (lowered == "error") *outLevel = OVPHYSX_LOG_ERROR;
    else if (lowered == "none") *outLevel = OVPHYSX_LOG_NONE;
    else return false;
    return true;
}

bool isAsciiWhitespace(char value)
{
    return value == ' ' || value == '\t' || value == '\r' || value == '\n' || value == '\f' || value == '\v';
}

std::string trimAsciiWhitespace(const std::string& text)
{
    size_t begin = 0;
    while (begin < text.size() && isAsciiWhitespace(text[begin]))
        ++begin;
    size_t end = text.size();
    while (end > begin && isAsciiWhitespace(text[end - 1]))
        --end;
    return text.substr(begin, end - begin);
}

bool parseChannelFilter(const ovphysx_string_t* filter, std::vector<ChannelRule>* outRules, std::string* outError)
{
    outRules->clear();
    if (!filter || filter->length == 0)
        return true;
    if (!filter->ptr)
    {
        *outError = "channel_filter.ptr must not be NULL when length is nonzero";
        return false;
    }

    const std::string text(filter->ptr, filter->length);
    size_t begin = 0;
    while (begin <= text.size())
    {
        const size_t end = text.find(',', begin);
        const std::string entry = trimAsciiWhitespace(
            text.substr(begin, end == std::string::npos ? std::string::npos : end - begin));
        const size_t equals = entry.find('=');
        if (equals == std::string::npos || equals == 0 || equals + 1 == entry.size())
        {
            *outError = "channel_filter entries must use channel=level";
            return false;
        }
        ChannelRule rule;
        rule.channel = trimAsciiWhitespace(entry.substr(0, equals));
        const std::string levelText = trimAsciiWhitespace(entry.substr(equals + 1));
        if (rule.channel.empty() || !parseLogLevel(levelText, &rule.minSeverity))
        {
            *outError = "channel_filter contains an unknown level";
            return false;
        }
        rule.minSeverity = effectiveLogLevel(rule.minSeverity);
        outRules->push_back(std::move(rule));
        if (end == std::string::npos)
            break;
        begin = end + 1;
    }
    return true;
}

ovphysx_log_level_t thresholdForChannel(
    ovphysx_log_level_t defaultThreshold,
    const std::vector<ChannelRule>& rules,
    const std::string& channel)
{
    ovphysx_log_level_t threshold = defaultThreshold;
    size_t longestMatch = 0;
    for (const ChannelRule& rule : rules)
    {
        if (channelMatches(rule.channel, channel) && rule.channel.size() >= longestMatch)
        {
            threshold = rule.minSeverity;
            longestMatch = rule.channel.size();
        }
    }
    return threshold;
}

struct UserCallbackLogger : public carb::logging::Logger2
{
    void handleMessage(const carb::logging::LogMessage& msg) override
    {
        std::shared_ptr<CallbackRegistration> registration;
        std::shared_ptr<DispatchNode> dispatchNode;
        uint64_t dispatchTicket = 0;
        bool registrationAccepted = false;
        bool dispatchAccepted = false;
        try
        {
            if (t_inDispatch)
                return;

            const ovphysx_log_level_t level = static_cast<ovphysx_log_level_t>(fromCarbLogLevel(msg.level));

            std::string channelText;
            {
                std::lock_guard<std::mutex> stateLock(g_callbackMutex);
                registration = g_callback;
                if (!registration)
                    return;

                // Carbonite owns msg.source. Copy it only after confirming that
                // delivery is active so a serialized callback receives stable,
                // NUL-terminated channel storage while it waits its turn.
                channelText = msg.source ? msg.source : "";
                if (level < thresholdForChannel(
                        registration->minSeverity, registration->channelRules, channelText))
                {
                    return;
                }
                dispatchNode = std::make_shared<DispatchNode>();
                {
                    std::scoped_lock<std::mutex, std::mutex> acceptanceLock(
                        registration->drainMutex, g_dispatchMutex);
                    if (g_dispatchAccepted == UINT64_MAX)
                        return;
                    ++registration->inFlight;
                    registrationAccepted = true;
                    dispatchTicket = ++g_dispatchAccepted;
                    dispatchNode->ticket = dispatchTicket;
                    if (g_dispatchTail)
                        g_dispatchTail->next = dispatchNode;
                    else
                        g_dispatchHead = dispatchNode;
                    g_dispatchTail = dispatchNode.get();
                    dispatchAccepted = true;
                }
            }

            {
                std::lock_guard<std::mutex> invokeLock(registration->invokeMutex);
                const bool previousInDispatch = t_inDispatch;
                t_inDispatch = true;
                const char* message = msg.message ? msg.message : "";
                double timestamp = 0.0;
                if (CARB_INCLUDES_MEMBER(msg.sizeOf, carb::logging::LogMessage::timestampNsSinceUnixEpoch) &&
                    msg.timestampNsSinceUnixEpoch.count() > 0)
                {
                    timestamp = static_cast<double>(msg.timestampNsSinceUnixEpoch.count()) / 1000000000.0;
                }
                else
                {
                    const std::chrono::duration<double> sinceEpoch =
                        std::chrono::system_clock::now().time_since_epoch();
                    timestamp = sinceEpoch.count();
                }
                try
                {
                    registration->callback(
                        level,
                        ovphysx_cstr(message),
                        ovphysx_cstr(channelText.c_str()),
                        timestamp,
                        registration->user_data);
                }
                catch (...)
                {
                    // C ABI callbacks must not throw. Swallow defensively so
                    // state and drain accounting cannot remain wedged.
                }
                t_inDispatch = previousInDispatch;
            }

            {
                std::lock_guard<std::mutex> drainLock(registration->drainMutex);
                --registration->inFlight;
            }
            registrationAccepted = false;
            registration->drainCv.notify_all();
            {
                std::lock_guard<std::mutex> dispatchLock(g_dispatchMutex);
                dispatchNode->complete = true;
                while (g_dispatchHead && g_dispatchHead->complete)
                {
                    g_dispatchCompleted = g_dispatchHead->ticket;
                    g_dispatchHead = g_dispatchHead->next;
                }
                if (!g_dispatchHead)
                    g_dispatchTail = nullptr;
            }
            dispatchAccepted = false;
            g_dispatchCv.notify_all();
        }
        catch (...)
        {
            // Carbonite Logger2 callbacks must not throw. If an exception
            // occurs after acceptance, retire both counters so flush/replace
            // cannot remain wedged.
            if (registrationAccepted && registration)
            {
                {
                    std::lock_guard<std::mutex> drainLock(registration->drainMutex);
                    if (registration->inFlight != 0)
                        --registration->inFlight;
                }
                registration->drainCv.notify_all();
            }
            if (dispatchAccepted && dispatchNode && dispatchTicket != 0)
            {
                {
                    std::lock_guard<std::mutex> dispatchLock(g_dispatchMutex);
                    dispatchNode->complete = true;
                    while (g_dispatchHead && g_dispatchHead->complete)
                    {
                        g_dispatchCompleted = g_dispatchHead->ticket;
                        g_dispatchHead = g_dispatchHead->next;
                    }
                    if (!g_dispatchHead)
                        g_dispatchTail = nullptr;
                }
                g_dispatchCv.notify_all();
            }
            t_inDispatch = false;
        }
    }
};

static UserCallbackLogger g_userLogger;
static bool g_userLoggerRegistered = false;
std::mutex g_userLoggerRegistrationMutex;

void syncUserLoggerRegistration()
{
    std::lock_guard<std::mutex> registrationLock(g_userLoggerRegistrationMutex);
    bool shouldRegister = false;
    {
        std::lock_guard<std::mutex> callbackLock(g_callbackMutex);
        shouldRegister = static_cast<bool>(g_callback);
    }
    shouldRegister = shouldRegister && g_carboniteReady.load(std::memory_order_acquire);

    if (shouldRegister == g_userLoggerRegistered)
        return;
    if (carb::logging::ILogging* logging = carb::logging::getLogging())
    {
        if (shouldRegister)
        {
            logging->addLogger(&g_userLogger);
            g_userLoggerRegistered = true;
        }
        else
        {
            logging->removeLogger(&g_userLogger);
            g_userLoggerRegistered = false;
        }
    }
}

ovphysx_result_t replaceLogCallback(
    std::shared_ptr<CallbackRegistration> replacement,
    bool waitForTransition)
{
    bool ownsTransition = false;
    try
    {
        std::shared_ptr<CallbackRegistration> previous;
        bool transitionBusy = false;
        {
            std::unique_lock<std::mutex> stateLock(g_callbackMutex);
            if (waitForTransition)
            {
                g_callbackTransitionCv.wait(stateLock, [] { return !g_callbackTransition; });
            }
            else if (g_callbackTransition)
            {
                transitionBusy = true;
            }

            if (!transitionBusy)
            {
                g_callbackTransition = true;
                ownsTransition = true;
                previous = g_callback;
                g_callback = std::move(replacement);
            }
        }
        if (transitionBusy)
            return set_error(OVPHYSX_API_ERROR, "Another log callback transition is in progress");

        if (previous)
        {
            std::unique_lock<std::mutex> drainLock(previous->drainMutex);
            previous->drainCv.wait(drainLock, [&previous] { return previous->inFlight == 0; });
        }
        {
            std::lock_guard<std::mutex> stateLock(g_callbackMutex);
            g_callbackTransition = false;
            ownsTransition = false;
        }
        g_callbackTransitionCv.notify_all();
        syncUserLoggerRegistration();
        return success();
    }
    catch (...)
    {
        try
        {
            if (ownsTransition)
            {
                std::lock_guard<std::mutex> stateLock(g_callbackMutex);
                g_callbackTransition = false;
            }
            g_callbackTransitionCv.notify_all();
            return set_error(OVPHYSX_API_ERROR, "Unable to configure log callback");
        }
        catch (...)
        {
            return {OVPHYSX_API_ERROR};
        }
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
        std::unordered_map<uint32_t, std::vector<std::string>>::const_iterator it = m_messages.find(level);
        if (it == m_messages.end()) return false;
        for (const std::string& message : it->second)
            if (message.find(substring) != std::string::npos)
                return true;
        return false;
    }

    uint32_t count(uint32_t level) const
    {
        std::lock_guard<std::mutex> g(m_mutex);
        std::unordered_map<uint32_t, std::vector<std::string>>::const_iterator it = m_messages.find(level);
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
    std::lock_guard<std::mutex> readyLock(g_carboniteReadyControlMutex);

    if (carb::logging::ILogging* logging = carb::logging::getLogging())
    {
        // Ovruntime is linked into libovphysx instead of loaded as a Carbonite
        // plugin, so its named channel has no static registrar. Register it
        // explicitly before applying the stored per-source policy. This runs on
        // every instance creation, but registration is process-wide and the
        // framework is never released, so do it exactly once: re-registering
        // would re-add the same channel pointer on each new instance.
        static bool sRuntimeSourceRegistered = false;
        if (!sRuntimeSourceRegistered)
        {
            sRuntimeSourceRegistered = true;
            logging->registerSource(kDefaultChannel.name, setPhysxRuntimeLogLevel);
        }

        // Latched separately: omniGetLogWithoutAcquire() can still be null on an
        // early call, and folding this into the flag above would skip the channel
        // add permanently once the source registration has been latched.
        static bool sRuntimeChannelAdded = false;
        if (!sRuntimeChannelAdded)
        {
            if (omni::log::ILog* omniLog = omniGetLogWithoutAcquire())
            {
                sRuntimeChannelAdded = true;
                omniLog->addChannel(
                    kDefaultChannel.name,
                    reinterpret_cast<omni::log::Level*>(&kDefaultChannel.level),
                    kDefaultChannel.description);
            }
        }

        // Apply only to ovphysx's named target sources. Unnamed and other
        // non-target process sources retain their own policies.
        const uint32_t level = g_logLevel.load(std::memory_order_acquire);
        applyCarboniteLogLevel(logging, level);

        // Apply stored default-output preference (user may have called
        // ovphysx_enable_default_log_output(false) before Carbonite init).
        // Carbonite starts with the default logger registered, which matches
        // g_defaultOutputEnabled == true, so only the false case needs action.
        if (!g_defaultOutputEnabled)
        {
            if (carb::logging::StandardLogger2* defLogger = logging->getDefaultLogger())
                logging->removeLogger(defLogger->getLogger());
        }
    }

    // Publish readiness only after the stored policies have been applied.
    // Setters take this same mutex, so they cannot interleave an add/remove or
    // threshold update with initialization and leave the side effect stale.
    g_carboniteReady.store(true, std::memory_order_release);

    syncUserLoggerRegistration();
}

ovphysx_result_t shutdownLogCallback()
{
    if (carb::logging::ILogging* logging = carb::logging::getLogging())
        logging->flushLogs();
    // Public replacements reject an overlapping transition. Shutdown cannot
    // fail after runtime teardown solely because one was already draining, so
    // wait to own the transition and then disable the callback directly.
    return replaceLogCallback(nullptr, true);
}

bool isInLogCallback()
{
    return t_inDispatch;
}

} // namespace ovphysx

// ============================================================================
// Public C API -- Log level
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_set_log_level(uint32_t level)
{
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot set log level from within a log callback");

    if (level > OVPHYSX_LOG_NONE)
    {
        CARB_LOG_WARN("ovphysx_set_log_level: invalid level %u (max %u); level not changed",
                      static_cast<unsigned>(level), static_cast<unsigned>(OVPHYSX_LOG_NONE));
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Level out of range (0-5); no change applied");
    }

    std::lock_guard<std::mutex> readyLock(g_carboniteReadyControlMutex);
    const uint32_t effectiveLevel = static_cast<uint32_t>(effectiveLogLevel(static_cast<ovphysx_log_level_t>(level)));
    g_logLevel.store(effectiveLevel, std::memory_order_release);

    if (g_carboniteReady.load(std::memory_order_acquire))
    {
        if (carb::logging::ILogging* logging = carb::logging::getLogging())
        {
            applyCarboniteLogLevel(logging, effectiveLevel);
        }
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
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot change default log output from within a log callback");

    std::lock_guard<std::mutex> readyLock(g_carboniteReadyControlMutex);
    const bool wasEnabled = g_defaultOutputEnabled;
    g_defaultOutputEnabled = enable;

    if (wasEnabled != enable && g_carboniteReady.load(std::memory_order_acquire))
    {
        carb::logging::ILogging* logging = carb::logging::getLogging();
        if (logging)
        {
            carb::logging::StandardLogger2* defLogger = logging->getDefaultLogger();
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

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_set_log_callback(
    ovphysx_log_level_t min_severity,
    const ovphysx_string_t* channel_filter,
    ovphysx_log_callback_t callback,
    void* user_data)
{
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot set log callback from within a callback");

    try
    {
        if (!isValidLogLevel(min_severity))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "min_severity must be in range 0-5");

        std::vector<ChannelRule> rules;
        std::string parseError;
        if (!parseChannelFilter(channel_filter, &rules, &parseError))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, parseError);

        std::shared_ptr<CallbackRegistration> replacement;
        if (callback)
        {
            replacement = std::make_shared<CallbackRegistration>();
            replacement->callback = callback;
            replacement->user_data = user_data;
            replacement->minSeverity = effectiveLogLevel(min_severity);
            replacement->channelRules = std::move(rules);
        }

        return replaceLogCallback(std::move(replacement), false);
    }
    catch (...)
    {
        try
        {
            return set_error(OVPHYSX_API_ERROR, "Unable to configure log callback");
        }
        catch (...)
        {
            return {OVPHYSX_API_ERROR};
        }
    }
}

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_flush_log(ovphysx_timeout_t timeout_ns)
{
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot flush log from within a callback");

    std::unique_lock<std::mutex> lock(g_dispatchMutex);
    const uint64_t target = g_dispatchAccepted;
    if (timeout_ns == OVPHYSX_TIMEOUT_INFINITE)
        g_dispatchCv.wait(lock, [target] { return g_dispatchCompleted >= target; });
    else
    {
        const uint64_t maximumNs = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
        const int64_t boundedNs = static_cast<int64_t>(timeout_ns > maximumNs ? maximumNs : timeout_ns);
        if (!g_dispatchCv.wait_for(
                lock,
                std::chrono::nanoseconds(boundedNs),
                [target] { return g_dispatchCompleted >= target; }))
            return set_error(OVPHYSX_API_TIMEOUT, "Timed out waiting for log delivery");
    }

    return success();
}

// ============================================================================
// Public C API -- Test capture
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_log_capture_start(void)
{
    if (g_captureLogger)
        return success(); // already capturing

    carb::logging::ILogging* logging = carb::logging::getLogging();
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

    if (carb::logging::ILogging* logging = carb::logging::getLogging())
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
    // Fatal is needed only to prove that NONE suppresses Carbonite's highest
    // severity. Avoid injecting fatal records into unrelated logging tests.
    if (g_logLevel.load(std::memory_order_acquire) == OVPHYSX_LOG_NONE)
        CARB_LOG_FATAL("[LogTest] FATAL test message");
    CARB_LOG_ERROR("[LogTest] ERROR test message");
    CARB_LOG_WARN("[LogTest] WARNING test message");
    CARB_LOG_INFO("[LogTest] INFO test message");
    CARB_LOG_VERBOSE("[LogTest] VERBOSE test message");
}

extern "C" OVPHYSX_API void ovphysx_log_set_global_enabled_for_test(bool enabled)
{
    if (carb::logging::ILogging* logging = carb::logging::getLogging())
        logging->setLogEnabled(enabled);
}

extern "C" OVPHYSX_API bool ovphysx_log_get_global_enabled_for_test(void)
{
    if (carb::logging::ILogging* logging = carb::logging::getLogging())
        return logging->isLogEnabled();
    return false;
}
