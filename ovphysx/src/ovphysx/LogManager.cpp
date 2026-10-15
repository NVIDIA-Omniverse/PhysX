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
 *
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-5
 */

// LogManager: global log level, one application callback, and test capture.
//
// Manages a single Logger2 for user callbacks and a separate one for test
// capture. Both are registered with Carbonite's ILogging when available.

#include "LogManager.hpp"
#include "internal/GlobalState.hpp"
#include "internal/LogManagerState.hpp"
#include "internal/Profiling.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <carb/logging/Logger.h>
#include <carb/logging/Log.h>
#include <omni/log/ILog.h>
#include <omni/log/LogChannel.h>
#include <omni/physx/RuntimeError.h>

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

using ovphysx::CallbackRegistration;
using ovphysx::ChannelRule;
using ovphysx::DispatchNode;
using ovphysx::LogCaptureLogger;

// Recursion suppression belongs to the current thread, not the process owner.
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

} // anonymous namespace

void ovphysx::UserCallbackLogger::handleMessage(const carb::logging::LogMessage& msg)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
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
            std::lock_guard<std::mutex> stateLock(state.callbackMutex);
            registration = state.callback;
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
                    registration->drainMutex, state.dispatchMutex);
                if (state.dispatchAccepted == UINT64_MAX)
                    return;
                ++registration->inFlight;
                registrationAccepted = true;
                dispatchTicket = ++state.dispatchAccepted;
                dispatchNode->ticket = dispatchTicket;
                if (state.dispatchTail)
                    state.dispatchTail->next = dispatchNode;
                else
                    state.dispatchHead = dispatchNode;
                state.dispatchTail = dispatchNode.get();
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
                // Errors from application code belong to its nested calls, not the operation logging here.
                omni::physx::RuntimeErrorScope callbackErrors;
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
            std::lock_guard<std::mutex> dispatchLock(state.dispatchMutex);
            dispatchNode->complete = true;
            while (state.dispatchHead && state.dispatchHead->complete)
            {
                state.dispatchCompleted = state.dispatchHead->ticket;
                state.dispatchHead = state.dispatchHead->next;
            }
            if (!state.dispatchHead)
                state.dispatchTail = nullptr;
        }
        dispatchAccepted = false;
        state.dispatchCv.notify_all();
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
                std::lock_guard<std::mutex> dispatchLock(state.dispatchMutex);
                dispatchNode->complete = true;
                while (state.dispatchHead && state.dispatchHead->complete)
                {
                    state.dispatchCompleted = state.dispatchHead->ticket;
                    state.dispatchHead = state.dispatchHead->next;
                }
                if (!state.dispatchHead)
                    state.dispatchTail = nullptr;
            }
            state.dispatchCv.notify_all();
        }
        t_inDispatch = false;
    }
}

namespace
{

void syncUserLoggerRegistration()
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    std::lock_guard<std::mutex> registrationLock(state.userLoggerRegistrationMutex);
    bool shouldRegister = false;
    {
        std::lock_guard<std::mutex> callbackLock(state.callbackMutex);
        shouldRegister = static_cast<bool>(state.callback);
    }
    shouldRegister = shouldRegister && state.carboniteReady.load(std::memory_order_acquire);

    if (shouldRegister == state.userLoggerRegistered)
        return;
    if (carb::logging::ILogging* logging = carb::logging::getLogging())
    {
        if (shouldRegister)
        {
            logging->addLogger(&state.userLogger);
            state.userLoggerRegistered = true;
        }
        else
        {
            logging->removeLogger(&state.userLogger);
            state.userLoggerRegistered = false;
        }
    }
}

ovphysx_result_t replaceLogCallback(
    std::shared_ptr<CallbackRegistration> replacement,
    bool waitForTransition)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    bool ownsTransition = false;
    try
    {
        std::shared_ptr<CallbackRegistration> previous;
        bool transitionBusy = false;
        {
            std::unique_lock<std::mutex> stateLock(state.callbackMutex);
            if (waitForTransition)
            {
                state.callbackTransitionCv.wait(stateLock, [&state] { return !state.callbackTransition; });
            }
            else if (state.callbackTransition)
            {
                transitionBusy = true;
            }

            if (!transitionBusy)
            {
                state.callbackTransition = true;
                ownsTransition = true;
                previous = state.callback;
                state.callback = std::move(replacement);
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
            std::lock_guard<std::mutex> stateLock(state.callbackMutex);
            state.callbackTransition = false;
            ownsTransition = false;
        }
        state.callbackTransitionCv.notify_all();
        syncUserLoggerRegistration();
        return success();
    }
    catch (...)
    {
        try
        {
            if (ownsTransition)
            {
                std::lock_guard<std::mutex> stateLock(state.callbackMutex);
                state.callbackTransition = false;
            }
            state.callbackTransitionCv.notify_all();
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

} // anonymous namespace

void ovphysx::LogCaptureLogger::handleMessage(const carb::logging::LogMessage& msg)
{
    std::lock_guard<std::mutex> g(m_mutex);
    uint32_t level = fromCarbLogLevel(msg.level);
    m_messages[level].push_back(msg.message);
}

bool ovphysx::LogCaptureLogger::find(uint32_t level, const char* substring) const
{
    std::lock_guard<std::mutex> g(m_mutex);
    std::unordered_map<uint32_t, std::vector<std::string>>::const_iterator it = m_messages.find(level);
    if (it == m_messages.end()) return false;
    for (const std::string& message : it->second)
        if (message.find(substring) != std::string::npos)
            return true;
    return false;
}

uint32_t ovphysx::LogCaptureLogger::count(uint32_t level) const
{
    std::lock_guard<std::mutex> g(m_mutex);
    std::unordered_map<uint32_t, std::vector<std::string>>::const_iterator it = m_messages.find(level);
    return it != m_messages.end() ? static_cast<uint32_t>(it->second.size()) : 0;
}

void ovphysx::LogCaptureLogger::clear()
{
    std::lock_guard<std::mutex> g(m_mutex);
    m_messages.clear();
}

// ============================================================================
// Internal API (called by CarboniteLoader)
// ============================================================================

namespace ovphysx
{

void onCarboniteLoggingReady()
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    std::lock_guard<std::mutex> readyLock(state.carboniteReadyControlMutex);

    if (carb::logging::ILogging* logging = carb::logging::getLogging())
    {
        // Ovruntime is linked into libovphysx instead of loaded as a Carbonite
        // plugin, so its named channel has no static registrar. Register it
        // explicitly before applying the stored per-source policy. This runs on
        // every instance creation, but register only once per framework: re-registering
        // would re-add the same channel pointer on each new instance.
        if (!state.runtimeSourceRegistered)
        {
            state.runtimeSourceRegistered = true;
            logging->registerSource(kDefaultChannel.name, setPhysxRuntimeLogLevel);
            if (state.captureLogger)
                logging->addLogger(state.captureLogger);
        }

        // Latched separately: omniGetLogWithoutAcquire() can still be null on an
        // early call, and folding this into the flag above would skip the channel
        // add permanently once the source registration has been latched.
        if (!state.runtimeChannelAdded)
        {
            if (omni::log::ILog* omniLog = omniGetLogWithoutAcquire())
            {
                state.runtimeChannelAdded = true;
                omniLog->addChannel(
                    kDefaultChannel.name,
                    reinterpret_cast<omni::log::Level*>(&kDefaultChannel.level),
                    kDefaultChannel.description);
            }
        }

        // Apply only to ovphysx's named target sources. Unnamed and other
        // non-target process sources retain their own policies.
        const uint32_t level = state.logLevel.load(std::memory_order_acquire);
        applyCarboniteLogLevel(logging, level);

        // Apply stored default-output preference (user may have called
        // ovphysx_enable_default_log_output(false) before Carbonite init).
        // Carbonite starts with the default logger registered, which matches
        // state.defaultOutputEnabled == true, so only the false case needs action.
        if (!state.defaultOutputEnabled)
        {
            if (carb::logging::StandardLogger2* defLogger = logging->getDefaultLogger())
                logging->removeLogger(defLogger->getLogger());
        }
    }

    // Publish readiness only after the stored policies have been applied.
    // Setters take this same mutex, so they cannot interleave an add/remove or
    // threshold update with initialization and leave the side effect stale.
    state.carboniteReady.store(true, std::memory_order_release);

    syncUserLoggerRegistration();
}

void onCarboniteLoggingStopped()
{
    LogManagerState& state = globalState().logging;
    std::lock_guard<std::mutex> readyLock(state.carboniteReadyControlMutex);
    state.carboniteReady.store(false, std::memory_order_release);
    state.runtimeSourceRegistered = false;
    state.runtimeChannelAdded = false;
}

ovphysx_result_t shutdownLogCallback()
{
    if (carb::logging::ILogging* logging = carb::logging::getLogging())
        logging->flushLogs();
    LogManagerState& state = globalState().logging;
    {
        std::lock_guard<std::mutex> readyLock(state.carboniteReadyControlMutex);
        state.carboniteReady.store(false, std::memory_order_release);
    }
    // Close registration before draining: a late replacement must remain
    // pending until bootstrap, rather than attach to a framework being released.
    // removeLogger waits for executing callbacks. Hold neither readiness nor
    // bootstrapMutex while removing it: callbacks may query loader/settings state.
    syncUserLoggerRegistration();
    // Public replacements reject an overlapping transition. Shutdown cannot
    // fail after runtime teardown solely because one was already draining, so
    // wait to own the transition and then disable the callback directly.
    return replaceLogCallback(nullptr, true);
}

bool isInLogCallback()
{
    return t_inDispatch;
}

ovphysx_result_t setLogLevel(uint32_t level)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot set log level from within a log callback");

    if (level > OVPHYSX_LOG_NONE)
    {
        CARB_LOG_WARN("ovphysx_set_log_level: invalid level %u (max %u); level not changed",
                      static_cast<unsigned>(level), static_cast<unsigned>(OVPHYSX_LOG_NONE));
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Level out of range (0-5); no change applied");
    }

    std::lock_guard<std::mutex> readyLock(state.carboniteReadyControlMutex);
    const uint32_t effectiveLevel =
        static_cast<uint32_t>(effectiveLogLevel(static_cast<ovphysx_log_level_t>(level)));
    state.logLevel.store(effectiveLevel, std::memory_order_release);

    if (state.carboniteReady.load(std::memory_order_acquire))
    {
        if (carb::logging::ILogging* logging = carb::logging::getLogging())
            applyCarboniteLogLevel(logging, effectiveLevel);
    }

    return success();
}

uint32_t getLogLevel()
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    return state.logLevel.load(std::memory_order_acquire);
}

ovphysx_result_t setDefaultLogOutput(bool enable)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot change default log output from within a log callback");

    std::lock_guard<std::mutex> readyLock(state.carboniteReadyControlMutex);
    const bool wasEnabled = state.defaultOutputEnabled;
    state.defaultOutputEnabled = enable;

    if (wasEnabled != enable && state.carboniteReady.load(std::memory_order_acquire))
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

bool getDefaultLogOutput()
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    std::lock_guard<std::mutex> readyLock(state.carboniteReadyControlMutex);
    return state.defaultOutputEnabled;
}

} // namespace ovphysx

// ============================================================================
// Public C API -- Log level
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_set_log_level(uint32_t level)
{
    OVPHYSX_API_PROFILE_ZONE(set_log_level);
    return ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL,
        ovphysx_config_value_int64(static_cast<int64_t>(level)));
}

extern "C" OVPHYSX_API uint32_t ovphysx_get_log_level(void)
{
    OVPHYSX_API_PROFILE_ZONE(get_log_level);
    ovphysx_config_value_t value = {};
    const ovphysx_result_t result =
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL, &value);
    return result.status == OVPHYSX_API_SUCCESS ?
        static_cast<uint32_t>(value.value.int64_value) :
        ovphysx::getLogLevel();
}

// ============================================================================
// Public C API -- Default console output
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_enable_default_log_output(bool enable)
{
    OVPHYSX_API_PROFILE_ZONE(enable_default_log_output);
    return ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT,
        ovphysx_config_value_bool(enable));
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
    OVPHYSX_API_PROFILE_ZONE(set_log_callback);
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
    OVPHYSX_API_PROFILE_ZONE(flush_log);
    if (t_inDispatch)
        return set_error(OVPHYSX_API_ERROR, "Cannot flush log from within a callback");

    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    std::unique_lock<std::mutex> lock(state.dispatchMutex);
    const uint64_t target = state.dispatchAccepted;
    if (timeout_ns == OVPHYSX_TIMEOUT_INFINITE)
        state.dispatchCv.wait(lock, [&state, target] { return state.dispatchCompleted >= target; });
    else
    {
        const uint64_t maximumNs = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
        const int64_t boundedNs = static_cast<int64_t>(timeout_ns > maximumNs ? maximumNs : timeout_ns);
        if (!state.dispatchCv.wait_for(
                lock,
                std::chrono::nanoseconds(boundedNs),
                [&state, target] { return state.dispatchCompleted >= target; }))
            return set_error(OVPHYSX_API_TIMEOUT, "Timed out waiting for log delivery");
    }

    return success();
}

// ============================================================================
// Public C API -- Test capture
// ============================================================================

extern "C" OVPHYSX_API ovphysx_result_t ovphysx_log_capture_start(void)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    if (state.captureLogger)
        return success(); // already capturing

    carb::logging::ILogging* logging = carb::logging::getLogging();
    if (!logging)
        return set_error(OVPHYSX_API_ERROR, "Carbonite logging not available");

    state.captureLogger = new LogCaptureLogger();
    logging->addLogger(state.captureLogger);
    return success();
}

extern "C" OVPHYSX_API void ovphysx_log_capture_stop(void)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    if (!state.captureLogger)
        return;

    if (carb::logging::ILogging* logging = carb::logging::getLogging())
        logging->removeLogger(state.captureLogger);

    delete state.captureLogger;
    state.captureLogger = nullptr;
}

extern "C" OVPHYSX_API bool ovphysx_log_capture_find(uint32_t level, const char* substring)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    if (!state.captureLogger || !substring)
        return false;
    return state.captureLogger->find(level, substring);
}

extern "C" OVPHYSX_API uint32_t ovphysx_log_capture_count(uint32_t level)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    if (!state.captureLogger)
        return 0;
    return state.captureLogger->count(level);
}

extern "C" OVPHYSX_API void ovphysx_log_emit_test_messages(void)
{
    ovphysx::LogManagerState& state = ovphysx::globalState().logging;
    // Fatal is needed only to prove that NONE suppresses Carbonite's highest
    // severity. Avoid injecting fatal records into unrelated logging tests.
    if (state.logLevel.load(std::memory_order_acquire) == OVPHYSX_LOG_NONE)
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
