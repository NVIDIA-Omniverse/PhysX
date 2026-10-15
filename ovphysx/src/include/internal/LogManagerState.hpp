// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LOG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 */

#pragma once

#include "ovphysx/ovphysx_types.h"

#include <carb/logging/Logger.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ovphysx
{

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

struct DispatchNode
{
    uint64_t ticket = 0;
    bool complete = false;
    std::shared_ptr<DispatchNode> next;
};

struct UserCallbackLogger : public carb::logging::Logger2
{
    void handleMessage(const carb::logging::LogMessage& msg) override;
};

struct LogCaptureLogger : public carb::logging::Logger2
{
    void handleMessage(const carb::logging::LogMessage& msg) override;
    bool find(uint32_t level, const char* substring) const;
    uint32_t count(uint32_t level) const;
    void clear();

private:
    mutable std::mutex m_mutex;
    std::unordered_map<uint32_t, std::vector<std::string>> m_messages;
};

// Process-wide logging storage. Lock order: readiness/control, then user-logger
// registration, then callback state, then registration drain and dispatch
// acceptance. Never hold callback state or registration drain locks while
// invoking user code or waiting for a prior registration to drain.
struct LogManagerState
{
    std::atomic<uint32_t> logLevel{OVPHYSX_LOG_WARNING};
    // False between SDK lifecycles, while framework-owned logging is unavailable.
    std::atomic<bool> carboniteReady{false};
    // Applied eagerly when Carbonite is ready, otherwise at initialization.
    bool defaultOutputEnabled = true;
    std::mutex callbackMutex;
    std::condition_variable callbackTransitionCv;
    std::shared_ptr<CallbackRegistration> callback;
    bool callbackTransition = false;
    std::mutex carboniteReadyControlMutex;

    // Accepted deliveries retain stable nodes until their contiguous completion
    // prefix advances, so flush can observe an exact point-in-time barrier.
    std::mutex dispatchMutex;
    std::condition_variable dispatchCv;
    uint64_t dispatchAccepted = 0;
    uint64_t dispatchCompleted = 0;
    std::shared_ptr<DispatchNode> dispatchHead;
    DispatchNode* dispatchTail = nullptr;

    UserCallbackLogger userLogger;
    bool userLoggerRegistered = false;
    std::mutex userLoggerRegistrationMutex;
    LogCaptureLogger* captureLogger = nullptr;
    bool runtimeSourceRegistered = false;
    bool runtimeChannelAdded = false;
};

} // namespace ovphysx
