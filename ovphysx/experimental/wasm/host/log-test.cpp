// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "Log.h"
#include <carb/Framework.h>
#include <omni/log/ILog.h>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
CARB_FRAMEWORK_GLOBALS("ovphysx.wasm.log-test")
OMNI_LOG_ADD_CHANNEL(OMNI_LOG_DEFAULT_CHANNEL, "ovphysx.wasm.log-test", "Static logger contract test");
// logging: allow fprintf - standalone contract test failure diagnostics.
#define CHECK(value) do { if (!(value)) { std::fprintf(stderr, "check failed at %d: %s\n", __LINE__, #value); return 1; } } while (false)
using namespace omni::log;
int main()
{
    ILog* log = ovphysxWasmLog();
    CHECK(log == omniGetLogWithoutAcquire());
    CHECK(!log->isAsync() && !log->setAsync(false));
    log->setLevel(Level::eWarn);
    log->setChannelEnabled("contract", false, SettingBehavior::eOverride);
    Level first = Level::eVerbose;
    Level second = Level::eVerbose;
    log->addChannel("contract", &first, "copied description");
    log->addChannel("contract", &second, "ignored description");
    CHECK(first == Level::eDisabled && second == first);
    log->setChannelEnabled("contract", true, SettingBehavior::eInherit);
    CHECK(first == Level::eWarn && second == first);
    log->setChannelLevel("contract", Level::eInfo, SettingBehavior::eOverride);
    CHECK(first == Level::eInfo);
    log->setEnabled(false);
    CHECK(first == Level::eDisabled);
    log->setChannelEnabled("contract", true, SettingBehavior::eOverride);
    CHECK(first == Level::eInfo);
    CHECK(log->isLoggingAtLevel("contract", Level::eWarn));
    CHECK(!log->isLoggingAtLevel("unknown", Level::eFatal));
    CHECK(!log->isLoggingAtLevel("contract", Level::eVerbose));
    std::vector<std::string> messages;
    omni::core::ObjectPtr<ILogMessageConsumer> consumer = log->addMessageConsumer(
        [&](const char* channel, Level level, const char*, const char*, const char*, uint32_t,
            const char* message, carb::thread::ProcessId, carb::thread::ThreadId threadId, uint64_t timestamp) {
            if (std::string(channel) != "contract" || level < Level::eInfo || !threadId || !timestamp) std::abort();
            messages.emplace_back(message);
        });
    log->logf("contract", Level::eInfo, "module", "file", "function", 17, "value=%d text=%s", 42, "100%");
    log->log("contract", Level::eVerbose, nullptr, nullptr, nullptr, 0, "filtered", 0);
    CHECK(messages.size() == 1 && messages[0] == "value=42 text=100%");
    CHECK(log->getMessageConsumers().size() == 1);
    log->addMessageConsumer(consumer);
    CHECK(log->getMessageConsumers().size() == 1);
    uint32_t updates = 0;
    omni::core::ObjectPtr<ILogChannelUpdateConsumer> observer = log->addChannelUpdateConsumer(
        [&](ILog* source, omni::str::IReadOnlyCString*, ChannelUpdateReason) {
            source->getLevel(); // Reentrant reads from update callbacks are explicitly supported.
            ++updates;
        });
    log->setChannelDescription("contract", "new description");
    CHECK(updates == 1);
    omni::str::IReadOnlyCString* description = nullptr;
    CHECK(log->getChannelDescription("contract", &description) == omni::core::kResultSuccess);
    CHECK(std::string(description->getBuffer()) == "new description");
    description->release();
    CHECK(!log->getChannelNames().empty());
    log->removeChannel("contract", &second);
    log->setChannelLevel("contract", Level::eError, SettingBehavior::eOverride);
    CHECK(first == Level::eError && second == Level::eInfo);
    log->setChannelLevel("contract", Level::eWarn, SettingBehavior::eOverride);
    std::thread one([&] { for (int index = 0; index < 3; ++index) log->log("contract", Level::eWarn, nullptr, nullptr, nullptr, 0, "thread-one", 0); });
    std::thread two([&] { for (int index = 0; index < 3; ++index) log->log("contract", Level::eWarn, nullptr, nullptr, nullptr, 0, "thread-two", 0); });
    one.join(); two.join();
    CHECK(messages.size() == 7);
    log->removeMessageConsumer(consumer);
    log->removeChannelUpdateConsumer(observer);
    CHECK(log->getMessageConsumers().empty() && log->getChannelUpdateConsumers().empty());
    log->removeChannel("contract", &first);
    log->flush();
    std::puts("PASS real ILog filtering, inherited overrides, channel slots, formatting, consumer lifetime and threaded delivery");
}
