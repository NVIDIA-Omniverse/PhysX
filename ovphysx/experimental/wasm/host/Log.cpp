// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "Log.h"
#include <omni/log/ILog.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace
{
using namespace omni::log;
using omni::core::ObjectPtr;
using omni::core::Result;

[[noreturn]] void unsupportedLog(const char* operation)
{
    // logging: allow fprintf - the logging backend cannot diagnose itself through its own interface.
    std::fprintf(stderr, "OvPhysX WASM logging does not support %s\n", operation);
    std::abort();
}

template <class T>
Result enumerate(const std::vector<ObjectPtr<T>>& items, T** output, uint32_t* count)
{
    if (!count) return omni::core::kResultInvalidArgument;
    const uint32_t capacity = *count;
    *count = static_cast<uint32_t>(items.size());
    if (!output) return omni::core::kResultSuccess;
    if (capacity < items.size()) return omni::core::kResultInsufficientBuffer;
    for (size_t index = 0; index < items.size(); ++index)
    {
        if (output[index]) return omni::core::kResultInvalidArgument;
    }
    for (size_t index = 0; index < items.size(); ++index)
    {
        items[index]->acquire();
        output[index] = items[index].get();
    }
    return omni::core::kResultSuccess;
}

class Log final : public omni::core::Implements<ILog>
{
    struct Channel
    {
        Level level = Level::eWarn;
        bool enabled = true;
        SettingBehavior levelBehavior = SettingBehavior::eInherit;
        SettingBehavior enabledBehavior = SettingBehavior::eInherit;
        std::string description;
        std::vector<Level*> slots;
    };
    std::recursive_mutex m_mutex;
    Level m_level = Level::eWarn;
    bool m_enabled = true;
    std::map<std::string, Channel> m_channels;
    std::vector<ObjectPtr<ILogMessageConsumer>> m_messages;
    std::vector<ObjectPtr<ILogChannelUpdateConsumer>> m_updates;

    Level effective(const Channel& channel) const
    {
        const bool enabled = channel.enabledBehavior == SettingBehavior::eInherit ? m_enabled : channel.enabled;
        if (!enabled) return Level::eDisabled;
        return channel.levelBehavior == SettingBehavior::eInherit ? m_level : channel.level;
    }
    void refresh(Channel& channel)
    {
        for (Level* slot : channel.slots) *slot = effective(channel);
    }
    void notify(const char* name, ChannelUpdateReason reason)
    {
        const std::vector<ObjectPtr<ILogChannelUpdateConsumer>> listeners = m_updates;
        const ObjectPtr<omni::str::IReadOnlyCString> text = name ?
            omni::str::ReadOnlyCString::create(name) : nullptr;
        for (const ObjectPtr<ILogChannelUpdateConsumer>& listener : listeners)
            listener->onChannelUpdate(this, text, reason);
    }
    template <class T> void add(std::vector<ObjectPtr<T>>& items, T* item)
    {
        if (!item) unsupportedLog("null consumer registration");
        if (std::none_of(items.begin(), items.end(), [item](const ObjectPtr<T>& entry) { return entry.get() == item; }))
            items.emplace_back(item, omni::core::kBorrow);
    }
    template <class T> void remove(std::vector<ObjectPtr<T>>& items, T* item)
    {
        items.erase(std::remove_if(items.begin(), items.end(),
            [item](const ObjectPtr<T>& entry) { return entry.get() == item; }), items.end());
    }

    void log_abi(const char* channel, Level level, const char* module, const char* file,
                 const char* function, uint32_t line, const char* message, uint32_t count) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!channel || !message) unsupportedLog("null channel or message");
        if (!isLoggingAtLevel_abi(channel, level)) return;
        if (count && message[count - 1] != '\0') unsupportedLog("unterminated message");
        // logging: allow fprintf - this is the final stderr backend, not an application log call.
        std::fprintf(stderr, "[%s] %s\n", channel, message);
        const uint64_t timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        const std::vector<ObjectPtr<ILogMessageConsumer>> consumers = m_messages;
        for (const ObjectPtr<ILogMessageConsumer>& consumer : consumers)
            consumer->onMessage(channel, level, module, file, function, line, message,
                                carb::this_process::getId(), carb::this_thread::getId(), timestamp);
    }
    void logf_abi(const char* channel, Level level, const char* module, const char* file,
                  const char* function, uint32_t line, const char* format, va_list args) noexcept override
    {
        if (!format) unsupportedLog("null format");
        va_list measured;
        va_copy(measured, args);
        const int count = std::vsnprintf(nullptr, 0, format, measured);
        va_end(measured);
        if (count < 0) unsupportedLog("invalid format");
        std::vector<char> message(static_cast<size_t>(count) + 1);
        va_list copied;
        va_copy(copied, args);
        std::vsnprintf(message.data(), message.size(), format, copied);
        va_end(copied);
        log_abi(channel, level, module, file, function, line, message.data(), static_cast<uint32_t>(message.size()));
    }
    void addMessageConsumer_abi(ILogMessageConsumer* consumer) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); add(m_messages, consumer);
    }
    void removeMessageConsumer_abi(ILogMessageConsumer* consumer) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); remove(m_messages, consumer);
    }
    Result getMessageConsumers_abi(ILogMessageConsumer** consumers, uint32_t* count) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); return enumerate(m_messages, consumers, count);
    }
    void setLevel_abi(Level level) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        m_level = level;
        for (std::pair<const std::string, Channel>& entry : m_channels) refresh(entry.second);
        notify(nullptr, ChannelUpdateReason::eLevelUpdated);
    }
    Level getLevel_abi() noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); return m_level;
    }
    void setEnabled_abi(bool enabled) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        m_enabled = enabled;
        for (std::pair<const std::string, Channel>& entry : m_channels) refresh(entry.second);
        notify(nullptr, ChannelUpdateReason::eEnabledUpdated);
    }
    bool isEnabled_abi() noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); return m_enabled;
    }
    bool setAsync_abi(bool enabled) noexcept override
    {
        if (enabled) unsupportedLog("asynchronous delivery");
        return false;
    }
    bool isAsync_abi() noexcept override { return false; }
    void addChannel_abi(const char* name, Level* slot, const char* description) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name || !slot) unsupportedLog("null channel registration");
        Channel& channel = m_channels[name];
        if (std::find(channel.slots.begin(), channel.slots.end(), slot) == channel.slots.end())
            channel.slots.push_back(slot);
        if (channel.description.empty() && description) channel.description = description;
        refresh(channel);
        notify(name, ChannelUpdateReason::eChannelAdded);
    }
    void removeChannel_abi(const char* name, Level* slot) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name) unsupportedLog("null channel removal");
        const auto found = m_channels.find(name);
        if (found == m_channels.end()) return;
        std::vector<Level*>& slots = found->second.slots;
        slots.erase(std::remove(slots.begin(), slots.end(), slot), slots.end());
        notify(name, ChannelUpdateReason::eChannelRemoved);
    }
    Result getChannelNames_abi(omni::str::IReadOnlyCString** names, uint32_t* count) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        std::vector<ObjectPtr<omni::str::IReadOnlyCString>> values;
        for (const std::pair<const std::string, Channel>& entry : m_channels) values.push_back(omni::str::ReadOnlyCString::create(entry.first.c_str()));
        return enumerate(values, names, count);
    }
    void setChannelLevel_abi(const char* name, Level level, SettingBehavior behavior) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name) unsupportedLog("null channel name");
        Channel& channel = m_channels[name];
        channel.level = level; channel.levelBehavior = behavior;
        refresh(channel); notify(name, ChannelUpdateReason::eLevelUpdated);
    }
    Result getChannelLevel_abi(const char* name, Level* level, SettingBehavior* behavior) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name || !level || !behavior) return omni::core::kResultInvalidArgument;
        const auto found = m_channels.find(name);
        if (found == m_channels.end()) return omni::core::kResultNotFound;
        *level = found->second.level; *behavior = found->second.levelBehavior;
        return omni::core::kResultSuccess;
    }
    void setChannelEnabled_abi(const char* name, bool enabled, SettingBehavior behavior) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name) unsupportedLog("null channel name");
        Channel& channel = m_channels[name];
        channel.enabled = enabled; channel.enabledBehavior = behavior;
        refresh(channel); notify(name, ChannelUpdateReason::eEnabledUpdated);
    }
    Result getChannelEnabled_abi(const char* name, bool* enabled, SettingBehavior* behavior) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name || !enabled || !behavior) return omni::core::kResultInvalidArgument;
        const auto found = m_channels.find(name);
        if (found == m_channels.end()) return omni::core::kResultNotFound;
        *enabled = found->second.enabled; *behavior = found->second.enabledBehavior;
        return omni::core::kResultSuccess;
    }
    void setChannelDescription_abi(const char* name, const char* description) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name || !description) unsupportedLog("null description");
        m_channels[name].description = description;
        notify(name, ChannelUpdateReason::eDescriptionUpdated);
    }
    Result getChannelDescription_abi(const char* name, omni::str::IReadOnlyCString** result) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name || !result || *result) return omni::core::kResultInvalidArgument;
        const auto found = m_channels.find(name);
        if (found == m_channels.end()) return omni::core::kResultNotFound;
        if (!found->second.description.empty())
        {
            const ObjectPtr<omni::str::IReadOnlyCString> text = omni::str::ReadOnlyCString::create(found->second.description.c_str());
            text->acquire(); *result = text.get();
        }
        return omni::core::kResultSuccess;
    }
    bool isLoggingAtLevel_abi(const char* name, Level level) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        if (!name) return false;
        const auto found = m_channels.find(name);
        const Level threshold = found == m_channels.end() ? (m_enabled ? m_level : Level::eDisabled) : effective(found->second);
        return level < Level::eDisabled && level >= threshold;
    }
    void flush_abi() noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); std::fflush(stderr);
    }
    void addChannelUpdateConsumer_abi(ILogChannelUpdateConsumer* consumer) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); add(m_updates, consumer);
    }
    void removeChannelUpdateConsumer_abi(ILogChannelUpdateConsumer* consumer) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); remove(m_updates, consumer);
    }
    Result getChannelUpdateConsumers_abi(ILogChannelUpdateConsumer** consumers, uint32_t* count) noexcept override
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex); return enumerate(m_updates, consumers, count);
    }
};
} // namespace

omni::log::ILog* ovphysxWasmLog()
{
    // The shared image owns this reference until worker termination; callers borrow it.
    static Log* const instance = new Log();
    return instance;
}

void ovphysxWasmInitializeLog()
{
    omni::log::addModulesChannels();
}
