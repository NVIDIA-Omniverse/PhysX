// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "LoggingUtils.h"

#include <carb/settings/ISettings.h>

#include <carb/extras/ScopeExit.h>
#include <omni/Function.h>
#include <omni/Span.h>

namespace omni
{
namespace physx
{

// Lock guard with wrappable mutex class.  Default MutexBase class does nothing.
struct LockGuard
{
    struct MutexBase
    {
        virtual void lock()
        {
        }
        virtual void unloock()
        {
        }
    };

    /* Reference implementation for a mutex class that will enable locking
    struct Mutex : public LockGuard::MutexBase
    {
        virtual void    lock()      override { m_mutex.lock(); }
        virtual void    unloock()   override { m_mutex.unlock(); }
    private:
        std::mutex  m_mutex;
    };
    */

    explicit LockGuard(MutexBase& mutex) : m_mutex(mutex)
    {
        m_mutex.lock();
    }
    ~LockGuard()
    {
        m_mutex.unloock();
    }

    // Non-copyable
    LockGuard() = delete;
    LockGuard(const LockGuard&) = delete;
    LockGuard(LockGuard&&) = delete;
    LockGuard& operator=(LockGuard&&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    MutexBase& m_mutex;
};


// Simple class to stream data from a uint8_t buffer
struct BufferReader
{
    BufferReader(const std::vector<uint8_t>& data) : curr(data.data()), stop(data.data() + data.size())
    {
    }

    template <typename T>
    bool readValue(T& value)
    {
        const uint8_t* newCurr = curr + sizeof(T);
        CHECK_RETURN_FALSE_ON_FAIL(newCurr <= stop);
        memcpy(&value, curr, sizeof(T));
        curr = newCurr;
        return true;
    }

    template <typename SizeType, typename DataType>
    bool readConstSpan(omni::span<const DataType>& span)
    {
        SizeType count = 0;
        CHECK_RETURN_FALSE_ON_FAIL(readValue(count));
        const DataType* block = reinterpret_cast<const DataType*>(curr);
        const uint8_t* blockStop = reinterpret_cast<const uint8_t*>(block + count);
        CHECK_RETURN_FALSE_ON_FAIL(blockStop <= stop);
        curr = blockStop;
        span = { count ? block : nullptr, (size_t)count };
        return true;
    }

private:
    const uint8_t* curr;
    const uint8_t* stop;
};


// Safe settings string read utility
// Returns a default value if the path is not in the settings dictionary
inline std::string getSettingsStringSafe(carb::settings::ISettings& settings,
                                         const char* path,
                                         const char* defaultValue = "")
{
    const char* c_str = settings.getStringBuffer(path);
    return std::string(c_str ? c_str : defaultValue);
}


} // namespace physx
} // namespace omni
