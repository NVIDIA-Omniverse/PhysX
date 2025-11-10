// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once


#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include <vector>

template <typename T>
class LockingList
{
public:
    typedef void (*ReadFn)(const T* data, size_t size, void* userData);
    typedef void (*AccessFn)(std::vector<T>& list, void* userData);

    void access(AccessFn accessFn, void* userData)
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(m_mutex);
        accessFn(m_list, userData);
    }

    // read/write convenience functions:
    void push(const T& value)
    {
        access([](std::vector<T>& list, void* userData) { list.push_back(*(T*)userData); }, (T*)&value);
    }

    void read(ReadFn readFn, void* userData) const
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(m_mutex);
        readFn(m_list.data(), m_list.size(), userData);
    }

    void clear()
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(m_mutex);
        m_list.clear();
    }

private:
    std::vector<T> m_list;
    mutable carb::tasking::MutexWrapper m_mutex;
};
