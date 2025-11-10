// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "carb/Defines.h"

template <typename T>
struct Pair
{
    Pair(T ptr0, T ptr1) : m_ptr0(ptr0), m_ptr1(ptr1)
    {
        if (m_ptr0 > ptr1)
        {
            T cp = m_ptr0;
            m_ptr0 = m_ptr1;
            m_ptr1 = cp;
        }
    }

    bool operator==(const Pair& a) const
    {
        return a.m_ptr0 == m_ptr0 && a.m_ptr1 == m_ptr1;
    }

    bool operator!=(const Pair& a) const
    {
        return !(a == *this);
    }

    bool operator<(const Pair& a) const
    {
        return (m_ptr0 != a.m_ptr0) ? (m_ptr0 < a.m_ptr0) : (m_ptr1 < a.m_ptr1);
    }

    bool contains(T ptr) const
    {
        if (m_ptr0 == ptr)
            return true;
        if (m_ptr1 == ptr)
            return true;
        return false;
    }

    void swap(T oldPtr, T newPtr) const
    {
        if (m_ptr0 == oldPtr)
            m_ptr0 = newPtr;
        if (m_ptr1 == oldPtr)
            m_ptr1 = newPtr;
    }

    std::size_t hash() const
    {
        const std::size_t h1 = std::hash<T>()(m_ptr0);
        const std::size_t h2 = std::hash<T>()(m_ptr1);

        return h1 ^ h2;
    }

    T first() const
    {
        return m_ptr0;
    }
    T second() const
    {
        return m_ptr1;
    }

private:
    mutable T m_ptr0;
    mutable T m_ptr1;
};

template <>
inline std::size_t Pair<uint32_t>::hash() const
{
    const uint64_t h1 = uint64_t(m_ptr0);
    const uint64_t h2 = uint64_t(m_ptr1);

    return h1 << 32 | h2;
}

template <>
inline std::size_t Pair<uint64_t>::hash() const
{
    const uint64_t h1 = uint64_t(m_ptr0);
    const uint64_t h2 = uint64_t(m_ptr1);

    return h1 | h2;
}

template <>
inline std::size_t Pair<const void*>::hash() const
{
    const uint64_t h1 = uint64_t(m_ptr0);
    const uint64_t h2 = uint64_t(m_ptr1);

    return h1 | h2;
}

struct PairHash
{

    template <typename T>
    size_t operator()(const Pair<T>& pair) const
    {
        return pair.hash();
    }
};
