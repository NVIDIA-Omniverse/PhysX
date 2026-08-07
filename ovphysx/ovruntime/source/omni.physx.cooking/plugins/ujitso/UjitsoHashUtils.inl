// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/extras/Hash.h>

#include <random>

namespace omni
{
namespace physx
{

/**
 * Generate a random key using std::random_device
 */

template <typename T>
inline T generateRandomKey()
{
    CARB_LOG_ERROR("No default implementation of generateRandomKey<T>");
    return (T)0;
}

template <>
inline uint32_t generateRandomKey<uint32_t>()
{
    return std::random_device()();
}

template <>
inline uint64_t generateRandomKey<uint64_t>()
{
    std::random_device rd;
    return (uint64_t)rd() << 32 | (uint64_t)rd();
}

template <>
inline carb::extras::hash128_t generateRandomKey<carb::extras::hash128_t>()
{
    carb::extras::hash128_t ret;
    std::random_device rd;
    ret.d[0] = (uint64_t)rd() << 32 | (uint64_t)rd();
    ret.d[1] = (uint64_t)rd() << 32 | (uint64_t)rd();
    return ret;
}

} // namespace physx
} // namespace omni
