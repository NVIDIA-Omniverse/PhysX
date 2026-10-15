// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */
#pragma once

#include "IPhysicsSource.h"
#include <type_traits>
#include <limits>

namespace omni::physics::parse
{
// A typed input records success separately from the value. No source pointer,
// token lookup, lazy cache or dynamic storage is available to a fill kernel.
template <typename T>
struct PreparedAttribute
{
    T value{};
    bool valid = false;
    bool authored = false;
    T valueOr(T fallback) const noexcept { return valid ? value : fallback; }
    bool read(T& out) const noexcept
    {
        if (valid) out = value;
        return valid;
    }
};

template <typename T, typename Reader>
PreparedAttribute<T> prepareAttribute(const Reader& source, ObjectKey key, TokenId token,
                                      bool requireAuthored = false)
{
    PreparedAttribute<T> result;
    result.authored = requireAuthored && source.hasAuthoredAttribute(key, token);
    if (!requireAuthored || result.authored)
    {
        if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool> && !std::is_same_v<T, int64_t>)
        {
            int64_t value = 0;
            result.valid = source.getAttribute(key, token, value);
            if (result.valid)
            {
                if constexpr (std::is_unsigned_v<T>)
                    result.valid = value >= 0 && uint64_t(value) <= std::numeric_limits<T>::max();
                else
                    result.valid = value >= std::numeric_limits<T>::lowest() && value <= std::numeric_limits<T>::max();
                if (result.valid) result.value = static_cast<T>(value);
            }
        }
        else
            result.valid = source.getAttribute(key, token, result.value);
    }
    return result;
}

} // namespace omni::physics::parse
