// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <common/utilities/OptionalCudaShim.h>

namespace omni
{
namespace physx
{
namespace optionalCuda
{
namespace detail
{

std::atomic<ProviderFn>& getProviderSlot()
{
    static std::atomic<ProviderFn> s_provider{ nullptr };
    return s_provider;
}

std::atomic<omni::physx::IOptionalCuda*>& getCachedSlot()
{
    static std::atomic<omni::physx::IOptionalCuda*> s_cached{ nullptr };
    return s_cached;
}

} // namespace detail
} // namespace optionalCuda
} // namespace physx
} // namespace omni
