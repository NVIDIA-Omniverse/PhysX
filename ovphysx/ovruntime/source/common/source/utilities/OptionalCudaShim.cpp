// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


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
