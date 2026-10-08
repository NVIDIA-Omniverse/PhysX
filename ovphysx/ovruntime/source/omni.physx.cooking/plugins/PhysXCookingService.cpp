// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "service/CookingComputeService.h"

#ifndef OVRUNTIME_NO_UJITSO
#include "ujitso/UjitsoCookingComputeService.h"
#endif

namespace omni
{
namespace physx
{

/**
 * Both services take the same acquiring provider callback, so whichever one is selected here holds
 * a real reference on the host's PxCudaContextManager rather than borrowing a raw pointer. The
 * UJITSO service forwards to the fallback for context resolution, so the contract is identical
 * either way.
 *
 * @implements REQ-COOK-CUDACTX-001
 * @covers AC-1
 */
ICookingComputeService* createDefaultCookingComputingService(::physx::PxFoundation& foundation,
                                                            AcquireSharedCudaContextManagerFn acquireSharedCudaContextManagerFn)
{
#ifndef OVRUNTIME_NO_UJITSO
    // Always try the UJITSO cache service first; it internally falls back to local cooking when disabled or unavailable.
    if (ICookingComputeService* service = createUjitsoCookingComputingService(foundation, acquireSharedCudaContextManagerFn))
    {
        return service;
    }
#endif
    return createCookingComputingService(foundation, acquireSharedCudaContextManagerFn);
}

} // namespace physx
} // namespace omni
