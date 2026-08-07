// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "service/CookingComputeService.h"

#ifndef OVRUNTIME_NO_UJITSO
#include "ujitso/UjitsoCookingComputeService.h"
#endif

namespace omni
{
namespace physx
{

ICookingComputeService* createDefaultCookingComputingService(::physx::PxFoundation& foundation,
                                                            SharedCudaContextManagerFn sharedCudaContextManagerFn)
{
#ifndef OVRUNTIME_NO_UJITSO
    // Always try the UJITSO cache service first; it internally falls back to local cooking when disabled or unavailable.
    if (ICookingComputeService* service = createUjitsoCookingComputingService(foundation, sharedCudaContextManagerFn))
    {
        return service;
    }
#endif
    return createCookingComputingService(foundation, sharedCudaContextManagerFn);
}

} // namespace physx
} // namespace omni
