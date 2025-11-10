// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <carb/cudainterop/CudaInterop.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>
#include <cudamanager/PxCudaContextManager.h>

#include <omni/physx/IPhysxFoundation.h> // PhysxFoundationDeviceOrdinal

#include <cuda.h>

#include "Defer.h"

namespace omni
{
namespace physx
{
inline bool createOrRefreshPxCudaContextManager(PhysxFoundationDeviceOrdinal preferences,
                                                ::physx::PxFoundation* pxFoundation,
                                                ::physx::PxCudaContextManager*& pxCudaContextManager,
                                                const bool enableSynchronousKernelLaunches)
{
    carb::cudainterop::CudaInterop* cudaInterop = carb::getCachedInterface<carb::cudainterop::CudaInterop>();

    CUcontext desiredCtx = nullptr;
    int cachedCudaInteropDeviceOrdinal = -1;

    auto deferRestoreDevice = CreateDeferLambda([&] {
        // revert device ordinal if needed
        if (cudaInterop && cachedCudaInteropDeviceOrdinal != -1)
        {
            cudaInterop->setDevice(cachedCudaInteropDeviceOrdinal);
        }
    });
    switch (preferences.mode)
    {
    case PhysxFoundationDeviceOrdinal::eMODE_ACTIVE_CONTEXT: {
        if (cuCtxGetCurrent(&desiredCtx) != CUDA_SUCCESS)
        {
            CARB_LOG_ERROR("Failed to determine current CUDA context");
            return false;
        }
    }
    break;
    case PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL: {
        if (preferences.deviceOrdinal < 0)
        {
            CARB_LOG_ERROR("CUDA deviceOrdinal is invalid");
            return false;
        }

        CARB_LOG_INFO("Using CUDA device ordinal %d.", preferences.deviceOrdinal);
        // setup device ordinal and cache the old one
        if (cudaInterop)
        {
            if (cudaInterop->getDevice(&cachedCudaInteropDeviceOrdinal) != 0)
            {
                cachedCudaInteropDeviceOrdinal = -1;
            }

            cudaInterop->setDevice(preferences.deviceOrdinal);

            if (preferences.deviceOrdinal != cachedCudaInteropDeviceOrdinal)
            {
                CARB_LOG_INFO("Device ordinal to revert to is %d.", cachedCudaInteropDeviceOrdinal);
            }
            else
            {
                cachedCudaInteropDeviceOrdinal = -1;
            }
        }

        int deviceCount = 0;
        if (cuDeviceGetCount(&deviceCount) == CUDA_SUCCESS && deviceCount > 0)
        {
            CUdevice device;
            if (preferences.deviceOrdinal < deviceCount && cuDeviceGet(&device, preferences.deviceOrdinal) == CUDA_SUCCESS)
            {
                if (cuDevicePrimaryCtxRetain(&desiredCtx, device) != CUDA_SUCCESS)
                {
                    CARB_LOG_ERROR(
                        "Failed to get primary context of requested CUDA device %d", preferences.deviceOrdinal);
                    return false;
                }
            }
            else
            {
                CARB_LOG_ERROR("Requested CUDA device ordinal %d is not valid", preferences.deviceOrdinal);
                return false;
            }
        }
        else
        {
            CARB_LOG_ERROR("No CUDA devices found");
            return false;
        }
    }
    break;
    case PhysxFoundationDeviceOrdinal::eMODE_PHYSX_DEFAULTS:
        break;
    }

    if (pxCudaContextManager)
    {
        // AD: if we are in skip state, let's just reset.
        if (pxCudaContextManager->getCudaContext()->isInAbortMode())
        {
            pxCudaContextManager->getCudaContext()->setAbortMode(false);
        }

        if (pxCudaContextManager->getCudaContext()->getLastError())
        {
            CARB_LOG_INFO("PhysX CUDA context manager recreated due to past errors.");
            PX_RELEASE(pxCudaContextManager);
        }
        else if (desiredCtx && pxCudaContextManager->getContext() != desiredCtx)
        {
            // If there is a context manager already, check if it's compatible with current settings.
            // If it's not, we release it and recreate it with the desired CUDA context.
            CARB_LOG_INFO("PhysX CUDA context manager recreated to use requested context.");
            PX_RELEASE(pxCudaContextManager);
        }
    }
    if (!pxCudaContextManager)
    {
        ::physx::PxCudaContextManagerDesc cudaContextManagerDesc;
        if (desiredCtx)
        {
            cudaContextManagerDesc.ctx = &desiredCtx;
        }
        pxCudaContextManager = PxCreateCudaContextManager(*pxFoundation, cudaContextManagerDesc, NULL, enableSynchronousKernelLaunches);
        if (pxCudaContextManager && !pxCudaContextManager->contextIsValid())
        {
            CARB_LOG_ERROR("Failed to create Cuda Context Manager.");
            PX_RELEASE(pxCudaContextManager);
            return false;
        }
    }

    return true;
}
} // namespace physx
} // namespace omni
