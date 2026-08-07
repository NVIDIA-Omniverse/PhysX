// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "PhysXFoundation.h"

#include <omni/physx/IPhysxSettings.h>
#include "PhysXGpuLoader.h"
#include "cuda_shim/CudaShim.h"
#include <common/utilities/Defer.h>

#include <atomic>

#include <carb/Framework.h>
#include <carb/settings/ISettings.h>
#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>
#include <cudamanager/PxCudaContextManager.h>

#include <cuda.h>

namespace omni
{
namespace physx
{
struct PhysXFoundation;
}
} // namespace omni
struct omni::physx::PhysXFoundation
{
    inline static std::atomic<bool> s_started{ false };
    inline static std::atomic<bool> s_forceCpuMode{ false };

    // One process-local loader for the PhysX GPU runtime library handles.
    // The loaded module registration is process-global; per-scene/per-device
    // CUDA state remains owned by PhysX context managers, not by this loader.
    // shutdownRuntime() is the explicit unload point; keep this as an object
    // rather than adding a hidden global pointer lifecycle.
    inline static omni::physx::gpu::PhysXGpuLoader s_gpuLoader;

    static void CARB_ABI setCpuMode(bool enabled)
    {
        if (enabled)
        {
            CARB_LOG_INFO("PhysXFoundation: CPU-only mode enabled.");
            s_forceCpuMode.store(true, std::memory_order_release);
        }
        else if (s_forceCpuMode.load(std::memory_order_acquire))
        {
            // Do not attempt to re-enable GPU after CPU mode was requested. This avoids creating partially
            // initialized/tearing-down CUDA state (unloading/reloading is not supported).
            CARB_LOG_WARN("PhysXFoundation: setCpuMode(false) ignored; CPU mode is sticky for process lifetime.");
        }
    }

    static bool CARB_ABI isCpuMode()
    {
        return s_forceCpuMode.load(std::memory_order_acquire);
    }

    static void CARB_ABI getSingleCudaContextManagerOrdinal(PhysxFoundationDeviceOrdinal& preferences)
    {
        if (isCpuMode())
        {
            preferences.mode = PhysxFoundationDeviceOrdinal::eMODE_PHYSX_DEFAULTS;
            preferences.deviceOrdinal = -1;
            return;
        }
        preferences.mode = PhysxFoundationDeviceOrdinal::eMODE_PHYSX_DEFAULTS;
        carb::settings::ISettings* carbSettings = carb::getCachedInterface<carb::settings::ISettings>();

        // figure out the desired CUDA context based on settings
        if (carbSettings->getAsBool(kSettingUseActiveCudaContext))
        {
            preferences.mode = PhysxFoundationDeviceOrdinal::eMODE_ACTIVE_CONTEXT;
            preferences.deviceOrdinal = -1;
            return;
        }

        // get device ordinal based on settings
        preferences.deviceOrdinal = carbSettings->getAsInt(kSettingCudaDevice);
        if (preferences.deviceOrdinal >= 0)
        {
            preferences.mode = PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL;
            return;
        }

        // Direct CUDA Driver API query — no dependency on GPU Foundation or CudaInterop
        if (!omni::physx::cudaShim::isCudaAvailable())
        {
            preferences.mode = PhysxFoundationDeviceOrdinal::eMODE_PHYSX_DEFAULTS;
            preferences.deviceOrdinal = -1;
            return;
        }

        preferences.deviceOrdinal = omni::physx::cudaShim::selectBestPhysicsDevice();
        CARB_LOG_INFO("PhysXFoundation: Direct CUDA query selected device ordinal: %d", preferences.deviceOrdinal);
        if (preferences.deviceOrdinal >= 0)
        {
            preferences.mode = PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL;
        }
    }

    static bool CARB_ABI cudaDeviceCheck()
    {
        if (isCpuMode())
        {
            return false;
        }

        // Guard: selectBestPhysicsDevice uses the shim's dlsym-resolved function pointers.
        // Bail out early on CPU-only machines.
        if (!omni::physx::cudaShim::isCudaAvailable())
        {
            CARB_LOG_WARN("PhysXFoundation: cudaDeviceCheck -> isCudaAvailable() returned false (no CUDA driver or no NVIDIA GPU detected).");
            return false;
        }

        // Cache the result — device availability does not change at runtime and
        // some drivers return inconsistent results on consecutive queries.
        static std::atomic<int> s_cachedResult{ -1 }; // -1=unset, 0=false, 1=true
        {
            int cached = s_cachedResult.load(std::memory_order_acquire);
            if (cached != -1)
                return cached == 1;
        }

        int deviceOrdinal = omni::physx::cudaShim::selectBestPhysicsDevice();
        bool result = (deviceOrdinal >= 0);
        if (result)
            CARB_LOG_INFO("PhysXFoundation: CUDA device %d found via Driver API.", deviceOrdinal);
        else
            CARB_LOG_INFO("PhysXFoundationCheck: No CUDA device found.");
        CARB_LOG_INFO("PhysXFoundation: cudaDeviceCheck result: %s (cached for process lifetime).", result ? "GPU available" : "no GPU / CPU-only");
        s_cachedResult.store(result ? 1 : 0, std::memory_order_release);
        return result;
    }

    static void CARB_ABI createGpuFoundation()
    {
        // GPU Foundation dependency removed; retained as no-op for interface compatibility.
    }

    static void CARB_ABI releaseGpuFoundation()
    {
        // GPU Foundation dependency removed; retained as no-op for interface compatibility.
    }

    static bool createOrRefreshPxCudaContextManager(PhysxFoundationDeviceOrdinal preferences,
                                                ::physx::PxFoundation* pxFoundation,
                                                ::physx::PxCudaContextManager*& pxCudaContextManager,
                                                const bool enableSynchronousKernelLaunches)
    {
        if (isCpuMode())
        {
            return false;
        }

        // Guard: PxCreateCudaContextManager (and the CUDA shim calls in this function)
        // require a live CUDA driver. Bail early on CPU-only machines.
        if (!omni::physx::cudaShim::isCudaAvailable())
        {
            return false;
        }

        CUcontext desiredCtx = nullptr;
        CUcontext savedCtx = nullptr;
        if (omni::physx::cudaShim::cuCtxGetCurrent_(&savedCtx) != CUDA_SUCCESS)
            CARB_LOG_WARN("PhysXFoundation: Could not save current CUDA context; caller's context will not be restored on exit.");

        bool contextChanged = false;

        // Restore the caller's prior context on exit — but only when one existed and
        // we actually changed it. Skipping the restore when contextChanged is false
        // avoids a no-op driver round-trip (e.g. when setDevice failed).
        // cuCtxSetCurrent(nullptr) would clear any context made current inside this
        // function, so the null guard is also required.
        auto deferRestoreCtx = CreateDeferLambda([&] {
            if (contextChanged && savedCtx)
            {
                CUresult r = omni::physx::cudaShim::cuCtxSetCurrent_(savedCtx);
                if (r != CUDA_SUCCESS)
                    CARB_LOG_ERROR("PhysXFoundation: Failed to restore caller CUDA context %p (CUresult %d); "
                                   "thread context state is undefined.", savedCtx, static_cast<int>(r));
            }
        });

        switch (preferences.mode)
        {
        case PhysxFoundationDeviceOrdinal::eMODE_ACTIVE_CONTEXT: {
            desiredCtx = savedCtx;
            if (!desiredCtx)
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

            int deviceCount = 0;
            CUresult result = omni::physx::cudaShim::cuDeviceGetCount_(&deviceCount);
            if (result != CUDA_SUCCESS || deviceCount == 0)
            {
                CARB_LOG_ERROR("No CUDA devices found");
                return false;
            }

            if (preferences.deviceOrdinal >= deviceCount)
            {
                CARB_LOG_ERROR("Requested CUDA device ordinal %d is not valid (device count: %d)", preferences.deviceOrdinal, deviceCount);
                return false;
            }

            // Activate the primary context for the requested device (Driver API equivalent of cudaSetDevice),
            // so that cuCtxGetCurrent returns it and any context already set by the renderer is reused.
            if (!omni::physx::cudaShim::setDevice(preferences.deviceOrdinal))
            {
                CARB_LOG_ERROR("PhysXFoundation: Failed to activate CUDA context for device ordinal %d.", preferences.deviceOrdinal);
                return false;
            }
            contextChanged = true;
            if (omni::physx::cudaShim::cuCtxGetCurrent_(&desiredCtx) != CUDA_SUCCESS)
                CARB_LOG_WARN("PhysXFoundation: Could not read back current CUDA context after setDevice(%d); PhysX will create its own.", preferences.deviceOrdinal);
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
            else if (preferences.mode == PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL && preferences.deviceOrdinal >= 0)
            {
                // Let PhysX create the context for the specified device
                cudaContextManagerDesc.deviceOrdinal = preferences.deviceOrdinal;
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

    static void setCudaDevice(int deviceOrdinal)
    {
        if (isCpuMode())
            return;
        if (!omni::physx::cudaShim::setDevice(deviceOrdinal))
            CARB_LOG_WARN("PhysXFoundation: setCudaDevice(%d): failed to activate CUDA context.", deviceOrdinal);
    }

};

namespace omni
{
namespace physx
{
namespace foundation
{
namespace
{

void populateInterface(omni::physx::IPhysxFoundation& iface)
{
    using namespace omni::physx;

    iface.getSingleCudaContextManagerOrdinal = &omni::physx::PhysXFoundation::getSingleCudaContextManagerOrdinal;
    iface.setCpuMode = &omni::physx::PhysXFoundation::setCpuMode;
    iface.isCpuMode = &omni::physx::PhysXFoundation::isCpuMode;
    iface.cudaDeviceCheck = &omni::physx::PhysXFoundation::cudaDeviceCheck;
    iface.createGpuFoundation = &omni::physx::PhysXFoundation::createGpuFoundation;
    iface.releaseGpuFoundation = &omni::physx::PhysXFoundation::releaseGpuFoundation;
    iface.createOrRefreshPxCudaContextManager = &omni::physx::PhysXFoundation::createOrRefreshPxCudaContextManager;
    iface.setCudaDevice = &omni::physx::PhysXFoundation::setCudaDevice;
}

void populateInterface(omni::physx::IOptionalCuda& iface)
{
    omni::physx::cudaShim::fill(iface);
}

} // namespace

void initializeRuntime()
{
    using namespace omni::physx;

    // The static foundation library has no Carbonite plugin lifecycle; its
    // current runtime owner chooses when process-level setup runs. Keep setup
    // idempotent because direct tests also call it, and because the function
    // tables below do not own the PhysX GPU runtime library handles.
    if (PhysXFoundation::s_started.exchange(true, std::memory_order_acq_rel))
    {
        return;
    }

    carb::settings::ISettings* carbSettings = carb::getCachedInterface<carb::settings::ISettings>();
    if (!carbSettings)
    {
        PhysXFoundation::s_started.store(false, std::memory_order_release);
        CARB_LOG_ERROR("PhysXFoundation: carb.settings interface is unavailable during startup.");
        return;
    }

    carbSettings->setDefaultBool(kSettingForceCpuMode, false);

    // Honor an explicit CPU-only request before any CUDA probe. If no request
    // was made, force CPU mode when no CUDA driver or real GPU is present so
    // PhysXGpu_64.dll/PhysXDevice64.dll are never loaded on CPU-only machines.
    if (carbSettings->getAsBool(kSettingForceCpuMode))
    {
        PhysXFoundation::setCpuMode(true);
    }
    else if (!omni::physx::cudaShim::isCudaAvailable() || !PhysXFoundation::cudaDeviceCheck())
    {
        PhysXFoundation::setCpuMode(true);
    }
    else
    {
        // Load PhysXGpu_64 into the process so PxLoadPhysxGPUModule can find it via GetModuleHandle.
        // Only reached when CUDA is available AND a suitable device is confirmed present.
        (void)PhysXFoundation::s_gpuLoader.load();
    }

    carbSettings->setDefaultInt(
        (std::string(DEFAULT_SETTING_PREFIX) + kSettingCudaDevice).c_str(), kSettingCudaDeviceDefaultVal);
    carbSettings->setDefaultInt(kSettingCudaDevice, kSettingCudaDeviceDefaultVal);
    carbSettings->setDefaultBool((std::string(DEFAULT_SETTING_PREFIX) + kSettingUseActiveCudaContext).c_str(),
                                 kSettingUseActiveCudaContextDefaultVal);
    carbSettings->setDefaultBool(kSettingUseActiveCudaContext, kSettingUseActiveCudaContextDefaultVal);
}

void shutdownRuntime()
{
    if (!PhysXFoundation::s_started.exchange(false, std::memory_order_acq_rel))
    {
        return;
    }

    omni::physx::PhysXFoundation::releaseGpuFoundation();
    omni::physx::PhysXFoundation::s_gpuLoader.unload();
}

omni::physx::IPhysxFoundation& getInterface()
{
    static omni::physx::IPhysxFoundation iface = [] {
        omni::physx::IPhysxFoundation table{};
        populateInterface(table);
        return table;
    }();
    return iface;
}

omni::physx::IOptionalCuda& getOptionalCudaInterface()
{
    static omni::physx::IOptionalCuda iface = [] {
        omni::physx::IOptionalCuda table{};
        populateInterface(table);
        return table;
    }();
    return iface;
}

} // namespace foundation
} // namespace physx
} // namespace omni
