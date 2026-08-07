// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <carb/Defines.h>
#include <carb/Types.h>
#include <cstdint>

namespace physx
{
class PxFoundation;
class PxCudaContextManager;
}

namespace omni
{
namespace physx
{

/// PhysxFoundationDeviceOrdinal is an output filled by IPhysxFoundation::getSingleCudaContextManagerOrdinal
struct PhysxFoundationDeviceOrdinal
{
    enum Mode
    {
        eMODE_PHYSX_DEFAULTS = 0, //!< Let Physx decides automonously which ordinal to use
        eMODE_ACTIVE_CONTEXT = 1, //!< Read the ordinal indicated by kSettingUseActiveCudaContext persistent setting
        eMODE_DEVICE_ORDINAL = 2 //!< Use a custom device ordinal written inside deviceOrdinal field
    };
    Mode mode = eMODE_DEVICE_ORDINAL;
    int32_t deviceOrdinal = -1; //!< ordinal to be used when mode == eMODE_DEVICE_ORDINAL
};

struct IPhysxFoundation
{
    /// Fills PhysxFoundationDeviceOrdinal with settings from kSettingUseActiveCudaContext or kSettingCudaDevice ordinal
    void(CARB_ABI* getSingleCudaContextManagerOrdinal)(PhysxFoundationDeviceOrdinal& ordinal);

    /// Force CPU-only mode for PhysX initialization.
    /// When enabled, omni.physx must not attempt to create CUDA context managers nor enable GPU dynamics/DirectGPU.
    ///
    /// Intended for kit-less SDK bootstraps (e.g. ovphysx) that must remain loadable on systems without an NVIDIA
    /// driver and that may need to force CPU simulation even when a driver is present.
    ///
    /// This is a process-global switch and should be set before PhysX is created. Toggling after initialization is
    /// not guaranteed to fully tear down GPU state.
    void(CARB_ABI* setCpuMode)(bool enabled);

    /// Returns true if CPU-only mode is enabled.
    bool(CARB_ABI* isCpuMode)();

    /// Checks for presence of at least one suitable CUDA device to prevent crashing
    bool(CARB_ABI* cudaDeviceCheck)();

    /// @deprecated No-op stub retained for ABI compatibility. GPU Foundation dependency was removed;
    /// CUDA availability is now determined directly via the CUDA driver shim. Do not call.
    void(CARB_ABI* createGpuFoundation)();

    /// @deprecated No-op stub retained for ABI compatibility. See createGpuFoundation.
    void(CARB_ABI* releaseGpuFoundation)();

    /// Set CUDA device and create or refresh a PxCudaContextManager for a given PhysxFoundationDeviceOrdinal mode setting
    bool(CARB_ABI* createOrRefreshPxCudaContextManager)(PhysxFoundationDeviceOrdinal preferences,
                                                        ::physx::PxFoundation* pxFoundation,
                                                        ::physx::PxCudaContextManager*& pxCudaContextManager,
                                                        const bool enableSynchronousKernelLaunches);

    /// Set a current CUDA device with cudainterop->setDevice
    void(CARB_ABI* setCudaDevice)(int deviceOrdinal);
};

} // namespace physx
} // namespace omni
