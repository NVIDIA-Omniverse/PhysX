// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <carb/Defines.h>
#include <carb/Types.h>

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
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxFoundation", 0, 1)

    /// Fills PhysxFoundationDeviceOrdinal with settings from kSettingUseActiveCudaContext or kSettingCudaDevice ordinal
    void(CARB_ABI* getSingleCudaContextManagerOrdinal)(PhysxFoundationDeviceOrdinal& ordinal);

    /// Checks for presence of at least one suitable CUDA device to prevent crashing
    bool(CARB_ABI* cudaDeviceCheck)();

    /// Create GPU foundation and devices. Use only when not already created by e.g.
    /// omni::kit::renderer::IGpuFoundation, otherwise use getters provided there.
    void(CARB_ABI* createGpuFoundation)();

    /// Release instances created by createGpuFoundation. Ran on plugin shutdown and usually not needed to call
    /// manually.
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
