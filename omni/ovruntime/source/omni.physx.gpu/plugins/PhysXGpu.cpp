// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#define CARB_EXPORTS

#include <omni/physx/IPhysxGpu.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/extras/Library.h>
#include <carb/Format.h>
#include <carb/logging/Log.h>

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.gpu.plugin", "Physics GPU", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxGpu)
CARB_PLUGIN_IMPL_NO_DEPS()

carb::extras::LibraryHandle gPhysxGPUHandle{ nullptr };
carb::extras::LibraryHandle gPhysxDeviceHandle{ nullptr };

static std::string getBinaryLibraryPath(const char* suffix)
{
    return carb::fmt::format(
        "{}/{}", carb::extras::getLibraryDirectory(reinterpret_cast<void*>(getBinaryLibraryPath)), suffix);
}

CARB_EXPORT void carbOnPluginStartup()
{
#if CARB_PLATFORM_WINDOWS
    {
        const std::string path = getBinaryLibraryPath("PhysXDevice64");
        gPhysxDeviceHandle = carb::extras::loadLibrary(path.c_str(), carb::extras::fLibFlagMakeFullLibName);
        if (!gPhysxDeviceHandle)
        {
            CARB_LOG_ERROR("omni.physx.gpu: failed to load PhysXDevice64 from '%s'", path.c_str());
            return;
        }
    }
#endif
    {
        const std::string path = getBinaryLibraryPath("PhysXGpu_64");
        gPhysxGPUHandle = carb::extras::loadLibrary(path.c_str(), carb::extras::fLibFlagMakeFullLibName);
        if (!gPhysxGPUHandle)
        {
            CARB_LOG_ERROR("omni.physx.gpu: failed to load PhysXGpu_64 from '%s'", path.c_str());
#if CARB_PLATFORM_WINDOWS
            carb::extras::unloadLibrary(gPhysxDeviceHandle);
            gPhysxDeviceHandle = nullptr;
#endif
            return;
        }
    }
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if (gPhysxGPUHandle)
    {
        carb::extras::unloadLibrary(gPhysxGPUHandle);
        gPhysxGPUHandle = nullptr;
    }
    if (gPhysxDeviceHandle)
    {
        carb::extras::unloadLibrary(gPhysxDeviceHandle);
        gPhysxDeviceHandle = nullptr;
    }
}

void fillInterface(omni::physx::IPhysxGpu&)
{
}
