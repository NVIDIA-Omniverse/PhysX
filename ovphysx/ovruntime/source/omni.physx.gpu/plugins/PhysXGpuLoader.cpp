// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "PhysXGpuLoader.h"

#include <carb/Format.h>
#include <carb/extras/Library.h>
#include <carb/logging/Log.h>

#include <array>
#include <string>

namespace omni
{
namespace physx
{
namespace gpu
{
namespace
{

static void physXGpuLoaderAnchor()
{
}

static std::string getBinaryLibraryPath(const char* directory, const char* suffix)
{
    return carb::fmt::format("{}/{}", directory, suffix);
}

// OvruntimePhysXGpu is a static library in ovphysx, so this code becomes part
// of the final libovphysx shared library. Use an address from this object file
// to find the libovphysx directory, then look next to it for the packaged
// PhysX GPU runtime DLLs/SOs.
static std::string getLoaderLibraryDirectory()
{
    return carb::extras::getLibraryDirectory(reinterpret_cast<void*>(physXGpuLoaderAnchor));
}

} // namespace

PhysXGpuLoader::~PhysXGpuLoader()
{
    unload();
}

bool PhysXGpuLoader::load()
{
    const std::string directory = getLoaderLibraryDirectory();
    const std::array<std::string, 3> directories = {
        directory,
        directory + "/../plugins",
        directory + "/../plugins/gpu",
    };
    for (const std::string& candidate : directories)
    {
        if (loadFromDirectory(candidate.c_str(), false))
        {
            return true;
        }
    }
    CARB_LOG_ERROR("omni.physx.gpu: failed to load PhysXGpu_64 from '%s', '%s', or '%s'", directories[0].c_str(),
                   directories[1].c_str(), directories[2].c_str());
    return false;
}

bool PhysXGpuLoader::loadFromDirectory(const char* directory)
{
    return loadFromDirectory(directory, true);
}

bool PhysXGpuLoader::loadFromDirectory(const char* directory, bool logErrors)
{
    if (isLoaded())
    {
        return true;
    }

#if CARB_PLATFORM_WINDOWS
    {
        const std::string path = getBinaryLibraryPath(directory, "PhysXDevice64");
        mPhysxDeviceHandle = carb::extras::loadLibrary(path.c_str(), carb::extras::fLibFlagMakeFullLibName);
        if (!mPhysxDeviceHandle)
        {
            if (logErrors)
            {
                CARB_LOG_ERROR("omni.physx.gpu: failed to load PhysXDevice64 from '%s'", path.c_str());
            }
            return false;
        }
    }
#endif

    {
        const std::string path = getBinaryLibraryPath(directory, "PhysXGpu_64");
        mPhysxGpuHandle = carb::extras::loadLibrary(path.c_str(), carb::extras::fLibFlagMakeFullLibName);
        if (!mPhysxGpuHandle)
        {
            if (logErrors)
            {
                CARB_LOG_ERROR("omni.physx.gpu: failed to load PhysXGpu_64 from '%s'", path.c_str());
            }
#if CARB_PLATFORM_WINDOWS
            carb::extras::unloadLibrary(mPhysxDeviceHandle);
            mPhysxDeviceHandle = nullptr;
#endif
            return false;
        }
    }

    return true;
}

void PhysXGpuLoader::unload()
{
    if (mPhysxGpuHandle)
    {
        carb::extras::unloadLibrary(mPhysxGpuHandle);
        mPhysxGpuHandle = nullptr;
    }
    if (mPhysxDeviceHandle)
    {
        carb::extras::unloadLibrary(mPhysxDeviceHandle);
        mPhysxDeviceHandle = nullptr;
    }
}

bool PhysXGpuLoader::isLoaded() const
{
    return mPhysxGpuHandle != nullptr;
}

} // namespace gpu
} // namespace physx
} // namespace omni
