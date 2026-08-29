// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <carb/extras/Library.h>

namespace omni
{
namespace physx
{
namespace gpu
{

class PhysXGpuLoader final
{
public:
    PhysXGpuLoader() = default;
    ~PhysXGpuLoader();

    PhysXGpuLoader(const PhysXGpuLoader&) = delete;
    PhysXGpuLoader& operator=(const PhysXGpuLoader&) = delete;
    PhysXGpuLoader(PhysXGpuLoader&&) = delete;
    PhysXGpuLoader& operator=(PhysXGpuLoader&&) = delete;

    bool load();
    bool loadFromDirectory(const char* directory);
    void unload();

    bool isLoaded() const;

private:
    // load() probes several package layouts. Keep those probes quiet and log
    // one final error only after every candidate directory fails.
    bool loadFromDirectory(const char* directory, bool logErrors);

    carb::extras::LibraryHandle mPhysxGpuHandle{ nullptr };
    carb::extras::LibraryHandle mPhysxDeviceHandle{ nullptr };
};

} // namespace gpu
} // namespace physx
} // namespace omni
