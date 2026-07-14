// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#define USE_PHYSX_GPU 1 // GPU Rigid Bodies

#include <PxPhysicsAPI.h>

namespace omni
{
namespace physx
{
struct IPhysxCooking;
struct IPhysxCookingServicePrivate;
struct IPhysxCookingService;
}
}

class CookingTest
{
public:

    static CookingTest* getCookingTests();

    void release();

    omni::physx::IPhysxCooking* acquireCookingInterface();

    omni::physx::IPhysxCookingServicePrivate* acquireCookingServicePrivateInterface();

    omni::physx::IPhysxCookingService* acquireCookingServiceInterface();

    carb::AppScoped* getApp() { return mApp; }

private:
    CookingTest();

    ~CookingTest();

    carb::AppScoped*    mApp;
};
