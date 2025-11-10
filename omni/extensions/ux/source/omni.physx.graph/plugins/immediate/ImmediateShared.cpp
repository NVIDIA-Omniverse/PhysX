// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <carb/tasking/ITasking.h>
#include <carb/logging/Log.h>
#include "ImmediateShared.h"

using namespace omni::physx::graph;
using namespace ::physx;
bool ImmediateShared::initialize()
{
    refCount++;
    if (refCount != 1)
    {
        return true; // already initialized
    }
    carbTasking = carb::getCachedInterface<carb::tasking::ITasking>();
    if (!carbTasking)
    {
        CARB_LOG_ERROR("ImmediateShared::Couldn't get tasking system");
        return false;
    }

    pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, pxAllocator, pxErrorCallback);
    if (!pxFoundation)
    {
        CARB_LOG_ERROR("ImmediateShared::Couldn't get PhysX Foundation");
        return false;
    }

    immediateMeshCache.initialize(*pxFoundation);
    return true;
}

void ImmediateShared::release()
{
    refCount--;
    if (refCount != 0)
    {
        return;
    }

    if (carbTasking)
    {
        carb::getFramework()->releaseInterface<carb::tasking::ITasking>(carbTasking);
        carbTasking = nullptr;
    }

    immediateMeshCache.release();

    // Foundation must be deleted as last object
    if (pxFoundation)
    {
        pxFoundation->release();
        pxFoundation = nullptr;
    }
}
