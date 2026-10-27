// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CUDA_CONTEXT_MANAGER_H
#define CUDA_CONTEXT_MANAGER_H

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

namespace physx
{

class PxCudaContextManager;
class PxCudaContextManagerDesc;
class PxErrorCallback;

/**
Creates cuda context manager for PhysX and APEX.
Set launchSynchronous to true for Cuda to report the actual point of failure
*/
PxCudaContextManager* createCudaContextManager(const PxCudaContextManagerDesc& desc, PxErrorCallback& errorCallback, bool launchSynchronous);

}

#endif

#endif

