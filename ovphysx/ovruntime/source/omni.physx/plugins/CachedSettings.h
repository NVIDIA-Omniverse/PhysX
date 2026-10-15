// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <omni/physx/AttachOptions.h>

// @implements REQ-ATTACH-SERVICES-001
// @covers AC-3
namespace omni::physx
{
// Process defaults are copied when an attachment is constructed. Stage consumers
// read the owner's copy; process services retain their own configuration.
struct OmniCachedSettings : PhysxAttachmentSettings
{
    bool localMeshCacheEnabled = false;
    int localMeshCacheSize = 0;
    bool enableSynchronousKernelLaunches = false;
};
}
