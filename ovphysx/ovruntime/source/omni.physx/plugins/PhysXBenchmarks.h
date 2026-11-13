// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-PUBLICAPI-001
// @covers AC-52
#pragma once

#include "PhysXUpdate.h"
#include "usdBridge/RuntimeBridge.h"
#include <private/omni/physx/IPhysxTests.h>
#include <vector>

// Internal harness helpers. Callers initialize and retain the runtime owner.
// Stage loading and stepping use the native bridge/update entries included above.
namespace omni::physx::benchmarks
{
// Change defaults for future attachments; live attachments retain their snapshots.
void setThreadCount(uint32_t threadCount);
void overwriteGPUSetting(bool enableGPU);
void enablePVDProfile(bool enablePVD);
void enableProfile(bool enableProfile);
// Drains the collected profile records, preserving their existing order.
void getProfileStats(std::vector<PhysicsProfileStats>& stats);
SubscriptionId subscribeProfileStatsEvents(ProfileStatsNotificationFn onEvent, void* userData);
void unsubscribeProfileStatsEvents(SubscriptionId id);
} // namespace omni::physx::benchmarks
