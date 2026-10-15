// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-4

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-4
#pragma once
#include <omni/physx/IPhysx.h>
#include <map>
namespace omni::physx
{
struct StepEventSub
{
    AttachHandle attach;
    std::function<void(float, void*)> stepFn;
    void* userData;
};
struct SimulationEventSub
{
    AttachHandle attach;
    std::function<void(SimulationStatusEvent, void*)> eventFn;
    void* userData;
};
using StepEventMap = std::map<std::pair<int, SubscriptionId>, StepEventSub>;
using SimulationEventMap = std::map<SubscriptionId, SimulationEventSub>;
}
