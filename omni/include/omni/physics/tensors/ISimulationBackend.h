// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physics
{
namespace tensors
{
class ISimulationView;

class ISimulationBackend
{
public:
    virtual ISimulationView* createSimulationView(long stageId = -1) = 0;

    // call when simulation stops to release internal resources
    virtual void reset() = 0;

protected:
    virtual ~ISimulationBackend() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
