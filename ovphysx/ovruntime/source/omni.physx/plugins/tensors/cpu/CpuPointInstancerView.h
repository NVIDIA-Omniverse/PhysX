// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BasePointInstancerView.h"

namespace omni
{
namespace physx
{
namespace tensors
{

class CpuSimulationView;

// Host instancer view: the base owns the reframe and every whole-view getter, since CPU actor accessors
// are live. This adds only the ovstage column fill, which writes the reader's host buffer directly.
class CpuPointInstancerView : public BasePointInstancerView
{
public:
    CpuPointInstancerView(CpuSimulationView* sim, const std::vector<PointInstancerEntry>& entries);

    bool getInstancerColumnsOvStage(InstancerColumn column,
                                    void* dst,
                                    const ::physx::PxU32* offsets,
                                    ::physx::PxU32 numInstancers) const override;
    bool setInstancerColumnOvStage(InstancerColumn column,
                                   ::physx::PxU32 instancerIndex,
                                   const void* src,
                                   ::physx::PxU32 arrayLength) override;

};

} // namespace tensors
} // namespace physx
} // namespace omni
