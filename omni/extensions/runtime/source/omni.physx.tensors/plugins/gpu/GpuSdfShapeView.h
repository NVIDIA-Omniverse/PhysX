// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSdfShapeView.h"
#include "GpuSimulationData.h"

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuSdfShapeView : public BaseSdfShapeView
{
public:
    GpuSdfShapeView(GpuSimulationView* sim, const std::vector<SdfShapeEntry>& entries, int device);

    bool getSdfAndGradients(const TensorDesc* dstTensor, const TensorDesc* srcPointTensor) const override;

    ~GpuSdfShapeView() override;

private:
    GpuSimulationDataPtr mGpuSimData;

    int mDevice = -1;
    std::unordered_map<::physx::PxShapeGPUIndex, ::physx::PxU32> mShapeToViewIndexMap;

    // indexing data for all sdf shapes
    std::vector<GpuSdfShapeRecord> mSdfRecords;
    GpuSdfShapeRecord* mSdfRecordsDev = nullptr;

    // phyx shape indices
    ::physx::PxShapeGPUIndex* mShapeIndicesD;
    std::vector<::physx::PxShapeGPUIndex> mShapeIndices;

    // phyx sample points per shapes
    ::physx::PxU32* samplePointCountsD;
    std::vector<::physx::PxU32> samplePointCounts;

    ::physx::PxU32 mMaxNumPoints = 0;
    ::physx::PxVec4* samplePointsPerShape;
};
} // namespace tensors
} // namespace physx
} // namespace omni
