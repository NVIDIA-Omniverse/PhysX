// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSoftBodyView.h"
#include "CpuSimulationData.h"

#include <omni/physics/tensors/ISoftBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuSoftBodyView : public BaseSoftBodyView
{
public:
    CpuSoftBodyView(CpuSimulationView* sim, const std::vector<SoftBodyEntry>& entries);

    ~CpuSoftBodyView() override;

    bool getElementIndices(const TensorDesc* dstTensor) const override;
    bool getSimElementIndices(const TensorDesc* dstTensor) const override;

    bool getNodalPositions(const TensorDesc* dstTensor) const override;
    bool getSimNodalPositions(const TensorDesc* dstTensor) const override;

    bool getElementStresses(const TensorDesc* dstTensor) const override;
    bool getSimElementStresses(const TensorDesc* dstTensor) const override;

    bool getElementRestPoses(const TensorDesc* dstTensor) const override;
    bool getSimElementRestPoses(const TensorDesc* dstTensor) const override;

    bool getElementRotations(const TensorDesc* dstTensor) const override;
    bool getSimElementRotations(const TensorDesc* dstTensor) const override;

    bool getElementDeformationGradients(const TensorDesc* dstTensor) const override;
    bool getSimElementDeformationGradients(const TensorDesc* dstTensor) const override;

    bool getSimKinematicTargets(const TensorDesc* dstTensor) const override;
    bool getSimNodalVelocities(const TensorDesc* dstTensor) const override;

    bool setSimNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setSimNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setSimKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

private:
    CpuSimulationDataPtr mCpuSimData;

    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni
