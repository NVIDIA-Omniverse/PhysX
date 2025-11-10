// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSurfaceDeformableBodyView.h"
#include "CpuSimulationData.h"

#include <omni/physics/tensors/IDeformableBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class CpuSimulationView;

class CpuSurfaceDeformableBodyView : public BaseSurfaceDeformableBodyView
{
public:
    CpuSurfaceDeformableBodyView(CpuSimulationView* sim, const std::vector<DeformableBodyEntry>& entries);

    ~CpuSurfaceDeformableBodyView() override;

    // simulation mesh
    virtual bool getSimulationElementIndices(const TensorDesc* dstTensor) const override;
    virtual bool getSimulationNodalPositions(const TensorDesc* dstTensor) const override;
    virtual bool setSimulationNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    virtual bool getSimulationNodalVelocities(const TensorDesc* dstTensor) const override;
    virtual bool setSimulationNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    virtual bool getSimulationNodalKinematicTargets(const TensorDesc* dstTensor) const override;
    virtual bool setSimulationNodalKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    
    // rest shape
    virtual bool getRestElementIndices(const TensorDesc* dstTensor) const override;
    virtual bool getRestNodalPositions(const TensorDesc* dstTensor) const override;

    // collision mesh
    virtual bool getCollisionElementIndices(const TensorDesc* dstTensor) const override;
    virtual bool getCollisionNodalPositions(const TensorDesc* dstTensor) const override;

private:
    CpuSimulationDataPtr mCpuSimData;

    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni
