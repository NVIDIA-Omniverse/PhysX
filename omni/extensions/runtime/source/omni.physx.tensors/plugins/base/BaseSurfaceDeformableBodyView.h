// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/IDeformableBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseSurfaceDeformableBodyView : public omni::physics::tensors::IDeformableBodyView
{
public:
    BaseSurfaceDeformableBodyView(BaseSimulationView* sim, const std::vector<DeformableBodyEntry>& entries);

    virtual ~BaseSurfaceDeformableBodyView() override;

    uint32_t getCount() const override;

    const char* getUsdPrimPath(uint32_t dbIdx) const override;
    const char* getUsdSimulationMeshPrimPath(uint32_t dbIdx) const override;
    const char* getUsdCollisionMeshPrimPath(uint32_t dbIdx) const override;

    bool getTransforms(const TensorDesc* dstTensor) const override;

    uint32_t getNumNodesPerElement() const override
    {
        return 3u;
    }

    uint32_t getMaxSimulationElementsPerBody() const override;
    uint32_t getMaxSimulationNodesPerBody() const override;

    uint32_t getMaxRestNodesPerBody() const override;
    
    uint32_t getMaxCollisionElementsPerBody() const override;
    uint32_t getMaxCollisionNodesPerBody() const override;

    bool check() const override;
    void release() override;

    //
    // helpers
    //

    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:
    BaseSimulationDataPtr mSimData;

    uint32_t mMaxSimElementsPerBody;
    uint32_t mMaxSimNodesPerBody;
    uint32_t mMaxRestNodesPerBody;

    std::vector<DeformableBodyEntry> mEntries;
};

} // namespace tensors
} // namespace physx
} // namespace omni
