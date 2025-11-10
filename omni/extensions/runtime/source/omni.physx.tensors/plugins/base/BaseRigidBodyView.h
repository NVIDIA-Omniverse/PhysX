// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseSimulationData.h"

#include <omni/physics/tensors/IRigidBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseRigidBodyView : public omni::physics::tensors::IRigidBodyView
{
public:
    BaseRigidBodyView(BaseSimulationView* sim, const std::vector<RigidBodyEntry>& entries);

    virtual ~BaseRigidBodyView() override;

    uint32_t getCount() const override;
    uint32_t getMaxShapes() const override;

    const char* getUsdPrimPath(uint32_t rbIdx) const override;

    bool check() const override;

    void release() override;

    // rigid body properties
    bool getMasses(const TensorDesc* dstTensor) const override;
    bool getInvMasses(const TensorDesc* dstTensor) const override;
    bool getCOMs(const TensorDesc* dstTensor) const override;
    bool getInertias(const TensorDesc* dstTensor) const override;
    bool getInvInertias(const TensorDesc* dstTensor) const override;
    bool getDisableGravities(const TensorDesc* dstTensor) const override;
    bool getDisableSimulations(const TensorDesc* dstTensor) const override;

    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    // materials/shapes
    bool getMaterialProperties(const TensorDesc* dstTensor) const override;
    bool getRestOffsets(const TensorDesc* dstTensor) const override;
    bool getContactOffsets(const TensorDesc* dstTensor) const override;

    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;

    void setComsCacheStateValid(bool flag)
    {
        validComsCache = flag;
    };
    bool getComsCacheStateValid()
    {
        return validComsCache;
    };
    //
    // helpers
    //

    void _onParentRelease();

protected:
    BaseSimulationView* mSim = nullptr;

    BaseSimulationDataPtr mSimData;

    std::vector<RigidBodyEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
    bool validComsCache = true;
    uint32_t mMaxShapes = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni
