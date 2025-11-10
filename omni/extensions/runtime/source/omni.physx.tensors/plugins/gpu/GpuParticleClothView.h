// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseParticleClothView.h"
#include "GpuSimulationData.h"

#include <omni/physics/tensors/IParticleClothView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuParticleClothView : public BaseParticleClothView
{
public:
    GpuParticleClothView(GpuSimulationView* sim, const std::vector<ParticleClothEntry>& entries, int device);

    ~GpuParticleClothView() override;

    bool getPositions(const TensorDesc* dstTensor) const override;
    bool setPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool getVelocities(const TensorDesc* dstTensor) const override;
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool getMasses(const TensorDesc* dstTensor) const override;
    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool getSpringDamping(const TensorDesc* dstTensor) const override;
    bool setSpringDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool getSpringStiffness(const TensorDesc* dstTensor) const override;
    bool setSpringStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

private:
    int mDevice = -1;
    GpuSimulationDataPtr mGpuSimData;
    bool applyParticleCloth();

    uint32_t mNumCloths = 0;

    SingleAllocPolicy mPcIndexSingleAllocPolicy;

    // indexing data for all cloth buffers
    GpuParticleClothRecord* mPcRecordsDev = nullptr;
    // all indices in this view
    ::physx::PxU32* mClothIndicesDev = nullptr;
    ::physx::PxParticleBufferFlags* mPcDirtyFlagsDev = nullptr; // which particle buffers have been modified in the
                                                                // view. size mNumCloths, indexed by global index
    ::physx::PxU32* mDirtyPcIndicesDev = nullptr; // index into indexPair list, global index
    ::physx::PxGpuParticleBufferIndexPair* mPcIndexPairsDev = nullptr; // list of indexpairs, indexed by global index.
    std::unordered_map<uint64_t, uint32_t> mIndexPair2ClothMap;
};

} // namespace tensors
} // namespace physx
} // namespace omni
