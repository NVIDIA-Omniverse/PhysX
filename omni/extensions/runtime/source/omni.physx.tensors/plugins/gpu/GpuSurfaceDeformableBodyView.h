// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "../base/BaseSurfaceDeformableBodyView.h"
#include "GpuSimulationData.h"
#include "PxDeformableVolumeFlag.h"

#include <omni/physics/tensors/IDeformableBodyView.h>
#include <omni/physics/tensors/TensorUtils.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuSurfaceDeformableBodyView : public BaseSurfaceDeformableBodyView
{
public:
    GpuSurfaceDeformableBodyView(GpuSimulationView* sim, const std::vector<DeformableBodyEntry>& entries, int device);
    ~GpuSurfaceDeformableBodyView() override;

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
    int mDevice = -1;
    GpuSimulationDataPtr mGpuSimData;
    using checkTensorType = std::function<bool(const TensorDesc& tensor, const char* tensorName, const char* funcName)>;

    void resizeDeviceBuffer(::physx::PxU32 previousSize,
                            ::physx::PxU32 currentSize,
                            ::physx::PxU32 dataSize,
                            void* previousBufferDev);

    bool fetchData(const TensorDesc* dstTensor,
                   const DeformableBodyData::Enum dataFlag,
                   const uint32_t maxItemsPerBody,
                   const uint32_t itemSize,
                   const char* tensorName,
                   const char* callingFunctionName,
                   const checkTensorType checkTensorBaseType) const;

    bool submitData(const TensorDesc* srcTensor,
                    const TensorDesc* indexTensor,
                    const DeformableBodyData::Enum dataFlag,
                    const uint32_t maxItemsPerBody,
                    const uint32_t itemSize,
                    const char* tensorName,
                    const char* callingFunctionName,
                    const checkTensorType checkTensorBaseType);

    // indexing data for all deformable body buffers
    std::vector<GpuDeformableBodyRecord> mDeformableBodyRecords;
    GpuDeformableBodyRecord* mDeformableBodyRecordsD = nullptr;

    // all indices in this view
    ::physx::PxU32* mViewIndicesD = nullptr;
    std::vector<::physx::PxU32> mViewIndices;

    DeformableBodyBufferManager mSimElementIndices;
    DeformableBodyBufferManager mRestNodalPositions;
};

} // namespace tensors
} // namespace physx
} // namespace omni
