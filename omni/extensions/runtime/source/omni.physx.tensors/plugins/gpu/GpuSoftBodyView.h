// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../base/BaseSoftBodyView.h"
#include "GpuSimulationData.h"
#include "PxSoftBodyFlag.h"

#include <omni/physics/tensors/ISoftBodyView.h>
#include <omni/physics/tensors/TensorUtils.h>
using omni::physics::tensors::checkTensorFloat32;

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuSoftBodyView : public BaseSoftBodyView
{
public:
    GpuSoftBodyView(GpuSimulationView* sim, const std::vector<SoftBodyEntry>& entries, int device);
    ~GpuSoftBodyView() override;

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
    int mDevice = -1;
    GpuSimulationDataPtr mGpuSimData;
    using checkTensorType = std::function<bool(const TensorDesc& tensor, const char* tensorName, const char* funcName)>;

    void resizeDeviceBuffer(::physx::PxU32 previousSize,
                            ::physx::PxU32 currentSize,
                            ::physx::PxU32 dataSize,
                            void* previousBufferDev);
    bool fetchPxValues(const TensorDesc* dstTensor,
                       ::physx::PxSoftBodyGpuDataFlag::Enum flag,
                       SoftBodyData::Enum dataType,
                       ::physx::PxU32 copyEvent,
                       const char* name,
                       ::physx::PxU32 expectedSize,
                       void** mBuffersD,
                       ::physx::PxU32* mBufferSizesD,
                       ::physx::PxU32* mSbIndicesD,
                       ::physx::PxU32 numSb,
                       ::physx::PxU32 mMaxBufferSize,
                       ::physx::PxU32 mMaxElementsOrVerticesPerBody,
                       const char* callingFunction,
                       checkTensorType callBack = checkTensorFloat32) const;

    bool submitPxValues(const TensorDesc* srcTensor,
                        const TensorDesc* indexTensor,
                        ::physx::PxSoftBodyGpuDataFlag::Enum dataType,
                        const char* callingFunction,
                        const char* tensorName,
                        bool transform,
                        bool kinematicTarget);

    void copySoftBodyData(::physx::PxSoftBodyGpuDataFlag::Enum flag,
                          ::physx::PxU32 copyEvent,
                          void** mBuffersD,
                          ::physx::PxU32* mBufferSizesD,
                          ::physx::PxU32* mIndicesD,
                          ::physx::PxU32 size,
                          ::physx::PxU32 mMaxBufferSize) const;


    // indexing data for all soft body buffers
    std::vector<GpuSoftBodyRecord> mSbRecords;
    GpuSoftBodyRecord* mSbRecordsDev = nullptr;

    // all indices in this view
    ::physx::PxU32* mViewIndicesD = nullptr;
    std::vector<::physx::PxU32> mViewIndices;

    // phyx host soft body indices
    ::physx::PxU32* mSbIndicesD;
    std::vector<::physx::PxU32> mSbIndices;

    SoftBodyBufferManager mNodalPositions;
    SoftBodyBufferManager mElementRestPoses;
    SoftBodyBufferManager mElementRotations;
    SoftBodyBufferManager mElementIndices;

    SoftBodyBufferManager mSimElementRotations;
    SoftBodyBufferManager mSimElementIndices;
    SoftBodyBufferManager mSimNodalValues;
    SoftBodyBufferManager mSimKinematicTargets;

    ::physx::PxVec4* materialPropertiesDev;
    ::physx::PxFEMSoftBodyMaterialModel::Enum* materialModelsDev;
};

} // namespace tensors
} // namespace physx
} // namespace omni
