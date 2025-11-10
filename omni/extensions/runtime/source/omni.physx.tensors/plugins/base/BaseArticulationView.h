// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../ArticulationMetatype.h"
#include "../CommonTypes.h"
#include "BaseSimulationData.h"

#include <omni/physics/tensors/IArticulationView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseArticulationView : public omni::physics::tensors::IArticulationView
{
public:
    BaseArticulationView(BaseSimulationView* sim, const std::vector<ArticulationEntry>& entries);

    ~BaseArticulationView() override;

    //
    // public API
    //

    uint32_t getCount() const override;
    uint32_t getMaxLinks() const override;
    uint32_t getMaxDofs() const override;
    uint32_t getMaxShapes() const override;
    uint32_t getMaxFixedTendons() const override;
    uint32_t getMaxSpatialTendons() const override;

    const char* getUsdPrimPath(uint32_t artiIdx) const override;
    const char* getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const override;
    const char* getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const override;

    // articulation type info
    bool isHomogeneous() const override;
    const ArticulationMetatype* getSharedMetatype() const override;
    const ArticulationMetatype* getMetatype(uint32_t artiIdx) const override;

    bool getJacobianShape(uint32_t* numRows, uint32_t* numCols) const override;
    bool getMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const override; // deprecated
    bool getGeneralizedMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const override;

    // DOF properties
    bool getDofTypes(const TensorDesc* dstTensor) const override;
    bool getDofMotions(const TensorDesc* dstTensor) const override;
    bool getDofLimits(const TensorDesc* dstTensor) const override;
    bool getDriveTypes(const TensorDesc* dstTensor) const override;
    bool getDofStiffnesses(const TensorDesc* dstTensor) const override;
    bool getDofDampings(const TensorDesc* dstTensor) const override;
    bool getDofMaxForces(const TensorDesc* dstTensor) const override;
    bool getDofDriveModelProperties(const TensorDesc* dstTensor) const override;
    bool getDofFrictionCoefficients(const TensorDesc* dstTensor) const override; // deprecated
    bool getDofFrictionProperties(const TensorDesc* dstTensor) const override;
    bool getDofMaxVelocities(const TensorDesc* dstTensor) const override;
    bool getDofArmatures(const TensorDesc* dstTensor) const override;

    bool setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofDriveModelProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofFrictionCoefficients(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override; // deprecated
    bool setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    // rigid body properties
    bool getMasses(const TensorDesc* dstTensor) const override;
    bool getInvMasses(const TensorDesc* dstTensor) const override;
    bool getCOMs(const TensorDesc* dstTensor) const override;
    bool getInertias(const TensorDesc* dstTensor) const override;
    bool getInvInertias(const TensorDesc* dstTensor) const override;
    bool getDisableGravities(const TensorDesc* srcTensor) const override;

    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;


    // materials/shapes
    bool getMaterialProperties(const TensorDesc* dstTensor) const override;
    bool getRestOffsets(const TensorDesc* dstTensor) const override;
    bool getContactOffsets(const TensorDesc* dstTensor) const override;

    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;

    bool check() const override;

    void release() override;

    //
    // helpers
    //

    void _onParentRelease();


protected:
    BaseSimulationView* mSim = nullptr;
    BaseSimulationDataPtr mSimData;

    std::vector<ArticulationEntry> mEntries;
    std::vector<uint32_t> mAllIndices;

    uint32_t mMaxLinks = 0;
    uint32_t mMaxDofs = 0;
    uint32_t mMaxShapes = 0;
    uint32_t mMaxFixedTendons = 0;
    uint32_t mMaxSpatialTendons = 0;

    // whether all articulations have the same metatype
    bool mIsHomogeneous = false;
    bool validComsCache = true;

    void setComsCacheStateValid(bool flag)
    {
        validComsCache = flag;
    };
    bool getComsCacheStateValid()
    {
        return validComsCache;
    };
};

} // namespace tensors
} // namespace physx
} // namespace omni
