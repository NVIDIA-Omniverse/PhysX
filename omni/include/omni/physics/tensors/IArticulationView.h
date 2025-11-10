// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "IArticulationMetatype.h"
#include "TensorDesc.h"

namespace omni
{
namespace physics
{
namespace tensors
{

class IArticulationView
{
public:
    virtual uint32_t getCount() const = 0;
    virtual uint32_t getMaxLinks() const = 0;
    virtual uint32_t getMaxDofs() const = 0;
    virtual uint32_t getMaxShapes() const = 0;
    virtual uint32_t getMaxFixedTendons() const = 0;
    virtual uint32_t getMaxSpatialTendons() const = 0;

    // articulation type info
    virtual bool isHomogeneous() const = 0;
    virtual const IArticulationMetatype* getSharedMetatype() const = 0;
    virtual const IArticulationMetatype* getMetatype(uint32_t artiIdx) const = 0;

    // usd paths
    virtual const char* getUsdPrimPath(uint32_t artiIdx) const = 0;
    virtual const char* getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const = 0;
    virtual const char* getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const = 0;

    // DOF properties
    virtual bool getDofTypes(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofMotions(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofLimits(const TensorDesc* dstTensor) const = 0;
    virtual bool getDriveTypes(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofStiffnesses(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofDampings(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofMaxForces(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofDriveModelProperties(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofFrictionCoefficients(const TensorDesc* dstTensor) const = 0; // deprecated
    virtual bool getDofFrictionProperties(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofMaxVelocities(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofArmatures(const TensorDesc* dstTensor) const = 0;

    virtual bool setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofDriveModelProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofFrictionCoefficients(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0; // deprecated
    virtual bool setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getDofActuationForces(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofProjectedJointForces(const TensorDesc* dstTensor) const = 0;

    virtual bool getLinkTransforms(const TensorDesc* dstTensor) const = 0;
    virtual bool getLinkVelocities(const TensorDesc* dstTensor) const = 0;
    virtual bool getLinkAccelerations(const TensorDesc* dstTensor) const = 0;

    virtual bool getRootTransforms(const TensorDesc* dstTensor) const = 0;
    virtual bool getRootVelocities(const TensorDesc* dstTensor) const = 0;

    virtual bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getDofPositions(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofVelocities(const TensorDesc* dstTensor) const = 0;

    virtual bool setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getDofPositionTargets(const TensorDesc* dstTensor) const = 0;
    virtual bool getDofVelocityTargets(const TensorDesc* dstTensor) const = 0;

    virtual bool getJacobianShape(uint32_t* numRows, uint32_t* numCols) const = 0;
    virtual bool getJacobians(const TensorDesc* dstTensor) const = 0;

    virtual bool getMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const = 0; // deprecated
    virtual bool getMassMatrices(const TensorDesc* dstTensor) const = 0; // deprecated
    virtual bool getGeneralizedMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const = 0;
    virtual bool getGeneralizedMassMatrices(const TensorDesc* dstTensor) const = 0;

    virtual bool getCoriolisAndCentrifugalForces(const TensorDesc* dstTensor) const = 0; // deprecated
    virtual bool getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const = 0;
    virtual bool getGeneralizedGravityForces(const TensorDesc* dstTensor) const = 0; // deprecated
    virtual bool getGravityCompensationForces(const TensorDesc* dstTensor) const = 0;

    virtual bool getLinkIncomingJointForce(const TensorDesc* dstTensor) const = 0;
    virtual bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                 const TensorDesc* srcTorqueTensor,
                                                 const TensorDesc* srcPositionTensor,
                                                 const TensorDesc* indexTensor,
                                                 const bool isGlobal) = 0;
    // rigid body properties
    virtual bool getMasses(const TensorDesc* dstTensor) const = 0;
    virtual bool getInvMasses(const TensorDesc* dstTensor) const = 0;
    virtual bool getCOMs(const TensorDesc* dstTensor) const = 0;
    virtual bool getInertias(const TensorDesc* dstTensor) const = 0;
    virtual bool getInvInertias(const TensorDesc* dstTensor) const = 0;
    virtual bool getDisableGravities(const TensorDesc* dstTensor) const = 0;
    virtual bool getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const = 0;
    virtual bool getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const = 0;

    virtual bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;


    // materials/shapes
    virtual bool getMaterialProperties(const TensorDesc* dstTensor) const = 0;
    virtual bool getRestOffsets(const TensorDesc* dstTensor) const = 0;
    virtual bool getContactOffsets(const TensorDesc* dstTensor) const = 0;

    virtual bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const = 0;
    virtual bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const = 0;
    virtual bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const = 0;

    // tendons
    virtual bool getFixedTendonStiffnesses(const TensorDesc* dstTensor) const = 0;
    virtual bool getFixedTendonDampings(const TensorDesc* dstTensor) const = 0;
    virtual bool getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const = 0;
    virtual bool getFixedTendonLimits(const TensorDesc* dstTensor) const = 0;
    virtual bool getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const = 0;
    virtual bool getFixedTendonOffsets(const TensorDesc* dstTensor) const = 0;
    virtual bool getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const = 0;
    virtual bool getSpatialTendonDampings(const TensorDesc* dstTensor) const = 0;
    virtual bool getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const = 0;
    virtual bool getSpatialTendonOffsets(const TensorDesc* dstTensor) const = 0;
    virtual bool setFixedTendonProperties(const TensorDesc* stiffnesses,
                                          const TensorDesc* dampings,
                                          const TensorDesc* limitStiffnesses,
                                          const TensorDesc* limits,
                                          const TensorDesc* restLengths,
                                          const TensorDesc* offsets,
                                          const TensorDesc* indexTensor) const = 0;
    virtual bool setSpatialTendonProperties(const TensorDesc* stiffnesses,
                                            const TensorDesc* dampings,
                                            const TensorDesc* limitStiffnesses,
                                            const TensorDesc* offsets,
                                            const TensorDesc* indexTensor) const = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~IArticulationView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
