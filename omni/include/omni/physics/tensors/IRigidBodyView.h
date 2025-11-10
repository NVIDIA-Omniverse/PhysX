// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "TensorDesc.h"

#include <cstdint>

namespace omni
{
namespace physics
{
namespace tensors
{

class IRigidBodyView
{
public:
    virtual uint32_t getCount() const = 0;
    virtual uint32_t getMaxShapes() const = 0;

    virtual const char* getUsdPrimPath(uint32_t rbIdx) const = 0;

    virtual bool getTransforms(const TensorDesc* dstTensor) const = 0;
    virtual bool getVelocities(const TensorDesc* dstTensor) const = 0;
    virtual bool getAccelerations(const TensorDesc* dstTensor) const = 0;

    virtual bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
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
    virtual bool getDisableSimulations(const TensorDesc* dstTensor) const = 0;

    virtual bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    // materials/shapes
    virtual bool getMaterialProperties(const TensorDesc* dstTensor) const = 0;
    virtual bool getRestOffsets(const TensorDesc* dstTensor) const = 0;
    virtual bool getContactOffsets(const TensorDesc* dstTensor) const = 0;

    virtual bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const = 0;
    virtual bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const = 0;
    virtual bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~IRigidBodyView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
