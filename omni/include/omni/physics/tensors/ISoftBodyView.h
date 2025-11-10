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

class ISoftBodyView
{
public:
    virtual uint32_t getCount() const = 0;
    virtual uint32_t getMaxElementsPerBody() const = 0;
    virtual uint32_t getMaxSimElementsPerBody() const = 0;
    virtual uint32_t getMaxVerticesPerBody() const = 0;
    virtual uint32_t getMaxSimVerticesPerBody() const = 0;
    virtual bool getTransforms(const TensorDesc* dstTensor) const = 0;

    // collision mesh nodal and element-wise values
    virtual bool getNodalPositions(const TensorDesc* dstTensor) const = 0;
    virtual bool getElementStresses(const TensorDesc* dstTensor) const = 0;
    virtual bool getElementRestPoses(const TensorDesc* dstTensor) const = 0;
    virtual bool getElementRotations(const TensorDesc* dstTensor) const = 0;
    virtual bool getElementDeformationGradients(const TensorDesc* dstTensor) const = 0;

    virtual bool getSimElementStresses(const TensorDesc* dstTensor) const = 0;
    virtual bool getSimElementRestPoses(const TensorDesc* dstTensor) const = 0;
    virtual bool getSimElementRotations(const TensorDesc* dstTensor) const = 0;
    virtual bool getSimElementDeformationGradients(const TensorDesc* dstTensor) const = 0;

    // element-wise indices
    virtual bool getElementIndices(const TensorDesc* dstTensor) const = 0;
    virtual bool getSimElementIndices(const TensorDesc* dstTensor) const = 0;

    // simulation mesh nodal position
    virtual bool getSimNodalPositions(const TensorDesc* dstTensor) const = 0;
    virtual bool setSimNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    // simulation mesh nodal velocities
    virtual bool getSimNodalVelocities(const TensorDesc* dstTensor) const = 0;
    virtual bool setSimNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    // targets for kinematic nodes
    virtual bool getSimKinematicTargets(const TensorDesc* dstTensor) const = 0;
    virtual bool setSimKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~ISoftBodyView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
