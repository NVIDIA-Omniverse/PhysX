// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "TensorDesc.h"

#include <cstdint>

namespace omni
{
namespace physics
{
namespace tensors
{

class IDeformableMaterialView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual bool getDynamicFriction(const TensorDesc* dstTensor) const = 0;
    virtual bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    // Masked variant: srcTensor is full [N,...], maskTensor is uint8[N] where nonzero = selected.
    virtual bool setDynamicFrictionMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getYoungsModulus(const TensorDesc* dstTensor) const = 0;
    virtual bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setYoungsModulusMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getPoissonsRatio(const TensorDesc* dstTensor) const = 0;
    virtual bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setPoissonsRatioMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    // Volume + surface deformable materials
    virtual bool getElasticityDamping(const TensorDesc* dstTensor) const = 0;
    virtual bool setElasticityDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setElasticityDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    // Surface deformable materials only; get reads 0.0 and set is skipped for volume material entries
    virtual bool getBendingStiffness(const TensorDesc* dstTensor) const = 0;
    virtual bool setBendingStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setBendingStiffnessMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getThickness(const TensorDesc* dstTensor) const = 0;
    virtual bool setThickness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setThicknessMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getBendingDamping(const TensorDesc* dstTensor) const = 0;
    virtual bool setBendingDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setBendingDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool check() const = 0;
    virtual void release() = 0;

protected:
    virtual ~IDeformableMaterialView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
