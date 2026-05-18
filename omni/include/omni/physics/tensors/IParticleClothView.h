// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

class IParticleClothView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual uint32_t getMaxParticlesPerCloth() const = 0;

    virtual uint32_t getMaxSpringsPerCloth() const = 0;

    virtual bool getPositions(const TensorDesc* dstTensor) const = 0;

    virtual bool setPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    // Masked variant: srcTensor is full [N,...], maskTensor is uint8[N] where nonzero = selected.
    virtual bool setPositionsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getVelocities(const TensorDesc* dstTensor) const = 0;

    virtual bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getMasses(const TensorDesc* srcTensor) const = 0;

    virtual bool setMasses(const TensorDesc* dstTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setMassesMasked(const TensorDesc* dstTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getSpringDamping(const TensorDesc* srcTensor) const = 0;

    virtual bool setSpringDamping(const TensorDesc* dstTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setSpringDampingMasked(const TensorDesc* dstTensor, const TensorDesc* maskTensor) = 0;

    virtual bool getSpringStiffness(const TensorDesc* srcTensor) const = 0;

    virtual bool setSpringStiffness(const TensorDesc* dstTensor, const TensorDesc* indexTensor) = 0;
    virtual bool setSpringStiffnessMasked(const TensorDesc* dstTensor, const TensorDesc* maskTensor) = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~IParticleClothView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
