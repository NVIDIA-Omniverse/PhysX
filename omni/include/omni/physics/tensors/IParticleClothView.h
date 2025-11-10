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

class IParticleClothView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual uint32_t getMaxParticlesPerCloth() const = 0;

    virtual uint32_t getMaxSpringsPerCloth() const = 0;

    virtual bool getPositions(const TensorDesc* dstTensor) const = 0;

    virtual bool setPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getVelocities(const TensorDesc* dstTensor) const = 0;

    virtual bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getMasses(const TensorDesc* srcTensor) const = 0;

    virtual bool setMasses(const TensorDesc* dstTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getSpringDamping(const TensorDesc* srcTensor) const = 0;

    virtual bool setSpringDamping(const TensorDesc* dstTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getSpringStiffness(const TensorDesc* srcTensor) const = 0;

    virtual bool setSpringStiffness(const TensorDesc* dstTensor, const TensorDesc* indexTensor) = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~IParticleClothView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
