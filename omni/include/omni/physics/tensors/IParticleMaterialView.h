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

class IParticleMaterialView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual bool getFriction(const TensorDesc* dstTensor) const = 0;
    virtual bool setFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getDamping(const TensorDesc* dstTensor) const = 0;
    virtual bool setDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getGravityScale(const TensorDesc* dstTensor) const = 0;
    virtual bool setGravityScale(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getLift(const TensorDesc* dstTensor) const = 0;
    virtual bool setLift(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getDrag(const TensorDesc* dstTensor) const = 0;
    virtual bool setDrag(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~IParticleMaterialView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
