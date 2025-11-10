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

class IParticleSystemView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual bool getSolidRestOffset(const TensorDesc* dstTensor) const = 0;
    virtual bool setSolidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getFluidRestOffset(const TensorDesc* dstTensor) const = 0;
    virtual bool setFluidRestOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getParticleContactOffset(const TensorDesc* dstTensor) const = 0;
    virtual bool setParticleContactOffset(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getWind(const TensorDesc* dstTensor) const = 0;
    virtual bool setWind(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

protected:
    virtual ~IParticleSystemView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
