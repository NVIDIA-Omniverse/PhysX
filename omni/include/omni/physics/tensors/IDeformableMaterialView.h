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

class IDeformableMaterialView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual bool getDynamicFriction(const TensorDesc* dstTensor) const = 0;
    virtual bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getYoungsModulus(const TensorDesc* dstTensor) const = 0;
    virtual bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool getPoissonsRatio(const TensorDesc* dstTensor) const = 0;
    virtual bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    virtual bool check() const = 0;
    virtual void release() = 0;

protected:
    virtual ~IDeformableMaterialView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
