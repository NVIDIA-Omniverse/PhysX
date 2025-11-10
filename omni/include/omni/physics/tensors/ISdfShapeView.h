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

class ISdfShapeView
{
public:
    virtual uint32_t getCount() const = 0;

    virtual uint32_t getMaxNumPoints() const = 0;

    virtual bool getSdfAndGradients(const TensorDesc* dstTensor, const TensorDesc* srcPointTensor) const = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

    virtual const char* getUsdPrimPath(uint32_t sensorIdx) const = 0;

protected:
    virtual ~ISdfShapeView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
