// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-1
 */

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

    // Returns nullptr for an out-of-range index.
    virtual const char* getUsdPrimPath(uint32_t sensorIdx) const = 0;

protected:
    virtual ~ISdfShapeView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
