// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

namespace quickhull
{

class WuQuantizer
{
public:
    static WuQuantizer* create(void);

    // use the Wu quantizer with 10 bits of resolution on each axis.  Precision down to 0.0009765625
    // All input data is normalized to a unit cube.
    virtual const double* wuQuantize3D(uint32_t vcount,
                                       const double* vertices,
                                       bool denormalizeResults,
                                       uint32_t maxVertices,
                                       uint32_t& outputCount) = 0;

    virtual const double* getDenormalizeScale(void) const = 0;

    virtual const double* getDenormalizeCenter(void) const = 0;

    virtual void release(void) = 0;


protected:
    virtual ~WuQuantizer(void)
    {
    }
};

}; // namespace quickhull
