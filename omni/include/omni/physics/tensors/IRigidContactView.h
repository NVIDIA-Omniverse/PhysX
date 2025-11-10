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

class IRigidContactView
{
public:
    virtual uint32_t getSensorCount() const = 0;
    virtual uint32_t getFilterCount() const = 0;
    virtual uint32_t getMaxContactDataCount() const = 0;

    // net contact force per sensor, shape (getSensorCount(), 3)
    virtual bool getNetContactForces(const TensorDesc* dstTensor, float dt) const = 0;

    // contact force per filter per sensor, shape (getSensorCount(), getFilterCount(), 3)
    virtual bool getContactForceMatrix(const TensorDesc* dstTensor, float dt) const = 0;

    // contact data per filter per sensor per patch
    virtual bool getContactData(const TensorDesc* contactForceTensor, //(getSensorCount(), getFilterCount(),
                                                                      // getMaxContactDataCount(), 1)
                                const TensorDesc* contactPointTensor, //(getSensorCount(), getFilterCount(),
                                                                      // getMaxContactDataCount(), 3)
                                const TensorDesc* contactNormalTensor, //(getSensorCount(), getFilterCount(),
                                                                       // getMaxContactDataCount(), 3)
                                const TensorDesc* contactSeparationTensor, //(getSensorCount(), getFilterCount(),
                                                                           // getMaxContactDataCount(), 1)
                                const TensorDesc* contactCountTensor, //(getSensorCount(), getFilterCount())
                                const TensorDesc* contactStartIndicesTensor, //(getSensorCount(), getFilterCount())
                                float dt) const = 0;

    // friction data per filter per sensor per patch
    virtual bool getFrictionData(const TensorDesc* FrictionForceTensor, //(getSensorCount(), getFilterCount(),
                                                                        // getMaxContactDataCount(), 4, 3)
                                 const TensorDesc* contactPointTensor, //(getSensorCount(), getFilterCount(),
                                                                       // getMaxContactDataCount(), 2, 3)
                                 const TensorDesc* contactCountTensor, //(getSensorCount(), getFilterCount())
                                 const TensorDesc* contactStartIndicesTensor, //(getSensorCount(), getFilterCount())
                                 float dt) const = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

    virtual const char* getUsdPrimPath(uint32_t sensorIdx) const = 0;
    virtual const char* getUsdPrimName(uint32_t sensorIdx) const = 0;
    virtual const char* getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const = 0;
    virtual const char* getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const = 0;

protected:
    virtual ~IRigidContactView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
