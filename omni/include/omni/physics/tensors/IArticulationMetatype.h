// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "JointTypes.h"

namespace omni
{
namespace physics
{
namespace tensors
{

// used for duck typing articulations
class IArticulationMetatype
{
public:
    virtual uint32_t getLinkCount() const = 0;
    virtual uint32_t getJointCount() const = 0;
    virtual uint32_t getDofCount() const = 0;

    virtual const char* getLinkName(uint32_t linkIdx) const = 0;
    virtual const char* getLinkParentName(uint32_t linkIdx) const = 0;
    virtual const char* getJointName(uint32_t jointIdx) const = 0;
    virtual const char* getDofName(uint32_t dofIdx) const = 0;

    virtual int32_t findLinkIndex(const char* linkName) const = 0;
    virtual int32_t findLinkParentIndex(const char* linkName) const = 0;
    virtual int32_t findJointIndex(const char* jointName) const = 0;
    virtual int32_t findDofIndex(const char* dofName) const = 0;

    virtual JointType getJointType(uint32_t jointIdx) const = 0;
    virtual uint32_t getJointDofOffset(uint32_t jointIdx) const = 0;
    virtual uint32_t getJointDofCount(uint32_t jointIdx) const = 0;

    virtual DofType getDofType(uint32_t dofIdx) const = 0;

    virtual bool getFixedBase() const = 0;

protected:
    virtual ~IArticulationMetatype() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
