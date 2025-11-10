// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "ArticulationMetatype.h"

#include <carb/logging/Log.h>

namespace omni
{
namespace physx
{
namespace tensors
{

uint32_t ArticulationMetatype::getLinkCount() const
{
    return uint32_t(mLinkNames.size());
}

uint32_t ArticulationMetatype::getJointCount() const
{
    return uint32_t(mJointNames.size());
}

uint32_t ArticulationMetatype::getDofCount() const
{
    return uint32_t(mDofNames.size());
}

const char* ArticulationMetatype::getLinkName(uint32_t linkIdx) const
{
    if (linkIdx < mLinkNames.size())
    {
        return mLinkNames[linkIdx].c_str();
    }
    return nullptr;
}

const char* ArticulationMetatype::getJointName(uint32_t jointIdx) const
{
    if (jointIdx < mJointNames.size())
    {
        return mJointNames[jointIdx].c_str();
    }
    return nullptr;
}

const char* ArticulationMetatype::getLinkParentName(uint32_t linkIdx) const
{
    if (linkIdx < mLinkParentIndices.size())
    {
        uint32_t parentIdx = mLinkParentIndices[linkIdx];
        if (parentIdx < mLinkNames.size() && parentIdx != -1)
        {
            return mLinkNames[parentIdx].c_str();
        }
    }
    return nullptr;
}

JointType ArticulationMetatype::getJointType(uint32_t jointIdx) const
{
    if (jointIdx < mJointTypes.size())
    {
        return mJointTypes[jointIdx];
    }
    return JointType::eInvalid;
}

uint32_t ArticulationMetatype::getJointDofOffset(uint32_t jointIdx) const
{
    if (jointIdx < mJointDofOffsets.size())
    {
        return mJointDofOffsets[jointIdx];
    }
    return 0; // hmmm
}

uint32_t ArticulationMetatype::getJointDofCount(uint32_t jointIdx) const
{
    if (jointIdx < mJointDofCounts.size())
    {
        return mJointDofCounts[jointIdx];
    }
    return 0;
}

const char* ArticulationMetatype::getDofName(uint32_t dofIdx) const
{
    if (dofIdx < mDofNames.size())
    {
        return mDofNames[dofIdx].c_str();
    }
    return nullptr;
}

DofType ArticulationMetatype::getDofType(uint32_t dofIdx) const
{
    if (dofIdx < mDofTypes.size())
    {
        return mDofTypes[dofIdx];
    }
    return DofType::eInvalid;
}

int32_t ArticulationMetatype::findLinkIndex(const char* linkName) const
{
    if (linkName)
    {
        auto it = mLinkMap.find(linkName);
        if (it != mLinkMap.end())
        {
            return int32_t(it->second);
        }
    }
    return -1;
}

int32_t ArticulationMetatype::findLinkParentIndex(const char* linkName) const
{
    if (linkName)
    {
        int32_t linkIdx = findLinkIndex(linkName);
        if (linkIdx != -1 && linkIdx < mLinkParentIndices.size())
        {
            return mLinkParentIndices[linkIdx];
        }
    }
    return -1;
}

void ArticulationMetatype::setLinkParentIndex(uint32_t linkIdx, uint32_t parentIdx)
{
        if (linkIdx < mLinkParentIndices.size())
            mLinkParentIndices[linkIdx] = parentIdx;
}

int32_t ArticulationMetatype::findJointIndex(const char* jointName) const
{
    if (jointName)
    {
        auto it = mJointMap.find(jointName);
        if (it != mJointMap.end())
        {
            return int32_t(it->second);
        }
    }
    return -1;
}

int32_t ArticulationMetatype::findDofIndex(const char* dofName) const
{
    if (dofName)
    {
        auto it = mDofMap.find(dofName);
        if (it != mDofMap.end())
        {
            return int32_t(it->second);
        }
    }
    return -1;
}

bool ArticulationMetatype::getFixedBase() const
{
    return mFixedBase;
}

void ArticulationMetatype::setFixedBase(bool fixedBase)
{
    mFixedBase = fixedBase;
}

void ArticulationMetatype::addLink(const std::string& name)
{
    uint32_t linkIdx = uint32_t(mLinkNames.size());

    mLinkNames.push_back(name);
    // temporary placeholder for parent index, which should be updated once a joint connects this link to another
    mLinkParentIndices.push_back(-1); 

    if (mLinkMap.find(name) == mLinkMap.end())
    {
        mLinkMap[name] = linkIdx;
    }
    else
    {
        CARB_LOG_WARN("Duplicate link name '%s' in articulation metatype", name.c_str());
    }
}

void ArticulationMetatype::addJoint(const JointDesc& desc)
{
    uint32_t jointIdx = uint32_t(mJointNames.size());

    mJointNames.push_back(desc.name);
    mJointTypes.push_back(desc.type);
    if (mJointMap.find(desc.name) == mJointMap.end())
    {
        mJointMap[desc.name] = jointIdx;
    }
    else
    {
        CARB_LOG_WARN("Duplicate joint name '%s' in articulation metatype", desc.name.c_str());
    }

    uint32_t numDofs = uint32_t(desc.dofs.size());
    mJointDofOffsets.push_back(uint32_t(mDofNames.size()));
    mJointDofCounts.push_back(numDofs);

    for (uint32_t i = 0; i < numDofs; i++)
    {
        addDof(desc.dofs[i], desc.body0IsParent);
    }
}

void ArticulationMetatype::addDof(const DofDesc& desc, bool body0IsParent)
{
    uint32_t dofIdx = uint32_t(mDofNames.size());

    mDofNames.push_back(desc.name);
    mDofTypes.push_back(desc.type);
    mDofIsBody0Parent.push_back(body0IsParent);

    if (mDofMap.find(desc.name) == mDofMap.end())
    {
        mDofMap[desc.name] = dofIdx;
    }
    else
    {
        CARB_LOG_WARN("Duplicate DOF name '%s' in articulation metatype", desc.name.c_str());
    }
}

bool ArticulationMetatype::isDofBody0Parent(uint32_t DofIdx) const
{
    if (DofIdx >= getDofCount())
    {
        CARB_LOG_ERROR(
            "Incorrect DofIdx encountered while evaluating articulation metatype. Joint state data will be incorrect.");
        return true;
    }
    return mDofIsBody0Parent[DofIdx];
}

void ArticulationMetatype::print() const
{
    printf("Link names:");
    for (auto& ln : mLinkNames)
    {
        printf(" %s", ln.c_str());
    }
    printf("\n");

    printf("Joint names:");
    for (auto& jn : mJointNames)
    {
        printf(" %s", jn.c_str());
    }
    printf("\n");

    printf("Joint types:");
    for (auto& jt : mJointTypes)
    {
        printf(" %d", int(jt));
    }
    printf("\n");
}

}
}
}
