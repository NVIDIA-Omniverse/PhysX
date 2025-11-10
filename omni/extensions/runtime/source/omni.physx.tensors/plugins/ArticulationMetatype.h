// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/physics/tensors/IArticulationMetatype.h>

#include <map>
#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::DofType;
using omni::physics::tensors::JointType;

class ArticulationMetatype : public omni::physics::tensors::IArticulationMetatype
{
public:
    //
    // public interface
    //

    uint32_t getLinkCount() const override;
    uint32_t getJointCount() const override;
    uint32_t getDofCount() const override;

    const char* getLinkName(uint32_t linkIdx) const override;
    const char* getLinkParentName(uint32_t linkIdx) const override;
    const char* getJointName(uint32_t jointIdx) const override;
    const char* getDofName(uint32_t dofIdx) const override;

    int32_t findLinkIndex(const char* linkName) const override;
    int32_t findLinkParentIndex(const char* linkName) const override;
    int32_t findJointIndex(const char* jointName) const override;
    int32_t findDofIndex(const char* dofName) const override;

    JointType getJointType(uint32_t jointIdx) const override;
    uint32_t getJointDofOffset(uint32_t jointIdx) const override;
    uint32_t getJointDofCount(uint32_t jointIdx) const override;

    DofType getDofType(uint32_t dofIdx) const override;

    bool getFixedBase() const override;

    //
    // utilities
    //

    struct DofDesc
    {
        DofType type = DofType::eInvalid;
        std::string name;

        DofDesc() = default;

        DofDesc(const std::string& name, DofType type) : type(type), name(name)
        {
        }
    };

    struct JointDesc
    {
        JointType type = JointType::eInvalid;
        bool body0IsParent = true;
        std::string name;
        std::vector<DofDesc> dofs;
    };

    void addLink(const std::string& name);
    void addJoint(const JointDesc& desc);
    bool isDofBody0Parent(uint32_t DofIdx) const;
    void setFixedBase(bool fixedBase);
    void setLinkParentIndex(uint32_t linkIdx, uint32_t parentIdx);
    void print() const;

private:
    void addDof(const DofDesc& desc, bool body0IsParent);

    std::vector<std::string> mLinkNames;
    std::vector<std::string> mJointNames;
    std::vector<JointType> mJointTypes;
    std::vector<uint32_t> mLinkParentIndices;
    std::vector<uint32_t> mJointDofOffsets;
    std::vector<uint32_t> mJointDofCounts;
    std::vector<bool> mDofIsBody0Parent;

    std::vector<std::string> mDofNames;
    std::vector<DofType> mDofTypes;

    std::map<std::string, uint32_t> mLinkMap;
    std::map<std::string, uint32_t> mJointMap;
    std::map<std::string, uint32_t> mDofMap;

    bool mFixedBase = false;

    friend struct ArticulationMetatypeLT;
};

struct ArticulationMetatypeLT
{
    bool operator()(const ArticulationMetatype& a, const ArticulationMetatype& b) const
    {
        if (a.mFixedBase < b.mFixedBase)
        {
            return true;
        }
        else if (a.mFixedBase == b.mFixedBase)
        {
            if (a.mJointTypes < b.mJointTypes)
            {
                return true;
            }
            else if (a.mJointTypes == b.mJointTypes)
            {
                if (a.mJointNames < b.mJointNames)
                {
                    return true;
                }
                else if (a.mJointNames == b.mJointNames)
                {
                    if (a.mDofIsBody0Parent < b.mDofIsBody0Parent)
                    {
                        return true;
                    }
                    else if (a.mDofIsBody0Parent == b.mDofIsBody0Parent)
                    {
                        if (a.mLinkParentIndices < b.mLinkParentIndices)
                        {
                            return true;
                        }
                        else if (a.mLinkParentIndices == b.mLinkParentIndices)
                        {
                            return a.mLinkNames < b.mLinkNames;
                        }
                    }
                }
            }
        }
        return false;
    }
};

} // namespace tensors
} // namespace physx
} // namespace omni
