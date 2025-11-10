// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysxCustomJoint.h>
#include <common/foundation/Allocator.h>

#include <private/omni/physx/PhysxUsd.h>
#include <physx/include/extensions/PxJointLimit.h>

namespace omni
{
namespace physx
{
struct CustomJointInfo
{
    pxr::TfToken jointPrimType;
    ICustomJointCallback customJointCb;
    ::physx::PxConstraintSolverPrep solverPrepFn;
    size_t jointDataSize;
    ::physx::PxU32 typeId;
};

class CustomPhysXJoint : public ::physx::PxConstraintConnector, public Allocateable
{
public:
    CustomPhysXJoint(const pxr::SdfPath& path,
                     const usdparser::CustomPhysxJointDesc& jointDesc,
                     ::physx::PxPhysics& physics,
                     const CustomJointInfo& jointInfo,
                     ::physx::PxConstraintFlag::Enum flags,
                     ::physx::PxRigidActor* actor0,
                     ::physx::PxRigidActor* actor1);

    void release();

    // PxConstraintConnector boilerplate
    void* prepareData();
    void onConstraintRelease();
    void onComShift(::physx::PxU32 actor);
    void onOriginShift(const ::physx::PxVec3& shift);
    void* getExternalReference(::physx::PxU32& typeID);

    ::physx::PxBase* getSerializable()
    {
        return NULL;
    }

    virtual ::physx::PxConstraintSolverPrep getPrep() const;
    virtual const void* getConstantBlock() const;
    virtual bool updatePvdProperties(::physx::pvdsdk::PvdDataStream& pvdConnection,
                                     const ::physx::PxConstraint* c,
                                     ::physx::PxPvdUpdateType::Enum updateType) const
    {
        return true;
    }
    virtual void updateOmniPvdProperties() const {};

    ~CustomPhysXJoint()
    {
    }

    ::physx::PxConstraint* getConstraint()
    {
        return mConstraint;
    }

private:
    ::physx::PxConstraint* mConstraint;

    pxr::SdfPath mJointPath;
    CustomJointInfo mCustomJointInfo;
};

using CustomJointRegistryMap = std::unordered_map<size_t, CustomJointInfo>;
using CustomJointTypeMap = std::unordered_map<pxr::TfToken, CustomJointInfo, pxr::TfToken::HashFunctor>;
using CustomJointMap = std::unordered_map<pxr::SdfPath, CustomPhysXJoint*, pxr::SdfPath::Hash>;

class PhysXCustomJointManager
{
public:
    PhysXCustomJointManager();
    ~PhysXCustomJointManager();

    size_t registerCustomJoint(const pxr::TfToken& jointPrimType,
                               ICustomJointCallback& jointCallback,
                               ::physx::PxConstraintSolverPrep jointPrepFn,
                               size_t jointDataSize);
    void unregisterCustomJoint(size_t id);
    void markJointDirty(const pxr::SdfPath& primPath);
    void setJointFlags(const pxr::SdfPath& primPath, CustomJointFlag::Enum flags);
    CustomJointFlag::Enum getJointFlags(const pxr::SdfPath& primPath);

    void clear()
    {
        for (CustomJointMap::reference ref : mCustomJointMap)
        {
            delete ref.second;
        }
        mCustomJointMap.clear();
    }

    CustomPhysXJoint* createCustomJoint(const pxr::SdfPath& primPath,
                                        const usdparser::CustomPhysxJointDesc& jointDesc,
                                        ::physx::PxRigidActor* actor0,
                                        const ::physx::PxTransform& localFrame0,
                                        ::physx::PxRigidActor* actor1,
                                        const ::physx::PxTransform& localFrame1);

    void removeCustomJoint(const pxr::SdfPath& primPath);

    const CustomJointTypeMap& getCustomJointTypeMap() const
    {
        return mCustomJointTypeMap;
    }

private:
    size_t mJointRegistryCounter;
    CustomJointRegistryMap mCustomJointRegistryMap;
    CustomJointTypeMap mCustomJointTypeMap;
    CustomJointMap mCustomJointMap;
};


inline void checkRevoluteJointLimits(::physx::PxJointAngularLimitPair& limitPair, const char* primPath)
{
    const float ANGLE_CLAMP_THRESHOLD = ::physx::PxPi * 1.0001f;
    if (limitPair.lower < -ANGLE_CLAMP_THRESHOLD)
    {
        limitPair.lower = -::physx::PxPi;
        CARB_LOG_WARN("Lower angle limit on a revolute D6 joint was clamped to -180 degrees. %s", primPath);
    }

    if (limitPair.upper > ANGLE_CLAMP_THRESHOLD)
    {
        limitPair.upper = ::physx::PxPi;
        CARB_LOG_WARN("Upper angle limit on a revolute D6 joint was clamped to 180 degrees. %s", primPath);
    }
}

} // namespace physx
} // namespace omni
