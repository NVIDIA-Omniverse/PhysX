// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    PXR_NS::TfToken jointPrimType;
    ICustomJointCallback customJointCb;
    ::physx::PxConstraintSolverPrep solverPrepFn;
    size_t jointDataSize;
    ::physx::PxU32 typeId;
};

class CustomPhysXJoint : public ::physx::PxConstraintConnector, public Allocateable
{
public:
    CustomPhysXJoint(const PXR_NS::SdfPath& path,
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

    PXR_NS::SdfPath mJointPath;
    CustomJointInfo mCustomJointInfo;
};

using CustomJointRegistryMap = std::unordered_map<size_t, CustomJointInfo>;
using CustomJointTypeMap = std::unordered_map<PXR_NS::TfToken, CustomJointInfo, PXR_NS::TfToken::HashFunctor>;
using CustomJointMap = std::unordered_map<PXR_NS::SdfPath, CustomPhysXJoint*, PXR_NS::SdfPath::Hash>;

class PhysXCustomJointManager
{
public:
    PhysXCustomJointManager();
    ~PhysXCustomJointManager();

    size_t registerCustomJoint(const PXR_NS::TfToken& jointPrimType,
                               ICustomJointCallback& jointCallback,
                               ::physx::PxConstraintSolverPrep jointPrepFn,
                               size_t jointDataSize);
    void unregisterCustomJoint(size_t id);
    void markJointDirty(const PXR_NS::SdfPath& primKey);
    void setJointFlags(const PXR_NS::SdfPath& primKey, CustomJointFlag::Enum flags);
    CustomJointFlag::Enum getJointFlags(const PXR_NS::SdfPath& primKey);

    void clear()
    {
        for (CustomJointMap::reference ref : mCustomJointMap)
        {
            delete ref.second;
        }
        mCustomJointMap.clear();
    }

    CustomPhysXJoint* createCustomJoint(const PXR_NS::SdfPath& primKey,
                                        const usdparser::CustomPhysxJointDesc& jointDesc,
                                        ::physx::PxRigidActor* actor0,
                                        const ::physx::PxTransform& localFrame0,
                                        ::physx::PxRigidActor* actor1,
                                        const ::physx::PxTransform& localFrame1);

    void removeCustomJoint(const PXR_NS::SdfPath& primKey);

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


inline void checkRevoluteJointLimits(::physx::PxJointAngularLimitPair& limitPair, const char* primKey)
{
    const float ANGLE_CLAMP_THRESHOLD = ::physx::PxPi * 1.0001f;
    if (limitPair.lower < -ANGLE_CLAMP_THRESHOLD)
    {
        limitPair.lower = -::physx::PxPi;
        CARB_LOG_WARN("Lower angle limit on a revolute D6 joint was clamped to -180 degrees. %s", primKey);
    }

    if (limitPair.upper > ANGLE_CLAMP_THRESHOLD)
    {
        limitPair.upper = ::physx::PxPi;
        CARB_LOG_WARN("Upper angle limit on a revolute D6 joint was clamped to 180 degrees. %s", primKey);
    }
}

} // namespace physx
} // namespace omni
