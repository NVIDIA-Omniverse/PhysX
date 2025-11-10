// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <PxPhysicsAPI.h>

#include <private/omni/physx/PhysxUsd.h>

#include <common/foundation/Allocator.h>


namespace omni
{
namespace physx
{
namespace internal
{

class InternalScene;


class InternalMimicJoint : public Allocateable
{
private:
    InternalMimicJoint();
    ~InternalMimicJoint();

public:
    InternalMimicJoint(InternalScene&,
                       const ::physx::PxArticulationJointReducedCoordinate& targetJoint,
                       const ::physx::PxArticulationJointReducedCoordinate& referenceJoint,
                       int targetJointAxis,
                       int referenceJointAxis,
                       float gearing,
                       float offset,
                       float naturalFrequency,
                       float dampingRatio,
                       usdparser::ObjectType);

    // for PhysXReplicator only
    InternalMimicJoint(InternalScene&, ::physx::PxArticulationMimicJoint&, usdparser::ObjectType);

    void release(bool removeFromTrackers, bool releasePhysXObject);

    void releasePhysXMimicJoint();

    usdparser::ObjectType getObjectType() const
    {
        return mObjectType;
    }

    ::physx::PxArticulationMimicJoint* getPhysXMimicJoint() const
    {
        return mPhysXMimicJoint;
    }

    ::physx::PxArticulationReducedCoordinate* getArticulation() const;

    __forceinline const ::physx::PxArticulationJointReducedCoordinate* getTargetJoint()
    {
        if (mPhysXMimicJoint)
            return &mPhysXMimicJoint->getJointA();
        else
            return nullptr;
    }

    __forceinline const ::physx::PxArticulationJointReducedCoordinate* getReferenceJoint()
    {
        if (mPhysXMimicJoint)
            return &mPhysXMimicJoint->getJointB();
        else
            return nullptr;
    }

    void setGearing(float gearing);
    void setOffset(float offset);
    void setNaturalFrequency(float naturalFrequency);
    void setDampingRatio(float dampingRatio);

private:
    InternalScene& mInternalScene;
    ::physx::PxArticulationMimicJoint* mPhysXMimicJoint;
    usdparser::ObjectType mObjectType; // for PhysXReplicator only
};


} // namespace internal
} // namespace physx
} // namespace omni
