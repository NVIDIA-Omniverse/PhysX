// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
                       usdparser::ObjectType,
                       // Per-joint sign factors that convert authored gearing/offset (which are defined
                       // against each joint's natural USD body0->body1 position convention) to the PhysX
                       // articulation joint-coordinate convention. They are 1.0 unless a referenced joint
                       // was authored with reversed body order (mBody0IsParentLink == false), in which
                       // case omni.physx negates that joint's PhysX coordinate. Applies to both the
                       // PhysxMimicJointAPI and NewtonMimicAPI paths.
                       float gearingSign = 1.0f,
                       float offsetSign = 1.0f);

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
    // see the constructor doc; applied to gearing/offset (incl. runtime updates) so the authored
    // relationship holds regardless of the referenced joints' body order
    float mGearingSign = 1.0f;
    float mOffsetSign = 1.0f;
};


} // namespace internal
} // namespace physx
} // namespace omni
