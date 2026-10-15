// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4
#pragma once

#include "AttachmentObjectLookup.h"
#include <omni/physx/IPhysxJoint.h>
#include <common/foundation/CarbPhysXCast.h>

namespace omni::physx::attachmentJoint
{
// Private owner-qualified reads; callers hold public runtime admission.
inline carb::Float4 frameCorrection(const usdparser::AttachedStage* owner, usdparser::ObjectId id)
{
    if (!owner)
        return carb::Float4{0, 0, 0, 1.0f};
    PhysXType internalType = PhysXType::ePTRemoved;
    internal::InternalPhysXDatabase& internalPhysXDatabase = owner->getInternalPhysXDatabase();
    void* objectRecord = internalPhysXDatabase.getRecord(internalType, id);

    if (objectRecord != nullptr && (internalType == PhysXType::ePTJoint || internalType == PhysXType::ePTLinkJoint))
    {
        internal::InternalDatabase::Record& jointRec = internalPhysXDatabase.getRecords()[id];
        internal::InternalJoint* intJoint = static_cast<internal::InternalJoint*>(jointRec.mInternalPtr);

        if (!intJoint)
            return carb::Float4{0, 0, 0, 1.0f};
        ::physx::PxQuat jointCorrection = intJoint->getLocalPoseFixupQuat();

        return toFloat4(jointCorrection);
    }

    return carb::Float4{ 0, 0, 0, 1.f };
}

inline void stateData(const usdparser::AttachedStage* owner, usdparser::ObjectId id, JointStateData* jointStateData)
{
    if (!jointStateData)
        return;
    memset(jointStateData, 0, sizeof(JointStateData));
    if (!owner)
        return;
    const internal::InternalPhysXDatabase& db = owner->getInternalPhysXDatabase();
    if (id < db.getRecords().size())
    {
        const internal::InternalDatabase::Record& record = db.getRecords()[id];
        if (record.mInternalPtr && record.mType == ePTLinkJoint)
        {
            const internal::InternalJoint* intJoint =
                reinterpret_cast<const internal::InternalJoint*>(record.mInternalPtr);
            jointStateData->body0IsParentLink = intJoint->mBody0IsParentLink;
            for (int i = 0; i < 6; ++i)
            {
                jointStateData->enabled[i] = intJoint->mJointStates[i].enabled;
                jointStateData->physxAxis[i] = intJoint->mJointStates[i].physxAxis;
                jointStateData->convertToDegrees[i] = intJoint->mJointStates[i].convertToDegrees;
                jointStateData->initialPosition[i] = intJoint->mJointStates[i].initialState.position;
                jointStateData->initialVelocity[i] = intJoint->mJointStates[i].initialState.velocity;
            }
        }
    }
}

}
