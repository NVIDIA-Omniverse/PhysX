// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include <omni/fabric/usd/PathConversion.h>
#include <omni/physx/IPhysxJoint.h>
#include <PxPhysicsAPI.h>
#include <common/foundation/TypeCast.h>
#include "OmniPhysX.h"

namespace omni
{
namespace physx
{

static carb::Float4 physxJointGetJointFramePhysxToUsdQuat(usdparser::ObjectId id)
{
    PhysXType internalType = PhysXType::ePTRemoved;
    internal::InternalPhysXDatabase& internalPhysXDatabase = OmniPhysX::getInstance().getInternalPhysXDatabase();
    void* objectRecord = internalPhysXDatabase.getRecord(internalType, id);

    if (objectRecord != nullptr && (internalType == PhysXType::ePTJoint || internalType == PhysXType::ePTLinkJoint))
    {
        internal::InternalDatabase::Record& jointRec = internalPhysXDatabase.getRecords()[id];
        internal::InternalJoint* intJoint = static_cast<internal::InternalJoint*>(jointRec.mInternalPtr);

        ::physx::PxQuat jointCorrection = intJoint->getLocalPoseFixupQuat();

        return toFloat4(jointCorrection);
    }

    return carb::Float4{ 0, 0, 0, 1.f };
}

static void physxJointGetJointStateData(usdparser::ObjectId id, JointStateData* jointStateData)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    memset(jointStateData, 0, sizeof(JointStateData));
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
                jointStateData->fabricTokenC[i] = omni::fabric::asInt(intJoint->mJointStates[i].usdToken).token;
            }
        }
    }
}

} // namespace physx
} // namespace omni

void fillInterface(omni::physx::IPhysxJoint& iface)
{
    iface.getJointStateData = omni::physx::physxJointGetJointStateData;
    iface.getJointFramePhysxToUsdQuat = omni::physx::physxJointGetJointFramePhysxToUsdQuat;
}
