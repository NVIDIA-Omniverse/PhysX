// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */
/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-9 AC-11 AC-12 AC-13
 */

#include <carb/logging/Log.h>

#include "PhysXCustomJoint.h"
#include "OmniPhysX.h"
#include "usdLoad/AttachedStage.h"

#include <omni/physics/parse/CustomTokens.h>

using namespace ::physx;

namespace omni
{
namespace physx
{

CustomPhysXJoint::CustomPhysXJoint(omni::physics::parse::ObjectKey key, const usdparser::CustomPhysxJointDesc& jointDesc,::physx::PxPhysics& physics, const CustomJointInfo& jointInfo, PxConstraintFlag::Enum flags,
    ::physx::PxRigidActor* actor0, ::physx::PxRigidActor* actor1)
{
    uint16_t constraintFlags = flags;
    if (jointDesc.enableCollision)
        constraintFlags |= PxConstraintFlag::eCOLLISION_ENABLED;
    else
        constraintFlags &= ~(uint16_t)PxConstraintFlag::eCOLLISION_ENABLED;

    PxConstraintShaderTable shaderTable = { jointInfo.solverPrepFn, nullptr, (PxConstraintFlag::Enum)constraintFlags };
    mConstraint = physics.createConstraint(actor0, actor1, *this, shaderTable, ::physx::PxU32(jointInfo.jointDataSize));

    mConstraint->setBreakForce(isfinite(jointDesc.breakForce) ? jointDesc.breakForce : FLT_MAX,
        isfinite(jointDesc.breakTorque) ? jointDesc.breakTorque : FLT_MAX);

    mCustomJointInfo = jointInfo;
    mJointKey = key;
}

void CustomPhysXJoint::release()
{
    mConstraint->release();
}

void* CustomPhysXJoint::prepareData()
{
    return mCustomJointInfo.customJointCb.prepareJointDataFn(mJointKey, mCustomJointInfo.customJointCb.userData);
}

void  CustomPhysXJoint::onConstraintRelease()
{
    mCustomJointInfo.customJointCb.releaseJointFn(mJointKey, mCustomJointInfo.customJointCb.userData);
    OmniPhysX::getInstance().getCustomJointManager().removeCustomJoint(mJointKey);
    delete this;
}

void  CustomPhysXJoint::onComShift(::physx::PxU32 actor)
{
    mCustomJointInfo.customJointCb.onComShiftFn(mJointKey, actor, mCustomJointInfo.customJointCb.userData);
}

void  CustomPhysXJoint::onOriginShift(const ::physx::PxVec3& shift)
{
    mCustomJointInfo.customJointCb.onOriginShift(mJointKey, shift, mCustomJointInfo.customJointCb.userData);
}

void* CustomPhysXJoint::getExternalReference(::physx::PxU32& typeID)
{
    typeID = mCustomJointInfo.typeId;
    return this;
}

::physx::PxConstraintSolverPrep CustomPhysXJoint::getPrep() const
{
    return mCustomJointInfo.solverPrepFn;
}

const void* CustomPhysXJoint::getConstantBlock() const
{
    return mCustomJointInfo.customJointCb.getConstantBlockFn(mJointKey, mCustomJointInfo.customJointCb.userData);
}

PxConstraintFlag::Enum convertJointFlags(CustomJointFlag::Enum inFlags)
{
    PxConstraintFlags flags = PxConstraintFlag::Enum(0);
    if (inFlags & CustomJointFlag::eALWAYS_UPDATE)
    {
        flags |= PxConstraintFlag::eALWAYS_UPDATE;
    }
    return PxConstraintFlag::Enum(uint16_t(flags));
}

void modifyJointFlags(CustomJointFlag::Enum inFlags, PxConstraintFlags outFlags)
{
    PxConstraintFlags flags = PxConstraintFlag::Enum(0);
    if (inFlags & CustomJointFlag::eALWAYS_UPDATE)
    {
        outFlags |= PxConstraintFlag::eALWAYS_UPDATE;
    }
    else
    {
        outFlags &= ~PxConstraintFlag::eALWAYS_UPDATE;
    }    
}

CustomJointFlag::Enum convertJointFlags(PxConstraintFlags inFlags)
{
    uint16_t flags = 0;
    if (inFlags & PxConstraintFlag::eALWAYS_UPDATE)
    {
        flags |= CustomJointFlag::eALWAYS_UPDATE;
    }
    return (CustomJointFlag::Enum)flags;
}

PhysXCustomJointManager::PhysXCustomJointManager()
    : mJointRegistryCounter(1)
{
}

PhysXCustomJointManager::~PhysXCustomJointManager()
{
}

CustomPhysXJoint* PhysXCustomJointManager::createCustomJoint(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, const usdparser::CustomPhysxJointDesc& jointDesc,::physx::PxRigidActor* actor0, const ::physx::PxTransform& localFrame0,
    ::physx::PxRigidActor* actor1, const ::physx::PxTransform& localFrame1)
{
    CustomPhysXJoint* customJoint = nullptr;
    // jointDesc.customJointToken is a source-interned TokenId (ADR-0019
    // increment 7); mCustomJointTypeMap stays plain-string-keyed (registerCustomJoint
    // takes a bare `const char*`, with no source to intern against), so bridge
    // through attachedStage's own source (ADR-0019) rather than the process-wide
    // "active attach", which is null whenever 2+ attaches are simultaneously live.
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    const std::string customJointTypeStr =
        source ? std::string(source->tokenToString(jointDesc.customJointToken)) : std::string();
    if (customJointTypeStr.empty() && jointDesc.customJointToken.valid())
    {
        CARB_LOG_ERROR(
            "Custom Joint: failed to resolve type token (id %u) for prim (key %llu), joint will not be created.",
            jointDesc.customJointToken.id, static_cast<unsigned long long>(primKey.handle));
    }
    CustomJointTypeMap::const_iterator fit = mCustomJointTypeMap.find(customJointTypeStr);
    if (fit != mCustomJointTypeMap.end())
    {
        const CustomJointInfo& jointInfo = fit->second;
        CustomJointFlag::Enum inFlags = CustomJointFlag::Enum(0);
        // The consumer callback receives the attach the joint belongs to, not a stage id (ADR-0016).
        const AttachHandle attachHandle = attachedStage.getAttachHandle();
        if (jointInfo.customJointCb.createJointFn(primKey, attachHandle, actor0, localFrame0, actor1, localFrame1, inFlags, jointInfo.customJointCb.userData))
        {
            PxConstraintFlag::Enum flags = convertJointFlags(inFlags);
            customJoint = ICE_NEW(CustomPhysXJoint)(primKey, jointDesc, *OmniPhysX::getInstance().getPhysXSetup().getPhysics(), jointInfo, flags, actor0, actor1);
            mCustomJointMap[primKey] = customJoint;
        }
    }
    return customJoint;
}

void PhysXCustomJointManager::removeCustomJoint(omni::physics::parse::ObjectKey primKey)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(primKey);
    if (fit != mCustomJointMap.end())
    {
        mCustomJointMap.erase(fit);
    }
}

size_t PhysXCustomJointManager::registerCustomJoint(const char* jointPrimType, ICustomJointCallback& jointCallback, ::physx::PxConstraintSolverPrep jointPrepFn, size_t jointDataSize)
{
    const std::string jointPrimTypeStr(jointPrimType);
    if (mCustomJointTypeMap.find(jointPrimTypeStr) != mCustomJointTypeMap.end())
    {
        CARB_LOG_ERROR("Custom Joint Type (%s) already registered.", jointPrimType);
        return kInvalidCustomJointRegId;
    }

    if (!jointCallback.createJointFn || !jointCallback.getConstantBlockFn || !jointCallback.onComShiftFn
        || !jointCallback.onOriginShift || !jointCallback.prepareJointDataFn || !jointCallback.releaseJointFn)
    {
        CARB_LOG_ERROR("Custom Joint Type (%s) has invalid joint callback, please provide all functions.", jointPrimType);
        return kInvalidCustomJointRegId;
    }

    if (!jointPrepFn)
    {
        CARB_LOG_ERROR("Custom Joint Type (%s) has invalid joint prep function, please provide it.", jointPrimType);
        return kInvalidCustomJointRegId;
    }

    const size_t currentRegistryCounter = mJointRegistryCounter;
    const ::physx::PxU32 typeId = OmniPhysX::getInstance().getFreeTypeId();
    CustomJointInfo info = { jointPrimTypeStr, jointCallback, jointPrepFn, jointDataSize, typeId };
    mCustomJointRegistryMap[currentRegistryCounter] = info;
    mCustomJointTypeMap[jointPrimTypeStr] = info;
    mJointRegistryCounter++;

    // Register with the USD-free parse-core registry so both the native USD walker
    // and the ovstage walker recognize this custom joint prim type (REQ-PARSE-CORE-005).
    omni::physics::parse::registerCustomToken(omni::physics::parse::CustomTokenKind::eJoint, jointPrimType);

    return currentRegistryCounter;
}

void PhysXCustomJointManager::unregisterCustomJoint(size_t id)
{
    CustomJointRegistryMap::const_iterator fit = mCustomJointRegistryMap.find(id);
    if (fit != mCustomJointRegistryMap.end())
    {
        const std::string& jt = fit->second.jointPrimType;
        mCustomJointTypeMap.erase(jt);
        omni::physics::parse::unregisterCustomToken(omni::physics::parse::CustomTokenKind::eJoint, jt);
        mCustomJointRegistryMap.erase(fit);

    }
}

void PhysXCustomJointManager::markJointDirty(omni::physics::parse::ObjectKey key)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(key);
    if (fit != mCustomJointMap.end())
    {
        fit->second->getConstraint()->markDirty();
    }
}

void PhysXCustomJointManager::setJointFlags(omni::physics::parse::ObjectKey key, CustomJointFlag::Enum inFlags)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(key);
    if (fit != mCustomJointMap.end())
    {
        PxConstraintFlags currentFlags = fit->second->getConstraint()->getFlags();
        modifyJointFlags(inFlags, currentFlags);
        fit->second->getConstraint()->setFlags(currentFlags);
    }
}

CustomJointFlag::Enum PhysXCustomJointManager::getJointFlags(omni::physics::parse::ObjectKey key)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(key);
    if (fit != mCustomJointMap.end())
    {
        return convertJointFlags(fit->second->getConstraint()->getFlags());
    }
    return CustomJointFlag::Enum(0);
}

size_t registerCustomJoint(const char* jointPrimType, ICustomJointCallback& jointCallback, ::physx::PxConstraintSolverPrep jointPrepFn, size_t jointDataSize)
{
    return OmniPhysX::getInstance().getCustomJointManager().registerCustomJoint(jointPrimType, jointCallback, jointPrepFn, jointDataSize);
}

void unregisterCustomJoint(size_t id)
{
    OmniPhysX::getInstance().getCustomJointManager().unregisterCustomJoint(id);
}

void markJointDirty(omni::physics::parse::ObjectKey key)
{
    OmniPhysX::getInstance().getCustomJointManager().markJointDirty(key);
}

void setJointFlags(omni::physics::parse::ObjectKey key, CustomJointFlag::Enum flags)
{
    OmniPhysX::getInstance().getCustomJointManager().setJointFlags(key, flags);
}

CustomJointFlag::Enum getJointFlags(omni::physics::parse::ObjectKey key)
{
    return OmniPhysX::getInstance().getCustomJointManager().getJointFlags(key);
}

}
}

void fillInterface(omni::physx::IPhysxCustomJoint& iface)
{
    iface.registerCustomJoint = omni::physx::registerCustomJoint;
    iface.unregisterCustomJoint = omni::physx::unregisterCustomJoint;
    iface.markJointDirty = omni::physx::markJointDirty;
    iface.setJointFlags = omni::physx::setJointFlags;
    iface.getJointFlags = omni::physx::getJointFlags;
}
