// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-PARAMETERS-001
// @covers AC-1

// @implements REQ-CUSTOM-LIFETIME-001
// @covers AC-4 AC-1 AC-2

// @implements REQ-ATTACH-IO-001
// @covers AC-3

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */
/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-9 AC-11 AC-12 AC-13
 */

#include <carb/logging/Log.h>
#include <cstring>

#include "PhysXCustomJoint.h"
#include "OmniPhysX.h"
#include "RuntimeExecution.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physics/parse/CustomTokens.h>

using namespace ::physx;

namespace omni
{
namespace physx
{

namespace
{
// The SDK copies the constant block into solver tasks. Carry the original
// function in that copy so the callback guard is entered on the executing worker.
constexpr size_t kShaderHeaderSize = 16;
PxU32 guardedSolverPrep(Px1DConstraint* constraints, PxVec3p& bodyAWorldOffset,
    PxU32 maxConstraints, PxConstraintInvMassScale& invMassScale, const void* constantBlock,
    const PxTransform& bodyAToWorld, const PxTransform& bodyBToWorld, bool useExtendedLimits,
    PxVec3p& cAtW, PxVec3p& cBtW)
{
    RuntimeCallbackScope callbackScope;
    PxConstraintSolverPrep prep;
    std::memcpy(&prep, constantBlock, sizeof(prep));
    const auto* data = static_cast<const unsigned char*>(constantBlock) + kShaderHeaderSize;
    return prep(constraints, bodyAWorldOffset, maxConstraints, invMassScale, data,
                bodyAToWorld, bodyBToWorld, useExtendedLimits, cAtW, cBtW);
}
}

const void* CustomPhysXJoint::wrapConstantBlock(const void* data) const
{
    mShaderData.resize((kShaderHeaderSize + mCustomJointInfo.jointDataSize + 15) / 16);
    auto* bytes = reinterpret_cast<unsigned char*>(mShaderData.data());
    std::memcpy(bytes, &mCustomJointInfo.solverPrepFn, sizeof(mCustomJointInfo.solverPrepFn));
    if (data && mCustomJointInfo.jointDataSize)
        std::memcpy(bytes + kShaderHeaderSize, data, mCustomJointInfo.jointDataSize);
    return bytes;
}

CustomPhysXJoint::CustomPhysXJoint(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key, const usdparser::CustomPhysxJointDesc& jointDesc,::physx::PxPhysics& physics, const CustomJointInfo& jointInfo, PxConstraintFlag::Enum flags,
    ::physx::PxRigidActor* actor0, ::physx::PxRigidActor* actor1)
    : mAttachedStage(owner), mJointKey(key), mCustomJointInfo(jointInfo)
{
    RuntimeCallbackScope callbackScope;
    uint16_t constraintFlags = flags;
    if (jointDesc.enableCollision)
        constraintFlags |= PxConstraintFlag::eCOLLISION_ENABLED;
    else
        constraintFlags &= ~(uint16_t)PxConstraintFlag::eCOLLISION_ENABLED;

    PxConstraintShaderTable shaderTable = { guardedSolverPrep, nullptr, (PxConstraintFlag::Enum)constraintFlags };
    mConstraint = physics.createConstraint(actor0, actor1, *this, shaderTable, ::physx::PxU32(kShaderHeaderSize + jointInfo.jointDataSize));

    if (mConstraint)
        mConstraint->setBreakForce(isfinite(jointDesc.breakForce) ? jointDesc.breakForce : FLT_MAX,
            isfinite(jointDesc.breakTorque) ? jointDesc.breakTorque : FLT_MAX);

}

void CustomPhysXJoint::release()
{
    RuntimeCallbackScope callbackScope;
    mConstraint->release();
}

void* CustomPhysXJoint::prepareData()
{
    RuntimeCallbackScope callbackScope;
    return const_cast<void*>(wrapConstantBlock(mCustomJointInfo.customJointCb.prepareJointDataFn(mJointKey, mAttachedStage.getAttachHandle(), mCustomJointInfo.customJointCb.userData)));
}

void  CustomPhysXJoint::onConstraintRelease()
{
    RuntimeCallbackScope callbackScope;
    OmniPhysX::getInstance().getCustomJointManager().removeCustomJoint(mAttachedStage, mJointKey, this);
    try
    {
        mCustomJointInfo.customJointCb.releaseJointFn(mJointKey, mAttachedStage.getAttachHandle(), mCustomJointInfo.customJointCb.userData);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Custom joint release callback threw an exception.");
    }
    delete this;
}

void  CustomPhysXJoint::onComShift(::physx::PxU32 actor)
{
    RuntimeCallbackScope callbackScope;
    mCustomJointInfo.customJointCb.onComShiftFn(mJointKey, mAttachedStage.getAttachHandle(), actor, mCustomJointInfo.customJointCb.userData);
}

void  CustomPhysXJoint::onOriginShift(const ::physx::PxVec3& shift)
{
    RuntimeCallbackScope callbackScope;
    mCustomJointInfo.customJointCb.onOriginShift(mJointKey, mAttachedStage.getAttachHandle(), shift, mCustomJointInfo.customJointCb.userData);
}

void* CustomPhysXJoint::getExternalReference(::physx::PxU32& typeID)
{
    RuntimeCallbackScope callbackScope;
    typeID = mCustomJointInfo.typeId;
    return this;
}

::physx::PxConstraintSolverPrep CustomPhysXJoint::getPrep() const
{
    RuntimeCallbackScope callbackScope;
    return guardedSolverPrep;
}

const void* CustomPhysXJoint::getConstantBlock() const
{
    RuntimeCallbackScope callbackScope;
    return wrapConstantBlock(mCustomJointInfo.customJointCb.getConstantBlockFn(mJointKey, mAttachedStage.getAttachHandle(), mCustomJointInfo.customJointCb.userData));
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
    if (attachedStage.customJoints.count(primKey))
        return nullptr;
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
    const auto types = getCustomJointTypeMap();
    CustomJointTypeMap::const_iterator fit = types.find(customJointTypeStr);
    if (fit != types.end())
    {
        const CustomJointInfo jointInfo = fit->second;
        CustomJointFlag::Enum inFlags = CustomJointFlag::Enum(0);
        // The consumer callback receives the attach the joint belongs to, not a stage id (ADR-0016).
        const AttachHandle attachHandle = attachedStage.getAttachHandle();
        RuntimeCallbackScope callbackScope(attachHandle, false, &attachedStage);
        if (jointInfo.customJointCb.createJointFn(primKey, attachHandle, actor0, localFrame0, actor1, localFrame1, inFlags, jointInfo.customJointCb.userData))
        {
            PxConstraintFlag::Enum flags = convertJointFlags(inFlags);
            bool needsCallbackRelease = true;
            try
            {
                customJoint = ICE_NEW(CustomPhysXJoint)(attachedStage, primKey, jointDesc, *OmniPhysX::getInstance().getPhysXSetup().getPhysics(), jointInfo, flags, actor0, actor1);
                if (!customJoint->getConstraint())
                {
                    delete customJoint;
                    customJoint = nullptr;
                    needsCallbackRelease = false;
                    jointInfo.customJointCb.releaseJointFn(primKey, attachHandle, jointInfo.customJointCb.userData);
                    return nullptr;
                }
                const bool inserted = attachedStage.customJoints.emplace(primKey, customJoint).second;
                if (!inserted)
                {
                    // A local key already names a live constraint; do not replace it.
                    customJoint->release();
                    return nullptr;
                }
            }
            catch (...)
            {
                if (customJoint)
                    customJoint->release();
                else if (needsCallbackRelease)
                    jointInfo.customJointCb.releaseJointFn(primKey, attachHandle, jointInfo.customJointCb.userData);
                throw;
            }
        }
    }
    return customJoint;
}

void PhysXCustomJointManager::clear(const usdparser::AttachedStage& owner)
{
    CustomJointMap retiring;
    retiring.swap(owner.customJoints);
    for (const CustomJointMap::value_type& entry : retiring)
        entry.second->release();
}

void PhysXCustomJointManager::clear()
{
    for (const auto& entry : usdparser::UsdLoad::getUsdLoad()->getAttachedStages())
        clear(*entry.second);
}

void PhysXCustomJointManager::removeCustomJoint(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey primKey, const CustomPhysXJoint* joint)
{
    CustomJointMap& mCustomJointMap = owner.customJoints;
    CustomJointMap::iterator fit = mCustomJointMap.find(primKey);
    if (fit != mCustomJointMap.end() && fit->second == joint)
    {
        mCustomJointMap.erase(fit);
    }
}

size_t PhysXCustomJointManager::registerCustomJoint(const char* jointPrimType, ICustomJointCallback& jointCallback, ::physx::PxConstraintSolverPrep jointPrepFn, size_t jointDataSize)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
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
    CustomJointInfo info = { jointPrimTypeStr, jointCallback, jointPrepFn, jointDataSize, typeId,
        std::make_shared<unsigned char>(0) };
    try
    {
        mCustomJointRegistryMap.emplace(currentRegistryCounter, info);
        mCustomJointTypeMap.emplace(jointPrimTypeStr, info);
        // Publish parse discovery last. No allocation follows successful publication.
        omni::physics::parse::registerCustomToken(omni::physics::parse::CustomTokenKind::eJoint, jointPrimType);
    }
    catch (...)
    {
        mCustomJointTypeMap.erase(jointPrimTypeStr);
        mCustomJointRegistryMap.erase(currentRegistryCounter);
        throw;
    }
    mJointRegistryCounter++;

    return currentRegistryCounter;
}

void PhysXCustomJointManager::unregisterCustomJoint(size_t id)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
    CustomJointRegistryMap::const_iterator fit = mCustomJointRegistryMap.find(id);
    if (fit != mCustomJointRegistryMap.end())
    {
        const std::string& jt = fit->second.jointPrimType;
        mCustomJointTypeMap.erase(jt);
        omni::physics::parse::unregisterCustomToken(omni::physics::parse::CustomTokenKind::eJoint, jt);
        mCustomJointRegistryMap.erase(fit);

    }
}

bool PhysXCustomJointManager::tryUnregisterCustomJoint(size_t id)
{
    if (getRuntimeCallbackDepth())
        return false;
    std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
    const std::unordered_map<uint64_t, omni::physx::CustomJointInfo>::iterator found = mCustomJointRegistryMap.find(id);
    // Exactly two references belong to the registry and type index. A copied
    // index pins even the interval before create enters the plugin callback.
    if (found == mCustomJointRegistryMap.end() || found->second.lifetime.use_count() != 2)
        return false;
    const std::string& token = found->second.jointPrimType;
    mCustomJointTypeMap.erase(token);
    omni::physics::parse::unregisterCustomToken(omni::physics::parse::CustomTokenKind::eJoint, token);
    mCustomJointRegistryMap.erase(found);
    return true;
}

void PhysXCustomJointManager::markJointDirty(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key)
{
    CustomJointMap& mCustomJointMap = owner.customJoints;
    CustomJointMap::iterator fit = mCustomJointMap.find(key);
    if (fit != mCustomJointMap.end())
    {
        fit->second->getConstraint()->markDirty();
    }
}

void PhysXCustomJointManager::setJointFlags(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key, CustomJointFlag::Enum inFlags)
{
    CustomJointMap& mCustomJointMap = owner.customJoints;
    CustomJointMap::iterator fit = mCustomJointMap.find(key);
    if (fit != mCustomJointMap.end())
    {
        // Change only the writable flag. Replaying getFlags() through setFlags()
        // also writes SDK-owned bits such as eBROKEN and rejects the update.
        fit->second->getConstraint()->setFlag(PxConstraintFlag::eALWAYS_UPDATE,
                                             (inFlags & CustomJointFlag::eALWAYS_UPDATE) != 0);
    }
}

CustomJointFlag::Enum PhysXCustomJointManager::getJointFlags(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key)
{
    CustomJointMap& mCustomJointMap = owner.customJoints;
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

bool tryUnregisterCustomJoint(size_t id)
{
    return OmniPhysX::getInstance().getCustomJointManager().tryUnregisterCustomJoint(id);
}

void markJointDirty(AttachHandle attachHandle, omni::physics::parse::ObjectKey key)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return;
    const usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    OmniPhysX::getInstance().getCustomJointManager().markJointDirty(*owner, key);
}

void setJointFlags(AttachHandle attachHandle, omni::physics::parse::ObjectKey key, CustomJointFlag::Enum flags)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return;
    const usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    OmniPhysX::getInstance().getCustomJointManager().setJointFlags(*owner, key, flags);
}

CustomJointFlag::Enum getJointFlags(AttachHandle attachHandle, omni::physics::parse::ObjectKey key)
{
    RuntimeExecutionScope operation(attachHandle);
    if (!operation)
        return CustomJointFlag::Enum(0);
    const usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return CustomJointFlag::Enum(0);
    return OmniPhysX::getInstance().getCustomJointManager().getJointFlags(*owner, key);
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
    iface.tryUnregisterCustomJoint = omni::physx::tryUnregisterCustomJoint;
}
