// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <carb/logging/Log.h>

#include "PhysXCustomJoint.h"
#include "OmniPhysX.h"

using namespace ::physx;

namespace omni
{
namespace physx
{

CustomPhysXJoint::CustomPhysXJoint(const pxr::SdfPath& path, const usdparser::CustomPhysxJointDesc& jointDesc,::physx::PxPhysics& physics, const CustomJointInfo& jointInfo, PxConstraintFlag::Enum flags,
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
    mJointPath = path;
}

void CustomPhysXJoint::release()
{
    mConstraint->release();
}

void* CustomPhysXJoint::prepareData()
{
    return mCustomJointInfo.customJointCb.prepareJointDataFn(mJointPath, mCustomJointInfo.customJointCb.userData);
}

void  CustomPhysXJoint::onConstraintRelease()
{
    mCustomJointInfo.customJointCb.releaseJointFn(mJointPath, mCustomJointInfo.customJointCb.userData);
    OmniPhysX::getInstance().getCustomJointManager().removeCustomJoint(mJointPath);
    delete this;
}

void  CustomPhysXJoint::onComShift(::physx::PxU32 actor)
{
    mCustomJointInfo.customJointCb.onComShiftFn(mJointPath, actor, mCustomJointInfo.customJointCb.userData);
}

void  CustomPhysXJoint::onOriginShift(const ::physx::PxVec3& shift)
{
    mCustomJointInfo.customJointCb.onOriginShift(mJointPath, shift, mCustomJointInfo.customJointCb.userData);
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
    return mCustomJointInfo.customJointCb.getConstantBlockFn(mJointPath, mCustomJointInfo.customJointCb.userData);
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

CustomPhysXJoint* PhysXCustomJointManager::createCustomJoint(const pxr::SdfPath& primPath, const usdparser::CustomPhysxJointDesc& jointDesc,::physx::PxRigidActor* actor0, const ::physx::PxTransform& localFrame0,
    ::physx::PxRigidActor* actor1, const ::physx::PxTransform& localFrame1)
{
    CustomPhysXJoint* customJoint = nullptr;
    CustomJointTypeMap::const_iterator fit = mCustomJointTypeMap.find(jointDesc.customJointToken);
    if (fit != mCustomJointTypeMap.end())
    {
        const CustomJointInfo& jointInfo = fit->second;        
        CustomJointFlag::Enum inFlags = CustomJointFlag::Enum(0);
        if (jointInfo.customJointCb.createJointFn(primPath, OmniPhysX::getInstance().getStageId(), actor0, localFrame0, actor1, localFrame1, inFlags, jointInfo.customJointCb.userData))
        {
            PxConstraintFlag::Enum flags = convertJointFlags(inFlags);
            customJoint = ICE_NEW(CustomPhysXJoint)(primPath, jointDesc, *OmniPhysX::getInstance().getPhysXSetup().getPhysics(), jointInfo, flags, actor0, actor1);
            mCustomJointMap[primPath] = customJoint;
        }
    }
    return customJoint;
}

void PhysXCustomJointManager::removeCustomJoint(const pxr::SdfPath& primPath)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(primPath);
    if (fit != mCustomJointMap.end())
    {
        mCustomJointMap.erase(fit);
    }
}

size_t PhysXCustomJointManager::registerCustomJoint(const pxr::TfToken& jointPrimType, ICustomJointCallback& jointCallback, ::physx::PxConstraintSolverPrep jointPrepFn, size_t jointDataSize)
{
    if (mCustomJointTypeMap.find(jointPrimType) != mCustomJointTypeMap.end())
    {
        CARB_LOG_ERROR("Custom Joint Type (%s) already registered.", jointPrimType.GetText());
        return kInvalidCustomJointRegId;
    }

    if (!jointCallback.createJointFn || !jointCallback.getConstantBlockFn || !jointCallback.onComShiftFn
        || !jointCallback.onOriginShift || !jointCallback.prepareJointDataFn || !jointCallback.releaseJointFn)
    {
        CARB_LOG_ERROR("Custom Joint Type (%s) has invalid joint callback, please provide all functions.", jointPrimType.GetText());
        return kInvalidCustomJointRegId;
    }

    if (!jointPrepFn)
    {
        CARB_LOG_ERROR("Custom Joint Type (%s) has invalid joint prep function, please provide it.", jointPrimType.GetText());
        return kInvalidCustomJointRegId;
    }

    const size_t currentRegistryCounter = mJointRegistryCounter;
    const ::physx::PxU32 typeId = OmniPhysX::getInstance().getFreeTypeId();
    CustomJointInfo info = { jointPrimType, jointCallback, jointPrepFn, jointDataSize, typeId };
    mCustomJointRegistryMap[currentRegistryCounter] = info;
    mCustomJointTypeMap[jointPrimType] = info;
    mJointRegistryCounter++;

    carb::Framework* framework = carb::getFramework();
    omni::physics::schema::IUsdPhysics* usdPhysics = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
    usdPhysics->addCustomJointToken(jointPrimType);

    return currentRegistryCounter;
}

void PhysXCustomJointManager::unregisterCustomJoint(size_t id)
{
    CustomJointRegistryMap::const_iterator fit = mCustomJointRegistryMap.find(id);
    if (fit != mCustomJointRegistryMap.end())
    {
        const pxr::TfToken& jt = fit->second.jointPrimType;
        mCustomJointTypeMap.erase(jt);
        carb::Framework* framework = carb::getFramework();
        omni::physics::schema::IUsdPhysics* usdPhysics = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
        usdPhysics->removeCustomJointToken(jt);
        mCustomJointRegistryMap.erase(fit);

    }
}

void PhysXCustomJointManager::markJointDirty(const pxr::SdfPath& primPath)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(primPath);
    if (fit != mCustomJointMap.end())
    {
        fit->second->getConstraint()->markDirty();
    }
}

void PhysXCustomJointManager::setJointFlags(const pxr::SdfPath& primPath, CustomJointFlag::Enum inFlags)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(primPath);
    if (fit != mCustomJointMap.end())
    {        
        PxConstraintFlags currentFlags = fit->second->getConstraint()->getFlags();
        modifyJointFlags(inFlags, currentFlags);
        fit->second->getConstraint()->setFlags(currentFlags);
    }
}

CustomJointFlag::Enum PhysXCustomJointManager::getJointFlags(const pxr::SdfPath& primPath)
{
    CustomJointMap::iterator fit = mCustomJointMap.find(primPath);
    if (fit != mCustomJointMap.end())
    {
        return convertJointFlags(fit->second->getConstraint()->getFlags());
    }
    return CustomJointFlag::Enum(0);
}

size_t registerCustomJoint(const pxr::TfToken& jointPrimType, ICustomJointCallback& jointCallback, ::physx::PxConstraintSolverPrep jointPrepFn, size_t jointDataSize)
{
    return OmniPhysX::getInstance().getCustomJointManager().registerCustomJoint(jointPrimType, jointCallback, jointPrepFn, jointDataSize);
}

void unregisterCustomJoint(size_t id)
{
    OmniPhysX::getInstance().getCustomJointManager().unregisterCustomJoint(id);
}

void markJointDirty(const pxr::SdfPath& primPath)
{
    OmniPhysX::getInstance().getCustomJointManager().markJointDirty(primPath);
}

void setJointFlags(const pxr::SdfPath& primPath, CustomJointFlag::Enum flags)
{
    OmniPhysX::getInstance().getCustomJointManager().setJointFlags(primPath, flags);
}

CustomJointFlag::Enum getJointFlags(const pxr::SdfPath& primPath)
{
    return OmniPhysX::getInstance().getCustomJointManager().getJointFlags(primPath);
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
