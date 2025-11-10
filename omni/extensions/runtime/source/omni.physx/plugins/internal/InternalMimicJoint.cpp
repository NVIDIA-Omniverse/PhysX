// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>

#include <common/utilities/MemoryMacros.h>

#include <PhysXTools.h>

#include "InternalMimicJoint.h"

#include "InternalScene.h"


using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;


static PxArticulationAxis::Enum getArticulationAxis(const ::physx::PxArticulationJointReducedCoordinate& joint,
    int jointAxis)
{
    if (joint.getJointType() == PxArticulationJointType::ePRISMATIC)
    {
        CARB_ASSERT(jointAxis == MimicJointDesc::eDEFAULT_AXIS);
        return PxArticulationAxis::eX;  // omni.physx arranges joint frames such that X is the motion axis
    }
    else if (joint.getJointType() == PxArticulationJointType::eREVOLUTE_UNWRAPPED)
    {
        CARB_ASSERT(jointAxis == MimicJointDesc::eDEFAULT_AXIS);
        return PxArticulationAxis::eTWIST;  // omni.physx arranges joint frames such that twist=X is the motion axis
    }
    else if (joint.getJointType() == PxArticulationJointType::eSPHERICAL)
    {
        if (jointAxis == eRotX)
            return PxArticulationAxis::eTWIST;
        else if (jointAxis == eRotY)
            return PxArticulationAxis::eSWING1;
        else
        {
            CARB_ASSERT(jointAxis == eRotZ);
            return PxArticulationAxis::eSWING2;
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Mimic Joint: unsupported joint type %d. Behavior will be undefined.\n",
            joint.getJointType());

        return PxArticulationAxis::eX;
    }
}

static PxReal translateGearing(float gearing, PxArticulationAxis::Enum pxMimicJointAxis,
    PxArticulationAxis::Enum pxReferenceJointAxis)
{
    static_assert(PxArticulationAxis::eTWIST < PxArticulationAxis::eX, "");
    static_assert(PxArticulationAxis::eSWING1 < PxArticulationAxis::eX, "");
    static_assert(PxArticulationAxis::eSWING2 < PxArticulationAxis::eX, "");

    PxReal physxGearing;  // PhysX is using radians
    if ((pxMimicJointAxis < PxArticulationAxis::eX) && (pxReferenceJointAxis >= PxArticulationAxis::eX))
    {
        // mimic=angular, reference=linear => gearing = angular/linear

        physxGearing = degToRad(gearing);
    }
    else if ((pxMimicJointAxis >= PxArticulationAxis::eX) && (pxReferenceJointAxis < PxArticulationAxis::eX))
    {
        // mimic=linear, reference=angular => gearing = linear/angular

        physxGearing = radToDeg(gearing);  // this is the same as multiplying by 1 / (pi / 180)
    }
    else
    {
        physxGearing = gearing;
    }

    return physxGearing;
}

static PxReal translateOffset(float offset, PxArticulationAxis::Enum pxMimicJointAxis)
{
    static_assert(PxArticulationAxis::eTWIST < PxArticulationAxis::eX, "");
    static_assert(PxArticulationAxis::eSWING1 < PxArticulationAxis::eX, "");
    static_assert(PxArticulationAxis::eSWING2 < PxArticulationAxis::eX, "");

    PxReal physxOffset;  // PhysX is using radians
    if (pxMimicJointAxis < PxArticulationAxis::eX)
    {
        physxOffset = degToRad(offset);
    }
    else
    {
        physxOffset = offset;
    }

    return physxOffset;
}


InternalMimicJoint::InternalMimicJoint(InternalScene& internalScene,
    const ::physx::PxArticulationJointReducedCoordinate& targetJoint,
    const ::physx::PxArticulationJointReducedCoordinate& referenceJoint,
    int targetJointAxis, int referenceJointAxis, float gearing, float offset, float naturalFrequency, float dampingRatio,
    usdparser::ObjectType objectType)
    : mInternalScene(internalScene)
    , mObjectType(objectType)
{
    const PxArticulationAxis::Enum pxTargetJointAxis = getArticulationAxis(targetJoint, targetJointAxis);
    const PxArticulationAxis::Enum pxReferenceJointAxis = getArticulationAxis(referenceJoint, referenceJointAxis);

    const PxReal pxGearing = translateGearing(gearing, pxTargetJointAxis, pxReferenceJointAxis);

    const PxReal pxOffset = translateOffset(offset, pxTargetJointAxis);

    PxArticulationReducedCoordinate& art = targetJoint.getChildArticulationLink().getArticulation();

    mPhysXMimicJoint = art.createMimicJoint(targetJoint, pxTargetJointAxis,
        referenceJoint, pxReferenceJointAxis,
        pxGearing, pxOffset, naturalFrequency, dampingRatio);

    if (mPhysXMimicJoint)
    {
        internalScene.addMimicJoint(*this);
    }
}


// Constructor used by replicator, it cant call the internalScene.addMimicJoint(*this);, this is not thread safe, will be called after creation
InternalMimicJoint::InternalMimicJoint(InternalScene& internalScene, ::physx::PxArticulationMimicJoint& pxMimicJoint,
    usdparser::ObjectType objectType)
    : mInternalScene(internalScene)
    , mPhysXMimicJoint(&pxMimicJoint)
    , mObjectType(objectType)
{    
}


InternalMimicJoint::~InternalMimicJoint()
{
}


void InternalMimicJoint::release(bool removeFromTrackers, bool releasePhysXObject)
{
    if (removeFromTrackers)
        mInternalScene.removeMimicJoint(*this);

    if (releasePhysXObject)
        releasePhysXMimicJoint();

    if (mOwnsMemory)
    {
        delete this;
    }
}


void InternalMimicJoint::releasePhysXMimicJoint()
{
    // this method is separated from the destructor because InternalMimicJoint might live longer than
    // the PhysX joint. The idea here is that the lifetime of InternalMimicJoint is coupled to the
    // lifetime of the corresponding USD schema. Thus, it might stay around even though the PhysX
    // mimic joint has been released (for example because the articulation has been released).

    if (mPhysXMimicJoint)
    {
        mPhysXMimicJoint->release();
 
        mPhysXMimicJoint = nullptr;
    }
}


PxArticulationReducedCoordinate* InternalMimicJoint::getArticulation() const
{
    if (mPhysXMimicJoint)
    {
        PxArticulationReducedCoordinate* art = &mPhysXMimicJoint->getArticulation();
        return art;
    }

    return nullptr;
}


void InternalMimicJoint::setGearing(float gearing)
{
    if (mPhysXMimicJoint)
    {
        const PxReal pxGearing = translateGearing(gearing, mPhysXMimicJoint->getAxisA(), mPhysXMimicJoint->getAxisB());

        mPhysXMimicJoint->setGearRatio(pxGearing);
    }
}


void InternalMimicJoint::setOffset(float offset)
{
    if (mPhysXMimicJoint)
    {
        const PxReal pxOffset = translateOffset(offset, mPhysXMimicJoint->getAxisA());

        mPhysXMimicJoint->setOffset(pxOffset);
    }
}


void InternalMimicJoint::setNaturalFrequency(float naturalFrequency)
{
    if (mPhysXMimicJoint)
    {
        mPhysXMimicJoint->setNaturalFrequency(naturalFrequency);
    }
}


void InternalMimicJoint::setDampingRatio(float dampingRatio)
{
    if (mPhysXMimicJoint)
    {
        mPhysXMimicJoint->setDampingRatio(dampingRatio);
    }
}

