// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationMimicJointSim.h"
#include "ScArticulationMimicJointCore.h"
#include "PxArticulationReducedCoordinate.h"
#include "ScArticulationSim.h"
#include "PxArticulationReducedCoordinate.h"
#include "DyArticulationMimicJointCore.h"

namespace physx
{

Sc::ArticulationMimicJointSim::ArticulationMimicJointSim(ArticulationMimicJointCore& mimicJointCore, Scene& scene) :
	mScene(scene), mCore(mimicJointCore),
	mLLIndex(0xffffffff)
{
	mimicJointCore.setSim(this);
	mLLMimicJoint.axisA = mimicJointCore.mAxisA;
	mLLMimicJoint.axisB = mimicJointCore.mAxisB;
	mLLMimicJoint.gearRatio = mimicJointCore.mGearRatio;
	mLLMimicJoint.offset = mimicJointCore.mOffset;
	mLLMimicJoint.naturalFrequency = mimicJointCore.mNaturalFrequency;
	mLLMimicJoint.dampingRatio = mimicJointCore.mDampingRatio;
}

Sc::ArticulationMimicJointSim::~ArticulationMimicJointSim()
{
	mCore.setSim(NULL);
}

void Sc::ArticulationMimicJointSim::setGearRatio(const PxReal gearRatio)
{
	mLLMimicJoint.gearRatio = gearRatio;
	mArticulationSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT);
}

void Sc::ArticulationMimicJointSim::setOffset(const PxReal offset)
{
	mLLMimicJoint.offset = offset;
	mArticulationSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT);
}

void Sc::ArticulationMimicJointSim::setNaturalFrequency(const PxReal naturalFrequency)
{
	mLLMimicJoint.naturalFrequency = naturalFrequency;
	mArticulationSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT);
}

void Sc::ArticulationMimicJointSim::setDampingRatio(const PxReal dampingRatio)
{
	mLLMimicJoint.dampingRatio = dampingRatio;
	mArticulationSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT);
}


}

