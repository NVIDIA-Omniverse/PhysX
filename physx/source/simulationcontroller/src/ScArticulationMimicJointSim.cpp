// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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

