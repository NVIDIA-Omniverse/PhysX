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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScArticulationJointCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationSim.h"
#include "ScArticulationJointSim.h"
#include "ScBodyCore.h"
#include "ScPhysics.h"

using namespace physx;

Sc::ArticulationJointCore::ArticulationJointCore(const PxTransform& parentFrame, const PxTransform& childFrame) :
	mCore			(parentFrame, childFrame),
	mSim			(NULL),
	mArticulation	(NULL),
	mRootType		(NULL),
	mLLLinkIndex	(0xffffffff)
{
}

Sc::ArticulationJointCore::~ArticulationJointCore()
{
	PX_ASSERT(getSim() == 0);
}

void Sc::ArticulationJointCore::setSimDirty()
{
	Sc::ArticulationJointSim* sim = getSim();
	if(sim)
		sim->setDirty();
}

void Sc::ArticulationJointCore::setParentPose(const PxTransform& t)
{
	mCore.parentPose = t;
	setDirty(Dy::ArticulationJointCoreDirtyFlag::eFRAME);
}

void Sc::ArticulationJointCore::setChildPose(const PxTransform& t)
{
	mCore.childPose = t;
	setDirty(Dy::ArticulationJointCoreDirtyFlag::eFRAME);
}

void Sc::ArticulationJointCore::setTargetP(PxArticulationAxis::Enum axis, PxReal targetP)
{
	mCore.targetP[axis] = targetP;
	
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim)
	{
		Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		Dy::ArticulationData& data = llarticulation->getArticulationData();
		//Dy::ArticulationJointCoreData* jointData = data.getJointData();
		//Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		Dy::ArticulationJointTargetData* targetData = data.getJointTranData();
		Dy::ArticulationJointTargetData& targetDatum = targetData[mLLLinkIndex];

		PxReal* jointTargetPositions = targetDatum.targetJointPosition;
	
		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
		{
			jointTargetPositions[dofId] = targetP;

			setDirty(Dy::ArticulationJointCoreDirtyFlag::eTARGETPOSE);
		}
	}
}

void Sc::ArticulationJointCore::setTargetV(PxArticulationAxis::Enum axis, PxReal targetV)
{
	mCore.targetV[axis] = targetV;

	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->getLLArticulationInitialized())
	{
		Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		Dy::ArticulationData& data = llarticulation->getArticulationData();
		//Dy::ArticulationJointCoreData* jointData = data.getJointData();
		//Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		Dy::ArticulationJointTargetData* targetData = data.getJointTranData();
		Dy::ArticulationJointTargetData& targetDatum = targetData[mLLLinkIndex];

		PxReal* jointTargetVelocities = targetDatum.targetJointVelocity;

		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
		{
			jointTargetVelocities[dofId] = targetV;

			setDirty(Dy::ArticulationJointCoreDirtyFlag::eTARGETVELOCITY);
		}
	}
}

void Sc::ArticulationJointCore::setArmature(PxArticulationAxis::Enum axis, PxReal armature)
{
	mCore.armature[axis] = armature;

	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->getLLArticulationInitialized())
	{
		Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		Dy::ArticulationData& data = llarticulation->getArticulationData();

		Dy::ArticulationJointTargetData* targetData = data.getJointTranData();
		Dy::ArticulationJointTargetData& targetDatum = targetData[mLLLinkIndex];

		PxReal* jArmatures = targetDatum.armature;

		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
		{
			jArmatures[dofId] = armature;

			setDirty(Dy::ArticulationJointCoreDirtyFlag::eARMATURE);
		}
	}
}

void Sc::ArticulationJointCore::setJointPosition(PxArticulationAxis::Enum axis, const PxReal jointPos)
{
	mCore.jointPos[axis] = jointPos;
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->getLLArticulationInitialized())
	{
		Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		Dy::ArticulationData& data = llarticulation->getArticulationData();
		Dy::ArticulationJointCoreData* jointData = data.getJointData();
		Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		PxReal* jointPositions = data.getJointPositions();
		PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];

		if (dofId != 0xff)
		{
			jPosition[dofId] = jointPos;

			////replace with update kinematics
			//llarticulation->teleportLinks(data);
			//llarticulation->computeLinkVelocities(data);

			//artiSim->setJointPosition(axis, jointPos);
			setDirty(Dy::ArticulationJointCoreDirtyFlag::eJOINT_POS);
		}
	}
}

PxReal Sc::ArticulationJointCore::getJointPosition(PxArticulationAxis::Enum axis) const
{
	PxReal jointPos = mCore.jointPos[axis];
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->getLLArticulationInitialized())
	{
		const Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		const Dy::ArticulationData& data = llarticulation->getArticulationData();
		const Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		const PxReal* jointPositions = data.getJointPositions();
		const PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];

		if(dofId != 0xff)
			jointPos = jPosition[dofId];
	}

	return jointPos;
}

void Sc::ArticulationJointCore::setJointVelocity(PxArticulationAxis::Enum axis, const PxReal jointVel)
{
	mCore.jointVel[axis] = jointVel;

	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->getLLArticulationInitialized())
	{
		Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		Dy::ArticulationData& data = llarticulation->getArticulationData();
		Dy::ArticulationJointCoreData* jointData = data.getJointData();
		Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		PxReal* jointVelocities = data.getJointVelocities();
		PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
		{
			jVelocity[dofId] = jointVel;

			//llarticulation->computeLinkVelocities(data);

			setDirty(Dy::ArticulationJointCoreDirtyFlag::eJOINT_VEL);
		}
	}
}

PxReal Sc::ArticulationJointCore::getJointVelocity(PxArticulationAxis::Enum axis) const
{
	PxReal jointVel = mCore.jointVel[axis];
	ArticulationSim* artiSim = mArticulation->getSim();
	if (artiSim && artiSim->getLLArticulationInitialized())
	{
		const Dy::FeatherstoneArticulation* llarticulation = artiSim->getLowLevelArticulation();
		const Dy::ArticulationData& data = llarticulation->getArticulationData();
		const Dy::ArticulationJointCoreData* jointData = data.getJointData();
		const Dy::ArticulationJointCoreData& jointDatum = jointData[mLLLinkIndex];

		const PxReal* jointVelocities = data.getJointVelocities();
		const PxReal* jVelocities = &jointVelocities[jointDatum.jointOffset];

		const PxU32 dofId = mCore.invDofIds[axis];
		if (dofId != 0xff)
			jointVel = jVelocities[dofId];
	}

	return jointVel;
}

void Sc::ArticulationJointCore::setLimit(PxArticulationAxis::Enum axis, const PxArticulationLimit& limit)
{
	mCore.initLimit(axis, limit);
	
    setDirty(Dy::ArticulationJointCoreDirtyFlag::eLIMIT);
}

void Sc::ArticulationJointCore::setDrive(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive)
{
	mCore.initDrive(axis, drive);
	setDirty(Dy::ArticulationJointCoreDirtyFlag::eDRIVE);
}
