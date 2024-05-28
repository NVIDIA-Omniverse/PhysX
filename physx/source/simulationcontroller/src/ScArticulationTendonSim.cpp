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

#include "ScArticulationTendonSim.h"
#include "ScArticulationTendonCore.h"
#include "ScArticulationAttachmentCore.h"
#include "ScArticulationTendonJointCore.h"
#include "ScArticulationJointCore.h"
#include "ScScene.h"
#include "DyArticulationTendon.h"
#include "ScArticulationSim.h"


using namespace physx;

Sc::ArticulationSpatialTendonSim::ArticulationSpatialTendonSim(ArticulationSpatialTendonCore& tendon, Scene& scene) :
	mTendonCore(tendon), mScene(scene)
{
	mTendonCore.setSim(this);
	mLLTendon.mStiffness = tendon.mStiffness;
	mLLTendon.mDamping = tendon.mDamping;
	mLLTendon.mOffset = tendon.mOffset;
	mLLTendon.mLimitStiffness = tendon.mLimitStiffness;
}


Sc::ArticulationSpatialTendonSim::~ArticulationSpatialTendonSim()
{
	mTendonCore.setSim(NULL);
}


void Sc::ArticulationSpatialTendonSim::setStiffness(const PxReal stiffness)
{
	mLLTendon.mStiffness = stiffness;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON);
}

PxReal Sc::ArticulationSpatialTendonSim::getStiffness() const
{
	return mLLTendon.mStiffness;
}

void Sc::ArticulationSpatialTendonSim::setDamping(const PxReal damping)
{
	mLLTendon.mDamping = damping;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON);
}

PxReal Sc::ArticulationSpatialTendonSim::getDamping() const
{
	return mLLTendon.mDamping;
}

void Sc::ArticulationSpatialTendonSim::setLimitStiffness(const PxReal stiffness)
{
	mLLTendon.mLimitStiffness = stiffness;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal Sc::ArticulationSpatialTendonSim::getLimitStiffness() const
{
	return mLLTendon.mLimitStiffness;
}

void Sc::ArticulationSpatialTendonSim::setOffset(const PxReal offset)
{
	mLLTendon.mOffset = offset;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON);
}

PxReal Sc::ArticulationSpatialTendonSim::getOffset() const
{
	return mLLTendon.mOffset;
}


void Sc::ArticulationSpatialTendonSim::setAttachmentCoefficient(ArticulationAttachmentCore& core, const PxReal coefficient)
{
	const PxU32 index = core.mAttachmentIndex;

	Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.coefficient = coefficient;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);
}

void Sc::ArticulationSpatialTendonSim::setAttachmentRelativeOffset(ArticulationAttachmentCore& core, const PxVec3& offset)
{
	const PxU32 index = core.mAttachmentIndex;

	Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.relativeOffset = offset;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);
}

void Sc::ArticulationSpatialTendonSim::setAttachmentLimits(ArticulationAttachmentCore& core, const PxReal lowLimit, const PxReal highLimit)
{
	const PxU32 index = core.mAttachmentIndex;

	Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.lowLimit = lowLimit;
	attachment.highLimit = highLimit;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);
}

void Sc::ArticulationSpatialTendonSim::setAttachmentRestLength(ArticulationAttachmentCore& core, const PxReal restLength)
{
	const PxU32 index = core.mAttachmentIndex;
	Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);
	attachment.restLength = restLength;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT);

}


void Sc::ArticulationSpatialTendonSim::addAttachment(ArticulationAttachmentCore& core)
{

	const PxU32 index = mLLTendon.getNewID();

	Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	attachment.relativeOffset = core.mRelativeOffset;
	attachment.linkInd = PxU16(core.mLLLinkIndex);
	attachment.lowLimit = core.mLowLimit;
	attachment.highLimit = core.mHighLimit;
	attachment.coefficient = core.mCoefficient;
	attachment.myInd = index;
	attachment.children = 0;
	attachment.childCount = 0;
	attachment.restLength = core.mRestLength;

	core.mAttachmentIndex = index;
	core.mTendonSim = this;

	if (core.mParent)
	{
		const PxU32 parentIndex = core.mParent->mAttachmentIndex;
		attachment.parent = parentIndex;
		mLLTendon.getAttachment(parentIndex).children |= Dy::ArticulationAttachmentBitField(1) << index;
		mLLTendon.getAttachment(parentIndex).childCount++;
	}
	else
	{
		attachment.parent = DY_ARTICULATION_ATTACHMENT_NONE;
	}

}

void Sc::ArticulationSpatialTendonSim::removeAttachment(ArticulationAttachmentCore& core)
{
	const PxU32 index = core.mAttachmentIndex;

	Dy::ArticulationAttachment& attachment = mLLTendon.getAttachment(index);

	PX_ASSERT(attachment.childCount == 0);

	if (attachment.parent != DY_ARTICULATION_ATTACHMENT_NONE)
	{
		Dy::ArticulationAttachment& parent = mLLTendon.getAttachment(attachment.parent);
		parent.children &= ~(Dy::ArticulationAttachmentBitField(1) << index);
		parent.childCount--;
	}

	mLLTendon.freeID(index);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::ArticulationFixedTendonSim::ArticulationFixedTendonSim(ArticulationFixedTendonCore& tendon, Scene& scene) :
	mTendonCore(tendon), mScene(scene)
{
	mTendonCore.setSim(this);
	mLLTendon.mStiffness = tendon.mStiffness;
	mLLTendon.mDamping = tendon.mDamping;
	mLLTendon.mOffset = tendon.mOffset;
	mLLTendon.mLimitStiffness = tendon.mLimitStiffness;
	mLLTendon.mLowLimit = tendon.mLowLimit;
	mLLTendon.mHighLimit = tendon.mHighLimit;
	mLLTendon.mRestLength = tendon.mRestLength;
}

Sc::ArticulationFixedTendonSim::~ArticulationFixedTendonSim()
{
	mTendonCore.setSim(NULL);
}

void Sc::ArticulationFixedTendonSim::setStiffness(const PxReal stiffness)
{
	mLLTendon.mStiffness = stiffness;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal Sc::ArticulationFixedTendonSim::getStiffness() const
{
	return mLLTendon.mStiffness;
}

void Sc::ArticulationFixedTendonSim::setDamping(const PxReal damping)
{
	mLLTendon.mDamping = damping;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal Sc::ArticulationFixedTendonSim::getDamping() const
{
	return mLLTendon.mDamping;
}

void Sc::ArticulationFixedTendonSim::setLimitStiffness(const PxReal stiffness)
{
	mLLTendon.mLimitStiffness = stiffness;

	Dy::FeatherstoneArticulation* llArticulation = static_cast<Dy::FeatherstoneArticulation*>(mArtiSim->getLowLevelArticulation());
	llArticulation->setGpuDirtyFlag(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal Sc::ArticulationFixedTendonSim::getLimitStiffness() const
{
	return mLLTendon.mLimitStiffness;
}

void Sc::ArticulationFixedTendonSim::setOffset(const PxReal offset)
{
	mLLTendon.mOffset = offset;

	mArtiSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal Sc::ArticulationFixedTendonSim::getOffset() const
{
	return mLLTendon.mOffset;
}

void Sc::ArticulationFixedTendonSim::setSpringRestLength(const PxReal restLength)
{
	mLLTendon.mRestLength = restLength;

	mArtiSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

PxReal Sc::ArticulationFixedTendonSim::getSpringRestLength() const
{
	return mLLTendon.mRestLength;
}


void Sc::ArticulationFixedTendonSim::setLimitRange(const PxReal lowLimit, const PxReal highLimit)
{
	mLLTendon.mLowLimit = lowLimit;
	mLLTendon.mHighLimit = highLimit;

	mArtiSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON);
}

void Sc::ArticulationFixedTendonSim::getLimitRange(PxReal& lowLimit, PxReal& highLimit) const
{
	lowLimit = mLLTendon.mLowLimit;
	highLimit = mLLTendon.mHighLimit;
}

void Sc::ArticulationFixedTendonSim::addTendonJoint(ArticulationTendonJointCore& tendonJointCore)
{

	const PxU32 jointIndex = mLLTendon.getNewID();

	Dy::ArticulationTendonJoint& tendonJoint = mLLTendon.getTendonJoint(jointIndex);

	tendonJoint.axis = PxU16(tendonJointCore.axis);
	tendonJoint.coefficient = tendonJointCore.coefficient;
	tendonJoint.recipCoefficient = tendonJointCore.recipCoefficient;
	tendonJoint.linkInd = PxU16(tendonJointCore.mLLLinkIndex);
	tendonJoint.children = 0;
	tendonJoint.childCount = 0;

	tendonJointCore.mLLTendonJointIndex = jointIndex;
	//tendonJointCore.mLLTendonJoint = &tendonJoint;
	tendonJointCore.mTendonSim = this;

	if (tendonJointCore.mParent)
	{
		const PxU32 parentIndex = tendonJointCore.mParent->mLLTendonJointIndex;
		tendonJoint.parent = parentIndex;
		mLLTendon.getTendonJoint(parentIndex).children |= Dy::ArticulationAttachmentBitField(1) << jointIndex;
		mLLTendon.getTendonJoint(parentIndex).childCount++;
	}
	else
	{
		tendonJoint.parent = DY_ARTICULATION_ATTACHMENT_NONE;
	}
	
}

void Sc::ArticulationFixedTendonSim::removeTendonJoint(ArticulationTendonJointCore& core)
{
	const PxU32 index = core.mLLTendonJointIndex;

	Dy::ArticulationTendonJoint& tendonJoint = mLLTendon.getTendonJoint(index);

	PX_ASSERT(tendonJoint.childCount == 0);

	if (tendonJoint.parent != DY_ARTICULATION_ATTACHMENT_NONE)
	{
		Dy::ArticulationTendonJoint& parent = mLLTendon.getTendonJoint(tendonJoint.parent);
		parent.children &= ~(Dy::ArticulationAttachmentBitField(1) << index);
		parent.childCount--;
	}

	mLLTendon.freeID(index);
}

void Sc::ArticulationFixedTendonSim::setTendonJointCoefficient(ArticulationTendonJointCore& core, const PxArticulationAxis::Enum axis, const float coefficient, const float recipCoefficient)
{
	const PxU32 index = core.mLLTendonJointIndex;

	Dy::ArticulationTendonJoint& tendonJoint = mLLTendon.getTendonJoint(index);
	tendonJoint.axis = PxU16(axis);
	tendonJoint.coefficient = coefficient;
	tendonJoint.recipCoefficient = recipCoefficient;

	mArtiSim->setArticulationDirty(Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT);

}


