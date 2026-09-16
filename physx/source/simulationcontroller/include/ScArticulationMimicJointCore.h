// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ARTICULATION_MIMIC_JOINT_CORE
#define SC_ARTICULATION_MIMIC_JOINT_CORE

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"

namespace physx
{
namespace Sc
{

class ArticulationCore;
class ArticulationMimicJointSim;

class ArticulationMimicJointCore 
{
public:

// PX_SERIALIZATION	
	ArticulationMimicJointCore(const PxEMPTY) :mSim(NULL) {}
//~PX_SERIALIZATION


	ArticulationMimicJointCore() : mSim(NULL) {}

	PX_FORCE_INLINE	void setSim(ArticulationMimicJointSim* sim)
	{
		PX_ASSERT((sim == 0) ^ (mSim == 0));
		mSim = sim;
	}

	PX_FORCE_INLINE	ArticulationMimicJointSim* getSim() const { return mSim; }

	ArticulationMimicJointSim*	mSim;
	PxU32 mAxisA;
	PxU32 mAxisB;
	PxReal mGearRatio;
	PxReal mOffset;
	PxReal mNaturalFrequency;
	PxReal mDampingRatio;
};
}//namespace Sc
}//namespace physx

#endif //SC_ARTICULATION_MIMIC_JOINT_CORE

