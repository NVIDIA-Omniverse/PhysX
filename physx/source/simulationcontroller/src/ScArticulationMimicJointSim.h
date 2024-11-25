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

#ifndef SC_ARTICULATION_MIMIC_JOINT_SIM_H
#define SC_ARTICULATION_MIMIC_JOINT_SIM_H

#include "foundation/PxUserAllocated.h"
#include "PxArticulationReducedCoordinate.h"
#include "DyFeatherstoneArticulation.h"
#include "DyArticulationMimicJointCore.h"

namespace physx
{
namespace Sc
{
class Scene;
class ArticulationMimicJointCore;
class ArticulationCore;
class ArticulationSim;

class ArticulationMimicJointSim : public PxUserAllocated
{

	PX_NOCOPY(ArticulationMimicJointSim)

	
public:
	ArticulationMimicJointSim(Sc::ArticulationMimicJointCore& core, Sc::Scene& scene);

	~ArticulationMimicJointSim();

	PX_FORCE_INLINE Sc::Scene& getScene() { return mScene; }
	PX_FORCE_INLINE const Sc::Scene& getScene() const { return mScene; }

	PX_FORCE_INLINE void setLowLevelIndex(const PxU32 llIndex) { mLLIndex = llIndex;}
	PX_FORCE_INLINE PxU32 getLowLevelIndex() const { return mLLIndex; }
	
	PX_FORCE_INLINE Sc::ArticulationMimicJointCore& getCore() { return mCore; }
	PX_FORCE_INLINE const Sc::ArticulationMimicJointCore& getCore() const { return mCore; }

	PX_FORCE_INLINE Dy::ArticulationMimicJointCore& getLLMimicJoint() { return mLLMimicJoint; }

	void setGearRatio(const PxReal gearRatio);
	void setOffset(const PxReal offset);
	void setNaturalFrequency(const PxReal naturalFrequency);
	void setDampingRatio(const PxReal dampingRatio);

	Sc::Scene&							mScene;
	Sc::ArticulationMimicJointCore&		mCore;
	Sc::ArticulationSim*				mArticulationSim;
	Dy::ArticulationMimicJointCore		mLLMimicJoint;
	PxU32								mLLIndex;
};

}
}

#endif //SC_ARTICULATION_MIMIC_JOINT_SIM_H


