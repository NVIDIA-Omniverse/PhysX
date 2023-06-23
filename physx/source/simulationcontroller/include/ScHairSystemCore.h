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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

#ifndef SC_HAIR_SYSTEM_CORE_H
#define SC_HAIR_SYSTEM_CORE_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "ScActorCore.h"
#include "ScHairSystemShapeCore.h"
#include "DyHairSystem.h"

namespace physx
{
namespace Sc
{
class HairSystemSim;
class BodySim;
class SoftBodySim;

class HairSystemCore : public ActorCore
{
	// PX_SERIALIZATION
  public:
	HairSystemCore(const PxEMPTY) : ActorCore(PxEmpty) {}
	static void getBinaryMetaData(PxOutputStream& stream);
	//~PX_SERIALIZATION

	HairSystemCore();
	~HairSystemCore();

	//---------------------------------------------------------------------------------
	// External API
	//---------------------------------------------------------------------------------
	void setMaterial(const PxU16 handle);
	void clearMaterials();

	PxReal getContactOffset() const;
	void setContactOffset(PxReal v);

	PxReal getSleepThreshold() const { return mShapeCore.getLLCore().mSleepThreshold; }
	void setSleepThreshold(const PxReal v);

	PxU16 getSolverIterationCounts() const { return mShapeCore.getLLCore().mSolverIterationCounts; }
	void setSolverIterationCounts(PxU16 c);

	PxReal getWakeCounter() const { return mShapeCore.getLLCore().mWakeCounter; }
	void setWakeCounter(const PxReal v);

	bool isSleeping() const;
	void wakeUp(PxReal wakeCounter);
	void putToSleep();

	PxActor* getPxActor() const;

	void addAttachment(const BodySim& bodySim);
	void removeAttachment(const BodySim& bodySim);
	void addAttachment(const SoftBodySim& sbSim);
	void removeAttachment(const SoftBodySim& sbSim);

	//---------------------------------------------------------------------------------
	// Internal API
	//---------------------------------------------------------------------------------
  public:
	HairSystemSim* getSim() const;

	PX_FORCE_INLINE const HairSystemShapeCore& getShapeCore() const { return mShapeCore; }
	PX_FORCE_INLINE HairSystemShapeCore& getShapeCore() { return mShapeCore; }

	PxHairSystemFlags getFlags() const { return PxHairSystemFlags(mShapeCore.getLLCore().mParams.mFlags); }

	void setFlags(PxHairSystemFlags flags);

  private:
	HairSystemShapeCore mShapeCore;
};
} // namespace Sc
} // namespace physx

#endif
#endif
