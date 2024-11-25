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

#ifndef SC_DEFORMABLE_SURFACE_CORE
#define SC_DEFORMABLE_SURFACE_CORE

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX

#include "PxDeformableSurface.h"
#include "../../lowleveldynamics/include/DyDeformableSurfaceCore.h"
#include "foundation/PxAssert.h"
#include "ScActorCore.h"
#include "ScShapeCore.h"
#include "PxFiltering.h"
#include "ScRigidCore.h" //KS - required for ShapeChangeNotifyFlags. Ideally, we should move that to a separate shared file
#include "PxConeLimitedConstraint.h"

namespace physx
{
namespace Sc
{

class DeformableSurfaceSim;

class DeformableSurfaceCore : public ActorCore
{
public:
	// PX_SERIALIZATION
	DeformableSurfaceCore(const PxEMPTY) : ActorCore(PxEmpty) {}
	static void getBinaryMetaData(PxOutputStream& stream);
	//~PX_SERIALIZATION
	DeformableSurfaceCore();
	~DeformableSurfaceCore();

	//---------------------------------------------------------------------------------
	// PxDeformableBody API
	//---------------------------------------------------------------------------------

	void						setBodyFlags(PxDeformableBodyFlags flags);
	PxDeformableBodyFlags		getBodyFlags() const { return mCore.bodyFlags; }

	void						setLinearDamping(const PxReal linearDamping);
	PxReal						getLinearDamping() const { return mCore.linearDamping; }

	void						setMaxVelocity(const PxReal maxVelocity);
	PxReal						getMaxVelocity() const { return mCore.maxVelocity; }

	void						setMaxDepenetrationVelocity(const PxReal maxDepenetrationVelocity);
	PxReal						getMaxDepenetrationVelocity() const { return mCore.maxDepenetrationVelocity; }

	void						setSolverIterationCounts(PxU16 c);
	PxU16						getSolverIterationCounts() const { return mCore.solverIterationCounts; }

	void						setSleepThreshold(const PxReal sleepThreshold);
	PxReal						getSleepThreshold() const { return mCore.sleepThreshold; }

	void						setSettlingThreshold(const PxReal settlingThreshold);
	PxReal						getSettlingThreshold() const { return mCore.settlingThreshold; }

	void						setSettlingDamping(const PxReal linearDamping);
	PxReal						getSettlingDamping() const { return mCore.settlingDamping; }

	void						setSelfCollisionFilterDistance(const PxReal selfCollisionFilterDistance);
	PxReal						getSelfCollisionFilterDistance() const { return mCore.selfCollisionFilterDistance; }

	//deprecated
	void						setSelfCollisionStressTolerance(const PxReal selfCollisionStressTolerance);
	PxReal						getSelfCollisionStressTolerance() const { return mCore.selfCollisionStressTolerance; }

	void						setWakeCounter(const PxReal v);
	void						setWakeCounterInternal(const PxReal v);
	PxReal						getWakeCounter() const { return mCore.wakeCounter; }

	//---------------------------------------------------------------------------------
	// PxDeformableBody API
	//---------------------------------------------------------------------------------

	void						setSurfaceFlags(PxDeformableSurfaceFlags flags);
	PxDeformableSurfaceFlags	getSurfaceFlags() const { return mCore.surfaceFlags; }

	void						setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency);
	PxU32						getNbCollisionPairUpdatesPerTimestep() const { return mCore.nbCollisionPairUpdatesPerTimestep; }

	void						setNbCollisionSubsteps(const PxU32 frequency);
	PxU32						getNbCollisionSubsteps() const { return mCore.nbCollisionSubsteps; }

	//---------------------------------------------------------------------------------
	// Internal API
	//---------------------------------------------------------------------------------

	PxU32						addRigidAttachment(Sc::BodyCore* core, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
	void						removeRigidAttachment(Sc::BodyCore* core, PxU32 handle);

	void						addTriRigidFilter(Sc::BodyCore* core, PxU32 triIdx);
	void						removeTriRigidFilter(Sc::BodyCore* core, PxU32 triIdx);

	PxU32						addTriRigidAttachment(Sc::BodyCore* core, PxU32 triIdx, const PxVec4& barycentric, 
									const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
	void						removeTriRigidAttachment(Sc::BodyCore* core, PxU32 handle);

	void						addClothFilter(Sc::DeformableSurfaceCore* otherCore, PxU32 otherTriIdx, PxU32 triIdx);
	void						removeClothFilter(Sc::DeformableSurfaceCore* otherCore, PxU32 otherTriIdx0, PxU32 triIdx);

	PxU32						addClothAttachment(Sc::DeformableSurfaceCore* otherCore, PxU32 otherTriIdx, const PxVec4& otherTriBarycentric, PxU32 triIdx, 
									const PxVec4& triBarycentric);
	void						removeClothAttachment(Sc::DeformableSurfaceCore* otherCore, PxU32 handle);

	void						addMaterial(const PxU16 handle);
	void						clearMaterials();
	PxActor*					getPxActor() const;
	void						attachShapeCore(ShapeCore* shapeCore);
	void						onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags);
	PX_FORCE_INLINE	PxU64&		getGpuMemStat() { return mGpuMemStat; }

	DeformableSurfaceSim*								getSim() const;
	PX_FORCE_INLINE	const Dy::DeformableSurfaceCore&	getCore() const { return mCore; }
	PX_FORCE_INLINE	Dy::DeformableSurfaceCore&			getCore() { return mCore; }

private:
	Dy::DeformableSurfaceCore	mCore;
	PxU64						mGpuMemStat;
};

} // namespace Sc
} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
#endif // SC_DEFORMABLE_SURFACE_CORE
