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

#ifndef SC_DEFORMABLE_VOLUME_CORE_H
#define SC_DEFORMABLE_VOLUME_CORE_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxDeformableVolume.h"
#include "DyDeformableVolumeCore.h"
#include "foundation/PxAssert.h"
#include "ScActorCore.h"
#include "ScShapeCore.h"
#include "PxFiltering.h"
#include "ScRigidCore.h" //KS - needed for ShapeChangeNotifyFlags. Move to a shared header

namespace physx
{
namespace Sc
{

class DeformableVolumeSim;
class BodyCore;
class DeformableSurfaceCore;
class ParticleSystemCore;

class DeformableVolumeCore : public ActorCore
{
public:
	// PX_SERIALIZATION
	DeformableVolumeCore(const PxEMPTY) : ActorCore(PxEmpty){}
	static void getBinaryMetaData(PxOutputStream& stream);
	//~PX_SERIALIZATION
	DeformableVolumeCore();
	~DeformableVolumeCore();

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

	void						setWakeCounter(const PxReal v);
	void						setWakeCounterInternal(const PxReal v);
	PxReal						getWakeCounter() const { return mCore.wakeCounter; }

	//---------------------------------------------------------------------------------
	// PxDeformableBody API
	//---------------------------------------------------------------------------------

	void						setVolumeFlags(PxDeformableVolumeFlags flags);
	PxDeformableVolumeFlags		getVolumeFlags() const { return mCore.volumeFlags; }

	void						setSelfCollisionStressTolerance(const PxReal selfCollisionStressTolerance);
	PxReal						getSelfCollisionStressTolerance() const { return mCore.selfCollisionStressTolerance; }

	void						setKinematicTargets(const PxVec4* positions);

	PxU32						getGpuIndex()	const;

	void						addParticleFilter(Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId);
	void						removeParticleFilter(Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId);

	PxU32						addParticleAttachment(Sc::ParticleSystemCore* core, PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentric);
	void						removeParticleAttachment(Sc::ParticleSystemCore* core, PxU32 handle);

	void						addRigidFilter(Sc::BodyCore* core, PxU32 vertId);
	void						removeRigidFilter(Sc::BodyCore* core, PxU32 vertId);

	PxU32						addRigidAttachment(Sc::BodyCore* core, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint, bool doConversion);
	void						removeRigidAttachment(Sc::BodyCore* core, PxU32 handle);

	void						addTetRigidFilter(Sc::BodyCore* core, PxU32 tetIdx);
	void						removeTetRigidFilter(Sc::BodyCore* core, PxU32 tetIdx);

	PxU32						addTetRigidAttachment(Sc::BodyCore* core, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose,
									PxConeLimitedConstraint* constraint, bool doConversion);

	void						addSoftBodyFilter(Sc::DeformableVolumeCore& core, PxU32 tetIdx0, PxU32 tetIdx1);
	void						removeSoftBodyFilter(Sc::DeformableVolumeCore& core, PxU32 tetIdx0, PxU32 tetIdx1);
	void						addSoftBodyFilters(Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
	void						removeSoftBodyFilters(Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

	PxU32						addSoftBodyAttachment(Sc::DeformableVolumeCore& core, PxU32 tetIdx0, const PxVec4& triBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
									PxConeLimitedConstraint* constraint, PxReal constraintOffset, bool doConversion);
	void						removeSoftBodyAttachment(Sc::DeformableVolumeCore& core, PxU32 handle);

	void						addClothFilter(Sc::DeformableSurfaceCore& core, PxU32 triIdx, PxU32 tetIdx);
	void						removeClothFilter(Sc::DeformableSurfaceCore& core, PxU32 triIdx, PxU32 tetIdx);

	PxU32						addClothAttachment(Sc::DeformableSurfaceCore& core, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric,
									PxConeLimitedConstraint* constraint, PxReal constraintOffset, bool doConversion);
	void						removeClothAttachment(Sc::DeformableSurfaceCore& core, PxU32 handle);

	//---------------------------------------------------------------------------------
	// Internal API
	//---------------------------------------------------------------------------------

	void						addMaterial(const PxU16 handle);
	void						clearMaterials();
	PxActor*					getPxActor() const;
	void						attachShapeCore(ShapeCore* shapeCore);
	void						attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxDeformableVolumeAuxData* simulationState);
	void						onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags);
	PX_FORCE_INLINE	PxU64&		getGpuMemStat() { return mGpuMemStat; }

	DeformableVolumeSim*								getSim() const;
	PX_FORCE_INLINE	const Dy::DeformableVolumeCore&		getCore() const { return mCore; }
	PX_FORCE_INLINE	Dy::DeformableVolumeCore&			getCore() { return mCore; }

private:
	Dy::DeformableVolumeCore	mCore;
	PxU64						mGpuMemStat;
};

} // namespace Sc
} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
#endif // SC_DEFORMABLE_VOLUME_CORE_H
