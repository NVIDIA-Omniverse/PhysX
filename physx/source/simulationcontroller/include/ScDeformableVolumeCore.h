// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

class DeformableVolumeCore : public ActorCore
{
public:
	// PX_SERIALIZATION
	DeformableVolumeCore(const PxEMPTY) : ActorCore(PxEmpty){}
	//~PX_SERIALIZATION
	DeformableVolumeCore();
	~DeformableVolumeCore();

	//---------------------------------------------------------------------------------
	// PxActor API
	//---------------------------------------------------------------------------------

	void						setActorFlags(PxActorFlags flags);
	PxActorFlags				getActorFlags() const { return mCore.actorFlags; }

	//---------------------------------------------------------------------------------
	// PxDeformableBody API
	//---------------------------------------------------------------------------------

	void						setBodyFlags(PxDeformableBodyFlags flags);
	PxDeformableBodyFlags		getBodyFlags() const { return mCore.bodyFlags; }

	void						setLinearDamping(const PxReal linearDamping);
	PxReal						getLinearDamping() const { return mCore.linearDamping; }

	void						setMaxLinearVelocity(const PxReal maxLinearVelocity);
	PxReal						getMaxLinearVelocity() const { return mCore.maxLinearVelocity; }

	void						setMaxPenetrationBias(const PxReal maxPenetrationBias);
	PxReal						getMaxPenetrationBias() const { return mCore.maxPenetrationBias; }

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

	PxU32						addRigidAttachment(Sc::BodyCore* core, PxU32 vertId, const PxVec3& actorSpacePose, bool doConversion);
	void						removeRigidAttachment(Sc::BodyCore* core, PxU32 handle);

	void						addTetRigidFilter(Sc::BodyCore* core, PxU32 tetIdx);
	void						removeTetRigidFilter(Sc::BodyCore* core, PxU32 tetIdx);

	PxU32						addTetRigidAttachment(Sc::BodyCore* core, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose,
									bool doConversion);

	void						addSoftBodyFilter(Sc::DeformableVolumeCore& core, PxU32 tetIdx0, PxU32 tetIdx1);
	void						removeSoftBodyFilter(Sc::DeformableVolumeCore& core, PxU32 tetIdx0, PxU32 tetIdx1);
	void						addSoftBodyFilters(Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
	void						removeSoftBodyFilters(Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

	PxU32						addSoftBodyAttachment(Sc::DeformableVolumeCore& core, PxU32 tetIdx0, const PxVec4& triBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
									bool doConversion);
	void						removeSoftBodyAttachment(Sc::DeformableVolumeCore& core, PxU32 handle);

	void						addClothFilter(Sc::DeformableSurfaceCore& core, PxU32 triIdx, PxU32 tetIdx);
	void						removeClothFilter(Sc::DeformableSurfaceCore& core, PxU32 triIdx, PxU32 tetIdx);

	PxU32						addClothAttachment(Sc::DeformableSurfaceCore& core, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric,
									bool doConversion);
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
