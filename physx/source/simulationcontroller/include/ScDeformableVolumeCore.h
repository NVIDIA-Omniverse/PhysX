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

	PxU32						addRigidAttachments(Sc::BodyCore* core, PxDeformableAttachmentTargetType::Enum feature,
									const PxU32* indices, const PxVec4* barycentrics, const PxTransform& frame, const PxVec4* coords, PxU32 count);
	void						removeRigidAttachments(Sc::BodyCore* core, PxU32 handle);

	PxU32						addVolumeAttachments(Sc::DeformableVolumeCore& core, PxDeformableAttachmentTargetType::Enum feature0, const PxU32* indices0, const PxVec4* barycentrics0,
									PxDeformableAttachmentTargetType::Enum feature1, const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count);
	void						removeVolumeAttachments(Sc::DeformableVolumeCore& core, PxU32 handle);

	PxU32						addSurfaceAttachments(Sc::DeformableSurfaceCore& core, PxDeformableAttachmentTargetType::Enum feature0, const PxU32* indices0, const PxVec4* barycentrics0,
									PxDeformableAttachmentTargetType::Enum feature1, const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count);
	void						removeSurfaceAttachments(Sc::DeformableSurfaceCore& core, PxU32 handle);

	void						addRigidFilters(Sc::RigidCore* core, PxU32* tetIndices, PxU32 tetIndicesSize);
	void						removeRigidFilters(Sc::RigidCore* core, PxU32* tetIndices, PxU32 tetIndicesSize);

	void						addVolumeFilters(Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
	void						removeVolumeFilters(Sc::DeformableVolumeCore& core, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

	void						addSurfaceFilters(Sc::DeformableSurfaceCore& core, PxU32* triIndices, PxU32* tetIndices, PxU32 indicesSize);
	void						removeSurfaceFilters(Sc::DeformableSurfaceCore& core, PxU32* triIndices, PxU32* tetIndices, PxU32 indicesSize);

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
