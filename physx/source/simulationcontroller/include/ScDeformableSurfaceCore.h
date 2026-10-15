// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	//~PX_SERIALIZATION
	DeformableSurfaceCore();
	~DeformableSurfaceCore();

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

	PxU32						addRigidAttachments(Sc::BodyCore* core, PxDeformableAttachmentTargetType::Enum feature,
									const PxU32* indices, const PxVec4* barycentrics, const PxTransform& frame, const PxVec4* coords, PxU32 count);
	void						removeRigidAttachments(Sc::BodyCore* core, PxU32 handle);

	PxU32						addSurfaceAttachments(Sc::DeformableSurfaceCore* otherCore, PxDeformableAttachmentTargetType::Enum feature0, const PxU32* indices0, const PxVec4* barycentrics0,
									PxDeformableAttachmentTargetType::Enum feature1, const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count);
	void						removeSurfaceAttachments(Sc::DeformableSurfaceCore* otherCore, PxU32 handle);

	void						addRigidFilters(Sc::RigidCore* core, PxU32* triIndices, PxU32 triIndicesSize);
	void						removeRigidFilters(Sc::RigidCore* core, PxU32* triIndices, PxU32 triIndicesSize);

	void						addSurfaceFilters(Sc::DeformableSurfaceCore* otherCore, PxU32* otherTriIndices, PxU32* triIndices, PxU32 triIndicesSize);
	void						removeSurfaceFilters(Sc::DeformableSurfaceCore* otherCore, PxU32* otherTriIndices, PxU32* triIndices, PxU32 triIndicesSize);

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
