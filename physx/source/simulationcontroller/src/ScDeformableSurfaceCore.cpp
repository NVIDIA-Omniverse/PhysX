// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScDeformableSurfaceCore.h"

#include "ScPhysics.h"
#include "ScDeformableSurfaceSim.h"
#include "DyDeformableSurface.h"
#include "GuTetrahedronMesh.h"
#include "GuBV4.h"
#include "geometry/PxTetrahedronMesh.h"
#include "cudamanager/PxCudaContextManager.h"

using namespace physx;

Sc::DeformableSurfaceCore::DeformableSurfaceCore() :
	ActorCore(PxActorType::eDEFORMABLE_SURFACE, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0),
	mGpuMemStat(0)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	// Dy::DeformableCore
	mCore.sleepThreshold = 5e-5f * scale.speed * scale.speed;
	mCore.solverIterationCounts = (1 << 8) | 4;
	mCore.wakeCounter = Physics::sWakeCounterOnCreation;
	mCore.dirty = true;
}

Sc::DeformableSurfaceCore::~DeformableSurfaceCore() { }

/////////////////////////////////////////////////////////////////////////////////////////
// PxActor API
/////////////////////////////////////////////////////////////////////////////////////////

void Sc::DeformableSurfaceCore::setActorFlags(PxActorFlags flags)
{
	mCore.actorFlags = flags;
	mCore.dirty = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableBody API
/////////////////////////////////////////////////////////////////////////////////////////

void Sc::DeformableSurfaceCore::setBodyFlags(PxDeformableBodyFlags flags)
{
	mCore.bodyFlags = flags;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setLinearDamping(const PxReal v)
{
	mCore.linearDamping = v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setMaxLinearVelocity(const PxReal v)
{
	mCore.maxLinearVelocity = (v > 1e15f) ? PX_MAX_REAL : v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setMaxPenetrationBias(const PxReal v)
{
	mCore.maxPenetrationBias = v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setSolverIterationCounts(const PxU16 c)
{
	mCore.solverIterationCounts = c;
	mCore.dirty = true;
	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().setDynamicsDirty();
}

void Sc::DeformableSurfaceCore::setSleepThreshold(const PxReal v)
{
	mCore.sleepThreshold = v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setSettlingThreshold(const PxReal v)
{
	mCore.settlingThreshold = v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setSettlingDamping(const PxReal v)
{
	mCore.settlingDamping = v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setSelfCollisionFilterDistance(const PxReal v)
{
	mCore.selfCollisionFilterDistance = v;
	mCore.dirty = true;
}

//deprecated
void Sc::DeformableSurfaceCore::setSelfCollisionStressTolerance(const PxReal v)
{
	mCore.selfCollisionStressTolerance = v;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setWakeCounter(const PxReal v)
{
	setWakeCounterInternal(v);
}

void Sc::DeformableSurfaceCore::setWakeCounterInternal(const PxReal v)
{
	mCore.wakeCounter = v;
	mCore.dirty = true;

	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
	{
		sim->onSetWakeCounter();
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableSurface API
/////////////////////////////////////////////////////////////////////////////////////////

void Sc::DeformableSurfaceCore::setSurfaceFlags(PxDeformableSurfaceFlags flags)
{
	mCore.surfaceFlags = flags;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency)
{
	mCore.nbCollisionPairUpdatesPerTimestep = frequency;
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::setNbCollisionSubsteps(const PxU32 frequency)
{
	mCore.nbCollisionSubsteps = frequency;
	mCore.dirty = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Internal API
/////////////////////////////////////////////////////////////////////////////////////////

PxU32 Sc::DeformableSurfaceCore::addRigidAttachments(Sc::BodyCore* core, PxDeformableAttachmentTargetType::Enum feature,
	const PxU32* indices, const PxVec4* barycentrics, const PxTransform& frame, const PxVec4* coords, PxU32 count)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = PX_INVALID_U32;
	if (sim)
		handle = sim->getScene().addSurfaceRigidAttachments(core, *sim, feature, indices, barycentrics, frame, coords, count);

	return handle;
}

void Sc::DeformableSurfaceCore::removeRigidAttachments(Sc::BodyCore* core, PxU32 handle)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
	{
		sim->getScene().removeSurfaceRigidAttachments(core, *sim, handle);
		setWakeCounterInternal(ScInternalWakeCounterResetValue);
	}
}

PxU32 Sc::DeformableSurfaceCore::addSurfaceAttachments(Sc::DeformableSurfaceCore* otherCore, PxDeformableAttachmentTargetType::Enum feature0,
	const PxU32* indices0, const PxVec4* barycentrics0, PxDeformableAttachmentTargetType::Enum feature1,
	const PxU32* indices1, const PxVec4* barycentrics1, PxU32 count)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	PxU32 handle = PX_INVALID_U32;
	if (sim)
		handle = sim->getScene().addSurfaceSurfaceAttachments(*otherCore, feature0, indices0, barycentrics0, *sim,
			feature1, indices1, barycentrics1, count);

	return handle;
}

void Sc::DeformableSurfaceCore::removeSurfaceAttachments(Sc::DeformableSurfaceCore* otherCore, PxU32 handle)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	setWakeCounterInternal(ScInternalWakeCounterResetValue);
	otherCore->setWakeCounterInternal(ScInternalWakeCounterResetValue);
	if (sim)
		sim->getScene().removeSurfaceSurfaceAttachments(*otherCore, *sim, handle);
}

void Sc::DeformableSurfaceCore::addRigidFilters(Sc::RigidCore* core, PxU32* triIndices, PxU32 triIndicesSize)
{
	Sc::DeformableSurfaceSim* sim = getSim();

	if (sim)
		sim->getScene().addSurfaceRigidFilters(core, *sim, triIndices, triIndicesSize);
}

void Sc::DeformableSurfaceCore::removeRigidFilters(Sc::RigidCore* core, PxU32* triIndices, PxU32 triIndicesSize)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().removeSurfaceRigidFilters(core, *sim, triIndices, triIndicesSize);
}

void Sc::DeformableSurfaceCore::addSurfaceFilters(Sc::DeformableSurfaceCore* otherCore, PxU32* otherTriIndices, PxU32* triIndices, PxU32 triIndicesSize)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().addSurfaceSurfaceFilters(*otherCore, otherTriIndices, *sim, triIndices, triIndicesSize);
}

void Sc::DeformableSurfaceCore::removeSurfaceFilters(Sc::DeformableSurfaceCore* otherCore, PxU32* otherTriIndices, PxU32* triIndices, PxU32 triIndicesSize)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->getScene().removeSurfaceSurfaceFilters(*otherCore, otherTriIndices, *sim, triIndices, triIndicesSize);
}

void Sc::DeformableSurfaceCore::addMaterial(const PxU16 handle)
{
	mCore.materialHandles.pushBack(handle);
	mCore.dirty = true;
}

void Sc::DeformableSurfaceCore::clearMaterials()
{
	mCore.materialHandles.clear();
	mCore.dirty = true;
}

PxActor* Sc::DeformableSurfaceCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<DeformableSurfaceCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

void Sc::DeformableSurfaceCore::attachShapeCore(ShapeCore* shapeCore)
{
	Sc::DeformableSurfaceSim* sim = getSim();
	if (sim)
		sim->attachShapeCore(shapeCore);
}

void Sc::DeformableSurfaceCore::onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags)
{
	PX_UNUSED(shape);
	DeformableSurfaceSim* sim = getSim();
	if (!sim)
		return;
	ShapeSimBase& s = sim->getShapeSim();

	if (notifyFlags & ShapeChangeNotifyFlag::eGEOMETRY)
		s.onVolumeOrTransformChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eRESET_FILTERING)
		s.onResetFiltering();
	if (notifyFlags & ShapeChangeNotifyFlag::eSHAPE2BODY)
		s.onVolumeOrTransformChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eFILTERDATA)
		s.onFilterDataChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eCONTACTOFFSET)
		s.onContactOffsetChange();
	if (notifyFlags & ShapeChangeNotifyFlag::eRESTOFFSET)
		s.onRestOffsetChange();
}

Sc::DeformableSurfaceSim* Sc::DeformableSurfaceCore::getSim() const
{
	return static_cast<Sc::DeformableSurfaceSim*>(ActorCore::getSim());
}

#endif //PX_SUPPORT_GPU_PHYSX

