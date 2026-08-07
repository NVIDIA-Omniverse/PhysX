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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "DyDeformableVolumeCore.h"
#include "PxsHeapMemoryAllocator.h"
#include "PxsMemoryManager.h"
#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpDeformableVolume.h"
#include "NpCheck.h"
#include "NpScene.h"
#include "NpShape.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxTetrahedronMeshGeometry.h"
#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "GuTetrahedronMesh.h"
#include "ScDeformableVolumeSim.h"
#include "NpDeformableVolumeMaterial.h"
#include "omnipvd/NpOmniPvdSetData.h"

using namespace physx;

class PxCudaContextManager;

namespace physx
{

NpDeformableVolume::NpDeformableVolume(PxCudaContextManager& cudaContextManager) :
	NpActorTemplate	(PxConcreteType::eDEFORMABLE_VOLUME, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eDEFORMABLE_VOLUME),
	mShape			(NULL),
	mSimulationMesh(NULL),
	mAuxData		(NULL),
	mCudaContextManager(&cudaContextManager),
	mMemoryManager(NULL),
	mDeviceMemoryAllocator(NULL)
{
}

NpDeformableVolume::NpDeformableVolume(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager) :
	NpActorTemplate	(baseFlags), 
	mShape			(NULL),
	mSimulationMesh(NULL),
	mAuxData		(NULL),
	mCudaContextManager(&cudaContextManager),
	mMemoryManager(NULL),
	mDeviceMemoryAllocator(NULL)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxActor API
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableVolume::release()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	// AD why is this commented out?
	//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

	if (npScene)
	{
		removeAttachments(*this, true);
		removeElementFilters(*this, true);

		npScene->scRemoveDeformableVolume(*this);
		npScene->removeFromDeformableVolumeList(*this);
	}

	detachSimulationMesh();
	detachShape();

	PX_ASSERT(!isAPIWriteForbidden());
	releaseAllocator();
	NpDestroyDeformableVolume(this);
}

PxBounds3 NpDeformableVolume::getWorldBounds(float inflation) const
{
	NP_READ_CHECK(getNpScene());

	if (!getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Querying bounds of a PxDeformableBody which is not part of a PxScene is not supported.");
		return PxBounds3::empty();
	}

	const Sc::DeformableVolumeSim* sim = mCore.getSim();
	PX_ASSERT(sim);

	PX_SIMD_GUARD

	NP_CHECK_SCENE_CORRUPTION_AND_RETURN_VAL(getNpScene(), PxBounds3::empty())

	const PxU32 boundsIndex = sim->getShapeSim().getElementID();
	const Bp::BoundsArray& boundsArray = getNpScene()->getScScene().getBoundsArray();
	const PxBounds3 bounds = boundsArray.getBounds(boundsIndex);
	PX_ASSERT(bounds.isValid());

	// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
	const PxVec3 center = bounds.getCenter();
	const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
	return PxBounds3::centerExtents(center, inflatedExtents);
}

void NpDeformableVolume::setActorFlag(PxActorFlag::Enum flag, bool val)
{
	PX_CHECK_AND_RETURN(flag == PxActorFlag::eDISABLE_GRAVITY || !val, "PxDeformableBody only supports PxActorFlag::eDISABLE_GRAVITY!");

	PxActorFlags flags = mCore.getActorFlags();
	if(val)
		flags.raise(flag);
	else
		flags.clear(flag);

	mCore.setActorFlags(flags);
	NpActorTemplate<PxDeformableVolume>::setActorFlag(flag, val);
}

void NpDeformableVolume::setActorFlags(PxActorFlags inFlags)
{
	PX_CHECK_AND_RETURN(inFlags & PxActorFlag::eVISUALIZATION, "PxDeformableBody doesn't supports PxActorFlag::eVISUALIZATION!");
	PX_CHECK_AND_RETURN(inFlags & PxActorFlag::eSEND_SLEEP_NOTIFIES, "PxDeformableBody doesn't supports PxActorFlag::eSEND_SLEEP_NOTIFIES!");
	PX_CHECK_AND_RETURN(inFlags & PxActorFlag::eDISABLE_SIMULATION, "PxDeformableBody doesn't supports PxActorFlag::eDISABLE_SIMULATION!");

	mCore.setActorFlags(inFlags);
	NpActorTemplate<PxDeformableVolume>::setActorFlags(inFlags);
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableBody API
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableVolume::setDeformableBodyFlag(PxDeformableBodyFlag::Enum flag, bool val)
{
	PxDeformableBodyFlags flags = mCore.getBodyFlags();
	if (val)
		flags.raise(flag);
	else
		flags.clear(flag);

	mCore.setBodyFlags(flags);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, deformableBodyFlags, static_cast<PxDeformableBody&>(*this), flags);
}

void NpDeformableVolume::setDeformableBodyFlags(PxDeformableBodyFlags flags)
{
	mCore.setBodyFlags(flags);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, deformableBodyFlags, static_cast<PxDeformableBody&>(*this), flags);
}

PxDeformableBodyFlags NpDeformableVolume::getDeformableBodyFlags() const
{
	return mCore.getBodyFlags();
}

void NpDeformableVolume::setLinearDamping(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setLinearDamping() not allowed while simulation is running. Call will be ignored.")

	mCore.setLinearDamping(v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, linearDamping, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getLinearDamping() const
{
	return mCore.getLinearDamping();
}

void NpDeformableVolume::setMaxLinearVelocity(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setMaxLinearVelocity() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setMaxLinearVelocity(v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, maxLinearVelocity, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getMaxLinearVelocity() const
{
	return mCore.getMaxLinearVelocity();
}

void NpDeformableVolume::setMaxDepenetrationVelocity(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(v > 0.0f, "PxDeformableBody::setMaxDepenetrationVelocity(): value must be greater than zero.");
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setMaxDepenetrationVelocity() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setMaxPenetrationBias(-v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, maxDepenetrationVelocity, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getMaxDepenetrationVelocity() const
{
	return -mCore.getMaxPenetrationBias();
}

void NpDeformableVolume::setSelfCollisionFilterDistance(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSelfCollisionFilterDistance() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setSelfCollisionFilterDistance(v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, selfCollisionFilterDistance, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getSelfCollisionFilterDistance() const
{
	return mCore.getSelfCollisionFilterDistance();
}

void NpDeformableVolume::setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(minPositionIters > 0, "PxDeformableBody::setSolverIterationCounts: minPositionIters must be more than zero!");
	PX_CHECK_AND_RETURN(minPositionIters <= 255, "PxDeformableBody::setSolverIterationCounts: minPositionIters must be no greater than 255!");
	PX_CHECK_AND_RETURN(minVelocityIters <= 255, "PxDeformableBody::setSolverIterationCounts: minVelocityIters must be no greater than 255!");
	PX_WARN_ONCE_IF(minVelocityIters > 1, "PxDeformableBody::setSolverIterationCounts: minVelocityIters are ignored!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.");

	mCore.setSolverIterationCounts((minVelocityIters & 0xff) << 8 | (minPositionIters & 0xff));
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, solverIterationCount, static_cast<PxDeformableBody&>(*this), minPositionIters);
	UPDATE_PVD_PROPERTY
}

void NpDeformableVolume::getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const
{
	NP_READ_CHECK(getNpScene());

	PxU16 x = mCore.getSolverIterationCounts();
	minVelocityIters = PxU32(x >> 8);
	minPositionIters = PxU32(x & 0xff);
}

void NpDeformableVolume::setSleepThreshold(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSleepThreshold() not allowed while simulation is running. Call will be ignored.")

	mCore.setSleepThreshold(v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, sleepThreshold, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getSleepThreshold() const
{
	return mCore.getSleepThreshold();
}

void NpDeformableVolume::setSettlingThreshold(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSettlingThreshold() not allowed while simulation is running. Call will be ignored.")

	mCore.setSettlingThreshold(v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, settlingThreshold, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getSettlingThreshold() const
{
	return mCore.getSettlingThreshold();
}

void NpDeformableVolume::setSettlingDamping(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSettlingDamping() not allowed while simulation is running. Call will be ignored.")

	mCore.setSettlingDamping(v);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, settlingDamping, static_cast<PxDeformableBody&>(*this), v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getSettlingDamping() const
{
	return mCore.getSettlingDamping();
}

void NpDeformableVolume::setWakeCounter(PxReal wakeCounterValue)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setWakeCounter() not allowed while simulation is running. Call will be ignored.")

	mCore.setWakeCounter(wakeCounterValue);
	//UPDATE_PVD_PROPERTIES_OBJECT()
}

PxReal NpDeformableVolume::getWakeCounter() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getWakeCounter();
}

bool NpDeformableVolume::isSleeping() const
{
	NpScene* npScene = getNpScene();
	// The rest of the function is incorrect if sleeping is disabled, return false immediately
	if (npScene && (npScene->getFlags() & PxSceneFlag::eDISABLE_SLEEPING))
		return false;

	Sc::DeformableVolumeSim* sim = mCore.getSim();
	if (sim)
	{
		return sim->isSleeping();
	}
	return true;
}

PxShape* NpDeformableVolume::getShape()
{
	return mShape;
}

bool NpDeformableVolume::attachShape(PxShape& shape)
{
	NpShape* npShape = static_cast<NpShape*>(&shape);

	PX_CHECK_AND_RETURN_NULL(npShape->getGeometryTypeFast() == PxGeometryType::eTETRAHEDRONMESH,
		"PxDeformableVolume::attachShape: Geometry type must be tetrahedron mesh geometry");
	PX_CHECK_AND_RETURN_NULL(mShape == NULL,
		"PxDeformableVolume::attachShape: deformable volume can just have one shape");
	PX_CHECK_AND_RETURN_NULL(shape.isExclusive(),
		"PxDeformableVolume::attachShape: shape must be exclusive");

	const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(npShape->getGeometry());
	PX_CHECK_AND_RETURN_NULL(tetGeom.tetrahedronMesh != NULL,
		"PxDeformableVolume::attachShape: PxTetrahedronMeshGeometry::tetrahedronMesh is NULL");

	Gu::BVTetrahedronMesh* tetMesh = static_cast<Gu::BVTetrahedronMesh*>(tetGeom.tetrahedronMesh);
	PX_CHECK_AND_RETURN_NULL(tetMesh->getNbTetrahedronsFast() <= PX_MAX_NB_DEFORMABLE_VOLUME_TET,
		"PxDeformableVolume::attachShape: collision mesh consists of too many tetrahedrons, see PX_MAX_NB_DEFORMABLE_VOLUME_TET");

	PX_CHECK_AND_RETURN_NULL(npShape->getCore().mShapeCoreFlags & PxShapeCoreFlag::eDEFORMABLE_VOLUME_SHAPE,
		"PxDeformableVolume::attachShape: shape must be a deformable volume shape!");

	Dy::DeformableVolumeCore& core = mCore.getCore();
	PX_CHECK_AND_RETURN_NULL(core.positionInvMass == NULL,
		"PxDeformableVolume::attachShape: positionInvMass already exists, overwrite not allowed, call detachShape first");

	mShape = npShape;

	PX_ASSERT(shape.getActor() == NULL);
	npShape->onActorAttach(*this);

	createAllocator();
	const PxU32 numVerts = tetMesh->getNbVerticesFast();
	core.positionInvMass = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVerts * sizeof(PxVec4), PxsHeapStats::eSHARED_SOFTBODY, PX_FL));
	core.restPosition = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVerts * sizeof(PxVec4), PxsHeapStats::eSHARED_SOFTBODY, PX_FL));

	updateMaterials();

	// Stream collision shape to OmniPVD
	OMNI_PVD_ADD(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, shapes, static_cast<PxDeformableBody&>(*this), shape);

	return true;
}

void NpDeformableVolume::detachShape()
{
	PX_CHECK_MSG(getNpSceneFromActor(*this) == NULL,
		"Detaching a shape from a PxDeformableVolume is currenly only allowed as long as it is not part of a scene. "
		"Please remove the deformable volume from its scene first.");

	if (!mShape)
		return;

	PX_ASSERT(mDeviceMemoryAllocator);

	Dy::DeformableVolumeCore& core = mCore.getCore();
	if (core.restPosition)
	{
		mDeviceMemoryAllocator->deallocate(core.restPosition);
		core.restPosition = NULL;
	}
	if (core.positionInvMass)
	{
		mDeviceMemoryAllocator->deallocate(core.positionInvMass);
		core.positionInvMass = NULL;
	}

	OMNI_PVD_REMOVE(OMNI_PVD_CONTEXT_HANDLE, PxDeformableBody, shapes, static_cast<PxDeformableBody&>(*this), *mShape);
	mShape->onActorDetach();
	mShape = NULL;
}

PxCudaContextManager* NpDeformableVolume::getCudaContextManager() const
{
	return mCudaContextManager;
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableVolume API
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableVolume::setDeformableVolumeFlag(PxDeformableVolumeFlag::Enum flag, bool val)
{
	PxDeformableVolumeFlags flags = mCore.getVolumeFlags();
	if (val)
		flags.raise(flag);
	else
		flags.clear(flag);

	mCore.setVolumeFlags(flags);
}

void NpDeformableVolume::setDeformableVolumeFlags(PxDeformableVolumeFlags flags)
{
	mCore.setVolumeFlags(flags);
}

PxDeformableVolumeFlags NpDeformableVolume::getDeformableVolumeFlags() const
{
	return mCore.getVolumeFlags();
}

void NpDeformableVolume::setSelfCollisionStressTolerance(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSelfCollisionStressTolerance() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setSelfCollisionStressTolerance(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getSelfCollisionStressTolerance() const
{
	return mCore.getSelfCollisionStressTolerance();
}

PxVec4* NpDeformableVolume::getPositionInvMassBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mShape != NULL, "PxDeformableVolume::getPositionInvMassBufferD: Softbody does not have a shape, attach shape first.");

	Dy::DeformableVolumeCore& core = mCore.getCore();
	return core.positionInvMass;
}

PxVec4* NpDeformableVolume::getRestPositionBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mShape != NULL, "PxDeformableVolume::getRestPositionBufferD: Softbody does not have a shape, attach shape first.");

	Dy::DeformableVolumeCore& core = mCore.getCore();
	return core.restPosition;
}

PxVec4* NpDeformableVolume::getSimPositionInvMassBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mSimulationMesh != NULL, "PxDeformableVolume::getSimPositionInvMassBufferD: Softbody does not have a simulation mesh, attach simulation mesh first.");

	Dy::DeformableVolumeCore& core = mCore.getCore();
	return core.simPositionInvMass;
}

PxVec4* NpDeformableVolume::getSimVelocityBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mSimulationMesh != NULL, "PxDeformableVolume::getSimVelocityBufferD: Softbody does not have a simulation mesh, attach simulation mesh first.");

	Dy::DeformableVolumeCore& core = mCore.getCore();
	return core.simVelocity;
}

void NpDeformableVolume::markDirty(PxDeformableVolumeDataFlags flags)
{
	NP_WRITE_CHECK(getNpScene());

	Dy::DeformableVolumeCore& core = mCore.getCore();
	core.dirtyFlags |= flags;
}

void NpDeformableVolume::setKinematicTargetBufferD(const PxVec4* positions)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(),
		"PxDeformableVolume::setKinematicTargetBufferD() not allowed while simulation is running. Call will be ignored.")

	mCore.setKinematicTargets(positions);
}

bool NpDeformableVolume::attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxDeformableVolumeAuxData& deformableVolumeAuxData)
{
	Dy::DeformableVolumeCore& core = mCore.getCore();

	PX_CHECK_AND_RETURN_NULL(core.simPositionInvMass == NULL, "PxDeformableVolume::attachSimulationMesh: simPositionInvMass already exists, overwrite not allowed, call detachSimulationMesh first");
	PX_CHECK_AND_RETURN_NULL(core.simVelocity == NULL, "PxDeformableVolume::attachSimulationMesh: simVelocity already exists, overwrite not allowed, call detachSimulationMesh first");
	Gu::TetrahedronMesh& tetMesh = static_cast<Gu::TetrahedronMesh&>(simulationMesh);
	PX_CHECK_AND_RETURN_NULL(tetMesh.getNbTetrahedronsFast() <= PX_MAX_NB_DEFORMABLE_VOLUME_TET, "PxDeformableVolume::attachSimulationMesh: simulation mesh contains too many tetrahedrons, see PX_MAX_NB_DEFORMABLE_VOLUME_TET");

	mSimulationMesh = &tetMesh;
	mAuxData = static_cast<Gu::DeformableVolumeAuxData*>(&deformableVolumeAuxData);
	const PxU32 numVertsGM = tetMesh.getNbVerticesFast();

	createAllocator();

	core.simPositionInvMass = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVertsGM * sizeof(PxVec4), PxsHeapStats::eSHARED_SOFTBODY, PX_FL));
	core.simVelocity = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVertsGM * sizeof(PxVec4), PxsHeapStats::eSHARED_SOFTBODY, PX_FL));

	return true;
}

void NpDeformableVolume::detachSimulationMesh()
{
	if (!mSimulationMesh)
		return;

	PX_ASSERT(mDeviceMemoryAllocator);

	Dy::DeformableVolumeCore& core = mCore.getCore();
	if (core.simPositionInvMass)
	{
		mDeviceMemoryAllocator->deallocate(core.simPositionInvMass);
		core.simPositionInvMass = NULL;
	}

	if (core.simVelocity)
	{
		mDeviceMemoryAllocator->deallocate(core.simVelocity);
		core.simVelocity = NULL;
	}

	mSimulationMesh = NULL;
	mAuxData = NULL;
}

PxTetrahedronMesh* NpDeformableVolume::getCollisionMesh()
{
	const PxTetrahedronMeshGeometry& tetMeshGeom = static_cast<const PxTetrahedronMeshGeometry&>(mShape->getGeometry());
	return tetMeshGeom.tetrahedronMesh;
}

const PxTetrahedronMesh* NpDeformableVolume::getCollisionMesh() const
{
	const PxTetrahedronMeshGeometry& tetMeshGeom = static_cast<const PxTetrahedronMeshGeometry&>(mShape->getGeometry());
	return tetMeshGeom.tetrahedronMesh;
}

PxU32 NpDeformableVolume::getGpuDeformableVolumeIndex()
{
	NP_READ_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene(), "PxDeformableVolume::getGpuDeformableVolumeIndex: Soft body must be in a scene.", 0xffffffff);

	return mCore.getGpuIndex();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Internal
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableVolume::updateMaterials()
{
	mCore.clearMaterials();
	for (PxU32 i = 0; i < mShape->getNbMaterials(); ++i)
	{
		PxDeformableVolumeMaterial* material;
		mShape->getDeformableVolumeMaterials(&material, 1, i);
		mCore.addMaterial(static_cast<NpDeformableVolumeMaterial*>(material)->mMaterial.mMaterialIndex);
	}
}

void NpDeformableVolume::createAllocator()
{
	if (!mMemoryManager)
	{
		PxPhysXGpu* physXGpu = PxvGetPhysXGpu(true);
		PX_ASSERT(physXGpu != NULL);

		mMemoryManager = physXGpu->createGpuMemoryManager(mCudaContextManager);
		mDeviceMemoryAllocator = mMemoryManager->getDeviceMemoryAllocator();
	}
	PX_ASSERT(mMemoryManager != NULL);
	PX_ASSERT(mDeviceMemoryAllocator != NULL);
}

void NpDeformableVolume::releaseAllocator()
{
	// deallocate device memory if not released already.
	Dy::DeformableVolumeCore& core = mCore.getCore();
	if (core.simVelocity)
	{
		mDeviceMemoryAllocator->deallocate(core.simVelocity);
		core.simVelocity = NULL;
	}
	if (core.simPositionInvMass)
	{
		mDeviceMemoryAllocator->deallocate(core.simPositionInvMass);
		core.simPositionInvMass = NULL;
	}
	if (core.restPosition)
	{
		mDeviceMemoryAllocator->deallocate(core.restPosition);
		core.restPosition = NULL;
	}
	if (core.positionInvMass)
	{
		mDeviceMemoryAllocator->deallocate(core.positionInvMass);
		core.positionInvMass = NULL;
	}

	if (mMemoryManager != NULL)
	{
		mDeviceMemoryAllocator = NULL; // released by memory manager
		PX_DELETE(mMemoryManager);
	}
}

Sc::DeformableVolumeCore* getDeformableVolumeCore(PxActor* actor)
{
	if (actor->getConcreteType() == PxConcreteType::eDEFORMABLE_VOLUME)
	{
		NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(actor);
		return &dyn->getCore();
	}
	return NULL;
}

} // namespace physx

#endif //PX_SUPPORT_GPU_PHYSX
