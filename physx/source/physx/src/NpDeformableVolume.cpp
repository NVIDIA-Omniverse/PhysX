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

#include "DyDeformableVolumeCore.h"
#include "PxsHeapMemoryAllocator.h"
#include "PxsMemoryManager.h"
#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpDeformableVolume.h"
#include "NpPBDParticleSystem.h"
#include "NpCheck.h"
#include "NpScene.h"
#include "NpShape.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxTetrahedronMeshGeometry.h"
#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "GuTetrahedronMesh.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulationLink.h"
#include "ScDeformableVolumeSim.h"
#include "NpDeformableVolumeMaterial.h"

using namespace physx;

class PxCudaContextManager;

namespace
{
	// for deprecation
	static const PxU32 sNumTwinFlags = 3;
	static PxDeformableVolumeFlag::Enum sVFlags[sNumTwinFlags] = { PxDeformableVolumeFlag::eDISABLE_SELF_COLLISION, PxDeformableVolumeFlag::eENABLE_CCD, PxDeformableVolumeFlag::eKINEMATIC };
	static PxDeformableBodyFlag::Enum sBFlags[sNumTwinFlags] = { PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, PxDeformableBodyFlag::eKINEMATIC };

	void mirrorTwinFlags(PxDeformableBodyFlags& bodyFlags, const PxDeformableVolumeFlags volumeFlags)
	{
		for (PxU32 f = 0; f < sNumTwinFlags; ++f)
		{
			bool val = volumeFlags.isSet(sVFlags[f]);
			if (val)
			{
				bodyFlags.raise(sBFlags[f]);
			}
			else
			{
				bodyFlags.clear(sBFlags[f]);
			}
		}
	}
}

namespace physx
{

NpDeformableVolume::NpDeformableVolume(PxCudaContextManager& cudaContextManager) :
	NpActorTemplate	(PxConcreteType::eDEFORMABLE_VOLUME, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eDEFORMABLE_VOLUME),
	mShape			(NULL),
	mCudaContextManager(&cudaContextManager),
	mMemoryManager(NULL),
	mDeviceMemoryAllocator(NULL)
{
}

NpDeformableVolume::NpDeformableVolume(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager) :
	NpActorTemplate	(baseFlags), 
	mShape			(NULL),
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

	PX_SIMD_GUARD;

	PxBounds3 bounds = sim->getBounds();
	PX_ASSERT(bounds.isValid());

	// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
	const PxVec3 center = bounds.getCenter();
	const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
	return PxBounds3::centerExtents(center, inflatedExtents);
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
}

void NpDeformableVolume::setDeformableBodyFlags(PxDeformableBodyFlags flags)
{
	mCore.setBodyFlags(flags);
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
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getLinearDamping() const
{
	return mCore.getLinearDamping();
}

void NpDeformableVolume::setMaxVelocity(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setMaxVelocity() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setMaxVelocity(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getMaxVelocity() const
{
	return mCore.getMaxVelocity();
}

void NpDeformableVolume::setMaxDepenetrationVelocity(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setMaxDepenetrationVelocity() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setMaxDepenetrationVelocity(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableVolume::getMaxDepenetrationVelocity() const
{
	return mCore.getMaxDepenetrationVelocity();
}

void NpDeformableVolume::setSelfCollisionFilterDistance(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSelfCollisionFilterDistance() not allowed while simulation is running. "
		"Call will be ignored.");

	mCore.setSelfCollisionFilterDistance(v);
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

	PX_CHECK_AND_RETURN_NULL(npShape->getCore().getCore().mShapeCoreFlags & PxShapeCoreFlag::eDEFORMABLE_VOLUME_SHAPE,
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

	return true;
}

void NpDeformableVolume::detachShape()
{
	PX_CHECK_MSG(getNpSceneFromActor(*this) == NULL, 
		"Detaching a shape from a PxDeformableVolume is currenly only allowed as long as it is not part of a scene. "
		"Please remove the deformable volume from its scene first.");

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

	if (mShape)
		mShape->onActorDetach();
	mShape = NULL;
}

PxCudaContextManager* NpDeformableVolume::getCudaContextManager() const
{
	return mCudaContextManager;
}

// deprecated
void NpDeformableVolume::setParameter(const PxFEMParameters& parameters)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setParameter() not allowed while simulation is running. Call will be ignored.")

	mCore.setLinearDamping(parameters.velocityDamping);
	mCore.setSettlingThreshold(parameters.settlingThreshold);
	mCore.setSleepThreshold(parameters.sleepThreshold);
	mCore.setSettlingDamping(parameters.sleepDamping);
	mCore.setSelfCollisionFilterDistance(parameters.selfCollisionFilterDistance);
	mCore.setSelfCollisionStressTolerance(parameters.selfCollisionStressTolerance);
	UPDATE_PVD_PROPERTY
}

// deprecated
PxFEMParameters NpDeformableVolume::getParameter() const
{
	NP_READ_CHECK(getNpScene());
	PxFEMParameters parameters;
	parameters.velocityDamping = mCore.getLinearDamping();
	parameters.settlingThreshold = mCore.getSettlingThreshold();
	parameters.sleepThreshold = mCore.getSleepThreshold();
	parameters.sleepDamping = mCore.getSettlingDamping();
	parameters.selfCollisionFilterDistance = mCore.getSelfCollisionFilterDistance();
	parameters.selfCollisionStressTolerance = mCore.getSelfCollisionStressTolerance();
	return parameters;
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

	//deprecation functionality
	PxDeformableBodyFlags bodyFlags = mCore.getBodyFlags();
	mirrorTwinFlags(bodyFlags, flags);
	mCore.setBodyFlags(bodyFlags);
}

void NpDeformableVolume::setDeformableVolumeFlags(PxDeformableVolumeFlags flags)
{
	mCore.setVolumeFlags(flags);

	//deprecation functionality
	PxDeformableBodyFlags bodyFlags = mCore.getBodyFlags();
	mirrorTwinFlags(bodyFlags, flags);
	mCore.setBodyFlags(bodyFlags);
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

// deprecated
void NpDeformableVolume::setKinematicTargetBufferD(const PxVec4* positions, PxDeformableVolumeFlags flags)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(),
		"PxDeformableVolume::setKinematicTargetBufferD() not allowed while simulation is running. Call will be ignored.")
	
	PX_CHECK_AND_RETURN(!(positions == NULL && flags != PxDeformableVolumeFlags(0)),
		"PxDeformableVolume::setKinematicTargetBufferD: targets cannot be null if flags are set to be kinematic.");

	//toggling flags automatically seems SDK atypcal, hence the deprecation ...
	PxDeformableVolumeFlags volumeFlags = mCore.getVolumeFlags();
	if (positions != NULL)
	{
		if (flags.isSet(PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC))
		{
			volumeFlags.raise(PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC);
		}
		if (flags.isSet(PxDeformableVolumeFlag::eKINEMATIC))
		{
			volumeFlags.raise(PxDeformableVolumeFlag::eKINEMATIC);
		}
	}
	else
	{
		volumeFlags.clear(PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC);
		volumeFlags.clear(PxDeformableVolumeFlag::eKINEMATIC);
	}
	mCore.setVolumeFlags(volumeFlags);

	//deprecation functionality
	PxDeformableBodyFlags bodyFlags = mCore.getBodyFlags();
	mirrorTwinFlags(bodyFlags, volumeFlags);
	mCore.setBodyFlags(bodyFlags);

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

// deprecated
void NpDeformableVolume::addParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::addParticleFilter: Soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((particlesystem == NULL || particlesystem->getScene() != NULL), "PxDeformableVolume::addParticleFilter: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::addParticleFilter: Illegal to call while simulation is running.");

	NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
	Sc::ParticleSystemCore& core = npParticleSystem->getCore();
	mCore.addParticleFilter(&core, particleId, buffer ? buffer->getUniqueId() : 0, tetId);
}

// deprecated
void NpDeformableVolume::removeParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeParticleFilter: Soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((particlesystem == NULL || particlesystem->getScene() != NULL), "PxDeformableVolume::removeParticleFilter: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeParticleFilter: Illegal to call while simulation is running.");

	NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
	Sc::ParticleSystemCore& core = npParticleSystem->getCore();
	mCore.removeParticleFilter(&core, particleId, buffer ? buffer->getUniqueId() : 0, tetId);
}

// deprecated
PxU32 NpDeformableVolume::addParticleAttachment(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId, const PxVec4& barycentric)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "PxDeformableVolume::addParticleAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL((particlesystem == NULL || particlesystem->getScene() != NULL), "PxDeformableVolume::addParticleAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxDeformableVolume::addParticleAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

	NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
	Sc::ParticleSystemCore& core = npParticleSystem->getCore();
	return mCore.addParticleAttachment(&core, particleId, buffer ? buffer->getUniqueId() : 0, tetId, barycentric);
}

// deprecated
void NpDeformableVolume::removeParticleAttachment(PxPBDParticleSystem* particlesystem, PxU32 handle)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::addParticleAttachment: Soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((particlesystem == NULL || particlesystem->getScene() != NULL), "PxDeformableVolume::addParticleAttachment: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::addParticleAttachment: Illegal to call while simulation is running.");

	NpPBDParticleSystem* npParticleSystem = static_cast<NpPBDParticleSystem*>(particlesystem);
	Sc::ParticleSystemCore& core = npParticleSystem->getCore();
	mCore.removeParticleAttachment(&core, handle);
}

// deprecated
void NpDeformableVolume::addRigidFilter(PxRigidActor* actor, PxU32 vertId)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::addRigidFilter: Soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::addRigidFilter: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::addRigidFilter: Illegal to call while simulation is running.");

	Sc::BodyCore* core = getBodyCore(actor);
	mCore.addRigidFilter(core, vertId);
}

// deprecated
void NpDeformableVolume::removeRigidFilter(PxRigidActor* actor, PxU32 vertId)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeRigidFilter: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::removeRigidFilter: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeRigidFilter: Illegal to call while simulation is running.");

	Sc::BodyCore* core = getBodyCore(actor);
	mCore.removeRigidFilter(core, vertId);
}

// deprecated
PxU32 NpDeformableVolume::addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "PxDeformableVolume::addRigidAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::addRigidAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL(constraint == NULL || constraint->isValid(), "PxDeformableVolume::addRigidAttachment: PxConeLimitedConstraint needs to be valid if specified.", 0xFFFFFFFF);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxDeformableVolume::addRigidAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

	Sc::BodyCore* core = getBodyCore(actor);

	PxVec3 aPose = actorSpacePose;
	if (actor && actor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
	{
		NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor);
		aPose = stat->getGlobalPose().transform(aPose);
	}

	return mCore.addRigidAttachment(core, vertId, aPose, constraint, true);
}

// deprecated
void NpDeformableVolume::removeRigidAttachment(PxRigidActor* actor, PxU32 handle)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeRigidAttachment: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::removeRigidAttachment: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeRigidAttachment: Illegal to call while simulation is running.");

	Sc::BodyCore* core = getBodyCore(actor);
	mCore.removeRigidAttachment(core, handle);
}

// deprecated
void NpDeformableVolume::addTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::addTetRigidFilter: Soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::addTetRigidFilter: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::addTetRigidFilter: Illegal to call while simulation is running.");

	Sc::BodyCore* core = getBodyCore(actor);
	return mCore.addTetRigidFilter(core, tetIdx);
}

// deprecated
void NpDeformableVolume::removeTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeTetRigidFilter: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::removeTetRigidFilter: actor must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeTetRigidFilter: Illegal to call while simulation is running.");

	Sc::BodyCore* core = getBodyCore(actor);
	mCore.removeTetRigidFilter(core, tetIdx);
}

// deprecated
PxU32 NpDeformableVolume::addTetRigidAttachment(PxRigidActor* actor, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "PxDeformableVolume::addTetRigidAttachment: Soft body must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL((actor == NULL || actor->getScene() != NULL), "PxDeformableVolume::addTetRigidAttachment: actor must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL(constraint == NULL || constraint->isValid(), "PxDeformableVolume::addTetRigidAttachment: PxConeLimitedConstraint needs to be valid if specified.", 0xFFFFFFFF);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxDeformableVolume::addTetRigidAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

	Sc::BodyCore* core = getBodyCore(actor);

	PxVec3 aPose = actorSpacePose;
	if (actor && actor->getConcreteType()==PxConcreteType::eRIGID_STATIC)
	{
		NpRigidStatic* stat = static_cast<NpRigidStatic*>(actor);
		aPose = stat->getGlobalPose().transform(aPose);
	}

	return mCore.addTetRigidAttachment(core, tetIdx, barycentric, aPose, constraint, true);
}

// deprecated
void NpDeformableVolume::addSoftBodyFilter(PxDeformableVolume* softbody0, PxU32 tetIdx0, PxU32 tetIdx1)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(softbody0 != NULL, "PxDeformableVolume::addSoftBodyFilter: soft body must not be null");
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::addSoftBodyFilter: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "PxDeformableVolume::addSoftBodyFilter: soft body must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::addSoftBodyFilter: Illegal to call while simulation is running.");

	NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(softbody0);
	Sc::DeformableVolumeCore* core = &dyn->getCore();

	mCore.addSoftBodyFilter(*core, tetIdx0, tetIdx1);
}

// deprecated
void NpDeformableVolume::removeSoftBodyFilter(PxDeformableVolume* softbody0, PxU32 tetIdx0, PxU32 tetIdx1)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(softbody0 != NULL, "PxDeformableVolume::removeSoftBodyFilter: soft body must not be null");
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeSoftBodyFilter: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "PxDeformableVolume::removeSoftBodyFilter: soft body must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeSoftBodyFilter: Illegal to call while simulation is running.");

	NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(softbody0);
	Sc::DeformableVolumeCore* core = &dyn->getCore();

	mCore.removeSoftBodyFilter(*core, tetIdx0, tetIdx1);
}

// deprecated
void NpDeformableVolume::addSoftBodyFilters(PxDeformableVolume* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(softbody0 != NULL, "PxDeformableVolume::addSoftBodyFilter: soft body must not be null");
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::addSoftBodyFilter: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "PxDeformableVolume::addSoftBodyFilter: soft body must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::addSoftBodyFilter: Illegal to call while simulation is running.");

	NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(softbody0);
	Sc::DeformableVolumeCore* core = &dyn->getCore();

	mCore.addSoftBodyFilters(*core, tetIndices0, tetIndices1, tetIndicesSize);
}

// deprecated
void NpDeformableVolume::removeSoftBodyFilters(PxDeformableVolume* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(softbody0 != NULL, "PxDeformableVolume::removeSoftBodyFilter: soft body must not be null");
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeSoftBodyFilter: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "PxDeformableVolume::removeSoftBodyFilter: soft body must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeSoftBodyFilter: Illegal to call while simulation is running.");

	NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(softbody0);
	Sc::DeformableVolumeCore* core = &dyn->getCore();

	mCore.removeSoftBodyFilters(*core, tetIndices0, tetIndices1, tetIndicesSize);
}

// deprecated
PxU32 NpDeformableVolume::addSoftBodyAttachment(PxDeformableVolume* softbody0, PxU32 tetIdx0, const PxVec4& tetBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
										PxConeLimitedConstraint* constraint, PxReal constraintOffset)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN_VAL(softbody0 != NULL, "PxDeformableVolume::addSoftBodyAttachment: soft body must not be null", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL(getNpScene() != NULL, "PxDeformableVolume::addSoftBodyAttachment: soft body must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL(softbody0->getScene() != NULL, "PxDeformableVolume::addSoftBodyAttachment: soft body must be inserted into the scene.", 0xFFFFFFFF);
	PX_CHECK_AND_RETURN_VAL(constraint == NULL || constraint->isValid(), "PxDeformableVolume::addSoftBodyAttachment: PxConeLimitedConstraint needs to be valid if specified.", 0xFFFFFFFF);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(getNpScene(), "PxDeformableVolume::addSoftBodyAttachment: Illegal to call while simulation is running.", 0xFFFFFFFF);

	NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(softbody0);
	Sc::DeformableVolumeCore* core = &dyn->getCore();

	return mCore.addSoftBodyAttachment(*core, tetIdx0, tetBarycentric0, tetIdx1, tetBarycentric1, constraint, constraintOffset, true);
}

// deprecated
void NpDeformableVolume::removeSoftBodyAttachment(PxDeformableVolume* softbody0, PxU32 handle)
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(softbody0 != NULL, "PxDeformableVolume::removeSoftBodyAttachment: soft body must not be null");
	PX_CHECK_AND_RETURN(getNpScene() != NULL, "PxDeformableVolume::removeSoftBodyAttachment: soft body must be inserted into the scene.");
	PX_CHECK_AND_RETURN(softbody0->getScene() != NULL, "PxDeformableVolume::removeSoftBodyAttachment: soft body must be inserted into the scene.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "PxDeformableVolume::removeSoftBodyAttachment: Illegal to call while simulation is running.");

	NpDeformableVolume* dyn = static_cast<NpDeformableVolume*>(softbody0);
	Sc::DeformableVolumeCore* core = &dyn->getCore();

	mCore.removeSoftBodyAttachment(*core, handle);
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
