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

#include "PxsHeapMemoryAllocator.h"
#include "PxsMemoryManager.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpDeformableSurface.h"
#include "NpCheck.h"
#include "NpScene.h"
#include "NpShape.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "GuTriangleMesh.h"
#include "ScDeformableSurfaceSim.h"


using namespace physx;

class PxCudaContextManager;

namespace physx
{

NpDeformableSurface::NpDeformableSurface(PxCudaContextManager& cudaContextManager)
	:
	NpActorTemplate(PxConcreteType::eDEFORMABLE_SURFACE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, NpType::eDEFORMABLE_SURFACE),
	mShape(NULL),
	mCudaContextManager(&cudaContextManager),
	mMemoryManager(NULL),
	mDeviceMemoryAllocator(NULL)
{
}

NpDeformableSurface::NpDeformableSurface(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager) :
	NpActorTemplate(baseFlags),
	mShape(NULL),
	mCudaContextManager(&cudaContextManager),
	mMemoryManager(NULL),
	mDeviceMemoryAllocator(NULL)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxActor API
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableSurface::release()
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	if (npScene)
	{
		removeAttachments(*this, true);
		removeElementFilters(*this, true);

		npScene->scRemoveDeformableSurface(*this);
		npScene->removeFromDeformableSurfaceList(*this);
	}

	detachShape();

	PX_ASSERT(!isAPIWriteForbidden());
	releaseAllocator();
	NpDestroyDeformableSurface(this);
}

PxBounds3 NpDeformableSurface::getWorldBounds(float inflation) const
{
	NP_READ_CHECK(getNpScene());

	if (!getNpScene())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Querying bounds of a PxDeformableBody which is not part of a PxScene is not supported.");
		return PxBounds3::empty();
	}

	const Sc::DeformableSurfaceSim* sim = mCore.getSim();
	PX_ASSERT(sim);

	PX_SIMD_GUARD;

	const PxBounds3 bounds = sim->getBounds();
	PX_ASSERT(bounds.isValid());

	// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
	const PxVec3 center = bounds.getCenter();
	const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
	return PxBounds3::centerExtents(center, inflatedExtents);
}

/////////////////////////////////////////////////////////////////////////////////////////
// PxDeformableBody API
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableSurface::setDeformableBodyFlag(PxDeformableBodyFlag::Enum flag, bool val)
{
	PxDeformableBodyFlags flags = mCore.getBodyFlags();
	if (val)
		flags.raise(flag);
	else
		flags.clear(flag);

	mCore.setBodyFlags(flags);
}

void NpDeformableSurface::setDeformableBodyFlags(PxDeformableBodyFlags flags)
{
	mCore.setBodyFlags(flags);
}

PxDeformableBodyFlags NpDeformableSurface::getDeformableBodyFlags() const
{
	return mCore.getBodyFlags();
}

void NpDeformableSurface::setLinearDamping(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setLinearDamping() not allowed while simulation is running. Call will be ignored.")

	mCore.setLinearDamping(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getLinearDamping() const
{
	return mCore.getLinearDamping();
}

void NpDeformableSurface::setMaxVelocity(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setMaxVelocity() not allowed while simulation is running. Call will be ignored.")

	mCore.setMaxVelocity(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getMaxVelocity() const
{
	return mCore.getMaxVelocity();
}

void NpDeformableSurface::setMaxDepenetrationVelocity(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setMaxDepenetrationVelocity() not allowed while simulation is running. Call will be ignored.")

	mCore.setMaxDepenetrationVelocity(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getMaxDepenetrationVelocity() const
{
	return mCore.getMaxDepenetrationVelocity();
}

void NpDeformableSurface::setSelfCollisionFilterDistance(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSelfCollisionFilterDistance() not allowed while simulation is running. Call will be ignored.")

	mCore.setSelfCollisionFilterDistance(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getSelfCollisionFilterDistance() const
{
	return mCore.getSelfCollisionFilterDistance();
}

void NpDeformableSurface::setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(minPositionIters > 0, "PxDeformableBody::setSolverIterationCounts: minPositionIters must be more than zero!");
	PX_CHECK_AND_RETURN(minPositionIters <= 255, "PxDeformableBody::setSolverIterationCounts: minPositionIters must be no greater than 255!");
	PX_CHECK_AND_RETURN(minVelocityIters <= 255, "PxDeformableBody::setSolverIterationCounts: minVelocityIters must be no greater than 255!");
	PX_WARN_ONCE_IF(minVelocityIters > 1, "PxDeformableBody::setSolverIterationCounts: minVelocityIters are ignored!");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSolverIterationCounts() not allowed while simulation is running. Call will be ignored.")

	mCore.setSolverIterationCounts((minVelocityIters & 0xff) << 8 | (minPositionIters & 0xff));
	UPDATE_PVD_PROPERTY
}

void NpDeformableSurface::getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const
{
	NP_READ_CHECK(getNpScene());

	PxU16 x = mCore.getSolverIterationCounts();
	minVelocityIters = PxU32(x >> 8);
	minPositionIters = PxU32(x & 0xff);
}

void NpDeformableSurface::setSleepThreshold(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSleepThreshold() not allowed while simulation is running. Call will be ignored.")

	mCore.setSleepThreshold(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getSleepThreshold() const
{
	return mCore.getSleepThreshold();
}

void NpDeformableSurface::setSettlingThreshold(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSettlingThreshold() not allowed while simulation is running. Call will be ignored.")

	mCore.setSettlingThreshold(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getSettlingThreshold() const
{
	return mCore.getSettlingThreshold();
}

void NpDeformableSurface::setSettlingDamping(const PxReal v)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setSettlingDamping() not allowed while simulation is running. Call will be ignored.")

	mCore.setSettlingDamping(v);
	UPDATE_PVD_PROPERTY
}

PxReal NpDeformableSurface::getSettlingDamping() const
{
	return mCore.getSettlingDamping();
}

void NpDeformableSurface::setWakeCounter(PxReal wakeCounterValue)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableBody::setWakeCounter() not allowed while simulation is running. Call will be ignored.")

	mCore.setWakeCounter(wakeCounterValue);
	//UPDATE_PVD_PROPERTIES_OBJECT()
}

PxReal NpDeformableSurface::getWakeCounter() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getWakeCounter();
}

bool NpDeformableSurface::isSleeping() const
{
	Sc::DeformableSurfaceSim* sim = mCore.getSim();
	if (sim)
	{
		return sim->isSleeping();
	}
	return true;
}

PxShape* NpDeformableSurface::getShape()
{
	return mShape;
}

bool NpDeformableSurface::attachShape(PxShape& shape)
{
	NpShape* npShape = static_cast<NpShape*>(&shape);

	PX_CHECK_AND_RETURN_NULL(npShape->getGeometryTypeFast() == PxGeometryType::eTRIANGLEMESH,
		"PxDeformableSurface::attachShape: Geometry type must be triangle mesh geometry");
	PX_CHECK_AND_RETURN_NULL(mShape == NULL,
		"PxDeformableSurface::attachShape: deformable surface can just have one shape");
	PX_CHECK_AND_RETURN_NULL(shape.isExclusive(),
		"PxDeformableSurface::attachShape: shape must be exclusive");

	const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(npShape->getGeometry());
	PX_CHECK_AND_RETURN_NULL(triGeom.triangleMesh != NULL,
		"PxDeformableSurface::attachShape: PxTriangleMeshGeometry::triangleMesh is NULL");

	Gu::TriangleMesh* triMesh = static_cast<Gu::TriangleMesh*>(triGeom.triangleMesh);
	PX_CHECK_AND_RETURN_NULL(triMesh->getNbTrianglesFast() <= PX_MAX_NB_DEFORMABLE_SURFACE_TRI,
		"PxDeformableSurface::attachShape: triangle mesh consists of too many triangles, see PX_MAX_NB_DEFORMABLE_SURFACE_TRI");
	PX_CHECK_AND_RETURN_NULL(triMesh->getNbVerticesFast() <= PX_MAX_NB_DEFORMABLE_SURFACE_VTX,
		"PxDeformableSurface::attachShape: triangle mesh consists of too many vertices, see PX_MAX_NB_DEFORMABLE_SURFACE_VTX");

#if PX_CHECKED
	const PxU32 triangleReference = triMesh->getNbTriangleReferences();
	PX_CHECK_AND_RETURN_NULL(triangleReference > 0,
		"PxDeformableSurface::attachShape: deformable surface triangle mesh has been cooked with eENABLE_VERT_MAPPING");
#endif

	PX_CHECK_AND_RETURN_NULL(npShape->getCore().getCore().mShapeCoreFlags & PxShapeCoreFlag::eDEFORMABLE_SURFACE_SHAPE,
		"PxDeformableSurface::attachShape: shape must be a deformable surface shape!");

	Dy::DeformableSurfaceCore& core = mCore.getCore();
	PX_CHECK_AND_RETURN_NULL(core.positionInvMass == NULL,
		"PxDeformableSurface::attachShape: positionInvMass already exists, overwrite not allowed, call detachShape first");
	PX_CHECK_AND_RETURN_NULL(core.velocity == NULL,
		"PxDeformableSurface::attachShape: velocity already exists, overwrite not allowed, call detachShape first");
	PX_CHECK_AND_RETURN_NULL(core.restPosition == NULL,
		"PxDeformableSurface::attachShape: restPosition already exists, overwrite not allowed, call detachShape first");

	mShape = npShape;
	
	PX_ASSERT(shape.getActor() == NULL);
	npShape->onActorAttach(*this);

	// AD: these allocations need to happen immediately. We cannot defer these to the PxgSimulationController.
	createAllocator();
	const PxU32 numVerts = triMesh->getNbVerticesFast();
	core.positionInvMass = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVerts * sizeof(PxVec4), PxsHeapStats::eSHARED_FEMCLOTH, PX_FL));
	core.velocity = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVerts * sizeof(PxVec4), PxsHeapStats::eSHARED_FEMCLOTH, PX_FL));
	core.restPosition = reinterpret_cast<PxVec4*>(mDeviceMemoryAllocator->allocate(numVerts * sizeof(PxVec4), PxsHeapStats::eSHARED_FEMCLOTH, PX_FL));

	updateMaterials();

	return true;
}
	
void NpDeformableSurface::detachShape()
{
	Dy::DeformableSurfaceCore& core = mCore.getCore();

	PX_ASSERT(mDeviceMemoryAllocator);

	if (core.positionInvMass)
	{
		mDeviceMemoryAllocator->deallocate(core.positionInvMass);
		core.positionInvMass = NULL;
	}

	if (core.velocity)
	{
		mDeviceMemoryAllocator->deallocate(core.velocity);
		core.velocity = NULL;
	}

	if (core.restPosition)
	{
		mDeviceMemoryAllocator->deallocate(core.restPosition);
		core.restPosition = NULL;
	}

	if (mShape)
		mShape->onActorDetach();
	mShape = NULL;
}

PxCudaContextManager* NpDeformableSurface::getCudaContextManager() const
{
	return mCudaContextManager;
}

// deprecated
void NpDeformableSurface::setParameter(const PxFEMParameters& parameters)
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
PxFEMParameters NpDeformableSurface::getParameter() const
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
// PxDeformableSurface API
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableSurface::setDeformableSurfaceFlag(PxDeformableSurfaceFlag::Enum flag, bool val)
{
	PxDeformableSurfaceFlags flags = mCore.getSurfaceFlags();
	if (val)
		flags.raise(flag);
	else
		flags.clear(flag);

	mCore.setSurfaceFlags(flags);
}

void NpDeformableSurface::setDeformableSurfaceFlags(PxDeformableSurfaceFlags flags)
{
	mCore.setSurfaceFlags(flags);
}

PxDeformableSurfaceFlags NpDeformableSurface::getDeformableSurfaceFlags() const
{
	return mCore.getSurfaceFlags();
}

void NpDeformableSurface::setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableSurface::setNbCollisionPairUpdatesPerTimestep() not allowed while simulation is running. Call will be ignored.")

	mCore.setNbCollisionPairUpdatesPerTimestep(frequency);
	UPDATE_PVD_PROPERTY
}

PxU32 NpDeformableSurface::getNbCollisionPairUpdatesPerTimestep() const
{
	return mCore.getNbCollisionPairUpdatesPerTimestep();
}

void NpDeformableSurface::setNbCollisionSubsteps(const PxU32 frequency)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxDeformableSurface::setNbCollisionSubsteps() not allowed while simulation is running. Call will be ignored.")

	mCore.setNbCollisionSubsteps(frequency);
	UPDATE_PVD_PROPERTY
}

PxU32 NpDeformableSurface::getNbCollisionSubsteps() const
{
	return mCore.getNbCollisionSubsteps();
}

PxVec4* NpDeformableSurface::getPositionInvMassBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mShape != NULL, " PxDeformableSurface::getPositionInvMassBufferD: Deformable surface does not have a shape, attach shape first.");
	return mCore.getCore().positionInvMass;
}

PxVec4* NpDeformableSurface::getVelocityBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mShape != NULL, " PxDeformableSurface::getVelocityBufferD: Deformable surface does not have a shape, attach shape first.");
	return mCore.getCore().velocity;
}

PxVec4* NpDeformableSurface::getRestPositionBufferD()
{
	PX_CHECK_AND_RETURN_NULL(mShape != NULL, " PxDeformableSurface::getRestPositionBufferD: Deformable surface does not have a shape, attach shape first.");
	return mCore.getCore().restPosition;
}

void NpDeformableSurface::markDirty(PxDeformableSurfaceDataFlags flags)
{
	NP_WRITE_CHECK(getNpScene());
	mCore.getCore().dirtyFlags |= flags;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Internal
/////////////////////////////////////////////////////////////////////////////////////////

void NpDeformableSurface::updateMaterials()
{
	mCore.clearMaterials();
	for (PxU32 i = 0; i < mShape->getNbMaterials(); ++i)
	{
		PxDeformableSurfaceMaterial* material;
		mShape->getDeformableSurfaceMaterials(&material, 1, i);
		mCore.addMaterial(static_cast<NpDeformableSurfaceMaterial*>(material)->mMaterial.mMaterialIndex);
	}
}

void NpDeformableSurface::createAllocator()
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

void NpDeformableSurface::releaseAllocator()
{
	// deallocate device memory if not released already.
	Dy::DeformableSurfaceCore& core = mCore.getCore();
	if (core.positionInvMass)
	{
		mDeviceMemoryAllocator->deallocate(core.positionInvMass);
		core.positionInvMass = NULL;
	}
	if (core.velocity)
	{
		mDeviceMemoryAllocator->deallocate(core.velocity);
		core.velocity = NULL;
	}
	if (core.restPosition)
	{
		mDeviceMemoryAllocator->deallocate(core.restPosition);
		core.restPosition = NULL;
	}

	if (mMemoryManager != NULL)
	{
		mDeviceMemoryAllocator = NULL; // released by memory manager
		PX_DELETE(mMemoryManager);
	}
}

Sc::DeformableSurfaceCore* getDeformableSurfaceCore(PxActor* actor)
{
	if (actor->getConcreteType() == PxConcreteType::eDEFORMABLE_SURFACE)
	{
		NpDeformableSurface* dyn = static_cast<NpDeformableSurface*>(actor);
		return &dyn->getCore();
	}
	return NULL;
}

} // namespace physx

#endif //PX_SUPPORT_GPU_PHYSX
