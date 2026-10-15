// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_DEFORMABLE_VOLUME_H
#define DY_DEFORMABLE_VOLUME_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxArray.h"
#include "foundation/PxUserAllocated.h"
#include "DyDeformableVolumeCore.h"
#include "PxvGeometry.h"

namespace physx
{
namespace Sc
{
	class DeformableVolumeSim;
}

namespace Dy
{

typedef size_t DeformableVolumeHandle;
struct DeformableVolumeCore;

class DeformableVolume
{
	PX_NOCOPY(DeformableVolume)
public:
	DeformableVolume(Sc::DeformableVolumeSim* sim, Dy::DeformableVolumeCore& core) :
		mSim(sim), mCore(core), mElementId(0xffffffff), mGpuRemapId(0xffffffff)
	{
	}

	PX_FORCE_INLINE void								setGpuRemapId(const PxU32 remapId) 
	{ 
		mGpuRemapId = remapId;
		PxTetrahedronMeshGeometryLL& geom = mShapeCore->mGeometry.get<PxTetrahedronMeshGeometryLL>();
		geom.materialsLL.gpuRemapId = remapId;
	}

	PX_FORCE_INLINE PxU32								getGpuRemapId() { return mGpuRemapId; }

	PX_FORCE_INLINE void								setElementId(const PxU32 elementId) { mElementId = elementId; }
	PX_FORCE_INLINE PxU32								getElementId() { return mElementId; }

	PX_FORCE_INLINE PxsShapeCore&						getShapeCore() { return *mShapeCore; }
	PX_FORCE_INLINE void								setShapeCore(PxsShapeCore* shapeCore) { mShapeCore = shapeCore; }

	PX_FORCE_INLINE void								setSimShapeCore(PxTetrahedronMesh* simulationMesh, PxDeformableVolumeAuxData* simulationState)
	{ 
		mSimulationMesh = simulationMesh;
		mAuxData = simulationState;
	}

	PX_FORCE_INLINE const PxTetrahedronMesh*			getCollisionMesh() const { return mShapeCore->mGeometry.get<PxTetrahedronMeshGeometryLL>().tetrahedronMesh; }
	PX_FORCE_INLINE PxTetrahedronMesh*					getCollisionMesh() { return mShapeCore->mGeometry.get<PxTetrahedronMeshGeometryLL>().tetrahedronMesh; }

	PX_FORCE_INLINE const PxTetrahedronMesh*			getSimulationMesh() const { return mSimulationMesh; }
	PX_FORCE_INLINE PxTetrahedronMesh*					getSimulationMesh() { return mSimulationMesh; }

	PX_FORCE_INLINE const PxDeformableVolumeAuxData*	getAuxData() const { return mAuxData; }
	PX_FORCE_INLINE PxDeformableVolumeAuxData*			getAuxData() { return mAuxData; }

	PX_FORCE_INLINE Sc::DeformableVolumeSim*			getSim() const { return mSim; }
	PX_FORCE_INLINE const DeformableVolumeCore&			getCore() const { return mCore; }
	PX_FORCE_INLINE DeformableVolumeCore&				getCore() { return mCore; }

	PX_FORCE_INLINE PxU16								getIterationCounts() const { return mCore.solverIterationCounts; }

	PX_FORCE_INLINE PxU32								getGpuIndex() const { return mGpuRemapId;  }

	PxArray<PxU32>										mRigidAttachments;
	PxArray<PxU32>										mVolumeAttachments;
	PxArray<PxU32>										mSurfaceAttachments;

private:
	Sc::DeformableVolumeSim*							mSim;
	DeformableVolumeCore&								mCore;
	PxsShapeCore*										mShapeCore; 

	PxU32												mElementId; //this is used for the bound array, contactDist
	PxU32												mGpuRemapId;

	PxTetrahedronMesh*									mSimulationMesh;
	PxDeformableVolumeAuxData*							mAuxData;
};

PX_FORCE_INLINE DeformableVolume*						getDeformableVolume(DeformableVolumeHandle handle)
{
	return reinterpret_cast<DeformableVolume*>(handle);
}

} // namespace Dy
} // namespace physx

#endif // DY_DEFORMABLE_VOLUME_H
