// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_DEFORMABLE_VOLUME_SIM_H
#define SC_DEFORMABLE_VOLUME_SIM_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "foundation/PxUserAllocated.h"
#include "DyDeformableVolume.h"
#include "ScDeformableVolumeCore.h" 
#include "ScGpuActorSim.h"

namespace physx
{
namespace Sc
{

class Scene;

class DeformableVolumeSim : public GPUActorSim
{
	PX_NOCOPY(DeformableVolumeSim)
public:
											DeformableVolumeSim(DeformableVolumeCore& core, Scene& scene);
											~DeformableVolumeSim();

	PX_INLINE	Dy::DeformableVolume*		getLowLevelDeformableVolume() const { return mLLDeformableVolume; }
	PX_INLINE	DeformableVolumeCore&		getCore() const { return static_cast<DeformableVolumeCore&>(mCore); }

	virtual		PxActor*					getPxActor() const PX_OVERRIDE { return getCore().getPxActor(); }

				bool						isSleeping() const;
				bool						isActive() const { return !isSleeping(); }
		
				void						setActive(bool active, bool asPartOfCreation=false);

				void						enableSelfCollision();
				void						disableSelfCollision();

				void						onSetWakeCounter();

				PxBounds3					getWorldBounds() const;

				void						attachShapeCore(ShapeCore* core);
				void						attachSimulationMesh(PxTetrahedronMesh* simulationMesh, PxDeformableVolumeAuxData* simulationState);
				PxTetrahedronMesh*			getSimulationMesh();
				PxDeformableVolumeAuxData*	getAuxData();

				PxTetrahedronMesh*			getCollisionMesh();

				PxU32						getGpuIndex()	const;
private:
				Dy::DeformableVolume*		mLLDeformableVolume;
				PxU32						mIslandNodeIndex;
		
// PT: as far as I can tell these are never actually called
//							void			activate();
//							void			deactivate();
};

} // namespace Sc
} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
#endif // SC_DEFORMABLE_VOLUME_SIM_H
