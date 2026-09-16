// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_DEFORMABLE_SURFACE_SIM_H
#define SC_DEFORMABLE_SURFACE_SIM_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "foundation/PxUserAllocated.h"
#include "DyDeformableSurface.h"
#include "ScDeformableSurfaceCore.h" 
#include "ScGpuActorSim.h"

namespace physx
{
namespace Sc
{

class Scene;

class DeformableSurfaceSim : public GPUActorSim
{
	PX_NOCOPY(DeformableSurfaceSim)
public:
	DeformableSurfaceSim(DeformableSurfaceCore& core, Scene& scene);
	~DeformableSurfaceSim();

	PX_INLINE	Dy::DeformableSurface*	getLowLevelDeformableSurface() const { return mLLDeformableSurface; }
	PX_INLINE	DeformableSurfaceCore&	getCore() const { return static_cast<DeformableSurfaceCore&>(mCore); }
	virtual		PxActor*				getPxActor() const PX_OVERRIDE { return getCore().getPxActor(); }

	bool							isSleeping() const;
	PX_FORCE_INLINE	bool			isActive() const { return !isSleeping(); }

	void							setActive(bool active, bool asPartOfCreation=false);

	void							onSetWakeCounter();

	void							attachShapeCore(ShapeCore* core);
	PxBounds3						getWorldBounds()	const;

private:
	Dy::DeformableSurface*			mLLDeformableSurface;

	PxU32							mIslandNodeIndex;

					void			activate();
					void			deactivate();
};

} // namespace Sc
} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
#endif // SC_DEFORMABLE_SURFACE_SIM_H
