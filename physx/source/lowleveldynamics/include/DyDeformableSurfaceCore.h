// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_DEFORMABLE_SURFACE_CORE_H
#define DY_DEFORMABLE_SURFACE_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec4.h"
#include "foundation/PxArray.h"
#include "PxDeformableSurface.h"
#include "PxDeformableSurfaceFlag.h"
#include "DyDeformableBodyCore.h"

namespace physx
{
namespace Dy
{

struct DeformableSurfaceCore : public DeformableBodyCore
{
public:
	// number of collision pair updates per timestep. Collision pair is updated at least once per timestep and increasing the frequency provides better collision pairs.
	PxU32							nbCollisionPairUpdatesPerTimestep;

	// number of collision substeps in each sub-timestep. Collision constraints can be applied multiple times in each sub-timestep.
	PxU32							nbCollisionSubsteps;

	//device - managed by PhysX
	PxVec4*							positionInvMass;
	PxVec4*							velocity;
	PxVec4*							restPosition;

	PxDeformableSurfaceDataFlags	dirtyFlags;
	PxDeformableSurfaceFlags		surfaceFlags;

	DeformableSurfaceCore()
		: nbCollisionPairUpdatesPerTimestep(0)
		, nbCollisionSubsteps(1)
		, positionInvMass(NULL)
		, velocity(NULL)
		, restPosition(NULL)
		, dirtyFlags(0)
		, surfaceFlags(0)
	{
	}
};

} // namespace Dy
} // namespace physx

#endif // DY_DEFORMABLE_SURFACE_CORE_H
