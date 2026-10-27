// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_DEFORMABLE_VOLUME_CORE_H
#define DY_DEFORMABLE_VOLUME_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "PxDeformableVolume.h"
#include "PxDeformableVolumeFlag.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "foundation/PxArray.h"
#include "DyDeformableBodyCore.h"

namespace physx
{
namespace Dy
{

struct DeformableVolumeCore : public DeformableBodyCore
{
public:
	PxQuat							initialRotation;
	PxReal							freezeThreshold;	// not exposed (stabilization threshold)

	//device - managed by PhysX
	PxVec4*							positionInvMass;     // collision mesh positions, alloc on attachShape(), dealloc detachShape()
	PxVec4* 						restPosition;        // collision mesh rest positions, alloc on attachShape(), dealloc detachShape()
	PxVec4*							simPositionInvMass;  // simulation mesh positions, alloc on attachSimulationMesh(), dealloc detachSimulationMesh()
	PxVec4*							simVelocity;         // simulation mesh velocities, alloc on attachSimulationMesh(), dealloc detachSimulationMesh()

	// device - just the pointer, user responsible.
	const PxVec4*					kinematicTarget;

	PxDeformableVolumeDataFlags		dirtyFlags;
	PxDeformableVolumeFlags			volumeFlags;

	DeformableVolumeCore()
		: initialRotation(PxIdentity)
		, freezeThreshold(0.0f)
		, positionInvMass(NULL)
		, restPosition(NULL)
		, simPositionInvMass(NULL)
		, simVelocity(NULL)
		, kinematicTarget(NULL)
		, dirtyFlags(0)
		, volumeFlags(0)
	{
	}
};

} // namespace Dy
} // namespace physx

#endif // DY_DEFORMABLE_VOLUME_CORE_H

