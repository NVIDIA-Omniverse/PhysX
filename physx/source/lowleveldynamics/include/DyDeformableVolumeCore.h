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

