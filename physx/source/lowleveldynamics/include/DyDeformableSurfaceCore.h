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
