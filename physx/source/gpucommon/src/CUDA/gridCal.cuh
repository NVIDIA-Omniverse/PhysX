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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef __CU_GRID_CAL_CUH__
#define __CU_GRID_CAL_CUH__

#include "foundation/PxSimpleTypes.h"

PX_FORCE_INLINE static __device__ int3 calcGridPos(const float4& particlePos, const PxReal cellWidth)
{
	int3 gridPos;

	gridPos.x = floor((particlePos.x) / cellWidth);
	gridPos.y = floor((particlePos.y) / cellWidth);
	gridPos.z = floor((particlePos.z) / cellWidth);

	return gridPos;
}

// calculate address in grid from position (wrap-around)
PX_FORCE_INLINE static __device__ physx::PxU32 calcGridHash(int3 gridPos, uint3 gridSize)
{
	gridPos.x = gridPos.x & (gridSize.x - 1);
	gridPos.y = gridPos.y & (gridSize.y - 1);
	gridPos.z = gridPos.z & (gridSize.z - 1);

	return ((gridPos.z * gridSize.y) * gridSize.x) + (gridPos.y * gridSize.x) + gridPos.x;
}

PX_FORCE_INLINE static __device__ PxU32 calcGridHashPeriodic(int3 gridPos, int3 gridSize, int3 periodGridSize)
{
	//With periodic boundaries, gridPos for a particle should be >= 0 always. Particle positions were wrapped based on periodic grid size so 
	//the particles are within in the periodic range.
	//When reading neighboring cells, we wrap using a simple < and >= check. We don't need modulo.
	//We use the original grid size (non-periodic) to then work out the hash of the particle.

	if (gridPos.x < 0)
		gridPos.x += periodGridSize.x;
	else if (gridPos.x >= periodGridSize.x)
		gridPos.x -= periodGridSize.x;

	if (gridPos.y < 0)
		gridPos.y += periodGridSize.y;
	else if (gridPos.y >= periodGridSize.y)
		gridPos.y -= periodGridSize.y;

	if (gridPos.z < 0)
		gridPos.z += periodGridSize.z;
	else if (gridPos.z >= periodGridSize.z)
		gridPos.z -= periodGridSize.z;

	return ((gridPos.z * gridSize.y) * gridSize.x) + (gridPos.y * gridSize.x) + gridPos.x;
}

/**
 * takes a global grid range e.g. (-4 , 2, -1) to (-1, 3, 2) and computes a range size
 * which is clamped to the wrapped grid size. The result is just used to compute cell hashes
 * hence if the range in a dimension exceeds the wrapped grid size, all wrapped cells in that dimension
 * are covered anyways. The clamp to 0 is probably there to catch numerical issues.
 */
PX_FORCE_INLINE static __device__ uint3 calcWrappedGridRangeSize(int3 gridRangeMin, int3 gridRangeMax, uint3 gridSize)
{
	const PxU32 x = PxClamp(gridRangeMax.x - gridRangeMin.x + 1, 0, PxI32(gridSize.x));
	const PxU32 y = PxClamp(gridRangeMax.y - gridRangeMin.y + 1, 0, PxI32(gridSize.y));
	const PxU32 z = PxClamp(gridRangeMax.z - gridRangeMin.z + 1, 0, PxI32(gridSize.z));
	return make_uint3(x, y, z);
}

/**
 * Takes a linear cell offset and maps it to a 3D offset within a 3D grid range.
 */
PX_FORCE_INLINE static __device__ uint3 calcGridOffsetInRange(uint3 gridRangeSize, PxU32 offset)
{
	const PxU32 x = offset % gridRangeSize.x;
	const PxU32 y = (offset / gridRangeSize.x) % gridRangeSize.y;
	const PxU32 z = offset / (gridRangeSize.x * gridRangeSize.y);
	return make_uint3(x, y, z);
}

PX_FORCE_INLINE static __device__ void calcGridRange(int3& gridPosMin, int3& gridPosMax, const PxBounds3& bounds, float cellWidth)
{
	float4 min = make_float4(bounds.minimum.x, bounds.minimum.y, bounds.minimum.z, 0.f);
	float4 max = make_float4(bounds.maximum.x, bounds.maximum.y, bounds.maximum.z, 0.f);
	gridPosMin = calcGridPos(min, cellWidth);
	gridPosMax = calcGridPos(max, cellWidth);
}

#define PARTICLE_FORWARD_PROJECTION_STEP_SCALE_PGS 1.0f
#define PARTICLE_FORWARD_PROJECTION_STEP_SCALE_TGS 0.5f
#define PARTICLE_FORWARD_PROJECTION_STEP_SCALE_DIFFUSE 1.0f

/**
 * Returns position and radius of the volume that needs to be tested for particle collision.
 * The function reconstructs the full forward projection step in pre integration and selects the mid point and a 
 * radius that covers the whole projected motion range and adds the contact distance.
 */
 PX_FORCE_INLINE static __device__ PxReal getParticleSpeculativeContactVolume(PxVec3& cVolumePos,
	const PxVec3& currentPos, const PxVec3& predictedPos, const PxReal contactDist, const bool isDiffuse, const bool isTGS)
{
	static const PxReal sScaleInvPGS = (1.0f / PARTICLE_FORWARD_PROJECTION_STEP_SCALE_PGS);
	static const PxReal sScaleInvTGS = (1.0f / PARTICLE_FORWARD_PROJECTION_STEP_SCALE_TGS);
	static const PxReal sScaleInvDiffuse = (1.0f / PARTICLE_FORWARD_PROJECTION_STEP_SCALE_DIFFUSE);
	PxReal scaleInv;
	if (isDiffuse)
	{
		scaleInv = sScaleInvDiffuse;
	}
	else
	{
		scaleInv = isTGS ? sScaleInvTGS : sScaleInvPGS;
	}
	PxVec3 cVolumeOffset = (predictedPos - currentPos)*scaleInv*0.5f;
	cVolumePos = currentPos + cVolumeOffset;
	return cVolumeOffset.magnitude() + contactDist;
}

#endif
