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

#ifndef __PARTICLE_COLLISION_CUH__
#define __PARTICLE_COLLISION_CUH__

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "sphereCollision.cuh"
#include "deformableCollision.cuh"
#include "GuConvexSupport.h"
/**
* Retuns distance and normal information for a particle-box contact.
* If maxDist < PX_MAX_F32, then no result is provided if the distance is larger than maxDist.
*/
__device__ __forceinline__ static
bool contactPointBox(PxVec3& normal, PxReal& dist,
	const PxVec3& pointPos, const PxTransform& boxToWorld, const PxVec3& halfBoxExtents, const PxReal maxDist)
{
	PxVec3 relPointPos = boxToWorld.transformInv(pointPos);
	
	const PxVec3 closestPos(
		PxClamp(relPointPos.x, -halfBoxExtents.x, halfBoxExtents.x),
		PxClamp(relPointPos.y, -halfBoxExtents.y, halfBoxExtents.y),
		PxClamp(relPointPos.z, -halfBoxExtents.z, halfBoxExtents.z)
	);

	if (maxDist < PX_MAX_F32)
	{
		PxReal maxDistSq = maxDist*maxDist;
		PxReal distSq = (relPointPos - closestPos).magnitudeSquared();
		if (distSq > maxDistSq)
		{
			return false;
		}
	}

	//check whether the point is inside the aabb
	const bool bX = halfBoxExtents.x >= PxAbs(relPointPos.x);
	const bool bY = halfBoxExtents.y >= PxAbs(relPointPos.y);
	const bool bZ = halfBoxExtents.z >= PxAbs(relPointPos.z);

	const bool isInside = bX && bY && bZ;
	if (isInside)
	{
		//dist from embedded point to box surface along 3 dimensions.
		const PxVec3 distToSurface = halfBoxExtents - closestPos.abs();

		//assume z is the smallest element of the distToSurface
		normal = closestPos.z >= 0.f ? PxVec3(0.f, 0.f, 1.f) : PxVec3(0.f, 0.f, -1.f);
		dist = -distToSurface.z;
		//find smallest element of distToSurface
		if (distToSurface.x <= distToSurface.y && distToSurface.x <= distToSurface.z)
		{
			normal = closestPos.x >= 0.f ? PxVec3(1.f, 0.f, 0.f) : PxVec3(-1.0f, 0.f, 0.f);
			dist = -distToSurface.x;
		}
		else if (distToSurface.y <= distToSurface.z)
		{
			normal = closestPos.y >= 0.f ? PxVec3(0.f, 1.f, 0.f) : PxVec3(0.f, -1.f, 0.f);
			dist = -distToSurface.y;
		}
	}
	else
	{
		normal = relPointPos - closestPos;
		dist = normal.normalizeFast();
	}
	normal = boxToWorld.rotate(normal);
	return true;
}

/**
* Retuns distance and normal information for a particle-sphere contact.
* If maxDist < PX_MAX_F32, then no result is provided if the distance is larger than maxDist.
*/
__device__ __forceinline__ static
bool contactPointSphere(PxVec3& normal, PxReal& dist,
	const PxVec3& pointPos, const PxVec3& spherePos, const PxReal sphereRadius, const PxReal maxDist)
{
	PxVec3 relPointPos = pointPos - spherePos;
	const PxReal distCenterSq = relPointPos.magnitudeSquared();
	if (maxDist < PX_MAX_F32)
	{
		const PxReal maxDistCenter = maxDist + sphereRadius;
		const PxReal maxDistCenterSq = maxDistCenter*maxDistCenter;
		if (distCenterSq > maxDistCenterSq)
		{
			return false;
		}
	}
	const PxReal distCenter = PxSqrt(distCenterSq);
	normal = (distCenter > PX_EPS_REAL) ? relPointPos * (1.0f / distCenter) : PxVec3(1.0f, 0.0f, 0.0f);
	dist = distCenter - sphereRadius;
	return true;
}

/**
* Retuns distance and normal information for a point-capsule contact.
* If maxDist < PX_MAX_F32, then no result is provided if the distance is larger than maxDist.
*/
__device__ __forceinline__ static
bool contactPointCapsule(PxVec3& normal, PxReal& dist, 
	const PxVec3& pointPos, const PxVec3& capsulePos, const PxVec3& capsuleDir, const PxReal capsuleRadius, const PxReal capsuleHalfHeight, const PxReal maxDist)
{
	const PxVec3 capsuleAxisVec = capsuleDir * capsuleHalfHeight;
	const PxVec3 relPointPos = pointPos - capsulePos;
	PxReal t;
	const PxReal distSegmentSq = distancePointSegmentSquared(-capsuleAxisVec, capsuleAxisVec, relPointPos, t);
	if (maxDist < PX_MAX_F32)
	{
		const PxReal maxDistSegment = maxDist + capsuleRadius;
		const PxReal maxDistSegmentSq = maxDistSegment * maxDistSegment;
		if (distSegmentSq > maxDistSegmentSq)
		{
			return false;
		}
	}
	const PxVec3 distSegmentVec = relPointPos - capsuleAxisVec * (2.0f * t - 1.0f);
	const PxReal distSegment = PxSqrt(distSegmentSq);
	normal = (distSegment > PX_EPS_REAL) ? (distSegmentVec * (1.0f / distSegment)) : PxVec3(1.0f, 0.0f, 0.0f);
	dist = distSegment - capsuleRadius;
	return true;
}

/**
* Combined speculative particle triangle cd. Returns normal and distance.
* If CCD is enabled two tests are performed. The first one decides if the triangle
* is overlapping with the contact volume (sphere), which is typically located at a
* predicted position of the particle, covering it's whole predicted path. The second 
* test then establishes the correct distance and normal with respect to the actual particle
* position.
* If CCD is disabled, particlePos == cVolumePos, and cVolumeRadius is the standard contact 
* distance.
*/
__device__ __forceinline__ static
bool particleTriangleTest(PxVec3& normal, PxReal& distance,
	const PxVec3& particlePos, const PxVec3& cVolumePos, const PxReal cVolumeRadius,
	const PxVec3& triV0, const PxVec3& triV1, const PxVec3& triV2, const bool enableCCD)
{
	const PxVec3 triNormal = ((triV1 - triV0).cross(triV2 - triV0)).getNormalized();
	const PxReal cVolumeRadiusSq = cVolumeRadius * cVolumeRadius;

	//contact volume test
	const PxReal cVolumePosPlaneDist = (cVolumePos - triV0).dot(triNormal);
	PxVec3 closestPos;
	if (cVolumePosPlaneDist < cVolumeRadius)
	{
		PxReal distSq = distancePointTriangleSquared(cVolumePos, triV0, triV1, triV2, closestPos);
		if (distSq <= cVolumeRadiusSq)
		{
			//intersection test
			//back face culling with respect to current particle position
			const PxReal particlePosPlaneDist = (particlePos - triV0).dot(triNormal);
			if (particlePosPlaneDist > 0.0f)
			{
				if (enableCCD)
				{
					//in case of CCD we replace the closest and distance from the contact volume with
					//the corresponding from the current particle position 
					distSq = distancePointTriangleSquared(particlePos, triV0, triV1, triV2, closestPos);
				}
				//assumes particlePos == cVolumePos if enableCCD == false.
				PxVec3 distVec = particlePos - closestPos;
				normal = (distSq > 0.0f) ? distVec*PxRecipSqrt(distSq) : triNormal;
				distance = PxSqrt(distSq);
				return true;
			}
		}
	}
	return false;
}

/**
* Retuns distance and normal information for a particle-convexCore contact.
* If maxDist < PX_MAX_F32, then no result is provided if the distance is larger than maxDist.
*/
__device__ __forceinline__ static
bool particleConvexCore(PxVec3& normal, PxReal& dist, const PxVec3& particlePos,
	const Gu::ConvexShape& convex, const PxReal maxDist)
{
	Gu::ConvexShape particle;
	particle.coreType = Gu::ConvexCore::Type::ePOINT;
	particle.pose = PxTransform(particlePos);
	particle.margin = 0;

	PxVec3 point0, point1;
	dist = Gu::RefGjkEpa::computeGjkDistance(particle, convex, particle.pose, convex.pose,
		convex.margin + particle.margin + maxDist, point0, point1, normal);

	if (dist < FLT_EPSILON)
		dist = Gu::RefGjkEpa::computeEpaDepth(particle, convex, particle.pose, convex.pose,
			point0, point1, normal);

	if (dist > maxDist)
		return false;

	return true;
}
#endif // __PARTICLE_COLLISION_CUH__