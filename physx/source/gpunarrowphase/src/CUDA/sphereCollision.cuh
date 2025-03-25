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

#ifndef __SPHERE_COLLISION_CUH__
#define __SPHERE_COLLISION_CUH__

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "distanceSegmentSegment.cuh"

//0 is sphere, 1 is sphere, point = [x, y, z, pen];
__device__ __forceinline__ static
PxU32 spheresphere(const PxTransform& transform0, const PxTransform& transform1, const PxReal r0, 
	const PxReal r1, const PxReal contactDist, float4& outPointPen, PxVec3& outNormal)
{
	const PxVec3& p0 = transform0.p;
	const PxVec3& p1 = transform1.p;

	const PxVec3 _delta = p0 - p1;
	const PxReal distanceSq = _delta.dot(_delta);
	const PxReal radiusSum = r0 + r1;
	const PxReal inflatedSum = radiusSum + contactDist;

	if ((inflatedSum * inflatedSum) > distanceSq)
	{
		const PxReal eps = 1e-4f;
		const PxReal dist = PxSqrt(distanceSq);
		const PxVec3 normal = (eps >= dist) ? PxVec3(1.0f, 0.f, 0.f) : (_delta * (1.f / dist));
		const PxVec3 point = p1 + (normal * r1);
		const PxReal pen = dist - radiusSum;

		outPointPen = make_float4(point.x, point.y, point.z, pen);
		outNormal = normal;

		return 1;
	}

	return 0;
}

__device__ __forceinline__ static
PxU32 spheresphere(const PxVec3& p0, const PxVec3& p1, const PxReal r0,
	const PxReal r1, const PxReal contactDist, float4& outNormalPen)
{
	const PxVec3 _delta = p0 - p1;
	const PxReal distanceSq = _delta.dot(_delta);
	const PxReal radiusSum = r0 + r1;
	const PxReal inflatedSum = radiusSum + contactDist;

	if ((inflatedSum * inflatedSum) > distanceSq)
	{
		const PxReal eps = 1e-4f;
		const PxReal dist = PxSqrt(distanceSq);
		const PxVec3 normal = (eps >= dist) ? PxVec3(1.0f, 0.f, 0.f) : (_delta * (1.f / dist));
		const PxReal pen = dist - radiusSum;

		outNormalPen = make_float4(normal.x, normal.y, normal.z, pen);

		return 1;
	}

	return 0;
}

__device__ __forceinline__ static
PxReal distancePointSegmentSquared(const PxVec3 a, const PxVec3 b, const PxVec3 p, PxReal& param)
{
	const PxVec3 ap = p - a;
	const PxVec3 ab = b - a;

	const PxReal nom = ap.dot(ab);
	const PxReal denom = ab.dot(ab);
	const PxReal tValue = PxClamp(nom / denom, 0.f, 1.f);
	const PxReal t = denom > 0.f ? tValue : 0.f;
	const PxVec3 v = ap - (ab * t);
	param = t;
	return v.dot(v);
}

//0 is sphere, 1 is capsule
__device__ __forceinline__ static
PxU32 spherecapsule(const PxTransform& transform0, const PxTransform& transform1, const PxReal sphereRadius, 
	const PxReal capsuleRadius, const PxReal halfHeight, const PxReal contactDist, float4& outPointPen, PxVec3& outNormal)
{

	//Sphere in world space
	const PxVec3& sphereCenter = transform0.p;
	const PxQuat& q1 = transform1.q;
	const PxVec3& p1 = transform1.p;

	const PxVec3 basisVector0 = q1.getBasisVector0();
	const PxVec3 tmp0 = basisVector0 * halfHeight;
	const PxVec3 s = p1 + tmp0;
	const PxVec3 e = p1 - tmp0;

	const PxReal radiusSum = sphereRadius + capsuleRadius;
	const PxReal inflatedSum = radiusSum + contactDist;

	PxReal t;
	const PxReal squareDist = distancePointSegmentSquared(s, e, sphereCenter, t);
	const PxReal sqInflatedSum = inflatedSum * inflatedSum;

	// Collision detection
	if (sqInflatedSum > squareDist)
	{
		const PxVec3 p = s + (e - s) * t;//V3ScaleAdd(V3Sub(e, s), t, s);
		const PxVec3 dir = sphereCenter - p;// V3Sub(sphereCenter, p);

		const PxReal length = PxSqrt(dir.magnitudeSquared());
		const PxVec3 normal = length > PX_EPS_REAL ? (dir * (1.f / length)) : PxVec3(1.0f, 0.f, 0.f);
		const PxVec3 point = sphereCenter - normal * sphereRadius;
		const PxReal pen = PxSqrt(squareDist) - radiusSum;

		outPointPen = make_float4(point.x, point.y, point.z, pen);
		outNormal = normal;

		/*printf("outPoint(%f, %f, %f), pen %f\n", outPointPen.x, outPointPen.y, outPointPen.z, outPointPen.w);
		printf("outNormal(%f, %f, %f)\n", outNormal.x, outNormal.y, outNormal.z, 0.f);*/

		return 1;
	}

	return 0;
}

//0 is sphere, 1 is capsule
__device__ __forceinline__ static
PxU32 spherecapsule(const PxTransform& transform0, const PxTransform& transform1, const PxReal sphereRadius,
	const PxReal capsuleRadius, const PxReal halfHeight, const PxReal contactDist, float4& outNormalPen, PxReal* outT = NULL)
{

	//Sphere in world space
	const PxVec3& sphereCenter = transform0.p;
	const PxQuat& q1 = transform1.q;
	const PxVec3& p1 = transform1.p;

	const PxVec3 basisVector0 = q1.getBasisVector0();
	const PxVec3 tmp0 = basisVector0 * halfHeight;
	const PxVec3 s = p1 + tmp0;
	const PxVec3 e = p1 - tmp0;

	const PxReal radiusSum = sphereRadius + capsuleRadius;
	const PxReal inflatedSum = radiusSum + contactDist;

	PxReal t;
	const PxReal squareDist = distancePointSegmentSquared(s, e, sphereCenter, t);
	const PxReal sqInflatedSum = inflatedSum * inflatedSum;
	if (outT != NULL)
	{
		*outT = 1.0f - t;
	}

	// Collision detection
	if (sqInflatedSum > squareDist)
	{
		const PxVec3 p = s + (e - s) * t;//V3ScaleAdd(V3Sub(e, s), t, s);
		const PxVec3 dir = sphereCenter - p;// V3Sub(sphereCenter, p);

		const PxReal length = PxSqrt(dir.magnitudeSquared());
		const PxVec3 normal = length > PX_EPS_REAL ? (dir * (1.f / length)) : PxVec3(1.0f, 0.f, 0.f);
		const PxReal pen = PxSqrt(squareDist) - radiusSum;

		outNormalPen = make_float4(normal.x, normal.y, normal.z, pen);

		/*printf("outPoint(%f, %f, %f), pen %f\n", outPointPen.x, outPointPen.y, outPointPen.z, outPointPen.w);
		printf("outNormal(%f, %f, %f)\n", outNormal.x, outNormal.y, outNormal.z, 0.f);*/

		return 1;
	}

	return 0;
}

//0 is sphere, 1 is plane
__device__ __forceinline__ static
PxU32 sphereplane(const PxTransform& transform0, const PxTransform& transform1, const PxReal sphereRadius, const PxReal contactDist,
	float4& outPointPen, PxVec3& outNormal)
{
	//sphere transform
	const PxVec3& p0 = transform0.p;
	const PxVec3& p1 = transform1.p;
	const PxQuat& q1 = transform1.q;

	const PxTransform transf1(p1, q1);
	//Sphere in plane space
	const PxVec3 sphereCenterInPlaneSpace = transf1.transformInv(p0);

	//separation / penetration
	const PxReal separation = sphereCenterInPlaneSpace.x - sphereRadius;// FSub(V3GetX(sphereCenterInPlaneSpace), radius);

	if (contactDist >= separation)
	{
		//get the plane normal
		const PxVec3 worldNormal = q1.getBasisVector0();
		const PxVec3 worldPoint = p0 - worldNormal * sphereRadius;//V3NegScaleSub(worldNormal, radius, p0);
		
		outPointPen = make_float4(worldPoint.x, worldPoint.y, worldPoint.z, separation);
		outNormal = worldNormal;

		return 1;
	}

	return 0;
}

//0 is sphere, 1 is plane
__device__ __forceinline__ static
PxU32 sphereplane(const PxTransform& transform0, const PxTransform& transform1, const PxReal sphereRadius, const PxReal contactDist,
	float4& outNormalPen)
{
	//sphere transform
	const PxVec3& p0 = transform0.p;
	const PxVec3& p1 = transform1.p;
	const PxQuat& q1 = transform1.q;

	const PxTransform transf1(p1, q1);
	//Sphere in plane space
	const PxVec3 sphereCenterInPlaneSpace = transf1.transformInv(p0);

	//separation / penetration
	const PxReal separation = sphereCenterInPlaneSpace.x - sphereRadius;// FSub(V3GetX(sphereCenterInPlaneSpace), radius);

	if (contactDist >= separation)
	{
		//get the plane normal
		const PxVec3 worldNormal = q1.getBasisVector0();
		outNormalPen = make_float4(worldNormal.x, worldNormal.y, worldNormal.z, separation);

		return 1;
	}

	return 0;
}

//0 is sphere, 1 is box
__device__ __forceinline__ static
PxU32 spherebox(const PxTransform& transform0, const PxTransform& transform1, const PxReal sphereRadius, 
	const PxVec3 boxExtents, const PxReal contactDist, float4& outPointPen, PxVec3& outNormal)
{
	const PxVec3& sphereOrigin = transform0.p;

	//const PxQuat& q1 = transform1.q;
	//const PxVec3& p1 = transform1.p;

	//const PxTransform transf1(p1, q1);

	//translate sphere center into the box space
	const PxVec3 sphereCenter = transform1.transformInv(sphereOrigin);

	const PxVec3 nBoxExtents = -boxExtents;

	const PxReal inflatedSum = sphereRadius + contactDist;
	const PxReal sqInflatedSum = inflatedSum * inflatedSum;

	const PxReal x = PxClamp(sphereCenter.x, nBoxExtents.x, boxExtents.x);
	const PxReal y = PxClamp(sphereCenter.y, nBoxExtents.y, boxExtents.y);
	const PxReal z = PxClamp(sphereCenter.z, nBoxExtents.z, boxExtents.z);
	const PxVec3 p(x, y, z);
	const PxVec3 v = sphereCenter - p;
	const PxReal lengthSq = v.dot(v);

	if (sqInflatedSum > lengthSq)//intersect
	{
		//check whether the spherCenter is inside the box
		const bool bX = boxExtents.x >= PxAbs(sphereCenter.x);
		const bool bY = boxExtents.y >= PxAbs(sphereCenter.y);
		const bool bZ = boxExtents.z >= PxAbs(sphereCenter.z);
		
		const bool bInsideBox = bX && bY && bZ;// V3IsGrtrOrEq(boxExtents, V3Abs(sphereCenter));
		if (bInsideBox)//sphere center inside the box
		{
			//Pick directions and sign
			const PxVec3 absP(PxAbs(p.x), PxAbs(p.y), PxAbs(p.z));
			const PxVec3 distToSurface = boxExtents - absP;//dist from embedded center to box surface along 3 dimensions.
			
			//assume z is the smallest elemtn of the distToSurface
			PxVec3 localNorm = p.z >= 0.f ? PxVec3(0.f, 0.f, 1.f) : PxVec3(0.f, 0.f, -1.f);
			PxReal dist = -distToSurface.z;
			//find smallest element of distToSurface
			if (distToSurface.x <= distToSurface.y && distToSurface.x <= distToSurface.z)
			{
				localNorm = p.x >= 0.f ? PxVec3(1.f, 0.f, 0.f) : PxVec3(-1.0f, 0.f, 0.f);
				dist = -distToSurface.x;
			}
			else if (distToSurface.y <= distToSurface.z)
			{
				localNorm = p.y >= 0.f ? PxVec3(0.f, 1.f, 0.f) : PxVec3(0.f, -1.f, 0.f);
				dist = -distToSurface.y;
			}
		/*	printf("p(%f, %f, %f)\n", p.x, p.y, p.z);
			printf("distToSurface(%f, %f, %f)\n", distToSurface.x, distToSurface.y, distToSurface.z);
			printf("dist %f\n", dist);*/

			//separation so far is just the embedding of the center point; we still have to push out all of the radius.
			const PxVec3 normal = transform1.rotate(localNorm);
			const PxReal penetration = dist - sphereRadius;
			const PxVec3 point = sphereOrigin - normal * dist;

			outPointPen = make_float4(point.x, point.y, point.z, penetration);
			outNormal = normal;

			//printf("Inside the box\n");

			/*printf("pentration %f\n", penetration);
			printf("tP(%f, %f, %f), tQ(%f, %f, %f, %f)\n", transf1.p.x, transf1.p.y, transf1.p.z, transf1.q.x, transf1.q.y, transf1.q.z, transf1.q.w);

			printf("outPoint(%f, %f, %f), pen %f\n", outPointPen.x, outPointPen.y, outPointPen.z, outPointPen.w);
			printf("outNormal(%f, %f, %f)\n", outNormal.x, outNormal.y, outNormal.z, 0.f);*/
		}
		else
		{
			//get the closest point from the center to the box surface
			const PxReal recipLength = PxRecipSqrt(lengthSq);
			const PxReal length = 1 / recipLength;
			const PxVec3 locNorm = v * recipLength;
			const PxReal penetration = length - sphereRadius;
			const PxVec3 normal = transform1.rotate(locNorm);
			const PxVec3 point = transform1.transform(p);

			outPointPen = make_float4(point.x, point.y, point.z, penetration);
			outNormal = normal;


			
			/*printf("recipLength %f, length %f \n", recipLength, length);
			printf("locNorm(%f, %f, %f)\n", locNorm.x, locNorm.y, locNorm.z);
			printf("pentration %f\n", penetration);
			printf("tP(%f, %f, %f), tQ(%f, %f, %f, %f)\n", transf1.p.x, transf1.p.y, transf1.p.z, transf1.q.x, transf1.q.y, transf1.q.z, transf1.q.w);

			printf("outPoint(%f, %f, %f), pen %f\n", outPointPen.x, outPointPen.y, outPointPen.z, outPointPen.w);
			printf("outNormal(%f, %f, %f)\n", outNormal.x, outNormal.y, outNormal.z, 0.f);*/
			
			
	
		}
		return 1;
	}

	//printf("Not intersect! \n");

	return 0;
}

//0 is sphere, 1 is box
__device__ __forceinline__ static
PxU32 spherebox(const PxVec3& sphereOrigin, const PxTransform& transform1, const PxReal sphereRadius,
	const PxVec3 boxExtents, const PxReal contactDist, float4& outNormalPen)
{
	/*const PxVec3& sphereOrigin = transform0.p;

	const PxQuat& q1 = transform1.q;
	const PxVec3& p1 = transform1.p;

	const PxTransform transf1(p1, q1);*/

	//translate sphere center into the box space
	const PxVec3 sphereCenter = transform1.transformInv(sphereOrigin);

	const PxVec3 nBoxExtents = -boxExtents;

	const PxReal inflatedSum = sphereRadius + contactDist;
	const PxReal sqInflatedSum = inflatedSum * inflatedSum;

	const PxReal x = PxClamp(sphereCenter.x, nBoxExtents.x, boxExtents.x);
	const PxReal y = PxClamp(sphereCenter.y, nBoxExtents.y, boxExtents.y);
	const PxReal z = PxClamp(sphereCenter.z, nBoxExtents.z, boxExtents.z);
	const PxVec3 p(x, y, z);
	const PxVec3 v = sphereCenter - p;
	const PxReal lengthSq = v.dot(v);

	if (sqInflatedSum > lengthSq)//intersect
	{
		//check whether the spherCenter is inside the box
		const bool bX = boxExtents.x >= PxAbs(sphereCenter.x);
		const bool bY = boxExtents.y >= PxAbs(sphereCenter.y);
		const bool bZ = boxExtents.z >= PxAbs(sphereCenter.z);

		const bool bInsideBox = bX && bY && bZ;// V3IsGrtrOrEq(boxExtents, V3Abs(sphereCenter));
		if (bInsideBox)//sphere center inside the box
		{
			//Pick directions and sign
			const PxVec3 absP(PxAbs(p.x), PxAbs(p.y), PxAbs(p.z));
			const PxVec3 distToSurface = boxExtents - absP;//dist from embedded center to box surface along 3 dimensions.

														   //assume z is the smallest elemtn of the distToSurface
			PxVec3 localNorm = p.z >= 0.f ? PxVec3(0.f, 0.f, 1.f) : PxVec3(0.f, 0.f, -1.f);
			PxReal dist = -distToSurface.z;
			//find smallest element of distToSurface
			if (distToSurface.x <= distToSurface.y && distToSurface.x <= distToSurface.z)
			{
				localNorm = p.x >= 0.f ? PxVec3(1.f, 0.f, 0.f) : PxVec3(-1.0f, 0.f, 0.f);
				dist = -distToSurface.x;
			}
			else if (distToSurface.y <= distToSurface.z)
			{
				localNorm = p.y >= 0.f ? PxVec3(0.f, 1.f, 0.f) : PxVec3(0.f, -1.f, 0.f);
				dist = -distToSurface.y;
			}
		
			//separation so far is just the embedding of the center point; we still have to push out all of the radius.
			const PxVec3 normal = transform1.rotate(localNorm);
			const PxReal penetration = dist - sphereRadius;

			outNormalPen = make_float4(normal.x, normal.y, normal.z, penetration);

		}
		else
		{
			//get the closest point from the center to the box surface
			const PxReal recipLength = PxRecipSqrt(lengthSq);
			const PxReal length = 1 / recipLength;
			const PxVec3 locNorm = v * recipLength;
			const PxReal penetration = length - sphereRadius;
			const PxVec3 normal = transform1.rotate(locNorm);
			
			outNormalPen = make_float4(normal.x, normal.y, normal.z, penetration);
		}
		return 1;
	}

	return 0;
}

//0 is sphere, 1 is box(sphere is in the local space of box), normal is the in the local space of box
__device__ __forceinline__ static
PxU32 spherebox(const PxVec3& sphereCenter, const PxReal sphereRadius,
	const PxVec3 boxExtents, const PxReal contactDist, float4& outNormalPen)
{

	const PxVec3 nBoxExtents = -boxExtents;

	const PxReal inflatedSum = sphereRadius + contactDist;
	const PxReal sqInflatedSum = inflatedSum * inflatedSum;

	const PxReal x = PxClamp(sphereCenter.x, nBoxExtents.x, boxExtents.x);
	const PxReal y = PxClamp(sphereCenter.y, nBoxExtents.y, boxExtents.y);
	const PxReal z = PxClamp(sphereCenter.z, nBoxExtents.z, boxExtents.z);
	const PxVec3 p(x, y, z);
	const PxVec3 v = sphereCenter - p;
	const PxReal lengthSq = v.dot(v);

	if (sqInflatedSum > lengthSq)//intersect
	{
		//check whether the spherCenter is inside the box
		const bool bX = boxExtents.x >= PxAbs(sphereCenter.x);
		const bool bY = boxExtents.y >= PxAbs(sphereCenter.y);
		const bool bZ = boxExtents.z >= PxAbs(sphereCenter.z);

		const bool bInsideBox = bX && bY && bZ;// V3IsGrtrOrEq(boxExtents, V3Abs(sphereCenter));
		if (bInsideBox)//sphere center inside the box
		{
			//Pick directions and sign
			const PxVec3 absP(PxAbs(p.x), PxAbs(p.y), PxAbs(p.z));
			const PxVec3 distToSurface = boxExtents - absP;//dist from embedded center to box surface along 3 dimensions.

														   //assume z is the smallest elemtn of the distToSurface
			PxVec3 localNorm = p.z >= 0.f ? PxVec3(0.f, 0.f, 1.f) : PxVec3(0.f, 0.f, -1.f);
			PxReal dist = -distToSurface.z;
			//find smallest element of distToSurface
			if (distToSurface.x <= distToSurface.y && distToSurface.x <= distToSurface.z)
			{
				localNorm = p.x >= 0.f ? PxVec3(1.f, 0.f, 0.f) : PxVec3(-1.0f, 0.f, 0.f);
				dist = -distToSurface.x;
			}
			else if (distToSurface.y <= distToSurface.z)
			{
				localNorm = p.y >= 0.f ? PxVec3(0.f, 1.f, 0.f) : PxVec3(0.f, -1.f, 0.f);
				dist = -distToSurface.y;
			}

			//separation so far is just the embedding of the center point; we still have to push out all of the radius.
			const PxReal penetration = dist - sphereRadius;

			outNormalPen = make_float4(localNorm.x, localNorm.y, localNorm.z, penetration);

		}
		else
		{
			//get the closest point from the center to the box surface
			const PxReal recipLength = PxRecipSqrt(lengthSq);
			const PxReal length = 1 / recipLength;
			const PxVec3 locNorm = v * recipLength;
			const PxReal penetration = length - sphereRadius;
			
			outNormalPen = make_float4(locNorm.x, locNorm.y, locNorm.z, penetration);
		}
		return 1;
	}

	return 0;
}



//0 is plane, 1 is capsule
__device__ __forceinline__ static
PxU32 planeCapsule(const PxTransform& transform0, const PxTransform& transform1,
	const PxReal radius, const PxReal halfHeight, const PxReal contactDist,
	float4 *outPointPen, PxVec3& outNormal, PxReal* t = NULL)
{
	//capsule to plane
	const PxTransform bToA = transform0.transformInv(transform1);

	//in world space
	const PxVec3 planeNormal = transform0.q.getBasisVector0().getNormalized();
	//const PxVec3 contactNormal =-planeNormal;

	outNormal = -planeNormal;
	//unit x
	const PxVec3 localNormal(1.f, 0.f, 0.f);

	//printf("outNormal (%f, %f, %f)\n", outNormal.x, outNormal.y, outNormal.z);

	//capsule is in the local space of plane(n = (1.f, 0.f, 0.f), d=0.f)
	const PxVec3 basisVector = bToA.q.getBasisVector0();
	const PxVec3 tmp = basisVector * halfHeight;
	const PxVec3 s = bToA.p + tmp;
	const PxVec3 e = bToA.p - tmp;

	const PxReal inflatedRadius = radius + contactDist;
	PxU32 numContacts = 0;
	const PxReal signDist0 = s.x;//V3Dot(localNormal, s);
	if (inflatedRadius >= signDist0)
	{
		const PxVec3 worldPoint = transform0.transform(s) - planeNormal * radius;
		const PxReal pen = signDist0 - radius;
		//printf("s localPoint(%f, %f, %f) pen %f\n", localPoint.x, localPoint.y, localPoint.z, pen);
		outPointPen[numContacts] = make_float4(worldPoint.x, worldPoint.y, worldPoint.z, pen);
		numContacts++;
	}

	const PxReal signDist1 = e.x;//V3Dot(localNormal, s);
	if (inflatedRadius >= signDist1)
	{
		const PxVec3 worldPoint = transform0.transform(e) - planeNormal * radius;
		const PxReal pen = signDist1 - radius;
		//printf("e worldPoint(%f, %f, %f) pen %f\n", worldPoint.x, worldPoint.y, worldPoint.z, pen);
		//printf("e normal(%f, %f, %f)\n", outNormal.x, outNormal.y, outNormal.z);
		outPointPen[numContacts] = make_float4(worldPoint.x, worldPoint.y, worldPoint.z, pen);
		numContacts++;
	}

	if (t != NULL)
	{
		if (contactDist > 0.0f)
		{
			// *t = PxClamp(0.5f * (signDist1 - signDist0) / contactDist + 0.5f, 0.0f, 1.0f);
			*t = 0.5f + 0.5f * tanh(5.0f * (signDist1 - signDist0) / contactDist);
		}
		else
		{
			*t = signDist0 > signDist1 ? 0.0f : 1.0f;
		}
	}

	return numContacts;
}


__device__ __forceinline__ static
void pcmDistancePointSegmentTValue22(
	const PxVec3& a0, const PxVec3& b0,
	const PxVec3& a1, const PxVec3& b1,
	const PxVec3& p0, const PxVec3& p1,
	const PxVec3& p2, const PxVec3& p3,
	PxReal& t0, PxReal& t1, 
	PxReal& t2, PxReal& t3)
{
	
	const PxVec3 ap00 = p0 - a0;
	const PxVec3 ap10 = p1 - a0;
	const PxVec3 ap01 = p2 - a1;
	const PxVec3 ap11 = p3 - a1;

	const PxVec3 ab0 = b0 - a0;
	const PxVec3 ab1 = b1 - a1;

	/*	const FloatV nom00 = V3Dot(ap00, ab0);
		const FloatV nom10 = V3Dot(ap10, ab0);
		const FloatV nom01 = V3Dot(ap01, ab1);
		const FloatV nom11 = V3Dot(ap11, ab1);*/

	const PxReal nom00 = ap00.dot(ab0);
	const PxReal nom10 = ap10.dot(ab0);
	const PxReal nom01 = ap01.dot(ab1);
	const PxReal nom11 = ap11.dot(ab1);

	/*const Vec4V combinedDot = V3Dot4(ap00, ab0, ap10, ab0, ap01, ab1, ap11, ab1);
	const FloatV nom00 = V4GetX(combinedDot);
	const FloatV nom10 = V4GetY(combinedDot);
	const FloatV nom01 = V4GetZ(combinedDot);
	const FloatV nom11 = V4GetW(combinedDot);*/

	const PxReal denom0 = ab0.dot(ab0);
	const PxReal denom1 = ab1.dot(ab1);

	/*const FloatV denom0 = V3Dot(ab0, ab0);
	const FloatV denom1 = V3Dot(ab1, ab1);*/

	const PxReal eps = 1e-6f;
	t0 = (PxAbs(denom0) < eps) ? 0.f : (nom00 / denom0);
	t1 = (PxAbs(denom0) < eps) ? 0.f : (nom10 / denom0);
	t2 = (PxAbs(denom1) < eps) ? 0.f : (nom01 / denom1);
	t3 = (PxAbs(denom1) < eps) ? 0.f : (nom11 / denom1);

	/*const Vec4V nom = V4Merge(nom00, nom10, nom01, nom11);
	const Vec4V denom = V4Merge(denom0, denom0, denom1, denom1);

	const Vec4V tValue = V4Div(nom, denom);
	return V4Sel(V4IsEq(denom, zero), zero, tValue);*/
}



__device__ __forceinline__ static
PxU32 capsuleCapsule(const PxTransform& transform0, const PxTransform& transform1,
	const PxReal radius0, const PxReal halfHeight0, const PxReal radius1, const PxReal halfHeight1,
	const PxReal contactDist,
	float4 *outPointPen, PxVec3& outNormal, PxReal* outT0 = NULL, PxReal* outT1 = NULL)
{

	PxU32 numContacts = 0;

	const PxVec3 positionOffset = (transform0.p + transform1.p) * 0.5f;
	const PxVec3 p0 = transform0.p - positionOffset;
	const PxVec3 p1 = transform1.p - positionOffset;

	const PxVec3 basisVector0 = transform0.q.getBasisVector0();
	const PxVec3 tmp0 = basisVector0 * halfHeight0;
	const PxVec3 s0 = p0 + tmp0;
	const PxVec3 e0 = p0 - tmp0;
	const PxVec3 d0 = e0 - s0;

	const PxVec3 basisVector1 = transform1.q.getBasisVector0();
	const PxVec3 tmp1 = basisVector1 * halfHeight1;
	const PxVec3 s1 = p1 + tmp1;
	const PxVec3 e1 = p1 - tmp1;
	const PxVec3 d1 = e1 - s1;

	const PxReal sumRadius = radius0 + radius1; //FAdd(r0, r1);
	const PxReal inflatedSum = sumRadius + contactDist;
	const PxReal inflatedSumSquared = inflatedSum * inflatedSum; 
	const PxReal a = d0.dot(d0);//squared length of segment1
	const PxReal e = d1.dot(d1);//squared length of segment2
	const PxReal eps = 1e-6f;//FEps();

	PxReal t0, t1;
	distanceSegmentSegmentSquared(s0, d0, s1, d1, t0, t1);
	if (outT0 != NULL)
	{
		*outT0 = (1.0f - t0);
	}
	if (outT1 != NULL)
	{
		*outT1 = (1.0f - t1);
	}

	const PxVec3 closestA = s0 + d0 * t0;
	const PxVec3 closestB = s1 + d1 * t1;
	const PxVec3 vv = closestA - closestB;
	const PxReal sqDist0 = vv.dot(vv);

	if (eps > sqDist0)
	{
		const PxVec3 _normal = transform0.q.rotateInv(PxVec3(0.f, 1.f, 0.f));
		const PxVec3 normal = _normal.getNormalized();

		const PxVec3 _point = closestA - normal * radius0;
		const PxVec3 p = _point + positionOffset;
		const PxReal pen = -radius0;
		outPointPen[0] = make_float4(p.x, p.y, p.z, pen);
		//printf("This is two segment inline with each other");

		outNormal = normal;
		numContacts = 1;
	}
	else
	{ 
		if (inflatedSumSquared >= sqDist0)
		{
			//check to see whether these two capsule are paralle
			const PxReal parallelTolerance = 0.9998f;

			const PxVec3 dir0 = (eps > a) ? PxVec3(0.f) : (d0 * (1.f / PxSqrt(a)));
			const PxVec3 dir1 = (eps > e) ? PxVec3(0.f) : (d1 * (1.f / PxSqrt(e)));

			const PxReal cos = PxAbs(dir0.dot(dir1));
			if (cos > parallelTolerance)//paralle
			{
				//tV0 is s1 project to s0e0
				//tV1 is e1 project to s0e0
				//tV2 is s0 project to s1e1
				//tV3 is e0 project to s1e1
				PxReal tV0, tV1, tV2, tV3;
				pcmDistancePointSegmentTValue22(s0, e0, s1, e1,
					s1, e1, s0, e0, tV0, tV1, tV2, tV3);

				//printf("t(%f %f %f %f)\n", tV0, tV1, tV2, tV3);

				if (tV0 >= 0.f && tV0 <= 1.f)
				{
					const PxVec3 projS1 = s0 + d0 * tV0;
					const PxVec3 v = projS1 - s1;
					const PxReal sqDist = v.dot(v);
					
					const PxReal dist = PxSqrt(sqDist);
					const PxReal pen = dist - sumRadius;
					const PxVec3 normal = v / dist;// V3ScaleInv(v, dist);
					//assert(isFiniteVec3V(normal));
					const PxVec3 _p = projS1 - normal * radius0;// V3NegScaleSub(normal, r0, projS1);
					const PxVec3 p = _p + positionOffset;// V3Add(_p, positionOffset);
					outPointPen[numContacts] = make_float4(p.x, p.y, p.z, pen);
					outNormal = normal;

					//printf("_p(%f, %f, %f)\n", _p.x, _p.y, _p.z);
					//printf("p(%f, %f, %f pen %f)\n", p.x, p.y, p.z, pen);
					//printf("normal(%f, %f, %f)\n", normal.x, normal.y, normal.z);
					//storeContact(p, normal, pen, contactBuffer);
					numContacts++;
					
				}
				if (tV1 >= 0.f && tV1 <= 1.f)
				{
					const PxVec3 projE1 = s0 + d0 * tV1;
					const PxVec3 v = projE1 - e1;
					const PxReal sqDist = v.dot(v);

					const PxReal dist = PxSqrt(sqDist);
					const PxReal pen = dist - sumRadius;
					const PxVec3 normal = v / dist; // V3ScaleInv(v, dist);
					//assert(isFiniteVec3V(normal));
					const PxVec3 _p = projE1 - normal * radius0;// V3NegScaleSub(normal, r0, projE1);
					const PxVec3 p = _p + positionOffset;
					//storeContact(p, normal, pen, contactBuffer);

					outPointPen[numContacts] = make_float4(p.x, p.y, p.z, pen);
					outNormal = normal;

					//printf("p(%f, %f, %f pen %f)\n", p.x, p.y, p.z, pen);
					//printf("normal(%f, %f, %f)\n", normal.x, normal.y, normal.z);
					numContacts++;
					
				}

				if (numContacts < 2 && (tV2 >= 0.f && tV2 <= 1.f))
				{
					const PxVec3 projS0 = s1 + d1 * tV2;
					const PxVec3 v = s0 - projS0;
					const PxReal sqDist = v.dot(v);

					const PxReal dist = PxSqrt(sqDist);
					const PxReal pen = dist - sumRadius;
					const PxVec3 normal = v / dist;// V3ScaleInv(v, dist);
					//assert(isFiniteVec3V(normal));
					const PxVec3 _p = s0 - normal * radius0;// V3NegScaleSub(normal, r0, s0);
					const PxVec3 p = _p + positionOffset;
					outPointPen[numContacts] = make_float4(p.x, p.y, p.z, pen);
					outNormal = normal;

					//printf("p(%f, %f, %f pen %f)\n", p.x, p.y, p.z, pen);
					//printf("normal(%f, %f, %f)\n", normal.x, normal.y, normal.z);
					numContacts++;
					
				}

				if (numContacts < 2 && (tV3 >= 0.f && tV3 <= 1.f))
				{
					const PxVec3 projE0 = s1 + d1 * tV3;// V3ScaleAdd(d1, V4GetW(t), s1);
					const PxVec3 v = e0 - projE0;// V3Sub(e0, projE0);
					const PxReal sqDist = v.dot(v);// V3Dot(v, v);
					
					const PxReal dist = PxSqrt(sqDist);
					const PxReal pen = dist - sumRadius;
					const PxVec3 normal = v / dist;
					//assert(isFiniteVec3V(normal));
					const PxVec3 _p = e0 - normal * radius0;// V3NegScaleSub(normal, r0, e0);
					const PxVec3 p = _p + positionOffset;// V3Add(_p, positionOffset);
					
					outPointPen[numContacts] = make_float4(p.x, p.y, p.z, pen);
					outNormal = normal;

					//printf("p(%f, %f, %f pen %f)\n", p.x, p.y, p.z, pen);
					//printf("normal(%f, %f, %f)\n", normal.x, normal.y, normal.z);
					numContacts++;
					
				}
			}
			
			if(numContacts == 0)
			{
				const PxVec3 normal = vv.getNormalized();

				const PxVec3 _point = closestA - normal * radius0;
				const PxVec3 p = _point + positionOffset;

				const PxReal dist = PxSqrt(sqDist0);
				const PxReal pen = dist - sumRadius;

				outPointPen[0] = make_float4(p.x, p.y, p.z, pen);
				outNormal = normal;

				//printf("one point p(%f, %f, %f pen %f)\n", p.x, p.y, p.z, pen);
				//printf("one point normal(%f, %f, %f)\n", normal.x, normal.y, normal.z);

				numContacts = 1;
			}
		}
	}

	return numContacts;

}


#endif