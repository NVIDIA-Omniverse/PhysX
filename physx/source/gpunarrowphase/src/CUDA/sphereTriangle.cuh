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

#ifndef __SPHERE_TRIANGLE_CUH__
#define __SPHERE_TRIANGLE_CUH__

//#include "shuffle.cuh"
#include "nputils.cuh"
#include "dataReadWriteHelper.cuh"
#include "convexNpCommon.h"
#include "cudaNpCommon.h"

#include "triangle.cuh"
#include "stdio.h"

__device__ static PxReal distancePointTriangleSquared(const PxVec3 p,
	const PxVec3 a,
	const PxVec3 b,
	const PxVec3 c,
	const uint4 adjIdxs,
	PxVec3& closestP,
	bool& generateContact,
	bool& faceContact,
	PxU32* mask=NULL)
{
	faceContact = false;

	// Check if P in vertex region outside A
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 ap = p - a;
	const PxReal d1 = ab.dot(ap);
	const PxReal d2 = ac.dot(ap);
	if (d1 < 0.0f && d2 < 0.0f)
	{
		if(mask)
			*mask = PxU32(ConvexTriIntermediateData::eV0);
		generateContact = (!isEdgeNonconvex(adjIdxs.x)) || (!isEdgeNonconvex(adjIdxs.z));
		closestP = a;
		return ap.magnitudeSquared();	// Barycentric coords 1,0,0
	}
	// Check if P in vertex region outside B
	const PxVec3 bp = p - b;
	const PxReal d3 = ab.dot(bp);
	const PxReal d4 = ac.dot(bp);
	if (d3 > 0.0f && d4 < d3)
	{
		if(mask)
			*mask = PxU32(ConvexTriIntermediateData::eV1);
		generateContact = (!isEdgeNonconvex(adjIdxs.x)) || (!isEdgeNonconvex(adjIdxs.y));
		closestP = b;
		return bp.magnitudeSquared();	// Barycentric coords 0,1,0
	}
	// Check if P in vertex region outside C
	const PxVec3 cp = p - c;
	const PxReal d5 = ab.dot(cp);
	const PxReal d6 = ac.dot(cp);
	if (d6 > 0.0f && d5 < d6)
	{
		if (mask)
			*mask = PxU32(ConvexTriIntermediateData::eV2);
		generateContact = (!isEdgeNonconvex(adjIdxs.y)) || (!isEdgeNonconvex(adjIdxs.z));
		closestP = c;
		return cp.magnitudeSquared();	// Barycentric coords 0,0,1
	}

	PxReal vc = d1 * d4 - d3 * d2;
	PxReal vb = d5 * d2 - d1 * d6;
	PxReal va = d3 * d6 - d5 * d4;

	const float edgeTol = (va + vb + vc) * 1e-5;

	// Check if P in edge region of AB, if so return projection of P onto AB
	if (vc < 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		if(mask)
			*mask = PxU32(ConvexTriIntermediateData::eE01);
		generateContact = !isEdgeNonconvex(adjIdxs.x) || vc > -edgeTol;
		const PxReal v = d1 / (d1 - d3);
		closestP = a + v * ab;	// barycentric coords (1-v, v, 0)
		return (closestP - p).magnitudeSquared();
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC	
	if (vb < 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		if(mask)
			*mask = PxU32(ConvexTriIntermediateData::eE02);
		generateContact = !isEdgeNonconvex(adjIdxs.z) || vb > -edgeTol;
		const PxReal w = d2 / (d2 - d6);
		closestP = a + w * ac;	// barycentric coords (1-w, 0, w)
		return (closestP - p).magnitudeSquared();
	}

	// Check if P in edge region of BC, if so return projection of P onto BC	
	if (va < 0.0f && d4 >= d3 && d5 >= d6)
	{
		if(mask)
			*mask = PxU32(ConvexTriIntermediateData::eE12);
		generateContact = !isEdgeNonconvex(adjIdxs.y) || va > -edgeTol;
		const PxReal w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closestP = b + w * (c - b);	// barycentric coords (0, 1-w, w)
		return (closestP - p).magnitudeSquared();
	}

	generateContact = true;
	faceContact = true;

	if(mask)
		*mask = 0;

	// P inside face region. Compute Q through its barycentric coords (u,v,w)	
	closestP = a + (ab * vb + ac * vc)/ (va + vb + vc);
	return (closestP - p).magnitudeSquared();
}

__device__ inline PxReal scalarTriple(const PxVec3& a, const PxVec3& b, const PxVec3& c)
{
	return a.cross(b).dot(c);
}

// intersects a line (through points p and q, against a triangle a, b, c - mostly taken from Real Time Collision Detection - p186
__device__ inline bool intersectLineTri(const PxVec3& p, const PxVec3& q, const PxVec3& a, const PxVec3& b, const PxVec3& c)
{
	const PxVec3 pq = q - p;
	const PxVec3 pa = a - p;
	const PxVec3 pb = b - p;
	const PxVec3 pc = c - p;

	PxVec3 m = pq.cross(pc);
	PxReal u = pb.dot(m);
	PxReal v = -pa.dot(m);
	PxReal w = scalarTriple(pq, pb, pa);

	return u >= 0.f && v >= 0.f && w >= 0.f;
}

#endif
