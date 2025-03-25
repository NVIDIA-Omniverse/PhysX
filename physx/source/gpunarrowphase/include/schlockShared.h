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

#ifndef GU_SCHLOCKSHARED_H
#define GU_SCHLOCKSHARED_H

#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxTransform.h"
#include "geometry/PxMeshScale.h"

#include <assert.h>
#include <stdio.h>

#define GJK_LOGGING 0
#define EPA_REPLACE_TRIANGLE_LOGGING 0
#define EPA_LOGGING 0
#define EPA_LOG_DEGENERATE_CASES 0

#if GJK_LOGGING
#define GJK_LOG(...) if(gjkDebug) printf(__VA_ARGS__)
#else
#define GJK_LOG(...)
#endif


namespace schlock
{

extern bool epaDebug;

using namespace physx;

template<typename T>
PX_CUDA_CALLABLE PX_FORCE_INLINE void swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 encodeIndices(PxU8 a, PxU8 b)		{ return 0x3f800000u | b<<8 | a;}
PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 aIndex(PxU32 indices)					{ return indices&0xff; }
PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 bIndex(PxU32 indices)					{ return (indices>>8)&0xff; }

struct CachedVertex
{
	PxVec3 v;
	PxU32 indices;
	PX_FORCE_INLINE PX_CUDA_CALLABLE CachedVertex() {}
	PX_FORCE_INLINE PX_CUDA_CALLABLE CachedVertex(PxU8 iA, PxU8 iB, const PxVec3& iV): v(iV), indices(encodeIndices(iA,iB)) {}
	PX_FORCE_INLINE PX_CUDA_CALLABLE CachedVertex(const CachedVertex& other): v(other.v), indices(other.indices) {}
	PX_FORCE_INLINE PX_CUDA_CALLABLE void operator=(const CachedVertex& other)	{	v = other.v, indices = other.indices; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 a() const { return aIndex(indices); }
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 b() const { return bIndex(indices); }

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 a() const volatile { return aIndex(indices); }
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 b() const volatile { return bIndex(indices); }
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxVec3 getV() const volatile { return PxVec3(v.x, v.y, v.z); }
};

struct GjkCachedData
{
	CachedVertex vertices[4];
	struct
	{
		PxVec3 n;
		PxReal d;
	} normals[4];

	PxU32 size;

	PX_FORCE_INLINE PX_CUDA_CALLABLE GjkCachedData(){}

	__device__ void Reset() { size = 0; __syncwarp(); }
};

struct GjkResult
{
	enum Enum
	{
		eDISTANT,
		eCLOSE,
		eOVERLAP
	};
};

struct GjkOutput
{
	PxVec3 direction;				// unit witness for lower bound
	PxReal lowerBound;				// provable lower bound
	PxReal upperBound;				// provable upper bound

	PxVec3 closestPointA;			// witnesses for upper bound
	PxVec3 closestPointB;

	bool degenerate;
	PxVec3 closestPointDir;
};

PX_CUDA_CALLABLE PX_FORCE_INLINE void barycentrics(const PxVec3& a, const PxVec3& b, const PxVec3& c, float &u, float &v, float &w, const PxVec3& p)
{
    PxVec3 m = (b - a).cross(c - a), ma = m.abs(), a1 = (b-p).cross(c-p), a2 = (c-p).cross(a-p);
    PxReal d;

    // area ratios in plane with largest normal component
    if (ma.x >= ma.y && ma.x >= ma.z) 
		d = 1.0f/m.x, u = a1.x * d,  v = a2.x * d;
    else if (ma.y >= ma.z)
		d = 1.0f/m.y, u = a1.y * d,  v = a2.y * d;
	else
		d = 1.0f/m.z, u = a1.z * d,  v = a2.z * d;

    w = 1.0f - u - v;
}


// top-level functions

// unit-level functions for testing
static const PxU32 EPA_MAX_VERTS = 18, EPA_MAX_FACES = 2*EPA_MAX_VERTS-4;		// for N verts, we'll have 2(N-2) faces. 

typedef int EPABitMap;

#define MAKE_Vec3I8(A, B, C) (physx::PxU32(A) | (physx::PxU32(B) << 8) | (physx::PxU32(C) << 16))

struct Vec3I8
{
	PxU32 a;
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU8 get(PxU32 i)	const			{ return (a >> (i << 3)) & 0xFFul; }
	PX_FORCE_INLINE PX_CUDA_CALLABLE void set(PxU32 i, PxU8 v)			{ a = (a & ~(0xFFul << (i << 3))) | (PxU32(v) << (i << 3)); }
	PX_FORCE_INLINE PX_CUDA_CALLABLE void setInt(int i)					{ a = i; }
	PX_FORCE_INLINE PX_CUDA_CALLABLE int getInt()						{ return a; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE const Vec3I8& operator=(const Vec3I8& other)						{a = other.a; return *this;}

	bool operator==(const Vec3I8& other) { return (a & 0xFFffFFul) == (other.a & 0xFFffFFul); }
};

struct EPATriangle
{
	PxVec3 normal;			// unit normal
	PxReal distance;		// *plane* distance from origin
	Vec3I8 v;				// vertices
};

struct EPAHullData
{
	PxVec3				v[EPA_MAX_VERTS];
	PxU32				i[EPA_MAX_VERTS];
	EPATriangle			t[EPA_MAX_FACES];
	Vec3I8				a[EPA_MAX_FACES];

	PxU8				nbVertices;
	EPABitMap			liveMap;
};


// GJK routine. 
//
// Input:
// convexA, convexB: two convexes in the same space
// initialDir: an unnormalized start direction for GJK iteration
// minSep: the minimum distance at which the two objects are judged to be separated. Must be positive.
// convergenceRatio: minimum ratio of lowerBound/upperBound for convergence (a number close to, but less than, 1)
//
// returns DISTANT if a lower bound on distance is found that is > minSep
//         OVERLAP if the objects are provably overlapping
//         else CLOSE.
// 
// for OVERLAP, the cachedData (vertex indices & search directions) can be passed to EPA
// for DISTANT, a direction is generated which provides a witness for the lower bound
// for CLOSE, a pair of closest points is generated, which provide (in MS space) a witness for the upper bound, and
// if nondegenerate, also a direction which witnesses the lower bound
//
// the degenerate flag is raised in the output if the algorithm encountered geometric instability while the minSep value is in [lower, upper].
// In particular, we are not overlapping (lower>minSep) nor overlapping (upper == 0), so this can only accompany a result of CLOSE. 
// geometric instability can only happen very close to convergence, so unless very high tolerances are necessary, convergence can be assumed.
// in this case if the lower bound is zero, then the the lower bound witness direction is not generated. 
// So when CLOSE is returned, the upper bound should be used as the authoritative distance, and the direction should be taken from the closest points

// EPA routine. 
//
// Input:
// convexA, convexB: two convexes in the same space
// cache: a cache of initial vertices produced by GJK. Must have size at least 2
// convergenceRatio: minimum ratio of lowerBound/upperBound for convergence (a number close to, but less than, 1)
//
// on output, the following are emitted:
// * a pair of closest points which are the best depenetration found in the allowed space
// * a lower bound and upper bound on penetration depth.
// * a direction corresponding to the lower bound (this direction *opposes* closestB - closestA), so if we translate object B by lowerBound * direction, we 
//   should approximately separate the shapes
// 
// the degenerate flag is raised in the output if the algorithm encountered geometric instability before convergence, or ran out of space.
// in this case, the bounds, direction and closest points are still generated. 
//
// The algorithm returns GjkResult::eOVERLAP in almost all cases. However, it's possible for EPA to find separation even if GJK found penetration,
// and in this anomalous case it returns GjkResult::eCLOSE
//
// // There is currently no direction for the upper bound, which would allow for reliable (if not minimal) separation in degenerate cases

}
#endif
