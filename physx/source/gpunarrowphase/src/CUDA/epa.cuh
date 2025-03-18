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

#ifndef __EPA_CUH__
#define __EPA_CUH__

#include <assert.h>

#include "nputils.cuh"
#include "schlockShared.h"

namespace squawk
{

using namespace physx;

struct EpaScratch
{
	PxVec3 vertices[schlock::EPA_MAX_VERTS];
	PxVec3 bestDir;
	PxReal bestDist;
	PxU16 indices[schlock::EPA_MAX_VERTS];
	PxU8 loop[schlock::EPA_MAX_VERTS];
	PxU8 edges[schlock::EPA_MAX_VERTS];
	PxU8 next[schlock::EPA_MAX_VERTS]; 
};


__constant__ __device__ schlock::Vec3I8 sStartVertices4[4] =		{	MAKE_Vec3I8(0,1,2),	MAKE_Vec3I8(3,1,0),	MAKE_Vec3I8(3,2,1),	MAKE_Vec3I8(3,0,2) };
__constant__ __device__ schlock::Vec3I8 sStartAdjacencies4[4] =	{	MAKE_Vec3I8(2,3,1),	MAKE_Vec3I8(0,3,2),	MAKE_Vec3I8(0,1,3),	MAKE_Vec3I8(0,2,1) };

__constant__ __device__ schlock::Vec3I8 sStartVertices3[2] =		{	MAKE_Vec3I8(0,1,2),	MAKE_Vec3I8(1,0,2) };
__constant__ __device__ schlock::Vec3I8 sStartAdjacency3[2] =	{	MAKE_Vec3I8(1,1,1),	MAKE_Vec3I8(0,0,0)	};



typedef int VMapType;

__device__ inline bool replaceTriangle(EpaScratch& epaS, int nbVertices, int triIndex,
												PxReal lowerBound, PxVec3& normal, PxReal& distance,  
												schlock::Vec3I8& v, schlock::Vec3I8& a, int& liveB, const PxReal releps)
{
	// to make it easier to keep track, we use a 'B' suffix for a bitmap where each bit represents a lane (and therefore a triangle)
	int tI = threadIdx.x, laneB = 1<<tI;			
	//PxVec3 apex = ldS(S.vertices[nbVertices-1]);
	PxVec3 apex = epaS.vertices[nbVertices - 1];

	// triangles that are silhouetted by the new vertex are threatened, but not yet killed
	bool threatened = liveB&laneB && (normal.dot(apex) - distance >= /*-*/releps);
	schlock::EPABitMap threatenedB = __ballot_sync(FULL_MASK, threatened);

	if (threatenedB == 0 || threatenedB == liveB || !(threatenedB & (1<<triIndex)))
	{
#if EPA_LOG_DEGENERATE_CASES
		if(threadIdx.x == 0)
			printf("degenrate: threatenedB %d liveB %d < tri %d \n", threatenedB, liveB, tri);
#endif

		return false;
	}

	// the threatened set is not guaranteed connected, so flood fill to find killB, a bitmap of tris to remove
	schlock::EPABitMap b0B = 1<<a.get(0), b1B = 1<<a.get(1), b2B = 1<<a.get(2);
	// nbr = threatened neighbors of threatened triangles
	schlock::EPABitMap killB = 1<<triIndex, nbr = (b0B | b1B | b2B) & (threatened?threatenedB:0);

	// flood the threatened triangles to find the killed ones	
	//for(schlock::EPABitMap pB = 0; pB != killB; pB = killB, killB |= __ballot(nbr&killB));

	for (schlock::EPABitMap pB = 0; pB != killB;)
	{
		pB = killB;
		killB |= __ballot_sync(FULL_MASK, nbr&killB);
	}

	//every triangles aren't alive and the triangles we want to kill in this iteration
	schlock::EPABitMap deadB = (~liveB) | killB;

	// d0, d1, d2 denote for each triangle whether the corresponding edge is a silhouette edge
	const int alive = laneB&(~deadB);
	const int d0 = alive && (b0B&deadB);
	const int d1 = alive && (b1B&deadB);
	const int d2 = alive && (b2B&deadB);
	
	// write the silhouette edges into the loop buffer
	int v0 = v.get(0), v1 = v.get(1), v2 = v.get(2), z = 0;			

	assert(!alive || v0 < schlock::EPA_MAX_VERTS);
	assert(!alive || v1 < schlock::EPA_MAX_VERTS);
	assert(!alive || v2 < schlock::EPA_MAX_VERTS);

	// S.edges is the adjacency for the new cone tri on this edge, and
	// also where the cone triangle will write back its index for later fixup
	if (d0) 
		z = v2, epaS.next[v2] = v1, epaS.edges[v2] = tI;
	if (d1) 
		z = v0, epaS.next[v0] = v2, epaS.edges[v0] = tI;
	if (d2) 
		z = v1, epaS.next[v1] = v0, epaS.edges[v1] = tI;

	int d0d1d2B = __ballot_sync(FULL_MASK, d0|d1|d2);
	
	if (!d0d1d2B)
	{
#if EPA_LOG_DEGENERATE_CASES
		if (threadIdx.x == 0)
			printf("degenrate: d0d1d2B %d b0B %d b1B %d b2B %d deadB %d \n", d0d1d2B, b0B, b1B, b2B, deadB);
#endif
	
		return false;
	}

	int loopStart = __shfl_sync(FULL_MASK, z, lowestSetIndex(d0d1d2B));		// pick a loop vert from the lowest indexed triangle that has one
	int nbLoop = 0;
	
	__syncwarp();

	if(tI == 0) 
	{
		assert(loopStart < schlock::EPA_MAX_VERTS);
		
		epaS.loop[nbLoop++] = loopStart;									// flatten the linked list into a vertex array							
		for(int vertex = epaS.next[loopStart]; vertex!=loopStart; vertex = epaS.next[vertex])	
		{
			if(vertex >= schlock::EPA_MAX_VERTS)
			{
#if EPA_LOG_DEGENERATE_CASES
				printf("degenrate: vertex %d\n", vertex);
#endif

				nbLoop = -1;
				break;
			}

			if(nbLoop >= schlock::EPA_MAX_VERTS - 1)
			{
#if EPA_LOG_DEGENERATE_CASES
				printf("degenrate: nbLoop %d\n", nbLoop);
#endif

				nbLoop = -1;
				break;
			}
			
			epaS.loop[nbLoop++] = vertex;
		}

#if EPA_REPLACE_TRIANGLE_LOGGING
		for(int i=0;i<nbLoop;i++)
			printf("%d, ", S.loop[i]);
		printf("(starting at %d)\n",loopStart);
#endif
	}

		
	nbLoop = __shfl_sync(FULL_MASK, nbLoop, 0);
	
	if(nbLoop != __popc(__ballot_sync(FULL_MASK, d0)) + __popc(__ballot_sync(FULL_MASK, d1)) + __popc(__ballot_sync(FULL_MASK, d2)))
	{
#if EPA_LOG_DEGENERATE_CASES
		{
			printf("nbLoop %d __ballot(d0) %d __ballot(d1) %d __ballot(d2) %d tid %d", nbLoop, __ballot(d0), __ballot(d1), __ballot(d2), threadIdx.x);
		}
#endif
		return false;
	}
	
	__syncwarp();

	// make the cone with the nbLoop least non-survivors 
	int coneIndex = __popc(deadB&(laneB-1)), coneB = __ballot_sync(FULL_MASK, deadB&laneB && coneIndex < nbLoop);
	bool fail = false;
	if(coneB & laneB)
	{ 
		assert(coneIndex < nbLoop);
		assert(nbLoop < schlock::EPA_MAX_VERTS);

		int v0 = nbVertices-1, v1 = epaS.loop[coneIndex], v2 = epaS.loop[coneIndex+1 == nbLoop ? 0 : coneIndex+1];
		v.setInt(MAKE_Vec3I8(v0, v1, v2));
		
		assert(v0 < schlock::EPA_MAX_VERTS);
		assert(v1 < schlock::EPA_MAX_VERTS);
		assert(v2 < schlock::EPA_MAX_VERTS);

		a.set(0, epaS.edges[v1]);
		a.set(1, lowestSetIndex(coneB & ~(coneIndex+1 == nbLoop ? 0 : 2*laneB-1)));		// previous bit in the active mask, wrapping around
		a.set(2, highestSetIndex(coneB & ((coneIndex==0 ? 0 : laneB)-1)));				// next bit in the active mask, wrapping around


		epaS.edges[v1] = tI;																// write our index back into the buffer for later fixup

		//PxVec3 p0 = ldS(S.vertices[v0]), p1 = ldS(S.vertices[v1]), p2 = ldS(S.vertices[v2]);	
		PxVec3 p0 = epaS.vertices[v0], p1 = epaS.vertices[v1], p2 = epaS.vertices[v2];
		PxVec3 e01 = p1 - p0, e02 = p2 - p0;
		PxVec3 n;
		
		if (e01.magnitudeSquared() < e02.magnitudeSquared())
		{
			n = e01.cross(p2 - p1);
		}
		else
		{
			n = e02.cross(p2 - p1);
		}

		PxReal nm = n.magnitude();
		normal = n/nm;
		distance = normal.dot(p0);
		/*fail = nm == 0 || (lowerBound > 0.0f && distance < lowerBound);
		if (fail)
		{
			printf("fail nm %f\ lowerBound %f distance %f \n", nm, lowerBound, distance);
		}*/
		fail = nm == 0 || (lowerBound > 0.0f && (distance - lowerBound) < -1e-4f);
	}

	__syncwarp();

	// ... to fix up the silhouette adjacency with the new tris
	if(d0) a.set(0, epaS.edges[v.get(2)]);													
	if(d1) a.set(1, epaS.edges[v.get(0)]);
	if(d2) a.set(2, epaS.edges[v.get(1)]);

	liveB = (liveB & ~deadB) | coneB;
	return __ballot_sync(FULL_MASK, fail) == 0;
}

		
__device__ inline void epaStart4(EpaScratch& S,
										  const schlock::GjkCachedData* data, 
										  schlock::Vec3I8& a, schlock::Vec3I8& v,
										  PxVec3& normal,
										  PxReal& distance)
{
	int tI = threadIdx.x;
	PxReal input = reinterpret_cast<const PxReal*>(data)[tI];
	if(tI<16)
	{
		if((tI&3)<3)
			(reinterpret_cast<volatile PxReal*>(S.vertices))[tI-tI/4] = input;
		else
			S.indices[tI>>2] = __float_as_int(input);
	}

	PxReal d = dot(input, input), m = __shfl_sync(FULL_MASK, 1/sqrt(d), tI*4+16);

	normal.x = __shfl_sync(FULL_MASK, input, tI*4+16) * m;
	normal.y = __shfl_sync(FULL_MASK, input, tI*4+17) * m;
	normal.z = __shfl_sync(FULL_MASK, input, tI*4+18) * m;
	distance = __shfl_sync(FULL_MASK, input, tI*4+19) * m;
	if(tI<4)
		a = sStartAdjacencies4[tI], v = sStartVertices4[tI];
}


__device__ inline PxU8 supportIndex(const PxVec3* v, PxU8 nbVertices, const PxVec3& dir)
{
	int bestN = 0xFFFFFFFF;
	PxReal bestDist = -FLT_MAX;
	for(PxU32 i=0;i<nbVertices;i+=32)
	{
		int n = i+threadIdx.x;
		PxReal d = n < nbVertices ? dir.dot(v[n]) : -FLT_MAX, e;

		e = fmaxf(d, __shfl_xor_sync(FULL_MASK, d, 16));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 8));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 4));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 2));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 1));

		if(e>bestDist)
			bestDist = e, bestN = i+__ffs(__ballot_sync(FULL_MASK, e == d))-1;
	}

	assert(bestN < nbVertices);
	return PxU8(bestN);
}

// find a new support point in either the positive or negative direction furthest from the reference point
// we could be a bit more efficient here but this is a super-rare case
__device__ inline void newPoint(const PxVec3* vA, PxU32 nbA, const PxVec3* vB, PxU32 nbB, const PxVec3& dir, const PxVec3& ref, volatile PxVec3& vertex, volatile PxU16& indices)
{
	PxU8 indexAp = supportIndex(vA, nbA, dir), indexBp = supportIndex(vB, nbB, -dir);	// support point in +ve dir
	PxU8 indexAn = supportIndex(vA, nbA, -dir), indexBn = supportIndex(vB, nbB, dir);	// support point in +ve dir
	PxVec3 vp = vA[indexAp] - vB[indexBp], vn = vA[indexAn] - vB[indexBn];		

	if(threadIdx.x == 0)
	{
		if(PxAbs((vp-ref).dot(dir)) > PxAbs((vn-ref).dot(dir)))
		{
			stS(vertex, vp);
			indices = indexBp<<8 | indexAp;
		}
		else
		{
			stS(vertex, vn);
			indices = indexBn<<8 | indexAn;
		}
	}
}

// find a new support point in either the positive or negative direction furthest from the reference point
// we could be a bit more efficient here but this is a super-rare case
			
__device__ inline void epaStart123(volatile EpaScratch& S,
											const schlock::GjkCachedData* data,
											const PxVec3* vA, PxU32 nbA,
											const PxVec3* vB, PxU32 nbB,
											schlock::Vec3I8& a, schlock::Vec3I8& v,
											PxVec3& normal,
											PxReal& distance,
											PxU32 warpMask)
{
	int tI = threadIdx.x;
	PxReal input = reinterpret_cast<const PxReal*>(data)[tI];
	if(tI<16)
	{
		if((tI&3)<3)
			(reinterpret_cast<volatile PxReal*>(S.vertices))[tI-tI/4] = input;
		else
			S.indices[tI>>2] = __float_as_int(input);
	}

	__syncwarp(warpMask); //S.vertices is written above and read below

	PxVec3 vertex0 = ldS(S.vertices[0]);

	if(data->size<2)
		newPoint(vA, nbA, vB, nbB, PxVec3(1,0,0), ldS(S.vertices[0]), S.vertices[1], S.indices[1]);

	__syncwarp(warpMask); //S.vertices is written (inside newPoint) above and read below
	PxVec3 vertex1 = ldS(S.vertices[1]);

	if(data->size<3)
	{
		PxVec3 dir = vertex1 - vertex0, s = (PxAbs(dir.x) < PxAbs(dir.y) ? PxVec3(1,0,0) : PxVec3(0,1,0)).cross(dir);
		newPoint(vA, nbA, vB, nbB, s, vertex0, S.vertices[2], S.indices[2]);
	}

	__syncwarp(warpMask); //S.vertices is written (inside newPoint) above and read below
	PxVec3 vertex2 = ldS(S.vertices[2]);

	PxVec3 e01 = vertex1 - vertex0, e12 = vertex2 - vertex1, e02 = vertex2 - vertex0;

	if (e01.magnitude()<=e02.magnitude() )
	{
		normal = e01.cross(e12).getNormalized();
	}
	else
	{
		normal = e02.cross(e12).getNormalized();
	}

	distance = normal.dot(vertex0);
		
	if(tI < 2)
		v = sStartVertices3[tI], a = sStartAdjacency3[tI];
	if(tI==1)
		normal = -normal, distance = -distance;
}


__device__ inline schlock::GjkResult::Enum epa(EpaScratch& epaS,
								const PxVec3* vA, PxU32 nbA, 
								const PxVec3* vB, PxU32 nbB, 
								const schlock::GjkCachedData* input,
								PxReal convergenceRatio, 
								PxReal relEpsBase,
								schlock::GjkOutput& output)
{
	PxReal releps = relEpsBase * (1-convergenceRatio);

	schlock::Vec3I8 a, v;					// per-triangle adjancencies and vertex indices
	PxVec3 normal;					// per-triangle normal
	PxReal distance;				// per-triangle distance
	schlock::Vec3I8 bestVerts;

	PxReal lowerBound = -PX_MAX_F32, upperBound = PX_MAX_F32;
	int nbVertices, liveTriangles;

	bool b = input->size == 4;
	PxU32 warpMask = __ballot_sync(FULL_MASK, !b);

	if(b)
	{
		epaStart4(epaS, input, a, v, normal, distance);
		nbVertices = 4;
		//This is a tetrahedron so there are 4 triangles, hence liveTriangles = 0xf (1111)
		liveTriangles = 0xf;
	}
	else
	{
		epaStart123(epaS, input, vA, nbA, vB, nbB, a, v, normal, distance, warpMask);
		nbVertices = 3;
		//This is a double-sided triangle so there are 2 triangles, hence liveTriangles = 0x3 (11)
		liveTriangles = 0x3;
	}

	__syncwarp();
			
	// TODO: check here for vertex count is slighly conservative
	bool nondegenerate = true;
	while (nondegenerate && nbVertices < schlock::EPA_MAX_VERTS)
	{
		PxReal t = liveTriangles & 1 << threadIdx.x ? distance : FLT_MAX, e;
		//get the shortest distance triangle
		e = fminf(t, __shfl_xor_sync(FULL_MASK, t, 16));
		e = fminf(e, __shfl_xor_sync(FULL_MASK, e, 8));
		e = fminf(e, __shfl_xor_sync(FULL_MASK, e, 4));
		e = fminf(e, __shfl_xor_sync(FULL_MASK, e, 2));
		e = fminf(e, __shfl_xor_sync(FULL_MASK, e, 1));

		//try to find out which triangle index has the shortest distance
		int f = __ballot_sync(FULL_MASK, e == t);
		assert(f); //Trigger assert if e == t failed on all lanes (indicates a nan)
		int triIndex = lowestSetIndex(f);

		//get the normal from the shortest distance triangle
		PxVec3 d(__shfl_sync(FULL_MASK, normal.x, triIndex), __shfl_sync(FULL_MASK, normal.y, triIndex), __shfl_sync(FULL_MASK, normal.z, triIndex));
		//get the support vertex
		PxU8 indexA = squawk::supportIndex(vA, nbA, d), indexB = squawk::supportIndex(vB, nbB, -d);
		PxVec3 supportVertex = vA[indexA] - vB[indexB];

		//calculate the distance from the origin to the support vertex
		PxReal thisFrameUb = supportVertex.dot(d);

		//found the MTD, epa terminate
		if (abs(thisFrameUb - e) <= releps)
		{
			upperBound = thisFrameUb, lowerBound = e;
			bestVerts.setInt(__shfl_sync(FULL_MASK, v.getInt(), triIndex));

			if (threadIdx.x == 0)
				epaS.bestDir = d;

			break;
		}

		// all remaining tris are worse than the best candidate, epa terminate
		if (e > upperBound)
			break;


		PxReal ub = PxMin(upperBound, thisFrameUb);

		if (ub < upperBound)
		{
			upperBound = ub, lowerBound = e;
			bestVerts.setInt(__shfl_sync(FULL_MASK, v.getInt(), triIndex));

			if (threadIdx.x == 0)
				epaS.bestDir = d;
		}

#if EPA_LOGGING
		if (threadIdx.x == 0)
			printf("[%f, %f], (%f, %f, %f)\n", lowerBound, upperBound, supportVertex.x, supportVertex.y, supportVertex.z);
#endif

		// save off the new vertex
		if (threadIdx.x == 0)
		{
			epaS.vertices[nbVertices] = supportVertex;
			epaS.indices[nbVertices] = PxU16(indexB) << 8 | PxU16(indexA);
		}

		__syncwarp();

		nondegenerate = replaceTriangle(epaS, ++nbVertices, triIndex, e, normal, distance, v, a, liveTriangles, releps);
	}

#if EPA_LOGGING
		if(!nondegenerate)
			printf("degenerate\n");
#endif

	output.degenerate = !nondegenerate;

	__syncwarp();

	schlock::GjkResult::Enum result;
	PxVec3 bestDir = epaS.bestDir;
	PxVec3 closestPoint = bestDir * lowerBound;

	if(threadIdx.x == 0)
	{
		output.direction = bestDir;
		output.lowerBound = lowerBound;
		output.upperBound = upperBound;

		int v0 = bestVerts.get(0), v1 = bestVerts.get(1), v2 = bestVerts.get(2);
		PxReal u, v, w;
		schlock::barycentrics(epaS.vertices[v0], epaS.vertices[v1], epaS.vertices[v2], u, v, w, closestPoint);
		PxVec3 cpA = vA[schlock::aIndex(epaS.indices[v0])] * u + vA[schlock::aIndex(epaS.indices[v1])] * v + vA[schlock::aIndex(epaS.indices[v2])] * w;
		output.closestPointA = cpA;
		output.closestPointB = cpA - closestPoint;
	}
	result = lowerBound < 0 ? schlock::GjkResult::eCLOSE : schlock::GjkResult::eOVERLAP;

	return result;
}
}


#endif
