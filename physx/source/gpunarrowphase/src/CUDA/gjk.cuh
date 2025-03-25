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

#ifndef __GJK_CUH__
#define __GJK_CUH__

namespace squawk
{
// The simplex is stored as 4 vectors and then 4 normals (so we can shoot it straight into EPA)
// The 3 component of each vector contains 3f80 in the high bits (so it's a valid float) and 
// the vertex indices for a and b in the lower bits

typedef PxReal Simplex;

static __device__ inline Simplex setNormal0(Simplex z, PxReal nV, PxU32 index)
{
	PxReal tmpV = splatV(index, nV);
	return threadIdx.x < 16 || threadIdx.x > 19 ? z : tmpV;
}

static __device__ inline Simplex setNormal123(Simplex z, PxReal nV)
{
	PxReal tmpV = __shfl_up_sync(FULL_MASK, nV, 20);
	return threadIdx.x < 20 ? z : tmpV;
}

static __device__ inline Simplex loadSimplex(const schlock::GjkCachedData& data)
{
	return reinterpret_cast<const PxReal*>(&data)[threadIdx.x];
}

static __device__ inline void storeSimplex(schlock::GjkCachedData& data, Simplex z)
{
	reinterpret_cast<PxReal*>(&data)[threadIdx.x] = z;
}

static __device__ inline void storeSimplex(volatile schlock::GjkCachedData& data, Simplex z)
{
	reinterpret_cast<volatile PxReal*>(&data)[threadIdx.x] = z;
}

// suffixes for vector warps: 
// * V = 3 vectors: (aX,aY,aZ,_,bX,bY,bZ,_,cX,cY,cZ,_)
// * D = dense scalar: (a,a,a,a,b,b,b,b,c,c,c,c)
// * S = sparse scalar: (a,_,_,_,b,_,_,_,c,_,_,_)
// * I = index vector for shuffle

#define DEGENERATE 0xffff

static __device__ inline PxU32 closestPointTri(Simplex &zV, PxReal cV, PxReal& closestPoint)
{
	PxU32 tI = threadIdx.x, cp0I = (tI&28)|next3(tI&3), cp1I = (tI&28)|prev3(tI&3);		// cross product indices

	closestPoint = 0;
	PxReal eV = cV - zV, lD = splatX(dot(eV, eV));										// edges to apex & squared lengths
	if(anyX<2>(lD==0))																	
		return DEGENERATE;																// FAIL: apex coincident with a base vert

	PxReal tmp = __shfl_xor_sync(FULL_MASK, eV, 4), shortEdgeV = __shfl_xor_sync(FULL_MASK, lD, 4) < lD ? tmp : eV;			// shorter edge to apex
	PxReal mV = cross(__shfl_xor_sync(FULL_MASK, zV,4)-zV, shortEdgeV, cp0I, cp1I);						// (normal, -normal)
	
	// lots of dot products here with cV, (plus proj below). TODO: optimize
	PxReal pS = dot(eV,cV);																// projection of origin onto edges
	bool uS = dot(cross(mV, eV, cp0I, cp1I), cV) <= 0;									// origin outside edge planes
	
	if(!anyX<2>(uS))																	// REGION: face - inside both edge planes
	{
		PxReal projD = __shfl_sync(FULL_MASK, dot(mV,cV),0);												// project onto face
		closestPoint = splatV(0, mV*projD/splatX(dot(mV,mV)));								
		bool flip = projD<0;															// ensure the normal winding makes it outward-facing in the tet
		mV = (tI & 3) == 3 ? projD : mV; 
		zV = setNormal0(zV, mV, flip);
		return flip ? 0x0123u : 0x1023u;
	}

	PxU32 eid = __ffs(packXFlags<2>(uS & bool(pS>0)));									// REGION: edges - outside the edge plane, and projects onto the edge
	if(eid--)																			// __ffs returns 0 for no bit, 1 for lsb
	{
		closestPoint = splatV(eid, cV - eV*splatX(pS)/lD);
		return eid<<8 | 0x22;
	}

	closestPoint = cV;																	// REGION: apex - not an edge, not a face
	return 0x0021;
}


static __device__ inline PxU32 closestPointTet(Simplex &zV, PxReal dV, PxReal& closestPoint)
{
	closestPoint = 0;

	PxReal n0V = splatV(4, zV);
	if(__shfl_sync(FULL_MASK, dot(n0V, dV) > dot(n0V, zV), 0))											// FAIL: new vertex is beyond old face (TODO: set up normal in UT)
		return DEGENERATE;
	
	PxU32 tI = threadIdx.x, cp0I = (tI&28)|next3(tI&3), cp1I = (tI&28)|prev3(tI&3);		// cross product indices
	PxU32 vNextI = tI>7 ? tI-8 : tI+4, vPrevI = tI<4 ? tI + 8 : tI-4;					// block rotate indices
	PxReal eV = dV - zV, lD = splatX(dot(eV, eV));										// edges to apex and squared lengths
	if(anyX<3>(lD==0))
		return DEGENERATE;																// FAIL: apex is coincident with a base vert
	
	PxReal tmp = __shfl_sync(FULL_MASK, eV, vNextI), shortEdgeV = __shfl_sync(FULL_MASK, lD, vNextI) < lD ? tmp : eV;		// shorter of edges to apex for each tri
	PxReal nV = cross(shortEdgeV, __shfl_sync(FULL_MASK, zV, vNextI) - zV, cp0I, cp1I);					// face normals, outward facing 
	PxReal aS = dot(nV, dV);															// distances along normals of apex
	PxReal gS = dot(__shfl_sync(FULL_MASK, zV, vPrevI), nV);												// distances along normal of vert not in this triangle
	
	if(anyX<3>(aS<=gS))
		return DEGENERATE;																// FAIL: simplex inversion/collapse

	PxReal tmpV = __shfl_up_sync(FULL_MASK, aS, 3);
	nV = (tI & 3) == 3 ? tmpV : nV; 

	if(allX<3>(aS>0))																	// REGION: inside simplex
	{
		zV = setNormal123(zV, nV);														// set normals for EPA start
		return 0x2104;
	}
	
	const PxReal pS = dot(eV,dV);														// projection of origin onto edges
	if(allX<3>(pS<=0))																	// REGION: apex
	{
		closestPoint = dV;
		return 0x2131;
	}

	bool uiS = dot(cross(eV, nV, cp0I, cp1I), dV) <= 0;									// origin outside one edge plane of each triangle
	bool ujS = dot(cross(__shfl_sync(FULL_MASK, eV,vNextI), nV, cp0I, cp1I), dV) > 0;						// origin outside other edge plane of each triangle
	int eid = __ffs(packXFlags<3>(uiS & __shfl_sync(FULL_MASK, ujS, vPrevI) & bool(pS>=0)));				// REGION: edges - origin outside edge planes and projects onto edge
	if(eid--)																			// __ffs returns 0 for no bit, 1 for lsb
	{
		closestPoint = splatV(eid, dV - eV * splatX(pS/lD));
		return eid<<8| 0x32;
	}

	PxU32 fid = __ffs(packXFlags<3>(!uiS & !ujS & bool(aS<=0)));						// REGION: faces - origin outside face plane, inside edge planes
	if(fid--)																			// __ffs returns 0 for no bit, 1 for lsb
	{
		closestPoint = splatV(fid, nV * splatX(aS / dot(nV, nV)));
		zV = setNormal0(zV, -nV, fid);													// flip normal to be outward facing for next iteration
		return next3(fid)<<12 | fid<<8| 0x33;
	}

	return DEGENERATE;																	// FAIL: couldn't find voronoi region. Chort vozmi!
}

static __device__ inline PxU8 supportIndex(const PxVec3* v, PxU8 nbVertices, PxReal dir)
{
	int bestN = 0xFFFFFFFF; //Ininitialize to a very illegal value to force a crash when we get nans
	PxReal bestDist = -FLT_MAX;
	PxReal x = __shfl_sync(FULL_MASK, dir, 0), y = __shfl_sync(FULL_MASK, dir, 1), z = __shfl_sync(FULL_MASK, dir, 2);
	for(PxU32 i=0;i<nbVertices;i+=32)
	{
		int n = i+threadIdx.x;
		PxReal d = n < nbVertices ? x*v[n].x + y*v[n].y + z*v[n].z : -FLT_MAX, e;

		e = fmaxf(d, __shfl_xor_sync(FULL_MASK, d, 16));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 8));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 4));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 2));
		e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 1));

		unsigned mask_bestDist = __ballot_sync(FULL_MASK, e > bestDist);
		if(e>bestDist)
			bestDist = e, bestN = i+__ffs(__ballot_sync(mask_bestDist, e == d))-1;
	}

	assert(bestN < nbVertices);
	return PxU8(bestN);
}

static __device__ inline PxU8 supportIndex(const PxVec3& v0, const PxVec3& v1, PxU8 nbVertices, PxReal dir)
{
	int bestN = 0xFFFFFFFF;
	//PxReal bestDist = -FLT_MAX;
	PxReal x = __shfl_sync(FULL_MASK, dir, 0), y = __shfl_sync(FULL_MASK, dir, 1), z = __shfl_sync(FULL_MASK, dir, 2);
	
	PxReal d = -FLT_MAX;
	PxU32 n = threadIdx.x;
	if (threadIdx.x < nbVertices)
	{
		d = x*v0.x + y*v0.y + z*v0.z;

		if ((threadIdx.x + 32) < nbVertices)
		{
			PxReal d1 = x*v0.x + y*v0.y + z*v0.z;
			if (d1 > d)
			{
				d = d1;
				n += 32;
			}
		}
	}
	
	PxReal e = fmaxf(d, __shfl_xor_sync(FULL_MASK, d, 16));
	e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 8));
	e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 4));
	e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 2));
	e = fmaxf(e, __shfl_xor_sync(FULL_MASK, e, 1));

	//bestDist = e;
	bestN = __ffs(__ballot_sync(FULL_MASK, e == d)) - 1;
	bestN += __popc(__ballot_sync(FULL_MASK, n >= 32) & (1 << bestN)) * 32;

	assert(bestN < nbVertices);
	
	return PxU8(bestN);
}


static __device__ inline schlock::GjkResult::Enum gjk(
										const PxVec3* vA, PxU32 nbA,
										const PxVec3* vB, PxU32 nbB,	
										const PxVec3& initialUnitDir, 
										PxReal minSep, 
										PxReal convergenceRatio,
										schlock::GjkOutput& output, schlock::GjkCachedData& cacheData)
{
	int cacheMask = (1<< cacheData.size)-1;
	__syncwarp(); //cacheData.size is written further below
	Simplex simplexV = loadSimplex(cacheData);

restart:
	bool warmStarted = false;
	Simplex backupSimplexV;
	int size = 0, backupSize = 0;
	PxReal closestPointV, backupClosestV;
	
	if (cacheMask)
		closestPointV = splatV(0, simplexV);
	else
	{
		if ((threadIdx.x & 3) == 0)
			closestPointV = initialUnitDir.x;
		else if ((threadIdx.x & 3) == 1)
			closestPointV = initialUnitDir.y;
		else if ((threadIdx.x & 3) == 2)
			closestPointV = initialUnitDir.z;
		else
			closestPointV = PX_MAX_F32;
	}

	PxReal upperBound = PX_MAX_F32, lowerBound = -PX_MAX_F32, cm = magnitude(closestPointV); // if we hit a lower bound dir with distance exactly zero, -FLT_MIN means we keep it
	bool degenerate;

	do
	{	
		PxReal dV = closestPointV/cm, supportVertexV;

#if GJK_LOGGING
		PxReal dx = dV, dy = shfl(dV, 1), dz = shfl(dV,2);
		PxReal cpx = closestPointV, cpy = shfl(closestPointV, 1), cpz = shfl(closestPointV,2);
		if(threadIdx.x == 0)
			printf("[%f, %f], dir (%f, %f, %f), closestPoint (%f, %f, %f), cm %f \n", lowerBound, upperBound, dx, dy, dz, cpx, cpy, cpz, cm);
#endif

		// find a warm start entry that can be added to the simplex because it's closer than the closest feature
		int candidates = 0;
		
		if (cacheMask)
			candidates = __ballot_sync(FULL_MASK, __shfl_sync(FULL_MASK, dot(dV, simplexV)<upperBound, threadIdx.x<<2)) & cacheMask;

		if(candidates)		
		{
			warmStarted = true;
			cacheMask &= ~(lowestSetBit(candidates)*2-1);
			supportVertexV = splatV(lowestSetIndex(candidates), simplexV);
			if(threadIdx.x>>2 == size)
				simplexV = supportVertexV;
		}
		else				// if not, stop warm-starting and start using support vertices
		{
			cacheMask = 0;
			PxU8 indexA = supportIndex(vA, nbA, -dV), indexB = supportIndex(vB, nbB, dV);
			supportVertexV = loadV3Unsafe(vA+indexA) - loadV3Unsafe(vB+indexB);
			if(threadIdx.x>>2 == size)
				simplexV = (threadIdx.x&3) < 3 ? supportVertexV : __int_as_float(schlock::encodeIndices(indexA, indexB));
			PxReal lb = splatX(dot(dV,supportVertexV));
			if(lb>lowerBound)
			{
				storeV3(&output.direction, 0, -dV);
				lowerBound = lb;
				if(lowerBound > minSep || lowerBound>=upperBound*convergenceRatio)
					break;
			}
		}
	
#if GJK_LOGGING
		PxReal svx = supportVertexV, svy = shfl(supportVertexV,1), svz = shfl(supportVertexV,2);
		if(threadIdx.x == 0)
			printf("support (%f, %f, %f)\n", svx, svy, svz);
#endif

		PxU32 f = 0;											// f is feature index: (i2<<12 | i1<<8 | i0<<4 | size)
		PxReal pointV;
		switch(size)
		{
		case 0:		f = 0x2101, pointV = supportVertexV; break;	// have to be a bit for 0,1 cases not to trash the cached verts, which also live in the simplex
		case 1:	
		{
			PxReal aV = splatV(0,simplexV), lV = supportVertexV - aV, m = splatX(dot(lV,lV)), x = -splatX(dot(aV,lV));			
			if(m==0)		f = DEGENERATE, pointV = 0;							// FAIL: coincident vertex
			else if(x<m)	f = 0x2102,		pointV = aV + lV * x/m;				// REGION: line segment
			else 			f = 0x2111,		pointV = supportVertexV;			// REGION: new vertex
			break;
		}
		case 2:	f = squawk::closestPointTri(simplexV, supportVertexV, pointV);	break; // no worries for 2 & 3: only (0,1,2) are affected by permute, and they're already used
		case 3:	f = squawk::closestPointTet(simplexV, supportVertexV, pointV);	break;
		}

		cm = magnitude(pointV);
		int newSize = f&0xf;				
		degenerate = f == DEGENERATE || (cm>=upperBound && newSize<=size);
		if(degenerate)
		{
			if(!warmStarted)	// hit a problem warm-starting: just restart cold
				break;
			cacheMask = 0;
			goto restart;
		}

		// local minima happen. Continue so long as either the simplex size increases or the upper bound decreases, 
		backupSize = cm >= upperBound ? size : 0;
		if(backupSize)
			backupSimplexV = simplexV, backupClosestV = closestPointV;	// save the best simplex and size before updating state
		else
			upperBound = cm;

#if GJK_LOGGING
		PxReal px = pointV, py = shfl(pointV,1), pz = shfl(pointV,2);
		if(threadIdx.x == 0)
			printf("%04x (%f, %f, %f) %f\n", f, px, py, pz, cm);
#endif		

		closestPointV = pointV, size = newSize;										// update closest point and simplex
		PxU32 sourceIndex = f>>((threadIdx.x&28)+4)&0xf;							// +4 because size is the first nibble
		PxReal tmp = __shfl_sync(FULL_MASK, simplexV, sourceIndex<<2 | (threadIdx.x&3));
		if(threadIdx.x<12)
			simplexV = tmp;					
	}
	while(upperBound > 0 && lowerBound < upperBound * convergenceRatio);

	if(backupSize>0)
		simplexV = backupSimplexV, closestPointV = backupClosestV, size = backupSize;
	
	__syncwarp();
	storeSimplex(cacheData, simplexV);

	if(threadIdx.x == 0)
	{
		output.degenerate = degenerate;
		output.lowerBound = lowerBound;
		output.upperBound = upperBound;
		cacheData.size = size;
	}

	__syncwarp();

	if(upperBound == 0)
		return schlock::GjkResult::eOVERLAP;

	if(lowerBound>minSep)
		return schlock::GjkResult::eDISTANT;

	PxU8 a0 = cacheData.vertices[0].a(), a1 = cacheData.vertices[1].a(), a2 = cacheData.vertices[2].a();
	PxU8 b0 = cacheData.vertices[0].b(), b1 = cacheData.vertices[1].b(), b2 = cacheData.vertices[2].b();

	PxVec3 closestPoint(closestPointV, __shfl_sync(FULL_MASK, closestPointV,1), __shfl_sync(FULL_MASK, closestPointV,2));

	if(threadIdx.x == 0)
	{
		PxVec3 closestPointDir = -closestPoint.getNormalized();
		output.closestPointDir = closestPointDir;

		PxReal p, q, r;
		if(size == 3)
		{
			schlock::barycentrics(cacheData.vertices[0].getV(), cacheData.vertices[1].getV(), cacheData.vertices[2].getV(), p, q, r, closestPoint);
			output.closestPointA = p * vA[a0] + q * vA[a1] + r * vA[a2];
		}
		else if(size == 2)
		{
			PxVec3 d = cacheData.vertices[1].getV()- cacheData.vertices[0].getV();
			PxReal q = -cacheData.vertices[0].getV().dot(d)/d.magnitudeSquared();
			output.closestPointA = (1-q) * vA[a0] + q * vA[a1];
		}
		else
			output.closestPointA = vA[a0];

		output.closestPointB = output.closestPointA + upperBound * closestPointDir;
	}

	return schlock::GjkResult::eCLOSE;
}
}
#undef DEGENERATE_SIZE_FLAG

#endif
