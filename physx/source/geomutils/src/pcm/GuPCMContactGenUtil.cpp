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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuPCMContactGenUtil.h"

using namespace physx;
using namespace Gu;
using namespace aos;

bool physx::Gu::contains(aos::Vec3V* verts, PxU32 numVerts, const aos::Vec3VArg p, const aos::Vec3VArg min, const aos::Vec3VArg max)
{
	const BoolV tempCon = BOr(V3IsGrtr(min, p), V3IsGrtr(p, max));
	const BoolV con = BOr(BGetX(tempCon), BGetY(tempCon));

	if (BAllEqTTTT(con))
		return false;

	const FloatV tx = V3GetX(p);
	const FloatV ty = V3GetY(p);

	const FloatV eps = FEps();
	const FloatV zero = FZero();
	PxU32 intersectionPoints = 0;

	PxU32 i = 0, j = numVerts - 1;

	for (; i < numVerts; j = i++)
	{
		const FloatV jy = V3GetY(verts[j]);
		const FloatV iy = V3GetY(verts[i]);

		const FloatV jx = V3GetX(verts[j]);
		const FloatV ix = V3GetX(verts[i]);

		//if p is one of the end point, we will return intersect
		const BoolV con0 = BAnd(FIsEq(tx, jx), FIsEq(ty, jy));
		const BoolV con1 = BAnd(FIsEq(tx, ix), FIsEq(ty, iy));

		if (BAllEqTTTT(BOr(con0, con1)))
			return true;

		//(verts[i].y > test.y) != (points[j].y > test.y) 
		const PxU32 yflag0 = FAllGrtr(jy, ty);
		const PxU32 yflag1 = FAllGrtr(iy, ty);

		//ML: the only case the ray will intersect this segment is when the p's y is in between two segments y
		if (yflag0 != yflag1)
		{
			//ML: choose ray, which start at p and every points in the ray will have the same y component
			//t1 = (yp - yj)/(yi - yj)
			//qx = xj + t1*(xi-xj)
			//t = qx - xp > 0 for the ray and segment intersection happen
			const FloatV jix = FSub(ix, jx);
			const FloatV jiy = FSub(iy, jy);
			//const FloatV jtx = FSub(tx, jy);
			const FloatV jty = FSub(ty, jy);
			const FloatV part1 = FMul(jty, jix);
			//const FloatV part2 = FMul(jx,  jiy);
			//const FloatV part3 = FMul(V3Sub(tx, eps),  jiy);
			const FloatV part2 = FMul(FAdd(jx, eps), jiy);
			const FloatV part3 = FMul(tx, jiy);

			const BoolV comp = FIsGrtr(jiy, zero);
			const FloatV tmp = FAdd(part1, part2);
			const FloatV comp1 = FSel(comp, tmp, part3);
			const FloatV comp2 = FSel(comp, part3, tmp);

			if (FAllGrtrOrEq(comp1, comp2))
			{
				if (intersectionPoints == 1)
					return false;

				intersectionPoints++;
			}
		}
	}
	return intersectionPoints> 0;
}

PxI32 physx::Gu::getPolygonIndex(const Gu::PolygonalData& polyData, const SupportLocal* map, const aos::Vec3VArg normal, PxI32& polyIndex2)
{
	//normal is in shape space, need to transform the vertex space
	const Vec3V n = M33TrnspsMulV3(map->vertex2Shape, normal);
	const Vec3V nnormal = V3Neg(n);
	const Vec3V planeN_ = V3LoadU_SafeReadW(polyData.mPolygons[0].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
	FloatV minProj = V3Dot(n, planeN_);

	const FloatV zero = FZero();
	PxI32 closestFaceIndex = 0;

	polyIndex2 = -1;

	for (PxU32 i = 1; i < polyData.mNbPolygons; ++i)
	{
		const Vec3V planeN = V3LoadU_SafeReadW(polyData.mPolygons[i].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
		const FloatV proj = V3Dot(n, planeN);
		if (FAllGrtr(minProj, proj))
		{
			minProj = proj;
			closestFaceIndex = PxI32(i);
		}
	}

	const PxU32 numEdges = polyData.mNbEdges;
	const PxU8* const edgeToFace = polyData.mFacesByEdges;

	//Loop through edges
	PxU32 closestEdge = 0xffffffff;
	FloatV maxDpSq = FMul(minProj, minProj);

	FloatV maxDpSq2 = FLoad(-10.0f);

	for (PxU32 i = 0; i < numEdges; ++i)//, inc = VecI32V_Add(inc, vOne))
	{
		const PxU32 index = i * 2;
		const PxU8 f0 = edgeToFace[index];
		const PxU8 f1 = edgeToFace[index + 1];

		const Vec3V planeNormal0 = V3LoadU_SafeReadW(polyData.mPolygons[f0].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
		const Vec3V planeNormal1 = V3LoadU_SafeReadW(polyData.mPolygons[f1].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class

		// unnormalized edge normal
		const Vec3V edgeNormal = V3Add(planeNormal0, planeNormal1);//polys[f0].mPlane.n + polys[f1].mPlane.n;
		const FloatV enMagSq = V3Dot(edgeNormal, edgeNormal);//edgeNormal.magnitudeSquared();
		//Test normal of current edge - squared test is valid if dp and maxDp both >= 0
		const FloatV dp = V3Dot(edgeNormal, nnormal);//edgeNormal.dot(normal);
		const FloatV sqDp = FMul(dp, dp);

		if(f0==closestFaceIndex || f1==closestFaceIndex)
		{
			if(BAllEqTTTT(FIsGrtrOrEq(sqDp, maxDpSq2)))
			{
				maxDpSq2 = sqDp;
				polyIndex2 = f0==closestFaceIndex ? PxI32(f1) : PxI32(f0);
			}
		}

		const BoolV con0 = FIsGrtrOrEq(dp, zero);
		const BoolV con1 = FIsGrtr(sqDp, FMul(maxDpSq, enMagSq));
		const BoolV con = BAnd(con0, con1);
		if (BAllEqTTTT(con))
		{
			maxDpSq = FDiv(sqDp, enMagSq);
			closestEdge = i;
		}
	}

	if (closestEdge != 0xffffffff)
	{
		const PxU8* FBE = edgeToFace;

		const PxU32 index = closestEdge * 2;
		const PxU32 f0 = FBE[index];
		const PxU32 f1 = FBE[index + 1];

		const Vec3V planeNormal0 = V3LoadU_SafeReadW(polyData.mPolygons[f0].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
		const Vec3V planeNormal1 = V3LoadU_SafeReadW(polyData.mPolygons[f1].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class

		const FloatV dp0 = V3Dot(planeNormal0, nnormal);
		const FloatV dp1 = V3Dot(planeNormal1, nnormal);
		if (FAllGrtr(dp0, dp1))
		{
			closestFaceIndex = PxI32(f0);
			polyIndex2 = PxI32(f1);
		}
		else
		{
			closestFaceIndex = PxI32(f1);
			polyIndex2 = PxI32(f0);
		}
	}

	return closestFaceIndex;
}

PxU32 physx::Gu::getWitnessPolygonIndex(const Gu::PolygonalData& polyData, const SupportLocal* map, const aos::Vec3VArg normal,
	const aos::Vec3VArg closest, PxReal tolerance)
{
	PxReal pd[256];
	//first pass : calculate the smallest distance from the closest point to the polygon face

	//transform the closest p to vertex space
	const Vec3V p = M33MulV3(map->shape2Vertex, closest);
	PxU32 closestFaceIndex = 0;

	PxVec3 closestP;
	V3StoreU(p, closestP);

	const PxReal eps = -tolerance;

	PxPlane plane = polyData.mPolygons[0].mPlane;
	PxReal dist = plane.distance(closestP);
	PxReal minDist = dist >= eps ? PxAbs(dist) : PX_MAX_F32;
	pd[0] = minDist;
	PxReal maxDist = dist;
	PxU32 maxFaceIndex = 0;

	for (PxU32 i = 1; i < polyData.mNbPolygons; ++i)
	{
		plane = polyData.mPolygons[i].mPlane;
		dist = plane.distance(closestP);
		pd[i] = dist >= eps ? PxAbs(dist) : PX_MAX_F32;
		if (minDist > pd[i])
		{
			minDist = pd[i];
			closestFaceIndex = i;
		}
		if (dist > maxDist)
		{
			maxDist = dist;
			maxFaceIndex = i;
		}
	}

	if (minDist == PX_MAX_F32)
		return maxFaceIndex;

	//second pass : select the face which has the normal most close to the gjk/epa normal
	Vec4V plane4 = V4LoadU(&polyData.mPolygons[closestFaceIndex].mPlane.n.x);
	Vec3V n = Vec3V_From_Vec4V(plane4);
	n = V3Normalize(M33TrnspsMulV3(map->shape2Vertex, n));
	FloatV bestProj = V3Dot(n, normal);

	const PxU32 firstPassIndex = closestFaceIndex;

	for (PxU32 i = 0; i< polyData.mNbPolygons; ++i)
	{
		//if the difference between the minimum distance and the distance of p to plane i is within tolerance, we use the normal to chose the best plane 
		if ((tolerance >(pd[i] - minDist)) && (firstPassIndex != i))
		{
			plane4 = V4LoadU(&polyData.mPolygons[i].mPlane.n.x);
			n = Vec3V_From_Vec4V(plane4);
			//rotate the plane normal to shape space. We can roate normal into the vertex space. The reason for
			//that is because it will lose some numerical precision if the object has large scale and this algorithm
			//will choose different face
			n = V3Normalize(M33TrnspsMulV3(map->shape2Vertex, n));
			FloatV proj = V3Dot(n, normal);

			if (FAllGrtr(bestProj, proj))
			{
				closestFaceIndex = i;
				bestProj = proj;
			}
		}
	}

	return closestFaceIndex;
}

/*	PX_FORCE_INLINE bool boxContainsInXY(const aos::FloatVArg x, const aos::FloatVArg y, const aos::Vec3VArg p, const aos::Vec3V* verts, const aos::Vec3VArg min, const aos::Vec3VArg max)
	{
		using namespace aos;

		const BoolV tempCon = BOr(V3IsGrtr(min, p), V3IsGrtr(p, max));
		const BoolV con = BOr(BGetX(tempCon), BGetY(tempCon));
		
		if(BAllEqTTTT(con))
			return false;  
		
		const FloatV zero = FZero();
		FloatV PreviousX = V3GetX(verts[3]);
		FloatV PreviousY = V3GetY(verts[3]);

		// Loop through quad vertices
		for(PxI32 i=0; i<4; i++)
		{
			const FloatV CurrentX = V3GetX(verts[i]);
			const FloatV CurrentY = V3GetY(verts[i]);

			// |CurrentX - PreviousX      x - PreviousX|
			// |CurrentY - PreviousY      y - PreviousY|
			// => similar to backface culling, check each one of the 4 triangles are consistent, in which case
			// the point is within the parallelogram.
			const FloatV v00 = FSub(CurrentX, PreviousX);
			const FloatV v01 = FSub(y,  PreviousY);
			const FloatV v10 = FSub(CurrentY, PreviousY);
			const FloatV v11 = FSub(x,  PreviousX);
			const FloatV temp0 = FMul(v00, v01);
			const FloatV temp1 = FMul(v10, v11);
			if(FAllGrtrOrEq(FSub(temp0, temp1), zero))
				return false;

			PreviousX = CurrentX;
			PreviousY = CurrentY;
		}

		return true;
	}
*/

