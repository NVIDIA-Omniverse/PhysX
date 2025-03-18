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

#ifndef __CAPSULE_COLLISION_CUH__
#define __CAPSULE_COLLISION_CUH__

#include "distanceSegmentSegment.cuh"

#define ETD_CONVEX_EDGE_01  (1 << 3)	// PT: important value, don't change
#define ETD_CONVEX_EDGE_12  (1 << 4)	// PT: important value, don't change
#define ETD_CONVEX_EDGE_20  (1 << 5)	// PT: important value, don't change

__device__ __forceinline__ bool selectNormal(const PxReal u,
	const PxReal v, PxU8 triFlags)
{
	const PxReal zero = 1e-6f;
	const PxReal one = 0.999999f;
	// Analysis
	if (zero > u)
	{
		if (zero > v)
		{
			// Vertex 0
			if (!(triFlags & (ETD_CONVEX_EDGE_01 | ETD_CONVEX_EDGE_20)))
				return true;
		}
		else if (v > one)
		{
			// Vertex 2
			if (!(triFlags & (ETD_CONVEX_EDGE_12 | ETD_CONVEX_EDGE_20)))
				return true;
		}
		else
		{
			// Edge 0-2
			if (!(triFlags & ETD_CONVEX_EDGE_20))
				return true;
		}
	}
	else if (u > one)
	{
		if (zero > v)
		{
			// Vertex 1
			if (!(triFlags & (ETD_CONVEX_EDGE_01 | ETD_CONVEX_EDGE_12)))
				return true;
		}
	}
	else
	{
		if (zero > v)
		{
			// Edge 0-1
			if (!(triFlags & ETD_CONVEX_EDGE_01))
				return true;
		}
		else
		{
			const PxReal threshold = 0.9999f;
			const PxReal temp = u + v;
			if (temp >= threshold)
			{
				// Edge 1-2
				if (!(triFlags & ETD_CONVEX_EDGE_12))
					return true;
			}
			else
			{
				// Face
				return true;
			}
		}
	}
	return false;
}

__device__ __forceinline__ bool isValidTriangleBarycentricCoord(
	const PxReal v, const PxReal w)
{
	const PxReal eps = 1e-6f;
	const PxReal zero(-eps);
	const PxReal one(1.f + eps);

	return (v >= zero && v <= one) && (w >= zero && w <= one) && ((v + w) <= one);
}

/*
	t is the barycenteric coordinate of a segment
	u is the barycenteric coordinate of a triangle
	v is the barycenteric coordinate of a triangle
*/
__device__ __forceinline__ PxReal distanceSegmentTriangleSquared(
	const PxVec3& p, const PxVec3& q,
	const PxVec3& a, const PxVec3& b,
	const PxVec3& c, PxReal& t, PxReal& u,
	PxReal& v)
{
	const PxVec3 pq = q - p;
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 bc = c - b;
	const PxVec3 ap = p - a;
	const PxVec3 aq = q - a;

	//This is used to calculate the barycentric coordinate
	const PxReal d00 = ab.dot(ab);
	const PxReal d01 = ab.dot(ac);
	const PxReal d11 = ac.dot(ac);
	const PxReal tDenom = d00 * d11 - d01 * d01;

	const PxReal bdenom = tDenom > 0.f ? 1.f / tDenom : PX_MAX_F32;//  FSel(FIsGrtr(tDenom, zero), FRecip(tDenom), zero);

	const PxVec3 n = (ab.cross(ac)).getNormalized();// V3Normalize(V3Cross(ab, ac)); // normalize vector

	//compute the closest point of p and triangle plane abc
	const PxReal dist3 = ap.dot(n);// V3Dot(ap, n);
	const PxReal sqDist3 = dist3 * dist3;

	//compute the closest point of q and triangle plane abc
	const PxReal dist4 = aq.dot(n);// V3Dot(aq, n);
	const PxReal sqDist4 = dist4 * dist4;// FMul(dist4, dist4);
	const PxReal dMul = dist3 * dist4;//FMul(dist3, dist4);
	//const BoolV con = FIsGrtr(zero, dMul);

	// intersect with the plane
	if (dMul <= 0.f)
	{
		//compute the intersect point
		const PxReal nom = -n.dot(ap); // FNeg(V3Dot(n, ap));
		const PxReal denom = 1.f / n.dot(pq);// FRecip(V3Dot(n, pq));
		const PxReal t0 = nom * denom; // FMul(nom, denom);
		const PxVec3 ip = p + pq * t0;// V3ScaleAdd(pq, t, p);//V3Add(p, V3Scale(pq, t));
		const PxVec3 v2 = ip - a;// V3Sub(ip, a);
		const PxReal d20 = v2.dot(ab);// V3Dot(v2, ab);
		const PxReal d21 = v2.dot(ac);// V3Dot(v2, ac);
		const PxReal v00 = (d11 * d20 - d01 * d21) * bdenom; //FMul(FSub(FMul(d11, d20), FMul(d01, d21)), bdenom);
		const PxReal w00 = (d00 * d21 - d01 * d20) * bdenom; //FMul(FSub(FMul(d00, d21), FMul(d01, d20)), bdenom);
		if (isValidTriangleBarycentricCoord(v00, w00))
		{
			t = t0;
			u = v00;
			v = w00;
			return 0.f;
		}
	}

	const PxVec3 closestP31 = p - n * dist3;// V3NegScaleSub(n, dist3, p);//V3Sub(p, V3Scale(n, dist3));
	//const PxVec3 closestP30 = p;

	//Compute the barycentric coordinate for project point of q
	const PxVec3 pV20 = closestP31 - a;
	const PxReal pD20 = pV20.dot(ab);
	const PxReal pD21 = pV20.dot(ac);
	const PxReal v0 = (d11 * pD20 - d01 * pD21) * bdenom;//   FMul(FSub(FMul(d11, pD20), FMul(d01, pD21)), bdenom);
	const PxReal w0 = (d00 * pD21 - d01 * pD20) * bdenom;// FMul(FSub(FMul(d00, pD21), FMul(d01, pD20)), bdenom);

	//check closestP3 is inside the triangle
	const bool con0 = isValidTriangleBarycentricCoord(v0, w0);

	const PxVec3 closestP41 = q - n * dist4;// V3NegScaleSub(n, dist4, q);// V3Sub(q, V3Scale(n, dist4));
	//const PxVec3 closestP40 = q;

	//Compute the barycentric coordinate for project point of q
	const PxVec3 qV20 = closestP41 - a;
	const PxReal qD20 = qV20.dot(ab);
	const PxReal qD21 = qV20.dot(ac);
	const PxReal v1 = (d11 * qD20 - d01 * qD21) * bdenom;// FMul(FSub(FMul(d11, qD20), FMul(d01, qD21)), bdenom);
	const PxReal w1 = (d00 * qD21 - d01 * qD20) * bdenom;// FMul(FSub(FMul(d00, qD21), FMul(d01, qD20)), bdenom);

	const bool con1 = isValidTriangleBarycentricCoord(v1, w1);

	if (con0 && con1)
	{
		/*
			both p and q project points are interior point
		*/
		if (sqDist4 > sqDist3)
		{
			t = 0.f;
			u = v0;
			v = w0;

			return sqDist3;
		}
		else
		{
			t = 1.f;
			u = v1;
			v = w1;

			return sqDist4;
		}
	}
	else
	{
		PxReal t00, t01, t02;
		PxReal t10, t11, t12;

		distanceSegmentSegmentSquared(p, pq, a, ab, t00, t10);
		distanceSegmentSegmentSquared(p, pq, b, bc, t01, t11);
		distanceSegmentSegmentSquared(p, pq, a, ac, t02, t12);

		const PxVec3 closestP00 = p + pq * t00;// V3ScaleAdd(pq, t00, p);
		const PxVec3 closestP01 = a + ab * t10;// V3ScaleAdd(ab, t01, a);
		PxVec3 tempV = closestP00 - closestP01;
		const PxReal sqDist0 = tempV.dot(tempV);

		const PxVec3 closestP10 = p + pq * t01;// V3ScaleAdd(pq, t10, p);
		const PxVec3 closestP11 = b + bc * t11;// V3ScaleAdd(bc, t11, b);
		tempV = closestP10 - closestP11;
		const PxReal sqDist1 = tempV.dot(tempV);

		const PxVec3 closestP20 = p + pq * t02;// V3ScaleAdd(pq, t20, p);
		const PxVec3 closestP21 = a + ac * t12;// V3ScaleAdd(ac, t21, a);
		tempV = closestP20 - closestP21;
		const PxReal sqDist2 = tempV.dot(tempV);

		//edge ab
		const PxReal u10 = t10;
		const PxReal v10 = 0.f;

		//edge bc
		const PxReal u11 = 1.f - t11;
		const PxReal v11 = t11;

		//edge ac
		const PxReal u12 = 0.f;
		const PxReal v12 = t12;

		PxReal sqDistPE;
		PxReal uEdge, vEdge, tSeg;
		if (sqDist1 > sqDist0 && sqDist2 > sqDist0)
		{
			//edge ab
			sqDistPE = sqDist0;
			uEdge = u10;
			vEdge = v10;
			tSeg = t00;
		}
		else if (sqDist2 > sqDist1)
		{
			//edge bc
			sqDistPE = sqDist1;
			uEdge = u11;
			vEdge = v11;
			tSeg = t01;
		}
		else
		{
			//edge ac
			sqDistPE = sqDist2;
			uEdge = u12;
			vEdge = v12;
			tSeg = t02;
		}

		if (con0)
		{
			//p's project point is an interior point
			if (sqDistPE > sqDist3)
			{
				t = 0.f;
				u = v0;
				v = w0;
				return sqDist3;
			}
			else
			{
				t = tSeg;
				u = uEdge;
				v = vEdge;
				return sqDistPE;
			}
		}
		else if (con1)
		{
			//q's project point is an interior point

			if (sqDistPE > sqDist4)
			{
				t = 1.f;
				u = v1;
				v = w1;
				return sqDist4;
			}
			else
			{
				t = tSeg;
				u = uEdge;
				v = vEdge;
				return sqDistPE;
			}
		}
		else
		{
			t = tSeg;
			u = uEdge;
			v = vEdge;
			return sqDistPE;
		}
	}
}

__device__ __forceinline__ PxU32 generateContacts(
	const PxVec3& a, const PxVec3& b,
	const PxVec3& c, const PxVec3& planeNormal,
	const PxVec3& normal, const PxVec3& p,
	const PxVec3& q, const PxReal inflatedRadius,
	PxVec3* outContacts, PxReal* outPen)
{
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 ap = p - a;
	const PxVec3 aq = q - a;

	//This is used to calculate the barycentric coordinate
	const PxReal d00 = ab.dot(ab);// V3Dot(ab, ab);
	const PxReal d01 = ab.dot(ac);// V3Dot(ab, ac);
	const PxReal d11 = ac.dot(ac);// V3Dot(ac, ac);
	const PxReal tdenom = (d00 * d11 - d01 * d01);
	const PxReal bdenom = (tdenom > 0.f) ? (1.f / tdenom) : PX_MAX_F32;// FRecip(FSub(FMul(d00, d11), FMul(d01, d01)));

	//compute the intersect point of p and triangle plane abc
	const PxReal inomp = planeNormal.dot(-ap);// V3Dot(planeNormal, V3Neg(ap));
	const PxReal ideom = planeNormal.dot(normal);// V3Dot(planeNormal, normal);

	const PxReal ipt = ideom > 0.f ? -(inomp / ideom) : PX_MAX_F32;// FSel(FIsGrtr(ideom, FZero()), FDiv(inomp, ideom), FZero());

	const PxVec3 closestP31 = p - normal * ipt;// V3ScaleAdd(normal, ipt, p);
	//const PxVec3 closestP30 = p;

	//Compute the barycentric coordinate for project point of q
	const PxVec3 pV20 = closestP31 - a;
	const PxReal pD20 = pV20.dot(ab);
	const PxReal pD21 = pV20.dot(ac);
	const PxReal v0 = (d11 * pD20 - d01 * pD21) * bdenom;// FMul(FSub(FMul(d11, pD20), FMul(d01, pD21)), bdenom);
	const PxReal w0 = (d00 * pD21 - d01 * pD20) * bdenom;// FMul(FSub(FMul(d00, pD21), FMul(d01, pD20)), bdenom);

	//check closestP3 is inside the triangle
	const bool con0 = isValidTriangleBarycentricCoord(v0, w0);

	//printf("v0 %f, w0 %f\n", v0, w0);

	PxU32 numContacts = 0;

	if (con0 && (inflatedRadius > ipt))
	{
		outContacts[numContacts] = p;
		outPen[numContacts] = ipt;
		numContacts++;

		//printf("con0 %i inflatedRadius %f dist %f pen %f\n", con0, inflatedRadius, dist3, -ipt);
	}

	const PxReal inomq = planeNormal.dot(-aq);

	const PxReal iqt = ideom > 0.f ? -(inomq / ideom) : PX_MAX_F32;//  FSel(FIsGrtr(ideom, FZero()), FDiv(inomq, ideom), FZero());

	const PxVec3 closestP41 = q - normal * iqt;// V3ScaleAdd(normal, iqt, q);
	//const PxVec3 closestP40 = q;

	//Compute the barycentric coordinate for project point of q
	const PxVec3 qV20 = closestP41 - a;// V3Sub(closestP41, a);
	const PxReal qD20 = qV20.dot(ab);// V3Dot(qV20, ab);
	const PxReal qD21 = qV20.dot(ac);// V3Dot(qV20, ac);
	const PxReal v1 = (d11 * qD20 - d01 * qD21) * bdenom;// FMul(FSub(FMul(d11, qD20), FMul(d01, qD21)), bdenom);
	const PxReal w1 = (d00 * qD21 - d01 * qD20) * bdenom;// FMul(FSub(FMul(d00, qD21), FMul(d01, qD20)), bdenom);

	const bool con1 = isValidTriangleBarycentricCoord(v1, w1);

	if (con1 && (inflatedRadius > iqt))
	{
		outContacts[numContacts] = q;
		outPen[numContacts] = iqt;
		numContacts++;

		//printf("con1 %i inflatedRadius %f dist %f pen %f\n", con1, inflatedRadius, dist4, -iqt);
	}

	return numContacts;
}

__device__ __forceinline__ void generateEE(const PxVec3& p, const PxVec3& q, const PxReal sqInflatedRadius,
	const PxVec3& normal, const PxVec3& a, const PxVec3& b,
	PxVec3* outContacts, PxReal* outPen, PxU32& numContacts)
{
	const PxVec3 ab = b - a;
	const PxVec3 n = ab.cross(normal);// V3Cross(ab, normal);
	const PxReal d = n.dot(a);// V3Dot(n, a);
	const PxReal np = n.dot(p);// V3Dot(n, p);
	const PxReal nq = n.dot(q);// V3Dot(n, q);
	const PxReal signP = np - d;// FSub(np, d);
	const PxReal signQ = nq - d;
	const PxReal temp = signP * signQ;
	if (temp > 0.f) return;//If both points in the same side as the plane, no intersect points

	// if colliding edge (p3,p4) and plane are parallel return no collision
	const PxVec3 pq = q - p;
	const PxReal npq = n.dot(pq);// V3Dot(n, pq);
	if (npq == 0.f) return;

	//calculate the intersect point in the segment pq
	const PxReal segTValue = (d - np) / npq;//FDiv(FSub(d, np), npq);
	const PxVec3 localPointA = p + pq * segTValue;// V3ScaleAdd(pq, segTValue, p);

	//printf("edge segTValue %f p (%f, %f, %f) pq(%f, %f, %f)\n", segTValue, p.x, p.y, p.z,
	//	pq.x, pq.y, pq.z);

	//calculate a normal perpendicular to ray localPointA + normal, 2D segment segment intersection
	const PxVec3 perNormal = normal.cross(pq);// V3Cross(normal, pq);
	const PxVec3 ap = localPointA - a;// V3Sub(localPointA, a);
	const PxReal nom = perNormal.dot(ap);// V3Dot(perNormal, ap);
	const PxReal denom = perNormal.dot(ab);// V3Dot(perNormal, ab);

	//const FloatV tValue = FClamp(FDiv(nom, denom), zero, FOne());
	const PxReal tValue = nom / denom;// FDiv(nom, denom);
	//printf("edge segTValue %f tValue %f\n", segTValue, tValue);
	//const bool con = 1.0 >= tValue && tValue >= 0.f;// BAnd(FIsGrtrOrEq(one, tValue), FIsGrtrOrEq(tValue, zero));
	if (tValue > 1.f || tValue < 0.f)
		return;

	//const Vec3V localPointB = V3ScaleAdd(ab, tValue, a); v = V3Sub(localPointA, localPointB); v =  V3NegScaleSub(ab, tValue, tap)
	const PxVec3 v = ap - ab * tValue;// V3NegScaleSub(ab, tValue, ap);
	const PxReal sqDist = v.dot(v); // V3Dot(v, v);

	//printf("edge sqDist %f sqInflatedRadius %f\n", sqDist, sqInflatedRadius);

	if (sqInflatedRadius > sqDist)
	{
		const PxReal signedDist = v.dot(normal);
		//printf("edge signedDist %f localPointA(%f, %f, %f)\n", signedDist, localPointA.x, localPointA.y, localPointA.z);
		outContacts[numContacts] = localPointA;
		outPen[numContacts] = signedDist;
		numContacts++;
	}
}

__device__ __forceinline__ void generateEEContacts(const PxVec3& a, const PxVec3& b,
	const PxVec3& c, const PxVec3& normal,
	const PxVec3& p, const PxVec3& q, const PxReal sqInflatedRadius,
	PxVec3* outContacts, PxReal* outPens, PxU32& numContacts)
{
	if (numContacts < 2)
		generateEE(p, q, sqInflatedRadius, normal, a, b, outContacts, outPens, numContacts);
	if (numContacts < 2)
		generateEE(p, q, sqInflatedRadius, normal, b, c, outContacts, outPens, numContacts);
	if (numContacts < 2)
		generateEE(p, q, sqInflatedRadius, normal, a, c, outContacts, outPens, numContacts);
}

#endif