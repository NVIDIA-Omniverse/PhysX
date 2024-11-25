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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuBV4.h"
using namespace physx;
using namespace Gu;
using namespace aos;

#include "GuBV4_BoxOverlap_Internal.h"
#include "GuBV4_BoxBoxOverlapTest.h"

#define USE_GU_TRI_TRI_OVERLAP_FUNCTION
#ifdef USE_GU_TRI_TRI_OVERLAP_FUNCTION
	#include "GuIntersectionTriangleTriangle.h"
#endif
#include "GuDistanceTriangleTriangle.h"

#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#include "GuBV4_ProcessStreamNoOrder_OBBOBB.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_SwizzledNoOrder.h"
#endif

//#include <stdio.h>
#include "geometry/PxMeshQuery.h"

//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
//! if OPC_TRITRI_EPSILON_TEST is true then we do a check (if |dv|<EPSILON then dv=0.0;) else no check is done (which is less robust, but faster)
#define LOCAL_EPSILON 0.000001f

//! Use epsilon value in tri-tri overlap test
#define OPC_TRITRI_EPSILON_TEST

//! sort so that a<=b
#define SORT(a,b)			\
	if(a>b)					\
	{						\
		const float _c=a;	\
		a=b;				\
		b=_c;				\
	}

//! Edge to edge test based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202
#define EDGE_EDGE_TEST(V0, U0, U1)						\
	Bx = U0[i0] - U1[i0];								\
	By = U0[i1] - U1[i1];								\
	Cx = V0[i0] - U0[i0];								\
	Cy = V0[i1] - U0[i1];								\
	f  = Ay*Bx - Ax*By;									\
	d  = By*Cx - Bx*Cy;									\
	if((f>0.0f && d>=0.0f && d<=f) || (f<0.0f && d<=0.0f && d>=f))	\
	{													\
		const float e=Ax*Cy - Ay*Cx;					\
		if(f>0.0f)										\
		{												\
			if(e>=0.0f && e<=f) return 1;				\
		}												\
		else											\
		{												\
			if(e<=0.0f && e>=f) return 1;				\
		}												\
	}

//! TO BE DOCUMENTED
#define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)		\
{														\
	float Bx,By,Cx,Cy,d,f;								\
	const float Ax = V1[i0] - V0[i0];					\
	const float Ay = V1[i1] - V0[i1];					\
	/* test edge U0,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U0, U1);							\
	/* test edge U1,U2 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U1, U2);							\
	/* test edge U2,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U2, U0);							\
}

//! TO BE DOCUMENTED
#define POINT_IN_TRI(V0, U0, U1, U2)					\
{														\
	/* is T1 completly inside T2? */					\
	/* check if V0 is inside tri(U0,U1,U2) */			\
	float a  = U1[i1] - U0[i1];							\
	float b  = -(U1[i0] - U0[i0]);						\
	float c  = -a*U0[i0] - b*U0[i1];					\
	float d0 = a*V0[i0] + b*V0[i1] + c;					\
														\
	a  = U2[i1] - U1[i1];								\
	b  = -(U2[i0] - U1[i0]);							\
	c  = -a*U1[i0] - b*U1[i1];							\
	const float d1 = a*V0[i0] + b*V0[i1] + c;			\
														\
	a  = U0[i1] - U2[i1];								\
	b  = -(U0[i0] - U2[i0]);							\
	c  = -a*U2[i0] - b*U2[i1];							\
	const float d2 = a*V0[i0] + b*V0[i1] + c;			\
	if(d0*d1>0.0f)										\
	{													\
		if(d0*d2>0.0f) return 1;						\
	}													\
}

static PxU32 CoplanarTriTri(const PxVec3& n, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& u0, const PxVec3& u1, const PxVec3& u2)
{
	int i0,i1;
	{
	float A[3];
	/* first project onto an axis-aligned plane, that maximizes the area */
	/* of the triangles, compute indices: i0,i1. */
	A[0] = fabsf(n.x);
	A[1] = fabsf(n.y);
	A[2] = fabsf(n.z);
	if(A[0]>A[1])
	{
		if(A[0]>A[2])
		{
			i0=1;      /* A[0] is greatest */
			i1=2;
		}
		else
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
	}
	else   /* A[0]<=A[1] */
	{
		if(A[2]>A[1])
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
		else
		{
			i0=0;      /* A[1] is greatest */
			i1=2;
		}
	}
	}

	/* test all edges of triangle 1 against the edges of triangle 2 */
	EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
	POINT_IN_TRI(v0, u0, u1, u2);
	POINT_IN_TRI(u0, v0, v1, v2);

	return 0;
}

//! TO BE DOCUMENTED
#define NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1)	\
{																						\
	if(D0D1>0.0f)																		\
	{																					\
		/* here we know that D0D2<=0.0 */												\
		/* that is D0, D1 are on the same side, D2 on the other or on the plane */		\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else if(D0D2>0.0f)																	\
	{																					\
		/* here we know that d0d1<=0.0 */												\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D1*D2>0.0f || D0!=0.0f)														\
	{																					\
		/* here we know that d0d1<=0.0 or that D0!=0.0 */								\
		A=VV0; B=(VV1 - VV0)*D0; C=(VV2 - VV0)*D0; X0=D0 - D1; X1=D0 - D2;				\
	}																					\
	else if(D1!=0.0f)																	\
	{																					\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D2!=0.0f)																	\
	{																					\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else																				\
	{																					\
		/* triangles are coplanar */													\
		return ignoreCoplanar ? 0 : CoplanarTriTri(N1, V0, V1, V2, U0, U1, U2);			\
	}																					\
}
//#endif

namespace
{
	PX_ALIGN_PREFIX(16)
	struct TriangleData
	{
		PxVec3p	mV0, mV1, mV2;
		PxVec3p	mXXX, mYYY, mZZZ;
//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
		PxVec3	mNormal;
		float	mD;
//#endif
		PX_FORCE_INLINE	void	init(const PxVec3& V0, const PxVec3& V1, const PxVec3& V2)
		{
			// 45 lines of asm (x64)
			const Vec4V V0V = V4LoadU(&V0.x);
			const Vec4V V1V = V4LoadU(&V1.x);
			const Vec4V V2V = V4LoadU(&V2.x);

//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
			const Vec4V E1V = V4Sub(V1V, V0V);
			const Vec4V E2V = V4Sub(V2V, V0V);
			const Vec4V NV = V4Cross(E1V, E2V);
			const FloatV dV = FNeg(V4Dot3(NV, V0V));
//#endif
			V4StoreA(V0V, &mV0.x);
			V4StoreA(V1V, &mV1.x);
			V4StoreA(V2V, &mV2.x);
//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
			V4StoreA(NV, &mNormal.x);
			FStore(dV, &mD);
//#endif
			// 62 lines of asm (x64)
			// const PxVec3 E1 = V1 - V0;
			// const PxVec3 E2 = V2 - V0;
			// const PxVec3 N = E1.cross(E2);
			// mV0 = V0;
			// mV1 = V1;
			// mV2 = V2;
			// mNormal = N;
			// mD = -N.dot(V0);


			const Vec4V tri_xs = V4LoadXYZW(V0.x, V1.x, V2.x, 0.0f);
			const Vec4V tri_ys = V4LoadXYZW(V0.y, V1.y, V2.y, 0.0f);
			const Vec4V tri_zs = V4LoadXYZW(V0.z, V1.z, V2.z, 0.0f);
			V4StoreA(tri_xs, &mXXX.x);
			V4StoreA(tri_ys, &mYYY.x);
			V4StoreA(tri_zs, &mZZZ.x);

		}
	}PX_ALIGN_SUFFIX(16);
}

//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
static PxU32 TriTriOverlap(const TriangleData& data0, const TriangleData& data1, bool ignoreCoplanar)
{
	const PxVec3& V0 = data0.mV0;
	const PxVec3& V1 = data0.mV1;
	const PxVec3& V2 = data0.mV2;
	const PxVec3& U0 = data1.mV0;
	const PxVec3& U1 = data1.mV1;
	const PxVec3& U2 = data1.mV2;

	const PxVec3& N1 = data0.mNormal;
	float du0, du1, du2, du0du1, du0du2;
	{
		const float d1 = data0.mD;

		// Put U0,U1,U2 into plane equation 1 to compute signed distances to the plane
		du0 = N1.dot(U0) + d1;
		du1 = N1.dot(U1) + d1;
		du2 = N1.dot(U2) + d1;

		// Coplanarity robustness check
#ifdef OPC_TRITRI_EPSILON_TEST
		if(fabsf(du0)<LOCAL_EPSILON) du0 = 0.0f;
		if(fabsf(du1)<LOCAL_EPSILON) du1 = 0.0f;
		if(fabsf(du2)<LOCAL_EPSILON) du2 = 0.0f;
#endif
		du0du1 = du0 * du1;
		du0du2 = du0 * du2;

		if(du0du1>0.0f && du0du2>0.0f)	// same sign on all of them + not equal 0 ?
			return 0;					// no intersection occurs
	}

	const PxVec3& N2 = data1.mNormal;
	float dv0, dv1, dv2, dv0dv1, dv0dv2;
	{
		const float d2 = data1.mD;

		// put V0,V1,V2 into plane equation 2
		dv0 = N2.dot(V0) + d2;
		dv1 = N2.dot(V1) + d2;
		dv2 = N2.dot(V2) + d2;

	#ifdef OPC_TRITRI_EPSILON_TEST
		if(fabsf(dv0)<LOCAL_EPSILON) dv0 = 0.0f;
		if(fabsf(dv1)<LOCAL_EPSILON) dv1 = 0.0f;
		if(fabsf(dv2)<LOCAL_EPSILON) dv2 = 0.0f;
	#endif

		dv0dv1 = dv0 * dv1;
		dv0dv2 = dv0 * dv2;

		if(dv0dv1>0.0f && dv0dv2>0.0f)	// same sign on all of them + not equal 0 ?
			return 0;					// no intersection occurs
	}

	// Compute direction of intersection line
	// Compute and index to the largest component of D
	short index = 0;
	{
		const PxVec3 D = N1.cross(N2);

		float max = fabsf(D[0]);
		const float bb = fabsf(D[1]);
		const float cc = fabsf(D[2]);
		if(bb>max)
		{
			max=bb;
			index=1;
		}
		if(cc>max)
		{
			max=cc;
			index=2;
		}
	}

	// This is the simplified projection onto L
	const float vp0 = V0[index];
	const float vp1 = V1[index];
	const float vp2 = V2[index];

	const float up0 = U0[index];
	const float up1 = U1[index];
	const float up2 = U2[index];

	// Compute interval for triangle 1
	float a,b,c,x0,x1;
	NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

	// Compute interval for triangle 2
	float d,e,f,y0,y1;
	NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

	const float xx=x0*x1;
	const float yy=y0*y1;
	const float xxyy=xx*yy;

	float isect1[2], isect2[2];

	float tmp=a*xxyy;
	isect1[0]=tmp+b*x1*yy;
	isect1[1]=tmp+c*x0*yy;

	tmp=d*xxyy;
	isect2[0]=tmp+e*xx*y1;
	isect2[1]=tmp+f*xx*y0;

	SORT(isect1[0],isect1[1]);
	SORT(isect2[0],isect2[1]);

	if(isect1[1]<isect2[0] || isect2[1]<isect1[0])
		return 0;
	return 1;
}
//#endif

//////////

static PX_FORCE_INLINE void projectTriangle4(
	const TriangleData& data,
	const Vec4V& axesX,	// axis0x axis1x axis2x axis3x
	const Vec4V& axesY,	// axis0y axis1y axis2y axis3y
	const Vec4V& axesZ,	// axis0z axis1z axis2z axis3z
	Vec4V& min4,
	Vec4V& max4
	)
{
	Vec4V dp0_4 = V4Mul(V4Load(data.mV0.x), axesX);
	dp0_4 = V4MulAdd(V4Load(data.mV0.y), axesY, dp0_4);	//dp0_4 = V4Add(dp0_4, V4Mul(V4Load(data.mV0.y), axesY));
	dp0_4 = V4MulAdd(V4Load(data.mV0.z), axesZ, dp0_4);	//dp0_4 = V4Add(dp0_4, V4Mul(V4Load(data.mV0.z), axesZ));

	Vec4V dp1_4 = V4Mul(V4Load(data.mV1.x), axesX);
	dp1_4 = V4MulAdd(V4Load(data.mV1.y), axesY, dp1_4);	//dp1_4 = V4Add(dp1_4, V4Mul(V4Load(data.mV1.y), axesY));
	dp1_4 = V4MulAdd(V4Load(data.mV1.z), axesZ, dp1_4);	//dp1_4 = V4Add(dp1_4, V4Mul(V4Load(data.mV1.z), axesZ));

	Vec4V dp2_4 = V4Mul(V4Load(data.mV2.x), axesX);
	dp2_4 = V4MulAdd(V4Load(data.mV2.y), axesY, dp2_4);	//dp2_4 = V4Add(dp2_4, V4Mul(V4Load(data.mV2.y), axesY));
	dp2_4 = V4MulAdd(V4Load(data.mV2.z), axesZ, dp2_4);	//dp2_4 = V4Add(dp2_4, V4Mul(V4Load(data.mV2.z), axesZ));

	min4 = V4Min(V4Min(dp0_4, dp1_4), dp2_4);
	max4 = V4Max(V4Max(dp0_4, dp1_4), dp2_4);
}

static PX_FORCE_INLINE PxU32 V4AnyGrtrX(const Vec4V a, const Vec4V b, PxU32 mask)
{
	const PxU32 moveMask = BGetBitMask(V4IsGrtr(a, b));
	return moveMask & mask;
}

static PX_FORCE_INLINE bool testSepAxis(
	const TriangleData& data0,
	const TriangleData& data1,
	const Vec4V& axesX,	// axis0x axis1x axis2x axis3x
	const Vec4V& axesY,	// axis0y axis1y axis2y axis3y
	const Vec4V& axesZ,	// axis0z axis1z axis2z axis3z
	PxU32 mask
	)
{
	Vec4V min0, max0;
	projectTriangle4(data0, axesX, axesY, axesZ, min0, max0);

	Vec4V min1, max1;
	projectTriangle4(data1, axesX, axesY, axesZ, min1, max1);

	if(	V4AnyGrtrX(min1, max0, mask)
	||	V4AnyGrtrX(min0, max1, mask))
		return false;

	return true;
}

static PX_FORCE_INLINE bool testEdges4(	const TriangleData& data0, const TriangleData& data1,
										const FloatV& edge0_x,
										const FloatV& edge0_y,
										const FloatV& edge0_z,
										const Vec4V& edge1_xs,
										const Vec4V& edge1_ys,
										const Vec4V& edge1_zs
										)
{
	const Vec4V axis_xs = V4Sub(V4Scale(edge1_zs, edge0_y), V4Scale(edge1_ys, edge0_z));
	const Vec4V axis_ys = V4Sub(V4Scale(edge1_xs, edge0_z), V4Scale(edge1_zs, edge0_x));
	const Vec4V axis_zs = V4Sub(V4Scale(edge1_ys, edge0_x), V4Scale(edge1_xs, edge0_y));

	const Vec4V eps = V4Load(1e-6f);

	Vec4V maxV = V4Max(axis_ys, axis_xs);
	maxV = V4Max(axis_zs, maxV);
	Vec4V minV = V4Min(axis_ys, axis_xs);
	minV = V4Min(axis_zs, minV);
	maxV = V4Max(V4Neg(minV), maxV);

	BoolV cmpV = V4IsGrtr(maxV, eps);
	const PxU32 mask = BGetBitMask(cmpV) & 0x7;

	return testSepAxis(data0, data1, axis_xs, axis_ys, axis_zs, mask);
}

//////////

static PX_FORCE_INLINE void projectTriangle(const PxVec3& axis, const TriangleData& triangle, float& min, float& max)
{
	const float dp0 = triangle.mV0.dot(axis);
	const float dp1 = triangle.mV1.dot(axis);
	min = PxMin(dp0, dp1);
	max = PxMax(dp0, dp1);

	const float dp2 = triangle.mV2.dot(axis);
	min = PxMin(min, dp2);
	max = PxMax(max, dp2);
}

static PX_FORCE_INLINE bool testSepAxis(const PxVec3& axis, const TriangleData& triangle0, const TriangleData& triangle1)
{
	float min0, max0;
	projectTriangle(axis, triangle0, min0, max0);

	float min1, max1;
	projectTriangle(axis, triangle1, min1, max1);

	if(max0<min1 || max1<min0)
		return false;

	return true;
}

static PX_FORCE_INLINE bool isAlmostZero(const PxVec3& v)
{
	if(PxAbs(v.x)>1e-6f || PxAbs(v.y)>1e-6f || PxAbs(v.z)>1e-6f)
		return false;
	return true;
}

static PX_FORCE_INLINE bool testEdges(	const TriangleData& tri0, const TriangleData& tri1,
										const PxVec3& edge0, const PxVec3& edge1)
{
	PxVec3 cp = edge0.cross(edge1);
	if(!isAlmostZero(cp))
	{
		if(!testSepAxis(cp, tri0, tri1))
			return false;
	}
	return true;
}

static bool TriTriSAT(const TriangleData& data0, const TriangleData& data1, bool ignoreCoplanar)
{
	{
		const PxReal data1_v0_dot_N0 = data1.mV0.dot(data0.mNormal);
		const PxReal data1_v1_dot_N0 = data1.mV1.dot(data0.mNormal);
		const PxReal data1_v2_dot_N0 = data1.mV2.dot(data0.mNormal);

		const PxReal p1ToA = data1_v0_dot_N0 + data0.mD;
		const PxReal p1ToB = data1_v1_dot_N0 + data0.mD;
		const PxReal p1ToC = data1_v2_dot_N0 + data0.mD;

		const PxReal tolerance = 1e-8f;

		if(PxAbs(p1ToA) < tolerance && PxAbs(p1ToB) < tolerance && PxAbs(p1ToC) < tolerance)
		{
			return ignoreCoplanar ? false : CoplanarTriTri(data0.mNormal,	data0.mV0, data0.mV1, data0.mV2,
																			data1.mV0, data1.mV1, data1.mV2)!=0;
		}

		if ((p1ToA > 0) == (p1ToB > 0) && (p1ToA > 0) == (p1ToC > 0))
		{
			return false; //All points of triangle 2 on same side of triangle 1 -> no intersection
		}
	}

	{
		const PxReal data0_v0_dot_N1 = data0.mV0.dot(data1.mNormal);
		const PxReal data0_v1_dot_N1 = data0.mV1.dot(data1.mNormal);
		const PxReal data0_v2_dot_N1 = data0.mV2.dot(data1.mNormal);

		const PxReal p2ToA = data0_v0_dot_N1 + data1.mD;
		const PxReal p2ToB = data0_v1_dot_N1 + data1.mD;
		const PxReal p2ToC = data0_v2_dot_N1 + data1.mD;

		if ((p2ToA > 0) == (p2ToB > 0) && (p2ToA > 0) == (p2ToC > 0))
			return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	
	}

	{
		const PxVec3 edge1_01 = data1.mV0 - data1.mV1;
		const PxVec3 edge1_12 = data1.mV1 - data1.mV2;
		const PxVec3 edge1_20 = data1.mV2 - data1.mV0;

		{
			const PxVec3 edge0_01 = data0.mV0 - data0.mV1;

			if(!testEdges(data0, data1, edge0_01, edge1_01))
				return false;
			if(!testEdges(data0, data1, edge0_01, edge1_12))
				return false;
			if(!testEdges(data0, data1, edge0_01, edge1_20))
				return false;
		}

		{
			const PxVec3 edge0_12 = data0.mV1 - data0.mV2;

			if(!testEdges(data0, data1, edge0_12, edge1_01))
				return false;
			if(!testEdges(data0, data1, edge0_12, edge1_12))
				return false;
			if(!testEdges(data0, data1, edge0_12, edge1_20))
				return false;
		}

		{
			const PxVec3 edge0_20 = data0.mV2 - data0.mV0;

			if(!testEdges(data0, data1, edge0_20, edge1_01))
				return false;
			if(!testEdges(data0, data1, edge0_20, edge1_12))
				return false;
			if(!testEdges(data0, data1, edge0_20, edge1_20))
				return false;
		}
	}

	return true;
}

static bool TriTriSAT_SIMD(const TriangleData& data0, const TriangleData& data1, bool ignoreCoplanar)
{
	const Vec4V tri1_xs = V4LoadA(&data1.mXXX.x);
	const Vec4V tri1_ys = V4LoadA(&data1.mYYY.x);
	const Vec4V tri1_zs = V4LoadA(&data1.mZZZ.x);
	{
		const Vec4V tri0_normal_x = V4Load(data0.mNormal.x);
		const Vec4V tri0_normal_y = V4Load(data0.mNormal.y);
		const Vec4V tri0_normal_z = V4Load(data0.mNormal.z);

		Vec4V tri1_dot_N0 = V4Mul(tri1_xs, tri0_normal_x);
		// PT: TODO: V4MulAdd
		tri1_dot_N0 = V4Add(tri1_dot_N0, V4Mul(tri1_ys, tri0_normal_y));
		tri1_dot_N0 = V4Add(tri1_dot_N0, V4Mul(tri1_zs, tri0_normal_z));

		const Vec4V p1ToABC = V4Add(tri1_dot_N0, V4Load(data0.mD));
		if(V4AllGrtrOrEq3(V4Load(1e-8f), V4Abs(p1ToABC)))
		{
			return ignoreCoplanar ? false : CoplanarTriTri(data0.mNormal,	data0.mV0, data0.mV1, data0.mV2,
																			data1.mV0, data1.mV1, data1.mV2)!=0;
		}

		PxU32 mm = BGetBitMask(V4IsGrtr(p1ToABC, V4Zero()));
		if((mm & 0x7) == 0x7)
			return false;
		mm = BGetBitMask(V4IsGrtrOrEq(V4Zero(), p1ToABC));
		if((mm & 0x7) == 0x7)
			return false;
	}

	{
		const Vec4V tri0_xs = V4LoadA(&data0.mXXX.x);
		const Vec4V tri0_ys = V4LoadA(&data0.mYYY.x);
		const Vec4V tri0_zs = V4LoadA(&data0.mZZZ.x);

		const Vec4V tri1_normal_x = V4Load(data1.mNormal.x);
		const Vec4V tri1_normal_y = V4Load(data1.mNormal.y);
		const Vec4V tri1_normal_z = V4Load(data1.mNormal.z);

		Vec4V tri0_dot_N1 = V4Mul(tri0_xs, tri1_normal_x);
		// PT: TODO: V4MulAdd
		tri0_dot_N1 = V4Add(tri0_dot_N1, V4Mul(tri0_ys, tri1_normal_y));
		tri0_dot_N1 = V4Add(tri0_dot_N1, V4Mul(tri0_zs, tri1_normal_z));

		const Vec4V p2ToABC = V4Add(tri0_dot_N1, V4Load(data1.mD));

		PxU32 mm = BGetBitMask(V4IsGrtr(p2ToABC, V4Zero()));
		if((mm & 0x7) == 0x7)
			return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	
		mm = BGetBitMask(V4IsGrtrOrEq(V4Zero(), p2ToABC));
		if((mm & 0x7) == 0x7)
			return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	
	}

	{
		const Vec4V tri1_xs_shuffled = V4PermYZXW(tri1_xs);
		const Vec4V tri1_ys_shuffled = V4PermYZXW(tri1_ys);
		const Vec4V tri1_zs_shuffled = V4PermYZXW(tri1_zs);

		const Vec4V edge1_xs = V4Sub(tri1_xs, tri1_xs_shuffled);
		const Vec4V edge1_ys = V4Sub(tri1_ys, tri1_ys_shuffled);
		const Vec4V edge1_zs = V4Sub(tri1_zs, tri1_zs_shuffled);

		const Vec4V data0_V0 = V4LoadA(&data0.mV0.x);
		const Vec4V data0_V1 = V4LoadA(&data0.mV1.x);
		const Vec4V data0_V2 = V4LoadA(&data0.mV2.x);

		const Vec4V edge0_01 = V4Sub(data0_V0, data0_V1);
		if(!testEdges4(data0, data1,
			V4GetX(edge0_01), V4GetY(edge0_01), V4GetZ(edge0_01),
			edge1_xs, edge1_ys, edge1_zs))
			return false;

		const Vec4V edge0_12 = V4Sub(data0_V1, data0_V2);
		if(!testEdges4(data0, data1,
			V4GetX(edge0_12), V4GetY(edge0_12), V4GetZ(edge0_12),
			edge1_xs, edge1_ys, edge1_zs))
			return false;

		const Vec4V edge0_20 = V4Sub(data0_V2, data0_V0);
		if(!testEdges4(data0, data1,
			V4GetX(edge0_20), V4GetY(edge0_20), V4GetZ(edge0_20),
			edge1_xs, edge1_ys, edge1_zs))
			return false;
	}

	return true;
}







// PT: beware, needs padding at the end of src
static PX_FORCE_INLINE void transformV(PxVec3p* PX_RESTRICT dst, const PxVec3* PX_RESTRICT src, const Vec4V& c0, const Vec4V& c1, const Vec4V& c2, const Vec4V& c3)
{
	const Vec4V vertexV = V4LoadU(&src->x);

	Vec4V ResV = V4Scale(c0, V4GetX(vertexV));
	// PT: TODO: V4ScaleAdd
	ResV = V4Add(ResV, V4Scale(c1, V4GetY(vertexV)));
	ResV = V4Add(ResV, V4Scale(c2, V4GetZ(vertexV)));
	ResV = V4Add(ResV, c3);

	V4StoreU(ResV, &dst->x);
}

static bool accumulateResults(PxReportCallback<PxGeomIndexPair>& callback, PxGeomIndexPair*& dst, PxU32& capacity, PxU32& currentSize, PxU32 primIndex0, PxU32 primIndex1, bool mustFlip, bool& abort)
{
	dst[currentSize].id0 = mustFlip ? primIndex1 : primIndex0;
	dst[currentSize].id1 = mustFlip ? primIndex0 : primIndex1;
	currentSize++;
	if(currentSize==capacity)
	{
		callback.mSize = 0;
		if(!callback.flushResults(currentSize, dst))
		{
			abort = true;
			return false;
		}
		dst = callback.mBuffer;
		capacity = callback.mCapacity;
		currentSize = callback.mSize;
	}
	return true;
}

namespace
{
	struct TriVsTriParams;

	typedef bool (*trisVsTrisFunction)(	const TriVsTriParams& params,
										PxU32 nb0, PxU32 startPrim0, const TriangleData* data0,
										PxU32 nb1, PxU32 startPrim1, const TriangleData* data1,
										bool& abort);

	enum TriVsTriImpl
	{
		TRI_TRI_MOLLER_REGULAR,	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
		TRI_TRI_MOLLER_NEW,		// Alternative implementation in Gu
		TRI_TRI_NEW_SAT,		// "New" SAT-based implementation
		TRI_TRI_NEW_SAT_SIMD,	// "New" SAT-based implementation using SIMD
	};

	struct TriVsTriParams
	{
		PX_FORCE_INLINE	TriVsTriParams(trisVsTrisFunction leafFunc, PxReportCallback<PxGeomIndexPair>& callback, float tolerance, bool mustFlip, bool ignoreCoplanar) :
			mLeafFunction	(leafFunc),
			mCallback		(callback),
			mTolerance		(tolerance),
			mMustFlip		(mustFlip),
			mIgnoreCoplanar	(ignoreCoplanar)
		{
		}

		const trisVsTrisFunction			mLeafFunction;
		PxReportCallback<PxGeomIndexPair>&	mCallback;
		const float							mTolerance;
		const bool							mMustFlip;
		const bool							mIgnoreCoplanar;

		PX_NOCOPY(TriVsTriParams)
	};
}

template<const TriVsTriImpl impl>
static bool doTriVsTri_Overlap(	const TriVsTriParams& params,
								PxU32 nb0, PxU32 startPrim0, const TriangleData* data0,
								PxU32 nb1, PxU32 startPrim1, const TriangleData* data1,
								bool& abort)
{
	PX_ASSERT(nb0<=16);
	PX_ASSERT(nb1<=16);

	PxReportCallback<PxGeomIndexPair>& callback = params.mCallback;
	PxGeomIndexPair* dst = callback.mBuffer;
	PxU32 capacity = callback.mCapacity;
	PxU32 currentSize = callback.mSize;
	PX_ASSERT(currentSize<capacity);

	const bool ignoreCoplanar = params.mIgnoreCoplanar;
	const bool mustFlip = params.mMustFlip;

	bool foundHit = false;
	abort = false;

	for(PxU32 i=0;i<nb0;i++)
	{
		for(PxU32 j=0;j<nb1;j++)
		{
			bool ret;
			if(impl==TRI_TRI_MOLLER_REGULAR)
				ret = TriTriOverlap(data0[i], data1[j], ignoreCoplanar);
			else if(impl==TRI_TRI_MOLLER_NEW)
				ret = intersectTriangleTriangle(data0[i].mV0, data0[i].mV1, data0[i].mV2, data1[j].mV0, data1[j].mV1, data1[j].mV2, ignoreCoplanar);
			else if(impl==TRI_TRI_NEW_SAT)
				ret = TriTriSAT(data0[i], data1[j], ignoreCoplanar);
			else if(impl==TRI_TRI_NEW_SAT_SIMD)
				ret = TriTriSAT_SIMD(data0[i], data1[j], ignoreCoplanar);
			else
				ret = false;

			if(ret)
			{
				foundHit = true;
				if(!accumulateResults(callback, dst, capacity, currentSize, startPrim0 + i, startPrim1 + j, mustFlip, abort))
					return true;
			}
		}
	}
	
	callback.mSize = currentSize;
	return foundHit;
}

static bool doTriVsTri_Distance(const TriVsTriParams& params,
								PxU32 nb0, PxU32 startPrim0, const TriangleData* data0,
								PxU32 nb1, PxU32 startPrim1, const TriangleData* data1,
								bool& abort)
{
	PX_ASSERT(nb0<=16);
	PX_ASSERT(nb1<=16);

	PxReportCallback<PxGeomIndexPair>& callback = params.mCallback;
	PxGeomIndexPair* dst = callback.mBuffer;
	PxU32 capacity = callback.mCapacity;
	PxU32 currentSize = callback.mSize;
	PX_ASSERT(currentSize<capacity);

	const bool ignoreCoplanar = params.mIgnoreCoplanar;
	const bool mustFlip = params.mMustFlip;

	bool foundHit = false;
	abort = false;

	const float toleranceSquared = params.mTolerance * params.mTolerance;
	for(PxU32 i=0;i<nb0;i++)
	{
		// PT: TODO: improve this
		const PxVec3p pp[3] = { data0[i].mV0, data0[i].mV1, data0[i].mV2 };
		for(PxU32 j=0;j<nb1;j++)
		{
			PxVec3 cp, cq;

			// PT: TODO: improve this
			const PxVec3p qq[3] = { data1[j].mV0, data1[j].mV1, data1[j].mV2 };

			const float d = distanceTriangleTriangleSquared(cp, cq, pp, qq);
			if(d<=toleranceSquared)
			{
				bool skip = false;
				if(d==0.0f && ignoreCoplanar)
				{
					// PT: distance queries don't know about coplanarity so we must run an extra test here to discard coplanar hits.
					const Vec4V tri1_xs = V4LoadA(&data1->mXXX.x);
					const Vec4V tri1_ys = V4LoadA(&data1->mYYY.x);
					const Vec4V tri1_zs = V4LoadA(&data1->mZZZ.x);
					const Vec4V tri0_normal_x = V4Load(data0->mNormal.x);
					const Vec4V tri0_normal_y = V4Load(data0->mNormal.y);
					const Vec4V tri0_normal_z = V4Load(data0->mNormal.z);

					Vec4V tri1_dot_N0 = V4Mul(tri1_xs, tri0_normal_x);
					// PT: TODO: V4MulAdd
					tri1_dot_N0 = V4Add(tri1_dot_N0, V4Mul(tri1_ys, tri0_normal_y));
					tri1_dot_N0 = V4Add(tri1_dot_N0, V4Mul(tri1_zs, tri0_normal_z));

					const Vec4V p1ToABC = V4Add(tri1_dot_N0, V4Load(data0->mD));
					if(V4AllGrtrOrEq3(V4Load(1e-8f), V4Abs(p1ToABC)))
					{
						skip = CoplanarTriTri(data0->mNormal,	data0->mV0, data0->mV1, data0->mV2,
																data1->mV0, data1->mV1, data1->mV2)!=0;
					}
				}

				if(!skip)
				{
					foundHit = true;
					// PT: TODO: this is not enough here
					if(!accumulateResults(callback, dst, capacity, currentSize, startPrim0 + i, startPrim1 + j, mustFlip, abort))
						return true;
				}
			}
		}
	}

	callback.mSize = currentSize;
	return foundHit;
}

static bool doLeafVsLeaf(const TriVsTriParams& params, const PxU32 prim0, const PxU32 prim1, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1, bool& abort)
{
	// PT: TODO: revisit this approach, it was fine with the original overlap code but now with the 2 additional queries, not so much
	TriangleData data0[16];
	TriangleData data1[16];

	PxU32 nb0 = 0;
	PxU32 startPrim0;
	{
		PxU32 primIndex0 = prim0;
		PxU32 nbTris0 = getNbPrimitives(primIndex0);
		startPrim0 = primIndex0;
		const PxVec3* verts0 = mesh0->getVerts();
		do
		{
			PX_ASSERT(primIndex0<mesh0->getNbTriangles());
			PxU32 VRef00, VRef01, VRef02;
			getVertexReferences(VRef00, VRef01, VRef02, primIndex0++, mesh0->getTris32(), mesh0->getTris16());
			PX_ASSERT(VRef00<mesh0->getNbVertices());
			PX_ASSERT(VRef01<mesh0->getNbVertices());
			PX_ASSERT(VRef02<mesh0->getNbVertices());

			if(mat0to1)
			{
				//const PxVec3 p0 = mat0to1->transform(verts0[VRef00]);
				//const PxVec3 p1 = mat0to1->transform(verts0[VRef01]);
				//const PxVec3 p2 = mat0to1->transform(verts0[VRef02]);
				//data0[nb0++].init(p0, p1, p2);

				const Vec4V c0 = V4LoadU(&mat0to1->column0.x);
				const Vec4V c1 = V4LoadU(&mat0to1->column1.x);
				const Vec4V c2 = V4LoadU(&mat0to1->column2.x);
				const Vec4V c3 = V4LoadU(&mat0to1->column3.x);

				PxVec3p p0, p1, p2;
				transformV(&p0, &verts0[VRef00], c0, c1, c2, c3);
				transformV(&p1, &verts0[VRef01], c0, c1, c2, c3);
				transformV(&p2, &verts0[VRef02], c0, c1, c2, c3);

				data0[nb0++].init(p0, p1, p2);
			}
			else
			{
				data0[nb0++].init(verts0[VRef00], verts0[VRef01], verts0[VRef02]);
			}

		}while(nbTris0--);
	}

	PxU32 nb1 = 0;
	PxU32 startPrim1;
	{
		PxU32 primIndex1 = prim1;
		PxU32 nbTris1 = getNbPrimitives(primIndex1);
		startPrim1 = primIndex1;
		const PxVec3* verts1 = mesh1->getVerts();
		do
		{
			PX_ASSERT(primIndex1<mesh1->getNbTriangles());
			PxU32 VRef10, VRef11, VRef12;
			getVertexReferences(VRef10, VRef11, VRef12, primIndex1++, mesh1->getTris32(), mesh1->getTris16());
			PX_ASSERT(VRef10<mesh1->getNbVertices());
			PX_ASSERT(VRef11<mesh1->getNbVertices());
			PX_ASSERT(VRef12<mesh1->getNbVertices());

			data1[nb1++].init(verts1[VRef10], verts1[VRef11], verts1[VRef12]);

		}while(nbTris1--);
	}

	return (params.mLeafFunction)(params, nb0, startPrim0, data0, nb1, startPrim1, data1, abort);
}

namespace
{
struct MeshMeshParams : OBBTestParams
{
	PX_FORCE_INLINE	MeshMeshParams(	trisVsTrisFunction leafFunc, PxReportCallback<PxGeomIndexPair>& callback, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1, const BV4Tree& tree,
									bool mustFlip, bool ignoreCoplanar, float tolerance) :
		mTriVsTriParams		(leafFunc, callback, tolerance, mustFlip, ignoreCoplanar),
		mMesh0				(mesh0),
		mMesh1				(mesh1),
		mMat0to1			(mat0to1),
		mStatus				(false)
	{
		V4StoreA_Safe(V4LoadU_Safe(&tree.mCenterOrMinCoeff.x), &mCenterOrMinCoeff_PaddedAligned.x);
		V4StoreA_Safe(V4LoadU_Safe(&tree.mExtentsOrMaxCoeff.x), &mExtentsOrMaxCoeff_PaddedAligned.x);

		PxMat33 localBox_rot;
		if(mat0to1)
			localBox_rot = PxMat33(	PxVec3(mat0to1->column0.x, mat0to1->column0.y, mat0to1->column0.z),
									PxVec3(mat0to1->column1.x, mat0to1->column1.y, mat0to1->column1.z),
									PxVec3(mat0to1->column2.x, mat0to1->column2.y, mat0to1->column2.z));
		else
			localBox_rot = PxMat33(PxIdentity);

		setupBoxBoxRotationData(this, &mAbsRot, &localBox_rot);
	}

	void	setupBoxBoxCenterAndExtentData(const PxVec3p& center, const PxVec3p& extents, float tolerance)
	{
		if(mMat0to1)
		{
			const Vec4V c0 = V4LoadU(&mMat0to1->column0.x);
			const Vec4V c1 = V4LoadU(&mMat0to1->column1.x);
			const Vec4V c2 = V4LoadU(&mMat0to1->column2.x);
			const Vec4V c3 = V4LoadU(&mMat0to1->column3.x);
			transformV(&mTBoxToModel_PaddedAligned, &center, c0, c1, c2, c3);
		}
		else
			mTBoxToModel_PaddedAligned = center;

		setupBoxBoxExtentData(this, extents + PxVec3(tolerance), &mAbsRot);
	}

	PxMat33						mAbsRot;	//!< Absolute rotation matrix
	const TriVsTriParams		mTriVsTriParams;
	const SourceMesh*	const	mMesh0;
	const SourceMesh*	const	mMesh1;
	const PxMat44*		const	mMat0to1;
	PxU32						mPrimIndex0;
	bool						mStatus;

	PX_NOCOPY(MeshMeshParams)
};

class LeafFunction_MeshMesh
{
public:
	static PX_FORCE_INLINE PxIntBool doLeafTest(MeshMeshParams* PX_RESTRICT params, PxU32 primIndex1)
	{
		bool abort;
		if(doLeafVsLeaf(params->mTriVsTriParams, params->mPrimIndex0, primIndex1, params->mMesh0, params->mMesh1, params->mMat0to1, abort))
			params->mStatus = true;
		return PxIntBool(abort);
	}
};
}

static PX_FORCE_INLINE void getNodeBounds(Vec4V& centerV, Vec4V& extentsV, const BVDataSwizzledQ* PX_RESTRICT node, PxU32 i, const PxVec3p* PX_RESTRICT centerOrMinCoeff_PaddedAligned, const PxVec3p* PX_RESTRICT extentsOrMaxCoeff_PaddedAligned)
{
	// Dequantize box0
	//OPC_SLABS_GET_MIN_MAX(tn0, i)
	const VecI32V minVi = I4LoadXYZW(node->mX[i].mMin, node->mY[i].mMin, node->mZ[i].mMin, 0);
	const Vec4V minCoeffV = V4LoadA_Safe(&centerOrMinCoeff_PaddedAligned->x);
	Vec4V minV = V4Mul(Vec4V_From_VecI32V(minVi), minCoeffV);
	const VecI32V maxVi = I4LoadXYZW(node->mX[i].mMax, node->mY[i].mMax, node->mZ[i].mMax, 0);
	const Vec4V maxCoeffV = V4LoadA_Safe(&extentsOrMaxCoeff_PaddedAligned->x);
	Vec4V maxV = V4Mul(Vec4V_From_VecI32V(maxVi), maxCoeffV);

	// OPC_SLABS_GET_CEQ(i)
	const FloatV HalfV = FLoad(0.5f);
	centerV = V4Scale(V4Add(maxV, minV), HalfV);
	extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
}

static PX_FORCE_INLINE void getNodeBounds(Vec4V& centerV, Vec4V& extentsV, const BVDataSwizzledNQ* PX_RESTRICT node, PxU32 i, const PxVec3p* PX_RESTRICT /*centerOrMinCoeff_PaddedAligned*/, const PxVec3p* PX_RESTRICT /*extentsOrMaxCoeff_PaddedAligned*/)
{
	const FloatV HalfV = FLoad(0.5f);
	const Vec4V minV = V4LoadXYZW(node->mMinX[i], node->mMinY[i], node->mMinZ[i], 0.0f);
	const Vec4V maxV = V4LoadXYZW(node->mMaxX[i], node->mMaxY[i], node->mMaxZ[i], 0.0f);
	centerV = V4Scale(V4Add(maxV, minV), HalfV);
	extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
}

static PX_FORCE_INLINE PxIntBool doLeafVsNode(const BVDataPackedQ* const PX_RESTRICT root, const BVDataSwizzledQ* PX_RESTRICT node, PxU32 i, MeshMeshParams* PX_RESTRICT params)
{
	return BV4_ProcessStreamSwizzledNoOrderQ<LeafFunction_MeshMesh, MeshMeshParams>(root, node->getChildData(i), params);
}

static PX_FORCE_INLINE PxIntBool doLeafVsNode(const BVDataPackedNQ* const PX_RESTRICT root, const BVDataSwizzledNQ* PX_RESTRICT node, PxU32 i, MeshMeshParams* PX_RESTRICT params)
{
	return BV4_ProcessStreamSwizzledNoOrderNQ<LeafFunction_MeshMesh, MeshMeshParams>(root, node->getChildData(i), params);
}

static void computeBoundsAroundVertices(Vec4V& centerV, Vec4V& extentsV, PxU32 nbVerts, const PxVec3* PX_RESTRICT verts)
{
	Vec4V minV = V4LoadU(&verts[0].x);
	Vec4V maxV = minV;

	for(PxU32 i=1; i<nbVerts; i++)
	{
		const Vec4V vV = V4LoadU(&verts[i].x);
		minV = V4Min(minV, vV);
		maxV = V4Max(maxV, vV);
	}

	const FloatV HalfV = FLoad(0.5f);
	centerV = V4Scale(V4Add(maxV, minV), HalfV);
	extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
}

static PX_NOINLINE bool abortQuery(PxReportCallback<PxGeomIndexPair>& callback, bool& abort)
{
	abort = true;
	callback.mSize = 0;
	return true;
}

static PX_FORCE_INLINE trisVsTrisFunction getLeafFunc(PxMeshMeshQueryFlags meshMeshFlags, float tolerance)
{
	if(tolerance!=0.0f)
		return doTriVsTri_Distance;
	if(meshMeshFlags & PxMeshMeshQueryFlag::eRESERVED1)
		return doTriVsTri_Overlap<TRI_TRI_MOLLER_NEW>;
	if(meshMeshFlags & PxMeshMeshQueryFlag::eRESERVED2)
		return doTriVsTri_Overlap<TRI_TRI_NEW_SAT>;
	if(meshMeshFlags & PxMeshMeshQueryFlag::eRESERVED3)
		return doTriVsTri_Overlap<TRI_TRI_NEW_SAT_SIMD>;
	return doTriVsTri_Overlap<TRI_TRI_MOLLER_REGULAR>;
}

static PX_NOINLINE bool doSmallMeshVsSmallMesh(	PxReportCallback<PxGeomIndexPair>& callback, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1,
												bool& _abort, bool ignoreCoplanar, trisVsTrisFunction leafFunc, float tolerance)
{
	const PxU32 nbTris0 = mesh0->getNbTriangles();
	PX_ASSERT(nbTris0<16);
	const PxU32 nbTris1 = mesh1->getNbTriangles();
	PX_ASSERT(nbTris1<16);

	const TriVsTriParams params(leafFunc, callback, tolerance, false, ignoreCoplanar);

	bool abort;
	bool status = false;
	if(doLeafVsLeaf(params, nbTris0, nbTris1, mesh0, mesh1, mat0to1, abort))
		status = true;
	if(abort)
		return abortQuery(callback, _abort);
	return status;
}

template<class PackedNodeT, class SwizzledNodeT>
static PX_NOINLINE bool doSmallMeshVsTree(	PxReportCallback<PxGeomIndexPair>& callback, MeshMeshParams& params,
											const PackedNodeT* PX_RESTRICT node, const SourceMesh* mesh0, const SourceMesh* mesh1, bool& _abort)
{
	const PxU32 nbTris = mesh0->getNbTriangles();
	PX_ASSERT(nbTris<16);

	{
		BV4_ALIGN16(PxVec3p boxCenter);
		BV4_ALIGN16(PxVec3p boxExtents);

		Vec4V centerV, extentsV;
		computeBoundsAroundVertices(centerV, extentsV, mesh0->getNbVertices(), mesh0->getVerts());
		V4StoreA(centerV, &boxCenter.x);
		V4StoreA(extentsV, &boxExtents.x);

		params.setupBoxBoxCenterAndExtentData(boxCenter, boxExtents, params.mTriVsTriParams.mTolerance);
		params.mPrimIndex0	= nbTris;
	}

	const PackedNodeT* const root = node;
	const SwizzledNodeT* tn = reinterpret_cast<const SwizzledNodeT*>(node);

	bool status = false;
	for(PxU32 i=0;i<4;i++)
	{
		if(tn->mData[i]==0xffffffff)
			continue;

		Vec4V centerV1, extentsV1;
		getNodeBounds(centerV1, extentsV1, tn, i, &params.mCenterOrMinCoeff_PaddedAligned, &params.mExtentsOrMaxCoeff_PaddedAligned);

		if(BV4_BoxBoxOverlap(centerV1, extentsV1, &params))
		{
			if(tn->isLeaf(i))
			{
				bool abort;
				if(doLeafVsLeaf(params.mTriVsTriParams, nbTris, tn->getPrimitive(i), mesh0, mesh1, params.mMat0to1, abort))
					status = true;
				if(abort)
					return abortQuery(callback, _abort);
			}
			else
			{
				if(doLeafVsNode(root, tn, i, &params))
					return abortQuery(callback, _abort);
			}
		}
	}

	return status || params.mStatus;
}

template<class PackedNodeT0, class PackedNodeT1, class SwizzledNodeT0, class SwizzledNodeT1>
static bool BV4_OverlapMeshVsMeshT(	PxReportCallback<PxGeomIndexPair>& callback, const BV4Tree& tree0, const BV4Tree& tree1, const PxMat44* mat0to1, const PxMat44* mat1to0,
									bool& _abort, bool ignoreCoplanar, trisVsTrisFunction leafFunc, float tolerance)
{
	const SourceMesh* mesh0 = static_cast<const SourceMesh*>(tree0.mMeshInterface);
	const SourceMesh* mesh1 = static_cast<const SourceMesh*>(tree1.mMeshInterface);

	const PackedNodeT0* PX_RESTRICT node0 = reinterpret_cast<const PackedNodeT0*>(tree0.mNodes);
	const PackedNodeT1* PX_RESTRICT node1 = reinterpret_cast<const PackedNodeT1*>(tree1.mNodes);
	PX_ASSERT(node0 || node1);

	if(!node0 && node1)
	{
		MeshMeshParams ParamsForTree1Traversal(leafFunc, callback, mesh0, mesh1, mat0to1, tree1, false, ignoreCoplanar, tolerance);
		return doSmallMeshVsTree<PackedNodeT1, SwizzledNodeT1>(callback, ParamsForTree1Traversal, node1, mesh0, mesh1, _abort);
	}
	else if(node0 && !node1)
	{
		MeshMeshParams ParamsForTree0Traversal(leafFunc, callback, mesh1, mesh0, mat1to0, tree0, true, ignoreCoplanar, tolerance);
		return doSmallMeshVsTree<PackedNodeT0, SwizzledNodeT0>(callback, ParamsForTree0Traversal, node0, mesh1, mesh0, _abort);
	}
	else
	{
		PX_ASSERT(node0);
		PX_ASSERT(node1);

		MeshMeshParams ParamsForTree1Traversal(leafFunc, callback, mesh0, mesh1, mat0to1, tree1, false, ignoreCoplanar, tolerance);
		MeshMeshParams ParamsForTree0Traversal(leafFunc, callback, mesh1, mesh0, mat1to0, tree0, true, ignoreCoplanar, tolerance);

		BV4_ALIGN16(PxVec3p boxCenter0);
		BV4_ALIGN16(PxVec3p boxExtents0);
		BV4_ALIGN16(PxVec3p boxCenter1);
		BV4_ALIGN16(PxVec3p boxExtents1);

		struct indexPair
		{
			PxU32	index0;
			PxU32	index1;
		};

		PxU32 nb=1;
		indexPair stack[GU_BV4_STACK_SIZE];
		stack[0].index0 = tree0.mInitData;
		stack[0].index1 = tree1.mInitData;

		bool status = false;

		const PackedNodeT0* const root0 = node0;
		const PackedNodeT1* const root1 = node1;

		do
		{
			const indexPair& childData = stack[--nb];
			node0 = root0 + getChildOffset(childData.index0);
			node1 = root1 + getChildOffset(childData.index1);

			const SwizzledNodeT0* tn0 = reinterpret_cast<const SwizzledNodeT0*>(node0);
			const SwizzledNodeT1* tn1 = reinterpret_cast<const SwizzledNodeT1*>(node1);

			for(PxU32 i=0;i<4;i++)
			{
				if(tn0->mData[i]==0xffffffff)
					continue;

				Vec4V centerV0, extentsV0;
				getNodeBounds(centerV0, extentsV0, tn0, i, &ParamsForTree0Traversal.mCenterOrMinCoeff_PaddedAligned, &ParamsForTree0Traversal.mExtentsOrMaxCoeff_PaddedAligned);
				V4StoreA(centerV0, &boxCenter0.x);
				V4StoreA(extentsV0, &boxExtents0.x);

				ParamsForTree1Traversal.setupBoxBoxCenterAndExtentData(boxCenter0, boxExtents0, tolerance);

				for(PxU32 j=0;j<4;j++)
				{
					if(tn1->mData[j]==0xffffffff)
						continue;

					Vec4V centerV1, extentsV1;
					getNodeBounds(centerV1, extentsV1, tn1, j, &ParamsForTree1Traversal.mCenterOrMinCoeff_PaddedAligned, &ParamsForTree1Traversal.mExtentsOrMaxCoeff_PaddedAligned);

					if(BV4_BoxBoxOverlap(centerV1, extentsV1, &ParamsForTree1Traversal))
					{
						const PxU32 isLeaf0 = tn0->isLeaf(i);
						const PxU32 isLeaf1 = tn1->isLeaf(j);

						if(isLeaf0)
						{
							if(isLeaf1)
							{
								bool abort;
								if(doLeafVsLeaf(ParamsForTree1Traversal.mTriVsTriParams, tn0->getPrimitive(i), tn1->getPrimitive(j), mesh0, mesh1, mat0to1, abort))
									status = true;
								if(abort)
									return abortQuery(callback, _abort);
							}
							else
							{
								ParamsForTree1Traversal.mPrimIndex0	= tn0->getPrimitive(i);
								if(doLeafVsNode(root1, tn1, j, &ParamsForTree1Traversal))
									return abortQuery(callback, _abort);
							}
						}
						else
						{
							if(isLeaf1)
							{
								V4StoreA(centerV1, &boxCenter1.x);
								V4StoreA(extentsV1, &boxExtents1.x);

								ParamsForTree0Traversal.setupBoxBoxCenterAndExtentData(boxCenter1, boxExtents1, tolerance);

								ParamsForTree0Traversal.mPrimIndex0	= tn1->getPrimitive(j);
								if(doLeafVsNode(root0, tn0, i, &ParamsForTree0Traversal))
									return abortQuery(callback, _abort);
							}
							else
							{
								stack[nb].index0 = tn0->getChildData(i);
								stack[nb].index1 = tn1->getChildData(j);
								nb++;
							}
						}
					}
				}
			}

		}while(nb);

		return status || ParamsForTree0Traversal.mStatus || ParamsForTree1Traversal.mStatus;
	}
}





// PT: experimental version supporting scaling. Passed matrices etc are all temporary.

#include "geometry/PxMeshScale.h"
#include "CmMatrix34.h"
#include "CmScaling.h"
#include "GuConvexUtilsInternal.h"
#include "GuBoxConversion.h"

using namespace Cm;

// PT: dups from NpDebugViz.cpp
static PX_FORCE_INLINE Vec4V multiply3x3V_(const Vec4V p, const PxMat34& mat)
{
	Vec4V ResV = V4Scale(V4LoadU(&mat.m.column0.x), V4GetX(p));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat.m.column1.x), V4GetY(p)));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat.m.column2.x), V4GetZ(p)));
	return ResV;
}

// PT: beware, needs padding at the end of dst/src
static PX_FORCE_INLINE void transformV_(PxVec3* dst, const PxVec3* src, const Vec4V p, const PxMat34& mat)
{
	const Vec4V vertexV = V4LoadU(&src->x);
	const Vec4V transformedV = V4Add(multiply3x3V_(vertexV, mat), p);
	V4StoreU(transformedV, &dst->x);
}

// PT: Following ones fetched from GuBounds.cpp and adapted to Vec4V inputs.
// TODO: refactor! this is just a test

// PT: this one may have duplicates in GuBV4_BoxSweep_Internal.h & GuBV4_Raycast.cpp
static PX_FORCE_INLINE Vec4V multiply3x3V_(const Vec4V p, const PxMat33Padded& mat_Padded)
{
	Vec4V ResV = V4Scale(V4LoadU(&mat_Padded.column0.x), V4GetX(p));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat_Padded.column1.x), V4GetY(p)));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat_Padded.column2.x), V4GetZ(p)));
	return ResV;
}

static PX_FORCE_INLINE void transformNoEmptyTestV(Vec4V& c, Vec4V& ext, const PxMat33Padded& rot, const PxVec3& pos, Vec4V boundsCenterV, Vec4V boundsExtentsV)
{
	// PT: unfortunately we can't V4LoadU 'pos' directly (it can come directly from users!). So we have to live with this for now:
	const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&pos.x));
	// PT: but eventually we'd like to use the "unsafe" version (e.g. by switching p&q in PxTransform), which would save 6 instructions on Win32
	const Vec4V cV = V4Add(multiply3x3V_(boundsCenterV, rot), posV);

	c = cV;

	// extended basis vectors
	const Vec4V c0V = V4Scale(V4LoadU(&rot.column0.x), V4GetX(boundsExtentsV));
	const Vec4V c1V = V4Scale(V4LoadU(&rot.column1.x), V4GetY(boundsExtentsV));
	const Vec4V c2V = V4Scale(V4LoadU(&rot.column2.x), V4GetZ(boundsExtentsV));

	// find combination of base vectors that produces max. distance for each component = sum of abs()
	Vec4V extentsV = V4Add(V4Abs(c0V), V4Abs(c1V));
	extentsV = V4Add(extentsV, V4Abs(c2V));

	ext = extentsV;
}

static PX_FORCE_INLINE void transformNoEmptyTest(const PxMat34& absPose, Vec4V& c, Vec4V& ext, Vec4V boundsCenterV, Vec4V boundsExtentsV)
{
	transformNoEmptyTestV(c, ext, static_cast<const PxMat33Padded&>(absPose.m), absPose.p, boundsCenterV, boundsExtentsV);
}

static void computeMeshBounds(const PxMat34& absPose, Vec4V boundsCenterV, Vec4V boundsExtentsV, Vec4V& origin, Vec4V& extent)
{
	transformNoEmptyTest(absPose, origin, extent, boundsCenterV, boundsExtentsV);
}

// PT: TODO: refactor with non-scaled version
static bool doLeafVsLeaf_Scaled(const TriVsTriParams& params, const PxU32 prim0, const PxU32 prim1, const SourceMesh* mesh0, const SourceMesh* mesh1,
								const PxMat34& absPose0, const PxMat34& absPose1, bool& abort)
{
	TriangleData data0[16];
	TriangleData data1[16];

	PxU32 nb0 = 0;
	PxU32 startPrim0;
	{
		PxU32 primIndex0 = prim0;
		PxU32 nbTris0 = getNbPrimitives(primIndex0);
		startPrim0 = primIndex0;
		const PxVec3* verts0 = mesh0->getVerts();
		do
		{
			PX_ASSERT(primIndex0<mesh0->getNbTriangles());
			PxU32 VRef00, VRef01, VRef02;
			getVertexReferences(VRef00, VRef01, VRef02, primIndex0++, mesh0->getTris32(), mesh0->getTris16());
			PX_ASSERT(VRef00<mesh0->getNbVertices());
			PX_ASSERT(VRef01<mesh0->getNbVertices());
			PX_ASSERT(VRef02<mesh0->getNbVertices());

			PxVec3p p0, p1, p2;
			const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&absPose0.p.x));
			transformV_(&p0, &verts0[VRef00], posV, absPose0);
			transformV_(&p1, &verts0[VRef01], posV, absPose0);
			transformV_(&p2, &verts0[VRef02], posV, absPose0);
			data0[nb0++].init(p0, p1, p2);

		}while(nbTris0--);
	}

	PxU32 nb1 = 0;
	PxU32 startPrim1;
	{
		PxU32 primIndex1 = prim1;
		PxU32 nbTris1 = getNbPrimitives(primIndex1);
		startPrim1 = primIndex1;
		const PxVec3* verts1 = mesh1->getVerts();
		do
		{
			PX_ASSERT(primIndex1<mesh1->getNbTriangles());
			PxU32 VRef10, VRef11, VRef12;
			getVertexReferences(VRef10, VRef11, VRef12, primIndex1++, mesh1->getTris32(), mesh1->getTris16());
			PX_ASSERT(VRef10<mesh1->getNbVertices());
			PX_ASSERT(VRef11<mesh1->getNbVertices());
			PX_ASSERT(VRef12<mesh1->getNbVertices());

			PxVec3p p0, p1, p2;
			const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&absPose1.p.x));
			transformV_(&p0, &verts1[VRef10], posV, absPose1);
			transformV_(&p1, &verts1[VRef11], posV, absPose1);
			transformV_(&p2, &verts1[VRef12], posV, absPose1);
			data1[nb1++].init(p0, p1, p2);

		}while(nbTris1--);
	}

	return (params.mLeafFunction)(params, nb0, startPrim0, data0, nb1, startPrim1, data1, abort);
}

static PX_NOINLINE bool doSmallMeshVsSmallMesh_Scaled(	PxReportCallback<PxGeomIndexPair>& callback, const SourceMesh* mesh0, const SourceMesh* mesh1,
														const PxMat34& absPose0, const PxMat34& absPose1,
														bool& _abort, bool ignoreCoplanar, trisVsTrisFunction leafFunc, float tolerance)
{
	const PxU32 nbTris0 = mesh0->getNbTriangles();
	PX_ASSERT(nbTris0<16);
	const PxU32 nbTris1 = mesh1->getNbTriangles();
	PX_ASSERT(nbTris1<16);

	const TriVsTriParams params(leafFunc, callback, tolerance, false, ignoreCoplanar);

	bool abort;
	bool status = false;
	if(doLeafVsLeaf_Scaled(params, nbTris0, nbTris1, mesh0, mesh1, absPose0, absPose1, abort))
		status = true;
	if(abort)
		return abortQuery(callback, _abort);
	return status;
}

namespace
{

// PT: this bit from NpDebugViz.cpp, visualizeTriangleMesh()
static PxMat34 getAbsPose(const PxTransform& meshPose, const PxMeshScale& meshScale)
{
	const PxMat33Padded m33(meshPose.q);
	return PxMat34(m33 * toMat33(meshScale), meshPose.p);
}

struct MeshMeshParams_Scaled : MeshMeshParams
{
	PX_FORCE_INLINE	MeshMeshParams_Scaled(trisVsTrisFunction leafFunc, PxReportCallback<PxGeomIndexPair>& callback, const SourceMesh* mesh0, const SourceMesh* mesh1, 
		const PxTransform& meshPose0, const PxTransform& meshPose1,
		const PxMeshScale& meshScale0, const PxMeshScale& meshScale1,
		const PxMat34& absPose0, const PxMat34& absPose1,
		const PxMat44* mat0to1, const BV4Tree& tree, bool mustFlip, bool ignoreCoplanar, float tolerance) :
		MeshMeshParams	(leafFunc, callback, mesh0, mesh1, mat0to1, tree, mustFlip, ignoreCoplanar, tolerance),
		mMeshPose0		(meshPose0),
		mMeshPose1		(meshPose1),
		mMeshScale0		(meshScale0),
		mMeshScale1		(meshScale1),
		mAbsPose0		(absPose0),
		mAbsPose1		(absPose1)
	{
	}

	const PxTransform&	mMeshPose0;
	const PxTransform&	mMeshPose1;
	const PxMeshScale&	mMeshScale0;
	const PxMeshScale&	mMeshScale1;
	const PxMat34&		mAbsPose0;
	const PxMat34&		mAbsPose1;

	PX_NOCOPY(MeshMeshParams_Scaled)
};

template<class ParamsT>
static PX_FORCE_INLINE void setupBoxBoxData(ParamsT* PX_RESTRICT params, const Box& localBox)
{
	params->setupFullBoxBoxData(localBox.center, localBox.extents, &localBox.rot);
}

class LeafFunction_MeshMesh_Scaled
{
public:
	static PX_FORCE_INLINE PxIntBool doLeafTest(MeshMeshParams_Scaled* PX_RESTRICT params, PxU32 primIndex1)
	{
		bool abort;
		if(doLeafVsLeaf_Scaled(params->mTriVsTriParams, params->mPrimIndex0, primIndex1, params->mMesh0, params->mMesh1, params->mAbsPose0, params->mAbsPose1, abort))
			params->mStatus = true;
		return PxIntBool(abort);
	}
};

	PX_FORCE_INLINE PxIntBool BV4_AABBAABBOverlap(const Vec4VArg boxCenter0V, const Vec4VArg extents0V, const Vec4VArg boxCenter1V, const Vec4VArg extents1V)
	{
		const Vec4V absTV = V4Abs(V4Sub(boxCenter0V, boxCenter1V));
		const BoolV res = V4IsGrtr(absTV, V4Add(extents0V, extents1V));
		const PxU32 test = BGetBitMask(res);
		if(test&7)
			return 0;
		return 1;
	}
}

static PX_FORCE_INLINE PxIntBool doLeafVsNode_Scaled(const BVDataPackedQ* const PX_RESTRICT root, const BVDataSwizzledQ* PX_RESTRICT node, PxU32 i, MeshMeshParams_Scaled* PX_RESTRICT params)
{
	return BV4_ProcessStreamSwizzledNoOrderQ<LeafFunction_MeshMesh_Scaled, MeshMeshParams_Scaled>(root, node->getChildData(i), params);
}

static PX_FORCE_INLINE PxIntBool doLeafVsNode_Scaled(const BVDataPackedNQ* const PX_RESTRICT root, const BVDataSwizzledNQ* PX_RESTRICT node, PxU32 i, MeshMeshParams_Scaled* PX_RESTRICT params)
{
	return BV4_ProcessStreamSwizzledNoOrderNQ<LeafFunction_MeshMesh_Scaled, MeshMeshParams_Scaled>(root, node->getChildData(i), params);
}

// PT: similar to Gu::computeVertexSpaceOBB() but we already computed the inverse, and the source box is an AABB
static void computeVertexSpaceOBB(Box& dst, const PxVec3& center, const PxVec3& extents, const PxMat34& inverse)
{
	// PT: TODO: optimize this
	const Box src(center, extents, PxMat33(PxIdentity));
	dst = transform(inverse, src);
}

template<class PackedNodeT, class SwizzledNodeT>
static PX_NOINLINE bool doSmallMeshVsTree_Scaled(	PxReportCallback<PxGeomIndexPair>& callback, MeshMeshParams_Scaled& params,
													const PackedNodeT* PX_RESTRICT node, const SourceMesh* mesh0, const SourceMesh* mesh1, bool& _abort)
{
	const PxU32 nbTris = mesh0->getNbTriangles();
	PX_ASSERT(nbTris<16);

	Vec4V scaledCenterV, scaledExtentV;
	Box vertexOBB; // query box in vertex space
	{
		BV4_ALIGN16(PxVec3p boxCenter);
		BV4_ALIGN16(PxVec3p boxExtents);

		{
			Vec4V centerV, extentsV;
			computeBoundsAroundVertices(centerV, extentsV, mesh0->getNbVertices(), mesh0->getVerts());

			computeMeshBounds(params.mAbsPose0, centerV, extentsV, scaledCenterV, scaledExtentV);

			V4StoreA(scaledCenterV, &boxCenter.x);
			V4StoreA(scaledExtentV, &boxExtents.x);
		}

		const PxMat34 inverse1 = params.mMeshScale1.getInverse() * Matrix34FromTransform(params.mMeshPose1.getInverse());
		computeVertexSpaceOBB(vertexOBB, boxCenter, boxExtents + PxVec3(params.mTriVsTriParams.mTolerance), inverse1);

		// PT: looks like this was not needed, overwritten by next call
		//params.setupBoxBoxCenterAndExtentData(boxCenter, boxExtents, 0.0f/*params.mTriVsTriParams.mTolerance*/);

		setupBoxBoxData(&params, vertexOBB);

		params.mPrimIndex0	= nbTris;
	}

	const PackedNodeT* const root = node;
	const SwizzledNodeT* tn = reinterpret_cast<const SwizzledNodeT*>(node);

	// PT: TODO: revisit this
	scaledExtentV = V4Add(scaledExtentV, V4Load(params.mTriVsTriParams.mTolerance));

	bool status = false;
	for(PxU32 i=0;i<4;i++)
	{
		if(tn->mData[i]==0xffffffff)
			continue;

		Vec4V centerV1, extentsV1;
		getNodeBounds(centerV1, extentsV1, tn, i, &params.mCenterOrMinCoeff_PaddedAligned, &params.mExtentsOrMaxCoeff_PaddedAligned);

		Vec4V scaledCenterV1, scaledExtentV1;
		computeMeshBounds(params.mAbsPose1, centerV1, extentsV1, scaledCenterV1, scaledExtentV1);

		if(BV4_AABBAABBOverlap(scaledCenterV, scaledExtentV, scaledCenterV1, scaledExtentV1))
		{
			if(tn->isLeaf(i))
			{
				bool abort;
				if(doLeafVsLeaf_Scaled(params.mTriVsTriParams, nbTris, tn->getPrimitive(i), mesh0, mesh1, params.mAbsPose0, params.mAbsPose1, abort))
					status = true;
				if(abort)
					return abortQuery(callback, _abort);
			}
			else
			{
				if(doLeafVsNode_Scaled(root, tn, i, &params))
					return abortQuery(callback, _abort);
			}
		}
	}

	return status || params.mStatus;
}

template<class PackedNodeT0, class PackedNodeT1, class SwizzledNodeT0, class SwizzledNodeT1>
static bool BV4_OverlapMeshVsMeshT_Scaled(PxReportCallback<PxGeomIndexPair>& callback, const BV4Tree& tree0, const BV4Tree& tree1, const PxMat44* mat0to1, const PxMat44* mat1to0,
	const PxTransform& meshPose0, const PxTransform& meshPose1,
	const PxMeshScale& meshScale0, const PxMeshScale& meshScale1,
	const PxMat34& absPose0, const PxMat34& absPose1,
	bool& _abort, bool ignoreCoplanar, trisVsTrisFunction leafFunc, float tolerance)
{
	const SourceMesh* mesh0 = static_cast<const SourceMesh*>(tree0.mMeshInterface);
	const SourceMesh* mesh1 = static_cast<const SourceMesh*>(tree1.mMeshInterface);

	const PackedNodeT0* PX_RESTRICT node0 = reinterpret_cast<const PackedNodeT0*>(tree0.mNodes);
	const PackedNodeT1* PX_RESTRICT node1 = reinterpret_cast<const PackedNodeT1*>(tree1.mNodes);
	PX_ASSERT(node0 || node1);

	if(!node0 && node1)
	{
		MeshMeshParams_Scaled ParamsForTree1Traversal(leafFunc, callback, mesh0, mesh1, meshPose0, meshPose1, meshScale0, meshScale1, absPose0, absPose1, mat0to1, tree1, false, ignoreCoplanar, tolerance);
		return doSmallMeshVsTree_Scaled<PackedNodeT1, SwizzledNodeT1>(callback, ParamsForTree1Traversal, node1, mesh0, mesh1, _abort);
	}
	else if(node0 && !node1)
	{
		MeshMeshParams_Scaled ParamsForTree0Traversal(leafFunc, callback, mesh1, mesh0, meshPose1, meshPose0, meshScale1, meshScale0, absPose1, absPose0, mat1to0, tree0, true, ignoreCoplanar, tolerance);
		return doSmallMeshVsTree_Scaled<PackedNodeT0, SwizzledNodeT0>(callback, ParamsForTree0Traversal, node0, mesh1, mesh0, _abort);
	}
	else
	{
		PX_ASSERT(node0);
		PX_ASSERT(node1);

		// ### some useless computations in there now
		MeshMeshParams_Scaled ParamsForTree1Traversal(leafFunc, callback, mesh0, mesh1, meshPose0, meshPose1, meshScale0, meshScale1, absPose0, absPose1, mat0to1, tree1, false, ignoreCoplanar, tolerance);
		MeshMeshParams_Scaled ParamsForTree0Traversal(leafFunc, callback, mesh1, mesh0, meshPose1, meshPose0, meshScale1, meshScale0, absPose1, absPose0, mat1to0, tree0, true, ignoreCoplanar, tolerance);

		const PxMat34 inverse0 = meshScale0.getInverse() * Matrix34FromTransform(meshPose0.getInverse());
		const PxMat34 inverse1 = meshScale1.getInverse() * Matrix34FromTransform(meshPose1.getInverse());

		BV4_ALIGN16(PxVec3p boxCenter0);
		BV4_ALIGN16(PxVec3p boxExtents0);
		BV4_ALIGN16(PxVec3p boxCenter1);
		BV4_ALIGN16(PxVec3p boxExtents1);

		struct indexPair
		{
			PxU32	index0;
			PxU32	index1;
		};

		PxU32 nb=1;
		indexPair stack[GU_BV4_STACK_SIZE];
		stack[0].index0 = tree0.mInitData;
		stack[0].index1 = tree1.mInitData;

		bool status = false;

		const PackedNodeT0* const root0 = node0;
		const PackedNodeT1* const root1 = node1;

		do
		{
			const indexPair& childData = stack[--nb];
			node0 = root0 + getChildOffset(childData.index0);
			node1 = root1 + getChildOffset(childData.index1);

			const SwizzledNodeT0* tn0 = reinterpret_cast<const SwizzledNodeT0*>(node0);
			const SwizzledNodeT1* tn1 = reinterpret_cast<const SwizzledNodeT1*>(node1);

			for(PxU32 i=0;i<4;i++)
			{
				if(tn0->mData[i]==0xffffffff)
					continue;

				Vec4V scaledCenterV0, scaledExtentV0;
				{
					Vec4V centerV0, extentsV0;
					getNodeBounds(centerV0, extentsV0, tn0, i, &ParamsForTree0Traversal.mCenterOrMinCoeff_PaddedAligned, &ParamsForTree0Traversal.mExtentsOrMaxCoeff_PaddedAligned);

					computeMeshBounds(absPose0, centerV0, extentsV0, scaledCenterV0, scaledExtentV0);

					V4StoreA(scaledCenterV0, &boxCenter0.x);
					V4StoreA(scaledExtentV0, &boxExtents0.x);
				}

				// PT: TODO: revisit this
				scaledExtentV0 = V4Add(scaledExtentV0, V4Load(tolerance));

				for(PxU32 j=0;j<4;j++)
				{
					if(tn1->mData[j]==0xffffffff)
						continue;

					Vec4V scaledCenterV1, scaledExtentV1;
					{
						Vec4V centerV1, extentsV1;
						getNodeBounds(centerV1, extentsV1, tn1, j, &ParamsForTree1Traversal.mCenterOrMinCoeff_PaddedAligned, &ParamsForTree1Traversal.mExtentsOrMaxCoeff_PaddedAligned);

						computeMeshBounds(absPose1, centerV1, extentsV1, scaledCenterV1, scaledExtentV1);
					}

					if(BV4_AABBAABBOverlap(scaledCenterV0, scaledExtentV0, scaledCenterV1, scaledExtentV1))
					{
						const PxU32 isLeaf0 = tn0->isLeaf(i);
						const PxU32 isLeaf1 = tn1->isLeaf(j);

						if(isLeaf0)
						{
							if(isLeaf1)
							{
								bool abort;
								if(doLeafVsLeaf_Scaled(ParamsForTree1Traversal.mTriVsTriParams, tn0->getPrimitive(i), tn1->getPrimitive(j), mesh0, mesh1, absPose0, absPose1, abort))
									status = true;
								if(abort)
									return abortQuery(callback, _abort);
							}
							else
							{
								// PT: TODO: reuse scaledExtentV0 instead of doing "boxExtents0 + PxVec3(tolerance)" ?
								Box vertexOBB; // query box in vertex space
								computeVertexSpaceOBB(vertexOBB, boxCenter0, boxExtents0 + PxVec3(tolerance), inverse1);

								setupBoxBoxData(&ParamsForTree1Traversal, vertexOBB);

								ParamsForTree1Traversal.mPrimIndex0	= tn0->getPrimitive(i);
								if(doLeafVsNode_Scaled(root1, tn1, j, &ParamsForTree1Traversal))
									return abortQuery(callback, _abort);
							}
						}
						else
						{
							if(isLeaf1)
							{
								V4StoreA(scaledCenterV1, &boxCenter1.x);
								V4StoreA(scaledExtentV1, &boxExtents1.x);

								Box vertexOBB; // query box in vertex space
								computeVertexSpaceOBB(vertexOBB, boxCenter1, boxExtents1 + PxVec3(tolerance), inverse0);

								setupBoxBoxData(&ParamsForTree0Traversal, vertexOBB);

								ParamsForTree0Traversal.mPrimIndex0	= tn1->getPrimitive(j);
								if(doLeafVsNode_Scaled(root0, tn0, i, &ParamsForTree0Traversal))
									return abortQuery(callback, _abort);
							}
							else
							{
								stack[nb].index0 = tn0->getChildData(i);
								stack[nb].index1 = tn1->getChildData(j);
								nb++;
							}
						}
					}
				}
			}

		}while(nb);

		return status || ParamsForTree0Traversal.mStatus || ParamsForTree1Traversal.mStatus;
	}
}

// PT: end of scaling-related code. Below are the merged entry points for both scaled & non-scaled versions.

/////////

// PT: each mesh can be:
// 1) a small mesh without tree
// 2) a regular mesh with a quantized tree
// 3) a regular mesh with a non-quantized tree
//
// So for mesh-vs-mesh that's 3*3 = 9 possibilities. Some of them are redundant (e.g. 1 vs 2 and 2 vs 1) so it comes down to:
// 1) small mesh vs small mesh
// 2) small mesh vs quantized tree
// 3) small mesh vs non-quantized tree
// 4) non-quantized tree vs non-quantized tree
// 5) quantized tree vs non-quantized tree
// 6) quantized tree vs quantized tree
// => 6 codepaths
//
// But for each of these codepaths the query can be:
// - all hits or any hits
// - using PxRegularReportCallback / PxLocalStorageReportCallback / PxExternalStorageReportCallback / PxDynamicArrayReportCallback
// So for each codepath that's 2*4 = 8 possible queries.
//
// Thus we'd need 6*8 = 48 different test cases.
//
// This gets worse if we take scaling into account.
//
// UPDATE: and now we also want distance/tolerance queries so multiply this by 2. This is getting too complicated.
// We were at 48 test cases, *2 for scaling, *2 for distance queries = 192 cases to test?

bool BV4_OverlapMeshVsMesh(	PxReportCallback<PxGeomIndexPair>& callback,
							const BV4Tree& tree0, const BV4Tree& tree1, const PxMat44* mat0to1, const PxMat44* mat1to0,
							const PxTransform& meshPose0, const PxTransform& meshPose1,
							const PxMeshScale& meshScale0, const PxMeshScale& meshScale1,
							PxMeshMeshQueryFlags meshMeshFlags, float tolerance)
{
	PxGeomIndexPair stackBuffer[256];
	bool mustResetBuffer;
	if(callback.mBuffer)
	{
		PX_ASSERT(callback.mCapacity);
		mustResetBuffer = false;
	}
	else
	{
		callback.mBuffer = stackBuffer;
		PX_ASSERT(callback.mCapacity<=256);
		if(callback.mCapacity==0 || callback.mCapacity>256)
		{
			callback.mCapacity = 256;
		}
		callback.mSize = 0;
		mustResetBuffer = true;
	}

	const bool ignoreCoplanar = meshMeshFlags & PxMeshMeshQueryFlag::eDISCARD_COPLANAR;
	const trisVsTrisFunction leafFunc = getLeafFunc(meshMeshFlags, tolerance);

	bool status;
	bool abort = false;

	if(!meshScale0.isIdentity() || !meshScale1.isIdentity())
	{
		// PT: TODO: we now compute this pose eagerly rather than lazily. Maybe revisit this later.
		const PxMat34 absPose0 = getAbsPose(meshPose0, meshScale0);
		const PxMat34 absPose1 = getAbsPose(meshPose1, meshScale1);

		if(!tree0.mNodes && !tree1.mNodes)
		{
			const SourceMesh* mesh0 = static_cast<const SourceMesh*>(tree0.mMeshInterface);
			const SourceMesh* mesh1 = static_cast<const SourceMesh*>(tree1.mMeshInterface);
			status = doSmallMeshVsSmallMesh_Scaled(callback, mesh0, mesh1, absPose0, absPose1, abort, ignoreCoplanar, leafFunc, tolerance);
		}
		else
		{
			if(tree0.mQuantized)
			{
				if(tree1.mQuantized)
					status = BV4_OverlapMeshVsMeshT_Scaled<BVDataPackedQ, BVDataPackedQ, BVDataSwizzledQ, BVDataSwizzledQ>(callback, tree0, tree1, mat0to1, mat1to0, meshPose0, meshPose1, meshScale0, meshScale1, absPose0, absPose1, abort, ignoreCoplanar, leafFunc, tolerance);
				else
					status = BV4_OverlapMeshVsMeshT_Scaled<BVDataPackedQ, BVDataPackedNQ, BVDataSwizzledQ, BVDataSwizzledNQ>(callback, tree0, tree1, mat0to1, mat1to0, meshPose0, meshPose1, meshScale0, meshScale1, absPose0, absPose1, abort, ignoreCoplanar, leafFunc, tolerance);
			}
			else
			{
				if(tree1.mQuantized)
					status = BV4_OverlapMeshVsMeshT_Scaled<BVDataPackedNQ, BVDataPackedQ, BVDataSwizzledNQ, BVDataSwizzledQ>(callback, tree0, tree1, mat0to1, mat1to0, meshPose0, meshPose1, meshScale0, meshScale1, absPose0, absPose1, abort, ignoreCoplanar, leafFunc, tolerance);
				else
					status = BV4_OverlapMeshVsMeshT_Scaled<BVDataPackedNQ, BVDataPackedNQ, BVDataSwizzledNQ, BVDataSwizzledNQ>(callback, tree0, tree1, mat0to1, mat1to0, meshPose0, meshPose1, meshScale0, meshScale1, absPose0, absPose1, abort, ignoreCoplanar, leafFunc, tolerance);
			}
		}
	}
	else
	{
		if(!tree0.mNodes && !tree1.mNodes)
		{
			const SourceMesh* mesh0 = static_cast<const SourceMesh*>(tree0.mMeshInterface);
			const SourceMesh* mesh1 = static_cast<const SourceMesh*>(tree1.mMeshInterface);
			status = doSmallMeshVsSmallMesh(callback, mesh0, mesh1, mat0to1, abort, ignoreCoplanar, leafFunc, tolerance);
		}
		else
		{
			if(tree0.mQuantized)
			{
				if(tree1.mQuantized)
					status = BV4_OverlapMeshVsMeshT<BVDataPackedQ, BVDataPackedQ, BVDataSwizzledQ, BVDataSwizzledQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
				else
					status = BV4_OverlapMeshVsMeshT<BVDataPackedQ, BVDataPackedNQ, BVDataSwizzledQ, BVDataSwizzledNQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
			}
			else
			{
				if(tree1.mQuantized)
					status = BV4_OverlapMeshVsMeshT<BVDataPackedNQ, BVDataPackedQ, BVDataSwizzledNQ, BVDataSwizzledQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
				else
					status = BV4_OverlapMeshVsMeshT<BVDataPackedNQ, BVDataPackedNQ, BVDataSwizzledNQ, BVDataSwizzledNQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
			}
		}
	}

	if(!abort)
	{
		const PxU32 currentSize = callback.mSize;
		if(currentSize)
		{
			callback.mSize = 0;
			callback.flushResults(currentSize, callback.mBuffer);
		}
	}

	if(mustResetBuffer)
		callback.mBuffer = NULL;

	return status;
}

