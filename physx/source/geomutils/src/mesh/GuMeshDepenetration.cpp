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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuMeshDepenetration.h"
#include "GuLocalCluster.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxMeshQuery.h"
#include "geometry/PxBoxGeometry.h"
#include "foundation/PxSIMDHelpers.h"
#include "foundation/PxFPU.h"
#include "foundation/PxMat34.h"
#include "foundation/PxVecMath.h"
#include "CmRandom.h"
#include "GuEdgeList.h"
#include <stdio.h>

#define VERIFY_SIMD_CODE		0
#define SIMD_EPILOG				1
#define TEST_SIMD_CROSS			0	// ### faster in some cases, slower in others
#define USE_PX_TRANSPOSE_44_34	0	// ### slower!
#define DEP_OFFSET				1.0f
#define USE_2D_SWEEP_BACKUP		1

// PT: this one is tricky and not equivalent to other epsilons used to test cross products.
// - if it is too small it creates inaccuracies in reported time of impact for small triangles.
// - If it is too large it discards too many small triangles and collisions can be missed entirely.
#define DIR_DOT_AXIS_EPSILON	1.0e-5f
//#define DIR_DOT_AXIS_EPSILON	1.0e-6f

//#define testSeparationAxes testSeparationAxes_Scalar
//#define testSeparationAxes testSeparationAxes_SIMD1
#define testSeparationAxes testSeparationAxes_SIMD2

using namespace physx;
using namespace Gu;
using namespace aos;

// PT: TODO: refactor with other versions in the codebase
static PX_FORCE_INLINE void projectTriangle(const PxVec3& dir, const PxVec3* PX_RESTRICT triangle, float& min1, float& max1)
{
	const float dp0 = triangle[0].dot(dir);
	const float dp1 = triangle[1].dot(dir);
	const float dp2 = triangle[2].dot(dir);

	const float partialMin1 = PxMin(dp0, dp1);
	const float partialMax1 = PxMax(dp0, dp1);
	min1 = PxMin(partialMin1, dp2);
	max1 = PxMax(partialMax1, dp2);
}

typedef	PxIntBool RetType;
typedef	PxIntBool MTDType;

#define TEST_OVERLAP									\
	const float d0 = triMin0 - triMax1;					\
	const float d1 = triMax0 - triMin1;					\
	const MTDType bIntersect = (d0<=0.0f && d1>=0.0f);	\
	bValidMTD &= bIntersect;

static PX_FORCE_INLINE RetType testAxis(
	const PxTriangle& tri0, const PxTriangle& tri1,
	const PxVec3& dir, const PxVec3& axis, MTDType& bValidMTD, float& tfirst, float& tlast)
{
	float triMin0, triMax0;
	projectTriangle(axis, tri0.verts, triMin0, triMax0);

	////////

	float triMin1, triMax1;
	projectTriangle(axis, tri1.verts, triMin1, triMax1);

	////////

	TEST_OVERLAP

	const float v = dir.dot(axis);
	if(PxAbs(v) < DIR_DOT_AXIS_EPSILON)
		return bIntersect;
	const float oneOverV = -1.0f / v;

	const float t0_ = d0 * oneOverV;
	const float t1_ = d1 * oneOverV;
	const float t0 = PxMin(t0_, t1_);
	const float t1 = PxMax(t0_, t1_);

	if(t0 > tlast)
		return false;
	if(t1 < tfirst)
		return false;

	tlast = PxMin(t1, tlast);

	tfirst = PxMax(t0, tfirst);

	return true;
}

#if USE_2D_SWEEP_BACKUP
// PT: force no-inline for this backup procedure, to avoid bloating the main codepath
static PX_NOINLINE int sweep2D(
	const PxTriangle& tri0, const PxTriangle& tri1,
	const PxVec3& normal0, const PxVec3& normal1,
	const PxVec3& dir, MTDType& bValidMTD, float& tfirst, float& tlast)
{
	// PT: only a scalar version for now. Probably ok, as it should rarely run.
	const float v0 = dir.dot(normal0);
	const float v1 = dir.dot(normal1);
	if(PxAbs(v0) < 1.0e-6f && PxAbs(v1) < 1.0e-6f)
	{
		const PxVec3 e0a = (tri0.verts[1] - tri0.verts[0]).cross(normal0);
		if((e0a.dot(e0a))>=1.0e-6f && !testAxis(tri0, tri1, dir, e0a, bValidMTD, tfirst, tlast))
			return 0;

		const PxVec3 e0b = (tri0.verts[2] - tri0.verts[1]).cross(normal0);
		if((e0b.dot(e0b))>=1.0e-6f && !testAxis(tri0, tri1, dir, e0b, bValidMTD, tfirst, tlast))
			return 0;

		const PxVec3 e0c = (tri0.verts[0] - tri0.verts[2]).cross(normal0);
		if((e0c.dot(e0c))>=1.0e-6f && !testAxis(tri0, tri1, dir, e0c, bValidMTD, tfirst, tlast))
			return 0;

		const PxVec3 e1a = (tri1.verts[1] - tri1.verts[0]).cross(normal1);
		if((e1a.dot(e1a))>=1.0e-6f && !testAxis(tri0, tri1, dir, e1a, bValidMTD, tfirst, tlast))
			return 0;

		const PxVec3 e1b = (tri1.verts[2] - tri1.verts[1]).cross(normal1);
		if((e1b.dot(e1b))>=1.0e-6f && !testAxis(tri0, tri1, dir, e1b, bValidMTD, tfirst, tlast))
			return 0;

		const PxVec3 e1c = (tri1.verts[0] - tri1.verts[2]).cross(normal1);
		if((e1c.dot(e1c))>=1.0e-6f && !testAxis(tri0, tri1, dir, e1c, bValidMTD, tfirst, tlast))
			return 0;
	}
	return 1;
}
#endif

static PX_FORCE_INLINE int processTimeOfImpactResults(
	const PxTriangle& tri0, const PxTriangle& tri1,
	const PxVec3& normal0, const PxVec3& normal1,
	const PxVec3& dir, float& tcoll, float tfirst, float tlast, float tmax, MTDType bValidMTD)
{
#if USE_2D_SWEEP_BACKUP
	// PT: we can reach this place with tfirst left to its initial value in 2D cases, e.g. when two 2D triangles in
	// a plane are swept against each-other. In most PhysX usages this does not create problems, but it does for the
	// mesh depenetration code. Running the extra 2D tests has a performance cost but in the context of clash detection
	// it is unfortunately necessary.
	if(tfirst == -FLT_MAX)
	{
		if(!sweep2D(tri0, tri1, normal0, normal1, dir, bValidMTD, tfirst, tlast))
			return 0;
	}
#else
	PX_UNUSED(tri0);
	PX_UNUSED(tri1);
	PX_UNUSED(normal0);
	PX_UNUSED(normal1);
	PX_UNUSED(dir);
#endif

	if(tfirst > tmax || tlast < 0.0f)
		return 0;

	if(tfirst <= 0.0f)
	{
		// PT: in very rare cases where everything falls below the 1.0e-6f limit we can reach this place with tfirst
		// left to its initial value. This does not mean we had an initial overlap, so we should not return "tcoll = 0"
		// in this case. Safest course of action is to pretend there were no collisions.
		if(tfirst == -FLT_MAX)
			return 0;

		if(!bValidMTD)
			return 0;
		tcoll = 0.0f;
//tcoll = tfirst;
	}
	else tcoll = tfirst;

	return 1;
}

static PX_FORCE_INLINE int testSeparationAxes_Scalar(
	const PxTriangle& tri0, const PxTriangle& tri1,
	const PxVec3& normal0, const PxVec3& normal1,
	const PxVec3& dir, float tmax, float& tcoll)
{
	MTDType bValidMTD = true;
	float tfirst = -FLT_MAX;
	float tlast  = FLT_MAX;

	// Triangle normals
	if(!testAxis(tri0, tri1, dir, normal0, bValidMTD, tfirst, tlast))
		return 0;
	if(!testAxis(tri0, tri1, dir, normal1, bValidMTD, tfirst, tlast))
		return 0;

	// Edges
	for(PxU32 i=0; i<3; i++)
	{
		int ip1 = int(i+1);
		if(i>=2)	ip1 = 0;
		const PxVec3 triEdge0 = tri0.verts[ip1] - tri0.verts[i];

		{
			const PxVec3 triEdge1 = tri1.verts[1] - tri1.verts[0];
			const PxVec3 sep = triEdge0.cross(triEdge1);
			if((sep.dot(sep))>=1.0e-6f && !testAxis(tri0, tri1, dir, sep, bValidMTD, tfirst, tlast))
				return 0;
		}
		{
			const PxVec3 triEdge1 = tri1.verts[2] - tri1.verts[1];
			const PxVec3 sep = triEdge0.cross(triEdge1);
			if((sep.dot(sep))>=1.0e-6f && !testAxis(tri0, tri1, dir, sep, bValidMTD, tfirst, tlast))
				return 0;
		}
		{
			const PxVec3 triEdge1 = tri1.verts[0] - tri1.verts[2];
			const PxVec3 sep = triEdge0.cross(triEdge1);
			if((sep.dot(sep))>=1.0e-6f && !testAxis(tri0, tri1, dir, sep, bValidMTD, tfirst, tlast))
				return 0;
		}
	}

	return processTimeOfImpactResults(tri0, tri1, normal0, normal1, dir, tcoll, tfirst, tlast, tmax, bValidMTD);
}

	static PX_FORCE_INLINE PxIntBool epilogue(
#if SIMD_EPILOG
		PxU32 cndt, float t0, float t1,
#else
		float d0, float d1, const PxVec3& dir, const PxVec3& axis,
#endif
		PxIntBool bIntersect, PxIntBool& bValidMTD, float& tfirst, float& tlast
	)
	{
		bValidMTD &= bIntersect;

#if SIMD_EPILOG
		if(cndt)
			return bIntersect;
#else
		const float v = dir.dot(axis);

		if(PxAbs(v) < DIR_DOT_AXIS_EPSILON)
			return bIntersect;

		const float oneOverV = -1.0f / v;
		const float t0_ = d0 * oneOverV;
		const float t1_ = d1 * oneOverV;
		const float t0 = PxMin(t0_, t1_);
		const float t1 = PxMax(t0_, t1_);
#endif

		if(t0 > tlast)
			return false;
		if(t1 < tfirst)
			return false;

		tlast = PxMin(t1, tlast);
		tfirst = PxMax(t0, tfirst);

		return true;
	}

	static PX_FORCE_INLINE void projectTriangle4(
		const Vec4VArg axesXV, const Vec4VArg axesYV, const Vec4VArg axesZV,
		const PxTrianglePadded& PX_RESTRICT triangle,
		Vec4V* PX_RESTRICT triMin, Vec4V* PX_RESTRICT triMax)
	{
		const Vec4V v0V = V4LoadU(&triangle.verts[0].x);
		Vec4V dp0V = V4Scale(axesXV, V4GetX(v0V));
		dp0V = V4ScaleAdd(axesYV, V4GetY(v0V), dp0V);
		dp0V = V4ScaleAdd(axesZV, V4GetZ(v0V), dp0V);

		const Vec4V v1V = V4LoadU(&triangle.verts[1].x);
		Vec4V dp1V = V4Scale(axesXV, V4GetX(v1V));
		dp1V = V4ScaleAdd(axesYV, V4GetY(v1V), dp1V);
		dp1V = V4ScaleAdd(axesZV, V4GetZ(v1V), dp1V);

		const Vec4V v2V = V4LoadU(&triangle.verts[2].x);
		Vec4V dp2V = V4Scale(axesXV, V4GetX(v2V));
		dp2V = V4ScaleAdd(axesYV, V4GetY(v2V), dp2V);
		dp2V = V4ScaleAdd(axesZV, V4GetZ(v2V), dp2V);

		// PT: this version is slower on platforms that support SSE2 but it could be faster on platforms where shuffles are slow
		/*
			Vec4V dp0V = V4Mul(V4Load(triangle.verts[0].x), axesXV);
			dp0V = V4Add(dp0V, V4Mul(V4Load(triangle.verts[0].y), axesYV));
			dp0V = V4Add(dp0V, V4Mul(V4Load(triangle.verts[0].z), axesZV));

			Vec4V dp1V = V4Mul(V4Load(triangle.verts[1].x), axesXV);
			dp1V = V4Add(dp1V, V4Mul(V4Load(triangle.verts[1].y), axesYV));
			dp1V = V4Add(dp1V, V4Mul(V4Load(triangle.verts[1].z), axesZV));

			Vec4V dp2V = V4Mul(V4Load(triangle.verts[2].x), axesXV);
			dp2V = V4Add(dp2V, V4Mul(V4Load(triangle.verts[2].y), axesYV));
			dp2V = V4Add(dp2V, V4Mul(V4Load(triangle.verts[2].z), axesZV));
		*/
		*triMin = V4Min(V4Min(dp0V, dp1V), dp2V);
		*triMax = V4Max(V4Max(dp0V, dp1V), dp2V);
	}

	static PX_FORCE_INLINE void testAxes(
		const PxTrianglePadded& tri0, const PxTrianglePadded& tri1,
		const PxVec3& axis0, const PxVec3& axis1,
		const PxVec3& axis2, const PxVec3& axis3,
		PxU32* intersect,
#if SIMD_EPILOG
		const PxVec3& dir,
		PxU32* cndt,
		float* t0,
		float* t1
#else
		float* d0,
		float* d1
#endif
		)
	{
#if USE_PX_TRANSPOSE_44_34
		Vec4V axis0V = V4LoadU(&axis0.x);
		Vec4V axis1V = V4LoadU(&axis1.x);
		Vec4V axis2V = V4LoadU(&axis2.x);
		Vec4V axis3V = V4LoadU(&axis3.x);

		Vec4V axesXV, axesYV, axesZV;
		PX_TRANSPOSE_44_34(axis0V, axis1V, axis2V, axis3V, axesXV, axesYV, axesZV);
#else
		const Vec4V axesXV = V4LoadXYZW(axis0.x, axis1.x, axis2.x, axis3.x);
		const Vec4V axesYV = V4LoadXYZW(axis0.y, axis1.y, axis2.y, axis3.y);
		const Vec4V axesZV = V4LoadXYZW(axis0.z, axis1.z, axis2.z, axis3.z);
#endif
		Vec4V triMin0, triMax0;
		projectTriangle4(axesXV, axesYV, axesZV, tri0, &triMin0, &triMax0);

		Vec4V triMin1, triMax1;
		projectTriangle4(axesXV, axesYV, axesZV, tri1, &triMin1, &triMax1);

		const Vec4V d0V = V4Sub(triMin0, triMax1);
		const Vec4V d1V = V4Sub(triMax0, triMin1);

#if !SIMD_EPILOG
		V4StoreA(d0V, d0);
		V4StoreA(d1V, d1);
#endif
		//const bool bIntersect = (d0<=0.0f && d1>=0.0f);
		const Vec4V zeroV = V4Zero();
		const BoolV d0b = V4IsGrtrOrEq(zeroV, d0V);
		const BoolV d1b = V4IsGrtrOrEq(d1V, zeroV);
		BStoreA(BAnd(d0b, d1b), intersect);

#if SIMD_EPILOG
		Vec4V vV = V4Mul(V4Load(dir.x), axesXV);
		vV = V4Add(vV, V4Mul(V4Load(dir.y), axesYV));
		vV = V4Add(vV, V4Mul(V4Load(dir.z), axesZV));	//const float v = dir.dot(axis);

		const Vec4V epsilonV = V4Load(DIR_DOT_AXIS_EPSILON);
		BStoreA(V4IsGrtr(epsilonV, V4Abs(vV)), cndt);	//if(PxAbs(v) < DIR_DOT_AXIS_EPSILON)

		const Vec4V oneOverVV = V4Neg(V4Recip(vV));		//const float oneOverV = -1.0f / v;
		const Vec4V t0_V = V4Mul(d0V, oneOverVV);		//const float t0_ = d0 * oneOverV;
		const Vec4V t1_V = V4Mul(d1V, oneOverVV);		//const float t1_ = d1 * oneOverV;
		const Vec4V t0V = V4Min(t0_V, t1_V);			//const float t0 = PxMin(t0_, t1_);
		const Vec4V t1V = V4Max(t0_V, t1_V);			//const float t1 = PxMax(t0_, t1_);
		V4StoreA(t0V, t0);
		V4StoreA(t1V, t1);
#endif
	}

	static PX_FORCE_INLINE int processAxes4(
		const PxVec3& sep0, const PxVec3& sep1, const PxVec3& sep2, const PxVec3& sep3,
		bool invalid0, bool invalid1, bool invalid2, bool invalid3,
		const PxTrianglePadded& tri0, const PxTrianglePadded& tri1, const PxVec3& dir,
		PxIntBool& bValidMTD, float& tfirst, float& tlast)
	{
		PX_ALIGN(16, PxU32 intersect[4]);
#if SIMD_EPILOG
		PX_ALIGN(16, PxU32 cndt[4]);
		PX_ALIGN(16, float t0[4]);
		PX_ALIGN(16, float t1[4]);
#else
		PX_ALIGN(16, float d0[4]);
		PX_ALIGN(16, float d1[4]);
#endif

		testAxes(tri0, tri1, sep0, sep1, sep2, sep3, intersect
#if SIMD_EPILOG
				, dir, cndt, t0, t1
#else
				, d0, d1
#endif
				);

		if(!invalid0 && !epilogue(
#if SIMD_EPILOG
			cndt[0], t0[0], t1[0],
#else
			d0[0], d1[0], dir, sep0,
#endif
			intersect[0], bValidMTD, tfirst, tlast))
			return 0;

		if(!invalid1 && !epilogue(
#if SIMD_EPILOG
			cndt[1], t0[1], t1[1],
#else
			d0[1], d1[1], dir, sep1,
#endif
			intersect[1], bValidMTD, tfirst, tlast))
			return 0;

		if(!invalid2 && !epilogue(
#if SIMD_EPILOG
			cndt[2], t0[2], t1[2],
#else
			d0[2], d1[2], dir, sep2,
#endif			
			intersect[2], bValidMTD, tfirst, tlast))
			return 0;

		if(!invalid3 && !epilogue(
#if SIMD_EPILOG
			cndt[3], t0[3], t1[3],
#else
			d0[3], d1[3], dir, sep3,
#endif	
			intersect[3], bValidMTD, tfirst, tlast))
			return 0;

		return 1;
	}

	static PX_FORCE_INLINE int test(PxU32& nbAxes, PxVec3* axes,
									const PxTrianglePadded& tri0, const PxTrianglePadded& tri1, const PxVec3& dir,
									PxIntBool& bValidMTD, float& tfirst, float& tlast)
	{
		const PxU32 savedNbAxes = nbAxes;
		nbAxes = 0;

		const bool validate1 = savedNbAxes > 1;
		const bool validate2 = savedNbAxes > 2;
		const bool validate3 = savedNbAxes > 3;

		const PxVec3& sep0 = axes[0];
		const PxVec3& sep1 = axes[1];
		const PxVec3& sep2 = axes[2];
		const PxVec3& sep3 = axes[3];

		return processAxes4(sep0, sep1, sep2, sep3,
							false, !validate1, !validate2, !validate3,
							tri0, tri1, dir, bValidMTD, tfirst, tlast);
	}

	static PX_FORCE_INLINE int test4(PxU32& nbAxes, PxVec3* axes,
									const PxTrianglePadded& tri0, const PxTrianglePadded& tri1, const PxVec3& dir,
									PxIntBool& bValidMTD, float& tfirst, float& tlast)
	{
		nbAxes = 0;
		return processAxes4(axes[0], axes[1], axes[2], axes[3],
							false, false, false, false,
							tri0, tri1, dir, bValidMTD, tfirst, tlast);
	}

	static PX_FORCE_INLINE int testSeparationAxes_SIMD1(
		const PxTrianglePadded& tri0, const PxTrianglePadded& tri1,
		const PxVec3& normal0, const PxVec3& normal1,
		const PxVec3& dir, float tmax, float& tcoll)
	{
		PxIntBool bValidMTD = true;
		float tfirst = -FLT_MAX;
		float tlast  = FLT_MAX;

		// Triangle normals
		if(!processAxes4(normal0, normal1, PxVec3(0.0f), PxVec3(0.0f),
						false, false, true, true,
						tri0, tri1, dir, bValidMTD, tfirst, tlast))
			return 0;

#if TEST_SIMD_CROSS
		const Vec4V epsilonV = V4Load(1.0e-6f);
#endif

		// Edges
		for(PxU32 i=0; i<3; i++)
		{
			int ip1 = int(i+1);
			if(i>=2)	ip1 = 0;
#if TEST_SIMD_CROSS
			const Vec4V v0_0V = V4LoadU(&tri0.verts[ip1].x);
			const Vec4V v0_1V = V4LoadU(&tri0.verts[i].x);
			const Vec4V triEdge0V = V4Sub(v0_1V, v0_0V);
#else
			const PxVec3 triEdge0 = tri0.verts[ip1] - tri0.verts[i];
#endif
			{
#if TEST_SIMD_CROSS
				const Vec4V v1_0V = V4LoadU(&tri1.verts[0].x);
				const Vec4V v1_1V = V4LoadU(&tri1.verts[1].x);
				const Vec4V v1_2V = V4LoadU(&tri1.verts[2].x);

				const Vec4V triEdge1aV = V4Sub(v1_1V, v1_0V);
				const Vec4V triEdge1bV = V4Sub(v1_2V, v1_1V);
				const Vec4V triEdge1cV = V4Sub(v1_0V, v1_2V);

				const Vec4V sep0V = V4Cross(triEdge0V, triEdge1aV);
				const Vec4V sep1V = V4Cross(triEdge0V, triEdge1bV);
				const Vec4V sep2V = V4Cross(triEdge0V, triEdge1cV);

				PxVec3Padded sep0, sep1, sep2;
				bool invalid0, invalid1, invalid2;

				if(BGetBitMask(V4IsGrtrOrEq(V4Mul(sep0V, sep0V), epsilonV)) & 7)
				{
					V4StoreU(sep0V, &sep0.x);
					invalid0 = false;
				}
				else
				{
					sep0 = PxVec3(0.0f);
					invalid0 = true;
				}

				if(BGetBitMask(V4IsGrtrOrEq(V4Mul(sep1V, sep1V), epsilonV)) & 7)
				{
					V4StoreU(sep1V, &sep1.x);
					invalid1 = false;
				}
				else
				{
					sep1 = PxVec3(0.0f);
					invalid1 = true;
				}

				if(BGetBitMask(V4IsGrtrOrEq(V4Mul(sep2V, sep2V), epsilonV)) & 7)
				{
					V4StoreU(sep2V, &sep2.x);
					invalid2 = false;
				}
				else
				{
					sep2 = PxVec3(0.0f);
					invalid2 = true;
				}
#else
				const PxVec3 triEdge1a = tri1.verts[1] - tri1.verts[0];
				const PxVec3 triEdge1b = tri1.verts[2] - tri1.verts[1];
				const PxVec3 triEdge1c = tri1.verts[0] - tri1.verts[2];

				PxVec3 sep0 = triEdge0.cross(triEdge1a);
				PxVec3 sep1 = triEdge0.cross(triEdge1b);
				PxVec3 sep2 = triEdge0.cross(triEdge1c);

				const bool invalid0 = sep0.dot(sep0) < 1.0e-6f;
				const bool invalid1 = sep1.dot(sep1) < 1.0e-6f;
				const bool invalid2 = sep2.dot(sep2) < 1.0e-6f;
#endif
				const bool invalid3 = true;
				const PxVec3 sep3(0.0f);

				if(!processAxes4(sep0, sep1, sep2, sep3,
								invalid0, invalid1, invalid2, invalid3,
								tri0, tri1, dir, bValidMTD, tfirst, tlast))
					return 0;
			}
		}

		return processTimeOfImpactResults(tri0, tri1, normal0, normal1, dir, tcoll, tfirst, tlast, tmax, bValidMTD);
	}

	static PX_FORCE_INLINE int testSeparationAxes_SIMD2(
		const PxTrianglePadded& tri0, const PxTrianglePadded& tri1,
		const PxVec3& normal0, const PxVec3& normal1,
		const PxVec3& dir, float tmax, float& tcoll)
	{
		PxIntBool bValidMTD = true;
		float tfirst = -FLT_MAX;
		float tlast  = FLT_MAX;

		PxVec3 axes[4+1];

		// Triangle normals
		axes[0] = normal0;
		axes[1] = normal1;
		PxU32 nbAxes = 2;

#if TEST_SIMD_CROSS
		const Vec4V epsilonV = V4Load(1.0e-6f);
#endif
		// Edges
		for(PxU32 i=0; i<3; i++)
		{
			int ip1 = int(i+1);
			if(i>=2)	ip1 = 0;
#if TEST_SIMD_CROSS
			const Vec4V v0_0V = V4LoadU(&tri0.verts[ip1].x);
			const Vec4V v0_1V = V4LoadU(&tri0.verts[i].x);
			const Vec4V triEdge0V = V4Sub(v0_1V, v0_0V);
#else
			const PxVec3 triEdge0 = tri0.verts[ip1] - tri0.verts[i];
#endif
			// TODO: try to swizzle right here when storing the axes
			{
#if TEST_SIMD_CROSS
				const Vec4V v1_0V = V4LoadU(&tri1.verts[0].x);
				const Vec4V v1_1V = V4LoadU(&tri1.verts[1].x);
				const Vec4V triEdge1aV = V4Sub(v1_1V, v1_0V);
				const Vec4V sep0V = V4Cross(triEdge0V, triEdge1aV);
				if(BGetBitMask(V4IsGrtrOrEq(V4Mul(sep0V, sep0V), epsilonV)) & 7)
#else
				const PxVec3 triEdge1a = tri1.verts[1] - tri1.verts[0];
				PxVec3 sep0 = triEdge0.cross(triEdge1a);
				if(sep0.dot(sep0) >= 1.0e-6f)
#endif
				{
#if TEST_SIMD_CROSS
					V4StoreU(sep0V, &axes[nbAxes++].x);
#else
					axes[nbAxes++] = sep0;
#endif
					if(nbAxes == 4 && !::test4(nbAxes, axes, tri0, tri1, dir, bValidMTD, tfirst, tlast))
						return 0;
				}

#if TEST_SIMD_CROSS
				const Vec4V v1_2V = V4LoadU(&tri1.verts[2].x);
				const Vec4V triEdge1bV = V4Sub(v1_2V, v1_1V);
				const Vec4V sep1V = V4Cross(triEdge0V, triEdge1bV);
				if(BGetBitMask(V4IsGrtrOrEq(V4Mul(sep1V, sep1V), epsilonV)) & 7)
#else
				const PxVec3 triEdge1b = tri1.verts[2] - tri1.verts[1];
				PxVec3 sep1 = triEdge0.cross(triEdge1b);
				if(sep1.dot(sep1) >= 1.0e-6f)
#endif
				{
#if TEST_SIMD_CROSS
					V4StoreU(sep1V, &axes[nbAxes++].x);
#else
					axes[nbAxes++] = sep1;
#endif
					if(nbAxes == 4 && !::test4(nbAxes, axes, tri0, tri1, dir, bValidMTD, tfirst, tlast))
						return 0;
				}

#if TEST_SIMD_CROSS
				const Vec4V triEdge1cV = V4Sub(v1_0V, v1_2V);
				const Vec4V sep2V = V4Cross(triEdge0V, triEdge1cV);
				if(BGetBitMask(V4IsGrtrOrEq(V4Mul(sep2V, sep2V), epsilonV)) & 7)
#else
				const PxVec3 triEdge1c = tri1.verts[0] - tri1.verts[2];
				PxVec3 sep2 = triEdge0.cross(triEdge1c);
				if(sep2.dot(sep2) >= 1.0e-6f)
#endif
				{
#if TEST_SIMD_CROSS
					V4StoreU(sep2V, &axes[nbAxes++].x);
#else
					axes[nbAxes++] = sep2;
#endif
					if(nbAxes == 4 && !::test4(nbAxes, axes, tri0, tri1, dir, bValidMTD, tfirst, tlast))
						return 0;
				}
			}
		}
		if(nbAxes && !::test(nbAxes, axes, tri0, tri1, dir, bValidMTD, tfirst, tlast))
			return 0;

		return processTimeOfImpactResults(tri0, tri1, normal0, normal1, dir, tcoll, tfirst, tlast, tmax, bValidMTD);
	}

static PX_FORCE_INLINE int triTriSweep(
	const PxTrianglePadded& tri0, const PxTrianglePadded& tri1,
	const PxVec3& dir, float tmax, float& toi/*, PxU32 doBackfaceCulling*/)
{
	// Create triangle normal
	PxVec3 triNormal0;
	tri0.denormalizedNormal(triNormal0);

	PxVec3 triNormal1;
	tri1.denormalizedNormal(triNormal1);

	// Backface culling
//	if(doBackfaceCulling && (triNormal.dot(dir)) >= 0.0f)	// ">=" is important !
//		return 0;

	return testSeparationAxes(tri0, tri1, triNormal0, triNormal1, dir, tmax, toi);
}

namespace
{
	class TriangleProvider
	{
		const void*		mTriangles;
		const PxVec3*	mVerts;
		const PxU32		m16BitIndices;
		const bool		mIdtScale;
		PxMat33			mRot;

		public:
		TriangleProvider(const PxTriangleMeshGeometry& meshGeom, const PxQuat& rot);

		void getTriangle(PxTrianglePadded& tri, const PxTransform& meshPose, PxU32 triangleIndex)	const;
	};
};

TriangleProvider::TriangleProvider(const PxTriangleMeshGeometry& meshGeom, const PxQuat& rot) :
	mTriangles(meshGeom.triangleMesh->getTriangles()),
	mVerts(meshGeom.triangleMesh->getVertices()),
	m16BitIndices(meshGeom.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES),
	mIdtScale(meshGeom.scale.isIdentity())
{
	if(mIdtScale)
	{
		mRot = PxMat33Padded(rot);
	}
	else
	{
		const PxMat33Padded m33(rot);
		mRot = m33 * meshGeom.scale.toMat33();
	}
}

void TriangleProvider::getTriangle(PxTrianglePadded& tri, const PxTransform& meshPose, PxU32 triangleIndex) const
{
	PxU32 vref0, vref1, vref2;
	Gu::getVertexReferences(vref0, vref1, vref2, triangleIndex, mTriangles, m16BitIndices);

	tri.verts[0] = meshPose.p + mRot.transform(mVerts[vref0]);
	tri.verts[1] = meshPose.p + mRot.transform(mVerts[vref1]);
	tri.verts[2] = meshPose.p + mRot.transform(mVerts[vref2]);
}

static PX_FORCE_INLINE void accumulate(PxVec3& disp, PxTransform& meshPose0, const PxVec3& dir, float sep, float offset = 1e-3f)
{
	const PxVec3 accum = dir * (sep - offset);
	disp += accum;
	//meshPose0.p += accum;
	meshPose0.p = disp;	// we can use disp directly here because we moved to mesh0 space
}

static PX_FORCE_INLINE float computeBoundsExtent(const PxTriangle& tri)
{
	PxBounds3 bounds(tri.verts[0], tri.verts[0]);
	bounds.include(tri.verts[1]);
	bounds.include(tri.verts[2]);
	const PxVec3 extents = bounds.getExtents();
	return extents.magnitude();
}

PxVec3 Gu::depenetrateMeshesRef(const PxTriangleMeshGeometry& meshGeom0, const PxTriangleMeshGeometry& meshGeom1, const PxTransform& pose0, const PxTransform& pose1, const PxVec3& inputDir, PxU32 maxIter, PxU32* nbIter)
{
	if(inputDir.isZero())
		return PxVec3(0.0f);

	PX_SIMD_GUARD;

	const TriangleProvider triangleProvider0(meshGeom0, pose0.q);
	const TriangleProvider triangleProvider1(meshGeom1, pose1.q);

	PxTransform meshPose0(PxVec3(0.0f), pose0.q);
	const PxTransform meshPose1(pose1.p - pose0.p, pose1.q);

	const PxVec3 dir = inputDir.getNormalized();

	//float tmax = dir.magnitude();	// ### does this really help?
	float tmax = FLT_MAX;
	//float tmax = 0.0f;

	//const PxGeometryQueryFlags queryFlags = PxGeometryQueryFlag::eDEFAULT;
	const PxGeometryQueryFlags queryFlags = PxGeometryQueryFlags(0);
	//const PxMeshMeshQueryFlags meshMeshFlags = PxMeshMeshQueryFlag::eDEFAULT;
	const PxMeshMeshQueryFlags meshMeshFlags = PxMeshMeshQueryFlag::eDEFAULT|PxMeshMeshQueryFlag::eRESERVED3;
	const float tolerance = 0.0f;

	PxArray<PxGeomIndexPair> overlaps;

	PxVec3 disp(0.0f);
	PxU32 iter = 0;

	while(iter < maxIter)
	{
		iter++;

		class MyPxReportCallback : public PxDynamicArrayReportCallback<PxGeomIndexPair>
		{
			public:
							MyPxReportCallback(PxArray<PxGeomIndexPair>& overlaps) : PxDynamicArrayReportCallback(overlaps)	{}

			PX_FORCE_INLINE	PxU32					getNbPairs()	const	{ return mResults.size();	}
			PX_FORCE_INLINE	const PxGeomIndexPair*	getPairs()		const	{ return mResults.begin();	}
		};

		overlaps.clear();
		MyPxReportCallback callback(overlaps);

		bool status = PxMeshQuery::findOverlapTriangleMesh(callback, meshGeom0, meshPose0, meshGeom1, meshPose1, queryFlags, meshMeshFlags, tolerance);
		PX_UNUSED(status);
		const PxU32 nbPairs = callback.getNbPairs();
		const PxGeomIndexPair* pairs = callback.getPairs();
	
		float sep = 0.0f;
		const bool log = false;
		const bool supportRestart = false;

		PxTrianglePadded tri0, tri1, offsetTri0;
		for(PxU32 i=0;i<nbPairs;i++)
		{
			triangleProvider0.getTriangle(tri0, meshPose0, pairs[i].id0);
			triangleProvider1.getTriangle(tri1, meshPose1, pairs[i].id1);

			float scale = 1.0f;
			bool tryAgain = true;

			float be0 = computeBoundsExtent(tri0);
			float be1 = computeBoundsExtent(tri1);
			
			while(tryAgain)
			{
				tryAgain = false;

				const float offset = be0 + be1 + DEP_OFFSET;

				const PxVec3 d = (sep*scale - offset) * dir;
				offsetTri0.verts[0] = tri0.verts[0] + d;
				offsetTri0.verts[1] = tri0.verts[1] + d;
				offsetTri0.verts[2] = tri0.verts[2] + d;

				float toi;
				if(triTriSweep(offsetTri0, tri1, dir, tmax + offset, toi))
				{
					toi -= offset;
					// PT: the displacement from the first pair might have resolved collisions for subsequent pairs already.
					// In this case we can get a positive TOI, which we should discard (as if the initial overlap query would not
					// have returned these pairs). Failing to do so (i.e. adding positive TOIs to sep) is a mistake, it undoes the
					// progress made so far, can make the query significantly slower, or make it fail entirely.
					if(toi < 0.0f)
					{
						// PT: the time-of-impact can become inaccurate for small triangles whose normals have the same magnitude as
						// our internal epsilon (DIR_DOT_AXIS_EPSILON). Tweaking the epsilon fixed a lot of problems but we can still
						// get incorrect results occasionally. We try to catch these cases here.
						const float limit = (be0 + be1)*2.0f;	// PT: we can't have a penetration depth larger than the size of both tris
						if(-toi > limit)
						{
							// PT: we detected a clearly invalid result and need to decide what to do about it. A strategy that works is
							// to scale up the source triangles by an order of magnitude and recompute the result. Another strategy is
							// simply to discard the result and ignore that pair. Due to the iterative nature of the algorithm other pairs
							// take over and depenetrate meshes anyway. The 'supportRestart' bool selects which strategy is applied.
							if(!tryAgain && scale == 1.0f)
							{
								if(log)
									printf("Invalid result detected: %f %f\n", double(-toi), double(limit));
								if(supportRestart)
								{
									tryAgain = true;

									scale = 10.0f;
									tri0.verts[0] *= scale;
									tri0.verts[1] *= scale;
									tri0.verts[2] *= scale;
									tri1.verts[0] *= scale;
									tri1.verts[1] *= scale;
									tri1.verts[2] *= scale;
									be0 *= scale;
									be1 *= scale;
								}
							}
							else if(log)
								printf("ERROR: invalid result still failing after rescale\n");
						}
						else
						{
							if(log && scale>1.0f)
								printf("Invalid result fixed (success) after rescale\n");

							toi /= scale;
							sep += toi;
						}
					}
					else
					{
						if(log && scale>1.0f)
							printf("Invalid result fixed (discarded) after rescale\n");
					}
				}
			}
		}

		if(sep != 0.0f)
			accumulate(disp, meshPose0, dir, sep);
		else
			break;
	}

	if(nbIter)
		*nbIter = iter;

	return disp;
}

#include "GuMidphaseInterface.h"
#include "GuTriangleMeshBV4.h"
#include "GuBV4_Common.h"

static PX_FORCE_INLINE void setIdentity(PxMat44& m)
{
	m.column0 = PxVec4(1.0f, 0.0f, 0.0f, 0.0f);
	m.column1 = PxVec4(0.0f, 1.0f, 0.0f, 0.0f);
	m.column2 = PxVec4(0.0f, 0.0f, 1.0f, 0.0f);
	m.column3 = PxVec4(0.0f, 0.0f, 0.0f, 1.0f);
}

static PX_FORCE_INLINE void setRotation(PxMat44& m, const PxQuat& q)
{
	const QuatV qV = V4LoadU(&q.x);
	Vec3V column0V, column1V, column2V;
	QuatGetMat33V(qV, column0V, column1V, column2V);

	V4StoreU(Vec4V_From_Vec3V(column0V), &m.column0.x);
	V4StoreU(Vec4V_From_Vec3V(column1V), &m.column1.x);
	V4StoreU(Vec4V_From_Vec3V(column2V), &m.column2.x);
}

bool BV4_OverlapMeshVsMesh(	PxReportCallback<PxGeomIndexPair>& callback,
							const BV4Tree& tree0, const BV4Tree& tree1, const PxMat44* mat0to1, const PxMat44* mat1to0,
							const PxTransform& meshPose0, const PxTransform& meshPose1,
							const PxMeshScale& meshScale0, const PxMeshScale& meshScale1,
							PxMeshMeshQueryFlags meshMeshFlags, float tolerance);

static PX_FORCE_INLINE void updatePos(PxVec4& dst, const PxTransform& src, const PxVec3& p, const PxQuat& qinv)
{
	const PxVec3 v = qinv.rotate(src.p - p);
	dst.x = v.x;
	dst.y = v.y;
	dst.z = v.z;
}

PxVec3 Gu::depenetrateMeshes(const PxTriangleMeshGeometry& meshGeom0, const PxTriangleMeshGeometry& meshGeom1, const PxTransform& pose0, const PxTransform& pose1, const PxVec3& inputDir, PxU32 maxIter, PxU32* nbIter)
{
	if(inputDir.isZero())
		return PxVec3(0.0f);

	PX_SIMD_GUARD;

	const TriangleMesh* tm0 = static_cast<const TriangleMesh*>(meshGeom0.triangleMesh);
	const TriangleMesh* tm1 = static_cast<const TriangleMesh*>(meshGeom1.triangleMesh);

	if(!tm0 || !tm1 || tm0->getConcreteType()!=PxConcreteType::eTRIANGLE_MESH_BVH34 || tm1->getConcreteType()!=PxConcreteType::eTRIANGLE_MESH_BVH34)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Gu::depenetrateMeshes(): only available between two BVH34 triangles meshes.");
		return PxVec3(0.0f);
	}

	/////

	const BV4Tree& tree0 = static_cast<const BV4TriangleMesh*>(tm0)->getBV4Tree();
	const BV4Tree& tree1 = static_cast<const BV4TriangleMesh*>(tm1)->getBV4Tree();

	/////

	class Sweeper : public PxReportCallback<PxGeomIndexPair>
	{
		public:
		const TriangleProvider	mTriangleProvider0;
		const TriangleProvider	mTriangleProvider1;
		PxTransform				mMeshPose0;
		PxTransform				mMeshPose1;
		PxVec3					mDir;

		Sweeper(const PxTriangleMeshGeometry& meshGeom0, const PxTriangleMeshGeometry& meshGeom1, const PxTransform& pose0, const PxTransform& pose1, const PxVec3& dir) :
			mTriangleProvider0(meshGeom0, pose0.q), mTriangleProvider1(meshGeom1, pose1.q)
		{
			this->mBuffer = &mPair;
			this->mCapacity = 1;

			mMeshPose0 = PxTransform(PxVec3(0.0f), pose0.q);
			mMeshPose1 = PxTransform(pose1.p - pose0.p, pose1.q);
			mDir = dir;
			mSep = 0.0f;
		}

		void	reset()
		{
			mSep = 0.0f;
		}

		virtual	bool	flushResults(PxU32 nbItems, const PxGeomIndexPair* pair)
		{
			PX_UNUSED(nbItems);
			PX_ASSERT(nbItems == 1);
			PxTrianglePadded tri0, tri1, offsetTri0;
			mTriangleProvider0.getTriangle(tri0, mMeshPose0, pair->id0);
			mTriangleProvider1.getTriangle(tri1, mMeshPose1, pair->id1);

			//const float offset = computeBoundsExtent(tri0) + computeBoundsExtent(tri1) + DEP_OFFSET;
			const float be0 = computeBoundsExtent(tri0);
			const float be1 = computeBoundsExtent(tri1);
			const float offset = be0 + be1 + DEP_OFFSET;

			const PxVec3 dirOffset = mDir * offset;

			offsetTri0.verts[0] = tri0.verts[0] - dirOffset;
			offsetTri0.verts[1] = tri0.verts[1] - dirOffset;
			offsetTri0.verts[2] = tri0.verts[2] - dirOffset;

			//float tmax = dir.magnitude();	// ### does this really help?
			const float tmax = FLT_MAX;
			//float tmax = 0.0f;

			float toi;
			if(!triTriSweep(offsetTri0, tri1, mDir, tmax + offset, toi/*, 0.0f*/))
				return true;

			toi -= offset;

			// PT: in this version the test is not needed anymore in theory, but we keep it to
			// fight FPU accuracy issues that could potentially make the code produce a +epsilon TOI.
			if(toi < 0.0f)
			{
				// PT: see reference code for why we have this test here
				const float limit = (be0 + be1)*2.0f;	// PT: we can't have a penetration depth larger than the size of both tris
				if(-toi > limit)
				{
					if(0)
						printf("Invalid result detected: %f %f\n", double(-toi), double(limit));
					return true;
				}

				mSep += toi;
				return false;
			}
			return true;
		}

		PxGeomIndexPair	mPair;
		float			mSep;
	};

	//const PxMeshMeshQueryFlags meshMeshFlags = PxMeshMeshQueryFlag::eDEFAULT;
	const PxMeshMeshQueryFlags meshMeshFlags = PxMeshMeshQueryFlag::eDEFAULT|PxMeshMeshQueryFlag::eRESERVED3;

	PxVec3 disp(0.0f);

	PxU32 iter = 0;

	const PxVec3 dir = inputDir.getNormalized();

	Sweeper sweeper(meshGeom0, meshGeom1, pose0, pose1, dir);

/*	const PxU32 nbPasses = 3;
	//const float offsets[3] = { 2e-1f, 1e-2f, 1e-3f };
	const float offsets[3] = { 1e-1f, 1e-2f, 1e-3f };
//	const float offsets[3] = { 1e-1f, 1e-2f, 1e-4f };*/

	const PxU32 nbPasses = 2;
	const float offsets[2] = { 1e-1f, 1e-3f };

//	const PxU32 nbPasses = 2;
//	const float offsets[2] = { 1e-2f, 1e-5f };

//	const PxU32 nbPasses = 1;
//	const float offsets[2] = { 1e-3f };

	PxVec3 prevDisp(0.0f);

	BV4_ALIGN16(PxMat44 World0to1);
	BV4_ALIGN16(PxMat44 World1to0);

	setIdentity(World0to1);
	setIdentity(World1to0);

	const PxMat44* TM0to1 = &World0to1;
	const PxMat44* TM1to0 = &World1to0;
	{
		const PxTransform t0to1 = sweeper.mMeshPose1.transformInv(sweeper.mMeshPose0);
		const PxTransform t1to0 = sweeper.mMeshPose0.transformInv(sweeper.mMeshPose1);

		setRotation(World0to1, t0to1.q);
		setRotation(World1to0, t1to0.q);

		World0to1.column3.x = t0to1.p.x;
		World0to1.column3.y = t0to1.p.y;
		World0to1.column3.z = t0to1.p.z;

		World1to0.column3.x = t1to0.p.x;
		World1to0.column3.y = t1to0.p.y;
		World1to0.column3.z = t1to0.p.z;
	}

	const PxQuat qinv0 = sweeper.mMeshPose0.q.getConjugate();
	const PxQuat qinv1 = sweeper.mMeshPose1.q.getConjugate();

	for(PxU32 pass=0; pass<nbPasses; pass++)
	{
		float offset = offsets[pass];

		while(iter < maxIter)
		{
			iter++;

			sweeper.reset();

			{
				updatePos(World0to1.column3, sweeper.mMeshPose0, sweeper.mMeshPose1.p, qinv1);
				updatePos(World1to0.column3, sweeper.mMeshPose1, sweeper.mMeshPose0.p, qinv0);
				BV4_OverlapMeshVsMesh(sweeper, tree0, tree1, TM0to1, TM1to0, sweeper.mMeshPose0, sweeper.mMeshPose1, meshGeom0.scale, meshGeom1.scale, meshMeshFlags, 0.0f);
			}

			if(sweeper.mSep != 0.0f)
			{
				prevDisp = disp;
				accumulate(disp, sweeper.mMeshPose0, dir, sweeper.mSep, offset);
			}
			else
			{
				//const float offset = 1e-5f;
				const float offset2 = 1e-3f;
				//const float offset = 1e-2f;	// ### 2x faster but "floating" results
				//const float offset = 1e-1f;	// ### 2x faster but "floating" results
				//const float offset = 0.0f;
				const PxVec3 accum = - dir * offset2;
				//disp += accum;
				sweeper.mMeshPose0.p = disp + accum;

				sweeper.reset();

				{
					updatePos(World0to1.column3, sweeper.mMeshPose0, sweeper.mMeshPose1.p, qinv1);
					updatePos(World1to0.column3, sweeper.mMeshPose1, sweeper.mMeshPose0.p, qinv0);
					BV4_OverlapMeshVsMesh(sweeper, tree0, tree1, TM0to1, TM1to0, sweeper.mMeshPose0, sweeper.mMeshPose1, meshGeom0.scale, meshGeom1.scale, meshMeshFlags, 0.0f);
				}

				if(sweeper.mSep != 0.0f)
				{
					// This was a false alarm
					prevDisp = disp;
					disp += accum;
				}
				else
				{
					if(pass != nbPasses - 1)
					{
						disp = prevDisp;
						sweeper.mMeshPose0.p = disp;
					}
					break;
				}
			}
		}
	}

	if(nbIter)
		*nbIter = iter;

	return disp;
}

///////////////////////////////////////////////////////////////////////////////

void Gu::computeDepenetrationVectors(PxU32 nbDirs, PxVec3* dirs)
{
	const int n = int(nbDirs);
	const float goldenRatio = (1.0f + sqrtf(5.0f)) * 0.5f;
	for(int i=0; i<n; i++)
	{
		const float theta = 2.0f * PxPi * i / goldenRatio;
		const float phi = acosf(1.0f - 2.0f * (i + 0.5f) / n);
		const PxVec3 dir(cosf(theta) * sinf(phi), sinf(theta) * sinf(phi), cosf(phi));
		dirs[i] = dir.getNormalized();
	}

	Cm::BasicRandom rnd(42);
	for(int i=0; i<n*100; i++)
	{
		const PxU32 j = rnd.randomize() % n;
		const PxU32 k = rnd.randomize() % n;

		if(j != k)
			PxSwap(dirs[j], dirs[k]);
	}
}

static void getTriangle(PxVec3& p0, PxVec3& p1, PxVec3& p2, PxU32 triangleIndex, const PxMeshScale& scale, const PxTransform* meshPose, const PxVec3* verts, const void* triangles, PxU32 has16BitIndices)
{
	PxU32 vref0, vref1, vref2;
	getVertexReferences(vref0, vref1, vref2, triangleIndex, triangles, has16BitIndices);

	if(scale.isIdentity())
	{
		if(meshPose)
		{
			p0 = meshPose->transform(verts[vref0]);
			p1 = meshPose->transform(verts[vref1]);
			p2 = meshPose->transform(verts[vref2]);
		}
		else
		{
			p0 = verts[vref0];
			p1 = verts[vref1];
			p2 = verts[vref2];
		}
	}
	else
	{
		if(meshPose)
		{
			const PxMat33Padded m33(meshPose->q);
			const PxMat34 absPose(m33 * scale.toMat33(), meshPose->p);

			p0 = absPose.transform(verts[vref0]);
			p1 = absPose.transform(verts[vref1]);
			p2 = absPose.transform(verts[vref2]);
		}
		else
		{
			const PxMat33 absPose = scale.toMat33();

			p0 = absPose.transform(verts[vref0]);
			p1 = absPose.transform(verts[vref1]);
			p2 = absPose.transform(verts[vref2]);
		}
	}
}

void Gu::getTriangle(PxVec3& p0, PxVec3& p1, PxVec3& p2, PxU32 triangleIndex, const PxTriangleMeshGeometry& meshGeom, const PxTransform* meshPose)
{
	const PxTriangleMesh* tm = meshGeom.triangleMesh;
	::getTriangle(p0, p1, p2, triangleIndex, meshGeom.scale, meshPose, tm->getVertices(), tm->getTriangles(), tm->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES);
}

void LocalCluster::reset()
{
	mTris.clear();
	mNormals.clear();
	mTris.setExtraSize(4);	// Allocate 4 more bytes to safely SIMD-load the last vertex
}

static PX_FORCE_INLINE void addTriangleData(LocalCluster& cluster, const PxTriangle& tri)
{
	cluster.mTris.pushBack(tri);

    PxVec3 triNormal;
	tri.normal(triNormal);
    cluster.mNormals.pushBack(triNormal);
}

void LocalCluster::init(const float* src, PxU32 nbFloats)
{
	const PxU32 nbTris = nbFloats / 9;

	PxTriangle tri;
	for(PxU32 i=0; i<nbTris; i++)
	{
		tri.verts[0].x = *src++;
		tri.verts[0].y = *src++;
		tri.verts[0].z = *src++;

		tri.verts[1].x = *src++;
		tri.verts[1].y = *src++;
		tri.verts[1].z = *src++;

		tri.verts[2].x = *src++;
		tri.verts[2].y = *src++;
		tri.verts[2].z = *src++;

		addTriangleData(*this, tri);
	}
}

void LocalCluster::addTriangle(PxU32 triangleIndex, const PxMeshScale& scale, const PxTransform* meshPose, const PxVec3* verts, const void* triangles, PxU32 has16BitIndices)
{
	PxTriangle tri;
	::getTriangle(tri.verts[0], tri.verts[1], tri.verts[2], triangleIndex, scale, meshPose, verts, triangles, has16BitIndices);

	addTriangleData(*this, tri);
}

static void processAdjacent(LocalCluster& mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform* meshPose, const PxU32* faceByEdge, PxU32 nbTris, PxU32 triangleIndex)
{
	PxTriangleMesh* tm = meshGeom.triangleMesh;
	const PxVec3* verts = tm->getVertices();
	const void* triangles = tm->getTriangles();
	const PxU32 has16BitIndices = tm->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	const PxMeshScale& scale = meshGeom.scale;

	for(PxU32 i=0; i<nbTris; i++)
	{
		const PxU32 candidateTriIndex = faceByEdge[i];
		if(candidateTriIndex != triangleIndex)
			mesh.addTriangle(candidateTriIndex, scale, meshPose, verts, triangles, has16BitIndices);
	}
}

static PX_FORCE_INLINE bool canConnect(PxU32 vref0, PxU32 vref1, PxU32 vref0b, PxU32 vref1b)
{
	PX_ASSERT(vref0b <= vref1b);
	return (vref0 == vref0b && vref1 == vref1b);
}

static PX_FORCE_INLINE bool canConnect(PxU32 vref0, PxU32 vref1, PxU32 vref0b, PxU32 vref1b, PxU32 vref2b)
{
	PX_ASSERT(vref0 <= vref1);
	if(canConnect(vref0, vref1, vref0b, vref1b))
		return true;
	if(canConnect(vref0, vref1, vref1b, vref2b))
		return true;
	if(canConnect(vref0, vref1, vref0b, vref2b))
		return true;
	return false;
}

static PX_FORCE_INLINE void sort3(PxU32& vref0, PxU32& vref1, PxU32& vref2)
{
	if(vref0 > vref1)
		PxSwap(vref0, vref1);
	if(vref1 > vref2)
		PxSwap(vref1, vref2);
	if(vref0 > vref1)
		PxSwap(vref0, vref1);
}

void Gu::createLocalCluster(LocalCluster& mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform* meshPose, PxU32 triIndex, const EdgeList* edgeList)
{
	PxTriangleMesh* tm = meshGeom.triangleMesh;
	const PxVec3* verts = tm->getVertices();
	const void* triangles = tm->getTriangles();
	const PxU32 has16BitIndices = tm->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	const PxMeshScale& scale = meshGeom.scale;

	mesh.addTriangle(triIndex, scale, meshPose, verts, triangles, has16BitIndices);

	if(edgeList)
	{
		// PT: TODO: revisit that edgelist code, it's been butchered over the years
		const EdgeTriangleData* edgeTriangleData = edgeList->getEdgeTriangles();
		const EdgeDescData* edgeToTriangle = edgeList->getEdgeToTriangles();
		const PxU32* faceByEdge = edgeList->getFacesByEdges();

		const EdgeTriangleData& edgeTri = edgeTriangleData[triIndex];
		const EdgeDescData& edgeData0 = edgeToTriangle[edgeTri.mLink[0] & MSH_EDGE_LINK_MASK];
		const EdgeDescData& edgeData1 = edgeToTriangle[edgeTri.mLink[1] & MSH_EDGE_LINK_MASK];
		const EdgeDescData& edgeData2 = edgeToTriangle[edgeTri.mLink[2] & MSH_EDGE_LINK_MASK];

		//printf("%d %d %d\n", edgeData0.Count, edgeData1.Count, edgeData2.Count);

		processAdjacent(mesh, meshGeom, meshPose, faceByEdge + edgeData0.Offset, edgeData0.Count, triIndex);
		processAdjacent(mesh, meshGeom, meshPose, faceByEdge + edgeData1.Offset, edgeData1.Count, triIndex);
		processAdjacent(mesh, meshGeom, meshPose, faceByEdge + edgeData2.Offset, edgeData2.Count, triIndex);
	}
	else
	{
		PxU32 vref0, vref1, vref2;
		Gu::getVertexReferences(vref0, vref1, vref2, triIndex, triangles, has16BitIndices);

		const PxVec3& v0 = verts[vref0];
		const PxVec3& v1 = verts[vref1];
		const PxVec3& v2 = verts[vref2];

		PxBounds3 bounds(v0, v0);
		bounds.include(v1);
		bounds.include(v2);

		bounds.fattenFast(1e-3f);

		const PxBoxGeometry boxGeom(bounds.getExtents());
		const PxTransform boxPose(bounds.getCenter());

		const PxTriangleMeshGeometry meshGeomNoScale(tm);

		const PxGeometryQueryFlags queryFlags = PxGeometryQueryFlag::eDEFAULT;
		//const PxGeometryQueryFlags queryFlags = PxGeometryQueryFlag::Enum(0);

		// PT: TODO: why don't we have a callback version for box-vs-mesh findOverlapTriangleMesh ?
		PxU32 results[1024];
		bool overflow;
		const PxU32 nbTris = PxMeshQuery::findOverlapTriangleMesh(boxGeom, boxPose, meshGeomNoScale, PxTransform(PxIdentity), results, 1024, 0, overflow, queryFlags);
		if(nbTris)
		{
			sort3(vref0, vref1, vref2);

			for(PxU32 i=0; i<nbTris; i++)
			{
				const PxU32 candidateTriIndex = results[i];
				if(candidateTriIndex == triIndex)
					continue;

				PxU32 vref0b, vref1b, vref2b;
				Gu::getVertexReferences(vref0b, vref1b, vref2b, candidateTriIndex, triangles, has16BitIndices);

				sort3(vref0b, vref1b, vref2b);

				if(		canConnect(vref0, vref1, vref0b, vref1b, vref2b)
					||	canConnect(vref1, vref2, vref0b, vref1b, vref2b)
					||	canConnect(vref0, vref2, vref0b, vref1b, vref2b)
					)
				{
					// PT: TODO: we fetch the vertex refs again inside addTriangle
					mesh.addTriangle(candidateTriIndex, scale, meshPose, verts, triangles, has16BitIndices);
				}
			}
		}
	}

	{
		const PxU32 nbTris = mesh.mTris.size();
		const PxTriangle* tris = mesh.mTris.begin();

		PxBounds3 bounds = PxBounds3::empty();
		for(PxU32 i=0; i<nbTris; i++)
		{
			const PxTriangle& tri = tris[i];
			bounds.include(tri.verts[0]);
			bounds.include(tri.verts[1]);
			bounds.include(tri.verts[2]);
		}

		mesh.mExtent = bounds.getExtents().magnitude();
	}
}

// PT: this is safe when using the "extra size" parameter in LocalCluster's inline arrays
static PX_FORCE_INLINE const PxTrianglePadded& getPaddedTriangle(const PxTriangle& triangle)
{
	return static_cast<const PxTrianglePadded&>(triangle);
}

float Gu::depenetrateLocalClusters(const LocalCluster& mesh0, const LocalCluster& mesh1, const PxVec3& inputDir, bool iterative)
{
	const PxVec3 dir = inputDir.getNormalized();
	const PxVec3 ndir = -dir;

	const PxU32 nbTris0 = mesh0.mTris.size();
	const PxTriangle* PX_RESTRICT tris0 = mesh0.mTris.begin();
	const PxVec3* PX_RESTRICT normals0 = mesh0.mNormals.begin();
	const PxU32 nbTris1 = mesh1.mTris.size();
	const PxTriangle* PX_RESTRICT tris1 = mesh1.mTris.begin();
    const PxVec3* PX_RESTRICT normals1 = mesh1.mNormals.begin();

	// ### TODO: add early exit if already larger than best

	float minDist = PX_MAX_F32;
	if(!iterative)
	{
		const float offset = mesh0.mExtent + mesh1.mExtent + DEP_OFFSET;

		const PxVec3 dirOffset = dir * offset;

		PxTrianglePadded offsetTri0;
		for(PxU32 i=0; i<nbTris0; i++)
		{
			const PxTriangle& tri0 = tris0[i];

			offsetTri0.verts[0] = tri0.verts[0] + dirOffset;
			offsetTri0.verts[1] = tri0.verts[1] + dirOffset;
			offsetTri0.verts[2] = tri0.verts[2] + dirOffset;

			const PxVec3& triNormal0 = normals0[i];

			for(PxU32 j=0; j<nbTris1; j++)
			{
				const PxTrianglePadded& tri1 = getPaddedTriangle(tris1[j]);

				const PxVec3& triNormal1 = normals1[j];

#if VERIFY_SIMD_CODE
				float toi_ref;
				int status_ref = testSeparationAxes_Scalar(offsetTri0, tri1, triNormal0, triNormal1, ndir, FLT_MAX, toi_ref);
#endif
				float toi;
				int status = testSeparationAxes(offsetTri0, tri1, /*edge0, edge1, */triNormal0, triNormal1, ndir, FLT_MAX, toi);

#if VERIFY_SIMD_CODE
				if((status != status_ref ) || (status && status_ref && toi != toi_ref))
					printf("ERROR! %f %f\n", toi_ref, toi);
#endif
				if(status)
				{
					toi -= offset;
					minDist = PxMin(minDist, toi);
				}
			}
		}
	}
	else
	{
		float disp = 0.0f;

		PxU32 nbIter = 0;

		while(nbIter < 16)
		{
			nbIter++;

			PxU32 nb = 0;

			bool found = false;

			for(PxU32 i=0; i<nbTris0; i++)
			{
				if(found)
					break;

				const PxTriangle& tri0 = tris0[i];
				const PxVec3& triNormal0 = normals0[i];

				const PxVec3 dispVector = dir * disp;
				const PxVec3 v00 = tri0.verts[0] + dispVector;
				const PxVec3 v10 = tri0.verts[1] + dispVector;
				const PxVec3 v20 = tri0.verts[2] + dispVector;

				PxTrianglePadded dispTri0;
				dispTri0.verts[0] = v00;
				dispTri0.verts[1] = v10;
				dispTri0.verts[2] = v20;

				for(PxU32 j=0; j<nbTris1; j++)
				{
					const PxTrianglePadded& tri1 = getPaddedTriangle(tris1[j]);

					const PxVec3& triNormal1 = normals1[j];

					float toi;
					if(testSeparationAxes(dispTri0, tri1, triNormal0, triNormal1, ndir, FLT_MAX, toi))
					{
						//printf("toi %f\n", toi);
						if(toi == 0.0f)
						{
							nb++;

							const float offset = computeBoundsExtent(tri0) + computeBoundsExtent(tri1) + DEP_OFFSET;

							const PxVec3 dirOffset = dir * offset;

							PxTrianglePadded offsetTri0;
							offsetTri0.verts[0] = v00 + dirOffset;
							offsetTri0.verts[1] = v10 + dirOffset;
							offsetTri0.verts[2] = v20 + dirOffset;

							if(testSeparationAxes(offsetTri0, tri1, triNormal0, triNormal1, ndir, FLT_MAX, toi))
							{
								toi -= offset;
								//minDist = PxMin(minDist, toi);

								if(toi < 0.0f)
								{
									disp -= toi;
									disp += 1e-3f;
									found = true;
									break;
								}
							}
						}
					}
				}
			}
			if(!found)
				break;

			//printf("%d %d %f\n", nbIter, nb, disp);
		}

		minDist = -disp;
	}

	//printf("minDist: %f\n", minDist);
	return minDist;
}

static bool testDir(const LocalCluster& mesh0, const LocalCluster& mesh1, const PxVec3& dir, float& dmin, bool testOpposite, bool iterative)
{
	float M0 = depenetrateLocalClusters(mesh0, mesh1, dir, iterative);
	if(M0 < 0.0f)
	{
		M0 = -M0;
	}
	else
	{
		M0 = 0.0f;
		return false;
	}
	dmin = PxMin(dmin, M0);

	if(testOpposite)
	{
		float M1 = depenetrateLocalClusters(mesh0, mesh1, -dir, iterative);
		if(M1 < 0.0f)
		{
			M1 = -M1;
		}
		else
		{
			M1 = 0.0f;
			return false;
		}
		dmin = PxMin(dmin, M1);
	}
	return true;
}

static PX_FORCE_INLINE bool isAlmostZero(const PxVec3& v)
{
	if(fabsf(v.x)>1e-6f || fabsf(v.y)>1e-6f || fabsf(v.z)>1e-6f)
		return false;
	return true;
}

static bool testAxes(const LocalCluster& mesh0, const LocalCluster& mesh1, float& minDistMiniMesh, const MaxLocalDepthContext& context)
{
	const bool iterative = context.mIterative;
	const float minDepthCutoff = context.mMinDepthCutoff;

	if(context.mIncludeSATAxes)
	{
        const PxTriangle& tri0 = mesh0.mTris[0];
        const PxVec3& n0 = mesh0.mNormals[0];
		if(!::testDir(mesh0, mesh1, n0, minDistMiniMesh, true, iterative))
			return false;
		if(minDistMiniMesh <= minDepthCutoff)
			return true;

        const PxTriangle& tri1 = mesh1.mTris[0];
        const PxVec3& n1 = mesh1.mNormals[0];
		if(!::testDir(mesh0, mesh1, n1, minDistMiniMesh, true, iterative))
			return false;
		if(minDistMiniMesh <= minDepthCutoff)
			return true;

		for(int e0=0;e0<3;e0++)
		{
			const PxVec3 edge0 = tri0.verts[e0] - tri0.verts[(e0+1)%3];
			for(int e1=0;e1<3;e1++)
			{
				const PxVec3 edge1 = tri1.verts[e1] - tri1.verts[(e1+1)%3];
				PxVec3 cross = edge0.cross(edge1);
				if(!::isAlmostZero(cross) && !::testDir(mesh0, mesh1, cross.getNormalized(), minDistMiniMesh, true, iterative))
					return false;
				if(minDistMiniMesh <= minDepthCutoff)
					return true;
			}
		}
	}

	const PxVec3* userDirs = context.mUserDirs;
	if(userDirs)
	{
		const PxU32 nbUserDirs = context.mNbUserDirs;
		for(PxU32 i=0; i<nbUserDirs; i++)
		{
			if(!::testDir(mesh0, mesh1, userDirs[i], minDistMiniMesh, false, iterative))
				return false;
			if(minDistMiniMesh <= minDepthCutoff)
				return true;
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

float Gu::computeMaxLocalDepth(
	const PxTriangleMeshGeometry& meshGeom0, const PxTransform& meshPose0, const EdgeList* edgeList0,
	const PxTriangleMeshGeometry& meshGeom1, const PxTransform& meshPose1, const EdgeList* edgeList1,
	PxU32 nbPairs, const PxGeomIndexPair* pairs, const MaxLocalDepthContext& context)
{
	float maxLocalDepth = 0.0f;
	const float maxDepthCutoff = context.mMaxDepthCutoff;

	const PxU32 cacheSize = 64;
	LocalCluster mesh0[cacheSize];
	LocalCluster mesh1[cacheSize];
	PxU32 cacheId0[cacheSize];
	PxU32 cacheId1[cacheSize];
	for(PxU32 k=0; k<cacheSize; k++)
		cacheId0[k] = cacheId1[k] = PX_INVALID_U32;

	const bool runTests = context.mIncludeSATAxes || (context.mNbUserDirs!=0);

	while(nbPairs--)
	{
		const PxU32 id0 = pairs->id0;
		const PxU32 id1 = pairs->id1;
		pairs++;

		//printf("%d %d\n", id0, id1);

		const PxU32 cachedIndex0 = id0 & (cacheSize - 1);
		if(cacheId0[cachedIndex0] != id0)
		{
			cacheId0[cachedIndex0] = id0;
			mesh0[cachedIndex0].reset();
			createLocalCluster(mesh0[cachedIndex0], meshGeom0, &meshPose0, id0, edgeList0);
		}

		const PxU32 cachedIndex1 = id1 & (cacheSize - 1);
		if(cacheId1[cachedIndex1] != id1)
		{
			cacheId1[cachedIndex1] = id1;
			mesh1[cachedIndex1].reset();
			createLocalCluster(mesh1[cachedIndex1], meshGeom1, &meshPose1, id1, edgeList1);
		}

		if(runTests)
		{
			float minDistMiniMesh = FLT_MAX;
			if(!testAxes(mesh0[cachedIndex0], mesh1[cachedIndex1], minDistMiniMesh, context))
				continue;

			if(minDistMiniMesh > maxLocalDepth)
			{
				maxLocalDepth = minDistMiniMesh;
				if(maxLocalDepth >= maxDepthCutoff)
					break;
			}
		}
	}

	return maxLocalDepth;
}

///////////////////////////////////////////////////////////////////////////////

static bool testSepAxis(const PxVec3& sep_axis, const PxVec3* triangle0, const PxVec3* triangle1, float& depth)
{
	float Min0,Max0;
	projectTriangle(sep_axis, triangle0, Min0, Max0);

	float Min1,Max1;
	projectTriangle(sep_axis, triangle1, Min1, Max1);

	if(Max0<Min1 || Max1<Min0)
		return false;

	const float d0 = Max0 - Min1;
	PX_ASSERT(d0>=0.0f);
	const float d1 = Max1 - Min0;
	PX_ASSERT(d1>=0.0f);
	depth = d0<d1 ? d0:d1;
	return true;
}

static int triTriSAT(const PxTriangle& tri0, const PxTriangle& tri1, float& depth, PxVec3& dd)
{
	float dmin = FLT_MAX;
	PxVec3 sep;
	int code = 0;

	{
		PxVec3 n0;
		tri0.normal(n0);

		float d;
		if(!testSepAxis(n0, tri0.verts, tri1.verts, d))
			return false;

		if(d<dmin)
		{
			dmin = d;
			sep = n0;
			code = 1;
		}
	}

	{
		PxVec3 n1;
		tri1.normal(n1);

		float d;
		if(!testSepAxis(n1, tri0.verts, tri1.verts, d))
			return false;

		if(d<dmin)
		{
			dmin = d;
			sep = n1;
			code = 2;
		}
	}

	if(1)
	{
		for(int e0=0;e0<3;e0++)
		{
			const PxVec3 edge0 = tri0.verts[e0] - tri0.verts[(e0+1)%3];

			for(int e1=0;e1<3;e1++)
			{
				const PxVec3 edge1 = tri1.verts[e1] - tri1.verts[(e1+1)%3];

				PxVec3 cross = edge0.cross(edge1);
				if(!::isAlmostZero(cross))
				{
					cross.normalize();

					float d;
					if(!testSepAxis(cross, tri0.verts, tri1.verts, d))
						return false;

					if(d < dmin)
					{
						dmin = d;
						sep = cross;
						code = 3;
					}
				}
			}
		}
	}

    depth = dmin;
	dd = sep;

	return code;
}

float Gu::computeMaxLocalDepthBasic(
	const PxTriangleMeshGeometry& meshGeom0, const PxTransform& meshPose0,
	const PxTriangleMeshGeometry& meshGeom1, const PxTransform& meshPose1,
	PxU32 nbPairs, const PxGeomIndexPair* pairs)
{
	float maxLocalDepth = 0.0f;

	PxTriangle tri0;
	PxTriangle tri1;

	for(PxU32 k=0; k<nbPairs; k++)
	{
		const PxU32 id0 = pairs[k].id0;
		const PxU32 id1 = pairs[k].id1;

		Gu::getTriangle(tri0.verts[0], tri0.verts[1], tri0.verts[2], id0, meshGeom0, &meshPose0);
		Gu::getTriangle(tri1.verts[0], tri1.verts[1], tri1.verts[2], id1, meshGeom1, &meshPose1);

		float depth = 0.0f;
		PxVec3 dd;
		const int code = triTriSAT(tri0, tri1, depth, dd);
		PX_UNUSED(code);

		if(depth > maxLocalDepth)
			maxLocalDepth = depth;
	}
	return maxLocalDepth;
}
