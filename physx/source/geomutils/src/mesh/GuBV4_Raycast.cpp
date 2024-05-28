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

// PT: this one avoids a divide but doesn't seem to make a difference
//#define TEST_DISTANCE_INSIDE_RAY_TRI

// PT: works but unclear performance benefits. Needs more testing.
//#define USE_SIMD_RAY_VS_TRI

#include "PxQueryReport.h"
#include "GuInternal.h"

#include "GuIntersectionRayTriangle.h"

#include "foundation/PxVecMath.h"
using namespace physx::aos;

#include "GuBV4_Common.h"

class RaycastHitInternalUV : public RaycastHitInternal
{
	public:
	PX_FORCE_INLINE			RaycastHitInternalUV()	{}
	PX_FORCE_INLINE			~RaycastHitInternalUV()	{}

					float	mU, mV;
};

#ifdef USE_SIMD_RAY_VS_TRI

//#define DotV	V4Dot
#define DotV	V4Dot3
//PX_FORCE_INLINE FloatV V4Dot3_SSE42(const Vec4V a, const Vec4V b)	{ return _mm_dp_ps(a, b, 0x7f);	}
//#define DotV	V4Dot3_SSE42

template<class T>
static PX_FORCE_INLINE PxIntBool SimdRayTriOverlapT(PxGeomRaycastHit& mStabbedFace, const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, const T* PX_RESTRICT params)
{
	// Find vectors for two edges sharing vert0
	const Vec4V vert0V = V4LoadU(&vert0.x);
	
	const Vec4V edge1V = V4Sub(V4LoadU(&vert1.x), vert0V);	// const PxVec3 edge1 = vert1 - vert0;
	const Vec4V edge2V = V4Sub(V4LoadU(&vert2.x), vert0V);	// const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const Vec4V localDirV = V4LoadU(&params->mLocalDir_Padded.x);
	/*const*/ Vec4V pvecV = V4Cross(localDirV, edge2V);		// const PxVec3 pvec = params->mLocalDir_Padded.cross(edge2);
//pvecV = V4ClearW(pvecV);
	// If determinant is near zero, ray lies in plane of triangle
	const FloatV detV = DotV(edge1V, pvecV);	// const float det = edge1.dot(pvec);

// PT: TODO: if we zero W in pvecV and qvecV we can switch to V4Dot	*************

	// PT: because the SIMD version delays the branches we can share more computations between culling & non-culling codepaths

	// Calculate distance from vert0 to ray origin
	const Vec4V tvecV = V4Sub(V4LoadU(&params->mOrigin_Padded.x), vert0V);	// const PxVec3 tvec = params->mOrigin_Padded - vert0;

	// Prepare to test V parameter
	/*const*/ Vec4V qvecV = V4Cross(tvecV, edge1V);	// const PxVec3 qvec = tvec.cross(edge1);
//qvecV = V4ClearW(qvecV);

	const FloatV localEpsilonV = FLoad(GU_CULLING_EPSILON_RAY_TRIANGLE);
	const FloatV geomEpsilonV = FLoad(params->mGeomEpsilon);

	if(params->mBackfaceCulling)
	{
		// Calculate U parameter and test bounds
		const FloatV uV = DotV(tvecV, pvecV);	// const float u = tvec.dot(pvec);

		// Calculate V parameter and test bounds
		const FloatV vV = DotV(localDirV, qvecV);	// const float v = params->mLocalDir_Padded.dot(qvec);

		// Calculate t, scale parameters, ray intersects triangle
		const FloatV dV = DotV(edge2V, qvecV);	// const float d = edge2.dot(qvec);

		/////

		// if(det<GU_CULLING_EPSILON_RAY_TRIANGLE)
		//	return 0;
		BoolV res = FIsGrtr(localEpsilonV, detV);

		/////

		// const PxReal enlargeCoeff = params->mGeomEpsilon*det;
		const FloatV enlargeCoeffV = FMul(detV, geomEpsilonV);
/*
		const PxReal uvlimit = -enlargeCoeff;
		const PxReal uvlimit2 = det + enlargeCoeff;

		if(u < uvlimit || u > uvlimit2)
			return 0;

		if(v < uvlimit || (u + v) > uvlimit2)
			return 0;

		// Det > 0 so we can early exit here
		// Intersection point is valid if distance is positive (else it can just be a face behind the orig point)
		if(d<0.0f)
			return 0;
*/
		res = BOr(res, FIsGrtr(FZero(), FMin(dV, FMin(FAdd(uV, enlargeCoeffV), FAdd(vV, enlargeCoeffV)))));
		res = BOr(res, FIsGrtr(FMax(uV, FAdd(uV, vV)), FAdd(detV, enlargeCoeffV)));
//		if(!BAllEqFFFF(res))
//			return 0;
		if(BGetBitMask(res))
			return 0;

		// Else go on
		// const float OneOverDet = 1.0f / det;
		const FloatV oneOverDetV = FDiv(FLoad(1.0f), detV);
		FStore(FMul(dV, oneOverDetV), &mStabbedFace.distance);	// mStabbedFace.distance = d * OneOverDet;
		FStore(FMul(uV, oneOverDetV), &mStabbedFace.u);			// mStabbedFace.u = u * OneOverDet;
		FStore(FMul(vV, oneOverDetV), &mStabbedFace.v);			// mStabbedFace.v = v * OneOverDet;
		return 1;
	}
	else
	{
		//if(PxAbs(det)<GU_CULLING_EPSILON_RAY_TRIANGLE)
		//	return 0;
		BoolV res = FIsGrtr(localEpsilonV, FAbs(detV));

		const FloatV oneV = FLoad(1.0f);
		const FloatV oneOverDetV = FDiv(oneV, detV);		// const float OneOverDet = 1.0f / det;

		const FloatV uV = FMul(DotV(tvecV, pvecV), oneOverDetV);	// const float u = tvec.dot(pvec) * OneOverDet;

		// Calculate V parameter and test bounds
		const FloatV vV = FMul(DotV(localDirV, qvecV), oneOverDetV);	// const float v = params->mLocalDir_Padded.dot(qvec) * OneOverDet;

		// Calculate t, ray intersects triangle
		const FloatV dV = FMul(DotV(edge2V, qvecV), oneOverDetV);	// const float d = edge2.dot(qvec) * OneOverDet;
/*
		if(u<-params->mGeomEpsilon || u>1.0f+params->mGeomEpsilon)
			return 0;

		if(v < -params->mGeomEpsilon || (u + v) > 1.0f + params->mGeomEpsilon)
			return 0;

		// Intersection point is valid if distance is positive (else it can just be a face behind the orig point)
		if(d<0.0f)
			return 0;
*/
		res = BOr(res, FIsGrtr(FZero(), FMin(dV, FMin(FAdd(uV, geomEpsilonV), FAdd(vV, geomEpsilonV)))));
		res = BOr(res, FIsGrtr(FMax(uV, FAdd(uV, vV)), FAdd(oneV, geomEpsilonV)));
		//if(!BAllEqFFFF(res))
		//	return 0;
		if(BGetBitMask(res))
			return 0;

		FStore(dV, &mStabbedFace.distance);	// mStabbedFace.distance = d;
		FStore(uV, &mStabbedFace.u);		// mStabbedFace.u = u;
		FStore(vV, &mStabbedFace.v);		// mStabbedFace.v = v;
		return 1;
	}
}

template<class T>
static PX_FORCE_INLINE PxIntBool SimdRayTriOverlapT2(PxGeomRaycastHit& mStabbedFace, const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, const T* PX_RESTRICT params)
{
	// Find vectors for two edges sharing vert0
	const Vec4V vert0V = V4LoadU(&vert0.x);
	
	const Vec4V edge1V = V4Sub(V4LoadU(&vert1.x), vert0V);	// const PxVec3 edge1 = vert1 - vert0;
	const Vec4V edge2V = V4Sub(V4LoadU(&vert2.x), vert0V);	// const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const Vec4V localDirV = V4LoadU(&params->mLocalDir_Padded.x);
	const Vec4V pvecV = V4Cross(localDirV, edge2V);		// const PxVec3 pvec = params->mLocalDir_Padded.cross(edge2);
	// If determinant is near zero, ray lies in plane of triangle
	const FloatV detV = DotV(edge1V, pvecV);	// const float det = edge1.dot(pvec);

	const FloatV localEpsilonV = FLoad(GU_CULLING_EPSILON_RAY_TRIANGLE);

	if(params->mBackfaceCulling)
	{
		// if(det<GU_CULLING_EPSILON_RAY_TRIANGLE)
		//	return 0;
		BoolV res = FIsGrtr(localEpsilonV, detV);
		if(BGetBitMask(res))
			return 0;

		// Calculate distance from vert0 to ray origin
		const Vec4V tvecV = V4Sub(V4LoadU(&params->mOrigin_Padded.x), vert0V);	// const PxVec3 tvec = params->mOrigin_Padded - vert0;

		// Prepare to test V parameter
		const Vec4V qvecV = V4Cross(tvecV, edge1V);	// const PxVec3 qvec = tvec.cross(edge1);

		const FloatV geomEpsilonV = FLoad(params->mGeomEpsilon);

		// Calculate U parameter and test bounds
		const FloatV uV = DotV(tvecV, pvecV);	// const float u = tvec.dot(pvec);

		// Calculate V parameter and test bounds
		const FloatV vV = DotV(localDirV, qvecV);	// const float v = params->mLocalDir_Padded.dot(qvec);

		// Calculate t, scale parameters, ray intersects triangle
		const FloatV dV = DotV(edge2V, qvecV);	// const float d = edge2.dot(qvec);

		/////

		// const PxReal enlargeCoeff = params->mGeomEpsilon*det;
		const FloatV enlargeCoeffV = FMul(detV, geomEpsilonV);

		res = BOr(res, FIsGrtr(FZero(), FMin(dV, FMin(FAdd(uV, enlargeCoeffV), FAdd(vV, enlargeCoeffV)))));
		res = BOr(res, FIsGrtr(FMax(uV, FAdd(uV, vV)), FAdd(detV, enlargeCoeffV)));
//		if(!BAllEqFFFF(res))
//			return 0;
		if(BGetBitMask(res))
			return 0;

		// Else go on
		// const float OneOverDet = 1.0f / det;
		const FloatV oneOverDetV = FDiv(FLoad(1.0f), detV);
		FStore(FMul(dV, oneOverDetV), &mStabbedFace.distance);	// mStabbedFace.distance = d * OneOverDet;
		FStore(FMul(uV, oneOverDetV), &mStabbedFace.u);			// mStabbedFace.u = u * OneOverDet;
		FStore(FMul(vV, oneOverDetV), &mStabbedFace.v);			// mStabbedFace.v = v * OneOverDet;
		return 1;
	}
	else
	{
		//if(PxAbs(det)<GU_CULLING_EPSILON_RAY_TRIANGLE)
		//	return 0;
		BoolV res = FIsGrtr(localEpsilonV, FAbs(detV));
		if(BGetBitMask(res))
			return 0;

		// Calculate distance from vert0 to ray origin
		const Vec4V tvecV = V4Sub(V4LoadU(&params->mOrigin_Padded.x), vert0V);	// const PxVec3 tvec = params->mOrigin_Padded - vert0;

		// Prepare to test V parameter
		const Vec4V qvecV = V4Cross(tvecV, edge1V);	// const PxVec3 qvec = tvec.cross(edge1);

		const FloatV geomEpsilonV = FLoad(params->mGeomEpsilon);


		const FloatV oneV = FLoad(1.0f);
		const FloatV oneOverDetV = FDiv(oneV, detV);		// const float OneOverDet = 1.0f / det;

		const FloatV uV = FMul(DotV(tvecV, pvecV), oneOverDetV);	// const float u = tvec.dot(pvec) * OneOverDet;

		// Calculate V parameter and test bounds
		const FloatV vV = FMul(DotV(localDirV, qvecV), oneOverDetV);	// const float v = params->mLocalDir_Padded.dot(qvec) * OneOverDet;

		// Calculate t, ray intersects triangle
		const FloatV dV = FMul(DotV(edge2V, qvecV), oneOverDetV);	// const float d = edge2.dot(qvec) * OneOverDet;

		res = BOr(res, FIsGrtr(FZero(), FMin(dV, FMin(FAdd(uV, geomEpsilonV), FAdd(vV, geomEpsilonV)))));
		res = BOr(res, FIsGrtr(FMax(uV, FAdd(uV, vV)), FAdd(oneV, geomEpsilonV)));

		if(BGetBitMask(res))
			return 0;

		FStore(dV, &mStabbedFace.distance);	// mStabbedFace.distance = d;
		FStore(uV, &mStabbedFace.u);		// mStabbedFace.u = u;
		FStore(vV, &mStabbedFace.v);		// mStabbedFace.v = v;
		return 1;
	}
}

#endif

template<class T>
static PX_FORCE_INLINE PxIntBool RayTriOverlapT(PxGeomRaycastHit& mStabbedFace, const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, const T* PX_RESTRICT params)
{
#ifdef USE_SIMD_RAY_VS_TRI
	if(0)
		return SimdRayTriOverlapT(mStabbedFace, vert0, vert1, vert2, params);
	if(0)
		return SimdRayTriOverlapT2(mStabbedFace, vert0, vert1, vert2, params);
#endif	

	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = params->mLocalDir_Padded.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const float det = edge1.dot(pvec);

	if(params->mBackfaceCulling)
	{
		if(det<GU_CULLING_EPSILON_RAY_TRIANGLE)
			return 0;

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = params->mOrigin_Padded - vert0;

		// Calculate U parameter and test bounds
		const float u = tvec.dot(pvec);

		const PxReal enlargeCoeff = params->mGeomEpsilon*det;
		const PxReal uvlimit = -enlargeCoeff;
		const PxReal uvlimit2 = det + enlargeCoeff;

		if(u < uvlimit || u > uvlimit2)
			return 0;

		// Prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		const float v = params->mLocalDir_Padded.dot(qvec);
		if(v < uvlimit || (u + v) > uvlimit2)
			return 0;

		// Calculate t, scale parameters, ray intersects triangle
		const float d = edge2.dot(qvec);
		// Det > 0 so we can early exit here
		// Intersection point is valid if distance is positive (else it can just be a face behind the orig point)
		if(d<0.0f)
			return 0;

#ifdef TEST_DISTANCE_INSIDE_RAY_TRI
		if(d>=det*params->mStabbedFace.mDistance)	//### just for a corner case UT in PhysX :(
			return 0;
#endif
		// Else go on
		const float OneOverDet = 1.0f / det;
		mStabbedFace.distance = d * OneOverDet;
		mStabbedFace.u = u * OneOverDet;
		mStabbedFace.v = v * OneOverDet;
	}
	else
	{
		if(PxAbs(det)<GU_CULLING_EPSILON_RAY_TRIANGLE)
			return 0;

		const float OneOverDet = 1.0f / det;

		const PxVec3 tvec = params->mOrigin_Padded - vert0;

		const float u = tvec.dot(pvec) * OneOverDet;
		if(u<-params->mGeomEpsilon || u>1.0f+params->mGeomEpsilon)
			return 0;

		// prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		const float v = params->mLocalDir_Padded.dot(qvec) * OneOverDet;
		if(v < -params->mGeomEpsilon || (u + v) > 1.0f + params->mGeomEpsilon)
			return 0;

		// Calculate t, ray intersects triangle
		const float d = edge2.dot(qvec) * OneOverDet;
		// Intersection point is valid if distance is positive (else it can just be a face behind the orig point)
		if(d<0.0f)
			return 0;

#ifdef TEST_DISTANCE_INSIDE_RAY_TRI
		if(d>=params->mStabbedFace.mDistance)	//### just for a corner case UT in PhysX :(
			return 0;
#endif

		mStabbedFace.distance = d;
		mStabbedFace.u = u;
		mStabbedFace.v = v;
	}
	return 1;
}

#if PX_VC
#pragma warning ( disable : 4324 )
#endif

namespace
{
struct RayParams_Raycast	// PT: compiler gets confused otherwise
{
	BV4_ALIGN16(PxVec3p			mCenterOrMinCoeff_PaddedAligned);
	BV4_ALIGN16(PxVec3p			mExtentsOrMaxCoeff_PaddedAligned);
// Organized in the order they are accessed
#ifndef GU_BV4_USE_SLABS
	BV4_ALIGN16(PxVec3p			mData2_PaddedAligned);
	BV4_ALIGN16(PxVec3p			mFDir_PaddedAligned);
	BV4_ALIGN16(PxVec3p			mData_PaddedAligned);
#endif
	const IndTri32*	PX_RESTRICT	mTris32;
	const IndTri16*	PX_RESTRICT	mTris16;
	const PxVec3*	PX_RESTRICT	mVerts;
	PxVec3						mLocalDir_Padded;
	PxVec3						mOrigin_Padded;

	float						mGeomEpsilon;
	PxU32						mBackfaceCulling;

	RaycastHitInternalUV		mStabbedFace;
	PxU32						mEarlyExit;

	PxVec3						mOriginalExtents_Padded;	// Added to please the slabs code

	BV4_ALIGN16(PxVec3p			mP0_PaddedAligned);
	BV4_ALIGN16(PxVec3p			mP1_PaddedAligned);
	BV4_ALIGN16(PxVec3p			mP2_PaddedAligned);
};
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE void updateParamsAfterImpact(RayParams_Raycast* PX_RESTRICT params, PxU32 primIndex, PxU32 VRef0, PxU32 VRef1, PxU32 VRef2, const PxGeomRaycastHit& StabbedFace)
{
	V4StoreA_Safe(V4LoadU_Safe(&params->mVerts[VRef0].x), &params->mP0_PaddedAligned.x);
	V4StoreA_Safe(V4LoadU_Safe(&params->mVerts[VRef1].x), &params->mP1_PaddedAligned.x);
	V4StoreA_Safe(V4LoadU_Safe(&params->mVerts[VRef2].x), &params->mP2_PaddedAligned.x);

	params->mStabbedFace.mTriangleID = primIndex;
	params->mStabbedFace.mDistance = StabbedFace.distance;
	params->mStabbedFace.mU = StabbedFace.u;
	params->mStabbedFace.mV = StabbedFace.v;
}

namespace
{
class LeafFunction_RaycastClosest
{
public:
	static /*PX_FORCE_INLINE*/ PxIntBool doLeafTest(RayParams_Raycast* PX_RESTRICT params, PxU32 primIndex)
	{
		PX_ALIGN_PREFIX(16)	char buffer[sizeof(PxGeomRaycastHit)] PX_ALIGN_SUFFIX(16);
		PxGeomRaycastHit& StabbedFace = reinterpret_cast<PxGeomRaycastHit&>(buffer);

		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			if(RayTriOverlapT<RayParams_Raycast>(StabbedFace, params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params))
			{
#ifndef TEST_DISTANCE_INSIDE_RAY_TRI
				if(StabbedFace.distance<params->mStabbedFace.mDistance)	//### just for a corner case UT in PhysX :(
#endif
				{
					updateParamsAfterImpact(params, primIndex, VRef0, VRef1, VRef2, StabbedFace);

#ifndef GU_BV4_USE_SLABS
					setupRayData(params, StabbedFace.distance, params->mOrigin_Padded, params->mLocalDir_Padded);
#endif
				}
			}

			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};

class LeafFunction_RaycastAny
{
public:
	static /*PX_FORCE_INLINE*/ PxIntBool doLeafTest(RayParams_Raycast* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			PX_ALIGN_PREFIX(16)	char buffer[sizeof(PxGeomRaycastHit)] PX_ALIGN_SUFFIX(16);
			PxGeomRaycastHit& StabbedFace = reinterpret_cast<PxGeomRaycastHit&>(buffer);
			if(RayTriOverlapT<RayParams_Raycast>(StabbedFace, params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params))
			{
#ifndef TEST_DISTANCE_INSIDE_RAY_TRI
				if(StabbedFace.distance<params->mStabbedFace.mDistance)	//### just for a corner case UT in PhysX :(
#endif
				{
					updateParamsAfterImpact(params, primIndex, VRef0, VRef1, VRef2, StabbedFace);
					return 1;
				}
			}

			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

static PX_FORCE_INLINE Vec4V multiply3x3V_Aligned(const Vec4V p, const PxMat44* PX_RESTRICT mat)	
{
	const FloatV xxxV = V4GetX(p);
	const FloatV yyyV = V4GetY(p);
	const FloatV zzzV = V4GetZ(p);

	Vec4V ResV = V4Scale(V4LoadA(&mat->column0.x), xxxV);
	ResV = V4Add(ResV, V4Scale(V4LoadA(&mat->column1.x), yyyV));
	ResV = V4Add(ResV, V4Scale(V4LoadA(&mat->column2.x), zzzV));
	return ResV;
}

static PX_FORCE_INLINE PxIntBool computeImpactData(PxGeomRaycastHit* PX_RESTRICT hit, const RayParams_Raycast* PX_RESTRICT params, const PxMat44* PX_RESTRICT worldm_Aligned, PxHitFlags /*hitFlags*/)
{
	if(params->mStabbedFace.mTriangleID!=PX_INVALID_U32 /*&& !params->mEarlyExit*/)	//### PhysX needs the raycast data even for "any hit" :(
	{
		const float u = params->mStabbedFace.mU;
		const float v = params->mStabbedFace.mV;
		const float d = params->mStabbedFace.mDistance;
		const PxU32 id = params->mStabbedFace.mTriangleID;
		hit->u = u;
		hit->v = v;
		hit->distance = d;
		hit->faceIndex = id;

		{
			const Vec4V P0V = V4LoadA_Safe(&params->mP0_PaddedAligned.x);
			const Vec4V P1V = V4LoadA_Safe(&params->mP1_PaddedAligned.x);
			const Vec4V P2V = V4LoadA_Safe(&params->mP2_PaddedAligned.x);

			const FloatV uV = FLoad(params->mStabbedFace.mU);
			const FloatV vV = FLoad(params->mStabbedFace.mV);
			const float w = 1.0f - params->mStabbedFace.mU - params->mStabbedFace.mV;
			const FloatV wV = FLoad(w);
			//pt = (1.0f - u - v)*p0 + u*p1 + v*p2;
			Vec4V LocalPtV = V4Scale(P1V, uV);
			LocalPtV = V4Add(LocalPtV, V4Scale(P2V, vV));
			LocalPtV = V4Add(LocalPtV, V4Scale(P0V, wV));

			const Vec4V LocalNormalV = V4Cross(V4Sub(P0V, P1V), V4Sub(P0V, P2V));

			BV4_ALIGN16(PxVec3p tmp_PaddedAligned);
			if(worldm_Aligned)
			{
				const Vec4V TransV = V4LoadA(&worldm_Aligned->column3.x);
				V4StoreU_Safe(V4Add(multiply3x3V_Aligned(LocalPtV, worldm_Aligned), TransV), &hit->position.x);
				V4StoreA_Safe(multiply3x3V_Aligned(LocalNormalV, worldm_Aligned), &tmp_PaddedAligned.x);
			}
			else
			{
				V4StoreU_Safe(LocalPtV, &hit->position.x);
				V4StoreA_Safe(LocalNormalV, &tmp_PaddedAligned.x);
			}
			tmp_PaddedAligned.normalize();
			hit->normal = tmp_PaddedAligned;	// PT: TODO: check asm here (TA34704)
		}
	}
	return params->mStabbedFace.mTriangleID!=PX_INVALID_U32;
}

static PX_FORCE_INLINE float clipRay(const PxVec3& ray_orig, const PxVec3& ray_dir, const LocalBounds& local_bounds)
{
	const float dpc = local_bounds.mCenter.dot(ray_dir);
	const float dpMin = dpc - local_bounds.mExtentsMagnitude;
	const float dpMax = dpc + local_bounds.mExtentsMagnitude;
	const float dpO = ray_orig.dot(ray_dir);
	const float boxLength = local_bounds.mExtentsMagnitude * 2.0f;
	const float distToBox = PxMin(fabsf(dpMin - dpO), fabsf(dpMax - dpO));
	return distToBox + boxLength * 2.0f;
}

template<class ParamsT>
static PX_FORCE_INLINE void setupRayParams(ParamsT* PX_RESTRICT params, const PxVec3& origin, const PxVec3& dir, const BV4Tree* PX_RESTRICT tree, const PxMat44* PX_RESTRICT world, const SourceMesh* PX_RESTRICT mesh, float maxDist, float geomEpsilon, PxU32 flags)
{
	params->mGeomEpsilon = geomEpsilon;
	setupParamsFlags(params, flags);

	computeLocalRay(params->mLocalDir_Padded, params->mOrigin_Padded, dir, origin, world);

	// PT: TODO: clipRay may not be needed with GU_BV4_USE_SLABS (TA34704)
	const float MaxDist = clipRay(params->mOrigin_Padded, params->mLocalDir_Padded, tree->mLocalBounds);
	maxDist = PxMin(maxDist, MaxDist);
	params->mStabbedFace.mDistance = maxDist;
	params->mStabbedFace.mTriangleID = PX_INVALID_U32;

	setupMeshPointersAndQuantizedCoeffs(params, mesh, tree);

#ifndef GU_BV4_USE_SLABS
	setupRayData(params, maxDist, params->mOrigin_Padded, params->mLocalDir_Padded);
#endif
}

#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#include "GuBV4_ProcessStreamOrdered_SegmentAABB.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_KajiyaNoOrder.h"
	#include "GuBV4_Slabs_KajiyaOrdered.h"
#endif

#define GU_BV4_PROCESS_STREAM_RAY_NO_ORDER
#define GU_BV4_PROCESS_STREAM_RAY_ORDERED
#include "GuBV4_Internal.h"

#ifndef GU_BV4_USE_SLABS
#ifdef GU_BV4_QUANTIZED_TREE

// A.B. enable new version only for intel non simd path
#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
//	#define NEW_VERSION
#endif // PX_INTEL_FAMILY && !PX_SIMD_DISABLED

static PX_FORCE_INLINE /*PX_NOINLINE*/ PxIntBool BV4_SegmentAABBOverlap(const BVDataPacked* PX_RESTRICT node, const RayParams* PX_RESTRICT params)
{
#ifdef NEW_VERSION
	SSE_CONST4(maskV,	0x7fffffff);
	SSE_CONST4(maskQV,	0x0000ffff);
#endif
	
#ifdef NEW_VERSION
	Vec4V centerV = V4LoadA((float*)node->mAABB.mData);
	__m128 extentsV = _mm_castsi128_ps(_mm_and_si128(_mm_castps_si128(centerV), SSE_CONST(maskQV)));
	extentsV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(extentsV)), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));
	centerV = _mm_castsi128_ps(_mm_srai_epi32(_mm_castps_si128(centerV), 16));
	centerV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(centerV)), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));
#else
	const VecI32V centerVI = I4LoadA((PxI32*)node->mAABB.mData);	
	const VecI32V extentsVI = VecI32V_And(centerVI, I4Load(0x0000ffff));
	const Vec4V extentsV = V4Mul(Vec4V_From_VecI32V(extentsVI), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));
    const VecI32V centerVShift = VecI32V_RightShift(centerVI, 16);
	const Vec4V centerV = V4Mul(Vec4V_From_VecI32V(centerVShift), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));
#endif

	const Vec4V fdirV = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
	const Vec4V DV = V4Sub(V4LoadA_Safe(&params->mData2_PaddedAligned.x), centerV);

#ifdef NEW_VERSION
	__m128 absDV = _mm_and_ps(DV, SSE_CONSTF(maskV));
#else
	Vec4V absDV = V4Abs(DV);
#endif

	const BoolV resDV = V4IsGrtr(absDV, V4Add(extentsV, fdirV));
	const PxU32 test = BGetBitMask(resDV);
	if(test&7)
		return 0;

	if(1)
	{
		const Vec4V dataZYX_V = V4LoadA_Safe(&params->mData_PaddedAligned.x);
		const Vec4V dataXZY_V = V4Perm<1, 2, 0, 3>(dataZYX_V);			
		const Vec4V DXZY_V = V4Perm<1, 2, 0, 3>(DV);

		const Vec4V fV = V4Sub(V4Mul(dataZYX_V, DXZY_V), V4Mul(dataXZY_V, DV));

		const Vec4V fdirZYX_V = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
		const Vec4V fdirXZY_V = V4Perm<1, 2, 0, 3>(fdirZYX_V);			
		const Vec4V extentsXZY_V = V4Perm<1, 2, 0, 3>(extentsV);

		// PT: TODO: use V4MulAdd here (TA34704)
		const Vec4V fg = V4Add(V4Mul(extentsV, fdirXZY_V), V4Mul(extentsXZY_V, fdirZYX_V));

#ifdef NEW_VERSION
		__m128 absfV = _mm_and_ps(fV, SSE_CONSTF(maskV));
#else
		Vec4V absfV = V4Abs(fV);
#endif
		const BoolV resfV = V4IsGrtr(absfV, fg);
		const PxU32 test2 = BGetBitMask(resfV);
		if(test2&7)
			return 0;
		return 1;
	}
}
#else
static PX_FORCE_INLINE /*PX_NOINLINE*/ PxIntBool BV4_SegmentAABBOverlap(const PxVec3& center, const PxVec3& extents, const RayParams* PX_RESTRICT params)
{
	const PxU32 maskI = 0x7fffffff;

	const Vec4V fdirV = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
	const Vec4V extentsV = V4LoadU(&extents.x);

	const Vec4V DV = V4Sub(V4LoadA_Safe(&params->mData2_PaddedAligned.x), V4LoadU(&center.x));	//###center should be aligned

	__m128 absDV = _mm_and_ps(DV, _mm_load1_ps((float*)&maskI));
	
	absDV = _mm_cmpgt_ps(absDV, V4Add(extentsV, fdirV));
	const PxU32 test = (PxU32)_mm_movemask_ps(absDV);
	if(test&7)
		return 0;

	if(1)
	{
		const Vec4V dataZYX_V = V4LoadA_Safe(&params->mData_PaddedAligned.x);
		const __m128 dataXZY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(dataZYX_V), _MM_SHUFFLE(3,0,2,1)));
		const __m128 DXZY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(DV), _MM_SHUFFLE(3,0,2,1)));
		const Vec4V fV = V4Sub(V4Mul(dataZYX_V, DXZY_V), V4Mul(dataXZY_V, DV));

		const Vec4V fdirZYX_V = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
		const __m128 fdirXZY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(fdirZYX_V), _MM_SHUFFLE(3,0,2,1)));
		const __m128 extentsXZY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,0,2,1)));
		// PT: TODO: use V4MulAdd here (TA34704)
		const Vec4V fg = V4Add(V4Mul(extentsV, fdirXZY_V), V4Mul(extentsXZY_V, fdirZYX_V));

		__m128 absfV = _mm_and_ps(fV, _mm_load1_ps((float*)&maskI));
		absfV = _mm_cmpgt_ps(absfV, fg);
		const PxU32 test2 = (PxU32)_mm_movemask_ps(absfV);
		if(test2&7)
			return 0;
		return 1;
	}
}
#endif
#endif

PxIntBool BV4_RaycastSingle(const PxVec3& origin, const PxVec3& dir, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxGeomRaycastHit* PX_RESTRICT hit, float maxDist, float geomEpsilon, PxU32 flags, PxHitFlags hitFlags)
{
	const SourceMesh* PX_RESTRICT mesh = static_cast<SourceMesh*>(tree.mMeshInterface);

	RayParams_Raycast Params;
	setupRayParams(&Params, origin, dir, &tree, worldm_Aligned, mesh, maxDist, geomEpsilon, flags);

	if(tree.mNodes)
	{
		if(Params.mEarlyExit)
			processStreamRayNoOrder<0, LeafFunction_RaycastAny>(tree, &Params);
		else
			processStreamRayOrdered<0, LeafFunction_RaycastClosest>(tree, &Params);
	}
	else
		doBruteForceTests<LeafFunction_RaycastAny, LeafFunction_RaycastClosest>(mesh->getNbTriangles(), &Params);

	return computeImpactData(hit, &Params, worldm_Aligned, hitFlags);
}



// Callback-based version

namespace
{
struct RayParamsCB : RayParams_Raycast
{
	MeshRayCallback	mCallback;
	void*			mUserData;
};

class LeafFunction_RaycastCB
{
public:
	static PxIntBool doLeafTest(RayParamsCB* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			const PxVec3& p0 = params->mVerts[VRef0];
			const PxVec3& p1 = params->mVerts[VRef1];
			const PxVec3& p2 = params->mVerts[VRef2];

			PX_ALIGN_PREFIX(16)	char buffer[sizeof(PxGeomRaycastHit)] PX_ALIGN_SUFFIX(16);
			PxGeomRaycastHit& StabbedFace = reinterpret_cast<PxGeomRaycastHit&>(buffer);
			if(RayTriOverlapT<RayParams_Raycast>(StabbedFace, p0, p1, p2, params))
			{
#ifndef TEST_DISTANCE_INSIDE_RAY_TRI
				if(StabbedFace.distance<params->mStabbedFace.mDistance)
#endif
				{
					HitCode Code = (params->mCallback)(params->mUserData, p0, p1, p2, primIndex, StabbedFace.distance, StabbedFace.u, StabbedFace.v);
					if(Code==HIT_EXIT)
						return 1;
				}

				// PT: TODO: no shrinking here? (TA34704)
			}

			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};

}

#include "GuBV4_ProcessStreamNoOrder_SegmentAABB.h"

void BV4_RaycastCB(const PxVec3& origin, const PxVec3& dir, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, float maxDist, float geomEpsilon, PxU32 flags, MeshRayCallback callback, void* userData)
{
	const SourceMesh* PX_RESTRICT mesh = static_cast<SourceMesh*>(tree.mMeshInterface);

	//### beware, some parameters in the struct aren't used
	RayParamsCB Params;
	Params.mCallback	= callback;
	Params.mUserData	= userData;
	setupRayParams(&Params, origin, dir, &tree, worldm_Aligned, mesh, maxDist, geomEpsilon, flags);

	if(tree.mNodes)
		processStreamRayNoOrder<0, LeafFunction_RaycastCB>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
//		if(Params.mEarlyExit)
//			LeafFunction_BoxSweepAnyCB::doLeafTest(&Params, nbTris);
//		else
			LeafFunction_RaycastCB::doLeafTest(&Params, nbTris);
	}
}

// Raycast all

namespace
{
struct RayParamsAll : RayParams_Raycast
{
	PX_FORCE_INLINE RayParamsAll(PxGeomRaycastHit* hits, PxU32 maxNbHits, PxU32 stride, const PxMat44* mat, PxHitFlags hitFlags) :
		mHits			(reinterpret_cast<PxU8*>(hits)),
		mNbHits			(0),
		mMaxNbHits		(maxNbHits),
		mStride			(stride),
		mWorld_Aligned	(mat),
		mHitFlags		(hitFlags)
	{
	}

	PxU8*				mHits;
	PxU32				mNbHits;
	const PxU32			mMaxNbHits;
	const PxU32			mStride;
	const PxMat44*		mWorld_Aligned;
	const PxHitFlags	mHitFlags;

	PX_NOCOPY(RayParamsAll)
};

class LeafFunction_RaycastAll
{
public:
	static /*PX_FORCE_INLINE*/ PxIntBool doLeafTest(RayParams_Raycast* PX_RESTRICT p, PxU32 primIndex)
	{
		RayParamsAll* params = static_cast<RayParamsAll*>(p);

		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			PxGeomRaycastHit& StabbedFace = *reinterpret_cast<PxGeomRaycastHit*>(params->mHits);
			if(RayTriOverlapT<RayParams_Raycast>(StabbedFace, params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params))
			{
#ifndef TEST_DISTANCE_INSIDE_RAY_TRI
				if(StabbedFace.distance<params->mStabbedFace.mDistance)
#endif
				{
					updateParamsAfterImpact(params, primIndex, VRef0, VRef1, VRef2, StabbedFace);

					computeImpactData(&StabbedFace, params, params->mWorld_Aligned, params->mHitFlags);

					params->mNbHits++;
					params->mHits += params->mStride;
					if(params->mNbHits==params->mMaxNbHits)
						return 1;
				}
			}
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

// PT: this function is not used yet, but eventually it should be
PxU32 BV4_RaycastAll(const PxVec3& origin, const PxVec3& dir, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxGeomRaycastHit* PX_RESTRICT hits, PxU32 maxNbHits, PxU32 stride, float maxDist, float geomEpsilon, PxU32 flags, PxHitFlags hitFlags)
{
	const SourceMesh* PX_RESTRICT mesh = static_cast<SourceMesh*>(tree.mMeshInterface);

	RayParamsAll Params(hits, maxNbHits, stride, worldm_Aligned, hitFlags);
	setupRayParams(&Params, origin, dir, &tree, worldm_Aligned, mesh, maxDist, geomEpsilon, flags);

	if(tree.mNodes)
		processStreamRayNoOrder<0, LeafFunction_RaycastAll>(tree, &Params);
	else
	{
		PX_ASSERT(0);
	}
	return Params.mNbHits;
}

