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

#ifndef __CU_FEMCLOTHUTIL_CUH__
#define __CU_FEMCLOTHUTIL_CUH__

#include "PxgFEMCloth.h"
#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "copy.cuh"
#include "shuffle.cuh"
#include "assert.h"
#include "stdio.h"
#include "PxgFEMClothCoreKernelIndices.h"
#include "atomic.cuh"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "femMidphaseScratch.cuh"
#include "GuBV32.h"
#include "deformableUtils.cuh"
#include "particleSystem.cuh"
#include "utils.cuh"


using namespace physx;


/*******************************************************************************
 *
 * 
 * Definitions
 * 
 * 
 ******************************************************************************/
 
#define FEMCLOTH_SQRT2		1.4142135623730950488016887242097f
#define FEMCLOTH_SQRT3		1.7320508075688772935274463415059f

#define FEMCLOTH_THRESHOLD	1.0e-12f // to check if the value is near zero
#define FEMCLOTH_PI			3.14159265358979323846f
#define FEMCLOTH_HALF_PI	1.57079632679489661923f
#define FEMCLOTH_2PI		6.28318530717958647692f
#define FEMCLOTH_2PI_INV	0.15915494309189533576888376337251f





/*******************************************************************************
 *
 *
 * Math functions
 *
 *
 ******************************************************************************/

//! 
//! \brief    : Extract rotation R [r0, r1] from F [f0, f1] in 2D 
//! \reference: https://en.wikipedia.org/wiki/Square_root_of_a_2_by_2_matrix
//! 

static PX_FORCE_INLINE __device__ void extractRotation2D(PxVec2& r0, PxVec2& r1, const PxVec2& f0, const PxVec2& f1)
{
	// R: rotation part of F (by polar decopmosition)
	// F^T * F = [S2[0], S2[2]]
	//           [S2[2], S2[1]]
	const PxVec3 S2(f0.dot(f0), f1.dot(f1), f0.dot(f1));
	const float det = S2[0] * S2[1] - S2[2] * S2[2];

	if (det < FEMCLOTH_THRESHOLD)
	{
		r0.x = 1.0f;
		r0.y = 0.0f;
		r1.x = 0.0f;
		r1.y = 1.0f;
		return;
	}

	const float s0 = sqrtf(det);
	const float t = sqrtf(S2[0] + S2[1] + 2.0f * s0);
	
	assert(t > 0.0f);
	if (t < FEMCLOTH_THRESHOLD)
	{
		r0.x = 1.0f;
		r0.y = 0.0f;
		r1.x = 0.0f;
		r1.y = 1.0f;
		return;
	}
	const float tInv = 1.0f / t;

	PxVec3 S(S2);
	S[0] += s0;
	S[1] += s0;
	S *= tInv;

	const float sDet = S[0] * S[1] - S[2] * S[2];
	assert(sDet > 0.0f);
	
	if (sDet < FEMCLOTH_THRESHOLD)
	{
		r0.x = 1.0f;
		r0.y = 0.0f;
		r1.x = 0.0f;
		r1.y = 1.0f;
		return;
	}

	const float sDetInv = 1.0f / sDet;

	PxVec3 SInv(S[1], S[0], -S[2]);
	SInv *= sDetInv;

	// R = [r0 r1]
	r0 = SInv[0] * f0 + SInv[2] * f1;
	r1 = SInv[2] * f0 + SInv[1] * f1;
}



//!
//! \brief    : Approximated atan2: max error of ~1/10000
//! \reference: https://mazzo.li/posts/vectorized-atan2.html
//!

static PX_FORCE_INLINE __device__ PxReal atanApprox(PxReal x)
{
	PxReal x2 = x * x;
	return x * (0.99997726f + x2 * (-0.33262347f + x2 * (0.19354346f + x2 * (-0.11643287f + x2 * (0.05265332f + x2 * (-0.01172120f))))));
}

static PX_FORCE_INLINE __device__ PxReal atan2Approx(PxReal y, PxReal x)
{
	bool swap = PxAbs(x) < PxAbs(y);
	PxReal input = swap ? (x / y) : (y / x);
	PxReal output = atanApprox(input);

	output = swap ? (input >= 0.f ? FEMCLOTH_HALF_PI : -FEMCLOTH_HALF_PI) - output : output;

	if(x < 0.f)
	{
		output += (y >= 0.f ? FEMCLOTH_PI : -FEMCLOTH_PI);
	}

	return output;
}

static PX_FORCE_INLINE __device__ bool velocityClamping(float4& pos, float4& vel, float4& accumDelta, PxReal maxVel, PxReal dt,
														const float4& prevPos)
{
	const PxReal maxVelSq = maxVel * maxVel;
	const PxReal velMagSq = PxLoad3(vel).magnitudeSquared();
	if (velMagSq > maxVelSq)
	{
		vel *= maxVel / PxSqrt(velMagSq);
		float4 newPos = prevPos + vel * dt;
		newPos.w = pos.w;
		vel.w = pos.w;

		const float4 delta = newPos - pos;
		pos = newPos;
		accumDelta += delta;
		return true; // Velocity is clamped.
	}

	return false; // Velocity is not clamped.
}




/*******************************************************************************
 *
 *
 * Delta lambda updates
 *
 *
 ******************************************************************************/

//! 
//! \brief    : Returns the delta lambda in XPBD when a constraint has three vertex degrees of freedom, applicable in both 2D and 3D but without damping.
//! 

template <typename PxVec2Or3>
static PX_FORCE_INLINE __device__ float queryDeltaLambda(float C, const PxVec2Or3& dCdx0, const PxVec2Or3& dCdx1, const PxVec2Or3& dCdx2,
														 float alphaTilde, float lambda, float massInv0, float massInv1, float massInv2)
{
	const float denom =
		(massInv0 * dCdx0.magnitudeSquared() + massInv1 * dCdx1.magnitudeSquared() + massInv2 * dCdx2.magnitudeSquared()) + alphaTilde;
	assert(denom != 0.0f);

	if (denom < FEMCLOTH_THRESHOLD)
		return 0.0f;

	return (-C - alphaTilde * lambda) / denom;
}

//! 
//! \brief    : Returns the delta lambda in XPBD when a constraint has three vertex degrees of freedom, applicable in both 2D and 3D with damping.
//! 

template <typename PxVec2Or3>
static PX_FORCE_INLINE __device__ float queryDeltaLambda(float C, const PxVec2Or3& dCdx0, const PxVec2Or3& dCdx1, const PxVec2Or3& dCdx2,
														 float alphaTilde, float lambda, float massInv0, float massInv1, float massInv2,
														 float damping, float dtInv, float dCdT)
{
	const float denom = (1.0f + damping * dtInv) * (massInv0 * dCdx0.magnitudeSquared() + massInv1 * dCdx1.magnitudeSquared() +
													massInv2 * dCdx2.magnitudeSquared()) +
						alphaTilde;
	assert(denom != 0.0f);

	if (denom < FEMCLOTH_THRESHOLD)
		return 0.0f;

	return -(C + alphaTilde * lambda + damping * dCdT) / denom;
}

//! 
//! \brief    : Returns the delta lambda in XPBD when a constraint has four vertex degrees of freedom, applicable in 3D but without damping.
//! 

static PX_FORCE_INLINE __device__ float queryDeltaLambda(float C, const PxVec3& dCdx0, const PxVec3& dCdx1, const PxVec3& dCdx2,
														 const PxVec3& dCdx3, float alphaTilde, float lambda, float massInv0,
														 float massInv1, float massInv2, float massInv3)
{
	const float denom = (massInv0 * dCdx0.magnitudeSquared() + massInv1 * dCdx1.magnitudeSquared() + massInv2 * dCdx2.magnitudeSquared() +
						 massInv3 * dCdx3.magnitudeSquared()) +
						alphaTilde;
	assert(denom != 0.0f);

	if (denom < FEMCLOTH_THRESHOLD)
		return 0.0f;

	return (-C - alphaTilde * lambda) / denom;
}

//! 
//! \brief    : Returns the delta lambda in XPBD when a constraint has four vertex degrees of freedom, applicable in 3D with damping.
//! 

static PX_FORCE_INLINE __device__ float queryDeltaLambda(float C, const PxVec3& dCdx0, const PxVec3& dCdx1, const PxVec3& dCdx2,
														 const PxVec3& dCdx3, float alphaTilde, float lambda, float massInv0, float massInv1,
														 float massInv2, float massInv3, float damping, float dtInv, const float dCdT)
{
	const float denom = (1.0f + damping * dtInv) * (massInv0 * dCdx0.magnitudeSquared() + massInv1 * dCdx1.magnitudeSquared() +
													massInv2 * dCdx2.magnitudeSquared() + massInv3 * dCdx3.magnitudeSquared()) +
						alphaTilde;
	assert(denom != 0.0f);

	if (denom < FEMCLOTH_THRESHOLD)
		return 0.0f;

	return -(C + alphaTilde * lambda + damping * dCdT) / denom;
}





/*******************************************************************************
 *
 *
 * Deformation gradient and its derivatives
 *
 *
 ******************************************************************************/

//! 
//! \brief    : query deformation gradients (F \in R^{2x2})
//! 

static PX_FORCE_INLINE __device__ void queryDeformationGradient_F2x2(PxVec2& f0, PxVec2& f1, const float4& QInv, const PxVec2& xp01,
																	 const PxVec2& xp02)
{
	f0 = QInv.x * xp01 + QInv.y * xp02;
	f1 = QInv.z * xp01 + QInv.w * xp02;
}



//! 
//! \brief    : compute gradient of constraint (F \in R^{2x2})
//! 

static PX_FORCE_INLINE __device__ void queryConstraintGradient_F2x2(PxVec2& grad1, PxVec2& grad2, const float4& qInv, const PxVec2& pC_pF0,
																	const PxVec2& pC_pF1)
{
	grad1 = qInv.x * pC_pF0 + qInv.z * pC_pF1;
	grad2 = qInv.y * pC_pF0 + qInv.w * pC_pF1;
}





/*******************************************************************************
 *
 *
 * Constraint functions
 *
 *
 ******************************************************************************/

 //! 
 //! \brief    : As-Rigid-As-Possible constraint using {sqrt(||F - R||_F^2)}, F \in R^{2x2}
 //! 

static inline __device__ void ARAPConstraint_F2X2(float& lambda, PxVec2& dx0, PxVec2& dx1, PxVec2& dx2, float alphaTilde,
												  const float4& QInv, const PxVec2& x01, const PxVec2& x02, float massInv0, float massInv1,
												  float massInv2, const PxgFEMCloth& shFEMCloth)
{
	PxVec2 f0, f1, r0, r1;      // F = [f0 f1], R = [r0 r1]
	PxVec2 grad0, grad1, grad2; // gradient of constraint

	queryDeformationGradient_F2x2(f0, f1, QInv, x01, x02);
	extractRotation2D(r0, r1, f0, f1);

	PxVec2 FMinusR0 = f0 - r0;
	PxVec2 FMinusR1 = f1 - r1;

	const float C = sqrt(FMinusR0.dot(FMinusR0) + FMinusR1.dot(FMinusR1)); // ARAP constraint

	if(C > FEMCLOTH_THRESHOLD)
	{
		const float CInv = 1.0f / C;

		// pC/pF = [pCA_pF0 pCA_pF1]
		const PxVec2 pC_pF0 = CInv * FMinusR0;
		const PxVec2 pC_pF1 = CInv * FMinusR1;

		queryConstraintGradient_F2x2(grad1, grad2, QInv, pC_pF0, pC_pF1);
		grad0 = -grad1 - grad2;

		const float deltaLambda = queryDeltaLambda(C, grad0, grad1, grad2, alphaTilde, lambda, massInv0, massInv1, massInv2);
		lambda += deltaLambda;

		dx0 += massInv0 * deltaLambda * grad0;
		dx1 += massInv1 * deltaLambda * grad1;
		dx2 += massInv2 * deltaLambda * grad2;
	}
}



//! 
//! \brief    : Area conservation constraint
//! 

static inline __device__ void areaConstraint_F2X2(float& lambda, PxVec2& dx0, PxVec2& dx1, PxVec2& dx2, float alphaTilde,
												  const float4& QInv, const PxVec2& x01, const PxVec2& x02, float massInv0, float massInv1,
												  float massInv2, float area, const PxgFEMCloth& shFEMCloth)
{
#if 1

	// Area constraints
	// C = |x01 X x02| / |u01 X u02| - 1.0
	const PxReal x01CrossX02 = x01.x * x02.y - x01.y * x02.x;
	const float undeformedAreaInv = 1.0f / area;

	const float C = 0.5f * x01CrossX02 * undeformedAreaInv - 1.0f;

	const PxVec2 grad1(0.5f * undeformedAreaInv * x02.y, -0.5f * undeformedAreaInv * x02.x);
	const PxVec2 grad2(-0.5f * undeformedAreaInv * x01.y, 0.5f * undeformedAreaInv * x01.x);
	const PxVec2 grad0 = -grad1 - grad2;

#else

	// Area constraints
	// C = det(F) - 1, F \in R^ { 2x2 }
	PxVec2 f0, f1, r0, r1;		// F = [f0 f1], R = [r0 r1]
	PxVec2 grad0, grad1, grad2; // gradient of constraint

	queryDeformationGradient_F2x2(f0, f1, QInv, x01, x02);

	const PxReal C = f0.x * f1.y - f0.y * f1.x - 1.0f;

	// pC/pF = [pCA_pF0 pCA_pF1]
	const PxVec2 pC_pF0(f1.y, -f0.y);
	const PxVec2 pC_pF1(-f1.x, f0.x);

	queryConstraintGradient_F2x2(grad1, grad2, QInv, pC_pF0, pC_pF1);
	grad0 = -grad1 - grad2;

#endif

	const float deltaLambda = queryDeltaLambda(C, grad0, grad1, grad2, alphaTilde, lambda, massInv0, massInv1, massInv2);
	lambda += deltaLambda;

	dx0 += massInv0 * deltaLambda * grad0;
	dx1 += massInv1 * deltaLambda * grad1;
	dx2 += massInv2 * deltaLambda * grad2;
}





/*******************************************************************************
 *
 *
 * Energy models
 *
 *
 ******************************************************************************/

//! 
//! \brief    : XPBD formulation of fixed corotated model 
//! 

static __device__ inline void membraneEnergySolvePerTriangle(PxgFEMCloth& shFEMCloth, float4& xx0, float4& xx1, float4& xx2, PxReal dt,
															 const PxsDeformableSurfaceMaterialData& material, const float4& QInv,
															 float vertexScale0, float vertexScale1, float vertexScale2, PxU32 lambdaIndex,
															 bool isShared, bool isTGS)
{
	if (material.youngs < FEMCLOTH_THRESHOLD)
	{
		return;
	}

	PxVec3 x0 = PxLoad3(xx0);
	PxVec3 x1 = PxLoad3(xx1);
	PxVec3 x2 = PxLoad3(xx2);

	const PxVec3 x01 = x1 - x0;
	const PxVec3 x02 = x2 - x0;

	const PxVec3 axis0 = x01.getNormalized();
	PxVec3 normal = x01.cross(x02);
	const PxVec3 axis1 = (normal.cross(axis0)).getNormalized();

	const PxReal dt2 = dt * dt;

	const PxReal det = QInv.x * QInv.w - QInv.y * QInv.z;
	const PxReal area = 1.0f / (2.0f * det);
	const PxReal volume = area * material.thickness;

	PxVec2 dx0(0.0f), dx1(0.0f), dx2(0.0f);
	float lambda0 = 0.0f, lambda1 = 0.0f;

	if (!isTGS)
	{
		lambda0 = isShared ? shFEMCloth.mOrderedSharedTriangleLambdas[lambdaIndex].x : shFEMCloth.mOrderedNonSharedTriangleLambdas[lambdaIndex].x;
		lambda1 = isShared ? shFEMCloth.mOrderedSharedTriangleLambdas[lambdaIndex].y : shFEMCloth.mOrderedNonSharedTriangleLambdas[lambdaIndex].y;
	}

	// Lame's parameters
	const PxPair<PxReal, PxReal> lames = lameParameters(material.youngs, material.poissons);

	// 1) enforcing ARAP constraint
	PxVec2 xp01(axis0.dot(x01), axis1.dot(x01));
	PxVec2 xp02(axis0.dot(x02), axis1.dot(x02));

	// Lame's second parameters
	const PxReal mu = lames.second; 
	const PxReal alphaTilde0 = 1.0f / (2.0f * mu * volume * dt2);

	ARAPConstraint_F2X2(lambda0, dx0, dx1, dx2, alphaTilde0, QInv, xp01, xp02, vertexScale0 * xx0.w, vertexScale1 * xx1.w,
						vertexScale2 * xx2.w, shFEMCloth);

	// 2) enforcing area constraint
	if (material.poissons > FEMCLOTH_THRESHOLD)
	{
		PxReal alphaTilde1 = 0.0f;

		if(material.poissons < 0.5f - FEMCLOTH_THRESHOLD)
		{
			// Lame's first parameters
			const PxReal lambda = lames.first;
			alphaTilde1 = 1.0f / (lambda * volume * dt2);
		}

		xp01 += dx1 - dx0;
		xp02 += dx2 - dx0;

		areaConstraint_F2X2(lambda1, dx0, dx1, dx2, alphaTilde1, QInv, xp01, xp02, vertexScale0 * xx0.w, vertexScale1 * xx1.w,
							vertexScale2 * xx2.w, area, shFEMCloth);
	}

	x0 += dx0.x * axis0 + dx0.y * axis1;
	x1 += dx1.x * axis0 + dx1.y * axis1;
	x2 += dx2.x * axis0 + dx2.y * axis1;

	if (!isTGS)
	{
		if (isShared)
		{
			shFEMCloth.mOrderedSharedTriangleLambdas[lambdaIndex].x = lambda0;
			shFEMCloth.mOrderedSharedTriangleLambdas[lambdaIndex].y = lambda1;
		}
		else
		{
			shFEMCloth.mOrderedNonSharedTriangleLambdas[lambdaIndex].x = lambda0;
			shFEMCloth.mOrderedNonSharedTriangleLambdas[lambdaIndex].y = lambda1;
		}
	}

	xx0.x = x0.x;
	xx0.y = x0.y;
	xx0.z = x0.z;

	xx1.x = x1.x;
	xx1.y = x1.y;
	xx1.z = x1.z;

	xx2.x = x2.x;
	xx2.y = x2.y;
	xx2.z = x2.z;
}



//! 
//! \brief    : XPBD formulation of "Discrete Shells" 
//! 

static __device__ inline void bendingEnergySolvePerTrianglePair(PxgFEMCloth& shFEMCloth, float4& x0, float4& x1, float4& x2, float4& x3,
																const float4& vertexReferenceCounts, float dt, PxU32 trianglePairIndex,
																bool isSharedTrianglePartition, bool isTGS)
{
	const PxVec3 x02 = PxLoad3(x2 - x0);
	const PxVec3 x03 = PxLoad3(x3 - x0);
	const PxVec3 x13 = PxLoad3(x3 - x1);
	const PxVec3 x12 = PxLoad3(x2 - x1);
	const PxVec3 x23 = PxLoad3(x3 - x2);
	const PxReal x23Len = x23.magnitude();

	if(x23Len < FEMCLOTH_THRESHOLD)
		return;

	const PxReal x23LenInv = 1.f / x23Len;
	const PxVec3 x23Normalized = x23 * x23LenInv;

	const float4 restBendingAngle_flexuralStiffness_damping =
		isSharedTrianglePartition ? shFEMCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping[trianglePairIndex]
								  : shFEMCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping[trianglePairIndex];

	const PxReal restBendingAngle = restBendingAngle_flexuralStiffness_damping.x;
	const PxReal kInv = restBendingAngle_flexuralStiffness_damping.y;

	if (kInv <= 0.f)
		return;

	//const PxReal damping = restBendingAngle_flexuralStiffness_damping.z;

	const PxVec3 scaledN0 = x02.cross(x03);
	const PxVec3 scaledN1 = x13.cross(x12);

	const PxReal n0LenInv = 1.f / scaledN0.magnitude();
	const PxReal n1LenInv = 1.f / scaledN1.magnitude();

	const PxVec3 n0 = scaledN0 * n0LenInv;
	PxVec3 n1 = scaledN1 * n1LenInv;

	const PxReal cosAngle = n0.dot(n1);
	const PxReal sinAngle = n0.cross(n1).dot(x23Normalized);
	PxReal angle = atan2f(sinAngle, cosAngle);

	PxReal C = 0.f;
	PxReal alphaTilde = 0.f;
	float dtInv = 1.0f / dt;

	alphaTilde = kInv * dtInv * dtInv;

#if 0 //! incremental angle update

		const PxReal prevBendingAngle = shFEMCloth.mPrevBendingAngles[trianglePairIndex];
		//const PxReal tentativeAngle = atan2(sinAngle, cosAngle);

		const PxReal dif = angle - prevBendingAngle;
		const PxReal sign = (dif > 0.f) ? 1.f : -1.f;
		const PxReal absDif = sign * dif;

		PxReal quotient = floorf(absDif * FEMCLOTH_2PI_INV);
		PxReal residual = absDif - FEMCLOTH_2PI * quotient;

		if(residual > FEMCLOTH_PI)
			quotient += 1.f;

		angle -= sign * quotient * FEMCLOTH_2PI;
		shFEMCloth.mPrevBendingAngles[trianglePairIndex] = angle;

		C = angle - restBendingAngle;

#else //! direct angle update

	C = angle - restBendingAngle;

	if(PxAbs(C + FEMCLOTH_2PI) < PxAbs(C))
	{
		C += FEMCLOTH_2PI;
	}
	else if(PxAbs(C - FEMCLOTH_2PI) < PxAbs(C))
	{
		C -= FEMCLOTH_2PI;
	}

#endif

	// Bending constraint clamped.
	C = PxClamp(C, -FEMCLOTH_HALF_PI, FEMCLOTH_HALF_PI);
	const PxVec3 temp0 = n0 * n0LenInv;
	const PxVec3 temp1 = n1 * n1LenInv;
	const PxVec3 dCdx0 = -x23Len * temp0;
	const PxVec3 dCdx1 = -x23Len * temp1;
	const PxVec3 dCdx2 = x03.dot(x23Normalized) * temp0 + x13.dot(x23Normalized) * temp1;
	const PxVec3 dCdx3 = -(x02.dot(x23Normalized) * temp0 + x12.dot(x23Normalized) * temp1);

	PxReal lambda = 0.0f;

	if (!isTGS)
	{
		lambda = isSharedTrianglePartition ? shFEMCloth.mSharedBendingLambdas[trianglePairIndex] :
			shFEMCloth.mNonSharedBendingLambdas[trianglePairIndex];
	}

	float deltaLambda =
		queryDeltaLambda(C, dCdx0, dCdx1, dCdx2, dCdx3, alphaTilde, lambda, vertexReferenceCounts.x * x0.w, vertexReferenceCounts.y * x1.w,
						 vertexReferenceCounts.z * x2.w, vertexReferenceCounts.w * x3.w);

	if (!isTGS)
	{
		if (isSharedTrianglePartition)
		{
			shFEMCloth.mSharedBendingLambdas[trianglePairIndex] = lambda + deltaLambda;
		}
		else
		{
			shFEMCloth.mNonSharedBendingLambdas[trianglePairIndex] = lambda + deltaLambda;
		}
	}

	PxReal scale0 = vertexReferenceCounts.x * x0.w * deltaLambda;
	x0.x += scale0 * dCdx0.x;
	x0.y += scale0 * dCdx0.y;
	x0.z += scale0 * dCdx0.z;

	PxReal scale1 = vertexReferenceCounts.y * x1.w * deltaLambda;
	x1.x += scale1 * dCdx1.x;
	x1.y += scale1 * dCdx1.y;
	x1.z += scale1 * dCdx1.z;

	PxReal scale2 = vertexReferenceCounts.z * x2.w * deltaLambda;
	x2.x += scale2 * dCdx2.x;
	x2.y += scale2 * dCdx2.y;
	x2.z += scale2 * dCdx2.z;

	PxReal scale3 = vertexReferenceCounts.w * x3.w * deltaLambda;
	x3.x += scale3 * dCdx3.x;
	x3.y += scale3 * dCdx3.y;
	x3.z += scale3 * dCdx3.z;

	return;
}

//! 
//! \brief    : Cloth shell energies in a triangle-pair (two adjacent triangles): in-plane + bending
//! 

static __device__ inline 
void
	clothSharedEnergySolvePerTrianglePair(PxgFEMCloth& shFEMCloth, float4& x0, float4& x1, float4& x2, float4& x3,
										  const float4& vertexReferenceCount, const PxsDeformableSurfaceMaterialData* PX_RESTRICT clothMaterials,
										  float dt, PxU32 trianglePairIndex, bool isTGS)
{
	// shared edge: the shared edge between two adjacent triangles (triangle0, triangle1).
	// edge0, edge1: non-shared edge in triangle0 and triangle1, respectively.
	// tri0Count, tri1Count: the number of references to triangle0 and triangle1 in the entire triangle pairs.
	const float4 restData0 = shFEMCloth.mOrderedSharedRestEdge0_edge1[trianglePairIndex];
	const float4 restData1 = shFEMCloth.mOrderedSharedRestEdgeLength_material0_material1[trianglePairIndex];

	const PxU32 globalMaterialIndex0 = static_cast<PxU32>(restData1.y);
	const PxU32 globalMaterialIndex1 = static_cast<PxU32>(restData1.z);

	const PxVec2 restEdge0(restData0.x, restData0.y);
	const PxVec2 restEdge1(restData0.z, restData0.w);
	const float restSharedEdgeLength = restData1.x;

	const float det0 = restSharedEdgeLength * restEdge0.y;
	const float det1 = restSharedEdgeLength * restEdge1.y;

	// In-plane constraint for triangle0 with vertex x2, x3, and x0.
	if(PxAbs(det0) > FEMCLOTH_THRESHOLD)
	{
		const PxU32 lambdaIndex = 2*trianglePairIndex;
		const float4 QInv0 = make_float4(restEdge0.y, 0.0f, -restEdge0.x, restSharedEdgeLength) / det0;
		membraneEnergySolvePerTriangle(shFEMCloth, x2, x3, x0, dt, clothMaterials[globalMaterialIndex0], QInv0, vertexReferenceCount.z,
									   vertexReferenceCount.w, vertexReferenceCount.x, lambdaIndex, true, isTGS);
	}

	// In-plane constraint for triangle1 with vertex x2, x3, and x1.
	if(PxAbs(det1) > FEMCLOTH_THRESHOLD)
	{
		const PxU32 lambdaIndex = 2 * trianglePairIndex + 1;
		const float4 QInv1 = make_float4(restEdge1.y, 0.0f, -restEdge1.x, restSharedEdgeLength) / det1;
		membraneEnergySolvePerTriangle(shFEMCloth, x2, x3, x1, dt, clothMaterials[globalMaterialIndex1], QInv1, vertexReferenceCount.z,
									   vertexReferenceCount.w, vertexReferenceCount.y, lambdaIndex, true, isTGS);
	}

	// Bending constraint for the triangle pair
	bendingEnergySolvePerTrianglePair(shFEMCloth, x0, x1, x2, x3, vertexReferenceCount, dt, trianglePairIndex, true, isTGS);
}

#endif  // FEMCLOTHUTIL