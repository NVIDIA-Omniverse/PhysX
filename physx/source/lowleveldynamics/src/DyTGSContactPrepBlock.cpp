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

#include "foundation/PxPreprocessor.h"
#include "foundation/PxVecMath.h"
#include "PxcNpWorkUnit.h"
#include "PxcNpContactPrepShared.h"
#include "DyTGSDynamics.h"
#include "DyCpuGpu1dConstraint.h"
#include "DyAllocator.h"

using namespace physx;
using namespace Gu;

#include "PxsMaterialManager.h"
#include "DyContactPrepShared.h"
#include "DyConstraintPrep.h"
#include "DyTGS.h"
#include "DySolverContext.h"

namespace physx
{
namespace Dy
{
	inline bool ValidateVec4(const Vec4V v)
	{
		PX_ALIGN(16, PxVec4 vF);
		aos::V4StoreA(v, &vF.x);
		return vF.isFinite();
	}

	PX_FORCE_INLINE void QuatRotate4(const Vec4VArg qx, const Vec4VArg qy, const Vec4VArg qz, const Vec4VArg qw, const Vec4VArg vx, const Vec4VArg vy, const Vec4VArg vz,
		Vec4V& rX, Vec4V& rY, Vec4V& rZ)
	{
		/*
		const PxVec3 qv(x,y,z);
		return (v*(w*w-0.5f) + (qv.cross(v))*w + qv*(qv.dot(v)))*2;
		*/

		const Vec4V two = V4Splat(FLoad(2.f));
		const Vec4V nhalf = V4Splat(FLoad(-0.5f));
		const Vec4V w2 = V4MulAdd(qw, qw, nhalf);
		const Vec4V ax = V4Mul(vx, w2);
		const Vec4V ay = V4Mul(vy, w2);
		const Vec4V az = V4Mul(vz, w2);

		const Vec4V crX = V4NegMulSub(qz, vy, V4Mul(qy, vz));
		const Vec4V crY = V4NegMulSub(qx, vz, V4Mul(qz, vx));
		const Vec4V crZ = V4NegMulSub(qy, vx, V4Mul(qx, vy));

		const Vec4V tempX = V4MulAdd(crX, qw, ax);
		const Vec4V tempY = V4MulAdd(crY, qw, ay);
		const Vec4V tempZ = V4MulAdd(crZ, qw, az);

		Vec4V dotuv = V4Mul(qx, vx);
		dotuv = V4MulAdd(qy, vy, dotuv);
		dotuv = V4MulAdd(qz, vz, dotuv);

		rX = V4Mul(V4MulAdd(qx, dotuv, tempX), two);
		rY = V4Mul(V4MulAdd(qy, dotuv, tempY), two);
		rZ = V4Mul(V4MulAdd(qz, dotuv, tempZ), two);
	}

struct SolverContactHeaderStepBlock
{
	enum
	{
		eHAS_MAX_IMPULSE = 1 << 0,
		eHAS_TARGET_VELOCITY = 1 << 1
	};

	PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
	PxU8	numNormalConstr;
	PxU8	numFrictionConstr;
	PxU8	flag;

	PxU8	flags[4];

	//KS - used for write-back only
	PxU8	numNormalConstrs[4];
	PxU8	numFrictionConstrs[4];

	//Vec4V	restitution;
	Vec4V   staticFriction;
	Vec4V	dynamicFriction;
	//Technically, these mass properties could be pulled out into a new structure and shared. For multi-manifold contacts,
	//this would save 64 bytes per-manifold after the cost of the first manifold
	Vec4V	invMass0D0;
	Vec4V	invMass1D1;
	Vec4V	angDom0;
	Vec4V	angDom1;
	//Normal is shared between all contacts in the batch. This will save some memory!
	Vec4V normalX;
	Vec4V normalY;
	Vec4V normalZ;

	Vec4V maxPenBias;

	Sc::ShapeInteraction* shapeInteraction[4];		//192 or 208

	BoolV broken;
	PxU8* frictionBrokenWritebackByte[4];
};

struct SolverContactPointStepBlock
{
	Vec4V raXnI[3];
	Vec4V rbXnI[3];
	Vec4V separation;
	Vec4V velMultiplier;
	Vec4V targetVelocity;
	Vec4V biasCoefficient;
	Vec4V recipResponse;
};

//KS - technically, this friction constraint has identical data to the above contact constraint.
//We make them separate structs for clarity
struct SolverContactFrictionStepBlock
{
	Vec4V normal[3];
	Vec4V raXnI[3];
	Vec4V rbXnI[3];
	Vec4V error;
	Vec4V velMultiplier;
	Vec4V targetVel;
	Vec4V biasCoefficient;
};

struct SolverConstraint1DHeaderStep4
{
	PxU8	type;			// enum SolverConstraintType - must be first byte
	PxU8	pad0[3];
	//These counts are the max of the 4 sets of data.
	//When certain pairs have fewer constraints than others, they are padded with 0s so that no work is performed but 
	//calculations are still shared (afterall, they're computationally free because we're doing 4 things at a time in SIMD)
	PxU32	count;
	PxU8	counts[4];
	PxU8	breakable[4];

	Vec4V	linBreakImpulse;
	Vec4V	angBreakImpulse;
	Vec4V	invMass0D0;
	Vec4V	invMass1D1;
	Vec4V	angD0;
	Vec4V	angD1;

	Vec4V	body0WorkOffset[3];
	Vec4V	rAWorld[3];
	Vec4V	rBWorld[3];

	Vec4V	angOrthoAxis0X[3];
	Vec4V	angOrthoAxis0Y[3];
	Vec4V	angOrthoAxis0Z[3];
	Vec4V	angOrthoAxis1X[3];
	Vec4V	angOrthoAxis1Y[3];
	Vec4V	angOrthoAxis1Z[3];
	Vec4V	angOrthoRecipResponse[3];
	Vec4V	angOrthoError[3];	
};

PX_ALIGN_PREFIX(16)
struct SolverConstraint1DStep4
{
public:
	Vec4V		lin0[3];				//!< linear velocity projection (body 0)	
	Vec4V		error;					//!< constraint error term - must be scaled by biasScale. Can be adjusted at run-time

	Vec4V		lin1[3];				//!< linear velocity projection (body 1)
	Vec4V		biasScale;				//!< constraint constant bias scale. Constant

	Vec4V		ang0[3];				//!< angular velocity projection (body 0)
	Vec4V		velMultiplier;			//!< constraint velocity multiplier

	Vec4V		ang1[3];				//!< angular velocity projection (body 1)

	Vec4V		velTarget;				//!< Scaled target velocity of the constraint drive

	Vec4V		minImpulse;				//!< Lower bound on impulse magnitude	 
	Vec4V		maxImpulse;				//!< Upper bound on impulse magnitude
	Vec4V		appliedForce;			//!< applied force to correct velocity+bias

	Vec4V		maxBias;
	Vec4V		angularErrorScale;		//Constant
	PxU32		flags[4];
} PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct SolverConstraint1DStep4WithResidual : public SolverConstraint1DStep4
{
	Vec4V		residualVelIter;
	Vec4V		residualPosIter;
}PX_ALIGN_SUFFIX(16);

static void setupFinalizeSolverConstraints4Step(PxTGSSolverContactDesc* PX_RESTRICT descs, CorrelationBuffer& c,
	PxU8* PX_RESTRICT workspace, PxReal invDtF32, PxReal totalDtF32, PxReal invTotalDtF32,
	PxReal dtF32, PxReal bounceThresholdF32, PxReal biasCoefficient,
	const aos::Vec4VArg invMassScale0, const aos::Vec4VArg invInertiaScale0,
	const aos::Vec4VArg invMassScale1, const aos::Vec4VArg invInertiaScale1)
{
	//OK, we have a workspace of pre-allocated space to store all 4 descs in. We now need to create the constraints in it

	//const Vec4V ccdMaxSeparation = aos::V4LoadXYZW(descs[0].maxCCDSeparation, descs[1].maxCCDSeparation, descs[2].maxCCDSeparation, descs[3].maxCCDSeparation);
	const Vec4V solverOffsetSlop = aos::V4LoadXYZW(descs[0].offsetSlop, descs[1].offsetSlop, descs[2].offsetSlop, descs[3].offsetSlop);

	const Vec4V zero = V4Zero();
	const Vec4V one = V4One();
	const BoolV bFalse = BFFFF();
	const BoolV bTrue = BTTTT();
	const FloatV fZero = FZero();

	PxU8 flags[4] = { PxU8(descs[0].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
		PxU8(descs[1].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
		PxU8(descs[2].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0),
		PxU8(descs[3].hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0) };

	const bool hasMaxImpulse = descs[0].hasMaxImpulse || descs[1].hasMaxImpulse || descs[2].hasMaxImpulse || descs[3].hasMaxImpulse;

	//The block is dynamic if **any** of the constraints have a non-static body B. This allows us to batch static and non-static constraints but we only get a memory/perf
	//saving if all 4 are static. This simplifies the constraint partitioning such that it only needs to care about separating contacts and 1D constraints (which it already does)
	bool isDynamic = false;
	bool hasKinematic = false;
	
	PxReal kinematicScale0F32[4];
	PxReal kinematicScale1F32[4];

	for (PxU32 a = 0; a < 4; ++a)
	{
		isDynamic = isDynamic || (descs[a].bodyState1 == PxSolverContactDesc::eDYNAMIC_BODY);
		hasKinematic = hasKinematic || descs[a].bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY;
		kinematicScale0F32[a] = descs[a].body0->isKinematic ? 1.f : 0.f;
		kinematicScale1F32[a] = descs[a].body1->isKinematic ? 1.f : 0.f;
	}

	/*BoolV kinematic0 = BLoad(isKinematic0);
	BoolV kinematic1 = BLoad(isKinematic1);*/

	const Vec4V kinematicScale0 = V4LoadU(kinematicScale0F32);
	const Vec4V kinematicScale1 = V4LoadU(kinematicScale1F32);

	const PxU32 constraintSize = sizeof(SolverContactPointStepBlock);
	const PxU32 frictionSize = sizeof(SolverContactFrictionStepBlock);

	PxU8* PX_RESTRICT ptr = workspace;

	const Vec4V dom0 = invMassScale0;
	const Vec4V dom1 = invMassScale1;
	const Vec4V angDom0 = invInertiaScale0;
	const Vec4V angDom1 = invInertiaScale1;

	const Vec4V maxPenBias = V4Max(V4LoadXYZW(descs[0].bodyData0->penBiasClamp, descs[1].bodyData0->penBiasClamp,
		descs[2].bodyData0->penBiasClamp, descs[3].bodyData0->penBiasClamp),
		V4LoadXYZW(descs[0].bodyData1->penBiasClamp, descs[1].bodyData1->penBiasClamp,
			descs[2].bodyData1->penBiasClamp, descs[3].bodyData1->penBiasClamp));

	const Vec4V restDistance = V4LoadXYZW(descs[0].restDistance, descs[1].restDistance, descs[2].restDistance,
		descs[3].restDistance);

	//load up velocities
	Vec4V linVel00 = V4LoadA(&descs[0].bodyData0->originalLinearVelocity.x);
	Vec4V linVel10 = V4LoadA(&descs[1].bodyData0->originalLinearVelocity.x);
	Vec4V linVel20 = V4LoadA(&descs[2].bodyData0->originalLinearVelocity.x);
	Vec4V linVel30 = V4LoadA(&descs[3].bodyData0->originalLinearVelocity.x);

	Vec4V linVel01 = V4LoadA(&descs[0].bodyData1->originalLinearVelocity.x);
	Vec4V linVel11 = V4LoadA(&descs[1].bodyData1->originalLinearVelocity.x);
	Vec4V linVel21 = V4LoadA(&descs[2].bodyData1->originalLinearVelocity.x);
	Vec4V linVel31 = V4LoadA(&descs[3].bodyData1->originalLinearVelocity.x);

	Vec4V angVel00 = V4LoadA(&descs[0].bodyData0->originalAngularVelocity.x);
	Vec4V angVel10 = V4LoadA(&descs[1].bodyData0->originalAngularVelocity.x);
	Vec4V angVel20 = V4LoadA(&descs[2].bodyData0->originalAngularVelocity.x);
	Vec4V angVel30 = V4LoadA(&descs[3].bodyData0->originalAngularVelocity.x);

	Vec4V angVel01 = V4LoadA(&descs[0].bodyData1->originalAngularVelocity.x);
	Vec4V angVel11 = V4LoadA(&descs[1].bodyData1->originalAngularVelocity.x);
	Vec4V angVel21 = V4LoadA(&descs[2].bodyData1->originalAngularVelocity.x);
	Vec4V angVel31 = V4LoadA(&descs[3].bodyData1->originalAngularVelocity.x);

	Vec4V linVelT00, linVelT10, linVelT20;
	Vec4V linVelT01, linVelT11, linVelT21;
	Vec4V angVelT00, angVelT10, angVelT20;
	Vec4V angVelT01, angVelT11, angVelT21;

	PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVelT00, linVelT10, linVelT20);
	PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVelT01, linVelT11, linVelT21);
	PX_TRANSPOSE_44_34(angVel00, angVel10, angVel20, angVel30, angVelT00, angVelT10, angVelT20);
	PX_TRANSPOSE_44_34(angVel01, angVel11, angVel21, angVel31, angVelT01, angVelT11, angVelT21);

	const Vec4V vrelX = V4Sub(linVelT00, linVelT01);
	const Vec4V vrelY = V4Sub(linVelT10, linVelT11);
	const Vec4V vrelZ = V4Sub(linVelT20, linVelT21);

	//Load up masses and invInertia

	const Vec4V invMass0 = V4LoadXYZW(descs[0].bodyData0->invMass, descs[1].bodyData0->invMass, descs[2].bodyData0->invMass, descs[3].bodyData0->invMass);
	const Vec4V invMass1 = V4LoadXYZW(descs[0].bodyData1->invMass, descs[1].bodyData1->invMass, descs[2].bodyData1->invMass, descs[3].bodyData1->invMass);

	const Vec4V invMass0D0 = V4Mul(dom0, invMass0);
	const Vec4V invMass1D1 = V4Mul(dom1, invMass1);

	Vec4V invInertia00X = V4LoadU(&descs[0].body0TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia00Y = V4LoadU(&descs[0].body0TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia00Z = Vec4V_From_Vec3V(V3LoadU(descs[0].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia10X = V4LoadU(&descs[1].body0TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia10Y = V4LoadU(&descs[1].body0TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia10Z = Vec4V_From_Vec3V(V3LoadU(descs[1].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia20X = V4LoadU(&descs[2].body0TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia20Y = V4LoadU(&descs[2].body0TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia20Z = Vec4V_From_Vec3V(V3LoadU(descs[2].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia30X = V4LoadU(&descs[3].body0TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia30Y = V4LoadU(&descs[3].body0TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia30Z = Vec4V_From_Vec3V(V3LoadU(descs[3].body0TxI->sqrtInvInertia.column2));

	Vec4V invInertia01X = V4LoadU(&descs[0].body1TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia01Y = V4LoadU(&descs[0].body1TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia01Z = Vec4V_From_Vec3V(V3LoadU(descs[0].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia11X = V4LoadU(&descs[1].body1TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia11Y = V4LoadU(&descs[1].body1TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia11Z = Vec4V_From_Vec3V(V3LoadU(descs[1].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia21X = V4LoadU(&descs[2].body1TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia21Y = V4LoadU(&descs[2].body1TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia21Z = Vec4V_From_Vec3V(V3LoadU(descs[2].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia31X = V4LoadU(&descs[3].body1TxI->sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia31Y = V4LoadU(&descs[3].body1TxI->sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia31Z = Vec4V_From_Vec3V(V3LoadU(descs[3].body1TxI->sqrtInvInertia.column2));

	Vec4V invInertia0X0, invInertia0X1, invInertia0X2;
	Vec4V invInertia0Y0, invInertia0Y1, invInertia0Y2;
	Vec4V invInertia0Z0, invInertia0Z1, invInertia0Z2;

	Vec4V invInertia1X0, invInertia1X1, invInertia1X2;
	Vec4V invInertia1Y0, invInertia1Y1, invInertia1Y2;
	Vec4V invInertia1Z0, invInertia1Z1, invInertia1Z2;

	PX_TRANSPOSE_44_34(invInertia00X, invInertia10X, invInertia20X, invInertia30X, invInertia0X0, invInertia0Y0, invInertia0Z0);
	PX_TRANSPOSE_44_34(invInertia00Y, invInertia10Y, invInertia20Y, invInertia30Y, invInertia0X1, invInertia0Y1, invInertia0Z1);
	PX_TRANSPOSE_44_34(invInertia00Z, invInertia10Z, invInertia20Z, invInertia30Z, invInertia0X2, invInertia0Y2, invInertia0Z2);

	PX_TRANSPOSE_44_34(invInertia01X, invInertia11X, invInertia21X, invInertia31X, invInertia1X0, invInertia1Y0, invInertia1Z0);
	PX_TRANSPOSE_44_34(invInertia01Y, invInertia11Y, invInertia21Y, invInertia31Y, invInertia1X1, invInertia1Y1, invInertia1Z1);
	PX_TRANSPOSE_44_34(invInertia01Z, invInertia11Z, invInertia21Z, invInertia31Z, invInertia1X2, invInertia1Y2, invInertia1Z2);

	const FloatV invDt = FLoad(invDtF32);
	const PxReal scale = PxMin(0.8f, biasCoefficient);
	
	const FloatV p8 = FLoad(scale);
	const FloatV frictionBiasScale = FMul(invDt, p8);
	const Vec4V totalDt = V4Load(totalDtF32);
	const FloatV invTotalDt = FLoad(invTotalDtF32);
	
	const Vec4V p84 = V4Splat(p8);
	const Vec4V bounceThreshold = V4Splat(FLoad(bounceThresholdF32));

	const Vec4V invDtp8 = V4Splat(FMul(invDt, p8));

	const FloatV dt = FLoad(dtF32);

	Vec4V bodyFrame00p4 = V4LoadU(&descs[0].bodyFrame0.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase
	Vec4V bodyFrame01p4 = V4LoadU(&descs[1].bodyFrame0.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase
	Vec4V bodyFrame02p4 = V4LoadU(&descs[2].bodyFrame0.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase
	Vec4V bodyFrame03p4 = V4LoadU(&descs[3].bodyFrame0.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase

	Vec4V bodyFrame0pX, bodyFrame0pY, bodyFrame0pZ;
	PX_TRANSPOSE_44_34(bodyFrame00p4, bodyFrame01p4, bodyFrame02p4, bodyFrame03p4, bodyFrame0pX, bodyFrame0pY, bodyFrame0pZ);

	Vec4V bodyFrame10p4 = V4LoadU(&descs[0].bodyFrame1.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase
	Vec4V bodyFrame11p4 = V4LoadU(&descs[1].bodyFrame1.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase
	Vec4V bodyFrame12p4 = V4LoadU(&descs[2].bodyFrame1.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase
	Vec4V bodyFrame13p4 = V4LoadU(&descs[3].bodyFrame1.p.x);	// PT: safe because of compile-time-assert in PxTGSSolverConstraintPrepDescBase

	Vec4V bodyFrame1pX, bodyFrame1pY, bodyFrame1pZ;
	PX_TRANSPOSE_44_34(bodyFrame10p4, bodyFrame11p4, bodyFrame12p4, bodyFrame13p4, bodyFrame1pX, bodyFrame1pY, bodyFrame1pZ);

	const QuatV bodyFrame00q = QuatVLoadU(&descs[0].bodyFrame0.q.x);
	const QuatV bodyFrame01q = QuatVLoadU(&descs[1].bodyFrame0.q.x);
	const QuatV bodyFrame02q = QuatVLoadU(&descs[2].bodyFrame0.q.x);
	const QuatV bodyFrame03q = QuatVLoadU(&descs[3].bodyFrame0.q.x);

	const QuatV bodyFrame10q = QuatVLoadU(&descs[0].bodyFrame1.q.x);
	const QuatV bodyFrame11q = QuatVLoadU(&descs[1].bodyFrame1.q.x);
	const QuatV bodyFrame12q = QuatVLoadU(&descs[2].bodyFrame1.q.x);
	const QuatV bodyFrame13q = QuatVLoadU(&descs[3].bodyFrame1.q.x);

	PxU32 frictionPatchWritebackAddrIndex0 = 0;
	PxU32 frictionPatchWritebackAddrIndex1 = 0;
	PxU32 frictionPatchWritebackAddrIndex2 = 0;
	PxU32 frictionPatchWritebackAddrIndex3 = 0;

	PxPrefetchLine(c.contactID);
	PxPrefetchLine(c.contactID, 128);

	PxU32 frictionIndex0 = 0, frictionIndex1 = 0, frictionIndex2 = 0, frictionIndex3 = 0;
	//PxU32 contactIndex0 = 0, contactIndex1 = 0, contactIndex2 = 0, contactIndex3 = 0;

	//OK, we iterate through all friction patch counts in the constraint patch, building up the constraint list etc.

	PxU32 maxPatches = PxMax(descs[0].numFrictionPatches, PxMax(descs[1].numFrictionPatches, PxMax(descs[2].numFrictionPatches, descs[3].numFrictionPatches)));

	const Vec4V p1 = V4Splat(FLoad(0.0001f));
	const Vec4V orthoThreshold = V4Splat(FLoad(0.70710678f));

	PxU32 contact0 = 0, contact1 = 0, contact2 = 0, contact3 = 0;
	PxU32 patch0 = 0, patch1 = 0, patch2 = 0, patch3 = 0;

	PxU8 flag = 0;
	if (hasMaxImpulse)
		flag |= SolverContactHeader4::eHAS_MAX_IMPULSE;

	bool hasFinished[4];

	for (PxU32 i = 0; i<maxPatches; i++)
	{
		hasFinished[0] = i >= descs[0].numFrictionPatches;
		hasFinished[1] = i >= descs[1].numFrictionPatches;
		hasFinished[2] = i >= descs[2].numFrictionPatches;
		hasFinished[3] = i >= descs[3].numFrictionPatches;

		frictionIndex0 = hasFinished[0] ? frictionIndex0 : descs[0].startFrictionPatchIndex + i;
		frictionIndex1 = hasFinished[1] ? frictionIndex1 : descs[1].startFrictionPatchIndex + i;
		frictionIndex2 = hasFinished[2] ? frictionIndex2 : descs[2].startFrictionPatchIndex + i;
		frictionIndex3 = hasFinished[3] ? frictionIndex3 : descs[3].startFrictionPatchIndex + i;

		PxU32 clampedContacts0 = hasFinished[0] ? 0 : c.frictionPatchContactCounts[frictionIndex0];
		PxU32 clampedContacts1 = hasFinished[1] ? 0 : c.frictionPatchContactCounts[frictionIndex1];
		PxU32 clampedContacts2 = hasFinished[2] ? 0 : c.frictionPatchContactCounts[frictionIndex2];
		PxU32 clampedContacts3 = hasFinished[3] ? 0 : c.frictionPatchContactCounts[frictionIndex3];

		PxU32 firstPatch0 = c.correlationListHeads[frictionIndex0];
		PxU32 firstPatch1 = c.correlationListHeads[frictionIndex1];
		PxU32 firstPatch2 = c.correlationListHeads[frictionIndex2];
		PxU32 firstPatch3 = c.correlationListHeads[frictionIndex3];

		const PxContactPoint* contactBase0 = descs[0].contacts + c.contactPatches[firstPatch0].start;
		const PxContactPoint* contactBase1 = descs[1].contacts + c.contactPatches[firstPatch1].start;
		const PxContactPoint* contactBase2 = descs[2].contacts + c.contactPatches[firstPatch2].start;
		const PxContactPoint* contactBase3 = descs[3].contacts + c.contactPatches[firstPatch3].start;

		// negative restitution
		const Vec4V restitution = V4Neg(V4LoadXYZW(contactBase0->restitution, contactBase1->restitution, contactBase2->restitution,
			contactBase3->restitution));
		const Vec4V damping = V4LoadXYZW(contactBase0->damping, contactBase1->damping, contactBase2->damping, contactBase3->damping);
		const bool accelSpring_[] = { !!(contactBase0->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING),
			                          !!(contactBase1->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING),
			                          !!(contactBase2->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING),
			                          !!(contactBase3->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING) };
		const BoolV accelSpring = BLoad(accelSpring_);

		SolverContactHeaderStepBlock* PX_RESTRICT header = reinterpret_cast<SolverContactHeaderStepBlock*>(ptr);
		ptr += sizeof(SolverContactHeaderStepBlock);

		header->flags[0] = flags[0];
		header->flags[1] = flags[1];
		header->flags[2] = flags[2];
		header->flags[3] = flags[3];

		header->flag = flag;

		PxU32 totalContacts = PxMax(clampedContacts0, PxMax(clampedContacts1, PxMax(clampedContacts2, clampedContacts3)));

		Vec4V* PX_RESTRICT appliedNormalForces = reinterpret_cast<Vec4V*>(ptr);
		ptr += sizeof(Vec4V)*totalContacts;

		PxMemZero(appliedNormalForces, sizeof(Vec4V) * totalContacts);

		header->numNormalConstr = PxTo8(totalContacts);
		header->numNormalConstrs[0] = PxTo8(clampedContacts0);
		header->numNormalConstrs[1] = PxTo8(clampedContacts1);
		header->numNormalConstrs[2] = PxTo8(clampedContacts2);
		header->numNormalConstrs[3] = PxTo8(clampedContacts3);
		//header->sqrtInvMassA = sqrtInvMass0;
		//header->sqrtInvMassB = sqrtInvMass1;
		header->invMass0D0 = invMass0D0;
		header->invMass1D1 = invMass1D1;
		header->angDom0 = angDom0;
		header->angDom1 = angDom1;
		header->shapeInteraction[0] = getInteraction(descs[0]); header->shapeInteraction[1] = getInteraction(descs[1]);
		header->shapeInteraction[2] = getInteraction(descs[2]); header->shapeInteraction[3] = getInteraction(descs[3]);

		Vec4V* maxImpulse = reinterpret_cast<Vec4V*>(ptr + constraintSize * totalContacts);

		//header->restitution = restitution;

		Vec4V normal0 = V4LoadA(&contactBase0->normal.x);
		Vec4V normal1 = V4LoadA(&contactBase1->normal.x);
		Vec4V normal2 = V4LoadA(&contactBase2->normal.x);
		Vec4V normal3 = V4LoadA(&contactBase3->normal.x);

		Vec4V normalX, normalY, normalZ;
		PX_TRANSPOSE_44_34(normal0, normal1, normal2, normal3, normalX, normalY, normalZ);

		PX_ASSERT(ValidateVec4(normalX));
		PX_ASSERT(ValidateVec4(normalY));
		PX_ASSERT(ValidateVec4(normalZ));

		header->normalX = normalX;
		header->normalY = normalY;
		header->normalZ = normalZ;

		header->maxPenBias = maxPenBias;

		const Vec4V norVel0 = V4MulAdd(normalZ, linVelT20, V4MulAdd(normalY, linVelT10, V4Mul(normalX, linVelT00)));
		const Vec4V norVel1 = V4MulAdd(normalZ, linVelT21, V4MulAdd(normalY, linVelT11, V4Mul(normalX, linVelT01)));
		const Vec4V relNorVel = V4Sub(norVel0, norVel1);

		//For all correlation heads - need to pull this out I think

		//OK, we have a counter for all our patches...
		PxU32 finished = (PxU32(hasFinished[0])) |
			((PxU32(hasFinished[1])) << 1) |
			((PxU32(hasFinished[2])) << 2) |
			((PxU32(hasFinished[3])) << 3);

		CorrelationListIterator iter0(c, firstPatch0);
		CorrelationListIterator iter1(c, firstPatch1);
		CorrelationListIterator iter2(c, firstPatch2);
		CorrelationListIterator iter3(c, firstPatch3);

		//PxU32 contact0, contact1, contact2, contact3;
		//PxU32 patch0, patch1, patch2, patch3;

		if (!hasFinished[0])
			iter0.nextContact(patch0, contact0);
		if (!hasFinished[1])
			iter1.nextContact(patch1, contact1);
		if (!hasFinished[2])
			iter2.nextContact(patch2, contact2);
		if (!hasFinished[3])
			iter3.nextContact(patch3, contact3);

		PxU8* p = ptr;

		PxU32 contactCount = 0;
		PxU32 newFinished =
			(PxU32(hasFinished[0] || !iter0.hasNextContact())) |
			((PxU32(hasFinished[1] || !iter1.hasNextContact())) << 1) |
			((PxU32(hasFinished[2] || !iter2.hasNextContact())) << 2) |
			((PxU32(hasFinished[3] || !iter3.hasNextContact())) << 3);

		BoolV bFinished = BLoad(hasFinished);

		while (finished != 0xf)
		{
			finished = newFinished;
			++contactCount;
			PxPrefetchLine(p, 384);
			PxPrefetchLine(p, 512);
			PxPrefetchLine(p, 640);

			SolverContactPointStepBlock* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointStepBlock*>(p);
			p += constraintSize;

			const PxContactPoint& con0 = descs[0].contacts[c.contactPatches[patch0].start + contact0];
			const PxContactPoint& con1 = descs[1].contacts[c.contactPatches[patch1].start + contact1];
			const PxContactPoint& con2 = descs[2].contacts[c.contactPatches[patch2].start + contact2];
			const PxContactPoint& con3 = descs[3].contacts[c.contactPatches[patch3].start + contact3];

			//Now we need to splice these 4 contacts into a single structure

			{
				Vec4V point0 = V4LoadA(&con0.point.x);
				Vec4V point1 = V4LoadA(&con1.point.x);
				Vec4V point2 = V4LoadA(&con2.point.x);
				Vec4V point3 = V4LoadA(&con3.point.x);

				Vec4V pointX, pointY, pointZ;
				PX_TRANSPOSE_44_34(point0, point1, point2, point3, pointX, pointY, pointZ);

				Vec4V cTargetVel0 = V4LoadA(&con0.targetVel.x);
				Vec4V cTargetVel1 = V4LoadA(&con1.targetVel.x);
				Vec4V cTargetVel2 = V4LoadA(&con2.targetVel.x);
				Vec4V cTargetVel3 = V4LoadA(&con3.targetVel.x);

				Vec4V cTargetVelX, cTargetVelY, cTargetVelZ;
				PX_TRANSPOSE_44_34(cTargetVel0, cTargetVel1, cTargetVel2, cTargetVel3, cTargetVelX, cTargetVelY, cTargetVelZ);

				const Vec4V separation = V4LoadXYZW(con0.separation, con1.separation, con2.separation, con3.separation);

				const Vec4V cTargetNorVel = V4MulAdd(cTargetVelX, normalX, V4MulAdd(cTargetVelY, normalY, V4Mul(cTargetVelZ, normalZ)));

				const Vec4V raX = V4Sub(pointX, bodyFrame0pX);
				const Vec4V raY = V4Sub(pointY, bodyFrame0pY);
				const Vec4V raZ = V4Sub(pointZ, bodyFrame0pZ);

				const Vec4V rbX = V4Sub(pointX, bodyFrame1pX);
				const Vec4V rbY = V4Sub(pointY, bodyFrame1pY);
				const Vec4V rbZ = V4Sub(pointZ, bodyFrame1pZ);

				/*raX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raX)), zero, raX);
				raY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raY)), zero, raY);
				raZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raZ)), zero, raZ);

				rbX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbX)), zero, rbX);
				rbY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbY)), zero, rbY);
				rbZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbZ)), zero, rbZ);*/

				PX_ASSERT(ValidateVec4(raX));
				PX_ASSERT(ValidateVec4(raY));
				PX_ASSERT(ValidateVec4(raZ));

				PX_ASSERT(ValidateVec4(rbX));
				PX_ASSERT(ValidateVec4(rbY));
				PX_ASSERT(ValidateVec4(rbZ));

				//raXn = cross(ra, normal) which = Vec3V( a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);

				Vec4V raXnX = V4NegMulSub(raZ, normalY, V4Mul(raY, normalZ));
				Vec4V raXnY = V4NegMulSub(raX, normalZ, V4Mul(raZ, normalX));
				Vec4V raXnZ = V4NegMulSub(raY, normalX, V4Mul(raX, normalY));

				Vec4V rbXnX = V4NegMulSub(rbZ, normalY, V4Mul(rbY, normalZ));
				Vec4V rbXnY = V4NegMulSub(rbX, normalZ, V4Mul(rbZ, normalX));
				Vec4V rbXnZ = V4NegMulSub(rbY, normalX, V4Mul(rbX, normalY));

				const Vec4V relAngVel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4Mul(raXnX, angVelT00)));
				const Vec4V relAngVel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4Mul(rbXnX, angVelT01)));

				const Vec4V relAng = V4Sub(relAngVel0, relAngVel1);

				const Vec4V slop = V4Mul(solverOffsetSlop, V4Max(V4Sel(V4IsEq(relNorVel, zero), V4Splat(FMax()), V4Div(relAng, relNorVel)), V4One()));

				raXnX = V4Sel(V4IsGrtr(slop, V4Abs(raXnX)), zero, raXnX);
				raXnY = V4Sel(V4IsGrtr(slop, V4Abs(raXnY)), zero, raXnY);
				raXnZ = V4Sel(V4IsGrtr(slop, V4Abs(raXnZ)), zero, raXnZ);

				Vec4V delAngVel0X = V4Mul(invInertia0X0, raXnX);
				Vec4V delAngVel0Y = V4Mul(invInertia0X1, raXnX);
				Vec4V delAngVel0Z = V4Mul(invInertia0X2, raXnX);

				delAngVel0X = V4MulAdd(invInertia0Y0, raXnY, delAngVel0X);
				delAngVel0Y = V4MulAdd(invInertia0Y1, raXnY, delAngVel0Y);
				delAngVel0Z = V4MulAdd(invInertia0Y2, raXnY, delAngVel0Z);

				delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, delAngVel0X);
				delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, delAngVel0Y);
				delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, delAngVel0Z);

				PX_ASSERT(ValidateVec4(delAngVel0X));
				PX_ASSERT(ValidateVec4(delAngVel0Y));
				PX_ASSERT(ValidateVec4(delAngVel0Z));

				const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0X, delAngVel0X, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0Z, delAngVel0Z)));
				
				Vec4V unitResponse = V4MulAdd(dotDelAngVel0, angDom0, invMass0D0);
				Vec4V vrel0 = V4Add(norVel0, relAngVel0);
				Vec4V vrel1 = V4Add(norVel1, relAngVel1);

				Vec4V delAngVel1X = zero;
				Vec4V delAngVel1Y = zero;
				Vec4V delAngVel1Z = zero;

				//The dynamic-only parts - need to if-statement these up. A branch here shouldn't cost us too much
				if (isDynamic)
				{
					rbXnX = V4Sel(V4IsGrtr(slop, V4Abs(rbXnX)), zero, rbXnX);
					rbXnY = V4Sel(V4IsGrtr(slop, V4Abs(rbXnY)), zero, rbXnY);
					rbXnZ = V4Sel(V4IsGrtr(slop, V4Abs(rbXnZ)), zero, rbXnZ);

					delAngVel1X = V4Mul(invInertia1X0, rbXnX);
					delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
					delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

					delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
					delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
					delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

					delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
					delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
					delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);

					PX_ASSERT(ValidateVec4(delAngVel1X));
					PX_ASSERT(ValidateVec4(delAngVel1Y));
					PX_ASSERT(ValidateVec4(delAngVel1Z));

					const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1X, delAngVel1X, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1Z, delAngVel1Z)));

					const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

					unitResponse = V4Add(unitResponse, resp1);
				}

				Vec4V vrel = V4Sub(vrel0, vrel1);

				solverContact->rbXnI[0] = delAngVel1X;
				solverContact->rbXnI[1] = delAngVel1Y;
				solverContact->rbXnI[2] = delAngVel1Z;
				
				Vec4V penetration = V4Sub(separation, restDistance);

				const Vec4V penetrationInvDt = V4Scale(penetration, invTotalDt);
				const BoolV isSeparated = V4IsGrtr(penetration, zero);

				const BoolV isGreater2 = BAnd(BAnd(V4IsGrtr(zero, restitution), V4IsGrtr(bounceThreshold, vrel)),
						V4IsGrtr(V4Neg(vrel), penetrationInvDt));

				const Vec4V ratio = V4Sel(isGreater2, V4Add(totalDt, V4Div(penetration, vrel)), zero);

				const Vec4V recipResponse = V4Sel(V4IsGrtr(unitResponse, zero), V4Recip(unitResponse), zero);

				//Restitution is negated in the block setup code
				// rdt, a, b, x only needed in compliant case
				const BoolV isCompliant = V4IsGrtr(restitution, zero);
				const Vec4V rdt = V4Scale(restitution, dt);
				const BoolV collidingWithVrel = V4IsGrtr(V4Mul(V4Neg(vrel), totalDt), penetration); // Note: using totalDt here instead of penetrationInvDt because the latter has a fudge factor if there are velocity iterations
				const Vec4V dampingIfEnabled = V4Sel(BAndNot(isSeparated, collidingWithVrel), zero, damping);
				const Vec4V a = V4Scale(V4Add(dampingIfEnabled, rdt), dt);
				const Vec4V massIfAccelElseOne = V4Sel(accelSpring, recipResponse, one);
				const Vec4V oneIfAccelElseR = V4Sel(accelSpring, one, unitResponse);
				const Vec4V x = V4Recip(V4MulAdd(a, oneIfAccelElseR, one));
				const Vec4V velMultiplier = V4Sel(isCompliant, V4Mul(V4Mul(x, a), massIfAccelElseOne), recipResponse);

				// biasCoeff includes the unit response s.t. velDeltaFromPosError = separation*biasCoeff 
				const Vec4V scaledBias = V4Neg(V4Sel(isCompliant, V4Mul(rdt, V4Mul(x, oneIfAccelElseR)), V4Sel(isSeparated, V4Splat(invDt), invDtp8)));

				Vec4V targetVelocity = V4NegMulSub(vrel0, kinematicScale0, V4MulAdd(vrel1, kinematicScale1, V4Sel(isGreater2, V4Mul(vrel, restitution), zero)));

				penetration = V4MulAdd(targetVelocity, ratio, penetration);

				//Vec4V biasedErr = V4Sel(isGreater2, targetVelocity, scaledBias);
				//Vec4V biasedErr = V4Add(targetVelocity, scaledBias);

				//biasedErr = V4NegMulSub(V4Sub(vrel, cTargetNorVel), velMultiplier, biasedErr);

				//These values are present for static and dynamic contacts			
				solverContact->raXnI[0] = delAngVel0X;
				solverContact->raXnI[1] = delAngVel0Y;
				solverContact->raXnI[2] = delAngVel0Z;
				solverContact->velMultiplier = V4Sel(bFinished, zero, velMultiplier);
				solverContact->targetVelocity = V4Add(cTargetNorVel, targetVelocity);
				solverContact->separation = penetration;
				solverContact->biasCoefficient = V4Sel(bFinished, zero, scaledBias);
				solverContact->recipResponse = V4Sel(bFinished, zero, recipResponse);

				if (hasMaxImpulse)
				{
					maxImpulse[contactCount - 1] = V4Merge(FLoad(con0.maxImpulse), FLoad(con1.maxImpulse), FLoad(con2.maxImpulse),
						FLoad(con3.maxImpulse));
				}
			}
			if (!(finished & 0x1))
			{
				iter0.nextContact(patch0, contact0);
				newFinished |= PxU32(!iter0.hasNextContact());
			}
			else
				bFinished = BSetX(bFinished, bTrue);

			if (!(finished & 0x2))
			{
				iter1.nextContact(patch1, contact1);
				newFinished |= (PxU32(!iter1.hasNextContact()) << 1);
			}
			else
				bFinished = BSetY(bFinished, bTrue);

			if (!(finished & 0x4))
			{
				iter2.nextContact(patch2, contact2);
				newFinished |= (PxU32(!iter2.hasNextContact()) << 2);
			}
			else
				bFinished = BSetZ(bFinished, bTrue);

			if (!(finished & 0x8))
			{
				iter3.nextContact(patch3, contact3);
				newFinished |= (PxU32(!iter3.hasNextContact()) << 3);
			}
			else
				bFinished = BSetW(bFinished, bTrue);
		}
		ptr = p;
		if (hasMaxImpulse)
		{
			ptr += sizeof(Vec4V) * totalContacts;
		}

		//OK...friction time :-)

		Vec4V maxImpulseScale = V4One();
		{
			const FrictionPatch& frictionPatch0 = c.frictionPatches[frictionIndex0];
			const FrictionPatch& frictionPatch1 = c.frictionPatches[frictionIndex1];
			const FrictionPatch& frictionPatch2 = c.frictionPatches[frictionIndex2];
			const FrictionPatch& frictionPatch3 = c.frictionPatches[frictionIndex3];

			PxU32 anchorCount0 = frictionPatch0.anchorCount;
			PxU32 anchorCount1 = frictionPatch1.anchorCount;
			PxU32 anchorCount2 = frictionPatch2.anchorCount;
			PxU32 anchorCount3 = frictionPatch3.anchorCount;

			PxU32 clampedAnchorCount0 = hasFinished[0] || (contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount0;
			PxU32 clampedAnchorCount1 = hasFinished[1] || (contactBase1->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount1;
			PxU32 clampedAnchorCount2 = hasFinished[2] || (contactBase2->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount2;
			PxU32 clampedAnchorCount3 = hasFinished[3] || (contactBase3->materialFlags & PxMaterialFlag::eDISABLE_FRICTION) ? 0 : anchorCount3;

			const PxU32 maxAnchorCount = PxMax(clampedAnchorCount0, PxMax(clampedAnchorCount1, PxMax(clampedAnchorCount2, clampedAnchorCount3)));

			PX_ALIGN(16, PxReal staticFriction[4]);
			PX_ALIGN(16, PxReal dynamicFriction[4]);

			//for (PxU32 f = 0; f < 4; ++f)
			{
				PxReal coeff0 = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount0 == 2) ? 0.5f : 1.f;
				PxReal coeff1 = (contactBase1->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount1 == 2) ? 0.5f : 1.f;
				PxReal coeff2 = (contactBase2->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount2 == 2) ? 0.5f : 1.f;
				PxReal coeff3 = (contactBase3->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && clampedAnchorCount3 == 2) ? 0.5f : 1.f;

				staticFriction[0] = contactBase0->staticFriction * coeff0;
				dynamicFriction[0] = contactBase0->dynamicFriction * coeff0;
				staticFriction[1] = contactBase1->staticFriction * coeff1;
				dynamicFriction[1] = contactBase1->dynamicFriction * coeff1;
				staticFriction[2] = contactBase2->staticFriction * coeff2;
				dynamicFriction[2] = contactBase2->dynamicFriction * coeff2;
				staticFriction[3] = contactBase3->staticFriction * coeff3;
				dynamicFriction[3] = contactBase3->dynamicFriction * coeff3;
			}

			PX_ASSERT(totalContacts == contactCount);
			
			header->numFrictionConstr = PxTo8(maxAnchorCount * 2);
			header->numFrictionConstrs[0] = PxTo8(clampedAnchorCount0 * 2);
			header->numFrictionConstrs[1] = PxTo8(clampedAnchorCount1 * 2);
			header->numFrictionConstrs[2] = PxTo8(clampedAnchorCount2 * 2);
			header->numFrictionConstrs[3] = PxTo8(clampedAnchorCount3 * 2);

			//KS - TODO - extend this if needed
			header->type = PxTo8(DY_SC_TYPE_BLOCK_RB_CONTACT);

			if (maxAnchorCount)
			{
				const BoolV cond = V4IsGrtr(orthoThreshold, V4Abs(normalX));

				const Vec4V t0FallbackX = V4Sel(cond, zero, V4Neg(normalY));
				const Vec4V t0FallbackY = V4Sel(cond, V4Neg(normalZ), normalX);
				const Vec4V t0FallbackZ = V4Sel(cond, normalY, zero);

				//const Vec4V dotNormalVrel = V4MulAdd(normalZ, vrelZ, V4MulAdd(normalY, vrelY, V4Mul(normalX, vrelX)));
				const Vec4V vrelSubNorVelX = V4NegMulSub(normalX, relNorVel, vrelX);
				const Vec4V vrelSubNorVelY = V4NegMulSub(normalY, relNorVel, vrelY);
				const Vec4V vrelSubNorVelZ = V4NegMulSub(normalZ, relNorVel, vrelZ);

				const Vec4V lenSqvrelSubNorVelZ = V4MulAdd(vrelSubNorVelX, vrelSubNorVelX, V4MulAdd(vrelSubNorVelY, vrelSubNorVelY, V4Mul(vrelSubNorVelZ, vrelSubNorVelZ)));

				const BoolV bcon2 = V4IsGrtr(lenSqvrelSubNorVelZ, p1);

				Vec4V t0X = V4Sel(bcon2, vrelSubNorVelX, t0FallbackX);
				Vec4V t0Y = V4Sel(bcon2, vrelSubNorVelY, t0FallbackY);
				Vec4V t0Z = V4Sel(bcon2, vrelSubNorVelZ, t0FallbackZ);

				//Now normalize this...
				const Vec4V recipLen = V4Rsqrt(V4MulAdd(t0Z, t0Z, V4MulAdd(t0Y, t0Y, V4Mul(t0X, t0X))));

				t0X = V4Mul(t0X, recipLen);
				t0Y = V4Mul(t0Y, recipLen);
				t0Z = V4Mul(t0Z, recipLen);

				Vec4V t1X = V4NegMulSub(normalZ, t0Y, V4Mul(normalY, t0Z));
				Vec4V t1Y = V4NegMulSub(normalX, t0Z, V4Mul(normalZ, t0X));
				Vec4V t1Z = V4NegMulSub(normalY, t0X, V4Mul(normalX, t0Y));

				PX_ASSERT((uintptr_t(descs[0].frictionPtr) & 0xF) == 0);
				PX_ASSERT((uintptr_t(descs[1].frictionPtr) & 0xF) == 0);
				PX_ASSERT((uintptr_t(descs[2].frictionPtr) & 0xF) == 0);
				PX_ASSERT((uintptr_t(descs[3].frictionPtr) & 0xF) == 0);

				PxU8* PX_RESTRICT writeback0 = descs[0].frictionPtr + frictionPatchWritebackAddrIndex0 * sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback1 = descs[1].frictionPtr + frictionPatchWritebackAddrIndex1 * sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback2 = descs[2].frictionPtr + frictionPatchWritebackAddrIndex2 * sizeof(FrictionPatch);
				PxU8* PX_RESTRICT writeback3 = descs[3].frictionPtr + frictionPatchWritebackAddrIndex3 * sizeof(FrictionPatch);

				PxU32 index0 = 0, index1 = 0, index2 = 0, index3 = 0;

				header->broken = bFalse;
				header->frictionBrokenWritebackByte[0] = writeback0;
				header->frictionBrokenWritebackByte[1] = writeback1;
				header->frictionBrokenWritebackByte[2] = writeback2;
				header->frictionBrokenWritebackByte[3] = writeback3;

				/*header->frictionNormal[0][0] = t0X;
				header->frictionNormal[0][1] = t0Y;
				header->frictionNormal[0][2] = t0Z;

				header->frictionNormal[1][0] = t1X;
				header->frictionNormal[1][1] = t1Y;
				header->frictionNormal[1][2] = t1Z;*/

				Vec4V* PX_RESTRICT appliedForces = reinterpret_cast<Vec4V*>(ptr);
				ptr += sizeof(Vec4V)*header->numFrictionConstr;

				PxMemZero(appliedForces, sizeof(Vec4V) * header->numFrictionConstr);

				for (PxU32 j = 0; j < maxAnchorCount; j++)
				{
					PxPrefetchLine(ptr, 384);
					PxPrefetchLine(ptr, 512);
					PxPrefetchLine(ptr, 640);
					SolverContactFrictionStepBlock* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionStepBlock*>(ptr);
					ptr += frictionSize;
					SolverContactFrictionStepBlock* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionStepBlock*>(ptr);
					ptr += frictionSize;

					index0 = j < clampedAnchorCount0 ? j : index0;
					index1 = j < clampedAnchorCount1 ? j : index1;
					index2 = j < clampedAnchorCount2 ? j : index2;
					index3 = j < clampedAnchorCount3 ? j : index3;

					if (j >= clampedAnchorCount0)
						maxImpulseScale = V4SetX(maxImpulseScale, fZero);
					if (j >= clampedAnchorCount1)
						maxImpulseScale = V4SetY(maxImpulseScale, fZero);
					if (j >= clampedAnchorCount2)
						maxImpulseScale = V4SetZ(maxImpulseScale, fZero);
					if (j >= clampedAnchorCount3)
						maxImpulseScale = V4SetW(maxImpulseScale, fZero);

					t0X = V4Mul(maxImpulseScale, t0X);
					t0Y = V4Mul(maxImpulseScale, t0Y);
					t0Z = V4Mul(maxImpulseScale, t0Z);

					t1X = V4Mul(maxImpulseScale, t1X);
					t1Y = V4Mul(maxImpulseScale, t1Y);
					t1Z = V4Mul(maxImpulseScale, t1Z);

					Vec3V body0Anchor0 = V3LoadU(frictionPatch0.body0Anchors[index0]);
					Vec3V body0Anchor1 = V3LoadU(frictionPatch1.body0Anchors[index1]);
					Vec3V body0Anchor2 = V3LoadU(frictionPatch2.body0Anchors[index2]);
					Vec3V body0Anchor3 = V3LoadU(frictionPatch3.body0Anchors[index3]);

					Vec4V ra0 = Vec4V_From_Vec3V(QuatRotate(bodyFrame00q, body0Anchor0));
					Vec4V ra1 = Vec4V_From_Vec3V(QuatRotate(bodyFrame01q, body0Anchor1));
					Vec4V ra2 = Vec4V_From_Vec3V(QuatRotate(bodyFrame02q, body0Anchor2));
					Vec4V ra3 = Vec4V_From_Vec3V(QuatRotate(bodyFrame03q, body0Anchor3));

					Vec4V raX, raY, raZ;
					PX_TRANSPOSE_44_34(ra0, ra1, ra2, ra3, raX, raY, raZ);

					/*raX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raX)), zero, raX);
					raY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raY)), zero, raY);
					raZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raZ)), zero, raZ);*/

					const Vec4V raWorldX = V4Add(raX, bodyFrame0pX);
					const Vec4V raWorldY = V4Add(raY, bodyFrame0pY);
					const Vec4V raWorldZ = V4Add(raZ, bodyFrame0pZ);

					Vec3V body1Anchor0 = V3LoadU(frictionPatch0.body1Anchors[index0]);
					Vec3V body1Anchor1 = V3LoadU(frictionPatch1.body1Anchors[index1]);
					Vec3V body1Anchor2 = V3LoadU(frictionPatch2.body1Anchors[index2]);
					Vec3V body1Anchor3 = V3LoadU(frictionPatch3.body1Anchors[index3]);

					Vec4V rb0 = Vec4V_From_Vec3V(QuatRotate(bodyFrame10q, body1Anchor0));
					Vec4V rb1 = Vec4V_From_Vec3V(QuatRotate(bodyFrame11q, body1Anchor1));
					Vec4V rb2 = Vec4V_From_Vec3V(QuatRotate(bodyFrame12q, body1Anchor2));
					Vec4V rb3 = Vec4V_From_Vec3V(QuatRotate(bodyFrame13q, body1Anchor3));

					Vec4V rbX, rbY, rbZ;
					PX_TRANSPOSE_44_34(rb0, rb1, rb2, rb3, rbX, rbY, rbZ);

					/*rbX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbX)), zero, rbX);
					rbY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbY)), zero, rbY);
					rbZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbZ)), zero, rbZ);*/

					const Vec4V rbWorldX = V4Add(rbX, bodyFrame1pX);
					const Vec4V rbWorldY = V4Add(rbY, bodyFrame1pY);
					const Vec4V rbWorldZ = V4Add(rbZ, bodyFrame1pZ);

					Vec4V errorX = V4Sub(raWorldX, rbWorldX);
					Vec4V errorY = V4Sub(raWorldY, rbWorldY);
					Vec4V errorZ = V4Sub(raWorldZ, rbWorldZ);

					/*errorX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(errorX)), zero, errorX);
					errorY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(errorY)), zero, errorY);
					errorZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(errorZ)), zero, errorZ);*/

					//KS - todo - get this working with per-point friction
					PxU32 contactIndex0 = c.contactID[frictionIndex0][index0];
					PxU32 contactIndex1 = c.contactID[frictionIndex1][index1];
					PxU32 contactIndex2 = c.contactID[frictionIndex2][index2];
					PxU32 contactIndex3 = c.contactID[frictionIndex3][index3];

					//Ensure that the contact indices are valid
					PX_ASSERT(contactIndex0 == 0xffff || contactIndex0 < descs[0].numContacts);
					PX_ASSERT(contactIndex1 == 0xffff || contactIndex1 < descs[1].numContacts);
					PX_ASSERT(contactIndex2 == 0xffff || contactIndex2 < descs[2].numContacts);
					PX_ASSERT(contactIndex3 == 0xffff || contactIndex3 < descs[3].numContacts);

					Vec4V targetVel0 = V4LoadA(contactIndex0 == 0xFFFF ? &contactBase0->targetVel.x : &descs[0].contacts[contactIndex0].targetVel.x);
					Vec4V targetVel1 = V4LoadA(contactIndex1 == 0xFFFF ? &contactBase0->targetVel.x : &descs[1].contacts[contactIndex1].targetVel.x);
					Vec4V targetVel2 = V4LoadA(contactIndex2 == 0xFFFF ? &contactBase0->targetVel.x : &descs[2].contacts[contactIndex2].targetVel.x);
					Vec4V targetVel3 = V4LoadA(contactIndex3 == 0xFFFF ? &contactBase0->targetVel.x : &descs[3].contacts[contactIndex3].targetVel.x);

					Vec4V targetVelX, targetVelY, targetVelZ;
					PX_TRANSPOSE_44_34(targetVel0, targetVel1, targetVel2, targetVel3, targetVelX, targetVelY, targetVelZ);

					{
						Vec4V raXnX = V4NegMulSub(raZ, t0Y, V4Mul(raY, t0Z));
						Vec4V raXnY = V4NegMulSub(raX, t0Z, V4Mul(raZ, t0X));
						Vec4V raXnZ = V4NegMulSub(raY, t0X, V4Mul(raX, t0Y));

						raXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnX)), zero, raXnX);
						raXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnY)), zero, raXnY);
						raXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnZ)), zero, raXnZ);

						Vec4V delAngVel0X = V4Mul(invInertia0X0, raXnX);
						Vec4V delAngVel0Y = V4Mul(invInertia0X1, raXnX);
						Vec4V delAngVel0Z = V4Mul(invInertia0X2, raXnX);

						delAngVel0X = V4MulAdd(invInertia0Y0, raXnY, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Y1, raXnY, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Y2, raXnY, delAngVel0Z);

						delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, delAngVel0Z);

						const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0Z, delAngVel0Z, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0X, delAngVel0X)));

						Vec4V resp = V4MulAdd(dotDelAngVel0, angDom0, invMass0D0);

						const Vec4V tVel0 = V4MulAdd(t0Z, linVelT20, V4MulAdd(t0Y, linVelT10, V4Mul(t0X, linVelT00)));
						Vec4V vrel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4MulAdd(raXnX, angVelT00, tVel0)));

						Vec4V delAngVel1X = zero;
						Vec4V delAngVel1Y = zero;
						Vec4V delAngVel1Z = zero;

						Vec4V vrel1 = zero;

						if (isDynamic)
						{
							Vec4V rbXnX = V4NegMulSub(rbZ, t0Y, V4Mul(rbY, t0Z));
							Vec4V rbXnY = V4NegMulSub(rbX, t0Z, V4Mul(rbZ, t0X));
							Vec4V rbXnZ = V4NegMulSub(rbY, t0X, V4Mul(rbX, t0Y));

							rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
							rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
							rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

							delAngVel1X = V4Mul(invInertia1X0, rbXnX);
							delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
							delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

							delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

							delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);

							const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));

							const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

							resp = V4Add(resp, resp1);

							const Vec4V tVel1 = V4MulAdd(t0Z, linVelT21, V4MulAdd(t0Y, linVelT11, V4Mul(t0X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}
						else if (hasKinematic)
						{
							const Vec4V rbXnX = V4NegMulSub(rbZ, t0Y, V4Mul(rbY, t0Z));
							const Vec4V rbXnY = V4NegMulSub(rbX, t0Z, V4Mul(rbZ, t0X));
							const Vec4V rbXnZ = V4NegMulSub(rbY, t0X, V4Mul(rbX, t0Y));

							const Vec4V tVel1 = V4MulAdd(t0Z, linVelT21, V4MulAdd(t0Y, linVelT11, V4Mul(t0X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}

						f0->rbXnI[0] = delAngVel1X;
						f0->rbXnI[1] = delAngVel1Y;
						f0->rbXnI[2] = delAngVel1Z;

						const Vec4V velMultiplier = V4Mul(maxImpulseScale, V4Sel(V4IsGrtr(resp, zero), V4Div(p84, resp), zero));

						Vec4V error = V4MulAdd(t0Z, errorZ, V4MulAdd(t0Y, errorY, V4Mul(t0X, errorX)));

						Vec4V targetVel = V4NegMulSub(vrel0, kinematicScale0, V4MulAdd(vrel1, kinematicScale1, V4MulAdd(t0Z, targetVelZ, V4MulAdd(t0Y, targetVelY, V4Mul(t0X, targetVelX)))));
						
						f0->normal[0] = t0X;
						f0->normal[1] = t0Y;
						f0->normal[2] = t0Z;
						f0->raXnI[0] = delAngVel0X;
						f0->raXnI[1] = delAngVel0Y;
						f0->raXnI[2] = delAngVel0Z;
						f0->error = error;
						f0->velMultiplier = velMultiplier;
						f0->biasCoefficient = V4Splat(frictionBiasScale);
						f0->targetVel = targetVel;
					}

					{
						Vec4V raXnX = V4NegMulSub(raZ, t1Y, V4Mul(raY, t1Z));
						Vec4V raXnY = V4NegMulSub(raX, t1Z, V4Mul(raZ, t1X));
						Vec4V raXnZ = V4NegMulSub(raY, t1X, V4Mul(raX, t1Y));
						raXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnX)), zero, raXnX);
						raXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnY)), zero, raXnY);
						raXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(raXnZ)), zero, raXnZ);

						Vec4V delAngVel0X = V4Mul(invInertia0X0, raXnX);
						Vec4V delAngVel0Y = V4Mul(invInertia0X1, raXnX);
						Vec4V delAngVel0Z = V4Mul(invInertia0X2, raXnX);

						delAngVel0X = V4MulAdd(invInertia0Y0, raXnY, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Y1, raXnY, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Y2, raXnY, delAngVel0Z);

						delAngVel0X = V4MulAdd(invInertia0Z0, raXnZ, delAngVel0X);
						delAngVel0Y = V4MulAdd(invInertia0Z1, raXnZ, delAngVel0Y);
						delAngVel0Z = V4MulAdd(invInertia0Z2, raXnZ, delAngVel0Z);

						const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0Z, delAngVel0Z, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0X, delAngVel0X)));

						Vec4V resp = V4MulAdd(dotDelAngVel0, angDom0, invMass0D0);

						const Vec4V tVel0 = V4MulAdd(t1Z, linVelT20, V4MulAdd(t1Y, linVelT10, V4Mul(t1X, linVelT00)));
						Vec4V vrel0 = V4MulAdd(raXnZ, angVelT20, V4MulAdd(raXnY, angVelT10, V4MulAdd(raXnX, angVelT00, tVel0)));

						Vec4V delAngVel1X = zero;
						Vec4V delAngVel1Y = zero;
						Vec4V delAngVel1Z = zero;

						Vec4V vrel1 = zero;

						if (isDynamic)
						{
							Vec4V rbXnX = V4NegMulSub(rbZ, t1Y, V4Mul(rbY, t1Z));
							Vec4V rbXnY = V4NegMulSub(rbX, t1Z, V4Mul(rbZ, t1X));
							Vec4V rbXnZ = V4NegMulSub(rbY, t1X, V4Mul(rbX, t1Y));
							rbXnX = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnX)), zero, rbXnX);
							rbXnY = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnY)), zero, rbXnY);
							rbXnZ = V4Sel(V4IsGrtr(solverOffsetSlop, V4Abs(rbXnZ)), zero, rbXnZ);

							delAngVel1X = V4Mul(invInertia1X0, rbXnX);
							delAngVel1Y = V4Mul(invInertia1X1, rbXnX);
							delAngVel1Z = V4Mul(invInertia1X2, rbXnX);

							delAngVel1X = V4MulAdd(invInertia1Y0, rbXnY, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Y1, rbXnY, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Y2, rbXnY, delAngVel1Z);

							delAngVel1X = V4MulAdd(invInertia1Z0, rbXnZ, delAngVel1X);
							delAngVel1Y = V4MulAdd(invInertia1Z1, rbXnZ, delAngVel1Y);
							delAngVel1Z = V4MulAdd(invInertia1Z2, rbXnZ, delAngVel1Z);

							const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1Z, delAngVel1Z, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1X, delAngVel1X)));

							const Vec4V resp1 = V4MulAdd(dotDelAngVel1, angDom1, invMass1D1);

							resp = V4Add(resp, resp1);

							const Vec4V tVel1 = V4MulAdd(t1Z, linVelT21, V4MulAdd(t1Y, linVelT11, V4Mul(t1X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}
						else if (hasKinematic)
						{
							const Vec4V rbXnX = V4NegMulSub(rbZ, t1Y, V4Mul(rbY, t1Z));
							const Vec4V rbXnY = V4NegMulSub(rbX, t1Z, V4Mul(rbZ, t1X));
							const Vec4V rbXnZ = V4NegMulSub(rbY, t1X, V4Mul(rbX, t1Y));

							const Vec4V tVel1 = V4MulAdd(t1Z, linVelT21, V4MulAdd(t1Y, linVelT11, V4Mul(t1X, linVelT01)));
							vrel1 = V4MulAdd(rbXnZ, angVelT21, V4MulAdd(rbXnY, angVelT11, V4MulAdd(rbXnX, angVelT01, tVel1)));
						}

						f1->rbXnI[0] = delAngVel1X;
						f1->rbXnI[1] = delAngVel1Y;
						f1->rbXnI[2] = delAngVel1Z;

						const Vec4V velMultiplier = V4Mul(maxImpulseScale, V4Sel(V4IsGrtr(resp, zero), V4Div(p84, resp), zero));

						Vec4V error = V4MulAdd(t1Z, errorZ, V4MulAdd(t1Y, errorY, V4Mul(t1X, errorX)));

						Vec4V targetVel = V4NegMulSub(vrel0, kinematicScale0, V4MulAdd(vrel1, kinematicScale1, V4MulAdd(t1Z, targetVelZ, V4MulAdd(t1Y, targetVelY, V4Mul(t1X, targetVelX)))));
						
						f1->normal[0] = t1X;
						f1->normal[1] = t1Y;
						f1->normal[2] = t1Z;
						f1->raXnI[0] = delAngVel0X;
						f1->raXnI[1] = delAngVel0Y;
						f1->raXnI[2] = delAngVel0Z;
						f1->error = error;
						f1->velMultiplier = velMultiplier;
						f1->targetVel = targetVel;
						f1->biasCoefficient = V4Splat(frictionBiasScale);
					}
				}

				header->dynamicFriction = V4LoadA(dynamicFriction);
				header->staticFriction = V4LoadA(staticFriction);

				frictionPatchWritebackAddrIndex0++;
				frictionPatchWritebackAddrIndex1++;
				frictionPatchWritebackAddrIndex2++;
				frictionPatchWritebackAddrIndex3++;
			}
		}
	}
}

static PX_FORCE_INLINE void computeBlockStreamFrictionByteSizes(const CorrelationBuffer& c,
	PxU32& _frictionPatchByteSize, PxU32& _numFrictionPatches,
	PxU32 frictionPatchStartIndex, PxU32 frictionPatchEndIndex)
{
	// PT: use local vars to remove LHS
	PxU32 numFrictionPatches = 0;

	for (PxU32 i = frictionPatchStartIndex; i < frictionPatchEndIndex; i++)
	{
		//Friction patches.
		if (c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
			numFrictionPatches++;
	}
	PxU32 frictionPatchByteSize = numFrictionPatches * sizeof(FrictionPatch);

	_numFrictionPatches = numFrictionPatches;

	//16-byte alignment.
	_frictionPatchByteSize = ((frictionPatchByteSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_frictionPatchByteSize & 0x0f));
}

static bool reserveFrictionBlockStreams(const CorrelationBuffer& c, PxConstraintAllocator& constraintAllocator, PxU32 frictionPatchStartIndex, PxU32 frictionPatchEndIndex,
	FrictionPatch*& _frictionPatches,
	PxU32& numFrictionPatches)
{
	//From frictionPatchStream we just need to reserve a single buffer.
	PxU32 frictionPatchByteSize = 0;
	//Compute the sizes of all the buffers.

	computeBlockStreamFrictionByteSizes(c, frictionPatchByteSize, numFrictionPatches, frictionPatchStartIndex, frictionPatchEndIndex);

	FrictionPatch* frictionPatches = NULL;
	//If the constraint block reservation didn't fail then reserve the friction buffer too.
	if (frictionPatchByteSize > 0)
	{
		frictionPatches = reinterpret_cast<FrictionPatch*>(constraintAllocator.reserveFrictionData(frictionPatchByteSize));
		if(!checkFrictionDataPtr(frictionPatches))
			frictionPatches = NULL;
	}

	_frictionPatches = frictionPatches;

	//Return true if neither of the two block reservations failed.
	return (0 == frictionPatchByteSize || frictionPatches);
}

//The persistent friction patch correlation/allocation will already have happenned as this is per-pair.
//This function just computes the size of the combined solve data.
static void computeBlockStreamByteSizes4(PxTGSSolverContactDesc* descs,
	PxU32& _solverConstraintByteSize, PxU32* _axisConstraintCount,
	const CorrelationBuffer& c)
{
	PX_ASSERT(0 == _solverConstraintByteSize);

	PxU32 maxPatches = 0;
	PxU32 maxContactCount[CorrelationBuffer::MAX_FRICTION_PATCHES];
	PxU32 maxFrictionCount[CorrelationBuffer::MAX_FRICTION_PATCHES];
	PxMemZero(maxContactCount, sizeof(maxContactCount));
	PxMemZero(maxFrictionCount, sizeof(maxFrictionCount));
	bool hasMaxImpulse = false;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxU32 axisConstraintCount = 0;
		hasMaxImpulse = hasMaxImpulse || descs[a].hasMaxImpulse;
		for (PxU32 i = 0; i < descs[a].numFrictionPatches; i++)
		{
			PxU32 ind = i + descs[a].startFrictionPatchIndex;

			const FrictionPatch& frictionPatch = c.frictionPatches[ind];

			const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0
				&& frictionPatch.anchorCount != 0;
			//Solver constraint data.
			if (c.frictionPatchContactCounts[ind] != 0)
			{
				maxContactCount[i] = PxMax(c.frictionPatchContactCounts[ind], maxContactCount[i]);
				axisConstraintCount += c.frictionPatchContactCounts[ind];

				if (haveFriction)
				{
					const PxU32 fricCount = PxU32(c.frictionPatches[ind].anchorCount) * 2;
					maxFrictionCount[i] = PxMax(fricCount, maxFrictionCount[i]);
					axisConstraintCount += fricCount;
				}
			}
		}
		maxPatches = PxMax(descs[a].numFrictionPatches, maxPatches);
		_axisConstraintCount[a] = axisConstraintCount;
	}

	PxU32 totalContacts = 0, totalFriction = 0;
	for (PxU32 a = 0; a < maxPatches; ++a)
	{
		totalContacts += maxContactCount[a];
		totalFriction += maxFrictionCount[a];
	}

	//OK, we have a given number of friction patches, contact points and friction constraints so we can calculate how much memory we need

	//Body 2 is considered static if it is either *not dynamic* or *kinematic*

	/*bool hasDynamicBody = false;
	for (PxU32 a = 0; a < 4; ++a)
	{
		hasDynamicBody = hasDynamicBody || ((descs[a].bodyState1 == PxSolverContactDesc::eDYNAMIC_BODY));
	}

	const bool isStatic = !hasDynamicBody;*/

	const PxU32 headerSize = sizeof(SolverContactHeaderStepBlock) * maxPatches;
	//PxU32 constraintSize = isStatic ? (sizeof(SolverContactBatchPointBase4) * totalContacts) + (sizeof(SolverContactFrictionBase4) * totalFriction) :
	//	(sizeof(SolverContactBatchPointDynamic4) * totalContacts) + (sizeof(SolverContactFrictionDynamic4) * totalFriction);

	PxU32 constraintSize = (sizeof(SolverContactPointStepBlock) * totalContacts) + (sizeof(SolverContactFrictionStepBlock) * totalFriction);

	//Space for the appliedForce buffer
	constraintSize += sizeof(Vec4V)*(totalContacts + totalFriction);

	//If we have max impulse, reserve a buffer for it
	if (hasMaxImpulse)
		constraintSize += sizeof(aos::Vec4V) * totalContacts;

	_solverConstraintByteSize = ((constraintSize + headerSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
}

static SolverConstraintPrepState::Enum reserveBlockStreams4(PxTGSSolverContactDesc* descs, Dy::CorrelationBuffer& c,
	PxU8*& solverConstraint, PxU32* axisConstraintCount,
	PxU32& solverConstraintByteSize,
	PxConstraintAllocator& constraintAllocator)
{
	PX_ASSERT(NULL == solverConstraint);
	PX_ASSERT(0 == solverConstraintByteSize);

	//Compute the sizes of all the buffers.
	computeBlockStreamByteSizes4(descs,
		solverConstraintByteSize, axisConstraintCount,
		c);

	//Reserve the buffers.

	//First reserve the accumulated buffer size for the constraint block.
	PxU8* constraintBlock = NULL;
	const PxU32 constraintBlockByteSize = solverConstraintByteSize;
	if (constraintBlockByteSize > 0)
	{
		if ((constraintBlockByteSize + 16u) > 16384)
			return SolverConstraintPrepState::eUNBATCHABLE;

		constraintBlock = constraintAllocator.reserveConstraintData(constraintBlockByteSize + 16u);
		if(!checkConstraintDataPtr<false>(constraintBlock))
			constraintBlock = NULL;
	}

	//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
	if (0 == constraintBlockByteSize || constraintBlock)
	{
		if (solverConstraintByteSize)
		{
			solverConstraint = constraintBlock;
			PX_ASSERT(0 == (uintptr_t(solverConstraint) & 0x0f));
		}
	}

	return ((0 == constraintBlockByteSize || constraintBlock)) ? SolverConstraintPrepState::eSUCCESS : SolverConstraintPrepState::eOUT_OF_MEMORY;
}

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
	Dy::CorrelationBuffer& c,
	PxTGSSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	const PxReal totalDtF32,
	const PxReal invTotalDtF32,
	const PxReal dt,
	const PxReal bounceThresholdF32,
	const PxReal	frictionOffsetThreshold,
	const PxReal correlationDistance,
	const PxReal biasCoefficient,
	PxConstraintAllocator& constraintAllocator)
{
	PX_ALIGN(16, PxReal invMassScale0[4]);
	PX_ALIGN(16, PxReal invMassScale1[4]);
	PX_ALIGN(16, PxReal invInertiaScale0[4]);
	PX_ALIGN(16, PxReal invInertiaScale1[4]);

	c.frictionPatchCount = 0;
	c.contactPatchCount = 0;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxTGSSolverContactDesc& blockDesc = blockDescs[a];

		invMassScale0[a] = blockDesc.invMassScales.linear0;
		invMassScale1[a] = blockDesc.invMassScales.linear1;
		invInertiaScale0[a] = blockDesc.invMassScales.angular0;
		invInertiaScale1[a] = blockDesc.invMassScales.angular1;

		blockDesc.startFrictionPatchIndex = c.frictionPatchCount;
		if (!(blockDesc.disableStrongFriction))
		{
			const bool valid = getFrictionPatches(c, blockDesc.frictionPtr, blockDesc.frictionCount,
				blockDesc.bodyFrame0, blockDesc.bodyFrame1, correlationDistance);
			if (!valid)
				return SolverConstraintPrepState::eUNBATCHABLE;
		}
		//Create the contact patches
		blockDesc.startContactPatchIndex = c.contactPatchCount;
		if (!createContactPatches(c, blockDesc.contacts, blockDesc.numContacts, PXC_SAME_NORMAL))
			return SolverConstraintPrepState::eUNBATCHABLE;
		blockDesc.numContactPatches = PxU16(c.contactPatchCount - blockDesc.startContactPatchIndex);

		const bool overflow = correlatePatches(c, blockDesc.contacts, blockDesc.bodyFrame0, blockDesc.bodyFrame1, PXC_SAME_NORMAL,
			blockDesc.startContactPatchIndex, blockDesc.startFrictionPatchIndex);

		if (overflow)
			return SolverConstraintPrepState::eUNBATCHABLE;

		growPatches(c, blockDesc.contacts, blockDesc.bodyFrame0, blockDesc.bodyFrame1, blockDesc.startFrictionPatchIndex,
			frictionOffsetThreshold + blockDescs[a].restDistance);

		//Remove the empty friction patches - do we actually need to do this?
		for (PxU32 p = c.frictionPatchCount; p > blockDesc.startFrictionPatchIndex; --p)
		{
			if (c.correlationListHeads[p - 1] == 0xffff)
			{
				//We have an empty patch...need to bin this one...
				for (PxU32 p2 = p; p2 < c.frictionPatchCount; ++p2)
				{
					c.correlationListHeads[p2 - 1] = c.correlationListHeads[p2];
					c.frictionPatchContactCounts[p2 - 1] = c.frictionPatchContactCounts[p2];
				}
				c.frictionPatchCount--;
			}
		}

		PxU32 numFricPatches = c.frictionPatchCount - blockDesc.startFrictionPatchIndex;
		blockDesc.numFrictionPatches = numFricPatches;
	}

	FrictionPatch* frictionPatchArray[4];
	PxU32 frictionPatchCounts[4];

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxTGSSolverContactDesc& blockDesc = blockDescs[a];

		const bool successfulReserve = reserveFrictionBlockStreams(c, constraintAllocator, blockDesc.startFrictionPatchIndex, blockDesc.numFrictionPatches + blockDesc.startFrictionPatchIndex,
			frictionPatchArray[a],
			frictionPatchCounts[a]);

		//KS - TODO - how can we recover if we failed to allocate this memory?
		if (!successfulReserve)
		{
			return SolverConstraintPrepState::eOUT_OF_MEMORY;
		}
	}
	//At this point, all the friction data has been calculated, the correlation has been done. Provided this was all successful, 
	//we are ready to create the batched constraints

	PxU8* solverConstraint = NULL;
	PxU32 solverConstraintByteSize = 0;

	{
		PxU32 axisConstraintCount[4];
		SolverConstraintPrepState::Enum state = reserveBlockStreams4(blockDescs, c,
			solverConstraint, axisConstraintCount,
			solverConstraintByteSize,
			constraintAllocator);

		if (state != SolverConstraintPrepState::eSUCCESS)
			return state;

		for (PxU32 a = 0; a < 4; ++a)
		{
			FrictionPatch* frictionPatches = frictionPatchArray[a];

			PxTGSSolverContactDesc& blockDesc = blockDescs[a];
			PxSolverConstraintDesc& desc = *blockDesc.desc;
			blockDesc.frictionPtr = reinterpret_cast<PxU8*>(frictionPatches);
			blockDesc.frictionCount = PxTo8(frictionPatchCounts[a]);

			//Initialise friction buffer.
			if (frictionPatches)
			{
				frictionPatches->prefetch();

				for (PxU32 i = 0; i<blockDesc.numFrictionPatches; i++)
				{
					if (c.correlationListHeads[blockDesc.startFrictionPatchIndex + i] != CorrelationBuffer::LIST_END)
					{
						//*frictionPatches++ = c.frictionPatches[blockDesc.startFrictionPatchIndex + i];
						PxMemCopy(frictionPatches++, &c.frictionPatches[blockDesc.startFrictionPatchIndex + i], sizeof(FrictionPatch));
						//PxPrefetchLine(frictionPatches, 256);
					}
				}
			}

			blockDesc.axisConstraintCount += PxTo16(axisConstraintCount[a]);

			desc.constraint = solverConstraint;
			desc.constraintLengthOver16 = PxTo16(solverConstraintByteSize / 16);
			desc.writeBack = blockDesc.contactForces;
		}

		const Vec4V iMassScale0 = V4LoadA(invMassScale0);
		const Vec4V iInertiaScale0 = V4LoadA(invInertiaScale0);
		const Vec4V iMassScale1 = V4LoadA(invMassScale1);
		const Vec4V iInertiaScale1 = V4LoadA(invInertiaScale1);

		setupFinalizeSolverConstraints4Step(blockDescs, c, solverConstraint, invDtF32, totalDtF32, invTotalDtF32, dt, bounceThresholdF32,
			biasCoefficient, iMassScale0, iInertiaScale0, iMassScale1, iInertiaScale1);

		PX_ASSERT((*solverConstraint == DY_SC_TYPE_BLOCK_RB_CONTACT) || (*solverConstraint == DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT));

		*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
	}
	return SolverConstraintPrepState::eSUCCESS;
}

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
	PxsContactManagerOutput** cmOutputs,
	ThreadContext& threadContext,
	PxTGSSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	const PxReal totalDtF32,
	const PxReal invTotalDtF32,
	const PxReal dtF32,
	const PxReal bounceThresholdF32,
	const PxReal frictionOffsetThreshold,
	const PxReal correlationDistance,
	const PxReal biasCoefficient,
	PxConstraintAllocator& constraintAllocator)
{
	for (PxU32 a = 0; a < 4; ++a)
	{
		blockDescs[a].desc->constraintLengthOver16 = 0;
	}

	//PX_ASSERT(cmOutputs[0]->nbContacts && cmOutputs[1]->nbContacts && cmOutputs[2]->nbContacts && cmOutputs[3]->nbContacts);

	PxContactBuffer& buffer = threadContext.mContactBuffer;

	buffer.count = 0;

	//PxTransform idt = PxTransform(PxIdentity);

	CorrelationBuffer& c = threadContext.mCorrelationBuffer;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxTGSSolverContactDesc& blockDesc = blockDescs[a];
		PxSolverConstraintDesc& desc = *blockDesc.desc;

		//blockDesc.startContactIndex = buffer.count;
		blockDesc.contacts = buffer.contacts + buffer.count;

		PxPrefetchLine(desc.bodyA);
		PxPrefetchLine(desc.bodyB);

		//Unbatchable if we have (a) too many contacts or (b) torsional friction enabled - it just seems easier to handle this on an individual contact basis because it is expected to 
		//be used relatively rarely
		if ((buffer.count + cmOutputs[a]->nbContacts) > 64 || (blockDesc.torsionalPatchRadius != 0.f || blockDesc.minTorsionalPatchRadius != 0.f) )
		{
			return SolverConstraintPrepState::eUNBATCHABLE;
		}

		bool hasMaxImpulse = false;
		bool hasTargetVelocity = false;

		//OK...do the correlation here as well...
		PxPrefetchLine(blockDescs[a].frictionPtr);
		PxPrefetchLine(blockDescs[a].frictionPtr, 64);
		PxPrefetchLine(blockDescs[a].frictionPtr, 128);

		if (a < 3)
		{
			PxPrefetchLine(cmOutputs[a]->contactPatches);
			PxPrefetchLine(cmOutputs[a]->contactPoints);
		}

		PxReal invMassScale0, invMassScale1, invInertiaScale0, invInertiaScale1;

		const PxReal defaultMaxImpulse = PxMin(blockDesc.bodyData0->maxContactImpulse, blockDesc.bodyData1->maxContactImpulse);

		PxU32 contactCount = extractContacts(buffer, *cmOutputs[a], hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
			invInertiaScale0, invInertiaScale1, defaultMaxImpulse);

		if (contactCount == 0 || hasTargetVelocity)
			return SolverConstraintPrepState::eUNBATCHABLE;

		blockDesc.numContacts = contactCount;
		blockDesc.hasMaxImpulse = hasMaxImpulse;
		blockDesc.disableStrongFriction = blockDesc.disableStrongFriction || hasTargetVelocity;

		blockDesc.invMassScales.linear0 *= invMassScale0;
		blockDesc.invMassScales.linear1 *= invMassScale1;
		blockDesc.invMassScales.angular0 *= blockDesc.body0->isKinematic ? 0.f : invInertiaScale0;
		blockDesc.invMassScales.angular1 *= blockDesc.body1->isKinematic ? 0.f : invInertiaScale1;
	}

	return createFinalizeSolverContacts4Step(c, blockDescs,
		invDtF32, totalDtF32, invTotalDtF32, dtF32, bounceThresholdF32, frictionOffsetThreshold,
		correlationDistance, biasCoefficient, constraintAllocator);
}

static void setOrthoData(const PxReal& ang0X, const PxReal& ang0Y, const PxReal& ang0Z, const PxReal& ang1X, const PxReal& ang1Y, const PxReal& ang1Z,
	const PxReal& recipResponse, const PxReal& error, PxReal& orthoAng0X, PxReal& orthoAng0Y, PxReal& orthoAng0Z, PxReal& orthoAng1X, PxReal& orthoAng1Y, PxReal& orthoAng1Z,
	PxReal& orthoRecipResponse, PxReal& orthoError, bool disableProcessing, PxU32 solveHint, PxU32& flags, PxU32& orthoCount, bool finished)
{
	if (!finished && !disableProcessing)
	{
		if (solveHint == PxConstraintSolveHint::eROTATIONAL_EQUALITY)
		{
			flags |= DY_SC_FLAG_ROT_EQ;
			orthoAng0X = ang0X; orthoAng0Y = ang0Y; orthoAng0Z = ang0Z;
			orthoAng1X = ang1X; orthoAng1Y = ang1Y; orthoAng1Z = ang1Z;
			orthoRecipResponse = recipResponse;
			orthoError = error;
			orthoCount++;
		}
		else if (solveHint & PxConstraintSolveHint::eEQUALITY)
			flags |= DY_SC_FLAG_ORTHO_TARGET;
	}
}

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, PxU32 maxRows, const PxReal lengthScale, const PxReal biasCoefficient, bool isResidualReportingEnabled);

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
	PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, const PxReal lengthScale, const PxReal biasCoefficient, bool isResidualReportingEnabled)
{
	//KS - we will never get here with constraints involving articulations so we don't need to stress about those in here

	totalRows = 0;

	Px1DConstraint allRows[MAX_CONSTRAINT_ROWS * 4];
	Px1DConstraint* rows = allRows;
	Px1DConstraint* rows2 = allRows;

	PxU32 maxRows = 0;
	PxU32 nbToPrep = MAX_CONSTRAINT_ROWS;

	for (PxU32 a = 0; a < 4; ++a)
	{
		SolverConstraintShaderPrepDesc& shaderDesc = constraintShaderDescs[a];
		PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];

		if (!shaderDesc.solverPrep)
			return SolverConstraintPrepState::eUNBATCHABLE;

		PX_ASSERT(rows2 + nbToPrep <= allRows + MAX_CONSTRAINT_ROWS*4);
		setupConstraintRows(rows2, nbToPrep);
		rows2 += nbToPrep;

		desc.invMassScales.linear0 = desc.invMassScales.linear1 = desc.invMassScales.angular0 = desc.invMassScales.angular1 = 1.0f;
		desc.body0WorldOffset = PxVec3(0.0f);

		//TAG:solverprepcall
		const PxU32 constraintCount = desc.disableConstraint ? 0 : (*shaderDesc.solverPrep)(rows,
			desc.body0WorldOffset,
			MAX_CONSTRAINT_ROWS,
			desc.invMassScales,
			shaderDesc.constantBlock,
			desc.bodyFrame0, desc.bodyFrame1, desc.extendedLimits, desc.cA2w, desc.cB2w);

		nbToPrep = constraintCount;
		maxRows = PxMax(constraintCount, maxRows);

		if (constraintCount == 0)
			return SolverConstraintPrepState::eUNBATCHABLE;

		desc.rows = rows;
		desc.numRows = constraintCount;
		rows += constraintCount;

		if (desc.body0->isKinematic)
			desc.invMassScales.angular0 = 0.0f;
		if (desc.body1->isKinematic)
			desc.invMassScales.angular1 = 0.0f;
	}

	return setupSolverConstraintStep4(constraintDescs, dt, totalDt, recipdt, recipTotalDt, totalRows, allocator, maxRows, lengthScale, biasCoefficient, isResidualReportingEnabled);
}

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal stepDt, const PxReal simDt, const PxReal recipStepDt, const PxReal recipSimDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, PxU32 maxRows,
	const PxReal lengthScale, const PxReal biasCoefficient, bool isResidualReportingEnabled)
{
	const Vec4V zero = V4Zero();
	Px1DConstraint* allSorted[MAX_CONSTRAINT_ROWS * 4];
	PxU32 startIndex[4];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia0[MAX_CONSTRAINT_ROWS * 4];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia1[MAX_CONSTRAINT_ROWS * 4];

	PxU32 numRows = 0;

	for (PxU32 a = 0; a < 4; ++a)
	{
		startIndex[a] = numRows;
		PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];
		Px1DConstraint** sorted = allSorted + numRows;

		for (PxU32 i = 0; i < desc.numRows; ++i)
		{
			if (desc.rows[i].flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT)
			{
				if (desc.rows[i].solveHint == PxConstraintSolveHint::eEQUALITY)
					desc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_EQUALITY;
				else if (desc.rows[i].solveHint == PxConstraintSolveHint::eINEQUALITY)
					desc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_INEQUALITY;
			}
		}

		preprocessRows(sorted, desc.rows, angSqrtInvInertia0 + numRows, angSqrtInvInertia1 + numRows, desc.numRows,
			desc.body0TxI->sqrtInvInertia, desc.body1TxI->sqrtInvInertia, desc.bodyData0->invMass, desc.bodyData1->invMass,
			desc.invMassScales, desc.disablePreprocessing, desc.improvedSlerp);

		numRows += desc.numRows;
	}

	const PxU32 stride = isResidualReportingEnabled ? sizeof(SolverConstraint1DStep4WithResidual) : sizeof(SolverConstraint1DStep4);

	const PxU32 constraintLength = sizeof(SolverConstraint1DHeaderStep4) + stride * maxRows;

	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if(!checkConstraintDataPtr<true>(ptr))
	{
		for (PxU32 a = 0; a < 4; ++a)
		{
			PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];
			desc.desc->constraint = NULL;
			desc.desc->constraintLengthOver16 = 0;
			desc.desc->writeBack = desc.writeback;
		}
		return SolverConstraintPrepState::eOUT_OF_MEMORY;
	}
	//desc.constraint = ptr;

	totalRows = numRows;

	const PxReal erp = 0.5f * biasCoefficient;

	const bool isKinematic00 = constraintDescs[0].body0->isKinematic;
	const bool isKinematic01 = constraintDescs[0].body1->isKinematic;
	const bool isKinematic10 = constraintDescs[1].body0->isKinematic;
	const bool isKinematic11 = constraintDescs[1].body1->isKinematic;
	const bool isKinematic20 = constraintDescs[2].body0->isKinematic;
	const bool isKinematic21 = constraintDescs[2].body1->isKinematic;
	const bool isKinematic30 = constraintDescs[3].body0->isKinematic;
	const bool isKinematic31 = constraintDescs[3].body1->isKinematic;

	for (PxU32 a = 0; a < 4; ++a)
	{
		PxTGSSolverConstraintPrepDesc& desc = constraintDescs[a];
		desc.desc->constraint = ptr;
		desc.desc->constraintLengthOver16 = PxU16(constraintLength/16);
		desc.desc->writeBack = desc.writeback;
	}

	{
		PxU8* currPtr = ptr;
		SolverConstraint1DHeaderStep4* header = reinterpret_cast<SolverConstraint1DHeaderStep4*>(currPtr);
		currPtr += sizeof(SolverConstraint1DHeaderStep4);

		const PxTGSSolverBodyData& bd00 = *constraintDescs[0].bodyData0;
		const PxTGSSolverBodyData& bd01 = *constraintDescs[1].bodyData0;
		const PxTGSSolverBodyData& bd02 = *constraintDescs[2].bodyData0;
		const PxTGSSolverBodyData& bd03 = *constraintDescs[3].bodyData0;

		const PxTGSSolverBodyData& bd10 = *constraintDescs[0].bodyData1;
		const PxTGSSolverBodyData& bd11 = *constraintDescs[1].bodyData1;
		const PxTGSSolverBodyData& bd12 = *constraintDescs[2].bodyData1;
		const PxTGSSolverBodyData& bd13 = *constraintDescs[3].bodyData1;

		//Load up masses, invInertia, velocity etc.

		const Vec4V invMassScale0 = V4LoadXYZW(constraintDescs[0].invMassScales.linear0, constraintDescs[1].invMassScales.linear0,
			constraintDescs[2].invMassScales.linear0, constraintDescs[3].invMassScales.linear0);
		const Vec4V invMassScale1 = V4LoadXYZW(constraintDescs[0].invMassScales.linear1, constraintDescs[1].invMassScales.linear1,
			constraintDescs[2].invMassScales.linear1, constraintDescs[3].invMassScales.linear1);

		const Vec4V iMass0 = V4LoadXYZW(bd00.invMass, bd01.invMass, bd02.invMass, bd03.invMass);

		const Vec4V iMass1 = V4LoadXYZW(bd10.invMass, bd11.invMass, bd12.invMass, bd13.invMass);

		const Vec4V invMass0 = V4Mul(iMass0, invMassScale0);
		const Vec4V invMass1 = V4Mul(iMass1, invMassScale1);

		const Vec4V invInertiaScale0 = V4LoadXYZW(constraintDescs[0].invMassScales.angular0, constraintDescs[1].invMassScales.angular0,
			constraintDescs[2].invMassScales.angular0, constraintDescs[3].invMassScales.angular0);
		const Vec4V invInertiaScale1 = V4LoadXYZW(constraintDescs[0].invMassScales.angular1, constraintDescs[1].invMassScales.angular1,
			constraintDescs[2].invMassScales.angular1, constraintDescs[3].invMassScales.angular1);

		//body world offsets
		Vec4V workOffset0 = V4LoadU(&constraintDescs[0].body0WorldOffset.x);
		Vec4V workOffset1 = V4LoadU(&constraintDescs[1].body0WorldOffset.x);
		Vec4V workOffset2 = V4LoadU(&constraintDescs[2].body0WorldOffset.x);
		Vec4V workOffset3 = V4LoadU(&constraintDescs[3].body0WorldOffset.x);

		Vec4V workOffsetX, workOffsetY, workOffsetZ;

		PX_TRANSPOSE_44_34(workOffset0, workOffset1, workOffset2, workOffset3, workOffsetX, workOffsetY, workOffsetZ);

		const FloatV dtV = FLoad(simDt);
		Vec4V linBreakForce = V4LoadXYZW(	constraintDescs[0].linBreakForce, constraintDescs[1].linBreakForce,
											constraintDescs[2].linBreakForce, constraintDescs[3].linBreakForce);
		Vec4V angBreakForce = V4LoadXYZW(	constraintDescs[0].angBreakForce, constraintDescs[1].angBreakForce,
											constraintDescs[2].angBreakForce, constraintDescs[3].angBreakForce);

		header->breakable[0] = PxU8((constraintDescs[0].linBreakForce != PX_MAX_F32) || (constraintDescs[0].angBreakForce != PX_MAX_F32));
		header->breakable[1] = PxU8((constraintDescs[1].linBreakForce != PX_MAX_F32) || (constraintDescs[1].angBreakForce != PX_MAX_F32));
		header->breakable[2] = PxU8((constraintDescs[2].linBreakForce != PX_MAX_F32) || (constraintDescs[2].angBreakForce != PX_MAX_F32));
		header->breakable[3] = PxU8((constraintDescs[3].linBreakForce != PX_MAX_F32) || (constraintDescs[3].angBreakForce != PX_MAX_F32));
		
		//OK, I think that's everything loaded in

		header->invMass0D0 = invMass0;
		header->invMass1D1 = invMass1;
		header->angD0 = invInertiaScale0;
		header->angD1 = invInertiaScale1;
		header->body0WorkOffset[0] = workOffsetX;
		header->body0WorkOffset[1] = workOffsetY;
		header->body0WorkOffset[2] = workOffsetZ;

		header->count = maxRows;
		header->type = DY_SC_TYPE_BLOCK_1D;
		header->linBreakImpulse = V4Scale(linBreakForce, dtV);
		header->angBreakImpulse = V4Scale(angBreakForce, dtV);
		header->counts[0] = PxTo8(constraintDescs[0].numRows);
		header->counts[1] = PxTo8(constraintDescs[1].numRows);
		header->counts[2] = PxTo8(constraintDescs[2].numRows);
		header->counts[3] = PxTo8(constraintDescs[3].numRows);

		Vec4V ca2WX, ca2WY, ca2WZ;
		Vec4V cb2WX, cb2WY, cb2WZ;

		Vec4V ca2W0 = V4LoadU(&constraintDescs[0].cA2w.x);
		Vec4V ca2W1 = V4LoadU(&constraintDescs[1].cA2w.x);
		Vec4V ca2W2 = V4LoadU(&constraintDescs[2].cA2w.x);
		Vec4V ca2W3 = V4LoadU(&constraintDescs[3].cA2w.x);

		Vec4V cb2W0 = V4LoadU(&constraintDescs[0].cB2w.x);
		Vec4V cb2W1 = V4LoadU(&constraintDescs[1].cB2w.x);
		Vec4V cb2W2 = V4LoadU(&constraintDescs[2].cB2w.x);
		Vec4V cb2W3 = V4LoadU(&constraintDescs[3].cB2w.x);

		PX_TRANSPOSE_44_34(ca2W0, ca2W1, ca2W2, ca2W3, ca2WX, ca2WY, ca2WZ);
		PX_TRANSPOSE_44_34(cb2W0, cb2W1, cb2W2, cb2W3, cb2WX, cb2WY, cb2WZ);

		Vec4V pos00 = V4LoadA(&constraintDescs[0].body0TxI->body2WorldP.x);
		Vec4V pos01 = V4LoadA(&constraintDescs[0].body1TxI->body2WorldP.x);
		Vec4V pos10 = V4LoadA(&constraintDescs[1].body0TxI->body2WorldP.x);
		Vec4V pos11 = V4LoadA(&constraintDescs[1].body1TxI->body2WorldP.x);
		Vec4V pos20 = V4LoadA(&constraintDescs[2].body0TxI->body2WorldP.x);
		Vec4V pos21 = V4LoadA(&constraintDescs[2].body1TxI->body2WorldP.x);
		Vec4V pos30 = V4LoadA(&constraintDescs[3].body0TxI->body2WorldP.x);
		Vec4V pos31 = V4LoadA(&constraintDescs[3].body1TxI->body2WorldP.x);

		Vec4V pos0X, pos0Y, pos0Z;
		Vec4V pos1X, pos1Y, pos1Z;

		PX_TRANSPOSE_44_34(pos00, pos10, pos20, pos30, pos0X, pos0Y, pos0Z);
		PX_TRANSPOSE_44_34(pos01, pos11, pos21, pos31, pos1X, pos1Y, pos1Z);

		Vec4V linVel00 = V4LoadA(&constraintDescs[0].bodyData0->originalLinearVelocity.x);
		Vec4V linVel01 = V4LoadA(&constraintDescs[0].bodyData1->originalLinearVelocity.x);
		Vec4V angState00 = V4LoadA(&constraintDescs[0].bodyData0->originalAngularVelocity.x);
		Vec4V angState01 = V4LoadA(&constraintDescs[0].bodyData1->originalAngularVelocity.x);

		Vec4V linVel10 = V4LoadA(&constraintDescs[1].bodyData0->originalLinearVelocity.x);
		Vec4V linVel11 = V4LoadA(&constraintDescs[1].bodyData1->originalLinearVelocity.x);
		Vec4V angState10 = V4LoadA(&constraintDescs[1].bodyData0->originalAngularVelocity.x);
		Vec4V angState11 = V4LoadA(&constraintDescs[1].bodyData1->originalAngularVelocity.x);

		Vec4V linVel20 = V4LoadA(&constraintDescs[2].bodyData0->originalLinearVelocity.x);
		Vec4V linVel21 = V4LoadA(&constraintDescs[2].bodyData1->originalLinearVelocity.x);
		Vec4V angState20 = V4LoadA(&constraintDescs[2].bodyData0->originalAngularVelocity.x);
		Vec4V angState21 = V4LoadA(&constraintDescs[2].bodyData1->originalAngularVelocity.x);

		Vec4V linVel30 = V4LoadA(&constraintDescs[3].bodyData0->originalLinearVelocity.x);
		Vec4V linVel31 = V4LoadA(&constraintDescs[3].bodyData1->originalLinearVelocity.x);
		Vec4V angState30 = V4LoadA(&constraintDescs[3].bodyData0->originalAngularVelocity.x);
		Vec4V angState31 = V4LoadA(&constraintDescs[3].bodyData1->originalAngularVelocity.x);

		Vec4V linVel0T0, linVel0T1, linVel0T2;
		Vec4V linVel1T0, linVel1T1, linVel1T2;
		Vec4V angState0T0, angState0T1, angState0T2;
		Vec4V angState1T0, angState1T1, angState1T2;

		PX_TRANSPOSE_44_34(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2);
		PX_TRANSPOSE_44_34(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2);
		PX_TRANSPOSE_44_34(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2);
		PX_TRANSPOSE_44_34(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2);

		const Vec4V raWorldX = V4Sub(ca2WX, pos0X);
		const Vec4V raWorldY = V4Sub(ca2WY, pos0Y);
		const Vec4V raWorldZ = V4Sub(ca2WZ, pos0Z);

		const Vec4V rbWorldX = V4Sub(cb2WX, pos1X);
		const Vec4V rbWorldY = V4Sub(cb2WY, pos1Y);
		const Vec4V rbWorldZ = V4Sub(cb2WZ, pos1Z);
		
		header->rAWorld[0] = raWorldX;
		header->rAWorld[1] = raWorldY;
		header->rAWorld[2] = raWorldZ;

		header->rBWorld[0] = rbWorldX;
		header->rBWorld[1] = rbWorldY;
		header->rBWorld[2] = rbWorldZ;

		//Now we loop over the constraints and build the results...

		PxU32 index0 = 0;
		PxU32 endIndex0 = constraintDescs[0].numRows - 1;
		PxU32 index1 = startIndex[1];
		PxU32 endIndex1 = index1 + constraintDescs[1].numRows - 1;
		PxU32 index2 = startIndex[2];
		PxU32 endIndex2 = index2 + constraintDescs[2].numRows - 1;
		PxU32 index3 = startIndex[3];
		PxU32 endIndex3 = index3 + constraintDescs[3].numRows - 1;

		const Vec4V one = V4One();

		PxU32 orthoCount0 = 0, orthoCount1 = 0, orthoCount2 = 0, orthoCount3 = 0;
		for (PxU32 a = 0; a < 3; ++a)
		{
			header->angOrthoAxis0X[a] = V4Zero();
			header->angOrthoAxis0Y[a] = V4Zero();
			header->angOrthoAxis0Z[a] = V4Zero();

			header->angOrthoAxis1X[a] = V4Zero();
			header->angOrthoAxis1Y[a] = V4Zero();
			header->angOrthoAxis1Z[a] = V4Zero();

			header->angOrthoRecipResponse[a] = V4Zero();
			header->angOrthoError[a] = V4Zero();
		}

		for (PxU32 a = 0; a < maxRows; ++a)
		{
			const bool finished[] = { a >= constraintDescs[0].numRows, a >= constraintDescs[1].numRows, a >= constraintDescs[2].numRows, a >= constraintDescs[3].numRows };
			BoolV bFinished = BLoad(finished);
			SolverConstraint1DStep4* c = reinterpret_cast<SolverConstraint1DStep4*>(currPtr);
			currPtr += stride;

			Px1DConstraint* con0 = allSorted[index0];
			Px1DConstraint* con1 = allSorted[index1];
			Px1DConstraint* con2 = allSorted[index2];
			Px1DConstraint* con3 = allSorted[index3];

			const bool angularConstraint[4] =
			{
				!!(con0->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
				!!(con1->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
				!!(con2->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
				!!(con3->flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT),
			};

			BoolV bAngularConstraint = BLoad(angularConstraint);

			Vec4V cangDelta00 = V4LoadA(&angSqrtInvInertia0[index0].x);
			Vec4V cangDelta01 = V4LoadA(&angSqrtInvInertia0[index1].x);
			Vec4V cangDelta02 = V4LoadA(&angSqrtInvInertia0[index2].x);
			Vec4V cangDelta03 = V4LoadA(&angSqrtInvInertia0[index3].x);

			Vec4V cangDelta10 = V4LoadA(&angSqrtInvInertia1[index0].x);
			Vec4V cangDelta11 = V4LoadA(&angSqrtInvInertia1[index1].x);
			Vec4V cangDelta12 = V4LoadA(&angSqrtInvInertia1[index2].x);
			Vec4V cangDelta13 = V4LoadA(&angSqrtInvInertia1[index3].x);

			index0 = index0 == endIndex0 ? index0 : index0 + 1;
			index1 = index1 == endIndex1 ? index1 : index1 + 1;
			index2 = index2 == endIndex2 ? index2 : index2 + 1;
			index3 = index3 == endIndex3 ? index3 : index3 + 1;

			PxReal minImpulse0, minImpulse1, minImpulse2, minImpulse3;
			PxReal maxImpulse0, maxImpulse1, maxImpulse2, maxImpulse3;
			computeMinMaxImpulseOrForceAsImpulse(
				con0->minImpulse, con0->maxImpulse, 
				con0->flags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT, constraintDescs[0].driveLimitsAreForces, simDt,
				minImpulse0, maxImpulse0);
			computeMinMaxImpulseOrForceAsImpulse(
				con1->minImpulse, con1->maxImpulse, 
				con1->flags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT, constraintDescs[1].driveLimitsAreForces, simDt,
				minImpulse1, maxImpulse1);
			computeMinMaxImpulseOrForceAsImpulse(
				con2->minImpulse, con2->maxImpulse, 
				con2->flags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT, constraintDescs[2].driveLimitsAreForces, simDt,
				minImpulse2, maxImpulse2);
			computeMinMaxImpulseOrForceAsImpulse(
				con3->minImpulse, con3->maxImpulse, 
				con3->flags & Px1DConstraintFlag::eHAS_DRIVE_LIMIT, constraintDescs[3].driveLimitsAreForces, simDt,
				minImpulse3, maxImpulse3);
			const Vec4V minImpulse = V4LoadXYZW(minImpulse0, minImpulse1, minImpulse2, minImpulse3);
			const Vec4V maxImpulse = V4LoadXYZW(maxImpulse0, maxImpulse1, maxImpulse2, maxImpulse3);

			Vec4V clin00 = V4LoadA(&con0->linear0.x);
			Vec4V clin01 = V4LoadA(&con1->linear0.x);
			Vec4V clin02 = V4LoadA(&con2->linear0.x);
			Vec4V clin03 = V4LoadA(&con3->linear0.x);

			Vec4V clin0X, clin0Y, clin0Z;

			PX_TRANSPOSE_44_34(clin00, clin01, clin02, clin03, clin0X, clin0Y, clin0Z);

			Vec4V cang00 = V4LoadA(&con0->angular0.x);
			Vec4V cang01 = V4LoadA(&con1->angular0.x);
			Vec4V cang02 = V4LoadA(&con2->angular0.x);
			Vec4V cang03 = V4LoadA(&con3->angular0.x);

			Vec4V cang0X, cang0Y, cang0Z;

			PX_TRANSPOSE_44_34(cang00, cang01, cang02, cang03, cang0X, cang0Y, cang0Z);

			Vec4V cang10 = V4LoadA(&con0->angular1.x);
			Vec4V cang11 = V4LoadA(&con1->angular1.x);
			Vec4V cang12 = V4LoadA(&con2->angular1.x);
			Vec4V cang13 = V4LoadA(&con3->angular1.x);

			Vec4V cang1X, cang1Y, cang1Z;

			PX_TRANSPOSE_44_34(cang10, cang11, cang12, cang13, cang1X, cang1Y, cang1Z);

			Vec4V angDelta0X, angDelta0Y, angDelta0Z;

			PX_TRANSPOSE_44_34(cangDelta00, cangDelta01, cangDelta02, cangDelta03, angDelta0X, angDelta0Y, angDelta0Z);

			c->flags[0] = 0;
			c->flags[1] = 0;
			c->flags[2] = 0;
			c->flags[3] = 0;

			c->lin0[0] = V4Sel(bFinished, zero, clin0X);
			c->lin0[1] = V4Sel(bFinished, zero, clin0Y);
			c->lin0[2] = V4Sel(bFinished, zero, clin0Z);
			c->ang0[0] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang0X, zero);
			c->ang0[1] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang0Y, zero);
			c->ang0[2] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang0Z, zero);
			c->angularErrorScale = V4Sel(bAngularConstraint, one, zero);

			c->minImpulse = minImpulse;
			c->maxImpulse = maxImpulse;
			c->appliedForce = zero;
			if (isResidualReportingEnabled) 
			{
				SolverConstraint1DStep4WithResidual* cc = static_cast<SolverConstraint1DStep4WithResidual*>(c);
				cc->residualPosIter = zero;
				cc->residualVelIter = zero;
			}

			const Vec4V lin0MagSq = V4MulAdd(clin0Z, clin0Z, V4MulAdd(clin0Y, clin0Y, V4Mul(clin0X, clin0X)));
			const Vec4V cang0DotAngDelta = V4MulAdd(angDelta0Z, angDelta0Z, V4MulAdd(angDelta0Y, angDelta0Y, V4Mul(angDelta0X, angDelta0X)));

			Vec4V unitResponse = V4MulAdd(lin0MagSq, invMass0, V4Mul(cang0DotAngDelta, invInertiaScale0));

			Vec4V clin10 = V4LoadA(&con0->linear1.x);
			Vec4V clin11 = V4LoadA(&con1->linear1.x);
			Vec4V clin12 = V4LoadA(&con2->linear1.x);
			Vec4V clin13 = V4LoadA(&con3->linear1.x);

			Vec4V clin1X, clin1Y, clin1Z;
			PX_TRANSPOSE_44_34(clin10, clin11, clin12, clin13, clin1X, clin1Y, clin1Z);

			Vec4V angDelta1X, angDelta1Y, angDelta1Z;

			PX_TRANSPOSE_44_34(cangDelta10, cangDelta11, cangDelta12, cangDelta13, angDelta1X, angDelta1Y, angDelta1Z);

			const Vec4V lin1MagSq = V4MulAdd(clin1Z, clin1Z, V4MulAdd(clin1Y, clin1Y, V4Mul(clin1X, clin1X)));
			const Vec4V cang1DotAngDelta = V4MulAdd(angDelta1Z, angDelta1Z, V4MulAdd(angDelta1Y, angDelta1Y, V4Mul(angDelta1X, angDelta1X)));

			c->lin1[0] = V4Sel(bFinished, zero, clin1X);
			c->lin1[1] = V4Sel(bFinished, zero, clin1Y);
			c->lin1[2] = V4Sel(bFinished, zero, clin1Z);

			c->ang1[0] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang1X, zero);
			c->ang1[1] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang1Y, zero);
			c->ang1[2] = V4Sel(BAndNot(bAngularConstraint, bFinished), cang1Z, zero);

			unitResponse = V4Add(unitResponse, V4MulAdd(lin1MagSq, invMass1, V4Mul(cang1DotAngDelta, invInertiaScale1)));

			const Vec4V lnormalVel0 = V4MulAdd(clin0X, linVel0T0, V4MulAdd(clin0Y, linVel0T1, V4Mul(clin0Z, linVel0T2)));
			const Vec4V lnormalVel1 = V4MulAdd(clin1X, linVel1T0, V4MulAdd(clin1Y, linVel1T1, V4Mul(clin1Z, linVel1T2)));

			const Vec4V angVel0 = V4MulAdd(cang0X, angState0T0, V4MulAdd(cang0Y, angState0T1, V4Mul(cang0Z, angState0T2)));
			const Vec4V angVel1 = V4MulAdd(angDelta1X, angState1T0, V4MulAdd(angDelta1Y, angState1T1, V4Mul(angDelta1Z, angState1T2)));

			const Vec4V normalVel0 = V4Add(lnormalVel0, angVel0);
			const Vec4V normalVel1 = V4Add(lnormalVel1, angVel1);

			const Vec4V normalVel = V4Sub(normalVel0, normalVel1);

			angDelta0X = V4Mul(angDelta0X, invInertiaScale0);
			angDelta0Y = V4Mul(angDelta0Y, invInertiaScale0);
			angDelta0Z = V4Mul(angDelta0Z, invInertiaScale0);

			angDelta1X = V4Mul(angDelta1X, invInertiaScale1);
			angDelta1Y = V4Mul(angDelta1Y, invInertiaScale1);
			angDelta1Z = V4Mul(angDelta1Z, invInertiaScale1);
		
			{
				//Constant inputs
				const Px1DConstraint* constraints[4] = {con0, con1, con2, con3};
				const PxReal* unitResponses4 = reinterpret_cast<const PxReal*>(&unitResponse);
				const PxReal* nVel0s4 = reinterpret_cast<const PxReal*>(&normalVel0);
				const PxReal* nVel1s4 =  reinterpret_cast<const PxReal*>(&normalVel1);
				const PxReal* jointSpeedForRestitutionBounces4 =  reinterpret_cast<const PxReal*>(&normalVel);
				const bool isBody0Kinematics[4] = {isKinematic00, isKinematic10, isKinematic20, isKinematic30};
				const bool isBody1Kinematics[4] = {isKinematic01, isKinematic11, isKinematic21, isKinematic31};

				//outputs
				PxReal* biasScales4 = reinterpret_cast<PxReal*>(&c->biasScale);
				PxReal* errors4 =  reinterpret_cast<PxReal*>(&c->error);
				PxReal* velMultipliers4 = reinterpret_cast<PxReal*>(&c->velMultiplier);
				PxReal* targetScales4 =  reinterpret_cast<PxReal*>(&c->velTarget);
				PxReal* maxBiasSpeeds4 = reinterpret_cast<PxReal*>(&c->maxBias);
				PxReal recipResponses[4];
				PxReal originalError[4];

				for(PxU32 i = 0; i < 4; i++)
				{

					if(a < constraintDescs[i].numRows)
					{
						const PxReal minRowResponseI = constraintDescs[i].minResponseThreshold;
						const PxU16 constraintFlagsI = constraints[i]->flags;
						const PxReal stiffnessI = constraints[i]->mods.spring.stiffness;
						const PxReal dampingI = constraints[i]->mods.spring.damping;
						const PxReal restitutionI = constraints[i]->mods.bounce.restitution;
						const PxReal bounceThresholdVelocityI = constraints[i]->mods.bounce.velocityThreshold;
						const PxReal geometricErrorI = constraints[i]->geometricError;
						originalError[i] = geometricErrorI;

						const PxReal velocityTargetI = constraints[i]->velocityTarget;
						const PxReal jointSpeedForRestitutionBounceI = jointSpeedForRestitutionBounces4[i];
						const PxReal unitResponseI = unitResponses4[i];
					
						const PxReal recipUnitResponseI = computeRecipUnitResponse(unitResponseI, minRowResponseI);
						recipResponses[i] = recipUnitResponseI;

						const PxReal maxBiasVelocityI = computeMaxBiasVelocityTGS(
							constraintFlagsI, 
							jointSpeedForRestitutionBounceI, bounceThresholdVelocityI, 
							restitutionI, geometricErrorI,
							false, lengthScale, recipSimDt);

						PxReal initJointSpeedI = 0.f;
						if (isBody0Kinematics[i])
							initJointSpeedI -= nVel0s4[i];
						if (isBody1Kinematics[i])
							initJointSpeedI += nVel1s4[i];

						const Constraint1dSolverConstantsTGS desc = compute1dConstraintSolverConstantsTGS(
								constraintFlagsI,
								stiffnessI, dampingI,
								restitutionI, bounceThresholdVelocityI,
								geometricErrorI, velocityTargetI,
								jointSpeedForRestitutionBounceI, initJointSpeedI, 
								unitResponseI, recipUnitResponseI, 
								erp, 
								stepDt, recipStepDt);
					
						biasScales4[i] = desc.biasScale;
						errors4[i] = desc.error;
						velMultipliers4[i] = desc.velMultiplier;
						targetScales4[i] = desc.targetVel;
						maxBiasSpeeds4[i] = maxBiasVelocityI;
					}	
					else
					{
						biasScales4[i] = 0.0f;
						errors4[i] = 0.0f;
						velMultipliers4[i] = 0.0f;
						targetScales4[i] = 0.0f;
						maxBiasSpeeds4[i] = 0.0f;
					}

					raiseInternalFlagsTGS(constraints[i]->flags, constraints[i]->solveHint, c->flags[i]);
				}

				PxVec4* angOrthoAxes0X = reinterpret_cast<PxVec4*>(header->angOrthoAxis0X);
				PxVec4* angOrthoAxes0Y = reinterpret_cast<PxVec4*>(header->angOrthoAxis0Y);
				PxVec4* angOrthoAxes0Z = reinterpret_cast<PxVec4*>(header->angOrthoAxis0Z);
				PxVec4* angOrthoAxes1X = reinterpret_cast<PxVec4*>(header->angOrthoAxis1X);
				PxVec4* angOrthoAxes1Y = reinterpret_cast<PxVec4*>(header->angOrthoAxis1Y);
				PxVec4* angOrthoAxes1Z = reinterpret_cast<PxVec4*>(header->angOrthoAxis1Z);
				PxVec4* orthoRecipResponse = reinterpret_cast<PxVec4*>(header->angOrthoRecipResponse);
				PxVec4* orthoError = reinterpret_cast<PxVec4*>(header->angOrthoError);

				const PxVec4& ang0X = reinterpret_cast<const PxVec4&>(angDelta0X);
				const PxVec4& ang0Y = reinterpret_cast<const PxVec4&>(angDelta0Y);
				const PxVec4& ang0Z = reinterpret_cast<const PxVec4&>(angDelta0Z);

				const PxVec4& ang1X = reinterpret_cast<const PxVec4&>(angDelta1X);
				const PxVec4& ang1Y = reinterpret_cast<const PxVec4&>(angDelta1Y);
				const PxVec4& ang1Z = reinterpret_cast<const PxVec4&>(angDelta1Z);
				
				setOrthoData(
					ang0X.x, ang0Y.x, ang0Z.x, ang1X.x, ang1Y.x, ang1Z.x, 
					recipResponses[0], originalError[0],
					angOrthoAxes0X[orthoCount0].x, angOrthoAxes0Y[orthoCount0].x, angOrthoAxes0Z[orthoCount0].x, 
					angOrthoAxes1X[orthoCount0].x, angOrthoAxes1Y[orthoCount0].x, angOrthoAxes1Z[orthoCount0].x, 
					orthoRecipResponse[orthoCount0].x, orthoError[orthoCount0].x, 
					constraintDescs[0].disablePreprocessing, con0->solveHint,
					c->flags[0], orthoCount0, a >= constraintDescs[0].numRows);

				setOrthoData(
					ang0X.y, ang0Y.y, ang0Z.y, ang1X.y, ang1Y.y, ang1Z.y, 
					recipResponses[1], originalError[1],
					angOrthoAxes0X[orthoCount1].y, angOrthoAxes0Y[orthoCount1].y, angOrthoAxes0Z[orthoCount1].y, 
					angOrthoAxes1X[orthoCount1].y, angOrthoAxes1Y[orthoCount1].y, angOrthoAxes1Z[orthoCount1].y, 
					orthoRecipResponse[orthoCount1].y, orthoError[orthoCount1].y, 
					constraintDescs[1].disablePreprocessing, con1->solveHint,
					c->flags[1], orthoCount1, a >= constraintDescs[1].numRows);

				setOrthoData(
					ang0X.z, ang0Y.z, ang0Z.z, ang1X.z, ang1Y.z, ang1Z.z, 
					recipResponses[2], originalError[2],
					angOrthoAxes0X[orthoCount2].z, angOrthoAxes0Y[orthoCount2].z, angOrthoAxes0Z[orthoCount2].z,
					angOrthoAxes1X[orthoCount2].z, angOrthoAxes1Y[orthoCount2].z, angOrthoAxes1Z[orthoCount2].z, 
					orthoRecipResponse[orthoCount2].z, orthoError[orthoCount2].z, 
					constraintDescs[2].disablePreprocessing, con2->solveHint,
					c->flags[2], orthoCount2, a >= constraintDescs[2].numRows);

				setOrthoData(
					ang0X.w, ang0Y.w, ang0Z.w, ang1X.w, ang1Y.w, ang1Z.w, 
					recipResponses[3], originalError[3],
					angOrthoAxes0X[orthoCount3].w, angOrthoAxes0Y[orthoCount3].w, angOrthoAxes0Z[orthoCount3].w, 
					angOrthoAxes1X[orthoCount3].w, angOrthoAxes1Y[orthoCount3].w, angOrthoAxes1Z[orthoCount3].w, 
					orthoRecipResponse[orthoCount3].w, orthoError[orthoCount3].w, 
					constraintDescs[3].disablePreprocessing, con3->solveHint,
					c->flags[3], orthoCount3, a >= constraintDescs[3].numRows);
			}
		}
		*(reinterpret_cast<PxU32*>(currPtr)) = 0;
		*(reinterpret_cast<PxU32*>(currPtr + 4)) = 0;
	}

	//OK, we're ready to allocate and solve prep these constraints now :-)
	return SolverConstraintPrepState::eSUCCESS;
}

static void solveContact4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, const bool doFriction, const PxReal minPenetration,
	const PxReal elapsedTimeF32, SolverContext& cache)
{
	PxTGSSolverBodyVel& b00 = *desc[0].tgsBodyA;
	PxTGSSolverBodyVel& b01 = *desc[0].tgsBodyB;
	PxTGSSolverBodyVel& b10 = *desc[1].tgsBodyA;
	PxTGSSolverBodyVel& b11 = *desc[1].tgsBodyB;
	PxTGSSolverBodyVel& b20 = *desc[2].tgsBodyA;
	PxTGSSolverBodyVel& b21 = *desc[2].tgsBodyB;
	PxTGSSolverBodyVel& b30 = *desc[3].tgsBodyA;
	PxTGSSolverBodyVel& b31 = *desc[3].tgsBodyB;

	const Vec4V minPen = V4Load(minPenetration);

	const Vec4V elapsedTime = V4Load(elapsedTimeF32);

	//We'll need this.
	const Vec4V vZero = V4Zero();

	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V linVel01 = V4LoadA(&b01.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularVelocity.x);
	Vec4V angState01 = V4LoadA(&b01.angularVelocity.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&b11.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularVelocity.x);
	Vec4V angState11 = V4LoadA(&b11.angularVelocity.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&b21.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularVelocity.x);
	Vec4V angState21 = V4LoadA(&b21.angularVelocity.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&b31.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularVelocity.x);
	Vec4V angState31 = V4LoadA(&b31.angularVelocity.x);

	Vec4V linVel0T0, linVel0T1, linVel0T2, linVel0T3;
	Vec4V linVel1T0, linVel1T1, linVel1T2, linVel1T3;
	Vec4V angState0T0, angState0T1, angState0T2, angState0T3;
	Vec4V angState1T0, angState1T1, angState1T2, angState1T3;

	PX_TRANSPOSE_44(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2, linVel0T3);
	PX_TRANSPOSE_44(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2, linVel1T3);
	PX_TRANSPOSE_44(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2, angState0T3);
	PX_TRANSPOSE_44(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2, angState1T3);

	Vec4V linDelta00_ = V4LoadA(&b00.deltaLinDt.x);
	Vec4V linDelta01_ = V4LoadA(&b01.deltaLinDt.x);
	Vec4V angDelta00_ = V4LoadA(&b00.deltaAngDt.x);
	Vec4V angDelta01_ = V4LoadA(&b01.deltaAngDt.x);

	Vec4V linDelta10_ = V4LoadA(&b10.deltaLinDt.x);
	Vec4V linDelta11_ = V4LoadA(&b11.deltaLinDt.x);
	Vec4V angDelta10_ = V4LoadA(&b10.deltaAngDt.x);
	Vec4V angDelta11_ = V4LoadA(&b11.deltaAngDt.x);

	Vec4V linDelta20_ = V4LoadA(&b20.deltaLinDt.x);
	Vec4V linDelta21_ = V4LoadA(&b21.deltaLinDt.x);
	Vec4V angDelta20_ = V4LoadA(&b20.deltaAngDt.x);
	Vec4V angDelta21_ = V4LoadA(&b21.deltaAngDt.x);

	Vec4V linDelta30_ = V4LoadA(&b30.deltaLinDt.x);
	Vec4V linDelta31_ = V4LoadA(&b31.deltaLinDt.x);
	Vec4V angDelta30_ = V4LoadA(&b30.deltaAngDt.x);
	Vec4V angDelta31_ = V4LoadA(&b31.deltaAngDt.x);

	Vec4V linDelta0T0, linDelta0T1, linDelta0T2;
	Vec4V linDelta1T0, linDelta1T1, linDelta1T2;
	Vec4V angDelta0T0, angDelta0T1, angDelta0T2;
	Vec4V angDelta1T0, angDelta1T1, angDelta1T2;

	PX_TRANSPOSE_44_34(linDelta00_, linDelta10_, linDelta20_, linDelta30_, linDelta0T0, linDelta0T1, linDelta0T2);
	PX_TRANSPOSE_44_34(linDelta01_, linDelta11_, linDelta21_, linDelta31_, linDelta1T0, linDelta1T1, linDelta1T2);
	PX_TRANSPOSE_44_34(angDelta00_, angDelta10_, angDelta20_, angDelta30_, angDelta0T0, angDelta0T1, angDelta0T2);
	PX_TRANSPOSE_44_34(angDelta01_, angDelta11_, angDelta21_, angDelta31_, angDelta1T0, angDelta1T1, angDelta1T2);

	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	Vec4V vMax = V4Splat(FMax());

	SolverContactHeaderStepBlock* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

	const Vec4V invMassA = hdr->invMass0D0;
	const Vec4V invMassB = hdr->invMass1D1;

	const Vec4V sumInvMass = V4Add(invMassA, invMassB);

	Vec4V linDeltaX = V4Sub(linDelta0T0, linDelta1T0);
	Vec4V linDeltaY = V4Sub(linDelta0T1, linDelta1T1);
	Vec4V linDeltaZ = V4Sub(linDelta0T2, linDelta1T2);

	Dy::ErrorAccumulator error;
	const bool residualReportingActive = cache.contactErrorAccumulator;

	while (currPtr < last)
	{
		hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

		PX_ASSERT(hdr->type == DY_SC_TYPE_BLOCK_RB_CONTACT);

		currPtr = reinterpret_cast<PxU8*>(const_cast<SolverContactHeaderStepBlock*>(hdr) + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		const bool hasMaxImpulse = (hdr->flag & SolverContactHeaderStepBlock::eHAS_MAX_IMPULSE) != 0;

		Vec4V* appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		SolverContactPointStepBlock* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStepBlock*>(currPtr);

		Vec4V* maxImpulses;
		currPtr = reinterpret_cast<PxU8*>(contacts + numNormalConstr);
		PxU32 maxImpulseMask = 0;
		if (hasMaxImpulse)
		{
			maxImpulseMask = 0xFFFFFFFF;
			maxImpulses = reinterpret_cast<Vec4V*>(currPtr);
			currPtr += sizeof(Vec4V) * numNormalConstr;
		}
		else
		{
			maxImpulses = &vMax;
		}

		/*SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(currPtr);
		if (numFrictionConstr)
			currPtr += sizeof(SolverFrictionSharedData4);*/

		Vec4V* frictionAppliedForce = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numFrictionConstr;

		const SolverContactFrictionStepBlock* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStepBlock*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionStepBlock);

		Vec4V accumulatedNormalImpulse = vZero;

		const Vec4V angD0 = hdr->angDom0;
		const Vec4V angD1 = hdr->angDom1;

		const Vec4V _normalT0 = hdr->normalX;
		const Vec4V _normalT1 = hdr->normalY;
		const Vec4V _normalT2 = hdr->normalZ;

		Vec4V contactNormalVel1 = V4Mul(linVel0T0, _normalT0);
		Vec4V contactNormalVel3 = V4Mul(linVel1T0, _normalT0);
		contactNormalVel1 = V4MulAdd(linVel0T1, _normalT1, contactNormalVel1);
		contactNormalVel3 = V4MulAdd(linVel1T1, _normalT1, contactNormalVel3);
		contactNormalVel1 = V4MulAdd(linVel0T2, _normalT2, contactNormalVel1);
		contactNormalVel3 = V4MulAdd(linVel1T2, _normalT2, contactNormalVel3);

		const Vec4V maxPenBias = hdr->maxPenBias;

		Vec4V relVel1 = V4Sub(contactNormalVel1, contactNormalVel3);

		Vec4V deltaNormalV = V4Mul(linDeltaX, _normalT0);
		deltaNormalV = V4MulAdd(linDeltaY, _normalT1, deltaNormalV);
		deltaNormalV = V4MulAdd(linDeltaZ, _normalT2, deltaNormalV);

		Vec4V accumDeltaF = vZero;

		for (PxU32 i = 0; i<numNormalConstr; i++)
		{
			const SolverContactPointStepBlock& c = contacts[i];

			/*PxU32 offset = 0;
			PxPrefetchLine(prefetchAddress, offset += 64);
			PxPrefetchLine(prefetchAddress, offset += 64);
			PxPrefetchLine(prefetchAddress, offset += 64);
			prefetchAddress += offset;*/

			const Vec4V appliedForce = appliedForces[i];
			const Vec4V maxImpulse = maxImpulses[i & maxImpulseMask];

			Vec4V contactNormalVel2 = V4Mul(c.raXnI[0], angState0T0);
			Vec4V contactNormalVel4 = V4Mul(c.rbXnI[0], angState1T0);

			contactNormalVel2 = V4MulAdd(c.raXnI[1], angState0T1, contactNormalVel2);
			contactNormalVel4 = V4MulAdd(c.rbXnI[1], angState1T1, contactNormalVel4);

			contactNormalVel2 = V4MulAdd(c.raXnI[2], angState0T2, contactNormalVel2);
			contactNormalVel4 = V4MulAdd(c.rbXnI[2], angState1T2, contactNormalVel4);

			const Vec4V normalVel = V4Add(relVel1, V4Sub(contactNormalVel2, contactNormalVel4));

			Vec4V angDelta0 = V4Mul(angDelta0T0, c.raXnI[0]);
			Vec4V angDelta1 = V4Mul(angDelta1T0, c.rbXnI[0]);
			angDelta0 = V4MulAdd(angDelta0T1, c.raXnI[1], angDelta0);
			angDelta1 = V4MulAdd(angDelta1T1, c.rbXnI[1], angDelta1);
			angDelta0 = V4MulAdd(angDelta0T2, c.raXnI[2], angDelta0);
			angDelta1 = V4MulAdd(angDelta1T2, c.rbXnI[2], angDelta1);

			const Vec4V deltaAng = V4Sub(angDelta0, angDelta1);

			const Vec4V targetVel = c.targetVelocity;

			const Vec4V deltaBias = V4Sub(V4Add(deltaNormalV, deltaAng), V4Mul(targetVel, elapsedTime));
			//const Vec4V deltaBias = V4Add(deltaNormalV, deltaAng);

			const Vec4V biasCoefficient = c.biasCoefficient;

			const Vec4V sep = V4Max(minPen, V4Add(c.separation, deltaBias));

			const Vec4V bias = V4Min(V4Neg(maxPenBias), V4Mul(biasCoefficient, sep));

			const Vec4V velMultiplier = c.velMultiplier;

			const Vec4V tVelBias = V4Mul(bias, c.recipResponse);

			const Vec4V _deltaF = V4Max(V4Sub(tVelBias, V4Mul(V4Sub(normalVel, targetVel), velMultiplier)), V4Neg(appliedForce));
			//Vec4V deltaF = V4NegMulSub(normalVel, c.velMultiplier, c.biasedErr);

			const Vec4V newAppliedForce = V4Min(V4Add(appliedForce, _deltaF), maxImpulse);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);

			if (residualReportingActive)
				error.accumulateErrorLocalV4(deltaF, velMultiplier);

			accumDeltaF = V4Add(accumDeltaF, deltaF);

			const Vec4V angDetaF0 = V4Mul(deltaF, angD0);
			const Vec4V angDetaF1 = V4Mul(deltaF, angD1);

			relVel1 = V4MulAdd(sumInvMass, deltaF, relVel1);

			angState0T0 = V4MulAdd(c.raXnI[0], angDetaF0, angState0T0);
			angState1T0 = V4NegMulSub(c.rbXnI[0], angDetaF1, angState1T0);

			angState0T1 = V4MulAdd(c.raXnI[1], angDetaF0, angState0T1);
			angState1T1 = V4NegMulSub(c.rbXnI[1], angDetaF1, angState1T1);

			angState0T2 = V4MulAdd(c.raXnI[2], angDetaF0, angState0T2);
			angState1T2 = V4NegMulSub(c.rbXnI[2], angDetaF1, angState1T2);

			appliedForces[i] = newAppliedForce;

			accumulatedNormalImpulse = V4Add(accumulatedNormalImpulse, newAppliedForce);
		}

		const Vec4V accumDeltaF_IM0 = V4Mul(accumDeltaF, invMassA);
		const Vec4V accumDeltaF_IM1 = V4Mul(accumDeltaF, invMassB);

		linVel0T0 = V4MulAdd(_normalT0, accumDeltaF_IM0, linVel0T0);
		linVel1T0 = V4NegMulSub(_normalT0, accumDeltaF_IM1, linVel1T0);
		linVel0T1 = V4MulAdd(_normalT1, accumDeltaF_IM0, linVel0T1);
		linVel1T1 = V4NegMulSub(_normalT1, accumDeltaF_IM1, linVel1T1);
		linVel0T2 = V4MulAdd(_normalT2, accumDeltaF_IM0, linVel0T2);
		linVel1T2 = V4NegMulSub(_normalT2, accumDeltaF_IM1, linVel1T2);

		if (doFriction && numFrictionConstr)
		{
			const Vec4V staticFric = hdr->staticFriction;
			const Vec4V dynamicFric = hdr->dynamicFriction;

			const Vec4V maxFrictionImpulse = V4Add(V4Mul(staticFric, accumulatedNormalImpulse), V4Load(1e-5f));
			const Vec4V maxDynFrictionImpulse = V4Mul(dynamicFric, accumulatedNormalImpulse);
			BoolV broken = BFFFF();

			for (PxU32 i = 0; i<numFrictionConstr; i+=2)
			{
				const SolverContactFrictionStepBlock& f0 = frictions[i];
				const SolverContactFrictionStepBlock& f1 = frictions[i+1];

				/*PxU32 offset = 0;
				PxPrefetchLine(prefetchAddress, offset += 64);
				PxPrefetchLine(prefetchAddress, offset += 64);
				PxPrefetchLine(prefetchAddress, offset += 64);
				PxPrefetchLine(prefetchAddress, offset += 64);
				prefetchAddress += offset;*/

				const Vec4V appliedForce0 = frictionAppliedForce[i];
				const Vec4V appliedForce1 = frictionAppliedForce[i+1];

				const Vec4V normalT00 = f0.normal[0];
				const Vec4V normalT10 = f0.normal[1];
				const Vec4V normalT20 = f0.normal[2];

				const Vec4V normalT01 = f1.normal[0];
				const Vec4V normalT11 = f1.normal[1];
				const Vec4V normalT21 = f1.normal[2];

				Vec4V normalVel10 = V4Mul(linVel0T0, normalT00);
				Vec4V normalVel20 = V4Mul(f0.raXnI[0], angState0T0);
				Vec4V normalVel30 = V4Mul(linVel1T0, normalT00);
				Vec4V normalVel40 = V4Mul(f0.rbXnI[0], angState1T0);
				Vec4V normalVel11 = V4Mul(linVel0T0, normalT01);
				Vec4V normalVel21 = V4Mul(f1.raXnI[0], angState0T0);
				Vec4V normalVel31 = V4Mul(linVel1T0, normalT01);
				Vec4V normalVel41 = V4Mul(f1.rbXnI[0], angState1T0);

				normalVel10 = V4MulAdd(linVel0T1, normalT10, normalVel10);
				normalVel20 = V4MulAdd(f0.raXnI[1], angState0T1, normalVel20);
				normalVel30 = V4MulAdd(linVel1T1, normalT10, normalVel30);
				normalVel40 = V4MulAdd(f0.rbXnI[1], angState1T1, normalVel40);
				normalVel11 = V4MulAdd(linVel0T1, normalT11, normalVel11);
				normalVel21 = V4MulAdd(f1.raXnI[1], angState0T1, normalVel21);
				normalVel31 = V4MulAdd(linVel1T1, normalT11, normalVel31);
				normalVel41 = V4MulAdd(f1.rbXnI[1], angState1T1, normalVel41);

				normalVel10 = V4MulAdd(linVel0T2, normalT20, normalVel10);
				normalVel20 = V4MulAdd(f0.raXnI[2], angState0T2, normalVel20);
				normalVel30 = V4MulAdd(linVel1T2, normalT20, normalVel30);
				normalVel40 = V4MulAdd(f0.rbXnI[2], angState1T2, normalVel40);
				normalVel11 = V4MulAdd(linVel0T2, normalT21, normalVel11);
				normalVel21 = V4MulAdd(f1.raXnI[2], angState0T2, normalVel21);
				normalVel31 = V4MulAdd(linVel1T2, normalT21, normalVel31);
				normalVel41 = V4MulAdd(f1.rbXnI[2], angState1T2, normalVel41);

				const Vec4V normalVel0_tmp1 = V4Add(normalVel10, normalVel20);
				const Vec4V normalVel0_tmp2 = V4Add(normalVel30, normalVel40);
				const Vec4V normalVel0 = V4Sub(normalVel0_tmp1, normalVel0_tmp2);
				const Vec4V normalVel1_tmp1 = V4Add(normalVel11, normalVel21);
				const Vec4V normalVel1_tmp2 = V4Add(normalVel31, normalVel41);
				const Vec4V normalVel1 = V4Sub(normalVel1_tmp1, normalVel1_tmp2);

				Vec4V deltaV0 = V4Mul(linDeltaX, normalT00);
				deltaV0 = V4MulAdd(linDeltaY, normalT10, deltaV0);
				deltaV0 = V4MulAdd(linDeltaZ, normalT20, deltaV0);
				Vec4V deltaV1 = V4Mul(linDeltaX, normalT01);
				deltaV1 = V4MulAdd(linDeltaY, normalT11, deltaV1);
				deltaV1 = V4MulAdd(linDeltaZ, normalT21, deltaV1);

				Vec4V angDelta00 = V4Mul(angDelta0T0, f0.raXnI[0]);
				Vec4V angDelta10 = V4Mul(angDelta1T0, f0.rbXnI[0]);
				angDelta00 = V4MulAdd(angDelta0T1, f0.raXnI[1], angDelta00);
				angDelta10 = V4MulAdd(angDelta1T1, f0.rbXnI[1], angDelta10);
				angDelta00 = V4MulAdd(angDelta0T2, f0.raXnI[2], angDelta00);
				angDelta10 = V4MulAdd(angDelta1T2, f0.rbXnI[2], angDelta10);

				Vec4V angDelta01 = V4Mul(angDelta0T0, f1.raXnI[0]);
				Vec4V angDelta11 = V4Mul(angDelta1T0, f1.rbXnI[0]);
				angDelta01 = V4MulAdd(angDelta0T1, f1.raXnI[1], angDelta01);
				angDelta11 = V4MulAdd(angDelta1T1, f1.rbXnI[1], angDelta11);
				angDelta01 = V4MulAdd(angDelta0T2, f1.raXnI[2], angDelta01);
				angDelta11 = V4MulAdd(angDelta1T2, f1.rbXnI[2], angDelta11);

				const Vec4V deltaAng0 = V4Sub(angDelta00, angDelta10);
				const Vec4V deltaAng1 = V4Sub(angDelta01, angDelta11);

				const Vec4V deltaBias0 = V4Sub(V4Add(deltaV0, deltaAng0), V4Mul(f0.targetVel, elapsedTime));
				const Vec4V deltaBias1 = V4Sub(V4Add(deltaV1, deltaAng1), V4Mul(f1.targetVel, elapsedTime));

				const Vec4V error0 = V4Add(f0.error, deltaBias0);
				const Vec4V error1 = V4Add(f1.error, deltaBias1);

				const Vec4V bias0 = V4Mul(error0, f0.biasCoefficient);
				const Vec4V bias1 = V4Mul(error1, f1.biasCoefficient);

				const Vec4V tmp10 = V4NegMulSub(V4Sub(bias0, f0.targetVel), f0.velMultiplier, appliedForce0);
				const Vec4V tmp11 = V4NegMulSub(V4Sub(bias1, f1.targetVel), f1.velMultiplier, appliedForce1);

				const Vec4V totalImpulse0 = V4NegMulSub(normalVel0, f0.velMultiplier, tmp10);
				const Vec4V totalImpulse1 = V4NegMulSub(normalVel1, f1.velMultiplier, tmp11);

				const Vec4V totalImpulse = V4Sqrt(V4MulAdd(totalImpulse0, totalImpulse0, V4Mul(totalImpulse1, totalImpulse1)));

				const BoolV clamped = V4IsGrtr(totalImpulse, maxFrictionImpulse);

				broken = BOr(broken, clamped);

				const Vec4V totalClamped = V4Sel(broken, V4Min(totalImpulse, maxDynFrictionImpulse), totalImpulse);
				const Vec4V ratio = V4Sel(V4IsGrtr(totalImpulse, vZero), V4Div(totalClamped, totalImpulse), vZero);

				const Vec4V newAppliedForce0 = V4Mul(totalImpulse0, ratio);
				const Vec4V newAppliedForce1 = V4Mul(totalImpulse1, ratio);

				const Vec4V deltaF0 = V4Sub(newAppliedForce0, appliedForce0);
				const Vec4V deltaF1 = V4Sub(newAppliedForce1, appliedForce1);

				if (residualReportingActive)
					error.accumulateErrorLocalV4(deltaF0, f0.velMultiplier, deltaF1, f1.velMultiplier);

				frictionAppliedForce[i] = newAppliedForce0;
				frictionAppliedForce[i+1] = newAppliedForce1;

				const Vec4V deltaFIM00 = V4Mul(deltaF0, invMassA);
				const Vec4V deltaFIM10 = V4Mul(deltaF0, invMassB);
				const Vec4V angDetaF00 = V4Mul(deltaF0, angD0);
				const Vec4V angDetaF10 = V4Mul(deltaF0, angD1);

				const Vec4V deltaFIM01 = V4Mul(deltaF1, invMassA);
				const Vec4V deltaFIM11 = V4Mul(deltaF1, invMassB);
				const Vec4V angDetaF01 = V4Mul(deltaF1, angD0);
				const Vec4V angDetaF11 = V4Mul(deltaF1, angD1);

				linVel0T0 = V4MulAdd(normalT00, deltaFIM00, V4MulAdd(normalT01, deltaFIM01, linVel0T0));
				linVel1T0 = V4NegMulSub(normalT00, deltaFIM10, V4NegMulSub(normalT01, deltaFIM11, linVel1T0));
				angState0T0 = V4MulAdd(f0.raXnI[0], angDetaF00, V4MulAdd(f1.raXnI[0], angDetaF01, angState0T0));
				angState1T0 = V4NegMulSub(f0.rbXnI[0], angDetaF10, V4NegMulSub(f1.rbXnI[0], angDetaF11, angState1T0));

				linVel0T1 = V4MulAdd(normalT10, deltaFIM00, V4MulAdd(normalT11, deltaFIM01, linVel0T1));
				linVel1T1 = V4NegMulSub(normalT10, deltaFIM10, V4NegMulSub(normalT11, deltaFIM11, linVel1T1));
				angState0T1 = V4MulAdd(f0.raXnI[1], angDetaF00, V4MulAdd(f1.raXnI[1], angDetaF01, angState0T1));
				angState1T1 = V4NegMulSub(f0.rbXnI[1], angDetaF10, V4NegMulSub(f1.rbXnI[1], angDetaF11, angState1T1));

				linVel0T2 = V4MulAdd(normalT20, deltaFIM00, V4MulAdd(normalT21, deltaFIM01, linVel0T2));
				linVel1T2 = V4NegMulSub(normalT20, deltaFIM10, V4NegMulSub(normalT21, deltaFIM11, linVel1T2));
				angState0T2 = V4MulAdd(f0.raXnI[2], angDetaF00, V4MulAdd(f1.raXnI[2], angDetaF01, angState0T2));
				angState1T2 = V4NegMulSub(f0.rbXnI[2], angDetaF10, V4NegMulSub(f1.rbXnI[2], angDetaF11, angState1T2));
			}
			hdr->broken = broken;
		}
	}

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(linVel1T0, linVel1T1, linVel1T2, linVel1T3, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_44(angState1T0, angState1T1, angState1T2, angState1T3, angState01, angState11, angState21, angState31);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());

	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(angState00, &b00.angularVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(angState10, &b10.angularVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(angState20, &b20.angularVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);
	V4StoreA(angState30, &b30.angularVelocity.x);

	if (desc[0].bodyBDataIndex != 0)
	{
		V4StoreA(linVel01, &b01.linearVelocity.x);
		V4StoreA(angState01, &b01.angularVelocity.x);
	}
	if (desc[1].bodyBDataIndex != 0)
	{
		V4StoreA(linVel11, &b11.linearVelocity.x);
		V4StoreA(angState11, &b11.angularVelocity.x);
	}
	if (desc[2].bodyBDataIndex != 0)
	{
		V4StoreA(linVel21, &b21.linearVelocity.x);
		V4StoreA(angState21, &b21.angularVelocity.x);
	}
	if (desc[3].bodyBDataIndex != 0)
	{
		V4StoreA(linVel31, &b31.linearVelocity.x);
		V4StoreA(angState31, &b31.angularVelocity.x);
	}

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());

	if (residualReportingActive)
		error.accumulateErrorGlobal(*cache.contactErrorAccumulator);
}

// VR: used in both PGS and TGS
void computeFrictionImpulseBlock(
	const Vec4V& axis0X, const Vec4V& axis0Y, const Vec4V& axis0Z,
	const Vec4V& axis1X, const Vec4V& axis1Y, const Vec4V& axis1Z,
	const Vec4V appliedForce0, const Vec4V appliedForce1,
	Vec4V& impulse0, Vec4V& impulse1, Vec4V& impulse2, Vec4V& impulse3
);

static void writeBackContact4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext* /*cache*/)
{
	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;
	PxReal* PX_RESTRICT vForceWriteback0 = reinterpret_cast<PxReal*>(desc[0].writeBack);
	PxReal* PX_RESTRICT vForceWriteback1 = reinterpret_cast<PxReal*>(desc[1].writeBack);
	PxReal* PX_RESTRICT vForceWriteback2 = reinterpret_cast<PxReal*>(desc[2].writeBack);
	PxReal* PX_RESTRICT vForceWriteback3 = reinterpret_cast<PxReal*>(desc[3].writeBack);
	PxVec3* PX_RESTRICT vFrictionWriteback0 = reinterpret_cast<PxVec3*>(desc[0].writeBackFriction);
	PxVec3* PX_RESTRICT vFrictionWriteback1 = reinterpret_cast<PxVec3*>(desc[1].writeBackFriction);
	PxVec3* PX_RESTRICT vFrictionWriteback2 = reinterpret_cast<PxVec3*>(desc[2].writeBackFriction);
	PxVec3* PX_RESTRICT vFrictionWriteback3 = reinterpret_cast<PxVec3*>(desc[3].writeBackFriction);

	//const PxU8 type = *desc[0].constraint;
	const PxU32 contactSize = sizeof(SolverContactPointStepBlock);
	const PxU32 frictionSize = sizeof(SolverContactFrictionStepBlock);

	Vec4V normalForce = V4Zero();

	//We'll need this.
	//const Vec4V vZero	= V4Zero();

	bool writeBackThresholds[4] = { false, false, false, false };

	while ((currPtr < last))
	{
		SolverContactHeaderStepBlock* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(hdr + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		Vec4V* PX_RESTRICT appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		//SolverContactBatchPointBase4* PX_RESTRICT contacts = (SolverContactBatchPointBase4*)currPtr;
		currPtr += (numNormalConstr * contactSize);

		const bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

		if (hasMaxImpulse)
			currPtr += sizeof(Vec4V) * numNormalConstr;

		Vec4V* frictionAppliedForce = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numFrictionConstr;

		SolverContactFrictionStepBlock* PX_RESTRICT frictions = (SolverContactFrictionStepBlock*)currPtr;
		currPtr += (numFrictionConstr * frictionSize);

		writeBackThresholds[0] = hdr->flags[0] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[1] = hdr->flags[1] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[2] = hdr->flags[2] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[3] = hdr->flags[3] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;

		for (PxU32 i = 0; i<numNormalConstr; i++)
		{
			//contacts = (SolverContactBatchPointBase4*)(((PxU8*)contacts) + contactSize);
			const FloatV appliedForce0 = V4GetX(appliedForces[i]);
			const FloatV appliedForce1 = V4GetY(appliedForces[i]);
			const FloatV appliedForce2 = V4GetZ(appliedForces[i]);
			const FloatV appliedForce3 = V4GetW(appliedForces[i]);

			normalForce = V4Add(normalForce, appliedForces[i]);

			if (vForceWriteback0 && i < hdr->numNormalConstrs[0])
				FStore(appliedForce0, vForceWriteback0++);
			if (vForceWriteback1 && i < hdr->numNormalConstrs[1])
				FStore(appliedForce1, vForceWriteback1++);
			if (vForceWriteback2 && i < hdr->numNormalConstrs[2])
				FStore(appliedForce2, vForceWriteback2++);
			if (vForceWriteback3 && i < hdr->numNormalConstrs[3])
				FStore(appliedForce3, vForceWriteback3++);
		}

		// Writeback friction impulses
		if (numFrictionConstr)
		{
			//We will have either 4 or 2 frictions (with friction pairs).
			//With torsional friction, we may have 3 (a single friction anchor + twist).
			const PxU32 numFrictionPairs = (numFrictionConstr & 6);

			for (PxU32 i = 0; i < numFrictionPairs; i += 2)
			{
				SolverContactFrictionStepBlock& f0 = frictions[i + 0];
				SolverContactFrictionStepBlock& f1 = frictions[i + 1];

				const Vec4V axis0X = f0.normal[0];
				const Vec4V axis0Y = f0.normal[1];
				const Vec4V axis0Z = f0.normal[2];

				const Vec4V axis1X = f1.normal[0];
				const Vec4V axis1Y = f1.normal[1];
				const Vec4V axis1Z = f1.normal[2];

				const Vec4V appliedForce0 = frictionAppliedForce[i + 0];
				const Vec4V appliedForce1 = frictionAppliedForce[i + 1];

				Vec4V impulse0, impulse1, impulse2, impulse3;
				computeFrictionImpulseBlock(axis0X, axis0Y, axis0Z,
											axis1X, axis1Y, axis1Z,
											appliedForce0, appliedForce1,
											impulse0, impulse1, impulse2, impulse3);

				if (vFrictionWriteback0)
					V3StoreU(Vec3V_From_Vec4V_WUndefined(impulse0), vFrictionWriteback0[i / 2]);
				if (vFrictionWriteback1)
					V3StoreU(Vec3V_From_Vec4V_WUndefined(impulse1), vFrictionWriteback1[i / 2]);
				if (vFrictionWriteback2)
					V3StoreU(Vec3V_From_Vec4V_WUndefined(impulse2), vFrictionWriteback2[i / 2]);
				if (vFrictionWriteback3)
					V3StoreU(Vec3V_From_Vec4V_WUndefined(impulse3), vFrictionWriteback3[i / 2]);
			}
		}

		if (numFrictionConstr)
		{
			PX_ALIGN(16, PxU32 broken[4]);
			BStoreA(hdr->broken, broken);

			PxU8* frictionCounts = hdr->numNormalConstrs;

			for (PxU32 a = 0; a < 4; ++a)
			{
				if (frictionCounts[a] && broken[a])
					*hdr->frictionBrokenWritebackByte[a] = 1;	// PT: bad L2 miss here
			}
		}
	}

	PX_UNUSED(writeBackThresholds);

#if 0
	if (cache)
	{

		PX_ALIGN(16, PxReal nf[4]);
		V4StoreA(normalForce, nf);

		Sc::ShapeInteraction** shapeInteractions = reinterpret_cast<SolverContactHeader4*>(desc[0].constraint)->shapeInteraction;

		for (PxU32 a = 0; a < 4; ++a)
		{
			if (writeBackThresholds[a] && desc[a].linkIndexA == PxSolverConstraintDesc::NO_LINK && desc[a].linkIndexB == PxSolverConstraintDesc::NO_LINK &&
				nf[a] != 0.f && (bd0[a]->reportThreshold < PX_MAX_REAL || bd1[a]->reportThreshold < PX_MAX_REAL))
			{
				ThresholdStreamElement elt;
				elt.normalForce = nf[a];
				elt.threshold = PxMin<float>(bd0[a]->reportThreshold, bd1[a]->reportThreshold);
				elt.nodeIndexA = bd0[a]->nodeIndex;
				elt.nodeIndexB = bd1[a]->nodeIndex;
				elt.shapeInteraction = shapeInteractions[a];
				PxOrder(elt.nodeIndexA, elt.nodeIndexB);
				PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);
				PX_ASSERT(cache.mThresholdStreamIndex < cache.mThresholdStreamLength);
				cache.mThresholdStream[cache.mThresholdStreamIndex++] = elt;
			}
		}
	}
#endif
}

void solveContact4(DY_TGS_SOLVE_METHOD_PARAMS)
{
	PX_UNUSED(txInertias);
	//PX_UNUSED(cache);

	solveContact4_Block(desc + hdr.startIndex, true, minPenetration, elapsedTime, cache);
}

void writeBackContact4(DY_TGS_WRITEBACK_METHOD_PARAMS)
{
	writeBackContact4_Block(desc + hdr.startIndex, cache);
}

static PX_FORCE_INLINE Vec4V V4Dot3(const Vec4V& x0, const Vec4V& y0, const Vec4V& z0, const Vec4V& x1, const Vec4V& y1, const Vec4V& z1)
{
	return V4MulAdd(x0, x1, V4MulAdd(y0, y1, V4Mul(z0, z1)));
}

static void solve1DStep4(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxTGSSolverBodyTxInertia* const txInertias, PxReal elapsedTimeF32, const SolverContext& cache)
{
	PxU8* PX_RESTRICT bPtr = desc->constraint;
	if (bPtr == NULL)
		return;

	const FloatV elapsedTime = FLoad(elapsedTimeF32);

	PxTGSSolverBodyVel& b00 = *desc[0].tgsBodyA;
	PxTGSSolverBodyVel& b01 = *desc[0].tgsBodyB;
	PxTGSSolverBodyVel& b10 = *desc[1].tgsBodyA;
	PxTGSSolverBodyVel& b11 = *desc[1].tgsBodyB;
	PxTGSSolverBodyVel& b20 = *desc[2].tgsBodyA;
	PxTGSSolverBodyVel& b21 = *desc[2].tgsBodyB;
	PxTGSSolverBodyVel& b30 = *desc[3].tgsBodyA;
	PxTGSSolverBodyVel& b31 = *desc[3].tgsBodyB;

	const PxTGSSolverBodyTxInertia& txI00 = txInertias[desc[0].bodyADataIndex];
	const PxTGSSolverBodyTxInertia& txI01 = txInertias[desc[0].bodyBDataIndex];
	const PxTGSSolverBodyTxInertia& txI10 = txInertias[desc[1].bodyADataIndex];
	const PxTGSSolverBodyTxInertia& txI11 = txInertias[desc[1].bodyBDataIndex];
	const PxTGSSolverBodyTxInertia& txI20 = txInertias[desc[2].bodyADataIndex];
	const PxTGSSolverBodyTxInertia& txI21 = txInertias[desc[2].bodyBDataIndex];
	const PxTGSSolverBodyTxInertia& txI30 = txInertias[desc[3].bodyADataIndex];
	const PxTGSSolverBodyTxInertia& txI31 = txInertias[desc[3].bodyBDataIndex];

	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V linVel01 = V4LoadA(&b01.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularVelocity.x);
	Vec4V angState01 = V4LoadA(&b01.angularVelocity.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&b11.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularVelocity.x);
	Vec4V angState11 = V4LoadA(&b11.angularVelocity.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&b21.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularVelocity.x);
	Vec4V angState21 = V4LoadA(&b21.angularVelocity.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&b31.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularVelocity.x);
	Vec4V angState31 = V4LoadA(&b31.angularVelocity.x);

	Vec4V linVel0T0, linVel0T1, linVel0T2, linVel0T3;
	Vec4V linVel1T0, linVel1T1, linVel1T2, linVel1T3;
	Vec4V angState0T0, angState0T1, angState0T2, angState0T3;
	Vec4V angState1T0, angState1T1, angState1T2, angState1T3;

	PX_TRANSPOSE_44(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2, linVel0T3);
	PX_TRANSPOSE_44(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2, linVel1T3);
	PX_TRANSPOSE_44(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2, angState0T3);
	PX_TRANSPOSE_44(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2, angState1T3);

	Vec4V linDelta00 = V4LoadA(&b00.deltaLinDt.x);
	Vec4V linDelta01 = V4LoadA(&b01.deltaLinDt.x);
	Vec4V angDelta00 = V4LoadA(&b00.deltaAngDt.x);
	Vec4V angDelta01 = V4LoadA(&b01.deltaAngDt.x);

	Vec4V linDelta10 = V4LoadA(&b10.deltaLinDt.x);
	Vec4V linDelta11 = V4LoadA(&b11.deltaLinDt.x);
	Vec4V angDelta10 = V4LoadA(&b10.deltaAngDt.x);
	Vec4V angDelta11 = V4LoadA(&b11.deltaAngDt.x);

	Vec4V linDelta20 = V4LoadA(&b20.deltaLinDt.x);
	Vec4V linDelta21 = V4LoadA(&b21.deltaLinDt.x);
	Vec4V angDelta20 = V4LoadA(&b20.deltaAngDt.x);
	Vec4V angDelta21 = V4LoadA(&b21.deltaAngDt.x);

	Vec4V linDelta30 = V4LoadA(&b30.deltaLinDt.x);
	Vec4V linDelta31 = V4LoadA(&b31.deltaLinDt.x);
	Vec4V angDelta30 = V4LoadA(&b30.deltaAngDt.x);
	Vec4V angDelta31 = V4LoadA(&b31.deltaAngDt.x);

	Vec4V linDelta0T0, linDelta0T1, linDelta0T2;
	Vec4V linDelta1T0, linDelta1T1, linDelta1T2;
	Vec4V angDelta0T0, angDelta0T1, angDelta0T2;
	Vec4V angDelta1T0, angDelta1T1, angDelta1T2;

	PX_TRANSPOSE_44_34(linDelta00, linDelta10, linDelta20, linDelta30, linDelta0T0, linDelta0T1, linDelta0T2);
	PX_TRANSPOSE_44_34(linDelta01, linDelta11, linDelta21, linDelta31, linDelta1T0, linDelta1T1, linDelta1T2);
	PX_TRANSPOSE_44_34(angDelta00, angDelta10, angDelta20, angDelta30, angDelta0T0, angDelta0T1, angDelta0T2);
	PX_TRANSPOSE_44_34(angDelta01, angDelta11, angDelta21, angDelta31, angDelta1T0, angDelta1T1, angDelta1T2);

	const SolverConstraint1DHeaderStep4* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep4*>(bPtr);
	PxU8* PX_RESTRICT base = reinterpret_cast<PxU8*>(bPtr + sizeof(SolverConstraint1DHeaderStep4));

	Vec4V invInertia00X = V4LoadU(&txI00.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia00Y = V4LoadU(&txI00.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia00Z = Vec4V_From_Vec3V(V3LoadU(txI00.sqrtInvInertia.column2));

	Vec4V invInertia10X = V4LoadU(&txI10.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia10Y = V4LoadU(&txI10.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia10Z = Vec4V_From_Vec3V(V3LoadU(txI10.sqrtInvInertia.column2));

	Vec4V invInertia20X = V4LoadU(&txI20.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia20Y = V4LoadU(&txI20.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia20Z = Vec4V_From_Vec3V(V3LoadU(txI20.sqrtInvInertia.column2));

	Vec4V invInertia30X = V4LoadU(&txI30.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia30Y = V4LoadU(&txI30.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia30Z = Vec4V_From_Vec3V(V3LoadU(txI30.sqrtInvInertia.column2));

	Vec4V invInertia01X = V4LoadU(&txI01.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia01Y = V4LoadU(&txI01.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia01Z = Vec4V_From_Vec3V(V3LoadU(txI01.sqrtInvInertia.column2));

	Vec4V invInertia11X = V4LoadU(&txI11.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia11Y = V4LoadU(&txI11.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia11Z = Vec4V_From_Vec3V(V3LoadU(txI11.sqrtInvInertia.column2));

	Vec4V invInertia21X = V4LoadU(&txI21.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia21Y = V4LoadU(&txI21.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia21Z = Vec4V_From_Vec3V(V3LoadU(txI21.sqrtInvInertia.column2));

	Vec4V invInertia31X = V4LoadU(&txI31.sqrtInvInertia.column0.x);	// PT: safe because 'column1' follows 'column0' in PxMat33
	Vec4V invInertia31Y = V4LoadU(&txI31.sqrtInvInertia.column1.x);	// PT: safe because 'column2' follows 'column1' in PxMat33
	Vec4V invInertia31Z = Vec4V_From_Vec3V(V3LoadU(txI31.sqrtInvInertia.column2));

	Vec4V invInertia0X0, invInertia0X1, invInertia0X2;
	Vec4V invInertia0Y0, invInertia0Y1, invInertia0Y2;
	Vec4V invInertia0Z0, invInertia0Z1, invInertia0Z2;

	Vec4V invInertia1X0, invInertia1X1, invInertia1X2;
	Vec4V invInertia1Y0, invInertia1Y1, invInertia1Y2;
	Vec4V invInertia1Z0, invInertia1Z1, invInertia1Z2;

	PX_TRANSPOSE_44_34(invInertia00X, invInertia10X, invInertia20X, invInertia30X, invInertia0X0, invInertia0Y0, invInertia0Z0);
	PX_TRANSPOSE_44_34(invInertia00Y, invInertia10Y, invInertia20Y, invInertia30Y, invInertia0X1, invInertia0Y1, invInertia0Z1);
	PX_TRANSPOSE_44_34(invInertia00Z, invInertia10Z, invInertia20Z, invInertia30Z, invInertia0X2, invInertia0Y2, invInertia0Z2);

	PX_TRANSPOSE_44_34(invInertia01X, invInertia11X, invInertia21X, invInertia31X, invInertia1X0, invInertia1Y0, invInertia1Z0);
	PX_TRANSPOSE_44_34(invInertia01Y, invInertia11Y, invInertia21Y, invInertia31Y, invInertia1X1, invInertia1Y1, invInertia1Z1);
	PX_TRANSPOSE_44_34(invInertia01Z, invInertia11Z, invInertia21Z, invInertia31Z, invInertia1X2, invInertia1Y2, invInertia1Z2);

	const Vec4V invInertiaScale0 = header->angD0;
	const Vec4V invInertiaScale1 = header->angD1;

	//KS - todo - load this a bit quicker...
	Vec4V rot00 = V4LoadA(&txI00.deltaBody2WorldQ.x);
	Vec4V rot01 = V4LoadA(&txI01.deltaBody2WorldQ.x);
	Vec4V rot10 = V4LoadA(&txI10.deltaBody2WorldQ.x);
	Vec4V rot11 = V4LoadA(&txI11.deltaBody2WorldQ.x);
	Vec4V rot20 = V4LoadA(&txI20.deltaBody2WorldQ.x);
	Vec4V rot21 = V4LoadA(&txI21.deltaBody2WorldQ.x);
	Vec4V rot30 = V4LoadA(&txI30.deltaBody2WorldQ.x);
	Vec4V rot31 = V4LoadA(&txI31.deltaBody2WorldQ.x);

	Vec4V rot0X, rot0Y, rot0Z, rot0W;
	Vec4V rot1X, rot1Y, rot1Z, rot1W;

	PX_TRANSPOSE_44(rot00, rot10, rot20, rot30, rot0X, rot0Y, rot0Z, rot0W);
	PX_TRANSPOSE_44(rot01, rot11, rot21, rot31, rot1X, rot1Y, rot1Z, rot1W);

	Vec4V raX, raY, raZ;
	Vec4V rbX, rbY, rbZ;

	QuatRotate4(rot0X, rot0Y, rot0Z, rot0W, header->rAWorld[0], header->rAWorld[1], header->rAWorld[2], raX, raY, raZ);
	QuatRotate4(rot1X, rot1Y, rot1Z, rot1W, header->rBWorld[0], header->rBWorld[1], header->rBWorld[2], rbX, rbY, rbZ);

	const Vec4V raMotionX = V4Sub(V4Add(raX, linDelta0T0), header->rAWorld[0]);
	const Vec4V raMotionY = V4Sub(V4Add(raY, linDelta0T1), header->rAWorld[1]);
	const Vec4V raMotionZ = V4Sub(V4Add(raZ, linDelta0T2), header->rAWorld[2]);
	const Vec4V rbMotionX = V4Sub(V4Add(rbX, linDelta1T0), header->rBWorld[0]);
	const Vec4V rbMotionY = V4Sub(V4Add(rbY, linDelta1T1), header->rBWorld[1]);
	const Vec4V rbMotionZ = V4Sub(V4Add(rbZ, linDelta1T2), header->rBWorld[2]);

	const Vec4V mass0 = header->invMass0D0;
	const Vec4V mass1 = header->invMass1D1;

	const VecU32V orthoMask = U4Load(DY_SC_FLAG_ORTHO_TARGET);
	const VecU32V limitMask = U4Load(DY_SC_FLAG_INEQUALITY);
	const VecU32V springFlagMask = U4Load(DY_SC_FLAG_SPRING);
	const Vec4V zero = V4Zero();
	const Vec4V one = V4One();

	Vec4V error0 = V4Add(header->angOrthoError[0], 
		V4Sub(V4Dot3(header->angOrthoAxis0X[0], header->angOrthoAxis0Y[0], header->angOrthoAxis0Z[0], angDelta0T0, angDelta0T1, angDelta0T2),
			V4Dot3(header->angOrthoAxis1X[0], header->angOrthoAxis1Y[0], header->angOrthoAxis1Z[0], angDelta1T0, angDelta1T1, angDelta1T2)));

	Vec4V error1 = V4Add(header->angOrthoError[1],
		V4Sub(V4Dot3(header->angOrthoAxis0X[1], header->angOrthoAxis0Y[1], header->angOrthoAxis0Z[1], angDelta0T0, angDelta0T1, angDelta0T2),
			V4Dot3(header->angOrthoAxis1X[1], header->angOrthoAxis1Y[1], header->angOrthoAxis1Z[1], angDelta1T0, angDelta1T1, angDelta1T2)));

	Vec4V error2 = V4Add(header->angOrthoError[2],
		V4Sub(V4Dot3(header->angOrthoAxis0X[2], header->angOrthoAxis0Y[2], header->angOrthoAxis0Z[2], angDelta0T0, angDelta0T1, angDelta0T2),
			V4Dot3(header->angOrthoAxis1X[2], header->angOrthoAxis1Y[2], header->angOrthoAxis1Z[2], angDelta1T0, angDelta1T1, angDelta1T2)));

	PxU32 stride = cache.contactErrorAccumulator ? sizeof(SolverConstraint1DStep4WithResidual) : sizeof(SolverConstraint1DStep4);

	const PxU32 count = header->count;
	for (PxU32 i = 0; i<count; ++i/*, base++*/)
	{
		PxPrefetchLine(base + stride);
		SolverConstraint1DStep4& c = reinterpret_cast<SolverConstraint1DStep4&>(*base);

		const Vec4V cangVel0X = V4Add(c.ang0[0], V4NegMulSub(raZ, c.lin0[1], V4Mul(raY, c.lin0[2])));
		const Vec4V cangVel0Y = V4Add(c.ang0[1], V4NegMulSub(raX, c.lin0[2], V4Mul(raZ, c.lin0[0])));
		const Vec4V cangVel0Z = V4Add(c.ang0[2], V4NegMulSub(raY, c.lin0[0], V4Mul(raX, c.lin0[1])));

		const Vec4V cangVel1X = V4Add(c.ang1[0], V4NegMulSub(rbZ, c.lin1[1], V4Mul(rbY, c.lin1[2])));
		const Vec4V cangVel1Y = V4Add(c.ang1[1], V4NegMulSub(rbX, c.lin1[2], V4Mul(rbZ, c.lin1[0])));
		const Vec4V cangVel1Z = V4Add(c.ang1[2], V4NegMulSub(rbY, c.lin1[0], V4Mul(rbX, c.lin1[1])));

		const VecU32V flags = U4LoadA(c.flags);

		const BoolV useOrtho = V4IsEqU32(V4U32and(flags, orthoMask), orthoMask);

		const Vec4V angOrthoCoefficient = V4Sel(useOrtho, one, zero);

		Vec4V delAngVel0X = V4Mul(invInertia0X0, cangVel0X);
		Vec4V delAngVel0Y = V4Mul(invInertia0X1, cangVel0X);
		Vec4V delAngVel0Z = V4Mul(invInertia0X2, cangVel0X);

		delAngVel0X = V4MulAdd(invInertia0Y0, cangVel0Y, delAngVel0X);
		delAngVel0Y = V4MulAdd(invInertia0Y1, cangVel0Y, delAngVel0Y);
		delAngVel0Z = V4MulAdd(invInertia0Y2, cangVel0Y, delAngVel0Z);

		delAngVel0X = V4MulAdd(invInertia0Z0, cangVel0Z, delAngVel0X);
		delAngVel0Y = V4MulAdd(invInertia0Z1, cangVel0Z, delAngVel0Y);
		delAngVel0Z = V4MulAdd(invInertia0Z2, cangVel0Z, delAngVel0Z);

		Vec4V delAngVel1X = V4Mul(invInertia1X0, cangVel1X);
		Vec4V delAngVel1Y = V4Mul(invInertia1X1, cangVel1X);
		Vec4V delAngVel1Z = V4Mul(invInertia1X2, cangVel1X);

		delAngVel1X = V4MulAdd(invInertia1Y0, cangVel1Y, delAngVel1X);
		delAngVel1Y = V4MulAdd(invInertia1Y1, cangVel1Y, delAngVel1Y);
		delAngVel1Z = V4MulAdd(invInertia1Y2, cangVel1Y, delAngVel1Z);

		delAngVel1X = V4MulAdd(invInertia1Z0, cangVel1Z, delAngVel1X);
		delAngVel1Y = V4MulAdd(invInertia1Z1, cangVel1Z, delAngVel1Y);
		delAngVel1Z = V4MulAdd(invInertia1Z2, cangVel1Z, delAngVel1Z);

		Vec4V err = c.error;
		{
			const Vec4V proj0 = V4Mul(V4MulAdd(header->angOrthoAxis0X[0], delAngVel0X, V4MulAdd(header->angOrthoAxis0Y[0], delAngVel0Y,
				V4MulAdd(header->angOrthoAxis0Z[0], delAngVel0Z, V4MulAdd(header->angOrthoAxis1X[0], delAngVel1X,
					V4MulAdd(header->angOrthoAxis1Y[0], delAngVel1Y, V4Mul(header->angOrthoAxis1Z[0], delAngVel1Z)))))), header->angOrthoRecipResponse[0]);

			const Vec4V proj1 = V4Mul(V4MulAdd(header->angOrthoAxis0X[1], delAngVel0X, V4MulAdd(header->angOrthoAxis0Y[1], delAngVel0Y,
				V4MulAdd(header->angOrthoAxis0Z[1], delAngVel0Z, V4MulAdd(header->angOrthoAxis1X[1], delAngVel1X,
					V4MulAdd(header->angOrthoAxis1Y[1], delAngVel1Y, V4Mul(header->angOrthoAxis1Z[1], delAngVel1Z)))))), header->angOrthoRecipResponse[1]);

			const Vec4V proj2 = V4Mul(V4MulAdd(header->angOrthoAxis0X[2], delAngVel0X, V4MulAdd(header->angOrthoAxis0Y[2], delAngVel0Y,
				V4MulAdd(header->angOrthoAxis0Z[2], delAngVel0Z, V4MulAdd(header->angOrthoAxis1X[2], delAngVel1X,
					V4MulAdd(header->angOrthoAxis1Y[2], delAngVel1Y, V4Mul(header->angOrthoAxis1Z[2], delAngVel1Z)))))), header->angOrthoRecipResponse[2]);

			const Vec4V delta0X = V4MulAdd(header->angOrthoAxis0X[0], proj0, V4MulAdd(header->angOrthoAxis0X[1], proj1, V4Mul(header->angOrthoAxis0X[2], proj2)));
			const Vec4V delta0Y = V4MulAdd(header->angOrthoAxis0Y[0], proj0, V4MulAdd(header->angOrthoAxis0Y[1], proj1, V4Mul(header->angOrthoAxis0Y[2], proj2)));
			const Vec4V delta0Z = V4MulAdd(header->angOrthoAxis0Z[0], proj0, V4MulAdd(header->angOrthoAxis0Z[1], proj1, V4Mul(header->angOrthoAxis0Z[2], proj2)));

			const Vec4V delta1X = V4MulAdd(header->angOrthoAxis1X[0], proj0, V4MulAdd(header->angOrthoAxis1X[1], proj1, V4Mul(header->angOrthoAxis1X[2], proj2)));
			const Vec4V delta1Y = V4MulAdd(header->angOrthoAxis1Y[0], proj0, V4MulAdd(header->angOrthoAxis1Y[1], proj1, V4Mul(header->angOrthoAxis1Y[2], proj2)));
			const Vec4V delta1Z = V4MulAdd(header->angOrthoAxis1Z[0], proj0, V4MulAdd(header->angOrthoAxis1Z[1], proj1, V4Mul(header->angOrthoAxis1Z[2], proj2)));

			delAngVel0X = V4NegMulSub(delta0X, angOrthoCoefficient, delAngVel0X);
			delAngVel0Y = V4NegMulSub(delta0Y, angOrthoCoefficient, delAngVel0Y);
			delAngVel0Z = V4NegMulSub(delta0Z, angOrthoCoefficient, delAngVel0Z);

			delAngVel1X = V4NegMulSub(delta1X, angOrthoCoefficient, delAngVel1X);
			delAngVel1Y = V4NegMulSub(delta1Y, angOrthoCoefficient, delAngVel1Y);
			delAngVel1Z = V4NegMulSub(delta1Z, angOrthoCoefficient, delAngVel1Z);

			const Vec4V orthoBasisError = V4Mul(c.biasScale, V4MulAdd(error0, proj0, V4MulAdd(error1, proj1, V4Mul(error2, proj2))));
			err = V4Sub(err, V4Mul(orthoBasisError, angOrthoCoefficient));
		}

		Vec4V ang0IX = V4Mul(invInertia0X0, delAngVel0X);
		Vec4V ang0IY = V4Mul(invInertia0X1, delAngVel0X);
		Vec4V ang0IZ = V4Mul(invInertia0X2, delAngVel0X);

		ang0IX = V4MulAdd(invInertia0Y0, delAngVel0Y, ang0IX);
		ang0IY = V4MulAdd(invInertia0Y1, delAngVel0Y, ang0IY);
		ang0IZ = V4MulAdd(invInertia0Y2, delAngVel0Y, ang0IZ);

		ang0IX = V4MulAdd(invInertia0Z0, delAngVel0Z, ang0IX);
		ang0IY = V4MulAdd(invInertia0Z1, delAngVel0Z, ang0IY);
		ang0IZ = V4MulAdd(invInertia0Z2, delAngVel0Z, ang0IZ);

		Vec4V ang1IX = V4Mul(invInertia1X0, delAngVel1X);
		Vec4V ang1IY = V4Mul(invInertia1X1, delAngVel1X);
		Vec4V ang1IZ = V4Mul(invInertia1X2, delAngVel1X);

		ang1IX = V4MulAdd(invInertia1Y0, delAngVel1Y, ang1IX);
		ang1IY = V4MulAdd(invInertia1Y1, delAngVel1Y, ang1IY);
		ang1IZ = V4MulAdd(invInertia1Y2, delAngVel1Y, ang1IZ);

		ang1IX = V4MulAdd(invInertia1Z0, delAngVel1Z, ang1IX);
		ang1IY = V4MulAdd(invInertia1Z1, delAngVel1Z, ang1IY);
		ang1IZ = V4MulAdd(invInertia1Z2, delAngVel1Z, ang1IZ);

		const Vec4V clinVel0X = c.lin0[0];
		const Vec4V clinVel0Y = c.lin0[1];
		const Vec4V clinVel0Z = c.lin0[2];

		const Vec4V clinVel1X = c.lin1[0];
		const Vec4V clinVel1Y = c.lin1[1];
		const Vec4V clinVel1Z = c.lin1[2];

		const Vec4V clinVel0X_ = c.lin0[0];
		const Vec4V clinVel0Y_ = c.lin0[1];
		const Vec4V clinVel0Z_ = c.lin0[2];

		const Vec4V clinVel1X_ = c.lin1[0];
		const Vec4V clinVel1Y_ = c.lin1[1];
		const Vec4V clinVel1Z_ = c.lin1[2];

		const aos::BoolV isSpringConstraint = V4IsEqU32(V4U32and(flags, springFlagMask), springFlagMask);
		
		const Vec4V errorChange = computeResolvedGeometricErrorTGSBlock(
			raMotionX, raMotionY, raMotionZ,
			rbMotionX, rbMotionY, rbMotionZ,
			clinVel0X_, clinVel0Y_, clinVel0Z_,
			clinVel1X_, clinVel1Y_, clinVel1Z_,
			angDelta0T0, angDelta0T1, angDelta0T2,
			angDelta1T0, angDelta1T1, angDelta1T2,
			delAngVel0X, delAngVel0Y, delAngVel0Z,
			delAngVel1X, delAngVel1Y, delAngVel1Z,
			c.angularErrorScale,
			isSpringConstraint, c.velTarget, elapsedTime);

		//KS - compute raXnI and effective mass. Unfortunately, the joints are noticeably less stable if we don't do this each 
		//iteration. It's skippable. If we do that, there's no need for the invInertiaTensors

		const Vec4V dotDelAngVel0 = V4MulAdd(delAngVel0X, delAngVel0X, V4MulAdd(delAngVel0Y, delAngVel0Y, V4Mul(delAngVel0Z, delAngVel0Z)));
		const Vec4V dotDelAngVel1 = V4MulAdd(delAngVel1X, delAngVel1X, V4MulAdd(delAngVel1Y, delAngVel1Y, V4Mul(delAngVel1Z, delAngVel1Z)));

		const Vec4V dotClinVel0 = V4MulAdd(clinVel0X, clinVel0X, V4MulAdd(clinVel0Y, clinVel0Y, V4Mul(clinVel0Z, clinVel0Z)));
		const Vec4V dotClinVel1 = V4MulAdd(clinVel1X, clinVel1X, V4MulAdd(clinVel1Y, clinVel1Y, V4Mul(clinVel1Z, clinVel1Z)));

		const Vec4V resp0 = V4MulAdd(mass0, dotClinVel0, V4Mul(invInertiaScale0, dotDelAngVel0));
		const Vec4V resp1 = V4MulAdd(mass1, dotClinVel1, V4Mul(invInertiaScale1, dotDelAngVel1));
		const Vec4V response = V4Add(resp0, resp1);
		const Vec4V recipResponse = V4Sel(V4IsGrtr(response, V4Zero()), V4Recip(response), V4Zero());
		
		const Vec4V vMul = V4Sel(isSpringConstraint, c.velMultiplier, V4Mul(recipResponse, c.velMultiplier));

		const Vec4V minBias = computeMinBiasTGSBlock(flags, limitMask, c.maxBias);
		const Vec4V unclampedBias = V4MulAdd(errorChange, c.biasScale, err);
		const Vec4V bias = V4Clamp(unclampedBias, minBias, c.maxBias);

		const Vec4V constant = V4Sel(isSpringConstraint, V4Add(bias, c.velTarget), V4Mul(recipResponse, V4Add(bias, c.velTarget)));

		const Vec4V normalVel0 = V4MulAdd(clinVel0X_, linVel0T0, V4MulAdd(clinVel0Y_, linVel0T1, V4Mul(clinVel0Z_, linVel0T2)));
		const Vec4V normalVel1 = V4MulAdd(clinVel1X_, linVel1T0, V4MulAdd(clinVel1Y_, linVel1T1, V4Mul(clinVel1Z_, linVel1T2)));

		const Vec4V angVel0 = V4MulAdd(delAngVel0X, angState0T0, V4MulAdd(delAngVel0Y, angState0T1, V4Mul(delAngVel0Z, angState0T2)));
		const Vec4V angVel1 = V4MulAdd(delAngVel1X, angState1T0, V4MulAdd(delAngVel1Y, angState1T1, V4Mul(delAngVel1Z, angState1T2)));

		const Vec4V normalVel = V4Add(V4Sub(normalVel0, normalVel1), V4Sub(angVel0, angVel1));

		const Vec4V unclampedForce = V4Add(c.appliedForce, V4MulAdd(vMul, normalVel, constant));
		const Vec4V clampedForce = V4Clamp(unclampedForce, c.minImpulse, c.maxImpulse);
		const Vec4V deltaF = V4Sub(clampedForce, c.appliedForce);

		c.appliedForce = clampedForce;
		if (cache.contactErrorAccumulator) 
		{
			SolverConstraint1DStep4WithResidual& cc = static_cast<SolverConstraint1DStep4WithResidual&>(c);
			const Vec4V residual = Dy::calculateResidualV4(deltaF, vMul);
			if (cache.isPositionIteration)
				cc.residualPosIter = residual;
			else
				cc.residualVelIter = residual;
		}

		const Vec4V deltaFIM0 = V4Mul(deltaF, mass0);
		const Vec4V deltaFIM1 = V4Mul(deltaF, mass1);
		const Vec4V angDetaF0 = V4Mul(deltaF, invInertiaScale0);
		const Vec4V angDetaF1 = V4Mul(deltaF, invInertiaScale1);

		linVel0T0 = V4MulAdd(clinVel0X_, deltaFIM0, linVel0T0);
		linVel1T0 = V4NegMulSub(clinVel1X_, deltaFIM1, linVel1T0);
		angState0T0 = V4MulAdd(delAngVel0X, angDetaF0, angState0T0);
		angState1T0 = V4NegMulSub(delAngVel1X, angDetaF1, angState1T0);

		linVel0T1 = V4MulAdd(clinVel0Y_, deltaFIM0, linVel0T1);
		linVel1T1 = V4NegMulSub(clinVel1Y_, deltaFIM1, linVel1T1);
		angState0T1 = V4MulAdd(delAngVel0Y, angDetaF0, angState0T1);
		angState1T1 = V4NegMulSub(delAngVel1Y, angDetaF1, angState1T1);

		linVel0T2 = V4MulAdd(clinVel0Z_, deltaFIM0, linVel0T2);
		linVel1T2 = V4NegMulSub(clinVel1Z_, deltaFIM1, linVel1T2);
		angState0T2 = V4MulAdd(delAngVel0Z, angDetaF0, angState0T2);
		angState1T2 = V4NegMulSub(delAngVel1Z, angDetaF1, angState1T2);

		base += stride;
	}

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(linVel1T0, linVel1T1, linVel1T2, linVel1T3, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_44(angState1T0, angState1T1, angState1T2, angState1T3, angState01, angState11, angState21, angState31);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());

	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(angState00, &b00.angularVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(angState10, &b10.angularVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(angState20, &b20.angularVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);
	V4StoreA(angState30, &b30.angularVelocity.x);

	V4StoreA(linVel01, &b01.linearVelocity.x);
	V4StoreA(angState01, &b01.angularVelocity.x);
	V4StoreA(linVel11, &b11.linearVelocity.x);
	V4StoreA(angState11, &b11.angularVelocity.x);
	V4StoreA(linVel21, &b21.linearVelocity.x);
	V4StoreA(angState21, &b21.angularVelocity.x);
	V4StoreA(linVel31, &b31.linearVelocity.x);
	V4StoreA(angState31, &b31.angularVelocity.x);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularVelocity.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularVelocity.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularVelocity.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularVelocity.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularVelocity.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularVelocity.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularVelocity.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularVelocity.isFinite());
}

void solve1D4(DY_TGS_SOLVE_METHOD_PARAMS)
{
	PX_UNUSED(minPenetration);
	PX_UNUSED(cache);

	solve1DStep4(desc + hdr.startIndex, txInertias, elapsedTime, cache);
}

void writeBack1D4(DY_TGS_WRITEBACK_METHOD_PARAMS)
{
	PX_UNUSED(cache);

	ConstraintWriteback* writeback0 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.startIndex].writeBack);
	ConstraintWriteback* writeback1 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.startIndex + 1].writeBack);
	ConstraintWriteback* writeback2 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.startIndex + 2].writeBack);
	ConstraintWriteback* writeback3 = reinterpret_cast<ConstraintWriteback*>(desc[hdr.startIndex + 3].writeBack);

	if (writeback0 || writeback1 || writeback2 || writeback3)
	{
		SolverConstraint1DHeaderStep4* header = reinterpret_cast<SolverConstraint1DHeaderStep4*>(desc[hdr.startIndex].constraint);
		PxU8* base = reinterpret_cast<PxU8*>(desc[hdr.startIndex].constraint + sizeof(SolverConstraint1DHeaderStep4));
		PxU32 stride = cache->contactErrorAccumulator ? sizeof(SolverConstraint1DStep4WithResidual) : sizeof(SolverConstraint1DStep4);

		const Vec4V zero = V4Zero();
		Vec4V linX(zero), linY(zero), linZ(zero);
		Vec4V angX(zero), angY(zero), angZ(zero);
		Vec4V residual(zero);
		Vec4V residualPosIter(zero);

		const PxU32 count = header->count;
		for (PxU32 i = 0; i<count; i++)
		{
			const SolverConstraint1DStep4* c = reinterpret_cast<const SolverConstraint1DStep4*>(base);

			//Load in flags
			const VecI32V flags = I4LoadU(reinterpret_cast<const PxI32*>(&c->flags[0]));
			//Work out masks
			const VecI32V mask = I4Load(DY_SC_FLAG_OUTPUT_FORCE);

			const VecI32V masked = VecI32V_And(flags, mask);
			const BoolV isEq = VecI32V_IsEq(masked, mask);

			const Vec4V appliedForce = V4Sel(isEq, c->appliedForce, zero);

			if (cache->contactErrorAccumulator) 
			{
				const SolverConstraint1DStep4WithResidual* cc = static_cast<const SolverConstraint1DStep4WithResidual*>(c);
				residual = V4MulAdd(cc->residualVelIter, cc->residualVelIter, residual);
				residualPosIter = V4MulAdd(cc->residualPosIter, cc->residualPosIter, residualPosIter);
			}

			linX = V4MulAdd(c->lin0[0], appliedForce, linX);
			linY = V4MulAdd(c->lin0[1], appliedForce, linY);
			linZ = V4MulAdd(c->lin0[2], appliedForce, linZ);

			angX = V4MulAdd(c->ang0[0], appliedForce, angX);
			angY = V4MulAdd(c->ang0[1], appliedForce, angY);
			angZ = V4MulAdd(c->ang0[2], appliedForce, angZ);

			base += stride;
		}

		//We need to do the cross product now

		angX = V4Sub(angX, V4NegMulSub(header->body0WorkOffset[0], linY, V4Mul(header->body0WorkOffset[1], linZ)));
		angY = V4Sub(angY, V4NegMulSub(header->body0WorkOffset[1], linZ, V4Mul(header->body0WorkOffset[2], linX)));
		angZ = V4Sub(angZ, V4NegMulSub(header->body0WorkOffset[2], linX, V4Mul(header->body0WorkOffset[0], linY)));

		const Vec4V linLenSq = V4MulAdd(linZ, linZ, V4MulAdd(linY, linY, V4Mul(linX, linX)));
		const Vec4V angLenSq = V4MulAdd(angZ, angZ, V4MulAdd(angY, angY, V4Mul(angX, angX)));

		const Vec4V linLen = V4Sqrt(linLenSq);
		const Vec4V angLen = V4Sqrt(angLenSq);

		const BoolV broken = BOr(V4IsGrtr(linLen, header->linBreakImpulse), V4IsGrtr(angLen, header->angBreakImpulse));

		PX_ALIGN(16, PxU32 iBroken[4]);
		BStoreA(broken, iBroken);

		PX_ALIGN(16, PxReal residual4[4]);
		V4StoreA(residual, residual4);

		PX_ALIGN(16, PxReal residual4PosIter[4]);
		V4StoreA(residualPosIter, residual4PosIter);

		Vec4V lin0, lin1, lin2, lin3;
		Vec4V ang0, ang1, ang2, ang3;

		PX_TRANSPOSE_34_44(linX, linY, linZ, lin0, lin1, lin2, lin3);
		PX_TRANSPOSE_34_44(angX, angY, angZ, ang0, ang1, ang2, ang3);
		
		if (writeback0)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin0), writeback0->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang0), writeback0->angularImpulse);
			writeback0->setCombined(header->breakable[0] ? PxU32(iBroken[0] != 0) : 0, residual4PosIter[0]);
			writeback0->residual = residual4[0];
		}
		if (writeback1)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin1), writeback1->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang1), writeback1->angularImpulse);
			writeback1->setCombined(header->breakable[1] ? PxU32(iBroken[1] != 0) : 0, residual4PosIter[1]);
			writeback1->residual = residual4[1];
		}
		if (writeback2)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin2), writeback2->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang2), writeback2->angularImpulse);
			writeback2->setCombined(header->breakable[2] ? PxU32(iBroken[2] != 0) : 0, residual4PosIter[2]);
			writeback2->residual = residual4[2];
		}
		if (writeback3)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin3), writeback3->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang3), writeback3->angularImpulse);
			writeback3->setCombined(header->breakable[3] ? PxU32(iBroken[3] != 0) : 0, residual4PosIter[3]);
			writeback3->residual = residual4[3];
		}
	}
}

static void concludeContact4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc)
{
	PX_UNUSED(desc);
	//const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	////hopefully pointer aliasing doesn't bite.
	//PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	//const Vec4V zero = V4Zero();

	////const PxU8 type = *desc[0].constraint;
	//const PxU32 contactSize = sizeof(SolverContactPointStepBlock);
	//const PxU32 frictionSize = sizeof(SolverContactFrictionStepBlock);

	//while ((currPtr < last))
	//{
	//	SolverContactHeaderStepBlock* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStepBlock*>(currPtr);

	//	currPtr = reinterpret_cast<PxU8*>(hdr + 1);

	//	const PxU32 numNormalConstr = hdr->numNormalConstr;
	//	const PxU32	numFrictionConstr = hdr->numFrictionConstr;

	//	//Applied forces
	//	currPtr += sizeof(Vec4V)*numNormalConstr;

	//	//SolverContactPointStepBlock* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStepBlock*>(currPtr);
	//	currPtr += (numNormalConstr * contactSize);

	//	bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

	//	if (hasMaxImpulse)
	//		currPtr += sizeof(Vec4V) * numNormalConstr;

	//	currPtr += sizeof(Vec4V)*numFrictionConstr;

	//	SolverContactFrictionStepBlock* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStepBlock*>(currPtr);
	//	currPtr += (numFrictionConstr * frictionSize);

	//	/*for (PxU32 i = 0; i<numNormalConstr; i++)
	//	{
	//		contacts[i].biasCoefficient = V4Sel(V4IsGrtr(contacts[i].separation, zero), contacts[i].biasCoefficient, zero);
	//	}*/

	//	for (PxU32 i = 0; i<numFrictionConstr; i++)
	//	{
	//		frictions[i].biasCoefficient = zero;
	//	}
	//}
}

static void conclude1DStep4(const PxSolverConstraintDesc* PX_RESTRICT desc, bool isResidualReportingEnabled)
{
	PxU8* PX_RESTRICT bPtr = desc->constraint;
	if (bPtr == NULL)
		return;

	const SolverConstraint1DHeaderStep4* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep4*>(bPtr);
	PxU8* PX_RESTRICT base = reinterpret_cast<PxU8*>(bPtr + sizeof(SolverConstraint1DHeaderStep4));
	PxU32 stride = isResidualReportingEnabled ? sizeof(SolverConstraint1DStep4WithResidual) : sizeof(SolverConstraint1DStep4);

	const VecI32V keepBiasMask = I4Load(DY_SC_FLAG_KEEP_BIAS);
	const VecI32V isSpringMask = I4Load(DY_SC_FLAG_SPRING); 
	const Vec4V zero = V4Zero();

	const PxU32 count = header->count;
	for (PxU32 i = 0; i<count; ++i/*, base++*/)
	{
		PxPrefetchLine(base + 1);
		SolverConstraint1DStep4& c = reinterpret_cast<SolverConstraint1DStep4&>(*base);

		const VecI32V flags = I4LoadA(reinterpret_cast<PxI32*>(c.flags));

		const BoolV keepBias = VecI32V_IsEq(VecI32V_And(flags, keepBiasMask), keepBiasMask);
		c.biasScale = V4Sel(keepBias, c.biasScale, zero);
		c.error = V4Sel(keepBias, c.error, zero);

		const BoolV isSpring = VecI32V_IsEq(VecI32V_And(flags, isSpringMask), isSpringMask);
		c.biasScale = V4Sel(isSpring, zero, c.biasScale);
		c.error = V4Sel(isSpring, zero, c.error);
		c.velMultiplier = V4Sel(isSpring, zero, c.velMultiplier);
		c.velTarget = V4Sel(isSpring, zero, c.velTarget);

		base += stride;
	}
}

void solveConcludeContact4(DY_TGS_CONCLUDE_METHOD_PARAMS)
{
	PX_UNUSED(txInertias);
	//PX_UNUSED(cache);

	solveContact4_Block(desc + hdr.startIndex, true, -PX_MAX_F32, elapsedTime, cache);
	concludeContact4_Block(desc + hdr.startIndex);
}

void solveConclude1D4(DY_TGS_CONCLUDE_METHOD_PARAMS)
{
	PX_UNUSED(cache);

	solve1DStep4(desc + hdr.startIndex, txInertias, elapsedTime, cache);
	conclude1DStep4(desc + hdr.startIndex, cache.contactErrorAccumulator != NULL);
}

}
}

