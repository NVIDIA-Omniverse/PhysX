// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "DyFeatherstoneArticulation.h"
#include "DyContactPrepShared.h"

namespace physx
{
namespace Dy
{

// constraint-gen only, since these use getVelocity methods
// which aren't valid during the solver phase

//PX_INLINE void computeFrictionTangents(const Vec3V& vrel,const Vec3V& unitNormal, Vec3V& t0, Vec3V& t1)
//{
//	using namespace aos;
//	//PX_ASSERT(PxAbs(unitNormal.magnitude()-1)<1e-3f);
//
//	t0 = V3Sub(vrel, V3Scale(unitNormal, V3Dot(unitNormal, vrel)));
//	const FloatV ll = V3Dot(t0, t0);
//
//	if (FAllGrtr(ll, FLoad(1e10f)))										//can set as low as 0.
//	{
//		t0 = V3Scale(t0, FRsqrt(ll));
//		t1 = V3Cross(unitNormal, t0);
//	}
//	else
//		PxNormalToTangents(unitNormal, t0, t1);		//fallback
//}

PxReal SolverExtBody::projectVelocity(const PxVec3& linear, const PxVec3& angular) const
{
	if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
	{
		return mBodyData->projectVelocity(linear, angular);
	}
	else
	{
		const Cm::SpatialVectorV velocity = mArticulation->getLinkVelocity(mLinkIndex);

		const FloatV fv = velocity.dot(Cm::SpatialVectorV(Cm::SpatialVector(linear, angular)));

		PxF32 f;
		FStore(fv, &f);
		return f;
		/*PxF32 f;
		FStore(getVelocity(*mFsData)[mLinkIndex].dot(Cm::SpatialVector(linear, angular)), &f);
		return f;*/
	}
}

PxReal SolverExtBody::getCFM() const
{
	return (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY) ? 0.f : 
		mArticulation->getCfm(mLinkIndex);
}

FloatV SolverExtBody::projectVelocity(const Vec3V& linear, const Vec3V& angular) const
{
	if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
	{
		return V3SumElems(V3Add(V3Mul(V3LoadA(mBodyData->linearVelocity), linear), V3Mul(V3LoadA(mBodyData->angularVelocity), angular)));
	}
	else
	{
		const Cm::SpatialVectorV velocity = mArticulation->getLinkVelocity(mLinkIndex);

		return velocity.dot(Cm::SpatialVectorV(linear, angular));
	}
}

Cm::SpatialVectorV SolverExtBody::getVelocity() const
{
	if(mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		return Cm::SpatialVectorV(V3LoadA(mBodyData->linearVelocity), V3LoadA(mBodyData->angularVelocity));
	else
		return mArticulation->getLinkVelocity(mLinkIndex);
}

Cm::SpatialVector createImpulseResponseVector(const PxVec3& linear, const PxVec3& angular, const SolverExtBody& body)
{
	// Note: compared to the TGS version of this method, there seems to be no need to check for kinematic or static
	//       bodies in PGS since the original angular constraint axis is tracked separately (ang0Writeback) and a zero
	//       inverse inertia matrix will thus not destroy that information (important for reporting applied
	//       joint/constraint torques, for example).

	if (body.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
	{
		return Cm::SpatialVector(linear, body.mBodyData->sqrtInvInertia * angular);
	}
	return Cm::SpatialVector(linear, angular);
}

Cm::SpatialVectorV createImpulseResponseVector(const Vec3V& linear, const Vec3V& angular, const SolverExtBody& body)
{
	if (body.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
	{
		return Cm::SpatialVectorV(linear, M33MulV3(M33Load(body.mBodyData->sqrtInvInertia), angular));
	}
	return Cm::SpatialVectorV(linear, angular);
}

PxReal getImpulseResponse(	const SolverExtBody& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0, 
							const SolverExtBody& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1, 
							bool allowSelfCollision)
{
	PxReal response;
	allowSelfCollision = false;
	// right now self-collision with contacts crashes the solver
	
	//KS - knocked this out to save some space on SPU
	if(allowSelfCollision && b0.mLinkIndex!=PxSolverConstraintDesc::RIGID_BODY && b0.mArticulation == b1.mArticulation && 0)
	{
		/*ArticulationHelper::getImpulseSelfResponse(*b0.mFsData,b0.mLinkIndex, impulse0, deltaV0, 
													  b1.mLinkIndex, impulse1, deltaV1);*/

		b0.mArticulation->getImpulseSelfResponse(b0.mLinkIndex, b1.mLinkIndex, impulse0.scale(dom0, angDom0), impulse1.scale(dom1, angDom1), 
			deltaV0, deltaV1);
		
		response = impulse0.dot(deltaV0) + impulse1.dot(deltaV1);
	}
	else 
	{
		if(b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		{
			deltaV0.linear = impulse0.linear * b0.mBodyData->invMass * dom0;
			deltaV0.angular = impulse0.angular * angDom0;
		}
		else
		{
			b0.mArticulation->getImpulseResponse(b0.mLinkIndex, impulse0.scale(dom0, angDom0), deltaV0);
		}

		response = impulse0.dot(deltaV0);
		if(b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		{
			deltaV1.linear = impulse1.linear * b1.mBodyData->invMass * dom1;
			deltaV1.angular = impulse1.angular * angDom1;
		}
		else
		{
			b1.mArticulation->getImpulseResponse( b1.mLinkIndex, impulse1.scale(dom1, angDom1), deltaV1);
		}
		response += impulse1.dot(deltaV1);
	}

	return response;
}

static FloatV getImpulseResponse_(	const SolverExtBody& b0, const Cm::SpatialVectorV& impulse0, Cm::SpatialVectorV& deltaV0, const FloatV& dom0, const FloatV& angDom0,
									const SolverExtBody& b1, const Cm::SpatialVectorV& impulse1, Cm::SpatialVectorV& deltaV1, const FloatV& dom1, const FloatV& angDom1)
{
	Vec3V response;
	{
		if (b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		{
			deltaV0.linear = V3Scale(impulse0.linear, FMul(FLoad(b0.mBodyData->invMass), dom0));
			deltaV0.angular = V3Scale(impulse0.angular, angDom0);
		}
		else
		{
			b0.mArticulation->getImpulseResponse(b0.mLinkIndex, impulse0.scale(dom0, angDom0), deltaV0);
		}

		response = V3Add(V3Mul(impulse0.linear, deltaV0.linear), V3Mul(impulse0.angular, deltaV0.angular));
		if (b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		{
			deltaV1.linear = V3Scale(impulse1.linear, FMul(FLoad(b1.mBodyData->invMass), dom1));
			deltaV1.angular = V3Scale(impulse1.angular, angDom1);
		}
		else
		{
			b1.mArticulation->getImpulseResponse(b1.mLinkIndex, impulse1.scale(dom1, angDom1), deltaV1);
		}
		response = V3Add(response, V3Add(V3Mul(impulse1.linear, deltaV1.linear), V3Mul(impulse1.angular, deltaV1.angular)));
	}

	return V3SumElems(response);
}

struct ExtSolverContactParams
{
	PxReal dtF32;
	PxReal invDtF32;
	PxReal bounceThresholdF32;
};

static FloatV setupExtSolverContact(
	const ExtSolverContactParams& params,
	const PxSolverContactDesc& contactDesc,
	const SolverExtBody& b0,
	const SolverExtBody& b1,
	const Vec3V& bodyFrame0p,
	const Vec3V& bodyFrame1p,
	const Vec3VArg normal,
	const FloatVArg invDtWithBiasCoefficient,
	const FloatVArg maxPenBias,
	const PxContactPoint* contactBase0,
	const PxContactPoint& contact,
	SolverContactPointExt& solverContact,
	const Cm::SpatialVectorV& v0,
	const Cm::SpatialVectorV& v1,
	const FloatV& cfm,
	const FloatVArg norVel0,
	const FloatVArg norVel1)
{
	const FloatV zero = FZero();

	const FloatV penetration = FLoad(contact.separation - contactDesc.restDistance);

	const Vec3V point = V3LoadA(contact.point);

	const Vec3V ra = V3Sub(point, bodyFrame0p);
	const Vec3V rb = V3Sub(point, bodyFrame1p);

	Vec3V raXn = V3Cross(ra, normal);
	Vec3V rbXn = V3Cross(rb, normal);

	FloatV aVel0 = V3Dot(v0.angular, raXn);
	FloatV aVel1 = V3Dot(v1.angular, raXn);

	FloatV relLinVel = FSub(norVel0, norVel1);
	FloatV relAngVel = FSub(aVel0, aVel1);
	
	const Vec3V slop = V3Scale(V3Load(contactDesc.offsetSlop), FMax(FSel(FIsEq(relLinVel, zero), FMax(), FDiv(relAngVel, relLinVel)), FOne()));

	raXn = V3Sel(V3IsGrtr(slop, V3Abs(raXn)), V3Zero(), raXn);
	rbXn = V3Sel(V3IsGrtr(slop, V3Abs(rbXn)), V3Zero(), rbXn);

	aVel0 = V3Dot(raXn, v0.angular);
	aVel1 = V3Dot(rbXn, v1.angular);

	relAngVel = FSub(aVel0, aVel1);

	Cm::SpatialVectorV deltaV0, deltaV1;

	const Cm::SpatialVectorV resp0 = createImpulseResponseVector(normal, raXn, b0);
	const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(normal), V3Neg(rbXn), b1);

	const FloatV d0 = FLoad(contactDesc.invMassScales.linear0);
	const FloatV d1 = FLoad(contactDesc.invMassScales.linear1);
	const FloatV angD0 = FLoad(contactDesc.invMassScales.angular0);
	const FloatV angD1 = FLoad(contactDesc.invMassScales.angular1);

	const FloatV unitResponse = getImpulseResponse_(b0, resp0, deltaV0, d0, angD0,
													b1, resp1, deltaV1, d1, angD1);

	const FloatV vrel = FAdd(relLinVel, relAngVel);

	const FloatV invDt = FLoad(params.invDtF32);

	const FloatV penetrationInvDt = FMul(penetration, invDt);
	const BoolV isSeparated = FIsGrtrOrEq(penetration, zero);

	const BoolV collidingWithVrel = FIsGrtr(FNeg(vrel), penetrationInvDt); // true if (pen + dt*vrel) < 0
	const FloatV bounceThreshold = FLoad(params.bounceThresholdF32);

	const FloatV restitution = FLoad(contactBase0->restitution);
	const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), collidingWithVrel);

	const FloatV tVel = FSel(isGreater2, FMul(FNeg(vrel), restitution), zero);
	FloatV targetVelocity = tVel;
	//Get the rigid body's current velocity and embed into the constraint target velocities
	if (b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		targetVelocity = FSub(targetVelocity, FAdd(norVel0, aVel0));
	else if (b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		targetVelocity = FAdd(targetVelocity, FAdd(norVel1, aVel1));

	targetVelocity = FAdd(targetVelocity, V3Dot(V3LoadA(contact.targetVel), normal));

	// jcarius: the addition of the cfm term is not present in equivalent code for rigid bodies
	const FloatV recipResponse = FSel(FIsGrtr(unitResponse, zero), FRecip(FAdd(unitResponse, cfm)), zero);

	FloatV velMultiplier, impulseMultiplier;
	FloatV biasedErr, unbiasedErr;

	if (FAllGrtr(zero, restitution))
	{
		const FloatV dt = FLoad(params.dtF32);
		const FloatV damping = FLoad(contactBase0->damping);
		const BoolV accelerationSpring = BLoad(!!(contactBase0->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING));
		computeCompliantContactCoefficients(dt, restitution, damping, recipResponse, unitResponse, penetration,
		                                    targetVelocity, accelerationSpring, isSeparated, collidingWithVrel,
		                                    velMultiplier, impulseMultiplier, unbiasedErr, biasedErr);
	}
	else
	{
		const BoolV ccdSeparationCondition = FIsGrtrOrEq(FLoad(contactDesc.maxCCDSeparation), penetration);
		velMultiplier = recipResponse;
		const FloatV penetrationInvDtScaled = FSel(isSeparated, penetrationInvDt, FMul(penetration, invDtWithBiasCoefficient));
		FloatV scaledBias = FMul(velMultiplier, FMax(maxPenBias, penetrationInvDtScaled));
		scaledBias = FSel(BAnd(ccdSeparationCondition, isGreater2), zero, scaledBias);

		biasedErr = FScaleAdd(targetVelocity, velMultiplier, FNeg(scaledBias));
		unbiasedErr = FScaleAdd(targetVelocity, velMultiplier, FSel(isGreater2, zero, FNeg(FMax(scaledBias, zero))));
		impulseMultiplier = FOne();
	}

	const FloatV deltaF = FMax(FMul(FSub(tVel, FAdd(vrel, FMax(penetrationInvDt, zero))), velMultiplier), zero);

	FStore(biasedErr, &solverContact.biasedErr);
	FStore(unbiasedErr, &solverContact.unbiasedErr);

	solverContact.raXn_velMultiplierW = V4SetW(Vec4V_From_Vec3V(resp0.angular), velMultiplier);
	solverContact.rbXn_maxImpulseW = V4SetW(Vec4V_From_Vec3V(V3Neg(resp1.angular)), FLoad(contact.maxImpulse));
	solverContact.linDeltaVA = deltaV0.linear;
	solverContact.angDeltaVA = deltaV0.angular;
	solverContact.linDeltaVB = deltaV1.linear;
	solverContact.angDeltaVB = deltaV1.angular;
	FStore(impulseMultiplier, &solverContact.impulseMultiplier);

	return deltaF;
}

void setupFinalizeExtSolverContacts(
	const PxSolverContactDesc& contactDesc,
	const CorrelationBuffer& c,
	PxU8* workspace,
	const SolverExtBody& b0,
	const SolverExtBody& b1,
	PxReal invDtF32,
	PxReal dtF32,
	PxReal bounceThresholdF32,
	PxReal biasCoefficient,
	PxU8* frictionDataPtr)
{
	const PxContactPoint* buffer = contactDesc.contacts;
	const PxTransform& bodyFrame0 = contactDesc.bodyFrame0;
	const PxTransform& bodyFrame1 = contactDesc.bodyFrame1;
	const PxReal invMassScale0 = contactDesc.invMassScales.linear0;
	const PxReal invInertiaScale0 = contactDesc.invMassScales.angular0;
	const PxReal invMassScale1 = contactDesc.invMassScales.linear1;
	const PxReal invInertiaScale1 = contactDesc.invMassScales.angular1;

	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	/*const bool haveFriction = PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;*/

	PxU8* PX_RESTRICT ptr = workspace;

	const FloatV zero = FZero();

	//KS - TODO - this should all be done in SIMD to avoid LHS
	//const PxF32 maxPenBias0 = b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b0.mBodyData->penBiasClamp : getMaxPenBias(*b0.mFsData)[b0.mLinkIndex];
	//const PxF32 maxPenBias1 = b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b1.mBodyData->penBiasClamp : getMaxPenBias(*b1.mFsData)[b1.mLinkIndex];

	PxF32 maxPenBias0;
	PxF32 maxPenBias1;

	if (b0.mLinkIndex != PxSolverConstraintDesc::RIGID_BODY)
		maxPenBias0 = b0.mArticulation->getLinkMaxPenBias(b0.mLinkIndex);
	else
		maxPenBias0 = b0.mBodyData->penBiasClamp;

	if (b1.mLinkIndex != PxSolverConstraintDesc::RIGID_BODY)
		maxPenBias1 = b1.mArticulation->getLinkMaxPenBias(b1.mLinkIndex);
	else
		maxPenBias1 = b1.mBodyData->penBiasClamp;

	const FloatV maxPenBias = FLoad(PxMax(maxPenBias0, maxPenBias1));

	const Vec3V frame0p = V3LoadU_SafeReadW(bodyFrame0.p);
	const Vec3V frame1p = V3LoadU_SafeReadW(bodyFrame1.p);

	const Cm::SpatialVectorV vel0 = b0.getVelocity();
	const Cm::SpatialVectorV vel1 = b1.getVelocity();

	const FloatV quarter = FLoad(0.25f);

	const FloatV d0 = FLoad(invMassScale0);
	const FloatV d1 = FLoad(invMassScale1);

	const FloatV cfm = FLoad(PxMax(b0.getCFM(), b1.getCFM()));

	const FloatV angD0 = FLoad(invInertiaScale0);
	const FloatV angD1 = FLoad(invInertiaScale1);

	Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, d0);
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, d1);

	PxU32 frictionPatchWritebackAddrIndex = 0;

	PxPrefetchLine(c.contactID);
	PxPrefetchLine(c.contactID, 128);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV biasCoefficientV = FLoad(biasCoefficient);

	const FloatV invDtWithBiasCoefficient = FMul(invDt, biasCoefficientV);

	PxU8 flags = 0;

	// PT: push these to the stack once, not once per call to setupExtSolverContact
	ExtSolverContactParams params;
	params.dtF32 = dtF32;
	params.invDtF32 = invDtF32;
	params.bounceThresholdF32 = bounceThresholdF32;

	for(PxU32 i=0;i<c.frictionPatchCount;i++)
	{
		PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const FrictionPatch& frictionPatch = c.frictionPatches[i];
		PX_ASSERT(frictionPatch.anchorCount <= 2);  //0==anchorCount is allowed if all the contacts in the manifold have a large offset. 

		const PxContactPoint* contactBase0 = buffer + c.contactPatches[c.correlationListHeads[i]].start;

		const PxReal coefficient = (frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = contactBase0->staticFriction * coefficient;
		const PxReal dynamicFriction = contactBase0->dynamicFriction * coefficient;
		const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));
	
		SolverContactHeader* PX_RESTRICT header = reinterpret_cast<SolverContactHeader*>(ptr);
		ptr += sizeof(SolverContactHeader);		

		PxPrefetchLine(ptr + 128);
		PxPrefetchLine(ptr + 256);
		PxPrefetchLine(ptr + 384);
		
		const bool haveFriction = (disableStrongFriction == 0) ;//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		header->numNormalConstr		= PxTo8(contactCount);
		header->numFrictionConstr	= PxTo8(haveFriction ? frictionPatch.anchorCount*2 : 0);
	
		header->type	= PxTo8(DY_SC_TYPE_EXT_CONTACT);
		header->flags	= flags;

		header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;

		header->angDom0 = invInertiaScale0;
		header->angDom1 = invInertiaScale1;
	
		const PxU32 pointStride = sizeof(SolverContactPointExt);
		const PxU32 frictionStride = sizeof(SolverContactFrictionExt);

		const Vec3V normal = V3LoadA(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
		
		FloatV accumImpulse = FZero();

		const FloatV norVel0 = V3Dot(normal, vel0.linear);
		const FloatV norVel1 = V3Dot(normal, vel1.linear);

		for(PxU32 patch=c.correlationListHeads[i]; 
			patch!=CorrelationBuffer::LIST_END; 
			patch = c.contactPatches[patch].next)
		{
			const PxU32 count = c.contactPatches[patch].count;
			const PxContactPoint* contactBase = buffer + c.contactPatches[patch].start;
				
			PxU8* p = ptr;

			for(PxU32 j=0;j<count;j++)
			{
				const PxContactPoint& contact = contactBase[j];

				SolverContactPointExt* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointExt*>(p);
				p += pointStride;

				accumImpulse = FAdd(accumImpulse, setupExtSolverContact(params, contactDesc, b0, b1, frame0p, frame1p, normal, invDtWithBiasCoefficient, maxPenBias,
					contactBase0, contact, *solverContact, vel0, vel1, cfm, norVel0, norVel1));
			}

			ptr = p;
		}

		accumImpulse = FMul(FDiv(accumImpulse, FLoad(PxF32(contactCount))), quarter);

		header->normal_minAppliedImpulseForFrictionW = V4SetW(Vec4V_From_Vec3V(normal), accumImpulse);

		PxF32* forceBuffer = reinterpret_cast<PxF32*>(ptr);
		PxMemZero(forceBuffer, sizeof(PxF32) * contactCount);
		ptr += sizeof(PxF32) * ((contactCount + 3) & (~3));

		header->broken = 0;

		if(haveFriction)
		{
			//const Vec3V normal = Vec3V_From_PxVec3(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);
			//Vec3V normalS = V3LoadA(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);

			const Vec3V linVrel = V3Sub(vel0.linear, vel1.linear);
			//const Vec3V normal = Vec3V_From_PxVec3_Aligned(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

			const FloatV orthoThreshold = FLoad(0.70710678f);
			const FloatV p1 = FLoad(0.0001f);
			// fallback: normal.cross((1,0,0)) or normal.cross((0,0,1))
			const FloatV normalX = V3GetX(normal);
			const FloatV normalY = V3GetY(normal);
			const FloatV normalZ = V3GetZ(normal);

			Vec3V t0Fallback1 = V3Merge(zero, FNeg(normalZ), normalY);
			Vec3V t0Fallback2 = V3Merge(FNeg(normalY), normalX, zero);
			Vec3V t0Fallback = V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			Vec3V t0 = V3Sub(linVrel, V3Scale(normal, V3Dot(normal, linVrel)));
			t0 = V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0 = V3Normalize(t0);

			const VecCrossV t0Cross = V3PrepareCross(t0);

			const Vec3V t1 = V3Cross(normal, t0Cross);
			const VecCrossV t1Cross = V3PrepareCross(t1);
			
			//We want to set the writeBack ptr to point to the broken flag of the friction patch.
			//On spu we have a slight problem here because the friction patch array is 
			//in local store rather than in main memory. The good news is that the address of the friction 
			//patch array in main memory is stored in the work unit. These two addresses will be equal 
			//except on spu where one is local store memory and the other is the effective address in main memory.
			//Using the value stored in the work unit guarantees that the main memory address is used on all platforms.
			PxU8* PX_RESTRICT writeback = frictionDataPtr + frictionPatchWritebackAddrIndex*sizeof(FrictionPatch);

			header->frictionBrokenWritebackByte = writeback;			

			const QuatV bodyFrame0q = QuatVLoadU(&bodyFrame0.q.x);
			const QuatV bodyFrame1q = QuatVLoadU(&bodyFrame1.q.x);

			for(PxU32 j = 0; j < frictionPatch.anchorCount; j++)
			{
				SolverContactFrictionExt* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionExt*>(ptr);
				ptr += frictionStride;
				SolverContactFrictionExt* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionExt*>(ptr);
				ptr += frictionStride;

				const Vec3V body0Anchor = V3LoadU_SafeReadW(frictionPatch.body0Anchors[j]);	// PT: see compile-time-assert in FrictionPatch
				const Vec3V body1Anchor = V3LoadU_SafeReadW(frictionPatch.body1Anchors[j]);	// PT: see compile-time-assert in FrictionPatch

				const Vec3V ra = QuatRotate(bodyFrame0q, body0Anchor);
				const Vec3V rb = QuatRotate(bodyFrame1q, body1Anchor);

				const Vec3V error = V3Sub(V3Add(ra, frame0p), V3Add(rb, frame1p));

				const Vec3VArg solverOffsetSlop = V3Load(contactDesc.offsetSlop);
				{
					Vec3V raXn = V3Cross(ra, t0Cross);
					Vec3V rbXn = V3Cross(rb, t0Cross);
					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					Cm::SpatialVectorV deltaV0, deltaV1;

					const Cm::SpatialVectorV resp0 = createImpulseResponseVector(t0, raXn, b0);
					const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(t0), V3Neg(rbXn), b1);
					FloatV resp = FAdd(cfm, getImpulseResponse_(b0, resp0, deltaV0, d0, angD0,
																b1, resp1, deltaV1, d1, angD1));

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FDiv(biasCoefficientV, resp), zero);

					const PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
					FloatV targetVel = V3Dot(V3LoadA(buffer[index].targetVel), t0);

					if(b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
						targetVel = FSub(targetVel, b0.projectVelocity(t0, raXn));
					else if(b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
						targetVel = FAdd(targetVel, b1.projectVelocity(t0, rbXn));

					f0->normalXYZ_appliedForceW = V4ClearW(Vec4V_From_Vec3V(t0));
					f0->raXnXYZ_velMultiplierW = V4SetW(Vec4V_From_Vec3V(resp0.angular), velMultiplier);
					f0->rbXnXYZ_biasW = V4SetW(V4Neg(Vec4V_From_Vec3V(resp1.angular)), FMul(V3Dot(t0, error), invDt));
					f0->linDeltaVA = deltaV0.linear;
					f0->angDeltaVA = deltaV0.angular;
					f0->linDeltaVB = deltaV1.linear;
					f0->angDeltaVB = deltaV1.angular;
					FStore(targetVel, &f0->targetVel);
				}

				{
					Vec3V raXn = V3Cross(ra, t1Cross);
					Vec3V rbXn = V3Cross(rb, t1Cross);
					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					Cm::SpatialVectorV deltaV0, deltaV1;

					const Cm::SpatialVectorV resp0 = createImpulseResponseVector(t1, raXn, b0);
					const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(t1), V3Neg(rbXn), b1);

					FloatV resp = FAdd(cfm, getImpulseResponse_(b0, resp0, deltaV0, d0, angD0,
																b1, resp1, deltaV1, d1, angD1));

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FDiv(biasCoefficientV, resp), zero);

					const PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
					FloatV targetVel = V3Dot(V3LoadA(buffer[index].targetVel), t1);

					if(b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
						targetVel = FSub(targetVel, b0.projectVelocity(t1, raXn));
					else if(b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
						targetVel = FAdd(targetVel, b1.projectVelocity(t1, rbXn));

					f1->normalXYZ_appliedForceW = V4ClearW(Vec4V_From_Vec3V(t1));
					f1->raXnXYZ_velMultiplierW = V4SetW(Vec4V_From_Vec3V(resp0.angular), velMultiplier);
					f1->rbXnXYZ_biasW = V4SetW(V4Neg(Vec4V_From_Vec3V(resp1.angular)), FMul(V3Dot(t1, error), invDt));
					f1->linDeltaVA = deltaV0.linear;
					f1->angDeltaVA = deltaV0.angular;
					f1->linDeltaVB = deltaV1.linear;
					f1->angDeltaVB = deltaV1.angular;
					FStore(targetVel, &f1->targetVel);
				}
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}

}


}
