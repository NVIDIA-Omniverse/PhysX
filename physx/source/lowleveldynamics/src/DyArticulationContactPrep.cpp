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
#include "DyCorrelationBuffer.h"
#include "DySolverConstraintExtShared.h"
#include "DyCpuGpuArticulation.h"
#include "DyFeatherstoneArticulation.h"

using namespace physx::Gu;

namespace physx
{

namespace Dy
{

// constraint-gen only, since these use getVelocity methods
// which aren't valid during the solver phase

//PX_INLINE void computeFrictionTangents(const aos::Vec3V& vrel,const aos::Vec3V& unitNormal, aos::Vec3V& t0, aos::Vec3V& t1)
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

		const FloatV fv = velocity.dot(Cm::SpatialVector(linear, angular));

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

aos::FloatV SolverExtBody::projectVelocity(const aos::Vec3V& linear, const aos::Vec3V& angular) const
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


PxVec3 SolverExtBody::getLinVel() const
{
	if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		return mBodyData->linearVelocity;
	else
	{
		const Vec3V linear = mArticulation->getLinkVelocity(mLinkIndex).linear;

		PxVec3 result;
		V3StoreU(linear, result);
		/*PxVec3 result;
		V3StoreU(getVelocity(*mFsData)[mLinkIndex].linear, result);*/
		return result;
	}
}

PxVec3 SolverExtBody::getAngVel() const
{
	if(mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		return mBodyData->angularVelocity;
	else
	{
		const Vec3V angular = mArticulation->getLinkVelocity(mLinkIndex).angular;
		
		PxVec3 result;
		V3StoreU(angular, result);
		/*PxVec3 result;
		V3StoreU(getVelocity(*mFsData)[mLinkIndex].angular, result);*/
		return result;
	}
}

aos::Vec3V SolverExtBody::getLinVelV() const
{
	if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		return V3LoadA(mBodyData->linearVelocity);
	else
	{
		return mArticulation->getLinkVelocity(mLinkIndex).linear;
	}
}


Vec3V SolverExtBody::getAngVelV() const
{
	if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		return V3LoadA(mBodyData->angularVelocity);
	else
	{

		return mArticulation->getLinkVelocity(mLinkIndex).angular;
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
	if (body.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
	{
		return Cm::SpatialVector(linear, body.mBodyData->sqrtInvInertia * angular);
	}
	return Cm::SpatialVector(linear, angular);
}

Cm::SpatialVectorV createImpulseResponseVector(const aos::Vec3V& linear, const aos::Vec3V& angular, const SolverExtBody& body)
{
	if (body.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
	{
		return Cm::SpatialVectorV(linear, M33MulV3(M33Load(body.mBodyData->sqrtInvInertia), angular));
	}
	return Cm::SpatialVectorV(linear, angular);
}

PxReal getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0, 
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

FloatV getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVectorV& impulse0, Cm::SpatialVectorV& deltaV0, const FloatV& dom0, const FloatV& angDom0,
	const SolverExtBody& b1, const Cm::SpatialVectorV& impulse1, Cm::SpatialVectorV& deltaV1, const FloatV& dom1, const FloatV& angDom1,
	bool /*allowSelfCollision*/)
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



void setupFinalizeExtSolverContacts(
	const PxContactPoint* buffer,
	const CorrelationBuffer& c,
	const PxTransform& bodyFrame0,
	const PxTransform& bodyFrame1,
	PxU8* workspace,
	const SolverExtBody& b0,
	const SolverExtBody& b1,
	const PxReal invDtF32,
	const PxReal dtF32,
	PxReal bounceThresholdF32,
	PxReal invMassScale0, PxReal invInertiaScale0,
	PxReal invMassScale1, PxReal invInertiaScale1,
	const PxReal restDist,
	PxU8* frictionDataPtr,
	PxReal ccdMaxContactDist,
	Cm::SpatialVectorF* Z,
	const PxReal offsetSlop)
{
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	/*const bool haveFriction = PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;*/

	const FloatV ccdMaxSeparation = FLoad(ccdMaxContactDist);

	const Vec3VArg solverOffsetSlop = V3Load(offsetSlop);

	PxU8* PX_RESTRICT ptr = workspace;

	const FloatV zero = FZero();

	//KS - TODO - this should all be done in SIMD to avoid LHS
	//const PxF32 maxPenBias0 = b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b0.mBodyData->penBiasClamp : getMaxPenBias(*b0.mFsData)[b0.mLinkIndex];
	//const PxF32 maxPenBias1 = b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b1.mBodyData->penBiasClamp : getMaxPenBias(*b1.mFsData)[b1.mLinkIndex];

	PxF32 maxPenBias0;
	PxF32 maxPenBias1;

	if (b0.mLinkIndex != PxSolverConstraintDesc::RIGID_BODY)
	{
		maxPenBias0 = b0.mArticulation->getLinkMaxPenBias(b0.mLinkIndex);
	}
	else
		maxPenBias0 = b0.mBodyData->penBiasClamp;

	if (b1.mLinkIndex != PxSolverConstraintDesc::RIGID_BODY)
	{
		maxPenBias1 = b1.mArticulation->getLinkMaxPenBias(b1.mLinkIndex);
	}
	else
		maxPenBias1 = b1.mBodyData->penBiasClamp;

	const FloatV maxPenBias = FLoad(PxMax(maxPenBias0, maxPenBias1));

	const Vec3V frame0p = V3LoadU(bodyFrame0.p);
	const Vec3V frame1p = V3LoadU(bodyFrame1.p);

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

	const FloatV restDistance = FLoad(restDist); 

	PxU32 frictionPatchWritebackAddrIndex = 0;

	PxPrefetchLine(c.contactID);
	PxPrefetchLine(c.contactID, 128);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV p8 = FLoad(0.8f);
	const FloatV bounceThreshold = FLoad(bounceThresholdF32);

	const FloatV invDtp8 = FMul(invDt, p8);
	const FloatV dt = FLoad(dtF32);

	PxU8 flags = 0;

	for(PxU32 i=0;i<c.frictionPatchCount;i++)
	{
		PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const FrictionPatch& frictionPatch = c.frictionPatches[i];
		PX_ASSERT(frictionPatch.anchorCount <= 2);  //0==anchorCount is allowed if all the contacts in the manifold have a large offset. 

		const PxContactPoint* contactBase0 = buffer + c.contactPatches[c.correlationListHeads[i]].start;

		const PxReal coefficient = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = contactBase0->staticFriction * coefficient;
		const PxReal dynamicFriction = contactBase0->dynamicFriction * coefficient;
		const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));
		const BoolV accelerationSpring = BLoad(!!(contactBase0->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING));
	
		SolverContactHeader* PX_RESTRICT header = reinterpret_cast<SolverContactHeader*>(ptr);
		ptr += sizeof(SolverContactHeader);		

		PxPrefetchLine(ptr + 128);
		PxPrefetchLine(ptr + 256);
		PxPrefetchLine(ptr + 384);
		
		const bool haveFriction = (disableStrongFriction == 0) ;//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		header->numNormalConstr		= PxTo8(contactCount);
		header->numFrictionConstr	= PxTo8(haveFriction ? frictionPatch.anchorCount*2 : 0);
	
		header->type				= PxTo8(DY_SC_TYPE_EXT_CONTACT);

		header->flags = flags;

		const FloatV restitution = FLoad(contactBase0->restitution);
		const FloatV damping = FLoad(contactBase0->damping);
	
		header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;

		header->angDom0 = invInertiaScale0;
		header->angDom1 = invInertiaScale1;
	
		const PxU32 pointStride = sizeof(SolverContactPointExt);
		const PxU32 frictionStride = sizeof(SolverContactFrictionExt);

		const Vec3V normal = V3LoadU(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
		
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

				accumImpulse = FAdd(accumImpulse, setupExtSolverContact(b0, b1, d0, d1, angD0, angD1, frame0p, frame1p, normal, invDt, invDtp8, dt, restDistance, maxPenBias, restitution,
					bounceThreshold, contact, *solverContact, ccdMaxSeparation, Z, vel0, vel1, cfm, solverOffsetSlop, norVel0, norVel1, damping, accelerationSpring));
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

			for(PxU32 j = 0; j < frictionPatch.anchorCount; j++)
			{
				SolverContactFrictionExt* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionExt*>(ptr);
				ptr += frictionStride;
				SolverContactFrictionExt* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionExt*>(ptr);
				ptr += frictionStride;

				Vec3V ra = V3LoadU(bodyFrame0.q.rotate(frictionPatch.body0Anchors[j]));
				Vec3V rb = V3LoadU(bodyFrame1.q.rotate(frictionPatch.body1Anchors[j]));
				Vec3V error = V3Sub(V3Add(ra, frame0p), V3Add(rb, frame1p));

				{
					Vec3V raXn = V3Cross(ra, t0Cross);
					Vec3V rbXn = V3Cross(rb, t0Cross);
					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					Cm::SpatialVectorV deltaV0, deltaV1;

					const Cm::SpatialVectorV resp0 = createImpulseResponseVector(t0, raXn, b0);
					const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(t0), V3Neg(rbXn), b1);
					FloatV resp = FAdd(cfm, getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
															 b1, resp1, deltaV1, d1, angD1, reinterpret_cast<Cm::SpatialVectorV*>(Z)));

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FDiv(p8, resp), zero);

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

					FloatV resp = FAdd(cfm, getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
														   b1, resp1, deltaV1, d1, angD1, reinterpret_cast<Cm::SpatialVectorV*>(Z)));

					//const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FMul(p8, FRecip(resp)), zero);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FDiv(p8, resp), zero);

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
