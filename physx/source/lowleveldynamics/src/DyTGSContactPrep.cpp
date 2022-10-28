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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved. 

#include "foundation/PxPreprocessor.h"
#include "foundation/PxVecMath.h"
#include "PxcNpWorkUnit.h"
#include "DyThreadContext.h"
#include "PxcNpContactPrepShared.h"
#include "DyFeatherstoneArticulation.h"

using namespace physx;
using namespace Gu;

#include "PxsMaterialManager.h"
#include "DyContactPrepShared.h"
#include "DyConstraintPrep.h"

#include "DySolverContext.h"
#include "DySolverConstraint1DStep.h"

using namespace aos;

namespace physx
{
namespace Dy
{
	PX_FORCE_INLINE PxReal safeRecip(const PxReal x)
	{
		return x > PX_EPS_F32 ? 1.f/x : 0.f;
	}

	PX_FORCE_INLINE void computeBlockStreamByteSizesStep(const bool useExtContacts, const CorrelationBuffer& c,
		PxU32& _solverConstraintByteSize, PxU32& _frictionPatchByteSize, PxU32& _numFrictionPatches,
		PxU32& _axisConstraintCount, PxReal torsionalPatchRadius)
	{
		PX_ASSERT(0 == _solverConstraintByteSize);
		PX_ASSERT(0 == _frictionPatchByteSize);
		PX_ASSERT(0 == _numFrictionPatches);
		PX_ASSERT(0 == _axisConstraintCount);

		// PT: use local vars to remove LHS
		PxU32 solverConstraintByteSize = 0;
		PxU32 numFrictionPatches = 0;
		PxU32 axisConstraintCount = 0;


		for (PxU32 i = 0; i < c.frictionPatchCount; i++)
		{
			//Friction patches.
			if (c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
				numFrictionPatches++;

			const FrictionPatch& frictionPatch = c.frictionPatches[i];

			const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0;

			//Solver constraint data.
			if (c.frictionPatchContactCounts[i] != 0)
			{
				solverConstraintByteSize += sizeof(SolverContactHeaderStep);
				solverConstraintByteSize += useExtContacts ? c.frictionPatchContactCounts[i] * sizeof(SolverContactPointStepExt)
					: c.frictionPatchContactCounts[i] * sizeof(SolverContactPointStep);
				solverConstraintByteSize += sizeof(PxF32) * ((c.frictionPatchContactCounts[i] + 3)&(~3)); //Add on space for applied impulses

				axisConstraintCount += c.frictionPatchContactCounts[i];

				if (haveFriction)
				{
					PxU32 nbAnchors = PxU32(c.frictionPatches[i].anchorCount * 2);
					if (torsionalPatchRadius > 0.f && c.frictionPatches[i].anchorCount == 1)
						nbAnchors++;
					solverConstraintByteSize += useExtContacts ? nbAnchors * sizeof(SolverContactFrictionStepExt)
						: nbAnchors * sizeof(SolverContactFrictionStep);
					axisConstraintCount += nbAnchors;

				}
			}
		}
		PxU32 frictionPatchByteSize = numFrictionPatches*sizeof(FrictionPatch);

		_numFrictionPatches = numFrictionPatches;
		_axisConstraintCount = axisConstraintCount;

		//16-byte alignment.
		_frictionPatchByteSize = ((frictionPatchByteSize + 0x0f) & ~0x0f);
		_solverConstraintByteSize = ((solverConstraintByteSize + 0x0f) & ~0x0f);
		PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
		PX_ASSERT(0 == (_frictionPatchByteSize & 0x0f));
	}

	static bool reserveBlockStreams(const bool useExtContacts, Dy::CorrelationBuffer& cBuffer,
		PxU8*& solverConstraint,
		FrictionPatch*& _frictionPatches,
		PxU32& numFrictionPatches, PxU32& solverConstraintByteSize,
		PxU32& axisConstraintCount, PxConstraintAllocator& constraintAllocator,
		PxReal torsionalPatchRadius)
	{
		PX_ASSERT(NULL == solverConstraint);
		PX_ASSERT(NULL == _frictionPatches);
		PX_ASSERT(0 == numFrictionPatches);
		PX_ASSERT(0 == solverConstraintByteSize);
		PX_ASSERT(0 == axisConstraintCount);

		//From frictionPatchStream we just need to reserve a single buffer.
		PxU32 frictionPatchByteSize = 0;
		//Compute the sizes of all the buffers.
		computeBlockStreamByteSizesStep(
			useExtContacts, cBuffer,
			solverConstraintByteSize, frictionPatchByteSize, numFrictionPatches,
			axisConstraintCount, torsionalPatchRadius);

		//Reserve the buffers.

		//First reserve the accumulated buffer size for the constraint block.
		PxU8* constraintBlock = NULL;
		const PxU32 constraintBlockByteSize = solverConstraintByteSize;
		if (constraintBlockByteSize > 0)
		{
			constraintBlock = constraintAllocator.reserveConstraintData(constraintBlockByteSize + 16u);

			if (0 == constraintBlock || (reinterpret_cast<PxU8*>(-1)) == constraintBlock)
			{
				if (0 == constraintBlock)
				{
					PX_WARN_ONCE(
						"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
						"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
				}
				else
				{
					PX_WARN_ONCE(
						"Attempting to allocate more than 16K of contact data for a single contact pair in constraint prep. "
						"Either accept dropped contacts or simplify collision geometry.");
					constraintBlock = NULL;
				}
			}
			PX_ASSERT((size_t(constraintBlock) & 0xF) == 0);
		}

		FrictionPatch* frictionPatches = NULL;
		//If the constraint block reservation didn't fail then reserve the friction buffer too.
		if (frictionPatchByteSize > 0 && (0 == constraintBlockByteSize || constraintBlock))
		{
			frictionPatches = reinterpret_cast<FrictionPatch*>(constraintAllocator.reserveFrictionData(frictionPatchByteSize));

			if (0 == frictionPatches || (reinterpret_cast<FrictionPatch*>(-1)) == frictionPatches)
			{
				if (0 == frictionPatches)
				{
					PX_WARN_ONCE(
						"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
						"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
				}
				else
				{
					PX_WARN_ONCE(
						"Attempting to allocate more than 16K of friction data for a single contact pair in constraint prep. "
						"Either accept dropped contacts or simplify collision geometry.");
					frictionPatches = NULL;
				}
			}
		}

		_frictionPatches = frictionPatches;

		//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
		if (0 == constraintBlockByteSize || constraintBlock)
		{
			if (solverConstraintByteSize)
			{
				solverConstraint = constraintBlock;
				PX_ASSERT(0 == (uintptr_t(solverConstraint) & 0x0f));
			}
		}

		//Return true if neither of the two block reservations failed.
		return ((0 == constraintBlockByteSize || constraintBlock) && (0 == frictionPatchByteSize || frictionPatches));
	}

	class SolverExtBodyStep
	{
	public:
		union
		{
			const FeatherstoneArticulation* mArticulation;
			const PxTGSSolverBodyVel* mBody;
		};

		const PxTGSSolverBodyTxInertia* mTxI;
		const PxTGSSolverBodyData* mData;

		PxU32 mLinkIndex;

		SolverExtBodyStep(const void* bodyOrArticulation, const PxTGSSolverBodyTxInertia* txI, 
			const PxTGSSolverBodyData* data, PxU32 linkIndex) :
			mBody(reinterpret_cast<const PxTGSSolverBodyVel*>(bodyOrArticulation)),
			mTxI(txI),
			mData(data),
			mLinkIndex(linkIndex)
		{}

		PxReal projectVelocity(const PxVec3& linear, const PxVec3& angular) const;
		PxVec3 getLinVel() const;
		PxVec3 getAngVel() const;

		Cm::SpatialVectorV getVelocity() const;
		bool isKinematic() const 
		{ 
			return (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY) && mBody->isKinematic;
		}

		PxReal getCFM() const
		{
			return (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY) ? 0.f :
				mArticulation->getCfm(mLinkIndex);
		}
	};

	Cm::SpatialVector createImpulseResponseVector(const PxVec3& linear, const PxVec3& angular, const SolverExtBodyStep& body)
	{
		if (body.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			return Cm::SpatialVector(linear, body.mTxI->sqrtInvInertia * angular);

		return Cm::SpatialVector(linear, angular);
	}

	Cm::SpatialVectorV createImpulseResponseVector(const aos::Vec3V& linear, const aos::Vec3V& angular, const SolverExtBodyStep& body)
	{
		if (body.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
		{
			return Cm::SpatialVectorV(linear, M33MulV3(M33Load(body.mTxI->sqrtInvInertia), angular));
		}
		return Cm::SpatialVectorV(linear, angular);
	}

	PxReal getImpulseResponse(const SolverExtBodyStep& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0,
		const SolverExtBodyStep& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1,
		bool allowSelfCollision)
	{
		PxReal response;
		if (allowSelfCollision && b0.mArticulation == b1.mArticulation)
		{
			Cm::SpatialVectorF Z[64];
			b0.mArticulation->getImpulseSelfResponse(b0.mLinkIndex, b1.mLinkIndex, Z,
				impulse0.scale(dom0, angDom0), impulse1.scale(dom1, angDom1), deltaV0, deltaV1);

			response = impulse0.dot(deltaV0) + impulse1.dot(deltaV1);
		}
		else
		{
			if (b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			{
				deltaV0.linear = impulse0.linear * b0.mData->invMass * dom0;
				deltaV0.angular =/* b0.mBody->sqrtInvInertia * */impulse0.angular * angDom0;
			}
			else
			{
				//ArticulationHelper::getImpulseResponse(*b0.mFsData, b0.mLinkIndex, impulse0.scale(dom0, angDom0), deltaV0);
				const FeatherstoneArticulation* articulation = b0.mArticulation;
				Cm::SpatialVectorF Z[64];
				articulation->getImpulseResponse(b0.mLinkIndex, Z, impulse0.scale(dom0, angDom0), deltaV0);
			}

			response = impulse0.dot(deltaV0);
			if (b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			{
				deltaV1.linear = impulse1.linear * b1.mData->invMass * dom1;
				deltaV1.angular = /*b1.mBody->sqrtInvInertia * */impulse1.angular * angDom1;
			}
			else
			{
				const FeatherstoneArticulation* articulation = b1.mArticulation;
				Cm::SpatialVectorF Z[64];
				articulation->getImpulseResponse(b1.mLinkIndex, Z, impulse1.scale(dom1, angDom1), deltaV1);
				//ArticulationHelper::getImpulseResponse(*b1.mFsData, b1.mLinkIndex, impulse1.scale(dom1, angDom1), deltaV1);

			}
			response += impulse1.dot(deltaV1);
		}

		return response;
	}

	FloatV getImpulseResponse(const SolverExtBodyStep& b0, const Cm::SpatialVectorV& impulse0, Cm::SpatialVectorV& deltaV0, const FloatV& dom0, const FloatV& angDom0,
		const SolverExtBodyStep& b1, const Cm::SpatialVectorV& impulse1, Cm::SpatialVectorV& deltaV1, const FloatV& dom1, const FloatV& angDom1,
		bool /*allowSelfCollision*/)
	{
		Vec3V response;
		{

			if (b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			{
				deltaV0.linear = V3Scale(impulse0.linear, FMul(FLoad(b0.mData->invMass), dom0));
				deltaV0.angular = V3Scale(impulse0.angular, angDom0);
			}
			else
			{
				b0.mArticulation->getImpulseResponse(b0.mLinkIndex, NULL, impulse0.scale(dom0, angDom0), deltaV0);
			}

			response = V3Add(V3Mul(impulse0.linear, deltaV0.linear), V3Mul(impulse0.angular, deltaV0.angular));
			if (b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			{
				deltaV1.linear = V3Scale(impulse1.linear, FMul(FLoad(b1.mData->invMass), dom1));
				deltaV1.angular = V3Scale(impulse1.angular, angDom1);
			}
			else
			{
				b1.mArticulation->getImpulseResponse(b1.mLinkIndex, NULL, impulse1.scale(dom1, angDom1), deltaV1);
			}
			response = V3Add(response, V3Add(V3Mul(impulse1.linear, deltaV1.linear), V3Mul(impulse1.angular, deltaV1.angular)));
		}

		return V3SumElems(response);
	}



	PxReal SolverExtBodyStep::projectVelocity(const PxVec3& linear, const PxVec3& angular) const
	{
		if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			return mData->projectVelocity(linear, angular);
		else
		{
			Cm::SpatialVectorV velocities = mArticulation->getLinkVelocity(mLinkIndex);
			FloatV fv = velocities.dot(Cm::SpatialVector(linear, angular));
			PxF32 f;
			FStore(fv, &f);
			/*PxF32 f;
			FStore(getVelocity(*mFsData)[mLinkIndex].dot(Cm::SpatialVector(linear, angular)), &f);
			return f;*/
			return f;
		}
	}



	static FloatV constructContactConstraintStep(const Mat33V& sqrtInvInertia0, const Mat33V& sqrtInvInertia1, const FloatVArg invMassNorLenSq0,
		const FloatVArg invMassNorLenSq1, const FloatVArg angD0, const FloatVArg angD1, const Vec3VArg bodyFrame0p, const Vec3VArg bodyFrame1p,
		const QuatV& /*bodyFrame0q*/, const QuatV& /*bodyFrame1q*/,
		const Vec3VArg normal, const FloatVArg norVel0, const FloatVArg norVel1, const VecCrossV& norCross, const Vec3VArg angVel0, const Vec3VArg angVel1,
		const FloatVArg invDtp8, const FloatVArg invStepDt, const FloatVArg totalDt, const FloatVArg invTotalDt, 
		const FloatVArg restDistance, const FloatVArg restitution,
		const FloatVArg bounceThreshold, const PxContactPoint& contact, SolverContactPointStep& solverContact,
		const FloatVArg /*ccdMaxSeparation*/, const bool isKinematic0, const bool isKinematic1,
		const Vec3VArg solverOffsetSlop, const FloatVArg dt, const FloatVArg damping)
	{
		const FloatV zero = FZero();
		const Vec3V point = V3LoadA(contact.point);
		const FloatV separation = FLoad(contact.separation);

		const FloatV cTargetVel = V3Dot(normal, V3LoadA(contact.targetVel));

		const Vec3V ra = V3Sub(point, bodyFrame0p);
		const Vec3V rb = V3Sub(point, bodyFrame1p);

		Vec3V raXn = V3Cross(ra, norCross);
		Vec3V rbXn = V3Cross(rb, norCross);

		const FloatV angV0 = V3Dot(raXn, angVel0);
		const FloatV angV1 = V3Dot(rbXn, angVel1);

		const FloatV vRelAng = FSub(angV0, angV1);
		const FloatV vRelLin = FSub(norVel0, norVel1);

		const Vec3V slop = V3Scale(solverOffsetSlop, FMax(FSel(FIsEq(vRelLin,zero), FMax(), FDiv(vRelAng, vRelLin)), FOne()));

		const FloatV vrel1 = FAdd(norVel0, angV0);
		const FloatV vrel2 = FAdd(norVel1, angV1);

		const FloatV vrel = FSub(vrel1, vrel2);

		raXn = V3Sel(V3IsGrtr(slop, V3Abs(raXn)), V3Zero(), raXn);
		rbXn = V3Sel(V3IsGrtr(slop, V3Abs(rbXn)), V3Zero(), rbXn);

		const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, raXn);
		const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, rbXn);

		const FloatV i0 = FMul(V3Dot(raXnInertia, raXnInertia), angD0);
		const FloatV i1 = FMul(V3Dot(rbXnInertia, rbXnInertia), angD1);

		const FloatV resp0 = FAdd(invMassNorLenSq0, i0);
		const FloatV resp1 = FSub(i1, invMassNorLenSq1);

		const FloatV unitResponse = FAdd(resp0, resp1);


		const FloatV penetration = FSub(separation, restDistance);

		const BoolV isSeparated = FIsGrtr(penetration, zero);

		const FloatV penetrationInvDt = FMul(penetration, invTotalDt);

		const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), FIsGrtr(FNeg(vrel), penetrationInvDt));


		//The following line was replaced by an equivalent to avoid triggering an assert when FAdd sums totalDt to infinity which happens if vrel==0
		//const FloatV ratio = FSel(isGreater2, FAdd(totalDt, FDiv(penetration, vrel)), zero);
		const FloatV ratio = FAdd(totalDt, FSel(isGreater2, FDiv(penetration, vrel), FNeg(totalDt)));

		FloatV scaledBias;
		FloatV velMultiplier;

		FloatV recipResponse = FSel(FIsGrtr(unitResponse, FZero()), FRecip(unitResponse), FZero());

		if (FAllGrtr(zero, restitution))
		{
			//-ve restitution, so we treat it as a -spring coefficient.
			const FloatV nrdt = FMul(dt, restitution);

			const FloatV a = FMul(dt, FSub(damping, nrdt));

			const FloatV x = FRecip(FScaleAdd(a, unitResponse, FOne()));

			velMultiplier = FMul(x, a);
			//scaledBias = FSel(isSeparated, FNeg(invStepDt), FDiv(FMul(nrdt, FMul(x, unitResponse)), velMultiplier));
			scaledBias = FMul(nrdt, FMul(x, unitResponse));

		}
		else
		{
			scaledBias = FNeg(FSel(isSeparated, invStepDt, invDtp8));
			velMultiplier = recipResponse;
		}

		FloatV totalError = penetration;

		const FloatV sumVRel(vrel);

		FloatV targetVelocity = FAdd(cTargetVel, FSel(isGreater2, FMul(FNeg(sumVRel), restitution), zero));

		totalError = FScaleAdd(targetVelocity, ratio, totalError);

		if(isKinematic0)
			targetVelocity = FSub(targetVelocity, vrel1);
		if(isKinematic1)
			targetVelocity = FAdd(targetVelocity, vrel2);

		//KS - don't store scaled by angD0/angD1 here! It'll corrupt the velocity projection...
		V3StoreA(raXnInertia, solverContact.raXnI);
		V3StoreA(rbXnInertia, solverContact.rbXnI);

		FStore(velMultiplier, &solverContact.velMultiplier);

		FStore(totalError, &solverContact.separation);
		FStore(scaledBias, &solverContact.biasCoefficient);
		FStore(targetVelocity, &solverContact.targetVelocity);
		FStore(recipResponse, &solverContact.recipResponse);

		solverContact.maxImpulse = contact.maxImpulse;

		return penetration;

	}


	static void setupFinalizeSolverConstraints(Sc::ShapeInteraction* shapeInteraction,
		const PxContactPoint* buffer,
		const CorrelationBuffer& c,
		const PxTransform& bodyFrame0,
		const PxTransform& bodyFrame1,
		PxU8* workspace,
		const PxTGSSolverBodyVel& b0,
		const PxTGSSolverBodyVel& b1,
		const PxTGSSolverBodyTxInertia& txI0,
		const PxTGSSolverBodyTxInertia& txI1,
		const PxTGSSolverBodyData& data0,
		const PxTGSSolverBodyData& data1,
		const PxReal invDtF32,
		const PxReal totalDtF32,
		const PxReal invTotalDtF32,
		const PxReal dtF32,
		PxReal bounceThresholdF32,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		bool hasForceThreshold, bool staticOrKinematicBody,
		const PxReal restDist, PxU8* frictionDataPtr,
		const PxReal maxCCDSeparation,
		const bool disableStrongFriction,
		const PxReal torsionalPatchRadiusF32,
		const PxReal minTorsionalPatchRadiusF32,
		const PxReal biasCoefficient,
		const PxReal solverOffsetSlop)
	{
		bool hasTorsionalFriction = torsionalPatchRadiusF32 > 0.f || minTorsionalPatchRadiusF32 > 0.f;

		// NOTE II: the friction patches are sparse (some of them have no contact patches, and
		// therefore did not get written back to the cache) but the patch addresses are dense,
		// corresponding to valid patches

		const bool isKinematic0 = b0.isKinematic;
		const bool isKinematic1 = b1.isKinematic;

		const FloatV ccdMaxSeparation = FLoad(maxCCDSeparation);

		PxU8 flags = PxU8(hasForceThreshold ? SolverContactHeaderStep::eHAS_FORCE_THRESHOLDS : 0);

		PxU8* PX_RESTRICT ptr = workspace;

		PxU8 type = PxTo8(staticOrKinematicBody ? DY_SC_TYPE_STATIC_CONTACT
			: DY_SC_TYPE_RB_CONTACT);

		const FloatV zero = FZero();

		const FloatV d0 = FLoad(invMassScale0);
		const FloatV d1 = FLoad(invMassScale1);
		const FloatV angD0 = FLoad(invInertiaScale0);
		const FloatV angD1 = FLoad(invInertiaScale1);

		const Vec3V offsetSlop = V3Load(solverOffsetSlop);

		const FloatV nDom1fV = FNeg(d1);

		const FloatV invMass0 = FLoad(data0.invMass);
		const FloatV invMass1 = FLoad(data1.invMass);

		const FloatV invMass0_dom0fV = FMul(d0, invMass0);
		const FloatV invMass1_dom1fV = FMul(nDom1fV, invMass1);

		Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass0_dom0fV);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass1_dom1fV);

		const FloatV restDistance = FLoad(restDist);

		const PxReal maxPenBias = PxMax(data0.penBiasClamp, data1.penBiasClamp);

		const QuatV bodyFrame0q = QuatVLoadU(&bodyFrame0.q.x);
		const Vec3V bodyFrame0p = V3LoadU(bodyFrame0.p);

		const QuatV bodyFrame1q = QuatVLoadU(&bodyFrame1.q.x);
		const Vec3V bodyFrame1p = V3LoadU(bodyFrame1.p);

		

		PxU32 frictionPatchWritebackAddrIndex = 0;

		PxPrefetchLine(c.contactID);
		PxPrefetchLine(c.contactID, 128);

		//const Vec3V linVel0 = V3LoadU_SafeReadW(b0.linearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		//const Vec3V linVel1 = V3LoadU_SafeReadW(b1.linearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		//const Vec3V angVel0 = V3LoadU_SafeReadW(b0.angularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData
		//const Vec3V angVel1 = V3LoadU_SafeReadW(b1.angularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData


		const Vec3V linVel0 = V3LoadU_SafeReadW(data0.originalLinearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		const Vec3V linVel1 = V3LoadU_SafeReadW(data1.originalLinearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
		const Vec3V angVel0 = V3LoadU_SafeReadW(data0.originalAngularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData
		const Vec3V angVel1 = V3LoadU_SafeReadW(data1.originalAngularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData


		PX_ALIGN(16, const Mat33V sqrtInvInertia0)
			(
			V3LoadU_SafeReadW(txI0.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
			V3LoadU_SafeReadW(txI0.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
			V3LoadU(txI0.sqrtInvInertia.column2)
			);

		PX_ALIGN(16, const Mat33V sqrtInvInertia1)
			(
			V3LoadU_SafeReadW(txI1.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
			V3LoadU_SafeReadW(txI1.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
			V3LoadU(txI1.sqrtInvInertia.column2)
			);

		const FloatV invDt = FLoad(invDtF32);
		const FloatV dt = FLoad(dtF32);

		const FloatV totalDt = FLoad(totalDtF32);
		const FloatV invTotalDt = FLoad(invTotalDtF32);

		const PxReal scale = PxMin(0.8f, biasCoefficient);

		const FloatV p8 = FLoad(scale);
		const FloatV bounceThreshold = FLoad(bounceThresholdF32);

		const FloatV invDtp8 = FMul(invDt, p8);

		const PxReal frictionBiasScale = disableStrongFriction ? 0.f : invDtF32 * scale;


		for (PxU32 i = 0; i<c.frictionPatchCount; i++)
		{
			PxU32 contactCount = c.frictionPatchContactCounts[i];
			if (contactCount == 0)
				continue;

			const FrictionPatch& frictionPatch = c.frictionPatches[i];
			PX_ASSERT(frictionPatch.anchorCount <= 2);

			PxU32 firstPatch = c.correlationListHeads[i];
			const PxContactPoint* contactBase0 = buffer + c.contactPatches[firstPatch].start;

			const PxReal combinedRestitution = contactBase0->restitution;
			const PxReal combinedDamping = contactBase0->damping;

			SolverContactHeaderStep* PX_RESTRICT header = reinterpret_cast<SolverContactHeaderStep*>(ptr);
			ptr += sizeof(SolverContactHeaderStep);


			PxPrefetchLine(ptr, 128);
			PxPrefetchLine(ptr, 256);

			header->shapeInteraction = shapeInteraction;
			header->flags = flags;
			header->minNormalForce = 0.f;
			FStore(invMass0_dom0fV, &header->invMass0);
			FStore(FNeg(invMass1_dom1fV), &header->invMass1);
			const FloatV restitution = FLoad(combinedRestitution);
			const FloatV damping = FLoad(combinedDamping);

			PxU32 pointStride = sizeof(SolverContactPointStep);
			PxU32 frictionStride = sizeof(SolverContactFrictionStep);

			const Vec3V normal = V3LoadA(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
			const FloatV normalLenSq = V3LengthSq(normal);
			const VecCrossV norCross = V3PrepareCross(normal);
			//const FloatV norVel = V3SumElems(V3NegMulSub(normal, linVel1, V3Mul(normal, linVel0)));
			const FloatV norVel0 = V3Dot(linVel0, normal);
			const FloatV norVel1 = V3Dot(linVel1, normal);

			const FloatV invMassNorLenSq0 = FMul(invMass0_dom0fV, normalLenSq);
			const FloatV invMassNorLenSq1 = FMul(invMass1_dom1fV, normalLenSq);

			V3StoreA(normal, header->normal);
			header->maxPenBias = contactBase0->restitution < 0.f ? -PX_MAX_F32 : maxPenBias;

			FloatV maxPenetration = FMax();

			for (PxU32 patch = c.correlationListHeads[i];
				patch != CorrelationBuffer::LIST_END;
				patch = c.contactPatches[patch].next)
			{
				const PxU32 count = c.contactPatches[patch].count;
				const PxContactPoint* contactBase = buffer + c.contactPatches[patch].start;

				PxU8* p = ptr;

				for (PxU32 j = 0; j<count; j++)
				{
					PxPrefetchLine(p, 256);
					const PxContactPoint& contact = contactBase[j];

					SolverContactPointStep* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointStep*>(p);
					p += pointStride;

					maxPenetration = FMin(maxPenetration, constructContactConstraintStep(sqrtInvInertia0, sqrtInvInertia1, invMassNorLenSq0,
						invMassNorLenSq1, angD0, angD1, bodyFrame0p, bodyFrame1p, bodyFrame0q, bodyFrame1q,
						normal, norVel0, norVel1, norCross, angVel0, angVel1,
						invDtp8, invDt, totalDt, invTotalDt, restDistance, restitution,
						bounceThreshold, contact, *solverContact,
						ccdMaxSeparation, isKinematic0, isKinematic1, offsetSlop, dt, damping));
				}

				ptr = p;
			}

			PxF32* forceBuffers = reinterpret_cast<PxF32*>(ptr);
			PxMemZero(forceBuffers, sizeof(PxF32) * contactCount);
			ptr += ((contactCount + 3) & (~3)) * sizeof(PxF32); // jump to next 16-byte boundary

			const PxReal staticFriction = contactBase0->staticFriction;
			const PxReal dynamicFriction = contactBase0->dynamicFriction;
			const bool disableFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));

			const bool haveFriction = (disableFriction == 0 && frictionPatch.anchorCount != 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
			header->numNormalConstr = PxTo8(contactCount);
			header->numFrictionConstr = PxTo8(haveFriction ? frictionPatch.anchorCount * 2 : 0);

			header->type = type;

			header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;
			FStore(angD0, &header->angDom0);
			FStore(angD1, &header->angDom1);

			header->broken = 0;

			if (haveFriction)
			{
				

				const Vec3V linVrel = V3Sub(linVel0, linVel1);
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

				const Vec3V t1 = V3Normalize(V3Cross(norCross, t0Cross));
				//const VecCrossV t1Cross = V3PrepareCross(t1);


				// since we don't even have the body velocities we can't compute the tangent dirs, so 
				// the only thing we can do right now is to write the geometric information (which is the
				// same for both axis constraints of an anchor) We put ra in the raXn field, rb in the rbXn
				// field, and the error in the normal field. See corresponding comments in
				// completeContactFriction()

				//We want to set the writeBack ptr to point to the broken flag of the friction patch.
				//On spu we have a slight problem here because the friction patch array is 
				//in local store rather than in main memory. The good news is that the address of the friction 
				//patch array in main memory is stored in the work unit. These two addresses will be equal 
				//except on spu where one is local store memory and the other is the effective address in main memory.
				//Using the value stored in the work unit guarantees that the main memory address is used on all platforms.
				PxU8* PX_RESTRICT writeback = frictionDataPtr + frictionPatchWritebackAddrIndex*sizeof(FrictionPatch);

				

				const FloatV norVel00 = V3Dot(linVel0, t0);
				const FloatV norVel01 = V3Dot(linVel1, t0);
				const FloatV norVel10 = V3Dot(linVel0, t1);
				const FloatV norVel11 = V3Dot(linVel1, t1);
				
				const Vec3V relTr = V3Sub(bodyFrame0p, bodyFrame1p);

				header->frictionBrokenWritebackByte = writeback;

				PxReal frictionScale = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

				for (PxU32 j = 0; j < frictionPatch.anchorCount; j++)
				{
					PxPrefetchLine(ptr, 256);
					PxPrefetchLine(ptr, 384);
					SolverContactFrictionStep* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionStep*>(ptr);
					ptr += frictionStride;
					SolverContactFrictionStep* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionStep*>(ptr);
					ptr += frictionStride;

					Vec3V body0Anchor = V3LoadU(frictionPatch.body0Anchors[j]);
					Vec3V body1Anchor = V3LoadU(frictionPatch.body1Anchors[j]);

					Vec3V ra = QuatRotate(bodyFrame0q, body0Anchor);
					Vec3V rb = QuatRotate(bodyFrame1q, body1Anchor);

					PxU32 index = c.contactID[i][j];

					index = index == 0xFFFF ? c.contactPatches[c.correlationListHeads[i]].start : index;

					const Vec3V tvel = V3LoadA(buffer[index].targetVel);

					const Vec3V error = V3Add(V3Sub(ra, rb), relTr);

					

					{
						Vec3V raXn = V3Cross(ra, t0);
						Vec3V rbXn = V3Cross(rb, t0);

						raXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(raXn)), V3Zero(), raXn);
						rbXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

						const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, raXn);
						const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, rbXn);

						const FloatV resp0 = FAdd(invMassNorLenSq0, FMul(V3Dot(raXnInertia, raXnInertia), angD0));
						const FloatV resp1 = FSub(FMul(V3Dot(rbXnInertia, rbXnInertia), angD1), invMassNorLenSq1);

						const FloatV unitResponse = FAdd(resp0, resp1);

						const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FDiv(p8, unitResponse), zero);
						//const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FRecip(unitResponse), zero);

						FloatV targetVel = V3Dot(tvel, t0);

						if(isKinematic0)
							targetVel = FSub(targetVel, FAdd(norVel00, V3Dot(raXn, angVel0)));
						if (isKinematic1)
							targetVel = FAdd(targetVel, FAdd(norVel01, V3Dot(rbXn, angVel1)));

						f0->normalXYZ_ErrorW = V4SetW(t0, V3Dot(error, t0));
						//f0->raXnXYZ_targetVelW = V4SetW(body0Anchor, targetVel);
						//f0->rbXnXYZ_biasW = V4SetW(body1Anchor, FZero());
						/*f0->raXn_biasScaleW = V4SetW(Vec4V_From_Vec3V(raXn), frictionBiasScale);
						f0->rbXn_errorW = V4SetW(rbXn, V3Dot(error, t0));*/
						f0->raXnI_targetVelW = V4SetW(raXnInertia, targetVel);
						f0->rbXnI_velMultiplierW = V4SetW(rbXnInertia, velMultiplier);
						f0->appliedForce = 0.f;
						f0->frictionScale = frictionScale;
						f0->biasScale = frictionBiasScale;
					}

					{
						FloatV targetVel = V3Dot(tvel, t1);


						Vec3V raXn = V3Cross(ra, t1);
						Vec3V rbXn = V3Cross(rb, t1);

						raXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(raXn)), V3Zero(), raXn);
						rbXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

						const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, raXn);
						const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, rbXn);

						const FloatV resp0 = FAdd(invMassNorLenSq0, FMul(V3Dot(raXnInertia, raXnInertia), angD0));
						const FloatV resp1 = FSub(FMul(V3Dot(rbXnInertia, rbXnInertia), angD1), invMassNorLenSq1);

						const FloatV unitResponse = FAdd(resp0, resp1);

						const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FDiv(p8, unitResponse), zero);
						//const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FRecip(unitResponse), zero);

						if (isKinematic0)
							targetVel = FSub(targetVel, FAdd(norVel10, V3Dot(raXn, angVel0)));
						if (isKinematic1)
							targetVel = FAdd(targetVel, FAdd(norVel11, V3Dot(rbXn, angVel1)));


						f1->normalXYZ_ErrorW = V4SetW(t1, V3Dot(error, t1));
						//f1->raXnXYZ_targetVelW = V4SetW(body0Anchor, targetVel);
						//f1->rbXnXYZ_biasW = V4SetW(body1Anchor, FZero());
						//f1->raXn_biasScaleW = V4SetW(Vec4V_From_Vec3V(V3Cross(ra, t1)), frictionBiasScale);
						//f1->rbXn_errorW = V4SetW(V3Cross(rb, t1), V3Dot(error, t1));
						f1->raXnI_targetVelW = V4SetW(raXnInertia, targetVel);
						f1->rbXnI_velMultiplierW = V4SetW(rbXnInertia, velMultiplier);
						f1->appliedForce = 0.f;
						f1->frictionScale = frictionScale;
						f1->biasScale = frictionBiasScale;
					}
				}

				if (hasTorsionalFriction && frictionPatch.anchorCount == 1)
				{
					const FloatV torsionalPatchRadius = FLoad(torsionalPatchRadiusF32);
					const FloatV minTorsionalPatchRadius = FLoad(minTorsionalPatchRadiusF32);
					const FloatV torsionalFriction = FMax(minTorsionalPatchRadius, FSqrt(FMul(FMax(zero, FNeg(maxPenetration)), torsionalPatchRadius)));
					header->numFrictionConstr++;
					SolverContactFrictionStep* PX_RESTRICT f = reinterpret_cast<SolverContactFrictionStep*>(ptr);
					ptr += frictionStride;

					const Vec3V raXnInertia = M33MulV3(sqrtInvInertia0, normal);
					const Vec3V rbXnInertia = M33MulV3(sqrtInvInertia1, normal);

					const FloatV resp0 = FMul(V3Dot(raXnInertia, raXnInertia), angD0);
					const FloatV resp1 = FMul(V3Dot(rbXnInertia, rbXnInertia), angD1);

					const FloatV unitResponse = FAdd(resp0, resp1);

					const FloatV velMultiplier = FSel(FIsGrtr(unitResponse, zero), FDiv(p8, unitResponse), zero);

					FloatV targetVel = zero;

					if (isKinematic0)
						targetVel = V3Dot(normal, angVel0);
					if (isKinematic1)
						targetVel = V3Dot(normal, angVel1);

					f->normalXYZ_ErrorW = V4Zero();
					f->raXnI_targetVelW = V4SetW(raXnInertia, targetVel);
					f->rbXnI_velMultiplierW = V4SetW(rbXnInertia, velMultiplier);
					f->biasScale = 0.f;
					f->appliedForce = 0.f;
					FStore(torsionalFriction, &f->frictionScale);
				}
			}

			frictionPatchWritebackAddrIndex++;
		}
	}


	static FloatV setupExtSolverContactStep(const SolverExtBodyStep& b0, const SolverExtBodyStep& b1,
		const FloatV& d0, const FloatV& d1, const FloatV& angD0, const FloatV& angD1, const Vec3V& bodyFrame0p, const Vec3V& bodyFrame1p,
		const Vec3VArg normal, const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg invStepDt, const FloatVArg totalDt,
		const FloatVArg dt, const FloatVArg restDistance, const FloatVArg restitution, const FloatVArg damping,
		const FloatVArg bounceThreshold, const PxContactPoint& contact, SolverContactPointStepExt& solverContact, const FloatVArg /*ccdMaxSeparation*/,
		const bool isKinematic0, const bool isKinematic1, const FloatVArg cfm,
		const Cm::SpatialVectorV& v0, const Cm::SpatialVectorV& v1,
		const Vec3VArg solverOffsetSlop, const FloatVArg norVel0, const FloatVArg norVel1)
	{
		const FloatV zero = FZero();
		const FloatV separation = FLoad(contact.separation);

		FloatV penetration = FSub(separation, restDistance);

		const Vec3V ra = V3Sub(V3LoadA(contact.point),bodyFrame0p);
		const Vec3V rb = V3Sub(V3LoadA(contact.point), bodyFrame1p);

		Vec3V raXn = V3Cross(ra, normal);
		Vec3V rbXn = V3Cross(rb, normal);

		Cm::SpatialVectorV deltaV0, deltaV1;

		

		const FloatV vRelAng = V3SumElems(V3Sub(V3Mul(v0.angular, raXn), V3Mul(v1.angular, rbXn)));
		const FloatV vRelLin = FSub(norVel0, norVel1);

		const Vec3V slop = V3Scale(solverOffsetSlop, FMax(FSel(FIsEq(vRelLin, zero), FOne(), FDiv(vRelAng, vRelLin)), FOne()));

		raXn = V3Sel(V3IsGrtr(slop, V3Abs(raXn)), V3Zero(), raXn);
		rbXn = V3Sel(V3IsGrtr(slop, V3Abs(rbXn)), V3Zero(), rbXn);

		const FloatV angV0 = V3Dot(v0.angular, raXn);
		const FloatV angV1 = V3Dot(v1.angular, rbXn);

		const Cm::SpatialVectorV resp0 = createImpulseResponseVector(normal, raXn, b0);
		const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(normal), V3Neg(rbXn), b1);

		FloatV unitResponse = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
			b1, resp1, deltaV1, d1, angD1, false);


		const FloatV vrel = FAdd(vRelAng, vRelLin);

		FloatV scaledBias, velMultiplier;

		FloatV recipResponse = FSel(FIsGrtr(unitResponse, FEps()), FRecip(FAdd(unitResponse, cfm)), zero);


		if (FAllGrtr(zero, restitution))
		{
			const FloatV nrdt = FMul(dt, restitution);

			const FloatV a = FMul(dt, FSub(damping, nrdt));

			const FloatV x = FRecip(FScaleAdd(a, unitResponse, FOne()));

			velMultiplier = FMul(x, a);
			//scaledBias = FSel(isSeparated, FNeg(invStepDt), FDiv(FMul(nrdt, FMul(x, unitResponse)), velMultiplier));
			scaledBias = FMul(nrdt, FMul(x, unitResponse));

		}
		else
		{
			velMultiplier = recipResponse;
			scaledBias = FNeg(FSel(FIsGrtr(penetration, zero), invStepDt, invDtp8));
		}
		
		const FloatV penetrationInvDt = FMul(penetration, invDt);

		const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), FIsGrtr(FNeg(vrel), penetrationInvDt));


		FloatV targetVelocity = FSel(isGreater2, FMul(FNeg(vrel), restitution), zero);

		const FloatV cTargetVel = V3Dot(V3LoadA(contact.targetVel), normal);

		targetVelocity = FAdd(targetVelocity, cTargetVel);

		//const FloatV deltaF = FMax(FMul(FSub(targetVelocity, FAdd(vrel, FMax(zero, penetrationInvDt))), velMultiplier), zero);
		const FloatV deltaF = FMax(FMul(FAdd(targetVelocity, FSub(FNeg(penetrationInvDt), vrel)), velMultiplier), zero);

		const FloatV ratio = FSel(isGreater2, FAdd(totalDt, FDiv(penetration, vrel)), zero);

		penetration = FScaleAdd(targetVelocity, ratio, penetration);

		if (isKinematic0)
			targetVelocity = FSub(targetVelocity, FAdd(norVel0, angV0));
		if(isKinematic1)
			targetVelocity = FAdd(targetVelocity, FAdd(norVel1, angV1));

		FStore(scaledBias, &solverContact.biasCoefficient);
		FStore(targetVelocity, &solverContact.targetVelocity);
		FStore(recipResponse, &solverContact.recipResponse);

		const Vec4V raXnI_Sepw = V4SetW(Vec4V_From_Vec3V(resp0.angular), penetration);
		const Vec4V rbXnI_velMulW = V4SetW(Vec4V_From_Vec3V(V3Neg(resp1.angular)), velMultiplier);
		//solverContact.raXnI = resp0.angular;
		//solverContact.rbXnI = -resp1.angular;

		V4StoreA(raXnI_Sepw, &solverContact.raXnI.x);
		V4StoreA(rbXnI_velMulW, &solverContact.rbXnI.x);
		solverContact.linDeltaVA = deltaV0.linear;
		solverContact.angDeltaVA = deltaV0.angular;
		solverContact.linDeltaVB = deltaV1.linear;
		solverContact.angDeltaVB = deltaV1.angular;
		solverContact.maxImpulse = contact.maxImpulse;

		return deltaF;
	}

	//PX_INLINE void computeFrictionTangents(const PxVec3& vrel, const PxVec3& unitNormal, PxVec3& t0, PxVec3& t1)
	//{
	//	PX_ASSERT(PxAbs(unitNormal.magnitude() - 1)<1e-3f);

	//	t0 = vrel - unitNormal * unitNormal.dot(vrel);
	//	PxReal ll = t0.magnitudeSquared();

	//	if (ll > 0.1f)										//can set as low as 0.
	//	{
	//		t0 *= PxRecipSqrt(ll);
	//		t1 = unitNormal.cross(t0);
	//	}
	//	else
	//		PxNormalToTangents(unitNormal, t0, t1);		//fallback
	//}

	PxVec3 SolverExtBodyStep::getLinVel() const
	{
		if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			return mBody->linearVelocity;
		else
		{
			Cm::SpatialVectorV velocity = mArticulation->getLinkVelocity(mLinkIndex);
			PxVec3 result;
			V3StoreU(velocity.linear, result);
			return result;
		}
	}

	Cm::SpatialVectorV SolverExtBodyStep::getVelocity() const
	{
		if (mLinkIndex == PxSolverConstraintDesc::RIGID_BODY)
			return Cm::SpatialVectorV(V3LoadA(mData->originalLinearVelocity), V3LoadA(mData->originalAngularVelocity));
		else
			return mArticulation->getLinkVelocity(mLinkIndex);
	}



	void setupFinalizeExtSolverContactsStep(
		const PxContactPoint* buffer,
		const CorrelationBuffer& c,
		const PxTransform& bodyFrame0,
		const PxTransform& bodyFrame1,
		PxU8* workspace,
		const SolverExtBodyStep& b0,
		const SolverExtBodyStep& b1,
		const PxReal invDtF32,
		const PxReal invTotalDtF32,
		const PxReal totalDtF32,
		const PxReal dtF32,
		PxReal bounceThresholdF32,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		const PxReal restDist,
		PxU8* frictionDataPtr,
		PxReal ccdMaxContactDist,
		const PxReal torsionalPatchRadiusF32,
		const PxReal minTorsionalPatchRadiusF32,
		const PxReal biasCoefficient,
		const PxReal solverOffsetSlop)
	{
		// NOTE II: the friction patches are sparse (some of them have no contact patches, and
		// therefore did not get written back to the cache) but the patch addresses are dense,
		// corresponding to valid patches

		/*const bool haveFriction = PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;*/

		const bool isKinematic0 = b0.isKinematic();
		const bool isKinematic1 = b1.isKinematic();

		const FloatV ccdMaxSeparation = FLoad(ccdMaxContactDist);

		bool hasTorsionalFriction = torsionalPatchRadiusF32 > 0.f || minTorsionalPatchRadiusF32 > 0.f;
		const FloatV quarter = FLoad(0.25f);

		PxU8* PX_RESTRICT ptr = workspace;

		const FloatV zero = FZero();

		const PxF32 maxPenBias0 = b0.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY ? b0.mData->penBiasClamp : b0.mArticulation->getLinkMaxPenBias(b0.mLinkIndex);
		const PxF32 maxPenBias1 = b1.mLinkIndex == PxSolverConstraintDesc::RIGID_BODY ? b1.mData->penBiasClamp : b1.mArticulation->getLinkMaxPenBias(b1.mLinkIndex);

		const PxReal maxPenBias = PxMax(maxPenBias0, maxPenBias1);

		const Cm::SpatialVectorV v0 = b0.getVelocity();
		const Cm::SpatialVectorV v1 = b1.getVelocity();

		const FloatV d0 = FLoad(invMassScale0);
		const FloatV d1 = FLoad(invMassScale1);

		const FloatV angD0 = FLoad(invInertiaScale0);
		const FloatV angD1 = FLoad(invInertiaScale1);

		const Vec3V bodyFrame0p = V3LoadU(bodyFrame0.p);
		const Vec3V bodyFrame1p = V3LoadU(bodyFrame1.p);

		Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, d0);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, d1);

		const FloatV restDistance = FLoad(restDist);

		PxU32 frictionPatchWritebackAddrIndex = 0;

		PxPrefetchLine(c.contactID);
		PxPrefetchLine(c.contactID, 128);

		const FloatV invDt = FLoad(invDtF32);
		const FloatV invTotalDt = FLoad(invTotalDtF32);
		const PxReal scale = PxMin(0.8f, biasCoefficient);
		const FloatV p8 = FLoad(scale);
		const FloatV bounceThreshold = FLoad(bounceThresholdF32);

		const FloatV totalDt = FLoad(totalDtF32);
		const FloatV dt = FLoad(dtF32);

		const FloatV invDtp8 = FMul(invDt, p8);

		const FloatV cfm = FLoad(PxMax(b0.getCFM(), b1.getCFM()));

		PxU8 flags = 0;

		const Vec3V offsetSlop = V3Load(solverOffsetSlop);

		for (PxU32 i = 0; i<c.frictionPatchCount; i++)
		{
			PxU32 contactCount = c.frictionPatchContactCounts[i];
			if (contactCount == 0)
				continue;

			const FrictionPatch& frictionPatch = c.frictionPatches[i];
			PX_ASSERT(frictionPatch.anchorCount <= 2);  //0==anchorCount is allowed if all the contacts in the manifold have a large offset. 

			const PxContactPoint* contactBase0 = buffer + c.contactPatches[c.correlationListHeads[i]].start;
	
			const bool useImprovedFrictionPatch = !!(contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION) && frictionPatch.anchorCount == 2;
		
			PxReal coefficient = useImprovedFrictionPatch ? 1.f/frictionPatch.anchorCount : 1.f;

			const PxReal staticFriction = contactBase0->staticFriction*coefficient;
			const PxReal dynamicFriction = contactBase0->dynamicFriction*coefficient;
			const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
			staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));

			const PxReal frictionBiasScale = disableStrongFriction ? 0.f : invDtF32 * 0.8f;

			SolverContactHeaderStep* PX_RESTRICT header = reinterpret_cast<SolverContactHeaderStep*>(ptr);
			ptr += sizeof(SolverContactHeaderStep);


			PxPrefetchLine(ptr + 128);
			PxPrefetchLine(ptr + 256);
			PxPrefetchLine(ptr + 384);

			const bool haveFriction = (disableStrongFriction == 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
			header->numNormalConstr = PxTo8(contactCount);
			header->numFrictionConstr = PxTo8(haveFriction ? frictionPatch.anchorCount * 2 : 0);

			header->type = PxTo8(DY_SC_TYPE_EXT_CONTACT);

			header->flags = flags;

			const FloatV restitution = FLoad(contactBase0->restitution);
			const FloatV damping = FLoad(contactBase0->damping);

			header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;

			header->angDom0 = invInertiaScale0;
			header->angDom1 = invInertiaScale1;

			const PxU32 pointStride = sizeof(SolverContactPointStepExt);
			const PxU32 frictionStride = sizeof(SolverContactFrictionStepExt);

			const Vec3V normal = V3LoadU(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);

			V3StoreA(normal, header->normal);
			header->maxPenBias = maxPenBias;

			FloatV maxPenetration = FZero();

			FloatV accumulatedImpulse = FZero();

			const FloatV norVel0 = V3Dot(v0.linear, normal);
			const FloatV norVel1 = V3Dot(v1.linear, normal);

			for (PxU32 patch = c.correlationListHeads[i];
				patch != CorrelationBuffer::LIST_END;
				patch = c.contactPatches[patch].next)
			{
				const PxU32 count = c.contactPatches[patch].count;
				const PxContactPoint* contactBase = buffer + c.contactPatches[patch].start;

				PxU8* p = ptr;
				
				for (PxU32 j = 0; j<count; j++)
				{
					const PxContactPoint& contact = contactBase[j];

					SolverContactPointStepExt* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointStepExt*>(p);
					p += pointStride;

					accumulatedImpulse = FAdd(accumulatedImpulse, setupExtSolverContactStep(b0, b1, d0, d1, angD0, angD1, bodyFrame0p, bodyFrame1p, normal, invTotalDt, invDtp8, 
						invDt, totalDt, dt, restDistance, restitution, damping, bounceThreshold, contact, *solverContact, ccdMaxSeparation, isKinematic0, isKinematic1, 
						cfm, v0, v1, offsetSlop, norVel0, norVel1));

					maxPenetration = FMin(FLoad(contact.separation), maxPenetration);

				}

				ptr = p;
			}
			accumulatedImpulse = FMul(FDiv(accumulatedImpulse, FLoad(PxF32(contactCount))), quarter);

			FStore(accumulatedImpulse, &header->minNormalForce);

			PxF32* forceBuffer = reinterpret_cast<PxF32*>(ptr);
			PxMemZero(forceBuffer, sizeof(PxF32) * contactCount);
			ptr += sizeof(PxF32) * ((contactCount + 3) & (~3));

			header->broken = 0;

			if (haveFriction)
			{
				//const Vec3V normal = Vec3V_From_PxVec3(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);
				const Vec3V linVrel = V3Sub(v0.linear, v1.linear);
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
				PxU8* PX_RESTRICT writeback = frictionDataPtr + frictionPatchWritebackAddrIndex * sizeof(FrictionPatch);

				header->frictionBrokenWritebackByte = writeback;

				PxReal frictionScale = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

				for (PxU32 j = 0; j < frictionPatch.anchorCount; j++)
				{
					SolverContactFrictionStepExt* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionStepExt*>(ptr);
					ptr += frictionStride;
					SolverContactFrictionStepExt* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionStepExt*>(ptr);
					ptr += frictionStride;

					Vec3V ra = V3LoadU(bodyFrame0.q.rotate(frictionPatch.body0Anchors[j]));
					Vec3V rb = V3LoadU(bodyFrame1.q.rotate(frictionPatch.body1Anchors[j]));
					Vec3V error = V3Sub(V3Add(ra, bodyFrame0p), V3Add(rb,bodyFrame1p));

					{
						Vec3V raXn = V3Cross(ra, t0Cross);
						Vec3V rbXn = V3Cross(rb, t0Cross);
						raXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(raXn)), V3Zero(), raXn);
						rbXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

						Cm::SpatialVectorV deltaV0, deltaV1;

						const Cm::SpatialVectorV resp0 = createImpulseResponseVector(t0, raXn, b0);
						const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(t0), V3Neg(rbXn), b1);
						FloatV resp = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
							b1, resp1, deltaV1, d1, angD1, false);

						const FloatV velMultiplier = FSel(FIsGrtr(resp, FEps()), FDiv(p8, FAdd(cfm, resp)), zero);

						PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
						FloatV targetVel = V3Dot(V3LoadA(buffer[index].targetVel), t0);

						if(isKinematic0)
							targetVel = FSub(targetVel, v0.dot(resp0));
						if(isKinematic1)
							targetVel = FSub(targetVel, v1.dot(resp1));

						f0->normalXYZ_ErrorW = V4SetW(Vec4V_From_Vec3V(t0), V3Dot(error, t0));
						f0->raXnI_targetVelW = V4SetW(Vec4V_From_Vec3V(resp0.angular), targetVel);
						f0->rbXnI_velMultiplierW = V4SetW(V4Neg(Vec4V_From_Vec3V(resp1.angular)), velMultiplier);
						f0->appliedForce = 0.f;
						f0->biasScale = frictionBiasScale;
						f0->linDeltaVA = deltaV0.linear;
						f0->linDeltaVB = deltaV1.linear;
						f0->angDeltaVA = deltaV0.angular;
						f0->angDeltaVB = deltaV1.angular;
						f0->frictionScale = frictionScale;
					}

					{

						Vec3V raXn = V3Cross(ra, t1Cross);
						Vec3V rbXn = V3Cross(rb, t1Cross);
						raXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(raXn)), V3Zero(), raXn);
						rbXn = V3Sel(V3IsGrtr(offsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

						Cm::SpatialVectorV deltaV0, deltaV1;


						const Cm::SpatialVectorV resp0 = createImpulseResponseVector(t1, raXn, b0);
						const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(t1), V3Neg(rbXn), b1);

						FloatV resp = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
							b1, resp1, deltaV1, d1, angD1, false);

						const FloatV velMultiplier = FSel(FIsGrtr(resp, FEps()), FDiv(p8, FAdd(cfm, resp)), zero);

						PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
						FloatV targetVel = V3Dot(V3LoadA(buffer[index].targetVel), t1);

						if (isKinematic0)
							targetVel = FSub(targetVel, v0.dot(resp0));
						if (isKinematic1)
							targetVel = FSub(targetVel, v1.dot(resp1));

						f1->normalXYZ_ErrorW = V4SetW(Vec4V_From_Vec3V(t1), V3Dot(error, t1));
						f1->raXnI_targetVelW = V4SetW(Vec4V_From_Vec3V(resp0.angular), targetVel);
						f1->rbXnI_velMultiplierW = V4SetW(V4Neg(Vec4V_From_Vec3V(resp1.angular)), velMultiplier);
						f1->appliedForce = 0.f;
						f1->biasScale = frictionBiasScale;
						f1->linDeltaVA = deltaV0.linear;
						f1->linDeltaVB = deltaV1.linear;
						f1->angDeltaVA = deltaV0.angular;
						f1->angDeltaVB = deltaV1.angular;
						f1->frictionScale = frictionScale;
					}
				}

				if (hasTorsionalFriction && frictionPatch.anchorCount == 1)
				{
					const FloatV torsionalPatchRadius = FLoad(torsionalPatchRadiusF32);
					const FloatV minTorsionalPatchRadius = FLoad(minTorsionalPatchRadiusF32);
					const FloatV torsionalFriction = FMax(minTorsionalPatchRadius, FSqrt(FMul(FMax(zero, FNeg(maxPenetration)), torsionalPatchRadius)));
					header->numFrictionConstr++;
					SolverContactFrictionStepExt* PX_RESTRICT f = reinterpret_cast<SolverContactFrictionStepExt*>(ptr);
					ptr += frictionStride;

					const Cm::SpatialVector resp0 = createImpulseResponseVector(PxVec3(0.f), header->normal, b0);
					const Cm::SpatialVector resp1 = createImpulseResponseVector(PxVec3(0.f), -header->normal, b1);

					Cm::SpatialVector deltaV0, deltaV1;

					PxReal ur = getImpulseResponse(b0, resp0, deltaV0, invMassScale0, invInertiaScale0,
						b1, resp1, deltaV1, invMassScale1, invInertiaScale1, false);

					FloatV resp = FLoad(ur);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FEps()), FDiv(p8, FAdd(cfm, resp)), zero);

					f->normalXYZ_ErrorW = V4Zero();
					f->raXnI_targetVelW = V4SetW(V3LoadA(resp0.angular), zero);
					f->rbXnI_velMultiplierW = V4SetW(V4Neg(Vec4V_From_Vec3V(V3LoadA(resp1.angular))), velMultiplier);
					f->biasScale = 0.f;
					f->appliedForce = 0.f;
					FStore(torsionalFriction, &f->frictionScale);
					f->linDeltaVA = V3LoadA(deltaV0.linear);
					f->linDeltaVB = V3LoadA(deltaV1.linear);
					f->angDeltaVA = V3LoadA(deltaV0.angular);
					f->angDeltaVB = V3LoadA(deltaV1.angular);

				}

			}

			frictionPatchWritebackAddrIndex++;
		}
	}




	bool createFinalizeSolverContactsStep(
		PxTGSSolverContactDesc& contactDesc,
		CorrelationBuffer& c,
		const PxReal invDtF32,
		const PxReal invTotalDtF32,
		const PxReal totalDtF32,
		const PxReal dtF32,
		const PxReal bounceThresholdF32,
		const PxReal frictionOffsetThreshold,
		const PxReal correlationDistance,
		const PxReal biasCoefficient,
		PxConstraintAllocator& constraintAllocator)
	{
		PxPrefetchLine(contactDesc.body0);
		PxPrefetchLine(contactDesc.body1);

		c.frictionPatchCount = 0;
		c.contactPatchCount = 0;

		const bool hasForceThreshold = contactDesc.hasForceThresholds;
		const bool staticOrKinematicBody = contactDesc.bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY || contactDesc.bodyState1 == PxSolverContactDesc::eSTATIC_BODY;

		const bool disableStrongFriction = contactDesc.disableStrongFriction;

		PxSolverConstraintDesc& desc = *contactDesc.desc;

		const bool useExtContacts = (desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY || desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY);

		desc.constraintLengthOver16 = 0;


		if (contactDesc.numContacts == 0)
		{
			contactDesc.frictionPtr = NULL;
			contactDesc.frictionCount = 0;
			desc.constraint = NULL;
			return true;
		}

		if (!disableStrongFriction)
		{
			getFrictionPatches(c, contactDesc.frictionPtr, contactDesc.frictionCount, contactDesc.bodyFrame0, contactDesc.bodyFrame1, correlationDistance);
		}

		bool overflow = !createContactPatches(c, contactDesc.contacts, contactDesc.numContacts, PXC_SAME_NORMAL);
		overflow = correlatePatches(c, contactDesc.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, PXC_SAME_NORMAL, 0, 0) || overflow;
		PX_UNUSED(overflow);

#if PX_CHECKED
		if (overflow)
		{
			PxGetFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
				"Dropping contacts in solver because we exceeded limit of 32 friction patches.");
		}
#endif

		growPatches(c, contactDesc.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, 0, frictionOffsetThreshold + contactDesc.restDistance);

		//PX_ASSERT(patchCount == c.frictionPatchCount);

		FrictionPatch* frictionPatches = NULL;
		PxU8* solverConstraint = NULL;
		PxU32 numFrictionPatches = 0;
		PxU32 solverConstraintByteSize = 0;
		PxU32 axisConstraintCount = 0;

		const bool successfulReserve = reserveBlockStreams(
			useExtContacts, c,
			solverConstraint, frictionPatches,
			numFrictionPatches,
			solverConstraintByteSize,
			axisConstraintCount,
			constraintAllocator,
			PxMax(contactDesc.torsionalPatchRadius, contactDesc.minTorsionalPatchRadius));
		// initialise the work unit's ptrs to the various buffers.

		contactDesc.frictionPtr = NULL;
		contactDesc.frictionCount = 0;
		desc.constraint = NULL;
		desc.constraintLengthOver16 = 0;
		// patch up the work unit with the reserved buffers and set the reserved buffer data as appropriate.

		if (successfulReserve)
		{
			PxU8* frictionDataPtr = reinterpret_cast<PxU8*>(frictionPatches);
			contactDesc.frictionPtr = frictionDataPtr;
			desc.constraint = solverConstraint;
			//output.nbContacts = PxTo8(numContacts);
			contactDesc.frictionCount = PxTo8(numFrictionPatches);
			PX_ASSERT((solverConstraintByteSize & 0xf) == 0);
			desc.constraintLengthOver16 = PxTo16(solverConstraintByteSize / 16);
			desc.writeBack = contactDesc.contactForces;

			//Initialise friction buffer.
			if (frictionPatches)
			{
				// PT: TODO: revisit this... not very satisfying
				//const PxU32 maxSize = numFrictionPatches*sizeof(FrictionPatch);
				PxPrefetchLine(frictionPatches);
				PxPrefetchLine(frictionPatches, 128);
				PxPrefetchLine(frictionPatches, 256);

				for (PxU32 i = 0; i<c.frictionPatchCount; i++)
				{
					//if(c.correlationListHeads[i]!=CorrelationBuffer::LIST_END)
					if (c.frictionPatchContactCounts[i])
					{
						*frictionPatches++ = c.frictionPatches[i];
						PxPrefetchLine(frictionPatches, 256);
					}
				}
			}

			//Initialise solverConstraint buffer.
			if (solverConstraint)
			{
				
				if (useExtContacts)
				{
					const SolverExtBodyStep b0(reinterpret_cast<const void*>(contactDesc.body0), contactDesc.body0TxI, contactDesc.bodyData0, desc.linkIndexA);
					const SolverExtBodyStep b1(reinterpret_cast<const void*>(contactDesc.body1), contactDesc.body1TxI, contactDesc.bodyData1, desc.linkIndexB);

					setupFinalizeExtSolverContactsStep(contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
						b0, b1, invDtF32, invTotalDtF32, totalDtF32, dtF32, bounceThresholdF32,
						contactDesc.invMassScales.linear0, contactDesc.invMassScales.angular0, contactDesc.invMassScales.linear1, contactDesc.invMassScales.angular1,
						contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, contactDesc.torsionalPatchRadius, contactDesc.minTorsionalPatchRadius,
						biasCoefficient, contactDesc.offsetSlop);
				}
				else
				{

					const PxTGSSolverBodyVel& b0 = *contactDesc.body0;
					const PxTGSSolverBodyVel& b1 = *contactDesc.body1;

					setupFinalizeSolverConstraints(getInteraction(contactDesc), contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
						b0, b1, *contactDesc.body0TxI, *contactDesc.body1TxI, *contactDesc.bodyData0, *contactDesc.bodyData1, invDtF32, totalDtF32, invTotalDtF32, dtF32, bounceThresholdF32,
						contactDesc.invMassScales.linear0, contactDesc.invMassScales.angular0, contactDesc.invMassScales.linear1, contactDesc.invMassScales.angular1,
						hasForceThreshold, staticOrKinematicBody, contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, disableStrongFriction,
						contactDesc.torsionalPatchRadius, contactDesc.minTorsionalPatchRadius, biasCoefficient,
						contactDesc.offsetSlop);
				}
				//KS - set to 0 so we have a counter for the number of times we solved the constraint
				//only going to be used on SPU but might as well set on all platforms because this code is shared
				*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
			}
		}

		return successfulReserve;
	}




	bool createFinalizeSolverContactsStep(PxTGSSolverContactDesc& contactDesc,
		PxsContactManagerOutput& output,
		ThreadContext& threadContext,
		const PxReal invDtF32,
		const PxReal invTotalDt,
		const PxReal totalDtF32,
		const PxReal dt,
		const PxReal bounceThresholdF32,
		const PxReal frictionOffsetThreshold,
		const PxReal correlationDistance,
		const PxReal biasCoefficient,
		PxConstraintAllocator& constraintAllocator)
	{
		PxContactBuffer& buffer = threadContext.mContactBuffer;

		buffer.count = 0;

		// We pull the friction patches out of the cache to remove the dependency on how
		// the cache is organized. Remember original addrs so we can write them back 
		// efficiently.

		PxU32 numContacts = 0;
		{
			PxReal invMassScale0 = 1.f;
			PxReal invMassScale1 = 1.f;
			PxReal invInertiaScale0 = 1.f;
			PxReal invInertiaScale1 = 1.f;
			contactDesc.invMassScales.angular0 = (contactDesc.bodyState0 != PxSolverContactDesc::eARTICULATION && contactDesc.body0->isKinematic) ? 0.f : contactDesc.invMassScales.angular0;
			contactDesc.invMassScales.angular1 = (contactDesc.bodyState1 != PxSolverContactDesc::eARTICULATION && contactDesc.body1->isKinematic) ? 0.f : contactDesc.invMassScales.angular1;

			bool hasMaxImpulse = false, hasTargetVelocity = false;

			numContacts = extractContacts(buffer, output, hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
				invInertiaScale0, invInertiaScale1, contactDesc.maxImpulse);

			contactDesc.contacts = buffer.contacts;
			contactDesc.numContacts = numContacts;
			contactDesc.disableStrongFriction = contactDesc.disableStrongFriction || hasTargetVelocity;
			contactDesc.hasMaxImpulse = hasMaxImpulse;
			contactDesc.invMassScales.linear0 *= invMassScale0;
			contactDesc.invMassScales.linear1 *= invMassScale1;
			contactDesc.invMassScales.angular0 *= invInertiaScale0;
			contactDesc.invMassScales.angular1 *= invInertiaScale1;
		}

		CorrelationBuffer& c = threadContext.mCorrelationBuffer;

		return createFinalizeSolverContactsStep(contactDesc, c, invDtF32, invTotalDt, totalDtF32, dt, bounceThresholdF32,
			frictionOffsetThreshold, correlationDistance, biasCoefficient, constraintAllocator);
	}


	static FloatV solveDynamicContactsStep(SolverContactPointStep* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
		const FloatVArg invMassA, const FloatVArg invMassB, Vec3V& linVel0_, Vec3V& angState0_,
		Vec3V& linVel1_, Vec3V& angState1_, PxF32* PX_RESTRICT forceBuffer,
		const Vec3V& angMotion0, const Vec3V& angMotion1,
		const Vec3V& linRelMotion, const FloatVArg maxPenBias,
		const FloatVArg angD0, const FloatVArg angD1, const FloatVArg minPen,
		const FloatVArg elapsedTime)
	{
		Vec3V linVel0 = linVel0_;
		Vec3V angState0 = angState0_;
		Vec3V linVel1 = linVel1_;
		Vec3V angState1 = angState1_;
		const FloatV zero = FZero();
		FloatV accumulatedNormalImpulse = zero;

		const Vec3V delLinVel0 = V3Scale(contactNormal, invMassA);
		const Vec3V delLinVel1 = V3Scale(contactNormal, invMassB);

		const FloatV deltaV = V3Dot(linRelMotion, contactNormal);

		for (PxU32 i = 0; i<nbContactPoints; i++)
		{
			SolverContactPointStep& c = contacts[i];
			PxPrefetchLine(&contacts[i], 128);


			const Vec3V raXnI = V3LoadA(c.raXnI);
			const Vec3V rbXnI = V3LoadA(c.rbXnI);

			const FloatV angDelta0 = V3Dot(angMotion0, raXnI);
			const FloatV angDelta1 = V3Dot(angMotion1, rbXnI);

			const FloatV deltaAng = FSub(angDelta0, angDelta1);

			const FloatV targetVel = FLoad(c.targetVelocity);

			//const FloatV tVelBias = FLoad(c.targetVelBias);

			const FloatV deltaBias = FSub(FAdd(deltaV, deltaAng), FMul(targetVel, elapsedTime));

			const FloatV biasCoefficient = FLoad(c.biasCoefficient);

			FloatV sep = FMax(minPen, FAdd(FLoad(c.separation), deltaBias));

			const FloatV bias = FMin(FNeg(maxPenBias), FMul(biasCoefficient, sep));

			const Vec3V v0 = V3MulAdd(linVel0, contactNormal, V3Mul(angState0, raXnI));
			const Vec3V v1 = V3MulAdd(linVel1, contactNormal, V3Mul(angState1, rbXnI));
			const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

			const FloatV velMultiplier = FLoad(c.velMultiplier);

			const FloatV biasNV = FMul(bias, FLoad(c.recipResponse));

			const FloatV tVelBias = FNegScaleSub(FSub(normalVel, targetVel), velMultiplier, biasNV);

			const FloatV appliedForce = FLoad(forceBuffer[i]);

			const FloatV maxImpulse = FLoad(c.maxImpulse); //KS - todo - hook up!

			//Compute the normal velocity of the constraint.
			
			const FloatV _deltaF = FMax(tVelBias, FNeg(appliedForce));
			const FloatV _newForce = FAdd(appliedForce, _deltaF);
			const FloatV newForce = FMin(_newForce, maxImpulse);
			const FloatV deltaF = FSub(newForce, appliedForce);

			linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
			linVel1 = V3NegScaleSub(delLinVel1, deltaF, linVel1);
			angState0 = V3ScaleAdd(raXnI, FMul(deltaF, angD0), angState0);
			angState1 = V3NegScaleSub(rbXnI, FMul(deltaF, angD1), angState1);

			FStore(newForce, &forceBuffer[i]);

			accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newForce);
		}

		linVel0_ = linVel0;
		angState0_ = angState0;
		linVel1_ = linVel1;
		angState1_ = angState1;
		return accumulatedNormalImpulse;
	}


	void solveContact(const PxSolverConstraintDesc& desc, bool doFriction, const PxReal minPenetration,
		const PxReal elapsedTimeF32)
	{
		PxTGSSolverBodyVel& b0 = *desc.tgsBodyA;
		PxTGSSolverBodyVel& b1 = *desc.tgsBodyB;

		const FloatV minPen = FLoad(minPenetration);

		Vec3V linVel0 = V3LoadA(b0.linearVelocity);
		Vec3V linVel1 = V3LoadA(b1.linearVelocity);
		Vec3V angState0 = V3LoadA(b0.angularVelocity);
		Vec3V angState1 = V3LoadA(b1.angularVelocity);

		const Vec3V angMotion0 = V3LoadA(b0.deltaAngDt);
		const Vec3V angMotion1 = V3LoadA(b1.deltaAngDt);

		const Vec3V linMotion0 = V3LoadA(b0.deltaLinDt);
		const Vec3V linMotion1 = V3LoadA(b1.deltaLinDt);

		const Vec3V relMotion = V3Sub(linMotion0, linMotion1);

		const FloatV zero = FZero();

		const FloatV elapsedTime = FLoad(elapsedTimeF32);

		const PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

		//hopefully pointer aliasing doesn't bite.
		PxU8* PX_RESTRICT currPtr = desc.constraint;

		while (currPtr < last)
		{
			SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);
			currPtr += sizeof(SolverContactHeaderStep);

			const PxU32 numNormalConstr = hdr->numNormalConstr;
			const PxU32	numFrictionConstr = hdr->numFrictionConstr;

			SolverContactPointStep* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStep*>(currPtr);
			PxPrefetchLine(contacts);
			currPtr += numNormalConstr * sizeof(SolverContactPointStep);

			PxF32* forceBuffer = reinterpret_cast<PxF32*>(currPtr);
			currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

			SolverContactFrictionStep* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStep*>(currPtr);
			currPtr += numFrictionConstr * sizeof(SolverContactFrictionStep);

			const FloatV invMassA = FLoad(hdr->invMass0);
			const FloatV invMassB = FLoad(hdr->invMass1);

			const FloatV angDom0 = FLoad(hdr->angDom0);
			const FloatV angDom1 = FLoad(hdr->angDom1);

			//const FloatV sumMass = FAdd(invMassA, invMassB);

			const Vec3V contactNormal = V3LoadA(hdr->normal);

			const FloatV maxPenBias = FLoad(hdr->maxPenBias);

			const FloatV accumulatedNormalImpulse = solveDynamicContactsStep(contacts, numNormalConstr, contactNormal, invMassA, invMassB,
				linVel0, angState0, linVel1, angState1, forceBuffer,angMotion0, angMotion1, relMotion, maxPenBias, angDom0, angDom1, minPen,
				elapsedTime);

			FStore(accumulatedNormalImpulse, &hdr->minNormalForce);

			if (numFrictionConstr && doFriction)
			{
				const FloatV staticFrictionCof = hdr->getStaticFriction();
				const FloatV dynamicFrictionCof = hdr->getDynamicFriction();
				const FloatV maxFrictionImpulse = FMul(staticFrictionCof, accumulatedNormalImpulse);
				const FloatV maxDynFrictionImpulse = FMul(dynamicFrictionCof, accumulatedNormalImpulse);
				const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);

				BoolV broken = BFFFF();

				//We will have either 4 or 2 frictions (with friction pairs).
				//With torsional friction, we may have 3 (a single friction anchor + twist).
				const PxU32 numFrictionPairs = (numFrictionConstr&6);

				for (PxU32 i = 0; i<numFrictionPairs; i += 2)
				{
					SolverContactFrictionStep& f0 = frictions[i];
					SolverContactFrictionStep& f1 = frictions[i + 1];
					PxPrefetchLine(&frictions[i + 2], 128);

					const FloatV frictionScale = FLoad(f0.frictionScale);


					const Vec4V normalXYZ_ErrorW0 = f0.normalXYZ_ErrorW;
					const Vec4V raXnI_targetVelW0 = f0.raXnI_targetVelW;
					const Vec4V rbXnI_velMultiplierW0 = f0.rbXnI_velMultiplierW;

					const Vec4V normalXYZ_ErrorW1 = f1.normalXYZ_ErrorW;
					const Vec4V raXnI_targetVelW1 = f1.raXnI_targetVelW;
					const Vec4V rbXnI_velMultiplierW1 = f1.rbXnI_velMultiplierW;

					const Vec3V normal0 = Vec3V_From_Vec4V(normalXYZ_ErrorW0);
					const Vec3V normal1 = Vec3V_From_Vec4V(normalXYZ_ErrorW1);

					const FloatV initialError0 = V4GetW(normalXYZ_ErrorW0);
					const FloatV initialError1 = V4GetW(normalXYZ_ErrorW1);

					const FloatV biasScale = FLoad(f0.biasScale); //Bias scale will be common

					const Vec3V raXnI0 = Vec3V_From_Vec4V(raXnI_targetVelW0);
					const Vec3V rbXnI0 = Vec3V_From_Vec4V(rbXnI_velMultiplierW0);

					const Vec3V raXnI1 = Vec3V_From_Vec4V(raXnI_targetVelW1);
					const Vec3V rbXnI1 = Vec3V_From_Vec4V(rbXnI_velMultiplierW1);

					const FloatV appliedForce0 = FLoad(f0.appliedForce);
					const FloatV appliedForce1 = FLoad(f1.appliedForce);

					const FloatV targetVel0 = V4GetW(raXnI_targetVelW0);
					const FloatV targetVel1 = V4GetW(raXnI_targetVelW1);

					FloatV deltaV0 = FAdd(FSub(V3Dot(raXnI0, angMotion0), V3Dot(rbXnI0, angMotion1)), V3Dot(normal0, relMotion));
					FloatV deltaV1 = FAdd(FSub(V3Dot(raXnI1, angMotion0), V3Dot(rbXnI1, angMotion1)), V3Dot(normal1, relMotion));

					deltaV0 = FSub(deltaV0, FMul(targetVel0, elapsedTime));
					deltaV1 = FSub(deltaV1, FMul(targetVel1, elapsedTime));

					const FloatV error0 = FAdd(initialError0, deltaV0);
					const FloatV error1 = FAdd(initialError1, deltaV1);

					const FloatV bias0 = FMul(error0, biasScale);
					const FloatV bias1 = FMul(error1, biasScale);

					const FloatV velMultiplier0 = V4GetW(rbXnI_velMultiplierW0);
					const FloatV velMultiplier1 = V4GetW(rbXnI_velMultiplierW1);

					const Vec3V delLinVel00 = V3Scale(normal0, invMassA);
					const Vec3V delLinVel10 = V3Scale(normal0, invMassB);
					const Vec3V delLinVel01 = V3Scale(normal1, invMassA);
					const Vec3V delLinVel11 = V3Scale(normal1, invMassB);

					const Vec3V v00 = V3MulAdd(linVel0, normal0, V3Mul(angState0, raXnI0));
					const Vec3V v10 = V3MulAdd(linVel1, normal0, V3Mul(angState1, rbXnI0));
					const FloatV normalVel0 = V3SumElems(V3Sub(v00, v10));

					const Vec3V v01 = V3MulAdd(linVel0, normal1, V3Mul(angState0, raXnI1));
					const Vec3V v11 = V3MulAdd(linVel1, normal1, V3Mul(angState1, rbXnI1));
					const FloatV normalVel1 = V3SumElems(V3Sub(v01, v11));



					// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
					const FloatV tmp10 = FNegScaleSub(FSub(bias0, targetVel0), velMultiplier0, appliedForce0);
					const FloatV tmp11 = FNegScaleSub(FSub(bias1, targetVel1), velMultiplier1, appliedForce1);

					// Algorithm:
					// if abs(appliedForce + deltaF) > maxFrictionImpulse
					//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
					//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
					//    set broken flag to true || broken flag

					// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
					// FloatV potentialSumF = FAdd(appliedForce, deltaF);

					const FloatV totalImpulse0 = FNegScaleSub(normalVel0, velMultiplier0, tmp10);
					const FloatV totalImpulse1 = FNegScaleSub(normalVel1, velMultiplier1, tmp11);

					// On XBox this clamping code uses the vector simple pipe rather than vector float,
					// which eliminates a lot of stall cycles

					const FloatV totalImpulse = FSqrt(FAdd(FMul(totalImpulse0, totalImpulse0), FMul(totalImpulse1, totalImpulse1)));

					const BoolV clamp = FIsGrtr(totalImpulse, FMul(frictionScale, maxFrictionImpulse));

					const FloatV totalClamped = FSel(clamp, FMin(FMul(frictionScale, maxDynFrictionImpulse), totalImpulse), totalImpulse);

					const FloatV ratio = FSel(FIsGrtr(totalImpulse, zero), FDiv(totalClamped, totalImpulse), zero);

					const FloatV newAppliedForce0 = FMul(totalImpulse0, ratio);
					const FloatV newAppliedForce1 = FMul(totalImpulse1, ratio);

					broken = BOr(broken, clamp);

					FloatV deltaF0 = FSub(newAppliedForce0, appliedForce0);
					FloatV deltaF1 = FSub(newAppliedForce1, appliedForce1);

					// we could get rid of the stall here by calculating and clamping delta separately, but
					// the complexity isn't really worth it.

					linVel0 = V3ScaleAdd(delLinVel00, deltaF0, V3ScaleAdd(delLinVel01, deltaF1, linVel0));
					linVel1 = V3NegScaleSub(delLinVel10, deltaF0, V3NegScaleSub(delLinVel11, deltaF1, linVel1));
					angState0 = V3ScaleAdd(raXnI0, FMul(deltaF0, angDom0), V3ScaleAdd(raXnI1, FMul(deltaF1, angDom0), angState0));
					angState1 = V3NegScaleSub(rbXnI0, FMul(deltaF0, angDom1), V3NegScaleSub(rbXnI1, FMul(deltaF1, angDom1), angState1));

					f0.setAppliedForce(newAppliedForce0);
					f1.setAppliedForce(newAppliedForce1);
				}

				for (PxU32 i = numFrictionPairs; i<numFrictionConstr; i++)
				{
					SolverContactFrictionStep& f = frictions[i];

					const FloatV frictionScale = FLoad(f.frictionScale);


					const Vec4V raXnI_targetVelW = f.raXnI_targetVelW;
					const Vec4V rbXnI_velMultiplierW = f.rbXnI_velMultiplierW;

					const Vec3V raXnI = Vec3V_From_Vec4V(raXnI_targetVelW);
					const Vec3V rbXnI = Vec3V_From_Vec4V(rbXnI_velMultiplierW);

					const FloatV appliedForce = FLoad(f.appliedForce);

					const FloatV targetVel = V4GetW(raXnI_targetVelW);
					
					const FloatV velMultiplier = V4GetW(rbXnI_velMultiplierW);

					const Vec3V v0 =V3Mul(angState0, raXnI);
					const Vec3V v1 =V3Mul(angState1, rbXnI);
					const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

					// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
					const FloatV tmp1 = FNegScaleSub(FNeg(targetVel), velMultiplier, appliedForce);

					// Algorithm:
					// if abs(appliedForce + deltaF) > maxFrictionImpulse
					//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
					//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
					//    set broken flag to true || broken flag

					// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
					// FloatV potentialSumF = FAdd(appliedForce, deltaF);

					const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

					// On XBox this clamping code uses the vector simple pipe rather than vector float,
					// which eliminates a lot of stall cycles

					const BoolV clamp = FIsGrtr(FAbs(totalImpulse), FMul(frictionScale, maxFrictionImpulse));

					const FloatV totalClamped = FMin(FMul(frictionScale, maxDynFrictionImpulse), FMax(FMul(frictionScale, negMaxDynFrictionImpulse), totalImpulse));

					const FloatV newAppliedForce = FSel(clamp, totalClamped, totalImpulse);

					broken = BOr(broken, clamp);

					FloatV deltaF = FSub(newAppliedForce, appliedForce);

					// we could get rid of the stall here by calculating and clamping delta separately, but
					// the complexity isn't really worth it.

					angState0 = V3ScaleAdd(raXnI, FMul(deltaF, angDom0), angState0);
					angState1 = V3NegScaleSub(rbXnI, FMul(deltaF, angDom1), angState1);

					f.setAppliedForce(newAppliedForce);


				}
				Store_From_BoolV(broken, &hdr->broken);
			}

		}

		PX_ASSERT(b0.linearVelocity.isFinite());
		PX_ASSERT(b0.angularVelocity.isFinite());
		PX_ASSERT(b1.linearVelocity.isFinite());
		PX_ASSERT(b1.angularVelocity.isFinite());

		// Write back
		V3StoreA(linVel0, b0.linearVelocity);
		V3StoreA(linVel1, b1.linearVelocity);
		V3StoreA(angState0, b0.angularVelocity);
		V3StoreA(angState1, b1.angularVelocity);

		PX_ASSERT(b0.linearVelocity.isFinite());
		PX_ASSERT(b0.angularVelocity.isFinite());
		PX_ASSERT(b1.linearVelocity.isFinite());
		PX_ASSERT(b1.angularVelocity.isFinite());

		PX_ASSERT(currPtr == last);
	}

	void writeBackContact(const PxSolverConstraintDesc& desc, SolverContext* cache)
	{
		PX_UNUSED(cache);
		// PxReal normalForce = 0;

		PxU8* PX_RESTRICT cPtr = desc.constraint;
		PxReal* PX_RESTRICT vForceWriteback = reinterpret_cast<PxReal*>(desc.writeBack);
		PxU8* PX_RESTRICT last = desc.constraint + desc.constraintLengthOver16 * 16;

		bool forceThreshold = false;

		while (cPtr < last)
		{
			const SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<const SolverContactHeaderStep*>(cPtr);
			cPtr += sizeof(SolverContactHeaderStep);

			forceThreshold = hdr->flags & SolverContactHeaderStep::eHAS_FORCE_THRESHOLDS;
			const PxU32 numNormalConstr = hdr->numNormalConstr;
			const PxU32	numFrictionConstr = hdr->numFrictionConstr;

			//if(cPtr < last)
			PxPrefetchLine(cPtr, 256);
			PxPrefetchLine(cPtr, 384);

			const PxU32 pointStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactPointStepExt)
				: sizeof(SolverContactPointStep);

			cPtr += pointStride * numNormalConstr;
			PxF32* forceBuffer = reinterpret_cast<PxF32*>(cPtr);
			cPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

			if (vForceWriteback != NULL)
			{
				for (PxU32 i = 0; i<numNormalConstr; i++)
				{
					PxReal appliedForce = forceBuffer[i];
					*vForceWriteback++ = appliedForce;
					// normalForce += appliedForce;
				}
			}

			const PxU32 frictionStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactFrictionStepExt)
				: sizeof(SolverContactFrictionStep);

			if (hdr->broken && hdr->frictionBrokenWritebackByte != NULL)
			{
				*hdr->frictionBrokenWritebackByte = 1;
			}

			cPtr += frictionStride * numFrictionConstr;

		}
		PX_ASSERT(cPtr == last);

		PX_UNUSED(forceThreshold);

#if 0
		if (cache && forceThreshold && desc.linkIndexA == PxSolverConstraintDesc::NO_LINK && desc.linkIndexB == PxSolverConstraintDesc::NO_LINK &&
		normalForce != 0 && (desc.bodyA->reportThreshold < PX_MAX_REAL || desc.bodyB->reportThreshold < PX_MAX_REAL))
		{
			ThresholdStreamElement elt;
			elt.normalForce = normalForce;
			elt.threshold = PxMin<float>(desc.bodyA->reportThreshold, desc.bodyB->reportThreshold);
			elt.nodeIndexA = desc.bodyA->nodeIndex;
			elt.nodeIndexB = desc.bodyB->nodeIndex;
			elt.shapeInteraction = reinterpret_cast<const SolverContactHeader*>(desc.constraint)->shapeInteraction;
			PxOrder(elt.nodeIndexA, elt.nodeIndexB);
			PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);
			PX_ASSERT(cache->mThresholdStreamIndex<cache->mThresholdStreamLength);
			cache->mThresholdStream[cache->mThresholdStreamIndex++] = elt;
		}
#endif
	}

	PX_FORCE_INLINE Vec3V V3FromV4(Vec4V x)			{ return Vec3V_From_Vec4V(x); }
	PX_FORCE_INLINE Vec3V V3FromV4Unsafe(Vec4V x)	{ return Vec3V_From_Vec4V_WUndefined(x); }
	PX_FORCE_INLINE Vec4V V4FromV3(Vec3V x)			{ return Vec4V_From_Vec3V(x); }
	//PX_FORCE_INLINE Vec4V V4ClearW(Vec4V x)			{ return V4SetW(x, FZero()); }

void setSolverConstantsStep(PxReal& error,
	PxReal& biasScale,
	PxReal& targetVel,
	PxReal& maxBias,
	PxReal& velMultiplier,
	PxReal& rcpResponse,
	const Px1DConstraint& c,
	PxReal normalVel,
	PxReal unitResponse,
	PxReal minRowResponse,
	PxReal erp,
	PxReal dt,
	PxReal totalDt,
	PxReal biasClamp,
	PxReal recipdt,
	PxReal recipTotalDt,
	PxReal velTarget)
{
	PX_UNUSED(dt);
	PX_UNUSED(totalDt);
	PX_UNUSED(minRowResponse);
	PX_ASSERT(PxIsFinite(unitResponse));
	PxReal recipResponse = unitResponse <= minRowResponse ? 0 : 1.0f / unitResponse;
	//PX_ASSERT(recipResponse < 1e5f);  
	PxReal geomError = c.geometricError;

	rcpResponse = recipResponse;

	

	if (c.flags & Px1DConstraintFlag::eSPRING)
	{
		PxReal a = dt * (dt*c.mods.spring.stiffness + c.mods.spring.damping);
		PxReal aDamp = dt * dt * (c.mods.spring.damping + c.mods.spring.stiffness);
		PxReal b = dt * (c.mods.spring.damping * (c.velocityTarget));// - c.mods.spring.stiffness * geomError);
		maxBias = PX_MAX_F32;
		PxReal errorTerm;

		if (c.flags & Px1DConstraintFlag::eACCELERATION_SPRING)
		{
			const PxReal x = 1.0f / (1.0f + a);
			const PxReal xDamp = 1.0f / (1.0f + aDamp);
			targetVel = x * b;
			velMultiplier = -x * a;
			errorTerm = -x * c.mods.spring.stiffness * dt;
			biasScale = errorTerm - xDamp*c.mods.spring.damping*dt;
		}
		else
		{
			const PxReal x = 1.0f / (1.0f + a*unitResponse);
			const PxReal xDamp = 1.0f / (1.0f + aDamp*unitResponse);
			targetVel = x * b*unitResponse;
			velMultiplier = -x * a*unitResponse;			
			errorTerm = -x * c.mods.spring.stiffness * unitResponse * dt;
			biasScale = errorTerm - xDamp*c.mods.spring.damping*unitResponse*dt;
		}

		error = geomError * errorTerm;
		
	}
	else
	{
		velMultiplier = -1.f;

		if (c.flags & Px1DConstraintFlag::eRESTITUTION && -normalVel>c.mods.bounce.velocityThreshold)
		{
			error = 0.f;
			biasScale = 0.f;

			targetVel = c.mods.bounce.restitution*-normalVel;
			maxBias = 0.f;
		}
		else
		{
			
			biasScale = -recipdt*erp;// *recipResponse;
			if (c.flags & Px1DConstraintFlag::eDRIVE_ROW)
			{
				error = 0.f;
				targetVel = c.velocityTarget - geomError *recipTotalDt;
			}
			else
			{
				error = geomError * biasScale;
				//KS - if there is a velocity target, then we cannot also have bias otherwise the two compete against each-other.
				//Therefore, we set the velocity target 
				targetVel = c.velocityTarget;// *recipResponse;
			}

			maxBias = biasClamp;// *recipResponse;

			/*PxReal errorBias = PxClamp(geomError*erp*recipdt, -biasClamp, biasClamp);

			constant = (c.velocityTarget - errorBias) * recipResponse;*/

		}
	}
	targetVel -= velMultiplier * velTarget;
}



PxU32 setupSolverConstraintStep(
	const PxTGSSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt,
	const PxReal lengthScale, const PxReal biasCoefficient)
{

	if (prepDesc.numRows == 0)
	{
		prepDesc.desc->constraint = NULL;
		prepDesc.desc->writeBack = NULL;
		prepDesc.desc->constraintLengthOver16 = 0;
		return 0;
	}

	PxSolverConstraintDesc& desc = *prepDesc.desc;

	const bool isExtended = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY
		|| desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY;

	const bool isKinematic0 = desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY && 
		desc.tgsBodyA->isKinematic;

	const bool isKinematic1 = desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY &&
		desc.tgsBodyB->isKinematic;

	PxU32 stride = isExtended ? sizeof(SolverConstraint1DExtStep) : sizeof(SolverConstraint1DStep);
	const PxU32 constraintLength = sizeof(SolverConstraint1DHeaderStep) + stride * prepDesc.numRows;

	//KS - +16 is for the constraint progress counter, which needs to be the last element in the constraint (so that we
	//know SPU DMAs have completed)
	PxU8* ptr = allocator.reserveConstraintData(constraintLength + 16u);
	if (NULL == ptr || (reinterpret_cast<PxU8*>(-1)) == ptr)
	{
		if (NULL == ptr)
		{
			PX_WARN_ONCE(
				"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
				"Either accept joints detaching/exploding or increase buffer size allocated for constraint prep by increasing PxSceneDesc::maxNbContactDataBlocks.");
			return 0;
		}
		else
		{
			PX_WARN_ONCE(
				"Attempting to allocate more than 16K of constraint data. "
				"Either accept joints detaching/exploding or simplify constraints.");
			ptr = NULL;
			return 0;
		}
	}
	desc.constraint = ptr;

	//setConstraintLength(desc, constraintLength);
	PX_ASSERT((constraintLength & 0xf) == 0);
	desc.constraintLengthOver16 = PxTo16(constraintLength / 16);

	desc.writeBack = prepDesc.writeback;

	PxMemSet(desc.constraint, 0, constraintLength);

	SolverConstraint1DHeaderStep* header = reinterpret_cast<SolverConstraint1DHeaderStep*>(desc.constraint);
	PxU8* constraints = desc.constraint + sizeof(SolverConstraint1DHeaderStep);
	init(*header, PxTo8(prepDesc.numRows), isExtended, prepDesc.invMassScales);
	header->body0WorldOffset = prepDesc.body0WorldOffset;
	header->linBreakImpulse = prepDesc.linBreakForce * totalDt;
	header->angBreakImpulse = prepDesc.angBreakForce * totalDt;
	header->breakable = PxU8((prepDesc.linBreakForce != PX_MAX_F32) || (prepDesc.angBreakForce != PX_MAX_F32));
	header->invMass0D0 = prepDesc.bodyData0->invMass * prepDesc.invMassScales.linear0;
	header->invMass1D1 = prepDesc.bodyData1->invMass * prepDesc.invMassScales.linear1;

	header->rAWorld = prepDesc.cA2w - prepDesc.bodyFrame0.p;
	header->rBWorld = prepDesc.cB2w - prepDesc.bodyFrame1.p;

	/*printf("prepDesc.cA2w.p = (%f, %f, %f), prepDesc.cB2w.p = (%f, %f, %f)\n",
		prepDesc.cA2w.x, prepDesc.cA2w.y, prepDesc.cA2w.z,
		prepDesc.cB2w.x, prepDesc.cB2w.y, prepDesc.cB2w.z);*/

	Px1DConstraint* sorted[MAX_CONSTRAINT_ROWS];

	PX_ALIGN(16, PxVec4) angSqrtInvInertia0[MAX_CONSTRAINT_ROWS];
	PX_ALIGN(16, PxVec4) angSqrtInvInertia1[MAX_CONSTRAINT_ROWS];

	for (PxU32 i = 0; i < prepDesc.numRows; ++i)
	{
		if (prepDesc.rows[i].flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT)
		{
			if (prepDesc.rows[i].solveHint == PxConstraintSolveHint::eEQUALITY)
				prepDesc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_EQUALITY;
			else if (prepDesc.rows[i].solveHint == PxConstraintSolveHint::eINEQUALITY)
				prepDesc.rows[i].solveHint = PxConstraintSolveHint::eROTATIONAL_INEQUALITY;
		}
	}

	preprocessRows(sorted, prepDesc.rows, angSqrtInvInertia0, angSqrtInvInertia1, prepDesc.numRows,
		prepDesc.body0TxI->sqrtInvInertia, prepDesc.body1TxI->sqrtInvInertia, prepDesc.bodyData0->invMass, prepDesc.bodyData1->invMass,
		prepDesc.invMassScales, isExtended || prepDesc.disablePreprocessing, prepDesc.improvedSlerp);

	PxReal erp = 0.5f * biasCoefficient;

	const PxReal recipDt = invdt;

	const PxReal angSpeedLimit = invTotalDt*0.75f;
	const PxReal linSpeedLimit = isExtended ? invTotalDt*1.5f*lengthScale : invTotalDt*15.f*lengthScale;

	PxU32 orthoCount = 0;
	PxU32 outCount = 0;

	const SolverExtBodyStep eb0(reinterpret_cast<const void*>(prepDesc.body0), prepDesc.body0TxI, prepDesc.bodyData0, desc.linkIndexA);
	const SolverExtBodyStep eb1(reinterpret_cast<const void*>(prepDesc.body1), prepDesc.body1TxI, prepDesc.bodyData1, desc.linkIndexB);

	PxReal cfm = 0.f;
	if (isExtended)
	{
		cfm = PxMax(eb0.getCFM(), eb1.getCFM());
	}

	for (PxU32 i = 0; i<prepDesc.numRows; i++)
	{
		PxPrefetchLine(constraints, 128);
		SolverConstraint1DStep &s = *reinterpret_cast<SolverConstraint1DStep *>(constraints);
		Px1DConstraint& c = *sorted[i];

		PxReal driveScale = c.flags&Px1DConstraintFlag::eHAS_DRIVE_LIMIT && prepDesc.driveLimitsAreForces ? PxMin(totalDt, 1.0f) : 1.0f;

		PxReal unitResponse;
		PxReal normalVel = 0.0f;

		PxReal vel0, vel1;

		if (!isExtended)
		{		
			const PxVec3 angSqrtInvInertia0V3(angSqrtInvInertia0[i].x, angSqrtInvInertia0[i].y, angSqrtInvInertia0[i].z);
			const PxVec3 angSqrtInvInertia1V3(angSqrtInvInertia1[i].x, angSqrtInvInertia1[i].y, angSqrtInvInertia1[i].z);
			init(s, c.linear0, c.linear1, c.angular0, c.angular1, c.minImpulse * driveScale, c.maxImpulse * driveScale);

			/*unitResponse = prepDesc.body0->getResponse(c.linear0, c.angular0, s.ang0, prepDesc.mInvMassScales.linear0, prepDesc.mInvMassScales.angular0)
				+ prepDesc.body1->getResponse(-c.linear1, -c.angular1, s.ang1, prepDesc.mInvMassScales.linear1, prepDesc.mInvMassScales.angular1);*/

			const PxReal linSumMass = s.lin0.magnitudeSquared() * prepDesc.bodyData0->invMass * prepDesc.invMassScales.linear0 + s.lin1.magnitudeSquared() * prepDesc.bodyData1->invMass * prepDesc.invMassScales.linear1;

			PxReal resp0 = angSqrtInvInertia0V3.magnitudeSquared() * prepDesc.invMassScales.angular0;
			PxReal resp1 = angSqrtInvInertia1V3.magnitudeSquared() * prepDesc.invMassScales.angular1;
			unitResponse = resp0 + resp1 + linSumMass;

			//s.recipResponseOrLinearSumMass = linSumMass;

			vel0 = prepDesc.bodyData0->projectVelocity(s.lin0, s.ang0);
			vel1 = prepDesc.bodyData1->projectVelocity(s.lin1, s.ang1);

			normalVel = vel0 - vel1;

			//if (c.solveHint & PxConstraintSolveHint::eEQUALITY)
			if(!(c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT))
			{
				s.ang0 = PxVec3(0.f);
				s.ang1 = PxVec3(0.f);
				s.angularErrorScale = 0.f;
			}
			
		}
		else
		{
			const Cm::SpatialVector resp0 = createImpulseResponseVector(c.linear0, c.angular0, eb0);
			const Cm::SpatialVector resp1 = createImpulseResponseVector(-c.linear1, -c.angular1, eb1);

			init(s, resp0.linear, -resp1.linear, resp0.angular, -resp1.angular, c.minImpulse * driveScale, c.maxImpulse * driveScale);
			SolverConstraint1DExtStep& e = static_cast<SolverConstraint1DExtStep&>(s);

			Cm::SpatialVector& delta0 = unsimdRef(e.deltaVA);
			Cm::SpatialVector& delta1 = unsimdRef(e.deltaVB);

			unitResponse = getImpulseResponse(eb0, resp0, delta0, prepDesc.invMassScales.linear0, prepDesc.invMassScales.angular0,
				eb1, resp1, delta1, prepDesc.invMassScales.linear1, prepDesc.invMassScales.angular1, false);

			//PxReal totalVelChange = (delta0 - delta1).magnitude();
			//PX_UNUSED(totalVelChange);

			if (unitResponse < DY_ARTICULATION_MIN_RESPONSE)
			{
				continue;
			}
			else
				unitResponse += cfm;


		
			{
				vel0 = eb0.projectVelocity(s.lin0, s.ang0);
				vel1 = eb1.projectVelocity(s.lin1, s.ang1);

				normalVel = vel0 - vel1;
			}

			if (!(c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT))
			{
				s.angularErrorScale = 0.f;
			}
		}

		PxReal recipResponse = 0.f;

		PxReal velTarget = 0.f;
		if (isKinematic0)
			velTarget -= vel0;
		if (isKinematic1)
			velTarget += vel1;

		setSolverConstantsStep(s.error, s.biasScale, s.velTarget, s.maxBias, s.velMultiplier, recipResponse, c,
			normalVel, unitResponse, prepDesc.minResponseThreshold, erp, dt, totalDt,
			c.flags & Px1DConstraintFlag::eANGULAR_CONSTRAINT ? angSpeedLimit : linSpeedLimit, recipDt, invTotalDt,
			velTarget);

		s.recipResponse = recipResponse;

		

		

		if (c.flags & Px1DConstraintFlag::eOUTPUT_FORCE)
			s.flags |= DY_SC_FLAG_OUTPUT_FORCE;

		if ((c.flags & Px1DConstraintFlag::eKEEPBIAS))
			s.flags |= DY_SC_FLAG_KEEP_BIAS;
		if (c.solveHint & 1)
			s.flags |= DY_SC_FLAG_INEQUALITY;

		if (!(isExtended || prepDesc.disablePreprocessing))
		{
			//KS - the code that orthogonalizes constraints on-the-fly only works if the linear and angular constraints have already been pre-orthogonalized
			if (c.solveHint == PxConstraintSolveHint::eROTATIONAL_EQUALITY)
			{
				s.flags |= DY_SC_FLAG_ROT_EQ;

				PX_ASSERT(orthoCount < 3);

				/*angOrtho0[orthoCount] = PxVec3(angSqrtInvInertia0[i].x, angSqrtInvInertia0[i].y, angSqrtInvInertia0[i].z);
				angOrtho1[orthoCount] = PxVec3(angSqrtInvInertia1[i].x, angSqrtInvInertia1[i].y, angSqrtInvInertia1[i].z);
				recipResponses[orthoCount] = recipResponse;*/

				header->angOrthoAxis0_recipResponseW[orthoCount] =
					PxVec4(angSqrtInvInertia0[i].x*prepDesc.invMassScales.angular0, angSqrtInvInertia0[i].y*prepDesc.invMassScales.angular0, angSqrtInvInertia0[i].z*prepDesc.invMassScales.angular0, recipResponse);
				header->angOrthoAxis1_Error[orthoCount].x = angSqrtInvInertia1[i].x*prepDesc.invMassScales.angular1;
				header->angOrthoAxis1_Error[orthoCount].y = angSqrtInvInertia1[i].y*prepDesc.invMassScales.angular1;
				header->angOrthoAxis1_Error[orthoCount].z = angSqrtInvInertia1[i].z*prepDesc.invMassScales.angular1;
				header->angOrthoAxis1_Error[orthoCount].w = c.geometricError;

				orthoCount++;
			}

			else if (c.solveHint & PxConstraintSolveHint::eEQUALITY)
				s.flags |= DY_SC_FLAG_ORTHO_TARGET;
		}
		



		constraints += stride;
		outCount++;
	}


	//KS - we now need to re-set count because we may have skipped degenerate rows when solving articulation constraints.
	//In this case, the degenerate rows would have produced no force. Skipping them is just an optimization
	header->count = PxU8(outCount);
	return prepDesc.numRows;
}


PxU32 SetupSolverConstraintStep(SolverConstraintShaderPrepDesc& shaderDesc,
	PxTGSSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt,
	const PxReal lengthScale, const PxReal biasCoefficient)
{
	// LL shouldn't see broken constraints

	PX_ASSERT(!(reinterpret_cast<ConstraintWriteback*>(prepDesc.writeback)->broken));

	prepDesc.desc->constraintLengthOver16 = 0;
	//setConstraintLength(*prepDesc.desc, 0);

	if (!shaderDesc.solverPrep)
		return 0;

	//PxU32 numAxisConstraints = 0;

	Px1DConstraint rows[MAX_CONSTRAINT_ROWS];
	setupConstraintRows(rows, MAX_CONSTRAINT_ROWS);

	prepDesc.invMassScales.linear0 = prepDesc.invMassScales.linear1 = prepDesc.invMassScales.angular0 = prepDesc.invMassScales.angular1 = 1.f;
	prepDesc.body0WorldOffset = PxVec3(0.0f);

	//TAG:solverprepcall
	prepDesc.numRows = prepDesc.disableConstraint ? 0 : (*shaderDesc.solverPrep)(rows,
		prepDesc.body0WorldOffset,
		MAX_CONSTRAINT_ROWS,
		prepDesc.invMassScales,
		shaderDesc.constantBlock,
		prepDesc.bodyFrame0, prepDesc.bodyFrame1,
		prepDesc.extendedLimits, prepDesc.cA2w, prepDesc.cB2w);

	prepDesc.rows = rows;

	if (prepDesc.bodyState0 != PxSolverContactDesc::eARTICULATION && prepDesc.body0->isKinematic)
		prepDesc.invMassScales.angular0 = 0.f;
	if (prepDesc.bodyState1 != PxSolverContactDesc::eARTICULATION && prepDesc.body1->isKinematic)
		prepDesc.invMassScales.angular1 = 0.f;

	return setupSolverConstraintStep(prepDesc, allocator, dt, totalDt, invdt, invTotalDt, lengthScale, biasCoefficient);
}

void solveExt1D(const PxSolverConstraintDesc& desc, Vec3V& linVel0, Vec3V& linVel1, Vec3V& angVel0, Vec3V& angVel1,
	const Vec3V& linMotion0, const Vec3V& linMotion1, const Vec3V& angMotion0, const Vec3V& angMotion1,
	const QuatV& rotA, const QuatV& rotB, const PxReal elapsedTimeF32, Vec3V& linImpulse0, Vec3V& linImpulse1, Vec3V& angImpulse0,
	Vec3V& angImpulse1)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;

	const SolverConstraint1DHeaderStep* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep*>(bPtr);
	SolverConstraint1DExtStep* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DExtStep*>(bPtr + sizeof(SolverConstraint1DHeaderStep));

	const FloatV elapsedTime = FLoad(elapsedTimeF32);

	const Vec3V raPrev = V3LoadA(header->rAWorld);
	const Vec3V rbPrev = V3LoadA(header->rBWorld);

	const Vec3V ra = QuatRotate(rotA, V3LoadA(header->rAWorld));
	const Vec3V rb = QuatRotate(rotB, V3LoadA(header->rBWorld));

	const Vec3V raMotion = V3Sub(V3Add(ra, linMotion0), raPrev);
	const Vec3V rbMotion = V3Sub(V3Add(rb, linMotion1), rbPrev);

	Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

	for (PxU32 i = 0; i<header->count; ++i, base++)
	{
		PxPrefetchLine(base + 1);

		SolverConstraint1DExtStep& c = *base;

		const Vec3V clinVel0 = V3LoadA(c.lin0);
		const Vec3V clinVel1 = V3LoadA(c.lin1);

		const Vec3V cangVel0 = V3LoadA(c.ang0);
		const Vec3V cangVel1 = V3LoadA(c.ang1);

		const FloatV recipResponse = FLoad(c.recipResponse);

		const FloatV targetVel = FLoad(c.velTarget);

		const FloatV deltaAng = FMul(FSub(V3Dot(cangVel0, angMotion0), V3Dot(cangVel1, angMotion1)), FLoad(c.angularErrorScale));
		const FloatV errorChange = FNegScaleSub(targetVel, elapsedTime, FAdd(FSub(V3Dot(raMotion, clinVel0), V3Dot(rbMotion, clinVel1)), deltaAng));

		const FloatV biasScale = FLoad(c.biasScale);
		const FloatV maxBias = FLoad(c.maxBias);

		const FloatV vMul = FMul(recipResponse, FLoad(c.velMultiplier));
		const FloatV appliedForce = FLoad(c.appliedForce);

		const FloatV unclampedBias = FScaleAdd(errorChange, biasScale, FLoad(c.error));
		const FloatV minBias = c.flags & DY_SC_FLAG_INEQUALITY ? FNeg(FMax()) : FNeg(maxBias);
		const FloatV bias = FClamp(unclampedBias, minBias, maxBias);

		const FloatV constant = FMul(recipResponse, FAdd(bias, targetVel));

		const FloatV maxImpulse = FLoad(c.maxImpulse);
		const FloatV minImpulse = FLoad(c.minImpulse);

		const Vec3V v0 = V3MulAdd(linVel0, clinVel0, V3Mul(angVel0, cangVel0));
		const Vec3V v1 = V3MulAdd(linVel1, clinVel1, V3Mul(angVel1, cangVel1));
		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

		const FloatV unclampedForce = FAdd(appliedForce, FScaleAdd(vMul, normalVel, constant));
		const FloatV clampedForce = FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const FloatV deltaF = FSub(clampedForce, appliedForce);

		FStore(clampedForce, &c.appliedForce);

		//PX_ASSERT(FAllGrtr(FLoad(1000.f), FAbs(deltaF)));

		FStore(clampedForce, &base->appliedForce);
		li0 = V3ScaleAdd(clinVel0, deltaF, li0);	ai0 = V3ScaleAdd(cangVel0, deltaF, ai0);
		li1 = V3ScaleAdd(clinVel1, deltaF, li1);	ai1 = V3ScaleAdd(cangVel1, deltaF, ai1);

		linVel0 = V3ScaleAdd(base->deltaVA.linear, deltaF, linVel0); 		angVel0 = V3ScaleAdd(base->deltaVA.angular, deltaF, angVel0);
		linVel1 = V3ScaleAdd(base->deltaVB.linear, deltaF, linVel1); 		angVel1 = V3ScaleAdd(base->deltaVB.angular, deltaF, angVel1);

		/*PX_ASSERT(FAllGrtr(FLoad(40.f * 40.f), V3Dot(linVel0, linVel0)));
		PX_ASSERT(FAllGrtr(FLoad(40.f * 40.f), V3Dot(linVel1, linVel1)));
		PX_ASSERT(FAllGrtr(FLoad(20.f * 20.f), V3Dot(angVel0, angVel0)));
		PX_ASSERT(FAllGrtr(FLoad(20.f * 20.f), V3Dot(angVel1, angVel1)));*/
	}

	linImpulse0 = V3Scale(li0, FLoad(header->linearInvMassScale0));
	linImpulse1 = V3Scale(li1, FLoad(header->linearInvMassScale1));
	angImpulse0 = V3Scale(ai0, FLoad(header->angularInvMassScale0));
	angImpulse1 = V3Scale(ai1, FLoad(header->angularInvMassScale1));
}

//Port of scalar implementation to SIMD maths with some interleaving of instructions
void solveExt1DStep(const PxSolverConstraintDesc& desc, const PxReal elapsedTimeF32, SolverContext& cache,
	const PxTGSSolverBodyTxInertia* const txInertias)
{
	Vec3V linVel0, angVel0, linVel1, angVel1;
	Vec3V linMotion0, angMotion0, linMotion1, angMotion1;

	QuatV rotA, rotB;

	Dy::FeatherstoneArticulation* artA = getArticulationA(desc);
	Dy::FeatherstoneArticulation* artB = getArticulationB(desc);

	if (artA == artB)
	{
		Cm::SpatialVectorV v0, v1;
		artA->pxcFsGetVelocities(desc.linkIndexA, desc.linkIndexB, v0, v1);
		linVel0 = v0.linear;
		angVel0 = v0.angular;
		linVel1 = v1.linear;
		angVel1 = v1.angular;

		const Cm::SpatialVectorV motionV0 = artA->getLinkMotionVector(desc.linkIndexA);  //PxcFsGetMotionVector(*artA, desc.linkIndexA);
		const Cm::SpatialVectorV motionV1 = artB->getLinkMotionVector(desc.linkIndexB); //PxcFsGetMotionVector(*artB, desc.linkIndexB);

		linMotion0 = motionV0.linear;
		angMotion0 = motionV0.angular;
		linMotion1 = motionV1.linear;
		angMotion1 = motionV1.angular;

		rotA = aos::QuatVLoadU(&artA->getDeltaQ(desc.linkIndexA).x);
		rotB = aos::QuatVLoadU(&artB->getDeltaQ(desc.linkIndexB).x);
	}
	else
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			linVel0 = V3LoadA(desc.tgsBodyA->linearVelocity);
			angVel0 = V3LoadA(desc.tgsBodyA->angularVelocity);
			linMotion0 = V3LoadA(desc.tgsBodyA->deltaLinDt);
			angMotion0 = V3LoadA(desc.tgsBodyA->deltaAngDt);
			rotA = aos::QuatVLoadA(&txInertias[desc.bodyADataIndex].deltaBody2World.q.x);
		}
		else
		{
			const Cm::SpatialVectorV v = artA->pxcFsGetVelocity(desc.linkIndexA);
			rotA = aos::QuatVLoadU(&artA->getDeltaQ(desc.linkIndexA).x);
			const Cm::SpatialVectorV motionV = artA->getLinkMotionVector(desc.linkIndexA);//PxcFsGetMotionVector(*artA, desc.linkIndexA);
			linVel0 = v.linear;
			angVel0 = v.angular;

			linMotion0 = motionV.linear;
			angMotion0 = motionV.angular;
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			linVel1 = V3LoadA(desc.tgsBodyB->linearVelocity);
			angVel1 = V3LoadA(desc.tgsBodyB->angularVelocity);
			linMotion1 = V3LoadA(desc.tgsBodyB->deltaLinDt);
			angMotion1 = V3LoadA(desc.tgsBodyB->deltaAngDt);
			rotB = aos::QuatVLoadA(&txInertias[desc.bodyBDataIndex].deltaBody2World.q.x);
		}
		else
		{
			Cm::SpatialVectorV v = artB->pxcFsGetVelocity(desc.linkIndexB);
			rotB = aos::QuatVLoadU(&artB->getDeltaQ(desc.linkIndexB).x);
			Cm::SpatialVectorV motionV = artB->getLinkMotionVector(desc.linkIndexB);// PxcFsGetMotionVector(*artB, desc.linkIndexB);
			linVel1 = v.linear;
			angVel1 = v.angular;

			linMotion1 = motionV.linear;
			angMotion1 = motionV.angular;
		}
	}

	Vec3V li0, li1, ai0, ai1;
	
	solveExt1D(desc, linVel0, linVel1, angVel0, angVel1, linMotion0, linMotion1, angMotion0, angMotion1, rotA, rotB, elapsedTimeF32, li0, li1, ai0, ai1);

	if (artA == artB)
	{
		artA->pxcFsApplyImpulses(desc.linkIndexA, li0,
			ai0, desc.linkIndexB, li1,
			ai1, cache.Z, cache.deltaV);
	}
	else
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			V3StoreA(linVel0, desc.tgsBodyA->linearVelocity);
			V3StoreA(angVel0, desc.tgsBodyA->angularVelocity);
		}
		else
		{
			artA->pxcFsApplyImpulse(desc.linkIndexA, li0, ai0, cache.Z, cache.deltaV);
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			V3StoreA(linVel1, desc.tgsBodyB->linearVelocity);
			V3StoreA(angVel1, desc.tgsBodyB->angularVelocity);
		}
		else
		{
			artB->pxcFsApplyImpulse(desc.linkIndexB, li1, ai1, cache.Z, cache.deltaV);
		}
	}
}




//Port of scalar implementation to SIMD maths with some interleaving of instructions
void solve1DStep(const PxSolverConstraintDesc& desc, const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	if (bPtr == NULL)
		return;

	PxTGSSolverBodyVel& b0 = *desc.tgsBodyA;
	PxTGSSolverBodyVel& b1 = *desc.tgsBodyB;

	const FloatV elapsed = FLoad(elapsedTime);

	const PxTGSSolverBodyTxInertia& txI0 = txInertias[desc.bodyADataIndex];
	const PxTGSSolverBodyTxInertia& txI1 = txInertias[desc.bodyBDataIndex];	

	const SolverConstraint1DHeaderStep* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep*>(bPtr);
	SolverConstraint1DStep* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DStep*>(bPtr + sizeof(SolverConstraint1DHeaderStep));

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V linVel1 = V3LoadA(b1.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularVelocity);
	Vec3V angState1 = V3LoadA(b1.angularVelocity);

	Mat33V sqrtInvInertia0 = Mat33V(V3LoadU(txI0.sqrtInvInertia.column0), V3LoadU(txI0.sqrtInvInertia.column1), V3LoadU(txI0.sqrtInvInertia.column2));
	Mat33V sqrtInvInertia1 = Mat33V(V3LoadU(txI1.sqrtInvInertia.column0), V3LoadU(txI1.sqrtInvInertia.column1), V3LoadU(txI1.sqrtInvInertia.column2));

	const FloatV invMass0 = FLoad(header->invMass0D0);
	const FloatV invMass1 = FLoad(header->invMass1D1);
	const FloatV invInertiaScale0 = FLoad(header->angularInvMassScale0);
	const FloatV invInertiaScale1 = FLoad(header->angularInvMassScale1);

	const QuatV deltaRotA = aos::QuatVLoadA(&txI0.deltaBody2World.q.x);
	const QuatV deltaRotB = aos::QuatVLoadA(&txI1.deltaBody2World.q.x);

	const Vec3V raPrev = V3LoadA(header->rAWorld);
	const Vec3V rbPrev = V3LoadA(header->rBWorld);

	const Vec3V ra = QuatRotate(deltaRotA, raPrev);
	const Vec3V rb = QuatRotate(deltaRotB, rbPrev);

	const Vec3V ang0 = V3LoadA(b0.deltaAngDt);
	const Vec3V ang1 = V3LoadA(b1.deltaAngDt);

	const Vec3V lin0 = V3LoadA(b0.deltaLinDt);
	const Vec3V lin1 = V3LoadA(b1.deltaLinDt);

	const Vec3V raMotion = V3Sub(V3Add(ra, lin0), raPrev);
	const Vec3V rbMotion = V3Sub(V3Add(rb, lin1), rbPrev);

	const VecCrossV raCross = V3PrepareCross(ra);
	const VecCrossV rbCross = V3PrepareCross(rb);

	const Vec4V ang0Ortho0_recipResponseW = V4LoadA(&header->angOrthoAxis0_recipResponseW[0].x);
	const Vec4V ang0Ortho1_recipResponseW = V4LoadA(&header->angOrthoAxis0_recipResponseW[1].x);
	const Vec4V ang0Ortho2_recipResponseW = V4LoadA(&header->angOrthoAxis0_recipResponseW[2].x);

	const Vec4V ang1Ortho0_Error0 = V4LoadA(&header->angOrthoAxis1_Error[0].x);
	const Vec4V ang1Ortho1_Error1 = V4LoadA(&header->angOrthoAxis1_Error[1].x);
	const Vec4V ang1Ortho2_Error2 = V4LoadA(&header->angOrthoAxis1_Error[2].x);

	const FloatV recipResponse0 = V4GetW(ang0Ortho0_recipResponseW);
	const FloatV recipResponse1 = V4GetW(ang0Ortho1_recipResponseW);
	const FloatV recipResponse2 = V4GetW(ang0Ortho2_recipResponseW);

	const Vec3V ang0Ortho0 = Vec3V_From_Vec4V(ang0Ortho0_recipResponseW);
	const Vec3V ang0Ortho1 = Vec3V_From_Vec4V(ang0Ortho1_recipResponseW);
	const Vec3V ang0Ortho2 = Vec3V_From_Vec4V(ang0Ortho2_recipResponseW);

	const Vec3V ang1Ortho0 = Vec3V_From_Vec4V(ang1Ortho0_Error0);
	const Vec3V ang1Ortho1 = Vec3V_From_Vec4V(ang1Ortho1_Error1);
	const Vec3V ang1Ortho2 = Vec3V_From_Vec4V(ang1Ortho2_Error2);

	FloatV error0 = FAdd(V4GetW(ang1Ortho0_Error0), FSub(V3Dot(ang0Ortho0, ang0), V3Dot(ang1Ortho0, ang1)));
	FloatV error1 = FAdd(V4GetW(ang1Ortho1_Error1), FSub(V3Dot(ang0Ortho1, ang0), V3Dot(ang1Ortho1, ang1)));
	FloatV error2 = FAdd(V4GetW(ang1Ortho2_Error2), FSub(V3Dot(ang0Ortho2, ang0), V3Dot(ang1Ortho2, ang1)));

	for (PxU32 i = 0; i<header->count; ++i, base++)
	{
		PxPrefetchLine(base + 1);
		SolverConstraint1DStep& c = *base;

		const Vec3V clinVel0 = V3LoadA(c.lin0);
		const Vec3V clinVel1 = V3LoadA(c.lin1);

		const Vec3V cangVel0_ = V3LoadA(c.ang0);
		const Vec3V cangVel1_ = V3LoadA(c.ang1);

		const FloatV angularErrorScale = FLoad(c.angularErrorScale);

		const FloatV biasScale = FLoad(c.biasScale);
		const FloatV maxBias = FLoad(c.maxBias);
		const FloatV targetVel = FLoad(c.velTarget);
		const FloatV appliedForce = FLoad(c.appliedForce);
		const FloatV velMultiplier = FLoad(c.velMultiplier);

		const FloatV maxImpulse = FLoad(c.maxImpulse);
		const FloatV minImpulse = FLoad(c.minImpulse);

		Vec3V cangVel0 = V3Add(cangVel0_, V3Cross(raCross, clinVel0));
		Vec3V cangVel1 = V3Add(cangVel1_, V3Cross(rbCross, clinVel1));
		
		FloatV error = FLoad(c.error);

		const FloatV minBias = (c.flags & DY_SC_FLAG_INEQUALITY) ? FNeg(FMax()) : FNeg(maxBias);
	
		Vec3V raXnI = M33MulV3(sqrtInvInertia0, cangVel0);
		Vec3V rbXnI = M33MulV3(sqrtInvInertia1, cangVel1);

		if (c.flags & DY_SC_FLAG_ORTHO_TARGET)
		{
			//Re-orthogonalize the constraints before velocity projection and impulse response calculation
			//Can be done in using instruction parallelism because angular locked axes are orthogonal to linear axes!

			const FloatV proj0 = FMul(V3SumElems(V3MulAdd(raXnI, ang0Ortho0,
				V3Mul(rbXnI, ang1Ortho0))), recipResponse0);

			const FloatV proj1 = FMul(V3SumElems(V3MulAdd(raXnI, ang0Ortho1, 
				V3Mul(rbXnI, ang1Ortho1))), recipResponse1);
			const FloatV proj2 = FMul(V3SumElems(V3MulAdd(raXnI, ang0Ortho2,
				V3Mul(rbXnI, ang1Ortho2))), recipResponse2);

			const Vec3V delta0 = V3ScaleAdd(ang0Ortho0, proj0, V3ScaleAdd(ang0Ortho1, proj1, V3Scale(ang0Ortho2, proj2)));
			const Vec3V delta1 = V3ScaleAdd(ang1Ortho0, proj0, V3ScaleAdd(ang1Ortho1, proj1, V3Scale(ang1Ortho2, proj2)));

			raXnI = V3Sub(raXnI, delta0);
			rbXnI = V3Sub(rbXnI, delta1);

			error = FSub(error, FScaleAdd(error0, proj0, FScaleAdd(error1, proj1, FMul(error2, proj2))));
		}

		const FloatV deltaAng = FMul(angularErrorScale, FSub(V3Dot(raXnI, ang0), V3Dot(rbXnI, ang1)));

		FloatV errorChange = FNegScaleSub(targetVel, elapsed, FAdd(FSub(V3Dot(raMotion, clinVel0), V3Dot(rbMotion, clinVel1)), deltaAng));

		const FloatV resp0 = FScaleAdd(invMass0, V3Dot(clinVel0, clinVel0), V3SumElems(V3Mul(V3Scale(raXnI, invInertiaScale0), raXnI)));
		const FloatV resp1 = FSub(FMul(invMass1, V3Dot(clinVel1, clinVel1)), V3SumElems(V3Mul(V3Scale(rbXnI, invInertiaScale1), rbXnI)));
		
		const FloatV response = FAdd(resp0, resp1);
		const FloatV recipResponse = FSel(FIsGrtr(response, FZero()), FRecip(response), FZero());
	
		const FloatV vMul = FMul(recipResponse, velMultiplier);
		
		const FloatV unclampedBias = FScaleAdd(errorChange, biasScale, error);
		const FloatV bias = FClamp(unclampedBias, minBias, maxBias);

		const FloatV constant = FMul(recipResponse, FAdd(bias, targetVel));

		const Vec3V v0 = V3MulAdd(linVel0, clinVel0, V3Mul(angState0, raXnI));
		const Vec3V v1 = V3MulAdd(linVel1, clinVel1, V3Mul(angState1, rbXnI));
		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

		const FloatV unclampedForce = FAdd(appliedForce, FScaleAdd(vMul, normalVel, constant));
		const FloatV clampedForce = FClamp(unclampedForce, minImpulse, maxImpulse);
		const FloatV deltaF = FSub(clampedForce, appliedForce);

		FStore(clampedForce, &c.appliedForce);
		linVel0 = V3ScaleAdd(clinVel0, FMul(deltaF, invMass0), linVel0);
		linVel1 = V3NegScaleSub(clinVel1, FMul(deltaF, invMass1), linVel1);
		angState0 = V3ScaleAdd(raXnI, FMul(deltaF, invInertiaScale0), angState0);
		angState1 = V3ScaleAdd(rbXnI, FMul(deltaF, invInertiaScale1), angState1);
	}

	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(angState0, b0.angularVelocity);
	V3StoreA(linVel1, b1.linearVelocity);
	V3StoreA(angState1, b1.angularVelocity);

	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularVelocity.isFinite());
	PX_ASSERT(b1.linearVelocity.isFinite());
	PX_ASSERT(b1.angularVelocity.isFinite());
}


//Port of scalar implementation to SIMD maths with some interleaving of instructions
void conclude1DStep(const PxSolverConstraintDesc& desc)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	if (bPtr == NULL)
		return;
	
	const SolverConstraint1DHeaderStep* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeaderStep*>(bPtr);
	PxU8* PX_RESTRICT base = bPtr + sizeof(SolverConstraint1DHeaderStep);
	const PxU32 stride = header->type == DY_SC_TYPE_RB_1D ? sizeof(SolverConstraint1DStep) : sizeof(SolverConstraint1DExtStep);

	for (PxU32 i = 0; i<header->count; ++i, base+=stride)
	{
		SolverConstraint1DStep& c = *reinterpret_cast<SolverConstraint1DStep*>(base);
		PxPrefetchLine(&c + 1);
		if (!(c.flags & DY_SC_FLAG_KEEP_BIAS))
		{
			c.biasScale = 0.f;
			c.error = 0.f;
		}
	}
}

void concludeContact(const PxSolverConstraintDesc& desc)
{
	PX_UNUSED(desc);
	//const PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

	////hopefully pointer aliasing doesn't bite.
	//PxU8* PX_RESTRICT currPtr = desc.constraint;

	//SolverContactHeaderStep* PX_RESTRICT firstHdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);

	//bool isExtended = firstHdr->type == DY_SC_TYPE_EXT_CONTACT;

	//const PxU32 contactStride = isExtended ? sizeof(SolverContactPointStepExt) : sizeof(SolverContactPointStep);
	//const PxU32 frictionStride = isExtended ? sizeof(SolverContactFrictionStepExt) : sizeof(SolverContactFrictionStep);


	//while (currPtr < last)
	//{
	//	SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);
	//	currPtr += sizeof(SolverContactHeaderStep);

	//	const PxU32 numNormalConstr = hdr->numNormalConstr;
	//	const PxU32	numFrictionConstr = hdr->numFrictionConstr;

	//	/*PxU8* PX_RESTRICT contacts = currPtr;
	//	prefetchLine(contacts);*/
	//	currPtr += numNormalConstr * contactStride;

	//	//PxF32* forceBuffer = reinterpret_cast<PxF32*>(currPtr);
	//	currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

	//	PxU8* PX_RESTRICT frictions = currPtr;
	//	currPtr += numFrictionConstr * frictionStride;

	//	/*for (PxU32 i = 0; i < numNormalConstr; ++i)
	//	{
	//		SolverContactPointStep& c = *reinterpret_cast<SolverContactPointStep*>(contacts);
	//		contacts += contactStride;
	//		if(c.separation <= 0.f)
	//			c.biasCoefficient = 0.f;
	//	}*/

	//	for (PxU32 i = 0; i < numFrictionConstr; ++i)
	//	{
	//		SolverContactFrictionStep& f = *reinterpret_cast<SolverContactFrictionStep*>(frictions);
	//		frictions += frictionStride;
	//		f.biasScale = 0.f;
	//	}
	//}

	//PX_ASSERT(currPtr == last);
}




void writeBack1D(const PxSolverConstraintDesc& desc)
{
	ConstraintWriteback* writeback = reinterpret_cast<ConstraintWriteback*>(desc.writeBack);
	if (writeback)
	{
		SolverConstraint1DHeaderStep* header = reinterpret_cast<SolverConstraint1DHeaderStep*>(desc.constraint);
		PxU8* base = desc.constraint + sizeof(SolverConstraint1DHeaderStep);
		PxU32 stride = header->type == DY_SC_TYPE_EXT_1D ? sizeof(SolverConstraint1DExtStep) : sizeof(SolverConstraint1DStep);

		PxVec3 lin(0), ang(0);
		for (PxU32 i = 0; i<header->count; i++)
		{
			const SolverConstraint1DStep* c = reinterpret_cast<SolverConstraint1DStep*>(base);
			if (c->flags & DY_SC_FLAG_OUTPUT_FORCE)
			{
				lin += c->lin0 * c->appliedForce;
				ang += (c->ang0 + c->lin0.cross(header->rAWorld)) * c->appliedForce;
			}
			base += stride;
		}

		ang -= header->body0WorldOffset.cross(lin);
		writeback->linearImpulse = lin;
		writeback->angularImpulse = ang;
		writeback->broken = header->breakable ? PxU32(lin.magnitude()>header->linBreakImpulse || ang.magnitude()>header->angBreakImpulse) : 0;

		//KS - the amount of memory we allocated may now be significantly larger than the number of constraint rows. This is because
		//we discard degenerate rows in the articulation constraint prep code.
		//PX_ASSERT(desc.constraint + (desc.constraintLengthOver16 * 16) == base);
	}
}

static FloatV solveExtContactsStep(SolverContactPointStepExt* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
	Vec3V& linVel0, Vec3V& angVel0,
	Vec3V& linVel1, Vec3V& angVel1,
	Vec3V& li0, Vec3V& ai0,
	Vec3V& li1, Vec3V& ai1,
	const Vec3V& linDeltaA, const Vec3V& linDeltaB, const Vec3V& angDeltaA, const Vec3V angDeltaB, const FloatV& maxPenBias,
	PxF32* PX_RESTRICT appliedForceBuffer,
	const FloatV& minPen,
	const FloatV& elapsedTime)
{
	PX_UNUSED(elapsedTime);
	const FloatV deltaV = V3Dot(contactNormal, V3Sub(linDeltaA, linDeltaB));

	FloatV accumulatedNormalImpulse = FZero();
	for (PxU32 i = 0; i<nbContactPoints; i++)
	{
		SolverContactPointStepExt& c = contacts[i];
		PxPrefetchLine(&contacts[i + 1]);

		const Vec3V raXn = V3LoadA(c.raXnI);
		const Vec3V rbXn = V3LoadA(c.rbXnI);

		const FloatV appliedForce = FLoad(appliedForceBuffer[i]);
		const FloatV velMultiplier = FLoad(c.velMultiplier);
		const FloatV recipResponse = FLoad(c.recipResponse);

		Vec3V v = V3MulAdd(linVel0, contactNormal, V3Mul(angVel0, raXn));
		v = V3Sub(v, V3MulAdd(linVel1, contactNormal, V3Mul(angVel1, rbXn)));
		const FloatV normalVel = V3SumElems(v);

		const FloatV angDelta0 = V3Dot(angDeltaA, raXn);
		const FloatV angDelta1 = V3Dot(angDeltaB, rbXn);

		const FloatV deltaAng = FSub(angDelta0, angDelta1);

		const FloatV targetVel = FLoad(c.targetVelocity);

		const FloatV deltaBias = FSub(FAdd(deltaV, deltaAng), FMul(targetVel, elapsedTime));
		//const FloatV deltaBias = FAdd(deltaV, deltaAng);

		const FloatV biasCoefficient = FLoad(c.biasCoefficient);

		FloatV sep = FMax(minPen, FAdd(FLoad(c.separation), deltaBias));

		const FloatV bias = FMin(FNeg(maxPenBias), FMul(biasCoefficient, sep));

		const FloatV tVelBias = FMul(bias, recipResponse);

		const FloatV _deltaF = FMax(FSub(tVelBias, FMul(FSub(normalVel, targetVel), velMultiplier)), FNeg(appliedForce));
		const FloatV _newForce = FAdd(appliedForce, _deltaF);
		const FloatV newForce = FMin(_newForce, FLoad(c.maxImpulse));
		const FloatV deltaF = FSub(newForce, appliedForce);

		const Vec3V raXnI = c.angDeltaVA;
		const Vec3V rbXnI = c.angDeltaVB;

		linVel0 = V3ScaleAdd(c.linDeltaVA, deltaF, linVel0);
		angVel0 = V3ScaleAdd(raXnI, deltaF, angVel0);
		linVel1 = V3ScaleAdd(c.linDeltaVB, deltaF, linVel1);
		angVel1 = V3ScaleAdd(rbXnI, deltaF, angVel1);

		li0 = V3ScaleAdd(contactNormal, deltaF, li0);	ai0 = V3ScaleAdd(raXn, deltaF, ai0);
		li1 = V3ScaleAdd(contactNormal, deltaF, li1);	ai1 = V3ScaleAdd(rbXn, deltaF, ai1);

		const FloatV newAppliedForce = FAdd(appliedForce, deltaF);

		FStore(newAppliedForce, &appliedForceBuffer[i]);

		accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newAppliedForce);
	}
	return accumulatedNormalImpulse;
}

void solveExtContactStep(const PxSolverConstraintDesc& desc, Vec3V& linVel0, Vec3V& linVel1, Vec3V& angVel0, Vec3V& angVel1,
	Vec3V& linDelta0, Vec3V& linDelta1, Vec3V& angDelta0, Vec3V& angDelta1, Vec3V& linImpulse0, Vec3V& linImpulse1, Vec3V& angImpulse0, Vec3V& angImpulse1, 
	bool doFriction, const PxReal minPenetration, const PxReal elapsedTimeF32)
{
	PX_UNUSED(doFriction);
	const FloatV elapsedTime = FLoad(elapsedTimeF32);
	const FloatV minPen = FLoad(minPenetration);

	const FloatV zero = FZero();

	const PxU8* PX_RESTRICT last = desc.constraint + desc.constraintLengthOver16 * 16;

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	const Vec3V relMotion = V3Sub(linDelta0, linDelta1);

	while (currPtr < last)
	{
		SolverContactHeaderStep* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeaderStep*>(currPtr);
		currPtr += sizeof(SolverContactHeaderStep);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverContactPointStepExt* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointStepExt*>(currPtr);
		PxPrefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPointStepExt);

		PxF32* appliedForceBuffer = reinterpret_cast<PxF32*>(currPtr);
		currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

		SolverContactFrictionStepExt* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionStepExt*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionStepExt);

		Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

		const Vec3V contactNormal = V3LoadA(hdr->normal);

		const FloatV accumulatedNormalImpulse = FMax(solveExtContactsStep(contacts, numNormalConstr, contactNormal, linVel0, angVel0, linVel1,
			angVel1, li0, ai0, li1, ai1, linDelta0, linDelta1, angDelta0, angDelta1, FLoad(hdr->maxPenBias), appliedForceBuffer, minPen, elapsedTime),
			FLoad(hdr->minNormalForce));

		if (numFrictionConstr)
		{
			PxPrefetchLine(frictions);
			const FloatV maxFrictionImpulse = FMul(hdr->getStaticFriction(), accumulatedNormalImpulse);
			const FloatV maxDynFrictionImpulse = FMul(hdr->getDynamicFriction(), accumulatedNormalImpulse);

			BoolV broken = BFFFF();

			const PxU32 numFrictionPairs = numFrictionConstr &6;

			for (PxU32 i = 0; i<numFrictionPairs; i += 2)
			{
				SolverContactFrictionStepExt& f0 = frictions[i];
				SolverContactFrictionStepExt& f1 = frictions[i+1];
				PxPrefetchLine(&frictions[i + 2]);

				const Vec4V normalXYZ_ErrorW0 = f0.normalXYZ_ErrorW;
				const Vec4V raXn_targetVelW0 = f0.raXnI_targetVelW;
				const Vec4V rbXn_velMultiplierW0 = f0.rbXnI_velMultiplierW;
				const Vec4V normalXYZ_ErrorW1 = f1.normalXYZ_ErrorW;
				const Vec4V raXn_targetVelW1 = f1.raXnI_targetVelW;
				const Vec4V rbXn_velMultiplierW1 = f1.rbXnI_velMultiplierW;

				const Vec3V normal0 = Vec3V_From_Vec4V(normalXYZ_ErrorW0);
				const Vec3V raXn0 = Vec3V_From_Vec4V(raXn_targetVelW0);
				const Vec3V rbXn0 = Vec3V_From_Vec4V(rbXn_velMultiplierW0);
				const Vec3V raXnI0 = f0.angDeltaVA;
				const Vec3V rbXnI0 = f0.angDeltaVB;

				const Vec3V normal1 = Vec3V_From_Vec4V(normalXYZ_ErrorW1);
				const Vec3V raXn1 = Vec3V_From_Vec4V(raXn_targetVelW1);
				const Vec3V rbXn1 = Vec3V_From_Vec4V(rbXn_velMultiplierW1);
				const Vec3V raXnI1 = f1.angDeltaVA;
				const Vec3V rbXnI1 = f1.angDeltaVB;

				const FloatV frictionScale = FLoad(f0.frictionScale);
				const FloatV biasScale = FLoad(f0.biasScale);

				const FloatV appliedForce0 = FLoad(f0.appliedForce);
				const FloatV velMultiplier0 = V4GetW(rbXn_velMultiplierW0);
				const FloatV targetVel0 = V4GetW(raXn_targetVelW0);
				const FloatV initialError0 = V4GetW(normalXYZ_ErrorW0);

				const FloatV appliedForce1 = FLoad(f1.appliedForce);
				const FloatV velMultiplier1 = V4GetW(rbXn_velMultiplierW1);
				const FloatV targetVel1 = V4GetW(raXn_targetVelW1);
				const FloatV initialError1 = V4GetW(normalXYZ_ErrorW1);

				const FloatV error0 = FAdd(initialError0, FNegScaleSub(targetVel0, elapsedTime, FAdd(FSub(V3Dot(raXn0, angDelta0), V3Dot(rbXn0, angDelta1)), V3Dot(normal0, relMotion))));
				const FloatV error1 = FAdd(initialError1, FNegScaleSub(targetVel1, elapsedTime, FAdd(FSub(V3Dot(raXn1, angDelta0), V3Dot(rbXn1, angDelta1)), V3Dot(normal1, relMotion))));

				const FloatV bias0 = FMul(error0, biasScale);
				const FloatV bias1 = FMul(error1, biasScale);

				const Vec3V v00 = V3MulAdd(linVel0, normal0, V3Mul(angVel0, raXn0));
				const Vec3V v10 = V3MulAdd(linVel1, normal0, V3Mul(angVel1, rbXn0));
				const FloatV normalVel0 = V3SumElems(V3Sub(v00, v10));

				const Vec3V v01 = V3MulAdd(linVel0, normal1, V3Mul(angVel0, raXn1));
				const Vec3V v11 = V3MulAdd(linVel1, normal1, V3Mul(angVel1, rbXn1));
				const FloatV normalVel1 = V3SumElems(V3Sub(v01, v11));

				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const FloatV tmp10 = FNegScaleSub(FSub(bias0, targetVel0), velMultiplier0, appliedForce0);
				const FloatV tmp11 = FNegScaleSub(FSub(bias1, targetVel1), velMultiplier1, appliedForce1);

				// Algorithm:
				// if abs(appliedForce + deltaF) > maxFrictionImpulse
				//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
				//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
				//    set broken flag to true || broken flag

				// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
				// FloatV potentialSumF = FAdd(appliedForce, deltaF);

				const FloatV totalImpulse0 = FNegScaleSub(normalVel0, velMultiplier0, tmp10);
				const FloatV totalImpulse1 = FNegScaleSub(normalVel1, velMultiplier1, tmp11);

				// On XBox this clamping code uses the vector simple pipe rather than vector float,
				// which eliminates a lot of stall cycles

				const FloatV totalImpulse = FSqrt(FAdd(FMul(totalImpulse0, totalImpulse0), FMul(totalImpulse1, totalImpulse1)));

				const BoolV clamp = FIsGrtr(totalImpulse, FMul(maxFrictionImpulse, frictionScale));

				const FloatV totalClamped = FSel(clamp, FMin(FMul(maxDynFrictionImpulse, frictionScale), totalImpulse), totalImpulse);

				const FloatV ratio = FSel(FIsGrtr(totalImpulse, zero), FDiv(totalClamped, totalImpulse), zero);

				const FloatV newAppliedForce0 = FMul(ratio, totalImpulse0);
				const FloatV newAppliedForce1 = FMul(ratio, totalImpulse1);

				broken = BOr(broken, clamp);

				const FloatV deltaF0 = FSub(newAppliedForce0, appliedForce0);
				const FloatV deltaF1 = FSub(newAppliedForce1, appliedForce1);

				linVel0 = V3ScaleAdd(f0.linDeltaVA, deltaF0, V3ScaleAdd(f1.linDeltaVA, deltaF1, linVel0));
				angVel0 = V3ScaleAdd(raXnI0, deltaF0, V3ScaleAdd(raXnI1, deltaF1, angVel0));
				linVel1 = V3ScaleAdd(f0.linDeltaVB, deltaF0, V3ScaleAdd(f1.linDeltaVB, deltaF1, linVel1));
				angVel1 = V3ScaleAdd(rbXnI0, deltaF0, V3ScaleAdd(rbXnI1, deltaF1, angVel1));

				/*PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel0, linVel0)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel1, linVel1)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel0, angVel0)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel1, angVel1)));*/

				li0 = V3ScaleAdd(normal0, deltaF0, V3ScaleAdd(normal1, deltaF1, li0));
				ai0 = V3ScaleAdd(raXn0, deltaF0, V3ScaleAdd(raXn1, deltaF1, ai0));
				li1 = V3ScaleAdd(normal0, deltaF0, V3ScaleAdd(normal1, deltaF1, li1));
				ai1 = V3ScaleAdd(rbXn0, deltaF0, V3ScaleAdd(rbXn1, deltaF1, ai1));

				f0.setAppliedForce(newAppliedForce0);
				f1.setAppliedForce(newAppliedForce1);
			}

			for (PxU32 i = numFrictionPairs; i<numFrictionConstr; i++)
			{
				SolverContactFrictionStepExt& f = frictions[i];
				PxPrefetchLine(&frictions[i + 1]);

				const Vec4V raXn_targetVelW = f.raXnI_targetVelW;
				const Vec4V rbXn_velMultiplierW = f.rbXnI_velMultiplierW;

				const Vec3V raXn = Vec3V_From_Vec4V(raXn_targetVelW);
				const Vec3V rbXn = Vec3V_From_Vec4V(rbXn_velMultiplierW);
				const Vec3V raXnI = f.angDeltaVA;
				const Vec3V rbXnI = f.angDeltaVB;

				const FloatV frictionScale = FLoad(f.frictionScale);

				const FloatV appliedForce = FLoad(f.appliedForce);
				const FloatV velMultiplier = V4GetW(rbXn_velMultiplierW);
				const FloatV targetVel = V4GetW(raXn_targetVelW);

				const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);
				//const FloatV negMaxFrictionImpulse = FNeg(maxFrictionImpulse);

				const Vec3V v0 = V3Mul(angVel0, raXn);
				const Vec3V v1 = V3Mul(angVel1, rbXn);
				const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const FloatV tmp1 = FNegScaleSub(FNeg(targetVel), velMultiplier, appliedForce);

				// Algorithm:
				// if abs(appliedForce + deltaF) > maxFrictionImpulse
				//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
				//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
				//    set broken flag to true || broken flag

				// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
				// FloatV potentialSumF = FAdd(appliedForce, deltaF);

				const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

				// On XBox this clamping code uses the vector simple pipe rather than vector float,
				// which eliminates a lot of stall cycles

				const BoolV clamp = FIsGrtr(FAbs(totalImpulse), FMul(maxFrictionImpulse, frictionScale));

				const FloatV totalClamped = FMin(FMul(maxDynFrictionImpulse, frictionScale), FMax(FMul(negMaxDynFrictionImpulse, frictionScale), totalImpulse));

				const FloatV newAppliedForce = FSel(clamp, totalClamped, totalImpulse);

				broken = BOr(broken, clamp);

				FloatV deltaF = FSub(newAppliedForce, appliedForce);

				linVel0 = V3ScaleAdd(f.linDeltaVA, deltaF, linVel0);
				angVel0 = V3ScaleAdd(raXnI, deltaF, angVel0);
				linVel1 = V3ScaleAdd(f.linDeltaVB, deltaF, linVel1);
				angVel1 = V3ScaleAdd(rbXnI, deltaF, angVel1);

				/*PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel0, linVel0)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel1, linVel1)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel0, angVel0)));
				PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel1, angVel1)));*/

				ai0 = V3ScaleAdd(raXn, deltaF, ai0);
				ai1 = V3ScaleAdd(rbXn, deltaF, ai1);

				f.setAppliedForce(newAppliedForce);
			}
			Store_From_BoolV(broken, &hdr->broken);
		}

		linImpulse0 = V3ScaleAdd(li0, hdr->getDominance0(), linImpulse0);
		angImpulse0 = V3ScaleAdd(ai0, FLoad(hdr->angDom0), angImpulse0);
		linImpulse1 = V3NegScaleSub(li1, hdr->getDominance1(), linImpulse1);
		angImpulse1 = V3NegScaleSub(ai1, FLoad(hdr->angDom1), angImpulse1);
	}

	PX_ASSERT(currPtr == last);
}


void solveExtContactStep(const PxSolverConstraintDesc& desc, bool doFriction, const PxReal minPenetration,
	const PxReal elapsedTimeF32, SolverContext& cache)
{
	Vec3V linVel0, angVel0, linVel1, angVel1;
	Vec3V linDelta0, angDelta0, linDelta1, angDelta1;

	Dy::FeatherstoneArticulation* artA = getArticulationA(desc);
	Dy::FeatherstoneArticulation* artB = getArticulationB(desc);

	if (artA == artB)
	{
		Cm::SpatialVectorV v0, v1;
		artA->pxcFsGetVelocities(desc.linkIndexA, desc.linkIndexB, v0, v1);
		linVel0 = v0.linear;
		angVel0 = v0.angular;
		linVel1 = v1.linear;
		angVel1 = v1.angular;

		Cm::SpatialVectorV motionV0 = artA->getLinkMotionVector(desc.linkIndexA);// PxcFsGetMotionVector(*artA, desc.linkIndexA);
		Cm::SpatialVectorV motionV1 = artB->getLinkMotionVector(desc.linkIndexB);// PxcFsGetMotionVector(*artB, desc.linkIndexB);

		linDelta0 = motionV0.linear;
		angDelta0 = motionV0.angular;
		linDelta1 = motionV1.linear;
		angDelta1 = motionV1.angular;
	}
	else
	{
		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			linVel0 = V3LoadA(desc.tgsBodyA->linearVelocity);
			angVel0 = V3LoadA(desc.tgsBodyA->angularVelocity);
			linDelta0 = V3LoadA(desc.tgsBodyA->deltaLinDt);
			angDelta0 = V3LoadA(desc.tgsBodyA->deltaAngDt);
		}
		else
		{
			Cm::SpatialVectorV v = artA->pxcFsGetVelocity(desc.linkIndexA);
			Cm::SpatialVectorV deltaV = artA->getLinkMotionVector(desc.linkIndexA);// PxcFsGetMotionVector(*artA, desc.linkIndexA);
			linVel0 = v.linear;
			angVel0 = v.angular;
			linDelta0 = deltaV.linear;
			angDelta0 = deltaV.angular;
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			linVel1 = V3LoadA(desc.tgsBodyB->linearVelocity);
			angVel1 = V3LoadA(desc.tgsBodyB->angularVelocity);
			linDelta1 = V3LoadA(desc.tgsBodyB->deltaLinDt);
			angDelta1 = V3LoadA(desc.tgsBodyB->deltaAngDt);
		}
		else
		{
			Cm::SpatialVectorV v = artB->pxcFsGetVelocity(desc.linkIndexB);
			Cm::SpatialVectorV deltaV = artB->getLinkMotionVector(desc.linkIndexB);// PxcFsGetMotionVector(*artB, desc.linkIndexB);
			linVel1 = v.linear;
			angVel1 = v.angular;
			linDelta1 = deltaV.linear;
			angDelta1 = deltaV.angular;
		}
	}

	/*PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel0, linVel0)));
	PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(linVel1, linVel1)));
	PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel0, angVel0)));
	PX_ASSERT(FAllGrtr(FLoad(100.f * 100.f), V3Dot(angVel1, angVel1)));*/

	Vec3V linImpulse0 = V3Zero(), linImpulse1 = V3Zero(), angImpulse0 = V3Zero(), angImpulse1 = V3Zero();


	solveExtContactStep(desc, linVel0, linVel1, angVel0, angVel1, linDelta0, linDelta1, angDelta0, angDelta1, linImpulse0, linImpulse1, angImpulse0,
		angImpulse1, doFriction, minPenetration, elapsedTimeF32);

	if (artA == artB)
	{
		artA->pxcFsApplyImpulses(desc.linkIndexA, 
			linImpulse0, angImpulse0, desc.linkIndexB, linImpulse1, angImpulse1, cache.Z, cache.deltaV);
	}
	else
	{

		if (desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY)
		{
			V3StoreA(linVel0, desc.tgsBodyA->linearVelocity);
			V3StoreA(angVel0, desc.tgsBodyA->angularVelocity);
		}
		else
		{
			artA->pxcFsApplyImpulse(desc.linkIndexA, linImpulse0, angImpulse0, cache.Z, cache.deltaV);
		}

		if (desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY)
		{
			V3StoreA(linVel1, desc.tgsBodyB->linearVelocity);
			V3StoreA(angVel1, desc.tgsBodyB->angularVelocity);
		}
		else
		{
			artB->pxcFsApplyImpulse(desc.linkIndexB, linImpulse1, angImpulse1, cache.Z, cache.deltaV);
		}
	}
}

void solveContactBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const /*txInertias*/, const PxReal minPenetration, const PxReal elapsedTime, SolverContext& /*cache*/)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solveContact(desc[i], true, minPenetration, elapsedTime);
	}
}

void solve1DBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const txInertias, const PxReal /*minPenetration*/, const PxReal elapsedTime, SolverContext& /*cache*/)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solve1DStep(desc[i], txInertias, elapsedTime);
	}
}

void solveExtContactBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const /*txInertias*/, const PxReal minPenetration, const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solveExtContactStep(desc[i], true, minPenetration, elapsedTime, cache);
	}
}

void solveExt1DBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const txInertias, const PxReal /*minPenetration*/, const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solveExt1DStep(desc[i], elapsedTime, cache, txInertias);
	}
}

void writeBackContact(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc, SolverContext* cache)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		writeBackContact(desc[i], cache);
	}
}

void writeBack1D(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc, SolverContext* /*cache*/)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		writeBack1D(desc[i]);
	}
}

void solveConclude1DBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime, SolverContext& /*cache*/)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solve1DStep(desc[i], txInertias, elapsedTime);
		conclude1DStep(desc[i]);
	}
}

void solveConclude1DBlockExt(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solveExt1DStep(desc[i], elapsedTime, cache, txInertias);
		conclude1DStep(desc[i]);
	}
}


void solveConcludeContactBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const /*txInertias*/, const PxReal elapsedTime, SolverContext& /*cache*/)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solveContact(desc[i], true, -PX_MAX_F32, elapsedTime);
		concludeContact(desc[i]);
	}
}

void solveConcludeContactExtBlock(const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const /*txInertias*/, const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 i = hdr.startIndex, endIdx = hdr.startIndex + hdr.stride; i < endIdx; ++i)
	{
		solveExtContactStep(desc[i], true, -PX_MAX_F32, elapsedTime, cache);
		concludeContact(desc[i]);
	}
}


}
}
