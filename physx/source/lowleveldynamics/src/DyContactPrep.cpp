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
#include "DyThreadContext.h"
#include "PxcNpContactPrepShared.h"
#include "DyConstraintPrep.h"
#include "DyAllocator.h"

using namespace physx;

#include "DyContactPrepShared.h"

#include "DySolverConstraint1DStep.h"

using namespace aos;

namespace physx
{
namespace Dy
{

PxcCreateFinalizeSolverContactMethod createFinalizeMethods[3] =
{
	createFinalizeSolverContacts,
	createFinalizeSolverContactsCoulomb1D,
	createFinalizeSolverContactsCoulomb2D
};

static void setupFinalizeSolverConstraints(
							const PxSolverContactDesc& contactDesc,
							const CorrelationBuffer& c,
							PxU8* workspace,
							const PxSolverBodyData& data0,
							const PxSolverBodyData& data1,
							PxReal invDtF32, PxReal dtF32, PxReal bounceThresholdF32,
							bool hasForceThreshold, bool staticOrKinematicBody,
							PxU8* frictionDataPtr
							)
{
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	const Vec3V solverOffsetSlop = V3Load(contactDesc.offsetSlop);

	const FloatV ccdMaxSeparation = FLoad(contactDesc.maxCCDSeparation);

	const PxU8 flags = PxU8(hasForceThreshold ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0);

	const PxU8 type = PxTo8(staticOrKinematicBody ? DY_SC_TYPE_STATIC_CONTACT : DY_SC_TYPE_RB_CONTACT);

	const FloatV zero = FZero();

	const FloatV d0 = FLoad(contactDesc.invMassScales.linear0);
	const FloatV d1 = FLoad(contactDesc.invMassScales.linear1);
	const FloatV angD0 = FLoad(contactDesc.invMassScales.angular0);
	const FloatV angD1 = FLoad(contactDesc.invMassScales.angular1);

	const FloatV nDom1fV = FNeg(d1);

	const FloatV invMass0 = FLoad(data0.invMass);
	const FloatV invMass1 = FLoad(data1.invMass);

	const FloatV invMass0_dom0fV = FMul(d0, invMass0);
	const FloatV invMass1_dom1fV = FMul(nDom1fV, invMass1);

	Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass0_dom0fV);
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass1_dom1fV);

	const FloatV restDistance = FLoad(contactDesc.restDistance);

	const FloatV maxPenBias = FMax(FLoad(data0.penBiasClamp), FLoad(data1.penBiasClamp));

	const QuatV bodyFrame0q = QuatVLoadU(&contactDesc.bodyFrame0.q.x);
	const Vec3V bodyFrame0p = V3LoadU_SafeReadW(contactDesc.bodyFrame0.p);	// PT: see compile-time-assert in PxSolverConstraintPrepDescBase
	const QuatV bodyFrame1q = QuatVLoadU(&contactDesc.bodyFrame1.q.x);
	const Vec3V bodyFrame1p = V3LoadU_SafeReadW(contactDesc.bodyFrame1.p);	// PT: see compile-time-assert in PxSolverConstraintPrepDescBase

	PxU32 frictionPatchWritebackAddrIndex = 0;

	PxPrefetchLine(c.contactID);
	PxPrefetchLine(c.contactID, 128);

	const Vec3V linVel0 = V3LoadU_SafeReadW(data0.linearVelocity);	// PT: safe because 'invMass' follows 'linearVelocity' in PxSolverBodyData
	const Vec3V linVel1 = V3LoadU_SafeReadW(data1.linearVelocity);	// PT: safe because 'invMass' follows 'linearVelocity' in PxSolverBodyData
	const Vec3V angVel0 = V3LoadU_SafeReadW(data0.angularVelocity);	// PT: safe because 'reportThreshold' follows 'angularVelocity' in PxSolverBodyData
	const Vec3V angVel1 = V3LoadU_SafeReadW(data1.angularVelocity);	// PT: safe because 'reportThreshold' follows 'angularVelocity' in PxSolverBodyData

	PX_ALIGN(16, const Mat33V invSqrtInertia0)
	(
		V3LoadU_SafeReadW(data0.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
		V3LoadU_SafeReadW(data0.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
		V3LoadU_SafeReadW(data0.sqrtInvInertia.column2)		// PT: safe because sqrtInvInertia is not the last member of PxSolverBodyData, see compile-time-assert in PxSolverBodyData
	);
	
	PX_ALIGN(16, const Mat33V invSqrtInertia1)
	(
		V3LoadU_SafeReadW(data1.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
		V3LoadU_SafeReadW(data1.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
		V3LoadU_SafeReadW(data1.sqrtInvInertia.column2)		// PT: safe because sqrtInvInertia is not the last member of PxSolverBodyData, see compile-time-assert in PxSolverBodyData
	);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV p8 = FLoad(0.8f);
	const FloatV bounceThreshold = FLoad(bounceThresholdF32);

	const FloatV invDtp8 = FMul(invDt, p8);
	const FloatV dt = FLoad(dtF32);

	const PxContactPoint* PX_RESTRICT buffer = contactDesc.contacts;
	PxU8* PX_RESTRICT ptr = workspace;
	for(PxU32 i=0;i<c.frictionPatchCount;i++)
	{
		const PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const FrictionPatch& frictionPatch = c.frictionPatches[i];
		PX_ASSERT(frictionPatch.anchorCount <= 2);

		const PxU32 firstPatch = c.correlationListHeads[i];
		const PxContactPoint* contactBase0 = buffer + c.contactPatches[firstPatch].start;

		SolverContactHeader* PX_RESTRICT header = reinterpret_cast<SolverContactHeader*>(ptr);
		ptr += sizeof(SolverContactHeader);		

		PxPrefetchLine(ptr, 128);
		PxPrefetchLine(ptr, 256);

		header->shapeInteraction = getInteraction(contactDesc);
		header->flags = flags;
		FStore(invMass0_dom0fV, &header->invMass0);
		FStore(FNeg(invMass1_dom1fV), &header->invMass1);
		const FloatV restitution = FLoad(contactBase0->restitution);
		const BoolV accelerationSpring = BLoad(!!(contactBase0->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING));

		const FloatV damping = FLoad(contactBase0->damping);
		const PxU32 pointStride = sizeof(SolverContactPoint);
		const PxU32 frictionStride = sizeof(SolverContactFriction);

		const Vec3V normal = V3LoadA(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
		const FloatV normalLenSq = V3LengthSq(normal);
		const VecCrossV norCross = V3PrepareCross(normal);
		const FloatV norVel = V3SumElems(V3NegMulSub(normal, linVel1, V3Mul(normal, linVel0)));

		const FloatV invMassNorLenSq0 = FMul(invMass0_dom0fV, normalLenSq);
		const FloatV invMassNorLenSq1 = FMul(invMass1_dom1fV, normalLenSq);

		header->normal_minAppliedImpulseForFrictionW = Vec4V_From_Vec3V(normal);
		
		for(PxU32 patch=c.correlationListHeads[i]; 
			patch!=CorrelationBuffer::LIST_END; 
			patch = c.contactPatches[patch].next)
		{
			const PxU32 count = c.contactPatches[patch].count;
			const PxContactPoint* contactBase = buffer + c.contactPatches[patch].start;
				
			PxU8* p = ptr;
			
			for(PxU32 j=0;j<count;j++)
			{
				PxPrefetchLine(p, 256);
				const PxContactPoint& contact = contactBase[j];

				SolverContactPoint* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPoint*>(p);
				p += pointStride;

				constructContactConstraint(invSqrtInertia0, invSqrtInertia1, invMassNorLenSq0, 
					invMassNorLenSq1, angD0, angD1, bodyFrame0p, bodyFrame1p,
					normal, norVel, norCross, angVel0, angVel1,
					invDt, invDtp8, dt, restDistance, maxPenBias, restitution,
					bounceThreshold, contact, *solverContact,
					ccdMaxSeparation, solverOffsetSlop, damping, accelerationSpring);
			}

			ptr = p;
		}

		PxF32* forceBuffers = reinterpret_cast<PxF32*>(ptr);
		PxMemZero(forceBuffers, sizeof(PxF32) * contactCount);
		ptr += ((contactCount + 3) & (~3)) * sizeof(PxF32); // jump to next 16-byte boundary
		
		const PxReal frictionCoefficient = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = contactBase0->staticFriction * frictionCoefficient;
		const PxReal dynamicFriction = contactBase0->dynamicFriction* frictionCoefficient;

		const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));

		const bool haveFriction = (disableStrongFriction == 0 && frictionPatch.anchorCount != 0);//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		header->numNormalConstr		= PxTo8(contactCount);
		header->numFrictionConstr	= PxTo8(haveFriction ? frictionPatch.anchorCount*2 : 0);
	
		header->type				= type;

		header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;
		FStore(angD0, &header->angDom0);
		FStore(angD1, &header->angDom1);

		header->broken = 0;

		if(haveFriction)
		{
			const Vec3V linVrel = V3Sub(linVel0, linVel1);
			//const Vec3V normal = Vec3V_From_PxVec3_Aligned(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

			const FloatV orthoThreshold = FLoad(0.70710678f);
			const FloatV p1 = FLoad(0.0001f);
			// fallback: normal.cross((1,0,0)) or normal.cross((0,0,1))
			const FloatV normalX = V3GetX(normal);
			const FloatV normalY = V3GetY(normal);
			const FloatV normalZ = V3GetZ(normal);
			
			const Vec3V t0Fallback1 = V3Merge(zero, FNeg(normalZ), normalY);
			const Vec3V t0Fallback2 = V3Merge(FNeg(normalY), normalX, zero);
			const Vec3V t0Fallback = V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			Vec3V t0 = V3Sub(linVrel, V3Scale(normal, V3Dot(normal, linVrel)));
			t0 = V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0 = V3Normalize(t0);

			const VecCrossV t0Cross = V3PrepareCross(t0);

			const Vec3V t1 = V3Cross(norCross, t0Cross);
			const VecCrossV t1Cross = V3PrepareCross(t1);
		
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

			header->frictionBrokenWritebackByte = writeback;

			const Vec3V v3Zero = V3Zero();
			for(PxU32 j = 0; j < frictionPatch.anchorCount; j++)
			{
				PxPrefetchLine(ptr, 256);
				PxPrefetchLine(ptr, 384);
				SolverContactFriction* PX_RESTRICT f0 = reinterpret_cast<SolverContactFriction*>(ptr);
				ptr += frictionStride;
				SolverContactFriction* PX_RESTRICT f1 = reinterpret_cast<SolverContactFriction*>(ptr);
				ptr += frictionStride;

				const Vec3V body0Anchor = V3LoadU_SafeReadW(frictionPatch.body0Anchors[j]);	// PT: see compile-time-assert in FrictionPatch
				const Vec3V body1Anchor = V3LoadU_SafeReadW(frictionPatch.body1Anchors[j]);	// PT: see compile-time-assert in FrictionPatch

				const Vec3V ra = QuatRotate(bodyFrame0q, body0Anchor);
				const Vec3V rb = QuatRotate(bodyFrame1q, body1Anchor);

				/*ra = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(ra)), v3Zero, ra);
				rb = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rb)), v3Zero, rb);*/

				PxU32 index = c.contactID[i][j];
				Vec3V error = V3Sub(V3Add(ra, bodyFrame0p), V3Add(rb, bodyFrame1p));
				error = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(error)), v3Zero, error);

				index = index == 0xFFFF ? c.contactPatches[c.correlationListHeads[i]].start : index;

				const Vec3V tvel = V3LoadA(buffer[index].targetVel);
				
				{
					Vec3V raXn = V3Cross(ra, t0Cross);
					Vec3V rbXn = V3Cross(rb, t0Cross);

					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					const Vec3V raXnSqrtInertia = M33MulV3(invSqrtInertia0, raXn);
					const Vec3V rbXnSqrtInertia = M33MulV3(invSqrtInertia1, rbXn);	

					const FloatV resp0 = FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const FloatV resp1 = FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);
					const FloatV resp = FAdd(resp0, resp1);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, zero), FDiv(p8, resp), zero);

					FloatV targetVel = V3Dot(tvel, t0);

					const FloatV vrel1 = FAdd(V3Dot(t0, linVel0), V3Dot(raXn, angVel0));
					const FloatV vrel2 = FAdd(V3Dot(t0, linVel1), V3Dot(rbXn, angVel1));
					const FloatV vrel = FSub(vrel1, vrel2);

					targetVel = FSub(targetVel, vrel);

					f0->normalXYZ_appliedForceW = V4ClearW(Vec4V_From_Vec3V(t0));
					f0->raXnXYZ_velMultiplierW = V4SetW(raXnSqrtInertia, velMultiplier);
					f0->rbXnXYZ_biasW = V4SetW(rbXnSqrtInertia, FMul(V3Dot(t0, error), invDt));
					FStore(targetVel, &f0->targetVel);
				}

				{
					Vec3V raXn = V3Cross(ra, t1Cross);
					Vec3V rbXn = V3Cross(rb, t1Cross);

					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					const Vec3V raXnSqrtInertia = M33MulV3(invSqrtInertia0, raXn);
					const Vec3V rbXnSqrtInertia = M33MulV3(invSqrtInertia1, rbXn);	

					const FloatV resp0 = FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const FloatV resp1 = FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);
					const FloatV resp = FAdd(resp0, resp1);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, zero), FDiv(p8, resp), zero);

					FloatV targetVel = V3Dot(tvel, t1);

					const FloatV vrel1 = FAdd(V3Dot(t1, linVel0), V3Dot(raXn, angVel0));
					const FloatV vrel2 = FAdd(V3Dot(t1, linVel1), V3Dot(rbXn, angVel1));
					const FloatV vrel = FSub(vrel1, vrel2);

					targetVel = FSub(targetVel, vrel);

					f1->normalXYZ_appliedForceW = V4ClearW(Vec4V_From_Vec3V(t1));
					f1->raXnXYZ_velMultiplierW = V4SetW(raXnSqrtInertia, velMultiplier);
					f1->rbXnXYZ_biasW = V4SetW(rbXnSqrtInertia, FMul(V3Dot(t1, error), invDt));
					FStore(targetVel, &f1->targetVel);
				}
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}

PX_FORCE_INLINE void computeBlockStreamByteSizes(const bool useExtContacts, const CorrelationBuffer& c,
								PxU32& _solverConstraintByteSize, PxU32& _frictionPatchByteSize, PxU32& _numFrictionPatches,
								PxU32& _axisConstraintCount)
{
	PX_ASSERT(0 == _solverConstraintByteSize);
	PX_ASSERT(0 == _frictionPatchByteSize);
	PX_ASSERT(0 == _numFrictionPatches);
	PX_ASSERT(0 == _axisConstraintCount);

	// PT: use local vars to remove LHS
	PxU32 solverConstraintByteSize = 0;
	PxU32 numFrictionPatches = 0;
	PxU32 axisConstraintCount = 0;

	for(PxU32 i = 0; i < c.frictionPatchCount; i++)
	{
		//Friction patches.
		if(c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
			numFrictionPatches++;

		const FrictionPatch& frictionPatch = c.frictionPatches[i];

		const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0;

		//Solver constraint data.
		if(c.frictionPatchContactCounts[i]!=0)
		{
			solverConstraintByteSize += sizeof(SolverContactHeader);
			solverConstraintByteSize += useExtContacts ? c.frictionPatchContactCounts[i] * sizeof(SolverContactPointExt) 
				: c.frictionPatchContactCounts[i] * sizeof(SolverContactPoint);
			solverConstraintByteSize += sizeof(PxF32) * ((c.frictionPatchContactCounts[i] + 3)&(~3)); //Add on space for applied impulses

			axisConstraintCount += c.frictionPatchContactCounts[i];

			if(haveFriction)
			{
				solverConstraintByteSize += useExtContacts ? c.frictionPatches[i].anchorCount * 2 * sizeof(SolverContactFrictionExt)
					: c.frictionPatches[i].anchorCount * 2 * sizeof(SolverContactFriction);
				axisConstraintCount += c.frictionPatches[i].anchorCount * 2;
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
						PxU32& axisConstraintCount, PxConstraintAllocator& constraintAllocator)
{
	PX_ASSERT(NULL == solverConstraint);
	PX_ASSERT(NULL == _frictionPatches);
	PX_ASSERT(0 == numFrictionPatches);
	PX_ASSERT(0 == solverConstraintByteSize);
	PX_ASSERT(0 == axisConstraintCount);

	//From frictionPatchStream we just need to reserve a single buffer.
	PxU32 frictionPatchByteSize = 0;
	//Compute the sizes of all the buffers.
	computeBlockStreamByteSizes(
		useExtContacts, cBuffer,
		solverConstraintByteSize, frictionPatchByteSize, numFrictionPatches,
		axisConstraintCount);

	//Reserve the buffers.

	//First reserve the accumulated buffer size for the constraint block.
	PxU8* constraintBlock = NULL;
	const PxU32 constraintBlockByteSize = solverConstraintByteSize;
	if(constraintBlockByteSize > 0)
	{
		constraintBlock = constraintAllocator.reserveConstraintData(constraintBlockByteSize + 16u);
		if(!checkConstraintDataPtr<false>(constraintBlock))
			constraintBlock = NULL;

		PX_ASSERT((size_t(constraintBlock) & 0xF) == 0);
	}

	FrictionPatch* frictionPatches = NULL;
	//If the constraint block reservation didn't fail then reserve the friction buffer too.
	if(frictionPatchByteSize >0 && (0==constraintBlockByteSize || constraintBlock))
	{
		frictionPatches = reinterpret_cast<FrictionPatch*>(constraintAllocator.reserveFrictionData(frictionPatchByteSize));
		if(!checkFrictionDataPtr(frictionPatches))
			frictionPatches = NULL;
	}

	_frictionPatches = frictionPatches;

	//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
	if(0==constraintBlockByteSize || constraintBlock)
	{
		if(solverConstraintByteSize)
		{
			solverConstraint = constraintBlock;
			PX_ASSERT(0==(uintptr_t(solverConstraint) & 0x0f));
		}
	}

	//Return true if neither of the two block reservations failed.
	return ((0==constraintBlockByteSize || constraintBlock) && (0==frictionPatchByteSize || frictionPatches));
}

bool createFinalizeSolverContacts(
	PxSolverContactDesc& contactDesc,
	CorrelationBuffer& c,
	const PxReal invDtF32,
	const PxReal dtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxConstraintAllocator& constraintAllocator,
	Cm::SpatialVectorF* Z)
{
	PxPrefetchLine(contactDesc.body0);
	PxPrefetchLine(contactDesc.body1);
	PxPrefetchLine(contactDesc.data0);
	PxPrefetchLine(contactDesc.data1);

	c.frictionPatchCount = 0;
	c.contactPatchCount = 0;

	const bool hasForceThreshold = contactDesc.hasForceThresholds;
	const bool staticOrKinematicBody = contactDesc.bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY || contactDesc.bodyState1 == PxSolverContactDesc::eSTATIC_BODY;

	const bool disableStrongFriction = contactDesc.disableStrongFriction;

	PxSolverConstraintDesc& desc = *contactDesc.desc;

	const bool useExtContacts = (desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY || desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY);

	desc.constraint = NULL;
	desc.constraintLengthOver16 = 0; // from here onwards we use this field for the constraint size, not the constraint type anymore

	if (contactDesc.numContacts == 0)
	{
		contactDesc.frictionPtr = NULL;
		contactDesc.frictionCount = 0;
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
		PxGetFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, PX_FL, "Dropping contacts in solver because we exceeded limit of 32 friction patches.");
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
		constraintAllocator);
	// initialise the work unit's ptrs to the various buffers.

	// patch up the work unit with the reserved buffers and set the reserved buffer data as appropriate.
	contactDesc.frictionPtr = NULL;
	contactDesc.frictionCount = 0;

	if (successfulReserve)
	{
		PxU8* frictionDataPtr = reinterpret_cast<PxU8*>(frictionPatches);
		contactDesc.frictionPtr = frictionDataPtr;
		desc.constraint = solverConstraint;
		//output.nbContacts = PxTo8(numContacts);
		contactDesc.frictionCount = PxTo8(numFrictionPatches);
		PX_ASSERT(solverConstraintByteSize % 16 == 0);
		desc.constraintLengthOver16 = PxTo16(solverConstraintByteSize / 16);
		desc.writeBack = contactDesc.contactForces;

		//Initialise friction buffer.
		if (frictionPatches)
		{
			frictionPatches->prefetch();

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
			const PxSolverBodyData& data0 = *contactDesc.data0;
			const PxSolverBodyData& data1 = *contactDesc.data1;
			if (useExtContacts)
			{
				const SolverExtBody b0(reinterpret_cast<const void*>(contactDesc.body0), reinterpret_cast<const void*>(&data0), desc.linkIndexA);
				const SolverExtBody b1(reinterpret_cast<const void*>(contactDesc.body1), reinterpret_cast<const void*>(&data1), desc.linkIndexB);

				setupFinalizeExtSolverContacts(contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
					b0, b1, invDtF32, dtF32, bounceThresholdF32,
					contactDesc.invMassScales.linear0, contactDesc.invMassScales.angular0, contactDesc.invMassScales.linear1, contactDesc.invMassScales.angular1, 
					contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, Z, contactDesc.offsetSlop);
			}
			else
			{
				setupFinalizeSolverConstraints(
					contactDesc,
					c,
					solverConstraint,
					data0, data1, invDtF32, dtF32, bounceThresholdF32,
					hasForceThreshold, staticOrKinematicBody,
					frictionDataPtr
					);
			}
			//KS - set to 0 so we have a counter for the number of times we solved the constraint
			//only going to be used on SPU but might as well set on all platforms because this code is shared
			*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
		}
	}

	return successfulReserve;
}

FloatV setupExtSolverContact(const SolverExtBody& b0, const SolverExtBody& b1,
	const FloatV& d0, const FloatV& d1, const FloatV& angD0, const FloatV& angD1, const Vec3V& bodyFrame0p, const Vec3V& bodyFrame1p,
	const Vec3VArg normal, const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg dt, const FloatVArg restDistance, 
	const FloatVArg maxPenBias, const FloatVArg restitution,const FloatVArg bounceThreshold, const PxContactPoint& contact, SolverContactPointExt& solverContact, const FloatVArg ccdMaxSeparation, Cm::SpatialVectorF* zVector,
	const Cm::SpatialVectorV& v0, const Cm::SpatialVectorV& v1, const FloatV& cfm, const Vec3VArg solverOffsetSlop,
	const FloatVArg norVel0, const FloatVArg norVel1, const FloatVArg damping, const BoolVArg accelerationSpring)
{
	const FloatV zero = FZero();
	const FloatV separation = FLoad(contact.separation);

	const FloatV penetration = FSub(separation, restDistance);

	const Vec3V point = V3LoadA(contact.point);

	const Vec3V ra = V3Sub(point, bodyFrame0p);
	const Vec3V rb = V3Sub(point, bodyFrame1p);

	Vec3V raXn = V3Cross(ra, normal);
	Vec3V rbXn = V3Cross(rb, normal);

	FloatV aVel0 = V3Dot(v0.angular, raXn);
	FloatV aVel1 = V3Dot(v1.angular, raXn);

	FloatV relLinVel = FSub(norVel0, norVel1);
	FloatV relAngVel = FSub(aVel0, aVel1);
	
	const Vec3V slop = V3Scale(solverOffsetSlop, FMax(FSel(FIsEq(relLinVel, zero), FMax(), FDiv(relAngVel, relLinVel)), FOne()));

	raXn = V3Sel(V3IsGrtr(slop, V3Abs(raXn)), V3Zero(), raXn);
	rbXn = V3Sel(V3IsGrtr(slop, V3Abs(rbXn)), V3Zero(), rbXn);

	aVel0 = V3Dot(raXn, v0.angular);
	aVel1 = V3Dot(rbXn, v1.angular);

	relAngVel = FSub(aVel0, aVel1);

	Cm::SpatialVectorV deltaV0, deltaV1;

	const Cm::SpatialVectorV resp0 = createImpulseResponseVector(normal, raXn, b0);
	const Cm::SpatialVectorV resp1 = createImpulseResponseVector(V3Neg(normal), V3Neg(rbXn), b1);

	const FloatV unitResponse = getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
		b1, resp1, deltaV1, d1, angD1, reinterpret_cast<Cm::SpatialVectorV*>(zVector));

	const FloatV vrel = FAdd(relLinVel, relAngVel);

	const FloatV penetrationInvDt = FMul(penetration, invDt);
	const BoolV isSeparated = FIsGrtrOrEq(penetration, zero);

	const BoolV collidingWithVrel = FIsGrtr(FNeg(vrel), penetrationInvDt); // true if (pen + dt*vrel) < 0
	const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), collidingWithVrel);

	FloatV velMultiplier, impulseMultiplier;
	FloatV biasedErr, unbiasedErr;

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

	if (FAllGrtr(zero, restitution))
	{
		computeCompliantContactCoefficients(dt, restitution, damping, recipResponse, unitResponse, penetration,
		                                    targetVelocity, accelerationSpring, isSeparated, collidingWithVrel,
		                                    velMultiplier, impulseMultiplier, unbiasedErr, biasedErr);
	}
	else
	{
		const BoolV ccdSeparationCondition = FIsGrtrOrEq(ccdMaxSeparation, penetration);
		velMultiplier = recipResponse;
		const FloatV penetrationInvDtScaled = FSel(isSeparated, penetrationInvDt, FMul(penetration, invDtp8));
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

bool createFinalizeSolverContacts(PxSolverContactDesc& contactDesc,
								  PxsContactManagerOutput& output,
								 ThreadContext& threadContext,
								 const PxReal invDtF32,
								 const PxReal dtF32,
								 PxReal bounceThresholdF32,
								 PxReal frictionOffsetThreshold,
								 PxReal correlationDistance,
								 PxConstraintAllocator& constraintAllocator,
								 Cm::SpatialVectorF* Z)
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

		bool hasMaxImpulse = false, hasTargetVelocity = false;

		numContacts = extractContacts(buffer, output, hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
			invInertiaScale0, invInertiaScale1, PxMin(contactDesc.data0->maxContactImpulse, contactDesc.data1->maxContactImpulse));

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

	return createFinalizeSolverContacts(contactDesc, c, invDtF32, dtF32, bounceThresholdF32, frictionOffsetThreshold,
		correlationDistance, constraintAllocator, Z);
}
  
PxU32 getContactManagerConstraintDesc(const PxsContactManagerOutput& cmOutput, const PxsContactManager& /*cm*/, PxSolverConstraintDesc& desc)
{
	desc.writeBack = cmOutput.contactForces;
	desc.writeBackFriction = NULL;
	return cmOutput.nbContacts;// cm.getWorkUnit().axisConstraintCount;
}

template
void updateFrictionAnchorCountAndPosition<PxSolverContactDesc>(PxSolverConstraintDesc& desc, PxsContactManagerOutput& output, PxSolverContactDesc& blockDesc);
template
void updateFrictionAnchorCountAndPosition<PxTGSSolverContactDesc>(PxSolverConstraintDesc& desc, PxsContactManagerOutput& output, PxTGSSolverContactDesc& blockDesc);
template <typename SolverContactDesc>
void updateFrictionAnchorCountAndPosition(PxSolverConstraintDesc& desc, PxsContactManagerOutput& output, SolverContactDesc& blockDesc)
{
	desc.writeBackFriction = NULL;
	if (output.frictionPatches == NULL) return;
	const PxReal NORMAL_THRESHOLD = 0.999f;
	PxTransform& bodyFrame0 = blockDesc.bodyFrame0;
	for (PxU32 frictionIndex = 0; frictionIndex < blockDesc.frictionCount; ++frictionIndex)
	{
		FrictionPatch& frictionPatch = reinterpret_cast<FrictionPatch*>(blockDesc.frictionPtr)[frictionIndex];
		PxVec3 frictionNormal = bodyFrame0.rotate(frictionPatch.body0Normal);
		for (PxU32 patchIndex = 0; patchIndex < output.nbPatches; ++patchIndex)
		{
			PxContactPatch& patch = reinterpret_cast<PxContactPatch*>(output.contactPatches)[patchIndex];
			if (patch.normal.dot(frictionNormal) > NORMAL_THRESHOLD &&
				patch.staticFriction == frictionPatch.staticFriction &&
				patch.dynamicFriction == frictionPatch.dynamicFriction &&
				patch.restitution == frictionPatch.restitution)
			{
				PxFrictionPatch& outPatch = reinterpret_cast<PxFrictionPatch*>(output.frictionPatches)[patchIndex];
				outPatch.anchorCount = frictionPatch.anchorCount;
				outPatch.anchorPositions[0] = bodyFrame0.transform(frictionPatch.body0Anchors[0]);
				outPatch.anchorPositions[1] = bodyFrame0.transform(frictionPatch.body0Anchors[1]);
				desc.writeBackFriction = outPatch.anchorImpulses;
				break;
			}
		}
	}
}

template
void writeBackContactFriction<SolverContactFrictionStep>(const SolverContactFrictionStep* PX_RESTRICT frictions, PxU32 numFrictionConstr, PxU32 frictionStride, PxVec3* PX_RESTRICT vFrictionWriteback);
template
void writeBackContactFriction<SolverContactFriction>(const SolverContactFriction* PX_RESTRICT frictions, PxU32 numFrictionConstr, PxU32 frictionStride, PxVec3* PX_RESTRICT vFrictionWriteback);
template <typename SolverFriction>
void writeBackContactFriction(const SolverFriction* PX_RESTRICT frictions, PxU32 numFrictionConstr, PxU32 frictionStride, PxVec3* PX_RESTRICT vFrictionWriteback)
{
	if (numFrictionConstr && vFrictionWriteback)
	{
		//We will have either 4 or 2 frictions (with friction pairs).
		//With torsional friction, we may have 3 (a single friction anchor + twist).
		const PxU32 numFrictionPairs = (numFrictionConstr & 6);
		const PxU8* ptr = reinterpret_cast<const PxU8*>(frictions);

		for (PxU32 i = 0; i < numFrictionPairs; i += 2)
		{
			const SolverFriction& f0 = *reinterpret_cast<const SolverFriction*>(ptr + (i + 0) * frictionStride);
			const SolverFriction& f1 = *reinterpret_cast<const SolverFriction*>(ptr + (i + 1) * frictionStride);
			const Vec3V normal0 = f0.getNormal();
			const Vec3V normal1 = f1.getNormal();
			const FloatV appliedForce0 = f0.getAppliedForce();
			const FloatV appliedForce1 = f1.getAppliedForce();
			Vec3V impulse = V3Add(V3Scale(normal0, appliedForce0), V3Scale(normal1, appliedForce1));
			PxVec3& frictionImpulse = vFrictionWriteback[i / 2];
			V3StoreU(impulse, frictionImpulse);
		}
	}
}

}

}
