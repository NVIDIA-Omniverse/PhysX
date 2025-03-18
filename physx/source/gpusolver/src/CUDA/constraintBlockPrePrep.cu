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

#include "foundation/PxVec4.h"

#include "PxContact.h"
#include "PxMaterial.h"

#include <stdio.h>
#include <assert.h>
#include "PxsContactManagerState.h"
#include "PxgCommonDefines.h"
#include "PxgConstraintBlock.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgSolverCoreDesc.h"
#include "PxgConstraintPrep.h"
#include "PxgSolverKernelIndices.h"
#include "PxgD6JointData.h"
#include "PxgConstraintHelper.h"
#include "PxgSolverBody.h"
#include "PxgDynamicsConfiguration.h"
#include "PxgEdgeType.h"
#include "PxgSolverFlags.h"
#include "PxsNphaseCommon.h"
#include "PxgSolverCoreDesc.h"
#include "PxgIslandContext.h"
#include "DyConstraintPrep.h"

#include "MemoryAllocator.cuh"
#include "vector_functions.h"
#include "reduction.cuh"
#include "constraintPrepShared.cuh"
#include "copy.cuh"

using namespace physx;
using namespace Cm;

extern "C" __host__ void initSolverKernels2() {}

struct MassModificationProps
{
	PxReal mInvMassScale0;
	PxReal mInvInertiaScale0;
	PxReal mInvMassScale1;
	PxReal mInvInertiaScale1;
};

/**
\brief Header for contact patch where all points share same material and normal
*/

//KS - this is used rather than PxContactPatch because we can't guarantee 16-byte alignment in the shared mem representation (in fact, we pad to ensure we don't get
//alignment to avoid bank conflicts, and using the aligned PxContactPatch type triggers aligned float4 loads that trigger crashes
struct ContactPatch
{
	MassModificationProps mMassModification;						//16

	PxVec3	normal;													//28
	PxReal	restitution;											//32

	PxReal	dynamicFriction;										//36
	PxReal	staticFriction;											//40
	PxReal	damping;												//44
	PxU16	startContactIndex;										//46
	PxU8	nbContacts;												//47
	PxU8	materialFlags;											//48

	PxU16	internalFlags;											//50
	PxU16	materialIndex0;											//52
	PxU16	materialIndex1;											//54
	PxU16	pad[5];													//64
};
PX_COMPILE_TIME_ASSERT(sizeof(ContactPatch) == sizeof(PxContactPatch));
PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(ContactPatch, pad) == PX_OFFSET_OF(PxContactPatch, pad));


PX_FORCE_INLINE PX_CUDA_CALLABLE float	computeSwingAngle(float swingYZ, float swingW)
{
	return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
}

static PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal computePhi(const PxQuat& q)
{
	PxQuat twist = q;
	twist.normalize();

	PxReal angle = twist.getAngle();
	if (twist.x<0.0f)
		angle = -angle;
	return angle;
}

static __device__ PxU32 createSolverContactConstraintDescsFromPatchWithSharedMem(
	PxgBlockContactData& prepData, PxgBlockContactPoint* blockContactPoints, PxgBlockWorkUnit& workUnit,
	PxgBlockConstraintBatch& /*batch*/, const PxU32 threadIndex,
	PxContactPatch* gContactPatch, PxContact* contactPoints, PxU32 endIndex,
	PxU32 prevFrictionPatchCount, PxU32& contactStartIndex, PxU32 numActiveThreads)
{  
	__shared__ void* srcPatch[PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK];
	__shared__ PxU32 shPatchBuff[PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK][16+1]; //+1 to avoid bank conflicts!
	//if(threadIndex >= startIndex && threadIndex < endIndex)
	const PxU32 nbThreadsPerPatch = 4;// sizeof(PxContactPatch) / sizeof(uint4);
	const PxU32 nbPatchesPerPass = WARP_SIZE / nbThreadsPerPatch;
	
	//We load 10 patches per-pass using 3 threads. This will mean that last 2 threads do nothing each pass when loading data...

	srcPatch[threadIdx.x] = threadIndex < endIndex ? gContactPatch : NULL;
	__syncwarp();

	//__threadfence_block();

	PxU32 warpMask = threadIdx.x&(~(WARP_SIZE-1));

	for(PxU32 i = 0; i < endIndex; i+= nbPatchesPerPass)
	{
		const PxU32 pairToLoad = i + threadIndex / nbThreadsPerPatch;
		const PxU32 srcLane = warpMask + pairToLoad;
		//PxU32* p = reinterpret_cast<PxU32*>(srcPatch[srcLane]);
		uint4* p = srcLane < numActiveThreads ? reinterpret_cast<uint4*>(srcPatch[srcLane]) : NULL;

		PxU32 copyIndex = threadIndex & (nbThreadsPerPatch - 1);

		if (p)
		{
			uint4 val = p[copyIndex];
			shPatchBuff[srcLane][copyIndex * 4] = val.x;
			shPatchBuff[srcLane][copyIndex * 4 + 1] = val.y;
			shPatchBuff[srcLane][copyIndex * 4 + 2] = val.z;
			shPatchBuff[srcLane][copyIndex * 4 + 3] = val.w;
		}
	}

	

	PxU32 numContacts = 0;
	PxU32 stride = 1;

	__syncwarp();

	//__threadfence_block();

	if (threadIndex < endIndex)
	{
		//PxContactHeader* header = reinterpret_cast<PxContactHeader*>(contactStream);

		PxgMaterialContactData data;
		//PxU32 startInd = 0;
		if (gContactPatch)
		{
			//PxContactPatch* contactPatch = reinterpret_cast<PxContactPatch*>(contactPatches_);
			//float4 lin0X_ang0Y_lin1Z_ang1W = make_float4(1.f);

			//PxU32 dominanceFlags = workUnit.mInternalFlags[threadIndex];

			ContactPatch* contactPatch = reinterpret_cast<ContactPatch*>(shPatchBuff[threadIdx.x]);

			PxU32 flags = workUnit.mFlags[threadIndex];

			PxU32 internalFlags = contactPatch->internalFlags;

			bool perPointFriction = internalFlags & PxContactPatch::eHAS_TARGET_VELOCITY || workUnit.mFlags[threadIndex] & PxgNpWorkUnitFlag::eDISABLE_STRONG_FRICTION;
			bool disableFriction = false;
			bool isAccelerationSpring = false;

			//is it modifiable or not?
			if ((internalFlags & PxContactPatch::eFORCE_NO_RESPONSE) == 0)
			{
				const bool isModifiable = internalFlags & PxContactPatch::eCOMPRESSED_MODIFIED_CONTACT; //KS - TODO - if we get here, we should not have modifiable data!!! It should have been pruned already!
				//PxU32 headerSize = isModifiable ? sizeof(PxModifyContactHeader) : sizeof(PxContactHeader);

				//PxContactPatch* contactPatch = reinterpret_cast<PxContactPatch*>(contactStream + headerSize + patchIndex * sizeof(PxContactPatch));

				contactPoints += contactPatch->startContactIndex;
				if (isModifiable)
				{
					contactPoints += contactPatch->startContactIndex;
					stride = 2;
				}

				numContacts = contactPatch->nbContacts;
				contactStartIndex = contactPatch->startContactIndex;

				

				float4 lin0X_ang0Y_lin1Z_ang1W = make_float4(contactPatch->mMassModification.mInvMassScale0, contactPatch->mMassModification.mInvInertiaScale0,
					contactPatch->mMassModification.mInvMassScale1, contactPatch->mMassModification.mInvInertiaScale1);

				float4 normal_restitution = make_float4(contactPatch->normal.x, contactPatch->normal.y, contactPatch->normal.z, contactPatch->restitution);

				if (flags & PxgNpWorkUnitFlag::eDOMINANCE_0)
				{
					lin0X_ang0Y_lin1Z_ang1W.x = 0.f;
					lin0X_ang0Y_lin1Z_ang1W.y = 0.f;
				}
				if (flags & PxgNpWorkUnitFlag::eDOMINANCE_1)
				{
					lin0X_ang0Y_lin1Z_ang1W.z = 0.f;
					lin0X_ang0Y_lin1Z_ang1W.w = 0.f;
				}
				prepData.mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W = lin0X_ang0Y_lin1Z_ang1W;

				prepData.normal_restitutionW[threadIndex] = normal_restitution;
				prepData.damping[threadIndex] = contactPatch->damping;

				data.dynamicFriction = contactPatch->dynamicFriction;
				data.staticFriction = contactPatch->staticFriction;
				disableFriction = contactPatch->materialFlags & PxMaterialFlag::eDISABLE_FRICTION;
				isAccelerationSpring = contactPatch->materialFlags & PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING;
			}

			PxU8 solverFlags = 0;
			if (perPointFriction)
				solverFlags |= PxgSolverContactFlags::ePER_POINT_FRICTION;
			if (flags & PxgNpWorkUnitFlag::eFORCE_THRESHOLD)
				solverFlags |= PxgSolverContactFlags::eHAS_FORCE_THRESHOLDS;
			if (disableFriction)
				solverFlags |= PxgSolverContactFlags::eDISABLE_FRICTION;
			if(isAccelerationSpring)
				solverFlags |= PxgSolverContactFlags::eCOMPLIANT_ACCELERATION_SPRING;
			data.mSolverFlags = solverFlags;
		}

		data.mNumContacts = numContacts & 0xFF;
		data.prevFrictionPatchCount = PxU8(prevFrictionPatchCount);
		data.restDistance = workUnit.mRestDistance[threadIndex];

		reinterpret_cast<float4&>(prepData.contactData[threadIndex]) = reinterpret_cast<float4&>(data);
	}

	{
		//Now copy contacts...

		srcPatch[threadIdx.x] = threadIndex < endIndex ? contactPoints : NULL;
		//accumulator[threadIdx.x] = threadIndex < endIndex ? numContacts * stride : 0;
		PxU32 contactSize = numContacts*stride;
		__syncwarp();

		//__threadfence_block();

		PxU32 maxCount = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, contactSize);
		maxCount = __shfl_sync(FULL_MASK, maxCount, 31);


		const PxU32 nbIters = (maxCount + 3)/4; //Enough space to store 4 contacts per pass, per pair

		PxU32 contactIndex = 0;

		for(PxU32 i = 0, copiedBytes = 0; i < nbIters; ++i, copiedBytes += sizeof(float4)*4)
		{
			for(PxU32 a = 0; a < endIndex; a += 8)
			{
				const PxU32 srcLane0 = a + threadIndex / nbThreadsPerPatch;
				const PxU32 srcLane = warpMask + srcLane0;
				uint4* p = srcLane < numActiveThreads ? reinterpret_cast<uint4*>(srcPatch[srcLane]) : NULL;

				PxU32 nbBytesToCopy = __shfl_sync(FULL_MASK, contactSize, srcLane0)*sizeof(float4);
				//__syncwarp(); //Should not be needed here!

				if(p && nbBytesToCopy > copiedBytes)
				{
					PxU32 nbWordsToCopy = (nbBytesToCopy - copiedBytes)/sizeof(uint4);
					PxU32 copyIndex = threadIndex&(nbThreadsPerPatch-1);

					if(copyIndex < nbWordsToCopy)
					{
						uint4 val = p[copyIndex + copiedBytes/sizeof(uint4)];
						shPatchBuff[srcLane][copyIndex*4] = val.x;
						shPatchBuff[srcLane][copyIndex*4+1] = val.y;
						shPatchBuff[srcLane][copyIndex*4+2] = val.z;
						shPatchBuff[srcLane][copyIndex*4+3] = val.w;
					}
				}
				//
			}

			__syncwarp();
			//__threadfence_block();

			PxU32 count = (numContacts * stride);
			PxU32 startIndex = i*4;
			count = PxMin(count, 4 + startIndex);

			PxVec4* points = reinterpret_cast<PxVec4*>(shPatchBuff[threadIdx.x]);

			for(PxU32 a = startIndex; a < count; a+=stride)
			{
				PxU32 readInd = a - startIndex;
				PxgBlockContactPoint& p = blockContactPoints[contactIndex++];
				p.point_separationW[threadIndex] = make_float4(points[readInd].x, points[readInd].y, points[readInd].z, points[readInd].w);
				if(stride != 1)
					p.targetVel_maxImpulseW[threadIndex] = make_float4(points[readInd+1].x, points[readInd+1].y, points[readInd+1].z, points[readInd+1].w);
				else
					p.targetVel_maxImpulseW[threadIndex] = make_float4(0.f, 0.f, 0.f, PX_MAX_REAL);
			}

			__syncwarp(); //Required according to racecheck. From ptr points is read above but the next iteration of the for loop will already write to shPatchBuff (ptr points to shPatchBuff)
		}
	}

	return 0;
}

//Ks - magic structs to try and take advantage of aligned loads...

//static __device__ PxU32 createSolverContactConstraintDescsFromPatch(PxgBlockContactData& prepData, PxgBlockContactPoint* blockContactPoints, PxgBlockWorkUnit& workUnit, 
//													PxgConstraintBatch& batch, const PxU32 threadIndex,
//													PxContactPatch* contactPatch, PxContact* contactPoints, PxU32 endIndex,
//													PxU32 prevFrictionPatchCount)
//{  
//	
//	if(threadIndex < endIndex)
//	{
//
//		PxU32 numContacts = 0;
//
//		//PxContactHeader* header = reinterpret_cast<PxContactHeader*>(contactStream);
//		
//	
//		PxgMaterialContactData data;
//		//PxU32 startInd = 0;
//		if(contactPatch)
//		{
//			//PxContactPatch* contactPatch = reinterpret_cast<PxContactPatch*>(contactPatches_);
//			//float4 lin0X_ang0Y_lin1Z_ang1W = make_float4(1.f);
//			
//			//PxU32 dominanceFlags = workUnit.mInternalFlags[threadIndex];
//			PxU32 flags = workUnit.mFlags[threadIndex];
//
//			PxU32 internalFlags = contactPatch->internalFlags;
//
//			PxU32 startIndex = contactPatch->startContactIndex;
//
//			contactPoints += startIndex;
//
//			bool perPointFriction = internalFlags & PxContactPatch::eHAS_TARGET_VELOCITY || workUnit.mFlags[threadIndex] & PxgNpWorkUnitFlag::eDISABLE_STRONG_FRICTION;
//			bool disableFriction = false;
//
//			
//			//is it modifiable or not?
//			if((internalFlags & PxContactPatch::eFORCE_NO_RESPONSE) == 0)
//			{
//				const bool isModifiable = internalFlags & PxContactPatch::eCOMPRESSED_MODIFIED_CONTACT; //KS - TODO - if we get here, we should not have modifiable data!!! It should have been pruned already!
//				//PxU32 headerSize = isModifiable ? sizeof(PxModifyContactHeader) : sizeof(PxContactHeader);
//
//				if(isModifiable)
//					contactPoints += startIndex;
//
//				//PxContactPatch* contactPatch = reinterpret_cast<PxContactPatch*>(contactStream + headerSize + patchIndex * sizeof(PxContactPatch));
//
//				float4 lin0X_ang0Y_lin1Z_ang1W = *((float4*)&contactPatch->mMassModification);
//
//
//				float4 normal_restitution = *((float4*)&contactPatch->normal);
//				
//
//				if(flags & PxgNpWorkUnitFlag::eDOMINANCE_0)
//				{
//					lin0X_ang0Y_lin1Z_ang1W.x = 0.f;
//					lin0X_ang0Y_lin1Z_ang1W.y = 0.f;
//				}
//				if(flags & PxgNpWorkUnitFlag::eDOMINANCE_1)
//				{
//					lin0X_ang0Y_lin1Z_ang1W.z = 0.f;
//					lin0X_ang0Y_lin1Z_ang1W.w = 0.f;
//				}
//				prepData.mInvMassScale[threadIndex].lin0X_ang0Y_lin1Z_ang1W = lin0X_ang0Y_lin1Z_ang1W;
//
//				prepData.normal_restitutionW[threadIndex] = normal_restitution;
//				
//				data.dynamicFriction = contactPatch->dynamicFriction;
//				data.staticFriction = contactPatch->staticFriction;
//				disableFriction = contactPatch->materialFlags & PxMaterialFlag::eDISABLE_FRICTION;
//				//prepData.materialIndex0[threadIndex] = base.materialIndex0;
//				//prepData.materialIndex1[threadIndex] = base.materialIndex1;
//				//prepData.materialFlags[threadIndex] = base.flags;
//
//				//const PxU32 pointStride = sizeof(PxContact);
//
//				//PxU8* contactPtr = contactPoints_;
//				float4* contactPtr = reinterpret_cast<float4*>(contactPoints);
//
//				const PxU32 stride = isModifiable ? 2 : 1;
//				const PxU32 count = contactPatch->nbContacts * stride;
//
//				for(PxU32 a = 0; a < count; a+=stride)
//				{
//					PxgBlockContactPoint& p = blockContactPoints[numContacts++];
//					p.point_separationW[threadIndex] = contactPtr[a];
//					if(isModifiable)
//					{
//						p.targetVel_maxImpulseW[threadIndex] = contactPtr[a+1];
//					}
//					else
//					{
//						p.targetVel_maxImpulseW[threadIndex] = make_float4(0.f, 0.f, 0.f, PX_MAX_REAL);
//					}
//				}
//			}
//
//			PxU8 solverFlags = 0;
//			if(perPointFriction)
//				solverFlags |= PxgBlockSolverContactHeader::ePER_POINT_FRICTION;
//			if(flags & PxgNpWorkUnitFlag::eFORCE_THRESHOLD)
//				solverFlags |= PxgBlockSolverContactHeader::eHAS_FORCE_THRESHOLDS;
//			if(disableFriction)
//				solverFlags |= PxgBlockSolverContactHeader::eDISABLE_FRICTION;
//			data.mSolverFlags = solverFlags;
//		}
//			
//		assert(numContacts <= 64);
//		data.mNumContacts = numContacts & 0xFF;
//		data.prevFrictionPatchCount = PxU8(prevFrictionPatchCount);
//		data.restDistance =  workUnit.mRestDistance[threadIndex];	
//
//		reinterpret_cast<float4&>(prepData.contactData[threadIndex]) = reinterpret_cast<float4&>(data);		
//	}
//	return 0;
//}


static PX_CUDA_CALLABLE PxU32 setupConeSwingLimits(Px1DConstraint* constraint, PxU32 constraintCount, const PxgConstraintHelper& ch, const PxgD6JointData& data, const PxQuat& swing, const PxTransform& cA2w)
{
	PxVec3 axis;
	PxReal error;
	const ConeLimitHelperTanLess coneHelper(data.swingLimit.yAngle, data.swingLimit.zAngle);
	coneHelper.getLimit(swing, axis, error);
	ch.angularLimit(&constraint[constraintCount++], cA2w.rotate(axis), error, data.swingLimit);

	return constraintCount;
}

static PX_CUDA_CALLABLE PxU32 setupPyramidSwingLimits(Px1DConstraint* constraint, PxU32 constraintCount, PxgConstraintHelper& ch, const PxgD6JointData& data, const PxQuat& swing, const PxTransform& cA2w, bool useY, bool useZ)
{
	const PxQuat q = cA2w.q * swing;
	const PxgJointLimitPyramid& l = data.pyramidSwingLimit;
	if (useY)
		constraintCount = ch.anglePair(constraint, constraintCount, computeSwingAngle(swing.y, swing.w), l.yAngleMin, l.yAngleMax, q.getBasisVector1(), l);
	if (useZ)
		constraintCount = ch.anglePair(constraint, constraintCount, computeSwingAngle(swing.z, swing.w), l.zAngleMin, l.zAngleMax, q.getBasisVector2(), l);

	return constraintCount;
}

static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 setupSingleSwingLimit(Px1DConstraint* constraints, PxU32 constraintCount, PxgConstraintHelper& ch, const PxgD6JointData& data, const PxVec3& axis, float swingYZ, float swingW, float swingLimitYZ)
{
	return ch.anglePair(constraints,constraintCount, computeSwingAngle(swingYZ, swingW), -swingLimitYZ, swingLimitYZ, axis, data.swingLimit);
}

static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 setupDualConeSwingLimits(Px1DConstraint* constraints, PxU32 constraintCount, PxgConstraintHelper& ch, const PxgD6JointData& data, const PxVec3& axis, float sin, float swingLimitYZ)
{
	return ch.anglePair(constraints, constraintCount, PxAsin(sin), -swingLimitYZ, swingLimitYZ, axis.getNormalized(), data.swingLimit);
}

static PX_CUDA_CALLABLE PxU32 setupLinearLimit(Px1DConstraint* constraints, PxU32 constraintCount, PxgConstraintHelper& ch, const PxgJointLinearLimitPair& limit, const float origin, const PxVec3& axis)
{
	constraintCount = ch.linearLimit(constraints, constraintCount, axis, origin, limit.upper, limit);
	constraintCount = ch.linearLimit(constraints, constraintCount, -axis, -origin, -limit.lower, limit);
	return constraintCount;
}

PX_FORCE_INLINE PX_CUDA_CALLABLE void applyNeighborhoodOperator(const PxTransform& cA2w, PxTransform& cB2w)
{
	if (cA2w.q.dot(cB2w.q) < 0.0f)	// minimum dist quat (equiv to flipping cB2bB.q, which we don't use anywhere)
		cB2w.q = -cB2w.q;
}

static __device__ PxU32 D6JointSolverPrep(Px1DConstraint* constraints, const PxgD6JointData& data, 
											const PxTransform& bA2w, const PxTransform& bB2w,
											bool useExtendedLimits,
											PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	PxTransform cA2w, cB2w;
	PxgConstraintHelper g(cA2w, cB2w, data, bA2w, bB2w);

	const PxU32 SWING1_FLAG = 1<<PxgD6Axis::eSWING1, 
		SWING2_FLAG = 1<<PxgD6Axis::eSWING2, 
		TWIST_FLAG  = 1<<PxgD6Axis::eTWIST;

	const PxU32 ANGULAR_MASK = SWING1_FLAG | SWING2_FLAG | TWIST_FLAG;
	const PxU32 LINEAR_MASK = (1<<PxgD6Axis::eX) | (1<<PxgD6Axis::eY) | (1<<PxgD6Axis::eZ);

	const PxgD6JointDrive* drives = data.drive;
	PxU32 locked = data.locked, limited = data.limited, driving = data.driving;

	//const PxVec3 tBody0WorldOffset = cA2w.p-bA2w.p;
	//body0WorldOffset = tBody0WorldOffset;
	PxU32 constraintCount = 0;

	if (!useExtendedLimits)
		applyNeighborhoodOperator(cA2w, cB2w);

	const PxTransform cB2cA = cA2w.transformInv(cB2w);	

	assert(data.c2b[0].isValid());
	assert(data.c2b[1].isValid());
	assert(cA2w.isValid());
	assert(cB2w.isValid());
	assert(cB2cA.isValid());

	PxMat33 cA2w_m(cA2w.q), cB2w_m(cB2w.q);

	// handy for swing computation
	PxVec3 bX = cB2w_m.column0, aY = cA2w_m.column1, aZ = cA2w_m.column2;

	if(driving & ((1<<PxgD6Drive::eX)|(1<<PxgD6Drive::eY)|(1<<PxgD6Drive::eZ)))
	{
		// TODO: make drive unilateral if we are outside the limit
		const PxVec3 posErr = data.drivePosition.p - cB2cA.p;
		for(PxU32 i = 0; i < 3; i++)
		{
			// -driveVelocity because velTarget is child (body1) - parent (body0) and Jacobian is 1 for body0 and -1 for parent
			if(driving & (1<<(PxgD6Drive::eX+i)))
				g.linear(&constraints[constraintCount++], cA2w_m[i], -data.driveLinearVelocity[i], posErr[i], drives[PxgD6Drive::eX+i]); 
		}
	}

	if(driving & ((1<<PxgD6Drive::eSLERP)|(1<<PxgD6Drive::eSWING)|(1<<PxgD6Drive::eTWIST)))
	{
		const PxQuat d2cA_q = cB2cA.q.dot(data.drivePosition.q)>0 ? data.drivePosition.q : -data.drivePosition.q; 

		const PxVec3& v = data.driveAngularVelocity;
		const PxQuat delta = d2cA_q.getConjugate() * cB2cA.q;

		if(driving & (1<<PxgD6Drive::eSLERP))
		{
			const PxVec3 velTarget = -cA2w.rotate(data.driveAngularVelocity);

			PxVec3 axis[3] = { PxVec3(1.f,0,0), PxVec3(0,1.f,0), PxVec3(0,0,1.f) };
				
			if(drives[PxgD6Drive::eSLERP].stiffness!=0)
				computeJacobianAxes(axis, cA2w.q * d2cA_q, cB2w.q);	// converges faster if there is only velocity drive

			for(PxU32 i = 0; i < 3; i++)
				g.angular(&constraints[constraintCount++], axis[i], axis[i].dot(velTarget), -delta.getImaginaryPart()[i], drives[PxgD6Drive::eSLERP], PxConstraintSolveHint::eSLERP_SPRING);
		}
		else 
		{
			if(driving & (1<<PxgD6Drive::eTWIST))
				g.angular(&constraints[constraintCount++], cA2w_m.column0, v.x, -2.0f * delta.x, drives[PxgD6Drive::eTWIST]); 

			if(driving & (1<<PxgD6Drive::eSWING))
			{
				const PxVec3 err = delta.getBasisVector0();

				if(!(locked & SWING1_FLAG))
					g.angular(&constraints[constraintCount++], aY, v.y, err.z, drives[PxgD6Drive::eSWING]);

				if(!(locked & SWING2_FLAG))
					g.angular(&constraints[constraintCount++], aZ, v.z, -err.y, drives[PxgD6Drive::eSWING]);
			}
		}
	}

	if (limited & ANGULAR_MASK)
	{
		PxQuat swing, twist;
		PxSeparateSwingTwist(cB2cA.q, swing, twist);

		// swing limits: if just one is limited: if the other is free, we support 
		// (-pi/2, +pi/2) limit, using tan of the half-angle as the error measure parameter. 
		// If the other is locked, we support (-pi, +pi) limits using the tan of the quarter-angle
		// Notation: th == PxTanHalf, tq = tanQuarter

		if(limited & SWING1_FLAG && limited & SWING2_FLAG)
		{
			if (data.mUseConeLimit)
				constraintCount = setupConeSwingLimits(constraints, constraintCount, g, data, swing, cA2w);

			// PT: no else here by design, we want to allow creating both the cone & the pyramid at the same time,
			// which can be useful to make the cone more robust against large velocities.
			if (data.mUsePyramidLimits)
				constraintCount = setupPyramidSwingLimits(constraints, constraintCount, g, data, swing, cA2w, true, true);
		}
		else
		{
			if(limited & SWING1_FLAG)
			{
				if(locked & SWING2_FLAG)
				{
					//constraintCount = g.quarterAnglePair(constraints, constraintCount, PxTanHalf(swing.y, swing.w), -data.tqSwingY, data.tqSwingY, tqPad, aY, limit);

					if (data.mUsePyramidLimits)
						constraintCount = setupPyramidSwingLimits(constraints, constraintCount, g, data, swing, cA2w, true, false);
					else
						constraintCount = setupSingleSwingLimit(constraints, constraintCount, g, data, aY, swing.y, swing.w, data.swingLimit.yAngle);			// PT: swing Y limited, swing Z locked
				}
				else
				{
					constraintCount = setupDualConeSwingLimits(constraints, constraintCount, g, data, aZ.cross(bX), -aZ.dot(bX), data.swingLimit.yAngle);	// PT: swing Y limited, swing Z free
				}
			}
			if(limited & SWING2_FLAG)
			{
				if(locked & SWING1_FLAG)
				{
					if (data.mUsePyramidLimits)
						constraintCount = setupPyramidSwingLimits(constraints, constraintCount, g, data, swing, cA2w, false, true);
					else
						constraintCount = setupSingleSwingLimit(constraints, constraintCount, g, data, aZ, swing.z, swing.w, data.swingLimit.zAngle);			// PT: swing Z limited, swing Y locked
				}
				else
				{
					constraintCount = setupDualConeSwingLimits(constraints, constraintCount, g, data, -aY.cross(bX), aY.dot(bX), data.swingLimit.zAngle);	// PT: swing Z limited, swing Y free
				}
			}
		}

		if(limited & TWIST_FLAG)
		{
			constraintCount = g.anglePair(constraints, constraintCount, computePhi(twist), data.twistLimit.lower, data.twistLimit.upper, cB2w_m.column0, data.twistLimit);
		}
	}

	if(limited & LINEAR_MASK)
	{
		if (data.mUseDistanceLimit)
		{
			PxVec3 limitDir = PxVec3(0);

			for (PxU32 i = 0; i < 3; i++)
			{
				if (limited & (1 << (PxgD6Axis::eX + i)))
					limitDir += cA2w_m[i] * cB2cA.p[i];
			}

			PxReal distance = limitDir.magnitude();
			if (distance > data.distanceMinDist)
				constraintCount = g.linearLimit(constraints, constraintCount, limitDir * (1.0f / distance), distance, data.distanceLimit.value, data.distanceLimit);
		}

		if (data.mUseNewLinearLimits)	// PT: new asymmetric linear limits
		{
			const PxVec3& bOriginInA = cB2cA.p;

			// PT: TODO: we check that the DOFs are not "locked" to be consistent with the prismatic joint, but it
			// doesn't look like this case is possible, since it would be caught by the "isValid" check when setting
			// the limits. And in fact the "distance" linear limit above doesn't do this check.
			if ((limited & (1 << PxgD6Axis::eX)) && data.linearLimitX.lower <= data.linearLimitX.upper)
				constraintCount = setupLinearLimit(constraints, constraintCount, g, data.linearLimitX, bOriginInA.x, cA2w_m.column0);

			if ((limited & (1 << PxgD6Axis::eY)) && data.linearLimitY.lower <= data.linearLimitY.upper)
				constraintCount = setupLinearLimit(constraints, constraintCount, g, data.linearLimitY, bOriginInA.y, cA2w_m.column1);

			if ((limited & (1 << PxgD6Axis::eZ)) && data.linearLimitZ.lower <= data.linearLimitZ.upper)
				constraintCount = setupLinearLimit(constraints, constraintCount, g, data.linearLimitZ, bOriginInA.z, cA2w_m.column2);
		}
	}

	// we handle specially the case of just one swing dof locked

	PxU32 angularLocked = locked & ANGULAR_MASK;

	if(angularLocked == SWING1_FLAG)
	{
		g.angularHard(&constraints[constraintCount++], bX.cross(aZ), -bX.dot(aZ));
		locked &= ~SWING1_FLAG;
	}
	else if(angularLocked == SWING2_FLAG)
	{
		locked &= ~SWING2_FLAG;
		g.angularHard(&constraints[constraintCount++], bX.cross(aY), -bX.dot(aY));
	}

	cB2wOut = cB2w.p;

	PxVec3 ra;

	constraintCount = g.prepareLockedAxes(constraints, constraintCount, cA2w.q, cB2w.q, cB2cA.p,locked&7, locked>>3, ra);
	cA2wOut = ra + bA2w.p;

	return constraintCount;
}

extern "C" __global__ void constraintContactBlockPrePrepLaunch(PxgPrePrepDesc* gDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	__shared__ PxgBlockContactPoint* volatile blockContactPoints[PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK/32];

	__shared__ PxgPrePrepDesc shDesc;

	/*if(threadIdx.x < (sizeof(PxgPrePrepDesc)/sizeof(PxU32)))
	{
		PxU32* shAddress = (PxU32*)&shDesc;
		shAddress[threadIdx.x] = ((PxU32*)gDesc)[threadIdx.x];
	}*/

	warpCopy<PxU32>((PxU32*)&shDesc, (PxU32*)gDesc, sizeof(PxgPrePrepDesc));

	__syncthreads();	

	//KS - TODO - is this the same as the count in the island? It should be, but need to make sure!
	const PxU32 nbArticulations = shDesc.mTotalActiveArticulations;

	const PxgBodySim* const PX_RESTRICT bodySims = sharedDesc->mBodySimBufferDeviceData;

	const PxU32* const PX_RESTRICT solverBodyIndices = shDesc.solverBodyIndices;

	PxU32* sharedFrictionIndex = &gDesc->sharedFrictionConstraintIndex; //Must be global as this is the shared index!
	PxU32* sharedContactIndex = &gDesc->sharedContactConstraintIndex; //Must be global as this is the shared index!

	float2* torsionalData = reinterpret_cast<float2*>(gDesc->mTorsionalFrictionData);

	const PxU32 blockStride = blockDim.x/WARP_SIZE;

	const PxU32 warpIndexInBlock = threadIdx.x/WARP_SIZE;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = blockIdx.x * blockStride + warpIndexInBlock;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	//total numbers of warps in all blocks
//	const PxU32 totalNumWarps = blockStride * gridDim.x;

	const PxU32 numBatches = shDesc.numBatches;
	const PxU32 numStaticBatches = shDesc.numStaticBatches;
	const PxU32 numArtiBatches = shDesc.numArtiBatches;
	const PxU32 numArtiStaticBatches = shDesc.numArtiStaticBatches;
	const PxU32 numArtiSelfBatches = shDesc.numArtiSelfBatches;
	const PxU32 dynamicBatchEnd = numBatches + numArtiBatches;
	const PxU32 totalStaticPlusDynamic = dynamicBatchEnd + numArtiStaticBatches;
	const PxU32 totalBatches = totalStaticPlusDynamic + numArtiSelfBatches + numStaticBatches;

	const PxU32 nbElemsPerBody = gDesc->nbElementsPerBody;

	const PartitionIndexData* const PX_RESTRICT partitionIndexData = shDesc.mPartitionIndices;

	//for(PxU32 a = warpIndex; a < numBatches; a+=totalNumWarps)  
	PxU32 a = warpIndex;
	if(a < totalBatches)
	{
		const PxI32 numActiveWarpsInBlock = PxClamp(PxI32(totalBatches) - PxI32(blockIdx.x * blockStride), 0, PxI32(blockStride));

		PxgBlockConstraintBatch& batch = shDesc.blockBatches[a];	
		PxgConstraintBatchHeader batchHeader = shDesc.mBatchHeaders[a];
		
		PxU32 conType = batchHeader.constraintType;

		if(conType == PxgSolverConstraintDesc::eCONTACT ||
			conType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
		{			
			//Copy across data from batchHeader!!!
			batch.constraintType = batchHeader.constraintType;
			batch.mConstraintBatchIndex = batchHeader.mConstraintBatchIndex;
			batch.mDescStride = batchHeader.mDescStride;
			batch.mask = batchHeader.mask;

			/*if(threadIndexInWarp == 0)
				printf("warpIndex %i conType %i constraintBatchIndex %i descStride %i\n", warpIndex, conType, batchHeader.mConstraintBatchIndex, batchHeader.mDescStride);*/

			PxgBlockWorkUnit& n = shDesc.blockWorkUnit[batchHeader.mConstraintBatchIndex];

			PxgBlockContactData& contactBlockPrepData = shDesc.blockContactData[batchHeader.mConstraintBatchIndex];

			//__shared__ volatile PxU32 accumulator[PxgKernelBlockDim::CONSTRAINT_PREPREP_BLOCK];

			PxContactPatch* contactPatch = NULL;
			PxContact* contacts = NULL;
			
			PxU32 prevFrictionPatchCount = 0;

			PxU32 contactCount = 0;
			PxU32 forceIndex = 0xFFFFFFFF;
			n.mFrictionPatchIndex[threadIndexInWarp] = 0xFFFFFFFF;

			if(threadIndexInWarp < batchHeader.mDescStride)
			{
				PxU32 uniqueIndex = conType == PxgSolverConstraintDesc::eCONTACT ? shDesc.mContactUniqueIndices[batchHeader.mStartPartitionIndex + threadIndexInWarp] :
				shDesc.mArtiContactUniqueIndices[batchHeader.mStartPartitionIndex + threadIndexInWarp];

				//printf("Pre-prep a = %i, uniqueIndex = %i, batchHeader.mStartPartitionIndex = %i, batchHeader.mConstraintBatchIndex %i\n", a, uniqueIndex, batchHeader.mStartPartitionIndex, batchHeader.mConstraintBatchIndex);

				const PxgSolverConstraintManagerConstants* const PX_RESTRICT constants = &shDesc.mContactConstantData[uniqueIndex];

				const PartitionNodeData& nodeData = shDesc.mPartitionNodeData[uniqueIndex];
				PxNodeIndex igNodeIndexA = nodeData.mNodeIndex0;
				PxNodeIndex igNodeIndexB = nodeData.mNodeIndex1;

				/*uint4 nodeData = reinterpret_cast<uint4*>(shDesc.mPartitionNodeData)[uniqueIndex];

				IG::NodeIndex igNodeIndexA = *reinterpret_cast<IG::NodeIndex*>(&nodeData.x);
				IG::NodeIndex igNodeIndexB = *reinterpret_cast<IG::NodeIndex*>(&nodeData.y);*/

				const PxU32 nodeIndexA = igNodeIndexA.index();
				const PxU32 nodeIndexB = igNodeIndexB.index();
				
				/*PxU32 nextIndexA = nodeData.z;
				PxU32 nextIndexB = nodeData.w;*/

				const PxU32 nextIndexA = nodeData.mNextIndex[0];
				const PxU32 nextIndexB = nodeData.mNextIndex[1];

				const bool isStaticA = nodeIndexA == PX_INVALID_NODE;
				const bool isStaticB = nodeIndexB == PX_INVALID_NODE;

				const bool isArticulationA = igNodeIndexA.isArticulation();
				const bool isArticulationB = igNodeIndexB.isArticulation();

				/*if(batchHeader.mConstraintBatchIndex == 30)
					printf("warpIndex %i threadIdx %i isArticulationA %i isArticulationB %i\n", warpIndex, threadIndexInWarp, isArticulationA, isArticulationB);*/


				//KS - TODO - this value needs to be replaced with the articulation remap ID.
				//We can do this in the articulation prep code!
				PxU32 solverBodyIndexA = isStaticA ? 0 : solverBodyIndices[nodeIndexA];
				PxU32 solverBodyIndexB = isStaticB ? 0 : solverBodyIndices[nodeIndexB];

				batch.bodyAIndex[threadIndexInWarp] = solverBodyIndexA;
				batch.bodyBIndex[threadIndexInWarp] = solverBodyIndexB;

				batch.bodyANodeIndex[threadIndexInWarp] = igNodeIndexA;
				batch.bodyBNodeIndex[threadIndexInWarp] = igNodeIndexB;

				//printf("Contact batch %i nodeIndexA = (%i, %i), nodeIndexB = (%i, %i)\n", a, igNodeIndexA.index(),
				//	igNodeIndexA.articulationLinkId(), igNodeIndexB.index(), igNodeIndexB.articulationLinkId());

				PxU32 patchIndex = partitionIndexData[uniqueIndex].mPatchIndex;

				const PxU32 maxConstraintPartitions = gDesc->mMaxConstraintPartitions;
				const PxU32 nbSlabs = gDesc->mTotalSlabs;

				PxU32 currentPartition = partitionIndexData[uniqueIndex].mPartitionIndex;

				PxU32 slabId = currentPartition/maxConstraintPartitions;

				batch.slabId[threadIndexInWarp] = slabId;

				//Static articulation contacts don't have all the data in the partitioning assigned to be able to compute the data below
				if (a < dynamicBatchEnd)
				{
					if (!isArticulationA)
					{
						batch.remappedBodyAIndex[threadIndexInWarp] = computeRemapIndexRigidBody(nextIndexA & 1, shDesc.mPartitionstartBatchIndices, shDesc.mPartitionArtiStartBatchIndices,
							shDesc.mPartitionJointCounts, shDesc.mPartitionArtiJointCounts,
							partitionIndexData[nextIndexA >> 1], shDesc.mSolverBodyReferences, currentPartition, shDesc.mMaxConstraintPartitions, shDesc.mTotalActiveBodies,
							solverBodyIndexA, shDesc.mActiveBodyStartOffset, numBatches, numArtiBatches, nbElemsPerBody, nbSlabs, sharedDesc->deltaOutOffset);
					}
					else
					{
						//KS - fill in here!
						PxU32 artiId = bodySims[nodeIndexA].articulationRemapId;
						batch.remappedBodyAIndex[threadIndexInWarp] = artiId;

						PxU32 partitionId = currentPartition&(maxConstraintPartitions-1);
						const PxU32 ind = solverBodyIndexA + (slabId + partitionId*nbSlabs)*nbArticulations;

						sharedDesc->articulationSlabMask[ind].x = igNodeIndexA.articulationLinkId();
						sharedDesc->articulationSlabMask[ind].y = a*WARP_SIZE*2* nbElemsPerBody + threadIndexInWarp;
					}

					if (!isArticulationB)
					{
						batch.remappedBodyBIndex[threadIndexInWarp] = computeRemapIndexRigidBody(nextIndexB & 1, shDesc.mPartitionstartBatchIndices, shDesc.mPartitionArtiStartBatchIndices,
							shDesc.mPartitionJointCounts, shDesc.mPartitionArtiJointCounts,
							partitionIndexData[nextIndexB >> 1], shDesc.mSolverBodyReferences, currentPartition, shDesc.mMaxConstraintPartitions, shDesc.mTotalActiveBodies,
							solverBodyIndexB, shDesc.mActiveBodyStartOffset, numBatches, numArtiBatches, nbElemsPerBody, nbSlabs, sharedDesc->deltaOutOffset);
					}
					else
					{
						//KS - fill in here!
						batch.remappedBodyBIndex[threadIndexInWarp] = bodySims[nodeIndexB].articulationRemapId;

						PxU32 partitionId = currentPartition&(maxConstraintPartitions - 1);

						const PxU32 ind = solverBodyIndexB + (slabId + partitionId*nbSlabs)*nbArticulations;

						sharedDesc->articulationSlabMask[ind].z = igNodeIndexB.articulationLinkId();
						sharedDesc->articulationSlabMask[ind].w = (a*WARP_SIZE * 2 + WARP_SIZE)* nbElemsPerBody + threadIndexInWarp;
					}
				}
				else
				{
					/*printf("%i: Static contact with between %i and %i, uniqueIndex = %i, batchHeader.mStartPartitionIndex = %i\n", 
						threadIndexInWarp, nodeIndexA, nodeIndexB, uniqueIndex, batchHeader.mStartPartitionIndex);*/
					if (isArticulationA)
					{
						batch.remappedBodyAIndex[threadIndexInWarp] = bodySims[nodeIndexA].articulationRemapId;
					}
					if (isArticulationB)
					{
						batch.remappedBodyBIndex[threadIndexInWarp] = bodySims[nodeIndexB].articulationRemapId;
					}
				}
				
				const PxU32 edgeIndex = constants->mEdgeIndex;
				n.mEdgeIndex[threadIndexInWarp] = edgeIndex;

				n.mPatchIndex[threadIndexInWarp] = patchIndex;

				const PxU32 npIndex = shDesc.mNpOutputIndices[uniqueIndex];

				//PxU32 cmOutputIndex = shDesc.mCmOutputOffsets[npIndex & ((1<<MaxBucketBits)-1)] + (npIndex >> MaxBucketBits);

				const PxU32 tIndex = PxsContactManagerBase::computeBucketIndexFromId(npIndex);
				const PxU32 cmOutputIndex = shDesc.mCmOutputOffsets[tIndex] + (npIndex >> PxsContactManagerBase::MaxBucketBits);

				batch.shapeInteraction[threadIndexInWarp] = shDesc.mShapeInteractions[cmOutputIndex];
				n.mRestDistance[threadIndexInWarp] = shDesc.mRestDistances[cmOutputIndex];
				n.mTorsionalFrictionData[threadIndexInWarp] = torsionalData[cmOutputIndex];
				
				const PxsContactManagerOutput* const PX_RESTRICT cmOutput = &shDesc.contactManagerOutputBase[cmOutputIndex];

				n.mFlags[threadIndexInWarp] = cmOutput->flags;

				const PxU32 totalContacts = cmOutput->nbContacts;

				const bool hasContacts = totalContacts != 0;

				PxU32 patchStartIndex = reinterpret_cast<PxContactPatch*>(cmOutput->contactPatches) - shDesc.cpuCompressedPatchesBase;
				PxU32 contactIndex = reinterpret_cast<PxContact*>(cmOutput->contactPoints) - shDesc.cpuCompressedContactsBase;
				if (cmOutput->contactForces)
					forceIndex = reinterpret_cast<PxReal*>(cmOutput->contactForces) - shDesc.cpuForceBufferBase;

				prevFrictionPatchCount = shDesc.prevFrictionPatchCount[edgeIndex];

				shDesc.currFrictionPatchCount[edgeIndex] = cmOutput->nbPatches;

				if (hasContacts && patchIndex < cmOutput->nbPatches)
				{
					if(cmOutput->contactPatches != NULL)
						contactPatch = shDesc.compressedPatches + patchStartIndex + patchIndex;

					n.mFrictionPatchIndex[threadIndexInWarp] = patchStartIndex + patchIndex;

					assert(contactPatch->nbContacts <= PXG_MAX_NUM_POINTS_PER_CONTACT_PATCH);

					if(cmOutput->contactPoints != NULL)
						contacts = shDesc.compressedContacts + contactIndex;

					contactCount = contactPatch->nbContacts;

				}
			}

			

			//accumulator[threadIdx.x] = contactCount;
			//__syncwarp();

			PxU32 maxCount = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, contactCount);

			
			if(threadIndexInWarp == 0)
			{
				//PxU32 maxCount = accumulator[threadIdx.x];

				PxU32 contactIndex = atomicAdd(sharedContactIndex, maxCount);

				PxgBlockContactPoint* results = shDesc.blockContactPoints + contactIndex;

				blockContactPoints[warpIndexInBlock] = results;

				PxU32 startConstraintIndex = contactIndex;
				batch.startConstraintIndex = startConstraintIndex;
				batch.blockContactIndex = startConstraintIndex;

				PxU32 maxFrictions = 4u;
				batch.startFrictionIndex = (PxU32)(atomicAdd(sharedFrictionIndex, (PxI32)maxFrictions));

				if (batchHeader.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
				{
					PxU32 startResponse = (PxU32)(atomicAdd(&gDesc->sharedArticulationResponseIndex, PxI32(maxFrictions + maxCount)));
					batch.mArticulationResponseIndex = startResponse;
				}
				

				//store the gpu pointer to the batch so that other kernel can access those pointer
			}
			__syncwarp();

			PxU32 contactStartIndex = 0;

			createSolverContactConstraintDescsFromPatchWithSharedMem(contactBlockPrepData, blockContactPoints[warpIndexInBlock], n, batch,
				threadIndexInWarp, contactPatch, contacts,
				batchHeader.mDescStride, prevFrictionPatchCount, contactStartIndex, WARP_SIZE * numActiveWarpsInBlock);

			n.mWriteback[threadIndexInWarp] = forceIndex + contactStartIndex;

			/*createSolverContactConstraintDescsFromPatch(contactBlockPrepData, blockContactPoints[warpIndexInBlock], n, batch,
				threadIndexInWarp, contactPatch, contacts,
				batch.mDescStride, prevFrictionPatchCount);*/
		}
	}
}

static const bool gConstraintHasArticulationLink = true;  // only used for template instantiation to describe the meaning of the boolean parameter

//
// tHasArticulationLink defines whether the PxConstraint has at least one articulation link among the two connected actors
//
template<bool tHasArticulationLink>
static PX_FORCE_INLINE __device__  void constraint1DPrePrep(PxU32 jointDataIndex, 
	const PxgD6JointData* jointDataEntries, const PxgConstraintPrePrep* constraintPrePrepEntries,
	const PxU32* solverBodyIndices, const PxgSolverBodyData* solverBodyData,
	const PxgBodySim* bodySimEntries, const PxgArticulation* articulations,
	PxgConstraintData* constraintDataEntries, Px1DConstraint* constraintRows)
{
	const PxgD6JointData& jointData = jointDataEntries[jointDataIndex];

	const PxgConstraintPrePrep& constraintPrePre = constraintPrePrepEntries[jointDataIndex];

	const bool disableConstraints = constraintPrePre.mFlags & PxConstraintFlag::eDISABLE_CONSTRAINT;
	const bool useExtendedLimits = constraintPrePre.mFlags & PxConstraintFlag::eENABLE_EXTENDED_LIMITS;

	const PxNodeIndex igNodeIndexA = constraintPrePre.mNodeIndexA;
	const PxNodeIndex igNodeIndexB = constraintPrePre.mNodeIndexB;

	const PxU32 nodeIndexA = igNodeIndexA.index();
	const PxU32 nodeIndexB = igNodeIndexB.index();

	//ML - if both indices are PX_INVALID_NODE, this indicates that this joint has been removed
	if (nodeIndexA != PX_INVALID_NODE || nodeIndexB != PX_INVALID_NODE)
	{
		PxTransform pose0, pose1;

		if (tHasArticulationLink)
		{
			const PxgBodySim* const PX_RESTRICT bodySims = bodySimEntries;

			const bool isArticulationA = igNodeIndexA.isArticulation();
			const bool isArticulationB = igNodeIndexB.isArticulation();

			if (isArticulationA)
			{
				const PxU32 articulationId = bodySims[nodeIndexA].articulationRemapId;
				const PxU32 linkId = igNodeIndexA.articulationLinkId();
				const PxgArticulation& arti = articulations[articulationId];

				pose0 = arti.linkBody2Worlds[linkId];
			}
			else
			{
				const PxU32 solverBodyIndexA = nodeIndexA == PX_INVALID_NODE ? 0 : solverBodyIndices[nodeIndexA];
				const PxAlignedTransform pose0_ = solverBodyData[solverBodyIndexA].body2World;
				pose0 = pose0_.getTransform();
			}

			if (isArticulationB)
			{
				const PxU32 articulationId = bodySims[nodeIndexB].articulationRemapId;
				const PxU32 linkId = igNodeIndexB.articulationLinkId();
				const PxgArticulation& arti = articulations[articulationId];

				pose1 = arti.linkBody2Worlds[linkId];
			}
			else
			{
				const PxU32 solverBodyIndexB = nodeIndexB == PX_INVALID_NODE ? 0 : solverBodyIndices[nodeIndexB];
				const PxAlignedTransform pose1_ = solverBodyData[solverBodyIndexB].body2World;
				pose1 = pose1_.getTransform();
			}
		}
		else
		{
			PX_UNUSED(bodySimEntries);
			PX_UNUSED(articulations);

			const PxU32 solverBodyIndexA = nodeIndexA == PX_INVALID_NODE ? 0 : solverBodyIndices[nodeIndexA];
			const PxU32 solverBodyIndexB = nodeIndexB == PX_INVALID_NODE ? 0 : solverBodyIndices[nodeIndexB];

			const PxAlignedTransform pose0_ = solverBodyData[solverBodyIndexA].body2World;
			const PxAlignedTransform pose1_ = solverBodyData[solverBodyIndexB].body2World;

			pose0 = pose0_.getTransform();
			pose1 = pose1_.getTransform();
		}

		const PxU32 constraintsStartIndex = jointDataIndex * Dy::MAX_CONSTRAINT_ROWS;
		Px1DConstraint* constraintRowStart = constraintRows + constraintsStartIndex;
		PxU32 numRows;
		PxVec3 ra, rb;

		if (!disableConstraints)
		{
			PxVec3 ca2w, cb2w;

			numRows = D6JointSolverPrep(constraintRowStart,
				jointData,
				pose0, pose1,
				useExtendedLimits,
				ca2w, cb2w);

			ra = ca2w - pose0.p;
			rb = cb2w - pose1.p;
		}
		else
		{
			numRows = 0;

			ra.x = ra.y = ra.z = 0.0f;
			rb.x = rb.y = rb.z = 0.0f;
		}

		constraintDataEntries[jointDataIndex].mInvMassScale = jointData.invMassScale;
		constraintDataEntries[jointDataIndex].mRaWorld_linBreakForceW = make_float4(ra.x, ra.y, ra.z, constraintPrePre.mLinBreakForce);
		constraintDataEntries[jointDataIndex].mRbWorld_angBreakForceW = make_float4(rb.x, rb.y, rb.z, constraintPrePre.mAngBreakForce);
		constraintDataEntries[jointDataIndex].mNumRows_Flags_StartIndex = make_uint4(numRows, constraintPrePre.mFlags, constraintsStartIndex, 0);
	}
}

extern "C" __global__ void constraint1DPrePrepLaunch(PxgPrePrepDesc* desc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	const PxgD6JointData* gJointData = desc->rigidJointData;
	const PxgConstraintPrePrep* gConstraintPrePrep =  desc->rigidConstraintPrePrep;
	PxgConstraintData*	gConstraintData =  desc->constraintData; // output
	Px1DConstraint* gConstraintRows = desc->constraintRows;		 // output

	const PxgD6JointData* gArtiJointData = desc->artiJointData;
	const PxgConstraintPrePrep* gArtiConstraintPrePrep = desc->artiConstraintPrePrep;
	PxgConstraintData*	gArtiConstraintData = desc->artiConstraintData;	 // output
	Px1DConstraint*		gArtiConstraintRows = desc->artiConstraintRows;  // output

	const PxgSolverBodyData* solverBodyData = desc->mSolverBodyData;

	const PxU32* solverBodyIndices = desc->solverBodyIndices;

	const PxU32 nbRigidJoints = desc->nbD6RigidJoints; 
	const PxU32 nbArtiJoints = desc->nbD6ArtiJoints;

	const PxU32 nbTotalJoints = nbRigidJoints + nbArtiJoints;
	const PxU32 threadIndex = threadIdx.x + blockIdx.x * blockDim.x;

	if(threadIndex < nbTotalJoints)
	{
		if (threadIndex < nbRigidJoints)
		{
			constraint1DPrePrep<!gConstraintHasArticulationLink>(
				threadIndex, gJointData, gConstraintPrePrep,
				solverBodyIndices, solverBodyData,
				NULL, NULL,
				gConstraintData, gConstraintRows);
		}
		else
		{
			//articulation
			const PxU32 index = threadIndex - nbRigidJoints;

			assert(index < desc->nbTotalArtiJoints);

			constraint1DPrePrep<gConstraintHasArticulationLink>(
				index, gArtiJointData, gArtiConstraintPrePrep,
				solverBodyIndices, solverBodyData,
				sharedDesc->mBodySimBufferDeviceData, sharedDesc->articulations,
				gArtiConstraintData, gArtiConstraintRows);
		}
	}
}

//This method is called after constraint1DPrePrepLaunch. Because in GPU, we just support D6 joint so other joint type's pre-prepare work need to be done in CPU. After that, we need to
//append the CPU result at the end of the GPU result(PxgConstraintData) and create a block format for the contactConstraintBlockPrepareParallelLaunch
extern "C" __global__ void constraint1DBlockPrePrepLaunch(
	PxgPrePrepDesc* gDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{

	//printf("constraint1DBlockPrePrepLaunch \n");

	__shared__ PxgPrePrepDesc shDesc;

	warpCopy<PxU32>((PxU32*)&shDesc, (PxU32*)gDesc, sizeof(PxgPrePrepDesc));

	__syncthreads();

	const PxU32 numBatches = shDesc.numBatches;
	const PxU32 numArtiBatches = shDesc.numArtiBatches;
	const PxU32 numArtiStaticBatches = shDesc.numArtiStaticBatches;
	const PxU32 numArtiSelfBatches = shDesc.numArtiSelfBatches;
	const PxU32 totalDynamicBatches = numBatches + numArtiBatches;
	const PxU32 totalBatches = totalDynamicBatches + numArtiStaticBatches + numArtiSelfBatches + shDesc.numStaticBatches;

	PxU32* sharedJointRowIndex = &gDesc->sharedJointRowIndex;

	const PxU32 blockStride = blockDim.x/WARP_SIZE;

	const PxU32 warpIndexInBlock = threadIdx.x/WARP_SIZE;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = blockIdx.x * blockStride + warpIndexInBlock;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE-1);

	const PxU32 nbElemsPerBody = gDesc->nbElementsPerBody;

	const PxU32 nbArticulations = shDesc.mTotalActiveArticulations;

	const PxU32 nbSlabs = gDesc->mTotalSlabs;

	const PxgBodySim* const PX_RESTRICT bodySims = sharedDesc->mBodySimBufferDeviceData;

	//total numbers of warps in all blocks
//	const PxU32 totalNumWarps = blockStride * gridDim.x;

	//for(PxU32 a = warpIndex; a < numBatches; a+=totalNumWarps)  
	PxU32 a = warpIndex;
	unsigned mask_numBatches = __ballot_sync(FULL_MASK, a < totalBatches);
	if(a < totalBatches)
	{
		//if (threadIndexInWarp == 0)
		//	printf("a = %i, totalBatches = %i\n", a, totalBatches);
		PxgBlockConstraintBatch& batch = shDesc.blockBatches[a];
		PxgConstraintBatchHeader batchHeader = shDesc.mBatchHeaders[a];
	
		PxU32 conType = batchHeader.constraintType;

		/*if(threadIndexInWarp == 0)
			printf("ConType = %i\n", conType);*/
		//KS - technically, this will match mast_numBatches
		//unsigned mask_eCONSTRAINT_1D = __ballot_sync(mask_numBatches, batchHeader.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D);
		if(conType == PxgSolverConstraintDesc::eCONSTRAINT_1D ||
			conType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D)
		{
			batch.constraintType = batchHeader.constraintType;
			batch.mConstraintBatchIndex = batchHeader.mConstraintBatchIndex;
			batch.mDescStride = batchHeader.mDescStride;
			batch.mStartPartitionIndex = batchHeader.mStartPartitionIndex;
			batch.mask = batchHeader.mask;

			PxU32 outputRowStartIndex = batchHeader.mConstraintBatchIndex*physx::Dy::MAX_CONSTRAINT_ROWS;

			//for joint
			PxgConstraintData*	gPrepData = shDesc.constraintData; // GPU input data
			Px1DConstraint* rows = shDesc.constraintRows;
			//for articulation external joint
			PxgConstraintData* artiConstraintData = shDesc.artiConstraintData;
			Px1DConstraint* artiRows = shDesc.artiConstraintRows;
			PxgBlockConstraint1DData& gBlockPrepData = shDesc.blockPrepData[batchHeader.mConstraintBatchIndex];	//GPU output data	
			PxgBlockConstraint1DVelocities* gBlockPrepVelocityData = &shDesc.blockPrepVelocityData[outputRowStartIndex];		//GPU output data
			PxgBlockConstraint1DParameters* gBlockPrepParameterData = &shDesc.blockPrepParameterData[outputRowStartIndex];	//GPU output data

			PxU32 numRows = 0;
			if(threadIndexInWarp < batchHeader.mDescStride)
			{
			//	//PxU32 uniqueId = shDesc.mConstraintUniqueIndices[batchHeader.mStartPartitionIndex + threadIndexInWarp];
				PxU32 uniqueId = conType == PxgSolverConstraintDesc::eCONSTRAINT_1D ? shDesc.mConstraintUniqueIndices[batchHeader.mStartPartitionIndex + threadIndexInWarp] :
					shDesc.mArtiConstraintUniqueIndices[batchHeader.mStartPartitionIndex + threadIndexInWarp];

				const PartitionNodeData& nodeData = shDesc.mPartitionNodeData[uniqueId];

				PxNodeIndex igNodeIndexA = nodeData.mNodeIndex0;
				PxNodeIndex igNodeIndexB = nodeData.mNodeIndex1;

				/*uint4 nodeData = reinterpret_cast<uint4*>(shDesc.mPartitionNodeData)[uniqueId];

				IG::NodeIndex igNodeIndexA = *reinterpret_cast<IG::NodeIndex*>(&nodeData.x);
				IG::NodeIndex igNodeIndexB = *reinterpret_cast<IG::NodeIndex*>(&nodeData.y);*/

				const bool isArticulationA = igNodeIndexA.isArticulation();
				const bool isArticulationB = igNodeIndexB.isArticulation();

				//printf("uniqueId %i isArticulationA %i isArticulationB %i\n", uniqueId, isArticulationA, isArticulationB);

				const PxU32 nodeIndexA = igNodeIndexA.index();
				const PxU32 nodeIndexB = igNodeIndexB.index();

				/*PxU32 nextIndexA = nodeData.z;
				PxU32 nextIndexB = nodeData.w;*/
				const PxU32 nextIndexA = nodeData.mNextIndex[0];
				const PxU32 nextIndexB = nodeData.mNextIndex[1];

				PxU32 solverBodyIndexA = nodeIndexA == PX_INVALID_NODE ? 0 : shDesc.solverBodyIndices[nodeIndexA];
				PxU32 solverBodyIndexB = nodeIndexB == PX_INVALID_NODE ? 0 : shDesc.solverBodyIndices[nodeIndexB];

				batch.bodyAIndex[threadIndexInWarp] = solverBodyIndexA;
				batch.bodyBIndex[threadIndexInWarp] = solverBodyIndexB;

				batch.bodyANodeIndex[threadIndexInWarp] = igNodeIndexA;
				batch.bodyBNodeIndex[threadIndexInWarp] = igNodeIndexB;

				/*printf("Joint batch %i nodeIndexA = (%i, %i), nodeIndexB = (%i, %i)\n", a, igNodeIndexA.index(),
					igNodeIndexA.articulationLinkId(), igNodeIndexB.index(), igNodeIndexB.articulationLinkId());*/

				const PxU32 currentPartition = shDesc.mPartitionIndices[uniqueId].mPartitionIndex;

				const PxU32 maxConstraintPartitions = gDesc->mMaxConstraintPartitions;

				PxU32 slabId = currentPartition / maxConstraintPartitions;

				batch.slabId[threadIndexInWarp] = slabId;

				//Static articulation constraint don't have all the data in the partitioning assigned to be able to compute the data below
				if (a < totalDynamicBatches)
				{
					if (!isArticulationA)
					{
						batch.remappedBodyAIndex[threadIndexInWarp] = computeRemapIndexRigidBody(nextIndexA & 1, shDesc.mPartitionstartBatchIndices, shDesc.mPartitionArtiStartBatchIndices,
							shDesc.mPartitionJointCounts, shDesc.mPartitionArtiJointCounts,
							shDesc.mPartitionIndices[nextIndexA >> 1], shDesc.mSolverBodyReferences, currentPartition, shDesc.mMaxConstraintPartitions, shDesc.mTotalActiveBodies, solverBodyIndexA,
							shDesc.mActiveBodyStartOffset, numBatches, numArtiBatches, nbElemsPerBody, nbSlabs, sharedDesc->deltaOutOffset);
					}
					else
					{
						batch.remappedBodyAIndex[threadIndexInWarp] = bodySims[nodeIndexA].articulationRemapId;
						PxU32 partitionId = currentPartition&(maxConstraintPartitions - 1);

						const PxU32 ind = solverBodyIndexA + (slabId + partitionId*nbSlabs)*nbArticulations;

						sharedDesc->articulationSlabMask[ind].x = igNodeIndexA.articulationLinkId();
						sharedDesc->articulationSlabMask[ind].y = (a*WARP_SIZE * 2)* nbElemsPerBody + threadIndexInWarp;
					}

					if (!isArticulationB)
					{
						batch.remappedBodyBIndex[threadIndexInWarp] = computeRemapIndexRigidBody(nextIndexB & 1, shDesc.mPartitionstartBatchIndices, shDesc.mPartitionArtiStartBatchIndices,
							shDesc.mPartitionJointCounts, shDesc.mPartitionArtiJointCounts,
							shDesc.mPartitionIndices[nextIndexB >> 1], shDesc.mSolverBodyReferences, currentPartition, shDesc.mMaxConstraintPartitions, shDesc.mTotalActiveBodies, solverBodyIndexB,
							shDesc.mActiveBodyStartOffset, numBatches, numArtiBatches, nbElemsPerBody, nbSlabs, sharedDesc->deltaOutOffset);
					}
					else
					{
						batch.remappedBodyBIndex[threadIndexInWarp] = bodySims[nodeIndexB].articulationRemapId;
						PxU32 partitionId = currentPartition&(maxConstraintPartitions - 1);

						const PxU32 ind = solverBodyIndexB + (slabId + partitionId*nbSlabs)*nbArticulations;

						sharedDesc->articulationSlabMask[ind].z = igNodeIndexB.articulationLinkId();
						sharedDesc->articulationSlabMask[ind].w = (a*WARP_SIZE * 2 + WARP_SIZE)* nbElemsPerBody + threadIndexInWarp;
					}
				}
				else
				{
					if (isArticulationA)
					{
						batch.remappedBodyAIndex[threadIndexInWarp] = bodySims[nodeIndexA].articulationRemapId;
					}
					if (isArticulationB)
					{
						batch.remappedBodyBIndex[threadIndexInWarp] = bodySims[nodeIndexB].articulationRemapId;
					}
				}

				const PxU32 index = shDesc.mNpOutputIndices[uniqueId]; //KS - doubles up as joint indices for joints!

				const bool hasArticulations = isArticulationA || isArticulationB;
				PxgConstraintData& constraintData = hasArticulations ? artiConstraintData[index] : gPrepData[index];
				//write to PxgBlockConstraint1DData
				const float4 raWorld_linBreakForceW = constraintData.mRaWorld_linBreakForceW;
				const float4 rbWorld_angBreakForceW = constraintData.mRbWorld_angBreakForceW;
				
				//printf("raWorld (%f, %f, %f)\n", raWorld_linBreakForceW.x, raWorld_linBreakForceW.y, raWorld_linBreakForceW.z);
				const uint4 mNumRows_Flags_StartIndex = constraintData.mNumRows_Flags_StartIndex;
				numRows = mNumRows_Flags_StartIndex.x;
				const uint rowStartIndex = mNumRows_Flags_StartIndex.z;
				gBlockPrepData.mInvMassScale[threadIndexInWarp] = constraintData.mInvMassScale;
				gBlockPrepData.mRAWorld_linBreakForce[threadIndexInWarp] = raWorld_linBreakForceW;
				gBlockPrepData.mRBWorld_AngBreakForce[threadIndexInWarp] = rbWorld_angBreakForceW;
				gBlockPrepData.mFlags[threadIndexInWarp] = mNumRows_Flags_StartIndex.y;
				
				gBlockPrepData.mNumRows[threadIndexInWarp] = numRows;

				Px1DConstraint* tRows = hasArticulations ? artiRows : rows;

				Px1DConstraint* startRows = tRows + rowStartIndex;

				//write to PxgBlockConstraint1DVelocities
				for (PxU32 i = 0; i<numRows; ++i)
				{
					const Px1DConstraint& constraint1D = startRows[i];

					gBlockPrepVelocityData[i].linear0XYZ_geometricErrorW[threadIndexInWarp] = reinterpret_cast<const float4&>(constraint1D.linear0);
					gBlockPrepVelocityData[i].angular0XYZ_velocityTargetW[threadIndexInWarp] = reinterpret_cast<const float4&>(constraint1D.angular0);
					gBlockPrepVelocityData[i].linear1XYZ_minImpulseW[threadIndexInWarp] = reinterpret_cast<const float4&>(constraint1D.linear1);
					gBlockPrepVelocityData[i].angular1XYZ_maxImpulseW[threadIndexInWarp] = reinterpret_cast<const float4&>(constraint1D.angular1);
					
					gBlockPrepParameterData[i].flags[threadIndexInWarp] = constraint1D.flags;
					gBlockPrepParameterData[i].mods.bounce.restitution[threadIndexInWarp] = constraint1D.mods.bounce.restitution;
					gBlockPrepParameterData[i].mods.bounce.velocityThreshold[threadIndexInWarp] = constraint1D.mods.bounce.velocityThreshold;
					gBlockPrepParameterData[i].solveHint[threadIndexInWarp] = constraint1D.solveHint;
				}
			}
			
			numRows = warpScan<MaxOpPxU32, int>(mask_numBatches, numRows);

			//we need to update the sharedJointRowIndex
			if(threadIndexInWarp == (WARP_SIZE -1))
			{
				//Atomic add 
				batch.startConstraintIndex  = atomicAdd(sharedJointRowIndex, numRows);

				if (conType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D)
				{
					PxU32 startResponse = (PxU32)(atomicAdd(&gDesc->sharedArticulationResponseIndex, PxI32(numRows)));
					batch.mArticulationResponseIndex = startResponse;
				}
			}
		}
	}
}

extern "C" __global__ void clearFrictionPatchCounts(PxU32* frictionPatchCount, PxU32* edgeIndicesToClear, PxU32 nbToClear)
{
	const PxU32 threadIndex = threadIdx.x + blockIdx.x * blockDim.x;

	if(threadIndex < nbToClear)
	{
		PxU32 indexToClear = edgeIndicesToClear[threadIndex];
		frictionPatchCount[indexToClear] = 0;
	}
}


//each block has 16 warps, each warp has 32 threads, 32 blocks
extern "C" __global__ 
__launch_bounds__(PxgKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1)
void rigidSumInternalContactAndJointBatches1(
	PxgPrePrepDesc* PX_RESTRICT prePrepDesc,
	const PxU32 nbBodies)
{

	const PxU32 numThreadsPerBlock = PxgKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;

	const PxU32* const PX_RESTRICT staticContactCount = prePrepDesc->mRigidStaticContactCounts;
	const PxU32* const PX_RESTRICT staticJointCount = prePrepDesc->mRigidStaticConstraintCounts;
	
	PxU32* tempStaticContactUniqueIndicesBlockSum = prePrepDesc->mTempContactUniqueIndices;
	PxU32* tempStaticJointUniqueIndicesBlockSum = prePrepDesc->mTempConstraintUniqueIndices;

	PxU32* tempStaticContactHeaderBlockSum = prePrepDesc->mTempContactBlockHeader;
	PxU32* tempStaticJointHeaderBlockSum = prePrepDesc->mTempConstraintBlockHeader;

	const PxU32 warpPerBlock = numThreadsPerBlock / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	const PxU32 block_size = 32;

	const PxU32 totalBlockRequired = (nbBodies + (numThreadsPerBlock - 1)) / numThreadsPerBlock;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	__shared__ PxU32 shContactUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shJointUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shContactHeaderWarpSum[warpPerBlock];
	__shared__ PxU32 shJointHeaderWarpSum[warpPerBlock];

	__shared__ PxU32 sContactUniqueIndicesAccum;
	__shared__ PxU32 sJointUniqueIndicesAccum;
	__shared__ PxU32 sContactHeaderAccum;
	__shared__ PxU32 sJointHeaderAccum;



	if (threadIdx.x == (WARP_SIZE - 1))
	{
		sContactUniqueIndicesAccum = 0;
		sJointUniqueIndicesAccum = 0;
		sContactHeaderAccum = 0;
		sJointHeaderAccum = 0;
	}

	__syncthreads();

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + threadIdx.x + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 contactCount = 0;
		PxU32 jointCount = 0;

		if (workIndex < nbBodies)
		{
			contactCount = staticContactCount[workIndex];
			jointCount = staticJointCount[workIndex];

			//printf("workIndex %i contactCount %i jointCount %i \n", workIndex, contactCount, jointCount);
		}


		PxU32 maxContact = contactCount;
		PxU32 maxJoint = jointCount;

		contactCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, contactCount);
		jointCount = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, jointCount);
		maxContact = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxContact);
		maxJoint = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxJoint);

		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			shContactUniqueIndicesWarpSum[warpIndex] = contactCount;
			shJointUniqueIndicesWarpSum[warpIndex] = jointCount;
			shContactHeaderWarpSum[warpIndex] = maxContact;
			shJointHeaderWarpSum[warpIndex] = maxJoint;

			/*if (blockIdx.x == 0)
			{
				printf("warpIndex %i sumContact %i, sumJoint %i maxContact %i maxJoint %i\n", warpIndex, sumContact, sumJoint, maxContact, maxJoint);
			}*/
		}


		__syncthreads();

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);

		if (threadIdx.x < warpPerBlock)
		{
			PxU32 contactUniqueIndicesWarpSum = shContactUniqueIndicesWarpSum[threadIndexInWarp];
			PxU32 jointUniqueIndicesWarpSum = shJointUniqueIndicesWarpSum[threadIndexInWarp];
			PxU32 contactHeaderWarpSum = shContactHeaderWarpSum[threadIndexInWarp];
			PxU32 jointHeaderWarpSum = shJointHeaderWarpSum[threadIndexInWarp];

			contactUniqueIndicesWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, contactUniqueIndicesWarpSum);
			jointUniqueIndicesWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, jointUniqueIndicesWarpSum);
			contactHeaderWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, contactHeaderWarpSum);
			jointHeaderWarpSum = warpReduction<AddOpPxU32, PxU32>(mask_idx, jointHeaderWarpSum);

			if (threadIdx.x == (warpPerBlock - 1))
			{
				sContactUniqueIndicesAccum += contactUniqueIndicesWarpSum;
				sJointUniqueIndicesAccum += jointUniqueIndicesWarpSum;
				sContactHeaderAccum += contactHeaderWarpSum;
				sJointHeaderAccum += jointHeaderWarpSum;

				/*			if(blockIdx.x == 0)
						printf("contactUniqueIndicesWarpSum %i jointUniqueIndicesWarpSum %i contactHeaderWarpSum %i sJointHeaderAccum %i\n", contactUniqueIndicesWarpSum,
							jointUniqueIndicesWarpSum, contactHeaderWarpSum, jointHeaderWarpSum);*/
			}
		}

		__syncthreads();

	}

	if (threadIdx.x == (warpPerBlock - 1))
	{
		tempStaticContactUniqueIndicesBlockSum[blockIdx.x] = sContactUniqueIndicesAccum;
		tempStaticJointUniqueIndicesBlockSum[blockIdx.x] = sJointUniqueIndicesAccum;
		tempStaticContactHeaderBlockSum[blockIdx.x] = sContactHeaderAccum;
		tempStaticJointHeaderBlockSum[blockIdx.x] = sJointHeaderAccum;
	}



}


extern "C" __global__ 
__launch_bounds__(PxgKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT, 1)
void rigidSumInternalContactAndJointBatches2(
	PxgPrePrepDesc* PX_RESTRICT prePrepDesc,
	PxgSolverCoreDesc* PX_RESTRICT solverCoreDesc,
	PxgConstraintPrepareDesc* constraintPrepDesc, 
	const PxU32 nbBodies)
{

	const PxU32 numThreadsPerBlock = PxgKernelBlockDim::COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT;

	const PxU32* const PX_RESTRICT staticContactCount = prePrepDesc->mRigidStaticContactCounts;
	const PxU32* const PX_RESTRICT staticJointCount = prePrepDesc->mRigidStaticConstraintCounts;

	const PxU32* const PX_RESTRICT contactStaticUniqueIds = prePrepDesc->mRigidStaticContactIndices;
	const PxU32* const PX_RESTRICT jointStaticUniqueIndices = prePrepDesc->mRigidStaticConstraintIndices;

	PxU32* PX_RESTRICT contactStaticStartIndices = prePrepDesc->mRigidStaticContactStartIndices;
	PxU32* PX_RESTRICT jointStaticStartIndices = prePrepDesc->mRigidStaticConstraintStartIndices;

	PxgConstraintBatchHeader* PX_RESTRICT batchHeaders = prePrepDesc->mBatchHeaders;

	PxU32* outContactUniqueIds = prePrepDesc->mContactUniqueIndices;
	PxU32* outJointUniqueIndices = prePrepDesc->mConstraintUniqueIndices;

	PxU32* tempStaticContactUniqueIndicesBlock = prePrepDesc->mTempContactUniqueIndices;
	PxU32* tempStaticJointUniqueIndicesBlock = prePrepDesc->mTempConstraintUniqueIndices;

	PxU32* tempStaticContactHeaderBlock = prePrepDesc->mTempContactBlockHeader;
	PxU32* tempStaticJointHeaderBlock = prePrepDesc->mTempConstraintBlockHeader;

	PxU32& outNumStaticBatches = prePrepDesc->numStaticBatches;

	//const PxU32 contactUniqueIndexOffset = 


	const PxU32 warpPerBlock = numThreadsPerBlock / WARP_SIZE;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	const PxU32 block_size = 32; // 32 blocks

	__shared__ PxU32 shContactUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shJointUniqueIndicesWarpSum[warpPerBlock];
	__shared__ PxU32 shContactHeaderWarpSum[warpPerBlock];
	__shared__ PxU32 shJointHeaderWarpSum[warpPerBlock];

	__shared__ PxU32 sContactUniqueIndicesBlockHistogram[block_size];
	__shared__ PxU32 sJointUniqueIndicesBlockHistogram[block_size];
	__shared__ PxU32 sContactHeaderBlockHistogram[block_size];
	__shared__ PxU32 sJointHeaderBlockHistogram[block_size];

	__shared__ PxU32 sContactUniqueIndicesAccum;
	__shared__ PxU32 sJointUniqueIndicesAccum;
	__shared__ PxU32 sContactHeaderAccum;
	__shared__ PxU32 sJointHeaderAccum;

	PxU32* jointConstraintBatchIndices = constraintPrepDesc->jointConstraintBatchIndices;
	PxU32* contactConstraintBatchIndices = constraintPrepDesc->contactConstraintBatchIndices;

	const PxU32 numRigidContacts = prePrepDesc->numTotalContacts;
	const PxU32 numRigidJoints = prePrepDesc->numTotalConstraints;

	/*const PxU32 numArtiContacts = prePrepDesc->numTotalArtiContacts + prePrepDesc->numTotalStaticArtiContacts + prePrepDesc->numTotalSelfArtiContacts;
	const PxU32 numArtiJoints = prePrepDesc->numTotalConstraints + prePrepDesc->numTotalStaticArtiConstraints + prePrepDesc->numTotalSelfArtiConstraints;*/

	const PxU32 numContactBatches = constraintPrepDesc->numContactBatches;
	const PxU32 numConstraintBatches = constraintPrepDesc->num1dConstraintBatches;

	const PxU32 numArtiBatches = prePrepDesc->numArtiBatches + prePrepDesc->numArtiSelfBatches + prePrepDesc->numArtiStaticBatches;


	const PxU32 batchOffset = prePrepDesc->numBatches + numArtiBatches;
	const PxU32 contactUniqueIndexOffset = numRigidContacts;// +prePrepDesc->numTotalArtiContacts + prePrepDesc->numTotalStaticArtiContacts + prePrepDesc->numTotalSelfArtiContacts;
	const PxU32 jointUniqueIndexOffset = numRigidJoints;// +prePrepDesc->numTotalArtiConstraints + prePrepDesc->numTotalStaticArtiConstraints + prePrepDesc->numTotalSelfArtiConstraints;

	const PxU32 staticContactBatchOffset = prePrepDesc->artiStaticContactBatchOffset + constraintPrepDesc->numArtiStaticContactBatches + constraintPrepDesc->numArtiSelfContactBatches;
	const PxU32 staticJointBatchOffset = prePrepDesc->artiStaticConstraintBatchOffset + constraintPrepDesc->numArtiStatic1dConstraintBatches + constraintPrepDesc->numArtiSelf1dConstraintBatches;


	if (threadIdx.x == (WARP_SIZE - 1))
	{
		sContactUniqueIndicesAccum = 0;
		sJointUniqueIndicesAccum = 0;
		sContactHeaderAccum = 0;
		sJointHeaderAccum = 0;
	}


	//accumulate num pairs per block and compute exclusive run sum
	//unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < block_size);
	if (warpIndex == 0/* && threadIndexInWarp < block_size*/)
	{
		const PxU32 oriContactUniqueIndiceOffset = tempStaticContactUniqueIndicesBlock[threadIndexInWarp];
		const PxU32 oriJointUniqueIndiceOffset = tempStaticJointUniqueIndicesBlock[threadIndexInWarp];
		const PxU32 oriContactHeaderOffset = tempStaticContactHeaderBlock[threadIndexInWarp];
		const PxU32 oriJointHeaderOffset = tempStaticJointHeaderBlock[threadIndexInWarp];

		const PxU32 contactUniqueIndiceOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriContactUniqueIndiceOffset);
		const PxU32 jointUniqueIndiceOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriJointUniqueIndiceOffset);
		const PxU32 contactHeaderOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriContactHeaderOffset);
		const PxU32 jointHeaderOffset = warpScan<AddOpPxU32, PxU32>(FULL_MASK, oriJointHeaderOffset);
		//store exclusive run sum
		sContactUniqueIndicesBlockHistogram[threadIndexInWarp] = contactUniqueIndiceOffset - oriContactUniqueIndiceOffset;
		sJointUniqueIndicesBlockHistogram[threadIndexInWarp] = jointUniqueIndiceOffset - oriJointUniqueIndiceOffset;
		sContactHeaderBlockHistogram[threadIndexInWarp] = contactHeaderOffset - oriContactHeaderOffset;
		sJointHeaderBlockHistogram[threadIndexInWarp] = jointHeaderOffset - oriJointHeaderOffset;

		/*if (blockIdx.x == 0)
		{
			printf("sContactHeaderBlockHistogram[%i] = %i + %i\n", threadIndexInWarp, contactHeaderOffset, oriContactHeaderOffset);
		}*/

		if (blockIdx.x == 0 && threadIdx.x == (WARP_SIZE - 1))
		{
			//Output total number of articulation static blocks
			const PxU32 totalNumStaticBatches = contactHeaderOffset + jointHeaderOffset;
			outNumStaticBatches = totalNumStaticBatches;

			constraintPrepDesc->numStaticContactBatches = contactHeaderOffset;
			constraintPrepDesc->numStatic1dConstraintBatches = jointHeaderOffset;
			constraintPrepDesc->numStaticBatches = contactHeaderOffset + jointHeaderOffset;

			PxgIslandContext& island = solverCoreDesc->islandContextPool[0];
			island.mStaticRigidBatchCount = totalNumStaticBatches;
			/*printf("StaticContactCount = %i, dynamicContactCount = %i, staticJointCount = %i, %p\n", contactHeaderOffset, constraintPrepDesc->numContactBatches, jointHeaderOffset,
				constraintPrepDesc);*/

		}

	}

	__syncthreads();

	//We now have the exclusive runsum for this block. Next step is to recompute the local
	//offsets within the block and output data...

	const PxU32 totalBlockRequired = (nbBodies + (blockDim.x - 1)) / blockDim.x;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (block_size - 1)) / block_size;

	for (PxU32 a = 0; a < numIterationPerBlock; ++a)
	{
		const PxU32 workIndex = a * blockDim.x + threadIdx.x + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU32 contactCount = 0;
		PxU32 jointCount = 0;

		if (workIndex < nbBodies)
		{
			contactCount = staticContactCount[workIndex];
			jointCount = staticJointCount[workIndex];
		}





		//we need to use contactCount and jointCount later
		PxU32 sumContact = contactCount;
		PxU32 sumJoint = jointCount;
		PxU32 maxContact = contactCount;
		PxU32 maxJoint = jointCount;

		sumContact = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sumContact);
		sumJoint = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, sumJoint);
		maxContact = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxContact);
		maxJoint = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, maxJoint);


		if (threadIndexInWarp == 31)
		{
			shContactUniqueIndicesWarpSum[warpIndex] = sumContact;
			shJointUniqueIndicesWarpSum[warpIndex] = sumJoint;
			shContactHeaderWarpSum[warpIndex] = maxContact;
			shJointHeaderWarpSum[warpIndex] = maxJoint;
		}

		__syncthreads();

		PxU32 contactWarpOffset = 0;
		PxU32 jointWarpOffset = 0;
		PxU32 contactBlockWarpOffset = 0;
		PxU32 jointBlockWarpOffset = 0;

		PxU32 contactUniqueIndicesWarpSum = 0;
		PxU32 jointUniqueIndicesWarpSum = 0;
		PxU32 contactHeaderWarpSum = 0;
		PxU32 jointHeaderWarpSum = 0;

		unsigned mask_idx = __ballot_sync(FULL_MASK, threadIndexInWarp < warpPerBlock);

		//warpPerBlock should be less than 32, each warp will do the runsum
		if (threadIndexInWarp < warpPerBlock)
		{

			const PxU32 oriContactUniqueIndicesWarpSum = shContactUniqueIndicesWarpSum[threadIndexInWarp];
			const PxU32 oriJointUniqueIndicesWarpSum = shJointUniqueIndicesWarpSum[threadIndexInWarp];
			const PxU32 oriContactHeaderWarpSum = shContactHeaderWarpSum[threadIndexInWarp];
			const PxU32 oriJointHeaderWarpSum = shJointHeaderWarpSum[threadIndexInWarp];

			contactUniqueIndicesWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriContactUniqueIndicesWarpSum);
			jointUniqueIndicesWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriJointUniqueIndicesWarpSum);
			contactHeaderWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriContactHeaderWarpSum);
			jointHeaderWarpSum = warpScan<AddOpPxU32, PxU32>(mask_idx, oriJointHeaderWarpSum);


			//exclusive runsum
			contactWarpOffset = contactUniqueIndicesWarpSum - oriContactUniqueIndicesWarpSum;
			jointWarpOffset = jointUniqueIndicesWarpSum - oriJointUniqueIndicesWarpSum;
			contactBlockWarpOffset = contactHeaderWarpSum - oriContactHeaderWarpSum;
			jointBlockWarpOffset = jointHeaderWarpSum - oriJointHeaderWarpSum;


		}

		//make sure each thread in a warp has the correct warp offset
		contactWarpOffset = __shfl_sync(FULL_MASK, contactWarpOffset, warpIndex);
		jointWarpOffset = __shfl_sync(FULL_MASK, jointWarpOffset, warpIndex);
		contactBlockWarpOffset = __shfl_sync(FULL_MASK, contactBlockWarpOffset, warpIndex);
		jointBlockWarpOffset = __shfl_sync(FULL_MASK, jointBlockWarpOffset, warpIndex);


		/*if (workIndex == 512)
		{
			printf("sContactHeaderBlockHistogram[%i] = %i, sContactUniqueIndicesBlockHistogram[%i] = %i, contatBlockWarpOffset = %i, sContactUniqueIndicesAccum = %i\n",
				blockIdx.x, sContactHeaderBlockHistogram[blockIdx.x], blockIdx.x, sContactUniqueIndicesBlockHistogram[blockIdx.x],
				contactBlockWarpOffset, sContactUniqueIndicesAccum);
		}*/

		//OK. We finally have enough information to figure out where to write the blocks related to this
		//articulation!

		//Where the contact uniqueIds should go. This is the start of where this warp should write.
		//The uids will be interleaved depending on the number of constraints in a contact block
		//contactUniqueIndexOffset : articulation static contact start offset
		//sContactUniqueIndicesAccum :: accumulation from the previous iterations
		PxU32 contactOffset = contactUniqueIndexOffset + contactWarpOffset + sContactUniqueIndicesBlockHistogram[blockIdx.x] + sContactUniqueIndicesAccum;
		//Where the joint unique Ids should go. See above for explanation
		PxU32 jointOffset = jointUniqueIndexOffset + jointWarpOffset + sJointUniqueIndicesBlockHistogram[blockIdx.x] + sJointUniqueIndicesAccum;
		//Where the blocks should go. Shared between all threads in a block
		PxU32 contactBlockOffset = contactBlockWarpOffset + sContactHeaderBlockHistogram[blockIdx.x] + sContactHeaderAccum;
		PxU32 jointBlockOffset = jointBlockWarpOffset + sJointHeaderBlockHistogram[blockIdx.x] + sJointHeaderAccum;

		PxU32 blockOffset = contactBlockOffset + jointBlockOffset + batchOffset;


		//we have to sync in here so all the threads in a warp has finished reading sContactUniqueIndicesAccum, sJointUniqueIndicesAccum,
		//sContactHeaderAccum, sJointHeaderAccum before we overwrite those values for another iterations
		__syncthreads();

		if (threadIdx.x == (warpPerBlock - 1))
		{
			sContactUniqueIndicesAccum += contactUniqueIndicesWarpSum;
			sJointUniqueIndicesAccum += jointUniqueIndicesWarpSum;
			sContactHeaderAccum += contactHeaderWarpSum;
			sJointHeaderAccum += jointHeaderWarpSum;
		}

		__syncthreads();

		if (workIndex < nbBodies)
		{
			contactStaticStartIndices[workIndex] = blockOffset;
			jointStaticStartIndices[workIndex] = blockOffset + maxContact;
		}


		for (PxU32 i = 0; i < maxContact; ++i)
		{
			PxU32 mask = __ballot_sync(FULL_MASK, contactCount > i);

			const PxU32 stride = __popc(mask);
			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (contactCount > i)
			{
				PxU32 contactUniqueId = contactStaticUniqueIds[workIndex + nbBodies * i];
				outContactUniqueIds[contactOffset + offset] = contactUniqueId;
				//printf("outContactUniqueIds[%i] = contactStaticUniqueIds[%i] = %i\n", contactOffset + offset, workIndex + nbBodies * i, contactUniqueId);
				//printf("outContactUniqueIds[%i] = %i, ptr = %p\n", contactOffset + offset, contactUniqueId, &outContactUniqueIds[contactOffset + offset]);
				//printf("globalThreadIndex %i contactUniqueId %i contactOffset %i offset %i blockOffset %i\n", globalThreadIndex, contactUniqueId, contactOffset, offset, blockOffset);
			}

			if (threadIndexInWarp == 0)
			{
				const PxU32 batchIndex = contactBlockOffset + i + staticContactBatchOffset;
				PxgConstraintBatchHeader header;
				header.mDescStride = stride;
				header.constraintType = PxgSolverConstraintDesc::eCONTACT;
				header.mConstraintBatchIndex = batchIndex;
				header.mStartPartitionIndex = contactOffset/* - numArtiContacts*/;
				header.mask = mask;
				batchHeaders[blockOffset] = header;
				contactConstraintBatchIndices[contactBlockOffset + numContactBatches + i] = blockOffset;// -numArtiBatches;

				//printf("batchIndex %i blockOffset %i contactOffset %i mStartPartitionIndex %i numRigidContacts %i contactConstraintBatchIndices[%i] = %i\n", 
				//	batchIndex, blockOffset,
				//	contactOffset, header.mStartPartitionIndex, numRigidContacts,
				//	contactBlockOffset + numContactBatches + i, 
				//	contactConstraintBatchIndices[contactBlockOffset + numContactBatches + i]);

				//printf("contactConstraintBatchIndices[%i] = %i\n", contactBlockOffset + numContactBatches + i, blockOffset/* - numArtiBatches*/);

					//printf("STATIC* batchHeaders[%i] output, artiContactConstraintBatchIndex[%i] = %i\n", blockOffset, contactBlockOffset + numArtiContactBatches + i, blockOffset - numRigidBatches);

			}

			contactOffset += stride;
			blockOffset++;
		}

		for (PxU32 i = 0; i < maxJoint; ++i)
		{
			PxU32 mask = __ballot_sync(FULL_MASK, jointCount > i);

			const PxU32 stride = __popc(mask);
			const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);

			if (jointCount > i)
			{
				PxU32 jointUniqueId = jointStaticUniqueIndices[workIndex + nbBodies * i];
				outJointUniqueIndices[jointOffset + offset] = jointUniqueId;

				//printf("outJointUniqueIndices[%i] = %i, ptr = %p\n", jointOffset + offset, jointUniqueId, &outJointUniqueIndices[jointOffset + offset]);
			}

			if (threadIndexInWarp == 0)
			{
				PxgConstraintBatchHeader header;
				header.mDescStride = stride;
				const PxU32 batchIndex = jointBlockOffset + i + staticJointBatchOffset;
				header.constraintType = PxgSolverConstraintDesc::eCONSTRAINT_1D;
				header.mConstraintBatchIndex = batchIndex;
				header.mStartPartitionIndex = jointOffset/* - numArtiJoints*/;
				header.mask = mask;
				batchHeaders[blockOffset] = header;
				jointConstraintBatchIndices[jointBlockOffset + i + numConstraintBatches] = blockOffset/* - numArtiBatches*/;

				//if(jointBlockOffset > 0)
				//	printf("Joint batchIndex[%i] = %i, startPartitionIndex = %i\n", jointBlockOffset + numArtiJointBatches + i, blockOffset - numRigidBatches, jointOffset - numRigidJoints);
			}

			jointOffset += stride;
			blockOffset++;
		}
	}
}

