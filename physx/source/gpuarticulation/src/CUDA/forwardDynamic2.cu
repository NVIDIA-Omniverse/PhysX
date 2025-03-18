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

#include "CmSpatialVector.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgArticulationLink.h"
#include "DyArticulationJointCore.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "PxgBodySim.h" 
#include "cutil_math.h"
#include "reduction.cuh"
#include "copy.cuh"
#include "articulationDynamic.cuh"
#include "articulationImpulseResponse.cuh"
#include "PxSpatialMatrix.h"
#include "foundation/PxMath.h"
#include "DyFeatherstoneArticulation.h"
#include "PxgSolverCoreDesc.h"
#include "PxsRigidBody.h"
#include "PxArticulationTendonData.h"
#include "foundation/PxMathUtils.h"
#include "utils.cuh"
#include "DyArticulationMimicJointCore.h"

using namespace physx;
using namespace Dy;

extern "C" __host__ void initArticulationKernels1() {}

template<class T>
__device__ static PX_FORCE_INLINE void translateSpatialVectorInPlace(const PxVec3& offset, T& vec)
{
	vec.bottom += offset.cross(vec.top);
}

static __device__ void initialize(
	PxgArticulationBlockData& PX_RESTRICT blockData,
	PxgArticulationBlockLinkData* PX_RESTRICT linkData,
	PxgArticulationBlockDofData* PX_RESTRICT dofData,
	const PxU32 numLinks, const PxU32 totalDofs, 
	const PxU32 threadIndexInWarp)
{
	const Cm::UnAlignedSpatialVector zero = Cm::UnAlignedSpatialVector::Zero();

	//KS - TODO - do we actually need deltaMotionVelocity? We will be computing motionVelocity every iteration
	//regardless...
	//initialize all delta motion velocites to be zero
	for (PxU32 i = 0; i < numLinks; ++i)
	{
		storeSpatialVector(linkData[i].mDeltaMotion, zero, threadIndexInWarp);
		storeSpatialVector(linkData[i].mPosMotionVelocity, zero, threadIndexInWarp);
		linkData[i].mDeltaQ[threadIndexInWarp] = make_float4(0.f, 0.f, 0.f, 1.f);
		storeSpatialVector(linkData[i].mScratchImpulse, zero, threadIndexInWarp);
		storeSpatialVector(linkData[i].mSolverSpatialDeltaVel, zero, threadIndexInWarp);
		storeSpatialVector(linkData[i].mSolverSpatialImpulse, zero, threadIndexInWarp);
		storeSpatialVector(linkData[i].mSolverSpatialInternalConstraintImpulse, zero, threadIndexInWarp);
		storeSpatialVector(linkData[i].mConstraintForces, zero, threadIndexInWarp);
		linkData[i].mStaticContactStartIndex[threadIndexInWarp] = 0;
		linkData[i].mNbStaticContacts[threadIndexInWarp] = 0;
		linkData[i].mStaticJointStartIndex[threadIndexInWarp] = 0;
		linkData[i].mNbStaticJoints[threadIndexInWarp] = 0;
		linkData[i].mDeltaScale[threadIndexInWarp] = 0.f;
	}

	storeSpatialVector(blockData.mRootDeferredZ, zero, threadIndexInWarp);
	storeSpatialVector(blockData.mCommonLinkDeltaVelocity, zero, threadIndexInWarp);
	blockData.mLinkWithDeferredImpulse[threadIndexInWarp] = 0;

	for (PxU32 i = 0; i < totalDofs; ++i)
	{
		dofData[i].mDeferredQstZ[threadIndexInWarp] = 0.f;
		dofData[i].mInvStIsT_x[threadIndexInWarp] = 0.f;
		dofData[i].mInvStIsT_y[threadIndexInWarp] = 0.f;
		dofData[i].mInvStIsT_z[threadIndexInWarp] = 0.f;
	}

}


static __device__ void initializeSpatialTendonsBlock(
	const PxgArticulation& articulation,
	PxgArticulationData& artiData,
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT  spatialTendonBlocks,
	PxgArticulationBlockAttachmentData* PX_RESTRICT attachmentBlocks,
	const PxU32 maxNumAttachments,
	const PxU32 threadIndexInWarp)
{
	//spatial tendon block data initialization
	PxGpuSpatialTendonData* spatialTendonParams = articulation.spatialTendonParams;
	PxgArticulationTendon* spatialTendons = articulation.spatialTendons;
	const PxU32 numSpatialTendons = artiData.numSpatialTendons;
	for (PxU32 i = 0; i < numSpatialTendons; ++i)
	{
		PxgArticulationTendon tendon = spatialTendons[i];
		PxGpuSpatialTendonData& tendonParams = spatialTendonParams[i];

		PxgArticulationBlockSpatialTendonData& tendonBlockData = spatialTendonBlocks[i];

		tendonBlockData.mNumAttachments[threadIndexInWarp] = tendon.mNbElements;
		tendonBlockData.mStiffness[threadIndexInWarp] = tendonParams.stiffness;
		tendonBlockData.mLimitStiffness[threadIndexInWarp] = tendonParams.limitStiffness;
		tendonBlockData.mDamping[threadIndexInWarp] = tendonParams.damping;
		tendonBlockData.mOffset[threadIndexInWarp] = tendonParams.offset;


		PxgArticulationBlockAttachmentData* attachmentBlock = &attachmentBlocks[i * maxNumAttachments];

		PxgArticulationTendonElementFixedData* fixedData = reinterpret_cast<PxgArticulationTendonElementFixedData*>(tendon.mFixedElements);
		PxGpuTendonAttachmentData* modData = reinterpret_cast<PxGpuTendonAttachmentData*>(tendon.mModElements);

		for (PxU32 j = 0; j < tendon.mNbElements; ++j)
		{
			PxgArticulationTendonElementFixedData& fData = fixedData[j];
			PxGpuTendonAttachmentData& mData = modData[j];

			attachmentBlock[j].mParents[threadIndexInWarp] = fData.parent;
			attachmentBlock[j].mChildrens[threadIndexInWarp] = fData.children;
			attachmentBlock[j].mLinkIndex[threadIndexInWarp] = fData.linkInd;

			attachmentBlock[j].mRelativeOffset[threadIndexInWarp] = mData.relativeOffset;
			attachmentBlock[j].mRestDistance[threadIndexInWarp] = mData.restLength;
			attachmentBlock[j].mCoefficient[threadIndexInWarp] = mData.coefficient;
			attachmentBlock[j].mLowLimit[threadIndexInWarp] = mData.lowLimit;
			attachmentBlock[j].mHighLimit[threadIndexInWarp] = mData.highLimit;
		}
	}

	artiBlock.mNumSpatialTendons[threadIndexInWarp] = numSpatialTendons;
}

static __device__ void updateSpatialTendonsBlock(
	const PxgArticulation& articulation,
	PxgArticulationData& artiData,
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT  spatialTendonBlocks,
	PxgArticulationBlockAttachmentData* PX_RESTRICT attachmentBlocks,
	const PxU32 maxNumAttachments,
	const PxU32 threadIndexInWarp)
{
	//spatial tendon block data initialization
	PxGpuSpatialTendonData* spatialTendonParams = articulation.spatialTendonParams;
	PxgArticulationTendon* spatialTendons = articulation.spatialTendons;
	const PxU32 numSpatialTendons = artiData.numSpatialTendons;
	for (PxU32 i = 0; i < numSpatialTendons; ++i)
	{
		PxgArticulationTendon tendon = spatialTendons[i];
		PxGpuSpatialTendonData& tendonParams = spatialTendonParams[i];

		PxgArticulationBlockSpatialTendonData& tendonBlockData = spatialTendonBlocks[i];

		tendonBlockData.mStiffness[threadIndexInWarp] = tendonParams.stiffness;
		tendonBlockData.mLimitStiffness[threadIndexInWarp] = tendonParams.limitStiffness;
		tendonBlockData.mDamping[threadIndexInWarp] = tendonParams.damping;
		tendonBlockData.mOffset[threadIndexInWarp] = tendonParams.offset;

		PxgArticulationBlockAttachmentData* attachmentBlock = &attachmentBlocks[i * maxNumAttachments];
		PxGpuTendonAttachmentData* modData = reinterpret_cast<PxGpuTendonAttachmentData*>(tendon.mModElements);

		for (PxU32 j = 0; j < tendon.mNbElements; ++j)
		{
			
			PxGpuTendonAttachmentData& mData = modData[j];

			attachmentBlock[j].mRelativeOffset[threadIndexInWarp] = mData.relativeOffset;
			attachmentBlock[j].mRestDistance[threadIndexInWarp] = mData.restLength;
			attachmentBlock[j].mCoefficient[threadIndexInWarp] = mData.coefficient;
			attachmentBlock[j].mLowLimit[threadIndexInWarp] = mData.lowLimit;
			attachmentBlock[j].mHighLimit[threadIndexInWarp] = mData.highLimit;
		}
	}
}

static __device__ void initializeFixedTendonsBlock(
	const PxgArticulation& articulation, 
	PxgArticulationData& artiData,
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockFixedTendonData* PX_RESTRICT  fixedTendonBlocks,
	PxgArticulationBlockTendonJointData* PX_RESTRICT tendonJointBlocks,
	const PxU32 maxNumTendonJoints,
	const PxU32 threadIndexInWarp)
{
	//Fixed tendon block data initialization
	PxGpuFixedTendonData* fixedTendonParams = articulation.fixedTendonParams;
	PxgArticulationTendon* fixedTendons = articulation.fixedTendons;
	const PxU32 numFixedTendons = artiData.numFixedTendons;
	for (PxU32 i = 0; i < numFixedTendons; ++i)
	{
		PxgArticulationTendon tendon = fixedTendons[i];
		PxGpuFixedTendonData& tendonParams = fixedTendonParams[i];

		PxgArticulationBlockFixedTendonData& tendonBlockData = fixedTendonBlocks[i];

		tendonBlockData.mNumTendonJoints[threadIndexInWarp] = tendon.mNbElements;
		tendonBlockData.mStiffness[threadIndexInWarp] = tendonParams.stiffness;
		tendonBlockData.mDamping[threadIndexInWarp] = tendonParams.damping;
		tendonBlockData.mLimitStiffness[threadIndexInWarp] = tendonParams.limitStiffness;
		tendonBlockData.mOffset[threadIndexInWarp] = tendonParams.offset;
		tendonBlockData.mLowLimit[threadIndexInWarp] = tendonParams.lowLimit;
		tendonBlockData.mHighLimit[threadIndexInWarp] = tendonParams.highLimit;
		tendonBlockData.mRestLength[threadIndexInWarp] = tendonParams.restLength;

		PxgArticulationBlockTendonJointData* tendonJointBlock = &tendonJointBlocks[i * maxNumTendonJoints];
		PxgArticulationTendonElementFixedData* fixedData = reinterpret_cast<PxgArticulationTendonElementFixedData*>(tendon.mFixedElements);
		PxGpuTendonJointCoefficientData* coefficientData = reinterpret_cast<PxGpuTendonJointCoefficientData*>(tendon.mModElements);

		for (PxU32 j = 0; j < tendon.mNbElements; ++j)
		{
			PxgArticulationTendonElementFixedData& fData = fixedData[j];
			PxGpuTendonJointCoefficientData& cData = coefficientData[j];

			tendonJointBlock[j].mParents[threadIndexInWarp] = fData.parent;
			tendonJointBlock[j].mChildrens[threadIndexInWarp] = fData.children;
			tendonJointBlock[j].mLinkIndex[threadIndexInWarp] = fData.linkInd;
			tendonJointBlock[j].mCoefficient[threadIndexInWarp] = cData.coefficient;
			tendonJointBlock[j].mRecipCoefficient[threadIndexInWarp] = cData.recipCoefficient;

			tendonJointBlock[j].mAxis[threadIndexInWarp] = cData.axis;
		}
	}

	artiBlock.mNumFixedTendons[threadIndexInWarp] = numFixedTendons;

}

static __device__ void updateFixedTendonsBlock(
	const PxgArticulation& articulation,
	PxgArticulationData& artiData,
	PxgArticulationBlockFixedTendonData* PX_RESTRICT  fixedTendonBlocks,
	PxgArticulationBlockTendonJointData* PX_RESTRICT tendonJointBlocks,
	const PxU32 maxNumTendonJoints,
	const PxU32 threadIndexInWarp)
{
	//Fixed tendon block data initialization
	PxGpuFixedTendonData* fixedTendonParams = articulation.fixedTendonParams;
	PxgArticulationTendon* fixedTendons = articulation.fixedTendons;
	const PxU32 numFixedTendons = artiData.numFixedTendons;
	for (PxU32 i = 0; i < numFixedTendons; ++i)
	{
		PxgArticulationTendon tendon = fixedTendons[i];
		PxGpuFixedTendonData& tendonParams = fixedTendonParams[i];

		PxgArticulationBlockFixedTendonData& tendonBlockData = fixedTendonBlocks[i];

		tendonBlockData.mStiffness[threadIndexInWarp] = tendonParams.stiffness;
		tendonBlockData.mDamping[threadIndexInWarp] = tendonParams.damping;
		tendonBlockData.mLimitStiffness[threadIndexInWarp] = tendonParams.limitStiffness;
		tendonBlockData.mOffset[threadIndexInWarp] = tendonParams.offset;
		tendonBlockData.mLowLimit[threadIndexInWarp] = tendonParams.lowLimit;
		tendonBlockData.mHighLimit[threadIndexInWarp] = tendonParams.highLimit;
		tendonBlockData.mRestLength[threadIndexInWarp] = tendonParams.restLength;

		PxgArticulationBlockTendonJointData* tendonJointBlock = &tendonJointBlocks[i * maxNumTendonJoints];

		PxGpuTendonJointCoefficientData* coefficientData = reinterpret_cast<PxGpuTendonJointCoefficientData*>(tendon.mModElements);

		for (PxU32 j = 0; j < tendon.mNbElements; ++j)
		{
		
			PxGpuTendonJointCoefficientData& cData = coefficientData[j];

			tendonJointBlock[j].mCoefficient[threadIndexInWarp] = cData.coefficient;
			tendonJointBlock[j].mRecipCoefficient[threadIndexInWarp] = cData.recipCoefficient;
			tendonJointBlock[j].mAxis[threadIndexInWarp] = cData.axis;
		}
	}
}

static __device__ void copyMimicJointsBlock(
	const PxgArticulation& articulation,
	PxgArticulationData& artiData, 
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockMimicJointData* PX_RESTRICT mimicJointBlocks,
	const PxU32 threadIndexInWarp)
{
	for (PxU32 i = 0; i < artiData.numMimicJoints; ++i)
	{
		const Dy::ArticulationMimicJointCore& mimicJointCore = articulation.mimicJointCores[i];
		PxgArticulationBlockMimicJointData& mimicJointBlock = mimicJointBlocks[i];
		mimicJointBlock.mLinkA[threadIndexInWarp] = mimicJointCore.linkA;
		mimicJointBlock.mLinkB[threadIndexInWarp] = mimicJointCore.linkB;
		mimicJointBlock.mAxisA[threadIndexInWarp] = mimicJointCore.axisA;
		mimicJointBlock.mAxisB[threadIndexInWarp] = mimicJointCore.axisB;
		mimicJointBlock.mGearRatio[threadIndexInWarp] = mimicJointCore.gearRatio;
		mimicJointBlock.mOffset[threadIndexInWarp] = mimicJointCore.offset;
		mimicJointBlock.mNaturalFrequency[threadIndexInWarp] = mimicJointCore.naturalFrequency;
		mimicJointBlock.mDampingRatio[threadIndexInWarp] = mimicJointCore.dampingRatio;
	}
}

static __device__ void initializeMimicJointsBlock(
	const PxgArticulation& articulation,
	PxgArticulationData& artiData, 
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockMimicJointData* PX_RESTRICT mimicJointBlocks,
	const PxU32 threadIndexInWarp)
{
	copyMimicJointsBlock(articulation, artiData, artiBlock, mimicJointBlocks, threadIndexInWarp);
	artiBlock.mNumMimicJoints[threadIndexInWarp] = artiData.numMimicJoints;
}

static __device__ void updateMimicJointsBlock(
	const PxgArticulation& articulation,
	PxgArticulationData& artiData,
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockMimicJointData* PX_RESTRICT mimicJointBlocks,
	const PxU32 threadIndexInWarp)
{
	copyMimicJointsBlock(articulation, artiData, artiBlock, mimicJointBlocks, threadIndexInWarp);
}

static __device__ PX_FORCE_INLINE void computeJointAxis(PxU32 dof, const ArticulationJointCore* PX_RESTRICT joint, Cm::UnAlignedSpatialVector* PX_RESTRICT jointAxis)
{
	assert(dof<=3);
	for (PxU32 i = 0; i < 3; ++i)
	{
		if(i<dof)
		{
			const PxU32 ind = joint->dofIds[i];
			jointAxis[i] = Cm::UnAlignedSpatialVector::Zero();
			jointAxis[i][ind] = 1.0f;	//axis is in the local space of joint
		}
	}
}

static __device__ void jcalc(const PxgArticulation& articulation, PxgArticulationData& artiData,
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	const bool refillBlockData,
	PxgArticulationBlockData& artiBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT articulationLinkBlocks,
	PxgArticulationBlockDofData* PX_RESTRICT articulationDofBlocks,
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT spatialTendonBlocks,
	PxgArticulationBlockAttachmentData* PX_RESTRICT attachmentBlocks,
	PxgArticulationBlockFixedTendonData* PX_RESTRICT fixedTendonBlocks,
	PxgArticulationBlockTendonJointData* PX_RESTRICT tendonJointBlocks,
	PxgArticulationBlockMimicJointData* PX_RESTRICT mimicJointBlocks,
	PxgArticulationBitFieldStackData* PX_RESTRICT linkBitFieldsBlocks,
	const PxU32 numLinks,
	const PxU32 maxNumLinks,
	const PxU32 maxNumAttachments,
	const PxU32 maxNumTendonJoints,
	const PxU32 threadIndexInWarp,  // this is used for correct indexing into the block data, where we get the per-warp block passed in
	const PxReal invLengthScale,
	const bool directAPI)
{
	using namespace Dy;

	const bool confiDirty = artiData.confiDirty;
	const bool dataDirty = (artiData.updateDirty != 0);

	ArticulationJointCore* gJoints = articulation.joints;
	ArticulationJointCoreData* gJointData = articulation.jointData;

	if (confiDirty || dataDirty)
	{
		Cm::UnAlignedSpatialVector* PX_RESTRICT jointAxis = articulation.jointAxis;
		Dy::SpatialSubspaceMatrix* PX_RESTRICT motionMatrix = articulation.motionMatrix;
		PxU32* PX_RESTRICT jointOffsets = articulation.jointOffsets;
		PxQuat* PX_RESTRICT relativeQuats = articulation.relativeQuat;
		PxReal* PX_RESTRICT cfmScale = articulation.cfmScale;
		const PxgArticulationLink* PX_RESTRICT links = articulation.links;
		PxReal* PX_RESTRICT cfms = articulation.cfms;
		const PxgArticulationLinkProp* PX_RESTRICT props = articulation.linkProps;

		if (confiDirty)
			cfmScale[0] = (artiData.flags & PxArticulationFlag::eFIX_BASE) ? 0.f : links[0].cfmScale * invLengthScale;

		if (dataDirty)
			cfms[0] = (artiData.flags & PxArticulationFlag::eFIX_BASE) ? 0.f : props[0].invInertiaXYZ_invMass.w * cfmScale[0];

		// PT: preload next link data
		PxReal nextDof = gJointData[1].nbDof;
		PxReal nextOffset = gJointData[1].jointOffset;

		PxU32 totalDofs = 0;
		for (PxU32 linkID = 1; linkID < numLinks; linkID++)
		{
			ArticulationJointCore& joint = gJoints[linkID];

			// PT: preload next link data
			const PxReal dof = nextDof;
			const PxReal offset = nextOffset;
			if(linkID!=numLinks-1)
			{
				nextDof = gJointData[linkID+1].nbDof;
				nextOffset = gJointData[linkID+1].jointOffset;
			}

			computeJointAxis(dof, &joint, jointAxis + totalDofs);

			joint.setJointFrame(motionMatrix[linkID].columns, jointAxis + totalDofs, relativeQuats[linkID], dof);

			jointOffsets[linkID] = offset;

			if (confiDirty)
				cfmScale[linkID] = links[linkID].cfmScale * invLengthScale;

			if (dataDirty)
				cfms[linkID] = props[linkID].invInertiaXYZ_invMass.w * cfmScale[linkID];

			totalDofs += dof;
		}

		artiData.confiDirty = false;
		// do not reset artiData.updateDirty here; hold on until block data is copied below in the else if (dataDirty) block
	}
	//KS - now we copy the data we may/may not have computed above into the batch format.
	//This has to be done every frame because there are no guarantees that articulations will be batched 
	//consistently due to sleeping.
	
	if (confiDirty || refillBlockData) //TODO - need to compare cached index to new index to skip this stage...
	{
		PxgArticulationLink* links = articulation.links;
		PxTransform* body2Worlds = articulation.linkBody2Worlds;
		PxU32* parents = articulation.parents;

		Cm::UnAlignedSpatialVector* jointAxis = articulation.jointAxis;


		const PxReal* PX_RESTRICT jointPositions = articulation.jointPositions;
		const PxReal* PX_RESTRICT jointVelocities = articulation.jointVelocities;
		const PxReal* PX_RESTRICT jointTargetPositions = articulation.jointTargetPositions;
		const PxReal* PX_RESTRICT jointTargetVelocities = articulation.jointTargetVelocities;

		Dy::SpatialSubspaceMatrix* motionMatrix = articulation.motionMatrix;

		PxQuat* PX_RESTRICT relativeQuats = articulation.relativeQuat;


		PxgArticulationBlockLinkData& rootLinkBlockData = articulationLinkBlocks[0];
		PxgArticulationLink& rootLink = links[0];
		rootLinkBlockData.mDisableGravity[threadIndexInWarp] = rootLink.disableGravity;
		rootLinkBlockData.mRetainsAcceleration[threadIndexInWarp] = rootLink.retainsAccelerations;
		rootLinkBlockData.mDofs[threadIndexInWarp] = 0;

		rootLinkBlockData.mLinDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW[threadIndexInWarp] =
			make_float4(rootLink.linearDamping, rootLink.angularDamping, rootLink.maxLinearVelocitySq, rootLink.maxAngularVelocitySq);
		const float4 initAngVel = rootLink.initialAngVelXYZ_penBiasClamp;
		const float4 initLinVel = rootLink.initialLinVelXYZ_invMassW;

		storeSpatialVector(rootLinkBlockData.mMotionVelocity, Cm::UnAlignedSpatialVector(PxVec3(initAngVel.x, initAngVel.y, initAngVel.z),
			PxVec3(initLinVel.x, initLinVel.y, initLinVel.z)), threadIndexInWarp);

		storeSpatialTransform(rootLinkBlockData.mAccumulatedPose, threadIndexInWarp, body2Worlds[0]);

		rootLinkBlockData.mChildrenOffset[threadIndexInWarp] = links[0].childrenOffset;
		rootLinkBlockData.mNumChildren[threadIndexInWarp] = links[0].numChildren;
		
		// initialize bitfields to 0.
		const PxU32 wordSize = (maxNumLinks + 63) / 64;
		for (PxU32 linkID = 0; linkID < maxNumLinks; ++linkID)
		{
			for (PxU32 i = 0; i < wordSize; ++i)
			{
				linkBitFieldsBlocks[linkID * wordSize + i].bitField[threadIndexInWarp] = 0;
			}
		}

		//add on root
		linkBitFieldsBlocks[0].bitField[threadIndexInWarp] |= PxU64(1) << 0;

		PxU32* pathToRootElems = articulation.pathToRoot;

		for (PxU32 linkID = 1; linkID < numLinks; linkID++)
		{
			PxgArticulationBlockLinkData& linkBlockData = articulationLinkBlocks[linkID];
			const ArticulationJointCore& joint = gJoints[linkID];
			const ArticulationJointCoreData& jointData = gJointData[linkID];
			const PxgArticulationLink& link = links[linkID];

			const PxU32 nbDofs = jointData.nbDof;

			const PxU32 jointOffset = jointData.jointOffset;

			const PxReal* PX_RESTRICT jPos = &jointPositions[jointOffset];
			const PxReal* PX_RESTRICT jVel = &jointVelocities[jointOffset];
			const PxReal* PX_RESTRICT jTargetPos = &jointTargetPositions[jointOffset];
			const PxReal* PX_RESTRICT jTargetVel = &jointTargetVelocities[jointOffset];

			for (PxU32 i = 0; i < nbDofs; ++i)
			{
				PxgArticulationBlockDofData& dofBlock = articulationDofBlocks[jointOffset + i];

				const Cm::UnAlignedSpatialVector axis = jointAxis[jointOffset + i];
				storeSpatialVector(dofBlock.mJointAxis, axis, threadIndexInWarp);
				const PxU32 dofId = joint.dofIds[i];
				dofBlock.mDofIds[threadIndexInWarp] = dofId;
				linkBlockData.mInvDofIds[dofId][threadIndexInWarp] = i;
				dofBlock.mConstraintData.mDriveStiffness[threadIndexInWarp] = joint.drives[dofId].stiffness;
				dofBlock.mConstraintData.mDamping[threadIndexInWarp] = joint.drives[dofId].damping;
				dofBlock.mConstraintData.mDriveTargetVel[threadIndexInWarp] = jTargetVel[i];
				dofBlock.mConstraintData.mTargetPosBias[threadIndexInWarp] = 0.0f;
				dofBlock.mConstraintData.mArmature[threadIndexInWarp] = joint.armature[dofId];
				dofBlock.mConstraintData.mDriveTargetPos[threadIndexInWarp] = jTargetPos[i];
				dofBlock.mConstraintData.mLimits_LowLimitX_highLimitY[threadIndexInWarp] = make_float2(joint.limits[dofId].low, joint.limits[dofId].high);
				
				dofBlock.mConstraintData.mMaxForce[threadIndexInWarp] = joint.drives[dofId].maxForce;
				//old friction
				dofBlock.mConstraintData.mFrictionCoefficient[threadIndexInWarp] = joint.frictionCoefficient;
				//new friction
				dofBlock.mConstraintData.mStaticFrictionEffort[threadIndexInWarp] = joint.frictionParams[dofId].staticFrictionEffort;
				dofBlock.mConstraintData.mDynamicFrictionEffort[threadIndexInWarp] = joint.frictionParams[dofId].dynamicFrictionEffort;
				dofBlock.mConstraintData.mViscousFrictionCoefficient[threadIndexInWarp] = joint.frictionParams[dofId].viscousFrictionCoefficient;
				dofBlock.mConstraintData.mDriveType[threadIndexInWarp] = joint.drives[dofId].driveType;

				dofBlock.mConstraintData.mMaxJointVelocity[threadIndexInWarp] = joint.maxJointVelocity[dofId];
				dofBlock.mMotion[threadIndexInWarp] = joint.motion[dofId];
				dofBlock.mJointPositions[threadIndexInWarp] = jPos[i];
				dofBlock.mJointVelocities[threadIndexInWarp] = jVel[i];
				storeSpatialVector(dofBlock.mLocalMotionMatrix, motionMatrix[linkID].columns[i], threadIndexInWarp);

			}
			linkBlockData.mRelativeQuat[threadIndexInWarp] = reinterpret_cast<float4*>(relativeQuats)[linkID];
			linkBlockData.mJointOffset[threadIndexInWarp] = jointOffset;
			linkBlockData.mDofs[threadIndexInWarp] = jointData.nbDof;
			linkBlockData.mJointType[threadIndexInWarp] = joint.jointType;
			linkBlockData.mLinDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW[threadIndexInWarp] =
				make_float4(link.linearDamping, link.angularDamping, link.maxLinearVelocitySq, link.maxAngularVelocitySq);

			linkBlockData.mParents[threadIndexInWarp] = parents[linkID];

			const PxU32 offset = link.pathToRootOffset;
			const PxU32 numPathToRoot = link.numPathToRoot;
			for (PxU32 j = 0; j < numPathToRoot; ++j)
			{
				const PxU32 index = pathToRootElems[offset + j];
				const PxU32 word = index / 64;
				linkBitFieldsBlocks[linkID * wordSize + word].bitField[threadIndexInWarp] |= PxU64(1) << (index - 64 * word);
			}

			//add on root
			linkBitFieldsBlocks[linkID * wordSize].bitField[threadIndexInWarp] |= PxU64(1) << 0;

			linkBlockData.mChildrenOffset[threadIndexInWarp] = link.childrenOffset;
			linkBlockData.mNumChildren[threadIndexInWarp] = link.numChildren;

			linkBlockData.mDisableGravity[threadIndexInWarp] = link.disableGravity;
			linkBlockData.mRetainsAcceleration[threadIndexInWarp] = link.retainsAccelerations;

			storeSpatialTransform(linkBlockData.mChildPose, threadIndexInWarp, joint.childPose);
			storeSpatialTransform(linkBlockData.mParentPose, threadIndexInWarp, joint.parentPose);
			storeSpatialTransform(linkBlockData.mAccumulatedPose, threadIndexInWarp, body2Worlds[linkID]);
		}


		initializeSpatialTendonsBlock(articulation, artiData, artiBlock, spatialTendonBlocks, attachmentBlocks, maxNumAttachments, threadIndexInWarp);

		initializeFixedTendonsBlock(articulation, artiData, artiBlock, fixedTendonBlocks, tendonJointBlocks, maxNumTendonJoints, threadIndexInWarp);

		initializeMimicJointsBlock(articulation, artiData, artiBlock, mimicJointBlocks, threadIndexInWarp);

		artiBlock.mFlags[threadIndexInWarp] = articulation.data.flags;
	}
	else if (dataDirty)
	{
		PxU32 updateDirty = artiData.updateDirty;

		updateDirty &= ~(Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON
			| Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT
			| Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON
			| Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT);


		if (updateDirty)
		{
			PxgArticulationLink* links = articulation.links;
			const PxTransform* body2Worlds = articulation.linkBody2Worlds;

			const PxReal* PX_RESTRICT jointPositions = articulation.jointPositions;
			const PxReal* PX_RESTRICT jointVelocities = articulation.jointVelocities;
			const PxReal* PX_RESTRICT jointTargetPositions = articulation.jointTargetPositions;
			const PxReal* PX_RESTRICT jointTargetVelocities = articulation.jointTargetVelocities;
			Dy::SpatialSubspaceMatrix* PX_RESTRICT motionMatrix = articulation.motionMatrix;
			PxQuat* PX_RESTRICT relativeQuats = articulation.relativeQuat;

			PxgArticulationBlockLinkData& rootLinkBlockData = articulationLinkBlocks[0];
			PxgArticulationLink& rootLink = links[0];
			rootLinkBlockData.mDisableGravity[threadIndexInWarp] = rootLink.disableGravity;

			rootLinkBlockData.mLinDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW[threadIndexInWarp] =
				make_float4(rootLink.linearDamping, rootLink.angularDamping, rootLink.maxLinearVelocitySq, rootLink.maxAngularVelocitySq);

			if (!directAPI)
			{
				const float4 initAngVel = rootLink.initialAngVelXYZ_penBiasClamp;
				const float4 initLinVel = rootLink.initialLinVelXYZ_invMassW;

				storeSpatialVector(rootLinkBlockData.mMotionVelocity, Cm::UnAlignedSpatialVector(PxVec3(initAngVel.x, initAngVel.y, initAngVel.z),
					PxVec3(initLinVel.x, initLinVel.y, initLinVel.z)), threadIndexInWarp);
			}

			storeSpatialTransform(rootLinkBlockData.mAccumulatedPose, threadIndexInWarp, body2Worlds[0]);

			// PT: preload next link data
			ArticulationJointCore& joint1 = gJoints[1];
			PxgArticulationLink& link1 = links[1];
			ArticulationJointCoreData& jointData1 = gJointData[1];

			float4 nextRelQuat = reinterpret_cast<float4*>(relativeQuats)[1];
			bool nextDisableGravity = link1.disableGravity;
			PxTransform nextJointChildPose = joint1.childPose;
			PxTransform nextJointParentPose = joint1.parentPose;
			PxReal nextLinearDamping = link1.linearDamping;
			PxReal nextAngularDamping = link1.angularDamping;
			PxReal nextMaxLinearVelocitySq = link1.maxLinearVelocitySq;
			PxReal nextMaxAngularVelocitySq = link1.maxAngularVelocitySq;
			PxU32 nextNbDofs = jointData1.nbDof;
			PxU32 nextJointOffset = jointData1.jointOffset;
			PxU32 nextDofId0 = joint1.dofIds[0];
			PxReal nextFrictionCoefficient = joint1.frictionCoefficient;

			for (PxU32 linkID = 1; linkID < numLinks; linkID++)
			{
				PxgArticulationBlockLinkData& linkBlockData = articulationLinkBlocks[linkID];
				ArticulationJointCore& joint = gJoints[linkID];

				// PT: preload next link data
				const float4 relQuat = nextRelQuat;
				const bool link_disableGravity = nextDisableGravity;
				const PxReal link_LinearDamping = nextLinearDamping;
				const PxReal link_angularDamping = nextAngularDamping;
				const PxReal link_maxLinearVelocitySq = nextMaxLinearVelocitySq;
				const PxReal link_maxAngularVelocitySq = nextMaxAngularVelocitySq;
				const PxTransform joint_ChildPose = nextJointChildPose;
				const PxTransform joint_ParentPose = nextJointParentPose;
				const PxU32 nbDofs = nextNbDofs;
				const PxU32 jointOffset = nextJointOffset;
				const PxU32 dofId0 = nextDofId0;
				const PxReal frictionCoefficient = nextFrictionCoefficient;

				if(linkID!=numLinks-1)
				{
					ArticulationJointCore& nextJoint = gJoints[linkID+1];
					PxgArticulationLink& nextLink = links[linkID+1];
					ArticulationJointCoreData& nextJointData = gJointData[linkID+1];

					nextRelQuat = reinterpret_cast<float4*>(relativeQuats)[linkID+1];
					nextDisableGravity = nextLink.disableGravity;
					nextJointChildPose = nextJoint.childPose;
					nextJointParentPose = nextJoint.parentPose;
					nextLinearDamping = nextLink.linearDamping;
					nextAngularDamping = nextLink.angularDamping;
					nextMaxLinearVelocitySq = nextLink.maxLinearVelocitySq;
					nextMaxAngularVelocitySq = nextLink.maxAngularVelocitySq;
					nextNbDofs = nextJointData.nbDof;
					nextJointOffset = nextJointData.jointOffset;
					nextDofId0 = nextJoint.dofIds[0];
					nextFrictionCoefficient = nextJoint.frictionCoefficient;
				}

				const PxReal* PX_RESTRICT jPos = &jointPositions[jointOffset];
				const PxReal* PX_RESTRICT jVel = &jointVelocities[jointOffset];
				const PxReal* PX_RESTRICT jTargetPos = &jointTargetPositions[jointOffset];
				const PxReal* PX_RESTRICT jTargetVel = &jointTargetVelocities[jointOffset];

				for (PxU32 i = 0; i < nbDofs; ++i)
				{
					PxgArticulationBlockDofData& dofBlock = articulationDofBlocks[jointOffset + i];

					const PxU32 dofId = i ? joint.dofIds[i] : dofId0;

					dofBlock.mConstraintData.mDriveStiffness[threadIndexInWarp] = joint.drives[dofId].stiffness;
					dofBlock.mConstraintData.mDamping[threadIndexInWarp] = joint.drives[dofId].damping;
					dofBlock.mConstraintData.mDriveTargetVel[threadIndexInWarp] = jTargetVel[i];
					dofBlock.mConstraintData.mTargetPosBias[threadIndexInWarp] = 0.0f;
					dofBlock.mConstraintData.mArmature[threadIndexInWarp] = joint.armature[dofId];
					dofBlock.mConstraintData.mLimits_LowLimitX_highLimitY[threadIndexInWarp] = make_float2(joint.limits[dofId].low, joint.limits[dofId].high);
					dofBlock.mConstraintData.mMaxForce[threadIndexInWarp] = joint.drives[dofId].maxForce;
					dofBlock.mConstraintData.mFrictionCoefficient[threadIndexInWarp] = frictionCoefficient;


					dofBlock.mConstraintData.mStaticFrictionEffort[threadIndexInWarp] = joint.frictionParams[dofId].staticFrictionEffort;
					dofBlock.mConstraintData.mDynamicFrictionEffort[threadIndexInWarp] = joint.frictionParams[dofId].dynamicFrictionEffort;
					dofBlock.mConstraintData.mViscousFrictionCoefficient[threadIndexInWarp] = joint.frictionParams[dofId].viscousFrictionCoefficient;

					dofBlock.mConstraintData.mDriveType[threadIndexInWarp] = joint.drives[dofId].driveType;

					dofBlock.mConstraintData.mMaxJointVelocity[threadIndexInWarp] = joint.maxJointVelocity[dofId];

					// now independent of direct-API. The remaining question is if we should just not do this here and 
					// always raise the flag for the codepath below. We would have to raise in updateArticulationsLaunch,
					// newArticulationsLaunch, and when using the direct-API.
					dofBlock.mJointPositions[threadIndexInWarp] = jPos[i];
					dofBlock.mJointVelocities[threadIndexInWarp] = jVel[i];
					dofBlock.mConstraintData.mDriveTargetPos[threadIndexInWarp] = jTargetPos[i];
					storeSpatialVector(dofBlock.mLocalMotionMatrix, motionMatrix[linkID].columns[i], threadIndexInWarp);
				}

				linkBlockData.mRelativeQuat[threadIndexInWarp] = relQuat;
				linkBlockData.mLinDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW[threadIndexInWarp] =
					make_float4(link_LinearDamping, link_angularDamping, link_maxLinearVelocitySq, link_maxAngularVelocitySq);
				linkBlockData.mDisableGravity[threadIndexInWarp] = link_disableGravity;

				storeSpatialTransform(linkBlockData.mChildPose, threadIndexInWarp, joint_ChildPose);
				storeSpatialTransform(linkBlockData.mParentPose, threadIndexInWarp, joint_ParentPose);
				storeSpatialTransform(linkBlockData.mAccumulatedPose, threadIndexInWarp, body2Worlds[linkID]);
			}
		}

		if (artiData.updateDirty & (Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON
			| Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT))
			updateSpatialTendonsBlock(articulation, artiData, spatialTendonBlocks, attachmentBlocks, maxNumAttachments, threadIndexInWarp);

		if (artiData.updateDirty & (Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON
			| Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT))
			updateFixedTendonsBlock(articulation, artiData, fixedTendonBlocks, tendonJointBlocks, maxNumTendonJoints, threadIndexInWarp);
		
		if(artiData.updateDirty & (Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT))
			updateMimicJointsBlock(articulation, artiData, artiBlock, mimicJointBlocks, threadIndexInWarp);

		// reset updateDirty to avoid re-running this code on next sim start
		artiData.updateDirty = 0;
	}

	const PxU32 artiGpuDirty = artiData.gpuDirtyFlag;

	if(artiGpuDirty)
	{
		const PxU32 totalDofs = artiData.numJointDofs;

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
		{
			const PxReal* PX_RESTRICT jointPositions = articulation.jointPositions;

			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				PxgArticulationBlockDofData& dofBlock = articulationDofBlocks[i];
				dofBlock.mJointPositions[threadIndexInWarp] = jointPositions[i];
			}
		}

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
		{
			const PxReal* PX_RESTRICT jointVelocities = articulation.jointVelocities;

			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				PxgArticulationBlockDofData& dofBlock = articulationDofBlocks[i];
				dofBlock.mJointVelocities[threadIndexInWarp] = jointVelocities[i];
			}
		}
		
		if (artiGpuDirty & (Dy::ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM | Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS))
		{
			const PxTransform* body2Worlds = articulation.linkBody2Worlds;
			for (PxU32 linkID = 0; linkID < numLinks; linkID++)
			{
				PxgArticulationBlockLinkData& linkBlockData = articulationLinkBlocks[linkID];
				storeSpatialTransform(linkBlockData.mAccumulatedPose, threadIndexInWarp, body2Worlds[linkID]);

			}
		}

		if (artiGpuDirty &  Dy::ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES)
		{
			PxgArticulationBlockLinkData& linkBlockData = articulationLinkBlocks[0];
			storeSpatialVector(linkBlockData.mMotionVelocity, articulation.motionVelocities[0], threadIndexInWarp);
		}

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
		{
			const PxReal* PX_RESTRICT jointTargetVelocities = articulation.jointTargetVelocities;

			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				PxgArticulationBlockDofData& dofBlock = articulationDofBlocks[i];
				dofBlock.mConstraintData.mDriveTargetVel[threadIndexInWarp] = jointTargetVelocities[i];
			}
		}

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
		{
			const PxReal* PX_RESTRICT jointTargetPositions = articulation.jointTargetPositions;

			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				PxgArticulationBlockDofData& dofBlock = articulationDofBlocks[i];
				dofBlock.mConstraintData.mDriveTargetPos[threadIndexInWarp] = jointTargetPositions[i];
			}
		}

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON)
		{
			PxGpuSpatialTendonData* tendonParams = articulation.spatialTendonParams;
			const PxU32 numSpatialTendons = articulation.data.numSpatialTendons;

			for (PxU32 i = 0; i < numSpatialTendons; ++i)
			{
				PxGpuSpatialTendonData& tendonData = tendonParams[i];

				PxgArticulationBlockSpatialTendonData& tendonBlockData = spatialTendonBlocks[i];
				tendonBlockData.mStiffness[threadIndexInWarp] = tendonData.stiffness;
				tendonBlockData.mDamping[threadIndexInWarp] = tendonData.damping;
				tendonBlockData.mLimitStiffness[threadIndexInWarp] = tendonData.limitStiffness;
				tendonBlockData.mOffset[threadIndexInWarp] = tendonData.offset;
			}
		}

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT)
		{
			PxgArticulationTendon* spatialTendons = articulation.spatialTendons;
			const PxU32 numSpatialTendons = articulation.data.numSpatialTendons;
			for (PxU32 i = 0; i < numSpatialTendons; ++i)
			{
				PxgArticulationTendon tendon = spatialTendons[i];
				PxgArticulationBlockAttachmentData* attachmentBlock = &attachmentBlocks[i * maxNumAttachments];

				PxGpuTendonAttachmentData* modData = reinterpret_cast<PxGpuTendonAttachmentData*>(tendon.mModElements);

				for (PxU32 j = 0; j < tendon.mNbElements; ++j)
				{
					PxGpuTendonAttachmentData& mData = modData[j];

					attachmentBlock[j].mRelativeOffset[threadIndexInWarp] = mData.relativeOffset;
					attachmentBlock[j].mRestDistance[threadIndexInWarp] = mData.restLength;
					attachmentBlock[j].mCoefficient[threadIndexInWarp] = mData.coefficient;
					attachmentBlock[j].mLowLimit[threadIndexInWarp] = mData.lowLimit;
					attachmentBlock[j].mHighLimit[threadIndexInWarp] = mData.highLimit;
				}

			}
		}

		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON)
		{
			PxGpuFixedTendonData* tendonParam = articulation.fixedTendonParams;
			const PxU32 numFixedTendons = articulation.data.numFixedTendons;

			for (PxU32 i = 0; i < numFixedTendons; ++i)
			{
				PxGpuFixedTendonData& tendonData = tendonParam[i];

				PxgArticulationBlockFixedTendonData& tendonBlockData = fixedTendonBlocks[i];
				tendonBlockData.mStiffness[threadIndexInWarp] = tendonData.stiffness;
				tendonBlockData.mDamping[threadIndexInWarp] = tendonData.damping;
				tendonBlockData.mLimitStiffness[threadIndexInWarp] = tendonData.limitStiffness;
				tendonBlockData.mOffset[threadIndexInWarp] = tendonData.offset;
				tendonBlockData.mLowLimit[threadIndexInWarp] = tendonData.lowLimit;
				tendonBlockData.mHighLimit[threadIndexInWarp] = tendonData.highLimit;
				tendonBlockData.mRestLength[threadIndexInWarp] = tendonData.restLength;

			}
		}
		
		if (artiGpuDirty & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT)
		{
		
			PxgArticulationTendon* fixedTendons = articulation.fixedTendons;
			const PxU32 numFixedTendons = articulation.data.numFixedTendons;
			for (PxU32 i = 0; i < numFixedTendons; ++i)
			{
				PxgArticulationTendon tendon = fixedTendons[i];
				PxgArticulationBlockTendonJointData* tendonJointBlock = &tendonJointBlocks[i * maxNumTendonJoints];
				PxGpuTendonJointCoefficientData* coefficientData = reinterpret_cast<PxGpuTendonJointCoefficientData*>(tendon.mModElements);
				for (PxU32 j = 0; j < tendon.mNbElements; ++j)
				{
					PxGpuTendonJointCoefficientData& cData = coefficientData[j];
					tendonJointBlock[j].mCoefficient[threadIndexInWarp] = cData.coefficient;
					tendonJointBlock[j].mRecipCoefficient[threadIndexInWarp] = cData.recipCoefficient;
					tendonJointBlock[j].mAxis[threadIndexInWarp] = cData.axis;
				}
			}
		}
		

		artiData.gpuDirtyFlag = 0;
	}
}

static __device__ void computeUnconstrainedVelocitiesInternal1T(const PxgBodySim& bodySim,
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	PxgArticulationBlockData& articulationBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT articulationLinkBlocks,
	PxgArticulationBlockDofData* PX_RESTRICT articulationDofBlocks,
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT articulationSpatialTendonBlocks,
	PxgArticulationBlockAttachmentData* PX_RESTRICT articulationAttachmentBlocks,
	PxgArticulationBlockFixedTendonData* PX_RESTRICT articulationFixedTendonBlocks,
	PxgArticulationBlockTendonJointData* PX_RESTRICT articulationTendonJointBlocks,
	PxgArticulationBlockMimicJointData* PX_RESTRICT articulationMimicJointBlocks,
	PxgArticulationBitFieldStackData* PX_RESTRICT linkBitFields,
	const PxReal invLengthScale,
	const PxU32 threadIndexInWarp,
	const bool directAPI,
	const bool forceRecomputeBlockFormat)
{
	const PxU32 articulationIndex = bodySim.articulationRemapId;

	PxgArticulation& msArticulation = scDesc->articulations[articulationIndex];

	PxgArticulationData& msArtiData = msArticulation.data;
	
	const PxU32 maxNumAttachments = scDesc->mMaxAttachmentPerArticulation;
	const PxU32 maxNumTendonJoints = scDesc->mMaxTendonJointPerArticulation;
	const PxU32 maxNumLinks = scDesc->mMaxLinksPerArticulation;

	PxU32 numLinks;
	PxU32 totalDofs;

	const bool refillBlockData = (articulationBlock.mArticulationIndex[threadIndexInWarp] != articulationIndex) || forceRecomputeBlockFormat;

	const bool isDirty = msArtiData.confiDirty;

	if (refillBlockData || isDirty)
	{
		numLinks = msArtiData.numLinks;
		totalDofs = msArtiData.numJointDofs;

		articulationBlock.mNumLinks[threadIndexInWarp] = numLinks;
		articulationBlock.mTotalDofs[threadIndexInWarp] = totalDofs;
		articulationBlock.mFlags[threadIndexInWarp] = msArtiData.flags;
		articulationBlock.mArticulationIndex[threadIndexInWarp] = articulationIndex;
		articulationBlock.mSleepThreshold[threadIndexInWarp] = bodySim.freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.z;
		articulationBlock.mStateDirty[threadIndexInWarp] = 0;

		articulationBlock.mMotionVelocitiesPtr[threadIndexInWarp] = msArticulation.motionVelocities;
	}
	else
	{
		numLinks = articulationBlock.mNumLinks[threadIndexInWarp];
		totalDofs = articulationBlock.mTotalDofs[threadIndexInWarp];
	}

	//initialize delta motion velocities and delta joint velocities to be zero
	initialize(articulationBlock, articulationLinkBlocks, articulationDofBlocks, numLinks,
		totalDofs, threadIndexInWarp);

	jcalc(msArticulation, msArtiData, scDesc, refillBlockData, articulationBlock, articulationLinkBlocks, articulationDofBlocks,
		articulationSpatialTendonBlocks, articulationAttachmentBlocks, articulationFixedTendonBlocks, 
		articulationTendonJointBlocks, articulationMimicJointBlocks, linkBitFields, numLinks, maxNumLinks, maxNumAttachments, maxNumTendonJoints,
		threadIndexInWarp, invLengthScale, directAPI);

	{
		// PT: this codepath fuses the computeRelativeTransformC2P & computeLinkVelocities loops
		// so that we do not reload parent, body2World and linkBlock.mRw_xyz multiple times.
		// A longer loop also offers more opportunities for preloading & hiding latencies.

		const Cm::UnAlignedSpatialVector rootVel = loadSpatialVector(articulationLinkBlocks[0].mMotionVelocity, threadIndexInWarp);
		articulationLinkBlocks[0].mPreTransform.p[threadIndexInWarp] = articulationLinkBlocks[0].mAccumulatedPose.p[threadIndexInWarp];
		articulationLinkBlocks[0].mPreTransform.q[threadIndexInWarp] = articulationLinkBlocks[0].mAccumulatedPose.q[threadIndexInWarp];

		//Is it really necessary? It is already resolved as an internal cosntraint.
		PxgArticulationBlockDofData* PX_RESTRICT dofs = articulationDofBlocks;
		PxReal ratio = 1.0f;
		for (PxU32 linkID = 1; linkID < numLinks; ++linkID)
		{
			PxgArticulationBlockLinkData& linkBlock = articulationLinkBlocks[linkID];
			const PxU32 dof = linkBlock.mDofs[threadIndexInWarp];
			assert(dof<=3);
			for (PxU32 ind = 0; ind < 3; ++ind)
			{
				if(ind<dof)
				{
					const PxReal maxJVelocity = dofs->mConstraintData.mMaxJointVelocity[threadIndexInWarp];
					const PxReal jVel = dofs->mJointVelocities[threadIndexInWarp];
					if (jVel != 0.0f)
						ratio = PxMin(ratio, maxJVelocity / PxAbs(jVel));
					dofs++;
				}
			}
		}

		msArticulation.rootPreMotionVelocity->top = rootVel.top;
		msArticulation.rootPreMotionVelocity->bottom = rootVel.bottom;

		//velocities contributed by joint velocities
		dofs = articulationDofBlocks;

		// PT: preload next link data
		PxU32 nextParent = articulationLinkBlocks[1].mParents[threadIndexInWarp];

		for (PxU32 linkID = 1; linkID < numLinks; linkID++)
		{
			// PT: preload next link data
			const PxU32 parent = nextParent;
			if(linkID!=numLinks-1)
			{
				nextParent = articulationLinkBlocks[linkID+1].mParents[threadIndexInWarp];
			}

			PxgArticulationBlockLinkData& linkBlock = articulationLinkBlocks[linkID];

			const PxTransform body2World = loadSpatialTransform(linkBlock.mAccumulatedPose, threadIndexInWarp);
			const PxU32 dof = linkBlock.mDofs[threadIndexInWarp];

			Cm::UnAlignedSpatialVector linkVelocity = loadSpatialVector(articulationLinkBlocks[parent].mMotionVelocity, threadIndexInWarp);
			const PxVec3 pBody2World = loadPxVec3(articulationLinkBlocks[parent].mAccumulatedPose.p, threadIndexInWarp);

			storeSpatialTransform(linkBlock.mPreTransform, threadIndexInWarp, body2World);

			const PxVec3 rw = body2World.p - pBody2World;
			linkBlock.mRw_x[threadIndexInWarp] = rw.x;
			linkBlock.mRw_y[threadIndexInWarp] = rw.y;
			linkBlock.mRw_z[threadIndexInWarp] = rw.z;
		
			translateSpatialVectorInPlace(-rw, linkVelocity);

			assert(dof<=3);
			for (PxU32 ind = 0; ind < 3; ++ind)
			{
				if(ind<dof)
				{
					PxgArticulationBlockDofData& dofData = *dofs++;

					const Cm::UnAlignedSpatialVector worldCol = loadSpatialVector(dofData.mLocalMotionMatrix, threadIndexInWarp).rotate(body2World);
					const PxReal jVel = dofData.mJointVelocities[threadIndexInWarp] * ratio;
					linkVelocity += worldCol * jVel;
					dofData.mJointVelocities[threadIndexInWarp] = jVel;
					storeSpatialVector(dofData.mWorldMotionMatrix, worldCol, threadIndexInWarp);
				}
			}

			storeSpatialVector(linkBlock.mMotionVelocity, linkVelocity, threadIndexInWarp);
		}
	}
}

extern "C" __global__ void computeUnconstrainedVelocities1TLaunch(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	const bool directAPI,
	const bool recomputeBlockFormat)
{

	const PxU32 globalWarpIndex = (blockIdx.x * blockDim.y) + threadIdx.y;

	const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
	const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;
	const PxU32 maxSpatialTendons = scDesc->mMaxSpatialTendonsPerArticulation;
	const PxU32 maxAttachments = scDesc->mMaxAttachmentPerArticulation;
	const PxU32 maxFixedTendons = scDesc->mMaxFixedTendonsPerArticulation;
	const PxU32 maxTendonJoints = scDesc->mMaxTendonJointPerArticulation;
	const PxU32 maxMimicJoints = scDesc->mMaxMimicJointsPerArticulation;

	const PxU32 globalThreadIndex = threadIdx.x + globalWarpIndex * WARP_SIZE;

	const PxU32 nbArticulations = scDesc->nbArticulations;
	const PxNodeIndex* const PX_RESTRICT gIslandNodeIndex = scDesc->islandNodeIndices;
	const PxgBodySim* const PX_RESTRICT gBodySim = scDesc->mBodySimBufferDeviceData;
	const PxU32 articulationOffset = scDesc->articulationOffset;
	PxgArticulationBlockData* PX_RESTRICT articulationBlocks = scDesc->mArticulationBlocks;
	PxgArticulationBlockLinkData* PX_RESTRICT articulationLinkBlocks = scDesc->mArticulationLinkBlocks;
	PxgArticulationBlockDofData* PX_RESTRICT articulationDofBlocks = scDesc->mArticulationDofBlocks;
	PxgArticulationBlockSpatialTendonData* PX_RESTRICT articulationSpatialTendonBlocks = scDesc->mArticulationSpatialTendonBlocks;
	PxgArticulationBlockAttachmentData* PX_RESTRICT articulationAttachmentBlocks = scDesc->mArticulationAttachmentBlocks;


	PxgArticulationBlockFixedTendonData* PX_RESTRICT articulationFixedTendonBlocks = scDesc->mArticulationFixedTendonBlocks;
	PxgArticulationBlockTendonJointData* PX_RESTRICT articulationTendonJointBlocks = scDesc->mArticulationTendonJointBlocks;
	PxgArticulationBlockMimicJointData* PX_RESTRICT articulationMimicJointBlocks = scDesc->mArticulationMimicJointBlocks;
	PxgArticulationBitFieldData* PX_RESTRICT linkBitFieldBlocks = scDesc->mPathToRootBitFieldBlocks;


	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 nodeIndex = gIslandNodeIndex[globalThreadIndex + articulationOffset].index();

		const PxU32 wordSize = (maxLinks + 63) / 64;
		const PxgBodySim& bodySim = gBodySim[nodeIndex];
		scDesc->solverBodyIndices[nodeIndex] = globalThreadIndex;
		const PxReal invLengthScale = scDesc->invLengthScale;
		// pass in the articulation's body-sim, and the blocks offset to the start of the warp
		// so the unconstrained velocity update is running single threaded, hence the name 1T
		computeUnconstrainedVelocitiesInternal1T(bodySim, scDesc, articulationBlocks[globalWarpIndex],
			&articulationLinkBlocks[globalWarpIndex * maxLinks], &articulationDofBlocks[globalWarpIndex * maxDofs],
			&articulationSpatialTendonBlocks[globalWarpIndex * maxSpatialTendons], &articulationAttachmentBlocks[globalWarpIndex * maxSpatialTendons * maxAttachments],
			&articulationFixedTendonBlocks[globalWarpIndex * maxFixedTendons], &articulationTendonJointBlocks[globalWarpIndex * maxFixedTendons * maxTendonJoints],
			&articulationMimicJointBlocks[globalWarpIndex * maxMimicJoints],
			&linkBitFieldBlocks[globalWarpIndex * maxLinks * wordSize], invLengthScale, threadIndexInWarp,
			directAPI, recomputeBlockFormat);

	}
}

static __device__ void computeSpatialInertiaW(const PxgArticulation& msArticulation,
	const PxU32 numLinks,
	const PxU32 threadIndexInWarp,
	PxgArticulationBlockData* PX_RESTRICT articulationBlocks,
	PxgArticulationBlockLinkData* PX_RESTRICT linkBlockData,
	const PxgArticulationBlockDofData* PX_RESTRICT dofBlockData,
	Cm::UnAlignedSpatialVector * PX_RESTRICT externalAccels,
	Cm::UnAlignedSpatialVector * PX_RESTRICT externalZIsolated,
	const PxVec3& gravity,
	const PxReal dt,
	const bool isExternalForcesEveryTgsIterationEnabled
	)
{
	const PxgArticulationLinkProp* const PX_RESTRICT gLinkProps = msArticulation.linkProps;

	const PxReal invDt = 1.f/dt;

	PxVec3 COM(0.f);
	PxReal totalMass = 0.f;
	
	Dy::SpatialMatrix spatialInertia;
	spatialInertia.topLeft = PxMat33(PxZero);
	spatialInertia.topRight = PxMat33(PxZero);

	// PT: preload next link data
	float4 next_invInertiaXYZ_invMass = gLinkProps[0].invInertiaXYZ_invMass;

	for (PxU32 linkID = 0; linkID < numLinks; linkID ++)
	{
		// PT: preload next link data
		const float4 invInertiaXYZ_invMass = next_invInertiaXYZ_invMass;
		if(linkID!=numLinks-1)
		{
			next_invInertiaXYZ_invMass = gLinkProps[linkID+1].invInertiaXYZ_invMass;
		}

		PxgArticulationBlockLinkData& linkData = linkBlockData[linkID];
		
		const PxTransform t = loadSpatialTransform(linkData.mAccumulatedPose, threadIndexInWarp);

		//construct mass matrix
		const PxReal m = invInertiaXYZ_invMass.w == 0.f ? 0.f : (1.f / invInertiaXYZ_invMass.w);
		//construct inertia matrix
		const PxVec3 inertiaTensor(	invInertiaXYZ_invMass.x == 0.f ? 0.f : (1.f / invInertiaXYZ_invMass.x), 
									invInertiaXYZ_invMass.y == 0.f ? 0.f : (1.f / invInertiaXYZ_invMass.y), 
									invInertiaXYZ_invMass.z == 0.f ? 0.f : (1.f / invInertiaXYZ_invMass.z));

		spatialInertia.topRight[0][0] = spatialInertia.topRight[1][1] = spatialInertia.topRight[2][2] = m;	// PxMat33::createDiagonal(PxVec3(m));

		Cm::transformInertiaTensor(inertiaTensor, PxMat33(t.q), spatialInertia.bottomLeft);
		//KS - TODO - can we propagate this back up as part of this process to avoid a global store/load?
		storeSpatialMatrix(linkData.mSpatialArticulatedInertia, threadIndexInWarp, spatialInertia);

		storePxMat33(linkData.mIsolatedInertia, threadIndexInWarp, spatialInertia.bottomLeft);

		COM += t.p * m;
		totalMass += m;

		const Cm::UnAlignedSpatialVector vel = loadSpatialVector(linkData.mMotionVelocity, threadIndexInWarp);

		const float4 linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW =
			linkData.mLinDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW[threadIndexInWarp];

		//Next, compute Z, C and D!

		PxVec3 force(0.f), torque(0.f);

		//Coriolis terms
		if (linkID != 0)
		{
			const PxU32 parent = linkData.mParents[threadIndexInWarp];
			const PxgArticulationBlockLinkData& parentLink = linkBlockData[parent];

			const Cm::UnAlignedSpatialVector pVel = loadSpatialVector(parentLink.mMotionVelocity, threadIndexInWarp);

			const float rwx = linkData.mRw_x[threadIndexInWarp];
			const float rwy = linkData.mRw_y[threadIndexInWarp];
			const float rwz = linkData.mRw_z[threadIndexInWarp];
			

			torque = pVel.top.cross(pVel.top.cross(PxVec3(rwx, rwy, rwz)));

			const PxU32 jointOffset = linkData.mJointOffset[threadIndexInWarp];
			const PxU32 nbDofs = linkData.mDofs[threadIndexInWarp];

			const PxgArticulationBlockDofData* dofs = &dofBlockData[jointOffset];

			if (nbDofs)
			{
				Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
				for (PxU32 ind = 0; ind < nbDofs; ++ind)
				{
					PxReal jVel = dofs[ind].mJointVelocities[threadIndexInWarp];
					deltaV += loadSpatialVector(dofs[ind].mWorldMotionMatrix, threadIndexInWarp) * jVel;
				}

				const PxVec3 aVec = deltaV.top;
				force = pVel.top.cross(aVec);

				//compute linear part
				const PxVec3 lVel = deltaV.bottom;

				const PxVec3 temp1 = 2.f * pVel.top.cross(lVel);
				const PxVec3 temp2 = aVec.cross(lVel);
				torque += temp1 + temp2;
			}
		}

		storeSpatialVector(linkData.mCoriolis, Cm::UnAlignedSpatialVector(force, torque), threadIndexInWarp);

		linkData.mInvInertiaXYZ_invMassW[threadIndexInWarp] = invInertiaXYZ_invMass;

		PxVec3 vA = vel.top;
		PxReal mag = vA.normalize();
		mag = PxMin(mag, invDt);
		vA *= mag;

		PxVec3 gravLinAccel(0.f);
		if (!linkData.mDisableGravity[threadIndexInWarp])
			gravLinAccel = -gravity;

		Cm::UnAlignedSpatialVector zExt(gravLinAccel*m, PxVec3(0.f));
		Cm::UnAlignedSpatialVector zDamp = Cm::UnAlignedSpatialVector::Zero();
		const Cm::UnAlignedSpatialVector zInt(PxVec3(0.f), vA.cross(spatialInertia.bottomLeft * vA));

		if (externalAccels)
		{
			const Cm::UnAlignedSpatialVector externalAccel = externalAccels[linkID];

			zExt.top += (-externalAccel.top * m);
			zExt.bottom += spatialInertia.bottomLeft * (-externalAccel.bottom);

			if(!linkData.mRetainsAcceleration[threadIndexInWarp])
				externalAccels[linkID] = Cm::UnAlignedSpatialVector::Zero();
		}

		// linear damping
		if (linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.x > 0.f ||
			linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.y > 0.f)
		{
			const PxReal linDamp = PxMin(linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.x, invDt);
			const PxReal angDamp = PxMin(linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.y, invDt);

			zDamp.top = (vel.bottom * linDamp*m) - zExt.top * linDamp*dt;
			zDamp.bottom = spatialInertia.bottomLeft * (vel.top* angDamp) - zExt.bottom * angDamp*dt;
		}

		// angular damping
		const PxReal angMag = vel.top.magnitudeSquared();
		const PxReal linMag = vel.bottom.magnitudeSquared();
		if (angMag > linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.w || linMag > linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.z)
		{
			if (angMag > linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.w)
			{
				const PxReal scale = 1.f - PxSqrt(linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.w) / PxSqrt(angMag);
				const PxVec3 tmpaccelerationAng = (spatialInertia.bottomLeft * vel.top)*scale;
				zDamp.bottom += tmpaccelerationAng*invDt;
			}

			if (linMag > linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.z)
			{
				const PxReal scale = 1.f - (PxSqrt(linDampingX_AngDampingY_maxLinVelSqZ_maxAngVelSqW.z) / PxSqrt(linMag));
				const PxVec3 tmpaccelerationLin = (vel.bottom*m*scale);
				zDamp.top += tmpaccelerationLin*invDt;
			}
		}

		Cm::UnAlignedSpatialVector zExtToStore;
		if(isExternalForcesEveryTgsIterationEnabled)
		{
			// Contrary to the CPU version, where we can just re-use the externalAcceleration buffer to store
			// the isolated external Z forces, we cannot do this here because the externalAccelerations is only
			// refreshed on demand if something changes, so we must not override it. Instead, we write into the zAForces
			// buffer, which otherwise just serves as a scratch space.
			// We could consider also applying the internal forces per substep, in which case we can even use the linkData.mZAVector (PX-4793)
			externalZIsolated[linkID] = zExt;
			zExtToStore = zDamp;
		}
		else
		{
			zExtToStore = zDamp + zExt;
		}

		//KS - we store in 2x places because we need the bias force afterwards to compute the articulated
		//transmission force
		storeSpatialVector(linkData.mZAVector, zExtToStore, threadIndexInWarp);
		storeSpatialVector(linkData.mZAIntVector, zInt, threadIndexInWarp);
		storeSpatialVector(linkData.mBiasForce, zExtToStore + zInt, threadIndexInWarp);

		linkData.mMass[threadIndexInWarp] = m;
	}

	const PxReal invMass = 1.f / totalMass;
	COM *= invMass;

	articulationBlocks->mCOM_TotalInvMassW[threadIndexInWarp] = make_float4(COM.x, COM.y, COM.z, invMass);
}

extern "C" __global__ void computeUnconstrainedSpatialInertiaLaunchPartial1T(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc)
{
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = threadIdx.x + globalWarpIndex * WARP_SIZE;

	const PxU32 nbArticulations = scDesc->nbArticulations;
	if (globalThreadIndex < nbArticulations)
	{
		const PxNodeIndex* const PX_RESTRICT gIslandNodeIndex = scDesc->islandNodeIndices;
		const PxgBodySim* const PX_RESTRICT gBodySim = scDesc->mBodySimBufferDeviceData;
		const PxU32 articulationOffset = scDesc->articulationOffset;

		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

		PxgArticulationBlockLinkData* PX_RESTRICT linkBlockData = scDesc->mArticulationLinkBlocks + globalWarpIndex * maxLinks;
		PxgArticulationBlockDofData* PX_RESTRICT dofBlockData = scDesc->mArticulationDofBlocks + globalWarpIndex * maxDofs;
		PxgArticulationBlockData* PX_RESTRICT articulationBlocks = scDesc->mArticulationBlocks + globalWarpIndex;
	
		const PxU32 nodeIndex = gIslandNodeIndex[globalThreadIndex + articulationOffset].index();
		const PxgBodySim& bodySim = gBodySim[nodeIndex];

		const PxU32 articulationIndex = bodySim.articulationRemapId;
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];
		const PxU32 numLinks = articulation.data.numLinks;

		computeSpatialInertiaW(articulation, numLinks, threadIndexInWarp, articulationBlocks, linkBlockData, dofBlockData,
	                       articulation.externalAccelerations, articulation.zAForces, scDesc->gravity, scDesc->dt,
	                       scDesc->isExternalForcesEveryTgsIterationEnabled);
	}
}

static __device__ PX_FORCE_INLINE void computeIs(
	const PxU32 dofs,
	const Dy::SpatialMatrix& spatialInertia,
	Cm::UnAlignedSpatialVector* Is,
	PxgArticulationBlockDofData* dofData,
	PxgArticulationBlockLinkData& linkData,
	const PxU32 threadIndexInWarp)
{	
	assert(dofs<=3);
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if(ind<dofs)
		{
			Cm::UnAlignedSpatialVector lIs = spatialInertia * loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIdx.x);
			Is[ind] = lIs;
			storeSpatialVector(dofData[ind].mIsW, lIs, threadIdx.x);
		}
	}
}

static __device__ SpatialMatrix constructSpatialMatrix(const Cm::UnAlignedSpatialVector& Is, const Cm::UnAlignedSpatialVector& stI)
{
	//construct top left
	PxVec3 tLeftC0 = Is.top * stI.top.x;
	PxVec3 tLeftC1 = Is.top * stI.top.y;
	PxVec3 tLeftC2 = Is.top * stI.top.z;

	PxMat33 topLeft(tLeftC0, tLeftC1, tLeftC2);

	//construct top right
	PxVec3 tRightC0 = Is.top * stI.bottom.x;
	PxVec3 tRightC1 = Is.top * stI.bottom.y;
	PxVec3 tRightC2 = Is.top * stI.bottom.z;
	PxMat33 topRight(tRightC0, tRightC1, tRightC2);

	//construct bottom left
	PxVec3 bLeftC0 = Is.bottom * stI.top.x;
	PxVec3 bLeftC1 = Is.bottom * stI.top.y;
	PxVec3 bLeftC2 = Is.bottom * stI.top.z;
	PxMat33 bottomLeft(bLeftC0, bLeftC1, bLeftC2);

	return SpatialMatrix(topLeft, topRight, bottomLeft);
}

// Note:
// the propagation of articulated spatial inertia Ii^A from child i to parent h follows a simple rule that applies a delta
// to the isolated articulated spatial inertia of the parent. We can write Ih^A as follows, assuming it has been
// initialized as the isolated articulated spatial inertia:
// 	Ih^A += delta
// with delta having the form
// 	delta = Ii^A - (Ii^A * si * si^T * Ii^A)/(si * Ii^A * si)
// This function computes and returns the term
// 	(I^A * s * s^T * I^A)/(s * I^A * s)
// To complete the computation of the delta to apply to the parent articulated spatial inertia it is necessary to compute
// 	Ii^A - computePropagateSpatialInertia_ZA_ZIc()
static __device__ Dy::SpatialMatrix computePropagateSpatialInertia_ZA_ZIc(PxgArticulationBlockLinkData& linkData,
	PxgArticulationBlockDofData* dofData,
	const Cm::UnAlignedSpatialVector* const PX_RESTRICT msIs, 
	const PxReal* const PX_RESTRICT jF, // can be NULL in which case assume zero joint forces
	const Cm::UnAlignedSpatialVector& Z,
	const Cm::UnAlignedSpatialVector& ZIcInt,
	Cm::UnAlignedSpatialVector& ZA,
	Cm::UnAlignedSpatialVector& ZAInt,
	const PxU32 threadIndexInWarp,
	const PxU32 linkID)
{
	const PxU8 jointType = linkData.mJointType[threadIndexInWarp];

	switch (jointType)
	{
	case PxArticulationJointType::ePRISMATIC:
	case PxArticulationJointType::eREVOLUTE:
	case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
	{
		const Cm::UnAlignedSpatialVector sa = loadSpatialVector(dofData[0].mWorldMotionMatrix, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector& Is = msIs[0];

		PxReal stIs = sa.innerProduct(Is) + dofData[0].mConstraintData.mArmature[threadIndexInWarp];
		
		const PxReal iStIs = ((stIs > 0.f) ? (1.f / stIs) : 0.f);
		dofData[0].mInvStIsT_x[threadIndexInWarp] = iStIs;

		Cm::UnAlignedSpatialVector isInvD = Is * iStIs;

		storeSpatialVector(dofData[0].mIsInvDW, isInvD, threadIndexInWarp);
		//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
		Cm::UnAlignedSpatialVector stI(Is.bottom, Is.top);

		const PxReal stZ = sa.innerProduct(Z);
		const PxReal stZInt = sa.innerProduct(ZIcInt);

		//link.qstZIc[ind] = jF[ind] - stZ;
		//const PxReal qstZic = jF[0] - stZ;
		const PxReal qstZ = -stZ;
		const PxReal qstZIcInternal = (jF ? jF[0] : 0.0f) - stZInt;
		dofData[0].mQstZ[threadIndexInWarp] = qstZ;
		dofData[0].mQstZIcInternal[threadIndexInWarp] = qstZIcInternal;

		ZA += isInvD * qstZ;
		ZAInt += isInvD * qstZIcInternal;

		return constructSpatialMatrix(isInvD, stI);
	}
	case PxArticulationJointType::eSPHERICAL:
	{

		PxMat33 invStIs(PxIdentity);
		
		const PxU32 dofs = linkData.mDofs[threadIndexInWarp];
		assert(dofs<=3);
		PxReal qstZG[3];
		PxReal qstZIcIntG[3];
#pragma unroll(3)
		for (PxU32 ind2 = 0; ind2 < 3; ++ind2)
		{
			if (ind2 < dofs)
			{
				const Cm::UnAlignedSpatialVector sa = loadSpatialVector(dofData[ind2].mWorldMotionMatrix, threadIndexInWarp);
				for (PxU32 ind = 0; ind < 3; ++ind)
				{
					if(ind<dofs)
						invStIs[ind][ind2] = sa.innerProduct(msIs[ind]);
				}

				invStIs[ind2][ind2] += dofData[ind2].mConstraintData.mArmature[threadIndexInWarp];

				const PxReal stZ = sa.innerProduct(Z);
				const PxReal stZInt = sa.innerProduct(ZIcInt);

				//link.qstZIc[ind] = jF[ind] - stZ;
				const PxReal qstZ = -stZ;
				const PxReal qstZicInt = (jF ? jF[ind2] : 0.0f) - stZInt;
				qstZG[ind2] = qstZ;
				qstZIcIntG[ind2] = qstZicInt;
				dofData[ind2].mQstZ[threadIndexInWarp] = qstZ;
				dofData[ind2].mQstZIcInternal[threadIndexInWarp] = qstZicInt;
			}
		}

		//invStIs = SpatialMatrix::invertSym33(invStIs);
		invStIs = invStIs.getInverse();
		for (PxU32 ind2 = 0; ind2 < 3; ++ind2)
		{
			if(ind2<dofs)
			{
				dofData[ind2].mInvStIsT_x[threadIndexInWarp] = invStIs[0][ind2];
				dofData[ind2].mInvStIsT_y[threadIndexInWarp] = invStIs[1][ind2];
				dofData[ind2].mInvStIsT_z[threadIndexInWarp] = invStIs[2][ind2];
			}
		}

		Cm::UnAlignedSpatialVector columns[6];
		columns[0] = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));
		columns[1] = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));
		columns[2] = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));
		columns[3] = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));
		columns[4] = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));
		columns[5] = Cm::UnAlignedSpatialVector(PxVec3(0.f), PxVec3(0.f));

#pragma unroll (3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			if (ind < dofs)
			{
				Cm::UnAlignedSpatialVector isID(PxVec3(0.f), PxVec3(0.f));

				for (PxU32 ind2 = 0; ind2 < 3; ++ind2)
				{
					if(ind2<dofs)
					{
						const Cm::UnAlignedSpatialVector& Is = msIs[ind2];
						isID += Is * invStIs[ind][ind2];
					}
				}

				columns[0] += isID * msIs[ind].bottom.x;
				columns[1] += isID * msIs[ind].bottom.y;
				columns[2] += isID * msIs[ind].bottom.z;
				columns[3] += isID * msIs[ind].top.x;
				columns[4] += isID * msIs[ind].top.y;
				columns[5] += isID * msIs[ind].top.z;

				storeSpatialVector(dofData[ind].mIsInvDW, isID, threadIndexInWarp);

				ZA += isID * qstZG[ind];
				ZAInt += isID * qstZIcIntG[ind];
			}
		}

		return SpatialMatrix::constructSpatialMatrix(columns);
	}
	default:
	{
		return SpatialMatrix(PxZero);
	}

	}
}

static __device__ void translateInertia(const PxMat33& sTod, Dy::SpatialMatrix& inertia)
{
	const PxMat33 dTos = sTod.getTranspose();

	PxMat33 bl = sTod * inertia.topLeft + inertia.bottomLeft;
	PxMat33 br = sTod * inertia.topRight + inertia.getBottomRight();

	inertia.topLeft = inertia.topLeft + inertia.topRight * dTos;
	inertia.bottomLeft = bl + br * dTos;

	//aligned inertia - make it symmetrical! OPTIONAL!!!!
	inertia.bottomLeft = (inertia.bottomLeft + inertia.bottomLeft.getTranspose()) * 0.5f;
}

// PT: TODO: take advantage of zeros?
static __device__ inline PxMat33 constructSkewSymmetricMatrix(const float4 r)
{
	return PxMat33(	PxVec3(0.f, r.z, -r.y),
					PxVec3(-r.z, 0.f, r.x),
					PxVec3(r.y, -r.x, 0.f));
}

// PT: TODO: take advantage of zeros?
static __device__ inline PxMat33 translateInertia(const PxMat33& inertia, const PxReal mass, const PxVec3& t)
{
	PxMat33 s(PxVec3(0, t.z, -t.y),
		PxVec3(-t.z, 0, t.x),
		PxVec3(t.y, -t.x, 0));

	PxMat33 translatedIT = s.getTranspose() * s * mass + inertia;
	return translatedIT;
}


static __device__ void computeArticulatedSpatialInertiaW(
	const PxU32 numLinks,
	PxgArticulationBlockLinkData* PX_RESTRICT linkBlockData,
	PxgArticulationBlockDofData* PX_RESTRICT dofBlockData,
	PxgArticulationBlockData& articulationBlock,
	const PxReal* jointForces,
	const bool isExternalForcesEveryTgsIterationEnabled,
	const PxU32 threadIndexInWarp)
{
	for (PxU32 linkID = numLinks - 1; linkID > 0; --linkID)
	{
		PxgArticulationBlockLinkData& blockData = linkBlockData[linkID];
		
		const PxU32 parent = blockData.mParents[threadIdx.x];
		const PxU32 dof = blockData.mDofs[threadIdx.x];
		const PxU32 jointOffset = blockData.mJointOffset[threadIdx.x];

		const float rwx = blockData.mRw_x[threadIndexInWarp];
		const float rwy = blockData.mRw_y[threadIndexInWarp];
		const float rwz = blockData.mRw_z[threadIndexInWarp];

		PxgArticulationBlockDofData* PX_RESTRICT dofData = dofBlockData + jointOffset;

		Dy::SpatialMatrix parentSpatialArticulatedInertia;
		loadSpatialMatrix(linkBlockData[parent].mSpatialArticulatedInertia, threadIdx.x, parentSpatialArticulatedInertia);

		Dy::SpatialMatrix articulatedInertia;
		Cm::UnAlignedSpatialVector msIs[3];
		loadSpatialMatrix(blockData.mSpatialArticulatedInertia, threadIdx.x, articulatedInertia);

		const Cm::UnAlignedSpatialVector coriolis = loadSpatialVector(blockData.mCoriolis, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector spatialZA = loadSpatialVector(blockData.mZAVector, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector spatialZAInt = loadSpatialVector(blockData.mZAIntVector, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector parentZAVector = loadSpatialVector(linkBlockData[parent].mZAVector, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector parentZAIntVector = loadSpatialVector(linkBlockData[parent].mZAIntVector, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector Ic = articulatedInertia * coriolis;
		const Cm::UnAlignedSpatialVector ZIcInt = spatialZAInt + Ic;

		Cm::UnAlignedSpatialVector translatedZA = spatialZA;
		Cm::UnAlignedSpatialVector translatedZAInt = ZIcInt;

		const PxReal* const PX_RESTRICT jF = isExternalForcesEveryTgsIterationEnabled ? NULL : &jointForces[jointOffset];

		computeIs(dof, articulatedInertia, msIs, dofData, blockData, threadIdx.x);
		Dy::SpatialMatrix spatialInertiaW = articulatedInertia - computePropagateSpatialInertia_ZA_ZIc(blockData, dofData, msIs, jF, spatialZA, ZIcInt, translatedZA, translatedZAInt, threadIdx.x, linkID);

		//accumulate childen's articulated zero acceleration force to parent's articulated zero acceleration
		translateSpatialVectorInPlace(PxVec3(rwx, rwy, rwz), translatedZA);
		translateSpatialVectorInPlace(PxVec3(rwx, rwy, rwz), translatedZAInt);

		storeSpatialVector(linkBlockData[parent].mZAVector, parentZAVector + translatedZA, threadIndexInWarp);
		storeSpatialVector(linkBlockData[parent].mZAIntVector, parentZAIntVector + translatedZAInt, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector pSpatialZA = loadSpatialVector(linkBlockData[parent].mZAVector, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector pspatialZAInt = loadSpatialVector(linkBlockData[parent].mZAIntVector, threadIndexInWarp);

		//transform spatial inertia into parent space
		translateInertia(constructSkewSymmetricMatrix(make_float4(rwx, rwy, rwz, 0.f)), spatialInertiaW);

		const PxReal minPropagatedInertia = 0.f;

		// Make sure we do not propagate up negative inertias around the principal inertial axes 
		// due to numerical rounding errors
		spatialInertiaW.bottomLeft.column0.x = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column0.x);
		spatialInertiaW.bottomLeft.column1.y = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column1.y);
		spatialInertiaW.bottomLeft.column2.z = PxMax(minPropagatedInertia, spatialInertiaW.bottomLeft.column2.z);

		storeSpatialMatrix(linkBlockData[parent].mSpatialArticulatedInertia, threadIndexInWarp, parentSpatialArticulatedInertia + spatialInertiaW);
	}

	Dy::SpatialMatrix inertia;
	
	if (articulationBlock.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE)
	{
		inertia.setZero();
	}
	else
	{
		loadSpatialMatrix(linkBlockData[0].mSpatialArticulatedInertia, threadIdx.x, inertia);
		inertia = inertia.invertInertia();
	}
	storeSpatialMatrix(articulationBlock.mInvSpatialArticulatedInertia, threadIndexInWarp, inertia);
}

extern "C" __global__ void computeUnconstrainedSpatialInertiaLaunch1T(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc)
{
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	const PxU32 nbArticulations = scDesc->nbArticulations;
	if (globalThreadIndex < nbArticulations)
	{
		const PxNodeIndex* const PX_RESTRICT gIslandNodeIndex = scDesc->islandNodeIndices;
		const PxgBodySim* const PX_RESTRICT gBodySim = scDesc->mBodySimBufferDeviceData;
		const PxU32 articulationOffset = scDesc->articulationOffset;

		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

		PxgArticulationBlockLinkData* PX_RESTRICT linkBlockData = scDesc->mArticulationLinkBlocks + globalWarpIndex * maxLinks;
		PxgArticulationBlockDofData* PX_RESTRICT dofBlockData = scDesc->mArticulationDofBlocks + globalWarpIndex * maxDofs;
		PxgArticulationBlockData* PX_RESTRICT articulationBlocks = scDesc->mArticulationBlocks + globalWarpIndex;

		const PxU32 nodeIndex = gIslandNodeIndex[globalThreadIndex + articulationOffset].index();
		const PxgBodySim& bodySim = gBodySim[nodeIndex];

		const PxU32 numLinks = articulationBlocks->mNumLinks[threadIndexInWarp];
		const PxgArticulation& articulation = scDesc->articulations[bodySim.articulationRemapId];

		computeArticulatedSpatialInertiaW(numLinks, linkBlockData, dofBlockData, *articulationBlocks, articulation.jointForce,
											scDesc->isExternalForcesEveryTgsIterationEnabled, threadIndexInWarp);
	}
}

static __device__ Cm::UnAlignedSpatialVector propagateImpulseW_2(const Cm::UnAlignedSpatialVector* isInvD, const PxVec3& childToParent,
	const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::UnAlignedSpatialVector& Z, const PxU32 dofCount, PxReal* qstZ)
{
	Cm::UnAlignedSpatialVector temp = Z;

#pragma unroll (3)
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			const PxReal stZ = -motionMatrix[ind].innerProduct(Z);
			qstZ[ind] += stZ;
			temp += isInvD[ind] * stZ;
		}
	}

	//parent space's spatial zero acceleration impulse
	return FeatherstoneArticulation::translateSpatialVector(childToParent, temp);
}

static __device__ PX_FORCE_INLINE void writeMatrix(const PxSpatialMatrix& matrix, PxSpatialMatrix* dstMatrix, 
	const PxU32 mask, const PxU32 threadIndexInWarp, bool isActive)
{
	const PxU32 NbPerPass = 8;
	__shared__ PxSpatialMatrix shMatrix[NbPerPass];
	__shared__ PxSpatialMatrix* outBuff[NbPerPass];

	const PxU32 count = __popc(mask); //how many things we need to write out...

	const PxU32 offset = warpScanExclusive(mask, threadIndexInWarp);
	for (PxU32 i = 0; i < count; i += NbPerPass)
	{
		PxU32 off = offset - i;
		if (isActive && off < NbPerPass)
		{
			//Time to write this one out!
			shMatrix[off] = matrix;
			outBuff[off] = dstMatrix;
		}

		__syncwarp();
		//Now let's write this to global address...
		const PxU32 NbToProcess = PxMin(count - i, NbPerPass);
		for (PxU32 j = 0; j < NbToProcess; ++j)
		{
			uint2* dst = reinterpret_cast<uint2*>(outBuff[j]);
			uint2* src = reinterpret_cast<uint2*>(&shMatrix[j]);

			if(threadIndexInWarp < 18)
				dst[threadIndexInWarp] = src[threadIndexInWarp];
		}

		__syncwarp(); //Syncwarp needed because outBuff and shMatrix (shared memory) is read and written in the same loop
	}

}

extern "C" __global__ void computeMassMatrix1T(const PxgArticulationCoreDesc* const PX_RESTRICT scDesc)
{

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.y;

	const PxU32 nbArticulations = scDesc->nbArticulations;
	//The number of articulation blocks.
	const PxU32 nbArticulationBlocks = (nbArticulations + (WARP_SIZE-1))/WARP_SIZE;


	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	if(globalWarpIndex < nbArticulationBlocks)
	{
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

		const PxU32 articulationsInBlock = PxMin(32u, nbArticulations - globalWarpIndex*WARP_SIZE);

		PxgArticulationBlockData& articBlock = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* linkBlocks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* dofBlocks = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];

		PxgArticulation* articulations = scDesc->articulations;

		PxU32 numLinks = 0;

		bool active = globalThreadIndex < nbArticulations;
		numLinks = active ? articBlock.mNumLinks[threadIndexInWarp] : 0;

		const PxU32 invalidArticulationId = 0xFFFFFFFF;
		PxU32 articulationIndex = invalidArticulationId;

		PxSpatialMatrix* responseMatrices;
		PxReal* cfmScale;
		PxReal* cfm;

		if (active)
		{
			articulationIndex = articBlock.mArticulationIndex[threadIndexInWarp];
			PxgArticulation& artic = articulations[articulationIndex];
			responseMatrices = artic.spatialResponseMatrixW;
			cfmScale = artic.cfmScale;
			cfm = artic.cfms;
		}

		bool fixBase = active && articBlock.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE; 
		bool notFixBase = !fixBase && active;

		PxU32 fixMask = __ballot_sync(FULL_MASK, fixBase);
		PxU32 notFixMask = __ballot_sync(FULL_MASK, notFixBase);

		if (fixMask)
		{
			//Fixed base, so response is zero
			if(fixBase)
				zeroSpatialMatrix(linkBlocks[0].mSpatialResponseMatrix, threadIndexInWarp);

			for (PxU32 i = fixMask; i != 0; i = clearLowestSetBit(i))
			{
				//This is the articulation index that we will be writing to...
				const PxU32 artiInd = __shfl_sync(FULL_MASK, articulationIndex, lowestSetIndex(i));
				assert(artiInd != invalidArticulationId);
				PxgArticulation& artic = articulations[artiInd];
				const uint zero = 0;
				warpCopy<uint>(reinterpret_cast<uint*>(artic.spatialResponseMatrixW), zero, sizeof(PxSpatialMatrix));
			}

			if (fixBase)
			{
				cfm[0] = 0.f;

				linkBlocks[0].mCfm[threadIndexInWarp] = 0.f;
			}

		}
		
		if(notFixMask)
		{
			//Compute impulse response matrix. Compute the impulse response of unit responses on all 6 axes...
			SpatialMatrix inverseArticulatedInertiaW;
			PxSpatialMatrix response;

			if (notFixBase)
			{
				loadSpatialMatrix(articBlock.mInvSpatialArticulatedInertia, threadIndexInWarp, inverseArticulatedInertiaW);

				for (PxU32 i = 0; i < 6; ++i)
				{
					Cm::UnAlignedSpatialVector vec = Cm::UnAlignedSpatialVector::Zero();
					vec[i] = 1.f;
					Cm::UnAlignedSpatialVector r = inverseArticulatedInertiaW * vec;

					response.column[i][0] = r.top.x; response.column[i][1] = r.top.y; response.column[i][2] = r.top.z;
					response.column[i][3] = r.bottom.x; response.column[i][4] = r.bottom.y; response.column[i][5] = r.bottom.z;
				}

				storeSpatialMatrix(linkBlocks[0].mSpatialResponseMatrix, threadIndexInWarp, response);

				PxReal cfmVal = cfmScale[0] * PxMax(inverseArticulatedInertiaW.bottomLeft.column0.x, PxMax(inverseArticulatedInertiaW.bottomLeft.column1.y, inverseArticulatedInertiaW.bottomLeft.column2.z));

				cfm[0] = cfmVal;
				linkBlocks[0].mCfm[threadIndexInWarp] = cfmVal;
			}

			writeMatrix(response, responseMatrices, notFixMask, threadIndexInWarp, notFixBase);
		}


		PxU32 maxLinksInWarp = warpReduction<MaxOpPxU32, PxU32>(FULL_MASK, numLinks);

		for (PxU32 linkID = 1; linkID < maxLinksInWarp; ++linkID)
		{
			//__syncwarp();

			PxSpatialMatrix response;

			bool thisThreadActive = linkID < numLinks;

			PxU32 activeThreadMask = __ballot_sync(FULL_MASK, thisThreadActive);

			if (thisThreadActive)
			{
				PxgArticulationBlockLinkData& thisLink = linkBlocks[linkID];
				//KS - techically, we could avoid reading this data by instead stepping based on dof below
				//Will evaluate to see if that's worthwhile doing but keeping it simple for now
				PxgArticulationBlockDofData* dofData = dofBlocks + thisLink.mJointOffset[threadIndexInWarp];

				const PxU32 parentId = thisLink.mParents[threadIndexInWarp];

				const float rwx = thisLink.mRw_x[threadIndexInWarp];
				const float rwy = thisLink.mRw_y[threadIndexInWarp];
				const float rwz = thisLink.mRw_z[threadIndexInWarp];

				const PxVec3 rw(rwx, rwy, rwz);
				const PxU32 dof = thisLink.mDofs[threadIndexInWarp];


				Cm::UnAlignedSpatialVector motionV[3];
				float3 invStIsT[3];
				Cm::UnAlignedSpatialVector isInvD[3];
				Cm::UnAlignedSpatialVector isW[3];
				PxSpatialMatrix parentResponse;

				loadSpatialMatrix(linkBlocks[parentId].mSpatialResponseMatrix, threadIndexInWarp, parentResponse);

				assert(dof<=3);
				for (PxU32 i = 0; i < 3; ++i)
				{
					if(i<dof)
					{
						motionV[i] = loadSpatialVector(dofData[i].mWorldMotionMatrix, threadIndexInWarp);
						invStIsT[i] = make_float3(dofData[i].mInvStIsT_x[threadIndexInWarp], dofData[i].mInvStIsT_y[threadIndexInWarp], dofData[i].mInvStIsT_z[threadIndexInWarp]);
						isInvD[i] = loadSpatialVector(dofData[i].mIsInvDW, threadIndexInWarp);
						isW[i] = loadSpatialVector(dofData[i].mIsW, threadIndexInWarp);
					}
				}

								
#pragma unroll (6)
				for (PxU32 i = 0; i < 6; ++i)
				{
					//Impulse has to be negated!

					Cm::UnAlignedSpatialVector vec(PxVec3(0.f), PxVec3(0.f));
					vec[i] = -1.f;

					PxReal qstZ[3] = { 0.f, 0.f, 0.f };
					//(1) Propagate impulse to parent
					const Cm::UnAlignedSpatialVector Zp = propagateImpulseW_2(isInvD, rw, motionV, vec, dof, qstZ);

					//(2) Get deltaV response for parent
					const Cm::UnAlignedSpatialVector Zr = -(parentResponse * Zp);
					const Cm::UnAlignedSpatialVector deltaV = propagateAccelerationW(rw, invStIsT, motionV, Zr, dof, isW, qstZ);

					response.column[i][0] = deltaV.top.x; response.column[i][1] = deltaV.top.y; response.column[i][2] = deltaV.top.z;
					response.column[i][3] = deltaV.bottom.x; response.column[i][4] = deltaV.bottom.y; response.column[i][5] = deltaV.bottom.z;

				}

				PxReal cfmVal = cfmScale[linkID] * PxMax(response.column[0][3], PxMax(response.column[1][4], response.column[2][5]));

				cfm[linkID] = cfmVal; 
				thisLink.mCfm[threadIndexInWarp] = cfmVal;

				storeSpatialMatrix(thisLink.mSpatialResponseMatrix, threadIndexInWarp, response);
			}

			writeMatrix(response, responseMatrices + linkID, activeThreadMask, threadIndexInWarp, thisThreadActive);
		}
	}
}

static __device__ void computeLinkAcceleration(PxgArticulationBlockData& PX_RESTRICT articulationBlock,
	PxgArticulationBlockLinkData* PX_RESTRICT articulationLinks,
	PxgArticulationBlockDofData* PX_RESTRICT articulationDofs,
	const PxU32 numLinks,
	const PxReal dt,
	const PxU32 threadIndexInWarp)
{
	const bool fixBase = articulationBlock.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;
	Cm::UnAlignedSpatialVector* PX_RESTRICT motionVelocities = articulationBlock.mMotionVelocitiesPtr[threadIndexInWarp];

	const float4 COM_invMassW = articulationBlock.mCOM_TotalInvMassW[threadIndexInWarp];
	const PxVec3 COM(COM_invMassW.x, COM_invMassW.y, COM_invMassW.z);

	Cm::UnAlignedSpatialVector preMomentum(PxVec3(0.f), PxVec3(0.f));
	Cm::UnAlignedSpatialVector postMomentum(PxVec3(0.f), PxVec3(0.f));
	PxMat33 compoundInertia(PxZero);
	if (!fixBase)
	{

		Dy::SpatialMatrix invInertia;
		loadSpatialMatrix(articulationBlock.mInvSpatialArticulatedInertia, threadIndexInWarp, invInertia);
		const Cm::UnAlignedSpatialVector za = loadSpatialVector(articulationLinks[0].mZAVector, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector accel = -(invInertia * za);
		storeSpatialVector(articulationLinks[0].mMotionAcceleration, 
			accel, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector motionVel = loadSpatialVector(articulationLinks[0].mMotionVelocity, threadIndexInWarp) + accel * dt;

		const PxReal mass = articulationLinks[0].mMass[threadIndexInWarp];
		preMomentum.top = motionVel.bottom * mass;

		storeSpatialVector(articulationLinks[0].mMotionVelocity, motionVel, threadIndexInWarp);
	}
	else
	{
		storeSpatialVector(articulationLinks[0].mMotionAcceleration, 
			Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		storeSpatialVector(articulationLinks[0].mMotionAccelerationInternal,
			Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		storeSpatialVector(articulationLinks[0].mMotionVelocity,
			Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);

		//Store motionVel in flat array (inefficient) - required by the constraint prep, which does not process block data!
		motionVelocities[0] = Cm::UnAlignedSpatialVector::Zero();
	}

	// PT: preload next link data
	PxU32 nextJointOffset = articulationLinks[1].mJointOffset[threadIndexInWarp];
	PxU32 nextParent = articulationLinks[1].mParents[threadIndexInWarp];

	for (PxU32 linkID = 1; linkID < numLinks; ++linkID)
	{
		PxgArticulationBlockLinkData& tLink = articulationLinks[linkID];

		// PT: preload next link data
		const PxU32 jointOffset = nextJointOffset;
		const PxU32 parent = nextParent;
		if(linkID!=numLinks-1)
		{
			nextJointOffset = articulationLinks[linkID+1].mJointOffset[threadIndexInWarp];
			nextParent = articulationLinks[linkID+1].mParents[threadIndexInWarp];
		}

		PxgArticulationBlockDofData* PX_RESTRICT tDofs = &articulationDofs[jointOffset];

		const float rwx = tLink.mRw_x[threadIndexInWarp];
		const float rwy = tLink.mRw_y[threadIndexInWarp];
		const float rwz = tLink.mRw_z[threadIndexInWarp];

		const Cm::UnAlignedSpatialVector linkMotionVel = loadSpatialVector(tLink.mMotionVelocity, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(PxVec3(-rwx, -rwy, -rwz), 
			loadSpatialVector(articulationLinks[parent].mMotionAcceleration, threadIndexInWarp));

		Cm::UnAlignedSpatialVector motionAcceleration = pMotionAcceleration;
		//Cm::UnAlignedSpatialVector motionAccelerationInternal = pMotionAccelerationInt +loadSpatialVector(tLink.mCoriolis, threadIndexInWarp);

		const PxU32 dofs = tLink.mDofs[threadIndexInWarp];

		Cm::UnAlignedSpatialVector isWs[3];
		PxReal qstZ[3];
		PxReal jointVel[3];
		Cm::UnAlignedSpatialVector axes[3];
		float invStIsTx[3];
		float invStIsTy[3];
		float invStIsTz[3];

// the split into three separate loops is an optimization that allows dispatching the loads as early as possible.
#pragma unroll (3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			if (ind < dofs)
			{
				isWs[ind] = loadSpatialVector(tDofs[ind].mIsW, threadIndexInWarp);
				qstZ[ind] = tDofs[ind].mQstZ[threadIndexInWarp];
				jointVel[ind] = tDofs[ind].mJointVelocities[threadIndexInWarp];
				axes[ind] = loadSpatialVector(tDofs[ind].mWorldMotionMatrix, threadIndexInWarp);
				invStIsTx[ind] = tDofs[ind].mInvStIsT_x[threadIndexInWarp];
				invStIsTy[ind] = tDofs[ind].mInvStIsT_y[threadIndexInWarp];
				invStIsTz[ind] = tDofs[ind].mInvStIsT_z[threadIndexInWarp];
			}
		}

		PxReal tJAccel[3];
#pragma unroll (3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			PxReal jAccel = 0.f;
			if (ind < dofs)
			{
				jAccel = qstZ[ind] - isWs[ind].innerProduct(pMotionAcceleration);;
			}
			tJAccel[ind] = jAccel;
		}

		//calculate jointAcceleration

#pragma unroll(3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			if (ind < dofs)
			{
				const float invStIsT_x = invStIsTx[ind];
				const float invStIsT_y = invStIsTy[ind];
				const float invStIsT_z = invStIsTz[ind];

				const PxReal jAccel = invStIsT_x * tJAccel[0] + invStIsT_y * tJAccel[1] + invStIsT_z * tJAccel[2];
				tDofs[ind].mJointAccel[threadIndexInWarp] = jAccel;

				const PxReal jVel = jointVel[ind] + jAccel * dt;

				tDofs[ind].mJointVelocities[threadIndexInWarp] = jVel;
				tDofs[ind].mJointUnconstrainedVelocities[threadIndexInWarp] = jVel;  

				motionAcceleration += axes[ind] * jAccel;
			}
		}

		storeSpatialVector(tLink.mMotionAcceleration, motionAcceleration, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector motionVel = linkMotionVel + motionAcceleration * dt;

		//Now store the pre-momentum stuff...

		const PxReal mass = tLink.mMass[threadIndexInWarp];

		preMomentum.top += motionVel.bottom * mass;
		//preMomentum.bottom += inertia * motionVel.top + offset.cross(motionVel.bottom)*mass;

		
		//Store motionVel in flat array (inefficient) - required by the constraint prep, which does not process block data!
		motionVelocities[linkID] = motionVel;

		storeSpatialVector(tLink.mMotionVelocity, motionVel, threadIndexInWarp);
	}

	const PxVec3 preRootVel = preMomentum.top * COM_invMassW.w;

	if (!fixBase)
	{

		Dy::SpatialMatrix invInertia;
		loadSpatialMatrix(articulationBlock.mInvSpatialArticulatedInertia, threadIndexInWarp, invInertia);
		const Cm::UnAlignedSpatialVector zaInt = loadSpatialVector(articulationLinks[0].mZAIntVector, threadIndexInWarp);

		const Cm::UnAlignedSpatialVector accel = -(invInertia * zaInt);
		storeSpatialVector(articulationLinks[0].mMotionAccelerationInternal,
			accel, threadIndexInWarp);
		addSpatialVector(articulationLinks[0].mMotionAcceleration, accel, threadIndexInWarp);
		Cm::UnAlignedSpatialVector motionVel = loadSpatialVector(articulationLinks[0].mMotionVelocity, threadIndexInWarp);

		PxMat33 inertia;
		loadPxMat33(articulationLinks[0].mIsolatedInertia, threadIndexInWarp, inertia);
		const PxReal mass = articulationLinks[0].mMass[threadIndexInWarp];

		const PxVec3 pos = loadPxVec3(articulationLinks[0].mAccumulatedPose.p, threadIndexInWarp);
		const PxVec3 offset = (pos - COM);

		preMomentum.bottom = inertia * motionVel.top + offset.cross(motionVel.bottom - preRootVel)*mass;

		compoundInertia += translateInertia(inertia, mass, offset);

		motionVel += accel * dt;

		postMomentum.top = motionVel.bottom * mass;
		storeSpatialVector(articulationLinks[0].mMotionVelocity, motionVel, threadIndexInWarp);
		motionVelocities[0] = motionVel;

	}
	for (PxU32 linkID = 1; linkID < numLinks; ++linkID)
	{
		PxgArticulationBlockLinkData& tLink = articulationLinks[linkID];
		PxgArticulationBlockDofData* PX_RESTRICT tDofs = &articulationDofs[tLink.mJointOffset[threadIndexInWarp]];

		const PxU32 parent = tLink.mParents[threadIndexInWarp];

		const float rwx = tLink.mRw_x[threadIndexInWarp];
		const float rwy = tLink.mRw_y[threadIndexInWarp];
		const float rwz = tLink.mRw_z[threadIndexInWarp];

		const Cm::UnAlignedSpatialVector pMotionAcceleration = FeatherstoneArticulation::translateSpatialVector(PxVec3(-rwx, -rwy, -rwz),
			loadSpatialVector(articulationLinks[parent].mMotionAccelerationInternal, threadIndexInWarp));

		Cm::UnAlignedSpatialVector motionAcceleration = pMotionAcceleration + loadSpatialVector(tLink.mCoriolis, threadIndexInWarp);
		//Cm::UnAlignedSpatialVector motionAccelerationInternal = pMotionAccelerationInt +loadSpatialVector(tLink.mCoriolis, threadIndexInWarp);

		const PxU32 dofs = tLink.mDofs[threadIndexInWarp];


		Cm::UnAlignedSpatialVector isWs[3];
		PxReal qstZ[3];
		PxReal jointVel[3];
		Cm::UnAlignedSpatialVector axes[3];
		float invStIsTx[3];
		float invStIsTy[3];
		float invStIsTz[3];

// the split into three separate loops is an optimization that allows dispatching the loads as early as possible.
#pragma unroll (3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			if (ind < dofs)
			{
				isWs[ind] = loadSpatialVector(tDofs[ind].mIsW, threadIndexInWarp);
				qstZ[ind] = tDofs[ind].mQstZIcInternal[threadIndexInWarp];
				jointVel[ind] = tDofs[ind].mJointVelocities[threadIndexInWarp];
				axes[ind] = loadSpatialVector(tDofs[ind].mWorldMotionMatrix, threadIndexInWarp);
				invStIsTx[ind] = tDofs[ind].mInvStIsT_x[threadIndexInWarp];
				invStIsTy[ind] = tDofs[ind].mInvStIsT_y[threadIndexInWarp];
				invStIsTz[ind] = tDofs[ind].mInvStIsT_z[threadIndexInWarp];
			}
		}


		PxReal tJAccel[3];
#pragma unroll (3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			PxReal jAccel = 0.f;
			if (ind < dofs)
			{
				jAccel = qstZ[ind] - isWs[ind].innerProduct(pMotionAcceleration);
			}
			tJAccel[ind] = jAccel;
		}

		//calculate jointAcceleration

#pragma unroll(3)
		for (PxU32 ind = 0; ind < 3; ++ind)
		{
			if (ind < dofs)
			{
				const float invStIsT_x = invStIsTx[ind];
				const float invStIsT_y = invStIsTy[ind];
				const float invStIsT_z = invStIsTz[ind];

				const PxReal jAccel = invStIsT_x * tJAccel[0] + invStIsT_y * tJAccel[1] + invStIsT_z * tJAccel[2];
				tDofs[ind].mJointAccel[threadIndexInWarp] += jAccel;

				const PxReal jVel = jointVel[ind] + (jAccel)* dt;
				
				tDofs[ind].mJointVelocities[threadIndexInWarp] = jVel;
				tDofs[ind].mJointUnconstrainedVelocities[threadIndexInWarp] = jVel;  

				motionAcceleration += axes[ind] * jAccel;
			}
		}

		storeSpatialVector(tLink.mMotionAccelerationInternal, motionAcceleration, threadIndexInWarp);

		const PxVec3 pos = loadPxVec3(tLink.mAccumulatedPose.p, threadIndexInWarp);
		Cm::UnAlignedSpatialVector motionVel = loadSpatialVector(tLink.mMotionVelocity, threadIndexInWarp);// +motionAcceleration * dt;

		//Now store the pre-momentum stuff...

		PxMat33 inertia;
		loadPxMat33(tLink.mIsolatedInertia, threadIndexInWarp, inertia);
		const PxReal mass = tLink.mMass[threadIndexInWarp];

		const PxVec3 offset = (pos - COM);

		compoundInertia += translateInertia(inertia, mass, offset);

		//Store pre-momentum before updating motion velocity
		preMomentum.bottom += inertia * motionVel.top + offset.cross(motionVel.bottom - preRootVel)*mass;


		motionVel += motionAcceleration * dt;

		postMomentum.top += motionVel.bottom * mass;

		//Store motionVel in flat array (inefficient) - required by the constraint prep, which does not process block data!
		motionVelocities[linkID] = motionVel;

		storeSpatialVector(tLink.mMotionVelocity, motionVel, threadIndexInWarp);
	}

	if (!fixBase)
	{
		const PxVec3 postRootVel = postMomentum.top * COM_invMassW.w;

		for (PxU32 linkID = 0; linkID < numLinks; ++linkID)
		{
			const PxgArticulationBlockLinkData& PX_RESTRICT tLink = articulationLinks[linkID];
			const Cm::UnAlignedSpatialVector motionVel = loadSpatialVector(tLink.mMotionVelocity, threadIndexInWarp);// +motionAcceleration * dt;

			PxMat33 inertia;
			loadPxMat33(tLink.mIsolatedInertia, threadIndexInWarp, inertia);
			const PxReal mass = tLink.mMass[threadIndexInWarp];

			const PxVec3 pos = loadPxVec3(tLink.mAccumulatedPose.p, threadIndexInWarp);
			const PxVec3 offset = (pos - COM);

			postMomentum.bottom += inertia * motionVel.top + offset.cross(motionVel.bottom - postRootVel)*mass;
		}

		const PxMat33 invCompoundInertia = compoundInertia.getInverse();

		const PxReal denom0 = postMomentum.bottom.magnitude();
		const PxReal num = preMomentum.bottom.magnitude();
		const PxReal angRatio = denom0 == 0.f ? 1.f : num / denom0;

		const PxVec3 deltaAngMom = postMomentum.bottom * (angRatio - 1.f);

		PxVec3 deltaAng = invCompoundInertia * deltaAngMom;

		const PxVec3 angVel = (invCompoundInertia * postMomentum.bottom) + deltaAng;

		//addSpatialVector(articulationLinks[0].mMotionVelocity, Cm::UnAlignedSpatialVector(deltaAng, PxVec3(0.f)), threadIndexInWarp);

		for (PxU32 linkID = 0; linkID < numLinks; ++linkID)
		{
			const PxVec3 offset = (loadPxVec3(articulationLinks[linkID].mAccumulatedPose.p, threadIndexInWarp) - COM);
			const Cm::UnAlignedSpatialVector velChange(deltaAng, offset.cross(deltaAng));
			addSpatialVector(articulationLinks[linkID].mMotionVelocity, velChange, threadIndexInWarp);
			postMomentum.top += velChange.bottom * articulationLinks[linkID].mMass[threadIndexInWarp];
		}

		const PxVec3 deltaLinMom = preMomentum.top - postMomentum.top;

		PxVec3 deltaLin = deltaLinMom * COM_invMassW.w;

		const PxVec3 linVel = (postMomentum.top*COM_invMassW.w) + deltaLin;

		for (PxU32 linkID = 0; linkID < numLinks; ++linkID)
		{
			Cm::UnAlignedSpatialVector vel = loadSpatialVector(articulationLinks[linkID].mMotionVelocity, threadIndexInWarp);
			vel.bottom += deltaLin;
			storeSpatialVector(articulationLinks[linkID].mMotionVelocity, vel, threadIndexInWarp);
			//storeSpatialVector(&motionVelocities[linkID], vel);
			motionVelocities[linkID] = vel;

		}

#if 0
		const bool validateMomentum = false;
		if (validateMomentum)
		{

			PxVec3 rootVel = postMomentum.top * COM_invMassW.w + deltaLin;

			Cm::SpatialVectorF momentum2(PxVec3(0.f), PxVec3(0.f));
			for (PxU32 linkID = 0; linkID < numLinks; ++linkID)
			{
				const PxReal mass = articulationLinks[linkID].mMass[threadIndexInWarp];
				PxVec3 offset = (loadPxVec3(articulationLinks[linkID].mAccumulatedPose.p, threadIndexInWarp) - COM);

				PxMat33 inertia;
				loadPxMat33(articulationLinks[linkID].mIsolatedInertia, threadIndexInWarp, inertia);

				Cm::UnAlignedSpatialVector vel = loadSpatialVector(articulationLinks[linkID].mMotionVelocity, threadIndexInWarp);

				const PxVec3 angMom = inertia * vel.top +
					offset.cross(vel.bottom - rootVel)*mass;
				momentum2.bottom += angMom;
				momentum2.top += vel.bottom * mass;
			}


			printf("Compute acceleration: linMom0 %f, linMom1 %f, angMom0 %f, angMom1 %f\n\n\n",
				preMomentum.top.magnitude(), momentum2.top.magnitude(),
				preMomentum.bottom.magnitude(), momentum2.bottom.magnitude());
		}
#endif
	}
}

static void __device__ computeJointTransmittedFrictionForce(PxgArticulationBlockLinkData* artiLinks, PxU32 nbLinks,
	const PxU32 threadIndexInWarp)
{
	//const PxU32 linkCount = data.getLinkCount();
	const PxU32 startIndex = nbLinks - 1;

	for (PxU32 linkID = startIndex; linkID > 0; --linkID)
	{
		PxgArticulationBlockLinkData& link = artiLinks[linkID];
		const PxU32 parent = link.mParents[threadIndexInWarp];
		const Cm::UnAlignedSpatialVector biasForce = loadSpatialVector(artiLinks[parent].mBiasForce, threadIndexInWarp);

		const float rwx = link.mRw_x[threadIndexInWarp];
		const float rwy = link.mRw_y[threadIndexInWarp];
		const float rwz = link.mRw_z[threadIndexInWarp];

		const float4 invInertiaXYZ_invMassW = link.mInvInertiaXYZ_invMassW[threadIndexInWarp];
		const float4 q = link.mAccumulatedPose.q[threadIndexInWarp];
		PxQuat rot(q.x, q.y, q.z, q.w);
		const PxReal m = invInertiaXYZ_invMassW.w == 0.f ? 0.f : 1.f/ invInertiaXYZ_invMassW.w;
		const PxVec3 inertia(invInertiaXYZ_invMassW.x == 0.f ? 0.f : 1.f / invInertiaXYZ_invMassW.x,
			invInertiaXYZ_invMassW.y == 0.f ? 0.f : 1.f / invInertiaXYZ_invMassW.y, 
			invInertiaXYZ_invMassW.z == 0.f ? 0.f : 1.f / invInertiaXYZ_invMassW.z);

		const Cm::UnAlignedSpatialVector motionAcceleration = loadSpatialVector(link.mMotionAcceleration, threadIndexInWarp)
			+ loadSpatialVector(link.mMotionAccelerationInternal, threadIndexInWarp);
		Cm::UnAlignedSpatialVector Ia = loadSpatialVector(link.mBiasForce, threadIndexInWarp);
		Ia.bottom += rot.rotate(rot.rotateInv(motionAcceleration.top).multiply(inertia));
		Ia.top += motionAcceleration.bottom * m;

		storeSpatialVector(link.mBiasForce, Ia, threadIndexInWarp);

		storeSpatialVector(artiLinks[parent].mBiasForce, biasForce + FeatherstoneArticulation::translateSpatialVector(PxVec3(rwx, rwy, rwz),
			Ia), threadIndexInWarp);
	}

	storeSpatialVector(artiLinks[0].mBiasForce, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
}

extern "C" __global__ void computeUnconstrainedAccelerationsLaunch1T(
	PxgArticulationCoreDesc* scDesc)
{
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.y;

	const PxU32 nbArticulations = scDesc->nbArticulations;


	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* linkData = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* dofData = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];

		const PxU32 numLinks = articulation.mNumLinks[threadIndexInWarp];

		//compute link accelerations and velocities
		computeLinkAcceleration(articulation, linkData, dofData, numLinks, scDesc->dt, threadIndexInWarp);

		computeJointTransmittedFrictionForce(linkData, numLinks, threadIndexInWarp);
	}
}

static PX_CUDA_CALLABLE PX_INLINE PxVec3 toRotationVector(const PxQuat& q)
{
	const float quatEpsilon = 1.0e-8f;
	const float s2 = q.x * q.x + q.y * q.y + q.z * q.z;
	PxReal angle;
	PxVec3 axis;
	if (s2 < quatEpsilon * quatEpsilon) // can't extract a sensible axis
	{
		angle = 0.0f;
		axis = PxVec3(1.0f, 0.0f, 0.0f);
	}
	else
	{
		const float s = PxRecipSqrt(s2);
		axis = PxVec3(q.x, q.y, q.z) * s;
		angle = PxAbs(q.w) < quatEpsilon ? PxPi : PxAtan2(s2 * s, q.w) * 2.0f;
	}

	return axis * angle;
}

static __device__ PxQuat computeSphericalJointPositions(const PxQuat& relativeQuat,
	const PxQuat& newRot, const PxQuat& pBody2WorldRot,
	PxgArticulationBlockDofData* dofData, const PxU32 dofs, const PxU32 threadIndexInWarp)
{
	PxQuat newParentToChild = (newRot.getConjugate() * pBody2WorldRot).getNormalized();

	if (newParentToChild.w < 0.f)
		newParentToChild = -newParentToChild;

	if (0)
	{
		PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();

		//PxVec3 axis = toRotationVector(jointRotation);
		PxVec3 axis = jointRotation.getImaginaryPart();

		for (PxU32 d = 0; d < dofs; ++d)
		{
			const float4 top = dofData[d].mLocalMotionMatrix.mTopxyz_bx[threadIndexInWarp];
			PxReal p = -compAng(PxVec3(top.x, top.y, top.z).dot(axis), jointRotation.w);
			dofData[d].mJointPositions[threadIndexInWarp] = p;
		}
	}
	else
	{
		PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();
		if(jointRotation.w < 0)
			jointRotation = -jointRotation;

		PxVec3 axis = toRotationVector(jointRotation);
		for (PxU32 d = 0; d < dofs; ++d)
		{
			const float4 top = dofData[d].mLocalMotionMatrix.mTopxyz_bx[threadIndexInWarp];
			PxReal ang = -PxVec3(top.x, top.y, top.z).dot(axis);
			dofData[d].mJointPositions[threadIndexInWarp] = ang;
		}
	}



	return newParentToChild;
}

static __device__ void computeAndEnforceJointPositions(
	PxgArticulationBlockLinkData* linkData,
	PxgArticulationBlockDofData* dofData, const PxU32 linkCount,
	const PxU32 threadIndexInWarp)
{
	using namespace Dy;

	for (PxU32 linkID = 1; linkID < linkCount; linkID ++)
	{
		PxgArticulationBlockLinkData& link = linkData[linkID];
		PxU8 jointType = link.mJointType[threadIndexInWarp];
		PxU8 jointOffset = link.mJointOffset[threadIndexInWarp];

		PxgArticulationBlockDofData* dof = &dofData[jointOffset];
		// We don't need to deal with eREVOLUTE_UNWRAPPED case because there are no joint position changed
		if (jointType == PxArticulationJointType::eREVOLUTE)
		{
			PxReal jPos = dof->mJointPositions[threadIndexInWarp];

			if (jPos > PxTwoPi)
				jPos -= PxTwoPi * 2.f;
			else if (jPos < -PxTwoPi)
				jPos += PxTwoPi * 2.f;

			jPos = PxClamp(jPos, -PxTwoPi * 2.f, PxTwoPi * 2.f);

			dof->mJointPositions[threadIndexInWarp] = jPos;
		}
		else if (jointType == PxArticulationJointType::ePRISMATIC)
		{
			if (dof->mMotion[threadIndexInWarp] == PxArticulationMotion::eLIMITED)
			{
				float2 limits_LowX_highY = dof->mConstraintData.mLimits_LowLimitX_highLimitY[threadIndexInWarp];
				PxReal jPosition = dof->mJointPositions[threadIndexInWarp];
				if (jPosition < limits_LowX_highY.x)
					jPosition = limits_LowX_highY.x;

				if (jPosition > limits_LowX_highY.y)
					jPosition = limits_LowX_highY.y;

				dof->mJointPositions[threadIndexInWarp] = jPosition;
			}
		}
		else if (jointType == PxArticulationJointType::eSPHERICAL)
		{

			const PxU32 parent = link.mParents[threadIndexInWarp];
			const float4 pBody2WorldRot = linkData[parent].mAccumulatedPose.q[threadIndexInWarp];

			const float4 newRot = link.mAccumulatedPose.q[threadIndexInWarp];
			const float4 relativeQuat = link.mRelativeQuat[threadIndexInWarp];

			PxQuat newQ(newRot.x, newRot.y, newRot.z, newRot.w);

			computeSphericalJointPositions(PxQuat(relativeQuat.x, relativeQuat.y, relativeQuat.z, relativeQuat.w),
				newQ, PxQuat(pBody2WorldRot.x, pBody2WorldRot.y, pBody2WorldRot.z, pBody2WorldRot.w),
				dof, link.mDofs[threadIndexInWarp], threadIndexInWarp);

		}
	}
}

static __device__ PX_FORCE_INLINE PxTransform updateRootBody(const Cm::UnAlignedSpatialVector& motionVelocity,
	const PxTransform& preTransform,
	const PxReal dt,
	PxU32 threadIndexInWarp)
{
	//PxTransform body2World = preTransform;

	//(1) project the current body's velocity (based on its pre-pose) to the geometric COM that we're integrating around...

	const PxVec3 comLinVel = motionVelocity.bottom;

	//using the position iteration motion velocity to compute the body2World
	const PxVec3 newP = (preTransform.p) + comLinVel * dt;

	const PxQuat deltaQ = PxExp(motionVelocity.top*dt);

	return PxTransform(newP, (deltaQ* preTransform.q).getNormalized());
}

template <bool isSubstep>
static __device__ void propagateLink(PxTransform& PX_RESTRICT body2World, const PxgArticulationBlockLinkData& PX_RESTRICT link,
	const PxgArticulationBlockLinkData* PX_RESTRICT links, 
	PxgArticulationBlockDofData* PX_RESTRICT dofs, PxReal dt, const PxU32 threadIndexInWarp)
{
	const PxU32 jointOffset = link.mJointOffset[threadIndexInWarp];
	const PxU32 jointType = link.mJointType[threadIndexInWarp];
	const PxU32 parent = link.mParents[threadIndexInWarp];
	const PxgArticulationBlockLinkData& pLink = links[parent];

	PxgArticulationBlockDofData* PX_RESTRICT dof = dofs + jointOffset;

	const PxTransform pBody2World = loadSpatialTransform(pLink.mAccumulatedPose, threadIndexInWarp);
	
	PxQuat newParentToChild;

	const float4 co = link.mChildPose.p[threadIndexInWarp];
	const float4 po = link.mParentPose.p[threadIndexInWarp];
	const float4 rq = link.mRelativeQuat[threadIndexInWarp];

	const PxVec3 childOffset(-co.x, -co.y, -co.z);
	const PxVec3 parentOffset(po.x, po.y, po.z);
	const PxQuat relativeQuat(rq.x, rq.y, rq.z, rq.w);

	PxVec3 offset(0.f);
	switch (jointType)
	{
	case PxArticulationJointType::ePRISMATIC:
	{
		const PxReal delta = (isSubstep ? dof[0].mJointVelocities[threadIndexInWarp] : dof[0].mPosJointVelocities[threadIndexInWarp]) * dt;

		const PxReal pos = dof[0].mJointPositions[threadIndexInWarp] + delta;
		dof[0].mJointPositions[threadIndexInWarp] = pos;

		//KS - TODO - requires some plumbing!
		//enforcePrismaticLimits(jPosition, joint);

		newParentToChild = relativeQuat;
		const Cm::UnAlignedSpatialVector motionMatrix = loadSpatialVector(dof[0].mLocalMotionMatrix, threadIndexInWarp);
		offset = motionMatrix.bottom*pos;
		break;
	}
	case PxArticulationJointType::eREVOLUTE:
	{
		//use positional iteration JointVelociy to integrate
		const PxReal delta = (isSubstep ? dof[0].mJointVelocities[threadIndexInWarp] : dof[0].mPosJointVelocities[threadIndexInWarp]) * dt;

		PxReal jPos = dof[0].mJointPositions[threadIndexInWarp] + delta;

		if (jPos > PxTwoPi)
			jPos -= PxTwoPi * 2.f;
		else if (jPos < -PxTwoPi)
			jPos += PxTwoPi * 2.f;

		jPos = PxClamp(jPos, -PxTwoPi * 2.f, PxTwoPi * 2.f);
		dof[0].mJointPositions[threadIndexInWarp] = jPos;

		const float4 u = dof[0].mLocalMotionMatrix.mTopxyz_bx[threadIndexInWarp];

		PxQuat jointRotation = PxQuat(-jPos, PxVec3(u.x, u.y, u.z));
		if (jointRotation.w < 0)	//shortest angle.
			jointRotation = -jointRotation;

		newParentToChild = (jointRotation * relativeQuat).getNormalized();

		break;
	}
	case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
	{
		//use positional iteration JointVelociy to integrate
		const PxReal delta = (isSubstep ? dof[0].mJointVelocities[threadIndexInWarp] : dof[0].mPosJointVelocities[threadIndexInWarp]) * dt;

		const PxReal jPos = dof[0].mJointPositions[threadIndexInWarp] + delta;

		dof[0].mJointPositions[threadIndexInWarp] = jPos;

		const float4 u = dof[0].mLocalMotionMatrix.mTopxyz_bx[threadIndexInWarp];

		PxQuat jointRotation = PxQuat(-jPos, PxVec3(u.x, u.y, u.z));
		if (jointRotation.w < 0)	//shortest angle.
			jointRotation = -jointRotation;

		newParentToChild = (jointRotation * relativeQuat).getNormalized();

		break;
	}
	case PxArticulationJointType::eSPHERICAL:
	{
		const PxU32 nbDofs = link.mDofs[threadIndexInWarp];
		{
			const PxTransform oldTransform = loadSpatialTransform(link.mAccumulatedPose, threadIndexInWarp);
			
			Cm::UnAlignedSpatialVector worldVel = isSubstep ? loadSpatialVector(link.mMotionVelocity, threadIndexInWarp) :
				loadSpatialVector(link.mPosMotionVelocity, threadIndexInWarp);

			PxVec3 worldAngVel = worldVel.top;

			const PxReal dist = worldAngVel.normalize() * dt;

			PxQuat newWorldQ;
			if (dist > 1e-6f)
				newWorldQ = PxQuat(dist, worldAngVel) * oldTransform.q;
			else
				newWorldQ = oldTransform.q;

			/*PxReal jPos[3];
			jPos[0] = jPosition[0] + (jVelocity[0] + jDeltaVelocity[0])*dt;
			jPos[1] = jPosition[1] + (jVelocity[1] + jDeltaVelocity[1])*dt;
			jPos[2] = jPosition[2] + (jVelocity[2] + jDeltaVelocity[2])*dt;*/

			newParentToChild = computeSphericalJointPositions(relativeQuat, newWorldQ, pBody2World.q, dof, nbDofs, threadIndexInWarp);
		}

		break;
	}
	case PxArticulationJointType::eFIX:
	{
		//this is fix joint so joint don't have velocity
		newParentToChild = relativeQuat;
		break;
	}
	default:
		break;
	}

	const PxVec3 e = newParentToChild.rotate(parentOffset);
	const PxVec3 d = childOffset;
	const PxVec3 r = e + d + offset;

	assert(newParentToChild.isSane());
	assert(newParentToChild.isFinite());

	body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
	body2World.p = pBody2World.p + body2World.q.rotate(r);

	assert(body2World.isSane());
	assert(body2World.isValid());
}


static void __device__ propagateLinksDown(const PxgArticulationBlockData& data, PxgArticulationBlockLinkData* artiLinks,
	PxgArticulationBlockDofData* artiDofs, const PxU32 linkCount, const PxReal dt, const PxU32 threadIndexInWarp)
{
	for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
	{
		PxgArticulationBlockLinkData& link = artiLinks[linkID];

		PxTransform body2World;
		propagateLink<false>(body2World, link, artiLinks, artiDofs, dt, threadIndexInWarp);

		storeSpatialTransform(link.mAccumulatedPose, threadIndexInWarp, body2World);
	}
}


static __device__ void updatePoses(const PxgArticulationBlockData& artiData, PxgArticulationBlockLinkData* artiLinks,
	PxgArticulationBlockDofData* artiDofs, const PxU32 linkCount, const bool fixBase,
	const PxReal dt, PxU32 threadIndexInWarp)
{
	if (!fixBase)
	{
		const Cm::UnAlignedSpatialVector posVel = loadSpatialVector(artiLinks[0].mPosMotionVelocity, threadIndexInWarp);
		const PxTransform preTrans = loadSpatialTransform(artiLinks[0].mAccumulatedPose, threadIndexInWarp);

		const PxTransform trans = updateRootBody(posVel, preTrans, dt, threadIndexInWarp);

		storeSpatialTransform(artiLinks[0].mAccumulatedPose, threadIndexInWarp, trans);
	}

	//using the original joint velocities and delta velocities changed in the positional iter to update joint position/body transform
	propagateLinksDown(artiData, artiLinks, artiDofs, linkCount, dt, threadIndexInWarp);
}

static __device__ PxReal sleepCheck1T(
	PxgArticulationLinkSleepData& sleepData,
	PxReal& wakeCounter,
	const PxTransform& body2World,
	const PxVec3& linearMotionVel,
	const PxVec3& angularMotionVel,
	const float dt,
	const float sleepThreshold,
	const float4 inverseInertiaXYZ_invMassW
)
{
	const PxReal wakeCounterResetTime = 20.0f*0.02f;

	float4 sleepLinVelAccXYZ = sleepData.sleepLinVelAccXYZ;
	float4 sleepAngVelAccXYZ = sleepData.sleepAngVelAccXYZ;

	PxReal wc = wakeCounter;

	bool alreadyUpdateWC = false;
	PxVec3 sleepLinVelAcc(0.f), sleepAngVelAcc(0.f);

	if (wc < wakeCounterResetTime * 0.5f || wc < dt)
	{
		// calculate normalized energy: kinetic energy divided by mass
		const PxVec3 inertia(inverseInertiaXYZ_invMassW.x > 0.f ? 1.0f / inverseInertiaXYZ_invMassW.x : 1.f, inverseInertiaXYZ_invMassW.y > 0.f ? 1.0f / inverseInertiaXYZ_invMassW.y : 1.f, inverseInertiaXYZ_invMassW.z > 0.f ? 1.0f / inverseInertiaXYZ_invMassW.z : 1.f);

		sleepLinVelAcc = linearMotionVel;// originalBody->mAcceleration.linear;
		sleepAngVelAcc = body2World.q.rotateInv(angularMotionVel);// originalBody->mAcceleration.angular;

		sleepLinVelAcc.x += sleepLinVelAccXYZ.x;
		sleepLinVelAcc.y += sleepLinVelAccXYZ.y;
		sleepLinVelAcc.z += sleepLinVelAccXYZ.z;

		sleepAngVelAcc.x += sleepAngVelAccXYZ.x;
		sleepAngVelAcc.y += sleepAngVelAccXYZ.y;
		sleepAngVelAcc.z += sleepAngVelAccXYZ.z;


		PxReal invMass = inverseInertiaXYZ_invMassW.w;
		if (invMass == 0.f)
			invMass = 1.f;

		const PxReal angular = sleepAngVelAcc.multiply(sleepAngVelAcc).dot(inertia) * invMass;
		const PxReal linear = sleepLinVelAcc.magnitudeSquared();
		PxReal normalizedEnergy = 0.5f * (angular + linear);

		if (normalizedEnergy >= sleepThreshold)
		{
			//assert(isActive());
			sleepLinVelAcc = PxVec3(0);
			sleepAngVelAcc = PxVec3(0);
			const float factor = sleepThreshold == 0.f ? 2.0f : PxMin(normalizedEnergy / sleepThreshold, 2.0f);
			//PxReal oldWc = wc;
			wc = factor * 0.5f * wakeCounterResetTime;

			alreadyUpdateWC = true;
		}
	}

	if (!alreadyUpdateWC)
		wc = PxMax(wc - dt, 0.0f);

	bool wakeCounterZero = (wc == 0.0f);

	if (wakeCounterZero)
	{
		sleepLinVelAcc = PxVec3(0);
		sleepAngVelAcc = PxVec3(0);
	}

	wakeCounter = wc;
	sleepData.sleepLinVelAccXYZ = make_float4(sleepLinVelAcc.x, sleepLinVelAcc.y, sleepLinVelAcc.z, 0.f);
	sleepData.sleepAngVelAccXYZ = make_float4(sleepAngVelAcc.x, sleepAngVelAcc.y, sleepAngVelAcc.z, 0.f);

	return wc;
}

template<bool doIntegrate>
static void __device__ conserveMomentum(PxgArticulationBlockData& articulation,
	PxgArticulationBlockLinkData* artiLinks,
	PxgArticulationBlockDofData* artiDofs,
	PxU32 linkCount, bool fixBase, const PxU32 threadIndexInWarp,
	const PxReal dt)
{
	if (!fixBase)
	{
		const float4 COM_invMassW = articulation.mCOM_TotalInvMassW[threadIndexInWarp];
		const PxVec3 preCOM(COM_invMassW.x, COM_invMassW.y, COM_invMassW.z);
		PxVec3 postCOM(0.f);

		Cm::UnAlignedSpatialVector preMomentum(PxVec3(0.f), PxVec3(0.f));
		Cm::UnAlignedSpatialVector postMomentum(PxVec3(0.f), PxVec3(0.f));
		PxVec3 posMomentum(0.f);

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const PxReal mass = artiLinks[linkID].mMass[threadIndexInWarp];
			preMomentum.top += loadSpatialVector(artiLinks[linkID].mMotionVelocity, threadIndexInWarp).bottom * mass;
			posMomentum += loadSpatialVector(artiLinks[linkID].mPosMotionVelocity, threadIndexInWarp).bottom * mass;
		}

		
		PxReal mass = artiLinks[0].mMass[threadIndexInWarp];
		PxVec3 comPreLinVel = preMomentum.top * COM_invMassW.w;
		PxMat33 inertia;
		loadPxMat33(artiLinks[0].mIsolatedInertia, threadIndexInWarp, inertia);
		Cm::UnAlignedSpatialVector vel = loadSpatialVector(artiLinks[0].mMotionVelocity, threadIndexInWarp);
		preMomentum.bottom = inertia * vel.top
			+ (loadPxVec3(artiLinks[0].mPreTransform.p, threadIndexInWarp) - preCOM).cross(vel.bottom - comPreLinVel)*mass;

		postMomentum.top = vel.bottom * mass;

		postCOM += loadPxVec3(artiLinks[0].mAccumulatedPose.p, threadIndexInWarp) * mass;

		// PT: preload next link data
		PxU32 nextParent = artiLinks[1].mParents[threadIndexInWarp];

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			// PT: preload next link data
			const PxU32 parent = nextParent;
			if(linkID!=linkCount-1)
			{
				nextParent = artiLinks[linkID+1].mParents[threadIndexInWarp];
			}

			//Propagate velocity down...
			PxTransform childPose = loadSpatialTransform(artiLinks[linkID].mAccumulatedPose, threadIndexInWarp);
			PxVec3 parentPos = loadPxVec3(artiLinks[parent].mAccumulatedPose.p, threadIndexInWarp);
			mass = artiLinks[linkID].mMass[threadIndexInWarp];

			PxVec3 rw = childPose.p - parentPos;

			PxU32 dofs = artiLinks[linkID].mDofs[threadIndexInWarp];
			PxU32 jointOffset = artiLinks[linkID].mJointOffset[threadIndexInWarp];

			loadPxMat33(artiLinks[linkID].mIsolatedInertia, threadIndexInWarp, inertia);
			Cm::UnAlignedSpatialVector vel = loadSpatialVector(artiLinks[linkID].mMotionVelocity, threadIndexInWarp);
			preMomentum.bottom += inertia * vel.top
				+ (loadPxVec3(artiLinks[linkID].mPreTransform.p, threadIndexInWarp) - preCOM).cross(vel.bottom - comPreLinVel)*mass;


			vel = FeatherstoneArticulation::translateSpatialVector(-rw, loadSpatialVector(artiLinks[parent].mMotionVelocity, threadIndexInWarp));

			assert(dofs<=3);
			for (PxU32 ind = 0; ind < 3; ++ind)
			{
				if(ind<dofs)
				{
					PxReal jVel = artiDofs[jointOffset + ind].mJointVelocities[threadIndexInWarp];
					//deltaV += data.mWorldMotionMatrix[jointDatum.jointOffset + ind] * jVel;
					vel += loadSpatialVector(artiDofs[jointOffset + ind].mLocalMotionMatrix, threadIndexInWarp).rotate(childPose) * jVel;
				}
			}

			postCOM += childPose.p * mass;
			storeSpatialVector(artiLinks[linkID].mMotionVelocity, vel, threadIndexInWarp);

			postMomentum.top += vel.bottom * mass;

		}
		postCOM *= COM_invMassW.w;
		PxVec3 comPostLinVel = postMomentum.top * COM_invMassW.w;

		PxMat33 compoundInertia(PxZero);

		// PT: preload next link data
		float4 nextInvInertiaDiag = artiLinks[0].mInvInertiaXYZ_invMassW[threadIndexInWarp];
		PxReal nextMass = artiLinks[0].mMass[threadIndexInWarp];
		PxTransform nextPose = loadSpatialTransform(artiLinks[0].mAccumulatedPose, threadIndexInWarp);

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const Cm::UnAlignedSpatialVector vel = loadSpatialVector(artiLinks[linkID].mMotionVelocity, threadIndexInWarp);

			// PT: preload next link data
			const PxVec3 inertiaDiag(1.f / nextInvInertiaDiag.x, 1.f / nextInvInertiaDiag.y, 1.f / nextInvInertiaDiag.z);
			const PxReal mass = nextMass;
			const PxTransform pose = nextPose;
			if(linkID!=linkCount-1)
			{
				nextInvInertiaDiag = artiLinks[linkID+1].mInvInertiaXYZ_invMassW[threadIndexInWarp];
				nextMass = artiLinks[linkID+1].mMass[threadIndexInWarp];
				nextPose = loadSpatialTransform(artiLinks[linkID+1].mAccumulatedPose, threadIndexInWarp);
			}

			const PxVec3 offset = pose.p - postCOM;
			PxMat33 R(pose.q);
			const PxVec3 offsetMass = offset * mass;

			PxMat33 inertia;
			Cm::transformInertiaTensor(inertiaDiag, R, inertia);

			compoundInertia += translateInertia(inertia, mass, offset);

			postMomentum.bottom += inertia * vel.top + offset.cross(vel.bottom - comPostLinVel) * mass;

			//storePxMat33(artiLinks[linkID].mIsolatedInertia, threadIndexInWarp, inertia);
		}

		const PxMat33 invCompoundInertia = compoundInertia.getInverse();
		
		const PxReal aDenom = postMomentum.bottom.magnitude();
		const PxReal angRatio = aDenom == 0.f ? 0.f : preMomentum.bottom.magnitude() / aDenom;
		const PxVec3 angMomDelta = postMomentum.bottom * (angRatio - 1.f);

		const PxVec3 angDelta = invCompoundInertia * angMomDelta;

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const Cm::UnAlignedSpatialVector vel = loadSpatialVector(artiLinks[linkID].mMotionVelocity, threadIndexInWarp);
			const PxReal mass = artiLinks[linkID].mMass[threadIndexInWarp];

			const PxVec3 offset = (loadPxVec3(artiLinks[linkID].mAccumulatedPose.p, threadIndexInWarp) - postCOM);
			const Cm::UnAlignedSpatialVector velChange(angDelta, -offset.cross(angDelta));
			storeSpatialVector(artiLinks[linkID].mMotionVelocity, vel + velChange, threadIndexInWarp);
			postMomentum.top += velChange.bottom * mass;
		}

		const Cm::UnAlignedSpatialVector delta(PxVec3(0.f), (preMomentum.top - postMomentum.top)*COM_invMassW.w);

		PxVec3 predictedCOM;
		if (doIntegrate)
		{
			predictedCOM = preCOM + posMomentum * COM_invMassW.w * dt;
		}
		else
		{
			//If we are doing TGS, posMomentum stores the 
			predictedCOM = preCOM + posMomentum * COM_invMassW.w;
		}

		PxVec3 correction = predictedCOM - postCOM;

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			addSpatialVector(artiLinks[linkID].mMotionVelocity, delta, threadIndexInWarp);
			artiLinks[linkID].mAccumulatedPose.p[threadIndexInWarp] += make_float4(correction.x, correction.y, correction.z, 0.f);
		}

		postCOM += correction;


#if 0
		const bool validateMomentum = true;
		if (validateMomentum)
		{

			PxVec3 rootVel = postMomentum.top * COM_invMassW.w + delta.bottom;

			Cm::SpatialVectorF momentum2(PxVec3(0.f), PxVec3(0.f));
			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				const PxReal mass = artiLinks[linkID].mMass[threadIndexInWarp];
				PxVec3 offset = (loadPxVec3(artiLinks[linkID].mAccumulatedPose.p, threadIndexInWarp) - postCOM);

				loadPxMat33(artiLinks[linkID].mIsolatedInertia, threadIndexInWarp, inertia);

				vel = loadSpatialVector(artiLinks[linkID].mMotionVelocity, threadIndexInWarp);

				const PxVec3 angMom = inertia * vel.top +
					offset.cross(vel.bottom - rootVel)*mass;
				momentum2.bottom += angMom;
				momentum2.top += vel .bottom * mass;
			}


			printf("preCOM = (%f, %f, %f), COM = (%f, %f, %f)\n", preCOM.x, preCOM.y, preCOM.z, postCOM.x, postCOM.y, postCOM.z);
			printf("linMom0 %f, linMom1 %f, angMom0 %f, angMom1 %f\n\n\n",
				preMomentum.top.magnitude(), momentum2.top.magnitude(),
				preMomentum.bottom.magnitude(), momentum2.bottom.magnitude());
		}
#endif

	}
}

template<bool integrateJointPositions>
static __device__ void updateBodiesInternal(
	PxgArticulationBlockData& articulationBlock,
	PxgArticulationBlockLinkData* linkData,
	PxgArticulationBlockDofData* dofData,
	const PxReal dt,
	const PxU32 threadIndexInWarp,
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	const PxU32 maxLinks,
	const PxU32 maxDofs,
	const PxU32 globalThreadIndex,
	const PxU32 nbArticulations)
{
	const PxU32 linkCount = articulationBlock.mNumLinks[threadIndexInWarp];

	const PxU32 flags = articulationBlock.mFlags[threadIndexInWarp];

	const bool fixBase = flags & PxArticulationFlag::eFIX_BASE;

	const PxReal invDt = 1.f / dt;

	if (!integrateJointPositions)
	{
		computeAndEnforceJointPositions(linkData, dofData, linkCount, threadIndexInWarp);
	}
	else
	{
		updatePoses(articulationBlock, linkData, dofData, linkCount, fixBase, dt, threadIndexInWarp);
	}


	//update joint velocities/accelerations due to contacts/constraints
	PxU8 stateDirty = articulationBlock.mStateDirty[threadIndexInWarp];
	if (stateDirty)
	{
		if (stateDirty & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
		{
			averageLinkImpulsesAndPropagate(scDesc->slabHasChanges, scDesc->impulses, articulationBlock, linkData, dofData, globalThreadIndex, maxLinks,
				nbArticulations, scDesc->nbSlabs, linkCount, threadIndexInWarp);
		}
		PxcFsFlushVelocity(articulationBlock, linkData, dofData, linkCount, fixBase, threadIndexInWarp);

		articulationBlock.mStateDirty[threadIndexInWarp] = 0;
	}

	conserveMomentum<integrateJointPositions>(articulationBlock, linkData, dofData, linkCount, fixBase, threadIndexInWarp, dt);

	const PxU32 totalDofs = articulationBlock.mTotalDofs[threadIndexInWarp];
	const PxU32 articulationIndex = articulationBlock.mArticulationIndex[threadIndexInWarp];

	const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

	PxReal* jointP = articulation.jointPositions;
	PxReal* jointV = articulation.jointVelocities;
	PxReal* jointA = articulation.jointAccelerations;
	for (PxU32 i = 0; i < totalDofs; ++i)
	{
		jointP[i] = dofData[i].mJointPositions[threadIndexInWarp];
		jointV[i] = dofData[i].mJointVelocities[threadIndexInWarp];
		jointA[i] = dofData[i].mJointAccel[threadIndexInWarp] + 
			(dofData[i].mJointVelocities[threadIndexInWarp] - dofData[i].mJointUnconstrainedVelocities[threadIndexInWarp])*invDt;
	}
}

extern "C" __global__ void updateBodiesLaunch1T(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	PxReal dt, bool integrate)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* linkData = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* dofData = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];

		const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

		if(integrate)
			updateBodiesInternal<true>(articulation, linkData, dofData, dt, threadIndexInWarp, scDesc,
				maxLinks, maxDofs, globalThreadIndex, nbArticulations);
		else
			updateBodiesInternal<false>(articulation, linkData, dofData, dt, threadIndexInWarp, scDesc,
				maxLinks, maxDofs, globalThreadIndex, nbArticulations);
	}
}

extern "C" __global__ void updateBodiesLaunch_Part2(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	PxReal dt, bool integrate)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;
	const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	// PT: generally speaking the launch params are the same as for updateBodiesLaunch1T, except we use the Z dimension
	// to execute this part "per link" rather than "per articulation". We use 4 threads per block on Z, see the details
	// in PxgArticulationCore::updateBodies().
	const PxU32 linkIndex = threadIdx.z + blockIdx.z * 4;

	if (globalThreadIndex < nbArticulations && linkIndex<maxLinks)
	{
		PxgArticulationBlockData& articulationBlock = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* linkData = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];

		const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

		const PxU32 linkCount = articulationBlock.mNumLinks[threadIndexInWarp];

		if(linkIndex<linkCount)
		{
			const PxReal invDt = 1.f / dt;

			const PxU32 articulationIndex = articulationBlock.mArticulationIndex[threadIndexInWarp];

			const PxgArticulation& articulation = scDesc->articulations[articulationIndex];
			PxTransform* body2World = articulation.linkBody2Worlds;
			Cm::UnAlignedSpatialVector* motionVelocities = articulation.motionVelocities;

			const PxReal sleepThreshold = articulationBlock.mSleepThreshold[threadIndexInWarp];

			PxgArticulationLink* links = articulation.links;

			PxReal* gLinkWakeCounters = articulation.linkWakeCounters;
			PxgArticulationLinkSleepData* gSleepData = articulation.linkSleepData;

			// Compute the link incoming joint force
			Cm::UnAlignedSpatialVector* linkAccelerations = articulation.motionAccelerations;
			Cm::UnAlignedSpatialVector* linkIncomingJointForces = articulation.linkIncomingJointForces;

			if(linkIndex==0)
			{	
				const Cm::SpatialVectorF linkMotionAccelerationW = loadSpatialVectorF(linkData[0].mMotionAcceleration, threadIndexInWarp);
				const Cm::SpatialVectorF linkSpatialDeltaVelW = loadSpatialVectorF(linkData[0].mSolverSpatialDeltaVel, threadIndexInWarp);

				//Compute the acceleration
				const Cm::SpatialVectorF accelerationW = linkMotionAccelerationW + linkSpatialDeltaVelW*invDt;

				//Store the link accelerations.
				linkAccelerations[0].top = accelerationW.top;
				linkAccelerations[0].bottom = accelerationW.bottom;

				linkIncomingJointForces[0].top = PxVec3(PxZero);
				linkIncomingJointForces[0].bottom = PxVec3(PxZero);
			}
			else
			{	
				const Cm::SpatialVectorF linkZAForceExtW = loadSpatialVectorF(linkData[linkIndex].mZAVector, threadIndexInWarp);
				const Cm::SpatialVectorF linkZAForceIntW = loadSpatialVectorF(linkData[linkIndex].mZAIntVector, threadIndexInWarp);
				const Cm::SpatialVectorF linkMotionAccelerationExtW = loadSpatialVectorF(linkData[linkIndex].mMotionAcceleration, threadIndexInWarp);
				const Cm::SpatialVectorF linkMotionAccelerationIntW = loadSpatialVectorF(linkData[linkIndex].mMotionAccelerationInternal, threadIndexInWarp);
				SpatialMatrix linkSpatialInertiaW;
				loadSpatialMatrix(linkData[linkIndex].mSpatialArticulatedInertia, threadIndexInWarp, linkSpatialInertiaW);
				const Cm::SpatialVectorF linkSpatialDeltaVelW = loadSpatialVectorF(linkData[linkIndex].mSolverSpatialDeltaVel, threadIndexInWarp);
				const Cm::SpatialVectorF linkSpatialImpulseW = loadSpatialVectorF(linkData[linkIndex].mSolverSpatialImpulse, threadIndexInWarp);
				const PxTransform Gc = loadSpatialTransform(linkData[linkIndex].mAccumulatedPose, threadIndexInWarp);		
				const PxTransform Lc = loadSpatialTransform(linkData[linkIndex].mChildPose, threadIndexInWarp);
				const PxTransform GcLc = Gc*Lc;
				const PxVec3 dW = Gc.rotate(Lc.p);

				//Compute the acceleration
				const Cm::SpatialVectorF accelerationW = linkMotionAccelerationExtW + linkMotionAccelerationIntW + linkSpatialDeltaVelW*invDt;

				//Compute the force measured at the link.
				Cm::SpatialVectorF incomingJointForceW =
					linkSpatialInertiaW*accelerationW + 
					(linkZAForceExtW + linkZAForceIntW + linkSpatialImpulseW*invDt);	// PT: at link

				//Compute the equivalent force measured at the joint.
				translateSpatialVectorInPlace(-dW, incomingJointForceW);	// PT: now at joint

				//Store the link accelerations.
				linkAccelerations[linkIndex].top = accelerationW.top;
				linkAccelerations[linkIndex].bottom = accelerationW.bottom;

				//Transform the force to the child joint frame.
				linkIncomingJointForces[linkIndex].top =  GcLc.rotateInv(incomingJointForceW.top);
				linkIncomingJointForces[linkIndex].bottom =  GcLc.rotateInv(incomingJointForceW.bottom);
			}

			PxgArticulationLink& link = links[linkIndex];
			PxgArticulationBlockLinkData& blockLinkData = linkData[linkIndex];

			const Cm::UnAlignedSpatialVector motionV = loadSpatialVector(blockLinkData.mMotionVelocity, threadIndexInWarp);
			const float initialLinVelXYZ_invMassW = link.initialLinVelXYZ_invMassW.w;
			const float initialAngVelXYZ_penBiasClamp = link.initialAngVelXYZ_penBiasClamp.w;
			PxReal lwc = gLinkWakeCounters[linkIndex];
			const PxTransform accumulatedPose = loadSpatialTransform(blockLinkData.mAccumulatedPose, threadIndexInWarp);

			Cm::UnAlignedSpatialVector posMotionV = loadSpatialVector(blockLinkData.mPosMotionVelocity, threadIndexInWarp);
			const float4 inverseInertiaXYZ_invMass = blockLinkData.mInvInertiaXYZ_invMassW[threadIndexInWarp];

			motionVelocities[linkIndex] = motionV;
			link.initialLinVelXYZ_invMassW = make_float4(motionV.bottom.x, motionV.bottom.y, motionV.bottom.z, initialLinVelXYZ_invMassW);
			link.initialAngVelXYZ_penBiasClamp = make_float4(motionV.top.x, motionV.top.y, motionV.top.z, initialAngVelXYZ_penBiasClamp);

			body2World[linkIndex] = accumulatedPose;

			if (!integrate)
				posMotionV *= invDt;

			//each thread produce a wc
			sleepCheck1T(gSleepData[linkIndex], lwc, accumulatedPose,
				posMotionV.bottom, posMotionV.top, dt, sleepThreshold,
				inverseInertiaXYZ_invMass);

			gLinkWakeCounters[linkIndex] = lwc;
		}
	}
}


extern "C" __global__ void updateBodiesLaunch_Part3(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	PxReal dt, bool integrate)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulationBlockData& articulationBlock = scDesc->mArticulationBlocks[globalWarpIndex];

		const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

		const PxU32 linkCount = articulationBlock.mNumLinks[threadIndexInWarp];

		const PxU32 articulationIndex = articulationBlock.mArticulationIndex[threadIndexInWarp];

		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		const PxReal* gLinkWakeCounters = articulation.linkWakeCounters;

		PxgSolverBodySleepData artiSleepData = scDesc->articulationSleepData[articulationIndex];
		const PxReal oldWc = artiSleepData.wakeCounter;
		PxReal maxWc = 0.f;

		for (PxU32 linkID = 0; linkID < linkCount; linkID++)
		{
			PxReal lwc = gLinkWakeCounters[linkID];
			maxWc = PxMax(lwc, maxWc);
		}

		PxU32 newFlags = artiSleepData.internalFlags & (~(PxsRigidBody::eACTIVATE_THIS_FRAME | PxsRigidBody::eDEACTIVATE_THIS_FRAME));
		if (oldWc == 0.0f && maxWc != 0.f)
			newFlags |= PxsRigidBody::eACTIVATE_THIS_FRAME;
		else if (maxWc == 0.f && oldWc != 0.f)
			newFlags |= PxsRigidBody::eDEACTIVATE_THIS_FRAME;

		artiSleepData.internalFlags = newFlags;
		artiSleepData.wakeCounter = maxWc;
		scDesc->articulationSleepData[articulationIndex] = artiSleepData;
	}
}




// PGS only
extern "C" __global__ void artiSaveVelocity1TPGS(PxgArticulationCoreDesc* scDesc)
{
	//shared memory shared within a block
	const PxU32 nbArticulations = scDesc->nbArticulations;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = threadIdx.y;
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

		

		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];

		const PxU32 numLinks = articulation.mNumLinks[threadIndexInWarp];

		const PxU32 totalDofs = articulation.mTotalDofs[threadIndexInWarp];

		const bool fixBase = articulation.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;
		PxU32 stateDirty = articulation.mStateDirty[threadIndexInWarp];
		if (stateDirty)
		{
			if (stateDirty & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
			{
				averageLinkImpulsesAndPropagate(scDesc->slabHasChanges, scDesc->impulses, articulation, artiLinks, artiDofs, globalThreadIndex, maxLinks,
					nbArticulations, scDesc->nbSlabs, numLinks, threadIndexInWarp);
			}
			
			PxcFsFlushVelocity(articulation, artiLinks, artiDofs, numLinks, fixBase, threadIndexInWarp);
			articulation.mStateDirty[threadIndexInWarp] = 0;
		}
		
		//copy back motion velocities and joint delta velocities
		for (PxU32 linkID = 0; linkID < numLinks; ++linkID)
		{
			artiLinks[linkID].mPosMotionVelocity.mTopxyz_bx[threadIndexInWarp] =
				artiLinks[linkID].mMotionVelocity.mTopxyz_bx[threadIndexInWarp];
			artiLinks[linkID].mPosMotionVelocity.mbyz[threadIndexInWarp] =
				artiLinks[linkID].mMotionVelocity.mbyz[threadIndexInWarp];
		}

		for(PxU32 dofId = 0; dofId < totalDofs; ++dofId)
		{
			artiDofs[dofId].mPosJointVelocities[threadIndexInWarp] = artiDofs[dofId].mJointVelocities[threadIndexInWarp];
		}

	}
}

extern "C" __global__ void artiComputeDependencies(PxgArticulationCoreDesc* scDesc, const PxU32 nbPartitions)
{
	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	// We're using one thread per articulation but some data is stored in blocks of 32 articulations
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	const PxU32 nbArticulations = scDesc->nbArticulations;

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 articulationId = globalThreadIndex;

		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

		const PxU32 nbSlabs = scDesc->nbSlabs;
		uint4* slabDirtyMasksBase = scDesc->slabDirtyMasks; // initialized to 0xffffffff in PxgArticulationCore::allocDeltaVBuffer

		PxU32* dirtyLink = scDesc->mImpulseHoldingLink;
		PxReal* partitionAverageScale = scDesc->mPartitionAverageScale;

		const PxU32 slabPartitionStride = nbArticulations * nbSlabs;

		const PxU32 wordSize = (maxLinks + 63) / 64;
		PxgArticulationBitFieldStackData* pathToRootBitField = &scDesc->mTempPathToRootBitFieldBlocks[globalWarpIndex * wordSize];
		PxgArticulationBitFieldStackData* sharedBitField = &scDesc->mTempSharedBitFieldBlocks[globalWarpIndex * wordSize];
		PxgArticulationBitFieldStackData* rootBitField = &scDesc->mTempRootBitFieldBlocks[globalWarpIndex * wordSize];

		const PxgArticulationBitFieldData* linkBitFieldBlocks = &scDesc->mPathToRootBitFieldBlocks[globalWarpIndex * maxLinks * wordSize];

		//zero the bit fields
		{
			PxgArticulationBitFieldStackData* pathToRootPerPartition = &scDesc->mPathToRootsPerPartition[globalWarpIndex * wordSize];
			for (PxU32 j = 0; j < wordSize; ++j)
			{
				pathToRootBitField[j].bitField[threadIndexInWarp] = 0;
				rootBitField[j].bitField[threadIndexInWarp] = 0;
				sharedBitField[j].bitField[threadIndexInWarp] = 0;
				pathToRootPerPartition[j].bitField[threadIndexInWarp] = 0;
			}
		}
		
		//All full mask
		PxU32 commonNode = 0xFFFFFFFF;
		PxU32 oldCommonNode = commonNode;

		bool sharedPathEmpty = true;
		
		const PxU32 nbArticulationBatchPerPartition = (nbArticulations + 31) / 32;

		for (PxU32 partition = 0; partition < nbPartitions; ++partition)
		{
			const PxU32 partitionOffset = articulationId + nbArticulations * partition;

			uint4* slabDirtyMasks = slabDirtyMasksBase + slabPartitionStride*partition; //Step to the next set of slab dirty masks!

			PxReal count = 0.f;

			for (PxU32 s = 0; s < nbSlabs; ++s)
			{
				const PxU32 slabDirtyIndex = articulationId + s * nbArticulations;
				const uint4 dirtyIndex2 = slabDirtyMasks[slabDirtyIndex];

				if (dirtyIndex2.x != 0xFFFFFFFF || dirtyIndex2.z != 0xFFFFFFFF)
					count += 1.f; // Register that this articulation has one or two links involved in this slab

				for (PxU32 i = 0, dirtyIndex = dirtyIndex2.x; i < 2; ++i, dirtyIndex = dirtyIndex2.z)
				{
					if (dirtyIndex != 0xFFFFFFFF)
					{
						const PxgArticulationBitFieldStackData* linkPathToRoot = &linkBitFieldBlocks[dirtyIndex * wordSize];
						PxU32 commonLink = 0;
						if (sharedPathEmpty)
						{
							//The highest set bit is the bit furthest from the root link (root link is the LSB - bit 0)
							for (PxI32 j = wordSize - 1; j >= 0; j--)
							{
								PxU64 word = linkPathToRoot[j].bitField[threadIndexInWarp];
								if (word != 0)
								{
									PxU32 bitIndex = articulationHighestSetBit(word);
									commonLink = bitIndex + j * 64;
									break;
								}
							}
						}
						else
						{
							//The highest set bit is the bit furthest from the root link (root link is the LSB - bit 0)
							for (PxI32 j = wordSize - 1; j >= 0; j--)
							{
								const PxU64 word0 = sharedBitField[j].bitField[threadIndexInWarp];
								const PxU64 word1 = linkPathToRoot[j].bitField[threadIndexInWarp];

								const PxU64 word = word0 & word1;

								if (word != 0)
								{
									PxU32 bitIndex = articulationHighestSetBit(word);
									commonLink = bitIndex + j * 64;
									break;
								}
							}
						}


						//Choose the smaller of the current common node and the common node found from this link
						commonNode = PxMin(commonLink, commonNode);

						//Append this link's path to root to the current path to root
						for (PxU32 j = 0; j < wordSize; ++j)
						{
							const PxU64 word = linkPathToRoot[j].bitField[threadIndexInWarp];
							pathToRootBitField[j].bitField[threadIndexInWarp] |= word;
							sharedBitField[j].bitField[threadIndexInWarp] |= word;
							sharedPathEmpty &= (word == 0);
						}
					}
				}
			} // end of loop over slabs

			//Common node changed, so re-load rootPath!
			if (commonNode != oldCommonNode)
			{

				const PxgArticulationBitFieldStackData* linkPathToRoot = &linkBitFieldBlocks[commonNode * wordSize];
				for (PxU32 j = 0; j < wordSize; ++j)
				{
					rootBitField[j].bitField[threadIndexInWarp] = linkPathToRoot[j].bitField[threadIndexInWarp];
				}

				oldCommonNode = commonNode;
			}
			

			const PxU32 nextPartition = partition + 1;
			//Output for this partition. This part is where to propagate up for this link
			if (nextPartition < nbPartitions)
			{
				const PxU32 offset = globalWarpIndex + nextPartition * nbArticulationBatchPerPartition * wordSize;
				PxgArticulationBitFieldStackData* pathToRootPerPartition = &scDesc->mPathToRootsPerPartition[offset];

				for (PxU32 j = 0; j < wordSize; ++j)
				{
					pathToRootPerPartition[j].bitField[threadIndexInWarp] = pathToRootBitField[j].bitField[threadIndexInWarp];
				}
			}


			for (PxU32 j = 0; j < wordSize; ++j)
			{
				PxU64 word = rootBitField[j].bitField[threadIndexInWarp];
				sharedBitField[j].bitField[threadIndexInWarp] = word;
				sharedPathEmpty &= (word == 0);
				//zero pathToRoot
				pathToRootBitField[j].bitField[threadIndexInWarp] = 0;
			}
			
			const PxReal scale = count < 1.f ? 0.f : 1.f/count;
			dirtyLink[partitionOffset] = commonNode;
			partitionAverageScale[partitionOffset] = scale;
		}
	}
}

extern "C" __global__ void artiApplyTgsSubstepForces(PxgArticulationCoreDesc* scDesc, const PxReal stepDt)
{
	// one thread per articulation
	const PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 globalWarpIndex = globalThreadIndex / WARP_SIZE;

	const PxU32 nbArticulations = scDesc->nbArticulations;
	const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
	const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

	if(globalThreadIndex < nbArticulations)
	{
		const PxgArticulation& articulation = scDesc->articulations[globalThreadIndex];
		const Cm::UnAlignedSpatialVector* PX_RESTRICT zExt = articulation.zAForces;
		const PxReal* PX_RESTRICT jointForces = articulation.jointForce;
		PxgArticulationBlockData& articulationBlock = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex * maxLinks];
		PxgArticulationBlockDofData* artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex * maxDofs];
		const PxU32 numLinks = articulationBlock.mNumLinks[threadIndexInWarp];
		const bool isFixedBase = articulationBlock.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;

		const Cm::UnAlignedSpatialVector zero = Cm::UnAlignedSpatialVector::Zero();

		// Since a link can have multiple children we're working with linkData.mScratchImpulse
		// to accumulate the propagated gravity effect of a child to its parent link.
		// The linkData.mScratchImpulse is likely zero here when entering this kernel because
		// it's at the beginning of an iteration but even if not, we're just adding to it, propagate
		// the impulses and then set the scratch back to zero.

		// from leaves towards the root
		for(PxI32 linkIdx = numLinks - 1; linkIdx > 0; linkIdx--)
		{
			PxgArticulationBlockLinkData& linkData = artiLinks[linkIdx];
			
			const PxU32 parent = linkData.mParents[threadIndexInWarp];
			const float rwx = linkData.mRw_x[threadIndexInWarp];
			const float rwy = linkData.mRw_y[threadIndexInWarp];
			const float rwz = linkData.mRw_z[threadIndexInWarp];

			const PxU32 dofCount = linkData.mDofs[threadIndexInWarp];

			const PxU32 jointOffset = linkData.mJointOffset[threadIndexInWarp];
			PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[jointOffset];
			const PxReal* PX_RESTRICT jointForce = &jointForces[jointOffset];

			const Cm::UnAlignedSpatialVector isolatedYW = zExt[linkIdx];
			const Cm::UnAlignedSpatialVector articulatedYW =
			    isolatedYW * stepDt + loadSpatialVector(linkData.mScratchImpulse, threadIndexInWarp);

			Cm::UnAlignedSpatialVector propagatedYWParent =
			    propagateImpulseW_0(PxVec3(rwx, rwy, rwz), dofData, articulatedYW, dofCount, threadIndexInWarp, jointForce, stepDt);

			if(parent > 0 || !isFixedBase)
				addSpatialVector(artiLinks[parent].mScratchImpulse, propagatedYWParent, threadIndexInWarp);

			// reset scratch impulse again for future kernels
			storeSpatialVector(linkData.mScratchImpulse, zero, threadIndexInWarp);
		}

		if(!isFixedBase)
		{
			PxgArticulationBlockLinkData& linkData = artiLinks[0];
			const Cm::UnAlignedSpatialVector isolatedYW = zExt[0];
			const Cm::UnAlignedSpatialVector articulatedYW =
			    isolatedYW * stepDt + loadSpatialVector(linkData.mScratchImpulse, threadIndexInWarp);

			addSpatialVector(articulationBlock.mRootDeferredZ, articulatedYW, threadIndexInWarp);

			// reset scratch impulse again for future kernels
			storeSpatialVector(linkData.mScratchImpulse, zero, threadIndexInWarp);
		}

		articulationBlock.mStateDirty[threadIndexInWarp] = PxgArtiStateDirtyFlag::eVEL_DIRTY;
	}
}

template <typename IterativeData>
__device__ void artiPropagateImpulses2(PxgArticulationCoreDesc* scDesc, 
	const PxgSolverCoreDesc* const PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeData>* const PX_RESTRICT sharedDesc,
	const PxU32 partitionId)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];

		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

		PxU8 dirtyFlag = articulation.mStateDirty[threadIndexInWarp];
		if (dirtyFlag)
		{
			PxgArticulationBlockLinkData* artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
			PxgArticulationBlockDofData* artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];
			const PxU32 numLinks = articulation.mNumLinks[threadIndexInWarp];

			if (partitionId == 0)
			{
				PxcFsFlushVelocity(articulation, artiLinks, artiDofs, numLinks,
					articulation.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE, threadIndexInWarp);
				articulation.mStateDirty[threadIndexInWarp] = 0;
			}
			else if (dirtyFlag & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
			{
				const PxU32 offset = globalThreadIndex + partitionId*nbArticulations;
				const PxReal partitionAverageScale = scDesc->mPartitionAverageScale[offset];
				 
				//If scale is 0, this articulation is not involved in any slab of this partition, so we can skip this path!
				if (partitionAverageScale > 0.f)
				{
					const PxU32 nbSlabs = scDesc->nbSlabs;
					uint2* isSlabDirty = scDesc->slabHasChanges;
					const PxU32 dirtyLink = scDesc->mImpulseHoldingLink[offset];

					const PxU32 nbArticulationBatchPerPartition = (nbArticulations + 31) / 32;
					const PxU32 wordSize = (maxLinks + 63) / 64;

					const PxU32 offset2 = globalWarpIndex + partitionId * nbArticulationBatchPerPartition * wordSize;

					// Counting the number of active slabs used in contacts and joints.
					// The split mass used in contacts and joints is tied back here.
					const PxU32 articulationReferenceCountOffset =
						solverDesc->islandContextPool->mBodyCount + solverDesc->islandContextPool->mBodyStartIndex;
					const PxU32 numSolverBodies = articulationReferenceCountOffset + scDesc->nbArticulations;
					const PxU32 referenceCount = countActiveSlabs(articulationReferenceCountOffset + globalThreadIndex, nbSlabs,
																  numSolverBodies, sharedDesc->iterativeData.solverEncodedReferenceCount);

					const PxReal scale = 1.0f / static_cast<PxReal>(referenceCount);

					PxgArticulationBitFieldStackData* pathToRootPerPartition = &scDesc->mPathToRootsPerPartition[offset2];

					Cm::UnAlignedSpatialVector* impulses = scDesc->impulses;

					averageLinkImpulsesAndPropagate2(isSlabDirty, impulses, articulation, artiLinks, artiDofs, globalThreadIndex, maxLinks,
						nbArticulations, nbSlabs, numLinks, threadIndexInWarp, scale, pathToRootPerPartition, wordSize, dirtyLink, dirtyFlag);

					//KS - we explicitly *do not* clear the dirty flags because this algorithm just does partial work!
					//articulation.mStateDirty[threadIndexInWarp] = (dirtyFlag & (~PxgArtiStateDirtyFlag::eHAS_IMPULSES));
				}
			}
		}
	}
}

extern "C" __global__ void artiPropagateImpulses2PGS(PxgArticulationCoreDesc* scDesc,
	const PxgSolverCoreDesc* const PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveData>* const PX_RESTRICT sharedDesc,
	const PxU32 partitionId)
{
	artiPropagateImpulses2(scDesc, solverDesc, sharedDesc, partitionId);
}

extern "C" __global__ void artiPropagateImpulses2TGS(PxgArticulationCoreDesc* scDesc,
	const PxgSolverCoreDesc* const PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* const PX_RESTRICT sharedDesc,
	const PxU32 partitionId)
{
	artiPropagateImpulses2(scDesc, solverDesc, sharedDesc, partitionId);
}

template <bool isTGS>
static __device__ void artiPropagateVelocityInternal(const PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxU32 partitionId,
	float4* PX_RESTRICT velocityOutput)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = threadIdx.y;
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];
		const PxU32 slabId = blockIdx.y; //Dimension y on blockIdx = which slabId
		
		const PxU32 offset = globalThreadIndex + partitionId*nbArticulations;

		const PxReal partitionAverageScale = scDesc->mPartitionAverageScale[offset];

		if(partitionAverageScale == 0.f)
			return;

		// AD: if partitionAverageScale is 0 here, that means we don't have any occurrence of this articulation
		// in this partition (which will be solved after this kernel). This means we don't have to write the deltaV out into the solverBodyVelPool
		// because nobody is reading it anyway.
		
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;
		const PxU32 wordSize = (maxLinks + 63) / 64;

		const PxU32 slabPartitionOffset = nbArticulations*scDesc->nbSlabs;

		PxU32 dirtyLink = scDesc->mImpulseHoldingLink[offset];

		PxgArticulationBlockLinkData* PX_RESTRICT artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* PX_RESTRICT artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];
		PxgArticulationBitFieldData* PX_RESTRICT linkBitFields = &scDesc->mPathToRootBitFieldBlocks[globalWarpIndex * maxLinks * wordSize];

		uint4* PX_RESTRICT slabDirtyMasks = scDesc->slabDirtyMasks + slabPartitionOffset*partitionId; //Step to the next set of slab dirty masks!

		uint4 dirtyIndex4 = slabDirtyMasks[globalThreadIndex + slabId*nbArticulations];
		for (PxU32 i = 0, dirtyIndex = dirtyIndex4.x, writeIndex = dirtyIndex4.y; i < 2; ++i, dirtyIndex = dirtyIndex4.z, writeIndex = dirtyIndex4.w)
		{
			if (dirtyIndex != 0xFFFFFFFF)
			{
				Cm::UnAlignedSpatialVector velocity = loadSpatialVector(artiLinks[dirtyIndex].mMotionVelocity, threadIndexInWarp);

				if (partitionId != 0 && dirtyLink != 0xFFFFFFFF)
				{
					Cm::UnAlignedSpatialVector deltaV = loadSpatialVector(articulation.mCommonLinkDeltaVelocity, threadIndexInWarp);

					for (PxU32 j = 0, wordOffset = 0; j < wordSize; ++j, wordOffset += 64)
					{
						//Propagate velocity down...
						const PxU64 wordA = linkBitFields[dirtyIndex * wordSize + j].bitField[threadIndexInWarp];
						const PxU64 wordB = linkBitFields[dirtyLink * wordSize + j].bitField[threadIndexInWarp];
						PxU64 pathToRoot = wordA & (~wordB);

						while (pathToRoot)
						{
							PxU32 index = articulationLowestSetBit(pathToRoot) + wordOffset;

							assert(index != 0);

							const PxgArticulationBlockLinkData& link = artiLinks[index];

							deltaV = propagateAccelerationWNoJVelUpdate(link, &artiDofs[link.mJointOffset[threadIndexInWarp]], link.mDofs[threadIndexInWarp],
								deltaV, threadIndexInWarp);

							pathToRoot &= (pathToRoot - 1);
						}


					}

					velocity += deltaV;
				}
					
				//Output velocity to be read in by solver...
				if (isTGS)
				{
					Cm::UnAlignedSpatialVector deltaMotion = loadSpatialVector(artiLinks[dirtyIndex].mDeltaMotion, threadIndexInWarp);
					velocityOutput[writeIndex] = make_float4(velocity.bottom.x, velocity.bottom.y, velocity.bottom.z, velocity.top.x);
					velocityOutput[writeIndex + 32] = make_float4(velocity.top.y, velocity.top.z, deltaMotion.bottom.x, deltaMotion.bottom.y);
					velocityOutput[writeIndex + 64] = make_float4(deltaMotion.bottom.z, deltaMotion.top.x, deltaMotion.top.y, deltaMotion.top.z);
				}
				else
				{
					velocityOutput[writeIndex] = make_float4(velocity.bottom.x, velocity.bottom.y, velocity.bottom.z, 0.f);
					velocityOutput[writeIndex+32] = make_float4(velocity.top.x, velocity.top.y, velocity.top.z, 0.f);
				}
			}
		}
	}
}

extern "C" __global__ void artiPropagateVelocity(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxU32 partitionId,
	float4* PX_RESTRICT velocityOutput)
{
	artiPropagateVelocityInternal<false>(scDesc, partitionId, velocityOutput);
}


extern "C" __global__ void artiPropagateVelocityTGS(PxgArticulationCoreDesc* PX_RESTRICT scDesc, const PxU32 partitionId,
	float4* PX_RESTRICT velocityOutput)
{
	artiPropagateVelocityInternal<true>(scDesc, partitionId, velocityOutput);
}


extern "C" __global__ void dmaArticulationResidual(PxgArticulationCoreDesc* scDesc, Dy::ErrorAccumulator* errorPinnedHost)
{
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
	 
	const PxNodeIndex* gIslandNodeIndex = scDesc->islandNodeIndices;
	const PxgBodySim* gBodySim = scDesc->mBodySimBufferDeviceData;
	const PxU32 articulationOffset = scDesc->articulationOffset;
	const PxgArticulation* articulations = scDesc->articulations;

	const PxU32 nbArticulations = scDesc->nbArticulations;

	if (globalThreadIdx < nbArticulations)
	{
		const PxU32 nodeIndex = gIslandNodeIndex[globalThreadIdx + articulationOffset].index();

		const PxgBodySim& bodySim = gBodySim[nodeIndex];

		const PxU32 index = bodySim.articulationRemapId;

		const PxgArticulation& articulation = articulations[index];

		assert(index < nbArticulations);		
		errorPinnedHost[index] = articulation.internalResidualAccumulator; //Copies the position iteration residual because this kernel runs after the position iterations and before the velocity iterations
		errorPinnedHost[index + nbArticulations] = articulation.contactResidualAccumulator;
	}
}


extern "C" __global__ void dmaBackArticulationDataLaunch(
	PxgArticulationCoreDesc* scDesc,
	PxgArticulationOutputDesc* outputDesc)
{

	const PxU32 nbArticulations = scDesc->nbArticulations;

	const PxU32 warpIndex = threadIdx.y;

	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	PxNodeIndex* gIslandNodeIndex = scDesc->islandNodeIndices;
	PxgBodySim* gBodySim = scDesc->mBodySimBufferDeviceData;
	PxgSolverBodySleepData* gArticulationSleepData = scDesc->articulationSleepData;
	PxU32 articulationOffset = scDesc->articulationOffset;
	PxgArticulation* articulations = scDesc->articulations;

	if (globalThreadIndex == 0 && outputDesc->contactResidualAccumulator)
		outputDesc->contactResidualAccumulator[0] = scDesc->mContactErrorAccumulator;

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 nodeIndex = gIslandNodeIndex[globalThreadIndex + articulationOffset].index();

		PxgBodySim& bodySim = gBodySim[nodeIndex];

		PxgSolverBodySleepData* gOutputSleepData = outputDesc->sleepData;

		const PxU32 index = bodySim.articulationRemapId;

		reinterpret_cast<uint2*>(gOutputSleepData)[globalThreadIndex] = reinterpret_cast<uint2*>(gArticulationSleepData)[index];
	}

	__syncwarp();

	if (globalWarpIndex < nbArticulations)
	{


		const PxU32 nodeIndex = gIslandNodeIndex[globalWarpIndex + articulationOffset].index();

		PxgBodySim& bodySim = gBodySim[nodeIndex];

		const PxU32 index = bodySim.articulationRemapId;

		PxgArticulation& articulation = articulations[index];
		if (outputDesc->errorAccumulator) 
		{
			outputDesc->errorAccumulator[index] = articulation.internalResidualAccumulator; //Copies the velocity iteration residual because this kernel runs at the end of the solver
			outputDesc->errorAccumulator[index + nbArticulations] = articulation.contactResidualAccumulator;
		}

		//copy from buffers unique to each articulation to a single buffer for the entire scene.

		uint4* outBuff = reinterpret_cast<uint4*>(
			PxgArticulationLinkJointRootStateData::getArticulationStateDataBuffer(
				outputDesc->linkAndJointAndRootStateData, 
				scDesc->mMaxLinksPerArticulation, scDesc->mMaxDofsPerArticulation,
				globalWarpIndex));

		const PxU32 outSize = 
			PxgArticulationLinkJointRootStateData::computeStateDataBufferByteSizeAligned16
				(articulation.data.numLinks, articulation.data.numJointDofs, 1);

		uint4* sLinkAndJointAndRootStateData = 
			reinterpret_cast<uint4*>(articulation.linkJointRootStateDataBuffer);

		warpCopy<uint4>(outBuff, sLinkAndJointAndRootStateData, outSize);
	}
}

static PX_FORCE_INLINE PX_CUDA_CALLABLE PxQuat loadSpatialTransformQuatConjugate(const PxgSpatialTransformBlock& block, const PxU32 threadIndexInWarp)
{
	const float4 q = block.q[threadIndexInWarp];
	return PxQuat(-q.x, -q.y, -q.z, q.w);
}

// TGS only
extern "C" __global__ void stepArticulation1TTGS(const PxgArticulationCoreDesc* const PX_RESTRICT scDesc, PxReal stepDt)
{
	const PxU32 nbArticulations = scDesc->nbArticulations;

	// PT: kernel is called with:
	// gridDimX = numBlocks;				=> blockIdx.x is in [0;numBlocks[
	// gridDimY = 1;						=> blockIdx.y = 1
	// gridDimZ = 1;						=> blockIdx.z = 1
	// blockDimX = numThreadsPerWarp (32);	=> threadIdx.x is in [0;numThreadsPerWarp[
	// blockDimY = numWarpsPerBlock (2);	=> threadIdx.y is in [0;numWarpsPerBlock[
	// blockDimZ = 1;						=> threadIdx.z = 1

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 globalThreadIndex = globalWarpIndex*WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

		PxgArticulationBlockData& PX_RESTRICT articulation = scDesc->mArticulationBlocks[globalWarpIndex];
		PxgArticulationBlockLinkData* PX_RESTRICT artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* PX_RESTRICT artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];
		const PxU32 numLinks = articulation.mNumLinks[threadIndexInWarp];

		PxU8 stateDirty = articulation.mStateDirty[threadIndexInWarp];

		if (stateDirty)
		{
			if (stateDirty & PxgArtiStateDirtyFlag::eHAS_IMPULSES)
			{
				averageLinkImpulsesAndPropagate(scDesc->slabHasChanges, scDesc->impulses, articulation, artiLinks, artiDofs, globalThreadIndex, maxLinks,
					nbArticulations, scDesc->nbSlabs, numLinks, threadIndexInWarp);
			}
			PxcFsFlushVelocity(articulation, artiLinks, artiDofs, numLinks, articulation.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE, threadIndexInWarp);
			articulation.mStateDirty[threadIndexInWarp] = false;
		}

		//Now data has been flushed, we need to integrate by the tiny step...
		if (!(articulation.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE))
		{
			PxgArticulationBlockLinkData& link = artiLinks[0];

			const PxQuat prevPoseConjugate = loadSpatialTransformQuatConjugate(link.mPreTransform, threadIndexInWarp);
			const Cm::UnAlignedSpatialVector motionVelocity = loadSpatialVector(link.mMotionVelocity, threadIndexInWarp);

			const Cm::UnAlignedSpatialVector deltaMotion = loadSpatialVector(link.mDeltaMotion, threadIndexInWarp);
			const Cm::UnAlignedSpatialVector posMotionVelocity = loadSpatialVector(link.mPosMotionVelocity, threadIndexInWarp);

			const PxTransform body2World = updateRootBody(motionVelocity, loadSpatialTransform(link.mAccumulatedPose, threadIndexInWarp),
				stepDt, threadIndexInWarp);

			PxQuat dq = body2World.q * prevPoseConjugate;

			if (dq.w < 0.f)
				dq = -dq;

			link.mDeltaQ[threadIndexInWarp] = make_float4(dq.x, dq.y, dq.z, dq.w);

			const Cm::UnAlignedSpatialVector delta = motionVelocity * stepDt;

			storeSpatialVector(link.mDeltaMotion, deltaMotion + delta, threadIndexInWarp);
			storeSpatialVector(link.mPosMotionVelocity, posMotionVelocity + delta, threadIndexInWarp);

			storeSpatialTransform(link.mAccumulatedPose, threadIndexInWarp, body2World);
		}

		for (PxU32 linkID = 1; linkID < numLinks; ++linkID)
		{
			PxgArticulationBlockLinkData& link = artiLinks[linkID];

			const PxQuat prevPoseConjugate = loadSpatialTransformQuatConjugate(link.mPreTransform, threadIndexInWarp);
			Cm::UnAlignedSpatialVector deltaMotionVel = loadSpatialVector(link.mMotionVelocity, threadIndexInWarp);
			Cm::UnAlignedSpatialVector deltaMotion = loadSpatialVector(link.mDeltaMotion, threadIndexInWarp);
			const Cm::UnAlignedSpatialVector posMotionVelocity = loadSpatialVector(link.mPosMotionVelocity, threadIndexInWarp);

			PxTransform body2World;
			propagateLink<true>(body2World, link, artiLinks, artiDofs, stepDt, threadIndexInWarp);

			PxQuat dq = body2World.q * prevPoseConjugate;

			if (dq.w < 0.f)
				dq = -dq;

			link.mDeltaQ[threadIndexInWarp] = make_float4(dq.x, dq.y, dq.z, dq.w);

			//The accumulated delta may be computed from either the change in body2World:
			//deltaLin = currPos - origPos; deltaAng = dqAxis*dqAngle 
			//or by integrating the linear and angular parts of deltaMotionVel:
			//deltaLin += deltaVelLin*dt; deltaAng += deltaVelAng*dt.
			//In the limit that dt tends towards zero, these two methods should give the same outcome.
			//currPos - origPos was found to accumulate more numeric noise.
			//https://nvidia-omniverse.atlassian.net/browse/PX-3638
			//Computing dqAxis and dqAngle is likely more expensive than integrating the angular velocity.
			deltaMotionVel *= stepDt;
			deltaMotion += deltaMotionVel;

			storeSpatialVector(link.mDeltaMotion, deltaMotion, threadIndexInWarp);

			storeSpatialVector(link.mPosMotionVelocity, deltaMotionVel + posMotionVelocity, threadIndexInWarp);

			storeSpatialTransform(link.mAccumulatedPose, threadIndexInWarp, body2World);
		}
	}
}



//This version writes out the link velocity during each solver iterations. This will
//allow articulations to work with features outside of the rigid solver (e.g., deformables) correctly.
//Otherwise the same as the version in articulationDynamic.cuh.
static void __device__ PxcFsFlushVelocity(PxgArticulationBlockData& articulation,
	PxgArticulationBlockLinkData* artiLinks,
	PxgArticulationBlockDofData* artiDofs,
	PxU32 linkCount, bool fixBase, const PxU32 threadIndexInWarp,
	float4* outVelocity, const PxU32 offset) //linear start at 0, angular start at offset
{
	Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
	Cm::UnAlignedSpatialVector deferredZ = -loadSpatialVector(articulation.mRootDeferredZ, threadIndexInWarp);
	if (!fixBase)
	{
		Dy::SpatialMatrix invInertia;
		loadSpatialMatrix(articulation.mInvSpatialArticulatedInertia, threadIndexInWarp, invInertia);
		//deltaV = invInertia * (-loadSpatialVector(artiLinks[0].mDeferredZ, threadIndexInWarp));

		deltaV = invInertia * deferredZ;

		Cm::UnAlignedSpatialVector vel = loadSpatialVector(artiLinks[0].mMotionVelocity, threadIndexInWarp);

		vel += deltaV;

		//output velocity to the solver outArtiVelocity buffer
		outVelocity[0] = make_float4(vel.bottom.x, vel.bottom.y, vel.bottom.z, 0.f);
		outVelocity[offset] = make_float4(vel.top.x, vel.top.y, vel.top.z, 0.f);

		//store back vel to block velocity data
		storeSpatialVector(artiLinks[0].mMotionVelocity, vel, threadIndexInWarp);

		storeSpatialVector(articulation.mRootDeferredZ, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
	}

	storeSpatialVector(artiLinks[0].mScratchDeltaV, deltaV, threadIndexInWarp);
	addSpatialVector(artiLinks[0].mConstraintForces, deferredZ, threadIndexInWarp);
	
	storeSpatialVector(articulation.mCommonLinkDeltaVelocity, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);

	PxgArticulationBlockDofData* dofs = artiDofs;

	if (linkCount > 1)
	{
		Cm::UnAlignedSpatialVector nextMotionV = loadSpatialVector(artiLinks[1].mMotionVelocity, threadIndexInWarp);
		PxU32 nextNbDofs = artiLinks[1].mDofs[threadIndexInWarp];
		PxU32 nextParent = artiLinks[1].mParents[threadIndexInWarp];

		for (PxU32 i = 1; i < linkCount; i++)
		{
			PxgArticulationBlockLinkData& tLink = artiLinks[i];
			const PxU32 nbDofs = nextNbDofs;
			const PxU32 parent = nextParent;

			Cm::UnAlignedSpatialVector motionV = nextMotionV;

			if ((i + 1) < linkCount)
			{
				nextMotionV = loadSpatialVector(artiLinks[i + 1].mMotionVelocity, threadIndexInWarp);
				nextNbDofs = artiLinks[i + 1].mDofs[threadIndexInWarp];
				nextParent = artiLinks[i + 1].mParents[threadIndexInWarp];
			}

			if (parent != (i - 1))
				deltaV = loadSpatialVector(artiLinks[parent].mScratchDeltaV, threadIndexInWarp);

			deltaV = propagateAccelerationW(tLink, dofs, nbDofs, deltaV, threadIndexInWarp);

			//zeroing mDeferredQstZ
			for (PxU32 ind = 0; ind < nbDofs; ++ind)
			{
				dofs[ind].mDeferredQstZ[threadIndexInWarp] = 0.f;
			}

			motionV += deltaV;

			storeSpatialVector(tLink.mScratchDeltaV, deltaV, threadIndexInWarp);
			
			outVelocity[i] = make_float4(motionV.bottom.x, motionV.bottom.y, motionV.bottom.z, 0.f);
			outVelocity[i + offset] = make_float4(motionV.top.x, motionV.top.y, motionV.top.z, 0.f);

			storeSpatialVector(tLink.mMotionVelocity, motionV, threadIndexInWarp);

			addSpatialVector(tLink.mConstraintForces, deltaV, threadIndexInWarp);

			dofs += nbDofs;
		}
	}
}


extern "C" __global__ void artiOutputVelocity(
	PxgArticulationCoreDesc* scDesc,
	const PxgSolverCoreDesc* solverCoreDesc,
	const bool isTGS)
{
	//shared memory shared within a block
	const PxU32 nbArticulations = scDesc->nbArticulations;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = threadIdx.y;
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];

		const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;

		const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

		PxgArticulationBlockLinkData* artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
		PxgArticulationBlockDofData* artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];
		const PxU32 numLinks = articulation.mNumLinks[threadIndexInWarp];

		float4* outArtiVelocity = &solverCoreDesc->outArtiVelocity[globalThreadIndex*maxLinks];
		const PxU32 offset = nbArticulations * maxLinks;
		PxU8 dirtyFlag = articulation.mStateDirty[threadIndexInWarp];
		if (dirtyFlag)
		{
			PxcFsFlushVelocity(articulation, artiLinks, artiDofs, numLinks,
				articulation.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE,
				threadIndexInWarp, outArtiVelocity, offset);

			articulation.mStateDirty[threadIndexInWarp] = 0;
		}
		else
		{
			for (PxU32 i = 0; i < numLinks; i++)
			{
				Cm::UnAlignedSpatialVector vel = loadSpatialVector(artiLinks[i].mMotionVelocity, threadIndexInWarp);
				outArtiVelocity[i] = make_float4(vel.bottom.x, vel.bottom.y, vel.bottom.z, 0.f);
				outArtiVelocity[i + offset] = make_float4(vel.top.x, vel.top.y, vel.top.z, 0.f);
			}
		}

		if (isTGS)
		{
			for (PxU32 i = 0; i < numLinks; i++)
			{
				Cm::UnAlignedSpatialVector delta = loadSpatialVector(artiLinks[i].mDeltaMotion, threadIndexInWarp);
				outArtiVelocity[i + 2*offset] = make_float4(delta.bottom.x, delta.bottom.y, delta.bottom.z, 0.f);
				outArtiVelocity[i + 3*offset] = make_float4(delta.top.x, delta.top.y, delta.top.z, 0.f);
			}
		}
		
	}
}

extern "C" __global__ void artiPushImpulse(
	PxgArticulationCoreDesc* scDesc)
{
	//shared memory shared within a block
	const PxU32 nbArticulations = scDesc->nbArticulations;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = threadIdx.y;
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxU32 globalThreadIndex = globalWarpIndex * WARP_SIZE + threadIdx.x;

	if (globalThreadIndex < nbArticulations)
	{
		PxgArticulationBlockData& articulation = scDesc->mArticulationBlocks[globalWarpIndex];

		PxU8 dirtyFlag = articulation.mStateDirty[threadIndexInWarp];
		if (dirtyFlag)
		{
			const PxU32 maxLinks = scDesc->mMaxLinksPerArticulation;
			const PxU32 maxDofs = scDesc->mMaxDofsPerArticulation;

			PxgArticulationBlockLinkData* artiLinks = &scDesc->mArticulationLinkBlocks[globalWarpIndex*maxLinks];
			PxgArticulationBlockDofData* artiDofs = &scDesc->mArticulationDofBlocks[globalWarpIndex*maxDofs];


			PxU32 nbLinks = articulation.mNumLinks[threadIndexInWarp];

			for(PxU32 index = nbLinks-1; index > 0; --index)
			{
				PxgArticulationBlockLinkData& linkData = artiLinks[index];

				PxgArticulationBlockDofData* PX_RESTRICT dofData = &artiDofs[linkData.mJointOffset[threadIndexInWarp]];

				const PxU32 parent = linkData.mParents[threadIndexInWarp];
				const float rwx = linkData.mRw_x[threadIndexInWarp];
				const float rwy = linkData.mRw_y[threadIndexInWarp];
				const float rwz = linkData.mRw_z[threadIndexInWarp];
				const PxU32 dofCount = linkData.mDofs[threadIndexInWarp];

				const Cm::UnAlignedSpatialVector Z = loadSpatialVector(linkData.mScratchImpulse, threadIndexInWarp);

				Cm::UnAlignedSpatialVector propagatedZ = propagateImpulseW_0(PxVec3(rwx, rwy, rwz),
					dofData, Z,
					dofCount, threadIndexInWarp);

				//KS - we should be able to remove mImpulses once we are 100% certain that we will not have any deferredZ residuals 
				addSpatialVector(artiLinks[parent].mScratchImpulse, propagatedZ, threadIndexInWarp);
				storeSpatialVector(linkData.mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
			}

			const bool fixedBase = articulation.mFlags[threadIndexInWarp] & PxArticulationFlag::eFIX_BASE;
			
			if (!fixedBase)
			{
				//(1) Compute updated link velocity...
				PxSpatialMatrix mat;
				loadSpatialMatrix(artiLinks[0].mSpatialResponseMatrix, threadIndexInWarp, mat);

				Cm::UnAlignedSpatialVector deltaV = mat * (-loadSpatialVector(artiLinks[0].mScratchImpulse, threadIndexInWarp));

				storeSpatialVector(articulation.mRootDeferredZ, deltaV, threadIndexInWarp);

				storeSpatialVector(artiLinks[0].mScratchImpulse, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
			}
		}
	}
}

