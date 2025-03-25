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

#ifndef	__ARTI_DYNAMIC_CUH__
#define	__ARTI_DYNAMIC_CUH__

#include "PxgArticulation.h"
#include "PxgArticulationLink.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyFeatherstoneArticulation.h"

//This function stores Q-stZ to mDeferredQstZ
static __device__ Cm::UnAlignedSpatialVector propagateImpulseW_0(const PxVec3& childToParent,
	PxgArticulationBlockDofData* PX_RESTRICT dofData, const Cm::UnAlignedSpatialVector& Z,
	const PxU32 dofCount, const PxU32 threadIndexInWarp,
	const PxReal* PX_RESTRICT jointForce = NULL, const PxReal jointForceMultiplier = 1.0f)
{
	Cm::UnAlignedSpatialVector temp = Z;
	Cm::UnAlignedSpatialVector sas[3];
	Cm::UnAlignedSpatialVector isInvD[3];
	PxReal jf[3];

// the split into two separate loops is an optimization that allows dispatching the loads as early as possible.
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			sas[ind] = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
			isInvD[ind] = loadSpatialVector(dofData[ind].mIsInvDW, threadIndexInWarp);
			jf[ind] = (jointForce ? jointForce[ind] * jointForceMultiplier : 0.0f);
		}
	}

#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			const PxReal stZ = jf[ind] - sas[ind].innerProduct(Z);
			dofData[ind].mDeferredQstZ[threadIndexInWarp] += stZ;
			temp += isInvD[ind] * stZ;
		}
	}

	//parent space's spatial zero acceleration impulse
	return Dy::FeatherstoneArticulation::translateSpatialVector(childToParent, temp);
}

static __device__ Cm::UnAlignedSpatialVector propagateImpulseWTemp(const PxVec3& childToParent,
	PxgArticulationBlockDofData* PX_RESTRICT dofData, const Cm::UnAlignedSpatialVector& Z,
	const PxU32 dofCount, const PxU32 threadIndexInWarp)
{
	Cm::UnAlignedSpatialVector temp = Z;

	assert(dofCount<=3);
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if(ind<dofCount)
		{
			const Cm::UnAlignedSpatialVector sa = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
			const Cm::UnAlignedSpatialVector isInvD = loadSpatialVector(dofData[ind].mIsInvDW, threadIndexInWarp);
			const PxReal stZ = -sa.innerProduct(Z);
			dofData[ind].mTmpQstZ[threadIndexInWarp] += stZ;

			temp += isInvD * stZ;
		}
	}

	//parent space's spatial zero acceleration impulse
	return Dy::FeatherstoneArticulation::translateSpatialVector(childToParent, temp);
}

static __device__ Cm::UnAlignedSpatialVector propagateImpulseW_1(
	const PxVec3& childToParent,
	const PxgArticulationBlockDofData* PX_RESTRICT dofData, 
	const Cm::UnAlignedSpatialVector& Z,
	const PxReal* jointDofImpulses, const PxU32 dofCount,
	const PxU32 threadIndexInWarp, 
	PxReal* qstZ)
{
	Cm::UnAlignedSpatialVector temp = Z;

	assert(dofCount<=3);
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if(ind<dofCount)
		{
			const Cm::UnAlignedSpatialVector sa = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
			const Cm::UnAlignedSpatialVector isInvD = loadSpatialVector(dofData[ind].mIsInvDW, threadIndexInWarp);
			const PxReal jointDofImpulse = jointDofImpulses ? jointDofImpulses[ind] : 0.0f;
			const PxReal QMinusSTZ = jointDofImpulse - sa.innerProduct(Z);
			qstZ[ind] += QMinusSTZ;

			temp += isInvD * QMinusSTZ;
		}
	}

	//parent space's spatial zero acceleration impulse
	return Dy::FeatherstoneArticulation::translateSpatialVector(childToParent, temp);
}

static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW(const PxVec3& c2p, const float3* invStIsT,
	const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::UnAlignedSpatialVector& hDeltaV, const PxU32 dofCount,
	const Cm::UnAlignedSpatialVector* IsW, const PxReal* qstZ)
{
	Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-c2p, hDeltaV); //parent velocity change

	//Convert parent velocity change into an impulse
	PxReal tJointDelta[3] = { 0.f, 0.f, 0.f };
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			//stI * pAcceleration
			const PxReal temp = IsW[ind].innerProduct(pDeltaV);

			tJointDelta[ind] = (qstZ[ind] - temp);
		}
	}

	assert(dofCount<=3);
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if(ind<dofCount)
		{
			const float3 iStIsTi = invStIsT[ind];

			const PxReal jDelta = iStIsTi.x * tJointDelta[0]
								+ iStIsTi.y * tJointDelta[1]
								+ iStIsTi.z * tJointDelta[2];

			pDeltaV += motionMatrix[ind] * jDelta;
		}
	}

	return pDeltaV;
}

static __device__ Cm::UnAlignedSpatialVector computeSpatialJointDelta(
	const PxgArticulationBlockDofData* PX_RESTRICT dofData,
	const PxReal* PX_RESTRICT QSTZMinusISDotTranslatedParentDeltaV, PxReal* PX_RESTRICT jointDeltaDofSpeeds, const PxU32 dofCount, 
	const PxU32 threadIndexInWarp)
{
	Cm::UnAlignedSpatialVector sas[3];

	// the split into two separate loops is an optimization that allows dispatching the loads as early as possible.
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			sas[ind] = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
		}
	}

	Cm::UnAlignedSpatialVector jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			const float iStIsTi_x = dofData[ind].mInvStIsT_x[threadIndexInWarp];
			const float iStIsTi_y = dofData[ind].mInvStIsT_y[threadIndexInWarp];
			const float iStIsTi_z = dofData[ind].mInvStIsT_z[threadIndexInWarp];

			const PxReal jDelta = iStIsTi_x * QSTZMinusISDotTranslatedParentDeltaV[0]
								+ iStIsTi_y * QSTZMinusISDotTranslatedParentDeltaV[1]
								+ iStIsTi_z * QSTZMinusISDotTranslatedParentDeltaV[2];

			if(jointDeltaDofSpeeds)
				jointDeltaDofSpeeds[ind] = jDelta;

			jointSpatialDeltaV += sas[ind] * jDelta;
		}
	}

	return jointSpatialDeltaV;
}

//This function use mDeferredQstZ
static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW(const PxVec3& c2p,
	const PxgArticulationBlockDofData* PX_RESTRICT dofData,
	const Cm::UnAlignedSpatialVector& hDeltaV,
	const PxU32 dofCount, PxReal* jointDeltaDofSpeeds, const PxU32 threadIndexInWarp)
{
	const Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-c2p, hDeltaV); //parent velocity change

	//[(Q - S^T *Z)] - [(I*S).innerProduct(translated(parentDeltaV))]
	PxReal QSTZMinusISDotTranslatedParentDeltaV[3] = { 0.f, 0.f, 0.f };
	Cm::UnAlignedSpatialVector IsW;
#pragma unroll(3)
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			IsW = loadSpatialVector(dofData[ind].mIsW, threadIndexInWarp);
			//stI * pAcceleration
			const PxReal temp = IsW.innerProduct(pDeltaV);

			QSTZMinusISDotTranslatedParentDeltaV[ind] = (dofData[ind].mDeferredQstZ[threadIndexInWarp] - temp);
		}
	}

	const Cm::UnAlignedSpatialVector jointSpatialDeltaV = computeSpatialJointDelta(dofData, QSTZMinusISDotTranslatedParentDeltaV, jointDeltaDofSpeeds, dofCount, threadIndexInWarp);

	return pDeltaV + jointSpatialDeltaV;
}

//This function use mTmpQstZ
static __device__ Cm::UnAlignedSpatialVector propagateAccelerationWTemp(const PxVec3& c2p,
	const PxgArticulationBlockDofData* PX_RESTRICT dofData,
	const Cm::UnAlignedSpatialVector& hDeltaV,
	const PxU32 dofCount, const PxU32 threadIndexInWarp)
{
	const Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-c2p, hDeltaV); //parent velocity change

	//[(Q - S^T *Z)] - [(I*S).innerProduct(translated(parentDeltaV))]
	PxReal QSTZMinusISDotTransaltedParentDeltaV[3] = { 0.f, 0.f, 0.f };

#pragma unroll(3)
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			const Cm::UnAlignedSpatialVector IsW = loadSpatialVector(dofData[ind].mIsW, threadIndexInWarp);
			//stI * pAcceleration
			const PxReal temp = IsW.innerProduct(pDeltaV);

			QSTZMinusISDotTransaltedParentDeltaV[ind] = (dofData[ind].mTmpQstZ[threadIndexInWarp] - temp);
		}
	}

	const Cm::UnAlignedSpatialVector jointSpatialDeltaV = computeSpatialJointDelta(dofData, QSTZMinusISDotTransaltedParentDeltaV, NULL, dofCount, threadIndexInWarp);

	return pDeltaV + jointSpatialDeltaV;
}

//This function use qstZ as input
static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW(const PxVec3& c2p,
	const PxgArticulationBlockDofData* PX_RESTRICT dofData,
	const Cm::UnAlignedSpatialVector& hDeltaV,
	const PxU32 dofCount, const PxU32 threadIndexInWarp,
	const PxReal* qstZ)
{
	const Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-c2p, hDeltaV); //parent velocity change

	//[(Q - S^T *Z)] - [(I*S).innerProduct(translated(parentDeltaV))]
	PxReal QSTZMinusISDotTransaltedParentDeltaV[3] = { 0.f, 0.f, 0.f };
	Cm::UnAlignedSpatialVector IsW;
#pragma unroll(3)
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			IsW = loadSpatialVector(dofData[ind].mIsW, threadIndexInWarp);
			//stI * pAcceleration
			const PxReal temp = IsW.innerProduct(pDeltaV);

			QSTZMinusISDotTransaltedParentDeltaV[ind] = (qstZ[ind] - temp);
		}
	}

	const Cm::UnAlignedSpatialVector jointSpatialDeltaV = computeSpatialJointDelta(dofData, QSTZMinusISDotTransaltedParentDeltaV, NULL, dofCount, threadIndexInWarp);

	return pDeltaV + jointSpatialDeltaV;
}

static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW(
	PxgArticulationBlockLinkData& linkData,
	PxgArticulationBlockDofData* dofData,
	const PxU32 dofCount,
	const Cm::UnAlignedSpatialVector& hDeltaV,
	const PxU32 threadIndexInWarp)
{
	const float c2px = linkData.mRw_x[threadIndexInWarp];
	const float c2py = linkData.mRw_y[threadIndexInWarp];
	const float c2pz = linkData.mRw_z[threadIndexInWarp];

	float3 invStIsT[3];
	PxReal tJointDelta[3] = { 0.f, 0.f, 0.f };
	Cm::UnAlignedSpatialVector isW[3];
	Cm::UnAlignedSpatialVector mMotionMatrix[3];
	PxReal jVel[3];

// the split into three separate loops is an optimization that allows dispatching the loads as early as possible.
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			isW[ind] = loadSpatialVector(dofData[ind].mIsW, threadIndexInWarp);
			tJointDelta[ind] = (dofData[ind].mDeferredQstZ[threadIndexInWarp]);
			jVel[ind] = dofData[ind].mJointVelocities[threadIndexInWarp];
			invStIsT[ind] = make_float3(dofData[ind].mInvStIsT_x[threadIndexInWarp], dofData[ind].mInvStIsT_y[threadIndexInWarp], dofData[ind].mInvStIsT_z[threadIndexInWarp]);
			mMotionMatrix[ind] = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
		}
	}

	Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(PxVec3(-c2px, -c2py, -c2pz), hDeltaV); //parent velocity change
	
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			tJointDelta[ind] -= isW[ind].innerProduct(pDeltaV);
		}
	}

#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			const PxReal jDelta = invStIsT[ind].x * tJointDelta[0] + invStIsT[ind].y * tJointDelta[1]
				+ invStIsT[ind].z * tJointDelta[2];

			dofData[ind].mJointVelocities[threadIndexInWarp] = jVel[ind] + jDelta;

			pDeltaV += mMotionMatrix[ind] * jDelta;
		}
	}

	return pDeltaV;
}

// There is a another version of this function in forwardDynamic2.cu which additionally writes
// the link velocity to a global buffer
static void __device__ PxcFsFlushVelocity(PxgArticulationBlockData& articulation,
	PxgArticulationBlockLinkData* PX_RESTRICT artiLinks,
	PxgArticulationBlockDofData* PX_RESTRICT artiDofs,
	PxU32 linkCount, bool fixBase, const PxU32 threadIndexInWarp)
{
	Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
	Cm::UnAlignedSpatialVector deferredZ = -loadSpatialVector(articulation.mRootDeferredZ, threadIndexInWarp);
	if (!fixBase)
	{
		//ArticulationLink& link = links[0];

		// PT: preload data
		const Cm::UnAlignedSpatialVector motionVelocity0 = loadSpatialVector(artiLinks[0].mMotionVelocity, threadIndexInWarp);
		const Cm::UnAlignedSpatialVector solverSpatialDeltaVel0 = loadSpatialVector(artiLinks[0].mSolverSpatialDeltaVel, threadIndexInWarp);

		Dy::SpatialMatrix invInertia;
		loadSpatialMatrix(articulation.mInvSpatialArticulatedInertia, threadIndexInWarp, invInertia);
		//deltaV = invInertia * (-loadSpatialVector(artiLinks[0].mDeferredZ, threadIndexInWarp));

		deltaV = invInertia * deferredZ;

		//motionVelocities[0] += deltaV[0];
		storeSpatialVector(artiLinks[0].mMotionVelocity, motionVelocity0 + deltaV, threadIndexInWarp);
		//storeSpatialVector(artiLinks[0].mDeferredZ, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		storeSpatialVector(articulation.mRootDeferredZ, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);
		storeSpatialVector(artiLinks[0].mSolverSpatialDeltaVel, solverSpatialDeltaVel0 + deltaV, threadIndexInWarp);	
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
		//Cm::UnAlignedSpatialVector nextDeferredZ = loadSpatialVector(artiLinks[1].mDeferredZ, threadIndexInWarp);

		for (PxU32 i = 1; i < linkCount; i++)
		{
			PxgArticulationBlockLinkData& tLink = artiLinks[i];
			const PxU32 nbDofs = nextNbDofs;
			const PxU32 parent = nextParent;

			const Cm::UnAlignedSpatialVector preloadedConstraintForces = loadSpatialVector(tLink.mConstraintForces, threadIndexInWarp);
			const Cm::UnAlignedSpatialVector preloadedSolverSpatialDeltaVel = loadSpatialVector(tLink.mSolverSpatialDeltaVel, threadIndexInWarp);

			Cm::UnAlignedSpatialVector motionV = nextMotionV;
			//const Cm::UnAlignedSpatialVector deferredZ = nextDeferredZ;

			//storeSpatialVector(tLink.mDeferredZ, Cm::UnAlignedSpatialVector::Zero(), threadIndexInWarp);

			if ((i + 1) < linkCount)
			{
				nextMotionV = loadSpatialVector(artiLinks[i + 1].mMotionVelocity, threadIndexInWarp);
				nextNbDofs = artiLinks[i + 1].mDofs[threadIndexInWarp];
				nextParent = artiLinks[i + 1].mParents[threadIndexInWarp];
				//nextDeferredZ = loadSpatialVector(artiLinks[i + 1].mDeferredZ, threadIndexInWarp);
			}

			if (parent != (i - 1))
				deltaV = loadSpatialVector(artiLinks[parent].mScratchDeltaV, threadIndexInWarp);

			deltaV = propagateAccelerationW(tLink, dofs, nbDofs, deltaV, threadIndexInWarp);

			//Accumulate the DeltaVel arising from solver impulses applied to this link.
			storeSpatialVector(tLink.mSolverSpatialDeltaVel, preloadedSolverSpatialDeltaVel + deltaV, threadIndexInWarp);

			//zeroing mDeferredQstZ
			for (PxU32 ind = 0; ind < nbDofs; ++ind)
			{
				dofs[ind].mDeferredQstZ[threadIndexInWarp] = 0.f;
			}

			motionV += deltaV;

			storeSpatialVector(tLink.mScratchDeltaV, deltaV, threadIndexInWarp);

			//const PxTransform& tBody2World = poses[i];
			storeSpatialVector(tLink.mMotionVelocity, motionV, threadIndexInWarp);
			
			storeSpatialVector(tLink.mConstraintForces, preloadedConstraintForces + deltaV, threadIndexInWarp);

			dofs += nbDofs;
		}
	}
}

#endif

