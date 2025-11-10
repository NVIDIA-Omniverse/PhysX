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

class WriteToDeferredQstZ
{
public:
	static __device__ void writeQstZ(PxgArticulationBlockDofData* PX_RESTRICT dofData, const PxU32 ind, const PxU32 threadIndexInWarp, const PxReal QstZ, const PxReal* providedQstZ) 
	{
		PX_UNUSED(providedQstZ);
		dofData[ind].mDeferredQstZ[threadIndexInWarp] += QstZ;
	}
};

class WriteToTempQstZ
{
public:
	static __device__ void writeQstZ(PxgArticulationBlockDofData* PX_RESTRICT dofData, const PxU32 ind, const PxU32 threadIndexInWarp, const PxReal QstZ, const PxReal* providedQstZ) 
	{
		PX_UNUSED(providedQstZ);
		dofData[ind].mTmpQstZ[threadIndexInWarp] += QstZ;
	}
};

class WriteToProvidedQstZ
{
public:
	static __device__ void writeQstZ(PxgArticulationBlockDofData* PX_RESTRICT dofData, const PxU32 ind, const PxU32 threadIndexInWarp, const PxReal QstZ, PxReal* providedQstZ) 
	{
		PX_UNUSED(dofData);
		PX_UNUSED(threadIndexInWarp);
		if(providedQstZ)
			providedQstZ[ind] += QstZ;
	}
};

class ReadFromDeferredQstZ
{
public:
	static __device__ PxReal readFromQstZ(const PxgArticulationBlockDofData* PX_RESTRICT dofData, const PxU32 ind, const PxU32 threadIndexInWarp, const PxReal* providedQstZ)
	{
		PX_UNUSED(providedQstZ);
		return dofData[ind].mDeferredQstZ[threadIndexInWarp];
	}
};

class ReadFromTempQstZ
{
public:
	static __device__ PxReal readFromQstZ(const PxgArticulationBlockDofData* PX_RESTRICT dofData, const PxU32 ind, const PxU32 threadIndexInWarp, const PxReal* providedQstZ)
	{
		PX_UNUSED(providedQstZ);
		return dofData[ind].mTmpQstZ[threadIndexInWarp];
	}
};

class ReadFromProvidedQstZ
{
public:
	static __device__ PxReal readFromQstZ(const PxgArticulationBlockDofData* PX_RESTRICT dofData, const PxU32 ind, const PxU32 threadIndexInWarp, const PxReal* providedQstZ)
	{
		PX_UNUSED(dofData);
		PX_UNUSED(threadIndexInWarp);
		return providedQstZ[ind];
	}
};

/**
\brief Propagate to the parent link 
a) a spatial impulse applied to a child link
b) a joint impulse applied to the child link's inbound joint.
The Mirtich equivalent is the equation for Y in Figure 5.7, page 141 but with a modification
to account for a joint impulse applied to the child link's inbound joint.
If the joint impulse is Q and the child link impulse is ZChildW then the parent link impulse has
the form:
YParentW = translateChildToParent{ ZChildW + (I * s) *(Q - s^T * ZChildW)/(s * I * s^T) }
Optionally accumulate [Q - S^T * ZChildW] because this can be useful to reuse when propagating
delta spatial velocity from parent link to child link.
\param[in] parentToChild is the vector from parent link to child link such that childLinkPos == parentLinkPos + childToParent
\param[in] Z is the link impulse to apply to the child link expressed in the world frame.
\param[in] dofCount is the number of dofs of the child link's incoming joint.
\param[in] threadIndexInWarp is index of the articulation in the warp.
\param[in] jointForce is an optional of array joint impulses or forces ({Q}) to apply to each dof of the inbound joint of the child link. 
If NULL is chosen, the jonit forces are assumed to be zero.
\param[in] jointForceMultiplier is a multiplier to be applied to each element of jointForce. This is useful when applying a force for a finite timestep
with the jointForceMultiplier acting as the duration of the timestep. A value of 1.0 for jointForceMultiplier corresponds to jointForceMultiplier representing 
an array of dof impulses.
\param[in] providedQstZ is an optional array that accumulates (Q - s^T * ZChildW) for the child link's inbound joint.
\note providedQstZ is ignored unless WriteToProvidedQstZ is specified as the template parameter.
\note If WriteToDeferredQstZ is the template parameter, the array PxgArticulationBlockDofData::mDeferredQstZ will be used to accumulate (Q - s^T * ZChildW)
\note If WriteToTempQstZ is the template parameter, the array PxgArticulationBlockDofData::mTempQstZ will be used to accumulate (Q - s^T * ZChildW)
\note If WriteToProvidedQstZ is the template parameter, the array providedQstZ will be used to accumulate (Q - s^T * ZChildW). The array providedQstZ is permitted to be NULL.
\note jointForce and providedQstZ are either NULL or have dofCount entries ie one entry for each dof of the joint.
\return The propagated spatial impulse in the world frame.
*/
template<class WriteQstZ>
__device__ PX_FORCE_INLINE Cm::UnAlignedSpatialVector propagateImpulseW(const PxVec3& parentToChild,
	PxgArticulationBlockDofData* PX_RESTRICT dofData, const Cm::UnAlignedSpatialVector& Z,
	const PxU32 dofCount, const PxU32 threadIndexInWarp,
	const PxReal* PX_RESTRICT jointForce = NULL, const PxReal jointForceMultiplier = 1.0f,
	PxReal* providedQstZ = NULL)
{
	Cm::UnAlignedSpatialVector temp = Z;

	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			Cm::UnAlignedSpatialVector sas = loadSpatialVector(dofData[ind].mWorldMotionMatrix, threadIndexInWarp);
			Cm::UnAlignedSpatialVector isInvD = loadSpatialVector(dofData[ind].mIsInvDW, threadIndexInWarp);
			const PxReal jf = (jointForce ? jointForce[ind] * jointForceMultiplier : 0.0f);
			const PxReal stZ = jf - sas.innerProduct(Z);
			WriteQstZ::writeQstZ(dofData, ind, threadIndexInWarp, stZ, providedQstZ);
			temp += isInvD * stZ;
		}
	}

	//parent space's spatial zero acceleration impulse
	return Dy::FeatherstoneArticulation::translateSpatialVector(parentToChild, temp);
}


/**
\brief Propagate to the parent link a spatial impulse applied to a child link.
\note This function performs the same role as propagateImpulseW but with the following differences:
1. The attributes isInvD and motionMatrix required for the computation are preloaded. 
2. It is assumed that zero force/impulse is applied to the joint dofs.
3. The optionally provided array qstZ will accumulate (Q - s^T * ZChildW)
\note: This function is particularly useful when computing the link impulse response matrix because we propagate 6 orthogonal impulses to the
same link, thereby reusing the same motionMatrix etc 6 times. 
\note When computing the link impulse response matrix there is no need to accumulate (Q - s^T * ZChildW) 
\note This function is the companion of propagateAccelerationW_everythingPreLoaded.
\return The propagated spatial impulse in the world frame.
*/
static __device__ Cm::UnAlignedSpatialVector propagateImpulseW_everythingPreLoaded(const Cm::UnAlignedSpatialVector* isInvD, const PxVec3& childToParent,
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
	return Dy::FeatherstoneArticulation::translateSpatialVector(childToParent, temp);
}

/**
/brief Propagate an acceleration (or velocity) from a parent link to a child link.
\param[in] parentToChild is the vector from parent link to child link such that childLinkPos == parentLinkPos + parentToChild
\param[in] invStISW is the Mirtich equivalent of 1/[S_i^T * I_i^A * S_i]
param[in] motionMatrixW is the Mirth equivalent of S_i of the child link's inbound joint.
param[in] parentLinkAccelerationW is the parent link acceleration (or velocity) expressed in the world frame.
\param[in] dofCount is the number of dofs on the child links' inbound joint.
\param[in] IsW is the Mirtich equvialent of I_i^A * S_i (== S_i^T * I_i^A)
\param[in] QMinusSTZ is the equivalent of Q_i - S_i^T * ZA_i in Mirtich notiation with 
	Q_i the joint force (or impulse) of the child link's inbound joint and ZA_i the child link 
	zero acceleration force (or impulse)
\return The spatial acceleration (or velocity) of the child link.
\note See Mirtich p121 and equations for propagating forces/applying accelerations and
	p141 for propagating velocities/applying impulses.
\note This function is particularly useful when computing the link impulse response matrix because we propagate 6 orthogonal impulses to the
same link, thereby reusing the same motionMatrix etc 6 times.
\note This function is the companion of propagateImpulseW_everythingPreLoaded
\note Call this function if 1) all necessary attributes are already loaded 2) Q-stZ is provided by the user.
*/
static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW_everythingPreLoaded(const PxVec3& parentToChild, const float3* invStISW,
	const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::UnAlignedSpatialVector& parentLinkAccelerationW, const PxU32 dofCount,
	const Cm::UnAlignedSpatialVector* IsW, const PxReal* QMinusSTZ)
{
	Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-parentToChild, parentLinkAccelerationW); //parent velocity change

	//Convert parent velocity change into an impulse
	PxReal tJointDelta[3] = { 0.f, 0.f, 0.f };
#pragma unroll 3
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if (ind < dofCount)
		{
			//stI * pAcceleration
			const PxReal temp = IsW[ind].innerProduct(pDeltaV);

			tJointDelta[ind] = (QMinusSTZ[ind] - temp);
		}
	}

	assert(dofCount<=3);
	for (PxU32 ind = 0; ind < 3; ++ind)
	{
		if(ind<dofCount)
		{
			const float3 iStIsTi = invStISW[ind];

			const PxReal jDelta = iStIsTi.x * tJointDelta[0]
								+ iStIsTi.y * tJointDelta[1]
								+ iStIsTi.z * tJointDelta[2];

			pDeltaV += motionMatrix[ind] * jDelta;
		}
	}

	return pDeltaV;
}

/**
/brief Propagate an acceleration (or velocity) from a parent link to a child link.
\note This function performs the same role as propagateAccelerationW_everythingPreLoaded but with the following differences:
1) The parameters invStISW and motionMatrixW have not been preloaded and therefore will be loaded afresh by this function.
2) The accumulated QMinusSTZ may be read from PxgArticulationBlockDofData::mDeferredQstZ or PxgArticulationBlockDofData::mTmpQstZ
or providedQMinusSTZ by selecting the corresponding template parameter.
3) The delta to the joint dof speeds arising may be optionally recorded in the provided array jointDeltaDofSpeeds.
\note Call this function if
1) parentToChild is already loaded but all other necessary attributes still need to be loaded.
) Q-stZ is to be loaded from either mDeferredQstZ, mTmpQstZ or user-provided array providedQstZ.
3) It is required that PxgArticulationBlockDofData::mJointVelocities is *not* to be modified as a consequence of the propagation.
*/
template<class ReadFromQstZ>
static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW_child2ParentPreLoaded(const PxVec3& parentToChild,
	const PxgArticulationBlockDofData* PX_RESTRICT dofData,
	const Cm::UnAlignedSpatialVector& parentLinkAccelerationW,
	const PxU32 dofCount, const PxU32 threadIndexInWarp, const PxReal* providedQMinusSTZ, PxReal* jointDeltaDofSpeeds)
{
	const Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(-parentToChild, parentLinkAccelerationW); //parent velocity change

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

			QSTZMinusISDotTranslatedParentDeltaV[ind] = ReadFromQstZ::readFromQstZ(dofData, ind, threadIndexInWarp, providedQMinusSTZ)  - temp;
		}
	}

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

	return pDeltaV + jointSpatialDeltaV;
}

/**
/brief Propagate an acceleration (or velocity) from a parent link to a child link.
\note This function performs the same role as propagateAccelerationW_everythingPreLoaded but with the following differences:
1) None of the necessary parameters have been pre-loaded (including parentToChild) and will therefore be loaded afresh by this function.
2) The accumulated QMinusSTZ will be read from PxgArticulationBlockDofData::mDeferredQstZ
3) PxgArticulationBlockDofData::mJointVelocities will be overwritten with the delta to the joint dof velocities.
\note Call this function if 
1) none of the necessary link and dof attributes are loaded.
2) it is required that PxgArticulationBlockDofData::mJointVelocities is to be updated with the speed deltas of the joint dofs arising 
from the propagation.
\note This function is particularly useful when it is required to update the joint dof velocities either after 
a solver step with TGS or a sim step with PGS.
*/
static __device__ Cm::UnAlignedSpatialVector propagateAccelerationW_nothingpreloaded_updateJointDofVels(
	PxgArticulationBlockLinkData& linkData,
	PxgArticulationBlockDofData* dofData,
	const PxU32 dofCount,
	const Cm::UnAlignedSpatialVector& parentLinkAccelerationW,
	const PxU32 threadIndexInWarp)
{
	const float parentToChildX = linkData.mRw_x[threadIndexInWarp];
	const float parentToChildY = linkData.mRw_y[threadIndexInWarp];
	const float parentToChildZ = linkData.mRw_z[threadIndexInWarp];

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

	Cm::UnAlignedSpatialVector pDeltaV = Dy::FeatherstoneArticulation::translateSpatialVector(PxVec3(-parentToChildX, -parentToChildY, -parentToChildZ), parentLinkAccelerationW); //parent velocity change
	
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

			deltaV = propagateAccelerationW_nothingpreloaded_updateJointDofVels(tLink, dofs, nbDofs, deltaV, threadIndexInWarp);

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

