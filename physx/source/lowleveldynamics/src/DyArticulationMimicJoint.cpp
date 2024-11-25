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
   
#include "DyFeatherstoneArticulation.h"
#include "DyArticulationMimicJointCore.h"
#include "DyCpuGpuArticulation.h"

namespace physx
{
namespace Dy
{

/**
\brief Compute the deltaQDot response of a joint dof to a unit joint impulse applied to that joint dof.
\param[in] linkIndex specifies the index of the child link of the joint under consideration.
\param[in] dof is the joint dof that will receive the test impulse.
\param[in] artData contains pre-computed values that will be used in the computation of the joint response.
\return The deltaQDot response of the specified joint and dof.
\note dof is in range (0,3) because articulation joints only support 3 degrees of freedom.
*/
PX_INLINE PxReal computeMimicJointSelfResponse(const PxU32 linkIndex, const PxU32 dof, const ArticulationData& artData)
{			
	const ArticulationLink* links = artData.getLinks();

	const PxU32 parentLinkIndex = links[linkIndex].parent;

	//childLinkPos - parentLinkPos
	const PxVec3& parentLinkToChildLink = artData.getRw(linkIndex);	

	const PxU32 jointOffset = artData.getJointData(linkIndex).jointOffset;
	const PxU8 dofCount = artData.getJointData(linkIndex).nbDof;

	const PxReal testJointImpulses[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	const PxReal* testJointImpulse = testJointImpulses[dof];

	//(1) Propagate joint impulse (and zero link impulse) to parent
	PxReal QMinusStZ[3] = { 0.f, 0.f, 0.f };
	const Cm::UnAlignedSpatialVector* motionMatrixW = artData.getWorldMotionMatrix();
	const Cm::SpatialVectorF* IsInvStISW = artData.getISInvStIS();	
	const Cm::SpatialVectorF Zp = propagateImpulseW(
		parentLinkToChildLink,
		Cm::SpatialVectorF(PxVec3(0, 0,0), PxVec3(0, 0, 0)), 
		testJointImpulse, &IsInvStISW[jointOffset], &motionMatrixW[jointOffset], dofCount, 
		QMinusStZ);				

	//(2) Get deltaV response for parent
	const TestImpulseResponse* linkImpulseResponses = artData.getImpulseResponseMatrixWorld();
	const Cm::SpatialVectorF deltaVParent = -linkImpulseResponses[parentLinkIndex].getLinkDeltaVImpulseResponse(Zp);

	//(3) Propagate parent deltaV and apply test impulse (encoded in QMinusStZ).
	PxReal jointDeltaQDot[3]= {0, 0, 0};
	const InvStIs* invStIS = artData.getInvStIS();
	const Cm::SpatialVectorF* ISW = artData.getIsW();
	const Cm::SpatialVectorF deltaVChild = 
		propagateAccelerationW(
			parentLinkToChildLink, deltaVParent, 
			invStIS[linkIndex], &motionMatrixW[jointOffset], &ISW[jointOffset], QMinusStZ, dofCount, 
			jointDeltaQDot);

	const PxReal jointSelfResponse = jointDeltaQDot[dof];
	return jointSelfResponse;
}

/**
\brief Compute the deltaQDot response of a joint dof given a unit impulse applied to a different joint and dof.
\param[in] linkA is the link whose inbound joint receives the test impulse.
\param[in] dofA is the relevant dof of the inbound joint of linkA.
\param[in] linkB is the link whose inbound joint receives the deltaQDot arising from the unit impulse applied to the inbound joint of linkA.
\param[in] dofB is the relevant dof of the the inbound joint of linkB.
\param[in] artData contains pre-computed values that will be used in the computation of the joint response.
\param[in] QMinusSTZ is used to cache Q - S^T*Z for each dof encountered when propagating from linkA to root.
\param[in] QMinusStZLength is the length of the QMinusSTZ array ie the number of joint dofs of the articulation.
\return The deltaQDot response of the specified joint and dof corresponding to linkB and dofB.
\note dofA and dofB are in range (0,3) because articulation joints only support 3 degrees of freedom.
*/
PX_INLINE PxReal computeMimicJointCrossResponse
(const PxU32 linkA, const PxU32 dofA, const PxU32 linkB, const PxU32 dofB, const ArticulationData& artData, 
 PxReal* QMinusSTZ, const PxU32 QMinusStZLength)
{
	const ArticulationLink* links = artData.getLinks();

	//Compute the test impulse to apply the inbound joint of linkA.
	const PxReal testJointImpulses[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	const PxReal* testJointImpulse = testJointImpulses[dofA];
	const Cm::SpatialVectorF testLinkImpulse(PxVec3(0, 0, 0), PxVec3(0, 0, 0));

	//Zero QMinusSTZ before using it.
	PxMemZero(QMinusSTZ, sizeof(PxReal) * QMinusStZLength);

	//Get the path from linkA to root to linkB.
	const PxU32* pathToRootElements	= artData.getPathToRootElements();
	const PxU32 numFromRootToLink = links[linkA].mPathToRootCount;
	const PxU32* pathFromRootToLink = &pathToRootElements[links[linkA].mPathToRootStartIndex];
	const PxU32 numFromRootToOtherLink = links[linkB].mPathToRootCount;
	const PxU32* pathFromRootToOtherLink = &pathToRootElements[links[linkB].mPathToRootStartIndex];
			
	const Cm::UnAlignedSpatialVector* motionMatrixW = artData.getWorldMotionMatrix();
	const Cm::SpatialVectorF* ISInvStISW = artData.getISInvStIS();	
	const InvStIs* InvStISW = artData.getInvStIS();
	const Cm::SpatialVectorF* ISW = artData.getIsW();

	// Propagate test joint impulse from inbound joint of start link to its parent link.
	// This generates a link impulse that we can propagate to root.
	Cm::SpatialVectorF Zp;
	{					
		const PxU32 linkIndex = pathFromRootToLink[numFromRootToLink - 1];
		PX_ASSERT(linkA == linkIndex);

		//childLinkPos - parentLinkPos
		const PxVec3& parentLinkToChildLink = artData.getRw(linkIndex);	

		const PxU32 jointOffset = artData.getJointData(linkIndex).jointOffset;
		const PxU8 dofCount = artData.getJointData(linkIndex).nbDof;

		Zp = propagateImpulseW(
				parentLinkToChildLink,
				testLinkImpulse,
				testJointImpulse, &ISInvStISW[jointOffset], &motionMatrixW[jointOffset], dofCount, 
				&QMinusSTZ[jointOffset]);				
	}
				
	//Now propagate the link impulse to root.
	//An optimisation would be to propagate the impulse to the common ancestor of link A and link B.
	//Let's keep it simple for the time being.
	for(PxU32 k = 1; k < numFromRootToLink; k++)
	{
		const PxU32 linkIndex = pathFromRootToLink[numFromRootToLink - 1 - k];

		//childLinkPos - parentLinkPos
		const PxVec3& parentLinkToChildLink = artData.getRw(linkIndex);

		const PxU32 jointOffset = artData.getJointData(linkIndex).jointOffset;
		const PxU8 dofCount = artData.getJointData(linkIndex).nbDof;

		//(1) Propagate link impulse (and zero joint impulse) to parent
		Zp = propagateImpulseW(
				parentLinkToChildLink,
				Zp,
				NULL, &ISInvStISW[jointOffset], &motionMatrixW[jointOffset], dofCount, 
				&QMinusSTZ[jointOffset]);				
	}

	//We can now get the deltaV on the root link.
	const TestImpulseResponse* linkImpulseResponses = artData.getImpulseResponseMatrixWorld();
	Cm::SpatialVectorF deltaVParent = -linkImpulseResponses[0].getLinkDeltaVImpulseResponse(Zp);

	//Propagate deltaV down the tree to linkB.
	//We only care about jointVelocity of the inbound joint of linkB.
	PxReal jointVelocity[3] = {0, 0, 0};
	for(PxU32 k = 0; k < numFromRootToOtherLink; k++)
	{
		const PxU32 linkIndex = pathFromRootToOtherLink[k];
		PX_ASSERT((0 != k) ||( 0 == links[linkIndex].parent));

		//childLinkPos - parentLinkPos
		const PxVec3& parentToChild = artData.getRw(linkIndex);

		const PxU32 jointOffset = artData.getJointData(linkIndex).jointOffset;
		const PxU8 dofCount = artData.getJointData(linkIndex).nbDof;

		//Compute the jointVelocity only when we reach linkB.
		PxReal* jointVelocityToUse = ((numFromRootToOtherLink - 1) == k) ? jointVelocity : NULL;

		deltaVParent = propagateAccelerationW(
			parentToChild, deltaVParent, 
			InvStISW[linkIndex], &motionMatrixW[jointOffset], &ISW[jointOffset], &QMinusSTZ[jointOffset], dofCount, 
			jointVelocityToUse); 
	}

	//Zero QMinusSTZ after using it.
	PxMemZero(QMinusSTZ, sizeof(PxReal) * QMinusStZLength);

	//Now pick out the dof associated with joint B.
	const PxReal r = jointVelocity[dofB];
	return r;
}

void setupMimicJointInternal
(const ArticulationMimicJointCore& mimicJointCore, const ArticulationData& artData, 
 PxReal* scratchBufferQMinusStZ, const PxU32 scratchBufferQMinusStZLength, 
 ArticulationInternalMimicJoint& mimicJointInternal)
{
	//The coupled joints are the inbound joints of link0 and link1.
	const PxU32 linkA = mimicJointCore.linkA;
	const PxU32 linkB = mimicJointCore.linkB;
 
	//The dof indices of the coupled joints are computed from the axes.
	const PxU32 dofA = artData.getLink(linkA).inboundJoint->invDofIds[mimicJointCore.axisA];
	const PxU32 dofB = artData.getLink(linkB).inboundJoint->invDofIds[mimicJointCore.axisB];

	//Compute all 4 response terms.
	const PxReal rAA = computeMimicJointSelfResponse(linkA, dofA, artData);
	const PxReal rBB = computeMimicJointSelfResponse(linkB, dofB, artData);
	const PxReal rBA = computeMimicJointCrossResponse(linkA, dofA, linkB, dofB, artData, scratchBufferQMinusStZ, scratchBufferQMinusStZLength);
	const PxReal rAB = computeMimicJointCrossResponse(linkB, dofB, linkA, dofA, artData, scratchBufferQMinusStZ, scratchBufferQMinusStZLength);

	//Combine all 4 response terms to compute (J * M^-1 * J^T)
	const PxReal gearRatio = mimicJointCore.gearRatio;
	const PxReal recipEffectiveInertia = computeRecipMimicJointEffectiveInertia(rAA, rAB, rBB, rBA, gearRatio);

	//Set everything we now know about the mimic joint.
	mimicJointInternal.gearRatio = mimicJointCore.gearRatio;
	mimicJointInternal.offset = mimicJointCore.offset;
	mimicJointInternal.naturalFrequency = mimicJointCore.naturalFrequency;
	mimicJointInternal.dampingRatio = mimicJointCore.dampingRatio;
	mimicJointInternal.linkA = mimicJointCore.linkA;
	mimicJointInternal.linkB = mimicJointCore.linkB;
	mimicJointInternal.dofA = dofA;
	mimicJointInternal.dofB = dofB;
	mimicJointInternal.recipEffectiveInertia = recipEffectiveInertia;
}

void FeatherstoneArticulation::setupInternalMimicJointConstraints()
{
	//Prepare the mimic joints for the solver.
	//We need an array {Q - S^T*Z} when computing the mimic joint response terms.
	//We should be safe to use mDeferredQstZ here because we are pre-solver.
	//Just make sure that we zero it again before exiting so that it is zero
	//when we get to the solver.
	mArticulationData.mInternalMimicJoints.reserve(mArticulationData.mNbMimicJoints);
	mArticulationData.mInternalMimicJoints.forceSize_Unsafe(mArticulationData.mNbMimicJoints);
	for(PxU32 i = 0; i < mArticulationData.mNbMimicJoints; i++)
	{
		const ArticulationMimicJointCore& mimicJointCore = *mArticulationData.mMimicJoints[i];
		ArticulationInternalMimicJoint& mimicJointInternal = mArticulationData.mInternalMimicJoints[i];
		setupMimicJointInternal(
			mimicJointCore, 
			mArticulationData,
			mArticulationData.mDeferredQstZ.begin(), mArticulationData.mDeferredQstZ.size(),
			mimicJointInternal);
	}//nbMimicJoints							
}

void FeatherstoneArticulation::solveInternalMimicJointConstraints(const PxReal dt, const PxReal invDt, const bool velocityIteration, const bool isTGS, const PxReal biasCoefficient)
{
	PX_UNUSED(dt);
	PX_UNUSED(isTGS);
		
	for(PxU32 i = 0; i < mArticulationData.mNbMimicJoints; i++)
	{
		const ArticulationInternalMimicJoint& internalMimicJoint = mArticulationData.mInternalMimicJoints[i];

		//Get the gearing ratio and offset.
		const PxReal gearRatio = internalMimicJoint.gearRatio;
		const PxReal offset = internalMimicJoint.offset;

		//Get the compliance of the mimic joint
		const PxReal naturalFrequency = internalMimicJoint.naturalFrequency;
		const PxReal dampingRatio = internalMimicJoint.dampingRatio;

		//Get the responses of the mimic joint.
		const PxReal mimicJointRecipEffectiveInertia = internalMimicJoint.recipEffectiveInertia;

		//Get the dofs involved in the mimic joint.
		//We need these to work out the joint dof speeds and positions.
		const PxU32 linkA = internalMimicJoint.linkA;
		const PxU32 dofA = internalMimicJoint.dofA;
		const PxU32 linkB = internalMimicJoint.linkB;
		const PxU32 dofB = internalMimicJoint.dofB;

		//Get the positions of the joint dofs coupled by the mimic joint.
		const PxU32 jointOffsetA = mArticulationData.mLinks[linkA].inboundJoint->jointOffset;
		const PxU32 jointOffsetB = mArticulationData.mLinks[linkB].inboundJoint->jointOffset;
		const PxReal qA = mArticulationData.mJointPosition[jointOffsetA + dofA];
		const PxReal qB = mArticulationData.mJointPosition[jointOffsetB + dofB];

		//Get the speeds of the joint dofs coupled by the mimic joint.
		PxReal qADot = 0;
		PxReal qBDot = 0;
		{
			PxReal jointDofSpeedsA[3] = {0, 0, 0};
			pxcFsGetVelocity(linkA, jointDofSpeedsA);
			PxReal jointDofSpeedsB[3] = {0, 0, 0};
			pxcFsGetVelocity(linkB, jointDofSpeedsB);
			qADot = jointDofSpeedsA[dofA];
			qBDot = jointDofSpeedsB[dofB];
		}

		//We can now compute the joint impulses to apply to the inbound joints of links A and B.
		PxReal jointImpulseA[3] = {0, 0, 0};
		PxReal jointImpulseB[3] = {0, 0, 0};
		{			
			PxReal jointDofImpA = 0;
			PxReal jointdofImpB = 0;
			computeMimicJointImpulses(
				biasCoefficient, dt, invDt, 
				qA, qB, qADot, qBDot, 
				gearRatio, offset, 
				naturalFrequency, dampingRatio, 
				mimicJointRecipEffectiveInertia,
				velocityIteration,
				jointDofImpA, jointdofImpB);
			jointImpulseA[dofA] = jointDofImpA;
			jointImpulseB[dofB] = jointdofImpB;
		}

		//Apply the joint impulses from link to root.
		//This will accumulate deferredQMinusSTZ for each encountered joint dof 
		//and deferredZ for the root link.
		const aos::Vec3V zeroImpulse = aos::V3Zero();
		pxcFsApplyImpulses(
			linkA, zeroImpulse, zeroImpulse, jointImpulseA,
			linkB, zeroImpulse, zeroImpulse, jointImpulseB);
	}

}


} //namespace Dy
} //namespace physx