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

#ifndef PXG_SOLVER_CORE_DESC_H
#define PXG_SOLVER_CORE_DESC_H

#include "PxgNarrowphaseCore.h"
#include "DyResidualAccumulator.h"

struct float4;

namespace physx
{
	namespace Cm
	{
		struct UnAlignedSpatialVector;
	}

	namespace Sc
	{
		class ShapeInteraction;
	}

	struct PxgConstraintData;
	struct PxgConstraintPrePrep;

	struct PxgBlockConstraint1DData;
	struct PxgBlockConstraint1DVelocities;
	struct PxgBlockConstraint1DParameters;
	struct PxgBlockContactData;
	struct PxgBlockContactPoint;
	struct PxgConstraint1DData;
	struct PxgConstraint1DVelocities;
	struct PxgConstraint1DParameters;

	struct PxgSolverBodyData;
	struct PxgSolverBodySleepData;
	struct PxgSolverTxIData;
	
	struct PxgIslandContext;
	struct PxgBodySim;
	struct PxgBodySimVelocities;
	class PxgArticulation;

	struct PxgSolverConstraintDesc;
	struct PxgBlockWorkUnit;

	struct PxgBlockConstraintBatch;

	struct PxgBlockFrictionPatch;
	struct PxgBlockFrictionAnchorPatch;

	struct PxgBlockSolverConstraint1DHeader;
	struct PxgBlockSolverConstraint1DCon;
	struct PxgBlockSolverConstraint1DMod;

	struct PxgTGSBlockSolverConstraint1DHeader;
	struct PxgTGSBlockSolverConstraint1DCon;
	
	struct PxgBlockSolverContactHeader;
	struct PxgBlockSolverFrictionHeader;
	struct PxgBlockSolverContactPoint;
	struct PxgBlockSolverContactFriction;
	struct PxgBlockFrictionIndex;

	struct PxgTGSBlockSolverContactHeader;
	struct PxgTGSBlockSolverFrictionHeader;
	struct PxgTGSBlockSolverContactPoint;
	struct PxgTGSBlockSolverContactFriction;

	struct PxgFrictionPatch;
	struct PxgFrictionAnchorPatch;

	struct PxgSolverConstraint1DHeader;
	struct PxgSolverConstraint1DCon;
	struct PxgSolverConstraint1DMod;

	struct PxgTGSSolverConstraint1DHeader;
	struct PxgTGSSolverConstraint1DCon;
	struct PxgTGSSolverConstraint1DMod;

	struct PxgSolverContactHeader;
	struct PxgSolverFrictionHeader;
	struct PxgSolverContactPointExt;
	struct PxgSolverContactFrictionExt;
	  
	struct PxContact;
	struct PxContactPatch;
	struct PxgD6JointData;
	struct PxgSolverReferences;
	struct PxFrictionPatch;

	struct PxsContactManagerOutput;
	struct PartitionIndexData;
	struct PartitionNodeData;
	struct PxgSolverConstraintManagerConstants;
	struct PxgConstraintBatchHeader;
	struct PxgConstraintWriteback;
	class PxAlignedTransform;
	struct Px1DConstraint;

	struct PxgTGSSolverContactHeader;
	struct PxgTGSSolverContactPointExt;
	struct PxgTGSSolverFrictionExt;

	struct PxgArticulationBlockResponse;

	struct PxsTorsionalFrictionData;

	namespace Dy
	{
		struct ThresholdStreamElement;
		class ThresholdStream;
	}

	struct IterativeSolveData
	{
		PxgBlockConstraintBatch*		blockConstraintBatch;
		PxgBlockSolverConstraint1DHeader* blockJointConstraintHeaders;
		PxgBlockSolverConstraint1DCon*	blockJointConstraintRowsCon;
		PxgBlockSolverConstraint1DMod*	blockJointConstraintRowsMod;

		PxgBlockSolverContactHeader*	blockContactHeaders;
		PxgBlockSolverFrictionHeader*	blockFrictionHeaders;
		PxgBlockSolverContactPoint*		blockContactPoints;
		PxgBlockSolverContactFriction*	blockFrictions;

		//first numSolverBodies float4s are linear velocity, last numSolverBodies float4s are angular velocity
		float4*							solverBodyVelPool; 
		float4*							tempStaticBodyOutputPool;

		// Each bit encodes the activation of a slab (32 bits). When there are more than 32 slabs, use multiple indices.
		// To query the reference count, count the number of active slabs/bits.
		PxU32*							solverEncodedReferenceCount;
		PxgSolverContactHeader*			contactHeaders;
		PxgSolverFrictionHeader*		frictionHeaders;
		PxgSolverContactPointExt*		contactPoints;
		PxgSolverContactFrictionExt*	frictions;

		PxgArticulationBlockResponse*	artiResponse;
	};

	struct IterativeSolveDataTGS
	{
		PxgBlockConstraintBatch*				blockConstraintBatch;
		PxgTGSBlockSolverConstraint1DHeader*	blockJointConstraintHeaders;
		PxgTGSBlockSolverConstraint1DCon*		blockJointConstraintRowsCon;
		PxgBlockSolverConstraint1DMod*			blockJointConstraintRowsMod;

		PxgTGSBlockSolverContactHeader*			blockContactHeaders;
		PxgTGSBlockSolverFrictionHeader*		blockFrictionHeaders;
		PxgTGSBlockSolverContactPoint*			blockContactPoints;
		PxgTGSBlockSolverContactFriction*		blockFrictions;

		//first numSolverBodies float4s are linear velocity, last numSolverBodies float4s are angular velocity
		float4*									solverBodyVelPool;
		float4*									tempStaticBodyOutputs;

		// Each bit encodes the activation of a slab (32 bits). When there are more than 32 slabs, use multiple indices.
		// To query the reference count, count the number of active slabs/bits.
		PxU32*									solverEncodedReferenceCount;
		
		PxgTGSSolverContactHeader*				contactHeaders;
		PxgSolverFrictionHeader*				frictionHeaders;	//Technically, not needed
		PxgTGSSolverContactPointExt*			contactPoints;

		PxgTGSSolverFrictionExt*				frictions;

		PxgArticulationBlockResponse*			artiResponse;
	};

	struct PxgSolverSharedDescBase
	{
		PxgBlockFrictionPatch*		blockCurrentFrictionPatches;
		PxgBlockFrictionPatch*		blockPreviousFrictionPatches;

		PxgFrictionPatch*			currentFrictionPatches;
		PxgFrictionPatch*			previousFrictionPatches;

		PxgBodySim*					mBodySimBufferDeviceData; //If the body is articulation, we will have a remap index to the articulation array
		PxgArticulation*			articulations;

		Cm::UnAlignedSpatialVector* articulationDeferredZ;
		PxU32*						articulationDirty;
		uint4*						articulationSlabMask;
		PxU32						deltaOutOffset;

		float						dt;
		float						stepDt;
		float						invDtF32;
		float						stepInvDtF32;

		float						lengthScale;
	};

	//this desc is shared by solve and prepare kernels
	template <typename IterData>
	struct PxgSolverSharedDesc : PxgSolverSharedDescBase
	{
		IterData	iterativeData;
	};

	struct PxgSolverCoreDesc
	{
		float4* outSolverVelocity;
		PxAlignedTransform* outBody2World;
		PxgSolverBodyData* solverBodyDataPool;
		PxgSolverTxIData* solverBodyTxIDataPool;
		PxgSolverBodySleepData* solverBodySleepDataPool;

		float4* outArtiVelocity;

		PxgIslandContext* islandContextPool;
		float4* motionVelocityArray; // first numSolverBodies float4s are linear velocity, last numSolverBodies float4s are angular velocity
		PxU32* constraintsPerPartition;
		PxU32* artiConstraintsPerPartition;
		Dy::ThresholdStreamElement* thresholdStream;
		Dy::ThresholdStreamElement* tmpThresholdStream;
		Dy::ThresholdStreamElement*	exceededForceElements;
		Dy::ThresholdStreamElement*	prevExceededForceElements;
		Dy::ThresholdStreamElement*	forceChangeThresholdElements; //this is store all pairs which will trigger force exceeded or lost events
		PxReal* thresholdStreamAccumulatedForce;
		PxReal* thresholdStreamAccumulatedForceBetweenBlocks;
		PxU32* thresholdStreamWriteIndex;
		PxU32* thresholdStreamWriteIndexBetweenBlocks;
		bool*  thresholdStreamWriteable;
		PxReal* accumulatedForceObjectPairs;

		PxgConstraintWriteback* constraintWriteBack; // 1D constraint write back
		PxF32* forceBuffer; // contact write back
		PxFrictionPatch* frictionPatches;

		PxU32* mRigidStaticContactCounts;
		PxU32* mRigidStaticContactStartIndices;

		PxU32* mRigidStaticJointCounts;
		PxU32* mRigidStaticJointStartIndices;

		PxgSolverReferences* solverBodyReferences;
		PxsContactManagerOutput* contactManagerOutputBase;
		PxgBodySim*	mBodySimBufferDeviceData;
		PxgBodySimVelocities* mBodySimPrevVelocitiesBufferDeviceData;

		PxU32 numIslands;	 
		PxU32 numBatches;
		PxU32 numArticBatches;
		PxU32 numSolverBodies;
		PxU32 numSlabs;
		PxU32 accumulatedBodyDeltaVOffset;
		
		PxI32 sharedThresholdStreamIndex;
		
		bool enableStabilization;
	
		PxU32 nbExceededThresholdElements;
		PxU32 nbPrevExceededThresholdElements;
		PxU32 nbForceChangeElements;

		PxU32 maxLinksPerArticulation;

		Dy::ErrorAccumulator contactErrorAccumulator;
	};

	struct PxgConstraintPrepareDesc
	{
		PxU32* jointConstraintBatchIndices;			//indices for joint batch
		PxU32* contactConstraintBatchIndices;		//indices for contact batch
		PxU32* artiJointConstraintBatchIndices;		//indices for articulation joint batch
		PxU32* artiContactConstraintBatchIndices;	//indices for articulation contact batch

		PxgSolverConstraintManagerConstants*	solverConstantData;
		PxgBlockConstraint1DData*				blockJointPrepPool;
		PxgBlockConstraint1DVelocities*			blockJointPrepPool0;
		PxgBlockConstraint1DParameters*			blockJointPrepPool1;

		PxgSolverBodyData*						solverBodyDataPool;
		PxgSolverTxIData*						solverBodyTxIDataPool;
		PxgBlockWorkUnit*						blockWorkUnit;

		PxgBlockFrictionIndex*					blockCurrentFrictionIndices;
		PxgBlockFrictionIndex*					blockPreviousFrictionIndices;

		PxgBlockContactData*					blockContactCurrentPrepPool;
		PxgBlockContactPoint*					blockContactPoints;

		PxgBlockFrictionAnchorPatch*			blockCurrentAnchorPatches;
		PxgBlockFrictionAnchorPatch*			blockPreviousAnchorPatches;

		////////////////////////////////////////////////////////////////////////////
		//for articulation
		PxgFrictionPatch*						currentFrictionPatches;
		PxgFrictionPatch*						previousFrictionPatches;

		PxgFrictionAnchorPatch*					currentAnchorPatches;
		PxgFrictionAnchorPatch*					previousAnchorPatches;

	/*	PxgConstraint1DData*					jointPrepPool;
		PxgConstraint1DVelocities*				jointPrepPool0;
		PxgConstraint1DParameters*				jointPrepPool1;*/

		//////////////////////////////////////////////////////////////////////////////

		PxAlignedTransform*						body2WorldPool;

		PxsContactManagerOutput* contactManagerOutputBase;

		PxU32* constraintUniqueIndices;
		PxU32* artiConstraintUniqueIndices;
		PxU32* artiContactUniqueIndices;

		PxU32 num1dConstraintBatches;
		PxU32 numContactBatches;

		PxU32 numStatic1dConstraintBatches;
		PxU32 numStaticContactBatches;

		PxU32 numArti1dConstraintBatches;
		PxU32 numArtiStatic1dConstraintBatches;
		PxU32 numArtiSelf1dConstraintBatches;
		PxU32 numArtiContactBatches;
		PxU32 numArtiStaticContactBatches;
		PxU32 numArtiSelfContactBatches;

		PxU32 totalBodyCount;
		
		PxU32 numBatches;
		PxU32 numStaticBatches;
	
		float bounceThresholdF32;
		float frictionOffsetThreshold;
		float correlationDistance;
		float ccdMaxSeparation;
		
		PxU32 totalPreviousEdges;
		PxU32 totalCurrentEdges;

		PxU32 articContactIndex;
		PxU32 articJointIndex;
		PxU32 nbElementsPerBody;

		PxReal biasCoefficient;
	};


	struct PxgPrePrepDesc
	{
		PxgBlockConstraintBatch*			blockBatches;
		PxU32								numBatches;
		PxU32								numStaticBatches;
		PxU32								numArtiBatches;
		PxU32								numArtiStaticBatches;
		PxU32								numArtiSelfBatches;
		PxU32								nbD6RigidJoints;
		PxU32								nbD6ArtiJoints;
		PxU32								nbTotalArtiJoints; //Only used for an assert

		PxU32								numTotalContacts;
		PxU32								numTotalConstraints;
		PxU32								numTotalStaticContacts;
		PxU32								numTotalStaticConstraints;

		PxU32								numTotalArtiContacts;		//dynamic contacts
		PxU32								numTotalArtiConstraints;	//external constraints
		PxU32								numTotalStaticArtiContacts;		//static contacts
		PxU32								numTotalStaticArtiConstraints;	//static constraints
		PxU32								numTotalSelfArtiContacts;		//static contacts
		PxU32								numTotalSelfArtiConstraints;	//static constraints

		PxU32								artiStaticConstraintBatchOffset;
		PxU32								artiStaticContactBatchOffset;

		PxgBlockWorkUnit*					blockWorkUnit;
		PxgBlockContactData*				blockContactData;						//GPU output data		
		PxgBlockContactPoint*				blockContactPoints;
		PxContact*							compressedContacts;
		PxContactPatch*						compressedPatches;
		PxU8*								forceBuffer;

		PxContact*							cpuCompressedContactsBase;
		PxContactPatch*						cpuCompressedPatchesBase;
		PxReal*								cpuForceBufferBase;

		PxgBlockConstraint1DData*			blockPrepData;						//GPU output data
		PxgBlockConstraint1DVelocities*		blockPrepVelocityData;				//GPU output data
		PxgBlockConstraint1DParameters*		blockPrepParameterData;				//GPU output data

		PxgConstraintData*					constraintData;						//GPU output/Input data for d6 joint, GPU input for cpu joint
		Px1DConstraint*						constraintRows;						//GPU output/Input joint row data  for d6 joint. GPU input for cpu joint

		PxgConstraintData*					artiConstraintData;					//GPU input data
		Px1DConstraint*						artiConstraintRows;					//GPU input joint row data

		const PxgD6JointData*				rigidJointData;						//GPU input data
		const PxgConstraintPrePrep*			rigidConstraintPrePrep;				//GPU input data

		const PxgD6JointData*				artiJointData;						//GPU input data
		const PxgConstraintPrePrep*			artiConstraintPrePrep;				//GPU input data

		PxsContactManagerOutput*			contactManagerOutputBase;

		PxU32								sharedJointRowIndex;
		PxU32								sharedFrictionConstraintIndex;
		PxU32								sharedContactConstraintIndex;
		PxU32								sharedArticulationResponseIndex;
		PxU32*								solverBodyIndices;

		PartitionIndexData*					mPartitionIndices;
		PxU32*								mPartitionstartBatchIndices;
		PxU32*								mPartitionArtiStartBatchIndices;
		PxU32*								mPartitionJointCounts;		
		PxU32*								mPartitionArtiJointCounts;

		PxU32*								prevFrictionPatchCount;
		PxU32*								currFrictionPatchCount;

		PxU32*								mNpOutputIndices;

		PxgSolverBodyData*					mSolverBodyData;

		PxU32								mCmOutputOffsets[GPU_BUCKET_ID::eCount];
		PartitionNodeData*					mPartitionNodeData;
		PxgSolverConstraintManagerConstants* mContactConstantData;

		PxgConstraintBatchHeader*			mBatchHeaders;
		PxU32*								mContactUniqueIndices;
		PxU32*								mConstraintUniqueIndices;

		PxU32*								mArtiConstraintUniqueIndices; //external constraints
		PxU32*								mArtiContactUniqueIndices; //dynamic contacts

		PxgSolverReferences*				mSolverBodyReferences;
		PxU32								mMaxConstraintPartitions;
		PxU32								mTotalSlabs;
		PxU32								mTotalActiveBodies;
		PxU32								mTotalActiveArticulations;
		PxU32								mActiveBodyStartOffset;
		PxU32								nbElementsPerBody;

		Sc::ShapeInteraction**				mShapeInteractions;
		PxReal*								mRestDistances;
		PxsTorsionalFrictionData*			mTorsionalFrictionData;

		//Static articulation contact data
		PxU32*								mArtiStaticContactIndices;
		PxU32*								mArtiStaticConstraintIndices;
		PxU32*								mArtiStaticContactCounts;
		PxU32*								mArtiStaticConstraintCounts;

		PxU32*								mArtiSelfContactIndices;
		PxU32*								mArtiSelfConstraintIndices;
		PxU32*								mArtiSelfContactCounts;
		PxU32*								mArtiSelfConstraintCounts;

		//Static rigid body contact data
		PxU32*								mRigidStaticContactIndices;
		PxU32*								mRigidStaticConstraintIndices;
		PxU32*								mRigidStaticContactCounts;
		PxU32*								mRigidStaticConstraintCounts;

		PxU32*								mRigidStaticContactStartIndices;
		PxU32*								mRigidStaticConstraintStartIndices;

		PxU32*								mTempContactUniqueIndices;
		PxU32*								mTempConstraintUniqueIndices;
		PxU32*								mTempContactBlockHeader;
		PxU32*								mTempConstraintBlockHeader;
	};
}

#endif