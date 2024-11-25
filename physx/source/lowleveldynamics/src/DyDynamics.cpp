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

#include "foundation/PxTime.h"
#include "foundation/PxAtomic.h"
#include "PxvDynamics.h"

#include "common/PxProfileZone.h"
#include "PxsRigidBody.h"
#include "PxsContactManager.h"
#include "DyDynamics.h"
#include "DyBodyCoreIntegrator.h"
#include "DySolverCore.h"
#include "DySolverControl.h"
#include "DySolverContact.h"
#include "DySolverContactPF.h"
#include "DyArticulationContactPrep.h"
#include "DySolverBody.h"

#include "DyConstraintPrep.h"
#include "DyConstraintPartition.h"

#include "CmFlushPool.h"
#include "DyArticulationPImpl.h"
#include "DyFeatherstoneArticulation.h"
#include "PxsMaterialManager.h"
#include "DySolverContactPF4.h"
#include "DyContactReduction.h"
#include "PxcNpContactPrepShared.h"
#include "DyContactPrep.h"
#include "DySolverControlPF.h"
#include "PxSceneDesc.h"
#include "PxsSimpleIslandManager.h"
#include "PxvNphaseImplementationContext.h"
#include "PxvSimStats.h"
#include "PxsContactManagerState.h"
#include "DyContactPrepShared.h"
#include "DySleep.h"
#include "DyIslandManager.h"

//KS - used to turn on/off batched SIMD constraints.
#define DY_BATCH_CONSTRAINTS 1
//KS - used to specifically turn on/off batches 1D SIMD constraints.
#define DY_BATCH_1D 1

namespace physx
{
namespace Dy
{
// PT: TODO: what is const and what is not?
struct SolverIslandObjects
{
	PxsRigidBody**				bodies;	
	FeatherstoneArticulation**	articulations;
	PxsIndexedContactManager*	contactManagers;	// PT: points to DynamicsContext::mContactList

	const IG::IslandId*			islandIds;
	PxU32						numIslands;
	PxU32*						bodyRemapTable;
	PxU32*						nodeIndexArray;

	PxSolverConstraintDesc*		constraintDescs;
	PxSolverConstraintDesc*		orderedConstraintDescs;
	PxSolverConstraintDesc*		tempConstraintDescs;
	PxConstraintBatchHeader*	constraintBatchHeaders;
	Cm::SpatialVector*			motionVelocities;
	PxsBodyCore**				bodyCoreArray;

	SolverIslandObjects() : bodies(NULL), articulations(NULL), 
		contactManagers(NULL), islandIds(NULL), numIslands(0), nodeIndexArray(NULL), constraintDescs(NULL), orderedConstraintDescs(NULL), 
		tempConstraintDescs(NULL), constraintBatchHeaders(NULL), motionVelocities(NULL), bodyCoreArray(NULL)
	{
	}
};

Context* createDynamicsContext(	PxcNpMemBlockPool* memBlockPool, PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
								PxvSimStats& simStats, PxTaskManager* taskManager, PxVirtualAllocatorCallback* allocatorCallback, 
								PxsMaterialManager* materialManager, IG::SimpleIslandManager& islandManager, PxU64 contextID,
								bool enableStabilization, bool useEnhancedDeterminism,
								PxReal maxBiasCoefficient, bool frictionEveryIteration, PxReal lengthScale, bool isResidualReportingEnabled)
{
	return PX_NEW(DynamicsContext)(	memBlockPool, scratchAllocator, taskPool, simStats, taskManager, allocatorCallback, materialManager, islandManager, contextID,
									enableStabilization, useEnhancedDeterminism, maxBiasCoefficient, frictionEveryIteration, lengthScale, isResidualReportingEnabled);
}

void DynamicsContext::destroy()
{
	this->~DynamicsContext();
	PX_FREE_THIS;
}

void DynamicsContext::resetThreadContexts()
{
	PxcThreadCoherentCacheIterator<ThreadContext, PxcNpMemBlockPool> threadContextIt(mThreadContextPool);
	ThreadContext* threadContext = threadContextIt.getNext();

	while(threadContext != NULL)
	{
		threadContext->reset();
		threadContext = threadContextIt.getNext();
	}
}

// =========================== Basic methods

DynamicsContext::DynamicsContext(	PxcNpMemBlockPool* memBlockPool,
									PxcScratchAllocator& scratchAllocator,
									Cm::FlushPool& taskPool,
									PxvSimStats& simStats,
									PxTaskManager* taskManager,
									PxVirtualAllocatorCallback* allocatorCallback,
									PxsMaterialManager* materialManager,
									IG::SimpleIslandManager& islandManager,
									PxU64 contextID,
									bool enableStabilization,
									bool useEnhancedDeterminism,
									PxReal maxBiasCoefficient,
									bool frictionEveryIteration,
									PxReal lengthScale,
									bool isResidualReportingEnabled) :
	Dy::Context			(islandManager, allocatorCallback, simStats, enableStabilization, useEnhancedDeterminism, maxBiasCoefficient, lengthScale, contextID, isResidualReportingEnabled),
	// PT: TODO: would make sense to move all the following members to the base class but include paths get in the way atm
	mThreadContextPool	(memBlockPool),
	mMaterialManager	(materialManager),
	mScratchAllocator	(scratchAllocator),
	mTaskPool			(taskPool),
	mTaskManager		(taskManager)
{
	createThresholdStream(*allocatorCallback);
	createForceChangeThresholdStream(*allocatorCallback);
	mExceededForceThresholdStream[0] = PX_NEW(ThresholdStream)(*allocatorCallback);
	mExceededForceThresholdStream[1] = PX_NEW(ThresholdStream)(*allocatorCallback);
	mThresholdStreamOut = 0;
	mCurrentIndex = 0;

	mWorldSolverBody.linearVelocity = PxVec3(0);
	mWorldSolverBody.angularState = PxVec3(0);
	mWorldSolverBodyData.invMass = 0;
	mWorldSolverBodyData.sqrtInvInertia = PxMat33(PxZero);
	mWorldSolverBodyData.nodeIndex = PX_INVALID_NODE;
	mWorldSolverBodyData.reportThreshold = PX_MAX_REAL;
	mWorldSolverBodyData.penBiasClamp = -PX_MAX_REAL;
	mWorldSolverBodyData.maxContactImpulse = PX_MAX_REAL;
	mWorldSolverBody.solverProgress=MAX_PERMITTED_SOLVER_PROGRESS;
	mWorldSolverBody.maxSolverNormalProgress=MAX_PERMITTED_SOLVER_PROGRESS;
	mWorldSolverBody.maxSolverFrictionProgress=MAX_PERMITTED_SOLVER_PROGRESS;
	mWorldSolverBodyData.linearVelocity = mWorldSolverBodyData.angularVelocity = PxVec3(0.f);
	mWorldSolverBodyData.body2World = PxTransform(PxIdentity);
	mSolverCore[PxFrictionType::ePATCH] = PX_NEW(SolverCoreGeneral)(frictionEveryIteration);
	mSolverCore[PxFrictionType::eONE_DIRECTIONAL] = PX_NEW(SolverCoreGeneralPF);
	mSolverCore[PxFrictionType::eTWO_DIRECTIONAL] = PX_NEW(SolverCoreGeneralPF);
}

DynamicsContext::~DynamicsContext()
{
	for(PxU32 i = 0; i < PxFrictionType::eFRICTION_COUNT; ++i)
	{
		PX_DELETE(mSolverCore[i]);
	}

	PX_DELETE(mExceededForceThresholdStream[1]);
	PX_DELETE(mExceededForceThresholdStream[0]);
}

#if PX_ENABLE_SIM_STATS
void DynamicsContext::addThreadStats(const ThreadContext::ThreadSimStats& stats)
{
	mSimStats.mNbActiveConstraints += stats.numActiveConstraints;
	mSimStats.mNbActiveDynamicBodies += stats.numActiveDynamicBodies;
	mSimStats.mNbActiveKinematicBodies += stats.numActiveKinematicBodies;
	mSimStats.mNbAxisSolverConstraints += stats.numAxisSolverConstraints;
}
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

// =========================== Solve methods!

void DynamicsContext::setDescFromIndices(PxSolverConstraintDesc& desc, const IG::IslandSim& islandSim, const PxsIndexedInteraction& constraint, PxU32 solverBodyOffset)
{
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eBODY == 0);
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eKINEMATIC == 1);
	const PxU32 offsetMap[] = {solverBodyOffset, 0};
	//const PxU32 offsetMap[] = {mKinematicCount, 0};

	if(constraint.indexType0 == PxsIndexedInteraction::eARTICULATION)
	{
		const PxNodeIndex& nodeIndex0 = reinterpret_cast<const PxNodeIndex&>(constraint.articulation0);
		const IG::Node& node0 = islandSim.getNode(nodeIndex0);
		desc.articulationA = node0.mObject;
		desc.linkIndexA = nodeIndex0.articulationLinkId();
	}
	else
	{
		desc.linkIndexA = PxSolverConstraintDesc::RIGID_BODY;
		//desc.articulationALength = 0; //this is unioned with bodyADataIndex
		/*desc.bodyA = constraint.indexType0 == PxsIndexedInteraction::eWORLD ? &mWorldSolverBody
																			: &mSolverBodyPool[(PxU32)constraint.solverBody0 + offsetMap[constraint.indexType0]];
		desc.bodyADataIndex = PxU16(constraint.indexType0 == PxsIndexedInteraction::eWORLD ? 0
																			: (PxU16)constraint.solverBody0 + 1 + offsetMap[constraint.indexType0]);*/

		desc.bodyA = constraint.indexType0 == PxsIndexedInteraction::eWORLD ? &mWorldSolverBody
																			: &mSolverBodyPool[PxU32(constraint.solverBody0) + offsetMap[constraint.indexType0]];
		desc.bodyADataIndex = constraint.indexType0 == PxsIndexedInteraction::eWORLD ? 0
																			: PxU32(constraint.solverBody0) + 1 + offsetMap[constraint.indexType0];
	}

	if(constraint.indexType1 == PxsIndexedInteraction::eARTICULATION)
	{
		const PxNodeIndex& nodeIndex1 = reinterpret_cast<const PxNodeIndex&>(constraint.articulation1);
		const IG::Node& node1 = islandSim.getNode(nodeIndex1);
		desc.articulationB = node1.mObject;
		desc.linkIndexB = nodeIndex1.articulationLinkId();// PxTo8(getLinkIndex(constraint.articulation1));
	}
	else
	{
		desc.linkIndexB = PxSolverConstraintDesc::RIGID_BODY;
		//desc.articulationBLength = 0; //this is unioned with bodyBDataIndex
		desc.bodyB = constraint.indexType1 == PxsIndexedInteraction::eWORLD ? &mWorldSolverBody
																			: &mSolverBodyPool[PxU32(constraint.solverBody1) + offsetMap[constraint.indexType1]];
		desc.bodyBDataIndex = constraint.indexType1 == PxsIndexedInteraction::eWORLD ? 0
																			: PxU32(constraint.solverBody1) + 1 + offsetMap[constraint.indexType1];
	}
}

void DynamicsContext::setDescFromIndices(PxSolverConstraintDesc& desc, IG::EdgeIndex edgeIndex, const IG::SimpleIslandManager& islandManager, PxU32* bodyRemap, PxU32 solverBodyOffset)
{
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eBODY == 0);
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eKINEMATIC == 1);

	const IG::IslandSim& islandSim = islandManager.getAccurateIslandSim();

	const PxNodeIndex node1 = islandSim.mCpuData.getNodeIndex1(edgeIndex);
	if (node1.isStaticBody())
	{
		desc.bodyA = &mWorldSolverBody;
		desc.bodyADataIndex = 0;
		desc.linkIndexA = PxSolverConstraintDesc::RIGID_BODY;
	}
	else
	{
		const IG::Node& node = islandSim.getNode(node1);
		if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
		{
			Dy::FeatherstoneArticulation* a = getArticulationFromIG(islandSim, node1);

			desc.articulationA = a;
			desc.linkIndexA = node1.articulationLinkId();
		}
		else
		{
			const PxU32 activeIndex = islandSim.getActiveNodeIndex(node1);
			const PxU32 index = node.isKinematic() ? activeIndex : bodyRemap[activeIndex] + solverBodyOffset;
			desc.bodyA = &mSolverBodyPool[index];
			desc.bodyADataIndex = index + 1;
			desc.linkIndexA = PxSolverConstraintDesc::RIGID_BODY;
		}
	}

	const PxNodeIndex node2 = islandSim.mCpuData.getNodeIndex2(edgeIndex);
	if (node2.isStaticBody())
	{
		desc.bodyB = &mWorldSolverBody;
		desc.bodyBDataIndex = 0;
		desc.linkIndexB = PxSolverConstraintDesc::RIGID_BODY;
	}
	else
	{
		const IG::Node& node = islandSim.getNode(node2);
		if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
		{
			Dy::FeatherstoneArticulation* b = getArticulationFromIG(islandSim, node2);

			desc.articulationB = b;
			desc.linkIndexB = node2.articulationLinkId();
		}
		else
		{
			const PxU32 activeIndex = islandSim.getActiveNodeIndex(node2);
			const PxU32 index = node.isKinematic() ? activeIndex : bodyRemap[activeIndex] + solverBodyOffset;
			desc.bodyB = &mSolverBodyPool[index];
			desc.bodyBDataIndex = index + 1;
			desc.linkIndexB = PxSolverConstraintDesc::RIGID_BODY;
		}
	}
}

class PxsPreIntegrateTask : public Cm::Task
{
	PxsPreIntegrateTask& operator=(const PxsPreIntegrateTask&);
public:
	PxsPreIntegrateTask(	DynamicsContext&	context,
							PxsBodyCore*const*	bodyArray,
							PxsRigidBody*const*	originalBodyArray,
							PxU32 const*		nodeIndexArray,
							PxSolverBodyData*	solverBodyDataPool,
							PxF32				dt,
							PxU32				numBodies,
							volatile PxU32*		maxSolverPositionIterations,
							volatile PxU32*		maxSolverVelocityIterations,
							PxU32				startIndex,
							PxU32				numToIntegrate,
							const PxVec3&		gravity) :
		Cm::Task					(context.getContextId()),
		mContext					(context),
		mBodyArray					(bodyArray),
		mOriginalBodyArray			(originalBodyArray),
		mNodeIndexArray				(nodeIndexArray),
		mSolverBodyDataPool			(solverBodyDataPool),
		mDt							(dt),
		mNumBodies					(numBodies),
		mMaxSolverPositionIterations(maxSolverPositionIterations),
		mMaxSolverVelocityIterations(maxSolverVelocityIterations),
		mStartIndex					(startIndex),
		mNumToIntegrate				(numToIntegrate),
		mGravity					(gravity)
	{}

	virtual void runInternal();

	virtual const char* getName() const
	{
		return "PxsDynamics.preIntegrate";
	}

public:
	DynamicsContext&		mContext;
	PxsBodyCore*const*		mBodyArray;
	PxsRigidBody*const*		mOriginalBodyArray;
	PxU32 const*			mNodeIndexArray;
	PxSolverBody*			mSolverBodies;
	PxSolverBodyData*		mSolverBodyDataPool;
	PxF32					mDt;
	PxU32					mNumBodies;
	volatile PxU32*			mMaxSolverPositionIterations;
	volatile PxU32*			mMaxSolverVelocityIterations;
	PxU32					mStartIndex;
	PxU32					mNumToIntegrate;
	PxVec3					mGravity;
};

class PxsParallelSolverTask : public Cm::Task
{
	PxsParallelSolverTask& operator=(PxsParallelSolverTask&);
public:

	PxsParallelSolverTask(SolverIslandParams& params, DynamicsContext& context, IG::IslandSim& islandSim)
		: Cm::Task(context.getContextId()), mParams(params), mContext(context), mIslandSim(islandSim)
	{
	}

	virtual void runInternal()
	{
		solveParallel(mContext, mParams, mIslandSim);
	}

	virtual const char* getName() const
	{
		return "PxsDynamics.parallelSolver";
	}

	SolverIslandParams&		mParams;
	DynamicsContext&		mContext;
	IG::IslandSim&			mIslandSim;
};

#define PX_CONTACT_REDUCTION 1

class PxsSolverConstraintPostProcessTask : public Cm::Task
{
	PxsSolverConstraintPostProcessTask& operator=(const PxsSolverConstraintPostProcessTask&);
public:

	PxsSolverConstraintPostProcessTask(DynamicsContext& context,
		ThreadContext& threadContext,
		const SolverIslandObjects& objects,				  
		PxU32 solverBodyOffset,
		PxU32 startIndex,
		PxU32 stride,
		PxsMaterialManager* materialManager,
		PxsContactManagerOutputIterator& iterator) :
		Cm::Task			(context.getContextId()),
		mContext			(context), 
		mThreadContext		(threadContext),
		mObjects			(objects),
		mSolverBodyOffset	(solverBodyOffset),
		mStartIndex			(startIndex),
		mStride				(stride),
		mMaterialManager	(materialManager),
		mOutputs			(iterator)
	{}

	void mergeContacts(CompoundContactManager& header, ThreadContext& threadContext)
	{
		PxContactBuffer& buffer = threadContext.mContactBuffer;
		PxsMaterialInfo materialInfo[PxContactBuffer::MAX_CONTACTS];
		PxU32 size = 0;

		for(PxU32 a = 0; a < header.mStride; ++a)
		{
			PxsContactManager* manager = mThreadContext.orderedContactList[a+header.mStartIndex]->contactManager;
			PxcNpWorkUnit& unit = manager->getWorkUnit();
			const PxsContactManagerOutput& output = mOutputs.getContactManagerOutput(unit.mNpIndex);
			PxContactStreamIterator iter(output.contactPatches, output.contactPoints, output.getInternalFaceIndice(), output.nbPatches, output.nbContacts);

			PxU32 origSize = size;
			PX_UNUSED(origSize);
			if(!iter.forceNoResponse)
			{
				while(iter.hasNextPatch())
				{
					iter.nextPatch();
					while(iter.hasNextContact())
					{
						PX_ASSERT(size < PxContactBuffer::MAX_CONTACTS);
						iter.nextContact();
						PxsMaterialInfo& info = materialInfo[size];
						PxContactPoint& point = buffer.contacts[size++];
						point.dynamicFriction = iter.getDynamicFriction();
						point.staticFriction = iter.getStaticFriction();
						point.restitution = iter.getRestitution();
						point.internalFaceIndex1 = iter.getFaceIndex1();
						PX_ASSERT(iter.getMaterialFlags() <= PX_MAX_U8);
						point.materialFlags = PxU8(iter.getMaterialFlags());
						point.maxImpulse = iter.getMaxImpulse();
						point.targetVel = iter.getTargetVel();
						point.normal = iter.getContactNormal();
						point.point = iter.getContactPoint();
						point.separation = iter.getSeparation();
						info.mMaterialIndex0 = iter.getMaterialIndex0();
						info.mMaterialIndex1 = iter.getMaterialIndex1();
					}
				}
				PX_ASSERT(output.nbContacts == (size - origSize));
			}	
		}

		PxU32 origSize = size;
#if PX_CONTACT_REDUCTION
		ContactReduction<6, 6> reduction(buffer.contacts, materialInfo, size);
		reduction.reduceContacts();
		//OK, now we write back the contacts...

		PxU8 histo[PxContactBuffer::MAX_CONTACTS];
		PxMemZero(histo, sizeof(histo));

		size = 0;
		for(PxU32 a = 0; a < reduction.mNumPatches; ++a)
		{
			ReducedContactPatch<6>& patch = reduction.mPatches[a];
			for(PxU32 b = 0; b < patch.numContactPoints; ++b)
			{
				histo[patch.contactPoints[b]] = 1;
				++size;
			}
		}
#endif

		// TODO AD: this probably needs an overflow check as well.
		PxU16* PX_RESTRICT data = reinterpret_cast<PxU16*>(threadContext.mConstraintBlockStream.reserve(size * sizeof(PxU16), mThreadContext.mConstraintBlockManager));

		header.forceBufferList = data;
		
#if PX_CONTACT_REDUCTION
		const PxU32 reservedSize = size;
		PX_UNUSED(reservedSize);
		size = 0;
		for(PxU32 a = 0; a < origSize; ++a)
		{
			if(histo[a])
			{
				if(size != a)
				{
					buffer.contacts[size] = buffer.contacts[a];
					materialInfo[size] = materialInfo[a];
				}
				data[size] = PxTo16(a);
				size++;
			}
		}
		PX_ASSERT(reservedSize >= size);
#else
		for(PxU32 a = 0; a < size; ++a)
			data[a] = a;
#endif

		PxU32 contactForceByteSize = size * sizeof(PxReal);

		PxsContactManagerOutput& output = mOutputs.getContactManagerOutput(header.unit->mNpIndex);

		PxU16 compressedContactSize;

		physx::writeCompressedContact(buffer.contacts, size, NULL, output.nbContacts, output.contactPatches, output.contactPoints, compressedContactSize,
			reinterpret_cast<PxReal*&>(output.contactForces), contactForceByteSize,
			output.frictionPatches, NULL,
			mMaterialManager, false,
			false, materialInfo, output.nbPatches, 0, &mThreadContext.mConstraintBlockManager, &threadContext.mConstraintBlockStream, false);
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("ConstraintPostProcess", mContext.getContextId());
		PxU32 endIndex = mStartIndex + mStride;

		ThreadContext* threadContext = mContext.getThreadContext();
		//TODO - we need to do this somewhere else
		//threadContext->mContactBlockStream.reset();
		threadContext->mConstraintBlockStream.reset();

		for(PxU32 a = mStartIndex; a < endIndex; ++a)
		{
			mergeContacts(mThreadContext.compoundConstraints[a], *threadContext);
		}
		mContext.putThreadContext(threadContext);
	}

	virtual const char* getName() const { return "PxsDynamics.solverConstraintPostProcess"; }

	DynamicsContext&			mContext;
	ThreadContext&				mThreadContext;
	const SolverIslandObjects	mObjects;
	const PxU32					mSolverBodyOffset;
	const PxU32					mStartIndex;
	const PxU32					mStride;
	PxsMaterialManager*			mMaterialManager;
	PxsContactManagerOutputIterator& mOutputs;
};

class PxsForceThresholdTask  : public Cm::Task
{
	DynamicsContext&		mDynamicsContext;

	PxsForceThresholdTask& operator=(const PxsForceThresholdTask&);
public:

	PxsForceThresholdTask(DynamicsContext& context): Cm::Task(context.getContextId()), mDynamicsContext(context) 
	{
	}

	void createForceChangeThresholdStream()
	{
		ThresholdStream& thresholdStream = mDynamicsContext.getThresholdStream();
		//bool haveThresholding = thresholdStream.size()!=0;

		ThresholdTable& thresholdTable = mDynamicsContext.getThresholdTable();
		thresholdTable.build(thresholdStream);

		//generate current force exceeded threshold stream
		ThresholdStream& curExceededForceThresholdStream = *mDynamicsContext.mExceededForceThresholdStream[mDynamicsContext.mCurrentIndex];
		ThresholdStream& preExceededForceThresholdStream = *mDynamicsContext.mExceededForceThresholdStream[1 - mDynamicsContext.mCurrentIndex];
		curExceededForceThresholdStream.forceSize_Unsafe(0);

		//fill in the currrent exceeded force threshold stream
		for(PxU32 i=0; i<thresholdTable.mPairsSize; ++i)
		{
			ThresholdTable::Pair& pair = thresholdTable.mPairs[i];
			ThresholdStreamElement& elem = thresholdStream[pair.thresholdStreamIndex];
			if(pair.accumulatedForce > elem.threshold * mDynamicsContext.mDt)
			{
				elem.accumulatedForce = pair.accumulatedForce;
				curExceededForceThresholdStream.pushBack(elem);
			}
		}

		ThresholdStream& forceChangeThresholdStream = mDynamicsContext.getForceChangedThresholdStream();
		forceChangeThresholdStream.forceSize_Unsafe(0);
		PxArray<PxU32>& forceChangeMask = mDynamicsContext.mExceededForceThresholdStreamMask;

		const PxU32 nbPreExceededForce = preExceededForceThresholdStream.size();
		const PxU32 nbCurExceededForce = curExceededForceThresholdStream.size();

		//generate force change thresholdStream
		if(nbPreExceededForce)
		{
			thresholdTable.build(preExceededForceThresholdStream);

			//set force change mask
			const PxU32 nbTotalExceededForce = nbPreExceededForce + nbCurExceededForce;
			forceChangeMask.reserve(nbTotalExceededForce);
			forceChangeMask.forceSize_Unsafe(nbTotalExceededForce);
			
			//initialize the forceChangeMask
			for (PxU32 i = 0; i < nbTotalExceededForce; ++i)
				forceChangeMask[i] = 1;

			for(PxU32 i=0; i< nbCurExceededForce; ++i)
			{
				ThresholdStreamElement& curElem = curExceededForceThresholdStream[i];
				
				PxU32 pos;
				if(thresholdTable.check(preExceededForceThresholdStream, curElem, pos))
				{
					forceChangeMask[pos] = 0;
					forceChangeMask[i + nbPreExceededForce] = 0;
				}
			}

			//create force change threshold stream
			for(PxU32 i=0; i<nbTotalExceededForce; ++i)
			{
				const PxU32 hasForceChange = forceChangeMask[i];
				if(hasForceChange)
				{
					bool lostPair = (i < nbPreExceededForce);
					ThresholdStreamElement& elem = lostPair ?  preExceededForceThresholdStream[i] : curExceededForceThresholdStream[i - nbPreExceededForce];
					ThresholdStreamElement elt;
					elt = elem;
					elt.accumulatedForce = lostPair ? 0.f : elem.accumulatedForce;
					forceChangeThresholdStream.pushBack(elt);
				}
				else
				{
					//persistent pair
					if (i < nbPreExceededForce)
					{
						ThresholdStreamElement& elem = preExceededForceThresholdStream[i];
						ThresholdStreamElement elt;
						elt = elem;
						elt.accumulatedForce = elem.accumulatedForce;
						forceChangeThresholdStream.pushBack(elt);
					}
				}
			}
		}
		else
		{
			forceChangeThresholdStream.reserve(nbCurExceededForce);
			forceChangeThresholdStream.forceSize_Unsafe(nbCurExceededForce);
			PxMemCopy(forceChangeThresholdStream.begin(), curExceededForceThresholdStream.begin(), sizeof(ThresholdStreamElement) * nbCurExceededForce);
		}
	}

	virtual void runInternal()
	{
		mDynamicsContext.getThresholdStream().forceSize_Unsafe(PxU32(mDynamicsContext.mThresholdStreamOut));
		createForceChangeThresholdStream();
	}

	virtual const char* getName() const { return "PxsDynamics.createForceChangeThresholdStream"; }
};

struct ConstraintLess
{
	bool operator()(const PxSolverConstraintDesc& left, const PxSolverConstraintDesc& right) const
	{
		return reinterpret_cast<Constraint*>(left.constraint)->index > reinterpret_cast<Constraint*>(right.constraint)->index;
	}
};

struct ArticulationSortPredicate
{
	bool operator()(const PxsIndexedContactManager*& left, const PxsIndexedContactManager*& right) const
	{
		return left->contactManager->getWorkUnit().mIndex < right->contactManager->getWorkUnit().mIndex;
	}
};

class SolverArticulationUpdateTask : public Cm::Task
{
	ThreadContext& mIslandThreadContext;

	FeatherstoneArticulation**	mArticulations;
	ArticulationSolverDesc*		mArticulationDescArray;
	PxU32 mNbToProcess;

	Dy::DynamicsContext& mContext;
	

public:

	static const PxU32 NbArticulationsPerTask = 32;

	SolverArticulationUpdateTask(ThreadContext& islandThreadContext, FeatherstoneArticulation** articulations, ArticulationSolverDesc* articulationDescArray, PxU32 nbToProcess, Dy::DynamicsContext& context) :
		Cm::Task(context.getContextId()), mIslandThreadContext(islandThreadContext), mArticulations(articulations), mArticulationDescArray(articulationDescArray), mNbToProcess(nbToProcess), mContext(context)
	{
	}

	virtual const char* getName() const { return "SolverArticulationUpdateTask"; }

	virtual void runInternal()
	{
		ThreadContext& threadContext = *mContext.getThreadContext();

		threadContext.mConstraintBlockStream.reset(); //Clear in case there's some left-over memory in this context, for which the block has already been freed 
		PxU32 maxVelIters = 0;
		PxU32 maxPosIters = 0;
		PxU32 maxLinks = 0;

		for (PxU32 i = 0; i < mNbToProcess; i++)
		{
			FeatherstoneArticulation& a = *(mArticulations[i]);
			a.getSolverDesc(mArticulationDescArray[i]);

			maxLinks = PxMax(maxLinks, PxU32(mArticulationDescArray[i].linkCount));
		}

		threadContext.mDeltaV.forceSize_Unsafe(0);
		threadContext.mDeltaV.reserve(maxLinks);
		threadContext.mDeltaV.forceSize_Unsafe(maxLinks);

		BlockAllocator blockAllocator(mIslandThreadContext.mConstraintBlockManager, threadContext.mConstraintBlockStream, threadContext.mFrictionPatchStreamPair, threadContext.mConstraintSize);

		const PxReal invLengthScale = 1.f/mContext.getLengthScale();

		for(PxU32 i=0;i<mNbToProcess; i++)
		{
			FeatherstoneArticulation& a = *(mArticulations[i]);

			PxU32 acCount, descCount;
			
			descCount = ArticulationPImpl::computeUnconstrainedVelocities(mArticulationDescArray[i], mContext.mDt,
				acCount, mContext.getGravity(), invLengthScale);

			mArticulationDescArray[i].numInternalConstraints = PxTo8(descCount);

			const PxU16 iterWord = a.getIterationCounts();
			maxVelIters = PxMax<PxU32>(PxU32(iterWord >> 8),	maxVelIters);
			maxPosIters = PxMax<PxU32>(PxU32(iterWord & 0xff), maxPosIters);
		}

		PxAtomicMax(reinterpret_cast<PxI32*>(&mIslandThreadContext.mMaxSolverPositionIterations), PxI32(maxPosIters));
		PxAtomicMax(reinterpret_cast<PxI32*>(&mIslandThreadContext.mMaxSolverVelocityIterations), PxI32(maxVelIters));
		PxAtomicMax(reinterpret_cast<PxI32*>(&mIslandThreadContext.mMaxArticulationLinks), PxI32(maxLinks));

		mContext.putThreadContext(&threadContext);
	}

private:
	PX_NOCOPY(SolverArticulationUpdateTask)
};

struct EnhancedSortPredicate
{
	bool operator()(const PxsIndexedContactManager& left, const PxsIndexedContactManager& right) const
	{
		PxcNpWorkUnit& unit0 = left.contactManager->getWorkUnit();
		PxcNpWorkUnit& unit1 = right.contactManager->getWorkUnit();
		return (unit0.mTransformCache0 < unit1.mTransformCache0) ||
			((unit0.mTransformCache0 == unit1.mTransformCache0) && (unit0.mTransformCache1 < unit1.mTransformCache1));
	}
};

class PxsSolverStartTask : public Cm::Task
{
	PxsSolverStartTask& operator=(const PxsSolverStartTask&);
public:

	PxsSolverStartTask(DynamicsContext& context,
		IslandContext& islandContext,
		const SolverIslandObjects& objects,
		PxU32 solverBodyOffset,
		PxU32 kinematicCount,
		IG::SimpleIslandManager& islandManager,
		PxU32* bodyRemapTable,
		PxsMaterialManager* materialManager,
		PxsContactManagerOutputIterator& iterator,
		bool enhancedDeterminism
		) :
		Cm::Task				(context.getContextId()),
		mContext				(context), 
		mIslandContext			(islandContext),
		mObjects				(objects),
		mSolverBodyOffset		(solverBodyOffset),
		mKinematicCount			(kinematicCount),
		mIslandManager			(islandManager),
		mBodyRemapTable			(bodyRemapTable),
		mMaterialManager		(materialManager),
		mOutputs				(iterator),
		mEnhancedDeterminism	(enhancedDeterminism)
	{}

	void startTasks()
	{
		PX_PROFILE_ZONE("Dynamics.solveGroup", mContext.getContextId());
		{
			ThreadContext& mThreadContext = *mContext.getThreadContext();

			mIslandContext.mThreadContext = &mThreadContext;

			mThreadContext.mMaxSolverPositionIterations = 0;
			mThreadContext.mMaxSolverVelocityIterations = 0;
			mThreadContext.mAxisConstraintCount = 0;
			mThreadContext.mFrictionDescPtr = mThreadContext.frictionConstraintDescArray.begin();
			mThreadContext.mNumDifferentBodyConstraints = 0;
			mThreadContext.mNumStaticConstraints = 0;
			mThreadContext.mNumSelfConstraints = 0;
			mThreadContext.mNumDifferentBodyFrictionConstraints = 0;
			mThreadContext.mNumSelfConstraintFrictionBlocks = 0;
			mThreadContext.mNumSelfFrictionConstraints = 0;
			mThreadContext.numContactConstraintBatches = 0;
			mThreadContext.contactDescArraySize = 0;
			mThreadContext.mMaxArticulationLinks = 0;

			mThreadContext.contactConstraintDescArray = mObjects.constraintDescs;
			mThreadContext.orderedContactConstraints = mObjects.orderedConstraintDescs;
			mThreadContext.mContactDescPtr = mObjects.constraintDescs;
			mThreadContext.tempConstraintDescArray = mObjects.tempConstraintDescs;
			mThreadContext.contactConstraintBatchHeaders = mObjects.constraintBatchHeaders;
			mThreadContext.motionVelocityArray = mObjects.motionVelocities;
			mThreadContext.mBodyCoreArray = mObjects.bodyCoreArray;
			mThreadContext.mRigidBodyArray = mObjects.bodies;
			mThreadContext.mArticulationArray = mObjects.articulations;
			mThreadContext.bodyRemapTable = mObjects.bodyRemapTable;
			mThreadContext.mNodeIndexArray = mObjects.nodeIndexArray;

			const PxU32 frictionConstraintCount = mContext.getFrictionType() == PxFrictionType::ePATCH ? 0 : PxU32(mIslandContext.mCounts.contactManagers);
			mThreadContext.resizeArrays(frictionConstraintCount, mIslandContext.mCounts.articulations);

			PxsBodyCore** PX_RESTRICT bodyArrayPtr = mThreadContext.mBodyCoreArray;
			PxsRigidBody** PX_RESTRICT rigidBodyPtr = mThreadContext.mRigidBodyArray;
			FeatherstoneArticulation** PX_RESTRICT articulationPtr = mThreadContext.mArticulationArray;

			PxU32* PX_RESTRICT bodyRemapTable = mThreadContext.bodyRemapTable;
			PxU32* PX_RESTRICT nodeIndexArray = mThreadContext.mNodeIndexArray;

			PxU32 nbIslands = mObjects.numIslands;
			const IG::IslandId* const islandIds = mObjects.islandIds;

			const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

			PxU32 bodyIndex = 0, articIndex = 0;

			for (PxU32 i = 0; i < nbIslands; ++i)
			{
				const IG::Island& island = islandSim.getIsland(islandIds[i]);

				PxNodeIndex currentIndex = island.mRootNode;

				while (currentIndex.isValid())
				{
					const IG::Node& node = islandSim.getNode(currentIndex);

					if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
					{
						articulationPtr[articIndex++] = getObjectFromIG<FeatherstoneArticulation>(node);
					}
					else
					{
						PX_ASSERT(bodyIndex < (mIslandContext.mCounts.bodies + mContext.mKinematicCount + 1));
						nodeIndexArray[bodyIndex++] = currentIndex.index();
					}

					currentIndex = node.mNextNode;
				}
			}

			//Bodies can come in a slightly jumbled order from islandGen. It's deterministic if the scene is 
			//identical but can vary if there are additional bodies in the scene in a different island.
			if (mEnhancedDeterminism)
				PxSort(nodeIndexArray, bodyIndex);

			for (PxU32 a = 0; a < bodyIndex; ++a)
			{
				const PxNodeIndex currentIndex(nodeIndexArray[a]);
				PxsRigidBody* rigid = getRigidBodyFromIG(islandSim, currentIndex);
				rigidBodyPtr[a] = rigid;
				bodyArrayPtr[a] = &rigid->getCore();
				bodyRemapTable[islandSim.getActiveNodeIndex(currentIndex)] = a;
			}

			PxsIndexedContactManager* indexedManagers = mObjects.contactManagers;

			PxU32 currentContactIndex = 0;
			for(PxU32 i = 0; i < nbIslands; ++i)
			{
				const IG::Island& island = islandSim.getIsland(islandIds[i]);

				IG::EdgeIndex contactEdgeIndex = island.mFirstEdge[IG::Edge::eCONTACT_MANAGER];

				while(contactEdgeIndex != IG_INVALID_EDGE)
				{
					const IG::Edge& edge = islandSim.getEdge(contactEdgeIndex);

					PxsContactManager* contactManager = mIslandManager.getContactManager(contactEdgeIndex);

					if(contactManager)
					{
						const PxNodeIndex nodeIndex1 = islandSim.mCpuData.getNodeIndex1(contactEdgeIndex);
						const PxNodeIndex nodeIndex2 = islandSim.mCpuData.getNodeIndex2(contactEdgeIndex);

						PxsIndexedContactManager& indexedManager = indexedManagers[currentContactIndex++];
						indexedManager.contactManager = contactManager;

						PX_ASSERT(!nodeIndex1.isStaticBody());
						{
							const IG::Node& node1 = islandSim.getNode(nodeIndex1);

							//Is it an articulation or not???
							if(node1.getNodeType() == IG::Node::eARTICULATION_TYPE)
							{
								indexedManager.articulation0 = nodeIndex1.getInd();
								indexedManager.indexType0 = PxsIndexedInteraction::eARTICULATION;
							}
							else
							{
								if(node1.isKinematic())
								{
									indexedManager.indexType0 = PxsIndexedInteraction::eKINEMATIC;
									indexedManager.solverBody0 = islandSim.getActiveNodeIndex(nodeIndex1);
								}
								else
								{
									indexedManager.indexType0 = PxsIndexedInteraction::eBODY;
									indexedManager.solverBody0 = bodyRemapTable[islandSim.getActiveNodeIndex(nodeIndex1)];
								}
								PX_ASSERT(indexedManager.solverBody0 < (mIslandContext.mCounts.bodies + mContext.mKinematicCount + 1));
							}
						}

						if(nodeIndex2.isStaticBody())
						{
							indexedManager.indexType1 = PxsIndexedInteraction::eWORLD;
						}
						else
						{
							const IG::Node& node2 = islandSim.getNode(nodeIndex2);

							//Is it an articulation or not???
							if(node2.getNodeType() == IG::Node::eARTICULATION_TYPE)
							{
								indexedManager.articulation1 = nodeIndex2.getInd();
								indexedManager.indexType1 = PxsIndexedInteraction::eARTICULATION;
							}
							else
							{
								if(node2.isKinematic())
								{
									indexedManager.indexType1 = PxsIndexedInteraction::eKINEMATIC;
									indexedManager.solverBody1 = islandSim.getActiveNodeIndex(nodeIndex2);
								}
								else
								{
									indexedManager.indexType1 = PxsIndexedInteraction::eBODY;
									indexedManager.solverBody1 = bodyRemapTable[islandSim.getActiveNodeIndex(nodeIndex2)];
								}
								PX_ASSERT(indexedManager.solverBody1 < (mIslandContext.mCounts.bodies + mContext.mKinematicCount + 1));
							}
						}
					}
					contactEdgeIndex = edge.mNextIslandEdge;
				}
			}

			if (mEnhancedDeterminism)
				PxSort(indexedManagers, currentContactIndex, EnhancedSortPredicate());

			mIslandContext.mCounts.contactManagers = currentContactIndex;
		}
	}

	void integrate()
	{
		ThreadContext& mThreadContext = *mIslandContext.mThreadContext;
		PxSolverBody* solverBodies = mContext.mSolverBodyPool.begin() + mSolverBodyOffset;
		PxSolverBodyData* solverBodyData = mContext.mSolverBodyDataPool.begin() + mSolverBodyOffset;

		{			
			PX_PROFILE_ZONE("Dynamics.updateVelocities", mContext.getContextId());

			mContext.preIntegrationParallel(	
				mContext.mDt,
				mThreadContext.mBodyCoreArray,
				mObjects.bodies,
				mThreadContext.mNodeIndexArray,
				mIslandContext.mCounts.bodies,
				solverBodies,
				solverBodyData,
				mThreadContext.motionVelocityArray,
				mThreadContext.mMaxSolverPositionIterations,
				mThreadContext.mMaxSolverVelocityIterations,
				*mCont
				);
		}
	}

	void articulationTask()
	{
		ThreadContext& mThreadContext = *mIslandContext.mThreadContext;
		ArticulationSolverDesc* articulationDescArray = mThreadContext.getArticulations().begin();

		for(PxU32 i=0;i<mIslandContext.mCounts.articulations; i+= SolverArticulationUpdateTask::NbArticulationsPerTask)
		{
			SolverArticulationUpdateTask* task = PX_PLACEMENT_NEW(mContext.getTaskPool().allocate(sizeof(SolverArticulationUpdateTask)), SolverArticulationUpdateTask)(mThreadContext, 
				&mObjects.articulations[i], &articulationDescArray[i], PxMin(SolverArticulationUpdateTask::NbArticulationsPerTask, mIslandContext.mCounts.articulations - i), mContext);

			task->setContinuation(mCont);
			task->removeReference();
		}
	}

	void setupDescTask()
	{
		PX_PROFILE_ZONE("SetupDescs", mContext.getContextId());
		ThreadContext& mThreadContext = *mIslandContext.mThreadContext;
		PxSolverConstraintDesc* contactDescPtr = mThreadContext.mContactDescPtr;

		//PxU32 constraintCount = mCounts.constraints + mCounts.contactManagers;

		PxU32 nbIslands = mObjects.numIslands;
		const IG::IslandId* const islandIds = mObjects.islandIds;

		const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

		for(PxU32 i = 0; i < nbIslands; ++i)
		{
			const IG::Island& island = islandSim.getIsland(islandIds[i]);

			IG::EdgeIndex edgeId = island.mFirstEdge[IG::Edge::eCONSTRAINT];

			while(edgeId != IG_INVALID_EDGE)
			{
				PxSolverConstraintDesc& desc = *contactDescPtr;
				
				const IG::Edge& edge = islandSim.getEdge(edgeId);
				Dy::Constraint* constraint = mIslandManager.getConstraint(edgeId);
				mContext.setDescFromIndices(desc, edgeId, mIslandManager, mBodyRemapTable, mSolverBodyOffset);
				desc.constraint = reinterpret_cast<PxU8*>(constraint);
				desc.constraintType = DY_SC_TYPE_RB_1D;
				contactDescPtr++;
				edgeId = edge.mNextIslandEdge;
			}
		}

#if 1
		PxSort(mThreadContext.mContactDescPtr, PxU32(contactDescPtr - mThreadContext.mContactDescPtr), ConstraintLess());
#endif

		mThreadContext.orderedContactList.forceSize_Unsafe(0);
		mThreadContext.orderedContactList.reserve(mIslandContext.mCounts.contactManagers);
		mThreadContext.orderedContactList.forceSize_Unsafe(mIslandContext.mCounts.contactManagers);
		mThreadContext.tempContactList.forceSize_Unsafe(0);
		mThreadContext.tempContactList.reserve(mIslandContext.mCounts.contactManagers);
		mThreadContext.tempContactList.forceSize_Unsafe(mIslandContext.mCounts.contactManagers);

		const PxsIndexedContactManager** constraints = mThreadContext.orderedContactList.begin();

		//OK, we sort the orderedContactList 

		mThreadContext.compoundConstraints.forceSize_Unsafe(0);
		if(mIslandContext.mCounts.contactManagers)
		{
			{
				mThreadContext.sortIndexArray.forceSize_Unsafe(0);

				PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eBODY == 0);
				PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eKINEMATIC == 1);

				const PxI32 offsetMap[] = {PxI32(mContext.mKinematicCount), 0};

				const PxU32 totalBodies = mContext.mKinematicCount + mIslandContext.mCounts.bodies+1;

				mThreadContext.sortIndexArray.reserve(totalBodies);
				mThreadContext.sortIndexArray.forceSize_Unsafe(totalBodies);
				PxMemZero(mThreadContext.sortIndexArray.begin(), totalBodies * 4);

				//Iterate over the array based on solverBodyDatapool, creating a list of sorted constraints (in order of body pair)
				//We only do this with contacts. It's important that this is done this way because we don't want to break our rules that all joints
				//appear before all contacts in the constraint list otherwise we will lose all guarantees about sorting joints.

				for(PxU32 a = 0; a < mIslandContext.mCounts.contactManagers; ++a)
				{
					PX_ASSERT(mObjects.contactManagers[a].indexType0 != PxsIndexedInteraction::eWORLD);
					//Index first body...
					PxU8 indexType = mObjects.contactManagers[a].indexType0;
					if(indexType != PxsIndexedInteraction::eARTICULATION && mObjects.contactManagers[a].indexType1 != PxsIndexedInteraction::eARTICULATION)
					{
						PX_ASSERT((indexType == PxsIndexedInteraction::eBODY) || (indexType == PxsIndexedInteraction::eKINEMATIC));

						PxI32 index = PxI32(mObjects.contactManagers[a].solverBody0 + offsetMap[indexType]);
						PX_ASSERT(index >= 0);
						mThreadContext.sortIndexArray[PxU32(index)]++;
					}
				}

				PxU32 accumulatedCount = 0;

				for(PxU32 a = mThreadContext.sortIndexArray.size(); a > 0; --a)
				{
					PxU32 ind = a - 1;
					PxU32 val = mThreadContext.sortIndexArray[ind];
					mThreadContext.sortIndexArray[ind] = accumulatedCount;
					accumulatedCount += val;
				}

				//OK, now copy across data to orderedConstraintDescs, pushing articulations to the end...
				for(PxU32 a = 0; a < mIslandContext.mCounts.contactManagers; ++a)
				{
					//Index first body...
					PxU8 indexType = mObjects.contactManagers[a].indexType0;
					if(indexType != PxsIndexedInteraction::eARTICULATION && mObjects.contactManagers[a].indexType1 != PxsIndexedInteraction::eARTICULATION)
					{
						PX_ASSERT((indexType == PxsIndexedInteraction::eBODY) || (indexType == PxsIndexedInteraction::eKINEMATIC));

						PxI32 index = PxI32(mObjects.contactManagers[a].solverBody0 + offsetMap[indexType]);
						PX_ASSERT(index >= 0);
						mThreadContext.tempContactList[mThreadContext.sortIndexArray[PxU32(index)]++] = &mObjects.contactManagers[a];
					}
					else
					{
						mThreadContext.tempContactList[accumulatedCount++] = &mObjects.contactManagers[a];
					}
				}

				//Now do the same again with bodyB, being careful not to overwrite the joints
				PxMemZero(mThreadContext.sortIndexArray.begin(), totalBodies * 4);

				for(PxU32 a = 0; a < mIslandContext.mCounts.contactManagers; ++a)
				{
					//Index first body...
					PxU8 indexType = mThreadContext.tempContactList[a]->indexType1;
					if(indexType != PxsIndexedInteraction::eARTICULATION && mObjects.contactManagers[a].indexType0 != PxsIndexedInteraction::eARTICULATION)
					{
						PX_ASSERT((indexType == PxsIndexedInteraction::eBODY) || (indexType == PxsIndexedInteraction::eKINEMATIC) || (indexType == PxsIndexedInteraction::eWORLD));

						PxI32 index = (indexType == PxsIndexedInteraction::eWORLD) ? 0 : PxI32(mThreadContext.tempContactList[a]->solverBody1 + offsetMap[indexType]);
						PX_ASSERT(index >= 0);
						mThreadContext.sortIndexArray[PxU32(index)]++;
					}
				}

				accumulatedCount = 0;
				for(PxU32 a = mThreadContext.sortIndexArray.size(); a > 0; --a)
				{
					PxU32 ind = a - 1;
					PxU32 val = mThreadContext.sortIndexArray[ind];
					mThreadContext.sortIndexArray[ind] = accumulatedCount;
					accumulatedCount += val;
				}

				PxU32 articulationStartIndex = accumulatedCount;

				//OK, now copy across data to orderedConstraintDescs, pushing articulations to the end...
				for(PxU32 a = 0; a < mIslandContext.mCounts.contactManagers; ++a)
				{
					//Index first body...
					PxU8 indexType = mThreadContext.tempContactList[a]->indexType1;
					if(indexType != PxsIndexedInteraction::eARTICULATION && mObjects.contactManagers[a].indexType0 != PxsIndexedInteraction::eARTICULATION)
					{
						PX_ASSERT((indexType == PxsIndexedInteraction::eBODY) || (indexType == PxsIndexedInteraction::eKINEMATIC) || (indexType == PxsIndexedInteraction::eWORLD));

						PxI32 index = (indexType == PxsIndexedInteraction::eWORLD) ? 0 : PxI32(mThreadContext.tempContactList[a]->solverBody1 + offsetMap[indexType]);
						PX_ASSERT(index >= 0);
						constraints[mThreadContext.sortIndexArray[PxU32(index)]++] = mThreadContext.tempContactList[a];
					}
					else
					{
						constraints[accumulatedCount++] = mThreadContext.tempContactList[a];
					}
				}
#if 1
				PxSort(constraints + articulationStartIndex, accumulatedCount - articulationStartIndex, ArticulationSortPredicate());
#endif
			}

			mThreadContext.mStartContactDescPtr = contactDescPtr;

			mThreadContext.compoundConstraints.reserve(1024);
			mThreadContext.compoundConstraints.forceSize_Unsafe(0);
			//mThreadContext.compoundConstraints.forceSize_Unsafe(mCounts.contactManagers);

			PxSolverConstraintDesc* startDesc = contactDescPtr;
			mContext.setDescFromIndices(*startDesc, islandSim, *constraints[0], mSolverBodyOffset);
			startDesc->constraint = reinterpret_cast<PxU8*>(constraints[0]->contactManager);
			startDesc->constraintType = DY_SC_TYPE_RB_CONTACT;

			PxsContactManagerOutput* startManagerOutput = &mOutputs.getContactManagerOutput(constraints[0]->contactManager->getWorkUnit().mNpIndex);
			PxU32 contactCount = startManagerOutput->nbContacts;
			PxU32 startIndex = 0;
			PxU32 numHeaders = 0;

			const bool gDisableConstraintWelding = false;

			for(PxU32 a = 1; a < mIslandContext.mCounts.contactManagers; ++a)
			{
				PxSolverConstraintDesc& desc = *(contactDescPtr+1);
				mContext.setDescFromIndices(desc, islandSim, *constraints[a], mSolverBodyOffset);

				PxsContactManager* manager = constraints[a]->contactManager;
				PxsContactManagerOutput& output = mOutputs.getContactManagerOutput(manager->getWorkUnit().mNpIndex);

				desc.constraint = reinterpret_cast<PxU8*>(constraints[a]->contactManager);
				desc.constraintType = DY_SC_TYPE_RB_CONTACT;

				if (contactCount == 0)
				{
					//This is the first object in the pair
					*startDesc = *(contactDescPtr + 1);
					startIndex = a;
					startManagerOutput = &output;
				}
				
				if(startDesc->bodyA != desc.bodyA || startDesc->bodyB != desc.bodyB 
					|| startDesc->linkIndexA != PxSolverConstraintDesc::RIGID_BODY || startDesc->linkIndexB != PxSolverConstraintDesc::RIGID_BODY
					|| contactCount + output.nbContacts > PxContactBuffer::MAX_CONTACTS
					|| manager->isChangeable()
					|| gDisableConstraintWelding
					) //If this is the first thing and no contacts...then we skip
				{
					const PxU32 stride = a - startIndex;
					if(contactCount > 0)
					{
						if(stride > 1)
						{
							++numHeaders;
							CompoundContactManager& header = mThreadContext.compoundConstraints.insert();
							header.mStartIndex = startIndex;
							header.mStride = PxTo16(stride);	
							header.mReducedContactCount = PxTo16(contactCount);
							PxsContactManager* manager1 = constraints[startIndex]->contactManager;
							PxcNpWorkUnit& unit = manager1->getWorkUnit();

							PX_ASSERT(startManagerOutput == &mOutputs.getContactManagerOutput(unit.mNpIndex));

							header.unit = &unit;
							header.cmOutput = startManagerOutput;
							header.originalContactPatches = startManagerOutput->contactPatches;
							header.originalContactPoints = startManagerOutput->contactPoints;
							header.originalContactCount = startManagerOutput->nbContacts;
							header.originalPatchCount = startManagerOutput->nbPatches;
							header.originalForceBuffer = reinterpret_cast<PxReal*>(startManagerOutput->contactForces);
							header.originalStatusFlags = startManagerOutput->statusFlag;
							header.originalFrictionPatches = startManagerOutput->frictionPatches;
						}
						startDesc = ++contactDescPtr;
					}
					else
					{
						//Copy back next contactDescPtr
						*startDesc = *(contactDescPtr+1);
					}
					contactCount = 0;
					startIndex = a;
					startManagerOutput = &output;
				}
				contactCount += output.nbContacts;
			}

			const PxU32 stride = mIslandContext.mCounts.contactManagers - startIndex;
			if(contactCount > 0)
			{
				if(stride > 1)
				{
					++numHeaders;
					CompoundContactManager& header = mThreadContext.compoundConstraints.insert();
					header.mStartIndex = startIndex;
					header.mStride = PxTo16(stride);
					header.mReducedContactCount = PxTo16(contactCount);
					PxsContactManager* manager = constraints[startIndex]->contactManager;
					PxcNpWorkUnit& unit = manager->getWorkUnit();
					header.unit = &unit;
					header.cmOutput = startManagerOutput;
					header.originalContactPatches = startManagerOutput->contactPatches;
					header.originalContactPoints = startManagerOutput->contactPoints;
					header.originalContactCount = startManagerOutput->nbContacts;
					header.originalPatchCount = startManagerOutput->nbPatches;
					header.originalForceBuffer = reinterpret_cast<PxReal*>(startManagerOutput->contactForces);
					header.originalStatusFlags = startManagerOutput->statusFlag;
					header.originalFrictionPatches = startManagerOutput->frictionPatches;
				}
				contactDescPtr++;
			}

			if(numHeaders)
			{
				const PxU32 unrollSize = 8;
				for(PxU32 a = 0; a < numHeaders; a+= unrollSize)
				{
					PxsSolverConstraintPostProcessTask* postProcessTask = PX_PLACEMENT_NEW( mContext.getTaskPool().allocate(sizeof(PxsSolverConstraintPostProcessTask)), 
						PxsSolverConstraintPostProcessTask)(mContext, mThreadContext, mObjects, mSolverBodyOffset, a, PxMin(unrollSize, numHeaders - a), mMaterialManager,
						mOutputs);
					postProcessTask->setContinuation(mCont);
					postProcessTask->removeReference();
				}
			}
		}
		mThreadContext.contactDescArraySize = PxU32(contactDescPtr - mThreadContext.contactConstraintDescArray);
		mThreadContext.mContactDescPtr = contactDescPtr;
	}

	virtual void runInternal()
	{
		startTasks();
		integrate();
		setupDescTask();
		articulationTask();
	}

	virtual const char* getName() const
	{
		return "PxsDynamics.solverStart";
	}

private:
	DynamicsContext&			mContext;
	IslandContext&				mIslandContext;
	const SolverIslandObjects	mObjects;
	const PxU32					mSolverBodyOffset;
	const PxU32					mKinematicCount;
	IG::SimpleIslandManager&	mIslandManager;
	PxU32*						mBodyRemapTable;
	PxsMaterialManager*			mMaterialManager;
	PxsContactManagerOutputIterator& mOutputs;
	const bool					mEnhancedDeterminism;
};

class PxsSolverConstraintPartitionTask : public Cm::Task
{
	PxsSolverConstraintPartitionTask& operator=(const PxsSolverConstraintPartitionTask&);
public:

	PxsSolverConstraintPartitionTask(DynamicsContext& context, IslandContext& islandContext, const SolverIslandObjects& objects, PxU32 solverBodyOffset, bool enhancedDeterminism) :
		Cm::Task			(context.getContextId()),
		mContext			(context), 
		mIslandContext		(islandContext),
		mObjects			(objects),
		mSolverBodyOffset	(solverBodyOffset),
		mEnhancedDeterminism(enhancedDeterminism)
	{}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("PartitionConstraints", mContext.getContextId());
		ThreadContext& mThreadContext = *mIslandContext.mThreadContext;

		//Compact articulation pairs...
		const ArticulationSolverDesc* artics = mThreadContext.getArticulations().begin();

		const PxSolverConstraintDesc* descBegin = mThreadContext.contactConstraintDescArray;
		const PxU32 descCount = mThreadContext.contactDescArraySize;

		PxSolverBody* solverBodies = mContext.mSolverBodyPool.begin() + mSolverBodyOffset;
		
		mThreadContext.mNumDifferentBodyConstraints = descCount;

		{
			mThreadContext.mNumDifferentBodyConstraints = 0;
			mThreadContext.mNumSelfConstraints = 0;
			mThreadContext.mNumStaticConstraints = 0;
			mThreadContext.mNumDifferentBodyFrictionConstraints = 0;
			mThreadContext.mNumSelfConstraintFrictionBlocks = 0;
			mThreadContext.mNumSelfFrictionConstraints = 0;

			if(descCount > 0)
			{
				const PxU32 numArticulations = mThreadContext.getArticulations().size();
				PX_ALLOCA(_eaArticulations, Dy::FeatherstoneArticulation*, numArticulations);
				Dy::FeatherstoneArticulation** eaArticulations = _eaArticulations;
				for(PxU32 i=0;i<numArticulations;i++)
					eaArticulations[i] = artics[i].articulation;

				// PT: maxPartitions = 64 in TGS
				const ConstraintPartitionIn in(	reinterpret_cast<PxU8*>(solverBodies), mIslandContext.mCounts.bodies, sizeof(PxSolverBody),
												eaArticulations, numArticulations, descBegin, descCount, PX_MAX_U32, mContext.getFrictionType() != PxFrictionType::ePATCH);
				
				ConstraintPartitionOut out(mThreadContext.orderedContactConstraints, mThreadContext.tempConstraintDescArray, &mThreadContext.mConstraintsPerPartition);
				//args.mBitField = &mThreadContext.mPartitionNormalizationBitmap;	// PT: removed, unused

				mThreadContext.mMaxPartitions = partitionContactConstraints(out, in);
				mThreadContext.mNumDifferentBodyConstraints = out.mNumDifferentBodyConstraints;
				mThreadContext.mNumSelfConstraints = out.mNumSelfConstraints;
				mThreadContext.mNumStaticConstraints = out.mNumStaticConstraints;
			}
			else
			{
				PxMemZero(mThreadContext.mConstraintsPerPartition.begin(), sizeof(PxU32)*mThreadContext.mConstraintsPerPartition.capacity());
			}

			PX_ASSERT((mThreadContext.mNumDifferentBodyConstraints + mThreadContext.mNumSelfConstraints + mThreadContext.mNumStaticConstraints) == descCount);
		}
	}

	virtual const char* getName() const { return "PxsDynamics.solverConstraintPartition"; }

	DynamicsContext&			mContext;
	IslandContext&				mIslandContext;
	const SolverIslandObjects	mObjects;
	const PxU32					mSolverBodyOffset;
	const bool					mEnhancedDeterminism;
};

// PT: 368 => 367 => 351 lines of assembly
static void integrate(	const IG::IslandSim& islandSim, PxSolverBodyData* PX_RESTRICT solverBodyData, PxsRigidBody** PX_RESTRICT rigidBodies,
						Cm::SpatialVector* PX_RESTRICT motionVelocityArray, PxSolverBody* PX_RESTRICT solverBodies, 
						PxU32 count, PxF32 dt, bool enableStabilization)
{
	for(PxU32 i=0; i<count; i++)
	{
		// PT: let the HW prefetcher do its job. Current cache lines aren't 128 bytes anyway.

		PxSolverBodyData& data = solverBodyData[i];
		PxsRigidBody& rBody = *rigidBodies[i];
		PxsBodyCore& core = rBody.getCore();

		// PT: note that only PGS uses "integrateCore", TGS actually uses "integrateCoreStep".
		integrateCore(motionVelocityArray[i].linear, motionVelocityArray[i].angular, solverBodies[i], data, dt, core.lockFlags);

		rBody.mLastTransform = core.body2World;
		core.body2World = data.body2World;
		core.linearVelocity = data.linearVelocity;
		core.angularVelocity = data.angularVelocity;

		const PxU32 hasStaticTouch = islandSim.getIslandStaticTouchCount(PxNodeIndex(data.nodeIndex));
		sleepCheck(rigidBodies[i], dt, enableStabilization, motionVelocityArray[i], hasStaticTouch);
	}
}

class PxsSolverSetupSolveTask : public Cm::Task
{
	PxsSolverSetupSolveTask& operator=(const PxsSolverSetupSolveTask&);
public:

	PxsSolverSetupSolveTask(DynamicsContext& context, IslandContext& islandContext, const SolverIslandObjects& objects, PxU32 solverBodyOffset, IG::IslandSim& islandSim) :
		Cm::Task			(context.getContextId()),
		mContext			(context), 
		mIslandContext		(islandContext),
		mObjects			(objects),
		mSolverBodyOffset	(solverBodyOffset),
		mIslandSim			(islandSim)
	{}

	virtual void runInternal()
	{
		ThreadContext& mThreadContext = *mIslandContext.mThreadContext;

		PxSolverConstraintDesc* contactDescBegin = mThreadContext.orderedContactConstraints;
		PxSolverConstraintDesc* contactDescPtr = mThreadContext.orderedContactConstraints;

		PxSolverBody* solverBodies = mContext.mSolverBodyPool.begin() + mSolverBodyOffset;
		PxSolverBodyData* solverBodyDatas = mContext.mSolverBodyDataPool.begin();

		PxU32 frictionDescCount = mThreadContext.mNumDifferentBodyFrictionConstraints;

		PxU32 j = 0, i = 0;
		
		//On PS3, self-constraints will be bumped to the end of the constraint list
		//and processed separately. On PC/360, they will be mixed in the array and
		//classed as "different body" constraints regardless of the fact that they're self-constraints.
		//PxU32 numBatches = mThreadContext.numDifferentBodyBatchHeaders;
		// TODO: maybe replace with non-null joints from end of the array

		PxU32 numBatches = 0;

		PxU32 currIndex = 0;
		for(PxU32 a = 0; a < mThreadContext.mConstraintsPerPartition.size(); ++a)
		{
			const PxU32 endIndex = currIndex + mThreadContext.mConstraintsPerPartition[a];

			PxU32 numBatchesInPartition = 0;
			for(PxU32 b = currIndex; b < endIndex; ++b)
			{
				PxConstraintBatchHeader& _header = mThreadContext.contactConstraintBatchHeaders[b];
				PxU16 stride = _header.stride, newStride = _header.stride;
				PxU32 startIndex = j;
				for(PxU16 c = 0; c < stride; ++c)
				{
					if(getConstraintLength(contactDescBegin[i]) == 0)
					{
						newStride--;
						i++;
					}
					else
					{
						if(i!=j)
							contactDescBegin[j] = contactDescBegin[i];
						i++;
						j++;
						contactDescPtr++;
					}
				}

				if(newStride != 0)
				{
					mThreadContext.contactConstraintBatchHeaders[numBatches].startIndex = startIndex;
					mThreadContext.contactConstraintBatchHeaders[numBatches].stride = newStride;
					PxU8 type = *contactDescBegin[startIndex].constraint;
					if(type == DY_SC_TYPE_STATIC_CONTACT)
					{
						//Check if any block of constraints is classified as type static (single) contact constraint.
						//If they are, iterate over all constraints grouped with it and switch to "dynamic" contact constraint
						//type if there's a dynamic contact constraint in the group.
						for(PxU32 c = 1; c < newStride; ++c)
						{
							if(*contactDescBegin[startIndex+c].constraint == DY_SC_TYPE_RB_CONTACT)
							{
								type = DY_SC_TYPE_RB_CONTACT;
							}
						}
					}

					mThreadContext.contactConstraintBatchHeaders[numBatches].constraintType = type;
					numBatches++;
					numBatchesInPartition++;
				}
			}
			PxU32 numHeaders = numBatchesInPartition;
			currIndex += mThreadContext.mConstraintsPerPartition[a];
			mThreadContext.mConstraintsPerPartition[a] = numHeaders;
		}

		PxU32 contactDescCount = PxU32(contactDescPtr - contactDescBegin);

		mThreadContext.mNumDifferentBodyConstraints = contactDescCount;		

		mThreadContext.numContactConstraintBatches = numBatches;
		mThreadContext.mNumSelfConstraints = j - contactDescCount; //self constraint count
		contactDescCount = j;
		mThreadContext.mOrderedContactDescCount = j;

		//Now do the friction constraints if we're not using the sticky model
		if(mContext.getFrictionType() != PxFrictionType::ePATCH)
		{
			PxSolverConstraintDesc* frictionDescBegin = mThreadContext.frictionConstraintDescArray.begin();
			PxSolverConstraintDesc* frictionDescPtr = frictionDescBegin;

			PxArray<PxConstraintBatchHeader>& frictionHeaderArray = mThreadContext.frictionConstraintBatchHeaders;
			frictionHeaderArray.forceSize_Unsafe(0);
			frictionHeaderArray.reserve(mThreadContext.numContactConstraintBatches);
			PxConstraintBatchHeader* headers = frictionHeaderArray.begin();

			PxArray<PxU32>& constraintsPerPartition = mThreadContext.mConstraintsPerPartition;
			PxArray<PxU32>& frictionConstraintsPerPartition = mThreadContext.mFrictionConstraintsPerPartition;
			frictionConstraintsPerPartition.forceSize_Unsafe(0);
			frictionConstraintsPerPartition.reserve(constraintsPerPartition.capacity());
			
			PxU32 fricI = 0;
			PxU32 startIndex = 0;
			PxU32 fricHeaders = 0;
			for(PxU32 k = 0; k < constraintsPerPartition.size(); ++k)
			{
				PxU32 numBatchesInK = constraintsPerPartition[k];
				PxU32 endIndex = startIndex + numBatchesInK;

				PxU32 startFricH = fricHeaders;

				for(PxU32 a = startIndex; a < endIndex; ++a)
				{
					PxConstraintBatchHeader& _header = mThreadContext.contactConstraintBatchHeaders[a];
					PxU16 stride = _header.stride;
					if(_header.constraintType == DY_SC_TYPE_RB_CONTACT || _header.constraintType == DY_SC_TYPE_EXT_CONTACT || 
						_header.constraintType == DY_SC_TYPE_STATIC_CONTACT)
					{
						PxU8 type = 0;
						//Extract friction from this constraint
						for(PxU16 b = 0; b < stride; ++b)
						{
							//create the headers...
							PxSolverConstraintDesc& desc = contactDescBegin[_header.startIndex + b];
							PX_ASSERT(desc.constraint);
							SolverContactCoulombHeader* header = reinterpret_cast<SolverContactCoulombHeader*>(desc.constraint);
							PxU32 frictionOffset = header->frictionOffset;
							PxU8* PX_RESTRICT constraint = reinterpret_cast<PxU8*>(header) + frictionOffset;
							const PxU32 origLength = getConstraintLength(desc);
							const PxU32 length = (origLength - frictionOffset);

							setConstraintLength(*frictionDescPtr, length);
							frictionDescPtr->constraint	= constraint;
							frictionDescPtr->bodyA = desc.bodyA;
							frictionDescPtr->bodyB = desc.bodyB;
							frictionDescPtr->bodyADataIndex = desc.bodyADataIndex;
							frictionDescPtr->bodyBDataIndex = desc.bodyBDataIndex;
							frictionDescPtr->linkIndexA = desc.linkIndexA;
							frictionDescPtr->linkIndexB = desc.linkIndexB;
							frictionDescPtr->writeBack = NULL;
							type = *constraint;
							frictionDescPtr++;
						}
						headers->startIndex = fricI;
						headers->stride = stride;
						headers->constraintType = type;
						headers++;
						fricHeaders++;
						fricI += stride;
					}
					else if(_header.constraintType == DY_SC_TYPE_BLOCK_RB_CONTACT || _header.constraintType == DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT)
					{
						//KS - TODO - Extract block of 4 contacts from this constraint. This isn't implemented yet for coulomb friction model
						PX_ASSERT(contactDescBegin[_header.startIndex].constraint);
						SolverContactCoulombHeader4* head = reinterpret_cast<SolverContactCoulombHeader4*>(contactDescBegin[_header.startIndex].constraint);
						PxU32 frictionOffset = head->frictionOffset;
						PxU8* PX_RESTRICT constraint = reinterpret_cast<PxU8*>(head) + frictionOffset;
						const PxU32 origLength = getConstraintLength(contactDescBegin[_header.startIndex]);
						const PxU32 length = (origLength - frictionOffset);
						PxU8 type = *constraint;
						PX_ASSERT(type == DY_SC_TYPE_BLOCK_FRICTION || type == DY_SC_TYPE_BLOCK_STATIC_FRICTION);
						for(PxU32 b = 0; b < 4; ++b)
						{
							PxSolverConstraintDesc& desc = contactDescBegin[_header.startIndex+b];
							setConstraintLength(*frictionDescPtr, length);
							frictionDescPtr->constraint	= constraint;
							frictionDescPtr->bodyA = desc.bodyA;
							frictionDescPtr->bodyB = desc.bodyB;
							frictionDescPtr->bodyADataIndex = desc.bodyADataIndex;
							frictionDescPtr->bodyBDataIndex = desc.bodyBDataIndex;
							frictionDescPtr->linkIndexA = desc.linkIndexA;
							frictionDescPtr->linkIndexB = desc.linkIndexB;
							frictionDescPtr->writeBack = NULL;
							frictionDescPtr++;
						}
						headers->startIndex = fricI;
						headers->stride = stride;
						headers->constraintType = type;
						headers++;
						fricHeaders++;
						fricI += stride;
					}
				}
				startIndex += numBatchesInK;
				if(startFricH < fricHeaders)
				{
					frictionConstraintsPerPartition.pushBack(fricHeaders - startFricH);
				}
			}
		
			frictionDescCount = PxU32(frictionDescPtr - frictionDescBegin);
			
			mThreadContext.mNumDifferentBodyFrictionConstraints = frictionDescCount;

			frictionHeaderArray.forceSize_Unsafe(PxU32(headers - frictionHeaderArray.begin()));

			mThreadContext.mNumSelfFrictionConstraints = fricI - frictionDescCount; //self constraint count
			mThreadContext.mNumDifferentBodyFrictionConstraints = frictionDescCount;
			frictionDescCount = fricI;
			mThreadContext.mOrderedFrictionDescCount = frictionDescCount;
		}

		{
			{
				PX_PROFILE_ZONE("Dynamics.solver", mContext.getContextId());

				PxSolverConstraintDesc* contactDescs = mThreadContext.orderedContactConstraints;
				PxSolverConstraintDesc* frictionDescs = mThreadContext.frictionConstraintDescArray.begin();

				PxI32* thresholdPairsOut = &mContext.mThresholdStreamOut;

				SolverIslandParams& params = *reinterpret_cast<SolverIslandParams*>(mContext.getTaskPool().allocate(sizeof(SolverIslandParams)));
				params.positionIterations = mThreadContext.mMaxSolverPositionIterations;
				params.velocityIterations = mThreadContext.mMaxSolverVelocityIterations;
				params.bodyListStart = solverBodies;
				params.bodyDataList = solverBodyDatas;
				params.solverBodyOffset = mSolverBodyOffset;
				params.bodyListSize = mIslandContext.mCounts.bodies;
				params.articulationListStart = mThreadContext.getArticulations().begin();
				params.articulationListSize = mThreadContext.getArticulations().size();
				params.constraintList = contactDescs;
				params.constraintIndex = 0;
				params.constraintIndexCompleted = 0;
				params.bodyListIndex = 0;
				params.bodyListIndexCompleted = 0;
				params.articSolveIndex = 0;
				params.articSolveIndexCompleted = 0;
				params.bodyIntegrationListIndex = 0;
				params.thresholdStream = mContext.getThresholdStream().begin();
				params.thresholdStreamLength = mContext.getThresholdStream().size();
				params.outThresholdPairs = thresholdPairsOut;
				params.motionVelocityArray = mThreadContext.motionVelocityArray;
				params.numObjectsIntegrated = 0;
				params.constraintBatchHeaders = mThreadContext.contactConstraintBatchHeaders;
				params.numConstraintHeaders = mThreadContext.numContactConstraintBatches;
				params.headersPerPartition = mThreadContext.mConstraintsPerPartition.begin();
				params.nbPartitions = mThreadContext.mConstraintsPerPartition.size();
				params.rigidBodies = const_cast<PxsRigidBody**>(mObjects.bodies);
				params.frictionHeadersPerPartition = mThreadContext.mFrictionConstraintsPerPartition.begin();
				params.nbFrictionPartitions = mThreadContext.mFrictionConstraintsPerPartition.size();
				params.frictionConstraintBatches = mThreadContext.frictionConstraintBatchHeaders.begin();
				params.numFrictionConstraintHeaders = mThreadContext.frictionConstraintBatchHeaders.size();
				params.frictionConstraintIndex = 0;
				params.frictionConstraintList = frictionDescs;
				params.mMaxArticulationLinks = mThreadContext.mMaxArticulationLinks;
				params.dt = mContext.mDt;
				params.invDt = mContext.mInvDt;

				const PxU32 unrollSize = 8;
				const PxU32 denom = PxMax(1u, (mThreadContext.mMaxPartitions*unrollSize));
				const PxU32 MaxTasks = getTaskManager()->getCpuDispatcher()->getWorkerCount();
				// PT: small improvement: if there's no contacts, use the number of bodies instead.
				// That way the integration work still benefits from multiple tasks.
				const PxU32 numWorkItems = mThreadContext.numContactConstraintBatches ? mThreadContext.numContactConstraintBatches : mIslandContext.mCounts.bodies;
				const PxU32 idealThreads = (numWorkItems+denom-1)/denom;
				const PxU32 numTasks = PxMax(1u, PxMin(idealThreads, MaxTasks));
				
				if(numTasks > 1)
				{
					const PxU32 idealBatchSize = PxMax(unrollSize, idealThreads*unrollSize/(numTasks*2));

					params.batchSize = idealBatchSize; //assigning ideal batch size for the solver to grab work at. Only needed by the multi-threaded island solver.

					for(PxU32 a = 1; a < numTasks; ++a)
					{
						void* tsk = mContext.getTaskPool().allocate(sizeof(PxsParallelSolverTask));
						PxsParallelSolverTask* pTask = PX_PLACEMENT_NEW(tsk, PxsParallelSolverTask)(
							params, mContext, mIslandSim);

						//Force to complete before merge task!
						pTask->setContinuation(mCont);
						
						pTask->removeReference();
					}

					//Avoid kicking off one parallel task when we can do the work inline in this function
					{						
						PX_PROFILE_ZONE("Dynamics.parallelSolve", mContext.getContextId());

						solveParallel(mContext, params, mIslandSim);
					}
					const PxI32 numBodiesPlusArtics = PxI32( mIslandContext.mCounts.bodies + mIslandContext.mCounts.articulations );

					PxI32* numObjectsIntegrated = &params.numObjectsIntegrated;

					WAIT_FOR_PROGRESS_NO_TIMER(numObjectsIntegrated, numBodiesPlusArtics);
				}
				else
				{				
					mThreadContext.mDeltaV.forceSize_Unsafe(0);
					mThreadContext.mDeltaV.reserve(mThreadContext.mMaxArticulationLinks);
					mThreadContext.mDeltaV.forceSize_Unsafe(mThreadContext.mMaxArticulationLinks);

					params.deltaV = mThreadContext.mDeltaV.begin();
					params.errorAccumulator = mContext.isResidualReportingEnabled() ? &mThreadContext.getSimStats().contactErrorAccumulator : NULL;

					//Only one task - a small island so do a sequential solve (avoid the atomic overheads)
					mContext.mSolverCore[mContext.getFrictionType()]->solveV_Blocks(params);

					PxSolverBodyData* solverBodyData2 = solverBodyDatas + mSolverBodyOffset + 1;
					integrate(mIslandSim, solverBodyData2, mObjects.bodies, mThreadContext.motionVelocityArray, solverBodies, mIslandContext.mCounts.bodies, mContext.mDt, mContext.mEnableStabilization);

					for(PxU32 cnt=0;cnt<mIslandContext.mCounts.articulations;cnt++)
					{
						ArticulationSolverDesc &d = mThreadContext.getArticulations()[cnt];
						//PX_PROFILE_ZONE("Articulations.integrate", mContext.getContextId());

						ArticulationPImpl::updateBodies(d, mThreadContext.mDeltaV.begin(), mContext.getDt());
					}
				}
			}
		}
	}

	virtual const char* getName() const { return "PxsDynamics.solverSetupSolve"; }

	DynamicsContext&			mContext;
	IslandContext&				mIslandContext;
	const SolverIslandObjects	mObjects;
	const PxU32					mSolverBodyOffset;
	IG::IslandSim&				mIslandSim;
};

class PxsSolverEndTask : public Cm::Task
{
	PxsSolverEndTask& operator=(const PxsSolverEndTask&);
public:

	PxsSolverEndTask(DynamicsContext& context, IslandContext& islandContext, const SolverIslandObjects& objects, PxU32 solverBodyOffset, PxsContactManagerOutputIterator& cmOutputs) :
		Cm::Task			(context.getContextId()),
		mContext			(context), 
		mIslandContext		(islandContext),
		mObjects			(objects),
		mSolverBodyOffset	(solverBodyOffset),
		mOutputs			(cmOutputs)
	{}

	virtual void runInternal()
	{		
		PX_PROFILE_ZONE("Dynamics.endTask", getContextId());
		ThreadContext& mThreadContext = *mIslandContext.mThreadContext;
#if PX_ENABLE_SIM_STATS
		mThreadContext.getSimStats().numAxisSolverConstraints += mThreadContext.mAxisConstraintCount;
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
		//Patch up the contact managers (TODO - fix up force writeback)
		PxU32 numCompoundConstraints = mThreadContext.compoundConstraints.size();
		for(PxU32 i = 0; i < numCompoundConstraints; ++i)
		{
			CompoundContactManager& manager = mThreadContext.compoundConstraints[i];
			PxsContactManagerOutput* cmOutput = manager.cmOutput;

			PxReal* contactForces = reinterpret_cast<PxReal*>(cmOutput->contactForces);
			PxU32 contactCount = cmOutput->nbContacts;

			cmOutput->contactPatches = manager.originalContactPatches;
			cmOutput->contactPoints = manager.originalContactPoints;
			cmOutput->nbContacts = manager.originalContactCount;
			cmOutput->nbPatches = manager.originalPatchCount;
			cmOutput->statusFlag = manager.originalStatusFlags;
			cmOutput->contactForces = manager.originalForceBuffer;
			cmOutput->frictionPatches = manager.originalFrictionPatches;
			
			for(PxU32 a = 1; a < manager.mStride; ++a)
			{
				PxsContactManager* pManager = mThreadContext.orderedContactList[manager.mStartIndex + a]->contactManager;
				pManager->getWorkUnit().mFrictionDataPtr = manager.unit->mFrictionDataPtr;
				pManager->getWorkUnit().mFrictionPatchCount = manager.unit->mFrictionPatchCount;
				//pManager->getWorkUnit().prevFrictionPatchCount = manager.unit->prevFrictionPatchCount;
			}

			//This is a stride-based contact force writer. The assumption is that we may have skipped certain unimportant contacts reported by the 
			//discrete narrow phase
			if(contactForces)
			{
				PxU32 currentContactIndex = 0;
				PxU32 currentManagerIndex = manager.mStartIndex;
				PxU32 currentManagerContactIndex = 0;

				for(PxU32 a = 0; a < contactCount; ++a)
				{
					PxU32 index = manager.forceBufferList[a];
					PxsContactManager* pManager = mThreadContext.orderedContactList[currentManagerIndex]->contactManager;
					const PxsContactManagerOutput* output = &mOutputs.getContactManagerOutput(pManager->getWorkUnit().mNpIndex);
					while(currentContactIndex < index || output->nbContacts == 0)
					{
						//Step forwards...first in this manager...
						
						PxU32 numToStep = PxMin(index - currentContactIndex, PxU32(output->nbContacts) - currentManagerContactIndex);
						currentContactIndex += numToStep;
						currentManagerContactIndex += numToStep;
						if(currentManagerContactIndex == output->nbContacts)
						{
							currentManagerIndex++;
							currentManagerContactIndex = 0;
							pManager = mThreadContext.orderedContactList[currentManagerIndex]->contactManager;
							output = &mOutputs.getContactManagerOutput(pManager->getWorkUnit().mNpIndex);
						}
					}
					if(output->nbContacts > 0 && output->contactForces)
						output->contactForces[currentManagerContactIndex] = contactForces[a];
				}
			}
		}

		mThreadContext.compoundConstraints.forceSize_Unsafe(0);

		mThreadContext.mConstraintBlockManager.reset();

		mContext.putThreadContext(&mThreadContext);
	}

	virtual const char* getName() const
	{
		return "PxsDynamics.solverEnd";
	}

	DynamicsContext&					mContext;	
	IslandContext&						mIslandContext;
	const SolverIslandObjects			mObjects;
	const PxU32							mSolverBodyOffset;
	PxsContactManagerOutputIterator&	mOutputs;
};

class PxsSolverCreateFinalizeConstraintsTask : public Cm::Task
{
	PxsSolverCreateFinalizeConstraintsTask& operator=(const PxsSolverCreateFinalizeConstraintsTask&);
public:

	PxsSolverCreateFinalizeConstraintsTask(DynamicsContext& context, IslandContext& islandContext, PxU32 solverDataOffset, PxsContactManagerOutputIterator& outputs, bool enhancedDeterminism) : 
		Cm::Task				(context.getContextId()),
		mContext				(context),
		mIslandContext			(islandContext),
		mSolverDataOffset		(solverDataOffset),
		mOutputs				(outputs),
		mEnhancedDeterminism	(enhancedDeterminism)
	{
	}
	
	virtual void runInternal();

	virtual const char* getName() const { return "PxsDynamics.solverCreateFinalizeConstraints"; }

	DynamicsContext&					mContext;
	IslandContext&						mIslandContext;
	const PxU32							mSolverDataOffset;
	PxsContactManagerOutputIterator&	mOutputs;
	const bool							mEnhancedDeterminism;
};

// helper function to join two tasks together and ensure ref counts are correct
static void chainTasks(PxLightCpuTask* first, PxLightCpuTask* next)
{
	first->setContinuation(next);
	next->removeReference();
}

static void createSolverTaskChain(	DynamicsContext& dynamicContext,
									const SolverIslandObjects& objects,				  
									const PxsIslandIndices& counts,
									PxU32 solverBodyOffset, 
									IG::SimpleIslandManager& islandManager, 
									PxU32* bodyRemapTable, PxsMaterialManager* materialManager, PxBaseTask* continuation,
									PxsContactManagerOutputIterator& iterator, bool useEnhancedDeterminism)
{
	Cm::FlushPool& taskPool = dynamicContext.getTaskPool();
	taskPool.lock();

	IslandContext* islandContext = reinterpret_cast<IslandContext*>(taskPool.allocate(sizeof(IslandContext)));
	islandContext->mThreadContext = NULL;
	islandContext->mCounts = counts;

	// create lead task
	PxsSolverStartTask* startTask = PX_PLACEMENT_NEW(taskPool.allocateNotThreadSafe(sizeof(PxsSolverStartTask)), PxsSolverStartTask)(dynamicContext, *islandContext, objects, solverBodyOffset, dynamicContext.getKinematicCount(), 
		islandManager, bodyRemapTable, materialManager, iterator, useEnhancedDeterminism);
	PxsSolverEndTask* endTask = PX_PLACEMENT_NEW(taskPool.allocateNotThreadSafe(sizeof(PxsSolverEndTask)), PxsSolverEndTask)(dynamicContext, *islandContext, objects, solverBodyOffset, iterator);

	PxsSolverCreateFinalizeConstraintsTask* createFinalizeConstraintsTask = PX_PLACEMENT_NEW(taskPool.allocateNotThreadSafe(sizeof(PxsSolverCreateFinalizeConstraintsTask)), PxsSolverCreateFinalizeConstraintsTask)(dynamicContext, *islandContext, solverBodyOffset, iterator, useEnhancedDeterminism);
	PxsSolverSetupSolveTask* setupSolveTask = PX_PLACEMENT_NEW(taskPool.allocateNotThreadSafe(sizeof(PxsSolverSetupSolveTask)), PxsSolverSetupSolveTask)(dynamicContext, *islandContext, objects, solverBodyOffset, islandManager.getAccurateIslandSim());

	PxsSolverConstraintPartitionTask* partitionConstraintsTask = PX_PLACEMENT_NEW(taskPool.allocateNotThreadSafe(sizeof(PxsSolverConstraintPartitionTask)), PxsSolverConstraintPartitionTask)(dynamicContext, *islandContext, objects, solverBodyOffset, useEnhancedDeterminism);

	taskPool.unlock();

	endTask->setContinuation(continuation);

	// set up task chain in reverse order
	chainTasks(setupSolveTask, endTask);
	chainTasks(createFinalizeConstraintsTask, setupSolveTask);
	chainTasks(partitionConstraintsTask, createFinalizeConstraintsTask);
	chainTasks(startTask, partitionConstraintsTask);

	startTask->removeReference();
}

namespace
{
class UpdateContinuationTask : public Cm::Task
{
	DynamicsContext& mContext;
	IG::SimpleIslandManager& mSimpleIslandManager;
	PxBaseTask* mLostTouchTask;
	PxU32 mMaxArticulationLinks;

	PX_NOCOPY(UpdateContinuationTask)
public:

	UpdateContinuationTask(DynamicsContext& context,
		IG::SimpleIslandManager& simpleIslandManager,
		PxBaseTask* lostTouchTask, PxU64 contextID, PxU32 maxLinks) :
		Cm::Task(contextID), mContext(context), mSimpleIslandManager(simpleIslandManager),
		mLostTouchTask(lostTouchTask), mMaxArticulationLinks(maxLinks)
	{
	}

	virtual const char* getName() const { return "UpdateContinuationTask"; }

	virtual void runInternal()
	{
		mContext.updatePostKinematic(mSimpleIslandManager, mCont, mLostTouchTask, mMaxArticulationLinks);
		//Allow lost touch task to run once all tasks have be scheduled
		mLostTouchTask->removeReference();
	}
};

class KinematicCopyTask : public Cm::Task
{
	const PxNodeIndex* const	mKinematicIndices;
	const PxU32					mNbKinematics;
	const IG::IslandSim&		mIslandSim;
	PxSolverBodyData*			mBodyData;

	PX_NOCOPY(KinematicCopyTask)

public:

	static const PxU32 NbKinematicsPerTask = 1024;

	KinematicCopyTask(const PxNodeIndex* const kinematicIndices,
		PxU32 nbKinematics, const IG::IslandSim& islandSim, 
		PxSolverBodyData* datas, PxU64 contextID) : Cm::Task(contextID),
		mKinematicIndices(kinematicIndices), mNbKinematics(nbKinematics),
		mIslandSim(islandSim), mBodyData(datas)
	{
	}

	virtual const char* getName() const { return "KinematicCopyTask"; }

	virtual void runInternal()
	{
		for (PxU32 i = 0; i<mNbKinematics; i++)
		{
			PxsRigidBody* rigidBody = getRigidBodyFromIG(mIslandSim, mKinematicIndices[i]);
			const PxsBodyCore& core = rigidBody->getCore();
			copyToSolverBodyData(core.linearVelocity, core.angularVelocity, core.inverseMass, core.inverseInertia, core.body2World, core.maxPenBias,
				core.maxContactImpulse, mKinematicIndices[i].index(), core.contactReportThreshold, mBodyData[i + 1], core.lockFlags, 0.f,
				core.mFlags & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES);
			rigidBody->saveLastCCDTransform();
		}
	}
};
}

void DynamicsContext::update(PxBaseTask* continuation, PxBaseTask* lostTouchTask,
	PxvNphaseImplementationContext* nphase, PxU32 /*maxPatches*/, PxU32 maxArticulationLinks,
	PxReal dt, const PxVec3& gravity, PxBitMapPinned& /*changedHandleMap*/)
{
	PX_PROFILE_ZONE("Dynamics.solverQueueTasks", mContextID);

	mOutputIterator = nphase->getContactManagerOutputs();

	mDt = dt;
	mInvDt = dt == 0.0f ? 0.0f : 1.0f / dt;
	mGravity = gravity;

	const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

	const PxU32 islandCount = islandSim.getNbActiveIslands();

	const PxU32 activatedContactCount = islandSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);
	const IG::EdgeIndex* const activatingEdges = islandSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER);

	for (PxU32 a = 0; a < activatedContactCount; ++a)
	{
		PxsContactManager* cm = mIslandManager.getContactManager(activatingEdges[a]);
		if (cm)
			cm->getWorkUnit().mFrictionPatchCount = 0; //KS - zero the friction patch count on any activating edges
	}

#if PX_ENABLE_SIM_STATS
	if (islandCount > 0)
	{
		mSimStats.mNbActiveKinematicBodies = islandSim.getNbActiveKinematics();
		mSimStats.mNbActiveDynamicBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
		mSimStats.mNbActiveConstraints = islandSim.getNbActiveEdges(IG::Edge::eCONSTRAINT);
	}
	else
	{
		mSimStats.mNbActiveKinematicBodies = islandSim.getNbActiveKinematics();
		mSimStats.mNbActiveDynamicBodies = 0;
		mSimStats.mNbActiveConstraints = 0;
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	mThresholdStreamOut = 0;

	resetThreadContexts();

	//If there is no work to do then we can do nothing at all.
	if(!islandCount)
		return;

	//Block to make sure it doesn't run before stage2 of update!
	lostTouchTask->addReference();

	UpdateContinuationTask* task = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(UpdateContinuationTask)), UpdateContinuationTask)
		(*this, mIslandManager, lostTouchTask, mContextID, maxArticulationLinks);

	task->setContinuation(continuation);

	//KS - test that world solver body's velocities are finite and 0, then set it to 0.
	//Technically, the velocity should always be 0 but can be stomped if a NAN creeps into the simulation.
	PX_ASSERT(mWorldSolverBody.linearVelocity == PxVec3(0.0f));
	PX_ASSERT(mWorldSolverBody.angularState == PxVec3(0.0f));
	PX_ASSERT(mWorldSolverBody.linearVelocity.isFinite());
	PX_ASSERT(mWorldSolverBody.angularState.isFinite());

	mWorldSolverBody.linearVelocity = mWorldSolverBody.angularState = PxVec3(0.f);

	const PxU32 kinematicCount = islandSim.getNbActiveKinematics();
	const PxNodeIndex* const kinematicIndices = islandSim.getActiveKinematics();
	mKinematicCount = kinematicCount;

	const PxU32 bodyCount = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	const PxU32 numArtics = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);

	{
		if (kinematicCount + bodyCount > mSolverBodyPool.capacity())
		{
			mSolverBodyPool.reserve((kinematicCount + bodyCount + 31) & ~31); // pad out to 32 * 128 = 4k to prevent alloc churn
			mSolverBodyDataPool.reserve((kinematicCount + bodyCount + 31 + 1) & ~31); // pad out to 32 * 128 = 4k to prevent alloc churn
			mSolverBodyRemapTable.reserve((kinematicCount + bodyCount + 31 + 1) & ~31);
		}

		{
			PxSolverBody emptySolverBody;
			PxMemZero(&emptySolverBody, sizeof(PxSolverBody));
			mSolverBodyPool.resize(kinematicCount + bodyCount, emptySolverBody);
			PxSolverBodyData emptySolverBodyData;
			PxMemZero(&emptySolverBodyData, sizeof(PxSolverBodyData));
			mSolverBodyDataPool.resize(kinematicCount + bodyCount + 1, emptySolverBodyData);
			mSolverBodyRemapTable.resize(bodyCount);
		}

		// integrate and copy all the kinematics - overkill, since not all kinematics
		// need solver bodies

		mSolverBodyDataPool[0] = mWorldSolverBodyData;

		if(kinematicCount)
		{
			PX_PROFILE_ZONE("Dynamics.updateKinematics", mContextID);
			PxMemZero(mSolverBodyPool.begin(), kinematicCount * sizeof(PxSolverBody));
			for (PxU32 i = 0; i < kinematicCount; i+= KinematicCopyTask::NbKinematicsPerTask)
			{
				const PxU32 nbToProcess = PxMin(KinematicCopyTask::NbKinematicsPerTask, kinematicCount - i);

				KinematicCopyTask* copyTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(KinematicCopyTask)), KinematicCopyTask)
					(&kinematicIndices[i], nbToProcess, islandSim, &mSolverBodyDataPool[i], mContextID);
				
				copyTask->setContinuation(task);

				copyTask->removeReference();
			}
		}
	}

	//Resize arrays of solver constraints...

	const PxU32 numArticulationConstraints = numArtics* maxArticulationLinks; //Just allocate enough memory to fit worst-case maximum size articulations...

	const PxU32 nbActiveContactManagers = islandSim.getNbActiveEdges(IG::Edge::eCONTACT_MANAGER);
	const PxU32 nbActiveConstraints = islandSim.getNbActiveEdges(IG::Edge::eCONSTRAINT);

	const PxU32 totalConstraintCount = nbActiveConstraints + nbActiveContactManagers + numArticulationConstraints;

	mSolverConstraintDescPool.forceSize_Unsafe(0);
	mSolverConstraintDescPool.reserve((totalConstraintCount + 63) & (~63));
	mSolverConstraintDescPool.forceSize_Unsafe(totalConstraintCount);

	mOrderedSolverConstraintDescPool.forceSize_Unsafe(0);
	mOrderedSolverConstraintDescPool.reserve((totalConstraintCount + 63) & (~63));
	mOrderedSolverConstraintDescPool.forceSize_Unsafe(totalConstraintCount);

	mTempSolverConstraintDescPool.forceSize_Unsafe(0);
	mTempSolverConstraintDescPool.reserve((totalConstraintCount + 63) & (~63));
	mTempSolverConstraintDescPool.forceSize_Unsafe(totalConstraintCount);

	mContactConstraintBatchHeaders.forceSize_Unsafe(0);
	mContactConstraintBatchHeaders.reserve((totalConstraintCount + 63) & (~63));
	mContactConstraintBatchHeaders.forceSize_Unsafe(totalConstraintCount);

	mContactList.forceSize_Unsafe(0);
	mContactList.reserve((nbActiveContactManagers + 63u) & (~63u));
	mContactList.forceSize_Unsafe(nbActiveContactManagers);

	mMotionVelocityArray.forceSize_Unsafe(0);
	mMotionVelocityArray.reserve((bodyCount + 63u) & (~63u));
	mMotionVelocityArray.forceSize_Unsafe(bodyCount);

	mBodyCoreArray.forceSize_Unsafe(0);
	mBodyCoreArray.reserve((bodyCount + 63u) & (~63u));
	mBodyCoreArray.forceSize_Unsafe(bodyCount);

	mRigidBodyArray.forceSize_Unsafe(0);
	mRigidBodyArray.reserve((bodyCount + 63u) & (~63u));
	mRigidBodyArray.forceSize_Unsafe(bodyCount);

	mArticulationArray.forceSize_Unsafe(0);
	mArticulationArray.reserve((numArtics + 63u) & (~63u));
	mArticulationArray.forceSize_Unsafe(numArtics);

	mNodeIndexArray.forceSize_Unsafe(0);
	mNodeIndexArray.reserve((bodyCount + 63u) & (~63u));
	mNodeIndexArray.forceSize_Unsafe(bodyCount);

	ThresholdStream& stream = getThresholdStream();
	stream.forceSize_Unsafe(0);
	stream.reserve(PxNextPowerOfTwo(nbActiveContactManagers != 0 ? nbActiveContactManagers - 1 : nbActiveContactManagers));

	//flip exceeded force threshold buffer
	mCurrentIndex = 1 - mCurrentIndex;

	task->removeReference();
}

void DynamicsContext::updatePostKinematic(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* /*continuation*/, PxBaseTask* lostTouchTask, PxU32 maxLinks)
{
	const IG::IslandSim& islandSim = simpleIslandManager.getAccurateIslandSim();

	const PxU32 islandCount = islandSim.getNbActiveIslands();

	PxU32 constraintIndex = 0;

	PxU32 solverBatchMax = mSolverBatchSize;
	PxU32 articulationBatchMax = mSolverArticBatchSize;
	PxU32 minimumConstraintCount = 1;

	//create force threshold tasks to produce force change events
	PxsForceThresholdTask* forceThresholdTask = PX_PLACEMENT_NEW(getTaskPool().allocate(sizeof(PxsForceThresholdTask)), PxsForceThresholdTask)(*this);
	forceThresholdTask->setContinuation(lostTouchTask);

	const IG::IslandId*const islandIds = islandSim.getActiveIslands();

	PxU32 currentIsland = 0;
	PxU32 currentBodyIndex = 0;
	PxU32 currentArticulation = 0;
	PxU32 currentContact = 0;

	while(currentIsland < islandCount)
	{
		SolverIslandObjects objectStarts;
		objectStarts.articulations				= mArticulationArray.begin() + currentArticulation;
		objectStarts.bodies						= mRigidBodyArray.begin() + currentBodyIndex;
		objectStarts.contactManagers			= mContactList.begin() + currentContact;
		objectStarts.constraintDescs			= mSolverConstraintDescPool.begin() + constraintIndex;
		objectStarts.orderedConstraintDescs		= mOrderedSolverConstraintDescPool.begin() + constraintIndex;
		objectStarts.tempConstraintDescs		= mTempSolverConstraintDescPool.begin() + constraintIndex;
		objectStarts.constraintBatchHeaders		= mContactConstraintBatchHeaders.begin() + constraintIndex;
		objectStarts.motionVelocities			= mMotionVelocityArray.begin() + currentBodyIndex;
		objectStarts.bodyCoreArray				= mBodyCoreArray.begin() + currentBodyIndex;
		objectStarts.islandIds					= islandIds + currentIsland;
		objectStarts.bodyRemapTable				= mSolverBodyRemapTable.begin();
		objectStarts.nodeIndexArray				= mNodeIndexArray.begin() + currentBodyIndex;

		PxU32 startIsland = currentIsland;
		PxU32 constraintCount = 0;

		PxU32 nbArticulations = 0;
		PxU32 nbBodies = 0;
		PxU32 nbConstraints = 0;
		PxU32 nbContactManagers =0;

		// islandSim.checkInternalConsistency();

		//KS - logic is a bit funky here. We will keep rolling the island together provided currentIsland < islandCount AND either we haven't exceeded the max number of bodies or we have
		//zero constraints AND we haven't exceeded articulation batch counts (it's still currently beneficial to keep articulations in separate islands but this is only temporary).
		while((currentIsland < islandCount && (nbBodies < solverBatchMax || constraintCount < minimumConstraintCount)) && 
			nbArticulations < articulationBatchMax)
		{
			const IG::Island& island = islandSim.getIsland(islandIds[currentIsland]);
			nbBodies += island.mNodeCount[IG::Node::eRIGID_BODY_TYPE];
			nbArticulations += island.mNodeCount[IG::Node::eARTICULATION_TYPE];
			nbConstraints += island.mEdgeCount[IG::Edge::eCONSTRAINT];
			nbContactManagers += island.mEdgeCount[IG::Edge::eCONTACT_MANAGER];
			constraintCount = nbConstraints + nbContactManagers;
			currentIsland++;
		}

		objectStarts.numIslands = currentIsland - startIsland;

		constraintIndex += nbArticulations* maxLinks;

		PxsIslandIndices counts;
		
		counts.articulations	= nbArticulations;
		counts.bodies			= nbBodies;

		counts.constraints		= nbConstraints;
		counts.contactManagers	= nbContactManagers;
		if(counts.articulations + counts.bodies > 0)
		{
			createSolverTaskChain(*this, objectStarts, counts, 
				mKinematicCount + currentBodyIndex, simpleIslandManager, mSolverBodyRemapTable.begin(), mMaterialManager,
				forceThresholdTask, mOutputIterator, mUseEnhancedDeterminism);
		}

		currentBodyIndex += nbBodies;
		currentArticulation += nbArticulations;
		currentContact += nbContactManagers;

		constraintIndex += constraintCount;
	}

	//kick off forceThresholdTask
	forceThresholdTask->removeReference();
}

void DynamicsContext::mergeResults()
{
	PX_PROFILE_ZONE("Dynamics.solverMergeResults", mContextID);
	//OK. Sum up sim stats here...

#if PX_ENABLE_SIM_STATS	
	{
		PxcThreadCoherentCacheIterator<ThreadContext, PxcNpMemBlockPool> threadContextIt(mThreadContextPool);
		ThreadContext* threadContext = threadContextIt.getNext();

		mTotalContactError.reset();
		mContactErrorPosIter = &mTotalContactError.mPositionIterationErrorAccumulator;
		mContactErrorVelIter = &mTotalContactError.mVelocityIterationErrorAccumulator;

		while (threadContext != NULL)
		{
			ThreadContext::ThreadSimStats& threadStats = threadContext->getSimStats();
			addThreadStats(threadStats);

			if (mIsResidualReportingEnabled)
			{
				mTotalContactError.combine(threadStats.contactErrorAccumulator);
				threadStats.contactErrorAccumulator.reset();
			}

			threadStats.clear();
			threadContext = threadContextIt.getNext();
		}
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
}

static void preIntegrationParallel(
	PxF32 dt,
	PxsBodyCore*const* bodyArray,					// INOUT: core body attributes
	PxsRigidBody*const* originalBodyArray,			// IN: original bodies
	PxU32 const* nodeIndexArray,					// IN: island node index
	PxU32 bodyCount,								// IN: body count
	PxSolverBodyData* solverBodyDataPool,			// IN: solver body data pool (space preallocated)
	volatile PxU32* maxSolverPositionIterations,
	volatile PxU32* maxSolverVelocityIterations,
	const PxVec3& gravity)
{
	PxU32 localMaxPosIter = 0;
	PxU32 localMaxVelIter = 0;

	for(PxU32 a = 1; a < bodyCount; ++a)
	{
		PxU32 i = a-1;
		PxPrefetchLine(bodyArray[a]);
		PxPrefetchLine(bodyArray[a],128);
		PxPrefetchLine(&solverBodyDataPool[a]);
		PxPrefetchLine(&solverBodyDataPool[a],128);

		PxsBodyCore& core = *bodyArray[i];
		const PxsRigidBody& rBody = *originalBodyArray[i];
		
		const PxU16 iterWord = core.solverIterationCounts;
		localMaxPosIter = PxMax<PxU32>(PxU32(iterWord & 0xff), localMaxPosIter);
		localMaxVelIter = PxMax<PxU32>(PxU32(iterWord >> 8), localMaxVelIter);

		//const Cm::SpatialVector& accel = originalBodyArray[i]->getAccelerationV();
		bodyCoreComputeUnconstrainedVelocity(gravity, dt, core.linearDamping, core.angularDamping, rBody.mAccelScale, core.maxLinearVelocitySq, core.maxAngularVelocitySq, 
			core.linearVelocity, core.angularVelocity, core.disableGravity!=0);

		copyToSolverBodyData(core.linearVelocity, core.angularVelocity, core.inverseMass, core.inverseInertia, core.body2World, core.maxPenBias, core.maxContactImpulse, nodeIndexArray[i], 
			core.contactReportThreshold, solverBodyDataPool[i + 1], core.lockFlags, dt, core.mFlags & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES);
	}
	const PxU32 i = bodyCount - 1;
	PxsBodyCore& core = *bodyArray[i];
	const PxsRigidBody& rBody = *originalBodyArray[i];
		
	PxU16 iterWord = core.solverIterationCounts;
	localMaxPosIter = PxMax<PxU32>(PxU32(iterWord & 0xff), localMaxPosIter);
	localMaxVelIter = PxMax<PxU32>(PxU32(iterWord >> 8), localMaxVelIter);

	bodyCoreComputeUnconstrainedVelocity(gravity, dt, core.linearDamping, core.angularDamping, rBody.mAccelScale, core.maxLinearVelocitySq, core.maxAngularVelocitySq,
		core.linearVelocity, core.angularVelocity, core.disableGravity!=0);

	copyToSolverBodyData(core.linearVelocity, core.angularVelocity, core.inverseMass, core.inverseInertia, core.body2World, core.maxPenBias, core.maxContactImpulse, nodeIndexArray[i], 
		core.contactReportThreshold, solverBodyDataPool[i + 1], core.lockFlags, dt, core.mFlags & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES);

	physx::PxAtomicMax(reinterpret_cast<volatile PxI32*>(maxSolverPositionIterations), PxI32(localMaxPosIter));
	physx::PxAtomicMax(reinterpret_cast<volatile PxI32*>(maxSolverVelocityIterations), PxI32(localMaxVelIter));
}

void PxsPreIntegrateTask::runInternal()
{
	PX_PROFILE_ZONE("PreIntegration", mContext.getContextId());
	preIntegrationParallel(mDt, mBodyArray + mStartIndex, mOriginalBodyArray + mStartIndex, mNodeIndexArray + mStartIndex, mNumToIntegrate,
						mSolverBodyDataPool + mStartIndex, mMaxSolverPositionIterations, mMaxSolverVelocityIterations, mGravity);
}

void DynamicsContext::preIntegrationParallel(
	PxF32 dt,
	PxsBodyCore*const* bodyArray,					// INOUT: core body attributes
	PxsRigidBody*const* originalBodyArray,			// IN: original bodies
	PxU32 const* nodeIndexArray,					// IN: island node index
	PxU32 bodyCount,								// IN: body count
	PxSolverBody* solverBodyPool,					// IN: solver body pool (space preallocated)
	PxSolverBodyData* solverBodyDataPool,			// IN: solver body data pool (space preallocated)
	Cm::SpatialVector* /*motionVelocityArray*/,		// OUT: motion velocities
	PxU32& maxSolverPositionIterations,
	PxU32& maxSolverVelocityIterations,
	PxBaseTask& task
	)
{
	//TODO - make this based on some variables so we can try different configurations
	const PxU32 IntegrationPerThread = 256;

	const PxU32 numTasks = ((bodyCount + IntegrationPerThread-1)/IntegrationPerThread);
	const PxU32 taskBatchSize = 64;

	for(PxU32 i = 0; i < numTasks; i+=taskBatchSize)
	{
		const PxU32 nbTasks = PxMin(numTasks - i, taskBatchSize);
		PxsPreIntegrateTask* tasks = reinterpret_cast<PxsPreIntegrateTask*>(getTaskPool().allocate(sizeof(PxsPreIntegrateTask)*nbTasks));
		for(PxU32 a = 0; a < nbTasks; ++a)
		{
			PxU32 startIndex = (i+a)*IntegrationPerThread;
			PxU32 nbToIntegrate = PxMin((bodyCount-startIndex), IntegrationPerThread);
			PxsPreIntegrateTask* pTask = PX_PLACEMENT_NEW(&tasks[a], PxsPreIntegrateTask)(*this, bodyArray,
							originalBodyArray, nodeIndexArray, solverBodyDataPool, dt, bodyCount,
							&maxSolverPositionIterations, &maxSolverVelocityIterations, startIndex, 
							nbToIntegrate, mGravity);

			pTask->setContinuation(&task);
			pTask->removeReference();
		}
	}

	PxMemZero(solverBodyPool, bodyCount * sizeof(PxSolverBody));
}

void solveParallel(SOLVER_PARALLEL_METHOD_ARGS)
{
	Dy::ThreadContext& threadContext = *context.getThreadContext();

	threadContext.mDeltaV.forceSize_Unsafe(0);
	threadContext.mDeltaV.reserve(params.mMaxArticulationLinks);
	threadContext.mDeltaV.forceSize_Unsafe(params.mMaxArticulationLinks);

	context.solveParallel(params, islandSim, threadContext.mDeltaV.begin(), context.isResidualReportingEnabled() ? &threadContext.getSimStats().contactErrorAccumulator : NULL);

	context.putThreadContext(&threadContext);
}

void DynamicsContext::solveParallel(SolverIslandParams& params, IG::IslandSim& islandSim, 
	Cm::SpatialVectorF* deltaV, Dy::ErrorAccumulatorEx* errorAccumulator)
{
	mSolverCore[mFrictionType]->solveVParallelAndWriteBack(params, deltaV, errorAccumulator);
	integrateCoreParallel(params, deltaV, islandSim);
}

void DynamicsContext::integrateCoreParallel(SolverIslandParams& params, Cm::SpatialVectorF* deltaV, IG::IslandSim& islandSim)
{
	const PxI32 unrollCount = 128;

	PxI32* bodyIntegrationListIndex = &params.bodyIntegrationListIndex;

	PxI32 index = physx::PxAtomicAdd(bodyIntegrationListIndex, unrollCount) - unrollCount;

	const PxI32 numBodies = PxI32(params.bodyListSize);
	const PxI32 numArtics = PxI32(params.articulationListSize);

	Cm::SpatialVector* PX_RESTRICT motionVelocityArray = params.motionVelocityArray;
	PxsRigidBody** PX_RESTRICT rigidBodies = params.rigidBodies;
	ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;

	PxI32 numIntegrated = 0;

	PxI32 bodyRemainder = unrollCount;

	while(index < numArtics)
	{
		const PxI32 remainder = PxMin(numArtics - index, unrollCount);
		bodyRemainder -= remainder;

		for(PxI32 a = 0; a < remainder; ++a, index++)
		{
			const PxI32 i = index;
			{
				//PX_PROFILE_ZONE("Articulations.integrate", mContextID);

				ArticulationPImpl::updateBodies(articulationListStart[i], deltaV, mDt);
			}

			++numIntegrated;
		}
		if(bodyRemainder == 0)
		{
			index = physx::PxAtomicAdd(bodyIntegrationListIndex, unrollCount) - unrollCount;
			bodyRemainder = unrollCount;
		}
	}	

	index -= numArtics;

	const PxI32 unrollPlusArtics = unrollCount + numArtics;

	PxSolverBody* PX_RESTRICT solverBodies = const_cast<PxSolverBody*>(params.bodyListStart);
	PxSolverBodyData* PX_RESTRICT solverBodyData = params.bodyDataList + params.solverBodyOffset+1;

	while(index < numBodies)
	{
		const PxI32 remainder = PxMin(numBodies - index, bodyRemainder);
		bodyRemainder -= remainder;
		integrate(islandSim, solverBodyData + index, rigidBodies + index, motionVelocityArray + index, solverBodies + index, remainder, mDt, mEnableStabilization);
		numIntegrated += remainder;

		{
			index = physx::PxAtomicAdd(bodyIntegrationListIndex, unrollCount) - unrollPlusArtics;
			bodyRemainder = unrollCount;
		}
	}

	PxMemoryBarrier();
	physx::PxAtomicAdd(&params.numObjectsIntegrated, numIntegrated);
}

static PxU32 createFinalizeContacts_Parallel(PxSolverBodyData* solverBodyData, ThreadContext& mThreadContext, DynamicsContext& context,
									  PxU32 startIndex, PxU32 endIndex, PxsContactManagerOutputIterator& outputs)
{
	PX_PROFILE_ZONE("createFinalizeContacts_Parallel", context.getContextId());
	const PxFrictionType::Enum frictionType = context.getFrictionType();
	const PxReal correlationDist = context.getCorrelationDistance();
	const PxReal bounceThreshold = context.getBounceThreshold();
	const PxReal frictionOffsetThreshold = context.getFrictionOffsetThreshold();
	const PxReal dt = context.getDt();
	const PxReal invDt = PxMin(context.getMaxBiasCoefficient(), context.getInvDt());

	PxSolverConstraintDesc* contactDescPtr = mThreadContext.orderedContactConstraints;

	PxConstraintBatchHeader* headers = mThreadContext.contactConstraintBatchHeaders;
	
	PxI32 axisConstraintCount = 0;
	ThreadContext* threadContext = context.getThreadContext();
	threadContext->mConstraintBlockStream.reset(); //ensure there's no left-over memory that belonged to another island

	threadContext->mZVector.forceSize_Unsafe(0);
	threadContext->mZVector.reserve(mThreadContext.mMaxArticulationLinks);
	threadContext->mZVector.forceSize_Unsafe(mThreadContext.mMaxArticulationLinks);

	//threadContext->mDeltaV.forceSize_Unsafe(0);
	//threadContext->mDeltaV.reserve(mThreadContext.mMaxArticulationLinks);
	//threadContext->mDeltaV.forceSize_Unsafe(mThreadContext.mMaxArticulationLinks);

	Cm::SpatialVectorF* Z = threadContext->mZVector.begin();

	const PxTransform idt(PxIdentity);

	BlockAllocator blockAllocator(mThreadContext.mConstraintBlockManager, threadContext->mConstraintBlockStream, threadContext->mFrictionPatchStreamPair, threadContext->mConstraintSize);

	const PxReal ccdMaxSeparation = context.getCCDSeparationThreshold();

	for(PxU32 a = startIndex; a < endIndex; ++a)
	{
		PxConstraintBatchHeader& header = headers[a];

		if(contactDescPtr[header.startIndex].constraintType == DY_SC_TYPE_RB_CONTACT)
		{
			PxSolverContactDesc blockDescs[4];
			PxsContactManagerOutput* cmOutputs[4];
			PxsContactManager* cms[4];
			for (PxU32 i = 0; i < header.stride; ++i)
			{
				PxSolverConstraintDesc& desc = contactDescPtr[header.startIndex + i];
				PxSolverContactDesc& blockDesc = blockDescs[i];
				PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);

				cms[i] = cm;

				PxcNpWorkUnit& unit = cm->getWorkUnit();

				cmOutputs[i] = &outputs.getContactManagerOutput(unit.mNpIndex);

				PxSolverBodyData& data0 = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyADataIndex];
				PxSolverBodyData& data1 = desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyBDataIndex];

				blockDesc.data0 = &data0;
				blockDesc.data1 = &data1;

				PxU8 flags = unit.mRigidCore0->mFlags;
				if (unit.mRigidCore1)
					flags |= PxU8(unit.mRigidCore1->mFlags);

				blockDesc.bodyFrame0 = unit.mRigidCore0->body2World;
				blockDesc.bodyFrame1 = unit.mRigidCore1 ? unit.mRigidCore1->body2World : idt;
				blockDesc.shapeInteraction = cm->getShapeInteraction();
				blockDesc.contactForces = cmOutputs[i]->contactForces;
				blockDesc.desc = &desc;
				blockDesc.body0 = desc.bodyA;
				blockDesc.body1 = desc.bodyB;
				blockDesc.hasForceThresholds = !!(unit.mFlags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);
				blockDesc.disableStrongFriction = !!(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION);
				blockDesc.bodyState0 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? PxSolverContactDesc::eARTICULATION : PxSolverContactDesc::eDYNAMIC_BODY;
				//second body is articulation
				if (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY1)
				{
					//kinematic link
					if (desc.linkIndexB == 0xff)
					{
						blockDesc.bodyState1 = PxSolverContactDesc::eSTATIC_BODY;
					}
					else
					{
						blockDesc.bodyState1 = PxSolverContactDesc::eARTICULATION;
					}
				}
				else
				{
					blockDesc.bodyState1 = (unit.mFlags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR) ? PxSolverContactDesc::eKINEMATIC_BODY :
						((unit.mFlags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1) ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eSTATIC_BODY);
				}
				//blockDesc.flags = unit.flags;

				const PxReal dominance0 = unit.mDominance0 ? 1.f : 0.f;
				const PxReal dominance1 = unit.mDominance1 ? 1.f : 0.f;

				blockDesc.invMassScales.linear0 = blockDesc.invMassScales.angular0 = dominance0;
				blockDesc.invMassScales.linear1 = blockDesc.invMassScales.angular1 = dominance1;
				blockDesc.restDistance = unit.mRestDistance;
				blockDesc.frictionPtr = unit.mFrictionDataPtr;
				blockDesc.frictionCount = unit.mFrictionPatchCount;
				blockDesc.maxCCDSeparation = (flags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD) ? ccdMaxSeparation : PX_MAX_F32;
				blockDesc.offsetSlop = unit.mOffsetSlop;
			}

#if DY_BATCH_CONSTRAINTS
			SolverConstraintPrepState::Enum state = SolverConstraintPrepState::eUNBATCHABLE;
			if(header.stride == 4)
			{
				//KS - todo - plumb in axisConstraintCount into this method to keep track of the number of axes
				state = createFinalizeMethods4[frictionType](cmOutputs, *threadContext,
					 blockDescs,
					 invDt,
					 dt,
					 bounceThreshold,
					 frictionOffsetThreshold,
					 correlationDist,
					 blockAllocator);
			}
			if(SolverConstraintPrepState::eSUCCESS != state)
#endif
			{
				for(PxU32 i = 0; i < header.stride; ++i)
				{
					PxSolverConstraintDesc& desc = contactDescPtr[header.startIndex+i];
					PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);
					PxsContactManagerOutput& output = *cmOutputs[i];

					createFinalizeMethods[frictionType](blockDescs[i], output, *threadContext,
						invDt, dt, bounceThreshold, frictionOffsetThreshold, correlationDist,
						blockAllocator, Z);
			
					getContactManagerConstraintDesc(output,*cm,desc);
				}
			}

			for (PxU32 i = 0; i < header.stride; ++i)
			{
				updateFrictionAnchorCountAndPosition(contactDescPtr[header.startIndex + i], *cmOutputs[i], blockDescs[i]);
			}

			for (PxU32 i = 0; i < header.stride; ++i)
			{
				PxsContactManager* cm = cms[i];

				PxcNpWorkUnit& unit = cm->getWorkUnit();
				unit.mFrictionDataPtr = blockDescs[i].frictionPtr;
				unit.mFrictionPatchCount = blockDescs[i].frictionCount;
				axisConstraintCount += blockDescs[i].axisConstraintCount;
			}
		}
		else if(contactDescPtr[header.startIndex].constraintType == DY_SC_TYPE_RB_1D)
		{
			SolverConstraintShaderPrepDesc shaderDescs[4];
			PxSolverConstraintPrepDesc descs[4];

			for (PxU32 i = 0; i < header.stride; ++i)
			{
				PxSolverConstraintDesc& desc = contactDescPtr[header.startIndex + i];
				const Constraint* constraint = reinterpret_cast<const Constraint*>(desc.constraint);

				SolverConstraintShaderPrepDesc& shaderPrepDesc = shaderDescs[i];
				PxSolverConstraintPrepDesc& prepDesc = descs[i];

				const PxConstraintSolverPrep solverPrep = constraint->solverPrep;
				const void* constantBlock = constraint->constantBlock;
				const PxU32 constantBlockByteSize = constraint->constantBlockSize;
				const PxTransform& pose0 = (constraint->body0 ? constraint->body0->getPose() : idt);
				const PxTransform& pose1 = (constraint->body1 ? constraint->body1->getPose() : idt);
				const PxSolverBody* sbody0 = desc.bodyA;
				const PxSolverBody* sbody1 = desc.bodyB;
				PxSolverBodyData* sbodyData0 = &solverBodyData[desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY ? desc.bodyADataIndex : 0];
				PxSolverBodyData* sbodyData1 = &solverBodyData[desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY ? desc.bodyBDataIndex : 0];

				shaderPrepDesc.constantBlock = constantBlock;
				shaderPrepDesc.constantBlockByteSize = constantBlockByteSize;
				shaderPrepDesc.constraint = constraint;
				shaderPrepDesc.solverPrep = solverPrep;

				prepDesc.desc = &desc;
				prepDesc.bodyFrame0 = pose0;
				prepDesc.bodyFrame1 = pose1;
				prepDesc.data0 = sbodyData0;
				prepDesc.data1 = sbodyData1;
				prepDesc.body0 = sbody0;
				prepDesc.body1 = sbody1;
				prepDesc.linBreakForce = constraint->linBreakForce;
				prepDesc.angBreakForce = constraint->angBreakForce;
				prepDesc.writeback = &context.getConstraintWriteBackPool()[constraint->index];
				setupConstraintFlags(prepDesc, constraint->flags);
				prepDesc.minResponseThreshold = constraint->minResponseThreshold;
			}

#if DY_BATCH_CONSTRAINTS && DY_BATCH_1D
			SolverConstraintPrepState::Enum state = SolverConstraintPrepState::eUNBATCHABLE;
			if(header.stride == 4)
			{
				PxU32 totalRows;
				state = setupSolverConstraint4
					(shaderDescs, descs, dt, invDt, totalRows,
					blockAllocator, context.isResidualReportingEnabled());

				axisConstraintCount += totalRows;
			}
			if(state != SolverConstraintPrepState::eSUCCESS)
#endif
			{
				for(PxU32 i = 0; i < header.stride; ++i)
				{
					axisConstraintCount += SetupSolverConstraint(shaderDescs[i], descs[i], blockAllocator, dt, invDt);
				}
			}
		}
	}

#if PX_ENABLE_SIM_STATS
	threadContext->getSimStats().numAxisSolverConstraints += axisConstraintCount;
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

	context.putThreadContext(threadContext);
	return PxU32(axisConstraintCount); //Can't write to mThreadContext as it's shared!!!!
}

class PxsCreateFinalizeContactsTask : public Cm::Task
{
	PxsCreateFinalizeContactsTask& operator=(const PxsCreateFinalizeContactsTask&);
public:
	PxsCreateFinalizeContactsTask( const PxU32 numConstraints, PxSolverConstraintDesc* descArray, PxSolverBodyData* solverBodyData,
		ThreadContext& threadContext, DynamicsContext& context, PxU32 startIndex, PxU32 endIndex, PxsContactManagerOutputIterator& outputs) :
			Cm::Task(context.getContextId()),
			mNumConstraints(numConstraints), mDescArray(descArray), mSolverBodyData(solverBodyData),
			mThreadContext(threadContext), mDynamicsContext(context),
			mOutputs(outputs),
			mStartIndex(startIndex), mEndIndex(endIndex)
	{}

	virtual void runInternal()
	{
		createFinalizeContacts_Parallel(mSolverBodyData, mThreadContext, mDynamicsContext, mStartIndex, mEndIndex, mOutputs);
	}

	virtual const char* getName() const
	{
		return "PxsDynamics.createFinalizeContacts";
	}

public:
	const PxU32 mNumConstraints;
	PxSolverConstraintDesc* mDescArray;
	PxSolverBodyData* mSolverBodyData;
	ThreadContext& mThreadContext;
	DynamicsContext& mDynamicsContext;
	PxsContactManagerOutputIterator& mOutputs;
	PxU32 mStartIndex;
	PxU32 mEndIndex;
};

PxU8* BlockAllocator::reserveConstraintData(const PxU32 size)
{
	mTotalConstraintByteSize += size;
	return mConstraintBlockStream.reserve(size, mConstraintBlockManager);
}

PxU8* BlockAllocator::reserveFrictionData(const PxU32 size)
{
	return mFrictionPatchStreamPair.reserve<PxU8>(size);
}

class PxsCreateArticConstraintsTask : public Cm::Task
{
	PxsCreateArticConstraintsTask& operator=(const PxsCreateArticConstraintsTask&);	

public:

	static const PxU32 NbArticsPerTask = 32;

	PxsCreateArticConstraintsTask(Dy::FeatherstoneArticulation** articulations, const PxU32 nbArticulations, PxSolverBodyData* solverBodyData, ThreadContext& threadContext, DynamicsContext& context,
		PxsContactManagerOutputIterator& outputs) :
		Cm::Task(context.getContextId()),
		mArticulations(articulations),
		mNbArticulations(nbArticulations),
		mSolverBodyData(solverBodyData),
		mThreadContext(threadContext), mDynamicsContext(context),
		mOutputs(outputs)
	{}

	virtual void runInternal()
	{
		const PxReal correlationDist = mDynamicsContext.getCorrelationDistance();
		const PxReal bounceThreshold = mDynamicsContext.getBounceThreshold();
		const PxReal frictionOffsetThreshold = mDynamicsContext.getFrictionOffsetThreshold();
		const PxReal dt = mDynamicsContext.getDt();
		const PxReal invDt = PxMin(mDynamicsContext.getMaxBiasCoefficient(), mDynamicsContext.getInvDt());
		const PxReal ccdMaxSeparation = mDynamicsContext.getCCDSeparationThreshold();

		ThreadContext* threadContext = mDynamicsContext.getThreadContext();
		threadContext->mConstraintBlockStream.reset(); //ensure there's no left-over memory that belonged to another island

		threadContext->mZVector.forceSize_Unsafe(0);
		threadContext->mZVector.reserve(mThreadContext.mMaxArticulationLinks);
		threadContext->mZVector.forceSize_Unsafe(mThreadContext.mMaxArticulationLinks);

		for (PxU32 i = 0; i < mNbArticulations; ++i)
		{
			mArticulations[i]->prepareStaticConstraints(dt, invDt, mOutputs, *threadContext, correlationDist, bounceThreshold, frictionOffsetThreshold,
				ccdMaxSeparation, mSolverBodyData, mThreadContext.mConstraintBlockManager, mDynamicsContext.getConstraintWriteBackPool().begin());
		}

		mDynamicsContext.putThreadContext(threadContext);
	}

	virtual const char* getName() const
	{
		return "PxsDynamics.createFinalizeContacts";
	}

public:

	Dy::FeatherstoneArticulation** mArticulations;
	PxU32 mNbArticulations;
	PxSolverBodyData* mSolverBodyData;
	ThreadContext& mThreadContext;
	DynamicsContext& mDynamicsContext;
	PxsContactManagerOutputIterator& mOutputs;
};

void PxsSolverCreateFinalizeConstraintsTask::runInternal()
{
	PX_PROFILE_ZONE("CreateConstraints", mContext.getContextId());
	ThreadContext& mThreadContext = *mIslandContext.mThreadContext;
	PxU32 descCount = mThreadContext.mNumDifferentBodyConstraints;
	PxU32 selfConstraintDescCount = mThreadContext.contactDescArraySize - (mThreadContext.mNumDifferentBodyConstraints + mThreadContext.mNumStaticConstraints);

	PxArray<PxU32>& accumulatedConstraintsPerPartition = mThreadContext.mConstraintsPerPartition;

	PxU32 numHeaders = 0;
	PxU32 currentPartition = 0;
	PxU32 maxJ = descCount == 0 ? 0 : accumulatedConstraintsPerPartition[0];

	const PxU32 maxBatchPartition = 0xFFFFFFFF;

	const PxU32 maxBatchSize = mEnhancedDeterminism ? 1u : 4u;

	PxU32 headersPerPartition = 0;
	for(PxU32 a = 0; a < descCount;)
	{
		PxU32 loopMax = PxMin(maxJ - a, maxBatchSize);
		PxU16 j = 0;
		if(loopMax > 0)
		{
			PxConstraintBatchHeader& header = mThreadContext.contactConstraintBatchHeaders[numHeaders++];
			
			j=1;
			PxSolverConstraintDesc& desc = mThreadContext.orderedContactConstraints[a];
			if(!isArticulationConstraint(desc) && (desc.constraintType == DY_SC_TYPE_RB_CONTACT || 
				desc.constraintType == DY_SC_TYPE_RB_1D) && currentPartition < maxBatchPartition)
			{
				for(; j < loopMax && desc.constraintType == mThreadContext.orderedContactConstraints[a+j].constraintType && 
					!isArticulationConstraint(mThreadContext.orderedContactConstraints[a+j]); ++j);
			}
			header.startIndex = a;
			header.stride = j;
			headersPerPartition++;
		}
		if(maxJ == (a + j) && maxJ != descCount)
		{
			//Go to next partition!
			accumulatedConstraintsPerPartition[currentPartition] = headersPerPartition;
			headersPerPartition = 0;
			currentPartition++;
			maxJ = accumulatedConstraintsPerPartition[currentPartition];
		}
		a+= j;
	}
	if(descCount)
		accumulatedConstraintsPerPartition[currentPartition] = headersPerPartition;

	accumulatedConstraintsPerPartition.forceSize_Unsafe(mThreadContext.mMaxPartitions);

	PxU32 numDifferentBodyBatchHeaders = numHeaders;

	for(PxU32 a = 0; a < selfConstraintDescCount; ++a)
	{
		PxConstraintBatchHeader& header = mThreadContext.contactConstraintBatchHeaders[numHeaders++];
		header.startIndex = a + descCount;
		header.stride = 1;
	}

	PxU32 numSelfConstraintBatchHeaders = numHeaders - numDifferentBodyBatchHeaders;

	mThreadContext.numDifferentBodyBatchHeaders = numDifferentBodyBatchHeaders;
	mThreadContext.numSelfConstraintBatchHeaders = numSelfConstraintBatchHeaders;
	mThreadContext.numContactConstraintBatches = numHeaders;

	{
		PxSolverConstraintDesc* descBegin = mThreadContext.orderedContactConstraints;

		const PxU32 numThreads = getTaskManager()->getCpuDispatcher()->getWorkerCount();
				
		//Choose an appropriate number of constraint prep tasks. This must be proportionate to the number of constraints to prep and the number
		//of worker threads available.
		const PxU32 TaskBlockSize = 16;
		const PxU32 TaskBlockLargeSize = 64;
		const PxU32 BlockAllocationSize = 64;

		PxU32 numTasks = (numHeaders+TaskBlockLargeSize-1)/TaskBlockLargeSize;

		if(numTasks)
		{
			if(numTasks < numThreads)
				numTasks = PxMax(1u, (numHeaders+TaskBlockSize-1)/TaskBlockSize);

			const PxU32 constraintsPerTask = (numHeaders + numTasks-1)/numTasks;

			for(PxU32 i = 0; i < numTasks; i+=BlockAllocationSize)
			{
				PxU32 blockSize = PxMin(numTasks - i, BlockAllocationSize);

				PxsCreateFinalizeContactsTask* tasks = reinterpret_cast<PxsCreateFinalizeContactsTask*>(mContext.getTaskPool().allocate(sizeof(PxsCreateFinalizeContactsTask)*blockSize));

				for(PxU32 a = 0; a < blockSize; ++a)
				{
					PxU32 startIndex = (a + i) * constraintsPerTask;
					PxU32 endIndex = PxMin(startIndex + constraintsPerTask, numHeaders);
					PxsCreateFinalizeContactsTask* pTask = PX_PLACEMENT_NEW(&tasks[a], PxsCreateFinalizeContactsTask( descCount, descBegin, mContext.mSolverBodyDataPool.begin(), mThreadContext, mContext, startIndex, endIndex, mOutputs));

					pTask->setContinuation(mCont);
					pTask->removeReference();
				}
			}
		}
	}

	const PxU32 articCount = mIslandContext.mCounts.articulations;

	for (PxU32 i = 0; i < articCount; i += PxsCreateArticConstraintsTask::NbArticsPerTask)
	{
		const PxU32 nbToProcess = PxMin(articCount - i, PxsCreateArticConstraintsTask::NbArticsPerTask);

		PxsCreateArticConstraintsTask* task = PX_PLACEMENT_NEW(mContext.getTaskPool().allocate(sizeof(PxsCreateArticConstraintsTask)), PxsCreateArticConstraintsTask)
			(mThreadContext.mArticulationArray + i, nbToProcess, mContext.mSolverBodyDataPool.begin(), mThreadContext, mContext, mOutputs);

		task->setContinuation(mCont);
		task->removeReference();
	}
}

}
}


