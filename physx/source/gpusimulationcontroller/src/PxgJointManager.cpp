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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxgJointManager.h"
#include "DyConstraint.h"
#include "PxgD6JointData.h"
#include "PxgConstraintPrep.h"
#include "common/PxProfileZone.h"
#include "PxsPartitionEdge.h"
#include "PxsHeapStats.h"
#include "cudamanager/PxCudaContext.h"

#define GPU_JOINT_PREP	1

using namespace physx;

static PX_FORCE_INLINE void resetConstraintPrepPrep(PxgConstraintPrePrep& preData)
{
	preData.mNodeIndexA = preData.mNodeIndexB = PxNodeIndex(PX_INVALID_NODE);
}

static PX_FORCE_INLINE void setupConstraintPrepPrep(PxgConstraintPrePrep& preData, const Dy::Constraint* constraint)
{
	preData.mFlags = constraint->flags;
	preData.mLinBreakForce = constraint->linBreakForce;
	preData.mAngBreakForce = constraint->angBreakForce;
}

static PX_FORCE_INLINE void setupConstraintPrepPrep(PxgConstraintPrePrep& preData, const Dy::Constraint* constraint,
													const IG::IslandSim& islandSim, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1)
{
	preData.mNodeIndexA = nodeIndex0;
	preData.mNodeIndexB = nodeIndex1;

	PX_UNUSED(islandSim);
	PX_ASSERT(nodeIndex0.index() < islandSim.getNbNodes() || nodeIndex0.index() == PX_INVALID_NODE);
	PX_ASSERT(nodeIndex1.index() < islandSim.getNbNodes() || nodeIndex1.index() == PX_INVALID_NODE);

	::setupConstraintPrepPrep(preData, constraint);
}

PxgJointManager::GpuJoints::GpuJoints(Cm::VirtualAllocatorCallback& hostMappedAlloc)
:
	mJointDataMapped(hostMappedAlloc, PxsHeapStats::eSOLVER, Cm::PinnableAllocatorFallback::eDISABLED),
	mJointPrePrepMapped(hostMappedAlloc, PxsHeapStats::eSOLVER, Cm::PinnableAllocatorFallback::eDISABLED),
	mDirtyIndicesMapped(hostMappedAlloc, PxsHeapStats::eSOLVER, Cm::PinnableAllocatorFallback::eDISABLED)
{
}

PxgJointManager::CpuJoints::CpuJoints(Cm::VirtualAllocatorCallback& hostAlloc)
:
	mConstraintData(hostAlloc, PxsHeapStats::eSOLVER),
	mConstraintRows(hostAlloc, PxsHeapStats::eSOLVER),
	mNbConstraintRows(0)
{
}

PxgJointManager::PxgJointManager(Cm::VirtualAllocatorCallback& hostAlloc, Cm::VirtualAllocatorCallback& hostMappedAlloc,
								 bool isDirectGpuApiEnabled, PxCudaContext* cudaContext)
:
	mGpuRigidJoints(hostMappedAlloc),
	mGpuArtiJoints(hostMappedAlloc),
	mCpuRigidJoints(hostAlloc),
	mCpuArtiJoints(hostAlloc),
	mGpuConstraintIdMapHost(hostAlloc, PxsHeapStats::eSOLVER),
	mMaxConstraintId(0),
	mIsGpuConstraintIdMapDirty(false),
	mIsDirectGpuApiEnabled(isDirectGpuApiEnabled),
	mCudaContext(cudaContext)
{
}

PxgJointManager::~PxgJointManager()
{
}

void PxgJointManager::reserveMemory(PxU32 maxConstraintRows)
{
	const PxU32 nbCpuRigidConstraints = mCpuRigidJoints.mConstraints.size();
	const PxU32 nbCpuArtiConstraints = mCpuArtiJoints.mConstraints.size();

	mCpuRigidJoints.mConstraintData.reserve(nbCpuRigidConstraints);
	mCpuRigidJoints.mConstraintData.forceSize_Unsafe(nbCpuRigidConstraints);

	mCpuRigidJoints.mConstraintRows.reserve(nbCpuRigidConstraints * maxConstraintRows);
	mCpuRigidJoints.mConstraintRows.forceSize_Unsafe(nbCpuRigidConstraints * maxConstraintRows);

	mCpuArtiJoints.mConstraintData.reserve(nbCpuArtiConstraints);
	mCpuArtiJoints.mConstraintData.forceSize_Unsafe(nbCpuArtiConstraints);

	mCpuArtiJoints.mConstraintRows.reserve(nbCpuArtiConstraints * maxConstraintRows);
	mCpuArtiJoints.mConstraintRows.forceSize_Unsafe(nbCpuArtiConstraints * maxConstraintRows);

	mCpuRigidJoints.mNbConstraintRows = 0;
	mCpuArtiJoints.mNbConstraintRows = 0;
}

void PxgJointManager::reserveMemoryPreAddRemove()
{
	if (mMaxConstraintId >= mGpuConstraintIdMapHost.size())
	{
		const PxU32 newSize = mMaxConstraintId * 2 + 1;
		mGpuConstraintIdMapHost.resize(newSize);

		mIsGpuConstraintIdMapDirty = true;
	}
}

void PxgJointManager::registerJoint(const Dy::Constraint& constraint)
{
	if (mIsDirectGpuApiEnabled && (constraint.flags & PxConstraintFlag::eGPU_COMPATIBLE) && GPU_JOINT_PREP)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(mMaxConstraintId) >= sizeof(constraint.index));

		mMaxConstraintId = PxMax(constraint.index, mMaxConstraintId);
	}
}

static PX_FORCE_INLINE void removeJointCpu(PxU32 edgeIndex, PxU32 index,
	const IG::GPUExternalData& islandSimGpuData,
	PxHashMap<PxU32, PxU32>& cpuConstraintIndexMap,
	PxArray<const Dy::Constraint*>& cpuConstraintList,
	PxArray<PxU32>& cpuConstraintEdgeIndices,
	PxArray<PxU32>& cpuUniqueIndices,
	PxArray<PxU32>& jointIndices)
{
	cpuConstraintList.replaceWithLast(index);

	PxU32 replaceEdgeIndex = cpuConstraintEdgeIndices.back();

	cpuConstraintEdgeIndices.replaceWithLast(index);

	cpuConstraintIndexMap[replaceEdgeIndex] = index;

	cpuUniqueIndices.replaceWithLast(index);

	const PartitionEdge* pEdge = islandSimGpuData.getFirstPartitionEdge(replaceEdgeIndex);
	if(pEdge)
		jointIndices[pEdge->mUniqueIndex] = index;

	cpuConstraintIndexMap.erase(edgeIndex);
}

static PX_FORCE_INLINE bool removeJointGpu(PxU32 edgeIndex, PxU32 jointDataIndex,
	PxHashMap<PxU32, PxU32>& gpuConstraintIndexMap,
	Cm::PinnableArray<PxgConstraintPrePrep>& gpuConstraintPrePrepEntriesMapped,
	Cm::PinnableArray<PxU32>& gpuDirtyJointDataIndicesMapped,
	Cm::IDPool& gpuIdPool,
	PxgJointManager::ConstraintIdMap& gpuConstraintIdMapHost,
	PxHashMap<PxU32, PxU32>& edgeIndexToGpuConstraintIdMap,
	bool& isGpuConstraintIdMapDirty,
	bool isDirectGpuApiEnabled)
{
	// update the dirty list
	if(!gpuDirtyJointDataIndicesMapped.pushBack(jointDataIndex))
	{
		return false;
	}

	resetConstraintPrepPrep(gpuConstraintPrePrepEntriesMapped[jointDataIndex]);

	gpuIdPool.freeID(jointDataIndex);

	gpuConstraintIndexMap.erase(edgeIndex);

	if(isDirectGpuApiEnabled)
	{
		PxPair<const PxU32, PxU32> entry;
		const bool found = edgeIndexToGpuConstraintIdMap.erase(edgeIndex, entry);
		PX_ASSERT(found);

		if(found) // should always be the case but extra safety if something went horribly wrong
		{
			const PxU32 mapIndex = entry.second;
			PX_ASSERT(mapIndex < gpuConstraintIdMapHost.size());
			gpuConstraintIdMapHost[mapIndex].invalidate();
			isGpuConstraintIdMapDirty = true;
		}
	}

	return true;
}

void PxgJointManager::removeJoint(PxU32 edgeIndex, PxArray<PxU32>& jointIndices, const IG::CPUExternalData& islandSimCpuData,
								  const IG::GPUExternalData& islandSimGpuData)
{
	if(mCudaContext->isInAbortMode())
	{
		return;
	}

	const PxNodeIndex nodeIndex0 = islandSimCpuData.getNodeIndex1(edgeIndex);
	const PxNodeIndex nodeIndex1 = islandSimCpuData.getNodeIndex2(edgeIndex);
	bool isArticulationJoint = nodeIndex0.isArticulation() || nodeIndex1.isArticulation();
	GpuJoints& gpuJoints = isArticulationJoint ? mGpuArtiJoints : mGpuRigidJoints;

	const PxPair<const PxU32, PxU32>* gpuPair = gpuJoints.mConstraintIndices.find(edgeIndex);
	if(gpuPair)
	{
		if(!removeJointGpu(edgeIndex, gpuPair->second, gpuJoints.mConstraintIndices, gpuJoints.mJointPrePrepMapped, gpuJoints.mDirtyIndicesMapped,
						   gpuJoints.mIDPool, mGpuConstraintIdMapHost, mEdgeIndexToGpuConstraintIdMap, mIsGpuConstraintIdMapDirty,
						   mIsDirectGpuApiEnabled))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgJointManager::removeJoint: failed to allocate pinned host buffer");
			mCudaContext->setAbortMode(true);
		}
	}
	else
	{
		CpuJoints& cpuJoints = isArticulationJoint ? mCpuArtiJoints : mCpuRigidJoints;
		const PxPair<const PxU32, PxU32>* cpuPair = cpuJoints.mConstraintIndices.find(edgeIndex);
		if(cpuPair)
		{
			removeJointCpu(edgeIndex, cpuPair->second, islandSimGpuData, cpuJoints.mConstraintIndices, cpuJoints.mConstraints,
						   cpuJoints.mConstraintEdgeIndices, cpuJoints.mUniqueIndex, jointIndices);
		}
	}
}

static PX_FORCE_INLINE bool addJointGpu(const Dy::Constraint& constraint, PxU32 edgeIndex, PxU32 uniqueId,
	PxNodeIndex nodeIndex0, PxNodeIndex nodeIndex1,
	Cm::IDPool& idPool, 
	Cm::PinnableArray<PxgD6JointData>& jointDataEntriesMapped,
	Cm::PinnableArray<PxgConstraintPrePrep>& constraintPrePrepEntriesMapped,
	Cm::PinnableArray<PxU32>& dirtyJointDataIndicesMapped,
	PxHashMap<PxU32, PxU32>& constraintIndexMap,
	PxArray<PxU32>& jointIndices, 
	Cm::PinnableArray<PxgSolverConstraintManagerConstants>& managerIter,
	const IG::IslandSim& islandSim,
	PxgJointManager::ConstraintIdMap& gpuConstraintIdMapHost,
	PxHashMap<PxU32, PxU32>& edgeIndexToGpuConstraintIdMap,
	bool& isGpuConstraintIdMapDirty,
	bool isDirectGpuApiEnabled)
{
	//In GPU, we work with PxgD6JointData and fill in PxgConstraintData
	const PxU32 jointDataId = idPool.getNewID();

	if (jointDataId >= jointDataEntriesMapped.capacity())
	{
		const PxU32 capacity = jointDataEntriesMapped.capacity() * 2 + 1;
		if (!jointDataEntriesMapped.resize(capacity))
		{
			return false;
		}
		if (!constraintPrePrepEntriesMapped.resize(capacity))
		{
			return false;
		}
	}

	PxgD6JointData& jointData = jointDataEntriesMapped[jointDataId];

	PxMemCopy(&jointData, constraint.constantBlock, constraint.constantBlockSize);

	//mark dirty
	if(!dirtyJointDataIndicesMapped.pushBack(jointDataId))
	{
		return false;
	}

	constraintIndexMap.insert(edgeIndex, jointDataId);

	::setupConstraintPrepPrep(constraintPrePrepEntriesMapped[jointDataId], &constraint, islandSim, nodeIndex0, nodeIndex1);

	jointIndices[uniqueId] = jointDataId;

	managerIter[uniqueId].mConstraintWriteBackIndex = constraint.index; // this is the joint writeback index

	if (isDirectGpuApiEnabled)
	{
		PX_ASSERT(constraint.index < gpuConstraintIdMapHost.size());
		PX_ASSERT(!edgeIndexToGpuConstraintIdMap.find(edgeIndex));

		edgeIndexToGpuConstraintIdMap.insert(edgeIndex, constraint.index);

		gpuConstraintIdMapHost[constraint.index].setJointDataId(jointDataId);

		isGpuConstraintIdMapDirty = true;
	}
	return true;
}

static PX_FORCE_INLINE void addJointCpu(const Dy::Constraint& constraint, PxU32 edgeIndex, PxU32 uniqueId,
	PxArray<const Dy::Constraint*>& constraintList,
	PxHashMap<PxU32, PxU32>& constraintIndexMap,
	PxArray<PxU32>& constraintEdgeIndices,
	PxArray<PxU32>& uniqueIndices,
	PxArray<PxU32>& jointIndices, 
	Cm::PinnableArray<PxgSolverConstraintManagerConstants>& managerIter)
{
	const PxU32 index = constraintList.size();
	//In CPU, we work with Dy::Constraint and fill in PxgConstraintData
	constraintIndexMap.insert(edgeIndex, index);

	constraintList.pushBack(&constraint);
	constraintEdgeIndices.pushBack(edgeIndex);
	uniqueIndices.pushBack(uniqueId);
	
	jointIndices[uniqueId] = index;
	managerIter[uniqueId].mConstraintWriteBackIndex = constraint.index; // this is the joint writeback index
}

void PxgJointManager::addJoint(PxU32 edgeIndex, const Dy::Constraint* constraint, IG::IslandSim& islandSim, PxArray<PxU32>& jointIndices, 
	Cm::PinnableArray<PxgSolverConstraintManagerConstants>& managerIter, PxU32 uniqueId)
{
	if(mCudaContext->isInAbortMode())
	{
		return;
	}

	const PxNodeIndex nodeIndex0 = islandSim.mCpuData.getNodeIndex1(edgeIndex);
	const PxNodeIndex nodeIndex1 = islandSim.mCpuData.getNodeIndex2(edgeIndex);

	bool isArticulationJoint = nodeIndex0.isArticulation() || nodeIndex1.isArticulation();
	//d6 joint(articulation + rigid body) with GPU shader
	if ((constraint->flags & PxConstraintFlag::eGPU_COMPATIBLE) && GPU_JOINT_PREP)
	{
		//GPU shader
		GpuJoints& gpuJoints = isArticulationJoint ? mGpuArtiJoints : mGpuRigidJoints;

		bool added = addJointGpu(*constraint, edgeIndex, uniqueId, nodeIndex0, nodeIndex1, 
						gpuJoints.mIDPool, gpuJoints.mJointDataMapped, gpuJoints.mJointPrePrepMapped,
						gpuJoints.mDirtyIndicesMapped, gpuJoints.mConstraintIndices,
						jointIndices, managerIter, islandSim, mGpuConstraintIdMapHost, 
						mEdgeIndexToGpuConstraintIdMap, mIsGpuConstraintIdMapDirty, mIsDirectGpuApiEnabled);

		if(!added)
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgJointManager::addJoint: failed to allocate pinned host buffer");
			mCudaContext->setAbortMode(true);
		}
	}
	else
	{
		//CPU
		CpuJoints& cpuJoints = isArticulationJoint ? mCpuArtiJoints : mCpuRigidJoints;
		addJointCpu(*constraint, edgeIndex, uniqueId, cpuJoints.mConstraints, cpuJoints.mConstraintIndices,
					cpuJoints.mConstraintEdgeIndices, cpuJoints.mUniqueIndex, jointIndices, managerIter);
	}
}

static PX_FORCE_INLINE bool updateJointGpu(const Dy::Constraint& constraint, const PxU32 jointDataId,
	Cm::PinnableArray<PxgD6JointData>& jointDataEntriesMapped,
	Cm::PinnableArray<PxgConstraintPrePrep>& constraintPrePrepEntriesMapped,
	Cm::PinnableArray<PxU32>& dirtyJointDataIndicesMapped)
{
	PxgD6JointData jointDataCopy = *reinterpret_cast<const PxgD6JointData*>(constraint.constantBlock);

	jointDataEntriesMapped[jointDataId] = jointDataCopy;
	if (!dirtyJointDataIndicesMapped.pushBack(jointDataId))
	{
		return false;
	}

	::setupConstraintPrepPrep(constraintPrePrepEntriesMapped[jointDataId], &constraint);
	return true;
}

void PxgJointManager::updateJoint(PxU32 edgeIndex, const Dy::Constraint* constraint)
{
	if(mCudaContext->isInAbortMode())
	{
		return;
	}

	if (constraint->flags & PxConstraintFlag::eGPU_COMPATIBLE && GPU_JOINT_PREP)
	{
		// ML:: if pair is NULL, this means the pair this constraint connect to asleep. In this case, we will let the
		// updateIncrementalIslands to
		// activate this pair of objects and create new gpu joint data in addJoint(). Otherwise, we need to update the jointData and
		// push the jointDataId to the dirty list
		GpuJoints* gpuJoints = NULL;
		PxU32 jointDataId = PX_INVALID_U32;

		const PxPair<const PxU32, PxU32>* rigidPair = mGpuRigidJoints.mConstraintIndices.find(edgeIndex);
		if(rigidPair)
		{
			jointDataId = rigidPair->second;
			gpuJoints = &mGpuRigidJoints;
			// if a joint does not involve an articulation link, it should not be
			// tracked in mGpuArtiConstraintIndices. Keep in mind though that this
			// function might get called for joints that are asleep, in which case
			// it will not be tracked in either of the two maps.
			PX_ASSERT(mGpuArtiJoints.mConstraintIndices.find(edgeIndex) == NULL);
		}
		else
		{
			const PxPair<const PxU32, PxU32>* artiPair = mGpuArtiJoints.mConstraintIndices.find(edgeIndex);
			if(artiPair)
			{
				jointDataId = artiPair->second;
				gpuJoints = &mGpuArtiJoints;
			}
		}

		if(jointDataId != PX_INVALID_U32 && !updateJointGpu(*constraint, jointDataId,
															gpuJoints->mJointDataMapped, gpuJoints->mJointPrePrepMapped, 
															gpuJoints->mDirtyIndicesMapped))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
									"PxgJointManager: failed to allocate pinned host buffer for dirty indices");
			mCudaContext->setAbortMode(true);
		}
	}
}

PxU32 PxgJointManager::getGpuNbRigidConstraints()
{
	return mGpuRigidJoints.mIDPool.getMaxID();
}

PxU32 PxgJointManager::getGpuNbArtiConstraints()
{
	return mGpuArtiJoints.mIDPool.getMaxID();
}

PxU32 PxgJointManager::getGpuNbActiveRigidConstraints()
{
	return mGpuRigidJoints.mIDPool.getNumUsedID();
}

PxU32 PxgJointManager::getGpuNbActiveArtiConstraints()
{
	return mGpuArtiJoints.mIDPool.getNumUsedID();
}

PxU32 PxgJointManager::getCpuNbRigidConstraints()
{
	return mCpuRigidJoints.mConstraints.size();
}

PxU32 PxgJointManager::getCpuNbArtiConstraints()
{
	return mCpuArtiJoints.mConstraints.size();
}

void PxgJointManager::update(PxArray<PxU32>& jointOutputIndex)
{
	PX_PROFILE_ZONE("PxgJointManager.update", 0);
		
	//update constraints transform
	const PxU32 nbGpuRigidConstraints = mGpuRigidJoints.mIDPool.getMaxID() ;//mGpuConstraints.size();

	//reassign cpu rigid index
	const PxU32 nbCpuRigidConstraints = mCpuRigidJoints.mConstraints.size();
	for (PxU32 i = 0; i < nbCpuRigidConstraints; ++i)
	{
		jointOutputIndex[mCpuRigidJoints.mUniqueIndex[i]] = nbGpuRigidConstraints + i;
	}
		
	const PxU32 nbGpuArtiConstraints = mGpuArtiJoints.mIDPool.getMaxID();

	//reassign cpu rigid index
	const PxU32 nbCpuArtiConstraints = mCpuArtiJoints.mConstraints.size();
	for (PxU32 i = 0; i < nbCpuArtiConstraints; ++i)
	{
		jointOutputIndex[mCpuArtiJoints.mUniqueIndex[i]] = nbGpuArtiConstraints + i;
	}
}

void PxgJointManager::reset()
{
	mGpuRigidJoints.mDirtyIndicesMapped.forceSize_Unsafe(0);
	mGpuArtiJoints.mDirtyIndicesMapped.forceSize_Unsafe(0);
}
