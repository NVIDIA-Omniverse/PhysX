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

#include "PxgJointManager.h"
#include "DyConstraint.h"
#include "PxgD6JointData.h"
#include "PxgConstraintPrep.h"
#include "common/PxProfileZone.h"
#include "PxsPartitionEdge.h"

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

PxgJointManager::PxgJointManager(const PxVirtualAllocator& allocator, bool isDirectGpuApiEnabled) :
	mGpuRigidJointData(allocator), mGpuArtiJointData(allocator),
	mGpuRigidJointPrePrep(allocator), mGpuArtiJointPrePrep(allocator),
	mCpuRigidConstraintData(allocator), mCpuRigidConstraintRows(allocator),
	mCpuArtiConstraintData(allocator), mCpuArtiConstraintRows(allocator),
	mDirtyGPURigidJointDataIndices(allocator), mDirtyGPUArtiJointDataIndices(allocator)
	, mGpuConstraintIdMapHost(allocator)
	, mMaxConstraintId(0)
	, mIsGpuConstraintIdMapDirty(false)
	, mIsDirectGpuApiEnabled(isDirectGpuApiEnabled)
{
}

PxgJointManager::~PxgJointManager()
{
}

void PxgJointManager::reserveMemory(PxU32 maxConstraintRows)
{
	const PxU32 nbCpuRigidConstraints = mCpuRigidConstraints.size();
	const PxU32 nbCpuArtiConstraints = mCpuArtiConstraints.size();

	mCpuRigidConstraintData.reserve(nbCpuRigidConstraints);
	mCpuRigidConstraintData.forceSize_Unsafe(nbCpuRigidConstraints);

	mCpuRigidConstraintRows.reserve(nbCpuRigidConstraints * maxConstraintRows);
	mCpuRigidConstraintRows.forceSize_Unsafe(nbCpuRigidConstraints * maxConstraintRows);

	mCpuArtiConstraintData.reserve(nbCpuArtiConstraints);
	mCpuArtiConstraintData.forceSize_Unsafe(nbCpuArtiConstraints);

	mCpuArtiConstraintRows.reserve(nbCpuArtiConstraints * maxConstraintRows);
	mCpuArtiConstraintRows.forceSize_Unsafe(nbCpuArtiConstraints * maxConstraintRows);

	mNbCpuRigidConstraintRows = 0;
	mNbCpuArtiConstraintRows = 0;
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

static PX_FORCE_INLINE void removeJointGpuCpu(PxU32 edgeIndex,
	const IG::GPUExternalData& islandSimGpuData,
	PxHashMap<PxU32, PxU32>& gpuConstraintIndexMap,
	PxPinnedArray<PxgConstraintPrePrep>& gpuConstraintPrePrepEntries,
	PxInt32ArrayPinned& gpuDirtyJointDataIndices,
	Cm::IDPool& gpuIdPool,
	PxHashMap<PxU32, PxU32>& cpuConstraintIndexMap,
	PxArray<const Dy::Constraint*>& cpuConstraintList,
	PxArray<PxU32>& cpuConstraintEdgeIndices,
	PxArray<PxU32>& cpuUniqueIndices,
	PxArray<PxU32>& jointIndices,
	PxgJointManager::ConstraintIdMap& gpuConstraintIdMapHost,
	PxHashMap<PxU32, PxU32>& edgeIndexToGpuConstraintIdMap,
	bool& isGpuConstraintIdMapDirty,
	bool isDirectGpuApiEnabled)
{
	const PxPair<const PxU32, PxU32>* gpuPair = gpuConstraintIndexMap.find(edgeIndex);
	if (gpuPair)
	{
		//gpu
		const PxU32 jointDataIndex = gpuPair->second;

		//update the dirty list
		gpuDirtyJointDataIndices.pushBack(jointDataIndex);

		resetConstraintPrepPrep(gpuConstraintPrePrepEntries[jointDataIndex]);

		gpuIdPool.freeID(jointDataIndex);

		gpuConstraintIndexMap.erase(edgeIndex);

		if (isDirectGpuApiEnabled)
		{
			PxPair<const PxU32, PxU32> entry;
			const bool found = edgeIndexToGpuConstraintIdMap.erase(edgeIndex, entry);
			PX_ASSERT(found);

			if (found)  // should always be the case but extra safety if something went horribly wrong
			{
				const PxU32 mapIndex = entry.second;

				PX_ASSERT(mapIndex < gpuConstraintIdMapHost.size());
		
				gpuConstraintIdMapHost[mapIndex].invalidate();

				isGpuConstraintIdMapDirty = true;
			}
		}
	}
	else
	{
		//cpu
		const PxPair<const PxU32, PxU32>* cpuPair = cpuConstraintIndexMap.find(edgeIndex);
		if (cpuPair)
		{
			const PxU32 index = cpuPair->second;
			cpuConstraintList.replaceWithLast(index);

			PxU32 replaceEdgeIndex = cpuConstraintEdgeIndices.back();

			cpuConstraintEdgeIndices.replaceWithLast(index);

			cpuConstraintIndexMap[replaceEdgeIndex] = index;

			cpuUniqueIndices.replaceWithLast(index);

			const PartitionEdge* pEdge = islandSimGpuData.getFirstPartitionEdge(replaceEdgeIndex);
			if (pEdge)
				jointIndices[pEdge->mUniqueIndex] = index;

			cpuConstraintIndexMap.erase(edgeIndex);
		}
	}
}

void PxgJointManager::removeJoint(PxU32 edgeIndex, PxArray<PxU32>& jointIndices, const IG::CPUExternalData& islandSimCpuData, const IG::GPUExternalData& islandSimGpuData)
{
	const PxNodeIndex nodeIndex0 = islandSimCpuData.getNodeIndex1(edgeIndex);
	const PxNodeIndex nodeIndex1 = islandSimCpuData.getNodeIndex2(edgeIndex);

	if (nodeIndex0.isArticulation() || nodeIndex1.isArticulation())
	{
		removeJointGpuCpu(edgeIndex, islandSimGpuData,
			mGpuArtiConstraintIndices, mGpuArtiJointPrePrep, mDirtyGPUArtiJointDataIndices, mGpuArtiJointDataIDPool,
			mCpuArtiConstraintIndices, mCpuArtiConstraints, mCpuArtiConstraintEdgeIndices, mCpuArtiUniqueIndex,
			jointIndices,
			mGpuConstraintIdMapHost, mEdgeIndexToGpuConstraintIdMap,
			mIsGpuConstraintIdMapDirty, mIsDirectGpuApiEnabled);
	}
	else
	{
		removeJointGpuCpu(edgeIndex, islandSimGpuData,
			mGpuRigidConstraintIndices, mGpuRigidJointPrePrep, mDirtyGPURigidJointDataIndices, mGpuRigidJointDataIDPool,
			mCpuRigidConstraintIndices, mCpuRigidConstraints, mCpuRigidConstraintEdgeIndices, mCpuRigidUniqueIndex,
			jointIndices,
			mGpuConstraintIdMapHost, mEdgeIndexToGpuConstraintIdMap,
			mIsGpuConstraintIdMapDirty, mIsDirectGpuApiEnabled);
	}
}

static PX_FORCE_INLINE void addJointGpu(const Dy::Constraint& constraint, PxU32 edgeIndex, PxU32 uniqueId,
	PxNodeIndex nodeIndex0, PxNodeIndex nodeIndex1,
	Cm::IDPool& idPool, 
	PxPinnedArray<PxgD6JointData>& jointDataEntries,
	PxPinnedArray<PxgConstraintPrePrep>& constraintPrePrepEntries,
	PxInt32ArrayPinned& dirtyJointDataIndices,
	PxHashMap<PxU32, PxU32>& constraintIndexMap,
	PxArray<PxU32>& jointIndices, 
	PxPinnedArray<PxgSolverConstraintManagerConstants>& managerIter,
	const IG::IslandSim& islandSim,
	PxgJointManager::ConstraintIdMap& gpuConstraintIdMapHost,
	PxHashMap<PxU32, PxU32>& edgeIndexToGpuConstraintIdMap,
	bool& isGpuConstraintIdMapDirty,
	bool isDirectGpuApiEnabled)
{
	//In GPU, we work with PxgD6JointData and fill in PxgConstraintData
	const PxU32 jointDataId = idPool.getNewID();

	if (jointDataId >= jointDataEntries.capacity())
	{
		const PxU32 capacity = jointDataEntries.capacity() * 2 + 1;
		jointDataEntries.resize(capacity);
		constraintPrePrepEntries.resize(capacity);
	}

	PxgD6JointData& jointData = jointDataEntries[jointDataId];

	PxMemCopy(&jointData, constraint.constantBlock, constraint.constantBlockSize);

	//mark dirty
	dirtyJointDataIndices.pushBack(jointDataId);

	constraintIndexMap.insert(edgeIndex, jointDataId);

	::setupConstraintPrepPrep(constraintPrePrepEntries[jointDataId], &constraint, islandSim, nodeIndex0, nodeIndex1);

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
}

static PX_FORCE_INLINE void addJointCpu(const Dy::Constraint& constraint, PxU32 edgeIndex, PxU32 uniqueId,
	PxArray<const Dy::Constraint*>& constraintList,
	PxHashMap<PxU32, PxU32>& constraintIndexMap,
	PxArray<PxU32>& constraintEdgeIndices,
	PxArray<PxU32>& uniqueIndices,
	PxArray<PxU32>& jointIndices, 
	PxPinnedArray<PxgSolverConstraintManagerConstants>& managerIter)
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
	PxPinnedArray<PxgSolverConstraintManagerConstants>& managerIter, PxU32 uniqueId)
{
	const PxNodeIndex nodeIndex0 = islandSim.mCpuData.getNodeIndex1(edgeIndex);
	const PxNodeIndex nodeIndex1 = islandSim.mCpuData.getNodeIndex2(edgeIndex);

	bool isArticulationJoint = nodeIndex0.isArticulation() || nodeIndex1.isArticulation();
	//d6 joint(articulation + rigid body) with GPU shader
	if ((constraint->flags & PxConstraintFlag::eGPU_COMPATIBLE) && GPU_JOINT_PREP)
	{
		//GPU shader
		if (isArticulationJoint)
		{
			addJointGpu(*constraint, edgeIndex, uniqueId, nodeIndex0, nodeIndex1,
				mGpuArtiJointDataIDPool, mGpuArtiJointData, mGpuArtiJointPrePrep,
				mDirtyGPUArtiJointDataIndices, mGpuArtiConstraintIndices,
				jointIndices, managerIter, islandSim,
				mGpuConstraintIdMapHost, mEdgeIndexToGpuConstraintIdMap,
				mIsGpuConstraintIdMapDirty, mIsDirectGpuApiEnabled);
		}
		else
		{
			addJointGpu(*constraint, edgeIndex, uniqueId, nodeIndex0, nodeIndex1,
				mGpuRigidJointDataIDPool, mGpuRigidJointData, mGpuRigidJointPrePrep,
				mDirtyGPURigidJointDataIndices, mGpuRigidConstraintIndices,
				jointIndices, managerIter, islandSim,
				mGpuConstraintIdMapHost, mEdgeIndexToGpuConstraintIdMap,
				mIsGpuConstraintIdMapDirty, mIsDirectGpuApiEnabled);
		}	
	}
	else
	{
		//CPU
		if (isArticulationJoint)
		{
			addJointCpu(*constraint, edgeIndex, uniqueId, mCpuArtiConstraints, mCpuArtiConstraintIndices,
				mCpuArtiConstraintEdgeIndices, mCpuArtiUniqueIndex, jointIndices, managerIter);
		}
		else
		{
			addJointCpu(*constraint, edgeIndex, uniqueId, mCpuRigidConstraints, mCpuRigidConstraintIndices,
				mCpuRigidConstraintEdgeIndices, mCpuRigidUniqueIndex, jointIndices, managerIter);
		}
	}
}

static PX_FORCE_INLINE bool updateJointGpu(const Dy::Constraint& constraint, PxU32 edgeIndex,
	const PxHashMap<PxU32, PxU32>& constraintIndexMap,
	PxPinnedArray<PxgD6JointData>& jointDataEntries,
	PxPinnedArray<PxgConstraintPrePrep>& constraintPrePrepEntries,
	PxInt32ArrayPinned& dirtyJointDataIndices)
{
	const PxPair<const PxU32, PxU32>* pair = constraintIndexMap.find(edgeIndex);

	//ML:: if pair is NULL, this means the pair this constraint connect to asleep. In this case, we will let the updateIncrementalIslands to
	//activate this pair of objects and create new gpu joint data in addJoint(). Otherwise, we need to update the jointData and push the jointDataId to the dirty
	//list
	if (pair)
	{
		const PxU32 jointDataId = pair->second;
			
		PxgD6JointData jointDataCopy = *reinterpret_cast<const PxgD6JointData*>(constraint.constantBlock);

		jointDataEntries[jointDataId] = jointDataCopy;
		dirtyJointDataIndices.pushBack(jointDataId);

		::setupConstraintPrepPrep(constraintPrePrepEntries[jointDataId], &constraint);

		return true;
	}

	return false;
}

void PxgJointManager::updateJoint(PxU32 edgeIndex, const Dy::Constraint* constraint)
{
	if (constraint->flags & PxConstraintFlag::eGPU_COMPATIBLE && GPU_JOINT_PREP)
	{
		if (updateJointGpu(*constraint, edgeIndex, mGpuRigidConstraintIndices,
			mGpuRigidJointData, mGpuRigidJointPrePrep, mDirtyGPURigidJointDataIndices))
		{
			// if a joint does not involve an articulation link, it should not be
			// tracked in mGpuArtiConstraintIndices. Keep in mind though that this
			// function might get called for joints that are asleep, in which case
			// it will not be tracked in either of the two maps.

			PX_ASSERT(mGpuArtiConstraintIndices.find(edgeIndex) == NULL);
		}
		else
		{
			updateJointGpu(*constraint, edgeIndex, mGpuArtiConstraintIndices,
				mGpuArtiJointData, mGpuArtiJointPrePrep, mDirtyGPUArtiJointDataIndices);
		}
	}
}

PxU32 PxgJointManager::getGpuNbRigidConstraints()
{
	return mGpuRigidJointDataIDPool.getMaxID();
}

PxU32 PxgJointManager::getGpuNbArtiConstraints()
{
	return mGpuArtiJointDataIDPool.getMaxID();
}

PxU32 PxgJointManager::getGpuNbActiveRigidConstraints()
{
	return mGpuRigidJointDataIDPool.getNumUsedID();
}

PxU32 PxgJointManager::getGpuNbActiveArtiConstraints()
{
	return mGpuArtiJointDataIDPool.getNumUsedID();
}

PxU32 PxgJointManager::getCpuNbRigidConstraints()
{
	return mCpuRigidConstraints.size();
}

PxU32 PxgJointManager::getCpuNbArtiConstraints()
{
	return mCpuArtiConstraints.size();
}

void PxgJointManager::update(PxArray<PxU32>& jointOutputIndex)
{
	PX_PROFILE_ZONE("PxgJointManager.update", 0);
		
	//update constraints transform
	const PxU32 nbGpuRigidConstraints = mGpuRigidJointDataIDPool.getMaxID() ;//mGpuConstraints.size();

	//reassign cpu rigid index
	const PxU32 nbCpuRigidConstraints = mCpuRigidConstraints.size();
	for (PxU32 i = 0; i < nbCpuRigidConstraints; ++i)
	{
		jointOutputIndex[mCpuRigidUniqueIndex[i]] = nbGpuRigidConstraints + i;
	}
		
	const PxU32 nbGpuArtiConstraints = mGpuArtiJointDataIDPool.getMaxID();

	//reassign cpu rigid index
	const PxU32 nbCpuArtiConstraints = mCpuArtiConstraints.size();
	for (PxU32 i = 0; i < nbCpuArtiConstraints; ++i)
	{
		jointOutputIndex[mCpuArtiUniqueIndex[i]] = nbGpuArtiConstraints + i;
	}
}

void PxgJointManager::reset()
{
	mDirtyGPURigidJointDataIndices.resize(0);
	mDirtyGPUArtiJointDataIndices.resize(0);
}
