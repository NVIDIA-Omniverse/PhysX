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

#include "PxgBodySimManager.h"
#include "PxvDynamics.h"
#include "PxsRigidBody.h"
#include "PxgBodySim.h"
#include "foundation/PxFPU.h"
#include "foundation/PxAtomic.h"
#include "DyFeatherstoneArticulation.h"
#include "DyDeformableSurface.h"
#include "DyDeformableVolume.h"
#include "DyParticleSystem.h"

#define BODY_SIM_VALIDATE	0

using namespace physx;

void PxgBodySimManager::addBody(PxsRigidBody* rigidBody, const PxU32 nodeIndex)
{
	if (mUpdatedMap.boundedTest(nodeIndex))
		return;

	if (mBodies.capacity() <= nodeIndex)
	{
		mBodies.resize(2 * nodeIndex + 1);
	}

	mBodies[nodeIndex] = reinterpret_cast<void*>(rigidBody);

	mUpdatedMap.growAndSet(nodeIndex);
	mNewOrUpdatedBodySims.pushBack(nodeIndex);
	mTotalNumBodies = PxMax(mTotalNumBodies, nodeIndex + 1);

	mStaticConstraints.reserve(mTotalNumBodies);
	mStaticConstraints.resize(mTotalNumBodies);

	PxgStaticConstraints& constraints = mStaticConstraints[nodeIndex];
	constraints.mStaticContacts.forceSize_Unsafe(0);
	constraints.mStaticJoints.forceSize_Unsafe(0);

	// raise first DMA to GPU flag so we can handle body updates correctly with direct GPU API
	rigidBody->mInternalFlags |= PxsRigidBody::eFIRST_BODY_COPY_GPU;

#if BODY_SIM_VALIDATE
	else
	{
		bool found = false;
		for (PxU32 i = 0; i < mNewBodySims.size(); ++i)
		{
			if (mNewBodySims[i] == nodeIndex)
			{
				found = true;
				break;
			}
		}

		PX_ASSERT(found);
	}
#endif
}

PxgBodySimManager::~PxgBodySimManager()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////articulation
void PxgBodySimManager::addArticulation(Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex, bool OmniPVDRecordDirectGPUAPI)
{
	if (mUpdatedMap.boundedTest(nodeIndex))
		return;

	if (mBodies.capacity() <= nodeIndex)
	{
		mBodies.resize(2 * nodeIndex + 1);
	}

	mBodies[nodeIndex] = reinterpret_cast<void*>(articulation);

	mUpdatedMap.growAndSet(nodeIndex);
	PxgArticulationIndices index;
	index.nodeIndex = nodeIndex;
	index.remapIndex = mArticulationIdPool.getNewID();

	Dy::ArticulationCore* core = articulation->getCore();
	core->gpuRemapIndex = index.remapIndex;

	mNewArticulationSims.pushBack(index);
	//Mark as in dirty list so that it doesn't appear in update list!
	articulation->mGPUDirtyFlags |= Dy::ArticulationDirtyFlag::eIN_DIRTY_LIST;

	mNodeToRemapMap.insert(nodeIndex, index.remapIndex);
#if PX_SUPPORT_OMNI_PVD
	if (OmniPVDRecordDirectGPUAPI)
	{
		mRemapToNodeMap.insert(index.remapIndex, nodeIndex);
	}
#else
	PX_UNUSED(OmniPVDRecordDirectGPUAPI);
#endif
	//articulation->setGpuRemapId(index.remapIndex);

	mTotalNumBodies = PxMax(mTotalNumBodies, nodeIndex + 1);
	mTotalNumArticulations = PxMax(mTotalNumArticulations, index.remapIndex + 1);

	mStaticConstraints.resize(mTotalNumBodies); //Shared between RBs and articulations...
	mArticulationSelfConstraints.resize(mTotalNumArticulations);

	PxgStaticConstraints& constraints = mStaticConstraints[nodeIndex];
	constraints.mStaticContacts.forceSize_Unsafe(0);
	constraints.mStaticContacts.reserve(articulation->getBodyCount()); //We expect a contact per-body, so bump up the initial reservation
	constraints.mStaticJoints.forceSize_Unsafe(0);

	PxgArticulationSelfConstraints& selfConstraints = mArticulationSelfConstraints[index.remapIndex];
	selfConstraints.mSelfContacts.forceSize_Unsafe(0);
	selfConstraints.mSelfJoints.forceSize_Unsafe(0);
}

void PxgBodySimManager::updateArticulation(Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex)
{
	if (!(articulation->mGPUDirtyFlags & Dy::ArticulationDirtyFlag::eIN_DIRTY_LIST))
	{
		const PxHashMap<PxU32, PxU32>::Entry* entry = mNodeToRemapMap.find(nodeIndex);
		//If entry is not there, it means that this articulation is pending insertion (added to scene, but not yet simulated). In this
		//case, it will not need to appear in the update list
		if (entry)
		{
			articulation->mGPUDirtyFlags |= Dy::ArticulationDirtyFlag::eIN_DIRTY_LIST;
			PxU32 index = entry->second;

			PxgArticulationUpdate update;
			update.articulationIndex = index;
			update.articulation = articulation;
			//KS - we don't write out startIndex - this happens later in the heavy-lifting code that actually writes out data from the articulation.
			//We defer this until during the frame because it is quite common that the user might update multiple properties and we would prefer to batc

			mUpdatedArticulations.pushBack(update);
		}
	}
}

bool PxgBodySimManager::addStaticArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(nodeIndex.isArticulation());
	PxgStaticConstraints& staticConstraints = mStaticConstraints[nodeIndex.index()];

	if (staticConstraints.mStaticContacts.size() < PxgStaticConstraints::MaxConstraints)
	{
		PxgStaticConstraint con;
		con.uniqueId = uniqueIndex;
		con.linkID = nodeIndex.articulationLinkId();
		PxU32 i = 0;
		PxU32 count = staticConstraints.mStaticContacts.size();
		for (; i < count; ++i)
		{
			if (con.linkID <= staticConstraints.mStaticContacts[i].linkID)
				break;
		}
		staticConstraints.mStaticContacts.resizeUninitialized(count + 1);

		for (PxU32 j = count; j > i; --j)
		{
			staticConstraints.mStaticContacts[j] = staticConstraints.mStaticContacts[j - 1];
		}

		staticConstraints.mStaticContacts[i] = con;
		mMaxStaticArticContacts = PxMax(staticConstraints.mStaticContacts.size(), mMaxStaticArticContacts);
		mTotalStaticArticContacts++;
		return true;
	}
	return false;
}

template<class T>
static bool remove(PxU32 uniqueIndex, PxArray<T>& container, PxU32& counter)
{
	// PT: one of these loops was checking the indices in decreasing order for no clear reason (introduced in CL 30572881)
	PxU32 size = container.size();
	for(PxU32 i=0; i<size; i++)
	{
		if(container[i].uniqueId == uniqueIndex)
		{
			//Need to maintain order for batching to work!
			container.remove(i);	// PT: beware, this is an actual shift
			counter--;
			return true;
		}
	}
	return false;
}

bool PxgBodySimManager::removeStaticArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(nodeIndex.isArticulation());
	return remove(uniqueIndex, mStaticConstraints[nodeIndex.index()].mStaticContacts, mTotalStaticArticContacts);
}

bool PxgBodySimManager::addStaticArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(nodeIndex.isArticulation());
	PxgStaticConstraints& staticConstraints = mStaticConstraints[nodeIndex.index()];

	if (staticConstraints.mStaticJoints.size() < PxgStaticConstraints::MaxConstraints)
	{
		PxgStaticConstraint con;
		con.uniqueId = uniqueIndex;
		con.linkID = nodeIndex.articulationLinkId();
		//We need to find where to insert this contact into the articulation. The assumption is that we will not have a very
		//large number of static constraints, so we can just do a linear search. If this turns out to be a problem, we can try
		//something like binary search...
		PxU32 i = 0;
		PxU32 count = staticConstraints.mStaticJoints.size();
		for (; i < count; ++i)
		{
			if (con.linkID <= staticConstraints.mStaticJoints[i].linkID)
				break;
		}
		staticConstraints.mStaticJoints.resizeUninitialized(count + 1);

		for (PxU32 j = count; j > i; --j)
		{
			staticConstraints.mStaticJoints[j] = staticConstraints.mStaticJoints[j - 1];
		}

		staticConstraints.mStaticJoints[i] = con;

		mMaxStaticArticJoints = PxMax(mMaxStaticArticJoints, count + 1);
		mTotalStaticArticJoints++;

		return true;
	}
	return false;
}

bool PxgBodySimManager::removeStaticArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(nodeIndex.isArticulation());
	return remove(uniqueIndex, mStaticConstraints[nodeIndex.index()].mStaticJoints, mTotalStaticArticJoints);
}

bool PxgBodySimManager::addStaticRBContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PxgStaticConstraints& staticConstraints = mStaticConstraints[nodeIndex.index()];
	{
		PX_ASSERT(!nodeIndex.isArticulation());
		PxgStaticConstraint con;
		con.uniqueId = uniqueIndex;
		con.linkID = 0;
		staticConstraints.mStaticContacts.pushBack(con);
		mMaxStaticRBContacts = PxMax(staticConstraints.mStaticContacts.size(), mMaxStaticRBContacts);
		mTotalStaticRBContacts++;
		return true;
	}
}

bool PxgBodySimManager::removeStaticRBContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(!nodeIndex.isArticulation());
	return remove(uniqueIndex, mStaticConstraints[nodeIndex.index()].mStaticContacts, mTotalStaticRBContacts);
}

bool PxgBodySimManager::addStaticRBJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(!nodeIndex.isArticulation());
	PxgStaticConstraints& staticConstraints = mStaticConstraints[nodeIndex.index()];

	{
		PxgStaticConstraint con;
		con.uniqueId = uniqueIndex;
		con.linkID = 0;

		staticConstraints.mStaticJoints.pushBack(con);

		mMaxStaticRBJoints = PxMax(mMaxStaticRBJoints, staticConstraints.mStaticJoints.size());
		mTotalStaticRBJoints++;
		return true;
	}
}

bool PxgBodySimManager::removeStaticRBJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex)
{
	PX_ASSERT(!nodeIndex.isArticulation());
	return remove(uniqueIndex, mStaticConstraints[nodeIndex.index()].mStaticJoints, mTotalStaticRBJoints);
}

bool PxgBodySimManager::addSelfArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1)
{
	PxU32 remapIndex = mNodeToRemapMap[nodeIndex0.index()];
	PxgArticulationSelfConstraints& selfConstraints = mArticulationSelfConstraints[remapIndex];
	if (selfConstraints.mSelfContacts.size() < PxgArticulationSelfConstraints::MaxConstraints)
	{
		PxgSelfConstraint con;
		con.uniqueId = uniqueIndex;
		con.linkID0 = nodeIndex0.articulationLinkId();
		con.linkID1 = nodeIndex1.articulationLinkId();

		PxU32 count = selfConstraints.mSelfContacts.size();

		selfConstraints.mSelfContacts.resizeUninitialized(count + 1);

		PxU32 i = count;

		selfConstraints.mSelfContacts[i] = con;
		mMaxSelfArticContacts = PxMax(selfConstraints.mSelfContacts.size(), mMaxSelfArticContacts);
		mTotalSelfArticContacts++;
		return true;
	}
	return false;
}

bool PxgBodySimManager::removeSelfArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1)
{
	PX_UNUSED(nodeIndex1);
	const PxHashMap<PxU32, PxU32>::Entry* entry = mNodeToRemapMap.find(nodeIndex0.index());

	if(!entry)
		return false;

	const PxU32 index = entry->second;
	return remove(uniqueIndex, mArticulationSelfConstraints[index].mSelfContacts, mTotalSelfArticContacts);
}

bool PxgBodySimManager::addSelfArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1)
{
	PxU32 remapIndex = mNodeToRemapMap[nodeIndex0.index()];
	PxgArticulationSelfConstraints& selfConstraints = mArticulationSelfConstraints[remapIndex];
	if (selfConstraints.mSelfJoints.size() < PxgArticulationSelfConstraints::MaxConstraints)
	{
		PxgSelfConstraint con;
		con.uniqueId = uniqueIndex;
		con.linkID0 = nodeIndex0.articulationLinkId();
		con.linkID1 = nodeIndex1.articulationLinkId();
		//We need to find where to insert this contact into the articulation. The assumption is that we will not have a very
		//large number of static constraints, so we can just do a linear search. If this turns out to be a problem, we can try
		//something like binary search...
		PxU32 i = 0;
		PxU32 count = selfConstraints.mSelfJoints.size();
		for (; i < count; ++i)
		{
			if (con.linkID0 <= selfConstraints.mSelfJoints[i].linkID0)
				break;
		}
		selfConstraints.mSelfJoints.resizeUninitialized(count + 1);

		for (PxU32 j = count; j > i; --j)
		{
			selfConstraints.mSelfJoints[j] = selfConstraints.mSelfJoints[j - 1];
		}

		selfConstraints.mSelfJoints[i] = con;

		mMaxSelfArticJoints = PxMax(mMaxSelfArticJoints, count + 1);
		mTotalSelfArticJoints++;
		return true;
	}
	return false;
}

bool PxgBodySimManager::removeSelfArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1)
{
	PX_UNUSED(nodeIndex1);
	const PxHashMap<PxU32, PxU32>::Entry* entry = mNodeToRemapMap.find(nodeIndex0.index());

	if(!entry)
		return false;

	const PxU32 index = entry->second;
	return remove(uniqueIndex, mArticulationSelfConstraints[index].mSelfJoints, mTotalSelfArticJoints);
}

void PxgBodySimManager::releaseArticulation(Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex)
{
	mDeferredFreeNodeIDs.pushBack(nodeIndex);

	if (articulation->mGPUDirtyFlags & Dy::ArticulationDirtyFlag::eIN_DIRTY_LIST)
	{
		for (PxU32 i = 0; i < mUpdatedArticulations.size(); ++i)
		{
			if (mUpdatedArticulations[i].articulation == articulation)
			{
				mUpdatedArticulations.replaceWithLast(i);
				break;
			}
		}

		for (PxU32 i = 0; i < mNewArticulationSims.size(); ++i)
		{
			if (mNewArticulationSims[i].nodeIndex == nodeIndex)
			{
				mNewArticulationSims.replaceWithLast(i);
				break;
			}
		}
	}
}

void PxgBodySimManager::releaseDeferredArticulationIds()
{
	PxHashMap<PxU32, PxU32>::Entry entry;
	PxU32 size = mDeferredFreeNodeIDs.size();
	for(PxU32 i=0;i<size;i++)
	{
		bool found = mNodeToRemapMap.erase(mDeferredFreeNodeIDs[i], entry);
		PX_UNUSED(found);
		PX_ASSERT(found);
#if PX_SUPPORT_OMNI_PVD
		mRemapToNodeMap.erase(entry.second);
#endif	
		mArticulationIdPool.deferredFreeID(entry.second);
	}
	mDeferredFreeNodeIDs.clear();
	mArticulationIdPool.processDeferredIds();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////softbody
	
void PxgBodySimManager::addSoftBody(Dy::DeformableVolume* deformableVolume, const PxU32 nodeIndex)
{
	if (mUpdatedMap.boundedTest(nodeIndex))
		return;

	if (mBodies.capacity() <= nodeIndex)
	{
		mBodies.resize(2 * nodeIndex + 1);
	}

	mBodies[nodeIndex] = reinterpret_cast<void*>(deformableVolume);

	mUpdatedMap.growAndSet(nodeIndex);
	PxgSoftBodyIndices index;
	index.nodeIndex = nodeIndex;
	index.remapIndex = mSoftBodyIdPool.getNewID();
	mNewSoftBodySims.pushBack(index);

	deformableVolume->setGpuRemapId(index.remapIndex);

	if (mActiveSoftbodyIndex.size() < (index.remapIndex + 1))
	{
		mActiveSoftbodyIndex.resize(PxMax(index.remapIndex + 1, mActiveSoftbodyIndex.size() * 2));
		mActiveSelfCollisionSoftbodyIndex.resize(PxMax(index.remapIndex + 1, mActiveSelfCollisionSoftbodyIndex.size() * 2));
	}
	mActiveSoftbodyIndex[index.remapIndex] = mActiveSoftbodiesStaging.size();

	if (deformableVolume->getCore().bodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION)
		mActiveSelfCollisionSoftbodyIndex[index.remapIndex] = 0xFFFFFFFF;
	else
	{
		mActiveSelfCollisionSoftbodyIndex[index.remapIndex] = mActiveSelfCollisionSoftBodiesStaging.size();
		mActiveSelfCollisionSoftBodiesStaging.pushBack(index.remapIndex);
	}

	mTotalNumBodies = PxMax(mTotalNumBodies, nodeIndex + 1);
	mTotalNumSoftBodies = PxMax(mTotalNumSoftBodies, index.remapIndex + 1);

	mActiveSoftbodiesStaging.pushBack(index.remapIndex);
	mActiveSoftbodiesDirty = true;
	if (index.remapIndex == mDeformableVolumes.size())
		mDeformableVolumes.pushBack(deformableVolume);
	else
		mDeformableVolumes[index.remapIndex] = deformableVolume;
}

void PxgBodySimManager::releaseSoftBody(Dy::DeformableVolume* deformableVolume)
{
	PxU32 remapIndex = deformableVolume->getGpuRemapId();
	PxU32 index = mActiveSoftbodyIndex[remapIndex];

	if (index != 0xFFFFFFFF)
	{
		PX_ASSERT(mActiveSoftbodiesStaging[index] == remapIndex);
		mActiveSoftbodiesDirty = true;
		mActiveSoftbodyIndex[remapIndex] = 0xFFFFFFFF;


		mActiveSoftbodiesStaging.replaceWithLast(index);
		if (index < mActiveSoftbodiesStaging.size())
			mActiveSoftbodyIndex[mActiveSoftbodiesStaging[index]] = index;

		PxU32 selfCollisionIndex = mActiveSelfCollisionSoftbodyIndex[remapIndex];

		if (selfCollisionIndex != 0xFFFFFFFF)
		{
			PX_ASSERT(mActiveSelfCollisionSoftBodiesStaging[selfCollisionIndex] == remapIndex);
			mActiveSelfCollisionSoftbodyIndex[remapIndex] = 0xFFFFFFFF;


			mActiveSelfCollisionSoftBodiesStaging.replaceWithLast(selfCollisionIndex);
			if (selfCollisionIndex < mActiveSelfCollisionSoftBodiesStaging.size())
				mActiveSelfCollisionSoftbodyIndex[mActiveSelfCollisionSoftBodiesStaging[selfCollisionIndex]] = selfCollisionIndex;
		}
	}

	for (PxU32 i = 0; i < mNewSoftBodySims.size(); ++i)
	{
		if (mNewSoftBodySims[i].remapIndex == remapIndex)
		{
			mNewSoftBodySims.replaceWithLast(i);
		}
	}

	mDeformableVolumes[remapIndex] = NULL;

	mSoftBodyIdPool.deferredFreeID(deformableVolume->getGpuRemapId());
}

void PxgBodySimManager::releaseDeferredSoftBodyIds()
{
	mSoftBodyIdPool.processDeferredIds();
}

bool PxgBodySimManager::activateSoftbody(Dy::DeformableVolume* deformableVolume)
{
	PxU32 remapIndex = deformableVolume->getGpuRemapId();
	PxU32 index = mActiveSoftbodyIndex[remapIndex];
	if (0xFFFFFFFF == index)
	{
		mActiveSoftbodyIndex[remapIndex] = mActiveSoftbodiesStaging.size();
		mActiveSoftbodiesStaging.pushBack(remapIndex);
		mActiveSoftbodiesDirty = true;

		if (deformableVolume->getCore().bodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION)
			mActiveSelfCollisionSoftbodyIndex[remapIndex] = 0xFFFFFFFF;
		else
		{
			mActiveSelfCollisionSoftbodyIndex[remapIndex] = mActiveSelfCollisionSoftBodiesStaging.size();
			mActiveSelfCollisionSoftBodiesStaging.pushBack(remapIndex);
		}

		return true;
	}
	return false;
}

bool PxgBodySimManager::deactivateSoftbody(Dy::DeformableVolume* deformableVolume)
{
	PxU32 remapIndex = deformableVolume->getGpuRemapId();
	PxU32 index = mActiveSoftbodyIndex[remapIndex];
	if (0xFFFFFFFF != index)
	{
		mActiveSoftbodyIndex[remapIndex] = 0xFFFFFFFF;
		mActiveSoftbodiesStaging.replaceWithLast(index);
		mActiveSoftbodiesDirty = true;
		if (index < mActiveSoftbodiesStaging.size())
			mActiveSoftbodyIndex[mActiveSoftbodiesStaging[index]] = index;

		deactivateSoftbodySelfCollision(deformableVolume);

		return true;
	}
	return false;
}

bool PxgBodySimManager::activateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume)
{
	PxU32 remapIndex = deformableVolume->getGpuRemapId();
	PxU32 index = mActiveSelfCollisionSoftbodyIndex[remapIndex];
	if (0xFFFFFFFF == index)
	{
		mActiveSelfCollisionSoftbodyIndex[remapIndex] = mActiveSelfCollisionSoftBodiesStaging.size();
		mActiveSelfCollisionSoftBodiesStaging.pushBack(remapIndex);
		mActiveSoftbodiesDirty = true;
		return true;
	}
	return false;
}

bool PxgBodySimManager::deactivateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume)
{
	PxU32 remapIndex = deformableVolume->getGpuRemapId();
	PxU32 index = mActiveSelfCollisionSoftbodyIndex[remapIndex];
	if (0xFFFFFFFF != index)
	{
		mActiveSoftbodiesDirty = true;
		PX_ASSERT(mActiveSelfCollisionSoftBodiesStaging[index] == remapIndex);
		mActiveSelfCollisionSoftbodyIndex[remapIndex] = 0xFFFFFFFF;

		mActiveSelfCollisionSoftBodiesStaging.replaceWithLast(index);
		if (index < mActiveSelfCollisionSoftBodiesStaging.size())
			mActiveSelfCollisionSoftbodyIndex[mActiveSelfCollisionSoftBodiesStaging[index]] = index;

		return true;
	}
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////femCloth

void PxgBodySimManager::addFEMCloth(Dy::DeformableSurface* deformableSurface, const PxU32 nodeIndex)
{
	if (mUpdatedMap.boundedTest(nodeIndex))
		return;

	if (mBodies.capacity() <= nodeIndex)
	{
		mBodies.resize(2 * nodeIndex + 1);
	}

	mBodies[nodeIndex] = reinterpret_cast<void*>(deformableSurface);

	mUpdatedMap.growAndSet(nodeIndex);
		
	PxgFEMClothIndices index;
	index.nodeIndex = nodeIndex;
	index.remapIndex = mFEMClothIdPool.getNewID();
	mNewFEMClothSims.pushBack(index);

	deformableSurface->setGpuRemapId(index.remapIndex);

	if (mActiveFEMClothIndex.size() < (index.remapIndex+1))
	{
		mActiveFEMClothIndex.resize(PxMax(index.remapIndex + 1, mActiveFEMClothIndex.size() * 2));
	}
	mActiveFEMClothIndex[index.remapIndex] = mActiveFEMClothStaging.size();

	mTotalNumBodies = PxMax(mTotalNumBodies, nodeIndex + 1);
	mTotalNumFEMCloths = PxMax(mTotalNumFEMCloths, index.remapIndex + 1);

	mActiveFEMClothStaging.pushBack(index.remapIndex);
	mActiveFEMClothsDirty = true;

	if (index.remapIndex == mDeformableSurfaces.size())
		mDeformableSurfaces.pushBack(deformableSurface);
	else
		mDeformableSurfaces[index.remapIndex] = deformableSurface;
}

void PxgBodySimManager::releaseFEMCloth(Dy::DeformableSurface* deformableSurface)
{
	PxU32 remapIndex = deformableSurface->getGpuRemapId();
	PxU32 index = mActiveFEMClothIndex[remapIndex];

	if (index != 0xFFFFFFFF)
	{
		PX_ASSERT(mActiveFEMClothStaging[index] == remapIndex);
		mActiveFEMClothsDirty = true;
		mActiveFEMClothIndex[remapIndex] = 0xFFFFFFFF;
			
		mActiveFEMClothStaging.replaceWithLast(index);
		if (index < mActiveFEMClothStaging.size())
			mActiveFEMClothIndex[mActiveFEMClothStaging[index]] = index;
	}

	for (PxU32 i = 0; i < mNewFEMClothSims.size(); ++i)
	{
		if (mNewFEMClothSims[i].remapIndex == remapIndex)
		{
			mNewFEMClothSims.replaceWithLast(i);
		}
	}

	mDeformableSurfaces[remapIndex] = NULL;

	mFEMClothIdPool.deferredFreeID(remapIndex);
}

void PxgBodySimManager::releaseDeferredFEMClothIds()
{
	mFEMClothIdPool.processDeferredIds();
}

bool PxgBodySimManager::activateCloth(Dy::DeformableSurface* deformableSurface)
{
	PxU32 remapIndex = deformableSurface->getGpuRemapId();
	PxU32 index = mActiveFEMClothIndex[remapIndex];
	if (0xFFFFFFFF == index)
	{
		mActiveFEMClothIndex[remapIndex] = mActiveFEMClothStaging.size();
		mActiveFEMClothStaging.pushBack(remapIndex);
		mActiveFEMClothsDirty = true;
		return true;
	}
	return false;
}

bool PxgBodySimManager::deactivateCloth(Dy::DeformableSurface* deformableSurface)
{
	PxU32 remapIndex = deformableSurface->getGpuRemapId();
	PxU32 index = mActiveFEMClothIndex[remapIndex];
	if (0xFFFFFFFF != index)
	{
		mActiveFEMClothIndex[remapIndex] = 0xFFFFFFFF;
		mActiveFEMClothStaging.replaceWithLast(index);
		mActiveFEMClothsDirty = true;
		if (index < mActiveFEMClothStaging.size())
			mActiveFEMClothIndex[mActiveFEMClothStaging[index]] = index;
		return true;
	}
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////particlesystem

void PxgBodySimManager::addPBDParticleSystem(Dy::ParticleSystem* particleSystem, const PxU32 nodeIndex)
{
	if (mUpdatedMap.boundedTest(nodeIndex))
		return;

	if (mBodies.capacity() <= nodeIndex)
	{
		mBodies.resize(2 * nodeIndex + 1);
	}

	mBodies[nodeIndex] = reinterpret_cast<void*>(particleSystem);

	mUpdatedMap.growAndSet(nodeIndex);
	PxgParticleSystemIndices index;
	index.nodeIndex = nodeIndex;
	index.remapIndex = mPBDParticleSystemIdPool.getNewID();
	mNewPBDParticleSystemSims.pushBack(index);

	particleSystem->setGpuRemapId(index.remapIndex);

	mTotalNumBodies = PxMax(mTotalNumBodies, nodeIndex + 1);
	mTotalNumPBDParticleSystems = PxMax(mTotalNumPBDParticleSystems, index.remapIndex + 1);

	mActivePBDParticleSystems.pushBack(index.remapIndex);
	mActivePBDParticleSystemsDirty = true;
}

void PxgBodySimManager::releasePBDParticleSystem(Dy::ParticleSystem* particleSystem)
{
	PxU32 remapIndex = particleSystem->getGpuRemapId();
	for (PxU32 i = 0; i < mActivePBDParticleSystems.size(); ++i)
	{
		if (mActivePBDParticleSystems[i] == remapIndex)
		{
			mActivePBDParticleSystems.replaceWithLast(i);
			mActivePBDParticleSystemsDirty = true;
			break;
		}
	}

	for (PxU32 i = 0; i < mNewPBDParticleSystemSims.size(); ++i)
	{
		if (mNewPBDParticleSystemSims[i].remapIndex == remapIndex)
		{
			mNewPBDParticleSystemSims.replaceWithLast(i);
		}
	}

	mPBDParticleSystemIdPool.deferredFreeID(remapIndex);
}

void PxgBodySimManager::releaseDeferredPBDParticleSystemIds()
{
	mPBDParticleSystemIdPool.processDeferredIds();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////softbody
	
void PxgBodySimManager::updateBody(const PxNodeIndex& nodeIndex)
{
	const PxU32 index = nodeIndex.index();
	if (!mUpdatedMap.boundedTest(index))
	{
		mUpdatedMap.growAndSet(index);
		mNewOrUpdatedBodySims.pushBack(index);
	}
}
	
void PxgBodySimManager::updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, PxU32 nbBodies, 
	PxsExternalAccelerationProvider* externalAccelerations)
{
	mExternalAccelerations = externalAccelerations;
	PxU32 newVal = static_cast<PxU32>(PxAtomicAdd(reinterpret_cast<PxI32*>(&mNbUpdatedBodies), static_cast<PxI32>(nbBodies)));

	PxgBodySimVelocityUpdate* newUpdatedBodies = mNewUpdatedBodies.begin();
	const PxU32 startIndex = newVal - nbBodies;
	for(PxU32 i=0; i<nbBodies; ++i)
	{
		PxsRigidBody* rigid = rigidBodies[i];
		PxsBodyCore& bcLL = rigid->getCore();

		newUpdatedBodies[i + startIndex].linearVelocityXYZ_bodySimIndexW = make_float4(bcLL.linearVelocity.x, bcLL.linearVelocity.y, bcLL.linearVelocity.z, PX_FR(nodeIndices[i]));
		newUpdatedBodies[i + startIndex].angularVelocityXYZ_maxPenBiasW = make_float4(bcLL.angularVelocity.x, bcLL.angularVelocity.y, bcLL.angularVelocity.z, bcLL.maxPenBias);
		
		if (externalAccelerations) 
		{
			if (externalAccelerations->hasAccelerations())
			{
				const PxsRigidBodyExternalAcceleration& acc = externalAccelerations->get(nodeIndices[i]);
				newUpdatedBodies[i + startIndex].externalLinearAccelerationXYZ = make_float4(acc.linearAcceleration.x, acc.linearAcceleration.y, acc.linearAcceleration.z, 0.0f);
				newUpdatedBodies[i + startIndex].externalAngularAccelerationXYZ = make_float4(acc.angularAcceleration.x, acc.angularAcceleration.y, acc.angularAcceleration.z, 0.0f);
			}
			else
			{
				newUpdatedBodies[i + startIndex].externalLinearAccelerationXYZ = make_float4(0.0f);
				newUpdatedBodies[i + startIndex].externalAngularAccelerationXYZ = make_float4(0.0f);
			}
		}
	}
}

void PxgBodySimManager::reserve(const PxU32 nbBodies)
{
	const PxU32 requiredSize = mNewUpdatedBodies.size() + nbBodies;
	mNbUpdatedBodies = mNewUpdatedBodies.size();
	if (mNewUpdatedBodies.capacity() < requiredSize)
	{
		mNewUpdatedBodies.reserve(requiredSize * 2);
	}
}

void PxgBodySimManager::destroy()
{
	this->~PxgBodySimManager();
	PX_FREE_THIS;
}

void PxgBodySimManager::reset() 
{ 
	mNewOrUpdatedBodySims.forceSize_Unsafe(0); 
	mNewArticulationSims.forceSize_Unsafe(0);

	mNewSoftBodySims.forceSize_Unsafe(0);
	mNewFEMClothSims.forceSize_Unsafe(0);
	mNewPBDParticleSystemSims.forceSize_Unsafe(0);

	mNewUpdatedBodies.forceSize_Unsafe(0);
	mUpdatedArticulations.forceSize_Unsafe(0);
	mNbUpdatedBodies = 0;
}
