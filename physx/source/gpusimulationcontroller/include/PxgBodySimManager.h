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

#ifndef PXG_BODYSIM_MANAGER_H
#define	PXG_BODYSIM_MANAGER_H

#include "foundation/PxBitMap.h"
#include "foundation/PxPinnedArray.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxNodeIndex.h"
#include "PxgBodySim.h"
#include "CmIDPool.h"
#include "CmBlockArray.h"
#include "foundation/PxHashMap.h"

namespace physx
{
	class PxsRigidBody;
	struct PxsExternalAccelerationProvider;

	namespace IG
	{
		class NodeIndex;
	}

	namespace Dy
	{
		class FeatherstoneArticulation;
		class DeformableSurface;
		class DeformableVolume;
		class ParticleSystem;
	}

	struct PxgRemapIndices
	{
	public:
		PxU32 nodeIndex; //this is the index to PxgBodySim in GPU
		PxU32 remapIndex; //this is the index map between PxgBodySim and the PxgArticulation in GPU
	};

	struct PxgArticulationUpdate
	{
		PxU32 articulationIndex; //Which articulation on GPU
		Dy::FeatherstoneArticulation* articulation; //Which articulation on CPU
	};
	struct PxgArticulationIndices	: public PxgRemapIndices {};
	struct PxgSoftBodyIndices		: public PxgRemapIndices {};
	struct PxgFEMClothIndices		: public PxgRemapIndices {};
	struct PxgParticleSystemIndices	: public PxgRemapIndices {};

	struct PxgStaticConstraint
	{
		PxU32 uniqueId;
		PxU32 linkID;
	};

	struct PxgSelfConstraint
	{
		PxU32 uniqueId;
		PxU32 linkID0;
		PxU32 linkID1;
	};

	struct PxgStaticConstraints
	{
		static const PxU32 MaxConstraints = 16;
		PxArray<PxgStaticConstraint> mStaticContacts;
		PxArray<PxgStaticConstraint> mStaticJoints;
		
	};

	struct PxgArticulationSelfConstraints
	{
		static const PxU32 MaxConstraints = 32;
		PxArray<PxgSelfConstraint> mSelfContacts;
		PxArray<PxgSelfConstraint> mSelfJoints;
	};

	class PxgBodySimManager
	{
		PX_NOCOPY(PxgBodySimManager)
	public:
		PxgBodySimManager(const PxVirtualAllocator& allocator) : mNewUpdatedBodies(allocator), 
			mTotalNumBodies(0), mNbUpdatedBodies(0), 
			mTotalNumArticulations(0), mTotalNumSoftBodies(0), mTotalNumFEMCloths(0),
			mTotalNumPBDParticleSystems(0),
			mActivePBDParticleSystems(allocator),
			mActivePBDParticleSystemsDirty(false),
			mActiveSoftbodies(allocator),
			mActiveSelfCollisionSoftbodies(allocator),
			mActiveSoftbodiesDirty(false),
			mActiveFEMCloths(allocator),
			mActiveFEMClothsDirty(false),
			mTotalStaticArticContacts(0), 
			mTotalStaticArticJoints(0),
			mTotalSelfArticContacts(0),
			mTotalSelfArticJoints(0),
			mMaxStaticArticContacts(0),
			mMaxStaticArticJoints(0),
			mMaxSelfArticContacts(0),
			mMaxSelfArticJoints(0),
			mTotalStaticRBContacts(0),
			mTotalStaticRBJoints(0),
			mMaxStaticRBContacts(0),
			mMaxStaticRBJoints(0),
			mExternalAccelerations(NULL)
		{
		}

		~PxgBodySimManager();

		void	addBody(PxsRigidBody* bodyCore, const PxU32 nodeIndex);

		void	addArticulation(Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex, bool OmniPVDRecordDirectGPUAPI);
		void	releaseArticulation(Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex);
		void	releaseDeferredArticulationIds();

		void	addSoftBody(Dy::DeformableVolume* deformableVolume, const PxU32 nodeIndex);
		void	releaseSoftBody(Dy::DeformableVolume* deformableVolume);
		void	releaseDeferredSoftBodyIds();
		bool	activateSoftbody(Dy::DeformableVolume* deformableVolume);
		bool	deactivateSoftbody(Dy::DeformableVolume* deformableVolume);

		bool	activateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume);
		bool	deactivateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume);

		void	addFEMCloth(Dy::DeformableSurface*, const PxU32 nodeIndex);
		void	releaseFEMCloth(Dy::DeformableSurface*);
		void	releaseDeferredFEMClothIds();
		bool	activateCloth(Dy::DeformableSurface*);
		bool	deactivateCloth(Dy::DeformableSurface*);

		void	addPBDParticleSystem(Dy::ParticleSystem* particleSystem, const PxU32 nodeIndex);
		void	releasePBDParticleSystem(Dy::ParticleSystem* particleSystem);
		void	releaseDeferredPBDParticleSystemIds();

		void	updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies, PxsExternalAccelerationProvider* externalAccelerations);
		void	updateBody(const PxNodeIndex&);
		void	destroy();

		void	updateArticulation(Dy::FeatherstoneArticulation* articulation, const PxU32 nodeIndex);

		void	reset();
		void	reserve(const PxU32 nbBodies);

		bool	addStaticArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		bool	addStaticArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		bool	addSelfArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);
		bool	removeSelfArticulationContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);

		bool	addSelfArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);
		bool	removeSelfArticulationJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex0, const PxNodeIndex nodeIndex1);

		bool	addStaticRBContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticRBContactManager(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		bool	addStaticRBJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);
		bool	removeStaticRBJoint(PxU32 uniqueIndex, const PxNodeIndex nodeIndex);

		PX_INLINE PxU32	getArticulationRemapIndex(PxU32 nodeIndex) { return mNodeToRemapMap.find(nodeIndex)->second;}

		PX_INLINE PxU32 getNumActiveParticleSystem()
		{
			return mActivePBDParticleSystems.size();
		}

		PxArray<PxgArticulationUpdate>							mUpdatedArticulations;
		PxArray<void*>											mBodies; //rigid bodies, articulations, soft bodies and particle systems
		
		PxArray<PxU32>											mNewOrUpdatedBodySims;
		PxArray<PxgArticulationIndices>							mNewArticulationSims;
		PxArray<PxgSoftBodyIndices>								mNewSoftBodySims;
		PxArray<PxgFEMClothIndices>								mNewFEMClothSims;
		PxArray<Dy::DeformableSurface*>							mDeformableSurfaces;
		PxArray<Dy::DeformableVolume*>							mDeformableVolumes;
		PxArray<PxgParticleSystemIndices>						mNewPBDParticleSystemSims;


		Cm::DeferredIDPool										mArticulationIdPool; //generate the remap id between pxgbodysim and pxgarticulation
		Cm::DeferredIDPool										mSoftBodyIdPool; //generate the remap id between pxgbodysim and pxgsoftbody
		Cm::DeferredIDPool										mFEMClothIdPool; //generate the remap id between pxgbodysim and pxgfemcloth
		Cm::DeferredIDPool										mPBDParticleSystemIdPool; //generate the remap id between pxgbodysim and pxgparticlesystem


		PxPinnedArray<PxgBodySimVelocityUpdate>					mNewUpdatedBodies;
		PxU32													mTotalNumBodies; //include rigid body and articulation
		PxU32													mNbUpdatedBodies; //this is used for multiply threads in the ScBeforeSolverTask to update body information

		PxBitMap												mUpdatedMap;
		PxU32													mTotalNumArticulations;
		PxU32													mTotalNumSoftBodies;
		PxU32													mTotalNumFEMCloths;
		PxU32													mTotalNumPBDParticleSystems;
		PxArray<PxU32>											mActiveFEMClothIndex;
		PxArray<PxU32>											mActiveSoftbodyIndex;
		PxArray<PxU32>											mActiveSelfCollisionSoftbodyIndex;

		PxInt32ArrayPinned										mActivePBDParticleSystems;
		bool													mActivePBDParticleSystemsDirty;
		PxInt32ArrayPinned										mActiveSoftbodies;
		PxInt32ArrayPinned										mActiveSelfCollisionSoftbodies;
		PxArray<PxU32>											mActiveSoftbodiesStaging;
		PxArray<PxU32>											mActiveSelfCollisionSoftBodiesStaging;
		bool													mActiveSoftbodiesDirty;
		PxInt32ArrayPinned										mActiveFEMCloths;
		PxArray<PxU32>											mActiveFEMClothStaging;
		bool													mActiveFEMClothsDirty;

		PxHashMap<PxU32, PxU32>									mNodeToRemapMap;
#if PX_SUPPORT_OMNI_PVD
		PxHashMap<PxU32, PxU32>									mRemapToNodeMap;
#endif
		PxArray<PxU32>											mDeferredFreeNodeIDs;

		Cm::BlockArray<PxgStaticConstraints, 1024>				mStaticConstraints;
		Cm::BlockArray<PxgArticulationSelfConstraints, 1024>	mArticulationSelfConstraints;

		PxU32													mTotalStaticArticContacts;
		PxU32													mTotalStaticArticJoints;

		PxU32													mTotalSelfArticContacts;
		PxU32													mTotalSelfArticJoints;

		PxU32													mMaxStaticArticContacts;
		PxU32													mMaxStaticArticJoints;

		PxU32													mMaxSelfArticContacts;
		PxU32													mMaxSelfArticJoints;

		PxU32													mTotalStaticRBContacts;
		PxU32													mTotalStaticRBJoints;

		PxU32													mMaxStaticRBContacts;
		PxU32													mMaxStaticRBJoints;

		PxsExternalAccelerationProvider*						mExternalAccelerations;
	};

}

#endif
