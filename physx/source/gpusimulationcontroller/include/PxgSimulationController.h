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

#ifndef PXG_SIMULATION_CONTROLLER_H
#define	PXG_SIMULATION_CONTROLLER_H

#include "PxgBodySimManager.h"
#include "PxgJointManager.h"
#include "PxsSimulationController.h"
#include "PxgHeapMemAllocator.h"
#include "CmTask.h"
#include "PxgArticulationLink.h"
#include "DyArticulationJointCore.h"
#include "PxgArticulation.h"
#include "PxgArticulationTendon.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgSoftBody.h"
#include "PxgFEMCloth.h"
#include "PxgParticleSystem.h"
#include "PxArticulationTendonData.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

#include "BpAABBManagerBase.h"
#include "PxgAABBManager.h"
#include "PxsTransformCache.h"
#include "PxgNarrowphaseCore.h"
#define PXG_SC_DEBUG	0

namespace physx
{
	//this is needed to force PhysXSimulationControllerGpu linkage as Static Library!
	void createPxgSimulationController();

	namespace shdfnd
	{
		class PxVirtualAllocatorCallback;
	}

	namespace Dy
	{
		class ParticleSystemCore;
	}

	namespace Bp
	{
		class BroadPhase;
	}

	class PxgSimulationCore;
	class PxgPBDParticleSystemCore;
	class PxgSoftBodyCore;
	class PxgFEMClothCore;
	class PxgGpuContext;
	class PxgNphaseImplementationContext;
	struct PxsCachedTransform;

	class PxgSimulationController;
	class PxgCudaKernelWranglerManager;
	class PxgCudaBroadPhaseSap;

	struct SoftBodyAttachmentAndFilterData
	{
	public:
		PxPinnedArray<PxgFEMRigidAttachment>* rigidAttachments;
		PxPinnedArray<PxgRigidFilterPair>* rigidFilterPairs;
		bool dirtyRigidAttachments;
		PxInt32ArrayPinned* activeRigidAttachments;
		bool dirtyActiveRigidAttachments;
		PxPinnedArray<PxgFEMFEMAttachment>* softBodyAttachments;
		bool dirtySoftBodyAttachments;
		PxInt32ArrayPinned* activeSoftBodyAttachments;
		bool dirtyActiveSoftBodyAttachments;
		PxArray<Dy::DeformableVolume*>* dirtyDeformableVolumeForFilterPairs;
		PxPinnedArray<PxgFEMFEMAttachment>* clothAttachments;
		PxPinnedArray<PxgNonRigidFilterPair>* clothFilterPairs;
		bool dirtyClothAttachments;
		PxInt32ArrayPinned* activeClothAttachments;
		bool dirtyActiveClothAttachments;
		PxPinnedArray<PxgFEMFEMAttachment>* particleAttachments;
		PxPinnedArray<PxgNonRigidFilterPair>* particleFilterPairs;
		bool dirtyParticleAttachments;
		PxInt32ArrayPinned* activeParticleAttachments;
		bool dirtyActiveParticleAttachments;
	};
	
	class PxgCopyToBodySimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mNewBodySimOffset;
		PxU32 mStartIndex;
		PxU32 mNbToProcess;

	public:
		PxgCopyToBodySimTask(PxgSimulationController& controller, PxU32 bodySimOffset, PxU32 startIdx, PxU32 nbToProcess) : Cm::Task(0), mController(controller), 
			mNewBodySimOffset(bodySimOffset), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgCopyToBodySimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToBodySimTask)
	};

	class PxgCopyToArticulationSimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mNewBodySimOffset; 
		PxU32 mStartIndex, mNbToProcess;
		PxI32* mSharedArticulationLinksIndex;
		PxI32* mSharedDofIndex;
		PxI32* mSharedSpatialTendonIndex;
		PxI32* mSharedSpatialTendonAttachmentIndex;
		PxI32* mSharedFixedTendonIndex;
		PxI32* mSharedFixedTendonJointIndex;
		PxI32* mSharedArticulationMimicJointIndex;
		PxI32* mSharedPathToRootIndex;

	public:
		PxgCopyToArticulationSimTask(PxgSimulationController& controller, const PxU32 bodySimOffset, PxU32 startIdx, 
			PxU32 nbToProcess, PxI32* sharedArticulationLinksIndex, PxI32* sharedDofIndex, 
			PxI32* sharedSpatialTendonIndex,
			PxI32* sharedSpatialTendonAttachmentsIndex,
			PxI32* sharedFixedTendonIndex,
			PxI32* sharedFixedTendonJointIndex,
			PxI32* sharedArticulationMimicJointIndex,
			PxI32* sharedPathToRootIndex) :
			Cm::Task(0), mController(controller), mNewBodySimOffset(bodySimOffset), mStartIndex(startIdx),
			mNbToProcess(nbToProcess), mSharedArticulationLinksIndex(sharedArticulationLinksIndex),
			mSharedDofIndex(sharedDofIndex), 
			mSharedSpatialTendonIndex(sharedSpatialTendonIndex),
			mSharedSpatialTendonAttachmentIndex(sharedSpatialTendonAttachmentsIndex),
			mSharedFixedTendonIndex(sharedFixedTendonIndex), 
			mSharedFixedTendonJointIndex(sharedFixedTendonJointIndex),
			mSharedArticulationMimicJointIndex(sharedArticulationMimicJointIndex),
			mSharedPathToRootIndex(sharedPathToRootIndex)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgCopyToArticulationSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToArticulationSimTask)
	};

	class PxgUpdateArticulationSimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mStartIndex, mNbToProcess;
		PxI32* mSharedArticulationLinksIndex;
		PxI32* mSharedArticulationDofIndex;
		PxI32* mSharedSpatialTendonIndex;
		PxI32* mSharedSpatialTendonAttachmentIndex;
		PxI32* mSharedFixedTendonIndex;
		PxI32* mSharedFixedTendonJointIndex;
		PxI32* mSharedMimicJointIndex;

	public:
		PxgUpdateArticulationSimTask(PxgSimulationController& controller, PxU32 startIdx,
			PxU32 nbToProcess, PxI32* sharedArticulationLinksLindex, 
			PxI32* sharedArticulationDofIndex,
			PxI32* sharedSpatialTendonIndex,
			PxI32* sharedSpatialTendonAttachmentIndex,
			PxI32* sharedFixedTendonIndex,
			PxI32* sharedFixedTendonJointIndex,
			PxI32* sharedMimicJointIndex) :
			Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess), mSharedArticulationLinksIndex(sharedArticulationLinksLindex),
			mSharedArticulationDofIndex(sharedArticulationDofIndex),
			mSharedSpatialTendonIndex(sharedSpatialTendonIndex),
			mSharedSpatialTendonAttachmentIndex(sharedSpatialTendonAttachmentIndex),
			mSharedFixedTendonIndex(sharedFixedTendonIndex),
			mSharedFixedTendonJointIndex(sharedFixedTendonJointIndex),
			mSharedMimicJointIndex(sharedMimicJointIndex)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgUpdateArticulationSimTask";
		}

	private:
		PX_NOCOPY(PxgUpdateArticulationSimTask)
	};

	class PxgCopyToSoftBodySimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mStartIndex, mNbToProcess;

	public:

		static const PxU32 NbSoftBodiesPerTask = 50;

		PxgCopyToSoftBodySimTask(PxgSimulationController& controller, PxU32 startIdx, PxU32 nbToProcess) :
			Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgCopyToSoftBodySimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToSoftBodySimTask)
	};

	class PxgCopyToFEMClothSimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		PxU32 mStartIndex, mNbToProcess;

	public:

		static const PxU32 NbFEMClothsPerTask = 50;

		PxgCopyToFEMClothSimTask(PxgSimulationController& controller, PxU32 startIdx, PxU32 nbToProcess) :
			Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgCopyToFEMClothSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToFEMClothSimTask)
	};

	class PxgCopyToPBDParticleSystemSimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		PxgParticleSystemCore* core;
		PxU32 mStartIndex, mNbToProcess;

	public:
		PxgCopyToPBDParticleSystemSimTask(PxgSimulationController& controller, PxU32 startIdx, PxU32 nbToProcess) :
			Cm::Task(0), mController(controller), mStartIndex(startIdx),
			mNbToProcess(nbToProcess)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgCopyToPBDParticleSystemSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToPBDParticleSystemSimTask)
	};

	class PxgPostCopyToShapeSimTask : public Cm::Task
	{
		PxgSimulationController& mController;

	public:
		PxgPostCopyToShapeSimTask(PxgSimulationController& controller) : Cm::Task(0), mController(controller)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgPostCopyToShapeSimTask";
		}
	private:
		PX_NOCOPY(PxgPostCopyToShapeSimTask)
	};

	class PxgPostCopyToBodySimTask : public Cm::Task
	{
		PxgSimulationController& mController;
		const bool mEnableBodyAccelerations;

	public:
		PxgPostCopyToBodySimTask(PxgSimulationController& controller, bool enableBodyAccelerations) : Cm::Task(0), mController(controller), mEnableBodyAccelerations(enableBodyAccelerations)
		{
		}

		virtual void runInternal();

		virtual const char* getName() const
		{
			return "PxgPostCopyToBodySimTask";
		}
	private:
		PX_NOCOPY(PxgPostCopyToBodySimTask)
	};
	class PxgPostUpdateParticleAndSoftBodyTask : public Cm::Task
	{
		PxgSimulationController&	mController;
		PxVec3						mGravity;
		PxReal						mDt;

	public:
		PxgPostUpdateParticleAndSoftBodyTask(PxgSimulationController& controller) : Cm::Task(0), mController(controller) {}

		virtual void runInternal();

		void setGravity(const PxVec3 gravity) { mGravity = gravity; }
		void setDt(const PxReal dt) { mDt = dt; }

		virtual const char* getName() const
		{
			return "PxgPostUpdateParticleAndSoftBodyTask";
		}
	private:
		PX_NOCOPY(PxgPostUpdateParticleAndSoftBodyTask)
	};

	template <typename Attachment>
	class AttachmentManager
	{
	public:
		PxPinnedArray<Attachment>	mAttachments;
		PxInt32ArrayPinned	mActiveAttachments;
		PxHashMap<PxU32, PxU32> mHandleToAttachmentMapping;
		PxHashMap<PxU32, PxU32>	mHandleToActiveIndex;
		PxArray<PxU32> mHandles;

		PxU32 mBaseHandle;
		bool mAttachmentsDirty;
		bool mActiveAttachmentsDirty;

		AttachmentManager(PxgHeapMemoryAllocatorManager* manager) : 
			mAttachments(manager->mMappedMemoryAllocators),
			mActiveAttachments(manager->mMappedMemoryAllocators),
			mBaseHandle(0),
			mAttachmentsDirty(false),
			mActiveAttachmentsDirty(false)
		{
		}

		void addAttachment(const Attachment& attachment, const PxU32 handle)
		{
			const PxU32 size = mAttachments.size();
			mAttachments.pushBack(attachment);
			mHandles.pushBack(handle);
			mHandleToAttachmentMapping[handle] = size;
			mAttachmentsDirty = true;
		}

		bool removeAttachment(const PxU32 handle)
		{
			deactivateAttachment(handle);

			//Now remove this current handle...
			PxHashMap<PxU32, PxU32>::Entry mapping;
			bool found = mHandleToAttachmentMapping.erase(handle, mapping);
			if (found)
			{
				mAttachments.replaceWithLast(mapping.second);
				mHandles.replaceWithLast(mapping.second);
				if (mapping.second < mAttachments.size())
				{
					PxU32 newHandle = mHandles[mapping.second];
					mHandleToAttachmentMapping[newHandle] = mapping.second;
					const PxHashMap<PxU32, PxU32>::Entry* activeMapping = mHandleToActiveIndex.find(newHandle);
					if (activeMapping)
					{
						mActiveAttachments[activeMapping->second] = mapping.second;
					}
				}
				mAttachmentsDirty = true;
			}
			return found;
		}

		void activateAttachment(const PxU32 handle)
		{
			PX_ASSERT(!mHandleToActiveIndex.find(handle));
			PxU32 index = mHandleToAttachmentMapping[handle];
			mHandleToActiveIndex[handle] = mActiveAttachments.size();
			mActiveAttachments.pushBack(index);
			mActiveAttachmentsDirty = true;
		}

		void deactivateAttachment(const PxU32 handle)
		{
			PxHashMap<PxU32, PxU32>::Entry mapping;
			bool found = mHandleToActiveIndex.erase(handle, mapping);
			if (found)
			{
				mActiveAttachments.replaceWithLast(mapping.second);

				if (mapping.second < mActiveAttachments.size())
				{
					PxU32 replaceHandle = mHandles[mActiveAttachments[mapping.second]];
					mHandleToActiveIndex[replaceHandle] = mapping.second;
				}

				mActiveAttachmentsDirty = true;
			}
		}
	};

	class PxgSimulationController : public PxsSimulationController
	{
		PX_NOCOPY(PxgSimulationController)
	public:
		PxgSimulationController(PxsKernelWranglerManager* gpuWranglerManagers, PxCudaContextManager* cudaContextManager,
			PxgGpuContext* dynamicContext, PxgNphaseImplementationContext* npContext, Bp::BroadPhase* bp, bool useGpuBroadphase,
			PxsSimulationControllerCallback* callback, PxgHeapMemoryAllocatorManager* heapMemoryManager,
			PxU32 maxSoftBodyContacts, PxU32 maxFemClothContacts, PxU32 maxParticleContacts, PxU32 collisionStackSizeBytes, bool enableBodyAccelerations);

		virtual ~PxgSimulationController();

		virtual void addPxgShape(Sc::ShapeSimBase* shapeSimBase, const PxsShapeCore* shapeCore, PxNodeIndex nodeIndex, PxU32 index)	PX_OVERRIDE;
		virtual void setPxgShapeBodyNodeIndex(PxNodeIndex nodeIndex, PxU32 index)	PX_OVERRIDE;
		virtual void removePxgShape(PxU32 index)	PX_OVERRIDE;

		virtual void addDynamic(PxsRigidBody* rigidBody, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void addDynamics(PxsRigidBody** rigidBody, const PxU32* nodeIndex, PxU32 nbBodies)	PX_OVERRIDE;

		virtual void addArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseDeferredArticulationIds()	PX_OVERRIDE;

		virtual void addParticleFilter(Dy::DeformableVolume* deformableVolume, Dy::ParticleSystem* particleSystem,
			PxU32 particleId, PxU32 userBufferId, PxU32 tetId)	PX_OVERRIDE;
		virtual void removeParticleFilter(Dy::DeformableVolume* deformableVolume,
			const Dy::ParticleSystem* particleSystem, PxU32 particleId, PxU32 userBufferId, PxU32 tetId)	PX_OVERRIDE;

		virtual PxU32 addParticleAttachment(Dy::DeformableVolume* deformableVolume, const Dy::ParticleSystem* particleSystem,
			 PxU32 particleId, PxU32 userBufferId, PxU32 tetId, const PxVec4& barycentrics, const bool isActive)	PX_OVERRIDE;
		virtual void removeParticleAttachment(Dy::DeformableVolume* deformableVolume, PxU32 handle)	PX_OVERRIDE;

		virtual void addRigidFilter(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex)	PX_OVERRIDE;
		virtual void removeRigidFilter(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex)	PX_OVERRIDE;

		virtual PxU32 addRigidAttachment(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& softBodyNodeIndex,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex, const PxVec3& actorSpacePose,
			PxConeLimitedConstraint* constraint, const bool isActive, bool doConversion)	PX_OVERRIDE;
		
		virtual PxU32 addTetRigidAttachment(Dy::DeformableVolume* deformableVolume,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 tetIdx, const PxVec4& barycentrics, const PxVec3& actorSpacePose,
			PxConeLimitedConstraint* constraint, const bool isActive, bool doConversion)	PX_OVERRIDE;

		virtual void removeRigidAttachment(Dy::DeformableVolume* deformableVolume, PxU32 handle)	PX_OVERRIDE;

		virtual void addTetRigidFilter(Dy::DeformableVolume* deformableVolume,
			const PxNodeIndex& rigidNodeIndex, PxU32 tetId)	PX_OVERRIDE;
		virtual void removeTetRigidFilter(Dy::DeformableVolume* deformableVolume,
			const PxNodeIndex& rigidNodeIndex, PxU32 tetId)	PX_OVERRIDE;

		virtual void addSoftBodyFilter(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32 tetIdx0,
			PxU32 tetIdx1)	PX_OVERRIDE;
		virtual void removeSoftBodyFilter(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32 tetIdx0,
			PxU32 tetId1)	PX_OVERRIDE;
		virtual void addSoftBodyFilters(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1,
			PxU32 tetIndicesSize)	PX_OVERRIDE;
		virtual void removeSoftBodyFilters(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices0, PxU32* tetIndices1,
			PxU32 tetIndicesSize)	PX_OVERRIDE;

		virtual PxU32 addSoftBodyAttachment(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32 tetIdx0, PxU32 tetIdx1,
			const PxVec4& tetBarycentric0, const PxVec4& tetBarycentric1, PxConeLimitedConstraint* constraint, PxReal constraintOffset,
			const bool addToActive, bool doConversion)	PX_OVERRIDE;

		virtual void removeSoftBodyAttachment(Dy::DeformableVolume* deformableVolume0, PxU32 handle)	PX_OVERRIDE;

		virtual void addClothFilter(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface, PxU32 triIdx, PxU32 tetIdx)	PX_OVERRIDE;
		virtual void removeClothFilter(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface, PxU32 triId, PxU32 tetIdx)	PX_OVERRIDE;

		virtual PxU32 addClothAttachment(Dy::DeformableVolume* deformableVolume, Dy::DeformableSurface* deformableSurface, PxU32 triIdx,
			const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric,
			PxConeLimitedConstraint* constraint, PxReal constraintOffset,
			const bool isActive, bool doConversion)	PX_OVERRIDE;

		virtual void removeClothAttachment(Dy::DeformableVolume* deformableVolume, PxU32 handle)	PX_OVERRIDE;

		virtual PxU32 addRigidAttachment(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& clothNodeIndex,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 vertIndex, const PxVec3& actorSpacePose,
			PxConeLimitedConstraint* constraint, const bool isActive)	PX_OVERRIDE;

		virtual void removeRigidAttachment(Dy::DeformableSurface* deformableSurface, PxU32 handle)	PX_OVERRIDE;

		virtual void addTriRigidFilter(Dy::DeformableSurface* deformableSurface,
			const PxNodeIndex& rigidNodeIndex, PxU32 triIdx)	PX_OVERRIDE;

		virtual void removeTriRigidFilter(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& rigidNodeIndex,PxU32 triIdx)	PX_OVERRIDE;

		virtual PxU32 addTriRigidAttachment(Dy::DeformableSurface* deformableSurface,
			PxsRigidBody* rigidBody, const PxNodeIndex& rigidNodeIndex, PxU32 triIdx, const PxVec4& barycentrics, 
			const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint,
			const bool isActive)	PX_OVERRIDE;

		virtual void removeTriRigidAttachment(Dy::DeformableSurface* deformableSurface, PxU32 handle)	PX_OVERRIDE;

		virtual void addClothFilter(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1, PxU32 triIdx0, PxU32 triIdx1)	PX_OVERRIDE;
		virtual void removeClothFilter(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1, PxU32 triIdx0, PxU32 triId1)	PX_OVERRIDE;

		virtual PxU32 addTriClothAttachment(Dy::DeformableSurface* deformableSurface0, Dy::DeformableSurface* deformableSurface1, PxU32 triIdx0, PxU32 triIdx1,
			const PxVec4& triBarycentric0, const PxVec4& triBarycentric1, const bool addToActive)	PX_OVERRIDE;
		
		virtual void removeTriClothAttachment(Dy::DeformableSurface* deformableSurface0, PxU32 handle)	PX_OVERRIDE;

		PxU32 addRigidAttachmentInternal(const PxU32 nonRigidId, const PxU32 elemId, const bool isVertex, const PxVec4& barycentric, PxsRigidBody* rigidBody,
			const PxNodeIndex& rigidNodeIndex, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint,
			AttachmentManager<PxgFEMRigidAttachment>& attachments, bool addToActive);

		void addSoftBodyFiltersInternal(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices, PxU32 size);
		void removeSoftBodyFiltersInternal(Dy::DeformableVolume* deformableVolume0, Dy::DeformableVolume* deformableVolume1, PxU32* tetIndices, PxU32 size);

		void createDeformableSurfaceCore();
		void createDeformableVolumeCore();
		virtual void addSoftBody(Dy::DeformableVolume* deformableVolume, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseSoftBody(Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void releaseDeferredSoftBodyIds()	PX_OVERRIDE;
		virtual void activateSoftbody(Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void deactivateSoftbody(Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void activateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void deactivateSoftbodySelfCollision(Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;
		virtual void setSoftBodyWakeCounter(Dy::DeformableVolume* deformableVolume)	PX_OVERRIDE;

		virtual void addFEMCloth(Dy::DeformableSurface* deformableSurface, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseFEMCloth(Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;
		virtual void releaseDeferredFEMClothIds()	PX_OVERRIDE;
		virtual void activateCloth(Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;
		virtual void deactivateCloth(Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;
		virtual void setClothWakeCounter(Dy::DeformableSurface* deformableSurface)	PX_OVERRIDE;

		virtual void addParticleSystem(Dy::ParticleSystem* particleSystem, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void releaseParticleSystem(Dy::ParticleSystem* particleSystem)	PX_OVERRIDE;
		virtual void releaseDeferredParticleSystemIds()	PX_OVERRIDE;

		virtual void updateDynamic(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies, PxsExternalAccelerationProvider* externalAccelerations)	PX_OVERRIDE;
		virtual void addJoint(const Dy::Constraint&) PX_OVERRIDE;
		virtual void updateJoint(const PxU32 edgeIndex, Dy::Constraint* constraint)	PX_OVERRIDE;
		virtual void updateBodies(PxBaseTask* continuation)	PX_OVERRIDE;
		virtual void updateShapes(PxBaseTask* continuation)	PX_OVERRIDE;

		virtual void preIntegrateAndUpdateBound(PxBaseTask* continuation, const PxVec3 gravity, const PxReal dt)	PX_OVERRIDE;
		virtual void updateParticleSystemsAndSoftBodies()	PX_OVERRIDE;
		virtual void sortContacts()	PX_OVERRIDE;
		virtual void update(PxBitMapPinned& changedHandleMap)	PX_OVERRIDE;
		virtual void mergeChangedAABBMgHandle()	PX_OVERRIDE;
		virtual void gpuDmabackData(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBitMapPinned&  changedAABBMgrHandles, bool enableDirectGPUAPI)	PX_OVERRIDE;
		virtual void updateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation)	PX_OVERRIDE;
		virtual void updateArticulation(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateArticulationJoint(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateArticulationExtAccel(Dy::FeatherstoneArticulation* articulation, const PxNodeIndex& nodeIndex)	PX_OVERRIDE;
		virtual void updateArticulationAfterIntegration(PxsContext* /*llContext*/, Bp::AABBManagerBase* /*aabbManager*/,
			PxArray<Sc::BodySim*>& /*ccdBodies*/, PxBaseTask* /*continuation*/, IG::IslandSim& /*islandSim*/, float /*dt*/) PX_OVERRIDE	{}
		virtual PxU32* getActiveBodies()	PX_OVERRIDE;
		virtual PxU32* getDeactiveBodies()	PX_OVERRIDE;
		virtual void** getRigidBodies()	PX_OVERRIDE;
		virtual PxU32	getNbBodies()	PX_OVERRIDE;

		virtual Sc::ShapeSimBase** getShapeSims()	PX_OVERRIDE;
		virtual PxU32*	getUnfrozenShapes()	PX_OVERRIDE;
		virtual PxU32*	getFrozenShapes()	PX_OVERRIDE;
		virtual PxU32	getNbFrozenShapes()	PX_OVERRIDE;
		virtual PxU32	getNbUnfrozenShapes()	PX_OVERRIDE;
		virtual PxU32	getNbShapes()	PX_OVERRIDE;
		virtual void	clear()	PX_OVERRIDE	{ mNbFrozenShapes = 0; mNbUnfrozenShapes = 0; }
		virtual void	setBounds(Bp::BoundsArray* boundArray)	PX_OVERRIDE;
		virtual void	reserve(const PxU32 nbBodies)	PX_OVERRIDE;

		PX_INLINE PxU32   getArticulationRemapIndex(const PxU32 nodeIndex) { return mBodySimManager.getArticulationRemapIndex(nodeIndex); }
		
		virtual void	setDeformableSurfaceGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback) PX_OVERRIDE PX_FINAL;
		virtual void	setDeformableVolumeGpuPostSolveCallback(PxPostSolveCallback* postSolveCallback) PX_OVERRIDE PX_FINAL;

		// deprecated direct-GPU API

		PX_DEPRECATED	virtual	void	copySoftBodyDataDEPRECATED(void** data, void* dataEndIndices, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, CUevent copyEvent) PX_OVERRIDE PX_FINAL;
		PX_DEPRECATED	virtual	void	applySoftBodyDataDEPRECATED(void** data, void* dataEndIndices, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, CUevent applyEvent, CUevent signalEvent) PX_OVERRIDE PX_FINAL;	

		PX_DEPRECATED	virtual void	applyParticleBufferDataDEPRECATED(const PxU32* indices, const PxGpuParticleBufferIndexPair* indexPair, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, CUevent waitEvent, CUevent signalEvent)	PX_OVERRIDE;
		// end deprecated direct-GPU API

		// new direct-GPU API
		virtual bool	getRigidDynamicData(void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, float oneOverDt, CUevent startEvent, CUevent finishEvent) const PX_OVERRIDE PX_FINAL;
		virtual bool 	setRigidDynamicData(const void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;
		
		virtual bool 	getArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const PX_OVERRIDE PX_FINAL;
		virtual bool 	setArticulationData(const void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;
		virtual	bool	computeArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;

		virtual bool 	evaluateSDFDistances(PxVec4* localGradientAndSDFConcatenated, const PxShapeGPUIndex* shapeIndices, const PxVec4* localSamplePointsConcatenated, const PxU32* samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent, CUevent finishEvent) PX_OVERRIDE PX_FINAL;
		virtual	bool	copyContactData(void* data, PxU32* numContactPairs, const PxU32 maxContactPairs, CUevent startEvent, CUevent copyEvent) PX_OVERRIDE PX_FINAL;

		virtual PxArticulationGPUAPIMaxCounts getArticulationGPUAPIMaxCounts()	const	PX_OVERRIDE PX_FINAL;

		virtual bool	getD6JointData(void* data, const PxD6JointGPUIndex* gpuIndices, PxD6JointGPUAPIReadType::Enum dataType, PxU32 nbElements, PxF32 oneOverDt, CUevent startEvent, CUevent finishEvent) const PX_OVERRIDE PX_FINAL;

		// end new direct-GPU API

		virtual	PxU32	getInternalShapeIndex(const PxsShapeCore& shapeCore)	PX_OVERRIDE PX_FINAL;

		virtual void	syncParticleData()	PX_OVERRIDE;
		virtual void    updateBoundsAndShapes(Bp::AABBManagerBase& aabbManager, bool useDirectApi)	PX_OVERRIDE;

		PX_FORCE_INLINE PxgSimulationCore* getSimulationCore() { return mSimulationCore; }
		PX_FORCE_INLINE PxgJointManager& getJointManager() { return mJointManager; }
		PX_FORCE_INLINE PxgBodySimManager& getBodySimManager() { return mBodySimManager; }

		PX_FORCE_INLINE PxgPBDParticleSystemCore* getPBDParticleSystemCore() { return mPBDParticleSystemCore; }

		PX_FORCE_INLINE PxgSoftBodyCore* getSoftBodyCore() { return mSoftBodyCore; }

		PX_FORCE_INLINE PxgFEMClothCore* getFEMClothCore() { return mFEMClothCore; }

		PX_FORCE_INLINE PxgSoftBody* getSoftBodies() { return mSoftBodyPool.begin(); }
		PX_FORCE_INLINE PxU32 getNbSoftBodies() { return mSoftBodyPool.size(); }

		PX_FORCE_INLINE PxU32* getActiveSoftBodies() { return mBodySimManager.mActiveSoftbodies.begin(); }
		PX_FORCE_INLINE PxU32 getNbActiveSoftBodies() { return mBodySimManager.mActiveSoftbodies.size(); }

		PX_FORCE_INLINE PxU32* getSoftBodyNodeIndex() { return mSoftBodyNodeIndexPool.begin(); }

		PX_FORCE_INLINE PxgFEMCloth* getFEMCloths() { return mFEMClothPool.begin(); }
		PX_FORCE_INLINE PxU32 getNbFEMCloths() { return mFEMClothPool.size(); }

		PX_FORCE_INLINE PxU32* getActiveFEMCloths() { return mBodySimManager.mActiveFEMCloths.begin(); }
		PX_FORCE_INLINE PxU32 getNbActiveFEMCloths() { return mBodySimManager.mActiveFEMCloths.size(); }

		PX_FORCE_INLINE PxU32* getFEMClothNodeIndex() { return mFEMClothNodeIndexPool.begin(); }

		void postCopyToShapeSim();
		void postCopyToBodySim(bool enableBodyAccelerations);
		//integrate particle system and update bound/update grid/self collision
		void preIntegrateAndUpdateBoundParticleSystem(const PxVec3 gravity, const PxReal dt);
		void preIntegrateAndUpdateBoundSoftBody(const PxVec3 gravity, const PxReal dt);
		void preIntegrateAndUpdateBoundFEMCloth(const PxVec3 gravity, const PxReal dt);
		void updateJointsAndSyncData();

		void computeSoftBodySimMeshData(Dy::DeformableVolume* deformableVolume, PxU32 tetId, const PxVec4& tetBarycentric,
			PxU32& outTetId, PxVec4& outTetBarycentric);

		PX_FORCE_INLINE PxU32 getMaxLinks() { return mMaxLinks; }

		PX_FORCE_INLINE PxU32 getMaxFemContacts() { return mMaxFemClothContacts; }

		virtual PxU32					getNbDeactivatedDeformableSurfaces() const PX_OVERRIDE;
		virtual PxU32					getNbActivatedDeformableSurfaces() const PX_OVERRIDE;

		virtual Dy::DeformableSurface**	getDeactivatedDeformableSurfaces() const PX_OVERRIDE;
		virtual Dy::DeformableSurface**	getActivatedDeformableSurfaces() const PX_OVERRIDE;

		virtual PxU32					getNbDeactivatedDeformableVolumes() const PX_OVERRIDE;
		virtual PxU32					getNbActivatedDeformableVolumes() const PX_OVERRIDE;

		virtual Dy::DeformableVolume**	getDeactivatedDeformableVolumes() const PX_OVERRIDE;
		virtual Dy::DeformableVolume**	getActivatedDeformableVolumes() const PX_OVERRIDE;

		virtual const PxReal*			getDeformableVolumeWakeCounters() const PX_OVERRIDE;


		virtual void setEnableOVDReadback(bool enableOVDReadback) PX_OVERRIDE;
		virtual bool getEnableOVDReadback() const PX_OVERRIDE;

		virtual void setEnableOVDCollisionReadback(bool enableOVDCollisionsReadback) PX_OVERRIDE;
		virtual bool getEnableOVDCollisionReadback() const PX_OVERRIDE;

#if PX_SUPPORT_OMNI_PVD
		virtual void setOVDCallbacks(PxsSimulationControllerOVDCallbacks& ovdCallbacks) PX_OVERRIDE;
		PX_FORCE_INLINE PxsSimulationControllerOVDCallbacks* getOVDCallbacks() { return mOvdCallbacks; }
#endif
		virtual bool hasDeformableSurfaces() const PX_OVERRIDE	{ return mFEMClothCore != NULL;  }
		virtual bool hasDeformableVolumes() const PX_OVERRIDE	{ return mSoftBodyCore != NULL; }

		bool	 		getRecomputeArticulationBlockFormat() const { return mRecomputeArticulationBlockFormat; }

	private:

		void copyToGpuBodySim(PxBaseTask* continuation);
		void copyToGpuParticleSystem(PxBaseTask* continuation);
		void copyToGpuSoftBody(PxBaseTask* continuation);
		void copyToGpuFEMCloth(PxBaseTask* continuation);

		void copyToGpuBodySim(const PxU32 bodySimOffset, PxU32 bodyStartIndex, PxU32 nbToCopy);

		void copyToGpuArticulationSim(const PxU32 bodySimOffset, PxU32 startIndex, PxU32 nbToCopy, 
			PxI32* sharedArticulationLinksIndex, PxI32* sharedArticulationDofIndex, 
			PxI32* sharedArticulationSpatialTendonIndex,
			PxI32* sharedArticulationAttachmentIndex,
			PxI32* sharedArticulationFixedTendonIndex,
			PxI32* sharedArticulationTendonJointIndex,
			PxI32* sharedArticulationMimicJointIndex,
			PxI32* sharedArticulationPathToRootIndex);

		void updateGpuArticulationSim(PxU32 startIndex, PxU32 nbToCopy,
			PxI32* sharedArticulationLinksIndex, PxI32* sharedArticulationDofIndex,
			PxI32* sharedSpatialTendonIndex, PxI32* sharedAttachmentIndex,
			PxI32* sharedFixedTendonIndex, PxI32* sharedFixedTendonJointIndex,
			PxI32* sharedMimicJointIndex);

		void copyToGpuSoftBodySim(PxU32 startIndex, PxU32 nbToCopy);

		void copyToGpuFEMClothSim(PxU32 startIndex, PxU32 nbToCopy);
		
		void copyToGpuPBDParticleSystemSim(PxU32 startIndex, PxU32 nbToCopy);

		// bounds are shared by NP and BP, so we have the update in the simulation controller.
		// cache is for np, we update it together with bounds due to similar update logic
		void updateBoundsAndTransformCache(Bp::AABBManagerBase& aabbManager, CUstream stream, PxsTransformCache& cache, PxgCudaBuffer& mGpuTransformCache);

		void copyBoundsAndTransforms(Bp::BoundsArray& boundsArray, PxsTransformCache& transformCache,
															  PxgCudaBuffer& gpuTransformCache, PxU32 boundsArraySize,
															  PxU32 totalTransformCacheSize, CUstream npStream);
		void mergeBoundsAndTransformsChanges(PxgBoundsArray& directGPUBoundsArray,
																	  PxsTransformCache& transformCache,
																	  PxgCudaBuffer& gpuTransformCache, PxU32 boundsArraySize, PxU32 totalTransformCacheSize,
																		 PxU32 numChanges, CUstream npStream);

#if PXG_SC_DEBUG
		void validateCacheAndBounds(PxBoundsArrayPinned& boundArray, PxCachedTransformArrayPinned& cachedTransform);
#endif

		PxgPostCopyToShapeSimTask								mPostCopyShapeSimTask;
		PxgPostCopyToBodySimTask								mPostCopyBodySimTask;
		PxgPostUpdateParticleAndSoftBodyTask					mPostUpdateParticleSystemTask;
		PxgBodySimManager										mBodySimManager;
		PxgJointManager											mJointManager;

		PxgSimulationCore*										mSimulationCore;
		PxgSoftBodyCore*										mSoftBodyCore;
		PxgFEMClothCore*										mFEMClothCore;
		PxgPBDParticleSystemCore*								mPBDParticleSystemCore;

		PxgGpuContext*											mDynamicContext;
		PxgNphaseImplementationContext*							mNpContext;

		PxPinnedArray<PxgBodySim>								mNewBodySimPool;
		PxPinnedArray<PxgArticulationLink>						mLinksPool;
		PxFloatArrayPinned										mLinkWakeCounterPool;
		PxPinnedArray<Cm::UnAlignedSpatialVector>				mLinkAccelPool;
		PxPinnedArray<PxgArticulationLinkProp>					mLinkPropPool;
		PxPinnedArray<PxgArticulationLinkSleepData>				mLinkSleepDataPool;
		PxPinnedArray<ArticulationBitField>						mLinkChildPool;
		PxInt32ArrayPinned										mLinkParentPool;
		PxPinnedArray<PxTransform>								mLinkBody2WorldPool;
		PxPinnedArray<PxTransform>								mLinkBody2ActorPool;
		
		PxPinnedArray<Dy::ArticulationJointCore>				mJointPool;
		PxPinnedArray<Dy::ArticulationJointCoreData>			mJointDataPool;
		PxPinnedArray<PxgArticulationSimUpdate>					mLinkJointIndexPool; //this record the start index of the link for an articulation in an array
		PxPinnedArray<PxgArticulation>							mArticulationPool;
		PxPinnedArray<PxGpuSpatialTendonData>					mSpatialTendonParamPool;
		PxPinnedArray<PxgArticulationTendon>					mSpatialTendonPool;
		PxPinnedArray<PxgArticulationTendonElementFixedData>	mAttachmentFixedPool;
		PxPinnedArray<PxGpuTendonAttachmentData>				mAttachmentModPool;
		PxInt32ArrayPinned										mTendonAttachmentMapPool; //store each start index of the attachment to the corresponding tendons
		PxPinnedArray<PxGpuFixedTendonData>						mFixedTendonParamPool;
		PxPinnedArray<PxgArticulationTendon>					mFixedTendonPool;
		PxPinnedArray<PxgArticulationTendonElementFixedData>	mTendonJointFixedDataPool;
		PxPinnedArray<PxGpuTendonJointCoefficientData>			mTendonJointCoefficientDataPool;
		PxInt32ArrayPinned										mTendonTendonJointMapPool; //store each start index of the attachment to the corresponding tendons

		PxInt32ArrayPinned										mPathToRootPool;

		PxPinnedArray<Dy::ArticulationMimicJointCore>			mMimicJointPool;

		PxPinnedArray<PxgArticulationSimUpdate>					mArticulationUpdatePool;	//Articulation update headers
		PxFloatArrayPinned										mArticulationDofDataPool;	//Articulation dof information (jointV, jointP etc.)
		PxPinnedArray<PxgSoftBody>								mNewSoftBodyPool;
		PxArray<PxgSoftBodyData>								mNewSoftBodyDataPool;
		PxPinnedArray<PxgSoftBody>								mSoftBodyPool;
		PxArray<PxgSoftBodyData>								mSoftBodyDataPool;
		PxInt32ArrayPinned										mSoftBodyElementIndexPool;
		PxArray<PxU32>											mNewSoftBodyNodeIndexPool;
		PxArray<PxU32>											mNewSoftBodyElementIndexPool;
		PxArray<PxU32>											mSoftBodyNodeIndexPool;
		PxArray<PxU32>											mNewTetMeshByteSizePool;

		AttachmentManager<PxgFEMFEMAttachment>					mParticleSoftBodyAttachments;
		PxPinnedArray<PxgNonRigidFilterPair>					mSoftBodyParticleFilterPairs;
		PxArray<PxU32>											mSoftBodyParticleFilterRefs;

		AttachmentManager<PxgFEMRigidAttachment>				mRigidSoftBodyAttachments;
		PxPinnedArray<PxgRigidFilterPair>						mSoftBodyRigidFilterPairs;
		PxArray<PxU32>											mSoftBodyRigidFilterRefs;

		AttachmentManager<PxgFEMFEMAttachment>					mSoftBodySoftBodyAttachments;
		PxArray <Dy::DeformableVolume*>							mDirtyDeformableVolumeForFilterPairs;

		AttachmentManager<PxgFEMFEMAttachment>					mSoftBodyClothAttachments;
		PxPinnedArray<PxgNonRigidFilterPair>					mSoftBodyClothTetVertFilterPairs;
		PxArray<PxU32>											mSoftBodyClothTetVertFilterRefs;

		AttachmentManager<PxgFEMFEMAttachment>					mClothClothAttachments;
		PxPinnedArray<PxgNonRigidFilterPair>					mClothClothVertTriFilterPairs;
		PxArray<PxU32>											mClothClothVertTriFilterRefs;

		PxPinnedArray<PxgFEMCloth>								mNewFEMClothPool;
		PxArray<PxgFEMClothData>								mNewFEMClothDataPool;
		PxPinnedArray<PxgFEMCloth>								mFEMClothPool;
		PxArray<PxgFEMClothData>								mFEMClothDataPool;
		PxInt32ArrayPinned										mFEMClothElementIndexPool;
		PxArray<PxU32>											mNewFEMClothNodeIndexPool;
		PxArray<PxU32>											mNewFEMClothElementIndexPool;
		PxArray<PxU32>											mFEMClothNodeIndexPool;
		PxArray<PxU32>											mNewTriangleMeshByteSizePool;

		AttachmentManager<PxgFEMRigidAttachment>				mClothRigidAttachments;
		PxPinnedArray<PxgRigidFilterPair>						mClothRigidFilterPairs;
		PxArray<PxU32>											mClothRigidFilterRefs;

		PxInt32ArrayPinned										mFrozenPool;
		PxInt32ArrayPinned										mUnfrozenPool;
		PxInt32ArrayPinned										mActivatePool;
		PxInt32ArrayPinned										mDeactivatePool;

		PxU32													mNbFrozenShapes;
		PxU32													mNbUnfrozenShapes;
		bool													mHasBeenSimulated;//if there are no bodies in the scene, we don't run the update method so that we shouldn't need to syncback data

		PxI32													mSharedLinkIndex;
		PxI32													mSharedDofIndex;
		PxI32													mSharedSpatialTendonIndex;
		PxI32													mSharedSpatialAttachmentIndex;
		PxI32													mSharedFixedTendonIndex;
		PxI32													mSharedFixedTendonJointIndex;
		PxI32													mSharedMimicJointIndex;
		PxI32													mSharedPathToRootIndex;

		PxgCudaKernelWranglerManager*							mGpuWranglerManager;
		PxCudaContextManager*									mCudaContextManager;
		PxgHeapMemoryAllocatorManager*							mHeapMemoryManager;
		PxgCudaBroadPhaseSap*									mBroadPhase;
		PxU32													mMaxLinks;
		PxU32													mMaxDofs;
		PxU32													mMaxMimicJoints;
		PxU32													mMaxSpatialTendons;
		PxU32													mMaxAttachments;
		PxU32													mMaxFixedTendons;
		PxU32													mMaxTendonJoints;
		PxU32													mMaxPathToRoots;

		PxU32													mMaxSoftBodyContacts;
		PxU32													mMaxFemClothContacts;
		PxU32													mMaxParticleContacts;
		PxU32													mCollisionStackSizeBytes;

		bool 													mRecomputeArticulationBlockFormat;
		bool													mEnableOVDReadback;
		bool													mEnableOVDCollisionReadback;
#if PX_SUPPORT_OMNI_PVD
		PxsSimulationControllerOVDCallbacks*					mOvdCallbacks;
#endif
		friend class PxgCopyToBodySimTask;
		friend class PxgCopyToArticulationSimTask;
		friend class PxgUpdateArticulationSimTask;
		friend class PxgCopyToSoftBodySimTask;
		friend class PxgCopyToFEMClothSimTask;
		friend class PxgCopyToPBDParticleSystemSimTask;
		friend class PxgSimulationCore;
	};

}

#endif
