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

#ifndef PXG_SIMULATION_CORE_H
#define	PXG_SIMULATION_CORE_H

#include "PxgCudaBuffer.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgSoftBody.h"
#include "PxgSimulationController.h"
#include "PxgFEMCloth.h"
#include "foundation/PxUserAllocated.h"
#include "PxgFEMCore.h"
#include "PxgShapeSimManager.h"

namespace physx
{
	class PxCudaContextManager;
	class PxCudaContext;
	class KernelWrangler;
	class PxgCudaSolverCore;
	class PxgGpuNarrowphaseCore;
	class PxgCudaBroadPhaseSap;
	class PxgParticleSystemCore;
	class PxgSoftBodyCore;
	class PxgFEMClothCore;
	struct PxsCachedTransform;
	class PxgCudaKernelWranglerManager;
	class PxgSoftBodyBuffer;
	class PxgFEMClothBuffer;
	class PxgParticleSystemBuffer;
	class PxgBodySimManager;
	class PxgGpuContext;
	class PxSceneDesc;
	
	class PxgSimulationCore : public PxUserAllocated
	{
		PX_NOCOPY(PxgSimulationCore)

	public:
		PxgSimulationCore(PxgCudaKernelWranglerManager* gpuKernelWrangler,
			PxCudaContextManager* cudaContextManager,
			PxgHeapMemoryAllocatorManager* heapMemoryManager,
			PxgGpuContext* gpuContext,
			const bool useGpuBroadphase);

		~PxgSimulationCore();

		void gpuMemDmaUpBodySim(PxPinnedArray<PxgBodySimVelocityUpdate>& updatedBodySim,
			PxPinnedArray<PxgBodySim>& newBodySim,
			PxPinnedArray<PxgArticulationLink>& newLinkPool,
			PxFloatArrayPinned& newLinkWakeCounterPool,
			PxPinnedArray<Cm::UnAlignedSpatialVector>& newLinkExtAccelPool,
			PxPinnedArray<PxgArticulationLinkProp>& newLinkPropPool,
			PxInt32ArrayPinned& newLinkParentsPool,
			PxPinnedArray<Dy::ArticulationBitField>& newLinkChildPool,
			PxPinnedArray<PxTransform>& newLinkBody2WorldsPool,
			PxPinnedArray<PxTransform>& newLinkBody2ActorsPool,
			PxPinnedArray<Dy::ArticulationJointCore>& newJointCorePool,
			PxPinnedArray<Dy::ArticulationJointCoreData>& newJointDataPool,
			PxPinnedArray<PxgArticulationSimUpdate>& newLinkJointIndexPool,
			PxPinnedArray<PxgArticulation>& newArticulationPool,
			PxPinnedArray<PxGpuSpatialTendonData>& newSpatialTendonParamsPool,
			PxPinnedArray<PxgArticulationTendon>& newSpatialTendonPool,
			PxPinnedArray<PxgArticulationTendonElementFixedData>& newAttachmentFixedPool,
			PxPinnedArray<PxGpuTendonAttachmentData>& newAttachmentModPool,
			PxInt32ArrayPinned& newTendonToAttachmentRemapPool,
			PxPinnedArray<PxGpuFixedTendonData>& newFixedTendonParamsPool,
			PxPinnedArray<PxgArticulationTendon>& newFixedTendonPool,
			PxPinnedArray<PxgArticulationTendonElementFixedData>& newTendonJointFixedPool,
			PxPinnedArray<PxGpuTendonJointCoefficientData>& newTendonJointCoefficientPool,
			PxInt32ArrayPinned& newTendonToTendonJointRemapPool,
			PxPinnedArray<Dy::ArticulationMimicJointCore>& newMimicJointPool,
			PxInt32ArrayPinned& newPathToRootPool,
			PxU32 nbTotalBodies, PxU32 nbTotalArticulations, PxU32 maxLinks, 
			PxU32 maxDofs, PxU32 maxMimicJoints, PxU32 maxSpatialTendons, 
			PxU32 maxAttachments, PxU32 maxFixedTendons, PxU32 maxTendonJoints,
			bool enableBodyAccelerations);

		void gpuMemDmaUpSoftBodies(PxPinnedArray<PxgSoftBody>& newSoftBodyPool,
			PxU32* newTetMeshByteSizePool,
			PxArray<PxgSoftBodyData>& newSoftBodyDataPool,
			PxArray<PxU32>& newSoftBodyNodeIndexPool,
			PxArray<PxU32>& newSoftBodyElememtIndexPool,
			PxPinnedArray<PxgSoftBody>& softBodyPool,
			PxArray<PxgSoftBodyData>& softBodyDataPool,
			PxInt32ArrayPinned& softBodyElementIndexPool,
			PxArray<PxU32>& softBodyNodeIndexPool,
			PxgBodySimManager& bodySimManager,
			SoftBodyAttachmentAndFilterData& data);

		void gpuMemDmaUpFEMCloths(PxPinnedArray<PxgFEMCloth>& newFEMClothPool,
			PxU32* newTriangleMeshByteSizePool,
			PxArray<PxgFEMClothData>& newFEMClothDataPool,
			PxArray<PxU32>& newFEMClothNodeIndexPool,
			PxArray<PxU32>& newFEMClothElememtIndexPool,
			PxPinnedArray<PxgFEMCloth>& femClothPool,
			PxArray<PxgFEMClothData>& femClothDataPool,
			PxInt32ArrayPinned& femClothElementIndexPool,
			PxArray<PxU32>& femClothNodeIndexPool,
			PxgBodySimManager& bodySimManager,
			PxPinnedArray<PxgFEMRigidAttachment>& rigidAttachments,
			PxPinnedArray<PxgRigidFilterPair>& rigidAttachmentIds,
			bool dirtyRigidAttachments,
			PxInt32ArrayPinned& activeRigidAttachments,
			bool dirtyActiveRigidAttachments,
			PxPinnedArray<PxgFEMFEMAttachment>& clothAttachments,
			PxPinnedArray<PxgNonRigidFilterPair>& clothVertTriFilterIds,
			bool dirtyClothAttachments,
			PxInt32ArrayPinned& activeClothAttachments,
			bool dirtyActiveClothAttachments
		);

		void gpuMemDmaUpParticleSystem(PxgBodySimManager& bodySimManager);

		void mergeChangedAABBMgHandle();

		void gpuMemDmaUp(const PxU32 nbTotalBodies, const PxU32 nbTotalShapes,
			PxBitMapPinned& changedHandleMap, const bool enableDirectGPUAPI);
		void gpuMemDmaBack(PxInt32ArrayPinned& frozenArray,
			PxInt32ArrayPinned& unfrozenArray,
			PxInt32ArrayPinned& activateArray,
			PxInt32ArrayPinned& deactiveArray,
			PxCachedTransformArrayPinned* cachedTransform,
			const PxU32 cachedCapacity,
			Bp::BoundsArray& boundArray, PxBitMapPinned& changedAABBMgrHandles,
			const PxU32 numShapes, const PxU32 numActiveBodies, bool enableDirectGPUAPI);

		void syncDmaback(PxU32& nbFrozenShapesThisFrame, PxU32& nbUnfrozenShapesThisFrame, bool didSimulate);

		void updateBodies(const PxU32 nbUpdatedBodies, const PxU32 nbNewBodies);

		void updateArticulations(const PxU32 nbNewArticulations, PxgArticulationSimUpdate* updates,
			const PxU32 nbUpdatedArticulations, PxReal* dofData);

		void updateJointsAndSyncData(const PxPinnedArray<PxgD6JointData>& rigidJointData,
			const PxInt32ArrayPinned& dirtyRigidJointIndices,
			const PxPinnedArray<PxgD6JointData>& artiJointData,
			const PxInt32ArrayPinned& dirtyArtiJointIndices,
			const PxPinnedArray<PxgConstraintPrePrep>& rigidJointPrePrep,
			const PxPinnedArray<PxgConstraintPrePrep>& artiJointPrePrep, 
			const PxgJointManager::ConstraintIdMap& gpuConstraintIdMapHost,
			bool isGpuConstraintIdMapDirty,
			PxU32 nbTotalRigidJoints, PxU32 nbTotalArtiJoints);

		void update(bool enableDirectGPUAPI);

		void setBounds(Bp::BoundsArray* boundArray);

		PxgArticulationBuffer** getArticulationDataBuffer() { return mArticulationDataBuffer.begin(); }
		PxgTypedCudaBuffer<PxBounds3>*	getBoundArrayBuffer();

		void gpuDmaUpdateData();

		bool getRigidDynamicData(void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, float oneOverDt, CUevent startEvent, CUevent finishEvent) const;
		bool setRigidDynamicData(const void* data, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent);

		void setSoftBodyWakeCounter(const PxU32 remapId, const PxReal wakeCounter, const PxU32 numSoftBodies);
		void setFEMClothWakeCounter(const PxU32 remapId, const PxReal wakeCounter, const PxU32 numClothes);

		// PT: wrappers to make it easier to find the places where this is used.
		PX_FORCE_INLINE PxgDevicePointer<PxgBodySim>		getBodySimBufferDevicePtr()		const	{ return mBodySimCudaBuffer.getTypedDevicePtr();									}
		PX_FORCE_INLINE PxgDevicePointer<PxgBodySim>		getBodySimBufferDeviceData() { return mBodySimCudaBuffer.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxgDevicePointer<PxgBodySimVelocities>	getBodySimPrevVelocitiesBufferDevicePtr()	const	{ return mBodySimPreviousVelocitiesCudaBuffer.getTypedDevicePtr();	}
		PX_FORCE_INLINE PxgDevicePointer<PxgBodySimVelocities>	getBodySimPrevVelocitiesBufferDeviceData()			{ return mBodySimPreviousVelocitiesCudaBuffer.getTypedDevicePtr();	}

		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulation>&  getArticulationBuffer() { return mArticulationBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgSolverBodySleepData>&  getArticulationSleepDataBuffer() { return mArticulationSleepDataBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockData>&  getArticulationBatchData() { return mArticulationBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockLinkData>&  getArticulationBatchLinkData() { return mArticulationLinkBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationTraversalStackData>&  getArticulationTraversalStackData() { return mArticulationTraversalStackBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBitFieldStackData>&  getTempPathToRootBitFieldStackData() { return mTempPathToRootBitFieldStackBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBitFieldStackData>&  getTempSharedBitFieldStackData() { return mTempSharedBitFieldStackBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBitFieldStackData>&  getTempRootBitFieldStackData() { return mTempRootBitFieldStackBuffer;  }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBitFieldData>&  getPathToRootBitFieldStackData() { return mPathToRootBitFieldStackBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockDofData>&  getArticulationBatchDofData() { return mArticulationDofBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockMimicJointData>&  getArticulationBatchMimicJointData() { return mArticulationMimicJointBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockSpatialTendonData>&  getArticulationBatchSpatialTendonData() { return mArticulationSpatialTendonBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockAttachmentData>&  getArticulationBatchAttachmentData() { return mArticulationAttachmentBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationInternalTendonConstraintData>&  getArticulationBatchSpatialTendonConstraintData() { return mArticulationSpatialTendonConstraintsBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockFixedTendonData>&  getArticulationBatchFixedTendonData() { return mArticulationFixedTendonBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationBlockTendonJointData>&  getArticulationBatchTendonJointData() { return mArticulationTendonJointBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgArticulationInternalTendonConstraintData>&  getArticulationBatchFixedTendonConstraintData() { return mArticulationFixedTendonConstraintsBatchBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgSoftBody>&  getSoftBodyBuffer() { return mSoftBodyBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>&  getActiveSoftBodyBuffer() { return mActiveSoftBodyBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>&  getActiveSelfCollisionSoftBodyBuffer() { return mActiveSelfCollisionSoftBodyBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>&  getSoftBodyElementIndexBuffer() { return mSoftBodyElementIndexBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFEMCloth>&  getFEMClothBuffer() { return mFEMClothBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>&  getActiveFEMClothBuffer() { return mActiveFEMClothBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>&  getFEMClothElementIndexBuffer() { return mFEMClothElementIndexBuffer; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgD6JointData>&  getD6RigidJointBuffer() { return mRigidJointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgConstraintPrePrep>&  getD6RigidJointPrePreBuffer() { return mRigidJointPrePrepBuffer; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgD6JointData>&  getD6ArtiJointBuffer() { return mArtiJointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgConstraintPrePrep>&  getD6ArtiJointPrePreBuffer() { return mArtiJointPrePrepBuffer; }	
		PX_FORCE_INLINE CUstream getStream() { return mStream; }

		bool getD6JointData(void* data, const PxD6JointGPUIndex* gpuIndices, PxD6JointGPUAPIReadType::Enum dataType, PxU32 nbElements, PxF32 oneOverDt, 
			PxU32 directGpuApiIndexMapHostSize, CUevent startEvent, CUevent finishEvent) const;
			

		//soft body
		PX_FORCE_INLINE PxU32 getMaxTetraVerts() { return mMaxTetraVerts; }
		PX_FORCE_INLINE PxU32 getMaxTetrahedrons() { return mMaxTetrahedrons; }

		PX_FORCE_INLINE PxU32 getGMMaxTetraPartitions() { return mGMMaxPartitions; }
		PX_FORCE_INLINE PxU32 getGMMaxTetraVerts() { return mGMMaxTetraVerts; }
		PX_FORCE_INLINE PxU32 getGMMaxTetrahedrons() { return mGMMaxTetrahedrons; }
		PX_FORCE_INLINE PxU32 getGMMaxTetrahedronsPerPartition() { return mGMMaxTetrahedronsPerPartition; }
		PX_FORCE_INLINE PxU32 getGMMaxJacobiTetrahedrons() { return mGMMaxJacobiTetrahedrons; }
		PX_FORCE_INLINE PxU32 getGMMaxJacobiVertices() { return mGMMaxJacobiVertices; }
		PX_FORCE_INLINE bool getGMUsePartitionAveraging() { return mUsePartitionAveraging; }

		// FEM-cloth
		PX_FORCE_INLINE PxU32 getMaxClothVerts() { return mMaxNbClothVerts; }

		PX_FORCE_INLINE PxU32 getMaxNonSharedTrianglePartitions() { return mMaxNbNonSharedTriPartitions; }
		PX_FORCE_INLINE PxU32 getMaxNonSharedTrianglesPerPartition() { return mMaxNbNonSharedTrianglesPerPartition; }

		PX_FORCE_INLINE PxU32 getMaxNbNonSharedTriangles() { return mMaxNbNonSharedTriangles; }

		PX_FORCE_INLINE PxU32 getMaxSharedTrianglePairPartitions() { return mMaxNbSharedTriPairPartitions; }
		PX_FORCE_INLINE PxU32 getMaxNonSharedTrianglePairPartitions() { return mMaxNbNonSharedTriPairPartitions; }

		PX_FORCE_INLINE PxU32 getMaxNonSharedTriangleClusterId() { return mMaxNonSharedTriClusterId; }
		PX_FORCE_INLINE PxU32 getMaxSharedTrianglePairClusterId() { return mMaxSharedTriPairClusterId; }
		PX_FORCE_INLINE PxU32 getMaxNonSharedTrianglePairClusterId() { return mMaxNonSharedTriPairClusterId; }

		PX_FORCE_INLINE PxU32 getMaxSharedTrianglePairsPerPartition() { return mMaxNbSharedTrianglePairsPerPartition; }
		PX_FORCE_INLINE PxU32 getMaxNonSharedTrianglePairsPerPartition() { return mMaxNbNonSharedTrianglePairsPerPartition; }

		PX_FORCE_INLINE PxU32 getMaxNbSharedTrianglePairs() { return mMaxNbSharedTrianglePairs; }
		PX_FORCE_INLINE PxU32 getMaxNbNonSharedTrianglePairs() { return mMaxNbNonSharedTrianglePairs; }

		PX_FORCE_INLINE bool hasActiveBendingPairs() { return mHasActiveBendingPairs; }

		PX_FORCE_INLINE PxU32 getMaxNbCollisionPairUpdatesPerTimestep() { return mMaxNbCollisionPairUpdatesPerTimestep; }
		PX_FORCE_INLINE PxU32 getMaxNbCollisionSubsteps() { return mMaxNbCollisionSubsteps; }
		PX_FORCE_INLINE PxU32 getNumTotalFEMCloths() { return mNbTotalFEMCloths; }

		PX_FORCE_INLINE PxU32 getNumTotalShapes() { return mPxgShapeSimManager.getNbTotalShapeSims();  }
		PX_FORCE_INLINE PxU32 getNumTotalSoftbodies() { return mNbTotalSoftBodies; }

		PX_FORCE_INLINE PxU32 getNbRigidSoftBodyAttachments() const { return mNbRigidSoftBodyAttachments; }
		PX_FORCE_INLINE PxU32 getNbRigidSoftBodyFilters() const { return mNbRigidSoftBodyFilters; }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMRigidAttachment> getRigidSoftBodyAttachments() { return mSoftBodyRigidAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getActiveRigidSoftBodyAttachments() { return mActiveSoftBodyRigidConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMRigidAttachmentConstraint> getSoftBodyRigidConstraints() { return mSoftBodyRigidConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getGpuSoftBodyRigidCounter() { return mNumSoftBodyRigidAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxNodeIndex> getSoftBodyRigidAttachmentIds() { return mSoftBodyRigidAttachmentIds.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgRigidFilterPair> getRigidSoftBodyFilters() { return mSoftBodyRigidFilterPairs.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxU32 getNbSoftBodySoftBodyAttachments() const { return mNbSoftBodySoftBodyAttachments; }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachment> getSoftBodySoftBodyAttachments() { return mSoftBodySoftBodyAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getActiveSoftBodySoftAttachments() { return mActiveSoftBodySoftBodyConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachmentConstraint> getSoftBodySoftBodyConstraints() { return mSoftBodySoftBodyConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getGpuSoftBodySoftBodyCounter() { return mNumSoftBodySoftBodyAttachments.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxU32 getNbClothSoftBodyAttachments() const { return mNbClothSoftBodyAttachments; }
		PX_FORCE_INLINE PxU32 getNbClothSoftBodyFilters() const { return mNbClothSoftBodyFilters; }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachment> getClothSoftBodyAttachments() { return mSoftBodyClothAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getActiveClothSoftBodyAttachments() { return mActiveSoftBodyClothConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachmentConstraint> getSoftBodyClothConstraints() { return mSoftBodyClothConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getGpuSoftBodyClothCounter() { return mNumSoftBodyClothAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgNonRigidFilterPair> getClothSoftBodyFilters() { return mSoftBodyClothFilterPairs.getTypedDevicePtr(); }


		PX_FORCE_INLINE PxU32 getNbSoftBodyParticleAttachments() const { return mNbSoftBodyParticleAttachments; }
		PX_FORCE_INLINE PxU32 getNbSoftBodyParticleFilters() const { return mNbSoftBodyParticleFilters; }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachment> getSoftBodyParticleAttachments() { return mSoftBodyParticleAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getActiveSoftBodyParticleAttachments() { return mActiveSoftBodyParticleConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachmentConstraint> getSoftBodyParticleConstraints() { return mSoftBodyParticleConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgNonRigidFilterPair> getSoftBodyParticleFilters() { return mSoftBodyParticleFilterPairs.getTypedDevicePtr(); }


		PX_FORCE_INLINE PxU32 getNbActiveRigidClothAttachments() const { return mNbRigidClothAttachments; }
		PX_FORCE_INLINE PxU32 getNbRigidClothFilters() const { return mNbRigidClothFilters; }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMRigidAttachment> getRigidClothAttachments() { return mClothRigidAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getActiveRigidClothAttachments() { return mActiveClothRigidAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMRigidAttachmentConstraint> getClothRigidConstraints() { return mClothRigidConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getGpuClothRigidCounter() { return mNumClothRigidAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxNodeIndex> getClothRigidAttachmentIds() { return mClothRigidAttachmentIds.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgRigidFilterPair> getRigidClothFilters() { return mClothRigidFilterPairs.getTypedDevicePtr(); }


		PX_FORCE_INLINE PxU32 getNbActiveClothClothAttachments() const { return mNbClothClothAttachments; }
		PX_FORCE_INLINE PxU32 getNbClothClothVertTriFilters() const { return mNbClothClothVertTriFilters; }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachment> getClothClothAttachments() { return mClothClothAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getActiveClothClothAttachments() { return mActiveClothClothAttachments.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxgFEMFEMAttachmentConstraint> getClothClothConstraints() { return mClothClothConstraints.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxgDevicePointer<PxU32> getGpuClothClothCounter() { return mNumClothClothAttachments.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxgDevicePointer<PxgNonRigidFilterPair> getClothClothVertTriFilters() { return mClothClothVertTriFilterPairs.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxBitMapPinned& getActiveClothStateChangedMap() { return mActiveFEMClothStateChangedMap; }
		PX_FORCE_INLINE PxReal* getActiveClothWakeCountsCPU() { return mFEMClothWakeCounts.begin(); }
		PX_FORCE_INLINE PxgDevicePointer<PxReal> getActiveClothWakeCountsGPU() { return mFEMClothWakeCountsGPU.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxBitMapPinned& getActiveSBStateChangedMap() { return mActiveSBStateChangedMap; }
		PX_FORCE_INLINE const PxReal* getActiveSBWakeCountsCPU() const { return mSBWakeCounts.begin(); }
		PX_FORCE_INLINE const PxgDevicePointer<PxReal> getActiveSBWakeCountsGPU() const { return mSBWakeCountsGPU.getTypedDevicePtr(); }
		PX_FORCE_INLINE PxReal* getActiveSBWakeCountsCPU() { return mSBWakeCounts.begin(); }
		PX_FORCE_INLINE PxgDevicePointer<PxReal> getActiveSBWakeCountsGPU() { return mSBWakeCountsGPU.getTypedDevicePtr(); }

		PX_FORCE_INLINE PxgDevicePointer<PxgSimulationCoreDesc> getSimulationCoreDesc() { return mUpdatedCacheAndBoundsDescBuffer.getTypedDevicePtr(); }

		PxU32			getMaxArticulationLinks() const;
		PxU32			getMaxArticulationDofs() const;
		PxU32			getMaxArticulationMimicJoints() const;
		PxU32			getMaxArticuationSpatialTendons() const;
		PxU32			getMaxArticuationAttachments() const;
		PxU32			getMaxArticuationFixedTendons() const;
		PxU32			getMaxArticuationTendonJoints() const;

		PxgGpuContext*							mGpuContext;
		PxgCudaKernelWranglerManager*			mGpuKernelWranglerManager;
		PxCudaContextManager*					mCudaContextManager;
		PxCudaContext*							mCudaContext;
		PxgHeapMemoryAllocatorManager*			mHeapMemoryManager;
		Bp::BoundsArray*						mBoundArray;
		bool									mUseGpuBp;

	private:

		void constructDescriptor(CUdeviceptr boundsd, CUdeviceptr changedAABBMgrHandlesd, const PxU32 nbTotalShapes, const PxU32 bitMapWordCounts);
		void createGpuStreamsAndEvents();
		void releaseGpuStreamsAndEvents();
		void syncData();
		
		PxgSimulationCoreDesc*			mUpdatedCacheAndBoundsDesc;
		PxgNewBodiesDesc*				mNewBodiesDesc;
		PxgUpdateArticulationDesc*		mUpdateArticulationDesc;
		PxgUpdatedBodiesDesc*			mUpdatedBodiesDesc;
		PxgUpdatedJointsDesc*			mUpdatedJointsDesc;
		PxgUpdateActorDataDesc*			mUpdatedActorDataDesc;

		PxgTypedCudaBuffer<PxU32>	mFrozenBuffer;
		PxgTypedCudaBuffer<PxU32>	mUnfrozenBuffer;
		PxgTypedCudaBuffer<PxU32>	mFrozenBlockAndResBuffer;
		PxgTypedCudaBuffer<PxU32>	mUnfrozenBlockAndResBuffer;
		PxgTypedCudaBuffer<PxU32>	mUpdatedBuffer;
		PxgTypedCudaBuffer<PxU32>	mActivateBuffer;
		PxgTypedCudaBuffer<PxU32>	mDeactivateBuffer;

		PxgTypedCudaBuffer<PxU32>	mUpdatedDirectBuffer;

		// PT: new naming convention with "CudaBuffer" suffix and specific prefix for easier searching
		PxgTypedCudaBuffer<PxgBodySim>	mBodySimCudaBuffer;						// PT: contains PxgBodySim structs.
		PxgTypedCudaBuffer<PxgBodySimVelocities>	mBodySimPreviousVelocitiesCudaBuffer;	// PT: contains PxgBodySimVelocities structs. Only for direct GPU acceleration getters.

		public:
		PxgShapeSimManager	mPxgShapeSimManager;
		private:
		PxgTypedCudaBuffer<PxgArticulation>	mArticulationBuffer; //persistent buffer for articulation
		PxgTypedCudaBuffer<PxgSolverBodySleepData>	mArticulationSleepDataBuffer; //persistent buffer for sleepData
		PxgTypedCudaBuffer<PxgArticulationBlockData>	mArticulationBatchBuffer;
		PxgTypedCudaBuffer<PxgArticulationBlockLinkData>	mArticulationLinkBatchBuffer;
		PxgTypedCudaBuffer<PxgArticulationTraversalStackData>	mArticulationTraversalStackBuffer;
		PxgTypedCudaBuffer<PxgArticulationBitFieldStackData>	mTempPathToRootBitFieldStackBuffer;
		PxgTypedCudaBuffer<PxgArticulationBitFieldStackData>	mTempSharedBitFieldStackBuffer;
		PxgTypedCudaBuffer<PxgArticulationBitFieldStackData>	mTempRootBitFieldStackBuffer;
		PxgTypedCudaBuffer<PxgArticulationBitFieldData>	mPathToRootBitFieldStackBuffer;
		PxgTypedCudaBuffer<PxgArticulationBlockDofData>	mArticulationDofBatchBuffer;

		PxgTypedCudaBuffer<PxgArticulationBlockMimicJointData>	mArticulationMimicJointBatchBuffer;

		PxgTypedCudaBuffer<PxgArticulationBlockSpatialTendonData>	mArticulationSpatialTendonBatchBuffer;
		PxgTypedCudaBuffer<PxgArticulationInternalTendonConstraintData>	mArticulationSpatialTendonConstraintsBatchBuffer;
		PxgTypedCudaBuffer<PxgArticulationBlockAttachmentData>	mArticulationAttachmentBatchBuffer;

		PxgTypedCudaBuffer<PxgArticulationBlockFixedTendonData>	mArticulationFixedTendonBatchBuffer;
		PxgTypedCudaBuffer<PxgArticulationInternalTendonConstraintData>	mArticulationFixedTendonConstraintsBatchBuffer;
		PxgTypedCudaBuffer<PxgArticulationBlockTendonJointData>	mArticulationTendonJointBatchBuffer;

		PxArray<PxgArticulationBuffer*> mArticulationDataBuffer;// persistent data, map with mArticulationBuffer
		//PxU32			mMaxLinks;
		//PxU32			mMaxDofs;

		PxgTypedCudaBuffer<PxgSoftBody>	mSoftBodyBuffer; //persistent buffer for soft bodies
		PxgTypedCudaBuffer<PxU32>	mActiveSoftBodyBuffer;
		PxgTypedCudaBuffer<PxU32>	mActiveSelfCollisionSoftBodyBuffer;
		PxgTypedCudaBuffer<PxU32>	mSoftBodyElementIndexBuffer;
		PxgTypedCudaBuffer<PxgFEMCloth>	mFEMClothBuffer; // persistent buffer for FEM-cloth
		PxgTypedCudaBuffer<PxU32>	mActiveFEMClothBuffer;
		PxgTypedCudaBuffer<PxU32>	mFEMClothElementIndexBuffer;
		
		PxArray<PxgSoftBodyBuffer*> mSoftBodyDataBuffer; //persistent data, map with mSoftBodyBuffer
		PxArray<PxgFEMClothBuffer*> mFEMClothDataBuffer; //persistent data, map with mFEMClothBuffer
		
		PxBitMapPinned	mActiveFEMClothStateChangedMap;
		PxFloatArrayPinned mFEMClothWakeCounts;
		PxgTypedCudaBuffer<PxReal>	mFEMClothWakeCountsGPU;

		PxBitMapPinned	mActiveSBStateChangedMap;
		PxFloatArrayPinned mSBWakeCounts;
		PxgTypedCudaBuffer<PxReal>	mSBWakeCountsGPU;

		PxgTypedCudaBuffer<PxgD6JointData>	mRigidJointBuffer;
		PxgTypedCudaBuffer<PxgD6JointData>	mArtiJointBuffer;

		PxgTypedCudaBuffer<PxgConstraintPrePrep>	mRigidJointPrePrepBuffer;
		PxgTypedCudaBuffer<PxgConstraintPrePrep>	mArtiJointPrePrepBuffer;

		PxgTypedCudaBuffer<PxgConstraintIdMapEntry> mGpuConstraintIdMapDevice;
		// See PxgJointManager::mGpuConstraintIdMapHost. This is just the device buffer counterpart.

		PxgTypedCudaBuffer<PxgBodySimVelocityUpdate>	mUpdatedBodySimBuffer;
		PxgTypedCudaBuffer<PxgBodySim>	mNewBodySimBuffer;
		PxgTypedCudaBuffer<PxgArticulation>	mNewArticulationBuffer;
		PxgTypedCudaBuffer<PxgArticulationLink>	mNewLinkBuffer;
		PxgTypedCudaBuffer<PxReal>	mNewLinkWakeCounterBuffer;
		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector>	mNewLinkExtAccelBuffer;
		PxgTypedCudaBuffer<PxgArticulationLinkProp>	mNewLinkPropBuffer;
		PxgTypedCudaBuffer<PxU32>	mNewLinkParentBuffer;
		PxgTypedCudaBuffer<ArticulationBitField>	mNewLinkChildBuffer;
		PxgTypedCudaBuffer<PxTransform>   mNewLinkBody2WorldsBuffer;
		PxgTypedCudaBuffer<PxTransform>	mNewLinkBody2ActorsBuffer;
		PxgTypedCudaBuffer<Dy::ArticulationJointCore>	mNewJointCoreBuffer;
		PxgTypedCudaBuffer<Dy::ArticulationJointCoreData>	mNewJointDataBuffer;
		PxgTypedCudaBuffer<PxgArticulationSimUpdate>	mNewLinkIndexBuffer;
		PxgTypedCudaBuffer<PxGpuSpatialTendonData>	mNewSpatialTendonParamsBuffer;
		PxgTypedCudaBuffer<PxgArticulationTendon>	mNewSpatialTendonsBuffer;
		PxgTypedCudaBuffer<Dy::ArticulationMimicJointCore>	mNewMimicJointBuffer;
		PxgTypedCudaBuffer<PxU32>	mNewPathToRootBuffer;
		PxgTypedCudaBuffer<PxgArticulationTendonElementFixedData>	mNewAttachmentFixedBuffer;
		PxgTypedCudaBuffer<PxGpuTendonAttachmentData>	mNewAttachmentModBuffer;
		PxgTypedCudaBuffer<PxU32>	mNewTendonAttachmentRemapBuffer;
		PxgTypedCudaBuffer<PxGpuFixedTendonData>	mNewFixedTendonParamsBuffer;
		PxgTypedCudaBuffer<PxgArticulationTendon>	mNewFixedTendonsBuffer;
		PxgTypedCudaBuffer<PxgArticulationTendonElementFixedData>	mNewTendonJointsFixedBuffer;
		PxgTypedCudaBuffer<PxGpuTendonJointCoefficientData>	mNewTendonJointsCoefficientBuffer;
		PxgTypedCudaBuffer<PxU32>	mNewTendonTendonJointRemapBuffer;

		PxgTypedCudaBuffer<PxU32>	mActiveNodeIndices;	//this is index mapping between solver body data index and the node index

		PxgTypedCudaBuffer<PxgSimulationCoreDesc>	mUpdatedCacheAndBoundsDescBuffer;
		PxgTypedCudaBuffer<PxgNewBodiesDesc>	mBodiesDescBuffer;
		PxgTypedCudaBuffer<PxgUpdateArticulationDesc>	mArticulationDescBuffer;
		PxgTypedCudaBuffer<PxgUpdatedBodiesDesc>	mUpdatedBodiesDescBuffer;
		PxgTypedCudaBuffer<PxgUpdatedJointsDesc>	mUpdatedJointDescBuffer;
		PxgTypedCudaBuffer<PxgUpdateActorDataDesc>	mUpdatedActorDescBuffer;

		PxgTypedCudaBuffer<PxBounds3>	mBoundsBuffer;					//Bp in CPU so we can't use the BoundsBuffer in the GPU BP
		PxgTypedCudaBuffer<PxU32>   mChangedAABBMgrHandlesBuffer;  //Bp in CPU so we can't use the changedAABBMgrHandlesBuffer in the GPU AABBManager

		PxgTypedCudaBuffer<PxgFEMRigidAttachment>	mSoftBodyRigidAttachments;
		PxgTypedCudaBuffer<PxgRigidFilterPair>	mSoftBodyRigidFilterPairs;
		PxgTypedCudaBuffer<PxgFEMRigidAttachmentConstraint>	mSoftBodyRigidConstraints;
		PxgTypedCudaBuffer<PxU32>	mActiveSoftBodyRigidConstraints;
		PxgTypedCudaBuffer<PxU32>	mNumSoftBodyRigidAttachments;
		PxgTypedCudaBuffer<PxNodeIndex>	mSoftBodyRigidAttachmentIds;


		PxgTypedCudaBuffer<PxgFEMFEMAttachment>	mSoftBodySoftBodyAttachments;
		PxgTypedCudaBuffer<PxgFEMFEMAttachmentConstraint>	mSoftBodySoftBodyConstraints;
		PxgTypedCudaBuffer<PxU32>	mActiveSoftBodySoftBodyConstraints;
		PxgTypedCudaBuffer<PxU32>	mNumSoftBodySoftBodyAttachments;

		PxgTypedCudaBuffer<PxgFEMFEMAttachment>	mSoftBodyClothAttachments;
		PxgTypedCudaBuffer<PxgNonRigidFilterPair>	mSoftBodyClothFilterPairs;
		PxgTypedCudaBuffer<PxgFEMFEMAttachmentConstraint>	mSoftBodyClothConstraints;
		PxgTypedCudaBuffer<PxU32>	mActiveSoftBodyClothConstraints;
		PxgTypedCudaBuffer<PxU32>	mNumSoftBodyClothAttachments;

		PxgTypedCudaBuffer<PxgFEMFEMAttachment>	mSoftBodyParticleAttachments;
		PxgTypedCudaBuffer<PxgNonRigidFilterPair>	mSoftBodyParticleFilterPairs;
		PxgTypedCudaBuffer<PxgFEMFEMAttachmentConstraint>	mSoftBodyParticleConstraints;
		PxgTypedCudaBuffer<PxU32>	mActiveSoftBodyParticleConstraints;

		PxgTypedCudaBuffer<PxgFEMRigidAttachment>	mClothRigidAttachments;
		PxgTypedCudaBuffer<PxU32>	mActiveClothRigidAttachments;
		PxgTypedCudaBuffer<PxgRigidFilterPair>	mClothRigidFilterPairs;
		PxgTypedCudaBuffer<PxgFEMRigidAttachmentConstraint>	mClothRigidConstraints;
		PxgTypedCudaBuffer<PxU32>	mNumClothRigidAttachments;
		PxgTypedCudaBuffer<PxNodeIndex>	mClothRigidAttachmentIds;

		PxgTypedCudaBuffer<PxgFEMFEMAttachment>	mClothClothAttachments;
		PxgTypedCudaBuffer<PxU32>	mActiveClothClothAttachments;
		PxgTypedCudaBuffer<PxgNonRigidFilterPair>	mClothClothVertTriFilterPairs;
		PxgTypedCudaBuffer<PxgFEMFEMAttachmentConstraint>	mClothClothConstraints;
		PxgTypedCudaBuffer<PxU32>	mNumClothClothAttachments;

		PxU32*			mPinnedEvent;

		PxU32			mNbRigidSoftBodyAttachments;
		PxU32			mNbRigidSoftBodyFilters;

		PxU32			mNbSoftBodySoftBodyAttachments;

		PxU32			mNbClothSoftBodyAttachments;
		PxU32			mNbClothSoftBodyFilters;

		PxU32			mNbSoftBodyParticleAttachments;
		PxU32			mNbSoftBodyParticleFilters;

		PxU32			mNbRigidClothAttachments;
		PxU32			mNbRigidClothFilters;

		PxU32			mNbClothClothAttachments;
		PxU32			mNbClothClothVertTriFilters;

		PxU32			mNbTotalBodySim;
		PxU32			mNbTotalArticulations; //this is used for articulation
		PxU32			mNbTotalSoftBodies;
		PxU32			mNbTotalFEMCloths;

		PxU32			mMaxTetraVerts; //max number of verts for all the tetrahedron mesh
		PxU32			mMaxTetrahedrons;

		PxU32			mGMMaxPartitions;
		PxU32			mGMMaxTetraVerts;
		PxU32			mGMMaxTetrahedrons;
		PxU32			mGMMaxTetrahedronsPerPartition;
		PxU32			mGMMaxJacobiTetrahedrons;
		PxU32			mGMMaxJacobiVertices;

		PxU32			mMaxNbClothVerts;

		PxU32			mMaxNbNonSharedTriangles;

		PxU32			mMaxNbNonSharedTriPartitions;
		PxU32			mMaxNbNonSharedTrianglesPerPartition;

		PxU32			mMaxNbSharedTrianglePairs;
		PxU32			mMaxNbNonSharedTrianglePairs;

		PxU32			mMaxNbSharedTriPairPartitions;
		PxU32			mMaxNbNonSharedTriPairPartitions;

		PxU32			mMaxNonSharedTriClusterId;
		PxU32			mMaxSharedTriPairClusterId;
		PxU32			mMaxNonSharedTriPairClusterId;

		PxU32			mMaxNbSharedTrianglePairsPerPartition;
		PxU32			mMaxNbNonSharedTrianglePairsPerPartition;
		PxU32			mMaxNbCollisionPairUpdatesPerTimestep;
		PxU32			mMaxNbCollisionSubsteps;

		//PxU32			mMaxParticles;
		PxU32			mNbTotalRigidJoints;
		PxU32			mNbTotalArtiJoints;

		bool			mUsePartitionAveraging;
		bool			mHasActiveBendingPairs;

		CUstream		mStream;
		CUevent			mEvent;
		CUevent			mDmaEvent; 
	
		PxVec3			mGravity;

		PxArray<PxgSoftBody> mSoftBodiesToFree;
		PxArray<PxgFEMCloth> mClothsToFree;
#if PX_SUPPORT_OMNI_PVD
		PxU64 getRigidBodyDataTypeElementSize(PxRigidDynamicGPUAPIWriteType::Enum dataType);
		void ovdRigidBodyCallback(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements);		
		
		PxPinnedArray<PxU8>  mOvdDataBuffer;
		PxPinnedArray<PxU8>  mOvdIndexBuffer;
#endif
	};
}

#endif
