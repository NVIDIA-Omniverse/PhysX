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

#ifndef PXG_ARTICULATION_CORE_H
#define PXG_ARTICULATION_CORE_H

#include "PxDirectGPUAPI.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPinnedArray.h"
#include "foundation/PxUserAllocated.h"
#include "PxgCudaBuffer.h"
#if !PX_CUDA_COMPILER
#include <vector_types.h>
#endif

#include "DyFeatherstoneArticulation.h"
#include "PxgArticulation.h"

namespace physx
{
	//this is needed to force PhysXArticulationGpu linkage as Static Library!
	void createPxgArticulation();

	class PxgCudaKernelWranglerManager;
	class PxCudaContextManager;
	class PxCudaContext;
	class PxgHeapMemoryAllocatorManager;

	class PxgGpuContext;

	struct PxgSolverReferences;
	struct PxgSolverBodySleepData;

	struct PxgArticulationCoreDesc;
	struct PxgArticulationOutputDesc;

	struct PxIndexDataPair;
	class PxSceneDesc;

	class PxgArticulationCore : public PxUserAllocated
	{
	public:
		PxgArticulationCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager, PxgHeapMemoryAllocatorManager* heapMemoryManager);
		~PxgArticulationCore();

		void gpuMemDmaUpArticulationDesc(const PxU32 offset, const PxU32 nbArticulations, PxReal dt, const PxVec3& gravity, const PxReal invLengthScale, const bool isExternalForcesEveryTgsIterationEnabled);
		
		void createStaticContactAndConstraintsBatch(const PxU32 nbArticulations);

		PxU32 computeUnconstrainedVelocities(const PxU32 offset, const PxU32 nbArticulations, PxReal dt, const PxVec3& gravity, const PxReal invLengthScale, const bool isExternalForcesEveryTgsIterationEnabled, bool recomputeBlockFormat);
		PxU32 setupInternalConstraints(const PxU32 nbArticulations, const PxReal stepDt, const PxReal dt, const PxReal invDt, const bool isTGSSolver);
		void syncStream();

		void precomputeDependencies(const PxU32 nbPartitions);

		void syncUnconstrainedVelocities();

		void propagateRigidBodyImpulsesAndSolveInternalConstraints(const PxReal dt, const PxReal invDt, const bool velocityIteration, const PxReal elapsedTime,
			const PxReal biasCoefficient, PxU32* staticContactUniqueIndices, PxU32* staticJointUniqueIndices, 
			CUdeviceptr sharedDesc, bool doFriction, bool isTGS, bool residualReportingEnabled, bool isExternalForcesEveryTgsIterationEnabled = false);

		//These two methods is for articulation vs soft body interation
		void outputVelocity(CUdeviceptr sharedDesc, CUstream solverStream, bool isTGS);
		void pushImpulse(CUstream solverStream);

		void stepArticulation(const PxReal stepDt);
		void averageDeltaV(const PxU32 nbSlabs, CUstream stream, float4* velocities, const PxU32 partitionId,
			bool isTGS, CUdeviceptr sharedDescd);
		void applyTgsSubstepForces(PxReal stepDt, CUstream stream);
		void saveVelocities();
		void updateBodies(PxReal dt, bool integrate, bool enableDirectGPUAPI);
		void gpuMemDMAbackArticulation(PxInt8ArrayPinned& linkAndJointAndRootStateData,
			PxPinnedArray<PxgSolverBodySleepData>& wakeCounterPool, PxPinnedArray<Dy::ErrorAccumulator>& internalResidualPerArticulation, PxPinnedArray<Dy::ErrorAccumulator>& contactResidual);

		void setSolverStream(CUstream& solverStream) { mSolverStream = &solverStream; }
		
		void setGpuContext(PxgGpuContext* context) { mGpuContext = context; }

		PxgArticulationCoreDesc* getArticulationCoreDesc() { return mArticulationCoreDesc; }

		CUdeviceptr getArticulationCoreDescd() { return mArticulationCoreDescd.getDevicePtr(); }

		CUdeviceptr getDeferredZ() { return mDeltaVs.getDevicePtr(); }

		CUdeviceptr getArticulationDirty() { return mSlabHasChanges.getDevicePtr(); }

		CUdeviceptr getArticulationSlabMask() { return mSlabDirtyMasks.getDevicePtr(); }

		PxU32 getNbActiveArticulations() const { return mNbActiveArticulation; }

		CUstream getStream() { return mStream; }

		CUevent getFlushArticulationDataEvent() { return mFlushArticulationDataEvent; }

		void synchronizedStreams(CUstream bpStream, CUstream npStream);

		bool getArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent, const PxU32 maxLinks, const PxU32 maxDofs) const;
		bool setArticulationData(const void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent, PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialAttachments);
		bool computeArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements,
									PxU32 maxLinks, PxU32 maxDofs, CUevent startEvent, CUevent finishEvent);

		// needed if root transforms or joint positions are updated using direct-GPU API. needs to be called before computeUnconstrainedVelocities.
		void updateArticulationsKinematic(bool zeroSimOutput, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices=NULL, PxU32 nbElements=0);

		void allocDeltaVBuffer(PxU32 nbSlabs, PxU32 nbPartitions, CUstream stream);

		void layoutDeltaVBuffer(const PxU32 nbSlabs, const PxU32 nbPartitions, CUstream stream);

	private:
		// new Direct-GPU API methods
		bool getDofStates(void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxDofs, PxArticulationGPUAPIReadType::Enum dataType) const;
		bool getTransformStates(void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxLinks, PxArticulationGPUAPIReadType::Enum dataType) const;
		bool getLinkVelocityStates(void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxLinks, PxArticulationGPUAPIReadType::Enum dataType) const;
		bool getLinkSpatialForceStates(void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxLinks, PxArticulationGPUAPIReadType::Enum dataType) const;

		bool setDofStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxDofs, PxArticulationGPUAPIWriteType::Enum dataType);
		bool setRootGlobalPoseStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements);
		bool setRootVelocityStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxArticulationGPUAPIWriteType::Enum dataType);
		bool setLinkForceStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxLinks);
		bool setLinkTorqueStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxLinks);
		bool setTendonStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxTendons, PxArticulationGPUAPIWriteType::Enum dataType);
		bool setSpatialTendonAttachmentStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxTendonsXmaxAttachments);
		bool setFixedTendonJointStates(const void* data, const PxArticulationGPUIndex* gpuIndices, PxU32 nbElements, PxU32 maxFixedTendonsXmaxTendonJoints);
		
		PxgArticulationCoreDesc*			mArticulationCoreDesc;
		PxgArticulationOutputDesc*			mArticulationOutputDesc;

		PxgCudaKernelWranglerManager*		mGpuKernelWranglerManager;
		PxCudaContextManager*				mCudaContextManager;
		PxCudaContext*						mCudaContext;
		CUstream							mStream;
		CUstream*							mSolverStream; 
		CUevent								mFinishEvent;
		CUevent								mFlushArticulationDataEvent;

		PxgGpuContext*						mGpuContext;

		PxgTypedCudaBuffer<PxgArticulationCoreDesc>	mArticulationCoreDescd;
		PxgTypedCudaBuffer<PxgArticulationOutputDesc> mArticulationOutputDescd;
		PxU32								mNbActiveArticulation;

		PxgTypedCudaBuffer<Cm::UnAlignedSpatialVector> mDeltaVs;
		PxgTypedCudaBuffer<uint2>			mSlabHasChanges;
		PxgTypedCudaBuffer<uint4>			mSlabDirtyMasks;

		PxgTypedCudaBuffer<PxgArticulationBitFieldStackData> mPathToRootPerPartition;
		PxgTypedCudaBuffer<PxU32>			mDirtyLinksPerPartition;
		PxgTypedCudaBuffer<PxReal>			mImpulseScalePerPartition;

		PxgTypedCudaBuffer<PxU32>			mTempContactUniqueIndicesBlockBuffer;
		PxgTypedCudaBuffer<PxU32>			mTempConstraintUniqueIndicesBlockBuffer;
		PxgTypedCudaBuffer<PxU32>			mTempContactHeaderBlockBuffer;
		PxgTypedCudaBuffer<PxU32>			mTempConstraintHeaderBlockBuffer;

		PxgTypedCudaBuffer<PxU32>			mTempSelfContactUniqueIndicesBlockBuffer;
		PxgTypedCudaBuffer<PxU32>			mTempSelfConstraintUniqueIndicesBlockBuffer;
		PxgTypedCudaBuffer<PxU32>			mTempSelfContactHeaderBlockBuffer;
		PxgTypedCudaBuffer<PxU32>			mTempSelfConstraintHeaderBlockBuffer;

		CUevent								mComputeUnconstrainedEvent;

		bool 								mNeedsKinematicUpdate;

#if PX_SUPPORT_OMNI_PVD
		void ovdArticulationCallback(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
			PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
			PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments);				
		PxPinnedArray<PxU8> mOvdDataBuffer;
		PxPinnedArray<PxU8> mOvdIndexBuffer;
#endif
	};
}

#endif
