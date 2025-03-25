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

#ifndef PXG_PARTICLE_SYSTEM_CORE_H
#define PXG_PARTICLE_SYSTEM_CORE_H

#include "foundation/PxArray.h"
#include "foundation/PxPinnedArray.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

#include "PxSparseGridParams.h"

#include "cudamanager/PxCudaTypes.h"

#include "PxgCudaBuffer.h"
#include "PxgNarrowphaseCore.h"
#include "PxgNonRigidCoreCommon.h"
#include "PxgParticleSystem.h"
#include "PxgRadixSortDesc.h"
#include "PxgSimulationCoreDesc.h"

#include <vector_types.h>

namespace physx
{
	//this is needed to force PhysXSimulationControllerGpu linkage as Static Library!
	void createPxgParticleSystem();

	class PxgBodySimManager;
	class PxgEssentialCore;
	class PxgCudaKernelWranglerManager;
	class PxCudaContextManager;
	class PxgHeapMemoryAllocatorManager;
	class PxgSimulationController;
	class PxgGpuContext;
	class PxBounds3;

	struct PxgShapeDescBuffer;

	struct PxgParticlePrimitiveConstraintBlock
	{
		float4 raXn_velMultiplierW[32];
		float4 normal_errorW[32];
		//Friction tangent + invMass of the rigid body (avoids needing to read the mass)
		//Second tangent can be found by cross producting normal with fricTan0
		float4 fricTan0_invMass0[32];
		float4 raXnF0_velMultiplierW[32];
		float4 raXnF1_velMultiplierW[32];
		PxU64  rigidId[32];
		PxU64  particleId[32];
	};

	class PxgParticleSystemCore : public PxgNonRigidCore
	{
	public:
		PxgParticleSystemCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, 
			PxgGpuContext* gpuContext, PxU32 maxParticleContacts);
		virtual ~PxgParticleSystemCore();
	
		virtual void preIntegrateSystems(const PxU32 nbActiveParticleSystems, const PxVec3 gravity, const PxReal dt) = 0;
		virtual void updateBounds(PxgParticleSystem* particleSystems, PxU32* activeParticleSystems, const PxU32 nbActiveParticleSystems);
		virtual void updateGrid() = 0;
		virtual void selfCollision() = 0;
		void resetContactCounts();
		void sortContacts(const PxU32 nbActiveParticleSystems);


		virtual void gpuMemDmaUpParticleSystem(PxgBodySimManager& bodySimManager, CUstream stream) = 0;

		virtual void constraintPrep(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr solverCoreDescd, CUdeviceptr sharedDescd,
			const PxReal dt, CUstream solverStream, bool isTGS, PxU32 numSolverBodies) = 0;
		virtual void solve(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, 
			CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, const PxReal dt, CUstream solverStream) = 0;
		virtual void solveTGS(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd,
			CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, const PxReal dt, const PxReal totalInvDt, CUstream solverStream,
			const bool isVelocityIteration, PxI32 iterationIndex, PxI32 numTGSIterations, PxReal coefficient) = 0;

		virtual void integrateSystems(const PxReal dt, const PxReal epsilonSq) = 0;
		virtual void onPostSolve() = 0;

		virtual void getMaxIterationCount(PxgBodySimManager& bodySimManager, PxI32& maxPosIters, PxI32& maxVelIters) = 0;
		
		void updateParticleSystemData(PxgParticleSystem& sys, Dy::ParticleSystemCore& dyParticleSystemCore);
		//void updateSparseGridParams(PxSparseGridParams& params, Dy::ParticleSystemCore& dyParticleSystemCore);

		PxgTypedCudaBuffer<PxgParticleSystem>& getParticleSystemBuffer() { return mParticleSystemBuffer;  }
		PxgTypedCudaBuffer<PxU32>& getActiveParticleSystemBuffer() { return mActiveParticleSystemBuffer; }
		PxgCudaBuffer& getTempCellsHistogram() { return mTempCellsHistogramBuf; }
		PxgTypedCudaBuffer<PxU32>& getTempBlockCellsHistogram() { return mTempBlockCellsHistogramBuf; }
		PxgTypedCudaBuffer<PxU32>& getTempHistogramCount() { return mTempHistogramCountBuf; }

		PxgTypedCudaBuffer<PxgParticlePrimitiveContact>& getParticleContacts() { return mPrimitiveContactsBuf; }
		PxgTypedCudaBuffer<PxU32>& getParticleContactCount() { return mPrimitiveContactCountBuf; }
	
		CUstream getStream() { return mStream; }
		CUstream getFinalizeStream() { return mFinalizeStream; }

		CUevent getBoundsUpdatedEvent() { return mBoundUpdateEvent;}

		PxgDevicePointer<float4> getDeltaVelParticle() { return mDeltaVelParticleBuf.getTypedDevicePtr(); }

		virtual void updateParticles(const PxReal dt) = 0;

		virtual void finalizeVelocities(const PxReal dt, const PxReal scale) = 0;

		virtual void releaseParticleSystemDataBuffer() = 0;

		void gpuDMAActiveParticleIndices(const PxU32* activeParticleSystems, const PxU32 numActiveParticleSystems, CUstream stream);
		
		PX_FORCE_INLINE PxU32 getMaxParticles() { return mMaxParticles; }

		PxU32 getHostContactCount() { return *mHostContactCount; }

		PxPinnedArray<PxgParticleSystem>			mNewParticleSystemPool; //record the newly created particle system
		PxPinnedArray<PxgParticleSystem>			mParticleSystemPool; //persistent cpu mirror

		PxArray<PxU32>								mNewParticleSystemNodeIndexPool;
		PxArray<PxU32>								mParticleSystemNodeIndexPool;

		PxInt32ArrayPinned							mDirtyParamsParticleSystems;

	protected:

		void releaseInternalParticleSystemDataBuffer();
		
		void getMaxIterationCount(PxgBodySimManager& bodySimManager, const PxU32 nbActiveParticles, const PxU32* activeParticles, PxI32& maxPosIters, PxI32& maxVelIters);

		void updateGrid(PxgParticleSystem* particleSystems, const PxU32* activeParticleSystems, const PxU32 nbActiveParticles,
			CUdeviceptr particleSystemsd);


		void copyUserBufferToUnsortedArray(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, 
			const PxU32 nbActiveParticle, CUstream bpStream);

		void copyUnsortedArrayToUserBuffer(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles);

		void copyUserBufferDataToHost(PxgParticleSystem* particleSystems, PxU32* activeParticleSystems, PxU32 nbActiveParticleSystems);

		void copyUserDiffuseBufferToUnsortedArray(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
			const PxU32 nbActiveParticle, CUstream bpStream);
		
		//integrate particle position based on gravity
		void preIntegrateSystem(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles, const PxVec3 gravity,
			const PxReal dt, CUstream bpStream);
		
		// calculate particle system's world bound
		void updateBound(const PxgParticleSystem& sys, PxgParticleSystem* particleSystems,
			PxBounds3* boundArray, PxReal* contactDists, CUstream bpStream);
		
		// calculate grid hash
		void calculateHash(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 numActiveParticleSystems);

		// reorder particle arrays into sorted order and
		// find start and end of each cell
		void reorderDataAndFindCellStart(PxgParticleSystem* particleSystems, CUdeviceptr particleSystemsd, const PxU32 id, const PxU32 numParticles);
		
		void selfCollision(PxgParticleSystem& particleSystem, PxgParticleSystem* particleSystemsd, const PxU32 id, const PxU32 numParticles);

		//----------------------------------------------------------------------------------------
		//These method are using the particle stream
		virtual void prepParticleConstraint(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr sharedDescd, bool isTGS, const PxReal dt);

		//void solveSelfCollision(PxgParticleSystem& particleSystem, PxgParticleSystem* particleSystemsd, const PxU32 id, const PxU32 numParticles, PxReal dt);

		void applyDeltas(CUdeviceptr particleSystemd, CUdeviceptr activeParticleSystemd, const PxU32 nbActivParticleSystem, const PxReal dt, CUstream stream);
		/*void solveSprings(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
			const PxU32 nbActiveParticleSystems, const PxReal dt, bool isTGS);
*/
		void solveOneWayCollision(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, 
			const PxU32 nbActiveParticleSystems, const PxReal dt, const PxReal biasCoefficient, const bool isVelocityIteration);

		void updateSortedVelocity(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
			const PxU32 nbActiveParticleSystems, const PxReal dt, const bool skipNewPositionAdjustment = false);

		void stepParticleSystems(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticleSystems,
			const PxReal dt, const PxReal totalInvDt, bool isParticleSystem);

		//-------------------------------------------------------------------------------
		//These method are using the solverStream
		void prepPrimitiveConstraint(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr sharedDescd,
			const PxReal dt, bool isTGS, CUstream solverStream);
		
		//this is for solving contacts between particles and primitives based on sorted by rigid id
		void solvePrimitiveCollisionForParticles(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, 
			CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, const PxReal dt, bool isTGS, const PxReal coefficient,
			bool isVelIteration);

		void solvePrimitiveCollisionForRigids(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd,
			CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, CUstream solverStream, const PxReal dt, bool isTGS, const PxReal coefficient,
			bool isVelIteration);

		void accumulateRigidDeltas(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, 
			CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, CUdeviceptr rigidIdsd, CUdeviceptr numIdsd, CUstream stream,
			const bool isTGS);

		void prepRigidAttachments(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, bool isTGS, const PxReal dt, CUstream stream,
			const PxU32 nbActiveParticleSystems, CUdeviceptr activeParticleSystemsd, PxU32 numSolverBodies);

		void solveRigidAttachments(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, 
			CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, CUstream solverStream, const PxReal dt, 
			const bool isTGS, 	const PxReal biasCoefficient, const bool isVelocityIteration, CUdeviceptr particleSystemd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems);

		//integrate particle position and velocity based on contact constraints
		void integrateSystem(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt, const PxReal epsilonSq);
				
		PxgTypedCudaBuffer<PxgParticleSystem> mParticleSystemBuffer; //persistent buffer for particle system
		PxgTypedCudaBuffer<PxU32>		mActiveParticleSystemBuffer;
				
		PxgTypedCudaBuffer<PxU32>		mTempGridParticleHashBuf;
		PxgTypedCudaBuffer<PxU32>		mTempGridParticleIndexBuf;
		PxgTypedCudaBuffer<PxU32>		mTempGridDiffuseParticleHashBuf;
		PxgTypedCudaBuffer<PxU32>		mTempGridDiffuseParticleIndexBuf;

		PxgCudaBuffer					mTempCellsHistogramBuf;
		PxgTypedCudaBuffer<PxU32>		mTempBlockCellsHistogramBuf;
		PxgTypedCudaBuffer<PxU32>		mTempHistogramCountBuf;
		PxgTypedCudaBuffer<PxBounds3>	mTempBoundsBuf;

		//-----------------------buffer for primitive vs particles contacts
		PxgTypedCudaBuffer<PxgParticlePrimitiveContact> mPrimitiveContactsBuf;
		PxgTypedCudaBuffer<PxU32>		mPrimitiveContactCountBuf;

		PxgTypedCudaBuffer<PxgParticlePrimitiveContact> mPrimitiveContactSortedByParticleBuf;
		PxgTypedCudaBuffer<PxgParticlePrimitiveContact> mPrimitiveContactSortedByRigidBuf;

		PxgTypedCudaBuffer<PxgParticlePrimitiveConstraintBlock> mPrimitiveConstraintSortedByParticleBuf;
		PxgTypedCudaBuffer<PxgParticlePrimitiveConstraintBlock> mPrimitiveConstraintSortedByRigidBuf;

		PxgTypedCudaBuffer<float2>		mPrimitiveConstraintAppliedParticleForces;
		PxgTypedCudaBuffer<float2>		mPrimitiveConstraintAppliedRigidForces;

		PxgTypedCudaBuffer<float4>		mDeltaVelParticleBuf;

		PxgTypedCudaBuffer<float4>		mDeltaVelRigidBuf;
		PxgTypedCudaBuffer<PxVec4>		mTempBlockDeltaVelBuf;
		PxgTypedCudaBuffer<PxU64>		mTempBlockRigidIdBuf;

		PxgTypedCudaBuffer<PxgParticleRigidConstraint> mParticleRigidConstraints;
		PxgTypedCudaBuffer<PxU64>		mParticleRigidAttachmentIds;
		PxgTypedCudaBuffer<PxU32>		mParticleRigidConstraintCount;
		PxgTypedCudaBuffer<PxReal>		mParticleRigidAttachmentScaleBuffer;
		


		//------------------------------------------------------------------------

		CUstream								mFinalizeStream;
		CUevent									mFinalizeStartEvent;
		CUevent									mBoundUpdateEvent;//this event is used to synchronize the broad phase stream(updateBound is running on broad phase stream) and mStream
		CUevent									mSolveParticleEvent; //this event is used to synchronize solve particle/particle, paricle/rigid and solver rigid/particle
		CUevent									mSelfCollisionEvent;
		CUevent									mSolveParticleRigidEvent;
		CUevent									mSolveRigidParticleEvent;
		PxU32									mCurrentTMIndex; //current temp marker buffer index

		PxgCudaBuffer							mHasFlipPhase;


		PxArray<PxgParticleSystemBuffer*>		mParticleSystemDataBuffer; //persistent data, map with mParticleSystemBuffer


		// Diffuse particles
		PxPinnedArray<PxgRadixSortBlockDesc> mDiffuseParticlesRSDesc;
	
		PxVec3									mGravity; //this get set in preIntegrateSystems

		PxU32									mNbTotalParticleSystems;
		PxU32									mMaxParticles;
		bool									mHasNonZeroFluidBoundaryScale;
		PxU32									mMaxParticlesPerBuffer;
		PxU32									mMaxBuffersPerSystem;
		PxU32									mMaxDiffusePerBuffer;
		PxU32									mMaxDiffuseBuffersPerSystem;
		PxU32									mMaxRigidAttachmentsPerSystem;
		PxU32									mTotalRigidAttachments;
		PxU32*									mHostContactCount;
	

		friend class PxgSoftBodyCore;
};

class PxgDiffuseParticleCore
{
public:
	PxgDiffuseParticleCore(PxgEssentialCore* core);

	virtual ~PxgDiffuseParticleCore();

	void releaseInternalDiffuseParticleDataBuffer();

	void preDiffuseIntegrateSystem(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles, const PxVec3 gravity,
		const PxReal dt, CUstream bpStream);

	PxgEssentialCore*							mEssentialCore;
	PxgCudaBuffer								mDiffuseParticlesRandomTableBuf;

	PxArray<PxgParticleSystemDiffuseBuffer*>	mDiffuseParticleDataBuffer; //persistent data
	PxU32										mRandomTableSize;
	PxU32										mMaxDiffuseParticles; //the max number of diffuse particles

protected:
	void resizeDiffuseParticleParticleBuffers(PxgParticleSystem& particleSystem, PxgParticleSystemDiffuseBuffer* buffer, const PxU32 numParticles);
};

}

#endif
