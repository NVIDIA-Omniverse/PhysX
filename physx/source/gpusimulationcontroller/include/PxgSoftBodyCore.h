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

#ifndef PXG_SOFTBODY_CORE_H
#define PXG_SOFTBODY_CORE_H

#include "PxgFEMCore.h"
#include "PxgSoftBody.h"
#include "PxSoftBodyFlag.h"
#include "foundation/PxPreprocessor.h"

namespace physx
{
	//this is needed to force PhysXSimulationControllerGpu linkage as Static Library!
	void createPxgSoftBody();

	struct PxgConstraintPrepareDesc;
	class PxPostSolveCallback;

	namespace Dy
	{
		class DeformableVolume;
	}

	class PxgSoftBodyCore : public PxgFEMCore
	{
	public:
		PxgSoftBodyCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgAllocatorDesc& allocDesc, PxgSimulationController* simController,
			PxgGpuContext* context, const PxU32 maxContacts, const PxU32 collisionStackSize, const bool isTGS);

		~PxgSoftBodyCore();

		//integrate verts position based on gravity
		void preIntegrateSystems(PxgSoftBody* softbodies, PxU32* activeSoftbodies, const PxU32 nbActiveSoftbodies,
			const PxVec3 gravity, const PxReal dt);

		// calculate softbody's world bound
		void refitBound(PxgSoftBody* softbodies, const PxU32 nbActiveSoftbodies);

		void resetContactCounts();
		void checkBufferOverflows();
		
		void selfCollision();

		//after narrow phase, we sort soft body contact by rigid id and particle id
		void sortContacts(const PxU32 nbActiveSoftbodies);

		void updateSimTetraRotations();

		void updateTetraRotations();

		void solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal dt, CUstream solverStream,
			const bool isFirstIteration, const PxReal biasCoefficient);

		void solveTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal dt, CUstream solverStream,
			const bool isVelocityIteration, const PxReal biasCoefficient, const bool isFirstIteration, const PxVec3& gravity);

		void calculateStress();

		void plasticDeformation();
		
		void constraintPrep(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream,
			const bool isTGS, PxU32 nbSolverBodies, PxU32 nbArticulations);

		bool updateUserData(Cm::PinnableArray<PxgSoftBody>& softBodyPool, PxArray<PxU32>& softBodyNodeIndexPool,
			const PxU32* activeSoftBodies, const PxU32 nbActiveSoftBodies, void** bodySimsLL);

		void copySoftBodyDataDEPRECATED(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbCopySoftBodies, const PxU32 maxSize, CUevent copyEvent);
		void applySoftBodyDataDEPRECATED(void** data, void* dataSizes, void* softBodyIndices, PxSoftBodyGpuDataFlag::Enum flag, const PxU32 nbUpdatedSoftBodies, const PxU32 maxSize, CUevent applyEvent, CUevent signalEvent);

		CUstream getStream() { return mStream; }

		void syncSoftBodies();

		void createActivatedDeactivatedLists();


		PxgCudaBuffer& getTempCellsHistogram() { return mTempCellsHistogramBuf; }
		PxgTypedCudaBuffer<PxU32>& getTempBlockCellsHistogram() { return mTempBlockCellsHistogramBuf; }
		PxgTypedCudaBuffer<PxU32>& getTempHistogramCount() { return mTempHistogramCountBuf; }


		PxgTypedCudaBuffer<float4>& getClothVsSoftBodyContacts() { return mSCContactPointBuffer; }
		PxgTypedCudaBuffer<float4>& getClothVsSoftBodyNormalPens() { return mSCContactNormalPenBuffer; }
		PxgTypedCudaBuffer<float4>& getClothVsSoftBodyBarycentrics0() { return mSCContactBarycentricBuffer0; } //barycentric toward soft body contact
		PxgTypedCudaBuffer<float4>& getClothVsSoftBodyBarycentrics1() { return mSCContactBarycentricBuffer1; } //barycentric toward cloth contact
		PxgTypedCudaBuffer<PxgFemFemContactInfo>& getClothVsSoftBodyContactInfos() { return mSCContactInfoBuffer; }
		PxgTypedCudaBuffer<PxU32>& getClothVsSoftBodyContactCount() { return mSCTotalContactCountBuffer; }
		PxgTypedCudaBuffer<PxU32>& getPrevClothVsSoftBodyContactCount() { return mPrevSCContactCountBuffer; }

		// Create a soft body contact writer for softbody-vs-softbody or self-collision
		PxgSoftBodyContactWriter createSoftBodyContactWriter();

		// Create a soft body contact writer for cloth-vs-softbody collision
		// Uses dedicated cloth-vs-softbody buffers and takes minimum of both max contacts
		PxgSoftBodyContactWriter createClothVsSoftBodyContactWriter(PxU32 clothMaxContacts);

		PxgCudaPagedLinearAllocator& getStackAllocator() { return mIntermStackAlloc; }

		//apply position delta change original grid model tetra mesh
		void finalizeVelocities(const PxReal dt, const PxReal scale, const bool isTGS);

		//apply position delta change original grid model tetra mesh
		void applyExternalTetraDeltaGM(const PxU32 nbActiveSoftbodies, const PxReal dt, CUstream stream);

	private:
		PX_DEPRECATED void copyOrApplySoftBodyDataDEPRECATED(PxU32 dataIndex, PxU32* softBodyIndices, PxU8** data, PxU32* dataSizes, PxU32 maxSizeInBytes, const PxU32 nbSoftbodies, const PxU32 applyDataToSoftBodies);

		//integrate verts position based on gravity
		void preIntegrateSystem(PxgDevicePointer<PxgSoftBody> softbodiesd, PxgDevicePointer<PxU32> activeSoftBodiesd,
			const PxU32 nbActiveSoftBodies, const PxU32 maxVerts, const PxVec3 gravity, const PxReal dt, CUstream bpStream);

		//These method are running at the solverStream
		void prepRigidContactConstraint(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
			const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream, const bool isTGS, PxU32 numSolverBodies, PxU32 numArticulations);

		void prepRigidAttachmentConstraints(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
			const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream stream, bool isTGS);

		void prepSoftBodyAttachmentConstraints(CUstream stream);

		void prepClothAttachmentConstraints(CUstream stream);

		void prepParticleAttachmentConstraints(CUstream stream);

		void solveRSContactsOutputRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt);

		void solveRSContactsOutputRigidDeltaTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
			PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream,
			const PxReal dt);

		//run on soft body stream
		void prepSoftBodyParticleConstraint();

		//run on soft body stream
		void prepSoftBodyClothConstraint();

		//These method are running at the soft body stream
		void prepSoftbodyContactConstraint();

		void updateTetModelVerts(PxgDevicePointer<PxgSoftBody> softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
			const PxU32 nbActiveSoftbodies, CUstream updateStream);

		//solve in the grid model
		void solveCorotationalFEM(PxgSoftBody* softbodies, PxgSoftBody* softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
			const PxU32 nbActiveSoftbodies, const PxReal dt, CUstream stream, const bool isTGS, const bool isFirstIteration);

		void step(PxReal dt, CUstream stream, const PxU32 nbActiveSoftBodies, const PxVec3& gravity);


		void solveRigidAttachment(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, 
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt);

		void solveSoftBodyAttachmentDelta();

		void solveParticleAttachmentDelta();

		void solveClothAttachmentDelta();

		void solveRigidAttachmentTGS(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, 
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt, const PxReal biasCoefficient, bool isVelocityIteration);

		//solve soft body vs particle contact and output to soft body delta buffer
		void solveSPContactsOutputSoftBodyDelta(const PxReal dt, const PxReal biasCoefficient);

		//solve soft body vs particle contact and output to particle delta buffer
		void solveSPContactsOutputParticleDelta(const PxReal dt, const PxReal biasCoefficient, CUstream particleStream);

		//solve soft body vs cloth contact and update position 
		void solveSCContactsOutputDelta();

		//solve soft body vs soft body contact and output to soft body delta buffer
		void solveSSContactsOutputSoftBodyDelta(const float dt, const float biasCoefficient, const bool isTGS);

		void queryRigidContactReferenceCount(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
			PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt);

		//--------------------------------------------------------------------------------------

		//soft body vs cloth collision contacts
		PxgTypedCudaBuffer<float4>		mSCContactPointBuffer;
		PxgTypedCudaBuffer<float4>		mSCContactNormalPenBuffer;
		PxgTypedCudaBuffer<float4>		mSCContactBarycentricBuffer0;
		PxgTypedCudaBuffer<float4>		mSCContactBarycentricBuffer1;
		PxgTypedCudaBuffer<PxgFemFemContactInfo>	mSCContactInfoBuffer;
		PxgTypedCudaBuffer<PxU32>		mSCTotalContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mPrevSCContactCountBuffer;
		//contact prep buffer
		PxgTypedCudaBuffer<PxgSoftBodySoftBodyConstraintBlock>	mSCConstraintBuf; //constraint prep for cloth vs soft body

		//To do: ideally, we want to use two separate stream to solve the rigid body and soft body collision
		PxgTypedCudaBuffer<PxReal>		mSCLambdaNBuf; // accumulated deltaLambdaN for collision between FEMCloth and soft body

		CUevent							mBoundUpdateEvent;//this event is used to synchronize the broad phase stream(updateBound is running on broad phase stream) and mStream
		CUevent							mSolveRigidEvent;
		CUevent							mConstraintPrepSoftBodyParticleEvent; //this event is used to synchronize constraint prep(soft body stream) and solve soft body vs particle system contacts (particle stream)
		CUevent							mSolveSoftBodyParticleEvent; //this event is used to synchronize particle system contacts (particle stream) before we call applyExternalTetraDelta

		public:
		PxArray<Dy::DeformableVolume*>	mActivatingDeformableVolumes;
		PxArray<Dy::DeformableVolume*>	mDeactivatingDeformableVolumes;
		PxPostSolveCallback*			mPostSolveCallback;
	};
}

#endif
