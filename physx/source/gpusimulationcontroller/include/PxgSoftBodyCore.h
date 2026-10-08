// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SOFTBODY_CORE_H
#define PXG_SOFTBODY_CORE_H

#include "PxgFEMCore.h"
#include "PxgSoftBody.h"
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

		// Solve entry point for both PGS and TGS (dispatches on mIsTGS).
		void solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, const PxReal rigidAttachmentBiasCoefficient, const PxVec3& gravity,
			const PxReal dt, const bool isFirstIteration, const bool isVelocityIteration, CUstream solverStream);

		void calculateStress();

		void plasticDeformation();
		
		void constraintPrep(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream,
			const bool isTGS, PxU32 nbSolverBodies, PxU32 nbArticulations);

		bool updateUserData(Cm::PinnableArray<PxgSoftBody>& softBodyPool, PxArray<PxU32>& softBodyNodeIndexPool,
			const PxU32* activeSoftBodies, const PxU32 nbActiveSoftBodies, void** bodySimsLL);

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
		void finalizeVelocities(const PxReal dt);

		//apply position delta change original grid model tetra mesh
		void applyExternalTetraDeltaGM(const PxU32 nbActiveSoftbodies, const PxReal dt, CUstream stream, bool isVelocityIteration);

	private:
		//integrate verts position based on gravity
		void preIntegrateSystem(PxgDevicePointer<PxgSoftBody> softbodiesd, PxgDevicePointer<PxU32> activeSoftBodiesd,
			const PxU32 nbActiveSoftBodies, const PxU32 maxVerts, const PxVec3 gravity, const PxReal dt, CUstream bpStream);

		//These method are running at the solverStream
		void prepRigidContactBlocks(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
			const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream, const bool isTGS, PxU32 numSolverBodies, PxU32 numArticulations);

		void prepRigidAttachmentBlocks(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
			const PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream stream, bool isTGS);

		void prepSoftBodyAttachmentBlocks(CUstream stream);

		void prepClothAttachmentBlocks(CUstream stream);

		void solveRSContactsOutputRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt, bool isVelocityIteration);

		//run on soft body stream
		void prepSoftBodyParticleBlocks();

		//run on soft body stream
		void prepSoftBodyClothBlocks();

		//These method are running at the soft body stream
		void prepSoftBodyContactBlocks();

		void updateTetModelVerts(PxgDevicePointer<PxgSoftBody> softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
			const PxU32 nbActiveSoftbodies, CUstream updateStream);

		//solve in the grid model
		void solveCorotationalFEM(PxgSoftBody* softbodies, PxgSoftBody* softbodiesd, PxgDevicePointer<PxU32> activeSoftbodiesd,
			const PxU32 nbActiveSoftbodies, const PxReal dt, CUstream stream, const bool isTGS, const bool isFirstIteration);

		void step(PxReal dt, CUstream stream, const PxU32 nbActiveSoftBodies, const PxVec3& gravity);


		void solveRigidAttachment(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, const PxReal dt, const PxReal biasCoefficient, bool isVelocityIteration);

		void solveSoftBodyAttachmentDelta(PxReal dt, bool isVelocityIteration);

		void solveClothAttachmentDelta(PxReal dt, bool isVelocityIteration);

		void querySoftBodyAttachmentReferenceCount();

		void queryClothAttachmentReferenceCount();

		//solve soft body vs particle contact and output to soft body delta buffer
		void solveSPContactsOutputSoftBodyDelta(const PxReal dt, bool isVelocityIteration);

		//solve soft body vs particle contact and output to particle delta buffer
		void solveSPContactsOutputParticleDelta(const PxReal dt, CUstream particleStream, bool isVelocityIteration);

		void querySPContactReferenceCount(const PxReal dt, bool isVelocityIteration);

		//solve soft body vs cloth contact and update position
		void solveSCContactsOutputDelta(const PxReal dt, bool isVelocityIteration);

		void querySCContactReferenceCount(const PxReal dt, bool isVelocityIteration);

		//solve soft body vs soft body contact and output to soft body delta buffer
		void solveSSContactsOutputSoftBodyDelta(const float dt, const bool isTGS, bool isVelocityIteration);

		void querySSContactReferenceCount(const PxReal dt, bool isVelocityIteration);

		void queryRigidContactReferenceCount(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
			PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt, bool isVelocityIteration);

		void queryRigidAttachmentReferenceCount(CUstream solverStream);

		//--------------------------------------------------------------------------------------

		//soft body vs cloth collision contacts
		PxgTypedCudaBuffer<float4>		mSCContactPointBuffer;
		PxgTypedCudaBuffer<float4>		mSCContactNormalPenBuffer;
		PxgTypedCudaBuffer<float4>		mSCContactBarycentricBuffer0;
		PxgTypedCudaBuffer<float4>		mSCContactBarycentricBuffer1;
		PxgTypedCudaBuffer<PxgFemFemContactInfo>	mSCContactInfoBuffer;
		PxgTypedCudaBuffer<PxU32>		mSCTotalContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mPrevSCContactCountBuffer;
		//contact blocks
		PxgTypedCudaBuffer<PxgDbDbContactBlock>	mSSContactBlocks;   // soft body vs soft body (incl. self-collision)
		PxgTypedCudaBuffer<PxgDbDbContactBlock>	mSCContactBlocks;   // soft body vs cloth

		//To do: ideally, we want to use two separate stream to solve the rigid body and soft body collision
		PxgTypedCudaBuffer<float2>		mSCLambdaNBuf; // accumulated (lambdaN, lambdaT) for collision between FEMCloth and soft body

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
