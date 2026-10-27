// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_FEMCLOTH_CORE_H
#define PXG_FEMCLOTH_CORE_H

#include "PxgFEMCore.h"

namespace physx
{

#define FEMCLOTH_MAX_NUM_PARTITIONS 32
	
	// this is needed to force PhysXSimulationControllerGpu linkage as Static Library!
	void createPxgFEMCloth();

	namespace Dy
	{
		class DeformableSurface;
	}

	class PxRenderOutput;
	class PxPostSolveCallback;

	class PxgFEMCloth;
	struct PxgFEMClothData;
	struct PxgSolverCoreDesc;
	struct PxgArticulationCoreDesc;
	struct PxgPrePrepDesc;
	struct PxgConstraintPrepareDesc;
	struct PxgSolverSharedDescBase;

	class PxgFEMClothCore : public PxgFEMCore
	{
	  public:
		PxgFEMClothCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
						PxgAllocatorDesc& allocDesc, PxgSimulationController* simController, PxgGpuContext* context,
						PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS);

		~PxgFEMClothCore();

		void preIteration();

		// Integrate verts position based on gravity
		void preIntegrateSystems(PxU32 nbActiveFEMCloths, const PxVec3& gravity, PxReal dt);

		// Calculate femCloth's world bound
		void refitBound(PxU32 nbActiveFEMCloths, CUstream stream);

		void resetClothVsNonclothContactCounts();
		void checkBufferOverflows();

		void updateClothContactPairValidity(bool forceUpdateClothContactPairs, bool adaptiveCollisionPairUpdate, PxReal dt);

		void selfCollision(bool isVT);

		void differentClothCollision(bool isVT);

		void clampContactCounts();

		void sortContacts(PxU32 nbActiveFemCloths);

		void solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
				   PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
				   PxReal dt, CUstream solverStream, PxU32 iter, PxU32 maxIter, bool isVelocityIteration, const PxVec3& gravity,
				   PxReal particleCollisionBiasCoefficient, PxReal rigidAttachmentBiasCoefficient);

		void solve_position(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
							PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
							PxReal dt, CUstream solverStream, PxU32 iter, PxU32 maxIter, const PxVec3& gravity,
							PxReal particleCollisionBiasCoefficient, PxReal rigidAttachmentBiasCoefficient);

		void solve_velocity(PxU32 iter, PxU32 maxIter, PxReal dt);

		void step(PxReal dt, CUstream stream, PxU32 nbFEMCloths, const PxVec3& gravity, bool adaptiveCollisionPairUpdate, bool forceUpdateClothContactPairs);

		void finalizeVelocities(PxReal dt);

		void constraintPrep(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
							PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream,
							PxU32 nbSolverBodies, PxU32 nbArticulations);

		bool updateUserData(Cm::PinnableArray<PxgFEMCloth>& femClothPool, PxArray<PxU32>& femClothNodeIndexPool, const PxU32* activeFEMCloths,
							PxU32 nbActiveFEMCloths, void** bodySimsLL);

		CUstream getStream() { return mStream; }

		void partitionTriangleSimData(PxgFEMCloth& femCloth, PxgFEMClothData& clothData, PxArray<PxU32>& orderedTriangles,
									  const PxArray<PxU32>& activeTriangles);

		void partitionTrianglePairSimData(PxgFEMCloth& femCloth, PxgFEMClothData& clothData, PxU32 maximumPartitions,
										  PxArray<PxU32>& orderedTrianglePairs, const PxArray<PxU32>& activeTrianglePairs,
										  const PxArray<uint4>& trianglePairVertexIndices, bool isSharedTrianglePair);

		PxgCudaPagedLinearAllocator& getStackAllocator() { return mIntermStackAlloc; }

		PX_FORCE_INLINE PxU32 getMaxContacts() { return mMaxContacts; }

		void applyDamping(PxU32 nbActiveFemCloths, PxReal dt, CUstream stream);

		// Apply position delta change original triangle mesh
		void applyExternalDelta(PxU32 nbActiveFemCloths, PxReal dt, CUstream stream);

		void drawContacts(PxRenderOutput& out);

		void syncCloths();

		void createActivatedDeactivatedLists();

		void onClothRemoved();

	  private:
		void preIntegrateSystem(PxgFEMCloth* femClothsd, PxU32* activeFemCloths, PxU32 nbActiveFemCloths, PxU32 maxVertices,
								const PxVec3& gravity, PxReal dt, CUstream bpStream);

		void prepRigidContactConstraint(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
										PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream,
										PxU32 numSolverBodies, PxU32 numArticulations);

		void prepRigidAttachmentConstraints(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
											PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd, PxReal /*invDt*/,
											PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream stream);

		void prepClothAttachmentConstraints(CUstream stream);

		void prepClothParticleConstraint();

		// These method are running at the cloth stream
		void prepClothContactConstraint(bool isVT);

		void solveShellEnergy(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths, PxReal dt);
		void solveNonSharedTriangles(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths, PxReal dt);
		void solveTrianglePairs(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths, PxReal dt,
								bool isSharedTrianglePair);

		void queryRigidContactReferenceCount(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
											 PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
											 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
											 PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt);

		// Pre-count per-vertex refCount for rigid-cloth attachments. Bumps
		// cloth.mDeltaPos[v].w; the attach + contact solves both read .w as the
		// per-vertex Jacobi mass-splitting factor, for both PGS and TGS. Must
		// run after the FEM finalize (so .w starts from 0) and before both
		// attach + contact solves.
		void queryRigidAttachmentReferenceCount(CUstream solverStream);

		// Solve cloth vs rigid body contact
		void solveClothRigidContacts(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
									 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
									 PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt);

		// Solve cloth vs rigid body attachment
		void solveClothRigidAttachment(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
									   PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
									   PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt,
									   PxReal biasCoefficient);

		void solveClothAttachmentDelta();

		void prepareClothClothCollision(bool forceUpdateClothContactPairs, bool adaptiveCollisionPairUpdate, PxReal dt);

		void solveClothClothCollision(PxU32 nbActiveFEMCloths, PxReal dt);

		// Solve cloth vs cloth contact and output to cloth delta buffer
		void solveClothContactsOutputClothDelta(PxReal dt, bool isVT);

		// Solve cloth vs particle contact and output to cloth delta buffer
		void solveParticleContactsOutputClothDelta(CUstream particleStream, PxReal biasCoefficient);

		// Solve cloth vs particle contact and output to particle delta buffer
		void solveParticleContactsOutputParticleDelta(CUstream particleStream, PxReal biasCoefficient);

		//--------------------------------------------------------------------------------------

		PxgTypedCudaBuffer<PxU8> mUpdateClothContactPairs; // When set to 1, updates the cloth-cloth contact pairs.

		CUevent mSolveRigidEvent;			  // This event is recorded at the solver stream and the cloth stream need to wait for
											  // that event finish before it processes
		CUevent mConstraintPrepParticleEvent; // This event is used to synchronize constraint prep(cloth stream) and
											  // solve cloth vs particle system contacts (particle stream)
		CUevent mSolveParticleEvent;		  // This event is used to synchronize particle system contacts (particle stream)
											  // before we call applyExternalTetraDelta

	  public:
		PxArray<Dy::DeformableSurface*> mActivatingDeformableSurfaces;
		PxArray<Dy::DeformableSurface*> mDeactivatingDeformableSurfaces;
		PxPostSolveCallback* mPostSolveCallback;
	};
}

#endif
