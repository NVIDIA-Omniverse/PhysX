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

#ifndef PXG_FEMCLOTH_CORE_H
#define PXG_FEMCLOTH_CORE_H

#include "PxgFEMCore.h"

namespace physx
{

#define FEMCLOTH_MAX_NUM_PARTITIONS 32

	namespace Dy
	{
		struct DeformableSurfaceCore;
		class DeformableSurface;
	}

	// this is needed to force PhysXSimulationControllerGpu linkage as Static Library!
	void createPxgFEMCloth();

	struct PxGpuDynamicsMemoryConfig;

	class PxgCudaBroadPhaseSap;
	class PxgGpuNarrowphaseCore;
	class PxgFEMCloth;
	struct PxgFEMClothData;

	class PxRenderBuffer;

	class PxRenderOutput;

	struct PxgSolverCoreDesc;
	struct PxgArticulationCoreDesc;

	class PxPostSolveCallback;

	struct PxgClothConstraintBlock
	{
		float2 friction_restDist[32]; // x: friction, y: restDist
	};

	struct PxgPrePrepDesc;
	struct PxgConstraintPrepareDesc;
	struct PxgSolverSharedDescBase;

	class PxgFEMClothCore : public PxgFEMCore
	{
	  public:
		PxgFEMClothCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
						PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* context,
						PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS);
		~PxgFEMClothCore();

		void preIteration();

		// Integrate verts position based on gravity
		void preIntegrateSystems(PxU32 nbActiveFEMCloths, const PxVec3& gravity, PxReal dt);

		// Calculate femCloth's world bound
		void refitBound(PxU32 nbActiveFEMCloths, CUstream stream);

		void resetClothVsNonclothContactCounts();
		void checkBufferOverflows();

		void updateClothContactPairValidity();

		void selfCollision();

		void differentClothCollision();

		void clampContactCounts();

		void sortContacts(PxU32 nbActiveFemClothes);

		void solve(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
				   PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
				   PxReal dt, CUstream solverStream, PxU32 iter, PxU32 maxIter, bool isVelocityIteration, const PxVec3& gravity);

		void solve_position(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
							PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
							PxReal dt, CUstream solverStream, PxU32 iter, PxU32 maxIter, const PxVec3& gravity);

		void solve_velocity(PxU32 iter, PxU32 maxIter, PxReal dt);

		void step(PxReal dt, CUstream stream, PxU32 nbFEMCloths, const PxVec3& gravity, bool forceUpdateClothContactPairs);

		void finalizeVelocities(PxReal dt);

		void constraintPrep(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgConstraintPrepareDesc> prepDescd,
							PxReal invDt, PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, CUstream solverStream,
							PxU32 nbSolverBodies, PxU32 nbArticulations);

		bool updateUserData(PxPinnedArray<PxgFEMCloth>& femClothPool, PxArray<PxU32>& femClothNodeIndexPool, const PxU32* activeFEMCloths,
							PxU32 nbActiveFEMCloths, void** bodySimsLL);

		CUstream getStream() { return mStream; }

		void partitionTriangleSimData(PxgFEMCloth& femCloth, PxgFEMClothData& clothData, PxArray<PxU32>& orderedTriangles,
									  const PxArray<PxU32>& activeTriangles, PxsHeapMemoryAllocator* alloc);
		void partitionTrianglePairSimData(PxgFEMCloth& femCloth, PxgFEMClothData& clothData, PxU32 maximumPartitions,
										  PxArray<PxU32>& orderedTrianglePairs, const PxArray<PxU32>& activeTrianglePairs,
										  const PxArray<uint4>& trianglePairVertexIndices, bool isSharedTrianglePair,
										  PxsHeapMemoryAllocator* alloc);

		PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator>& getStackAllocator() { return mIntermStackAlloc; }

		PX_FORCE_INLINE PxU32 getMaxContacts() { return mMaxContacts; }

		void applyDamping(PxU32 nbActiveFemClothes, PxReal dt, CUstream stream);

		// Apply position delta change original triangle mesh
		void applyExternalDelta(PxU32 nbActiveFemClothes, PxReal dt, CUstream stream);

		void applyExternalDeltaAndCheckClothCollisionValidity(PxU32 nbActiveFemClothes, PxReal dt, bool adaptiveCollisionPairUpdate);

		void applyExternalDeltaWithVelocityClamping(PxU32 nbActiveFemClothes, PxReal dt, CUstream stream);

		void drawContacts(PxRenderOutput& out);

		void syncCloths();

		void createActivatedDeactivatedLists();

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
		void prepClothContactConstraint();

		void solveShellEnergy(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths, PxReal dt);
		void solveNonSharedTriangles(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths, PxReal dt);
		void solveTrianglePairs(PxgFEMCloth* femClothsd, PxgDevicePointer<PxU32> activeFEMClothsd, PxU32 nbActiveFEMCloths, PxReal dt,
								bool isSharedTrianglePair);

		void queryRigidContactReferenceCount(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
											 PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
											 PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
											 PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt);

		// Solve cloth vs rigid body contact and output to cloth delta buffer
		void solveRigidContactsOutputClothDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
												PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
												PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
												PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt);

		// Solve cloth vs rigid body contact and output to rigid delta buffer
		void solveRigidContactsOutputRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
												PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
												PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
												PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt);

		void solveRigidAttachmentRigidDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
											PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
											PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
											PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, CUstream solverStream, PxReal dt);

		void solveRigidAttachmentClothDelta(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd,
											PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
											PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd,
											PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxReal dt);

		void solveClothAttachmentDelta();

		void rewindCloth(PxU32 nbActiveFEMCloths);

		void advanceSubstep(PxU32 nbActiveFEMCloths, PxU32 nbCollisionSubsteps, PxReal dt);

		void prepareClothClothCollision(bool forceUpdateClothContactPairs, bool adaptiveCollisionPairUpdate);

		void solveClothClothCollision(PxU32 nbActiveFEMCloths, PxReal dt);

		// Solve cloth vs cloth contact and output to cloth delta buffer
		void solveClothContactsOutputClothDelta(PxReal dt);

		// Solve cloth vs particle contact and output to cloth delta buffer
		void solveParticleContactsOutputClothDelta(CUstream particleStream);

		// Solve cloth vs particle contact and output to particle delta buffer
		void solveParticleContactsOutputParticleDelta(CUstream particleStream);

		//--------------------------------------------------------------------------------------

		PxgTypedCudaBuffer<PxU8> mUpdateClothContactPairs; // When set to 1, updates the cloth-cloth contact pairs.

		CUevent mBoundUpdateEvent;			  // This event is used to synchronize the broad phase stream(updateBound is running on
											  // broad phase stream) and mStream
		CUevent mSolveClothEvent;			  // This event is recorded at the cloth stream and the solver stream need to wait for
											  // that event finish before it processes
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
