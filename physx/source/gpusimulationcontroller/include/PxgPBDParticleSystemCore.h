// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_PBD_PARTICLE_SYSTEM_CORE_H
#define PXG_PBD_PARTICLE_SYSTEM_CORE_H

#include "PxgParticleSystemCore.h"

namespace physx
{
	class PxgPBDParticleSystemCore : public PxgParticleSystemCore, public PxgDiffuseParticleCore
	{
	public:
		PxgPBDParticleSystemCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgAllocatorDesc& allocDesc, PxgSimulationController* simController,
			PxgGpuContext* gpuContext, PxU32 maxParticleContacts);
		virtual ~PxgPBDParticleSystemCore();


		virtual void preIntegrateSystems(const PxU32 nbActiveParticleSystems, const PxVec3 gravity, const PxReal dt) PX_OVERRIDE;
		//virtual void updateBounds(PxgParticleSystem* particleSystems, PxU32* activeParticleSystems, const PxU32 nbActiveParticleSystems);
		virtual void updateGrid() PX_OVERRIDE;
		virtual void selfCollision() PX_OVERRIDE;
		//this is for solving selfCollsion and contacts between particles and primitives based on sorted by particle id

		virtual void constraintPrep(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr solverCoreDescd, CUdeviceptr sharedDescd,
			const PxReal dt, CUstream solverStream, bool isTGS, PxU32 numSolverBodies) PX_OVERRIDE;
		virtual void updateParticles(const PxReal dt, bool isVelocityIteration) PX_OVERRIDE;
		virtual void solve(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, CUdeviceptr artiCoreDescd,
			const PxReal particleBiasCoefficient, const PxReal dt, const PxReal totalInvDt,
			const bool isFirstIteration, const bool isVelocityIteration, CUstream solverStream) PX_OVERRIDE;

		virtual void prepParticleConstraint(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr sharedDescd, bool isTGS, const PxReal dt) PX_OVERRIDE;


		virtual void integrateSystems(const PxReal dt, const PxReal epsilonSq) PX_OVERRIDE;
		virtual void onPostSolve() PX_OVERRIDE;
		virtual void gpuMemDmaUpParticleSystem(PxgBodySimManager& bodySimManager, CUstream stream) PX_OVERRIDE;
		virtual void getMaxIterationCount(PxgBodySimManager& bodySimManager, PxI32& maxPosIters, PxI32& maxVelIters) PX_OVERRIDE;
		virtual void releaseParticleSystemDataBuffer() PX_OVERRIDE;

		void solveVelocities(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticleSystems, const PxReal dt);

		void solveParticleCollision(const PxReal dt, bool isTGS, PxReal coefficient);

		virtual void finalizeVelocities(const PxReal dt, const PxReal scale) PX_OVERRIDE;

	private:

		void allocateParticleBuffer(const PxU32 nbTotalParticleSystems, CUstream stream);
		void allocateParticleDataBuffer(void** bodySimsLL, CUstream stream);
		void updateDirtyData(PxgBodySimManager& bodySimManager, CUstream stream);

		void resizeParticleDataBuffer(PxgParticleSystem& particleSystem, PxgParticleSystemBuffer* buffer, const PxU32 maxParticles, const PxU32 maxNeighborhood, CUstream stream);
		void resizeDiffuseParticleDiffuseBuffer(PxgParticleSystem& particleSystem, PxgParticleSystemDiffuseBuffer* diffuseBuffer, const PxU32 maxDiffuseParticles, CUstream stream);
		bool createUserParticleData(PxgParticleSystem& particleSystem, Dy::ParticleSystemCore& dyParticleSystemCore, PxgParticleSystemBuffer* buffer, PxgParticleSystemDiffuseBuffer* diffuseBuffer,
			CUstream stream);

		void calculateHashForDiffuseParticles(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 numActiveParticleSystems);

		void solveDensities(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt,
			PxReal coefficient);

		void solveDiffuseParticles(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt);

		//-------------------------------------------------------------------------------
		// Materials
		void updateMaterials(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, CUstream bpStream, const PxReal invTotalDt);

		PxU32							mMaxNumPhaseToMaterials; //compute the max number of phase to materials for each particle system
		bool							mComputePotentials;
		PxU32							mNumActiveParticleSystems;
	};
}


#endif
