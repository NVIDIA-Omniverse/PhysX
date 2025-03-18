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


#include "PxgPBDParticleSystemCore.h"
#include "foundation/PxAssert.h"
#include "common/PxProfileZone.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgCudaSolverCore.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgSimulationController.h"
#include "PxgSimulationCore.h"
#include "PxgNarrowphaseCore.h"
#include "PxgKernelWrangler.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgRadixSortDesc.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxgRadixSortCore.h"
#include "PxgCudaUtils.h"
#include "DyParticleSystem.h"
#include "PxgContext.h"
#include "CmRandom.h"


#include "cudamanager/PxCudaContext.h"

#define PS_GPU_DEBUG 0

namespace physx
{

	PxgPBDParticleSystemCore::PxgPBDParticleSystemCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
		PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* gpuContext, const PxU32 maxParticleContacts) :
		PxgParticleSystemCore(gpuKernelWrangler, cudaContextManager, heapMemoryManager, simController, gpuContext, maxParticleContacts),
		PxgDiffuseParticleCore(this),	
		mMaxClothBuffersPerSystem(0), mMaxClothsPerBuffer(0), mMaxSpringsPerBuffer(0), mMaxSpringPartitionsPerBuffer(0), mMaxSpringsPerPartitionPerBuffer(0),
		mMaxTrianglesPerBuffer(0), mMaxVolumesPerBuffer(0),
		mMaxRigidBuffersPerSystem(0), mMaxRigidsPerBuffer(0), mMaxNumPhaseToMaterials(0),
		mComputePotentials(false), mNumActiveParticleSystems(0)
	{
		mGpuContext->mGpuParticleSystemCores.pushBack(this);
		mGpuContext->mGpuPBDParticleSystemCore = this;
	}

	PxgPBDParticleSystemCore::~PxgPBDParticleSystemCore()
	{
		if (mGpuContext)
			mGpuContext->mGpuPBDParticleSystemCore = NULL;
	}

	void PxgPBDParticleSystemCore::updateVolumeBound(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 numActiveParticleSystems,
		CUstream bpStream)
	{
		if (mMaxVolumesPerBuffer > 0)
		{
			//Each block to compute a bound for one volume
			const CUfunction updateVolumeBoundKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_VOLUME_BOUND);

			const PxU32 numThreadsPerWarp = 32;
			const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND / numThreadsPerWarp;
			//const PxU32 numBlocks = mMaxVolumesPerBuffer;

			{

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
				};

				CUresult result = mCudaContext->launchKernel(updateVolumeBoundKernelFunction, mMaxVolumesPerBuffer, mMaxBuffersPerSystem, numActiveParticleSystems, numThreadsPerWarp, numWarpsPerBlock, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				/*result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateBound first pass kernel fail!\n");*
					*/

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(bpStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateBound first pass kernel fail!\n");

				//PxgParticleSystem ps;
				//mCudaContext->memcpyDtoH(&ps, particleSystemsd, sizeof(PxgParticleSystem));

				//PxgParticleSimBuffer buffer;
				//mCudaContext->memcpyDtoH(&buffer, (CUdeviceptr)ps.mParticleSimBuffers, sizeof(PxgParticleSimBuffer));
				////Dma back the bound
				//PxParticleVolume volumes[3];
				//mCudaContext->memcpyDtoH(volumes, (CUdeviceptr)buffer.mVolumes, sizeof(PxParticleVolume) * 3);

				//int bob = 0;
				//PX_UNUSED(bob);
#endif
			}
		}
	}

	void PxgPBDParticleSystemCore::preIntegrateSystems(const PxU32 nbActiveParticleSystems, const PxVec3 gravity, const PxReal dt)
	{
		//integrateSystems run on the broad phase stream so we don't need to have an extra event to sync in updateBounds
		CUstream bpStream = mGpuContext->mGpuBp->getBpStream();

		CUdeviceptr particleSystemsd = getParticleSystemBuffer().getDevicePtr();
		CUdeviceptr activeParticleSystemsd = getActiveParticleSystemBuffer().getDevicePtr();

		mGravity = gravity;
		mComputePotentials = true;

		updateMaterials(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, bpStream, 1.f/dt);
		preIntegrateSystem(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, gravity, dt, bpStream);
		preDiffuseIntegrateSystem(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, gravity, dt, bpStream);
	}

	void PxgPBDParticleSystemCore::updateGrid()
	{
		PX_PROFILE_ZONE("PxgPBDParticleSystemCore::UpdateGrid", 0);
		//This make sure updateBounds kernel are finished in GPU
		CUstream bpStream = mGpuContext->mGpuBp->getBpStream();

		synchronizeStreams(mCudaContext, bpStream, mStream, mBoundUpdateEvent);

		//KS - TODO - we need the active list of particle systems here!

		//PxgSimulationCore* simCore = simController->getSimulationCore();
		//const PxU32 numTotalParticleSystems = mSimController->getNbParticleSystems();

		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		PxU32* activeParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.begin();

		PxgParticleSystem* particleSystems = mParticleSystemPool.begin();// mSimController->getPBDParticleSystems();

		CUdeviceptr particleSystemsd = getParticleSystemBuffer().getDevicePtr();
		CUdeviceptr activeParticleSystemsd = getActiveParticleSystemBuffer().getDevicePtr();

		const PxU32 nbRequired = (2u + nbActiveParticleSystems);
		
		mDiffuseParticlesRSDesc.resize(nbRequired * 2u);

	
		//the maximum particles between normal particles and diffuse particles
		mTempGridParticleHashBuf.allocate(mMaxParticles * sizeof(PxU32), PX_FL);
		mTempGridParticleIndexBuf.allocate(mMaxParticles * sizeof(PxU32), PX_FL);
		mTempGridDiffuseParticleHashBuf.allocate(mMaxDiffuseParticles * sizeof(PxU32), PX_FL);
		mTempGridDiffuseParticleIndexBuf.allocate(mMaxDiffuseParticles * sizeof(PxU32), PX_FL);

		//We need 2x rsDesc on the host per-particle system. The reason for this is that, while the sorting occurs synchronously on the 
		//same stream on the device, the host-side buffers could get changed prior to the DMAs having occurred due to device latency
		//const PxU32 nbRequired = (2u + nbActiveParticleSystems);
		mRSDesc.resize(nbRequired * 2u);
		mDiffuseParticlesRSDesc.resize(nbRequired * 2u);

		mRadixCountTotalBuf.allocate(mRadixCountSize * nbRequired, PX_FL);

		for (PxU32 i = 0; i < 2; ++i)
		{
			mRadixSortDescBuf[i].allocate(sizeof(PxgRadixSortBlockDesc)*nbRequired, PX_FL);
		}

		calculateHash(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems);
		calculateHashForDiffuseParticles(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems);

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			PxU32 index = activeParticleSystems[i];
			PxgParticleSystem& ps = particleSystems[index];

			if (particleSystems[activeParticleSystems[i]].mCommonData.mNumParticles)
			{
				// Simulation particles
				{
					const PxU32 numSimParticles = ps.mCommonData.mNumParticles;
					
					PxgRadixSortDesc* rsDescs = &mRSDesc[i * 2];

					CUdeviceptr inputKeyd = reinterpret_cast<CUdeviceptr>(ps.mGridParticleHash);
					CUdeviceptr inputRankd = reinterpret_cast<CUdeviceptr>(ps.mSortedToUnsortedMapping);
					CUdeviceptr outputKeyd = mTempGridParticleHashBuf.getDevicePtr();
					CUdeviceptr outputRankd = mTempGridParticleIndexBuf.getDevicePtr();
					PxgRadixSortCore::updateGPURadixSortDesc(mCudaContext, mStream, inputKeyd, inputRankd, outputKeyd, outputRankd, mRadixCountTotalBuf.getDevicePtr(), rsDescs,
						mRadixSortDescBuf[0].getDevicePtr(), mRadixSortDescBuf[1].getDevicePtr(), numSimParticles);

					//PxgRadixSortCore::sort(mGpuKernelWranglerManager, mCudaContext, mStream, numSimParticles, mRadixSortDescBuf.begin(), PxgRadixSortCore::getNbBits(PxI32(ps.mData.mGridSizeX*ps.mData.mGridSizeY*ps.mData.mGridSizeZ)), rsDescs);
				}

				// Diffuse particles update for PBD
				bool hasDiffuse = ps.mCommonData.mMaxDiffuseParticles > 0;
				if (hasDiffuse)
				{
					const PxU32 numDiffuseParticles = ps.mCommonData.mMaxDiffuseParticles;
					
					PxgRadixSortDesc* rsDescs = &mDiffuseParticlesRSDesc[i * 2];

					CUdeviceptr inputKeyd = reinterpret_cast<CUdeviceptr>(ps.mDiffuseGridParticleHash);
					CUdeviceptr inputRankd = reinterpret_cast<CUdeviceptr>(ps.mDiffuseSortedToUnsortedMapping);
					CUdeviceptr outputKeyd = mTempGridDiffuseParticleHashBuf.getDevicePtr();
					CUdeviceptr outputRankd = mTempGridDiffuseParticleIndexBuf.getDevicePtr();
					PxgRadixSortCore::updateGPURadixSortDesc(mCudaContext, mStream, inputKeyd, inputRankd, outputKeyd, outputRankd, mRadixCountTotalBuf.getDevicePtr() + mRadixCountSize, rsDescs,
						mRadixSortDescBuf[0].getDevicePtr() + sizeof(PxgRadixSortDesc), mRadixSortDescBuf[1].getDevicePtr() + sizeof(PxgRadixSortDesc),
						numDiffuseParticles);

					//PxgRadixSortCore::sort(mGpuKernelWranglerManager, mCudaContext, mStream, numDiffuseParticles, mRadixSortDescBuf.begin(), PxgRadixSortCore::getNbBits(PxI32(ps.mData.mGridSizeX*ps.mData.mGridSizeY*ps.mData.mGridSizeZ)), rsDescs);
				}		


				PxgCudaBuffer* radixSortDescBuf = mRadixSortDescBuf.begin();

				CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK_NO_COUNT);
				CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK_NO_COUNT);

				{
					PxU32 startBit = 0;
					const PxU32 numPass = 8;

					const PxU32 count = hasDiffuse ? 2 : 1;

					for (PxU32 pass = 0; pass < numPass; ++pass)
					{
						const PxU32 descIndex = pass & 1;

						CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

						PxCudaKernelParam radixSortKernelParams[] =
						{
							PX_CUDA_KERNEL_PARAM(rsDesc),
							PX_CUDA_KERNEL_PARAM(startBit)
						};

						CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, count, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
						if (resultR != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

						resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, count, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
						if (resultR != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

						startBit += 4;
					}

					/*CUresult result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail!!\n");*/

#if PS_GPU_DEBUG
					CUresult result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail!!\n");
#endif
				}
			}

			const PxU32 numParticles = PxMax(ps.mCommonData.mNumParticles, ps.mCommonData.mMaxDiffuseParticles);
			reorderDataAndFindCellStart(particleSystems, particleSystemsd, index, numParticles);
		}
	}

	void PxgPBDParticleSystemCore::selfCollision()
	{
		PX_PROFILE_ZONE("PxgPBDParticleSystemCore::selfCollision", 0);
		//PxgSimulationCore* simCore = simController->getSimulationCore();
		//const PxU32 numTotalParticleSystems = mSimController->getNbParticleSystems();
		PxgParticleSystem* particleSystems = mParticleSystemPool.begin();// mSimController->getPBDParticleSystems();

		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		PxU32* activeParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.begin();

		PxgCudaBuffer& particleSystemBuffer = getParticleSystemBuffer();
		PxgParticleSystem* particleSystemsd = reinterpret_cast<PxgParticleSystem*>(particleSystemBuffer.getDevicePtr());

		//synchronizeStreams(mCudaContext, mStream, mSelfCollisionStream, mSelfCollisionEvent);

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			if (particleSystems[activeParticleSystems[i]].mCommonData.mNumParticles)
			{
				const PxU32 index = activeParticleSystems[i];
				PxgParticleSystem& ps = particleSystems[index];
				const PxU32 numParticles = ps.mCommonData.mNumParticles;
				PxgParticleSystemCore::selfCollision(ps, particleSystemsd, index, numParticles);
			}
		}

		//mStream does not yet need to sync with mSelfCollisionStream
		//synchronizeStreams(mSelfCollisionStream, mStream, mSelfCollisionEvent);
		//mCudaContext->streamFlush(mStream);
		//mCudaContext->streamFlush(mSelfCollisionStream);
	}

	void PxgPBDParticleSystemCore::solveVelocities(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticleSystems, const PxReal dt)
	{
		const PxU32 maxNumParticles = getMaxParticles();

		if (maxNumParticles)
		{
			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
				};

				const CUfunction vorticityConfinementKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_VORTICITY_CONFINEMENT);
				{
					const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
					const PxU32 numBlocks = (maxNumParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
					CUresult result = mCudaContext->launchKernel(vorticityConfinementKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mFinalizeStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
				}
			}


			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
					PX_CUDA_KERNEL_PARAM(dt)
				};

				const CUfunction solveVelocitiesKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_VELOCITIES);
				{
					const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
					const PxU32 numBlocks = (maxNumParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
					CUresult result = mCudaContext->launchKernel(solveVelocitiesKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mFinalizeStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
				}
			}
		}
	}


	//this is for solving selfCollsion and contacts between particles and primitives based on sorted by particle id
	void PxgPBDParticleSystemCore::solveParticleCollision(const PxReal dt, bool isTGS, PxReal coefficient)
	{
		//const PxU32 numTotalParticleSystems = mSimController->getNbParticleSystems();
		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();
		CUdeviceptr activeParticleSystemd = getActiveParticleSystemBuffer().getDevicePtr();

		solveSprings(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, dt, isTGS);

		solveDensities(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, dt, coefficient);

		solveShapes(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, dt, coefficient);

		solveInflatables(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, coefficient, dt);
		
		applyDeltas(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, dt, mStream);

		//updateSortedVelocity(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, dt);

		

		//the first iteration compute potentials for diffuse particles
		mComputePotentials = false;

	}

	void PxgPBDParticleSystemCore::constraintPrep(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr /*solverCoreDescd*/, CUdeviceptr sharedDescd,
		const PxReal dt, CUstream solverStream, bool isTGS, PxU32 numSolverBodies)
	{
		//const PxU32 numTotalParticleSystems = mSimController->getNbParticleSystems();

		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		//PxU32* activeParticles = mSimController->getBodySimManager().mActiveParticleSystems.begin();

		if (nbActiveParticleSystems == 0)
			return;

		//Wait for sorting to have completed on mStream before primitivePrep can run
		synchronizeStreams(mCudaContext, solverStream, mStream);
		//Wait for DMA of prePrepDescd and prepDescd before particle constraint prep can run
		synchronizeStreams(mCudaContext, mStream, solverStream);
		

		CUdeviceptr particleSystemsd = getParticleSystemBuffer().getDevicePtr();
		CUdeviceptr activeParticleSystemsd = getActiveParticleSystemBuffer().getDevicePtr();
		initializeSprings(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems);

		prepPrimitiveConstraint(prePrepDescd, prepDescd, sharedDescd, dt, isTGS, solverStream);
		prepParticleConstraint(prePrepDescd, prepDescd, sharedDescd, isTGS, dt);
		prepRigidAttachments(prePrepDescd, prepDescd, isTGS, dt, solverStream, nbActiveParticleSystems, activeParticleSystemsd, numSolverBodies);


		synchronizeStreams(mCudaContext, solverStream, mStream);
	}

	void PxgPBDParticleSystemCore::updateParticles(const PxReal dt)
	{
		updateSortedVelocity(getParticleSystemBuffer().getDevicePtr(),
			getActiveParticleSystemBuffer().getDevicePtr(), mSimController->getBodySimManager().mActivePBDParticleSystems.size(), dt);
	}

	void PxgPBDParticleSystemCore::solve(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd,
		CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, const PxReal dt, CUstream solverStream)
	{

#if PS_GPU_DEBUG
		PX_PROFILE_ZONE("PxgParticleSystemCore.solve", 0);
#endif
		//const PxU32 numTotalParticleSystems = mSimController->getNbParticleSystems();
		const PxU32 nbActiveParticles = mSimController->getBodySimManager().mActivePBDParticleSystems.size();

		if (nbActiveParticles == 0)
			return;

		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		CUdeviceptr activeParticleSystemd = getActiveParticleSystemBuffer().getDevicePtr();

		solveParticleCollision(dt, false, 0.7f);

		synchronizeStreams(mCudaContext, mStream, solverStream);

		solveRigidAttachments(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd,
			solverStream, dt, false, 0.7f, false, particleSystemd, activeParticleSystemd, nbActiveParticles);
		//Wait for solver stream to finish for mStream so it can do particle-rigid and rigid-particle work
		synchronizeStreams(mCudaContext, solverStream, mStream);

		solvePrimitiveCollisionForParticles(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, dt, false, 0.7f, false);
		solvePrimitiveCollisionForRigids(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt, false, 0.7f, false);

		solveOneWayCollision(particleSystemd, activeParticleSystemd, nbActiveParticles, dt, 0.7f, false);

		synchronizeStreams(mCudaContext, mStream, solverStream);
	}


	void PxgPBDParticleSystemCore::finalizeVelocities(const PxReal dt, const PxReal scale)
	{
		//skipping the velocity mixing based on eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS
		//is somewhat arbitrary. The goal is to get towards momentum conservation and accurate 
		//integration of external forces with TGS.
		if (mGpuContext->isExternalForcesEveryTgsIterationEnabled())
		{
			return;
		}

		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		PxU32* activeParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.begin();

		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		PxgParticleSystem* particleSystems = mParticleSystemPool.begin(); //mSimController->getPBDParticleSystems();

		const PxReal totalInvDt = 1.f / dt;

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			if (particleSystems[activeParticleSystems[i]].mCommonData.mNumParticles)
			{
				const PxU32 index = activeParticleSystems[i];
				PxgParticleSystem& ps = particleSystems[index];
				const PxU32 numParticles = ps.mCommonData.mNumParticles;

				{
					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(particleSystemd),
						PX_CUDA_KERNEL_PARAM(index),
						PX_CUDA_KERNEL_PARAM(totalInvDt),
						PX_CUDA_KERNEL_PARAM(scale)
					};

					CUfunction stepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_FINALIZE_PARTICLES);
					{
						const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
						const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
						CUresult result = mCudaContext->launchKernel(stepKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
						PX_ASSERT(result == CUDA_SUCCESS);
						PX_UNUSED(result);
					}

#if PS_GPU_DEBUG
					CUresult result = mCudaContext->streamSynchronize(mStream);
					PX_ASSERT(result == CUDA_SUCCESS);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_finalizeParticlesLaunch kernel fail!\n");
#endif
				}
			}
		}
	}

	void PxgPBDParticleSystemCore::solveTGS(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd,
		CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, const PxReal dt, const PxReal totalInvDt, CUstream solverStream, const bool isVelocityIteration,
		PxI32 iterationIndex, PxI32 /*numTGSIterations*/, PxReal coefficient)
	{
		bool isFirstIteration = iterationIndex == 0;
#if PS_GPU_DEBUG
		PX_PROFILE_ZONE("PxgParticleSystemCore.solveTGS", 0);
#endif
		PX_UNUSED(solverStream); PX_UNUSED(solverCoreDescd); PX_UNUSED(sharedDescd); PX_UNUSED(prePrepDescd); PX_UNUSED(artiCoreDescd);
		//const PxU32 numTotalParticleSystems = mSimController->getNbParticleSystems();
		const PxU32 nbActiveParticles = mSimController->getBodySimManager().mActivePBDParticleSystems.size();

		if (nbActiveParticles == 0)
			return;


		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		//PxgParticleSystem* particleSystems = mParticleSystemPool.begin(); //mSimController->getPBDParticleSystems();
		CUdeviceptr activeParticleSystemd = getActiveParticleSystemBuffer().getDevicePtr();

		if (!isVelocityIteration)
		{
			stepParticleSystems(particleSystemd, activeParticleSystemd, nbActiveParticles, dt, totalInvDt, isFirstIteration);
		}

		solveParticleCollision(dt, true, coefficient);

		////Wait for mStream to finish (all particle-particle work)
		synchronizeStreams(mCudaContext, mStream, solverStream);

		////solveRigidAttachments(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt);
		//////Wait for solver stream to finish for mStream so it can do particle-rigid and rigid-particle work
		//synchronizeStreams(mCudaContext, solverStream, mStream);

		solveRigidAttachments(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt, true,
			coefficient, isVelocityIteration, particleSystemd, activeParticleSystemd, nbActiveParticles);

		synchronizeStreams(mCudaContext, solverStream, mStream);

		solvePrimitiveCollisionForParticles(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, dt, true, PxMin(0.7f, coefficient), isVelocityIteration);
		solvePrimitiveCollisionForRigids(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd, solverStream, dt, true, PxMin(0.7f, coefficient), isVelocityIteration);

		solveOneWayCollision(particleSystemd, activeParticleSystemd, nbActiveParticles, dt, coefficient, isVelocityIteration);

		synchronizeStreams(mCudaContext, mStream, solverStream);

	}




	void PxgPBDParticleSystemCore::integrateSystems(const PxReal dt, const PxReal epsilonSq)
	{
		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		if (nbActiveParticleSystems == 0)
			return;

		mCudaContextManager->acquireContext();

		synchronizeStreams(mCudaContext, mStream, mFinalizeStream, mFinalizeStartEvent);

		PxU32* activeParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.begin();
		PxgParticleSystem* particleSystems = mParticleSystemPool.begin(); 
		PxgCudaBuffer& particleSystemBuffer = getParticleSystemBuffer();
		CUdeviceptr particleSystemsd = particleSystemBuffer.getDevicePtr();
		CUdeviceptr activeParticleSystemsd = getActiveParticleSystemBuffer().getDevicePtr();
		PxU32* particleSystemNodeIndex = mParticleSystemNodeIndexPool.begin();
		
		PxgParticleSystem* gpuParticleSystems = reinterpret_cast<PxgParticleSystem*>(particleSystemsd);

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			const PxU32 index = activeParticleSystems[i];
			PxgParticleSystem& particleSystem = particleSystems[index];
			if (particleSystem.mCommonData.mNumParticles)
			{
				void** bodySimsLL = mSimController->getBodySimManager().mBodies.begin();
				Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[particleSystemNodeIndex[index]]);
				Dy::ParticleSystemCore& dyParticleSystemCore = dyParticleSystem->getCore();

				PxParticleSystemCallback* callback = dyParticleSystemCore.mCallback;
				if (callback)
				{
					PxGpuMirroredPointer<PxGpuParticleSystem> mirroredSystem(gpuParticleSystems + index, &particleSystem);
					callback->onAdvance(mirroredSystem, mStream);
				}
			}
		}

		
		solveDiffuseParticles(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt);

		solveVelocities(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt);
		
		solveAerodynamics(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt);

		integrateSystem(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt, epsilonSq);

		//copy unsortedPositions_InvMass and velocity to user buffer
		copyUnsortedArrayToUserBuffer(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems);

		copyUserBufferDataToHost(particleSystems, activeParticleSystems, nbActiveParticleSystems);

		//PxU32* activeParticleSystemsd = reinterpret_cast<PxU32*>(getActiveParticleSystemBuffer().getDevicePtr());
		updateVolumeBound(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, mFinalizeStream);
		

		//Wait for diffuse particles to complete
		synchronizeStreams(mCudaContext, mStream, mFinalizeStream);

		mCudaContextManager->releaseContext();
	}

	void PxgPBDParticleSystemCore::onPostSolve()
	{
		PX_PROFILE_ZONE("PxgPBDParticleSystemCore::onPostSolve", 0);
		const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.size();
		PxU32* activeParticleSystems = mSimController->getBodySimManager().mActivePBDParticleSystems.begin();

		if (nbActiveParticleSystems == 0)
			return;

		mCudaContextManager->acquireContext();

		PxgParticleSystem* particleSystems = mParticleSystemPool.begin(); //mSimController->getPBDParticleSystems();
		PxU32* particleSystemNodeIndex = mParticleSystemNodeIndexPool.begin(); //mSimController->getPBDParticleSystemNodeIndex();
		void** bodySimsLL = mSimController->getBodySimManager().mBodies.begin();

		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();
		PxgParticleSystem* gpuParticleSystems = reinterpret_cast<PxgParticleSystem*>(particleSystemd);

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			const PxU32 index = activeParticleSystems[i];
			PxU32 nodeIndex = particleSystemNodeIndex[index];
			Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[nodeIndex]);
			Dy::ParticleSystemCore& dyParticleSystemCore = dyParticleSystem->getCore();

			if (dyParticleSystemCore.mCallback)
			{
				PxGpuMirroredPointer<PxGpuParticleSystem> mirroredSystem(gpuParticleSystems + index, &particleSystems[index]);
				dyParticleSystemCore.mCallback->onPostSolve(mirroredSystem, mFinalizeStream);
			}
		}

		mCudaContextManager->releaseContext();
	}

	void PxgPBDParticleSystemCore::gpuMemDmaUpParticleSystem(PxgBodySimManager& bodySimManager, CUstream stream)
	{
		const PxU32 nbTotalPBDParticleSystems = bodySimManager.mTotalNumPBDParticleSystems;

		if (nbTotalPBDParticleSystems > 0)
		{
			void** bodySimsLL = bodySimManager.mBodies.begin();

			allocateParticleBuffer(nbTotalPBDParticleSystems, stream);

			allocateParticleDataBuffer(bodySimsLL, stream);

			const PxU32 nbActivePBDParticleSystems = bodySimManager.mActivePBDParticleSystems.size();
			PxU32* activePBDParticleSystems = bodySimManager.mActivePBDParticleSystems.begin();

			if (bodySimManager.mActivePBDParticleSystemsDirty)
			{
				gpuDMAActiveParticleIndices(activePBDParticleSystems, nbActivePBDParticleSystems, stream);
				bodySimManager.mActivePBDParticleSystemsDirty = false;
			}

			if (nbActivePBDParticleSystems > 0)
			{
				updateDirtyData(bodySimManager, stream);
			}
		}

	}

	void PxgPBDParticleSystemCore::getMaxIterationCount(PxgBodySimManager& bodySimManager, PxI32& maxPosIters, PxI32& maxVelIters)
	{
		const PxU32 nbActivePBDParticles = bodySimManager.mActivePBDParticleSystems.size();
		
		if (nbActivePBDParticles > 0)
		{
			PxU32* activePBDParticles = bodySimManager.mActivePBDParticleSystems.begin();
			PxgParticleSystemCore::getMaxIterationCount(bodySimManager, nbActivePBDParticles, activePBDParticles, maxPosIters, maxVelIters);
		}
	}

	void PxgPBDParticleSystemCore::releaseParticleSystemDataBuffer()
	{

		releaseInternalDiffuseParticleDataBuffer();

		PxgParticleSystemCore::releaseInternalParticleSystemDataBuffer();

	}

	void PxgPBDParticleSystemCore::solveDensities(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt,
		PxReal coefficient)
	{

		const PxU32 maxParticles = getMaxParticles();
		if (maxParticles > 0)
		{
			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
			const PxU32 numBlocks = (maxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;

			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(mComputePotentials)
				};

				const CUfunction calculateDensitiesKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CALCULATE_DENSITIES_AND_POTENTIALS);
				
				CUresult result = mCudaContext->launchKernel(calculateDensitiesKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}

			const CUfunction solveDensitiesKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_DENSITIES);
			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(coefficient),
					PX_CUDA_KERNEL_PARAM(dt)
				};

			
				CUresult result = mCudaContext->launchKernel(solveDensitiesKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}

#if 0
			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(id),
					PX_CUDA_KERNEL_PARAM(particleSystem.mData.mRelaxationFactor),
					PX_CUDA_KERNEL_PARAM(dt),
				};

				const CUfunction applyDeltasKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_APPLY_DELTAS);
				{
					const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
					const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
					CUresult result = mCudaContext->launchKernel(applyDeltasKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
				}
			}
#endif
		}
	}

	void PxgPBDParticleSystemCore::solveInflatables(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal coefficient, const PxReal dt)
	{
		
		if (mMaxClothsPerBuffer > 0)
		{
			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_INFLATABLE;
			
			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd)
				};

				const CUfunction calculateInflatableKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CALCULATE_INFLATABLE_VOLUME);

				CUresult result = mCudaContext->launchKernel(calculateInflatableKernel, mMaxClothsPerBuffer, mMaxClothBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

			}

			{

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(coefficient)
				};

				const CUfunction solveInflatableKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_INFLATABLE_VOLUME);
				{

					CUresult result = mCudaContext->launchKernel(solveInflatableKernel, mMaxClothsPerBuffer, mMaxClothBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
				}
			}
		}
	}

	void PxgPBDParticleSystemCore::solveShapes(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt, const PxReal biasCoefficient)
	{
		const PxU32 maxRigids = getMaxRigidsPerBuffer();

		if (maxRigids > 0)
		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(biasCoefficient)
			};

			const CUfunction solveShapeKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_SHAPES);
			{
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE_SHAPE;
				CUresult result = mCudaContext->launchKernel(solveShapeKernel, maxRigids, mMaxRigidBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}

#if PS_GPU_DEBUG
			{
				CUresult result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solvesprings kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);
			}
#endif
		}
	}

	void PxgPBDParticleSystemCore::solveAerodynamics(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt)
	{
		if (mMaxTrianglesPerBuffer > 0)
		{
			{
				CUfunction updateAeroDynamic1Function = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_AERODYNAMIC_1);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(dt)
				};

				const PxU32 numBlocks = (mMaxTrianglesPerBuffer + PxgParticleSystemKernelBlockDim::PS_SOLVE - 1) / PxgParticleSystemKernelBlockDim::PS_SOLVE;

				CUresult result = mCudaContext->launchKernel(updateAeroDynamic1Function, numBlocks, mMaxClothBuffersPerSystem, nbActiveParticleSystems, PxgParticleSystemKernelBlockDim::PS_SOLVE, 1, 1, 0, mFinalizeStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);

				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mFinalizeStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solveAerodynamics1Launch kernel fail!\n");
#endif
			}

			{
				CUfunction updateAeroDynamic2Function = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_AERODYNAMIC_2);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(dt)
				};

				const PxU32 numBlocks = (mMaxParticlesPerBuffer + PxgParticleSystemKernelBlockDim::PS_SOLVE - 1) / PxgParticleSystemKernelBlockDim::PS_SOLVE;

				CUresult result = mCudaContext->launchKernel(updateAeroDynamic2Function, numBlocks, mMaxBuffersPerSystem, nbActiveParticleSystems, PxgParticleSystemKernelBlockDim::PS_SOLVE, 1, 1, 0, mFinalizeStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);

				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mFinalizeStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solveAerodynamics2Launch kernel fail!\n");
#endif
			}
		}

	}

	void PxgPBDParticleSystemCore::initializeSprings(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
		const PxU32 nbActiveParticleSystems)
	{

		const PxU32 maxSprings = getMaxSpringsPerBuffer();
		if (maxSprings > 0)
		{

			//update duplicate verts
			{
				//each block has 1024 threads
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND;
				const PxU32 numBlocks = (maxSprings + numThreadsPerBlock - 1) / numThreadsPerBlock;

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
				};

				const CUfunction updateRemapKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_INITIALIZE_SPRINGS);

				CUresult result = mCudaContext->launchKernel(updateRemapKernel, numBlocks, mMaxClothBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if PS_GPU_DEBUG
				{
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_initializeSpringsLaunch kernel fail!\n");
					PX_ASSERT(result == CUDA_SUCCESS);
				}
#endif
			}
		}
	}

	void PxgPBDParticleSystemCore::solveSprings(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
		const PxU32 nbActiveParticleSystems, const PxReal dt, bool isTGS)
	{
		const PxU32 maxParticles = mMaxParticlesPerBuffer;
		const PxU32 maxSprings = getMaxSpringsPerBuffer();

		if (maxSprings > 0)
		{
			//update duplicate verts
			{
				//each block has 1024 threads
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND;

				const PxU32 numBlocks = (maxSprings * 2 + numThreadsPerBlock - 1) / numThreadsPerBlock;

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
				};

				const CUfunction updateRemapKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_REMAP_VERTS);

				CUresult result = mCudaContext->launchKernel(updateRemapKernel, numBlocks, mMaxClothBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if PS_GPU_DEBUG
				{
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateRemapVertsLaunch kernel fail!\n");
					PX_ASSERT(result == CUDA_SUCCESS);
				}
#endif
			}


			const PxReal invDt = 1.f / dt;

			{

				const PxU32 maxPartitions = getMaxSpringPartitionsPerBuffer();
				const PxU32 maxSpringsPerPartitions = getMaxSpringsPerPartitionPerBuffer();
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = (maxSpringsPerPartitions + numThreadsPerBlock - 1) / numThreadsPerBlock;

				for (PxU32 i = 0; i < maxPartitions; ++i)
				{

					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(particleSystemsd),
						PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
						PX_CUDA_KERNEL_PARAM(invDt),
						PX_CUDA_KERNEL_PARAM(i),
						PX_CUDA_KERNEL_PARAM(isTGS)
					};

					const CUfunction solveSpringKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_SPRINGS);

					CUresult result = mCudaContext->launchKernel(solveSpringKernel, numBlocks, mMaxClothBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);


#if PS_GPU_DEBUG
					{
						result = mCudaContext->streamSynchronize(mStream);
						if (result != CUDA_SUCCESS)
							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solveSpringsPGSLaunch kernel fail!\n");
						PX_ASSERT(result == CUDA_SUCCESS);
					}
#endif
				}
			}


			//compute average verts and update sorted positions
			{

				//each block has 1024 threads
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND;
				const PxU32 numBlocks = (maxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
					PX_CUDA_KERNEL_PARAM(invDt)
				};

				const CUfunction computeAverageKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_AVERAGEVERTS);

				CUresult result = mCudaContext->launchKernel(
			        computeAverageKernel, numBlocks, mMaxClothBuffersPerSystem, nbActiveParticleSystems,
			        numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if PS_GPU_DEBUG
				{
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_averageVertsLaunch fail!\n");
					PX_ASSERT(result == CUDA_SUCCESS);
				}
#endif
			}

			//update sorted positions and sorted velocities
			updateSortedVelocity(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt);

		}

	}

	void PxgPBDParticleSystemCore::prepParticleConstraint(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr sharedDescd,
		bool isTGS, const PxReal dt)
	{
		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();
		CUdeviceptr activeParticelSystemd = getActiveParticleSystemBuffer().getDevicePtr();
		const PxU32 nbActiveParticles = mSimController->getBodySimManager().mActivePBDParticleSystems.size();

		CUdeviceptr totalContactCountsd = getParticleContactCount().getDevicePtr();
		CUdeviceptr contactsd = mPrimitiveContactSortedByParticleBuf.getDevicePtr();
		CUdeviceptr constraintsd = mPrimitiveConstraintSortedByParticleBuf.getDevicePtr();
		CUdeviceptr appliedForced = mPrimitiveConstraintAppliedParticleForces.getDevicePtr();
		void* nullContactDeltaV = NULL;

		//prepare primitive constraints sorted by particle id
		{
			const CUfunction prepPrimitiveCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CONTACT_PREPARE);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemd),
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(prepDescd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(isTGS),
				PX_CUDA_KERNEL_PARAM(nullContactDeltaV),
				PX_CUDA_KERNEL_PARAM(sharedDescd)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
			const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
			CUresult result = mCudaContext->launchKernel(prepPrimitiveCollisionKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepParticleConstraint ps_contactPrepareLaunch kernel fail!\n");
			PX_ASSERT(result == CUDA_SUCCESS);
#endif
		}


		//prepare primitive constraints sorted by particle id
		if (mMaxParticles > 0 && mHasNonZeroFluidBoundaryScale)
		{
			const CUfunction staticDensityFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_ACCUMULATE_STATIC_DENSITY);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemd),
				PX_CUDA_KERNEL_PARAM(activeParticelSystemd)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
			const PxU32 numBlocks = (mMaxParticles + PxgParticleSystemKernelBlockDim::PS_COLLISION - 1) / PxgParticleSystemKernelBlockDim::PS_COLLISION;
			CUresult result = mCudaContext->launchKernel(staticDensityFunction, numBlocks, nbActiveParticles, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepParticleConstraint ps_contactPrepareLaunch kernel fail!\n");
			PX_ASSERT(result == CUDA_SUCCESS);
#endif
		}

		if (mMaxParticles > 0 && mHasNonZeroFluidBoundaryScale)
		{

			CUdeviceptr pairCountd = mTempHistogramCountBuf.getDevicePtr();
			CUdeviceptr startd = mTempContactBuf.getDevicePtr();
			CUdeviceptr	endd = mTempContactRemapBuf.getDevicePtr();

			const CUfunction rigidDensityFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_ACCUMULATE_RIGID_DENSITY);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemd),
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(pairCountd),
				PX_CUDA_KERNEL_PARAM(startd),
				PX_CUDA_KERNEL_PARAM(endd)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
			const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
			CUresult result = mCudaContext->launchKernel(rigidDensityFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepParticleConstraint ps_contactPrepareLaunch kernel fail!\n");
			PX_ASSERT(result == CUDA_SUCCESS);
#endif
		}

	}


	void PxgPBDParticleSystemCore::solveDiffuseParticles(
		CUdeviceptr particleSystemsd, 
		CUdeviceptr activeParticleSystemd, 
		const PxU32 nbActiveParticleSystems, 
		const PxReal dt)
	{
		if (mMaxDiffusePerBuffer > 0)
		{

			static PxU32 count = 0;
			count++;
			// Handle one-way collision
			// Everything is one-way collision, but to match/reuse the particle-shape collision detection pipeline,
			// mesh/height-field are piped thru a different kernel.
			{
				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_DIFFUSE_PARTICLES_ONE_WAY_COLLISION);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(count)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
				const PxU32 numBlocks = (mMaxDiffuseParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_diffuseParticleOneWayCollision kernel fail!\n");
#endif
			}

			// Update and write to diffuse buffer
			{
				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_DIFFUSE_PARTICLES_UPDATE_PBF);
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(mGravity),
					PX_CUDA_KERNEL_PARAM(dt)
				};
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
				const PxU32 numBlocks = (mMaxDiffuseParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_diffuseParticleUpdatePBF kernel fail!\n");
#endif
			}

			// Update and write to diffuse buffer
			{
				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_DIFFUSE_PARTICLES_COMPACT);
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(mGravity),
					PX_CUDA_KERNEL_PARAM(dt)
				};
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
				const PxU32 numBlocks = (mMaxDiffusePerBuffer + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, mMaxDiffuseBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_diffuseParticleUpdatePBF kernel fail!\n");
#endif
			}

			// Create
			{
				CUdeviceptr randomTabled = mDiffuseParticlesRandomTableBuf.getDevicePtr();

				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_DIFFUSE_PARTICLES_CREATE);
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(randomTabled),
					PX_CUDA_KERNEL_PARAM(mRandomTableSize),
					PX_CUDA_KERNEL_PARAM(dt)
				};
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
				const PxU32 numBlocks = (mMaxParticlesPerBuffer + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, mMaxBuffersPerSystem, nbActiveParticleSystems, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_diffuseParticleCreate kernel fail!\n");
#endif
			}

			// Copy number of active particles
			{
				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_DIFFUSE_PARTICLES_COPY);
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(count)
				};
				CUresult result = mCudaContext->launchKernel(kernelFunction, 1, mMaxDiffuseBuffersPerSystem, nbActiveParticleSystems, 1, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_diffuseParticleCopy kernel fail!\n");
#endif

			}

			// Copy number of active particles
			{
				const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_DIFFUSE_PARTICLES_SUM);
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(count)
				};
				CUresult result = mCudaContext->launchKernel(kernelFunction, nbActiveParticleSystems, 1, 1, 32, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG 
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_diffuseParticleCopy kernel fail!\n");
#endif
			}
		}
	}
	
	void PxgPBDParticleSystemCore::updateMaterials(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, CUstream bpStream,
		const PxReal invTotalDt)
	{
		if (mMaxNumPhaseToMaterials && mMaxParticles)
		{
			PxgGpuNarrowphaseCore* npCore = mGpuContext->getNarrowphaseCore();
			CUdeviceptr pbdMaterialsd = npCore->mGpuPBDMaterialManager.mGpuMaterialBuffer.getDevicePtr();

			// Update materials
			const CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_MATERIALS);
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
				PX_CUDA_KERNEL_PARAM(pbdMaterialsd),
				PX_CUDA_KERNEL_PARAM(invTotalDt)
			};
			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
			const PxU32 numBlocks = (mMaxNumPhaseToMaterials + numThreadsPerBlock - 1) / numThreadsPerBlock;
			CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(bpStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateMaterials kernel fail!\n");
#endif
		}
	}

	void PxgPBDParticleSystemCore::allocateParticleBuffer(const PxU32 nbTotalParticleSystems, CUstream stream)
	{
		PX_PROFILE_ZONE("AllocateParticleBuffer", 0);
		if (nbTotalParticleSystems > mNbTotalParticleSystems)
		{
			
			mParticleSystemDataBuffer.resize(nbTotalParticleSystems);
			mDiffuseParticleDataBuffer.resize(nbTotalParticleSystems);
			
			const PxU64 oldCapacity = mParticleSystemBuffer.getSize();

			mParticleSystemBuffer.allocateCopyOldDataAsync(nbTotalParticleSystems * sizeof(PxgParticleSystem), mCudaContext, stream, PX_FL);			

			if (oldCapacity < mParticleSystemBuffer.getSize())
			{
				mCudaContext->memsetD32Async(mParticleSystemBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mParticleSystemBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);
			}

			mParticleSystemPool.resize(nbTotalParticleSystems);
			mParticleSystemNodeIndexPool.resize(nbTotalParticleSystems);


			mNbTotalParticleSystems = nbTotalParticleSystems;
		}
	}

	struct Comparer
	{
		const PxArray<PxU32>& mValues;

		Comparer(const PxArray<PxU32>& values) : mValues(values) {}

		bool operator()(const PxU32& a, const PxU32& b) const
		{
			return mValues[a] < mValues[b];
		}

	private:
		PX_NOCOPY(Comparer)
	};

	template<class PxsBufferType>
	void initParticleSimBuffer(PxgParticleSimBuffer& simBuffer, const PxgParticleBufferBase<PxsBufferType>& userBuffer)
	{
		simBuffer.mPositionInvMasses = reinterpret_cast<float4*>(userBuffer.mPositionInvMassesD);
		simBuffer.mVelocities = reinterpret_cast<float4*>(userBuffer.mVelocitiesD);
		simBuffer.mPhases = userBuffer.mPhasesD;
		simBuffer.mRestPositions = NULL;
		simBuffer.mNumActiveParticles = userBuffer.mNumActiveParticles;
		simBuffer.mFlags = userBuffer.mBufferFlags;
		simBuffer.mVolumes = reinterpret_cast<PxParticleVolume*>(userBuffer.mVolumesD);
		simBuffer.mNumVolumes = userBuffer.mNumParticleVolumes;
		simBuffer.mFilterPairs = userBuffer.getRigidFilters();
		simBuffer.mNumFilterPairs = userBuffer.getNbRigidFilters();
		simBuffer.mRigidAttachments = userBuffer.getRigidAttachments();
		simBuffer.mNumRigidAttachments = userBuffer.getNbRigidAttachments();
		simBuffer.mDiffuseParticleBufferIndex = 0xffffffff;
		simBuffer.mUniqueId = userBuffer.mUniqueId;
	}

	//create particle position buffers and create runsum for the buffer
    bool PxgPBDParticleSystemCore::createUserParticleData(PxgParticleSystem& particleSystem, Dy::ParticleSystemCore& dyParticleSystemCore, PxgParticleSystemBuffer* buffer, PxgParticleSystemDiffuseBuffer* diffuseParticlesBuffer, CUstream stream)
	{
		const PxU32 numPBDParticleBuffer = dyParticleSystemCore.mParticleBuffers.size();
		
		//mPositionInvMasses/mVelocities/mPhase need to be in device memory
		PxgParticleBuffer*const * srcParticleBuffers = reinterpret_cast<PxgParticleBuffer* const*>( dyParticleSystemCore.mParticleBuffers.begin());
		PxgParticleAndDiffuseBuffer* const* srcParticleDiffuseBuffers = reinterpret_cast<PxgParticleAndDiffuseBuffer* const*>(dyParticleSystemCore.mParticleDiffuseBuffers.begin());
		PxgParticleClothBuffer* const* srcParticleClothBuffers = reinterpret_cast<PxgParticleClothBuffer* const*>(dyParticleSystemCore.mParticleClothBuffers.begin());
		PxgParticleRigidBuffer* const* srcParticleRigidBuffers = reinterpret_cast<PxgParticleRigidBuffer* const*>(dyParticleSystemCore.mParticleRigidBuffers.begin());
		bool anyDirty = false;

		/*
		* Note for resizing: There is a circular dependency between the total particle count and the buffer updates,
		* as for any resized particle system, all the buffer flags need to be raised to make sure the data is copied
		* into the new buffer.
		*
		* Luckily, resizing is always connected to a buffer add/remove. So this will always trigger the forced update,
		* and thus all the flags will be raised no matter what in any case where resizing would actually be needed. So
		* we can defer the actual resizing to a time when we know the new particle count.
		*/

		// if the buffer size changed(add/remove buffer, this mParticleBufferUpdate will get raised)
	    bool forceUpdate = false;
	    if(dyParticleSystemCore.mParticleBufferUpdate)
	    {
		    anyDirty = true;
		    forceUpdate = true;
		    dyParticleSystemCore.mParticleBufferUpdate = false;
	    }

		// if the buffer size changed(add/remove buffer, this mClothBufferUpdate will get raised)
	    if(dyParticleSystemCore.mParticleClothBufferUpdate)
	    {
		    anyDirty = true;
		    forceUpdate = true;
		    dyParticleSystemCore.mParticleClothBufferUpdate = false;
	    }

		// if the buffer size changed(add/remove buffer, this mRigidBufferUpdate will get raised)
	    if(dyParticleSystemCore.mParticleRigidBufferUpdate)
	    {
		    anyDirty = true;
		    forceUpdate = true;
		    dyParticleSystemCore.mParticleRigidBufferUpdate = false;
	    }

	    if(dyParticleSystemCore.mParticleDiffuseBufferUpdate)
	    {
		    anyDirty |= true;
		    forceUpdate = true;
		    dyParticleSystemCore.mParticleDiffuseBufferUpdate = false;
	    }


		for (PxU32 i = 0; i < numPBDParticleBuffer; ++i)
		{
			PxgParticleBuffer& srcBuffer = *srcParticleBuffers[i];

			if(forceUpdate)
		    {
				srcBuffer.mBufferFlags |=
			        (PxParticleBufferFlag::eUPDATE_POSITION | PxParticleBufferFlag::eUPDATE_VELOCITY |
						PxParticleBufferFlag::eUPDATE_PHASE);
		    }

			anyDirty |= ((srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_PHASE) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_POSITION)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_VELOCITY) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_ATTACHMENTS));
		}

		const PxU32 numPBDClothBuffer = dyParticleSystemCore.mParticleClothBuffers.size();
		for (PxU32 i = 0; i < numPBDClothBuffer; ++i)
		{
			PxgParticleClothBuffer& srcBuffer = *srcParticleClothBuffers[i];

		    if(forceUpdate)
		    {
				srcBuffer.mBufferFlags |=
			        (PxParticleBufferFlag::eUPDATE_PHASE | PxParticleBufferFlag::eUPDATE_POSITION |
						PxParticleBufferFlag::eUPDATE_VELOCITY | PxParticleBufferFlag::eUPDATE_RESTPOSITION |
						PxParticleBufferFlag::eUPDATE_CLOTH);
		    }

			anyDirty |= ((srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_PHASE) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_POSITION)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_VELOCITY)) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_RESTPOSITION)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_CLOTH) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_ATTACHMENTS);
		}

		const PxU32 numRigidBuffer = dyParticleSystemCore.mParticleRigidBuffers.size();
		for (PxU32 i = 0; i < numRigidBuffer; ++i)
		{
			PxgParticleRigidBuffer& srcBuffer = *srcParticleRigidBuffers[i];

		    if(forceUpdate)
		    {
				srcBuffer.mBufferFlags |= (PxParticleBufferFlag::eUPDATE_PHASE | PxParticleBufferFlag::eUPDATE_POSITION |
				                      PxParticleBufferFlag::eUPDATE_VELOCITY | PxParticleBufferFlag::eUPDATE_RESTPOSITION |
									  PxParticleBufferFlag::eUPDATE_RIGID);
		    }

			anyDirty |= ((srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_PHASE) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_POSITION)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_VELOCITY)) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_RESTPOSITION)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_RIGID) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_ATTACHMENTS);
		}

		const PxU32 numDiffuseBuffer = dyParticleSystemCore.mParticleDiffuseBuffers.size();
		for (PxU32 i = 0; i < numDiffuseBuffer; ++i)
		{
			PxgParticleAndDiffuseBuffer& srcBuffer = *srcParticleDiffuseBuffers[i];

		    if(forceUpdate)
		    {
				srcBuffer.mBufferFlags |=
					(PxParticleBufferFlag::eUPDATE_POSITION | PxParticleBufferFlag::eUPDATE_VELOCITY |
					 PxParticleBufferFlag::eUPDATE_PHASE | PxParticleBufferFlag::eUPDATE_DIFFUSE_PARAM);
		    }

			anyDirty |= ((srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_PHASE) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_POSITION)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_VELOCITY) || (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_DIFFUSE_PARAM)
				|| (srcBuffer.mBufferFlags & PxParticleBufferFlag::eUPDATE_ATTACHMENTS));
		}
		
		if (anyDirty)
		{
			const PxU32 totalParticleBufferSize = dyParticleSystemCore.getNumUserBuffers();
			mMaxBuffersPerSystem = PxMax(mMaxBuffersPerSystem, totalParticleBufferSize);
			mMaxClothBuffersPerSystem = PxMax(mMaxClothBuffersPerSystem, numPBDClothBuffer);
			mMaxRigidBuffersPerSystem = PxMax(mMaxRigidBuffersPerSystem, numRigidBuffer);
			mMaxDiffuseBuffersPerSystem = PxMax(mMaxDiffuseBuffersPerSystem, numDiffuseBuffer);

			if (buffer->mHostParticleBuffers.size() < totalParticleBufferSize)
			{
				buffer->mHostParticleBuffers.reserve(totalParticleBufferSize);
				buffer->mHostParticleBuffers.forceSize_Unsafe(totalParticleBufferSize);
				buffer->mParticleBufferRunSum.reserve(totalParticleBufferSize);
				buffer->mParticleBufferRunSum.forceSize_Unsafe(totalParticleBufferSize);
				buffer->mAttachmentRunSum.reserve(totalParticleBufferSize);
				buffer->mAttachmentRunSum.forceSize_Unsafe(totalParticleBufferSize);
				buffer->mReferencedRigidsRunsum.reserve(totalParticleBufferSize);
				buffer->mReferencedRigidsRunsum.forceSize_Unsafe(totalParticleBufferSize);

				buffer->mParticleBufferSortedUniqueIds.reserve(totalParticleBufferSize);
				buffer->mParticleBufferSortedUniqueIds.forceSize_Unsafe(totalParticleBufferSize);
				buffer->mParticleBufferSortedUniqueIdsOriginalIndex.reserve(totalParticleBufferSize);
				buffer->mParticleBufferSortedUniqueIdsOriginalIndex.forceSize_Unsafe(totalParticleBufferSize);
			}

			if (buffer->mHostClothBuffers.size() < numPBDClothBuffer)
			{
				buffer->mHostClothBuffers.reserve(numPBDClothBuffer);
				buffer->mHostClothBuffers.forceSize_Unsafe(numPBDClothBuffer);
			}

			if (buffer->mHostRigidBuffers.size() < numRigidBuffer)
			{
				buffer->mHostRigidBuffers.reserve(numRigidBuffer);
				buffer->mHostRigidBuffers.forceSize_Unsafe(numRigidBuffer);
			}

			if (buffer->mHostDiffuseBuffers.size() < numDiffuseBuffer)
			{
				buffer->mHostDiffuseBuffers.reserve(numDiffuseBuffer);
				buffer->mHostDiffuseBuffers.forceSize_Unsafe(numDiffuseBuffer);
			}

			PxU32 runSumCount = 0;
			PxU32 numActiveParticles = 0;
			PxU32 numMaxParticles = 0;
			PxU32 numMaxDiffuseParticles = 0;
			PxU32 attachmentRunsum = 0;
			PxU32 referencedRigidsRunsum = 0;
			PxArray<PxU32> uniqueIds;
			uniqueIds.reserve(totalParticleBufferSize);
			//compute runsum
			PxU32 hostParticleBufferIndex = 0;
			for (PxU32 i = 0; i < numPBDParticleBuffer; ++i)
			{
				PxgParticleBuffer& srcBuffer = *srcParticleBuffers[i];
				PxgParticleSimBuffer& pinnedBuffer = buffer->mHostParticleBuffers[hostParticleBufferIndex];
				initParticleSimBuffer(pinnedBuffer, srcBuffer);
				uniqueIds.pushBack(srcBuffer.mUniqueId);

				numActiveParticles += srcBuffer.mNumActiveParticles;
				numMaxParticles += srcBuffer.mMaxNumParticles;

				buffer->mAttachmentRunSum[hostParticleBufferIndex] = attachmentRunsum;
				buffer->mReferencedRigidsRunsum[hostParticleBufferIndex] = referencedRigidsRunsum;
				buffer->mParticleBufferRunSum[hostParticleBufferIndex++] = runSumCount;
				srcBuffer.setFlatListStartIndex(runSumCount);

				runSumCount += srcBuffer.mNumActiveParticles;
				attachmentRunsum += srcBuffer.mNumRigidAttachments;

				mMaxParticlesPerBuffer = PxMax(mMaxParticlesPerBuffer, srcBuffer.mNumActiveParticles);
				mMaxVolumesPerBuffer = PxMax(mMaxVolumesPerBuffer, srcBuffer.mNumParticleVolumes);
				srcBuffer.mBufferFlags = 0;
			}

			for (PxU32 i = 0; i < numPBDClothBuffer; ++i)
			{
				PxgParticleClothBuffer& srcBuffer = *srcParticleClothBuffers[i];
				PxgParticleSimBuffer& pinnedBuffer = buffer->mHostParticleBuffers[hostParticleBufferIndex];
				initParticleSimBuffer(pinnedBuffer, srcBuffer);
				pinnedBuffer.mRestPositions = reinterpret_cast<float4*>(srcBuffer.mRestPositionsD);
				uniqueIds.pushBack(srcBuffer.mUniqueId);

				PxgParticleClothSimBuffer& clothBuffer = buffer->mHostClothBuffers[i];
				clothBuffer.mAccumulatedCopiesPerParticles = srcBuffer.mAccumulatedCopiesPerParticlesD;
				clothBuffer.mAccumulatedSpringsPerPartitions = srcBuffer.mAccumulatedSpringsPerPartitionsD;
				clothBuffer.mRemapOutput = srcBuffer.mRemapOutputD;
				clothBuffer.mOrderedSprings = srcBuffer.mOrderedSpringsD;
				clothBuffer.mTriangles = srcBuffer.mTriangleIndicesD;
				clothBuffer.mSortedClothStartIndices = srcBuffer.mSortedClothStartIndicesD;
				clothBuffer.mCloths = srcBuffer.mClothsD;
				
				clothBuffer.mParticleBufferIndex = hostParticleBufferIndex; //this is the index with particle buffer
				clothBuffer.mNumSprings = srcBuffer.mNumSprings;
				clothBuffer.mNumPartitions = srcBuffer.mNumPartitions;
				clothBuffer.mNumCloths = srcBuffer.mNumCloths;
				clothBuffer.mNumTriangles = srcBuffer.mNumTriangles;

				clothBuffer.mRemapPositions = reinterpret_cast<float4*>(srcBuffer.mRemapPositionsD);
				clothBuffer.mRemapVelocities = reinterpret_cast<float4*>(srcBuffer.mRemapVelocitiesD);
				clothBuffer.mSpringLambda = srcBuffer.mSpringLambdaD;
				clothBuffer.mInflatableLambda = srcBuffer.mInflatableLambdaD;
				numActiveParticles += srcBuffer.mNumActiveParticles;
				numMaxParticles += srcBuffer.mMaxNumParticles;

				buffer->mAttachmentRunSum[hostParticleBufferIndex] = attachmentRunsum;
				buffer->mReferencedRigidsRunsum[hostParticleBufferIndex] = referencedRigidsRunsum;
				buffer->mParticleBufferRunSum[hostParticleBufferIndex++] = runSumCount;
				srcBuffer.setFlatListStartIndex(runSumCount);
				
				runSumCount += srcBuffer.mNumActiveParticles;
				attachmentRunsum += srcBuffer.mNumRigidAttachments;
				mMaxParticlesPerBuffer = PxMax(mMaxParticlesPerBuffer, srcBuffer.mNumActiveParticles);
				mMaxVolumesPerBuffer = PxMax(mMaxVolumesPerBuffer, srcBuffer.mNumParticleVolumes);
				mMaxClothsPerBuffer = PxMax(mMaxClothsPerBuffer, srcBuffer.mNumCloths);
				mMaxSpringsPerBuffer = PxMax(mMaxSpringsPerBuffer, srcBuffer.mNumSprings);
				mMaxSpringPartitionsPerBuffer = PxMax(mMaxSpringPartitionsPerBuffer, srcBuffer.mNumPartitions);
				mMaxSpringsPerPartitionPerBuffer = PxMax(mMaxSpringsPerPartitionPerBuffer, srcBuffer.mMaxSpringsPerPartition);
				mMaxTrianglesPerBuffer = PxMax(mMaxTrianglesPerBuffer, srcBuffer.mNumTriangles);
								
				srcBuffer.mBufferFlags = 0;
			}

			for (PxU32 i = 0; i < numRigidBuffer; ++i)
			{
				PxgParticleRigidBuffer& srcBuffer = *srcParticleRigidBuffers[i];
				PxgParticleSimBuffer& pinnedBuffer = buffer->mHostParticleBuffers[hostParticleBufferIndex];
				initParticleSimBuffer(pinnedBuffer, srcBuffer);
				uniqueIds.pushBack(srcBuffer.mUniqueId);

				PxgParticleRigidSimBuffer& rigidBuffer = buffer->mHostRigidBuffers[i];
				rigidBuffer.mRigidCoefficients = srcBuffer.mRigidCoefficientsD;
				rigidBuffer.mRigidLocalPositions = reinterpret_cast<float4*>(srcBuffer.mRigidLocalPositionsD);
				rigidBuffer.mRigidLocalNormals = reinterpret_cast<float4*>(srcBuffer.mRigidLocalNormalsD);
				rigidBuffer.mRigidRotations = reinterpret_cast<float4*>(srcBuffer.mRigidRotationsD);
				rigidBuffer.mRigidTranslations = reinterpret_cast<float4*>(srcBuffer.mRigidTranslationsD);
				rigidBuffer.mRigidOffsets = srcBuffer.mRigidOffsetsD;
				rigidBuffer.mParticleBufferIndex = hostParticleBufferIndex;
				rigidBuffer.mNumRigids = srcBuffer.mNumActiveRigids;
				
				numActiveParticles += srcBuffer.mNumActiveParticles;
				numMaxParticles += srcBuffer.mMaxNumParticles;

				buffer->mAttachmentRunSum[hostParticleBufferIndex] = attachmentRunsum;
				buffer->mReferencedRigidsRunsum[hostParticleBufferIndex] = referencedRigidsRunsum;
				buffer->mParticleBufferRunSum[hostParticleBufferIndex++] = runSumCount;
				srcBuffer.setFlatListStartIndex(runSumCount);

				runSumCount += srcBuffer.mNumActiveParticles;
				attachmentRunsum += srcBuffer.mNumRigidAttachments;
				// AD: rigid runsum should probably be incremented here?
				mMaxParticlesPerBuffer = PxMax(mMaxParticlesPerBuffer, srcBuffer.mNumActiveParticles);
				mMaxVolumesPerBuffer = PxMax(mMaxVolumesPerBuffer, srcBuffer.mNumParticleVolumes);
				mMaxRigidsPerBuffer = PxMax(mMaxRigidsPerBuffer, srcBuffer.mNumActiveRigids);

				srcBuffer.mBufferFlags = 0;
			}

			for (PxU32 i = 0; i < numDiffuseBuffer; ++i)
			{
				PxgParticleAndDiffuseBuffer& srcBuffer = *srcParticleDiffuseBuffers[i];
				PxgParticleSimBuffer& pinnedBuffer = buffer->mHostParticleBuffers[hostParticleBufferIndex];
				initParticleSimBuffer(pinnedBuffer, srcBuffer);
				pinnedBuffer.mDiffuseParticleBufferIndex = i;
				uniqueIds.pushBack(srcBuffer.mUniqueId);

				PxgParticleDiffuseSimBuffer& diffuseBuffer = buffer->mHostDiffuseBuffers[i];
				diffuseBuffer.mMaxNumParticles = srcBuffer.mMaxActiveDiffuseParticles;
				diffuseBuffer.mParams = srcBuffer.mParams;
				diffuseBuffer.mDiffusePositions_LifeTime = reinterpret_cast<float4*>(srcBuffer.mDiffusePositionsLifeTimeD);
				diffuseBuffer.mDiffuseVelocities = reinterpret_cast<float4*>(srcBuffer.mDiffuseVelocitiesD);
				diffuseBuffer.mNumDiffuseParticles = srcBuffer.mNumDiffuseParticlesD;
				diffuseBuffer.mFlags = srcBuffer.mBufferFlags;
				diffuseBuffer.mStartIndex = 0;
				diffuseBuffer.mNumActiveDiffuseParticles = srcBuffer.mNumActiveDiffuseParticlesH;
				
				numActiveParticles += srcBuffer.mNumActiveParticles;
				numMaxParticles += srcBuffer.mMaxNumParticles;
				numMaxDiffuseParticles += diffuseBuffer.mMaxNumParticles;

				buffer->mAttachmentRunSum[hostParticleBufferIndex] = attachmentRunsum;
				buffer->mReferencedRigidsRunsum[hostParticleBufferIndex] = referencedRigidsRunsum;
				buffer->mParticleBufferRunSum[hostParticleBufferIndex++] = runSumCount;
				srcBuffer.setFlatListStartIndex(runSumCount);

				runSumCount += srcBuffer.mNumActiveParticles;
				attachmentRunsum += srcBuffer.mNumRigidAttachments;
				mMaxParticlesPerBuffer = PxMax(mMaxParticlesPerBuffer, srcBuffer.mNumActiveParticles);
				mMaxVolumesPerBuffer = PxMax(mMaxVolumesPerBuffer, srcBuffer.mNumParticleVolumes);
				mMaxDiffusePerBuffer = PxMax(mMaxDiffusePerBuffer, srcBuffer.mMaxActiveDiffuseParticles);

				srcBuffer.mBufferFlags = 0;
			}

		    if(numMaxParticles > particleSystem.mCommonData.mMaxParticles)
		    {
				resizeParticleDataBuffer(particleSystem, buffer, numMaxParticles, particleSystem.mCommonData.mMaxNeighborhood, stream);

			    resizeDiffuseParticleParticleBuffers(particleSystem, diffuseParticlesBuffer, numMaxParticles);
		    }

			if(numMaxDiffuseParticles > particleSystem.mCommonData.mMaxDiffuseParticles)
			{
				resizeDiffuseParticleDiffuseBuffer(particleSystem, diffuseParticlesBuffer, numMaxDiffuseParticles, stream);
			}

			const PxU32 byteSize1 = sizeof(PxgParticleSimBuffer) * totalParticleBufferSize;
			buffer->user_particle_buffer.allocate(byteSize1, PX_FL);
			const PxU32 byteSize2 = sizeof(PxU32) * totalParticleBufferSize;
			buffer->user_particle_buffer_runsum.allocate(byteSize2, PX_FL);
			buffer->user_particle_buffer_sorted_unique_ids.allocate(byteSize2, PX_FL);
			buffer->user_particle_buffer_runsum_sorted_unique_ids_original_index.allocate(byteSize2, PX_FL);
			buffer->attachmentRunSum.allocate(byteSize2, PX_FL);
			buffer->referencedRigidsRunsum.allocate(byteSize2, PX_FL);

			if (numPBDClothBuffer > 0)
			{
				const PxU32 byteSize3 = sizeof(PxgParticleClothSimBuffer) * numPBDClothBuffer;
				buffer->user_cloth_buffer.allocate(byteSize3, PX_FL);
				mCudaContext->memcpyHtoDAsync(buffer->user_cloth_buffer.getDevicePtr(), buffer->mHostClothBuffers.begin(), byteSize3, stream);
			}

			if (numRigidBuffer > 0)
			{
				const PxU32 byteSize4 = sizeof(PxgParticleRigidSimBuffer) * numRigidBuffer;
				buffer->user_rigid_buffer.allocate(byteSize4, PX_FL);
				mCudaContext->memcpyHtoDAsync(buffer->user_rigid_buffer.getDevicePtr(), buffer->mHostRigidBuffers.begin(), byteSize4, stream);
			}

			if (numDiffuseBuffer > 0)
			{
				const PxU32 byteSize = sizeof(PxgParticleDiffuseSimBuffer) * numDiffuseBuffer;
				buffer->user_diffuse_buffer.allocate(byteSize, PX_FL);
		
				mCudaContext->memcpyHtoDAsync(buffer->user_diffuse_buffer.getDevicePtr(), buffer->mHostDiffuseBuffers.begin(), byteSize, stream);
			}

			for(PxU32 i = 0; i < uniqueIds.size(); ++i)
				buffer->mParticleBufferSortedUniqueIdsOriginalIndex[i] = i;
		    PxSort(buffer->mParticleBufferSortedUniqueIdsOriginalIndex.begin(), uniqueIds.size(), Comparer(uniqueIds));
			for (PxU32 i = 0; i < uniqueIds.size(); ++i)
				buffer->mParticleBufferSortedUniqueIds[i] = uniqueIds[buffer->mParticleBufferSortedUniqueIdsOriginalIndex[i]];

			mCudaContext->memcpyHtoDAsync(buffer->user_particle_buffer.getDevicePtr(), buffer->mHostParticleBuffers.begin(), byteSize1, stream);
			mCudaContext->memcpyHtoDAsync(buffer->user_particle_buffer_runsum.getDevicePtr(), buffer->mParticleBufferRunSum.begin(), byteSize2, stream);
			mCudaContext->memcpyHtoDAsync(buffer->attachmentRunSum.getDevicePtr(), buffer->mAttachmentRunSum.begin(), byteSize2, stream);
			mCudaContext->memcpyHtoDAsync(buffer->user_particle_buffer_sorted_unique_ids.getDevicePtr(), buffer->mParticleBufferSortedUniqueIds.begin(), byteSize2, stream);
			mCudaContext->memcpyHtoDAsync(buffer->user_particle_buffer_runsum_sorted_unique_ids_original_index.getDevicePtr(), buffer->mParticleBufferSortedUniqueIdsOriginalIndex.begin(), byteSize2, stream);


			mCudaContext->memcpyHtoDAsync(buffer->referencedRigidsRunsum.getDevicePtr(), buffer->mReferencedRigidsRunsum.begin(), byteSize2, stream);

			particleSystem.mParticleSimBuffers = reinterpret_cast<PxgParticleSimBuffer*>(buffer->user_particle_buffer.getDevicePtr());
			particleSystem.mParticleBufferRunsum = reinterpret_cast<PxU32*>(buffer->user_particle_buffer_runsum.getDevicePtr());
			particleSystem.mClothSimBuffers = reinterpret_cast<PxgParticleClothSimBuffer*>(buffer->user_cloth_buffer.getDevicePtr());
			particleSystem.mRigidSimBuffers = reinterpret_cast<PxgParticleRigidSimBuffer*>(buffer->user_rigid_buffer.getDevicePtr());
			particleSystem.mDiffuseSimBuffers = reinterpret_cast<PxgParticleDiffuseSimBuffer*>(buffer->user_diffuse_buffer.getDevicePtr());
			particleSystem.mParticleBufferSortedUniqueIds = reinterpret_cast<PxU32*>(buffer->user_particle_buffer_sorted_unique_ids.getDevicePtr());
			particleSystem.mParticleBufferSortedUniqueIdsOriginalIndex = reinterpret_cast<PxU32*>(buffer->user_particle_buffer_runsum_sorted_unique_ids_original_index.getDevicePtr());

			particleSystem.mCommonData.mNumParticles = numActiveParticles;
			particleSystem.mCommonData.mMaxParticles = numMaxParticles;
			particleSystem.mCommonData.mNumParticleBuffers = totalParticleBufferSize;
			particleSystem.mCommonData.mMaxDiffuseParticles = numMaxDiffuseParticles;

			particleSystem.mAttachmentRunsum = reinterpret_cast<PxU32*>(buffer->attachmentRunSum.getDevicePtr());

			particleSystem.mNumRigidAttachments = attachmentRunsum;

			particleSystem.mNumClothBuffers = numPBDClothBuffer;
			particleSystem.mNumDiffuseBuffers = numDiffuseBuffer;
			particleSystem.mNumRigidBuffers = numRigidBuffer;
		} 

		mMaxRigidAttachmentsPerSystem = PxMax(mMaxRigidAttachmentsPerSystem, particleSystem.mNumRigidAttachments);

		return anyDirty;
	}

	void PxgPBDParticleSystemCore::resizeParticleDataBuffer(PxgParticleSystem& particleSystem, PxgParticleSystemBuffer* buffer, const PxU32 maxParticles,
		const PxU32 maxNeighborhood, CUstream stream)
	{
		buffer->grid_particle_hash.allocate(maxParticles * sizeof(PxU32), PX_FL);
		buffer->grid_particle_index.allocate(maxParticles * sizeof(PxU32), PX_FL);
		buffer->sorted_position_mass.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->sorted_originalPosition_mass.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->sortedPhaseArray.allocate(maxParticles * sizeof(PxU32), PX_FL);
		buffer->sorted_velocity.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->accumDeltaV.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->sortedDeltaP.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->originalPosition_mass.allocate(maxParticles * sizeof(PxVec4), PX_FL);

		buffer->density.allocate(maxParticles * sizeof(PxReal), PX_FL);
		buffer->staticDensity.allocate(maxParticles * sizeof(PxReal), PX_FL);
		buffer->surfaceNormal.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->delta.allocate(maxParticles * sizeof(PxVec4), PX_FL);
		buffer->curl.allocate(maxParticles * sizeof(PxVec4), PX_FL);

		// delta needs to be cleared
		mCudaContext->memsetD32Async(buffer->delta.getDevicePtr(), 0, maxParticles * sizeof(PxVec4) / 4, stream);

		//Static contacts consist of a collision plane (normal + distance), a node index and a surface velocity.
		//The node index is used to produce the surface velocities...
		const PxU32 maxOneWayContacts = maxParticles * PxgParticleContactInfo::MaxStaticContactsPerParticle;
		buffer->particleOneWayContacts.allocate(sizeof(PxgParticleContactInfo) * maxOneWayContacts, PX_FL);
		buffer->particleOneWayForces.allocate(sizeof(float2) * maxOneWayContacts, PX_FL);
		buffer->particleOneWayContactsNodeIndices.allocate(sizeof(PxNodeIndex) * maxOneWayContacts, PX_FL);
		buffer->particleOneWayContactsSurfaceVelocities.allocate(sizeof(float4) * maxOneWayContacts, PX_FL);
		buffer->particleOneWayContactCount.allocate(sizeof(PxU32) * maxParticles, PX_FL);

		buffer->reverseLookup.allocate(sizeof(PxU32) * maxParticles, PX_FL);

		buffer->collision_headers.allocate(maxParticles * sizeof(PxgParticleCollisionHeader), PX_FL);
		buffer->collision_counts.allocate(maxParticles * sizeof(PxU32), PX_FL);
		buffer->collision_index.allocate(maxParticles * maxNeighborhood * sizeof(PxU32), PX_FL);
		buffer->collision_impulses.allocate(maxParticles * maxNeighborhood * sizeof(float2), PX_FL);

		buffer->phases.allocate(maxParticles * sizeof(PxU32), PX_FL);
		buffer->unsortedpositions.allocate(maxParticles * sizeof(float4), PX_FL);
		buffer->unsortedvelocities.allocate(maxParticles * sizeof(float4), PX_FL);
		buffer->restArray.allocate(maxParticles * sizeof(float4), PX_FL);
		buffer->normal.allocate(maxParticles * sizeof(float4), PX_FL);
		
		particleSystem.mGridParticleHash = buffer->grid_particle_hash.getTypedPtr();
		particleSystem.mSortedToUnsortedMapping = buffer->grid_particle_index.getTypedPtr();
		particleSystem.mSortedPositions_InvMass = reinterpret_cast<float4*>(buffer->sorted_position_mass.getTypedPtr());
		particleSystem.mSortedOriginPos_InvMass = reinterpret_cast<float4*>(buffer->sorted_originalPosition_mass.getTypedPtr());
		particleSystem.mSortedPhaseArray = buffer->sortedPhaseArray.getTypedPtr();
		particleSystem.mSortedVelocities = reinterpret_cast<float4*>(buffer->sorted_velocity.getTypedPtr());
		particleSystem.mAccumDeltaP = reinterpret_cast<float4*>(buffer->accumDeltaV.getTypedPtr());
		particleSystem.mSortedDeltaP = reinterpret_cast<float4*>(buffer->sortedDeltaP.getTypedPtr());
		particleSystem.mOriginPos_InvMass = reinterpret_cast<float4*>(buffer->originalPosition_mass.getTypedPtr());

		particleSystem.mDensity = buffer->density.getTypedPtr();
		particleSystem.mStaticDensity = buffer->staticDensity.getTypedPtr();
		particleSystem.mSurfaceNormal = reinterpret_cast<float4*>(buffer->surfaceNormal.getTypedPtr());
		particleSystem.mDelta = reinterpret_cast<float4*>(buffer->delta.getTypedPtr());
		particleSystem.mCurl = reinterpret_cast<float4*>(buffer->curl.getTypedPtr());

		particleSystem.mOneWayContactInfos = buffer->particleOneWayContacts.getTypedPtr();
		particleSystem.mOneWayForces = buffer->particleOneWayForces.getTypedPtr();
		particleSystem.mOneWayNodeIndex = buffer->particleOneWayContactsNodeIndices.getTypedPtr();
		particleSystem.mOneWaySurfaceVelocity = reinterpret_cast<float4*>(buffer->particleOneWayContactsSurfaceVelocities.getTypedPtr());
		particleSystem.mOneWayContactCount = buffer->particleOneWayContactCount.getTypedPtr();

		particleSystem.mUnsortedToSortedMapping = buffer->reverseLookup.getTypedPtr();

		particleSystem.mCollisionHeaders = buffer->collision_headers.getTypedPtr();
		particleSystem.mParticleSelfCollisionCount = buffer->collision_counts.getTypedPtr();
		particleSystem.mCollisionIndex = buffer->collision_index.getTypedPtr();
		particleSystem.mDensityCollisionImpulses = buffer->collision_impulses.getTypedPtr();

		particleSystem.mUnsortedPhaseArray = buffer->phases.getTypedPtr();
		particleSystem.mUnsortedPositions_InvMass = reinterpret_cast<float4*>(buffer->unsortedpositions.getTypedPtr());
		particleSystem.mRestArray = reinterpret_cast<float4*>(buffer->restArray.getTypedPtr());
		particleSystem.mUnsortedVelocities = reinterpret_cast<float4*>(buffer->unsortedvelocities.getTypedPtr());
		particleSystem.mNormalArray = reinterpret_cast<float4*>(buffer->normal.getTypedPtr());

	}

	void PxgPBDParticleSystemCore::resizeDiffuseParticleDiffuseBuffer(PxgParticleSystem& particleSystem, PxgParticleSystemDiffuseBuffer* diffuseBuffer, const PxU32 maxDiffuseParticles, CUstream stream)
	{
		diffuseBuffer->diffuse_positions.allocate(maxDiffuseParticles * sizeof(PxVec4), PX_FL);
		diffuseBuffer->diffuse_velocities.allocate(maxDiffuseParticles * sizeof(PxVec4), PX_FL);

		diffuseBuffer->diffuse_grid_particle_hash.allocate(maxDiffuseParticles * sizeof(PxU32), PX_FL);
		diffuseBuffer->diffuse_sorted_to_unsorted_mapping.allocate(maxDiffuseParticles * sizeof(PxU32), PX_FL);
		diffuseBuffer->diffuse_unsorted_to_sorted_mapping.allocate(maxDiffuseParticles * sizeof(PxU32), PX_FL);
		diffuseBuffer->diffuse_origin_pos_life_time.allocate(maxDiffuseParticles * sizeof(PxVec4), PX_FL);
		diffuseBuffer->diffuse_sorted_pos_life_time.allocate(maxDiffuseParticles * sizeof(PxVec4), PX_FL);
		diffuseBuffer->diffuse_sorted_origin_pos_life_time.allocate(maxDiffuseParticles * sizeof(PxVec4), PX_FL);
		diffuseBuffer->diffuse_sorted_vel.allocate(maxDiffuseParticles * sizeof(PxVec4), PX_FL);


		const PxU32 maxDiffuseOneWayContacts = maxDiffuseParticles * PxgParticleContactInfo::MaxStaticContactsPerParticle;
		diffuseBuffer->diffuse_one_way_contacts.allocate(sizeof(PxgParticleContactInfo)*maxDiffuseOneWayContacts, PX_FL);
		diffuseBuffer->diffuse_one_way_forces.allocate(sizeof(PxReal)*maxDiffuseOneWayContacts, PX_FL);
		diffuseBuffer->diffuse_one_way_contacts_node_indices.allocate(sizeof(PxNodeIndex)*maxDiffuseOneWayContacts, PX_FL);
		diffuseBuffer->diffuse_one_way_contact_count.allocate(sizeof(PxU32)*maxDiffuseParticles, PX_FL);
		mCudaContext->memsetD32Async(diffuseBuffer->diffuse_one_way_contact_count.getDevicePtr(), 0, maxDiffuseParticles, stream);

		if (maxDiffuseParticles > 0)
		{
			particleSystem.mDiffusePosition_LifeTime = reinterpret_cast<float4*>(diffuseBuffer->diffuse_positions.getTypedPtr());
			particleSystem.mDiffuseVelocity = reinterpret_cast<float4*>(diffuseBuffer->diffuse_velocities.getTypedPtr());
			particleSystem.mDiffuseGridParticleHash = diffuseBuffer->diffuse_grid_particle_hash.getTypedPtr();
			particleSystem.mDiffuseSortedToUnsortedMapping = diffuseBuffer->diffuse_sorted_to_unsorted_mapping.getTypedPtr();
			particleSystem.mDiffuseUnsortedToSortedMapping = diffuseBuffer->diffuse_unsorted_to_sorted_mapping.getTypedPtr();
			particleSystem.mDiffuseOriginPos_LifeTime = reinterpret_cast<float4*>(diffuseBuffer->diffuse_origin_pos_life_time.getTypedPtr());
			particleSystem.mDiffuseSortedPos_LifeTime = reinterpret_cast<float4*>(diffuseBuffer->diffuse_sorted_pos_life_time.getTypedPtr());
			particleSystem.mDiffuseSortedOriginPos_LifeTime = reinterpret_cast<float4*>(diffuseBuffer->diffuse_sorted_origin_pos_life_time.getTypedPtr());
			particleSystem.mDiffuseSortedVel = reinterpret_cast<float4*>(diffuseBuffer->diffuse_sorted_vel.getTypedPtr());
			particleSystem.mDiffuseOneWayContactInfos = diffuseBuffer->diffuse_one_way_contacts.getTypedPtr();
			particleSystem.mDiffuseOneWayForces = diffuseBuffer->diffuse_one_way_forces.getTypedPtr();
			particleSystem.mDiffuseOneWayNodeIndex = diffuseBuffer->diffuse_one_way_contacts_node_indices.getTypedPtr();
			particleSystem.mDiffuseOneWayContactCount = diffuseBuffer->diffuse_one_way_contact_count.getTypedPtr();
		}
		else
		{
			particleSystem.mDiffusePosition_LifeTime = NULL;
			particleSystem.mDiffuseVelocity = NULL;
			particleSystem.mDiffuseGridParticleHash = NULL;
			particleSystem.mDiffuseSortedToUnsortedMapping = NULL;
			particleSystem.mDiffuseUnsortedToSortedMapping = NULL;
			particleSystem.mDiffuseOriginPos_LifeTime = NULL;
			particleSystem.mDiffuseSortedPos_LifeTime = NULL;
			particleSystem.mDiffuseSortedOriginPos_LifeTime = NULL;
			particleSystem.mDiffuseSortedVel = NULL;
			particleSystem.mDiffuseOneWayContactInfos = NULL;
			particleSystem.mDiffuseOneWayForces = NULL;
			particleSystem.mDiffuseOneWayNodeIndex = NULL;
			particleSystem.mDiffuseOneWayContactCount = NULL;	
		}
	}
	
	void PxgPBDParticleSystemCore::allocateParticleDataBuffer(void** bodySimsLL, CUstream stream)
	{
		PX_PROFILE_ZONE("AllocateParticleDataBuffer", 0);
		const PxU32 nbNewPBDParticleSystems = mNewParticleSystemPool.size();

		for (PxU32 id = 0; id < nbNewPBDParticleSystems; ++id)
		{
			PxgParticleSystem& newParticleSystem = mNewParticleSystemPool[id];
			PxgParticleSystemData& data = newParticleSystem.mData;
		
			PxgParticleSystem& particleSystem = mParticleSystemPool[data.mRemapIndex];

			PxU32 nodeIndex = mNewParticleSystemNodeIndexPool[id];
			Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[nodeIndex]);
			const Dy::ParticleSystemCore& dyParticleSystemCore = dyParticleSystem->getCore();

			mParticleSystemNodeIndexPool[data.mRemapIndex] = nodeIndex;
			particleSystem.mData = data;
			particleSystem.mCommonData = newParticleSystem.mCommonData;
			particleSystem.mNumClothBuffers = newParticleSystem.mNumClothBuffers;
			particleSystem.mNumRigidBuffers = newParticleSystem.mNumRigidBuffers;
			particleSystem.mNumDiffuseBuffers = newParticleSystem.mNumDiffuseBuffers;

			
			PxgParticleSystemBuffer* buffer = mParticleSystemDataBuffer[data.mRemapIndex];
			PxgParticleSystemDiffuseBuffer* diffuseBuffer = mDiffuseParticleDataBuffer[data.mRemapIndex];
			

			if (!buffer)
			{
				buffer = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgParticleSystemBuffer), "PxgParticleSystemBuffer"), PxgParticleSystemBuffer(mHeapMemoryManager));
				diffuseBuffer = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgParticleSystemDiffuseBuffer), "PxgParticleSystemDiffuseBuffer"), PxgParticleSystemDiffuseBuffer(mHeapMemoryManager));
				
				mParticleSystemDataBuffer[data.mRemapIndex] = buffer;
				mDiffuseParticleDataBuffer[data.mRemapIndex] = diffuseBuffer;
			}

			resizeParticleDataBuffer(particleSystem, buffer, newParticleSystem.mCommonData.mMaxParticles, newParticleSystem.mCommonData.mMaxNeighborhood, stream);

			const PxU32 numCells = newParticleSystem.getNumCells();

			if (particleSystem.mCommonData.mMaxDiffuseParticles > 0)
			{
				//const PxU32 kRandomTableSize = 1024;
				mDiffuseParticlesRandomTableBuf.allocate(mRandomTableSize * sizeof(PxReal), PX_FL);
				CUdeviceptr randomTabled = mDiffuseParticlesRandomTableBuf.getDevicePtr();
				Cm::BasicRandom rnd(42);
				buffer->mRandomTable.resize(mRandomTableSize); //would be nice to deallocate after first frame
				for (PxU32 i = 0; i < mRandomTableSize; ++i)
				{
					buffer->mRandomTable[i] = rnd.randomFloat32() + 0.5f;
				}

				mCudaContext->memcpyHtoDAsync(randomTabled, buffer->mRandomTable.begin(), mRandomTableSize * sizeof(PxReal), stream);
			}
			
			// diffuse particles

			resizeDiffuseParticleParticleBuffers(particleSystem, diffuseBuffer, newParticleSystem.mCommonData.mMaxParticles);
			resizeDiffuseParticleDiffuseBuffer(particleSystem, diffuseBuffer, newParticleSystem.mCommonData.mMaxDiffuseParticles, stream);
			
			diffuseBuffer->diffuse_cell_start.allocate(numCells * sizeof(PxU32), PX_FL);
			diffuseBuffer->diffuse_cell_end.allocate(numCells * sizeof(PxU32), PX_FL);

			if (particleSystem.mCommonData.mMaxDiffuseParticles > 0)
			{
				particleSystem.mDiffuseCellStart = diffuseBuffer->diffuse_cell_start.getTypedPtr();
				particleSystem.mDiffuseCellEnd = diffuseBuffer->diffuse_cell_end.getTypedPtr();
			}
			else
			{
				particleSystem.mDiffuseCellStart = NULL;
				particleSystem.mDiffuseCellEnd = NULL;
			}

			diffuseBuffer->diffuse_particle_count.allocate(sizeof(PxU32), PX_FL);
			mCudaContext->memsetD32Async(diffuseBuffer->diffuse_particle_count.getDevicePtr(), 0, 1, stream);
			particleSystem.mNumDiffuseParticles = reinterpret_cast<int*>(diffuseBuffer->diffuse_particle_count.getTypedPtr());

			// hash grid and others
			buffer->cell_start.allocate(numCells * sizeof(PxU32), PX_FL);
			buffer->cell_end.allocate(numCells * sizeof(PxU32), PX_FL);

			particleSystem.mCellStart = buffer->cell_start.getTypedPtr();
			particleSystem.mCellEnd = buffer->cell_end.getTypedPtr();

			const PxU16* phaseToMaterial = dyParticleSystemCore.mPhaseGroupToMaterialHandle.begin();
			const PxU32 numPhaseToMaterial = dyParticleSystemCore.mPhaseGroupToMaterialHandle.size();
			buffer->phase_group_to_material_handle.allocate(sizeof(PxU16) * numPhaseToMaterial, PX_FL);
			buffer->mHostPhaseGroupToMaterialHandle.resize(numPhaseToMaterial);
			if (numPhaseToMaterial)
			{
				mGpuContext->mGpuNpCore->mapMaterialIndices<PxsPBDMaterialData>(
					buffer->mHostPhaseGroupToMaterialHandle.begin(), phaseToMaterial, numPhaseToMaterial);
				mCudaContext->memcpyHtoDAsync(buffer->phase_group_to_material_handle.getDevicePtr(),
					buffer->mHostPhaseGroupToMaterialHandle.begin(), sizeof(PxU16) * numPhaseToMaterial, stream);
			}
			particleSystem.mParticleMaterials = reinterpret_cast<PxsParticleMaterialData*>(mGpuContext->mGpuNpCore->mGpuPBDMaterialManager.mGpuMaterialBuffer.getDevicePtr());
			particleSystem.mParticleMaterialStride = sizeof(PxsPBDMaterialData);
			particleSystem.mPhaseGroupToMaterialHandle = buffer->phase_group_to_material_handle.getTypedPtr();
			particleSystem.mData.mNumPhaseToMaterials = numPhaseToMaterial;

			mMaxNumPhaseToMaterials = PxMax(mMaxNumPhaseToMaterials, numPhaseToMaterial);
			buffer->derivedPBDMaterialProperties.allocate(sizeof(PxgPBDParticleMaterialDerived) * numPhaseToMaterial, PX_FL); // no need to copy old data
			particleSystem.mDerivedPBDMaterialData = buffer->derivedPBDMaterialProperties.getTypedPtr();
		}
	}

	
	void PxgPBDParticleSystemCore::calculateHashForDiffuseParticles(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 numActiveParticleSystems)
	{
		if (mMaxDiffuseParticles > 0)
		{
			const CUfunction calcHashKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CALCULATE_HASH_FOR_DIFFUSE_PARTICLES);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
			};

			{
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEGRID;
				const PxU32 numBlocks = (mMaxDiffuseParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(calcHashKernelFunction, numBlocks, numActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_calculateHashForDiffuseParticlesLaunch kernel fail!\n");
#endif
			}
		}
	}


	void PxgPBDParticleSystemCore::updateDirtyData(PxgBodySimManager& bodySimManager, CUstream stream)
	{
		PX_PROFILE_ZONE("updateDirtyData", 0);

		mDirtyParamsParticleSystems.forceSize_Unsafe(0);
		
		bool anyPBDDirty = false;
		bool anyBufferUpdate = false;

		void** bodySimsLL = bodySimManager.mBodies.begin();

		const PxU32 nbActiveParticleSystems = bodySimManager.mActivePBDParticleSystems.size();
		PxU32* activeParticleSystems = bodySimManager.mActivePBDParticleSystems.begin();

		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();
		PxgParticleSystem* gpuParticleSystems = reinterpret_cast<PxgParticleSystem*>(particleSystemd);

		mHasNonZeroFluidBoundaryScale = false;

		mNumActiveParticleSystems = nbActiveParticleSystems;


		mMaxRigidAttachmentsPerSystem = 0;
		PxU32 rigidAttachmentOffset = 0;

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			const PxU32 index = activeParticleSystems[i];
			PxgParticleSystem& particleSystem = mParticleSystemPool[index];
			PxgParticleSystemData& data = particleSystem.mData;

			PxU32 nodeIndex = mParticleSystemNodeIndexPool[data.mRemapIndex];
			Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[nodeIndex]);
			Dy::ParticleSystemCore& dyParticleSystemCore = dyParticleSystem->getCore();

			{
				if (dyParticleSystemCore.mCallback) 
				{
					PxGpuMirroredPointer<PxGpuParticleSystem> mirroredSystem(gpuParticleSystems + index, &particleSystem);
					dyParticleSystemCore.mCallback->onBegin(mirroredSystem, stream);
				}

				//in case user didn't request any reads we still need to synchronize on dangling work done in mFinalizeStream
				//PxScene::fetchResultsParticleSystem which synchronizes on mFinalizeStream is not mandatory.
				synchronizeStreams(mCudaContext, getFinalizeStream(), stream);
			}

			{

				PxgParticleSystemBuffer* buffer = mParticleSystemDataBuffer[data.mRemapIndex];
			    PxgParticleSystemDiffuseBuffer* diffuseBuffer = mDiffuseParticleDataBuffer[data.mRemapIndex];
				
				//create runsum for the particle position buffer
				anyBufferUpdate |= createUserParticleData(particleSystem, dyParticleSystemCore, buffer, diffuseBuffer, stream);
				
				anyPBDDirty |= anyBufferUpdate;

			}

			mMaxParticles = PxMax(mMaxParticles, particleSystem.mCommonData.mNumParticles);
			mMaxDiffuseParticles = PxMax(mMaxDiffuseParticles, particleSystem.mCommonData.mMaxDiffuseParticles);
			mMaxRigidAttachmentsPerSystem = PxMax(mMaxRigidAttachmentsPerSystem, particleSystem.mNumRigidAttachments);

			particleSystem.mRigidAttachmentOffset = rigidAttachmentOffset;
			rigidAttachmentOffset += particleSystem.mNumRigidAttachments;

			//mMaxVolumesPerPS = PxMax(mMaxVolumesPerPS, dyParticleSystemCore.mNumParticleVolumes);
			mHasNonZeroFluidBoundaryScale = mHasNonZeroFluidBoundaryScale || (dyParticleSystemCore.fluidBoundaryDensityScale > 0.0f);

			if (dyParticleSystem->mFlag & Dy::ParticleSystemFlag::eUPDATE_PARAMS)
			{
				updateParticleSystemData(particleSystem, dyParticleSystemCore);
				anyPBDDirty = true;

				//clear the flag
				dyParticleSystem->mFlag &= (~Dy::ParticleSystemFlag::eUPDATE_PARAMS);
			}

			if (dyParticleSystem->mFlag & Dy::ParticleSystemFlag::eUPDATE_MATERIAL)
			{
				anyPBDDirty = true;

				//clear the flag
				dyParticleSystem->mFlag &= (~Dy::ParticleSystemFlag::eUPDATE_MATERIAL);

				// Trigger phase update
				dyParticleSystem->mFlag |= Dy::ParticleSystemFlag::eUPDATE_PHASE;
			}

			if (dyParticleSystem->mFlag & Dy::ParticleSystemFlag::eUPDATE_PHASE)
			{
				anyPBDDirty = true;

				PxgParticleSystemBuffer* buffer = mParticleSystemDataBuffer[data.mRemapIndex];
				
				const PxU16* phaseToMaterial = dyParticleSystemCore.mPhaseGroupToMaterialHandle.begin();
				const PxU32 numPhaseToMaterial = dyParticleSystemCore.mPhaseGroupToMaterialHandle.size();
				buffer->phase_group_to_material_handle.allocate(sizeof(PxU16) * numPhaseToMaterial, PX_FL);
				buffer->mHostPhaseGroupToMaterialHandle.resize(numPhaseToMaterial);
				if(numPhaseToMaterial)
				{
					mGpuContext->mGpuNpCore->mapMaterialIndices<PxsPBDMaterialData>(
						buffer->mHostPhaseGroupToMaterialHandle.begin(), phaseToMaterial, numPhaseToMaterial);
					mCudaContext->memcpyHtoDAsync(buffer->phase_group_to_material_handle.getDevicePtr(),
						buffer->mHostPhaseGroupToMaterialHandle.begin(), sizeof(PxU16) * numPhaseToMaterial, stream);
				}
				particleSystem.mPhaseGroupToMaterialHandle = reinterpret_cast<PxU16*>(buffer->phase_group_to_material_handle.getDevicePtr());
				particleSystem.mData.mNumPhaseToMaterials = numPhaseToMaterial;
				buffer->derivedPBDMaterialProperties.allocate(sizeof(PxgPBDParticleMaterialDerived) * numPhaseToMaterial, PX_FL); // no need to copy old data
				particleSystem.mDerivedPBDMaterialData = reinterpret_cast<PxgPBDParticleMaterialDerived*>(buffer->derivedPBDMaterialProperties.getDevicePtr());

				mMaxNumPhaseToMaterials = PxMax(mMaxNumPhaseToMaterials, numPhaseToMaterial);

				//clear the flag
				dyParticleSystem->mFlag &= (~Dy::ParticleSystemFlag::eUPDATE_PHASE);
			}

			if (dyParticleSystem->mFlag & Dy::ParticleSystemFlag::eENABLE_GPU_DATA_SYNC)
			{
				PxgParticleBuffer** particleBuffers = reinterpret_cast<PxgParticleBuffer**>(dyParticleSystemCore.mParticleBuffers.begin());
				for (PxU32 j = 0; j < dyParticleSystemCore.mParticleBuffers.size(); ++j)
				{
					particleBuffers[j]->allocHostBuffers();
				}
				PxgParticleBuffer** particleDiffuseBuffers = reinterpret_cast<PxgParticleBuffer**>(dyParticleSystemCore.mParticleDiffuseBuffers.begin());
				for (PxU32 j = 0; j < dyParticleSystemCore.mParticleDiffuseBuffers.size(); ++j)
				{
					particleDiffuseBuffers[j]->allocHostBuffers();
				}
				PxgParticleBuffer** particleClothBuffers = reinterpret_cast<PxgParticleBuffer**>(dyParticleSystemCore.mParticleClothBuffers.begin());
				for (PxU32 j = 0; j < dyParticleSystemCore.mParticleClothBuffers.size(); ++j)
				{
					particleClothBuffers[j]->allocHostBuffers();
				}
				PxgParticleBuffer** particleRigidBuffers = reinterpret_cast<PxgParticleBuffer**>(dyParticleSystemCore.mParticleRigidBuffers.begin());
				for (PxU32 j = 0; j < dyParticleSystemCore.mParticleRigidBuffers.size(); ++j)
				{
					particleRigidBuffers[j]->allocHostBuffers();
				}
			}
		}

		mParticleRigidConstraints.allocate(sizeof(PxgParticleRigidConstraint)*(rigidAttachmentOffset + 31) / 32, PX_FL);
		mParticleRigidAttachmentIds.allocate(sizeof(PxU64)*rigidAttachmentOffset, PX_FL);
		mParticleRigidConstraintCount.allocate(sizeof(PxU32), PX_FL);
		mCudaContext->memsetD32Async(mParticleRigidConstraintCount.getDevicePtr(), rigidAttachmentOffset, 1, stream);

		mTotalRigidAttachments = rigidAttachmentOffset;


		CUdeviceptr activeParticleSystemsd = getActiveParticleSystemBuffer().getDevicePtr();
		//we have a cpu mirror of the particle system and we just dma the whole particle buffer to gpu
		//If any of the particle systems changed, we need to DMA up the particle systems because the num particles in data might change	

		if (anyPBDDirty || 1)
		{
			mCudaContext->memcpyHtoDAsync(particleSystemd, mParticleSystemPool.begin(), sizeof(PxgParticleSystem) * mNbTotalParticleSystems, stream);

			if (anyBufferUpdate)
				copyUserBufferToUnsortedArray(particleSystemd, activeParticleSystemsd, nbActiveParticleSystems, stream);
			
		}

		//update everyframe
		copyUserDiffuseBufferToUnsortedArray(particleSystemd, activeParticleSystemsd, nbActiveParticleSystems, stream);
	}
	
	void PxgPBDParticleSystemCore::applyParticleBufferDataDEPRECATED(const PxU32* indices, const PxGpuParticleBufferIndexPair* bufferIndexPairs, const PxParticleBufferFlags* flags, PxU32 nbUpdatedBuffers, CUevent waitEvent, CUevent signalEvent)
	{
		if (waitEvent)
			mCudaContext->streamWaitEvent(mStream, waitEvent);

		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		CUfunction kernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::APPLY_PARTICLE_BUFFER_DATA_DEPRECATED);

		const PxU32 numThreadPerBlock = 1024;
		const PxU32 numBlocks = (mMaxParticlesPerBuffer + numThreadPerBlock - 1) / numThreadPerBlock;

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(particleSystemd),
			PX_CUDA_KERNEL_PARAM(indices),
			PX_CUDA_KERNEL_PARAM(bufferIndexPairs),
			PX_CUDA_KERNEL_PARAM(flags),
		};

		CUresult result = mCudaContext->launchKernel(kernelFunction, numBlocks, nbUpdatedBuffers, 1, numThreadPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);


		PX_UNUSED(result);
		PX_ASSERT(result == CUDA_SUCCESS);

#if PS_GPU_DEBUG
		result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU applyParticleBufferData kernel fail!\n");
#endif

		if (signalEvent)
			mCudaContext->eventRecord(signalEvent, mStream);
		else
			mCudaContext->streamSynchronize(mStream);
	}
}