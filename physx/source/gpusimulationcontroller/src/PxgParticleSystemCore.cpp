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

#include "PxgCudaMemoryAllocator.h"
#include "PxgPBDParticleSystemCore.h"
#include "PxgParticleSystem.h"
#include "foundation/PxAssert.h"
#include "common/PxProfileZone.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxgCudaSolverCore.h"
#include "PxgCudaBroadPhaseSap.h"
#include "PxgSimulationController.h"
#include "PxgSimulationCore.h"
#include "PxgNarrowphaseCore.h"
#include "PxgNphaseImplementationContext.h"
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
#define PS_GPU_SPARSE_GRID_DEBUG 0
#define PS_SUBGRIDS_DEBUG 0
#define PS_SUBGRIDS_VERBOSE_DEBUG 0


namespace physx
{
	namespace
	{
		// Generates an optimally dense sphere packing at the origin (implicit sphere at the origin)
		inline PxU32 TightPack3D(PxReal radius, PxReal separation, PxVec3* points, PxU32 maxPoints)
		{
			if (separation <= 0.0f)
			{
				return 0;
			}

			PxI32 dim = PxI32(ceilf(radius / separation));
			PxU32 c = 0;

			for (PxI32 z = -dim; z <= dim; ++z)
			{
				for (PxI32 y = -dim; y <= dim; ++y)
				{
					for (PxI32 x = -dim; x <= dim; ++x)
					{
						PxReal xpos = x * separation + (((y + z) & 1) ? separation * 0.5f : 0.0f);
						PxReal ypos = y * sqrtf(0.75f)*separation;
						PxReal zpos = z * sqrtf(0.75f)*separation;

						PxVec3 p(xpos, ypos, zpos);

						// skip center
						if (p.magnitudeSquared() == 0.0f)
							continue;

						if (c == maxPoints)
							return c;

						if (p.magnitude() <= radius)
						{
							points[c] = p;
							++c;
						}
					}
				}
			}

			return c;
		}

		const PxReal kPi = 3.141592653589f;

		inline float sqr(float x) { return x * x; }

		inline float W(float x, float h)
		{
			// Clavet et al. unnormalized spike
			const float k = 15.0f / (kPi*h*h*h);
			return k * sqr(1.0f - x / h);
		}

		inline float dWdx(float x, float h)
		{
			const float k = 15.0f / (kPi*h*h*h);
			return -k * 2.0f*(1.0f - x / h) / h;
		}

		// calculate the rest density for given radius and rest distance
		// rho is the rest density, rhoDeriv is the scaling factor for the density constraint
		inline void CalculateRestDensity(float restDistance, float h, float& rho, float& rhoDeriv, float& surfaceDeriv)
		{
			PxVec3 samples[2048];

			PxU32 n = TightPack3D(h, restDistance, &samples[0], 2048);

			rho = 0.0f;
			rhoDeriv = 0.0f;

			float a = 0.0f;
			float b = 0.0f;

			// calculate rest density given fixed samples
			for (PxU32 i = 0; i < n; ++i)
			{
				float d = samples[i].magnitude();

				rho += W(d, h);
				rhoDeriv += sqr(dWdx(d, h));

				if (samples[i].y <= 0.0f)
				{
					float cosTheta = samples[i].y / samples[i].magnitude();

					a += dWdx(d, h)*cosTheta;
					b -= samples[i].magnitude() * cosTheta;
				}
			}

			surfaceDeriv = a / b;
		}
	}
	extern "C" void initParticleSystemKernels0();
	extern "C" void initDiffuseParticlesKernels0();
	extern "C" void initIsosurfaceExtractionKernels0();
	extern "C" void initSparseGridStandaloneKernels0();
	extern "C" void initAnisotropyKernels0();
	extern "C" void initAlgorithmsKernels0();
	extern "C" void initSdfConstructionKernels0();

	void createPxgParticleSystem()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXSimulationControllerGpu linkage as Static Library!
		initParticleSystemKernels0();
		initDiffuseParticlesKernels0();
		initIsosurfaceExtractionKernels0();
		initSparseGridStandaloneKernels0();
		initAnisotropyKernels0();
		initAlgorithmsKernels0();
		initSdfConstructionKernels0();
#endif
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	static PxU32 uniqueIdIndexer = 0;

	template<class BufferClass>
	PxgParticleBufferBase<BufferClass>::PxgParticleBufferBase(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxCudaContextManager& contextManager) :
		mBufferFlags(0),
		mContextManager(contextManager),
		mPositionInvMassesD(NULL),
		mVelocitiesD(NULL),
		mPhasesD(NULL),
		mVolumesD(NULL),
		mPositionInvMassesH(NULL),
		mVelocitiesH(NULL),
		mPhasesH(NULL),
		mVolumesH(NULL),
		mFilterPairs(NULL),
		mRigidAttachments(NULL),
		mNumActiveParticles(0),
		mMaxNumParticles(maxNumParticles),
		mNumParticleVolumes(0),
		mMaxNumVolumes(maxNumVolumes),
		mNumFilterPairs(0),
		mNumRigidAttachments(0),
		mFlatListStartIndex(0),
		mUniqueId(uniqueIdIndexer++)
	{
		PX_ASSERT(maxNumParticles > 0);
		mPositionInvMassesD = PX_DEVICE_MEMORY_ALLOC(PxVec4, contextManager, maxNumParticles);
		mVelocitiesD = PX_DEVICE_MEMORY_ALLOC(PxVec4, contextManager, maxNumParticles);
		mPhasesD = PX_DEVICE_MEMORY_ALLOC(PxU32, contextManager, maxNumParticles);
		if (maxNumVolumes)
		{
			mVolumesD = PX_DEVICE_MEMORY_ALLOC(PxParticleVolume, contextManager, maxNumVolumes);
		}
	}

	template<class BufferClass>
	PxgParticleBufferBase<BufferClass>::~PxgParticleBufferBase()
	{
		PX_DEVICE_MEMORY_FREE(mContextManager, mPositionInvMassesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mVelocitiesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mPhasesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mVolumesD);
		
		PX_PINNED_MEMORY_FREE(mContextManager, mPositionInvMassesH);
		PX_PINNED_MEMORY_FREE(mContextManager, mVelocitiesH);
		PX_PINNED_MEMORY_FREE(mContextManager, mPhasesH);
		PX_PINNED_MEMORY_FREE(mContextManager, mVolumesH);
	}

	template<class BufferClass>
	void PxgParticleBufferBase<BufferClass>::setNbActiveParticles(PxU32 numActiveParticles)
	{
		PX_ASSERT(numActiveParticles <= mMaxNumParticles);
		if (mNumActiveParticles != numActiveParticles)
		{
			mBufferFlags |= (PxParticleBufferFlag::eUPDATE_POSITION | PxParticleBufferFlag::eUPDATE_VELOCITY |
				PxParticleBufferFlag::eUPDATE_PHASE);
		}
		mNumActiveParticles = numActiveParticles;
	}

	template<class BufferClass>
	void PxgParticleBufferBase<BufferClass>::setRigidFilters(PxParticleRigidFilterPair* filters, PxU32 numFilters)
	{
		mFilterPairs = filters;
		mNumFilterPairs = numFilters;

		mBufferFlags |= PxParticleBufferFlag::eUPDATE_ATTACHMENTS;
	}

	template<class BufferClass>
	void PxgParticleBufferBase<BufferClass>::setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 numAttachments)
	{
		mRigidAttachments = attachments;
		mNumRigidAttachments = numAttachments;

		mBufferFlags |= PxParticleBufferFlag::eUPDATE_ATTACHMENTS;
	}

	template<class BufferClass>
	void PxgParticleBufferBase<BufferClass>::allocHostBuffers()
	{
		PX_ASSERT(mMaxNumParticles > 0);
		if (!mPositionInvMassesH)
		{
			PX_ASSERT(!mVelocitiesH && !mPhasesH && !mVolumesH);
			mPositionInvMassesH = PX_PINNED_MEMORY_ALLOC(PxVec4, mContextManager, mMaxNumParticles);
			mVelocitiesH = PX_PINNED_MEMORY_ALLOC(PxVec4, mContextManager, mMaxNumParticles);
			mPhasesH = PX_PINNED_MEMORY_ALLOC(PxU32, mContextManager, mMaxNumParticles);
			if (mMaxNumVolumes)
			{
				mVolumesH = PX_PINNED_MEMORY_ALLOC(PxParticleVolume, mContextManager, mMaxNumVolumes);
			}
		}
	}

	template<class BufferClass>
	void PxgParticleBufferBase<BufferClass>::copyToHost(CUstream stream)
	{
		mContextManager.copyDToHAsync<PxVec4>(mPositionInvMassesH, mPositionInvMassesD, mNumActiveParticles, stream);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	PxgParticleBuffer::PxgParticleBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxCudaContextManager& contextManager) :
		PxgParticleBufferBase<PxsParticleBuffer>(maxNumParticles, maxNumVolumes, contextManager)
	{}

	void PxgParticleBuffer::copyToHost(CUstream stream)
	{
		PxgParticleBufferBase<PxsParticleBuffer>::copyToHost(stream);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	PxgParticleAndDiffuseBuffer::PxgParticleAndDiffuseBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxU32 maxNumDiffuseParticles, PxCudaContextManager& contextManager) :
		PxgParticleBufferBase<PxsParticleAndDiffuseBuffer>(maxNumParticles, maxNumVolumes, contextManager),
		mDiffusePositionsLifeTimeD(NULL),
		mDiffuseVelocitiesD(NULL),
		mNumDiffuseParticlesD(NULL),
		mMaxNumDiffuseParticles(0),
		mMaxActiveDiffuseParticles(0),
		mNumActiveDiffuseParticlesH(NULL)
	{
		mContextManager.acquireContext();

		if (maxNumDiffuseParticles > 0)
		{
			mDiffusePositionsLifeTimeD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, maxNumDiffuseParticles);
			mDiffuseVelocitiesD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, maxNumDiffuseParticles);
			mMaxActiveDiffuseParticles = maxNumDiffuseParticles;
		}

		mNumDiffuseParticlesD = PX_DEVICE_MEMORY_ALLOC(int, mContextManager, 2);
		mNumActiveDiffuseParticlesH = PX_PINNED_MEMORY_ALLOC(int, mContextManager, 1);
		*mNumActiveDiffuseParticlesH = 0;

		PxCudaContext* cudaContext = mContextManager.getCudaContext();
		cudaContext->memsetD32((CUdeviceptr)mNumDiffuseParticlesD, 0, 2);
		mMaxNumDiffuseParticles = maxNumDiffuseParticles;
		mContextManager.releaseContext();
	}

	PxgParticleAndDiffuseBuffer::~PxgParticleAndDiffuseBuffer()
	{
		PX_DEVICE_MEMORY_FREE(mContextManager, mDiffusePositionsLifeTimeD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mDiffuseVelocitiesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mNumDiffuseParticlesD);
		PX_PINNED_MEMORY_FREE(mContextManager, mNumActiveDiffuseParticlesH);
	}

	void PxgParticleAndDiffuseBuffer::setMaxActiveDiffuseParticles(PxU32 maxActiveDiffuseParticles)
	{
		PX_ASSERT(maxActiveDiffuseParticles <= mMaxNumDiffuseParticles);
		mMaxActiveDiffuseParticles = maxActiveDiffuseParticles;
		mBufferFlags |= PxParticleBufferFlag::eUPDATE_DIFFUSE_PARAM;
	}

	void PxgParticleAndDiffuseBuffer::setDiffuseParticleParams(const PxDiffuseParticleParams& params)
	{
		mParams = params;
		mBufferFlags |= PxParticleBufferFlag::eUPDATE_DIFFUSE_PARAM;
	}

	void PxgParticleAndDiffuseBuffer::copyToHost(CUstream stream)
	{
		PxgParticleBufferBase<PxsParticleAndDiffuseBuffer>::copyToHost(stream);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	PxgParticleClothBuffer::PxgParticleClothBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, 
		PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager& contextManager) :
		PxgParticleBufferBase<PxsParticleClothBuffer>(maxNumParticles, maxNumVolumes, contextManager),
		mRestPositionsD(NULL),
		mTriangleIndicesD(NULL),
		mAccumulatedSpringsPerPartitionsD(NULL),
		mAccumulatedCopiesPerParticlesD(NULL),
		mRemapOutputD(NULL),
		mOrderedSpringsD(NULL),
		mSortedClothStartIndicesD(NULL),
		mClothsD(NULL),
		mRemapPositionsD(NULL),
		mRemapVelocitiesD(NULL),
		mSpringLambdaD(NULL),
		mInflatableLambdaD(NULL),
		mMaxNumCloths(maxNumCloths),
		mMaxNumTriangles(maxNumTriangles),
		mMaxNumSprings(maxNumSprings),
		mNumPartitions(0),
		mMaxSpringsPerPartition(0),
		mNumSprings(0),
		mNumCloths(0),
		mNumTriangles(0),
		mRemapOutputSize(0)
	{
		PX_ASSERT(maxNumParticles > 0 && maxNumTriangles > 0);
		mRestPositionsD = PX_DEVICE_MEMORY_ALLOC(PxVec4, contextManager, maxNumParticles);
		mTriangleIndicesD = PX_DEVICE_MEMORY_ALLOC(PxU32, contextManager, maxNumTriangles * 3);
	}

	PxgParticleClothBuffer::~PxgParticleClothBuffer()
	{
		PX_DEVICE_MEMORY_FREE(mContextManager, mRestPositionsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mTriangleIndicesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mAccumulatedSpringsPerPartitionsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mAccumulatedCopiesPerParticlesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRemapOutputD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mOrderedSpringsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mSortedClothStartIndicesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mClothsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRemapPositionsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRemapVelocitiesD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mSpringLambdaD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mInflatableLambdaD);
	}

	void PxgParticleClothBuffer::setNbActiveParticles(PxU32 numActiveParticles)
	{
		PxgParticleBufferBase<PxsParticleClothBuffer>::setNbActiveParticles(numActiveParticles);
		mBufferFlags |= PxParticleBufferFlag::eUPDATE_RESTPOSITION;
	}

	void PxgParticleClothBuffer::setCloths(PxPartitionedParticleCloth& cloths)
	{
		PX_ASSERT(mNumActiveParticles > 0 && mNumActiveParticles <= mMaxNumParticles);

		mContextManager.acquireContext();
		PxCudaContext* cudaContext = mContextManager.getCudaContext();

		if (mNumSprings != cloths.nbSprings || mNumCloths != cloths.nbCloths || mNumPartitions != cloths.nbPartitions || mRemapOutputSize != cloths.remapOutputSize)
		{
			mNumSprings = cloths.nbSprings;
			mNumCloths = cloths.nbCloths;
			mNumPartitions = cloths.nbPartitions;
			mRemapOutputSize = cloths.remapOutputSize;

			PX_DEVICE_MEMORY_FREE(mContextManager, mAccumulatedCopiesPerParticlesD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mAccumulatedSpringsPerPartitionsD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mRemapOutputD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mOrderedSpringsD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mSpringLambdaD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mSortedClothStartIndicesD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mClothsD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mRemapPositionsD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mRemapVelocitiesD);
			PX_DEVICE_MEMORY_FREE(mContextManager, mInflatableLambdaD);

			mAccumulatedCopiesPerParticlesD = PX_DEVICE_MEMORY_ALLOC(PxU32, mContextManager, mNumActiveParticles);
			mAccumulatedSpringsPerPartitionsD = PX_DEVICE_MEMORY_ALLOC(PxU32, mContextManager, mNumPartitions);
			if (mNumSprings > 0)
			{
				mRemapOutputD = PX_DEVICE_MEMORY_ALLOC(PxU32, mContextManager, mNumSprings * 2);
				mOrderedSpringsD = PX_DEVICE_MEMORY_ALLOC(PxParticleSpring, mContextManager, mNumSprings);
				mSpringLambdaD = PX_DEVICE_MEMORY_ALLOC(PxReal, mContextManager, mNumSprings);
			}
			else
			{
				mRemapOutputD = NULL;
				mOrderedSpringsD = NULL;
				mSpringLambdaD = NULL;
			}
			mSortedClothStartIndicesD = PX_DEVICE_MEMORY_ALLOC(PxU32, mContextManager, mNumCloths);
			mClothsD = PX_DEVICE_MEMORY_ALLOC(PxParticleCloth, mContextManager, mNumCloths);
			mRemapPositionsD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, mRemapOutputSize);
			mRemapVelocitiesD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, mRemapOutputSize);
			mInflatableLambdaD = PX_DEVICE_MEMORY_ALLOC(PxReal, mContextManager, mNumCloths);
		}

		
		mMaxSpringsPerPartition = cloths.maxSpringsPerPartition;

		//cuEventSynchronize((CUevent)waitEvent);
		cudaContext->memcpyHtoD(CUdeviceptr(mAccumulatedCopiesPerParticlesD), cloths.accumulatedCopiesPerParticles, size_t(sizeof(PxU32) * mNumActiveParticles));
		cudaContext->memcpyHtoD(CUdeviceptr(mAccumulatedSpringsPerPartitionsD), cloths.accumulatedSpringsPerPartitions, size_t(sizeof(PxU32) * mNumPartitions));
		cudaContext->memcpyHtoD(CUdeviceptr(mRemapOutputD), cloths.remapOutput, size_t(sizeof(PxU32) * mNumSprings * 2));
		cudaContext->memcpyHtoD(CUdeviceptr(mOrderedSpringsD), cloths.orderedSprings, size_t(sizeof(PxParticleSpring) * mNumSprings));

		cudaContext->memcpyHtoD(CUdeviceptr(mSortedClothStartIndicesD), cloths.sortedClothStartIndices, size_t(sizeof(PxU32) * mNumCloths));
		cudaContext->memcpyHtoD(CUdeviceptr(mClothsD), cloths.cloths, size_t(sizeof(PxParticleCloth) * mNumCloths));

		mBufferFlags |= PxParticleBufferFlag::eUPDATE_CLOTH;

		mContextManager.releaseContext();
	}

	void PxgParticleClothBuffer::copyToHost(CUstream stream)
	{
		PxgParticleBufferBase<PxsParticleClothBuffer>::copyToHost(stream);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	PxgParticleRigidBuffer::PxgParticleRigidBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager& contextManager) :
		PxgParticleBufferBase<PxsParticleRigidBuffer>(maxNumParticles, maxNumVolumes, contextManager),
		mRigidOffsetsD(NULL),
		mRigidCoefficientsD(NULL),
		mRigidLocalPositionsD(NULL),
		mRigidLocalNormalsD(NULL),
		mRigidTranslationsD(NULL),
		mRigidRotationsD(NULL),
		mNumActiveRigids(0),
		mMaxNumRigids(maxNumRigids)
	{
		PX_ASSERT(maxNumRigids > 0 && maxNumParticles > 0);

		mRigidOffsetsD = PX_DEVICE_MEMORY_ALLOC(PxU32, mContextManager, (maxNumRigids + 1));
		mRigidCoefficientsD = PX_DEVICE_MEMORY_ALLOC(PxReal, mContextManager, maxNumRigids);
		mRigidTranslationsD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, maxNumRigids);
		mRigidRotationsD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, maxNumRigids);

		mRigidLocalPositionsD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, maxNumParticles);
		mRigidLocalNormalsD = PX_DEVICE_MEMORY_ALLOC(PxVec4, mContextManager, maxNumParticles);
	}

	PxgParticleRigidBuffer::~PxgParticleRigidBuffer()
	{
		PX_DEVICE_MEMORY_FREE(mContextManager, mRigidOffsetsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRigidCoefficientsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRigidTranslationsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRigidRotationsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRigidLocalNormalsD);
		PX_DEVICE_MEMORY_FREE(mContextManager, mRigidLocalPositionsD);
	}

	void PxgParticleRigidBuffer::copyToHost(CUstream stream)
	{
		PxgParticleBufferBase<PxsParticleRigidBuffer>::copyToHost(stream);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	PxgParticleSystemCore::PxgParticleSystemCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
		PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController, PxgGpuContext* gpuContext, const PxU32 maxParticleContacts) :
		PxgNonRigidCore(gpuKernelWrangler, cudaContextManager, heapMemoryManager, simController, gpuContext, maxParticleContacts, 0, PxsHeapStats::eSHARED_PARTICLES),
		mNewParticleSystemPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mParticleSystemPool(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mDirtyParamsParticleSystems(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
		mParticleSystemBuffer(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mActiveParticleSystemBuffer(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempGridParticleHashBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempGridParticleIndexBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempGridDiffuseParticleHashBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempGridDiffuseParticleIndexBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempCellsHistogramBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempBlockCellsHistogramBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempHistogramCountBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempBoundsBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveContactsBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveContactCountBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveContactSortedByParticleBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveContactSortedByRigidBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveConstraintSortedByParticleBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveConstraintSortedByRigidBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveConstraintAppliedParticleForces(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mPrimitiveConstraintAppliedRigidForces(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mDeltaVelParticleBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mDeltaVelRigidBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempBlockDeltaVelBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mTempBlockRigidIdBuf(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mParticleRigidConstraints(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mParticleRigidAttachmentIds(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mParticleRigidConstraintCount(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mParticleRigidAttachmentScaleBuffer(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
		mHasFlipPhase(heapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),	
		mDiffuseParticlesRSDesc(heapMemoryManager->mMappedMemoryAllocators),
		mNbTotalParticleSystems(0),
		mMaxParticles(0),
		mHasNonZeroFluidBoundaryScale(true),
		mMaxParticlesPerBuffer(0),
		mMaxBuffersPerSystem(0),
		mMaxDiffusePerBuffer(0),
		mMaxDiffuseBuffersPerSystem(0),
		mMaxRigidAttachmentsPerSystem(0),
		mHostContactCount(NULL)
	{
		mCudaContextManager->acquireContext();

		int leastPriority, mostPriority;
		cuCtxGetStreamPriorityRange(&leastPriority, &mostPriority);

		mCudaContext->streamCreateWithPriority(&mStream, CU_STREAM_NON_BLOCKING, leastPriority);
		mCudaContext->streamCreateWithPriority(&mFinalizeStream, CU_STREAM_NON_BLOCKING, leastPriority);

		mCudaContext->eventCreate(&mBoundUpdateEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveParticleEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSelfCollisionEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mFinalizeStartEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveParticleRigidEvent, CU_EVENT_DISABLE_TIMING);
		mCudaContext->eventCreate(&mSolveRigidParticleEvent, CU_EVENT_DISABLE_TIMING);

#if 1

		mTempCellsHistogramBuf.allocate(1024 * 1024 * 64, PX_FL);
		mTempBlockCellsHistogramBuf.allocate(PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE * sizeof(PxU32), PX_FL);
		mTempHistogramCountBuf.allocate(sizeof(PxU32), PX_FL);

		const PxU32 contactBufSize = mMaxContacts * sizeof(PxgParticlePrimitiveContact);
		const PxU32 constraintBufSize = ((mMaxContacts+31)/32) * sizeof(PxgParticlePrimitiveConstraintBlock);
		const PxU32 appliedForceSize = mMaxContacts * sizeof(float2);

		mPrimitiveContactsBuf.allocate(contactBufSize, PX_FL);
		mPrimitiveContactCountBuf.allocate(sizeof(PxU32), PX_FL);

		mPrimitiveContactSortedByParticleBuf.allocate(contactBufSize, PX_FL);
		mPrimitiveContactSortedByRigidBuf.allocate(contactBufSize, PX_FL);

		mPrimitiveConstraintSortedByParticleBuf.allocate(constraintBufSize, PX_FL);
		mPrimitiveConstraintSortedByRigidBuf.allocate(constraintBufSize, PX_FL);

		mPrimitiveConstraintAppliedParticleForces.allocate(appliedForceSize, PX_FL);
		mPrimitiveConstraintAppliedRigidForces.allocate(appliedForceSize, PX_FL);

		mDeltaVelParticleBuf.allocate(mMaxContacts *sizeof(float4), PX_FL);
		mDeltaVelRigidBuf.allocate(mMaxContacts *sizeof(float4)*2, PX_FL);

		mTempBlockDeltaVelBuf.allocate(sizeof(PxVec4) * 2 * PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA, PX_FL);
		mTempBlockRigidIdBuf.allocate(sizeof(PxU64) * PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA, PX_FL);

		mHostContactCount = PX_PINNED_MEMORY_ALLOC(PxU32, *mCudaContextManager, 1);
		*mHostContactCount = 0;

#endif
		mCudaContextManager->releaseContext();


		mCurrentTMIndex = 0;

	}

	PxgParticleSystemCore::~PxgParticleSystemCore()
	{
		mCudaContextManager->acquireContext();

		//destroy stream
		mCudaContext->streamDestroy(mFinalizeStream);

		mCudaContext->eventDestroy(mBoundUpdateEvent);
		mCudaContext->eventDestroy(mSolveParticleEvent);
		mCudaContext->eventDestroy(mFinalizeStartEvent);
		mCudaContext->eventDestroy(mSolveParticleRigidEvent);
		mCudaContext->eventDestroy(mSolveRigidParticleEvent);

		PX_PINNED_MEMORY_FREE(*mCudaContextManager, mHostContactCount);

		mCudaContextManager->releaseContext();
	}

	void PxgParticleSystemCore::calculateHash(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 numActiveParticleSystems)
	{
		if (mMaxParticles)
		{
			const CUfunction calcHashKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_CALCULATE_HASH);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
			};

			{
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEGRID;
				const PxU32 numBlocks = (mMaxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(calcHashKernelFunction, numBlocks, numActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU calculateHash kernel fail!\n");
#endif
			}
		}

	}

	void PxgParticleSystemCore::reorderDataAndFindCellStart(PxgParticleSystem* particleSystems, CUdeviceptr particleSystemsd, const PxU32 id, const PxU32 numParticles)
	{
		//PxgParticleSystem* particleSystems = mSimController->getParticleSystems();
		PxgParticleSystem& particleSystem = particleSystems[id];
		CUdeviceptr cellStartd = reinterpret_cast<CUdeviceptr>(particleSystem.mCellStart);
		CUdeviceptr diffuseCellStartd = reinterpret_cast<CUdeviceptr>(particleSystem.mDiffuseCellStart);
		const PxU32 numCells = particleSystem.getNumCells();

		// set all cells to empty
		mCudaContext->memsetD32Async(cellStartd, EMPTY_CELL, numCells, mStream);
		if (particleSystem.mCommonData.mMaxDiffuseParticles > 0)
			mCudaContext->memsetD32Async(diffuseCellStartd, EMPTY_CELL, numCells, mStream);
		
		if (numParticles == 0)
			return;
		//cudaMemset((void*)cellStart, 0xffffffff, numCells * sizeof(PxU32));

		

		
		const CUfunction findCellKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_REORDER_PARTICLE_FIND_CELLSTARTEND);

		PxCudaKernelParam kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(particleSystemsd),
			PX_CUDA_KERNEL_PARAM(id)
		};

		{
			const PxU32 numThreadPerBlock = PxgParticleSystemKernelBlockDim::UPDATEGRID;
			const PxU32 numBlocks = (numParticles + numThreadPerBlock - 1) / numThreadPerBlock;
			CUresult result = mCudaContext->launchKernel(findCellKernelFunction, numBlocks, 1, 2, numThreadPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			/*result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU reorderDataAndFindCellStartD kernel fail!\n");*/

#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU reorderDataAndFindCellStartD kernel fail!\n");


			/*CUdeviceptr sortedPosd = reinterpret_cast<CUdeviceptr>(particleSystem.mSortedPostion_Mass);

			PxArray<PxVec4> sortedPos;
			sortedPos.reserve(numParticles);
			sortedPos.forceSize_Unsafe(numParticles);

			PxArray<PxU32> tCellStarts;
			tCellStarts.reserve(numCells);
			tCellStarts.forceSize_Unsafe(numCells);

			PxArray<PxU32> tCellEnds;
			tCellEnds.reserve(numCells);
			tCellEnds.forceSize_Unsafe(numCells);

			PxArray<PxU32> hash;
			PxArray<PxU32> particleIndex;
			hash.reserve(numParticles);
			hash.forceSize_Unsafe(numParticles);

			particleIndex.reserve(numParticles);
			particleIndex.forceSize_Unsafe(numParticles);

			CUdeviceptr hashd = reinterpret_cast<CUdeviceptr>(particleSystem.mGridParticleHash);
			CUdeviceptr particleIndexd = reinterpret_cast<CUdeviceptr>(particleSystem.mGridParticleIndex);
			CUdeviceptr cellEndd = reinterpret_cast<CUdeviceptr>(particleSystem.mCellEnd);

			mCudaContext->memcpyDtoH(hash.begin(), hashd, sizeof(PxU32) * numParticles);
			mCudaContext->memcpyDtoH(particleIndex.begin(), particleIndexd, sizeof(PxU32) * numParticles);

			mCudaContext->memcpyDtoH(tCellStarts.begin(), cellStartd, sizeof(PxU32) * numCells);
			mCudaContext->memcpyDtoH(tCellEnds.begin(), cellEndd, sizeof(PxU32) * numCells);

			
			mCudaContext->memcpyDtoH(sortedPos.begin(), sortedPosd, sizeof(PxVec4) * numParticles);
			mCudaContext->memcpyDtoH(sortedPos.begin(), sortedPosd, sizeof(PxVec4) * numParticles);

			int bob = 0;
			PX_UNUSED(bob);*/

#endif

		}
	}


	void PxgParticleSystemCore::copyUserBufferToUnsortedArray(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles, CUstream bpStream)
	{
		const CUfunction preIntegrateKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_UNSORTED_ARRAY);

		const PxU32 numBlocks = (mMaxParticlesPerBuffer + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;

		if(mMaxParticlesPerBuffer)
		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
			};

	
			CUresult result = mCudaContext->launchKernel(preIntegrateKernelFunction, numBlocks, mMaxBuffersPerSystem, nbActiveParticles, PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);


#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(bpStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateUnsortedArrayLaunch kernel fail!\n");
#endif
		}

	}

	void PxgParticleSystemCore::copyUserDiffuseBufferToUnsortedArray(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
		const PxU32 nbActiveParticles, CUstream bpStream)
	{
		if (mMaxDiffusePerBuffer > 0)
		{
			const CUfunction copyToDiffuseUnsortedFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_DIFFUSE_UNSORTED_ARRAY);

			const PxU32 numBlocks = (mMaxDiffusePerBuffer + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;


			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
				};


				CUresult result = mCudaContext->launchKernel(copyToDiffuseUnsortedFunction, numBlocks, mMaxDiffuseBuffersPerSystem, nbActiveParticles, PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(bpStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateUnsortedDiffuseArrayLaunch kernel fail!\n");
#endif
			}
		}
	}

	void PxgParticleSystemCore::preIntegrateSystem(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles,
		const PxVec3 gravity, const PxReal dt, CUstream bpStream)
	{
	    const bool isTGS = mGpuContext->isTGS();
	    const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled();

		if (mMaxParticles)
		{
			const CUfunction preIntegrateKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PREINTEGRATION);

			const PxU32 numBlocks = (mMaxParticles + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;

			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
					PX_CUDA_KERNEL_PARAM(gravity),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(isTGS),
					PX_CUDA_KERNEL_PARAM(externalForcesEveryTgsIterationEnabled)
				};

				//one  blockIdx.z to indicate normal particles or diffuse particles
				CUresult result = mCudaContext->launchKernel(preIntegrateKernelFunction, numBlocks, nbActiveParticles, 1, PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(bpStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU preIntegrateSystem kernel fail!\n");
#endif
			}
		}
	
	}

	void PxgParticleSystemCore::updateBound(const PxgParticleSystem& sys, PxgParticleSystem* particleSystems,
		PxBounds3* boundArray, PxReal* contactDists, CUstream bpStream)
	{
		const bool isTGS = mGpuContext->isTGS();
		const CUfunction updateBoundFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_BOUND_FRIST);
		const CUfunction updateBoundSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_BOUND_SECOND);

		const PxU32 numThreadsPerWarp = 32;
		const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND / numThreadsPerWarp;
		const PxU32 numDiffuseParticles = sys.mCommonData.mMaxDiffuseParticles;
		const PxU32 numBlocks = (PxMax(sys.mCommonData.mNumParticles, numDiffuseParticles) + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;
		
		mTempBoundsBuf.allocate(sizeof(PxBounds3) * numBlocks, PX_FL);
		CUdeviceptr tempBuf = mTempBoundsBuf.getDevicePtr();
		const PxU32 particleSystemId = sys.mData.mRemapIndex;

		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystems),
				PX_CUDA_KERNEL_PARAM(particleSystemId),
				PX_CUDA_KERNEL_PARAM(isTGS),
				PX_CUDA_KERNEL_PARAM(tempBuf)
			};

			CUresult result = mCudaContext->launchKernel(updateBoundFirstKernelFunction, numBlocks, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
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

			////Dma back the bound
			//PxBounds3 bounds;
			//mCudaContext->memcpyDtoH(&bounds, tempBuf, sizeof(PxBounds3));

			//int bob = 0;
			//PX_UNUSED(bob);
#endif
		}
		{
			//elemId is the index to the bound array in BP
			const PxU32 elemId = sys.mData.mElementIndex;
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystems),
				PX_CUDA_KERNEL_PARAM(particleSystemId),
				PX_CUDA_KERNEL_PARAM(tempBuf),
				PX_CUDA_KERNEL_PARAM(numBlocks),
				PX_CUDA_KERNEL_PARAM(boundArray),
				PX_CUDA_KERNEL_PARAM(contactDists),
				PX_CUDA_KERNEL_PARAM(elemId)
			};

			CUresult result = mCudaContext->launchKernel(updateBoundSecondKernelFunction, 1, 1, 1, numThreadsPerWarp, numWarpsPerBlock, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

			/*result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateBound second pass kernel fail!\n");*/

#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(bpStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU updateBound second pass kernel fail!\n");

			
			//Dma back the bound
			//PxBounds3 bounds[3];
			//mCudaContext->memcpyDtoH(bounds, CUdeviceptr(boundArray ), sizeof(PxBounds3) * 3);

			//int bob = 0;
			//PX_UNUSED(bob);
#endif
		}

	}

	void PxgParticleSystemCore::updateBounds(PxgParticleSystem* particleSystems, PxU32* activeParticleSystems, const PxU32 nbActiveParticleSystems)
	{
		//boundsd  and contactDistd are guaranteed in the GPU if updateBound run on the broad phase stream
		//update bounds need to run on the broad phase stream so the broad phase can kick off after particle update bound
		CUstream bpStream = mGpuContext->mGpuBp->getBpStream();

		
		PxgCudaBuffer& particleSystemBuffer = getParticleSystemBuffer();
		PxgParticleSystem* particleSystemsd = reinterpret_cast<PxgParticleSystem*>(particleSystemBuffer.getDevicePtr());

		PxBounds3* boundsd = reinterpret_cast<PxBounds3*>(mGpuContext->mGpuBp->getBoundsBuffer().getDevicePtr());
		PxReal* contactDistd = reinterpret_cast<PxReal*>(mGpuContext->mGpuBp->getContactDistBuffer().getDevicePtr());
		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			if (particleSystems[activeParticleSystems[i]].mCommonData.mNumParticles)
			{
				updateBound(particleSystems[activeParticleSystems[i]], particleSystemsd, boundsd, contactDistd, bpStream);
			}
		}


		//mCudaContext->streamFlush(bpStream); //Force update ASAP
	}

	PX_FORCE_INLINE PxI32 getNbBits(PxI32 x)
	{
		PxI32 n = 0;
		while (x >= 2)
		{
			++n;
			x /= 2;
		}

		return n;
	}

	void PxgParticleSystemCore::updateGrid(PxgParticleSystem* particleSystems, const PxU32* activeParticleSystems, 
		const PxU32 nbActiveParticleSystems, CUdeviceptr particleSystemsd)
	{
		PX_PROFILE_ZONE("PxgParticleSystemCore::UpdateGrid", 0);
		
		mTempGridParticleHashBuf.allocate(mMaxParticles * sizeof(PxU32), PX_FL);
		mTempGridParticleIndexBuf.allocate(mMaxParticles * sizeof(PxU32), PX_FL);

		//We need 2x rsDesc on the host per-particle system. The reason for this is that, while the sorting occurs synchronously on the 
		//same stream on the device, the host-side buffers could get changed prior to the DMAs having occurred due to device latency
		const PxU32 nbRequired = (2u + nbActiveParticleSystems);
		mRSDesc.resize(nbRequired * 2u);
		//mDiffuseParticlesRSDesc.resize(nbRequired * 2u);

		mRadixCountTotalBuf.allocate(mRadixCountSize * nbRequired, PX_FL);

		for (PxU32 i = 0; i < 2; ++i)
		{
			mRadixSortDescBuf[i].allocate(sizeof(PxgRadixSortBlockDesc)*nbRequired, PX_FL);
		}

		CUdeviceptr activeParticleSystemsd = getActiveParticleSystemBuffer().getDevicePtr();
		calculateHash(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems);


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

					PxgRadixSortCore::sort(mGpuKernelWranglerManager, mCudaContext, mStream, numSimParticles, mRadixSortDescBuf.begin(), PxgRadixSortCore::getNbBits(PxI32(ps.mCommonData.mGridSizeX*ps.mCommonData.mGridSizeY*ps.mCommonData.mGridSizeZ)), rsDescs);
				}
			}

			const PxU32 numParticles = PxMax(ps.mCommonData.mNumParticles, ps.mCommonData.mMaxDiffuseParticles);
			reorderDataAndFindCellStart(particleSystems, particleSystemsd, index, numParticles);
		}
	}

	void PxgParticleSystemCore::selfCollision(PxgParticleSystem& particleSystem, PxgParticleSystem* particleSystemsd, const PxU32 id, const PxU32 numParticles)
	{
		PX_UNUSED(particleSystem);
		//use the histogram count for the shared index for self collision
		CUdeviceptr sharedIndex = mTempHistogramCountBuf.getDevicePtr();
		mCudaContext->memsetD32Async(sharedIndex, 0, 1, mStream);

		if(!(particleSystem.mData.mFlags & PxParticleFlag::eDISABLE_SELF_COLLISION))
		{
			const CUfunction selfCollisionFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PARTICLE_SELF_COLLISION);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(id)
			};

			{
				const PxU32 numThreadsPerBlock = 64;
				const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(selfCollisionFirstKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

				/*result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU selfCollisionFirst kernel fail!\n");*/

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU selfCollisionFirst kernel fail!\n");


				/*PxArray<PxgParticleCollisionHeader> headers;
				headers.reserve(numParticles);
				headers.forceSize_Unsafe(numParticles);
				mCudaContext->memcpyDtoH(headers.begin(), reinterpret_cast<CUdeviceptr>(particleSystem.mCollisionHeaders), sizeof(PxgParticleCollisionHeader) * numParticles);

				int bob;
				PX_UNUSED(bob);*/
#endif
			}
		}

	}


	void PxgParticleSystemCore::resetContactCounts()
	{
		CUdeviceptr totalContactCountsd = getParticleContactCount().getDevicePtr();
		mCudaContext->memsetD32Async(totalContactCountsd, 0, 1, mStream);

		*mHostContactCount = 0;
	}

	//sort contacts based on (1)rigid body index (2)particle id 
	void PxgParticleSystemCore::sortContacts(const PxU32 nbActiveParticleSystems)
	{
	
		CUdeviceptr  totalContactCountsd = mPrimitiveContactCountBuf.getDevicePtr();

		// copy contacts to host for overflow checking.
		mCudaContext->memcpyDtoHAsync(mHostContactCount, totalContactCountsd, sizeof(PxU32), mStream);

		{
			CUfunction clampFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::CLAMP_MAX_VALUE);
			PxCudaKernelParam radixSortKernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(mMaxContacts)
			};

			CUresult  resultR = mCudaContext->launchKernel(clampFunction, 1, 2, 1, 1, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU clampMaxValue fail to launch kernel!!\n");

#if PS_GPU_DEBUG
			resultR = mCudaContext->streamSynchronize(mStream);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU clampMaxValue fail!!\n");
#endif

		}

		{
			//sort particle-primitives contact based on rigid body index(lower 32 bit of the node index, which get fill in contact gen)
			CUdeviceptr inputKeyd = mTempContactByRigidBitBuf.getDevicePtr();
			CUdeviceptr inputRankd = mContactRemapSortedByRigidBuf.getDevicePtr();

			CUdeviceptr outputKeyd = mTempContactBuf.getDevicePtr();
			CUdeviceptr outputRankd = mTempContactRemapBuf.getDevicePtr();

			updateGPURadixSortBlockDesc(mStream, inputKeyd, inputRankd, outputKeyd, outputRankd, mRadixCountTotalBuf.getDevicePtr(), 
				totalContactCountsd, &mRSDesc[2 * nbActiveParticleSystems], mRadixSortDescBuf[0].getDevicePtr(), mRadixSortDescBuf[1].getDevicePtr());

			//sort particle-primitives contact based on particle index(lower 32 bit is the particle index, which get fill in contact gen)
			CUdeviceptr inputKeyd2 = mTempContactByParticleBitBuf.getDevicePtr();
			CUdeviceptr inputRankd2 = mContactRemapSortedByParticleBuf.getDevicePtr();
			//reuse the temp buffer
			CUdeviceptr outputKeyd2 = mTempContactBuf2.getDevicePtr();
			CUdeviceptr outputRankd2 = mTempContactRemapBuf2.getDevicePtr();

			updateGPURadixSortBlockDesc(mStream, inputKeyd2, inputRankd2, outputKeyd2, outputRankd2, mRadixCountTotalBuf.getDevicePtr() + mRadixCountSize,
				totalContactCountsd, &mRSDesc[2 * nbActiveParticleSystems + 2], mRadixSortDescBuf[0].getDevicePtr() + sizeof(PxgRadixSortBlockDesc),
				mRadixSortDescBuf[1].getDevicePtr() + sizeof(PxgRadixSortBlockDesc));

			PxgCudaBuffer* radixSortDescBuf = mRadixSortDescBuf.begin();

			CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK);
			CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK);

			{
				PxU32 startBit = 0;
				const PxU32 numPass = 8;

				for (PxU32 i = 0; i < numPass; ++i)
				{
					const PxU32 descIndex = i & 1;

					CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

					PxCudaKernelParam radixSortKernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(rsDesc),
						PX_CUDA_KERNEL_PARAM(startBit)
					};

					CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
					if (resultR != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

					resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
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

			CUdeviceptr contactByRigidd = mContactByRigidBuf.getDevicePtr();
			CUdeviceptr contactByParticled = mContactSortedByParticleBuf.getDevicePtr();
			{
				//copy the higher 32 bit to the temp contact rigid index buffer
				CUdeviceptr tempContactByRigidd = mTempContactByRigidBitBuf.getDevicePtr();
				CUdeviceptr rankd = mContactRemapSortedByRigidBuf.getDevicePtr();

				//copy the higher 32 bit to the temp contact particle buffer
				CUdeviceptr tempContactByParticled = mTempContactByParticleBitBuf.getDevicePtr();
				CUdeviceptr rankParticled = mContactRemapSortedByParticleBuf.getDevicePtr();

				CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_DOUBLE_COPY_HIGH_32BITS2);


				PxCudaKernelParam copyKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(contactByRigidd),
					PX_CUDA_KERNEL_PARAM(tempContactByRigidd),
					PX_CUDA_KERNEL_PARAM(rankd),
					PX_CUDA_KERNEL_PARAM(contactByParticled),
					PX_CUDA_KERNEL_PARAM(tempContactByParticled),
					PX_CUDA_KERNEL_PARAM(rankParticled),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd)
				};

				CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail to launch kernel!!\n");

#if PS_GPU_DEBUG
				resultR = mCudaContext->streamSynchronize(mStream);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopyBits fail!!\n");
#endif
			}

			{
				//sort tempContactByRidid again
				PxU32 startBit = 0;
				const PxU32 numPass = 8;

				for (PxU32 i = 0; i < numPass; ++i)
				{
					const PxU32 descIndex = i & 1;

					CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

					PxCudaKernelParam radixSortKernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(rsDesc),
						PX_CUDA_KERNEL_PARAM(startBit)
					};

					CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
					if (resultR != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

					resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 2, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
					if (resultR != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticleContacts fail to launch kernel!!\n");

					startBit += 4;
				}
			}

			{
				//copy the original rigidId to the sorted buffer based on mContactRemapSortedByRigidBuf
				CUdeviceptr outContactByRigidd = mContactSortedByRigidBuf.getDevicePtr();
				CUdeviceptr rankd = mContactRemapSortedByRigidBuf.getDevicePtr();

				CUdeviceptr outContactByParticled = mContactSortedByParticleBuf.getDevicePtr();
				CUdeviceptr rankParticled = mContactRemapSortedByParticleBuf.getDevicePtr();

				CUfunction copyFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_DOUBLE_COPY_VALUE);

				PxCudaKernelParam copyKernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(contactByRigidd),
					PX_CUDA_KERNEL_PARAM(outContactByRigidd),
					PX_CUDA_KERNEL_PARAM(rankd),
					PX_CUDA_KERNEL_PARAM(contactByParticled),
					PX_CUDA_KERNEL_PARAM(outContactByParticled),
					PX_CUDA_KERNEL_PARAM(rankParticled),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd)
				};

				CUresult resultR = mCudaContext->launchKernel(copyFunction, 32, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, mStream, copyKernelParams, sizeof(copyKernelParams), 0, PX_FL);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail to launch kernel!!\n");

#if PS_GPU_DEBUG
				resultR = mCudaContext->streamSynchronize(mStream);
				if (resultR != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU radixSortCopy fail!!\n");
#endif
			}
		}

		{
			//create sorted contacts based on mContactRemapSortedByRigidBuf and mContactRemapSortedByParticleBuf
			CUdeviceptr remapByRigidd = mContactRemapSortedByRigidBuf.getDevicePtr();
			CUdeviceptr remapByParticled = mContactRemapSortedByParticleBuf.getDevicePtr();

			CUdeviceptr sortedContactsByRigidd = mPrimitiveContactSortedByRigidBuf.getDevicePtr();
			CUdeviceptr sortedContactsByParticled = mPrimitiveContactSortedByParticleBuf.getDevicePtr();
			
			CUdeviceptr contactsd = mPrimitiveContactsBuf.getDevicePtr();

			CUfunction reorderContactsFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_REORDER_PRIMITIVE_CONTACTS);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(remapByRigidd),
				PX_CUDA_KERNEL_PARAM(remapByParticled),
				PX_CUDA_KERNEL_PARAM(sortedContactsByRigidd),
				PX_CUDA_KERNEL_PARAM(sortedContactsByParticled)
			};

			CUresult  resultR = mCudaContext->launchKernel(reorderContactsFunction, PxgParticleSystemKernelGridDim::BOUNDCELLUPDATE, 1, 1, PxgParticleSystemKernelBlockDim::BOUNDCELLUPDATE, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			if (resultR != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail to launch kernel!!\n");

#if PS_GPU_DEBUG
			CUresult result = mCudaContext->streamSynchronize(mStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail!!\n");

			//PxU32 numContacts;
			//mCudaContext->memcpyDtoH(&numContacts, totalContactCountsd, sizeof(PxU32));
			//

			/*PxArray<PxgParticlePrimitiveContact> originalContacts;
			originalContacts.reserve(numContacts);
			originalContacts.forceSize_Unsafe(numContacts);
			mCudaContext->memcpyDtoH(originalContacts.begin(), contactsd, sizeof(PxgParticlePrimitiveContact) * numContacts);

			PxArray<PxgParticlePrimitiveContact> rigidContacts;
			rigidContacts.reserve(numContacts);
			rigidContacts.forceSize_Unsafe(numContacts);
			mCudaContext->memcpyDtoH(rigidContacts.begin(), sortedContactsByRigidd, sizeof(PxgParticlePrimitiveContact) * numContacts);

			PxArray<PxgParticlePrimitiveContact> particleContacts;
			particleContacts.reserve(numContacts);
			particleContacts.forceSize_Unsafe(numContacts);
			mCudaContext->memcpyDtoH(particleContacts.begin(), sortedContactsByParticled, sizeof(PxgParticlePrimitiveContact) * numContacts);

			int bob = 0;
			PX_UNUSED(bob);*/
#endif
		}


		//KS - force work to start running!
		//mCudaContext->streamFlush(mStream);

		//At this point, we need to sync the self-stream with the stream
		//synchronizeStreams(mCudaContext, mSelfCollisionStream, mStream, mSelfCollisionEvent);

		/*CUresult result = mCudaContext->streamSynchronize(mStream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail!!\n");*/
	}

	void PxgParticleSystemCore::solveOneWayCollision(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, 
		const PxU32 nbActiveParticleSystems, const PxReal dt, const PxReal biasCoefficient, const bool isVelocityIteration)
	{

		const PxU32 maxParticles = getMaxParticles();

		if (maxParticles > 0)
		{
			const PxReal invDt = 1.0f / dt;

			const CUfunction solveSelfCollisionKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_ONEWAY_CONTACTS);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
				PX_CUDA_KERNEL_PARAM(invDt),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(biasCoefficient),
				PX_CUDA_KERNEL_PARAM(isVelocityIteration)
			};

			{
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
				const PxU32 numBlocks = (maxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(solveSelfCollisionKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solveOneWayCollision kernel fail!\n");
#endif
			}

			//updateSortedVelocity(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt);

		}
	}

	//void PxgParticleSystemCore::updateParticles(const PxReal dt)
	//{
	//	updateSortedVelocity(getParticleSystemBuffer().getDevicePtr(),
	//		getActiveParticleSystemBuffer().getDevicePtr(), mSimController->getBodySimManager().mActivePBDParticleSystems.size(), dt);
	//}


//	void PxgParticleSystemCore::solveSprings(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
//		const PxU32 nbActiveParticleSystems, const PxReal dt, bool isTGS)
//	{
//		PxgSimulationCore* simCore = mSimController->getSimulationCore();
//		const PxU32 maxSprings = simCore->getMaxParticleSprings();
//
//		if (maxSprings > 0)
//		{
//
//			//update duplicate verts
//			{
//				//each block has 1024 threads
//				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND;
//				const PxU32 numBlocks = (maxSprings * 2 + numThreadsPerBlock - 1) / numThreadsPerBlock;
//
//				PxCudaKernelParam kernelParams[] =
//				{
//					PX_CUDA_KERNEL_PARAM(particleSystemsd),
//					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd)
//				};
//
//				const CUfunction updateRemapKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_REMAP_VERTS);
//
//				CUresult result = mCudaContext->launchKernel(updateRemapKernel, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
//				PX_ASSERT(result == CUDA_SUCCESS);
//				PX_UNUSED(result);
//
//
//#if PS_GPU_DEBUG
//				{
//					result = mCudaContext->streamSynchronize(mStream);
//					if (result != CUDA_SUCCESS)
//						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateRemapVertsLaunch kernel fail!\n");
//					PX_ASSERT(result == CUDA_SUCCESS);
//				}
//#endif
//			}
//
//
//			const PxReal invDt = 1.f / dt;
//
//			{
//
//				const PxU32 maxPartitions = simCore->getMaxSpringPartitions();
//				const PxU32 maxSpringsPerPartitions = simCore->getMaxSpringsPerPartition();
//				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
//				const PxU32 numBlocks = (maxSpringsPerPartitions + numThreadsPerBlock - 1) / numThreadsPerBlock;
//
//				for (PxU32 i = 0; i < maxPartitions; ++i)
//				{
//
//					PxCudaKernelParam kernelParams[] =
//					{
//						PX_CUDA_KERNEL_PARAM(particleSystemsd),
//						PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
//						PX_CUDA_KERNEL_PARAM(nbActiveParticleSystems),
//						PX_CUDA_KERNEL_PARAM(invDt),
//						PX_CUDA_KERNEL_PARAM(i),
//						PX_CUDA_KERNEL_PARAM(isTGS)
//					};
//
//					const CUfunction solveSpringKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_SPRINGS);
//
//					CUresult result = mCudaContext->launchKernel(solveSpringKernel, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
//					PX_ASSERT(result == CUDA_SUCCESS);
//					PX_UNUSED(result);
//
//
//#if PS_GPU_DEBUG
//					{
//						result = mCudaContext->streamSynchronize(mStream);
//						if (result != CUDA_SUCCESS)
//							PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solveSpringsPGSLaunch kernel fail!\n");
//						PX_ASSERT(result == CUDA_SUCCESS);
//					}
//#endif
//				}
//			}
//
//			
//			//compute average verts and update sorted positions
//			{
//	
//				//each block has 1024 threads
//				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND;
//				const PxU32 numBlocks = (maxSprings * 2 + numThreadsPerBlock - 1) / numThreadsPerBlock;
//
//				PxCudaKernelParam kernelParams[] =
//				{
//					PX_CUDA_KERNEL_PARAM(particleSystemsd),
//					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
//					PX_CUDA_KERNEL_PARAM(invDt)
//				};
//
//				const CUfunction computeAverageKernel = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_AVERAGEVERTS);
//
//				CUresult result = mCudaContext->launchKernel(computeAverageKernel, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
//				PX_ASSERT(result == CUDA_SUCCESS);
//				PX_UNUSED(result);
//
//
//#if PS_GPU_DEBUG
//				{
//					result = mCudaContext->streamSynchronize(mStream);
//					if (result != CUDA_SUCCESS)
//						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_averageVertsLaunch fail!\n");
//					PX_ASSERT(result == CUDA_SUCCESS);
//				}
//#endif
//			}
//
//			//update sorted positions and sorted velocities
//			updateSortedVelocity(particleSystemsd, activeParticleSystemsd, nbActiveParticleSystems, dt);
//
//		}
//
//	}
//

	void PxgParticleSystemCore::applyDeltas(CUdeviceptr particleSystemd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt, CUstream stream)
	{		
		const PxU32 maxParticles = getMaxParticles();
		if (maxParticles > 0)
		{
			const PxU32 numThreadsPerBlock = 64;
			const PxU32 numBlocks = (maxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;

			{

				const CUfunction applyDeltaKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_APPLY_DELTAS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(dt)
				};

				CUresult result = mCudaContext->launchKernel(applyDeltaKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if CLOTH_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU cloth_applyExternalDeltasLaunch first pass kernel fail!\n");
#endif
			}
		}
	}

	void PxgParticleSystemCore::updateSortedVelocity(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd,
		const PxU32 nbActiveParticleSystems, const PxReal dt, const bool skipNewPositionAdjustment)
	{

		if (nbActiveParticleSystems == 0)
			return;

		
		const PxU32 maxParticles = getMaxParticles();

		if (maxParticles > 0)
		{

			//each block has 1024 threads
			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
			const PxU32 numBlocks = (maxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;

			const CUfunction updateVelocityKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_PARTICLE);

			PxReal invDt = 1.f/dt;

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
				PX_CUDA_KERNEL_PARAM(invDt),
				PX_CUDA_KERNEL_PARAM(skipNewPositionAdjustment)
			};

			{
				CUresult result = mCudaContext->launchKernel(updateVelocityKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateParticleIntermittentLaunch kernel fail!\n");

				int bob = 0;
				PX_UNUSED(bob);
#endif
			}
		}
	}

	void PxgParticleSystemCore::prepPrimitiveConstraint(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr sharedDescd,
		const PxReal dt, bool isTGS, CUstream solverStream)
	{
		
		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		CUdeviceptr totalContactCountsd = getParticleContactCount().getDevicePtr();

		//prepare primitive constraints sorted by rigid id
		{
			CUdeviceptr contactsd = mPrimitiveContactSortedByRigidBuf.getDevicePtr();
			CUdeviceptr constraintsd = mPrimitiveConstraintSortedByRigidBuf.getDevicePtr();
			CUdeviceptr appliedForced = mPrimitiveConstraintAppliedRigidForces.getDevicePtr();

			CUdeviceptr deltaVBuff = mDeltaVelRigidBuf.getDevicePtr();

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
				PX_CUDA_KERNEL_PARAM(deltaVBuff),
				PX_CUDA_KERNEL_PARAM(sharedDescd)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
			const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
			CUresult result = mCudaContext->launchKernel(prepPrimitiveCollisionKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(solverStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_contactPrepareLaunch kernel fail!\n");
#endif
		}

		{
			CUdeviceptr contactsd = mPrimitiveContactSortedByParticleBuf.getDevicePtr();
			CUdeviceptr blockOffsetd = mTempBlockCellsHistogramBuf.getDevicePtr();
			CUdeviceptr offsetd = mTempCellsHistogramBuf.getDevicePtr();
			CUdeviceptr pairCountd = mTempHistogramCountBuf.getDevicePtr();
			CUdeviceptr startd = mTempContactBuf.getDevicePtr();
			CUdeviceptr	endd = mTempContactRemapBuf.getDevicePtr();

			//compute blockOffset and offset array for particle
			{
				const CUfunction findStartEndFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_FIND_RANGESTARTEND_PARTICLE_FIRST);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemd),
					PX_CUDA_KERNEL_PARAM(contactsd),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd),
					PX_CUDA_KERNEL_PARAM(blockOffsetd),
					PX_CUDA_KERNEL_PARAM(offsetd),
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(findStartEndFirstKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_findStartEndParticleFirst kernel fail!\n");

				/*PxU32 totalNumContacts;
				mCudaContext->memcpyDtoH(&totalNumContacts, totalContactCountsd, sizeof(PxU32));

				if (totalNumContacts > 0)
				{
				PxArray<PxU32> offsets;
				offsets.reserve(totalNumContacts);
				offsets.forceSize_Unsafe(totalNumContacts);

				PxU32 blockOffset[32];
				mCudaContext->memcpyDtoH(offsets.begin(), offsetd, sizeof(PxU32) * totalNumContacts);
				mCudaContext->memcpyDtoH(blockOffset, blockOffsetd, sizeof(PxU32) * 32);

				int bob = 0;
				PX_UNUSED(bob);
				}*/
#endif
			}

			//compute start and end range for particle
			{
				const CUfunction findStartEndSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_FIND_RANGESTARTEND_PARTICLE_SECONE);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(contactsd),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd),
					PX_CUDA_KERNEL_PARAM(blockOffsetd),
					PX_CUDA_KERNEL_PARAM(offsetd),
					PX_CUDA_KERNEL_PARAM(pairCountd),
					PX_CUDA_KERNEL_PARAM(startd),
					PX_CUDA_KERNEL_PARAM(endd)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(findStartEndSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_findStartEndParticleSecond kernel fail!\n");

				/*PxU32 pairCount;
				mCudaContext->memcpyDtoH(&pairCount, pairCountd, sizeof(PxU32));

				PxArray<PxU32> rangeStart;
				rangeStart.reserve(pairCount);
				rangeStart.forceSize_Unsafe(pairCount);

				PxArray<PxU32> rangeEnd;
				rangeEnd.reserve(pairCount);
				rangeEnd.forceSize_Unsafe(pairCount);

				mCudaContext->memcpyDtoH(rangeStart.begin(), startd, sizeof(PxU32) * pairCount);
				mCudaContext->memcpyDtoH(rangeEnd.begin(), endd, sizeof(PxU32) * pairCount);

				int bob = 0;
				PX_UNUSED(bob);*/

#endif
			}
		}

	}

	namespace
	{
		PxU32 debugDmaAccumulateRigidDeltasPass1Input(PxArray<PxU64>& rigidIds, PxArray<PxVec4>& deltaVs, 
			const CUdeviceptr numIdsd, const CUdeviceptr rigidIdsd, const CUdeviceptr deltaVd)
		{
			PxU32 numIds;
			cuMemcpyDtoH(&numIds, numIdsd, sizeof(PxU32));
			rigidIds.resize(numIds);
			deltaVs.resize(numIds);
			cuMemcpyDtoH(rigidIds.begin(), rigidIdsd, sizeof(PxU64) * numIds);
			cuMemcpyDtoH(deltaVs.begin(), deltaVd, sizeof(PxVec4) * numIds);
			return numIds;
		}

		void debugComputeAccumulateRigidDeltasPass1(PxArray<PxVec4>& deltaVRef, PxArray<PxVec4>& blockDeltaVRef, PxArray<PxU64>& blockRigidBodyIdRef,
			const PxU32 numBlocks, const PxU32 numThreadsPerBlock, const PxU32 numIds, const PxArray<PxU64>& rigidIds, const PxArray<PxVec4>& deltaVs)
		{
			deltaVRef.resize(numIds);
			blockDeltaVRef.resize(numBlocks, PxVec4(0.0f));
			blockRigidBodyIdRef.resize(numBlocks, 0x8fffffffffffffff);

			//compute reference histogram, with block 
			const PxU32 nbBlocksRequired = (numIds + numThreadsPerBlock - 1) / numThreadsPerBlock;
			const PxU32 nbIterationsPerBlock = (nbBlocksRequired + numBlocks - 1) / numBlocks;

			PxU32 offset = 0;
			PxU32 blockOffset = 0;
			while (offset < numIds)
			{
				PxVec4 deltaPrev = PxVec4(0.0f);
				PxU64 rigidBodyIdPrev = 0x8fffffffffffffff;
				for (PxU32 i = 0; i < nbIterationsPerBlock * numThreadsPerBlock; ++i)
				{
					if (offset < numIds)
					{
						PxVec4 deltaIn = deltaVs[offset];
						PxVec4& deltaOut = deltaVRef[offset];
						PxU64 rigidIdIn = rigidIds[offset];
						if (rigidIdIn != rigidBodyIdPrev)
						{
							deltaPrev = PxVec4(0.0f);
						}
						deltaOut = deltaIn + deltaPrev;
						deltaPrev = deltaOut;
						rigidBodyIdPrev = rigidIdIn;
						offset++;
					}
				}
				if (blockOffset < blockRigidBodyIdRef.size() && offset < numIds)
				{
					blockDeltaVRef[blockOffset] = deltaPrev;
					blockRigidBodyIdRef[blockOffset] = rigidBodyIdPrev;
					blockOffset++;
				}
			}
		}

		PX_FORCE_INLINE bool compare(const PxReal ref, const PxReal cand, const PxReal relEps)
		{
			return PxAbs(1.0f - cand/ref) < relEps;
		}

		PX_FORCE_INLINE bool compare(const PxVec4& ref, const PxVec4& cand, const PxReal relEps)
		{
			return compare(ref.x, cand.x, relEps) &&
				   compare(ref.y, cand.y, relEps) &&
				   compare(ref.z, cand.z, relEps) &&
				   compare(ref.w, cand.w, relEps);
		}

		void debugCompareAccumulateRigidDeltasPass1(const PxArray<PxVec4>& deltaVRef, const PxArray<PxVec4>& blockDeltaVRef, const PxArray<PxU64>& blockRigidIdRef,
			const PxU32 numBlocks, const PxU32 numIds, CUdeviceptr deltaVd, CUdeviceptr blockDeltaVd, CUdeviceptr blockRigidIdd)
		{
			PxArray<PxVec4> deltaVCand(numIds);
			PxArray<PxVec4> blockDeltaVCand(numBlocks);
			PxArray<PxU64> blockRigidIdCand(numBlocks);

			cuMemcpyDtoH(deltaVCand.begin(), deltaVd, sizeof(PxVec4) * numIds);
			cuMemcpyDtoH(blockDeltaVCand.begin(), blockDeltaVd, sizeof(PxVec4) * numBlocks);
			cuMemcpyDtoH(blockRigidIdCand.begin(), blockRigidIdd, sizeof(PxNodeIndex) * numBlocks);

			const PxReal deltaRelEps = 5e-2f; // 5% is very generous, but seems to be needed in some cases
			bool compPassed = true;
			for (PxU32 i = 0; i < numIds; ++i)
			{
				const bool pass = compare(deltaVRef[i], deltaVCand[i], deltaRelEps);
				compPassed &= pass;
			}
			for (PxU32 i = 0; i < numBlocks; ++i)
			{
				const bool pass = compare(blockDeltaVRef[i], blockDeltaVCand[i], deltaRelEps) &&
								  blockRigidIdCand[i] == blockRigidIdRef[i];
				compPassed &= pass;
			}
			if (!compPassed)
			{
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas first kernel fail!\n");
			}
		}

	} // namespace

	void PxgParticleSystemCore::accumulateRigidDeltas(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, 
		CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, CUdeviceptr rigidIdsd, CUdeviceptr numIdsd, CUstream stream,
		const bool isTGS)
	{
		{
			CUdeviceptr deltaVd = mDeltaVelRigidBuf.getDevicePtr();
			CUdeviceptr blockDeltaVd = mTempBlockDeltaVelBuf.getDevicePtr();
			CUdeviceptr blockRigidIdd = mTempBlockRigidIdBuf.getDevicePtr();

#if PS_GPU_DEBUG
			//copy input data to device for CPU reference code
			PxArray<PxU64> debugRigidIds;
			PxArray<PxVec4> debugDeltaVs;
			PxU32 debugNumIds = debugDmaAccumulateRigidDeltasPass1Input(debugRigidIds, debugDeltaVs,
				numIdsd, rigidIdsd, deltaVd);
#endif

			const CUfunction primitiveCollisionFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_FIRST);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(rigidIdsd),
				PX_CUDA_KERNEL_PARAM(numIdsd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(blockDeltaVd),
				PX_CUDA_KERNEL_PARAM(blockRigidIdd)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
			const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;

#if PS_GPU_DEBUG
			//compute reference on CPU
			PxArray<PxVec4> debugDeltaVRef;
			PxArray<PxVec4> debugBlockDeltaVRef;
			PxArray<PxU64> debugBlockRigidIdRef;
			debugComputeAccumulateRigidDeltasPass1(debugDeltaVRef, debugBlockDeltaVRef, debugBlockRigidIdRef,
				numBlocks, numThreadsPerBlock, debugNumIds, debugRigidIds, debugDeltaVs);

#endif
			CUresult result = mCudaContext->launchKernel(primitiveCollisionFirstKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);

#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(stream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas first kernel fail!\n");

			debugCompareAccumulateRigidDeltasPass1(debugDeltaVRef, debugBlockDeltaVRef, debugBlockRigidIdRef,
				numBlocks, debugNumIds, deltaVd, blockDeltaVd, blockRigidIdd);
#endif
		}
		{
			CUdeviceptr deltaVd = mDeltaVelRigidBuf.getDevicePtr();
			CUdeviceptr blockDeltaVd = mTempBlockDeltaVelBuf.getDevicePtr();
			CUdeviceptr blockRigidIdd = mTempBlockRigidIdBuf.getDevicePtr();

			const CUfunction primitiveCollisionSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_SECOND);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(rigidIdsd),
				PX_CUDA_KERNEL_PARAM(numIdsd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(blockDeltaVd),
				PX_CUDA_KERNEL_PARAM(blockRigidIdd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(solverCoreDescd),
				PX_CUDA_KERNEL_PARAM(artiCoreDescd),
				PX_CUDA_KERNEL_PARAM(sharedDescd),
				PX_CUDA_KERNEL_PARAM(isTGS)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
			const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
			CUresult result = mCudaContext->launchKernel(primitiveCollisionSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(stream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas second kernel fail!\n");
#endif
		}
	}

	void PxgParticleSystemCore::solvePrimitiveCollisionForParticles(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd,
		CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, const PxReal dt, bool isTGS, const PxReal coefficient,
		bool isVelIteration)
	{
		//solve particle-primitive collision
		
		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		CUdeviceptr totalContactCountsd = getParticleContactCount().getDevicePtr();

		//const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActiveParticleSystems.size();

		//printf("--------------Iter-----------\n");

		//Particle-rigid collisions affecting particles, solved on particle stream!
		{			

			CUdeviceptr contactsd = mPrimitiveContactSortedByParticleBuf.getDevicePtr();
			CUdeviceptr constraintsd = mPrimitiveConstraintSortedByParticleBuf.getDevicePtr();
			CUdeviceptr appliedForced = mPrimitiveConstraintAppliedParticleForces.getDevicePtr();
			CUdeviceptr deltaVd = mDeltaVelParticleBuf.getDevicePtr();
			{
				const CUfunction solvePCParticleKernelFunction = isTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_PC_PARTICLE_TGS) : 
					mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_PC_PARTICLE);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemd),
					PX_CUDA_KERNEL_PARAM(contactsd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(totalContactCountsd),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(appliedForced),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(coefficient),
					PX_CUDA_KERNEL_PARAM(isVelIteration),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
				CUresult result = mCudaContext->launchKernel(solvePCParticleKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU solvePrimitiveCollisionFirst kernel fail!\n");

				PxU32 totalNumContacts;

				mCudaContext->memcpyDtoH(&totalNumContacts, totalContactCountsd, sizeof(PxU32));

				if (totalNumContacts > 0)
				{
					PxArray<PxVec4> deltaV;
					deltaV.reserve(totalNumContacts);
					deltaV.forceSize_Unsafe(totalNumContacts);

					mCudaContext->memcpyDtoH(deltaV.begin(), deltaVd, sizeof(PxVec4) * totalNumContacts);

					int bob = 0;
					PX_UNUSED(bob);
				}
#endif
				mCudaContext->eventRecord(mSolveParticleRigidEvent, mStream);
			}

			{
				//CUdeviceptr blockOffsetd = mTempBlockCellsHistogramBuf.getDevicePtr();
				//CUdeviceptr offsetd = mTempCellsHistogramBuf.getDevicePtr();
				CUdeviceptr pairCountd = mTempHistogramCountBuf.getDevicePtr();
				CUdeviceptr startd = mTempContactBuf.getDevicePtr();
				CUdeviceptr	endd = mTempContactRemapBuf.getDevicePtr();

				//accumulate deltaV changes for particle
				{
					const CUfunction accumulatedDeltaVKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_ACCUMULATE_DELTAVEL_PARTICLE);

					PxCudaKernelParam kernelParams[] =
					{
						PX_CUDA_KERNEL_PARAM(particleSystemd),
						PX_CUDA_KERNEL_PARAM(contactsd),
						PX_CUDA_KERNEL_PARAM(pairCountd),
						PX_CUDA_KERNEL_PARAM(startd),
						PX_CUDA_KERNEL_PARAM(endd),
						PX_CUDA_KERNEL_PARAM(deltaVd)
					};

					const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
					const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
					CUresult result = mCudaContext->launchKernel(accumulatedDeltaVKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
					PX_ASSERT(result == CUDA_SUCCESS);
					PX_UNUSED(result);
#if PS_GPU_DEBUG
					result = mCudaContext->streamSynchronize(mStream);
					if (result != CUDA_SUCCESS)
						PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_accumulateDeltaVParticleLaunch kernel fail!\n");

					/*PxU32 totalNumContacts;

					mCudaContext->memcpyDtoH(&totalNumContacts, totalContactCountsd, sizeof(PxU32));

					if (totalNumContacts > 0)
					{
					PxArray<PxVec4> deltaV;
					deltaV.reserve(totalNumContacts);
					deltaV.forceSize_Unsafe(totalNumContacts);

					mCudaContext->memcpyDtoH(deltaV.begin(), deltaVd, sizeof(PxVec4) * totalNumContacts);

					int bob = 0;
					PX_UNUSED(bob);
					}*/
#endif
				}

				
			}
		}

		
	}

	//this will be just called by PBD
	void PxgParticleSystemCore::solvePrimitiveCollisionForRigids(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd,
		CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, CUstream solverStream, const PxReal dt, bool isTGS, const PxReal coefficient,
		bool isVelIteration)
	{
		//solve particle-primitive collision

		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

		CUdeviceptr totalContactCountsd = getParticleContactCount().getDevicePtr();

		//Particle-rigid particle collisions, affecting rigids, solved on solverStream
		if (1)
		{
			CUdeviceptr contactsd = mPrimitiveContactSortedByRigidBuf.getDevicePtr();
			CUdeviceptr constraintsd = mPrimitiveConstraintSortedByRigidBuf.getDevicePtr();
			CUdeviceptr appliedForced = mPrimitiveConstraintAppliedRigidForces.getDevicePtr();
			CUdeviceptr deltaVd = mDeltaVelRigidBuf.getDevicePtr();

			const CUfunction solvePCRigidKernelFunction = isTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_PC_RIGID_TGS)
				: mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_PC_RIGID);

			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemd),
				PX_CUDA_KERNEL_PARAM(contactsd),
				PX_CUDA_KERNEL_PARAM(constraintsd),
				PX_CUDA_KERNEL_PARAM(totalContactCountsd),
				PX_CUDA_KERNEL_PARAM(prePrepDescd),
				PX_CUDA_KERNEL_PARAM(solverCoreDescd),
				PX_CUDA_KERNEL_PARAM(sharedDescd),
				PX_CUDA_KERNEL_PARAM(deltaVd),
				PX_CUDA_KERNEL_PARAM(appliedForced),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(coefficient),
				PX_CUDA_KERNEL_PARAM(isVelIteration),
				PX_CUDA_KERNEL_PARAM(artiCoreDescd)
			};

			const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
			const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
			CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);
#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(solverStream);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solvePCOutputRigidDeltaVLaunch kernel fail!\n");

			/*PxU32 totalNumContacts;

			mCudaContext->memcpyDtoH(&totalNumContacts, totalContactCountsd, sizeof(PxU32));

			if (totalNumContacts > 0)
			{
				PxArray<PxVec4> deltaV;
				deltaV.reserve(totalNumContacts);
				deltaV.forceSize_Unsafe(totalNumContacts);

				mCudaContext->memcpyDtoH(deltaV.begin(), deltaVd, sizeof(PxVec4) * totalNumContacts);

				int bob = 0;
				PX_UNUSED(bob);
			}*/
#endif

			mCudaContext->eventRecord(mSolveRigidParticleEvent, solverStream);

			mCudaContext->streamWaitEvent(solverStream, mSolveParticleRigidEvent);

			
			accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd,
				mContactSortedByRigidBuf.getDevicePtr(), totalContactCountsd, solverStream,
				isTGS);
		}

		mCudaContext->streamWaitEvent(mStream, mSolveRigidParticleEvent);

	}


	void PxgParticleSystemCore::solveRigidAttachments(CUdeviceptr prePrepDescd, CUdeviceptr solverCoreDescd, 
		CUdeviceptr sharedDescd, CUdeviceptr artiCoreDescd, CUstream solverStream, const PxReal dt, const bool isTGS,
		const PxReal biasCoefficient, const bool isVelocityIteration,
		CUdeviceptr particleSystemd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems)
	{
		//const PxU32 nbRigidAttachments = simCore->getNbRigidParticleAttachments();
		const PxU32 nbRigidAttachments = mTotalRigidAttachments;

		if (nbRigidAttachments)
		{

			//CUdeviceptr particleSystemd = simCore->getParticleSystemBuffer().getDevicePtr();

			CUdeviceptr constraintsd = mParticleRigidConstraints.getDevicePtr();
			CUdeviceptr rigidAttachmentIds = mParticleRigidAttachmentIds.getDevicePtr();
			CUdeviceptr deltaVd = mDeltaVelRigidBuf.getDevicePtr();
			CUdeviceptr totalRigidAttachmentsd = mParticleRigidConstraintCount.getDevicePtr();

			{
				const CUfunction solvePCRigidKernelFunction = isTGS ? mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_RIGID_ATTACHMENTS_TGS)
					: mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_SOLVE_RIGID_ATTACHMENTS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemd),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(nbRigidAttachments),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(biasCoefficient),
					PX_CUDA_KERNEL_PARAM(isVelocityIteration)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
				CUresult result = mCudaContext->launchKernel(solvePCRigidKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_solvePCOutputRigidDeltaVLaunch kernel fail!\n");

				/*PxU32 totalNumContacts;

				mCudaContext->memcpyDtoH(&totalNumContacts, totalContactCountsd, sizeof(PxU32));

				if (totalNumContacts > 0)
				{
				PxArray<PxVec4> deltaV;
				deltaV.reserve(totalNumContacts);
				deltaV.forceSize_Unsafe(totalNumContacts);

				mCudaContext->memcpyDtoH(deltaV.begin(), deltaVd, sizeof(PxVec4) * totalNumContacts);

				int bob = 0;
				PX_UNUSED(bob);
				}*/
#endif
			}

			/*accumulateRigidDeltas(prePrepDescd, solverCoreDescd, sharedDescd, artiCoreDescd,
				rigidAttachmentIds, totalRigidAttachmentsd, solverStream, true, 1.f, isTGS);*/


			{
				CUdeviceptr blockDeltaVd = mTempBlockDeltaVelBuf.getDevicePtr();
				CUdeviceptr blockRigidIdd = mTempBlockRigidIdBuf.getDevicePtr();

				const CUfunction primitiveCollisionFirstKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_FIRST);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(totalRigidAttachmentsd),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(blockDeltaVd),
					PX_CUDA_KERNEL_PARAM(blockRigidIdd)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(primitiveCollisionFirstKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas first kernel fail!\n");
#endif
			}

			PxReal globalRelaxationCoefficient = 1.f;
			bool useLocalRelax = true;

			{
				CUdeviceptr tempDenomd = mParticleRigidAttachmentScaleBuffer.getDevicePtr();


				const CUfunction primitiveCollisionSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_MULTI_CLEAR);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(totalRigidAttachmentsd),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(tempDenomd)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(primitiveCollisionSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas second kernel fail!\n");
#endif
			}

			{
				CUdeviceptr blockDeltaVd = mTempBlockDeltaVelBuf.getDevicePtr();
				CUdeviceptr blockRigidIdd = mTempBlockRigidIdBuf.getDevicePtr();
				CUdeviceptr tempDenomd = mParticleRigidAttachmentScaleBuffer.getDevicePtr();



				const CUfunction primitiveCollisionSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_SECOND_MULTI1);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(totalRigidAttachmentsd),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(blockDeltaVd),
					PX_CUDA_KERNEL_PARAM(blockRigidIdd),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(tempDenomd),
					PX_CUDA_KERNEL_PARAM(useLocalRelax),
					PX_CUDA_KERNEL_PARAM(globalRelaxationCoefficient),
					PX_CUDA_KERNEL_PARAM(isTGS)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(primitiveCollisionSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas second kernel fail!\n");
#endif
			}

			{
				CUdeviceptr blockDeltaVd = mTempBlockDeltaVelBuf.getDevicePtr();
				CUdeviceptr blockRigidIdd = mTempBlockRigidIdBuf.getDevicePtr();
				CUdeviceptr tempDenomd = mParticleRigidAttachmentScaleBuffer.getDevicePtr();



				const CUfunction primitiveCollisionSecondKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::ACCUMULATE_DELTAVEL_RIGIDBODY_SECOND_MULTI2);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(totalRigidAttachmentsd),
					PX_CUDA_KERNEL_PARAM(deltaVd),
					PX_CUDA_KERNEL_PARAM(blockDeltaVd),
					PX_CUDA_KERNEL_PARAM(blockRigidIdd),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(solverCoreDescd),
					PX_CUDA_KERNEL_PARAM(artiCoreDescd),
					PX_CUDA_KERNEL_PARAM(sharedDescd),
					PX_CUDA_KERNEL_PARAM(tempDenomd),
					PX_CUDA_KERNEL_PARAM(useLocalRelax),
					PX_CUDA_KERNEL_PARAM(globalRelaxationCoefficient),
					PX_CUDA_KERNEL_PARAM(isTGS)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
				const PxU32 numBlocks = PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;
				CUresult result = mCudaContext->launchKernel(primitiveCollisionSecondKernelFunction, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, 0, solverStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(solverStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU accumRigidDeltas second kernel fail!\n");
#endif
			}


			/*const PxU32 nbActiveParticleSystems = mSimController->getBodySimManager().mActiveParticleSystems.size();
			PxgSimulationCore* core = mSimController->getSimulationCore();

			CUdeviceptr activeParticleSystemd = core->getActiveParticleSystemBuffer().getDevicePtr();*/

			applyDeltas(particleSystemd, activeParticleSystemd, nbActiveParticleSystems, dt, solverStream);

		}
	}



	void PxgParticleSystemCore::prepRigidAttachments(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd,
		bool isTGS, const PxReal dt, CUstream stream, const PxU32 nbActiveParticleSystems, CUdeviceptr activeParticleSystemsD,
		PxU32 numSolverBodies)
	{
		if (mMaxRigidAttachmentsPerSystem)
		{
			mParticleRigidAttachmentScaleBuffer.allocate(sizeof(PxReal) * numSolverBodies, PX_FL);
			CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

			CUdeviceptr constraintsd = mParticleRigidConstraints.getDevicePtr(); 
			CUdeviceptr rigidAttachmentIds = mParticleRigidAttachmentIds.getDevicePtr();

			PxU32 maxRigidAttachments = mMaxRigidAttachmentsPerSystem;
			PxU32 nbSystems = nbActiveParticleSystems;

			//prepare primitive constraints sorted by particle id
			{
				const CUfunction prepAttachmentKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PREP_RIGID_ATTACHMENTS);

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemd),
					PX_CUDA_KERNEL_PARAM(rigidAttachmentIds),
					PX_CUDA_KERNEL_PARAM(constraintsd),
					PX_CUDA_KERNEL_PARAM(prePrepDescd),
					PX_CUDA_KERNEL_PARAM(prepDescd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(isTGS),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsD)
				};

				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_COLLISION;
				//const PxU32 numBlocks = PxgParticleSystemKernelGridDim::PS_COLLISION;
				const PxU32 numBlocks = (maxRigidAttachments + PxgParticleSystemKernelBlockDim::PS_COLLISION - 1) / PxgParticleSystemKernelBlockDim::PS_COLLISION;
				CUresult result = mCudaContext->launchKernel(prepAttachmentKernelFunction, numBlocks, nbSystems, 1, numThreadsPerBlock, 1, 1, 0, stream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mStream);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU prepRigidAttachments ps_rigidAttachmentPrepareLaunch kernel fail!\n");
				PX_ASSERT(result == CUDA_SUCCESS);
#endif
			}
		}
	}

	void PxgParticleSystemCore::prepParticleConstraint(CUdeviceptr prePrepDescd, CUdeviceptr prepDescd, CUdeviceptr sharedDescd,
		bool isTGS, const PxReal dt)
	{
		CUdeviceptr particleSystemd = getParticleSystemBuffer().getDevicePtr();

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
	}
	
	void PxgParticleSystemCore::stepParticleSystems(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticleSystems,
		const PxReal dt, const PxReal totalInvDt, bool isFirstIteration)
	{
		const bool isTGS = mGpuContext->isTGS();
		const bool externalForcesEveryTgsIterationEnabled = mGpuContext->isExternalForcesEveryTgsIterationEnabled();

		if(mMaxParticles > 0)
		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
				PX_CUDA_KERNEL_PARAM(dt),
				PX_CUDA_KERNEL_PARAM(totalInvDt),
				PX_CUDA_KERNEL_PARAM(isFirstIteration),
				PX_CUDA_KERNEL_PARAM(isTGS),
			    PX_CUDA_KERNEL_PARAM(externalForcesEveryTgsIterationEnabled),
				PX_CUDA_KERNEL_PARAM(mGravity)
			};

			CUfunction stepKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_STEP_PARTICLES);
			{
				const PxU32 numThreadsPerBlock = PxgParticleSystemKernelBlockDim::PS_SOLVE;
				const PxU32 numBlocks = (mMaxParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
				CUresult result = mCudaContext->launchKernel(stepKernelFunction, numBlocks, nbActiveParticleSystems, 1, numThreadsPerBlock, 1, 1, 0, mStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);
			}

#if PS_GPU_DEBUG
			CUresult result = mCudaContext->streamSynchronize(mStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_stepParticleSystemLaunch kernel fail!\n");
#endif
		}

	}

	void PxgParticleSystemCore::integrateSystem(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemd, const PxU32 nbActiveParticleSystems, const PxReal dt, const PxReal epsilonSq)
	{
		const PxU32 maxParticles = getMaxParticles();

		if (maxParticles)
		{
			const CUfunction integrateKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_INTEGRATION);
			
			{
				const PxU32 numBlocks = (maxParticles + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;

				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemd),
					PX_CUDA_KERNEL_PARAM(dt),
					PX_CUDA_KERNEL_PARAM(epsilonSq)
				};

				CUresult result = mCudaContext->launchKernel(integrateKernelFunction, numBlocks, nbActiveParticleSystems, 1, PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1, 1, 0, mFinalizeStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);

				PX_UNUSED(result);

#if PS_GPU_DEBUG
				result = mCudaContext->streamSynchronize(mFinalizeStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU integrateSystem kernel fail!\n");
#endif
			}
		}
	}

	void PxgParticleSystemCore::copyUnsortedArrayToUserBuffer(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles)
	{
		static PxU32 count = 0;
		count++;

		const CUfunction preIntegrateKernelFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_UPDATE_BUFFER_DATA);

		const PxU32 numBlocks = (mMaxParticlesPerBuffer + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;

		if (numBlocks)
		{
			PxCudaKernelParam kernelParams[] =
			{
				PX_CUDA_KERNEL_PARAM(particleSystemsd),
				PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
				PX_CUDA_KERNEL_PARAM(count)
			};


			CUresult result = mCudaContext->launchKernel(preIntegrateKernelFunction, numBlocks, mMaxBuffersPerSystem, nbActiveParticles, PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1, 1, 0, mFinalizeStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
			PX_ASSERT(result == CUDA_SUCCESS);
			PX_UNUSED(result);


#if PS_GPU_DEBUG
			result = mCudaContext->streamSynchronize(mFinalizeStream);
			PX_ASSERT(result == CUDA_SUCCESS);
			if (result != CUDA_SUCCESS)
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_updateBufferLaunch kernel fail!\n");
#endif
		}
	}

	void PxgParticleSystemCore::copyUserBufferDataToHost(PxgParticleSystem* particleSystems, PxU32* activeParticleSystems, PxU32 nbActiveParticleSystems)
	{
		const PxU32* particleSystemNodeIndex = mParticleSystemNodeIndexPool.begin();
		void** bodySimsLL = mSimController->getBodySimManager().mBodies.begin();

		for (PxU32 i = 0; i < nbActiveParticleSystems; ++i)
		{
			const PxU32 index = activeParticleSystems[i];
			PxgParticleSystem& particleSystem = particleSystems[index];
			Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[particleSystemNodeIndex[index]]);
			Dy::ParticleSystemCore& dyParticleSystemCore = dyParticleSystem->getCore();

			if ((dyParticleSystem->mFlag & Dy::ParticleSystemFlag::eENABLE_GPU_DATA_SYNC) && (particleSystem.mCommonData.mNumParticles > 0))
			{
				for (PxU32 b = 0; b < dyParticleSystemCore.mParticleBuffers.size(); ++b)
				{
					PxgParticleBuffer* particleBuffer = static_cast<PxgParticleBuffer*>(dyParticleSystemCore.mParticleBuffers[b]);
					particleBuffer->copyToHost(mFinalizeStream);
				}
				for (PxU32 b = 0; b < dyParticleSystemCore.mParticleDiffuseBuffers.size(); ++b)
				{
					PxgParticleAndDiffuseBuffer* particleDiffuseBuffer = static_cast<PxgParticleAndDiffuseBuffer*>(dyParticleSystemCore.mParticleDiffuseBuffers[b]);
					particleDiffuseBuffer->copyToHost(mFinalizeStream);
				}
				for (PxU32 b = 0; b < dyParticleSystemCore.mParticleClothBuffers.size(); ++b)
				{
					PxgParticleClothBuffer* particleClothBuffer = static_cast<PxgParticleClothBuffer*>(dyParticleSystemCore.mParticleClothBuffers[b]);
					particleClothBuffer->copyToHost(mFinalizeStream);
				}
				for (PxU32 b = 0; b < dyParticleSystemCore.mParticleRigidBuffers.size(); ++b)
				{
					PxgParticleRigidBuffer* particleRigidBuffer = static_cast<PxgParticleRigidBuffer*>(dyParticleSystemCore.mParticleRigidBuffers[b]);
					particleRigidBuffer->copyToHost(mFinalizeStream);
				}
			}
		}
	}

	void PxgParticleSystemCore::getMaxIterationCount(PxgBodySimManager& bodySimManager, const PxU32 nbActiveParticles, const PxU32* activeParticles, PxI32& maxPosIters, PxI32& maxVelIters)
	{
		void** bodySimsLL = bodySimManager.mBodies.begin();

		PxU32* particleNodeIndex = mParticleSystemNodeIndexPool.begin();

		for (PxU32 i = 0; i < nbActiveParticles; ++i)
		{
			const PxU32 index = activeParticles[i];
			const PxU32 nodeIdex = particleNodeIndex[index];
			Dy::ParticleSystem* dyParticleSystem = reinterpret_cast<Dy::ParticleSystem*>(bodySimsLL[nodeIdex]);

			PxU16 solverIterationCounts = dyParticleSystem->getIterationCounts();

			maxPosIters = PxMax(PxI32(solverIterationCounts & 0xff), maxPosIters);
			maxVelIters = PxMax(PxI32(solverIterationCounts >> 8), maxVelIters);
		}
	}

	void PxgParticleSystemCore::updateParticleSystemData(PxgParticleSystem& sys, Dy::ParticleSystemCore& dyParticleSystemCore)
	{
		const PxReal particleContactDistance = 2.f * dyParticleSystemCore.particleContactOffset;
		const PxReal fluidRestDistance = 2.f * dyParticleSystemCore.fluidRestOffset;

		if (dyParticleSystemCore.particleContactOffset_prev != dyParticleSystemCore.particleContactOffset ||
			dyParticleSystemCore.fluidRestOffset_prev != dyParticleSystemCore.fluidRestOffset)
		{
			PxReal fluidRestDensity;
			PxReal fluidDensityConstraintScale;
			PxReal fluidSurfaceConstraintScale;

			// TOFIX: Hack params to known values
			CalculateRestDensity(
				fluidRestDistance,
				particleContactDistance,
				fluidRestDensity,
				fluidDensityConstraintScale,
				fluidSurfaceConstraintScale
			);

			sys.mCommonData.mParticleContactDistance = particleContactDistance;
			sys.mCommonData.mParticleContactDistanceInv = 1.0f / particleContactDistance;
			sys.mCommonData.mParticleContactDistanceSq = particleContactDistance * particleContactDistance;

			// norm for visocelastic spike (from viscoelastic fluids)
			sys.mData.mSpiky1 = 15.0f / (kPi * particleContactDistance * particleContactDistance * particleContactDistance);
			sys.mData.mSpiky2 = 30.0f / (kPi * particleContactDistance * particleContactDistance * particleContactDistance * particleContactDistance);

			sys.mData.mRestDensity = fluidRestDensity;
			sys.mData.mFluidSurfaceConstraintScale = fluidSurfaceConstraintScale;
			sys.mData.mInvRestDensity = 1.0f / fluidRestDensity;
			sys.mData.mLambdaScale = 1.0f / fluidDensityConstraintScale;

			dyParticleSystemCore.particleContactOffset_prev = dyParticleSystemCore.particleContactOffset;
			dyParticleSystemCore.fluidRestOffset_prev = dyParticleSystemCore.fluidRestOffset;
		}
		sys.mData.mRestDensityBoundary = sys.mData.mRestDensity*dyParticleSystemCore.fluidBoundaryDensityScale;

		sys.mData.mRestOffset = dyParticleSystemCore.restOffset;
		sys.mData.mFluidRestOffset = dyParticleSystemCore.fluidRestOffset;
		sys.mData.mSolidRestOffset = dyParticleSystemCore.solidRestOffset;

		//data.mRelaxationFactor = 1.0f / (1.0f + relaxationFactor);
		sys.mData.mRelaxationFactor = 1.0f;

		sys.mData.mFlags = dyParticleSystemCore.mFlags;
	    sys.mData.mLockFlags = PxU16(dyParticleSystemCore.mLockFlags);

		sys.mData.mWind = dyParticleSystemCore.mWind;

		sys.mData.mNumPhaseToMaterials = dyParticleSystemCore.mPhaseGroupToMaterialHandle.size();

		sys.mData.mMaxVelocity = dyParticleSystemCore.maxVelocity;
	}

	void PxgParticleSystemCore::gpuDMAActiveParticleIndices(const PxU32* activeParticleSystems, const PxU32 numActiveParticleSystems, CUstream stream)
	{
		PX_PROFILE_ZONE("gpuDMAActiveParticleIndices", 0);
		mActiveParticleSystemBuffer.allocate(sizeof(PxU32) * numActiveParticleSystems, PX_FL);
		mCudaContext->memcpyHtoDAsync(mActiveParticleSystemBuffer.getDevicePtr(), activeParticleSystems, sizeof(PxU32) * numActiveParticleSystems, stream);
		
	}

	void PxgParticleSystemCore::releaseInternalParticleSystemDataBuffer()
	{
		for (PxU32 i = 0; i < mParticleSystemDataBuffer.size(); ++i)
		{
			if (mParticleSystemDataBuffer[i])
			{
				mParticleSystemDataBuffer[i]->~PxgParticleSystemBuffer();
				PX_FREE(mParticleSystemDataBuffer[i]);
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////

	PxgDiffuseParticleCore::PxgDiffuseParticleCore(PxgEssentialCore* core) :
		mEssentialCore(core),
		mDiffuseParticlesRandomTableBuf(core->mHeapMemoryManager, PxsHeapStats::eSHARED_PARTICLES)
	{
		mRandomTableSize = 1024;
		mMaxDiffuseParticles = 0;
	}

	PxgDiffuseParticleCore::~PxgDiffuseParticleCore()
	{
	}

	void PxgDiffuseParticleCore::releaseInternalDiffuseParticleDataBuffer()
	{
		for (PxU32 i = 0; i < mDiffuseParticleDataBuffer.size(); ++i)
		{
			if (mDiffuseParticleDataBuffer[i])
			{
				mDiffuseParticleDataBuffer[i]->~PxgParticleSystemDiffuseBuffer();
				PX_FREE(mDiffuseParticleDataBuffer[i]);
			}
		}
	}

	void PxgDiffuseParticleCore::preDiffuseIntegrateSystem(CUdeviceptr particleSystemsd, CUdeviceptr activeParticleSystemsd, const PxU32 nbActiveParticles, const PxVec3 gravity,
		const PxReal dt, CUstream bpStream)
	{
		if (mMaxDiffuseParticles)
		{
			
			const CUfunction preIntegrateKernelFunction = mEssentialCore->mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::PS_PRE_DIFFUSE_INTEGRATION);

			const PxU32 numBlocks = (mMaxDiffuseParticles + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;

			{
				PxCudaKernelParam kernelParams[] =
				{
					PX_CUDA_KERNEL_PARAM(particleSystemsd),
					PX_CUDA_KERNEL_PARAM(activeParticleSystemsd),
					PX_CUDA_KERNEL_PARAM(gravity),
					PX_CUDA_KERNEL_PARAM(dt)
				};

				//one  blockIdx.z to indicate normal particles or diffuse particles
				CUresult result = mEssentialCore->mCudaContext->launchKernel(preIntegrateKernelFunction, numBlocks, nbActiveParticles, 1, PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1, 1, 0, bpStream, kernelParams, sizeof(kernelParams), 0, PX_FL);
				PX_ASSERT(result == CUDA_SUCCESS);
				PX_UNUSED(result);


#if PS_GPU_DEBUG
				result = mEssentialCore->mCudaContext->streamSynchronize(bpStream);
				PX_ASSERT(result == CUDA_SUCCESS);
				if (result != CUDA_SUCCESS)
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU ps_preDiffuseIntegrateLaunch kernel fail!\n");
#endif
			}
		}

	}

	void PxgDiffuseParticleCore::resizeDiffuseParticleParticleBuffers(PxgParticleSystem& particleSystem, PxgParticleSystemDiffuseBuffer* buffer, const PxU32 numParticles)
	{
		buffer->diffuse_potentials.allocate(numParticles * sizeof(float2), PX_FL);

		particleSystem.mDiffusePotentials = reinterpret_cast<float2*>(buffer->diffuse_potentials.getDevicePtr());
	}

}

//#pragma optimize("", on)
