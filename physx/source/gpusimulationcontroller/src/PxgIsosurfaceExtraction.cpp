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

#include "PxgIsosurfaceExtraction.h"

#include "foundation/PxUserAllocated.h"
#include "PxgCudaMemoryAllocator.h"

#include "PxPhysXGpu.h"
#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgCudaMemoryAllocator.h"
#include "PxgCudaHelpers.h"


namespace physx
{
#if ENABLE_KERNEL_LAUNCH_ERROR_CHECK
#define checkCudaError() { cudaError_t err = cudaDeviceSynchronize(); if (err != 0) printf("Cuda error file: %s, line: %i, error: %i\n", PX_FL, err); }	
#else
#define checkCudaError() { }
#endif

#define THREADS_PER_BLOCK 256


	void sparseGridClearDensity(PxgKernelLauncher& launcher, const PxSparseGridParams& sparseGridParams, PxReal* density, PxReal clearValue, PxU32* numActiveSubgrids, CUstream stream)
	{
		PxU32 subgridSize = sparseGridParams.subgridSizeX * sparseGridParams.subgridSizeY * sparseGridParams.subgridSizeZ;
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (sparseGridParams.maxNumSubgrids * subgridSize + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::sg_SparseGridClearDensity, numBlocks, numThreadsPerBlock, 0, stream,
			density, clearValue, numActiveSubgrids, subgridSize);
		checkCudaError();
	}

	void computeParticleDensityLaunchUsingSDF(PxgKernelLauncher& launcher, PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxIsosurfaceExtractionData& data, CUstream stream, PxU32* activeIndices = NULL,
		PxVec4* anisotropy1 = NULL, PxVec4* anisotropy2 = NULL, PxVec4* anisotropy3 = NULL, PxReal anisotropyFactor = 1.0f)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_ComputeParticleDensityUsingSDF, numBlocks, numThreadsPerBlock, 0, stream,
			deviceParticlePos, numParticles, phases, validPhaseMask, data,
			activeIndices, reinterpret_cast<float4*>(anisotropy1), reinterpret_cast<float4*>(anisotropy2), reinterpret_cast<float4*>(anisotropy3), anisotropyFactor);
		checkCudaError();
	}

	void computeParticleDensityLaunch(PxgKernelLauncher& launcher, PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_ComputeParticleDensity, numBlocks, numThreadsPerBlock, 0, stream,
			deviceParticlePos, numParticles, phases, validPhaseMask, data);
		checkCudaError();
	}

	void countCellVertsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CountCellVerts, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void createVertsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CreateVerts, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void countTriIdsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CountTriIds, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void createTriIdsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, bool flipTriangleOrientation, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CreateTriIds, numBlocks, numThreadsPerBlock, 0, stream, data, flipTriangleOrientation ? 1 : 0);
		checkCudaError();
	}

	void smoothVertsLaunch(PxgKernelLauncher& launcher, const PxVec4* vertices, PxVec4* output, const PxU32* triIds, const PxU32* numTriIds, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_SmoothVerts, numBlocks, numThreadsPerBlock, 0, stream, vertices, output, triIds, numTriIds);
		checkCudaError();
	}

	void averageVertsLaunch(PxgKernelLauncher& launcher, PxVec4* vertices, PxVec4* output, const PxU32* length, CUstream stream, PxReal blendWeight = 1.0f)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_AverageVerts, numBlocks, numThreadsPerBlock, 0, stream, vertices, output, length, blendWeight);
		checkCudaError();
	}

	void computeNormalsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_ComputeNormals, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void normalizeNormalsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_NormalizeNormals, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void smoothNormalsLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_SmoothNormals, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void smoothNormalsNormalizeLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_SmoothNormalsNormalize, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void gridFilterGaussLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, PxReal neighborWeight, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_GridFilterGauss, numBlocks, numThreadsPerBlock, 0, stream, data, neighborWeight);
		data.swapState = 1 - data.swapState;
		checkCudaError();
	}

	void gridFilterDilateErodeLaunch(PxgKernelLauncher& launcher, PxIsosurfaceExtractionData& data, PxReal sign, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_GridFilterDilateErode, numBlocks, numThreadsPerBlock, 0, stream, data, sign);
		data.swapState = 1 - data.swapState;
		checkCudaError();
	}




	void computeParticleDensityLaunchUsingSDF(PxgKernelLauncher& launcher, PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxSparseIsosurfaceExtractionData& data, CUstream stream, PxU32* activeIndices = NULL,
		PxVec4* anisotropy1 = NULL, PxVec4* anisotropy2 = NULL, PxVec4* anisotropy3 = NULL, PxReal anisotropyFactor = 1.0f)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_ComputeParticleDensityUsingSDFSparse, numBlocks, numThreadsPerBlock, 0, stream, deviceParticlePos, numParticles, phases, validPhaseMask, data,
			activeIndices, reinterpret_cast<float4*>(anisotropy1), reinterpret_cast<float4*>(anisotropy2), reinterpret_cast<float4*>(anisotropy3), anisotropyFactor);
		checkCudaError();
	}

	void computeParticleDensityLaunch(PxgKernelLauncher& launcher, PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (numParticles + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_ComputeParticleDensitySparse, numBlocks, numThreadsPerBlock, 0, stream, deviceParticlePos, numParticles, phases, validPhaseMask, data);
		checkCudaError();
	}

	void countCellVertsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CountCellVertsSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void createVertsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CreateVertsSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void countTriIdsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CountTriIdsSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void createTriIdsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, bool flipTriangleOrientation, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_CreateTriIdsSparse, numBlocks, numThreadsPerBlock, 0, stream, data, flipTriangleOrientation ? 1 : 0);
		checkCudaError();
	}

	void computeNormalsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 16;
		launcher.launchKernel(PxgKernelIds::iso_ComputeNormalsSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void normalizeNormalsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 16;
		launcher.launchKernel(PxgKernelIds::iso_NormalizeNormalsSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}


	void smoothNormalsLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_SmoothNormalsSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}

	void smoothNormalsNormalizeLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = 64;
		launcher.launchKernel(PxgKernelIds::iso_SmoothNormalsNormalizeSparse, numBlocks, numThreadsPerBlock, 0, stream, data);
		checkCudaError();
	}



	void gridFilterGaussLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, PxReal neighborWeight, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_GridFilterGaussSparse, numBlocks, numThreadsPerBlock, 0, stream, data, neighborWeight);
		data.swapState = 1 - data.swapState;
		checkCudaError();
	}

	void gridFilterDilateErodeLaunch(PxgKernelLauncher& launcher, PxSparseIsosurfaceExtractionData& data, PxReal sign, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = THREADS_PER_BLOCK;
		const PxU32 numBlocks = (data.maxNumCells() + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::iso_GridFilterDilateErodeSparse, numBlocks, numThreadsPerBlock, 0, stream, data, sign);
		data.swapState = 1 - data.swapState;
		checkCudaError();
	}


		

	void PxgSparseGridIsosurfaceExtractor::setMaxVerticesAndTriangles(PxU32 maxIsosurfaceVertices, PxU32 maxIsosurfaceTriangles)
	{
		bool vertexCountChanged = mData.maxVerts != maxIsosurfaceVertices;
		bool indexCountChannged = mData.maxTriIds != maxIsosurfaceTriangles * 3;
		if (vertexCountChanged)
		{
			mData.maxVerts = maxIsosurfaceVertices;
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.smoothingBuffer);
			mData.smoothingBuffer = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		}
		if (indexCountChannged)
			mData.maxTriIds = maxIsosurfaceTriangles * 3;

		if (!mShared.mOwnsOutputGPUBuffers)
			return;
		if (vertexCountChanged)
		{
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.verts);
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.normals);
			mData.verts = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
			mData.normals = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		}
		if (indexCountChannged)
		{
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.triIds);
			mData.triIds = PX_DEVICE_MEMORY_ALLOC(PxU32, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxTriIds);
		}
	}

	void PxgSparseGridIsosurfaceExtractor::releaseGPUBuffers()
	{
		if (!mData.verts)
			return;

		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.verts);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.normals);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.triIds);
	}

	void PxgSparseGridIsosurfaceExtractor::allocateGPUBuffers()
	{
		if (mData.verts)
			return;

		mData.verts = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		mData.normals = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		mData.triIds = PX_DEVICE_MEMORY_ALLOC(PxU32, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxTriIds);
	}

	void PxgSparseGridIsosurfaceExtractor::setResultBufferDevice(PxVec4* vertices, PxU32* triIndices, PxVec4* normals)
	{
		if (mShared.mOwnsOutputGPUBuffers)
			releaseGPUBuffers();
		mShared.mOwnsOutputGPUBuffers = false;
		mData.verts = vertices;
		mData.normals = normals;
		mData.triIds = triIndices;
		mShared.mVertices = NULL;
		mShared.mTriIndices = NULL;
		mShared.mNormals = NULL;
	}

	void PxgSparseGridIsosurfaceExtractor::setMaxParticles(PxU32 maxParticles)
	{
		PxSparseGridParams sparseGridParams = mSparseGrid.getGridParameters();

		bool ownsGpuBuffers = mShared.mOwnsOutputGPUBuffers;
		mShared.mOwnsOutputGPUBuffers = false; //The following call to initialize will release all existing owned memory. If the output buffers are marked as not-owned, they will persist.
		initialize(mShared.mKernelLauncher, sparseGridParams, mShared.mIsosurfaceParams, maxParticles, mData.maxVerts, mData.maxTriIds / 3);
		mShared.mOwnsOutputGPUBuffers = ownsGpuBuffers;
	}

	void PxgSparseGridIsosurfaceExtractor::setResultBufferHost(PxVec4* vertices, PxU32* triIndices, PxVec4* normals)
	{
		if (mShared.mOwnsOutputGPUBuffers)
			releaseGPUBuffers();
		mShared.mOwnsOutputGPUBuffers = true;
		mShared.mVertices = vertices;
		mShared.mTriIndices = triIndices;
		mShared.mNormals = normals;
		allocateGPUBuffers();
	}

	void PxgDenseGridIsosurfaceExtractor::setMaxVerticesAndTriangles(PxU32 maxIsosurfaceVertices, PxU32 maxIsosurfaceTriangles)
	{
		bool vertexCountChanged = mData.maxVerts != maxIsosurfaceVertices;
		bool indexCountChannged = mData.maxTriIds != maxIsosurfaceTriangles * 3;
		if (vertexCountChanged)
		{
			mData.maxVerts = maxIsosurfaceVertices;
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.smoothingBuffer);
			mData.smoothingBuffer = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		}
		if (indexCountChannged)
			mData.maxTriIds = maxIsosurfaceTriangles * 3;

		if (!mShared.mOwnsOutputGPUBuffers)
			return;
		if (vertexCountChanged)
		{
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.verts);
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.normals);
			mData.verts = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
			mData.normals = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		}
		if (indexCountChannged)
		{
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.triIds);
			mData.triIds = PX_DEVICE_MEMORY_ALLOC(PxU32, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxTriIds);
		}
	}

	void PxgDenseGridIsosurfaceExtractor::releaseGPUBuffers()
	{
		if (!mData.verts)
			return;

		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.verts);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.normals);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.triIds);
	}

	void PxgDenseGridIsosurfaceExtractor::allocateGPUBuffers()
	{
		if (mData.verts)
			return;

		mData.verts = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		mData.normals = PX_DEVICE_MEMORY_ALLOC(PxVec4, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxVerts);
		mData.triIds = PX_DEVICE_MEMORY_ALLOC(PxU32, *mShared.mKernelLauncher.getCudaContextManager(), mData.maxTriIds);
	}

	void PxgDenseGridIsosurfaceExtractor::setResultBufferDevice(PxVec4* vertices, PxU32* triIndices, PxVec4* normals)
	{
		if (mShared.mOwnsOutputGPUBuffers)
			releaseGPUBuffers();
		mShared.mOwnsOutputGPUBuffers = false;
		mData.verts = vertices;
		mData.normals = normals;
		mData.triIds = triIndices;
		mShared.mVertices = NULL;
		mShared.mTriIndices = NULL;
		mShared.mNormals = NULL;
	}

	void PxgDenseGridIsosurfaceExtractor::setResultBufferHost(PxVec4* vertices, PxU32* triIndices, PxVec4* normals)
	{
		if (mShared.mOwnsOutputGPUBuffers)
			releaseGPUBuffers();
		mShared.mOwnsOutputGPUBuffers = true;
		mShared.mVertices = vertices;
		mShared.mTriIndices = triIndices;
		mShared.mNormals = normals;
		allocateGPUBuffers();
	}

				


	void PxgSparseGridIsosurfaceExtractor::clearDensity(CUstream stream)
	{
		sparseGridClearDensity(mShared.mKernelLauncher, mData.mGrid.mGridParams, mData.density(), 0.0f, mSparseGrid.getSubgridsInUseGpuPointer(), stream);
	}

	void PxgDenseGridIsosurfaceExtractor::paramsToMCData()
	{
		const PxReal marginFactor = 1.01f;
		mData.kernelSize = mShared.mIsosurfaceParams.particleCenterToIsosurfaceDistance + marginFactor * mData.getSpacing();
		mData.threshold = -marginFactor * mData.getSpacing();
	}

	void PxgDenseGridIsosurfaceExtractor::initialize(PxgKernelLauncher& cudaContextManager, const PxBounds3& worldBounds,
		PxReal cellSize, const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles)
	{
		release();

		mMaxParticles = maxNumParticles;
		mShared.mIsosurfaceParams = isosurfaceParams;
		mData.mGrid.mGridParams.gridSpacing = cellSize;
		mShared.mKernelLauncher = cudaContextManager;

		paramsToMCData();

		mData.restDensity = 1.0f;
		mData.mGrid.mGridParams.origin = worldBounds.minimum;

		PxVec3 dim = worldBounds.getDimensions();
		mData.mGrid.mGridParams.numCellsX = (int)PxFloor(dim.x / mData.mGrid.mGridParams.gridSpacing) + 1;
		mData.mGrid.mGridParams.numCellsY = (int)PxFloor(dim.y / mData.mGrid.mGridParams.gridSpacing) + 1;
		mData.mGrid.mGridParams.numCellsZ = (int)PxFloor(dim.z / mData.mGrid.mGridParams.gridSpacing) + 1;
		const PxU32 numCells = mData.maxNumCells();

		mShared.mScan.initialize(&mShared.mKernelLauncher, numCells);

		mData.buffer[0] = PX_DEVICE_MEMORY_ALLOC(PxReal, *cudaContextManager.getCudaContextManager(), numCells);
		mData.firstCellVert = PX_DEVICE_MEMORY_ALLOC(PxU32, *cudaContextManager.getCudaContextManager(), numCells);
		mData.buffer[1] = PX_DEVICE_MEMORY_ALLOC(PxReal, *cudaContextManager.getCudaContextManager(), numCells);

		mData.maxVerts = maxNumVertices;
		mData.maxTriIds = maxNumTriangles * 3;

		mData.numVerticesNumIndices = PX_DEVICE_MEMORY_ALLOC(PxU32, *cudaContextManager.getCudaContextManager(), 2);

		mShared.mNumVerticesNumIndices = PX_PINNED_MEMORY_ALLOC(PxU32, *cudaContextManager.getCudaContextManager(), 2);
		mShared.mNumVerticesNumIndices[0] = 0;
		mShared.mNumVerticesNumIndices[1] = 0;

		mData.smoothingBuffer = PX_DEVICE_MEMORY_ALLOC(PxVec4, *cudaContextManager.getCudaContextManager(), maxNumVertices);
	}

	void PxgDenseGridIsosurfaceExtractor::release()
	{
		if (!mData.firstCellVert)
			return;

		mShared.mScan.release();
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.buffer[0]);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.firstCellVert);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.buffer[1]);
		if (mShared.mOwnsOutputGPUBuffers)
		{
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.verts);
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.normals);
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.triIds);
		}
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.numVerticesNumIndices);

		PX_PINNED_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mShared.mNumVerticesNumIndices);

		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.smoothingBuffer);

		//PX_DELETE_THIS;
	}

	void PxgSparseGridIsosurfaceExtractor::extractIsosurface(PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases, PxU32 validPhaseMask,
		PxU32* activeIndices, PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3, PxReal anisotropyFactor)
	{
		if (!mShared.mEnabled)
			return;

		PX_CHECK_AND_RETURN(deviceParticlePos, "PxSparseGridIsosurfaceExtractor::extractIsosurface no valid deviceParticlePositions provided");

		mData.mGrid.mNumSubgridsInUse = mSparseGrid.getSubgridsInUseGpuPointer();
		mSparseGrid.updateSparseGrid(deviceParticlePos, numParticles, phases, stream, validPhaseMask);
		clearDensity(stream);
		mShared.extractIso<PxSparseIsosurfaceExtractionData>(mData, deviceParticlePos, numParticles, stream, phases, validPhaseMask, activeIndices, anisotropy1, anisotropy2, anisotropy3, anisotropyFactor);
	}

	void PxgSparseGridIsosurfaceExtractor::paramsToMCData()
	{
		const PxReal marginFactor = 1.01f;
		mData.kernelSize = mShared.mIsosurfaceParams.particleCenterToIsosurfaceDistance + marginFactor * mData.getSpacing();
		mData.threshold = -marginFactor * mData.getSpacing();
	}


	void PxgSparseGridIsosurfaceExtractor::initialize(PxgKernelLauncher& cudaContextManager, const PxSparseGridParams sparseGridParams,
		const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles)
	{
		release();

		mShared.mKernelLauncher = cudaContextManager;

		mData.restDensity = 1.0f;
		mData.mGrid.mGridParams.gridSpacing = sparseGridParams.gridSpacing;
		mData.mGrid.mGridParams.maxNumSubgrids = sparseGridParams.maxNumSubgrids;
		mData.mGrid.mGridParams.subgridSizeX = sparseGridParams.subgridSizeX;
		mData.mGrid.mGridParams.subgridSizeY = sparseGridParams.subgridSizeY;
		mData.mGrid.mGridParams.subgridSizeZ = sparseGridParams.subgridSizeZ;
			
		mShared.mIsosurfaceParams = isosurfaceParams;
		paramsToMCData();

		const PxU32 numCells = mData.maxNumCells();

		PxU32 minSubgridRes = PxMin(PxMin(sparseGridParams.subgridSizeX, sparseGridParams.subgridSizeY), sparseGridParams.subgridSizeZ);
		PxU32 neighborhoodSize = PxMin(minSubgridRes, PxU32(PxFloor((isosurfaceParams.particleCenterToIsosurfaceDistance + sparseGridParams.gridSpacing) / sparseGridParams.gridSpacing)) + 1);

		mSparseGrid.initialize(&mShared.mKernelLauncher, mData.mGrid.mGridParams, maxNumParticles, neighborhoodSize);
		mShared.mScan.initialize(&mShared.mKernelLauncher, numCells);

		mData.buffer[0] = PX_DEVICE_MEMORY_ALLOC(PxReal, *cudaContextManager.getCudaContextManager(), numCells);
		mData.firstCellVert = PX_DEVICE_MEMORY_ALLOC(PxU32, *cudaContextManager.getCudaContextManager(), numCells);
		mData.buffer[1] = PX_DEVICE_MEMORY_ALLOC(PxReal, *cudaContextManager.getCudaContextManager(), numCells);

		mData.maxVerts = maxNumVertices;
		mData.maxTriIds = maxNumTriangles * 3;

		mData.numVerticesNumIndices = PX_DEVICE_MEMORY_ALLOC(PxU32, *cudaContextManager.getCudaContextManager(), 2);

		mShared.mNumVerticesNumIndices = PX_PINNED_MEMORY_ALLOC(PxU32, *cudaContextManager.getCudaContextManager(), 2);
		mShared.mNumVerticesNumIndices[0] = 0;
		mShared.mNumVerticesNumIndices[1] = 0;

		//Make sparse grid data available for isosurface extraction
		mData.mGrid.mUniqueHashkeyPerSubgrid = mSparseGrid.getUniqueHashkeysPerSubgrid();
		mData.mGrid.mSubgridNeighbors = mSparseGrid.getSubgridNeighborLookup();

		mData.smoothingBuffer = PX_DEVICE_MEMORY_ALLOC(PxVec4, *cudaContextManager.getCudaContextManager(), maxNumVertices);
	}

	void PxgSparseGridIsosurfaceExtractor::release()
	{
		if (!mData.firstCellVert)
			return;

		mSparseGrid.release();
		mShared.mScan.release();
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.buffer[0]);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.firstCellVert);
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.buffer[1]);
		if (mShared.mOwnsOutputGPUBuffers)
		{
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.verts);
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.normals);
			PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.triIds);
		}
		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.numVerticesNumIndices);

		PX_PINNED_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mShared.mNumVerticesNumIndices);

		PX_DEVICE_MEMORY_FREE(*mShared.mKernelLauncher.getCudaContextManager(), mData.smoothingBuffer);

		//PX_DELETE_THIS;
	}

	void PxgDenseGridIsosurfaceExtractor::clearDensity(CUstream stream)
	{
		PxgCudaHelpers::memsetAsync(*mShared.mKernelLauncher.getCudaContextManager(), mData.density(), PxReal(0.f), mData.maxNumCells(), stream);
	}

	void PxgDenseGridIsosurfaceExtractor::extractIsosurface(PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases, PxU32 validPhaseMask,
		PxU32* activeIndices, PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3, PxReal anisotropyFactor)
	{
		if (!mShared.mEnabled)
			return;

		PX_CHECK_AND_RETURN(deviceParticlePos, "PxDenseGridIsosurfaceExtractor::extractIsosurface no valid deviceParticlePositions provided");
		clearDensity(stream);
		mShared.extractIso<PxIsosurfaceExtractionData>(mData, deviceParticlePos, numParticles, stream, phases, validPhaseMask, activeIndices, anisotropy1, anisotropy2, anisotropy3, anisotropyFactor);
	}

	template<typename DenseOrSparseGpuDataPackage>
	void PxgSharedIsosurfaceExtractor::meshFromDensity(DenseOrSparseGpuDataPackage& mData, CUstream stream)
	{
		if (!mData.firstCellVert)
			return;

		PxCudaContext* cudaContext = mKernelLauncher.getCudaContextManager()->getCudaContext();

		PxU32 passIndex = 0;
		PxIsosurfaceGridFilteringType::Enum operation;		
		bool useSDFStyleDensityTransfer = true;
		PxReal sign = useSDFStyleDensityTransfer ? -1.0f : 1.0f;

		PxReal neighborWeight = PxExp((-mData.getSpacing() * mData.getSpacing()) / (2.0f* mIsosurfaceParams.gridSmoothingRadius*mIsosurfaceParams.gridSmoothingRadius));

		while (passIndex < 32 && mIsosurfaceParams.getGridFilteringPass(passIndex, operation))
		{
			if (operation == PxIsosurfaceGridFilteringType::eSMOOTH)
				gridFilterGaussLaunch(mKernelLauncher, mData, neighborWeight, stream);
			else
				gridFilterDilateErodeLaunch(mKernelLauncher, mData, sign * (operation == PxIsosurfaceGridFilteringType::eGROW ? 1.0f : -1.0f), stream);
			++passIndex;
		}

		// create vertices
		countCellVertsLaunch(mKernelLauncher, mData, stream);
		mScan.exclusiveScan(mData.firstCellVert, stream);
		createVertsLaunch(mKernelLauncher, mData, stream);

		countTriIdsLaunch(mKernelLauncher, mData, stream);
		mScan.exclusiveScan(mData.firstCellTriId(), stream);

		createTriIdsLaunch(mKernelLauncher, mData, !useSDFStyleDensityTransfer, stream);

		// smooth verts
		//The normals get computed after the vertex smoothing. We can use the normal storage as a temporary storage.
		PxVec4* avgVerts = mData.normals;

		PxgCudaHelpers::memsetAsync(*mKernelLauncher.getCudaContextManager(), reinterpret_cast<PxReal*>(avgVerts), PxReal(0.f), mData.maxVerts * 4, stream);

		//Compute weights for Taubin smoothing
		PxReal a = -1.0f;
		PxReal b = 0.01f; //pass band range 0...1
		PxReal c = 2.0f;

		PxReal d = PxSqrt(b * b - 4 * a * c);
		PxReal solution1 = (-b + d) / (2.0f * a * c);
		PxReal solution2 = (-b - d) / (2.0f * a * c);

		PxReal lambdaMu[2];
		lambdaMu[0] = PxMax(solution1, solution2);
		lambdaMu[1] = PxMin(solution1, solution2);

		//The following parameter convert Taubin smoothing to classical Laplassian smoothing
		//lambdaMu[0] = 1.0f;
		//lambdaMu[1] = 1.0f;

		for (PxU32 i = 0; i < mIsosurfaceParams.numMeshSmoothingPasses; i++)
		{
			smoothVertsLaunch(mKernelLauncher, mData.verts, avgVerts, mData.triIds, &mData.numVerticesNumIndices[1], stream);
			averageVertsLaunch(mKernelLauncher, avgVerts, mData.verts, mData.numVerticesNumIndices, stream, lambdaMu[i % 2]);
		}

		// compute normals
		computeNormalsLaunch(mKernelLauncher, mData, stream);
		normalizeNormalsLaunch(mKernelLauncher, mData, stream);

		for (PxU32 i = 0; i < mIsosurfaceParams.numMeshNormalSmoothingPasses; i++)
		{
			smoothNormalsLaunch(mKernelLauncher, mData, stream);
			smoothNormalsNormalizeLaunch(mKernelLauncher, mData, stream);
		}			

		if (mVertices)
			cudaContext->memcpyDtoHAsync(mVertices, CUdeviceptr(mData.verts), mData.maxVerts * sizeof(PxVec4), stream);
		if (mTriIndices)
			cudaContext->memcpyDtoHAsync(mTriIndices, CUdeviceptr(mData.triIds), mData.maxTriIds * sizeof(PxU32), stream);
		if (mNormals)
			cudaContext->memcpyDtoHAsync(mNormals, CUdeviceptr(mData.normals), mData.maxVerts * sizeof(PxVec4), stream);

		cudaContext->memcpyDtoHAsync(mNumVerticesNumIndices, CUdeviceptr(mData.numVerticesNumIndices), 2 * sizeof(PxU32), stream);
	}

	template<typename DenseOrSparseGpuDataPackage>
	void PxgSharedIsosurfaceExtractor::extractIso(DenseOrSparseGpuDataPackage& mData, PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases, PxU32 validPhaseMask,
		PxU32* activeIndices, PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3, PxReal anisotropyFactor)
	{
		if (!mData.firstCellVert)
			return;
		
		computeParticleDensityLaunchUsingSDF(mKernelLauncher, deviceParticlePos, numParticles, phases, validPhaseMask, mData, stream,
			activeIndices, anisotropy1, anisotropy2, anisotropy3, anisotropyFactor);
			
		meshFromDensity(mData, stream);
	}

	template void PxgSharedIsosurfaceExtractor::meshFromDensity<PxIsosurfaceExtractionData>(PxIsosurfaceExtractionData& mData, CUstream stream);
	template void PxgSharedIsosurfaceExtractor::meshFromDensity<PxSparseIsosurfaceExtractionData>(PxSparseIsosurfaceExtractionData& mData, CUstream stream);


	template void PxgSharedIsosurfaceExtractor::extractIso<PxIsosurfaceExtractionData>(PxIsosurfaceExtractionData& mData, PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases, PxU32 validPhaseMask,
		PxU32* activeIndices, PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3, PxReal anisotropyFactor);
	template void PxgSharedIsosurfaceExtractor::extractIso<PxSparseIsosurfaceExtractionData>(PxSparseIsosurfaceExtractionData& mData, PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases, PxU32 validPhaseMask,
		PxU32* activeIndices, PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3, PxReal anisotropyFactor);

	class PxIsosurfaceExtractorCallback : public PxParticleSystemCallback
	{
	public:
		PxIsosurfaceExtractor* mIsosurfaceExtractor;

		void initialize(PxIsosurfaceExtractor* isosurfaceExtractor)
		{
			mIsosurfaceExtractor = isosurfaceExtractor;
		}

		virtual void onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
		{
			mIsosurfaceExtractor->extractIsosurface(reinterpret_cast<PxVec4*>(gpuParticleSystem.mHostPtr->mUnsortedPositions_InvMass), 
				gpuParticleSystem.mHostPtr->mCommonData.mNumParticles, stream, gpuParticleSystem.mHostPtr->mUnsortedPhaseArray, 
				PxParticlePhaseFlag::eParticlePhaseFluid, /*gpuParticleSystem.mHostPtr->mActiveArray*/NULL);
		}

		virtual void onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

		virtual void onAdvance(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }
	};		
}
