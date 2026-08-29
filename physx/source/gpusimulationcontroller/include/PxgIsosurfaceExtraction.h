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

#ifndef PXG_ISOSURFACE_EXTRACTION_H
#define PXG_ISOSURFACE_EXTRACTION_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxArray.h"
#include "PxSparseGridParams.h"
#include "PxIsosurfaceExtraction.h"

#include "PxgSparseGridStandalone.h"
#include "PxgAlgorithms.h"
#include "PxgIsosurfaceData.h"
#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	
#if PX_SUPPORT_GPU_PHYSX
		
	class PxgSharedIsosurfaceExtractor
	{
	public:
		bool mEnabled;
		PxIsosurfaceParams mIsosurfaceParams;

		PxGpuScan mScan;

		PxgKernelLauncher mKernelLauncher;
		PxU32* mNumVerticesNumIndices;

		bool mOwnsOutputGPUBuffers;
		PxVec4* mVertices;
		PxVec4* mNormals;
		PxU32* mTriIndices;

		//public:
		PxgSharedIsosurfaceExtractor() :
			mEnabled(true), mKernelLauncher(), mNumVerticesNumIndices(NULL), mOwnsOutputGPUBuffers(false),
			mVertices(NULL), mNormals(NULL), mTriIndices(NULL)
		{}

		virtual ~PxgSharedIsosurfaceExtractor() {}

		template<typename DenseOrSparseGpuDataPackage>
		void extractIso(DenseOrSparseGpuDataPackage& mData, PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases, PxU32 validPhaseMask,
			PxU32* activeIndices = NULL, PxVec4* anisotropy1 = NULL, PxVec4* anisotropy2 = NULL, PxVec4* anisotropy3 = NULL, PxReal anisotropyFactor = 1.0f);

		template<typename DenseOrSparseGpuDataPackage>
		void meshFromDensity(DenseOrSparseGpuDataPackage& mData, CUstream stream);
	};

	/**
	\brief GPU based isosurface extractor operating on a sparse grid
	*/
	class PxgSparseGridIsosurfaceExtractor : public PxSparseGridIsosurfaceExtractor, public PxUserAllocated
	{
	protected:
		PxgSharedIsosurfaceExtractor mShared;
		PxSparseIsosurfaceExtractionData mData;
		PxSparseGridBuilder mSparseGrid;

		void paramsToMCData();

		virtual void setMaxVerticesAndTriangles(PxU32 maxIsosurfaceVertices, PxU32 maxIsosurfaceTriangles) PX_OVERRIDE;

		virtual void releaseGPUBuffers();

		virtual void allocateGPUBuffers();

	public:
		PxgSparseGridIsosurfaceExtractor() : mShared() {}

		virtual void setResultBufferDevice(PxVec4* vertices, PxU32* triIndices, PxVec4* normals) PX_OVERRIDE;

		virtual ~PxgSparseGridIsosurfaceExtractor() { }

		bool initialize(PxgKernelLauncher& kernelLauncher, const PxSparseGridParams sparseGridParams,
			const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles);

		virtual void release() PX_OVERRIDE;

		virtual void setIsosurfaceParams(const PxIsosurfaceParams& params) PX_OVERRIDE
		{
			mShared.mIsosurfaceParams = params;
			paramsToMCData();
		}

		virtual void clearDensity(CUstream stream);

		virtual PxU32 getMaxParticles() const PX_OVERRIDE
		{
			return mSparseGrid.getMaxParticles();
		}

		virtual PxU32 getMaxVertices() const PX_OVERRIDE
		{
			return mData.maxVerts;
		}

		virtual PxU32 getMaxTriangles() const PX_OVERRIDE
		{
			return mData.maxTriIds / 3;
		}

		virtual void setMaxParticles(PxU32 maxParticles) PX_OVERRIDE;

		virtual void extractIsosurface(PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases = NULL, PxU32 validPhaseMask = PxParticlePhaseFlag::eParticlePhaseFluid,
			PxU32* activeIndices = NULL, PxVec4* anisotropy1 = NULL, PxVec4* anisotropy2 = NULL, PxVec4* anisotropy3 = NULL, PxReal anisotropyFactor = 1.0f) PX_OVERRIDE;

		virtual void setResultBufferHost(PxVec4* vertices, PxU32* triIndices, PxVec4* normals) PX_OVERRIDE;

		virtual PxIsosurfaceParams getIsosurfaceParams() const PX_OVERRIDE
		{
			return mShared.mIsosurfaceParams;
		}

		virtual PxU32 getNumVertices() const PX_OVERRIDE
		{
			if (!mShared.mNumVerticesNumIndices)
				return 0;
			return mShared.mNumVerticesNumIndices[0];
		}

		virtual PxU32 getNumTriangles() const PX_OVERRIDE
		{
			if (!mShared.mNumVerticesNumIndices)
				return 0;
			return mShared.mNumVerticesNumIndices[1] / 3;
		}

		virtual void setEnabled(bool enabled) PX_OVERRIDE
		{
			mShared.mEnabled = enabled;
		}

		virtual bool isEnabled() const PX_OVERRIDE
		{
			return mShared.mEnabled;
		}

		virtual PxSparseGridParams getSparseGridParams() const PX_OVERRIDE
		{
			return mSparseGrid.getGridParameters();
		}

		virtual void setSparseGridParams(const PxSparseGridParams& params) PX_OVERRIDE
		{
			mSparseGrid.setGridParameters(params);
		}
	};


	/**
	\brief GPU based isosurface extractor operating on a dense grid
	*/
	class PxgDenseGridIsosurfaceExtractor : public PxIsosurfaceExtractor, public PxUserAllocated
	{
	protected:
		PxgSharedIsosurfaceExtractor mShared;
		PxIsosurfaceExtractionData mData;

		PxU32 mMaxParticles; //For compatibility with sparse grid isosurface extractor. There is no upper particle limit on the dense grid extractor.

		void paramsToMCData();

		virtual void setMaxVerticesAndTriangles(PxU32 maxIsosurfaceVertices, PxU32 maxIsosurfaceTriangles) PX_OVERRIDE;

		virtual void releaseGPUBuffers();

		virtual void allocateGPUBuffers();

	public:
		PxgDenseGridIsosurfaceExtractor() : mShared() {}

		virtual void setResultBufferDevice(PxVec4* vertices, PxU32* triIndices, PxVec4* normals) PX_OVERRIDE;

		virtual ~PxgDenseGridIsosurfaceExtractor() { }

		bool initialize(PxgKernelLauncher& kernelLauncher, const PxBounds3& worldBounds,
			PxReal cellSize, const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices, PxU32 maxNumTriangles);

		virtual void release() PX_OVERRIDE;

		virtual void setIsosurfaceParams(const PxIsosurfaceParams& params) PX_OVERRIDE
		{
			mShared.mIsosurfaceParams = params;
			paramsToMCData();
		}

		virtual void clearDensity(CUstream stream);

		virtual PxU32 getMaxParticles() const PX_OVERRIDE
		{
			return mMaxParticles;
		}

		virtual PxU32 getMaxVertices() const PX_OVERRIDE
		{
			return mData.maxVerts;
		}

		virtual PxU32 getMaxTriangles() const PX_OVERRIDE
		{
			return mData.maxTriIds / 3;
		}

		virtual void setMaxParticles(PxU32 maxParticles) PX_OVERRIDE
		{
			//No need to resize internal buffers on the dense grid isosurface;
			mMaxParticles = maxParticles;
		}

		virtual void extractIsosurface(PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* phases = NULL, PxU32 validPhaseMask = PxParticlePhaseFlag::eParticlePhaseFluid,
			PxU32* activeIndices = NULL, PxVec4* anisotropy1 = NULL, PxVec4* anisotropy2 = NULL, PxVec4* anisotropy3 = NULL, PxReal anisotropyFactor = 1.0f) PX_OVERRIDE;

		virtual void setResultBufferHost(PxVec4* vertices, PxU32* triIndices, PxVec4* normals) PX_OVERRIDE;

		virtual PxIsosurfaceParams getIsosurfaceParams() const PX_OVERRIDE
		{
			return mShared.mIsosurfaceParams;
		}

		virtual PxU32 getNumVertices() const PX_OVERRIDE
		{
			if (!mShared.mNumVerticesNumIndices)
				return 0;
			return mShared.mNumVerticesNumIndices[0];
		}

		virtual PxU32 getNumTriangles() const PX_OVERRIDE
		{
			if (!mShared.mNumVerticesNumIndices)
				return 0;
			return mShared.mNumVerticesNumIndices[1] / 3;
		}

		virtual void setEnabled(bool enabled) PX_OVERRIDE
		{
			mShared.mEnabled = enabled;
		}

		virtual bool isEnabled() const PX_OVERRIDE
		{
			return mShared.mEnabled;
		}
	};
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
