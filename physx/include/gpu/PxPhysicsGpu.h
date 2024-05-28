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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_PHYSICS_GPU_H
#define PX_PHYSICS_GPU_H


#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxParticleSystem.h"

#include "foundation/PxArray.h"
#include "PxParticleGpu.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX		

	class PxSceneDesc;

	class PxIsosurfaceExtractor;
	class PxSparseGridIsosurfaceExtractor;
	class PxAnisotropyGenerator;
	class PxSmoothedPositionGenerator;
	class PxParticleNeighborhoodProvider;

	class PxArrayConverter;
	class PxLineStripSkinning;
	class PxSoftBodyEmbedding;
	class PxSDFBuilder;

	struct PxIsosurfaceParams;
	struct PxSparseGridParams;
	
	class PxPhysicsGpu
	{
	public:
		/**
		\brief Creates an isosurface extractor operating on a dense grid

		\param[in] cudaContextManager A cuda context manager
		\param[in] worldBounds The bounds of the internally used dense grid. The isosurface can only be generated inside those bounds.
		\param[in] cellSize The size of a single grid cell
		\param[in] isosurfaceParams The isosurface parameters to control the isolevel etc.
		\param[in] maxNumParticles The maximal number of particles that can be processed
		\param[in] maxNumVertices The maximal number of vertices the output buffer can hold
		\param[in] maxNumTriangles The maximal number of triangles the output buffer can hold
		*/
		virtual PxIsosurfaceExtractor* createDenseGridIsosurfaceExtractor(PxCudaContextManager* cudaContextManager, const PxBounds3& worldBounds,
			PxReal cellSize, const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices = 512 * 1024, PxU32 maxNumTriangles = 1024 * 1024) = 0;

		/**
		\brief Creates an isosurface extractor operating on a sparse grid

		\param[in] cudaContextManager A cuda context manager
		\param[in] sparseGridParams The sparse grid parameters defining the cell size etc.
		\param[in] isosurfaceParams The isosurface parameters to control the isolevel etc.
		\param[in] maxNumParticles The maximal number of particles that can be processed
		\param[in] maxNumVertices The maximal number of vertices the output buffer can hold
		\param[in] maxNumTriangles The maximal number of triangles the output buffer can hold
		*/
		virtual PxSparseGridIsosurfaceExtractor* createSparseGridIsosurfaceExtractor(PxCudaContextManager* cudaContextManager, const PxSparseGridParams& sparseGridParams,
			const PxIsosurfaceParams& isosurfaceParams, PxU32 maxNumParticles, PxU32 maxNumVertices = 512 * 1024, PxU32 maxNumTriangles = 1024 * 1024) = 0;


		/**
		\brief Creates an anisotropy generator

		\param[in] cudaContextManager A cuda context manager
		\param[in] maxNumParticles The number of particles
		\param[in] anisotropyScale A uniform scaling factor to increase or decrease anisotropy
		\param[in] minAnisotropy The minimum scaling factor in any dimension that anisotropy can have
		\param[in] maxAnisotropy The maximum scaling factor in any dimension that anisotropy can have
		*/
		virtual PxAnisotropyGenerator* createAnisotropyGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles,
			PxReal anisotropyScale = 1.0f, PxReal minAnisotropy = 0.1f, PxReal maxAnisotropy = 2.0f) = 0;

		/**
		\brief Creates a smoothed position generator

		\param[in] cudaContextManager A cuda context manager
		\param[in] maxNumParticles The number of particles
		\param[in] smoothingStrength Controls the strength of the smoothing effect
		*/
		virtual PxSmoothedPositionGenerator* createSmoothedPositionGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles, PxReal smoothingStrength = 0.5f) = 0;


		/**
		\brief Creates a neighborhood provider

		\param[in] cudaContextManager A cuda context manager
		\param[in] maxNumParticles The number of particles
		\param[in] cellSize The grid cell size. Should be equal to 2*contactOffset for PBD particle systems.
		\param[in] maxNumSparseGridCells The maximal number of cells the internally used sparse grid can provide
		*/
		virtual PxParticleNeighborhoodProvider* createParticleNeighborhoodProvider(PxCudaContextManager* cudaContextManager, const PxU32 maxNumParticles,
			const PxReal cellSize, const PxU32 maxNumSparseGridCells = 262144) = 0;

		/**
		\brief Creates an array converter. If not used anymore, the caller needs to delete the returned pointer.

		\param[in] cudaContextManager A cuda context manager
		*/
		virtual PxArrayConverter* createArrayConverter(PxCudaContextManager* cudaContextManager) = 0;

		/**
		\brief Creates an line strip embedding helper. If not used anymore, the caller needs to delete the returned pointer.

		\param[in] cudaContextManager A cuda context manager

		\return Pointer to a new instance of a PxLineStripSkinning
		*/
		virtual PxLineStripSkinning* createLineStripSkinning(PxCudaContextManager* cudaContextManager) = 0;

		/**
		\brief Creates sdf builder to construct sdfs quickly on the GPU. If not used anymore, the caller needs to delete the returned pointer.

		\param[in] cudaContextManager A cuda context manager

		\return Pointer to a new instance of a PxSDFBuilder
		*/
		virtual PxSDFBuilder* createSDFBuilder(PxCudaContextManager* cudaContextManager) = 0;

		/**
		\brief Estimates the amount of GPU memory needed to create a scene for the given descriptor.

		\deprecated This function is deprecated, creating a PxPhyics::createScene will return a null pointer if scene creation fails
		due to low GPU memory availability.

		\param[in] sceneDesc a valid scene desriptor

		\note While this is a conservative estimate, scene allocation may still fail even though there is
		enough memory - this function does not contain the potential overhead coming from the CUDA allocator.
		Additionally, there may be fragmentation issues. Generally, this is not an issue for 
		scene allocation sizes < 500Mb, but may become problematic for larger scenes.

		\return A conservative estimate for the amount of GPU memory needed to create a scene for the given descriptor.
	 	*/
		PX_DEPRECATED virtual PxU64 estimateSceneCreationGpuMemoryRequirements(const PxSceneDesc& sceneDesc) = 0;

		virtual void release() = 0;

		virtual ~PxPhysicsGpu() {}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
