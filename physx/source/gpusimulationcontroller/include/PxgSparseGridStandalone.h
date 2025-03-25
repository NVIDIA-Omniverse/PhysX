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

#ifndef PXG_SPARSE_GRID_STANDALONE_H
#define PXG_SPARSE_GRID_STANDALONE_H

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#include "PxgAlgorithms.h"
#include "PxSparseGridParams.h"
#include "PxParticleSystemFlag.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief Constructs a sparse grid structure given an array of particle positions
	*/
	class PxSparseGridBuilder
	{
	protected:
		PxgKernelLauncher* mKernelLauncher;

		PxGpuScan mScan;
		PxGpuRadixSort<PxU32> mSort;
		PxSparseGridParams mSparseGridParams;
		PxU32* mHashkeyPerParticle;
		PxU32* mSortedParticleToSubgrid;
		PxU32* mSortedUniqueHashkeysPerSubgrid;
		PxU32* mSubgridNeighborLookup;

		//Optional
		PxU32* mSortedToOriginalParticleIndex;

		PxGpuScan mScanNeighbors;

		PxGpuRadixSort<PxU32> mNeighborSort;
		PxU32* mNeighborCollector;
		PxU32* mRequiredNeighborMask;

		PxU32 mMaxParticles;

		PxU32 mNeighborhoodSize;			//!< The minimum number of cells available around a particle that are guaranteed to be accessible. It can be computed as PxCeil(particleRadius/sparseGridSpacing)
		bool mTrackParticleOrder;

		bool mCopySubgridsInUseToHost;
		PxU32 mNumSubgridsInUse;

		PxU32* updateSubgrids(PxVec4* deviceParticlePos, const PxU32 numParticles, PxU32* devicePhases, CUstream stream,
			PxU32 validPhase = PxParticlePhaseFlag::eParticlePhaseFluid, const PxU32* activeIndices = NULL);

		void updateSubgridNeighbors(PxU32* totalCountPointer, CUstream stream);

	public:
		PxSparseGridBuilder() : mHashkeyPerParticle(NULL), mSortedToOriginalParticleIndex(NULL), mCopySubgridsInUseToHost(true), mNumSubgridsInUse(0) { }

		~PxSparseGridBuilder()
		{
			release();
		}

		/**
		\brief The number of subgrids that are in use. Subgrids in use contain at least one particle or have particles closely nearby

		\return The number of subgrids in use
		*/
		PX_FORCE_INLINE PxU32 getNumSubgridsInUse() const
		{
			return mNumSubgridsInUse;
		}

		/**
		\brief Sets the state of the flag that defines if the subgrid in use information gets copied to the host every frame

		\param[in] enabled Enable or disable copying the number of subgrids in use to the host every frame
		*/
		PX_FORCE_INLINE void setCopySubgridsInUseToHostEnabled(bool enabled)
		{
			mCopySubgridsInUseToHost = enabled;
		}

		/**
		\brief Gets the state of the flag that defines if the subgrid in use information gets copied to the host every frame

		\return True if enabled
		*/
		PX_FORCE_INLINE bool getCopySubgridsInUseToHostEnabled() const
		{
			return mCopySubgridsInUseToHost;
		}

		/**
		\brief Gets the sparse grid parameters

		\return The sparse grid parameters
		*/
		PX_FORCE_INLINE PxSparseGridParams getGridParameters() const
		{
			return mSparseGridParams;
		}

		/**
		\brief Sets the sparse grid parameters

		\param[in] params The new sparse grid parameters
		*/
		PX_FORCE_INLINE void setGridParameters(PxSparseGridParams params)
		{
			release();
			initialize(mKernelLauncher, params, mMaxParticles, mNeighborhoodSize, mTrackParticleOrder);
		}

		/**
		\brief Gets the device pointer to the number of subgrids in use

		\return The device pointer to the number of subgrids in use
		*/
		PX_FORCE_INLINE PxU32* getSubgridsInUseGpuPointer()
		{
			if (mNeighborhoodSize == 0)
				return mScan.getSumPointer();
			else
				return mScanNeighbors.getSumPointer();
		}

		/**
		\brief Gets the device array containing the subgrid hashkey for every subgrid in use

		\return The device array containing the subgrid hashkey for every subgrid in use. Be aware that this data gets overwritten by the subgridEndIndicesBuffer data after a call to updateSubgridEndIndices
		*/
		PX_FORCE_INLINE PxU32* getUniqueHashkeysPerSubgrid()
		{
			return mSortedUniqueHashkeysPerSubgrid;
		}

		/**
		\brief Gets the subgrid neighbor lookup table

		\return The device pointer to the subgrid neighbor lookup table. Contains 27 elements for every subgrid in use and provides indices to the neighbors in the 3x3x3 neighborhood
		*/
		PX_FORCE_INLINE PxU32* getSubgridNeighborLookup()
		{
			return mSubgridNeighborLookup;
		}

		/**
		\brief Gets the sorted particle to subgrid index device buffer.

		\return The sorted particle to subgrid index device buffer
		*/
		PX_FORCE_INLINE PxU32* getSortedParticleToSubgrid()
		{
			return mSortedParticleToSubgrid;
		}

		/**
		\brief Gets the sorted to original particle index device buffer.

		\return The sorted to original particle index device buffer
		*/
		PX_FORCE_INLINE PxU32* getSortedToOriginalParticleIndex()
		{
			return mSortedToOriginalParticleIndex;
		}

		/**
		\brief Gets the subgrid end indices buffer. This allows to traverse the (sorted) particles in a subgrid because the buffer holds an inclusive cumulative sum of the active subgrid particle counts.

		\return The subgrid end indices device buffer
		*/
		PX_FORCE_INLINE PxU32* getSubgridEndIndicesBuffer()
		{
			return mSortedUniqueHashkeysPerSubgrid;
		}

		/**
		\brief Gets the maximal number of particles that can be processed

		\return The maximal number of particles
		*/
		PX_FORCE_INLINE PxU32 getMaxParticles() const
		{
			return mMaxParticles;
		}

		/**
		\brief Sets the maximal number of particles that can be processed

		\param[in] maxParticles The maximal number of particles
		*/
		PX_FORCE_INLINE void setMaxParticles(PxU32 maxParticles)
		{
			release();
			initialize(mKernelLauncher, mSparseGridParams, maxParticles, mNeighborhoodSize, mTrackParticleOrder);
		}

		/**
		\brief Initializes the sparse grid builder

		\param[in] cudaContextManager A cuda context manager
		\param[in] sparseGridParams The sparse grid parameters
		\param[in] maxNumParticles The number of particles
		\param[in] neighborhoodSize The size of the neighborhood around a particle that must be accessible.
		\param[in] trackParticleOrder If set to true, the mSortedToOriginalParticleIndex array will be updated during every call of updateSparseGrid
		*/
		virtual void initialize(PxgKernelLauncher* cudaContextManager, const PxSparseGridParams& sparseGridParams,
			PxU32 maxNumParticles, PxU32 neighborhoodSize, bool trackParticleOrder = false);

		virtual void release();

		//Completely rebuilds the sparse grid. Does not support to access data from previous timesteps. Supports a particle radius (neighborhoodSize)

		/**
		\brief Updates the sparse grid given an array of particle positions

		\param[in] deviceParticlePos The particle positions device array
		\param[in] numParticles The number of particles
		\param[in] devicePhases The particle's phases device array (optional, can be null)
		\param[in] stream The stream on which the work gets scheduled
		\param[in] validPhase The valid phase mask marking the phase bits that particles must have set in order to contribute to the isosurface
		\param[in] activeIndices Optional device array of active particle indices
		*/
		virtual void updateSparseGrid(PxVec4* deviceParticlePos, PxU32 numParticles, PxU32* devicePhases, CUstream stream,
			PxU32 validPhase = PxParticlePhaseFlag::eParticlePhaseFluid, const PxU32* activeIndices = NULL);

		/**
		\brief Updates the subgrid end indices. This is only required if the particles per subgrid need to get processed (e. g. anisotropy generation)

		\param[in] numParticles The number of particles
		\param[in] stream The stream on which the work gets scheduled
		*/
		void updateSubgridEndIndices(PxU32 numParticles, CUstream stream);
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
