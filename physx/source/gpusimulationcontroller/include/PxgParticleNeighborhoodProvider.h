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

#ifndef PXG_PARTICLE_NEIGHBORHOOD_PROVIDER_H
#define PXG_PARTICLE_NEIGHBORHOOD_PROVIDER_H

#include "PxParticleNeighborhoodProvider.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#include "PxgSparseGridStandalone.h"
#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	class PxgParticleNeighborhoodProvider : public PxParticleNeighborhoodProvider, public PxUserAllocated
	{
	private:
		PxgKernelLauncher mKernelLauncher;

	public:
		PxSparseGridBuilder mSparseGridBuilder;


		PxgParticleNeighborhoodProvider(PxgKernelLauncher& cudaContextManager, const PxU32 maxNumParticles, const PxReal particleContactOffset, const PxU32 maxNumSparseGridCells);

		virtual void buildNeighborhood(PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* devicePhases = NULL,
			PxU32 validPhase = PxParticlePhaseFlag::eParticlePhaseFluid, const PxU32* deviceActiveIndices = NULL)
		{
			mSparseGridBuilder.updateSparseGrid(deviceParticlePos, numParticles, devicePhases, stream, validPhase, deviceActiveIndices);
			mSparseGridBuilder.updateSubgridEndIndices(numParticles, stream);
		}

		PxU32* getSubgridEndIndicesBuffer()
		{
			return mSparseGridBuilder.getSubgridEndIndicesBuffer();
		}

		virtual PxU32 getMaxParticles() const
		{
			return mSparseGridBuilder.getMaxParticles();
		}

		virtual void setMaxParticles(PxU32 maxParticles)
		{
			mSparseGridBuilder.setMaxParticles(maxParticles);
		}

		virtual void release()
		{
			mSparseGridBuilder.release();
			PX_DELETE_THIS;
		}

		virtual PxU32 getNumGridCellsInUse() const
		{
			return mSparseGridBuilder.getNumSubgridsInUse();
		}

		virtual PxU32 getMaxGridCells() const
		{
			return mSparseGridBuilder.getGridParameters().maxNumSubgrids;
		}

		virtual PxReal getCellSize() const
		{
			return mSparseGridBuilder.getGridParameters().gridSpacing;
		}

		virtual void setCellProperties(PxU32 maxGridCells, PxReal cellSize);

		virtual ~PxgParticleNeighborhoodProvider() {}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
