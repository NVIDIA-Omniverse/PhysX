// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
			PxU32 validPhase = PxParticlePhaseFlag::eParticlePhaseFluid, const PxU32* deviceActiveIndices = NULL) PX_OVERRIDE
		{
			mSparseGridBuilder.updateSparseGrid(deviceParticlePos, numParticles, devicePhases, stream, validPhase, deviceActiveIndices);
			mSparseGridBuilder.updateSubgridEndIndices(numParticles, stream);
		}

		PxU32* getSubgridEndIndicesBuffer()
		{
			return mSparseGridBuilder.getSubgridEndIndicesBuffer();
		}

		virtual PxU32 getMaxParticles() const PX_OVERRIDE
		{
			return mSparseGridBuilder.getMaxParticles();
		}

		virtual void setMaxParticles(PxU32 maxParticles) PX_OVERRIDE
		{
			mSparseGridBuilder.setMaxParticles(maxParticles);
		}

		virtual void release() PX_OVERRIDE
		{
			mSparseGridBuilder.release();
			PX_DELETE_THIS;
		}

		virtual PxU32 getNumGridCellsInUse() const PX_OVERRIDE
		{
			return mSparseGridBuilder.getNumSubgridsInUse();
		}

		virtual PxU32 getMaxGridCells() const PX_OVERRIDE
		{
			return mSparseGridBuilder.getGridParameters().maxNumSubgrids;
		}

		virtual PxReal getCellSize() const PX_OVERRIDE
		{
			return mSparseGridBuilder.getGridParameters().gridSpacing;
		}

		virtual void setCellProperties(PxU32 maxGridCells, PxReal cellSize) PX_OVERRIDE;

		virtual ~PxgParticleNeighborhoodProvider() {}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
