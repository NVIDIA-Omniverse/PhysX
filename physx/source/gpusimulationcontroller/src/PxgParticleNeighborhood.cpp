// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgParticleNeighborhoodProvider.h"
#include "PxgAlgorithms.h"
#include "PxgSparseGridStandalone.h"

#include "PxgAnisotropyData.h"

#include "PxPhysics.h"
#include "PxParticleSystem.h"
#include "foundation/PxUserAllocated.h"

#include "PxParticleGpu.h"
#include "foundation/PxHashSet.h"

#include "PxParticleGpu.h"
#include "PxgParticleNeighborhoodProvider.h"

#include "PxPhysXGpu.h"
#include "PxvGlobals.h"
#include "PxgKernelIndices.h"

using namespace physx;

PxgParticleNeighborhoodProvider::PxgParticleNeighborhoodProvider(PxgKernelLauncher& cudaContextManager, const PxU32 maxNumParticles, const PxReal particleContactOffset, const PxU32 maxNumSparseGridCells)
{
	mKernelLauncher = cudaContextManager;

	PxSparseGridParams p;
	p.maxNumSubgrids = maxNumSparseGridCells;
	p.gridSpacing = 2.0f * particleContactOffset;
	p.subgridSizeX = 1;
	p.subgridSizeY = 1;
	p.subgridSizeZ = 1;
	mSparseGridBuilder.initialize(&mKernelLauncher, p, maxNumParticles, 0, true);
}

void PxgParticleNeighborhoodProvider::setCellProperties(PxU32 maxGridCells, PxReal cellSize)
{
	PxU32 maxNumParticles = mSparseGridBuilder.getMaxParticles();

	mSparseGridBuilder.release();

	PxSparseGridParams p;
	p.maxNumSubgrids = maxGridCells;
	p.gridSpacing = cellSize;
	p.subgridSizeX = 1;
	p.subgridSizeY = 1;
	p.subgridSizeZ = 1;
	mSparseGridBuilder.initialize(&mKernelLauncher, p, maxNumParticles, 0, true);
}
