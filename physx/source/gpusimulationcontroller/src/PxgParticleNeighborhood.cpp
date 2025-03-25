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
