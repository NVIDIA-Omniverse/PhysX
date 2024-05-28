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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "foundation/PxErrorCallback.h"
#include "ScParticleSystemShapeCore.h"
#include "ScParticleSystemShapeSim.h"
#include "ScPhysics.h"
#include "PxvGlobals.h"
#include "PxPhysXGpu.h"
#include "cudamanager/PxCudaContextManager.h"
#include "CmVisualization.h"

using namespace physx;
using namespace Sc;


ParticleSystemShapeCore::ParticleSystemShapeCore()
	: ShapeCore(PxEmpty)
	, mGpuMemStat(0)
{
	mSimulationFilterData = PxFilterData();
	mCore = PxsShapeCore();

	mCore.mShapeCoreFlags |= PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY;

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mCore.setTransform(PxTransform(PxIdentity));
	mCore.mContactOffset = 0.01f * scale.length;
	mCore.mShapeFlags = 0;
	mCore.mMaterialIndex = 0;

	mCore.mMinTorsionalPatchRadius = 0.f;
	mCore.mTorsionalRadius = 0.f;

	mLLCore.sleepThreshold = 5e-5f * scale.speed * scale.speed;
	mLLCore.wakeCounter = Physics::sWakeCounterOnCreation;
	mLLCore.freezeThreshold = 5e-6f * scale.speed * scale.speed;

	//TODO, make this dependend on scale?
	//also set contact offset accordingly
	mLLCore.restOffset = 0.1f;
	const PxReal contactOffset = mLLCore.restOffset + 0.001f;
	setContactOffset(contactOffset);

	mLLCore.particleContactOffset = contactOffset;
	mLLCore.solidRestOffset = mLLCore.restOffset;
	mLLCore.fluidRestOffset = mLLCore.restOffset * 0.6f;

	mLLCore.particleContactOffset_prev = FLT_MIN;
	mLLCore.fluidRestOffset_prev = FLT_MIN;

	mLLCore.fluidBoundaryDensityScale = 0.0f;

	mLLCore.gridSizeX = 128;
	mLLCore.gridSizeY = 128;
	mLLCore.gridSizeZ = 128;

	mLLCore.mFlags = PxParticleFlags(0);
	mLLCore.mLockFlags = PxParticleLockFlags(0);

	mLLCore.solverIterationCounts = (1 << 8) | 4;

	mLLCore.mWind = PxVec3(0.f);

	// Sparse grid specific
	mLLCore.sparseGridParams.setToDefault();
	mLLCore.sparseGridParams.gridSpacing = 2.0f * mLLCore.particleContactOffset;
}


// PX_SERIALIZATION
ParticleSystemShapeCore::ParticleSystemShapeCore(const PxEMPTY)
	: ShapeCore(PxEmpty)
{
}

ParticleSystemShapeCore::~ParticleSystemShapeCore()
{
}

void ParticleSystemShapeCore::initializeLLCoreData(PxU32 maxNeighborhood, PxReal neighborhoodScale)
{
	const PxTolerancesScale& scale = Sc::Physics::getInstance().getTolerancesScale();

	mLLCore.mMaxNeighborhood = maxNeighborhood;
	mLLCore.mNeighborhoodScale = neighborhoodScale;
	
	mLLCore.maxDepenetrationVelocity = 50.f * scale.length;
	mLLCore.maxVelocity = 1e+6f;
}

#endif // PX_SUPPORT_GPU_PHYSX

