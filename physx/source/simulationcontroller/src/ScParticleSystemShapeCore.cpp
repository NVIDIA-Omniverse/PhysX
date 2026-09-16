// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX
#include "foundation/PxErrorCallback.h"
#include "ScParticleSystemShapeCore.h"
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
	mShapeCoreFlags |= PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY;

	mGeometry.set(PxParticleSystemGeometry());
	reinterpret_cast<PxParticleSystemGeometryLL&>(mGeometry).materialsLL = MaterialIndicesStruct();

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	setTransform(PxTransform(PxIdentity));
	mContactOffset = 0.01f * scale.length;
	mShapeFlags = 0;
	mMaterialIndex = 0;

	mMinTorsionalPatchRadius = 0.f;
	mTorsionalRadius = 0.f;
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
	mLLCore.mMaxNeighborhood = maxNeighborhood;
	mLLCore.mNeighborhoodScale = neighborhoodScale;

	// Unbounded by default.
	mLLCore.maxDepenetrationVelocity = 1e32f;
	mLLCore.maxVelocity = 1e+6f;
}

#endif // PX_SUPPORT_GPU_PHYSX

