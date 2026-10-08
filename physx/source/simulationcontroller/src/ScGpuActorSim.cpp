// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScGpuActorSim.h"
#include "ScNPhaseCore.h"

#if PX_SUPPORT_GPU_PHYSX
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#endif

using namespace physx;
using namespace Sc;

Sc::GPUActorSim::GPUActorSim(Scene& scene, ActorCore& core, const ShapeCore* shapeCore) :
	ActorSim	(scene, core),
	mShapeSim	(*this, shapeCore)
{
}

Sc::GPUActorSim::~GPUActorSim()
{
	destroyLowLevelVolume();
}

void Sc::GPUActorSim::addToAABBMgr(Bp::FilterType::Enum type)
{
	const PxReal contactOffset = mShapeSim.getContactOffset();
	mShapeSim.addToAABBMgr(contactOffset, type);

	const PxU32 index = mShapeSim.getElementID();
	mScene.updateContactDistance(index, contactOffset);

	PxsTransformCache& cache = mScene.getLowLevelContext()->getTransformCache();
#if PX_SUPPORT_GPU_PHYSX
	if(cache.initEntry(index))
	{
		cache.setTransformCache(PxTransform(PxIdentity), 0, index);
	}
	else
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL,
								"Sc::GPUActorSim::addToAABBMgr: failed to allocate pinned memory transform cache");
		mScene.getCudaContextManager()->getCudaContext()->setAbortMode(true);
	}
#else
	cache.initEntry(index);
#endif
}

void Sc::GPUActorSim::destroyLowLevelVolume()
{
	if(mShapeSim.isInBroadPhase())
	{
		PxsContactManagerOutputIterator outputs = mScene.getLowLevelContext()->getNphaseImplementationContext()->getContactManagerOutputs();
		mScene.getNPhaseCore()->onVolumeRemoved(&mShapeSim, 0, outputs);
		mShapeSim.removeFromAABBMgr();
	}
	PX_ASSERT(!mShapeSim.isInBroadPhase());
}
