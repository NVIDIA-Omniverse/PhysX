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
