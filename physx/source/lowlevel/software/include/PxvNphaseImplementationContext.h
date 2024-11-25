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

#ifndef PXV_NPHASE_IMPLEMENTATION_CONTEXT_H
#define PXV_NPHASE_IMPLEMENTATION_CONTEXT_H

#include "PxSceneDesc.h"
#include "PxsContactManagerState.h"
#include "foundation/PxArray.h"
#include "PxsNphaseCommon.h"

// PT: TODO: forward decl don't work easily with templates, to revisit
#include "PxsMaterialCore.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "PxsPBDMaterialCore.h"

namespace physx
{
namespace IG
{
	class IslandSim;
	typedef PxU32 EdgeIndex;
}

namespace Cm
{
	class FanoutTask;
}

namespace Sc
{
	class ShapeInteraction;
}

class PxNodeIndex;
class PxBaseTask;
class PxsContext;
struct PxsShapeCore;

class PxsContactManager;
struct PxsContactManagerOutput;
struct PxsTorsionalFrictionData;

class PxsContactManagerOutputIterator
{
	PxU32 mOffsets[1<<PxsContactManagerBase::MaxBucketBits];
	PxsContactManagerOutput* mOutputs;

public:

	PxsContactManagerOutputIterator() : mOutputs(NULL)
	{
	}

	PxsContactManagerOutputIterator(const PxU32* offsets, PxU32 nbOffsets, PxsContactManagerOutput* outputs) : mOutputs(outputs)
	{
		PX_ASSERT(nbOffsets <= (1<<PxsContactManagerBase::MaxBucketBits));

		for(PxU32 a = 0; a < nbOffsets; ++a)
		{
			mOffsets[a] = offsets[a];
		}
	}

	PX_FORCE_INLINE PxsContactManagerOutput& getContactManagerOutput(PxU32 id)
	{
		PX_ASSERT((id & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK) == 0);
		PxU32 bucketId = PxsContactManagerBase::computeBucketIndexFromId(id);
		PxU32 cmOutId = PxsContactManagerBase::computeIndexFromId(id);
		return mOutputs[mOffsets[bucketId] + cmOutId];
	}

	PxU32 getIndex(PxU32 id)	const
	{
		PX_ASSERT((id & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK) == 0);
		PxU32 bucketId = PxsContactManagerBase::computeBucketIndexFromId(id);
		PxU32 cmOutId = PxsContactManagerBase::computeIndexFromId(id);
		return mOffsets[bucketId] + cmOutId;
	}
};

class PxvNphaseImplementationContext
{
										PX_NOCOPY(PxvNphaseImplementationContext)

			PxsContext&					mContext;
public:
										PxvNphaseImplementationContext(PxsContext& context): mContext(context) {}
	virtual								~PxvNphaseImplementationContext() {}

	virtual void						destroy() = 0;
	virtual void						updateContactManager(PxReal dt, bool hasContactDistanceChanged, PxBaseTask* continuation, PxBaseTask* firstPassContinuation, Cm::FanoutTask* updateBoundAndShapeTask) = 0;
	virtual void						postBroadPhaseUpdateContactManager(PxBaseTask* continuation) = 0;
	virtual void						secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation) = 0;
	virtual void						fetchUpdateContactManager() = 0;
	
	virtual void						registerContactManager(PxsContactManager* cm, const Sc::ShapeInteraction* interaction, PxI32 touching, PxU32 patchCount) = 0;
//	virtual void						registerContactManagers(PxsContactManager** cm, Sc::ShapeInteraction** shapeInteractions, PxU32 nbContactManagers, PxU32 maxContactManagerId) = 0;
	virtual void						unregisterContactManager(PxsContactManager* cm) = 0;
	virtual void						refreshContactManager(PxsContactManager* cm) = 0;

	virtual void						registerShape(const PxNodeIndex& nodeIndex, const PxsShapeCore& shapeCore, const PxU32 transformCacheID, PxActor* actor, const bool isDeformableSurface = false) = 0;
	virtual void						unregisterShape(const PxsShapeCore& shapeCore, const PxU32 transformCacheID, const bool isDeformableSurface = false) = 0;

	virtual void						registerAggregate(const PxU32 transformCacheID) = 0;

	virtual void						registerMaterial(const PxsMaterialCore& materialCore) = 0;
	virtual void						updateMaterial(const PxsMaterialCore& materialCore) = 0;
	virtual void						unregisterMaterial(const PxsMaterialCore& materialCore) = 0;

	virtual void						registerMaterial(const PxsDeformableSurfaceMaterialCore& materialCore) = 0;
	virtual void						updateMaterial(const PxsDeformableSurfaceMaterialCore& materialCore) = 0;
	virtual void						unregisterMaterial(const PxsDeformableSurfaceMaterialCore& materialCore) = 0;

	virtual void						registerMaterial(const PxsDeformableVolumeMaterialCore& materialCore) = 0;
	virtual void						updateMaterial(const PxsDeformableVolumeMaterialCore& materialCore) = 0;
	virtual void						unregisterMaterial(const PxsDeformableVolumeMaterialCore& materialCore) = 0;

	virtual void						registerMaterial(const PxsPBDMaterialCore& materialCore) = 0;
	virtual void						updateMaterial(const PxsPBDMaterialCore& materialCore) = 0;
	virtual void						unregisterMaterial(const PxsPBDMaterialCore& materialCore) = 0;

	virtual void						updateShapeMaterial(const PxsShapeCore& shapeCore) = 0;
	
	virtual void						startNarrowPhaseTasks() = 0;

	virtual void						appendContactManagers() = 0;	

	virtual PxsContactManagerOutput&	getNewContactManagerOutput(PxU32 index) = 0;

	virtual PxsContactManagerOutputIterator	getContactManagerOutputs() = 0;

	virtual void						setContactModifyCallback(PxContactModifyCallback* callback) = 0;

	virtual void						acquireContext() = 0;
	virtual void						releaseContext() = 0;
	virtual void						preallocateNewBuffers(PxU32 nbNewPairs, PxU32 maxIndex) = 0;

	virtual	void						lock() = 0;
	virtual void						unlock() = 0;

	virtual PxsContactManagerOutputCounts*	getLostFoundPatchOutputCounts() = 0;
	virtual PxsContactManager**			getLostFoundPatchManagers() = 0;
	virtual PxU32						getNbLostFoundPatchManagers() = 0;		

	//GPU-specific buffers. Return null for CPU narrow phase

	virtual PxsContactManagerOutput*	getGPUContactManagerOutputBase() = 0;
	virtual PxReal*						getGPURestDistances() = 0;
	virtual Sc::ShapeInteraction**		getGPUShapeInteractions() = 0;
	virtual PxsTorsionalFrictionData*	getGPUTorsionalData() = 0;
};

class PxvNphaseImplementationFallback: public PxvNphaseImplementationContext
{
												PX_NOCOPY(PxvNphaseImplementationFallback)
public:
												PxvNphaseImplementationFallback(PxsContext& context) : PxvNphaseImplementationContext(context)	{}
	virtual										~PxvNphaseImplementationFallback() {}

	virtual void								unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs) = 0;

	virtual void								processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation) = 0;
	virtual void								processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation) = 0;

	virtual void								refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs) = 0;
	virtual void								appendContactManagersFallback(PxsContactManagerOutput* outputs) = 0;
	virtual void								removeContactManagersFallback(PxsContactManagerOutput* cmOutputs) = 0;

	virtual const Sc::ShapeInteraction*const*	getShapeInteractionsGPU()	const	= 0;
	virtual const PxReal*						getRestDistancesGPU()		const	= 0;
	virtual const PxsTorsionalFrictionData*		getTorsionalDataGPU()		const	= 0;
};

PxvNphaseImplementationFallback* createNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, PxVirtualAllocatorCallback* allocator, bool gpuDynamics);

}

#endif
