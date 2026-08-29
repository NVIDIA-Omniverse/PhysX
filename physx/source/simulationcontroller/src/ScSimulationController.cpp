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

#include "ScSimulationController.h"
#include "CmFlushPool.h"
#include "ScArticulationSim.h"
#include "PxsContext.h"
#include "BpAABBManager.h"
#include "common/PxProfileZone.h"
#include "ScShapeSimBase.h"

using namespace physx;
using namespace IG;
using namespace Sc;
using namespace Cm;

void SimulationController::updateScBodyAndShapeSim(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, PxBaseTask* continuation)
{
	mCallback->updateScBodyAndShapeSim(continuation);
}

namespace
{
class UpdateArticulationAfterIntegrationTask : public Cm::Task
{
	const UpdateCachedParams				mParams;
	PinnableBitMap*							mChangedAABBMgrActorHandles;
	IslandSim&								mIslandSim;
	const PxNodeIndex* const PX_RESTRICT	mNodeIndices;
	const PxU32								mNbArticulations;
	const PxReal							mDt;
	const bool								mIsSleepingDisabled;
	PxMutex&								mArticulationSleepLock;

	PX_NOCOPY(UpdateArticulationAfterIntegrationTask)
public:
	static const PxU32 NbArticulationsPerTask = 64;

	UpdateArticulationAfterIntegrationTask(const UpdateCachedParams& params, PxU32 nbArticulations, PxReal dt, const PxNodeIndex* nodeIndices, IslandSim& islandSim, PinnableBitMap* changedAABBMgrActorHandles, bool isSleepingDisabled, PxMutex& articulationSleepLock) :
		Cm::Task(islandSim.getContextId()),
		mParams(params),
		mChangedAABBMgrActorHandles(changedAABBMgrActorHandles),
		mIslandSim(islandSim),
		mNodeIndices(nodeIndices),
		mNbArticulations(nbArticulations),
		mDt(dt),
		mIsSleepingDisabled(isSleepingDisabled),
		mArticulationSleepLock(articulationSleepLock)
	{
	}

	virtual void runInternal() PX_OVERRIDE
	{
		const bool sleepingDisabled = mIsSleepingDisabled;
		const PxU32 nb = mNbArticulations;
		for(PxU32 i=0; i<nb; i++)
		{
			ArticulationSim* articSim = getArticulationSim(mIslandSim, mNodeIndices[i]);
			if(!sleepingDisabled)
				articSim->sleepCheck(mDt, mArticulationSleepLock);

			// PT: this is only executed in the CPU version so we can call the version that bypasses the virtual calls
			articSim->updateCached(mParams, mChangedAABBMgrActorHandles, true, true);
		}
	}

	virtual const char* getName() const PX_OVERRIDE { return "UpdateArticulationAfterIntegrationTask"; }
};
}

void updateCCDLinks(Sc::ArticulationSim& artic, PxArray<BodySim*>& sims);

// PT: warning, this runs in parallel with ScAfterIntegrationTask and updateKinematicCached, and all of these touching the getChangedAABBMgActorHandleMap() bitmap
void SimulationController::updateArticulationAfterIntegration(PxsContext* llContext, Bp::AABBManagerBase* aabbManager,
	PxArray<BodySim*>& ccdBodies, PxBaseTask* continuation, IslandSim& islandSim, float dt, bool isSleepingDisabled)
{
	const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(Node::eARTICULATION_TYPE);
	if(!nbActiveArticulations)
		return;

	Cm::FlushPool& flushPool = llContext->getTaskPool();
	UpdateCachedParams params(llContext->getTransformCache(), aabbManager->getBoundsArray());
	params.mTransformCache.setChangedState();
	params.mBoundsArray.setChangedState();

	const PxNodeIndex* activeArticulations = islandSim.getActiveNodes(Node::eARTICULATION_TYPE);

	PinnableBitMap& changedAABBMgrActorHandles = aabbManager->getChangedAABBMgActorHandleMap();
	for (PxU32 i = 0; i < nbActiveArticulations; i += UpdateArticulationAfterIntegrationTask::NbArticulationsPerTask)
	{
		UpdateArticulationAfterIntegrationTask* task =
			PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateArticulationAfterIntegrationTask)), UpdateArticulationAfterIntegrationTask)(params,
				PxMin(UpdateArticulationAfterIntegrationTask::NbArticulationsPerTask, PxU32(nbActiveArticulations - i)), dt,
				activeArticulations + i, islandSim, &changedAABBMgrActorHandles, isSleepingDisabled, mArticulationSleepLock);

		startTask(task, continuation);
	}

	if(llContext->getCCDFlag())
	{
		PX_PROFILE_ZONE("SimulationController::updateArticulationAfterIntegration_serial", llContext->getContextId());

		for (PxU32 i = 0; i < nbActiveArticulations; i++)
		{
			ArticulationSim* articSim = getArticulationSim(islandSim, activeArticulations[i]);

			//KS - check links for CCD flags and add to mCcdBodies list if required....
			updateCCDLinks(*articSim, ccdBodies);
		}
	}
}
