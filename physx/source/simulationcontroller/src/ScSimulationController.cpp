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

using namespace physx;
using namespace IG;
using namespace Sc;

void SimulationController::updateScBodyAndShapeSim(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, PxBaseTask* continuation)
{
	mCallback->updateScBodyAndShapeSim(continuation);
}

namespace
{
class UpdateArticulationAfterIntegrationTask : public Cm::Task
{
	IslandSim&								mIslandSim;
	const PxNodeIndex* const PX_RESTRICT	mNodeIndices;
	const PxU32								mNbArticulations;
	const PxReal							mDt;
	const bool								mIsSleepingDisabled;

	PX_NOCOPY(UpdateArticulationAfterIntegrationTask)
public:
	static const PxU32 NbArticulationsPerTask = 64;

	UpdateArticulationAfterIntegrationTask(PxU64 contextId, PxU32 nbArticulations, PxReal dt, const PxNodeIndex* nodeIndices, IslandSim& islandSim, bool isSleepingDisabled) :
		Cm::Task(contextId),
		mIslandSim(islandSim),
		mNodeIndices(nodeIndices),
		mNbArticulations(nbArticulations),
		mDt(dt),
		mIsSleepingDisabled(isSleepingDisabled)
	{
	}

	virtual void runInternal()
	{
		const bool sleepingDisabled = mIsSleepingDisabled;
		const PxU32 nb = mNbArticulations;
		for(PxU32 i=0; i<nb; i++)
		{
			ArticulationSim* articSim = getArticulationSim(mIslandSim, mNodeIndices[i]);
			if(!sleepingDisabled)
				articSim->sleepCheck(mDt);
			articSim->updateCached_NotThreadSafe(NULL);
		}
	}

	virtual const char* getName() const { return "UpdateArticulationAfterIntegrationTask"; }
};
}

//KS - TODO - parallelize this bit!!!!!
void SimulationController::updateArticulationAfterIntegration(PxsContext*	llContext, Bp::AABBManagerBase* aabbManager,
	PxArray<BodySim*>& ccdBodies, PxBaseTask* continuation, IslandSim& islandSim, float dt, bool isSleepingDisabled)
{
	const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(Node::eARTICULATION_TYPE);

	Cm::FlushPool& flushPool = llContext->getTaskPool();

	const PxNodeIndex* activeArticulations = islandSim.getActiveNodes(Node::eARTICULATION_TYPE);

	for (PxU32 i = 0; i < nbActiveArticulations; i += UpdateArticulationAfterIntegrationTask::NbArticulationsPerTask)
	{
		UpdateArticulationAfterIntegrationTask* task =
			PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateArticulationAfterIntegrationTask)), UpdateArticulationAfterIntegrationTask)(islandSim.getContextId(),
				PxMin(UpdateArticulationAfterIntegrationTask::NbArticulationsPerTask, PxU32(nbActiveArticulations - i)), dt,
				activeArticulations + i, islandSim, isSleepingDisabled);

		startTask(task, continuation);
	}

	{
		PX_PROFILE_ZONE("SimulationController::updateArticulationAfterIntegration_serial", llContext->getContextId());

		PxMutex::ScopedLock lock(llContext->getLock());

		PxBitMapPinned& changedAABBMgrActorHandles = aabbManager->getChangedAABBMgActorHandleMap();

		for (PxU32 i = 0; i < nbActiveArticulations; i++)
		{
			ArticulationSim* articSim = getArticulationSim(islandSim, activeArticulations[i]);

			//KS - check links for CCD flags and add to mCcdBodies list if required....
			articSim->updateCCDLinks(ccdBodies);

			// PT: TODO: do this directly in ShapeSimBase::updateCached() with atomics, only serially process leftovers afterwards (the ones for which the map had to grow)
			articSim->markShapesUpdated(&changedAABBMgrActorHandles);
		}
	}
}
