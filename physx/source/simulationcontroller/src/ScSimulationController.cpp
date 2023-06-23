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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScSimulationController.h"
#include "foundation/PxAllocator.h"
#include "CmTask.h"
#include "CmFlushPool.h"
#include "PxNodeIndex.h"
#include "ScArticulationSim.h"
#include "PxsContext.h"
#include "foundation/PxAllocator.h"
#include "BpAABBManager.h"
#include "DyVArticulation.h"

using namespace physx;

void Sc::SimulationController::updateScBodyAndShapeSim(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, PxBaseTask* continuation)
{
	mCallback->updateScBodyAndShapeSim(continuation);
}

namespace
{
class UpdateArticulationAfterIntegrationTask : public Cm::Task
{
	IG::IslandSim&							mIslandSim;
	const PxNodeIndex* const PX_RESTRICT	mNodeIndices;
	const PxU32								mNbArticulations;
	const PxReal							mDt;

	PX_NOCOPY(UpdateArticulationAfterIntegrationTask)
public:
	static const PxU32 NbArticulationsPerTask = 64;

	UpdateArticulationAfterIntegrationTask(PxU64 contextId, PxU32 nbArticulations, PxReal dt, const PxNodeIndex* nodeIndices, IG::IslandSim& islandSim) :
		Cm::Task(contextId),
		mIslandSim(islandSim),
		mNodeIndices(nodeIndices),
		mNbArticulations(nbArticulations),
		mDt(dt)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 i = 0; i < mNbArticulations; ++i)
		{
			PxNodeIndex nodeIndex = mNodeIndices[i];
			//Sc::ArticulationSim* articSim = getArticulationSim(mIslandSim, nodeIndex);
			Sc::ArticulationSim* articSim = mIslandSim.getArticulationSim(nodeIndex);
			articSim->sleepCheck(mDt);
			articSim->updateCached(NULL);
		}
	}

	virtual const char* getName() const { return "UpdateArticulationAfterIntegrationTask"; }
};
}

//KS - TODO - parallelize this bit!!!!!
void Sc::SimulationController::updateArticulationAfterIntegration(
	PxsContext*	llContext,
	Bp::AABBManagerBase* aabbManager,
	PxArray<Sc::BodySim*>& ccdBodies,
	PxBaseTask* continuation,
	IG::IslandSim& islandSim,
	float dt
	)
{
	const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);

	Cm::FlushPool& flushPool = llContext->getTaskPool();

	const PxNodeIndex* activeArticulations = islandSim.getActiveNodes(IG::Node::eARTICULATION_TYPE);

	for (PxU32 i = 0; i < nbActiveArticulations; i += UpdateArticulationAfterIntegrationTask::NbArticulationsPerTask)
	{
		UpdateArticulationAfterIntegrationTask* task =
			PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateArticulationAfterIntegrationTask)), UpdateArticulationAfterIntegrationTask)(islandSim.getContextId(), PxMin(UpdateArticulationAfterIntegrationTask::NbArticulationsPerTask, PxU32(nbActiveArticulations - i)), dt,
				activeArticulations + i, islandSim);

		startTask(task, continuation);
	}

	llContext->getLock().lock();

	//const IG::NodeIndex* activeArticulations = islandSim.getActiveNodes(IG::Node::eARTICULATION_TYPE);

	PxBitMapPinned& changedAABBMgrActorHandles = aabbManager->getChangedAABBMgActorHandleMap();

	for (PxU32 i = 0; i < nbActiveArticulations; i++)
	{
		Sc::ArticulationSim* articSim = islandSim.getArticulationSim(activeArticulations[i]);

		//KS - check links for CCD flags and add to mCcdBodies list if required....
		articSim->updateCCDLinks(ccdBodies);

		articSim->markShapesUpdated(&changedAABBMgrActorHandles);
	}
	llContext->getLock().unlock();
}
