// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
#if PX_SUPPORT_GPU_PHYSX
	const bool								mIsGPU;
#endif
	PxMutex&								mArticulationSleepLock;
	volatile PxU32*							mCurrent;

	PX_NOCOPY(UpdateArticulationAfterIntegrationTask)
public:
	UpdateArticulationAfterIntegrationTask(
		const UpdateCachedParams& params, PxU32 nbArticulations, PxReal dt,
#if PX_SUPPORT_GPU_PHYSX
		bool isGPU,
#endif
		const PxNodeIndex* nodeIndices, IslandSim& islandSim, PinnableBitMap* changedAABBMgrActorHandles,
		bool isSleepingDisabled, PxMutex& articulationSleepLock, volatile PxU32* current
	) :
		Cm::Task(islandSim.getContextId()),
		mParams(params),
		mChangedAABBMgrActorHandles(changedAABBMgrActorHandles),
		mIslandSim(islandSim),
		mNodeIndices(nodeIndices),
		mNbArticulations(nbArticulations),
		mDt(dt),
		mIsSleepingDisabled(isSleepingDisabled),
#if PX_SUPPORT_GPU_PHYSX
		mIsGPU(isGPU),
#endif
		mArticulationSleepLock(articulationSleepLock),
		mCurrent(current)
	{
	}

	virtual void runInternal() PX_OVERRIDE
	{
		const bool sleepingDisabled = mIsSleepingDisabled;

		volatile PxU32* current = mCurrent;
		const PxU32 nbToGo = mNbArticulations;
#if PX_SUPPORT_GPU_PHYSX
		const bool isGPU = mIsGPU;
#endif
		while(1)
		{
			const PxU32 index = PxU32(PxAtomicIncrement(reinterpret_cast<volatile PxI32*>(current))) - 1;
			if(index>=nbToGo)
				return;

			ArticulationSim* articSim = getArticulationSim(mIslandSim, mNodeIndices[index]);
			if(!sleepingDisabled)
				articSim->sleepCheck(mDt, mArticulationSleepLock);
			
#if PX_SUPPORT_GPU_PHYSX
			if(!isGPU)
#endif
			{
				const Bp::AggregateHandle aggregateHandle = articSim->getAggregateHandle();

				// PT: we can skip the bitmap updates when the articulation is in an aggregate, as in this case the only
				// thing we do when parsing the updated bitmap is fetch the aggregate from the articulation link and mark
				// it as dirty. So instead we can simply mark the aggregate dirty directly (later in the sequential part).
				PinnableBitMap* shapeChangedMap = aggregateHandle != PX_INVALID_U32 ? NULL : mChangedAABBMgrActorHandles;

				// PT: this is only executed in the CPU version so we can call the version that bypasses the virtual calls
				articSim->updateCached(mParams, shapeChangedMap, true, true);
			}
#if PX_SUPPORT_GPU_PHYSX
			else
			{
				// PT: the virtual calls are only needed in the DirectGPU version (which doesn't reach this code) so we can still skip them here.
				articSim->updateCached(mParams, mChangedAABBMgrActorHandles, true, true);
			}
#endif
		}
	}

	virtual const char* getName() const PX_OVERRIDE { return "UpdateArticulationAfterIntegrationTask"; }
};
}

void updateCCDLinks(Sc::ArticulationSim& artic, PxArray<BodySim*>& sims);

// PT: warning, this runs in parallel with ScAfterIntegrationTask and updateKinematicCached, and all of these touch the getChangedAABBMgActorHandleMap() bitmap.
// ScAfterIntegrationTask also writes to the "ccdBodies" array we get passed here, so that one needs the context lock as well (see below).
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

#if PX_SUPPORT_GPU_PHYSX
	const bool isGPU = aabbManager->isGPUManager();
#endif

	const PxNodeIndex* activeArticulations = islandSim.getActiveNodes(Node::eARTICULATION_TYPE);

	PinnableBitMap& changedAABBMgrActorHandles = aabbManager->getChangedAABBMgActorHandleMap();

	PxU32* nb = reinterpret_cast<PxU32*>(flushPool.allocate(sizeof(PxU32)));
	*nb = 0;
	volatile PxU32* current = nb;

	PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
	numCpuTasks = PxMax(1u, PxMin(numCpuTasks, nbActiveArticulations));

	for(PxU32 i=0; i<numCpuTasks; i++)
	{
		UpdateArticulationAfterIntegrationTask* task =
			PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateArticulationAfterIntegrationTask)), UpdateArticulationAfterIntegrationTask)(params,
				nbActiveArticulations, dt,
#if PX_SUPPORT_GPU_PHYSX
				isGPU,
#endif
				activeArticulations, islandSim, &changedAABBMgrActorHandles, isSleepingDisabled, mArticulationSleepLock, current);

		startTask(task, continuation);
	}

#if PX_SUPPORT_GPU_PHYSX
	if(!isGPU)
#endif
	{
		PX_PROFILE_ZONE("SimulationController::updateArticulationAfterIntegration_serial_agg", llContext->getContextId());

		// PT: this is the second part of what we started inside UpdateArticulationAfterIntegrationTask. We mark aggregates
		// as dirty directly here, for next frame, instead of parsing the bitmap and re-discovering these aggregates during
		// the broadphase update of next frame.

		// PT: this part cannot run in tasks as it is not thread safe but it could potentially run in parallel with the CCD loop below.
		// We could also eventually make "markAggregateAsDirty" thread safe, but the potential gains seem limited, as the number of articulations
		// is usually low.

		// PT: this cast is ok because the function is only executed for CPU sim
		Bp::AABBManager& aabbMan = static_cast<Bp::AABBManager&>(*aabbManager);

		for (PxU32 i = 0; i < nbActiveArticulations; i++)
		{
			const ArticulationSim* articSim = getArticulationSim(islandSim, activeArticulations[i]);

			const Bp::AggregateHandle handle = articSim->getAggregateHandle();

			if(handle != PX_INVALID_U32)
				aabbMan.markAggregateAsDirty(handle);
		}
	}

	if(llContext->getCCDFlag())
	{
		PX_PROFILE_ZONE("SimulationController::updateArticulationAfterIntegration_serial_ccd", llContext->getContextId());

		// PT: this lock protects "ccdBodies": ScAfterIntegrationTask pushes
		// into that same array from worker threads under this very lock (see ScScene.cpp), and this code runs in parallel with it
		PxMutex::ScopedLock lock(llContext->getLock());

		for (PxU32 i = 0; i < nbActiveArticulations; i++)
		{
			ArticulationSim* articSim = getArticulationSim(islandSim, activeArticulations[i]);

			//KS - check links for CCD flags and add to mCcdBodies list if required....
			updateCCDLinks(*articSim, ccdBodies);
		}
	}
}
