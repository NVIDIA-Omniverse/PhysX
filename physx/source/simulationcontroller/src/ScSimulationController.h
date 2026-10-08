// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_SIMULATION_CONTROLLER_H
#define	SC_SIMULATION_CONTROLLER_H

#include "PxsSimulationController.h"
#include "foundation/PxMutex.h"

namespace physx
{
namespace Sc
{
	class SimulationController : public PxsSimulationController
	{
		PX_NOCOPY(SimulationController)
	public:
						SimulationController(PxsSimulationControllerCallback* callback) : PxsSimulationController(callback, PxIntFalse)	{}
		virtual			~SimulationController()																							{}

		virtual void	updateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation)	PX_OVERRIDE;

		virtual void	updateArticulationAfterIntegration(PxsContext*	llContext, Bp::AABBManagerBase* aabbManager,
															PxArray<Sc::BodySim*>& ccdBodies, PxBaseTask* continuation, IG::IslandSim& islandSim, float dt, bool isSleepingDisabled)	PX_OVERRIDE;

		// PT: initial solution to sleepCheck() running non-thread-safe code in task. We should do better eventually.
		PxMutex	mArticulationSleepLock;
	};
}

}

#endif
