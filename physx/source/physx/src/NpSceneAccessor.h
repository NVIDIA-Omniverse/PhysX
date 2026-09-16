// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_SCENE_ACCESSOR_H
#define NP_SCENE_ACCESSOR_H

#include "PxScene.h"

namespace physx
{
	class PxsSimulationController;

	class NpSceneAccessor : public PxScene
	{
		PX_NOCOPY(NpSceneAccessor)

	public:
											NpSceneAccessor()	{}
		virtual								~NpSceneAccessor()	{}

		virtual	PxsSimulationController*	getSimulationController()							= 0;
		virtual void						setActiveActors(PxActor** actors, PxU32 nbActors)	= 0;
		virtual PxActor**					getFrozenActors(PxU32& nbActorsOut)					= 0;
		virtual void						setFrozenActorFlag(const bool buildFrozenActors)	= 0;
		virtual void						forceSceneQueryRebuild()							= 0;
		virtual void						frameEnd()											= 0;
	};
}

#endif

