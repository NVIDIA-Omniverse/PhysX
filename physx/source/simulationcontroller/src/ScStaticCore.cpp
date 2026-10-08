// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "ScStaticCore.h"
#include "ScStaticSim.h"
#include "PxRigidStatic.h"

using namespace physx;

Sc::StaticSim* Sc::StaticCore::getSim() const
{
	return static_cast<StaticSim*>(Sc::ActorCore::getSim());
}

void Sc::StaticCore::setActor2World(const PxTransform& actor2World)
{
	mCore.body2World = actor2World;

	StaticSim* sim = getSim();
	if(sim)
		sim->notifyShapesOfTransformChange();
}
