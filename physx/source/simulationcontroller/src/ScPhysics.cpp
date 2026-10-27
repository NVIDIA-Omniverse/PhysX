// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "common/PxTolerancesScale.h"

#include "ScPhysics.h"
#include "ScScene.h"
#include "PxvGlobals.h"

using namespace physx;

Sc::Physics* Sc::Physics::mInstance = NULL;
const PxReal Sc::Physics::sWakeCounterOnCreation = 20.0f*0.02f;

namespace physx
{
	namespace Sc
	{
		OffsetTable	gOffsetTable;
	}
}

Sc::Physics::Physics(const PxTolerancesScale& scale, const PxvOffsetTable& pxvOffsetTable) : mScale(scale)
{
	mInstance = this;
	PxvInit(pxvOffsetTable);
}

Sc::Physics::~Physics()
{
	PxvTerm();
	mInstance = NULL;
}
