// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScShapeSim.h"

using namespace physx;
using namespace Sc;

void resetElementID(Scene& scene, ShapeSimBase& shapeSim);

ShapeSim::ShapeSim(ActorSim& owner, ShapeCore& core) : ShapeSimBase(owner, &core)
{
	PX_ASSERT(core.getGeometryType() != PxGeometryType::eINVALID);
	const PxU32 index = getElementID();
	initSubsystemsDependingOnElementID(index);
	core.setExclusiveSim(this);
}

ShapeSim::~ShapeSim()
{
	mShapeCore->setExclusiveSim(NULL);
	Scene& scScene = getScene();
	resetElementID(scScene, *this);
}

