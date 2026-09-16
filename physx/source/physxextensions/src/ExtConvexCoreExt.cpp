// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "extensions/PxConvexCoreExt.h"
#include "GuConvexGeometry.h"

using namespace physx;

void PxConvexCoreExt::computeMassInfo(const PxConvexCoreGeometry& convex, PxReal& density1Mass, PxMat33& inertiaTensor, PxVec3& centerOfMass)
{
	Gu::computeMassInfo(convex, density1Mass, inertiaTensor, centerOfMass);
}

void PxConvexCoreExt::visualize(const PxConvexCoreGeometry& convex, const PxTransform& pose, bool drawCore, const PxBounds3& cullbox, PxRenderOutput& out)
{
	Gu::visualize(convex, pose, drawCore, cullbox, out);
}
