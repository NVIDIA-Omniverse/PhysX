// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PHYSX_MESHGENERATOR_H
#define PHYSX_MESHGENERATOR_H

#include "PxPhysicsAPI.h"
#include "extensions/PxRemeshingExt.h"

namespace meshgenerator
{
using namespace physx;

void createCube(PxArray<PxVec3>& triVerts, PxArray<PxU32>& triIndices, const PxVec3& pos, const PxVec3& scaling)
{
	triVerts.clear();
	triIndices.clear();
	triVerts.pushBack(scaling.multiply(PxVec3(0.5f, -0.5f, -0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(0.5f, -0.5f, 0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(-0.5f, -0.5f, 0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(-0.5f, -0.5f, -0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(0.5f, 0.5f, -0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(0.5f, 0.5f, 0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(-0.5f, 0.5f, 0.5f)) + pos);
	triVerts.pushBack(scaling.multiply(PxVec3(-0.5f, 0.5f, -0.5f)) + pos);

	triIndices.pushBack(1); triIndices.pushBack(2); triIndices.pushBack(3);
	triIndices.pushBack(7); triIndices.pushBack(6); triIndices.pushBack(5);
	triIndices.pushBack(4); triIndices.pushBack(5); triIndices.pushBack(1);
	triIndices.pushBack(5); triIndices.pushBack(6); triIndices.pushBack(2);

	triIndices.pushBack(2); triIndices.pushBack(6); triIndices.pushBack(7);
	triIndices.pushBack(0); triIndices.pushBack(3); triIndices.pushBack(7);
	triIndices.pushBack(0); triIndices.pushBack(1); triIndices.pushBack(3);
	triIndices.pushBack(4); triIndices.pushBack(7); triIndices.pushBack(5);

	triIndices.pushBack(0); triIndices.pushBack(4); triIndices.pushBack(1);
	triIndices.pushBack(1); triIndices.pushBack(5); triIndices.pushBack(2);
	triIndices.pushBack(3); triIndices.pushBack(2); triIndices.pushBack(7);
	triIndices.pushBack(4); triIndices.pushBack(0); triIndices.pushBack(7);
}

void createConeY(PxArray<PxVec3>& triVerts, PxArray<PxU32>& triIndices, const PxVec3& center, PxReal radius, PxReal height, PxU32 numPointsOnRing = 32)
{
	triVerts.clear();
	triIndices.clear();
	for (PxU32 i = 0; i < numPointsOnRing; ++i)
	{
		PxReal angle = i * 2.0f * 3.1415926535898f / numPointsOnRing;
		triVerts.pushBack(center + radius * PxVec3(PxSin(angle), 0, PxCos(angle)));
	}

	triVerts.pushBack(center);
	triVerts.pushBack(center + PxVec3(0, height, 0));
	for (PxU32 i = 0; i < numPointsOnRing; ++i)
	{
		triIndices.pushBack(numPointsOnRing);  triIndices.pushBack(i); triIndices.pushBack((i + 1) % numPointsOnRing);
		triIndices.pushBack(numPointsOnRing + 1); triIndices.pushBack((i + 1) % numPointsOnRing); triIndices.pushBack(i);
	}
}

void projectPointsOntoSphere(PxArray<PxVec3>& triVerts, const PxVec3& center, PxReal radius)
{
	for (PxU32 i = 0; i < triVerts.size(); ++i)
	{
		PxVec3 dir = triVerts[i] - center;
		dir.normalize();
		triVerts[i] = center + radius * dir;
	}
}

void createSphere(PxArray<PxVec3>& triVerts, PxArray<PxU32>& triIndices, const PxVec3& center, PxReal radius, const PxReal maxEdgeLength)
{
	triVerts.clear();
	triIndices.clear();
	createCube(triVerts, triIndices, center, PxVec3(radius));
	projectPointsOntoSphere(triVerts, center, radius);
	while (PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength, 1))
		projectPointsOntoSphere(triVerts, center, radius);
}
}

#endif
