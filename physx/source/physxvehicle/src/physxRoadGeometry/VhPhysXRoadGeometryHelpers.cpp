// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "vehicle/PxVehicleParams.h"

#include "vehicle/physxRoadGeometry/PxVehiclePhysXRoadGeometryHelpers.h"

#include "cooking/PxConvexMeshDesc.h"
#include "cooking/PxCooking.h"
#include "extensions/PxDefaultStreams.h"
#include "PxPhysics.h"

namespace physx
{

PxConvexMesh* PxVehicleUnitCylinderSweepMeshCreate
(const PxVehicleFrame& runtimeFrame, PxPhysics& physics, const PxCookingParams& params)
{
	const PxMat33 mat33 = runtimeFrame.getFrame();
	const PxQuat frame(mat33);
	const PxReal radius = 1.0f;
	const PxReal halfWidth = 1.0f;

	#define NB_CIRCUMFERENCE_POINTS 64
	PxVec3 points[2 * NB_CIRCUMFERENCE_POINTS];
	for (PxU32 i = 0; i < NB_CIRCUMFERENCE_POINTS; i++)
	{
		const PxF32 cosTheta = PxCos(i * PxPi * 2.0f / float(NB_CIRCUMFERENCE_POINTS));
		const PxF32 sinTheta = PxSin(i * PxPi * 2.0f / float(NB_CIRCUMFERENCE_POINTS));
		const PxF32 x = radius * cosTheta;
		const PxF32 z = radius * sinTheta;
		points[2 * i + 0] = frame.rotate(PxVec3(x, -halfWidth, z));
		points[2 * i + 1] = frame.rotate(PxVec3(x, +halfWidth, z));
	}

	// Create descriptor for convex mesh
	PxConvexMeshDesc convexDesc;
	convexDesc.points.count = sizeof(points)/sizeof(PxVec3);
	convexDesc.points.stride = sizeof(PxVec3);
	convexDesc.points.data = points;
	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxConvexMesh* convexMesh = NULL;
	PxDefaultMemoryOutputStream buf;
	if (PxCookConvexMesh(params, convexDesc, buf))
	{
		PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
		convexMesh = physics.createConvexMesh(id);
	}

	return convexMesh;
}

void PxVehicleUnitCylinderSweepMeshDestroy(PxConvexMesh* mesh)
{
	mesh->release();
}

} //namespace physx
