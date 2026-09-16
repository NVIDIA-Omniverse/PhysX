// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONTACTPOLYGONPOLYGON_H
#define GU_CONTACTPOLYGONPOLYGON_H


#include "foundation/PxMat33.h"
#include "foundation/PxMat34.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
class PxContactBuffer;
class PxPlane;

namespace Cm
{
	class FastVertex2ShapeScaling;
}

namespace Gu
{

PX_PHYSX_COMMON_API PxMat33 findRotationMatrixFromZ(const PxVec3& to);

PX_PHYSX_COMMON_API bool contactPolygonPolygonExt(	PxU32 numVerts0, const PxVec3* vertices0, const PxU8* indices0,//polygon 0
													const PxMat34& world0, const PxPlane& localPlane0,			//xform of polygon 0, plane of polygon
													const PxMat33& RotT0,

													PxU32 numVerts1, const PxVec3* vertices1, const PxU8* indices1,//polygon 1
													const PxMat34& world1, const PxPlane& localPlane1,			//xform of polygon 1, plane of polygon
													const PxMat33& RotT1,

													const PxVec3& worldSepAxis,									//world normal of separating plane - this is the world space normal of polygon0!!
													const PxMat34& transform0to1, const PxMat34& transform1to0,//transforms between polygons
													PxU32 polyIndex0, PxU32 polyIndex1,	//face indices for contact callback,
													PxContactBuffer& contactBuffer,
													bool flipNormal, const PxVec3& posShift, float sepShift
													);	// shape order, post gen shift.
}
}

#endif
