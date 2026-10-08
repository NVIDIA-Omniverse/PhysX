// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_GJKUTIL_H
#define GU_GJKUTIL_H

#include "foundation/PxVecMath.h"

/*
	This file is used to avoid the inner loop cross DLL calls
*/
namespace physx
{
namespace Gu
{

enum GjkStatus
{
	GJK_NON_INTERSECT,	// two shapes doesn't intersect
	GJK_CLOSE,			// two shapes doesn't intersect and gjk algorithm will return closest point information
	GJK_CONTACT,		// two shapes overlap within margin 
	GJK_UNDEFINED,		// undefined status
	GJK_DEGENERATE,		// gjk can't converge

	EPA_CONTACT,		// two shapes intersect
	EPA_DEGENERATE,		// epa can't converge
	EPA_FAIL			// epa fail to construct an initial polygon to work with 
};

struct GjkOutput
{
public:
	GjkOutput()
	{
		using namespace aos;
		closestA = closestB = normal = V3Zero();
		penDep = FZero();
	}
	aos::Vec3V closestA;
	aos::Vec3V closestB;
	aos::Vec3V normal;
	aos::Vec3V searchDir;
	aos::FloatV penDep;
};

}//Gu
}//physx

#endif
