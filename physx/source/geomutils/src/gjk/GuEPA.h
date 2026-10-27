// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_EPA_H
#define GU_EPA_H

#include "GuGJKUtil.h"
#include "GuGJKType.h"

namespace physx
{
  
namespace Gu
{
	//ML: The main entry point for EPA.
	// 
	//This function returns one of three status codes:
	//(1)EPA_FAIL:	the algorithm failed to create a valid polytope(the origin wasn't inside the polytope) from the input simplex.
	//(2)EPA_CONTACT : the algorithm found the MTD and converged successfully.
	//(3)EPA_DEGENERATE: the algorithm cannot make further progress and the result is unknown.

	GjkStatus epaPenetration(	const GjkConvex& a,						//convex a in the space of convex b
								const GjkConvex& b, 					//convex b							   
								const PxU8* PX_RESTRICT aInd,			//warm start index for convex a to create an initial simplex
								const PxU8* PX_RESTRICT bInd,			//warm start index for convex b to create an initial simplex
								const PxU8 size,						//number of warm-start indices						    
								const bool takeCoreShape,				//indicates whether we take support point from the core shape or surface of capsule/sphere
								const aos::FloatV tolerenceLength,		//the length of meter
								GjkOutput& output);						//result					

	GjkStatus epaPenetration(	const GjkConvex& a,						//convex a in the space of convex b
								const GjkConvex& b, 					//convex b							   
								const aos::Vec3V* PX_RESTRICT aPnt,		//warm start point for convex a to create an initial simplex
								const aos::Vec3V* PX_RESTRICT bPnt,		//warm start point for convex b to create an initial simplex
								const PxU8 size,						//number of warm-start indices						    
								const bool takeCoreShape,				//indicates whether we take support point from the core shape or surface of capsule/sphere
								const aos::FloatV tolerenceLength,		//the length of meter
								GjkOutput& output);						//result					
}

}

#endif
