// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_CONVEX_GEOMETRY_EXT_H
#define PX_CONVEX_GEOMETRY_EXT_H

#include "foundation/PxMat33.h"
#include "foundation/PxTransform.h"
#include "geometry/PxConvexCoreGeometry.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxBounds3;
class PxRenderOutput;

/**
\brief Convex geometry helpers
*/
class PxConvexCoreExt
{
public:

	/**
	\brief Compute mass properties of the convex core geometry.
	\param convex The convex geometry.
	\param[out] density1Mass The mass of the geometry assuming unit density.
	\param[out] inertiaTensor The inertia tensor of the geometry.
	\param[out] centerOfMass The center of mass of the geometry.
	*/
	static void computeMassInfo(const PxConvexCoreGeometry& convex, PxReal& density1Mass, PxMat33& inertiaTensor, PxVec3& centerOfMass);

	/**
	\brief Visualize the convex core geometry
	\param convex The convex geometry.
	\param pose The pose of the geometry in world space
	\param drawCore If true, draw the core inside the full convex geometry including the margin
	\param cullbox The culling box for visualization
	\param out The render output object to use for visualization
	*/
	static void visualize(const PxConvexCoreGeometry& convex, const PxTransform& pose, bool drawCore, const PxBounds3& cullbox, PxRenderOutput& out);

};

#if !PX_DOXYGEN
}
#endif

#endif
