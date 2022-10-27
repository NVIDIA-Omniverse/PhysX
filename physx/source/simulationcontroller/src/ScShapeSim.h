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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_SHAPE_SIM_H
#define SC_SHAPE_SIM_H

#include "ScElementSim.h"
#include "ScShapeCore.h"
#include "ScRigidSim.h"
#include "PxsShapeSim.h"
#include "ScShapeSimBase.h"

namespace physx
{
	class PxsTransformCache;

/** Simulation object corresponding to a shape core object. This object is created when
    a ShapeCore object is added to the simulation, and destroyed when it is removed
*/

struct PxsRigidCore;

namespace Sc
{
	class RigidSim;
	class ShapeCore;
	class BodySim;

	class ShapeSim : public ShapeSimBase
	{
		ShapeSim &operator=(const ShapeSim &);
	public:

		// passing in a pointer for the shape to output its bounds allows us not to have to compute them twice.
		// A neater way to do this would be to ask the AABB Manager for the bounds after the shape has been 
		// constructed, but there is currently no spec for what the AABBMgr is allowed to do with the bounds, 
		// hence better not to assume anything.

										ShapeSim(RigidSim&, ShapeCore& core);
										~ShapeSim();
	private:
	};

#if !PX_P64_FAMILY
//	PX_COMPILE_TIME_ASSERT(32==sizeof(Sc::ShapeSim)); // after removing bounds from shapes
//	PX_COMPILE_TIME_ASSERT((sizeof(Sc::ShapeSim) % 16) == 0); // aligned mem bounds are better for prefetching
#endif

} // namespace Sc

}

#endif
