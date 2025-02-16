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

#ifndef PX_RESIDUAL_H
#define PX_RESIDUAL_H

#include "PxPhysXConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief Structure representing residual values.

	This struct holds residual values, typically used in physics simulations to measure error or discrepancy.
	*/
	struct PxResidual
	{
		PxReal rmsResidual; //!< Root mean square residual value.
		PxReal maxResidual; //!< Maximum residual value.

		PxResidual() : rmsResidual(0.0f), maxResidual(0.0f) {}
	};

	/**
	\brief Structure representing residual values

	This struct holds residual values for both position and velocity iterations.
	*/
	struct PxResiduals
	{
		PxResidual positionIterationResidual; //!< Residual values for position iteration.
		PxResidual velocityIterationResidual; //!< Residual values for velocity iteration.
	};

	typedef PxResiduals PxArticulationResidual;
	typedef PxResiduals PxSceneResidual;

	/**
	\brief Structure representing residual values for a constraint.

	This struct holds residual values for both position and velocity iterations specific to a constraint.
	*/
	struct PxConstraintResidual
	{
		PxReal positionIterationResidual; //!< Residual value for position iteration.
		PxReal velocityIterationResidual; //!< Residual value for velocity iteration.

		PxConstraintResidual() : positionIterationResidual(0.0f), velocityIterationResidual(0.0f) {}
	};

#if !PX_DOXYGEN
}
#endif

#endif
