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

#ifndef DY_SOLVER_CONSTRAINT_EXT_SHARED_H
#define DY_SOLVER_CONSTRAINT_EXT_SHARED_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxVecMath.h"
#include "DyArticulationContactPrep.h"
#include "DySolverConstraintDesc.h"
#include "DySolverConstraint1D.h"
#include "DySolverContact.h"
#include "DySolverContactPF.h"
#include "PxcNpWorkUnit.h"
#include "PxsMaterialManager.h"

namespace physx
{
	namespace Dy
	{
		FloatV setupExtSolverContact(const SolverExtBody& b0, const SolverExtBody& b1,
			const FloatV& d0, const FloatV& d1, const FloatV& angD0, const FloatV& angD1, const Vec3V& bodyFrame0p, const Vec3V& bodyFrame1p,
			const Vec3VArg normal, const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg dt, const FloatVArg restDistance, const FloatVArg maxPenBias, const FloatVArg restitution,
			const FloatVArg bounceThreshold, const PxContactPoint& contact, SolverContactPointExt& solverContact, const FloatVArg ccdMaxSeparation,
			Cm::SpatialVectorF* Z, const Cm::SpatialVectorV& v0, const Cm::SpatialVectorV& v1, const FloatV& cfm, const Vec3VArg solverOffsetSlop,
			const FloatVArg norVel0, const FloatVArg norVel1, const FloatVArg damping);
	} //namespace Dy
}

#endif
