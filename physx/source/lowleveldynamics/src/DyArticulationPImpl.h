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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_ARTICULATION_INTERFACE_H
#define DY_ARTICULATION_INTERFACE_H

#include "DyArticulationUtils.h"
#include "DyFeatherstoneArticulation.h"

namespace physx
{
namespace Dy
{
	struct ArticulationSolverDesc;

class ArticulationPImpl
{
public:

	static PxU32 computeUnconstrainedVelocities(const ArticulationSolverDesc& desc,
											PxReal dt,
											PxU32& acCount,
											const PxVec3& gravity, 
											Cm::SpatialVectorF* Z, 
											Cm::SpatialVectorF* deltaV,
											const PxReal invLengthScale)
	{
		return FeatherstoneArticulation::computeUnconstrainedVelocities(desc, dt,  acCount, gravity, Z, deltaV, invLengthScale);
	}

	static void	updateBodies(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV,
						 PxReal dt)
	{
		FeatherstoneArticulation::updateBodies(desc, tempDeltaV, dt);
	}

	static void	updateBodiesTGS(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV,
		PxReal dt)
	{
		FeatherstoneArticulation::updateBodiesTGS(desc, tempDeltaV, dt);
	}

	static void	saveVelocity(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* tempDeltaV)
	{
		FeatherstoneArticulation::saveVelocity(articulation, tempDeltaV);
	}

	static void	saveVelocityTGS(FeatherstoneArticulation* articulation, PxReal invDtF32)
	{
		FeatherstoneArticulation::saveVelocityTGS(articulation, invDtF32);
	}

	static void computeUnconstrainedVelocitiesTGS(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV,
		const PxReal invLengthScale)
	{
		FeatherstoneArticulation::computeUnconstrainedVelocitiesTGS(desc, dt, gravity, contextID, Z, DeltaV, invLengthScale);
	}

	static void	updateDeltaMotion(const ArticulationSolverDesc& desc, const PxReal dt, Cm::SpatialVectorF* DeltaV, const PxReal totalInvDt)
	{
		FeatherstoneArticulation::recordDeltaMotion(desc, dt, DeltaV, totalInvDt);
	}

	static void	deltaMotionToMotionVel(const ArticulationSolverDesc& desc, const PxReal invDt)
	{
		FeatherstoneArticulation::deltaMotionToMotionVelocity(desc, invDt);
	}

	static PxU32 setupSolverInternalConstraintsTGS(const ArticulationSolverDesc& desc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		const PxReal biasCoefficient,
		PxU32& acCount,
		Cm::SpatialVectorF* Z)
	{
		return FeatherstoneArticulation::setupSolverConstraintsTGS(desc,  dt, invDt, totalDt, biasCoefficient, acCount,  Z);
	}
};


}
}
#endif

