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

namespace physx
{
namespace Dy
{
	struct ArticulationSolverDesc;

class ArticulationPImpl
{
public:

	typedef PxU32 (*ComputeUnconstrainedVelocitiesFn)(const ArticulationSolverDesc& desc,
													 PxReal dt,
													 PxU32& acCount,
													 const PxVec3& gravity, 
													 Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV, 
													 const PxReal invLengthScale);

	typedef void (*UpdateBodiesFn)(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* deltaV, PxReal dt);

	typedef void (*SaveVelocityFn)(const ArticulationSolverDesc &m, Cm::SpatialVectorF* deltaV);

	typedef void(*SaveVelocityTGSFn)(const ArticulationSolverDesc& m, PxReal invDtF32);

	typedef PxU32(*SetupInternalConstraintsTGSFn)(const ArticulationSolverDesc& desc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		const PxReal biasCoefficient,
		PxU32& acCount,
		Cm::SpatialVectorF* Z);

	typedef void(*ComputeUnconstrainedVelocitiesTGSFn)(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV,
		const PxReal invLengthScale);

	typedef void(*UpdateDeltaMotionFn)(const ArticulationSolverDesc &m, const PxReal dt, Cm::SpatialVectorF* DeltaV, const PxReal totalInvDt);

	typedef void(*DeltaMotionToMotionVelFn)(const ArticulationSolverDesc &m, const PxReal dt);

	static ComputeUnconstrainedVelocitiesFn sComputeUnconstrainedVelocities;
	static UpdateBodiesFn sUpdateBodies;
	static UpdateBodiesFn sUpdateBodiesTGS;
	static SaveVelocityFn sSaveVelocity;
	static SaveVelocityTGSFn sSaveVelocityTGS;

	static UpdateDeltaMotionFn sUpdateDeltaMotion;
	static DeltaMotionToMotionVelFn sDeltaMotionToMotionVel;
	static ComputeUnconstrainedVelocitiesTGSFn sComputeUnconstrainedVelocitiesTGS;
	static SetupInternalConstraintsTGSFn sSetupInternalConstraintsTGS;

	static PxU32 computeUnconstrainedVelocities(const ArticulationSolverDesc& desc,
											PxReal dt,
											PxU32& acCount,
											const PxVec3& gravity, 
											Cm::SpatialVectorF* Z, 
											Cm::SpatialVectorF* deltaV,
											const PxReal invLengthScale)
	{
		PX_ASSERT(sComputeUnconstrainedVelocities);
		if (sComputeUnconstrainedVelocities)
			return (sComputeUnconstrainedVelocities)(desc, dt,  acCount,
				gravity, Z, deltaV, invLengthScale);
		else
			return 0;
	}

	static void	updateBodies(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV,
						 PxReal dt)
	{
		PX_ASSERT(sUpdateBodies);
		if (sUpdateBodies)
			(*sUpdateBodies)(desc, tempDeltaV, dt);
	}

	static void	updateBodiesTGS(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV,
		PxReal dt)
	{
		PX_ASSERT(sUpdateBodiesTGS);
		if (sUpdateBodiesTGS)
			(*sUpdateBodiesTGS)(desc, tempDeltaV, dt);
	}

	static void	saveVelocity(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* tempDeltaV)
	{
		PX_ASSERT(sSaveVelocity);
		if (sSaveVelocity)
			(*sSaveVelocity)(desc, tempDeltaV);
	}


	static void	saveVelocityTGS(const ArticulationSolverDesc& desc, PxReal invDtF32)
	{
		PX_UNUSED(desc);
		PX_UNUSED(invDtF32);
		PX_ASSERT(sSaveVelocityTGS);
		if (sSaveVelocityTGS)
			(*sSaveVelocityTGS)(desc, invDtF32);
	}

	static void computeUnconstrainedVelocitiesTGS(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV,
		const PxReal invLengthScale)
	{
		PX_ASSERT(sComputeUnconstrainedVelocitiesTGS);
		if (sComputeUnconstrainedVelocitiesTGS)
			(sComputeUnconstrainedVelocitiesTGS)(desc, dt, gravity, contextID, Z, DeltaV, invLengthScale);
	}

	static void	updateDeltaMotion(const ArticulationSolverDesc& desc, const PxReal dt, Cm::SpatialVectorF* DeltaV, const PxReal totalInvDt)
	{
		PX_ASSERT(sUpdateDeltaMotion);
		if (sUpdateDeltaMotion)
			(*sUpdateDeltaMotion)(desc, dt, DeltaV, totalInvDt);
	}

	static void	deltaMotionToMotionVel(const ArticulationSolverDesc& desc, const PxReal invDt)
	{
		PX_ASSERT(sDeltaMotionToMotionVel);
		if (sDeltaMotionToMotionVel)
			(*sDeltaMotionToMotionVel)(desc, invDt);
	}

	static PxU32 setupSolverInternalConstraintsTGS(const ArticulationSolverDesc& desc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		const PxReal biasCoefficient,
		PxU32& acCount,
		Cm::SpatialVectorF* Z)
	{
		PX_ASSERT(sSetupInternalConstraintsTGS);
		if (sSetupInternalConstraintsTGS)
			return sSetupInternalConstraintsTGS(desc,  dt, invDt, 
				totalDt, biasCoefficient, acCount,  Z);
		return 0;

	}
};


}
}
#endif

