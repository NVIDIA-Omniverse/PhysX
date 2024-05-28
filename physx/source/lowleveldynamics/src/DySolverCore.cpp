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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxPreprocessor.h"
#include "DySolverCore.h"
#include "DyArticulationPImpl.h"
#include "DyCpuGpuArticulation.h"

using namespace physx;
using namespace aos;
using namespace Dy;

// PT: TODO: ideally we could reuse this in immediate mode as well, but the code currently uses separate arrays of PxVec3s instead of
// spatial vectors so the SIMD code won't work there. Switching to spatial vectors requires a change in the immediate mode API (PxSolveConstraints).
void Dy::saveMotionVelocities(PxU32 nbBodies, const PxSolverBody* PX_RESTRICT solverBodies, Cm::SpatialVector* PX_RESTRICT motionVelocityArray)
{
	for(PxU32 i=0; i<nbBodies; i++)
	{
		Cm::SpatialVector& motionVel = motionVelocityArray[i];
		const PxSolverBody& body = solverBodies[i];

		V4StoreA(V4LoadA(&body.linearVelocity.x), &motionVel.linear.x);
		V4StoreA(V4LoadA(&body.angularState.x), &motionVel.angular.x);

		PX_ASSERT(motionVel.linear.isFinite());
		PX_ASSERT(motionVel.angular.isFinite());
	}
}

// PT: this case is reached when e.g. a lot of objects falling but not touching yet. So there are no contacts but potentially a lot of bodies.
// See LegacyBenchmark/falling_spheres for example.
void Dy::solveNoContactsCase(	PxU32 nbBodies, const PxSolverBody* PX_RESTRICT solverBodies, Cm::SpatialVector* PX_RESTRICT motionVelocityArray,
								PxU32 nbArticulations, ArticulationSolverDesc* PX_RESTRICT articulationListStart, Cm::SpatialVectorF* PX_RESTRICT deltaV,
								PxU32 nbPosIter, PxU32 nbVelIter, PxF32 dt, PxF32 invDt, bool residualReportingActive)
{
	saveMotionVelocities(nbBodies, solverBodies, motionVelocityArray);

	if(!nbArticulations)
		return;

	const PxF32 biasCoefficient = DY_ARTICULATION_PGS_BIAS_COEFFICIENT;
	const bool isTGS = false;

	//Even thought there are no external constraints, there may still be internal constraints in the articulations...
	for(PxU32 i=0; i<nbPosIter; i++)
		for(PxU32 j=0; j<nbArticulations; j++)
			articulationListStart[j].articulation->solveInternalConstraints(dt, invDt, false, isTGS, 0.0f, biasCoefficient, residualReportingActive);

	for(PxU32 i=0; i<nbArticulations; i++)
		ArticulationPImpl::saveVelocity(articulationListStart[i].articulation, deltaV);

	for(PxU32 i=0; i<nbVelIter; i++)
		for(PxU32 j=0; j<nbArticulations; j++)
			articulationListStart[j].articulation->solveInternalConstraints(dt, invDt, true, isTGS, 0.0f, biasCoefficient, residualReportingActive);

	for(PxU32 j=0; j<nbArticulations; j++)
		articulationListStart[j].articulation->writebackInternalConstraints(isTGS);
}
