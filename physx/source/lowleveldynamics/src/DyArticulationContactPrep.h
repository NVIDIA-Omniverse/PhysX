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
   
#ifndef DY_ARTICULATION_CONTACT_PREP_H
#define DY_ARTICULATION_CONTACT_PREP_H

#include "DySolverExt.h"
#include "foundation/PxVecMath.h"

namespace physx
{
struct PxcNpWorkUnit;
class PxContactBuffer;
struct PxContactPoint;

namespace Dy
{

	struct CorrelationBuffer;

	PxReal getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0,
		const SolverExtBody& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1,
		Cm::SpatialVectorF* Z, bool allowSelfCollision = false);

	aos::FloatV getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVectorV& impulse0, Cm::SpatialVectorV& deltaV0, const aos::FloatV& dom0, const aos::FloatV& angDom0,
		const SolverExtBody& b1, const Cm::SpatialVectorV& impulse1, Cm::SpatialVectorV& deltaV1, const aos::FloatV& dom1, const aos::FloatV& angDom1,
		Cm::SpatialVectorV* Z, bool allowSelfCollision = false);

	Cm::SpatialVector createImpulseResponseVector(const PxVec3& linear, const PxVec3& angular, const SolverExtBody& body);
	Cm::SpatialVectorV createImpulseResponseVector(const aos::Vec3V& linear, const aos::Vec3V& angular, const SolverExtBody& body);

	void setupFinalizeExtSolverContacts(
		const PxContactPoint* buffer,
		const CorrelationBuffer& c,
		const PxTransform& bodyFrame0,
		const PxTransform& bodyFrame1,
		PxU8* workspace,
		const SolverExtBody& b0,
		const SolverExtBody& b1,
		const PxReal invDtF32,
		const PxReal dtF32,
		PxReal bounceThresholdF32,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		PxReal restDistance, PxU8* frictionDataPtr,
		PxReal ccdMaxContactDist,
		Cm::SpatialVectorF* Z,
		const PxReal offsetSlop);


	bool setupFinalizeExtSolverContactsCoulomb(
		const PxContactBuffer& buffer,
		const CorrelationBuffer& c,
		const PxTransform& bodyFrame0,
		const PxTransform& bodyFrame1,
		PxU8* workspace,
		PxReal invDt,
		PxReal dtF32,
		PxReal bounceThreshold,
		const SolverExtBody& b0,
		const SolverExtBody& b1,
		PxU32 frictionCountPerPoint,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		PxReal restDist,
		PxReal ccdMaxContactDist,
		Cm::SpatialVectorF* Z,
		const PxReal solverOffsetSlop);

} //namespace Dy

}

#endif

