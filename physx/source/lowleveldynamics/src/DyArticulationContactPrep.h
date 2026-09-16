// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

   
#ifndef DY_ARTICULATION_CONTACT_PREP_H
#define DY_ARTICULATION_CONTACT_PREP_H

#include "DySolverExt.h"
#include "foundation/PxVecMath.h"

namespace physx
{
struct PxcNpWorkUnit;
class PxContactBuffer;
struct PxContactPoint;

struct PxSolverContactDesc;

namespace Dy
{
	struct CorrelationBuffer;

	PxReal getImpulseResponse(	const SolverExtBody& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0,
								const SolverExtBody& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1,
								bool allowSelfCollision = false);

	Cm::SpatialVector createImpulseResponseVector(const PxVec3& linear, const PxVec3& angular, const SolverExtBody& body);
	Cm::SpatialVectorV createImpulseResponseVector(const aos::Vec3V& linear, const aos::Vec3V& angular, const SolverExtBody& body);

	void setupFinalizeExtSolverContacts(
		const PxSolverContactDesc& contactDesc,
		const CorrelationBuffer& c,
		PxU8* workspace,
		const SolverExtBody& b0,
		const SolverExtBody& b1,
		PxReal invDtF32,
		PxReal dtF32,
		PxReal bounceThresholdF32,
		PxReal biasCoefficient,
		PxU8* frictionDataPtr);

} //namespace Dy

}

#endif

