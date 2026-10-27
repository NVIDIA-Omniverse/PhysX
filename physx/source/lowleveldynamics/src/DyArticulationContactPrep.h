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

namespace Dy
{

	struct CorrelationBuffer;

	PxReal getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0,
		const SolverExtBody& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1,
		bool allowSelfCollision = false);

	aos::FloatV getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVectorV& impulse0, Cm::SpatialVectorV& deltaV0, const aos::FloatV& dom0, const aos::FloatV& angDom0,
		const SolverExtBody& b1, const Cm::SpatialVectorV& impulse1, Cm::SpatialVectorV& deltaV1, const aos::FloatV& dom1, const aos::FloatV& angDom1,
		bool allowSelfCollision = false);

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
		const PxReal biasCoefficient,
		PxReal invMassScale0, PxReal invInertiaScale0,
		PxReal invMassScale1, PxReal invInertiaScale1,
		PxReal restDistance, PxU8* frictionDataPtr,
		PxReal ccdMaxContactDist,
		Cm::SpatialVectorF* Z,
		const PxReal offsetSlop);

} //namespace Dy

}

#endif

