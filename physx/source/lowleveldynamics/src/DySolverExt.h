// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_EXT_H
#define DY_SOLVER_EXT_H

#include "CmSpatialVector.h"
#include "foundation/PxVecMath.h"

namespace physx
{
struct PxSolverBody;
struct PxSolverBodyData;

namespace Dy
{
class FeatherstoneArticulation;

class SolverExtBody
{
public:
	union
	{
		const FeatherstoneArticulation* mArticulation;
		const PxSolverBody* mBody;
	};
	const PxSolverBodyData* mBodyData;

	PxU32 mLinkIndex;

	SolverExtBody(const void* bodyOrArticulationOrSoftBody, const void* bodyData, PxU32 linkIndex) :
		mBody(reinterpret_cast<const PxSolverBody*>(bodyOrArticulationOrSoftBody)),
		mBodyData(reinterpret_cast<const PxSolverBodyData*>(bodyData)),
		mLinkIndex(linkIndex)
	{}

	PxReal projectVelocity(const PxVec3& linear, const PxVec3& angular) const;
	aos::FloatV projectVelocity(const aos::Vec3V& linear, const aos::Vec3V& angular) const;

	Cm::SpatialVectorV getVelocity() const;
	PxReal getCFM() const;
};

}

}

#endif
