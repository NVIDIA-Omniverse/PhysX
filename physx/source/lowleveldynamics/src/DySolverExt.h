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

#ifndef DY_SOLVER_EXT_H
#define DY_SOLVER_EXT_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "CmSpatialVector.h"
#include "foundation/PxVecMath.h"

namespace physx
{

class PxsRigidBody;
struct PxsBodyCore;
struct PxSolverBody;
struct PxSolverBodyData;


namespace Dy
{


class FeatherstoneArticulation;
struct SolverConstraint1D;

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

	SolverExtBody(const void* bodyOrArticulationOrSoftBody, const void* bodyData, PxU32 linkIndex): 
	  mBody(reinterpret_cast<const PxSolverBody*>(bodyOrArticulationOrSoftBody)),
	  mBodyData(reinterpret_cast<const PxSolverBodyData*>(bodyData)),
		  mLinkIndex(linkIndex)
	  {}

	  void getResponse(const PxVec3& linImpulse, const PxVec3& angImpulse,
					   PxVec3& linDeltaV, PxVec3& angDeltaV, PxReal dominance) const;

	  void getResponse(const aos::Vec3V& linImpulse, const aos::Vec3V& angImpulse,
		  aos::Vec3V& linDeltaV, aos::Vec3V& angDeltaV, aos::FloatV dominance) const;

	  PxReal projectVelocity(const PxVec3& linear, const PxVec3& angular) const;
	  aos::FloatV projectVelocity(const aos::Vec3V& linear, const aos::Vec3V& angular) const;
	  PxVec3 getLinVel() const;
	  PxVec3 getAngVel() const;

	  aos::Vec3V getLinVelV() const;
	  aos::Vec3V getAngVelV() const;

	  Cm::SpatialVectorV getVelocity() const;
	  PxReal getCFM() const;
};

}

}

#endif
