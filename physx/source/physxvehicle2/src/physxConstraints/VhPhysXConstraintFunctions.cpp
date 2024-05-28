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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintFunctions.h"
#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintHelpers.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle2/suspension/PxVehicleSuspensionParams.h"
#include "vehicle2/suspension/PxVehicleSuspensionStates.h"

#include "vehicle2/tire/PxVehicleTireStates.h"

#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintStates.h"
#include "vehicle2/physxConstraints/PxVehiclePhysXConstraintParams.h"

namespace physx
{
namespace vehicle2
{


PX_FORCE_INLINE PxVec3 computeAngular
(const PxVehicleSuspensionParams& suspParams,
	const PxVehicleSuspensionComplianceState& suspComplianceState, const PxVehicleRigidBodyState& rigidBodyState,
	const PxVec3& direction)
{
	const PxVec3 cmOffset = rigidBodyState.pose.rotate(suspParams.suspensionAttachment.transform(suspComplianceState.tireForceAppPoint));
	const PxVec3 angular = cmOffset.cross(direction);
	// note: not normalized on purpose. The angular component should hold the raw cross product as that
	//       is needed for the type of constraint we want to set up (see vehicleConstraintSolverPrep).
	return angular;
}

PX_FORCE_INLINE PxVec3 computeTireAngular
(const PxVehicleSuspensionParams& suspParams, 
 const PxVehicleSuspensionComplianceState& suspComplianceState, const PxVehicleTireDirectionState& trDirState, const PxVehicleRigidBodyState& rigidBodyState,
const PxVehicleTireDirectionModes::Enum direction)
{
	return computeAngular(suspParams, suspComplianceState, rigidBodyState, trDirState.directions[direction]);
}

PX_FORCE_INLINE PxVec3 computeSuspAngular
(const PxVehicleSuspensionParams& suspParams,
 const PxVehicleSuspensionComplianceState& suspComplianceState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxVec3& direction)
{
	return computeAngular(suspParams, suspComplianceState, rigidBodyState, direction);
}

void PxVehiclePhysXConstraintStatesUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxVehiclePhysXSuspensionLimitConstraintParams& suspensionLimitParams,
 const PxVehicleSuspensionState& suspState, const PxVehicleSuspensionComplianceState& suspComplianceState,
 const PxVec3& groundPlaneNormal,
 const PxReal tireStickyDampingLong, const PxReal tireStickyDampingLat, 
 const PxVehicleTireDirectionState& trDirState, const PxVehicleTireStickyState& trStickyState,
 const PxVehicleRigidBodyState& rigidBodyState,
 PxVehiclePhysXConstraintState& cnstrtState)
{
	cnstrtState.setToDefault();

	//Sticky tire longitudinal
	{
		const bool isActive = trStickyState.activeStatus[PxVehicleTireDirectionModes::eLONGITUDINAL];
		cnstrtState.tireActiveStatus[PxVehicleTireDirectionModes::eLONGITUDINAL] = isActive;
		if (isActive)
		{
			cnstrtState.tireLinears[PxVehicleTireDirectionModes::eLONGITUDINAL] = trDirState.directions[PxVehicleTireDirectionModes::eLONGITUDINAL];
			cnstrtState.tireAngulars[PxVehicleTireDirectionModes::eLONGITUDINAL] = computeTireAngular(suspParams, suspComplianceState, trDirState, rigidBodyState, PxVehicleTireDirectionModes::eLONGITUDINAL);
			cnstrtState.tireDamping[PxVehicleTireDirectionModes::eLONGITUDINAL] = tireStickyDampingLong;
		}
	}

	//Sticky tire lateral
	{
		const bool isActive = trStickyState.activeStatus[PxVehicleTireDirectionModes::eLATERAL];
		cnstrtState.tireActiveStatus[PxVehicleTireDirectionModes::eLATERAL] = isActive;
		if (isActive)
		{
			cnstrtState.tireLinears[PxVehicleTireDirectionModes::eLATERAL] = trDirState.directions[PxVehicleTireDirectionModes::eLATERAL];
			cnstrtState.tireAngulars[PxVehicleTireDirectionModes::eLATERAL] = computeTireAngular(suspParams, suspComplianceState, trDirState, rigidBodyState, PxVehicleTireDirectionModes::eLATERAL);
			cnstrtState.tireDamping[PxVehicleTireDirectionModes::eLATERAL] = tireStickyDampingLat;
		}
	}

	//Suspension limit
	{
		if (suspState.separation >= 0.0f || PxVehiclePhysXSuspensionLimitConstraintParams::eNONE == suspensionLimitParams.directionForSuspensionLimitConstraint)
		{
			cnstrtState.suspActiveStatus = false;
		}
		else
		{
			// To maintain the wheel on the ground plane, the suspension is required to compress beyond the suspension compression limit
			// or expand beyond maximum droop (for example, top of wheel being in collision with something).
			// We manage the compression up to the limit with a suspension force.
			// Everything beyond the limits is managed with an impulse applied to the rigid body via a constraint.
			// The constraint attempts to resolve the geometric error declared in the separation state.
			// We have two choices:
			// 1) apply the impulse along the suspension dir (more like the effect of a bump stop spring)
			// 2) apply the impulse along the ground normal (more like the effect of a real tire's contact wtih the ground).
			if (PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL == suspensionLimitParams.directionForSuspensionLimitConstraint)
			{
				cnstrtState.suspActiveStatus = true;
				cnstrtState.suspGeometricError = suspState.separation;
				cnstrtState.suspLinear = groundPlaneNormal;
				cnstrtState.suspAngular = computeSuspAngular(suspParams, suspComplianceState, rigidBodyState, groundPlaneNormal);
				cnstrtState.restitution = suspensionLimitParams.restitution;
			}
			else
			{
				const PxVec3 suspDirWorldFrame = rigidBodyState.pose.rotate(suspParams.suspensionTravelDir);
				const PxF32 projection = groundPlaneNormal.dot(suspDirWorldFrame);
				if (projection != 0.0f)
				{
					cnstrtState.suspActiveStatus = true;
					cnstrtState.suspGeometricError = suspState.separation;
					const PxVec3 suspLinear = suspDirWorldFrame * PxSign(projection);
					cnstrtState.suspLinear = suspLinear;
					cnstrtState.suspAngular = computeSuspAngular(suspParams, suspComplianceState, rigidBodyState, suspLinear);
					cnstrtState.restitution = suspensionLimitParams.restitution;
				}
				else
				{
					cnstrtState.suspActiveStatus = false;
				}
			}
		}
	}
}

} //namespace vehicle2
} //namespace physx
