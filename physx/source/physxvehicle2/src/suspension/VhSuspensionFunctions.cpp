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

#include "vehicle2/PxVehicleParams.h"

#include "vehicle2/roadGeometry/PxVehicleRoadGeometryState.h"

#include "vehicle2/rigidBody/PxVehicleRigidBodyStates.h"

#include "vehicle2/suspension/PxVehicleSuspensionParams.h"
#include "vehicle2/suspension/PxVehicleSuspensionFunctions.h"
#include "vehicle2/suspension/PxVehicleSuspensionHelpers.h"

namespace physx
{
namespace vehicle2
{

#define VH_SUSPENSION_NO_INTERSECTION_MARKER FLT_MIN

PX_FORCE_INLINE void computeJounceAndSeparation(const PxF32 depth, const PxF32 suspDirDotPlaneNormal,
	const PxF32 suspTravelDist, const PxF32 previousJounce,
	PxF32& jounce, PxF32& separation)
{
	if (suspDirDotPlaneNormal != 0.0f)
	{
		if (depth <= 0.0f)
		{
			//There is overlap at max droop

			const PxF32 suspDeltaToDepthZero = depth / suspDirDotPlaneNormal;

			if ((suspDeltaToDepthZero > 0.0f) && (suspDeltaToDepthZero <= suspTravelDist))
			{
				//The wheel can be placed on the plane for a jounce between max droop and max compression.
				jounce = suspDeltaToDepthZero;
				separation = 0.0f;
			}
			else
			{
				if (suspDeltaToDepthZero > suspTravelDist)
				{
					//There is overlap even at max compression. Compute the depth at max
					//compression.
					jounce = suspTravelDist;
					separation = (depth * (suspDeltaToDepthZero - suspTravelDist)) / suspDeltaToDepthZero;
				}
				else
				{
					PX_ASSERT(suspDeltaToDepthZero <= 0.0f);

					//There is overlap at max droop and in addition, the suspension would have to expand
					//beyond max droop to move out of plane contact. This scenario can be reached, for
					//example, if a car rolls over or if something touches a wheel from "above".
					jounce = 0.0f;
					separation = depth;
				}
			}
		}
		else
		{
			//At max droop there is no overlap => let the suspension fully expand.
			//Note that this check is important because without it, you can get unexpected
			//behavior like the wheel compressing just so that it touches the plane again.

			jounce = 0.0f;
			separation = VH_SUSPENSION_NO_INTERSECTION_MARKER;
		}
	}
	else
	{
		//The suspension direction and hit normal are perpendicular, thus no change to
		//suspension jounce will change the distance to the plane.

		if (depth >= 0.0f)
		{
			jounce = 0.0f;
			separation = VH_SUSPENSION_NO_INTERSECTION_MARKER;
		}
		else
		{
			jounce = (previousJounce != PX_VEHICLE_UNSPECIFIED_JOUNCE) ? previousJounce : suspTravelDist;
			separation = depth;
			// note: since The suspension direction and hit normal are perpendicular, the
			//       depth will be the same for all jounce values
		}
	}
}

PX_FORCE_INLINE void intersectRayPlane
(const PxVehicleFrame& frame, const PxVehicleWheelParams& wheelParams, const PxVehicleSuspensionParams& suspParams, 
 const PxF32 steerAngle, const PxVehicleRoadGeometryState& roadGeomState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxF32 previousJounce,
 PxVec3& suspDir, PxF32& jounce, PxF32& separation)
{
	//Compute the position on the wheel surface along the suspension direction that is closest to
	//the plane (for the purpose of computing the separation from the plane). The wheel center position
	//is chosen at max droop (zero jounce).
	PxVehicleSuspensionState suspState;
	suspState.setToDefault(0.0f);
	const PxTransform wheelPose = PxVehicleComputeWheelPose(frame, suspParams, suspState, 0.0f, 0.0f, steerAngle, 
		rigidBodyState.pose, 0.0f);

	suspDir = PxVehicleComputeSuspensionDirection(suspParams, rigidBodyState.pose);
	const PxPlane& hitPlane = roadGeomState.plane;
	const PxF32 suspDirDotPlaneNormal = suspDir.dot(hitPlane.n);

	PxVec3 wheelRefPoint;
	if (suspDirDotPlaneNormal < 0.0f)
	{
		wheelRefPoint = wheelPose.p + (suspDir * wheelParams.radius);
		//The "wheel bottom" has to be placed on the plane to resolve collision
	}
	else if (suspDirDotPlaneNormal > 0.0f)
	{
		wheelRefPoint = wheelPose.p - (suspDir * wheelParams.radius);
		//The "wheel top" has to be placed on the plane to resolve collision
	}
	else
	{
		//The hit normal and susp dir are prependicular
		//-> any point along the suspension direction will do to compute the depth
		wheelRefPoint = wheelPose.p;
	}

	//Compute the penetration depth of the reference point with respect to the plane
	const PxF32 depth = hitPlane.n.dot(wheelRefPoint) + hitPlane.d;

	//How far along the susp dir do we have to move to place the wheel exactly on the plane.
	computeJounceAndSeparation(depth, suspDirDotPlaneNormal, suspParams.suspensionTravelDist, previousJounce,
		jounce, separation);
}

PX_FORCE_INLINE bool intersectPlanes(const PxPlane& a, const PxPlane& b, PxVec3& v, PxVec3& w)
{
	const PxF32 n1x = a.n.x;
	const PxF32 n1y = a.n.y;
	const PxF32 n1z = a.n.z;
	const PxF32 n1d = a.d;

	const PxF32 n2x = b.n.x;
	const PxF32 n2y = b.n.y;
	const PxF32 n2z = b.n.z;
	const PxF32 n2d = b.d;

	PxF32 dx = (n1y * n2z) - (n1z * n2y);
	PxF32 dy = (n1z * n2x) - (n1x * n2z);
	PxF32 dz = (n1x * n2y) - (n1y * n2x);

	const PxF32 dx2 = dx * dx;
	const PxF32 dy2 = dy * dy;
	const PxF32 dz2 = dz * dz;

	PxF32 px, py, pz;
	if ((dz2 > dy2) && (dz2 > dx2) && (dz2 > 0))
	{
		px = ((n1y * n2d) - (n2y * n1d)) / dz;
		py = ((n2x * n1d) - (n1x * n2d)) / dz;
		pz = 0;
	}
	else if ((dy2 > dx2) && (dy2 > 0))
	{
		px = -((n1z * n2d) - (n2z * n1d)) / dy;
		py = 0;
		pz = -((n2x * n1d) - (n1x * n2d)) / dy;
	}
	else if (dx2 > 0)
	{
		px = 0;
		py = ((n1z * n2d) - (n2z * n1d)) / dx;
		pz = ((n2y * n1d) - (n1y * n2d)) / dx;
	}
	else
	{
		px = 0;
		py = 0;
		pz = 0;
		return false;
	}

	const PxF32 ld = PxSqrt(dx2 + dy2 + dz2);

	dx /= ld;
	dy /= ld;
	dz /= ld;

	w = PxVec3(dx, dy, dz);
	v = PxVec3(px, py, pz);

	return true;
}


// This method computes how much a wheel cylinder object at max droop (fully elongated suspension)
// has to be pushed along the suspension direction to end up just touching the plane provided in
// the road geometry state.
//
// output:
// suspDir: suspension direction in the world frame
// jounce: the suspension jounce.
//         jounce=0 the wheel is at max droop
//         jounce>0 the suspension is compressed by length jounce (up to max compression = suspensionTravelDist)
//
//         The jounce value will be set to 0 (in the case of no overlap) or to the previous jounce
//         (in the case of overlap) for the following special cases:
//         - the plane normal and the wheel lateral axis are parallel
//         - the plane normal and the suspension direction are perpendicular
// separation: 0 if the suspension can move between max droop and max compression to place
//             the wheel on the ground.
//             A negative value denotes by how much the wheel cylinder penetrates (along the hit 
//             plane normal) into the ground for the computed jounce.
//             A positive value denotes that the wheel does not touch the ground.
//
PX_FORCE_INLINE void intersectCylinderPlane
(const PxVehicleFrame& frame, 
 const PxVehicleWheelParams& whlParams, const PxVehicleSuspensionParams& suspParams, 
 const PxF32 steerAngle, const PxVehicleRoadGeometryState& roadGeomState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxF32 previousJounce,
 PxVec3& suspDir, PxF32& jounce, PxF32& separation)
{
	const PxPlane& hitPlane = roadGeomState.plane;
	const PxF32 radius = whlParams.radius;
	const PxF32 halfWidth = whlParams.halfWidth;

	//Compute the wheel pose at zero jounce ie at max droop.
	PxTransform wheelPoseAtZeroJounce;
	{
		PxTransform start;
		PxF32 dist;
		PxVehicleComputeSuspensionSweep(frame, suspParams, steerAngle, rigidBodyState.pose, start, suspDir, dist);
		wheelPoseAtZeroJounce = PxTransform(start.p + suspDir*dist, start.q);
	}

	//Compute the plane of the wheel.
	PxPlane wheelPlane;
	{
		wheelPlane = PxPlane(wheelPoseAtZeroJounce.p, wheelPoseAtZeroJounce.rotate(frame.getLatAxis()));
	}

	//Intersect the plane of the wheel with the hit plane.
	//This generates an intersection edge.
	PxVec3 intersectionEdgeV, intersectionEdgeW;
	const bool intersectPlaneSuccess = intersectPlanes(wheelPlane, hitPlane, intersectionEdgeV, intersectionEdgeW);

	PxF32 depth;
	if (intersectPlaneSuccess)
	{
		//Compute the position on the intersection edge that is closest to the wheel centre.
		PxVec3 closestPointOnIntersectionEdge;
		{
			const PxVec3& p = wheelPoseAtZeroJounce.p;
			const PxVec3& dir = intersectionEdgeW;
			const PxVec3& v = intersectionEdgeV;
			const PxF32 t = (p - v).dot(dir);
			closestPointOnIntersectionEdge = v + dir * t;
		}

		//Compute the vector that joins the wheel centre to the intersection edge;
		PxVec3 dir;
		{
			const PxF32 wheelCentreD = hitPlane.n.dot(wheelPoseAtZeroJounce.p) + hitPlane.d;
			dir = ((wheelCentreD >= 0) ? closestPointOnIntersectionEdge - wheelPoseAtZeroJounce.p : wheelPoseAtZeroJounce.p - closestPointOnIntersectionEdge);
			dir.normalize();
		}

		//Compute the point on the disc diameter that will be the closest to the hit plane or the deepest inside the hit plane.
		const PxVec3 pos = wheelPoseAtZeroJounce.p + dir*radius;

		//Now compute the maximum depth of the inside and outside discs against the plane.
		{
			const PxVec3& latDir = wheelPlane.n;
			const PxF32 signDot = PxVehicleComputeSign(hitPlane.n.dot(latDir));
			const PxVec3 deepestPos = pos - latDir * (signDot*halfWidth);
			depth = hitPlane.n.dot(deepestPos) + hitPlane.d;
		}
	}
	else
	{
		//The hit plane normal and the wheel lateral axis are parallel

		//Now compute the maximum depth of the inside and outside discs against the plane.
		const PxVec3& latDir = wheelPlane.n;
		const PxF32 signDot = PxVehicleComputeSign(hitPlane.n.dot(latDir));
		depth = hitPlane.d - halfWidth - (signDot*wheelPlane.d);

		// examples:
		// halfWidth = 0.5
		//
		// wheelPlane.d   hitplane.d   depth
		//=============================================
		// -3 ->          -3.5 ->      -1
		//  3 <-          -3.5 ->      -1
		//
		// -3 ->           3.5 <-       0
		//  3 <-           3.5 <-       0
		//
	}
	
	//How far along the susp dir do we have to move to place the wheel exactly on the plane.
	const PxF32 suspDirDotPlaneNormal = hitPlane.n.dot(suspDir);
	computeJounceAndSeparation(depth, suspDirDotPlaneNormal, suspParams.suspensionTravelDist, previousJounce,
		jounce, separation);
}

static void limitSuspensionExpansionVelocity
(const PxReal jounceSpeed, const PxReal previousJounceSpeed, const PxReal previousJounce,  
 const PxReal suspStiffness, const PxReal suspDamping,
 const PxVec3& suspDirWorld, const PxReal wheelMass, 
 const PxReal dt, const PxVec3& gravity, const bool hasGroundHit,
 PxVehicleSuspensionState& suspState)
{
	PX_ASSERT(jounceSpeed < 0.0f);  // don't call this method if the suspension is not expanding

	// The suspension is expanding. Check if the suspension can expand fast enough to actually reach the
	// target jounce within the given time step.

	// Without the suspension elongating, the wheel would end up in the air. Compute the suspension force
	// that pushes the wheel towards the ground. Note that gravity is ignored here as it applies to sprung 
	// mass and wheel equally.
	const PxReal springForceAlongSuspDir = (previousJounce * suspStiffness);
	const PxReal suspDirVelSpring = (springForceAlongSuspDir / wheelMass) * dt;
	const PxReal dampingForceAlongSuspDir = (previousJounceSpeed * suspDamping);  // note: negative jounce speed = expanding
	const PxReal suspDirVelDamping = (dampingForceAlongSuspDir / wheelMass) * dt;
	PxReal suspDirVel = suspDirVelSpring - previousJounceSpeed;
	// add damping part but such that it does not flip the velocity sign (covering case of
	// crazily high damping values)
	const PxReal suspDirVelTmp = suspDirVel + suspDirVelDamping;
	if (suspDirVel >= 0.0f)
		suspDirVel = PxMax(0.0f, suspDirVelTmp);
	else
		suspDirVel = PxMin(0.0f, suspDirVelTmp);

	const PxReal gravitySuspDir = gravity.dot(suspDirWorld);

	PxReal velocityThreshold;
	if (hasGroundHit)
	{
		velocityThreshold = (gravitySuspDir > 0.0f) ? suspDirVel + (gravitySuspDir * dt) : suspDirVel;
		// gravity is considered too as it affects the wheel and can close the distance to the ground
		// too. External forces acting on the sprung mass are ignored as those propagate
		// through the suspension to the wheel.
		// If gravity points in the opposite direction of the suspension travel direction, it is
		// ignored. The suspension should never elongate more than what's given by the current delta
		// jounce (defined by jounceSpeed, i.e., jounceSpeed < -suspDirVel has to hold all the time
		// for the clamping to take place).
	}
	else
	{
		velocityThreshold = suspDirVel;
		// if there was no hit detected, the gravity will not be taken into account since there is no
		// ground to move towards.
	}

	if (jounceSpeed < (-velocityThreshold))
	{
		// The suspension can not expand fast enough to place the wheel on the ground. As a result, 
		// the scenario is interpreted as if there was no hit and the wheels end up in air. The 
		// jounce is adjusted based on the clamped velocity to not have it snap to the target immediately.
		// note: could consider applying the suspension force to the sprung mass too but the complexity
		//       seems high enough already.

		const PxReal expansionDelta = suspDirVel * dt;
		const PxReal clampedJounce = previousJounce - expansionDelta;

		PX_ASSERT(clampedJounce >= 0.0f);
		// No need to cover the case of a negative jounce as the input jounce speed is expected to make
		// sure that no negative jounce would result (and the speed considered here is smaller than the
		// non-clamped input jounce speed).

		if (suspState.separation >= 0.0f)  // do not adjust separation if ground penetration was detected
		{
			suspState.separation = clampedJounce - suspState.jounce;
		}
		suspState.jounce = clampedJounce;
		suspState.jounceSpeed = -suspDirVel;
	}
}

void PxVehicleSuspensionStateUpdate
(const PxVehicleWheelParams& whlParams, const PxVehicleSuspensionParams& suspParams, const PxVehicleSuspensionStateCalculationParams& suspStateCalcParams,
 const PxReal suspStiffness, const PxReal suspDamping,
 const PxF32 steerAngle, const PxVehicleRoadGeometryState& roadGeomState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxReal dt, const PxVehicleFrame& frame, const PxVec3& gravity,
 PxVehicleSuspensionState& suspState)
{
	const PxReal prevJounce = suspState.jounce;
	const PxReal prevJounceSpeed = suspState.jounceSpeed;

	suspState.setToDefault(0.0f, VH_SUSPENSION_NO_INTERSECTION_MARKER);

	if(!roadGeomState.hitState)
	{
		if (suspStateCalcParams.limitSuspensionExpansionVelocity && (prevJounce != PX_VEHICLE_UNSPECIFIED_JOUNCE))
		{
			if (prevJounce > 0.0f)
			{
				PX_ASSERT(suspState.jounce == 0.0f);  // the expectation is that setToDefault() above does this
				const PxReal jounceSpeed = (-prevJounce) / dt;
				const PxVec3 suspDirWorld = PxVehicleComputeSuspensionDirection(suspParams, rigidBodyState.pose);
				limitSuspensionExpansionVelocity(jounceSpeed, prevJounceSpeed, prevJounce, 
					suspStiffness, suspDamping, suspDirWorld, whlParams.mass, dt, gravity, false,
					suspState);
			}
		}

		return;
	}

	PxVec3 suspDir;
	PxF32 currJounce;
	PxF32 separation;
	switch(suspStateCalcParams.suspensionJounceCalculationType)
	{
		case PxVehicleSuspensionJounceCalculationType::eRAYCAST:
		{
			//Compute the distance along the suspension direction that places the wheel on the ground plane.
			intersectRayPlane(frame, whlParams, suspParams, steerAngle, roadGeomState, rigidBodyState, 
				prevJounce,
				suspDir, currJounce, separation);
		}
		break;
		case PxVehicleSuspensionJounceCalculationType::eSWEEP:
		{
			//Compute the distance along the suspension direction that places the wheel on the ground plane.
			intersectCylinderPlane(frame, whlParams, suspParams, steerAngle, roadGeomState, rigidBodyState, 
				prevJounce,
				suspDir, currJounce, separation);
		}
		break;
		default:
		{
			PX_ASSERT(false);
			currJounce = 0.0f;
			separation = VH_SUSPENSION_NO_INTERSECTION_MARKER;
		}
		break;
	}

	suspState.jounce = currJounce;
	suspState.jounceSpeed = (PX_VEHICLE_UNSPECIFIED_JOUNCE != prevJounce) ? (currJounce - prevJounce) / dt : 0.0f;
	suspState.separation = separation;

	if (suspStateCalcParams.limitSuspensionExpansionVelocity && (suspState.jounceSpeed < 0.0f))
	{
		limitSuspensionExpansionVelocity(suspState.jounceSpeed, prevJounceSpeed, prevJounce, 
			suspStiffness, suspDamping, suspDir, whlParams.mass, dt, gravity, true,
			suspState);
	}
}

void PxVehicleSuspensionComplianceUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxVehicleSuspensionComplianceParams& compParams,
 const PxVehicleSuspensionState& suspState,
 PxVehicleSuspensionComplianceState& compState)
{
	compState.setToDefault();

	//Compliance is normalised in range (0,1) with:
	//0 being fully elongated (ie jounce = 0)
	//1 being fully compressed (ie jounce = maxTravelDist)
	const PxF32 jounce = suspState.jounce;
	const PxF32 maxTravelDist = suspParams.suspensionTravelDist;
	const PxF32 r = jounce/maxTravelDist;

	//Camber and toe relative the wheel reference pose.
	{
		compState.camber = compParams.wheelCamberAngle.interpolate(r);
		compState.toe = compParams.wheelToeAngle.interpolate(r);
	}

	//Tire force application point as an offset from wheel reference pose.
	{
		compState.tireForceAppPoint = compParams.tireForceAppPoint.interpolate(r);
	}

	//Susp force application point as an offset from wheel reference pose.
	{
		compState.suspForceAppPoint = compParams.suspForceAppPoint.interpolate(r);
	}
}

PX_FORCE_INLINE void setSuspensionForceAndTorque
(const PxVehicleSuspensionParams& suspParams, 
 const PxVehicleRoadGeometryState& roadGeomState, const PxVehicleSuspensionComplianceState& suspComplianceState, const PxVehicleRigidBodyState& rigidBodyState, 
 const PxF32 suspForceMagnitude, const PxF32 normalForceMagnitude,
 PxVehicleSuspensionForce& suspForce)
{
	const PxVec3 r = rigidBodyState.pose.rotate(suspParams.suspensionAttachment.transform(suspComplianceState.suspForceAppPoint));
	const PxVec3 f = roadGeomState.plane.n*suspForceMagnitude;
	suspForce.force = f;
	suspForce.torque = r.cross(f);
	suspForce.normalForce = normalForceMagnitude;
}

void PxVehicleSuspensionForceUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxVehicleSuspensionForceParams& suspForceParams,
 const PxVehicleRoadGeometryState& roadGeom, const PxVehicleSuspensionState& suspState,
 const PxVehicleSuspensionComplianceState& compState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxVec3& gravity, const PxReal vehicleMass,
 PxVehicleSuspensionForce& suspForces)
{
	suspForces.setToDefault();

	//If the wheel cannot touch the ground then carry on with zero force.
	if (!PxVehicleIsWheelOnGround(suspState))
		return;

	//For the following computations, the external force Fe acting on the suspension
	//is seen as the sum of two forces Fe0 and Fe1. Fe0 acts along the suspension 
	//direction while Fe1 is the part perpendicular to Fe0. The suspension spring
	//force Fs0 works against Fe0, while the suspension "collision" force Fs1 works
	//against Fe1 (can be seen as a force that is trying to push the suspension
	//into the ground in a direction where the spring can not do anything against
	//it because it's perpendicular to the spring direction).
	//For the system to be at rest, we require Fs0 = -Fe0 and Fs1 = -Fe1
	//The forces Fe0 and Fe1 can each be split further into parts that act along
	//the ground patch normal and parts that act parallel to the ground patch.
	//Fs0 and Fs1 work against the former as the ground prevents the suspension
	//from penetrating. However, to work against the latter, friction would have
	//to be considered. The current model does not do this (or rather: it is the
	//tire model that deals with forces parallel to the ground patch). As a result,
	//only the force parts of Fs0 and Fs1 perpendicular to the ground patch are
	//considered here. Furthermore, Fs1 is set to zero, if the external force is
	//not pushing the suspension towards the ground patch (imagine a vehicle with
	//a "tilted" suspension direction "driving" up a wall. No "collision" force
	//should be added in such a scenario).

	PxF32 suspForceMagnitude = 0.0f;
	{
		const PxF32 jounce = suspState.jounce;
		const PxF32 stiffness = suspForceParams.stiffness;
		const PxF32 jounceSpeed = suspState.jounceSpeed;
		const PxF32 damping = suspForceParams.damping;

		const PxVec3 suspDirWorld = PxVehicleComputeSuspensionDirection(suspParams, rigidBodyState.pose);
		const PxVec3 suspSpringForce = suspDirWorld * (-(jounce*stiffness + jounceSpeed*damping));
		const PxF32 suspSpringForceProjected = roadGeom.plane.n.dot(suspSpringForce);
		suspForceMagnitude = suspSpringForceProjected;

		const PxVec3 comToSuspWorld = rigidBodyState.pose.rotate(suspParams.suspensionAttachment.p);
		const PxVec3 externalForceLin = (gravity * vehicleMass) + rigidBodyState.externalForce;

		const PxReal comToSuspDistSqr = comToSuspWorld.magnitudeSquared();
		PxVec3 externalForceAng;
		if (comToSuspDistSqr > 0.0f)
		{
			// t = r x F
			// t x r = (r x F) x r = -[r x (r x F)] = -[((r o F) * r)  -  ((r o r) * F)]
			// r and F perpendicular (r o F = 0) => = (r o r) * F = |r|^2 * F

			externalForceAng = (rigidBodyState.externalTorque.cross(comToSuspWorld)) / comToSuspDistSqr;
		}
		else
			externalForceAng = PxVec3(PxZero);

		const PxVec3 externalForce = externalForceLin + externalForceAng;

		const PxVec3 externalForceSusp = externalForce * (suspForceParams.sprungMass / vehicleMass);
		if (roadGeom.plane.n.dot(externalForceSusp) < 0.0f)
		{
			const PxF32 suspDirExternalForceMagn = suspDirWorld.dot(externalForceSusp);
			const PxVec3 collisionForce = externalForceSusp - (suspDirWorld * suspDirExternalForceMagn);
			const PxF32 suspCollisionForceProjected = -roadGeom.plane.n.dot(collisionForce);
			suspForceMagnitude += suspCollisionForceProjected;
		}
	}

	setSuspensionForceAndTorque(suspParams, roadGeom, compState, rigidBodyState, suspForceMagnitude, suspForceMagnitude, suspForces);
}

void PxVehicleSuspensionLegacyForceUpdate
(const PxVehicleSuspensionParams& suspParams,
 const PxVehicleSuspensionForceLegacyParams& suspForceParamsLegacy,
 const PxVehicleRoadGeometryState& roadGeomState, const PxVehicleSuspensionState& suspState,
 const PxVehicleSuspensionComplianceState& compState, const PxVehicleRigidBodyState& rigidBodyState,
 const PxVec3& gravity, 
 PxVehicleSuspensionForce& suspForces)
{
	suspForces.setToDefault();

	//If the wheel cannot touch the ground then carry on with zero force.
	if (!PxVehicleIsWheelOnGround(suspState))
		return;

	PxF32 suspForceMagnitude = 0.0f;
	{
		//Get the params for the legacy model.
		//const PxF32 restDistance = suspForceParamsLegacy.restDistance;
		const PxF32 sprungMass = suspForceParamsLegacy.sprungMass;
		const PxF32 restDistance = suspForceParamsLegacy.restDistance;

		//Get the non-legacy params.
		const PxF32 stiffness = suspForceParamsLegacy.stiffness;
		const PxF32 damperRate = suspForceParamsLegacy.damping;
		const PxF32 maxTravelDist = suspParams.suspensionTravelDist;

		//Suspension state.
		const PxF32 jounce = suspState.jounce;
		const PxF32 jounceSpeed = suspState.jounceSpeed;

		//Decompose gravity into a term along w and a term perpendicular to w
		//gravity = w*alpha + T*beta
		//where T is a unit vector perpendicular to w; alpha and beta are scalars.
		//The vector w*alpha*mass is the component of gravitational force that acts along the spring direction.
		//The vector T*beta*mass is the component of gravitational force that will be resisted by the spring 
		//because the spring only supports a single degree of freedom along w.
		//We only really need to know T*beta so don't bother calculating T or beta.
		const PxVec3 w = PxVehicleComputeSuspensionDirection(suspParams, rigidBodyState.pose);
	
		const PxF32 gravitySuspDir = gravity.dot(w);
		const PxF32 alpha = PxMax(0.0f, gravitySuspDir);
		const PxVec3 TTimesBeta = (0.0f != alpha) ? gravity - w * alpha : PxVec3(0, 0, 0);
		//Compute the magnitude of the force along w.
		PxF32 suspensionForceW =
			PxMax(0.0f,
				sprungMass*alpha + 									//force to support sprung mass at zero jounce
				stiffness*(jounce + restDistance - maxTravelDist));	//linear spring
		suspensionForceW += jounceSpeed * damperRate;				//damping

		//Compute the total force acting on the suspension.
		//Remember that the spring force acts along -w.
		//Remember to account for the term perpendicular to w and that it acts along -TTimesBeta
		suspForceMagnitude =  roadGeomState.plane.n.dot(-w * suspensionForceW - TTimesBeta * sprungMass);

	}

	setSuspensionForceAndTorque(suspParams, roadGeomState, compState, rigidBodyState, suspForceMagnitude, suspForceMagnitude, suspForces);
}

void PxVehicleAntiRollForceUpdate
(const PxVehicleArrayData<const PxVehicleSuspensionParams>& suspensionParams,
	const PxVehicleSizedArrayData<const PxVehicleAntiRollForceParams>& antiRollParams,
	const PxVehicleArrayData<const PxVehicleSuspensionState>& suspensionStates, 
	const PxVehicleArrayData<const PxVehicleSuspensionComplianceState>& complianceStates,
	const PxVehicleRigidBodyState& rigidBodyState,
	PxVehicleAntiRollTorque& antiRollTorque)
{
	antiRollTorque.setToDefault();

	for (PxU32 i = 0; i < antiRollParams.size; i++)
	{
		if (antiRollParams[i].stiffness != 0.0f)
		{
			const PxU32 w0 = antiRollParams[i].wheel0;
			const PxU32 w1 = antiRollParams[i].wheel1;

			const bool w0InAir = (0.0f == suspensionStates[w0].jounce);
			const bool w1InAir = (0.0f == suspensionStates[w1].jounce);

			if (!w0InAir || !w1InAir)
			{
				//Compute the difference in jounce and compute the force.
				const PxF32 w0Jounce = suspensionStates[w0].jounce;
				const PxF32 w1Jounce = suspensionStates[w1].jounce;
				const PxF32 antiRollForceMag = (w0Jounce - w1Jounce) * antiRollParams[i].stiffness;

				//Apply the antiRollForce postiviely to wheel0, negatively to wheel 1
				PxU32 wheelIds[2] = { 0xffffffff, 0xffffffff };
				PxF32 antiRollForceMags[2];
				PxU32 numWheelIds = 0;
				if (!w0InAir)
				{
					wheelIds[numWheelIds] = w0;
					antiRollForceMags[numWheelIds] = -antiRollForceMag;
					numWheelIds++;
				}
				if (!w1InAir)
				{
					wheelIds[numWheelIds] = w1;
					antiRollForceMags[numWheelIds] = +antiRollForceMag;
					numWheelIds++;
				}

				for (PxU32 j = 0; j < numWheelIds; j++)
				{
					const PxU32 wheelId = wheelIds[j];
					const PxVec3& antiRollForceDir = suspensionParams[wheelId].suspensionTravelDir;
					const PxVec3 antiRollForce = antiRollForceDir * antiRollForceMags[j];
					const PxVec3 r = suspensionParams[wheelId].suspensionAttachment.transform(complianceStates[wheelId].suspForceAppPoint);
					antiRollTorque.antiRollTorque += rigidBodyState.pose.rotate(r.cross(antiRollForce));
				}
			}
		}
	}
}



} //namespace vehicle2
} //namespace physx
