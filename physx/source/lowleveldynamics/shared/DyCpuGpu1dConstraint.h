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

#ifndef DY_1DCONSTRAINT_CPUGPU_H
#define DY_1DCONSTRAINT_CPUGPU_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxMemory.h"
#include "foundation/PxVecMath.h"
#include "PxConstraintDesc.h"

#include <stdio.h>

namespace physx
{
namespace Dy
{

/**
\brief Compute the reciprocal of the unit response and avoid divide-by-small-number numerical failures.
\param[in] unitResponse has value J * M^-1 * J^T with J the constraint Jacobian and M the constraint mass matrix.
\param[in] minRowResponse is the smallest value of unitResponse that the constraint is configured to support.
\return 0 if unitResponse is smaller than the smallest supported value, 1/unitResponse otherwise.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal computeRecipUnitResponse(const PxReal unitResponse, const PxReal minRowResponse) 
{
	const PxReal recipResponse = unitResponse <= minRowResponse ? 0 : 1.0f/unitResponse;
	return recipResponse;
}

/**
\brief Compute the minimum and maximum allowed impulse from the user-specified values that were specified 
as either a force or an impulse. 
\param[in] minSpecifiedImpulseOrForce is either the minimum allowed impulse or the minimum allowed force depending on the driveLimitsAreForces flag.
\param[in] maxSpecifiedImpulseOrForce is either the maximum allowed impulse or the maximum allowed force depending on the driveLimitsAreForces flag.
\param[in] hasDriveLimit is true if a minimum/maximum pair have been specified and is false if the allowed impulses are unbounded.
\param[in] driveLimitsAreForces describes whether minSpecifiedImpulseOrForce and maxSpecifiedImpulseOrForce represent forces or impulses.
\param[in] simDt is the duration of the scene simulation step.
\param[out] minImpulseToUseInSolver is the minimum allowed impulse.
\param[out] maxImpulseToUseInSolver is the maximum allowed impulse.
\note minImpulseToUseInSolver is equal to minSpecifiedImpulseOrForce*simDt if both hasDriveLimit and driveLimitsAreForces are true. 
\note maxImpulseToUseInSolver is equal to maxSpecifiedImpulseOrForce*simDt if both hasDriveLimit and driveLimitsAreForces are true. 
\note minImpulseToUseInSolver is equal to minSpecifiedImpulseOrForce if either hasDriveLimit or driveLimitsAreForces are false. 
\note maxImpulseToUseInSolver is equal to maxSpecifiedImpulseOrForce if either hasDriveLimit or driveLimitsAreForces are false. 
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE void computeMinMaxImpulseOrForceAsImpulse
(const PxReal minSpecifiedImpulseOrForce, const PxReal maxSpecifiedImpulseOrForce,
 const bool hasDriveLimit, const bool driveLimitsAreForces, const PxReal simDt,
 PxReal& minImpulseToUseInSolver, PxReal& maxImpulseToUseInSolver)
{
	const PxReal driveScale = (hasDriveLimit && driveLimitsAreForces) ? simDt : 1.0f;
	minImpulseToUseInSolver = minSpecifiedImpulseOrForce*driveScale;
	maxImpulseToUseInSolver = maxSpecifiedImpulseOrForce*driveScale;
}

/**
\brief Compute the values jointSpeedForRestitutionBounce and initJointSpeed that will be used in compute1dConstraintSolverConstantsPGS().
\param[in] normalVel0 is the projection of the velocity of body 0 projected against the corresponding Jacobian terms.
\param[in] isRigidDynamic0 is true if body 0 is a rigid dynamic and false if it is an articulation link.
\param[in] normalVel1 is the projection of the velocity of body 1 projected against the corresponding Jacobian terms.
\param[in] isRigidDynamic1 is true if body 1 is a rigid dynamic and false if it is an articulation link.
\param[out] jointSpeedForRestitutionBounce is the projection of the velocity of the constraint's body pair against the constraint Jacobian.
param[out] initJointSpeed is the initial speed of the constraint as experienced by the solver. In the absence of rigid dynamics, this will have value 0.0.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE void computeJointSpeedPGS
(const PxReal normalVel0, const bool isRigidDynamic0, const PxReal normalVel1, const bool isRigidDynamic1,
 PxReal& jointSpeedForRestitutionBounce, PxReal& initJointSpeed)
{
	// this helper is only meant to be used for scenarios where at least one body is an articulation link
#if defined(__CUDACC__)
	assert(!(isRigidDynamic0 && isRigidDynamic1));  // until PX_ASSERT works on GPU (see PX-4133)
#else
	PX_ASSERT(!(isRigidDynamic0 && isRigidDynamic1));
#endif

	//
	// The solver tries to achieve a specified relative target velocity vT.
	// Comparing to the current relative velocity provides the velocity error
	// verr that has to resolved:
	// verr = vT - (v0 - v1)
	// 
	// However, for rigid dynamics, the PGS solver does not work with the actual
	// velocities of the bodies but the delta velocity dv (compared to the start).
	// As such, the body velocities at the first solver step will be 0.
	// 
	// v = vStart + dv
	// 
	// verr = vT - ((v0Start + dv0) - (v1Start + dv1))
	//      = vT - (v0Start - v1Start) - (dv0 - dv1)
	//      = vT' - (dv0 - dv1)
	// 
	// As shown above, this can be achieved by initially shifting the target velocity,
	// using vT' instead of vT.
	// initJointSpeed will hold this shift. Note that articulation links do use the
	// actual velocity in the solver and thus the code here only computes a non zero
	// shift if rigid dynamics are involved.
	//

	initJointSpeed = 0.f;
	if (isRigidDynamic0)
		initJointSpeed = normalVel0;
	else if (isRigidDynamic1)
		initJointSpeed = -normalVel1;
	jointSpeedForRestitutionBounce = normalVel0 - normalVel1;
}

/**
\brief Determine if a constraint will have a bounce response and compute the bounce velocity using the restitution parameter.
\param[in] constraintFlags describes whether the constraint is configured to have a bounce response using the restitution parameter.
\param[in] jointSpeedForRestitutionBounce is the velocity of the constraint's body pair projected against the constraint Jacobian.
\param[in] bounceThreshold is the minimum speed that will trigger a bounce response.
\param[in] restitution is the restitution parameter to apply for the bounce response.
\param[in] geometricError is the geometric error of the constraint.
\return The bounce velocity using the restitution parameter if a bounce should take place, 0 if no bounce response should get triggered.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal computeBounceVelocity(const PxU16 constraintFlags, const PxReal jointSpeedForRestitutionBounce, const PxReal bounceThreshold,
	const PxReal restitution, const PxReal geometricError)
{
	PxReal bounceVel = jointSpeedForRestitutionBounce * (-restitution);

	if ((constraintFlags & Px1DConstraintFlag::eRESTITUTION) &&
		(-jointSpeedForRestitutionBounce > bounceThreshold) &&
		((bounceVel * geometricError) <= 0.0f))  // for now, no bounce in case of a speculative CCD scenario, where the geometric error points in
		                                         // the same direction as the bounce velocity (separation, not penetration). If we solved the
		                                         // geometric error in the bounce scenario too, then it would likely work without this check,
		                                         // however, we are not doing that and thus would only generate undesired behavior if bounced.
	{
		return bounceVel;
	}
	else
	{
		return 0.0f;
	}
}

/**
\brief Constraint1dSolverConstantsPGS contains the constant terms used by the PGS 1d constraint solver functions.
For position iterations we have:
newImpulse = oldImpulse*impulseMultiplier + constraintVel*velMultplier + constant.
For velocity iterations we have: 
newImpulse = oldImpulse*impulseMultiplier + constraintVel*velMultplier + unbiasedConstant.
\see compute1dConstraintSolverConstantsPGS()
*/
struct Constraint1dSolverConstantsPGS
{
	PxReal constant;
	PxReal unbiasedConstant;
	PxReal velMultiplier;
	PxReal impulseMultiplier;
};
PX_COMPILE_TIME_ASSERT(sizeof(Constraint1dSolverConstantsPGS) == 16);

/**
\brief Compute the constant terms that will be used by the PGS solver.
\param[in] constraintFlags describes whether the constraint is an acceleration spring, a force spring, a bounce constraint or a hard constraint.
\param[in] springStiffness describes the stiffness of the spring.  This is ignored if Px1DConstraintFlag::eACCELERATION_SPRING is lowered in constraintFlags.
\param[in] springDamping describes the damping of the spring.  This is ignored if Px1DConstraintFlag::eACCELERATION_SPRING is lowered in constraintFlags.
\param[in] restitution describes the bounce restitution of the spring. This is ignored if Px1DConstraintFlag::eRESTITUTION is lowered in constraintFlags.
\param[in] bounceThreshold describes the minimum speed required to trigger a bounce response. This is ignored if Px1DConstraintFlag::eRESTITUTION is lowered in constraintFlags.
\param[in] geometricError is the position error of the constraint.
\param[in] velocityTarget is the target speed of the constraint.
\param[in] jointSpeedForRestitutionBounce is the projection of the velocity of the constraint's body pair against the constraint Jacobian.
\param[in] initJointSpeed is the initial speed of the constraint as experienced by the solver.
\param[in] unitResponse is J * M^-1 * J^T with J the constraint Jacobian and M the constraint Mass matrix.
\param[in] recipUnitResponse is 1/unitResponse with the caveat that very small values of unitResponse have a reciprocal of 0.0. See computeRecipUnitResponse()
\param[in] erp is the Baumgarte multiplier used to scale the geometricError. The constraint will attempt to resolve geometricError*erp. This is ignored for spring or bounce constraints.
\param[in] simDt is the timestep of the scene simulate step.
\param[in] recipSimDt is equal to 1/simDt.
\return A Constraint1dSolverConstantsPGS instance that contains the solver constant terms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE Constraint1dSolverConstantsPGS compute1dConstraintSolverConstantsPGS
(const PxU16 constraintFlags, 
 const PxReal springStiffness, const PxReal springDamping, 
 const PxReal restitution, const PxReal bounceThreshold,
 const PxReal geometricError, const PxReal velocityTarget,
 const PxReal jointSpeedForRestitutionBounce, const PxReal initJointSpeed, 
 const PxReal unitResponse, const PxReal recipUnitResponse, 
 const PxReal erp, 
 const PxReal simDt, const PxReal recipSimDt)
{
	Constraint1dSolverConstantsPGS desc = {0, 0, 0, 0};

	if(constraintFlags & Px1DConstraintFlag::eSPRING)
	{
		const PxReal a = simDt * simDt * springStiffness + simDt * springDamping;
		const PxReal b = simDt * (springDamping * velocityTarget - springStiffness * geometricError);

		if(constraintFlags & Px1DConstraintFlag::eACCELERATION_SPRING)
		{	
			const PxReal x = 1.0f/(1.0f+a);
			const PxReal constant = x * recipUnitResponse * b;
			desc.constant = constant;
			desc.unbiasedConstant = constant;
			desc.velMultiplier = -x * recipUnitResponse * a;
			desc.impulseMultiplier = 1.0f - x;
		}
		else
		{
			const PxReal x = 1.0f/(1.0f+a*unitResponse);
			const PxReal constant = x * b;
			desc.constant = constant;
			desc.unbiasedConstant = constant;
			desc.velMultiplier = -x * a;
			desc.impulseMultiplier = 1.0f - x;
		}
	}
	else
	{
		desc.velMultiplier = -recipUnitResponse;
		desc.impulseMultiplier = 1.0f;

		const PxReal bounceVel = computeBounceVelocity(constraintFlags, jointSpeedForRestitutionBounce, bounceThreshold, restitution, geometricError);
		if (bounceVel != 0.0f)
		{
			const PxReal constant = recipUnitResponse * bounceVel;
			desc.constant = constant;
			desc.unbiasedConstant = constant;
		}
		else
		{
			const PxReal geomError = geometricError * erp;
			desc.constant = recipUnitResponse * (velocityTarget - geomError*recipSimDt);
			desc.unbiasedConstant = (!(constraintFlags & Px1DConstraintFlag::eKEEPBIAS)) ? (recipUnitResponse * velocityTarget) : desc.constant;
		}
	}

	const PxReal velBias = initJointSpeed * desc.velMultiplier;
	desc.constant += velBias;
	desc.unbiasedConstant += velBias;

	return desc;
}

/**
\brief Constraint1dSolverConstantsTGS contains the constant terms used by the TGS 1d constraint solver functions.
For velocity iterations we have:
newImpulse = oldImpulse + constraintVel*velMultplier + (1/unitResponse)*(error + biasScale*deltaGeometricError + targetVel).
For position iterations we have:
newImpulse = oldImpulse + constraintVel*velMultplier + (1/unitResponse)*(targetVel).
*/
struct Constraint1dSolverConstantsTGS
{
	PxReal biasScale;
	PxReal error;
	PxReal velMultiplier;
	PxReal targetVel;
};

/**
\brief Compute the maximum constraint speed arising from geometric error that is allowed during solver iterations.
The bias speed is computed as: biasScale *(geometricError + deltaGeometricError)
\note Spring constraints have unbounded bias speed.  The implicit formulation ensures stability.
\note Bounce constraints are clamped with zero bias speed because they completetely ignore the geometric error and target only the bounce speed.
\note Hard constraints have hard-coded maximum bias speeds based on historical testing.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal computeMaxBiasVelocityTGS
(const PxU16 constraintFlags, 
 const PxReal jointSpeedForRestitutionBounce, const PxReal bounceThreshold, 
 const PxReal restitution, const PxReal geometricError, 
 const bool isExtendedConstraint,
 const PxReal lengthScale, const PxReal recipSimDt)
{
	PxReal maxBiasSpeed = PX_MAX_F32;
	if (constraintFlags & Px1DConstraintFlag::eSPRING)
	{
		maxBiasSpeed = PX_MAX_F32;
	}
	else
	{
		const PxReal bounceVel = computeBounceVelocity(constraintFlags, jointSpeedForRestitutionBounce, bounceThreshold, restitution, geometricError);
		if (bounceVel != 0.0f)
		{
			maxBiasSpeed = 0;
		}
		else if (constraintFlags & Px1DConstraintFlag::eANGULAR_CONSTRAINT)
		{
			maxBiasSpeed = recipSimDt * 0.75f;
		}
		else
		{
			maxBiasSpeed = isExtendedConstraint ? recipSimDt * 1.5f * lengthScale : recipSimDt * 15.f * lengthScale;
		}
	}
	return maxBiasSpeed;
}

/**
\brief Compute the values jointSpeedForRestitutionBounce and initJointSpeed that will be used in compute1dConstraintSolverConstantsTGS().
\param[in] normalVel0 is the projection of the velocity of body 0 projected against the corresponding Jacobian terms.
\param[in] isKinematic0 is true if body 0 is a kinematic, false otherwise.
\param[in] normalVel1 is the projection of the velocity of body 1 projected against the corresponding Jacobian terms.
\param[in] isKinematic1 is true if body 1 is a kinematic, false otherwise.
\param[out] jointSpeedForRestitutionBounce is the projection of the velocity of the constraint's body pair against the constraint Jacobian.
param[out] initJointSpeed is the initial speed of the constraint as experienced by the solver. In the absence of kinematics, this will have value 0.0.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE void computeJointSpeedTGS
(const PxReal normalVel0, const bool isKinematic0, const PxReal normalVel1, const bool isKinematic1,
 PxReal& jointSpeedForRestitutionBounce, PxReal& initJointSpeed)
{
	//
	// The solver treats kinematics more or less like static objects and
	// thus these objects will have a velocity of zero during the solver
	// iterations. This, however, requires that the target velocity gets
	// corrected initially to take this into account. initJointSpeed
	// represents this correction. See computeJointSpeedPGS() for a more
	// general explanation of shifting velocities. Kinematics have a constant
	// velocity that can not be changed by the solver, so it is fine to
	// apply the correction at the beginning and then consider the velocity
	// zero during all solver iterations. Note that unlike PGS, TGS is
	// using the full velocities of the objects during solver iterations
	// and only kinematics are the exception. Hence, the difference to
	// computeJointSpeedPGS().
	//

	initJointSpeed = 0.f;
	if (isKinematic0)
		initJointSpeed -= normalVel0;
	if (isKinematic1)
		initJointSpeed += normalVel1;
	jointSpeedForRestitutionBounce = normalVel0 - normalVel1;
}

/**
\brief Compute the constant terms that will be used by the TGS solver.
\param[in] constraintFlags describes whether the constraint is an acceleration spring, a force spring, a bounce constraint or a hard constraint.
\param[in] springStiffness describes the stiffness of the spring.  This is ignored if Px1DConstraintFlag::eACCELERATION_SPRING is lowered in constraintFlags.
\param[in] springDamping describes the damping of the spring.  This is ignored if Px1DConstraintFlag::eACCELERATION_SPRING is lowered in constraintFlags.
\param[in] restitution describes the bounce restitution of the spring. This is ignored if Px1DConstraintFlag::eRESTITUTION is lowered in constraintFlags.
\param[in] bounceThreshold describes the minimum speed required to trigger a bounce response. This is ignored if Px1DConstraintFlag::eRESTITUTION is lowered in constraintFlags.
\param[in] geometricError is the position error of the constraint.
\param[in] velocityTarget is the target speed of the constraint.
\param[in] jointSpeedForRestitutionBounce is the projection of the velocity of the constraint's body pair against the constraint Jacobian.
\param[in] initJointSpeed is the initial speed of the constraint as experienced by the solver. In the absence of kinematics, this has value 0.0.
\param[in] unitResponse is J * M^-1 * J^T with J the constraint Jacobian and M the constraint Mass matrix.
\param[in] recipUnitResponse is 1/unitResponse with the caveat that very small values of unitResponse have a reciprocal of 0.0. See computeRecipUnitResponse()
\param[in] erp is the Baumgarte multiplier used to scale the geometricError. The constraint will attempt to resolve geometricError*erp. This is ignored for spring or bounce constraints.
\param[in] stepDt is the timestep of each TGS position iteration step.
\param[in] recipStepDt is equal to 1/stepDt.
\return A Constraint1dSolverConstantsTGS instance that contains the solver constant terms.
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE Constraint1dSolverConstantsTGS compute1dConstraintSolverConstantsTGS
(const PxU16 constraintFlags, 
 const PxReal springStiffness, const PxReal springDamping, 
 const PxReal restitution, const PxReal bounceThreshold,
 const PxReal geometricError, const PxReal velocityTarget,
 const PxReal jointSpeedForRestitutionBounce, const PxReal initJointSpeed,
 const PxReal unitResponse, const PxReal recipUnitResponse,
 const PxReal erp, 
 const PxReal stepDt, const PxReal recipStepDt)
{
	Constraint1dSolverConstantsTGS desc = {0.0f, 0.0f, 0.0f, 0.0f};

	if (constraintFlags & Px1DConstraintFlag::eSPRING)
	{
		const PxReal a = stepDt * (stepDt * springStiffness + springDamping);
		const PxReal b = stepDt * (springDamping * velocityTarget);

		if (constraintFlags & Px1DConstraintFlag::eACCELERATION_SPRING)
		{
			const PxReal x = 1.0f / (1.0f + a);
			const PxReal biasScale = -x * springStiffness * stepDt * recipUnitResponse;
			const PxReal velMultiplier =  -x * a * recipUnitResponse;

			desc.biasScale = biasScale;
			desc.error = biasScale * geometricError;
			desc.velMultiplier = velMultiplier;
			desc.targetVel = x * b * recipUnitResponse - velMultiplier * initJointSpeed;
		}
		else
		{
			const PxReal x = 1.0f / (1.0f + a*unitResponse);
			const PxReal biasScale = -x * springStiffness * stepDt;		
			const PxReal velMultiplier =  -x * a;	

			desc.biasScale = biasScale;
			desc.error = biasScale * geometricError;
			desc.velMultiplier = velMultiplier;	
			desc.targetVel = x * b - velMultiplier * initJointSpeed;
		}
	}
	else
	{
		const PxReal bounceVel = computeBounceVelocity(constraintFlags, jointSpeedForRestitutionBounce, bounceThreshold, restitution, geometricError);

		if (bounceVel != 0.0f)
		{
			const PxReal velMultiplier = -1.0f;

			desc.biasScale = 0.f;
			desc.error = 0.f;
			desc.velMultiplier = velMultiplier;
			desc.targetVel = bounceVel - velMultiplier * initJointSpeed;
		}
		else
		{
			const PxReal velMultiplier = -1.0f;
			const PxReal biasScale = -recipStepDt*erp;

			desc.biasScale = biasScale;
			desc.error = geometricError*biasScale;
			desc.velMultiplier = velMultiplier;
			desc.targetVel = velocityTarget - velMultiplier * initJointSpeed;
		}
	}

	return desc;
}

/**
\brief Translate external flags and hints to internal flags.
\param[in] externalFlags Parameter that holds Px1DConstraintFlag::Type flags to translate.
\param[in] externalSolveHint Parameter that holds the solve hint information to translate.
\param[out] internalFlags Location to apply the internal flags to.
*/
template<typename T, typename U>
PX_CUDA_CALLABLE PX_FORCE_INLINE void raiseInternalFlagsTGS(T externalFlags, T externalSolveHint, U& internalFlags)
{
	if (externalFlags & Px1DConstraintFlag::eSPRING)
		internalFlags |= DY_SC_FLAG_SPRING;

	if (externalFlags & Px1DConstraintFlag::eOUTPUT_FORCE)
		internalFlags |= DY_SC_FLAG_OUTPUT_FORCE;

	if (externalFlags & Px1DConstraintFlag::eKEEPBIAS)
		internalFlags |= DY_SC_FLAG_KEEP_BIAS;

	if (externalSolveHint & 1)
		internalFlags |= DY_SC_FLAG_INEQUALITY;
}

/**
\brief Compute the amount of the geometric error that has been resolved.

A hard constraint tries to solve the equation:

(vT - vJ) + geomError/simDt = 0

With target velocity vT and vJ = jacobian * velocity

To satisfy the equation, the expected velocity is:

vJ = vT + geomError/simDt

The expected total motion p_n, thus is:

p_n = vJ * simDt = (vT*simDt) + geomError

During position iterations, the transforms get integrated and as such
part of the geometric error gets resolved. As a consequence, at each
iteration the amount of the resolved geometric error has to be evaluated
to know how much error remains. Assuming the expected velocity was
reached at the first iteration, the expected accumulated relative motion
at iteration i (starting at 0) is:

p_i = vJ * (i*stepDt)
    = vT * (i*stepDt)  +  geomError * (i*stepDt)/simDt
	= vT * (i*stepDt)  +  geomError * (i/posIterationCount)

With stepDt being the time step of a TGS position iteration, i.e.,
stepDt = simDt/posIterationCount

Note that we are currently using the following definition instead:

p_i = vT * (i*stepDt)  +  geomError

Splitting this into two components

pvt_i = vT * (i*stepDt) = vT * elapsedTime_i
pge_i = geomError

We get:

p_i = pvt_i  +  pge_i

For various reasons, the solver might not reach the expected velocity
immediately and as such the actual amount of motion (motion_i) can differ
from the expected motion (p_i).

motion_i = pvt_i  +  resolvedGeomError_i
resolvedGeomError_i = motion_i  -  pvt_i

This then allows to compute the remaining error as:

remainingGeomError_i = pge_i+1 - resolvedGeomError_i

For a soft constraint, the approach described above does not apply since
the formulation of a soft constraints represents a spring force/impulse
that should get applied during the sim step.

F = stiffness * -geomError + damping * (vT - vJ)

Unlike hard constraints, there is no clear goal to resolve the geometric
error within a sim step. A spring without damping just keeps oscillating.
Thus, there is no point in trying to distinguish what part of the motion
resulted from resolving target velocity vs. resolving geometric error.
Soft constraints should just take the current geometric error into account
without bringing the target velocity into the mix.

\param[in] deltaLin0 The accumulated translational change of the constraint anchor point of body 0 since the beginning of the solver.
\param[in] deltaLin1 See deltaLin0 but for body 1.
\param[in] cLinVel0 The constraint axis (jacobian) for the linear velocity of body 0.
\param[in] cLinVel1 See cLinVel0 but for body 1.
\param[in] deltaAngInertia0 The accumulated rotational change of the constraint anchor point of body 0 since the beginning of the solver.
                            Note: if momocity is used, this is not a pure delta angle but rather: Inertia^(1/2)*deltaAngle
\param[in] deltaAngInertia1 See deltaAngInertia0 but for body 1.
\param[in] cAngVelInertia0 The constraint axis (jacobian) for the angular velocity of body 0.
                           Note: if momocity is used, this is not the pure constraint axis but rather: Inertia1^(-1/2)*cAngVel0
\param[in] cAngVelInertia1 See cAngVelInertia0 but for body 1.
\param[in] angularErrorScale Multiplier for the accumulated relative anchor motion from angular velocity.
                             Can be set to 0 to ignore the angular part, for example.
\param[in] flags The internal constraint flags (see SolverConstraintFlags).
\param[in] springFlagMask The value DY_SC_FLAG_SPRING (as VecU32V for SIMD version, see U4Load(DY_SC_FLAG_SPRING))
\param[in] targetVel The target velocity for the constraint.
\param[in] elapsedTime The elapsed time since start of the solver (stepDt accumulated over solver iterations).
\return The resolved geometric error.
*/
#if defined(__CUDACC__)
__device__ PX_FORCE_INLINE PxReal computeResolvedGeometricErrorTGS(const PxVec3& deltaLin0, const PxVec3& deltaLin1,
	const PxVec3& cLinVel0, const PxVec3& cLinVel1,
	const PxVec3& deltaAngInertia0, const PxVec3& deltaAngInertia1,
	const PxVec3& cAngVelInertia0, const PxVec3& cAngVelInertia1,
	const PxReal angularErrorScale,
	const bool isSpringConstraint, const PxReal targetVel, const PxReal elapsedTime)
{
	const PxReal deltaAng = (cAngVelInertia0.dot(deltaAngInertia0) - cAngVelInertia1.dot(deltaAngInertia1)) * angularErrorScale;
	const PxReal deltaLin = (cLinVel0.dot(deltaLin0) - cLinVel1.dot(deltaLin1));

	const PxReal motion = deltaLin + deltaAng;

	if (isSpringConstraint)
	{
		return motion;
	}
	else
	{
		const PxReal resolvedError = motion - (targetVel * elapsedTime);
		return resolvedError;
	}
}
#else
PX_FORCE_INLINE aos::FloatV computeResolvedGeometricErrorTGS(const aos::Vec3VArg deltaLin0, const aos::Vec3VArg deltaLin1,
	const aos::Vec3VArg cLinVel0, const aos::Vec3VArg cLinVel1,
	const aos::Vec3VArg deltaAngInertia0, const aos::Vec3VArg deltaAngInertia1,
	const aos::Vec3VArg cAngVelInertia0, const aos::Vec3VArg cAngVelInertia1,
	const aos::FloatVArg angularErrorScale,
	const aos::BoolVArg isSpringConstraint, const aos::FloatVArg targetVel, const aos::FloatVArg elapsedTime)
{
	// motion = cLinVel0.dot(deltaLin0)  +  cAngVelInertia0.dot(deltaAngInertia0)
	//        - cLinVel1.dot(deltaLin1)  -  cAngVelInertia1.dot(deltaAngInertia1)
	//
	//        = cLinVel0.dot(deltaLin0)  +  [Inertia0^(-1/2)*cAngVel0].dot[Inertia0^(1/2)*deltaAng0]
	//        - cLinVel1.dot(deltaLin1)  -  [Inertia1^(-1/2)*cAngVel1].dot[Inertia1^(1/2)*deltaAng1]
	//
	//        = cLinVel0.dot(deltaLin0)  +  cAngVel0.dot(deltaAng0)
	//        - cLinVel1.dot(deltaLin1)  -  cAngVel1.dot(deltaAng1)

	const aos::FloatV deltaAng = aos::FMul(angularErrorScale, aos::FSub(aos::V3Dot(cAngVelInertia0, deltaAngInertia0), aos::V3Dot(cAngVelInertia1, deltaAngInertia1)));
	const aos::FloatV deltaLin = aos::FSub(aos::V3Dot(cLinVel0, deltaLin0), aos::V3Dot(cLinVel1, deltaLin1));

	const aos::FloatV motion = aos::FAdd(deltaLin, deltaAng);

	const aos::FloatV resolvedError = aos::FSel(isSpringConstraint, motion, aos::FNegScaleSub(targetVel, elapsedTime, motion));
	return resolvedError;
}

PX_FORCE_INLINE aos::Vec4V computeResolvedGeometricErrorTGSBlock(
	const aos::Vec4VArg deltaLin0X, const aos::Vec4VArg deltaLin0Y, const aos::Vec4VArg deltaLin0Z,
	const aos::Vec4VArg deltaLin1X, const aos::Vec4VArg deltaLin1Y, const aos::Vec4VArg deltaLin1Z,
	const aos::Vec4VArg cLinVel0X, const aos::Vec4VArg cLinVel0Y, const aos::Vec4VArg cLinVel0Z,
	const aos::Vec4VArg cLinVel1X, const aos::Vec4VArg cLinVel1Y, const aos::Vec4VArg cLinVel1Z,
	const aos::Vec4VArg deltaAngInertia0X, const aos::Vec4VArg deltaAngInertia0Y, const aos::Vec4VArg deltaAngInertia0Z,
	const aos::Vec4VArg deltaAngInertia1X, const aos::Vec4VArg deltaAngInertia1Y, const aos::Vec4VArg deltaAngInertia1Z,
	const aos::Vec4VArg cAngVelInertia0X, const aos::Vec4VArg cAngVelInertia0Y, const aos::Vec4VArg cAngVelInertia0Z,
	const aos::Vec4VArg cAngVelInertia1X, const aos::Vec4VArg cAngVelInertia1Y, const aos::Vec4VArg cAngVelInertia1Z,
	const aos::Vec4VArg angularErrorScale,
	const aos::BoolVArg isSpringConstraint, const aos::Vec4VArg targetVel, const aos::FloatVArg elapsedTime)
{
	const aos::Vec4V deltaAng0 = aos::V4MulAdd(cAngVelInertia0X, deltaAngInertia0X, aos::V4MulAdd(cAngVelInertia0Y, deltaAngInertia0Y, aos::V4Mul(cAngVelInertia0Z, deltaAngInertia0Z)));
	const aos::Vec4V deltaAng1 = aos::V4MulAdd(cAngVelInertia1X, deltaAngInertia1X, aos::V4MulAdd(cAngVelInertia1Y, deltaAngInertia1Y, aos::V4Mul(cAngVelInertia1Z, deltaAngInertia1Z)));
	const aos::Vec4V deltaAng = aos::V4Mul(angularErrorScale, aos::V4Sub(deltaAng0, deltaAng1));

	const aos::Vec4V deltaLin0 = aos::V4MulAdd(cLinVel0X, deltaLin0X, aos::V4MulAdd(cLinVel0Y, deltaLin0Y, aos::V4Mul(cLinVel0Z, deltaLin0Z)));
	const aos::Vec4V deltaLin1 = aos::V4MulAdd(cLinVel1X, deltaLin1X, aos::V4MulAdd(cLinVel1Y, deltaLin1Y, aos::V4Mul(cLinVel1Z, deltaLin1Z)));
	const aos::Vec4V deltaLin = aos::V4Sub(deltaLin0, deltaLin1);

	const aos::Vec4V motion = aos::V4Add(deltaLin, deltaAng);

	const aos::Vec4V resolvedError = aos::V4Sel(isSpringConstraint, motion, aos::V4NegScaleSub(targetVel, elapsedTime, motion));
	return resolvedError;
}
#endif

/*
\brief Compute the minimal bias given internal 1D constraint flags.
 
\param[in] flags The internal 1D constraint flags (see SolverConstraintFlags).
\param[in] maxBias The maximum bias to use. This has the form:
                   -(geometricError / dt) * erp
                   (erp = geom error based impulse multiplier, i.e, Baumgarte term)

\return The minimum bias.
*/
#if defined(__CUDACC__)
__device__ PX_FORCE_INLINE PxReal computeMinBiasTGS(PxU32 flags, PxReal maxBias)
{
	const PxReal minBias = -((flags & DY_SC_FLAG_INEQUALITY) ? PX_MAX_F32 : maxBias);
	return minBias;
}
#else
PX_FORCE_INLINE aos::FloatV computeMinBiasTGS(PxU32 flags, const aos::FloatVArg maxBias)
{
	const aos::FloatV minBias = aos::FNeg((flags & DY_SC_FLAG_INEQUALITY) ? aos::FMax() : maxBias);
	return minBias;
}

/*
\brief Compute the minimal bias given internal 1D constraint flags.

This version is for processing blocks of 4 constraints.
 
\param[in] flags See non-block version (but for a set of 4 constraints).
\param[in] inequalityFlagMask DY_SC_FLAG_INEQUALITY loaded into an integer SIMD
                              vector (U4Load(DY_SC_FLAG_INEQUALITY)).
\param[in] maxBias See non-block version (but for a set of 4 constraints).

\return The minimum bias (for a set of 4 constraints).
*/
PX_FORCE_INLINE aos::Vec4V computeMinBiasTGSBlock(const aos::VecU32VArg flags, const aos::VecU32VArg inequalityFlagMask, const aos::Vec4VArg maxBias)
{
	const aos::BoolV isInequalityConstraint = aos::V4IsEqU32(aos::V4U32and(flags, inequalityFlagMask), inequalityFlagMask);
	const aos::Vec4V minBias = aos::V4Neg(aos::V4Sel(isInequalityConstraint, aos::Vec4V_From_FloatV(aos::FMax()), maxBias));
	return minBias;
}
#endif


} //namespace Dy
} //namespace physx
#endif //DY_1DCONSTRAINT_CPUGPU_H

