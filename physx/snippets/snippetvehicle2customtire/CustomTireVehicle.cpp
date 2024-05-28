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

#include "CustomTireVehicle.h"


namespace snippetvehicle2
{

template<typename TFloat>
static void setExampleSharedParams(MFTireSharedParams<TFloat>& sharedParams, const TFloat lengthScale)
{
	sharedParams.r0 = TFloat(0.3135) * lengthScale;  // tire radius under no load (base unit is [m])
	sharedParams.v0 = TFloat(16.7) * lengthScale;  // reference velocity (=60km/h) (base unit is [m/s])
	sharedParams.fz0 = TFloat(4000.0) * lengthScale;  // nominal load (base unit is [N])
	sharedParams.pi0 = TFloat(220000.0) / lengthScale;  // nominal inflation pressure (=2.2 bar) (base unit is [Pa])

	sharedParams.epsilonV = TFloat(0.01) * lengthScale;
	sharedParams.slipReductionLowVelocity = TFloat(7.0) * lengthScale;
	sharedParams.lambdaFz0 = TFloat(1.0);
	sharedParams.lambdaK_alpha = TFloat(1.0);
	sharedParams.aMu = TFloat(10.0);
	sharedParams.zeta0 = TFloat(1.0);
	sharedParams.zeta2 = TFloat(1.0);

	mfTireComputeDerivedSharedParams(sharedParams);
}

template<typename TFloat>
static void setExampleLongitudinalForcePureParams(MFTireLongitudinalForcePureParams<TFloat>& longitudinalForcePureParams, const TFloat lengthScale)
{
	longitudinalForcePureParams.pK1 = TFloat(21.687);
	longitudinalForcePureParams.pK2 = TFloat(13.728);
	longitudinalForcePureParams.pK3 = TFloat(-0.4098);
	longitudinalForcePureParams.pp1 = TFloat(-0.3485);
	longitudinalForcePureParams.pp2 = TFloat(0.37824);
	longitudinalForcePureParams.lambdaK = TFloat(1.0);
	longitudinalForcePureParams.epsilon = TFloat(1.0) * lengthScale;
	//---
	longitudinalForcePureParams.pC1 = TFloat(1.579);
	longitudinalForcePureParams.lambdaC = TFloat(1.0);
	//---
	longitudinalForcePureParams.pD1 = TFloat(1.0422);
	longitudinalForcePureParams.pD2 = TFloat(-0.08285);
	longitudinalForcePureParams.pD3 = TFloat(0.0);
	longitudinalForcePureParams.pp3 = TFloat(-0.09603);
	longitudinalForcePureParams.pp4 = TFloat(0.06518);
	//---
	longitudinalForcePureParams.pE1 = TFloat(0.11113);
	longitudinalForcePureParams.pE2 = TFloat(0.3143);
	longitudinalForcePureParams.pE3 = TFloat(0.0);
	longitudinalForcePureParams.pE4 = TFloat(0.001719);
	longitudinalForcePureParams.lambdaE = TFloat(1.0);
	//---
	longitudinalForcePureParams.pH1 = TFloat(2.1615e-4);
	longitudinalForcePureParams.pH2 = TFloat(0.0011598);
	longitudinalForcePureParams.lambdaH = TFloat(1.0);
	//---
	longitudinalForcePureParams.pV1 = TFloat(2.0283e-5);
	longitudinalForcePureParams.pV2 = TFloat(1.0568e-4);
	longitudinalForcePureParams.lambdaV = TFloat(1.0);
	longitudinalForcePureParams.zeta1 = TFloat(1.0);
}

template<typename TFloat>
static void setExampleLongitudinalForceCombinedParams(MFTireLongitudinalForceCombinedParams<TFloat>& longitudinalForceCombinedParams)
{
	longitudinalForceCombinedParams.rB1 = TFloat(13.046);
	longitudinalForceCombinedParams.rB2 = TFloat(9.718);
	longitudinalForceCombinedParams.rB3 = TFloat(0.0);
	longitudinalForceCombinedParams.lambdaAlpha = TFloat(1.0);
	//---
	longitudinalForceCombinedParams.rC1 = TFloat(0.9995);
	//---
	longitudinalForceCombinedParams.rE1 = TFloat(-0.4403);
	longitudinalForceCombinedParams.rE2 = TFloat(-0.4663);
	//---
	longitudinalForceCombinedParams.rH1 = TFloat(-9.968e-5);
}

template<typename TFloat>
static void setExampleLateralForcePureParams(MFTireLateralForcePureParams<TFloat>& lateralForcePureParams, const TFloat lengthScale)
{
	lateralForcePureParams.pK1 = TFloat(15.324);  // note: original source uses ISO sign convention and thus the value was negative
	lateralForcePureParams.pK2 = TFloat(1.715);
	lateralForcePureParams.pK3 = TFloat(0.3695);
	lateralForcePureParams.pK4 = TFloat(2.0005);
	lateralForcePureParams.pK5 = TFloat(0.0);
	lateralForcePureParams.pp1 = TFloat(-0.6255);
	lateralForcePureParams.pp2 = TFloat(-0.06523);
	lateralForcePureParams.epsilon = TFloat(1.0) * lengthScale;
	lateralForcePureParams.zeta3 = TFloat(1.0);
	//---
	lateralForcePureParams.pC1 = TFloat(1.338);
	lateralForcePureParams.lambdaC = TFloat(1.0);
	//---
	lateralForcePureParams.pD1 = TFloat(0.8785);
	lateralForcePureParams.pD2 = TFloat(-0.06452);
	lateralForcePureParams.pD3 = TFloat(0.0);
	lateralForcePureParams.pp3 = TFloat(-0.16666);
	lateralForcePureParams.pp4 = TFloat(0.2811);
	//---
	lateralForcePureParams.pE1 = TFloat(-0.8057);
	lateralForcePureParams.pE2 = TFloat(-0.6046);
	lateralForcePureParams.pE3 = TFloat(0.09854);
	lateralForcePureParams.pE4 = TFloat(-6.697);
	lateralForcePureParams.pE5 = TFloat(0.0);
	lateralForcePureParams.lambdaE = TFloat(1.0);
	//---
	lateralForcePureParams.pH1 = TFloat(-0.001806);
	lateralForcePureParams.pH2 = TFloat(0.00352);
	lateralForcePureParams.pK6 = TFloat(-0.8987);
	lateralForcePureParams.pK7 = TFloat(-0.23303);
	lateralForcePureParams.pp5 = TFloat(0.0);
	lateralForcePureParams.lambdaH = TFloat(1.0);
	lateralForcePureParams.epsilonK = TFloat(10.0) * lengthScale;
	lateralForcePureParams.zeta4 = TFloat(1.0);
	//---
	lateralForcePureParams.pV1 = TFloat(-0.00661);
	lateralForcePureParams.pV2 = TFloat(0.03592);
	lateralForcePureParams.pV3 = TFloat(-0.162);
	lateralForcePureParams.pV4 = TFloat(-0.4864);
	lateralForcePureParams.lambdaV = TFloat(1.0);
	lateralForcePureParams.lambdaK_gamma = TFloat(1.0);
}

template<typename TFloat>
static void setExampleLateralForceCombinedParams(MFTireLateralForceCombinedParams<TFloat>& lateralForceCombinedParams)
{
	lateralForceCombinedParams.rB1 = TFloat(10.622);
	lateralForceCombinedParams.rB2 = TFloat(7.82);
	lateralForceCombinedParams.rB3 = TFloat(0.002037);
	lateralForceCombinedParams.rB4 = TFloat(0.0);
	lateralForceCombinedParams.lambdaKappa = TFloat(1.0);
	//---
	lateralForceCombinedParams.rC1 = TFloat(1.0587);
	//--
	lateralForceCombinedParams.rE1 = TFloat(0.3148);
	lateralForceCombinedParams.rE2 = TFloat(0.004867);
	//---
	lateralForceCombinedParams.rH1 = TFloat(0.009472);
	lateralForceCombinedParams.rH2 = TFloat(0.009754);
	//---
	lateralForceCombinedParams.rV1 = TFloat(0.05187);
	lateralForceCombinedParams.rV2 = TFloat(4.853e-4);
	lateralForceCombinedParams.rV3 = TFloat(0.0);
	lateralForceCombinedParams.rV4 = TFloat(94.63);
	lateralForceCombinedParams.rV5 = TFloat(1.8914);
	lateralForceCombinedParams.rV6 = TFloat(23.8);
	lateralForceCombinedParams.lambdaV = TFloat(1.0);
}

template<typename TFloat>
static void setExampleAligningTorquePurePneumaticTrailParams(MFTireAligningTorquePurePneumaticTrailParams<TFloat>& aligningTorquePurePneumaticTrailParams)
{
	aligningTorquePurePneumaticTrailParams.qB1 = TFloat(12.035);
	aligningTorquePurePneumaticTrailParams.qB2 = TFloat(-1.33);
	aligningTorquePurePneumaticTrailParams.qB3 = TFloat(0.0);
	aligningTorquePurePneumaticTrailParams.qB5 = TFloat(-0.14853);
	aligningTorquePurePneumaticTrailParams.qB6 = TFloat(0.0);
	//---
	aligningTorquePurePneumaticTrailParams.qC1 = TFloat(1.2923);
	//---
	aligningTorquePurePneumaticTrailParams.qD1 = TFloat(0.09068);
	aligningTorquePurePneumaticTrailParams.qD2 = TFloat(-0.00565);
	aligningTorquePurePneumaticTrailParams.qD3 = TFloat(0.3778);
	aligningTorquePurePneumaticTrailParams.qD4 = TFloat(0.0);
	aligningTorquePurePneumaticTrailParams.pp1 = TFloat(-0.4408);
	aligningTorquePurePneumaticTrailParams.lambdaT = TFloat(1.0);
	aligningTorquePurePneumaticTrailParams.zeta5 = TFloat(1.0);
	//---
	aligningTorquePurePneumaticTrailParams.qE1 = TFloat(-1.7924);
	aligningTorquePurePneumaticTrailParams.qE2 = TFloat(0.8975);
	aligningTorquePurePneumaticTrailParams.qE3 = TFloat(0.0);
	aligningTorquePurePneumaticTrailParams.qE4 = TFloat(0.2895);
	aligningTorquePurePneumaticTrailParams.qE5 = TFloat(-0.6786);
	//---
	aligningTorquePurePneumaticTrailParams.qH1 = TFloat(0.0014333);
	aligningTorquePurePneumaticTrailParams.qH2 = TFloat(0.0024087);
	aligningTorquePurePneumaticTrailParams.qH3 = TFloat(0.24973);
	aligningTorquePurePneumaticTrailParams.qH4 = TFloat(-0.21205);
}

template<typename TFloat>
static void setExampleAligningTorquePureResidualTorqueParams(MFTireAligningTorquePureResidualTorqueParams<TFloat>& aligningTorquePureResidualTorqueParams)
{
	aligningTorquePureResidualTorqueParams.qB9 = TFloat(34.5);
	aligningTorquePureResidualTorqueParams.qB10 = TFloat(0.0);
	aligningTorquePureResidualTorqueParams.zeta6 = TFloat(1.0);
	//---
	aligningTorquePureResidualTorqueParams.zeta7 = TFloat(1.0);
	//---
	aligningTorquePureResidualTorqueParams.qD6 = TFloat(0.0017015);
	aligningTorquePureResidualTorqueParams.qD7 = TFloat(-0.002091);
	aligningTorquePureResidualTorqueParams.qD8 = TFloat(-0.1428);
	aligningTorquePureResidualTorqueParams.qD9 = TFloat(0.00915);
	aligningTorquePureResidualTorqueParams.qD10 = TFloat(0.0);
	aligningTorquePureResidualTorqueParams.qD11 = TFloat(0.0);
	aligningTorquePureResidualTorqueParams.pp2 = TFloat(0.0);
	aligningTorquePureResidualTorqueParams.lambdaR = TFloat(1.0);
	aligningTorquePureResidualTorqueParams.lambdaK_gamma = TFloat(1.0);
	aligningTorquePureResidualTorqueParams.zeta8 = TFloat(1.0);
}

template<typename TFloat>
static void setExampleAligningTorqueCombinedParams(MFTireAligningTorqueCombinedParams<TFloat>& aligningTorqueCombinedParams)
{
	aligningTorqueCombinedParams.sS1 = TFloat(0.00918);
	aligningTorqueCombinedParams.sS2 = TFloat(0.03869);
	aligningTorqueCombinedParams.sS3 = TFloat(0.0);
	aligningTorqueCombinedParams.sS4 = TFloat(0.0);
	aligningTorqueCombinedParams.lambdaS = TFloat(1.0);
}

//For completeness but not used in this example
/*template<typename TFloat>
static void setExampleOverturningCoupleParams(MFTireOverturningCoupleParams<TFloat>& overturningCoupleParams)
{
	overturningCoupleParams.qS1 = TFloat(-0.007764);
	overturningCoupleParams.qS2 = TFloat(1.1915);
	overturningCoupleParams.qS3 = TFloat(0.013948);
	overturningCoupleParams.qS4 = TFloat(4.912);
	overturningCoupleParams.qS5 = TFloat(1.02);
	overturningCoupleParams.qS6 = TFloat(22.83);
	overturningCoupleParams.qS7 = TFloat(0.7104);
	overturningCoupleParams.qS8 = TFloat(-0.023393);
	overturningCoupleParams.qS9 = TFloat(0.6581);
	overturningCoupleParams.qS10 = TFloat(0.2824);
	overturningCoupleParams.qS11 = TFloat(5.349);
	overturningCoupleParams.ppM1 = TFloat(0.0);
	overturningCoupleParams.lambdaVM = TFloat(1.0);
	overturningCoupleParams.lambdaM = TFloat(1.0);
}

template<typename TFloat>
static void setExampleRollingResistanceMomentParams(MFTireRollingResistanceMomentParams<TFloat>& rollingResistanceMomentParams)
{
	rollingResistanceMomentParams.qS1 = TFloat(0.00702);
	rollingResistanceMomentParams.qS2 = TFloat(0.0);
	rollingResistanceMomentParams.qS3 = TFloat(0.001515);
	rollingResistanceMomentParams.qS4 = TFloat(8.514e-5);
	rollingResistanceMomentParams.qS5 = TFloat(0.0);
	rollingResistanceMomentParams.qS6 = TFloat(0.0);
	rollingResistanceMomentParams.qS7 = TFloat(0.9008);
	rollingResistanceMomentParams.qS8 = TFloat(-0.4089);
	rollingResistanceMomentParams.lambdaM = TFloat(1.0);
}*/

template<typename TFloat>
static void setExampleFreeRotatingRadiusParams(MFTireFreeRotatingRadiusParams<TFloat>& freeRotatingRadiusParams)
{
	freeRotatingRadiusParams.qre0 = TFloat(0.9974);
	freeRotatingRadiusParams.qV1 = TFloat(7.742e-4);
}

template<typename TFloat>
static void setExampleVerticalStiffnessParams(const TFloat r0, const TFloat fz0,
	MFTireVerticalStiffnessParams<TFloat>& verticalStiffnessParams)
{
	verticalStiffnessParams.qF1 = TFloat(14.435747);
	verticalStiffnessParams.qF2 = TFloat(15.4);
	verticalStiffnessParams.ppF1 = TFloat(0.7098);
	verticalStiffnessParams.lambdaC = TFloat(1.0);

	mfTireComputeDerivedVerticalTireStiffnessParams(r0, fz0, 
		verticalStiffnessParams);
}

template<typename TFloat>
static void setExampleNormalLoadParams(MFTireNormalLoadParams<TFloat>& normalLoadParams)
{
	normalLoadParams.qV2 = TFloat(0.04667);
	normalLoadParams.qFc_longitudinal = TFloat(0.0);
	normalLoadParams.qFc_lateral = TFloat(0.0);
	normalLoadParams.qF3 = TFloat(0.0);
}

template<typename TFloat>
static void setExampleEffectiveRollingRadiusParams(MFTireEffectiveRollingRadiusParams<TFloat>& effectiveRollingRadiusParams)
{
	effectiveRollingRadiusParams.Freff = TFloat(0.07394);
	effectiveRollingRadiusParams.Dreff = TFloat(0.25826);
	effectiveRollingRadiusParams.Breff = TFloat(8.386);
}

template<typename TFloat>
static void setExampleTireData(const TFloat lengthScale,
	const bool computeAligningMoment, MFTireDataT<TFloat>& tireData)
{
	setExampleSharedParams(tireData.sharedParams, lengthScale);
	setExampleFreeRotatingRadiusParams(tireData.freeRotatingRadiusParams);
	setExampleVerticalStiffnessParams(tireData.sharedParams.r0, tireData.sharedParams.fz0, 
		tireData.verticalStiffnessParams);
	setExampleEffectiveRollingRadiusParams(tireData.effectiveRollingRadiusParams);
	setExampleNormalLoadParams(tireData.normalLoadParams);
	setExampleLongitudinalForcePureParams(tireData.longitudinalForcePureParams, lengthScale);
	setExampleLongitudinalForceCombinedParams(tireData.longitudinalForceCombinedParams);
	setExampleLateralForcePureParams(tireData.lateralForcePureParams, lengthScale);
	setExampleLateralForceCombinedParams(tireData.lateralForceCombinedParams);
	setExampleAligningTorquePurePneumaticTrailParams(tireData.aligningTorquePurePneumaticTrailParams);
	setExampleAligningTorquePureResidualTorqueParams(tireData.aligningTorquePureResidualTorqueParams);
	setExampleAligningTorqueCombinedParams(tireData.aligningTorqueCombinedParams);

	if (computeAligningMoment)
		tireData.flags |= MFTireDataFlag::eALIGNING_MOMENT;
	else
		tireData.flags.clear(MFTireDataFlag::eALIGNING_MOMENT);
}

//Adjust those parameters that describe lateral asymmetry and as such lead more easily to drift.
//Can be useful when using the same dataset for all wheels.
template<typename TFloat>
static void makeTireSymmetric(MFTireDataT<TFloat>& tireData)
{
	tireData.longitudinalForceCombinedParams.rH1 = TFloat(0.0);

    tireData.lateralForcePureParams.pE3 = TFloat(0.0);
    tireData.lateralForcePureParams.pH1 = TFloat(0.0);
    tireData.lateralForcePureParams.pH2 = TFloat(0.0);
    tireData.lateralForcePureParams.pV1 = TFloat(0.0);
    tireData.lateralForcePureParams.pV2 = TFloat(0.0);

    tireData.lateralForceCombinedParams.rB3 = TFloat(0.0);
    tireData.lateralForceCombinedParams.rV1 = TFloat(0.0);
    tireData.lateralForceCombinedParams.rV2 = TFloat(0.0);

    tireData.aligningTorquePureResidualTorqueParams.qD6 = TFloat(0.0);
    tireData.aligningTorquePureResidualTorqueParams.qD7 = TFloat(0.0);

    tireData.aligningTorquePurePneumaticTrailParams.qD3 = TFloat(0.0);
    tireData.aligningTorquePurePneumaticTrailParams.qE4 = TFloat(0.0);
    tireData.aligningTorquePurePneumaticTrailParams.qH1 = TFloat(0.0);
    tireData.aligningTorquePurePneumaticTrailParams.qH2 = TFloat(0.0);

    tireData.aligningTorqueCombinedParams.sS1 = TFloat(0.0);
}

bool CustomTireVehicle::initialize(PxPhysics& physics, const PxCookingParams& cookingParams, PxMaterial& defaultMaterial, 
	bool addPhysXBeginEndComponents)
{
	typedef MFTireConfig::Float TFloat;

	if (!DirectDriveVehicle::initialize(physics, cookingParams, defaultMaterial, addPhysXBeginEndComponents))
		return false;

	//Set the custom parameters for the vehicle tires.
	const PxTolerancesScale& tolerancesScale = physics.getTolerancesScale();
	const bool computeAligningMoment = false;  //Not used in this snippet

	const PxU32 tireDataParameterSetCount = sizeof(mCustomTireParams) / sizeof(mCustomTireParams[0]);
	for (PxU32 i = 0; i < tireDataParameterSetCount; i++)
	{
		//Note: in this example, the same parameter values are used for all wheels.

		CustomTireParams& customTireParams = mCustomTireParams[i];

		setExampleTireData<TFloat>(tolerancesScale.length, computeAligningMoment,
			customTireParams.mfTireData);
		customTireParams.maxNormalizedLoad = 3.0f;  //For the given parameter set, larger loads than this resulted in values
		                                            //going out of the expected range in the Magic Formula Tire Model

		makeTireSymmetric(customTireParams.mfTireData);
	}

	PX_ASSERT(mBaseParams.axleDescription.getAxle(0) == 0);
	PX_ASSERT(mBaseParams.axleDescription.getAxle(1) == 0);
	mTireParamsList[0] = mCustomTireParams + 0;
	mTireParamsList[1] = mCustomTireParams + 0;

	PX_ASSERT(mBaseParams.axleDescription.getAxle(2) == 1);
	PX_ASSERT(mBaseParams.axleDescription.getAxle(3) == 1);
	mTireParamsList[2] = mCustomTireParams + 1;
	mTireParamsList[3] = mCustomTireParams + 1;

	const PxReal chassisMass = 1630.0f;
	const PxVec3 chassisMoi(2589.9f, 2763.1f, 607.0f);
	const PxReal massScale = chassisMass / mBaseParams.rigidBodyParams.mass;

	//Adjust some non custom parameters to match more closely with the wheel the custom tire model
	//parameters were taken from.
	const PxU32 wheelCount = mBaseParams.axleDescription.getNbWheels();
	for (PxU32 i = 0; i < wheelCount; i++)
	{
		PxVehicleWheelParams& wp = mBaseParams.wheelParams[i];
		wp.radius = PxReal(mTireParamsList[i]->mfTireData.sharedParams.r0);
		wp.halfWidth = 0.5f * 0.205f;
		wp.mass = 9.3f + 7.247f;
		wp.moi = 0.736f + 0.5698f;

		//Map the sprung masses etc. to the new total mass
		PxVehicleSuspensionForceParams& sfp = mBaseParams.suspensionForceParams[i];
		sfp.sprungMass *= massScale;
		sfp.stiffness *= massScale;

		//Adjust damping to a range that is not ultra extreme
		const PxReal dampingRatio = 0.3f;
		sfp.damping = dampingRatio * 2.0f * PxSqrt(sfp.sprungMass * sfp.stiffness);
	}

	mBaseParams.rigidBodyParams.mass = chassisMass;
	mBaseParams.rigidBodyParams.moi = chassisMoi;

	//Adjust some non custom parameters given that the model should be higher fidelity.
	mBaseParams.suspensionStateCalculationParams.limitSuspensionExpansionVelocity = true;
	mBaseParams.suspensionStateCalculationParams.suspensionJounceCalculationType = PxVehicleSuspensionJounceCalculationType::eSWEEP;
	mPhysXParams.physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eSWEEP;

	//Recreate PhysX actor and shapes since related properties changed
	mPhysXState.destroy();
	mPhysXState.create(mBaseParams, mPhysXParams, physics, cookingParams, defaultMaterial);

	return true;
}

void CustomTireVehicle::destroy()
{
	DirectDriveVehicle::destroy();
}

void CustomTireVehicle::initComponentSequence(const bool addPhysXBeginEndComponents)
{
	//Wake up the associated PxRigidBody if it is asleep and the vehicle commands signal an
	//intent to change state. 
	//Read from the physx actor and write the state (position, velocity etc) to the vehicle.
	if(addPhysXBeginEndComponents)
		mComponentSequence.add(static_cast<PxVehiclePhysXActorBeginComponent*>(this));

	//Read the input commands (throttle, brake etc) and forward them as torques and angles to the wheels on each axle.
	mComponentSequence.add(static_cast<PxVehicleDirectDriveCommandResponseComponent*>(this));

	//Work out which wheels have a non-zero drive torque and non-zero brake torque.
	//This is used to determine if any tire is to enter the "sticky" regime that will bring the 
	//vehicle to rest.
	mComponentSequence.add(static_cast<PxVehicleDirectDriveActuationStateComponent*>(this));

	//Start a substep group that can be ticked multiple times per update.
	//In this example, we perform multiple updates of the road geometry queries, suspensions, 
	//tires and wheels. Some tire models might need small time steps to be stable and running
	//the road geometry queries every substep can reduce large discontinuities (for example
	//having the wheel go from one frame with no ground contact to a highly compressed suspension
	//in the next frame). Running substeps on a subset of the operations is computationally
	//cheaper than simulating the entire sequence.
	mComponentSequenceSubstepGroupHandle = mComponentSequence.beginSubstepGroup(16);  //16 to get more or less a 1kHz update frequence, assuming main update is 60Hz

		//Perform a scene query against the physx scene to determine the plane and friction under each wheel.
		mComponentSequence.add(static_cast<PxVehiclePhysXRoadGeometrySceneQueryComponent*>(this));

		//Update the suspension compression given the plane under each wheel.
		//Update the kinematic compliance from the compression state of each suspension.
		//Convert suspension state to suspension force and torque.
		mComponentSequence.add(static_cast<PxVehicleSuspensionComponent*>(this));

		//Compute the load on the tire, the friction experienced by the tire 
		//and the lateral/longitudinal slip angles.
		//Convert load/friction/slip to tire force and torque.
		//If the vehicle is to come rest then compute the "sticky" velocity constraints to apply to the
		//vehicle.
		mComponentSequence.add(static_cast<CustomTireComponent*>(this));

		//Apply any velocity constraints to a data buffer that will be consumed by the physx scene
		//during the next physx scene update.
		mComponentSequence.add(static_cast<PxVehiclePhysXConstraintComponent*>(this));

		//Apply the tire force, brake force and drive force to each wheel and
		//forward integrate the rotation speed of each wheel.
		mComponentSequence.add(static_cast<PxVehicleDirectDrivetrainComponent*>(this));

		//Apply the suspension and tire forces to the vehicle's rigid body and forward 
		//integrate the state of the rigid body.
		mComponentSequence.add(static_cast<PxVehicleRigidBodyComponent*>(this));

	//Mark the end of the substep group.
	mComponentSequence.endSubstepGroup();

	//Update the rotation angle of the wheel by forwarding integrating the rotational
	//speed of each wheel.
	//Compute the local pose of the wheel in the rigid body frame after accounting 
	//suspension compression and compliance.
	mComponentSequence.add(static_cast<PxVehicleWheelComponent*>(this));

	//Write the local poses of each wheel to the corresponding shapes on the physx actor.
	//Write the momentum change applied to the vehicle's rigid body to the physx actor.
	//The physx scene can now try to apply that change to the physx actor.
	//The physx scene will account for collisions and constraints to be applied to the vehicle 
	//that occur by applying the change.
	if(addPhysXBeginEndComponents)
		mComponentSequence.add(static_cast<PxVehiclePhysXActorEndComponent*>(this));
}

}//namespace snippetvehicle2
