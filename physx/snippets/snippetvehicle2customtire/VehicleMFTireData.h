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

#ifndef VEHICLE_MF_TIRE_DATA_H
#define VEHICLE_MF_TIRE_DATA_H

#include "foundation/Px.h"
#include "foundation/PxMath.h"
#include "foundation/PxFlags.h"

namespace physx
{

// requirement: has to return 0 for input 0
template<typename TFloat>
PX_FORCE_INLINE PxI32 mfSignum(const TFloat v)
{
	// note: -0.0 returns 0, so sign is lost. +-NaN not covered

	return (TFloat(0.0) < v) - (v < TFloat(0.0));
}

/**
\brief Parameters for the Magic Formula Tire Model shared by various components of the model.

\note Contains derived parameters that need to be set as well before the model can be evaluated. The helper method
mfTireComputeDerivedSharedParams() can be used to compute those parameters.
*/
template<typename TFloat>
struct MFTireSharedParams
{
	TFloat r0; /// wheel radius at no load (SI unit would be metre [m])
	TFloat v0; /// reference velocity [m/s] (for normalization. Usually the speed at which measurements were taken, often 
	           /// 16.7m/s, i.e., 60km/h are used. SI unit would be metres per second [m/s])
	TFloat fz0; /// nominal vertical load (for normalization. SI unit would be Newton [N]). Make sure to call 
	            /// mfTireComputeDerivedSharedParams() after changing this parameter.
	TFloat pi0; /// nominal inflation pressure (for normalization. SI unit would be Pascal [Pa])

	TFloat epsilonV; /// small value to add to velocity at contact patch when dividing (to avoid division
	                 /// by 0 when vehicle is not moving)
	TFloat slipReductionLowVelocity; /// at low velocities the model is prone to show large oscillation. Scaling down the slip
	                                 /// in such scenarios will reduce the oscillation. This parameter defines a threshold for
	                                 /// the longitudinal velocity below which the divisor for the slip computation will be
	                                 /// interpolated between the actual longitudinal velocity and the specified threshold.
	                                 /// At 0 longitudinal velocity, slipReductionLowVelocity will be used as divisor. With
	                                 /// growing longitudinal velocity, the interpolation weight for slipReductionLowVelocity
	                                 /// will decrease fast (quadratic) and will show no effect for velocities above the threshold
	                                 /// anymore.
	TFloat lambdaFz0; /// user scaling factor for nominal load. Make sure to call mfTireComputeDerivedSharedParams() after
	                  /// changing this parameter.
	TFloat lambdaK_alpha; /// user scaling factor for cornering stiffness
	TFloat aMu; /// parameter for degressive friction factor computation. Vertical shifts of the force curves
	            /// should vanish slower when friction coefficients go to 0: (aMu * lambdaMu) / (1 + (aMu - 1) * lambdaMu).
	            /// Choose 10 as a good default. Make sure to call mfTireComputeDerivedSharedParams() after changing this parameter.
	TFloat zeta0; /// scaling factor to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1.
	TFloat zeta2; /// scaling factor to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1.

	// derived parameters
	TFloat recipFz0; /// 1 / fz0. mfTireComputeDerivedSharedParams() can be used to compute this value.
	TFloat fz0Scaled; /// fz0 * lambdaFz0. mfTireComputeDerivedSharedParams() can be used to compute this value.
	TFloat recipFz0Scaled; /// 1 / fz0Scaled. mfTireComputeDerivedSharedParams() can be used to compute this value.
	TFloat aMuMinus1; /// aMu - 1. mfTireComputeDerivedSharedParams() can be used to compute this value.
};

template<typename TFloat>
PX_FORCE_INLINE void mfTireComputeDerivedSharedParams(MFTireSharedParams<TFloat>& sharedParams)
{
	sharedParams.recipFz0 = TFloat(1.0) / sharedParams.fz0;
	sharedParams.fz0Scaled = sharedParams.fz0 * sharedParams.lambdaFz0;
	sharedParams.recipFz0Scaled = TFloat(1.0) / sharedParams.fz0Scaled;
	sharedParams.aMuMinus1 = sharedParams.aMu - TFloat(1.0);
}

template<typename TFloat>
struct MFTireVolatileSharedParams
{
	// gamma: camber angle
	// Fz: vertical load
	// Fz0Scaled: nominal vertical load (scaled by user scaling factor lambdaFz0)
	// pi: inflation pressure
    // pi0: nominal inflation pressure

	TFloat gamma;  // camber angle (in radians)
	TFloat gammaSqr;  // gamma^2
	TFloat sinGamma;  // sin(gamma)
	TFloat sinGammaAbs;  // |sin(gamma)|
	TFloat sinGammaSqr;  // sin(gamma)^2
	TFloat fzNormalized;  // Fz / Fz0
	TFloat fzNormalizedWithScale;  // Fz / Fz0Scaled
	TFloat dFz;  // normalized change in vertical load: (Fz - Fz0Scaled) / Fz0Scaled
	TFloat dFzSqr;  // dFz^2
	TFloat dpi;  // normalized inflation pressure change: (pi - pi0) / pi0
	TFloat dpiSqr;  // dpi^2
	TFloat lambdaMu_longitudinal; /// scaling factor for longitudinal friction mu term and thus peak value.
	TFloat lambdaMu_lateral; /// scaling factor for lateral friction mu term and thus peak value.
};

template<typename TConfig>
PX_FORCE_INLINE void mfTireComputeVolatileSharedParams(const typename TConfig::Float gamma, const typename TConfig::Float fz, 
	const typename TConfig::Float lambdaMu_longitudinal, const typename TConfig::Float lambdaMu_lateral,
	const typename TConfig::Float pi,
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	if (TConfig::supportCamber)
	{
		volatileSharedParams.gamma = gamma;
		volatileSharedParams.gammaSqr = gamma * gamma;
		volatileSharedParams.sinGamma = PxSin(gamma);
		volatileSharedParams.sinGammaAbs = PxAbs(volatileSharedParams.sinGamma);
		volatileSharedParams.sinGammaSqr = volatileSharedParams.sinGamma * volatileSharedParams.sinGamma;
	}

	volatileSharedParams.fzNormalized = fz * sharedParams.recipFz0;
	volatileSharedParams.fzNormalizedWithScale = fz * sharedParams.recipFz0Scaled;
	volatileSharedParams.dFz = (fz - sharedParams.fz0Scaled) * sharedParams.recipFz0Scaled;
	volatileSharedParams.dFzSqr = volatileSharedParams.dFz * volatileSharedParams.dFz;
	volatileSharedParams.lambdaMu_longitudinal = lambdaMu_longitudinal;
	volatileSharedParams.lambdaMu_lateral = lambdaMu_lateral;

	if (TConfig::supportInflationPressure)
	{
		volatileSharedParams.dpi = (pi - sharedParams.pi0) / sharedParams.pi0;
		volatileSharedParams.dpiSqr = volatileSharedParams.dpi * volatileSharedParams.dpi;
	}
}

template<typename TFloat>
struct MFTireLongitudinalForcePureParams
{
	/** @name magic formula: B (through longitudinal slip stiffness K)
	*/
	/**@{*/
	TFloat pK1; /// longitudinal slip stiffness (per load unit) Guidance: for many tires
	            /// (almost) linearly dependent on the vertical force. Typically, in the
	            /// range of 14 to 18. May be much higher for racing tires.
	TFloat pK2; /// variation of slip stiffness (per load unit) with load change
	TFloat pK3; /// exponential variation of slip stiffness with load change
	TFloat pp1; /// variation of slip stiffness with inflation pressure change
	            /// (set to 0 to ignore effect)
	TFloat pp2; /// variation of slip stiffness with inflation pressure change squared
	            /// (set to 0 to ignore effect)
	TFloat lambdaK; /// user scaling factor for slip stiffness
	TFloat epsilon; /// small value to avoid division by 0 if load is 0. The scale of the values
	                /// the epsilon gets added to is similar to the scale of the longitudinal force.
	/**@}*/

	/** @name magic formula: C
	*/
	/**@{*/
	TFloat pC1; /// shape factor. Guidance: value has to be larger or equal to 1, for example, 1.6
	            /// is a fair starting point.
	TFloat lambdaC; /// user scaling factor for shape factor
	/**@}*/

	/** @name magic formula: D
	*/
	/**@{*/
	TFloat pD1; /// friction mu term (per load unit). Guidance: for highly loaded tires
	            /// the value is below 1, for example around 0.8 on a dry surface. High
	            /// performance tires on racing cars may reach 1.5 or even 2.0 in extreme
	            /// cases.
	TFloat pD2; /// variation of friction mu term (per load unit) with load change.
	            /// Guidance: normally below 0 (typically between -0.1 and 0.0). Generally,
	            /// pD2 is larger than the counterpart in MFTireLateralForcePureParams.
	TFloat pD3; /// variation of friction mu term with camber angle squared
	            /// (set to 0 to ignore effect)
	TFloat pp3; /// variation of friction mu term with inflation pressure change
	            /// (set to 0 to ignore effect)
	TFloat pp4; /// variation of friction mu term with inflation pressure change squared
	            /// (set to 0 to ignore effect)
	//TFloat lambdaMu; // user scaling factor for friction mu term (and thus peak value).
	                   // (see MFTireVolatileSharedParams::lambdaMu_longitudinal)
	TFloat zeta1; /// scaling factor for friction mu term to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1.
	/**@}*/

	/** @name magic formula: E
	*/
	/**@{*/
	TFloat pE1; /// curvature factor. Guidance: value has to be smaller or equal to 1. An
	            /// increasing negative value will make the curve more peaky. 0 is a
	            /// fair starting point.
	TFloat pE2; /// variation of curvature factor with load change
	TFloat pE3; /// variation of curvature factor with load change squared
	TFloat pE4; /// scaling factor for curvature factor depending on signum 
	            /// of shifted longitudinal slip ratio (to create asymmetry under drive/brake
	            /// torque: 1.0 - pE4*sgn(slipRatioShifted). No effect when no slip)
	TFloat lambdaE; /// user scaling factor for curvature factor
	/**@}*/

	/** @name magic formula: Sh
	*/
	/**@{*/
	TFloat pH1; /// horizontal shift at nominal load
	TFloat pH2; /// variation of horizontal shift with load change
	TFloat lambdaH; /// user scaling factor for horizontal shift
	/**@}*/

	/** @name magic formula: Sv
	*/
	/**@{*/
	TFloat pV1; /// vertical shift (per load unit)
	TFloat pV2; /// variation of vertical shift (per load unit) with load change
	TFloat lambdaV; /// user scaling factor for vertical shift
	//TFloat lambdaMu; // user scaling factor for vertical shift (peak friction coefficient). See magic formula: D
	//TFloat zeta1; // scaling factor for vertical shift to take turn slip into account. See magic formula: D
	/**@}*/
};

template<typename TFloat>
struct MFTireLongitudinalForceCombinedParams
{
	/** @name magic formula: B
	*/
	/**@{*/
	TFloat rB1; /// curvature factor at force reduction peak. Guidance: 8.3 may be used as a starting point.
	TFloat rB2; /// variation of curvature factor at force reduction peak with longitudinal slip ratio.
	            /// Guidance: 5.0 may be used as a starting point.
	TFloat rB3; /// variation of curvature factor at force reduction peak with squared sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat lambdaAlpha; /// user scaling factor for influence of lateral slip angle alpha on longitudinal force
	/**@}*/

	/** @name magic formula: C
	*/
	/**@{*/
	TFloat rC1; /// shape factor. Guidance: 0.9 may be used as a starting point.
	/**@}*/

	/** @name magic formula: E
	*/
	/**@{*/
	TFloat rE1; /// long range falloff at nominal load
	TFloat rE2; /// variation of long range falloff with load change
	/**@}*/

	/** @name magic formula: Sh
	*/
	/**@{*/
	TFloat rH1; /// horizontal shift at nominal load. Set to 0 for a symmetric tire.
	/**@}*/
};

template<typename TFloat>
struct MFTireLateralForcePureParams
{
	/** @name magic formula: B (through cornering stiffness K_alpha)
	*/
	/**@{*/
	TFloat pK1; /// maximum cornering stiffness (per load unit) Guidance: values between
	            /// 10 and 20 can be expected (or higher for racing tires). Note: beware of
	            /// the sign when copying values from data sources. If the ISO sign
	            /// convention is used for the tire model, negative values will be seen.
	TFloat pK2; /// scaling factor for load at which cornering stiffness reaches maximum.
	            /// Guidance: typically, in a range of 1.5 to 3.
	TFloat pK3; /// variation of cornering stiffness with sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat pK4; /// shape factor to control limit and thus shape of stiffness curve
	            /// Guidance: typical value is 2 (especially, if camber angle is 0).
	TFloat pK5; /// scaling factor (depending on squared sine of camber angle) for load at which
	            /// cornering stiffness reaches maximum
	            /// (set to 0 to ignore effect)
	TFloat pp1; /// variation of cornering stiffness with inflation pressure change
	            /// (set to 0 to ignore effect)
	TFloat pp2; /// scaling factor (depending on inflation pressure change) for load at which
	            /// cornering stiffness reaches maximum
	            /// (set to 0 to ignore effect)
	//TFloat lambdaK_alpha; // user scaling factor for cornering stiffness (see MFTireSharedParams::lambdaK_alpha)
	TFloat epsilon; /// small value to avoid division by 0 if load is 0. The scale of the values
	                /// the epsilon gets added to is similar to the scale of the lareral force.
	TFloat zeta3; /// scaling factor for cornering stiffness to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1.
	/**@}*/

	/** @name magic formula: C
	*/
	/**@{*/
	TFloat pC1; /// shape factor. Guidance: value has to be larger or equal to 1, for example, 1.3
	            /// is a fair starting point.
	TFloat lambdaC; /// user scaling factor for shape factor
	/**@}*/

	/** @name magic formula: D
	*/
	/**@{*/
	TFloat pD1; /// friction mu term (per load unit). Guidance: for highly loaded tires
	            /// the value is below 1, for example around 0.8 on a dry surface. High
	            /// performance tires on racing cars may reach 1.5 or even 2.0 in extreme
	            /// cases.
	TFloat pD2; /// variation of friction mu term (per load unit) with load change.
	            /// Guidance: normally below 0 (typically between -0.1 and 0.0). Generally,
	            /// pD2 is smaller than the counterpart in MFTireLongitudinalForcePureParams.
	TFloat pD3; /// variation of friction mu term with squared sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat pp3; /// variation of friction mu term with inflation pressure change
	            /// (set to 0 to ignore effect)
	TFloat pp4; /// variation of friction mu term with inflation pressure change squared
	            /// (set to 0 to ignore effect)
	//TFloat lambdaMu; // user scaling factor for friction mu term (and thus peak value).
	                   // (see MFTireVolatileSharedParams::lambdaMu_lateral)
	//TFloat zeta2; // scaling factor for friction mu term to take turn slip into account.
	                // See MFTireSharedParams::zeta2.
	/**@}*/

	/** @name magic formula: E
	*/
	/**@{*/
	TFloat pE1; /// curvature factor. Guidance: value has to be smaller or equal to 1. An
	            /// increasing negative value will make the curve more peaky. 0 is a
	            /// fair starting point.
	TFloat pE2; /// variation of curvature factor with load change
	TFloat pE3; /// scaling factor for curvature factor depending on signum 
	            /// of shifted slip angle (to create asymmetry 
	            /// under positive/negative slip. No effect when no slip).
	            /// Set to 0 for a symmetric tire.
	TFloat pE4; /// variation of curvature factor with sine of camber angle and signum
	            /// of shifted slip angle.
	            /// (set to 0 to ignore effect)
	TFloat pE5; /// variation of curvature factor with squared sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat lambdaE; /// user scaling factor for curvature factor
	/**@}*/

	/** @name magic formula: Sh (with a camber stiffness term K_gamma)
	*/
	/**@{*/
	TFloat pH1; /// horizontal shift at nominal load. Set to 0 for a symmetric tire.
	TFloat pH2; /// variation of horizontal shift with load change. Set to 0 for a symmetric tire.
	TFloat pK6; /// camber stiffness (per load unit)
	            /// (set to 0 to ignore effect)
	TFloat pK7; /// variation of camber stiffness (per load unit) with load change
	            /// (set to 0 to ignore effect)
	TFloat pp5; /// variation of camber stiffness with inflation pressure change
	            /// (set to 0 to ignore effect)
	TFloat lambdaH; /// user scaling factor for horizontal shift
	//TFloat lambdaK_gamma; // user scaling factor for horizontal shift of camber stiffness term
	                        // (no effect if camber angle is 0).
	                        // See magic formula: Sv
	TFloat epsilonK; /// small value to avoid division by 0 in case cornering stiffness is 0
	                 /// (due to load being 0, for example). The scale of the values the
	                 /// epsilon gets added to is similar to the scale of the lareral force
	                 /// multiplied by the slope/stiffness when slip angle is approaching 0.
	//TFloat zeta0; // scaling factor for camber stiffness term to take turn slip into account.
	                // See MFTireSharedParams::zeta0.
	TFloat zeta4; /// horizontal shift term to take turn slip and camber into account.
	              /// If path radius is large and camber small, it can be set to 1 (since 1 gets
	              /// subtracted this will result in a 0 term).
	/**@}*/

	/** @name magic formula: Sv
	*/
	/**@{*/
	TFloat pV1; /// vertical shift (per load unit). Set to 0 for a symmetric tire.
	TFloat pV2; /// variation of vertical shift (per load unit) with load change. Set to 0 for a symmetric tire.
	TFloat pV3; /// vertical shift (per load unit) depending on sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat pV4; /// variation of vertical shift (per load unit) with load change depending on sine
	            /// of camber angle (set to 0 to ignore effect)
	TFloat lambdaV; /// user scaling factor for vertical shift
	TFloat lambdaK_gamma; /// user scaling factor for vertical shift depending on sine of camber angle
	//TFloat lambdaMu; // user scaling factor for vertical shift (peak friction coefficient). See magic formula: D
	//TFloat zeta2; // scaling factor for vertical shift to take turn slip into account.
	                // See MFTireSharedParams::zeta2.
	/**@}*/
};

template<typename TFloat>
struct MFTireLateralForceCombinedParams
{
	/** @name magic formula: B
	*/
	/**@{*/
	TFloat rB1; /// curvature factor at force reduction peak. Guidance: 4.9 may be used as a starting point.
	TFloat rB2; /// variation of curvature factor at force reduction peak with lateral slip
	            /// Guidance: 2.2 may be used as a starting point.
	TFloat rB3; /// shift term for lateral slip. rB2 gets applied to shifted slip. Set to 0 for a symmetric tire.
	TFloat rB4; /// variation of curvature factor at force reduction peak with squared sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat lambdaKappa; /// user scaling factor for influence of longitudinal slip ratio kappa on lateral force
	/**@}*/

	/** @name magic formula: C
	*/
	/**@{*/
	TFloat rC1; /// shape factor. Guidance: 1.0 may be used as a starting point.
	/**@}*/

	/** @name magic formula: E
	*/
	/**@{*/
	TFloat rE1; /// long range falloff at nominal load
	TFloat rE2; /// variation of long range falloff with load change
	/**@}*/

	/** @name magic formula: Sh
	*/
	/**@{*/
	TFloat rH1; /// horizontal shift at nominal load
	TFloat rH2; /// variation of horizontal shift with load change
	/**@}*/

	/** @name magic formula: Sv
	*/
	/**@{*/
	TFloat rV1; /// vertical shift (per load unit). Set to 0 for a symmetric tire.
	TFloat rV2; /// variation of vertical shift (per load unit) with load change. Set to 0 for a symmetric tire.
	TFloat rV3; /// variation of vertical shift (per load unit) with sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat rV4; /// variation of vertical shift (per load unit) with lateral slip angle
	TFloat rV5; /// variation of vertical shift (per load unit) with longitudinal slip ratio
	TFloat rV6; /// variation of vertical shift (per load unit) with arctan of longitudinal slip ratio
	TFloat lambdaV; /// user scaling factor for vertical shift
	//TFloat zeta2; // scaling factor for vertical shift to take turn slip into account.
	                // See MFTireSharedParams::zeta2.
	/**@}*/
};

template<typename TFloat>
struct MFTireAligningTorqueVolatileSharedParams
{
	// alpha: slip angle
	// Vcx: longitudinal velocity at contact patch
	// Vc: velocity at contact patch (longitudinal and lateral combined)

	TFloat cosAlpha;  // cos(slip angle) = Vcx / (Vc + epsilon) (signed value of Vcx allows to support backwards running)
	TFloat signumVc_longitudinal;  // signum of longitudinal velocity at contact patch
};

template<typename TFloat>
PX_FORCE_INLINE void mfTireComputeAligningTorqueVolatileSharedParams(const TFloat vc, const TFloat vc_longitudinal,
	const TFloat epsilon, MFTireAligningTorqueVolatileSharedParams<TFloat>& volatileSharedParams)
{
	volatileSharedParams.cosAlpha = vc_longitudinal / (vc + epsilon);
	volatileSharedParams.signumVc_longitudinal = static_cast<TFloat>(mfSignum(vc_longitudinal));
}

template<typename TFloat>
struct MFTireAligningTorquePurePneumaticTrailParams
{
	/** @name magic formula: B
	*/
	/**@{*/
	TFloat qB1; /// curvature factor at pneumatic trail peak (at nominal load)
	TFloat qB2; /// variation of curvature factor at pneumatic trail peak with load change
	TFloat qB3; /// variation of curvature factor at pneumatic trail peak with load change squared
	TFloat qB5; /// variation of curvature factor at pneumatic trail peak with sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat qB6; /// variation of curvature factor at pneumatic trail peak with squared sine of camber angle
	            /// (set to 0 to ignore effect)
	//TFloat lambdaK_alpha; // user scaling factor for curvature factor at pneumatic trail peak
	                        // (see MFTireSharedParams::lambdaK_alpha)
	//TFloat lambdaMu; // inverse user scaling factor for curvature factor at pneumatic trail peak
	                   // (see MFTireVolatileSharedParams::lambdaMu_lateral)
	/**@}*/

	/** @name magic formula: C
	*/
	/**@{*/
	TFloat qC1; /// shape factor (has to be greater than 0)
	/**@}*/

	/** @name magic formula: D
	*/
	/**@{*/
	TFloat qD1; /// pneumatic trail peak (per normalized load-torque unit)
	TFloat qD2; /// variation of pneumatic trail peak (per normalized load-torque unit) with load change
	TFloat qD3; /// variation of pneumatic trail peak with sine of camber angle
	            /// (set to 0 to ignore effect). Set to 0 for a symmetric tire.
	TFloat qD4; /// variation of pneumatic trail peak with squared sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat pp1; /// variation of pneumatic trail peak with inflation pressure change
	            /// (set to 0 to ignore effect)
	TFloat lambdaT; /// user scaling factor for pneumatic trail peak
	TFloat zeta5; /// scaling factor for pneumatic trail peak to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1.
	/**@}*/

	/** @name magic formula: E
	*/
	/**@{*/
	TFloat qE1; /// long range falloff at nominal load
	TFloat qE2; /// variation of long range falloff with load change
	TFloat qE3; /// variation of long range falloff with load change squared
	TFloat qE4; /// scaling factor for long range falloff depending on sign 
	            /// of shifted slip angle (to create asymmetry 
	            /// under positive/negative slip. No effect when no slip).
	            /// Set to 0 for a symmetric tire.
	TFloat qE5; /// scaling factor for long range falloff depending on sine of camber
	            /// angle and sign of shifted slip angle (to create asymmetry 
	            /// under positive/negative slip. No effect when no slip.
	            /// Set to 0 to ignore effect)
	/**@}*/

	/** @name magic formula: Sh
	*/
	/**@{*/
	TFloat qH1; /// horizontal shift at nominal load. Set to 0 for a symmetric tire.
	TFloat qH2; /// variation of horizontal shift with load change. Set to 0 for a symmetric tire.
	TFloat qH3; /// horizontal shift at nominal load depending on sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat qH4; /// variation of horizontal shift with load change depending on sine of camber angle 
	            /// (set to 0 to ignore effect)
	/**@}*/
};

template<typename TFloat>
struct MFTireAligningTorquePureResidualTorqueParams
{
	/** @name magic formula: B
	*/
	/**@{*/
	TFloat qB9; /// curvature factor at residual torque peak
	TFloat qB10; /// curvature factor at residual torque peak (multiplier for B*C term of lateral force under pure slip)
	//TFloat lambdaK_alpha; // user scaling factor for curvature factor at residual torque peak
	                        // (see MFTireSharedParams::lambdaK_alpha)
	//TFloat lambdaMu; // inverse user scaling factor for curvature factor at residual torque peak
	                   // (see MFTireVolatileSharedParams::lambdaMu_lateral)
	TFloat zeta6; /// scaling factor for curvature factor at residual torque peak to take turn slip
	              /// into account.
	              /// If path radius is large and camber small, it can be set to 1.
	/**@}*/

	/** @name magic formula: C
	*/
	/**@{*/
	TFloat zeta7; /// shape factor to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1.
	/**@}*/

	/** @name magic formula: D
	*/
	/**@{*/
	TFloat qD6; /// residual torque peak (per load-torque unit). Set to 0 for a symmetric tire.
	TFloat qD7; /// variation of residual torque peak (per load-torque unit) with load change.
	            /// Set to 0 for a symmetric tire.
	TFloat qD8; /// residual torque peak (per load-torque unit) depending on sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat qD9; /// variation of residual torque peak (per load-torque unit) with load
	            /// change and depending on sine of camber angle (set to 0 to ignore effect)
	TFloat qD10; /// residual torque peak (per load-torque unit) depending on signed square of
	             /// sine of camber angle (set to 0 to ignore effect)
	TFloat qD11; /// variation of residual torque peak (per load-torque unit) with load
	             /// change and depending on signed square of sine of camber angle
	             /// (set to 0 to ignore effect)
	TFloat pp2; /// variation of residual torque peak with inflation pressure change and depending
	            /// on sine of camber angle
	            /// (set to 0 to ignore effect)
	TFloat lambdaR; /// user scaling factor for residual torque peak
	TFloat lambdaK_gamma; /// user scaling factor for residual torque peak depending on sine of camber angle
	//TFloat lambdaMu; // user scaling factor for residual torque peak
	                   // (see MFTireVolatileSharedParams::lambdaMu_lateral)
	//TFloat zeta0; // scaling factor for residual torque peak to take turn slip into account.
	                // See MFTireSharedParams::zeta0.
	//TFloat zeta2; // scaling factor for residual torque peak to take turn slip into account.
	                // See MFTireSharedParams::zeta2.
	TFloat zeta8; /// additional term for residual torque peak to take turn slip into account.
	              /// If path radius is large and camber small, it can be set to 1 (since 1 gets
	              /// subtracted this will result in a 0 term).
	/**@}*/
};

template<typename TFloat>
struct MFTireAligningTorqueCombinedParams
{
	TFloat sS1; /// effect (per radius unit) of longitudinal force on aligning torque. Set to 0 for a symmetric tire.
	TFloat sS2; /// effect (per radius unit) of longitudinal force on aligning torque with lateral force
	TFloat sS3; /// effect (per radius unit) of longitudinal force on aligning torque with sine of camber angle
	TFloat sS4; /// effect (per radius unit) of longitudinal force on aligning torque with sine of camber angle and load change
	TFloat lambdaS; /// user scaling factor for effect of longitudinal force on aligning torque
};

template<typename TFloat>
struct MFTireFreeRotatingRadiusParams
{
	TFloat qre0; /// scaling factor for unloaded tire radius. Guidance: set to 1 if no adaptation to measurements are needed.
	TFloat qV1; /// increase of radius (per length unit) with normalized velocity squared (the tire radius grows
	            /// due to centrifugal force)
};

/**
\brief Parameters for the Magic Formula Tire Model related to vertical tire stiffness.

\note Contains derived parameters that need to be set as well before the model can be evaluated. The helper method
mfTireComputeDerivedVerticalTireStiffnessParams() can be used to compute those parameters.
*/
template<typename TFloat>
struct MFTireVerticalStiffnessParams
{
	TFloat qF1; /// vertical tire stiffness (per load unit and normalized tire deflection (deflection / unloaded radius)).
	            /// If qF2 is set to 0 and the vertical tire stiffness c0 is known, qF1 can be computed as c0 * r0 / F0
	            /// (r0: unloaded tire radius, F0: nominal tire load).
	TFloat qF2; /// vertical tire stiffness (per load unit and normalized tire deflection squared)
	TFloat ppF1; /// variation of vertical tire stiffness with inflation pressure change
	             /// (set to 0 to ignore effect). Guidance: suggested value range is between 0.7 and 0.9.
	TFloat lambdaC; /// user scaling factor for vertical tire stiffness.

	// derived parameters
	TFloat c0; /// vertical tire stiffness at nominal vertical load, nominal inflation pressure, no tangential forces and
	           /// zero forward velocity. mfTireComputeDerivedVerticalTireStiffnessParams() can be used to compute the value.
};

template<typename TFloat>
PX_FORCE_INLINE void mfTireComputeDerivedVerticalTireStiffnessParams(const TFloat r0, const TFloat fz0,
	MFTireVerticalStiffnessParams<TFloat>& params)
{
	params.c0 = params.lambdaC * (fz0 / r0) * PxSqrt((params.qF1 * params.qF1) + (TFloat(4.0) * params.qF2));
}

template<typename TFloat>
struct MFTireEffectiveRollingRadiusParams
{
	TFloat Freff; /// tire compression force (per load unit), linear part. Guidance: for radial tires a value
	              /// around 0.01 is suggested, for bias ply tires a value around 0.333.
	TFloat Dreff; /// scaling factor for tire compression force, non-linear part. Guidance: for radial tires
	              /// a value around 0.24 is suggested, for bias ply tires a value of 0.
	TFloat Breff; /// gradient of tire compression force, non-linear part. Guidance: for radial tires a value
	              /// around 8 is suggested, for bias ply tires a value of 0.
};

template<typename TFloat>
struct MFTireNormalLoadParams
{
	TFloat qV2; /// variation of normal load (per load unit) with normalized longitudinal velocity from tire rotation
	TFloat qFc_longitudinal; /// decrease of normal load (per load unit) with normalized longitudinal force squared.
	                         /// Can be considered optional (set to 0) for passenger car tires but should be considered for racing
	                         /// tires.
	TFloat qFc_lateral; /// decrease of normal load (per load unit) with normalized lateral force squared.
	                    /// Can be considered optional (set to 0) for passenger car tires but should be considered for racing
	                    /// tires.
	//TFloat qF1; // see MFTireVerticalStiffnessParams::qF1
	//TFloat qF2; // see MFTireVerticalStiffnessParams::qF2
	TFloat qF3; /// vertical tire stiffness (per load unit and normalized tire deflection (deflection / unloaded radius))
	            /// depending on camber angle squared (set to 0 to ignore effect)
	//TFloat ppF1; // see MFTireVerticalStiffnessParams::ppF1
};

struct MFTireDataFlag
{
	enum Enum
	{
		eALIGNING_MOMENT = 1 << 0  /// Compute the aligning moment of the tire.
	};
};

template<typename TFloat>
struct MFTireDataT
{
	MFTireDataT()
		: flags(0)
	{
	}


	MFTireSharedParams<TFloat> sharedParams;
	MFTireFreeRotatingRadiusParams<TFloat> freeRotatingRadiusParams;
	MFTireVerticalStiffnessParams<TFloat> verticalStiffnessParams;
	MFTireEffectiveRollingRadiusParams<TFloat> effectiveRollingRadiusParams;
	MFTireNormalLoadParams<TFloat> normalLoadParams;
	MFTireLongitudinalForcePureParams<TFloat> longitudinalForcePureParams;
	MFTireLongitudinalForceCombinedParams<TFloat> longitudinalForceCombinedParams;
	MFTireLateralForcePureParams<TFloat> lateralForcePureParams;
	MFTireLateralForceCombinedParams<TFloat> lateralForceCombinedParams;
	MFTireAligningTorquePureResidualTorqueParams<TFloat> aligningTorquePureResidualTorqueParams;
	MFTireAligningTorquePurePneumaticTrailParams<TFloat> aligningTorquePurePneumaticTrailParams;
	MFTireAligningTorqueCombinedParams<TFloat> aligningTorqueCombinedParams;
	PxFlags<MFTireDataFlag::Enum, PxU32> flags;
};

} // namespace physx

#endif //VEHICLE_MF_TIRE_DATA_H
