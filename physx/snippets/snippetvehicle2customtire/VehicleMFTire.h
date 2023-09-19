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

#ifndef VEHICLE_MF_TIRE_H
#define VEHICLE_MF_TIRE_H

#include "foundation/Px.h"
#include "foundation/PxAssert.h"
#include "foundation/PxMath.h"

#include "VehicleMFTireData.h"

namespace physx
{

template<typename TFloat>
struct MFTireOverturningCoupleParams
{
	TFloat qS1; /// overturning couple (per load-torque unit). Set to 0 for a symmetric tire.
	TFloat qS2; /// variation of overturning couple (per load-torque unit) with camber angle
	            /// (set to 0 to ignore effect)
	TFloat qS3; /// variation of overturning couple (per load-torque unit) with normalized lateral force
	TFloat qS4; /// peak of sine * cosine type contribution
	TFloat qS5; /// shape factor of cosine type contribution depending on normalized load
	TFloat qS6; /// curvature factor of cosine type contribution depending on normalized load
	TFloat qS7; /// horizontal shift of sine type contribution depending on camber angle
	            /// (set to 0 to ignore effect)
	TFloat qS8; /// shape factor of sine type contribution depending on normalized lateral force
	TFloat qS9; /// stiffness factor of sine type contribution depending on normalized lateral force
	TFloat qS10; /// shape factor of arctan type contribution depending on normalized load
	             /// and camber angle
	             /// (set to 0 to ignore effect)
	TFloat qS11; /// stiffness factor of arctan type contribution depending on normalized load
	             /// and camber angle
	             /// (set to 0 to ignore effect)
	TFloat ppM1; /// variation of camber angle dependent overturning couple with inflation pressure change
	             /// (set to 0 to ignore effect)
	             /// (no effect if camber angle is 0)
	TFloat lambdaVM; /// user scaling factor for overturning couple coefficient
	TFloat lambdaM; /// user scaling factor for overturning couple
};

template<typename TFloat>
struct MFTireRollingResistanceMomentParams
{
	TFloat qS1; /// rolling resistance moment (per load-torque unit)
	TFloat qS2; /// variation of rolling resistance moment with normalized longitudinal force
	TFloat qS3; /// variation of rolling resistance moment with normalized longitudinal velocity
	TFloat qS4; /// variation of rolling resistance moment with normalized longitudinal velocity to the power of 4
	TFloat qS5; /// variation of rolling resistance moment with camber angle squared
	            /// (set to 0 to ignore effect)
	TFloat qS6; /// variation of rolling resistance moment with camber angle squared and normalized load
	            /// (set to 0 to ignore effect)
	TFloat qS7; /// variation of rolling resistance moment with normalized load to the power of qS7
	TFloat qS8; /// variation of rolling resistance moment with normalized inflation pressure to the power of qS8
	            /// (set to 0 to ignore effect)
	TFloat lambdaM; /// user scaling factor for rolling resistance moment
};

//#define MF_ARCTAN(x) PxAtan(x)
#define MF_ARCTAN(x) mfApproximateArctan(x)  // more than 10% speedup could be seen

template <typename TFloat>
PX_FORCE_INLINE TFloat mfApproximateArctan(TFloat x)
{
	const TFloat absX = PxAbs(x);
	const TFloat a = TFloat(1.1);
	const TFloat b = TFloat(1.6);

	const TFloat nom = x * ( TFloat(1.0) + (a * absX) );
	const TFloat denom = TFloat(1.0)  +  ( (TFloat(2.0)/TFloat(PxPi))  *  ((b*absX) + (a*x*x)) );
	const TFloat result = nom / denom;
	return result;
}

PX_FORCE_INLINE PxF32 mfExp(PxF32 x)
{
	return PxExp(x);
}

PX_FORCE_INLINE PxF64 mfExp(PxF64 x)
{
	return exp(x);
}

PX_FORCE_INLINE PxF32 mfPow(PxF32 base, PxF32 exponent)
{
	return PxPow(base, exponent);
}

PX_FORCE_INLINE PxF64 mfPow(PxF64 base, PxF64 exponent)
{
	return pow(base, exponent);
}

template<typename TFloat>
TFloat mfMagicFormulaSine(const TFloat x, const TFloat B, const TFloat C,const TFloat D,
	const TFloat E)
{
	//
	// Magic Formula (sine version)
	//
	// y(x) = D * sin[  C * arctan{ B*x - E * (B*x - arctan(B*x)) }  ]
	//
	//              y
	//              ^
	//              |
	//              |                /
	//              |               /
	//              |              /
	//              |             /          __________
	//              |            /      ____/     ^    \_____________
	//              |           /   ___/          |                  \_____________________________________ y(x)
	//              |          / __/              |
	//              |         /_/                 |
	//              |        /                    |
	//              |       /                     |
	//              |      /                      | D
	//              |     /                       |
	//              |    /                        |
	//              |   /                         |
	//              |  /---  arctan(B*C*D)        |
	//              | /    \                      |
	//              |/      |                     v
	//  ------------/---------------------------------------------------------------------------------------------> x
	//             /|
	//            / | 
	//           /  |
	//          /   |
	//
	// ----------------------------------------------------------------------------------------------------------------
	// B: stiffness factor (tune slope at the origin)
	// ----------------------------------------------------------------------------------------------------------------
	// C: shape factor (controls limits and thus general shape of the curve)
	// ----------------------------------------------------------------------------------------------------------------
	// D: peak value (for C >= 1)
	// ----------------------------------------------------------------------------------------------------------------
	// E: curvature factor (controls curvature at peak and horizontal position of peak)
	// ----------------------------------------------------------------------------------------------------------------
	//

	const TFloat Bpart = B  *  x;
	const TFloat Epart = E  *  ( Bpart - MF_ARCTAN(Bpart) );
	const TFloat Cpart = C  *  MF_ARCTAN( Bpart - Epart );
	const TFloat Dpart = D  *  PxSin(Cpart);
	return Dpart;
}

template<typename TFloat>
TFloat mfMagicFormulaCosine(const TFloat x, const TFloat B, const TFloat C,const TFloat D,
	const TFloat E)
{
	/*
	// Magic Formula (cosine version)
	//
	// y(x) = D * cos[  C * arctan{ B*x - E * (B*x - arctan(B*x)) }  ]
	//
	//                                            y
	//                                            ^
	//                                            |
	//                                            |
	//                                        ____|____
	//                                   ____/    ^    \____
	//                               ___/         |         \___ y(x)
	//                            __/             |             \__
	//                          _/                |                \_
	//                         /                  | D                \
	//                       _/                   |                   \_
	//                    __/                     |                     \__
	//                ___/                        |                        \___
	//           ____/                            v                            \____
	//  ---_____/-------------------------------------------------------------------\_____------------------------> x  ^
	//  __/                                       |                                 ^     \______                      | -ya
	//                                            |                                 |            \_____________        v
	//                                            |                                 x0                     --------------
	//                                            |
	//                                            |
	//
	// ----------------------------------------------------------------------------------------------------------------
	// B: curvature factor (controls curvature at peak)
	// ----------------------------------------------------------------------------------------------------------------
	// C: shape factor (controls limit ya and thus general shape of the curve)
	// ----------------------------------------------------------------------------------------------------------------
	// D: peak value (for C >= 1)
	// ----------------------------------------------------------------------------------------------------------------
	// E: controls shape at larger values and controls location x0 of intersection with x-axis
	// ----------------------------------------------------------------------------------------------------------------
	*/

	const TFloat Bpart = B  *  x;
	const TFloat Epart = E  *  ( Bpart - MF_ARCTAN(Bpart) );
	const TFloat Cpart = C  *  MF_ARCTAN( Bpart - Epart );
	const TFloat Dpart = D  *  PxCos(Cpart);
	return Dpart;
}

template<typename TFloat>
TFloat mfMagicFormulaCosineNoD(const TFloat x, const TFloat B, const TFloat C,const TFloat E)
{
	//
	// Magic Formula (cosine version without D factor)
	//
	// y(x) = cos[  C * arctan{ B*x - E * (B*x - arctan(B*x)) }  ]
	//
	// see magicFormulaCosine() for some information
	//

	const TFloat Bpart = B  *  x;
	const TFloat Epart = E  *  ( Bpart - MF_ARCTAN(Bpart) );
	const TFloat Cpart = C  *  MF_ARCTAN( Bpart - Epart );
	const TFloat result = PxCos(Cpart);
	return result;
}

template<typename TFloat>
TFloat mfMagicFormulaCosineNoE(const TFloat x, const TFloat B, const TFloat C,const TFloat D)
{
	//
	// Magic Formula (cosine version without E factor)
	//
	// y(x) = D * cos[  C * arctan{ B*x }  ]
	//
	// see magicFormulaCosine() for some information
	//

	const TFloat Bpart = B  *  x;
	const TFloat Cpart = C  *  MF_ARCTAN( Bpart );
	const TFloat Dpart = D  *  PxCos(Cpart);
	return Dpart;
}

// Vertical shifts of the longitudinal/lateral force curves should vanish slower when friction coefficients go to 0
template<typename TFloat>
PX_FORCE_INLINE TFloat mfTireDegressiveFriction(const TFloat lambdaMu, const TFloat aMu, const TFloat aMuMinus1)
{
	const TFloat lambdaMuDegressive = (aMu * lambdaMu)  /  
		( TFloat(1.0) + (aMuMinus1 * lambdaMu) );

	return lambdaMuDegressive;
}

/**
\brief Longitudinal force at pure longitudinal slip

\note Ignores interdependency with lateral force (in other words, tire rolling on a straight line with 
no slip angle [alpha = 0])

\param[in] kappa Longitudinal slip ratio.
\param[in] fz Vertical load. fz >= 0.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] sharedParams Model parameters not just used for longitudinal force
\param[in] volatileSharedParams Model parameters not just used for longitudinal force
\param[out] K_kappa Magic formula longitudinal slip stiffness factor K
\return Longitudinal force.
*/
template<typename TConfig>
typename TConfig::Float mfTireLongitudinalForcePure(const typename TConfig::Float kappa, const typename TConfig::Float fz, 
	const MFTireLongitudinalForcePureParams<typename TConfig::Float>& params, const MFTireSharedParams<typename TConfig::Float>& sharedParams,
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams,
	typename TConfig::Float& K_kappa)
{
	typedef typename TConfig::Float TFloat;

	// magic formula:
	// x:    longitudinal slip ratio (kappa)
	// y(x): longitudinal force

	//
	// magic formula: Sh; horizontal shift
	//
	const TFloat Sh = ( params.pH1 + (params.pH2 * volatileSharedParams.dFz) )  *  params.lambdaH;

	//
	// magic formula: Sv; vertical shift
	//
	const TFloat lambdaMuDegressive = mfTireDegressiveFriction(volatileSharedParams.lambdaMu_longitudinal, sharedParams.aMu, sharedParams.aMuMinus1);
	TFloat Sv = fz  *  ( params.pV1 + (params.pV2 * volatileSharedParams.dFz) )  *  params.lambdaV  *  lambdaMuDegressive;
	if (TConfig::supportTurnSlip)
		Sv *= params.zeta1;

	//
	//
	//
	const TFloat kappaShifted = kappa + Sh;
	
	//
	// magic formula: C; shape factor
	//
	const TFloat C = params.pC1 * params.lambdaC;
	PX_ASSERT(C > TFloat(0.0));

	//
	// magic formula: D; peak value
	//
	TFloat mu = (params.pD1 + (params.pD2 * volatileSharedParams.dFz))  *  volatileSharedParams.lambdaMu_longitudinal;
	if (TConfig::supportCamber)
	{
		const TFloat mu_camberFactor = TFloat(1.0) - (params.pD3 * volatileSharedParams.gammaSqr);
		mu *= mu_camberFactor;
	}
	if (TConfig::supportInflationPressure)
	{
		const TFloat mu_inflationPressureFactor = TFloat(1.0)  +  (params.pp3 * volatileSharedParams.dpi)  +  (params.pp4 * volatileSharedParams.dpiSqr);
		mu *= mu_inflationPressureFactor;
	}
	TFloat D = mu * fz;
	if (TConfig::supportTurnSlip)
		D *= params.zeta1;
	PX_ASSERT(D >= TFloat(0.0));

	//
	// magic formula: E; curvature factor
	//
	const TFloat E = ( params.pE1 + (params.pE2*volatileSharedParams.dFz) + (params.pE3*volatileSharedParams.dFzSqr) )  *  ( TFloat(1.0) - (params.pE4*mfSignum(kappaShifted)) )  *  params.lambdaE;
	PX_ASSERT(E <= TFloat(1.0));

	//
	// longitudinal slip stiffness
	//
	TFloat K = fz  *  (params.pK1 + (params.pK2 * volatileSharedParams.dFz))  *  mfExp(params.pK3 * volatileSharedParams.dFz)  *  params.lambdaK;
	if (TConfig::supportInflationPressure)
	{
		const TFloat K_inflationPressureFactor = TFloat(1.0)  +  (params.pp1 * volatileSharedParams.dpi)  +  (params.pp2 * volatileSharedParams.dpiSqr);
		K *= K_inflationPressureFactor;
	}
	K_kappa = K;

	//
	// magic formula: B; stiffness factor
	//
	const TFloat B = K  /  ((C*D) + params.epsilon);

	//
	// resulting force
	//
	const TFloat F = mfMagicFormulaSine(kappaShifted, B, C, D, E) + Sv;

	return F;
}

/**
\brief Longitudinal force with combined slip

\note Includes the interdependency with lateral force

\param[in] kappa Longitudinal slip ratio.
\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] fx0 The longitudinal force at pure longitudinal slip.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParams Model parameters not just used for longitudinal force
\return Longitudinal force.

@see mfTireLongitudinalForcePure()
*/
template<typename TConfig>
typename TConfig::Float mfTireLongitudinalForceCombined(const typename TConfig::Float kappa, const typename TConfig::Float tanAlpha, const typename TConfig::Float fx0,
	const MFTireLongitudinalForceCombinedParams<typename TConfig::Float>& params,
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	//
	// magic formula: Sh; horizontal shift
	//
	const TFloat Sh = params.rH1;

	//
	//
	//
	const TFloat tanAlphaShifted = tanAlpha + Sh;

	//
	// magic formula: B; curvature factor
	//
	TFloat B = params.rB1;
	if (TConfig::supportCamber)
	{
		const TFloat B_term_camber = params.rB3 * volatileSharedParams.sinGammaSqr;
		B += B_term_camber;
	}
	const TFloat B_cosFactor = PxCos(MF_ARCTAN(params.rB2 * kappa));
	B *= B_cosFactor * params.lambdaAlpha;
	PX_ASSERT(B > TFloat(0.0));
	
	//
	// magic formula: C; shape factor
	//
	const TFloat C = params.rC1;

	//
	// magic formula: E;
	//
	const TFloat E = params.rE1 + (params.rE2*volatileSharedParams.dFz);
	PX_ASSERT(E <= TFloat(1.0));

	//
	// resulting force
	//
	const TFloat G0 = mfMagicFormulaCosineNoD(Sh, B, C, E);
	PX_ASSERT(G0 > TFloat(0.0));
	const TFloat G = mfMagicFormulaCosineNoD(tanAlphaShifted, B, C, E) / G0;
	PX_ASSERT(G > TFloat(0.0));
	const TFloat F = G * fx0;

	return F;
}

/**
\brief Lateral force at pure lateral/side slip

\note Ignores interdependency with longitudinal force (in other words, assumes no longitudinal slip, that is, 
longitudinal slip ratio of 0 [kappa = 0])

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] fz Vertical load (force). fz >= 0.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] sharedParams Model parameters not just used for lateral force
\param[in] volatileSharedParams Model parameters not just used for lateral force
\param[in,out] fy0NoCamberNoTurnSlip If not NULL, the lateral force discarding camber and turn slip
               (basically, the resulting force if both were 0) will be written to the pointer target.
\param[out] B_out Magic formula stiffnes factor B
\param[out] C_out Magic formula shape factor C
\param[out] K_alpha_withEpsilon_out Magic formula cornering stiffness factor K with a small epsilon added
            to avoid divison by 0.
\param[out] Sh_out Magic formula horizontal shift Sh
\param[out] Sv_out Magic formula vertical shift Sv
\param[out] mu_zeta2_out The friction factor multiplied by MFTireSharedParams::zeta2. Will
            be needed when computing the lateral force with combined slip.
\return Lateral force.
*/
template<typename TConfig>
typename TConfig::Float mfTireLateralForcePure(const typename TConfig::Float tanAlpha, const typename TConfig::Float fz, 
	const MFTireLateralForcePureParams<typename TConfig::Float>& params, const MFTireSharedParams<typename TConfig::Float>& sharedParams,
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams, typename TConfig::Float* fy0NoCamberNoTurnSlip,
	typename TConfig::Float& B_out, typename TConfig::Float& C_out, typename TConfig::Float& K_alpha_withEpsilon_out, 
	typename TConfig::Float& Sh_out, typename TConfig::Float& Sv_out,
	typename TConfig::Float& mu_zeta2_out)
{
	typedef typename TConfig::Float TFloat;

	// magic formula:
	// x:    tangens slip angle (tan(alpha))
	// y(x): lateral force

	//
	// magic formula: Sv; vertical shift
	//
	const TFloat lambdaMuDegressive = mfTireDegressiveFriction(volatileSharedParams.lambdaMu_lateral, sharedParams.aMu, sharedParams.aMuMinus1);
	const TFloat fz_x_lambdaMuDegressive = fz * lambdaMuDegressive;
	const TFloat Sv_term_noCamber_noTurnSlip = fz_x_lambdaMuDegressive  *  ( params.pV1 + (params.pV2 * volatileSharedParams.dFz) )  *  
		params.lambdaV;
	TFloat Sv = Sv_term_noCamber_noTurnSlip;
	if (TConfig::supportTurnSlip)
	{
		Sv *= sharedParams.zeta2;
	}
	TFloat Sv_term_camber;
	if (TConfig::supportCamber)
	{
		Sv_term_camber = fz_x_lambdaMuDegressive  *  ( params.pV3 + (params.pV4 * volatileSharedParams.dFz) )  *  volatileSharedParams.sinGamma  *  
			params.lambdaK_gamma;
		if (TConfig::supportTurnSlip)
		{
			Sv_term_camber *= sharedParams.zeta2;
		}

		Sv += Sv_term_camber;
	}
	// note: fz_x_lambdaMuDegressive and zeta2 are not pulled out because Sv_term_camber
    //       is needed for Sh computation and Sv_term_noCamber_noTurnSlip for lateralForceNoCamberNoTurnSlip
	Sv_out = Sv;

	//
	// cornering stiffness
	//
	const TFloat corneringStiffnessPeak_atNominalInflationPressure_noCamber_noTurnSlip = params.pK1  *  sharedParams.fz0Scaled  *  sharedParams.lambdaK_alpha;
	TFloat corneringStiffnessPeak = corneringStiffnessPeak_atNominalInflationPressure_noCamber_noTurnSlip;
	if (TConfig::supportCamber)
	{
		const TFloat corneringStiffnessPeak_camberFactor = TFloat(1.0) - (params.pK3 * volatileSharedParams.sinGammaAbs);
		corneringStiffnessPeak *= corneringStiffnessPeak_camberFactor;
	}
	if (TConfig::supportTurnSlip)
	{
		corneringStiffnessPeak *= params.zeta3;
	}
	TFloat corneringStiffnessPeak_inflationPressureFactor;
	if (TConfig::supportInflationPressure)
	{
		corneringStiffnessPeak_inflationPressureFactor = TFloat(1.0) + (params.pp1 * volatileSharedParams.dpi);
		corneringStiffnessPeak *= corneringStiffnessPeak_inflationPressureFactor;
	}
	
	const TFloat loadAtCorneringStiffnessPeak_atNominalInflationPressure_noCamber = params.pK2;
	TFloat loadAtCorneringStiffnessPeak = loadAtCorneringStiffnessPeak_atNominalInflationPressure_noCamber;
	if (TConfig::supportCamber)
	{
		const TFloat loadAtCorneringStiffnessPeak_term_camber = params.pK5 * volatileSharedParams.sinGammaSqr;
		loadAtCorneringStiffnessPeak += loadAtCorneringStiffnessPeak_term_camber;
	}
	TFloat loadAtCorneringStiffnessPeak_inflationPressureFactor;
	if (TConfig::supportInflationPressure)
	{
		loadAtCorneringStiffnessPeak_inflationPressureFactor = TFloat(1.0) + (params.pp2 * volatileSharedParams.dpi);
		loadAtCorneringStiffnessPeak *= loadAtCorneringStiffnessPeak_inflationPressureFactor;
	}
	
	const TFloat K_alpha = corneringStiffnessPeak  *  PxSin( params.pK4 * MF_ARCTAN(volatileSharedParams.fzNormalizedWithScale / loadAtCorneringStiffnessPeak) );
	const TFloat K_alpha_withEpsilon = K_alpha + params.epsilonK;
	K_alpha_withEpsilon_out = K_alpha_withEpsilon;

	//
	// magic formula: Sh; horizontal shift
	//
	const TFloat Sh_term_noCamber_noTurnSlip = ( params.pH1 + (params.pH2 * volatileSharedParams.dFz) )  *  params.lambdaH;
	TFloat Sh = Sh_term_noCamber_noTurnSlip;
	if (TConfig::supportCamber)
	{
		TFloat K_gamma = fz  *  ( params.pK6 + (params.pK7 * volatileSharedParams.dFz) )  *  params.lambdaK_gamma;
		if (TConfig::supportInflationPressure)
		{
			const TFloat K_gamma_inflationPressureFactor = TFloat(1.0) + (params.pp5 * volatileSharedParams.dpi);
			K_gamma *= K_gamma_inflationPressureFactor;
		}
		
		TFloat Sh_term_camber = ( (K_gamma * volatileSharedParams.sinGamma) - Sv_term_camber )  /  K_alpha_withEpsilon;
		if (TConfig::supportTurnSlip)
		{
			Sh_term_camber *= sharedParams.zeta0;
		}

		Sh += Sh_term_camber;
	}
	if (TConfig::supportTurnSlip)
	{
		Sh += params.zeta4 - TFloat(1.0);
	}
	Sh_out = Sh;

	//
	//
	//
	const TFloat tanAlphaShifted = tanAlpha + Sh;
	
	//
	// magic formula: C; shape factor
	//
	const TFloat C = params.pC1 * params.lambdaC;
	PX_ASSERT(C > TFloat(0.0));
	C_out = C;

	//
	// magic formula: D; peak value
	//
	TFloat mu_noCamber_noTurnSlip = (params.pD1 + (params.pD2 * volatileSharedParams.dFz))  *  volatileSharedParams.lambdaMu_lateral;
	if (TConfig::supportInflationPressure)
	{
		const TFloat mu_inflationPressureFactor = TFloat(1.0)  +  (params.pp3 * volatileSharedParams.dpi)  +  (params.pp4 * volatileSharedParams.dpiSqr);
		mu_noCamber_noTurnSlip *= mu_inflationPressureFactor;
	}
	TFloat mu = mu_noCamber_noTurnSlip;
	if (TConfig::supportCamber)
	{
		const TFloat mu_camberFactor = TFloat(1.0) - (params.pD3 * volatileSharedParams.sinGammaSqr);
		mu *= mu_camberFactor;
	}
	if (TConfig::supportTurnSlip)
	{
		mu *= sharedParams.zeta2;
	}
	mu_zeta2_out = mu;
	const TFloat D = mu * fz;
	PX_ASSERT(D >= TFloat(0.0));

	//
	// magic formula: E; curvature factor
	//
	const TFloat E_noCamber_noSignum = ( params.pE1 + (params.pE2*volatileSharedParams.dFz) )  *  params.lambdaE;
	TFloat E = E_noCamber_noSignum;
	if (TConfig::supportCamber)
	{
		const TFloat E_camberFactor = TFloat(1.0)  -  ( (params.pE3 + (params.pE4 * volatileSharedParams.sinGamma)) * mfSignum(tanAlphaShifted) )  +  (params.pE5 * volatileSharedParams.sinGammaSqr);
		E *= E_camberFactor;
	}
	else
	{
		const TFloat E_signumFactor = TFloat(1.0)  -  ( params.pE3 * mfSignum(tanAlphaShifted) );
		E *= E_signumFactor;
	}
	PX_ASSERT(E <= TFloat(1.0));

	//
	// magic formula: B; stiffness factor
	//
	const TFloat B = K_alpha  /  ((C*D) + params.epsilon);
	B_out = B;

	//
	// resulting force
	//
	const TFloat F = mfMagicFormulaSine(tanAlphaShifted, B, C, D, E) + Sv;

	if (fy0NoCamberNoTurnSlip)
	{
		if (TConfig::supportCamber || TConfig::supportTurnSlip)
		{
			const TFloat D_noCamber_noTurnSlip = mu_noCamber_noTurnSlip * fz;
			PX_ASSERT(D_noCamber_noTurnSlip >= TFloat(0.0));
			const TFloat tanAlphaShifted_noCamber_noTurnSlip = tanAlpha + Sh_term_noCamber_noTurnSlip;
		
			TFloat corneringStiffnessPeak_noCamber_noTurnSlip = corneringStiffnessPeak_atNominalInflationPressure_noCamber_noTurnSlip;
			TFloat loadAtCorneringStiffnessPeak_noCamber = loadAtCorneringStiffnessPeak_atNominalInflationPressure_noCamber;
			if (TConfig::supportInflationPressure)
			{
				corneringStiffnessPeak_noCamber_noTurnSlip *= corneringStiffnessPeak_inflationPressureFactor;
				loadAtCorneringStiffnessPeak_noCamber *= loadAtCorneringStiffnessPeak_inflationPressureFactor;
			}
			const TFloat K_alpha_noCamber_noTurnSlip = corneringStiffnessPeak_noCamber_noTurnSlip  *  PxSin( params.pK4 * MF_ARCTAN(volatileSharedParams.fzNormalizedWithScale / loadAtCorneringStiffnessPeak_noCamber) );

			const TFloat B_noCamber_noTurnSlip = K_alpha_noCamber_noTurnSlip  /  ((C*D_noCamber_noTurnSlip) + params.epsilon);
		
			const TFloat E_noCamber = E_noCamber_noSignum  *  ( TFloat(1.0) - (params.pE3 * mfSignum(tanAlphaShifted_noCamber_noTurnSlip)) );
			PX_ASSERT(E_noCamber <= TFloat(1.0));

			*fy0NoCamberNoTurnSlip = mfMagicFormulaSine(tanAlphaShifted_noCamber_noTurnSlip, B_noCamber_noTurnSlip, C, D_noCamber_noTurnSlip, E_noCamber) + 
				Sv_term_noCamber_noTurnSlip;
		}
		else
		{
			*fy0NoCamberNoTurnSlip = F;
		}
	}

	return F;
}

/**
\brief Lateral force with combined slip

\note Includes the interdependency with longitudinal force

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] kappa Longitudinal slip ratio.
\param[in] fz Vertical load (force). fz >= 0.
\param[in] fy0 The lateral force at pure lateral slip.
\param[in] mu_zeta2 The friction factor computed as part of the lateral force at pure lateral slip.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParams Model parameters not just used for lateral force
\param[in,out] fy0WeightNoCamberNoTurnSlip If not NULL, the weight factor for the lateral force
               at pure slip, discarding camber and turn slip (basically, if both were 0), will be written 
               to the pointer target.
\return Lateral force.

@see mfTireLateralForcePure()
*/
template<typename TConfig>
typename TConfig::Float mfTireLateralForceCombined(const typename TConfig::Float tanAlpha, const typename TConfig::Float kappa, 
	const typename TConfig::Float fz, const typename TConfig::Float fy0,
	const typename TConfig::Float mu_zeta2,
	const MFTireLateralForceCombinedParams<typename TConfig::Float>& params,
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams,
	typename TConfig::Float* fy0WeightNoCamberNoTurnSlip)
{
	typedef typename TConfig::Float TFloat;

	//
	// magic formula: Sh; horizontal shift
	//
	const TFloat Sh = params.rH1 + (params.rH2*volatileSharedParams.dFz);

	//
	// magic formula: Sv; vertical shift
	//
	TFloat DV = params.rV1 + (params.rV2*volatileSharedParams.dFz);
	if (TConfig::supportCamber)
	{
		const TFloat DV_term_camber = params.rV3 * volatileSharedParams.sinGamma;
		DV += DV_term_camber;
	}
	const TFloat DV_cosFactor = PxCos(MF_ARCTAN(params.rV4 * tanAlpha));
	DV *= mu_zeta2 * fz * DV_cosFactor;
	const TFloat Sv_sinFactor = PxSin(params.rV5 * MF_ARCTAN(params.rV6 * kappa));
	const TFloat Sv = DV * Sv_sinFactor * params.lambdaV;

	//
	//
	//
	const TFloat kappaShifted = kappa + Sh;

	//
	// magic formula: B; curvature factor
	//
	const TFloat B_term_noCamber = params.rB1;
	TFloat B = B_term_noCamber;
	if (TConfig::supportCamber)
	{
		const TFloat B_term_camber = params.rB4 * volatileSharedParams.sinGammaSqr;
		B += B_term_camber;
	}
	const TFloat B_cosFactor = PxCos(MF_ARCTAN(params.rB2 * (tanAlpha - params.rB3)));
	const TFloat B_cosFactor_lambdaKappa = B_cosFactor * params.lambdaKappa;
	B *= B_cosFactor_lambdaKappa;
	PX_ASSERT(B > TFloat(0.0));
	
	//
	// magic formula: C; shape factor
	//
	const TFloat C = params.rC1;

	//
	// magic formula: E;
	//
	const TFloat E = params.rE1 + (params.rE2*volatileSharedParams.dFz);
	PX_ASSERT(E <= TFloat(1.0));

	//
	// resulting force
	//
	const TFloat G0 = mfMagicFormulaCosineNoD(Sh, B, C, E);
	PX_ASSERT(G0 > TFloat(0.0));
	TFloat G = mfMagicFormulaCosineNoD(kappaShifted, B, C, E) / G0;
	if (G < TFloat(0.0))
	{
		// at very low velocity (or starting from standstill), the longitudinal slip ratio can get
		// large which can result in negative values
		G = TFloat(0.0);
	}
	const TFloat F = (G * fy0) + Sv;

	if (fy0WeightNoCamberNoTurnSlip)
	{
		if (TConfig::supportCamber || TConfig::supportTurnSlip)
		{
			const TFloat B_noCamber = B_term_noCamber * B_cosFactor_lambdaKappa;
			PX_ASSERT(B_noCamber > TFloat(0.0));

			const TFloat G0_noCamber = mfMagicFormulaCosineNoD(Sh, B_noCamber, C, E);
			PX_ASSERT(G0_noCamber > TFloat(0.0));
			TFloat G_noCamber = mfMagicFormulaCosineNoD(kappaShifted, B_noCamber, C, E) / G0_noCamber;
			if (G_noCamber < TFloat(0.0))
				G_noCamber = TFloat(0.0);

			*fy0WeightNoCamberNoTurnSlip = G_noCamber;
		}
		else
		{
			*fy0WeightNoCamberNoTurnSlip = G;
		}
	}

	return F;
}

template<typename TConfig>
void mfTireComputeAligningTorquePneumaticTrailIntermediateParams(const typename TConfig::Float tanAlpha,
	const MFTireAligningTorquePurePneumaticTrailParams<typename TConfig::Float>& params, 
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque,
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams,
	typename TConfig::Float& tanAlphaShifted_out, typename TConfig::Float& B_out, typename TConfig::Float& C_out, 
	typename TConfig::Float& D_out, typename TConfig::Float& E_out)
{
	typedef typename TConfig::Float TFloat;

	//
	// magic formula: Sh; horizontal shift
	//
	TFloat Sh = params.qH1 + (params.qH2 * volatileSharedParams.dFz);
	if (TConfig::supportCamber)
	{
		const TFloat Sh_term_camber = ( params.qH3 + (params.qH4 * volatileSharedParams.dFz) ) * volatileSharedParams.sinGamma;
		Sh += Sh_term_camber;
	}

	//
	//
	//
	const TFloat tanAlphaShifted = tanAlpha + Sh;
	tanAlphaShifted_out = tanAlphaShifted;

	//
	// magic formula: B; curvature factor
	//
	TFloat B = ( params.qB1 + (params.qB2 * volatileSharedParams.dFz) + (params.qB3 * volatileSharedParams.dFzSqr) )  *  
		(sharedParams.lambdaK_alpha / volatileSharedParams.lambdaMu_lateral);
	if (TConfig::supportCamber)
	{
		const TFloat B_camberFactor = TFloat(1.0) + (params.qB5 * volatileSharedParams.sinGammaAbs) + (params.qB6 * volatileSharedParams.sinGammaSqr);
		B *= B_camberFactor;
	}
	PX_ASSERT(B > TFloat(0.0));
	B_out = B;
	
	//
	// magic formula: C; shape factor
	//
	const TFloat C = params.qC1;
	PX_ASSERT(C > TFloat(0.0));
	C_out = C;

	//
	// magic formula: D; peak value
	//
	TFloat D = volatileSharedParams.fzNormalizedWithScale  *  sharedParams.r0  *
		(params.qD1 + (params.qD2 * volatileSharedParams.dFz))  *  params.lambdaT  *  volatileSharedParamsAligningTorque.signumVc_longitudinal;
	if (TConfig::supportInflationPressure)
	{
		const TFloat D_inflationPressureFactor = TFloat(1.0) - (params.pp1 * volatileSharedParams.dpi);
		D *= D_inflationPressureFactor;
	}
	if (TConfig::supportCamber)
	{
		const TFloat D_camberFactor = TFloat(1.0) + (params.qD3 * volatileSharedParams.sinGammaAbs) + (params.qD4 * volatileSharedParams.sinGammaSqr);
		D *= D_camberFactor;
	}
	if (TConfig::supportTurnSlip)
	{
		D *= params.zeta5;
	}
	D_out = D;

	//
	// magic formula: E;
	//
	TFloat E = params.qE1 + (params.qE2*volatileSharedParams.dFz) + (params.qE3*volatileSharedParams.dFzSqr);
	TFloat E_arctanFactor = params.qE4;
	if (TConfig::supportCamber)
	{
		const TFloat E_arctanFactor_term_camber = params.qE5 * volatileSharedParams.sinGamma;
		E_arctanFactor += E_arctanFactor_term_camber;
	}
	E_arctanFactor = TFloat(1.0)  +  ( E_arctanFactor * (TFloat(2.0) / TFloat(PxPi)) * MF_ARCTAN(B*C*tanAlphaShifted) );
	E *= E_arctanFactor;
	PX_ASSERT(E <= TFloat(1.0));
	E_out = E;
}

/**
\brief Aligning torque at pure lateral/side slip (pneumatic trail term)

The lateral/side forces act a small distance behind the center of the contact patch. This distance is called 
the pneumatic trail. The pneumatic trail decreases with increasing slip angle. The pneumatic trail increases 
with vertical load. The pneumatic trail causes a torque that turns the wheel away from the direction of turn.

\note Ignores interdependency with longitudinal force (in other words, assumes no longitudinal slip, that is, 
longitudinal slip ratio of 0 [kappa = 0])

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] fy0NoCamberNoTurnSlip Lateral force at pure lateral slip and camber angle and turn slip equals 0.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParamsAligningTorque Model parameters used for aligning torque (but not just pneumatic trail term)
\param[in] sharedParams Model parameters not just used for aligning torque
\param[in] volatileSharedParams Model parameters not just used for aligning torque
\return Aligning torque (pneumatic trail term).
*/
template<typename TConfig>
typename TConfig::Float mfTireAligningTorquePurePneumaticTrail(const typename TConfig::Float tanAlpha, 
	const typename TConfig::Float fy0NoCamberNoTurnSlip,
	const MFTireAligningTorquePurePneumaticTrailParams<typename TConfig::Float>& params, 
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque,
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	// magic formula:
	// x:    tangens slip angle (tan(alpha))
	// y(x): pneumatic trail

	TFloat tanAlphaShifted, B, C, D, E;

	mfTireComputeAligningTorquePneumaticTrailIntermediateParams<TConfig>(tanAlpha,
		params, volatileSharedParamsAligningTorque, sharedParams, volatileSharedParams,
		tanAlphaShifted, B, C, D, E);

	//
	// pneumatic trail
	//
	const TFloat t = mfMagicFormulaCosine(tanAlphaShifted, B, C, D, E) * volatileSharedParamsAligningTorque.cosAlpha;

	//
	// resulting torque
	//
	const TFloat Mz = -t * fy0NoCamberNoTurnSlip;

	return Mz;
}

/**
\brief Aligning torque with combined slip (pneumatic trail term)

\note Includes the interdependency between longitudinal/lateral force

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] kappaScaledSquared Longitudinal slip ratio scaled by the ratio of longitudinalStiffness and
           lateralStiffness and then squared
\param[in] fy0NoCamberNoTurnSlip Lateral force at pure lateral slip and camber angle and turn slip equals 0.
\param[in] fy0NoCamberNoTurnSlipWeight The weight factor for the lateral force at pure slip, discarding 
           camber and turn slip (basically, if both were 0).
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParamsAligningTorque Model parameters used for aligning torque (but not just pneumatic trail term)
\param[in] sharedParams Model parameters not just used for aligning torque
\param[in] volatileSharedParams Model parameters not just used for aligning torque
\return Aligning torque (pneumatic trail term).

@see mfTireAligningTorquePurePneumaticTrail
*/
template<typename TConfig>
typename TConfig::Float mfTireAligningTorqueCombinedPneumaticTrail(const typename TConfig::Float tanAlpha, const typename TConfig::Float kappaScaledSquared,
	const typename TConfig::Float fy0NoCamberNoTurnSlip, const typename TConfig::Float fy0NoCamberNoTurnSlipWeight,
	const MFTireAligningTorquePurePneumaticTrailParams<typename TConfig::Float>& params, 
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque,
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	// magic formula:
	// x:    tangens slip angle (tan(alpha))
	// y(x): pneumatic trail

	TFloat tanAlphaShifted, B, C, D, E;

	mfTireComputeAligningTorquePneumaticTrailIntermediateParams<TConfig>(tanAlpha,
		params, volatileSharedParamsAligningTorque, sharedParams, volatileSharedParams,
		tanAlphaShifted, B, C, D, E);

	const TFloat tanAlphaShiftedCombined = PxSqrt( tanAlphaShifted*tanAlphaShifted + kappaScaledSquared ) * mfSignum(tanAlphaShifted);

	const TFloat fyNoCamberNoTurnSlip = fy0NoCamberNoTurnSlipWeight * fy0NoCamberNoTurnSlip;

	//
	// pneumatic trail
	//
	const TFloat t = mfMagicFormulaCosine(tanAlphaShiftedCombined, B, C, D, E) * volatileSharedParamsAligningTorque.cosAlpha;

	//
	// resulting torque
	//
	const TFloat Mz = -t * fyNoCamberNoTurnSlip;

	return Mz;
}

template<typename TConfig>
void mfTireComputeAligningTorqueResidualTorqueIntermediateParams(const typename TConfig::Float tanAlpha, const typename TConfig::Float fz,
	const typename TConfig::Float B_lateral, const typename TConfig::Float C_lateral, const typename TConfig::Float K_alpha_withEpsilon,
	const typename TConfig::Float Sh_lateral, const typename TConfig::Float Sv_lateral,
	const MFTireAligningTorquePureResidualTorqueParams<typename TConfig::Float>& params, 
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque, 
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams,
	typename TConfig::Float& tanAlphaShifted_out, typename TConfig::Float& B_out, typename TConfig::Float& C_out, typename TConfig::Float& D_out)
{
	typedef typename TConfig::Float TFloat;

	//
	// magic formula: Sh; horizontal shift
	//
	const TFloat Sh = Sh_lateral + ( Sv_lateral / K_alpha_withEpsilon );

	//
	//
	//
	tanAlphaShifted_out = tanAlpha + Sh;

	//
	// magic formula: B; curvature factor
	//
	TFloat B = ( params.qB9 * (sharedParams.lambdaK_alpha / volatileSharedParams.lambdaMu_lateral) )  +
	           ( params.qB10 * B_lateral * C_lateral );
	if (TConfig::supportTurnSlip)
	{
		B *= params.zeta6;
	}
	B_out = B;
	
	//
	// magic formula: C; shape factor
	//
	if (TConfig::supportTurnSlip)
	{
		C_out = params.zeta7;
	}
	else
	{
		C_out = TFloat(1.0);
	}

	//
	// magic formula: D; peak value
	//
	TFloat D = (params.qD6 + (params.qD7 * volatileSharedParams.dFz)) * params.lambdaR;
	if (TConfig::supportTurnSlip)
	{
		D *= sharedParams.zeta2;
	}
	if (TConfig::supportCamber)
	{
		TFloat D_term_camber = params.qD8 + (params.qD9 * volatileSharedParams.dFz);
		if (TConfig::supportInflationPressure)
		{
			const TFloat D_term_camber_inflationPressureFactor = TFloat(1.0) + (params.pp2 * volatileSharedParams.dpi);
			D_term_camber *= D_term_camber_inflationPressureFactor;
		}
		D_term_camber += ( params.qD10 + (params.qD11 * volatileSharedParams.dFz) ) * volatileSharedParams.sinGammaAbs;
		D_term_camber *= volatileSharedParams.sinGamma * params.lambdaK_gamma;
		if (TConfig::supportTurnSlip)
		{
			D_term_camber *= sharedParams.zeta0;
		}

		D += D_term_camber;
	}
	D *= fz * sharedParams.r0 * volatileSharedParams.lambdaMu_lateral *
		volatileSharedParamsAligningTorque.signumVc_longitudinal * volatileSharedParamsAligningTorque.cosAlpha;
	if (TConfig::supportTurnSlip)
	{
		D += params.zeta8 - TFloat(1.0);
	}
	D_out = D;
}

/**
\brief Aligning torque at pure lateral/side slip (residual torque term)

Residual torque is produced by assymetries of the tire and the asymmetrical shape and pressure distribution 
of the contact patch etc.

\note Ignores interdependency with longitudinal force (in other words, assumes no longitudinal slip, that is, 
longitudinal slip ratio of 0 [kappa = 0])

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] fz Vertical load (force). fz >= 0.
\param[in] B_lateral B factor in magic formula for lateral force under pure lateral slip.
\param[in] C_lateral C factor in magic formula for lateral force under pure lateral slip.
\param[in] K_alpha_withEpsilon Cornering stiffness in magic formula for lateral force under pure lateral slip
           (with a small epsilon value added to avoid division by 0).
\param[in] Sh_lateral Horizontal shift in magic formula for lateral force under pure lateral slip.
\param[in] Sv_lateral Vertical shift in magic formula for lateral force under pure lateral slip.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParamsAligningTorque Model parameters used for aligning torque (but not just residual torque term)
\param[in] sharedParams Model parameters not just used for aligning torque
\param[in] volatileSharedParams Model parameters not just used for aligning torque
\return Aligning torque (residual torque term).
*/
template<typename TConfig>
typename TConfig::Float mfTireAligningTorquePureResidualTorque(const typename TConfig::Float tanAlpha, const typename TConfig::Float fz,
	const typename TConfig::Float B_lateral, const typename TConfig::Float C_lateral, const typename TConfig::Float K_alpha_withEpsilon,
	const typename TConfig::Float Sh_lateral, const typename TConfig::Float Sv_lateral,
	const MFTireAligningTorquePureResidualTorqueParams<typename TConfig::Float>& params, 
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque, 
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	// magic formula:
	// x:    tangens slip angle (tan(alpha))
	// y(x): residual torque

	TFloat tanAlphaShifted, B, C, D;

	mfTireComputeAligningTorqueResidualTorqueIntermediateParams<TConfig>(tanAlpha, fz, B_lateral, C_lateral, K_alpha_withEpsilon,
		Sh_lateral, Sv_lateral, params, volatileSharedParamsAligningTorque, sharedParams, volatileSharedParams,
		tanAlphaShifted, B, C, D);

	//
	// residual torque
	//
	const TFloat Mz = mfMagicFormulaCosineNoE(tanAlphaShifted, B, C, D) * volatileSharedParamsAligningTorque.cosAlpha;

	return Mz;
}

/**
\brief Aligning torque with combined slip (residual torque term)

\note Includes the interdependency between longitudinal/lateral force

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] kappaScaledSquared Longitudinal slip ratio scaled by the ratio of longitudinalStiffness and
           lateralStiffness and then squared
\param[in] fz Vertical load (force). fz >= 0.
\param[in] B_lateral B factor in magic formula for lateral force under pure lateral slip.
\param[in] C_lateral C factor in magic formula for lateral force under pure lateral slip.
\param[in] K_alpha_withEpsilon Cornering stiffness in magic formula for lateral force under pure lateral slip
           (with a small epsilon value added to avoid division by 0).
\param[in] Sh_lateral Horizontal shift in magic formula for lateral force under pure lateral slip.
\param[in] Sv_lateral Vertical shift in magic formula for lateral force under pure lateral slip.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParamsAligningTorque Model parameters used for aligning torque (but not just residual torque term)
\param[in] sharedParams Model parameters not just used for aligning torque
\param[in] volatileSharedParams Model parameters not just used for aligning torque
\return Aligning torque (residual torque term).

@see mfTireAligningTorquePureResidualTorque()
*/
template<typename TConfig>
typename TConfig::Float mfTireAligningTorqueCombinedResidualTorque(const typename TConfig::Float tanAlpha, const typename TConfig::Float kappaScaledSquared, 
	const typename TConfig::Float fz,
	const typename TConfig::Float B_lateral, const typename TConfig::Float C_lateral, const typename TConfig::Float K_alpha_withEpsilon,
	const typename TConfig::Float Sh_lateral, const typename TConfig::Float Sv_lateral,
	const MFTireAligningTorquePureResidualTorqueParams<typename TConfig::Float>& params, 
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque, 
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, 
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	// magic formula:
	// x:    tangens slip angle (tan(alpha))
	// y(x): residual torque

	TFloat tanAlphaShifted, B, C, D;

	mfTireComputeAligningTorqueResidualTorqueIntermediateParams<TConfig>(tanAlpha, fz, B_lateral, C_lateral, K_alpha_withEpsilon,
		Sh_lateral, Sv_lateral, params, volatileSharedParamsAligningTorque, sharedParams, volatileSharedParams,
		tanAlphaShifted, B, C, D);

	const TFloat tanAlphaShiftedCombined = PxSqrt( tanAlphaShifted*tanAlphaShifted + kappaScaledSquared ) * mfSignum(tanAlphaShifted);

	//
	// residual torque
	//
	const TFloat Mz = mfMagicFormulaCosineNoE(tanAlphaShiftedCombined, B, C, D) * volatileSharedParamsAligningTorque.cosAlpha;

	return Mz;
}

/**
\brief Aligning torque with combined longitudinal/lateral slip

\note Includes the interdependency between longitudinal/lateral force

\param[in] tanAlpha Tangens of slip angle (multiplied with sign of longitudinal velocity at contact patch,
           i.e., -Vcy / |Vcx|, Vcy/Vcx: lateral/longitudinal velocity at contact patch)
\param[in] kappa Longitudinal slip ratio.
\param[in] fz Vertical load (force). fz >= 0.
\param[in] fx Longitudinal force with combined slip.
\param[in] fy Lateral force with combined slip.
\param[in] fy0NoCamberNoTurnSlip Lateral force at pure lateral slip and camber angle and turn slip equals 0.
\param[in] fy0NoCamberNoTurnSlipWeight The weight factor for the lateral force at pure slip, discarding 
           camber and turn slip (basically, if both were 0).
\param[in] K_kappa Longitudinal slip stiffness in magic formula for longitudinal force under pure longitudinal slip.
\param[in] K_alpha_withEpsilon Cornering stiffness in magic formula for lateral force under pure lateral slip
           (with a small epsilon value added to avoid division by 0).
\param[in] B_lateral B factor in magic formula for lateral force under pure lateral slip.
\param[in] C_lateral C factor in magic formula for lateral force under pure lateral slip.
\param[in] Sh_lateral Horizontal shift in magic formula for lateral force under pure lateral slip.
\param[in] Sv_lateral Vertical shift in magic formula for lateral force under pure lateral slip.
\param[in] pneumaticTrailParams The nondimensional model parameters and user scaling factors used for the pneumatic
           trail term of aligning torque.
\param[in] residualTorqueParams The nondimensional model parameters and user scaling factors used for the residual
           torque term of aligning torque.
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParamsAligningTorque Model parameters used for aligning torque.
\param[in] sharedParams Model parameters not just used for aligning torque.
\param[in] volatileSharedParams Model parameters not just used for aligning torque.
\return Aligning torque.
*/
template<typename TConfig>
typename TConfig::Float mfTireAligningTorqueCombined(const typename TConfig::Float tanAlpha, const typename TConfig::Float kappa, 
	const typename TConfig::Float fz, const typename TConfig::Float fx, const typename TConfig::Float fy, 
	const typename TConfig::Float fy0NoCamberNoTurnSlip, const typename TConfig::Float fy0NoCamberNoTurnSlipWeight,
	const typename TConfig::Float K_kappa, const typename TConfig::Float K_alpha_withEpsilon,
	const typename TConfig::Float B_lateral, const typename TConfig::Float C_lateral,
	const typename TConfig::Float Sh_lateral, const typename TConfig::Float Sv_lateral,
	const MFTireAligningTorquePurePneumaticTrailParams<typename TConfig::Float>& pneumaticTrailParams, 
	const MFTireAligningTorquePureResidualTorqueParams<typename TConfig::Float>& residualTorqueParams, 
	const MFTireAligningTorqueCombinedParams<typename TConfig::Float>& params,
	const MFTireAligningTorqueVolatileSharedParams<typename TConfig::Float>& volatileSharedParamsAligningTorque, 
	const MFTireSharedParams<typename TConfig::Float>& sharedParams, 
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	const TFloat kappaScaled = kappa * (K_kappa / K_alpha_withEpsilon);
	const TFloat kappaScaledSquared = kappaScaled * kappaScaled;

	const TFloat Mzp = mfTireAligningTorqueCombinedPneumaticTrail<TConfig>(tanAlpha, kappaScaledSquared,
		fy0NoCamberNoTurnSlip, fy0NoCamberNoTurnSlipWeight,
		pneumaticTrailParams, volatileSharedParamsAligningTorque,
		sharedParams, volatileSharedParams);

	const TFloat Mzr = mfTireAligningTorqueCombinedResidualTorque<TConfig>(tanAlpha, kappaScaledSquared, fz,
		B_lateral, C_lateral, K_alpha_withEpsilon, Sh_lateral, Sv_lateral,
		residualTorqueParams, volatileSharedParamsAligningTorque,
		sharedParams, volatileSharedParams);

	TFloat s = params.sS1 + ( params.sS2 * (fy * sharedParams.recipFz0Scaled) );
	if (TConfig::supportCamber)
	{
		const TFloat s_term_camber = ( params.sS3 + (params.sS4 * volatileSharedParams.dFz) ) * volatileSharedParams.sinGamma;
		s += s_term_camber;
	}
	s *= sharedParams.r0 * params.lambdaS;

	const TFloat Mz = Mzp + Mzr + (s * fx);

	return Mz;
}

/**
\brief Overturning couple

\param[in] fz Vertical load (force). fz >= 0.
\param[in] r0 Tire radius under no load.
\param[in] fyNormalized Normalized lateral force (fy/fz0, fz0=nominal vertical load).
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParams Model parameters not just used for overturning couple
\return Overturning couple (torque).
*/
template<typename TConfig>
typename TConfig::Float mfTireOverturningCouple(const typename TConfig::Float fz, const typename TConfig::Float r0, 
	const typename TConfig::Float fyNormalized, 
	const MFTireOverturningCoupleParams<typename TConfig::Float>& params, 
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	TFloat Mx = params.qS1 * params.lambdaVM;
	if (TConfig::supportCamber)
	{
		TFloat Mx_term_camber = params.qS2 * volatileSharedParams.gamma;
		if (TConfig::supportInflationPressure)
		{
			const TFloat Mx_term_camber_inflationPressureFactor = TFloat(1.0) + (params.ppM1 * volatileSharedParams.dpi);
			Mx_term_camber *= Mx_term_camber_inflationPressureFactor;
		}

		Mx -= Mx_term_camber;
	}
	const TFloat Mx_term_lateral = params.qS3 * fyNormalized;
	Mx += Mx_term_lateral;
	const TFloat atanLoad = MF_ARCTAN(params.qS6 * volatileSharedParams.fzNormalized);
	const TFloat Mx_cosFactor = PxCos(params.qS5 * atanLoad * atanLoad);
	TFloat Mx_sinFactor = params.qS8 * MF_ARCTAN(params.qS9 * fyNormalized);
	if (TConfig::supportCamber)
	{
		const TFloat Mx_sinFactor_term_camber = params.qS7 * volatileSharedParams.gamma;
		Mx_sinFactor += Mx_sinFactor_term_camber;
	}
	Mx_sinFactor = PxSin(Mx_sinFactor);
	Mx += params.qS4 * Mx_cosFactor * Mx_sinFactor;
	if (TConfig::supportCamber)
	{
		const TFloat Mx_term_atan = params.qS10 * MF_ARCTAN(params.qS11 * volatileSharedParams.fzNormalized) * volatileSharedParams.gamma;
		Mx += Mx_term_atan;
	}
	Mx *= fz * r0 * params.lambdaM;

	return Mx;
}

/**
\brief Rolling resistance moment

Deformation of the wheel and the road surface usually results in a loss of energy as some of the deformation
remains (not fully elastic). Furthermore, slippage between wheel and surface dissipates energy too. The 
rolling resistance moment models the force that will make a free rolling wheel stop. Note that similar to
sliding friction, rolling resistance is often expressed as a coefficient times the normal force (but is
generally much smaller than the coefficient of sliding friction).

\param[in] fz Vertical load (force). fz >= 0.
\param[in] r0 Tire radius under no load.
\param[in] fxNormalized Normalized longitudinal force (fx/fz0, fz0=nominal vertical load).
\param[in] vxNormalized Normalized longitudinal velocity (vx/v0, v0=reference velocity).
\param[in] piNormalized Normalized tire inflation pressure (pi/pi0, pi0=nominal tire inflation pressure).
\param[in] params The nondimensional model parameters and user scaling factors.
\param[in] volatileSharedParams Model parameters not just used for rolling resistance moment
\return Rolling resistance moment.
*/
template<typename TConfig>
typename TConfig::Float mfTireRollingResistanceMoment(const typename TConfig::Float fz, const typename TConfig::Float r0, 
	const typename TConfig::Float fxNormalized, const typename TConfig::Float vxNormalized, const typename TConfig::Float piNormalized, 
	const MFTireRollingResistanceMomentParams<typename TConfig::Float>& params, 
	const MFTireVolatileSharedParams<typename TConfig::Float>& volatileSharedParams)
{
	typedef typename TConfig::Float TFloat;

	//
	// rolling resistance force Fr:
	//
	// Fr = cr * Fn
	//
	// cr: rolling resistance coefficient
	// Fn: normal force
	//
	// Given a drive/break moment Md, part of Md will flow into the longitudinal force and a part into the
	// rolling resistance moment:
	//
	// Md = My + Fx*Rl
	//
	// My: rolling resistance moment My (with respect to contact patch)
	// Fx: longitudinal force at contact patch
	// Rl: loaded wheel radius
	//
	// The rolling resistance moment My can be derived from this:
	//
	// My = Fr*Re + Fx*(Re-Rl)
	//    = (Fr+Fx)*Re - Fx*Rl
	//
	// Re: effective rolling radius of wheel
	//

	const TFloat absVxNormalized = PxAbs(vxNormalized);
	const TFloat vxNormalizedSqr = absVxNormalized * absVxNormalized;
	const TFloat vxNormalizedPow4 = vxNormalizedSqr * vxNormalizedSqr;
	TFloat My = params.qS1 + (params.qS2 * fxNormalized) + (params.qS3 * absVxNormalized) +
		(params.qS4 * vxNormalizedPow4);
	if (TConfig::supportCamber)
	{
		const TFloat My_term_camber = ( params.qS5 + (params.qS6 * volatileSharedParams.fzNormalized) )  *  volatileSharedParams.gammaSqr;
		My += My_term_camber;
	}
	if (volatileSharedParams.fzNormalized > TFloat(0.0))
	{
		TFloat My_powFactor = mfPow(volatileSharedParams.fzNormalized, params.qS7);

		if (TConfig::supportInflationPressure)
		{
			if (piNormalized > TFloat(0.0))
			{
				const TFloat My_powFactor_inflationPressureFactor = mfPow(piNormalized, params.qS8);
				My_powFactor *= My_powFactor_inflationPressureFactor;
			}
			else
				return TFloat(0.0);
		}

		My *= My_powFactor;
	}
	else
		return TFloat(0.0);

	My *= fz * r0 * params.lambdaM;

	return My;
}

/**
\brief Free radius of rotating tire

The free tire radius of the rotating tire when taking centrifugal growth into account.

\param[in] r0 Tire radius under no load.
\param[in] omega Tire angular velocity.
\param[in] v0 Reference velocity (for normalization. Usually the speed at which measurements were taken, often 
           16.7m/s, i.e., 60km/h)
\param[in] params The nondimensional model parameters.
\param[out] normalizedVelR0 Normalized longitudinal velocity due to rotation at unloaded radius (omega * r0 / v0).
\return Free radius of rotating tire.
*/
template<typename TFloat>
PX_FORCE_INLINE TFloat mfTireFreeRotatingRadius(const TFloat r0, const TFloat omega, const TFloat v0, 
	const MFTireFreeRotatingRadiusParams<TFloat>& params, TFloat& normalizedVelR0)
{
	const TFloat normalizedVel = (r0 * omega) / v0;
	normalizedVelR0 = normalizedVel;
	const TFloat normalizedVelSqr = normalizedVel * normalizedVel;
	const TFloat rOmega = r0 * ( params.qre0 + (params.qV1*normalizedVelSqr) );

	return rOmega;
}

/**
\brief Vertical tire stiffness

\param[in] dpi Normalized inflation pressure change: (pi - pi0) / pi0 (with inflation pressure pi and nominal
           inflation pressure pi0).
\param[in] params The nondimensional and derived model parameters.
\return Vertical tire stiffness.
*/
template<typename TConfig>
PX_FORCE_INLINE typename TConfig::Float mfTireVerticalStiffness(const typename TConfig::Float dpi, 
	const MFTireVerticalStiffnessParams<typename TConfig::Float>& params)
{
	typedef typename TConfig::Float TFloat;

	TFloat c = params.c0;
	if (TConfig::supportInflationPressure)
	{
		const TFloat c_inflationPressureFactor = TFloat(1.0) + (params.ppF1 * dpi);
		c *= c_inflationPressureFactor;
	}

	return c;
}

/**
\brief Effective rolling radius

The effective rolling radius (re) is the radius that matches the angular velocity (omega) and the velocity at
the contact patch (V) under free rolling conditions (no drive/break torque), i.e., re = V / omega. This radius
is bounded by the free tire radius of the rotating tire on one end and the loaded tire radius on the other end.
The effective rolling radius changes with the load (fast for low values of load but only marginally at higher
load values).

\param[in] rOmega The free tire radius of the rotating tire when taking centrifugal growth into account.
\param[in] cz Vertical stiffness of the tire.
\param[in] fz0 Nominal vertical load (force). fz0 >= 0.
\param[in] fzNormalized Normalized vertical load: fz / fz0. fzNormalized >= 0.
\param[in] params The nondimensional model parameters.
\return Effective rolling radius.
*/
template<typename TFloat>
PX_FORCE_INLINE TFloat mfTireEffectiveRollingRadius(const TFloat rOmega, const TFloat cz,
	const TFloat fz0, const TFloat fzNormalized,
	const MFTireEffectiveRollingRadiusParams<TFloat>& params)
{
	//
	// effective rolling radius
	//
	const TFloat B_term = params.Breff * fzNormalized;
	const TFloat D_term = params.Dreff * MF_ARCTAN(B_term);
	const TFloat F_term = (params.Freff * fzNormalized) + D_term;
	const TFloat re = rOmega - (fz0 * (F_term / cz));

	return re;
}

/**
\brief Normal load of the tire

\param[in] fz0 Nominal vertical load (force). fz0 >= 0.
\param[in] normalizedVelR0 Normalized longitudinal velocity due to rotation at unloaded radius
           (omega * r0 / v0, omega: tire angular velocity, r0: unloaded radius, v0: reference velocity
		   for normalization)
\param[in] r0 Tire radius under no load.
\param[in] fxNormalized Normalized longitudinal force (fx/fz0).
\param[in] fyNormalized Normalized lateral force (fy/fz0).
\param[in] gammaSqr Camber angle squared.
\param[in] deflection Tire normal deflection (difference between free rolling radius and loaded radius).
\param[in] dpi Normalized inflation pressure change: (pi - pi0) / pi0 (with inflation pressure pi and nominal
           inflation pressure pi0).
\param[in] params The nondimensional model parameters.
\param[in] verticalStiffnessParams The nondimensional model parameters for vertical tire stiffness.
\return Normal load (force) of the tire.
*/
template<typename TConfig>
typename TConfig::Float mfTireNormalLoad(const typename TConfig::Float fz0, const typename TConfig::Float normalizedVelR0, const typename TConfig::Float r0, 
	const typename TConfig::Float fxNormalized, const typename TConfig::Float fyNormalized, 
	const typename TConfig::Float gammaSqr, const typename TConfig::Float deflection, const typename TConfig::Float dpi, 
	const MFTireNormalLoadParams<typename TConfig::Float>& params, 
	const MFTireVerticalStiffnessParams<typename TConfig::Float>& verticalStiffnessParams)
{
	typedef typename TConfig::Float TFloat;

	const TFloat F_term_vel = params.qV2 * PxAbs(normalizedVelR0);
	const TFloat F_term_long = params.qFc_longitudinal * fxNormalized;
	const TFloat F_term_longSqr = F_term_long * F_term_long;
	const TFloat F_term_lat = params.qFc_lateral * fyNormalized;
	const TFloat F_term_latSqr = F_term_lat * F_term_lat;
	TFloat F_term_deflection = verticalStiffnessParams.qF1;
	if (TConfig::supportCamber)
	{
		const TFloat F_term_deflection_term_camber = params.qF3 * gammaSqr;
		F_term_deflection += F_term_deflection_term_camber;
	}
	const TFloat normalizedDeflection = deflection / r0;
	const TFloat F_deflectionFactor = ( F_term_deflection + (verticalStiffnessParams.qF2 * normalizedDeflection) )  *  normalizedDeflection;
	TFloat F = (TFloat(1.0) + F_term_vel - F_term_longSqr - F_term_latSqr) * F_deflectionFactor;
	if (TConfig::supportInflationPressure)
	{
		const TFloat F_inflationPressureFactor = TFloat(1.0) + (verticalStiffnessParams.ppF1 * dpi);
		F *= F_inflationPressureFactor;
	}
	F *= fz0;

	return F;
}

template<typename TConfig>
PX_FORCE_INLINE typename TConfig::Float mfTireComputeLoad(
	const MFTireDataT<typename TConfig::Float>& tireData, 
	const typename TConfig::Float wheelOmega, 
	const typename TConfig::Float tireDeflection, 
	const typename TConfig::Float gammaSqr,
	const typename TConfig::Float longTireForce, const typename TConfig::Float latTireForce,
	const typename TConfig::Float tireInflationPressure)
{
	typedef typename TConfig::Float TFloat;

	PX_ASSERT(tireDeflection >= TFloat(0.0));

	const TFloat longTireForceNormalized = longTireForce * tireData.sharedParams.recipFz0;
	const TFloat latTireForceNormalized = latTireForce * tireData.sharedParams.recipFz0;

	const TFloat normalizedVelR0 = (wheelOmega * tireData.sharedParams.r0) / tireData.sharedParams.v0;

	TFloat dpi;
	if (TConfig::supportInflationPressure)
		dpi = (tireInflationPressure - tireData.sharedParams.pi0) / tireData.sharedParams.pi0;
	else
		dpi = TFloat(0.0);

	TFloat tireLoad = mfTireNormalLoad<TConfig>(tireData.sharedParams.fz0, normalizedVelR0, tireData.sharedParams.r0,
		longTireForceNormalized, latTireForceNormalized,
		gammaSqr, tireDeflection, dpi,
		tireData.normalLoadParams, tireData.verticalStiffnessParams);

	return tireLoad;
}

template<typename TConfig>
PX_FORCE_INLINE void mfTireComputeSlip(
	const MFTireDataT<typename TConfig::Float>& tireData, 
	const typename TConfig::Float longVelocity, const typename TConfig::Float latVelocity,
	const typename TConfig::Float wheelOmega, 
	const typename TConfig::Float tireLoad,
	const typename TConfig::Float tireInflationPressure,
	typename TConfig::Float& longSlip, 
	typename TConfig::Float& tanLatSlip, 
	typename TConfig::Float& effectiveRollingRadius)
{
	//
	// see mfTireComputeForce() for the coordinate system used
	//

	typedef typename TConfig::Float TFloat;

	TFloat dpi;
	if (TConfig::supportInflationPressure)
		dpi = (tireInflationPressure - tireData.sharedParams.pi0) / tireData.sharedParams.pi0;
	else
		dpi = TFloat(0.0);

	const TFloat fzNormalized = tireLoad * tireData.sharedParams.recipFz0;

	TFloat normalizedVelR0;
	PX_UNUSED(normalizedVelR0);
	const TFloat rOmega = mfTireFreeRotatingRadius(tireData.sharedParams.r0, wheelOmega, tireData.sharedParams.v0, 
		tireData.freeRotatingRadiusParams, normalizedVelR0);

	const TFloat vertTireStiffness = mfTireVerticalStiffness<TConfig>(dpi, tireData.verticalStiffnessParams);

	const TFloat re = mfTireEffectiveRollingRadius(rOmega, vertTireStiffness, tireData.sharedParams.fz0, fzNormalized,
		tireData.effectiveRollingRadiusParams);
	effectiveRollingRadius = re;

    const TFloat absLongVelocityPlusEpsilon = PxAbs(longVelocity) + tireData.sharedParams.epsilonV;
    const bool isAboveOrEqLongVelThreshold = absLongVelocityPlusEpsilon >= tireData.sharedParams.slipReductionLowVelocity;
    TFloat absLongVelocityInterpolated;
    if (isAboveOrEqLongVelThreshold)
        absLongVelocityInterpolated = absLongVelocityPlusEpsilon;
    else
    {
        // interpolation with a quadratic weight change for the threshold velocity such that the effect reduces
        // fast with growing velocity. Read section about oscillation further below.
        const TFloat longVelRatio = absLongVelocityPlusEpsilon / tireData.sharedParams.slipReductionLowVelocity;
        TFloat interpFactor = TFloat(1.0) - longVelRatio;
        interpFactor = interpFactor * interpFactor;
        absLongVelocityInterpolated = (interpFactor * tireData.sharedParams.slipReductionLowVelocity) + ((TFloat(1.0) - interpFactor) * absLongVelocityPlusEpsilon);
    }

	//
	// longitudinal slip ratio (kappa):
	//
    // kappa = - (Vsx / |Vcx|)
	//
	// note: sign is chosen to get positive value under drive torque and negative under break torque
	//
	// ----------------------------------------------------------------------------------------------------------------
	// Vcx: velocity of wheel contact center, projected onto wheel x-axis (direction wheel is pointing to)
	// ----------------------------------------------------------------------------------------------------------------
	// Vsx: longitudinal slip velocity
	//      = Vcx - (omega * Re)
	//      (difference between the actual velocity at the wheel contact point and the "orbital" velocity from the 
	//      wheel rotation)
	//      -----------------------------------------------------------------------------------------------------------
	//      omega: wheel rotational velocity
	//      -----------------------------------------------------------------------------------------------------------
	//      Re: effective rolling radius (at free rolling of the tire, no brake/drive torque)
	//          At constant speed with no brake/drive torque, using Re will result in Vsx (and thus kappa) being 0.
	// ----------------------------------------------------------------------------------------------------------------
	//
	// Problems at low velocities and starting from standstill etc.
	// - oscillation:
	//   division by small velocity values results in large slip values and thus in large forces. For large
	//   time steps this results in overshooting a stable state.
	//   The applied formulas are for the steady-state only. This will add to the overshooting too.
	//   Furthermore, the algorithm is prone to oscillation at low velocities to begin with.
	// - zero slip does not mean zero force:
	//   the formulas include shift terms in the function input and output values. That will result in non-zero forces
	//   even if the slip is 0.
	//
	// Reducing these problems:
	// - run at smaller time steps
	// - artificially lower slip values when the velocity is below a threshold. This is done by replacing the actual Vcx
	//   velocity with an interpolation between the real Vcx and a threshold velocity if Vcx is below the threshold
	//   velocity (for example, if Vcx was 0, the threshold velocity would be used instead). The larger the threshold
	//   velocity, the less oscillation is observed at the cost of affecting the simulation behavior at normal speeds.
	//
	const TFloat longSlipVelocityNeg = (wheelOmega * re) - longVelocity;  // = -Vsx
	const TFloat longSlipRatio = longSlipVelocityNeg / absLongVelocityInterpolated;
	longSlip = longSlipRatio;

	//
	// lateral slip angle (alpha):
	//
    // alpha = arctan(-Vcy / |Vcx|)
	//
	// note: sign is chosen to get positive force value if the slip is positive
	//
	// ----------------------------------------------------------------------------------------------------------------
	// Vcx: velocity of wheel contact center, projected onto wheel x-axis (direction wheel is pointing to)
	// ----------------------------------------------------------------------------------------------------------------
	// Vcy: velocity of wheel contact center, projected onto wheel y-axis (direction pointing to the side of the wheel,
	//      that is, perpendicular to x-axis mentioned above)
	// ----------------------------------------------------------------------------------------------------------------
	//
	const TFloat tanSlipAngle = (-latVelocity) / absLongVelocityInterpolated;
	tanLatSlip = tanSlipAngle;
}

template<typename TConfig>
PX_FORCE_INLINE void mfTireComputeForce(
	const MFTireDataT<typename TConfig::Float>& tireData, 
	const typename TConfig::Float tireFriction,
	const typename TConfig::Float longSlip, const typename TConfig::Float tanLatSlip, 
	const typename TConfig::Float camber,
	const typename TConfig::Float effectiveRollingRadius,
	const typename TConfig::Float tireLoad,
	const typename TConfig::Float tireInflationPressure,
	const typename TConfig::Float longVelocity, const typename TConfig::Float latVelocity,
	typename TConfig::Float& wheelTorque, 
	typename TConfig::Float& tireLongForce, 
	typename TConfig::Float& tireLatForce, 
	typename TConfig::Float& tireAlignMoment)
{
	typedef typename TConfig::Float TFloat;

	PX_ASSERT(tireFriction > 0);
	PX_ASSERT(tireLoad >= 0);

	wheelTorque = TFloat(0.0);
	tireLongForce = TFloat(0.0);
	tireLatForce = TFloat(0.0);
	tireAlignMoment = TFloat(0.0);

	/*
	// Magic Formula Tire Model
	//
	// Implementation follows pretty closely the description in the book:
	// Tire and Vehicle Dynamics, 3rd Edition, Hans Pacejka
	// The steady-state model is implemented only.
	//
	// General limitations:
	// - expects rather smooth road surface (surface wavelengths longer than a tire radius) up to frequencies of 8 Hz
	//   (for steady-state model around 1 Hz only)
	//
	//
	// The magic formula (see magicFormulaSine(), magicFormulaCosine()) is used as a basis for describing curves for
	// longitudinal/lateral force, aligning torque etc.
	//
	//
	// The model uses the following coordinate system for the tire:
	//
	// top view:                                                  side view:
	//
	//
	//          ________________________                                   __________
	//         |                        |                              ___/          \___
	//         |             ----------------> x                    __/                  \__
	//         |____________|___________|                          /                        \
	//                      |                                     |                          |
	//                      |                                    /                            \
	//                      |                                    |                            |
	//                      |                                    |               ---------------------> x
	//                      |                                    \              |             /
	//                      v                                     |             |            |
	//                      y                                      \__          |         __/
	//                                                                \___      |     ___/
	//                                                        _ _ _ _ _ _ \_____|____/ _ _ _ _ _ _
	//                                                                          |
	//                                                                          |
	//                                                                          v
	//                                                                          z
	*/

	MFTireVolatileSharedParams<TFloat> volatileSharedParams;
	{
		mfTireComputeVolatileSharedParams<TConfig>(camber, tireLoad, 
			tireFriction, tireFriction, tireInflationPressure,
			tireData.sharedParams, volatileSharedParams);
	}

	// note: even if long slip/lat slip/camber are all zero, the computation takes place as the model can take effects like
	// conicity etc. into account (thus, forces might not be 0 even if slip and camber are 0)

	TFloat K_kappa_longitudinal;
	const TFloat longForcePure = mfTireLongitudinalForcePure<TConfig>(longSlip, tireLoad, 
		tireData.longitudinalForcePureParams, tireData.sharedParams, volatileSharedParams,
		K_kappa_longitudinal);

	const TFloat longForceCombined = mfTireLongitudinalForceCombined<TConfig>(longSlip, tanLatSlip,
		longForcePure, tireData.longitudinalForceCombinedParams, volatileSharedParams);
	tireLongForce = longForceCombined;

	TFloat latForcePureNoCamberNoTurnSlip;
	TFloat latForcePureNoCamberNoTurnSlipWeight;
	TFloat* latForcePureNoCamberNoTurnSlipPtr;
	TFloat* latForcePureNoCamberNoTurnSlipWeightPtr;
	if (tireData.flags & MFTireDataFlag::eALIGNING_MOMENT)
	{
		latForcePureNoCamberNoTurnSlipPtr = &latForcePureNoCamberNoTurnSlip;
		latForcePureNoCamberNoTurnSlipWeightPtr = &latForcePureNoCamberNoTurnSlipWeight;
	}
	else
	{
		latForcePureNoCamberNoTurnSlipPtr = NULL;
		latForcePureNoCamberNoTurnSlipWeightPtr = NULL;
	}

	TFloat B_lateral, C_lateral, K_alpha_withEpsilon_lateral, Sh_lateral, Sv_lateral, mu_zeta2;
	const TFloat latForcePure = mfTireLateralForcePure<TConfig>(tanLatSlip, tireLoad, 
		tireData.lateralForcePureParams, tireData.sharedParams, volatileSharedParams, latForcePureNoCamberNoTurnSlipPtr,
		B_lateral, C_lateral, K_alpha_withEpsilon_lateral, Sh_lateral, Sv_lateral, mu_zeta2);

	const TFloat latForceCombined = mfTireLateralForceCombined<TConfig>(tanLatSlip, longSlip, tireLoad,
		latForcePure, mu_zeta2, tireData.lateralForceCombinedParams, volatileSharedParams,
		latForcePureNoCamberNoTurnSlipWeightPtr);
	tireLatForce = latForceCombined;

	if (tireData.flags & MFTireDataFlag::eALIGNING_MOMENT)
	{
		MFTireAligningTorqueVolatileSharedParams<TFloat> aligningTorqueVolatileSharedParams;
		const TFloat combinedVelocity = PxSqrt((longVelocity*longVelocity) + (latVelocity*latVelocity));
		mfTireComputeAligningTorqueVolatileSharedParams(combinedVelocity, longVelocity, tireData.sharedParams.epsilonV,
			aligningTorqueVolatileSharedParams);

		const TFloat aligningTorque = mfTireAligningTorqueCombined<TConfig>(tanLatSlip, longSlip, tireLoad,
			longForceCombined, latForceCombined, *latForcePureNoCamberNoTurnSlipPtr, *latForcePureNoCamberNoTurnSlipWeightPtr,
			K_kappa_longitudinal, K_alpha_withEpsilon_lateral, B_lateral, C_lateral, Sh_lateral, Sv_lateral,
			tireData.aligningTorquePurePneumaticTrailParams, tireData.aligningTorquePureResidualTorqueParams,
			tireData.aligningTorqueCombinedParams, aligningTorqueVolatileSharedParams,
			tireData.sharedParams, volatileSharedParams);

		tireAlignMoment = aligningTorque;
	}

	wheelTorque = -tireLongForce * effectiveRollingRadius;
}

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif //VEHICLE_MF_TIRE_H
