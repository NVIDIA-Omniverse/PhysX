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
#include "vehicle2/PxVehicleFunctions.h"
#include "vehicle2/PxVehicleMaths.h"

#include "vehicle2/drivetrain/PxVehicleDrivetrainHelpers.h"
#include "vehicle2/drivetrain/PxVehicleDrivetrainParams.h"

#include "vehicle2/wheel/PxVehicleWheelStates.h"


namespace physx
{
namespace vehicle2
{

void PxVehicleMatrixNNLUSolver::decomposeLU(const PxVehicleMatrixNN& A)
{
	const PxU32 D = A.mSize;

	mLU = A;

	mDetM = 1.0f;

	for (PxU32 k = 0; k < D - 1; ++k)
	{
		PxU32 pivot_row = k;
		PxU32 pivot_col = k;
		float abs_pivot_elem = 0.0f;
		for (PxU32 c = k; c < D; ++c)
		{
			for (PxU32 r = k; r < D; ++r)
			{
				const PxF32 abs_elem = PxAbs(mLU.get(r, c));
				if (abs_elem > abs_pivot_elem)
				{
					abs_pivot_elem = abs_elem;
					pivot_row = r;
					pivot_col = c;
				}
			}
		}

		mP[k] = pivot_row;
		if (pivot_row != k)
		{
			mDetM = -mDetM;
			for (PxU32 c = 0; c < D; ++c)
			{
				//swap(m_LU(k,c), m_LU(pivot_row,c));
				const PxF32 pivotrowc = mLU.get(pivot_row, c);
				mLU.set(pivot_row, c, mLU.get(k, c));
				mLU.set(k, c, pivotrowc);
			}
		}

		mQ[k] = pivot_col;
		if (pivot_col != k)
		{
			mDetM = -mDetM;
			for (PxU32 r = 0; r < D; ++r)
			{
				//swap(m_LU(r,k), m_LU(r,pivot_col));
				const PxF32 rpivotcol = mLU.get(r, pivot_col);
				mLU.set(r, pivot_col, mLU.get(r, k));
				mLU.set(r, k, rpivotcol);
			}
		}

		mDetM *= mLU.get(k, k);

		if (mLU.get(k, k) != 0.0f)
		{
			for (PxU32 r = k + 1; r < D; ++r)
			{
				mLU.set(r, k, mLU.get(r, k) / mLU.get(k, k));
				for (PxU32 c = k + 1; c < D; ++c)
				{
					//m_LU(r,c) -= m_LU(r,k)*m_LU(k,c);
					const PxF32 rc = mLU.get(r, c);
					const PxF32 rk = mLU.get(r, k);
					const PxF32 kc = mLU.get(k, c);
					mLU.set(r, c, rc - rk * kc);
				}
			}
		}
	}

	mDetM *= mLU.get(D - 1, D - 1);
}

bool PxVehicleMatrixNNLUSolver::solve(const PxVehicleVectorN& b, PxVehicleVectorN& x) const
{
	const PxU32 D = x.getSize();

	if ((b.getSize() != x.getSize()) || (b.getSize() != mLU.getSize()) || (0.0f == mDetM))
	{
		for (PxU32 i = 0; i < D; i++)
		{
			x[i] = 0.0f;
		}
		return false;
	}

	x = b;

	// Perform row permutation to get Pb
	for (PxU32 i = 0; i < D - 1; ++i)
	{
		//swap(x(i), x(m_P[i]));														
		const PxF32 xp = x[mP[i]];
		x[mP[i]] = x[i];
		x[i] = xp;
	}

	// Forward substitute to get (L^-1)Pb
	for (PxU32 r = 1; r < D; ++r)
	{
		for (PxU32 i = 0; i < r; ++i)
		{
			x[r] -= mLU.get(r, i)*x[i];
		}
	}

	// Back substitute to get (U^-1)(L^-1)Pb
	for (PxU32 r = D; r-- > 0;)
	{
		for (PxU32 i = r + 1; i < D; ++i)
		{
			x[r] -= mLU.get(r, i)*x[i];
		}
		x[r] /= mLU.get(r, r);
	}

	// Perform column permutation to get the solution (Q^T)(U^-1)(L^-1)Pb	
	for (PxU32 i = D - 1; i-- > 0;)
	{
		//swap(x(i), x(m_Q[i]));													
		const PxF32 xq = x[mQ[i]];
		x[mQ[i]] = x[i];
		x[i] = xq;
	}

	return true;
}

void PxVehicleMatrixNGaussSeidelSolver::solve(const PxU32 maxIterations, const PxF32 tolerance, const PxVehicleMatrixNN& A, const PxVehicleVectorN& b, PxVehicleVectorN& result) const
{
	const PxU32 N = A.getSize();

	PxVehicleVectorN DInv(N);
	PxF32 bLength2 = 0.0f;
	for (PxU32 i = 0; i < N; i++)
	{
		DInv[i] = 1.0f / A.get(i, i);
		bLength2 += (b[i] * b[i]);
	}

	PxU32 iteration = 0;
	PxF32 error = PX_MAX_F32;
	while (iteration < maxIterations && tolerance < error)
	{
		for (PxU32 i = 0; i < N; i++)
		{
			PxF32 l = 0.0f;
			for (PxU32 j = 0; j < i; j++)
			{
				l += A.get(i, j) * result[j];
			}

			PxF32 u = 0.0f;
			for (PxU32 j = i + 1; j < N; j++)
			{
				u += A.get(i, j) * result[j];
			}

			result[i] = DInv[i] * (b[i] - l - u);
		}

		//Compute the error.
		PxF32 rLength2 = 0;
		for (PxU32 i = 0; i < N; i++)
		{
			PxF32 e = -b[i];
			for (PxU32 j = 0; j < N; j++)
			{
				e += A.get(i, j) * result[j];
			}
			rLength2 += e * e;
		}
		error = (rLength2 / (bLength2 + 1e-10f));

		iteration++;
	}
}

bool PxVehicleMatrix33Solver::solve(const PxVehicleMatrixNN& A_, const PxVehicleVectorN& b_, PxVehicleVectorN& result) const
{
	const PxF32 a = A_.get(0, 0);
	const PxF32 b = A_.get(0, 1);
	const PxF32 c = A_.get(0, 2);

	const PxF32 d = A_.get(1, 0);
	const PxF32 e = A_.get(1, 1);
	const PxF32 f = A_.get(1, 2);

	const PxF32 g = A_.get(2, 0);
	const PxF32 h = A_.get(2, 1);
	const PxF32 k = A_.get(2, 2);

	const PxF32 detA = a * (e*k - f * h) - b * (k*d - f * g) + c * (d*h - e * g);
	if (0.0f == detA)
	{
		return false;
	}
	const PxF32 detAInv = 1.0f / detA;

	const PxF32 A = (e*k - f * h);
	const PxF32 D = -(b*k - c * h);
	const PxF32 G = (b*f - c * e);
	const PxF32 B = -(d*k - f * g);
	const PxF32 E = (a*k - c * g);
	const PxF32 H = -(a*f - c * d);
	const PxF32 C = (d*h - e * g);
	const PxF32 F = -(a*h - b * g);
	const PxF32 K = (a*e - b * d);

	result[0] = detAInv * (A*b_[0] + D * b_[1] + G * b_[2]);
	result[1] = detAInv * (B*b_[0] + E * b_[1] + H * b_[2]);
	result[2] = detAInv * (C*b_[0] + F * b_[1] + K * b_[2]);

	return true;
}

void PxVehicleLegacyDifferentialWheelSpeedContributionsCompute
(const PxVehicleFourWheelDriveDifferentialLegacyParams& diffParams,
 const PxU32 nbWheels, PxReal* diffAveWheelSpeedContributions)
{
	PxMemZero(diffAveWheelSpeedContributions, sizeof(PxReal) * nbWheels);

	const PxU32 wheelIds[4] =
	{
		diffParams.frontWheelIds[0],
		diffParams.frontWheelIds[1],
		diffParams.rearWheelIds[0],
		diffParams.rearWheelIds[1],
	};

	const PxF32 frontRearSplit = diffParams.frontRearSplit;
	const PxF32 frontNegPosSplit = diffParams.frontNegPosSplit;
	const PxF32 rearNegPosSplit = diffParams.rearNegPosSplit;

	const PxF32 oneMinusFrontRearSplit = 1.0f - diffParams.frontRearSplit;
	const PxF32 oneMinusFrontNegPosSplit = 1.0f - diffParams.frontNegPosSplit;
	const PxF32 oneMinusRearNegPosSplit = 1.0f - diffParams.rearNegPosSplit;

	switch (diffParams.type)
	{
	case PxVehicleFourWheelDriveDifferentialLegacyParams::eDIFF_TYPE_LS_4WD:
		diffAveWheelSpeedContributions[wheelIds[0]] = frontRearSplit * frontNegPosSplit;
		diffAveWheelSpeedContributions[wheelIds[1]] = frontRearSplit * oneMinusFrontNegPosSplit;
		diffAveWheelSpeedContributions[wheelIds[2]] = oneMinusFrontRearSplit * rearNegPosSplit;
		diffAveWheelSpeedContributions[wheelIds[3]] = oneMinusFrontRearSplit * oneMinusRearNegPosSplit;
		break;
	case PxVehicleFourWheelDriveDifferentialLegacyParams::eDIFF_TYPE_LS_FRONTWD:
		diffAveWheelSpeedContributions[wheelIds[0]] = frontNegPosSplit;
		diffAveWheelSpeedContributions[wheelIds[1]] = oneMinusFrontNegPosSplit;
		diffAveWheelSpeedContributions[wheelIds[2]] = 0.0f;
		diffAveWheelSpeedContributions[wheelIds[3]] = 0.0f;
		break;
	case PxVehicleFourWheelDriveDifferentialLegacyParams::eDIFF_TYPE_LS_REARWD:
		diffAveWheelSpeedContributions[wheelIds[0]] = 0.0f;
		diffAveWheelSpeedContributions[wheelIds[1]] = 0.0f;
		diffAveWheelSpeedContributions[wheelIds[2]] = rearNegPosSplit;
		diffAveWheelSpeedContributions[wheelIds[3]] = oneMinusRearNegPosSplit;
		break;
	default:
		PX_ASSERT(false);
		break;
	}

	PX_ASSERT(
		((diffAveWheelSpeedContributions[wheelIds[0]] + diffAveWheelSpeedContributions[wheelIds[1]] + diffAveWheelSpeedContributions[wheelIds[2]] + diffAveWheelSpeedContributions[wheelIds[3]]) >= 0.999f) &&
		((diffAveWheelSpeedContributions[wheelIds[0]] + diffAveWheelSpeedContributions[wheelIds[1]] + diffAveWheelSpeedContributions[wheelIds[2]] + diffAveWheelSpeedContributions[wheelIds[3]]) <= 1.001f));
}

PX_FORCE_INLINE void splitTorque
(const PxF32 w1, const PxF32 w2, const PxF32 diffBias, const PxF32 defaultSplitRatio,
	PxF32* t1, PxF32* t2)
{
	PX_ASSERT(PxVehicleComputeSign(w1) == PxVehicleComputeSign(w2) && 0.0f != PxVehicleComputeSign(w1));
	const PxF32 w1Abs = PxAbs(w1);
	const PxF32 w2Abs = PxAbs(w2);
	const PxF32 omegaMax = PxMax(w1Abs, w2Abs);
	const PxF32 omegaMin = PxMin(w1Abs, w2Abs);
	const PxF32 delta = omegaMax - diffBias * omegaMin;
	const PxF32 deltaTorque = physx::intrinsics::fsel(delta, delta / omegaMax, 0.0f);
	const PxF32 f1 = physx::intrinsics::fsel(w1Abs - w2Abs, defaultSplitRatio*(1.0f - deltaTorque), defaultSplitRatio*(1.0f + deltaTorque));
	const PxF32 f2 = physx::intrinsics::fsel(w1Abs - w2Abs, (1.0f - defaultSplitRatio)*(1.0f + deltaTorque), (1.0f - defaultSplitRatio)*(1.0f - deltaTorque));
	const PxF32 denom = 1.0f / (f1 + f2);
	*t1 = f1 * denom;
	*t2 = f2 * denom;
	PX_ASSERT((*t1 + *t2) >= 0.999f && (*t1 + *t2) <= 1.001f);
}

void PxVehicleLegacyDifferentialTorqueRatiosCompute
(const PxVehicleFourWheelDriveDifferentialLegacyParams& diffParams,
 const PxVehicleArrayData<const PxVehicleWheelRigidBody1dState>& wheelOmegas, 
 const PxU32 nbWheels, PxReal* diffTorqueRatios)
{
	PxMemZero(diffTorqueRatios, sizeof(PxReal) * nbWheels);

	const PxU32 wheelIds[4] =
	{
		diffParams.frontWheelIds[0],
		diffParams.frontWheelIds[1],
		diffParams.rearWheelIds[0],
		diffParams.rearWheelIds[1],
	};

	const PxF32 wfl = wheelOmegas[wheelIds[0]].rotationSpeed;
	const PxF32 wfr = wheelOmegas[wheelIds[1]].rotationSpeed;
	const PxF32 wrl = wheelOmegas[wheelIds[2]].rotationSpeed;
	const PxF32 wrr = wheelOmegas[wheelIds[3]].rotationSpeed;

	const PxF32 centreBias = diffParams.centerBias;
	const PxF32 frontBias = diffParams.frontBias;
	const PxF32 rearBias = diffParams.rearBias;

	const PxF32 frontRearSplit = diffParams.frontRearSplit;
	const PxF32 frontLeftRightSplit = diffParams.frontNegPosSplit;
	const PxF32 rearLeftRightSplit = diffParams.rearNegPosSplit;

	const PxF32 oneMinusFrontRearSplit = 1.0f - diffParams.frontRearSplit;
	const PxF32 oneMinusFrontLeftRightSplit = 1.0f - diffParams.frontNegPosSplit;
	const PxF32 oneMinusRearLeftRightSplit = 1.0f - diffParams.rearNegPosSplit;

	const PxF32 swfl = PxVehicleComputeSign(wfl);

	//Split a torque of 1 between front and rear.
	//Then split that torque between left and right.
	PxF32 torqueFrontLeft = 0;
	PxF32 torqueFrontRight = 0;
	PxF32 torqueRearLeft = 0;
	PxF32 torqueRearRight = 0;
	switch (diffParams.type)
	{
	case PxVehicleFourWheelDriveDifferentialLegacyParams::eDIFF_TYPE_LS_4WD:
		if (0.0f != swfl && swfl == PxVehicleComputeSign(wfr) && swfl == PxVehicleComputeSign(wrl) && swfl == PxVehicleComputeSign(wrr))
		{
			PxF32 torqueFront, torqueRear;
			const PxF32 omegaFront = PxAbs(wfl + wfr);
			const PxF32 omegaRear = PxAbs(wrl + wrr);
			splitTorque(omegaFront, omegaRear, centreBias, frontRearSplit, &torqueFront, &torqueRear);
			splitTorque(wfl, wfr, frontBias, frontLeftRightSplit, &torqueFrontLeft, &torqueFrontRight);
			splitTorque(wrl, wrr, rearBias, rearLeftRightSplit, &torqueRearLeft, &torqueRearRight);
			torqueFrontLeft *= torqueFront;
			torqueFrontRight *= torqueFront;
			torqueRearLeft *= torqueRear;
			torqueRearRight *= torqueRear;
		}
		else
		{
			torqueFrontLeft = frontRearSplit * frontLeftRightSplit;
			torqueFrontRight = frontRearSplit * oneMinusFrontLeftRightSplit;
			torqueRearLeft = oneMinusFrontRearSplit * rearLeftRightSplit;
			torqueRearRight = oneMinusFrontRearSplit * oneMinusRearLeftRightSplit;
		}
		break;

	case PxVehicleFourWheelDriveDifferentialLegacyParams::eDIFF_TYPE_LS_FRONTWD:
		if (0.0f != swfl && swfl == PxVehicleComputeSign(wfr))
		{
			splitTorque(wfl, wfr, frontBias, frontLeftRightSplit, &torqueFrontLeft, &torqueFrontRight);
		}
		else
		{
			torqueFrontLeft = frontLeftRightSplit;
			torqueFrontRight = oneMinusFrontLeftRightSplit;
		}
		break;

	case PxVehicleFourWheelDriveDifferentialLegacyParams::eDIFF_TYPE_LS_REARWD:

		if (0.0f != PxVehicleComputeSign(wrl) && PxVehicleComputeSign(wrl) == PxVehicleComputeSign(wrr))
		{
			splitTorque(wrl, wrr, rearBias, rearLeftRightSplit, &torqueRearLeft, &torqueRearRight);
		}
		else
		{
			torqueRearLeft = rearLeftRightSplit;
			torqueRearRight = oneMinusRearLeftRightSplit;
		}
		break;

	default:
		PX_ASSERT(false);
		break;
	}

	diffTorqueRatios[wheelIds[0]] = torqueFrontLeft;
	diffTorqueRatios[wheelIds[1]] = torqueFrontRight;
	diffTorqueRatios[wheelIds[2]] = torqueRearLeft;
	diffTorqueRatios[wheelIds[3]] = torqueRearRight;

	PX_ASSERT(((torqueFrontLeft + torqueFrontRight + torqueRearLeft + torqueRearRight) >= 0.999f) && ((torqueFrontLeft + torqueFrontRight + torqueRearLeft + torqueRearRight) <= 1.001f));
}


} //namespace vehicle2
} //namespace physx

