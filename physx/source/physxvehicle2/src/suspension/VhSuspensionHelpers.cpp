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
#include "vehicle2/PxVehicleMaths.h"
#include "vehicle2/suspension/PxVehicleSuspensionHelpers.h"

namespace physx
{
namespace vehicle2
{
#define DETERMINANT_THRESHOLD (1e-6f)

	bool PxVehicleComputeSprungMasses(const PxU32 numSprungMasses, const PxVec3* sprungMassCoordinates, const PxReal totalMass, const PxVehicleAxes::Enum gravityDir, PxReal* sprungMasses)
	{
		if (numSprungMasses < 1)
			return false;

		if (numSprungMasses > PxVehicleLimits::eMAX_NB_WHEELS)
			return false;

		if (totalMass <= 0.0f)
			return false;

		if (!sprungMassCoordinates || !sprungMasses)
			return false;

		const PxVec3 centreOfMass(PxZero);

		PxU32 gravityDirection = 0xffffffff;
		switch (gravityDir)
		{
		case PxVehicleAxes::eNegX:
		case PxVehicleAxes::ePosX:
			gravityDirection = 0;
			break;
		case PxVehicleAxes::eNegY:
		case PxVehicleAxes::ePosY:
			gravityDirection = 1;
			break;
		case PxVehicleAxes::eNegZ:
		case PxVehicleAxes::ePosZ:
			gravityDirection = 2;
			break;
		default:
			PX_ASSERT(false);
			break;
		}

		if (1 == numSprungMasses)
		{
			sprungMasses[0] = totalMass;
		}
		else if (2 == numSprungMasses)
		{
			PxVec3 v = sprungMassCoordinates[0];
			v[gravityDirection] = 0;
			PxVec3 w = sprungMassCoordinates[1] - sprungMassCoordinates[0];
			w[gravityDirection] = 0;
			w.normalize();

			PxVec3 cm = centreOfMass;
			cm[gravityDirection] = 0;
			PxF32 t = w.dot(cm - v);
			PxVec3 p = v + w * t;

			PxVec3 x0 = sprungMassCoordinates[0];
			x0[gravityDirection] = 0;
			PxVec3 x1 = sprungMassCoordinates[1];
			x1[gravityDirection] = 0;
			const PxF32 r0 = (x0 - p).dot(w);
			const PxF32 r1 = (x1 - p).dot(w);

			if (PxAbs(r0 - r1) <= DETERMINANT_THRESHOLD)
				return false;

			const PxF32 m0 = totalMass * r1 / (r1 - r0);
			const PxF32 m1 = totalMass - m0;

			sprungMasses[0] = m0;
			sprungMasses[1] = m1;
		}
		else if (3 == numSprungMasses)
		{
			const PxU32 d0 = (gravityDirection + 1) % 3;
			const PxU32 d1 = (gravityDirection + 2) % 3;

			PxVehicleMatrixNN A(3);
			PxVehicleVectorN b(3);
			A.set(0, 0, sprungMassCoordinates[0][d0]);
			A.set(0, 1, sprungMassCoordinates[1][d0]);
			A.set(0, 2, sprungMassCoordinates[2][d0]);
			A.set(1, 0, sprungMassCoordinates[0][d1]);
			A.set(1, 1, sprungMassCoordinates[1][d1]);
			A.set(1, 2, sprungMassCoordinates[2][d1]);
			A.set(2, 0, 1.f);
			A.set(2, 1, 1.f);
			A.set(2, 2, 1.f);
			b[0] = totalMass * centreOfMass[d0];
			b[1] = totalMass * centreOfMass[d1];
			b[2] = totalMass;

			PxVehicleVectorN result(3);
			PxVehicleMatrixNNLUSolver solver;
			solver.decomposeLU(A);
			if (PxAbs(solver.getDet()) <= DETERMINANT_THRESHOLD)
				return false;
			solver.solve(b, result);

			sprungMasses[0] = result[0];
			sprungMasses[1] = result[1];
			sprungMasses[2] = result[2];
		}
		else if (numSprungMasses >= 4)
		{
			const PxU32 d0 = (gravityDirection + 1) % 3;
			const PxU32 d1 = (gravityDirection + 2) % 3;

			const PxF32 mbar = totalMass / (numSprungMasses*1.0f);

			//See http://en.wikipedia.org/wiki/Lagrange_multiplier
			//particularly the section on multiple constraints.

			//3 Constraint equations.
			//g0 = sum_ xi*mi=xcm	
			//g1 = sum_ zi*mi=zcm	
			//g2 = sum_ mi = totalMass		
			//Minimisation function to achieve solution with minimum mass variance.
			//f = sum_ (mi - mave)^2 
			//Lagrange terms (N equations, N+3 unknowns)
			//2*mi  - xi*lambda0 - zi*lambda1 - 1*lambda2 = 2*mave

			PxVehicleMatrixNN A(numSprungMasses + 3);
			PxVehicleVectorN b(numSprungMasses + 3);

			//g0, g1, g2
			for (PxU32 i = 0; i < numSprungMasses; i++)
			{
				A.set(0, i, sprungMassCoordinates[i][d0]);	//g0
				A.set(1, i, sprungMassCoordinates[i][d1]);	//g1
				A.set(2, i, 1.0f);							//g2
			}
			for (PxU32 i = numSprungMasses; i < numSprungMasses + 3; i++)
			{
				A.set(0, i, 0);								//g0 independent of lambda0,lambda1,lambda2
				A.set(1, i, 0);								//g1 independent of lambda0,lambda1,lambda2
				A.set(2, i, 0);								//g2 independent of lambda0,lambda1,lambda2
			}
			b[0] = totalMass * (centreOfMass[d0]);			//g0
			b[1] = totalMass * (centreOfMass[d1]);			//g1
			b[2] = totalMass;								//g2

			//Lagrange terms.
			for (PxU32 i = 0; i < numSprungMasses; i++)
			{
				//Off-diagonal terms from the derivative of f
				for (PxU32 j = 0; j < numSprungMasses; j++)
				{
					A.set(i + 3, j, 0);
				}
				//Diagonal term from the derivative of f
				A.set(i + 3, i, 2.f);

				//Derivative of g
				A.set(i + 3, numSprungMasses + 0, sprungMassCoordinates[i][d0]);
				A.set(i + 3, numSprungMasses + 1, sprungMassCoordinates[i][d1]);
				A.set(i + 3, numSprungMasses + 2, 1.0f);

				//rhs.
				b[i + 3] = 2 * mbar;
			}

			//Solve Ax=b
			PxVehicleVectorN result(numSprungMasses + 3);
			PxVehicleMatrixNNLUSolver solver;
			solver.decomposeLU(A);
			solver.solve(b, result);
			if (PxAbs(solver.getDet()) <= DETERMINANT_THRESHOLD)
				return false;
			for (PxU32 i = 0; i < numSprungMasses; i++)
			{
				sprungMasses[i] = result[i];
			}
		}

		return true;
	}
} //namespace vehicle2
} //namespace physx
