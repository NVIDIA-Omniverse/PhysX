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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_BIAS_COEFFICIENT_CPUGPU_H
#define DY_BIAS_COEFFICIENT_CPUGPU_H

#include "foundation/PxSimpleTypes.h"

#include <stdio.h>


namespace physx
{
namespace Dy
{


/**
\brief Struct that holds the different bias coefficients used for different devices, features and solvers
*/
struct BiasCoefficientCollection
{
	void set(bool isGPU, bool isTGS, PxU32 nbPositionIterations)
	{
		if (isTGS)
		{
			const PxReal biasCoefficientBase = PxSqrt(1.0f / nbPositionIterations);
			const PxReal biasCoefficientBaseTimes2 = 2.0f * biasCoefficientBase;

			rigidContact = PxMin(0.8f, biasCoefficientBaseTimes2);
			joint = biasCoefficientBase;
			articulation = PxMin(0.9f, biasCoefficientBaseTimes2);

			if (isGPU)
			{
				particlesAndDeformables = PxMin(0.9f, biasCoefficientBaseTimes2);
				femClothParticle = 0.7f;
				femClothRigidAttachment = 0.7f;
			}
			else
			{
				// these are GPU only features
				particlesAndDeformables = 0.0f;
				femClothParticle = 0.0f;
				femClothRigidAttachment = 0.0f;
			}
		}
		else
		{
			rigidContact = 0.8f;
			joint = 1.0f;
			articulation = 0.8f;

			if (isGPU)
			{
				particlesAndDeformables = 0.7f;
				femClothParticle = 0.7f;
				femClothRigidAttachment = 0.5f;
			}
			else
			{
				// these are GPU only features
				particlesAndDeformables = 0.0f;
				femClothParticle = 0.0f;
				femClothRigidAttachment = 0.0f;
			}
		}
	}

	// for contact constraints (including articulation contact). Applies to sticky friction bias too.
	//
	PxReal rigidContact;

	// for maximal coordinate joints (PxJoint, PxConstraint)
	//
	PxReal joint;

	// for articulation internal constraints, mimic joints etc.
	//
	PxReal articulation;

	// for particles, deformables etc. (only on GPU)
	//
	// note: PxgPBDParticleSystemCore::solveTGS() has two lines that do: PxMin(0.7f, coefficient)
	//
	PxReal particlesAndDeformables;

	// for FEM cloth collision with particles and attachments to rigid objects
	//
	PxReal femClothParticle;
	PxReal femClothRigidAttachment;
};


} //namespace Dy
} //namespace physx


#endif //DY_BIAS_COEFFICIENT_CPUGPU_H
