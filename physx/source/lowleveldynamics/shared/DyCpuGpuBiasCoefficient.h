// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
				particle = PxMin(0.9f, biasCoefficientBaseTimes2);
				deformableRigidAttachment = PxMin(0.9f, biasCoefficientBaseTimes2);
			}
			else
			{
				// these are GPU only features
				particle = 0.0f;
				deformableRigidAttachment = 0.0f;
			}
		}
		else
		{
			rigidContact = 0.8f;
			joint = 1.0f;
			articulation = 0.8f;

			if (isGPU)
			{
				particle = 0.7f;
				deformableRigidAttachment = 0.5f;
			}
			else
			{
				// these are GPU only features
				particle = 0.0f;
				deformableRigidAttachment = 0.0f;
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

	// Bias / under-relaxation for the internal PBD particle solve (GPU).
	PxReal particle;

	// for FEM deformable (volume + cloth) attachment to rigid bodies (only on GPU).
	// PGS value (0.5) is the Baumgarte coefficient empirically tuned for cloth-attach
	// stability: higher values (e.g. 0.7) destabilize low-mass single-vertex
	// cloth attachments. Volume FEM tolerates higher values but is also fine at 0.5.
	//
	PxReal deformableRigidAttachment;
};


} //namespace Dy
} //namespace physx


#endif //DY_BIAS_COEFFICIENT_CPUGPU_H
