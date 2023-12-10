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

#ifndef DY_CONSTRAINT_PREP_H
#define DY_CONSTRAINT_PREP_H

#include "DyConstraint.h"

#include "DySolverConstraintDesc.h"
#include "foundation/PxArray.h"
#include "PxConstraint.h"

namespace physx
{

class PxcConstraintBlockStream;
class PxsConstraintBlockManager;
struct PxSolverBody;
struct PxSolverBodyData;
struct PxSolverConstraintDesc;

namespace Cm
{
	struct SpatialVectorF;
}

namespace Dy
{

	static const PxU32 MAX_CONSTRAINT_ROWS = 20;

	struct SolverConstraintShaderPrepDesc
	{
		const Constraint* constraint;
		PxConstraintSolverPrep solverPrep;
		const void* constantBlock;
		PxU32 constantBlockByteSize;
	};

	SolverConstraintPrepState::Enum setupSolverConstraint4
		(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
		PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
			const PxReal dt, const PxReal recipdt, PxU32& totalRows,
			 PxConstraintAllocator& allocator);

	SolverConstraintPrepState::Enum setupSolverConstraint4
		(PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
		const PxReal dt, const PxReal recipdt, PxU32& totalRows,
		PxConstraintAllocator& allocator, PxU32 maxRows);

	PxU32 SetupSolverConstraint(SolverConstraintShaderPrepDesc& shaderDesc,
								PxSolverConstraintPrepDesc& prepDesc,
								   PxConstraintAllocator& allocator,
								   PxReal dt, PxReal invdt, Cm::SpatialVectorF* Z);


	class ConstraintHelper
	{
	public:

		static PxU32 setupSolverConstraint(
			PxSolverConstraintPrepDesc& prepDesc,
			PxConstraintAllocator& allocator,
			PxReal dt, PxReal invdt, Cm::SpatialVectorF* Z);
	};

	template<class PrepDescT>
	PX_FORCE_INLINE void setupConstraintFlags(PrepDescT& prepDesc, PxU16 flags)
	{
		prepDesc.disablePreprocessing	= (flags & PxConstraintFlag::eDISABLE_PREPROCESSING)!=0;
		prepDesc.improvedSlerp			= (flags & PxConstraintFlag::eIMPROVED_SLERP)!=0;
		prepDesc.driveLimitsAreForces	= (flags & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES)!=0;
		prepDesc.extendedLimits			= (flags & PxConstraintFlag::eENABLE_EXTENDED_LIMITS)!=0;
		prepDesc.disableConstraint		= (flags & PxConstraintFlag::eDISABLE_CONSTRAINT)!=0;
	}

	void preprocessRows(Px1DConstraint** sorted,
		Px1DConstraint* rows,
		PxVec4* angSqrtInvInertia0,
		PxVec4* angSqrtInvInertia1,
		PxU32 rowCount,
		const PxMat33& sqrtInvInertia0F32,
		const PxMat33& sqrtInvInertia1F32,
		const PxReal invMass0,
		const PxReal invMass1,
		const PxConstraintInvMassScale& ims,
		bool disablePreprocessing,
		bool diagonalizeDrive);

	PX_FORCE_INLINE	void setupConstraintRows(Px1DConstraint* PX_RESTRICT rows, PxU32 size)
	{
		// This is necessary so that there will be sensible defaults and shaders will
		// continue to work (albeit with a recompile) if the row format changes.
		// It's a bit inefficient because it fills in all constraint rows even if there
		// is only going to be one generated. A way around this would be for the shader to
		// specify the maximum number of rows it needs, or it could call a subroutine to
		// prep the row before it starts filling it it.

		PxMemZero(rows, sizeof(Px1DConstraint)*size);

		for(PxU32 i=0; i<size; i++)
		{
			Px1DConstraint& c = rows[i];
			//Px1DConstraintInit(c);
			c.minImpulse = -PX_MAX_REAL;
			c.maxImpulse = PX_MAX_REAL;
		}
	}
}

}

#endif
