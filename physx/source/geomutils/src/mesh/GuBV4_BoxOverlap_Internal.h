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

#ifndef GU_BV4_BOX_OVERLAP_INTERNAL_H
#define GU_BV4_BOX_OVERLAP_INTERNAL_H

#include "GuBV4_Common.h"

	// PT: precompute the constant rotation data used in the OBB-OBB overlap test.
	template<class ParamsT>
	PX_FORCE_INLINE void setupBoxBoxRotationData(ParamsT* PX_RESTRICT dst, PxMat33* PX_RESTRICT absRot, const PxMat33* PX_RESTRICT boxToModelR)
	{
		// Precompute absolute box-to-model rotation matrix
		dst->mPreca0_PaddedAligned.x = boxToModelR->column0.x;
		dst->mPreca0_PaddedAligned.y = boxToModelR->column1.y;
		dst->mPreca0_PaddedAligned.z = boxToModelR->column2.z;

		dst->mPreca1_PaddedAligned.x = boxToModelR->column0.y;
		dst->mPreca1_PaddedAligned.y = boxToModelR->column1.z;
		dst->mPreca1_PaddedAligned.z = boxToModelR->column2.x;

		dst->mPreca2_PaddedAligned.x = boxToModelR->column0.z;
		dst->mPreca2_PaddedAligned.y = boxToModelR->column1.x;
		dst->mPreca2_PaddedAligned.z = boxToModelR->column2.y;

		// Epsilon value prevents floating-point inaccuracies (strategy borrowed from RAPID)
		const PxReal epsilon = 1e-6f;
		// PT: TODO: it shouldn't be necessary to output both absRot and mPrecaXX_PaddedAligned
		absRot->column0.x = dst->mPreca0b_PaddedAligned.x = epsilon + fabsf(boxToModelR->column0.x);
		absRot->column0.y = dst->mPreca1b_PaddedAligned.x = epsilon + fabsf(boxToModelR->column0.y);
		absRot->column0.z = dst->mPreca2b_PaddedAligned.x = epsilon + fabsf(boxToModelR->column0.z);

		absRot->column1.x = dst->mPreca2b_PaddedAligned.y = epsilon + fabsf(boxToModelR->column1.x);
		absRot->column1.y = dst->mPreca0b_PaddedAligned.y = epsilon + fabsf(boxToModelR->column1.y);
		absRot->column1.z = dst->mPreca1b_PaddedAligned.y = epsilon + fabsf(boxToModelR->column1.z);

		absRot->column2.x = dst->mPreca1b_PaddedAligned.z = epsilon + fabsf(boxToModelR->column2.x);
		absRot->column2.y = dst->mPreca2b_PaddedAligned.z = epsilon + fabsf(boxToModelR->column2.y);
		absRot->column2.z = dst->mPreca0b_PaddedAligned.z = epsilon + fabsf(boxToModelR->column2.z);
	}

	// PT: precompute the extent data used in the OBB-OBB overlap test. This is separate from setupBoxBoxRotationData(),
	// because the extents can shrink during sweep traversals (while the rotation is constant).
	template<class ParamsT>
	PX_FORCE_INLINE	void setupBoxBoxExtentData(ParamsT* PX_RESTRICT dst, const PxVec3& extents, const PxMat33* PX_RESTRICT absRot)
	{
		dst->mBoxExtents_PaddedAligned = extents;

		const float Ex = extents.x;
		const float Ey = extents.y;
		const float Ez = extents.z;
		//dst->mBB_PaddedAligned.x = Ex*absRot->column0.x + Ey*absRot->column1.x + Ez*absRot->column2.x;
		//dst->mBB_PaddedAligned.y = Ex*absRot->column0.y + Ey*absRot->column1.y + Ez*absRot->column2.y;
		//dst->mBB_PaddedAligned.z = Ex*absRot->column0.z + Ey*absRot->column1.z + Ez*absRot->column2.z;
		// PT: the above code with absRot should be equivalent to:
		PX_ASSERT(absRot->column0.x==dst->mPreca0b_PaddedAligned.x);
		PX_ASSERT(absRot->column0.y==dst->mPreca1b_PaddedAligned.x);
		PX_ASSERT(absRot->column0.z==dst->mPreca2b_PaddedAligned.x);

		PX_ASSERT(absRot->column1.x==dst->mPreca2b_PaddedAligned.y);
		PX_ASSERT(absRot->column1.y==dst->mPreca0b_PaddedAligned.y);
		PX_ASSERT(absRot->column1.z==dst->mPreca1b_PaddedAligned.y);

		PX_ASSERT(absRot->column2.x==dst->mPreca1b_PaddedAligned.z);
		PX_ASSERT(absRot->column2.y==dst->mPreca2b_PaddedAligned.z);
		PX_ASSERT(absRot->column2.z==dst->mPreca0b_PaddedAligned.z);
		PX_UNUSED(absRot);

		dst->mBB_PaddedAligned.x = Ex*dst->mPreca0b_PaddedAligned.x + Ey*dst->mPreca2b_PaddedAligned.y + Ez*dst->mPreca1b_PaddedAligned.z;
		dst->mBB_PaddedAligned.y = Ex*dst->mPreca1b_PaddedAligned.x + Ey*dst->mPreca0b_PaddedAligned.y + Ez*dst->mPreca2b_PaddedAligned.z;
		dst->mBB_PaddedAligned.z = Ex*dst->mPreca2b_PaddedAligned.x + Ey*dst->mPreca1b_PaddedAligned.y + Ez*dst->mPreca0b_PaddedAligned.z;
	}

	struct OBBTestParams	// Data needed to perform the OBB-OBB overlap test
	{
		BV4_ALIGN16(PxVec3p	mCenterOrMinCoeff_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mExtentsOrMaxCoeff_PaddedAligned);

		BV4_ALIGN16(PxVec3p	mTBoxToModel_PaddedAligned);	// (1) Translation from obb space to model space
		BV4_ALIGN16(PxVec3p	mBB_PaddedAligned);				// (2)
		BV4_ALIGN16(PxVec3p	mBoxExtents_PaddedAligned);		// (3)

		BV4_ALIGN16(PxVec3p	mPreca0_PaddedAligned);			// (4)
		BV4_ALIGN16(PxVec3p	mPreca1_PaddedAligned);			//
		BV4_ALIGN16(PxVec3p	mPreca2_PaddedAligned);			//

		BV4_ALIGN16(PxVec3p	mPreca0b_PaddedAligned);		// (5)
		BV4_ALIGN16(PxVec3p	mPreca1b_PaddedAligned);		//
		BV4_ALIGN16(PxVec3p	mPreca2b_PaddedAligned);		//

		// PT: precompute the full data used in the OBB-OBB overlap test.
		PX_FORCE_INLINE	void	setupFullBoxBoxData(const PxVec3& center, const PxVec3& extents, const PxMat33* PX_RESTRICT box_to_model)
		{
			// PT: setup (1)
			mTBoxToModel_PaddedAligned = center;

			// PT: setup (4) and (5)
			PxMat33	absRot;	//!< Absolute rotation matrix
			setupBoxBoxRotationData(this, &absRot, box_to_model);

			// PT: setup (2) and (3)
			setupBoxBoxExtentData(this, extents, &absRot);
		}
	};

#endif // GU_BV4_BOX_OVERLAP_INTERNAL_H
