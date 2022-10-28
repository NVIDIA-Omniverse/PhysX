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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_MTD_H
#define GU_MTD_H

#include "foundation/PxVec3.h"
#include "geometry/PxGeometry.h"

namespace physx
{
namespace Gu
{
	// PT: we use a define to be able to quickly change the signature of all MTD functions.
	// (this also ensures they all use consistent names for passed parameters).
	// \param[out]	mtd		computed depenetration dir
	// \param[out]	depth	computed depenetration depth
	// \param[in]	geom0	first geometry object
	// \param[in]	pose0	pose of first geometry object
	// \param[in]	geom1	second geometry object
	// \param[in]	pose1	pose of second geometry object
	// \param[in]	cache	optional cached data for triggers
	#define GU_MTD_FUNC_PARAMS	PxVec3& mtd, PxF32& depth,							\
								const PxGeometry& geom0, const PxTransform& pose0,	\
								const PxGeometry& geom1, const PxTransform& pose1

	// PT: function pointer for Geom-indexed MTD functions
	// See GU_MTD_FUNC_PARAMS for function parameters details.
	// \return		true if an overlap was found, false otherwise
	// \note		depenetration vector D is equal to mtd * depth. It should be applied to the 1st object, to get out of the 2nd object.
	typedef bool (*GeomMTDFunc)	(GU_MTD_FUNC_PARAMS);

	PX_FORCE_INLINE PxF32 manualNormalize(PxVec3& mtd, const PxVec3& normal, PxReal lenSq)
	{
		const PxF32 len = PxSqrt(lenSq);

		// We do a *manual* normalization to check for singularity condition
		if(lenSq < 1e-6f)
			mtd = PxVec3(1.0f, 0.0f, 0.0f);			// PT: zero normal => pick up random one
		else
			mtd = normal * 1.0f / len;

		return len;
	}
}
}

#endif
