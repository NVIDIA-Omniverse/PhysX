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

#ifndef PX_EXTENDED_H
#define PX_EXTENDED_H

// This needs to be included in Foundation just for the debug renderer

#include "PxPhysXConfig.h"
#include "foundation/PxTransform.h"
#include "foundation/PxAssert.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

// This has to be done here since it also changes the top-level "Px" and "Np" APIs
#define PX_BIG_WORLDS

#ifdef PX_BIG_WORLDS
	typedef PxVec3d	PxExtendedVec3;
	typedef	double	PxExtended;
	#define	PX_MAX_EXTENDED	PX_MAX_F64

	PX_FORCE_INLINE PxVec3 toVec3(const PxExtendedVec3& v)
	{
		return PxVec3(float(v.x), float(v.y), float(v.z));
	}

	// Computes the single-precision difference between two extended-precision points
	PX_INLINE	PxVec3	diff(const PxExtendedVec3& p1, const PxExtendedVec3& p0)
	{
		return PxVec3(float(p1.x - p0.x), float(p1.y - p0.y), float(p1.z - p0.z));
	}
#else
	typedef	PxVec3	PxExtendedVec3;
	typedef	float	PxExtended;
	#define	PX_MAX_EXTENDED	PX_MAX_F32

	PX_FORCE_INLINE PxVec3 toVec3(const PxExtendedVec3& v)
	{
		return v;
	}

	// Computes the single-precision difference between two extended-precision points
	PX_INLINE	PxVec3	diff(const PxExtendedVec3& p1, const PxExtendedVec3& p0)
	{
		return p1 - p0;
	}
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
