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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved. 

#ifndef PXG_PERSISTENT_CONTACT_MANIFOLD_H
#define PXG_PERSISTENT_CONTACT_MANIFOLD_H

#include "cutil_math.h"
#include "AlignedTransform.h"

#define PXG_MAX_PCM_CONTACTS 4

#define PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS	4
#define PXG_SUBMANIFOLD_MAX_CONTACTS		6

namespace physx
{
	struct PX_ALIGN_PREFIX(16) PxgPersistentContactManifold
	{
		float4 mLocalNormal_PenW[PXG_MAX_PCM_CONTACTS];			//local normal + penetration
		float4 mLocalContactA[PXG_MAX_PCM_CONTACTS];			//local contact in body A's space
		float4 mLocalContactB[PXG_MAX_PCM_CONTACTS];			//local contact in body B's space
		float4 mRelativePos;									//Cached relative transforms
		PxAlignedQuat mQuatA;									//Cached body A's quatenion
		PxAlignedQuat mQuatB;									//Cached body B's quatenion
		PxU32 mNbContacts;										//NbContacts
		PxU32 mNbWarmStartPoints;								//Nb warm start points
		PxU32 mWarmStartA;										//GJK warm-start mask for shape A
		PxU32 mWarmStartB;										//GJK warm-start mask for shape B

		PX_FORCE_INLINE void clear()
		{
			mNbContacts = 0;
			mNbWarmStartPoints = 0;
			mRelativePos = make_float4(PX_MAX_REAL, PX_MAX_REAL, PX_MAX_REAL, PX_MAX_REAL);
			mQuatA = PxAlignedQuat(0.f, 0.f, 0.f, 1.f);
			mQuatB = PxAlignedQuat(0.f, 0.f, 0.f, 1.f);
		}

		//KS - TODO - extend and add functionality as is required...
	}
	PX_ALIGN_SUFFIX(16);

	struct PX_ALIGN_PREFIX(16) PxgContact
	{
		   PxVec3 pointA;
		   PxVec3 pointB;
		   PxVec3 normal;
		   PxReal penetration;
		   PxU32 triIndex;
		   PxU32 pad;
	} // 48 due to padding
	PX_ALIGN_SUFFIX(16);


	struct PX_ALIGN_PREFIX(16) PxgPersistentContactMultiManifold
	{
		PxgContact mContacts[PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS][PXG_SUBMANIFOLD_MAX_CONTACTS];	// 1152
		PxU32 mNbContacts[PXG_MULTIMANIFOLD_MAX_SUBMANIFOLDS];									// 1168
		PxAlignedTransform mRelativeTransform;													// 1200
		PxU32 mNbManifolds;																		// 1216 due to padding
		

		PX_FORCE_INLINE void clear()
		{
			mNbManifolds = 0;
			mRelativeTransform.p = make_float4(PX_MAX_REAL, PX_MAX_REAL, PX_MAX_REAL, PX_MAX_REAL);
			mRelativeTransform.q = PxAlignedQuat(0.f, 0.f, 0.f, 1.f);
		}
	}
	PX_ALIGN_SUFFIX(16);
}

#endif
