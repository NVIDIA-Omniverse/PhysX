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

#ifndef PXG_CONSTRAINT_ID_MAP_H
#define PXG_CONSTRAINT_ID_MAP_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxAssert.h"

#if PX_CUDA_COMPILER
#include "assert.h"
#endif

namespace physx
{
	// maps the constraint/joint ID to the internal joint data ID. This is used for direct GPU API
	// operations to use the same constraint/joint ID on the public interface level as long as the
	// constraint/joint stays in the same scene. In particular, changing actors of a constraint/joint
	// should be transparent to users and can be achieved using this map.
	class PxgConstraintIdMapEntry
	{
	public:
		static const PxU32 eINVALID_ID = 0xffffFFFF;

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxgConstraintIdMapEntry()
			: mJointDataId(eINVALID_ID)
		{
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void invalidate()
		{
			mJointDataId = eINVALID_ID;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void setJointDataId(PxU32 jointDataId)
		{
#if PX_CUDA_COMPILER
			assert(jointDataId < eINVALID_ID);  // until PX_ASSERT works on GPU (see PX-4133)
#else
			PX_ASSERT(jointDataId < eINVALID_ID);
#endif
			mJointDataId = jointDataId;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getJointDataId() const
		{
			return mJointDataId;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE bool isJointDataIdValid() const { return (mJointDataId != eINVALID_ID); }

	private:
		// maps the constraint/joint ID to the internal GPU joint data ID. eINVALID_ID is used if the 
		// joint/constraint is unmapped/removed or inactive.
		PxU32 mJointDataId;
	};
}

#endif  // PXG_CONSTRAINT_ID_MAP_H
