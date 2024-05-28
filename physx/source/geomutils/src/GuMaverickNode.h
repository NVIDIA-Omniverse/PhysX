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

#ifndef GU_MAVERICK_NODE_H
#define GU_MAVERICK_NODE_H

#include "foundation/PxBounds3.h"
#include "foundation/PxTransform.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuPrunerPayload.h"
#include "GuPrunerTypedef.h"

#define FREE_PRUNER_SIZE	16

#ifdef FREE_PRUNER_SIZE

namespace physx
{
namespace Gu
{
	class MaverickNode
	{
		public:
										MaverickNode() : mNbFree(0)	{}
										~MaverickNode()				{}

		PX_FORCE_INLINE	void			release()								{ mNbFree = 0;		}
		PX_FORCE_INLINE	const PxU32*	getPrimitives(const PxU32*)		const	{ return mIndices;	}
		PX_FORCE_INLINE	PxU32			getPrimitiveIndex()				const	{ return 0;			}
		PX_FORCE_INLINE	PxU32			getNbPrimitives()				const	{ return mNbFree;	}

						bool			addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp);

						bool			updateObject(const PrunerPayload& object, const PxBounds3& worldAABB, const PxTransform& transform);
						bool			updateObject(PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform);

						bool			removeObject(const PrunerPayload& object, PxU32& timeStamp);
						bool			removeObject(PrunerHandle handle, PxU32& timeStamp);

						PxU32			removeMarkedObjects(PxU32 timeStamp);
						void			shiftOrigin(const PxVec3& shift);

						void			remove(PxU32 index);

						PxU32			mNbFree;							// Current number of objects in the "free array" (mFreeObjects/mFreeBounds)
						PrunerPayload	mFreeObjects[FREE_PRUNER_SIZE];		// mNbFree objects are stored here
						PrunerHandle	mFreeHandles[FREE_PRUNER_SIZE];		// mNbFree handles are stored here
						PxBounds3		mFreeBounds[FREE_PRUNER_SIZE];		// mNbFree object bounds are stored here
						PxTransform		mFreeTransforms[FREE_PRUNER_SIZE];	// mNbFree transforms are stored here
						PxU32			mFreeStamps[FREE_PRUNER_SIZE];
		static			const PxU32		mIndices[FREE_PRUNER_SIZE];
	};
}
}

#endif
#endif
