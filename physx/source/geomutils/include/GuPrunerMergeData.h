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

#ifndef GU_PRUNER_MERGE_DATA_H
#define GU_PRUNER_MERGE_DATA_H
/** \addtogroup physics
@{ */

#include "foundation/PxSimpleTypes.h"
namespace physx
{
	namespace Gu
	{
		struct BVHNode;

		// PT: TODO: refactor with BVHCoreData ?
		struct AABBPrunerMergeData
		{
			AABBPrunerMergeData()
			{
				// PT: it's important to NOT initialize anything by default (for binary serialization)
			}

			PxU32		mNbNodes;			// Nb nodes in AABB tree
			BVHNode*	mAABBTreeNodes;		// AABB tree runtime nodes
			PxU32		mNbObjects;			// Nb objects in AABB tree
			PxU32*		mAABBTreeIndices;	// AABB tree indices

			void init(PxU32 nbNodes=0, BVHNode* nodes=NULL, PxU32 nbObjects=0, PxU32* indices=NULL)
			{
				mNbNodes			= nbNodes;
				mAABBTreeNodes		= nodes;
				mNbObjects			= nbObjects;
				mAABBTreeIndices	= indices;
			}
		};
	}
}

/** @} */
#endif
