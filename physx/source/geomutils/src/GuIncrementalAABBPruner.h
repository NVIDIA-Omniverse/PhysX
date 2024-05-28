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

#ifndef GU_INCREMENTAL_AABB_PRUNER_H
#define GU_INCREMENTAL_AABB_PRUNER_H

#include "common/PxPhysXCommonConfig.h"
#include "GuPruner.h"
#include "GuPruningPool.h"
#include "GuIncrementalAABBTree.h"
#include "GuSqInternal.h"

namespace physx
{
	class PxRenderOutput;

namespace Gu
{
	class IncrementalAABBPruner : public Pruner
	{
		public:
		PX_PHYSX_COMMON_API							IncrementalAABBPruner(PxU32 sceneLimit, PxU64 contextID);
		virtual										~IncrementalAABBPruner();

		// BasePruner
													DECLARE_BASE_PRUNER_API
		//~BasePruner

		// Pruner
													DECLARE_PRUNER_API_COMMON
		//~Pruner
		
		// direct access for test code
		PX_FORCE_INLINE	const IncrementalAABBTree*	getAABBTree()		const		{ return mAABBTree;	}
				
		private:
						void						release();
						void						fullRebuildAABBTree();
						void						test();
						void						updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node);

						IncrementalAABBTree*		mAABBTree; 						

						PruningPool					mPool; // Pool of AABBs

				PxArray<IncrementalAABBTreeNode*>	mMapping;

						PxU64						mContextID;
						NodeList					mChangedLeaves;
	};

}
}

#endif
