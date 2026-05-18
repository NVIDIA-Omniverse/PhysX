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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_V_ARTICULATION_H
#define DY_V_ARTICULATION_H

#include "DyArticulationJointCore.h"

namespace physx
{
	struct PxsBodyCore;

	namespace Dy
	{
		static const size_t DY_ARTICULATION_TENDON_MAX_SIZE = 64;
	
		struct Constraint;

		typedef PxU64 ArticulationBitField;

		struct ArticulationLoopConstraint
		{
		public:
			PxU32 linkIndex0;
			PxU32 linkIndex1;
			Dy::Constraint* constraint;
		};

#define DY_ARTICULATION_LINK_NONE 0xffffffff

		struct ArticulationLink
		{
			PxU32					mPathToRootStartIndex;
			PxU32					mChildrenStartIndex;
			PxU16					mPathToRootCount;
			PxU16					mNumChildren;
			PxsBodyCore*			bodyCore;
			ArticulationJointCore*	inboundJoint;
			PxU32					parent;
			PxReal					cfm;

			PX_FORCE_INLINE	void	initBody(PxsBodyCore* core)
			{
				bodyCore				= core;
				mPathToRootStartIndex	= 0;
				mPathToRootCount		= 0;
				mChildrenStartIndex		= 0xffffffff;
				mNumChildren			= 0;
			}

			PX_FORCE_INLINE	void	initJoint(ArticulationJointCore* core, PxU32 parentIndex)
			{
				inboundJoint	= core;
				parent			= parentIndex;
			}
		};
	}
}

#endif
