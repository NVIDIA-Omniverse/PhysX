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

#ifndef DY_V_ARTICULATION_H
#define DY_V_ARTICULATION_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxUtilities.h"
#include "CmUtils.h"
#include "CmSpatialVector.h"
#include "foundation/PxMemory.h"
#include "DyArticulationCore.h"
#include "DyArticulationJointCore.h"
#include "DyArticulationMimicJointCore.h"

namespace physx
{
	struct PxsBodyCore;
	class PxsConstraintBlockManager;
	class PxsContactManagerOutputIterator;
	struct PxSolverConstraintDesc;
	struct PxSolverBodyData;
	class PxContactJoint;
	struct PxTGSSolverBodyData;
	struct PxTGSSolverBodyTxInertia;
	struct PxSolverConstraintDesc;

	namespace Dy
	{
		
		struct SpatialSubspaceMatrix;
		
		struct ConstraintWriteback;
		class ThreadContext;

		static const size_t DY_ARTICULATION_TENDON_MAX_SIZE = 64;
	
		struct Constraint;
		class Context;
		class ArticulationSpatialTendon;
		class ArticulationFixedTendon;
		class ArticulationTendonJoint;

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
			PxU32						mPathToRootStartIndex;
			PxU32						mChildrenStartIndex;
			PxU16						mPathToRootCount;
			PxU16						mNumChildren;
			PxsBodyCore*				bodyCore;
			ArticulationJointCore*		inboundJoint;
			PxU32						parent;
			PxReal						cfm;
		};

		class FeatherstoneArticulation;

		struct ArticulationSolverDesc
		{
			void	initData(ArticulationCore* core_, const PxArticulationFlags* flags_)
			{
				articulation			= NULL;
				links					= NULL;
				motionVelocity			= NULL;
				acceleration			= NULL;
				poses					= NULL;
				deltaQ					= NULL;
	
				core					= core_;
				flags					= flags_;
				linkCount				= 0;
				numInternalConstraints	= 0;
				
			}

			FeatherstoneArticulation*	articulation;
			ArticulationLink*			links;
			Cm::SpatialVectorV*			motionVelocity;
			Cm::SpatialVector*			acceleration;
			PxTransform*				poses;
			PxQuat*						deltaQ;
	
			ArticulationCore*			core;
			const PxArticulationFlags*	flags;	// PT: PX-1399

			PxU8						linkCount;
			PxU8						numInternalConstraints;
			
		};

	}

}

#endif
