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

#ifndef GU_SECONDARY_PRUNER_H
#define GU_SECONDARY_PRUNER_H

#include "common/PxPhysXCommonConfig.h"
#include "GuPruner.h"

namespace physx
{
	class PxRenderOutput;

namespace Gu
{
	class PruningPool;

	class CompanionPruner : public PxUserAllocated
	{
		public:
						CompanionPruner()	{}
		virtual			~CompanionPruner()	{}

		virtual bool	addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp, PoolIndex poolIndex)	= 0;
		virtual	bool	updateObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PoolIndex poolIndex)				= 0;
		virtual	bool	removeObject(const PrunerPayload& object, PrunerHandle handle, PxU32 objectIndex, PxU32 swapObjectIndex)													= 0;
		virtual	void	swapIndex(PxU32 objectIndex, PxU32 swapObjectIndex)																											= 0;
		virtual	PxU32	removeMarkedObjects(PxU32 timeStamp)																														= 0;
		virtual	void	shiftOrigin(const PxVec3& shift)																															= 0;
		virtual	void	timeStampChange()																																			= 0;
		virtual	void	build()																																						= 0;
		virtual	PxU32	getNbObjects()																																		const	= 0;
		virtual	void	release()																																					= 0;
		virtual	void	visualize(PxRenderOutput& out, PxU32 color)																											const	= 0;
		virtual	bool	raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)									const	= 0;
		virtual	bool	overlap(const ShapeData& queryVolume, PrunerOverlapCallback& prunerCallback)																		const	= 0;
		virtual	bool	sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback& prunerCallback)							const	= 0;
		virtual	void	getGlobalBounds(PxBounds3&)																															const	= 0;
	};

	CompanionPruner* createCompanionPruner(PxU64 contextID, CompanionPrunerType type, const PruningPool* pool);
}
}

#endif
