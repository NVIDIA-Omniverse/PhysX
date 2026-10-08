// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
