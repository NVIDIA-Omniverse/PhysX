// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_MESH_CLEANER_H
#define GU_MESH_CLEANER_H

#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	class MeshCleaner
	{
		public:
			MeshCleaner(PxU32 nbVerts, const PxVec3* verts, PxU32 nbTris, const PxU32* indices, PxF32 meshWeldTolerance, PxF32 areaLimit);
			~MeshCleaner();

			PxU32	mNbVerts;
			PxU32	mNbTris;
			PxVec3*	mVerts;
			PxU32*	mIndices;
			PxU32*	mRemap;
	};
}
}

#endif

