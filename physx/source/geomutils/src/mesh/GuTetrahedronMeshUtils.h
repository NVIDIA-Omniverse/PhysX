// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_TETRAHEDRONMESHUTILS_H
#define GU_TETRAHEDRONMESHUTILS_H

#include <GuTetrahedronMesh.h>

namespace physx
{
namespace Gu
{

PX_PHYSX_COMMON_API
void convertDeformableVolumeCollisionToSimMeshTets(const PxTetrahedronMesh& simMesh, const DeformableVolumeAuxData& simState, const BVTetrahedronMesh& collisionMesh,
												   PxU32 inTetId, const PxVec4& inTetBarycentric, PxU32& outTetId, PxVec4& outTetBarycentric, bool bClampToClosestPoint = true);

}
}

#endif
