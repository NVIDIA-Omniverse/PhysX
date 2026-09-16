// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_VOLUME_INTEGRATION_H
#define GU_COOKING_VOLUME_INTEGRATION_H

#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"

namespace physx
{
class PxSimpleTriangleMesh;
class PxConvexMeshDesc;

/**
\brief Data structure used to store mass properties.
*/
struct PxIntegrals
{
	PxVec3 COM;						//!< Center of mass
	PxF64 mass;						//!< Total mass
	PxF64 inertiaTensor[3][3];		//!< Inertia tensor (mass matrix) relative to the origin
	PxF64 COMInertiaTensor[3][3];	//!< Inertia tensor (mass matrix) relative to the COM

	/**
	\brief Retrieve the inertia tensor relative to the center of mass.

	\param inertia Inertia tensor.
	*/
	void getInertia(PxMat33& inertia)
	{
		for(PxU32 j=0;j<3;j++)
		{
			for(PxU32 i=0;i<3;i++)
			{
				inertia(i,j) = PxF32(COMInertiaTensor[i][j]);
			}
		}
	}

	/**
	\brief Retrieve the inertia tensor relative to the origin.

	\param inertia Inertia tensor.
	*/
	void getOriginInertia(PxMat33& inertia)
	{
		for(PxU32 j=0;j<3;j++)
		{
			for(PxU32 i=0;i<3;i++)
			{
				inertia(i,j) = PxF32(inertiaTensor[i][j]);
			}
		}
	}
};

	bool computeVolumeIntegrals(const PxSimpleTriangleMesh& mesh, PxReal density, PxIntegrals& integrals);

	// specialized method taking polygons directly, so we don't need to compute and store triangles for each polygon
	bool computeVolumeIntegralsEberly(const PxConvexMeshDesc& mesh, PxReal density, PxIntegrals& integrals, const PxVec3& origin, bool useSimd);   // Eberly simplified method
}

#endif
