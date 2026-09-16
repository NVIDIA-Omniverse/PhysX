// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_BUCKET_H
#define PXG_BUCKET_H

#include "foundation/PxSimpleTypes.h"
#include "PxgPersistentContactManifold.h"

namespace physx
{
	struct GPU_BUCKET_ID
	{
		enum Enum
		{
			eFallback = 0,
			eConvex = 1,						//manifold
			eConvexPlane = 2,					//manifold
			eConvexTrimesh = 3,					//multi-manifold
			eConvexHeightfield = 4,				//multi-manifold
			eSphereTrimesh = 5,					//multi-manifold
			eSphereHeightfield = 6,				//multi-manifold
			eTrianglePlane = 7,					//multi-manifold - technically could be single manifold but needs 6 points

			eSphere = 8,						//no manifold
			eBoxBox = 9,

			eConvexCorePlane = 10,
			eConvexCoreConvex = 11,
			eConvexCoreTrimesh = 12,
			eConvexCoreTetmesh = 13,
			eConvexCoreClothmesh = 14,

			eTriangleHeightfield = 15,			//no mainfold
			eTriangleTriangle = 16,				//no manifold
			eSoftbody = 17,						//no manifold
			eSoftbodies = 18,					//no manifold
			eSoftbodyFemCloth = 19,				//no manifold
			eSoftbodyTrimesh = 20,				//no manifold
			eSoftbodySdfTrimesh = 21,			//no manifold
			eSoftbodyHeightfield = 22,			//no manifold
			eFemClothSphere = 23,				//no manifold
			eFemClothPlane = 24,				//no manifold
			eFemClothBox = 25,					//no manifold
			eFemClothConvexes = 26,				//no manifold
			eFemCloths = 27,					//no manifold
			eFemClothTrimesh = 28,				//no manifold
			eFemClothSdfTrimesh = 29,
			eFemClothHeightfield = 30,			//no manifold
			eConvexParticle = 31,				//no manifold
			eParticlesystems = 32,				//no manifold
			eParticlesystemSoftbody = 33,		//no manifold
			eParticlesystemFemCloth = 34,		//no manifold
			eParticlesystemTrimesh = 35,		//no manifold
			eParticlesystemSdfTrimesh = 36,		//no manifold
			eParticlesystemHeightfield = 37,	//no manifold

			eCount
		};
	};

	const PxU32 BUCKET_ManifoldSize[GPU_BUCKET_ID::eCount] = {
		0,														//0
		sizeof(PxgPersistentContactManifold),					//1
		sizeof(PxgPersistentContactManifold),					//2
		sizeof(PxgPersistentContactMultiManifold),				//3
		sizeof(PxgPersistentContactMultiManifold),				//4
		sizeof(PxgPersistentContactMultiManifold),				//5
		sizeof(PxgPersistentContactMultiManifold),				//6
		sizeof(PxgPersistentContactMultiManifold),				//7
		0,														//8
		0,														//9
		0,														//10
		0,														//11
		0,														//12
		0,														//13
		0,														//14
		0,														//15
		0,														//16
		0,														//17
		0,														//18
		0,														//19
		0,														//20
		0,														//21
		0,														//22
		0,														//23
		0,														//24
		0,														//25
		0,														//26
		0,														//27
		0,														//28
		0,														//29
		0,														//30
		0,														//31
		0,														//32
		0,														//33
		0,														//34
		0,														//35
		0,														//36
		0,														//37
	};
}

#endif
