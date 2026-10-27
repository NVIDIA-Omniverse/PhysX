// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_MATERIAL_METHOD_H
#define PXC_MATERIAL_METHOD_H

#include "geometry/PxGeometry.h"

namespace physx
{
struct PxsShapeCore;
struct PxsMaterialInfo;
class PxContactBuffer;

#define MATERIAL_METHOD_ARGS				\
	const PxsShapeCore* shape0,				\
	const PxsShapeCore* shape1,				\
	const PxContactBuffer& contactBuffer,	\
	PxsMaterialInfo* materialInfo

#define SINGLE_MATERIAL_METHOD_ARGS			\
	const PxsShapeCore* shape,				\
	PxU32 index,							\
	const PxContactBuffer& contactBuffer,	\
	PxsMaterialInfo* materialInfo

/*!
Method prototype for fetch material routines
*/
typedef void (*PxcGetMaterialMethod) (MATERIAL_METHOD_ARGS);

typedef void (*PxcGetSingleMaterialMethod) (SINGLE_MATERIAL_METHOD_ARGS);

extern PxcGetMaterialMethod g_GetMaterialMethodTable[][PxGeometryType::eGEOMETRY_COUNT];

extern PxcGetSingleMaterialMethod g_GetSingleMaterialMethodTable[PxGeometryType::eGEOMETRY_COUNT];

}

#endif
