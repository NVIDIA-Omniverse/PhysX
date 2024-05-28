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
