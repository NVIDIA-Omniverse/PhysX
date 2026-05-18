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

#ifndef PXV_GLOBALS_H
#define PXV_GLOBALS_H

#include "PxPhysXConfig.h"
#include "foundation/PxBasicTemplates.h"
#include "PxContactModifyCallback.h"

namespace physx
{
class PxShape;
class PxRigidActor;
struct PxsShapeCore;
struct PxsRigidCore;

struct PxvOffsetTable
{
	PX_FORCE_INLINE	void fillPairPointers(	PxContactModifyPair& p,
											const PxsShapeCore* PX_RESTRICT shapeCore0, const PxsShapeCore* PX_RESTRICT shapeCore1,
											const PxsRigidCore* PX_RESTRICT rigidCore0, const PxsRigidCore* PX_RESTRICT rigidCore1,
											bool isDynamic0, bool isDynamic1)
	{
		p.shape[0] = PxPointerOffset<const PxShape*>(shapeCore0, pxsShapeCore2PxShape);
		p.shape[1] = PxPointerOffset<const PxShape*>(shapeCore1, pxsShapeCore2PxShape);

		p.actor[0] = PxPointerOffset<const PxRigidActor*>(rigidCore0, isDynamic0 ? pxsRigidCore2PxRigidBody : pxsRigidCore2PxRigidStatic);
		p.actor[1] = PxPointerOffset<const PxRigidActor*>(rigidCore1, isDynamic1 ? pxsRigidCore2PxRigidBody : pxsRigidCore2PxRigidStatic);
	}

	ptrdiff_t	pxsShapeCore2PxShape;
	ptrdiff_t	pxsRigidCore2PxRigidBody;
	ptrdiff_t	pxsRigidCore2PxRigidStatic;
};
extern PxvOffsetTable gPxvOffsetTable;

/*!
Initialize low-level implementation.
*/
void PxvInit(const PxvOffsetTable& offsetTable);

/*!
Shut down low-level implementation.
*/
void PxvTerm();

#if PX_SUPPORT_GPU_PHYSX
class PxPhysXGpu* PxvGetPhysXGpu(bool createIfNeeded);
void PxvReleasePhysXGpu(PxPhysXGpu*);
#endif

}

#endif
