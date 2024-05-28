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

#ifndef DY_CONSTRAINT_H
#define DY_CONSTRAINT_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "PxvConfig.h"
#include "PxvDynamics.h"
#include "PxConstraint.h"
#include "DyConstraintWriteBack.h"

namespace physx
{

class PxsRigidBody;

namespace Dy
{

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif
PX_ALIGN_PREFIX(16)
struct Constraint
{
public:

	PxReal					linBreakForce;
	PxReal					angBreakForce;
	PxU16					constantBlockSize;
	PxU16					flags;

	PxConstraintSolverPrep	solverPrep;
	void*					constantBlock;

	PxsRigidBody*			body0;
	PxsRigidBody*			body1;

	PxsBodyCore*			bodyCore0;
	PxsBodyCore*			bodyCore1;
	PxU32					index;
	PxReal					minResponseThreshold;
}
PX_ALIGN_SUFFIX(16);
#if PX_VC 
     #pragma warning(pop) 
#endif

#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(48==sizeof(Constraint));
#endif

}

}

#endif
