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

#ifndef DY_DEFORMABLE_BODY_CORE_H
#define DY_DEFORMABLE_BODY_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxArray.h"

namespace physx
{
namespace Dy
{

struct DeformableBodyCore
{
public:
	//PxFEMParameters
	PxReal					linearDamping;
	PxReal					settlingThreshold;
	PxReal					sleepThreshold;
	PxReal					settlingDamping;
	PxReal					selfCollisionFilterDistance;
	PxReal					selfCollisionStressTolerance;
	//~PxFEMParameters

	PxReal					maxVelocity;
	PxReal					maxDepenetrationVelocity;

	PxU16					solverIterationCounts; //vel iters are in low word and pos iters in high word.
	PxArray<PxU16>			materialHandles;
	PxReal					wakeCounter;

	PxDeformableBodyFlags	bodyFlags;
	bool					dirty;

	DeformableBodyCore()
		//PxFEMParameters, same defaults as deprecated PxFEMParameters
		: linearDamping(0.05f)
		, settlingThreshold(0.1f)
		, sleepThreshold(0.05f)
		, settlingDamping(10.f)
		, selfCollisionFilterDistance(0.1f)
		, selfCollisionStressTolerance(0.9f)
		//~PxFEMParameters
		, maxVelocity(PX_MAX_REAL)
		, maxDepenetrationVelocity(0.0f)
		, solverIterationCounts(0)
		, wakeCounter(0)
		, bodyFlags(0)
		, dirty(false)
	{
	}
};

} // namespace Dy
} // namespace physx

#endif // DY_DEFORMABLE_CORE_H

