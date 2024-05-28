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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "foundation/PxErrorCallback.h"
#include "ScHairSystemShapeCore.h"
#include "ScHairSystemShapeSim.h"
#include "ScPhysics.h"
#include "PxvGlobals.h"
#include "PxPhysXGpu.h"

using namespace physx;
using namespace Sc;

namespace physx
{
namespace Sc
{

HairSystemShapeCore::HairSystemShapeCore()
	: ShapeCore(PxEmpty)
	, mGpuMemStat(0)
{
	mSimulationFilterData = PxFilterData();

	mCore = PxsShapeCore();

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mCore.setTransform(PxTransform(PxIdentity));
	mCore.mContactOffset = 0.0f;
	mCore.mRestOffset = 0.0f;
	mCore.mShapeFlags = 0;

	mCore.mMinTorsionalPatchRadius = 0.f;
	mCore.mTorsionalRadius = 0.f;

	mLLCore.mSleepThreshold = 5e-5f * scale.speed * scale.speed;
	mLLCore.mWakeCounter = Physics::sWakeCounterOnCreation;
}

// PX_SERIALIZATION
HairSystemShapeCore::HairSystemShapeCore(const PxEMPTY)
	: ShapeCore(PxEmpty)
{
}

HairSystemShapeCore::~HairSystemShapeCore()
{
	releaseBuffers();
}

void HairSystemShapeCore::createBuffers(PxCudaContextManager* cudaContextManager)
{
	mCudaContextManager = cudaContextManager;

	// nothing else to do at the moment
}

void HairSystemShapeCore::releaseBuffers()
{
	// nothing to do at the moment
}

} // namespace Sc
} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
