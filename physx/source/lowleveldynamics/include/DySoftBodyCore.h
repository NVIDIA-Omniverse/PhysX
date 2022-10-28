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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#ifndef DY_SOFTBODY_CORE_H
#define DY_SOFTBODY_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "PxSoftBody.h"
#include "PxsFEMSoftBodyMaterialCore.h"
#include "foundation/PxArray.h"

namespace physx
{
	namespace Dy
	{
		struct SoftBodyCore
		{
		public:
			PxQuat					initialRotation;	
			
			PxFEMParameters			parameters;
			PxReal					sleepThreshold;
			PxReal					freezeThreshold;
			PxReal					wakeCounter;
			PxReal					maxPenBias;
			PxU16					solverIterationCounts;	//vel iters are in low word and pos iters in high word.
			bool					dirty;
			PxSoftBodyFlags			mFlags;

			
			void setMaterial(const PxU16 materialHandle)
			{
				mMaterialHandles.pushBack(materialHandle);
			}

			void clearMaterials() { mMaterialHandles.clear(); }

			PxArray<PxU16>			mMaterialHandles;

			//device
			PxBuffer*				mPositionInvMass;

			//host
			PxBuffer*				mPositionInvMassCPU;
			PxBuffer*				mRestPositionInvMassCPU;

			//device
			PxBuffer*				mSimPositionInvMass;
			PxBuffer*				mSimVelocityInvMass;
			PxBuffer*				mKinematicTarget;

			//host
			PxBuffer*				mSimPositionInvMassCPU;
			PxBuffer*				mSimVelocityInvMassCPU;
			PxBuffer*				mKinematicTargetCPU;
		};

	}
}

#endif

