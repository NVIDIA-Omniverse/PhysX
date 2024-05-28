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

#ifndef DY_CONSTRAINT_WRITE_BACK_H
#define DY_CONSTRAINT_WRITE_BACK_H

#include "foundation/PxVec3.h"
#include "PxvConfig.h"
#include "PxvDynamics.h"

namespace physx
{
	namespace Dy
	{

		PX_ALIGN_PREFIX(16)
		struct ConstraintWriteback
		{
			void initialize()
			{
				linearImpulse = PxVec3(0);
				angularImpulse = PxVec3(0);
				residualPosIter = 0.0f;
				residual = 0.0f;
			}

			PxVec3	linearImpulse;
		private:
			union
			{
				PxU32	broken;
				PxReal	residualPosIter;
			};
		public:
			PxVec3  angularImpulse;
			PxReal	residual;

			PX_FORCE_INLINE PxU32 setBit(PxU32 value, PxU32 bitLocation, bool bitState)
			{
				if (bitState)
					return value | (1 << bitLocation);
				else
					return value & (~(1 << bitLocation));
			}

			PX_FORCE_INLINE bool isBroken() const { return broken & PX_SIGN_BITMASK; }
			
			PX_FORCE_INLINE PxReal getPositionIterationResidual() const { return PxAbs(residualPosIter); }
			
			PX_FORCE_INLINE void setCombined(bool isBroken, PxReal positionIterationResidual)
			{
				residualPosIter = positionIterationResidual;
				broken = setBit(broken, 31, isBroken);
			}

		}
		PX_ALIGN_SUFFIX(16);

	}
}

#endif
