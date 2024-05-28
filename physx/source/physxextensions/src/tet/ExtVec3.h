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

#ifndef EXT_VEC3_H
#define EXT_VEC3_H

#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"

namespace physx
{
	namespace Ext
	{
		// ---------------------------------------------------------------------------------
		struct Bounds3 {
			Bounds3() {}
			Bounds3(const PxVec3d &min, const PxVec3d &max) : minimum(min), maximum(max) {}

			void setEmpty() {
				minimum = PxVec3d(PX_MAX_F64, PX_MAX_F64, PX_MAX_F64);
				maximum = PxVec3d(-PX_MAX_F64, -PX_MAX_F64, -PX_MAX_F64);
			}

			PxVec3d getDimensions() const {
				return maximum - minimum;
			}

			void include(const PxVec3d &p) {
				minimum = minimum.minimum(p);
				maximum = maximum.maximum(p);
			}

			void include(const Bounds3 &b) {
				minimum = minimum.minimum(b.minimum);
				maximum = maximum.maximum(b.maximum);
			}
			void expand(double d) {
				minimum -= PxVec3d(d, d, d);
				maximum += PxVec3d(d, d, d);
			}
			PxVec3d minimum, maximum;
		};
	}
}

#endif

