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

#ifndef EXT_QUADRIC_H
#define EXT_QUADRIC_H

#include "foundation/PxVec3.h"

// MM: used in ExtMeshSimplificator
// see paper Garland and Heckbert: "Surface Simplification Using Quadric Error Metrics"

namespace physx
{
	namespace Ext
	{
		class Quadric {
		public:
			PX_FORCE_INLINE void zero()
			{
				a00 = 0.0f; a01 = 0.0f; a02 = 0.0f; a03 = 0.0f;
				a11 = 0.0f; a12 = 0.0f; a13 = 0.0f;
				a22 = 0.0f; a23 = 0.0f;
				a33 = 0.0f;
			}

			PX_FORCE_INLINE void setFromPlane(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2)
			{
				PxVec3 n = (v1 - v0).cross(v2 - v0); n.normalize();
				float d = -n.dot(v0);
				a00 = n.x * n.x; a01 = n.x * n.y; a02 = n.x * n.z; a03 = n.x * d;
				a11 = n.y * n.y; a12 = n.y * n.z; a13 = n.y * d;
				a22 = n.z * n.z; a23 = n.z * d;
				a33 = d * d;
			}

			PX_FORCE_INLINE Quadric operator +(const Quadric& q) const
			{
				Quadric sum;
				sum.a00 = a00 + q.a00; sum.a01 = a01 + q.a01; sum.a02 = a02 + q.a02; sum.a03 = a03 + q.a03;
				sum.a11 = a11 + q.a11; sum.a12 = a12 + q.a12; sum.a13 = a13 + q.a13;
				sum.a22 = a22 + q.a22; sum.a23 = a23 + q.a23;
				sum.a33 = a33 + q.a33;
				return sum;
			}

			void operator +=(const Quadric& q)
			{
				a00 += q.a00; a01 += q.a01; a02 += q.a02; a03 += q.a03;
				a11 += q.a11; a12 += q.a12; a13 += q.a13;
				a22 += q.a22; a23 += q.a23;
				a33 += q.a33;
			}

			PxF32 outerProduct(const PxVec3& v)
			{
				return a00 * v.x * v.x + 2.0f * a01 * v.x * v.y + 2.0f * a02 * v.x * v.z + 2.0f * a03 * v.x +
					a11 * v.y * v.y + 2.0f * a12 * v.y * v.z + 2.0f * a13 * v.y +
					a22 * v.z * v.z + 2.0f * a23 * v.z + a33;
			}
		private:
			PxF32 a00, a01, a02, a03;
			PxF32      a11, a12, a13;
			PxF32           a22, a23;
			PxF32                a33;

		};
	}
}

#endif
