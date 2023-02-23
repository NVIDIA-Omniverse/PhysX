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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

#ifndef EXT_VEC3_H
#define EXT_VEC3_H

#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"

namespace physx
{
	namespace Ext
	{
		//3D vector class with double precision
		class Vec3
		{
		public:
			PxF64 x;
			PxF64 y;
			PxF64 z;

			PX_FORCE_INLINE Vec3() : x(0), y(0), z(0) {}

			PX_FORCE_INLINE Vec3(const Vec3 &other) { x = other.x; y = other.y; z = other.z; }

			PX_FORCE_INLINE Vec3(PxF64 x_, PxF64 y_, PxF64 z_) : x(x_), y(y_), z(z_)
			{}

			PX_FORCE_INLINE Vec3(PxF32 x_, PxF32 y_, PxF32 z_) : x(PxF64(x_)), y(PxF64(y_)), z(PxF64(z_))
			{}

			PX_FORCE_INLINE void set(PxF64 xCoord, PxF64 yCoord, PxF64 zCoord)
			{
				x = xCoord;
				y = yCoord;
				z = zCoord;
			}

			PX_FORCE_INLINE PxVec3 toFloat() const
			{
				return PxVec3(PxF32(x), PxF32(y), PxF32(z));
			}

			PX_FORCE_INLINE Vec3 max(const Vec3& lhs) const
			{
				return Vec3(PxMax(x, lhs.x), PxMax(y, lhs.y), PxMax(z, lhs.z));
			}

			PX_FORCE_INLINE Vec3 min(const Vec3& lhs) const
			{
				return Vec3(PxMin(x, lhs.x), PxMin(y, lhs.y), PxMin(z, lhs.z));
			}

			PX_FORCE_INLINE PxF64 magnitudeSquared() const
			{
				return x * x + y * y + z * z;
			}

			PX_FORCE_INLINE PxF64 magnitude() const
			{
				return PxSqrt(x * x + y * y + z * z);
			}

			PX_FORCE_INLINE Vec3 getNormalized() const
			{
				PxF64 s = magnitudeSquared();
				if (s == 0)
					return *this;
				s = 1.0 / PxSqrt(s);
				return Vec3(s * x, s * y, s * z);
			}

			PX_FORCE_INLINE PxF64 dot(const Vec3& rhs) const
			{
				return x * rhs.x + y * rhs.y + z * rhs.z;
			}

			PX_FORCE_INLINE Vec3 cross(const Vec3& v) const
			{
				return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
			}

			PX_FORCE_INLINE Vec3 operator+(const Vec3& v) const
			{
				return Vec3(x + v.x, y + v.y, z + v.z);
			}

			PX_FORCE_INLINE Vec3 operator-(const Vec3& v) const
			{
				return Vec3(x - v.x, y - v.y, z - v.z);
			}

			PX_FORCE_INLINE Vec3 operator*(const PxF64& v) const
			{
				return Vec3(x * v, y * v, z * v);
			}

			PX_FORCE_INLINE Vec3 operator/(const PxF64& v) const
			{
				PxF64 inv = 1.0 / v;
				return Vec3(x * inv, y * inv, z * inv);
			}

			PX_FORCE_INLINE Vec3& operator+=(const Vec3& v)
			{
				x += v.x;
				y += v.y;
				z += v.z;
				return *this;
			}

			PX_FORCE_INLINE Vec3& operator-=(const Vec3& v)
			{
				x -= v.x;
				y -= v.y;
				z -= v.z;
				return *this;
			}

			PX_FORCE_INLINE Vec3& operator*=(PxF64 f)
			{
				x *= f;
				y *= f;
				z *= f;
				return *this;
			}

			PX_FORCE_INLINE Vec3& operator/=(PxF64 f)
			{
				f = 1.0 / f;
				x *= f;
				y *= f;
				z *= f;
				return *this;
			}

			PX_FORCE_INLINE PxF64& operator[](PxU32 index)
			{
				PX_ASSERT(index <= 2);

				return reinterpret_cast<PxF64*>(this)[index];
			}

			PX_FORCE_INLINE const PxF64& operator[](PxU32 index) const
			{
				PX_ASSERT(index <= 2);

				return reinterpret_cast<const PxF64*>(this)[index];
			}
		};

		// ---------------------------------------------------------------------------------
		struct Bounds3 {
			Bounds3() {}
			Bounds3(const Vec3 &min, const Vec3 &max) : minimum(min), maximum(max) {}

			void setEmpty() {
				minimum = Vec3(PX_MAX_F64, PX_MAX_F64, PX_MAX_F64);
				maximum = Vec3(-PX_MAX_F64, -PX_MAX_F64, -PX_MAX_F64);
			}

			Vec3 getDimensions() const {
				return maximum - minimum;
			}

			void include(const Vec3 &p) {
				minimum = minimum.min(p);
				maximum = maximum.max(p);
			}

			void include(const Bounds3 &b) {
				minimum = minimum.min(b.minimum);
				maximum = maximum.max(b.maximum);
			}
			void expand(double d) {
				minimum -= Vec3(d, d, d);
				maximum += Vec3(d, d, d);
			}
			Vec3 minimum, maximum;
		};
	}
}

#endif

