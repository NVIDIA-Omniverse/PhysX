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
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

// Workaround to scope includes per shader
#ifdef NV_FLOW_CPU_SHADER
#undef NV_FLOW_SHADER_TYPES_H
#undef NV_FLOW_SHADER_HLSLI
#undef NV_FLOW_RAY_MARCH_PARAMS_H
#undef NV_FLOW_RAY_MARCH_HLSLI
#undef NV_FLOW_RAY_MARCH_COMMON_HLSLI
#endif

// Disabled by default, to save build time
#define NV_FLOW_CPU_SHADER_DISABLE

#ifndef NV_FLOW_RESOURCE_CPU_H
#define NV_FLOW_RESOURCE_CPU_H

#include "NvFlowContext.h"
#include <math.h>
#include <atomic>
#include <string.h>

typedef NvFlowUint NvFlowCPU_Uint;

struct NvFlowCPU_Float2;
struct NvFlowCPU_Float3;
struct NvFlowCPU_Float4;
struct NvFlowCPU_Float4x4;

struct NvFlowCPU_Int2;
struct NvFlowCPU_Int3;
struct NvFlowCPU_Int4;

struct NvFlowCPU_Uint2;
struct NvFlowCPU_Uint3;
struct NvFlowCPU_Uint4;

NV_FLOW_INLINE int NvFlowCPU_max(int a, int b)
{
	return a > b ? a : b;
}

NV_FLOW_INLINE int NvFlowCPU_min(int a, int b)
{
	return a < b ? a : b;
}

NV_FLOW_INLINE float NvFlowCPU_round(float v)
{
	return roundf(v);
}

NV_FLOW_INLINE float NvFlowCPU_abs(float v)
{
	return fabsf(v);
}

NV_FLOW_INLINE float NvFlowCPU_floor(float v)
{
	return floorf(v);
}

NV_FLOW_INLINE int NvFlowCPU_abs(int v)
{
	return v < 0 ? -v : v;
}

NV_FLOW_INLINE float NvFlowCPU_sqrt(float v)
{
	return sqrtf(v);
}

NV_FLOW_INLINE float NvFlowCPU_exp(float v)
{
	return expf(v);
}

NV_FLOW_INLINE float NvFlowCPU_pow(float a, float b)
{
	return powf(a, b);
}

NV_FLOW_INLINE float NvFlowCPU_log2(float v)
{
	return log2f(v);
}

NV_FLOW_INLINE float NvFlowCPU_min(float a, float b)
{
	//return fminf(a, b);
	return a < b ? a : b;
}

NV_FLOW_INLINE float NvFlowCPU_max(float a, float b)
{
	//return fmaxf(a, b);
	return a > b ? a : b;
}

NV_FLOW_INLINE float NvFlowCPU_clamp(float v, float min, float max)
{
	return NvFlowCPU_max(min, NvFlowCPU_min(v, max));
}

struct NvFlowCPU_Float2
{
	float x, y;

	NvFlowCPU_Float2() {}
	NvFlowCPU_Float2(float x, float y) : x(x), y(y) {}

	NV_FLOW_INLINE NvFlowCPU_Float2(const NvFlowCPU_Int2& rhs);

	NvFlowCPU_Float2 operator+(const NvFlowCPU_Float2& rhs) const { return NvFlowCPU_Float2(x + rhs.x, y + rhs.y); }
	NvFlowCPU_Float2 operator-(const NvFlowCPU_Float2& rhs) const { return NvFlowCPU_Float2(x - rhs.x, y - rhs.y); }
	NvFlowCPU_Float2 operator*(const NvFlowCPU_Float2& rhs) const { return NvFlowCPU_Float2(x * rhs.x, y * rhs.y); }
	NvFlowCPU_Float2 operator/(const NvFlowCPU_Float2& rhs) const { return NvFlowCPU_Float2(x / rhs.x, y / rhs.y); }

	NvFlowCPU_Float2 operator+(const float& rhs) const { return NvFlowCPU_Float2(x + rhs, y + rhs); }
	NvFlowCPU_Float2 operator-(const float& rhs) const { return NvFlowCPU_Float2(x - rhs, y - rhs); }
	NvFlowCPU_Float2 operator*(const float& rhs) const { return NvFlowCPU_Float2(x * rhs, y * rhs); }
	NvFlowCPU_Float2 operator/(const float& rhs) const { return NvFlowCPU_Float2(x / rhs, y / rhs); }

	NvFlowCPU_Float2& operator+=(const NvFlowCPU_Float2& rhs) { x += rhs.x; y += rhs.y; return *this; }
	NvFlowCPU_Float2& operator-=(const NvFlowCPU_Float2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
	NvFlowCPU_Float2& operator*=(const NvFlowCPU_Float2& rhs) { x *= rhs.x; y *= rhs.y; return *this; }
	NvFlowCPU_Float2& operator/=(const NvFlowCPU_Float2& rhs) { x /= rhs.x; y /= rhs.y; return *this; }

	NvFlowCPU_Float2& operator+=(const float& rhs) { x += rhs; y += rhs; return *this; }
	NvFlowCPU_Float2& operator-=(const float& rhs) { x -= rhs; y -= rhs; return *this; }
	NvFlowCPU_Float2& operator*=(const float& rhs) { x *= rhs; y *= rhs; return *this; }
	NvFlowCPU_Float2& operator/=(const float& rhs) { x /= rhs; y /= rhs; return *this; }

	NvFlowCPU_Float2 operator+() const { return NvFlowCPU_Float2(+x, +y); }
	NvFlowCPU_Float2 operator-() const { return NvFlowCPU_Float2(-x, -y); }
};

NV_FLOW_INLINE NvFlowCPU_Float2 operator+(const float& lhs, const NvFlowCPU_Float2& rhs) { return NvFlowCPU_Float2(lhs + rhs.x, lhs + rhs.y); }
NV_FLOW_INLINE NvFlowCPU_Float2 operator-(const float& lhs, const NvFlowCPU_Float2& rhs) { return NvFlowCPU_Float2(lhs - rhs.x, lhs - rhs.y); }
NV_FLOW_INLINE NvFlowCPU_Float2 operator*(const float& lhs, const NvFlowCPU_Float2& rhs) { return NvFlowCPU_Float2(lhs * rhs.x, lhs * rhs.y); }
NV_FLOW_INLINE NvFlowCPU_Float2 operator/(const float& lhs, const NvFlowCPU_Float2& rhs) { return NvFlowCPU_Float2(lhs / rhs.x, lhs / rhs.y); }

NV_FLOW_INLINE NvFlowCPU_Float2 NvFlowCPU_floor(NvFlowCPU_Float2 v)
{
	return NvFlowCPU_Float2(floorf(v.x), floorf(v.y));
}

struct NvFlowCPU_Float3
{
	float x, y, z;

	NvFlowCPU_Float3() {}
	NvFlowCPU_Float3(float x, float y, float z) : x(x), y(y), z(z) {}
	NV_FLOW_INLINE NvFlowCPU_Float3(const NvFlowCPU_Int3& v);

	NvFlowCPU_Float3 operator+(const NvFlowCPU_Float3& rhs) const { return NvFlowCPU_Float3(x + rhs.x, y + rhs.y, z + rhs.z); }
	NvFlowCPU_Float3 operator-(const NvFlowCPU_Float3& rhs) const { return NvFlowCPU_Float3(x - rhs.x, y - rhs.y, z - rhs.z); }
	NvFlowCPU_Float3 operator*(const NvFlowCPU_Float3& rhs) const { return NvFlowCPU_Float3(x * rhs.x, y * rhs.y, z * rhs.z); }
	NvFlowCPU_Float3 operator/(const NvFlowCPU_Float3& rhs) const { return NvFlowCPU_Float3(x / rhs.x, y / rhs.y, z / rhs.z); }

	NvFlowCPU_Float3 operator+(const float& rhs) const { return NvFlowCPU_Float3(x + rhs, y + rhs, z + rhs); }
	NvFlowCPU_Float3 operator-(const float& rhs) const { return NvFlowCPU_Float3(x - rhs, y - rhs, z - rhs); }
	NvFlowCPU_Float3 operator*(const float& rhs) const { return NvFlowCPU_Float3(x * rhs, y * rhs, z * rhs); }
	NvFlowCPU_Float3 operator/(const float& rhs) const { return NvFlowCPU_Float3(x / rhs, y / rhs, z / rhs); }

	NvFlowCPU_Float3& operator+=(const NvFlowCPU_Float3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
	NvFlowCPU_Float3& operator-=(const NvFlowCPU_Float3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
	NvFlowCPU_Float3& operator*=(const NvFlowCPU_Float3& rhs) { x *= rhs.x; y *= rhs.y; z *= rhs.z; return *this; }
	NvFlowCPU_Float3& operator/=(const NvFlowCPU_Float3& rhs) { x /= rhs.x; y /= rhs.y; z /= rhs.z; return *this; }

	NvFlowCPU_Float3& operator+=(const float& rhs) { x += rhs; y += rhs; z += rhs; return *this; }
	NvFlowCPU_Float3& operator-=(const float& rhs) { x -= rhs; y -= rhs; z -= rhs; return *this; }
	NvFlowCPU_Float3& operator*=(const float& rhs) { x *= rhs; y *= rhs; z *= rhs; return *this; }
	NvFlowCPU_Float3& operator/=(const float& rhs) { x /= rhs; y /= rhs; z /= rhs; return *this; }

	NvFlowCPU_Float3 operator+() const { return NvFlowCPU_Float3(+x, +y, +z); }
	NvFlowCPU_Float3 operator-() const { return NvFlowCPU_Float3(-x, -y, -z); }
};

NV_FLOW_INLINE NvFlowCPU_Float3 operator*(const float& lhs, const NvFlowCPU_Float3& rhs) { return NvFlowCPU_Float3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z); }

NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_abs(NvFlowCPU_Float3 v)
{
	return NvFlowCPU_Float3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_floor(NvFlowCPU_Float3 v)
{
	return NvFlowCPU_Float3(floorf(v.x), floorf(v.y), floorf(v.z));
}

NV_FLOW_INLINE float NvFlowCPU_length(NvFlowCPU_Float3 v)
{
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_max(NvFlowCPU_Float3 a, NvFlowCPU_Float3 b)
{
	return NvFlowCPU_Float3(NvFlowCPU_max(a.x, b.x), NvFlowCPU_max(a.y, b.y), NvFlowCPU_max(a.z, b.z));
}

NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_min(NvFlowCPU_Float3 a, NvFlowCPU_Float3 b)
{
	return NvFlowCPU_Float3(NvFlowCPU_min(a.x, b.x), NvFlowCPU_min(a.y, b.y), NvFlowCPU_min(a.z, b.z));
}

NV_FLOW_INLINE float NvFlowCPU_dot(NvFlowCPU_Float3 a, NvFlowCPU_Float3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_normalize(NvFlowCPU_Float3 v)
{
	float length = NvFlowCPU_length(v);
	if (length > 0.f)
	{
		v /= length;
	}
	return v;
}

struct NvFlowCPU_Float4
{
	float x, y, z, w;

	NvFlowCPU_Float4() {}
	NvFlowCPU_Float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	NvFlowCPU_Float4(const NvFlowCPU_Float3& rhs, float w) : x(rhs.x), y(rhs.y), z(rhs.z), w(w) {}

	NvFlowCPU_Float3& rgb() { return *((NvFlowCPU_Float3*)this); }
	NvFlowCPU_Float2& rg() { return *((NvFlowCPU_Float2*)this); }
	float& r() { return *((float*)this); }
	NvFlowCPU_Float2& ba() { return *((NvFlowCPU_Float2*)&z); }

	const NvFlowCPU_Float3& rgb() const { return *((const NvFlowCPU_Float3*)this); }
	const NvFlowCPU_Float2& rg() const { return *((const NvFlowCPU_Float2*)this); }
	const float& r() const { return *((const float*)this); }
	const NvFlowCPU_Float2& ba() const { return *((const NvFlowCPU_Float2*)&z); }

	NvFlowCPU_Float4 operator+(const NvFlowCPU_Float4& rhs) const { return NvFlowCPU_Float4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w); }
	NvFlowCPU_Float4 operator-(const NvFlowCPU_Float4& rhs) const { return NvFlowCPU_Float4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w); }
	NvFlowCPU_Float4 operator*(const NvFlowCPU_Float4& rhs) const { return NvFlowCPU_Float4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w); }
	NvFlowCPU_Float4 operator/(const NvFlowCPU_Float4& rhs) const { return NvFlowCPU_Float4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w); }

	NvFlowCPU_Float4 operator+(const float& rhs) const { return NvFlowCPU_Float4(x + rhs, y + rhs, z + rhs, w + rhs); }
	NvFlowCPU_Float4 operator-(const float& rhs) const { return NvFlowCPU_Float4(x - rhs, y - rhs, z - rhs, w - rhs); }
	NvFlowCPU_Float4 operator*(const float& rhs) const { return NvFlowCPU_Float4(x * rhs, y * rhs, z * rhs, w * rhs); }
	NvFlowCPU_Float4 operator/(const float& rhs) const { return NvFlowCPU_Float4(x / rhs, y / rhs, z / rhs, w / rhs); }

	NvFlowCPU_Float4& operator+=(const NvFlowCPU_Float4& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w; return *this; }
	NvFlowCPU_Float4& operator-=(const NvFlowCPU_Float4& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w; return *this; }
	NvFlowCPU_Float4& operator*=(const NvFlowCPU_Float4& rhs) { x *= rhs.x; y *= rhs.y; z *= rhs.z; w *= rhs.w; return *this; }
	NvFlowCPU_Float4& operator/=(const NvFlowCPU_Float4& rhs) { x /= rhs.x; y /= rhs.y; z /= rhs.z; w /= rhs.w; return *this; }

	NvFlowCPU_Float4& operator*=(const float& rhs) { x *= rhs; y *= rhs; z *= rhs; w *= rhs; return *this; }
};

NV_FLOW_INLINE NvFlowCPU_Float4 operator*(const float& lhs, const NvFlowCPU_Float4& rhs) { return NvFlowCPU_Float4(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w); }

NV_FLOW_INLINE float NvFlowCPU_dot(NvFlowCPU_Float4 a, NvFlowCPU_Float4 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_max(NvFlowCPU_Float4 a, NvFlowCPU_Float4 b)
{
	return NvFlowCPU_Float4(NvFlowCPU_max(a.x, b.x), NvFlowCPU_max(a.y, b.y), NvFlowCPU_max(a.z, b.z), NvFlowCPU_max(a.w, b.w));
}

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_min(NvFlowCPU_Float4 a, NvFlowCPU_Float4 b)
{
	return NvFlowCPU_Float4(NvFlowCPU_min(a.x, b.x), NvFlowCPU_min(a.y, b.y), NvFlowCPU_min(a.z, b.z), NvFlowCPU_min(a.w, b.w));
}

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_sign(NvFlowCPU_Float4 v)
{
	return NvFlowCPU_Float4(
		v.x == 0.f ? 0.f : (v.x < 0.f ? -1.f : +1.f),
		v.y == 0.f ? 0.f : (v.y < 0.f ? -1.f : +1.f),
		v.z == 0.f ? 0.f : (v.z < 0.f ? -1.f : +1.f),
		v.w == 0.f ? 0.f : (v.w < 0.f ? -1.f : +1.f)
	);
}

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_abs(NvFlowCPU_Float4 v)
{
	return NvFlowCPU_Float4(fabsf(v.x), fabsf(v.y), fabsf(v.z), fabsf(v.w));
}

struct NvFlowCPU_Float4x4
{
	NvFlowCPU_Float4 x, y, z, w;

	NvFlowCPU_Float4x4() {}
	NvFlowCPU_Float4x4(const NvFlowCPU_Float4& x, const NvFlowCPU_Float4& y, const NvFlowCPU_Float4& z, const NvFlowCPU_Float4& w) : x(x), y(y), z(z), w(w) {}
};

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_mul(const NvFlowCPU_Float4& x, const NvFlowCPU_Float4x4 A)
{
	return NvFlowCPU_Float4(
		{ A.x.x * x.x + A.x.y * x.y + A.x.z * x.z + A.x.w * x.w },
		{ A.y.x * x.x + A.y.y * x.y + A.y.z * x.z + A.y.w * x.w },
		{ A.z.x * x.x + A.z.y * x.y + A.z.z * x.z + A.z.w * x.w },
		{ A.w.x * x.x + A.w.y * x.y + A.w.z * x.z + A.w.w * x.w }
	);
}

struct NvFlowCPU_Int2
{
	int x, y;

	NvFlowCPU_Int2() {}
	NvFlowCPU_Int2(int x, int y) : x(x), y(y) {}
	NvFlowCPU_Int2(const NvFlowCPU_Float2& rhs) : x(int(rhs.x)), y(int(rhs.y)) {}
	NV_FLOW_INLINE NvFlowCPU_Int2(const NvFlowCPU_Uint2& rhs);

	NvFlowCPU_Int2 operator+(const NvFlowCPU_Int2& rhs) const { return NvFlowCPU_Int2(x + rhs.x, y + rhs.y); }
	NvFlowCPU_Int2 operator-(const NvFlowCPU_Int2& rhs) const { return NvFlowCPU_Int2(x - rhs.x, y - rhs.y); }
	NvFlowCPU_Int2 operator*(const NvFlowCPU_Int2& rhs) const { return NvFlowCPU_Int2(x * rhs.x, y * rhs.y); }
	NvFlowCPU_Int2 operator/(const NvFlowCPU_Int2& rhs) const { return NvFlowCPU_Int2(x / rhs.x, y / rhs.y); }

	NvFlowCPU_Int2 operator+(const int& rhs) const { return NvFlowCPU_Int2(x + rhs, y + rhs); }
	NvFlowCPU_Int2 operator-(const int& rhs) const { return NvFlowCPU_Int2(x - rhs, y - rhs); }
	NvFlowCPU_Int2 operator*(const int& rhs) const { return NvFlowCPU_Int2(x * rhs, y * rhs); }
	NvFlowCPU_Int2 operator/(const int& rhs) const { return NvFlowCPU_Int2(x / rhs, y / rhs); }
};

NV_FLOW_INLINE NvFlowCPU_Float2::NvFlowCPU_Float2(const NvFlowCPU_Int2& rhs) : x(float(rhs.x)), y(float(rhs.y)) {}

NV_FLOW_INLINE NvFlowCPU_Int2 NvFlowCPU_max(NvFlowCPU_Int2 a, NvFlowCPU_Int2 b)
{
	return NvFlowCPU_Int2(NvFlowCPU_max(a.x, b.x), NvFlowCPU_max(a.y, b.y));
}

NV_FLOW_INLINE NvFlowCPU_Int2 NvFlowCPU_min(NvFlowCPU_Int2 a, NvFlowCPU_Int2 b)
{
	return NvFlowCPU_Int2(NvFlowCPU_min(a.x, b.x), NvFlowCPU_min(a.y, b.y));
}

struct NvFlowCPU_Int3
{
	int x, y, z;

	NvFlowCPU_Int3() {}
	NvFlowCPU_Int3(int x, int y, int z) : x(x), y(y), z(z) {}
	NV_FLOW_INLINE NvFlowCPU_Int3(const NvFlowCPU_Uint3& v);
	NV_FLOW_INLINE NvFlowCPU_Int3(const NvFlowCPU_Float3& v);

	NvFlowCPU_Int2& rg() { return *((NvFlowCPU_Int2*)this); }
	int& r() { return *((int*)this); }

	NvFlowCPU_Int3 operator+(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x + rhs.x, y + rhs.y, z + rhs.z); }
	NvFlowCPU_Int3 operator-(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x - rhs.x, y - rhs.y, z - rhs.z); }
	NvFlowCPU_Int3 operator*(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x * rhs.x, y * rhs.y, z * rhs.z); }
	NvFlowCPU_Int3 operator/(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x / rhs.x, y / rhs.y, z / rhs.z); }

	NvFlowCPU_Int3& operator+=(const NvFlowCPU_Int3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
	NvFlowCPU_Int3& operator-=(const NvFlowCPU_Int3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
	NvFlowCPU_Int3& operator*=(const NvFlowCPU_Int3& rhs) { x *= rhs.x; y *= rhs.y; z *= rhs.z; return *this; }
	NvFlowCPU_Int3& operator/=(const NvFlowCPU_Int3& rhs) { x /= rhs.x; y /= rhs.y; z /= rhs.z; return *this; }

	NV_FLOW_INLINE NvFlowCPU_Int3 operator>>(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x >> rhs.x, y >> rhs.y, z >> rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Int3 operator<<(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x << rhs.x, y << rhs.y, z << rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Int3 operator&(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x & rhs.x, y & rhs.y, z & rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Int3 operator|(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Int3(x | rhs.x, y | rhs.y, z | rhs.z); }

	NV_FLOW_INLINE NvFlowCPU_Int3 operator>>(const int& rhs) const { return NvFlowCPU_Int3(x >> rhs, y >> rhs, z >> rhs); }
	NV_FLOW_INLINE NvFlowCPU_Int3 operator<<(const int& rhs) const { return NvFlowCPU_Int3(x << rhs, y << rhs, z << rhs); }
	NV_FLOW_INLINE NvFlowCPU_Int3 operator>>(const NvFlowCPU_Uint& rhs) const { return NvFlowCPU_Int3(x >> rhs, y >> rhs, z >> rhs); }
	NV_FLOW_INLINE NvFlowCPU_Int3 operator<<(const NvFlowCPU_Uint& rhs) const { return NvFlowCPU_Int3(x << rhs, y << rhs, z << rhs); }

	NV_FLOW_INLINE NvFlowCPU_Int3 operator>>(const NvFlowCPU_Uint3& rhs) const;
	NV_FLOW_INLINE NvFlowCPU_Int3 operator<<(const NvFlowCPU_Uint3& rhs) const;
};

NV_FLOW_INLINE NvFlowCPU_Int3 NvFlowCPU_max(NvFlowCPU_Int3 a, NvFlowCPU_Int3 b)
{
	return NvFlowCPU_Int3(NvFlowCPU_max(a.x, b.x), NvFlowCPU_max(a.y, b.y), NvFlowCPU_max(a.z, b.z));
}

NV_FLOW_INLINE NvFlowCPU_Int3 NvFlowCPU_min(NvFlowCPU_Int3 a, NvFlowCPU_Int3 b)
{
	return NvFlowCPU_Int3(NvFlowCPU_min(a.x, b.x), NvFlowCPU_min(a.y, b.y), NvFlowCPU_min(a.z, b.z));
}

struct NvFlowCPU_Int4
{
	int x, y, z, w;

	NvFlowCPU_Int4() {}
	NvFlowCPU_Int4(int x, int y, int z, int w) : x(x), y(y), z(z), w(w) {}
	NvFlowCPU_Int4(const NvFlowCPU_Int2& a, const NvFlowCPU_Int2& b) : x(a.x), y(a.y), z(b.x), w(b.y) {}
	NvFlowCPU_Int4(const NvFlowCPU_Int3& rhs, int w) : x(rhs.x), y(rhs.y), z(rhs.z), w(w) {}
	NvFlowCPU_Int4(const NvFlowCPU_Uint4& rhs);

	NvFlowCPU_Int3& rgb() { return *((NvFlowCPU_Int3*)this); }
	NvFlowCPU_Int2& rg() { return *((NvFlowCPU_Int2*)this); }
	int& r() { return *((int*)this); }
	NvFlowCPU_Int2& ba() { return *((NvFlowCPU_Int2*)&z); }

	const NvFlowCPU_Int3& rgb()const { return *((const NvFlowCPU_Int3*)this); }
	const NvFlowCPU_Int2& rg()const { return *((const NvFlowCPU_Int2*)this); }
	const int& r()const { return *((const int*)this); }
	const NvFlowCPU_Int2& ba()const { return *((const NvFlowCPU_Int2*)&z); }

	NvFlowCPU_Int4 operator+(const NvFlowCPU_Int4& rhs) const { return NvFlowCPU_Int4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w); }
};

struct NvFlowCPU_Uint2
{
	NvFlowUint x, y;

	NvFlowCPU_Uint2() {}
	NvFlowCPU_Uint2(NvFlowUint x, NvFlowUint y) : x(x), y(y) {}
	NvFlowCPU_Uint2(const NvFlowCPU_Int2& rhs) : x(rhs.x), y(rhs.y) {}
};

NV_FLOW_INLINE NvFlowCPU_Int2::NvFlowCPU_Int2(const NvFlowCPU_Uint2& rhs) : x(rhs.x), y(rhs.y) {}

struct NvFlowCPU_Uint3
{
	NvFlowUint x, y, z;

	NvFlowCPU_Uint3() {}
	NvFlowCPU_Uint3(NvFlowUint x, NvFlowUint y, NvFlowUint z) : x(x), y(y), z(z) {}
	NV_FLOW_INLINE NvFlowCPU_Uint3(const NvFlowCPU_Int3& v);

	NvFlowCPU_Uint2& rg() { return *((NvFlowCPU_Uint2*)this); }
	NvFlowCPU_Uint& r() { return *((NvFlowCPU_Uint*)this); }

	NvFlowCPU_Uint3 operator+(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x + rhs.x, y + rhs.y, z + rhs.z); }
	NvFlowCPU_Uint3 operator-(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x - rhs.x, y - rhs.y, z - rhs.z); }
	NvFlowCPU_Uint3 operator*(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x * rhs.x, y * rhs.y, z * rhs.z); }
	NvFlowCPU_Uint3 operator/(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x / rhs.x, y / rhs.y, z / rhs.z); }

	NV_FLOW_INLINE NvFlowCPU_Uint3 operator&(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x & rhs.x, y & rhs.y, z & rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Uint3 operator|(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x | rhs.x, y | rhs.y, z | rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Uint3 operator>>(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x >> rhs.x, y >> rhs.y, z >> rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Uint3 operator<<(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Uint3(x << rhs.x, y << rhs.y, z << rhs.z); }

	NV_FLOW_INLINE NvFlowCPU_Uint3 operator>>(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Uint3(x >> rhs.x, y >> rhs.y, z >> rhs.z); }
	NV_FLOW_INLINE NvFlowCPU_Uint3 operator<<(const NvFlowCPU_Int3& rhs) const { return NvFlowCPU_Uint3(x << rhs.x, y << rhs.y, z << rhs.z); }
};

NV_FLOW_INLINE NvFlowCPU_Uint3 operator>>(const NvFlowCPU_Uint& lhs, const NvFlowCPU_Uint3& rhs) { return NvFlowCPU_Uint3(lhs >> rhs.x, lhs >> rhs.y, lhs >> rhs.z); }
NV_FLOW_INLINE NvFlowCPU_Uint3 operator>>(const NvFlowCPU_Uint3& lhs, const NvFlowCPU_Uint& rhs) { return NvFlowCPU_Uint3(lhs.x >> rhs, lhs.y >> rhs, lhs.z >> rhs); }
NV_FLOW_INLINE NvFlowCPU_Uint3 operator<<(const NvFlowCPU_Uint& lhs, const NvFlowCPU_Uint3& rhs) { return NvFlowCPU_Uint3(lhs << rhs.x, lhs << rhs.y, lhs << rhs.z); }
NV_FLOW_INLINE NvFlowCPU_Uint3 operator<<(const NvFlowCPU_Uint3& lhs, const NvFlowCPU_Uint& rhs) { return NvFlowCPU_Uint3(lhs.x << rhs, lhs.y << rhs, lhs.z << rhs); }

NV_FLOW_INLINE NvFlowCPU_Uint3 operator+(const NvFlowCPU_Uint& lhs, const NvFlowCPU_Uint3& rhs) { return NvFlowCPU_Uint3(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z); }
NV_FLOW_INLINE NvFlowCPU_Uint3 operator+(const NvFlowCPU_Uint3& lhs, const NvFlowCPU_Uint& rhs) { return NvFlowCPU_Uint3(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs); }
NV_FLOW_INLINE NvFlowCPU_Uint3 operator-(const NvFlowCPU_Uint& lhs, const NvFlowCPU_Uint3& rhs) { return NvFlowCPU_Uint3(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z); }
NV_FLOW_INLINE NvFlowCPU_Uint3 operator-(const NvFlowCPU_Uint3& lhs, const NvFlowCPU_Uint& rhs) { return NvFlowCPU_Uint3(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs); }

struct NvFlowCPU_Uint4
{
	NvFlowUint x, y, z, w;

	NvFlowCPU_Uint4() {}
	NvFlowCPU_Uint4(NvFlowUint x, NvFlowUint y, NvFlowUint z, NvFlowUint w) : x(x), y(y), z(z), w(w) {}

	NvFlowCPU_Uint4 operator+(const NvFlowCPU_Uint4& rhs) const { return NvFlowCPU_Uint4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w); }
	NvFlowCPU_Uint4 operator-(const NvFlowCPU_Uint4& rhs) const { return NvFlowCPU_Uint4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w); }
	NvFlowCPU_Uint4 operator*(const NvFlowCPU_Uint4& rhs) const { return NvFlowCPU_Uint4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w); }
	NvFlowCPU_Uint4 operator/(const NvFlowCPU_Uint4& rhs) const { return NvFlowCPU_Uint4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w); }

	NvFlowCPU_Uint4& operator+=(const NvFlowCPU_Uint4& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w; return *this; }
	NvFlowCPU_Uint4& operator-=(const NvFlowCPU_Uint4& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w; return *this; }
	NvFlowCPU_Uint4& operator*=(const NvFlowCPU_Uint4& rhs) { x *= rhs.x; y *= rhs.y; z *= rhs.z; w *= rhs.w; return *this; }
	NvFlowCPU_Uint4& operator/=(const NvFlowCPU_Uint4& rhs) { x /= rhs.x; y /= rhs.y; z /= rhs.z; w /= rhs.w; return *this; }
};

NV_FLOW_INLINE NvFlowCPU_Int3 NvFlowCPU_Int3::operator>>(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Int3(x >> rhs.x, y >> rhs.y, z >> rhs.z); }
NV_FLOW_INLINE NvFlowCPU_Int3 NvFlowCPU_Int3::operator<<(const NvFlowCPU_Uint3& rhs) const { return NvFlowCPU_Int3(x << rhs.x, y << rhs.y, z << rhs.z); }

NV_FLOW_INLINE NvFlowCPU_Float3::NvFlowCPU_Float3(const NvFlowCPU_Int3& v) : x(float(v.x)), y(float(v.y)), z(float(v.z)) {}

NV_FLOW_INLINE NvFlowCPU_Int3::NvFlowCPU_Int3(const NvFlowCPU_Uint3& v) : x(int(v.x)), y(int(v.y)), z(int(v.z)) {}
NV_FLOW_INLINE NvFlowCPU_Int3::NvFlowCPU_Int3(const NvFlowCPU_Float3& v) : x(int(v.x)), y(int(v.y)), z(int(v.z)) {}

NV_FLOW_INLINE NvFlowCPU_Uint3::NvFlowCPU_Uint3(const NvFlowCPU_Int3& v) : x(int(v.x)), y(int(v.y)), z(int(v.z)) {}

NV_FLOW_INLINE NvFlowCPU_Int4::NvFlowCPU_Int4(const NvFlowCPU_Uint4& rhs) : x(int(rhs.x)), y(int(rhs.y)), z(int(rhs.z)), w(int(rhs.w)) {}

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_asfloat(NvFlowCPU_Uint4 v) {return *((NvFlowCPU_Float4*)&v);}
NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_asfloat(NvFlowCPU_Uint3 v) {return *((NvFlowCPU_Float3*)&v);}
NV_FLOW_INLINE NvFlowCPU_Float2 NvFlowCPU_asfloat(NvFlowCPU_Uint2 v) {return *((NvFlowCPU_Float2*)&v);}
NV_FLOW_INLINE float NvFlowCPU_asfloat(NvFlowUint v) {return *((float*)&v);}

NV_FLOW_INLINE NvFlowCPU_Float4 NvFlowCPU_asfloat(NvFlowCPU_Int4 v) {return *((NvFlowCPU_Float4*)&v);}
NV_FLOW_INLINE NvFlowCPU_Float3 NvFlowCPU_asfloat(NvFlowCPU_Int3 v) {return *((NvFlowCPU_Float3*)&v);}
NV_FLOW_INLINE NvFlowCPU_Float2 NvFlowCPU_asfloat(NvFlowCPU_Int2 v) {return *((NvFlowCPU_Float2*)&v);}
NV_FLOW_INLINE float NvFlowCPU_asfloat(int v) {return *((float*)&v);}

NV_FLOW_INLINE NvFlowCPU_Uint4 NvFlowCPU_asuint(NvFlowCPU_Float4 v) {return *((NvFlowCPU_Uint4*)&v);}
NV_FLOW_INLINE NvFlowCPU_Uint3 NvFlowCPU_asuint(NvFlowCPU_Float3 v) {return *((NvFlowCPU_Uint3*)&v);}
NV_FLOW_INLINE NvFlowCPU_Uint2 NvFlowCPU_asuint(NvFlowCPU_Float2 v) {return *((NvFlowCPU_Uint2*)&v);}
NV_FLOW_INLINE NvFlowUint NvFlowCPU_asuint(float v) {return *((NvFlowUint*)&v);}

NV_FLOW_INLINE NvFlowCPU_Int4 NvFlowCPU_asint(NvFlowCPU_Float4 v) {return *((NvFlowCPU_Int4*)&v);}
NV_FLOW_INLINE NvFlowCPU_Int3 NvFlowCPU_asint(NvFlowCPU_Float3 v) {return *((NvFlowCPU_Int3*)&v);}
NV_FLOW_INLINE NvFlowCPU_Int2 NvFlowCPU_asint(NvFlowCPU_Float2 v) {return *((NvFlowCPU_Int2*)&v);}
NV_FLOW_INLINE int NvFlowCPU_asint(float v) {return *((int*)&v);}

struct NvFlowCPU_Resource
{
	void* data;
	NvFlowUint64 sizeInBytes;
	NvFlowUint elementSizeInBytes;
	NvFlowUint elementCount;
	NvFlowFormat format;
	NvFlowUint width;
	NvFlowUint height;
	NvFlowUint depth;
	NvFlowSamplerDesc samplerDesc;
};

template <typename T>
struct NvFlowCPU_ConstantBuffer
{
	const T* data;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (const T*)resource->data;
	}
};

template <typename T>
struct NvFlowCPU_StructuredBuffer
{
	const T* data;
	NvFlowUint count;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (const T*)resource->data;
		count = resource->elementCount;
	}

	const T& operator[](int index) {
		if (index < 0 || index >= int(count)) index = 0;
		return data[index];
	}
};

template <typename T>
struct NvFlowCPU_RWStructuredBuffer
{
	T* data;
	NvFlowUint count;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (T*)resource->data;
		count = resource->elementCount;
	}

	T& operator[](int index) {
		if (index < 0 || index >= int(count)) index = 0;
		return data[index];
	}
};

struct NvFlowCPU_SamplerState
{
	NvFlowSamplerDesc desc;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		desc = resource->samplerDesc;
	}
};

template <typename T>
struct NvFlowCPU_Texture1D
{
	const T* data;
	NvFlowFormat format;
	NvFlowUint width;
	T out_of_bounds = {};

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (const T*)resource->data;
		format = resource->format;
		width = resource->width;
		memset(&out_of_bounds, 0, sizeof(out_of_bounds));
	}
};

template <typename T>
NV_FLOW_FORCE_INLINE const T NvFlowCPU_textureRead(NvFlowCPU_Texture1D<T>& tex, int index)
{
	if (index < 0 || index >= int(tex.width))
	{
		return tex.out_of_bounds;
	}
	return tex.data[index];
}

template <typename T>
NV_FLOW_FORCE_INLINE T NvFlowCPU_textureSampleLevel(NvFlowCPU_Texture1D<T>& tex, NvFlowCPU_SamplerState state, const float pos, float lod)
{
	float posf(float(tex.width) * pos);

	// clamp sampler
	if (posf < 0.5f) posf = 0.5f;
	if (posf > float(tex.width) - 0.5f) posf = float(tex.width) - 0.5f;

	int pos0 = int(NvFlowCPU_floor(posf - 0.5f));
	float f = posf - 0.5f - float(pos0);
	float of = 1.f - f;

	T sum = of * NvFlowCPU_textureRead(tex, pos0 + 0);
	sum += f * NvFlowCPU_textureRead(tex, pos0 + 1);

	return sum;
}

template <typename T>
struct NvFlowCPU_RWTexture1D
{
	T* data;
	NvFlowFormat format;
	NvFlowUint width;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (T*)resource->data;
		format = resource->format;
		width = resource->width;
	}
};

template <typename T>
NV_FLOW_FORCE_INLINE const T NvFlowCPU_textureRead(NvFlowCPU_RWTexture1D<T>& tex, int index)
{
	return tex.data[index];
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_textureWrite(NvFlowCPU_RWTexture1D<T>& tex, int index, const T value)
{
	tex.data[index] = value;
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_textureWrite(bool pred, NvFlowCPU_RWTexture1D<T>& tex, int index, const T value)
{
	if (pred)
	{
		NvFlowCPU_textureWrite(tex, index, value);
	}
}

template <typename T>
struct NvFlowCPU_Texture2D
{
	const T* data;
	NvFlowFormat format;
	NvFlowUint width;
	NvFlowUint height;
	T out_of_bounds;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (const T*)resource->data;
		format = resource->format;
		width = resource->width;
		height = resource->height;
		memset(&out_of_bounds, 0, sizeof(out_of_bounds));
	}
};

template <typename T>
NV_FLOW_FORCE_INLINE const T NvFlowCPU_textureRead(NvFlowCPU_Texture2D<T>& tex, NvFlowCPU_Int2 index)
{
	if (index.x < 0 || index.x >= int(tex.width) ||
		index.y < 0 || index.y >= int(tex.height))
	{
		return tex.out_of_bounds;
	}
	return tex.data[index.y * tex.width + index.x];
}

template <typename T>
NV_FLOW_FORCE_INLINE T NvFlowCPU_textureSampleLevel(NvFlowCPU_Texture2D<T>& tex, NvFlowCPU_SamplerState state, const NvFlowCPU_Float2 pos, float lod)
{
	NvFlowCPU_Float2 posf(NvFlowCPU_Float2(float(tex.width), float(tex.height)) * pos);

	// clamp sampler
	if (posf.x < 0.5f) posf.x = 0.5f;
	if (posf.x > float(tex.width) - 0.5f) posf.x = float(tex.width) - 0.5f;
	if (posf.y < 0.5f) posf.y = 0.5f;
	if (posf.y > float(tex.height) - 0.5f) posf.y = float(tex.height) - 0.5f;

	NvFlowCPU_Int2 pos00 = NvFlowCPU_Int2(NvFlowCPU_floor(posf - NvFlowCPU_Float2(0.5f, 0.5f)));
	NvFlowCPU_Float2 f = posf - NvFlowCPU_Float2(0.5f, 0.5f) - NvFlowCPU_Float2(pos00);
	NvFlowCPU_Float2 of = NvFlowCPU_Float2(1.f, 1.f) - f;

	T sum = of.x * of.y * NvFlowCPU_textureRead(tex, pos00 + NvFlowCPU_Int2(0, 0));
	sum += f.x * of.y * NvFlowCPU_textureRead(tex, pos00 + NvFlowCPU_Int2(1, 0));
	sum += of.x * f.y * NvFlowCPU_textureRead(tex, pos00 + NvFlowCPU_Int2(0, 1));
	sum += f.x * f.y * NvFlowCPU_textureRead(tex, pos00 + NvFlowCPU_Int2(1, 1));

	return sum;
}

template <typename T>
struct NvFlowCPU_RWTexture2D
{
	T* data;
	NvFlowFormat format;
	NvFlowUint width;
	NvFlowUint height;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (T*)resource->data;
		format = resource->format;
		width = resource->width;
		height = resource->height;
	}
};

template <typename T>
NV_FLOW_FORCE_INLINE const T NvFlowCPU_textureRead(NvFlowCPU_RWTexture2D<T>& tex, NvFlowCPU_Int2 index)
{
	return tex.data[index.y * tex.width + index.x];
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_textureWrite(NvFlowCPU_RWTexture2D<T>& tex, NvFlowCPU_Int2 index, const T value)
{
	tex.data[index.y * tex.width + index.x] = value;
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_textureWrite(bool pred, NvFlowCPU_RWTexture2D<T>& tex, NvFlowCPU_Int2 index, const T value)
{
	if (pred)
	{
		NvFlowCPU_textureWrite(tex, index, value);
	}
}

template <typename T>
struct NvFlowCPU_Texture3D
{
	const T* data;
	NvFlowFormat format;
	NvFlowUint width;
	NvFlowUint height;
	NvFlowUint depth;
	T out_of_bounds;

	NvFlowUint wh;
	NvFlowUint whd;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (const T*)resource->data;
		format = resource->format;
		width = resource->width;
		height = resource->height;
		depth = resource->depth;
		memset(&out_of_bounds, 0, sizeof(out_of_bounds));

		wh = width * height;
		whd = wh * depth;
	}
};

template <typename T>
NV_FLOW_FORCE_INLINE const T NvFlowCPU_textureRead(NvFlowCPU_Texture3D<T>& tex, NvFlowCPU_Int3 index)
{
	if (index.x < 0 || index.x >= int(tex.width) ||
		index.y < 0 || index.y >= int(tex.height) ||
		index.z < 0 || index.z >= int(tex.depth))
	{
		return tex.out_of_bounds;
	}
	return tex.data[(index.z * tex.height + index.y) * tex.width + index.x];
}

template <typename T>
NV_FLOW_FORCE_INLINE T NvFlowCPU_textureSampleLevel(NvFlowCPU_Texture3D<T>& tex, NvFlowCPU_SamplerState state, const NvFlowCPU_Float3 pos, float lod)
{
	NvFlowCPU_Float3 posf(NvFlowCPU_Float3(float(tex.width), float(tex.height), float(tex.depth)) * pos);

	// clamp sampler
	if (posf.x < 0.5f) posf.x = 0.5f;
	if (posf.x > float(tex.width) - 0.5f) posf.x = float(tex.width) - 0.5f;
	if (posf.y < 0.5f) posf.y = 0.5f;
	if (posf.y > float(tex.height) - 0.5f) posf.y = float(tex.height) - 0.5f;
	if (posf.z < 0.5f) posf.z = 0.5f;
	if (posf.z > float(tex.depth) - 0.5f) posf.z = float(tex.depth) - 0.5f;

	NvFlowCPU_Int4 pos000 = NvFlowCPU_Int4(NvFlowCPU_floor(posf - NvFlowCPU_Float3(0.5f, 0.5f, 0.5f)), 0);
	NvFlowCPU_Float3 f = posf - NvFlowCPU_Float3(0.5f, 0.5f, 0.5f) - NvFlowCPU_Float3(float(pos000.x), float(pos000.y), float(pos000.z));
	NvFlowCPU_Float3 of = NvFlowCPU_Float3(1.f, 1.f, 1.f) - f;

	NvFlowCPU_Float4 wl(
		of.x * of.y * of.z,
		f.x * of.y * of.z,
		of.x * f.y * of.z,
		f.x * f.y * of.z
	);
	NvFlowCPU_Float4 wh(
		of.x * of.y * f.z,
		f.x * of.y * f.z,
		of.x * f.y * f.z,
		f.x * f.y * f.z
	);

	T sum;
	if (pos000.x >= 0 && pos000.y >= 0 && pos000.z >= 0 &&
		pos000.x <= int(tex.width - 2) && pos000.y <= int(tex.height - 2) && pos000.z <= int(tex.depth - 2))
	{
		NvFlowUint idx000 = pos000.z * tex.wh + pos000.y * tex.width + pos000.x;
		sum = wl.x * tex.data[idx000];
		sum += wl.y * tex.data[idx000 + 1u];
		sum += wl.z * tex.data[idx000 + tex.width];
		sum += wl.w * tex.data[idx000 + 1u + tex.width];
		sum += wh.x * tex.data[idx000 + tex.wh];
		sum += wh.y * tex.data[idx000 + 1u + tex.wh];
		sum += wh.z * tex.data[idx000 + tex.width + tex.wh];
		sum += wh.w * tex.data[idx000 + 1u + tex.width + tex.wh];
	}
	else
	{
		sum = wl.x * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(0, 0, 0));
		sum += wl.y * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(1, 0, 0));
		sum += wl.z * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(0, 1, 0));
		sum += wl.w * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(1, 1, 0));
		sum += wh.x * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(0, 0, 1));
		sum += wh.y * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(1, 0, 1));
		sum += wh.z * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(0, 1, 1));
		sum += wh.w * NvFlowCPU_textureRead(tex, pos000.rgb() + NvFlowCPU_Int3(1, 1, 1));
	}
	return sum;
}

template <typename T>
struct NvFlowCPU_RWTexture3D
{
	T* data;
	NvFlowFormat format;
	NvFlowUint width;
	NvFlowUint height;
	NvFlowUint depth;

	NV_FLOW_INLINE void bind(NvFlowCPU_Resource* resource)
	{
		data = (T*)resource->data;
		format = resource->format;
		width = resource->width;
		height = resource->height;
		depth = resource->depth;
	}
};

template <typename T>
NV_FLOW_FORCE_INLINE const T NvFlowCPU_textureRead(NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index)
{
	return tex.data[(index.z * tex.height + index.y) * tex.width + index.x];
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_textureWrite(NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index, const T value)
{
	tex.data[(index.z * tex.height + index.y) * tex.width + index.x] = value;
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_textureWrite(bool pred, NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index, const T value)
{
	if (pred)
	{
		NvFlowCPU_textureWrite(tex, index, value);
	}
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_InterlockedAdd(NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index, T value)
{
	((std::atomic<T>*)&tex.data[(index.z * tex.height + index.y) * tex.width + index.x])->fetch_add(value);
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_InterlockedMin(NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index, T value)
{
	((std::atomic<T>*)&tex.data[(index.z * tex.height + index.y) * tex.width + index.x])->fetch_min(value);
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_InterlockedOr(NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index, T value)
{
	((std::atomic<T>*)&tex.data[(index.z * tex.height + index.y) * tex.width + index.x])->fetch_or(value);
}

template <typename T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_InterlockedAnd(NvFlowCPU_RWTexture3D<T>& tex, NvFlowCPU_Int3 index, T value)
{
	((std::atomic<T>*)&tex.data[(index.z * tex.height + index.y) * tex.width + index.x])->fetch_and(value);
}

template <class T>
struct NvFlowCPU_Groupshared
{
	T data;
};

template <class T>
NV_FLOW_FORCE_INLINE void NvFlowCPU_swrite(int _groupshared_pass, int _groupshared_sync_count, NvFlowCPU_Groupshared<T>& g, const T& value)
{
	if (_groupshared_pass == _groupshared_sync_count)
	{
		g.data = value;
	}
}

template <class T>
NV_FLOW_FORCE_INLINE T NvFlowCPU_sread(NvFlowCPU_Groupshared<T>& g)
{
	return g.data;
}

template <class T, unsigned int arraySize>
struct NvFlowCPU_GroupsharedArray
{
	T data[arraySize];
};

template <class T, unsigned int arraySize>
NV_FLOW_FORCE_INLINE void NvFlowCPU_swrite(int _groupshared_pass, int _groupshared_sync_count, NvFlowCPU_GroupsharedArray<T, arraySize>& g, int index, const T& value)
{
	if (_groupshared_pass == _groupshared_sync_count)
	{
		g.data[index] = value;
	}
}

template <class T, unsigned int arraySize>
NV_FLOW_FORCE_INLINE T NvFlowCPU_sread(NvFlowCPU_GroupsharedArray<T, arraySize>& g, int index)
{
	return g.data[index];
}

#endif