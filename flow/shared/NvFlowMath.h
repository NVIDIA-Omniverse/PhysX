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

#pragma once

#include "NvFlowTypes.h"

#include <math.h>

namespace NvFlowMath
{
	static const float pi = 3.141592654f;

	NV_FLOW_INLINE NvFlowFloat4 operator+(const NvFlowFloat4& lhs, const NvFlowFloat4& rhs)
	{
		NvFlowFloat4 ret;
		ret.x = lhs.x + rhs.x;
		ret.y = lhs.y + rhs.y;
		ret.z = lhs.z + rhs.z;
		ret.w = lhs.w + rhs.w;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 operator-(const NvFlowFloat4& lhs, const NvFlowFloat4& rhs)
	{
		NvFlowFloat4 ret;
		ret.x = lhs.x - rhs.x;
		ret.y = lhs.y - rhs.y;
		ret.z = lhs.z - rhs.z;
		ret.w = lhs.w - rhs.w;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 operator*(const NvFlowFloat4& lhs, const NvFlowFloat4& rhs)
	{
		NvFlowFloat4 ret;
		ret.x = lhs.x * rhs.x;
		ret.y = lhs.y * rhs.y;
		ret.z = lhs.z * rhs.z;
		ret.w = lhs.w * rhs.w;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 operator/(const NvFlowFloat4& lhs, const NvFlowFloat4& rhs)
	{
		NvFlowFloat4 ret;
		ret.x = lhs.x / rhs.x;
		ret.y = lhs.y / rhs.y;
		ret.z = lhs.z / rhs.z;
		ret.w = lhs.w / rhs.w;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 operator*(float v, const NvFlowFloat4& rhs)
	{
		NvFlowFloat4 ret;
		ret.x = v * rhs.x;
		ret.y = v * rhs.y;
		ret.z = v * rhs.z;
		ret.w = v * rhs.w;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 operator*(const NvFlowFloat4& lhs, float v)
	{
		NvFlowFloat4 ret;
		ret.x = lhs.x * v;
		ret.y = lhs.y * v;
		ret.z = lhs.z * v;
		ret.w = lhs.w * v;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorSplatX(const NvFlowFloat4& a)
	{
		return NvFlowFloat4{ a.x, a.x, a.x, a.x };
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorSplatY(const NvFlowFloat4& a)
	{
		return NvFlowFloat4{ a.y, a.y, a.y, a.y };
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorSplatZ(const NvFlowFloat4& a)
	{
		return NvFlowFloat4{ a.z, a.z, a.z, a.z };
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorSplatW(const NvFlowFloat4& a)
	{
		return NvFlowFloat4{ a.w, a.w, a.w, a.w };
	}

	NV_FLOW_INLINE NvFlowFloat4 vector3Normalize(const NvFlowFloat4& v)
	{
		float magn = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

		if (magn > 0.f)
		{
			magn = 1.f / magn;
		}

		return NvFlowFloat4{ v.x * magn, v.y * magn, v.z * magn, v.w * magn };
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorPerspectiveDivide(const NvFlowFloat4& v)
	{
		return v / vectorSplatW(v);
	}

	NV_FLOW_INLINE NvFlowFloat4 matrixMultiplyRow(const NvFlowFloat4x4& b, const NvFlowFloat4& r)
	{
		NvFlowFloat4 result;
		result.x = b.x.x * r.x + b.y.x * r.y + b.z.x * r.z + b.w.x * r.w;
		result.y = b.x.y * r.x + b.y.y * r.y + b.z.y * r.z + b.w.y * r.w;
		result.z = b.x.z * r.x + b.y.z * r.y + b.z.z * r.z + b.w.z * r.w;
		result.w = b.x.w * r.x + b.y.w * r.y + b.z.w * r.z + b.w.w * r.w;
		return result;
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixMultiply(const NvFlowFloat4x4& a, const NvFlowFloat4x4& b)
	{
		NvFlowFloat4x4 result;
		result.x = matrixMultiplyRow(b, a.x);
		result.y = matrixMultiplyRow(b, a.y);
		result.z = matrixMultiplyRow(b, a.z);
		result.w = matrixMultiplyRow(b, a.w);
		return result;
	}

	NV_FLOW_INLINE NvFlowFloat4 matrixTransposeRow(const NvFlowFloat4x4& a, unsigned int offset)
	{
		NvFlowFloat4 result;
		result.x = *((&a.x.x) + offset);
		result.y = *((&a.y.x) + offset);
		result.z = *((&a.z.x) + offset);
		result.w = *((&a.w.x) + offset);
		return result;
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixTranspose(const NvFlowFloat4x4& a)
	{
		NvFlowFloat4x4 result;
		result.x = matrixTransposeRow(a, 0u);
		result.y = matrixTransposeRow(a, 1u);
		result.z = matrixTransposeRow(a, 2u);
		result.w = matrixTransposeRow(a, 3u);
		return result;
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixInverse(const NvFlowFloat4x4& a)
	{
		const NvFlowFloat4x4& m = a;

		float f = (float(1.0) /
			(m.x.x * m.y.y * m.z.z * m.w.w +
				m.x.x * m.y.z * m.z.w * m.w.y +
				m.x.x * m.y.w * m.z.y * m.w.z +
				m.x.y * m.y.x * m.z.w * m.w.z +
				m.x.y * m.y.z * m.z.x * m.w.w +
				m.x.y * m.y.w * m.z.z * m.w.x +
				m.x.z * m.y.x * m.z.y * m.w.w +
				m.x.z * m.y.y * m.z.w * m.w.x +
				m.x.z * m.y.w * m.z.x * m.w.y +
				m.x.w * m.y.x * m.z.z * m.w.y +
				m.x.w * m.y.y * m.z.x * m.w.z +
				m.x.w * m.y.z * m.z.y * m.w.x +
				-m.x.x * m.y.y * m.z.w * m.w.z +
				-m.x.x * m.y.z * m.z.y * m.w.w +
				-m.x.x * m.y.w * m.z.z * m.w.y +
				-m.x.y * m.y.x * m.z.z * m.w.w +
				-m.x.y * m.y.z * m.z.w * m.w.x +
				-m.x.y * m.y.w * m.z.x * m.w.z +
				-m.x.z * m.y.x * m.z.w * m.w.y +
				-m.x.z * m.y.y * m.z.x * m.w.w +
				-m.x.z * m.y.w * m.z.y * m.w.x +
				-m.x.w * m.y.x * m.z.y * m.w.z +
				-m.x.w * m.y.y * m.z.z * m.w.x +
				-m.x.w * m.y.z * m.z.x * m.w.y));

		float a00 = (m.y.y * m.z.z * m.w.w +
			m.y.z * m.z.w * m.w.y +
			m.y.w * m.z.y * m.w.z +
			-m.y.y * m.z.w * m.w.z +
			-m.y.z * m.z.y * m.w.w +
			-m.y.w * m.z.z * m.w.y);

		float a10 = (m.x.y * m.z.w * m.w.z +
			m.x.z * m.z.y * m.w.w +
			m.x.w * m.z.z * m.w.y +
			-m.x.y * m.z.z * m.w.w +
			-m.x.z * m.z.w * m.w.y +
			-m.x.w * m.z.y * m.w.z);

		float a20 = (m.x.y * m.y.z * m.w.w +
			m.x.z * m.y.w * m.w.y +
			m.x.w * m.y.y * m.w.z +
			-m.x.y * m.y.w * m.w.z +
			-m.x.z * m.y.y * m.w.w +
			-m.x.w * m.y.z * m.w.y);

		float a30 = (m.x.y * m.y.w * m.z.z +
			m.x.z * m.y.y * m.z.w +
			m.x.w * m.y.z * m.z.y +
			-m.x.y * m.y.z * m.z.w +
			-m.x.z * m.y.w * m.z.y +
			-m.x.w * m.y.y * m.z.z);

		float a01 = (m.y.x * m.z.w * m.w.z +
			m.y.z * m.z.x * m.w.w +
			m.y.w * m.z.z * m.w.x +
			-m.y.x * m.z.z * m.w.w +
			-m.y.z * m.z.w * m.w.x +
			-m.y.w * m.z.x * m.w.z);

		float a11 = (m.x.x * m.z.z * m.w.w +
			m.x.z * m.z.w * m.w.x +
			m.x.w * m.z.x * m.w.z +
			-m.x.x * m.z.w * m.w.z +
			-m.x.z * m.z.x * m.w.w +
			-m.x.w * m.z.z * m.w.x);

		float a21 = (m.x.x * m.y.w * m.w.z +
			m.x.z * m.y.x * m.w.w +
			m.x.w * m.y.z * m.w.x +
			-m.x.x * m.y.z * m.w.w +
			-m.x.z * m.y.w * m.w.x +
			-m.x.w * m.y.x * m.w.z);

		float a31 = (m.x.x * m.y.z * m.z.w +
			m.x.z * m.y.w * m.z.x +
			m.x.w * m.y.x * m.z.z +
			-m.x.x * m.y.w * m.z.z +
			-m.x.z * m.y.x * m.z.w +
			-m.x.w * m.y.z * m.z.x);

		float a02 = (m.y.x * m.z.y * m.w.w +
			m.y.y * m.z.w * m.w.x +
			m.y.w * m.z.x * m.w.y +
			-m.y.x * m.z.w * m.w.y +
			-m.y.y * m.z.x * m.w.w +
			-m.y.w * m.z.y * m.w.x);

		float a12 = (-m.x.x * m.z.y * m.w.w +
			-m.x.y * m.z.w * m.w.x +
			-m.x.w * m.z.x * m.w.y +
			m.x.x * m.z.w * m.w.y +
			m.x.y * m.z.x * m.w.w +
			m.x.w * m.z.y * m.w.x);

		float a22 = (m.x.x * m.y.y * m.w.w +
			m.x.y * m.y.w * m.w.x +
			m.x.w * m.y.x * m.w.y +
			-m.x.x * m.y.w * m.w.y +
			-m.x.y * m.y.x * m.w.w +
			-m.x.w * m.y.y * m.w.x);

		float a32 = (m.x.x * m.y.w * m.z.y +
			m.x.y * m.y.x * m.z.w +
			m.x.w * m.y.y * m.z.x +
			-m.x.y * m.y.w * m.z.x +
			-m.x.w * m.y.x * m.z.y +
			-m.x.x * m.y.y * m.z.w);

		float a03 = (m.y.x * m.z.z * m.w.y +
			m.y.y * m.z.x * m.w.z +
			m.y.z * m.z.y * m.w.x +
			-m.y.x * m.z.y * m.w.z +
			-m.y.y * m.z.z * m.w.x +
			-m.y.z * m.z.x * m.w.y);

		float a13 = (m.x.x * m.z.y * m.w.z +
			m.x.y * m.z.z * m.w.x +
			m.x.z * m.z.x * m.w.y +
			-m.x.x * m.z.z * m.w.y +
			-m.x.y * m.z.x * m.w.z +
			-m.x.z * m.z.y * m.w.x);

		float a23 = (m.x.x * m.y.z * m.w.y +
			m.x.y * m.y.x * m.w.z +
			m.x.z * m.y.y * m.w.x +
			-m.x.x * m.y.y * m.w.z +
			-m.x.y * m.y.z * m.w.x +
			-m.x.z * m.y.x * m.w.y);

		float a33 = (m.x.x * m.y.y * m.z.z +
			m.x.y * m.y.z * m.z.x +
			m.x.z * m.y.x * m.z.y +
			-m.x.x * m.y.z * m.z.y +
			-m.x.y * m.y.x * m.z.z +
			-m.x.z * m.y.y * m.z.x);

		return NvFlowFloat4x4{ 
			a00*f, a10*f, a20*f, a30*f,
			a01*f, a11*f, a21*f, a31*f,
			a02*f, a12*f, a22*f, a32*f,
			a03*f, a13*f, a23*f, a33*f };
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixIdentity()
	{
		return NvFlowFloat4x4{
			{ 1.f, 0.f, 0.f, 0.f },
			{ 0.f, 1.f, 0.f, 0.f },
			{ 0.f, 0.f, 1.f, 0.f },
			{ 0.f, 0.f, 0.f, 1.f }
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixScaling(float x, float y, float z)
	{
		return NvFlowFloat4x4{
			x, 0.f, 0.f, 0.f,
			0.f, y, 0.f, 0.f,
			0.f, 0.f, z, 0.f,
			0.f, 0.f, 0.f, 1.f
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixTranslation(float x, float y, float z)
	{
		return NvFlowFloat4x4{
			1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
				x,   y,   z, 1.f
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixPerspectiveFovRH(float fovAngleY, float aspectRatio, float nearZ, float farZ)
	{
		float sinfov = sinf(0.5f * fovAngleY);
		float cosfov = cosf(0.5f * fovAngleY);

		float height = cosfov / sinfov;
		float width = height / aspectRatio;
		float frange = farZ / (nearZ - farZ);

		if (nearZ == INFINITY)
		{
			return NvFlowFloat4x4{
				{ width, 0.f, 0.f, 0.f },
				{ 0.f, height, 0.f, 0.f },
				{ 0.f, 0.f, frange, -1.f },
				{ 0.f, 0.f, farZ, 0.f }
			};
		}

		return NvFlowFloat4x4{
			{ width, 0.f, 0.f, 0.f },
			{ 0.f, height, 0.f, 0.f },
			{ 0.f, 0.f, frange, -1.f },
			{ 0.f, 0.f, frange * nearZ, 0.f }
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixPerspectiveFovLH(float fovAngleY, float aspectRatio, float nearZ, float farZ)
	{
		float sinfov = sinf(0.5f * fovAngleY);
		float cosfov = cosf(0.5f * fovAngleY);

		float height = cosfov / sinfov;
		float width = height / aspectRatio;
		float frange = farZ / (farZ - nearZ);

		if (nearZ == INFINITY)
		{
			return NvFlowFloat4x4{
				{ width, 0.f, 0.f, 0.f },
				{ 0.f, height, 0.f, 0.f },
				{ 0.f, 0.f, frange, 1.f },
				{ 0.f, 0.f, farZ, 0.f }
			};
		}

		return NvFlowFloat4x4{
			{ width, 0.f, 0.f, 0.f },
			{ 0.f, height, 0.f, 0.f },
			{ 0.f, 0.f, frange, 1.f },
			{ 0.f, 0.f, -frange * nearZ, 0.f }
		};
	}

	NV_FLOW_INLINE NvFlowBool32 matrixPerspectiveIsRH(const NvFlowFloat4x4& m)
	{
		return m.z.w < 0.f ? NV_FLOW_TRUE : NV_FLOW_FALSE;
	}

	NV_FLOW_INLINE NvFlowBool32 matrixPerspectiveIsReverseZ(const NvFlowFloat4x4& m)
	{
		float nearZ = -m.w.z / m.z.z;
		float farZ = (m.w.w - m.w.z) / (m.z.z - m.z.w);
		float singZ = -m.w.w / m.z.w;
		return fabsf(farZ - singZ) < fabs(nearZ - singZ) ? NV_FLOW_TRUE : NV_FLOW_FALSE;
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixOrthographicLH(float width, float height, float nearZ, float farZ)
	{
		float frange = 1.f / (farZ - nearZ);

		return NvFlowFloat4x4{
			{ 2.f / width, 0.f, 0.f, 0.f },
			{ 0.f, 2.f / height, 0.f, 0.f },
			{ 0.f, 0.f, frange, 0.f },
			{ 0.f, 0.f, -frange * nearZ, 1.f }
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixOrthographicRH(float width, float height, float nearZ, float farZ)
	{
		float frange = 1.f / (nearZ - farZ);

		return NvFlowFloat4x4{
			{ 2.f / width, 0.f, 0.f, 0.f },
			{ 0.f, 2.f / height, 0.f, 0.f },
			{ 0.f, 0.f, frange, 0.f },
			{ 0.f, 0.f, frange * nearZ, 1.f }
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixRotationNormal(NvFlowFloat4 normal, float angle)
	{
		float sinAngle = sinf(angle);
		float cosAngle = cosf(angle);

		NvFlowFloat4 a = { sinAngle, cosAngle, 1.f - cosAngle, 0.f };

		NvFlowFloat4 c2 = vectorSplatZ(a);
		NvFlowFloat4 c1 = vectorSplatY(a);
		NvFlowFloat4 c0 = vectorSplatX(a);

		NvFlowFloat4 n0 = { normal.y, normal.z, normal.x, normal.w };
		NvFlowFloat4 n1 = { normal.z, normal.x, normal.y, normal.w };

		NvFlowFloat4 v0 = c2 * n0;
		v0 = v0 * n1;

		NvFlowFloat4 r0 = c2 * normal;
		r0 = (r0 * normal) + c1;

		NvFlowFloat4 r1 = (c0 * normal) + v0;
		NvFlowFloat4 r2 = v0 - (c0 * normal);

		v0 = NvFlowFloat4{ r0.x, r0.y, r0.z, a.w };
		NvFlowFloat4 v1 = { r1.z, r2.y, r2.z, r1.x };
		NvFlowFloat4 v2 = { r1.y, r2.x, r1.y, r2.x };

		return NvFlowFloat4x4{
			{ v0.x, v1.x, v1.y, v0.w },
			{ v1.z, v0.y, v1.w, v0.w },
			{ v2.x, v2.y, v0.z, v0.w },
			{ 0.f, 0.f, 0.f, 1.f }
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixRotationAxis(NvFlowFloat4 axis, float angle)
	{
		NvFlowFloat4 normal = vector3Normalize(axis);
		return matrixRotationNormal(normal, angle);
	}

	NV_FLOW_INLINE NvFlowFloat4 quaterionRotationRollPitchYawFromVector(NvFlowFloat4 angles)
	{
		NvFlowFloat4 sign = { 1.f, -1.f, -1.f, 1.f };

		NvFlowFloat4 halfAngles = angles * NvFlowFloat4{ 0.5f, 0.5f, 0.5f, 0.5f };

		NvFlowFloat4 sinAngle = NvFlowFloat4{ sinf(halfAngles.x), sinf(halfAngles.y), sinf(halfAngles.z), sinf(halfAngles.w) };
		NvFlowFloat4 cosAngle = NvFlowFloat4{ cosf(halfAngles.x), cosf(halfAngles.y), cosf(halfAngles.z), cosf(halfAngles.w) };

		NvFlowFloat4 p0 = { sinAngle.x, cosAngle.x, cosAngle.x, cosAngle.x };
		NvFlowFloat4 y0 = { cosAngle.y, sinAngle.y, cosAngle.y, cosAngle.y };
		NvFlowFloat4 r0 = { cosAngle.z, cosAngle.z, sinAngle.z, cosAngle.z };
		NvFlowFloat4 p1 = { cosAngle.x, sinAngle.x, sinAngle.x, sinAngle.x };
		NvFlowFloat4 y1 = { sinAngle.y, cosAngle.y, sinAngle.y, sinAngle.y };
		NvFlowFloat4 r1 = { sinAngle.z, sinAngle.z, cosAngle.z, sinAngle.z };

		NvFlowFloat4 q1 = p1 * sign;
		NvFlowFloat4 q0 = p0 * y0;
		q1 = q1 * y1;
		q0 = q0 * r0;
		NvFlowFloat4 q = (q1 * r1) + q0;

		return q;
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixRotationQuaternion(NvFlowFloat4 quaternion)
	{
		NvFlowFloat4 constant1110 = { 1.f, 1.f, 1.f, 0.f };

		NvFlowFloat4 q0 = quaternion + quaternion;
		NvFlowFloat4 q1 = quaternion * q0;

		NvFlowFloat4 v0 = { q1.y, q1.x, q1.x, constant1110.w };
		NvFlowFloat4 v1 = { q1.z, q1.z, q1.y, constant1110.w };
		NvFlowFloat4 r0 = constant1110 - v0;
		r0 = r0 - v1;

		v0 = NvFlowFloat4{ quaternion.x, quaternion.x, quaternion.y, quaternion.w };
		v1 = NvFlowFloat4{ q0.z, q0.y, q0.z, q0.w };
		v0 = v0 * v1;

		v1 = vectorSplatW(quaternion);
		NvFlowFloat4 v2 = { q0.y, q0.z, q0.x, q0.w };
		v1 = v1 * v2;

		NvFlowFloat4 r1 = v0 + v1;
		NvFlowFloat4 r2 = v0 - v1;

		v0 = NvFlowFloat4{ r1.y, r2.x, r2.y, r1.z };
		v1 = NvFlowFloat4{ r1.x, r2.z, r1.x, r2.z };

		return NvFlowFloat4x4{
			{ r0.x, v0.x, v0.y, r0.w },
			{ v0.z, r0.y, v0.w, r0.w },
			{ v1.x, v1.y, r0.z, r0.w },
			{ 0.f, 0.f, 0.f, 1.f }
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixRotationRollPitchYaw(float pitch, float yaw, float roll)
	{
		NvFlowFloat4 angles = { pitch, yaw, roll, 0.f };
		NvFlowFloat4 q = quaterionRotationRollPitchYawFromVector(angles);
		return matrixRotationQuaternion(q);
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorLerp(NvFlowFloat4 a, NvFlowFloat4 b, float t)
	{
		return NvFlowFloat4{
			(1.f - t) * a.x + t * b.x,
			(1.f - t) * a.y + t * b.y,
			(1.f - t) * a.z + t * b.z,
			(1.f - t) * a.w + t * b.w
		};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixInterpolateTranslation(const NvFlowFloat4x4& a, const NvFlowFloat4x4& b, float t)
	{
		NvFlowFloat4x4 ret;
		if (t < 0.5f)
		{
			ret = a;
		}
		else
		{
			ret = b;
		}
		ret.w.x = (1.f - t) * a.w.x + t * b.w.x;
		ret.w.y = (1.f - t) * a.w.y + t * b.w.y;
		ret.w.z = (1.f - t) * a.w.z + t * b.w.z;
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 vector4Normalize(const NvFlowFloat4& v)
	{
		float magn = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);

		if (magn > 0.f)
		{
			magn = 1.f / magn;
		}

		return NvFlowFloat4{v.x * magn,	v.y * magn,	v.z * magn,	v.w * magn};
	}

	NV_FLOW_INLINE NvFlowFloat4x4 matrixNormalize(const NvFlowFloat4x4& a)
	{
		NvFlowFloat4x4 temp = a;
		temp.x.w = 0.f;
		temp.y.w = 0.f;
		temp.z.w = 0.f;
		temp.w.w = 1.f;
		temp.w.x = 0.f;
		temp.w.y = 0.f;
		temp.w.z = 0.f;
		NvFlowFloat4x4 ret = temp;
		ret.x = vector4Normalize(ret.x);
		ret.y = vector4Normalize(ret.y);
		ret.z = vector4Normalize(ret.z);
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 vector4Transform(const NvFlowFloat4& x, const NvFlowFloat4x4& A)
	{
		return NvFlowFloat4{
			A.x.x * x.x + A.y.x * x.y + A.z.x * x.z + A.w.x * x.w,
			A.x.y * x.x + A.y.y * x.y + A.z.y * x.z + A.w.y * x.w,
			A.x.z * x.x + A.y.z * x.y + A.z.z * x.z + A.w.z * x.w,
			A.x.w * x.x + A.y.w * x.y + A.z.w * x.z + A.w.w * x.w
		};
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorMin(const NvFlowFloat4& a, const NvFlowFloat4& b)
	{
		return NvFlowFloat4{
			a.x < b.x ? a.x : b.x,
			a.y < b.y ? a.y : b.y,
			a.z < b.z ? a.z : b.z,
			a.w < b.w ? a.w : b.w
		};
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorMax(const NvFlowFloat4& a, const NvFlowFloat4& b)
	{
		return NvFlowFloat4{
			a.x > b.x ? a.x : b.x,
			a.y > b.y ? a.y : b.y,
			a.z > b.z ? a.z : b.z,
			a.w > b.w ? a.w : b.w
		};
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorMultiply(const NvFlowFloat4& a, const NvFlowFloat4& b)
	{
		return NvFlowFloat4{a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorFloor(const NvFlowFloat4& a)
	{
		return NvFlowFloat4{ floorf(a.x), floorf(a.y), floorf(a.z), floorf(a.w) };
	}

	NV_FLOW_INLINE NvFlowFloat4 vectorCeiling(const NvFlowFloat4& a)
	{
		return NvFlowFloat4{ceilf(a.x), ceilf(a.y), ceilf(a.z), ceilf(a.w)};
	}

	NV_FLOW_INLINE NvFlowFloat4 vector3Dot(const NvFlowFloat4& a, const NvFlowFloat4& b)
	{
		float magn = a.x * b.x + a.y * b.y + a.z * b.z;

		return NvFlowFloat4{ magn, magn, magn, magn };
	}

	NV_FLOW_INLINE NvFlowFloat4 vector4Dot(const NvFlowFloat4& a, const NvFlowFloat4& b)
	{
		float magn = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

		return NvFlowFloat4{ magn, magn, magn, magn };
	}

	NV_FLOW_INLINE NvFlowFloat4 vector3Cross(const NvFlowFloat4& a, const NvFlowFloat4& b)
	{
		return NvFlowFloat4{
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x,
			0.f
		};
	}

	NV_FLOW_INLINE NvFlowFloat4 vector3Length(const NvFlowFloat4& a)
	{
		float magn = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);

		return NvFlowFloat4{ magn, magn, magn, magn };
	}

	NV_FLOW_INLINE NvFlowFloat4 make_float4(NvFlowFloat3 a, float b)
	{
		return NvFlowFloat4{ a.x, a.y, a.z, b };
	}

	NV_FLOW_INLINE NvFlowFloat3 float4_to_float3(NvFlowFloat4 a)
	{
		return NvFlowFloat3{ a.x, a.y, a.z };
	}

	NV_FLOW_INLINE NvFlowUint log2ui(NvFlowUint val)
	{
		NvFlowUint ret = 0;
		for (NvFlowUint i = 0; i < 32; i++)
		{
			if ((1u << i) >= val)
			{
				ret = i;
				break;
			}
		}
		return ret;
	}

	NV_FLOW_INLINE NvFlowFloat4 computeRayOrigin(const NvFlowFloat4x4& viewInv, const NvFlowFloat4x4& projectionInv, NvFlowFloat2 ndc, float nearZ)
	{
		NvFlowFloat4 viewPos = vector4Transform(NvFlowFloat4{ ndc.x, ndc.y, nearZ, 1.f }, projectionInv);
		return vectorPerspectiveDivide(vector4Transform(viewPos, viewInv));
	}

	NV_FLOW_INLINE NvFlowFloat4 computeRayDir(const NvFlowFloat4x4& viewInv, const NvFlowFloat4x4& projectionInv, NvFlowFloat2 ndc, float nearZ)
	{
		NvFlowFloat4x4 projectionInvT = matrixTranspose(projectionInv);
		NvFlowFloat4 ndc_ext = NvFlowFloat4{ ndc.x, ndc.y, 0.f, 1.f };
		NvFlowFloat4 dir = {
			-projectionInvT.w.z * vector4Dot(projectionInvT.x, ndc_ext).x,
			-projectionInvT.w.z * vector4Dot(projectionInvT.y, ndc_ext).x,
			-projectionInvT.w.z * vector4Dot(projectionInvT.z, ndc_ext).x +
			projectionInvT.z.z * vector4Dot(projectionInvT.w, ndc_ext).x,
			0.f
		};
		if (nearZ > 0.5f)
		{
			dir = NvFlowFloat4{ 0.f, 0.f, 0.f, 0.f } - dir;
		}
		return vector4Transform(dir, viewInv);
	}

	struct FrustumRays
	{
		NvFlowFloat4 rayOrigin00;
		NvFlowFloat4 rayOrigin10;
		NvFlowFloat4 rayOrigin01;
		NvFlowFloat4 rayOrigin11;
		NvFlowFloat4 rayDir00;
		NvFlowFloat4 rayDir10;
		NvFlowFloat4 rayDir01;
		NvFlowFloat4 rayDir11;
		float nearZ;
		NvFlowBool32 isReverseZ;
	};

	NV_FLOW_INLINE void computeFrustumRays(FrustumRays* ptr, const NvFlowFloat4x4& viewInv, const NvFlowFloat4x4& projectionInv)
	{
		NvFlowFloat4 nearPoint = vector4Transform(NvFlowFloat4{ 0.f, 0.f, 0.f, 1.f }, projectionInv);
		NvFlowFloat4 farPoint = vector4Transform(NvFlowFloat4{ 0.f, 0.f, 1.f, 1.f }, projectionInv);

		nearPoint = nearPoint / vectorSplatW(nearPoint);
		farPoint = farPoint / vectorSplatW(farPoint);

		float nearZ = fabsf(nearPoint.z) < fabsf(farPoint.z) ? 0.f : 1.f;

		ptr->rayOrigin00 = computeRayOrigin(viewInv, projectionInv, NvFlowFloat2{ -1.f, +1.f }, nearZ);
		ptr->rayOrigin10 = computeRayOrigin(viewInv, projectionInv, NvFlowFloat2{ +1.f, +1.f }, nearZ);
		ptr->rayOrigin01 = computeRayOrigin(viewInv, projectionInv, NvFlowFloat2{ -1.f, -1.f }, nearZ);
		ptr->rayOrigin11 = computeRayOrigin(viewInv, projectionInv, NvFlowFloat2{ +1.f, -1.f }, nearZ);

		ptr->rayDir00 = computeRayDir(viewInv, projectionInv, NvFlowFloat2{ -1.f, +1.f }, nearZ);
		ptr->rayDir10 = computeRayDir(viewInv, projectionInv, NvFlowFloat2{ +1.f, +1.f }, nearZ);
		ptr->rayDir01 = computeRayDir(viewInv, projectionInv, NvFlowFloat2{ -1.f, -1.f }, nearZ);
		ptr->rayDir11 = computeRayDir(viewInv, projectionInv, NvFlowFloat2{ +1.f, -1.f }, nearZ);

		ptr->nearZ = nearZ;
		ptr->isReverseZ = fabsf(nearPoint.z) >= fabsf(farPoint.z);
	}
}