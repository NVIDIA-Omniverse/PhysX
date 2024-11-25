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

#include "foundation/PxBasicTemplates.h"
#include "geometry/PxConvexCoreGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "GuConvexGeometry.h"
#include "GuConvexSupport.h"
#include "GuBounds.h"
#include "common/PxRenderOutput.h"

using namespace physx;

// This enums should match
PX_COMPILE_TIME_ASSERT(PxU32(Gu::ConvexCore::Type::ePOINT) == PxU32(PxConvexCore::ePOINT));
PX_COMPILE_TIME_ASSERT(PxU32(Gu::ConvexCore::Type::eSEGMENT) == PxU32(PxConvexCore::eSEGMENT));
PX_COMPILE_TIME_ASSERT(PxU32(Gu::ConvexCore::Type::eBOX) == PxU32(PxConvexCore::eBOX));
PX_COMPILE_TIME_ASSERT(PxU32(Gu::ConvexCore::Type::eELLIPSOID) == PxU32(PxConvexCore::eELLIPSOID));
PX_COMPILE_TIME_ASSERT(PxU32(Gu::ConvexCore::Type::eCYLINDER) == PxU32(PxConvexCore::eCYLINDER));

PX_COMPILE_TIME_ASSERT(Gu::ConvexCore::MAX_CORE_SIZE >= PxConvexCoreGeometry::MAX_CORE_SIZE);

static const PxU32 GEOMETRY_COLOR = PxU32(PxDebugColor::eARGB_MAGENTA);
static const PxU32 GEOMETRY_CORE_COLOR = PxU32(0x88880088); // dark magenta

namespace aux
{
	PxReal coneRadiusAtHeight(PxReal height, PxReal radius, PxReal margin, PxReal h)
	{
		float angle = atan2f(radius, height);
		float aSin = sinf(angle);
		float aCos = cosf(angle);

		if (h > -height * 0.5f + margin * aSin && h < height * 0.5f + margin * aSin)
			return radius * (height * 0.5f - h) / height + margin / aCos;

		if (h <= -height * 0.5f + margin * aSin)
		{
			float a = -h - height * 0.5f;
			return radius + sqrtf(margin * margin - a * a);
		}

		if (h >= height * 0.5f + margin * aSin)
		{
			float a = h - height * 0.5f;
			return sqrtf(margin * margin - a * a);
		}

		PX_ASSERT(0);
		return 0;
	}
}

namespace debug
{
	static void drawArc(const PxVec3& center, const PxVec3& radius, const PxVec3& axis, PxReal angle, PxReal error, PxRenderOutput& out)
	{
		int sides = int(ceilf(angle / (2 * acosf(1.0f - error))));
		float step = angle / sides;
		out << PxRenderOutput::LINESTRIP;
		for (int i = 0; i <= sides; ++i)
			out << center + PxQuat(step * i, axis).rotate(radius);
	}
	static void drawCircle(const PxVec3& center, const PxVec3& radius, const PxVec3& axis, PxReal error, PxRenderOutput& out)
	{
		drawArc(center, radius, axis, PxTwoPi, error, out);
	}
	static void drawQuarterCircle(const PxVec3& center, const PxVec3& radius, const PxVec3& axis, PxReal error, PxRenderOutput& out)
	{
		drawArc(center, radius, axis, PxPiDivTwo, error, out);
	}
	static void drawLine(const PxVec3& s, const PxVec3& e, PxRenderOutput& out)
	{
		out << PxRenderOutput::LINES << s << e;
	}
	static void drawSphere(PxReal radius, PxReal error, PxRenderOutput& out)
	{
		drawCircle(PxVec3(0), PxVec3(0, radius, 0), PxVec3(1, 0, 0), error, out);
		drawCircle(PxVec3(0), PxVec3(0, 0, radius), PxVec3(0, 1, 0), error, out);
		drawCircle(PxVec3(0), PxVec3(radius, 0, 0), PxVec3(0, 0, 1), error, out);
	}
	static void drawCapsule(PxReal length, PxReal radius, PxReal error, PxRenderOutput& out)
	{
		drawLine(PxVec3(length * 0.5f, radius, 0), PxVec3(-length * 0.5f, radius, 0), out);
		drawLine(PxVec3(length * 0.5f, -radius, 0), PxVec3(-length * 0.5f, -radius, 0), out);
		drawLine(PxVec3(length * 0.5f, 0, radius), PxVec3(-length * 0.5f, 0, radius), out);
		drawLine(PxVec3(length * 0.5f, 0, -radius), PxVec3(-length * 0.5f, 0, -radius), out);
		drawCircle(PxVec3(length * 0.5f, 0, 0), PxVec3(0, radius, 0), PxVec3(1, 0, 0), error, out);
		drawCircle(PxVec3(-length * 0.5f, 0, 0), PxVec3(0, radius, 0), PxVec3(1, 0, 0), error, out);
		drawArc(PxVec3(length * 0.5f, 0, 0), PxVec3(0, radius, 0), PxVec3(0, 0, -1), PxPi, error, out);
		drawArc(PxVec3(length * 0.5f, 0, 0), PxVec3(0, 0, radius), PxVec3(0, 1, 0), PxPi, error, out);
		drawArc(PxVec3(-length * 0.5f, 0, 0), PxVec3(0, radius, 0), PxVec3(0, 0, 1), PxPi, error, out);
		drawArc(PxVec3(-length * 0.5f, 0, 0), PxVec3(0, 0, radius), PxVec3(0, -1, 0), PxPi, error, out);
	}
	static void drawBox(const PxVec3& extents, PxReal margin, PxReal error, PxRenderOutput& out)
	{
		for (PxU32 i = 0; i < 3; ++i)
		{
			PxU32 axis0 = i, axis1 = (i + 1) % 3, axis2 = (i + 2) % 3;
			PxVec3 ax0(0), ax1(0), ax2(0);
			ax0[axis0] = 1; ax1[axis1] = 1; ax2[axis2] = 1;
			PxReal s[4][2] = { { 1, 1 }, { -1, 1 }, { -1,-1 }, { 1, -1 } };
			for (PxU32 j = 0; j < 4; ++j)
			{
				PxVec3 c(0);
				c[axis1] = extents[axis1] * 0.5f * s[j][0];
				c[axis2] = extents[axis2] * 0.5f * s[j][1];
				if (margin > FLT_EPSILON)
				{
					drawLine(c + ax0 * extents[axis0] * 0.5f + ax1 * margin * s[j][0], c - ax0 * extents[axis0] * 0.5f + ax1 * margin * s[j][0], out);
					drawLine(c + ax0 * extents[axis0] * 0.5f + ax2 * margin * s[j][1], c - ax0 * extents[axis0] * 0.5f + ax2 * margin * s[j][1], out);
					drawQuarterCircle(c + ax0 * extents[axis0] * 0.5f, ax1 * margin * s[j][0], ax0 * (j % 2 ? -1.0f : 1.0f), error, out);
					drawQuarterCircle(c - ax0 * extents[axis0] * 0.5f, ax1 * margin * s[j][0], ax0 * (j % 2 ? -1.0f : 1.0f), error, out);
				}
				else
					drawLine(c + ax0 * extents[axis0] * 0.5f, c - ax0 * extents[axis0] * 0.5f, out);
			}
		}
	}
	static void drawEllipse(const PxVec3& center, const PxVec3& axis0, PxReal radius0, const PxVec3& axis1, PxReal radius1, PxReal margin, PxReal error, PxRenderOutput& out)
	{
		if (radius0 * radius1 > 0)
		{
			out << PxRenderOutput::LINESTRIP;
			for (PxReal t = 0, dT = 0; t - dT < PxTwoPi; t += dT)
			{
				PxReal si, co; PxSinCos(t, si, co);
				PxVec3 p = axis0 * radius0 * co + axis1 * radius1 * si;
				PxVec3 tang = -axis0 * radius0 * si + axis1 * radius1 * co;
				PxVec3 norm = axis1.cross(axis0).cross(tang).getNormalized();
				p += norm * margin;
				out << center + p;
				PxReal d0, d1, d20, d21, C;
				d0 = -radius0 * si; d1 = radius1 * co; d20 = -radius0 * co; d21 = -radius1 * si;
				C = PxAbs(d0 * d21 - d1 * d20) / PxPow(d0 * d0 + d1 * d1, 1.5f);
				dT = 100.0f * error / (1.0f + C) / PxMax(radius0, radius1);
			}
		}
		else
		{
			const PxVec3 axis = axis1.cross(axis0);
			if (radius0 > 0)
			{
				drawLine(center + axis0 * radius0 + axis1 * margin, center - axis0 * radius0 + axis1 * margin, out);
				drawLine(center + axis0 * radius0 - axis1 * margin, center - axis0 * radius0 - axis1 * margin, out);
				drawArc(center + axis0 * radius0, axis1 * margin, axis, PxPi, error, out);
				drawArc(center - axis0 * radius0, axis1 * margin, -axis, PxPi, error, out);
			}
			else if (radius1 > 0)
			{
				drawLine(center + axis1 * radius1 + axis0 * margin, center - axis1 * radius1 + axis0 * margin, out);
				drawLine(center + axis1 * radius1 - axis0 * margin, center - axis1 * radius1 - axis0 * margin, out);
				drawArc(center + axis1 * radius1, axis0 * margin, -axis, PxPi, error, out);
				drawArc(center - axis1 * radius1, axis0 * margin, axis, PxPi, error, out);
			}
			else
			{
				drawArc(center - axis1 * radius1, axis0 * margin, axis, PxTwoPi, error, out);
			}
		}
	}
	static void drawEllipsoid(const PxVec3& radii, PxReal margin, PxReal error, PxRenderOutput& out)
	{
		const PxVec3 zero(0), X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
		drawEllipse(zero, X, radii.x, Y, radii.y, margin, error, out);
		drawEllipse(zero, X, radii.x, Z, radii.z, margin, error, out);
		drawEllipse(zero, Z, radii.z, Y, radii.y, margin, error, out);
	}
	static void drawCylinder(PxReal height, PxReal radius, PxReal margin, PxReal error, PxRenderOutput& out)
	{
		const PxReal ERR = error;

		PxU32 axis0 = 0, axis1 = 1, axis2 = 2;

		PxVec3 zr(PxZero), rd(PxZero), ax(PxZero), ax1(PxZero), ax2(PxZero), r0(PxZero), r1(PxZero);
		ax[axis0] = ax1[axis1] = ax2[axis2] = 1.0f;
		r0[axis1] = r1[axis2] = radius;

		rd[axis1] = radius;
		rd[axis0] = -(height * 0.5f + margin);
		debug::drawCircle(zr, rd, ax, ERR, out);
		rd[axis0] = (height * 0.5f + margin);
		debug::drawCircle(zr, rd, ax, ERR, out);
		rd[axis1] = radius + margin;
		rd[axis0] = -(height * 0.5f);
		debug::drawCircle(zr, rd, ax, ERR, out);
		rd[axis0] = (height * 0.5f);
		debug::drawCircle(zr, rd, ax, ERR, out);

		debug::drawLine(-ax * height * 0.5f + ax1 * (radius + margin), ax * height * 0.5f + ax1 * (radius + margin), out);
		debug::drawLine(-ax * height * 0.5f - ax1 * (radius + margin), ax * height * 0.5f - ax1 * (radius + margin), out);
		debug::drawLine(-ax * height * 0.5f + ax2 * (radius + margin), ax * height * 0.5f + ax2 * (radius + margin), out);
		debug::drawLine(-ax * height * 0.5f - ax2 * (radius + margin), ax * height * 0.5f - ax2 * (radius + margin), out);

		debug::drawQuarterCircle(-ax * height * 0.5f + ax1 * radius, -ax * margin, -ax2, ERR, out);
		debug::drawQuarterCircle(-ax * height * 0.5f - ax1 * radius, -ax * margin, ax2, ERR, out);
		debug::drawQuarterCircle(-ax * height * 0.5f + ax2 * radius, -ax * margin, ax1, ERR, out);
		debug::drawQuarterCircle(-ax * height * 0.5f - ax2 * radius, -ax * margin, -ax1, ERR, out);

		debug::drawQuarterCircle(ax * height * 0.5f + ax1 * radius, ax * margin, ax2, ERR, out);
		debug::drawQuarterCircle(ax * height * 0.5f - ax1 * radius, ax * margin, -ax2, ERR, out);
		debug::drawQuarterCircle(ax * height * 0.5f + ax2 * radius, ax * margin, -ax1, ERR, out);
		debug::drawQuarterCircle(ax * height * 0.5f - ax2 * radius, ax * margin, ax1, ERR, out);
	}
	static void drawCone(PxReal height, PxReal radius, PxReal margin, PxReal error, PxRenderOutput& out)
	{
		const PxReal ERR = error;

		PxU32 axis0 = 0, axis1 = 1, axis2 = 2;

		PxVec3 zr(PxZero), rd(PxZero), ax(PxZero), ax1(PxZero), ax2(PxZero), r0(PxZero), r1(PxZero);
		ax[axis0] = ax1[axis1] = ax2[axis2] = 1.0f;
		r0[axis1] = r1[axis2] = radius;

		float ang = atan2f(radius, height);
		float aSin = sinf(ang);

		rd[axis1] = radius;
		rd[axis0] = -(height * 0.5f + margin);
		drawCircle(zr, rd, ax, ERR, out);
		if (radius >= height && margin > 0)
		{
			rd[axis1] = radius + margin;
			rd[axis0] = -height * 0.5f;
			drawCircle(zr, rd, ax, ERR, out);
		}
		rd[axis0] = -(height * 0.5f) + margin * aSin;
		rd[axis1] = aux::coneRadiusAtHeight(height, radius, margin, rd[axis0]);
		drawCircle(zr, rd, ax, ERR, out);
		rd[axis0] = height * 0.5f + margin * aSin;
		rd[axis1] = aux::coneRadiusAtHeight(height, radius, margin, rd[axis0]);
		drawCircle(zr, rd, ax, ERR, out);

		float h0 = -height * 0.5f + margin * aSin, h1 = height * 0.5f + margin * aSin;
		float s0 = aux::coneRadiusAtHeight(height, radius, margin, h0), s1 = aux::coneRadiusAtHeight(height, radius, margin, h1);
		debug::drawLine(ax * h0 + ax1 * s0, ax * h1 + ax1 * s1, out);
		debug::drawLine(ax * h0 - ax1 * s0, ax * h1 - ax1 * s1, out);
		debug::drawLine(ax * h0 + ax2 * s0, ax * h1 + ax2 * s1, out);
		debug::drawLine(ax * h0 - ax2 * s0, ax * h1 - ax2 * s1, out);

		debug::drawArc(-ax * height * 0.5f + ax1 * radius, -ax * margin, -ax2, PxPiDivTwo + ang, ERR, out);
		debug::drawArc(-ax * height * 0.5f - ax1 * radius, -ax * margin, ax2, PxPiDivTwo + ang, ERR, out);
		debug::drawArc(-ax * height * 0.5f + ax2 * radius, -ax * margin, ax1, PxPiDivTwo + ang, ERR, out);
		debug::drawArc(-ax * height * 0.5f - ax2 * radius, -ax * margin, -ax1, PxPiDivTwo + ang, ERR, out);

		debug::drawArc(ax * height * 0.5f, ax * margin, ax2, PxPiDivTwo - ang, ERR, out);
		debug::drawArc(ax * height * 0.5f, ax * margin, -ax2, PxPiDivTwo - ang, ERR, out);
		debug::drawArc(ax * height * 0.5f, ax * margin, -ax1, PxPiDivTwo - ang, ERR, out);
		debug::drawArc(ax * height * 0.5f, ax * margin, ax1, PxPiDivTwo - ang, ERR, out);
	}
}

PX_PHYSX_COMMON_API bool PxConvexCoreGeometry::isValid() const
{
	const PxReal margin = getMargin();

	if (margin < 0)
		return false;

	switch (getCoreType())
	{
		case PxConvexCore::ePOINT:
			return margin > 0; // has volume

		case PxConvexCore::eSEGMENT:
		{
			const PxConvexCore::Segment& c = getCore<PxConvexCore::Segment>();
			return c.length >= 0 &&
				(margin + c.length) * margin > 0; // has volume
		}

		case PxConvexCore::eBOX:
		{
			const PxConvexCore::Box& c = getCore<PxConvexCore::Box>();
			return c.extents.x >= 0 && c.extents.y >= 0 && c.extents.z >= 0 &&
				(margin + c.extents.x) * (margin + c.extents.y) * (margin + c.extents.z) > 0; // has volume
		}

		case PxConvexCore::eELLIPSOID:
		{
			const PxConvexCore::Ellipsoid& c = getCore<PxConvexCore::Ellipsoid>();
			return c.radii.x >= 0 && c.radii.y >= 0 && c.radii.z >= 0 &&
				(margin + c.radii.x) * (margin + c.radii.y) * (margin + c.radii.z) > 0; // has volume
		}

		case PxConvexCore::eCYLINDER:
		{
			const PxConvexCore::Cylinder& c = getCore<PxConvexCore::Cylinder>();
			return c.height >= 0 && c.radius >= 0 &&
				(margin + c.height) * (margin + c.radius) > 0; // has volume
		}

		case PxConvexCore::eCONE:
		{
			const PxConvexCore::Cone& c = getCore<PxConvexCore::Cone>();
			return c.height >= 0 && c.radius >= 0 &&
				(margin + c.height) * (margin + c.radius) > 0; // has volume
		}

		default:
			break;
	}

	return false;
}

PX_PHYSX_COMMON_API bool Gu::isGPUCompatible(const PxConvexCoreGeometry& convex)
{
	// there's no types a.t.m. that don't support GPU,
	// but if there will be, we'll return 'false' here.
	switch (convex.getCoreType())
	{
		case PxConvexCore::ePOINT:
		case PxConvexCore::eSEGMENT:
		case PxConvexCore::eBOX:
		case PxConvexCore::eELLIPSOID:
		case PxConvexCore::eCYLINDER:
		case PxConvexCore::eCONE:
			return true;

		default:
			break;
	}

	return false;
}

namespace
{
	struct MassInfo
	{
		PxReal density1Mass;
		PxMat33 inertiaTensor;
		PxVec3 centerOfMass;
	};

	MassInfo cylinderMassInfo(PxReal height, PxReal radius, const PxVec3& centerOfMass = PxVec3(0))
	{
		const PxReal h = height, r = radius;
		const PxReal m = PxPi * r * r * h;
		const PxReal ix = m * r * r / 2;
		const PxReal iyz = m * (3 * r * r + h * h) / 12;
		MassInfo mi;
		mi.density1Mass = m;
		mi.inertiaTensor = PxMat33::createDiagonal(PxVec3(ix, iyz, iyz));
		mi.centerOfMass = centerOfMass;
		return mi;
	}
}

PX_PHYSX_COMMON_API void Gu::computeMassInfo(const PxConvexCoreGeometry& convex, PxReal& density1Mass, PxMat33& inertiaTensor, PxVec3& centerOfMass)
{
	PxReal margin = convex.getMargin();
	switch (convex.getCoreType())
	{
		case PxConvexCore::ePOINT:
		{
			const PxReal r = margin;
			density1Mass = PxPi * r * r * r * 4.0f / 3.0f;
			inertiaTensor = PxMat33::createDiagonal(PxVec3(r * r)) * (density1Mass * 2.0f / 5.0f);
			centerOfMass = PxVec3(0);
			break;
		}

		case PxConvexCore::eCYLINDER:
		{
			const PxConvexCore::Cylinder& core = convex.getCore<PxConvexCore::Cylinder>();
			const PxReal H = core.height + margin * 2;
			const PxReal R = core.radius + margin;
			MassInfo mi = cylinderMassInfo(H, R);
			density1Mass = mi.density1Mass;
			inertiaTensor = mi.inertiaTensor;
			centerOfMass = mi.centerOfMass;
			break;
		}

		case PxConvexCore::eCONE:
		{
			const PxU32 SLICE_COUNT = 32;
			const PxConvexCore::Cone& core = convex.getCore<PxConvexCore::Cone>();
			const PxReal H = core.height + margin * 2;
			const PxReal h = H / SLICE_COUNT;

			MassInfo mis[SLICE_COUNT];

			for (PxU32 i = 0; i < SLICE_COUNT; ++i)
			{
				const PxReal t = -H * 0.5f + i * h + h * 0.5f;
				const PxReal r = aux::coneRadiusAtHeight(core.height, core.radius, margin, t);
				mis[i] = cylinderMassInfo(h, r, PxVec3(t, 0, 0));
			}

			MassInfo mi{ 0, PxMat33(PxZero), PxVec3(0) };

			for (PxU32 i = 0; i < SLICE_COUNT; i++)
			{
				mi.density1Mass += mis[i].density1Mass;
				mi.centerOfMass += mis[i].centerOfMass * mis[i].density1Mass;
			}

			if (mi.density1Mass > 0.f)
				mi.centerOfMass /= mi.density1Mass;

			for (PxU32 i = 0; i < SLICE_COUNT; i++)
			{
				const PxVec3 t = mi.centerOfMass - mis[i].centerOfMass;
				const PxMat33 s(PxVec3(0, t.z, -t.y), PxVec3(-t.z, 0, t.x), PxVec3(t.y, -t.x, 0));
				mi.inertiaTensor += s.getTranspose() * s * mis[i].density1Mass + mis[i].inertiaTensor;
			}

			{
				const PxVec3 t = mi.centerOfMass;
				const PxMat33 s(PxVec3(0, t.z, -t.y), PxVec3(-t.z, 0, t.x), PxVec3(t.y, -t.x, 0));
				mi.inertiaTensor += s.getTranspose() * s * mi.density1Mass;
			}

			density1Mass = mi.density1Mass;
			inertiaTensor = mi.inertiaTensor;
			centerOfMass = mi.centerOfMass;
			break;
		}

		default:
		{
			const PxBounds3 bounds = Gu::computeBounds(convex, PxTransform(PxIdentity));
			const PxVec3 halfExtents = bounds.getDimensions() * 0.5f;
			density1Mass = halfExtents.x * halfExtents.y * halfExtents.z * 8.0f;
			PxVec3 d2 = halfExtents.multiply(halfExtents);
			inertiaTensor = PxMat33::createDiagonal(PxVec3(d2.y + d2.z, d2.x + d2.z, d2.x + d2.y)) * (density1Mass * 1.0f / 3.0f);
			centerOfMass = bounds.getCenter();
			break;
		}
	}
}

PX_PHYSX_COMMON_API void Gu::visualize(const PxConvexCoreGeometry& convex, const PxTransform& pose, bool drawCore, const PxBounds3& cullbox, PxRenderOutput& out)
{
	PX_UNUSED(cullbox);

	PxReal margin = convex.getMargin();

	switch (convex.getCoreType())
	{
		case PxConvexCore::ePOINT:
		{
			const PxReal error = 0.001f;

			out << pose;
			out << GEOMETRY_COLOR;
			debug::drawSphere(margin, error, out);

			if (drawCore)
			{
				out << GEOMETRY_CORE_COLOR;
				debug::drawSphere(0, error, out);
			}

			break;
		}

		case PxConvexCore::eSEGMENT:
		{
			const PxConvexCore::Segment& core = convex.getCore<PxConvexCore::Segment>();
			const PxReal error = 0.001f;

			out << pose;
			out << GEOMETRY_COLOR;
			debug::drawCapsule(core.length, margin, error, out);

			if (drawCore)
			{
				out << GEOMETRY_CORE_COLOR;
				debug::drawCapsule(core.length, 0, error, out);
			}

			break;
		}

		case PxConvexCore::eBOX:
		{
			const PxConvexCore::Box& core = convex.getCore<PxConvexCore::Box>();
			const PxReal error = 0.001f;

			out << pose;
			out << GEOMETRY_COLOR;
			debug::drawBox(core.extents, margin, error, out);

			if (drawCore)
			{
				out << GEOMETRY_CORE_COLOR;
				debug::drawBox(core.extents, 0, error, out);
			}

			break;
		}

		case PxConvexCore::eELLIPSOID:
		{
			const PxConvexCore::Ellipsoid& core = convex.getCore<PxConvexCore::Ellipsoid>();
			const PxReal error = 0.001f;

			out << pose;
			out << GEOMETRY_COLOR;
			debug::drawEllipsoid(core.radii, margin, error, out);

			if (drawCore)
			{
				out << GEOMETRY_CORE_COLOR;
				debug::drawEllipsoid(core.radii, 0, error, out);
			}

			break;
		}

		case PxConvexCore::eCYLINDER:
		{
			const PxConvexCore::Cylinder& core = convex.getCore<PxConvexCore::Cylinder>();
			const PxReal height = core.height;
			const PxReal radius = core.radius;
			const PxReal error = 0.001f;

			out << pose;
			out << GEOMETRY_COLOR;
			debug::drawCylinder(height, radius, margin, error, out);

			if (drawCore)
			{
				out << GEOMETRY_CORE_COLOR;
				debug::drawCylinder(height, radius, 0, error, out);
			}

			break;
		}

		case PxConvexCore::eCONE:
		{
			const PxConvexCore::Cone& core = convex.getCore<PxConvexCore::Cone>();
			const PxReal height = core.height;
			const PxReal radius = core.radius;
			const PxReal error = 0.001f;

			out << pose;
			out << GEOMETRY_COLOR;
			debug::drawCone(height, radius, margin, error, out);

			if (drawCore)
			{
				out << GEOMETRY_CORE_COLOR;
				debug::drawCone(height, radius, 0, error, out);
			}

			break;
		}

		default:
		{
			out << pose;
			out << GEOMETRY_COLOR;
			out << PxDebugBox(Gu::computeBounds(convex, PxTransform(PxIdentity)));
			break;
		}
	}
}

PX_PHYSX_COMMON_API bool Gu::makeConvexShape(const PxGeometry& geom, const PxTransform& pose, ConvexShape& convex)
{
	convex.coreType = Gu::ConvexCore::Type::Enum(-1);
	convex.pose = pose;

	switch (geom.getType())
	{
		case PxGeometryType::eCONVEXCORE:
		{
			const PxConvexCoreGeometry& g = static_cast<const PxConvexCoreGeometry&>(geom);
			convex.coreType = Gu::ConvexCore::Type::Enum(g.getCoreType());
			PxMemCopy(convex.coreData, g.getCoreData(), PxConvexCoreGeometry::MAX_CORE_SIZE);
			convex.margin = g.getMargin();
			return true;
		}
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& g = static_cast<const PxSphereGeometry&>(geom);
			convex.coreType = Gu::ConvexCore::Type::ePOINT;
			convex.margin = g.radius;
			return true;
		}
		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& g = static_cast<const PxCapsuleGeometry&>(geom);
			convex.coreType = Gu::ConvexCore::Type::eSEGMENT;
			Gu::ConvexCore::SegmentCore& core = *reinterpret_cast<Gu::ConvexCore::SegmentCore*>(convex.coreData);
			core.length = g.halfHeight * 2.0f;
			convex.margin = g.radius;
			return true;
		}
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& g = static_cast<const PxBoxGeometry&>(geom);
			convex.coreType = Gu::ConvexCore::Type::eBOX;
			Gu::ConvexCore::BoxCore& core = *reinterpret_cast<Gu::ConvexCore::BoxCore*>(convex.coreData);
			core.extents = g.halfExtents * 2.0f;
			convex.margin = 0;
			return true;
		}
		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& g = static_cast<const PxConvexMeshGeometry&>(geom);
			convex.coreType = Gu::ConvexCore::Type::ePOINTS;
			Gu::ConvexCore::PointsCore& core = *reinterpret_cast<Gu::ConvexCore::PointsCore*>(convex.coreData);
			core.points = g.convexMesh->getVertices();
			core.numPoints = PxU8(g.convexMesh->getNbVertices());
			core.stride = sizeof(PxVec3);
			core.S = g.scale.scale;
			core.R = g.scale.rotation;
			convex.margin = 0;
			return true;
		}

		default:
			break;
	}

	return false;
}

