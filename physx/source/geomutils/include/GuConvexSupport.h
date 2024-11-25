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

#ifndef GU_CONVEX_SUPPORT_H
#define GU_CONVEX_SUPPORT_H

#include "foundation/PxMath.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxMat33.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMathUtils.h"
#include "GuRefGjkEpa.h"

namespace physx
{
	namespace Gu
	{
		// some helpers

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// select up to 4 points forming the biggest polygon
		PxU32 reducePolygon(const void* points, PxU32 count, PxU32 stride, const PxVec3& normal,
			PxU32 inds[4], bool keep1st = false);

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// rotate points around so that direction from center to points[0]
		// matches the direction from center to ref
		void rotatePoints(const PxVec3& center, const PxVec3& ref, const PxVec3& normal, PxVec3* points, PxU32 count);

		// convex cores
		// every core provides 2 functions: localSupport - used by GJK-EPA, and
		// contactFace - that returns up to 4 points on of the shape's contacting face to generate
		// multi-point contact from. adding a new core type implies authoring these 2 functions.

		namespace ConvexCore
		{
			struct Type
			{
				enum Enum
				{
					ePOINT,
					eSEGMENT,
					eBOX,
					eELLIPSOID,
					eCYLINDER,
					eCONE,

					// Internal use
					ePOINTS,

					eCOUNT
				};
			};

			static const PxU32 MAX_CORE_SIZE = sizeof(PxReal) * 10;
			static const PxU32 MAX_FACE_POINTS = 4;

			// Return the farthest point of this core type in a given direction
			template <Type::Enum T> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport(const PxVec3& dir, const void* data);

			// Return up to MAX_FACE_POINTS points of this core type face in a given direction
			template <Type::Enum T> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace(const PxVec3& dir, const PxVec3& point, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS]);

			/////////////////////////////////////////////////////////
			// Point Core
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::ePOINT>(const PxVec3&, const void*)
			{
				return PxVec3(0);
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::ePOINT>(const PxVec3&, const PxVec3&, const void*, PxVec3&, PxVec3*)
			{
				return 0;
			}
			/////////////////////////////////////////////////////////

			/////////////////////////////////////////////////////////
			// Segment Core
			struct SegmentCore { PxReal length; };
			PX_COMPILE_TIME_ASSERT(sizeof(SegmentCore) <= MAX_CORE_SIZE);
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::eSEGMENT>(const PxVec3& dir, const void* data)
			{
				const SegmentCore& core = *reinterpret_cast<const SegmentCore*>(data);
				return PxVec3(PxSign(dir.x) * core.length * 0.5f, 0, 0);
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::eSEGMENT>(const PxVec3& dir, const PxVec3&, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS])
			{
				PxU32 numPoints = 0;
				const SegmentCore& core = *reinterpret_cast<const SegmentCore*>(data);
				const PxVec3 d = dir.getNormalized();
				const PxReal dEps2 = 0.14f; // ~cos(90 - 8)
				facePoints[0] = facePoints[1] = facePoints[2] = facePoints[3] = faceNormal = PxVec3(0);
				if (PxAbs(d.x) < dEps2)
				{
					faceNormal = PxVec3(0, d.y, d.z).getNormalized();
					facePoints[0] = PxVec3(core.length * 0.5f, 0, 0);
					facePoints[1] = PxVec3(-core.length * 0.5f, 0, 0);
					numPoints = 2;
				}
				return numPoints;
			}
			/////////////////////////////////////////////////////////

			/////////////////////////////////////////////////////////
			// Box Core
			struct BoxCore { PxVec3 extents; };
			PX_COMPILE_TIME_ASSERT(sizeof(BoxCore) <= MAX_CORE_SIZE);
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::eBOX>(const PxVec3& dir, const void* data)
			{
				const BoxCore& core = *reinterpret_cast<const BoxCore*>(data);
				return PxVec3(PxSign(dir.x) * core.extents.x * 0.5f,
							  PxSign(dir.y) * core.extents.y * 0.5f,
							  PxSign(dir.z) * core.extents.z * 0.5f);
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::eBOX>(const PxVec3& dir, const PxVec3&, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS])
			{
				PxU32 numPoints = 0;
				const BoxCore& core = *reinterpret_cast<const BoxCore*>(data);
				const PxVec3 d = dir.getNormalized();
				const PxReal dEps1 = 0.99f, dEps2 = 0.14f; // ~cos(8), ~cos(90 - 8)
				const PxReal eps = FLT_EPSILON;
				facePoints[0] = facePoints[1] = facePoints[2] = facePoints[3] = faceNormal = PxVec3(0);
				for (PxU32 axis0 = 0; axis0 < 3 && numPoints == 0; ++axis0)
				{
					const PxU32 axis1 = (axis0 + 1) % 3, axis2 = (axis0 + 2) % 3;
					if (PxAbs(d[axis0]) > dEps1 && (core.extents[axis1] > eps || core.extents[axis2] > eps))
					{
						faceNormal[axis0] = PxSign(d[axis0]);

						facePoints[0][axis0] = PxSign(d[axis0]) * core.extents[axis0] * 0.5f;
						facePoints[0][axis1] = core.extents[axis1] * 0.5f;
						facePoints[0][axis2] = core.extents[axis2] * 0.5f;

						facePoints[1][axis0] = PxSign(d[axis0]) * core.extents[axis0] * 0.5f;
						facePoints[1][axis1] = -core.extents[axis1] * 0.5f;
						facePoints[1][axis2] = core.extents[axis2] * 0.5f;

						facePoints[2][axis0] = PxSign(d[axis0]) * core.extents[axis0] * 0.5f;
						facePoints[2][axis1] = -core.extents[axis1] * 0.5f;
						facePoints[2][axis2] = -core.extents[axis2] * 0.5f;

						facePoints[3][axis0] = PxSign(d[axis0]) * core.extents[axis0] * 0.5f;
						facePoints[3][axis1] = core.extents[axis1] * 0.5f;
						facePoints[3][axis2] = -core.extents[axis2] * 0.5f;

						numPoints = 4;
					}
					else if (PxAbs(d[axis0]) < dEps2 && core.extents[axis0] > eps &&
						(PxAbs(d[axis1]) < dEps1 || core.extents[axis2] < eps) &&
						(PxAbs(d[axis2]) < dEps1 || core.extents[axis1] < eps))
					{
						faceNormal[axis1] = d[axis1];
						faceNormal[axis2] = d[axis2];
						faceNormal.normalize();

						facePoints[0][axis0] = core.extents[axis0] * 0.5f;
						facePoints[0][axis1] = PxSign(d[axis1]) * core.extents[axis1] * 0.5f;
						facePoints[0][axis2] = PxSign(d[axis2]) * core.extents[axis2] * 0.5f;

						facePoints[1][axis0] = -core.extents[axis0] * 0.5f;
						facePoints[1][axis1] = PxSign(d[axis1]) * core.extents[axis1] * 0.5f;
						facePoints[1][axis2] = PxSign(d[axis2]) * core.extents[axis2] * 0.5f;

						numPoints = 2;
					}
				}
				return numPoints;
			}
			/////////////////////////////////////////////////////////

			/////////////////////////////////////////////////////////
			// Ellipsoid Core
			struct EllipsoidCore { PxVec3 radii; };
			PX_COMPILE_TIME_ASSERT(sizeof(EllipsoidCore) <= MAX_CORE_SIZE);
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::eELLIPSOID>(const PxVec3& dir, const void* data)
			{
				const EllipsoidCore& core = *reinterpret_cast<const EllipsoidCore*>(data);
				const PxMat33 xform = PxMat33::createDiagonal(core.radii);
				const PxVec3 dir1 = xform.transformTranspose(dir).getNormalized();
				return xform.transform(dir1);
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::eELLIPSOID>(const PxVec3& dir, const PxVec3&, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS])
			{
				PxU32 numPoints = 0;
				const EllipsoidCore& core = *reinterpret_cast<const EllipsoidCore*>(data);
				const PxVec3 d = dir.getNormalized();
				const PxReal dEps1 = 0.99f, dEps2 = 0.14f; // ~cos(8), ~cos(90 - 8)
				const PxReal eps = FLT_EPSILON;
				facePoints[0] = facePoints[1] = facePoints[2] = facePoints[3] = faceNormal = PxVec3(0);
				for (PxU32 axis0 = 0; axis0 < 3 && numPoints == 0; ++axis0)
				{
					const PxU32 axis1 = (axis0 + 1) % 3, axis2 = (axis0 + 2) % 3;
					if (PxAbs(d[axis0]) > dEps1 && core.radii[axis0] < eps && core.radii[axis1] > eps && core.radii[axis2] > eps)
					{
						faceNormal[axis0] = PxSign(d[axis0]);
						facePoints[0][axis1] = core.radii[axis1];
						facePoints[1][axis1] = -core.radii[axis1];
						facePoints[2][axis2] = core.radii[axis2];
						facePoints[3][axis2] = -core.radii[axis2];
						numPoints = 4;
					}
					else if (PxAbs(d[axis0]) < dEps2 && core.radii[axis0] > eps && core.radii[axis1] < eps && core.radii[axis2] < eps)
					{
						faceNormal[axis1] = d[axis1];
						faceNormal[axis2] = d[axis2];
						faceNormal.normalize();
						facePoints[0][axis0] = core.radii[axis0];
						facePoints[1][axis0] = -core.radii[axis0];
						numPoints = 2;
					}
				}
				return numPoints;
			}
			/////////////////////////////////////////////////////////

			/////////////////////////////////////////////////////////
			// Cylinder Core
			struct CylinderCore { PxReal height, radius; };
			PX_COMPILE_TIME_ASSERT(sizeof(CylinderCore) <= MAX_CORE_SIZE);
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::eCYLINDER>(const PxVec3& dir, const void* data)
			{
				const CylinderCore& core = *reinterpret_cast<const CylinderCore*>(data);
				const PxReal h = core.height * 0.5f;
				const PxReal r = core.radius;
				const PxVec3 d = dir.getNormalized();
				const PxReal eps = 1e-5f;
				if (PxAbs(d.y) < eps && PxAbs(d.z) < eps) return PxVec3(PxSign(d.x) * h, r, 0);
				return PxVec3(PxSign(d.x) * h, 0, 0) + PxVec3(0, d.y, d.z).getNormalized() * r;
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::eCYLINDER>(const PxVec3& dir, const PxVec3& ref, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS])
			{
				PxU32 numPoints = 0;
				const CylinderCore& core = *reinterpret_cast<const CylinderCore*>(data);
				const PxVec3 d = dir.getNormalized();
				const PxReal dEps1 = 0.99f, dEps2 = 0.14f; // ~cos(8), ~cos(90 - 8)
				if (PxAbs(d.x) > dEps1 && core.radius > 0)
				{
					faceNormal = PxVec3(PxSign(d.x), 0, 0);
					facePoints[0] = PxVec3(PxSign(d.x) * core.height * 0.5f, core.radius, 0);
					facePoints[1] = PxVec3(PxSign(d.x) * core.height * 0.5f, 0, core.radius);
					facePoints[2] = PxVec3(PxSign(d.x) * core.height * 0.5f, -core.radius, 0);
					facePoints[3] = PxVec3(PxSign(d.x) * core.height * 0.5f, 0, -core.radius);
					numPoints = 4;
					rotatePoints(PxVec3(PxSign(d.x) * core.height * 0.5f, 0, 0), ref, faceNormal, facePoints, numPoints);
				}
				else if (PxAbs(d.x) < dEps2 && core.height > 0)
				{
					const PxVec3 dr = faceNormal = PxVec3(0, d.y, d.z).getNormalized();
					facePoints[0] = PxVec3(core.height * 0.5f, 0, 0) + dr * core.radius;
					facePoints[1] = PxVec3(-core.height * 0.5f, 0, 0) + dr * core.radius;
					numPoints = 2;
				}
				return numPoints;
			}
			/////////////////////////////////////////////////////////

			/////////////////////////////////////////////////////////
			// Cone Core
			struct ConeCore { PxReal height, radius; };
			PX_COMPILE_TIME_ASSERT(sizeof(ConeCore) <= MAX_CORE_SIZE);
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::eCONE>(const PxVec3& dir, const void* data)
			{
				const ConeCore& core = *reinterpret_cast<const ConeCore*>(data);
				const PxReal h = core.height;
				const PxReal r = core.radius;
				const PxReal sinA = r / PxSqrt(h * h + r * r);
				const PxReal halfH = h * 0.5f;
				const PxVec3 d = dir.getNormalized();
				const PxReal eps = 1e-5f;
				if (d.x > sinA) return PxVec3(halfH, 0, 0);
				if (PxAbs(d.y) < eps && PxAbs(d.z) < eps) return PxVec3(-halfH, r, 0);
				return PxVec3(-halfH, 0, 0) + PxVec3(0, d.y, d.z).getNormalized() * r;
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::eCONE>(const PxVec3& dir, const PxVec3& ref, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS])
			{
				PxU32 numPoints = 0;
				const ConeCore& core = *reinterpret_cast<const ConeCore*>(data);
				if (core.height + core.radius > 0)
				{
					const PxVec3 d = dir.getNormalized();
					const PxReal dEps1 = 0.99f, dEps2 = 0.14f; // ~cos(8), ~cos(90 - 8)
					const PxReal eps = 1e-5f;
					if ((d.x < -dEps1 || (d.x > dEps1 && core.height < eps)) && core.radius > 0)
					{
						faceNormal = PxVec3(PxSign(d.x), 0, 0);
						facePoints[0] = PxVec3(PxSign(d.x) * core.height * 0.5f, core.radius, 0);
						facePoints[1] = PxVec3(PxSign(d.x) * core.height * 0.5f, 0, core.radius);
						facePoints[2] = PxVec3(PxSign(d.x) * core.height * 0.5f, -core.radius, 0);
						facePoints[3] = PxVec3(PxSign(d.x) * core.height * 0.5f, 0, -core.radius);
						numPoints = 4;
						rotatePoints(PxVec3(PxSign(d.x) * core.height * 0.5f, 0, 0), ref, faceNormal, facePoints, numPoints);
					}
					else if (core.height > 0)
					{
						const PxReal sinA = core.radius / PxSqrt(core.height * core.height + core.radius * core.radius);
						const PxReal dX = d.x * PxSqrt(1.0f - sinA * sinA) - PxSqrt(1.0f - d.x * d.x) * sinA;
						if (PxAbs(dX) < dEps2)
						{
							const PxVec3 dr = PxVec3(0, d.y, d.z).getNormalized();
							faceNormal = PxVec3(core.radius / core.height, dr.y, dr.z).getNormalized();
							facePoints[0] = PxVec3(core.height * 0.5f, 0, 0);
							facePoints[1] = PxVec3(-core.height * 0.5f, 0, 0) + dr * core.radius;
							numPoints = 2;
						}
					}
				}
				return numPoints;
			}
			/////////////////////////////////////////////////////////

			/////////////////////////////////////////////////////////
			// Points Core
			struct PointsCore { const void* points; PxVec3 S; PxQuat R; PxU8 numPoints, stride, pad[2]; };
			PX_COMPILE_TIME_ASSERT(sizeof(PointsCore) <= MAX_CORE_SIZE);
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 localSupport<Type::ePOINTS>(const PxVec3& dir, const void* data)
			{
				PointsCore core; memcpy(reinterpret_cast<void*>(&core), data, sizeof(core));
				if (core.points == NULL || core.numPoints == 0)
					return PxVec3(0);
				const PxU8* points = reinterpret_cast<const PxU8*>(core.points);
				const PxVec3 d = core.R.rotateInv(core.S.multiply(core.R.rotate(dir)));
				if (d.magnitude() < FLT_EPSILON)
					return PxVec3(0);
				PxU32 index = 0;
				PxReal maxDot = -FLT_MAX;
				for (PxU32 i = 0; i < core.numPoints; ++i)
				{
					const PxVec3& p = *reinterpret_cast<const PxVec3*>(points + i * core.stride);
					PxReal dot = d.dot(p);
					if (dot > maxDot)
					{
						maxDot = dot;
						index = i;
					}
				}
				const PxVec3& point = *reinterpret_cast<const PxVec3*>(points + index * core.stride);
				return core.R.rotateInv(core.S.multiply(core.R.rotate(point)));
			}
			template <> PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace<Type::ePOINTS>(const PxVec3& dir, const PxVec3&, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS])
			{
				PxU32 numPoints = 0;

				PointsCore core; memcpy(reinterpret_cast<void*>(&core), data, sizeof(core));
				if (core.points == NULL || core.numPoints == 0)
					return 0;

				#define getPoint(index) (core.R.rotateInv(core.S.multiply(core.R.rotate\
					(*reinterpret_cast<const PxVec3*>(reinterpret_cast<const PxU8*>(core\
					.points) + index * core.stride)))))

				const PxReal dEps1 = 0.99f; // ~cos(8)

				PxU32 index[3] = { 0xffffffff, 0xffffffff, 0xffffffff };
				PxReal maxDot[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
				for (PxU32 i = 0; i < core.numPoints; ++i)
				{
					const PxVec3 p = getPoint(i);
					PxReal dot = dir.dot(p);
					PxU32 ind = i;
					for (PxU32 j = 0; j < 3; ++j)
						if (dot > maxDot[j])
						{
							PxSwap(maxDot[j], dot);
							PxSwap(index[j], ind);
						}
				}

				if (index[1] == 0xffffffff)
					return 0;

				const PxVec3 p0 = getPoint(index[0]);

				if (index[2] == 0xffffffff)
				{
					const PxVec3 p1 = getPoint(index[1]);
					const PxVec3 n = dir.cross(p1 - p0).cross(p1 - p0).getNormalized();
					PxReal dot = dir.dot(n);
					if (dot > dEps1)
						faceNormal = n;
					else
						return 0;
				}
				else
				{
					const PxVec3 p1 = getPoint(index[1]), p2 = getPoint(index[2]);
					const PxVec3 n0 = (p1 - p0).cross(p2 - p0).getNormalized();
					PxReal dot0 = dir.dot(n0);
					if (dot0 < 0)
						dot0 = -dot0;
					if (dot0 > dEps1)
						faceNormal = n0;
					else
					{
						const PxVec3 n1 = (p1 - p0).cross(dir).cross(p1 - p0).getNormalized();
						PxReal dot1 = dir.dot(n1);
						if (dot1 > dEps1)
							faceNormal = n1;
						else
						{
							const PxVec3 n2 = (p2 - p0).cross(dir).cross(p2 - p0).getNormalized();
							PxReal dot2 = dir.dot(n2);
							if (dot2 > dEps1)
								faceNormal = n2;
							else
								return 0;
						}
					}
				}

				PxReal maxP = faceNormal.dot(p0);

				for (PxU32 i = 0; i < core.numPoints; ++i)
				{
					const PxVec3 p = getPoint(i);
					PxReal pP = faceNormal.dot(p);
					if (PxAbs(pP - maxP) < 1e-4f)
					{
						PxVec3 newFacePoints[MAX_FACE_POINTS + 1] = { p };
						memcpy(reinterpret_cast<void*>(&newFacePoints[1]), facePoints, sizeof(PxVec3) * numPoints);
						for (PxU32 j = 0; j < numPoints; ++j)
							if (newFacePoints[0].dot(dir) < newFacePoints[j + 1].dot(dir))
								PxSwap(newFacePoints[0], newFacePoints[j + 1]);

						PxU32 inds[4];
						numPoints = reducePolygon(newFacePoints, numPoints + 1, sizeof(PxVec3), dir, inds, true);
						for (PxU32 j = 0; j < numPoints; ++j)
							facePoints[j] = newFacePoints[inds[j]];
					}
				}

				return numPoints >= 2 ? numPoints : 0;
				#undef getPoint
			}

			/////////////////////////////////////////////////////////
			// Core functions by type
			typedef PxVec3(*LocalSupportFn)(const PxVec3& dir, const void* data);
			typedef PxU32(*ContactFaceFn)(const PxVec3& dir, const PxVec3& point, const void* data, PxVec3& faceNormal, PxVec3 facePoints[MAX_FACE_POINTS]);
			PX_CUDA_CALLABLE PX_FORCE_INLINE
			LocalSupportFn localSupport(Type::Enum type)
			{
				switch (type)
				{
				case Type::ePOINT: return localSupport<Type::ePOINT>;
				case Type::eSEGMENT: return localSupport<Type::eSEGMENT>;
				case Type::eBOX: return localSupport<Type::eBOX>;
				case Type::eELLIPSOID: return localSupport<Type::eELLIPSOID>;
				case Type::eCYLINDER: return localSupport<Type::eCYLINDER>;
				case Type::eCONE: return localSupport<Type::eCONE>;
				case Type::ePOINTS: return localSupport<Type::ePOINTS>;
				default: break;
				}
				PX_ASSERT(0);
				return NULL;
			}
			PX_CUDA_CALLABLE PX_FORCE_INLINE
			ContactFaceFn contactFace(Type::Enum type)
			{
				switch (type)
				{
				case Type::ePOINT: return contactFace<Type::ePOINT>;
				case Type::eSEGMENT: return contactFace<Type::eSEGMENT>;
				case Type::eBOX: return contactFace<Type::eBOX>;
				case Type::eELLIPSOID: return contactFace<Type::eELLIPSOID>;
				case Type::eCYLINDER: return contactFace<Type::eCYLINDER>;
				case Type::eCONE: return contactFace<Type::eCONE>;
				case Type::ePOINTS: return contactFace<Type::ePOINTS>;
				default: break;
				}
				PX_ASSERT(0);
				return NULL;
			}
			/////////////////////////////////////////////////////////
		}

		// the struct to pass into generateContacts function
		struct ConvexShape
		{
			ConvexCore::Type::Enum coreType;
			PxU8 coreData[ConvexCore::MAX_CORE_SIZE];
			PxReal margin;
			PxTransform pose;

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			bool isValid() const
			{
				return coreType < ConvexCore::Type::eCOUNT;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 supportLocal(const PxVec3& dir) const
			{
				// Used by RefGjkEpa so doesn't include margin
				return ConvexCore::localSupport(coreType)(dir, coreData);
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxVec3 support(const PxVec3& dir) const
			{
				return pose.transform(supportLocal(pose.rotateInv(dir))) + dir * margin;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 contactFace(const PxVec3& dir, const PxVec3& point, PxVec3& faceNormal, PxVec3 facePoints[ConvexCore::MAX_FACE_POINTS]) const
			{
				PxU32 numPoints = ConvexCore::contactFace(coreType)(pose.rotateInv(dir), pose.transformInv(point), coreData, faceNormal, facePoints);
				PX_ASSERT(numPoints <= ConvexCore::MAX_FACE_POINTS);
				faceNormal = pose.rotate(faceNormal);
				for (PxU32 i = 0; i < numPoints; ++i)
					facePoints[i] = pose.transform(facePoints[i]) + faceNormal * margin;

				return numPoints;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxBounds3 computeBounds() const
			{
				const PxVec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
				const PxVec3 minimum(support(-X).x, support(-Y).y, support(-Z).z);
				const PxVec3 maximum(support(X).x, support(Y).y, support(Z).z);
				return PxBounds3(minimum, maximum);
			}
		};

		// a helper struct to clip contact faces.
		// it takes a direction and a reference point (the output of GJKEPA),
		// and 2 sets of points (returned by contactFace functions). then it
		// computes 2 sets of planes parallel to the given direction and
		// enclosing a corresponding set of points. then it creates a quad
		// perpendicular to the direction and containing the reference point.
		// and finally it clips the quad with the planes from both sets.
		// MAX_CONTACTS vertices of the clipped poingon, forming the biggest
		// poligon are returned.
		struct FaceClipper
		{
			static const PxU32 MAX_CONTACTS = 4;

			PxVec3 faceNormal0, facePoints0[ConvexCore::MAX_FACE_POINTS];
			PxU32 numPoints0;
			PxVec3 faceNormal1, facePoints1[ConvexCore::MAX_FACE_POINTS];
			PxU32 numPoints1;
			PxVec3 contactPoints[MAX_CONTACTS];
			PxReal contactDists[MAX_CONTACTS];
			PxU32 numContacts;

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			FaceClipper()
				:
				numPoints0(0), numPoints1(0), numContacts(0)
			{}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 clip(const PxVec3& axis, const PxVec3& ref)
			{
				if (numPoints0 <= 1 || numPoints1 <= 1)
					return 0;

				PX_ASSERT(numPoints0 >= 2 && numPoints1 >= 2);

				if (numPoints0 == 2 && numPoints1 == 2)
					return clip2x2(axis);

				if (numPoints0 == 2)
					return clip2xN(axis);

				if (numPoints1 == 2)
					return clipNx2(axis);

				return clipNxN(axis, ref);
			}

		private:

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			// clip 2 point poygon by 2 point polygon
			// a special case when both input point sets have only 2 points
			// if we try to clip them usual way we'll most likely end up with
			// 0 of 1 point, and we want to have 2 if we can.
			// it projects the ends of one segment onto the other one
			// and check where they overlap.
			PxU32 clip2x2(const PxVec3& axis)
			{
				const PxReal eps = 1e-5f;
				const PxVec3 a = facePoints0[0], b = facePoints0[1];
				const PxVec3 c = facePoints1[0], d = facePoints1[1];

				// Only care if the segments are parallel
				if (PxAbs((b - a).cross(d - c).dot(axis)) > eps)
					return 0;

				if ((a - c).dot(d - c) > -eps && (a - d).dot(c - d) > -eps) // a projects on cd
				{
					const PxVec3 a1 = c + (d - c) * (d - c).dot(a - c) / (d - c).magnitudeSquared();
					addContact((a + a1) * 0.5f, axis, axis.dot(a - a1));
				}
				if ((b - c).dot(d - c) > -eps && (b - d).dot(c - d) > -eps) // b projects on cd
				{
					const PxVec3 b1 = c + (d - c) * (d - c).dot(b - c) / (d - c).magnitudeSquared();
					addContact((b + b1) * 0.5f, axis, axis.dot(b - b1));
				}
				if ((c - a).dot(b - a) > -eps && (c - b).dot(a - b) > -eps) // c projects on ab
				{
					const PxVec3 c1 = a + (b - a) * (b - a).dot(c - a) / (b - a).magnitudeSquared();
					addContact((c1 + c) * 0.5f, axis, axis.dot(c1 - c));
				}
				if ((d - a).dot(b - a) > -eps && (d - b).dot(a - b) > -eps) // d projects on ab
				{
					const PxVec3 d1 = a + (b - a) * (b - a).dot(d - a) / (b - a).magnitudeSquared();
					addContact((d1 + d) * 0.5f, axis, axis.dot(d1 - d));
				}

				return numContacts;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void makeSegmentPlanes(const PxVec3& axis, const PxVec3* facePoints, PxU32 numPoints, PxVec4d* planes, PxU32& numPlanes)
			{
				PX_UNUSED(numPoints);
				PX_ASSERT(numPoints == 2);
				const PxF64 eps = 1e-5;
				const PxVec3d a = V3_To_V3d(facePoints[0]), b = V3_To_V3d(facePoints[1]);
				const PxVec3d ab = b - a;
				const PxVec3d n0 = ab.cross(V3_To_V3d(axis)).getNormalized();
				const PxVec3d n1 = n0.cross(V3_To_V3d(axis)).getNormalized();
				planes[numPlanes++] = PxVec4d(n0, -n0.dot(a) + eps);
				planes[numPlanes++] = PxVec4d(-n0, n0.dot(a) + eps);
				planes[numPlanes++] = PxVec4d(n1, -n1.dot(b));
				planes[numPlanes++] = PxVec4d(-n1, n1.dot(a));
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void makePlanes(const PxVec3& axis, const PxVec3* facePoints, PxU32 numPoints, PxVec4d* planes, PxU32& numPlanes)
			{
				if (numPoints == 2)
					return makeSegmentPlanes(axis, facePoints, numPoints, planes, numPlanes);

				const PxReal eps = 1e-5f;
				const PxReal eps2 = eps * eps;
				PxU32 start = 0, stop = PxU32(-1);
				while (true)
				{
					const PxVec3& s = facePoints[start];
					for (PxU32 i = 0; i < numPoints; ++i)
					{
						if (i == start)
							continue;
				
						const PxVec3& e = facePoints[i];
						PxVec3 n = (e - s).cross(axis);
						if (n.magnitudeSquared() < eps2)
							continue;

						n.normalizeFast();
				
						bool edge = true;
						for (PxU32 j = 0; j < numPoints; ++j)
						{
							if (j == i || j == start)
								continue;
				
							const PxVec3& v = facePoints[j];
							if ((v - s).dot(n) > eps)
							{
								edge = false;
								break;
							}
						}

						if (edge)
						{
							planes[numPlanes++] = PxVec4d(-V3_To_V3d(n), PxF64(n.dot(s)));

							if (stop == PxU32(-1))
								stop = start;

							start = i;

							break;
						}
					}

					if (stop == PxU32(-1))
						++start;

					if (start == stop)
						break;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void makePlanes(const PxVec3& axis, PxVec4d* clipPlanes, PxU32& numClipPlanes)
			{
				makePlanes(axis, facePoints0, numPoints0, clipPlanes, numClipPlanes);
				makePlanes(axis, facePoints1, numPoints1, clipPlanes, numClipPlanes);
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void makePoly(const PxVec3& axis, const PxVec3& reff, PxVec3d* points, PxU32& numPoints)
			{
				PxVec3 Xf, Yf; PxComputeBasisVectors(axis, Xf, Yf);
				PxVec3d X = V3_To_V3d(Xf), Y = V3_To_V3d(Yf), ref = V3_To_V3d(reff);
				PxF64 minX = DBL_MAX, maxX = -DBL_MAX, minY = DBL_MAX, maxY = -DBL_MAX;
				for (PxU32 i = 0; i < numPoints0; ++i)
				{
					const PxVec3d p = V3_To_V3d(facePoints0[i]);
					const PxF64 pX = p.dot(X), pY = p.dot(Y);
					minX = PxMin(minX, pX); maxX = PxMax(maxX, pX);
					minY = PxMin(minY, pY); maxY = PxMax(maxY, pY);
				}
				for (PxU32 i = 0; i < numPoints1; ++i)
				{
					const PxVec3d p = V3_To_V3d(facePoints1[i]);
					const PxF64 pX = p.dot(X), pY = p.dot(Y);
					minX = PxMin(minX, pX); maxX = PxMax(maxX, pX);
					minY = PxMin(minY, pY); maxY = PxMax(maxY, pY);
				}
				const PxF64 refX = ref.dot(X), refY = ref.dot(Y);
				points[numPoints++] = ref + X * (minX - refX) + Y * (minY - refY);
				points[numPoints++] = ref + X * (maxX - refX) + Y * (minY - refY);
				points[numPoints++] = ref + X * (maxX - refX) + Y * (maxY - refY);
				points[numPoints++] = ref + X * (minX - refX) + Y * (maxY - refY);
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void clipPoly(const PxVec4d& plane, PxVec3d* points, PxU32& numPoints)
			{
				const PxF64 eps = 1e-5;
				PxF64 dist[4 + ConvexCore::MAX_FACE_POINTS * 2];
				for (PxU32 i = 0; i < numPoints; ++i)
					dist[i] = plane.dot(PxVec4d(points[i], 1));

				PxU32 newNumPoints = 0;
				PxVec3d newPoints[4 + ConvexCore::MAX_FACE_POINTS * 2];
				for (PxU32 i = 0; i < numPoints; ++i)
				{
					const PxU32 i0 = i, i1 = (i + 1) % numPoints;
					const PxF64 d0 = dist[i0], d1 = dist[i1];
					const PxVec3d p0 = points[i0], p1 = points[i1];
					if (d0 >= 0)
					{
						newPoints[newNumPoints++] = p0;
						if (d1 < 0 && d0 - d1 > eps)
							newPoints[newNumPoints++] = p0 + (p1 - p0) * (d0 / (d0 - d1));
					}
					else if (d1 >= 0 && d1 - d0 > eps)
						newPoints[newNumPoints++] = p0 + (p1 - p0) * (d0 / (d0 - d1));
				}

				numPoints = newNumPoints;
				for (PxU32 i = 0; i < numPoints; ++i)
					points[i] = newPoints[i];
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void addContact(const PxVec3& point, const PxVec3& normal, PxReal dist)
			{
				PxVec3 points[MAX_CONTACTS + 1] = { point };
				PxReal dists[MAX_CONTACTS + 1] = { dist };
				for (PxU32 i = 0; i < numContacts; ++i)
				{
					points[i + 1] = contactPoints[i];
					dists[i + 1] = contactDists[i];
				}
				for (PxU32 i = 0; i < numContacts; ++i)
					if (dists[i] > dists[i + 1])
					{
						PxSwap(points[i], points[i + 1]);
						PxSwap(dists[i], dists[i + 1]);
					}
					else break;

				PxU32 inds[4];
				PxU32 numPoints = reducePolygon(points, numContacts + 1, sizeof(PxVec3), normal, inds, true);
				for (PxU32 i = 0; i < numPoints; ++i)
				{
					contactPoints[i] = points[inds[i]];
					contactDists[i] = dists[inds[i]];
				}

				numContacts = numPoints;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void writePoints(const PxVec3& axis, const PxVec3d* polyPoints, PxU32 numPolyPoints)
			{
				const PxVec4 plane0(faceNormal0, -faceNormal0.dot(facePoints0[0]));
				const PxVec4 plane1(faceNormal1, -faceNormal1.dot(facePoints1[0]));
				for (PxU32 i = 0; i < numPolyPoints; ++i)
				{
					const PxVec3 p = V3d_To_V3(polyPoints[i]);
					const PxVec3 p0 = p - axis * plane0.dot(PxVec4(p, 1)) / axis.dot(plane0.getXYZ());
					const PxVec3 p1 = p - axis * plane1.dot(PxVec4(p, 1)) / axis.dot(plane1.getXYZ());
					addContact((p0 + p1) * 0.5f, axis, axis.dot(p0 - p1));
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			// clip N point poygon by N point polygon
			PxU32 clipNxN(const PxVec3& axis, const PxVec3& ref)
			{
				PxU32 numClipPlanes = 0;
				PxVec4d clipPlanes[ConvexCore::MAX_FACE_POINTS * 2];
				makePlanes(axis, clipPlanes, numClipPlanes);

				PxU32 numPolyPoints = 0;
				PxVec3d polyPoints[4 + ConvexCore::MAX_FACE_POINTS * 2];
				makePoly(axis, ref, polyPoints, numPolyPoints);

				for (PxU32 i = 0; i < numClipPlanes; ++i)
					clipPoly(clipPlanes[i], polyPoints, numPolyPoints);

				writePoints(axis, polyPoints, numPolyPoints);

				return numContacts;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			// clip 2 point segment by N point polygon
			PxU32 clip2xN(const PxVec3& axis)
			{
				PxU32 numClipPlanes = 0;
				PxVec4d clipPlanes[ConvexCore::MAX_FACE_POINTS];
				makePlanes(axis, facePoints1, numPoints1, clipPlanes, numClipPlanes);

				PxU32 numPolyPoints = 2;
				PxVec3d polyPoints[2 + ConvexCore::MAX_FACE_POINTS] =
					{ V3_To_V3d(facePoints0[0]), V3_To_V3d(facePoints0[1]) };

				for (PxU32 i = 0; i < numClipPlanes; ++i)
					clipPoly(clipPlanes[i], polyPoints, numPolyPoints);

				writePoints(axis, polyPoints, numPolyPoints);

				return numContacts;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			// clip N point polygon by 2 point segment
			PxU32 clipNx2(const PxVec3& axis)
			{
				PxU32 numClipPlanes = 0;
				PxVec4d clipPlanes[ConvexCore::MAX_FACE_POINTS];
				makePlanes(axis, facePoints0, numPoints0, clipPlanes, numClipPlanes);

				PxU32 numPolyPoints = 2;
				PxVec3d polyPoints[2 + ConvexCore::MAX_FACE_POINTS] =
					{ V3_To_V3d(facePoints1[0]), V3_To_V3d(facePoints1[1]) };

				for (PxU32 i = 0; i < numClipPlanes; ++i)
					clipPoly(clipPlanes[i], polyPoints, numPolyPoints);

				writePoints(axis, polyPoints, numPolyPoints);

				return numContacts;
			}
		};

		// contact patches generation
		// it takes a point and distributes it among up to 16
		// contact patches. each patch has up to 4 points. if
		// a patch is full, 4 points forming a biggest quad are
		// selected, while ensuring the the 1st point is the 
		// dippest one. if we already have 16 patches, it checks
		// if there are 2 patches that are closer to each other
		// (by their normals) that the new patch to any of the
		// existing patches. if yes, it drops the patch with the
		// smaller depth (1st point). if no, the new patch is
		// dropped.
		struct Contact
		{
			static const PxU32 MAX_PATCHES = 16;
			static const PxU32 MAX_PATCH_POINTS = 4;

			struct Point
			{
				PxVec3 p; PxReal d;

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				static Point make(const PxVec3& p, PxReal d)
					{ Point pt; pt.p = p; pt.d = d; return pt; }
			};

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			Contact()
				:
				mNumPatches(0), mNumPoints(0)
			{}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 numPatches() const
			{
				return mNumPatches;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 numPoints() const
			{
				return mNumPoints;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			PxU32 numPatchPoints(PxU32 patchIndex) const
			{
				return mPatches[patchIndex].numPoints;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			const PxVec3& patchNormal(PxU32 patchIndex) const
			{
				return mPatches[patchIndex].normal;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			const Point& patchPoint(PxU32 patchIndex, PxU32 pointIndex) const
			{
				return mPoints[patchIndex * MAX_PATCH_POINTS + pointIndex];
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void addPoint(const PxVec3& position, const PxVec3& normal, PxReal depth)
			{
				for (PxU32 i = 0; i < mNumPatches; ++i)
				{
					Patch& patch = mPatches[i];
					if (patch.normal.dot(normal) >= SAME_NORMAL)
					{
						addPatchPoint(i, Point::make(position, depth));
						return;
					}
				}

				addPatch(normal, Point::make(position, depth));
			}

		protected:

			PxReal SAME_NORMAL = 0.999f; // see PXC_SAME_NORMAL

			struct Patch
			{
				PxVec3 normal;
				PxU32 numPoints;
			};

			Patch mPatches[MAX_PATCHES];
			Point mPoints[MAX_PATCHES * MAX_PATCH_POINTS];
			PxU32 mNumPatches, mNumPoints;

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void addPatch(const PxVec3& normal, const Point& point)
			{
				if (mNumPatches < MAX_PATCHES)
				{
					PxU32 patchIndex = mNumPatches++;
					Patch& patch = mPatches[patchIndex];
					patch.normal = normal;
					patch.numPoints = 1;
					mPoints[patchIndex * MAX_PATCH_POINTS] = point;
					++mNumPoints;
					return;
				}

				PxReal maxDot0 = -FLT_MAX;
				for (PxU32 i = 0; i < mNumPatches; ++i)
					maxDot0 = PxMax(maxDot0, normal.dot(mPatches[i].normal));

				PxReal maxDot1 = -FLT_MAX;
				PxU32 pi0 = 0xffffffff, pi1 = 0xffffffff;
				for (PxU32 i = 0; i < mNumPatches; ++i)
				{
					const PxVec3& n0 = mPatches[i].normal;
					for (PxU32 j = i + 1; j < mNumPatches; ++j)
					{
						const PxVec3& n1 = mPatches[j].normal;
						PxReal dot = n0.dot(n1);
						if (dot > maxDot1)
						{
							maxDot1 = dot;
							pi0 = i; pi1 = j;
						}
					}
				}
				PX_ASSERT(pi0 != pi1);

				if (maxDot0 > maxDot1)
					return;

				PxReal depth0 = mPoints[pi0 * MAX_PATCH_POINTS].d;
				for (PxU32 i = 1; i < mPatches[pi0].numPoints; ++i)
					depth0 = PxMin(depth0, mPoints[pi0 * MAX_PATCH_POINTS + i].d);

				PxReal depth1 = mPoints[pi1 * MAX_PATCH_POINTS].d;
				for (PxU32 i = 1; i < mPatches[pi1].numPoints; ++i)
					depth1 = PxMin(depth1, mPoints[pi1 * MAX_PATCH_POINTS + i].d);

				PxU32 patchIndex = depth0 > depth1 ? pi0 : pi1;
				Patch& patch = mPatches[patchIndex];
				mNumPoints -= patch.numPoints;
				patch.normal = normal;
				patch.numPoints = 1;
				mPoints[patchIndex * MAX_PATCH_POINTS] = point;
				++mNumPoints;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE
			void addPatchPoint(PxU32 patchIndex, const Point& point)
			{
				Patch& patch = mPatches[patchIndex];

				Point points[MAX_PATCH_POINTS + 1];
				points[0] = point;
				for (PxU32 i = 0; i < patch.numPoints; ++i)
				{
					points[i + 1] = mPoints[patchIndex * MAX_PATCH_POINTS + i];
				}
				for (PxU32 i = 0; i < patch.numPoints; ++i)
				{
					if (points[i].d > points[i + 1].d)
						PxSwap(points[i], points[i + 1]);
					else break;
				}

				PxU32 inds[4];
				PxU32 numPoints = reducePolygon(&points[0].p, patch.numPoints + 1, sizeof(Point), patch.normal, inds, true);
				for (PxU32 i = 0; i < numPoints; ++i)
					mPoints[patchIndex * MAX_PATCH_POINTS + i] = points[inds[i]];

				mNumPoints -= patch.numPoints;
				patch.numPoints = numPoints;
				mNumPoints += patch.numPoints;
			}
		};

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// select 4 points forming the largest quad
		// it can keep the 1st point if asked
		PxU32 reducePolygon(const void* points, PxU32 count, PxU32 stride, const PxVec3& normal, PxU32 inds[4], bool keep1st)
		{
			const PxReal eps = FLT_EPSILON;
			using PxU8Ptr = PxU8*;
			using PxVec3Ptr = PxVec3*;
			#define getPoint(index) (*PxVec3Ptr(PxU8Ptr(points) + stride * index))
			PxU32 pi0 = 0xffffffff, pi1 = 0xffffffff;
			if (keep1st)
			{
				pi0 = 0;
				PxReal maxDist = -FLT_MAX;
				const PxVec3& p0 = getPoint(pi0);
				for (PxU32 j = 0; j < count; ++j)
				{
					const PxVec3& p1 = getPoint(j);
					PxReal dist = (p1 - p0).magnitudeSquared();
					if (dist > maxDist + eps)
					{
						maxDist = dist;
						pi1 = j;
					}
				}
			}
			else
			{
				PxReal maxDist = -FLT_MAX;
				for (PxU32 i = 0; i < count; ++i)
				{
					const PxVec3& p0 = getPoint(i);
					for (PxU32 j = 0; j < count; ++j)
					{
						const PxVec3& p1 = getPoint(j);
						PxReal dist = (p1 - p0).magnitudeSquared();
						if (dist > maxDist + eps)
						{
							maxDist = dist;
							pi0 = i; pi1 = j;
						}
					}
				}
			}

			if (pi0 == 0xffffffff || pi1 == 0xffffffff)
				return 0;

			if (pi0 == pi1)
			{
				inds[0] = pi0;
				return 1;
			}

			PxU32 pi2 = 0xffffffff, pi3 = 0xffffffff;
			{
				const PxVec3& p0 = getPoint(pi0);
				const PxVec3& p1 = getPoint(pi1);
				const PxVec3 n01 = normal.cross(p1 - p0);
				PxReal minDist = 0, maxDist = 0;
				for (PxU32 i = 0; i < count; ++i)
				{
					PxReal d = n01.dot(getPoint(i) - p0);
					if (d < minDist - eps)
					{
						pi2 = i;
						minDist = d;
					}
					if (d > maxDist + eps)
					{
						pi3 = i;
						maxDist = d;
					}
				}
			}

			PxU32 numPoints = 0;
			inds[numPoints++] = pi0;
			inds[numPoints++] = pi1;
			if (pi2 != 0xffffffff && pi2 != pi0 && pi2 != pi1)
				inds[numPoints++] = pi2;
			if (pi3 != 0xffffffff && pi3 != pi0 && pi3 != pi1 && pi3 != pi2)
				inds[numPoints++] = pi3;

			return numPoints;
			#undef getPoint
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// rotate given points around the center-normal, until
		// the 1st point matchs the reference point.
		void rotatePoints(const PxVec3& center, const PxVec3& ref, const PxVec3& normal, PxVec3* points, PxU32 count)
		{
			const PxReal eps = FLT_EPSILON;
			PxVec3 dir0 = points[0] - center;
			if (dir0.normalize() < eps)
				return;
			PxVec3 dir1 = ref - center + normal * normal.dot(ref - center);
			if (dir1.normalize() < eps)
				return;
			PxVec3 axis = dir0.cross(dir1);
			if (axis.normalize() < eps)
				return;
			axis = normal.dot(axis) > 0 ? normal : -normal;
			PxReal angle = PxAcos(dir0.dot(dir1));
			PxQuat rot(angle, axis);
			for (PxU32 i = 0; i < count; ++i)
				points[i] = rot.rotate(points[i] - center) + center;
		}

		static const PxU32 MAX_CONVEX_CONTACTS = FaceClipper::MAX_CONTACTS + 1;

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// generates contacts between a plane and a convex
		PxU32 generateContacts(const PxPlane& plane0, const ConvexShape& convex1, const PxReal contactDist,
			PxVec3& normal, PxVec3 points[MAX_CONVEX_CONTACTS], PxReal dists[MAX_CONVEX_CONTACTS])
		{
			normal = -plane0.n;

			const PxVec3 point1 = convex1.support(normal);
			const PxReal dist = plane0.distance(point1);

			PxU32 numContacts = 0;

			if (dist < contactDist)
			{
				const PxVec3 point = point1 + normal * dist * 0.5f;
				points[numContacts] = point;
				dists[numContacts] = dist;
				++numContacts;

				PxVec3 faceNormal, facePoints[Gu::ConvexCore::MAX_FACE_POINTS];
				const PxU32 numPoints = convex1.contactFace(normal, point1, faceNormal, facePoints);
				for (PxU32 i = 0; i < numPoints; ++i)
				{
					const PxVec3 p1 = facePoints[i];
					const PxReal d = plane0.distance(p1);
					points[numContacts] = p1 + normal * d * 0.5f;
					dists[numContacts] = d;
					++numContacts;
				}
			}

			return numContacts;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// generates contacts between 2 convexes (cullDir is for triangle backface culling)
		PxU32 generateContacts(const ConvexShape& convex0, const ConvexShape& convex1, const PxReal contactDist,
			const PxVec3& cullDir, PxVec3& normal, PxVec3 points[MAX_CONVEX_CONTACTS], PxReal dists[MAX_CONVEX_CONTACTS])
		{
			PxVec3 point0, point1, axis;
			PxReal dist = RefGjkEpa::gjkDistance(convex0, convex1, convex0.pose, convex1.pose,
				convex0.margin + convex1.margin + contactDist, point0, point1, axis);

			if (dist < FLT_EPSILON)
				dist = RefGjkEpa::epaDepth(convex0, convex1, convex0.pose, convex1.pose,
					point0, point1, axis);

			if (cullDir.dot(axis) < -FLT_EPSILON)
				return 0;

			point0 -= axis * convex0.margin;
			point1 += axis * convex1.margin;
			dist -= convex0.margin + convex1.margin;

			PxU32 count = 0;

			if (dist < contactDist)
			{
				normal = axis;

				const PxVec3 point = (point0 + point1) * 0.5f;

				FaceClipper clipper;
				clipper.numPoints0 = convex0.contactFace(-axis, point0, clipper.faceNormal0, clipper.facePoints0);
				clipper.numPoints1 = convex1.contactFace(axis, point1, clipper.faceNormal1, clipper.facePoints1);

				PxU32 numPoints = clipper.clip(axis, point);

				PxReal minDist = FLT_MAX;
				for (PxU32 i = 0; i < numPoints; ++i)
					minDist = PxMin(minDist, clipper.contactDists[i]);

				if (dist < minDist - FLT_EPSILON)
				{
					points[count] = point;
					dists[count] = dist;
					count++;
				}

				for (PxU32 i = 0; i < numPoints; ++i)
				{
					points[count] = clipper.contactPoints[i];
					dists[count] = clipper.contactDists[i];
					count++;
				}
			}

			return count;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		// generates contacts between 2 convexes
		PxU32 generateContacts(const ConvexShape& convex0, const ConvexShape& convex1, const PxReal contactDist,
			PxVec3& normal, PxVec3 points[MAX_CONVEX_CONTACTS], PxReal dists[MAX_CONVEX_CONTACTS])
		{
			return generateContacts(convex0, convex1, contactDist, PxVec3(0), normal, points, dists);
		}
	}
}

#endif
