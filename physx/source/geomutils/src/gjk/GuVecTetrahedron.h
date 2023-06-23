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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_VEC_TETRAHEDRON_H
#define GU_VEC_TETRAHEDRON_H
/** \addtogroup geomutils
@{
*/

#include "GuVecConvex.h"
#include "GuConvexSupportTable.h"
#include "GuDistancePointTriangle.h"

namespace physx
{
	namespace Gu
	{


		class TetrahedronV : public ConvexV
		{
		public:
			/**
			\brief Constructor
			*/
			PX_FORCE_INLINE			TetrahedronV() : ConvexV(ConvexType::eTETRAHEDRON)
			{
				margin = 0.02f;
				minMargin = PX_MAX_REAL;
				sweepMargin = PX_MAX_REAL;
			}
			/**
			\brief Constructor

			\param[in] p0 Point 0
			\param[in] p1 Point 1
			\param[in] p2 Point 2
			\param[in] p3 Point 3
			*/

			PX_FORCE_INLINE	TetrahedronV(const aos::Vec3VArg p0, const aos::Vec3VArg p1, const aos::Vec3VArg p2,
				const aos::Vec3VArg p3) : ConvexV(ConvexType::eTETRAHEDRON)
			{
				using namespace aos;
				//const FloatV zero = FZero();
				const FloatV num = FLoad(0.25f);
				center = V3Scale(V3Add(V3Add(p0, p1), V3Add(p2, p3)), num);
				//vertsX store all the x elements form those four point
				vertsX = V4SetW(V4SetZ(V4SetY(Vec4V_From_Vec3V(p0), V3GetX(p1)), V3GetX(p2)), V3GetX(p3));
				//vertsY store all the y elements from those four point
				vertsY = V4SetW(V4SetZ(V4SetY(V4Splat(V3GetY(p0)), V3GetY(p1)), V3GetY(p2)), V3GetY(p3));
				//vertsZ store all the z elements from those four point
				vertsZ = V4SetW(V4SetZ(V4SetY(V4Splat(V3GetZ(p0)), V3GetZ(p1)), V3GetZ(p2)), V3GetZ(p3));

				verts[0] = p0; verts[1] = p1; verts[2] = p2; verts[3] = p3;
				margin = 0.f;
				minMargin = PX_MAX_REAL;
				sweepMargin = PX_MAX_REAL;
			}

			PX_FORCE_INLINE	TetrahedronV(const PxVec3* pts) : ConvexV(ConvexType::eTETRAHEDRON)
			{
				using namespace aos;
				const Vec3V p0 = V3LoadU(pts[0]);
				const Vec3V p1 = V3LoadU(pts[1]);
				const Vec3V p2 = V3LoadU(pts[2]);
				const Vec3V p3 = V3LoadU(pts[3]);
				const FloatV num = FLoad(0.25f);
				center = V3Scale(V3Add(V3Add(p0, p1), V3Add(p2, p3)), num);

				vertsX = V4SetW(V4SetZ(V4SetY(Vec4V_From_Vec3V(p0), V3GetX(p1)), V3GetX(p2)), V3GetX(p3));
				vertsY = V4SetW(V4SetZ(V4SetY(V4Splat(V3GetY(p0)), V3GetY(p1)), V3GetY(p2)), V3GetY(p3));
				vertsZ = V4SetW(V4SetZ(V4SetY(V4Splat(V3GetZ(p0)), V3GetZ(p1)), V3GetZ(p2)), V3GetZ(p3));

				verts[0] = p0; verts[1] = p1; verts[2] = p2; verts[3] = p3;
				margin = 0.f;
				minMargin = PX_MAX_REAL;
				sweepMargin = PX_MAX_REAL;
			}

			/**
			\brief Copy constructor

			\param[in] tetrahedron Tetrahedron to copy
			*/
			PX_FORCE_INLINE			TetrahedronV(const Gu::TetrahedronV& tetrahedron) : ConvexV(ConvexType::eTETRAHEDRON)
			{
				using namespace aos;
				vertsX = tetrahedron.vertsX;
				vertsY = tetrahedron.vertsY;
				vertsZ = tetrahedron.vertsZ;
			
				verts[0] = tetrahedron.verts[0];
				verts[1] = tetrahedron.verts[1];
				verts[2] = tetrahedron.verts[2];
				verts[3] = tetrahedron.verts[3];

				center = tetrahedron.center;
				margin = 0.f;
				minMargin = PX_MAX_REAL;
				sweepMargin = PX_MAX_REAL;
			}
			/**
			\brief Destructor
			*/
			PX_FORCE_INLINE			~TetrahedronV()
			{
			}


			PX_FORCE_INLINE aos::FloatV getSweepMargin() const
			{
				return aos::FMax();
			}

			PX_FORCE_INLINE void setCenter(const aos::Vec3VArg _center)
			{
				using namespace aos;
				Vec3V offset = V3Sub(_center, center);
				center = _center;
				vertsX = V4Add(vertsX, V4Splat(V3GetX(offset)));
				vertsY = V4Add(vertsY, V4Splat(V3GetY(offset)));
				vertsZ = V4Add(vertsZ, V4Splat(V3GetZ(offset)));

				verts[0] = V3Add(verts[0], offset);
				verts[1] = V3Add(verts[1], offset);
				verts[2] = V3Add(verts[2], offset);
				verts[3] = V3Add(verts[3], offset);
			}

			PX_FORCE_INLINE aos::Vec4V getProjection(const aos::Vec3VArg dir) const 
			{
				using namespace aos;

				const Vec4V dx = V4Scale(vertsX, V3GetX(dir));
				const Vec4V dy = V4Scale(vertsY, V3GetY(dir));
				const Vec4V dz = V4Scale(vertsZ, V3GetZ(dir));

				return V4Add(dx, V4Add(dy, dz));
			}

			//dir is in local space, verts in the local space
			PX_FORCE_INLINE aos::Vec3V supportLocal(const aos::Vec3VArg dir) const
			{
				using namespace aos;

				const Vec4V d = getProjection(dir);

				const FloatV d0 = V4GetX(d);
				const FloatV d1 = V4GetY(d);
				const FloatV d2 = V4GetZ(d);
				const FloatV d3 = V4GetW(d);

				const BoolV con0 = BAnd(BAnd(FIsGrtr(d0, d1), FIsGrtr(d0, d2)), FIsGrtr(d0, d3));
				const BoolV con1 = BAnd(FIsGrtr(d1, d2), FIsGrtr(d1, d3));
				const BoolV con2 = FIsGrtr(d2, d3);
				return V3Sel(con0, verts[0], V3Sel(con1, verts[1], V3Sel(con2, verts[2], verts[3])));
			}


			//dir is in b space
			PX_FORCE_INLINE aos::Vec3V supportRelative(const aos::Vec3VArg dir, const aos::PxMatTransformV& aToB, const aos::PxMatTransformV& aTobT) const
			{
				using namespace aos;
				//verts are in local space
				//			const Vec3V _dir = aToB.rotateInv(dir); //transform dir back to a space
				const Vec3V _dir = aTobT.rotate(dir); //transform dir back to a space
				const Vec3V maxPoint = supportLocal(_dir);
				return aToB.transform(maxPoint);//transform maxPoint to the b space
			}

			PX_FORCE_INLINE aos::Vec3V supportLocal(const aos::Vec3VArg dir, PxI32& index) const
			{

				using namespace aos;
				
				const Vec4V d = getProjection(dir);
				const FloatV d0 = V4GetX(d);
				const FloatV d1 = V4GetY(d);
				const FloatV d2 = V4GetZ(d);
				const FloatV d3 = V4GetW(d);

				const BoolV con0 = BAnd(BAnd(FIsGrtr(d0, d1), FIsGrtr(d0, d2)), FIsGrtr(d0, d3));
				const BoolV con1 = BAnd(FIsGrtr(d1, d2), FIsGrtr(d1, d3));
				const BoolV con2 = FIsGrtr(d2, d3);

	
				const VecI32V vIndex = VecI32V_Sel(con0, I4Load(0), VecI32V_Sel(con1, I4Load(1), VecI32V_Sel(con2, I4Load(2), I4Load(3))));
				PxI32_From_VecI32V(vIndex, &index);

				//return V3Sel(con0, v0, V3Sel(con1, v1, v2));
				return verts[index];
			}

			PX_FORCE_INLINE aos::Vec3V supportRelative(const aos::Vec3VArg dir, const aos::PxMatTransformV& aToB,
				const aos::PxMatTransformV& aTobT, PxI32& index)const
			{
				//don't put margin in the triangle
				using namespace aos;
				//transfer dir into the local space of triangle
				//			const Vec3V _dir = aToB.rotateInv(dir);
				const Vec3V _dir = aTobT.rotate(dir);
				return aToB.transform(supportLocal(_dir, index));//transform the support poin to b space
			}

			PX_FORCE_INLINE aos::Vec3V supportPoint(const PxI32 index)const
			{
				return verts[index];
			}

			/**
			\brief Array of Vertices.
			*/
			aos::Vec3V		verts[4];
			aos::Vec4V		vertsX;
			aos::Vec4V		vertsY;
			aos::Vec4V		vertsZ;
		};
	}

}

#endif
