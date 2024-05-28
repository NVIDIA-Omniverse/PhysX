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

#ifndef DY_FEATHERSTONE_ARTICULATION_UTIL_H
#define DY_FEATHERSTONE_ARTICULATION_UTIL_H

#include "foundation/PxVecMath.h"
#include "CmSpatialVector.h"
#include "foundation/PxBitUtils.h"
#include "foundation/PxMemory.h"

namespace physx
{

namespace Dy
{
	static const size_t DY_MAX_DOF = 6;

	struct SpatialSubspaceMatrix
	{
		static const PxU32 MaxColumns = 3;
	public:

#ifndef __CUDACC__
		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialSubspaceMatrix() :numColumns(0)
		{
			//PxMemZero(columns, sizeof(Cm::SpatialVectorF) * 6);
			PxMemSet(columns, 0, sizeof(Cm::UnAlignedSpatialVector) * MaxColumns);
		}
#endif

		PX_CUDA_CALLABLE PX_FORCE_INLINE void setNumColumns(const PxU32 nc)
		{
			numColumns = nc;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getNumColumns() const
		{
			return numColumns;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF transposeMultiply(Cm::SpatialVectorF& v) const
		{
			PxReal result[6];
			for (PxU32 i = 0; i < numColumns; ++i)
			{
				const Cm::UnAlignedSpatialVector& row = columns[i];
				result[i] = row.dot(v);
			}

			Cm::SpatialVectorF res;
			res.top.x = result[0]; res.top.y = result[1]; res.top.z = result[2];
			res.bottom.x = result[3]; res.bottom.y = result[4]; res.bottom.z = result[5];

			return res;

		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void setColumn(const PxU32 index, const PxVec3& top, const PxVec3& bottom)
		{
			PX_ASSERT(index < MaxColumns);
			columns[index] = Cm::SpatialVectorF(top, bottom);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector& operator[](unsigned int num)
		{
			PX_ASSERT(num < MaxColumns);
			return columns[num];
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE const Cm::UnAlignedSpatialVector& operator[](unsigned int num)  const
		{
			PX_ASSERT(num < MaxColumns);
			return columns[num];
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE const Cm::UnAlignedSpatialVector* getColumns() const
		{
			return columns;
		}


	//private:
		Cm::UnAlignedSpatialVector columns[MaxColumns];			//3x24 = 72
		PxU32	numColumns;										//76
		PxU32	padding;										//80

	};

	//this should be 6x6 matrix
	//|R,		0|
	//|-R*rX,	R|
	struct SpatialTransform
	{
		PxMat33 R;
		PxQuat q;
		PxMat33 T;

	public:
		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialTransform() : R(PxZero), T(PxZero)
		{
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialTransform(const PxMat33& R_, const PxMat33& T_) : R(R_), T(T_)
		{
			q = PxQuat(R_);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialTransform(const PxQuat& q_, const PxMat33& T_) : q(q_), T(T_)
		{
			R = PxMat33(q_);
		}

		//This assume angular is the top vector and linear is the bottom vector
		/*PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVector operator *(const Cm::SpatialVector& s) const
		{
			const PxVec3 angular = R * s.angular;
			const PxVec3 linear = T * s.angular + R * s.linear;
			return Cm::SpatialVector(linear, angular);
		}*/


		////This assume angular is the top vector and linear is the bottom vector
		//PX_FORCE_INLINE Cm::SpatialVectorF operator *(Cm::SpatialVectorF& s) const
		//{
		//	const PxVec3 top = R * s.top;
		//	const PxVec3 bottom = T * s.top + R * s.bottom;

		//	const PxVec3 top1 = q.rotate(s.top);
		//	const PxVec3 bottom1 = T * s.top + q.rotate(s.bottom);

		///*	const PxVec3 tDif = (top - top1).abs();
		//	const PxVec3 bDif = (bottom - bottom1).abs();
		//	const PxReal eps = 0.001f;
		//	PX_ASSERT(tDif.x < eps && tDif.y < eps && tDif.z < eps);
		//	PX_ASSERT(bDif.x < eps && bDif.y < eps && bDif.z < eps);*/
		//	return Cm::SpatialVectorF(top1, bottom1);
		//}

		//This assume angular is the top vector and linear is the bottom vector
		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF operator *(const Cm::SpatialVectorF& s) const
		{
			//const PxVec3 top = R * s.top;
			//const PxVec3 bottom = T * s.top + R * s.bottom;

			const PxVec3 top1 = q.rotate(s.top);
			const PxVec3 bottom1 = T * s.top + q.rotate(s.bottom);

			return Cm::SpatialVectorF(top1, bottom1);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector operator *(const Cm::UnAlignedSpatialVector& s) const
		{
			//const PxVec3 top = R * s.top;
			//const PxVec3 bottom = T * s.top + R * s.bottom;

			const PxVec3 top1 = q.rotate(s.top);
			const PxVec3 bottom1 = T * s.top + q.rotate(s.bottom);

			return Cm::UnAlignedSpatialVector(top1, bottom1);
		}

		//transpose is the same as inverse, R(inverse) = R(transpose)
		//|R(t),	0	|
		//|rXR(t),	R(t)|
		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialTransform getTranspose() const
		{
			SpatialTransform ret;
			ret.q = q.getConjugate();
			ret.R = R.getTranspose();
			ret.T = T.getTranspose();
			return ret;
			
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF transposeTransform(const Cm::SpatialVectorF& s) const
		{
			const PxVec3 top1 = q.rotateInv(s.top);
			const PxVec3 bottom1 = T.transformTranspose(s.top) + q.rotateInv(s.bottom);

			return Cm::SpatialVectorF(top1, bottom1);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector transposeTransform(const Cm::UnAlignedSpatialVector& s) const
		{
			const PxVec3 top1 = q.rotateInv(s.top);
			const PxVec3 bottom1 = T.transformTranspose(s.top) + q.rotateInv(s.bottom);

			return Cm::UnAlignedSpatialVector(top1, bottom1);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void operator =(SpatialTransform& other)
		{
			R = other.R;
			q = other.q;
			T = other.T;
		}

	};

	struct InvStIs
	{
		PxReal invStIs[3][3];
	};

	//this should be 6x6 matrix and initialize to
	//|0,	M|
	//|I,	0|
	//this should be 6x6 matrix but bottomRight is the transpose of topLeft
	//so we can get rid of bottomRight
	struct SpatialMatrix
	{
		PxMat33 topLeft;		// intialize to 0 
		PxMat33 topRight;		// initialize to mass matrix
		PxMat33 bottomLeft;		// initialize to inertia
		PxU32	padding;		//4		112

	public:
		
		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix()
		{
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix(PxZERO r) : topLeft(PxZero), topRight(PxZero),
			bottomLeft(PxZero)
		{
			PX_UNUSED(r);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix(const PxMat33& topLeft_, const PxMat33& topRight_, const PxMat33& bottomLeft_)
		{
			topLeft = topLeft_;
			topRight = topRight_;
			bottomLeft = bottomLeft_;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33 getBottomRight() const
		{
			return topLeft.getTranspose();
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void setZero()
		{
			topLeft = PxMat33(0.f);
			topRight = PxMat33(0.f);
			bottomLeft = PxMat33(0.f);
		}


		//This assume angular is the top vector and linear is the bottom vector
		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVector operator *(const Cm::SpatialVector& s) const
		{
			const PxVec3 angular = topLeft * s.angular + topRight * s.linear;
			const PxVec3 linear = bottomLeft * s.angular + topLeft.transformTranspose(s.linear);
			return Cm::SpatialVector(linear, angular);
		}

		//This assume angular is the top vector and linear is the bottom vector
		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF operator *(const Cm::SpatialVectorF& s) const
		{
			const PxVec3 top = topLeft * s.top + topRight * s.bottom;
			const PxVec3 bottom = bottomLeft * s.top + topLeft.transformTranspose(s.bottom);

			return Cm::SpatialVectorF(top, bottom);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector operator *(const Cm::UnAlignedSpatialVector& s) const
		{
			const PxVec3 top = topLeft * s.top + topRight * s.bottom;
			const PxVec3 bottom = bottomLeft * s.top + topLeft.transformTranspose(s.bottom);

			return Cm::UnAlignedSpatialVector(top, bottom);
		}


		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix operator *(const PxReal& s) const
		{
			const PxMat33 newTopLeft = topLeft * s;
			const PxMat33 newTopRight = topRight * s;
			const PxMat33 newBottomLeft = bottomLeft * s;

			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix operator -(const SpatialMatrix& s) const
		{
			const PxMat33 newTopLeft = topLeft - s.topLeft;
			const PxMat33 newTopRight = topRight - s.topRight;
			const PxMat33 newBottomLeft = bottomLeft - s.bottomLeft;
			
			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix operator +(const SpatialMatrix& s) const
		{
			const PxMat33 newTopLeft = topLeft + s.topLeft;
			const PxMat33 newTopRight = topRight + s.topRight;
			const PxMat33 newBottomLeft = bottomLeft + s.bottomLeft;

			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix operator-()
		{
			const PxMat33 newTopLeft = -topLeft;
			const PxMat33 newTopRight = -topRight;
			const PxMat33 newBottomLeft = -bottomLeft;
			
			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE void operator +=(const SpatialMatrix& s)
		{
			topLeft += s.topLeft;
			topRight += s.topRight;
			bottomLeft += s.bottomLeft;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix operator *(const SpatialMatrix& s)
		{
			const PxMat33 sBottomRight = s.topLeft.getTranspose();
			const PxMat33 bottomRight = topLeft.getTranspose();

			const PxMat33 newTopLeft = topLeft * s.topLeft + topRight * s.bottomLeft;
			const PxMat33 newTopRight = topLeft * s.topRight + topRight * sBottomRight;
			const PxMat33 newBottomLeft = bottomLeft * s.topLeft + bottomRight * s.bottomLeft;

			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);
		}

		static SpatialMatrix constructSpatialMatrix(const Cm::SpatialVector& Is, const Cm::SpatialVector& stI)
		{
			//construct top left
			const PxVec3 tLeftC0 = Is.angular * stI.angular.x;
			const PxVec3 tLeftC1 = Is.angular * stI.angular.y;
			const PxVec3 tLeftC2 = Is.angular * stI.angular.z;

			const PxMat33 topLeft(tLeftC0, tLeftC1, tLeftC2);

			//construct top right
			const PxVec3 tRightC0 = Is.angular * stI.linear.x;
			const PxVec3 tRightC1 = Is.angular * stI.linear.y;
			const PxVec3 tRightC2 = Is.angular * stI.linear.z;
			const PxMat33 topRight(tRightC0, tRightC1, tRightC2);

			//construct bottom left
			const PxVec3 bLeftC0 = Is.linear * stI.angular.x;
			const PxVec3 bLeftC1 = Is.linear * stI.angular.y;
			const PxVec3 bLeftC2 = Is.linear * stI.angular.z;
			const PxMat33 bottomLeft(bLeftC0, bLeftC1, bLeftC2);

			return SpatialMatrix(topLeft, topRight, bottomLeft);
		}

		static PX_CUDA_CALLABLE SpatialMatrix constructSpatialMatrix(const Cm::SpatialVectorF& Is, const Cm::SpatialVectorF& stI)
		{
			//construct top left
			const PxVec3 tLeftC0 = Is.top * stI.top.x;
			const PxVec3 tLeftC1 = Is.top * stI.top.y;
			const PxVec3 tLeftC2 = Is.top * stI.top.z;

			const PxMat33 topLeft(tLeftC0, tLeftC1, tLeftC2);

			//construct top right
			const PxVec3 tRightC0 = Is.top * stI.bottom.x;
			const PxVec3 tRightC1 = Is.top * stI.bottom.y;
			const PxVec3 tRightC2 = Is.top * stI.bottom.z;
			const PxMat33 topRight(tRightC0, tRightC1, tRightC2);

			//construct bottom left
			const PxVec3 bLeftC0 = Is.bottom * stI.top.x;
			const PxVec3 bLeftC1 = Is.bottom * stI.top.y;
			const PxVec3 bLeftC2 = Is.bottom * stI.top.z;
			const PxMat33 bottomLeft(bLeftC0, bLeftC1, bLeftC2);

			return SpatialMatrix(topLeft, topRight, bottomLeft);
		}

		template <typename SpatialVector>
		static PX_CUDA_CALLABLE SpatialMatrix constructSpatialMatrix(const SpatialVector* columns)
		{
			const PxMat33 topLeft(columns[0].top, columns[1].top, columns[2].top);
			const PxMat33 bottomLeft(columns[0].bottom, columns[1].bottom, columns[2].bottom);
			const PxMat33 topRight(columns[3].top, columns[4].top, columns[5].top);
			
			return SpatialMatrix(topLeft, topRight, bottomLeft);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix getTranspose()
		{
			const PxMat33 newTopLeft = topLeft.getTranspose();
			const PxMat33 newTopRight = bottomLeft.getTranspose();
			const PxMat33 newBottomLeft = topRight.getTranspose();
			//const PxMat33 newBottomRight = bottomRight.getTranspose();

			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);// , newBottomRight);
		}

		//static bool isTranspose(const PxMat33& a, const PxMat33& b)
		//{
		//	PxReal eps = 0.01f;
		//	//test bottomRight is the transpose of topLeft
		//	for (PxU32 i = 0; i <3; ++i)
		//	{
		//		for (PxU32 j = 0; j <3; ++j)
		//		{
		//			if (PxAbs(a[i][j] - b[j][i]) > eps)
		//				return false;
		//		}
		//	}

		//	return true;
		//}

		PX_FORCE_INLINE bool isIdentity(const PxMat33& matrix)
		{
			const PxReal eps = 0.00001f;
			const float x = PxAbs(1.f - matrix.column0.x);
			const float y = PxAbs(1.f - matrix.column1.y);
			const float z = PxAbs(1.f - matrix.column2.z);
			const bool identity = ((x < eps) && PxAbs(matrix.column0.y - 0.f) < eps && PxAbs(matrix.column0.z - 0.f) < eps) &&
				(PxAbs(matrix.column1.x - 0.f) < eps && (y < eps) && PxAbs(matrix.column1.z - 0.f) < eps) &&
				(PxAbs(matrix.column2.x - 0.f) < eps && PxAbs(matrix.column2.y - 0.f) < eps && (z < eps));

			return identity;
		}

		PX_FORCE_INLINE bool isZero(const PxMat33& matrix)
		{
			const PxReal eps = 0.0001f;
			for (PxU32 i = 0; i < 3; ++i)
			{
				for (PxU32 j = 0; j < 3; ++j)
				{
					if (PxAbs(matrix[i][j]) > eps)
						return false;
				}
			}

			return true;
		}

		PX_FORCE_INLINE bool isIdentity()
		{
			const bool topLeftIsIdentity = isIdentity(topLeft);

			const bool topRightIsZero = isZero(topRight);

			const bool bottomLeftIsZero = isZero(bottomLeft);

			return topLeftIsIdentity && topRightIsZero && bottomLeftIsZero;
		}

		static bool isEqual(const PxMat33& s0, const PxMat33& s1)
		{
			const PxReal eps = 0.00001f;
			for (PxU32 i = 0; i < 3; ++i)
			{
				for (PxU32 j = 0; j < 3; ++j)
				{
					const PxReal t = s0[i][j] - s1[i][j];
					if (PxAbs(t) > eps)
						return false;
				}
			}

			return true;
		}

		PX_FORCE_INLINE bool isEqual(const SpatialMatrix& s)
		{
			const bool topLeftEqual = isEqual(topLeft, s.topLeft);
			const bool topRightEqual = isEqual(topRight, s.topRight);
			const bool bottomLeftEqual = isEqual(bottomLeft, s.bottomLeft);

			return topLeftEqual && topRightEqual && bottomLeftEqual;
		}

		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33 invertSym33(const PxMat33& in)
		{
			const PxVec3 v0 = in[1].cross(in[2]);
			const PxVec3 v1 = in[2].cross(in[0]);
			const PxVec3 v2 = in[0].cross(in[1]);

			const PxReal det = v0.dot(in[0]);

			if (det != 0)
			{
				const PxReal recipDet = 1.0f / det;

				return PxMat33(v0 * recipDet,
					PxVec3(v0.y, v1.y, v1.z) * recipDet,
					PxVec3(v0.z, v1.z, v2.z) * recipDet);
			}
			else
			{
				return PxMat33(PxIdentity);
			}
		}

		static PX_FORCE_INLINE aos::Mat33V invertSym33(const aos::Mat33V& in)
		{
			using namespace aos;
			const Vec3V v0 = V3Cross(in.col1, in.col2);
			const Vec3V v1 = V3Cross(in.col2, in.col0);
			const Vec3V v2 = V3Cross(in.col0, in.col1);

			const FloatV det = V3Dot(v0, in.col0);

			const FloatV recipDet = FRecip(det);

			if (!FAllEq(det, FZero()))
			{
				return Mat33V(V3Scale(v0, recipDet),
					V3Scale(V3Merge(V3GetY(v0), V3GetY(v1), V3GetZ(v1)), recipDet),
					V3Scale(V3Merge(V3GetZ(v0), V3GetZ(v1), V3GetZ(v2)), recipDet));
			}
			else
			{
				return Mat33V(V3UnitX(), V3UnitY(), V3UnitZ());
			}

			//return M33Inverse(in);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialMatrix invertInertia()
		{
			PxMat33 aa = bottomLeft, ll = topRight, la = topLeft;

			aa = (aa + aa.getTranspose())*0.5f;
			ll = (ll + ll.getTranspose())*0.5f;

			const PxMat33 AAInv = invertSym33(aa);

			const PxMat33 z = -la * AAInv;
			const PxMat33 S = ll + z * la.getTranspose();	// Schur complement of mAA

			const PxMat33 LL = invertSym33(S);

			const PxMat33 LA = LL * z;
			const PxMat33 AA = AAInv + z.getTranspose() * LA;

			const SpatialMatrix result(LA.getTranspose(), AA, LL);// , LA);

			return result;
		}

		PX_FORCE_INLINE void M33Store(const aos::Mat33V& src, PxMat33& dest)
		{
			aos::V3StoreU(src.col0, dest.column0);
			aos::V3StoreU(src.col1, dest.column1);
			aos::V3StoreU(src.col2, dest.column2);
		}

		PX_FORCE_INLINE void invertInertiaV(SpatialMatrix& result)
		{
			using namespace aos;
			Mat33V aa = M33Load(bottomLeft), ll = M33Load(topRight), la = M33Load(topLeft);

			aa = M33Scale(M33Add(aa, M33Trnsps(aa)), FHalf());
			ll = M33Scale(M33Add(ll, M33Trnsps(ll)), FHalf());

			const Mat33V AAInv = invertSym33(aa);

			const Mat33V z = M33MulM33(M33Neg(la), AAInv);
			const Mat33V S = M33Add(ll, M33MulM33(z, M33Trnsps(la)));	// Schur complement of mAA

			const Mat33V LL = invertSym33(S);

			const Mat33V LA = M33MulM33(LL, z);
			const Mat33V AA = M33Add(AAInv, M33MulM33(M33Trnsps(z), LA));

			M33Store(M33Trnsps(LA), result.topLeft);
			M33Store(AA, result.topRight);
			M33Store(LL, result.bottomLeft);
		}

		SpatialMatrix getInverse()
		{
			const PxMat33 bottomRight = topLeft.getTranspose();

			const PxMat33 blInverse = bottomLeft.getInverse();
			const PxMat33 lComp0 = blInverse * (-bottomRight);
			const PxMat33 lComp1 = topLeft * lComp0 + topRight;

			//This can be simplified
			const PxMat33 newBottomLeft = lComp1.getInverse();
			const PxMat33 newTopLeft = lComp0 * newBottomLeft;

			const PxMat33 trInverse = topRight.getInverse();
			const PxMat33 rComp0 = trInverse * (-topLeft);
			const PxMat33 rComp1 = bottomLeft + bottomRight * rComp0;

			const PxMat33 newTopRight = rComp1.getInverse();
			
			return SpatialMatrix(newTopLeft, newTopRight, newBottomLeft);
		}

		void zero()
		{
			topLeft = PxMat33(PxZero);
			topRight = PxMat33(PxZero);
			bottomLeft = PxMat33(PxZero);
		}

	};

	struct TestImpulseResponse
	{
		//Link deltaV responses to 6 test link impulses { [(1,0,0),(0,0,0)], [(0,1,0),(0,0,0)] ......[(0,0,0),(0,1,0)], [(0,0,0),(0,0,1)] }
		Cm::SpatialVectorF linkDeltaVTestImpulseResponses[6];

		Cm::SpatialVectorF getLinkDeltaVImpulseResponse(const Cm::SpatialVectorF& impulse) const
		{
			/*return rows[0] * impulse.top.x + rows[1] * impulse.top.y + rows[2] * impulse.top.z
			+ rows[3] * impulse.bottom.x + rows[4] * impulse.bottom.y + rows[5] * impulse.bottom.z;*/

			using namespace aos;
			const Cm::SpatialVectorV row0(V3LoadA(&linkDeltaVTestImpulseResponses[0].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[0].bottom.x));
			const Cm::SpatialVectorV row1(V3LoadA(&linkDeltaVTestImpulseResponses[1].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[1].bottom.x));
			const Cm::SpatialVectorV row2(V3LoadA(&linkDeltaVTestImpulseResponses[2].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[2].bottom.x));
			const Cm::SpatialVectorV row3(V3LoadA(&linkDeltaVTestImpulseResponses[3].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[3].bottom.x));
			const Cm::SpatialVectorV row4(V3LoadA(&linkDeltaVTestImpulseResponses[4].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[4].bottom.x));
			const Cm::SpatialVectorV row5(V3LoadA(&linkDeltaVTestImpulseResponses[5].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[5].bottom.x));

			const Vec4V top = V4LoadA(&impulse.top.x);
			const Vec4V bottom = V4LoadA(&impulse.bottom.x);

			const FloatV ix = V4GetX(top);
			const FloatV iy = V4GetY(top);
			const FloatV iz = V4GetZ(top);
			const FloatV ia = V4GetX(bottom);
			const FloatV ib = V4GetY(bottom);
			const FloatV ic = V4GetZ(bottom);

			Cm::SpatialVectorV res =  row0 * ix + row1 * iy + row2 * iz + row3 * ia + row4 * ib + row5 * ic;

			Cm::SpatialVectorF returnVal;
			V4StoreA(Vec4V_From_Vec3V(res.linear), &returnVal.top.x);
			V4StoreA(Vec4V_From_Vec3V(res.angular), &returnVal.bottom.x);

			return returnVal;
		}

		Cm::SpatialVectorV getLinkDeltaVImpulseResponse(const Cm::SpatialVectorV& impulse) const
		{
			using namespace aos;
			const Cm::SpatialVectorV row0(V3LoadA(&linkDeltaVTestImpulseResponses[0].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[0].bottom.x));
			const Cm::SpatialVectorV row1(V3LoadA(&linkDeltaVTestImpulseResponses[1].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[1].bottom.x));
			const Cm::SpatialVectorV row2(V3LoadA(&linkDeltaVTestImpulseResponses[2].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[2].bottom.x));
			const Cm::SpatialVectorV row3(V3LoadA(&linkDeltaVTestImpulseResponses[3].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[3].bottom.x));
			const Cm::SpatialVectorV row4(V3LoadA(&linkDeltaVTestImpulseResponses[4].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[4].bottom.x));
			const Cm::SpatialVectorV row5(V3LoadA(&linkDeltaVTestImpulseResponses[5].top.x), V3LoadA(&linkDeltaVTestImpulseResponses[5].bottom.x));

			const Vec3V top = impulse.linear;
			const Vec3V bottom = impulse.angular;

			const FloatV ix = V3GetX(top);
			const FloatV iy = V3GetY(top);
			const FloatV iz = V3GetZ(top);
			const FloatV ia = V3GetX(bottom);
			const FloatV ib = V3GetY(bottom);
			const FloatV ic = V3GetZ(bottom);

			Cm::SpatialVectorV res = row0 * ix + row1 * iy + row2 * iz + row3 * ia + row4 * ib + row5 * ic;
			return res;
		}
	};

	struct Temp6x6Matrix;

	struct Temp6x3Matrix
	{
		PxReal column[3][6];
	public:

		Temp6x3Matrix()
		{

		}

		Temp6x3Matrix(const Cm::SpatialVectorF* spatialAxis)
		{
			constructColumn(column[0], spatialAxis[0]);
			constructColumn(column[1], spatialAxis[1]);
			constructColumn(column[2], spatialAxis[2]);
		}

		void constructColumn(PxReal* dest, const Cm::SpatialVectorF& v)
		{
			dest[0] = v.top.x;
			dest[1] = v.top.y;
			dest[2] = v.top.z;

			dest[3] = v.bottom.x;
			dest[4] = v.bottom.y;
			dest[5] = v.bottom.z;
		}

		Temp6x6Matrix operator * (PxReal s[6][3]);

		////s is 3x6 matrix
		//PX_FORCE_INLINE Temp6x6Matrix operator * (PxReal s[6][3])
		//{
		//	Temp6x6Matrix temp;

		//	for (PxU32 i = 0; i < 6; ++i)
		//	{
		//		PxReal* tc = temp.column[i];

		//		for (PxU32 j = 0; j < 6; ++j)
		//		{
		//			tc[j] = 0.f;
		//			for (PxU32 k = 0; k < 3; ++k)
		//			{
		//				tc[j] += column[k][j] * s[i][k];
		//			}
		//		}
		//	}

		//	return temp;
		//}

		PX_FORCE_INLINE Temp6x3Matrix operator * (const PxMat33& s)
		{
			Temp6x3Matrix temp;

			for (PxU32 i = 0; i < 3; ++i)
			{
				PxReal* tc = temp.column[i];
				const PxVec3 sc = s[i];

				for (PxU32 j = 0; j < 6; ++j)
				{
					tc[j] = 0.f;
					for (PxU32 k = 0; k < 3; ++k)
					{
						tc[j] += column[k][j] * sc[k];
					}
				}
			}

			return temp;
		}

		PX_FORCE_INLINE bool isColumnEqual(const PxU32 ind, const Cm::SpatialVectorF& col)
		{
			PxReal temp[6];
			constructColumn(temp, col);
			const PxReal eps = 0.00001f;
			for (PxU32 i = 0; i < 6; ++i)
			{
				const PxReal dif = column[ind][i] - temp[i];
				if (PxAbs(dif) > eps)
					return false;
			}
			return true;
		}

	};


	struct Temp6x6Matrix
	{
		PxReal column[6][6];
	public:
		Temp6x6Matrix()
		{

		}

		Temp6x6Matrix(const SpatialMatrix& spatialMatrix)
		{
			constructColumn(column[0], spatialMatrix.topLeft.column0, spatialMatrix.bottomLeft.column0);
			constructColumn(column[1], spatialMatrix.topLeft.column1, spatialMatrix.bottomLeft.column1);
			constructColumn(column[2], spatialMatrix.topLeft.column2, spatialMatrix.bottomLeft.column2);

			const PxMat33 bottomRight = spatialMatrix.getBottomRight();
			constructColumn(column[3], spatialMatrix.topRight.column0, bottomRight.column0);
			constructColumn(column[4], spatialMatrix.topRight.column1, bottomRight.column1);
			constructColumn(column[5], spatialMatrix.topRight.column2, bottomRight.column2);
		}

		void constructColumn(const PxU32 ind, const PxReal* const values)
		{
			for (PxU32 i = 0; i < 6; ++i)
			{
				column[ind][i] = values[i];
			}
		}

		void constructColumn(PxReal* dest, const PxVec3& top, const PxVec3& bottom)
		{
			dest[0] = top.x;
			dest[1] = top.y;
			dest[2] = top.z;

			dest[3] = bottom.x;
			dest[4] = bottom.y;
			dest[5] = bottom.z;
		}

		Temp6x6Matrix getTranspose() const
		{
			Temp6x6Matrix temp;
			for (PxU32 i = 0; i < 6; ++i)
			{
				for (PxU32 j = 0; j < 6; ++j)
				{
					temp.column[i][j] = column[j][i];
				}
			}
			return temp;
		}

		PX_FORCE_INLINE Cm::SpatialVector operator * (const Cm::SpatialVector& s) const
		{
			Temp6x6Matrix tempMatrix = getTranspose();
			PxReal st[6];
			st[0] = s.angular.x; st[1] = s.angular.y; st[2] = s.angular.z;
			st[3] = s.linear.x; st[4] = s.linear.y; st[5] = s.linear.z;

			PxReal result[6];
			for (PxU32 i = 0; i < 6; i++)
			{
				result[i] = 0;
				for (PxU32 j = 0; j < 6; ++j)
				{
					result[i] += tempMatrix.column[i][j] * st[j];
				}
			}


			Cm::SpatialVector temp;
			temp.angular.x = result[0]; temp.angular.y = result[1]; temp.angular.z = result[2];
			temp.linear.x = result[3]; temp.linear.y = result[4]; temp.linear.z = result[5];
			return temp;
		}


		PX_FORCE_INLINE Cm::SpatialVectorF operator * (const Cm::SpatialVectorF& s) const
		{
			PxReal st[6];
			st[0] = s.top.x; st[1] = s.top.y; st[2] = s.top.z;
			st[3] = s.bottom.x; st[4] = s.bottom.y; st[5] = s.bottom.z;

			PxReal result[6];
			for (PxU32 i = 0; i < 6; ++i)
			{
				result[i] = 0.f;
				for (PxU32 j = 0; j < 6; ++j)
				{
					result[i] += column[j][i] * st[j];
				}
			}

			Cm::SpatialVectorF temp;
			temp.top.x = result[0]; temp.top.y = result[1]; temp.top.z = result[2];
			temp.bottom.x = result[3]; temp.bottom.y = result[4]; temp.bottom.z = result[5];
			return temp;
		}

		PX_FORCE_INLINE Temp6x3Matrix operator * (const Temp6x3Matrix& s) const
		{
			Temp6x3Matrix temp;
			for (PxU32 i = 0; i < 3; ++i)
			{
				PxReal* result = temp.column[i];

				const PxReal* input = s.column[i];

				for (PxU32 j = 0; j < 6; ++j)
				{
					result[j] = 0.f;
					for (PxU32 k = 0; k < 6; ++k)
					{
						result[j] += column[k][j] * input[k];
					}
				}
			}

			return temp;

		}

		PX_FORCE_INLINE Cm::SpatialVector spatialVectorMul(const Cm::SpatialVector& s)
		{
			PxReal st[6];
			st[0] = s.angular.x; st[1] = s.angular.y; st[2] = s.angular.z;
			st[3] = s.linear.x; st[4] = s.linear.y; st[5] = s.linear.z;

			PxReal result[6];
			for (PxU32 i = 0; i < 6; ++i)
			{
				result[i] = 0.f;
				for (PxU32 j = 0; j < 6; j++)
				{
					result[i] += column[i][j] * st[j];
				}
			}

			Cm::SpatialVector temp;
			temp.angular.x = result[0]; temp.angular.y = result[1]; temp.angular.z = result[2];
			temp.linear.x = result[3]; temp.linear.y = result[4]; temp.linear.z = result[5];
			return temp;
		}

		PX_FORCE_INLINE bool isEqual(const Cm::SpatialVectorF* m)
		{
			PxReal temp[6];
			const PxReal eps = 0.00001f;
			for (PxU32 i = 0; i < 6; ++i)
			{
				temp[0] = m[i].top.x; temp[1] = m[i].top.y; temp[2] = m[i].top.z;
				temp[3] = m[i].bottom.x; temp[4] = m[i].bottom.y; temp[5] = m[i].bottom.z;

				for (PxU32 j = 0; j < 6; ++j)
				{
					const PxReal dif = column[i][j] - temp[j];
					if (PxAbs(dif) > eps)
						return false;
				}
			}

			return true;
		}
	};

	//s is 3x6 matrix
	PX_FORCE_INLINE Temp6x6Matrix Temp6x3Matrix::operator * (PxReal s[6][3])
	{
		Temp6x6Matrix temp;

		for (PxU32 i = 0; i < 6; ++i)
		{
			PxReal* tc = temp.column[i];

			for (PxU32 j = 0; j < 6; ++j)
			{
				tc[j] = 0.f;
				for (PxU32 k = 0; k < 3; ++k)
				{
					tc[j] += column[k][j] * s[i][k];
				}
			}
		}

		return temp;
	}

	PX_FORCE_INLINE void calculateNewVelocity(const PxTransform& newTransform, const PxTransform& oldTransform,
		const PxReal dt, PxVec3& linear, PxVec3& angular)
	{
		//calculate the new velocity
		linear = (newTransform.p - oldTransform.p) / dt;
		PxQuat quat = newTransform.q * oldTransform.q.getConjugate();

		if (quat.w < 0)	//shortest angle.
			quat = -quat;

		PxReal angle;
		PxVec3 axis;
		quat.toRadiansAndUnitAxis(angle, axis);
		angular = (axis * angle) / dt;
	}


	// generates a pair of quaternions (swing, twist) such that in = swing * twist, with
	// swing.x = 0
	// twist.y = twist.z = 0, and twist is a unit quat
	PX_CUDA_CALLABLE PX_FORCE_INLINE void separateSwingTwist(const PxQuat& q, PxQuat& twist, PxQuat& swing1, PxQuat& swing2)
	{
		twist = q.x != 0.0f ? PxQuat(q.x, 0, 0, q.w).getNormalized() : PxQuat(PxIdentity);
		PxQuat swing = q * twist.getConjugate();
		swing1 = swing.y != 0.f ? PxQuat(0.f, swing.y, 0.f, swing.w).getNormalized() : PxQuat(PxIdentity);
		swing = swing * swing1.getConjugate();
		swing2 = swing.z != 0.f ? PxQuat(0.f, 0.f, swing.z, swing.w).getNormalized() : PxQuat(PxIdentity);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void separateSwingTwist2(const PxQuat& q, PxQuat& twist, PxQuat& swing1, PxQuat& swing2)
	{
		swing2 = q.z != 0.0f ? PxQuat(0.f, 0.f, q.z, q.w).getNormalized() : PxQuat(PxIdentity);
		PxQuat swing = q * swing2.getConjugate();
		swing1 = swing.y != 0.f ? PxQuat(0.f, swing.y, 0.f, swing.w).getNormalized() : PxQuat(PxIdentity);
		swing = swing * swing1.getConjugate();
		twist = swing.x != 0.f ? PxQuat(swing.x, 0.f, 0.f, swing.w).getNormalized() : PxQuat(PxIdentity);
	}


} //namespace Dy

}

#endif
