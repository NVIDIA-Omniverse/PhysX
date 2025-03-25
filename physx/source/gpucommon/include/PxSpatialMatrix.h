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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_SPATIAL_MATRIX_H
#define PX_SPATIAL_MATRIX_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxMat33.h"
#include "CmSpatialVector.h"

namespace physx
{
	struct PxSpatialMatrix33
	{
		PxReal column[3][3];
	};

	struct PxSpatialMatrix63
	{
		PxReal column[6][3];
	};
	struct PxSpatialMatrix36
	{
		PxReal column[3][6];
	};

	//144 bytes
	struct PxSpatialMatrix
	{
	public:
		PxReal column[6][6];

		PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector operator * (const Cm::UnAlignedSpatialVector& s) const
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

			Cm::UnAlignedSpatialVector temp;
			temp.top.x = result[0]; temp.top.y = result[1]; temp.top.z = result[2];
			temp.bottom.x = result[3]; temp.bottom.y = result[4]; temp.bottom.z = result[5];
			return temp;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxMat33 invertSym33(const PxMat33& in)
		{
			PxVec3 v0 = in[1].cross(in[2]),
				v1 = in[2].cross(in[0]),
				v2 = in[0].cross(in[1]);

			PxReal det = v0.dot(in[0]);

			if (det != 0)
			{
				PxReal recipDet = 1.0f / det;

				return PxMat33(v0 * recipDet,
					PxVec3(v0.y, v1.y, v1.z) * recipDet,
					PxVec3(v0.z, v1.z, v2.z) * recipDet);
			}
			else
			{
				return PxMat33(PxIdentity);
			}
		}

		PX_CUDA_CALLABLE void initialize(const PxMat33& inertia, const PxReal mass)
		{
			column[0][0] = 0.f; column[0][1] = 0.f; column[0][2] = 0.f;
			column[1][0] = 0.f; column[1][1] = 0.f; column[1][2] = 0.f;
			column[2][0] = 0.f; column[2][1] = 0.f; column[2][2] = 0.f;
			
			column[0][3] = mass; column[0][4] = 0.f; column[0][5] = 0.f;
			column[1][3] = 0.f; column[1][4] = mass; column[1][5] = 0.f;
			column[2][3] = 0.f; column[2][4] = 0.f; column[2][5] = mass;

			column[3][0] = inertia.column0.x; column[3][1] = inertia.column0.y; column[3][2] = inertia.column0.z;
			column[4][0] = inertia.column1.x; column[4][1] = inertia.column1.y; column[4][2] = inertia.column1.z;
			column[5][0] = inertia.column2.x; column[5][1] = inertia.column2.y; column[5][2] = inertia.column2.z;

			column[3][3] = 0.f; column[3][4] = 0.f; column[3][5] = 0.f;
			column[4][3] = 0.f; column[4][4] = 0.f; column[4][5] = 0.f;
			column[5][3] = 0.f; column[5][4] = 0.f; column[5][5] = 0.f;
		}

		PX_CUDA_CALLABLE PxMat33 topLeft() const
		{
			return PxMat33(	PxVec3(column[0][0], column[0][1], column[0][2]),
							PxVec3(column[1][0], column[1][1], column[1][2]),
							PxVec3(column[2][0], column[2][1], column[2][2]));
		}

		PX_CUDA_CALLABLE void setTopLeft(const PxMat33& m)
		{
			column[0][0] = m.column0.x; column[0][1] = m.column0.y; column[0][2] = m.column0.z;
			column[1][0] = m.column1.x; column[1][1] = m.column1.y; column[1][2] = m.column1.z;
			column[2][0] = m.column2.x; column[2][1] = m.column2.y; column[2][2] = m.column2.z;
		}

		PX_CUDA_CALLABLE PxMat33 bottomLeft() const
		{
			return PxMat33(	PxVec3(column[0][3], column[0][4], column[0][5]),
							PxVec3(column[1][3], column[1][4], column[1][5]),
							PxVec3(column[2][3], column[2][4], column[2][5]));
		}

		PX_CUDA_CALLABLE void setBottomLeft(const PxMat33& m)
		{
			column[0][3] = m.column0.x; column[0][4] = m.column0.y; column[0][5] = m.column0.z;
			column[1][3] = m.column1.x; column[1][4] = m.column1.y; column[1][5] = m.column1.z;
			column[2][3] = m.column2.x; column[2][4] = m.column2.y; column[2][5] = m.column2.z;
		}

		PX_CUDA_CALLABLE PxMat33 topRight() const
		{
			return PxMat33(	PxVec3(column[3][0], column[3][1], column[3][2]),
							PxVec3(column[4][0], column[4][1], column[4][2]),
							PxVec3(column[5][0], column[5][1], column[5][2]));
		}

		PX_CUDA_CALLABLE void setTopRight(const PxMat33& m)
		{
			column[3][0] = m.column0.x; column[3][1] = m.column0.y; column[3][2] = m.column0.z;
			column[4][0] = m.column1.x; column[4][1] = m.column1.y; column[4][2] = m.column1.z;
			column[5][0] = m.column2.x; column[5][1] = m.column2.y; column[5][2] = m.column2.z;
		}

		PX_CUDA_CALLABLE PxMat33 bottomRight() const
		{
			return PxMat33(	PxVec3(column[3][3], column[3][4], column[3][5]),
							PxVec3(column[4][3], column[4][4], column[4][5]),
							PxVec3(column[5][3], column[5][4], column[5][5]));
		}

		PX_CUDA_CALLABLE void setBottomRight(const PxMat33& m)
		{
			column[3][3] = m.column0.x; column[3][4] = m.column0.y; column[3][5] = m.column0.z;
			column[4][3] = m.column1.x; column[4][4] = m.column1.y; column[4][5] = m.column1.z;
			column[5][3] = m.column2.x; column[5][4] = m.column2.y; column[5][5] = m.column2.z;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxSpatialMatrix invertInertia()
		{
			//bottom left
			PxMat33 aa(	PxVec3(column[0][3], column[0][4], column[0][5]),
						PxVec3(column[1][3], column[1][4], column[1][5]),
						PxVec3(column[2][3], column[2][4], column[2][5]));


			// top right
			PxMat33 ll(PxVec3(column[3][0], column[3][1], column[3][2]),
				PxVec3(column[4][0], column[4][1], column[4][2]),
				PxVec3(column[5][0], column[5][1], column[5][2]));
			
			//top left
			PxMat33 la(PxVec3(column[0][0], column[0][1], column[0][2]),
				PxVec3(column[1][0], column[1][1], column[1][2]),
				PxVec3(column[2][0], column[2][1], column[2][2]));

			aa = (aa + aa.getTranspose())*0.5f;
			ll = (ll + ll.getTranspose())*0.5f;

			PxMat33 AAInv = invertSym33(aa);

			PxMat33 z = -la * AAInv;
			PxMat33 S = ll + z * la.getTranspose();	// Schur complement of mAA

			PxMat33 LL = invertSym33(S);

			PxMat33 LA = LL * z;
			PxMat33 AA = AAInv + z.getTranspose() * LA;

			PxSpatialMatrix result;
			PxMat33 topleft = LA.getTranspose();
			//top left
			result.column[0][0] = topleft.column0[0];
			result.column[0][1] = topleft.column0[1];
			result.column[0][2] = topleft.column0[2];

			result.column[1][0] = topleft.column1[0];
			result.column[1][1] = topleft.column1[1];
			result.column[1][2] = topleft.column1[2];

			result.column[2][0] = topleft.column2[0];
			result.column[2][1] = topleft.column2[1];
			result.column[2][2] = topleft.column2[2];

			//top right
			result.column[3][0] = AA.column0[0];
			result.column[3][1] = AA.column0[1];
			result.column[3][2] = AA.column0[2];

			result.column[4][0] = AA.column1[0];
			result.column[4][1] = AA.column1[1];
			result.column[4][2] = AA.column1[2];

			result.column[5][0] = AA.column2[0];
			result.column[5][1] = AA.column2[1];
			result.column[5][2] = AA.column2[2];

			//bottom left
			result.column[0][3] = LL.column0[0];
			result.column[0][4] = LL.column0[1];
			result.column[0][5] = LL.column0[2];

			result.column[1][3] = LL.column1[0];
			result.column[1][4] = LL.column1[1];
			result.column[1][5] = LL.column1[2];

			result.column[2][3] = LL.column2[0];
			result.column[2][4] = LL.column2[1];
			result.column[2][5] = LL.column2[2];

			//bottom right
			result.column[3][3] = LA.column0[0];
			result.column[3][4] = LA.column0[1];
			result.column[3][5] = LA.column0[2];

			result.column[4][3] = LA.column1[0];
			result.column[4][4] = LA.column1[1];
			result.column[4][5] = LA.column1[2];

			result.column[5][3] = LA.column2[0];
			result.column[5][4] = LA.column2[1];
			result.column[5][5] = LA.column2[2];
			return result;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal& operator()(PxU32 row, PxU32 col)
		{
			return column[col][row];
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal operator()(PxU32 row, PxU32 col) const
		{
			return column[col][row];
		}

		//A bit of a misuse. In featherstone articulations, we use this matrix to store
		//the invMass/sqrtInvInertia of a rigid body in a way that is compatible with the 
		//response matrix produced by our articulation code. In order for this to work,
		//we must also scale the angular term by the inertia tensor for rigid bodies.
		PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 multiplyInertia(const PxVec3& v) const 
		{
			return PxVec3(	column[3][0] * v.x + column[3][1] * v.y + column[3][2] * v.z,
							column[4][0] * v.x + column[4][1] * v.y + column[4][2] * v.z, 
							column[5][0] * v.x + column[5][1] * v.y + column[5][2] * v.z
			);
		}


	};

}

#endif