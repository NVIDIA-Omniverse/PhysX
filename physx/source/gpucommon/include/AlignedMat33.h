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

#ifndef PX_ALIGNED_MAT33_H
#define PX_ALIGNED_MAT33_H

#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "cutil_math.h"
#include "AlignedQuat.h"
#include "mathsExtensions.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
/*!
\brief 3x3 matrix class

Some clarifications, as there have been much confusion about matrix formats etc in the past.

Short:
- Matrix have base vectors in columns (vectors are column matrices, 3x1 matrices).
- Matrix is physically stored in column major format
- Matrices are concaternated from left

Long:
Given three base vectors a, b and c the matrix is stored as
         
|a.x b.x c.x|
|a.y b.y c.y|
|a.z b.z c.z|

Vectors are treated as columns, so the vector v is 

|x|
|y|
|z|

And matrices are applied _before_ the vector (pre-multiplication)
v' = M*v

|x'|   |a.x b.x c.x|   |x|   |a.x*x + b.x*y + c.x*z|
|y'| = |a.y b.y c.y| * |y| = |a.y*x + b.y*y + c.y*z|
|z'|   |a.z b.z c.z|   |z|   |a.z*x + b.z*y + c.z*z|


Physical storage and indexing:
To be compatible with popular 3d rendering APIs (read D3d and OpenGL)
the physical indexing is

|0 3 6|
|1 4 7|
|2 5 8|

index = column*3 + row

which in C++ translates to M[column][row]

The mathematical indexing is M_row,column and this is what is used for _-notation 
so _12 is 1st row, second column and operator(row, column)!

*/


class PxAlignedMat33
{
public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedMat33()
	{}

	//! identity constructor
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33(PxIDENTITY r)
		: column0(make_float4(1.0f,0.0f,0.0f, 0.f)), column1(make_float4(0.0f,1.0f,0.0f, 0.f)), column2(make_float4(0.0f,0.0f,1.0f, 0.f))
	{
		PX_UNUSED(r);
	}

	//! zero constructor
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33(PxZERO r)
		: column0(make_float4(0.0f)), column1(make_float4(0.0f)), column2(make_float4(0.0f))
	{
		PX_UNUSED(r);
	}


	//! Construct from three base vectors
	PX_CUDA_CALLABLE PxAlignedMat33(const float4& col0, const float4& col1, const float4& col2)
		: column0(col0), column1(col1), column2(col2)
	{}


	//! Construct from a quaternion
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedMat33(const PxAlignedQuat& q)
	{
		const PxReal x = q.q.x;
		const PxReal y = q.q.y;
		const PxReal z = q.q.z;
		const PxReal w = q.q.w;

		const PxReal x2 = x + x;
		const PxReal y2 = y + y;
		const PxReal z2 = z + z;

		const PxReal xx = x2*x;
		const PxReal yy = y2*y;
		const PxReal zz = z2*z;

		const PxReal xy = x2*y;
		const PxReal xz = x2*z;
		const PxReal xw = x2*w;

		const PxReal yz = y2*z;
		const PxReal yw = y2*w;
		const PxReal zw = z2*w;

		column0 = make_float4(1.0f - yy - zz, xy + zw, xz - yw, 0.f);
		column1 = make_float4(xy - zw, 1.0f - xx - zz, yz + xw, 0.f);
		column2 = make_float4(xz + yw, yz - xw, 1.0f - xx - yy, 0.f);
	}

	//! Copy constructor
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33(const PxAlignedMat33& other)
		: column0(other.column0), column1(other.column1), column2(other.column2)
	{}

	//! Assignment operator
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedMat33& operator=(const PxAlignedMat33& other)
	{
		column0 = other.column0;
		column1 = other.column1;
		column2 = other.column2;
		return *this;
	}

	//! Construct from diagonal, off-diagonals are zero.
	PX_CUDA_CALLABLE PX_INLINE static PxAlignedMat33 createDiagonal(const PxVec3& d)
	{
		return PxAlignedMat33(make_float4(d.x,0.0f,0.0f, 0.f), make_float4(0.0f,d.y,0.0f, 0.f), make_float4(0.0f,0.0f,d.z, 0.f));
	}


	//! Get transposed matrix
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedMat33 getTranspose() const
	{
		const float4 v0(make_float4(column0.x, column1.x, column2.x, 0.f));
		const float4 v1(make_float4(column0.y, column1.y, column2.y, 0.f));
		const float4 v2(make_float4(column0.z, column1.z, column2.z, 0.f));

		return PxAlignedMat33(v0,v1,v2);   
	}

	//! Get determinant
	PX_CUDA_CALLABLE PX_INLINE PxReal getDeterminant() const
	{
		return dot3(column0, cross3(column1, column2));
	}

	//! Unary minus
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33 operator-() const
	{
		return PxAlignedMat33(-column0, -column1, -column2);
	}

	//! Add
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33 operator+(const PxAlignedMat33& other) const
	{
		return PxAlignedMat33( column0+other.column0,
					  column1+other.column1,
					  column2+other.column2);
	}

	//! Subtract
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33 operator-(const PxAlignedMat33& other) const
	{
		return PxAlignedMat33( column0-other.column0,
					  column1-other.column1,
					  column2-other.column2);
	}

	//! Scalar multiplication
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33 operator*(PxReal scalar) const
	{
		return PxAlignedMat33(column0*scalar, column1*scalar, column2*scalar);
	}

	friend PxAlignedMat33 operator*(PxReal, const PxAlignedMat33&);

	//! Matrix vector multiplication (returns 'this->transform(vec)')
	PX_CUDA_CALLABLE PX_INLINE float4 operator*(const float4& vec) const
	{
		return transform(vec);
	}

	PX_CUDA_CALLABLE PX_INLINE PxVec3 operator*(const PxVec3& vec) const
	{
		return transform(vec);
	}


	// a <op>= b operators

	//! Matrix multiplication
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedMat33 operator*(const PxAlignedMat33& other) const
	{
		//Rows from this <dot> columns from other
		//column0 = transform(other.column0) etc
		return PxAlignedMat33(transform(other.column0), transform(other.column1), transform(other.column2));
	}

	//! Equals-add
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33& operator+=(const PxAlignedMat33& other)
	{
		column0 += other.column0;
		column1 += other.column1;
		column2 += other.column2;
		return *this;
	}

	//! Equals-sub
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33& operator-=(const PxAlignedMat33& other)
	{
		column0 -= other.column0;
		column1 -= other.column1;
		column2 -= other.column2;
		return *this;
	}

	//! Equals scalar multiplication
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33& operator*=(PxReal scalar)
	{
		column0 *= scalar;
		column1 *= scalar;
		column2 *= scalar;
		return *this;
	}

	//! Equals matrix multiplication
	PX_CUDA_CALLABLE PX_INLINE PxAlignedMat33& operator*=(const PxAlignedMat33 &other)
	{
		*this = *this * other;
		return *this;
	}


	//! Element access, mathematical way!
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal operator()(unsigned int row, unsigned int col) const
	{
		return (&((*this)[col]).x)[row];
	}

	//! Element access, mathematical way!
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal& operator()(unsigned int row, unsigned int col)
	{
		return (&((*this)[col]).x)[row];
	}

	// Transform etc
	
	//! Transform vector by matrix, equal to v' = M*v
	PX_CUDA_CALLABLE PX_FORCE_INLINE float4 transform(const float4& other) const
	{
		return column0*other.x + column1*other.y + column2*other.z;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transform(const PxVec3& other) const
	{
		return PxVec3(	column0.x*other.x + column1.x*other.y + column2.x*other.z,
						column0.y*other.x + column1.y*other.y + column2.y*other.z,
						column0.z*other.x + column1.z*other.y + column2.z*other.z);
	}

	//! Transform vector by matrix transpose, v' = M^t*v
	PX_CUDA_CALLABLE PX_INLINE float4 transformTranspose(const float4& other) const
	{
		return make_float4(	dot3(column0, other),
						dot3(column1, other),
						dot3(column2, other),
						0.f);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxReal* front() const
	{
		return &column0.x;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float4& operator[](unsigned int num) {return (&column0)[num];}
	PX_CUDA_CALLABLE PX_FORCE_INLINE const float4& operator[](unsigned int num) const {return (&column0)[num];}

	//Data, see above for format!

	float4 column0, column1, column2; //the three base vectors
};

// implementation from PxQuat.h
PX_CUDA_CALLABLE PX_INLINE PxAlignedQuat::PxAlignedQuat(const PxAlignedMat33& m)
{
	PxReal tr = m(0,0) + m(1,1) + m(2,2), h;
	if(tr >= 0)
	{
		h = PxSqrt(tr +1);
		q.w = 0.5f * h;
		h = 0.5f / h;

		q.x = (m(2,1) - m(1,2)) * h;
		q.y = (m(0,2) - m(2,0)) * h;
		q.z = (m(1,0) - m(0,1)) * h;
	}
	else
	{
		unsigned int i = 0; 
		if (m(1,1) > m(0,0))
			i = 1; 
		if (m(2,2) > m(i,i))
			i = 2; 
		switch (i)
		{
		case 0:
			h = PxSqrt((m(0,0) - (m(1,1) + m(2,2))) + 1);
			q.x = 0.5f * h;
			h = 0.5f / h;

			q.y = (m(0,1) + m(1,0)) * h; 
			q.z = (m(2,0) + m(0,2)) * h;
			q.w = (m(2,1) - m(1,2)) * h;
			break;
		case 1:
			h = PxSqrt((m(1,1) - (m(2,2) + m(0,0))) + 1);
			q.y = 0.5f * h;
			h = 0.5f / h;

			q.z = (m(1,2) + m(2,1)) * h;
			q.x = (m(0,1) + m(1,0)) * h;
			q.w = (m(0,2) - m(2,0)) * h;
			break;
		case 2:
			h = PxSqrt((m(2,2) - (m(0,0) + m(1,1))) + 1);
			q.z = 0.5f * h;
			h = 0.5f / h;

			q.x = (m(2,0) + m(0,2)) * h;
			q.y = (m(1,2) + m(2,1)) * h;
			q.w = (m(1,0) - m(0,1)) * h;
			break;
		default: // Make compiler happy
			q.x = q.y = q.z = q.w = 0;
			break;
		}
	}
}

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif // PX_FOUNDATION_PX_MAT33_H
