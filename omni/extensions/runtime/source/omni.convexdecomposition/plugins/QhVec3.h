// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef QH_VEC3_H
#define QH_VEC3_H

/** \addtogroup foundation
@{
*/

#include "QhMath.h"
#include <assert.h>

#if !QH_DOXYGEN
namespace quickhull
{
#endif

/**
\brief 3 Element vector class.

This is a 3-dimensional vector class with public data members.
*/
class QhVec3
{
  public:
	/**
	\brief default constructor leaves data uninitialized.
	*/
	 QH_FORCE_INLINE QhVec3()
	{
	}

	/**
	\brief zero constructor.
	*/
	 QH_FORCE_INLINE QhVec3(QhZERO r) : x(0.0f), y(0.0f), z(0.0f)
	{
		QH_UNUSED(r);
	}

	/**
	\brief Assigns scalar parameter to all elements.

	Useful to initialize to zero or one.

	\param[in] a Value to assign to elements.
	*/
	explicit  QH_FORCE_INLINE QhVec3(double a) : x(a), y(a), z(a)
	{
	}

	/**
	\brief Initializes from 3 scalar parameters.

	\param[in] nx Value to initialize X component.
	\param[in] ny Value to initialize Y component.
	\param[in] nz Value to initialize Z component.
	*/
	 QH_FORCE_INLINE QhVec3(double nx, double ny, double nz) : x(nx), y(ny), z(nz)
	{
	}

	/**
	\brief Copy ctor.
	*/
	 QH_FORCE_INLINE QhVec3(const QhVec3& v) : x(v.x), y(v.y), z(v.z)
	{
	}

	// Operators

	/**
	\brief Assignment operator
	*/
	 QH_FORCE_INLINE QhVec3& operator=(const QhVec3& p)
	{
		x = p.x;
		y = p.y;
		z = p.z;
		return *this;
	}

	/**
	\brief element access
	*/
	 QH_FORCE_INLINE double& operator[](unsigned int index)
	{
		assert(index <= 2);

		return reinterpret_cast<double*>(this)[index];
	}

	/**
	\brief element access
	*/
	 QH_FORCE_INLINE const double& operator[](unsigned int index) const
	{
		assert(index <= 2);

		return reinterpret_cast<const double*>(this)[index];
	}

	/**
	\brief returns true if the two vectors are exactly equal.
	*/
	 QH_FORCE_INLINE bool operator==(const QhVec3& v) const
	{
		return x == v.x && y == v.y && z == v.z;
	}

	/**
	\brief returns true if the two vectors are not exactly equal.
	*/
	 QH_FORCE_INLINE bool operator!=(const QhVec3& v) const
	{
		return x != v.x || y != v.y || z != v.z;
	}

	/**
	\brief tests for exact zero vector
	*/
	 QH_FORCE_INLINE bool isZero() const
	{
		return x == 0.0f && y == 0.0f && z == 0.0f;
	}

	/**
	\brief returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
	*/
	 QH_INLINE bool isFinite() const
	{
		return QhIsFinite(x) && QhIsFinite(y) && QhIsFinite(z);
	}

	/**
	\brief is normalized - used by API parameter validation
	*/
	 QH_FORCE_INLINE bool isNormalized() const
	{
		const double unitTolerance = 1e-4f;
		return isFinite() && QhAbs(magnitude() - 1) < unitTolerance;
	}

	/**
	\brief returns the squared magnitude

	Avoids calling QhSqrt()!
	*/
	 QH_FORCE_INLINE double magnitudeSquared() const
	{
		return x * x + y * y + z * z;
	}

	/**
	\brief returns the magnitude
	*/
	 QH_FORCE_INLINE double magnitude() const
	{
		return QhSqrt(magnitudeSquared());
	}

	/**
	\brief negation
	*/
	 QH_FORCE_INLINE QhVec3 operator-() const
	{
		return QhVec3(-x, -y, -z);
	}

	/**
	\brief vector addition
	*/
	 QH_FORCE_INLINE QhVec3 operator+(const QhVec3& v) const
	{
		return QhVec3(x + v.x, y + v.y, z + v.z);
	}

	/**
	\brief vector difference
	*/
	 QH_FORCE_INLINE QhVec3 operator-(const QhVec3& v) const
	{
		return QhVec3(x - v.x, y - v.y, z - v.z);
	}

	/**
	\brief scalar post-multiplication
	*/
	 QH_FORCE_INLINE QhVec3 operator*(double f) const
	{
		return QhVec3(x * f, y * f, z * f);
	}

	/**
	\brief scalar division
	*/
	 QH_FORCE_INLINE QhVec3 operator/(double f) const
	{
		f = 1.0f / f;
		return QhVec3(x * f, y * f, z * f);
	}

	/**
	\brief vector addition
	*/
	 QH_FORCE_INLINE QhVec3& operator+=(const QhVec3& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	/**
	\brief vector difference
	*/
	 QH_FORCE_INLINE QhVec3& operator-=(const QhVec3& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	/**
	\brief scalar multiplication
	*/
	 QH_FORCE_INLINE QhVec3& operator*=(double f)
	{
		x *= f;
		y *= f;
		z *= f;
		return *this;
	}
	/**
	\brief scalar division
	*/
	 QH_FORCE_INLINE QhVec3& operator/=(double f)
	{
		f = 1.0f / f;
		x *= f;
		y *= f;
		z *= f;
		return *this;
	}

	/**
	\brief returns the scalar product of this and other.
	*/
	 QH_FORCE_INLINE double dot(const QhVec3& v) const
	{
		return x * v.x + y * v.y + z * v.z;
	}

	/**
	\brief cross product
	*/
	 QH_FORCE_INLINE QhVec3 cross(const QhVec3& v) const
	{
		return QhVec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	/** return a unit vector */

	 QH_FORCE_INLINE QhVec3 getNormalized() const
	{
		const double m = magnitudeSquared();
		return m > 0.0f ? *this * QhRecipSqrt(m) : QhVec3(0, 0, 0);
	}

	/**
	\brief normalizes the vector in place
	*/
	 QH_FORCE_INLINE double normalize()
	{
		const double m = magnitude();
		if(m > 0.0f)
			*this /= m;
		return m;
	}

	/**
	\brief normalizes the vector in place. Does nothing if vector magnitude is under QH_NORMALIZATION_EPSILON.
	Returns vector magnitude if >= QH_NORMALIZATION_EPSILON and 0.0f otherwise.
	*/
	 QH_FORCE_INLINE double normalizeSafe()
	{
		const double mag = magnitude();
		if(mag < QH_NORMALIZATION_EPSILON)
			return 0.0f;
		*this *= 1.0f / mag;
		return mag;
	}

	/**
	\brief normalizes the vector in place. Asserts if vector magnitude is under QH_NORMALIZATION_EPSILON.
	returns vector magnitude.
	*/
	 QH_FORCE_INLINE double normalizeFast()
	{
		const double mag = magnitude();
		assert(mag >= QH_NORMALIZATION_EPSILON);
		*this *= 1.0f / mag;
		return mag;
	}

	/**
	\brief a[i] * b[i], for all i.
	*/
	 QH_FORCE_INLINE QhVec3 multiply(const QhVec3& a) const
	{
		return QhVec3(x * a.x, y * a.y, z * a.z);
	}

	/**
	\brief element-wise minimum
	*/
	 QH_FORCE_INLINE QhVec3 minimum(const QhVec3& v) const
	{
		return QhVec3(QhMin(x, v.x), QhMin(y, v.y), QhMin(z, v.z));
	}

	/**
	\brief returns MIN(x, y, z);
	*/
	 QH_FORCE_INLINE double minElement() const
	{
		return QhMin(x, QhMin(y, z));
	}

	/**
	\brief element-wise maximum
	*/
	 QH_FORCE_INLINE QhVec3 maximum(const QhVec3& v) const
	{
		return QhVec3(QhMax(x, v.x), QhMax(y, v.y), QhMax(z, v.z));
	}

	/**
	\brief returns MAX(x, y, z);
	*/
	 QH_FORCE_INLINE double maxElement() const
	{
		return QhMax(x, QhMax(y, z));
	}

	/**
	\brief returns absolute values of components;
	*/
	 QH_FORCE_INLINE QhVec3 abs() const
	{
		return QhVec3(QhAbs(x), QhAbs(y), QhAbs(z));
	}

	double x, y, z;
};

 static QH_FORCE_INLINE QhVec3 operator*(double f, const QhVec3& v)
{
	return QhVec3(f * v.x, f * v.y, f * v.z);
}

#if !QH_DOXYGEN
} // namespace quickhull
#endif

/** @} */
#endif
