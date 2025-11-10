// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef QH_MATH_H
#define QH_MATH_H

/** \addtogroup foundation
@{
*/

#include "QhPreprocessor.h"


#if QH_VC
#pragma warning(push)
#pragma warning(disable : 4985) // 'symbol name': attributes not present on previous declaration
#endif
#include <math.h>
#if QH_VC
#pragma warning(pop)
#endif

#include <float.h>
#include <assert.h>

#if !QH_DOXYGEN
namespace quickhull
{
#endif

// constants
static const double QhPi = double(3.141592653589793);
static const double QhHalfPi = double(1.57079632679489661923);
static const double QhTwoPi = double(6.28318530717958647692);
static const double QhInvPi = double(0.31830988618379067154);
static const double QhInvTwoPi = double(0.15915494309189533577);
static const double QhPiDivTwo = double(1.57079632679489661923);
static const double QhPiDivFour = double(0.78539816339744830962);

/**
\brief The return value is the greater of the two specified values.
*/
template <class T>
 QH_FORCE_INLINE T QhMax(T a, T b)
{
	return a < b ? b : a;
}

#define selectMax(a,b) a > b ? a : b

//! overload for double to use fsel on xbox
template <>
 QH_FORCE_INLINE double QhMax(double a, double b)
{
	return selectMax(a, b);
}

/**
\brief The return value is the lesser of the two specified values.
*/
template <class T>
 QH_FORCE_INLINE T QhMin(T a, T b)
{
	return a < b ? a : b;
}

#define selectMin(a,b) a < b ? a : b;

template <>
//! overload for double to use fsel on xbox
 QH_FORCE_INLINE double QhMin(double a, double b)
{
	return selectMin(a, b);
}

/**
\brief abs returns the absolute value of its argument.
*/
 QH_FORCE_INLINE double QhAbs(double a)
{
	return ::fabs(a);
}

/**
\brief abs returns the absolute value of its argument.
*/
 QH_FORCE_INLINE int32_t QhAbs(int32_t a)
{
	return ::abs(a);
}

/**
\brief Clamps v to the range [hi,lo]
*/
template <class T>
 QH_FORCE_INLINE T QhClamp(T v, T lo, T hi)
{
	assert(lo <= hi);
	return QhMin(hi, QhMax(lo, v));
}

//!	\brief Square root.
 QH_FORCE_INLINE double QhSqrt(double a)
{
	return ::sqrt(a);
}

//!	\brief reciprocal square root.
 QH_FORCE_INLINE double QhRecipSqrt(double a)
{
	return 1 / ::sqrt(a);
}

//!	\brief square of the argument
 QH_FORCE_INLINE double QhSqr(const double a)
{
	return a * a;
}

//! trigonometry -- all angles are in radians.

//!	\brief Sine of an angle ( <b>Unit:</b> Radians )
 QH_FORCE_INLINE double QhSin(double a)
{
	return ::sin(a);
}

//!	\brief Cosine of an angle (<b>Unit:</b> Radians)
 QH_FORCE_INLINE double QhCos(double a)
{
	return ::cos(a);
}

//! \brief compute sine and cosine at the same time
 QH_FORCE_INLINE void QhSinCos(const double a, double& sin, double& cos)
{
	sin = QhSin(a);
	cos = QhCos(a);
}

/**
\brief Tangent of an angle.
<b>Unit:</b> Radians
*/
 QH_FORCE_INLINE double QhTan(double a)
{
	return ::tan(a);
}

/**
\brief Arcsine.
Returns angle between -PI/2 and PI/2 in radians
<b>Unit:</b> Radians
*/
 QH_FORCE_INLINE double QhAsin(double f)
{
	return ::asin(QhClamp(f, -1.0, 1.0));
}

/**
\brief Arccosine.
Returns angle between 0 and PI in radians
<b>Unit:</b> Radians
*/
 QH_FORCE_INLINE double QhAcos(double f)
{
	return ::acos(QhClamp(f, -1.0, 1.0));
}

/**
\brief ArcTangent.
Returns angle between -PI/2 and PI/2 in radians
<b>Unit:</b> Radians
*/
 QH_FORCE_INLINE double QhAtan(double a)
{
	return ::atan(a);
}

/**
\brief Arctangent of (x/y) with correct sign.
Returns angle between -PI and PI in radians
<b>Unit:</b> Radians
*/
 QH_FORCE_INLINE double QhAtan2(double x, double y)
{
	return ::atan2(x, y);
}

/**
\brief Converts degrees to radians.
*/
 QH_FORCE_INLINE double QhDegToRad(const double a)
{
	return 0.01745329251994329547f * a;
}

//!	\brief returns true if the passed number is a finite floating point number as opposed to INF, NAN, etc.
 QH_FORCE_INLINE bool QhIsFinite(double f)
{
	return !!isfinite(f);
}

 QH_FORCE_INLINE double QhSign(double a)
{
	return (a >= 0.0f) ? 1.0f : -1.0f;
}

 QH_FORCE_INLINE double QhSign2(double a, double eps = FLT_EPSILON)
{
	return (a < -eps) ? -1.0f : (a > eps) ? 1.0f : 0.0f;
}

#if !QH_DOXYGEN
} // namespace quickhull
#endif

/** @} */
#endif
