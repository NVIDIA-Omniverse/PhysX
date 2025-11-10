// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


#ifndef QH_H
#define QH_H

/** \addtogroup foundation
@{
*/

/** files to always include */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define QH_PLACEMENT_NEW(p, T)	new (p) T
#define QH_DELETE_THIS			delete this
#define QH_DELETE(x)			if(x)	{ delete x;		x = NULL; }
#define QH_DELETE_ARRAY(x)		if(x)	{ delete []x;	x = NULL; }

#define QH_SIGN_BITMASK 0x80000000

#define QH_NORMALIZATION_EPSILON double(1e-20f)
#define QH_MAX_F32 3.4028234663852885981170418348452e+38F
// maximum possible double value
#define QH_MAX_F64 DBL_MAX // maximum possible double value

#define QH_EPS_F32 FLT_EPSILON // maximum relative error of double rounding
#define QH_EPS_F64 DBL_EPSILON // maximum relative error of double rounding

#define QH_MAX_REAL QH_MAX_F32
#define QH_EPS_REAL QH_EPS_F32
#define QH_NORMALIZATION_EPSILON double(1e-20f)

#if !QH_DOXYGEN
namespace quickhull
{
#endif

class QhAllocatorCallback;
class QhErrorCallback;
struct QhErrorCode;
class QhAssertHandler;

class QhInputStream;
class QhInputData;
class QhOutputStream;

class QhVec2;
class QhVec3;
class QhVec4;
class QhMat33;
class QhMat44;
class QhPlane;
class QhQuat;
class QhTransform;
class QhBounds3;

/** enum for empty constructor tag*/
enum QhEMPTY
{
	QhEmpty
};

/** enum for zero constructor tag for vectors and matrices */
enum QhZERO
{
	QhZero
};

/** enum for identity constructor flag for quaternions, transforms, and matrices */
enum QhIDENTITY
{
	QhIdentity
};

#if !QH_DOXYGEN
} // namespace quickhull
#endif

/** @} */
#endif
