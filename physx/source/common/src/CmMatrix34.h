// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_MATRIX34_H
#define CM_MATRIX34_H

#include "foundation/PxMat34.h"
#include "foundation/PxVecMath.h"

namespace physx
{
namespace Cm
{

#if !PX_CUDA_COMPILER
// PT: similar to PxMat33Padded
class Matrix34FromTransform : public PxMat34
{
public:
	//! Construct from a PxTransform
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE Matrix34FromTransform(const PxTransform& other)
	{
		using namespace aos;

		const QuatV qV = V4LoadU(&other.q.x);
		Vec3V column0V, column1V, column2V;
		QuatGetMat33V(qV, column0V, column1V, column2V);

		// From "buildFrom"
		// PT: TODO: investigate if these overlapping stores are a problem
		V4StoreU(Vec4V_From_Vec3V(column0V), &m.column0.x);
		V4StoreU(Vec4V_From_Vec3V(column1V), &m.column1.x);
		V4StoreU(Vec4V_From_Vec3V(column2V), &m.column2.x);

		p = other.p;
	}
};
#endif

} // namespace Cm

}

#endif
