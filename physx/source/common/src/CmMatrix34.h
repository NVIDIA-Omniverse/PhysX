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
