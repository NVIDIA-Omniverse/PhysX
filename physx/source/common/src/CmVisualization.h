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

#ifndef CM_VISUALIZATION_H
#define CM_VISUALIZATION_H

#include "foundation/PxTransform.h"
#include "common/PxRenderOutput.h"
#include "PxConstraintDesc.h"

namespace physx
{
namespace Cm
{
	// PT: the force-inlined functions in PxRenderOutput generate a lot of code. Use these non-inlined functions instead.
	PX_PHYSX_COMMON_API	void renderOutputDebugBox(PxRenderOutput& out, const PxBounds3& box);
	PX_PHYSX_COMMON_API	void renderOutputDebugCircle(PxRenderOutput& out, PxU32 s, PxReal r);
	PX_PHYSX_COMMON_API	void renderOutputDebugBasis(PxRenderOutput& out, const PxDebugBasis& basis);
	PX_PHYSX_COMMON_API	void renderOutputDebugArrow(PxRenderOutput& out, const PxDebugArrow& arrow);

	PX_PHYSX_COMMON_API void visualizeJointFrames(PxRenderOutput& out,
							  PxReal scale,
							  const PxTransform& parent,
							  const PxTransform& child);

	PX_PHYSX_COMMON_API void visualizeLinearLimit(PxRenderOutput& out,
							  PxReal scale,
							  const PxTransform& t0,
							  const PxTransform& t1,
							  PxReal value);

	PX_PHYSX_COMMON_API void visualizeAngularLimit(PxRenderOutput& out,
							   PxReal scale,
							   const PxTransform& t0,
							   PxReal lower,
							   PxReal upper);

	PX_PHYSX_COMMON_API void visualizeLimitCone(PxRenderOutput& out,
							PxReal scale,
							const PxTransform& t,
							PxReal ySwing,
							PxReal zSwing);

	PX_PHYSX_COMMON_API void visualizeDoubleCone(PxRenderOutput& out,
							 PxReal scale,
							 const PxTransform& t,
							 PxReal angle);
	
	struct ConstraintImmediateVisualizer : public PxConstraintVisualizer
	{
		PxF32			mFrameScale;
		PxF32			mLimitScale;
		PxRenderOutput&	mCmOutput;

		//Not possible to implement
		ConstraintImmediateVisualizer& operator=( const ConstraintImmediateVisualizer& );

		ConstraintImmediateVisualizer(PxF32 frameScale, PxF32 limitScale, PxRenderOutput& output) :
			mFrameScale	(frameScale),
			mLimitScale	(limitScale),
			mCmOutput	(output)
		{
		}

		virtual void visualizeJointFrames(const PxTransform& parent, const PxTransform& child)	PX_OVERRIDE
		{
			Cm::visualizeJointFrames(mCmOutput, mFrameScale, parent, child);
		}

		virtual void visualizeLinearLimit(const PxTransform& t0, const PxTransform& t1, PxReal value)	PX_OVERRIDE
		{
			Cm::visualizeLinearLimit(mCmOutput, mLimitScale, t0, t1, value);
		}

		virtual void visualizeAngularLimit(const PxTransform& t0, PxReal lower, PxReal upper)	PX_OVERRIDE
		{
			Cm::visualizeAngularLimit(mCmOutput, mLimitScale, t0, lower, upper);
		}

		virtual void visualizeLimitCone(const PxTransform& t, PxReal tanQSwingY, PxReal tanQSwingZ)	PX_OVERRIDE
		{
			Cm::visualizeLimitCone(mCmOutput, mLimitScale, t, tanQSwingY, tanQSwingZ);
		}

		virtual void visualizeDoubleCone(const PxTransform& t, PxReal angle)	PX_OVERRIDE
		{
			Cm::visualizeDoubleCone(mCmOutput, mLimitScale, t, angle);
		}

		virtual void visualizeLine( const PxVec3& p0, const PxVec3& p1, PxU32 color)	PX_OVERRIDE
		{
			mCmOutput << color;
			mCmOutput.outputSegment(p0, p1);
		}
	};
}

}

#endif
