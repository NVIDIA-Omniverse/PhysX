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

#ifndef DY_ARTICULATION_UTILS_H
#define DY_ARTICULATION_UTILS_H

#include "foundation/PxBitUtils.h"
#include "foundation/PxVecMath.h"
#include "CmSpatialVector.h"
#include "DyVArticulation.h"

namespace physx
{

namespace Dy
{
	struct ArticulationCore;
	struct ArticulationLink;

#define DY_ARTICULATION_DEBUG_VERIFY 0

PX_FORCE_INLINE PxU32 ArticulationLowestSetBit(ArticulationBitField val)
{
	PxU32 low = PxU32(val&0xffffffff), high = PxU32(val>>32);
	PxU32 mask = PxU32((!low)-1);
	PxU32 result = (mask&PxLowestSetBitUnsafe(low)) | ((~mask)&(PxLowestSetBitUnsafe(high)+32));
	PX_ASSERT(val & (PxU64(1)<<result));
	PX_ASSERT(!(val & ((PxU64(1)<<result)-1)));
	return result;
}

PX_FORCE_INLINE PxU32 ArticulationHighestSetBit(ArticulationBitField val)
{
	PxU32 low = PxU32(val & 0xffffffff), high = PxU32(val >> 32);
	PxU32 mask = PxU32((!high) - 1);
	PxU32 result = ((~mask)&PxHighestSetBitUnsafe(low)) | ((mask)&(PxHighestSetBitUnsafe(high) + 32));
	PX_ASSERT(val & (PxU64(1) << result));
	return result;
}

using namespace aos;

PX_FORCE_INLINE Cm::SpatialVector& unsimdRef(Cm::SpatialVectorV& v)				{ return reinterpret_cast<Cm::SpatialVector&>(v); }
PX_FORCE_INLINE const Cm::SpatialVector& unsimdRef(const Cm::SpatialVectorV& v) { return reinterpret_cast<const Cm::SpatialVector&>(v); }

}
}

#endif
