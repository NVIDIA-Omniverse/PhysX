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

#include "GuCooking.h"
#include "GuBVH.h"
#include "foundation/PxFPU.h"
#include "cooking/PxBVHDesc.h"
#include "common/PxInsertionCallback.h"

using namespace physx;
using namespace Gu;

static bool buildBVH(const PxBVHDesc& desc, BVHData& data, const char* errorMessage)
{
	if(!desc.isValid())
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, errorMessage);

	BVHBuildStrategy bs;
	if(desc.buildStrategy==PxBVHBuildStrategy::eFAST)
		bs = BVH_SPLATTER_POINTS;
	else if(desc.buildStrategy==PxBVHBuildStrategy::eDEFAULT)
		bs = BVH_SPLATTER_POINTS_SPLIT_GEOM_CENTER;
	else //if(desc.buildStrategy==PxBVHBuildStrategy::eSAH)
		bs = BVH_SAH;

	return data.build(desc.bounds.count, desc.bounds.data, desc.bounds.stride, desc.enlargement, desc.numPrimsPerLeaf, bs);
}

bool immediateCooking::cookBVH(const PxBVHDesc& desc, PxOutputStream& stream)
{
	PX_FPU_GUARD;

	BVHData bvhData;
	if(!buildBVH(desc, bvhData, "Cooking::cookBVH: user-provided BVH descriptor is invalid!"))
		return false;

	return bvhData.save(stream, platformMismatch());
}

PxBVH* immediateCooking::createBVH(const PxBVHDesc& desc, PxInsertionCallback& insertionCallback)
{
	PX_FPU_GUARD;

	BVHData bvhData;
	if(!buildBVH(desc, bvhData, "Cooking::createBVH: user-provided BVH descriptor is invalid!"))
		return NULL;

	return static_cast<PxBVH*>(insertionCallback.buildObjectFromData(PxConcreteType::eBVH, &bvhData));
}

