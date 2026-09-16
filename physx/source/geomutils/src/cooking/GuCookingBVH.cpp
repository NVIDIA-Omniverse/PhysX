// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

