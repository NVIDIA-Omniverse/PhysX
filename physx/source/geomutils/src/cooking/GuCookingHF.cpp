// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuCooking.h"
#include "GuHeightField.h"
#include "foundation/PxFPU.h"
#include "common/PxInsertionCallback.h"
#include "CmUtils.h"

using namespace physx;
using namespace Gu;

bool immediateCooking::cookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream)
{
	if(!desc.isValid())
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Cooking::cookHeightField: user-provided heightfield descriptor is invalid!");

	PX_FPU_GUARD;

	HeightField hf(NULL);

	if(!hf.loadFromDesc(desc))
	{
		hf.releaseMemory();		
		return false;
	}

	if(!hf.save(stream, platformMismatch()))
	{
		hf.releaseMemory();		
		return false;
	}

	hf.releaseMemory();	

	return true;
}

PxHeightField* immediateCooking::createHeightField(const PxHeightFieldDesc& desc, PxInsertionCallback& insertionCallback)
{
	if(!desc.isValid())
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Cooking::createHeightField: user-provided heightfield descriptor is invalid!");
		return NULL;
	}

	PX_FPU_GUARD;

	HeightField* hf;
	PX_NEW_SERIALIZED(hf, HeightField)(NULL);

	if(!hf->loadFromDesc(desc))
	{
		PX_DELETE(hf);
		return NULL;
	}

	// create heightfield and set the HF data
	HeightField* heightField = static_cast<HeightField*>(insertionCallback.buildObjectFromData(PxConcreteType::eHEIGHTFIELD, &hf->mData));
	if(!heightField)
	{
		PX_DELETE(hf);
		return NULL;
	}

	// copy the HeightField variables
	heightField->mSampleStride = hf->mSampleStride;
	heightField->mNbSamples = hf->mNbSamples;
	heightField->mMinHeight = hf->mMinHeight;
	heightField->mMaxHeight = hf->mMaxHeight;
	heightField->mModifyCount = hf->mModifyCount;

	PX_DELETE(hf);
	return heightField;
}
