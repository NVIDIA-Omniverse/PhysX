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
