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

#ifndef NP_DEFORMABLE_ELEMENT_FILTER
#define NP_DEFORMABLE_ELEMENT_FILTER

#include "foundation/PxAllocator.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxArray.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxDeformableElementFilter.h"

namespace physx
{

struct ElementFilterInfo
{
	NpInternalAttachmentType::Enum internalAttachmentType;
	PxU32 actorIndex[2];
};

class NpDeformableElementFilter : public PxDeformableElementFilter, public NpBase
{
public:
					NpDeformableElementFilter(const PxDeformableElementFilterData& data, const ElementFilterInfo& info);
	virtual			~NpDeformableElementFilter();

	virtual void	getActors(PxActor*& actor0, PxActor*& actor1) const;

	virtual void	release();

	NpScene*		getSceneFromActors();
	void			addElementFilter();
	void			removeElementFilter();

	static	bool	parseElementFilter(const PxDeformableElementFilterData& data, ElementFilterInfo& info);

private:
	NpInternalAttachmentType::Enum mInternalAttachmentType{ NpInternalAttachmentType::eUNDEFINED };

	PxActor* mActor[2];

	PxArray<PxU32> mCounts[2];
	PxArray<PxU32> mIndices[2];

	PxArray<PxU32> mPairwiseIndices[2];
	PxU32 mActorIndex[2];

	bool mEnabled;
};

}

#endif

#endif
