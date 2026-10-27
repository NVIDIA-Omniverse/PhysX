// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

	virtual void	getActors(PxActor*& actor0, PxActor*& actor1) const PX_OVERRIDE;

	virtual void	release() PX_OVERRIDE;

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
