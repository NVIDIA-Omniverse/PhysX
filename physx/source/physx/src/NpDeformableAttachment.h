// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_DEFORMABLE_ATTACHMENT
#define NP_DEFORMABLE_ATTACHMENT

#include "foundation/PxAllocator.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxArray.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxDeformableAttachment.h"

namespace physx
{

struct AttachmentInfo
{
	NpInternalAttachmentType::Enum internalAttachmentType;
	PxU32 actorIndex[2];
};

class NpDeformableAttachment : public PxDeformableAttachment, public NpBase
{
public:
					NpDeformableAttachment(const PxDeformableAttachmentData& data, const AttachmentInfo& info);
	virtual			~NpDeformableAttachment();

	virtual void	getActors(PxActor*& actor0, PxActor*& actor1) const PX_OVERRIDE;

	virtual void	updatePose(const PxTransform& pose) PX_OVERRIDE;

	virtual void	release() PX_OVERRIDE;

	NpScene*		getSceneFromActors();
	void			addAttachment();
	void			removeAttachment();

	static	bool	parseAttachment(const PxDeformableAttachmentData& data, AttachmentInfo& info);

private:
	NpInternalAttachmentType::Enum mInternalAttachmentType{ NpInternalAttachmentType::eUNDEFINED };

	PxActor* mActor[2];
	PxDeformableAttachmentTargetType::Enum mType[2];
	PxArray<PxU32> mIndices[2];
	PxArray<PxVec4> mCoords[2];
	PxTransform mPose[2];

	PxArray<PxU32> mHandles;
	PxU32 mActorIndex[2];

	bool mEnabled;
};

}

#endif

#endif
