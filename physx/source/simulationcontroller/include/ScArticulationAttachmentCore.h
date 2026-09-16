// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef SC_ATTACHMENT_CORE_H
#define SC_ATTACHMENT_CORE_H

#include "foundation/PxVec3.h"

namespace physx
{
namespace Sc
{
	class ArticulationAttachmentCore
	{
	public:

// PX_SERIALIZATION
															ArticulationAttachmentCore(const PxEMPTY) :  mTendonSim(NULL) {}
						void								preExportDataReset() { }
//~PX_SERIALIZATION

		ArticulationAttachmentCore() : mLowLimit(PX_MAX_F32), mHighLimit(-PX_MAX_F32), mRestLength(0.f)
		{

		}

		PxVec3								mRelativeOffset;		//relative offset to the link(in link space)
		ArticulationAttachmentCore*			mParent;
		PxReal								mLowLimit;
		PxReal								mHighLimit;
		PxReal								mRestLength;
		PxReal								mCoefficient;
		PxU32								mLLLinkIndex;
		PxU32								mAttachmentIndex;
		Sc::ArticulationSpatialTendonSim*	mTendonSim;

	};
}//namespace Sc
}//namespace physx

#endif
