// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef NP_OMNI_PVD_META_DATA_H
#define NP_OMNI_PVD_META_DATA_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{

class NpOmniPvdMetaData
{
public:
	NpOmniPvdMetaData();
	~NpOmniPvdMetaData();
	physx::PxU32 physxVersionMajor;
	physx::PxU32 physxVersionMinor;
	physx::PxU32 physxVersionBugfix;
	physx::PxU32 ovdIntegrationVersionMajor;
	physx::PxU32 ovdIntegrationVersionMinor;
};

}

#endif
