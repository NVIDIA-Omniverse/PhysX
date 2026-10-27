// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpOmniPvdMetaData.h"
#include "NpOmniPvd.h"
#include "foundation/PxPhysicsVersion.h"

namespace physx
{

NpOmniPvdMetaData::NpOmniPvdMetaData()
{
	physxVersionMajor = PX_PHYSICS_VERSION_MAJOR;
	physxVersionMinor = PX_PHYSICS_VERSION_MINOR;
	physxVersionBugfix = PX_PHYSICS_VERSION_BUGFIX;
	ovdIntegrationVersionMajor =  PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR;
	ovdIntegrationVersionMinor =  PX_PHYSICS_OVD_INTEGRATION_VERSION_MINOR;
}

NpOmniPvdMetaData::~NpOmniPvdMetaData()
{
}

}