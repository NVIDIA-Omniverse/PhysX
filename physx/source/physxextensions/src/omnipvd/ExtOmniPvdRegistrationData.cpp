// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ExtOmniPvdRegistrationData.h"

#if PX_SUPPORT_OMNI_PVD

namespace physx
{
namespace Ext
{

void OmniPvdPxExtensionsRegistrationData::registerData(OmniPvdWriter& writer)
{
	// auto-generate class/attribute registration code from object definition file
#define OMNI_PVD_WRITER_VAR writer

#include "omnipvd/CmOmniPvdAutoGenRegisterData.h"
#include "OmniPvdPxExtensionsTypes.h"
#include "omnipvd/CmOmniPvdAutoGenClearDefines.h"

#undef OMNI_PVD_WRITER_VAR
}

}
}

#endif
