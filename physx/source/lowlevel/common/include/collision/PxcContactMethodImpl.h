// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_CONTACT_METHOD_IMPL_H
#define PXC_CONTACT_METHOD_IMPL_H

#include "GuContactMethodImpl.h"

namespace physx
{

/*!
Method prototype for contact generation routines
*/
typedef bool (*PxcContactMethod) (GU_CONTACT_METHOD_ARGS);

// Matrix of types
extern PxcContactMethod g_ContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT];
extern const bool g_CanUseContactCache[][PxGeometryType::eGEOMETRY_COUNT];
extern PxcContactMethod g_PCMContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT];

extern const bool gEnablePCMCaching[][PxGeometryType::eGEOMETRY_COUNT];
}

#endif
