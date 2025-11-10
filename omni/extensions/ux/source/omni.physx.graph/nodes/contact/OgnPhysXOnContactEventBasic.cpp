// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

// include auto-generated header
#include <OgnPhysXOnContactEventBasicDatabase.h>

#define PHYSX_OGN_ON_CONTACT_EVENT_REPORT_FULL_DATA false

#include "OgnPhysXOnContactEvent.inl"

class OgnPhysXOnContactEventBasic : public OgnPhysXOnContactEvent<OgnPhysXOnContactEventBasic, OgnPhysXOnContactEventBasicDatabase>
{
public:
};

REGISTER_OGN_NODE()
