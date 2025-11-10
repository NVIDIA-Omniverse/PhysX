// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

// include auto-generated header
#include <OgnPhysXOnContactEventAdvancedDatabase.h>

#define PHYSX_OGN_ON_CONTACT_EVENT_REPORT_FULL_DATA true

#include "OgnPhysXOnContactEvent.inl"

class OgnPhysXOnContactEventAdvanced : public OgnPhysXOnContactEvent<OgnPhysXOnContactEventAdvanced, OgnPhysXOnContactEventAdvancedDatabase>
{
public:
};

REGISTER_OGN_NODE()
