// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "PvdDom.h"
#include <unordered_map>
#include <string>

void initClassConfigMap(std::unordered_map<std::string, OmniPvdPhysXClassEnum>& classConfigMap);
void initAttributeConfigMap(std::unordered_map<std::string, PvdDomAttributeEnum>& attributeConfigMap);
void initPvdDomState(OmniPvdDOMState& domState);
