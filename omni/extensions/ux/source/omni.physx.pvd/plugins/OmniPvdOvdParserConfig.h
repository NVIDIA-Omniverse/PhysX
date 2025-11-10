// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdLoader.h"
#include "OmniPvdDom.h"
#include <unordered_map>
#include <string>

void initClassConfigMap(std::unordered_map<std::string, OmniPvdClassConfig*>& classConfigMap);
void initAttributeConfigMap(std::unordered_map<std::string, OmniPvdUsdAttributeEnum>& attributeConfigMap);
void initPvdDomState(OmniPvdDOMState& domState);
