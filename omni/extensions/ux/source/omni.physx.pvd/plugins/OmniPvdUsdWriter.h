// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdOvdParserConfig.h"

void writeUSDFile(char* usdStageDir, int upAxis, int isUSDA, OmniPvdDOMState& domState);
long int createUSDFileInMemory(int upAxis, OmniPvdDOMState& domState);

