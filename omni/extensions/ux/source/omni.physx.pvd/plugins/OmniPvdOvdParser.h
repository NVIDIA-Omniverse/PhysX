// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdLoader.h"
#include "OmniPvdDom.h"

bool isSameString(const char* str, const char* str1);

// Init before DOM reading
void initPvdDomState(OmniPvdDOMState& domState);

// Called when OVD parsing
bool buildPvdDomState(char* omniPvdFile, OmniPvdDOMState& domState);

uint8_t* getAttribData(int32_t& attribIndex, int32_t& classIndex, const char* attribName, OmniPvdObject* omniPvdObject);
OmniPvdAttributeInstList* getAttribList(int32_t& attribIndex,
                                        int32_t& classIndex,
                                        const char* attribName,
                                        OmniPvdObject* omniPvdObject);
