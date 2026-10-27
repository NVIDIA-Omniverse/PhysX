// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "PvdDom.h"

class OmniPvdReader;
class OmniPvdFileReadStream;

void initPvdDomState(OmniPvdDOMState& domState);
bool buildPvdDomStateFromFile(const char* ovdFilePath, OmniPvdDOMState& domState);
bool buildPvdDomState(OmniPvdReader* reader, OmniPvdDOMState& domState);

// OVD stream messages collected during parsing.
// Layout and buffer size must match IPhysxPvd.h when building inside ovruntime/ovexts.
// IPhysxPvd.h defines OMNI_PVD_MESSAGE_LENGTH=2048 and struct OmniPvdMessage.
// If not already defined (standalone builds like PVD3), we provide the definition here.
#ifndef OMNI_PVD_MESSAGE_LENGTH
#define OMNI_PVD_MESSAGE_LENGTH 2048
struct OmniPvdMessage {
    char message[OMNI_PVD_MESSAGE_LENGTH];
    char file[OMNI_PVD_MESSAGE_LENGTH];
    uint32_t line;
    uint32_t type;
    uint32_t handle;
    char typeName[OMNI_PVD_MESSAGE_LENGTH];
    uint64_t frameId;
};
#endif

#include <vector>
typedef std::vector<OmniPvdMessage> OmniPvdMessages;
extern OmniPvdMessages gOmniPvdMessages;
