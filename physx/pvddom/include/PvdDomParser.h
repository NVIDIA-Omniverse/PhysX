// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
