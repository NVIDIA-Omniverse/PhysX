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

/**
 * \brief Apply a single already-read OmniPVD command to the DOM state.
 *
 * Applies one command (cmdType, with its payload still held by 'reader') to
 * 'domState'. This is the body of buildPvdDomState's dispatch loop factored out
 * so a live consumer can drive getNextCommand() itself and apply one command at
 * a time -- e.g. taking a lock around just the DOM mutation while leaving the
 * (blocking) getNextCommand() read unlocked, and reacting to frame boundaries
 * (eSTOP_FRAME) as they arrive. buildPvdDomState() is exactly a
 * getNextCommand()->applyPvdDomCommand() loop to the end of the stream.
 *
 * \param reader   The reader holding the just-read command and its payload.
 * \param domState The DOM state to mutate.
 * \param cmdType  The command returned by the reader's getNextCommand().
 */
void applyPvdDomCommand(OmniPvdReader& reader, OmniPvdDOMState& domState, OmniPvdCommand::Enum cmdType);

// OVD stream messages collected during parsing.
// Layout and buffer size must match IPhysxPvd.h when building inside ovruntime.
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
