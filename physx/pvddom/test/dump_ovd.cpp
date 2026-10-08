// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


// Raw OVD command-stream decoder for diagnosing multi-segment (stop/startSampling)
// captures. Prints every structural command (class/object/frame lifecycle) in stream
// order, and counts attribute writes. A standalone diagnostic tool (built by the test
// CMakeLists, not run by the test suite).
//
// Usage: dump_ovd <capture.ovd> [maxStructuralLines]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <map>

#include "OmniPvdReader.h"
#include "OmniPvdFileReadStream.h"
#include "OmniPvdDefines.h"
#include "OmniPvdCommands.h"
#include "OmniPvdLibraryFunctions.h"

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("usage: dump_ovd <capture.ovd> [maxStructuralLines]\n");
        return 1;
    }
    const char* path = argv[1];
    long maxLines = (argc > 2) ? atol(argv[2]) : 100000;

    OmniPvdReader* reader = createOmniPvdReader();
    OmniPvdFileReadStream* stream = createOmniPvdFileReadStream();
    if (!reader || !stream)
    {
        printf("factory failure\n");
        if (stream) destroyOmniPvdFileReadStream(*stream);
        if (reader) destroyOmniPvdReader(*reader);
        return 1;
    }
    stream->setFileName(path);
    if (!stream->openStream())
    {
        printf("cannot open %s\n", path);
        destroyOmniPvdReader(*reader);
        destroyOmniPvdFileReadStream(*stream);
        return 1;
    }
    reader->setReadStream(*stream);

    OmniPvdVersionType maj, min, patch;
    if (!reader->startReading(maj, min, patch))
    {
        printf("startReading failed\n");
        destroyOmniPvdReader(*reader);
        stream->closeStream();
        destroyOmniPvdFileReadStream(*stream);
        return 1;
    }
    printf("OVD stream version %u.%u.%u\n", (unsigned)maj, (unsigned)min, (unsigned)patch);

    std::map<OmniPvdClassHandle, std::string> classNames;
    // The list add/remove commands carry only the ATTRIBUTE handle (the reader
    // resets class handle and attribute name before decoding them), so the
    // names have to come from the register commands via this map.
    std::map<OmniPvdAttributeHandle, std::string> attrNames;
    long structural = 0;
    unsigned long long sets = 0, listAdds = 0, listRemoves = 0, cmds = 0;

    OmniPvdCommand::Enum cmd;
    while ((cmd = reader->getNextCommand()) != OmniPvdCommand::eINVALID)
    {
        cmds++;
        switch (cmd)
        {
        case OmniPvdCommand::eREGISTER_CLASS:
        {
            OmniPvdClassHandle h = reader->getClassHandle();
            classNames[h] = reader->getClassName();
            if (structural++ < maxLines)
                printf("[%llu] REGISTER_CLASS handle=%u name=%s base=%u\n",
                       cmds, (unsigned)h, reader->getClassName(), (unsigned)reader->getBaseClassHandle());
            break;
        }
        case OmniPvdCommand::eREGISTER_ATTRIBUTE:
        case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
        case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
        {
            // Not printed, but the handle-to-name mapping is what makes the
            // LIST_ADD/LIST_REMOVE lines below readable. Re-registered segments
            // keep counting handles, so entries never collide across segments.
            attrNames[reader->getAttributeHandle()] = reader->getAttributeName();
            break;
        }
        case OmniPvdCommand::eREGISTER_ENUM:
            break; // schema detail, skip
        case OmniPvdCommand::eCREATE_OBJECT:
        {
            OmniPvdClassHandle ch = reader->getClassHandle();
            const char* cn = classNames.count(ch) ? classNames[ch].c_str() : "?";
            if (structural++ < maxLines)
                printf("[%llu] CREATE ctx=%llu class=%u(%s) obj=0x%llx name='%s'\n",
                       cmds, (unsigned long long)reader->getContextHandle(), (unsigned)ch, cn,
                       (unsigned long long)reader->getObjectHandle(),
                       reader->getObjectName() ? reader->getObjectName() : "");
            break;
        }
        case OmniPvdCommand::eDESTROY_OBJECT:
            if (structural++ < maxLines)
                printf("[%llu] DESTROY ctx=%llu obj=0x%llx\n",
                       cmds, (unsigned long long)reader->getContextHandle(),
                       (unsigned long long)reader->getObjectHandle());
            break;
        case OmniPvdCommand::eSTART_FRAME:
            if (structural++ < maxLines)
                printf("[%llu] START_FRAME ctx=0x%llx frame=%llu\n",
                       cmds, (unsigned long long)reader->getContextHandle(),
                       (unsigned long long)reader->getFrameTimeStart());
            break;
        case OmniPvdCommand::eSTOP_FRAME:
            if (structural++ < maxLines)
                printf("[%llu] STOP_FRAME ctx=0x%llx frame=%llu\n",
                       cmds, (unsigned long long)reader->getContextHandle(),
                       (unsigned long long)reader->getFrameTimeStop());
            break;
        case OmniPvdCommand::eSET_ATTRIBUTE: sets++; break;
        case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE:
        {
            // list adds are structural for lifespans (PxScene.actors etc.)
            listAdds++;
            if (structural++ < maxLines)
            {
                OmniPvdAttributeHandle ah = reader->getAttributeHandle();
                const char* an = attrNames.count(ah) ? attrNames[ah].c_str() : "?";
                const uint8_t* d = reader->getAttributeDataPointer();
                unsigned long long ref = 0;
                if (d && reader->getAttributeDataLength() >= 8) memcpy(&ref, d, 8);
                printf("[%llu] LIST_ADD obj=0x%llx attr=%u('%s') ref=0x%llx\n",
                       cmds, (unsigned long long)reader->getObjectHandle(),
                       (unsigned)ah, an, ref);
            }
            break;
        }
        case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE:
        {
            listRemoves++;
            if (structural++ < maxLines)
            {
                OmniPvdAttributeHandle ah = reader->getAttributeHandle();
                const char* an = attrNames.count(ah) ? attrNames[ah].c_str() : "?";
                const uint8_t* d = reader->getAttributeDataPointer();
                unsigned long long ref = 0;
                if (d && reader->getAttributeDataLength() >= 8) memcpy(&ref, d, 8);
                printf("[%llu] LIST_REMOVE obj=0x%llx attr=%u('%s') ref=0x%llx\n",
                       cmds, (unsigned long long)reader->getObjectHandle(),
                       (unsigned)ah, an, ref);
            }
            break;
        }
        default: break;
        }
    }
    printf("---- totals: %llu commands, %llu SETs, %llu list-adds, %llu list-removes, %ld structural lines\n",
           cmds, sets, listAdds, listRemoves, structural);
    destroyOmniPvdReader(*reader);
    stream->closeStream();
    destroyOmniPvdFileReadStream(*stream);
    return 0;
}
