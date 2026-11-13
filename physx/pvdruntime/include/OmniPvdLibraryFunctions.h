// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_LIBRARY_FUNCTIONS_H
#define OMNI_PVD_LIBRARY_FUNCTIONS_H

#include "OmniPvdDefines.h"

class OmniPvdReader;
class OmniPvdWriter;
class OmniPvdFileReadStream;
class OmniPvdFileWriteStream;
class OmniPvdMemoryStream;
class OmniPvdSocketReadStream;
class OmniPvdSocketWriteStream;

// Each module that statically links PVDRuntime owns an independent runtime copy. Destroy every
// object returned by a createOmniPvd* function with the matching destroyOmniPvd* function from
// the same module. Pass only objects returned by these create functions to the matching destroy
// functions.
OMNI_PVD_API OmniPvdReader* OMNI_PVD_CALL createOmniPvdReader();
// Limits apply before payload allocation. Attribute paths cannot exceed 32 handles.
// Returns nullptr if the reader cannot be allocated.
OMNI_PVD_API OmniPvdReader* OMNI_PVD_CALL createOmniPvdReaderWithLimits(uint32_t maxAttributeBytes);
struct OmniPvdReaderStatus
{
    enum Code { eNONE, eEND_OF_STREAM, eINVALID, eUNSUPPORTED, eLIMIT };
    Code code;
    uint8_t attributePathDepth;
};
// Accepts readers from this module's createOmniPvdReader* factories only.
OMNI_PVD_API OmniPvdReaderStatus OMNI_PVD_CALL getOmniPvdReaderStatus(const OmniPvdReader& reader);

/** \brief Borrowed diagnostic callback and opt-in successful-read tracing. */
struct OmniPvdReaderDiagnostics
{
    OmniPvdLogFunction logFunction = nullptr; ///< Null disables this reader's callback, including errors.
    bool verbose = false; ///< Enables accepted-header detail and completed-command traces.
};

/**
 * \brief Replaces a factory reader's callback and verbosity without changing its parsing state.
 *
 * Accepts only readers from this module's createOmniPvdReader or createOmniPvdReaderWithLimits.
 * Apply before reading to receive the corresponding output. Reconfiguration never replays a
 * previous diagnostic. Errors, including a rejected complete header's version and reason, do not
 * require verbose. A command may produce several ordered callbacks with escaped, bounded text.
 * The callback runs synchronously. Its char buffer is read-only and valid only during that call;
 * copy it to retain it. The callback must not throw, re-enter or destroy the active reader.
 * Serialize reader access and protect mutable callback state shared by concurrent readers.
 *
 * \param[in,out] reader Reader created by this module's factory; custom and other-module readers are invalid.
 * \param[in] diagnostics Both fields replace the current settings. A null callback disables all output.
 */
OMNI_PVD_API void OMNI_PVD_CALL setOmniPvdReaderDiagnostics(
    OmniPvdReader& reader, const OmniPvdReaderDiagnostics& diagnostics);
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdReader(OmniPvdReader& reader);

OMNI_PVD_API OmniPvdWriter* OMNI_PVD_CALL createOmniPvdWriter();
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdWriter(OmniPvdWriter& writer);

OMNI_PVD_API OmniPvdFileReadStream* OMNI_PVD_CALL createOmniPvdFileReadStream();
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdFileReadStream(OmniPvdFileReadStream& stream);

OMNI_PVD_API OmniPvdFileWriteStream* OMNI_PVD_CALL createOmniPvdFileWriteStream();
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdFileWriteStream(OmniPvdFileWriteStream& stream);

OMNI_PVD_API OmniPvdMemoryStream* OMNI_PVD_CALL createOmniPvdMemoryStream();
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdMemoryStream(OmniPvdMemoryStream& stream);

// Creates a socket read stream (the reader-side TCP server). port: the TCP port to listen on.
OMNI_PVD_API OmniPvdSocketReadStream* OMNI_PVD_CALL createOmniPvdSocketReadStream(uint16_t port);
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdSocketReadStream(OmniPvdSocketReadStream& stream);

// Creates a socket write stream (the producer-side TCP client). address: the IP-address of the
// listening reader to connect to. port: its TCP port. sendTimeout: upper bound in milliseconds on a
// blocked send and on the total connect-attempt/retry phase. Zero leaves the blocked-send timeout at
// the OS default and uses the default 3000 millisecond connect window.
OMNI_PVD_API OmniPvdSocketWriteStream* OMNI_PVD_CALL createOmniPvdSocketWriteStream(
	const char* address, uint16_t port, uint32_t sendTimeout);
OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdSocketWriteStream(OmniPvdSocketWriteStream& stream);

#endif
