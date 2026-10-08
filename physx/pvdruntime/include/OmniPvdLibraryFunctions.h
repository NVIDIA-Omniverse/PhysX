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
