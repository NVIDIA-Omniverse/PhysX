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

#ifndef OMNI_PVD_SOCKET_PROTOCOL_H
#define OMNI_PVD_SOCKET_PROTOCOL_H

#include "OmniPvdDefines.h"
#include <stdint.h>

//
// A single fixed-size handshake struct sent once at connect time, immediately after the
// TCP connection is established and BELOW the OmniPVD byte layer (the writer and reader
// never see it). The on-wire order of a session is:
//
//   OmniPvdSocketStreamHandshake  ->  12-byte OmniPVD version header  ->  OmniPVD command stream
//
// The magic + proto guard only this socket transport's on-wire framing; OVD format and
// version compatibility is carried by the general 12-byte OmniPVD version header below,
// independent of the transport.
//
#define OMNI_PVD_HANDSHAKE_MAGIC  0x4F56444Cu // 'O''V''D''L'
#define OMNI_PVD_LIVE_PROTO       1

#pragma pack(push, 1)
struct OmniPvdSocketStreamHandshake
{
	OmniPvdSocketStreamHandshake()
	{
		magic = OMNI_PVD_HANDSHAKE_MAGIC;
		proto = OMNI_PVD_LIVE_PROTO;
		flags = 0; // reserved
	}

	uint32_t magic;    // OMNI_PVD_HANDSHAKE_MAGIC
	uint16_t proto;    // OMNI_PVD_LIVE_PROTO
	uint16_t flags;    // reserved, 0
};
#pragma pack(pop)

#endif
