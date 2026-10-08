// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
