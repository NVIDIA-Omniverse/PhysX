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
// Bits 0 and 1 were used by experimental OVC1/OVC2 control, whose data masks
// and selector lists have different meanings. Do not advertise or reinterpret them.
#define OMNI_PVD_LIVE_FILTER_CONTROL 4u
#define OMNI_PVD_LIVE_CAPTURE_CATALOG 8u

// Reverse direction only: little-endian OVC3, a 28-byte header followed by U64 handles.
// Header: magic(U32), byte length(U32), request ID(U64), filter flags(U32),
// scene count(U32), object count(U32). Scene handles precede object handles.
// Filter bit 0 records secondary data. Other bits are ignored. The public boolean
// maps to this fixed-width field without changing the negotiated wire layout.
// Handshake bit 2 alone permits these primary/secondary filter semantics. Older peers
// remain usable for forward streaming and receive no unsupported reverse messages.
// Forward bytes remain the existing OmniPVD command stream. Bound memory and work per poll.
static const uint32_t OMNI_PVD_CAPTURE_MAGIC = 0x3343564fu; // 'OVC3'
// Bit 3 separately permits OVD3 catalog requests. They use the same 28-byte header,
// with a nonzero request ID and zero flags/counts; no selector payload follows.
static const uint32_t OMNI_PVD_CAPTURE_CATALOG_MAGIC = 0x3344564fu; // 'OVD3'
static const uint32_t OMNI_PVD_CAPTURE_HEADER_BYTES = 28;
static const uint32_t OMNI_PVD_CAPTURE_MAX_SELECTORS = 65536;
static const uint32_t OMNI_PVD_CAPTURE_MAX_BYTES = OMNI_PVD_CAPTURE_HEADER_BYTES + 8 * OMNI_PVD_CAPTURE_MAX_SELECTORS;

inline void omniPvdEncodeControlInteger(uint8_t* bytes, uint64_t value, uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i) bytes[i] = static_cast<uint8_t>(value >> (8 * i));
}
inline uint64_t omniPvdDecodeControlInteger(const uint8_t* bytes, uint32_t count)
{
	uint64_t value = 0;
	for (uint32_t i = 0; i < count; ++i) value |= static_cast<uint64_t>(bytes[i]) << (8 * i);
	return value;
}

#pragma pack(push, 1)
struct OmniPvdSocketStreamHandshake
{
	OmniPvdSocketStreamHandshake()
	{
		magic = OMNI_PVD_HANDSHAKE_MAGIC;
		proto = OMNI_PVD_LIVE_PROTO;
		flags = 0; // no receiver control capability by default
	}

	uint32_t magic;    // OMNI_PVD_HANDSHAKE_MAGIC
	uint16_t proto;    // OMNI_PVD_LIVE_PROTO
	uint16_t flags;    // OMNI_PVD_LIVE_FILTER_CONTROL / OMNI_PVD_LIVE_CAPTURE_CATALOG; unknown bits ignored
};
#pragma pack(pop)

#endif
