// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_SOCKET_WRITE_STREAM_IMPL_H
#define OMNI_PVD_SOCKET_WRITE_STREAM_IMPL_H

#include "OmniPvdSocketWriteStream.h"
#include "OmniPvdSocket.h"

//
// Streams the OmniPVD command stream over a TCP connection. This stream is the TCP
// client: openStream() connects to the listening reader (endpoint fixed at creation),
// preceded by an OmniPvdSocketStreamHandshake.
//
// writeBytes does a direct blocking send() of the full payload while the OmniPvdWriter
// holds its exclusive lock; it returns exactly nbrBytes on success and a short count on a
// dead peer, which trips the writer's existing eSTREAM_WRITE_FAILURE path (matching the
// file stream's all-or-nothing fwrite).
//
class OmniPvdSocketWriteStreamImpl : public OmniPvdSocketWriteStream
{
public:
	OmniPvdSocketWriteStreamImpl(const char* address, uint16_t port, uint32_t sendTimeout);
	~OmniPvdSocketWriteStreamImpl();

	uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* bytes, uint64_t nbrBytes) override;
	bool OMNI_PVD_CALL flush() override;
	bool OMNI_PVD_CALL openStream() override;
	bool OMNI_PVD_CALL closeStream() override;

private:
	bool sendAll(const uint8_t* bytes, uint64_t nbrBytes);
	bool sendHandshake();

	OmniPvdSocket mSocket;
	char*         mAddress;        // listening reader address (fixed at creation)
	uint16_t      mPort;           // listening reader port (fixed at creation)
	uint32_t      mSendTimeoutMs;  // upper bound on a blocked send, in ms (fixed at creation)
	bool          mDead;           // a send failed/timed out; subsequent writes short-count
};

#endif
