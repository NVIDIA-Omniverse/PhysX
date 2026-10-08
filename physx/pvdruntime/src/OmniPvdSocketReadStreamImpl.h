// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_SOCKET_READ_STREAM_IMPL_H
#define OMNI_PVD_SOCKET_READ_STREAM_IMPL_H

#include "OmniPvdSocketReadStream.h"
#include "OmniPvdSocket.h"

//
// Receives the OmniPVD command stream over a TCP connection. This stream is the TCP
// server: openStream() listens and accepts a producer on the port fixed at creation, then
// reads the OmniPvdSocketStreamHandshake. readBytes blocks until exactly nbrBytes arrive;
// a short/zero return only happens at a true peer close, which lands on the command
// boundary in OmniPvdReader::getNextCommand and ends the read loop like a file EOF.
//
class OmniPvdSocketReadStreamImpl : public OmniPvdSocketReadStream
{
public:
	OmniPvdSocketReadStreamImpl(uint16_t port);
	~OmniPvdSocketReadStreamImpl();

	uint64_t OMNI_PVD_CALL readBytes(uint8_t* bytes, uint64_t nbrBytes) override;
	uint64_t OMNI_PVD_CALL skipBytes(uint64_t nbrBytes) override;
	bool OMNI_PVD_CALL openStream() override;
	bool OMNI_PVD_CALL closeStream() override;

private:
	bool recvAll(uint8_t* bytes, uint64_t nbrBytes);
	bool recvAndValidateHandshake();

	OmniPvdSocket mSocket;
	uint16_t      mPort; // port to listen on (fixed at creation)
};

#endif
