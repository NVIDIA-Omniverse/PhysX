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
