// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_MEMORY_READ_STREAM_IMPL_H
#define OMNI_PVD_MEMORY_READ_STREAM_IMPL_H

#include "OmniPvdReadStream.h"

class OmniPvdMemoryStreamImpl;

class OmniPvdMemoryReadStreamImpl : public OmniPvdReadStream
{
public:
	OmniPvdMemoryReadStreamImpl();
	~OmniPvdMemoryReadStreamImpl();

	uint64_t OMNI_PVD_CALL readBytes(uint8_t* destination, uint64_t nbrBytes);
	uint64_t OMNI_PVD_CALL skipBytes(uint64_t nbrBytes);
	bool OMNI_PVD_CALL openStream();
	bool OMNI_PVD_CALL closeStream();
	bool isOpen() const;

	OmniPvdMemoryStreamImpl* mMemoryStream;
	bool mIsOpen;
};

#endif
