// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdMemoryStreamImpl.h"
#include "OmniPvdMemoryReadStreamImpl.h"

OmniPvdMemoryReadStreamImpl::OmniPvdMemoryReadStreamImpl() : mMemoryStream(0), mIsOpen(false)
{
}

OmniPvdMemoryReadStreamImpl::~OmniPvdMemoryReadStreamImpl()
{
	closeStream();
}

uint64_t OMNI_PVD_CALL OmniPvdMemoryReadStreamImpl::readBytes(uint8_t* destination, uint64_t nbrBytes)
{
	return mIsOpen ? mMemoryStream->readBytes(destination, nbrBytes) : 0;
}

uint64_t OMNI_PVD_CALL OmniPvdMemoryReadStreamImpl::skipBytes(uint64_t nbrBytes)
{
	return mIsOpen ? mMemoryStream->skipBytes(nbrBytes) : 0;
}

bool OMNI_PVD_CALL OmniPvdMemoryReadStreamImpl::openStream()
{
	if (mIsOpen)
		return true;
	if (!mMemoryStream->hasBuffer())
		return false;
	mIsOpen = true;
	return true;
}

bool OMNI_PVD_CALL OmniPvdMemoryReadStreamImpl::closeStream()
{
	mIsOpen = false;
	return true;
}

bool OmniPvdMemoryReadStreamImpl::isOpen() const
{
	return mIsOpen;
}
