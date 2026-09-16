// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdMemoryStreamImpl.h"
#include "OmniPvdMemoryWriteStreamImpl.h"

OmniPvdMemoryWriteStreamImpl::OmniPvdMemoryWriteStreamImpl() : mMemoryStream(0), mIsOpen(false)
{
}

OmniPvdMemoryWriteStreamImpl::~OmniPvdMemoryWriteStreamImpl()
{
	closeStream();
}

uint64_t OMNI_PVD_CALL OmniPvdMemoryWriteStreamImpl::writeBytes(const uint8_t* source, uint64_t nbrBytes)
{
	return mIsOpen ? mMemoryStream->writeBytes(source, nbrBytes) : 0;
}

bool OMNI_PVD_CALL OmniPvdMemoryWriteStreamImpl::flush()
{
	return mIsOpen && mMemoryStream->flush();
}

bool OMNI_PVD_CALL OmniPvdMemoryWriteStreamImpl::openStream()
{
	if (mIsOpen)
		return true;
	if (!mMemoryStream->hasBuffer())
		return false;
	mIsOpen = true;
	return true;
}

bool OMNI_PVD_CALL OmniPvdMemoryWriteStreamImpl::closeStream()
{
	mIsOpen = false;
	return true;
}

bool OmniPvdMemoryWriteStreamImpl::isOpen() const
{
	return mIsOpen;
}
