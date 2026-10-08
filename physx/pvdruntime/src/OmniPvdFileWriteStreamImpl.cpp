// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdFileWriteStreamImpl.h"

OmniPvdFileWriteStreamImpl::OmniPvdFileWriteStreamImpl()
{
	mFileName = 0;
	mPFile = 0;
}

OmniPvdFileWriteStreamImpl::~OmniPvdFileWriteStreamImpl()
{
	closeStream();
	delete[] mFileName;
	mFileName = 0;
}

void OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::setFileName(const char* fileName)
{
	if (!fileName) return;
	const size_t maxLen = 4096; // Some reasonable filename cap
	size_t len = strnlen(fileName, maxLen);
	if (len == 0) return;
	delete[] mFileName;
	// +1 for null terminator
	mFileName = new char[len + 1];
	memcpy(mFileName, fileName, len);
	mFileName[len] = '\0';
}

uint64_t OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::writeBytes(const uint8_t *bytes, uint64_t nbrBytes)
{
	size_t result = 0;
	if (mPFile!=0)
	{
		result = fwrite(bytes, 1, nbrBytes, mPFile);
	}
	return result;
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::flush()
{
	return mPFile && fflush(mPFile) == 0;
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::openStream()
{
	if (mPFile)
		return true;
	if (!mFileName)
		return false;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
	FILE* file = 0;
	if (fopen_s(&file, mFileName, "wb") != 0)
		return false;
	mPFile = file;
#else
	mPFile = fopen(mFileName, "wb");
#endif
	return mPFile != 0;
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::closeStream()
{
	if (!mPFile)
		return true;
	FILE* file = mPFile;
	mPFile = 0;
	return fclose(file) == 0;
}
