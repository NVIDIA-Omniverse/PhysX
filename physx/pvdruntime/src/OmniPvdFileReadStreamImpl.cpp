// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdFileReadStreamImpl.h"

OmniPvdFileReadStreamImpl::OmniPvdFileReadStreamImpl()
{
	mFileName = 0;
	mPFile = 0;
}

OmniPvdFileReadStreamImpl::~OmniPvdFileReadStreamImpl()
{
	closeStream();
	delete[] mFileName;
	mFileName = 0;
}

void OMNI_PVD_CALL OmniPvdFileReadStreamImpl::setFileName(const char* fileName)
{
	if (!fileName) return;
	const size_t maxLen = 4096; // Some reasonable file name cap
	size_t len = strnlen(fileName, maxLen);
	if (len == 0) return;
	delete[] mFileName;
	// +1 for null terminator
	mFileName = new char[len + 1];
	memcpy(mFileName, fileName, len);
	mFileName[len] = '\0';
}

uint64_t OMNI_PVD_CALL OmniPvdFileReadStreamImpl::readBytes(uint8_t* bytes, uint64_t nbrBytes)
{
	size_t result = 0;
	if (mPFile!=0)
	{
		result = fread(bytes, 1, nbrBytes, mPFile);
	}
	return result;
}

uint64_t OMNI_PVD_CALL OmniPvdFileReadStreamImpl::skipBytes(uint64_t nbrBytes)
{
	if (mPFile==0)
	{
		return 0;
	}
	if (fseek(mPFile, (long)nbrBytes, SEEK_CUR)==0)
	{
		return nbrBytes;
	}
	return 0;
}

bool OMNI_PVD_CALL OmniPvdFileReadStreamImpl::openStream()
{
	if (mPFile)
		return true;
	if (!mFileName)
		return false;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
	FILE* file = 0;
	if (fopen_s(&file, mFileName, "rb") != 0)
		return false;
	mPFile = file;
#else
	mPFile = fopen(mFileName, "rb");
#endif
	return mPFile != 0;
}

bool OMNI_PVD_CALL OmniPvdFileReadStreamImpl::closeStream()
{
	if (mPFile)
	{
		fclose(mPFile);
		mPFile = 0;
	}
	return true;
}
