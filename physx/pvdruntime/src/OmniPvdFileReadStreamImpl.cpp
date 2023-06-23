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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "OmniPvdFileReadStreamImpl.h"

OmniPvdFileReadStreamImpl::OmniPvdFileReadStreamImpl()
{
	mFileName = 0;
	mFileWasOpened = false;
	mPFile = 0;
}

OmniPvdFileReadStreamImpl::~OmniPvdFileReadStreamImpl()
{
	closeFile();
	delete[] mFileName;
	mFileName = 0;
}

void OMNI_PVD_CALL OmniPvdFileReadStreamImpl::setFileName(const char* fileName)
{
	if (!fileName) return;
	int n = (int)strlen(fileName) + 1;
	if (n < 2) return;
	delete[] mFileName;
	mFileName = new char[n];
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
	strcpy_s(mFileName, n, fileName);
#else
	strcpy(mFileName, fileName);
#endif		
	mFileName[n - 1] = 0;
}

bool OMNI_PVD_CALL OmniPvdFileReadStreamImpl::openFile()
{
	if (mFileWasOpened)
	{
		return true;
	}
	if (!mFileName)
	{
		return false;
	}
	mPFile = 0;
	mFileWasOpened = true;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
	errno_t err = fopen_s(&mPFile, mFileName, "rb");
	if (err != 0)
	{
		mFileWasOpened = false;
	}
	else
	{
		fseek(mPFile, 0, SEEK_SET);
	}
#else
	mPFile = fopen(mFileName, "rb");
	if (mPFile)
	{
		fseek(mPFile, 0, SEEK_SET);
	}
	else
	{
		mFileWasOpened = false;
	}
#endif
	return mFileWasOpened;
}

bool OMNI_PVD_CALL OmniPvdFileReadStreamImpl::closeFile()
{
	if (mFileWasOpened)
	{
		fclose(mPFile);
		mPFile = 0;
		mFileWasOpened = false;
	}
	return true;
}

uint64_t OMNI_PVD_CALL OmniPvdFileReadStreamImpl::readBytes(uint8_t* bytes, uint64_t nbrBytes)
{
	if (mFileWasOpened)
	{
		size_t result = fread(bytes, 1, nbrBytes, mPFile);
		return (int)result;
	} else {
		return 0;
	}
}

uint64_t OMNI_PVD_CALL OmniPvdFileReadStreamImpl::skipBytes(uint64_t nbrBytes)
{
	if (mFileWasOpened)
	{
		fseek(mPFile, (long)nbrBytes, SEEK_CUR);
		return 0;
	}
	else {
		return 0;
	}
}

bool OMNI_PVD_CALL OmniPvdFileReadStreamImpl::openStream()
{
	return openFile();
}

bool OMNI_PVD_CALL OmniPvdFileReadStreamImpl::closeStream()
{
	return closeFile();
}
