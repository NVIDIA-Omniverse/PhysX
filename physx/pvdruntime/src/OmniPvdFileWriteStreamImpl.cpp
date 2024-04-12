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

#include "OmniPvdFileWriteStreamImpl.h"
#include <cstring> 

OmniPvdFileWriteStreamImpl::OmniPvdFileWriteStreamImpl() :
    mFileName(nullptr),
    mFileWasOpened(false),
    mPFile(nullptr)
{
}

OmniPvdFileWriteStreamImpl::~OmniPvdFileWriteStreamImpl()
{
    closeFile(); 
    delete[] mFileName;
}

void OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::setFileName(const char* fileName)
{
    if (!fileName || strlen(fileName) < 2) 
        return;
    
    delete[] mFileName; 
    size_t n = strlen(fileName) + 1;
    mFileName = new char[n];
    strcpy(mFileName, fileName);
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::openFile()
{
    if (mFileWasOpened || !mFileName) 
        return mFileWasOpened;
    
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    errno_t err = fopen_s(&mPFile, mFileName, "wb");
    mFileWasOpened = (err == 0);
#else
    mPFile = fopen(mFileName, "wb");
    mFileWasOpened = (mPFile != nullptr);
#endif

    return mFileWasOpened;
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::closeFile()
{
    if (mFileWasOpened && mPFile) 
    {
        fclose(mPFile);
        mPFile = nullptr; 
        mFileWasOpened = false;
        return true;
    }
    return false;
}

uint64_t OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::writeBytes(const uint8_t *bytes, uint64_t nbrBytes)
{
    if (mFileWasOpened && mPFile)
    {
        size_t result = fwrite(bytes, 1, nbrBytes, mPFile);
        return result; 
    }
    return 0;
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::flush()
{
    if (mFileWasOpened && mPFile)
    {
        fflush(mPFile); 
        return true;
    }
    return false;
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::openStream()
{
    return openFile();
}

bool OMNI_PVD_CALL OmniPvdFileWriteStreamImpl::closeStream()
{
    return closeFile();
}
