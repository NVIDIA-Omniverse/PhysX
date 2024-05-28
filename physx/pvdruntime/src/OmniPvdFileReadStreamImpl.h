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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef OMNI_PVD_FILE_READ_STREAM_IMPL_H
#define OMNI_PVD_FILE_READ_STREAM_IMPL_H

#include "OmniPvdFileReadStream.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

class OmniPvdFileReadStreamImpl : public OmniPvdFileReadStream
{
public:
	OmniPvdFileReadStreamImpl();
	~OmniPvdFileReadStreamImpl();
	void resetFileParams();
	void OMNI_PVD_CALL setFileName(const char *fileName);
	bool OMNI_PVD_CALL openFile();
	bool OMNI_PVD_CALL closeFile();
	uint64_t OMNI_PVD_CALL readBytes(uint8_t* bytes, uint64_t nbrBytes);
	uint64_t OMNI_PVD_CALL skipBytes(uint64_t nbrBytes);
	bool OMNI_PVD_CALL openStream();
	bool OMNI_PVD_CALL closeStream();

	char* mFileName;
	bool mFileOpenAttempted;
	FILE* mPFile;
};

#endif
