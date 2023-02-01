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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  
#ifndef OMNI_PVD_LIBRARY_FUNCTIONS_H
#define OMNI_PVD_LIBRARY_FUNCTIONS_H

#include "OmniPvdDefines.h"

class OmniPvdReader;
class OmniPvdWriter;
class OmniPvdFileReadStream;
class OmniPvdFileWriteStream;
class OmniPvdMemoryStream;

typedef OmniPvdReader* (OMNI_PVD_CALL *createOmniPvdReaderFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdReaderFp)(OmniPvdReader *reader);

typedef OmniPvdWriter* (OMNI_PVD_CALL *createOmniPvdWriterFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdWriterFp)(OmniPvdWriter *writer);

typedef OmniPvdFileReadStream* (OMNI_PVD_CALL *createOmniPvdFileReadStreamFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdFileReadStreamFp)(OmniPvdFileReadStream *fileReadStream);

typedef OmniPvdFileWriteStream* (OMNI_PVD_CALL *createOmniPvdFileWriteStreamFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdFileWriteStreamFp)(OmniPvdFileWriteStream *fileWriteStream);

typedef OmniPvdMemoryStream* (OMNI_PVD_CALL *createOmniPvdMemoryStreamFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdMemoryStreamFp)(OmniPvdMemoryStream *memoryStream);

#endif
