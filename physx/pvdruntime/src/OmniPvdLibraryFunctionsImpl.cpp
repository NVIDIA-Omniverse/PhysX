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

#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReaderImpl.h"
#include "OmniPvdWriterImpl.h"
#include "OmniPvdFileReadStreamImpl.h"
#include "OmniPvdFileWriteStreamImpl.h"
#include "OmniPvdMemoryStreamImpl.h"

OMNI_PVD_EXPORT OmniPvdReader* OMNI_PVD_CALL createOmniPvdReader()
{
	return new OmniPvdReaderImpl();
}

OMNI_PVD_EXPORT void OMNI_PVD_CALL destroyOmniPvdReader(OmniPvdReader& reader)
{
	OmniPvdReaderImpl* impl = (OmniPvdReaderImpl*)(&reader);
	delete impl;
}

OMNI_PVD_EXPORT OmniPvdWriter* OMNI_PVD_CALL createOmniPvdWriter()
{
	return new OmniPvdWriterImpl();
}

OMNI_PVD_EXPORT void OMNI_PVD_CALL destroyOmniPvdWriter(OmniPvdWriter& writer)
{
	OmniPvdWriterImpl* impl = (OmniPvdWriterImpl*)(&writer);
	delete impl;
}

OMNI_PVD_EXPORT OmniPvdFileReadStream* OMNI_PVD_CALL createOmniPvdFileReadStream()
{
	return new OmniPvdFileReadStreamImpl();
}

OMNI_PVD_EXPORT void OMNI_PVD_CALL destroyOmniPvdFileReadStream(OmniPvdFileReadStream& readStream)
{
	OmniPvdFileReadStreamImpl* impl = (OmniPvdFileReadStreamImpl*)(&readStream);
	delete impl;
}

OMNI_PVD_EXPORT OmniPvdFileWriteStream* OMNI_PVD_CALL createOmniPvdFileWriteStream()
{
	return new OmniPvdFileWriteStreamImpl();
}

OMNI_PVD_EXPORT void OMNI_PVD_CALL destroyOmniPvdFileWriteStream(OmniPvdFileWriteStream& writeStream)
{
	OmniPvdFileWriteStreamImpl* impl = (OmniPvdFileWriteStreamImpl*)(&writeStream);
	delete impl;
}

OMNI_PVD_EXPORT OmniPvdMemoryStream* OMNI_PVD_CALL createOmniPvdMemoryStream()
{
	return new OmniPvdMemoryStreamImpl();
}

OMNI_PVD_EXPORT void OMNI_PVD_CALL destroyOmniPvdMemoryStream(OmniPvdMemoryStream& memoryStream)
{
	OmniPvdMemoryStreamImpl* impl = (OmniPvdMemoryStreamImpl*)(&memoryStream);
	delete impl;
}
