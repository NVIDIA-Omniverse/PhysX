// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReaderImpl.h"
#include "OmniPvdWriterImpl.h"
#include "OmniPvdFileReadStreamImpl.h"
#include "OmniPvdFileWriteStreamImpl.h"
#include "OmniPvdMemoryStreamImpl.h"
#include "OmniPvdSocketWriteStreamImpl.h"
#include "OmniPvdSocketReadStreamImpl.h"

OMNI_PVD_API OmniPvdReader* OMNI_PVD_CALL createOmniPvdReader()
{
	return new OmniPvdReaderImpl();
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdReader(OmniPvdReader& reader)
{
	OmniPvdReaderImpl* impl = (OmniPvdReaderImpl*)(&reader);
	delete impl;
}

OMNI_PVD_API OmniPvdWriter* OMNI_PVD_CALL createOmniPvdWriter()
{
	return new OmniPvdWriterImpl();
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdWriter(OmniPvdWriter& writer)
{
	OmniPvdWriterImpl* impl = (OmniPvdWriterImpl*)(&writer);
	delete impl;
}

OMNI_PVD_API OmniPvdFileReadStream* OMNI_PVD_CALL createOmniPvdFileReadStream()
{
	return new OmniPvdFileReadStreamImpl();
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdFileReadStream(OmniPvdFileReadStream& readStream)
{
	OmniPvdFileReadStreamImpl* impl = (OmniPvdFileReadStreamImpl*)(&readStream);
	delete impl;
}

OMNI_PVD_API OmniPvdFileWriteStream* OMNI_PVD_CALL createOmniPvdFileWriteStream()
{
	return new OmniPvdFileWriteStreamImpl();
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdFileWriteStream(OmniPvdFileWriteStream& writeStream)
{
	OmniPvdFileWriteStreamImpl* impl = (OmniPvdFileWriteStreamImpl*)(&writeStream);
	delete impl;
}

OMNI_PVD_API OmniPvdMemoryStream* OMNI_PVD_CALL createOmniPvdMemoryStream()
{
	return new OmniPvdMemoryStreamImpl();
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdMemoryStream(OmniPvdMemoryStream& memoryStream)
{
	OmniPvdMemoryStreamImpl* impl = (OmniPvdMemoryStreamImpl*)(&memoryStream);
	delete impl;
}

OMNI_PVD_API OmniPvdSocketWriteStream* OMNI_PVD_CALL createOmniPvdSocketWriteStream(const char* address, uint16_t port, uint32_t sendTimeout)
{
	return new OmniPvdSocketWriteStreamImpl(address, port, sendTimeout);
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdSocketWriteStream(OmniPvdSocketWriteStream& writeStream)
{
	OmniPvdSocketWriteStreamImpl* impl = (OmniPvdSocketWriteStreamImpl*)(&writeStream);
	delete impl;
}

OMNI_PVD_API OmniPvdSocketReadStream* OMNI_PVD_CALL createOmniPvdSocketReadStream(uint16_t port)
{
	// The read stream is the TCP server: it listens on the given port for a producer.
	return new OmniPvdSocketReadStreamImpl(port);
}

OMNI_PVD_API void OMNI_PVD_CALL destroyOmniPvdSocketReadStream(OmniPvdSocketReadStream& readStream)
{
	OmniPvdSocketReadStreamImpl* impl = (OmniPvdSocketReadStreamImpl*)(&readStream);
	delete impl;
}
