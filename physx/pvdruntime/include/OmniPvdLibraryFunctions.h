// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_LIBRARY_FUNCTIONS_H
#define OMNI_PVD_LIBRARY_FUNCTIONS_H

#include "OmniPvdDefines.h"

class OmniPvdReader;
class OmniPvdWriter;
class OmniPvdFileReadStream;
class OmniPvdFileWriteStream;
class OmniPvdMemoryStream;

typedef OmniPvdReader* (OMNI_PVD_CALL *createOmniPvdReaderFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdReaderFp)(OmniPvdReader& reader);

typedef OmniPvdWriter* (OMNI_PVD_CALL *createOmniPvdWriterFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdWriterFp)(OmniPvdWriter& writer);

typedef OmniPvdFileReadStream* (OMNI_PVD_CALL *createOmniPvdFileReadStreamFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdFileReadStreamFp)(OmniPvdFileReadStream& fileReadStream);

typedef OmniPvdFileWriteStream* (OMNI_PVD_CALL *createOmniPvdFileWriteStreamFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdFileWriteStreamFp)(OmniPvdFileWriteStream& fileWriteStream);

typedef OmniPvdMemoryStream* (OMNI_PVD_CALL *createOmniPvdMemoryStreamFp)();
typedef void (OMNI_PVD_CALL *destroyOmniPvdMemoryStreamFp)(OmniPvdMemoryStream& memoryStream);

#endif
