// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
