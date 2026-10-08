// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_FILE_WRITE_STREAM_H
#define OMNI_PVD_FILE_WRITE_STREAM_H

#include "OmniPvdWriteStream.h"

/**
 * \brief A file-backed OmniPVD write stream.
 *
 * Each closed-to-open transition opens the configured file in truncating write mode and starts
 * at byte zero. Calling openStream() while the stream is already open is a non-destructive no-op:
 * it neither truncates the file nor resets the current position. A file name set while open takes
 * effect on the next closed-to-open transition; it does not retarget the active file handle.
 *
 * Closing an open file finalizes buffered output. closeStream() returns false if that final flush
 * or close fails, but the stream has still transitioned to closed and buffered bytes may have been
 * lost. Callers should close explicitly and check the result rather than rely on destructor cleanup.
 */
class OmniPvdFileWriteStream : public OmniPvdWriteStream
{
public:
	virtual ~OmniPvdFileWriteStream()
	{
	}

	/**
	 * \brief Sets the file name used by the next closed-to-open transition.
	 *
	 * Changing the name while open leaves the active file unchanged until closeStream() followed by
	 * openStream(). That later open truncates the newly configured file and starts at byte zero.
	 *
	 * \param fileName The file name of the file to open.
	 */
	virtual void OMNI_PVD_CALL setFileName(const char* fileName) = 0;
};

#endif
