// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_FILE_READ_STREAM_H
#define OMNI_PVD_FILE_READ_STREAM_H

#include "OmniPvdReadStream.h"

/**
 * \brief A file-backed OmniPVD read stream.
 *
 * Each closed-to-open transition opens the configured file at byte zero. Calling openStream()
 * while the stream is already open is a non-destructive no-op that preserves the current read
 * position. A file name set while open takes effect on the next closed-to-open transition; it
 * does not retarget the active file handle.
 */
class OmniPvdFileReadStream : public OmniPvdReadStream
{
public:
	virtual ~OmniPvdFileReadStream()
	{
	}

	/**
	 * \brief Sets the file name used by the next closed-to-open transition.
	 *
	 * Changing the name while open leaves the active file unchanged until closeStream() followed by
	 * openStream(). That later open starts reading the newly configured file at byte zero.
	 *
	 * \param fileName The file name of the file to open.
	 */
	virtual void OMNI_PVD_CALL setFileName(const char* fileName) = 0;
};

#endif
