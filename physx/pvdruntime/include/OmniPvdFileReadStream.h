// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_FILE_READ_STREAM_H
#define OMNI_PVD_FILE_READ_STREAM_H

#include "OmniPvdReadStream.h"

/**
 * \brief Used to abstract a file read stream
 *
 * Used to set the filename, opening and closing it.
 */
class OmniPvdFileReadStream : public OmniPvdReadStream
{
public:
	virtual ~OmniPvdFileReadStream()
	{
	}

	/**
	 * \brief Sets the file name of the file to read from
	 *
	 * \param fileName The file name of the file to open
	 */
	virtual void OMNI_PVD_CALL setFileName(const char* fileName) = 0;

	/**
	 * \brief Opens the file
	 *
	 * \return True if the file opening was successfull
	 */
	virtual bool OMNI_PVD_CALL openFile() = 0;

	/**
	 * \brief Closes the file
	 *
	 * \return True if the file closing was successfull
	 */
	virtual bool OMNI_PVD_CALL closeFile() = 0;
};

#endif
