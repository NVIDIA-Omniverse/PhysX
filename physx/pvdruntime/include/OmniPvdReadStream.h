// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_READ_STREAM_H
#define OMNI_PVD_READ_STREAM_H

#include "OmniPvdDefines.h"

/**
 * \brief Used to abstract a memory read stream
 *
 * Allows to read and skip bytes as well as open/close it.
 */
class OmniPvdReadStream
{
public:
	virtual ~OmniPvdReadStream()
	{
	}

	/**
	 * \brief Read n bytes from the shared memory buffer
	 *
	 * \param bytes Reads n bytes into the destination pointer
	 * \param nbrBytes The requested number of bytes to read
	 * \return The actual number of bytes read
	 */
	virtual uint64_t OMNI_PVD_CALL readBytes(uint8_t* bytes, uint64_t nbrBytes) = 0;

	/**
	 * \brief Skip n bytes from the shared memory buffer
	 *
	 * \param nbrBytes The requested number of bytes to skip
	 * \return The actual number of bytes skipped
	 */
	virtual uint64_t OMNI_PVD_CALL skipBytes(uint64_t nbrBytes) = 0;
	
	/**
	 * \brief Opens the read stream
	 *
	 * \return True if it succeeded
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the read stream
	 *
	 * \return True if it succeeded
	 */
	virtual bool OMNI_PVD_CALL closeStream() = 0;
};

#endif
