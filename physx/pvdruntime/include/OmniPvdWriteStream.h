// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_WRITE_STREAM_H
#define OMNI_PVD_WRITE_STREAM_H

#include "OmniPvdDefines.h"

/**
 * \brief Used to abstract a memory write stream
 *
 * Allows to write bytes as well as open/close the stream.
 */
class OmniPvdWriteStream
{
public:
	virtual ~OmniPvdWriteStream()
	{
	}

	/**
	 * \brief Write n bytes to the shared memory buffer
	 *
	 * \param bytes pointer to the bytes to write
	 * \param nbrBytes The requested number of bytes to write
	 * \return The actual number of bytes written
	 */
	virtual uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* bytes, uint64_t nbrBytes) = 0;

	/**
	 * \brief Flushes the writes
	 *
	 * \return The success of the operation
	 */
	virtual bool OMNI_PVD_CALL flush() = 0;

	/**
	 * \brief Opens the stream
	 *
	 * \return The success of the operation
	 */
	virtual bool OMNI_PVD_CALL openStream() = 0;

	/**
	 * \brief Closes the stream
	 *
	 * \return The success of the operation
	 */
	virtual bool OMNI_PVD_CALL closeStream() = 0;
};

#endif
