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
