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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_DEFAULT_STREAMS_H
#define PX_DEFAULT_STREAMS_H

#include <stdio.h>
#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxIO.h"
#include "foundation/PxFoundation.h"

typedef FILE* PxFileHandle;

#if !PX_DOXYGEN
namespace physx
{
#endif

/** 
\brief default implementation of a memory write stream

\see PxOutputStream
*/

class PxDefaultMemoryOutputStream: public PxOutputStream
{
public:
						PxDefaultMemoryOutputStream(PxAllocatorCallback &allocator = *PxGetAllocatorCallback());
	virtual				~PxDefaultMemoryOutputStream();

	virtual	PxU64		write(const void* src, PxU64 count);

	virtual	PxU64		getSize()	const	{	return mSize; }
	virtual	PxU8*		getData()	const	{	return mData; }

private:
		PxDefaultMemoryOutputStream(const PxDefaultMemoryOutputStream&);
		PxDefaultMemoryOutputStream& operator=(const PxDefaultMemoryOutputStream&);

		PxAllocatorCallback&	mAllocator;
		PxU8*					mData;
		PxU64					mSize;
		PxU64					mCapacity;
};

/** 
\brief default implementation of a memory read stream

\see PxInputData
*/
	
class PxDefaultMemoryInputData: public PxInputData
{
public:
						PxDefaultMemoryInputData(const PxU8* data, PxU64 length);

	virtual		PxU64	read(void* dest, PxU64 count);
	virtual		PxU64	getLength() const;
	virtual		void	seek(PxU64 pos);
	virtual		PxU64	tell() const;

private:
		PxU64		mSize;
		const PxU8*	mData;
		PxU64		mPos;
};



/** 
\brief default implementation of a file write stream

\see PxOutputStream
*/

class PxDefaultFileOutputStream: public PxOutputStream
{
public:
						PxDefaultFileOutputStream(const char* name);
	virtual				~PxDefaultFileOutputStream();

	virtual		PxU64	write(const void* src, PxU64 count);
	virtual		bool	isValid();
private:
		PxFileHandle	mFile;
};


/** 
\brief default implementation of a file read stream

\see PxInputData
*/

class PxDefaultFileInputData: public PxInputData
{
public:
						PxDefaultFileInputData(const char* name);
	virtual				~PxDefaultFileInputData();

	virtual		PxU64	read(void* dest, PxU64 count);
	virtual		void	seek(PxU64 pos);
	virtual		PxU64	tell() const;
	virtual		PxU64	getLength() const;
				
				bool	isValid() const;
private:
		PxFileHandle	mFile;
		PxU64			mLength;
};

#if !PX_DOXYGEN
}
#endif


#endif

