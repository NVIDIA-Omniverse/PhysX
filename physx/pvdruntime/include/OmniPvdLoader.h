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

#ifndef OMNI_PVD_LOADER_H
#define OMNI_PVD_LOADER_H

#include "OmniPvdWriter.h"
#include "OmniPvdReader.h"
#include "OmniPvdLibraryFunctions.h"

#include <stdio.h>

#ifdef OMNI_PVD_WIN
	#ifndef _WINDOWS_ // windows already included otherwise
		#include <foundation/windows/PxWindowsInclude.h>
	#endif
#elif defined(__linux__)
	#include <dlfcn.h>
#endif

class OmniPvdLoader
{
public:
	OmniPvdLoader();
	~OmniPvdLoader();
	bool loadOmniPvd(const char *libFile);
	void unloadOmniPvd();
	void* mLibraryHandle;	
	
	createOmniPvdWriterFp mCreateOmniPvdWriter;
	destroyOmniPvdWriterFp mDestroyOmniPvdWriter;
	
	createOmniPvdReaderFp mCreateOmniPvdReader;
	destroyOmniPvdReaderFp mDestroyOmniPvdReader;
	
	createOmniPvdFileReadStreamFp mCreateOmniPvdFileReadStream;
	destroyOmniPvdFileReadStreamFp mDestroyOmniPvdFileReadStream;

	createOmniPvdFileWriteStreamFp mCreateOmniPvdFileWriteStream;
	destroyOmniPvdFileWriteStreamFp mDestroyOmniPvdFileWriteStream;

	createOmniPvdMemoryStreamFp mCreateOmniPvdMemoryStream;
	destroyOmniPvdMemoryStreamFp mDestroyOmniPvdMemoryStream;
};

inline OmniPvdLoader::OmniPvdLoader()
{
	mLibraryHandle = 0;
	
	mCreateOmniPvdWriter = 0;
	mDestroyOmniPvdWriter = 0;
	
	mCreateOmniPvdReader = 0;
	mDestroyOmniPvdReader = 0;
	
	mCreateOmniPvdFileReadStream = 0;	
	mDestroyOmniPvdFileReadStream = 0;

	mCreateOmniPvdFileWriteStream = 0;
	mDestroyOmniPvdFileWriteStream = 0;

	mCreateOmniPvdMemoryStream = 0;
	mDestroyOmniPvdMemoryStream = 0;
}

inline OmniPvdLoader::~OmniPvdLoader()
{
	unloadOmniPvd();
}

inline bool OmniPvdLoader::loadOmniPvd(const char *libFile)
{

#ifdef OMNI_PVD_WIN
	mLibraryHandle = LoadLibraryA(libFile);
#elif defined(__linux__)
	mLibraryHandle = dlopen(libFile, RTLD_NOW);
#endif

	if (mLibraryHandle)
	{
#ifdef OMNI_PVD_WIN
		mCreateOmniPvdWriter = (createOmniPvdWriterFp)GetProcAddress((HINSTANCE)mLibraryHandle, "createOmniPvdWriter");
		mDestroyOmniPvdWriter = (destroyOmniPvdWriterFp)GetProcAddress((HINSTANCE)mLibraryHandle, "destroyOmniPvdWriter");

		mCreateOmniPvdReader = (createOmniPvdReaderFp)GetProcAddress((HINSTANCE)mLibraryHandle, "createOmniPvdReader");
		mDestroyOmniPvdReader = (destroyOmniPvdReaderFp)GetProcAddress((HINSTANCE)mLibraryHandle, "destroyOmniPvdReader");

		mCreateOmniPvdFileReadStream = (createOmniPvdFileReadStreamFp)GetProcAddress((HINSTANCE)mLibraryHandle, "createOmniPvdFileReadStream");
		mDestroyOmniPvdFileReadStream = (destroyOmniPvdFileReadStreamFp)GetProcAddress((HINSTANCE)mLibraryHandle, "destroyOmniPvdFileReadStream");
		
		mCreateOmniPvdFileWriteStream = (createOmniPvdFileWriteStreamFp)GetProcAddress((HINSTANCE)mLibraryHandle, "createOmniPvdFileWriteStream");
		mDestroyOmniPvdFileWriteStream = (destroyOmniPvdFileWriteStreamFp)GetProcAddress((HINSTANCE)mLibraryHandle, "destroyOmniPvdFileWriteStream");

		mCreateOmniPvdMemoryStream = (createOmniPvdMemoryStreamFp)GetProcAddress((HINSTANCE)mLibraryHandle, "createOmniPvdMemoryStream");
		mDestroyOmniPvdMemoryStream = (destroyOmniPvdMemoryStreamFp)GetProcAddress((HINSTANCE)mLibraryHandle, "destroyOmniPvdMemoryStream");
#elif defined(__linux__)
		mCreateOmniPvdWriter = (createOmniPvdWriterFp)dlsym(mLibraryHandle, "createOmniPvdWriter");
		mDestroyOmniPvdWriter = (destroyOmniPvdWriterFp)dlsym(mLibraryHandle, "destroyOmniPvdWriter");

		mCreateOmniPvdReader = (createOmniPvdReaderFp)dlsym(mLibraryHandle, "createOmniPvdReader");
		mDestroyOmniPvdReader = (destroyOmniPvdReaderFp)dlsym(mLibraryHandle, "destroyOmniPvdReader");

		mCreateOmniPvdFileReadStream = (createOmniPvdFileReadStreamFp)dlsym(mLibraryHandle, "createOmniPvdFileReadStream");
		mDestroyOmniPvdFileReadStream = (destroyOmniPvdFileReadStreamFp)dlsym(mLibraryHandle, "destroyOmniPvdFileReadStream");

		mCreateOmniPvdFileWriteStream = (createOmniPvdFileWriteStreamFp)dlsym(mLibraryHandle, "createOmniPvdFileWriteStream");
		mDestroyOmniPvdFileWriteStream = (destroyOmniPvdFileWriteStreamFp)dlsym(mLibraryHandle, "destroyOmniPvdFileWriteStream");

		mCreateOmniPvdMemoryStream = (createOmniPvdMemoryStreamFp)dlsym(mLibraryHandle, "createOmniPvdMemoryStream");
		mDestroyOmniPvdMemoryStream = (destroyOmniPvdMemoryStreamFp)dlsym(mLibraryHandle, "destroyOmniPvdMemoryStream");

#endif

		if ((!mCreateOmniPvdWriter)           ||
			(!mDestroyOmniPvdWriter)          ||
			(!mCreateOmniPvdReader)           ||
			(!mDestroyOmniPvdReader)          ||
			(!mCreateOmniPvdFileReadStream)   ||
			(!mDestroyOmniPvdFileReadStream)  ||
			(!mCreateOmniPvdFileWriteStream)  ||
			(!mDestroyOmniPvdFileWriteStream) ||
			(!mCreateOmniPvdMemoryStream)     ||
			(!mDestroyOmniPvdMemoryStream)
			)
		{
#ifdef OMNI_PVD_WIN
			FreeLibrary((HINSTANCE)mLibraryHandle);
#elif defined(__linux__)
			dlclose(mLibraryHandle);
#endif		
			mLibraryHandle = 0;
			return false;
		}
	}
	else {
		return false;
	}
	return true;
}

inline void OmniPvdLoader::unloadOmniPvd()
{
	if (mLibraryHandle)
	{
#ifdef OMNI_PVD_WIN
		FreeLibrary((HINSTANCE)mLibraryHandle);
#elif defined(__linux__)
		dlclose(mLibraryHandle);
#endif		
		mLibraryHandle = 0;
	}
}

#endif
