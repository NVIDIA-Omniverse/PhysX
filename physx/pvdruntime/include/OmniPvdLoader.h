// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
