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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PhysXIndicator.h"
#include "nvPhysXtoDrv.h"

#include "foundation/windows/PxWindowsInclude.h"

#include <stdio.h>

#if _MSC_VER >= 1800
#include <VersionHelpers.h>
#endif

physx::PhysXIndicator::PhysXIndicator(bool isGpu)
: mPhysXDataPtr(0), mFileHandle(0), mIsGpu(isGpu)
{
// Get the windows version (we can only create Global\ namespace objects in XP)
char configName[128];
#if _WIN32_WINNT >= _WIN32_WINNT_VISTA
NvPhysXToDrv_Build_SectionName(GetCurrentProcessId(), configName);
#else
NvPhysXToDrv_Build_SectionNameXP(GetCurrentProcessId(), configName);
#endif

mFileHandle = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL,
PAGE_READWRITE, 0, sizeof(NvPhysXToDrv_Data_V1), configName);
if (!mFileHandle || mFileHandle == INVALID_HANDLE_VALUE)
return;

bool alreadyExists = ERROR_ALREADY_EXISTS == GetLastError();

mPhysXDataPtr = (physx::NvPhysXToDrv_Data_V1*)MapViewOfFile(mFileHandle,
FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, sizeof(NvPhysXToDrv_Data_V1));
if (!mPhysXDataPtr)
return;

if (!alreadyExists)
{
mPhysXDataPtr->bCpuPhysicsPresent = 0;
mPhysXDataPtr->bGpuPhysicsPresent = 0;
}

updateCounter(1);

// init header last to prevent race conditions
// this must be done because the driver may have already created the shared memory block,
// thus alreadyExists may be true, even if PhysX hasn't been initialized
NvPhysXToDrv_Header_Init(mPhysXDataPtr->header);
}

physx::PhysXIndicator::~PhysXIndicator()
{
if (mPhysXDataPtr)
{
updateCounter(-1);
UnmapViewOfFile(mPhysXDataPtr);
}

if (mFileHandle && mFileHandle != INVALID_HANDLE_VALUE)
CloseHandle(mFileHandle);
}

void physx::PhysXIndicator::setIsGpu(bool isGpu)
{
if (!mPhysXDataPtr)
return;

updateCounter(-1);
mIsGpu = isGpu;
updateCounter(1);
}

void physx::PhysXIndicator::updateCounter(int delta)
{
int& counter = mIsGpu ? mPhysXDataPtr->bGpuPhysicsPresent : mPhysXDataPtr->bCpuPhysicsPresent;
counter += delta;
}
