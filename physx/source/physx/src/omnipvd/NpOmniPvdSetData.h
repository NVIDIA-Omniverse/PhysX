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

#ifndef NP_OMNI_PVD_SET_DATA_H
#define NP_OMNI_PVD_SET_DATA_H


//
// This header has to be included to use macros like OMNI_PVD_SET() (set attribute values,
// object instance registration etc.)
//


#define OMNI_PVD_CONTEXT_HANDLE 1


#if PX_SUPPORT_OMNI_PVD

#include "NpOmniPvdRegistrationData.h"
#include "OmniPvdPxSampler.h"
#include "NpOmniPvd.h"


#define OMNI_PVD_ACTIVE (::OmniPvdPxSampler::getInstance() != NULL)


//
// Define the macros needed in CmOmniPvdAutoGenSetData.h
//
#undef OMNI_PVD_GET_WRITER
#define OMNI_PVD_GET_WRITER(writer) \
physx::PxOmniPvd::ScopedExclusiveWriter writeLock(NpOmniPvdGetInstance()); \
OmniPvdWriter* writer = writeLock.getWriter();


#undef OMNI_PVD_GET_REGISTRATION_DATA
#define OMNI_PVD_GET_REGISTRATION_DATA(registrationData) \
const OmniPvdPxCoreRegistrationData* registrationData = NpOmniPvdGetPxCoreRegistrationData();


#else  // PX_SUPPORT_OMNI_PVD


#define OMNI_PVD_ACTIVE (false)


#endif  // PX_SUPPORT_OMNI_PVD


#include "omnipvd/CmOmniPvdAutoGenSetData.h"
// note: included in all cases since it will provide empty definitions of the helper macros such
//       that not all of them have to be guarded by PX_SUPPORT_OMNI_PVD


#endif  // NP_OMNI_PVD_SET_DATA_H
