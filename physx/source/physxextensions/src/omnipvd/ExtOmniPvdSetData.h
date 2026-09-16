// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_OMNI_PVD_SET_DATA_H
#define EXT_OMNI_PVD_SET_DATA_H


//
// This header has to be included to use macros like OMNI_PVD_SET() (set attribute values,
// object instance registration etc.)
//


#define OMNI_PVD_CONTEXT_HANDLE 1


#if PX_SUPPORT_OMNI_PVD

#include "OmniPvdPxExtensionsSampler.h"
#include "omnipvd/PxOmniPvd.h"

//
// Define the macros needed in CmOmniPvdAutoGenSetData.h
//
// The write scope yields a NULL writer when not sampling, so attribute setters do not write between
// stopSampling() and the next startSampling(). The gate uses the public PxOmniPvd::isSampling() so
// PhysXExtensions depends only on the public OVD API, not on PhysX core internals.
#undef OMNI_PVD_GET_WRITER
#define OMNI_PVD_GET_WRITER(writer) \
physx::PxOmniPvd* omniPvdInstance_ = physx::Ext::OmniPvdGetInstance(); \
physx::PxOmniPvd::ScopedExclusiveWriter writeLock((omniPvdInstance_ && omniPvdInstance_->isSampling()) ? omniPvdInstance_ : NULL); \
OmniPvdWriter* writer = writeLock.getWriter();


#undef OMNI_PVD_GET_REGISTRATION_DATA
#define OMNI_PVD_GET_REGISTRATION_DATA(registrationData) \
const OmniPvdPxExtensionsRegistrationData* registrationData = physx::Ext::OmniPvdGetPxExtensionsRegistrationData();

#endif  // PX_SUPPORT_OMNI_PVD


#include "omnipvd/CmOmniPvdAutoGenSetData.h"
// note: included in all cases since it will provide empty definitions of the helper macros such
//       that not all of them have to be guarded by PX_SUPPORT_OMNI_PVD


#endif  // EXT_OMNI_PVD_SET_DATA_H
