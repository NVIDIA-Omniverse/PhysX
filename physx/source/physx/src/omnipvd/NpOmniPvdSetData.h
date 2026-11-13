// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_OMNI_PVD_SET_DATA_H
#define NP_OMNI_PVD_SET_DATA_H


//
// This header has to be included to use macros like OMNI_PVD_SET() (set attribute values,
// object instance registration etc.)
//


#define OMNI_PVD_CONTEXT_HANDLE 1
#define OMNI_PVD_CAPTURE_CATALOG_CONTEXT 2


#if PX_SUPPORT_OMNI_PVD

#include "NpOmniPvdRegistrationData.h"
#include "OmniPvdPxSampler.h"
#include "NpOmniPvd.h"


// "Active" means a recording is in progress: OVD is set up AND sampling. No PhysX operation should
// write OVD data when not sampling (before startSampling() or after stopSampling()).
#define OMNI_PVD_ACTIVE (physx::NpOmniPvdSampling())

#undef OMNI_PVD_SHOULD_RECORD_CREATION
#undef OMNI_PVD_SHOULD_RECORD_DESTRUCTION
#undef OMNI_PVD_SHOULD_RECORD_VALUE
#undef OMNI_PVD_SHOULD_RECORD_ADDITION
#undef OMNI_PVD_SHOULD_RECORD_REMOVAL
#define OMNI_PVD_SHOULD_RECORD_CREATION(writer, regData, classID, objectRef) \
physx::NpOmniPvdShouldRecordCreation(#classID, regData->pvd##classID.getObjectHandle(objectRef))
#define OMNI_PVD_SHOULD_RECORD_DESTRUCTION(writer, regData, classID, objectRef) \
physx::NpOmniPvdShouldRecordDestruction(#classID, regData->pvd##classID.getObjectHandle(objectRef))
#define OMNI_PVD_SHOULD_RECORD_VALUE(writer, regData, classID, attributeID, objectRef) \
physx::NpOmniPvdShouldRecordValue(writer, regData, #classID, #attributeID, regData->pvd##classID.getObjectHandle(objectRef))
#define OMNI_PVD_SHOULD_RECORD_ADDITION(writer, regData, classID, attributeID, objectRef, valueRef) \
physx::NpOmniPvdShouldRecordAddition(writer, regData, #classID, #attributeID, regData->pvd##classID.getObjectHandle(objectRef), reinterpret_cast<physx::PxU64>(&(valueRef)))
#define OMNI_PVD_SHOULD_RECORD_REMOVAL(writer, regData, classID, attributeID, objectRef, valueRef) \
physx::NpOmniPvdShouldRecordRemoval(writer, regData, #classID, #attributeID, regData->pvd##classID.getObjectHandle(objectRef), reinterpret_cast<physx::PxU64>(&(valueRef)))


//
// Define the macros needed in CmOmniPvdAutoGenSetData.h
//
// The write scope yields a NULL writer when not sampling, so attribute setters and other per-object
// hooks do not write between stopSampling() and the next startSampling().
#undef OMNI_PVD_GET_WRITER
#define OMNI_PVD_GET_WRITER(writer) \
physx::PxOmniPvd::ScopedExclusiveWriter writeLock(physx::NpOmniPvdSampling() ? NpOmniPvdGetInstance() : NULL); \
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
