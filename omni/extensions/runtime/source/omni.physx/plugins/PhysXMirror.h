// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <PxPhysicsAPI.h>

namespace omni
{
namespace physx
{
inline void* copyAlignedMemory(const void* inMemory, size_t memSize)
{
    void* nm = malloc(memSize + PX_SERIAL_FILE_ALIGN);
    const void* memoryB = (const void*)((size_t(inMemory) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
    void* memoryA = (void*)((size_t(nm) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
    memcpy(memoryA, memoryB, memSize);

    return nm;
}

void mirrorActor(::physx::PxActor& actor,
                 void*& outMemory,
                 uint32_t& memSize,
                 ::physx::PxSerializationRegistry& sr,
                 ::physx::PxCollection*& outSharedCol);
::physx::PxActor* instantiateMirrorActor(void* nonalignedBlock,
                                         ::physx::PxSerializationRegistry& sr,
                                         ::physx::PxScene& scene,
                                         ::physx::PxCollection*& collectionOut,
                                         const ::physx::PxCollection* sharedCollection);

void mirrorHierarchy(::physx::PxScene& scene,
                     void*& outMemory,
                     uint32_t& memSize,
                     ::physx::PxSerializationRegistry& sr,
                     ::physx::PxCollection*& outSharedCol,
                     const std::unordered_set<void*>& inPtrs);

// typedef void (*ProcessObjectFn)(::physx::PxBase& object);

using ProcessObjectFn = std::function<void(::physx::PxBase& object)>;

::physx::PxCollection* instantiateHierarchy(void* nonalignedBlock,
                                            ::physx::PxSerializationRegistry& sr,
                                            ::physx::PxScene& scene,
                                            const ::physx::PxCollection* sharedCollection,
                                            const ProcessObjectFn& proccesObjectFn);
} // namespace physx
} // namespace omni
