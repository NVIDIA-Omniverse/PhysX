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


// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "PvdDom.h"

OmniPvdClass* findOmniPvdClass(OmniPvdClassHandle omniPvdClassHandle,
                               std::unordered_map<OmniPvdClassHandle, OmniPvdClass*>& classHandleToClassMap);
OmniPvdAttributeDef* findOmniPvdAttribute(
    OmniPvdAttributeHandle omniPvdAttributeHandle,
    std::unordered_map<OmniPvdAttributeHandle, OmniPvdAttributeDef*>& attributeHandleToAttributeMap);
OmniPvdObject* findOmniPvdObject(OmniPvdObjectHandle omniPvdObjectHandle,
                                 std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*>& objectHandleToObjectMap);
uint64_t getInternalHandle(OmniPvdObjectHandle externalHandle,
                           std::unordered_map<OmniPvdObjectHandle, OmniPvdObjectHandle>& external2InternalMap);
OmniPvdClass* createInternalClass(const char* className, OmniPvdPhysXClassEnum physXClass);
OmniPvdObject* createInternalObject(OmniPvdClass* internalCass, uint64_t objectHandle);
OmniPvdObject* createNamedObject(OmniPvdClass* internalCass, const std::string& objectName);
OmniPvdObject* createInternalNode(std::list<OmniPvdObject*>& objectCreations,
                                  OmniPvdObject* ancestor,
                                  OmniPvdClass* internalCass,
                                  OmniPvdObjectHandle objectHandle,
                                  uint8_t isShared,
                                  uint8_t isVisible);
OmniPvdObject* findChildObject(uint64_t* data,
                               std::unordered_map<OmniPvdObjectHandle, OmniPvdObjectHandle>& externalToInternalHandleMap,
                               std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*>& objectHandleToObjectMap);
OmniPvdObject* findOmniPvdRefObject(
    OmniPvdObjectHandle omniPvdObjectHandle,
    OmniPvdObjectHandle omniPvdSharedObjectHandle,
    std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, OmniPvdObject*, pair_hash_objectHandle>&
        objectSharedObjectToRefObjectMap);
OmniPvdObject* findClassNodeObject(OmniPvdClass* classNode,
                                   std::unordered_map<OmniPvdClass*, OmniPvdObject*>& classToNodeMap);

// Returns -1 if no offset found, 0 -> n otherwise
// Checks the vector offset of omniPvdInheritedClass in the omniPvdInstanceClass inheritance vector
int32_t findOmniPvdClassOffset(
    OmniPvdClassHandle omniPvdInstanceClass,
    OmniPvdClassHandle omniPvdInheritedClass,
    std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, uint32_t, pair_hash_objectHandle>&
        inheritanceClassOffsetMap);

OmniPvdObject* createOmniPvdBranchObject(const OmniPvdContextHandle contextHandle,
                                         OmniPvdClass* branchClass,
                                         OmniPvdObjectHandle branchHandle,
                                         OmniPvdObject* branchAncestor,
                                         std::list<OmniPvdObject*>& objectCreations,
                                         std::unordered_map<OmniPvdContextHandle, OmniPvdObject*>& contextHandleTobranchMap);
void attachChildObject(const std::string& attributeName,
                       const OmniPvdCommand::Enum cmdType,
                       const OmniPvdAttributeDef* omniPvdAttributeDef,
                       uint64_t* data,
                       OmniPvdObject* omniPvdObject,
                       std::unordered_map<uint64_t, uint64_t>& externalToInternalHandleMap,
                       std::unordered_map<uint64_t, OmniPvdObject*>& objectHandleToObjectMap);

uint8_t* getAttribData(int32_t& attribIndex, int32_t& classIndex, const char* attribName, OmniPvdObject* omniPvdObject);
OmniPvdAttributeInstList* getAttribList(int32_t& attribIndex,
                                        int32_t& classIndex,
                                        const char* attribName,
                                        OmniPvdObject* omniPvdObject);
void createSceneBranches(OmniPvdObjectHandle sceneHandle, OmniPvdObject* sceneBranch, OmniPvdDOMState* domState);
OmniPvdObject* getRigidDynamicBranch(OmniPvdObjectHandle sceneHandle,
                                     OmniPvdObject* sceneBranch,
                                     OmniPvdDOMState* domState);
OmniPvdObject* getRigidStaticBranch(OmniPvdObjectHandle sceneHandle, OmniPvdObject* sceneBranch, OmniPvdDOMState* domState);
OmniPvdObject* getArticulationBranch(OmniPvdObjectHandle sceneHandle,
                                     OmniPvdObject* sceneBranch,
                                     OmniPvdDOMState* domState);
OmniPvdObject* getParticleSystemBranch(OmniPvdObjectHandle sceneHandle,
                                       OmniPvdObject* sceneBranch,
                                       OmniPvdDOMState* domState);
OmniPvdObject* getDeformableVolumeBranch(OmniPvdObjectHandle sceneHandle,
                                         OmniPvdObject* sceneBranch,
                                         OmniPvdDOMState* domState);
OmniPvdObject* getDeformableSurfaceBranch(OmniPvdObjectHandle sceneHandle,
                                          OmniPvdObject* sceneBranch,
                                          OmniPvdDOMState* domState);
void getAttribIndex(int32_t& attribIndex, int32_t& classIndex, const char* attribName, OmniPvdObject* omniPvdObject);

uint64_t getFrameIdFromScene(OmniPvdObject* object, OmniPvdDOMState* domState);
uint64_t getLastFrameIdFromScene(OmniPvdDOMState* domState);

bool isSameString(const char* str, const char* str1);
void getSanitizedName(std::string& sanitizedName, const char* objectName);

bool isPowerOfTwo(uint32_t v);
uint32_t nbrBitsSet(uint32_t v);
uint32_t findFirstBitPos(uint32_t n);
