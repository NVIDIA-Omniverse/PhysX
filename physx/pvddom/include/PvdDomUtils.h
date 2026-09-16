// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


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
OmniPvdObject* createInternalObject(OmniPvdClass* internalCass, uint64_t objectHandle,
                                    uint64_t recordingSegmentId);
OmniPvdObject* createNamedObject(OmniPvdClass* internalCass, const std::string& objectName);
OmniPvdObject* createNamedObject(OmniPvdClass* internalCass, const std::string& objectName,
                                 uint64_t recordingSegmentId);
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
// Returns the scene frame, or 0 without a scene.
uint64_t getLifespanOpenFrameId(OmniPvdObject* object, OmniPvdDOMState* domState);
// Returns the scene frame, or the latest stream frame without a scene.
uint64_t getLifespanCloseFrameId(OmniPvdObject* object, OmniPvdDOMState* domState);
// Returns one past the scene frame or maximum stream frame.
uint64_t getSupersedeCloseFrameId(OmniPvdObject* object, OmniPvdDOMState* domState);

bool isSameString(const char* str, const char* str1);
void getSanitizedName(std::string& sanitizedName, const char* objectName);

bool isPowerOfTwo(uint32_t v);
uint32_t nbrBitsSet(uint32_t v);
uint32_t findFirstBitPos(uint32_t n);
