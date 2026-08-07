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

#include "OmniPvdDefines.h"
#include "OmniPvdCommands.h"

#include <string>
#include <vector>
#include <list>
#include <unordered_map>
#include <cstring>
#include <stdint.h>

#define PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR 1

enum OmniPvdPhysXClassEnum
{
    ePxScene,
    ePxSceneFlag,
    ePxMaterial,
    ePxActor,
    ePxShape,
    ePxGeomSphere,
    ePxGeomCapsule,
    ePxGeomBox,
    ePxGeomPlane,
    ePxGeomConvexMesh,
    ePxGeomHeightfield,
    ePxGeomTriangleMesh,
    ePxConvexMesh,
    ePxHeightfield,
    ePxTriangleMesh,
    ePxInternalOmnniPvd,
    ePxArticulation,
    ePxArticulationJoint,
    ePxArticulationLink,
    ePxCustomGeometry,
    ePxCustomGeometryCylinder,
    ePxCustomGeometryCone,
    ePxJoint,
    ePxParticleBuffer,
    ePxConvexCoreGeometry,
    ePxConvexCoreCylinder,
    ePxConvexCoreCone,
    ePxDeformableVolume,
    ePxDeformableSurface,
    ePxGeomTetMesh,
    ePxTetrahedronMesh,
    ePxDeformableVolumeMesh,
    ePxUndefined
};

enum PvdDomAttributeEnum
{
    ePvdDomAttribTranslateOp,
    ePvdDomAttribScaleOp,
    ePvdDomAttribRotationOp,
    ePvdDomAttribDisplayColor,
    ePvdDomAttribCustom,
    ePvdDomAttribChildNode,
    ePvdDomAttribAxis,
    ePvdDomAttribHeight,
    ePvdDomAttribRadius,
    ePvdDomAttribVerts,
    ePvdDomAttribTris,
    ePvdDomAttribPoints,
    ePvdDomAttribDeformablePositions,
    ePvdDomAttribDeformableVelocities,
    ePvdDomAttribTets,
    ePvdDomAttribEnum,
    ePvdDomAttribTransformFork,
    ePvdDomAttribNone
};

class OmniPvdClass;

class OmniPvdAttributeDef
{
public:
    OmniPvdAttributeDef();
    uint32_t mNbrFields;
    uint16_t mDataType;
    uint64_t mOmniAttributeHandle;
    std::string mAttributeName;
    PvdDomAttributeEnum mAttributeId;
    uint16_t mClassOffset;
    OmniPvdClass* mClass;
    OmniPvdClass* mDerivedFromClass;
    uint8_t mIsUniqueList;
};

class OmniPvdAttributeInst
{
public:
    OmniPvdAttributeInst();
    virtual ~OmniPvdAttributeInst() = 0;
    // mTimeStamp:    frame this sample was written.
    // mEndTimeStamp: same frame -- held as a separate field for source
    //                compatibility with older readers that read a
    //                [mTimeStamp, mEndTimeStamp] keyframe range.
    //
    // The parser used to dedup byte-identical successive writes and extend
    // the previous sample's mEndTimeStamp forward to bridge the gap; that
    // path was removed because it silently fused identical-but-distinct
    // writes across silent gaps, breaking strict-frame ("was data written
    // at frame F?") lookups. After the change, every setAttribute becomes
    // its own keyframe and mEndTimeStamp == mTimeStamp always. See the
    // setAttribute path in PvdDomParser.cpp for the matching comment.
    uint64_t mTimeStamp;
    uint64_t mEndTimeStamp;
    OmniPvdAttributeInst* mNextAttribute;
};

class OmniPvdAttributeInstList
{
public:
    OmniPvdAttributeInstList();
    ~OmniPvdAttributeInstList();
    void addAttribute(OmniPvdAttributeInst* attribute);
    OmniPvdAttributeDef* mAttributeDef;
    OmniPvdAttributeInst* mFirst;
    OmniPvdAttributeInst* mLast;
};

class OmniPvdAttributeSample : public OmniPvdAttributeInst
{
public:
    OmniPvdAttributeSample();
    ~OmniPvdAttributeSample();
    bool isSame(const uint8_t* data, uint32_t dataLen);
    void assignData(const uint8_t* data, uint32_t dataLen);
    uint8_t* mData;
    uint32_t mDataLen;
    uint32_t mAllocatedDataLen;
};

class OmniPvdUniqueListElement
{
public:
    OmniPvdUniqueListElement();
    ~OmniPvdUniqueListElement();
    bool isSame(const uint8_t* data, uint32_t dataLen);
    void set(const uint8_t* data, uint32_t dataLen);
    uint8_t* mData;
    uint32_t mDataLen;
};

class OmniPvdUniqueList : public OmniPvdAttributeInst
{
public:
    ~OmniPvdUniqueList();
    // Add this data if no element has this data
    void addElement(const uint8_t* data, uint32_t dataLen);
    // Remove any element that has the same data
    void removeElement(const uint8_t* data, uint32_t dataLen);

    std::list<OmniPvdUniqueListElement*> mElements;
};

////////////////////////////////////////////////////////////////////////////////
// OmniPvdClass -- Schema definition for one type in the OVD stream.
//
// Inheritance Model
// -----------------
// OVD classes form single-inheritance chains replicating the PhysX SDK hierarchy.
// Example: PxActor -> PxRigidActor -> PxRigidBody -> PxRigidDynamic
//
// Each class defines its OWN attributes only in mAttributeDefinitions.
//   PxActor:         type, flags, name, worldBounds, ...
//   PxRigidActor:    globalPose, shapes
//   PxRigidBody:     mass, linearVelocity, angularVelocity, ...
//   PxRigidDynamic:  isSleeping, sleepThreshold, wakeCounter, ...
//
// The full ancestry is stored in mInheritanceChain:
//   PxRigidDynamic.mInheritanceChain = [PxActor, PxRigidActor, PxRigidBody, PxRigidDynamic]
//                                       [0]       [1]           [2]          [3] = self
//
// To get ALL attributes of a class (own + inherited), walk mInheritanceChain
// and collect mAttributeDefinitions from each entry.
//
// OmniPvdObject mirrors this structure: its mInheritedClassInstances array
// is parallel to mInheritanceChain -- index i holds attribute VALUES for the
// class at mInheritanceChain[i]. See OmniPvdObject below.
////////////////////////////////////////////////////////////////////////////////
class OmniPvdClass
{
public:
    OmniPvdClass();
    uint64_t reserveObjectId();
    OmniPvdClassHandle mOmniBaseClassHandle;

    OmniPvdClassHandle mOmniClassHandle;
    std::string mClassUsdPath;
    std::string mClassName;

    // Attributes defined directly on THIS class (not inherited).
    // To enumerate all attributes including inherited ones, walk mInheritanceChain.
    std::vector<OmniPvdAttributeDef*> mAttributeDefinitions;

    OmniPvdPhysXClassEnum mPhysXBaseProcessingClassId;
    OmniPvdPhysXClassEnum mPhysXLeafClassId;
    uint64_t mNextObjectId;
    std::vector<OmniPvdAttributeDef*> mBitFieldAttribs;
    bool mIsEnumClass;
    bool mIsBitFieldEnum; // if (mIsEnumClass==true) { if (mIsBitFieldEnum==true) {is a bitField} else {holds a single
                          // enum value}
    bool mIsDefaultParsed;

    // Full inheritance chain from root ancestor to this class:
    //   [0]    = root base class (e.g. PxActor)
    //   [1..n] = intermediate classes
    //   [last] = this class itself (e.g. PxRigidDynamic)
    std::vector<OmniPvdClass*> mInheritanceChain;
};

class OmniPvdObjectLifeSpan
{
public:
    uint64_t mFrameStart;
    uint64_t mFrameStop;
};

// Holds the attribute values for ONE class in an object's inheritance chain.
// mClassAttributeLists[j] corresponds to mAttributeDefinitions[j] on the
// associated OmniPvdClass. Each entry is a linked list of timestamped samples.
class OmniPvdClassInstance
{
public:
    OmniPvdClassInstance()
    {
    }
    std::vector<OmniPvdAttributeInstList*> mClassAttributeLists;
};

////////////////////////////////////////////////////////////////////////////////
// OmniPvdObject -- A concrete instance in the scene graph.
//
// Attribute Storage (parallel to the class inheritance chain)
// ----------------------------------------------------------
// mInheritedClassInstances is parallel to mOmniPvdClass->mInheritanceChain:
//
//   mOmniPvdClass->mInheritanceChain[i]  ->  class schema (which attributes exist)
//   mInheritedClassInstances[i]          ->  attribute values for that class
//
// Example for a PxRigidDynamic object:
//   mInheritedClassInstances[0] = PxActor values       (type, flags, name, ...)
//   mInheritedClassInstances[1] = PxRigidActor values  (globalPose, shapes)
//   mInheritedClassInstances[2] = PxRigidBody values   (mass, linearVelocity, ...)
//   mInheritedClassInstances[3] = PxRigidDynamic values (isSleeping, wakeCounter, ...)
//
// To read attribute "mass" on this object:
//   1. Find "mass" in PxRigidBody.mAttributeDefinitions -> attrIndex
//   2. PxRigidBody is at inheritanceChain[2] -> classIndex = 2
//   3. Read mInheritedClassInstances[2].mClassAttributeLists[attrIndex]
//   4. Walk the linked list to find the sample at the desired frame
////////////////////////////////////////////////////////////////////////////////
class OmniPvdObject
{
public:
    OmniPvdObject();
    void appendChild(OmniPvdObject* child);
    void insertChildFirst(OmniPvdObject* child);
    void removeChild(OmniPvdObject* child);
    OmniPvdObject* getChild(const std::string& name);
    OmniPvdObject* findAncestorWithClass(OmniPvdClass* omniPvdClass);

    OmniPvdObjectHandle mOmniObjectHandle;
    OmniPvdObjectHandle mOmniAPIHandle; // PhysX pointer value in most cases
    OmniPvdObjectHandle mUID; // unique object ID
    std::string mPath;

    // Parallel to mOmniPvdClass->mInheritanceChain (see OmniPvdClass docs above).
    //   [i] holds attribute values for the class at mOmniPvdClass->mInheritanceChain[i]
    //   [0]    = root ancestor attribute values
    //   [last] = this class's own attribute values
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<OmniPvdClassInstance> mInheritedClassInstances;

    // This should be per inherited class rather?
    // There should be one attribute object per class I think
    // std::vector<OmniPvdAttributeInstList*> mAttributeLists;

    uint8_t mAppearedFirstTime;
    OmniPvdClass* mOmniPvdClass;
    bool mIsShared;
    uint8_t mIsStaticVisibility;
    uint8_t mIsStaticVisible;
    uint32_t mActortype;
    OmniPvdObject* mReferenceObject;
    std::vector<OmniPvdObjectLifeSpan> mLifeSpans;
    bool mWasSDFCreated;

    OmniPvdObject* mAncestor;
    OmniPvdObject* mFirstChild;
    OmniPvdObject* mLastChild;
    OmniPvdObject* mPrevSibling;
    OmniPvdObject* mNextSibling;

    bool mIsReferenced;
    std::string mObjectName;
    std::string mOmniObjectName;
    uint64_t mFrameId;
};

inline uint32_t getHash32(uint32_t x)
{
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x);
    return x;
}

inline uint64_t getCombinedHash32(uint8_t* key, uint32_t keyLen)
{
    int32_t keyLenLeft = keyLen;
    uint64_t hashCombined = 0;
    while (keyLenLeft > 0)
    {
        const uint32_t iterations = (keyLenLeft >= 4) ? 4 : keyLenLeft;
        uint32_t keyForHashOp = 0;
        uint32_t shiftLen = 0;
        for (uint32_t i = 0; i < iterations; i++)
        {
            keyForHashOp |= ((*key) << shiftLen);
            shiftLen += 8;
            key++;
        }
        hashCombined += (hashCombined << 1) + getHash32(keyForHashOp);
        hashCombined &= 0xffffffff;
        keyLenLeft -= iterations;
    }
    return hashCombined;
}

inline uint64_t pairHash32(uint64_t key1, uint64_t key2)
{
    uint8_t bigKey[16];
    memcpy(bigKey, &key1, 8);
    memcpy(bigKey + 8, &key2, 8);
    return getCombinedHash32(bigKey, 16);
}

struct pair_hash_objectHandle
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
        return pairHash32(p.first, p.second);
    }
};

class OmniPvdDOMState
{
public:
    OmniPvdDOMState();
    ~OmniPvdDOMState();

    void parentUnderClassGroup(OmniPvdObject* ovdObject, bool isShared);

    std::list<OmniPvdObject*> mObjectCreations;
    std::list<OmniPvdObject*> mSceneCreations;

    std::unordered_map<OmniPvdAttributeHandle, OmniPvdAttributeDef*> mAttributeHandleToAttributeMap;
    std::unordered_map<OmniPvdObjectHandle, OmniPvdObjectHandle> mExternalToInternalHandleMap;
    std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*> mObjectHandleToObjectMap;
    std::unordered_map<OmniPvdClassHandle, OmniPvdClass*> mClassHandleToClassMap;

    std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*> mConstraintToSceneMap;
    std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*> mConstraintToJointMap;

    std::unordered_map<OmniPvdClass*, OmniPvdObject*> mClassToOriginsNodeMap; // class pointer -> class node under
                                                                              // /scenes
    std::unordered_map<OmniPvdClass*, OmniPvdObject*> mClassToOriginsSharedNodeMap; // class pointer -> class node under
                                                                                    // /shared

    std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, OmniPvdObject*, pair_hash_objectHandle>
        mActorSharedShapeToShapeRefMap;

    std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, OmniPvdObject*, pair_hash_objectHandle>
        mObjectAttributeNameMap;
    std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, OmniPvdObject*, pair_hash_objectHandle>
        mObjectAttributeRefMap;

    // Used to map the class of a certain attribute (attributeClass), in conjunction with the class of the object that
    // instantiates the object (instanceClass), to an index of the attributeClass in the inheritanceChain vector of the
    // object
    std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, uint32_t, pair_hash_objectHandle>
        mInheritedClassToClassOffset;

    std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> mSceneContextHandleToRigidDynamicBranchMap;
    std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> mSceneContextHandleToRigidStaticBranchMap;
    std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> mSceneContextHandleToArticulationBranchMap;
    std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> mSceneContextHandleToParticleSystemBranchMap;
    std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> mSceneContextHandleToDeformableVolumeBranchMap;
    std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> mSceneContextHandleToDeformableSurfaceBranchMap;

    std::unordered_map<std::string, OmniPvdPhysXClassEnum> mClassConfigMap; // PhysXClassName -> OmniPvdPhysXClassEnum
    std::unordered_map<std::string, PvdDomAttributeEnum> mAttributeConfigMap; // PhysXClassName + '.' +
                                                                              // attributeName -> PvdDomAttributeEnum

    uint32_t mActorTypeEnumRigidDynamic;
    uint32_t mActorTypeEnumRigidStatic;
    uint32_t mActorTypeEnumParticleSystem;
    uint32_t mActorTypeEnumDeformableVolume;
    uint32_t mActorTypeEnumDeformableSurface;

    // uint64_t mCurrentFrameId;
    uint64_t mNextInternalHandle;

    uint64_t mMinFrame;
    uint64_t mMaxFrame;

    // Instantiated by OmniPvdObjects in scenes
    OmniPvdClass* mSceneRootClass;

    OmniPvdClass* mRigidDynamicBranchClass;
    OmniPvdClass* mRigidStaticBranchClass;
    OmniPvdClass* mArticulationBranchClass;
    OmniPvdClass* mParticleSystemBranchClass;
    OmniPvdClass* mDeformableVolumeBranchClass;
    OmniPvdClass* mDeformableSurfaceBranchClass;

    // Instantiated by OmniPvdObjects in shared
    OmniPvdClass* mSharedRootClass;

    OmniPvdClass* mSharedMaterialsClass;

    OmniPvdClass* mSharedMeshesClass;
    OmniPvdClass* mSharedConvexMeshesClass;
    OmniPvdClass* mSharedHeightfieldsClass;
    OmniPvdClass* mSharedTriangleMeshesClass;
    OmniPvdClass* mSharedTetrahedronMeshesClass;
    OmniPvdClass* mSharedDeformableVolumeMeshesClass;

    OmniPvdClass* mSharedShapesClass;

    OmniPvdClass* mAttributeNameClass; // object/attributeName
    OmniPvdClass* mAttributeRefClass; // object/attributeName/attribute_ref

    // Instantiated by OmniPvdObjects in both scenes and shared
    OmniPvdClass* mSharedShapeRefClass;
    OmniPvdClass* mConvexMeshRefClass;
    OmniPvdClass* mHeightfieldRefClass;
    OmniPvdClass* mTriangleMeshRefClass;
    OmniPvdClass* mTetrahedronMeshRefClass;

    OmniPvdClass* mSimulationMeshClass;         // synthetic container for simulation mesh

    // These OmniPvdObjects live in the scenes layer
    OmniPvdObject* mSceneRoot; // path = /scenes

    // These OmniPvdObjects live in the shared layer
    // Prim paths on the right
    OmniPvdObject* mSharedRoot; // path = /shared

    OmniPvdObject* mSharedMaterialsBranch; // path = /shared/materials

    OmniPvdObject* mSharedMeshesBranch; // path = /shared/meshes
    OmniPvdObject* mSharedConvexMeshesBranch; // path = /shared/meshes/convexmeshes
    OmniPvdObject* mSharedHeightfieldsBranch; // path = /shared/meshes/heightfields
    OmniPvdObject* mSharedTriangleMeshesBranch; // path = /shared/meshes/trianglemeshes
    OmniPvdObject* mSharedTetrahedronMeshesBranch; // path = /shared/meshes/tetrahedronmeshes
    OmniPvdObject* mSharedDeformableVolumeMeshesBranch; // path = /shared/meshes/deformablevolumemeshes

    OmniPvdObject* mSharedShapesBranch; // path = /shared/shapes

    uint32_t mOvdIntegrationVersionMajor; // What OVD integration major version that we accept

    // mOvdIntegVersionWasChecked is true if the first object if of class type PxOmniPvdMetaData
    //   AND its attribute ovdIntegrationVersionMajor was tested against domState.mOvdIntegrationVersionMajor
    // mOvdIntegVersionWasChecked is also true if the first object does not contain the attribute
    // ovdIntegrationVersionMajor -> older OVD stream
    bool mOvdIntegVersionWasChecked;
    // mOvdIntegVersionPassed is only relevant if mOvdIntegVersionWasChecked is true, then if the
    // ovdIntegrationVersionMajor of the reader/parser was larger or equal to the version of the stream,
    // mOvdIntegVersionPassed is set to true, otherwise false
    bool mOvdIntegVersionPassed;
    // Conclusion. To make use of this variable pair use for example:
    // if (mOvdIntegVersionWasChecked && !mOvdIntegVersionPassed) -> exit parsing

    uint32_t mStreamOvdIntegVersionMajor;
    uint32_t mStreamOvdIntegVersionMinor;

    // PhysX specific
    OmniPvdClass* mPxSceneClass;
    OmniPvdClass* mPxArticulationReducedCoordinateClass;
    OmniPvdClass* mPxArticulationLinkClass;
};
