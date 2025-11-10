// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdDefines.h"

enum OmniPvdUsdClassEnum
{
    eUSDClassXform,
    eUSDClassGeomScope,
    eUSDClassOver,
    eUSDClassGeomSphere,
    eUSDClassGeomCapsule,
    eUSDClassGeomCube,
    eUSDClassGeomPlane,
    eUSDClassGeomMesh,
    eUSDClassGeomPoints,
    eUSDClassEnum,
    eUSDClassDistantLight,
    eUSDClassCylinder,
    eUSDClassCone
};

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
    ePxUndefined
};

enum OmniPvdUsdAttributeEnum
{
    eUSDAttributeTranslateOp,
    eUSDAttributeScaleOp,
    eUSDAttributeRotationOp,
    eUSDAttributeDisplayColor,
    eUSDAttributeCustom,
    eUSDAttributeChildNode,
    eUSDAttributeAxis,
    eUSDAttributeHeight,
    eUSDAttributeRadius,
    eUSDAttributeVerts,
    eUSDAttributeTris,
    eUSDAttributePoints,
    eUSDAttributeEnum,
    eUSDAttributeTransformFork,
    eUSDAttributeNone
};

class OmniPvdClass;

class OmniPvdAttributeDef
{
public:
    OmniPvdAttributeDef();
    uint32_t mNbrFields;
    uint16_t mDataType;
    uint64_t mOmniAttributeHandle;
    pxr::TfToken* mPxrToken;
    std::string mAttributeName;
    OmniPvdUsdAttributeEnum mUsdAttributeId;
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
    uint64_t mTimeStamp;
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
    void setDataPtr(const uint8_t* data, uint32_t dataLen);
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

class OmniPvdClass
{
public:
    OmniPvdClass();
    uint64_t reserveObjectId();
    OmniPvdClassHandle mOmniBaseClassHandle;

    OmniPvdClassHandle mOmniClassHandle;
    std::string mClassUsdPath;
    std::string mClassName;
    std::vector<OmniPvdAttributeDef*> mAttributeDefinitions;
    OmniPvdUsdClassEnum mUsdClassId;
    OmniPvdPhysXClassEnum mPhysXBaseProcessingClassId;
    OmniPvdPhysXClassEnum mPhysXLeafClassId;
    uint64_t mNextObjectId;
    std::vector<OmniPvdAttributeDef*> mBitFieldAttribs;
    bool mIsEnumClass;
    bool mIsBitFieldEnum; // if (mIsEnumClass==true) { if (mIsBitFieldEnum==true) {is a bitField} else {holds a single
                          // enum value}
    bool mIsDefaultParsed;

    // mInheritanceChain[0] -> points to the root base class
    // ...
    // mInheritanceChain[nbrInheritedClass - 1] -> points to the base class of this class
    // mInheritanceChain[nbrInheritedClass] -> points to the class itself
    std::vector<OmniPvdClass*> mInheritanceChain;
};

class OmniPvdObjectLifeSpan
{
public:
    uint64_t mFrameStart;
    uint64_t mFrameStop;
};

// Contains the attributes of a certain class in the inheritance chain
// It only contains the attributes of that class and no other
class OmniPvdClassInstance
{
public:
    OmniPvdClassInstance()
    {
    }
    std::vector<OmniPvdAttributeInstList*> mClassAttributeLists;
};

class OmniPvdObject
{
public:
    OmniPvdObject();
    void appendChild(OmniPvdObject* child);
    void prependChild(OmniPvdObject* child);
    void removeChild(OmniPvdObject* child);
    OmniPvdObject* getChild(const std::string& name);
    OmniPvdObject* findAncestorWithClass(OmniPvdClass* omniPvdClass);

    OmniPvdObjectHandle mOmniObjectHandle;
    OmniPvdObjectHandle mOmniAPIHandle; // PhysX pointer value in most cases
    OmniPvdObjectHandle mUID; // unique object ID
    pxr::SdfPath mPrimPath;

    ////////////////////////////////////////////////////////////////////////////////
    // mInheritanceChain[ 0 ] -> attributes of the root class
    // mInheritanceChain[ 1 ] -> attributes of the class deriving from the root class
    // ...
    // mInheritanceChain[ nbrInheritedClasses -1 ] -> points to the class this object instances from
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

class OmniPvdClassConfig
{
public:
    OmniPvdClassConfig(OmniPvdPhysXClassEnum physxEnum, OmniPvdUsdClassEnum usdEnum);
    OmniPvdPhysXClassEnum mPhysxClass;
    OmniPvdUsdClassEnum mUSDClass;
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

    std::unordered_map<std::string, pxr::TfToken*> mTokenMap;

    std::unordered_map<std::string, OmniPvdClassConfig*> mClassConfigMap; // PhysXClassName -> {OmniPvdPhysXClassEnum,
                                                                          // OmniPvdUsdClassEnum}
    std::unordered_map<std::string, OmniPvdUsdAttributeEnum> mAttributeConfigMap; // PhysXClassName + '.' +
                                                                                  // attributeName -> USDAttributeType

    uint32_t mActorTypeEnumRigidDynamic;
    uint32_t mActorTypeEnumRigidStatic;
    uint32_t mActorTypeEnumParticleSystem;

    // uint64_t mCurrentFrameId;
    uint64_t mNextInternalHandle;

    uint64_t mMinFrame;
    uint64_t mMaxFrame;

    // Instantiated by OmniPvdObjects in scenes.usd(a)
    OmniPvdClass* mSceneRootClass;

    OmniPvdClass* mRigidDynamicBranchClass;
    OmniPvdClass* mRigidStaticBranchClass;
    OmniPvdClass* mArticulationBranchClass;
    OmniPvdClass* mParticleSystemBranchClass;

    // Instantiated by OmniPvdObjects in shared.usd(a)
    OmniPvdClass* mSharedRootClass;

    OmniPvdClass* mSharedMaterialsClass;

    OmniPvdClass* mSharedMeshesClass;
    OmniPvdClass* mSharedConvexMeshesClass;
    OmniPvdClass* mSharedHeightfieldsClass;
    OmniPvdClass* mSharedTriangleMeshesClass;

    OmniPvdClass* mSharedShapesClass;

    OmniPvdClass* mAttributeNameClass; // object/attributeName
    OmniPvdClass* mAttributeRefClass; // object/attributeName/attribute_ref

    // Instantiated by OmniPvdObjects in both scenes.usd(a) and shared.usd(a)
    OmniPvdClass* mSharedShapeRefClass;
    OmniPvdClass* mConvexMeshRefClass;
    OmniPvdClass* mHeightfieldRefClass;
    OmniPvdClass* mTriangleMeshRefClass;

    // These OmniPvdObjects live in the scenes.usd(a) layer
    OmniPvdObject* mSceneLayerRoot; // path = /scenes

    // These OmniPvdObjects live in the shared.usd(a) layer
    // Prim paths on the right
    OmniPvdObject* mSharedLayerRoot; // path = /shared

    OmniPvdObject* mSharedMaterialsBranch; // path = /shared/materials

    OmniPvdObject* mSharedMeshesBranch; // path = /shared/meshes
    OmniPvdObject* mSharedConvexMeshesBranch; // path = /shared/meshes/convexmeshes
    OmniPvdObject* mSharedHeightfieldsBranch; // path = /shared/meshes/heightfields
    OmniPvdObject* mSharedTriangleMeshesBranch; // path = /shared/meshes/trianglemeshes

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
