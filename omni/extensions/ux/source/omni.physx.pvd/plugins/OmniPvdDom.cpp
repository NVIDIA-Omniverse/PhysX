// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdDom.h"
#include "OmniPvdOvdParserConfig.h"

#include "OmniPvdDomUtils.h"

OmniPvdAttributeDef::OmniPvdAttributeDef()
{
    mIsUniqueList = 0;
    mPxrToken = 0;
    mDerivedFromClass = 0;
}

OmniPvdAttributeInst::OmniPvdAttributeInst()
{
    mNextAttribute = 0;
}

OmniPvdAttributeInst::~OmniPvdAttributeInst()
{
}

OmniPvdAttributeInstList::OmniPvdAttributeInstList()
{
    mFirst = 0;
    mLast = 0;
    mAttributeDef = 0;
}

OmniPvdAttributeInstList::~OmniPvdAttributeInstList()
{
}

void OmniPvdAttributeInstList::addAttribute(OmniPvdAttributeInst *attribute)
{
    if (mFirst)
    {
        mLast->mNextAttribute = attribute;
        mLast = attribute;
        attribute->mNextAttribute = 0;
    }
    else
    {
        mFirst = attribute;
        mLast = attribute;
        attribute->mNextAttribute = 0;
    }
}

OmniPvdAttributeSample::OmniPvdAttributeSample()
{
    mData = 0;
    mDataLen = 0;
    mAllocatedDataLen = 0;
}

OmniPvdAttributeSample::~OmniPvdAttributeSample()
{
    delete[] mData;
}

bool OmniPvdAttributeSample::isSame(const uint8_t *data, uint32_t dataLen)
{
    if (mDataLen != dataLen) return false;
    if (memcmp(mData, data, dataLen) == 0) return true;
    return false;
}

void OmniPvdAttributeSample::setDataPtr(const uint8_t *data, uint32_t dataLen)
{
    if (dataLen < 1) return;
    if (mAllocatedDataLen < dataLen)
    {
        delete[] mData;
        mData = new uint8_t[dataLen];
        mAllocatedDataLen = dataLen;
    }
    memcpy(mData, data, dataLen);
    mDataLen = dataLen;
}

OmniPvdUniqueListElement::OmniPvdUniqueListElement()
{
    mData = 0;
    mDataLen = 0;
}

OmniPvdUniqueListElement::~OmniPvdUniqueListElement()
{
    delete[] mData;
}

bool OmniPvdUniqueListElement::isSame(const uint8_t *data, uint32_t dataLen)
{
    if (mDataLen != dataLen) return false;
    if (memcmp(mData, data, dataLen) == 0) return true;
    return false;
}

void OmniPvdUniqueListElement::set(const uint8_t *data, uint32_t dataLen)
{
    if (mData) return;
    mData = new uint8_t[dataLen];
    mDataLen = dataLen;
    memcpy(mData, data, dataLen);
}

OmniPvdUniqueList::~OmniPvdUniqueList()
{
    std::list<OmniPvdUniqueListElement*>::iterator it;
    for (it = mElements.begin(); it != mElements.end(); ++it)
    {
        OmniPvdUniqueListElement *elem = (*it);
        delete elem;
    }
}
// Add this data if no element has this data
void OmniPvdUniqueList::addElement(const uint8_t *data, uint32_t dataLen)
{
    if (!data) return;
    if (dataLen < 1) return;
    std::list<OmniPvdUniqueListElement*>::iterator it;
    for (it = mElements.begin(); it != mElements.end(); ++it)
    {
        if ((*it)->isSame(data, dataLen))
        {
            break;
        }
    }
    if (it == mElements.end())
    {
        // Add this element
        OmniPvdUniqueListElement *elem = new OmniPvdUniqueListElement();
        elem->set(data, dataLen);
        mElements.emplace_back(elem);
    }
}
// Remove any element that has the same data
void OmniPvdUniqueList::removeElement(const uint8_t *data, uint32_t dataLen)
{
    if (!data) return;
    if (dataLen < 1) return;
    std::list<OmniPvdUniqueListElement*>::iterator it;
    for (it = mElements.begin(); it != mElements.end(); ++it)
    {
        if ((*it)->isSame(data, dataLen))
        {
            OmniPvdUniqueListElement *elem = (*it);
            delete elem;
            mElements.erase(it);
            break;
        }
    }
}

OmniPvdClass::OmniPvdClass()
{
    mIsEnumClass = false;
    mIsBitFieldEnum = true;
    mNextObjectId = 1;
    mIsDefaultParsed = true;
    mPhysXLeafClassId = OmniPvdPhysXClassEnum::ePxUndefined;
}

uint64_t OmniPvdClass::reserveObjectId()
{
    uint64_t objectId = mNextObjectId;
    mNextObjectId++;
    return objectId;
}

OmniPvdObject::OmniPvdObject()
{
    mAppearedFirstTime = 1;
    
    mLifeSpans.resize(1);
    mLifeSpans[0].mFrameStart = 0;
    mLifeSpans[0].mFrameStop = 0;

    mIsShared = 0;
    mReferenceObject = 0;
    mIsStaticVisibility = 0;
    mIsStaticVisible = 1;
    mOmniPvdClass = 0;

    mWasSDFCreated = false;

    mAncestor =  nullptr;
    mFirstChild = nullptr;
    mLastChild = nullptr;
    mPrevSibling = nullptr;
    mNextSibling = nullptr;

    mIsReferenced = false;

    mUID = 0;

    mFrameId = 0;
}

void OmniPvdObject::appendChild(OmniPvdObject* child)
{
    if (!child) return;
    if (child == this) return;
    if (child->mAncestor == this) return;
    if (child->mAncestor)
    {
        child->mAncestor->removeChild(child);
    }
    if (mLastChild) {
        mLastChild->mNextSibling = child;
        child->mPrevSibling = mLastChild;
    }
    else
    {
        mFirstChild = child;        
        child->mPrevSibling = nullptr;
    }    
    mLastChild = child;
    child->mAncestor = this;
    child->mNextSibling = nullptr;
}

void OmniPvdObject::prependChild(OmniPvdObject* child)
{
    if (!child) return;
    if (child == this) return;
    if (child->mAncestor == this) return;
    if (child->mAncestor)
    {
        child->mAncestor->removeChild(child);
    }
    if (mFirstChild) {
        mFirstChild->mPrevSibling = child;
        child->mPrevSibling = nullptr;
        child->mNextSibling = mFirstChild;
    }
    else
    {
        mLastChild = child;
        child->mNextSibling = nullptr;
        child->mPrevSibling = nullptr;
    }
    mFirstChild = child;
    child->mAncestor = this;
    child->mPrevSibling = nullptr;
}

void OmniPvdObject::removeChild(OmniPvdObject* child)
{
    if (!child) return;
    if (child == this) return;
    if (child->mAncestor != this) return;

    if (child->mPrevSibling)
    {
        child->mPrevSibling->mNextSibling = child->mNextSibling;
    }
    if (child->mNextSibling)
    {
        child->mNextSibling->mPrevSibling = child->mPrevSibling;
    }
    if (child == mFirstChild)
    {
        mFirstChild = child->mNextSibling;
    }
    if (child == mLastChild)
    {
        mLastChild = child->mPrevSibling;
    }
    if ((mFirstChild == nullptr) || (mLastChild == nullptr))
    {
        mFirstChild = nullptr;
        mLastChild = nullptr;
    }

    child->mPrevSibling = nullptr;
    child->mNextSibling = nullptr;
    child->mAncestor = nullptr;
}

OmniPvdObject* OmniPvdObject::getChild(const std::string& name) {
    OmniPvdObject* child = mFirstChild;
    while (child && (child->mObjectName != name))
    {
        child = child->mNextSibling;
    }
    return child;
}

OmniPvdObject* OmniPvdObject::findAncestorWithClass(OmniPvdClass *omniPvdClass)
{
    if (!mAncestor) return nullptr;
    OmniPvdObject* ancestor = mAncestor;
    while (ancestor)
    {
        if (ancestor->mOmniPvdClass == omniPvdClass)
        {
            return ancestor;
        }
        ancestor = ancestor->mAncestor;
    }
    return nullptr;
}

OmniPvdClassConfig::OmniPvdClassConfig(OmniPvdPhysXClassEnum physxEnum, OmniPvdUsdClassEnum usdEnum)
{
    mPhysxClass = physxEnum;
    mUSDClass = usdEnum;
}

OmniPvdDOMState::OmniPvdDOMState()
{
    initPvdDomState(*this);
}

template<typename Map>
void deletePointerVector(Map& map)
{
    std::list<OmniPvdObject*>::iterator it;
    for (it = map.begin(); it != map.end(); it++)
    {
        delete *it;
    }
    map.clear();
}

template<typename Map>
void deletePointerContainer(Map& map)
{
    for (auto& it : map)
    {
        delete it.second;
    }
    map.clear();
}

OmniPvdDOMState::~OmniPvdDOMState()
{
    // TODO : do automatically
    // Cleanup on isle 12!
    delete mSceneRootClass;
    delete mRigidDynamicBranchClass;
    delete mRigidStaticBranchClass;
    delete mArticulationBranchClass;
    delete mParticleSystemBranchClass;

    delete mSharedRootClass;
    delete mSharedMaterialsClass;
    delete mSharedMeshesClass;
    delete mSharedConvexMeshesClass;
    delete mSharedHeightfieldsClass;
    delete mSharedTriangleMeshesClass;
    delete mSharedShapesClass;
    delete mSharedShapeRefClass;
    delete mConvexMeshRefClass;
    delete mHeightfieldRefClass;
    delete mTriangleMeshRefClass;

    
    deletePointerVector(mObjectCreations);
    deletePointerContainer(mAttributeHandleToAttributeMap);

    mSceneCreations.clear();

    mExternalToInternalHandleMap.clear();
    mObjectHandleToObjectMap.clear();
    mConstraintToSceneMap.clear();
    mConstraintToJointMap.clear();
    mClassHandleToClassMap.clear();
    mActorSharedShapeToShapeRefMap.clear();
    mSceneContextHandleToRigidDynamicBranchMap.clear();
    mSceneContextHandleToRigidStaticBranchMap.clear();
    mSceneContextHandleToArticulationBranchMap.clear();
    mSceneContextHandleToParticleSystemBranchMap.clear();

    mObjectAttributeNameMap.clear();
    mObjectAttributeRefMap.clear();

    deletePointerContainer(mTokenMap);
    deletePointerContainer(mClassConfigMap);

    mAttributeConfigMap.clear();    
}

void OmniPvdDOMState::parentUnderClassGroup(OmniPvdObject* ovdObject, bool isShared)
{
    OmniPvdObject* classNode = findClassNodeObject(ovdObject->mOmniPvdClass, isShared ? mClassToOriginsSharedNodeMap : mClassToOriginsNodeMap );
    if (!classNode)
    {
        // Create the classNode
        if (isShared)
        {
            classNode = createInternalNode(mObjectCreations,  mSharedLayerRoot, ovdObject->mOmniPvdClass, 0, 1, 0);
            mClassToOriginsSharedNodeMap[ovdObject->mOmniPvdClass] = classNode;
        }
        else
        {
            classNode = createInternalNode(mObjectCreations, mSceneLayerRoot, ovdObject->mOmniPvdClass, 0, 0, 1);
            mClassToOriginsNodeMap[ovdObject->mOmniPvdClass] = classNode;
        }
    }
    // Parent the pvdObject under the class node
    classNode->appendChild(ovdObject);
}
