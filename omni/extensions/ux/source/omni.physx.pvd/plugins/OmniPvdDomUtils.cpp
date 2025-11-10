// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdDomUtils.h"
#include <carb/logging/Log.h>

OmniPvdClass* findOmniPvdClass(OmniPvdClassHandle omniPvdClassHandle, std::unordered_map<OmniPvdClassHandle, OmniPvdClass*> &classHandleToClassMap)
{
    auto it = classHandleToClassMap.find(omniPvdClassHandle);
    return (it != classHandleToClassMap.end()) ? it->second : nullptr;
}

OmniPvdAttributeDef* findOmniPvdAttribute(OmniPvdAttributeHandle omniPvdAttributeHandle, std::unordered_map<OmniPvdAttributeHandle, OmniPvdAttributeDef*> &attributeHandleToAttributeMap)
{
    auto it = attributeHandleToAttributeMap.find(omniPvdAttributeHandle);
    return (it != attributeHandleToAttributeMap.end()) ? it->second : nullptr;
}

OmniPvdObject* findOmniPvdObject(OmniPvdObjectHandle omniPvdObjectHandle, std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*> &objectHandleToObjectMap)
{
    auto it = objectHandleToObjectMap.find(omniPvdObjectHandle);
    return (it != objectHandleToObjectMap.end()) ? it->second : nullptr;
}

uint64_t getInternalHandle(OmniPvdObjectHandle externalHandle, std::unordered_map<OmniPvdObjectHandle, OmniPvdObjectHandle> &external2InternalMap)
{
    auto it = external2InternalMap.find(externalHandle);
    return (it != external2InternalMap.end()) ? it->second : 0;
}

OmniPvdClass* createInternalClass(const char *className, OmniPvdPhysXClassEnum physXClass, OmniPvdUsdClassEnum usdClass)
{
    OmniPvdClass *internalClass = new OmniPvdClass();
    internalClass->mOmniClassHandle = 0;
    internalClass->mClassName = std::string(className);
    internalClass->mPhysXBaseProcessingClassId = physXClass;
    internalClass->mUsdClassId = usdClass;
    internalClass->mIsDefaultParsed = false; // If the class is explicitly created by the user, it's not default parsed
    return internalClass;
}

OmniPvdObject* createInternalObject(OmniPvdClass* internalCass, uint64_t objectHandle)
{
    OmniPvdObject *refObject = new OmniPvdObject();
    refObject->mOmniObjectHandle = objectHandle;
    refObject->mOmniPvdClass = internalCass;
    return refObject;
}

OmniPvdObject* createNamedObject(OmniPvdClass* internalCass, const std::string& objectName)
{
    OmniPvdObject *refObject = new OmniPvdObject();
    refObject->mOmniObjectHandle = 0;
    refObject->mOmniPvdClass = internalCass;
    refObject->mObjectName = objectName;
    return refObject;
}

OmniPvdObject* createInternalNode(std::list<OmniPvdObject*> &objectCreations, OmniPvdObject* ancestor, OmniPvdClass* internalCass, OmniPvdObjectHandle objectHandle,uint8_t isShared, uint8_t isVisible)
{
    OmniPvdObject *node = createInternalObject(internalCass, objectHandle);
    node->mIsShared = isShared;
    node->mIsStaticVisibility = 1;
    node->mIsStaticVisible = isVisible;
    if (ancestor)
    {
        ancestor->appendChild(node);
    }
    objectCreations.push_back(node);
    return node;
}

OmniPvdObject* findChildObject(uint64_t *data, std::unordered_map<OmniPvdObjectHandle, OmniPvdObjectHandle> &externalToInternalHandleMap, std::unordered_map<OmniPvdObjectHandle, OmniPvdObject*> &objectHandleToObjectMap)
{
    OmniPvdObjectHandle childHandle;
    memcpy(&childHandle, data, sizeof(OmniPvdObjectHandle) * 1);
    childHandle = getInternalHandle(childHandle, externalToInternalHandleMap);
    return findOmniPvdObject(childHandle, objectHandleToObjectMap);
}

OmniPvdObject* findOmniPvdRefObject(OmniPvdObjectHandle omniPvdObjectHandle, OmniPvdObjectHandle omniPvdSharedObjectHandle, std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, OmniPvdObject*, pair_hash_objectHandle> &objectSharedObjectToRefObjectMap)
{
    std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle> searchKey{ omniPvdObjectHandle, omniPvdSharedObjectHandle };
    auto it = objectSharedObjectToRefObjectMap.find(searchKey);
    return (it != objectSharedObjectToRefObjectMap.end()) ? it->second : 0;
}

int32_t findOmniPvdClassOffset(OmniPvdClassHandle omniPvdInstanceClass, OmniPvdClassHandle omniPvdInheritedClass, std::unordered_map<std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle>, uint32_t, pair_hash_objectHandle> &inheritanceClassOffsetMap)
{
    std::pair<OmniPvdObjectHandle, OmniPvdObjectHandle> searchKey{ omniPvdInstanceClass, omniPvdInheritedClass };
    auto it = inheritanceClassOffsetMap.find(searchKey);
    return (it != inheritanceClassOffsetMap.end()) ? it->second : -1;
}

OmniPvdObject* findClassNodeObject(OmniPvdClass* classNode, std::unordered_map<OmniPvdClass*, OmniPvdObject*> &classToNodeMap)
{
    auto it = classToNodeMap.find(classNode);
    return (it != classToNodeMap.end()) ? it->second : nullptr;
}

pxr::TfToken* getAttributeToken(std::string attributeNameStr, std::unordered_map<std::string, pxr::TfToken*> &tokenMap, const std::string& prefix)
{
    const std::string concatString = prefix + attributeNameStr;
    auto it = tokenMap.find(concatString);
    if (it != tokenMap.end())
    {
        return it->second;
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Create the token and insert it into the map
    ////////////////////////////////////////////////////////////////////////////////
    return tokenMap[concatString] = new pxr::TfToken(concatString);
}

OmniPvdObject* createOmniPvdBranchObject(const OmniPvdContextHandle contextHandle, OmniPvdClass* branchClass, OmniPvdObjectHandle branchHandle, OmniPvdObject* branchAncestor, std::list<OmniPvdObject*> &objectCreations, std::unordered_map<OmniPvdContextHandle, OmniPvdObject*> &contextHandleTobranchMap)
{
    OmniPvdObject* branch = new OmniPvdObject();
    branch->mOmniObjectHandle = branchHandle; // Uses as display integer : branchName + _ + branchHandle
    branch->mOmniPvdClass = branchClass;
    branch->mIsStaticVisibility = 1;
    branch->mIsStaticVisible = 1;
    if (branchAncestor) branchAncestor->appendChild(branch); // The Branchestor, thirst mutilator!
    objectCreations.push_back(branch);
    contextHandleTobranchMap[contextHandle] = branch;
    return branch;
}

// Set the internal reference object (omniPvdObject->mReferenceObject) to point to the shared mesh data (childObject)
void attachChildObject(const std::string &attributeName, const OmniPvdCommand::Enum cmdType, const OmniPvdAttributeDef* omniPvdAttributeDef, uint64_t* data, OmniPvdObject *omniPvdObject, std::unordered_map<uint64_t, uint64_t> &externalToInternalHandleMap, std::unordered_map<uint64_t, OmniPvdObject*> &objectHandleToObjectMap)
{
    if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),attributeName.c_str()))
    {
        if (cmdType == OmniPvdCommand::eSET_ATTRIBUTE)
        {
            OmniPvdObject *childObject = findChildObject((uint64_t*)data, externalToInternalHandleMap, objectHandleToObjectMap);
            if (childObject)
            {
                // Only if we have a reference object can we point to the child object
                if (omniPvdObject->mReferenceObject)
                {
                    omniPvdObject->mReferenceObject->mReferenceObject = childObject;
                }
            }
        }
    }
}


void createSceneBranches(OmniPvdObjectHandle sceneHandle, OmniPvdObject *sceneBranch, OmniPvdDOMState* domState)
{
    createOmniPvdBranchObject(sceneHandle, domState->mRigidDynamicBranchClass, 0, sceneBranch, domState->mObjectCreations, domState->mSceneContextHandleToRigidDynamicBranchMap);
    createOmniPvdBranchObject(sceneHandle, domState->mRigidStaticBranchClass, 0, sceneBranch, domState->mObjectCreations, domState->mSceneContextHandleToRigidStaticBranchMap);
    createOmniPvdBranchObject(sceneHandle, domState->mArticulationBranchClass, 0, sceneBranch, domState->mObjectCreations, domState->mSceneContextHandleToArticulationBranchMap);
    createOmniPvdBranchObject(sceneHandle, domState->mParticleSystemBranchClass, 0, sceneBranch, domState->mObjectCreations, domState->mSceneContextHandleToParticleSystemBranchMap);
}

OmniPvdObject* getRigidDynamicBranch(OmniPvdObjectHandle sceneHandle, OmniPvdObject *sceneBranch, OmniPvdDOMState* domState)
{
    OmniPvdObject* rigidDynamicBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToRigidDynamicBranchMap);
    if (!rigidDynamicBranch)
    {
        createSceneBranches(sceneHandle, sceneBranch, domState);
        rigidDynamicBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToRigidDynamicBranchMap);
    }
    return rigidDynamicBranch;
}

OmniPvdObject* getRigidStaticBranch(OmniPvdObjectHandle sceneHandle, OmniPvdObject *sceneBranch, OmniPvdDOMState* domState)
{
    OmniPvdObject* rigiStaticBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToRigidStaticBranchMap);
    if (!rigiStaticBranch)
    {
        createSceneBranches(sceneHandle, sceneBranch, domState);
        rigiStaticBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToRigidStaticBranchMap);
    }
    return rigiStaticBranch;
}

OmniPvdObject* getArticulationBranch(OmniPvdObjectHandle sceneHandle, OmniPvdObject *sceneBranch, OmniPvdDOMState* domState)
{
    OmniPvdObject* articulationBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToArticulationBranchMap);
    if (!articulationBranch)
    {
        createSceneBranches(sceneHandle, sceneBranch, domState);
        articulationBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToArticulationBranchMap);
    }
    return articulationBranch;
}

OmniPvdObject* getParticleSystemBranch(OmniPvdObjectHandle sceneHandle, OmniPvdObject* sceneBranch, OmniPvdDOMState* domState)
{
    OmniPvdObject* particleSystemBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToParticleSystemBranchMap);
    if (!particleSystemBranch)
    {
        createSceneBranches(sceneHandle, sceneBranch, domState);
        particleSystemBranch = findOmniPvdObject(sceneHandle, domState->mSceneContextHandleToParticleSystemBranchMap);
    }
    return particleSystemBranch;
}

void getAttribIndex(int32_t& attribIndex, int32_t& classIndex, const char* attribName, OmniPvdObject* omniPvdObject)
{
    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
    const int nbrInheritedClassesInClass = static_cast<int>(omniPvdObject->mOmniPvdClass->mInheritanceChain.size());
    if (nbrInheritedClasses > nbrInheritedClassesInClass)
    {
        // But this should not be possible to happen. How can the class that the object instantiates have
        // less inheritance than the object?
        CARB_LOG_ERROR("object.nbrInheritedClasses > class.nbrInheritedClasses (%d, %d)", nbrInheritedClasses, nbrInheritedClassesInClass);
        return;
    }
    for (int c = 0; c < nbrInheritedClasses; c++)
    {
        const OmniPvdClass* inheritedClassDef = omniPvdObject->mOmniPvdClass->mInheritanceChain[c];
        const int nbrAttributesInClass = static_cast<int>(inheritedClassDef->mAttributeDefinitions.size());
        for (int j = 0; j < nbrAttributesInClass; j++)
        {
            OmniPvdAttributeDef* attribDef = inheritedClassDef->mAttributeDefinitions[j];
            if (!attribDef->mIsUniqueList)
            {
                if (isSameString(attribDef->mAttributeName.c_str(), attribName))
                {
                    attribIndex = j;
                    classIndex = c;
                    return;
                }
            }
        }
    }
}

uint8_t* getAttribData(int32_t& attribIndex, int32_t& classIndex, const char* attribName, OmniPvdObject* omniPvdObject)
{
    // Is the class even in the inheritance chain of the object?

    if (attribIndex < 0 || classIndex < 0)
    {
        getAttribIndex(attribIndex, classIndex, attribName, omniPvdObject);
        if (attribIndex < 0 || classIndex < 0)
        {
            return NULL;
        }
    }
    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mOmniPvdClass->mInheritanceChain.size());

    if (classIndex >= nbrInheritedClasses)
    {
        CARB_LOG_ERROR("classIndex of the attribute does not fit in the object instance class (%d, %d)", classIndex, nbrInheritedClasses);
        return NULL;
    }
    if (omniPvdObject->mInheritedClassInstances.size() < 1)
    {
        CARB_LOG_ERROR("object has no inheritance chain (%s)", omniPvdObject->mOmniPvdClass->mClassName.c_str());
        return NULL;
    }

    std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[classIndex].mClassAttributeLists;
    const int nbrAttributes = static_cast<int>(classAttributeLists.size());
    if (attribIndex < nbrAttributes)
    {
        OmniPvdAttributeInstList *attributeInstList = classAttributeLists[attribIndex];
        if (attributeInstList)
        {
            OmniPvdAttributeSample *attrib = (OmniPvdAttributeSample*)attributeInstList->mFirst;
            return attrib->mData;
        }
    }
    return NULL;
}

OmniPvdAttributeInstList* getAttribList(int32_t& attribIndex, int32_t& classIndex , const char* attribName, OmniPvdObject* omniPvdObject)
{
    if (attribIndex < 0 || classIndex < 0)
    {
        getAttribIndex(attribIndex, classIndex, attribName, omniPvdObject);
        if (attribIndex < 0 || classIndex < 0)
        {
            return NULL;
        }
    }
    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mOmniPvdClass->mInheritanceChain.size());
    if (classIndex >= nbrInheritedClasses)
    {
        return NULL;
    }

    std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[classIndex].mClassAttributeLists;
    const size_t nbrAttributes = (size_t)classAttributeLists.size();
    if (attribIndex < nbrAttributes)
    {
        return classAttributeLists[attribIndex];
    }
    return NULL;
}

uint64_t getFrameIdFromScene(OmniPvdObject* object, OmniPvdDOMState* domState)
{
    // Does the DOM have a PxScene/context class registered?
    if (domState->mPxSceneClass == nullptr)
    {
        return 0;
    }
    // Is this object itself considered a context class?
    if (object->mOmniPvdClass == domState->mPxSceneClass)
    {
        return object->mFrameId;
    }
    OmniPvdObject* ancestorContext = object->findAncestorWithClass(domState->mPxSceneClass);
    if (ancestorContext)
    {
        return ancestorContext->mFrameId;
    }
    // Is there at least one PxScene? Then use the last created one's frameId
    if (domState->mSceneCreations.size()>0) {
        return domState->mSceneCreations.back()->mFrameId;
    }
    return 0;
}

uint64_t getLastFrameIdFromScene(OmniPvdDOMState* domState)
{
    // Is there at least one PxScene? Then use the last created one's frameId
    if (domState->mSceneCreations.size() > 0)
    {
        OmniPvdObject* lastObject = domState->mSceneCreations.back();
        return lastObject->mFrameId;
    }

    return 0;
}

bool isSameString(const char* str, const char* str1)
{
#ifdef WIN32
    return !_stricmp(str, str1);
#else
    return !strcasecmp(str, str1);    
#endif
}

bool isAlpha(char theChar)
{
    if ((theChar >= 'a') && (theChar <= 'z'))
        return true;
    if ((theChar >= 'A') && (theChar <= 'Z'))
        return true;
    return false;
}

bool isAlphaNum(char theChar)
{
    if ((theChar >= 'a') && (theChar <= 'z'))
        return true;
    if ((theChar >= 'A') && (theChar <= 'Z'))
        return true;
    if ((theChar >= '0') && (theChar <= '9'))
        return true;
    return false;
}

void getSanitizedName(std::string& sanitizedName, const char* objectName)
{
    if (!objectName) return;
    const int maxBuffLen = 512;
    char tmpBuffer[maxBuffLen];
    int buffStep = 0;
    int nbrValidChars = 0;
    while ((*objectName) && (buffStep < (maxBuffLen - 1)))
    {
        char daChar = *objectName;

        if (buffStep == 0)
        {
            if (isAlpha(daChar))
            {
                tmpBuffer[buffStep] = daChar;
                nbrValidChars++;
            }
            else
            {
                tmpBuffer[buffStep] = 'a';
            }
        }
        else
        {
            if (isAlphaNum(daChar))
            {
                tmpBuffer[buffStep] = daChar;
                nbrValidChars++;
            }
            else
            {
                tmpBuffer[buffStep] = '_';
            }
        }
        buffStep++;
        objectName++;
    }
    tmpBuffer[buffStep] = 0;
    sanitizedName = std::string(tmpBuffer);
}


bool isPowerOfTwo(uint32_t v)
{
    return (v && !(v & (v - 1)));
}

uint32_t nbrBitsSet(uint32_t v)
{
    uint32_t c; // store the total here
    uint32_t S[] = { 1, 2, 4, 8, 16 }; // Magic Binary Numbers
    uint32_t B[] = { 0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF };

    c = v - ((v >> 1) & B[0]);
    c = ((c >> S[1]) & B[1]) + (c & B[1]);
    c = ((c >> S[2]) + c) & B[2];
    c = ((c >> S[3]) + c) & B[3];
    c = ((c >> S[4]) + c) & B[4];

    return c;
}

uint32_t findFirstBitPos(uint32_t n)
{
    uint32_t count = 0;
    // One by one move the only set bit to right till it reaches end
    while (n)
    {
        if (n & 1) {
            return count;
        }
        n = n >> 1;
        count++;
    }
    return 32;
}
