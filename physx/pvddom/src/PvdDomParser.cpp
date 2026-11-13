// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "PvdDomParser.h"
#include "PvdDomParserConfig.h"
#include "PvdDomUtils.h"
#include "PvdDomLog.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include "OmniPvdFileReadStream.h"

#include <cstdio>
#include <memory>
#include <string.h>

// OmniPvdMessage, OmniPvdMessages, gOmniPvdMessages defined in PvdDomParser.h
OmniPvdMessages gOmniPvdMessages;

template <typename T>
static bool readAttributeValue(const uint8_t* data, uint32_t dataLen, T& value)
{
    if (!data || dataLen < sizeof(T))
    {
        PVDDOM_LOG_ERROR("   [b2s] Invalid attribute payload: expected at least %u bytes, got %u",
                         static_cast<unsigned>(sizeof(T)), dataLen);
        return false;
    }
    memcpy(&value, data, sizeof(T));
    return true;
}

template <typename T>
static bool readFirstAttributeValue(int32_t& attribIndex, int32_t& classIndex, const char* name,
                                    OmniPvdObject* object, T& value)
{
    OmniPvdAttributeInstList* list = getAttribList(attribIndex, classIndex, name, object);
    if (!list)
        return false;
    for (const OmniPvdAttributeInst* attribute = list->mFirst; attribute;
         attribute = attribute->mNextAttribute)
    {
        const OmniPvdAttributeSample* sample = static_cast<const OmniPvdAttributeSample*>(attribute);
        if (readAttributeValue(sample->mData, sample->mDataLen, value))
            return true;
    }
    return false;
}

static bool attachChildObjectFromPayload(const std::string& name, OmniPvdCommand::Enum cmdType,
                                         const OmniPvdAttributeDef* attributeDef, const uint8_t* data,
                                         uint32_t dataLen, OmniPvdObject* object, OmniPvdDOMState* domState)
{
    if (cmdType != OmniPvdCommand::eSET_ATTRIBUTE ||
        !isSameString(attributeDef->mAttributeName.c_str(), name.c_str()))
        return true;

    uint64_t childHandle;
    if (!readAttributeValue(data, dataLen, childHandle))
        return false;
    attachChildObject(name, cmdType, attributeDef, &childHandle, object,
                      domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
    return true;
}

static inline bool largerOrEqualOVDIntegversion(uint32_t major, uint32_t minor, uint32_t majorStream, uint32_t minorStream)
{
    if (majorStream > major) return true;
    if ((majorStream == major) && (minorStream >= minor)) return true;
    return false;
}


////////////////////////////////////////////////////////////////////////////////
// Config functions that handle different types of events
////////////////////////////////////////////////////////////////////////////////

void handleClassRegistration(OmniPvdReader* reader, OmniPvdDOMState* domState)
{
    PVDDOM_LOG_INFO("   [b2s] register class (classHandle: %d, name: %s)", (int)reader->getClassHandle(), reader->getClassName());
    ////////////////////////////////////////////////////////////////////////////////
    // Does the class already exist? If yes, then skip.
    ////////////////////////////////////////////////////////////////////////////////
    const OmniPvdClassHandle classHandle = reader->getClassHandle();
    OmniPvdClass *omniPvdClass = findOmniPvdClass(classHandle, domState->mClassHandleToClassMap);
    if (!omniPvdClass) {
        const char *className = reader->getClassName();

        const OmniPvdClassHandle baseClassHandle = reader->getBaseClassHandle();

        ////////////////////////////////////////////////////////////////////////////////
        // Create the OmniPvdClass object
        ////////////////////////////////////////////////////////////////////////////////
        std::unique_ptr<OmniPvdClass> newClass(new OmniPvdClass());
        omniPvdClass = newClass.get();
        omniPvdClass->mOmniClassHandle = classHandle;
        omniPvdClass->mClassName = std::string(className);

        domState->mClassHandleToClassMap[classHandle] = omniPvdClass;
        newClass.release();

        if (omniPvdClass->mClassName.compare("PxScene") == 0)
        {
            omniPvdClass->mIsSceneClass = true;
        }
        else if (omniPvdClass->mClassName.compare("PxArticulationReducedCoordinate") == 0)
        {
            domState->mPxArticulationReducedCoordinateClass = omniPvdClass;
        }
        else if (omniPvdClass->mClassName.compare("PxArticulationLink") == 0)
        {
            domState->mPxArticulationLinkClass = omniPvdClass;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Map the incoming PhysX class to the corresponding processing class if possible.
        // Will default to an XForm if the incoming classname is not recognized by the
        // class config map. Probably should be extended for inheritance, such that the
        // most close leaf class that has a class config mapping decides the class.
        //
        // PxActor -> Xform
        //   PxSomething -> -
        //     PxDynamic -> Geom
        //
        // PxActor obj -> Xform
        // PxSomething obj -> Xform
        // PxDynamic obj -> Geom
        //
        // Just good to Keep this in Mind (TM) :-)
        //
        ////////////////////////////////////////////////////////////////////////////////
        auto it = domState->mClassConfigMap.find(omniPvdClass->mClassName);
        if (it != domState->mClassConfigMap.end())
        {
            omniPvdClass->mPhysXBaseProcessingClassId = it->second;
            omniPvdClass->mIsDefaultParsed = false;
        }
        else
        {
            omniPvdClass->mPhysXBaseProcessingClassId = OmniPvdPhysXClassEnum::ePxUndefined;
            omniPvdClass->mIsDefaultParsed = true;
        }

        // Hacky override, fix this properly... which also didn't work
        if (
            //(!omniPvdClass->mClassName.compare("PxGeometry")) ||
            (!omniPvdClass->mClassName.compare("PxCustomGeometryExtConeCallbacks")) ||
            (!omniPvdClass->mClassName.compare("PxCustomGeometryExtCylinderCallbacks")) ||
            (!omniPvdClass->mClassName.compare("PxConvexCoreCylinder")) ||
            (!omniPvdClass->mClassName.compare("PxConvexCoreCone")))
        {
            omniPvdClass->mIsDefaultParsed = true;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Handle inheritance if any
        ////////////////////////////////////////////////////////////////////////////////
        if (baseClassHandle)
        {
            ////////////////////////////////////////////////////////////////////////////////
            // Checks for loops in the inheritance.
            // The leaf node class (the one inheriting) must not be found in the base class
            // inheritance chain.
            ////////////////////////////////////////////////////////////////////////////////
            const int32_t baseClassOffset = findOmniPvdClassOffset(baseClassHandle, classHandle, domState->mInheritedClassToClassOffset);
            if (baseClassOffset == -1)
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Is the base class handle a valid (registered) class? Then append its
                // inheritance chain.
                ////////////////////////////////////////////////////////////////////////////////
                OmniPvdClass *ancestorClass = findOmniPvdClass(baseClassHandle, domState->mClassHandleToClassMap);
                if (ancestorClass)
                {
                    const int nbrInheritedClassesFromBase = static_cast<int>(ancestorClass->mInheritanceChain.size());
                    omniPvdClass->mInheritanceChain.resize(nbrInheritedClassesFromBase + 1);
                    for (int c = 0; c < nbrInheritedClassesFromBase; c++)
                    {
                        omniPvdClass->mInheritanceChain[c] = ancestorClass->mInheritanceChain[c];
                        ////////////////////////////////////////////////////////////////////////////////
                        // Populate the instanceClass,attributeClass -> classOffset map
                        ////////////////////////////////////////////////////////////////////////////////
                        std::pair<uint64_t, uint64_t> key = { classHandle , ancestorClass->mInheritanceChain[c]->mOmniClassHandle };
                        domState->mInheritedClassToClassOffset[key] = c;
                    }
                    // Append the instance class to the tail of the inheritance chain
                    omniPvdClass->mInheritanceChain[nbrInheritedClassesFromBase] = omniPvdClass;
                    ////////////////////////////////////////////////////////////////////////////////
                    // Also populate the instance class offset (even if it's always last in the
                    // chain, just not to handle that with an exception in the code and have it
                    // fully "data driven". So the case that an attribute with an attributeClass
                    // equal to the instanceClass is used.
                    ////////////////////////////////////////////////////////////////////////////////
                    std::pair<uint64_t, uint64_t> key = { classHandle , classHandle };
                    domState->mInheritedClassToClassOffset[key] = nbrInheritedClassesFromBase;
                }
            }
            else
            {
                omniPvdClass->mInheritanceChain.resize(1);
                omniPvdClass->mInheritanceChain[0] = omniPvdClass;
                std::pair<uint64_t, uint64_t> key = { classHandle , classHandle };
                domState->mInheritedClassToClassOffset[key] = 0;
            }
        }
        else
        {
            omniPvdClass->mInheritanceChain.resize(1);
            omniPvdClass->mInheritanceChain[0] = omniPvdClass;
            std::pair<uint64_t, uint64_t> key = { classHandle , classHandle };
            domState->mInheritedClassToClassOffset[key] = 0;
        }
    }
    else
    {
        PVDDOM_LOG_WARN("   [b2s] Error : class (classHandle: %d) already registered. Class registration skipped.", (int)classHandle);
    }
}

void handleAttributeRegistration(OmniPvdReader* reader, const OmniPvdCommand::Enum cmdType, OmniPvdDOMState* domState)
{
    if (cmdType == OmniPvdCommand::eREGISTER_ATTRIBUTE)
    {
        PVDDOM_LOG_INFO("   [b2s] register attribute (classHandle: %d, attributeHandle: %d, dataType: %d, nbrFields: %d, name: %s)", (int)reader->getClassHandle(), (int)reader->getAttributeHandle(), reader->getAttributeDataType(), reader->getAttributeNumberElements(), reader->getAttributeName());
    }
    else
    {
        PVDDOM_LOG_INFO("   [b2s] register attributeSet (classHandle: %d, attributeHandle: %d, dataType: %d, name: %s)", (int)reader->getClassHandle(), (int)reader->getAttributeHandle(), reader->getAttributeDataType(), reader->getAttributeName());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Does the attribute definition exist already? Skip if that's the case.
    // OVD re-registrations of an attribute are skipped and issue a warning.
    ////////////////////////////////////////////////////////////////////////////////
    OmniPvdAttributeHandle attributeHandle = reader->getAttributeHandle();
    OmniPvdAttributeDef *omniPvdAttributeDef = findOmniPvdAttribute(attributeHandle, domState->mAttributeHandleToAttributeMap);
    if (!omniPvdAttributeDef)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Does the class already exist? If not, then skip.
        ////////////////////////////////////////////////////////////////////////////////
        OmniPvdClassHandle classHandle = reader->getClassHandle();
        OmniPvdClass *omniPvdClass = findOmniPvdClass(classHandle, domState->mClassHandleToClassMap);
        if (!omniPvdClass)
        {
            PVDDOM_LOG_WARN("   [b2s] Error : class (classHandle: %d) not yet registered. Attribute registration skipped.", (int)classHandle);
        }
        else
        {
            std::string attribName;
            getSanitizedName(attribName, reader->getAttributeName());

            const OmniPvdDataType::Enum attributeDataType = OmniPvdDataType::Enum(reader->getAttributeDataType());

            ////////////////////////////////////////////////////////////////////////////////
            // An Enum class only has attributes that are OmniPvdDataType::eENUM_VALUE
            ////////////////////////////////////////////////////////////////////////////////
            if (attributeDataType == OmniPvdDataType::eFLAGS_WORD)
            {
                OmniPvdClassHandle enumClassHandle = reader->getEnumClassHandle();
                OmniPvdClass *enumClass = findOmniPvdClass(enumClassHandle, domState->mClassHandleToClassMap);
                if (!enumClass)
                {
                    PVDDOM_LOG_WARN("   [b2s] Error : enum class (enumClassHandle: %d) not yet registered. FLAGS_WORD attribute registration skipped.", (int)enumClassHandle);
                }
                else if (enumClass->mIsEnumClass && (cmdType == OmniPvdCommand::eREGISTER_ATTRIBUTE))
                {
                    std::unique_ptr<OmniPvdAttributeDef> newAttribute(new OmniPvdAttributeDef());
                    omniPvdAttributeDef = newAttribute.get();
                    omniPvdAttributeDef->mOmniAttributeHandle = attributeHandle;

                    omniPvdAttributeDef->mIsUniqueList = 0;
                    uint32_t attributeNbrFields = reader->getAttributeNumberElements();
                    omniPvdAttributeDef->mNbrFields = attributeNbrFields;

                    omniPvdAttributeDef->mDataType = attributeDataType;
                    omniPvdAttributeDef->mAttributeName = attribName;

                    omniPvdAttributeDef->mAttributeId = PvdDomAttributeEnum::ePvdDomAttribEnum;

                    omniPvdAttributeDef->mClassOffset = (uint16_t)omniPvdClass->mAttributeDefinitions.size();
                    omniPvdAttributeDef->mClass = omniPvdClass;
                    omniPvdAttributeDef->mDerivedFromClass = enumClass;
                    domState->mAttributeHandleToAttributeMap[attributeHandle] = omniPvdAttributeDef;
                    newAttribute.release();
                    omniPvdClass->mAttributeDefinitions.push_back(omniPvdAttributeDef);
                }
            }
            else if (attributeDataType == OmniPvdDataType::eENUM_VALUE)
            {
                if (cmdType == OmniPvdCommand::eREGISTER_ATTRIBUTE)
                {
                    ////////////////////////////////////////////////////////////////////////////////
                    // An Enum class has all attributes of OmniPvdDataType::eENUM_VALUE
                    ////////////////////////////////////////////////////////////////////////////////
                    if (omniPvdClass->mIsEnumClass || (omniPvdClass->mAttributeDefinitions.size() == 0))
                    {
                        omniPvdClass->mIsEnumClass = true;
                        if (omniPvdClass->mAttributeDefinitions.size() == 0)
                        {
                            omniPvdClass->mBitFieldAttribs.resize(32);
                            for (int bitIndex = 0; bitIndex < 32; bitIndex++)
                            {
                                omniPvdClass->mBitFieldAttribs[bitIndex] = NULL;
                            }
                        }

                        // We are a go, for now
                        // Check that there is no other attribute definition with the same value
                        bool noOverlap = true;
                        const int nbrDef = (int)omniPvdClass->mAttributeDefinitions.size();
                        const uint32_t attributeVal = reader->getEnumValue();

                        for (int defIndex = 0; defIndex < nbrDef; defIndex++)
                        {
                            if (omniPvdClass->mAttributeDefinitions[defIndex])
                            {
                                if (omniPvdClass->mAttributeDefinitions[defIndex]->mNbrFields == attributeVal)
                                {
                                    noOverlap = false;
                                    break;
                                }
                            }
                        }
                        ////////////////////////////////////////////////////////////////////////////////
                        // We ignore any enum values that are the same as the ones we have already registered
                        ////////////////////////////////////////////////////////////////////////////////
                        if (noOverlap)
                        {
                            ////////////////////////////////////////////////////////////////////////////////
                            // An Enum class with any attribute that does not set exactly one bit -> cannot be a bitField
                            ////////////////////////////////////////////////////////////////////////////////
                            if (!(nbrBitsSet(attributeVal) == 1))
                            {
                                omniPvdClass->mIsBitFieldEnum = false;
                            }

                            std::unique_ptr<OmniPvdAttributeDef> newAttribute(new OmniPvdAttributeDef());
                            omniPvdAttributeDef = newAttribute.get();
                            omniPvdAttributeDef->mOmniAttributeHandle = attributeHandle;

                            omniPvdAttributeDef->mIsUniqueList = 0;
                            omniPvdAttributeDef->mNbrFields = attributeVal; // Override the nbrFields with the value of the enum

                            omniPvdAttributeDef->mDataType = attributeDataType;
                            omniPvdAttributeDef->mAttributeName = attribName;

                            omniPvdAttributeDef->mAttributeId = PvdDomAttributeEnum::ePvdDomAttribNone;

                            omniPvdAttributeDef->mClassOffset = (uint16_t)omniPvdClass->mAttributeDefinitions.size();
                            omniPvdAttributeDef->mClass = omniPvdClass;
                            ////////////////////////////////////////////////////////////////////////////////
                            // Update the OmniPvd DOM state
                            ////////////////////////////////////////////////////////////////////////////////
                            domState->mAttributeHandleToAttributeMap[attributeHandle] = omniPvdAttributeDef;
                            newAttribute.release();
                            omniPvdClass->mAttributeDefinitions.push_back(omniPvdAttributeDef);
                            if (isSameString(omniPvdClass->mClassName.c_str(),"actortype") || isSameString(omniPvdClass->mClassName.c_str(), "PxActorType")) // guard against case changes
                            {
                                if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"eRIGID_DYNAMIC"))
                                {
                                    domState->mActorTypeEnumRigidDynamic = attributeVal;
                                }
                                else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(), "eRIGID_STATIC"))
                                {
                                    domState->mActorTypeEnumRigidStatic = attributeVal;
                                }
                                else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(), "ePBD_PARTICLESYSTEM"))
                                {
                                    domState->mActorTypeEnumParticleSystem = attributeVal;
                                }
                                else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(), "eDEFORMABLE_VOLUME"))
                                {
                                    domState->mActorTypeEnumDeformableVolume = attributeVal;
                                }
                                else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(), "eDEFORMABLE_SURFACE"))
                                {
                                    domState->mActorTypeEnumDeformableSurface = attributeVal;
                                }
                            }


                            if (omniPvdClass->mIsBitFieldEnum) {
                                const int bitPos = findFirstBitPos(attributeVal);

                                /*
                                ////////////////////////////////////////////////////////////////////////////////
                                // Expand the biltfield attribs vector?
                                ////////////////////////////////////////////////////////////////////////////////
                                const int previousSize = (int)omniPvdClass->mBitFieldAttribs.size();
                                if (bitPos >= previousSize)
                                {
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Make sure the new tail of bit positions point to NULL
                                    ////////////////////////////////////////////////////////////////////////////////
                                    const int newSize = bitPos + 1;
                                    omniPvdClass->mBitFieldAttribs.resize(newSize);
                                    const int diffSize = newSize - previousSize;
                                    const int startIndex = previousSize;
                                    for (int diffIndex = 0; diffIndex < diffSize; diffIndex++)
                                    {
                                        omniPvdClass->mBitFieldAttribs[startIndex + diffIndex] = NULL;
                                    }
                                }
                                */
                                ////////////////////////////////////////////////////////////////////////////////
                                // Set the bitfield index of the enum value to point to the attribute definition
                                ////////////////////////////////////////////////////////////////////////////////
                                omniPvdClass->mBitFieldAttribs[bitPos] = omniPvdAttributeDef;
                            }
                        }
                    }
                }
            }
            // Make sure that classes that are not Enum classes have no attributes that are of OmniPvdDataType::eENUM_VALUE
            else if (!omniPvdClass->mIsEnumClass)
            {
                std::unique_ptr<OmniPvdAttributeDef> newAttribute(new OmniPvdAttributeDef());
                omniPvdAttributeDef = newAttribute.get();
                omniPvdAttributeDef->mOmniAttributeHandle = attributeHandle;
                if (cmdType == OmniPvdCommand::eREGISTER_ATTRIBUTE)
                {
                    omniPvdAttributeDef->mIsUniqueList = 0;
                    uint32_t attributeNbrFields = reader->getAttributeNumberElements();
                    omniPvdAttributeDef->mNbrFields = attributeNbrFields;
                }
                else
                {
                    omniPvdAttributeDef->mIsUniqueList = 1;
                    omniPvdAttributeDef->mNbrFields = 0;
                }
                omniPvdAttributeDef->mDataType = attributeDataType;
                omniPvdAttributeDef->mAttributeName = attribName;

                ////////////////////////////////////////////////////////////////////////////////
                // Assign either a predefined data type or a custom one
                ////////////////////////////////////////////////////////////////////////////////
                std::string classAttrib = omniPvdClass->mClassName + "." + omniPvdAttributeDef->mAttributeName;
                auto it = domState->mAttributeConfigMap.find(classAttrib);
                if (it != domState->mAttributeConfigMap.end())
                {
                    omniPvdAttributeDef->mAttributeId = it->second;
                }
                else
                {
                    omniPvdAttributeDef->mAttributeId = PvdDomAttributeEnum::ePvdDomAttribCustom;
                }

                omniPvdAttributeDef->mClassOffset = (uint16_t)omniPvdClass->mAttributeDefinitions.size();
                omniPvdAttributeDef->mClass = omniPvdClass;
                domState->mAttributeHandleToAttributeMap[attributeHandle] = omniPvdAttributeDef;
                newAttribute.release();
                omniPvdClass->mAttributeDefinitions.push_back(omniPvdAttributeDef);
            }
        }
    }
    else
    {
        PVDDOM_LOG_WARN("   [b2s] Error : attribute (attributeHandle: %d) already registered. Attribute registration skipped.", (int)attributeHandle);
    }
}

// Limit subtree walks for malformed cyclic input.
static const int kMaxLifespanWalkNodes = 1 << 20;

static void finishSupersedeSnapshots(OmniPvdDOMState* domState)
{
    for (OmniPvdObject* object : domState->mSupersedeSnapshotObjects)
    {
        object->mIsSupersedeRecreate = false;
    }
    domState->mSupersedeSnapshotObjects.clear();
}

static void beginRecordingSegment(OmniPvdDOMState* domState)
{
    if (!domState->mSawRecordingSegmentMetadata)
    {
        domState->mSawRecordingSegmentMetadata = true;
        return;
    }
    finishSupersedeSnapshots(domState);
    ++domState->mCurrentRecordingSegmentId;
    domState->mRecordingSegmentMaxFrames.push_back(0);
}

// Reuse an open span or append a new residency span.
static void openLifespan(OmniPvdObject* object, uint64_t openFrameId)
{
    if (object->mLifeSpans.empty() || (object->mLifeSpans.back().mFrameStop != 0))
    {
        object->mLifeSpans.resize(object->mLifeSpans.size() + 1);
    }
    object->mLifeSpans.back().mFrameStart = openFrameId;
    object->mReopenable = false;
}

// Close the last open residency span.
static void closeLifespan(OmniPvdObject* object, uint64_t closeFrameId)
{
    if (!object->mLifeSpans.empty() && (object->mLifeSpans.back().mFrameStop == 0))
    {
        object->mLifeSpans.back().mFrameStop = closeFrameId;
    }
}

// Close open descendant spans when a subtree is removed.
static void closeOpenDescendants(OmniPvdObject* subtreeRoot, uint64_t closeFrameId,
                                 OmniPvdDOMState* supersedeState = nullptr)
{
    std::vector<OmniPvdObject*> stack;
    if (subtreeRoot->mFirstChild)
    {
        stack.push_back(subtreeRoot->mFirstChild);
    }
    int visited = 0;
    while (!stack.empty() && visited < kMaxLifespanWalkNodes)
    {
        OmniPvdObject* object = stack.back();
        stack.pop_back();
        ++visited;
        if (!object->mLifeSpans.empty() && (object->mLifeSpans.back().mFrameStop == 0))
        {
            object->mLifeSpans.back().mFrameStop = supersedeState
                ? getSupersedeCloseFrameId(object, supersedeState)
                : closeFrameId;
            object->mReopenable = true;
        }
        if (object->mNextSibling)
        {
            stack.push_back(object->mNextSibling);
        }
        if (object->mFirstChild)
        {
            stack.push_back(object->mFirstChild);
        }
    }
    if (!stack.empty())
    {
        PVDDOM_LOG_WARN("   [b2s] Warning : subtree close walk truncated at %d nodes; remaining lifespans stay open", kMaxLifespanWalkNodes);
    }
}

// Reopen descendants closed by the previous ancestor removal.
static void reopenDescendantsClosedAt(OmniPvdObject* subtreeRoot, uint64_t closedAtFrameId, uint64_t openFrameId)
{
    std::vector<OmniPvdObject*> stack;
    if (subtreeRoot->mFirstChild)
    {
        stack.push_back(subtreeRoot->mFirstChild);
    }
    int visited = 0;
    while (!stack.empty() && visited < kMaxLifespanWalkNodes)
    {
        OmniPvdObject* object = stack.back();
        stack.pop_back();
        ++visited;
        bool reopened = false;
        if (object->mReopenable && !object->mLifeSpans.empty() && (object->mLifeSpans.back().mFrameStop == closedAtFrameId))
        {
            openLifespan(object, openFrameId);
            object->mReopenable = false;
            reopened = true;
        }
        if (object->mNextSibling)
        {
            stack.push_back(object->mNextSibling);
        }
        // Do not reopen descendants of a detached node.
        if (reopened && object->mFirstChild)
        {
            stack.push_back(object->mFirstChild);
        }
    }
    if (!stack.empty())
    {
        PVDDOM_LOG_WARN("   [b2s] Warning : subtree reopen walk truncated at %d nodes; remaining spans stay closed", kMaxLifespanWalkNodes);
    }
}

// Move zero-based descendant spans to the scene join frame.
static void restampZeroOpenDescendants(OmniPvdObject* subtreeRoot, uint64_t openFrameId)
{
    if (openFrameId == 0)
    {
        return;
    }
    std::vector<OmniPvdObject*> stack;
    if (subtreeRoot->mFirstChild)
    {
        stack.push_back(subtreeRoot->mFirstChild);
    }
    int visited = 0;
    while (!stack.empty() && visited < kMaxLifespanWalkNodes)
    {
        OmniPvdObject* object = stack.back();
        stack.pop_back();
        ++visited;
        if (!object->mLifeSpans.empty() &&
            (object->mLifeSpans.back().mFrameStart == 0) && (object->mLifeSpans.back().mFrameStop == 0))
        {
            object->mLifeSpans.back().mFrameStart = openFrameId;
        }
        if (object->mNextSibling)
        {
            stack.push_back(object->mNextSibling);
        }
        if (object->mFirstChild)
        {
            stack.push_back(object->mFirstChild);
        }
    }
    if (!stack.empty())
    {
        PVDDOM_LOG_WARN("   [b2s] Warning : subtree restamp walk truncated at %d nodes; remaining opens stay at 0", kMaxLifespanWalkNodes);
    }
}

// Detach the child and close its subtree.
static void detachChildSubtree(OmniPvdObject* child, uint64_t closeFrameId)
{
    closeLifespan(child, closeFrameId);
    closeOpenDescendants(child, closeFrameId);
    child->mReopenable = false;
}

// Reattach the child and reopen its subtree.
static void reattachChildSubtree(OmniPvdObject* child, uint64_t openFrameId)
{
    const uint64_t prevClose = child->mLifeSpans.empty() ? 0 : child->mLifeSpans.back().mFrameStop;
    openLifespan(child, openFrameId);
    restampZeroOpenDescendants(child, child->mLifeSpans.back().mFrameStart);
    if (prevClose != 0)
    {
        reopenDescendantsClosedAt(child, prevClose, child->mLifeSpans.back().mFrameStart);
    }
}

static void attachJointToScene(OmniPvdObject* joint, OmniPvdObject* scene, OmniPvdDOMState* domState)
{
    if (joint->mWasDestroyed || scene->mWasDestroyed || joint->mRecordingSegmentId != scene->mRecordingSegmentId) return;
    ++domState->mJointLookupSteps;
    OmniPvdObject* jointsNode = scene->mSceneJoints;
    if (!jointsNode)
    {
        std::unique_ptr<OmniPvdObject> node(createNamedObject(domState->mAttributeNameClass, "Joints",
                                                            scene->mRecordingSegmentId));
        jointsNode = node.get();
        jointsNode->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(scene, domState);
        domState->mObjectCreations.push_back(jointsNode);
        node.release();
        scene->appendChild(jointsNode);
        scene->mSceneJoints = jointsNode;
    }
    if (joint->mAncestor == jointsNode && !joint->mLifeSpans.empty() && !joint->mLifeSpans.back().mFrameStop) return;
    openLifespan(joint, getLifespanOpenFrameId(scene, domState));
    joint->mIsShared = false;
    if (joint->mAncestor != jointsNode) jointsNode->appendChild(joint);
}

void handleAttributeSetting(OmniPvdReader* reader, const OmniPvdCommand::Enum cmdType, OmniPvdDOMState* domState)
{
    int32_t parentLinkAttribIndex = -1;
    int32_t parentLinkClassIndex = -1;

    ////////////////////////////////////////////////////////////////////////////////
    // Does the object exist? If not, then skip.
    ////////////////////////////////////////////////////////////////////////////////
    OmniPvdObjectHandle objectHandle = getInternalHandle(reader->getObjectHandle(), domState->mExternalToInternalHandleMap);
    //OmniPvdContextHandle contextHandle = reader->getContextHandle();

    OmniPvdObject *omniPvdObject = findOmniPvdObject(objectHandle, domState->mObjectHandleToObjectMap);
    if (omniPvdObject)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Does the attribute definition exist?
        ////////////////////////////////////////////////////////////////////////////////
        const OmniPvdAttributeHandle attributeHandle = reader->getAttributeHandle();
        const OmniPvdAttributeDef* omniPvdAttributeDef = findOmniPvdAttribute(attributeHandle, domState->mAttributeHandleToAttributeMap);
        if (omniPvdAttributeDef)
        {
            const OmniPvdDataType::Enum dataEnumVal = OmniPvdDataType::Enum(omniPvdAttributeDef->mDataType);
            ////////////////////////////////////////////////////////////////////////////////
            // Is the attribute part of a class that is in the objects's inheritance list?
            // Check if the class where the attribute is registered (attributeClass), is
            // part of the map of inherited classes for the class of the object (instanceClass)
            ////////////////////////////////////////////////////////////////////////////////
            int32_t inherticanceClassOffset = findOmniPvdClassOffset(
                omniPvdObject->mOmniPvdClass->mOmniClassHandle,
                omniPvdAttributeDef->mClass->mOmniClassHandle,
                domState->mInheritedClassToClassOffset);
            if (inherticanceClassOffset < 0) {
                PVDDOM_LOG_ERROR("!!! ERROR !!!! object.class != attribute.class");
                PVDDOM_LOG_ERROR("attribute name: %s attribute class: %s object class: %s", omniPvdAttributeDef->mAttributeName.c_str(), omniPvdAttributeDef->mClass->mClassName.c_str(), omniPvdObject->mOmniPvdClass->mClassName.c_str());
            }
            else
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Check that:
                //   setAttribute command is called only on a non Attribute Set attribute
                //   addToAttributeSet and removeFromAttributeSet commands are called only on an Attribute Set attribute
                ////////////////////////////////////////////////////////////////////////////////
                bool validCommandForAttributeType = false;
                if (omniPvdAttributeDef->mIsUniqueList)
                {
                    if ((cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE) || (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE))
                    {
                        validCommandForAttributeType = true;
                    }
                }
                else {
                    if (cmdType == OmniPvdCommand::eSET_ATTRIBUTE)
                    {
                        validCommandForAttributeType = true;
                    }
                }

                ////////////////////////////////////////////////////////////////////////////////
                // If a valid command type for the attribute type, go ahead and parse out the
                // data package into an attribute instance of a certain object
                ////////////////////////////////////////////////////////////////////////////////
                if (validCommandForAttributeType)
                {
                    OmniPvdClassInstance& inheritedClassAttributes = omniPvdObject->mInheritedClassInstances[inherticanceClassOffset];
                    ////////////////////////////////////////////////////////////////////////////////
                    // Extend thes object attributes array if necessary, in case a class added
                    // attribute definitions after the object instanced it for example.
                    ////////////////////////////////////////////////////////////////////////////////
                    if (omniPvdAttributeDef->mClassOffset >= inheritedClassAttributes.mClassAttributeLists.size())
                    {
                        inheritedClassAttributes.mClassAttributeLists.resize(omniPvdAttributeDef->mClassOffset + 1, 0);
                    }

                    OmniPvdAttributeInstList *attributeList = inheritedClassAttributes.mClassAttributeLists[omniPvdAttributeDef->mClassOffset];
                    const uint8_t *data = reader->getAttributeDataPointer();
                    uint32_t dataLen = reader->getAttributeDataLength();
                    if (!data && dataLen != 0)
                    {
                        PVDDOM_LOG_ERROR("   [b2s] Invalid attribute payload: nonzero length with null data");
                        return;
                    }

                    ////////////////////////////////////////////////////////////////////////////////
                    // Deduplication of attribute setting step
                    ////////////////////////////////////////////////////////////////////////////////

                    if (!attributeList)
                    {
                        attributeList = new OmniPvdAttributeInstList();
                        inheritedClassAttributes.mClassAttributeLists[omniPvdAttributeDef->mClassOffset] = attributeList;
                        attributeList->mAttributeDef = const_cast<OmniPvdAttributeDef*>(omniPvdAttributeDef);
                    }

                    // Validate the attribute data
                    // enum single value : is the incoming OVD attribute value registered?
                    //    if so->then compare to previous and see if same or not
                    //    else ignore it
                    //        enum flag values : bitwise AND with all the registered flags > 0 ?
                    //        if so->then compare to previous and see if same or not
                    //        else ignore it
                    if (dataEnumVal == OmniPvdDataType::eFLAGS_WORD)
                    {
                        // Clean the flag values, to only contain registered values
                    }

                    uint64_t currentFrame = getFrameIdFromScene(omniPvdObject, domState);
                    // Keep sample timestamps monotonic.
                    if (attributeList && attributeList->mLast && (currentFrame < attributeList->mLast->mTimeStamp))
                    {
                        currentFrame = attributeList->mLast->mTimeStamp;
                    }

                    // No dedup: every write becomes its own keyframe with
                    // mTimeStamp == mEndTimeStamp == currentFrame. Earlier
                    // versions tried to collapse byte-identical successive
                    // writes by extending the previous sample's mEndTimeStamp,
                    // but that turned a simple "was data written at frame F?"
                    // question into a subtle one -- identical bytes on either
                    // side of a silent gap got bridged into a single keyframe,
                    // and strict-range readers (contacts) would falsely report
                    // data on the silent frames. Dropping dedup altogether
                    // makes mEndTimeStamp redundant with mTimeStamp; we keep
                    // the field so existing readers stay source-compatible.
                    OmniPvdAttributeInst *attributeInst;
                    if (cmdType == OmniPvdCommand::eSET_ATTRIBUTE)
                    {
                        attributeInst = new OmniPvdAttributeSample();
                    }
                    else
                    {
                        attributeInst = new OmniPvdUniqueList();
                    }
                    attributeInst->mTimeStamp = currentFrame; // omniPvdObject : -
                    attributeInst->mEndTimeStamp = currentFrame;

                    if (attributeList)
                    {
                        attributeList->addAttribute(attributeInst);
                    }

                    ////////////////////////////////////////////////////////////////////////////////
                    // If this is a regular Attribute simply save that data in (eSET_ATTRIBUTE)
                    // If it's an SetAttribute then we need to differentiate between adding or removing
                    // an element in the current set (eADD_TO_UNIQUE_LIST_ATTRIBUTE/eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                    ////////////////////////////////////////////////////////////////////////////////
                    if (cmdType == OmniPvdCommand::eSET_ATTRIBUTE)
                    {
                        ((OmniPvdAttributeSample*)attributeInst)->assignData(data, dataLen);
                    }
                    else if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                    {
                        ((OmniPvdUniqueList*)attributeInst)->addElement(data, dataLen);
                    }
                    else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                    {
                        ((OmniPvdUniqueList*)attributeInst)->removeElement(data, dataLen);
                    }

                    // This is basically the heart of the lookup of an objectHandlerRegister, yes

                    ////////////////////////////////////////////////////////////////////////////////
                    // The "hooking in" or parenting of an object doesn't happen until
                    //   * actors sets its shapes attribute
                    //       one shape at a time
                    //   * shape sets its geom attribute
                    //   * geomconvexmesh sets its convexmesh attribute
                    //   * geomheightfield sets its heightfield attribute
                    //   * geomtrianglemesh sets its trianglemesh attribute
                    //
                    // An added complexity is that geomconvexmesh, geomheightfield and geomtrianglemesh
                    // also add an internal reference object (not created by an OmniPvd API call),
                    // which is the object referencing the mesh object in the shared layer, that
                    // is the object that is referenced is not cloneable, or the one that is referencing
                    // is not the one that should be able to clone it
                    //
                    // On top of this not until we know if a Shape is exclusive or not can we for
                    // certain say if the following should be living in the shared layer or not:
                    //   geomsphere
                    //   geomxcapsule
                    //   geombox
                    //   geomplane
                    //   geomconvexmesh
                    //     geom
                    //   geomheightfield
                    //   geomtrianglemesh
                    //
                    ////////////////////////////////////////////////////////////////////////////////

                    switch (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId)
                    {
                    case OmniPvdPhysXClassEnum::ePxDeformableVolume:
                    case OmniPvdPhysXClassEnum::ePxDeformableSurface:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"type"))
                        {
                            uint32_t actorType;
                            if (!readAttributeValue(data, dataLen, actorType)) return;
                            omniPvdObject->mActortype = actorType;
                            omniPvdObject->mActortypeSet = true;
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"shapes"))
                        {
                            // Parent shapes under deformable actors, following the ePxActor pattern
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t shapeHandle;
                                if (!readAttributeValue(data, dataLen, shapeHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&shapeHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    if (!childObject->mIsShared)
                                    {
                                        // Use the deformable actor's own lifespan start so the shape
                                        // is visible from when the actor first appeared, not from when
                                        // the shapes ADD fires during fetchResults.
                                        reattachChildSubtree(childObject, omniPvdObject->mLifeSpans.back().mFrameStart);
                                        omniPvdObject->appendChild(childObject);
                                    }
                                    else
                                    {
                                        OmniPvdObject *refObject = findOmniPvdRefObject(objectHandle, shapeHandle, domState->mActorSharedShapeToShapeRefMap);
                                        if (!refObject)
                                        {
                                            std::unique_ptr<OmniPvdObject> newRef(createInternalObject(domState->mSharedShapeRefClass, 0,
                                                                                                        omniPvdObject->mRecordingSegmentId));
                                            refObject = newRef.get();
                                            refObject->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(omniPvdObject, domState);
                                            refObject->mReferenceObject = childObject;
                                            domState->mObjectCreations.push_back(refObject);
                                            omniPvdObject->appendChild(refObject);
                                            newRef.release();
                                            std::pair<uint64_t, uint64_t> key = { objectHandle , shapeHandle };
                                            domState->mActorSharedShapeToShapeRefMap[key] = refObject;
                                        }
                                        else
                                        {
                                            refObject->mRecordingSegmentId = omniPvdObject->mRecordingSegmentId;
                                            openLifespan(refObject, getLifespanOpenFrameId(omniPvdObject, domState));
                                        }
                                    }
                                }
                            }
                            else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t shapeHandle;
                                if (!readAttributeValue(data, dataLen, shapeHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&shapeHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    if (!childObject->mIsShared)
                                    {
                                        detachChildSubtree(childObject, getLifespanCloseFrameId(omniPvdObject, domState));
                                    }
                                    else
                                    {
                                        OmniPvdObject *refObject = findOmniPvdRefObject(objectHandle, shapeHandle, domState->mActorSharedShapeToShapeRefMap);
                                        if (refObject)
                                        {
                                            closeLifespan(refObject, getLifespanCloseFrameId(omniPvdObject, domState));
                                        }
                                    }
                                }
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"simulationMeshShapes"))
                        {
                            // For volumes: create synthetic SimShape > SimGeometry > mesh_ref hierarchy
                            // for the simulation tet mesh, replicating the collision mesh shape path.
                            // Streamed from fetchResults (deferred) since the tet mesh object may not
                            // exist in the DOM when attachSimulationMesh fires during scene setup.
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE &&
                                omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableVolume)
                            {
                                uint64_t meshHandle;
                                if (!readAttributeValue(data, dataLen, meshHandle)) return;
                                OmniPvdObject* meshObject = findChildObject(&meshHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (meshObject && !omniPvdObject->getChild("SimulationMesh"))
                                {
                                    uint64_t frameStart = omniPvdObject->mLifeSpans.back().mFrameStart;

                                    std::unique_ptr<OmniPvdObject> newNode(createNamedObject(domState->mSimulationMeshClass, "SimulationMesh",
                                                                                               omniPvdObject->mRecordingSegmentId));
                                    OmniPvdObject* simMeshNode = newNode.get();
                                    simMeshNode->mLifeSpans[0].mFrameStart = frameStart;
                                    domState->mObjectCreations.push_back(simMeshNode);
                                    omniPvdObject->appendChild(simMeshNode);
                                    newNode.release();

                                    std::unique_ptr<OmniPvdObject> newRef(createInternalObject(domState->mTetrahedronMeshRefClass, 0,
                                                                                                 omniPvdObject->mRecordingSegmentId));
                                    OmniPvdObject* refObject = newRef.get();
                                    refObject->mLifeSpans[0].mFrameStart = frameStart;
                                    refObject->mReferenceObject = meshObject;
                                    meshObject->mIsReferenced = true;
                                    domState->mObjectCreations.push_back(refObject);
                                    simMeshNode->appendChild(refObject);
                                    newRef.release();
                                }
                            }
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxJoint:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"constraint"))
                        {
                            uint64_t constraintHandle;
                            if (!readAttributeValue(data, dataLen, constraintHandle)) return;
                            domState->mConstraintToJointMap[constraintHandle] = omniPvdObject;
                            const auto scene = domState->mConstraintToSceneMap.find(constraintHandle);
                            if (scene != domState->mConstraintToSceneMap.end())
                                attachJointToScene(omniPvdObject, scene->second, domState);
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxArticulationJoint:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"parentLink"))
                        {

                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"childLink"))
                        {
                            ////////////////////////////////////////////////////////////////////////////////
                            // childLink->ancestor = parentLink
                            ////////////////////////////////////////////////////////////////////////////////
                            uint64_t actorHandle;
                            if (!readAttributeValue(data, dataLen, actorHandle)) return;
                            OmniPvdObject *childObject = findChildObject(&actorHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                            if (childObject)
                            {
                                ////////////////////////////////////////////////////////////////////////////////
                                // find the parentLink attribute data
                                ////////////////////////////////////////////////////////////////////////////////
                                OmniPvdObjectHandle parentHandle;
                                if (readFirstAttributeValue(parentLinkAttribIndex, parentLinkClassIndex,
                                                            "parentLink", omniPvdObject, parentHandle))
                                {
                                    OmniPvdObject *ancestorObj = findChildObject(&parentHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                    if (ancestorObj)
                                    {
                                        ancestorObj->appendChild(childObject);
                                    }
                                }
                                childObject->insertChildFirst(omniPvdObject);
                            }
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxPhysics:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"scenes"))
                        {
                            // Parent each scene under its owning PxPhysics object.
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t sceneHandle;
                                if (!readAttributeValue(data, dataLen, sceneHandle)) return;
                                OmniPvdObject *sceneObject = findChildObject(&sceneHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (sceneObject)
                                {
                                    omniPvdObject->appendChild(sceneObject);
                                }
                            }
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxScene:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"actors"))
                        {
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t actorHandle;
                                if (!readAttributeValue(data, dataLen, actorHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&actorHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    const uint64_t joinFrame = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : PxScene
                                    const uint64_t prevResidencyClose = childObject->mLifeSpans.empty() ? 0 : childObject->mLifeSpans.back().mFrameStop;
                                    openLifespan(childObject, joinFrame);
                                    restampZeroOpenDescendants(childObject, joinFrame);
                                    if (prevResidencyClose != 0)
                                    {
                                        reopenDescendantsClosedAt(childObject, prevResidencyClose, joinFrame);
                                    }
                                    OmniPvdObject *leAncestor = nullptr;
                                    if (!childObject->mActortypeSet)
                                    {
                                        // Untyped actors use the scene fallback below.
                                    }
                                    else if (childObject->mActortype == domState->mActorTypeEnumRigidDynamic)
                                    {
                                        leAncestor = getRigidDynamicBranch(objectHandle, omniPvdObject, domState);
                                    }
                                    else if (childObject->mActortype == domState->mActorTypeEnumRigidStatic)
                                    {
                                        leAncestor = getRigidStaticBranch(objectHandle, omniPvdObject, domState);
                                    }
                                    else if (childObject->mActortype == domState->mActorTypeEnumParticleSystem)
                                    {
                                        leAncestor = getParticleSystemBranch(objectHandle, omniPvdObject, domState);
                                    }
                                    else if (childObject->mActortype == domState->mActorTypeEnumDeformableVolume)
                                    {
                                        leAncestor = getDeformableVolumeBranch(objectHandle, omniPvdObject, domState);
                                    }
                                    else if (childObject->mActortype == domState->mActorTypeEnumDeformableSurface)
                                    {
                                        leAncestor = getDeformableSurfaceBranch(objectHandle, omniPvdObject, domState);
                                    }
                                    if (!leAncestor)
                                    {
                                        // Parent untyped actors directly under the scene.
                                        leAncestor = omniPvdObject;
                                    }
                                    leAncestor->appendChild(childObject);
                                }
                            }
                            else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t actorHandle;
                                if (!readAttributeValue(data, dataLen, actorHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&actorHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    const uint64_t leaveFrame = getLifespanCloseFrameId(omniPvdObject, domState); // omniPvdObject : PxScene
                                    detachChildSubtree(childObject, leaveFrame);
                                }
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"articulations"))
                        {
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t actorHandle;
                                if (!readAttributeValue(data, dataLen, actorHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&actorHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    const uint64_t joinFrame = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : PxScene
                                    const uint64_t prevResidencyClose = childObject->mLifeSpans.empty() ? 0 : childObject->mLifeSpans.back().mFrameStop;
                                    openLifespan(childObject, joinFrame);
                                    restampZeroOpenDescendants(childObject, joinFrame);
                                    if (prevResidencyClose != 0)
                                    {
                                        reopenDescendantsClosedAt(childObject, prevResidencyClose, joinFrame);
                                    }
                                    OmniPvdObject *leAncestor = getArticulationBranch(objectHandle, omniPvdObject, domState);
                                    if (leAncestor)
                                    {
                                        leAncestor->appendChild(childObject);
                                    }
                                }
                            }
                            else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t actorHandle;
                                if (!readAttributeValue(data, dataLen, actorHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&actorHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    const uint64_t leaveFrame = getLifespanCloseFrameId(omniPvdObject, domState); // omniPvdObject : PxScene
                                    detachChildSubtree(childObject, leaveFrame);
                                }
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"constraints"))
                        {
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t constraintHandle;
                                if (!readAttributeValue(data, dataLen, constraintHandle)) return;
                                domState->mConstraintToSceneMap[constraintHandle] = omniPvdObject;
                                const auto joint = domState->mConstraintToJointMap.find(constraintHandle);
                                if (joint != domState->mConstraintToJointMap.end())
                                    attachJointToScene(joint->second, omniPvdObject, domState);
                            }
                            else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                            {
                                // Find the joint in the constraint->joint map and remove the joint from the scene
                                uint64_t constraintHandle;
                                if (!readAttributeValue(data, dataLen, constraintHandle)) return;
                                const auto joint = domState->mConstraintToJointMap.find(constraintHandle);
                                if (joint != domState->mConstraintToJointMap.end()) {
                                    // remove the joint from the scene
                                    closeLifespan(joint->second, getLifespanCloseFrameId(omniPvdObject, domState));
                                }
                                domState->mConstraintToSceneMap.erase(constraintHandle);
                            }
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxActor:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"shapes"))
                        {
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                // *data contains the ID or handle of the shape. In the case this is an exclusive Shape then it is indeed directly
                                // the child of the Actor object. However in the case it's a shared shape then the Actor must bridge this connection
                                // with a reference object that points to a different layer

                                uint64_t shapeHandle;
                                if (!readAttributeValue(data, dataLen, shapeHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&shapeHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    if (!childObject->mIsShared)
                                    {
                                        reattachChildSubtree(childObject, getLifespanOpenFrameId(omniPvdObject, domState)); // omniPvdObject : PxActor
                                        omniPvdObject->appendChild(childObject);
                                    }
                                    else
                                    {
                                        ////////////////////////////////////////////////////////////////////////////////
                                        // It's a shared shape, create a reference object, that in turn points to the shared shape
                                        // but first check if there is already a reference object to this shared shape for this actor
                                        // actorSharedShapeMap -> sharedShapeRef object
                                        // If we find that we already have such a reference object, we need to add a lifeSpan keyframe to the
                                        // lifeSpan vector/list of the object. Right. We also keep the same pointer to the shared shape
                                        // The shared shape reference object should be called
                                        // shape_[shapeId]_ref_[sharedShapeId]
                                        // Yeah that is it! That was the nut to crack :-)
                                        ////////////////////////////////////////////////////////////////////////////////
                                        OmniPvdObject *refObject = findOmniPvdRefObject(objectHandle, shapeHandle, domState->mActorSharedShapeToShapeRefMap);
                                        if (!refObject)
                                        {
                                            std::unique_ptr<OmniPvdObject> newRef(createInternalObject(domState->mSharedShapeRefClass, 0,
                                                                                                        omniPvdObject->mRecordingSegmentId));
                                            refObject = newRef.get();
                                            refObject->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : PxActor
                                            refObject->mReferenceObject = childObject;
                                            domState->mObjectCreations.push_back(refObject);
                                            omniPvdObject->appendChild(refObject);
                                            newRef.release();
                                            // Insert the shape reference into the actor|sharedShape map
                                            std::pair<uint64_t, uint64_t> key = { objectHandle , shapeHandle };
                                            domState->mActorSharedShapeToShapeRefMap[key] = refObject;
                                        }
                                        else
                                        {
                                            // Create yet another life time key in the refObject
                                            refObject->mRecordingSegmentId = omniPvdObject->mRecordingSegmentId;
                                            openLifespan(refObject, getLifespanOpenFrameId(omniPvdObject, domState)); // omniPvdObject : PxActor
                                        }
                                    }
                                }
                            }
                            else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t shapeHandle;
                                if (!readAttributeValue(data, dataLen, shapeHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&shapeHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    if (!childObject->mIsShared)
                                    {
                                        detachChildSubtree(childObject, getLifespanCloseFrameId(omniPvdObject, domState)); // omniPvdObject : PxActor
                                    }
                                    else
                                    {
                                        OmniPvdObject *refObject = findOmniPvdRefObject(objectHandle, shapeHandle, domState->mActorSharedShapeToShapeRefMap);
                                        if (refObject)
                                        {
                                            closeLifespan(refObject, getLifespanCloseFrameId(omniPvdObject, domState)); // omniPvdObject : PxActor
                                        }
                                    }
                                }
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"type"))
                        {
                            ////////////////////////////////////////////////////////////////////////////////
                            // Should bucket into :
                            //    rigid_static
                            //    rigid_dynamic
                            //    articulation_link
                            //    soft_body
                            //    fem_cloth
                            //    pbd_particlesystem
                            ////////////////////////////////////////////////////////////////////////////////
                            uint32_t actorType;
                            if (!readAttributeValue(data, dataLen, actorType)) return;
                            omniPvdObject->mActortype = actorType;
                            omniPvdObject->mActortypeSet = true;
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"articulation"))
                        {
                            // which articulation to attach the link to
                            uint64_t shapeHandle;
                            if (!readAttributeValue(data, dataLen, shapeHandle)) return;
                            OmniPvdObject *childObject = findChildObject(&shapeHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                            if (childObject)
                            {
                                childObject->appendChild(omniPvdObject);
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"inboundJoint"))
                        {
                            ////////////////////////////////////////////////////////////////////////////////
                            // parentLink  : ancestorObj
                            //   childLink : omniPvdObject <- append the childLink to the parentLink
                            //     joint   : jointObject  <- make sure that the joint appears first in the list of objects
                            ////////////////////////////////////////////////////////////////////////////////
                            uint64_t jointHandle;
                            if (!readAttributeValue(data, dataLen, jointHandle)) return;
                            OmniPvdObject *jointObject = findChildObject(&jointHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                            if (jointObject)
                            {
                                ////////////////////////////////////////////////////////////////////////////////
                                // Get the parentLink object
                                ////////////////////////////////////////////////////////////////////////////////
                                OmniPvdObjectHandle parentHandle;
                                if (readFirstAttributeValue(parentLinkAttribIndex, parentLinkClassIndex,
                                                            "parentLink", jointObject, parentHandle))
                                {
                                    OmniPvdObject *ancestorObj = findChildObject(&parentHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                    if (ancestorObj)
                                    {
                                        ancestorObj->appendChild(omniPvdObject);
                                    }
                                }
                                omniPvdObject->insertChildFirst(jointObject);
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(), "particleBuffers"))
                        {
                            if (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t bufferHandle;
                                if (!readAttributeValue(data, dataLen, bufferHandle)) return;
                                OmniPvdObject* childObject = findChildObject(&bufferHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    reattachChildSubtree(childObject, getLifespanOpenFrameId(omniPvdObject, domState)); // omniPvdObject : PxActor
                                    omniPvdObject->appendChild(childObject);
                                }
                            }
                            else if (cmdType == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)
                            {
                                uint64_t bufferHandle;
                                if (!readAttributeValue(data, dataLen, bufferHandle)) return;
                                OmniPvdObject* childObject = findChildObject(&bufferHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    detachChildSubtree(childObject, getLifespanCloseFrameId(omniPvdObject, domState)); // omniPvdObject : PxActor
                                }
                            }
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxShape:
                    {
                        if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"isExclusive"))
                        {
                            uint8_t isExclusive;
                            if (!readAttributeValue(data, dataLen, isExclusive)) return;
                            if (!isExclusive) // isExclusive == false -> shared Shape
                            {
                                omniPvdObject->mIsShared = 1;
                                if (domState->mSharedShapesBranch)
                                {
                                    domState->mSharedShapesBranch->insertChildFirst(omniPvdObject); // Switch the shape from exclusive to shared and put it into the shared layer
                                }
                                omniPvdObject->mIsStaticVisibility = 1;
                                omniPvdObject->mIsStaticVisible = 1;
                            }
                            else {
                            }
                        }
                        else if (isSameString(omniPvdAttributeDef->mAttributeName.c_str(),"geom"))
                        {
                            if (cmdType == OmniPvdCommand::eSET_ATTRIBUTE)
                            {
                                uint64_t childHandle;
                                if (!readAttributeValue(data, dataLen, childHandle)) return;
                                OmniPvdObject *childObject = findChildObject(&childHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                if (childObject)
                                {
                                    // Restamp only a newly created geometry object.
                                    if (!childObject->mLifeSpans.empty() &&
                                        (childObject->mLifeSpans[0].mFrameStart == 0) && (childObject->mLifeSpans[0].mFrameStop == 0))
                                    {
                                        childObject->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : PxShape
                                    }
                                    omniPvdObject->insertChildFirst(childObject);
                                    // The convexmesh/heighfield/trianglemesh are always shared but if the geom is sphere/capsule/box/
                                    if (omniPvdObject->mIsShared)
                                    {
                                        childObject->mIsShared = 1;
                                        if (childObject->mReferenceObject)
                                        {
                                            childObject->mReferenceObject->mIsShared = 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxGeomConvexMesh:
                    {
                        static const std::string str("convexmesh");
                        if (!attachChildObjectFromPayload(str, cmdType, omniPvdAttributeDef, data, dataLen, omniPvdObject, domState)) return;
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxGeomHeightfield:
                    {
                        static const std::string str("heightfield");
                        if (!attachChildObjectFromPayload(str, cmdType, omniPvdAttributeDef, data, dataLen, omniPvdObject, domState)) return;
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxGeomTriangleMesh:
                    {
                        static const std::string str("trianglemesh");
                        if (!attachChildObjectFromPayload(str, cmdType, omniPvdAttributeDef, data, dataLen, omniPvdObject, domState)) return;
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxGeomTetMesh:
                    {
                        static const std::string str("tetrahedronMesh");
                        if (!attachChildObjectFromPayload(str, cmdType, omniPvdAttributeDef, data, dataLen, omniPvdObject, domState)) return;
                    }
                    break;
                    case OmniPvdPhysXClassEnum::ePxConvexCoreGeometry:
                    {
                        static const std::string str("core");
                        if (!attachChildObjectFromPayload(str, cmdType, omniPvdAttributeDef, data, dataLen, omniPvdObject, domState)) return;
                    }
                    break;
                    default:
                    {
                        ////////////////////////////////////////////////////////////////////////////////
                        // Only end up here if the class is not part of the special processing classes
                        ////////////////////////////////////////////////////////////////////////////////
                        if (omniPvdObject->mOmniPvdClass->mIsDefaultParsed)
                        {
                            if (dataEnumVal == OmniPvdDataType::eOBJECT_HANDLE)
                            {
                                int nbrRefObjectHandles = dataLen / sizeof(uint64_t); // use objecthandle types
                                if (nbrRefObjectHandles > 0)
                                {
                                    if ((cmdType == OmniPvdCommand::eSET_ATTRIBUTE)
                                        || (cmdType == OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE))
                                    {
                                        //printf("settHandles class:attribute (%s):(%s)\n", omniPvdObject->mOmniPvdClass->mClassName.c_str(), omniPvdAttributeDef->mAttributeName.c_str());

                                        for (int i = 0; i < nbrRefObjectHandles; i++)
                                        {
                                            ////////////////////////////////////////////////////////////////////////////////
                                            // object
                                            //  attribNameObject   found using key(internalObjectHandle_1, attributeHandle)
                                            //    refNameObject    found using key(internalObjectHandle_1 + attributeHandle, internalObjectHandle_2)
                                            //    refNameObject    -"-
                                            //    refNameObject    -"-
                                            ////////////////////////////////////////////////////////////////////////////////
                                            // NOTE : These keys could potentially just be a single large key of n bytes,
                                            //        which is how the CRC or key value is computed in any case
                                            ////////////////////////////////////////////////////////////////////////////////
                                            uint64_t refObjectHandle;
                                            memcpy(&refObjectHandle, data + i * sizeof(uint64_t), sizeof(refObjectHandle));
                                            OmniPvdObject *attribNameObject = 0;
                                            OmniPvdObject *referencedObject = findChildObject(&refObjectHandle, domState->mExternalToInternalHandleMap, domState->mObjectHandleToObjectMap);
                                            ////////////////////////////////////////////////////////////////////////////////
                                            // Does this object exist in the OmniPVD DOM, that is did a user create it before
                                            // it was referenced?
                                            // Also only allow referencing of also default parsed objects. So a default parsed
                                            // object cannot reference custom parsed objects.
                                            ////////////////////////////////////////////////////////////////////////////////
                                            if (referencedObject && referencedObject->mOmniPvdClass->mIsDefaultParsed)
                                            {
                                                ////////////////////////////////////////////////////////////////////////////////
                                                // This ensures we only do this test once per loop, as potentially one command
                                                // could contain thousands of references for example.
                                                ////////////////////////////////////////////////////////////////////////////////
                                                // NOTE : Compress into a function, for code safety
                                                ////////////////////////////////////////////////////////////////////////////////
                                                if (!attribNameObject)
                                                {
                                                    attribNameObject = findOmniPvdRefObject(omniPvdObject->mOmniObjectHandle, attributeHandle, domState->mObjectAttributeNameMap);
                                                    ////////////////////////////////////////////////////////////////////////////////
                                                    // Use an existing attriNamebObject?
                                                    ////////////////////////////////////////////////////////////////////////////////
                                                    if (!attribNameObject)
                                                    {
                                                        std::unique_ptr<OmniPvdObject> newName(createNamedObject(domState->mAttributeNameClass,
                                                                                                                   omniPvdAttributeDef->mAttributeName,
                                                                                                                   omniPvdObject->mRecordingSegmentId));
                                                        attribNameObject = newName.get();
                                                        attribNameObject->mSkipMembershipTraversal = true;
                                                        ////////////////////////////////////////////////////////////////////////////////
                                                        // Make sure that the attribNameObject becomes shared if the parent object is shared
                                                        ////////////////////////////////////////////////////////////////////////////////
                                                        if (omniPvdObject->mIsShared)
                                                        {
                                                            attribNameObject->mIsShared = 1;
                                                        }
                                                        attribNameObject->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : -

                                                        domState->mObjectCreations.push_back(attribNameObject);
                                                        omniPvdObject->appendChild(attribNameObject);
                                                        newName.release();
                                                        ////////////////////////////////////////////////////////////////////////////////
                                                        // Insert the attribute name object into the object|attribute map
                                                        ////////////////////////////////////////////////////////////////////////////////
                                                        std::pair<uint64_t, uint64_t> key = { omniPvdObject->mOmniObjectHandle , attributeHandle };
                                                        domState->mObjectAttributeNameMap[key] = attribNameObject;
                                                    }
                                                }
                                                ////////////////////////////////////////////////////////////////////////////////
                                                // Now we have the attribute name node, now we can add object reference node(s)
                                                ////////////////////////////////////////////////////////////////////////////////
                                                uint64_t keyObjectAttribLow = attributeHandle;
                                                uint64_t keyObjectAttribHigh = omniPvdObject->mOmniObjectHandle; // So not more than 4 billion objects? Hmmmm...
                                                keyObjectAttribHigh = keyObjectAttribHigh << 32;
                                                uint64_t keyObjectAttribCombined = keyObjectAttribHigh | keyObjectAttribLow;
                                                OmniPvdObject *refNameObject = findOmniPvdRefObject(keyObjectAttribCombined, refObjectHandle, domState->mObjectAttributeRefMap);
                                                if (!refNameObject)
                                                {
                                                    std::unique_ptr<OmniPvdObject> newRef(createInternalObject(domState->mAttributeRefClass, refObjectHandle,
                                                                                                                 omniPvdObject->mRecordingSegmentId));
                                                    refNameObject = newRef.get();
                                                    refNameObject->mOmniObjectHandle = referencedObject->mOmniAPIHandle;
                                                    if (omniPvdObject->mIsShared)
                                                    {
                                                        refNameObject->mIsShared = 1;
                                                    }
                                                    referencedObject->mIsShared = 1;
                                                    refNameObject->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : -

                                                    domState->mObjectCreations.push_back(refNameObject);
                                                    attribNameObject->appendChild(refNameObject);
                                                    newRef.release();

                                                    refNameObject->mReferenceObject = referencedObject;
                                                    ////////////////////////////////////////////////////////////////////////////////
                                                    // Insert the attribute name object into the object+attribute|refObject map
                                                    ////////////////////////////////////////////////////////////////////////////////
                                                    std::pair<uint64_t, uint64_t> key = { keyObjectAttribCombined , refObjectHandle };
                                                    domState->mObjectAttributeRefMap[key] = referencedObject;
                                                }
                                                else
                                                {
                                                    referencedObject->mIsShared = 1;
                                                    // Create yet another life time key in the refObject
                                                    openLifespan(refNameObject, getLifespanOpenFrameId(omniPvdObject, domState)); // omniPvdObject : -
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                    }
                }
            }
        }
    }
    else {

    }
}

// Attach any yet unattached root articulation links (that reference the articulation) to the articulation
void attachFloatingRootLinksToArticulation(OmniPvdDOMState& domState, OmniPvdObject& articulation)
{
    if (domState.mPxArticulationLinkClass == nullptr) return; // No articulationLink class was yet registered
    int32_t attribIndex = -1;
    int32_t classIndex = -1;
    std::list<OmniPvdObject*>::iterator it;
    for (it = domState.mObjectCreations.begin(); it != domState.mObjectCreations.end(); it++)
    {
        OmniPvdObject* link = (*it);
        if (link)
        {
            if (link->mOmniPvdClass == domState.mPxArticulationLinkClass)
            {
                // Does the link not have an ancestor object set?
                if (link->mAncestor == nullptr)
                {
                    // Does the link reference the articulation?
                    OmniPvdObjectHandle articulationtHandle;
                    if (readFirstAttributeValue(attribIndex, classIndex, "articulation", link,
                                                articulationtHandle))
                    {
                        if (articulationtHandle == articulation.mOmniAPIHandle)
                        {
                            articulation.appendChild(link);
                        }
                    }
                }
            }
        }
    }
}

void handleObjectCreation(OmniPvdReader* reader, const OmniPvdCommand::Enum cmdType, OmniPvdDOMState* domState)
{
    // Validate metadata after the first stream object is complete.
    if (!domState->mOvdIntegVersionWasChecked)
    {
        if (domState->mNbStreamObjectCreations == 1)
        {
            OmniPvdObject* obj = domState->mFirstStreamObject;
            int32_t attribIndexMajor = -1;
            int32_t classIndex = -1;
            uint32_t versionMajorStream;
            if (readFirstAttributeValue(attribIndexMajor, classIndex, "ovdIntegrationVersionMajor", obj,
                                        versionMajorStream))
            {
                domState->mStreamOvdIntegVersionMajor = versionMajorStream;

                if (versionMajorStream <=  domState->mOvdIntegrationVersionMajor)
                {
                    domState->mOvdIntegVersionPassed = true;
                }
                else
                {
                    domState->mOvdIntegVersionPassed = false;
                    domState->mOvdIntegVersionWasChecked = true;
                    PVDDOM_LOG_WARN("   [b2s] Error : PhysX OVD integration version major is too high, not able to parse the OVD object stream confidently");
                    return;
                }
            }
            else
            {
                domState->mOvdIntegVersionPassed = true;
            }
            domState->mOvdIntegVersionWasChecked = true;

            int32_t attribIndexMinor = -1;
            uint32_t versionMinorStream;
            if (readFirstAttributeValue(attribIndexMinor, classIndex, "ovdIntegrationVersionMinor", obj,
                                        versionMinorStream))
            {
                domState->mStreamOvdIntegVersionMinor = versionMinorStream;
            }
        }
    }

    // If an object with the handle already exists, make that one invisible unless it was already set to invisible
    OmniPvdObjectHandle externalObjectHandle = reader->getObjectHandle();
    if (externalObjectHandle == 0)
    {
        PVDDOM_LOG_WARN("   [b2s] Error : object handle is zero");
        return;
    }
    OmniPvdClassHandle classHandle = reader->getClassHandle();
    OmniPvdClass* omniPvdClass = findOmniPvdClass(classHandle, domState->mClassHandleToClassMap);
    if (omniPvdClass && omniPvdClass->mClassName == "PxOmniPvdMetaData")
    {
        beginRecordingSegment(domState);
    }

    bool objectHandleWasSuperseded = false;
    {
        OmniPvdObjectHandle internalHandle = getInternalHandle(externalObjectHandle, domState->mExternalToInternalHandleMap);
        OmniPvdObject *oldOmniPvdObject = findOmniPvdObject(internalHandle, domState->mObjectHandleToObjectMap);
        if (oldOmniPvdObject)
        {
            const uint64_t supersedeClose = getSupersedeCloseFrameId(oldOmniPvdObject, domState); // oldOmniPvdObject : -
            closeLifespan(oldOmniPvdObject, supersedeClose);
            closeOpenDescendants(oldOmniPvdObject, supersedeClose, domState);
            oldOmniPvdObject->mReopenable = false; // superseded object is gone; never reopen it
            oldOmniPvdObject->mWasDestroyed = true;
            objectHandleWasSuperseded = true; // the object created below re-uses a live handle (segment re-attach)
        }
    }
    domState->mExternalToInternalHandleMap[externalObjectHandle] = domState->mNextInternalHandle;
    OmniPvdObjectHandle internalHandle = domState->mNextInternalHandle;
    domState->mNextInternalHandle++;
    if (omniPvdClass) // Does the class exist?
    {
        const int nbrInheritedClasses = static_cast<int>(omniPvdClass->mInheritanceChain.size());
        if (nbrInheritedClasses < 1)
        {
            PVDDOM_LOG_WARN("   [b2s] Error : class inheritance chain is empty");
            return;
        }

        std::unique_ptr<OmniPvdObject> newObject(new OmniPvdObject());
        OmniPvdObject *omniPvdObject = newObject.get();
        domState->mObjectCreations.push_back(omniPvdObject);
        newObject.release();
        omniPvdObject->mOmniPvdClass = omniPvdClass;
        omniPvdObject->mOmniAPIHandle = externalObjectHandle; // Holds the value of the PhysX pointer
        omniPvdObject->mOmniObjectHandle = omniPvdClass->reserveObjectId(); // Internal handle, less important actually
        omniPvdObject->mLifeSpans[0].mFrameStart = 0; // start of the timeline; list-add handlers restamp it
        omniPvdObject->mIsSupersedeRecreate = objectHandleWasSuperseded; // re-attach snapshot samples belong at frame 0
        omniPvdObject->mRecordingSegmentId = domState->mCurrentRecordingSegmentId;
        if (objectHandleWasSuperseded)
        {
            domState->mSupersedeSnapshotObjects.push_back(omniPvdObject);
        }

        omniPvdObject->mOmniObjectName = reader->getObjectName();

        if (domState->mNbStreamObjectCreations == 0)
        {
            domState->mFirstStreamObject = omniPvdObject;
        }
        domState->mNbStreamObjectCreations++;

        //printf("object creation omniPvdClass(%s)\n", omniPvdClass->mClassName.c_str());

        ////////////////////////////////////////////////////////////////////////////////
        // The object is a PxScene, add it to a special list for startFrame event processing
        ////////////////////////////////////////////////////////////////////////////////
        if (omniPvdClass->mIsSceneClass)
        {
            domState->mSceneCreations.push_back(omniPvdObject);
        }

        // Should be possible to have several ref objects
        std::unique_ptr<OmniPvdObject> refObject;

        switch (omniPvdClass->mPhysXBaseProcessingClassId)
        {
        case OmniPvdPhysXClassEnum::ePxScene:
        {
            // Parent the scene under the latest PxPhysics object.
            if (domState->mLastPhysics)
                domState->mLastPhysics->appendChild(omniPvdObject);
            else if (domState->mSceneRoot)
                domState->mSceneRoot->appendChild(omniPvdObject);
        }
        break;
        case OmniPvdPhysXClassEnum::ePxPhysics:
        {
            domState->mLastPhysics = omniPvdObject;
        }
        break;
        case OmniPvdPhysXClassEnum::ePxActor:
        {
        }
        break;
        case OmniPvdPhysXClassEnum::ePxGeomConvexMesh:
        {
            refObject.reset(createInternalObject(domState->mConvexMeshRefClass, 0,
                                                 omniPvdObject->mRecordingSegmentId));
        }
        break;
        case OmniPvdPhysXClassEnum::ePxGeomHeightfield:
        {
            refObject.reset(createInternalObject(domState->mHeightfieldRefClass, 0,
                                                 omniPvdObject->mRecordingSegmentId));
        }
        break;
        case OmniPvdPhysXClassEnum::ePxGeomTriangleMesh:
        {
            refObject.reset(createInternalObject(domState->mTriangleMeshRefClass, 0,
                                                 omniPvdObject->mRecordingSegmentId));
        }
        break;
        case OmniPvdPhysXClassEnum::ePxGeomTetMesh:
        {
            refObject.reset(createInternalObject(domState->mTetrahedronMeshRefClass, 0,
                                                 omniPvdObject->mRecordingSegmentId));
        }
        break;
        case OmniPvdPhysXClassEnum::ePxConvexMesh:
        {
            omniPvdObject->mIsShared = 1;
            if (domState->mSharedConvexMeshesBranch) domState->mSharedConvexMeshesBranch->appendChild(omniPvdObject);
        }
        break;
        case OmniPvdPhysXClassEnum::ePxHeightfield:
        {
            omniPvdObject->mIsShared = 1;
            if (domState->mSharedHeightfieldsBranch) domState->mSharedHeightfieldsBranch->appendChild(omniPvdObject);
        }
        break;
        case OmniPvdPhysXClassEnum::ePxTriangleMesh:
        {
            omniPvdObject->mIsShared = 1;
            if (domState->mSharedTriangleMeshesBranch)  domState->mSharedTriangleMeshesBranch->appendChild(omniPvdObject);
        }
        break;
        case OmniPvdPhysXClassEnum::ePxTetrahedronMesh:
        {
            omniPvdObject->mIsShared = 1;
            if (domState->mSharedTetrahedronMeshesBranch) domState->mSharedTetrahedronMeshesBranch->appendChild(omniPvdObject);
        }
        break;
        case OmniPvdPhysXClassEnum::ePxDeformableVolumeMesh:
        {
            omniPvdObject->mIsShared = 1;
            if (domState->mSharedDeformableVolumeMeshesBranch) domState->mSharedDeformableVolumeMeshesBranch->appendChild(omniPvdObject);
        }
        break;
        case OmniPvdPhysXClassEnum::ePxMaterial:
        {
            omniPvdObject->mIsShared = 1;
            if (domState->mSharedMaterialsBranch) domState->mSharedMaterialsBranch->appendChild(omniPvdObject);
        }
        break;
        default:
            break;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Pre-size the chain of classes and their inherited attributes. In the case that
        // the class from which the object is derived from is not inheriting from anything
        // the size of the inheritance chain is 1, as the last vector index of
        // (omniPvdClass->mInheritanceChain[omniPvdClass->mInheritanceChain.size() -1 ])
        // is always a pointer to the instance class of the object.
        //
        // Reserve space for the attributes of each inherited class
        ////////////////////////////////////////////////////////////////////////////////
        //const int nbrInheritedClasses = static_cast<int>(omniPvdClass->mInheritanceChain.size());
        omniPvdObject->mInheritedClassInstances.resize(nbrInheritedClasses);

        for (int i = 0; i < nbrInheritedClasses; i++)
        {
            omniPvdObject->mInheritedClassInstances[i].mClassAttributeLists.resize(omniPvdClass->mInheritanceChain[i]->mAttributeDefinitions.size(), 0);
        }
        domState->mObjectHandleToObjectMap[internalHandle] = omniPvdObject;
        omniPvdObject->mUID = internalHandle;

        ////////////////////////////////////////////////////////////////////////////////
        // Schedule the object for creation and also potentially its child reference
        // object.
        ////////////////////////////////////////////////////////////////////////////////
        if (refObject)
        {
            refObject->mLifeSpans[0].mFrameStart = getLifespanOpenFrameId(omniPvdObject, domState); // omniPvdObject : -
            domState->mObjectCreations.push_back(refObject.get());
            omniPvdObject->appendChild(refObject.get());
            omniPvdObject->mReferenceObject = refObject.release();
        }

        if (omniPvdObject->mOmniPvdClass == domState->mPxArticulationReducedCoordinateClass)
        {
            attachFloatingRootLinksToArticulation(*domState, *omniPvdObject);
        }

    }
    else {
        PVDDOM_LOG_WARN("   [b2s] Error : class (classHandle: %d) not yet registered. Object registration skipped.", (int)classHandle);
    }
}

void OMNI_PVD_CALL logFunc(char *logLine)
{
    printf("my log : %s", logLine);
}

void applyPvdDomCommand(OmniPvdReader& readerRef, OmniPvdDOMState& domState, OmniPvdCommand::Enum cmdType)
{
        OmniPvdReader* reader = &readerRef;
        switch (cmdType)
        {
            ////////////////////////////////////////////////////////////////////////////////
            // Class registration
            ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eREGISTER_CLASS:
            handleClassRegistration(reader, &domState);
            break;
            //case OmniPvdCommand::eREGISTER_ENUM:
            //    handleEnumRegistration(reader, &domState);
            //break;
            ////////////////////////////////////////////////////////////////////////////////
            // Attribute registration
            ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eREGISTER_ATTRIBUTE:
        case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
            handleAttributeRegistration(reader, cmdType, &domState);
            break;
        case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
        {
            //PVDDOM_LOG_INFO("   [b2s] register class attribute (classHandle: %d, attributeHandle: %d, classAttributeHandle: %d, name: %s)", (int)reader->getClassHandle(), (int)reader->getAttributeHandle(), (int)reader->getAttributeClassHandle(), reader->getAttributeName());
        }
        break;
        ////////////////////////////////////////////////////////////////////////////////
        // Object : setAttribute / addToAttributeSet / removeFromAttributeSet
        ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eSET_ATTRIBUTE:
        case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE:
        case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE:
            handleAttributeSetting(reader, cmdType, &domState);
            break;
            ////////////////////////////////////////////////////////////////////////////////
            // Object creation
            ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eCREATE_OBJECT:
            handleObjectCreation(reader, cmdType, &domState);
            break;
            ////////////////////////////////////////////////////////////////////////////////
            // Object destruction
            ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eDESTROY_OBJECT:
        {
            //printf("   [b2s] destroy object (contextHandle: %d, objectHandle: %d)\n", (int)reader->getCommandContextHandle(), (int)reader->getCommandObjectHandle());
            uint64_t externalObjectHandle = reader->getObjectHandle();
            {
                uint64_t internalHandle = getInternalHandle(externalObjectHandle, domState.mExternalToInternalHandleMap);
                OmniPvdObject *oldOmniPvdObject = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
                if (oldOmniPvdObject)
                {
                    const uint64_t destroyClose = getLifespanCloseFrameId(oldOmniPvdObject, &domState); // oldOmniPvdObject : -
                    closeLifespan(oldOmniPvdObject, destroyClose);
                    closeOpenDescendants(oldOmniPvdObject, destroyClose);
                    oldOmniPvdObject->mReopenable = false; // destroyed object is gone; never reopen it
                    oldOmniPvdObject->mWasDestroyed = true;
                }
            }
        }
        break;
        ////////////////////////////////////////////////////////////////////////////////
        // Context start frame recording
        ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eSTART_FRAME:
        {
            ////////////////////////////////////////////////////////////////////////////////
            // Should be per scene/contextId
            ////////////////////////////////////////////////////////////////////////////////
            uint64_t contextId = reader->getContextHandle();
            uint64_t frameId = reader->getFrameTimeStart();
            finishSupersedeSnapshots(&domState);

            bool contextAwareStartFrameStream = largerOrEqualOVDIntegversion(1, 4, domState.mStreamOvdIntegVersionMajor, domState.mStreamOvdIntegVersionMinor);
           if (contextAwareStartFrameStream)
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Context aware startFrame command : New code path, could be non-Px objects
                ////////////////////////////////////////////////////////////////////////////////
                uint64_t internalHandle = getInternalHandle(contextId, domState.mExternalToInternalHandleMap);
                OmniPvdObject *contextObject = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
                if (contextObject)
                {
                    contextObject->mFrameId = frameId;
                }
           }
            else
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Non-context aware startFrame command : backwards compatible code path
                // Set all PxScene's frameId to the current frameId -> allows for all scenes to
                // be overlaid. Will break for parallel scenes.
                ////////////////////////////////////////////////////////////////////////////////
                std::list<OmniPvdObject*>::iterator it;
                for (it = domState.mSceneCreations.begin(); it != domState.mSceneCreations.end(); it++)
                {
                    OmniPvdObject* sceneObject = *it;
                    sceneObject->mFrameId = frameId;
                }
            }
            if (domState.mMinFrame > frameId)
            {
                domState.mMinFrame = frameId;
            }
            if (domState.mMaxFrame < frameId)
            {
                domState.mMaxFrame = frameId;
            }
            uint64_t& segmentMax = domState.mRecordingSegmentMaxFrames[domState.mCurrentRecordingSegmentId];
            if (segmentMax < frameId)
            {
                segmentMax = frameId;
            }
            domState.mLatestStartedFrame = frameId;
            //printf("   [b2s] start frame (contextHandle: %d, timeStampe: %" PRIu64 ")\n", (int)reader->getCommandContextHandle(), reader->getCommandFrameTimeStart());
        }
        break;
        ////////////////////////////////////////////////////////////////////////////////
        // Context stop frame recording
        ////////////////////////////////////////////////////////////////////////////////
        case OmniPvdCommand::eSTOP_FRAME:
        {
            //printf("   [b2s] stop frame (contextHandle: %d, timeStampe: %" PRIu64 ")\n", (int)reader->getCommandContextHandle(), reader->getCommandFrameTimeStop());
        }
        break;
        case OmniPvdCommand::eRECORD_MESSAGE:
        {
            const char* pMessage;
            const char* pFileName;

            OmniPvdMessage message;

            if (reader->getMessageData(pMessage, pFileName, message.line, message.type, message.handle))
            {
                uint64_t contextId = reader->getContextHandle();
                uint64_t frameId = getLastFrameIdFromScene(&domState);
                snprintf(message.message, OMNI_PVD_MESSAGE_LENGTH, "%s", pMessage);
                snprintf(message.file, OMNI_PVD_MESSAGE_LENGTH, "%s", pFileName);
                message.frameId = frameId;

                // Look up the enum value
                message.typeName[0] = 0;

                if (message.handle)
                {
                    OmniPvdClass* errorCodeClass = findOmniPvdClass(message.handle, domState.mClassHandleToClassMap);

                    if (errorCodeClass)
                    {
                        for (unsigned int i = 0; i < errorCodeClass->mAttributeDefinitions.size(); i++)
                        {
                            if (message.type == errorCodeClass->mAttributeDefinitions[i]->mNbrFields)
                            {
                                snprintf(message.typeName, OMNI_PVD_MESSAGE_LENGTH, "%s", errorCodeClass->mAttributeDefinitions[i]->mAttributeName.c_str());

                                break;
                            }
                        }
                    }
                }

                gOmniPvdMessages.emplace_back(message);
            }
        }
        break;
        }
}

bool buildPvdDomState(OmniPvdReader* reader, OmniPvdDOMState& domState)
{
    OmniPvdCommand::Enum cmdType;
    while (((cmdType = reader->getNextCommand()) != OmniPvdCommand::eINVALID) &&  !(domState.mOvdIntegVersionWasChecked && !domState.mOvdIntegVersionPassed))
    {
        applyPvdDomCommand(*reader, domState, cmdType);
    }
    if (domState.mOvdIntegVersionWasChecked && !domState.mOvdIntegVersionPassed)
    {
        // Reject unsupported integration versions.
        return false;
    }
    return (domState.mExternalToInternalHandleMap.size() >= 1);
}

bool buildPvdDomStateFromFile(const char* ovdFilePath, OmniPvdDOMState& domState)
{
    return buildPvdDomStateFromFile(ovdFilePath, domState, OmniPvdReaderDiagnostics());
}

bool buildPvdDomStateFromFile(const char* ovdFilePath, OmniPvdDOMState& domState,
                              const OmniPvdReaderDiagnostics& diagnostics)
{
    // Clear any messages left over from a previous load so callers don't see
    // stale diagnostics accumulated across invocations.
    gOmniPvdMessages.clear();

    OmniPvdReader* reader = createOmniPvdReader();
    if (!reader) return false;
    setOmniPvdReaderDiagnostics(*reader, diagnostics);

    OmniPvdFileReadStream* readStream = createOmniPvdFileReadStream();
    if (!readStream) {
        destroyOmniPvdReader(*reader);
        return false;
    }

    readStream->setFileName(ovdFilePath);
    if (!readStream->openStream())
    {
        PVDDOM_LOG_ERROR("buildPvdDomStateFromFile: failed to open OVD file: %s", ovdFilePath);
        destroyOmniPvdReader(*reader);
        destroyOmniPvdFileReadStream(*readStream);
        return false;
    }
    reader->setReadStream(*readStream);
    OmniPvdVersionType majorVersion, minorVersion, patch;
    if (!reader->startReading(majorVersion, minorVersion, patch))
    {
        PVDDOM_LOG_ERROR("buildPvdDomStateFromFile: reader->startReading failed (malformed or incompatible OVD header): %s", ovdFilePath);
        destroyOmniPvdReader(*reader);
        readStream->closeStream();
        destroyOmniPvdFileReadStream(*readStream);
        return false;
    }

    bool result = buildPvdDomState(reader, domState);

    destroyOmniPvdReader(*reader);
    readStream->closeStream();
    destroyOmniPvdFileReadStream(*readStream);
    return result;
}
