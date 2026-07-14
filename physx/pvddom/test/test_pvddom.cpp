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

// pvddom unit tests: write OVD data with pvdruntime, read back with pvddom, verify DOM.

#include <gtest/gtest.h>

#include "PvdDom.h"
#include "PvdDomParser.h"
#include "PvdDomUtils.h"

#include "OmniPvdWriter.h"
#include "OmniPvdReader.h"
#include "OmniPvdMemoryStream.h"
#include "OmniPvdDefines.h"
#include "OmniPvdCommands.h"

// pvdruntime factory functions (compiled into pvddom in standalone mode)
extern "C" {
    OmniPvdWriter* OMNI_PVD_CALL createOmniPvdWriter();
    void OMNI_PVD_CALL destroyOmniPvdWriter(OmniPvdWriter& writer);
    OmniPvdReader* OMNI_PVD_CALL createOmniPvdReader();
    void OMNI_PVD_CALL destroyOmniPvdReader(OmniPvdReader& reader);
    OmniPvdMemoryStream* OMNI_PVD_CALL createOmniPvdMemoryStream();
    void OMNI_PVD_CALL destroyOmniPvdMemoryStream(OmniPvdMemoryStream& stream);
}

// Helper: write OVD to memory, then parse into DOM
class PvdDomTest : public ::testing::Test
{
protected:
    OmniPvdWriter* writer = nullptr;
    OmniPvdMemoryStream* memStream = nullptr;

    void SetUp() override
    {
        writer = createOmniPvdWriter();
        ASSERT_NE(writer, nullptr);
        memStream = createOmniPvdMemoryStream();
        ASSERT_NE(memStream, nullptr);
        memStream->setBufferSize(64 * 1024); // 64KB
        writer->setWriteStream(*memStream->getWriteStream());
    }

    void TearDown() override
    {
        if (writer) destroyOmniPvdWriter(*writer);
        if (memStream) destroyOmniPvdMemoryStream(*memStream);
    }

    bool parseDom(OmniPvdDOMState& domState)
    {
        OmniPvdReader* reader = createOmniPvdReader();
        if (!reader) return false;

        reader->setReadStream(*memStream->getReadStream());
        OmniPvdVersionType major, minor, patch;
        if (!reader->startReading(major, minor, patch))
        {
            destroyOmniPvdReader(*reader);
            return false;
        }

        bool result = buildPvdDomState(reader, domState);
        destroyOmniPvdReader(*reader);
        return result;
    }
};

// ============================================================================
// Test: DOM state initializes cleanly
// ============================================================================
TEST(PvdDomBasic, InitState)
{
    OmniPvdDOMState state;
    // initPvdDomState is called by constructor
    // Internal classes (SceneRoot, SharedRoot, etc.) have handle 0 so they're not in the handle map.
    // But the internal node objects are in mObjectCreations.
    EXPECT_FALSE(state.mObjectCreations.empty());
    EXPECT_NE(state.mSceneRoot, nullptr);
    EXPECT_NE(state.mSharedRoot, nullptr);
    EXPECT_NE(state.mSceneRootClass, nullptr);
    EXPECT_NE(state.mSharedRootClass, nullptr);
    EXPECT_GT(state.mMinFrame, 0u); // initialized to a large number
}

// ============================================================================
// Test: Register a class, create an object, verify DOM
// ============================================================================
TEST_F(PvdDomTest, RegisterClassAndCreateObject)
{
    const OmniPvdContextHandle ctx = 1;
    const OmniPvdObjectHandle objHandle = 100;

    // Write: register class, create object
    OmniPvdClassHandle classHandle = writer->registerClass("TestActor");
    writer->createObject(ctx, classHandle, objHandle, "myActor");

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // Verify class was registered
    OmniPvdClass* cls = findOmniPvdClass(classHandle, domState.mClassHandleToClassMap);
    ASSERT_NE(cls, nullptr);
    EXPECT_EQ(cls->mClassName, "TestActor");
    EXPECT_EQ(cls->mOmniClassHandle, classHandle);

    // Verify object was created (via internal handle mapping)
    EXPECT_FALSE(domState.mExternalToInternalHandleMap.empty());
    uint64_t internalHandle = getInternalHandle(objHandle, domState.mExternalToInternalHandleMap);
    EXPECT_NE(internalHandle, 0u);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->mOmniPvdClass, cls);
}

// ============================================================================
// Test: Class inheritance
// ============================================================================
TEST_F(PvdDomTest, ClassInheritance)
{
    // Write: register base and derived classes
    OmniPvdClassHandle baseHandle = writer->registerClass("BaseClass");
    OmniPvdClassHandle derivedHandle = writer->registerClass("DerivedClass", baseHandle);
    writer->createObject(1, derivedHandle, 200, "derivedObj");

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdClass* baseCls = findOmniPvdClass(baseHandle, domState.mClassHandleToClassMap);
    OmniPvdClass* derivedCls = findOmniPvdClass(derivedHandle, domState.mClassHandleToClassMap);
    ASSERT_NE(baseCls, nullptr);
    ASSERT_NE(derivedCls, nullptr);

    // Derived class should have 2 entries in inheritance chain: [base, derived]
    EXPECT_EQ(derivedCls->mInheritanceChain.size(), 2u);
    EXPECT_EQ(derivedCls->mInheritanceChain[0], baseCls);
    EXPECT_EQ(derivedCls->mInheritanceChain[1], derivedCls);

    // Base class should have 1 entry: [base]
    EXPECT_EQ(baseCls->mInheritanceChain.size(), 1u);
    EXPECT_EQ(baseCls->mInheritanceChain[0], baseCls);
}

// ============================================================================
// Test: Register attribute and set value
// ============================================================================
TEST_F(PvdDomTest, SetAttributeValue)
{
    const OmniPvdContextHandle ctx = 1;
    const OmniPvdObjectHandle objHandle = 300;

    // Write: register class with float3 attribute, create object, set value
    OmniPvdClassHandle classHandle = writer->registerClass("RigidBody");
    OmniPvdAttributeHandle posAttrib = writer->registerAttribute(classHandle, "position",
        OmniPvdDataType::eFLOAT32, 3);
    writer->createObject(ctx, classHandle, objHandle, "body0");

    float pos[3] = { 1.0f, 2.0f, 3.0f };
    writer->setAttribute(ctx, objHandle, posAttrib,
        reinterpret_cast<const uint8_t*>(pos), sizeof(pos));

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // Find the object
    uint64_t internalHandle = getInternalHandle(objHandle, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);

    // Find the attribute data
    int32_t attribIndex = -1;
    int32_t classIndex = -1;
    uint8_t* data = getAttribData(attribIndex, classIndex, "position", obj);
    ASSERT_NE(data, nullptr);

    float* readPos = reinterpret_cast<float*>(data);
    EXPECT_FLOAT_EQ(readPos[0], 1.0f);
    EXPECT_FLOAT_EQ(readPos[1], 2.0f);
    EXPECT_FLOAT_EQ(readPos[2], 3.0f);
}

// ============================================================================
// Test: Frame start/stop tracking
// ============================================================================
TEST_F(PvdDomTest, FrameTracking)
{
    const OmniPvdContextHandle ctx = 1;

    // Write: register a PxScene class so frame tracking works
    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    writer->createObject(ctx, sceneClass, 400, "scene0");

    writer->startFrame(ctx, 100);
    writer->stopFrame(ctx, 100);
    writer->startFrame(ctx, 200);
    writer->stopFrame(ctx, 200);
    writer->startFrame(ctx, 300);
    writer->stopFrame(ctx, 300);

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // Verify frame range
    EXPECT_LE(domState.mMinFrame, 100u);
    EXPECT_GE(domState.mMaxFrame, 300u);
}

// ============================================================================
// Test: String attribute
// ============================================================================
TEST_F(PvdDomTest, StringAttribute)
{
    const OmniPvdContextHandle ctx = 1;
    const OmniPvdObjectHandle objHandle = 500;

    OmniPvdClassHandle classHandle = writer->registerClass("NamedThing");
    OmniPvdAttributeHandle nameAttrib = writer->registerAttribute(classHandle, "name",
        OmniPvdDataType::eSTRING, 0);
    writer->createObject(ctx, classHandle, objHandle, "thing0");

    const char* name = "HelloWorld";
    writer->setAttribute(ctx, objHandle, nameAttrib,
        reinterpret_cast<const uint8_t*>(name),
        static_cast<uint32_t>(strlen(name) + 1));

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    uint64_t internalHandle = getInternalHandle(objHandle, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);

    int32_t attribIndex = -1;
    int32_t classIndex = -1;
    uint8_t* data = getAttribData(attribIndex, classIndex, "name", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_STREQ(reinterpret_cast<const char*>(data), "HelloWorld");
}

// ============================================================================
// Test: Enum class registration
// ============================================================================
TEST_F(PvdDomTest, EnumClass)
{
    OmniPvdClassHandle enumClass = writer->registerClass("MyEnum");
    writer->registerEnumValue(enumClass, "eValue0", 0);
    writer->registerEnumValue(enumClass, "eValue1", 1);
    writer->registerEnumValue(enumClass, "eValue2", 2);

    // Need at least one object for parsing to succeed
    OmniPvdClassHandle dummyClass = writer->registerClass("Dummy");
    writer->createObject(1, dummyClass, 600, "dummy");

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdClass* cls = findOmniPvdClass(enumClass, domState.mClassHandleToClassMap);
    ASSERT_NE(cls, nullptr);
    EXPECT_TRUE(cls->mIsEnumClass);
    EXPECT_EQ(cls->mAttributeDefinitions.size(), 3u);
    EXPECT_EQ(cls->mAttributeDefinitions[0]->mAttributeName, "eValue0");
    EXPECT_EQ(cls->mAttributeDefinitions[1]->mAttributeName, "eValue1");
    EXPECT_EQ(cls->mAttributeDefinitions[2]->mAttributeName, "eValue2");
}

// ============================================================================
// Test: Object destruction sets lifespan stop
// ============================================================================
TEST_F(PvdDomTest, ObjectDestruction)
{
    const OmniPvdContextHandle ctx = 1;

    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    OmniPvdClassHandle actorClass = writer->registerClass("TestActor");

    writer->createObject(ctx, sceneClass, 700, "scene");
    writer->startFrame(ctx, 10);
    writer->createObject(ctx, actorClass, 701, "actor");
    writer->startFrame(ctx, 20);
    writer->destroyObject(ctx, 701);
    writer->stopFrame(ctx, 20);

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    uint64_t internalHandle = getInternalHandle(701, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);
    ASSERT_GE(obj->mLifeSpans.size(), 1u);
    // Lifespan stop should be set (non-zero) after destruction
    EXPECT_NE(obj->mLifeSpans[0].mFrameStop, 0u);
}

// ============================================================================
// Test: Multiple attribute updates (deduplication)
// ============================================================================
TEST_F(PvdDomTest, AttributeDeduplication)
{
    const OmniPvdContextHandle ctx = 1;
    const OmniPvdObjectHandle objHandle = 800;

    OmniPvdClassHandle classHandle = writer->registerClass("Mover");
    OmniPvdAttributeHandle posAttrib = writer->registerAttribute(classHandle, "pos",
        OmniPvdDataType::eFLOAT32, 1);

    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    writer->createObject(ctx, sceneClass, 801, "scene");
    writer->createObject(ctx, classHandle, objHandle, "mover");

    // Set same value twice -- should be deduplicated
    float val1 = 5.0f;
    writer->startFrame(ctx, 1);
    writer->setAttribute(ctx, objHandle, posAttrib,
        reinterpret_cast<const uint8_t*>(&val1), sizeof(float));
    writer->setAttribute(ctx, objHandle, posAttrib,
        reinterpret_cast<const uint8_t*>(&val1), sizeof(float));
    writer->stopFrame(ctx, 1);

    // Set different value
    float val2 = 10.0f;
    writer->startFrame(ctx, 2);
    writer->setAttribute(ctx, objHandle, posAttrib,
        reinterpret_cast<const uint8_t*>(&val2), sizeof(float));
    writer->stopFrame(ctx, 2);

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    uint64_t internalHandle = getInternalHandle(objHandle, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);

    // Get attribute list -- should have 2 samples (deduplicated from 3 sets)
    int32_t attribIndex = -1;
    int32_t classIndex = -1;
    OmniPvdAttributeInstList* attribList = getAttribList(attribIndex, classIndex, "pos", obj);
    ASSERT_NE(attribList, nullptr);

    // Count samples in the linked list
    int count = 0;
    OmniPvdAttributeInst* inst = attribList->mFirst;
    while (inst)
    {
        count++;
        inst = inst->mNextAttribute;
    }
    EXPECT_EQ(count, 2); // val1 + val2 (duplicate val1 deduplicated)

    // Last value should be 10.0f
    uint8_t* data = getAttribData(attribIndex, classIndex, "pos", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_FLOAT_EQ(*reinterpret_cast<float*>(data), 5.0f); // getAttribData returns first sample
}

// ============================================================================
// Test: UniqueList attribute (add/remove)
// ============================================================================
TEST_F(PvdDomTest, UniqueListAttribute)
{
    const OmniPvdContextHandle ctx = 1;

    OmniPvdClassHandle classHandle = writer->registerClass("Container");
    OmniPvdAttributeHandle listAttrib = writer->registerUniqueListAttribute(classHandle,
        "items", OmniPvdDataType::eUINT64);

    writer->createObject(ctx, classHandle, 900, "container");

    uint64_t item1 = 111;
    uint64_t item2 = 222;
    writer->addToUniqueListAttribute(ctx, 900, listAttrib,
        reinterpret_cast<const uint8_t*>(&item1), sizeof(uint64_t));
    writer->addToUniqueListAttribute(ctx, 900, listAttrib,
        reinterpret_cast<const uint8_t*>(&item2), sizeof(uint64_t));

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // Verify the class has the unique list attribute registered
    OmniPvdClass* cls = findOmniPvdClass(classHandle, domState.mClassHandleToClassMap);
    ASSERT_NE(cls, nullptr);
    ASSERT_EQ(cls->mAttributeDefinitions.size(), 1u);
    EXPECT_TRUE(cls->mAttributeDefinitions[0]->mIsUniqueList);
    EXPECT_EQ(cls->mAttributeDefinitions[0]->mAttributeName, "items");

    // Verify object exists
    uint64_t internalHandle = getInternalHandle(900, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);

    // The object should have inherited class instances with the attribute
    ASSERT_GE(obj->mInheritedClassInstances.size(), 1u);
    ASSERT_GE(obj->mInheritedClassInstances[0].mClassAttributeLists.size(), 1u);
    OmniPvdAttributeInstList* attribList = obj->mInheritedClassInstances[0].mClassAttributeLists[0];
    ASSERT_NE(attribList, nullptr);
    ASSERT_NE(attribList->mAttributeDef, nullptr);
    EXPECT_TRUE(attribList->mAttributeDef->mIsUniqueList);
}

// ============================================================================
// Test: Multi-frame recording with changing values
// ============================================================================
TEST_F(PvdDomTest, MultiFrameRecording)
{
    const OmniPvdContextHandle ctx = 1;
    const OmniPvdObjectHandle sceneHandle = 1000;
    const OmniPvdObjectHandle bodyHandle = 1001;

    // Register classes and attributes
    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    OmniPvdClassHandle bodyClass = writer->registerClass("RigidDynamic");
    OmniPvdAttributeHandle posAttrib = writer->registerAttribute(bodyClass, "position",
        OmniPvdDataType::eFLOAT32, 3);
    OmniPvdAttributeHandle velAttrib = writer->registerAttribute(bodyClass, "velocity",
        OmniPvdDataType::eFLOAT32, 3);

    // Create scene and body
    writer->createObject(ctx, sceneClass, sceneHandle, "scene");
    writer->createObject(ctx, bodyClass, bodyHandle, "fallingBox");

    // Simulate 5 frames of a falling object
    float positions[5][3] = {
        { 0.0f, 10.0f, 0.0f },
        { 0.0f,  9.5f, 0.0f },
        { 0.0f,  8.5f, 0.0f },
        { 0.0f,  7.0f, 0.0f },
        { 0.0f,  5.0f, 0.0f },
    };
    float velocities[5][3] = {
        { 0.0f,  0.0f, 0.0f },
        { 0.0f, -1.0f, 0.0f },
        { 0.0f, -2.0f, 0.0f },
        { 0.0f, -3.0f, 0.0f },
        { 0.0f, -4.0f, 0.0f },
    };

    for (int frame = 0; frame < 5; frame++)
    {
        uint64_t timestamp = (frame + 1) * 100;
        writer->startFrame(ctx, timestamp);
        writer->setAttribute(ctx, bodyHandle, posAttrib,
            reinterpret_cast<const uint8_t*>(positions[frame]), sizeof(float) * 3);
        writer->setAttribute(ctx, bodyHandle, velAttrib,
            reinterpret_cast<const uint8_t*>(velocities[frame]), sizeof(float) * 3);
        writer->stopFrame(ctx, timestamp);
    }

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // Verify frame range
    EXPECT_EQ(domState.mMinFrame, 100u);
    EXPECT_EQ(domState.mMaxFrame, 500u);

    // Find the body object
    uint64_t internalHandle = getInternalHandle(bodyHandle, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(internalHandle, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);

    // Verify position attribute has multiple time samples
    int32_t posAttribIndex = -1;
    int32_t posClassIndex = -1;
    OmniPvdAttributeInstList* posAttribList = getAttribList(posAttribIndex, posClassIndex, "position", obj);
    ASSERT_NE(posAttribList, nullptr);

    // Count position samples -- all 5 are different so none should be deduplicated
    int posCount = 0;
    OmniPvdAttributeInst* inst = posAttribList->mFirst;
    OmniPvdAttributeSample* lastPosSample = nullptr;
    while (inst)
    {
        posCount++;
        lastPosSample = static_cast<OmniPvdAttributeSample*>(inst);
        inst = inst->mNextAttribute;
    }
    EXPECT_EQ(posCount, 5);

    // Verify last position sample is the final frame value
    ASSERT_NE(lastPosSample, nullptr);
    ASSERT_GE(lastPosSample->mDataLen, sizeof(float) * 3);
    float* lastPos = reinterpret_cast<float*>(lastPosSample->mData);
    EXPECT_FLOAT_EQ(lastPos[0], 0.0f);
    EXPECT_FLOAT_EQ(lastPos[1], 5.0f);
    EXPECT_FLOAT_EQ(lastPos[2], 0.0f);

    // Verify velocity attribute has multiple time samples
    int32_t velAttribIndex = -1;
    int32_t velClassIndex = -1;
    OmniPvdAttributeInstList* velAttribList = getAttribList(velAttribIndex, velClassIndex, "velocity", obj);
    ASSERT_NE(velAttribList, nullptr);

    int velCount = 0;
    OmniPvdAttributeSample* lastVelSample = nullptr;
    inst = velAttribList->mFirst;
    while (inst)
    {
        velCount++;
        lastVelSample = static_cast<OmniPvdAttributeSample*>(inst);
        inst = inst->mNextAttribute;
    }
    EXPECT_EQ(velCount, 5);

    // Verify last velocity is (-4, 0, 0) -- wait, it's (0, -4, 0)
    ASSERT_NE(lastVelSample, nullptr);
    float* lastVel = reinterpret_cast<float*>(lastVelSample->mData);
    EXPECT_FLOAT_EQ(lastVel[0], 0.0f);
    EXPECT_FLOAT_EQ(lastVel[1], -4.0f);
    EXPECT_FLOAT_EQ(lastVel[2], 0.0f);

    // Verify first position sample is the initial value
    OmniPvdAttributeSample* firstPosSample = static_cast<OmniPvdAttributeSample*>(posAttribList->mFirst);
    ASSERT_NE(firstPosSample, nullptr);
    float* firstPos = reinterpret_cast<float*>(firstPosSample->mData);
    EXPECT_FLOAT_EQ(firstPos[0], 0.0f);
    EXPECT_FLOAT_EQ(firstPos[1], 10.0f);
    EXPECT_FLOAT_EQ(firstPos[2], 0.0f);

    // Verify each sample has a timestamp
    inst = posAttribList->mFirst;
    uint64_t prevTimestamp = 0;
    while (inst)
    {
        // Timestamps should be monotonically non-decreasing
        EXPECT_GE(inst->mTimeStamp, prevTimestamp);
        prevTimestamp = inst->mTimeStamp;
        inst = inst->mNextAttribute;
    }
}

// ============================================================================
// Test: Multiple objects across frames with actor add/remove from scene
// ============================================================================
TEST_F(PvdDomTest, MultiObjectMultiFrame)
{
    const OmniPvdContextHandle ctx = 1;

    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    OmniPvdClassHandle actorClass = writer->registerClass("PxActor");
    OmniPvdAttributeHandle typeAttrib = writer->registerAttribute(actorClass, "type",
        OmniPvdDataType::eUINT32, 1);

    writer->createObject(ctx, sceneClass, 2000, "scene");

    // Frame 1: create 3 actors
    writer->startFrame(ctx, 1);
    for (int i = 0; i < 3; i++)
    {
        writer->createObject(ctx, actorClass, 2001 + i, "");
        uint32_t actorType = 0; // rigid dynamic
        writer->setAttribute(ctx, 2001 + i, typeAttrib,
            reinterpret_cast<const uint8_t*>(&actorType), sizeof(uint32_t));
    }
    writer->stopFrame(ctx, 1);

    // Frame 2: destroy actor 2002
    writer->startFrame(ctx, 2);
    writer->destroyObject(ctx, 2002);
    writer->stopFrame(ctx, 2);

    // Parse
    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // All 3 actors should exist in the DOM
    for (int i = 0; i < 3; i++)
    {
        uint64_t ih = getInternalHandle(2001 + i, domState.mExternalToInternalHandleMap);
        EXPECT_NE(ih, 0u);
        OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
        ASSERT_NE(obj, nullptr);

        if (i == 1) // actor 2002 was destroyed
        {
            EXPECT_NE(obj->mLifeSpans[0].mFrameStop, 0u);
        }
    }

    // Verify frame range
    EXPECT_LE(domState.mMinFrame, 1u);
    EXPECT_GE(domState.mMaxFrame, 2u);
}

// ============================================================================
// Test: Multiple independent classes (no inheritance)
// ============================================================================
TEST_F(PvdDomTest, MultipleIndependentClasses)
{
    OmniPvdClassHandle c1 = writer->registerClass("ClassA");
    OmniPvdClassHandle c2 = writer->registerClass("ClassB");
    OmniPvdClassHandle c3 = writer->registerClass("ClassC");
    writer->createObject(1, c1, 10001, "a");
    writer->createObject(1, c2, 10002, "b");
    writer->createObject(1, c3, 10003, "c");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    EXPECT_NE(findOmniPvdClass(c1, domState.mClassHandleToClassMap), nullptr);
    EXPECT_NE(findOmniPvdClass(c2, domState.mClassHandleToClassMap), nullptr);
    EXPECT_NE(findOmniPvdClass(c3, domState.mClassHandleToClassMap), nullptr);
    EXPECT_NE(findOmniPvdClass(c1, domState.mClassHandleToClassMap),
              findOmniPvdClass(c2, domState.mClassHandleToClassMap));
}

// ============================================================================
// Test: Deep inheritance chain (3 levels)
// ============================================================================
TEST_F(PvdDomTest, DeepInheritance)
{
    OmniPvdClassHandle grandparent = writer->registerClass("GrandParent");
    OmniPvdClassHandle parent = writer->registerClass("Parent", grandparent);
    OmniPvdClassHandle child = writer->registerClass("Child", parent);
    writer->createObject(1, child, 10010, "obj");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdClass* cls = findOmniPvdClass(child, domState.mClassHandleToClassMap);
    ASSERT_NE(cls, nullptr);
    EXPECT_EQ(cls->mInheritanceChain.size(), 3u);
}

// ============================================================================
// Test: Class with multiple attributes of different types
// ============================================================================
TEST_F(PvdDomTest, MultipleAttributeTypes)
{
    OmniPvdClassHandle cls = writer->registerClass("MultiAttrib");
    writer->registerAttribute(cls, "intVal", OmniPvdDataType::eINT32, 1);
    writer->registerAttribute(cls, "floatVal", OmniPvdDataType::eFLOAT32, 1);
    writer->registerAttribute(cls, "name", OmniPvdDataType::eSTRING, 0);
    writer->registerAttribute(cls, "ref", OmniPvdDataType::eOBJECT_HANDLE, 1);
    writer->createObject(1, cls, 10020, "obj");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdClass* c = findOmniPvdClass(cls, domState.mClassHandleToClassMap);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(c->mAttributeDefinitions.size(), 4u);
}

// ============================================================================
// Test: INT8 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Int8Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("I8");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eINT8, 1);
    writer->createObject(1, cls, 10030, "");
    int8_t val = -42;
    writer->setAttribute(1, 10030, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10030, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int8_t*>(data), -42);
}

// ============================================================================
// Test: INT16 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Int16Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("I16");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eINT16, 1);
    writer->createObject(1, cls, 10031, "");
    int16_t val = -1234;
    writer->setAttribute(1, 10031, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10031, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int16_t*>(data), -1234);
}

// ============================================================================
// Test: INT32 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Int32Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("I32");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eINT32, 1);
    writer->createObject(1, cls, 10032, "");
    int32_t val = -99999;
    writer->setAttribute(1, 10032, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10032, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int32_t*>(data), -99999);
}

// ============================================================================
// Test: INT64 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Int64Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("I64");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eINT64, 1);
    writer->createObject(1, cls, 10033, "");
    int64_t val = -123456789012345LL;
    writer->setAttribute(1, 10033, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10033, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int64_t*>(data), -123456789012345LL);
}

// ============================================================================
// Test: UINT8 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Uint8Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("U8");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eUINT8, 1);
    writer->createObject(1, cls, 10034, "");
    uint8_t val = 255;
    writer->setAttribute(1, 10034, attr, &val, sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10034, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*data, 255);
}

// ============================================================================
// Test: UINT16 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Uint16Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("U16");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eUINT16, 1);
    writer->createObject(1, cls, 10035, "");
    uint16_t val = 65535;
    writer->setAttribute(1, 10035, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10035, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<uint16_t*>(data), 65535);
}

// ============================================================================
// Test: UINT32 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Uint32Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("U32");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eUINT32, 1);
    writer->createObject(1, cls, 10036, "");
    uint32_t val = 0xDEADBEEF;
    writer->setAttribute(1, 10036, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10036, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<uint32_t*>(data), 0xDEADBEEF);
}

// ============================================================================
// Test: UINT64 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Uint64Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("U64");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eUINT64, 1);
    writer->createObject(1, cls, 10037, "");
    uint64_t val = 0xCAFEBABEDEADC0DEULL;
    writer->setAttribute(1, 10037, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10037, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<uint64_t*>(data), 0xCAFEBABEDEADC0DEULL);
}

// ============================================================================
// Test: FLOAT64 attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, Float64Attribute)
{
    OmniPvdClassHandle cls = writer->registerClass("F64");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eFLOAT64, 1);
    writer->createObject(1, cls, 10038, "");
    double val = 3.141592653589793;
    writer->setAttribute(1, 10038, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10038, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_DOUBLE_EQ(*reinterpret_cast<double*>(data), 3.141592653589793);
}

// ============================================================================
// Test: OBJECT_HANDLE attribute roundtrip
// ============================================================================
TEST_F(PvdDomTest, ObjectHandleAttribute)
{
    OmniPvdClassHandle cls = writer->registerClass("Ref");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "target", OmniPvdDataType::eOBJECT_HANDLE, 1);
    writer->createObject(1, cls, 10040, "src");
    writer->createObject(1, cls, 10041, "dst");
    OmniPvdObjectHandle targetHandle = 10041;
    writer->setAttribute(1, 10040, attr, reinterpret_cast<const uint8_t*>(&targetHandle), sizeof(targetHandle));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10040, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "target", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<uint64_t*>(data), 10041u);
}

// ============================================================================
// Test: Attribute on inherited class accessed via derived object
// ============================================================================
TEST_F(PvdDomTest, InheritedAttribute)
{
    OmniPvdClassHandle base = writer->registerClass("Base");
    OmniPvdAttributeHandle baseAttr = writer->registerAttribute(base, "baseVal", OmniPvdDataType::eINT32, 1);
    OmniPvdClassHandle derived = writer->registerClass("Derived", base);
    OmniPvdAttributeHandle derivedAttr = writer->registerAttribute(derived, "derivedVal", OmniPvdDataType::eINT32, 1);
    writer->createObject(1, derived, 10050, "obj");

    int32_t bv = 100, dv = 200;
    writer->setAttribute(1, 10050, baseAttr, reinterpret_cast<const uint8_t*>(&bv), sizeof(bv));
    writer->setAttribute(1, 10050, derivedAttr, reinterpret_cast<const uint8_t*>(&dv), sizeof(dv));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10050, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);

    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "baseVal", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int32_t*>(data), 100);

    ai = -1; ci = -1;
    data = getAttribData(ai, ci, "derivedVal", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int32_t*>(data), 200);
}

// ============================================================================
// Test: Attribute value update (overwrite with different value)
// ============================================================================
TEST_F(PvdDomTest, AttributeValueUpdate)
{
    OmniPvdClassHandle cls = writer->registerClass("Updatable");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "counter", OmniPvdDataType::eINT32, 1);
    writer->createObject(1, cls, 10060, "");

    int32_t val = 1;
    writer->setAttribute(1, 10060, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
    val = 2;
    writer->setAttribute(1, 10060, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
    val = 3;
    writer->setAttribute(1, 10060, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10060, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);

    int32_t ai = -1, ci = -1;
    OmniPvdAttributeInstList* list = getAttribList(ai, ci, "counter", obj);
    ASSERT_NE(list, nullptr);

    // 3 different values = 3 samples
    int count = 0;
    OmniPvdAttributeInst* inst = list->mFirst;
    while (inst) { count++; inst = inst->mNextAttribute; }
    EXPECT_EQ(count, 3);
}

// ============================================================================
// Test: UniqueList remove element
// ============================================================================
TEST_F(PvdDomTest, UniqueListRemove)
{
    OmniPvdClassHandle cls = writer->registerClass("SetContainer");
    OmniPvdAttributeHandle listAttr = writer->registerUniqueListAttribute(cls, "set", OmniPvdDataType::eUINT64);
    writer->createObject(1, cls, 10070, "");

    uint64_t a = 1, b = 2, c = 3;
    writer->addToUniqueListAttribute(1, 10070, listAttr, reinterpret_cast<const uint8_t*>(&a), sizeof(a));
    writer->addToUniqueListAttribute(1, 10070, listAttr, reinterpret_cast<const uint8_t*>(&b), sizeof(b));
    writer->addToUniqueListAttribute(1, 10070, listAttr, reinterpret_cast<const uint8_t*>(&c), sizeof(c));
    writer->removeFromUniqueListAttribute(1, 10070, listAttr, reinterpret_cast<const uint8_t*>(&b), sizeof(b));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdClass* clsPtr = findOmniPvdClass(cls, domState.mClassHandleToClassMap);
    ASSERT_NE(clsPtr, nullptr);
    EXPECT_TRUE(clsPtr->mAttributeDefinitions[0]->mIsUniqueList);
}

// ============================================================================
// Test: Flags attribute (bitfield enum)
// ============================================================================
TEST_F(PvdDomTest, FlagsAttribute)
{
    OmniPvdClassHandle enumCls = writer->registerClass("MyFlags");
    writer->registerEnumValue(enumCls, "eFLAG_A", 1);
    writer->registerEnumValue(enumCls, "eFLAG_B", 2);
    writer->registerEnumValue(enumCls, "eFLAG_C", 4);

    OmniPvdClassHandle objCls = writer->registerClass("Flagged");
    OmniPvdAttributeHandle flagsAttr = writer->registerFlagsAttribute(objCls, "flags", enumCls);
    writer->createObject(1, objCls, 10080, "");

    uint32_t flags = 5; // eFLAG_A | eFLAG_C
    writer->setAttribute(1, 10080, flagsAttr, reinterpret_cast<const uint8_t*>(&flags), sizeof(flags));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdClass* eCls = findOmniPvdClass(enumCls, domState.mClassHandleToClassMap);
    ASSERT_NE(eCls, nullptr);
    EXPECT_TRUE(eCls->mIsEnumClass);
    EXPECT_TRUE(eCls->mIsBitFieldEnum);

    uint64_t ih = getInternalHandle(10080, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "flags", obj);
    ASSERT_NE(data, nullptr);
}

// ============================================================================
// Test: Object with empty name
// ============================================================================
TEST_F(PvdDomTest, ObjectEmptyName)
{
    OmniPvdClassHandle cls = writer->registerClass("Anon");
    writer->createObject(1, cls, 10090, "");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10090, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);
    EXPECT_TRUE(obj->mOmniObjectName.empty());
}

// ============================================================================
// Test: Object with non-empty name
// ============================================================================
TEST_F(PvdDomTest, ObjectWithName)
{
    OmniPvdClassHandle cls = writer->registerClass("Named");
    writer->createObject(1, cls, 10091, "mySpecialObject");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(10091, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->mOmniObjectName, "mySpecialObject");
}

// ============================================================================
// Test: Object handle re-creation (destroy then create with same handle)
// ============================================================================
TEST_F(PvdDomTest, ObjectReCreation)
{
    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    OmniPvdClassHandle cls = writer->registerClass("Reusable");
    writer->createObject(1, sceneClass, 10100, "scene");
    writer->startFrame(1, 1);
    writer->createObject(1, cls, 10101, "first");
    writer->startFrame(1, 2);
    writer->destroyObject(1, 10101);
    writer->startFrame(1, 3);
    writer->createObject(1, cls, 10101, "second"); // reuse handle

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // The internal handle should point to the second object
    uint64_t ih = getInternalHandle(10101, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->mOmniObjectName, "second");
}

// ============================================================================
// Test: Many objects (stress)
// ============================================================================
TEST_F(PvdDomTest, ManyObjects)
{
    OmniPvdClassHandle cls = writer->registerClass("Particle");
    const int N = 100;
    for (int i = 0; i < N; i++)
    {
        writer->createObject(1, cls, 20000 + i, "");
    }

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    int found = 0;
    for (int i = 0; i < N; i++)
    {
        uint64_t ih = getInternalHandle(20000 + i, domState.mExternalToInternalHandleMap);
        if (ih != 0 && findOmniPvdObject(ih, domState.mObjectHandleToObjectMap) != nullptr)
            found++;
    }
    EXPECT_EQ(found, N);
}

// ============================================================================
// Test: PxScene class gets special treatment (added to mSceneCreations)
// ============================================================================
TEST_F(PvdDomTest, SceneClassSpecialHandling)
{
    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    writer->createObject(1, sceneClass, 30000, "scene0");
    writer->createObject(1, sceneClass, 30001, "scene1");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    EXPECT_EQ(domState.mSceneCreations.size(), 2u);
}

// ============================================================================
// Test: Attribute set before any frame started
// ============================================================================
TEST_F(PvdDomTest, AttributeSetBeforeFrame)
{
    OmniPvdClassHandle cls = writer->registerClass("Early");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "val", OmniPvdDataType::eINT32, 1);
    writer->createObject(1, cls, 30010, "");

    // Set attribute before any startFrame
    int32_t val = 42;
    writer->setAttribute(1, 30010, attr, reinterpret_cast<const uint8_t*>(&val), sizeof(val));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(30010, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "val", obj);
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(*reinterpret_cast<int32_t*>(data), 42);
}

// ============================================================================
// Test: Float array attribute (matrix-like)
// ============================================================================
TEST_F(PvdDomTest, FloatArrayAttribute)
{
    OmniPvdClassHandle cls = writer->registerClass("Transform");
    OmniPvdAttributeHandle attr = writer->registerAttribute(cls, "matrix", OmniPvdDataType::eFLOAT32, 16);
    writer->createObject(1, cls, 30020, "");

    float identity[16] = {
        1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1
    };
    writer->setAttribute(1, 30020, attr, reinterpret_cast<const uint8_t*>(identity), sizeof(identity));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(30020, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "matrix", obj);
    ASSERT_NE(data, nullptr);
    float* m = reinterpret_cast<float*>(data);
    EXPECT_FLOAT_EQ(m[0], 1.0f);
    EXPECT_FLOAT_EQ(m[5], 1.0f);
    EXPECT_FLOAT_EQ(m[10], 1.0f);
    EXPECT_FLOAT_EQ(m[15], 1.0f);
    EXPECT_FLOAT_EQ(m[1], 0.0f);
}

// ============================================================================
// Test: Enum with non-power-of-two values (not a bitfield)
// ============================================================================
TEST_F(PvdDomTest, NonBitFieldEnum)
{
    OmniPvdClassHandle enumCls = writer->registerClass("ShapeType");
    writer->registerEnumValue(enumCls, "eSPHERE", 0);
    writer->registerEnumValue(enumCls, "eBOX", 1);
    writer->registerEnumValue(enumCls, "eCAPSULE", 2);
    writer->registerEnumValue(enumCls, "eMESH", 3);

    // Need object for parsing
    OmniPvdClassHandle dummy = writer->registerClass("D");
    writer->createObject(1, dummy, 30030, "");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    OmniPvdClass* cls = findOmniPvdClass(enumCls, domState.mClassHandleToClassMap);
    ASSERT_NE(cls, nullptr);
    EXPECT_TRUE(cls->mIsEnumClass);
    EXPECT_FALSE(cls->mIsBitFieldEnum); // values 0,1,2,3 are not all single bits
}

// ============================================================================
// Test: Class attribute definition count
// ============================================================================
TEST_F(PvdDomTest, AttributeDefinitionCount)
{
    OmniPvdClassHandle cls = writer->registerClass("ManyAttribs");
    for (int i = 0; i < 10; i++)
    {
        char name[32];
        snprintf(name, sizeof(name), "attr_%d", i);
        writer->registerAttribute(cls, name, OmniPvdDataType::eFLOAT32, 1);
    }
    writer->createObject(1, cls, 30040, "");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    OmniPvdClass* c = findOmniPvdClass(cls, domState.mClassHandleToClassMap);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(c->mAttributeDefinitions.size(), 10u);
}

// ============================================================================
// Test: Attribute data type stored correctly
// ============================================================================
TEST_F(PvdDomTest, AttributeDataTypeStored)
{
    OmniPvdClassHandle cls = writer->registerClass("TypeCheck");
    writer->registerAttribute(cls, "f32", OmniPvdDataType::eFLOAT32, 1);
    writer->registerAttribute(cls, "i64", OmniPvdDataType::eINT64, 1);
    writer->registerAttribute(cls, "str", OmniPvdDataType::eSTRING, 0);
    writer->createObject(1, cls, 30050, "");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    OmniPvdClass* c = findOmniPvdClass(cls, domState.mClassHandleToClassMap);
    ASSERT_NE(c, nullptr);
    ASSERT_EQ(c->mAttributeDefinitions.size(), 3u);
    EXPECT_EQ(c->mAttributeDefinitions[0]->mDataType, OmniPvdDataType::eFLOAT32);
    EXPECT_EQ(c->mAttributeDefinitions[1]->mDataType, OmniPvdDataType::eINT64);
    EXPECT_EQ(c->mAttributeDefinitions[2]->mDataType, OmniPvdDataType::eSTRING);
}

// ============================================================================
// Test: Multiple scenes tracked independently
// ============================================================================
TEST_F(PvdDomTest, MultipleScenes)
{
    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    writer->createObject(1, sceneClass, 40000, "scene_A");
    writer->createObject(2, sceneClass, 40001, "scene_B");

    writer->startFrame(1, 100);
    writer->stopFrame(1, 100);
    writer->startFrame(2, 200);
    writer->stopFrame(2, 200);

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    EXPECT_EQ(domState.mSceneCreations.size(), 2u);
    EXPECT_LE(domState.mMinFrame, 100u);
    EXPECT_GE(domState.mMaxFrame, 200u);
}

// ============================================================================
// Test: Large attribute data (e.g. mesh vertices)
// ============================================================================
TEST_F(PvdDomTest, LargeAttributeData)
{
    OmniPvdClassHandle cls = writer->registerClass("Mesh");
    OmniPvdAttributeHandle vertsAttr = writer->registerAttribute(cls, "vertices",
        OmniPvdDataType::eFLOAT32, 0); // variable length
    writer->createObject(1, cls, 40010, "mesh");

    // 100 vertices = 300 floats
    const int numVerts = 100;
    float verts[numVerts * 3];
    for (int i = 0; i < numVerts; i++)
    {
        verts[i * 3 + 0] = static_cast<float>(i);
        verts[i * 3 + 1] = static_cast<float>(i * 2);
        verts[i * 3 + 2] = static_cast<float>(i * 3);
    }
    writer->setAttribute(1, 40010, vertsAttr,
        reinterpret_cast<const uint8_t*>(verts), sizeof(verts));

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));
    uint64_t ih = getInternalHandle(40010, domState.mExternalToInternalHandleMap);
    OmniPvdObject* obj = findOmniPvdObject(ih, domState.mObjectHandleToObjectMap);
    int32_t ai = -1, ci = -1;
    uint8_t* data = getAttribData(ai, ci, "vertices", obj);
    ASSERT_NE(data, nullptr);
    float* readVerts = reinterpret_cast<float*>(data);
    EXPECT_FLOAT_EQ(readVerts[0], 0.0f);
    EXPECT_FLOAT_EQ(readVerts[297], 99.0f);
    EXPECT_FLOAT_EQ(readVerts[298], 198.0f);
    EXPECT_FLOAT_EQ(readVerts[299], 297.0f);
}

// ============================================================================
// Test: Object UID is unique per object
// ============================================================================
TEST_F(PvdDomTest, UniqueObjectUIDs)
{
    OmniPvdClassHandle cls = writer->registerClass("UID");
    writer->createObject(1, cls, 50000, "a");
    writer->createObject(1, cls, 50001, "b");
    writer->createObject(1, cls, 50002, "c");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    uint64_t ih0 = getInternalHandle(50000, domState.mExternalToInternalHandleMap);
    uint64_t ih1 = getInternalHandle(50001, domState.mExternalToInternalHandleMap);
    uint64_t ih2 = getInternalHandle(50002, domState.mExternalToInternalHandleMap);
    EXPECT_NE(ih0, ih1);
    EXPECT_NE(ih1, ih2);
    EXPECT_NE(ih0, ih2);

    OmniPvdObject* o0 = findOmniPvdObject(ih0, domState.mObjectHandleToObjectMap);
    OmniPvdObject* o1 = findOmniPvdObject(ih1, domState.mObjectHandleToObjectMap);
    OmniPvdObject* o2 = findOmniPvdObject(ih2, domState.mObjectHandleToObjectMap);
    EXPECT_NE(o0->mUID, o1->mUID);
    EXPECT_NE(o1->mUID, o2->mUID);
}

// ============================================================================
// Test: Attribute handle lookup
// ============================================================================
TEST_F(PvdDomTest, AttributeHandleLookup)
{
    OmniPvdClassHandle cls = writer->registerClass("Lookup");
    OmniPvdAttributeHandle attr1 = writer->registerAttribute(cls, "alpha", OmniPvdDataType::eFLOAT32, 1);
    OmniPvdAttributeHandle attr2 = writer->registerAttribute(cls, "beta", OmniPvdDataType::eINT32, 1);
    writer->createObject(1, cls, 50010, "");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    OmniPvdAttributeDef* def1 = findOmniPvdAttribute(attr1, domState.mAttributeHandleToAttributeMap);
    OmniPvdAttributeDef* def2 = findOmniPvdAttribute(attr2, domState.mAttributeHandleToAttributeMap);
    ASSERT_NE(def1, nullptr);
    ASSERT_NE(def2, nullptr);
    EXPECT_EQ(def1->mAttributeName, "alpha");
    EXPECT_EQ(def2->mAttributeName, "beta");
}

// ============================================================================
// Test: Class name lookup in config map
// ============================================================================
TEST_F(PvdDomTest, ClassConfigMapPopulated)
{
    // Just create a dummy object so parsing succeeds
    OmniPvdClassHandle cls = writer->registerClass("Dummy");
    writer->createObject(1, cls, 50020, "");

    OmniPvdDOMState domState;
    ASSERT_TRUE(parseDom(domState));

    // Config map should have PhysX class mappings from initPvdDomState
    EXPECT_FALSE(domState.mClassConfigMap.empty());
    auto it = domState.mClassConfigMap.find("PxScene");
    EXPECT_NE(it, domState.mClassConfigMap.end());
    EXPECT_EQ(it->second, OmniPvdPhysXClassEnum::ePxScene);
}

// ============================================================================
// Test: Empty stream returns false
// ============================================================================
TEST_F(PvdDomTest, EmptyStreamFails)
{
    // Don't write anything -- parsing should fail
    OmniPvdDOMState domState;
    EXPECT_FALSE(parseDom(domState));
}

// ============================================================================
// Test: Parse real PhysX SDK OVD file
// ============================================================================
TEST(PvdDomFile, ParsePhysXOvdFile)
{
    // Search several relative paths so the test works from different working dirs
    const char* paths[] = {
        "test/data/test_scene.ovd",                      // from pvddom root
        "../test/data/test_scene.ovd",                   // from pvddom build dir
        "pvddom/test/data/test_scene.ovd",               // from physx root
        "../pvddom/test/data/test_scene.ovd",            // from physx/bin
        "../../pvddom/test/data/test_scene.ovd",         // from physx/bin/platform
    };
    OmniPvdDOMState domState;
    bool ok = false;
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i)
    {
        ok = buildPvdDomStateFromFile(paths[i], domState);
        if (ok)
        {
            printf("  Found test_scene.ovd at: %s\n", paths[i]);
            break;
        }
    }
    if (!ok)
    {
        printf("  SKIPPED: test_scene.ovd not found in any search path\n");
        return;
    }

    // Should have classes registered
    EXPECT_GT(domState.mClassHandleToClassMap.size(), 0u);
    printf("  Classes: %zu\n", domState.mClassHandleToClassMap.size());

    // Should have objects
    EXPECT_GT(domState.mObjectHandleToObjectMap.size(), 0u);
    printf("  Objects: %zu\n", domState.mObjectHandleToObjectMap.size());

    // Should have at least 1 scene
    EXPECT_GT(domState.mSceneCreations.size(), 0u);
    printf("  Scenes: %zu\n", domState.mSceneCreations.size());

    // Print class names for inspection
    printf("  Registered classes:\n");
    for (auto& pair : domState.mClassHandleToClassMap)
    {
        printf("    [%u] %s (attribs: %zu, chain: %zu)\n",
            pair.first,
            pair.second->mClassName.c_str(),
            pair.second->mAttributeDefinitions.size(),
            pair.second->mInheritanceChain.size());
    }

    printf("  Frame range: %llu - %llu\n",
        (unsigned long long)domState.mMinFrame,
        (unsigned long long)domState.mMaxFrame);
}
