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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "TkBaseTest.h"

#include <map>
#include <random>
#include <algorithm>

#include "NsMemoryBuffer.h"

#include "NvBlastTime.h"


/*
Composite and joint tests:

0) Test serialization of composites and assemblies

1) Create assembly, actors and joints should be created automatically

2) Create an actor with internal joints.  Splitting the actor should cause joint create events to be dispatched

3) Joint update events should be fired when attached actors change

4) Joint delete events should be fired when at least one attached actor is deleted

5) Creating a composite from assets with internal joints should have expected behaviors (1-4) above
*/


struct Composite
{
    std::vector<TkActorDesc>        m_actorDescs;
    std::vector<nvidia::NvTransform> m_relTMs;
    std::vector<TkJointDesc>        m_jointDescs;
};


template<int FailLevel, int Verbosity>
class TkCompositeTest : public TkBaseTest<FailLevel, Verbosity>
{
public:

    // Composite/joint tests
    void createAssembly(std::vector<TkActor*>& actors, std::vector<TkJoint*>& joints, bool createNRFJoints)
    {
        TkFramework* fw = NvBlastTkFrameworkGet();

        actors.resize(4, nullptr);
        actors[0] = fw->createActor(TkActorDesc(testAssets[0]));
        actors[1] = fw->createActor(TkActorDesc(testAssets[0]));
        actors[2] = fw->createActor(TkActorDesc(testAssets[1]));
        actors[3] = fw->createActor(TkActorDesc(testAssets[1]));

        std::vector<TkFamily*> families(4);
        families[0] = &actors[0]->getFamily();
        families[1] = &actors[1]->getFamily();
        families[2] = &actors[2]->getFamily();
        families[3] = &actors[3]->getFamily();

        EXPECT_FALSE(actors[0] == nullptr);
        EXPECT_FALSE(actors[1] == nullptr);
        EXPECT_FALSE(actors[2] == nullptr);
        EXPECT_FALSE(actors[3] == nullptr);

        const TkJointDesc jointDescsNoNRF[8] =
        {
            // Actor indices, chunk indices, attach position in the composite frame
            { { families[0], families[1] }, { 6, 5 }, { NvVec3(0.0f, -1.5f, 0.5f), NvVec3(0.0f, -1.5f, 0.5f) } },
            { { families[0], families[1] }, { 4, 3 }, { NvVec3(0.0f, -0.5f, -0.5f), NvVec3(0.0f, -0.5f, -0.5f) } },
                        
            { { families[0], families[2] }, { 8, 6 }, { NvVec3(-0.5f, 0.0f, 0.5f), NvVec3(-0.5f, 0.0f, 0.5f) } },
            { { families[0], families[2] }, { 3, 1 }, { NvVec3(-1.5f, 0.0f, -0.5f), NvVec3(-1.5f, 0.0f, -0.5f) } },
                        
            { { families[1], families[3] }, { 7, 5 }, { NvVec3(0.5f, 0.0f, 0.5f), NvVec3(0.5f, 0.0f, 0.5f) } },
            { { families[1], families[3] }, { 4, 2 }, { NvVec3(1.0f, 0.0f, -0.5f), NvVec3(1.0f, 0.0f, -0.5f) } },
                        
            { { families[2], families[3] }, { 8, 7 }, { NvVec3(0.0f, 1.5f, 0.5f), NvVec3(0.0f, 1.5f, 0.5f) } },
            { { families[2], families[3] }, { 2, 1 }, { NvVec3(0.0f, 0.5f, -0.5f), NvVec3(0.0f, 0.5f, -0.5f) } }
        };

        const TkJointDesc jointDescsWithNRF[12] =
        {
            // Actor indices, chunk indices, attach position in the composite frame
            { { families[0], families[1] }, { 6, 5 }, { NvVec3(0.0f, -1.5f, 0.5f), NvVec3(0.0f, -1.5f, 0.5f) } },
            { { families[0], families[1] }, { 4, 3 }, { NvVec3(0.0f, -0.5f, -0.5f), NvVec3(0.0f, -0.5f, -0.5f) } },
                        
            { { families[0], nullptr }, { 8, 0xFFFFFFFF }, { NvVec3(-0.5f, 0.0f, 0.5f), NvVec3(-0.5f, 0.0f, 0.5f) } },
            { { families[0], nullptr }, { 3, 0xFFFFFFFF }, { NvVec3(-1.5f, 0.0f, -0.5f), NvVec3(-1.5f, 0.0f, -0.5f) } },
                        
            { { nullptr, families[2] }, { 0xFFFFFFFF, 6 }, { NvVec3(-0.5f, 0.0f, 0.5f), NvVec3(-0.5f, 0.0f, 0.5f) } },
            { { nullptr, families[2] }, { 0xFFFFFFFF, 1 }, { NvVec3(-1.5f, 0.0f, -0.5f), NvVec3(-1.5f, 0.0f, -0.5f) } },

            { { families[1], nullptr }, { 7, 0xFFFFFFFF }, { NvVec3(0.5f, 0.0f, 0.5f), NvVec3(0.5f, 0.0f, 0.5f) } },
            { { families[1], nullptr }, { 4, 0xFFFFFFFF }, { NvVec3(1.0f, 0.0f, -0.5f), NvVec3(1.0f, 0.0f, -0.5f) } },
                        
            { { nullptr, families[3] }, { 0xFFFFFFFF, 5 }, { NvVec3(0.5f, 0.0f, 0.5f), NvVec3(0.5f, 0.0f, 0.5f) } },
            { { nullptr, families[3] }, { 0xFFFFFFFF, 2 }, { NvVec3(1.0f, 0.0f, -0.5f), NvVec3(1.0f, 0.0f, -0.5f) } },

            { { families[2], families[3] }, { 8, 7 }, { NvVec3(0.0f, 1.5f, 0.5f), NvVec3(0.0f, 1.5f, 0.5f) } },
            { { families[2], families[3] }, { 2, 1 }, { NvVec3(0.0f, 0.5f, -0.5f), NvVec3(0.0f, 0.5f, -0.5f), } }
        };

        const TkJointDesc* jointDescs = createNRFJoints ? jointDescsWithNRF : jointDescsNoNRF;
        const int jointCount = createNRFJoints ? 12 : 8;

        joints.resize(jointCount, nullptr);
        for (int i = 0; i < jointCount; ++i)
        {
            joints[i] = fw->createJoint(jointDescs[i]);
            EXPECT_FALSE(joints[i] == nullptr);
        }
    }

    void familySerialization(std::vector<TkFamily*>& families, TestFamilyTracker& tracker)
    {
#if 1
        NV_UNUSED(families);
        NV_UNUSED(tracker);
#else
        TkFramework* fw = NvBlastTkFrameworkGet();

        PsMemoryBuffer* membuf = NVBLAST_NEW(PsMemoryBuffer);
        EXPECT_TRUE(membuf != nullptr);
        if (membuf == nullptr)
        {
            return;
        }

        std::vector<TkFamily*> oldFamilies = families;

        for (size_t familyNum = 0; familyNum < families.size(); ++familyNum)
        {
            GTEST_FATAL_FAILURE_("Serialization of families needs to be put into extensions.");
//          families[familyNum]->serialize(*membuf);
        }

        for (size_t familyNum = 0; familyNum < families.size(); ++familyNum)
        {
            TkFamily* f = families[familyNum];

            std::vector<TkActor*> actors(f->getActorCount());
            f->getActors(actors.data(), static_cast<uint32_t>(actors.size()));
            for (auto a : actors)
            {
                tracker.eraseActor(a);
            }

            f->release();
            families[familyNum] = nullptr;
        }

        for (size_t familyNum = 0; familyNum < families.size(); ++familyNum)
        {
            GTEST_FATAL_FAILURE_("Deserialization of families needs to be put into extensions.");
//          TkFamily* f = reinterpret_cast<TkFamily*>(fw->deserialize(*membuf));
//          f->addListener(tracker);
//          families[familyNum] = f;
        }

        for (size_t familyNum = 0; familyNum < families.size(); ++familyNum)
        {
            TkFamily* f = families[familyNum];

            std::vector<TkActor*> actors(f->getActorCount());
            f->getActors(actors.data(), static_cast<uint32_t>(actors.size()));
            for (auto a : actors)
            {
                tracker.insertActor(a);

                std::vector<TkJoint*> joints(a->getJointCount());
                a->getJoints(joints.data(), (uint32_t)joints.size());

                for (auto j : joints)
                {
                    const TkJointData jd = j->getData();
                    if (jd.actors[0] != jd.actors[1])
                    {
                        tracker.joints.insert(j);
                    }
                }
            }
        }

        membuf->release();
#endif
    }

    void recollectActors(std::vector<TkFamily*>& families, std::vector<TkActor*>& actors)
    {
        uint32_t totalActorCount = 0;
        for (auto family : families)
        {
            EXPECT_LE(family->getActorCount() + totalActorCount, actors.size());
            totalActorCount += family->getActors(actors.data() + totalActorCount, static_cast<uint32_t>(actors.size()) - totalActorCount);
        }
    }

    void assemblyCreateAndRelease(bool createNRFJoints, bool serializationTest)
    {
        createFramework();
        createTestAssets();

        TkFramework* fw = NvBlastTkFrameworkGet();

        const TkType* familyType = fw->getType(TkTypeIndex::Family);
        EXPECT_TRUE(familyType != nullptr);

        TestFamilyTracker tracker;

        std::vector<TkFamily*> families1;
        std::vector<TkFamily*> families2;

        // Create one assembly
        std::vector<TkActor*> actors1;
        std::vector<TkJoint*> joints1;
        createAssembly(actors1, joints1, createNRFJoints);
        tracker.joints.insert(joints1.begin(), joints1.end());

        // Create another assembly
        std::vector<TkActor*> actors2;
        std::vector<TkJoint*> joints2;
        createAssembly(actors2, joints2, createNRFJoints);
        tracker.joints.insert(joints2.begin(), joints2.end());

        // Store families and fill group
        for (size_t actorNum = 0; actorNum < actors1.size(); ++actorNum)
        {
            TkFamily& family = actors1[actorNum]->getFamily();
            families1.push_back(&family);
            family.addListener(tracker);
        }
        for (size_t actorNum = 0; actorNum < actors2.size(); ++actorNum)
        {
            TkFamily& family = actors2[actorNum]->getFamily();
            families2.push_back(&family);
            family.addListener(tracker);
        }

        if (serializationTest)
        {
            familySerialization(families1, tracker);
            recollectActors(families1, actors1);
            familySerialization(families2, tracker);
            recollectActors(families2, actors2);
        }

        EXPECT_EQ(joints1.size() + joints2.size(), tracker.joints.size());

        // Release 1st assembly's actors
        for (size_t actorNum = 0; actorNum < actors1.size(); ++actorNum)
        {
            actors1[actorNum]->release();
        }

        if (serializationTest)
        {
            familySerialization(families2, tracker);
            recollectActors(families2, actors2);
        }

        EXPECT_EQ(joints2.size(), tracker.joints.size());

        // Release 2nd assembly's actors
        for (size_t actorNum = 0; actorNum < actors1.size(); ++actorNum)
        {
            actors2[actorNum]->release();
        }

        EXPECT_EQ(0, tracker.joints.size());

        releaseTestAssets();
        releaseFramework();
    }

    void assemblyInternalJoints(bool testAssemblySerialization)
    {
        createFramework();
        createTestAssets(true); // Create assets with internal joints

        TkFramework* fw = NvBlastTkFrameworkGet();

        TestFamilyTracker tracker;

        TkGroupDesc gdesc;
        gdesc.workerCount = m_taskman->getCpuDispatcher()->getWorkerCount();
        TkGroup* group = fw->createGroup(gdesc);
        EXPECT_TRUE(group != nullptr);

        m_groupTM->setGroup(group);

        TkActorDesc adesc(testAssets[0]);

        TkActor* actor1 = fw->createActor(adesc);
        EXPECT_TRUE(actor1 != nullptr);
        tracker.insertActor(actor1);

        actor1->getFamily().addListener(tracker);

        TkFamily* family = &actor1->getFamily();

        group->addActor(*actor1);

        CSParams cs2(2, 0.0f);
        NvBlastExtProgramParams csParams2 = { &cs2, nullptr };
        actor1->damage(getCubeSlicerProgram(), &csParams2);

        EXPECT_EQ((size_t)0, tracker.joints.size());

        m_groupTM->process();
        m_groupTM->wait();

        if (testAssemblySerialization)
        {
            std::vector<TkFamily*> families;
            families.push_back(family);
            familySerialization(families, tracker);
            family = families[0];
            std::vector<TkActor*> actors(family->getActorCount());
            family->getActors(actors.data(), static_cast<uint32_t>(actors.size()));
            for (TkActor* actor : actors)
            {
                group->addActor(*actor);
            }
        }

        EXPECT_EQ((size_t)2, family->getActorCount());
        EXPECT_EQ((size_t)4, tracker.joints.size());    // 2) Create an actor with internal joints.  Splitting the actor should cause joint create events to be dispatched

        std::vector<TkActor*> actors(family->getActorCount());
        family->getActors(actors.data(), static_cast<uint32_t>(actors.size()));

        for (TkJoint* joint : tracker.joints)
        {
            TkJointData jd = joint->getData();
            EXPECT_FALSE(actors.end() == std::find(actors.begin(), actors.end(), jd.actors[0]));
            EXPECT_FALSE(actors.end() == std::find(actors.begin(), actors.end(), jd.actors[1]));
        }

        NvBlastExtRadialDamageDesc radialDamage = getRadialDamageDesc(0, 0, 0);
        NvBlastExtProgramParams radialParams = { &radialDamage, nullptr };
        for (TkActor* actor : actors)
        {
            actor->damage(getFalloffProgram(), &radialParams);
        }

        m_groupTM->process();
        m_groupTM->wait();

        if (testAssemblySerialization)
        {
            std::vector<TkFamily*> families;
            families.push_back(family);
            familySerialization(families, tracker);
            family = families[0];
        }

        EXPECT_EQ((size_t)8, family->getActorCount());
        EXPECT_EQ((size_t)4, tracker.joints.size());

        // 3) Joint update events should be fired when attached actors change

        actors.resize(family->getActorCount());
        family->getActors(actors.data(), static_cast<uint32_t>(actors.size()));

        for (TkJoint* joint : tracker.joints)
        {
            TkJointData jd = joint->getData();
            EXPECT_FALSE(actors.end() == std::find(actors.begin(), actors.end(), jd.actors[0]));
            EXPECT_FALSE(actors.end() == std::find(actors.begin(), actors.end(), jd.actors[1]));
        }

        for (TkActor* actor : actors)
        {
            actor->release();
        }

        EXPECT_EQ((size_t)0, tracker.joints.size());    // 4) Joint delete events should be fired when at least one attached actor is deleted

        group->release();

        releaseTestAssets();
        releaseFramework();
    }

    void assemblyCompositeWithInternalJoints(bool createNRFJoints, bool serializationTest)
    {
        createFramework();
        createTestAssets(true); // Create assets with internal joints

        TkFramework* fw = NvBlastTkFrameworkGet();

        const TkType* familyType = fw->getType(TkTypeIndex::Family);
        EXPECT_TRUE(familyType != nullptr);

        if (familyType == nullptr)
        {
            return;
        }

        TestFamilyTracker tracker;

        std::vector<TkFamily*> families;

        // Create assembly
        std::vector<TkActor*> actors;
        std::vector<TkJoint*> joints;
        createAssembly(actors, joints, createNRFJoints);
        tracker.joints.insert(joints.begin(), joints.end());

        TkGroupDesc gdesc;
        gdesc.workerCount = m_taskman->getCpuDispatcher()->getWorkerCount();
        TkGroup* group = fw->createGroup(gdesc);
        EXPECT_TRUE(group != nullptr);

        m_groupTM->setGroup(group);

        for (size_t i = 0; i < actors.size(); ++i)
        {
            TkFamily& family = actors[i]->getFamily();
            families.push_back(&family);
            family.addListener(tracker);
            tracker.insertActor(actors[i]);
            group->addActor(*actors[i]);
        }

        if (serializationTest)
        {
            familySerialization(families, tracker);
            recollectActors(families, actors);
            for (auto actor : actors)
            {
                group->addActor(*actor);
            }
        }

        EXPECT_EQ((size_t)4, actors.size());

        const size_t compJointCount = createNRFJoints ? (size_t)12 : (size_t)8;

        EXPECT_EQ(compJointCount, tracker.joints.size());

        CSParams cs2(2, 0.0f);
        NvBlastExtProgramParams csParams2 = { &cs2, nullptr };

        size_t totalActorCount = 0;
        for (uint32_t i = 0; i < 4; ++i)
        {
            actors[i]->damage(getCubeSlicerProgram(), &csParams2);

            m_groupTM->process();
            m_groupTM->wait();

            if (serializationTest)
            {
                familySerialization(families, tracker);
                for (size_t j = 0; j < families.size(); ++j)
                {
                    TkFamily* family = families[j];
                    std::vector<TkActor*> a(family->getActorCount());
                    family->getActors(a.data(), static_cast<uint32_t>(a.size()));
                    for (auto actor : a)
                    {
                        group->addActor(*actor);
                    }
                    EXPECT_TRUE(j <= i || a.size() == 1);
                    if (j > i && a.size() == 1)
                    {
                        actors[j] = a[0];
                    }
                }
            }

            EXPECT_EQ((size_t)2, families[i]->getActorCount());
            EXPECT_EQ((size_t)(compJointCount + 4 * (i + 1)), tracker.joints.size());   // Four joints created per actor

            totalActorCount += families[i]->getActorCount();
        }

        actors.resize(totalActorCount);
        totalActorCount = 0;
        for (int i = 0; i < 4; ++i)
        {
            families[i]->getActors(actors.data() + totalActorCount, families[i]->getActorCount());
            totalActorCount += families[i]->getActorCount();
        }

        for (TkJoint* joint : tracker.joints)
        {
            TkJointData jd = joint->getData();
            EXPECT_TRUE(jd.actors[0] == nullptr || actors.end() != std::find(actors.begin(), actors.end(), jd.actors[0]));
            EXPECT_TRUE(jd.actors[1] == nullptr || actors.end() != std::find(actors.begin(), actors.end(), jd.actors[1]));
        }

        NvBlastExtRadialDamageDesc radialDamage = getRadialDamageDesc(0, 0, 0);
        NvBlastExtProgramParams radialParams = { &radialDamage, nullptr };
        for (TkActor* actor : actors)
        {
            actor->damage(getFalloffProgram(), &radialParams);
        }

        m_groupTM->process();
        m_groupTM->wait();

        totalActorCount = 0;
        for (int i = 0; i < 4; ++i)
        {
            totalActorCount += families[i]->getActorCount();
        }

        if (serializationTest)
        {
            familySerialization(families, tracker);
        }

        EXPECT_EQ((size_t)32, totalActorCount);
        EXPECT_EQ(compJointCount + (size_t)16, tracker.joints.size());

        actors.resize(totalActorCount);
        totalActorCount = 0;
        for (int i = 0; i < 4; ++i)
        {
            families[i]->getActors(actors.data() + totalActorCount, families[i]->getActorCount());
            totalActorCount += families[i]->getActorCount();
        }

        // 3) Joint update events should be fired when attached actors change

        for (TkActor* actor : actors)
        {
            actor->release();
        }

        EXPECT_EQ((size_t)0, tracker.joints.size());    // 4) Joint delete events should be fired when at least one attached actor is deleted

        group->release();

        releaseTestAssets();
        releaseFramework();
    }

    void assemblyExternalJoints_MultiFamilyDamage(bool explicitJointRelease = true)
    {
        createFramework();

        const NvBlastChunkDesc chunkDescs[3] =
        {
//            centroid              volume  parent idx  flags                           ID
            { { 0.0f, 0.0f, 0.0f }, 4.0f,   UINT32_MAX, NvBlastChunkDesc::NoFlags,      0 },
            { { 0.0f,-1.0f, 0.0f }, 2.0f,   0,          NvBlastChunkDesc::SupportFlag,  1 },
            { { 0.0f, 1.0f, 0.0f }, 2.0f,   0,          NvBlastChunkDesc::SupportFlag,  2 }
        };

        const NvBlastBondDesc bondDesc =
//          normal                area  centroid              userData chunks
        { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 0.0f, 0.0f, 0.0f }, 0 }, { 1, 2 } };

        TkFramework* framework = NvBlastTkFrameworkGet();

        TestFamilyTracker tracker;

        TkAssetDesc desc;
        desc.chunkCount = 3;
        desc.chunkDescs = chunkDescs;
        desc.bondCount = 1;
        desc.bondDescs = &bondDesc;
        desc.bondFlags = nullptr;
        TkAsset* asset = framework->createAsset(desc);
        EXPECT_TRUE(asset != nullptr);

        TkGroupDesc gdesc;
        gdesc.workerCount = m_taskman->getCpuDispatcher()->getWorkerCount();
        TkGroup* group = framework->createGroup(gdesc);
        EXPECT_TRUE(group != nullptr);

        m_groupTM->setGroup(group);

        TkActorDesc adesc(asset);
        TkActor* actor1 = framework->createActor(adesc);
        EXPECT_TRUE(actor1 != nullptr);
        TkActor* actor2 = framework->createActor(adesc);
        EXPECT_TRUE(actor2 != nullptr);

        group->addActor(*actor1);
        group->addActor(*actor2);

        TkFamily* family1 = &actor1->getFamily();
        TkFamily* family2 = &actor2->getFamily();

        family1->addListener(tracker);
        family2->addListener(tracker);
        tracker.insertActor(actor1);
        tracker.insertActor(actor2);

        TkJointDesc jdesc;
        jdesc.families[0] = family1;
        jdesc.families[1] = family2;
        jdesc.chunkIndices[0] = 2;
        jdesc.chunkIndices[1] = 1;
        jdesc.attachPositions[0] = NvVec3(0.0f, 1.0f, 0.0f);
        jdesc.attachPositions[1] = NvVec3(0.0f, -1.0f, 0.0f);
        TkJoint* joint = framework->createJoint(jdesc);
        EXPECT_TRUE(joint != nullptr);
        tracker.joints.insert(joint);

        NvBlastExtRadialDamageDesc radialDamage1 = getRadialDamageDesc(0, 1, 0, 2, 2);
        NvBlastExtProgramParams radialParams1 = { &radialDamage1, nullptr };
        actor1->damage(getFalloffProgram(), &radialParams1);
        NvBlastExtRadialDamageDesc radialDamage2 = getRadialDamageDesc(0, -1, 0, 2, 2);
        NvBlastExtProgramParams radialParams2 = { &radialDamage2, nullptr };
        actor2->damage(getFalloffProgram(), &radialParams2);

        m_groupTM->process();
        m_groupTM->wait();

        TkActor* actors1[2];
        TkActor* actors2[2];
        EXPECT_EQ(2, family1->getActors(actors1, 2));
        EXPECT_EQ(2, family2->getActors(actors2, 2));

        const TkJointData jdata = joint->getData();
        EXPECT_TRUE(jdata.actors[0] != nullptr);
        EXPECT_TRUE(jdata.actors[1] != nullptr);
        EXPECT_TRUE(&jdata.actors[0]->getFamily() == family1);
        EXPECT_TRUE(&jdata.actors[1]->getFamily() == family2);

        // Clean up
        if (explicitJointRelease)
        {
            joint->release();
            family2->release();
            family1->release();
            asset->release();
            releaseFramework();
        }
        else
        {
            EXPECT_EQ(1, tracker.joints.size());
            releaseFramework();
            // Commenting these out - but shouldn't we be sending delete events when we release the framework?
//          EXPECT_EQ(0, tracker.joints.size());
//          EXPECT_EQ(0, tracker.actors.size());
        }
    }

protected:
    // http://clang.llvm.org/compatibility.html#dep_lookup_bases
    // http://stackoverflow.com/questions/6592512/templates-parent-class-member-variables-not-visible-in-inherited-class

    using TkBaseTest<FailLevel, Verbosity>::testAssets;
    using TkBaseTest<FailLevel, Verbosity>::m_taskman;
    using TkBaseTest<FailLevel, Verbosity>::m_groupTM;
    using TkBaseTest<FailLevel, Verbosity>::createFramework;
    using TkBaseTest<FailLevel, Verbosity>::releaseFramework;
    using TkBaseTest<FailLevel, Verbosity>::createTestAssets;
    using TkBaseTest<FailLevel, Verbosity>::releaseTestAssets;
    using TkBaseTest<FailLevel, Verbosity>::getCubeSlicerProgram;
    using TkBaseTest<FailLevel, Verbosity>::getDefaultMaterial;
    using TkBaseTest<FailLevel, Verbosity>::getRadialDamageDesc;
    using TkBaseTest<FailLevel, Verbosity>::getFalloffProgram;
};


typedef TkCompositeTest<NvBlastMessage::Error, 1> TkCompositeTestAllowWarnings;
typedef TkCompositeTest<NvBlastMessage::Error, 1> TkCompositeTestStrict;


/*
1) Create assembly, actors and joints should be created automatically
*/

TEST_F(TkCompositeTestStrict, AssemblyCreateAndRelease_NoNRFJoints_NoSerialization)
{
    assemblyCreateAndRelease(false, false);
}

TEST_F(TkCompositeTestStrict, DISABLED_AssemblyCreateAndRelease_NoNRFJoints_AssemblySerialization)
{
    assemblyCreateAndRelease(false, true);
}

TEST_F(TkCompositeTestStrict, AssemblyCreateAndRelease_WithNRFJoints_NoSerialization)
{
    assemblyCreateAndRelease(true, false);
}

TEST_F(TkCompositeTestStrict, DISABLED_AssemblyCreateAndRelease_WithNRFJoints_AssemblySerialization)
{
    assemblyCreateAndRelease(true, true);
}


/**
2) Create an actor with internal joints.  Splitting the actor should cause joint create events to be dispatched

3) Joint update events should be fired when attached actors change

4) Joint delete events should be fired when at least one attached actor is deleted
*/

TEST_F(TkCompositeTestStrict, AssemblyInternalJoints_NoSerialization)
{
    assemblyInternalJoints(false);
}

TEST_F(TkCompositeTestStrict, DISABLED_AssemblyInternalJoints_AssemblySerialization)
{
    assemblyInternalJoints(true);
}


/**
5) Creating a composite from assets with internal joints should have expected behaviors (1-4) above
*/

TEST_F(TkCompositeTestStrict, AssemblyCompositeWithInternalJoints_NoNRFJoints_NoSerialization)
{
    assemblyCompositeWithInternalJoints(false, false);
}

TEST_F(TkCompositeTestStrict, DISABLED_AssemblyCompositeWithInternalJoints_NoNRFJoints_AssemblySerialization)
{
    assemblyCompositeWithInternalJoints(false, true);
}

TEST_F(TkCompositeTestStrict, AssemblyCompositeWithInternalJoints_WithNRFJoints_NoSerialization)
{
    assemblyCompositeWithInternalJoints(true, false);
}

TEST_F(TkCompositeTestStrict, DISABLED_AssemblyCompositeWithInternalJoints_WithNRFJoints_AssemblySerialization)
{
    assemblyCompositeWithInternalJoints(true, true);
}


/*
More tests
*/

TEST_F(TkCompositeTestStrict, AssemblyExternalJoints_MultiFamilyDamage)
{
    assemblyExternalJoints_MultiFamilyDamage(true);
}

TEST_F(TkCompositeTestStrict, AssemblyExternalJoints_MultiFamilyDamage_AutoJointRelease)
{
    assemblyExternalJoints_MultiFamilyDamage(false);
}
