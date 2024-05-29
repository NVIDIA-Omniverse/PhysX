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


#ifndef TKBASETEST_H
#define TKBASETEST_H


#include "NvBlastTk.h"
#include "NvBlastTkActor.h"
#include "NvTaskManager.h"
#include "NvBlastTkGroupTaskManager.h"
#include "NvCpuDispatcher.h"
#include "NsGlobals.h"

#include "BlastBaseTest.h"

#include "NvBlastExtDamageShaders.h"

#include "NvBlastIndexFns.h"
#include "TestProfiler.h"

#include "NvTask.h"

#include <thread>
#include <algorithm>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>


using namespace Nv::Blast;
using namespace nvidia;
using namespace nvidia::task;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Helpers
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NV_INLINE void ExpectArrayMatch(TkObject** arr0, size_t size0, TkObject** arr1, size_t size1)
{
    EXPECT_TRUE(size0 == size1);
    std::set<TkObject*> set0(arr0, arr0 + size0);
    std::set<TkObject*> set1(arr1, arr1 + size1);
    EXPECT_TRUE(set0 == set1);
}

class TestCpuDispatcher : public NvCpuDispatcher
{
    struct SharedContext
    {
        std::queue<NvBaseTask*> workQueue;
        std::condition_variable cv;
        std::mutex mutex;
        std::atomic<bool> quit;
    };

    void submitTask(NvBaseTask& task) override
    {
        if (m_threads.size() > 0)
        {
            std::unique_lock<std::mutex> lk(m_context.mutex);
            m_context.workQueue.push(&task);
            lk.unlock();
            m_context.cv.notify_one();
        }
        else
        {
            TEST_ZONE_BEGIN(task.getName());
            task.run();
            TEST_ZONE_END(task.getName());
            task.release();
        }
    }

    uint32_t getWorkerCount() const override { return (uint32_t)m_threads.size(); }

    static void execute(SharedContext& context)
    {
        while (!context.quit)
        {
            std::unique_lock<std::mutex> lk(context.mutex);
            if (!context.workQueue.empty())
            {
                NvBaseTask& task = *context.workQueue.front();
                context.workQueue.pop();
                lk.unlock();
                TEST_ZONE_BEGIN(task.getName());
                task.run();
                TEST_ZONE_END(task.getName());
                task.release();
            }
            else
            {
                // shared variables must be modified under the mutex in order
                // to correctly publish the modification to the waiting thread
                context.cv.wait(lk, [&]{ return !context.workQueue.empty() || context.quit; });
            }
        }
    }

    SharedContext m_context;
    std::vector<std::thread> m_threads;

public:
    TestCpuDispatcher(uint32_t numWorkers)
    {
        m_context.quit = false;
        for (uint32_t i = 0; i < numWorkers; ++i)
        {
            m_threads.push_back(std::thread(execute, std::ref(m_context)));
        }
    }

    void release()
    {
        std::unique_lock<std::mutex> lk(m_context.mutex);
        m_context.quit = true;
        lk.unlock();
        m_context.cv.notify_all();
        for (std::thread& t : m_threads)
        {
            t.join();
        }
        delete this;
    }
};


struct CSParams
{
    CSParams(uint32_t axis_, float coord_) : axis(axis_), coord(coord_) {}
    uint32_t axis;
    float coord;
};

static void CubeSlicer(NvBlastFractureBuffers* outbuf, const NvBlastGraphShaderActor* actor, const void* params)
{
    uint32_t bondFractureCount = 0;
    uint32_t bondFractureCountMax = outbuf->bondFractureCount;

    const CSParams& p = *reinterpret_cast<const CSParams*> (reinterpret_cast<const NvBlastExtProgramParams*>(params)->damageDesc);

    uint32_t currentNodeIndex = actor->firstGraphNodeIndex;
    while (!Nv::Blast::isInvalidIndex(currentNodeIndex))
    {
        for (uint32_t adj = actor->adjacencyPartition[currentNodeIndex]; adj < actor->adjacencyPartition[currentNodeIndex + 1]; ++adj)
        {
            if (currentNodeIndex < actor->adjacentNodeIndices[adj])
            {
                if (actor->assetBonds[actor->adjacentBondIndices[adj]].centroid[p.axis] == p.coord && bondFractureCount < bondFractureCountMax)
                {
                    NvBlastBondFractureData& data = outbuf->bondFractures[bondFractureCount++];
                    data.userdata = 0;
                    data.nodeIndex0 = currentNodeIndex;
                    data.nodeIndex1 = actor->adjacentNodeIndices[adj];
                    data.health = 1.0f;
                }
            }
        }
        currentNodeIndex = actor->graphNodeIndexLinks[currentNodeIndex];
    }

    outbuf->bondFractureCount = bondFractureCount;
    outbuf->chunkFractureCount = 0;

    //printf("slicer outcount %d\n", bondFractureCount);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              TkBaseTest Class
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<int FailLevel, int Verbosity>
class TkBaseTest : public BlastBaseTest<FailLevel, Verbosity>
{
public:
    TkBaseTest() : m_cpuDispatcher(), m_taskman(nullptr)
    {
    }

    virtual void SetUp() override
    {
        NvBlastInternalProfilerSetDetail(Nv::Blast::InternalProfilerDetail::LOW);
        NvBlastInternalProfilerSetPlatformEnabled(true);

        m_cpuDispatcher = new TestCpuDispatcher(4);

        m_taskman = NvTaskManager::createTaskManager(*NvBlastGlobalGetErrorCallback(), m_cpuDispatcher);
        m_groupTM = TkGroupTaskManager::create(*m_taskman);
    }

    virtual void TearDown() override
    {
        m_groupTM->release();
        m_cpuDispatcher->release();
        if (m_taskman) m_taskman->release();
    }
    
    void createFramework()
    {
        TkFramework* framework = NvBlastTkFrameworkCreate();
        EXPECT_TRUE(framework != nullptr);
        EXPECT_EQ(framework, NvBlastTkFrameworkGet());
    }

    void releaseFramework()
    {
        TkFramework* framework = NvBlastTkFrameworkGet();
        framework->release();
        EXPECT_TRUE(NvBlastTkFrameworkGet() == nullptr);
    }

    void createTestAssets(bool addInternalJoints = false)
    {
        const uint8_t cube1BondDescFlags_internalJoints[12] =
        {
            TkAssetDesc::NoFlags,
            TkAssetDesc::NoFlags,
            TkAssetDesc::NoFlags,
            TkAssetDesc::NoFlags,

            TkAssetDesc::NoFlags,
            TkAssetDesc::NoFlags,
            TkAssetDesc::NoFlags,
            TkAssetDesc::NoFlags,

            TkAssetDesc::BondJointed,
            TkAssetDesc::BondJointed,
            TkAssetDesc::BondJointed,
            TkAssetDesc::BondJointed
        };

        const uint32_t assetDescCount = sizeof(g_assetDescs) / sizeof(g_assetDescs[0]);
        TkFramework* framework = NvBlastTkFrameworkGet();
        for (uint32_t i = 0; i < assetDescCount; ++i)
        {
            TkAssetDesc desc;
            reinterpret_cast<NvBlastAssetDesc&>(desc) = g_assetDescs[i];
            desc.bondFlags = addInternalJoints ? cube1BondDescFlags_internalJoints : nullptr;
            testAssets.push_back(framework->createAsset(desc));
            EXPECT_TRUE(testAssets[i] != nullptr);
        }
    }

    TkAsset* createCubeAsset(size_t maxDepth, size_t width, int32_t supportDepth = -1, bool addInternalJoints = false)
    {
        TkFramework* framework = NvBlastTkFrameworkGet();
        GeneratorAsset cube;
        TkAssetDesc assetDesc;
        generateCube(cube, assetDesc, maxDepth, width, supportDepth);
        std::vector<uint8_t> bondFlags(assetDesc.bondCount);
        std::fill(bondFlags.begin(), bondFlags.end(), addInternalJoints ? 1 : 0);
        assetDesc.bondFlags = bondFlags.data();
        TkAsset* cubeAsset = framework->createAsset(assetDesc);
        testAssets.push_back(cubeAsset);
        return cubeAsset;
    }

    void releaseTestAssets()
    {
        for (uint32_t i = 0; i < testAssets.size(); ++i)
        {
            testAssets[i]->release();
        }
        testAssets.clear();
    }

    NvBlastExtRadialDamageDesc getRadialDamageDesc(float x, float y, float z, float minRadius = 10.0f, float maxRadius = 10.0f, float damage = 1.0f)
    {
        NvBlastExtRadialDamageDesc desc;
        desc.position[0] = x;
        desc.position[1] = y;
        desc.position[2] = z;

        desc.minRadius = minRadius;
        desc.maxRadius = maxRadius;
        desc.damage = damage;
        return desc;
    }

    NvBlastExtShearDamageDesc getShearDamageDesc(float x, float y, float z, float shearX = 1.0f, float shearY = 0.0f, float shearZ = 0.0f, float minRadius = 10.0f, float maxRadius = 10.0f, float damage = 1.0f)
    {
        NvBlastExtShearDamageDesc desc;
        desc.position[0] = x;
        desc.position[1] = y;
        desc.position[2] = z;

        desc.normal[0] = shearX;
        desc.normal[1] = shearY;
        desc.normal[2] = shearZ;

        desc.minRadius = minRadius;
        desc.maxRadius = maxRadius;
        desc.damage = damage;

        return desc;
    }

    static const NvBlastDamageProgram& getCubeSlicerProgram()
    {
        static NvBlastDamageProgram  program = { CubeSlicer, nullptr };
        return program;
    }

    static const NvBlastDamageProgram& getFalloffProgram()
    {
        static NvBlastDamageProgram program = { NvBlastExtFalloffGraphShader, NvBlastExtFalloffSubgraphShader };
        return program;
    }

    static const NvBlastDamageProgram& getShearProgram()
    {
        static NvBlastDamageProgram program = { NvBlastExtShearGraphShader, NvBlastExtShearSubgraphShader };
        return program;
    }

    static const NvBlastExtMaterial* getDefaultMaterial()
    {
        static NvBlastExtMaterial material;
        return &material;
    };

    TkFamily* familySerialization(TkFamily* family);


    std::vector<TkAsset*> testAssets;

    TestCpuDispatcher*  m_cpuDispatcher;

    NvTaskManager*      m_taskman;

    TkGroupTaskManager* m_groupTM;
};


#define TkNvErrorMask   (NvErrorCode::eINVALID_PARAMETER | NvErrorCode::eINVALID_OPERATION | NvErrorCode::eOUT_OF_MEMORY | NvErrorCode::eINTERNAL_ERROR | NvErrorCode::eABORT)
#define TkNvWarningMask (NvErrorCode::eDEBUG_WARNING | NvErrorCode::ePERF_WARNING)

typedef TkBaseTest<NvBlastMessage::Error, 1> TkTestAllowWarnings;
typedef TkBaseTest<NvBlastMessage::Warning, 1> TkTestStrict;


class TestFamilyTracker : public TkEventListener
{
public:
    TestFamilyTracker() {}

    typedef std::pair<TkFamily*, uint32_t> Actor;

    virtual void receive(const TkEvent* events, uint32_t eventCount) override
    {
        TEST_ZONE_BEGIN("TestFamilyTracker");
        for (size_t i = 0; i < eventCount; ++i)
        {
            const TkEvent& e = events[i];
            switch (e.type)
            {
            case (TkEvent::Split):
            {
                const TkSplitEvent* splitEvent = e.getPayload<TkSplitEvent>();
                EXPECT_EQ((size_t)1, actors.erase(Actor(splitEvent->parentData.family, splitEvent->parentData.index)));
                for (size_t i = 0; i < splitEvent->numChildren; ++i)
                {
                    TkActor* a = splitEvent->children[i];
                    EXPECT_TRUE(actors.insert(Actor(&a->getFamily(), a->getIndex())).second);
                }
                break;
            }
            case (TkEvent::FractureCommand):
            {
                const TkFractureCommands* fracEvent = e.getPayload<TkFractureCommands>();
                EXPECT_TRUE(!isInvalidIndex(fracEvent->tkActorData.index));
#if 0
                printf("chunks broken: %d\n", fracEvent->buffers.chunkFractureCount);
                printf("bonds  broken: %d\n", fracEvent->buffers.bondFractureCount);
                for (uint32_t t = 0; t < fracEvent->buffers.bondFractureCount; t++)
                {
                    //printf("%x ", fracEvent->buffers.bondFractures[t].userdata);
                }
                //printf("\n");
#endif
                break;
            }
            case (TkEvent::FractureEvent):
            {
                const TkFractureEvents* fracEvent = e.getPayload<TkFractureEvents>();
                EXPECT_TRUE(!isInvalidIndex(fracEvent->tkActorData.index));
                break;
            }
            case (TkEvent::JointUpdate):
            {
                const TkJointUpdateEvent* jointEvent = e.getPayload<TkJointUpdateEvent>();
                TkJoint* joint = jointEvent->joint;
                EXPECT_TRUE(joint != nullptr);

                switch (jointEvent->subtype)
                {
                case TkJointUpdateEvent::External:
                    EXPECT_TRUE(joints.end() == joints.find(joint));    // We should not have this joint yet
                    joints.insert(joint);
                    break;
                case TkJointUpdateEvent::Changed:
                    break;
                case TkJointUpdateEvent::Unreferenced:
                    EXPECT_EQ(1, joints.erase(joint));
                    joint->release();
                    break;
                }
                break;
            }
            default:
                break;
            }
        }
        TEST_ZONE_END("TestFamilyTracker");
    }

    void insertActor(const TkActor* actor)
    {
        actors.insert(TestFamilyTracker::Actor(&actor->getFamily(), actor->getIndex()));
    }
        
    void eraseActor(const TkActor* actor)
    {
        actors.erase(TestFamilyTracker::Actor(&actor->getFamily(), actor->getIndex()));
    }

    std::set<Actor> actors;
    std::set<TkJoint*> joints;
};


#endif // #ifndef TKBASETEST_H
