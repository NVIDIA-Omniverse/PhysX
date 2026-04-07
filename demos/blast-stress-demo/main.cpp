// Copyright (c) 2024 NVIDIA Corporation. All rights reserved.

#include <NvBlastTk.h>
#include <NvBlastExtAuthoring.h>
#include <NvBlastExtAuthoringBondGenerator.h>
#include <NvBlastExtAuthoringFractureTool.h>
#include <NvBlastExtStressSolver.h>
#include <NvBlastGlobals.h>
#include <NvCMath.h>

#include <PxPhysicsAPI.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <vector>

using namespace Nv::Blast;
using namespace physx;

namespace
{

// Simple PhysX allocator & error callbacks wired to Blast globals
class PxDefaultAllocatorCallback final : public PxAllocatorCallback
{
public:
    void* allocate(size_t size, const char*, const char*, int) override
    {
        return NvBlastGlobalGetAllocatorCallback()->allocate(size, nullptr, __FILE__, __LINE__);
    }

    void deallocate(void* ptr) override
    {
        NvBlastGlobalGetAllocatorCallback()->deallocate(ptr);
    }
};

class PxDefaultErrorCallback final : public PxErrorCallback
{
public:
    void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override
    {
        NvBlastGlobalGetErrorCallback()->reportError(static_cast<nvidia::NvErrorCode::Enum>(code), message, file, line);
    }
};

struct DemoPhysX
{
    PxFoundation* foundation{};
    PxPhysics* physics{};
    PxDefaultAllocatorCallback allocator;
    PxDefaultErrorCallback errorCallback;

    DemoPhysX()
    {
        foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
        if (!foundation)
        {
            std::fprintf(stderr, "Failed to create PhysX foundation\n");
            std::exit(EXIT_FAILURE);
        }

        physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
        if (!physics)
        {
            std::fprintf(stderr, "Failed to create PhysX\n");
            std::exit(EXIT_FAILURE);
        }
    }

    ~DemoPhysX()
    {
        if (physics)
        {
            physics->release();
        }
        if (foundation)
        {
            foundation->release();
        }
    }
};

struct DemoScene
{
    DemoPhysX px;
    PxScene* scene{};
    PxMaterial* material{};
    PxRigidStatic* ground{};

    DemoScene()
    {
        PxSceneDesc desc(px.physics->getTolerancesScale());
        desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
        desc.cpuDispatcher = PxDefaultCpuDispatcherCreate(2);
        desc.filterShader = PxDefaultSimulationFilterShader;
        scene = px.physics->createScene(desc);
        if (!scene)
        {
            std::fprintf(stderr, "Failed to create PhysX scene\n");
            std::exit(EXIT_FAILURE);
        }

        material = px.physics->createMaterial(0.6f, 0.6f, 0.1f);
        ground = PxCreatePlane(*px.physics, PxPlane(0.0f, 1.0f, 0.0f, 0.0f), *material);
        scene->addActor(*ground);
    }

    ~DemoScene()
    {
        if (ground)
        {
            ground->release();
        }
        if (material)
        {
            material->release();
        }
        if (scene)
        {
            scene->release();
        }
    }
};

struct TkSeer : public TkEventListener
{
    ExtStressSolver& solver;
    std::vector<PxRigidDynamic*>& chunkActors;
    PxPhysics& physics;
    PxMaterial& material;

    TkSeer(ExtStressSolver& s, std::vector<PxRigidDynamic*>& chunks, PxPhysics& p, PxMaterial& m)
        : solver(s)
        , chunkActors(chunks)
        , physics(p)
        , material(m)
    {
    }

    void receive(const TkEvent* events, uint32_t eventCount) override
    {
        for (uint32_t i = 0; i < eventCount; ++i)
        {
            const TkEvent& evt = events[i];
            if (evt.type == TkEvent::Split)
            {
                const auto* split = evt.getPayload<TkSplitEvent>();
                PxRigidDynamic* parentBody = chunkActors[split->parentData.index];

                if (parentBody)
                {
                    solver.notifyActorDestroyed(*split->parentData.family->getTkActor(split->parentData.index)->getActorLL());
                    parentBody->release();
                }

                for (uint32_t c = 0; c < split->numChildren; ++c)
                {
                    TkActor* child = split->children[c];
                    solver.notifyActorCreated(*child->getActorLL());

                    PxTransform pose(PxVec3(0.0f, 0.0f, 0.0f));
                    PxRigidDynamic* body = physics.createRigidDynamic(pose);
                    PxShape* shape = body->createShape(PxBoxGeometry(0.25f, 0.25f, 0.25f), material);
                    PX_UNUSED(shape);
                    body->setLinearVelocity(PxVec3(0.0f));
                    chunkActors[child->getIndex()] = body;
                }
            }
        }
    }
};

struct BlastDemo
{
    TkFramework* framework{};
    TkAsset* tkAsset{};
    TkActor* tkActor{};
    TkFamily* family{};
    ExtStressSolver* stress{};
    std::unique_ptr<TkSeer> listener;
    std::vector<PxRigidDynamic*> rigidBodies;
    DemoScene scene;

    BlastDemo()
    {
        framework = NvBlastTkFrameworkCreate();
        if (!framework)
        {
            std::fprintf(stderr, "Failed to create TkFramework\n");
            std::exit(EXIT_FAILURE);
        }
    }

    ~BlastDemo()
    {
        if (stress)
        {
            stress->release();
        }
        if (tkActor)
        {
            tkActor->release();
        }
        if (tkAsset)
        {
            tkAsset->release();
        }
        if (framework)
        {
            framework->release();
        }
    }

    static Mesh* createUnitCube()
    {
        static const float vertices[] = {
            -0.5f,-0.5f,-0.5f,  0.5f,-0.5f,-0.5f,
             0.5f, 0.5f,-0.5f, -0.5f, 0.5f,-0.5f,
            -0.5f,-0.5f, 0.5f,  0.5f,-0.5f, 0.5f,
             0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f
        };

        static const uint32_t indices[] = {
            0,1,2,  0,2,3,
            4,6,5,  4,7,6,
            0,4,5,  0,5,1,
            3,2,6,  3,6,7,
            0,3,7,  0,7,4,
            1,5,6,  1,6,2
        };

        return NvBlastExtAuthoringCreateMeshOnlyTriangles(vertices, 8, const_cast<uint32_t*>(indices), 36);
    }

    void authorAsset()
    {
        std::unique_ptr<Mesh, void(*)(Mesh*)> mesh(createUnitCube(), [](Mesh* m){ if (m) m->release(); });
        FractureTool* fracTool = NvBlastExtAuthoringCreateFractureTool();
        std::unique_ptr<FractureTool, void(*)(FractureTool*)> fracture(fracTool, [](FractureTool* f){ if (f) f->release(); });

        Mesh* meshes[] = { mesh.get() };
        fracture->setSourceMeshes(meshes, 1, nullptr);

        constexpr uint32_t cellCount = 32;
        std::vector<NvcVec3> sites;
        sites.reserve(cellCount);
        std::mt19937 rng(1337);
        std::uniform_real_distribution<float> dist(-0.5f, 0.5f);
        for (uint32_t i = 0; i < cellCount; ++i)
        {
            sites.push_back({ dist(rng), dist(rng), dist(rng) });
        }

        fracture->voronoiFracturing(0, cellCount, sites.data(), false);
        fracture->finalizeFracturing();

        std::unique_ptr<BlastBondGenerator, void(*)(BlastBondGenerator*)> bondGen(
            NvBlastExtAuthoringCreateBondGenerator(nullptr),
            [](BlastBondGenerator* g){ if (g) g->release(); }
        );

        std::vector<bool> isSupport(fracture->getChunkCount(), true);

        NvBlastBondDesc* bondDescs = nullptr;
        NvBlastChunkDesc* chunkDescs = nullptr;
        const int32_t bondCount = bondGen->buildDescFromInternalFracture(fracture.get(), isSupport.data(), bondDescs, chunkDescs);
        const uint32_t chunkCount = fracture->getChunkCount();

        NvBlastAssetDesc desc{};
        desc.chunkCount = chunkCount;
        desc.chunkDescs = chunkDescs;
        desc.bondCount = static_cast<uint32_t>(bondCount);
        desc.bondDescs = bondDescs;

        const size_t assetMemSize = NvBlastGetAssetMemorySize(&desc);
        void* assetMem = NVBLAST_ALLOC(assetMemSize);
        NvBlastAsset* llAsset = NvBlastCreateAsset(assetMem, &desc, Nv::Blast::logLL);

        if (!llAsset)
        {
            std::fprintf(stderr, "Failed to create low level asset\n");
            std::exit(EXIT_FAILURE);
        }

        tkAsset = framework->createAsset(llAsset, nullptr, 0, true);
        NvBlastExtAuthoringFreeBondDesc(bondDescs, bondCount);
        NvBlastExtAuthoringFreeChunkDesc(chunkDescs, chunkCount);
    }

    void initialize()
    {
        authorAsset();

        TkActorDesc actorDesc(tkAsset);
        tkActor = framework->createActor(actorDesc);
        family = &tkActor->getFamily();

        stress = ExtStressSolver::create(*family->getFamilyLL());
        stress->setAllNodesInfoFromLL(1200.0f);

        stress->notifyActorCreated(*tkActor->getActorLL());

        const uint32_t actorCount = family->getActorCount();
        rigidBodies.resize(actorCount, nullptr);

        listener = std::make_unique<TkSeer>(*stress, rigidBodies, *scene.px.physics, *scene.material);
        family->addListener(*listener);

        PxTransform pose(PxVec3(0.0f, 2.0f, 0.0f));
        PxRigidDynamic* body = scene.px.physics->createRigidDynamic(pose);
        PxShape* shape = body->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *scene.material);
        PX_UNUSED(shape);
        body->setMass(50.0f);
        rigidBodies[tkActor->getIndex()] = body;
        scene.scene->addActor(*body);
    }

    void spawnSphere()
    {
        PxTransform pose(PxVec3(0.0f, 3.0f, -4.0f));
        PxRigidDynamic* sphere = scene.px.physics->createRigidDynamic(pose);
        PxShape* shape = sphere->createShape(PxSphereGeometry(0.35f), *scene.material);
        PX_UNUSED(shape);
        sphere->setMass(10.0f);
        sphere->setLinearVelocity(PxVec3(0.0f, -1.0f, 15.0f));
        scene.scene->addActor(*sphere);
    }

    void applyStress()
    {
        std::vector<TkActor*> actors(family->getActorCount());
        family->getActors(actors.data(), static_cast<uint32_t>(actors.size()));

        for (TkActor* actor : actors)
        {
            if (!actor)
            {
                continue;
            }
            PxRigidDynamic* body = rigidBodies[actor->getIndex()];
            if (!body)
            {
                continue;
            }

            PxVec3 lv = body->getLinearVelocity();
            PxVec3 av = body->getAngularVelocity();
            PxVec3 pos = body->getGlobalPose().p;

            NvcVec3 localForce{ lv.x * body->getMass(), lv.y * body->getMass(), lv.z * body->getMass() };
            stress->addForce(*actor->getActorLL(), { pos.x, pos.y, pos.z }, localForce);
            stress->addCentrifugalAcceleration(*actor->getActorLL(), { 0.0f, 0.0f, 0.0f }, { av.x, av.y, av.z });
        }

        stress->update();

        const uint32_t overstressed = stress->getOverstressedBondCount();
        if (overstressed == 0)
        {
            return;
        }

        std::vector<const NvBlastActor*> llActors(actors.size(), nullptr);
        std::vector<NvBlastFractureBuffers> commands(actors.size());
        uint32_t count = stress->generateFractureCommandsPerActor(llActors.data(), commands.data(), static_cast<uint32_t>(commands.size()));

        for (uint32_t i = 0; i < count; ++i)
        {
            family->applyFracture(&commands[i]);
        }
    }

    void simulate()
    {
        constexpr float dt = 1.0f / 60.0f;
        constexpr uint32_t steps = 360;

        spawnSphere();

        for (uint32_t i = 0; i < steps; ++i)
        {
            applyStress();
            scene.scene->simulate(dt);
            scene.scene->fetchResults(true);
        }
    }
};

} // anonymous namespace

int main(int argc, char** argv)
{
    PX_UNUSED(argc);
    PX_UNUSED(argv);

    BlastDemo demo;
    demo.initialize();
    demo.simulate();

    std::puts("Blast stress demo finished");
    return EXIT_SUCCESS;
}

