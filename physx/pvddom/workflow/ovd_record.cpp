// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxPhysicsAPI.h"
#include "omnipvd/PxOmniPvd.h"
#include "OmniPvdWriter.h"
#include "OmniPvdWriteStream.h"
#include "PvdDomJson.h"
#include "OvdNewFile.h"
#include "OvdCheckedOutput.h"
#include <iostream>
#include <stdexcept>
#ifndef DISABLE_CUDA_PHYSX
#include "cudamanager/PxCudaContextManager.h"
#endif

using namespace physx;
using namespace PvdDomInspect;
namespace
{
void require(bool condition, const std::string& message) { if (!condition) throw std::runtime_error(message); }

// Exclusive creation preserves captures even when a requested path aliases an existing file.
class NewCapture : public OmniPvdWriteStream
{
public:
    explicit NewCapture(const char* path) : mPath(path) {}
    ~NewCapture() override { closeStream(); }
    bool OMNI_PVD_CALL openStream() override { if (!mFile) mFile = openNewOvdFile(mPath); return mFile != nullptr; }
    bool OMNI_PVD_CALL closeStream() override
    {
        if (!mFile) return true;
        FILE* file = mFile; mFile = nullptr; return fclose(file) == 0;
    }
    bool OMNI_PVD_CALL flush() override { return mFile && fflush(mFile) == 0; }
    uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* bytes, uint64_t count) override
    { return mFile && count <= SIZE_MAX ? fwrite(bytes, 1, size_t(count), mFile) : 0; }
private:
    const char* mPath;
    FILE* mFile = nullptr;
};
struct Physics
{
    PxDefaultAllocator allocator;
    PxDefaultErrorCallback errors;
    PxFoundation* foundation = nullptr;
    PxOmniPvd* omni = nullptr;
    PxPhysics* physics = nullptr;
    PxDefaultCpuDispatcher* dispatcher = nullptr;
    PxScene* scene = nullptr;
    PxMaterial* material = nullptr;
#ifndef DISABLE_CUDA_PHYSX
    PxCudaContextManager* cuda = nullptr;
#endif
    ~Physics()
    {
        if (omni && physics) omni->stopSampling();
        PX_RELEASE(scene); PX_RELEASE(dispatcher); PX_RELEASE(material); PX_RELEASE(physics); PX_RELEASE(omni);
#ifndef DISABLE_CUDA_PHYSX
        PX_RELEASE(cuda);
#endif
        PX_RELEASE(foundation);
    }
};
struct LiveSample { uint64_t frame; PxTransform pose; PxVec3 velocity; };
size_t verify(const Document& doc, uint64_t context, const std::vector<LiveSample>& live)
{
    ObjectSelector selector; selector.name = "moving-body";
    size_t verified = 0;
    for (const auto& expected : live)
    {
        FrameRef frame{0, context, expected.frame}; ObjectIdentity object; std::string error;
        require(selectObject(doc, selector, frame, object, error), error);
        SampleResult pose, velocity;
        require(readSample(doc, object.objectId, frame, "globalPose", pose, error), error);
        require(readSample(doc, object.objectId, frame, "linearVelocity", velocity, error), error);
#ifdef OVD_RECORD_TEST_FORCE_CARRIED
        OVD_RECORD_TEST_FORCE_CARRIED(pose, velocity, expected.frame);
#endif
        double position[3], quaternion[4], v[3];
        require(readTransform(pose, position, quaternion) && readVec3(velocity, v), "Recorded values unavailable");
        for (int i = 0; i < 3; ++i)
        {
            require(std::abs(position[i] - expected.pose.p[i]) < 1e-5, "Recorded position differs from live SDK");
            require(std::abs(v[i] - expected.velocity[i]) < 1e-5, "Recorded velocity differs from live SDK");
        }
        const double q[] = {expected.pose.q.x, expected.pose.q.y, expected.pose.q.z, expected.pose.q.w};
        double angle;
        require(quaternionAngularDistance(quaternion, q, angle) && angle < 1e-5, "Recorded orientation differs from live SDK");
        if (expected.frame % 2 == 0)
            require(pose.state == SampleState::Exact && velocity.state == SampleState::Exact, "Post-step observation is not exact");
        ++verified;
    }
    return verified;
}
int run(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--version")
    {
        Json j; j.out << "{\"tool\":\"ovd-record\",\"sdkVersion\":\"" << PX_PHYSICS_VERSION_MAJOR << '.'
            << PX_PHYSICS_VERSION_MINOR << '.' << PX_PHYSICS_VERSION_BUGFIX << "\",\"build\":";
        j.text(PVDDOM_BUILD_INFO); j.out << ",\"evidenceVersion\":1}";
        if (!writeOvdJson(std::cout, j.out.str())) { std::cerr << "Could not write stdout\n"; return 2; }
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        std::cout << "ovd-record NEW_FILE [--late-steps N] [--change-step N] [--gpu]\n"
            "Records six real SDK steps, verifies pre/post values, finalizes before teardown.\n"
            "Change-step (1..6) increases X velocity by 1. Late-steps is 0..1000.\n"
            "Requires checked/profile/debug SDK. GPU unavailable exits 77. Existing files are refused.\n";
        std::cout.flush();
        if (!std::cout) { std::cerr << "Could not write stdout\n"; return 2; }
        return 0;
    }
    require(argc >= 2, "Expected new output path. See --help");
    require(argv[1][0] != '-', "Unknown leading option. See --help or --version");
    unsigned late = 0, change = 0; bool gpu = false;
    for (int i = 2; i < argc; ++i)
    {
        const std::string option = argv[i];
        if (option == "--gpu") { gpu = true; continue; }
        require(option == "--late-steps" || option == "--change-step", "Unknown option");
        require(++i < argc, "Missing option value");
        unsigned value = 0;
        require(argv[i][0] != 0, "Empty option value");
        for (const char* p = argv[i]; *p; ++p)
        { require(*p >= '0' && *p <= '9' && value <= 1000, "Invalid step count"); value = value * 10 + unsigned(*p - '0'); }
        if (option == "--late-steps") { require(value <= 1000, "Late steps exceed 1000"); late = value; }
        else { require(value >= 1 && value <= 6, "Change step must be 1..6"); change = value; }
    }
    NewCapture stream(argv[1]);
    Physics p;
    p.foundation = PxCreateFoundation(PX_PHYSICS_VERSION, p.allocator, p.errors); require(p.foundation, "Foundation failed");
    p.omni = PxCreateOmniPvd(*p.foundation); require(p.omni, "OmniPVD unavailable. Use checked/profile/debug SDK");
    p.physics = PxCreatePhysics(PX_PHYSICS_VERSION, *p.foundation, PxTolerancesScale(), false, nullptr, p.omni);
    require(p.physics, "Physics failed");
    PxSceneDesc desc(p.physics->getTolerancesScale());
    p.dispatcher = PxDefaultCpuDispatcherCreate(1); require(p.dispatcher, "CPU dispatcher failed");
    desc.cpuDispatcher = p.dispatcher; desc.filterShader = PxDefaultSimulationFilterShader;
    desc.gravity = PxVec3(0, -9.81f, 0);
    if (gpu)
    {
#ifndef DISABLE_CUDA_PHYSX
        PxCudaContextManagerDesc cudaDesc;
        p.cuda = PxCreateCudaContextManager(*p.foundation, cudaDesc, PxGetProfilerCallback());
        if (!p.cuda || !p.cuda->contextIsValid()) { std::cerr << "GPU unavailable\n"; return 77; }
        desc.cudaContextManager = p.cuda;
        desc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
        desc.broadPhaseType = PxBroadPhaseType::eGPU;
#else
        std::cerr << "GPU support not built\n"; return 77;
#endif
    }
    p.scene = p.physics->createScene(desc); require(p.scene, "Scene failed");
    p.material = p.physics->createMaterial(0.5f, 0.5f, 0.1f); require(p.material, "Material failed");
    PxRigidDynamic* body = PxCreateDynamic(*p.physics, PxTransform(PxVec3(1, 20, 3), PxQuat(0.7f, PxVec3(0, 0, 1))),
        PxSphereGeometry(0.5f), *p.material, 1.0f);
    require(body, "Actor failed"); body->setName("moving-body"); body->setLinearVelocity(PxVec3(1, 2, 3));
    body->setLinearDamping(0); body->setAngularDamping(0); p.scene->addActor(*body);
    auto step = [&]() { require(p.scene->simulate(1.0f / 60.0f) && p.scene->fetchResults(true), "Simulation failed"); };
    for (unsigned i = 0; i < late; ++i) step();
    require(stream.openStream(), "Cannot exclusively create capture output");
    OmniPvdWriter* writer = p.omni->getWriter(); require(writer, "Writer failed");
    writer->setWriteStream(stream); require(p.omni->startSampling(), "Sampling failed");
    const uint64_t context = reinterpret_cast<uintptr_t>(p.scene);
    std::vector<LiveSample> live;
    for (unsigned i = 1; i <= 6; ++i)
    {
        if (i == change) body->setLinearVelocity(body->getLinearVelocity() + PxVec3(1, 0, 0));
        live.push_back({2 * i - 1, body->getGlobalPose(), body->getLinearVelocity()});
        step();
        live.push_back({2 * i, body->getGlobalPose(), body->getLinearVelocity()});
    }
    require(p.omni->stopSampling(), "Stop sampling failed");
    require(!(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE), "Capture write failed");
    require(stream.closeStream(), "Capture finalization failed");
    Document doc; Error error;
    require(doc.load(argv[1], LoadLimits(), error), error.message);
#ifdef OVD_RECORD_TEST_PERTURB_LIVE
    OVD_RECORD_TEST_PERTURB_LIVE(live);
#endif
    const size_t verifiedSamples = verify(doc, context, live);
    Json j; j.out << "{\"sourcePath\":"; j.text(argv[1]);
    j.out << ",\"context\":"; j.id(context);
    j.out << ",\"recordingSegment\":\"0\",\"verifiedSamples\":" << verifiedSamples << ",\"finalizedBeforeTeardown\":true,\"gpu\":";
    j.boolean(gpu); j.out << ",\"sdkVersion\":\"" << PX_PHYSICS_VERSION_MAJOR << '.' << PX_PHYSICS_VERSION_MINOR << '.' << PX_PHYSICS_VERSION_BUGFIX << "\"}";
    if (!writeOvdJson(std::cout, j.out.str())) { std::cerr << "Could not write stdout\n"; return 2; }
    return 0;
}
}
int main(int argc, char** argv)
{
    try { return run(argc, argv); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
