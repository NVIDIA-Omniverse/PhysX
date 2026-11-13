// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <sstream>
static bool perturbRun = false;
static bool carryPostStep = false;
#define OVD_RECORD_TEST_PERTURB_LIVE(live) if (perturbRun) (live)[3].velocity.y += 1e-3f
#define OVD_RECORD_TEST_FORCE_CARRIED(pose, velocity, frame) if (carryPostStep && frame == 2) pose.state = SampleState::Carried
#define main recorderMain
#include "../workflow/ovd_record.cpp"
#undef main
#include <cstdio>
#include <random>

int main()
{
    Document document;
    std::vector<LiveSample> live{{1, PxTransform(PxIdentity), PxVec3(1, 2, 3)}};
    bool sawUnloaded = false;
    try { verify(document, 100, live); }
    catch (const std::runtime_error& error)
    {
        sawUnloaded = std::string(error.what()) == "Document is not loaded";
        if (!sawUnloaded) std::cerr << "Lost verification diagnostic: " << error.what() << '\n';
    }
    if (!sawUnloaded) return 1;
    std::random_device random;
    const std::string path = "record-errors-" + std::to_string(random()) + "-" +
        std::to_string(random()) + ".ovd";
    char program[] = "ovd-record-errors";
    std::vector<char> pathBytes(path.begin(), path.end()); pathBytes.push_back(0);
    char* arguments[] = {program, pathBytes.data()};
    if (run(2, arguments) != 0) { std::cerr << "Real SDK capture failed\n"; return 1; }
    Error captureDiagnostic;
    if (!document.load(path.c_str(), LoadLimits(), captureDiagnostic))
    { std::cerr << "Real SDK capture did not load: " << captureDiagnostic.message << '\n'; return 1; }
    uint64_t context = 0;
    for (const auto& frame : document.frames())
        if (frame.recordedFrame == 1) { context = frame.context; break; }
    if (!context) { std::cerr << "First real SDK frame is missing\n"; return 1; }
    const LiveSample original{1, PxTransform(PxVec3(1, 20, 3), PxQuat(0.7f, PxVec3(0, 0, 1))), PxVec3(1, 2, 3)};
    if (verify(document, context, {original}) != 1)
    { std::cerr << "Unperturbed live value was not counted\n"; return 1; }
    for (int changed = 0; changed < 5; ++changed)
    {
        LiveSample sample = original;
        if (changed == 0) sample.pose.p.x += 1e-3f;
        if (changed == 1) sample.velocity.x += 1e-3f;
        if (changed == 2) sample.pose.q = PxQuat(0.701f, PxVec3(0, 0, 1));
        if (changed == 3) sample.pose.p.z += 1e-3f;
        if (changed == 4) sample.velocity.y += 1e-3f;
        const char* expected = changed == 0 ? "Recorded position differs from live SDK" :
            changed == 1 || changed == 4 ? "Recorded velocity differs from live SDK" :
            changed == 3 ? "Recorded position differs from live SDK" : "Recorded orientation differs from live SDK";
        try { verify(document, context, {sample}); }
        catch (const std::runtime_error& error)
        {
            if (std::string(error.what()) == expected) continue;
            std::cerr << "Wrong live-value diagnostic: " << error.what() << '\n'; return 1;
        }
        std::cerr << "Perturbed live value was accepted: " << expected << '\n';
        return 1;
    }
    ObjectSelector selector; selector.name = "moving-body";
    FrameRef postFrame{0, context, 2}; ObjectIdentity object; std::string error;
    SampleResult pose, velocity;
    if (!selectObject(document, selector, postFrame, object, error) ||
        !readSample(document, object.objectId, postFrame, "globalPose", pose, error) ||
        !readSample(document, object.objectId, postFrame, "linearVelocity", velocity, error))
    { std::cerr << "Could not read post-step sample: " << error << '\n'; return 1; }
    double position[3], rotation[4], speed[3];
    if (!readTransform(pose, position, rotation) || !readVec3(velocity, speed)) return 1;
    LiveSample post{2, PxTransform(PxVec3(float(position[0]), float(position[1]), float(position[2])),
        PxQuat(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))),
        PxVec3(float(speed[0]), float(speed[1]), float(speed[2]))};
    carryPostStep = true;
    bool rejectedCarried = false;
    try { verify(document, context, {post}); }
    catch (const std::runtime_error& failure)
    { rejectedCarried = std::string(failure.what()) == "Post-step observation is not exact"; }
    carryPostStep = false;
    if (!rejectedCarried) { std::cerr << "Carried post-step observation was accepted\n"; return 1; }

    const std::string perturbedPath = path + "-perturbed.ovd";
    std::vector<char> perturbedBytes(perturbedPath.begin(), perturbedPath.end()); perturbedBytes.push_back(0);
    char* perturbedArguments[] = {program, perturbedBytes.data()};
    std::ostringstream perturbStdout, perturbStderr;
    std::streambuf* originalStdout = std::cout.rdbuf(perturbStdout.rdbuf());
    std::streambuf* originalStderr = std::cerr.rdbuf(perturbStderr.rdbuf());
    perturbRun = true;
    const int perturbedCode = recorderMain(2, perturbedArguments);
    perturbRun = false;
    std::cout.rdbuf(originalStdout); std::cerr.rdbuf(originalStderr);
    if (perturbedCode != 2 || perturbStderr.str().find("Recorded velocity differs from live SDK") == std::string::npos ||
        !perturbStdout.str().empty())
    { std::cerr << "run() did not reject its perturbed live sample: " << perturbStderr.str() << '\n'; return 1; }
    if (std::remove(perturbedPath.c_str()) != 0)
    { std::cerr << "Could not remove expected failed-oracle capture: " << perturbedPath << '\n'; return 1; }
    if (std::remove(path.c_str()) != 0)
    { std::cerr << "Could not remove successful test capture: " << path << '\n'; return 1; }
    return 0;
}
