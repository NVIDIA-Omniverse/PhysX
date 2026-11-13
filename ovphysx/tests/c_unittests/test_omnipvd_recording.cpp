// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-OMNIPVD-CONTROL-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5

// @implements REQ-OMNIPVD-CAPTURE-001
// @covers AC-1 AC-3 AC-4 AC-5

// @implements REQ-CAPI-OMNIPVD-CAPTURE-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8

/**
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-2 AC-4 AC-5 AC-7
 *
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11
 *
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 */

// End-to-end tests for explicit OmniPVD recording. The modern process-global
// capability is configured before first create; create and attach remain dormant.
// These tests use standalone TEST() (not PhysXTestFixture) to control the
// full instance lifecycle.

#include <gtest/gtest.h>
#include "OmniPvdFileReadStream.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include "OmniPvdWriter.h"
#include "OmniPvdWriteStream.h"
#include "PxPhysics.h"
#include "PxActor.h"
#include "PxRigidActor.h"
#include "PxScene.h"
#include "PxDeformableSurface.h"
#include "PxDeformableSurfaceMaterial.h"
#include "PxDeformableVolume.h"
#include "PxDeformableVolumeMaterial.h"
#include "PxParticleBuffer.h"
#include "PxPBDParticleSystem.h"
#include "PxPBDMaterial.h"
#include "PxShape.h"
#include "geometry/PxGeometryHelpers.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTetrahedronMesh.h"
#include "omnipvd/PxOmniPvd.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysxTestHelpers.h"
#include "ovphysx_test_utils.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace test_utils;

namespace fs = std::filesystem;

#if defined(_WIN32)
using test_socket_t = SOCKET;
static constexpr test_socket_t kInvalidTestSocket = INVALID_SOCKET;
static void close_test_socket(test_socket_t socket)
{
    if (socket != kInvalidTestSocket)
        closesocket(socket);
}
static void shutdown_test_socket(test_socket_t socket)
{
    if (socket != kInvalidTestSocket)
        shutdown(socket, SD_BOTH);
}
#else
using test_socket_t = int;
static constexpr test_socket_t kInvalidTestSocket = -1;
static void close_test_socket(test_socket_t socket)
{
    if (socket != kInvalidTestSocket)
        close(socket);
}
static void shutdown_test_socket(test_socket_t socket)
{
    if (socket != kInvalidTestSocket)
        shutdown(socket, SHUT_RDWR);
}
#endif

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static ovphysx_api_status_t create_with_config(const ovphysx_config_entry_t* entries, uint32_t count)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries;
    args.config_entry_count = count;
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    const ovphysx_result_t result = ovphysx_create_instance_with_args(&args, &handle);
    if (result.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    }
    return result.status;
}

static ovphysx_result_t create_recording_capable_instance(ovphysx_instance_handle_t& handle)
{
    // CPU recording and selective cases have fresh processes. Later instances
    // use the already-latched capability without trying to change it.
    static bool configured = false;
    if (!configured)
    {
        const ovphysx_result_t result = ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, ovphysx_config_value_bool(true));
        if (result.status != OVPHYSX_API_SUCCESS)
            return result;
        configured = true;
    }
    return ovphysx_create_instance(&handle);
}

static ovphysx_result_t start_file_recording(ovphysx_instance_handle_t handle, const fs::path& path)
{
    const std::string pathString = path.u8string();
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE,
        make_ovx_string(pathString.c_str()),
        OVPHYSX_LITERAL(""),
        0,
        0,
    };
    return ovphysx_ovd_start_recording(handle, &destination, nullptr);
}

static ovphysx_result_t start_directory_recording(ovphysx_instance_handle_t handle, const fs::path& directory)
{
    // Directory inference uses the existing FILE descriptor and file_path.
    return start_file_recording(handle, directory);
}

struct DirectoryRecordingTestScope
{
    fs::path root = fs::temp_directory_path() /
        ("ovphysx_explicit_directory_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    bool attached = false;
    ~DirectoryRecordingTestScope()
    {
        if (handle != OVPHYSX_INVALID_HANDLE)
        {
            if (attached)
                destroy_ovstage_test_attachments(handle);
            ovphysx_destroy_instance(handle);
        }
        std::error_code ec;
        fs::remove_all(root, ec);
    }
};

static std::vector<fs::path> directory_captures(const fs::path& directory)
{
    std::vector<fs::path> captures;
    for (const fs::directory_entry& entry : fs::directory_iterator(directory))
        if (entry.is_regular_file() && entry.path().extension() == ".ovd")
            captures.push_back(entry.path());
    return captures;
}

static std::vector<char> capture_bytes(const fs::path& path)
{
    std::ifstream stream(path, std::ios::binary);
    return std::vector<char>(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>{});
}

static ::testing::AssertionResult create_stepped_recording_instance(ovphysx_instance_handle_t& handle)
{
    const ovphysx_result_t createResult = create_recording_capable_instance(handle);
    if (createResult.status != OVPHYSX_API_SUCCESS || handle == OVPHYSX_INVALID_HANDLE)
        return ::testing::AssertionFailure() << "Failed to create OvPhysX instance";
    if (!attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"))
    {
        ovphysx_destroy_instance(handle);
        handle = OVPHYSX_INVALID_HANDLE;
        return ::testing::AssertionFailure() << "Failed to attach the test stage";
    }
    const ovphysx_result_t stepResult = ovphysx_step_sync(handle, 1.0f / 60.0f);
    if (stepResult.status != OVPHYSX_API_SUCCESS)
    {
        destroy_ovstage_test_attachments(handle);
        ovphysx_destroy_instance(handle);
        handle = OVPHYSX_INVALID_HANDLE;
        return ::testing::AssertionFailure() << "Failed to advance the test stage before recording";
    }
    return ::testing::AssertionSuccess();
}

struct TcpCaptureResult
{
    std::vector<uint8_t> bytes;
    bool cleanEof{ false };
};

constexpr const char* kPhysXExtensionsClassNames[] = {
    "PxJoint",
    "PxFixedJoint",
    "PxPrismaticJoint",
    "PxRevoluteJoint",
    "PxSphericalJoint",
    "PxDistanceJoint",
    "PxGearJoint",
    "PxRackAndPinionJoint",
    "PxD6JointDrive",
    "PxD6Joint",
    "PxCustomGeometryExtBaseConvexCallbacks",
    "PxCustomGeometryExtCylinderCallbacks",
    "PxCustomGeometryExtConeCallbacks",
};
constexpr uint32_t kCompletePhysXExtensionsSchemaMask =
    (1u << static_cast<uint32_t>(std::size(kPhysXExtensionsClassNames))) - 1u;
constexpr uint32_t kCompletePhysXExtensionsObjectMask =
    kCompletePhysXExtensionsSchemaMask & ~(1u | (1u << 10)); // Exclude the two abstract base classes.

struct OvdCommandSummary
{
    size_t createObjectCount{ 0 };
    size_t destroyObjectCount{ 0 };
    size_t sceneObjectCount{ 0 };
    size_t actorObjectCount{ 0 };
    size_t shapeObjectCount{ 0 };
    std::set<OmniPvdClassHandle> shapeClasses;
    std::set<OmniPvdClassHandle> sceneClasses;
    std::set<OmniPvdClassHandle> actorClasses;
    std::set<OmniPvdClassHandle> volumeMeshClasses;
    std::set<OmniPvdObjectHandle> volumeMeshObjects;
    std::map<OmniPvdObjectHandle, OmniPvdObjectHandle> volumeMeshParents;
    std::set<OmniPvdObjectHandle> createdObjects;
    std::set<OmniPvdObjectHandle> discoveryTargets;
    std::map<OmniPvdObjectHandle, size_t> createCounts;
    std::map<OmniPvdAttributeHandle, std::string> attributeNames;
    std::map<OmniPvdObjectHandle, std::set<std::string>> writtenAttributes;
    std::map<OmniPvdObjectHandle, std::set<std::string>> currentAttributes;
    std::map<OmniPvdObjectHandle, std::map<std::string, std::vector<std::vector<uint8_t>>>> pointPayloads;
    uint64_t captureRequestId{ 0 };
    std::vector<std::pair<uint64_t, uint32_t>> captureResults;
    std::set<std::pair<OmniPvdContextHandle, OmniPvdObjectHandle>> liveVehicles;
    std::vector<size_t> vehicleCreatesAtAcks;
    std::vector<size_t> vehicleDestroysAtAcks;
    std::vector<size_t> liveVehiclesAtAcks;
    size_t vehicleDestroyCount{ 0 };
    std::set<std::pair<OmniPvdContextHandle, OmniPvdObjectHandle>> liveSceneObjects;
    std::vector<uint64_t> startFrameIds;
    std::vector<uint64_t> stopFrameIds;
    std::vector<std::pair<OmniPvdContextHandle, uint64_t>> stoppedFrames;
    size_t stopFrameCount{ 0 };
    bool hasVehicleSchema{ false };
    bool hasVehicleObject{ false };
    size_t vehicleObjectCount{ 0 };
    size_t physXMaterialFrictionObjectCount{ 0 };
    uint32_t physXExtensionsSchemaMask{ 0 };
    uint32_t physXExtensionsObjectMask{ 0 };
    std::array<OmniPvdClassHandle, std::size(kPhysXExtensionsClassNames)> physXExtensionsClassHandles{};
    OmniPvdClassHandle vehicleClassHandle{ OMNI_PVD_INVALID_HANDLE };
    OmniPvdClassHandle physXMaterialFrictionClassHandle{ OMNI_PVD_INVALID_HANDLE };
    OmniPvdCommand::Enum lastCommand{ OmniPvdCommand::eINVALID };
};

static ::testing::AssertionResult read_ovd_commands(const fs::path& path, OvdCommandSummary& summary)
{
    OmniPvdFileReadStream* stream = createOmniPvdFileReadStream();
    OmniPvdReader* reader = createOmniPvdReader();
    if (!stream || !reader)
    {
        if (reader)
            destroyOmniPvdReader(*reader);
        if (stream)
            destroyOmniPvdFileReadStream(*stream);
        return ::testing::AssertionFailure() << "Could not create OmniPVD reader";
    }

    stream->setFileName(path.string().c_str());
    reader->setReadStream(*stream);
    OmniPvdVersionType major = 0, minor = 0, patch = 0;
    const bool opened = reader->startReading(major, minor, patch);
    if (opened)
    {
        for (OmniPvdCommand::Enum command = reader->getNextCommand();
             command != OmniPvdCommand::eINVALID;
             command = reader->getNextCommand())
        {
            summary.lastCommand = command;
            if (command == OmniPvdCommand::eREGISTER_ATTRIBUTE)
                summary.attributeNames[reader->getAttributeHandle()] = reader->getAttributeName();
            if (command == OmniPvdCommand::eCREATE_OBJECT)
            {
                summary.createdObjects.insert(reader->getObjectHandle());
                ++summary.createCounts[reader->getObjectHandle()];
                summary.currentAttributes[reader->getObjectHandle()].clear();
                if (summary.volumeMeshClasses.count(reader->getClassHandle()))
                    summary.volumeMeshObjects.insert(reader->getObjectHandle());
            }
            if (command == OmniPvdCommand::eSET_ATTRIBUTE)
            {
                summary.writtenAttributes[reader->getObjectHandle()].insert(
                    summary.attributeNames[reader->getAttributeHandle()]);
                summary.currentAttributes[reader->getObjectHandle()].insert(
                    summary.attributeNames[reader->getAttributeHandle()]);
                const std::string& name = summary.attributeNames[reader->getAttributeHandle()];
                if (reader->getContextHandle() == 2 && name == "targetHandle" &&
                    reader->getAttributeDataLength() == sizeof(OmniPvdObjectHandle))
                {
                    OmniPvdObjectHandle target = 0;
                    std::memcpy(&target, reader->getAttributeDataPointer(), sizeof(target));
                    summary.discoveryTargets.insert(target);
                }
                if (name == "simulationMesh" && summary.volumeMeshObjects.count(reader->getObjectHandle()) &&
                    reader->getAttributeDataLength() == sizeof(OmniPvdObjectHandle))
                {
                    OmniPvdObjectHandle child = 0;
                    std::memcpy(&child, reader->getAttributeDataPointer(), sizeof(child));
                    summary.volumeMeshParents[child] = reader->getObjectHandle();
                }
                if ((name == "positions" || name == "collisionPositions" || name == "simulationPositions" ||
                     name == "velocities" || name == "positionInvMasses" || name == "phases") &&
                    reader->getAttributeDataLength())
                {
                    const uint8_t* data = static_cast<const uint8_t*>(reader->getAttributeDataPointer());
                    summary.pointPayloads[reader->getObjectHandle()][name].emplace_back(
                        data, data + reader->getAttributeDataLength());
                }
                if (name == "captureRequestId" && reader->getAttributeDataLength() == sizeof(uint64_t))
                    std::memcpy(&summary.captureRequestId, reader->getAttributeDataPointer(), sizeof(uint64_t));
                if (name == "captureRequestStatus" && reader->getAttributeDataLength() == sizeof(uint32_t))
                {
                    uint32_t status = 0;
                    std::memcpy(&status, reader->getAttributeDataPointer(), sizeof(status));
                    if (summary.captureRequestId && status)
                    {
                        summary.captureResults.emplace_back(summary.captureRequestId, status);
                        summary.vehicleCreatesAtAcks.push_back(summary.vehicleObjectCount);
                        summary.vehicleDestroysAtAcks.push_back(summary.vehicleDestroyCount);
                        summary.liveVehiclesAtAcks.push_back(summary.liveVehicles.size());
                    }
                }
            }
            summary.createObjectCount += command == OmniPvdCommand::eCREATE_OBJECT;
            summary.destroyObjectCount += command == OmniPvdCommand::eDESTROY_OBJECT;
            if (command == OmniPvdCommand::eSTART_FRAME)
                summary.startFrameIds.push_back(reader->getFrameTimeStart());
            if (command == OmniPvdCommand::eSTOP_FRAME)
            {
                summary.stopFrameIds.push_back(reader->getFrameTimeStop());
                summary.stoppedFrames.emplace_back(reader->getContextHandle(), reader->getFrameTimeStop());
            }
            summary.stopFrameCount += command == OmniPvdCommand::eSTOP_FRAME;
            summary.hasVehicleSchema |= command == OmniPvdCommand::eREGISTER_CLASS &&
                                        std::strcmp(reader->getClassName(), "DirectDrivetrain") == 0;
            if (command == OmniPvdCommand::eREGISTER_CLASS)
            {
                if (std::strcmp(reader->getClassName(), "PxDeformableVolumeMesh") == 0)
                    summary.volumeMeshClasses.insert(reader->getClassHandle());
                if (std::strcmp(reader->getClassName(), "PxShape") == 0)
                    summary.shapeClasses.insert(reader->getClassHandle());
                if (std::strcmp(reader->getClassName(), "PxScene") == 0)
                    summary.sceneClasses.insert(reader->getClassHandle());
                if (std::strcmp(reader->getClassName(), "PxRigidDynamic") == 0 ||
                    std::strcmp(reader->getClassName(), "PxRigidStatic") == 0 ||
                    std::strcmp(reader->getClassName(), "PxArticulationLink") == 0)
                    summary.actorClasses.insert(reader->getClassHandle());
                for (uint32_t i = 0; i < static_cast<uint32_t>(std::size(kPhysXExtensionsClassNames)); ++i)
                {
                    if (std::strcmp(reader->getClassName(), kPhysXExtensionsClassNames[i]) == 0)
                    {
                        summary.physXExtensionsSchemaMask |= 1u << i;
                        summary.physXExtensionsClassHandles[i] = reader->getClassHandle();
                    }
                }
            }
            if (command == OmniPvdCommand::eREGISTER_CLASS && std::strcmp(reader->getClassName(), "Vehicle") == 0)
                summary.vehicleClassHandle = reader->getClassHandle();
            if (command == OmniPvdCommand::eREGISTER_CLASS &&
                std::strcmp(reader->getClassName(), "PhysXMaterialFriction") == 0)
            {
                summary.physXMaterialFrictionClassHandle = reader->getClassHandle();
            }
            const bool isVehicleObject = summary.vehicleClassHandle != OMNI_PVD_INVALID_HANDLE &&
                                         reader->getClassHandle() == summary.vehicleClassHandle;
            summary.vehicleObjectCount += command == OmniPvdCommand::eCREATE_OBJECT && isVehicleObject;
            if (command == OmniPvdCommand::eCREATE_OBJECT && isVehicleObject)
                summary.liveVehicles.emplace(reader->getContextHandle(), reader->getObjectHandle());
            if (command == OmniPvdCommand::eDESTROY_OBJECT &&
                summary.liveVehicles.erase({reader->getContextHandle(), reader->getObjectHandle()}))
                ++summary.vehicleDestroyCount;
            summary.hasVehicleObject = summary.vehicleObjectCount != 0;
            summary.shapeObjectCount += command == OmniPvdCommand::eCREATE_OBJECT &&
                                        summary.shapeClasses.count(reader->getClassHandle()) != 0;
            summary.physXMaterialFrictionObjectCount +=
                command == OmniPvdCommand::eCREATE_OBJECT &&
                summary.physXMaterialFrictionClassHandle != OMNI_PVD_INVALID_HANDLE &&
                reader->getClassHandle() == summary.physXMaterialFrictionClassHandle;
            if (command == OmniPvdCommand::eCREATE_OBJECT)
            {
                const bool scene = summary.sceneClasses.count(reader->getClassHandle()) != 0;
                const bool actor = summary.actorClasses.count(reader->getClassHandle()) != 0;
                summary.sceneObjectCount += scene;
                summary.actorObjectCount += actor;
                if (scene || actor)
                    summary.liveSceneObjects.emplace(reader->getContextHandle(), reader->getObjectHandle());
                for (uint32_t i = 0; i < static_cast<uint32_t>(std::size(kPhysXExtensionsClassNames)); ++i)
                {
                    if (summary.physXExtensionsClassHandles[i] != OMNI_PVD_INVALID_HANDLE &&
                        reader->getClassHandle() == summary.physXExtensionsClassHandles[i])
                    {
                        summary.physXExtensionsObjectMask |= 1u << i;
                    }
                }
            }
            if (command == OmniPvdCommand::eDESTROY_OBJECT)
            {
                summary.liveSceneObjects.erase({reader->getContextHandle(), reader->getObjectHandle()});
                summary.currentAttributes.erase(reader->getObjectHandle());
            }
        }
    }
    stream->closeStream();
    destroyOmniPvdReader(*reader);
    destroyOmniPvdFileReadStream(*stream);
    if (!opened)
        return ::testing::AssertionFailure() << "Could not parse OVD capture " << path;
    return ::testing::AssertionSuccess();
}

static ::testing::AssertionResult has_physics_commands(
    const fs::path& path,
    bool teardown = false,
    bool expectVehicleSchema = true,
    bool expectVehicleObject = false,
    bool requirePhysXExtensions = false,
    size_t expectedPhysXMaterialFrictionObjectCount = 0)
{
    OvdCommandSummary summary;
    const ::testing::AssertionResult parsed = read_ovd_commands(path, summary);
    if (!parsed)
        return parsed;
    if (summary.createObjectCount == 0)
        return ::testing::AssertionFailure() << "OVD capture has no CREATE_OBJECT command: " << path;
    if (summary.stopFrameCount == 0)
        return ::testing::AssertionFailure()
               << "OVD capture has no completed simulation frame (last command "
               << static_cast<int>(summary.lastCommand) << ", " << summary.createObjectCount
               << " objects, Vehicle schema " << summary.hasVehicleSchema << ", Vehicle object "
               << summary.hasVehicleObject << "): " << path;
    if (summary.hasVehicleSchema != expectVehicleSchema)
        return ::testing::AssertionFailure() << "OVD capture Vehicle SDK telemetry presence did not match expectation: " << path;
    if (summary.hasVehicleObject != expectVehicleObject)
        return ::testing::AssertionFailure() << "OVD capture Vehicle object presence did not match expectation: " << path;
    if (expectedPhysXMaterialFrictionObjectCount != 0 &&
        summary.physXMaterialFrictionObjectCount != expectedPhysXMaterialFrictionObjectCount)
    {
        return ::testing::AssertionFailure()
               << "OVD capture contains " << summary.physXMaterialFrictionObjectCount
               << " PhysXMaterialFriction objects; expected " << expectedPhysXMaterialFrictionObjectCount << ": " << path;
    }
    if (requirePhysXExtensions &&
        (summary.physXExtensionsSchemaMask != kCompletePhysXExtensionsSchemaMask ||
         summary.physXExtensionsObjectMask != kCompletePhysXExtensionsObjectMask))
    {
        return ::testing::AssertionFailure()
               << "OVD capture is missing PhysXExtensions schema or live objects (schema mask "
               << summary.physXExtensionsSchemaMask << ", object mask "
               << summary.physXExtensionsObjectMask << "): " << path;
    }
    // Detach retires scenes and actors while the shared SDK can remain alive.
    // Verify their matching destruction instead of assuming SDK destruction is
    // the last serialized command (collection removals may follow it).
    if (teardown && (summary.destroyObjectCount == 0 || summary.sceneObjectCount == 0 ||
                     summary.actorObjectCount == 0 || !summary.liveSceneObjects.empty()))
        return ::testing::AssertionFailure() << "OVD teardown did not destroy all recorded scenes and actors: " << path;
    return ::testing::AssertionSuccess();
}

template <typename Producer>
static ::testing::AssertionResult capture_loopback_tcp(
    Producer&& producer, TcpCaptureResult& captureResult,
    std::shared_ptr<const std::vector<uint8_t>> control = nullptr,
    std::shared_ptr<std::atomic<bool>> controlSent = nullptr,
    bool halfCloseControl = false,
    std::shared_ptr<std::atomic<bool>> controlPermit = nullptr)
{
#if defined(_WIN32)
    WSADATA socketData{};
    if (WSAStartup(MAKEWORD(2, 2), &socketData) != 0)
        return ::testing::AssertionFailure() << "WSAStartup failed";
#endif

    const test_socket_t listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == kInvalidTestSocket)
    {
#if defined(_WIN32)
        WSACleanup();
#endif
        return ::testing::AssertionFailure() << "Could not create loopback listener";
    }

    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    endpoint.sin_port = 0;
    if (bind(listener, reinterpret_cast<const sockaddr*>(&endpoint), sizeof(endpoint)) != 0 ||
        listen(listener, 1) != 0)
    {
        close_test_socket(listener);
#if defined(_WIN32)
        WSACleanup();
#endif
        return ::testing::AssertionFailure() << "Could not bind loopback listener";
    }

    socklen_t endpointSize = sizeof(endpoint);
    if (getsockname(listener, reinterpret_cast<sockaddr*>(&endpoint), &endpointSize) != 0)
    {
        close_test_socket(listener);
#if defined(_WIN32)
        WSACleanup();
#endif
        return ::testing::AssertionFailure() << "Could not read loopback listener port";
    }
    const uint16_t port = ntohs(endpoint.sin_port);

    struct ReaderState
    {
#if defined(_WIN32)
        ~ReaderState()
        {
            WSACleanup();
        }
#endif
        std::atomic<test_socket_t> peer{ kInvalidTestSocket };
        std::atomic<bool> cancelled{ false };
        std::promise<TcpCaptureResult> result;
        std::shared_ptr<const std::vector<uint8_t>> control;
        std::shared_ptr<std::atomic<bool>> controlSent;
        std::shared_ptr<std::atomic<bool>> controlPermit;
        bool halfCloseControl{ false };
    };
    const std::shared_ptr<ReaderState> readerState = std::make_shared<ReaderState>();
    readerState->control = std::move(control);
    readerState->controlSent = std::move(controlSent);
    readerState->controlPermit = std::move(controlPermit);
    readerState->halfCloseControl = halfCloseControl;
    std::future<TcpCaptureResult> received = readerState->result.get_future();
    std::thread reader([listener, readerState]() {
        TcpCaptureResult result;
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(listener, &readSet);
        timeval acceptTimeout{ 5, 0 };
        if (select(static_cast<int>(listener + 1), &readSet, nullptr, nullptr, &acceptTimeout) > 0)
        {
            const test_socket_t peer = accept(listener, nullptr, nullptr);
            readerState->peer.store(peer);
            if (peer != kInvalidTestSocket)
            {
#if defined(_WIN32)
                const DWORD timeoutMs = 5000;
                setsockopt(peer, SOL_SOCKET, SO_RCVTIMEO,
                    reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));
#else
                timeval receiveTimeout{ 5, 0 };
                setsockopt(peer, SOL_SOCKET, SO_RCVTIMEO, &receiveTimeout, sizeof(receiveTimeout));
#endif
                char buffer[4096];
                for (;;)
                {
                    const int count = recv(peer, buffer, sizeof(buffer), 0);
                    if (count <= 0)
                    {
                        result.cleanEof = count == 0 && !readerState->cancelled.load();
                        break;
                    }
                    result.bytes.insert(result.bytes.end(), buffer, buffer + count);
                    if (readerState->control &&
                        (!readerState->controlPermit || readerState->controlPermit->load()))
                    {
                        size_t sent = 0;
                        while (sent < readerState->control->size())
                        {
                            const int written = send(peer,
                                reinterpret_cast<const char*>(readerState->control->data() + sent),
                                static_cast<int>(readerState->control->size() - sent), 0);
                            if (written <= 0)
                                break;
                            sent += static_cast<size_t>(written);
                        }
                        if (readerState->controlSent)
                            readerState->controlSent->store(sent == readerState->control->size());
                        readerState->control.reset();
                    }
                    if (readerState->halfCloseControl)
                    {
#if defined(_WIN32)
                        shutdown(peer, SD_SEND);
#else
                        shutdown(peer, SHUT_WR);
#endif
                        readerState->halfCloseControl = false;
                        if (readerState->controlSent)
                            readerState->controlSent->store(true);
                    }
                }
                close_test_socket(peer);
            }
        }
        readerState->result.set_value(result);
    });

    const ovphysx_api_status_t producerStatus = producer(port);
    close_test_socket(listener);
    std::future_status readerStatus = received.wait_for(std::chrono::seconds(6));
    if (readerStatus != std::future_status::ready)
    {
        readerState->cancelled.store(true);
        shutdown_test_socket(readerState->peer.load());
        readerStatus = received.wait_for(std::chrono::seconds(1));
    }
    if (readerStatus == std::future_status::ready)
    {
        reader.join();
        captureResult = received.get();
    }
    else
    {
        reader.detach();
    }

    if (producerStatus != OVPHYSX_API_SUCCESS)
        return ::testing::AssertionFailure() << "TCP producer failed with status " << producerStatus;
    if (readerStatus != std::future_status::ready)
        return ::testing::AssertionFailure() << "TCP reader did not finish within its bounded timeout";
    return ::testing::AssertionSuccess();
}

template <typename Producer>
static ::testing::AssertionResult with_refused_loopback_tcp(Producer&& producer)
{
#if defined(_WIN32)
    WSADATA socketData{};
    if (WSAStartup(MAKEWORD(2, 2), &socketData) != 0)
        return ::testing::AssertionFailure() << "WSAStartup failed";
#endif

    const test_socket_t reserved = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (reserved == kInvalidTestSocket)
    {
#if defined(_WIN32)
        WSACleanup();
#endif
        return ::testing::AssertionFailure() << "Could not create reserved loopback socket";
    }
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    endpoint.sin_port = 0;
    socklen_t endpointSize = sizeof(endpoint);
    if (bind(reserved, reinterpret_cast<const sockaddr*>(&endpoint), sizeof(endpoint)) != 0 ||
        getsockname(reserved, reinterpret_cast<sockaddr*>(&endpoint), &endpointSize) != 0)
    {
        close_test_socket(reserved);
#if defined(_WIN32)
        WSACleanup();
#endif
        return ::testing::AssertionFailure() << "Could not reserve a non-listening loopback port";
    }

    producer(ntohs(endpoint.sin_port));
    close_test_socket(reserved);
#if defined(_WIN32)
    WSACleanup();
#endif
    return ::testing::AssertionSuccess();
}

static ::testing::AssertionResult recording_capability_is(bool expected)
{
    ovphysx_config_value_t value{};
    const ovphysx_result_t result = ovphysx_get_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, &value);
    if (result.status != OVPHYSX_API_SUCCESS)
        return ::testing::AssertionFailure() << "Recording capability getter failed: " << result.status;
    if (value.type != OVPHYSX_CONFIG_VALUE_BOOL)
        return ::testing::AssertionFailure() << "Recording capability returned type " << value.type;
    if (value.value.bool_value != expected)
        return ::testing::AssertionFailure() << "Recording capability was " << value.value.bool_value
                                            << "; expected " << expected;
    return ::testing::AssertionSuccess();
}

// Each case owns initialization and runs in a fresh process: modern startup
// settings remain frozen after the last instance is destroyed until shutdown.
static ovphysx_result_t start_discovery(
    ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination,
    const ovphysx_omnipvd_recording_options_t* scope)
{
    ovphysx_omnipvd_recording_options_t options = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    if (scope)
        options = *scope;
    options.wait_for_selection = true;
    return ovphysx_ovd_start_recording(handle, destination, &options);
}

TEST(OmniPvdGlobalConfig, PendingBoolValueRejectsWrongTypes)
{
    EXPECT_TRUE(recording_capability_is(false));
    const ovphysx_config_value_t wrongTypes[] = {
        ovphysx_config_value_int64(0),
        ovphysx_config_value_double(0.0),
        ovphysx_config_value_string(OVPHYSX_LITERAL("false")),
    };
    for (bool enabled : { true, false })
    {
        SCOPED_TRACE(enabled);
        ASSERT_EQ(ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
            ovphysx_config_value_bool(enabled)).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(recording_capability_is(enabled));
        for (const ovphysx_config_value_t& wrongType : wrongTypes)
        {
            SCOPED_TRACE(wrongType.type);
            EXPECT_EQ(ovphysx_set_global_config(
                OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE, wrongType).status,
                OVPHYSX_API_INVALID_ARGUMENT);
            EXPECT_TRUE(recording_capability_is(enabled));
        }
    }
}

TEST(OmniPvdGlobalConfig, RetiredOvdEntriesAreRejected)
{
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    const ovphysx_config_entry_t retiredEntries[] = {
        ovphysx_config_entry_bool(static_cast<ovphysx_config_bool_t>(3), false),
        ovphysx_config_entry_bool(static_cast<ovphysx_config_bool_t>(4), false),
        ovphysx_config_entry_int32(static_cast<ovphysx_config_int32_t>(2), 5425),
        ovphysx_config_entry_int32(static_cast<ovphysx_config_int32_t>(3), 1000),
        ovphysx_config_entry_string(static_cast<ovphysx_config_string_t>(0), OVPHYSX_LITERAL("")),
        ovphysx_config_entry_string(static_cast<ovphysx_config_string_t>(2), OVPHYSX_LITERAL("file")),
        ovphysx_config_entry_string(static_cast<ovphysx_config_string_t>(3), OVPHYSX_LITERAL("127.0.0.1")),
    };
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    for (bool liveInstance : { false, true })
    {
        SCOPED_TRACE(liveInstance);
        if (liveInstance)
        {
            ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
        }
        for (const ovphysx_config_entry_t& entry : retiredEntries)
        {
            SCOPED_TRACE(entry.key_type);
            EXPECT_EQ(ovphysx_set_global_config_entry(entry).status, OVPHYSX_API_INVALID_ARGUMENT);
            const ovphysx_string_t error = ovphysx_get_last_error();
            EXPECT_NE(std::string(error.ptr, error.length).find("OVD_RECORDING_CAPABLE"), std::string::npos);
            EXPECT_EQ(create_with_config(&entry, 1), OVPHYSX_API_INVALID_ARGUMENT);
            EXPECT_TRUE(recording_capability_is(true));
        }
    }
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

TEST(OmniPvdGlobalConfig, RetiredRawOvdPathsAreRejected)
{
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    const char* pathsAndValues[][2] = {
        { "/physics/omniPvdRecordingCapable", "false" },
        { "/physics/omniPvdOutputEnabled", "true" },
        { "/persistent/physics/omniPvdOvdRecordingDirectory", "" },
        { "/physics/omniPvdTransport", "file" },
        { "/physics/omniPvdTcpAddress", "127.0.0.1" },
        { "/physics/omniPvdTcpPort", "5425" },
        { "/physics/omniPvdTcpTimeoutMs", "1000" },
    };
    for (const char* const* pathAndValue : pathsAndValues)
    {
        for (size_t prefixLength : { size_t(0), size_t(1) })
        {
            // Exclude the trailing sentinel from the borrowed view: matching
            // must honor its length, not require a null-terminated C string.
            const std::string storage = std::string(pathAndValue[0] + prefixLength) + "!";
            const ovphysx_string_t path = { storage.data(), storage.size() - 1 };
            SCOPED_TRACE(storage);
            const ovphysx_config_entry_t entry = ovphysx_config_entry_carbonite(
                path, make_ovx_string(pathAndValue[1]));
            EXPECT_EQ(ovphysx_set_global_config_entry(entry).status, OVPHYSX_API_INVALID_ARGUMENT);
            EXPECT_EQ(create_with_config(&entry, 1), OVPHYSX_API_INVALID_ARGUMENT);
            EXPECT_TRUE(recording_capability_is(true));
        }
    }
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

TEST(OmniPvdGlobalConfig, RetiredOvdGettersAreRejected)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    bool boolValue = false;
    for (int slot : { 3, 4 })
    {
        EXPECT_EQ(ovphysx_get_global_config_bool(
            static_cast<ovphysx_config_bool_t>(slot), &boolValue).status, OVPHYSX_API_INVALID_ARGUMENT);
        const ovphysx_string_t error = ovphysx_get_last_error();
        EXPECT_NE(std::string(error.ptr, error.length).find("OVD_RECORDING_CAPABLE"), std::string::npos);
    }
    int32_t intValue = 0;
    for (int slot : { 2, 3 })
    {
        EXPECT_EQ(ovphysx_get_global_config_int32(
            static_cast<ovphysx_config_int32_t>(slot), &intValue).status, OVPHYSX_API_INVALID_ARGUMENT);
        const ovphysx_string_t error = ovphysx_get_last_error();
        EXPECT_NE(std::string(error.ptr, error.length).find("ovphysx_ovd_start_recording"), std::string::npos);
    }
    char buffer[64]{};
    for (int slot : { 0, 2, 3 })
    {
        ovphysx_string_t value = { buffer, sizeof(buffer) };
        size_t requiredSize = 0;
        EXPECT_EQ(ovphysx_get_global_config_string(
            static_cast<ovphysx_config_string_t>(slot), &value, &requiredSize).status,
            OVPHYSX_API_INVALID_ARGUMENT);
        const ovphysx_string_t error = ovphysx_get_last_error();
        EXPECT_NE(std::string(error.ptr, error.length).find("ovphysx_ovd_start_recording"), std::string::npos);
    }
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
}

TEST(OmniPvdGlobalConfig, PlainCreateSupportsLateRecording)
{
    const fs::path capturePath = fs::temp_directory_path() / "ovphysx_global_capability_late.ovd";
    std::error_code ec;
    fs::remove(capturePath, ec);
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording_capability_is(true));
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_FALSE(fs::exists(capturePath));
    ASSERT_EQ(start_file_recording(handle, capturePath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(has_physics_commands(capturePath));

    destroy_ovstage_test_attachments(handle);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    fs::remove(capturePath, ec);
}

TEST(OmniPvdGlobalConfig, InheritedOutputEnabledDoesNotAutoStart)
{
    // Model a host that seeds the old output flag before the next runtime starts.
    // The raw-setting fixture bootstraps Carbonite after full SDK shutdown.
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t primer = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&primer).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(primer).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    const fs::path directory = fs::temp_directory_path() / "ovphysx_inherited_output";
    const fs::path capturePath = directory / "explicit.ovd";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);
    const std::string directoryString = directory.string();
    ASSERT_TRUE(ovphysx_set_raw_string_setting_for_test_internal(
        "/persistent/physics/omniPvdOvdRecordingDirectory",
        directoryString.data(), directoryString.size()));
    ASSERT_TRUE(ovphysx_set_raw_string_setting_for_test_internal(
        "/physics/omniPvdOutputEnabled", "true", 4));
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_TRUE(fs::is_empty(directory));
    ASSERT_EQ(start_file_recording(handle, capturePath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(has_physics_commands(capturePath));
    EXPECT_TRUE(destroy_ovstage_test_attachments(handle));
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    fs::remove_all(directory, ec);
}

TEST(OmniPvdGlobalConfig, InheritedOutputEnabledDoesNotProvideCapability)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t primer = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&primer).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(primer).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);

    const fs::path directory = fs::temp_directory_path() / "ovphysx_inherited_output_no_capability";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);
    const std::string directoryString = directory.string();
    ASSERT_TRUE(ovphysx_set_raw_string_setting_for_test_internal(
        "/persistent/physics/omniPvdOvdRecordingDirectory",
        directoryString.data(), directoryString.size()));
    ASSERT_TRUE(ovphysx_set_raw_string_setting_for_test_internal(
        "/physics/omniPvdOutputEnabled", "true", 4));

    // Do not set the modern key: inherited output must not create a provider.
    ASSERT_TRUE(recording_capability_is(false));
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    struct LogCaptureGuard
    {
        uint32_t originalLevel = ovphysx_get_log_level();
        ~LogCaptureGuard()
        {
            ovphysx_log_capture_stop();
            ovphysx_set_log_level(originalLevel);
        }
    } logCapture;
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_WARNING).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "OVD_RECORDING_CAPABLE"));
    EXPECT_TRUE(recording_capability_is(false));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    EXPECT_TRUE(recording_capability_is(false));
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_EQ(start_file_recording(handle, directory / "must_not_record.ovd").status,
              OVPHYSX_API_INVALID_STATE);
    const ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("OVD_RECORDING_CAPABLE"), std::string::npos);
    EXPECT_TRUE(fs::is_empty(directory));

    EXPECT_TRUE(destroy_ovstage_test_attachments(handle));
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    fs::remove_all(directory, ec);
}

TEST(OmniPvdGlobalConfig, ModernWritesStayFrozenUntilShutdown)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    const ovphysx_config_entry_t invalidThreads = ovphysx_config_entry_num_threads(256);
    EXPECT_EQ(create_with_config(&invalidThreads, 1), OVPHYSX_API_INVALID_ARGUMENT);
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(false)).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_TRUE(recording_capability_is(true));
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(false)).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_TRUE(recording_capability_is(true));

    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording_capability_is(true));
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(false)).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording_capability_is(false));
}

TEST(OmniPvdGlobalConfig, ShutdownWithLiveInstanceKeepsModernWritesFrozen)
{
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_TRUE(recording_capability_is(false));
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_TRUE(recording_capability_is(false));

    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
        ovphysx_config_value_bool(true)).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording_capability_is(true));
}

TEST(OmniPvdColdCreation, DefaultInstanceHasNoProviderAndRejectsLateStart)
{
    const fs::path capturePath =
        fs::temp_directory_path() / "ovphysx_default_must_not_record.ovd";
    std::error_code ec;
    fs::remove(capturePath, ec);
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&handle).status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t startResult = start_file_recording(handle, capturePath);
    EXPECT_EQ(startResult.status, OVPHYSX_API_INVALID_STATE);
    const ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("OVD_RECORDING_CAPABLE"), std::string::npos);

    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* physicsPtr = test_utils::read_physics_singleton(handle);
    ASSERT_NE(physicsPtr, nullptr);
    EXPECT_EQ(static_cast<physx::PxPhysics*>(physicsPtr)->getOmniPvd(), nullptr);

    startResult = start_file_recording(handle, capturePath);
    EXPECT_EQ(startResult.status, OVPHYSX_API_INVALID_STATE);
    const ovphysx_string_t attachedError = ovphysx_get_last_error();
    EXPECT_NE(
        std::string(attachedError.ptr, attachedError.length).find("OVD_RECORDING_CAPABLE"), std::string::npos);

    destroy_ovstage_test_attachments(handle);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    fs::remove(capturePath, ec);
}

TEST(OmniPvdColdCreation, DirectoryStartRequiresCapabilityWithoutCreatingOutput)
{
    DirectoryRecordingTestScope scope;
    const fs::path directory = scope.root / "nested" / "captures";
    const fs::path exactFile = scope.root / "missing" / "capture.ovd";
    ASSERT_EQ(ovphysx_create_instance(&scope.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(start_file_recording(scope.handle, directory).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(start_file_recording(scope.handle, exactFile).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(scope.root));
    ASSERT_TRUE(attach_usd_with_ovstage(scope.handle, "tests/data/simple_physics_scene.usda"));
    scope.attached = true;
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* physics = test_utils::read_physics_singleton(scope.handle);
    ASSERT_NE(physics, nullptr);
    EXPECT_EQ(static_cast<physx::PxPhysics*>(physics)->getOmniPvd(), nullptr);
    EXPECT_EQ(start_file_recording(scope.handle, directory).status, OVPHYSX_API_INVALID_STATE);
    const ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("OVD_RECORDING_CAPABLE"), std::string::npos);
    EXPECT_EQ(start_file_recording(scope.handle, exactFile).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(scope.root));
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(scope.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
}

TEST(OmniPvdRecording, DirectoryRecordingCapturesFirstFrameAndPreservesPriorFiles)
{
    DirectoryRecordingTestScope scope;
    const fs::path directory = scope.root / "nested" / "captures";
    const fs::path takeover = scope.root / "must_not_create";
    ASSERT_EQ(create_recording_capable_instance(scope.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(start_directory_recording(scope.handle, directory).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(scope.root));
    ASSERT_TRUE(attach_usd_with_ovstage(scope.handle, "tests/data/simple_physics_scene.usda"));
    scope.attached = true;
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(scope.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_FALSE(fs::exists(scope.root));

    struct LogCaptureGuard
    {
        uint32_t originalLevel = ovphysx_get_log_level();
        ~LogCaptureGuard()
        {
            ovphysx_log_capture_stop();
            ovphysx_set_log_level(originalLevel);
        }
    } logCapture;
    ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_INFO).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_log_capture_start().status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(start_directory_recording(scope.handle, directory).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(fs::is_directory(directory));
    EXPECT_EQ(start_directory_recording(scope.handle, takeover).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(takeover));
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    std::vector<fs::path> captures = directory_captures(directory);
    ASSERT_EQ(captures.size(), 1u);
    const fs::path first = captures.front();
    const std::string firstRecordingMessage = "OmniPvd: recording to '" + first.u8string() + "'";
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, firstRecordingMessage.c_str()));
    EXPECT_TRUE(std::regex_match(first.filename().string(),
        std::regex("[0-9]{4}_[0-9]{2}_[0-9]{2}_[0-9]{2}_[0-9]{2}_[0-9]{2}_[0-9]{2,}_rec\\.ovd")));
    EXPECT_FALSE(fs::exists(directory / "tmp.ovd"));
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(first, summary));
    EXPECT_GT(summary.createObjectCount, 0u);
    EXPECT_EQ(summary.startFrameIds, (std::vector<uint64_t>{1, 2, 3}));
    EXPECT_EQ(summary.stopFrameIds, (std::vector<uint64_t>{1, 2}));
    const std::vector<char> firstBytes = capture_bytes(first);
    ASSERT_GT(firstBytes.size(), 12u);

    // Restart immediately: each admitted start creates a distinct capture, even
    // when the wall-clock timestamp has not advanced to another second.
    ASSERT_EQ(start_directory_recording(scope.handle, directory).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    captures = directory_captures(directory);
    ASSERT_EQ(captures.size(), 2u);
    EXPECT_EQ(capture_bytes(first), firstBytes);
    for (const fs::path& path : captures)
    {
        const std::string recordingMessage = "OmniPvd: recording to '" + path.u8string() + "'";
        EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_INFO, recordingMessage.c_str()));
        EXPECT_TRUE(has_physics_commands(path));
    }
}

TEST(OmniPvdRecording, FilePathSuffixSelectsExactFileOrTimestampedDirectory)
{
    DirectoryRecordingTestScope scope;
    const fs::path exactFile = scope.root / "missing" / "parents" / "chosen.ovd";
    ASSERT_EQ(create_recording_capable_instance(scope.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(start_file_recording(scope.handle, exactFile).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(scope.root));
    ASSERT_TRUE(attach_usd_with_ovstage(scope.handle, "tests/data/simple_physics_scene.usda"));
    scope.attached = true;

    const std::string invalidPath = (scope.root / "invalid_tcp_fields").string();
    const ovphysx_omnipvd_destination_t invalid = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE, make_ovx_string(invalidPath.c_str()), OVPHYSX_LITERAL(""), 1, 0,
    };
    EXPECT_EQ(ovphysx_ovd_start_recording(scope.handle, &invalid, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_FALSE(fs::exists(scope.root));

    ASSERT_EQ(start_file_recording(scope.handle, exactFile).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(fs::is_regular_file(exactFile));
    EXPECT_EQ(directory_captures(exactFile.parent_path()), (std::vector<fs::path>{exactFile}));
    EXPECT_TRUE(has_physics_commands(exactFile));

    // Match the SDK helper's literal suffix rule: no case folding or filesystem
    // type override. A trailing separator makes even a .ovd name a directory.
    const std::array<fs::path, 3> directories = {
        scope.root / "Capture.OVD",
        scope.root / "capture.ovd.backup",
        fs::path((scope.root / "trailing.ovd").string() + "/"),
    };
    for (const fs::path& directory : directories)
    {
        SCOPED_TRACE(directory.string());
        ASSERT_EQ(start_file_recording(scope.handle, directory).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(fs::is_directory(directory));
        const std::vector<fs::path> captures = directory_captures(directory);
        ASSERT_EQ(captures.size(), 1u);
        EXPECT_TRUE(has_physics_commands(captures.front()));
        EXPECT_FALSE(fs::exists(directory / "tmp.ovd"));
    }

    const fs::path existingDirectory = scope.root / "existing.ovd";
    ASSERT_TRUE(fs::create_directory(existingDirectory));
    EXPECT_EQ(start_file_recording(scope.handle, existingDirectory).status, OVPHYSX_API_ERROR);
    EXPECT_TRUE(fs::is_empty(existingDirectory));
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(scope.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
}

TEST(OmniPvdRecording, BareExactFilenameRecordsInCurrentDirectory)
{
    DirectoryRecordingTestScope scope;
    ASSERT_EQ(create_recording_capable_instance(scope.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(scope.handle, "tests/data/simple_physics_scene.usda"));
    scope.attached = true;
    ASSERT_TRUE(fs::create_directory(scope.root));
    struct RestoreDirectory
    {
        fs::path original = fs::current_path();
        ~RestoreDirectory() { std::error_code ignored; fs::current_path(original, ignored); }
    } restoreDirectory;
    fs::current_path(scope.root);
    ASSERT_EQ(start_file_recording(scope.handle, "capture.ovd").status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    const fs::path exactFile = scope.root / "capture.ovd";
    EXPECT_EQ(directory_captures(scope.root), (std::vector<fs::path>{exactFile}));
    EXPECT_TRUE(has_physics_commands(exactFile));
}

TEST(OmniPvdRecording, Utf8ExactFileCreatesParentsAndRetainsReplacementBehavior)
{
    DirectoryRecordingTestScope scope;
    const fs::path exactFile = scope.root / fs::u8path(u8"\u00E9tage") / fs::u8path(u8"\u5DE5\u5382.ovd");
    ASSERT_EQ(create_recording_capable_instance(scope.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(scope.handle, "tests/data/simple_physics_scene.usda"));
    scope.attached = true;
    ASSERT_FALSE(fs::exists(exactFile.parent_path()));
    ASSERT_EQ(start_file_recording(scope.handle, exactFile).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(fs::is_regular_file(exactFile));
    EXPECT_EQ(directory_captures(exactFile.parent_path()), (std::vector<fs::path>{exactFile}));
    EXPECT_EQ(std::distance(fs::directory_iterator(scope.root), fs::directory_iterator{}), 1);
    // The SDK reader still accepts native narrow paths. Copy to an ASCII leaf
    // solely to decode the actual recording created at the requested UTF-8 path.
    const fs::path readerFile = scope.root / "reader.ovd";
    ASSERT_TRUE(fs::copy_file(exactFile, readerFile));
    EXPECT_TRUE(has_physics_commands(readerFile));
    {
        std::ofstream sentinel(exactFile, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(sentinel);
        sentinel << "replace this existing capture";
    }
    ASSERT_EQ(start_file_recording(scope.handle, exactFile).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(fs::copy_file(exactFile, readerFile, fs::copy_options::overwrite_existing));
    EXPECT_TRUE(has_physics_commands(readerFile));
    EXPECT_EQ(directory_captures(exactFile.parent_path()), (std::vector<fs::path>{exactFile}));
}

TEST(OmniPvdRecording, DirectoryFailureCanRetryAndActiveDestroyFinalizes)
{
    DirectoryRecordingTestScope scope;
    ASSERT_TRUE(fs::create_directories(scope.root));
    const fs::path blockedDirectory = scope.root / "blocked";
    const std::string sentinel = "preserve existing file";
    {
        std::ofstream file(blockedDirectory, std::ios::binary);
        ASSERT_TRUE(file);
        file << sentinel;
    }
    ASSERT_EQ(create_recording_capable_instance(scope.handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(scope.handle, "tests/data/simple_physics_scene.usda"));
    scope.attached = true;
    EXPECT_EQ(start_file_recording(scope.handle, blockedDirectory).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(capture_bytes(blockedDirectory), (std::vector<char>(sentinel.begin(), sentinel.end())));
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(scope.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);

    const fs::path directory = scope.root / "retry" / "nested";
    ASSERT_EQ(start_file_recording(scope.handle, directory).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(scope.handle).status, OVPHYSX_API_SUCCESS);
    std::vector<fs::path> captures = directory_captures(directory);
    ASSERT_EQ(captures.size(), 1u);
    const fs::path unsteppedCapture = captures.front();
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(unsteppedCapture, summary));
    EXPECT_GT(summary.createObjectCount, 0u);
    const std::vector<char> unsteppedBytes = capture_bytes(unsteppedCapture);
    ASSERT_GT(unsteppedBytes.size(), 12u);

    ASSERT_EQ(start_file_recording(scope.handle, directory).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(scope.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ovphysx_instance_handle_t destroyed = scope.handle;
    ASSERT_EQ(ovphysx_destroy_instance(destroyed).status, OVPHYSX_API_SUCCESS);
    scope.handle = OVPHYSX_INVALID_HANDLE;
    scope.attached = false;
    destroy_ovstage_test_stages(destroyed);
    captures = directory_captures(directory);
    ASSERT_EQ(captures.size(), 2u);
    EXPECT_EQ(capture_bytes(unsteppedCapture), unsteppedBytes);
    for (const fs::path& path : captures)
        if (path != unsteppedCapture)
        {
            EXPECT_TRUE(has_physics_commands(path, true));
        }
    EXPECT_FALSE(fs::exists(directory / "tmp.ovd"));
}

TEST(OmniPvdRecording, LateFileRecordingRejectsActiveTakeoverAndCanRestart)
{
    const fs::path directory = fs::temp_directory_path() / "ovphysx_pvd_late_file";
    const fs::path firstPath = directory / "first.ovd";
    const fs::path retargetPath = directory / "retarget.ovd";
    const fs::path destroyPath = directory / "destroy.ovd";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(handle));
    bool recording = true;
    EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_INVALID_STATE);

    const ovphysx_omnipvd_destination_t invalidDestination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE,
        OVPHYSX_LITERAL(""),
        OVPHYSX_LITERAL(""),
        0,
        0,
    };
    EXPECT_EQ(ovphysx_ovd_start_recording(handle, &invalidDestination, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);

    ASSERT_EQ(start_file_recording(handle, firstPath).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t pending = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(pending.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    ovphysx_wait_task_result_t waited{};
    EXPECT_EQ(ovphysx_wait_task(handle, pending, OVPHYSX_TIMEOUT_INFINITE, &waited).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(waited.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&waited);

    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(start_file_recording(handle, retargetPath).status, OVPHYSX_API_INVALID_STATE);
    ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("already active"), std::string::npos);
    EXPECT_FALSE(fs::exists(retargetPath));

    ovphysx_instance_handle_t peerHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(start_file_recording(peerHandle, retargetPath).status, OVPHYSX_API_INVALID_STATE);
    error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("already active"), std::string::npos);
    bool peerRecording = true;
    EXPECT_EQ(ovphysx_ovd_is_recording(peerHandle, &peerRecording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(peerRecording);
    EXPECT_EQ(ovphysx_ovd_stop_recording(peerHandle).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(ovphysx_destroy_instance(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    EXPECT_FALSE(fs::exists(retargetPath));

    ASSERT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    ASSERT_TRUE(fs::exists(firstPath));
    EXPECT_TRUE(has_physics_commands(firstPath));
    EXPECT_EQ(ovphysx_ovd_start_recording(handle, &invalidDestination, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    peerHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&peerHandle).status, OVPHYSX_API_SUCCESS);

    const ovphysx_enqueue_result_t startStep = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(startStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(start_file_recording(peerHandle, retargetPath).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle, startStep, 0));
    peerRecording = false;
    ASSERT_EQ(ovphysx_ovd_is_recording(peerHandle, &peerRecording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(peerRecording);
    recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);

    const ovphysx_enqueue_result_t stopStep = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(stopStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle, stopStep, 0));
    ASSERT_TRUE(fs::exists(retargetPath));
    EXPECT_TRUE(has_physics_commands(retargetPath));

    ASSERT_EQ(start_file_recording(peerHandle, destroyPath).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t destroyStep = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(destroyStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle, destroyStep, 0));
    ASSERT_TRUE(fs::exists(destroyPath));
    // The peer owns the stream, not the live stage. Destroy finalizes the
    // recording without tearing down the stage owner's PhysX objects.
    EXPECT_TRUE(has_physics_commands(destroyPath));

    destroy_ovstage_test_attachments(handle);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    fs::remove_all(directory, ec);
}


TEST(OmniPvdRecording, OwnerLifecycleRejectsRuntimeCallbackReentry)
{
    DirectoryRecordingTestScope owner;
    DirectoryRecordingTestScope peer;
    ASSERT_TRUE(create_stepped_recording_instance(owner.handle));
    owner.attached = true;
    ASSERT_TRUE(create_stepped_recording_instance(peer.handle));
    peer.attached = true;
    const fs::path path = owner.root / "runtime_callback.ovd";
    ASSERT_EQ(start_file_recording(owner.handle, path).status, OVPHYSX_API_SUCCESS);

    struct Probe
    {
        ovphysx_instance_handle_t owner;
        unsigned calls = 0;
        ovphysx_api_status_t destroyStatus = OVPHYSX_API_ERROR;
        ovphysx_enqueue_result_t detachReceipt{};
        std::chrono::steady_clock::duration elapsed{};
        static void callback(void* user)
        {
            Probe& probe = *static_cast<Probe*>(user);
            ++probe.calls;
            const auto started = std::chrono::steady_clock::now();
            // The peer detach invokes a real runtime callback while holding
            // transition admission; neither owner call may recursively wait.
            probe.destroyStatus = ovphysx_destroy_instance(probe.owner).status;
            probe.detachReceipt = ovphysx_detach_ovstage(probe.owner);
            probe.elapsed = std::chrono::steady_clock::now() - started;
        }
    } probe{owner.handle};
    struct Subscription
    {
        ovphysx_instance_handle_t peer;
        ovphysx_subscription_id_t id = 0;
        ~Subscription() { if (id) ovphysx_unsubscribe_object_changes(peer, id); }
    } subscription{peer.handle};
    ovphysx_object_change_callbacks_t callbacks{};
    callbacks.on_all_objects_destroyed = Probe::callback;
    callbacks.user_data = &probe;
    ASSERT_EQ(ovphysx_subscribe_object_changes(peer.handle, &callbacks, &subscription.id).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(destroy_ovstage_test_attachments(peer.handle));
    peer.attached = false;

    EXPECT_EQ(probe.calls, 1u);
    EXPECT_EQ(probe.destroyStatus, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.detachReceipt.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.detachReceipt.op_index, 0u);
    EXPECT_EQ(probe.detachReceipt.instance, 0u);
    EXPECT_LT(probe.elapsed, std::chrono::seconds(5));
    bool recording = false;
    ASSERT_EQ(ovphysx_ovd_is_recording(owner.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    ASSERT_EQ(ovphysx_step_sync(owner.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(owner.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(has_physics_commands(path));
}

TEST(OmniPvdRecording, AttachedIdleLifecycleRejectsLogCallbackBeforeTeardown)
{
    DirectoryRecordingTestScope idle;
    ASSERT_TRUE(create_stepped_recording_instance(idle.handle));
    idle.attached = true;
    struct Probe
    {
        ovphysx_instance_handle_t handle;
        std::atomic<bool> called{false};
        std::promise<void> finished;
        ovphysx_api_status_t destroyStatus = OVPHYSX_API_ERROR;
        ovphysx_enqueue_result_t detachReceipt{};
        static void callback(ovphysx_log_level_t level, ovphysx_string_t message, ovphysx_string_t, double, void* user)
        {
            Probe& probe = *static_cast<Probe*>(user);
            if (level != OVPHYSX_LOG_WARNING || !message.ptr ||
                std::string(message.ptr, message.length) != "[LogTest] WARNING test message" || probe.called.exchange(true))
                return;
            probe.detachReceipt = ovphysx_detach_ovstage(probe.handle);
            probe.destroyStatus = ovphysx_destroy_instance(probe.handle).status;
            probe.finished.set_value();
        }
    } probe{idle.handle};
    std::future<void> finished = probe.finished.get_future();
    struct RestoreLogging
    {
        uint32_t level = ovphysx_get_log_level();
        ~RestoreLogging()
        {
            ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
            ovphysx_set_log_level(level);
        }
    } restoreLogging;
    ovphysx_set_log_level(OVPHYSX_LOG_WARNING);
    ASSERT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_WARNING, nullptr, Probe::callback, &probe).status,
              OVPHYSX_API_SUCCESS);
    ovphysx_log_emit_test_messages();
    ASSERT_EQ(finished.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_EQ(ovphysx_flush_log(UINT64_MAX).status, OVPHYSX_API_SUCCESS);
    if (probe.destroyStatus == OVPHYSX_API_SUCCESS)
    {
        idle.handle = OVPHYSX_INVALID_HANDLE;
        idle.attached = false;
    }
    EXPECT_EQ(probe.detachReceipt.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.detachReceipt.op_index, 0u);
    EXPECT_EQ(probe.detachReceipt.instance, 0u);
    ASSERT_EQ(probe.destroyStatus, OVPHYSX_API_INVALID_STATE);
    bool attached = false;
    ASSERT_EQ(ovphysx_is_stage_attached(idle.handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attached);
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(idle.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_EQ(ovphysx_step_sync(idle.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
}


TEST(OmniPvdRecording, RecordingCallsQueueBehindSteppingPeer)
{
    DirectoryRecordingTestScope owner;
    DirectoryRecordingTestScope peer;
    ASSERT_TRUE(create_stepped_recording_instance(owner.handle));
    owner.attached = true;
    ASSERT_TRUE(create_stepped_recording_instance(peer.handle));
    peer.attached = true;
    const fs::path path = owner.root / "queued.ovd";
    ASSERT_EQ(start_file_recording(owner.handle, path).status, OVPHYSX_API_SUCCESS);

    std::atomic<bool> stop{ false };
    std::atomic<unsigned> peerSteps{ 0 };
    std::atomic<unsigned> peerFailures{ 0 };
    std::thread stepper([&] {
        while (!stop.load())
        {
            if (ovphysx_step_sync(peer.handle, 1.0f / 60.0f).status == OVPHYSX_API_SUCCESS)
                ++peerSteps;
            else
                ++peerFailures;
        }
    });
    const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (peerSteps.load() == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    const unsigned stepsBefore = peerSteps.load();
    unsigned failed = 0;
    for (int query = 0; query < 200; ++query)
    {
        bool recording = false;
        if (ovphysx_ovd_is_recording(owner.handle, &recording).status != OVPHYSX_API_SUCCESS || !recording)
            ++failed;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const unsigned stepsDuringQueries = peerSteps.load() - stepsBefore;
    const ovphysx_api_status_t stopStatus = ovphysx_ovd_stop_recording(owner.handle).status;
    stop = true;
    stepper.join();
    EXPECT_GE(stepsDuringQueries, 20u); // The peer really stepped during the queries.
    EXPECT_EQ(peerFailures.load(), 0u);
    EXPECT_EQ(failed, 0u);
    EXPECT_EQ(stopStatus, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(has_physics_commands(path));
}

TEST(OmniPvdRecording, OwnerDetachDrainsPeerAndPreservesTaskResult)
{
    DirectoryRecordingTestScope owner;
    DirectoryRecordingTestScope peer;
    ASSERT_TRUE(fs::create_directories(owner.root));
    const fs::path beforeDetachPath = owner.root / "before_detach.ovd";
    const fs::path peerCapturePath = owner.root / "peer_capture.ovd";

    ASSERT_TRUE(create_stepped_recording_instance(owner.handle));
    owner.attached = true;
    ASSERT_TRUE(create_stepped_recording_instance(peer.handle));
    peer.attached = true;
    ASSERT_EQ(start_file_recording(owner.handle, beforeDetachPath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(owner.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    // Leave the peer step unfetched: the runtime cannot stop recording until
    // detach finishes that work without collecting its task result.
    const ovphysx_enqueue_result_t peerStep = ovphysx_step(peer.handle, 1.0f / 60.0f);
    ASSERT_EQ(peerStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(destroy_ovstage_test_attachments(owner.handle));
    owner.attached = false;

    // Read before status queries or public waits can establish another safe point.
    EXPECT_TRUE(has_physics_commands(beforeDetachPath));
    OvdCommandSummary summary;
    const ::testing::AssertionResult parsed = read_ovd_commands(beforeDetachPath, summary);
    EXPECT_TRUE(parsed);
    if (parsed)
    {
        // Each scene closes frame 1 during simulate and frame 2 during fetch.
        // The owner's completed step alone cannot prove the peer was fetched.
        std::set<OmniPvdContextHandle> completedSceneContexts;
        for (const auto& [context, frameId] : summary.stoppedFrames)
        {
            if (frameId == 2)
                completedSceneContexts.insert(context);
        }
        EXPECT_EQ(completedSceneContexts.size(), 2u);
        EXPECT_GT(summary.destroyObjectCount, 0u);
    }

    ovphysx_wait_task_result_t waited{};
    EXPECT_EQ(ovphysx_wait_task(peer.handle, peerStep, OVPHYSX_TIMEOUT_POLL, &waited).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(waited.num_errors, 0u);
    ovphysx_destroy_wait_task_result(&waited);
    EXPECT_EQ(ovphysx_wait_task(peer.handle, peerStep, OVPHYSX_TIMEOUT_POLL, &waited).status,
              OVPHYSX_API_NOT_FOUND);
    ovphysx_destroy_wait_task_result(&waited);

    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(owner.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);

    ASSERT_EQ(start_file_recording(peer.handle, peerCapturePath).status, OVPHYSX_API_SUCCESS);
    recording = false;
    ASSERT_EQ(ovphysx_ovd_is_recording(peer.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    EXPECT_EQ(ovphysx_ovd_stop_recording(owner.handle).status, OVPHYSX_API_INVALID_STATE);
    ASSERT_EQ(ovphysx_step_sync(peer.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(peer.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(has_physics_commands(peerCapturePath));
}

static void check_non_owner_detach_preserves_recording(bool leaveOwnerStepPending)
{
    DirectoryRecordingTestScope owner;
    DirectoryRecordingTestScope peer;
    ASSERT_TRUE(fs::create_directories(owner.root));
    const fs::path capturePath = owner.root / "preserved.ovd";
    ASSERT_TRUE(create_stepped_recording_instance(owner.handle));
    owner.attached = true;
    ASSERT_TRUE(create_stepped_recording_instance(peer.handle));
    peer.attached = true;
    ASSERT_EQ(start_file_recording(owner.handle, capturePath).status, OVPHYSX_API_SUCCESS);

    const ovphysx_enqueue_result_t ownerStep = ovphysx_step(owner.handle, 1.0f / 60.0f);
    ASSERT_EQ(ownerStep.status, OVPHYSX_API_SUCCESS);
    // Check the idle case too: an unfetched owner step makes runtime recording
    // queries busy and can mask an incorrect attempt to finalize the owner.
    if (!leaveOwnerStepPending)
    {
        ASSERT_TRUE(waitForTaskSuccess(owner.handle, ownerStep, OVPHYSX_TIMEOUT_INFINITE));
    }
    ASSERT_TRUE(destroy_ovstage_test_attachments(peer.handle));
    peer.attached = false;
    if (leaveOwnerStepPending)
    {
        EXPECT_TRUE(waitForTaskSuccess(owner.handle, ownerStep, OVPHYSX_TIMEOUT_INFINITE));
    }

    bool recording = false;
    ASSERT_EQ(ovphysx_ovd_is_recording(owner.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(peer.handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_EQ(ovphysx_ovd_stop_recording(peer.handle).status, OVPHYSX_API_INVALID_STATE);
    ASSERT_EQ(ovphysx_step_sync(owner.handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(owner.handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(has_physics_commands(capturePath));
}

TEST(OmniPvdRecording, NonOwnerDetachPreservesIdleOwnerRecording)
{
    check_non_owner_detach_preserves_recording(false);
}

TEST(OmniPvdRecording, NonOwnerDetachPreservesPendingOwnerRecording)
{
    check_non_owner_detach_preserves_recording(true);
}

TEST(OmniPvdRecording, ActiveDetachFinalizesAndReattachCanRecord)
{
    const fs::path directory = fs::temp_directory_path() / "ovphysx_pvd_late_detach";
    const fs::path beforeDetachPath = directory / "before_detach.ovd";
    const fs::path whileDetachedPath = directory / "while_detached.ovd";
    const fs::path afterReattachPath = directory / "after_reattach.ovd";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(create_recording_capable_instance(handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(start_file_recording(handle, beforeDetachPath).status, OVPHYSX_API_INVALID_STATE);
    ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("attach and initialize"), std::string::npos);
    EXPECT_FALSE(fs::exists(beforeDetachPath));

    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ovphysx_instance_handle_t peerHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&peerHandle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(start_file_recording(peerHandle, beforeDetachPath).status, OVPHYSX_API_SUCCESS);
    // A recording owner without an attachment has nothing to detach. Its no-op
    // receipt must leave the shared stream active on the remaining live stage.
    ASSERT_TRUE(waitForTaskSuccess(peerHandle, ovphysx_detach_ovstage(peerHandle)));
    bool peerRecording = false;
    ASSERT_EQ(ovphysx_ovd_is_recording(peerHandle, &peerRecording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(peerRecording);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(fs::exists(beforeDetachPath));
    EXPECT_TRUE(has_physics_commands(beforeDetachPath, true));

    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(peerHandle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    ASSERT_EQ(start_file_recording(handle, whileDetachedPath).status, OVPHYSX_API_INVALID_STATE);
    error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("attach and initialize"), std::string::npos);
    EXPECT_FALSE(fs::exists(whileDetachedPath));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_FALSE(fs::exists(afterReattachPath));
    const ovphysx_api_status_t restartStatus = start_file_recording(handle, afterReattachPath).status;
    EXPECT_EQ(restartStatus, OVPHYSX_API_SUCCESS);
    if (restartStatus == OVPHYSX_API_SUCCESS)
    {
        EXPECT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        recording = true;
        EXPECT_EQ(ovphysx_ovd_is_recording(peerHandle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(recording);
        EXPECT_EQ(ovphysx_ovd_stop_recording(peerHandle).status, OVPHYSX_API_INVALID_STATE);
        EXPECT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(fs::exists(afterReattachPath));
        EXPECT_TRUE(has_physics_commands(afterReattachPath));
    }

    EXPECT_EQ(ovphysx_destroy_instance(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(destroy_ovstage_test_attachments(handle));
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    fs::remove_all(directory, ec);
}

TEST(OmniPvdRecording, FailedLateFileOpenCanRetryAndActiveDestroyFinalizes)
{
    const fs::path directory = fs::temp_directory_path() / "ovphysx_pvd_late_retry";
    const fs::path capturePath = directory / "retry.ovd";
    const fs::path nextCapturePath = directory / "next_instance.ovd";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);
    const fs::path exactDirectory = directory / "obstacle.ovd";
    ASSERT_TRUE(fs::create_directory(exactDirectory));

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(handle));
    EXPECT_EQ(start_file_recording(handle, exactDirectory).status, OVPHYSX_API_ERROR);
    bool recording = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);

    const ::testing::AssertionResult refusedTcp = with_refused_loopback_tcp(
        [handle](uint16_t port) {
            const ovphysx_omnipvd_destination_t tcpDestination = {
                OVPHYSX_OMNIPVD_TRANSPORT_TCP,
                OVPHYSX_LITERAL(""),
                OVPHYSX_LITERAL("127.0.0.1"),
                port,
                1000,
            };
            EXPECT_EQ(ovphysx_ovd_start_recording(handle, &tcpDestination, nullptr).status, OVPHYSX_API_ERROR);
            bool tcpRecording = true;
            EXPECT_EQ(ovphysx_ovd_is_recording(handle, &tcpRecording).status, OVPHYSX_API_SUCCESS);
            EXPECT_FALSE(tcpRecording);
        });
    ASSERT_TRUE(refusedTcp) << refusedTcp.message();

    ASSERT_EQ(start_file_recording(handle, capturePath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    destroy_ovstage_test_stages(handle);
    ASSERT_TRUE(fs::exists(capturePath));
    EXPECT_TRUE(has_physics_commands(capturePath, true));

    ovphysx_instance_handle_t nextHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(nextHandle));
    ASSERT_EQ(start_file_recording(nextHandle, nextCapturePath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(nextHandle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(nextHandle).status, OVPHYSX_API_SUCCESS);
    destroy_ovstage_test_attachments(nextHandle);
    ASSERT_EQ(ovphysx_destroy_instance(nextHandle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(fs::exists(nextCapturePath));
    EXPECT_TRUE(has_physics_commands(nextCapturePath));
    fs::remove_all(directory, ec);
}

TEST(OmniPvdRecording, ExplicitTcpCanStopAndRestartToFile)
{
    const fs::path directory = fs::temp_directory_path() / "ovphysx_pvd_explicit_tcp_file";
    const fs::path tcpCapturePath = directory / "explicit_tcp.ovd";
    const fs::path capturePath = directory / "after_explicit_tcp.ovd";
    const fs::path restartedCapturePath = directory / "after_late_file_restart.ovd";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);
    // Normalize the source Vehicle fixture's centimetres to the bootstrap's
    // metre scale so this test covers transport transitions only.
    const fs::path vehiclePath = directory / "vehicle_meters.usda";
    std::ifstream vehicleSource(
        OVPHYSX_SOURCE_DIR "/ovruntime/data/usd/tests/Physics/Vehicle_Schema_Tests/Vehicle.usda");
    std::ofstream vehicleFixture(vehiclePath);
    ASSERT_TRUE(vehicleSource && vehicleFixture);
    bool normalizedUnits = false;
    bool insertedFrictionMaterials = false;
    bool expandedFrictionValues = false;
    bool expandedGroundMaterials = false;
    bool assignedOriginalVehicleScene = false;
    for (std::string line; std::getline(vehicleSource, line);)
    {
        const size_t unitPosition = line.find("metersPerUnit = 0.01");
        if (unitPosition != std::string::npos)
        {
            line.replace(unitPosition, std::strlen("metersPerUnit = 0.01"), "metersPerUnit = 1");
            normalizedUnits = true;
        }
        if (line == "def PhysxVehicleTireFrictionTable \"WinterTire\"")
        {
            vehicleFixture << R"usd(def Material "OmniPvdFrictionMaterial1" (
    prepend apiSchemas = ["PhysicsMaterialAPI"]
)
{
}

def Material "OmniPvdFrictionMaterial2" (
    prepend apiSchemas = ["PhysicsMaterialAPI"]
)
{
}

def Material "OmniPvdFrictionMaterial3" (
    prepend apiSchemas = ["PhysicsMaterialAPI"]
)
{
}

)usd";
            insertedFrictionMaterials = true;
        }
        if (line.find("float[] frictionValues = [0.75, 0.6]") != std::string::npos)
        {
            line = "    float[] frictionValues = [0.75, 0.6, 0.55, 0.5, 0.45]";
            expandedFrictionValues = true;
        }
        vehicleFixture << line << '\n';
        if (line == "    vector3f physics:angularVelocity = (0, 0, 0)")
        {
            vehicleFixture << "    rel physics:simulationOwner = </physicsScene>\n";
            assignedOriginalVehicleScene = true;
        }
        if (expandedFrictionValues && !expandedGroundMaterials &&
            line.find("</GravelMaterial>,") != std::string::npos)
        {
            vehicleFixture << "        </OmniPvdFrictionMaterial1>,\n"
                              "        </OmniPvdFrictionMaterial2>,\n"
                              "        </OmniPvdFrictionMaterial3>,\n";
            expandedGroundMaterials = true;
        }
    }
    ASSERT_TRUE(normalizedUnits && insertedFrictionMaterials && expandedFrictionValues && expandedGroundMaterials &&
                assignedOriginalVehicleScene && vehicleSource.eof() && vehicleFixture.good());
    vehicleFixture << R"usd(

def PhysicsRevoluteJoint "OmniPvdExtensionMarker"
{
    uniform token physics:axis = "X"
    rel physics:body0 = </Vehicle>
    point3f physics:localPos0 = (0, 0, 0)
    point3f physics:localPos1 = (0, 0, 0)
    quatf physics:localRot0 = (1, 0, 0, 0)
    quatf physics:localRot1 = (1, 0, 0, 0)
}

def PhysicsScene "physicsScene2" (
    prepend apiSchemas = ["PhysxVehicleContextAPI"]
)
{
    vector3f physics:gravityDirection = (0, -1, 0)
    float physics:gravityMagnitude = 1000
    uniform token physxVehicleContext:longitudinalAxis = "posZ"
    uniform token physxVehicleContext:verticalAxis = "posY"
    uniform token physxVehicleContext:updateMode = "velocityChange"
}

def Xform "Vehicle2" (
    prepend references = </Vehicle>
)
{
    rel physics:simulationOwner = </physicsScene2>
    float3 xformOp:translate = (500, 500, 0)
}

)usd";
    ASSERT_TRUE(vehicleFixture.good());
    vehicleFixture.close();
    const std::string vehiclePathString = vehiclePath.string();

    TcpCaptureResult captureResult;
    const ::testing::AssertionResult captured = capture_loopback_tcp(
        [&capturePath, &restartedCapturePath, &vehiclePathString](uint16_t port) -> ovphysx_api_status_t {
            ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
            const ovphysx_result_t created = create_recording_capable_instance(handle);
            if (created.status != OVPHYSX_API_SUCCESS)
                return created.status;

            if (!attach_usd_with_ovstage(handle, vehiclePathString.c_str()))
            {
                ovphysx_destroy_instance(handle);
                return OVPHYSX_API_ERROR;
            }
            bool dormant = true;
            EXPECT_EQ(ovphysx_ovd_is_recording(handle, &dormant).status, OVPHYSX_API_SUCCESS);
            EXPECT_FALSE(dormant);
            const ovphysx_omnipvd_destination_t destination = {
                OVPHYSX_OMNIPVD_TRANSPORT_TCP, OVPHYSX_LITERAL(""),
                OVPHYSX_LITERAL("127.0.0.1"), port, 3000,
            };
            ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination, nullptr).status;
            if (status == OVPHYSX_API_SUCCESS)
                status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
            void* extensionsFixture = nullptr;
            if (status == OVPHYSX_API_SUCCESS &&
                !ovphysx_create_extensions_fixture_for_test_internal(handle, &extensionsFixture))
            {
                status = OVPHYSX_API_ERROR;
            }
            bool recording = false;
            if (status == OVPHYSX_API_SUCCESS)
                status = ovphysx_ovd_is_recording(handle, &recording).status;
            EXPECT_TRUE(recording);
            if (status == OVPHYSX_API_SUCCESS && !recording)
                status = OVPHYSX_API_INVALID_STATE;

            const ovphysx_api_status_t stopStatus = ovphysx_ovd_stop_recording(handle).status;
            EXPECT_EQ(stopStatus, OVPHYSX_API_SUCCESS);
            if (status == OVPHYSX_API_SUCCESS)
                status = stopStatus;

            const auto recordToFile = [handle](const fs::path& path, uint32_t stepCount) -> ovphysx_api_status_t {
                ovphysx_api_status_t fileStatus = start_file_recording(handle, path).status;
                for (uint32_t i = 0; fileStatus == OVPHYSX_API_SUCCESS && i < stepCount; ++i)
                    fileStatus = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
                if (fileStatus == OVPHYSX_API_SUCCESS)
                    fileStatus = ovphysx_ovd_stop_recording(handle).status;
                return fileStatus;
            };
            if (status == OVPHYSX_API_SUCCESS)
                status = recordToFile(capturePath, 32);
            if (status == OVPHYSX_API_SUCCESS)
                status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
            if (status == OVPHYSX_API_SUCCESS)
                status = recordToFile(restartedCapturePath, 0);
            ovphysx_destroy_extensions_fixture_for_test_internal(extensionsFixture);
            destroy_ovstage_test_attachments(handle);
            const ovphysx_api_status_t destroyStatus = ovphysx_destroy_instance(handle).status;
            return status == OVPHYSX_API_SUCCESS ? destroyStatus : status;
        },
        captureResult);
    ASSERT_TRUE(captured) << captured.message();
    EXPECT_TRUE(captureResult.cleanEof) << "Expected the producer to close the TCP stream cleanly";
    {
        constexpr size_t kSocketHandshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
        constexpr uint32_t kSocketHandshakeMagic = 0x4F56444Cu;
        ASSERT_GE(captureResult.bytes.size(), kSocketHandshakeSize);
        uint32_t handshakeMagic = 0;
        std::memcpy(&handshakeMagic, captureResult.bytes.data(), sizeof(handshakeMagic));
        ASSERT_EQ(handshakeMagic, kSocketHandshakeMagic);
        std::ofstream tcpCapture(tcpCapturePath, std::ios::binary);
        ASSERT_TRUE(tcpCapture);
        tcpCapture.write(
            reinterpret_cast<const char*>(captureResult.bytes.data() + kSocketHandshakeSize),
            static_cast<std::streamsize>(captureResult.bytes.size() - kSocketHandshakeSize));
    }
    EXPECT_TRUE(has_physics_commands(tcpCapturePath, false, true, true, true, 40));
    ASSERT_TRUE(fs::exists(capturePath));
    EXPECT_TRUE(has_physics_commands(capturePath, false, true, true, true, 40));
    OvdCommandSummary captureSummary;
    ASSERT_TRUE(read_ovd_commands(capturePath, captureSummary));
    EXPECT_EQ(captureSummary.vehicleObjectCount, 2u);
    EXPECT_GE(captureSummary.stopFrameCount, 32u);
    EXPECT_GT(captureSummary.destroyObjectCount, 0u);
    EXPECT_EQ(captureSummary.lastCommand, OmniPvdCommand::eDESTROY_OBJECT);
    ASSERT_TRUE(fs::exists(restartedCapturePath));
    OvdCommandSummary restartedSummary;
    ASSERT_TRUE(read_ovd_commands(restartedCapturePath, restartedSummary));
    EXPECT_GT(restartedSummary.createObjectCount, 0u);
    EXPECT_TRUE(restartedSummary.hasVehicleSchema);
    EXPECT_TRUE(restartedSummary.hasVehicleObject);
    EXPECT_EQ(restartedSummary.vehicleObjectCount, 2u);
    EXPECT_GT(restartedSummary.destroyObjectCount, 0u);
    EXPECT_EQ(restartedSummary.physXMaterialFrictionObjectCount, 40u);
    EXPECT_EQ(restartedSummary.physXExtensionsSchemaMask, kCompletePhysXExtensionsSchemaMask);
    EXPECT_EQ(restartedSummary.physXExtensionsObjectMask, kCompletePhysXExtensionsObjectMask);
    fs::remove_all(directory, ec);
}

// Both spellings must be real linkable exports throughout the compatibility window.
// @implements REQ-CAPI-OVD-NAMES-001
// @covers AC-1 AC-2
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4996)
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
TEST(OmniPvdRecording, PublicExportCompatibility)
{
    const ovphysx_instance_handle_t instance = OVPHYSX_INVALID_HANDLE;
    const ovphysx_handle_t legacyInstance = instance;
    EXPECT_EQ(sizeof(legacyInstance), sizeof(uint64_t));
    EXPECT_EQ(ovphysx_ovd_start_recording(instance, nullptr, nullptr).status,
              ovphysx_start_recording(legacyInstance, nullptr).status);
    EXPECT_EQ(ovphysx_ovd_stop_recording(instance).status,
              ovphysx_stop_recording(legacyInstance).status);
    bool recording = true;
    bool legacyRecording = true;
    EXPECT_EQ(ovphysx_ovd_is_recording(instance, &recording).status,
              ovphysx_is_recording(legacyInstance, &legacyRecording).status);
    EXPECT_FALSE(recording);
    EXPECT_FALSE(legacyRecording);
    EXPECT_EQ(ovphysx_ovd_is_recording(instance, nullptr).status,
              ovphysx_is_recording(legacyInstance, nullptr).status);
}
// This suite's canonical lifecycle pass owns initialization in a fresh process.
// @implements REQ-CAPI-OMNIPVD-LATE-001
// @covers AC-1 AC-6 AC-11
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-4
TEST(OmniPvdGlobalConfig, ColdAndRetiredIngressRejectsInvalidHandles)
{
    const fs::path path = fs::temp_directory_path() / "ovphysx_cold_ingress_must_not_record.ovd";
    std::error_code ec;
    fs::remove(path, ec);
    const std::string pathString = path.string();
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE, make_ovx_string(pathString.c_str()), OVPHYSX_LITERAL(""), 0, 0,
    };
    const ovphysx_omnipvd_destination_t tcpDestination = {
        OVPHYSX_OMNIPVD_TRANSPORT_TCP, OVPHYSX_LITERAL(""), OVPHYSX_LITERAL("127.0.0.1"), 1, 0,
    };
    ovphysx_omnipvd_recording_options_t filter = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    ovphysx_object_change_callbacks_t callbacks{};
    callbacks.on_all_objects_destroyed = [](void*) {};
    struct RuntimeCleanup
    {
        bool initialized = false;
        ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
        ~RuntimeCleanup()
        {
            if (handle != OVPHYSX_INVALID_HANDLE)
                ovphysx_destroy_instance(handle);
            if (initialized)
                ovphysx_shutdown();
        }
    } cleanup;
    const auto probe = [&](ovphysx_instance_handle_t handle)
    {
        SCOPED_TRACE(handle);
        EXPECT_EQ(ovphysx_ovd_start_recording(handle, &destination, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(ovphysx_ovd_start_recording(handle, &destination, &filter).status,
                  OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(start_discovery(handle, &tcpDestination, &filter).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(start_discovery(handle, nullptr, &filter).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(ovphysx_start_recording(handle, &destination).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(ovphysx_stop_recording(handle).status, OVPHYSX_API_INVALID_ARGUMENT);
        bool recording = true;
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_FALSE(recording);
        recording = true;
        EXPECT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_FALSE(recording);
        ovphysx_subscription_id_t subscription = 0;
        EXPECT_EQ(ovphysx_subscribe_object_changes(handle, &callbacks, &subscription).status,
                  OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(subscription, OVPHYSX_INVALID_SUBSCRIPTION_ID);
        EXPECT_EQ(ovphysx_ovd_start_recording(handle, nullptr, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(ovphysx_subscribe_object_changes(handle, &callbacks, nullptr).status,
                  OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_FALSE(fs::exists(path));
    };
    const ovphysx_instance_handle_t fake = static_cast<ovphysx_instance_handle_t>(UINT64_MAX);
    probe(OVPHYSX_INVALID_HANDLE);
    probe(fake);
    ASSERT_EQ(ovphysx_initialize().status, OVPHYSX_API_SUCCESS);
    cleanup.initialized = true;
    probe(OVPHYSX_INVALID_HANDLE);
    probe(fake);
    ovphysx_instance_handle_t retired = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&cleanup.handle).status, OVPHYSX_API_SUCCESS);
    retired = cleanup.handle;
    ASSERT_NE(retired, OVPHYSX_INVALID_HANDLE);
    ASSERT_EQ(ovphysx_destroy_instance(retired).status, OVPHYSX_API_SUCCESS);
    cleanup.handle = OVPHYSX_INVALID_HANDLE;
    probe(retired); // No live handle, but the direct runtime still exists.
    ASSERT_EQ(ovphysx_shutdown().status, OVPHYSX_API_SUCCESS);
    cleanup.initialized = false;
    probe(OVPHYSX_INVALID_HANDLE);
    probe(fake);
    probe(retired); // Slot unpublished and direct runtime fully torn down.
}

// Exercise both export families before any simulation step. Unlike invalid-handle
// parity alone, the state transitions detect a forwarding alias wired to the wrong call.
TEST(OmniPvdRecording, PublicExportsCaptureFirstSimulationFrame)
{
    for (bool legacy : { false, true })
    {
        SCOPED_TRACE(legacy ? "legacy exports" : "canonical exports");
        const fs::path path = fs::temp_directory_path() /
            (legacy ? "ovphysx_legacy_first_frame.ovd" : "ovphysx_canonical_first_frame.ovd");
        std::error_code ec;
        fs::remove(path, ec);

        ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
        ASSERT_EQ(create_recording_capable_instance(handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
        const std::string pathString = path.u8string();
        const ovphysx_omnipvd_destination_t destination = {
            OVPHYSX_OMNIPVD_TRANSPORT_FILE, make_ovx_string(pathString.c_str()), OVPHYSX_LITERAL(""), 0, 0,
        };
        ASSERT_EQ((legacy ? ovphysx_start_recording(handle, &destination) :
                            ovphysx_ovd_start_recording(handle, &destination, nullptr)).status,
                  OVPHYSX_API_SUCCESS);
        bool recording = false;
        bool legacyRecording = false;
        ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_is_recording(handle, &legacyRecording).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(recording);
        EXPECT_EQ(legacyRecording, recording);

        ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ((legacy ? ovphysx_stop_recording(handle) : ovphysx_ovd_stop_recording(handle)).status,
                  OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_is_recording(handle, &legacyRecording).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(recording);
        EXPECT_EQ(legacyRecording, recording);

        // Parse after stop, before detach/destroy: stop itself must finalize the file.
        OvdCommandSummary summary;
        ASSERT_TRUE(read_ovd_commands(path, summary));
        EXPECT_GT(summary.createObjectCount, 0u);
        // PhysX records pre-sim (odd) and post-sim (even) frames per step.
        // The snapshot opens frame 1; one step completes 1 and 2 and opens 3.
        EXPECT_EQ(summary.startFrameIds, (std::vector<uint64_t>{ 1, 2, 3 }));
        EXPECT_EQ(summary.stopFrameIds, (std::vector<uint64_t>{ 1, 2 }));
        EXPECT_TRUE(destroy_ovstage_test_attachments(handle));
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
        fs::remove(path, ec);
    }
}

class OmniPvdSelectiveCapture : public ::testing::Test
{
protected:
    ovphysx_instance_handle_t handle{ OVPHYSX_INVALID_HANDLE };
    ovphysx_instance_handle_t peer{ OVPHYSX_INVALID_HANDLE };
    std::vector<fs::path> files;

    void SetUp() override
    {
        ASSERT_EQ(create_recording_capable_instance(handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    }

    void TearDown() override
    {
        for (ovphysx_instance_handle_t instance : { peer, handle })
        {
            if (instance == OVPHYSX_INVALID_HANDLE)
                continue;
            bool active = false;
            if (ovphysx_ovd_is_recording(instance, &active).status == OVPHYSX_API_SUCCESS && active)
            {
                EXPECT_EQ(ovphysx_ovd_stop_recording(instance).status, OVPHYSX_API_SUCCESS);
            }
            EXPECT_TRUE(destroy_ovstage_test_attachments(instance));
            EXPECT_EQ(ovphysx_destroy_instance(instance).status, OVPHYSX_API_SUCCESS);
        }
        std::error_code error;
        for (const fs::path& file : files)
            fs::remove(file, error);
    }

    fs::path path(const char* suffix)
    {
        std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
        std::replace(testName.begin(), testName.end(), '/', '_');
        const fs::path file = fs::temp_directory_path() /
            (std::string("ovphysx_capture_") + testName + "_" + suffix + ".ovd");
        std::error_code error;
        fs::remove(file, error);
        files.push_back(file);
        return file;
    }

    ovphysx_result_t start(ovphysx_instance_handle_t owner, const fs::path& file,
                          const ovphysx_omnipvd_recording_options_t* capture)
    {
        const std::string fileString = file.u8string();
        const ovphysx_omnipvd_destination_t destination = {
            OVPHYSX_OMNIPVD_TRANSPORT_FILE, make_ovx_string(fileString.c_str()), OVPHYSX_LITERAL(""), 0, 0
        };
        return ovphysx_ovd_start_recording(owner, &destination, capture);
    }

    OmniPvdObjectHandle object(ovphysx_instance_handle_t owner, ovphysx_query_target_t type,
                               const char* primPath)
    {
        return reinterpret_cast<OmniPvdObjectHandle>(read_physx_pointer(owner, type, primPath));
    }

    void finish(ovphysx_instance_handle_t owner)
    {
        ASSERT_EQ(ovphysx_step_sync(owner, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_ovd_stop_recording(owner).status, OVPHYSX_API_SUCCESS);
    }
};

class OmniPvdSelectiveCaptureGpuTest : public OmniPvdSelectiveCapture, public ::testing::WithParamInterface<bool>
{
protected:
    bool previousReadback{ false };
    bool restoreReadback{ false };

    void SetUp() override
    {
        ASSERT_EQ(create_recording_capable_instance(handle).status, OVPHYSX_API_SUCCESS);
        bool cpuOnly = false;
        ASSERT_EQ(ovphysx_get_cpu_mode(&cpuOnly).status, OVPHYSX_API_SUCCESS);
        if (cpuOnly)
            GTEST_SKIP() << "Particle and deformable capture requires GPU dynamics";
        ASSERT_EQ(ovphysx_test_get_settings_bool("/physics/suppressReadback", &previousReadback).status,
                  OVPHYSX_API_SUCCESS);
        restoreReadback = true;
        ASSERT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/suppressReadback"),
            GetParam() ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false"))).status,
            OVPHYSX_API_SUCCESS);
    }

    void TearDown() override
    {
        OmniPvdSelectiveCapture::TearDown();
        if (restoreReadback)
        {
            EXPECT_EQ(ovphysx_set_global_config_entry(ovphysx_config_entry_carbonite(
                OVPHYSX_LITERAL("/physics/suppressReadback"),
                previousReadback ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false"))).status,
                OVPHYSX_API_SUCCESS);
        }
    }

    void loadGpuScene(const char* asset, const char* scenePath)
    {
        ASSERT_TRUE(attach_usd_with_ovstage(handle, asset));
        ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        const ::physx::PxScene* scene = static_cast<const ::physx::PxScene*>(
            read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_SCENE, scenePath));
        ASSERT_NE(scene, nullptr);
        ASSERT_TRUE(scene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_GPU_DYNAMICS));
        ASSERT_EQ(scene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_DIRECT_GPU_API), GetParam());
    }

    void* nativeActor(ovphysx_instance_handle_t owner, ovphysx_query_target_t type, const char* actorPath)
    {
        // The legacy query does not expose deformable actor pointer rows.
        // These one-body fixtures have public native scene enumerators.
        if (type == OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE || type == OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME)
        {
            const ::physx::PxScene* scene = static_cast<const ::physx::PxScene*>(
                read_physx_pointer(owner, OVPHYSX_QUERY_TARGET_SCENE, "/World/PhysicsScene"));
            if (!scene)
                return nullptr;
            if (type == OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE)
            {
                ::physx::PxDeformableSurface* surface = nullptr;
                return scene->getNbDeformableSurfaces() == 1 && scene->getDeformableSurfaces(&surface, 1) == 1 ?
                    surface : nullptr;
            }
            ::physx::PxDeformableVolume* volume = nullptr;
            return scene->getNbDeformableVolumes() == 1 && scene->getDeformableVolumes(&volume, 1) == 1 ?
                volume : nullptr;
        }
        return read_physx_pointer(owner, type, actorPath);
    }

    OmniPvdObjectHandle object(ovphysx_instance_handle_t owner, ovphysx_query_target_t type, const char* actorPath)
    {
        return reinterpret_cast<OmniPvdObjectHandle>(nativeActor(owner, type, actorPath));
    }

    void expectMovingPoints(OvdCommandSummary& summary, OmniPvdObjectHandle resource,
                            const char* attribute, size_t floatCount)
    {
        const std::vector<std::vector<uint8_t>>& payloads = summary.pointPayloads[resource][attribute];
        ASSERT_GE(payloads.size(), 2u) << attribute;
        for (const std::vector<uint8_t>& bytes : payloads)
        {
            ASSERT_EQ(bytes.size(), floatCount * sizeof(float));
            for (size_t offset = 0; offset < bytes.size(); offset += sizeof(float))
            {
                float value = 0;
                std::memcpy(&value, bytes.data() + offset, sizeof(value));
                EXPECT_TRUE(std::isfinite(value));
            }
        }
        EXPECT_NE(payloads.front(), payloads.back()) << "Completed solver state must update " << attribute;
    }

    // Base-particle velocity and phase data remain primary when secondary data is disabled.
    void expectParticlePrimaryState(OvdCommandSummary& summary, OmniPvdObjectHandle buffer, size_t floatCount)
    {
        expectMovingPoints(summary, buffer, "velocities", floatCount);
        const auto& phases = summary.pointPayloads[buffer]["phases"];
        ASSERT_FALSE(phases.empty());
        for (const auto& bytes : phases)
            EXPECT_EQ(bytes.size(), floatCount / 4 * sizeof(uint32_t));
    }
};

INSTANTIATE_TEST_SUITE_P(GpuModes, OmniPvdSelectiveCaptureGpuTest, ::testing::Bool());

TEST_F(OmniPvdSelectiveCapture, StatusQueriesDuringRecordingTransitions)
{
    const fs::path destination = path("concurrent_status");
    std::atomic<bool> stop{false};
    std::atomic<bool> badStatus{false};
    std::atomic<uint32_t> queries{0};
    std::thread reader([&] {
        while (!stop.load())
        {
            bool recording = false;
            if (ovphysx_ovd_is_recording(handle, &recording).status != OVPHYSX_API_SUCCESS)
                badStatus.store(true);
            ++queries;
            std::this_thread::yield();
        }
    });
    for (uint32_t wait = 0; wait < 100 && queries.load() == 0; ++wait)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Status is explicitly callback-safe and may run on another thread. All
    // instance work and recording transitions remain on this test thread.
    for (uint32_t iteration = 0; iteration < 8; ++iteration)
    {
        const ovphysx_result_t started = start(handle, destination, nullptr);
        EXPECT_EQ(started.status, OVPHYSX_API_SUCCESS);
        if (started.status != OVPHYSX_API_SUCCESS)
            break;
        bool recording = false;
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(recording);
        EXPECT_EQ(ovphysx_ovd_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(recording);
    }
    stop.store(true);
    reader.join();
    EXPECT_GT(queries.load(), 0u);
    EXPECT_FALSE(badStatus.load());
}

TEST_F(OmniPvdSelectiveCapture, InvalidDescriptorPreservesExistingDestination)
{
    const fs::path file = path("existing");
    const std::string sentinel = "Existing recording must survive invalid capture";
    {
        std::ofstream output(file, std::ios::binary);
        ASSERT_TRUE(output.good());
        output << sentinel;
    }
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_count = 1;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_INVALID_ARGUMENT);
    const ovphysx_string_t missing = OVPHYSX_LITERAL("/World/missingResource");
    capture.object_paths = &missing;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
    std::ifstream input(file, std::ios::binary);
    EXPECT_EQ(std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()), sentinel);
    bool active = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &active).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(active);
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, ParticleSystemIncludesLiveBufferPayload)
{
    loadGpuScene("tests/data/particles_simple.usda", "/World/physicsScene");
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    const OmniPvdObjectHandle system = object(handle, OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM, "/World/particleSystem");
    const OmniPvdObjectHandle buffer = object(handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/particles");
    ASSERT_NE(system, 0u);
    ASSERT_NE(buffer, 0u);
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/particleSystem");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &actor;
    capture.object_count = 1;
    const fs::path file = path("particles");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(system), 1u);
    EXPECT_EQ(summary.createdObjects.count(buffer), 1u);
    expectMovingPoints(summary, buffer, "positionInvMasses", 12);
    expectParticlePrimaryState(summary, buffer, 12);
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, IndividualParticleBufferExcludesSiblingPayload)
{
    loadGpuScene("tests/data/omnipvd_selective_particles.usda", "/World/physicsScene");
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/particles");
    const OmniPvdObjectHandle sibling = object(handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/otherParticles");
    ASSERT_NE(selected, 0u);
    ASSERT_NE(sibling, 0u);
    const ovphysx_string_t resource = OVPHYSX_LITERAL("/World/particles");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = &resource;
    capture.object_count = 1;
    const fs::path file = path("buffer");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    expectMovingPoints(summary, selected, "positionInvMasses", 12);
    expectParticlePrimaryState(summary, selected, 12);
    EXPECT_EQ(summary.writtenAttributes[sibling].count("positionInvMasses"), 0u);
    EXPECT_EQ(summary.writtenAttributes[sibling].count("velocities"), 0u);
    EXPECT_EQ(summary.writtenAttributes[sibling].count("phases"), 0u);
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, DeformableSurfaceIncludesLiveVertices)
{
    loadGpuScene("tests/data/surface_deformable_simple.usda", "/World/PhysicsScene");
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    const ::physx::PxDeformableSurface* surface = static_cast<const ::physx::PxDeformableSurface*>(
        nativeActor(handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, "/World/DeformableBody"));
    ASSERT_NE(surface, nullptr);
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/DeformableBody");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &actor;
    capture.object_count = 1;
    const fs::path file = path("surface");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(reinterpret_cast<OmniPvdObjectHandle>(surface)), 1u);
    expectMovingPoints(summary, reinterpret_cast<OmniPvdObjectHandle>(surface), "positions", 16);
    expectMovingPoints(summary, reinterpret_cast<OmniPvdObjectHandle>(surface), "velocities", 16);
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, DeformableVolumeIncludesLiveVertices)
{
    loadGpuScene("tests/data/volume_deformable_simple.usda", "/World/PhysicsScene");
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    const ::physx::PxDeformableVolume* volume = static_cast<const ::physx::PxDeformableVolume*>(
        nativeActor(handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, "/World/DeformableBody"));
    ASSERT_NE(volume, nullptr);
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/DeformableBody");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &actor;
    capture.object_count = 1;
    const fs::path file = path("volume");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(reinterpret_cast<OmniPvdObjectHandle>(volume)), 1u);
    expectMovingPoints(summary, reinterpret_cast<OmniPvdObjectHandle>(volume), "simulationPositions", 20);
    expectMovingPoints(summary, reinterpret_cast<OmniPvdObjectHandle>(volume), "collisionPositions", 20);
    expectMovingPoints(summary, reinterpret_cast<OmniPvdObjectHandle>(volume), "velocities", 20);
}

TEST_F(OmniPvdSelectiveCapture, InvalidListsAndPathsDoNotOpenDestination)
{
    DirectoryRecordingTestScope directory;
    const fs::path file = directory.root / "nested" / "invalid.ovd";
    ASSERT_FALSE(fs::exists(directory.root));
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.scene_count = 1;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_INVALID_ARGUMENT);
    capture.scene_count = 0;
    capture.object_count = 1;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_INVALID_ARGUMENT);
    capture.object_count = 0;
    const ovphysx_string_t invalidPaths[] = {
        OVPHYSX_LITERAL(""), OVPHYSX_LITERAL("relative"), OVPHYSX_LITERAL("/World/*"),
        OVPHYSX_LITERAL("/World/Cube1\0tail"), { nullptr, 5 }, OVPHYSX_LITERAL("/"),
        OVPHYSX_LITERAL("//World/Cube1"), OVPHYSX_LITERAL("/World//Cube1"),
        OVPHYSX_LITERAL("/World/Cube1/"), OVPHYSX_LITERAL("/World/Cube1.x"),
        OVPHYSX_LITERAL("/World/Cube1:attr"), OVPHYSX_LITERAL("/World/Cube 1"),
        OVPHYSX_LITERAL("/World/Cube\t1"), OVPHYSX_LITERAL("/World/Cube\n1"),
        OVPHYSX_LITERAL("/World/1Cube"), OVPHYSX_LITERAL("/World/Cube-1"),
        OVPHYSX_LITERAL("/World/\xC3"), OVPHYSX_LITERAL("/World/\xED\xA0\x80")
    };
    for (const ovphysx_string_t& primPath : invalidPaths)
    {
        capture.object_paths = &primPath;
        capture.object_count = 1;
        EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_FALSE(fs::exists(file));
        EXPECT_FALSE(fs::exists(directory.root));
    }
    const ovphysx_string_t missing = OVPHYSX_LITERAL("/World/missing");
    capture.object_paths = &missing;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
    const ovphysx_string_t missingUnicodePaths[] = {
        OVPHYSX_LITERAL("/World/\xC3\xA9"), OVPHYSX_LITERAL("/World/\xE4\xB8\xAD"),
        OVPHYSX_LITERAL("/World/env\xC3\xA9")
    };
    for (const ovphysx_string_t& primPath : missingUnicodePaths)
    {
        capture.object_paths = &primPath;
        EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
        EXPECT_FALSE(fs::exists(file));
        EXPECT_FALSE(fs::exists(directory.root));
    }
    const ovphysx_string_t wrongType = OVPHYSX_LITERAL("/World/PhysicsScene");
    capture.object_paths = &wrongType;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(fs::exists(file));
    EXPECT_FALSE(fs::exists(directory.root));
    capture.object_paths = nullptr;
    capture.object_count = 0;
    const ovphysx_string_t invalidScenes[] = {
        OVPHYSX_LITERAL("/World/missingScene"), OVPHYSX_LITERAL("/World/Cube1")
    };
    capture.scene_count = 1;
    for (const ovphysx_string_t& scene : invalidScenes)
    {
        capture.scene_paths = &scene;
        EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
        EXPECT_FALSE(fs::exists(file));
        EXPECT_FALSE(fs::exists(directory.root));
    }
    const ovphysx_string_t malformedScene = OVPHYSX_LITERAL("World/PhysicsScene");
    capture.scene_paths = &malformedScene;
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_FALSE(fs::exists(file));
    EXPECT_FALSE(fs::exists(directory.root));
    capture.scene_paths = nullptr;
    capture.scene_count = 0;
    bool active = true;
    ASSERT_EQ(ovphysx_ovd_is_recording(handle, &active).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(active);
    ASSERT_EQ(start(handle, file, nullptr).status, OVPHYSX_API_SUCCESS);
    finish(handle);
}

TEST_F(OmniPvdSelectiveCapture, DependentResourcePathsAreRejectedAsExplicitRoots)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/omnipvd_selective_resources.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const char* paths[] = { "/World/body/collider", "/World/material", "/World/joint" };
    const ovphysx_query_target_t types[] = {
        OVPHYSX_QUERY_TARGET_SHAPE, OVPHYSX_QUERY_TARGET_PHYSX_MATERIAL, OVPHYSX_QUERY_TARGET_JOINT
    };
    for (size_t index = 0; index < std::size(paths); ++index)
    {
        SCOPED_TRACE(paths[index]);
        const OmniPvdObjectHandle selected = object(handle, types[index], paths[index]);
        ASSERT_NE(selected, 0u);
        const ovphysx_string_t resource = make_ovx_string(paths[index]);
        ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        capture.object_paths = &resource;
        capture.object_count = 1;
        const fs::path file = path(std::to_string(index).c_str());
        EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
        EXPECT_FALSE(fs::exists(file));
    }
}

TEST_F(OmniPvdSelectiveCapture, SharedBodyPathIncludesMirrorsInEveryScene)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/omnipvd_selective_mirrors.usda"));
    std::vector<const ::physx::PxRigidActor*> actors;
    const char* scenePaths[] = { "/World/SceneA", "/World/SceneB" };
    for (const char* scenePath : scenePaths)
    {
        const ::physx::PxScene* scene = static_cast<const ::physx::PxScene*>(
            read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_SCENE, scenePath));
        ASSERT_NE(scene, nullptr);
        const ::physx::PxActorTypeFlags types =
            ::physx::PxActorTypeFlag::eRIGID_STATIC | ::physx::PxActorTypeFlag::eRIGID_DYNAMIC;
        std::vector<::physx::PxActor*> sceneActors(scene->getNbActors(types));
        ASSERT_EQ(scene->getActors(types, sceneActors.data(), static_cast<::physx::PxU32>(sceneActors.size())),
                  sceneActors.size());
        for (const ::physx::PxActor* actor : sceneActors)
            actors.push_back(static_cast<const ::physx::PxRigidActor*>(actor));
    }
    const char* selectedPaths[] = { "/World/SharedDynamic", "/World/SharedStatic" };
    for (const char* selectedPath : selectedPaths)
    {
        const ::physx::PxRigidActor* primary = reinterpret_cast<const ::physx::PxRigidActor*>(
            object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, selectedPath));
        ASSERT_NE(primary, nullptr);
        std::vector<const ::physx::PxRigidActor*> selected;
        std::vector<const ::physx::PxRigidActor*> excluded;
        for (const ::physx::PxRigidActor* actor : actors)
        {
            if (actor->getGlobalPose().p.x == primary->getGlobalPose().p.x)
                selected.push_back(actor);
            else
                excluded.push_back(actor);
        }
        ASSERT_EQ(selected.size(), sizeof(scenePaths) / sizeof(scenePaths[0]));
        ASSERT_FALSE(excluded.empty());
        const ovphysx_string_t root = make_ovx_string(selectedPath);
        ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        capture.record_secondary_data = false;
        capture.object_paths = &root;
        capture.object_count = 1;
        const fs::path file = path(primary->getType() == ::physx::PxActorType::eRIGID_DYNAMIC ?
                                   "dynamic_mirrors" : "static_mirrors");
        ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
        finish(handle);
        OvdCommandSummary summary;
        ASSERT_TRUE(read_ovd_commands(file, summary));
        for (const ::physx::PxRigidActor* actor : selected)
            EXPECT_EQ(summary.createdObjects.count(reinterpret_cast<OmniPvdObjectHandle>(actor)), 1u);
        for (const ::physx::PxRigidActor* actor : excluded)
            EXPECT_EQ(summary.createdObjects.count(reinterpret_cast<OmniPvdObjectHandle>(actor)), 0u);
    }
}

TEST_F(OmniPvdSelectiveCapture, GenericPrototypePathIncludesEveryNativeInstance)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/ovstage_output_instancer.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ::physx::PxScene* scene = static_cast<const ::physx::PxScene*>(
        read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/Scene"));
    ASSERT_NE(scene, nullptr);
    const ::physx::PxActorTypeFlags actorTypes(::physx::PxActorTypeFlag::eRIGID_DYNAMIC);
    std::vector<::physx::PxActor*> actors(scene->getNbActors(actorTypes));
    ASSERT_EQ(scene->getActors(actorTypes, actors.data(), static_cast<::physx::PxU32>(actors.size())), 3u);
    const OmniPvdObjectHandle excluded = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Fixed");
    ASSERT_NE(excluded, 0u);
    const ovphysx_string_t prototype = OVPHYSX_LITERAL("/World/Instances/Prototype");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &prototype;
    capture.object_count = 1;
    const fs::path file = path("instances");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    for (const ::physx::PxActor* actor : actors)
    {
        const OmniPvdObjectHandle identity = reinterpret_cast<OmniPvdObjectHandle>(actor);
        if (identity != excluded)
        {
            EXPECT_EQ(summary.createdObjects.count(identity), 1u);
            EXPECT_EQ(summary.writtenAttributes[identity].count("globalPose"), 1u);
        }
    }
    EXPECT_EQ(summary.writtenAttributes[excluded].count("globalPose"), 0u);
}

TEST_F(OmniPvdSelectiveCapture, PrimaryPrototypePathIncludesEveryNativeInstance)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/ovstage_output_instancer.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ::physx::PxScene* scene = static_cast<const ::physx::PxScene*>(
        read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/Scene"));
    ASSERT_NE(scene, nullptr);
    const ::physx::PxActorTypeFlags actorTypes(::physx::PxActorTypeFlag::eRIGID_DYNAMIC);
    std::vector<::physx::PxActor*> actors(scene->getNbActors(actorTypes));
    ASSERT_EQ(scene->getActors(actorTypes, actors.data(), static_cast<::physx::PxU32>(actors.size())), 3u);
    const OmniPvdObjectHandle excluded = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Fixed");
    ASSERT_NE(excluded, 0u);
    const ovphysx_string_t prototype = OVPHYSX_LITERAL("/World/Instances/Prototype");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = &prototype;
    capture.object_count = 1;
    const fs::path file = path("actor_instances");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    uint32_t capturedInstances = 0;
    for (const ::physx::PxActor* actor : actors)
    {
        const OmniPvdObjectHandle identity = reinterpret_cast<OmniPvdObjectHandle>(actor);
        if (identity != excluded)
        {
            ++capturedInstances;
            EXPECT_EQ(summary.createdObjects.count(identity), 1u);
            EXPECT_EQ(summary.writtenAttributes[identity].count("globalPose"), 1u);
        }
    }
    EXPECT_EQ(capturedInstances, 2u);
    EXPECT_EQ(summary.writtenAttributes[excluded].count("globalPose"), 0u);
}

TEST_F(OmniPvdSelectiveCapture, GenericFixedBaseJointPathCapturesItsArticulation)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/articulation_selection.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const OmniPvdObjectHandle articulation = object(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, "/World/RobotA");
    ASSERT_NE(articulation, 0u);
    const ovphysx_string_t resource = OVPHYSX_LITERAL("/World/RobotA/rootJoint");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &resource;
    capture.object_count = 1;
    const fs::path file = path("fixed_base_alias");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(articulation), 1u);
    EXPECT_FALSE(summary.writtenAttributes[articulation].empty());
}

TEST_F(OmniPvdSelectiveCapture, RuntimeCreatedJointPathIsRejectedAsExplicitRoot)
{
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* joint = ovphysx_create_d6_joint_at_path_for_test_internal(
        handle, "/DirectJoint", "/World/Cube1", "/World/Cube2");
    ASSERT_NE(joint, nullptr);
    const ovphysx_string_t resource = OVPHYSX_LITERAL("/DirectJoint");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &resource;
    capture.object_count = 1;
    const fs::path file = path("runtime_joint");
    EXPECT_EQ(start(handle, file, &capture).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(fs::exists(file));
}

TEST_F(OmniPvdSelectiveCapture, PrimaryObjectFilterKeepsPoseVelocityAndGeometry)
{
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const OmniPvdObjectHandle excluded = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2");
    ASSERT_NE(selected, 0u);
    ASSERT_NE(excluded, 0u);
    ovphysx_string_t actors[] = { OVPHYSX_LITERAL("/World/Cube1") };
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = actors;
    capture.object_count = 1;
    const fs::path file = path("pose");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    // Caller storage is borrowed only until start returns.
    actors[0] = OVPHYSX_LITERAL("/World/Cube2");
    capture.record_secondary_data = true;
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(selected), 1u);
    EXPECT_EQ(summary.createdObjects.count(excluded), 0u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("globalPose"), 1u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("linearVelocity"), 1u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("mass"), 0u);
    EXPECT_GT(summary.shapeObjectCount, 0u);
}

TEST_F(OmniPvdSelectiveCapture, PrimaryDataIsAlwaysRecordedAndSecondaryDataIsOptional)
{
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/Cube1");
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(selected, 0u);
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &actor;
    capture.object_count = 1;
    ASSERT_TRUE(capture.record_secondary_data);
    const bool secondaryOptions[] = { false, true };
    for (bool recordSecondaryData : secondaryOptions)
    {
        capture.record_secondary_data = recordSecondaryData;
        const fs::path file = path(recordSecondaryData ? "secondary" : "primary");
        ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
        finish(handle);
        OvdCommandSummary summary;
        ASSERT_TRUE(read_ovd_commands(file, summary));
        EXPECT_EQ(summary.createdObjects.count(selected), 1u) << recordSecondaryData;
        EXPECT_EQ(summary.writtenAttributes[selected].count("globalPose"), 1u) << recordSecondaryData;
        EXPECT_EQ(summary.writtenAttributes[selected].count("linearVelocity"), 1u) << recordSecondaryData;
        EXPECT_GT(summary.shapeObjectCount, 0u) << recordSecondaryData;
        EXPECT_EQ(summary.writtenAttributes[selected].count("mass"),
                  recordSecondaryData ? 1u : 0u) << recordSecondaryData;
    }
}

TEST_F(OmniPvdSelectiveCapture, DuplicatePathsAreCopiedAndNextSessionSelectsAnotherActor)
{
    const OmniPvdObjectHandle first = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const OmniPvdObjectHandle second = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2");
    ASSERT_NE(first, 0u);
    ASSERT_NE(second, 0u);
    const ovphysx_string_t actors[] = { OVPHYSX_LITERAL("/World/Cube1"), OVPHYSX_LITERAL("/World/Cube1") };
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = actors;
    capture.object_count = 2;
    const fs::path firstFile = path("first");
    ASSERT_EQ(start(handle, firstFile, &capture).status, OVPHYSX_API_SUCCESS);
    const fs::path takeoverFile = path("takeover");
    EXPECT_EQ(start(handle, takeoverFile, nullptr).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(takeoverFile));
    finish(handle);
    const ovphysx_string_t next = OVPHYSX_LITERAL("/World/Cube2");
    capture.object_paths = &next;
    capture.object_count = 1;
    const fs::path secondFile = path("second");
    ASSERT_EQ(start(handle, secondFile, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary a, b;
    ASSERT_TRUE(read_ovd_commands(firstFile, a));
    ASSERT_TRUE(read_ovd_commands(secondFile, b));
    EXPECT_EQ(a.createdObjects.count(first), 1u);
    EXPECT_EQ(a.createdObjects.count(second), 0u);
    EXPECT_EQ(b.createdObjects.count(first), 0u);
    EXPECT_EQ(b.createdObjects.count(second), 1u);
}

TEST_F(OmniPvdSelectiveCapture, IdenticalPathsResolveOnlyWithinCallingAttachment)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const OmniPvdObjectHandle peerObject = object(peer, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(selected, 0u);
    ASSERT_NE(peerObject, 0u);
    ASSERT_NE(selected, peerObject);
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/Cube1");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = &actor;
    capture.object_count = 1;
    const fs::path file = path("owner");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_ovd_stop_recording(peer).status, OVPHYSX_API_INVALID_STATE);
    ASSERT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(selected), 1u);
    EXPECT_EQ(summary.createdObjects.count(peerObject), 0u);
}

TEST_F(OmniPvdSelectiveCapture, UnattachedPeerRequiresAttachmentOnlyForPopulatedSelectors)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/Cube1");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &actor;
    capture.object_count = 1;
    DirectoryRecordingTestScope directory;
    const fs::path file = directory.root / "nested" / "peer.ovd";
    ASSERT_FALSE(fs::exists(directory.root));
    EXPECT_EQ(start(peer, file, &capture).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(file));
    EXPECT_FALSE(fs::exists(directory.root));
    capture.object_paths = nullptr;
    capture.object_count = 0;
    const ovphysx_string_t scene = OVPHYSX_LITERAL("/World/PhysicsScene");
    capture.scene_paths = &scene;
    capture.scene_count = 1;
    EXPECT_EQ(start(peer, file, &capture).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_FALSE(fs::exists(file));
    EXPECT_FALSE(fs::exists(directory.root));
    capture.scene_paths = nullptr;
    capture.scene_count = 0;
    ASSERT_EQ(start(peer, file, &capture).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_ovd_stop_recording(peer).status, OVPHYSX_API_SUCCESS);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_GT(summary.actorObjectCount, 1u);
}

TEST_F(OmniPvdSelectiveCapture, SceneAndObjectSelectorsCombineAsUnion)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/omnipvd_selective_scenes.usda"));
    const OmniPvdObjectHandle a = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/A");
    const OmniPvdObjectHandle b = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/B");
    ASSERT_NE(a, 0u);
    ASSERT_NE(b, 0u);
    const ovphysx_string_t scene = OVPHYSX_LITERAL("/World/SceneA");
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/B");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.scene_paths = &scene;
    capture.scene_count = 1;
    const fs::path sceneFile = path("scene");
    ASSERT_EQ(start(handle, sceneFile, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary sceneSummary;
    ASSERT_TRUE(read_ovd_commands(sceneFile, sceneSummary));
    EXPECT_EQ(sceneSummary.createdObjects.count(a), 1u);
    EXPECT_EQ(sceneSummary.createdObjects.count(b), 0u);
    capture.object_paths = &actor;
    capture.object_count = 1;
    const fs::path unionFile = path("union");
    ASSERT_EQ(start(handle, unionFile, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary unionSummary;
    ASSERT_TRUE(read_ovd_commands(unionFile, unionSummary));
    EXPECT_EQ(unionSummary.createdObjects.count(a), 1u);
    EXPECT_EQ(unionSummary.createdObjects.count(b), 1u);
}

TEST_F(OmniPvdSelectiveCapture, ArticulationSelectionIncludesLinksAndExcludesPeerArticulation)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/two_articulations.usda"));
    const OmniPvdObjectHandle articulation = object(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, "/World/articulation");
    const OmniPvdObjectHandle link = object(handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK,
                                           "/World/articulation/articulationLink1");
    const OmniPvdObjectHandle excluded = object(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, "/World/articulation2");
    ASSERT_NE(articulation, 0u);
    ASSERT_NE(link, 0u);
    ASSERT_NE(excluded, 0u);
    const ovphysx_string_t root = OVPHYSX_LITERAL("/World/articulation");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = &root;
    capture.object_count = 1;
    const ovphysx_string_t linkPath = OVPHYSX_LITERAL("/World/articulation/articulationLink1");
    capture.object_paths = &linkPath;
    const fs::path rejectedFile = path("link_root");
    EXPECT_EQ(start(handle, rejectedFile, &capture).status, OVPHYSX_API_NOT_FOUND);
    EXPECT_FALSE(fs::exists(rejectedFile));
    capture.object_paths = &root;
    const fs::path file = path("articulation");
    ASSERT_EQ(start(handle, file, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(articulation), 1u);
    EXPECT_EQ(summary.createdObjects.count(link), 1u);
    EXPECT_EQ(summary.createdObjects.count(excluded), 0u);
}

TEST_F(OmniPvdSelectiveCapture, VehicleTelemetryRequiresSelectedObjectAndSecondaryData)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/vehicle_read_write.usda"));
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Basic");
    ASSERT_NE(selected, 0u);
    ovphysx_string_t actor = OVPHYSX_LITERAL("/World/Basic");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.object_paths = &actor;
    capture.object_count = 1;
    capture.record_secondary_data = false;
    const fs::path poses = path("vehicle_poses");
    ASSERT_EQ(start(handle, poses, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary poseSummary;
    ASSERT_TRUE(read_ovd_commands(poses, poseSummary));
    EXPECT_EQ(poseSummary.createdObjects.count(selected), 1u);
    EXPECT_EQ(poseSummary.vehicleObjectCount, 0u);

    capture.record_secondary_data = true;
    const fs::path selectedFile = path("vehicle_selected");
    ASSERT_EQ(start(handle, selectedFile, &capture).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/vehicle_read_write.usda"));
    ASSERT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary selectedSummary;
    ASSERT_TRUE(read_ovd_commands(selectedFile, selectedSummary));
    EXPECT_EQ(selectedSummary.vehicleObjectCount, 1u);
    ASSERT_TRUE(destroy_ovstage_test_attachments(peer));
    ASSERT_EQ(ovphysx_destroy_instance(peer).status, OVPHYSX_API_SUCCESS);
    peer = OVPHYSX_INVALID_HANDLE;

    actor = OVPHYSX_LITERAL("/World/CollisionPlane");
    const fs::path excludedFile = path("vehicle_excluded");
    ASSERT_EQ(start(handle, excludedFile, &capture).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary excludedSummary;
    ASSERT_TRUE(read_ovd_commands(excludedFile, excludedSummary));
    EXPECT_EQ(excludedSummary.vehicleObjectCount, 0u);

    const fs::path legacyFile = path("vehicle_legacy");
    ASSERT_EQ(start(handle, legacyFile, nullptr).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary legacySummary;
    ASSERT_TRUE(read_ovd_commands(legacyFile, legacySummary));
    EXPECT_EQ(legacySummary.vehicleObjectCount, 4u);
}

TEST_F(OmniPvdSelectiveCapture, TcpSenderAppliesTheSamePrimaryDataFilter)
{
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const OmniPvdObjectHandle excluded = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2");
    const ovphysx_string_t actor = OVPHYSX_LITERAL("/World/Cube1");
    ovphysx_omnipvd_recording_options_t capture = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
    capture.record_secondary_data = false;
    capture.object_paths = &actor;
    capture.object_count = 1;
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &capture](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination, &capture).status;
        if (status == OVPHYSX_API_SUCCESS)
            status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
        if (status == OVPHYSX_API_SUCCESS)
            status = ovphysx_ovd_stop_recording(handle).status;
        return status;
    }, received));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("tcp");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.createdObjects.count(selected), 1u);
    EXPECT_EQ(summary.createdObjects.count(excluded), 0u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("globalPose"), 1u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("linearVelocity"), 1u);
    EXPECT_GT(summary.shapeObjectCount, 0u);
}

TEST_F(OmniPvdSelectiveCapture, TcpDiscoveryRejectsFileAndMissingAttachment)
{
    const fs::path file = path("discovery_rejected");
    const std::string filename = file.u8string();
    ovphysx_omnipvd_destination_t destination{};
    destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_FILE;
    destination.file_path = make_ovx_string(filename.c_str());
    EXPECT_EQ(start_discovery(handle, &destination, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_FALSE(fs::exists(file));
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
    destination.file_path = OVPHYSX_LITERAL("");
    destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
    destination.tcp_port = 1;
    EXPECT_EQ(start_discovery(peer, &destination, nullptr).status, OVPHYSX_API_INVALID_STATE);
    bool active = true;
    EXPECT_EQ(ovphysx_ovd_is_recording(handle, &active).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(active);
}

TEST_F(OmniPvdSelectiveCapture, TcpDiscoveryCatalogIsOwnedAndHasNoSimulationPayload)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const OmniPvdObjectHandle sibling = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2");
    const OmniPvdObjectHandle foreign = object(peer, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(selected, 0u);
    ASSERT_NE(sibling, 0u);
    ASSERT_NE(foreign, 0u);
    for (bool scoped : { false, true })
    {
        TcpCaptureResult received;
        ASSERT_TRUE(capture_loopback_tcp([this, scoped, selected](uint16_t port) -> ovphysx_api_status_t {
            ovphysx_omnipvd_destination_t destination{};
            destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
            destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
            destination.tcp_port = port;
            destination.tcp_timeout_ms = 3000;
            const ovphysx_string_t root = OVPHYSX_LITERAL("/World/Cube1");
            ovphysx_omnipvd_recording_options_t scope = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
            scope.object_paths = &root;
            scope.object_count = 1;
            const ovphysx_api_status_t started = start_discovery(
                handle, &destination, scoped ? &scope : nullptr).status;
            if (started != OVPHYSX_API_SUCCESS) return started;
            const ::physx::PxRigidActor* actor = reinterpret_cast<const ::physx::PxRigidActor*>(selected);
            ::physx::PxOmniPvd* native = actor->getScene()->getPhysics().getOmniPvd();
            EXPECT_FALSE(native->isSampling());
            EXPECT_TRUE(native->isCaptureDiscoveryActive());
            bool active = false;
            EXPECT_EQ(ovphysx_ovd_is_recording(handle, &active).status, OVPHYSX_API_SUCCESS);
            EXPECT_TRUE(active);
            EXPECT_EQ(ovphysx_ovd_is_recording(peer, &active).status, OVPHYSX_API_SUCCESS);
            EXPECT_FALSE(active);
            EXPECT_EQ(ovphysx_ovd_stop_recording(peer).status, OVPHYSX_API_INVALID_STATE);
            const ovphysx_api_status_t stepped = ovphysx_step_sync(handle, 0.01f).status;
            EXPECT_FALSE(native->isSampling());
            if (scoped)
            {
                EXPECT_TRUE(destroy_ovstage_test_attachments(handle));
                EXPECT_EQ(ovphysx_ovd_is_recording(handle, &active).status, OVPHYSX_API_SUCCESS);
                EXPECT_FALSE(active);
                return stepped;
            }
            const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(handle).status;
            return stepped == OVPHYSX_API_SUCCESS ? stopped : stepped;
        }, received));
        EXPECT_TRUE(received.cleanEof);
        constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
        ASSERT_GT(received.bytes.size(), handshakeSize);
        const fs::path file = path(scoped ? "discovery_scoped" : "discovery_owner");
        {
            std::ofstream output(file, std::ios::binary);
            ASSERT_TRUE(output);
            output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                         static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
        }
        OvdCommandSummary summary;
        ASSERT_TRUE(read_ovd_commands(file, summary));
        EXPECT_EQ(summary.discoveryTargets.count(selected), 1u);
        EXPECT_EQ(summary.discoveryTargets.count(sibling), scoped ? 0u : 1u);
        EXPECT_EQ(summary.discoveryTargets.count(foreign), 0u);
        EXPECT_EQ(summary.actorObjectCount, 0u);
        EXPECT_EQ(summary.shapeObjectCount, 0u);
            EXPECT_TRUE(summary.pointPayloads.empty());
        for (const auto& objectAttributes : summary.writtenAttributes)
            for (const char* attribute : { "globalPose", "linearVelocity", "angularVelocity", "vertices", "force", "torque" })
                EXPECT_EQ(objectAttributes.second.count(attribute), 0u) << attribute;
    }
}

// Raw wire fixture keeps unknown flag bits independently of the public boolean API.
struct CaptureWireRequest
{
    uint64_t requestId{ 0 };
    uint32_t flags{ 1u };
    const OmniPvdObjectHandle* scenes{ nullptr };
    uint32_t nbScenes{ 0 };
    const OmniPvdObjectHandle* objects{ nullptr };
    uint32_t nbObjects{ 0 };
};

static void append_capture_request(std::vector<uint8_t>& wire, const CaptureWireRequest& request)
{
    // Interoperability fixture: control integers are little endian and the
    // OVC3 header is 28 bytes, then scene and object U64 IDs.
    const auto appendInteger = [&wire](uint64_t value, uint32_t width) {
        for (uint32_t byte = 0; byte < width; ++byte)
            wire.push_back(static_cast<uint8_t>(value >> (8 * byte)));
    };
    const uint32_t counts[] = { request.nbScenes, request.nbObjects };
    const OmniPvdObjectHandle* lists[] = { request.scenes, request.objects };
    const uint32_t listCount = 2;
    const uint32_t headerSize = 28;
    appendInteger(0x3343564fu, 4); // OVC3
    appendInteger(headerSize + 8 * (request.nbScenes + request.nbObjects), 4);
    appendInteger(request.requestId, 8);
    appendInteger(request.flags, 4);
    for (uint32_t list = 0; list < listCount; ++list)
        appendInteger(counts[list], 4);
    for (uint32_t list = 0; list < listCount; ++list)
        for (uint32_t index = 0; index < counts[list]; ++index)
            appendInteger(lists[list][index], 8);
}

TEST_F(OmniPvdSelectiveCapture, TcpDiscoverySelectionCannotWidenInitialScope)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const OmniPvdObjectHandle sibling = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2");
    const OmniPvdObjectHandle foreign = object(peer, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(selected, 0u);
    ASSERT_NE(sibling, 0u);
    ASSERT_NE(foreign, 0u);
    const auto control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.flags = 0;
    request.nbObjects = 1;
    request.requestId = 901;
    request.objects = &sibling;
    append_capture_request(*control, request);
    request.requestId = 902;
    request.objects = &foreign;
    append_capture_request(*control, request);
    request.requestId = 903;
    request.objects = &selected;
    append_capture_request(*control, request);
    request.requestId = 904;
    request.objects = nullptr;
    request.nbObjects = 0;
    append_capture_request(*control, request);
    const auto sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent, selected](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        const ovphysx_string_t root = OVPHYSX_LITERAL("/World/Cube1");
        ovphysx_omnipvd_recording_options_t scope = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        scope.object_paths = &root;
        scope.object_count = 1;
        ovphysx_api_status_t status = start_discovery(handle, &destination, &scope).status;
        if (status != OVPHYSX_API_SUCCESS) return status;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (!sent->load()) status = OVPHYSX_API_ERROR;
        const ::physx::PxRigidActor* actor = reinterpret_cast<const ::physx::PxRigidActor*>(selected);
        ::physx::PxOmniPvd* native = actor->getScene()->getPhysics().getOmniPvd();
        bool pending = false;
        while (!pending && std::chrono::steady_clock::now() < deadline)
        {
            pending = native->hasPendingCaptureRequest();
            if (!pending)
                std::this_thread::yield();
        }
        EXPECT_TRUE(pending);
        EXPECT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(native->isSampling());
        EXPECT_TRUE(native->isCaptureDiscoveryActive());
        EXPECT_TRUE(native->hasPendingCaptureRequest());
        for (uint32_t step = 0; step < 6 && status == OVPHYSX_API_SUCCESS; ++step)
            status = ovphysx_step_sync(handle, 0.01f).status;
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(handle).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("discovery_selected");
    {
        std::ofstream output(file, std::ios::binary);
        ASSERT_TRUE(output);
        output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.captureResults, (std::vector<std::pair<uint64_t, uint32_t>>{ {901, 4}, {902, 4}, {903, 1}, {904, 1} }));
    EXPECT_EQ(summary.discoveryTargets.count(selected), 1u);
    EXPECT_EQ(summary.discoveryTargets.count(sibling), 0u);
    EXPECT_EQ(summary.discoveryTargets.count(foreign), 0u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("globalPose"), 1u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("linearVelocity"), 1u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("mass"), 0u);
    EXPECT_EQ(summary.createdObjects.count(sibling), 0u);
    EXPECT_EQ(summary.createdObjects.count(foreign), 0u);
}

TEST_F(OmniPvdSelectiveCapture, TcpDiscoveryReceiverChoosesSecondaryData)
{
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(selected, 0u);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.requestId = 905;
    request.flags = 1u; // secondary data
    request.objects = &selected;
    request.nbObjects = 1;
    append_capture_request(*control, request);
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        const ovphysx_string_t root = OVPHYSX_LITERAL("/World/Cube1");
        ovphysx_omnipvd_recording_options_t scope = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        scope.record_secondary_data = false;
        scope.object_paths = &root;
        scope.object_count = 1;
        ovphysx_api_status_t status = start_discovery(handle, &destination, &scope).status;
        if (status != OVPHYSX_API_SUCCESS) return status;
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (!sent->load()) status = OVPHYSX_API_ERROR;
        for (uint32_t step = 0; step < 6 && status == OVPHYSX_API_SUCCESS; ++step)
            status = ovphysx_step_sync(handle, 0.01f).status;
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(handle).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("discovery_selected");
    {
        std::ofstream output(file, std::ios::binary);
        ASSERT_TRUE(output);
        output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.captureResults, (std::vector<std::pair<uint64_t, uint32_t>>{ {905, 1} }));
    EXPECT_EQ(summary.writtenAttributes[selected].count("globalPose"), 1u);
    EXPECT_EQ(summary.writtenAttributes[selected].count("mass"), 1u);
}

static ::testing::AssertionResult capture_remote_policies(
    ovphysx_instance_handle_t owner, const std::shared_ptr<std::vector<uint8_t>>& control,
    TcpCaptureResult& received, const char* initialObject = "/World/Cube1")
{
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    return capture_loopback_tcp([owner, &sent, initialObject](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_omnipvd_recording_options_t discovery = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        discovery.record_secondary_data = false;
        const ovphysx_string_t initialRoot = { initialObject, std::strlen(initialObject) };
        discovery.object_paths = &initialRoot;
        discovery.object_count = 1;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(owner, &destination, &discovery).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!sent->load())
        {
            ovphysx_ovd_stop_recording(owner);
            return OVPHYSX_API_ERROR;
        }
        for (uint32_t step = 0; step < 6 && status == OVPHYSX_API_SUCCESS; ++step)
            status = ovphysx_step_sync(owner, 1.0f / 60.0f).status;
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(owner).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent);
}

// @implements REQ-OMNIPVD-CONTROL-001
// @covers AC-1 AC-3
TEST_F(OmniPvdSelectiveCapture, TcpPendingRequestWaitsForQueuedPeerWithoutCollectingItsReceipt)
{
    if (!ovphysxInstancesHaveWorkers())
        GTEST_SKIP() << "requires queued worker execution";
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 0.01f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(peer, 0.01f).status, OVPHYSX_API_SUCCESS);
    const auto* actor = static_cast<const ::physx::PxRigidActor*>(
        read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(actor, nullptr);
    ::physx::PxOmniPvd* omniPvd = actor->getScene()->getPhysics().getOmniPvd();
    ASSERT_NE(omniPvd, nullptr);
    const OmniPvdObjectHandle selected = reinterpret_cast<OmniPvdObjectHandle>(actor);
    const auto control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.requestId = 801;
    request.flags = 0u;
    request.objects = &selected;
    request.nbObjects = 1;
    append_capture_request(*control, request);
    const auto sent = std::make_shared<std::atomic<bool>>(false);
    const auto permit = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([&](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_api_status_t status =
            ovphysx_ovd_start_recording(handle, &destination, nullptr).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        struct WorkerPause
        {
            ovphysx_instance_handle_t handle;
            bool held;
            explicit WorkerPause(ovphysx_instance_handle_t value)
                : handle(value), held(ovphysx_set_worker_hold_for_test_internal(value, true)) {}
            void release()
            {
                if (held)
                    ovphysx_set_worker_hold_for_test_internal(handle, false);
                held = false;
            }
            ~WorkerPause() { release(); }
        } pause(peer);
        EXPECT_TRUE(pause.held);
        const ovphysx_enqueue_result_t queued = ovphysx_submit_test_job_for_test_internal(peer, nullptr, 0);
        EXPECT_EQ(queued.status, OVPHYSX_API_SUCCESS);
        std::promise<ovphysx_result_t> emptyPollResult;
        std::future<ovphysx_result_t> emptyPoll = emptyPollResult.get_future();
        std::thread emptyStep([&] { emptyPollResult.set_value(ovphysx_step_sync(handle, 0.01f)); });
        const bool emptyPollFinished = emptyPoll.wait_for(std::chrono::seconds(3)) == std::future_status::ready;
        if (!emptyPollFinished)
            pause.release();
        emptyStep.join();
        EXPECT_TRUE(emptyPollFinished) << "An empty receiver poll must not drain a peer's queued work";
        EXPECT_EQ(emptyPoll.get().status, OVPHYSX_API_SUCCESS);
        if (!emptyPollFinished)
            return ovphysx_ovd_stop_recording(handle).status;
        ovphysx_task_status_t heldState = OVPHYSX_TASK_STATUS_NOT_FOUND;
        EXPECT_EQ(ovphysx_query_task_status(peer, queued, &heldState).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(heldState, OVPHYSX_TASK_STATUS_PENDING);
        permit->store(true);
        {
            ::physx::PxOmniPvd::ScopedExclusiveWriter lock(omniPvd);
            omniPvd->getWriter()->startFrame(1, 0);
            omniPvd->getWriter()->stopFrame(1, 0);
            omniPvd->getWriter()->getWriteStream()->flush();
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        bool pending = false;
        while (!pending && std::chrono::steady_clock::now() < deadline)
        {
            pending = sent->load() && omniPvd->hasPendingCaptureRequest();
            if (!pending)
                std::this_thread::yield();
        }
        EXPECT_TRUE(pending);
        std::atomic<bool> entered{false}, returned{false};
        ovphysx_enqueue_result_t ownerStep{};
        std::thread owner([&] {
            entered = true;
            ownerStep = ovphysx_step(handle, 0.01f);
            returned = true;
        });
        const auto entryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!entered && std::chrono::steady_clock::now() < entryDeadline)
            std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const bool returnedBeforeRelease = returned.load();
        ovphysx_task_status_t state = OVPHYSX_TASK_STATUS_NOT_FOUND;
        EXPECT_EQ(ovphysx_query_task_status(peer, queued, &state).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(state, OVPHYSX_TASK_STATUS_PENDING);
        pause.release();
        owner.join();
        EXPECT_FALSE(returnedBeforeRelease);
        EXPECT_EQ(ownerStep.status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(waitForTaskSuccess(peer, queued, OVPHYSX_TIMEOUT_POLL));
        EXPECT_TRUE(waitForTaskSuccess(handle, ownerStep, OVPHYSX_TIMEOUT_INFINITE));
        return ovphysx_ovd_stop_recording(handle).status;
    }, received, control, sent, false, permit));
    ASSERT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("tcp_queued_receipts");
    {
        std::ofstream output(file, std::ios::binary);
        ASSERT_TRUE(output);
        output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.captureResults, (std::vector<std::pair<uint64_t, uint32_t>>{{801, 1}}));
}


TEST_F(OmniPvdSelectiveCapture, TcpMaterialsFollowActorSelectionAndRejectStandaloneRoots)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ::physx::PxRigidActor* ownActor = static_cast<const ::physx::PxRigidActor*>(
        read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    const ::physx::PxRigidActor* peerActor = static_cast<const ::physx::PxRigidActor*>(
        read_physx_pointer(peer, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(ownActor, nullptr);
    ASSERT_NE(peerActor, nullptr);
    ::physx::PxShape* ownShape = nullptr;
    ::physx::PxShape* peerShape = nullptr;
    ASSERT_EQ(ownActor->getShapes(&ownShape, 1), 1u);
    ASSERT_EQ(peerActor->getShapes(&peerShape, 1), 1u);
    ::physx::PxMaterial* ownMaterial = nullptr;
    ::physx::PxMaterial* peerMaterial = nullptr;
    ASSERT_EQ(ownShape->getMaterials(&ownMaterial, 1), 1u);
    ASSERT_EQ(peerShape->getMaterials(&peerMaterial, 1), 1u);
    ASSERT_NE(ownMaterial, nullptr);
    ASSERT_NE(peerMaterial, nullptr);
    ASSERT_NE(ownMaterial, peerMaterial);
    const OmniPvdObjectHandle selected = reinterpret_cast<OmniPvdObjectHandle>(ownMaterial);
    const OmniPvdObjectHandle foreign = reinterpret_cast<OmniPvdObjectHandle>(peerMaterial);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.flags = 1u;
    request.nbObjects = 1;
    request.objects = &foreign;
    request.requestId = 501;
    append_capture_request(*control, request);
    request.objects = &selected;
    request.requestId = 502;
    append_capture_request(*control, request);
    const OmniPvdObjectHandle actor = reinterpret_cast<OmniPvdObjectHandle>(ownActor);
    request.objects = &actor;
    request.requestId = 503;
    append_capture_request(*control, request);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_remote_policies(handle, control, received));
    ASSERT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("default_material");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {501, 2}, {502, 2}, {503, 1} };
    EXPECT_EQ(summary.captureResults, expected);
    EXPECT_GE(summary.createCounts[selected], 2u);
    EXPECT_FALSE(summary.currentAttributes[selected].empty());
    EXPECT_TRUE(summary.writtenAttributes[foreign].empty());
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, TcpGpuMaterialsFollowActorSelectionAndRejectStandaloneRoots)
{
    const char* assets[] = { "tests/data/particles_simple.usda", "tests/data/surface_deformable_simple.usda",
                            "tests/data/volume_deformable_simple.usda" };
    const ovphysx_query_target_t types[] = { OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM, OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE,
                                          OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME };
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    for (uint32_t index = 0; index < 3; ++index)
    {
        SCOPED_TRACE(assets[index]);
        // Remove authored bindings from a temporary copy to exercise the
        // generated scene materials rather than mapped material prims.
        fs::path fixture = path(("gpu_default_material_fixture_" + std::to_string(index)).c_str());
        fixture.replace_extension(".usda");
        files.push_back(fixture);
        {
            std::ifstream source(assets[index]);
            std::ofstream output(fixture);
            ASSERT_TRUE(source && output);
            bool skippingBinding = false;
            for (std::string line; std::getline(source, line);)
            {
                if (line.find("rel material:binding:physics") != std::string::npos)
                    skippingBinding = true;
                if (skippingBinding)
                {
                    if (line.find(')') != std::string::npos)
                        skippingBinding = false;
                    continue;
                }
                output << line << '\n';
            }
        }
        const std::string fixtureName = fixture.string();
        const char* actorPath = index == 0 ? "/World/particleSystem" : "/World/DeformableBody";
        loadGpuScene(fixtureName.c_str(), index == 0 ? "/World/physicsScene" : "/World/PhysicsScene");
        ASSERT_FALSE(::testing::Test::HasFatalFailure());
        ASSERT_TRUE(attach_usd_with_ovstage(peer, fixtureName.c_str()));
        ASSERT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        const ::physx::PxBase* materials[2] = {};
        const ovphysx_instance_handle_t owners[] = { handle, peer };
        for (uint32_t ownerIndex = 0; ownerIndex < 2; ++ownerIndex)
        {
            void* actor = nativeActor(owners[ownerIndex], types[index], actorPath);
            ASSERT_NE(actor, nullptr);
            if (index == 0)
            {
                ::physx::PxPBDMaterial* material = nullptr;
                ASSERT_EQ(static_cast<::physx::PxPBDParticleSystem*>(actor)->getParticleMaterials(&material, 1), 1u);
                materials[ownerIndex] = material;
            }
            else if (index == 1)
            {
                ::physx::PxDeformableSurfaceMaterial* material = nullptr;
                ASSERT_EQ(static_cast<::physx::PxDeformableSurface*>(actor)->getShape()->getDeformableSurfaceMaterials(
                              &material, 1), 1u);
                materials[ownerIndex] = material;
            }
            else
            {
                ::physx::PxDeformableVolumeMaterial* material = nullptr;
                ASSERT_EQ(static_cast<::physx::PxDeformableVolume*>(actor)->getShape()->getDeformableVolumeMaterials(
                              &material, 1), 1u);
                materials[ownerIndex] = material;
            }
            ASSERT_NE(materials[ownerIndex], nullptr);
        }
        ASSERT_NE(materials[0], materials[1]);
        const OmniPvdObjectHandle selected = reinterpret_cast<OmniPvdObjectHandle>(materials[0]);
        const OmniPvdObjectHandle foreign = reinterpret_cast<OmniPvdObjectHandle>(materials[1]);
        const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
        CaptureWireRequest request;
        request.flags = 1u;
        request.nbObjects = 1;
        request.objects = &foreign;
        request.requestId = 601;
        append_capture_request(*control, request);
        request.objects = &selected;
        request.requestId = 602;
        append_capture_request(*control, request);
        const OmniPvdObjectHandle actor = reinterpret_cast<OmniPvdObjectHandle>(
            nativeActor(handle, types[index], actorPath));
        request.objects = &actor;
        request.requestId = 603;
        append_capture_request(*control, request);
        TcpCaptureResult received;
        ASSERT_TRUE(capture_remote_policies(handle, control, received, actorPath));
        ASSERT_TRUE(received.cleanEof);
        constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
        ASSERT_GT(received.bytes.size(), handshakeSize);
        const fs::path file = path(("gpu_default_material_" + std::to_string(index)).c_str());
        {
            std::ofstream stream(file, std::ios::binary);
            ASSERT_TRUE(stream);
            stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                         static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
        }
        OvdCommandSummary summary;
        ASSERT_TRUE(read_ovd_commands(file, summary));
        const std::vector<std::pair<uint64_t, uint32_t>> expected = { {601, 2}, {602, 2}, {603, 1} };
        EXPECT_EQ(summary.captureResults, expected);
        EXPECT_GE(summary.createCounts[selected], 2u);
        EXPECT_FALSE(summary.currentAttributes[selected].empty());
        EXPECT_TRUE(summary.writtenAttributes[foreign].empty());
        ASSERT_TRUE(destroy_ovstage_test_attachments(peer));
        ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    }
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, TcpDeformableTransitionsUseCurrentActorLocalArrays)
{
    const char* assets[] = { "tests/data/surface_deformable_simple.usda", "tests/data/volume_deformable_simple.usda" };
    const ovphysx_query_target_t types[] = { OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME };
    for (size_t index = 0; index < std::size(assets); ++index)
    {
        SCOPED_TRACE(assets[index]);
        loadGpuScene(assets[index], "/World/PhysicsScene");
        ASSERT_FALSE(::testing::Test::HasFatalFailure());
        const OmniPvdObjectHandle actor = object(handle, types[index], "/World/DeformableBody");
        ASSERT_NE(actor, 0u);
        std::vector<const ::physx::PxBase*> meshes;
        if (index == 0)
        {
            ::physx::PxDeformableSurface* surface = static_cast<::physx::PxDeformableSurface*>(
                nativeActor(handle, types[index], "/World/DeformableBody"));
            meshes.push_back(static_cast<const ::physx::PxTriangleMeshGeometry&>(
                surface->getShape()->getGeometry()).triangleMesh);
        }
        else
        {
            const ::physx::PxDeformableVolume* volume = static_cast<const ::physx::PxDeformableVolume*>(
                nativeActor(handle, types[index], "/World/DeformableBody"));
            meshes = { volume->getSimulationMesh(), volume->getCollisionMesh() };
        }
        const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
        CaptureWireRequest request;
        request.objects = &actor;
        request.nbObjects = 1;
        request.flags = 1u;
        request.requestId = 401;
        append_capture_request(*control, request);
        request.flags = 0;
        request.requestId = 402;
        append_capture_request(*control, request);
        request.flags = 0x80000000u; // Unknown bits do not change mandatory primary data.
        request.requestId = 403;
        append_capture_request(*control, request);
        TcpCaptureResult received;
        ASSERT_TRUE(capture_remote_policies(handle, control, received, "/World/DeformableBody"));
        ASSERT_TRUE(received.cleanEof);
        constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
        ASSERT_GT(received.bytes.size(), handshakeSize);
        const fs::path file = path(index == 0 ? "tcp_surface" : "tcp_volume");
        {
            std::ofstream stream(file, std::ios::binary);
            ASSERT_TRUE(stream);
            stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                         static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
        }
        OvdCommandSummary summary;
        ASSERT_TRUE(read_ovd_commands(file, summary));
        EXPECT_EQ(summary.captureResults, (std::vector<std::pair<uint64_t, uint32_t>>{
            {401, 1}, {402, 1}, {403, 1}}));
        expectMovingPoints(summary, actor, index == 0 ? "positions" : "simulationPositions", index == 0 ? 16 : 20);
        expectMovingPoints(summary, actor, "velocities", index == 0 ? 16 : 20);
        // Primary-only epochs retain the geometry needed to interpret the selected actor.
        for (const ::physx::PxBase* mesh : meshes)
        {
            const OmniPvdObjectHandle identity = reinterpret_cast<OmniPvdObjectHandle>(mesh);
            EXPECT_EQ(summary.writtenAttributes[identity].count("verts"), 1u);
            EXPECT_EQ(summary.currentAttributes[identity].count("verts"), 1u);
            EXPECT_EQ(summary.currentAttributes[identity].count(index == 0 ? "tris" : "tets"), 1u);
        }
        ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    }
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, TcpVolumeMeshesFollowActorSelectionAndRejectStandaloneRoots)
{
    loadGpuScene("tests/data/volume_deformable_simple.usda", "/World/PhysicsScene");
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    fs::path peerFixture = path("peer_volume_fixture");
    peerFixture.replace_extension(".usda");
    files.push_back(peerFixture);
    {
        std::ifstream source("tests/data/volume_deformable_simple.usda");
        std::ofstream output(peerFixture);
        ASSERT_TRUE(source && output);
        for (std::string line; std::getline(source, line);)
        {
            const size_t position = line.find("(1, 1, 1)");
            if (position != std::string::npos)
                line.replace(position, 9, "(1.2, 1, 1)");
            output << line << '\n';
        }
    }
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, peerFixture.string().c_str()));
    ASSERT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ::physx::PxDeformableVolume* ownVolume = static_cast<const ::physx::PxDeformableVolume*>(
        nativeActor(handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, "/World/DeformableBody"));
    const ::physx::PxDeformableVolume* peerVolume = static_cast<const ::physx::PxDeformableVolume*>(
        nativeActor(peer, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, "/World/DeformableBody"));
    ASSERT_NE(ownVolume, nullptr);
    ASSERT_NE(peerVolume, nullptr);
    const fs::path discovery = path("volume_parent_discovery");
    ASSERT_EQ(start(handle, discovery, nullptr).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary catalog;
    ASSERT_TRUE(read_ovd_commands(discovery, catalog));
    const OmniPvdObjectHandle selected = catalog.volumeMeshParents[
        reinterpret_cast<OmniPvdObjectHandle>(ownVolume->getSimulationMesh())];
    const OmniPvdObjectHandle foreign = catalog.volumeMeshParents[
        reinterpret_cast<OmniPvdObjectHandle>(peerVolume->getSimulationMesh())];
    ASSERT_NE(selected, 0u);
    ASSERT_NE(foreign, 0u);
    ASSERT_NE(selected, foreign);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.flags = 0u;
    request.nbObjects = 1;
    request.objects = &foreign;
    request.requestId = 701;
    append_capture_request(*control, request);
    request.objects = &selected;
    request.requestId = 702;
    append_capture_request(*control, request);
    const OmniPvdObjectHandle actor = reinterpret_cast<OmniPvdObjectHandle>(ownVolume);
    request.objects = &actor;
    request.requestId = 703;
    append_capture_request(*control, request);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_remote_policies(handle, control, received, "/World/DeformableBody"));
    ASSERT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("volume_parent");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {701, 2}, {702, 2}, {703, 1} };
    EXPECT_EQ(summary.captureResults, expected);
    EXPECT_EQ(summary.currentAttributes[actor].count("simulationMesh"), 1u);
    EXPECT_EQ(summary.currentAttributes[actor].count("collisionMesh"), 1u);
    ASSERT_NE(ownVolume->getSimulationMesh(), peerVolume->getSimulationMesh());
    const OmniPvdObjectHandle ownSimulationMesh = reinterpret_cast<OmniPvdObjectHandle>(ownVolume->getSimulationMesh());
    EXPECT_GE(summary.createCounts[ownSimulationMesh], 2u);
    EXPECT_EQ(summary.currentAttributes[ownSimulationMesh].count("verts"), 1u);
    EXPECT_EQ(summary.currentAttributes[ownSimulationMesh].count("tets"), 1u);
    // The initial actor selection and subsequent requests must never expose
    // the peer's dense mesh payload.
    for (const ::physx::PxTetrahedronMesh* mesh : { peerVolume->getSimulationMesh(), peerVolume->getCollisionMesh() })
    {
        const OmniPvdObjectHandle identity = reinterpret_cast<OmniPvdObjectHandle>(mesh);
        EXPECT_EQ(summary.writtenAttributes[identity].count("verts"), 0u);
        EXPECT_EQ(summary.writtenAttributes[identity].count("tets"), 0u);
    }
}

TEST_P(OmniPvdSelectiveCaptureGpuTest, TcpBufferTransitionsRejectPeerAndRetireExcludedPayload)
{
    loadGpuScene("tests/data/omnipvd_selective_particles.usda", "/World/physicsScene");
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/omnipvd_selective_particles.usda"));
    ASSERT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const OmniPvdObjectHandle first = object(handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/particles");
    const OmniPvdObjectHandle second = object(handle, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/otherParticles");
    const OmniPvdObjectHandle foreign = object(peer, OVPHYSX_QUERY_TARGET_PARTICLE_SET, "/World/particles");
    const OmniPvdObjectHandle rigid = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/rigid");
    ASSERT_NE(first, 0u);
    ASSERT_NE(second, 0u);
    ASSERT_NE(foreign, 0u);
    ASSERT_NE(rigid, 0u);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.flags = 0u;
    request.nbObjects = 1;
    request.objects = &foreign;
    request.requestId = 301;
    append_capture_request(*control, request);
    request.objects = &first;
    request.requestId = 302;
    append_capture_request(*control, request);
    request.flags = 1u;
    request.requestId = 303;
    append_capture_request(*control, request);
    request.flags = 0u;
    const OmniPvdObjectHandle selectedObjects[] = { second, rigid };
    request.objects = selectedObjects;
    request.nbObjects = 2;
    request.requestId = 304;
    append_capture_request(*control, request);
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_omnipvd_recording_options_t discovery = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        discovery.record_secondary_data = false;
        const ovphysx_string_t initialRoot = OVPHYSX_LITERAL("/World/particles");
        discovery.object_paths = &initialRoot;
        discovery.object_count = 1;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination, &discovery).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!sent->load())
        {
            ovphysx_ovd_stop_recording(handle);
            return OVPHYSX_API_ERROR;
        }
        for (uint32_t step = 0; step < 6 && status == OVPHYSX_API_SUCCESS; ++step)
            status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(handle).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent));
    ASSERT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("tcp_buffers");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.captureResults, (std::vector<std::pair<uint64_t, uint32_t>>{
        {301, 4}, {302, 1}, {303, 1}, {304, 1}}));
    expectMovingPoints(summary, second, "positionInvMasses", 12);
    EXPECT_EQ(summary.currentAttributes[first].count("positionInvMasses"), 0u);
    expectParticlePrimaryState(summary, second, 12); // Request 304 is primary-only.
    EXPECT_EQ(summary.currentAttributes[first].count("velocities"), 0u);
    // Earlier selections recorded primary phases even before secondary data was enabled.
    EXPECT_EQ(summary.writtenAttributes[first].count("phases"), 1u);
    EXPECT_EQ(summary.writtenAttributes[foreign].count("positionInvMasses"), 0u);
    EXPECT_EQ(summary.currentAttributes[rigid].count("globalPose"), 1u);
}

TEST_F(OmniPvdSelectiveCapture, TcpReceiverRequestsApplyOnlyToRecordingAttachment)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    const ::physx::PxActor* first = static_cast<const ::physx::PxActor*>(
        read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    const ::physx::PxActor* second = static_cast<const ::physx::PxActor*>(
        read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube2"));
    const ::physx::PxActor* peerActor = static_cast<const ::physx::PxActor*>(
        read_physx_pointer(peer, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_NE(peerActor, nullptr);
    const OmniPvdObjectHandle foreign = reinterpret_cast<OmniPvdObjectHandle>(peerActor);
    const OmniPvdObjectHandle missing = UINT64_MAX;
    const OmniPvdObjectHandle selected = reinterpret_cast<OmniPvdObjectHandle>(first);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.flags = 0u;
    request.nbObjects = 1;
    request.requestId = 101;
    request.objects = &foreign;
    append_capture_request(*control, request);
    request.requestId = 102;
    request.objects = &missing;
    append_capture_request(*control, request);
    request.requestId = 103;
    request.objects = &selected;
    append_capture_request(*control, request);
    request.requestId = 104;
    request.objects = nullptr;
    request.nbObjects = 0;
    append_capture_request(*control, request);
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent, first, second, peerActor](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_omnipvd_recording_options_t discovery = OVPHYSX_OMNIPVD_RECORDING_OPTIONS_DEFAULT;
        discovery.record_secondary_data = false;
        const ovphysx_string_t initialRoot = OVPHYSX_LITERAL("/World/Cube1");
        discovery.object_paths = &initialRoot;
        discovery.object_count = 1;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination, &discovery).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!sent->load())
        {
            ovphysx_ovd_stop_recording(handle);
            return OVPHYSX_API_ERROR;
        }
        ::physx::PxOmniPvd* native = static_cast<::physx::PxPhysics*>(
            test_utils::read_physics_singleton(handle))->getOmniPvd();
        for (uint32_t step = 0; step < 4 && status == OVPHYSX_API_SUCCESS; ++step)
        {
            status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
        }
        // One public step can pass through multiple admitted runtime dispatchers.
        // The acknowledgement sequence below proves each request was processed;
        // inspect the final policy without depending on dispatcher count.
        EXPECT_TRUE(native->isObjectRecorded(*first));
        EXPECT_TRUE(native->isObjectRecorded(*second));
        EXPECT_FALSE(native->isObjectRecorded(*peerActor));
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(handle).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("remote");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {101, 4}, {102, 2}, {103, 1}, {104, 1} };
    EXPECT_EQ(summary.captureResults, expected);
}

TEST_F(OmniPvdSelectiveCapture, TcpReverseHalfCloseRetainsForwardRecording)
{
    const auto closed = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &closed](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination, nullptr).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!closed->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        EXPECT_TRUE(closed->load());
        for (uint32_t step = 0; step < 4 && status == OVPHYSX_API_SUCCESS; ++step)
            status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
        bool recording = false;
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(recording);
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(handle).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, nullptr, closed, true));
    EXPECT_TRUE(received.cleanEof);
    const fs::path file = path("reverse_half_close");
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    std::ofstream output(file, std::ios::binary);
    output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                 static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    output.close();
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_GE(summary.stopFrameCount, 4u);
}

TEST_F(OmniPvdSelectiveCapture, TcpMalformedControlFinalizesVehicleAndAllowsRestart)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/vehicle_read_write.usda"));
    const auto malformed = std::make_shared<std::vector<uint8_t>>(32, 0);
    const auto sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        const ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination, nullptr).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (!sent->load())
            return OVPHYSX_API_ERROR;
        for (uint32_t step = 0; step < 4; ++step)
            EXPECT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        bool recording = true;
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(recording);
        bool runtimeFlag = true;
        EXPECT_EQ(ovphysx_test_get_settings_bool("/physics/omniPvdIsRecording", &runtimeFlag).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(runtimeFlag);
        return OVPHYSX_API_SUCCESS;
    }, received, malformed, sent));
    EXPECT_TRUE(received.cleanEof);
    const fs::path file = path("after_terminal_control");
    ASSERT_EQ(start(handle, file, nullptr).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.vehicleObjectCount, 4u);
}

TEST_F(OmniPvdSelectiveCapture, TcpMalformedControlFinalizesDiscoveryAndAllowsRestart)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/vehicle_read_write.usda"));
    const OmniPvdObjectHandle vehicleActor = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Basic");
    ASSERT_NE(vehicleActor, 0u);
    const std::shared_ptr<std::vector<uint8_t>> malformed = std::make_shared<std::vector<uint8_t>>(32, 0);
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        const ovphysx_api_status_t status = start_discovery(handle, &destination, nullptr).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (!sent->load())
            return OVPHYSX_API_ERROR;
        for (uint32_t step = 0; step < 4; ++step)
            EXPECT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        bool recording = true;
        EXPECT_EQ(ovphysx_ovd_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(recording);
        bool runtimeFlag = true;
        EXPECT_EQ(ovphysx_test_get_settings_bool("/physics/omniPvdIsRecording", &runtimeFlag).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(runtimeFlag);
        return OVPHYSX_API_SUCCESS;
    }, received, malformed, sent));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path catalog = path("discovery_catalog");
    {
        std::ofstream output(catalog, std::ios::binary);
        ASSERT_TRUE(output);
        output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary discovered;
    ASSERT_TRUE(read_ovd_commands(catalog, discovered));
    EXPECT_EQ(discovered.discoveryTargets.count(vehicleActor), 1u);
    EXPECT_EQ(discovered.actorObjectCount, 0u);
    EXPECT_EQ(discovered.vehicleObjectCount, 0u);
    const fs::path file = path("after_terminal_control");
    ASSERT_EQ(start(handle, file, nullptr).status, OVPHYSX_API_SUCCESS);
    finish(handle);
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    EXPECT_EQ(summary.vehicleObjectCount, 4u);
}

TEST_F(OmniPvdSelectiveCapture, TcpVehicleResetRetiresOldEpochAndPreservesDeniedPolicy)
{
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/vehicle_read_write.usda"));
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Basic");
    const OmniPvdObjectHandle foreign = object(peer, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    ASSERT_NE(selected, 0u);
    ASSERT_NE(foreign, 0u);
    const auto control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.requestId = 501;
    request.flags = 1u;
    request.objects = &selected;
    request.nbObjects = 1;
    append_capture_request(*control, request);
    request.requestId = 502;
    append_capture_request(*control, request);
    request.requestId = 503;
    request.flags = 0;
    append_capture_request(*control, request);
    request.requestId = 504;
    request.flags = 1u;
    append_capture_request(*control, request);
    request.requestId = 505;
    request.objects = &foreign;
    append_capture_request(*control, request);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_remote_policies(handle, control, received, "/World/Basic"));
    const fs::path file = path("remote_vehicle_epochs");
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    std::ofstream output(file, std::ios::binary);
    output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                 static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    output.close();
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {501, 1}, {502, 1}, {503, 1}, {504, 1}, {505, 4} };
    EXPECT_EQ(summary.captureResults, expected);
    // Each secondary replacement recreates Vehicle diagnostics. Primary-only replacement
    // retires them, and a denied request leaves the latest diagnostics alive.
    EXPECT_EQ(summary.vehicleCreatesAtAcks, (std::vector<size_t>{1, 2, 2, 3, 3}));
    EXPECT_EQ(summary.vehicleDestroysAtAcks, (std::vector<size_t>{0, 1, 2, 2, 2}));
    EXPECT_EQ(summary.liveVehiclesAtAcks, (std::vector<size_t>{1, 1, 0, 1, 1}));
    EXPECT_EQ(summary.vehicleObjectCount, 3u);

    // Decode the primary-only epoch itself so a later full snapshot cannot hide
    // missing rigid-body motion while vehicle diagnostics are disabled.
    struct ReaderScope
    {
        OmniPvdFileReadStream* stream = createOmniPvdFileReadStream();
        OmniPvdReader* reader = createOmniPvdReader();
        ~ReaderScope()
        {
            if (reader) destroyOmniPvdReader(*reader);
            if (stream) destroyOmniPvdFileReadStream(*stream);
        }
    } input;
    ASSERT_NE(input.reader, nullptr); ASSERT_NE(input.stream, nullptr);
    input.stream->setFileName(file.string().c_str());
    input.reader->setReadStream(*input.stream);
    OmniPvdVersionType major = 0, minor = 0, patch = 0;
    ASSERT_TRUE(input.reader->startReading(major, minor, patch));
    bool pose = false, linearVelocity = false, angularVelocity = false, checkedPrimary = false;
    uint64_t requestId = 0;
    for (OmniPvdCommand::Enum command = input.reader->getNextCommand();
         command != OmniPvdCommand::eINVALID; command = input.reader->getNextCommand())
    {
        if ((command == OmniPvdCommand::eCREATE_OBJECT || command == OmniPvdCommand::eDESTROY_OBJECT) &&
            input.reader->getObjectHandle() == selected)
            pose = linearVelocity = angularVelocity = false;
        if (command != OmniPvdCommand::eSET_ATTRIBUTE)
            continue;
        const std::string& name = summary.attributeNames[input.reader->getAttributeHandle()];
        if (input.reader->getObjectHandle() == selected)
        {
            pose |= name == "globalPose";
            linearVelocity |= name == "linearVelocity";
            angularVelocity |= name == "angularVelocity";
        }
        if (name == "captureRequestId" && input.reader->getAttributeDataLength() == sizeof(requestId))
            std::memcpy(&requestId, input.reader->getAttributeDataPointer(), sizeof(requestId));
        if (name == "captureRequestStatus" && requestId == 503)
        {
            EXPECT_TRUE(pose); EXPECT_TRUE(linearVelocity); EXPECT_TRUE(angularVelocity);
            checkedPrimary = true;
        }
    }
    EXPECT_TRUE(checkedPrimary);
}

TEST_F(OmniPvdSelectiveCapture, TcpControlKeepsPeerStepsAndAttachmentTransitionsUsable)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda"));
    ovphysx_instance_handle_t third = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&third).status, OVPHYSX_API_SUCCESS);
    const OmniPvdObjectHandle selected = object(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
    const auto control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.requestId = 401;
    request.flags = 0u;
    request.objects = &selected;
    request.nbObjects = 1;
    append_capture_request(*control, request);
    const auto sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    const auto captured = capture_loopback_tcp([this, third, &sent](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        const ovphysx_api_status_t started = ovphysx_ovd_start_recording(handle, &destination, nullptr).status;
        if (started != OVPHYSX_API_SUCCESS)
            return started;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        std::atomic<bool> begin{ false };
        std::atomic<uint32_t> failures{ 0 };
        const auto stepAndFetch = [&begin, &failures](ovphysx_instance_handle_t owner) {
            while (!begin.load())
                std::this_thread::yield();
            bool reported = false;
            for (uint32_t step = 0; step < 240; ++step)
            {
                ovphysx_object_type_t type = OVPHYSX_OBJECT_TYPE_INVALID;
                const ovphysx_result_t classified =
                    ovphysx_get_object_type(owner, OVPHYSX_LITERAL("/World/Cube1"), &type);
                failures += classified.status != OVPHYSX_API_SUCCESS || type == OVPHYSX_OBJECT_TYPE_INVALID;
                const ovphysx_enqueue_result_t task = ovphysx_step(owner, 1.0f / 60.0f);
                if (task.status != OVPHYSX_API_SUCCESS)
                {
                    ++failures;
                    if (!reported)
                    {
                        const ovphysx_string_t message = ovphysx_get_last_error();
                        ADD_FAILURE() << "Peer step submission failed: " << task.status << ": "
                                      << std::string(message.ptr ? message.ptr : "", message.length);
                        reported = true;
                    }
                    continue;
                }
                ovphysx_wait_task_result_t outcome{};
                const ovphysx_api_status_t waited = ovphysx_wait_task(
                    owner, task, UINT64_C(5000000000), &outcome).status;
                failures += waited != OVPHYSX_API_SUCCESS || outcome.num_errors != 0;
                if (!reported && (waited != OVPHYSX_API_SUCCESS || outcome.num_errors != 0))
                {
                    const ovphysx_string_t message = outcome.num_errors ?
                        ovphysx_get_last_task_error(owner, outcome.error_tasks[0]) : ovphysx_get_last_error();
                    ADD_FAILURE() << "Peer fetch failed: " << waited << ", errors " << outcome.num_errors << ": "
                                  << std::string(message.ptr ? message.ptr : "", message.length);
                    reported = true;
                }
                ovphysx_destroy_wait_task_result(&outcome);
            }
        };
        std::thread ownerSteps(stepAndFetch, handle);
        std::thread peerSteps(stepAndFetch, peer);
        std::thread attachmentTransitions([third, &begin, &failures]() {
            while (!begin.load())
                std::this_thread::yield();
            for (uint32_t transition = 0; transition < 8; ++transition)
            {
                if (!attach_usd_with_ovstage(third, "tests/data/simple_physics_scene.usda"))
                    ++failures;
                if (!destroy_ovstage_test_attachments(third))
                    ++failures;
            }
        });
        begin.store(true);
        ownerSteps.join();
        peerSteps.join();
        attachmentTransitions.join();
        EXPECT_EQ(failures.load(), 0u);
        for (const ovphysx_instance_handle_t stepped : {handle, peer})
        {
            float simulationTime = 0.0f;
            float lastStepDt = 0.0f;
            EXPECT_TRUE(ovphysx_read_step_bookkeeping_for_test_internal(stepped, &simulationTime, &lastStepDt));
            EXPECT_NEAR(simulationTime, 240.0f / 60.0f, 0.0001f);
            EXPECT_FLOAT_EQ(lastStepDt, 1.0f / 60.0f);
        }
        // Every thread must remain usable after the request and any drain by a peer.
        EXPECT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(ovphysx_step_sync(peer, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        return ovphysx_ovd_stop_recording(handle).status;
    }, received, control, sent);
    EXPECT_EQ(ovphysx_destroy_instance(third).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(captured) << captured.message();
    const fs::path file = path("remote_concurrent");
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    std::ofstream output(file, std::ios::binary);
    output.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                 static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    output.close();
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {401, 1} };
    EXPECT_EQ(summary.captureResults, expected);
}

TEST_F(OmniPvdSelectiveCapture, StagelessTcpOwnerDeniesReceiverControl)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.requestId = 201;
    request.flags = 0u;
    append_capture_request(*control, request);
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, &sent](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(peer, &destination, nullptr).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        const std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sent->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (sent->load())
            status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
        else
            status = OVPHYSX_API_ERROR;
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(peer).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("stageless_remote");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {201, 4} };
    EXPECT_EQ(summary.captureResults, expected);
}

TEST_F(OmniPvdSelectiveCapture, StagelessTcpOwnerStaysDeniedAfterAttaching)
{
    ASSERT_EQ(ovphysx_create_instance(&peer).status, OVPHYSX_API_SUCCESS);
    void* physics = test_utils::read_physics_singleton(handle);
    ASSERT_NE(physics, nullptr);
    ::physx::PxOmniPvd* omniPvd = static_cast<::physx::PxPhysics*>(physics)->getOmniPvd();
    ASSERT_NE(omniPvd, nullptr);
    const std::shared_ptr<std::vector<uint8_t>> control = std::make_shared<std::vector<uint8_t>>();
    CaptureWireRequest request;
    request.requestId = 202;
    request.flags = 0u;
    append_capture_request(*control, request);
    const std::shared_ptr<std::atomic<bool>> sent = std::make_shared<std::atomic<bool>>(false);
    const std::shared_ptr<std::atomic<bool>> permit = std::make_shared<std::atomic<bool>>(false);
    TcpCaptureResult received;
    ASSERT_TRUE(capture_loopback_tcp([this, omniPvd, &sent, &permit](uint16_t port) -> ovphysx_api_status_t {
        ovphysx_omnipvd_destination_t destination{};
        destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_TCP;
        destination.tcp_address = OVPHYSX_LITERAL("127.0.0.1");
        destination.tcp_port = port;
        destination.tcp_timeout_ms = 3000;
        ovphysx_api_status_t status = ovphysx_ovd_start_recording(peer, &destination, nullptr).status;
        if (status != OVPHYSX_API_SUCCESS)
            return status;
        // The owner attaches and steps only after the start, before the receiver sends anything.
        if (!attach_usd_with_ovstage(peer, "tests/data/simple_physics_scene.usda") ||
            ovphysx_step_sync(peer, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_ovd_stop_recording(peer);
            return OVPHYSX_API_ERROR;
        }
        // Send only now. A valid empty frame wakes the reader if it already consumed the stream.
        permit->store(true);
        {
            ::physx::PxOmniPvd::ScopedExclusiveWriter lock(omniPvd);
            omniPvd->getWriter()->startFrame(1, 0);
            omniPvd->getWriter()->stopFrame(1, 0);
            omniPvd->getWriter()->getWriteStream()->flush();
        }
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        bool pending = false;
        while (std::chrono::steady_clock::now() < deadline && !pending)
        {
            pending = sent->load() && omniPvd->hasPendingCaptureRequest();
            if (!pending)
                std::this_thread::yield();
        }
        // Only the owner steps, so its own step decides the request.
        status = pending ? ovphysx_step_sync(peer, 1.0f / 60.0f).status : OVPHYSX_API_ERROR;
        const ovphysx_api_status_t stopped = ovphysx_ovd_stop_recording(peer).status;
        return status == OVPHYSX_API_SUCCESS ? stopped : status;
    }, received, control, sent, false, permit));
    EXPECT_TRUE(received.cleanEof);
    constexpr size_t handshakeSize = sizeof(uint32_t) + 2 * sizeof(uint16_t);
    ASSERT_GT(received.bytes.size(), handshakeSize);
    const fs::path file = path("stageless_attached_remote");
    {
        std::ofstream stream(file, std::ios::binary);
        ASSERT_TRUE(stream);
        stream.write(reinterpret_cast<const char*>(received.bytes.data() + handshakeSize),
                     static_cast<std::streamsize>(received.bytes.size() - handshakeSize));
    }
    OvdCommandSummary summary;
    ASSERT_TRUE(read_ovd_commands(file, summary));
    const std::vector<std::pair<uint64_t, uint32_t>> expected = { {202, 4} };
    EXPECT_EQ(summary.captureResults, expected);
}

#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
