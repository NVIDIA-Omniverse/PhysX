// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
 * @covers AC-2 AC-3
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
#include "PxPhysics.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysxTestHelpers.h"
#include "ovphysx_test_utils.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
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
    // Each OmniPvdRecording case has a fresh process. Later instances in the
    // same case use the already-latched capability without trying to change it.
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
    const std::string pathString = path.string();
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE,
        make_ovx_string(pathString.c_str()),
        OVPHYSX_LITERAL(""),
        0,
        0,
    };
    return ovphysx_ovd_start_recording(handle, &destination);
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
    std::set<OmniPvdClassHandle> sceneClasses;
    std::set<OmniPvdClassHandle> actorClasses;
    std::set<std::pair<OmniPvdContextHandle, OmniPvdObjectHandle>> liveSceneObjects;
    std::vector<uint64_t> startFrameIds;
    std::vector<uint64_t> stopFrameIds;
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
            summary.createObjectCount += command == OmniPvdCommand::eCREATE_OBJECT;
            summary.destroyObjectCount += command == OmniPvdCommand::eDESTROY_OBJECT;
            if (command == OmniPvdCommand::eSTART_FRAME)
                summary.startFrameIds.push_back(reader->getFrameTimeStart());
            if (command == OmniPvdCommand::eSTOP_FRAME)
                summary.stopFrameIds.push_back(reader->getFrameTimeStop());
            summary.stopFrameCount += command == OmniPvdCommand::eSTOP_FRAME;
            summary.hasVehicleSchema |= command == OmniPvdCommand::eREGISTER_CLASS &&
                                        std::strcmp(reader->getClassName(), "DirectDrivetrain") == 0;
            if (command == OmniPvdCommand::eREGISTER_CLASS)
            {
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
            summary.hasVehicleObject = summary.vehicleObjectCount != 0;
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
                summary.liveSceneObjects.erase({reader->getContextHandle(), reader->getObjectHandle()});
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
static ::testing::AssertionResult capture_loopback_tcp(Producer&& producer, TcpCaptureResult& captureResult)
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
    };
    const std::shared_ptr<ReaderState> readerState = std::make_shared<ReaderState>();
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
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "OVD_RECORDING_CAPABLE"));
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
    EXPECT_EQ(ovphysx_ovd_start_recording(handle, &invalidDestination).status, OVPHYSX_API_INVALID_ARGUMENT);

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
    EXPECT_EQ(ovphysx_ovd_start_recording(handle, &invalidDestination).status, OVPHYSX_API_INVALID_ARGUMENT);
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

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(handle));
    EXPECT_EQ(start_file_recording(handle, directory).status, OVPHYSX_API_ERROR);
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
            EXPECT_EQ(ovphysx_ovd_start_recording(handle, &tcpDestination).status, OVPHYSX_API_ERROR);
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
            ovphysx_api_status_t status = ovphysx_ovd_start_recording(handle, &destination).status;
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
    EXPECT_EQ(ovphysx_ovd_start_recording(instance, nullptr).status,
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
        const std::string pathString = path.string();
        const ovphysx_omnipvd_destination_t destination = {
            OVPHYSX_OMNIPVD_TRANSPORT_FILE, make_ovx_string(pathString.c_str()), OVPHYSX_LITERAL(""), 0, 0,
        };
        ASSERT_EQ((legacy ? ovphysx_start_recording(handle, &destination) :
                            ovphysx_ovd_start_recording(handle, &destination)).status,
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

#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
