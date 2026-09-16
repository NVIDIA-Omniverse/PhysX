// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11
 *
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 */

// End-to-end tests for OmniPVD OVD recording via ovphysx typed config.
//
// OmniPVD recording is initialized during createPhysics() which runs inside
// ovphysx_create_instance(). Config must therefore be passed via create_args.
// These tests use standalone TEST() (not PhysXTestFixture) to control the
// full instance lifecycle.

#include <gtest/gtest.h>
#include "AsyncEventManager/AsyncEventManager.h"
#include "OmniPvdFileReadStream.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include "PxPhysics.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysxTestHelpers.h"
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

// Count files matching *_rec.ovd in the given directory.
static int count_ovd_files(const std::string& dir)
{
    int count = 0;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec))
    {
        const auto name = entry.path().filename().string();
        if (name.size() > 8 && name.substr(name.size() - 8) == "_rec.ovd")
            ++count;
    }
    return count;
}

static ovphysx_api_status_t create_with_config(const ovphysx_config_entry_t* entries, uint32_t count)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = entries;
    args.config_entry_count = count;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    const ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
    if (result.status == OVPHYSX_API_SUCCESS)
    {
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    }
    return result.status;
}

static void reset_omnipvd_startup_config()
{
    destroySharedCpuInstance();
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_output_enabled(false)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL(""))).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_FILE_NAME)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL(""))).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_port(0)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_timeout_ms(0)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_recording_capable(false)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_carbonite(
        OVPHYSX_LITERAL("/physics/omniPvdIsOVDStage"), OVPHYSX_LITERAL("false"))).status,
        OVPHYSX_API_SUCCESS);
}

static ovphysx_result_t create_recording_capable_instance(ovphysx_handle_t& handle)
{
    const ovphysx_config_entry_t config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL("")),
        ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_FILE_NAME),
        ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("")),
        ovphysx_config_entry_omnipvd_tcp_port(0),
        ovphysx_config_entry_omnipvd_tcp_timeout_ms(0),
        ovphysx_config_entry_omnipvd_recording_capable(true),
        ovphysx_config_entry_omnipvd_output_enabled(false),
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/omniPvdIsOVDStage"), OVPHYSX_LITERAL("false")),
    };
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = config;
    args.config_entry_count = static_cast<uint32_t>(std::size(config));
    return ovphysx_create_instance(&args, &handle);
}

static ovphysx_result_t start_file_recording(ovphysx_handle_t handle, const fs::path& path)
{
    const std::string pathString = path.string();
    const ovphysx_omnipvd_destination_t destination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE,
        make_ovx_string(pathString.c_str()),
        OVPHYSX_LITERAL(""),
        0,
        0,
    };
    return ovphysx_start_recording(handle, &destination);
}

static ::testing::AssertionResult create_stepped_recording_instance(ovphysx_handle_t& handle)
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
            summary.stopFrameCount += command == OmniPvdCommand::eSTOP_FRAME;
            summary.hasVehicleSchema |= command == OmniPvdCommand::eREGISTER_CLASS &&
                                        std::strcmp(reader->getClassName(), "DirectDrivetrain") == 0;
            if (command == OmniPvdCommand::eREGISTER_CLASS)
            {
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
                for (uint32_t i = 0; i < static_cast<uint32_t>(std::size(kPhysXExtensionsClassNames)); ++i)
                {
                    if (summary.physXExtensionsClassHandles[i] != OMNI_PVD_INVALID_HANDLE &&
                        reader->getClassHandle() == summary.physXExtensionsClassHandles[i])
                    {
                        summary.physXExtensionsObjectMask |= 1u << i;
                    }
                }
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
    if (teardown && (summary.destroyObjectCount == 0 || summary.lastCommand != OmniPvdCommand::eDESTROY_OBJECT))
        return ::testing::AssertionFailure() << "OVD teardown did not end with DESTROY_OBJECT: " << path;
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

// Create an instance, load USD, step, destroy. Returns the number of
// *_rec.ovd files found in output_dir after destruction.
static ::testing::AssertionResult run_recording_workflow(
    const std::string& output_dir,
    bool enable_pvd,
    bool output_enabled_first,
    int& out_ovd_count,
    bool output_enabled_as_carbonite = false)
{
    reset_omnipvd_startup_config();
    out_ovd_count = 0;

    ovphysx_config_entry_t output_enabled_entry;
    if (output_enabled_as_carbonite)
    {
        // The logical view ends before '!' to verify raw keys need not be null-terminated.
        static constexpr char output_enabled_path_storage[] = "/physics/omniPvdOutputEnabled!";
        const ovphysx_string_t output_enabled_path = {
            output_enabled_path_storage,
            sizeof(output_enabled_path_storage) - 2,
        };
        const ovphysx_string_t value =
            enable_pvd ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false");
        output_enabled_entry = ovphysx_config_entry_carbonite(
            output_enabled_path, value);
    }
    else
    {
        output_enabled_entry = ovphysx_config_entry_omnipvd_output_enabled(enable_pvd);
    }

    // Configure OmniPVD via config entries passed at instance creation.
    ovphysx_config_entry_t config[2];
    if (output_enabled_first)
    {
        config[0] = output_enabled_entry;
        config[1] = ovphysx_config_entry_omnipvd_ovd_recording_directory(make_ovx_string(output_dir.c_str()));
    }
    else
    {
        config[0] = ovphysx_config_entry_omnipvd_ovd_recording_directory(make_ovx_string(output_dir.c_str()));
        config[1] = output_enabled_entry;
    }

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = config;
    args.config_entry_count = 2;
    ovphysx_handle_t handle = 0;
    ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
    if (r.status != OVPHYSX_API_SUCCESS || handle == 0)
        return ::testing::AssertionFailure() << "Failed to create PhysX instance";

    if (!attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"))
    {
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Failed to load USD scene";
    }

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i)
    {
        r = ovphysx_step_sync(handle, dt);
        if (r.status != OVPHYSX_API_SUCCESS)
        {
            destroy_ovstage_test_attachments(handle);
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Simulation step " << i << " failed";
        }
    }

    // Destroying the instance triggers writeOutOmniPVDFile(), which renames
    // tmp.ovd to <timestamp>_rec.ovd.
    destroy_ovstage_test_attachments(handle);
    ovphysx_destroy_instance(handle);

    out_ovd_count = count_ovd_files(output_dir);
    return ::testing::AssertionSuccess();
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(OmniPvdStartupConfig, InvalidTcpTupleIsRejected)
{
    reset_omnipvd_startup_config();
    const ovphysx_config_entry_t invalid_configs[][4] = {
        { ovphysx_config_entry_omnipvd_transport(OVPHYSX_LITERAL("unknown")) },
        { ovphysx_config_entry_omnipvd_transport(OVPHYSX_LITERAL("")) },
        { ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME),
          ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("")),
          ovphysx_config_entry_omnipvd_tcp_port(5425) },
        { ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME),
          ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("127.0.0.1")),
          ovphysx_config_entry_omnipvd_tcp_port(0) },
        { ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME),
          ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("127.0.0.1")),
          ovphysx_config_entry_omnipvd_tcp_port(65536) },
        { ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME),
          ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("127.0.0.1")),
          ovphysx_config_entry_omnipvd_tcp_port(5425),
          ovphysx_config_entry_omnipvd_tcp_timeout_ms(-1) },
    };
    const uint32_t counts[] = { 1, 1, 3, 3, 3, 4 };
    for (size_t i = 0; i < std::size(invalid_configs); ++i)
    {
        SCOPED_TRACE(i);
        EXPECT_EQ(create_with_config(invalid_configs[i], counts[i]), OVPHYSX_API_INVALID_ARGUMENT);
        reset_omnipvd_startup_config();
    }
}

TEST(OmniPvdStartupConfig, GlobalTupleIsAlwaysValidated)
{
    reset_omnipvd_startup_config();
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("127.0.0.1"))).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_omnipvd_tcp_port(5425)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(create_with_config(nullptr, 0), OVPHYSX_API_SUCCESS);

    reset_omnipvd_startup_config();
    ASSERT_EQ(ovphysx_set_global_config(
        ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(create_with_config(nullptr, 0), OVPHYSX_API_INVALID_ARGUMENT);
    reset_omnipvd_startup_config();
}

TEST(OmniPvdStartupConfig, SecondCreateCannotApplyRecordingKeys)
{
    ovphysx_handle_t first_handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(create_recording_capable_instance(first_handle).status, OVPHYSX_API_SUCCESS);

    const ovphysx_config_entry_t second_configs[] = {
        ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_FILE_NAME),
        ovphysx_config_entry_omnipvd_recording_capable(true),
    };
    for (const ovphysx_config_entry_t& second_config : second_configs)
        EXPECT_EQ(create_with_config(&second_config, 1), OVPHYSX_API_ERROR);
    ASSERT_EQ(ovphysx_destroy_instance(first_handle).status, OVPHYSX_API_SUCCESS);
    reset_omnipvd_startup_config();
}

TEST(OmniPvdStartupConfig, RejectsEmbeddedNullAliases)
{
    reset_omnipvd_startup_config();
    static constexpr char transport_value[] = { 't', 'c', 'p', '\0', 'x' };
    static constexpr char directory_value[] = { 'a', '\0', 'b' };
    static constexpr char raw_key[] = "/physics/omniPvdTcpPort\0alias";
    static constexpr char raw_value[] = { '5', '4', '2', '5', '\0', 'x' };
    const ovphysx_config_entry_t configs[] = {
        ovphysx_config_entry_omnipvd_transport({ transport_value, sizeof(transport_value) }),
        ovphysx_config_entry_omnipvd_ovd_recording_directory({ directory_value, sizeof(directory_value) }),
        ovphysx_config_entry_carbonite({ raw_key, sizeof(raw_key) - 1 }, OVPHYSX_LITERAL("5425")),
        ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/omniPvdTcpPort"),
                                      { raw_value, sizeof(raw_value) }),
    };
    for (const ovphysx_config_entry_t& config : configs)
        EXPECT_EQ(create_with_config(&config, 1), OVPHYSX_API_INVALID_ARGUMENT);
    reset_omnipvd_startup_config();
}

TEST(OmniPvdStartupConfig, PreloadedEmbeddedNullDestinationIsRejected)
{
    reset_omnipvd_startup_config();
    static constexpr char malformedTransport[] = { 'f', 'i', 'l', 'e', '\0', 'x' };
    ASSERT_TRUE(ovphysx_set_raw_string_setting_for_test_internal(
        "/physics/omniPvdTransport", malformedTransport, sizeof(malformedTransport)));
    EXPECT_EQ(create_with_config(nullptr, 0), OVPHYSX_API_INVALID_ARGUMENT);
    reset_omnipvd_startup_config();
}

TEST(OmniPvdStartupConfig, RawTcpNumericsRequireExactInt32)
{
    reset_omnipvd_startup_config();
    const ovphysx_config_entry_t configs[] = {
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/omniPvdTcpPort"), OVPHYSX_LITERAL("1.5")),
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/omniPvdTcpTimeoutMs"), OVPHYSX_LITERAL("not-a-number")),
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/omniPvdTcpPort"), OVPHYSX_LITERAL("2147483648")),
    };
    for (const ovphysx_config_entry_t& config : configs)
        EXPECT_EQ(create_with_config(&config, 1), OVPHYSX_API_INVALID_ARGUMENT);
    reset_omnipvd_startup_config();
}

TEST(OmniPvdColdCreation, DefaultInstanceHasNoProviderAndRejectsLateStart)
{
    const fs::path capturePath =
        fs::temp_directory_path() / "ovphysx_default_must_not_record.ovd";
    std::error_code ec;
    fs::remove(capturePath, ec);
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ASSERT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_SUCCESS);

    ovphysx_result_t startResult = start_file_recording(handle, capturePath);
    EXPECT_EQ(startResult.status, OVPHYSX_API_INVALID_STATE);
    const ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("omnipvd_recording_capable"), std::string::npos);

    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* physicsPtr = nullptr;
    ASSERT_EQ(ovphysx_get_physx_ptr(
        handle, { nullptr, 0 }, OVPHYSX_PHYSX_TYPE_PHYSICS, &physicsPtr).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_NE(physicsPtr, nullptr);
    EXPECT_EQ(static_cast<physx::PxPhysics*>(physicsPtr)->getOmniPvd(), nullptr);

    startResult = start_file_recording(handle, capturePath);
    EXPECT_EQ(startResult.status, OVPHYSX_API_INVALID_STATE);
    const ovphysx_string_t attachedError = ovphysx_get_last_error();
    EXPECT_NE(
        std::string(attachedError.ptr, attachedError.length).find("omnipvd_recording_capable"), std::string::npos);

    destroy_ovstage_test_attachments(handle);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    fs::remove(capturePath, ec);
}

TEST(OmniPvdColdCreation, OvdPlaybackOutputDoesNotInstallProvider)
{
    const ovphysx_config_entry_t config[] = {
        ovphysx_config_entry_omnipvd_output_enabled(true),
        ovphysx_config_entry_omnipvd_recording_capable(false),
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/omniPvdIsOVDStage"), OVPHYSX_LITERAL("true")),
    };
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = config;
    args.config_entry_count = static_cast<uint32_t>(std::size(config));
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    void* physicsPtr = nullptr;
    ASSERT_EQ(ovphysx_get_physx_ptr(
        handle, { nullptr, 0 }, OVPHYSX_PHYSX_TYPE_PHYSICS, &physicsPtr).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_NE(physicsPtr, nullptr);
    EXPECT_EQ(static_cast<physx::PxPhysics*>(physicsPtr)->getOmniPvd(), nullptr);

    destroy_ovstage_test_attachments(handle);
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
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

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(handle));
    bool recording = true;
    EXPECT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    EXPECT_EQ(ovphysx_stop_recording(handle).status, OVPHYSX_API_INVALID_STATE);

    const ovphysx_omnipvd_destination_t invalidDestination = {
        OVPHYSX_OMNIPVD_TRANSPORT_FILE,
        OVPHYSX_LITERAL(""),
        OVPHYSX_LITERAL(""),
        0,
        0,
    };
    EXPECT_EQ(ovphysx_start_recording(handle, &invalidDestination).status, OVPHYSX_API_INVALID_ARGUMENT);

    ASSERT_EQ(start_file_recording(handle, firstPath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(start_file_recording(handle, retargetPath).status, OVPHYSX_API_INVALID_STATE);
    ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("already active"), std::string::npos);
    EXPECT_FALSE(fs::exists(retargetPath));

    ovphysx_create_args peerArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t peerHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&peerArgs, &peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(start_file_recording(peerHandle, retargetPath).status, OVPHYSX_API_INVALID_STATE);
    error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("already active"), std::string::npos);
    bool peerRecording = true;
    EXPECT_EQ(ovphysx_is_recording(peerHandle, &peerRecording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(peerRecording);
    EXPECT_EQ(ovphysx_stop_recording(peerHandle).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(ovphysx_destroy_instance(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(recording);
    EXPECT_FALSE(fs::exists(retargetPath));

    ASSERT_EQ(ovphysx_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    ASSERT_TRUE(fs::exists(firstPath));
    EXPECT_TRUE(has_physics_commands(firstPath));
    EXPECT_EQ(ovphysx_start_recording(handle, &invalidDestination).status, OVPHYSX_API_INVALID_ARGUMENT);
    peerHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&peerArgs, &peerHandle).status, OVPHYSX_API_SUCCESS);

    const size_t eventBaseline = async_get_active_event_count();
    const ovphysx_enqueue_result_t startStep = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(startStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(async_get_active_event_count(), eventBaseline + 1);
    ASSERT_EQ(start_file_recording(peerHandle, retargetPath).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(async_get_active_event_count(), eventBaseline);
    EXPECT_TRUE(waitForOperationSuccess(handle, startStep.op_index, 0));
    peerRecording = false;
    ASSERT_EQ(ovphysx_is_recording(peerHandle, &peerRecording).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(peerRecording);
    recording = true;
    ASSERT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);

    const ovphysx_enqueue_result_t stopStep = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(stopStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(async_get_active_event_count(), eventBaseline + 1);
    ASSERT_EQ(ovphysx_stop_recording(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(async_get_active_event_count(), eventBaseline);
    EXPECT_TRUE(waitForOperationSuccess(handle, stopStep.op_index, 0));
    ASSERT_TRUE(fs::exists(retargetPath));
    EXPECT_TRUE(has_physics_commands(retargetPath));

    ASSERT_EQ(start_file_recording(peerHandle, destroyPath).status, OVPHYSX_API_SUCCESS);
    const ovphysx_enqueue_result_t destroyStep = ovphysx_step(handle, 1.0f / 60.0f);
    ASSERT_EQ(destroyStep.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(async_get_active_event_count(), eventBaseline + 1);
    ASSERT_EQ(ovphysx_destroy_instance(peerHandle).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForOperationSuccess(handle, destroyStep.op_index, 0));
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

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(create_recording_capable_instance(handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(start_file_recording(handle, beforeDetachPath).status, OVPHYSX_API_INVALID_STATE);
    ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("attach and initialize"), std::string::npos);
    EXPECT_FALSE(fs::exists(beforeDetachPath));

    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ovphysx_create_args peerArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t peerHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_instance(&peerArgs, &peerHandle).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(start_file_recording(peerHandle, beforeDetachPath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(destroy_ovstage_test_attachments(handle));
    ASSERT_TRUE(fs::exists(beforeDetachPath));
    EXPECT_TRUE(has_physics_commands(beforeDetachPath, true));

    bool recording = true;
    ASSERT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    recording = true;
    ASSERT_EQ(ovphysx_is_recording(peerHandle, &recording).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(recording);
    ASSERT_EQ(start_file_recording(handle, whileDetachedPath).status, OVPHYSX_API_INVALID_STATE);
    error = ovphysx_get_last_error();
    EXPECT_NE(std::string(error.ptr, error.length).find("attach and initialize"), std::string::npos);
    EXPECT_FALSE(fs::exists(whileDetachedPath));
    ASSERT_TRUE(attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"));
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    const ovphysx_api_status_t restartStatus = start_file_recording(handle, afterReattachPath).status;
    EXPECT_EQ(restartStatus, OVPHYSX_API_SUCCESS);
    if (restartStatus == OVPHYSX_API_SUCCESS)
    {
        EXPECT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        recording = true;
        EXPECT_EQ(ovphysx_is_recording(peerHandle, &recording).status, OVPHYSX_API_SUCCESS);
        EXPECT_FALSE(recording);
        EXPECT_EQ(ovphysx_stop_recording(peerHandle).status, OVPHYSX_API_INVALID_STATE);
        EXPECT_EQ(ovphysx_stop_recording(handle).status, OVPHYSX_API_SUCCESS);
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

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(handle));
    EXPECT_EQ(start_file_recording(handle, directory).status, OVPHYSX_API_ERROR);
    bool recording = true;
    ASSERT_EQ(ovphysx_is_recording(handle, &recording).status, OVPHYSX_API_SUCCESS);
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
            EXPECT_EQ(ovphysx_start_recording(handle, &tcpDestination).status, OVPHYSX_API_ERROR);
            bool tcpRecording = true;
            EXPECT_EQ(ovphysx_is_recording(handle, &tcpRecording).status, OVPHYSX_API_SUCCESS);
            EXPECT_FALSE(tcpRecording);
        });
    ASSERT_TRUE(refusedTcp) << refusedTcp.message();

    ASSERT_EQ(start_file_recording(handle, capturePath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
    destroy_ovstage_test_stages(handle);
    ASSERT_TRUE(fs::exists(capturePath));
    EXPECT_TRUE(has_physics_commands(capturePath, true));

    ovphysx_handle_t nextHandle = OVPHYSX_INVALID_HANDLE;
    ASSERT_TRUE(create_stepped_recording_instance(nextHandle));
    ASSERT_EQ(start_file_recording(nextHandle, nextCapturePath).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(nextHandle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_stop_recording(nextHandle).status, OVPHYSX_API_SUCCESS);
    destroy_ovstage_test_attachments(nextHandle);
    ASSERT_EQ(ovphysx_destroy_instance(nextHandle).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(fs::exists(nextCapturePath));
    EXPECT_TRUE(has_physics_commands(nextCapturePath));
    fs::remove_all(directory, ec);
}

// Positive path: a recording directory plus enable produces at least one non-empty .ovd file.
TEST(OmniPvdRecording, OvdFileProducedWhenEnabled)
{
    auto tmp_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_enabled";
    fs::create_directories(tmp_dir);
    // Clean up any leftover files from previous runs.
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
        fs::remove(entry.path());

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(
        tmp_dir.string(), /*enable_pvd=*/true, /*output_enabled_first=*/false, ovd_count));
    EXPECT_GE(ovd_count, 1)
        << "Expected at least one *_rec.ovd file in " << tmp_dir;

    // Every produced file must contain physics commands.
    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
    {
        const auto name = entry.path().filename().string();
        if (name.size() > 8 && name.substr(name.size() - 8) == "_rec.ovd")
        {
            EXPECT_TRUE(has_physics_commands(entry.path(), false, true));
        }
    }

    fs::remove_all(tmp_dir, ec);
}

TEST(OmniPvdRecording, StartupTcpCanStopAndRestartToLateFile)
{
    const fs::path directory = fs::temp_directory_path() / "ovphysx_pvd_startup_tcp_late_file";
    const fs::path tcpCapturePath = directory / "startup_tcp.ovd";
    const fs::path capturePath = directory / "after_startup_tcp.ovd";
    const fs::path restartedCapturePath = directory / "after_late_file_restart.ovd";
    std::error_code ec;
    fs::remove_all(directory, ec);
    fs::create_directories(directory, ec);
    ASSERT_FALSE(ec);
    // Keep this transition test on one TCP session. The source Vehicle fixture uses
    // centimetres, which legitimately rebuilds PxPhysics (and reconnects startup TCP)
    // when it is attached after the default metre-scale bootstrap.
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
            ovphysx_config_entry_t config[] = {
                ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL("")),
                ovphysx_config_entry_omnipvd_transport(OVPHYSX_OMNIPVD_TRANSPORT_TCP_NAME),
                ovphysx_config_entry_omnipvd_tcp_address(OVPHYSX_LITERAL("127.0.0.1")),
                ovphysx_config_entry_omnipvd_tcp_port(port),
                ovphysx_config_entry_omnipvd_tcp_timeout_ms(3000),
                ovphysx_config_entry_omnipvd_recording_capable(true),
                ovphysx_config_entry_omnipvd_output_enabled(true),
                ovphysx_config_entry_carbonite(
                    OVPHYSX_LITERAL("/physics/omniPvdIsOVDStage"), OVPHYSX_LITERAL("false")),
            };
            ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
            args.config_entries = config;
            args.config_entry_count = static_cast<uint32_t>(std::size(config));
            ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
            const ovphysx_result_t created = ovphysx_create_instance(&args, &handle);
            if (created.status != OVPHYSX_API_SUCCESS)
                return created.status;

            if (!attach_usd_with_ovstage(handle, vehiclePathString.c_str()))
            {
                ovphysx_destroy_instance(handle);
                return OVPHYSX_API_ERROR;
            }
            ovphysx_api_status_t status = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
            void* extensionsFixture = nullptr;
            if (status == OVPHYSX_API_SUCCESS &&
                !ovphysx_create_extensions_fixture_for_test_internal(handle, &extensionsFixture))
            {
                status = OVPHYSX_API_ERROR;
            }
            bool recording = false;
            if (status == OVPHYSX_API_SUCCESS)
                status = ovphysx_is_recording(handle, &recording).status;
            EXPECT_TRUE(recording);
            if (status == OVPHYSX_API_SUCCESS && !recording)
                status = OVPHYSX_API_INVALID_STATE;

            const ovphysx_api_status_t stopStatus = ovphysx_stop_recording(handle).status;
            EXPECT_EQ(stopStatus, OVPHYSX_API_SUCCESS);
            if (status == OVPHYSX_API_SUCCESS)
                status = stopStatus;

            const auto recordToFile = [handle](const fs::path& path, uint32_t stepCount) -> ovphysx_api_status_t {
                ovphysx_api_status_t fileStatus = start_file_recording(handle, path).status;
                for (uint32_t i = 0; fileStatus == OVPHYSX_API_SUCCESS && i < stepCount; ++i)
                    fileStatus = ovphysx_step_sync(handle, 1.0f / 60.0f).status;
                if (fileStatus == OVPHYSX_API_SUCCESS)
                    fileStatus = ovphysx_stop_recording(handle).status;
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

// Entry order must remain irrelevant after a previous instance has started the
// process-wide runtime and installed the OmniPVD output-setting subscription.
TEST(OmniPvdRecording, OvdFileProducedWhenOutputEntryComesFirstOnWarmRuntime)
{
    reset_omnipvd_startup_config();
    // Establish a known disabled/empty state while also ensuring the retained
    // process-wide runtime is already active before the order-under-test create.
    ovphysx_config_entry_t primer_config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL("")),
        ovphysx_config_entry_omnipvd_output_enabled(false),
    };
    ovphysx_create_args primer_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    primer_args.config_entries = primer_config;
    primer_args.config_entry_count = 2;
    ovphysx_handle_t primer_handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&primer_args, &primer_handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(primer_handle, 0u);
    ASSERT_EQ(ovphysx_destroy_instance(primer_handle).status, OVPHYSX_API_SUCCESS);

    auto tmp_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_output_first";
    std::error_code ec;
    fs::remove_all(tmp_dir, ec);
    fs::create_directories(tmp_dir, ec);
    ASSERT_FALSE(ec) << "Failed to create test directory: " << tmp_dir;

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(
        tmp_dir.string(), /*enable_pvd=*/true, /*output_enabled_first=*/true, ovd_count));
    EXPECT_GE(ovd_count, 1)
        << "Config entry order must not affect OmniPVD recording in " << tmp_dir;

    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
    {
        const auto name = entry.path().filename().string();
        if (name.size() > 8 && name.substr(name.size() - 8) == "_rec.ovd")
        {
            EXPECT_TRUE(has_physics_commands(entry.path(), false, true));
        }
    }

    fs::remove_all(tmp_dir, ec);
}

TEST(OmniPvdRecording, OvdFileProducedWhenRawOutputEntryComesFirstOnWarmRuntime)
{
    reset_omnipvd_startup_config();
    ovphysx_config_entry_t primer_config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL("")),
        ovphysx_config_entry_omnipvd_output_enabled(false),
    };
    ovphysx_create_args primer_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    primer_args.config_entries = primer_config;
    primer_args.config_entry_count = 2;
    ovphysx_handle_t primer_handle = 0;
    ASSERT_EQ(ovphysx_create_instance(&primer_args, &primer_handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(primer_handle, 0u);
    ASSERT_EQ(ovphysx_destroy_instance(primer_handle).status, OVPHYSX_API_SUCCESS);

    auto tmp_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_raw_output_first";
    std::error_code ec;
    fs::remove_all(tmp_dir, ec);
    fs::create_directories(tmp_dir, ec);
    ASSERT_FALSE(ec) << "Failed to create test directory: " << tmp_dir;

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(
        tmp_dir.string(),
        /*enable_pvd=*/true,
        /*output_enabled_first=*/true,
        ovd_count,
        /*output_enabled_as_carbonite=*/true));
    EXPECT_GE(ovd_count, 1)
        << "Raw Carbonite config entry order must not affect OmniPVD recording in " << tmp_dir;

    fs::remove_all(tmp_dir, ec);
}

// Negative path: recording disabled produces no .ovd files.
TEST(OmniPvdRecording, NoOvdFileWhenDisabled)
{
    auto tmp_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_disabled";
    fs::create_directories(tmp_dir);
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
        fs::remove(entry.path());

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(
        tmp_dir.string(), /*enable_pvd=*/false, /*output_enabled_first=*/false, ovd_count));
    EXPECT_EQ(ovd_count, 0)
        << "Expected no *_rec.ovd files when recording is disabled";

    fs::remove_all(tmp_dir, ec);
}

// Negative path: enable with an empty directory string must not crash and produces no .ovd file.
TEST(OmniPvdRecording, EmptyDirectoryDoesNotCrash)
{
    reset_omnipvd_startup_config();
    ovphysx_config_entry_t config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL("")),
        ovphysx_config_entry_omnipvd_output_enabled(true),
    };

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = config;
    args.config_entry_count = 2;
    ovphysx_handle_t handle = 0;
    ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
    // Instance creation should succeed even with misconfigured OmniPVD.
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_step_sync(handle, 1.0f / 60.0f);

    ovphysx_destroy_instance(handle);
    // Reaching this point without a crash is the pass condition.
}

// Directory auto-creation: a non-existent nested path is created by the runtime.
TEST(OmniPvdRecording, DirectoryAutoCreated)
{
    auto base_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_autocreate";
    auto nested_dir = base_dir / "sub" / "recordings";

    // Ensure the directory does NOT exist before the test.
    std::error_code ec;
    fs::remove_all(base_dir, ec);
    ASSERT_FALSE(fs::exists(nested_dir));

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(
        nested_dir.string(), /*enable_pvd=*/true, /*output_enabled_first=*/false, ovd_count));

    EXPECT_TRUE(fs::is_directory(nested_dir))
        << "Expected the runtime to auto-create " << nested_dir;
    EXPECT_GE(ovd_count, 1)
        << "Expected at least one *_rec.ovd file in auto-created directory";

    fs::remove_all(base_dir, ec);
}
