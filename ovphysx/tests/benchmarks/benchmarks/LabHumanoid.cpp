// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the tensorbinding rows measure the binding. They retire with it.

/**
 * @implements REQ-CAPI-BENCHMARK-010
 * @covers AC-2 AC-3 AC-4
 */

// IsaacLab's Humanoid task over 8192 environments on DirectGPU, through the session read and write API and through the
// deprecated tensor binding: the same work on the same data, held in device memory on both sides.
//
//   control_step -> IsaacLab's control step. A stand-in policy copies one of two device action buffers into its
//                   output buffer. Then, twice (decimation 2), the force is written from it and the scene steps. Then
//                   the root position, orientation and velocities and the joint positions and velocities are read.
//                   The physics steps are in the number.
//                   _session: held queries, a write session per substep filled from the policy's output, and a read
//                   per query.
//                   _session_captured: the same, with the fill and commit, and the reads, captured once into CUDA
//                   graphs and replayed, the write session and the reads kept open. The steps stay outside the graphs.
//                   _tensorbinding: bindings built once, the force written straight from the policy's output.
//   reset        -> IsaacLab's reset of the environments whose episode ended: the root and joint state of a rotating
//                   sixteenth of the environments (512) is written back to the initial state, held on the device. The
//                   session commits with a row mask, the binding writes masked. One untimed step between samples.
//
// The action gives every DOF of an environment the same force, so both APIs drive the same motion whatever the order
// of their rows. Every sample ends with the device idle, so device work counts, and each row prints its phase split.
// Setup is untimed: load, clone, a settling second, queries, bindings, sessions, captures, and a validation that
// throws, failing the process before any row publishes.
//
//   ovphysx_benchmarks --forceGpu --directGpu --hidden --filter=Lab.humanoid_* --data=tests/data

#include "framework/UsdPCH.h"

#include "../TaskCompletion.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include "ovphysx/dlpack/dlpack.h"
#include "ovphysx/experimental/TensorBinding.hpp"
#include <ovphysx/experimental/ovphysx.hpp>
#include <ovphysx/ovphysx_attributes.h>
#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/path_dictionary/path_dictionary_utils.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
#include <cuda_runtime_api.h>
#endif


void initLabHumanoid()
{
}

namespace
{

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)

constexpr uint32_t kEnvCount = 8192;
constexpr uint32_t kDofCount = 21;
constexpr uint32_t kBodyCount = 16;
constexpr uint32_t kGridWidth = 128;
constexpr float kEnvSpacing = 5.0f;
constexpr float kPhysicsDt = 1.0f / 120.0f;
constexpr uint32_t kDecimation = 2;
// A second of simulation before anything is timed, so the runs start with the robots down rather than falling.
constexpr uint32_t kSettleSteps = 120;
// The force the action gives every DOF of an environment, its sign alternating between environments and samples.
constexpr float kActionForce = 40.0f;
constexpr uint32_t kResetPhases = 16;
constexpr int32_t kCudaDevice = 0;
constexpr const char* kFixture = "/../benchmarks/data/humanoid_envs_gpu.usda";
// The articulation roots, which the bindings name.
constexpr const char* kRootPattern = "/World/envs/env_*/Robot/torso";
constexpr std::array<const char*, 15> kJointNames = { "head",           "lower_waist", "abdomen_x",  "right_thigh",
                                                      "right_knee",     "right_foot",  "left_thigh", "left_knee",
                                                      "left_foot",      "right_upper_arm", "right_elbow", "right_hand",
                                                      "left_upper_arm", "left_elbow",  "left_hand" };
constexpr std::array<const char*, 4> kRootAttributes = { OVPHYSX_ATTR_ROOT_POSITION, OVPHYSX_ATTR_ROOT_ORIENTATION,
                                                         OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY,
                                                         OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY };
constexpr std::array<const char*, 2> kJointAttributes = { OVPHYSX_ATTR_JOINT_POSITION, OVPHYSX_ATTR_JOINT_VELOCITY };
// What a control step reads per environment: root position, orientation, linear and angular velocity, then joint
// positions and velocities.
constexpr uint32_t kFloatsReadPerEnv = 3 + 4 + 3 + 3 + 2 * kDofCount;

std::string envPath(uint32_t env)
{
    return "/World/envs/env_" + std::to_string(env);
}

std::string lastError()
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    return error.ptr && error.length ? std::string(error.ptr, error.length) : std::string();
}

void requireSuccess(ovphysx_api_status_t status, const char* operation)
{
    if (status == OVPHYSX_API_SUCCESS)
        return;
    const std::string detail = lastError();
    throw std::runtime_error(std::string("Humanoid: ") + operation + " failed with status " +
                             std::to_string(static_cast<int>(status)) + (detail.empty() ? "" : ": " + detail));
}

void requireSuccess(const ovphysx_result_t& result, const char* operation)
{
    requireSuccess(result.status, operation);
}

void requireCuda(cudaError_t status, const char* operation)
{
    if (status != cudaSuccess)
        throw std::runtime_error(std::string("Humanoid: ") + operation + " failed: " + cudaGetErrorString(status));
}

void require(bool condition, const std::string& what)
{
    if (!condition)
        throw std::runtime_error("Humanoid: " + what);
}

double nowMs()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

ovx_token_or_string_t attributeName(const char* name)
{
    return ovx_token_or_string_t{ 0, { name, std::strlen(name) } };
}

template <size_t N>
std::vector<ovx_token_or_string_t> attributeNames(const std::array<const char*, N>& names)
{
    std::vector<ovx_token_or_string_t> out;
    for (const char* name : names)
        out.push_back(attributeName(name));
    return out;
}

// The environment a prim path names, from its /World/envs/env_<n>/ prefix.
uint32_t envOfPath(const std::string& path)
{
    const std::string prefix = "/World/envs/env_";
    require(path.compare(0, prefix.size(), prefix) == 0, "a row outside the environments: " + path);
    return static_cast<uint32_t>(std::stoul(path.substr(prefix.size())));
}

// Device memory a row owns.
class DeviceBuffer
{
public:
    DeviceBuffer() = default;
    DeviceBuffer(const DeviceBuffer&) = delete;
    DeviceBuffer& operator=(const DeviceBuffer&) = delete;
    ~DeviceBuffer()
    {
        if (mData)
            (void)cudaFree(mData);
    }

    void allocate(size_t bytes)
    {
        require(mData == nullptr, "a device buffer allocated twice");
        requireCuda(cudaMalloc(&mData, bytes), "cudaMalloc");
        mBytes = bytes;
    }
    void upload(const std::vector<float>& host)
    {
        if (!mData)
            allocate(host.size() * sizeof(float));
        require(host.size() * sizeof(float) == mBytes, "a device upload of the wrong size");
        requireCuda(cudaMemcpy(mData, host.data(), mBytes, cudaMemcpyHostToDevice), "cudaMemcpy host to device");
    }
    void uploadBytes(const std::vector<uint8_t>& host)
    {
        if (!mData)
            allocate(host.size());
        require(host.size() == mBytes, "a device upload of the wrong size");
        requireCuda(cudaMemcpy(mData, host.data(), mBytes, cudaMemcpyHostToDevice), "cudaMemcpy host to device");
    }
    void* data() const
    {
        return mData;
    }
    size_t bytes() const
    {
        return mBytes;
    }

private:
    void* mData = nullptr;
    size_t mBytes = 0;
};

std::vector<float> downloadFloats(const void* device, size_t floats)
{
    std::vector<float> host(floats);
    requireCuda(cudaMemcpy(host.data(), device, floats * sizeof(float), cudaMemcpyDeviceToHost),
                "cudaMemcpy device to host");
    return host;
}

DLTensor deviceTensor(void* data, int64_t* shape, int32_t ndim, DLDataType dtype)
{
    DLTensor tensor{};
    tensor.data = data;
    tensor.device = DLDevice{ kDLCUDA, kCudaDevice };
    tensor.ndim = ndim;
    tensor.dtype = dtype;
    tensor.shape = shape;
    return tensor;
}

constexpr DLDataType kFloat32{ static_cast<uint8_t>(kDLFloat), 32, 1 };
constexpr DLDataType kUint8{ static_cast<uint8_t>(kDLUInt), 8, 1 };

// A device mask over the rows of a group or binding, one byte per row.
struct DeviceMask
{
    DeviceBuffer buffer;
    std::array<int64_t, 1> shape{};

    void upload(const std::vector<uint8_t>& rows)
    {
        shape[0] = static_cast<int64_t>(rows.size());
        buffer.uploadBytes(rows);
    }
    DLTensor view()
    {
        return deviceTensor(buffer.data(), shape.data(), 1, kUint8);
    }
};

// A group's device column: its first float and how many floats its tensors hold.
struct Column
{
    char* data = nullptr;
    size_t floats = 0;
};

// The group's tensors as one column, each starting where the one before it ends. Throws when they are not.
Column columnOf(const ovstage_data_t& data)
{
    require(data.tensors && data.tensor_count > 0, "a group with no tensor");
    Column column;
    column.data = static_cast<char*>(data.tensors[0].data) + data.tensors[0].byte_offset;
    for (uint32_t i = 0; i < data.tensor_count; ++i)
    {
        const DLTensor& tensor = data.tensors[i];
        require(tensor.device.device_type == kDLCUDA && tensor.dtype.code == kDLFloat && tensor.dtype.bits == 32,
                "a group that is not a float32 device column");
        require(static_cast<char*>(tensor.data) + tensor.byte_offset == column.data + column.floats * sizeof(float),
                "a group whose tensors are not one column");
        size_t floats = tensor.dtype.lanes;
        for (int32_t d = 0; d < tensor.ndim; ++d)
            floats *= static_cast<size_t>(tensor.shape[d]);
        column.floats += floats;
    }
    return column;
}

// The address a fill copies to: the group's first tensor, the start of its column.
char* columnStart(const ovstage_data_t& data)
{
    return static_cast<char*>(data.tensors[0].data) + data.tensors[0].byte_offset;
}

// How many floats each row of a joint group holds, in row order: the group is one tensor with one row per DOF
// (ADR-0059), each row `lanes` floats wide.
std::vector<uint32_t> floatsPerRow(const ovstage_prim_group_t& prims, const ovstage_data_t& data)
{
    require(data.tensors && data.tensor_count == 1, "a joint group carries one tensor");
    require(data.tensors[0].shape && data.tensors[0].shape[0] == int64_t(prims.count),
            "a joint group has one row per prim-list entry");
    return std::vector<uint32_t>(prims.count, data.tensors[0].dtype.lanes);
}

// A session read over a held query, its groups in fetch order.
struct SessionRead
{
    ovphysx_read_handle_t read = 0;
    std::vector<const ovstage_read_group_t*> groups;
};

// A binding over every Humanoid root, and the device tensor it reads into or writes from.
struct BoundTensor
{
    ovphysx::TensorBinding binding;
    DeviceBuffer buffer;
    std::array<int64_t, 2> shape{};
    uint32_t width = 0;

    void bind(ovphysx::PhysX& physx, ovphysx_tensor_type_t type, uint32_t floatsPerEnv, const char* what)
    {
        requireSuccess(physx.createTensorBinding(binding, kRootPattern, type), what);
        ovphysx_tensor_spec_t spec{};
        requireSuccess(binding.spec(spec), what);
        require(spec.ndim == 2 && spec.shape[0] == kEnvCount && spec.shape[1] == floatsPerEnv &&
                    spec.dtype.code == kDLFloat && spec.dtype.bits == 32,
                std::string("the ") + what + " binding is not float32[" + std::to_string(kEnvCount) + ", " +
                    std::to_string(floatsPerEnv) + "]");
        width = floatsPerEnv;
        shape = { kEnvCount, floatsPerEnv };
        buffer.allocate(size_t(kEnvCount) * width * sizeof(float));
        requireCuda(cudaMemset(buffer.data(), 0, buffer.bytes()), "cudaMemset");
    }
    DLTensor view()
    {
        return deviceTensor(buffer.data(), shape.data(), 2, kFloat32);
    }
    // The same shape over other device memory: a source of the binding's width.
    DLTensor viewOf(const DeviceBuffer& other)
    {
        return deviceTensor(other.data(), shape.data(), 2, kFloat32);
    }
};

// What every row shares: the scene, its two held queries, the stream the session rows use, the action buffers, and
// the phase split.
class HumanoidLab : public BmBenchmark
{
public:
    ~HumanoidLab() override
    {
        if (mStream)
        {
            (void)cudaStreamSynchronize(mStream);
            (void)cudaStreamDestroy(mStream);
        }
        if (mStageLoaded && mPhysX)
        {
            for (ovphysx_query_handle_t query : { mArticulationQuery, mJointQuery })
                if (query)
                    (void)bmCompleteTask(mPhysX->handle(), ovphysx_release_query(mPhysX->handle(), query));
            benchmarkClearOvstage(mPhysX, mStage);
        }
        if (mLogCallbackRegistered)
            (void)ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    }

    bool isValid() const override
    {
        const BmGlobals& globals = BmGlobals::getInstance();
        if (!globals.forceGpu())
            throw std::runtime_error("Humanoid rows require --forceGpu");
        if (!globals.directGpu())
            throw std::runtime_error("Humanoid rows require --directGpu");
        if (!globals.getPhysX())
            throw std::runtime_error("Humanoid rows failed to initialize OVPhysX");
        return true;
    }

    uint32_t getNbSteps() const override
    {
        return 20;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
        if (!mInitialized)
        {
            setupScene();
            setupApi();
            validate();
            failOnLoggedError("setup");
            mInitialized = true;
        }
        mPhases = Phases{};
    }

    void preStep() override
    {
    }

    void endRun() override
    {
        if (mPhases.samples)
        {
            const double n = static_cast<double>(mPhases.samples);
            printFormatted("%s PHASES (ms/sample over %u): write=%.3f step=%.3f read=%.3f total=%.3f", rowName(),
                           mPhases.samples, mPhases.write / n, mPhases.step / n, mPhases.read / n,
                           (mPhases.write + mPhases.step + mPhases.read) / n);
        }
        failOnLoggedError("a run");
    }

protected:
    void step() override
    {
        sample();
        ++mPhases.samples;
    }

    virtual const char* rowName() const = 0;
    // The row's own API objects, built once the scene and queries exist.
    virtual void setupApi() = 0;
    // Throws when the row does not do what it claims.
    virtual void validate() = 0;
    // One timed sample.
    virtual void sample() = 0;

    ovphysx_instance_handle_t handle() const
    {
        return mPhysX->handle();
    }

    void complete(ovphysx_enqueue_result_t task, const char* operation) const
    {
        requireSuccess(bmCompleteTask(handle(), task), operation);
    }

    void physicsStep() const
    {
        requireSuccess(ovphysx_step_sync(handle(), kPhysicsDt), "step the Humanoid scene");
    }

    uintptr_t stream() const
    {
        return reinterpret_cast<uintptr_t>(mStream);
    }

    void syncStream() const
    {
        requireCuda(cudaStreamSynchronize(mStream), "cudaStreamSynchronize");
    }

    // Copies `bytes` from `source` into a write group's column on the row's stream, after the commit that last used
    // that column.
    void fillColumn(const ovstage_map_group_t& group, const void* source, size_t bytes) const
    {
        if (group.data.cuda_sync.wait_event)
            requireCuda(cudaStreamWaitEvent(mStream, reinterpret_cast<cudaEvent_t>(group.data.cuda_sync.wait_event), 0),
                        "cudaStreamWaitEvent");
        requireCuda(cudaMemcpyAsync(columnStart(group.data), source, bytes, cudaMemcpyDeviceToDevice, mStream),
                    "fill a write column");
    }

    // Opens a read of `attributes` over `query` on the row's stream and fetches its groups.
    SessionRead openRead(ovphysx_query_handle_t query, const std::vector<ovx_token_or_string_t>& attributes) const
    {
        SessionRead r;
        complete(ovphysx_read(handle(), query, attributes.data(), attributes.size(), ovstage_cuda_sync_t{ stream(), 0 },
                              &r.read, nullptr),
                 "open a Humanoid read");
        for (;;)
        {
            const ovstage_read_group_t* group = nullptr;
            complete(ovphysx_fetch_read_next(handle(), r.read, &group), "fetch a Humanoid read group");
            if (!group)
                break;
            r.groups.push_back(group);
        }
        require(r.groups.size() == attributes.size(), "a Humanoid read did not return one group per attribute");
        return r;
    }

    void releaseRead(SessionRead& r) const
    {
        for (const ovstage_read_group_t* group : r.groups)
            complete(ovphysx_release_group(handle(), r.read, group->read_group_id), "release a Humanoid read group");
        complete(ovphysx_release_read(handle(), r.read), "release a Humanoid read");
        r = SessionRead{};
    }

    // The group of a read that holds `attribute`.
    const ovstage_read_group_t& groupOf(const SessionRead& r, const char* attribute)
    {
        const ovx_token_t token = attributeToken(attribute);
        for (const ovstage_read_group_t* group : r.groups)
            if (group->columns[0].attribute == token)
                return *group;
        throw std::runtime_error(std::string("Humanoid: a read holds no ") + attribute + " group");
    }

    // The environment of each row of a session group: joint prims (/World/envs/env_<n>/Robot/joints/<name>) when
    // `joints`, else articulation roots (/World/envs/env_<n>/Robot/torso).
    std::vector<uint32_t> sessionRowEnvs(const ovstage_prim_group_t& prims, bool joints)
    {
        if (mRootEnvs.empty())
            internRowPaths();
        std::vector<uint32_t> rowEnvs;
        for (uint32_t row = 0; row < prims.count; ++row)
        {
            ovx_primpath_t path = OVX_INVALID_PRIMPATH;
            size_t got = 0;
            const uint32_t at = prims.index_map ? prims.index_map[row] : prims.offset + row;
            require(path_dictionary_get_paths_from_path_list(dictionary(), prims.list, at, 1, &path, &got).status ==
                            OVX_API_SUCCESS &&
                        got == 1,
                    "resolving a row's prim path failed");
            const std::unordered_map<ovx_primpath_t, uint32_t>& envs = joints ? mJointEnvs : mRootEnvs;
            const std::unordered_map<ovx_primpath_t, uint32_t>::const_iterator found = envs.find(path);
            require(found != envs.end(), "a row that is not a Humanoid root or joint");
            rowEnvs.push_back(found->second);
        }
        return rowEnvs;
    }

    // The two action buffers for rows in `rowEnvs` order holding `floats` floats each: every DOF of an environment
    // gets kActionForce, its sign alternating between environments and between the buffers.
    void buildActions(const std::vector<uint32_t>& rowEnvs, const std::vector<uint32_t>& floats)
    {
        require(rowEnvs.size() == floats.size(), "the action rows and their widths differ");
        std::vector<float> action;
        for (size_t row = 0; row < rowEnvs.size(); ++row)
            action.insert(action.end(), floats[row], rowEnvs[row] % 2 ? -kActionForce : kActionForce);
        require(action.size() == size_t(kEnvCount) * kDofCount, "the action does not cover every DOF");
        mActions[0].upload(action);
        for (float& value : action)
            value = -value;
        mActions[1].upload(action);
    }

    // The action buffer of the next sample: the two alternate.
    const DeviceBuffer& nextAction()
    {
        mActionIndex ^= 1u;
        return mActions[mActionIndex];
    }

    // The stand-in policy: writes the next action into `output`, the policy's output buffer, on the row's stream.
    void runPolicy(void* output)
    {
        const DeviceBuffer& action = nextAction();
        requireCuda(cudaMemcpyAsync(output, action.data(), action.bytes(), cudaMemcpyDeviceToDevice, mStream),
                    "run the policy");
    }

    size_t actionBytes() const
    {
        return mActions[0].bytes();
    }

    // The force the last sample's action gives every DOF of environment `env`.
    float lastActionForce(uint32_t env) const
    {
        return (env % 2 == 0) == (mActionIndex == 0) ? kActionForce : -kActionForce;
    }

    void failOnLoggedError(const char* operation) const
    {
        requireSuccess(ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE), "flush the Humanoid error callback");
        require(!mLoggedError.load(std::memory_order_relaxed),
                std::string(rowName()) + ": " + operation + " emitted an OVPhysX error");
    }

    struct Phases
    {
        double write = 0, step = 0, read = 0;
        uint32_t samples = 0;
    };
    Phases mPhases;

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_query_handle_t mArticulationQuery = 0;
    ovphysx_query_handle_t mJointQuery = 0;
    cudaStream_t mStream = nullptr;

private:
    void setupScene()
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
        requireSuccess(ovphysx_set_log_callback(OVPHYSX_LOG_ERROR, nullptr, logCallback, this),
                       "set the Humanoid error callback");
        mLogCallbackRegistered = true;

        const std::string fixture = BmGlobals::getInstance().getDataFolder() + kFixture;
        require(benchmarkLoadUsdWithOvstage(mPhysX, fixture, mStage), "loading " + fixture + " failed");
        mStageLoaded = true;

        std::vector<std::string> targets;
        std::vector<float> transforms(static_cast<size_t>(kEnvCount - 1) * 7, 0.0f);
        for (uint32_t env = 1; env < kEnvCount; ++env)
        {
            targets.push_back(envPath(env));
            float* const transform = transforms.data() + static_cast<size_t>(env - 1) * 7;
            transform[0] = static_cast<float>(env % kGridWidth) * kEnvSpacing;
            transform[1] = static_cast<float>(env / kGridWidth) * kEnvSpacing;
            transform[6] = 1.0f;
        }
        requireSuccess(mPhysX->clone(envPath(0).c_str(), targets, transforms.data()).status,
                       "clone the Humanoid environments");
        requireSuccess(mPhysX->waitAll().status(), "complete the Humanoid clone");

        requireCuda(cudaSetDevice(kCudaDevice), "cudaSetDevice");
        requireSuccess(ovphysx_warmup(handle()), "warm up the Humanoid scene");
        for (uint32_t i = 0; i < kSettleSteps; ++i)
            physicsStep();

        complete(ovphysx_query(handle(), OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &mArticulationQuery,
                               nullptr),
                 "query the Humanoid articulations");
        complete(ovphysx_query(handle(), OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &mJointQuery,
                               nullptr),
                 "query the Humanoid joints");
        requireCuda(cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking), "cudaStreamCreateWithFlags");
    }

    path_dictionary_instance_t* dictionary() const
    {
        path_dictionary_instance_t* const found = ovstage_get_path_dictionary(mStage.stage);
        require(found != nullptr, "the stage has no path dictionary");
        return found;
    }

    ovx_token_t attributeToken(const char* attribute)
    {
        const std::unordered_map<std::string, ovx_token_t>::const_iterator found = mTokens.find(attribute);
        if (found != mTokens.end())
            return found->second;
        const ovx_string_t name{ attribute, std::strlen(attribute) };
        ovx_token_t token{};
        require(path_dictionary_create_tokens_from_strings(dictionary(), &name, 1, &token).status == OVX_API_SUCCESS,
                std::string("interning ") + attribute + " failed");
        mTokens.emplace(attribute, token);
        return token;
    }

    // Every root and joint path of every environment, interned, so a group's rows map to their environments.
    void internRowPaths()
    {
        const auto intern = [&](const std::vector<std::string>& strings, const std::vector<uint32_t>& envs,
                                std::unordered_map<ovx_primpath_t, uint32_t>& out)
        {
            std::vector<ovx_string_t> views;
            for (const std::string& path : strings)
                views.push_back(ovx_string_t{ path.c_str(), path.size() });
            std::vector<ovx_primpath_t> paths(strings.size());
            require(path_dictionary_create_paths_from_strings(dictionary(), views.data(), views.size(), paths.data())
                            .status == OVX_API_SUCCESS,
                    "interning the Humanoid prim paths failed");
            for (size_t i = 0; i < paths.size(); ++i)
                out.emplace(paths[i], envs[i]);
        };
        std::vector<std::string> roots;
        std::vector<uint32_t> rootEnvs;
        std::vector<std::string> joints;
        std::vector<uint32_t> jointEnvs;
        for (uint32_t env = 0; env < kEnvCount; ++env)
        {
            roots.push_back(envPath(env) + "/Robot/torso");
            rootEnvs.push_back(env);
            for (const char* name : kJointNames)
            {
                joints.push_back(envPath(env) + "/Robot/joints/" + name);
                jointEnvs.push_back(env);
            }
        }
        intern(roots, rootEnvs, mRootEnvs);
        intern(joints, jointEnvs, mJointEnvs);
    }

    static void logCallback(ovphysx_log_level_t level, ovphysx_string_t, ovphysx_string_t, double, void* userData)
    {
        if (level == OVPHYSX_LOG_ERROR && userData)
            static_cast<HumanoidLab*>(userData)->mLoggedError.store(true, std::memory_order_relaxed);
    }

    bool mInitialized = false;
    bool mStageLoaded = false;
    bool mLogCallbackRegistered = false;
    std::atomic<bool> mLoggedError{ false };
    ovphysx_sample_stage_attachment_t mStage{};
    std::array<DeviceBuffer, 2> mActions;
    uint32_t mActionIndex = 0;
    std::unordered_map<std::string, ovx_token_t> mTokens;
    std::unordered_map<ovx_primpath_t, uint32_t> mRootEnvs;
    std::unordered_map<ovx_primpath_t, uint32_t> mJointEnvs;
};

// ---------------------------------------------------------------------------------------------------------------------
// Control step
// ---------------------------------------------------------------------------------------------------------------------

class HumanoidControlStep : public HumanoidLab
{
protected:
    // What the last sample read, on the host: its joint positions, and every value it read, those included.
    virtual void observe(std::vector<float>& jointPositions, std::vector<float>& everything) = 0;

    // Two control steps: every value read is finite, the joints move between them, and PhysX applies the last
    // action to every DOF.
    void validate() override
    {
        std::vector<float> joints[2];
        for (int i = 0; i < 2; ++i)
        {
            std::vector<float> everything;
            sample();
            observe(joints[i], everything);
            require(joints[i].size() == size_t(kEnvCount) * kDofCount &&
                        everything.size() == size_t(kEnvCount) * kFloatsReadPerEnv,
                    std::string(rowName()) + " read the wrong number of values");
            for (const float value : everything)
                require(std::isfinite(value), std::string(rowName()) + " read a non-finite value");
        }
        size_t moved = 0;
        for (size_t dof = 0; dof < joints[0].size(); ++dof)
            moved += std::fabs(joints[1][dof] - joints[0][dof]) > 1.0e-4f ? 1 : 0;
        require(moved * 2 > joints[0].size(), std::string(rowName()) + ": the joints did not move under the action");
        checkAppliedForce();
    }

    // The joint force PhysX applies, read over the joint query whichever API the row writes with: each DOF holds the
    // last action for its environment.
    void checkAppliedForce()
    {
        SessionRead read = openRead(mJointQuery, { attributeName(OVPHYSX_ATTR_JOINT_ACTUATION_FORCE) });
        syncStream();
        const ovstage_read_group_t& group = *read.groups[0];
        const Column column = columnOf(group.columns[0].data);
        const std::vector<float> forces = downloadFloats(column.data, column.floats);
        const std::vector<uint32_t> rowEnvs = sessionRowEnvs(group.prims, true);
        const std::vector<uint32_t> floats = floatsPerRow(group.prims, group.columns[0].data);
        releaseRead(read);
        require(forces.size() == size_t(kEnvCount) * kDofCount,
                std::string(rowName()) + " read the wrong number of joint forces");
        size_t at = 0;
        for (size_t row = 0; row < rowEnvs.size(); ++row)
        {
            for (uint32_t i = 0; i < floats[row]; ++i, ++at)
                require(std::fabs(forces[at] - lastActionForce(rowEnvs[row])) < 1.0e-3f,
                        std::string(rowName()) + ": PhysX does not apply the last action's force");
        }
    }

    // The joint positions two session reads hold, and every value they hold.
    void observeRead(const SessionRead& roots, const SessionRead& joints, std::vector<float>& jointPositions,
                     std::vector<float>& everything)
    {
        const Column jointColumn = columnOf(groupOf(joints, OVPHYSX_ATTR_JOINT_POSITION).columns[0].data);
        jointPositions = downloadFloats(jointColumn.data, jointColumn.floats);
        everything.clear();
        for (const SessionRead* read : { &roots, &joints })
            for (const ovstage_read_group_t* group : read->groups)
            {
                const Column column = columnOf(group->columns[0].data);
                const std::vector<float> values = downloadFloats(column.data, column.floats);
                everything.insert(everything.end(), values.begin(), values.end());
            }
    }
};

// Through bindings built once, the force written straight from the policy's output. A binding write and read
// complete before they return, so the policy is waited for before the write.
class HumanoidControlStepBinding : public HumanoidControlStep
{
protected:
    const char* rowName() const override
    {
        return "Lab.humanoid_8192_control_step_tensorbinding_gpu";
    }

    void setupApi() override
    {
        mForce.bind(*mPhysX, OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32, kDofCount, "actuation force");
        mRootPose.bind(*mPhysX, OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32, 7, "root pose");
        mRootVelocity.bind(*mPhysX, OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32, 6, "root velocity");
        mDofPosition.bind(*mPhysX, OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32, kDofCount, "DOF position");
        mDofVelocity.bind(*mPhysX, OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32, kDofCount, "DOF velocity");
        ovphysx_articulation_metadata_t metadata{};
        requireSuccess(mForce.binding.metadata(metadata), "query the Humanoid topology");
        require(metadata.dof_count == static_cast<int32_t>(kDofCount) &&
                    metadata.body_count == static_cast<int32_t>(kBodyCount) && !metadata.is_fixed_base,
                "the Humanoid must be floating-base with 16 bodies and 21 DOFs");

        std::vector<ovphysx_string_t> paths(kEnvCount);
        uint32_t count = 0;
        requireSuccess(ovphysx_tensor_binding_get_prim_paths(handle(), mForce.binding.handle(), paths.data(), kEnvCount,
                                                             &count),
                       "query the Humanoid binding rows");
        require(count == kEnvCount, "the Humanoid binding does not cover every environment");
        std::vector<uint32_t> rowEnvs;
        for (const ovphysx_string_t& path : paths)
            rowEnvs.push_back(envOfPath(std::string(path.ptr, path.length)));
        buildActions(rowEnvs, std::vector<uint32_t>(kEnvCount, kDofCount));
    }

    void sample() override
    {
        double writeMs = 0;
        double stepMs = 0;
        double t = nowMs();
        runPolicy(mForce.buffer.data());
        syncStream();
        const DLTensor force = mForce.view();
        writeMs += nowMs() - t;
        for (uint32_t substep = 0; substep < kDecimation; ++substep)
        {
            t = nowMs();
            requireSuccess(mForce.binding.write(force), "write the actuation force");
            writeMs += nowMs() - t;
            t = nowMs();
            physicsStep();
            stepMs += nowMs() - t;
        }
        t = nowMs();
        for (BoundTensor* state : { &mRootPose, &mRootVelocity, &mDofPosition, &mDofVelocity })
        {
            DLTensor destination = state->view();
            requireSuccess(state->binding.read(destination), "read the Humanoid state");
        }
        mPhases.read += nowMs() - t;
        mPhases.write += writeMs;
        mPhases.step += stepMs;
    }

    void observe(std::vector<float>& jointPositions, std::vector<float>& everything) override
    {
        jointPositions = downloadFloats(mDofPosition.buffer.data(), size_t(kEnvCount) * kDofCount);
        everything.clear();
        for (const BoundTensor* state : { &mRootPose, &mRootVelocity, &mDofPosition, &mDofVelocity })
        {
            const std::vector<float> values =
                downloadFloats(state->buffer.data(), state->buffer.bytes() / sizeof(float));
            everything.insert(everything.end(), values.begin(), values.end());
        }
    }

private:
    BoundTensor mForce;
    BoundTensor mRootPose;
    BoundTensor mRootVelocity;
    BoundTensor mDofPosition;
    BoundTensor mDofVelocity;
};

// Through held queries: a write session per substep, filled with one device copy of the policy's output, and a read
// per query.
class HumanoidControlStepSession : public HumanoidControlStep
{
protected:
    const char* rowName() const override
    {
        return "Lab.humanoid_8192_control_step_session_gpu";
    }

    void setupApi() override
    {
        // The first write's group fixes the action's layout: its joints' environments and DOF counts.
        ovphysx_write_handle_t write = 0;
        const ovstage_map_group_t& group = openForceWrite(write);
        require(columnOf(group.data).floats == size_t(kEnvCount) * kDofCount,
                "the actuation-force column does not hold every DOF");
        buildActions(sessionRowEnvs(group.prims, true), floatsPerRow(group.prims, group.data));
        complete(ovphysx_release_write(handle(), write), "release the Humanoid force write");
        mPolicyOutput.allocate(actionBytes());
    }

    void sample() override
    {
        double writeMs = 0;
        double stepMs = 0;
        double t = nowMs();
        runPolicy(mPolicyOutput.data());
        writeMs += nowMs() - t;
        for (uint32_t substep = 0; substep < kDecimation; ++substep)
        {
            t = nowMs();
            ovphysx_write_handle_t session = 0;
            const ovstage_map_group_t& group = openForceWrite(session);
            fillColumn(group, mPolicyOutput.data(), mPolicyOutput.bytes());
            complete(ovphysx_commit_group(handle(), session, &group, ovstage_cuda_sync_t{ stream(), 0 }),
                     "commit the actuation force");
            complete(ovphysx_release_write(handle(), session), "release the Humanoid force write");
            syncStream();
            writeMs += nowMs() - t;
            t = nowMs();
            physicsStep();
            stepMs += nowMs() - t;
        }
        t = nowMs();
        SessionRead roots = openRead(mArticulationQuery, mRootAttributes);
        SessionRead joints = openRead(mJointQuery, mJointAttributes);
        syncStream();
        if (mObserving)
            observeRead(roots, joints, mObservedJoints, mObservedAll);
        releaseRead(roots);
        releaseRead(joints);
        mPhases.read += nowMs() - t;
        mPhases.write += writeMs;
        mPhases.step += stepMs;
    }

    void validate() override
    {
        mObserving = true;
        HumanoidControlStep::validate();
        mObserving = false;
    }

    void observe(std::vector<float>& jointPositions, std::vector<float>& everything) override
    {
        jointPositions = mObservedJoints;
        everything = mObservedAll;
    }

private:
    // A write of the actuation force over the joint query, drained: its one group.
    const ovstage_map_group_t& openForceWrite(ovphysx_write_handle_t& write) const
    {
        complete(ovphysx_write(handle(), mJointQuery, &mForceAttribute, 1, &write, nullptr),
                 "open the Humanoid force write");
        ovphysx_fetched_write_group_t fetched{};
        complete(ovphysx_fetch_write_next(handle(), write, &fetched), "fetch the Humanoid force group");
        const ovstage_map_group_t* const group = fetched.group;
        require(group != nullptr, "the Humanoid force write has no group");
        complete(ovphysx_fetch_write_next(handle(), write, &fetched), "drain the Humanoid force write");
        require(fetched.group == nullptr, "the Humanoid force write has more than one group");
        return *group;
    }

    const ovx_token_or_string_t mForceAttribute = attributeName(OVPHYSX_ATTR_JOINT_ACTUATION_FORCE);
    const std::vector<ovx_token_or_string_t> mRootAttributes = attributeNames(kRootAttributes);
    const std::vector<ovx_token_or_string_t> mJointAttributes = attributeNames(kJointAttributes);
    DeviceBuffer mPolicyOutput;
    bool mObserving = false;
    std::vector<float> mObservedJoints;
    std::vector<float> mObservedAll;
};

// The session row with the fill and commit, and the reads, captured once into CUDA graphs. The fill is captured while
// the group is live, and each replay copies the policy's output into the column again. The write session and the
// reads stay open for as long as the graphs are replayed; the policy and the steps run outside them.
class HumanoidControlStepCaptured : public HumanoidControlStep
{
public:
    ~HumanoidControlStepCaptured() override
    {
        // Every replay has finished before the graphs and the sessions they replay go; all of it outside a capture.
        if (mStream)
            (void)cudaStreamSynchronize(mStream);
        for (cudaGraphExec_t exec : { mWriteExec, mReadExec })
            if (exec)
                (void)cudaGraphExecDestroy(exec);
        for (cudaGraph_t graph : { mWriteGraph, mReadGraph })
            if (graph)
                (void)cudaGraphDestroy(graph);
        if (!mPhysX)
            return;
        for (SessionRead* read : { &mRoots, &mJoints })
        {
            if (!read->read)
                continue;
            for (const ovstage_read_group_t* group : read->groups)
                (void)bmCompleteTask(handle(), ovphysx_release_group(handle(), read->read, group->read_group_id));
            (void)bmCompleteTask(handle(), ovphysx_release_read(handle(), read->read));
        }
        if (mWrite)
            (void)bmCompleteTask(handle(), ovphysx_release_write(handle(), mWrite));
    }

protected:
    const char* rowName() const override
    {
        return "Lab.humanoid_8192_control_step_session_captured_gpu";
    }

    void setupApi() override
    {
        // The captured reads gather into the columns a released read left on each query.
        SessionRead roots = openRead(mArticulationQuery, mRootAttributes);
        SessionRead joints = openRead(mJointQuery, mJointAttributes);
        syncStream();
        releaseRead(roots);
        releaseRead(joints);

        complete(ovphysx_write(handle(), mJointQuery, &mForceAttribute, 1, &mWrite, nullptr),
                 "open the Humanoid force write");
        ovphysx_fetched_write_group_t fetched{};
        complete(ovphysx_fetch_write_next(handle(), mWrite, &fetched), "fetch the Humanoid force group");
        // Live until the captured commit: nothing touches it after.
        const ovstage_map_group_t* const group = fetched.group;
        require(group != nullptr, "the Humanoid force write has no group");
        complete(ovphysx_fetch_write_next(handle(), mWrite, &fetched), "drain the Humanoid force write");
        require(fetched.group == nullptr, "the Humanoid force write has more than one group");
        require(columnOf(group->data).floats == size_t(kEnvCount) * kDofCount,
                "the actuation-force column does not hold every DOF");
        buildActions(sessionRowEnvs(group->prims, true), floatsPerRow(group->prims, group->data));
        mPolicyOutput.allocate(actionBytes());
        // A capture cannot wait on an event recorded outside it.
        if (group->data.cuda_sync.wait_event)
            requireCuda(
                cudaStreamWaitEvent(mStream, reinterpret_cast<cudaEvent_t>(group->data.cuda_sync.wait_event), 0),
                "cudaStreamWaitEvent");

        capture(mWriteGraph, mWriteExec,
                [&]()
                {
                    requireCuda(cudaMemcpyAsync(columnStart(group->data), mPolicyOutput.data(), mPolicyOutput.bytes(),
                                                cudaMemcpyDeviceToDevice, mStream),
                                "capture the actuation-force fill");
                    complete(ovphysx_commit_group(handle(), mWrite, group, ovstage_cuda_sync_t{ stream(), 0 }),
                             "capture the actuation-force commit");
                });
        capture(mReadGraph, mReadExec,
                [&]()
                {
                    mRoots = openRead(mArticulationQuery, mRootAttributes);
                    mJoints = openRead(mJointQuery, mJointAttributes);
                });
    }

    void sample() override
    {
        double writeMs = 0;
        double stepMs = 0;
        double t = nowMs();
        runPolicy(mPolicyOutput.data());
        writeMs += nowMs() - t;
        for (uint32_t substep = 0; substep < kDecimation; ++substep)
        {
            t = nowMs();
            requireCuda(cudaGraphLaunch(mWriteExec, mStream), "replay the actuation-force fill and commit");
            // A replay finishes before the next step.
            syncStream();
            writeMs += nowMs() - t;
            t = nowMs();
            physicsStep();
            stepMs += nowMs() - t;
        }
        t = nowMs();
        requireCuda(cudaGraphLaunch(mReadExec, mStream), "replay the Humanoid reads");
        syncStream();
        mPhases.read += nowMs() - t;
        mPhases.write += writeMs;
        mPhases.step += stepMs;
    }

    void observe(std::vector<float>& jointPositions, std::vector<float>& everything) override
    {
        observeRead(mRoots, mJoints, jointPositions, everything);
    }

private:
    // Captures what `record` queues on the row's stream into `graph`, and instantiates it. Ends the capture whatever
    // happens, so a failure leaves no stream capturing.
    void capture(cudaGraph_t& graph, cudaGraphExec_t& exec, const std::function<void()>& record)
    {
        requireCuda(cudaStreamBeginCapture(mStream, cudaStreamCaptureModeThreadLocal), "cudaStreamBeginCapture");
        try
        {
            record();
        }
        catch (...)
        {
            cudaGraph_t discarded = nullptr;
            if (cudaStreamEndCapture(mStream, &discarded) == cudaSuccess && discarded)
                (void)cudaGraphDestroy(discarded);
            throw;
        }
        requireCuda(cudaStreamEndCapture(mStream, &graph), "cudaStreamEndCapture");
        requireCuda(cudaGraphInstantiate(&exec, graph, 0), "cudaGraphInstantiate");
    }

    const ovx_token_or_string_t mForceAttribute = attributeName(OVPHYSX_ATTR_JOINT_ACTUATION_FORCE);
    const std::vector<ovx_token_or_string_t> mRootAttributes = attributeNames(kRootAttributes);
    const std::vector<ovx_token_or_string_t> mJointAttributes = attributeNames(kJointAttributes);
    ovphysx_write_handle_t mWrite = 0;
    DeviceBuffer mPolicyOutput;
    SessionRead mRoots;
    SessionRead mJoints;
    cudaGraph_t mWriteGraph = nullptr;
    cudaGraph_t mReadGraph = nullptr;
    cudaGraphExec_t mWriteExec = nullptr;
    cudaGraphExec_t mReadExec = nullptr;
};

// ---------------------------------------------------------------------------------------------------------------------
// Masked reset
// ---------------------------------------------------------------------------------------------------------------------

// The reset rows: a step between samples, and the validation they share. Validation moves every environment off the
// initial state, resets phase 0, and checks that each environment holds the initial state when phase 0 selects it and
// the moved state otherwise.
class HumanoidReset : public HumanoidLab
{
public:
    void preStep() override
    {
        physicsStep();
    }

protected:
    // How validation moves an environment: its root rises and every velocity and joint position gains an offset.
    // Orientations stay put, since a moved quaternion would have to stay normalized.
    static constexpr float kRootLift = 0.5f;
    static constexpr float kRootVelocityOffset = 0.5f;

    // One quantity the reset writes, read back after validation's reset, with the values it held initially, the
    // values validation moved it to, and each value's environment.
    struct Quantity
    {
        std::string name;
        std::vector<float> now;
        std::vector<float> initial;
        std::vector<float> moved;
        std::vector<uint32_t> envs;
    };

    // Writes the initial state of the environments phase `phase` selects.
    virtual void reset(uint32_t phase) = 0;
    // Moves every environment off the initial state.
    virtual void perturb() = 0;
    // Every quantity the reset writes, read back.
    virtual std::vector<Quantity> readState() = 0;
    // How far perturb() moves each joint position and velocity, in the API's units.
    virtual float jointOffset() const = 0;

    static bool selected(uint32_t env, uint32_t phase)
    {
        return env % kResetPhases == phase;
    }

    void sample() override
    {
        const double t = nowMs();
        reset(mPhase);
        mPhase = (mPhase + 1) % kResetPhases;
        mPhases.write += nowMs() - t;
    }

    void validate() override
    {
        perturb();
        reset(0);
        for (const Quantity& q : readState())
        {
            require(!q.now.empty() && q.now.size() % kEnvCount == 0 && q.initial.size() == q.now.size() &&
                        q.moved.size() == q.now.size() && q.envs.size() == q.now.size(),
                    std::string(rowName()) + " read the wrong number of " + q.name + " values");
            for (size_t i = 0; i < q.now.size(); ++i)
            {
                const float expected = selected(q.envs[i], 0) ? q.initial[i] : q.moved[i];
                require(std::fabs(q.now[i] - expected) < 1.0e-3f,
                        std::string(rowName()) + ": a " + q.name + " value does not hold what the masked reset leaves");
            }
        }
        mPhase = 1 % kResetPhases;
    }

private:
    uint32_t mPhase = 0;
};

// Through bindings: a masked write of the root pose, root velocity, DOF position and DOF velocity.
class HumanoidResetBinding : public HumanoidReset
{
protected:
    const char* rowName() const override
    {
        return "Lab.humanoid_8192_reset_tensorbinding_gpu";
    }

    void setupApi() override
    {
        constexpr std::array<ovphysx_tensor_type_t, 4> kTypes = {
            OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32, OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32,
            OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32, OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32
        };
        constexpr std::array<uint32_t, 4> kWidths = { 7, 6, kDofCount, kDofCount };
        for (size_t i = 0; i < mState.size(); ++i)
        {
            mState[i].bind(*mPhysX, kTypes[i], kWidths[i], kNames[i]);
            DLTensor destination = mState[i].view();
            requireSuccess(mState[i].binding.read(destination), "read the Humanoid initial state");
            mInitial[i].allocate(mState[i].buffer.bytes());
            requireCuda(cudaMemcpy(mInitial[i].data(), mState[i].buffer.data(), mState[i].buffer.bytes(),
                                   cudaMemcpyDeviceToDevice),
                        "keep the Humanoid initial state");
        }

        std::vector<ovphysx_string_t> paths(kEnvCount);
        uint32_t count = 0;
        requireSuccess(ovphysx_tensor_binding_get_prim_paths(handle(), mState[0].binding.handle(), paths.data(),
                                                             kEnvCount, &count),
                       "query the Humanoid binding rows");
        require(count == kEnvCount, "the Humanoid binding does not cover every environment");
        for (const ovphysx_string_t& path : paths)
            mRowEnvs.push_back(envOfPath(std::string(path.ptr, path.length)));
        for (uint32_t phase = 0; phase < kResetPhases; ++phase)
        {
            std::vector<uint8_t> rows;
            for (const uint32_t env : mRowEnvs)
                rows.push_back(static_cast<uint8_t>(selected(env, phase) ? 1 : 0));
            mMasks[phase].upload(rows);
        }
    }

    void reset(uint32_t phase) override
    {
        const DLTensor mask = mMasks[phase].view();
        for (size_t i = 0; i < mState.size(); ++i)
            requireSuccess(mState[i].binding.writeMasked(mState[i].viewOf(mInitial[i]), mask), "reset the Humanoid");
    }

    void perturb() override
    {
        for (size_t i = 0; i < mState.size(); ++i)
        {
            mMoved[i] = downloadFloats(mInitial[i].data(), mInitial[i].bytes() / sizeof(float));
            for (size_t at = 0; at < mMoved[i].size(); ++at)
                mMoved[i][at] += offset(i, at % mState[i].width);
            DeviceBuffer moved;
            moved.upload(mMoved[i]);
            requireSuccess(mState[i].binding.write(mState[i].viewOf(moved)), "move the Humanoid off its initial state");
        }
    }

    std::vector<Quantity> readState() override
    {
        std::vector<Quantity> quantities;
        for (size_t i = 0; i < mState.size(); ++i)
        {
            DLTensor destination = mState[i].view();
            requireSuccess(mState[i].binding.read(destination), "read the Humanoid state");
            Quantity q;
            q.name = kNames[i];
            q.now = downloadFloats(mState[i].buffer.data(), mState[i].buffer.bytes() / sizeof(float));
            q.initial = downloadFloats(mInitial[i].data(), mInitial[i].bytes() / sizeof(float));
            q.moved = mMoved[i];
            for (const uint32_t env : mRowEnvs)
                q.envs.insert(q.envs.end(), mState[i].width, env);
            quantities.push_back(std::move(q));
        }
        return quantities;
    }

    // Radians, and radians per second.
    float jointOffset() const override
    {
        return 0.2f;
    }

private:
    // How validation moves component `lane` of state `i`: the root pose's height, then every velocity and joint
    // position, and nothing else.
    float offset(size_t i, uint32_t lane) const
    {
        if (i == 0)
            return lane == 2 ? kRootLift : 0.0f;
        return i == 1 ? kRootVelocityOffset : jointOffset();
    }

    static constexpr std::array<const char*, 4> kNames = { "root pose", "root velocity", "DOF position",
                                                           "DOF velocity" };
    // Root pose, root velocity, DOF position, DOF velocity: what the reset writes, their initial values, and the
    // values validation moved them to.
    std::array<BoundTensor, 4> mState;
    std::array<DeviceBuffer, 4> mInitial;
    std::array<std::vector<float>, 4> mMoved;
    std::array<DeviceMask, kResetPhases> mMasks;
    std::vector<uint32_t> mRowEnvs;
};

// Through held queries: a write session over each, every group filled with one device copy of the initial state and
// committed with the phase's row mask.
class HumanoidResetSession : public HumanoidReset
{
protected:
    const char* rowName() const override
    {
        return "Lab.humanoid_8192_reset_session_gpu";
    }

    void setupApi() override
    {
        SessionRead roots = openRead(mArticulationQuery, mRootAttributes);
        SessionRead joints = openRead(mJointQuery, mJointAttributes);
        syncStream();
        keepInitial(roots, kRootAttributes.data(), mInitialRoots);
        keepInitial(joints, kJointAttributes.data(), mInitialJoints);
        const ovstage_read_group_t& rootGroup = groupOf(roots, OVPHYSX_ATTR_ROOT_POSITION);
        const ovstage_read_group_t& jointGroup = groupOf(joints, OVPHYSX_ATTR_JOINT_POSITION);
        mRootEnvs = sessionRowEnvs(rootGroup.prims, false);
        const std::vector<uint32_t> jointRowEnvs = sessionRowEnvs(jointGroup.prims, true);
        const std::vector<uint32_t> jointFloats = floatsPerRow(jointGroup.prims, jointGroup.columns[0].data);
        for (size_t row = 0; row < jointRowEnvs.size(); ++row)
            mJointFloatEnvs.insert(mJointFloatEnvs.end(), jointFloats[row], jointRowEnvs[row]);
        releaseRead(roots);
        releaseRead(joints);
        require(mRootEnvs.size() == kEnvCount, "the Humanoid root column does not cover every environment");

        for (uint32_t phase = 0; phase < kResetPhases; ++phase)
        {
            std::vector<uint8_t> rootRows;
            for (const uint32_t env : mRootEnvs)
                rootRows.push_back(static_cast<uint8_t>(selected(env, phase) ? 1 : 0));
            mRootMasks[phase].upload(rootRows);
            std::vector<uint8_t> jointRows;
            for (const uint32_t env : jointRowEnvs)
                jointRows.push_back(static_cast<uint8_t>(selected(env, phase) ? 1 : 0));
            mJointMasks[phase].upload(jointRows);
        }
    }

    void reset(uint32_t phase) override
    {
        const DLTensor rootMask = mRootMasks[phase].view();
        const DLTensor jointMask = mJointMasks[phase].view();
        write(mArticulationQuery, mRootAttributes, mInitialRoots.data(), &rootMask);
        write(mJointQuery, mJointAttributes, mInitialJoints.data(), &jointMask);
        syncStream();
    }

    void perturb() override
    {
        std::array<DeviceBuffer, 4> movedRoots;
        std::array<DeviceBuffer, 2> movedJoints;
        for (size_t i = 0; i < mInitialRoots.size(); ++i)
        {
            mMovedRoots[i] = downloadFloats(mInitialRoots[i].data(), mInitialRoots[i].bytes() / sizeof(float));
            const size_t lanes = mMovedRoots[i].size() / kEnvCount;
            for (size_t at = 0; at < mMovedRoots[i].size(); ++at)
                mMovedRoots[i][at] += rootOffset(i, at % lanes);
            movedRoots[i].upload(mMovedRoots[i]);
        }
        for (size_t i = 0; i < mInitialJoints.size(); ++i)
        {
            mMovedJoints[i] = downloadFloats(mInitialJoints[i].data(), mInitialJoints[i].bytes() / sizeof(float));
            for (float& value : mMovedJoints[i])
                value += jointOffset();
            movedJoints[i].upload(mMovedJoints[i]);
        }
        write(mArticulationQuery, mRootAttributes, movedRoots.data(), nullptr);
        write(mJointQuery, mJointAttributes, movedJoints.data(), nullptr);
        syncStream();
    }

    std::vector<Quantity> readState() override
    {
        SessionRead roots = openRead(mArticulationQuery, mRootAttributes);
        SessionRead joints = openRead(mJointQuery, mJointAttributes);
        syncStream();
        std::vector<Quantity> quantities;
        for (size_t i = 0; i < kRootAttributes.size(); ++i)
        {
            Quantity q = readBack(roots, kRootAttributes[i], mInitialRoots[i], mMovedRoots[i]);
            const size_t lanes = q.now.size() / kEnvCount;
            for (const uint32_t env : mRootEnvs)
                q.envs.insert(q.envs.end(), lanes, env);
            quantities.push_back(std::move(q));
        }
        for (size_t i = 0; i < kJointAttributes.size(); ++i)
        {
            Quantity q = readBack(joints, kJointAttributes[i], mInitialJoints[i], mMovedJoints[i]);
            q.envs = mJointFloatEnvs;
            quantities.push_back(std::move(q));
        }
        releaseRead(roots);
        releaseRead(joints);
        return quantities;
    }

    // Degrees, and degrees per second.
    float jointOffset() const override
    {
        return 10.0f;
    }

private:
    // How validation moves component `lane` of root attribute `i`, in the order of kRootAttributes: the position's
    // height, both velocities, and not the orientation.
    static float rootOffset(size_t i, size_t lane)
    {
        if (i == 0)
            return lane == 2 ? kRootLift : 0.0f;
        return i == 1 ? 0.0f : kRootVelocityOffset;
    }

    // `attribute` as `read` holds it, with its initial and moved values.
    Quantity readBack(const SessionRead& read, const char* attribute, const DeviceBuffer& initial,
                      const std::vector<float>& moved)
    {
        const Column column = columnOf(groupOf(read, attribute).columns[0].data);
        Quantity q;
        q.name = attribute;
        q.now = downloadFloats(column.data, column.floats);
        q.initial = downloadFloats(initial.data(), initial.bytes() / sizeof(float));
        q.moved = moved;
        return q;
    }

    // A copy of each column of `read`, in the order of `attributes`.
    template <size_t N>
    void keepInitial(const SessionRead& read, const char* const* attributes, std::array<DeviceBuffer, N>& out)
    {
        for (size_t i = 0; i < N; ++i)
        {
            const Column column = columnOf(groupOf(read, attributes[i]).columns[0].data);
            out[i].allocate(column.floats * sizeof(float));
            requireCuda(cudaMemcpy(out[i].data(), column.data, out[i].bytes(), cudaMemcpyDeviceToDevice),
                        "keep the Humanoid initial state");
        }
    }

    // Writes `attributes` over `query`, each group filled with one device copy of `sources[attribute index]` and
    // committed on the row's stream, with `mask` when given.
    void write(ovphysx_query_handle_t query, const std::vector<ovx_token_or_string_t>& attributes,
               const DeviceBuffer* sources, const DLTensor* mask)
    {
        ovphysx_write_handle_t session = 0;
        complete(ovphysx_write(handle(), query, attributes.data(), attributes.size(), &session, nullptr),
                 "open a Humanoid reset write");
        size_t groups = 0;
        for (;;)
        {
            ovphysx_fetched_write_group_t fetched{};
            complete(ovphysx_fetch_write_next(handle(), session, &fetched), "fetch a Humanoid reset group");
            if (!fetched.group)
                break;
            ++groups;
            const DeviceBuffer& source = sources[fetched.attribute_index];
            fillColumn(*fetched.group, source.data(), source.bytes());
            const ovstage_cuda_sync_t sync{ stream(), 0 };
            if (mask)
                complete(ovphysx_commit_group_masked(handle(), session, fetched.group, mask, sync),
                         "commit a Humanoid reset group");
            else
                complete(ovphysx_commit_group(handle(), session, fetched.group, sync), "commit a Humanoid group");
        }
        complete(ovphysx_release_write(handle(), session), "release a Humanoid reset write");
        require(groups == attributes.size(), "a Humanoid reset write did not return one group per attribute");
    }

    const std::vector<ovx_token_or_string_t> mRootAttributes = attributeNames(kRootAttributes);
    const std::vector<ovx_token_or_string_t> mJointAttributes = attributeNames(kJointAttributes);
    // In the order of kRootAttributes and kJointAttributes: the initial values, and the values validation moved
    // them to.
    std::array<DeviceBuffer, 4> mInitialRoots;
    std::array<DeviceBuffer, 2> mInitialJoints;
    std::array<std::vector<float>, 4> mMovedRoots;
    std::array<std::vector<float>, 2> mMovedJoints;
    std::array<DeviceMask, kResetPhases> mRootMasks;
    std::array<DeviceMask, kResetPhases> mJointMasks;
    std::vector<uint32_t> mRootEnvs;
    // The environment of each float of the joint columns.
    std::vector<uint32_t> mJointFloatEnvs;
};

#else

// Without CUDA no row can hold its data on the device, so each refuses to run.
template <int Row>
class HumanoidUnavailable : public BmBenchmark
{
public:
    bool isValid() const override
    {
        throw std::runtime_error("Humanoid rows require a CUDAToolkit-enabled benchmark build");
    }
    uint32_t getNbSteps() const override
    {
        return 20;
    }
    void startRun() override
    {
    }
    void preStep() override
    {
    }
    void endRun() override
    {
    }

protected:
    void step() override
    {
    }
};

using HumanoidControlStepBinding = HumanoidUnavailable<0>;
using HumanoidControlStepSession = HumanoidUnavailable<1>;
using HumanoidControlStepCaptured = HumanoidUnavailable<2>;
using HumanoidResetBinding = HumanoidUnavailable<3>;
using HumanoidResetSession = HumanoidUnavailable<4>;

#endif

Register<HumanoidControlStepBinding, true> sHumanoidControlStepBinding(
    "Lab.humanoid_8192_control_step_tensorbinding_gpu");
Register<HumanoidControlStepSession, true> sHumanoidControlStepSession("Lab.humanoid_8192_control_step_session_gpu");
Register<HumanoidControlStepCaptured, true> sHumanoidControlStepCaptured(
    "Lab.humanoid_8192_control_step_session_captured_gpu");
Register<HumanoidResetBinding, true> sHumanoidResetBinding("Lab.humanoid_8192_reset_tensorbinding_gpu");
Register<HumanoidResetSession, true> sHumanoidResetSession("Lab.humanoid_8192_reset_session_gpu");

} // namespace
