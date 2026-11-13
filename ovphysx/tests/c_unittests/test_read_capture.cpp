// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-READ-001
 * @covers AC-2 AC-4
 *
 * A read captured into a CUDA graph through the public C API (ADR-0055): `ovphysx_read` on a stream of the scene's
 * CUDA context, refused on a capturing stream until a read outside the capture leaves its plan held, then captured
 * and replayed against the poses current at each replay. The graph calls come from the CUDA driver, loaded at run
 * time so the binary needs no CUDA link.
 */

#include <gtest/gtest.h>

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>
#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContextManager.h>

#include <cstring>
#include <string>
#include <vector>

#if OVPHYSX_ENABLE_GPU_TESTS
#    if defined(_WIN32)
#        include <windows.h>
#    else
#        include <dlfcn.h>
#    endif

namespace
{
constexpr unsigned kStreamNonBlocking = 1; // CU_STREAM_NON_BLOCKING
constexpr int kCaptureThreadLocal = 1; // CU_STREAM_CAPTURE_MODE_THREAD_LOCAL

// The CUDA driver's graph calls, which IOptionalCuda does not carry. Loaded as test_ovstage_output.cpp loads the
// driver, so the binary stays loadable on CPU-only hosts.
class CudaGraphDriver
{
public:
    CudaGraphDriver()
    {
#    if defined(_WIN32)
        m_library = LoadLibraryA("nvcuda.dll");
#    else
        m_library = dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
#    endif
        if (!m_library)
            return;
        beginCapture = reinterpret_cast<BeginCapture>(symbol("cuStreamBeginCapture_v2"));
        endCapture = reinterpret_cast<EndCapture>(symbol("cuStreamEndCapture"));
        instantiate = reinterpret_cast<Instantiate>(symbol("cuGraphInstantiateWithFlags"));
        launch = reinterpret_cast<Launch>(symbol("cuGraphLaunch"));
        destroyExec = reinterpret_cast<Destroy>(symbol("cuGraphExecDestroy"));
        destroyGraph = reinterpret_cast<Destroy>(symbol("cuGraphDestroy"));
    }

    ~CudaGraphDriver()
    {
        if (!m_library)
            return;
#    if defined(_WIN32)
        FreeLibrary(m_library);
#    else
        dlclose(m_library);
#    endif
    }

    CudaGraphDriver(const CudaGraphDriver&) = delete;
    CudaGraphDriver& operator=(const CudaGraphDriver&) = delete;

    bool ok() const
    {
        return beginCapture && endCapture && instantiate && launch && destroyExec && destroyGraph;
    }

    using BeginCapture = int (*)(void*, int);
    using EndCapture = int (*)(void*, void**);
    using Instantiate = int (*)(void**, void*, unsigned long long);
    using Launch = int (*)(void*, void*);
    using Destroy = int (*)(void*);
    BeginCapture beginCapture = nullptr;
    EndCapture endCapture = nullptr;
    Instantiate instantiate = nullptr;
    Launch launch = nullptr;
    Destroy destroyExec = nullptr;
    Destroy destroyGraph = nullptr;

private:
    void* symbol(const char* name) const
    {
#    if defined(_WIN32)
        return reinterpret_cast<void*>(GetProcAddress(m_library, name));
#    else
        return dlsym(m_library, name);
#    endif
    }

#    if defined(_WIN32)
    HMODULE m_library = nullptr;
#    else
    void* m_library = nullptr;
#    endif
};

ovx_token_or_string_t attrName(const char* name)
{
    ovx_token_or_string_t a{};
    a.string.ptr = name;
    a.string.length = std::strlen(name);
    return a;
}

// The column's floats, once the device work producing them has completed.
std::vector<float> columnToHost(omni::physx::IOptionalCuda* cuda, const DLTensor& column)
{
    std::vector<float> host(size_t(column.shape[0]) * size_t(column.dtype.lanes));
    EXPECT_TRUE(cuda->memcpyDtoH(host.data(), reinterpret_cast<uintptr_t>(column.data) + column.byte_offset,
                                 host.size() * sizeof(float), nullptr));
    return host;
}
} // namespace

class ReadCaptureGpuTest : public ::testing::Test
{
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t config[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true")),
        };
        args.config_entries = config;
        args.config_entry_count = 1;
        if (ovphysx_create_instance_with_args(&args, &s_handle).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            s_skipReason.assign(error.ptr ? error.ptr : "", error.length);
            s_handle = 0;
        }
        else if (!ovphysx::test_cuda::cudaAvailable())
        {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            s_skipReason = "CUDA not available";
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle)
            ovphysx_destroy_instance(s_handle);
        s_handle = 0;
    }

    void SetUp() override
    {
        if (!s_handle)
        {
            if (ovphysxTestRequireCuda())
            {
                FAIL() << "GPU required: " << s_skipReason;
            }
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        if (!m_handle)
            return;
        if (m_failedRead)
        {
            EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, m_failedRead)).status,
                      OVPHYSX_API_SUCCESS);
            m_failedRead = 0;
        }
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
    }

    // Opens a read of `name` over `query` on `sync` and fetches every group. Returns 0 when the read failed.
    ovphysx_read_handle_t open(ovphysx_query_handle_t query,
                               const char* name,
                               ovstage_cuda_sync_t sync,
                               std::vector<const ovstage_read_group_t*>& groups)
    {
        const ovx_token_or_string_t attribute = attrName(name);
        ovphysx_read_handle_t read = 0;
        if (test_utils::complete_task(m_handle, ovphysx_read(m_handle, query, &attribute, 1, sync, &read, nullptr))
                .status != OVPHYSX_API_SUCCESS)
        {
            m_failedRead = read;
            return 0;
        }
        for (;;)
        {
            const ovstage_read_group_t* group = nullptr;
            if (test_utils::complete_task(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)).status !=
                    OVPHYSX_API_SUCCESS ||
                !group)
                break;
            groups.push_back(group);
        }
        return read;
    }

    ovphysx_read_handle_t m_failedRead = 0;
};

ovphysx_instance_handle_t ReadCaptureGpuTest::s_handle = 0;
std::string ReadCaptureGpuTest::s_skipReason;

TEST_F(ReadCaptureGpuTest, CapturedReadReplaysTheCurrentPoses)
{
    CudaGraphDriver driver;
    ASSERT_TRUE(driver.ok());
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    physx::PxRigidActor* actor = static_cast<physx::PxRigidActor*>(
        test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1"));
    ASSERT_NE(actor, nullptr);
    physx::PxCudaContextManager* manager = actor->getScene()->getCudaContextManager();
    ASSERT_NE(manager, nullptr);

    // The caller's stream, in the scene's CUDA context.
    ovphysx::test_cuda::ScopedCudaContextPush push(cuda, reinterpret_cast<uintptr_t>(manager->getContext()));
    ASSERT_TRUE(push.ok());
    uintptr_t stream = 0;
    ASSERT_TRUE(cuda->streamCreate(&stream, kStreamNonBlocking, nullptr));
    void* const cuStream = reinterpret_cast<void*>(stream);
    const ovstage_cuda_sync_t onStream{ stream, 0 };
    ovphysx_query_handle_t query = 0;
    const ovphysx_enqueue_result_t queried =
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr);
    ASSERT_EQ(test_utils::complete_task(m_handle, queried).status, OVPHYSX_API_SUCCESS);

    // Captured with no plan held on its query, the read is refused, and the capture goes on.
    void* graph = nullptr;
    ASSERT_EQ(driver.beginCapture(cuStream, kCaptureThreadLocal), 0);
    std::vector<const ovstage_read_group_t*> refused;
    EXPECT_EQ(open(query, OVPHYSX_ATTR_POSITION, onStream, refused), 0u);
    ASSERT_EQ(driver.endCapture(cuStream, &graph), 0);
    ASSERT_NE(graph, nullptr);
    EXPECT_EQ(driver.destroyGraph(graph), 0);
    graph = nullptr;
    ASSERT_NE(m_failedRead, 0u);
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, m_failedRead)).status,
              OVPHYSX_API_SUCCESS);
    m_failedRead = 0;

    // A read on the stream outside the capture leaves the plan held on the query.
    const void* priorColumn = nullptr;
    {
        std::vector<const ovstage_read_group_t*> prior;
        const ovphysx_read_handle_t read = open(query, OVPHYSX_ATTR_POSITION, onStream, prior);
        ASSERT_NE(read, 0u);
        ASSERT_EQ(prior.size(), 1u);
        EXPECT_EQ(prior[0]->columns[0].data.tensors[0].device.device_type, kDLCUDA);
        EXPECT_EQ(prior[0]->columns[0].data.cuda_sync.stream, 0u);
        EXPECT_NE(prior[0]->columns[0].data.cuda_sync.wait_event, 0u);
        priorColumn = prior[0]->columns[0].data.tensors[0].data;
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status,
                  OVPHYSX_API_SUCCESS);
    }

    std::vector<const ovstage_read_group_t*> captured;
    ASSERT_EQ(driver.beginCapture(cuStream, kCaptureThreadLocal), 0);
    const ovphysx_read_handle_t read = open(query, OVPHYSX_ATTR_POSITION, onStream, captured);
    ASSERT_EQ(driver.endCapture(cuStream, &graph), 0);
    ASSERT_NE(read, 0u);
    ASSERT_EQ(captured.size(), 1u);
    // The captured read gathers into the column the read before it left on the query.
    EXPECT_EQ(captured[0]->columns[0].data.tensors[0].data, priorColumn);
    // Its consumers in the graph are ordered by the stream, so the captured group names no event.
    EXPECT_EQ(captured[0]->columns[0].data.cuda_sync.wait_event, 0u);
    void* exec = nullptr;
    ASSERT_EQ(driver.instantiate(&exec, graph, 0), 0);

    for (int replay = 0; replay < 2; ++replay)
    {
        for (int i = 0; i < 5; ++i)
            ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(driver.launch(exec, cuStream), 0);
        ASSERT_TRUE(cuda->streamSynchronize(stream, nullptr));
        const std::vector<float> actual = columnToHost(cuda, captured[0]->columns[0].data.tensors[0]);

        // The bodies fall, so a replay that did not run would leave the poses of the read before the capture.
        std::vector<const ovstage_read_group_t*> fresh;
        const ovphysx_read_handle_t freshRead = open(query, OVPHYSX_ATTR_POSITION, ovstage_cuda_sync_t{ 0, 0 }, fresh);
        ASSERT_NE(freshRead, 0u);
        ASSERT_EQ(fresh.size(), 1u);
        ASSERT_EQ(fresh[0]->prims.count, captured[0]->prims.count);
        ASSERT_TRUE(fresh[0]->columns[0].data.cuda_sync.wait_event == 0 ||
                    cuda->eventSynchronize(fresh[0]->columns[0].data.cuda_sync.wait_event, nullptr));
        const std::vector<float> expected = columnToHost(cuda, fresh[0]->columns[0].data.tensors[0]);
        ASSERT_EQ(actual.size(), expected.size());
        for (size_t i = 0; i < actual.size(); ++i)
            EXPECT_NEAR(actual[i], expected[i], 1e-5f) << "replay " << replay << ", float " << i;
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, freshRead)).status,
                  OVPHYSX_API_SUCCESS);
    }

    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(driver.destroyExec(exec), 0);
    EXPECT_EQ(driver.destroyGraph(graph), 0);
    EXPECT_TRUE(cuda->streamDestroy(stream, nullptr));
    EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status,
              OVPHYSX_API_SUCCESS);
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
