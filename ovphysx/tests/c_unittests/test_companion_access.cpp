// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-5
// @implements REQ-COMP-INTERACTION-001
// @covers AC-1 AC-4
// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1
#include <ovphysx_companion/native_access.h>
#include <ovphysx/ovphysx_attributes.h>
#include "global_test_environment.h"
#include <cstring>
#include "../../companion/samples/native/read_only.h"
namespace sdk=ovphysx::companion::native;
namespace
{
class CompanionAccess:public PhysXTestFixture {};
struct Probe
{
    ovphysx_instance_handle_t core;
    ovphysx_companion_inspection_api_t inspection;
    int called=0;
    float mass=0;
    ovphysx_api_status_t waitStatus=OVPHYSX_API_SUCCESS;
    ovphysx_enqueue_result_t detach{};
    ovphysx_companion_status_t recursiveStatus=OVPHYSX_COMPANION_OK;
    bool outputPreserved=false;
};
bool inspect(const sdk::ObjectLease& lease,void* raw) noexcept
{
    Probe & probe=*static_cast<Probe*>(raw);++probe.called;
    if(!lease.dynamic() || !lease.actor() || lease.material())return false;
    probe.mass=lease.dynamic()->getMass();
    probe.detach=ovphysx_detach_ovstage(probe.core);
    ovphysx_wait_task_result_t wait{};
    probe.waitStatus=ovphysx_wait_all(probe.core,0,&wait).status;
    ovphysx_destroy_wait_task_result(&wait);
    ovphysx_companion_body_t output;std::memset(&output,0xa5,sizeof(output));
    unsigned char before[sizeof(output)];std::memcpy(before,&output,sizeof(output));
    const ovphysx_companion_object_t & ref=lease.identity();
    probe.recursiveStatus=probe.inspection.read(&ref,sizeof(ref),&output,sizeof(output));
    probe.outputPreserved=std::memcmp(before,&output,sizeof(output))==0;
    return true;
}
}
TEST(CompanionNativeAccess, BuildNegotiationPreservesRejectedOutput)
{
    sdk::NativeAccessApi api;std::memset(&api,0xa5,sizeof(api));
    unsigned char before[sizeof(api)];std::memcpy(before,&api,sizeof(api));
    sdk::Compatibility compatibility;
    ++compatibility.sdkVersion;
    EXPECT_EQ(ovphysx_companion_native_access_acquire(&compatibility,sizeof(compatibility),sizeof(api),&api),OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before,&api,sizeof(api)),0);
    compatibility=sdk::Compatibility();compatibility.debug^=1u;
    EXPECT_EQ(ovphysx_companion_native_access_acquire(&compatibility,sizeof(compatibility),sizeof(api),&api),OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before,&api,sizeof(api)),0);
    compatibility=sdk::Compatibility();compatibility.compiler[0]='?';
    EXPECT_EQ(ovphysx_companion_native_access_acquire(&compatibility,sizeof(compatibility),sizeof(api),&api),OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before,&api,sizeof(api)),0);
    compatibility=sdk::Compatibility();
    EXPECT_EQ(ovphysx_companion_native_access_acquire(&compatibility,sizeof(compatibility)-1,sizeof(api),&api),OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before,&api,sizeof(api)),0);
    ASSERT_EQ(ovphysx_companion_native_access_acquire(&compatibility,sizeof(compatibility),sizeof(api),&api),OVPHYSX_COMPANION_OK);
    EXPECT_NE(api.visit,nullptr);
}
// @implements REQ-CAPI-DETACH-001
// @covers AC-5
TEST_F(CompanionAccess, TypedBorrowRejectsRecursiveWaitAndStaleRecord)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle,"tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_TRUE(waitForTaskSuccess(m_handle,ovphysx_step(m_handle,1.0f/60)));
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle,
        ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_string_or_token_t velocity{0, {OVPHYSX_ATTR_LINEAR_VELOCITY, std::strlen(OVPHYSX_ATTR_LINEAR_VELOCITY)}};
    ovphysx_write_handle_t write = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr)));
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
    ASSERT_NE(fetched.group, nullptr);
    ASSERT_GT(fetched.group->data.tensor_count, 0u);
    for (size_t index = 0; index < fetched.group->data.tensor_count; ++index)
    {
        const DLTensor& tensor = fetched.group->data.tensors[index];
        ASSERT_EQ(tensor.device.device_type, kDLCPU);
        size_t bytes = tensor.dtype.bits / 8 * tensor.dtype.lanes;
        for (int dimension = 0; dimension < tensor.ndim; ++dimension)
            bytes *= static_cast<size_t>(tensor.shape[dimension]);
        std::memset(static_cast<unsigned char*>(tensor.data) + tensor.byte_offset, 0, bytes);
    }
    ovphysx_companion_api_t base{};
    ASSERT_EQ(ovphysx_companion_acquire(1,sizeof(base),&base),OVPHYSX_COMPANION_OK);
    uint64_t session=0;
    ASSERT_EQ(base.open_session(m_handle,&session),OVPHYSX_COMPANION_OK);
    struct Close {ovphysx_companion_api_t& api;uint64_t session;~Close(){api.close_session(session);}} close{base,session};
    ovphysx_companion_identity_t owner{};ASSERT_EQ(base.get_identity(session,&owner),OVPHYSX_COMPANION_OK);
    Probe probe{m_handle,{}};
    ASSERT_EQ(ovphysx_companion_inspection_acquire(1,sizeof(probe.inspection),&probe.inspection),OVPHYSX_COMPANION_OK);
    ovphysx_companion_object_t ref{};
    ASSERT_EQ(probe.inspection.resolve(owner.attachment,"/World/Cube1",1,UINT32_MAX,sizeof(ref),&ref),OVPHYSX_COMPANION_OK);
    sdk::NativeAccessApi api{};const sdk::Compatibility compatibility;
    ASSERT_EQ(ovphysx_companion_native_access_acquire(&compatibility,sizeof(compatibility),sizeof(api),&api),OVPHYSX_COMPANION_OK);
    EXPECT_EQ(api.visit(&ref,sizeof(ref),sdk::AccessKind::Static,inspect,&probe),OVPHYSX_COMPANION_INVALID_ARGUMENT);
    EXPECT_EQ(probe.called,0);
    ASSERT_EQ(api.visit(&ref,sizeof(ref),sdk::AccessKind::Dynamic,inspect,&probe),OVPHYSX_COMPANION_OK);
    EXPECT_EQ(probe.called,1);EXPECT_GT(probe.mass,0);
    EXPECT_EQ(probe.waitStatus,OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.detach.status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(probe.detach.op_index, 0u);
    EXPECT_EQ(probe.detach.instance, 0u);
    bool attached = false;
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attached);
    // Rejected teardown must leave an already-fetched write group usable.
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_commit_group(m_handle, write, fetched.group, {})));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
    EXPECT_EQ(probe.recursiveStatus,OVPHYSX_COMPANION_BUSY);EXPECT_TRUE(probe.outputPreserved);
    ovphysx_wait_task_result_t wait{};
    EXPECT_EQ(ovphysx_wait_all(m_handle,0,&wait).status,OVPHYSX_API_SUCCESS);
    ovphysx_destroy_wait_task_result(&wait);
    float sampleMass=0;
    EXPECT_EQ(companion_sample::readNativeMass(api,ref,sampleMass),OVPHYSX_COMPANION_OK);
    EXPECT_EQ(sampleMass,probe.mass);
    ++ref.generation;
    EXPECT_EQ(api.visit(&ref,sizeof(ref),sdk::AccessKind::Dynamic,inspect,&probe),OVPHYSX_COMPANION_STALE);
    EXPECT_EQ(probe.called,1);
}
