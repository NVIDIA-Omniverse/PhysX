// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-5
// @implements REQ-COMP-JOINT-001
// @covers AC-5
// @implements REQ-COMP-GEOMETRY-001
// @covers AC-5
#include <ovphysx_companion/custom_interfaces.h>
#include "global_test_environment.h"
#include "../../companion/samples/native/direct_custom.h"
#include <cstring>
#include <type_traits>

namespace sdk = ovphysx::companion::native;
namespace
{
class CustomInterfacesTest : public PhysXTestFixture {};
struct Release
{
    uint64_t lease;
    ~Release() { if (lease) ovphysx_companion_custom_interfaces_release(lease); }
};
// Raw runtime callbacks: no companion callback translation or ObjectContext.
bool rejectJoint(omni::physics::parse::ObjectKey, omni::physics::AttachHandle,
    physx::PxRigidActor*, const physx::PxTransform&, physx::PxRigidActor*, const physx::PxTransform&,
    omni::physx::CustomJointFlag::Enum&, void*) { return false; }
physx::PxU32 noRows(physx::Px1DConstraint*, physx::PxVec3p&, physx::PxU32,
    physx::PxConstraintInvMassScale&, const void*, const physx::PxTransform&, const physx::PxTransform&,
    bool, physx::PxVec3p&, physx::PxVec3p&) { return 0; }
void* rejectGeometry(omni::physics::parse::ObjectKey, omni::physics::AttachHandle,
    const physx::PxCustomGeometry::Type&, void*) { return nullptr; }
void releaseJoint(omni::physics::parse::ObjectKey, omni::physics::AttachHandle, void*) {}
void* prepareJoint(omni::physics::parse::ObjectKey, omni::physics::AttachHandle, void*) { return nullptr; }
const void* jointBlock(omni::physics::parse::ObjectKey, omni::physics::AttachHandle, void*) { return nullptr; }
void jointCom(omni::physics::parse::ObjectKey, omni::physics::AttachHandle, uint32_t, void*) {}
void jointOrigin(omni::physics::parse::ObjectKey, omni::physics::AttachHandle, const physx::PxVec3&, void*) {}
void releaseGeometry(omni::physics::parse::ObjectKey, omni::physics::AttachHandle, void*) {}
physx::PxBounds3 bounds(const physx::PxGeometry&, void*) { return physx::PxBounds3::empty(); }
bool contacts(const physx::PxGeometry&, const physx::PxGeometry&, const physx::PxTransform&,
    const physx::PxTransform&, float, float, float, physx::PxContactBuffer&, void*, void*) { return false; }
void mass(const physx::PxGeometry&, physx::PxMassProperties&, void*) {}
bool manifold(const physx::PxGeometry&, float&, void*) { return false; }
static_assert(std::is_same_v<decltype(sdk::CustomInterfaces::joints), omni::physx::IPhysxCustomJoint*>);
static_assert(std::is_same_v<decltype(sdk::CustomInterfaces::geometry), omni::physx::IPhysxCustomGeometry*>);
}
TEST_F(CustomInterfacesTest, RejectedAcquisitionPreservesOutput)
{
    sdk::CustomInterfaces output;
    std::memset(&output, 0xa5, sizeof(output));
    unsigned char before[sizeof(output)];
    std::memcpy(before, &output, sizeof(output));
    sdk::Compatibility compatibility;
    ++compatibility.sdkVersion;
    EXPECT_EQ(ovphysx_companion_custom_interfaces_acquire(m_handle, &compatibility, sizeof(compatibility),
        sizeof(output), &output), OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before, &output, sizeof(output)), 0);
    compatibility = sdk::Compatibility();
    compatibility.debug ^= 1u;
    EXPECT_EQ(ovphysx_companion_custom_interfaces_acquire(m_handle, &compatibility, sizeof(compatibility),
        sizeof(output), &output), OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before, &output, sizeof(output)), 0);
    compatibility = sdk::Compatibility();
    EXPECT_EQ(ovphysx_companion_custom_interfaces_acquire(m_handle, &compatibility, sizeof(compatibility),
        sizeof(output)-1, &output), OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before, &output, sizeof(output)), 0);
    EXPECT_EQ(ovphysx_companion_custom_interfaces_acquire(0, &compatibility, sizeof(compatibility),
        sizeof(output), &output), OVPHYSX_COMPANION_NOT_FOUND);
    EXPECT_EQ(std::memcmp(before, &output, sizeof(output)), 0);
}
TEST_F(CustomInterfacesTest, OriginalTablesRegisterBeforeAttachAndRetainRuntime)
{
    const sdk::Compatibility compatibility;
    sdk::CustomInterfaces first{}, second{};
    ASSERT_EQ(ovphysx_companion_custom_interfaces_acquire(m_handle, &compatibility, sizeof(compatibility),
        sizeof(first), &first), OVPHYSX_COMPANION_OK);
    Release releaseFirst{first.lease};
    ASSERT_EQ(first.version, 1u);
    ASSERT_NE(first.joints, nullptr);
    ASSERT_NE(first.geometry, nullptr);
    ovphysx_instance_handle_t bootstrap = 0;
    ASSERT_EQ(ovphysx_create_instance(&bootstrap).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_companion_custom_interfaces_acquire(bootstrap, &compatibility, sizeof(compatibility),
        sizeof(second), &second), OVPHYSX_COMPANION_OK);
    Release releaseSecond{second.lease};
    EXPECT_EQ(first.joints, second.joints);
    EXPECT_EQ(first.geometry, second.geometry);
    EXPECT_NE(first.lease, second.lease);
    ASSERT_EQ(ovphysx_destroy_instance(bootstrap).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_INVALID_STATE);
    omni::physx::ICustomJointCallback joint;
    joint.createJointFn = rejectJoint;
    joint.releaseJointFn = releaseJoint;
    joint.prepareJointDataFn = prepareJoint;
    joint.getConstantBlockFn = jointBlock;
    joint.onComShiftFn = jointCom;
    joint.onOriginShift = jointOrigin;
    omni::physx::ICustomGeometryCallback geometry;
    geometry.createCustomGeometryFn = rejectGeometry;
    geometry.releaseCustomGeometryFn = releaseGeometry;
    geometry.computeCustomGeometryLocalBoundsFn = bounds;
    geometry.generateCustomGeometryContactsFn = contacts;
    geometry.computeCustomGeometryMassPropertiesFn = mass;
    geometry.useCustomGeometryPersistentContactManifoldFn = manifold;
    companion_sample::CustomRegistrations registrations = companion_sample::registerCustomTypes(first,
        "CompanionDirectJoint", joint, noRows, 16, "CompanionDirectGeometryAPI", geometry);
    EXPECT_NE(registrations.joint, 0u);
    EXPECT_NE(registrations.geometry, 0u);
    EXPECT_TRUE(registrations.retire(second)); // Same loaded runtime through either bootstrap.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_step(m_handle, 1.0f/60)));
    EXPECT_EQ(ovphysx_companion_custom_interfaces_release(second.lease), OVPHYSX_COMPANION_OK);
    releaseSecond.lease = 0;
    EXPECT_EQ(ovphysx_companion_custom_interfaces_release(second.lease), OVPHYSX_COMPANION_NOT_FOUND);
}
