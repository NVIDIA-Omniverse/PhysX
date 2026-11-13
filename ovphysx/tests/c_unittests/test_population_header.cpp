// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-POPULATION-001
 * @covers AC-1 AC-2
 */

// Smoke gate for the shipped population header: it compiles as C++17 in an SDK consumer's
// translation unit (this executable), lives in namespace ovphysx::population, and enforces the
// contract rules that need no PhysX: applicability, required co-applied APIs, resolved defaults,
// ancestor derivation, half-precision conversion, and the read shape of tuple-valued array columns.
// What the builder writes is verified against PhysX by the runtime's generated doctest suite, and read
// back column by column for the builders added with the compiled ovstage cache by
// test_population_builder.py.

#include <gtest/gtest.h>

#include "ovphysx/population/Population.hpp"

#include <ovstage/ovstage.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

bool complete(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    if (enqueue.status != OVSTAGE_OK)
        return false;
    if (enqueue.op_index == OVSTAGE_INVALID_OP_ID)
        return true;
    ovstage_op_wait_result_t wait{};
    const bool ok = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait) == OVSTAGE_OK &&
                    wait.error_op_id_count == 0;
    return ovstage_release_op(stage, enqueue.op_index) == OVSTAGE_OK && ok;
}

// The read shape of one prim's `name` array: element count, then tuple width when above 1.
std::vector<int64_t> readArrayShape(ovstage_instance_t* stage, const char* path, const char* name)
{
    namespace detail = ovphysx::population::detail;
    std::vector<int64_t> shape;
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    const ovx_string_t pathString = detail::str(path);
    const ovx_string_t nameString = detail::str(name);
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    detail::check(dictionary, path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &list),
                  "create_path_list");
    const detail::PathListGuard listGuard{ dictionary, list };
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    EXPECT_EQ(ovstage_query_from_path_list(stage, list, &query), OVSTAGE_OK);
    ovx_token_t token = OVX_INVALID_TOKEN;
    detail::check(dictionary, path_dictionary_create_tokens_from_strings(dictionary, &nameString, 1, &token),
                  "intern_token");
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    EXPECT_TRUE(complete(stage, ovstage_read_attributes(stage, query, &token, 1, { 0, 1, false }, &read)));
    for (;;)
    {
        ovstage_read_group_t group{};
        size_t fetched = 0;
        if (ovstage_fetch_read_next(stage, read, OVSTAGE_TIMEOUT_INFINITE, &group, 1, &fetched) != OVSTAGE_OK)
            break;
        const ovstage_read_column_t& column = group.columns[0];
        EXPECT_TRUE(column.is_array) << name;
        const uint32_t index = column.data.index_map ? column.data.index_map[0] : 0;
        const DLTensor& tensor = column.data.tensors[index];
        shape.assign(tensor.shape, tensor.shape + tensor.ndim);
        EXPECT_EQ(ovstage_release_groups(stage, &group, 1), OVSTAGE_OK);
    }
    EXPECT_TRUE(complete(stage, ovstage_release_read(stage, read)));
    EXPECT_TRUE(complete(stage, ovstage_release_query(stage, query)));
    return shape;
}

class PopulationHeader : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ovstage_instance_desc_t desc{};
        desc.name = "population-header-smoke";
        ASSERT_EQ(ovstage_create_instance(&desc, &mInstance), OVSTAGE_OK);
        ASSERT_NE(mInstance, nullptr);
    }

    void TearDown() override
    {
        if (mInstance)
            ovstage_destroy_instance(mInstance);
    }

    ovstage_instance_t* mInstance = nullptr;
};

} // namespace

TEST_F(PopulationHeader, ResolvedDefaultsFollowStageUnits)
{
    using namespace ovphysx::population;
    Units cm{};
    cm.metersPerUnit = 0.01f;
    const PhysicsSceneDefaults z = PhysicsSceneDefaults::resolved(cm);
    EXPECT_FLOAT_EQ(z.gravityMagnitude, 981.0f); // 9.81 / metersPerUnit
    EXPECT_FLOAT_EQ(z.gravityDirection[2], -1.0f);
    Units y{};
    y.upAxis = 'Y';
    EXPECT_FLOAT_EQ(PhysicsSceneDefaults::resolved(y).gravityDirection[1], -1.0f);
}

TEST_F(PopulationHeader, AncestorPathsAreDerivedOnce)
{
    const std::vector<std::string> parents =
        ovphysx::population::ancestorPaths({ "/World/Env/Box_0", "/World/Env/Box_1", "/Other" });
    EXPECT_EQ(parents, (std::vector<std::string>{ "/World", "/World/Env" }));
}

TEST_F(PopulationHeader, RejectsApiOnTypeItDoesNotApplyTo)
{
    ovphysx::population::PrimBatch batch(mInstance, { "/World/Scene" });
    batch.definePhysicsScene();
    EXPECT_THROW(batch.applyPhysicsRigidBodyAPI(), std::invalid_argument);
}

TEST_F(PopulationHeader, RejectsWriteMissingRequiredApi)
{
    ovphysx::population::PrimBatch batch(mInstance, { "/World/Box" });
    batch.defineCube();
    batch.applyPhysxCollisionAPI(); // requires PhysicsCollisionAPI on the same prim
    EXPECT_THROW(batch.write(1), std::invalid_argument);
}

TEST_F(PopulationHeader, RequiresTheSameDriveInstance)
{
    ovphysx::population::PrimBatch batch(mInstance, { "/World/Joint" });
    batch.definePhysicsJoint();
    batch.applyPhysicsDriveAPI("rotX");
    batch.applyPhysxDrivePerformanceEnvelopeAPI("rotY"); // envelope extends the same-instance drive only
    EXPECT_THROW(batch.write(1), std::invalid_argument);
}

TEST_F(PopulationHeader, WritesAndSealsARigidCube)
{
    using namespace ovphysx::population;
    writeStageUnits(mInstance, 1, Units{});
    PrimBatch batch(mInstance, { "/World/Box_0", "/World/Box_1" });
    CubeArgs cube;
    cube.size = std::vector<double>{ 2.0, 2.0 };
    batch.defineCube(cube);
    batch.applyPhysicsRigidBodyAPI();
    batch.applyPhysicsCollisionAPI();
    EXPECT_NO_THROW(batch.write(1));
}

TEST_F(PopulationHeader, HalfColumnsTakeFloatsRoundedLikeNumpy)
{
    // Half-precision columns take floats, as the Python builder does, and store the IEEE 754
    // binary16 nearest to each, ties to even, as numpy's float16 conversion stores them.
    using ovphysx::population::detail::halfBits;
    EXPECT_EQ(halfBits(1.0f), 0x3c00);
    EXPECT_EQ(halfBits(-2.0f), 0xc000);
    EXPECT_EQ(halfBits(0.5f), 0x3800);
    EXPECT_EQ(halfBits(-0.0f), 0x8000);
    EXPECT_EQ(halfBits(65504.0f), 0x7bff); // largest half
    EXPECT_EQ(halfBits(65520.0f), 0x7c00); // halfway to the next binade rounds to infinity
    EXPECT_EQ(halfBits(1.0f + 1.0f / 2048), 0x3c00); // halfway, to the even neighbour
    EXPECT_EQ(halfBits(1.0f + 3.0f / 2048), 0x3c02);
    EXPECT_EQ(halfBits(std::ldexp(1.0f, -24)), 0x0001); // smallest subnormal
    EXPECT_EQ(halfBits(std::ldexp(1.0f, -25)), 0x0000); // halfway to zero, ties to even
    EXPECT_EQ(halfBits(std::ldexp(3.0f, -25)), 0x0002);
    EXPECT_EQ(halfBits(std::numeric_limits<float>::infinity()), 0x7c00);
    EXPECT_EQ(halfBits(std::numeric_limits<float>::quiet_NaN()) & 0x7e00, 0x7e00);
    const std::vector<std::vector<std::array<float, 4>>> rows{ { { 0.5f, 0.5f, 0.5f, 0.5f }, { 1, 0, 0, 0 } } };
    const std::vector<std::vector<std::array<uint16_t, 4>>> bits = halfBits(rows);
    EXPECT_EQ(bits, (std::vector<std::vector<std::array<uint16_t, 4>>>{
                        { { 0x3800, 0x3800, 0x3800, 0x3800 }, { 0x3c00, 0, 0, 0 } } }));
}

TEST_F(PopulationHeader, ArrayColumnsKeepTheirTupleWidth)
{
    // NVBug 7075162: an array element's tuple width travels in dtype.lanes. Published as an (n, lanes)
    // scalar tensor, two quaternions were stored as eight one-lane halves and read back as [8], not [2, 4].
    using namespace ovphysx::population;
    const std::vector<std::vector<std::array<float, 3>>> positions{ { { 1, 2, 3 }, { 4, 5, 6 } } };
    const std::vector<std::vector<std::array<float, 4>>> rotations{ { { 0, 0, 0, 1 }, { 0.5f, 0.5f, 0.5f, 0.5f } } };
    PrimBatch points(mInstance, { "/World/Instancer" });
    PointInstancerArgs pointArgs;
    pointArgs.protoIndices = std::vector<std::vector<int32_t>>{ { 0, 0 } };
    pointArgs.positions = positions;
    pointArgs.orientations = rotations;
    points.definePointInstancer(pointArgs);
    ASSERT_NO_THROW(points.write(1));
    PrimBatch joints(mInstance, { "/World/JointInstancer" });
    PhysxPhysicsJointInstancerArgs jointArgs;
    jointArgs.protoIndices = std::vector<std::vector<int32_t>>{ { 0, 0 } };
    jointArgs.localPos0s = positions;
    jointArgs.localRot0s = rotations;
    joints.definePhysxPhysicsJointInstancer(jointArgs);
    ASSERT_NO_THROW(joints.write(1));
    ovstage_write_floor_desc_t floor{};
    floor.ordinal = 1;
    floor.scope = OVSTAGE_SCOPE_ALL;
    ASSERT_TRUE(complete(mInstance, ovstage_advance_write_floor(mInstance, &floor)));

    using Shape = std::vector<int64_t>;
    EXPECT_EQ(readArrayShape(mInstance, "/World/Instancer", "protoIndices"), (Shape{ 2 }));
    EXPECT_EQ(readArrayShape(mInstance, "/World/Instancer", "positions"), (Shape{ 2, 3 }));
    EXPECT_EQ(readArrayShape(mInstance, "/World/Instancer", "orientations"), (Shape{ 2, 4 }));
    EXPECT_EQ(readArrayShape(mInstance, "/World/JointInstancer", "physics:localPos0s"), (Shape{ 2, 3 }));
    EXPECT_EQ(readArrayShape(mInstance, "/World/JointInstancer", "physics:localRot0s"), (Shape{ 2, 4 }));
}

TEST_F(PopulationHeader, WritesEveryBuilderAddedWithTheCompiledCache)
{
    // What each of these writes is read back column by column in test_population_builder.py;
    // here the C++ builders of the same contract compile, pass their rules and write.
    using namespace ovphysx::population;
    writeStageUnits(mInstance, 1, Units{});
    const auto write = [this](const char* path, const std::function<void(PrimBatch&)>& build) {
        PrimBatch batch(mInstance, { path });
        build(batch);
        EXPECT_NO_THROW(batch.write(1)) << path;
    };
    write("/World/Scope", [](PrimBatch& b) { b.defineScope(); });
    write("/World/Mesh/Subset", [](PrimBatch& b) {
        GeomSubsetArgs args;
        args.indices = std::vector<std::vector<int32_t>>{ { 0, 2 } };
        args.elementType = std::vector<std::string>{ "face" };
        b.defineGeomSubset(args);
    });
    write("/World/Instancer", [](PrimBatch& b) {
        PointInstancerArgs args;
        args.protoIndices = std::vector<std::vector<int32_t>>{ { 0, 1 } };
        args.positions = std::vector<std::vector<std::array<float, 3>>>{ { { 1, 2, 3 }, { 4, 5, 6 } } };
        args.orientations =
            std::vector<std::vector<std::array<float, 4>>>{ { { 0.5f, 0.5f, 0.5f, 0.5f }, { 1, 0, 0, 0 } } };
        args.prototypes = std::vector<std::vector<std::string>>{ { "/World/Proto" } };
        b.definePointInstancer(args);
    });
    write("/World/JointInstancer", [](PrimBatch& b) {
        PhysxPhysicsJointInstancerArgs args;
        args.protoIndices = std::vector<std::vector<int32_t>>{ { 0 } };
        args.localRot0s = std::vector<std::vector<std::array<float, 4>>>{ { { 0, 0, 0, 1 } } };
        args.localRot1s = std::vector<std::vector<std::array<float, 4>>>{ { { 0.5f, -0.5f, 0.5f, -0.5f } } };
        args.prototypes = std::vector<std::vector<std::string>>{ { "/World/JointProto" } };
        b.definePhysxPhysicsJointInstancer(args);
    });
    write("/World/Group", [](PrimBatch& b) {
        CollectionAPIArgs args;
        args.includes = std::vector<std::vector<std::string>>{ { "/World/A" } };
        b.defineXform().applyCollectionAPI("colliders", args);
    });
    // The Newton APIs require their built-in base APIs on the same prim.
    write("/World/Robot",
          [](PrimBatch& b) { b.defineXform().applyPhysicsArticulationRootAPI().applyNewtonArticulationRootAPI(); });
    write("/World/Box", [](PrimBatch& b) { b.defineCube().applyPhysicsCollisionAPI().applyNewtonCollisionAPI(); });
    write("/World/Hull", [](PrimBatch& b) {
        b.defineMesh()
            .applyPhysicsCollisionAPI()
            .applyPhysicsMeshCollisionAPI()
            .applyNewtonCollisionAPI()
            .applyNewtonMeshCollisionAPI();
    });
    write("/World/Scene", [](PrimBatch& b) { b.definePhysicsScene().applyNewtonSceneAPI(); });
    write("/World/Hinge", [](PrimBatch& b) {
        b.definePhysicsRevoluteJoint().applyNewtonJointAPI().applyNewtonMimicAPI().applyPhysxMimicJointAPI("rotX");
    });
    write("/World/Pushed", [](PrimBatch& b) {
        PhysxForceAPIArgs args;
        args.force = std::vector<std::array<float, 3>>{ { 1, 2, 3 } };
        args.mode = std::vector<std::string>{ "acceleration" };
        b.defineXform().applyPhysxForceAPI(args);
    });
    write("/World/Fluid", [](PrimBatch& b) { b.defineMaterial().applyPhysxPBDMaterialAPI(); });
    write("/World/Attachment", [](PrimBatch& b) {
        b.defineXform()
            .applyPhysxTendonAttachmentAPI("t")
            .applyPhysxTendonAttachmentLeafAPI("t")
            .applyPhysxTendonAttachmentRootAPI("t");
    });
    write("/World/Axis", [](PrimBatch& b) {
        PhysxTendonAxisAPIArgs args;
        args.gearing = std::vector<std::vector<float>>{ { 1, 2 } };
        b.definePhysicsRevoluteJoint().applyPhysxTendonAxisAPI("t", args).applyPhysxTendonAxisRootAPI("t");
    });
}
