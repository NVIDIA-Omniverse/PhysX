// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-SIM-OVSTAGE-READ-REUSE-001
// @maps_to TEST-SIM-OVSTAGE-READ-REUSE-001

#include <gtest/gtest.h>
#include <OvstageSource.h>
#include <ReadGroupUtils.h>

#include <carb/ClientUtils.h>
#include <omni/core/Omni.h>

#include <array>
#include <memory>
#include <limits>
#include <vector>

OMNI_MODULE_DEFINE_OMNI_FUNCTIONS()
CARB_GLOBALS("ovstage_source_bucket_unittests")

namespace omni::physics::ovstage
{
void resetOvstageLiveAttributeReadCountForTest();
size_t getOvstageLiveAttributeReadCountForTest();
}

namespace
{
using omni::physics::ovstage::OvstageSource;
using omni::physics::ovstage::getOvstageLiveAttributeReadCountForTest;
using omni::physics::ovstage::resetOvstageLiveAttributeReadCountForTest;
using omni::physics::parse::AttrValue;
using omni::physics::parse::ObjectKey;
using omni::physics::parse::TokenId;

class OvstageSourceBucket : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ovstage_instance_desc_t desc{};
        desc.name = "ovstage-source-bucket-test";
        ASSERT_EQ(ovstage_create_instance(&desc, &mStage), OVSTAGE_OK);
        mDictionary = ovstage_get_path_dictionary(mStage);
        ASSERT_NE(mDictionary, nullptr);

        ovstage_write_floor_desc_t floor{};
        floor.ordinal = 1;
        floor.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t enqueue = ovstage_advance_write_floor(mStage, &floor);
        ASSERT_EQ(enqueue.status, OVSTAGE_OK);
        ovstage_op_wait_result_t waited{};
        ASSERT_EQ(ovstage_wait_op(mStage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &waited), OVSTAGE_OK);
        EXPECT_EQ(waited.error_op_id_count, 0u);
        ASSERT_EQ(ovstage_release_op(mStage, enqueue.op_index), OVSTAGE_OK);

        mSource = std::make_unique<OvstageSource>(mStage, mDictionary);
        mAttribute = mSource->internToken("physics:velocity");
        const char* names[] = { "/World/A", "/World/B", "/World/C" };
        for (size_t i = 0; i < mKeys.size(); ++i)
        {
            mKeys[i] = mSource->findByPath(names[i]);
            ASSERT_TRUE(mKeys[i].valid());
            mPaths[i] = mSource->canonicalPath(mKeys[i]);
            ASSERT_NE(mPaths[i], OVX_INVALID_PRIMPATH);
        }
        resetOvstageLiveAttributeReadCountForTest();
    }

    void TearDown() override
    {
        mSource.reset();
        if (mStage)
            ovstage_destroy_instance(mStage);
    }

    ovstage_read_group_t group(DLTensor& tensor) const
    {
        ovstage_read_group_t result{};
        result.prims.count = static_cast<uint32_t>(mKeys.size());
        result.data.tensors = &tensor;
        result.data.tensor_count = 1;
        result.data.count = static_cast<uint32_t>(mKeys.size());
        return result;
    }

    static DLTensor tensor(float* data, int64_t* rows, uint64_t byteOffset = 0)
    {
        DLTensor result{};
        result.data = data;
        result.device = { kDLCPU, 0 };
        result.ndim = 1;
        result.dtype = { kDLFloat, 32, 3 };
        result.shape = rows;
        static int64_t stride = 1;
        result.strides = &stride;
        result.byte_offset = byteOffset;
        return result;
    }

    void seed(const ovstage_read_group_t& readGroup, const ObjectKey* keys = nullptr)
    {
        mSource->seedBucketFromReadGroup(
            mAttribute, readGroup, keys, keys ? mKeys.size() : 0, false, mPaths.data(), mPaths.size());
    }

    void expectVelocity(size_t key, float x, float y, float z)
    {
        const AttrValue value = mSource->getAttribute(mKeys[key], mAttribute);
        ASSERT_EQ(value.kind, AttrValue::Kind::eFloat3);
        EXPECT_FLOAT_EQ(value.f3.x, x);
        EXPECT_FLOAT_EQ(value.f3.y, y);
        EXPECT_FLOAT_EQ(value.f3.z, z);
    }

    ovstage_instance_t* mStage = nullptr;
    ovx_path_dictionary_t* mDictionary = nullptr;
    std::unique_ptr<OvstageSource> mSource;
    TokenId mAttribute{};
    std::array<ObjectKey, 3> mKeys{};
    std::array<ovx_primpath_t, 3> mPaths{};
};

TEST_F(OvstageSourceBucket, MappedRowsRespectPrimOrderRepeatedRowsAndByteOffset)
{
    // Three logical prim rows gather from a wider five-row source tensor.
    // The initial tuple is padding, excluded by byte_offset.
    float storage[] = { -99, -99, -99,
                        10, 11, 12, 20, 21, 22, 30, 31, 32, 40, 41, 42, 50, 51, 52 };
    int64_t rows = 5;
    DLTensor values = tensor(storage, &rows, 3 * sizeof(float));
    ovstage_read_group_t readGroup = group(values);
    const uint32_t primMap[] = { 2, 0, 1 };
    const uint32_t dataMap[] = { 4, 1, 4 };
    const ObjectKey keys[] = { mKeys[2], mKeys[0], mKeys[1] };
    readGroup.prims.index_map = primMap;
    readGroup.data.index_map = dataMap;
    seed(readGroup, keys);

    expectVelocity(0, 20, 21, 22);
    expectVelocity(1, 50, 51, 52);
    expectVelocity(2, 50, 51, 52);
    EXPECT_EQ(getOvstageLiveAttributeReadCountForTest(), 0u);
}

TEST_F(OvstageSourceBucket, MaskedRowsAcrossWordBoundaryDoNotReadLive)
{
    constexpr size_t count = 65;
    std::vector<ObjectKey> keys;
    std::vector<ovx_primpath_t> paths;
    std::vector<float> storage;
    for (size_t i = 0; i < count; ++i)
    {
        const ObjectKey key = mSource->findByPath("/World/Masked" + std::to_string(i));
        keys.push_back(key);
        paths.push_back(mSource->canonicalPath(key));
        storage.push_back(static_cast<float>(i));
        storage.push_back(static_cast<float>(i + 100));
        storage.push_back(static_cast<float>(i + 200));
    }
    int64_t rows = static_cast<int64_t>(count);
    DLTensor values = tensor(storage.data(), &rows);
    ovstage_read_group_t readGroup = group(values);
    readGroup.prims.count = static_cast<uint32_t>(count);
    readGroup.data.count = static_cast<uint32_t>(count);
    const uint64_t mask[] = { (uint64_t{ 1 } << 63) | 1, 1 };
    readGroup.data.mask = mask;
    mSource->seedBucketFromReadGroup(
        mAttribute, readGroup, keys.data(), count, false, paths.data(), count);

    for (size_t i = 0; i < count; ++i)
    {
        const AttrValue value = mSource->getAttribute(keys[i], mAttribute);
        if (i == 0 || i == 63 || i == 64)
        {
            ASSERT_EQ(value.kind, AttrValue::Kind::eFloat3) << "row " << i;
            EXPECT_FLOAT_EQ(value.f3.x, static_cast<float>(i));
            EXPECT_FLOAT_EQ(value.f3.y, static_cast<float>(i + 100));
            EXPECT_FLOAT_EQ(value.f3.z, static_cast<float>(i + 200));
        }
        else
        {
            EXPECT_FALSE(value.valid()) << "masked-out row " << i;
        }
    }
    EXPECT_EQ(getOvstageLiveAttributeReadCountForTest(), 0u);
}

TEST_F(OvstageSourceBucket, ReplacingMappedBucketReturnsFreshValues)
{
    float oldStorage[] = { 10, 11, 12, 20, 21, 22, 30, 31, 32 };
    int64_t rows = 3;
    DLTensor oldValues = tensor(oldStorage, &rows);
    ovstage_read_group_t oldGroup = group(oldValues);
    const uint32_t oldMap[] = { 2, 1, 0 };
    oldGroup.data.index_map = oldMap;
    seed(oldGroup);
    expectVelocity(0, 30, 31, 32);
    expectVelocity(1, 20, 21, 22);

    float newStorage[] = { 100, 101, 102, 200, 201, 202, 300, 301, 302 };
    DLTensor newValues = tensor(newStorage, &rows);
    ovstage_read_group_t newGroup = group(newValues);
    const uint32_t newMap[] = { 1, 2, 0 };
    newGroup.data.index_map = newMap;
    seed(newGroup);
    expectVelocity(0, 200, 201, 202);
    expectVelocity(1, 300, 301, 302);
    expectVelocity(2, 100, 101, 102);
    EXPECT_EQ(getOvstageLiveAttributeReadCountForTest(), 0u);
}

TEST_F(OvstageSourceBucket, InvalidMappedRowFallsBackWithoutOutOfBoundsDecode)
{
    float storage[] = { 10, 11, 12, 20, 21, 22, 30, 31, 32 };
    int64_t rows = 3;
    DLTensor values = tensor(storage, &rows);
    ovstage_read_group_t readGroup = group(values);
    const uint32_t invalidMap[] = { 0, 3, 2 };
    readGroup.data.index_map = invalidMap;
    seed(readGroup);

    // No value is authored on the real stage. An unsafe row must attempt the
    // normal live read and return absent, rather than treating invalid memory
    // as a cached value or claiming an authoritative missing value.
    EXPECT_FALSE(mSource->getAttribute(mKeys[1], mAttribute).valid());
    EXPECT_EQ(getOvstageLiveAttributeReadCountForTest(), 1u);
}

TEST_F(OvstageSourceBucket, DenseRowsContinueToAvoidLiveReads)
{
    float storage[] = { 10, 11, 12, 20, 21, 22, 30, 31, 32 };
    int64_t rows = 3;
    DLTensor values = tensor(storage, &rows);
    const ovstage_read_group_t readGroup = group(values);
    seed(readGroup);
    expectVelocity(0, 10, 11, 12);
    expectVelocity(1, 20, 21, 22);
    expectVelocity(2, 30, 31, 32);
    EXPECT_EQ(getOvstageLiveAttributeReadCountForTest(), 0u);
}

TEST(OvstageReadTensorLayout, LaneCanonicalExplicitAndImplicitCompactStrides)
{
    int64_t shape[] = { 5 };
    int64_t strides[] = { 1 };
    DLTensor value{};
    value.ndim = 1;
    value.dtype = { kDLFloat, 32, 3 };
    value.shape = shape;
    value.strides = strides;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    value.strides = nullptr;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}

TEST(OvstageReadTensorLayout, MultidimensionalCompactStrides)
{
    int64_t shape[] = { 4, 1, 3 };
    // A singleton dimension does not advance the address and its stride is
    // irrelevant. The other strides are in complete dtype elements.
    int64_t strides[] = { 3, 99, 1 };
    DLTensor value{};
    value.ndim = 3;
    value.dtype = { kDLFloat, 32, 1 };
    value.shape = shape;
    value.strides = strides;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    value.strides = nullptr;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}

TEST(OvstageReadTensorLayout, NoncompactAndNegativeStridesAreRejected)
{
    int64_t shape[] = { 4, 3 };
    int64_t strides[] = { 4, 1 };
    DLTensor value{};
    value.ndim = 2;
    value.dtype = { kDLFloat, 32, 1 };
    value.shape = shape;
    value.strides = strides;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    strides[0] = 1;
    strides[1] = 4;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    strides[0] = 3;
    strides[1] = -1;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}

TEST(OvstageReadTensorLayout, MissingShapeAndInvalidDimensionsAreRejected)
{
    DLTensor value{};
    value.dtype = { kDLFloat, 32, 3 };
    value.ndim = 1;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    value.ndim = -1;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    int64_t shape[] = { -1, 0 };
    value.shape = shape;
    value.ndim = 2;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    shape[0] = 0;
    shape[1] = -1;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    shape[0] = 1;
    shape[1] = 1;
    value.dtype.lanes = 0;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}

TEST(OvstageReadTensorLayout, ElementAndLaneCountOverflowAreRejected)
{
    int64_t shape[] = { std::numeric_limits<int64_t>::max(), 2 };
    DLTensor value{};
    value.ndim = 2;
    value.dtype = { kDLFloat, 32, 1 };
    value.shape = shape;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    value.ndim = 1;
    value.dtype.lanes = 3;
    EXPECT_FALSE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    shape[0] = std::numeric_limits<int64_t>::max() / 3;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}

TEST(OvstageReadTensorLayout, EmptyNonnegativeTensorsAreCompactWithoutStorage)
{
    int64_t shape[] = { 0, 3 };
    int64_t strides[] = { 3, 1 };
    DLTensor value{};
    value.ndim = 2;
    value.dtype = { kDLFloat, 32, 3 };
    value.shape = shape;
    value.strides = strides;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    shape[0] = 3;
    shape[1] = 0;
    strides[0] = 0;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
    value.strides = nullptr;
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}

TEST(OvstageReadTensorLayout, ScalarDoesNotRequireShapeOrStrides)
{
    DLTensor value{};
    value.ndim = 0;
    value.dtype = { kDLFloat, 32, 1 };
    EXPECT_TRUE(omni::physics::ovstage::detail::isCompactReadTensor(value));
}
} // namespace
