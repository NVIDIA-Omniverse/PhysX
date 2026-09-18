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
#include <cstring>
#include <memory>
#include <limits>
#include <vector>

OMNI_MODULE_DEFINE_OMNI_FUNCTIONS()
CARB_GLOBALS("ovstage_source_bucket_unittests")

namespace omni::physics::ovstage
{
void resetOvstageExistsQueryCountForTest();
size_t getOvstageExistsQueryCountForTest();
void resetOvstageLiveAttributeReadCountForTest();
size_t getOvstageLiveAttributeReadCountForTest();
}

namespace
{
using omni::physics::ovstage::OvstageSource;
using omni::physics::ovstage::getOvstageExistsQueryCountForTest;
using omni::physics::ovstage::resetOvstageExistsQueryCountForTest;
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

    bool waitOp(ovstage_enqueue_result_t enqueue)
    {
        if (enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_INVALID_OP_ID)
            return false;
        ovstage_op_wait_result_t result{};
        const ovstage_api_status_t status =
            ovstage_wait_op(mStage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
        const bool ok = status == OVSTAGE_OK && result.error_op_id_count == 0;
        return ovstage_release_op(mStage, enqueue.op_index) == OVSTAGE_OK && ok;
    }

    bool editPaths(const std::vector<std::string>& paths, ovstage_ordinal_t ordinal, bool remove = false)
    {
        std::vector<ovx_string_t> strings;
        for (const std::string& path : paths)
            strings.push_back({ path.data(), path.size() });
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
        if (ovx_path_dictionary_create_path_list_from_strings(
                mDictionary, strings.data(), strings.size(), &list) != OVX_OK)
            return false;
        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        const ovstage_api_status_t queryStatus = ovstage_query_from_path_list(mStage, list, &query);
        (void)ovx_path_dictionary_destroy_path_list(mDictionary, list);
        if (queryStatus != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
            return false;

        bool edited = false;
        if (remove)
        {
            edited = waitOp(ovstage_delete_attributes(mStage, query, nullptr, 0, ordinal));
        }
        else
        {
            std::vector<float> values(paths.size(), 1.0f);
            int64_t rows = static_cast<int64_t>(values.size());
            DLTensor value{};
            value.data = values.data();
            value.device = { kDLCPU, 0 };
            value.ndim = 1;
            value.dtype = { kDLFloat, 32, 1 };
            value.shape = &rows;
            ovstage_write_data_t write{};
            write.tensors = &value;
            write.tensor_count = 1;
            constexpr const char* attribute = "custom:existenceTest";
            ovx_string_or_token_t name{};
            name.string = { attribute, std::strlen(attribute) };
            edited = waitOp(ovstage_write_attribute(mStage, query, name, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT));
        }
        const bool released = waitOp(ovstage_release_query(mStage, query));
        if (!edited || !released)
            return false;
        ovstage_write_floor_desc_t floor{};
        floor.ordinal = ordinal;
        floor.scope = OVSTAGE_SCOPE_ALL;
        return waitOp(ovstage_advance_write_floor(mStage, &floor));
    }

    ovstage_instance_t* mStage = nullptr;
    ovx_path_dictionary_t* mDictionary = nullptr;
    std::unique_ptr<OvstageSource> mSource;
    TokenId mAttribute{};
    std::array<ObjectKey, 3> mKeys{};
    std::array<ovx_primpath_t, 3> mPaths{};
};

// @implements REQ-SIM-OVSTAGE-BINDING-RESOLVE-001
// @maps_to TEST-SIM-OVSTAGE-BINDING-RESOLVE-001
TEST_F(OvstageSourceBucket, BatchExistencePreservesOrderDuplicatesAndMissingPaths)
{
    ASSERT_TRUE(editPaths({ "/World/A", "/World/C" }, 2));
    const ObjectKey missing = mSource->findByPath("/World/Missing");
    const ObjectKey canonical = mSource->canonicalKey(mKeys[2]);
    const std::vector<ObjectKey> keys = { mKeys[2], missing, mKeys[0], mKeys[1], canonical, ObjectKey{} };
    const std::vector<bool> expected = { true, false, true, false, true, false };
    std::vector<bool> found;
    resetOvstageExistsQueryCountForTest();
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, expected);
    EXPECT_EQ(getOvstageExistsQueryCountForTest(), 1u);

    // Both positive and negative answers prime the scalar memo for this epoch.
    for (size_t i = 0; i < keys.size(); ++i)
        EXPECT_EQ(mSource->exists(keys[i]), expected[i]);
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, expected);
    EXPECT_EQ(getOvstageExistsQueryCountForTest(), 1u);
}

TEST_F(OvstageSourceBucket, BatchExistenceDoesNotTreatInternedPathsAsLive)
{
    const std::vector<ObjectKey> keys(mKeys.begin(), mKeys.end());
    std::vector<bool> found;
    resetOvstageExistsQueryCountForTest();
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, std::vector<bool>(keys.size(), false));
    EXPECT_EQ(getOvstageExistsQueryCountForTest(), 1u);

    // A fully empty successful read is authoritative absence and is memoized.
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, std::vector<bool>(keys.size(), false));
    EXPECT_EQ(getOvstageExistsQueryCountForTest(), 1u);
}

TEST_F(OvstageSourceBucket, BatchExistenceTracksDeletionAndRecreationAfterMemoInvalidation)
{
    const std::vector<ObjectKey> keys = { mKeys[0] };
    std::vector<bool> found;
    ASSERT_TRUE(editPaths({ "/World/A" }, 2));
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    ASSERT_EQ(found, std::vector<bool>{ true });

    ASSERT_TRUE(editPaths({ "/World/A" }, 3, true));
    mSource->clearExistsMemo(); // The change-feed drain starts a new existence epoch.
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, std::vector<bool>{ false });

    ASSERT_TRUE(editPaths({ "/World/A" }, 4));
    mSource->clearExistsMemo();
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, std::vector<bool>{ true });
}

TEST_F(OvstageSourceBucket, BatchExistenceUsesOneProbeForManyColdPaths)
{
    constexpr size_t count = 512;
    std::vector<std::string> paths;
    std::vector<ObjectKey> keys;
    for (size_t i = 0; i < count; ++i)
        paths.push_back("/World/Bulk" + std::to_string(i));
    ASSERT_TRUE(editPaths(paths, 2));
    for (const std::string& path : paths)
        keys.push_back(mSource->findByPath(path));
    std::vector<bool> found;
    resetOvstageExistsQueryCountForTest();
    ASSERT_TRUE(mSource->existsBatchChecked(keys, found));
    EXPECT_EQ(found, std::vector<bool>(count, true));
    EXPECT_EQ(getOvstageExistsQueryCountForTest(), 1u);
}

TEST_F(OvstageSourceBucket, BatchExistenceReportsUnavailableStageWithoutPositiveAnswers)
{
    OvstageSource unavailable(nullptr, mDictionary);
    const ObjectKey key = unavailable.findByPath("/World/A");
    ASSERT_TRUE(key.valid());
    std::vector<bool> found = { true };
    EXPECT_FALSE(unavailable.existsBatchChecked({ key }, found));
    EXPECT_EQ(found, std::vector<bool>{ false });
    EXPECT_FALSE(unavailable.existsBatchChecked({ key }, found));
}

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
