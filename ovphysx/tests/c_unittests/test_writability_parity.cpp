// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-2
 * @maps_to TEST-INPUT-COVERAGE-001
 */

#include "global_test_environment.h"
#include "test_utilities.h"
#include "ovphysx/ovphysx_attributes.h"

#include <fstream>
#include <iterator>
#include <regex>
#include <set>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> declaredAttributes()
{
    std::ifstream header(OVPHYSX_SOURCE_DIR "/include/ovphysx/ovphysx_attributes.h");
    if (!header.is_open())
        return {};
    const std::string text((std::istreambuf_iterator<char>(header)), std::istreambuf_iterator<char>());
    const std::regex names(R"RE(#\s*define\s+OVPHYSX_ATTR_\w+\s+(?:\\\s+)?"([^"]+)")RE");
    std::set<std::string> unique;
    for (std::sregex_iterator it(text.begin(), text.end(), names), end; it != end; ++it)
        unique.insert((*it)[1].str());
    return {unique.begin(), unique.end()};
}

class WritabilityParityTest : public PhysXTestFixture
{
};

// The vocabulary comes from the public header; support comes from actual native
// read/write opens. No expected capability table duplicates either implementation.
TEST_F(WritabilityParityTest, ClassifierMatchesRuntimeOnAnEmptyStage)
{
    ASSERT_TRUE(test_utils::register_physx_schemas_with_ovstage());
    ovstage_instance_desc_t desc{};
    desc.name = "writability-parity-empty-stage";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    ASSERT_NE(stage, nullptr);
    {
        std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
        test_utils::ovstage_test_attachments()[m_handle].push_back({stage, 1});
    }
    ovstage_write_floor_desc_t floor{};
    floor.ordinal = 1;
    floor.scope = OVSTAGE_SCOPE_ALL;
    const ovstage_enqueue_result_t seal = ovstage_advance_write_floor(stage, &floor);
    ASSERT_EQ(seal.status, OVSTAGE_OK);
    if (seal.op_index != OVSTAGE_INVALID_OP_ID)
    {
        ovstage_op_wait_result_t sealed{};
        ASSERT_EQ(ovstage_wait_op(stage, seal.op_index, OVSTAGE_TIMEOUT_INFINITE, &sealed), OVSTAGE_OK);
        ASSERT_EQ(sealed.error_op_id_count, 0u);
        ASSERT_EQ(ovstage_release_op(stage, seal.op_index), OVSTAGE_OK);
    }
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 1)).status, OVPHYSX_API_SUCCESS);

    const std::vector<std::string> names = declaredAttributes();
    ASSERT_FALSE(names.empty());
    for (int type = OVPHYSX_QUERY_TARGET_RIGID_BODY; type < OVPHYSX_QUERY_TARGET_COUNT; ++type)
    {
        const ovphysx_query_target_t objectType = static_cast<ovphysx_query_target_t>(type);
        ovphysx_query_handle_t query = 0;
        if (objectType == OVPHYSX_QUERY_TARGET_CONTACT_SENSOR)
        {
            // Filtered attributes require a contact query with filters, even on an empty stage.
            ovphysx_query_selection_t filter{};
            filter.pattern = OVPHYSX_LITERAL("/MissingFilter");
            ovphysx_contact_sensor_selection_t selection{};
            selection.sensors.pattern = OVPHYSX_LITERAL("/MissingSensor");
            selection.filters = &filter;
            ASSERT_EQ(test_utils::complete_task(m_handle,
                ovphysx_query_contacts(m_handle, &selection, 1, 1, &query, nullptr)).status,
                OVPHYSX_API_SUCCESS);
        }
        else
        {
            ASSERT_EQ(test_utils::complete_task(m_handle,
                ovphysx_query(m_handle, objectType, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
                OVPHYSX_API_SUCCESS);
        }
        for (const std::string& name : names)
        {
            SCOPED_TRACE(::testing::Message() << "type=" << type << " attribute=" << name);
            const ovx_string_or_token_t attribute{OVX_INVALID_TOKEN, {name.data(), name.size()}};
            ovphysx_writability_t classification = OVPHYSX_WRITABILITY_UNCLASSIFIED;
            ASSERT_EQ(ovphysx_writability(objectType, &attribute, &classification).status,
                      OVPHYSX_API_SUCCESS);

            ovphysx_read_handle_t read = 0;
            ovphysx_read_result_t readResult{};
            const ovphysx_result_t readTask = test_utils::complete_task(m_handle,
                ovphysx_read(m_handle, query, &attribute, 1, &read, &readResult));
            // Debug render is one instance row, not an empty prim set. A name it does not
            // accept fails the read instead of opening a session with an omission bit.
            if (objectType == OVPHYSX_QUERY_TARGET_DEBUG_RENDER &&
                classification == OVPHYSX_WRITABILITY_UNCLASSIFIED)
            {
                EXPECT_EQ(readTask.status, OVPHYSX_API_ERROR);
                EXPECT_EQ(read, 0u);
                ovphysx_write_handle_t rejected = 0;
                EXPECT_EQ(test_utils::complete_task(m_handle,
                    ovphysx_write(m_handle, query, &attribute, 1, &rejected, nullptr)).status,
                    OVPHYSX_API_ERROR);
                EXPECT_EQ(rejected, 0u);
                continue;
            }
            ASSERT_EQ(readTask.status, OVPHYSX_API_SUCCESS);
            ASSERT_EQ(readResult.attribute_count, 1u);
            ASSERT_NE(readResult.attributes, nullptr);
            const bool readable = !(readResult.attributes[0].omission_reasons &
                                    OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE);
            // Attach creates a synthetic default scene when the stage authors none, and
            // PxPhysics is one process-global row. Debug render is one instance row, so
            // its prim count stays zero. Every other type has no members here.
            // Only physxPointer is produced for the scene and physics rows.
            const bool presentOnEmptyStage =
                objectType == OVPHYSX_QUERY_TARGET_SCENE || objectType == OVPHYSX_QUERY_TARGET_PHYSICS;
            EXPECT_EQ(readResult.total_prim_count, presentOnEmptyStage ? 1u : 0u);
            const bool pointerGroup = presentOnEmptyStage && name == OVPHYSX_ATTR_PHYSX_POINTER;
            EXPECT_EQ(readResult.attributes[0].group_count,
                      (pointerGroup || objectType == OVPHYSX_QUERY_TARGET_DEBUG_RENDER) ? 1u : 0u);
            ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_release_read(m_handle, read)).status,
                      OVPHYSX_API_SUCCESS);

            ovphysx_write_handle_t write = 0;
            const ovphysx_result_t result = test_utils::complete_task(m_handle,
                ovphysx_write(m_handle, query, &attribute, 1, &write, nullptr));
            const bool writable = result.status == OVPHYSX_API_SUCCESS;
            if (writable)
            {
                EXPECT_NE(write, 0u);
                ovphysx_fetched_write_group_t fetched{};
                ASSERT_EQ(test_utils::complete_task(m_handle,
                    ovphysx_fetch_write_next(m_handle, write, &fetched)).status, OVPHYSX_API_SUCCESS);
                if (objectType == OVPHYSX_QUERY_TARGET_DEBUG_RENDER)
                {
                    EXPECT_NE(fetched.group, nullptr);
                    EXPECT_EQ(fetched.attribute_index, 0u);
                }
                else
                {
                    EXPECT_EQ(fetched.group, nullptr);
                    EXPECT_EQ(fetched.attribute_index, SIZE_MAX);
                }
                ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_release_write(m_handle, write)).status,
                          OVPHYSX_API_SUCCESS);
            }
            else
            {
                EXPECT_EQ(result.status, OVPHYSX_API_ERROR);
                EXPECT_EQ(write, 0u);
            }
            if (classification == OVPHYSX_WRITABILITY_CONDITIONAL)
            {
                // Scene/state preconditions are tested by the corresponding live-object cases.
                EXPECT_TRUE(writable);
            }
            else
            {
                const ovphysx_writability_t actual = writable ?
                    (readable ? OVPHYSX_WRITABILITY_WRITABLE : OVPHYSX_WRITABILITY_WRITE_ONLY) :
                    (readable ? OVPHYSX_WRITABILITY_READ_ONLY : OVPHYSX_WRITABILITY_UNCLASSIFIED);
                EXPECT_EQ(classification, actual);
            }
        }
        ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status,
                  OVPHYSX_API_SUCCESS);
    }
}

} // namespace
