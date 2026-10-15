// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ovphysx_query_contacts at the C boundary (ADR-0044): malformed arguments are refused before a task is
// created, a filter the runtime cannot pair fails the task with the runtime's message, and a valid contact
// query opens and releases like any query.

/**
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1 AC-2
 * @maps_to TEST-CAPI-READ-CONTACT-003
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <string>

using namespace test_utils;

namespace
{
ovphysx_query_selection_t glob(const char* pattern)
{
    ovphysx_query_selection_t s{};
    s.pattern = { pattern, std::char_traits<char>::length(pattern) };
    return s;
}
// A synchronous refusal: INVALID_ARGUMENT, and no task issued, so there is nothing to wait on.
::testing::AssertionResult refusedBeforeATask(const ovphysx_enqueue_result_t& receipt)
{
    if (receipt.status == OVPHYSX_API_INVALID_ARGUMENT && receipt.op_index == 0u)
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << "status " << receipt.status << ", op_index " << receipt.op_index;
}
} // namespace

class QueryContactsTest : public PhysXTestFixture
{
};

// Every malformed argument is a synchronous INVALID_ARGUMENT: no task is created, nothing to wait on.
TEST_F(QueryContactsTest, MalformedArgumentsAreRefusedBeforeATask)
{
    ovphysx_query_handle_t query = 0;
    const ovphysx_query_selection_t ground = glob("/World/BigBase");
    ovphysx_contact_sensor_selection_t selection{};
    selection.sensors = glob("/World/Cube*");
    selection.filters = &ground;

    ovstage_query_result_t summary{};
    summary.total_prim_count = 99;
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &selection, 1, 1, nullptr, &summary)));
    EXPECT_EQ(summary.total_prim_count, 0u);
    EXPECT_EQ(summary.attributes, nullptr);
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, nullptr, 1, 1, &query, nullptr)));
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &selection, 0, 1, &query, nullptr)));
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &selection, 1, 21846, &query, nullptr)));

    // Filters promised but not given.
    ovphysx_contact_sensor_selection_t noFilters = selection;
    noFilters.filters = nullptr;
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &noFilters, 1, 1, &query, nullptr)));

    // A selection naming nothing, a count without handles, an invalid handle, a NUL inside a pattern, and a
    // relative pattern -- on the sensors and on a filter alike.
    ovphysx_contact_sensor_selection_t empty = selection;
    empty.sensors = ovphysx_query_selection_t{};
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &empty, 1, 1, &query, nullptr)));
    ovphysx_contact_sensor_selection_t countOnly = selection;
    countOnly.sensors.prim_paths_count = 2;
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &countOnly, 1, 1, &query, nullptr)));
    const ovx_primpath_t invalid = OVX_INVALID_PRIMPATH;
    ovphysx_contact_sensor_selection_t invalidHandle = selection;
    invalidHandle.sensors.prim_paths = &invalid;
    invalidHandle.sensors.prim_paths_count = 1;
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &invalidHandle, 1, 1, &query, nullptr)));
    const char nulPattern[] = { '/', 'A', '\0', 'B' };
    ovphysx_query_selection_t nulFilter{};
    nulFilter.pattern = { nulPattern, sizeof(nulPattern) };
    ovphysx_contact_sensor_selection_t badFilter = selection;
    badFilter.filters = &nulFilter;
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &badFilter, 1, 1, &query, nullptr)));
    // A relative pattern would resolve from the root; every pattern-taking entry point refuses it.
    ovphysx_contact_sensor_selection_t relativeSensors = selection;
    relativeSensors.sensors = glob("World/Cube*");
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &relativeSensors, 1, 1, &query, nullptr)));
    const ovphysx_query_selection_t relativeFilter = glob("World/BigBase");
    ovphysx_contact_sensor_selection_t relativeFilterSelection = selection;
    relativeFilterSelection.filters = &relativeFilter;
    EXPECT_TRUE(refusedBeforeATask(ovphysx_query_contacts(m_handle, &relativeFilterSelection, 1, 1, &query, nullptr)));
    EXPECT_EQ(query, 0u);
}

// A filter the runtime cannot pair fails the task, and the task's error is the runtime's own message.
TEST_F(QueryContactsTest, UnpairableFilterFailsTheTaskWithTheRuntimeMessage)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to attach ovstage USD";

    // Eleven cubes, and a filter matching every child of /World: neither one object nor one per cube.
    const ovphysx_query_selection_t everything = glob("/World/*");
    ovphysx_contact_sensor_selection_t selection{};
    selection.sensors = glob("/World/Cube*");
    selection.filters = &everything;
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t summary{};
    summary.total_prim_count = 99;
    const ovphysx_result_t failed =
        complete_task(m_handle, ovphysx_query_contacts(m_handle, &selection, 1, 1, &query, &summary));
    EXPECT_EQ(failed.status, OVPHYSX_API_ERROR);
    EXPECT_NE(last_task_error().find("one object per sensor"), std::string::npos) << last_task_error();
    EXPECT_EQ(query, 0u);
    EXPECT_EQ(summary.total_prim_count, 0u);
    EXPECT_EQ(summary.attributes, nullptr);

    // One shared filter pairs: the query opens and releases.
    const ovphysx_query_selection_t ground = glob("/World/BigBase");
    selection.filters = &ground;
    ASSERT_EQ(complete_task(m_handle, ovphysx_query_contacts(m_handle, &selection, 1, 1, &query, &summary)).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_NE(query, 0u);
    EXPECT_GT(summary.attribute_count, 0u);
    EXPECT_NE(summary.attributes, nullptr);
    EXPECT_EQ(summary.query_result_id, 0u);
    EXPECT_EQ(summary.all_handle, 0u);
    EXPECT_EQ(complete_task(m_handle, ovphysx_release_query(m_handle, query)).status, OVPHYSX_API_SUCCESS);
}
