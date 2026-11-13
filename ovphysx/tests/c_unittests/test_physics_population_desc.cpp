// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-POPULATION-DESC-001
 * @covers AC-1 AC-2
 */

// The published physics population description: one immutable description for every caller and
// thread, needing no instance, selecting physics only. Its content against the contract is checked
// by test_physics_population_desc.py, which compares it with the generated dump; here only the
// guarantees a C caller relies on.

#include <gtest/gtest.h>

#include "ovphysx/ovphysx_population.h"

#include <string_view>
#include <thread>
#include <vector>

namespace
{

std::string_view view(const ovx_string_t& s)
{
    return { s.ptr, s.length };
}

TEST(PhysicsPopulationDesc, IsOneDescriptionNeedingNoInstance)
{
    const ovstage_population_desc_t* desc = ovphysx_get_physics_population_desc();
    ASSERT_NE(desc, nullptr);
    EXPECT_EQ(desc, ovphysx_get_physics_population_desc());
    EXPECT_EQ(desc->domains, static_cast<uint32_t>(OVSTAGE_POPULATION_DOMAIN_NONE));
    EXPECT_GT(desc->selector_count, 0u);
}

TEST(PhysicsPopulationDesc, IsTheSameFromEveryThread)
{
    std::vector<const ovstage_population_desc_t*> seen(8, nullptr);
    std::vector<std::thread> threads;
    for (size_t i = 0; i < seen.size(); ++i)
        threads.emplace_back([&seen, i] { seen[i] = ovphysx_get_physics_population_desc(); });
    for (std::thread& t : threads)
        t.join();
    for (const ovstage_population_desc_t* desc : seen)
        EXPECT_EQ(desc, ovphysx_get_physics_population_desc());
}

TEST(PhysicsPopulationDesc, PublishesTheStageUnitsAndUnitScaledDefaults)
{
    const ovstage_population_desc_t& desc = *ovphysx_get_physics_population_desc();
    ASSERT_EQ(desc.stage_metadata_path_count, 3u);
    EXPECT_EQ(view(desc.stage_metadata_paths[0]), "metersPerUnit");
    EXPECT_EQ(view(desc.stage_metadata_paths[1]), "kilogramsPerUnit");
    EXPECT_EQ(view(desc.stage_metadata_paths[2]), "upAxis");

    bool gravity = false;
    for (size_t s = 0; s < desc.selector_count; ++s)
    {
        const ovstage_population_selector_t& selector = desc.selectors[s];
        for (size_t r = 0; r < selector.derived_property_count; ++r)
        {
            const ovstage_population_derived_property_t& rule = selector.derived_properties[r];
            EXPECT_NE(rule.value, nullptr) << view(rule.name);
            gravity |= view(rule.name) == "physics:gravityMagnitude";
        }
    }
    EXPECT_TRUE(gravity);
}

} // namespace
