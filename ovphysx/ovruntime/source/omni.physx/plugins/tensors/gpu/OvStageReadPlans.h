// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-READ-DEVICE-001
// @covers AC-12

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{

// A view's last few ovstage read plans (ADR-0055). Planning walks every row of a read, to check it and lay out the
// head of its scratch, so a read repeating a plan's rows and columns takes the plan instead: a held query's read then
// does no per-row host work. Rows are named by `token`, the content version of their list (nextOvStageRowsVersion,
// never 0), and `set`, which of the lists derived from it they are; columns by a signature the view derives from
// them. A view is rebuilt with its scene's view generation, and its plans with it. The head is shared, so a plan
// evicted before the read that took it has uploaded it stays intact.
template <typename Plan>
class OvStageReadPlans
{
public:
    struct Entry
    {
        uint64_t token = 0;
        uint64_t set = 0;
        std::vector<uint64_t> columns;
        Plan plan;
        std::shared_ptr<const std::vector<uint8_t>> head;
        size_t scratchBytes = 0;
        uint64_t tick = 0;
    };

    // The plan made for `token`, `set` and `columns`, or null.
    const Entry* find(uint64_t token, uint64_t set, const std::vector<uint64_t>& columns)
    {
        for (Entry& entry : mEntries)
        {
            if (entry.token == token && entry.set == set && entry.columns == columns)
            {
                entry.tick = ++mClock;
                return &entry;
            }
        }
        return nullptr;
    }

    // Keeps a plan made for `token`, `set` and `columns` in place of the least recently used one.
    void keep(uint64_t token,
              uint64_t set,
              const std::vector<uint64_t>& columns,
              const Plan& plan,
              const std::shared_ptr<const std::vector<uint8_t>>& head,
              size_t scratchBytes)
    {
        Entry* victim = &mEntries[0];
        for (Entry& entry : mEntries)
            if (entry.tick < victim->tick)
                victim = &entry;
        *victim = Entry{ token, set, columns, plan, head, scratchBytes, ++mClock };
    }

private:
    // Every query on the view shares them, and a whole-articulation read takes one per inverse-dynamics cohort.
    static constexpr uint32_t kSlots = 16;
    Entry mEntries[kSlots];
    uint64_t mClock = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni
