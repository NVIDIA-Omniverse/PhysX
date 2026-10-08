// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-OVSTAGE-ATTACH-001
 * @covers AC-1
 *
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-6
 */

#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>

#include <carb/logging/Log.h>

#include <exception>

namespace omni::physics::parse
{
namespace
{
std::unique_ptr<IScanBackend>& activeSlot()
{
    static std::unique_ptr<IScanBackend> slot;
    return slot;
}
} // namespace

void setScanBackend(std::unique_ptr<IScanBackend> backend)
{
    activeSlot() = std::move(backend);
}

IScanBackend* scanBackend()
{
    return activeSlot().get();
}

std::unique_ptr<IScanBackend> takeScanBackend()
{
    return std::move(activeSlot());
}

ScannedStage scanStage(const parse::AttachTarget& target,
                       const std::vector<std::string>& scanRoots,
                       const std::vector<std::string>& excludePaths,
                       const parse::ScanOptions& options,
                       parse::IDescriptorAllocator& allocator)
{
    IScanBackend* backend = scanBackend();
    if (!backend)
        return {};
    // A backend scan failure is reported as an empty ScannedStage (null sourcePtr), never as
    // an escaping exception: callers (loadFromRange) treat that as a clean fail-closed load,
    // which is what drives the transactional attach rollback. Mirrors the identical guard in
    // omni::physics::usd::scanStage.
    try
    {
        return backend->scan(target, scanRoots, excludePaths, options, allocator);
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Physics scan backend failed: %s", error.what());
    }
    catch (...)
    {
        CARB_LOG_ERROR("Physics scan backend failed with an unknown exception");
    }
    return {};
}

} // namespace omni::physics::parse
