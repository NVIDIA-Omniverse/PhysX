// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physics/parse/ScanBackend.h>

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

} // namespace omni::physics::parse
