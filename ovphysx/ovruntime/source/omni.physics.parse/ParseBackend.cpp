// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-1
 */

#include <omni/physics/parse/IParseBackend.h>

namespace omni
{
namespace physics
{
namespace parse
{
namespace
{
// Owned, lazily-initialized active backend. A function-local static keeps the
// initialization order well-defined across translation units.
std::unique_ptr<IParseBackend>& activeSlot()
{
    static std::unique_ptr<IParseBackend> slot;
    return slot;
}
} // namespace

void setParseBackend(std::unique_ptr<IParseBackend> backend)
{
    activeSlot() = std::move(backend);
}

IParseBackend* parseBackend()
{
    return activeSlot().get();
}

} // namespace parse
} // namespace physics
} // namespace omni
