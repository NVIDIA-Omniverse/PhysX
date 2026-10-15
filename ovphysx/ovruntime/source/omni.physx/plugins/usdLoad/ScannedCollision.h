// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-006
 * @covers AC-9
 */
#pragma once

#include <omni/physics/parse/ScannedStage.h>
#include <utility>

namespace omni::physx::usdparser
{

// A selected collision remains owned by its scan, together with merged-mesh
// auxiliaries. The source is owned by the scan or borrowed from the attached
// stage, which must outlive the collision. Creation APIs only borrow.
class ScannedCollision
{
public:
    ScannedCollision() = default;
    ScannedCollision(omni::physics::parse::ScannedStage&& scan,
                     omni::physics::parse::PhysxShapeDesc* shape)
        : mScan(std::move(scan)), mShape(shape)
    {
    }
    ScannedCollision(ScannedCollision&& other) noexcept
        : mScan(std::move(other.mScan)), mShape(std::exchange(other.mShape, nullptr))
    {
    }
    ScannedCollision& operator=(ScannedCollision&& other) noexcept
    {
        if (this != &other)
        {
            mScan = std::move(other.mScan);
            mShape = std::exchange(other.mShape, nullptr);
        }
        return *this;
    }
    omni::physics::parse::PhysxShapeDesc* get() const { return mShape; }
    omni::physics::parse::PhysxShapeDesc* operator->() const { return mShape; }
    explicit operator bool() const { return mShape != nullptr; }

private:
    omni::physics::parse::ScannedStage mScan;
    omni::physics::parse::PhysxShapeDesc* mShape = nullptr;
};

} // namespace omni::physx::usdparser
