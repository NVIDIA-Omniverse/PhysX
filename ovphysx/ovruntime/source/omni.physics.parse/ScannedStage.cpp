// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-12
 */

/** @implements REQ-PARSE-BATCH-001
 * @covers AC-3
 */

/**
 * @implements REQ-PARSE-CORE-006
 * @covers AC-9
 */

#include <omni/physics/parse/ScannedStage.h>

namespace omni::physics::parse
{

// Owns the backend source that minted the descriptors' handles, plus the
// retained owned-mesh storage. Kept out-of-line so the public header stays
// free of the storage detail.
struct detail::ScannedStageOwners::Impl
{
    std::shared_ptr<IDescriptorAllocator> allocator;
    // `owned` is null for a borrowed source (the attach owns it and outlives this scan).
    std::unique_ptr<IPhysicsSource> owned;
    IPhysicsSource* source = nullptr;

    // Owned-storage buffers for mesh-shape `MergeMeshDesc` data.
    // `MergeMeshPhysxShapeDesc::mergedMesh` is a non-owning raw pointer; the
    // scan owns the underlying data here so it lives for the scan's lifetime.
    std::vector<parse::DescPtr<parse::MergeMeshDesc>> mergedMeshes;

    explicit Impl(std::unique_ptr<IPhysicsSource> s) : owned(std::move(s)), source(owned.get()) {}
    explicit Impl(IPhysicsSource& s) : source(&s) {}
};

detail::ScannedStageOwners::ScannedStageOwners() = default;
detail::ScannedStageOwners::~ScannedStageOwners() = default;
detail::ScannedStageOwners::ScannedStageOwners(ScannedStageOwners&&) noexcept = default;

ScannedStage::ScannedStage() = default;
ScannedStage::ScannedStage(std::unique_ptr<Impl> impl) { mImpl = std::move(impl); }
ScannedStage::~ScannedStage() = default;
ScannedStage::ScannedStage(ScannedStage&&) noexcept = default;
ScannedStage& ScannedStage::operator=(ScannedStage&& other) noexcept
{
    if (this != &other)
    {
        // Keep old owners alive until the implicitly generated field assignment
        // has returned all old slots. Also safe when this is a derived scan's base.
        auto previousOwners = std::move(mImpl);
        detail::ScannedStageDescriptors::operator=(std::move(other));
        mImpl = std::move(other.mImpl);
    }
    return *this;
}

const parse::IPhysicsSource& ScannedStage::source() const
{
    return *mImpl->source;
}

const parse::IPhysicsSource* ScannedStage::sourcePtr() const
{
    return mImpl ? mImpl->source : nullptr;
}

void ScannedStage::retainOwnedMesh(parse::DescPtr<parse::MergeMeshDesc> mergedMesh)
{
    if (!mImpl) mImpl = std::make_unique<Impl>(std::unique_ptr<IPhysicsSource>{});
    mImpl->mergedMeshes.push_back(std::move(mergedMesh));
}

void ScannedStage::retainAllocator(std::shared_ptr<IDescriptorAllocator> allocator)
{
    if (!mImpl) mImpl = std::make_unique<Impl>(std::unique_ptr<IPhysicsSource>{});
    mImpl->allocator = std::move(allocator);
}

ScannedStage makeScannedStageFromSource(std::unique_ptr<IPhysicsSource> source)
{
    return ScannedStage{ std::make_unique<ScannedStage::Impl>(std::move(source)) };
}

ScannedStage makeScannedStageBorrowingSource(IPhysicsSource& source)
{
    return ScannedStage{ std::make_unique<ScannedStage::Impl>(source) };
}

} // namespace omni::physics::parse
