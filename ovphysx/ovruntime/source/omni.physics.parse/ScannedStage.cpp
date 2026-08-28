// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <omni/physics/parse/ScannedStage.h>

namespace omni::physics::parse
{

// Owns the backend source that minted the descriptors' handles, plus the
// retained owned-mesh storage. Kept out-of-line so the public header stays
// free of the storage detail.
struct ScannedStage::Impl
{
    std::unique_ptr<IPhysicsSource> source;

    // Owned-storage buffers for mesh-shape `MergeMeshDesc` data.
    // `MergeMeshPhysxShapeDesc::mergedMesh` is a non-owning raw pointer; the
    // scan owns the underlying data here so it lives for the scan's lifetime.
    std::vector<parse::DescPtr<parse::MergeMeshDesc>> mergedMeshes;

    explicit Impl(std::unique_ptr<IPhysicsSource> s) : source(std::move(s)) {}
};

ScannedStage::ScannedStage() = default;
ScannedStage::ScannedStage(std::unique_ptr<Impl> impl) : mImpl(std::move(impl)) {}
ScannedStage::~ScannedStage() = default;
ScannedStage::ScannedStage(ScannedStage&&) noexcept = default;
ScannedStage& ScannedStage::operator=(ScannedStage&&) noexcept = default;

const parse::IPhysicsSource& ScannedStage::source() const
{
    return *mImpl->source;
}

const parse::IPhysicsSource* ScannedStage::sourcePtr() const
{
    return mImpl ? mImpl->source.get() : nullptr;
}

void ScannedStage::retainOwnedMesh(parse::DescPtr<parse::MergeMeshDesc> mergedMesh)
{
    mImpl->mergedMeshes.push_back(std::move(mergedMesh));
}

ScannedStage makeScannedStageFromSource(std::unique_ptr<IPhysicsSource> source)
{
    return ScannedStage{ std::make_unique<ScannedStage::Impl>(std::move(source)) };
}

} // namespace omni::physics::parse
