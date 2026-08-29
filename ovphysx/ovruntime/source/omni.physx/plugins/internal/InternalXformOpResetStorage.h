// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace internal
{
// helper class to store and restore the xformops of an xformable
// NOTE: inactive ops (e.g. preceding a resetXformStack) are not stored
class XformOpResetStorage
{
public:
    // ingest the xform ops to be restored later
    void store(const PXR_NS::UsdGeomXformable& xformable);
    // restore the xform ops that were stored - optional bool to chain in purging of orphaned sanitized ops
    void restore(PXR_NS::UsdGeomXformable& xformable, bool purgeOrphanedTranslateOrientScale = false) const;
    // helper to remove sanitized xform ops attributes that are not needed anymore
    void purgeOrphanedTranslateOrientScale(PXR_NS::UsdGeomXformable& xformable) const;

private:
    std::vector<PXR_NS::UsdGeomXformOp::Type> mOpType;
    std::vector<PXR_NS::UsdGeomXformOp::Precision> mPrecision;
    std::vector<PXR_NS::VtValue> mValue;
    std::vector<bool> mInvertOp;
    std::vector<bool> mOpWritten;
    std::vector<PXR_NS::TfToken> mSuffix;
    std::vector<PXR_NS::TfToken> mName;
    bool mResetXfromStack = false;
    bool mXformStackWritten;
    void resetToSize(size_t size);

    bool mTranslateOpWritten;
    bool mOrientOpWritten;
    bool mScaleOpWritten;

    PXR_NS::UsdEditTarget mStartEditTarget;
};

} // namespace internal
} // namespace physx
} // namespace omni
