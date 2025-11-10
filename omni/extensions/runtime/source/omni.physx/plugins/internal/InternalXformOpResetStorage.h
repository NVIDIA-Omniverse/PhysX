// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
    void store(const pxr::UsdGeomXformable& xformable);
    // restore the xform ops that were stored - optional bool to chain in purging of orphaned sanitized ops
    void restore(pxr::UsdGeomXformable& xformable, bool purgeOrphanedTranslateOrientScale = false) const;
    // helper to remove sanitized xform ops attributes that are not needed anymore
    void purgeOrphanedTranslateOrientScale(pxr::UsdGeomXformable& xformable) const;

private:
    std::vector<pxr::UsdGeomXformOp::Type> mOpType;
    std::vector<pxr::UsdGeomXformOp::Precision> mPrecision;
    std::vector<pxr::VtValue> mValue;
    std::vector<bool> mInvertOp;
    std::vector<bool> mOpWritten;
    std::vector<pxr::TfToken> mSuffix;
    std::vector<pxr::TfToken> mName;
    bool mResetXfromStack = false;
    bool mXformStackWritten;
    void resetToSize(size_t size);

    bool mTranslateOpWritten;
    bool mOrientOpWritten;
    bool mScaleOpWritten;

    pxr::UsdEditTarget mStartEditTarget;
};

} // namespace internal
} // namespace physx
} // namespace omni
