// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physics/parse/IChangeFeed.h>

#include <pxr/base/tf/notice.h>
#include <pxr/base/tf/weakBase.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/stage.h>

#include <vector>

namespace omni
{
namespace physics
{
namespace usd
{

class UsdSource;

class UsdChangeFeed final : public parse::IChangeFeed, public PXR_NS::TfWeakBase
{
public:
    // `source` must outlive the feed (it owns the path/token tables the feed
    // resolves through). `stage` is the sender the TfNotice is filtered to.
    UsdChangeFeed(UsdSource& source, PXR_NS::UsdStageWeakPtr stage);
    ~UsdChangeFeed() override;

    UsdChangeFeed(const UsdChangeFeed&) = delete;
    UsdChangeFeed& operator=(const UsdChangeFeed&) = delete;

    // IChangeFeed
    void registerInterest(parse::ObjectKey objectType,
                          parse::TokenId prop,
                          int device,
                          parse::OnChangeFn cb,
                          uint64_t userData) override;
    void registerGroupComplete(parse::OnGroupCompleteFn cb) override;
    bool drain() override; // no-op for the push-through USD feed
    void setEnabled(bool enabled) override;

private:
    struct Registration
    {
        parse::ObjectKey objectType;
        parse::TokenId   prop;
        parse::OnChangeFn cb;
        uint64_t         userData = 0;
    };

    // TfNotice entry point (filtered to mStage).
    void onObjectsChanged(const PXR_NS::UsdNotice::ObjectsChanged& notice);

    // Deliver one batch to every matching registration. A registration with an
    // invalid objectType + invalid prop is a wildcard (matches every batch).
    void dispatch(const parse::ChangeBatch& batch) const;

    UsdSource&                mSource;
    PXR_NS::UsdStageWeakPtr   mStage;
    PXR_NS::TfNotice::Key     mNoticeKey;
    std::vector<Registration> mRegistrations;
    parse::OnGroupCompleteFn  mGroupComplete;
    bool                      mEnabled = true;
};

} // namespace usd
} // namespace physics
} // namespace omni
