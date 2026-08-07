// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-2 AC-3
 */

#include <omni/physics/usd/UsdParseBackend.h>

#include <omni/physics/parse/IParseBackend.h>

#include "UsdSource.h"
#include "UsdPhysicsDataWrite.h"

#include <pxr/usd/usd/stage.h>

namespace omni
{
namespace physics
{
namespace usd
{

namespace
{

class UsdParseBackend final : public parse::IParseBackend
{
public:
    std::string_view id() const override
    {
        return "usd";
    }

    parse::SourceBundle createSource(const parse::AttachTarget& target) override
    {
        parse::SourceBundle bundle;
        if (!target.nativeStage)
            return bundle;

        // The USD backend interprets nativeStage as the live UsdStageWeakPtr.
        const PXR_NS::UsdStageWeakPtr stage = *static_cast<const PXR_NS::UsdStageWeakPtr*>(target.nativeStage);
        if (!stage)
            return bundle;

        // UsdSource interns paths/tokens bound to this stage. The output sink
        // resolves keys through the same UsdSource, so it is built against it;
        // the change feed is vended by the source (and references it). The
        // UsdSource heap object does not move when ownership transfers into the
        // bundle, so the sink's raw pointer and the feed's reference stay valid.
        auto source = std::make_unique<UsdSource>(stage);
        auto write = std::make_unique<UsdPhysicsDataWrite>(stage, source.get());
        std::unique_ptr<parse::IChangeFeed> changeFeed = source->createChangeFeed();

        bundle.source = std::move(source);
        bundle.write = std::move(write);
        bundle.changeFeed = std::move(changeFeed);
        return bundle;
    }
};

} // namespace

std::unique_ptr<parse::IParseBackend> makeUsdParseBackend()
{
    return std::make_unique<UsdParseBackend>();
}

} // namespace usd
} // namespace physics
} // namespace omni
