// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @implements REQ-PARSE-SCENE-001
 * @covers AC-4 AC-6
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-3
 */

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physics/usd/PrimIterator.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>
#include <OmniPhysX.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "Material.h"
#include "AttributeHelpers.h"
#include "NewtonCompat.h"

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include "UsdSource.h"

using namespace PXR_NS;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

// Units-aware scene defaults live in the parse lib's setToDefault(PhysxSceneDesc&,
// SourceUnits&) (nested material descs are defaulted at the consumer boundary in
// LoadStage). An unscoped OVStage scan can use makeDefaultSceneDesc(); a scoped
// scan emits no fallback, and LoadStage creates its own default for a scene-less
// initial load. See ParseScene.cpp and LoadStage.cpp.

} // namespace usdparser
} // namespace physx
} // namespace omni
