// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-SCENE-001
 * @covers AC-4 AC-6
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-3
 */

namespace omni
{
namespace physx
{
namespace usdparser
{

// Units-aware scene defaults live in the parse lib's setToDefault(PhysxSceneDesc&,
// SourceUnits&) (nested material descs are defaulted at the consumer boundary in
// LoadStage); the "no PhysicsScene authored" case uses makeDefaultSceneDesc().
// See ParseScene.cpp.

} // namespace usdparser
} // namespace physx
} // namespace omni
