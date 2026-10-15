// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "ParseApi.h"
namespace omni::physics::parse
{
// Endpoint/DOF validation uses the completed joint graph on the coordinator.
// This row holds raw coefficients; Newton sign conversion belongs to fill.
struct PreparedMimicJoint
{
    ObjectType type = eUndefined;
    ObjectKey mimicJointKey, referenceJointKey;
    int mimicJointAxis = -1, referenceJointAxis = -1;
    float gearing = 0.0f, offset = 0.0f, naturalFrequency = 0.0f, dampingRatio = 0.0f;
};
void prepareMimicJoints(ParseContext&, ObjectKey, const MimicJointParseInfo&, const JointTypeLookup&,
                        const JointLimitLookup&, std::vector<PreparedMimicJoint>&);
void prepareNewtonMimicJoints(ParseContext&, ObjectKey, const MimicJointParseInfo&, const JointTypeLookup&,
                              std::vector<PreparedMimicJoint>&);
bool fillMimicJoint(const PreparedMimicJoint& input, MimicJointDesc& output) noexcept;
}
