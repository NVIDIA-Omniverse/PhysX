// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "PvdDomInspection.h"

namespace PvdDomInspect
{
enum class InspectionValueKind { Missing, FloatingScalar, SignedScalar, UnsignedScalar, Vec3, Transform };
struct InspectionValue
{
    InspectionValueKind kind = InspectionValueKind::Missing;
    double floating = 0;
    int64_t signedValue = 0;
    uint64_t unsignedValue = 0;
    double components[3] = {};
    double quaternion[4] = {};
};
// Internal typed values shared by CLI and Python. Failure leaves result storage unchanged.
bool decodeInspectionValue(const OmniPvdAttributeDef&, const SampleResult&, InspectionValue&, std::string& error);
bool readInspectionValue(const Document&, uint64_t objectId, const FrameRef&, const char* attributeName,
                         SampleResult&, InspectionValue&, std::string& error);
}
