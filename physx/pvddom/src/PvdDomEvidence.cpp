// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PvdDomJson.h"

namespace PvdDomInspect
{

static const char* distanceName(DistanceKind kind)
{
    switch (kind)
    {
    case DistanceKind::Vec3Euclidean: return "Vec3Euclidean";
    case DistanceKind::TransformPositionEuclidean: return "TransformPositionEuclidean";
    case DistanceKind::TransformOrientationRadians: return "TransformOrientationRadians";
    }
    return "Unknown";
}

static std::string comparisonJson(const ComparisonResult& r, const CaptureProvenance& left,
                                  const CaptureProvenance& right, bool typed)
{
    Json json;
    auto& out = json.out;
    const bool different = r.status == ComparisonStatus::Different, equal = r.status == ComparisonStatus::Equal;
    out << "{\"format\":\"physx.ovd.inspection-evidence\",\"version\":" << (typed ? 2 : 1) <<
        ",\"viewerImportSupported\":false,\"status\":";
    json.text(different ? "Different" : equal ? "Equal" : "Incomparable");
    out << ",\"severity\":"; json.text(different ? "error" : equal ? "info" : "warning");
    out << ",\"color\":"; json.text(different ? "#D32F2F" : equal ? "#2E7D32" : "#ED6C02");
    out << ",\"message\":"; json.text(r.reason);
    out << ",\"assumptions\":"; json.text(r.assumptions);
    out << ",\"comparedFrames\":"; json.id(r.comparedFrames);
    out << ",\"metric\":"; json.text(r.metric);
    if (typed)
    {
        out << ",\"measure\":{\"attributeName\":"; json.text(r.measure.attributeName);
        out << ",\"distanceKind\":"; json.text(distanceName(r.measure.distance));
        out << ",\"units\":"; json.text(r.measure.units); out << '}';
        out << ",\"componentCount\":" << r.componentCount;
    }
    out << ",\"units\":"; json.text(r.options.units);
    out << ",\"tolerance\":"; json.number(r.options.tolerance);
    out << ",\"alignment\":{\"leftSegment\":"; json.id(r.options.leftSegment);
    out << ",\"rightSegment\":"; json.id(r.options.rightSegment);
    out << ",\"leftContext\":"; json.id(r.options.leftContext);
    out << ",\"rightContext\":"; json.id(r.options.rightContext);
    out << ",\"startFrame\":"; json.id(r.options.startFrame);
    out << ",\"endFrame\":"; json.id(r.options.endFrame);
    out << ",\"frameStride\":"; json.id(r.options.frameStride);
    out << ",\"rightFrameOffset\":\"" << r.options.rightFrameOffset << "\",\"requireExactSamples\":";
    json.boolean(r.options.requireExactSamples); out << '}';
    out << ",\"leftSelector\":"; json.selector(r.options.leftObject);
    out << ",\"rightSelector\":"; json.selector(r.options.rightObject);
    out << ",\"leftIdentity\":"; if (r.hasLeftIdentity) json.identity(r.leftIdentity); else out << "null";
    out << ",\"rightIdentity\":"; if (r.hasRightIdentity) json.identity(r.rightIdentity); else out << "null";
    out << ",\"leftCapture\":"; json.provenance(left);
    out << ",\"rightCapture\":"; json.provenance(right);
    out << ",\"hasEvidence\":"; json.boolean(r.hasEvidence);
    out << ",\"distance\":"; if (r.hasEvidence) json.number(r.distance); else out << "null";
    if (typed)
    {
        out << ",\"leftValue\":"; if (r.hasValues) json.vector(r.leftValue, r.componentCount); else out << "null";
        out << ",\"rightValue\":"; if (r.hasValues) json.vector(r.rightValue, r.componentCount); else out << "null";
        out << ",\"leftFrameCompleted\":"; if (r.hasLeftFrameCompletion) json.boolean(r.leftFrameCompleted); else out << "null";
        out << ",\"rightFrameCompleted\":"; if (r.hasRightFrameCompletion) json.boolean(r.rightFrameCompleted); else out << "null";
    }
    else
    {
        const bool velocity = r.hasEvidence && r.measure.attributeName == "linearVelocity" &&
            r.measure.distance == DistanceKind::Vec3Euclidean && r.componentCount == 3;
        out << ",\"leftVelocity\":"; if (velocity) json.vector(r.leftValue); else out << "null";
        out << ",\"rightVelocity\":"; if (velocity) json.vector(r.rightValue); else out << "null";
    }
    out << ",\"leftSample\":"; json.sample(r.leftSample);
    out << ",\"rightSample\":"; json.sample(r.rightSample);
    out << ",\"textWasReplaced\":"; json.boolean(json.textWasReplaced);
    out << '}';
    return out.str();
}

std::string evidenceToJson(const ComparisonResult& r, const CaptureProvenance& left, const CaptureProvenance& right)
{ return comparisonJson(r, left, right, false); }

std::string attributeEvidenceToJson(const ComparisonResult& r, const CaptureProvenance& left, const CaptureProvenance& right)
{ return comparisonJson(r, left, right, true); }
}
