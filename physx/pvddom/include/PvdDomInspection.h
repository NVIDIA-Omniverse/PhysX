// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "PvdDomDocument.h"

namespace PvdDomInspect
{
// Queries cover recorded PxScene memberships. Global resources and context-only streams
// have no implicit scene binding and return an Unsupported diagnostic.
struct FrameRef
{
    uint64_t recordingSegment = 0;
    uint64_t context = 0;
    uint64_t recordedFrame = 0;
};
struct ObjectSelector
{
    // Supply exactly one of objectId and name. IDs are local to one Document.
    uint64_t objectId = 0;
    std::string name;
    std::string className;
};
struct ObjectIdentity
{
    uint64_t objectId = 0;
    uint64_t externalHandle = 0;
    uint64_t recordingSegment = 0;
    std::string className;
    std::string name;
    uint64_t lifetimeStart = 0;
    uint64_t lifetimeEnd = 0;
    bool hasLifetimeEnd = false;
    uint64_t lifetimeStartCommand = 0;
    uint64_t lifetimeEndCommand = 0;
};
enum class SampleState { Exact, Carried, Missing };
struct SampleResult
{
    ObjectIdentity identity;
    FrameRef requested;
    FrameRef source;
    uint64_t rawContext = 0;
    uint64_t rawFrame = 0;
    bool hasRawFrame = false;
    uint64_t commandOrdinal = 0;
    uint64_t sampleTimestamp = 0;
    uint64_t age = 0;
    // Ages are available only for comparable source/requested scene counters.
    bool ageAvailable = false;
    bool hasSourceFrame = false;
    bool preMembership = false;
    SampleState state = SampleState::Missing;
    OmniPvdDataType::Enum dataType = OmniPvdDataType::eINT8;
    const OmniPvdAttributeSample* sample = nullptr;
};

bool selectObject(const Document&, const ObjectSelector&, const FrameRef&, ObjectIdentity&, std::string& error);
bool listObjects(const Document&, const FrameRef&, uint64_t offset, uint64_t limit,
                 std::vector<ObjectIdentity>&, std::string& error);
bool readSample(const Document&, uint64_t objectId, const FrameRef&, const char* attributeName,
                SampleResult&, std::string& error);
// FLOAT32/FLOAT64 only, with exact component count and finite values. Failure leaves output unchanged.
bool readVec3(const SampleResult&, double out[3]);
bool readTransform(const SampleResult&, double positionOut[3], double outQuaternion[4]);
bool quaternionAngularDistance(const double lhs[4], const double rhs[4], double& radians);

struct ComparisonOptions
{
    ObjectSelector leftObject;
    ObjectSelector rightObject;
    uint64_t leftSegment = 0;
    uint64_t rightSegment = 0;
    uint64_t leftContext = 0;
    uint64_t rightContext = 0;
    uint64_t startFrame = 0;
    uint64_t endFrame = 0;
    uint64_t frameStride = 1;
    int64_t rightFrameOffset = 0;
    double tolerance = 0;
    std::string units = "scene_length_units/s";
    bool requireExactSamples = true;
};
enum class ComparisonStatus { Equal, Different, Incomparable };
enum class DistanceKind { Vec3Euclidean, TransformPositionEuclidean, TransformOrientationRadians };
struct ComparisonMeasure
{
    std::string attributeName;
    DistanceKind distance = DistanceKind::Vec3Euclidean;
    std::string units;
};
struct ComparisonResult
{
    ComparisonStatus status = ComparisonStatus::Incomparable;
    uint64_t comparedFrames = 0;
    ComparisonOptions options;
    std::string assumptions;
    std::string reason;
    std::string metric = "linearVelocity.euclidean_distance";
    ComparisonMeasure measure;
    // Both raw values decoded as finite numbers; a quaternion distance may still be invalid.
    bool hasValues = false;
    // A finite distance was measured for this pair.
    bool hasEvidence = false;
    bool hasLeftIdentity = false;
    bool hasRightIdentity = false;
    ObjectIdentity leftIdentity;
    ObjectIdentity rightIdentity;
    double leftValue[4] = {};
    double rightValue[4] = {};
    uint32_t componentCount = 0;
    bool hasLeftFrameCompletion = false;
    bool hasRightFrameCompletion = false;
    bool leftFrameCompleted = false;
    bool rightFrameCompleted = false;
    double distance = 0;
    SampleResult leftSample;
    SampleResult rightSample;
};
ComparisonResult compareAttribute(const Document&, const Document&, const ComparisonOptions&, const ComparisonMeasure&);
ComparisonResult compareLinearVelocity(const Document&, const Document&, const ComparisonOptions&);
struct CaptureProvenance
{
    std::string sourcePath;
    std::string fileHash;
    std::string sdkVersion;
    std::string metadata;
};
// Samples reference Document storage. Serialize before either document is reloaded/destroyed.
// Version 1 velocity fields require a linearVelocity Vec3 measure. Use version 2 for other measures.
std::string evidenceToJson(const ComparisonResult&, const CaptureProvenance& left, const CaptureProvenance& right);
std::string attributeEvidenceToJson(const ComparisonResult&, const CaptureProvenance& left, const CaptureProvenance& right);
}
