// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PvdDomInspectionValue.h"
#include "PvdDomDocumentInternal.h"
#include "PvdDomUtils.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace PvdDomInspect
{
namespace
{
bool queryError(std::string& error, const char* text) { error = text; return false; }

bool checkRecordedFrame(const Document& document, const FrameRef& requested, std::string& error)
{
    if (!document.dom()) return queryError(error, "Document is not loaded");
    const auto found = document.frameIndex().find({requested.recordingSegment, requested.context});
    if (found == document.frameIndex().end()) return queryError(error, "Requested recorded frame/context is absent");
    const auto& index = found->second;
    if (!index.sceneMapped) return queryError(error, "Unsupported frame context without a recorded PxScene mapping");
    if (index.sceneContextRecreated)
        return queryError(error, "Unsupported scene context re-created within recording segment");
    if (!index.monotonic) return queryError(error, "Unsupported repeated or backwards frame numbers in selected segment/context");
    return std::binary_search(index.recordedFrames.begin(), index.recordedFrames.end(), requested.recordedFrame) ||
        queryError(error, "Requested recorded frame/context is absent");
}

const SceneMembership* membershipAt(const ObjectHistory& record, const FrameRef& f)
{
    for (const auto& m : record.memberships)
        if (m.recordingSegment == f.recordingSegment && m.context == f.context && m.startFrame <= f.recordedFrame &&
            (!m.stopOrdinal || f.recordedFrame < m.endFrame)) return &m;
    return nullptr;
}

const OmniPvdAttributeDef* attributeOn(const ObjectHistory& record, const char* name, std::string& error)
{
    const OmniPvdAttributeDef* found = nullptr;
    for (const auto* cls : record.object->mOmniPvdClass->mInheritanceChain)
        for (const auto* attribute : cls->mAttributeDefinitions)
            if (isSameString(attribute->mAttributeName.c_str(), name))
            {
                if (found) { error = "Ambiguous inherited attribute name"; return nullptr; }
                found = attribute;
            }
    return found;
}

const SampleWrite* latestWrite(const Document& document, const ObjectHistory& record, const SceneMembership& member, const FrameRef& f,
                               const OmniPvdAttributeDef* attribute)
{
    const auto found = record.writes.find(attribute->mOmniAttributeHandle);
    if (found == record.writes.end()) return nullptr;
    const auto& writes = found->second;
    const SampleWrite* latest = nullptr;
    // A new scene residency carries the latest state of the same incarnation.
    auto beforeJoin = std::lower_bound(writes.begin(), writes.end(), member.startCommand,
        [](const SampleWrite& w, uint64_t command) { return w.command < command; });
    if (beforeJoin != writes.begin())
    {
        --beforeJoin;
        if (beforeJoin->recordingSegment == f.recordingSegment) latest = &*beforeJoin;
    }
    const auto* scope = DocumentAccess::scopedWrites(document, record, attribute->mOmniAttributeHandle, f.recordingSegment, f.context);
    if (!scope) return latest;
    const auto& indices = *scope;
    auto atFrame = std::upper_bound(indices.begin(), indices.end(), f.recordedFrame,
        [&writes](uint64_t frame, size_t index) { return frame < (writes[index].frameKnown ? writes[index].frame : 0); });
    if (atFrame != indices.begin())
    {
        const auto& write = writes[*--atFrame];
        if ((!member.stopOrdinal || write.command < member.stopOrdinal) && (!latest || write.command > latest->command)) latest = &write;
    }
    return latest;
}

bool identityAt(const Document& document, const ObjectHistory& record, const SceneMembership& m, const FrameRef& f,
                ObjectIdentity& result, std::string& error)
{
    result.objectId = record.object->mUID;
    result.externalHandle = record.object->mOmniAPIHandle;
    result.recordingSegment = m.recordingSegment;
    result.className = record.object->mOmniPvdClass->mClassName;
    result.name = record.object->mOmniObjectName;
    result.lifetimeStart = m.startFrame;
    result.lifetimeEnd = m.endFrame;
    result.hasLifetimeEnd = m.stopOrdinal != 0;
    result.lifetimeStartCommand = m.startCommand;
    result.lifetimeEndCommand = m.stopOrdinal;
    const auto* attribute = attributeOn(record, "name", error);
    if (!error.empty()) return false;
    const auto* write = attribute ? latestWrite(document, record, m, f, attribute) : nullptr;
    if (write && attribute->mDataType == OmniPvdDataType::eSTRING)
    {
        const auto* s = write->sample;
        size_t length = 0;
        while (length < s->mDataLen && s->mData[length]) ++length;
        result.name.assign(length ? reinterpret_cast<const char*>(s->mData) : "", length);
    }
    return true;
}

bool floats(const SampleResult& result, size_t count, double* out)
{
    if (!out || result.state == SampleState::Missing || !result.sample || !result.sample->mData) return false;
    const bool single = result.dataType == OmniPvdDataType::eFLOAT32;
    if (!single && result.dataType != OmniPvdDataType::eFLOAT64) return false;
    const size_t width = single ? sizeof(float) : sizeof(double);
    if (result.sample->mDataLen != count * width) return false;
    double values[7];
    for (size_t i = 0; i < count; ++i)
    {
        if (single)
        {
            float value;
            std::memcpy(&value, result.sample->mData + i * width, width);
            values[i] = value;
        }
        else std::memcpy(&values[i], result.sample->mData + i * width, width);
        if (!std::isfinite(values[i])) return false;
    }
    std::copy(values, values + count, out);
    return true;
}

bool payloadWidth(const SampleResult& sample, size_t width, std::string& error)
{
    return (sample.sample && sample.sample->mData && sample.sample->mDataLen == width) ||
        queryError(error, "Invalid attribute payload width");
}
template<class T> bool integerValue(const SampleResult& sample, InspectionValue& out, std::string& error)
{
    if (!payloadWidth(sample, sizeof(T), error)) return false;
    T value;
    std::memcpy(&value, sample.sample->mData, sizeof(value));
    if (std::numeric_limits<T>::is_signed)
    {
        out.kind = InspectionValueKind::SignedScalar;
        out.signedValue = static_cast<int64_t>(value);
    }
    else
    {
        out.kind = InspectionValueKind::UnsignedScalar;
        out.unsignedValue = static_cast<uint64_t>(value);
    }
    return true;
}

bool normalizeQuaternion(const double* source, double* out)
{
    if (!source) return false;
    double scale = 0;
    for (unsigned i = 0; i != 4; ++i)
    {
        if (!std::isfinite(source[i])) return false;
        scale = (std::max)(scale, std::abs(source[i]));
    }
    if (!scale) return false;
    double squared = 0;
    for (unsigned i = 0; i != 4; ++i) { out[i] = source[i] / scale; squared += out[i] * out[i]; }
    const double length = std::sqrt(squared);
    for (unsigned i = 0; i != 4; ++i) out[i] /= length;
    return true;
}

bool offsetFrame(uint64_t left, int64_t offset, uint64_t& right)
{
    if (offset >= 0)
    {
        if (left > UINT64_MAX - uint64_t(offset)) return false;
        right = left + uint64_t(offset);
    }
    else
    {
        const uint64_t magnitude = uint64_t(-(offset + 1)) + 1;
        if (left < magnitude) return false;
        right = left - magnitude;
    }
    return true;
}
}

bool selectObject(const Document& document, const ObjectSelector& selector, const FrameRef& requested,
                  ObjectIdentity& result, std::string& error)
{
    result = ObjectIdentity(); error.clear();
    if ((selector.objectId != 0) == !selector.name.empty())
        return queryError(error, "Specify exactly one capture-local object ID or exact name");
    if (!checkRecordedFrame(document, requested, error)) return false;
    if (selector.objectId)
    {
        const auto found = document.history().find(selector.objectId);
        if (found == document.history().end()) return queryError(error, "Unknown capture-local object ID");
        if (found->second.memberships.empty()) return queryError(error, "Unsupported object without recorded scene membership");
        if (!selector.className.empty() && found->second.object->mOmniPvdClass->mClassName != selector.className)
            return queryError(error, "Selected object does not match class restriction");
        const auto* member = membershipAt(found->second, requested);
        if (!member) return queryError(error, "Object is absent from selected lifetime/scene");
        return identityAt(document, found->second, *member, requested, result, error);
    }
    bool found = false;
    for (const auto& entry : document.history())
    {
        if (selector.objectId && selector.objectId != entry.first) continue;
        const auto& record = entry.second;
        if (!selector.className.empty() && record.object->mOmniPvdClass->mClassName != selector.className) continue;
        const auto* member = membershipAt(record, requested);
        if (!member) continue;
        ObjectIdentity candidate;
        if (!identityAt(document, record, *member, requested, candidate, error)) return false;
        if (!selector.name.empty() && candidate.name != selector.name) continue;
        if (found) { result = ObjectIdentity(); return queryError(error, "Object name is ambiguous in selected scope"); }
        result = candidate; found = true;
    }
    return found || queryError(error, "Object is absent from selected lifetime/scene");
}

bool listObjects(const Document& document, const FrameRef& requested, uint64_t offset, uint64_t limit,
                 std::vector<ObjectIdentity>& result, std::string& error)
{
    result.clear(); error.clear();
    if (!checkRecordedFrame(document, requested, error)) return false;
    for (const auto& entry : document.history())
    {
        const auto* member = membershipAt(entry.second, requested);
        if (!member) continue;
        if (offset) { --offset; continue; }
        if (result.size() >= limit) break;
        ObjectIdentity object;
        if (!identityAt(document, entry.second, *member, requested, object, error)) { result.clear(); return false; }
        result.push_back(object);
    }
    return true;
}

bool readSample(const Document& document, uint64_t objectId, const FrameRef& requested, const char* name,
                SampleResult& result, std::string& error)
{
    result = SampleResult(); result.requested = requested; error.clear();
    if (!name || !*name) return queryError(error, "Missing attribute name");
    if (!checkRecordedFrame(document, requested, error)) return false;
    const auto found = document.history().find(objectId);
    if (found == document.history().end()) return queryError(error, "Unknown capture-local object ID");
    const auto& record = found->second;
    if (record.memberships.empty()) return queryError(error, "Unsupported object without recorded scene membership");
    const auto* member = membershipAt(record, requested);
    if (!member) return queryError(error, "Object is absent from selected lifetime/scene");
    if (!identityAt(document, record, *member, requested, result.identity, error)) return false;
    const auto* attribute = attributeOn(record, name, error);
    if (!attribute) return !error.empty() ? false : queryError(error, "Unknown attribute name");
    if (attribute->mIsUniqueList || attribute->mDataType == OmniPvdDataType::eOBJECT_HANDLE)
        return queryError(error, "Unsupported unique-list or historical object-reference query");
    result.dataType = static_cast<OmniPvdDataType::Enum>(attribute->mDataType);
    const auto* write = latestWrite(document, record, *member, requested, attribute);
    if (!write) return true;
    result.source.recordingSegment = write->recordingSegment;
    result.source.context = write->context;
    if (write->frameKnown) result.source.recordedFrame = write->frame;
    result.rawContext = write->rawContext;
    result.rawFrame = write->rawFrame;
    result.hasRawFrame = write->hasRawFrame;
    result.hasSourceFrame = write->frameKnown;
    result.commandOrdinal = write->command;
    result.sample = write->sample;
    result.sampleTimestamp = write->sample->mTimeStamp;
    result.preMembership = write->command < member->startCommand;
    const bool exact = !result.preMembership && write->frameKnown && write->context == requested.context &&
        write->frame == requested.recordedFrame;
    result.state = exact ? SampleState::Exact : SampleState::Carried;
    result.ageAvailable = write->frameKnown && write->context == requested.context && write->frame <= requested.recordedFrame;
    if (result.ageAvailable) result.age = requested.recordedFrame - write->frame;
    return true;
}

bool decodeInspectionValue(const OmniPvdAttributeDef& attribute, const SampleResult& sample,
                           InspectionValue& out, std::string& error)
{
    error.clear();
    InspectionValue value;
    if (sample.state == SampleState::Missing) { out = value; return true; }
    const auto type = static_cast<OmniPvdDataType::Enum>(attribute.mDataType);
    if (attribute.mIsUniqueList || type != sample.dataType)
        return queryError(error, "Unsupported attribute type");
    const bool floating = type == OmniPvdDataType::eFLOAT32 || type == OmniPvdDataType::eFLOAT64;
    if (floating)
    {
        const size_t width = type == OmniPvdDataType::eFLOAT32 ? sizeof(float) : sizeof(double);
        if (attribute.mNbrFields == 1)
        {
            if (!payloadWidth(sample, width, error)) return false;
            if (type == OmniPvdDataType::eFLOAT32)
            {
                float scalar;
                std::memcpy(&scalar, sample.sample->mData, sizeof(scalar));
                value.floating = scalar;
            }
            else std::memcpy(&value.floating, sample.sample->mData, sizeof(value.floating));
            if (!std::isfinite(value.floating)) return queryError(error, "Nonfinite attribute value");
            value.kind = InspectionValueKind::FloatingScalar;
        }
        else
        {
            size_t fields = attribute.mNbrFields;
            if (!fields && sample.sample)
            {
                if (sample.sample->mDataLen == 3 * width) fields = 3;
                else if (sample.sample->mDataLen == 7 * width) fields = 7;
            }
            if (fields != 3 && fields != 7) return queryError(error, "Unsupported attribute declaration shape");
            if (!payloadWidth(sample, fields * width, error)) return false;
            const bool valid = fields == 3 ? readVec3(sample, value.components) :
                readTransform(sample, value.components, value.quaternion);
            if (!valid) return queryError(error, "Nonfinite attribute value");
            value.kind = fields == 3 ? InspectionValueKind::Vec3 : InspectionValueKind::Transform;
        }
    }
    else if (type == OmniPvdDataType::eFLAGS_WORD)
    {
        const uint32_t size = sample.sample ? sample.sample->mDataLen : 0;
        if (size == 1) { if (!integerValue<uint8_t>(sample, value, error)) return false; }
        else if (size == 2) { if (!integerValue<uint16_t>(sample, value, error)) return false; }
        else if (size == 4) { if (!integerValue<uint32_t>(sample, value, error)) return false; }
        else return queryError(error, "Invalid attribute payload width");
    }
    else
    {
        if (type < OmniPvdDataType::eINT8 || type > OmniPvdDataType::eUINT64)
            return queryError(error, "Unsupported attribute type");
        if (attribute.mNbrFields != 1) return queryError(error, "Unsupported attribute declaration shape");
        bool valid = false;
        switch (type)
        {
        case OmniPvdDataType::eINT8: valid = integerValue<int8_t>(sample, value, error); break;
        case OmniPvdDataType::eINT16: valid = integerValue<int16_t>(sample, value, error); break;
        case OmniPvdDataType::eINT32: valid = integerValue<int32_t>(sample, value, error); break;
        case OmniPvdDataType::eINT64: valid = integerValue<int64_t>(sample, value, error); break;
        case OmniPvdDataType::eUINT8: valid = integerValue<uint8_t>(sample, value, error); break;
        case OmniPvdDataType::eUINT16: valid = integerValue<uint16_t>(sample, value, error); break;
        case OmniPvdDataType::eUINT32: valid = integerValue<uint32_t>(sample, value, error); break;
        case OmniPvdDataType::eUINT64: valid = integerValue<uint64_t>(sample, value, error); break;
        default: return queryError(error, "Unsupported attribute type");
        }
        if (!valid) return false;
    }
    out = value;
    return true;
}

bool readInspectionValue(const Document& document, uint64_t objectId, const FrameRef& requested, const char* name,
                         SampleResult& sample, InspectionValue& value, std::string& error)
{
    SampleResult selected;
    if (!readSample(document, objectId, requested, name, selected, error)) return false;
    const auto found = document.history().find(objectId);
    if (found == document.history().end()) return queryError(error, "Unknown capture-local object ID");
    const auto* attribute = attributeOn(found->second, name, error);
    if (!attribute) return !error.empty() ? false : queryError(error, "Unknown attribute name");
    InspectionValue decoded;
    if (!decodeInspectionValue(*attribute, selected, decoded, error)) return false;
    sample = selected;
    value = decoded;
    return true;
}

bool readVec3(const SampleResult& result, double out[3]) { return floats(result, 3, out); }
bool readTransform(const SampleResult& result, double positionOut[3], double outQuaternion[4])
{
    double values[7];
    if (!positionOut || !outQuaternion || !floats(result, 7, values)) return false;
    std::copy(values, values + 4, outQuaternion);
    std::copy(values + 4, values + 7, positionOut);
    return true;
}
bool quaternionAngularDistance(const double lhs[4], const double rhs[4], double& radians)
{
    double a[4], b[4];
    if (!normalizeQuaternion(lhs, a) || !normalizeQuaternion(rhs, b)) return false;
    double dot = 0, sum = 0, difference = 0;
    for (unsigned i = 0; i != 4; ++i) dot += a[i] * b[i];
    for (unsigned i = 0; i != 4; ++i)
    {
        if (dot < 0) b[i] = -b[i];
        const double d = a[i] - b[i], s = a[i] + b[i];
        difference += d * d; sum += s * s;
    }
    radians = 4 * std::atan2(std::sqrt(difference), std::sqrt(sum));
    return true;
}

ComparisonResult compareAttribute(const Document& left, const Document& right, const ComparisonOptions& options,
                                  const ComparisonMeasure& measure)
{
    ComparisonResult result;
    result.options = options;
    result.options.units = measure.units;
    result.measure = measure;
    result.metric = measure.attributeName + ".distance";
    result.assumptions = "Selectors are capture-local IDs or exact names resolved once at the first aligned frame. "
        "Those incarnations are pinned across the range, including later renames. External handles are not matched. "
        "rightFrame = leftFrame + explicit offset. "
        "Frames are recorded START_FRAME intervals, including writes after STOP_FRAME, not application steps. "
        "Equal means requested sampled values agree, not that either simulation finished. Units are caller-supplied. ";
    result.assumptions += options.requireExactSamples ? "Only exact writes are compared." : "Carried state is explicitly allowed.";
    if (measure.distance == DistanceKind::TransformOrientationRadians)
        result.assumptions += " Quaternion distance normalizes finite nonzero xyzw values and treats q and -q as equivalent; evidence keeps raw values.";
    bool attemptedFrame = false;
    auto finish = [&]() -> ComparisonResult {
        if (attemptedFrame)
        {
            for (const auto& frame : left.frames())
                if (frame.recordingSegment == result.leftSample.requested.recordingSegment &&
                    frame.context == result.leftSample.requested.context &&
                    frame.recordedFrame == result.leftSample.requested.recordedFrame)
                { result.hasLeftFrameCompletion = true; result.leftFrameCompleted = frame.completed; break; }
            for (const auto& frame : right.frames())
                if (frame.recordingSegment == result.rightSample.requested.recordingSegment &&
                    frame.context == result.rightSample.requested.context &&
                    frame.recordedFrame == result.rightSample.requested.recordedFrame)
                { result.hasRightFrameCompletion = true; result.rightFrameCompleted = frame.completed; break; }
        }
        return result;
    };
    if (left.loadInfo().recoveredTail || right.loadInfo().recoveredTail)
    { result.reason = "Recovered-tail documents cannot be compared"; return finish(); }
    if (measure.attributeName.empty() || measure.units.empty() ||
        (measure.distance != DistanceKind::Vec3Euclidean &&
         measure.distance != DistanceKind::TransformPositionEuclidean &&
         measure.distance != DistanceKind::TransformOrientationRadians) ||
        (measure.distance == DistanceKind::TransformOrientationRadians && measure.units != "radians"))
    { result.reason = "Invalid comparison measure or units"; return finish(); }
    if (!options.frameStride || options.startFrame > options.endFrame || !std::isfinite(options.tolerance) || options.tolerance < 0)
    { result.reason = "Invalid frame range, stride or tolerance"; return finish(); }
    uint64_t leftId = 0, rightId = 0;
    for (uint64_t f = options.startFrame;;)
    {
        result.hasEvidence = false;
        result.hasValues = false;
        result.distance = 0;
        std::fill(result.leftValue, result.leftValue + 4, 0);
        std::fill(result.rightValue, result.rightValue + 4, 0);
        FrameRef lf, rf;
        lf.recordingSegment = options.leftSegment; lf.context = options.leftContext; lf.recordedFrame = f;
        rf.recordingSegment = options.rightSegment; rf.context = options.rightContext;
        if (!offsetFrame(f, options.rightFrameOffset, rf.recordedFrame))
        { result.reason = "Recorded-frame offset overflows or underflows"; return finish(); }
        result.leftSample = SampleResult(); result.rightSample = SampleResult();
        result.leftSample.requested = lf; result.rightSample.requested = rf;
        result.hasLeftFrameCompletion = result.hasRightFrameCompletion = false;
        result.leftFrameCompleted = result.rightFrameCompleted = false;
        result.componentCount = 0;
        attemptedFrame = true;
        if (!leftId)
        {
            if (!selectObject(left, options.leftObject, lf, result.leftIdentity, result.reason)) return finish();
            result.hasLeftIdentity = true;
            leftId = result.leftIdentity.objectId;
            if (!selectObject(right, options.rightObject, rf, result.rightIdentity, result.reason)) return finish();
            result.hasRightIdentity = true;
            rightId = result.rightIdentity.objectId;
        }
        std::string leftError, rightError;
        const bool leftRead = readSample(left, leftId, lf, measure.attributeName.c_str(), result.leftSample, leftError);
        const bool rightRead = readSample(right, rightId, rf, measure.attributeName.c_str(), result.rightSample, rightError);
        if (!leftRead || !rightRead)
        {
            result.reason = !leftRead ? "left capture: " + leftError : std::string();
            if (!rightRead) result.reason += (!leftRead ? "; right capture: " : "right capture: ") + rightError;
            return finish();
        }
        if (options.requireExactSamples &&
            (result.leftSample.state != SampleState::Exact || result.rightSample.state != SampleState::Exact))
        { result.reason = "Missing exact attribute write at aligned recorded frame"; return finish(); }
        double leftPosition[3], rightPosition[3], leftQuaternion[4], rightQuaternion[4];
        if (measure.distance == DistanceKind::Vec3Euclidean)
        {
            double leftDecodedValues[3], rightDecodedValues[3];
            const bool leftDecoded = readVec3(result.leftSample, leftDecodedValues);
            const bool rightDecoded = readVec3(result.rightSample, rightDecodedValues);
            if (!leftDecoded || !rightDecoded)
            {
                result.reason = std::string("Missing, nonfinite or incorrectly typed ") +
                    (!leftDecoded && !rightDecoded ? "left and right" : !leftDecoded ? "left" : "right") + " attribute value";
                return finish();
            }
            result.componentCount = 3;
            std::copy(leftDecodedValues, leftDecodedValues + 3, result.leftValue);
            std::copy(rightDecodedValues, rightDecodedValues + 3, result.rightValue);
        }
        else
        {
            const bool leftDecoded = readTransform(result.leftSample, leftPosition, leftQuaternion);
            const bool rightDecoded = readTransform(result.rightSample, rightPosition, rightQuaternion);
            if (!leftDecoded || !rightDecoded)
            {
                result.reason = std::string("Missing, nonfinite or incorrectly typed ") +
                    (!leftDecoded && !rightDecoded ? "left and right" : !leftDecoded ? "left" : "right") + " transform value";
                return finish();
            }
            const bool orientation = measure.distance == DistanceKind::TransformOrientationRadians;
            result.componentCount = orientation ? 4 : 3;
            std::copy(orientation ? leftQuaternion : leftPosition,
                      (orientation ? leftQuaternion : leftPosition) + result.componentCount, result.leftValue);
            std::copy(orientation ? rightQuaternion : rightPosition,
                      (orientation ? rightQuaternion : rightPosition) + result.componentCount, result.rightValue);
        }
        result.hasValues = true;
        double distance = 0;
        if (measure.distance == DistanceKind::TransformOrientationRadians)
        {
            if (!quaternionAngularDistance(result.leftValue, result.rightValue, distance))
            {
                double normalized[4];
                const bool leftQuaternionValid = normalizeQuaternion(result.leftValue, normalized);
                const bool rightQuaternionValid = normalizeQuaternion(result.rightValue, normalized);
                result.reason = !leftQuaternionValid && !rightQuaternionValid ? "Invalid left and right transform quaternions" :
                    !leftQuaternionValid ? "Invalid left transform quaternion" : "Invalid right transform quaternion";
                return finish();
            }
        }
        else
            for (unsigned i = 0; i != 3; ++i) distance = std::hypot(distance, result.leftValue[i] - result.rightValue[i]);
        if (!std::isfinite(distance)) { result.reason = "Attribute distance is not finite"; return finish(); }
        result.distance = distance;
        result.hasEvidence = true;
        ++result.comparedFrames;
        if (distance > options.tolerance)
        {
            result.status = ComparisonStatus::Different;
            result.reason = "First aligned attribute distance exceeding tolerance";
            return finish();
        }
        if (options.endFrame - f < options.frameStride) break;
        f += options.frameStride;
    }
    result.status = ComparisonStatus::Equal;
    result.reason = "All requested aligned attribute samples are within tolerance";
    return finish();
}

ComparisonResult compareLinearVelocity(const Document& left, const Document& right, const ComparisonOptions& options)
{
    ComparisonMeasure measure;
    measure.attributeName = "linearVelocity";
    measure.distance = DistanceKind::Vec3Euclidean;
    measure.units = options.units;
    ComparisonResult result = compareAttribute(left, right, options, measure);
    result.metric = "linearVelocity.euclidean_distance";
    if (result.status == ComparisonStatus::Different)
        result.reason = "First aligned velocity distance exceeding tolerance";
    else if (result.status == ComparisonStatus::Equal)
        result.reason = "All requested aligned velocity samples are within tolerance";
    return result;
}
}
