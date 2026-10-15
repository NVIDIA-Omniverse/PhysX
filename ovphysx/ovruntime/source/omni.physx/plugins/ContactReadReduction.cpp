// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CONTACT-001
 * @covers AC-3, AC-12
 *
 * @implements REQ-READ-CONTACT-003
 * @covers AC-1
 *
 * @implements REQ-READ-CONTACT-004
 * @covers AC-1
 *
 * @implements REQ-READ-CONTACT-005
 * @covers AC-1
 *
 * @implements REQ-READ-CONTACT-006
 * @covers AC-1
 */

#include "ContactReadReduction.h"

#include <array>
#include <cmath>
#include <unordered_map>

namespace omni
{
namespace physx
{
namespace contactread
{
namespace
{
using omni::physics::parse::ObjectKey;
using omni::physx::tensors::ForceComponent;

// Sensor rows of a header side: its actor's row AND its colliding shape's row, among the rows' keys. A
// reporting body and its reporting collider are distinct rows, so a pair is attributed to BOTH -- the body
// keeps its aggregate and the collider reads its own shape's contacts (ADR-0044 2026-10-05 amendment). A
// single-prim body whose actor and collider share one key is one row, counted once.
class SensorRows
{
public:
    explicit SensorRows(const std::vector<ObjectKey>& sensors)
    {
        mRow.reserve(sensors.size());
        for (uint32_t i = 0; i < static_cast<uint32_t>(sensors.size()); ++i)
            mRow.emplace(sensors[i].handle, i);
    }
    // The distinct rows a side lands in -- its actor's first (the per-body aggregate), then its colliding
    // shape's -- written to `out`; returns how many (0, 1 or 2).
    int rowsFor(uint64_t actor, uint64_t collider, int64_t (&out)[2]) const
    {
        int n = 0;
        const int64_t a = lookup(actor);
        if (a >= 0)
            out[n++] = a;
        if (collider != actor)
        {
            const int64_t c = lookup(collider);
            if (c >= 0)
                out[n++] = c;
        }
        return n;
    }

private:
    int64_t lookup(uint64_t id) const
    {
        const std::unordered_map<uint64_t, uint32_t>::const_iterator it = mRow.find(id);
        return it == mRow.end() ? -1 : static_cast<int64_t>(it->second);
    }
    std::unordered_map<uint64_t, uint32_t> mRow;
};

// One pair's impulse, summed over its contact points (normal) or its friction anchors (friction).
void pairImpulse(const ContactEventHeader& header, const ContactReportView& report, bool friction, float (&sum)[3])
{
    sum[0] = sum[1] = sum[2] = 0.0f;
    const uint32_t count = friction ? header.numfrictionAnchorsData : header.numContactData;
    for (uint32_t k = 0; k < count; ++k)
    {
        const carb::Float3& imp = friction ? report.anchors[header.frictionAnchorsDataOffset + k].impulse
                                           : report.points[header.contactDataOffset + k].impulse;
        sum[0] += imp.x;
        sum[1] += imp.y;
        sum[2] += imp.z;
    }
}

bool hasComponent(const ContactReportView& report, bool friction)
{
    return report.headers && (friction ? report.anchors != nullptr : report.points != nullptr);
}

// Each row's filter objects -> their column. Duplicates are refused at query open, so one id names at most
// one column of a row.
class FilterColumns
{
public:
    FilterColumns(size_t rows, const std::vector<ObjectKey>& filters, uint32_t filterCount) : mColumns(rows)
    {
        for (size_t i = 0; i < rows; ++i)
            for (uint32_t f = 0; f < filterCount; ++f)
                if (filters[i * filterCount + f].valid())
                    mColumns[i].emplace(filters[i * filterCount + f].handle, f);
    }
    // The column of the other side's body in `row`'s filters -- its actor id, or failing that its collider
    // id -- or -1 when neither is a filter.
    int64_t find(int64_t row, uint64_t actor, uint64_t collider) const
    {
        const std::unordered_map<uint64_t, uint32_t>& c = mColumns[size_t(row)];
        std::unordered_map<uint64_t, uint32_t>::const_iterator it = c.find(actor);
        if (it == c.end() && collider != actor)
            it = c.find(collider);
        return it == c.end() ? -1 : static_cast<int64_t>(it->second);
    }

private:
    std::vector<std::unordered_map<uint64_t, uint32_t>> mColumns;
};

void addScaled(float* dst, const float (&sum)[3], float sign)
{
    dst[0] += sign * sum[0];
    dst[1] += sign * sum[1];
    dst[2] += sign * sum[2];
}

// A filtered-detail slot a header's sensor side lands in: [row, column], and whether the side is 1 (its forces
// negated).
struct DetailTarget
{
    size_t slot;
    bool invert;
};

// The slots a header lands in -- one per (sensor row, side) whose other side is one of that row's filters;
// returns how many (up to four: each side's actor row and collider row). Each row keys its own filter list.
int detailTargets(const SensorRows& rows,
                  const FilterColumns& columns,
                  uint32_t filterCount,
                  const ContactEventHeader& header,
                  DetailTarget (&t)[4])
{
    int n = 0;
    int64_t r0[2];
    const int n0 = rows.rowsFor(header.actor0, header.collider0, r0);
    for (int i = 0; i < n0; ++i)
    {
        const int64_t col = columns.find(r0[i], header.actor1, header.collider1);
        if (col >= 0)
            t[n++] = { size_t(r0[i]) * filterCount + size_t(col), false };
    }
    int64_t r1[2];
    const int n1 = rows.rowsFor(header.actor1, header.collider1, r1);
    for (int i = 0; i < n1; ++i)
    {
        const int64_t col = columns.find(r1[i], header.actor0, header.collider0);
        if (col >= 0)
            t[n++] = { size_t(r1[i]) * filterCount + size_t(col), true };
    }
    return n;
}

// The filtered detail's count pass: each slot's contact points or friction anchors, lost pairs skipped.
std::vector<int32_t> countDetail(const SensorRows& rows,
                                 const FilterColumns& columns,
                                 size_t sensorCount,
                                 uint32_t filterCount,
                                 const ContactReportView& report,
                                 bool friction)
{
    std::vector<int32_t> counts(sensorCount * filterCount, 0);
    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        if (header.type == ContactEventType::eCONTACT_LOST)
            continue;
        DetailTarget t[4];
        const int n = detailTargets(rows, columns, filterCount, header, t);
        const uint32_t elements = friction ? header.numfrictionAnchorsData : header.numContactData;
        for (int k = 0; k < n; ++k)
            counts[t[k].slot] += static_cast<int32_t>(elements);
    }
    return counts;
}
} // namespace

std::vector<float> reduceNetForces(const std::vector<ObjectKey>& sensors,
                                   const ContactReportView& report,
                                   float invDt,
                                   ForceComponent component)
{
    const bool friction = component == ForceComponent::eFriction;
    std::vector<float> out(sensors.size() * 3u, 0.0f);
    if (hasComponent(report, friction))
    {
        const SensorRows rows(sensors);
        for (uint32_t h = 0; h < report.numHeaders; ++h)
        {
            const ContactEventHeader& header = report.headers[h];
            if (header.type == ContactEventType::eCONTACT_LOST)
                continue; // a lost pair carries no live impulse
            int64_t r0[2];
            const int n0 = rows.rowsFor(header.actor0, header.collider0, r0);
            int64_t r1[2];
            const int n1 = rows.rowsFor(header.actor1, header.collider1, r1);
            if (n0 == 0 && n1 == 0)
                continue;
            float sum[3];
            pairImpulse(header, report, friction, sum);
            for (int i = 0; i < n0; ++i)
                addScaled(&out[size_t(r0[i]) * 3u], sum, 1.0f);
            for (int i = 0; i < n1; ++i)
                addScaled(&out[size_t(r1[i]) * 3u], sum, -1.0f);
        }
    }
    for (float& v : out)
        v *= invDt;
    return out;
}

std::vector<float> reduceForceMatrix(const std::vector<ObjectKey>& sensors,
                                     const std::vector<ObjectKey>& filters,
                                     uint32_t filterCount,
                                     const ContactReportView& report,
                                     float invDt,
                                     ForceComponent component)
{
    const bool friction = component == ForceComponent::eFriction;
    const size_t rowFloats = size_t(filterCount) * 3u;
    std::vector<float> out(sensors.size() * rowFloats, 0.0f);
    if (filterCount == 0 || !hasComponent(report, friction))
        return out;

    const FilterColumns columns(sensors.size(), filters, filterCount);
    const SensorRows rows(sensors);
    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        if (header.type == ContactEventType::eCONTACT_LOST)
            continue;
        DetailTarget t[4];
        const int n = detailTargets(rows, columns, filterCount, header, t);
        if (n == 0)
            continue;
        float sum[3];
        pairImpulse(header, report, friction, sum);
        for (int k = 0; k < n; ++k)
            addScaled(&out[t[k].slot * 3u], sum, t[k].invert ? -1.0f : 1.0f);
    }
    for (float& v : out)
        v *= invDt;
    return out;
}

ContactElements reduceFilteredDetail(const std::vector<ObjectKey>& sensors,
                                     const std::vector<ObjectKey>& filters,
                                     uint32_t filterCount,
                                     const ContactReportView& report,
                                     float invDt,
                                     ForceComponent component)
{
    const bool friction = component == ForceComponent::eFriction;
    ContactElements out;
    if (filterCount == 0 || !hasComponent(report, friction))
    {
        out.counts.assign(sensors.size() * filterCount, 0);
        return out;
    }

    // Count pass, then the slots' starts, then the fill -- the binding's two passes, with the total known.
    const FilterColumns columns(sensors.size(), filters, filterCount);
    const SensorRows rows(sensors);
    out.counts = countDetail(rows, columns, sensors.size(), filterCount, report, friction);
    std::vector<size_t> next(out.counts.size());
    size_t total = 0;
    for (size_t slot = 0; slot < out.counts.size(); ++slot)
    {
        next[slot] = total;
        total += size_t(out.counts[slot]);
    }
    out.force.assign(total * (friction ? 3u : 1u), 0.0f);
    out.point.assign(total * 3u, 0.0f);
    if (!friction)
    {
        out.normal.assign(total * 3u, 0.0f);
        out.separation.assign(total, 0.0f);
    }

    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        if (header.type == ContactEventType::eCONTACT_LOST)
            continue;
        DetailTarget t[4];
        const int n = detailTargets(rows, columns, filterCount, header, t);
        for (int k = 0; k < n; ++k)
        {
            const float sign = t[k].invert ? -1.0f : 1.0f;
            if (friction)
            {
                for (uint32_t a = 0; a < header.numfrictionAnchorsData; ++a)
                {
                    const FrictionAnchor& anchor = report.anchors[header.frictionAnchorsDataOffset + a];
                    const size_t e = next[t[k].slot]++;
                    out.force[e * 3 + 0] = sign * anchor.impulse.x * invDt;
                    out.force[e * 3 + 1] = sign * anchor.impulse.y * invDt;
                    out.force[e * 3 + 2] = sign * anchor.impulse.z * invDt;
                    out.point[e * 3 + 0] = anchor.position.x;
                    out.point[e * 3 + 1] = anchor.position.y;
                    out.point[e * 3 + 2] = anchor.position.z;
                }
            }
            else
            {
                for (uint32_t p = 0; p < header.numContactData; ++p)
                {
                    const ContactData& cd = report.points[header.contactDataOffset + p];
                    const size_t e = next[t[k].slot]++;
                    const float magnitude = std::sqrt(cd.impulse.x * cd.impulse.x + cd.impulse.y * cd.impulse.y +
                                                      cd.impulse.z * cd.impulse.z);
                    out.force[e] = sign * magnitude * invDt;
                    out.point[e * 3 + 0] = cd.position.x;
                    out.point[e * 3 + 1] = cd.position.y;
                    out.point[e * 3 + 2] = cd.position.z;
                    out.normal[e * 3 + 0] = cd.normal.x;
                    out.normal[e * 3 + 1] = cd.normal.y;
                    out.normal[e * 3 + 2] = cd.normal.z;
                    out.separation[e] = cd.separation;
                }
            }
        }
    }
    return out;
}

std::vector<int32_t> countFilteredDetail(const std::vector<ObjectKey>& sensors,
                                         const std::vector<ObjectKey>& filters,
                                         uint32_t filterCount,
                                         const ContactReportView& report,
                                         ForceComponent component)
{
    const bool friction = component == ForceComponent::eFriction;
    if (filterCount == 0 || !hasComponent(report, friction))
        return std::vector<int32_t>(sensors.size() * filterCount, 0);
    const FilterColumns columns(sensors.size(), filters, filterCount);
    const SensorRows rows(sensors);
    return countDetail(rows, columns, sensors.size(), filterCount, report, friction);
}

ContactElements reduceRawContacts(const std::vector<ObjectKey>& sensors, const ContactReportView& report, float invDt)
{
    ContactElements out;
    out.counts.assign(sensors.size(), 0);
    if (!hasComponent(report, false))
        return out;

    const SensorRows rows(sensors);
    // Count pass, then each row's start, then the fill.
    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        if (header.type == ContactEventType::eCONTACT_LOST)
            continue;
        int64_t r0[2];
        const int n0 = rows.rowsFor(header.actor0, header.collider0, r0);
        for (int i = 0; i < n0; ++i)
            out.counts[size_t(r0[i])] += static_cast<int32_t>(header.numContactData);
        int64_t r1[2];
        const int n1 = rows.rowsFor(header.actor1, header.collider1, r1);
        for (int i = 0; i < n1; ++i)
            out.counts[size_t(r1[i])] += static_cast<int32_t>(header.numContactData);
    }
    std::vector<size_t> next(out.counts.size());
    size_t total = 0;
    for (size_t row = 0; row < out.counts.size(); ++row)
    {
        next[row] = total;
        total += size_t(out.counts[row]);
    }
    out.force.assign(total, 0.0f);
    out.point.assign(total * 3u, 0.0f);
    out.normal.assign(total * 3u, 0.0f);
    out.separation.assign(total, 0.0f);
    out.other.assign(total, 0u);

    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        if (header.type == ContactEventType::eCONTACT_LOST)
            continue;
        int64_t sideRows[2][2];
        const int sideN[2] = { rows.rowsFor(header.actor0, header.collider0, sideRows[0]),
                               rows.rowsFor(header.actor1, header.collider1, sideRows[1]) };
        for (int side = 0; side < 2; ++side)
        {
            const float sign = side ? -1.0f : 1.0f;
            const uint64_t other = side ? header.actor0 : header.actor1;
            for (int ri = 0; ri < sideN[side]; ++ri)
                for (uint32_t p = 0; p < header.numContactData; ++p)
                {
                    const ContactData& cd = report.points[header.contactDataOffset + p];
                    const size_t e = next[size_t(sideRows[side][ri])]++;
                    const float magnitude = std::sqrt(cd.impulse.x * cd.impulse.x + cd.impulse.y * cd.impulse.y +
                                                      cd.impulse.z * cd.impulse.z);
                    out.force[e] = sign * magnitude * invDt;
                    out.point[e * 3 + 0] = cd.position.x;
                    out.point[e * 3 + 1] = cd.position.y;
                    out.point[e * 3 + 2] = cd.position.z;
                    out.normal[e * 3 + 0] = cd.normal.x;
                    out.normal[e * 3 + 1] = cd.normal.y;
                    out.normal[e * 3 + 2] = cd.normal.z;
                    out.separation[e] = cd.separation;
                    out.other[e] = other;
                }
        }
    }
    return out;
}

ContactEvents reduceContactEvents(const std::vector<ObjectKey>& sensors, const ContactReportView& report)
{
    ContactEvents out;
    out.eventCounts.assign(sensors.size(), 0);
    out.pointCounts.assign(sensors.size(), 0);
    out.anchorCounts.assign(sensors.size(), 0);
    if (!report.headers)
        return out;

    // The (row, side) targets each header lands in -- its actor row and collider row on each side, up to four
    // -- computed once, used by the count and fill passes. A body and its reporting collider both get the
    // event, as seen from that side (own and other).
    const SensorRows rows(sensors);
    struct EventTarget
    {
        int64_t row;
        int side;
    };
    std::vector<std::array<EventTarget, 4>> targets(report.numHeaders);
    std::vector<int> targetN(report.numHeaders, 0);
    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        int m = 0;
        int64_t r0[2];
        const int n0 = rows.rowsFor(header.actor0, header.collider0, r0);
        for (int i = 0; i < n0; ++i)
            targets[h][m++] = { r0[i], 0 };
        int64_t r1[2];
        const int n1 = rows.rowsFor(header.actor1, header.collider1, r1);
        for (int i = 0; i < n1; ++i)
            targets[h][m++] = { r1[i], 1 };
        targetN[h] = m;
        for (int k = 0; k < m; ++k)
        {
            const size_t row = size_t(targets[h][k].row);
            ++out.eventCounts[row];
            out.pointCounts[row] += report.points ? static_cast<int32_t>(header.numContactData) : 0;
            out.anchorCounts[row] += report.anchors ? static_cast<int32_t>(header.numfrictionAnchorsData) : 0;
        }
    }
    // Each row's first event, point and anchor.
    const size_t rowCount = sensors.size();
    std::vector<size_t> nextEvent(rowCount), nextPoint(rowCount), nextAnchor(rowCount);
    size_t events = 0, points = 0, anchors = 0;
    for (size_t row = 0; row < rowCount; ++row)
    {
        nextEvent[row] = events;
        nextPoint[row] = points;
        nextAnchor[row] = anchors;
        events += size_t(out.eventCounts[row]);
        points += size_t(out.pointCounts[row]);
        anchors += size_t(out.anchorCounts[row]);
    }
    out.type.resize(events);
    out.sensorSide.resize(events);
    out.collider.resize(events);
    out.otherBody.resize(events);
    out.otherCollider.resize(events);
    out.protoIndex.resize(events);
    out.otherProtoIndex.resize(events);
    out.pointCount.resize(events);
    out.anchorCount.resize(events);
    out.pointPosition.resize(points * 3u);
    out.pointNormal.resize(points * 3u);
    out.pointImpulse.resize(points * 3u);
    out.pointSeparation.resize(points);
    out.pointFaceIndex.resize(points);
    out.pointOtherFaceIndex.resize(points);
    out.pointMaterial.resize(points);
    out.pointOtherMaterial.resize(points);
    out.anchorPosition.resize(anchors * 3u);
    out.anchorImpulse.resize(anchors * 3u);

    const auto put3 = [](std::vector<float>& dst, size_t i, const carb::Float3& v)
    {
        dst[i * 3 + 0] = v.x;
        dst[i * 3 + 1] = v.y;
        dst[i * 3 + 2] = v.z;
    };
    for (uint32_t h = 0; h < report.numHeaders; ++h)
    {
        const ContactEventHeader& header = report.headers[h];
        for (int k = 0; k < targetN[h]; ++k)
        {
            const size_t row = size_t(targets[h][k].row);
            const int side = targets[h][k].side;
            const bool own0 = side == 0; // the row is side 0: own = 0, other = 1
            const size_t e = nextEvent[row]++;
            const uint32_t pointN = report.points ? header.numContactData : 0u;
            const uint32_t anchorN = report.anchors ? header.numfrictionAnchorsData : 0u;
            out.type[e] = static_cast<int32_t>(header.type);
            out.sensorSide[e] = side;
            out.collider[e] = own0 ? header.collider0 : header.collider1;
            out.otherBody[e] = own0 ? header.actor1 : header.actor0;
            out.otherCollider[e] = own0 ? header.collider1 : header.collider0;
            out.protoIndex[e] = own0 ? header.protoIndex0 : header.protoIndex1;
            out.otherProtoIndex[e] = own0 ? header.protoIndex1 : header.protoIndex0;
            out.pointCount[e] = static_cast<int32_t>(pointN);
            out.anchorCount[e] = static_cast<int32_t>(anchorN);
            for (uint32_t k = 0; k < pointN; ++k)
            {
                const ContactData& cd = report.points[header.contactDataOffset + k];
                const size_t p = nextPoint[row]++;
                put3(out.pointPosition, p, cd.position);
                put3(out.pointNormal, p, cd.normal);
                put3(out.pointImpulse, p, cd.impulse);
                out.pointSeparation[p] = cd.separation;
                out.pointFaceIndex[p] = own0 ? cd.faceIndex0 : cd.faceIndex1;
                out.pointOtherFaceIndex[p] = own0 ? cd.faceIndex1 : cd.faceIndex0;
                out.pointMaterial[p] = own0 ? cd.material0 : cd.material1;
                out.pointOtherMaterial[p] = own0 ? cd.material1 : cd.material0;
            }
            for (uint32_t k = 0; k < anchorN; ++k)
            {
                const FrictionAnchor& anchor = report.anchors[header.frictionAnchorsDataOffset + k];
                const size_t a = nextAnchor[row]++;
                put3(out.anchorPosition, a, anchor.position);
                put3(out.anchorImpulse, a, anchor.impulse);
            }
        }
    }
    return out;
}

} // namespace contactread
} // namespace physx
} // namespace omni
