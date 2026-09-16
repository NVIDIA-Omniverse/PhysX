// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SPLINE-CURVE-001
 * @covers AC-1 AC-5 AC-6 AC-7
 */
#pragma once

#include "carb/Defines.h"
#include "carb/Types.h"
#include <string>
#include <vector>
#include <PxPhysicsAPI.h>
#include <utils/OmniRenderBuffer.h>
#include <omni/physics/parse/Handles.h>
#include <common/foundation/CarbPhysXCast.h>

enum class eBasisCurveWrap
{
    Periodic = 0,
    NonPeriodic = 1,
    Pinned = 2
};

// USD's basis token has three values (bezier/catmullRom/bspline, the last being the
// default for anything else); ovstage carries the same vocabulary as TokenId. This
// enum is the source-neutral form both backends resolve down to before reaching
// SplineCurve.
enum class eCurveBasisType
{
    Bezier = 0,
    CatmullRom = 1,
    BSpline = 2
};

// USD's `type` token: `linear` curves are polylines through their control points and
// ignore `basis`; `cubic` (the default) curves are evaluated with the basis above.
enum class eCurveType
{
    Linear = 0,
    Cubic = 1
};

// Linear segment p0 + u * (p1 - p0): weights (1 - u, u, 0, 0) in the same row-major layout as the
// cubic tables, so a linear segment goes through the same evaluation path with p2/p3 unused.
static constexpr float s_linearFloats[4][4] = {
    { 1.0f, -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }
};
static constexpr float s_linearTangentFloats[4][4] = {
    { -1.0f, 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }
};

static constexpr float s_bezierFloats[4][4] = {
    { 1.0f, -3.0f, 3.0f, -1.0f }, { 0.0f, 3.0f, -6.0f, 3.0f }, { 0.0f, 0.0f, 3.0f, -3.0f }, { 0.0f, 0.0f, 0.0f, 1.0f }
};

static constexpr float s_bezierTangentFloats[4][4] = {
    { -3.0f, 6.0f, -3.0f, 0.0f }, { 3.0f, -12.0f, 9.0f, 0.0f }, { 0.0f, 6.0f, -9.0f, 0.0f }, { 0.0f, 0.0f, 3.0f, 0.0f }
};
static constexpr float s_catmullRomFloats[4][4] = {
    { 0.0f, -0.5f, 1.0f, -0.5f }, { 1.0f, 0.0f, -2.5f, 1.5f }, { 0.0f, 0.5f, 2.0f, -1.5f }, { 0.0f, 0.0f, -0.5f, 0.5f }
};
namespace omni { namespace physx { namespace usdparser { class AttachedStage; } } }

static constexpr float s_catmullRomTangentFloats[4][4] = {
    { -0.5f, 2.0f, -1.5f, 0.0f }, { 0.0f, -5.0f, 4.5f, 0.0f }, { 0.5f, 4.0f, -4.5f, 0.0f }, { 0.0f, -1.0f, 1.5f, 0.0f }
};
static constexpr float s_bsplineFloats[4][4] = { { (1.0f / 6.0f), -0.5f, 0.5f, -(1.0f / 6.0f) },
                                                 { (2.0f / 3.0f), 0.0f, -1.0f, 0.5f },
                                                 { (1.0f / 6.0f), 0.5f, 0.5f, -0.5f },
                                                 { 0.0f, 0.0f, 0.0f, (1.0f / 6.0f) } };
static constexpr float s_bsplineTangentFloats[4][4] = {
    { -0.5f, 1.0f, -0.5f, 0.0f }, { 0.0f, -2.0f, 1.5f, 0.0f }, { 0.5f, 1.0f, -1.5f, 0.0f }, { 0.0f, 0.0f, 0.5f, 0.0f }
};

/// Builds a PxMat44 whose element (row, column) is values[row][column], i.e. it reads the literal
/// tables above with the row-major convention they are written in. PxMat44 stores columns, so
/// column i is gathered from values[0..3][i]. The resulting matrix is used with
/// PxMat44::transform() (column-vector convention), which yields sum_i values[row][i] * v[i].
inline ::physx::PxMat44 basisTableToPxMat44(const float values[4][4])
{
    return ::physx::PxMat44(
        ::physx::PxVec4(values[0][0], values[1][0], values[2][0], values[3][0]),
        ::physx::PxVec4(values[0][1], values[1][1], values[2][1], values[3][1]),
        ::physx::PxVec4(values[0][2], values[1][2], values[2][2], values[3][2]),
        ::physx::PxVec4(values[0][3], values[1][3], values[2][3], values[3][3]));
}


class SplineCurve
{
public:
    // NOTE: Bezier curves don't need any special handling for pinned ends,
    //       since each segment is completely independent, hence the skipping by 3, instead of 1.
    // Linear curves have no pinned form either: every control point is on the curve already.
    SplineCurve(const eBasisCurveWrap& wrapMode, eCurveBasisType type, eCurveType curveType = eCurveType::Cubic)
        : mWrapMode(((type == eCurveBasisType::Bezier || curveType == eCurveType::Linear) &&
                     wrapMode == eBasisCurveWrap::Pinned) ?
                        eBasisCurveWrap::NonPeriodic :
                        wrapMode),
          mVstep((type == eCurveBasisType::Bezier && curveType == eCurveType::Cubic) ? 3 : 1),
          mLinear(curveType == eCurveType::Linear)
    {
        // NOTE: the bases used to be transposed here and then applied with the row-vector product
        //       `v * mBasis`. PxMat44::transform() is the column-vector product, so the transpose
        //       and the operand flip cancel out and both are dropped; the numeric result is the same.
        mBasis = mLinear ? basisTableToPxMat44(s_linearFloats) : getBasis(type);
        mTangentBasis = mLinear ? basisTableToPxMat44(s_linearTangentFloats) : getTangentBasis(type);
    }

    ::physx::PxMat44 getBasis(eCurveBasisType type)
    {
        if (type == eCurveBasisType::Bezier)
        {
            return basisTableToPxMat44(s_bezierFloats);
        }
        else if (type == eCurveBasisType::CatmullRom)
        {
            return basisTableToPxMat44(s_catmullRomFloats);
        }
        else
        {
            return basisTableToPxMat44(s_bsplineFloats);
        }
    }

    ::physx::PxMat44 getTangentBasis(eCurveBasisType type)
    {
        if (type == eCurveBasisType::Bezier)
        {
            return basisTableToPxMat44(s_bezierTangentFloats);
        }
        else if (type == eCurveBasisType::CatmullRom)
        {
            return basisTableToPxMat44(s_catmullRomTangentFloats);
        }
        else
        {
            return basisTableToPxMat44(s_bsplineTangentFloats);
        }
    }


    // A periodic curve closes back to its first point, except that two control points are
    // always one open segment: there is no loop to close, only the same edge twice.
    bool closesLoop(const size_t controlPointCount) const
    {
        return mWrapMode == eBasisCurveWrap::Periodic && controlPointCount > 2;
    }

    size_t getSegmentCount(const size_t controlPointCount) const
    {
        if (controlPointCount < 2)
        {
            return 0;
        }
        if (controlPointCount == 2)
        {
            return 1;
        }
        if (mLinear)
        {
            // One segment per consecutive point pair; periodic adds the closing segment.
            return closesLoop(controlPointCount) ? controlPointCount : controlPointCount - 1;
        }

        size_t segmentCount = 0;
        switch (mWrapMode)
        {
        case eBasisCurveWrap::Periodic:
            if (mVstep == 1)
            {
                segmentCount = controlPointCount;
            }
            else
            {
                // The CV count can be mVstep * segmentCount, or (for USD) mVstep * segmentCount + 1 to indicate
                // periodic; either way, segmentCount = controlPointCount / mVstep works.
                segmentCount = controlPointCount / mVstep;
            }
            break;
        case eBasisCurveWrap::NonPeriodic:
            if (controlPointCount < 4)
            {
                return 0;
            }
            segmentCount = (controlPointCount - 4) / mVstep + 1;
            break;
        case eBasisCurveWrap::Pinned:
            if (controlPointCount < 4)
            {
                return 0;
            }
            // 2 more than non-periodic for pinned segments
            segmentCount = (controlPointCount - 4) / mVstep + 3;
            break;
        }
        return segmentCount;
    }

    size_t getTessellationPointCount(const size_t controlPointCount,
                                                   const size_t stepsPerSegment,
                                                   bool includeSegmentFinalSamples = false) const
    {
        if (mLinear && controlPointCount >= 2)
        {
            const size_t segmentCount = getSegmentCount(controlPointCount);
            size_t pointCount = segmentCount * (stepsPerSegment + size_t(includeSegmentFinalSamples));
            if (!includeSegmentFinalSamples && !closesLoop(controlPointCount))
                ++pointCount;
            return pointCount;
        }
        if (controlPointCount < 3)
        {
            if (controlPointCount <= 1)
                return controlPointCount;

            // TODO: Should this subsample?
            return 2;
        }
        if (mWrapMode == eBasisCurveWrap::Periodic)
        {
            // Some systems store a redundant duplicate of the first point at the end
            // of a cubic Bezier, so allow a modulus of 1 in that case.
            CARB_ASSERT(controlPointCount % mVstep == 0 || (mVstep == 3 && controlPointCount % mVstep == 1));
            CARB_ASSERT(mVstep >= 1 && mVstep <= 3);
            return (controlPointCount / mVstep) * (stepsPerSegment + size_t(includeSegmentFinalSamples));
        }
        // FIXME: Handle controlPointCount being 3!
        CARB_ASSERT(controlPointCount != 3);
        if (controlPointCount == 3)
            return 0;
        const size_t centralSegmentCount = (controlPointCount - 4) / mVstep + 1;
        // NOTE: Bezier curves use NonPeriodic in place of Pinned
        size_t segmentCount = centralSegmentCount + ((mWrapMode == eBasisCurveWrap::Pinned) ? 2 : 0);
        size_t pointCount = segmentCount * (stepsPerSegment + size_t(includeSegmentFinalSamples));
        if (!includeSegmentFinalSamples)
            ++pointCount;
        return pointCount;
    }

    /// tessellate curve with a fixed number of samples per segment
    ///
    /// \param controlPoints:
    ///         control points of the curve
    /// \param controlPointCount
    ///         number of control points
    /// \param tessellatedPoints
    ///         collection to which tessellated points will be appended
    /// \param stepsPerSegment
    ///         number of tessellated points that will be produced per segment
    /// \param includeSegmentFinalSamples
    ///         If true, endpoints of segments will be sampled for tessellation,
    ///         resulting in stepsPerSegment+1 points per segment.
    ///         If false, only the final segment's endpoint is sampled
    ///         (i.e. the end of the curve).
    /// \param tessellatedTangents
    ///         If a pointer is provided, tangents corresponding to the
    ///         tessellated points will be appended to this collection    
    void tessellate(const ::physx::PxVec3* controlPoints,
                    const size_t controlPointCount,
                    std::vector<::physx::PxVec3>& tessellatedPoints,
                    const size_t stepsPerSegment,
                    bool includeSegmentFinalSamples = false,
                    std::vector<::physx::PxVec3>* tessellatedTangents = nullptr) const
    {
        size_t tessellatedPointCount =
            getTessellationPointCount(controlPointCount, stepsPerSegment, includeSegmentFinalSamples);
        tessellatedPoints.resize(tessellatedPointCount);
        if (tessellatedTangents)
            tessellatedTangents->resize(tessellatedPointCount);
        tessellate(controlPoints, controlPointCount, tessellatedPoints.data(), stepsPerSegment,
                   includeSegmentFinalSamples, tessellatedTangents ? tessellatedTangents->data() : nullptr);
    }
    
    void tessellate(const ::physx::PxVec3* controlPoints,
                    const size_t controlPointCount,
                    ::physx::PxVec3* tessellatedPoints,
                    const size_t stepsPerSegment,
                    bool includeSegmentFinalSamples = false,
                    ::physx::PxVec3* tessellatedTangents = nullptr) const
    {
        if (mLinear && controlPointCount >= 2)
        {
            // Polyline: every control point is a vertex, segment i runs cp[i] -> cp[i + 1]
            // (wrapping to cp[0] for the closing segment of a periodic curve).
            const size_t outputPointsPerSegment = stepsPerSegment + size_t(includeSegmentFinalSamples);
            const size_t segmentCount = getSegmentCount(controlPointCount);
            const bool periodic = closesLoop(controlPointCount);
            for (size_t i = 0; i < segmentCount; ++i)
            {
                const bool includeFinalSample = includeSegmentFinalSamples || (!periodic && i == segmentCount - 1);
                tessellateSegment(controlPoints, controlPointCount, tessellatedPoints, tessellatedTangents, i,
                                  stepsPerSegment, includeFinalSample);
                tessellatedPoints += outputPointsPerSegment;
                if (tessellatedTangents)
                    tessellatedTangents += outputPointsPerSegment;
            }
            return;
        }

        // Small edge cases
        if (controlPointCount < 3)
        {
            if (controlPointCount == 0)
                return;
            ::physx::PxVec3 p0 = controlPoints[0];
            tessellatedPoints[0] = p0;
            if (controlPointCount == 2)
            {
                // Straight line
                // TODO: Should this subsample?
                ::physx::PxVec3 p1 = controlPoints[1];
                ::physx::PxVec3 t = p1 - p0;
                tessellatedPoints[1] = p1;
                if (tessellatedTangents != nullptr)
                {
                    tessellatedTangents[0] = t;
                    tessellatedTangents[1] = t;
                }
            }
            else
            {
                if (tessellatedTangents != nullptr)
                {
                    // Single point, so arbitrarily pick zero tangent
                    tessellatedTangents[0] = ::physx::PxVec3(0.0f);
                }
            }
            return;
        }

        const size_t outputPointsPerSegment = stepsPerSegment + size_t(includeSegmentFinalSamples);

        if (mWrapMode == eBasisCurveWrap::Periodic)
        {
            // Periodic, so all segments are the same basis, apart from wrapping indices.
            // Some systems store a redundant duplicate of the first point at the end
            // of a cubic Bezier, so allow a modulus of 1 in that case.
            CARB_ASSERT(controlPointCount % mVstep == 0 || (mVstep == 3 && controlPointCount % mVstep == 1));
            // This should work for step size 2 (e.g. pos,tan,pos,tan) or 3 (pos,tan,tan,pos)
            // though for 3, the only thing that wraps is the last position, not its tangent.
            CARB_ASSERT(mVstep >= 1 && mVstep <= 3);
            if (mVstep == 1)
            {
                // Start first segment with point 0 at index 1, instead of index 0.
                tessellateSegment(controlPoints, controlPointCount, tessellatedPoints, tessellatedTangents,
                                  controlPointCount - 1, stepsPerSegment, includeSegmentFinalSamples);
                tessellatedPoints += outputPointsPerSegment;
                if (tessellatedTangents)
                    tessellatedTangents += outputPointsPerSegment;
            }
            for (size_t controlPointIndex = 0; controlPointIndex < controlPointCount - 1; controlPointIndex += mVstep)
            {
                tessellateSegment(controlPoints, controlPointCount, tessellatedPoints, tessellatedTangents,
                                  controlPointIndex, stepsPerSegment, includeSegmentFinalSamples);
                tessellatedPoints += outputPointsPerSegment;
                if (tessellatedTangents)
                    tessellatedTangents += outputPointsPerSegment;
            }
        }
        else
        {
            // FIXME: Handle controlPointCount being 3!
            CARB_ASSERT(controlPointCount != 3);
            if (controlPointCount == 3)
                return;

            if (mWrapMode == eBasisCurveWrap::Pinned)
            {
                tessellateEndSegment(controlPoints, controlPointCount, tessellatedPoints, tessellatedTangents, 0,
                                     stepsPerSegment, includeSegmentFinalSamples);
                tessellatedPoints += outputPointsPerSegment;
                if (tessellatedTangents)
                    tessellatedTangents += outputPointsPerSegment;
            }

            const size_t centralSegmentCount = (controlPointCount - 4) / mVstep + 1;
            const bool includeCentralFinalSample = (mWrapMode == eBasisCurveWrap::NonPeriodic);
            size_t controlPointIndex = 0;
            for (size_t i = 0; i < centralSegmentCount; ++i)
            {
                const bool includeFinalSample =
                    includeSegmentFinalSamples || ((i == centralSegmentCount - 1) && includeCentralFinalSample);
                tessellateSegment(controlPoints, controlPointCount, tessellatedPoints, tessellatedTangents,
                                  controlPointIndex, stepsPerSegment, includeFinalSample);
                controlPointIndex += mVstep;
                tessellatedPoints += outputPointsPerSegment;
                if (tessellatedTangents)
                    tessellatedTangents += outputPointsPerSegment;
            }

            if (mWrapMode == eBasisCurveWrap::Pinned)
            {
                tessellateEndSegment(controlPoints, controlPointCount, tessellatedPoints, tessellatedTangents,
                                     controlPointCount, stepsPerSegment, true);
            }
        }
    }

    void tessellateSegment(const ::physx::PxVec3* controlPoints,
                           const size_t controlPointCount,
                           ::physx::PxVec3* tessellatedPoints,
                           ::physx::PxVec3* tessellatedTangents,
                           size_t index,
                           const size_t stepsPerSegment,
                           bool includeFinalSample,
                           const float startT = 0.f,
                           const float endT = 1.f) const
    {
        // Wrap around as needed, for periodic case.
        const size_t index1 = (index + 1 < controlPointCount) ? (index + 1) : 0;
        const size_t index2 = (index1 + 1 < controlPointCount) ? (index1 + 1) : 0;
        const size_t index3 = (index2 + 1 < controlPointCount) ? (index2 + 1) : 0;

        tessellateSegment(controlPoints[index], controlPoints[index1], controlPoints[index2], controlPoints[index3],
                          tessellatedPoints, tessellatedTangents, stepsPerSegment, includeFinalSample, startT, endT);
    }

    ::physx::PxVec3 evaluateSegment(const ::physx::PxVec3* controlPoints,
                            const size_t controlPointCount,
                            const size_t index,
                            float u,
                            int order = 0,
                            ::physx::PxVec3* tangent = nullptr) const
    {
        // NOTE that index is the starting control point index, *not* the segment index.

        // Wrap around as needed, for periodic case.
        const size_t index1 = (index + 1 < controlPointCount) ? (index + 1) : 0;
        const size_t index2 = (index1 + 1 < controlPointCount) ? (index1 + 1) : 0;
        const size_t index3 = (index2 + 1 < controlPointCount) ? (index2 + 1) : 0;

        return evaluateSegment(controlPoints[index], controlPoints[index1], controlPoints[index2],
                               controlPoints[index3], u, order, tangent);
    }

    void tessellateSegment(const ::physx::PxVec3& p0,
                           const ::physx::PxVec3& p1,
                           const ::physx::PxVec3& p2,
                           const ::physx::PxVec3& p3,
                           ::physx::PxVec3* tessellatedPoints,
                           ::physx::PxVec3* tessellatedTangents,
                           const size_t stepsPerSegment,
                           bool includeFinalSample,
                           const float startT = 0.f,
                           const float endT = 1.f) const
    {
        const size_t sampleCount = stepsPerSegment + size_t(includeFinalSample);
        for (size_t sample = 0; sample < sampleCount; ++sample)
        {
            // From 0 to 1 along the segment
            const float u = startT + float(sample) / float(stepsPerSegment) * (endT - startT);
            const float u2 = u * u;
            const ::physx::PxVec4 v(1.0f, u, u2, u * u2);
            const ::physx::PxVec4 coeffs = mBasis.transform(v);
            tessellatedPoints[sample] = (p0 * coeffs[0] + p1 * coeffs[1] + p2 * coeffs[2] + p3 * coeffs[3]);
            if (tessellatedTangents != nullptr)
            {
                const ::physx::PxVec4 tangentCoeffs = mTangentBasis.transform(v);
                tessellatedTangents[sample] =
                    (p0 * tangentCoeffs[0] + p1 * tangentCoeffs[1] + p2 * tangentCoeffs[2] + p3 * tangentCoeffs[3]);
            }
        }
    }

    ::physx::PxVec3 evaluateSegment(const ::physx::PxVec3& p0,
                                 const ::physx::PxVec3& p1,
                                 const ::physx::PxVec3& p2,
                                 const ::physx::PxVec3& p3,
                                 float u,
                                 int order = 0,
                                 ::physx::PxVec3* tangent = nullptr) const
    {
        const float u2 = u * u;
        const float u3 = u * u2;
        const ::physx::PxVec4 v = (order == 0 ? ::physx::PxVec4(1.0f, u, u2, u3) :
                                order == 1 ? ::physx::PxVec4(0.0f, 1.0f, 2 * u, 3 * u2) :
                                order == 2 ? ::physx::PxVec4(0.0f, 0.0f, 2.0f, 6 * u) :
                                order == 3 ? ::physx::PxVec4(0.0f, 0.0f, 0.0f, 6.0f) :
                                             ::physx::PxVec4(0.0f));
        const ::physx::PxVec4 coeffs = mBasis.transform(v);

        if (tangent != nullptr)
        {
            const ::physx::PxVec4 tangentCoeffs = mTangentBasis.transform(v);
            *tangent = (p0 * tangentCoeffs[0] + p1 * tangentCoeffs[1] + p2 * tangentCoeffs[2] + p3 * tangentCoeffs[3]);
        }

        return (p0 * coeffs[0] + p1 * coeffs[1] + p2 * coeffs[2] + p3 * coeffs[3]);
    }

    void tessellateEndSegment(const ::physx::PxVec3* controlPoints,
                              const size_t /*controlPointCount*/,
                              ::physx::PxVec3* tessellatedPoints,
                              ::physx::PxVec3* tessellatedTangents,
                              size_t index,
                              const size_t stepsPerSegment,
                              bool includeFinalSample) const
    {
        ::physx::PxVec3 p0;
        ::physx::PxVec3 p1;
        ::physx::PxVec3 p2;
        ::physx::PxVec3 p3;
        if (index == 0)
        {
            p1 = controlPoints[index];
            p2 = controlPoints[index + 1];
            p3 = controlPoints[index + 2];
            p0 = p1 * 2.0f - p2;
        }
        else
        {
            p0 = controlPoints[index - 3];
            p1 = controlPoints[index - 2];
            p2 = controlPoints[index - 1];
            p3 = p2 * 2.0f - p1;
        }

        tessellateSegment(p0, p1, p2, p3, tessellatedPoints, tessellatedTangents, stepsPerSegment, includeFinalSample);
    }

public:
    eBasisCurveWrap mWrapMode;
    int mVstep;
    bool mLinear;

    ::physx::PxMat44 mBasis;
    ::physx::PxMat44 mTangentBasis;
};

class SplinesCurve
{
public:
    SplinesCurve(const omni::physx::usdparser::AttachedStage& attachedStage,
                 omni::physics::parse::ObjectKey curveKey);
    ~SplinesCurve();

    bool getClosestPoint(const ::physx::PxVec3& point,
                         ::physx::PxVec3& pointOnCurveOut,
                         ::physx::PxVec3& tangentOut,
                         ::physx::PxVec3& curvaturePointOut);

    bool isInitialized() const
    {
        return mInitialized;
    }

    void draw(omni::physx::OmniRenderBuffer& renderBuffer, const ::physx::PxTransform& tr) const;

private:
    void initialize(const std::string& curvePrimPath,
                    const std::vector<carb::Float3>& pointsIn,
                    const std::vector<int32_t>& curveVertexCounts,
                    eCurveBasisType basisType,
                    eBasisCurveWrap wrapType,
                    eCurveType curveType);

    bool mInitialized;
    // Prim-level `type == linear`: closest-point queries report the hit edge's own direction
    // instead of blending the tangent samples across a corner.
    bool mLinear = false;
    std::string mCurvePrimPath;

    std::vector<::physx::PxVec3> mPoints;
    std::vector<::physx::PxVec3> mTangents;
    std::vector<int> mCurveVertexCounts;
    std::vector<::physx::PxVec3> mCurvaturePoints;
#ifdef __AVX__
    std::vector<float> mXPoints;
    std::vector<float> mYPoints;
    std::vector<float> mZPoints;
#endif
};
