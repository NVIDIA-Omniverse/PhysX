// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "carb/Defines.h"

enum class eBasisCurveWrap
{
    Periodic = 0,
    NonPeriodic = 1,
    Pinned = 2
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


class SplineCurve
{
public:
    // NOTE: Bezier curves don't need any special handling for pinned ends,
    //       since each segment is completely independent, hence the skipping by 3, instead of 1.
    SplineCurve(const eBasisCurveWrap& wrapMode, const pxr::TfToken& type)
        : mWrapMode((type == pxr::UsdGeomTokens->bezier && wrapMode == eBasisCurveWrap::Pinned) ?
                        eBasisCurveWrap::NonPeriodic :
                        wrapMode),
          mVstep((type == pxr::UsdGeomTokens->bezier) ? 3 : 1)
    {
        mBasis = getBasis(type);
        mBasis = mBasis.GetTranspose();
        mTangentBasis = getTangentBasis(type);
        mTangentBasis = mTangentBasis.GetTranspose();
    }

    pxr::GfMatrix4f getBasis(const pxr::TfToken& type)
    {
        if (type == pxr::UsdGeomTokens->bezier)
        {
            return pxr::GfMatrix4f(s_bezierFloats);
        }
        else if (type == pxr::UsdGeomTokens->catmullRom)
        {
            return pxr::GfMatrix4f(s_catmullRomFloats);
        }
        else
        {
            return pxr::GfMatrix4f(s_bsplineFloats);
        }
    }

    pxr::GfMatrix4f getTangentBasis(const pxr::TfToken& type)
    {
        if (type == pxr::UsdGeomTokens->bezier)
        {
            return pxr::GfMatrix4f(s_bezierTangentFloats);
        }
        else if (type == pxr::UsdGeomTokens->catmullRom)
        {
            return pxr::GfMatrix4f(s_catmullRomTangentFloats);
        }
        else
        {
            return pxr::GfMatrix4f(s_bsplineTangentFloats);
        }
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
                // sometimes the CV count can be mVstep * segmentCount or  mVstep * segment + 1 to indicate peridoic
                // for USD it is mVstep * segmentCount. In either case, segmentCount = controlPointCount / mVstep
                // works
                segmentCount = controlPointCount / mVstep;
            }
            break;
        case eBasisCurveWrap::NonPeriodic:
            segmentCount = (controlPointCount - 4) / mVstep + 1;
            break;
        case eBasisCurveWrap::Pinned:
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
        // Add the final point if it wasn't already included
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
    void tessellate(const pxr::GfVec3f* controlPoints,
                    const size_t controlPointCount,
                    std::vector<pxr::GfVec3f>& tessellatedPoints,
                    const size_t stepsPerSegment,
                    bool includeSegmentFinalSamples = false,
                    std::vector<pxr::GfVec3f>* tessellatedTangents = nullptr) const
    {
        size_t tessellatedPointCount =
            getTessellationPointCount(controlPointCount, stepsPerSegment, includeSegmentFinalSamples);
        tessellatedPoints.resize(tessellatedPointCount);
        if (tessellatedTangents)
            tessellatedTangents->resize(tessellatedPointCount);
        tessellate(controlPoints, controlPointCount, tessellatedPoints.data(), stepsPerSegment,
                   includeSegmentFinalSamples, tessellatedTangents ? tessellatedTangents->data() : nullptr);
    }
    
    void tessellate(const pxr::GfVec3f* controlPoints,
                    const size_t controlPointCount,
                    pxr::GfVec3f* tessellatedPoints,
                    const size_t stepsPerSegment,
                    bool includeSegmentFinalSamples = false,
                    pxr::GfVec3f* tessellatedTangents = nullptr) const
    {
        // Small edge cases
        if (controlPointCount < 3)
        {
            if (controlPointCount == 0)
                return;
            pxr::GfVec3f p0 = controlPoints[0];
            tessellatedPoints[0] = p0;
            if (controlPointCount == 2)
            {
                // Straight line
                // TODO: Should this subsample?
                pxr::GfVec3f p1 = controlPoints[1];
                pxr::GfVec3f t = p1 - p0;
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
                    tessellatedTangents[0] = pxr::GfVec3f(0.0f);
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

    void tessellateSegment(const pxr::GfVec3f* controlPoints,
                           const size_t controlPointCount,
                           pxr::GfVec3f* tessellatedPoints,
                           pxr::GfVec3f* tessellatedTangents,
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

    pxr::GfVec3f evaluateSegment(const pxr::GfVec3f* controlPoints,
                            const size_t controlPointCount,
                            const size_t index,
                            float u,
                            int order = 0,
                            pxr::GfVec3f* tangent = nullptr) const
    {
        // NOTE that index is the starting control point index, *not* the segment index.

        // Wrap around as needed, for periodic case.
        const size_t index1 = (index + 1 < controlPointCount) ? (index + 1) : 0;
        const size_t index2 = (index1 + 1 < controlPointCount) ? (index1 + 1) : 0;
        const size_t index3 = (index2 + 1 < controlPointCount) ? (index2 + 1) : 0;

        return evaluateSegment(controlPoints[index], controlPoints[index1], controlPoints[index2],
                               controlPoints[index3], u, order, tangent);
    }

    void tessellateSegment(const pxr::GfVec3f& p0,
                           const pxr::GfVec3f& p1,
                           const pxr::GfVec3f& p2,
                           const pxr::GfVec3f& p3,
                           pxr::GfVec3f* tessellatedPoints,
                           pxr::GfVec3f* tessellatedTangents,
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
            const pxr::GfVec4f v(1.0f, u, u2, u * u2);
            //const pxr::GfVec4f coeffs = mBasis * v;
            const pxr::GfVec4f coeffs = v * mBasis;
            tessellatedPoints[sample] = (p0 * coeffs[0] + p1 * coeffs[1] + p2 * coeffs[2] + p3 * coeffs[3]);
            if (tessellatedTangents != nullptr)
            {
                //const pxr::GfVec4f tangentCoeffs = mTangentBasis * v;
                const pxr::GfVec4f tangentCoeffs = v * mTangentBasis;
                tessellatedTangents[sample] =
                    (p0 * tangentCoeffs[0] + p1 * tangentCoeffs[1] + p2 * tangentCoeffs[2] + p3 * tangentCoeffs[3]);
            }
        }
    }

    pxr::GfVec3f evaluateSegment(const pxr::GfVec3f& p0,
                                 const pxr::GfVec3f& p1,
                                 const pxr::GfVec3f& p2,
                                 const pxr::GfVec3f& p3,
                                 float u,
                                 int order = 0,
                                 pxr::GfVec3f* tangent = nullptr) const
    {
        const float u2 = u * u;
        const float u3 = u * u2;
        const pxr::GfVec4f v = (order == 0 ? pxr::GfVec4f(1, u, u2, u3) :
                                order == 1 ? pxr::GfVec4f(0, 1, 2 * u, 3 * u2) :
                                order == 2 ? pxr::GfVec4f(0, 0, 2, 6 * u) :
                                order == 3 ? pxr::GfVec4f(0, 0, 0, 6) :
                                             pxr::GfVec4f(0, 0, 0, 0));
        //const pxr::GfVec4f coeffs = mBasis * v;
        const pxr::GfVec4f coeffs = v * mBasis;

        if (tangent != nullptr)
        {
            //const pxr::GfVec4f tangentCoeffs = mTangentBasis * v;
            const pxr::GfVec4f tangentCoeffs = v * mTangentBasis;
            *tangent = (p0 * tangentCoeffs[0] + p1 * tangentCoeffs[1] + p2 * tangentCoeffs[2] + p3 * tangentCoeffs[3]);
        }

        return (p0 * coeffs[0] + p1 * coeffs[1] + p2 * coeffs[2] + p3 * coeffs[3]);
    }

    void tessellateEndSegment(const pxr::GfVec3f* controlPoints,
                              const size_t controlPointCount,
                              pxr::GfVec3f* tessellatedPoints,
                              pxr::GfVec3f* tessellatedTangents,
                              size_t index,
                              const size_t stepsPerSegment,
                              bool includeFinalSample) const
    {
        pxr::GfVec3f p0;
        pxr::GfVec3f p1;
        pxr::GfVec3f p2;
        pxr::GfVec3f p3;
        if (index == 0)
        {
            p1 = controlPoints[index];
            p2 = controlPoints[index + 1];
            p3 = controlPoints[index + 2];
            p0 = 2 * p1 - p2;
        }
        else
        {
            p0 = controlPoints[index - 3];
            p1 = controlPoints[index - 2];
            p2 = controlPoints[index - 1];
            p3 = 2 * p2 - p1;
        }

        tessellateSegment(p0, p1, p2, p3, tessellatedPoints, tessellatedTangents, stepsPerSegment, includeFinalSample);
    }

public:
    eBasisCurveWrap mWrapMode;
    int mVstep;

    pxr::GfMatrix4f mBasis;
    pxr::GfMatrix4f mTangentBasis;
};

class SplinesCurve
{
public:
    SplinesCurve(const pxr::UsdGeomBasisCurves& curvePrim);
    ~SplinesCurve();

    bool getClosestPoint(const pxr::GfVec3f& point, pxr::GfVec3f& pointOnCurveOut, pxr::GfVec3f& tangentOut);

    bool isInitialized() const
    {
        return mInitialized;
    }

private:
    bool mInitialized;
    pxr::SdfPath mCurvePrimPath;

    std::vector<pxr::GfVec3f> mPoints;
    std::vector<pxr::GfVec3f> mTangents;
    std::vector<float> mDistances;
    std::vector<int> mCurveVertexCounts;    
#ifdef __AVX__
    std::vector<float> mXPoints;
    std::vector<float> mYPoints;
    std::vector<float> mZPoints;
#endif
};

