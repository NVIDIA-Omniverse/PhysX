// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include "SplinesCurve.h"

#include <common/foundation/TypeCast.h>

using namespace pxr;

static const size_t samplesPerSegment = 96;

std::vector<float> approximateDistanceAlongCurve(const SplineCurve& curve,
                                                   const std::vector<GfVec3f>& controlPoints,
                                                   const bool normalizeLengths,
                                                   size_t stepsPerSegment = 20,
                                                   std::vector<GfVec3f>* pointsOut = nullptr,
                                                   std::vector<GfVec3f>* tangentsOut = nullptr)
{
    const bool includeSegmentFinalSamples = false;

    // do a tessellation with some relatively high resolution to approximate curve length
    std::vector<GfVec3f> tessellatedPoints;
    curve.tessellate(controlPoints.data(), controlPoints.size(), tessellatedPoints, stepsPerSegment,
                     includeSegmentFinalSamples, tangentsOut);

    const bool isPeriodic = (curve.mWrapMode == eBasisCurveWrap::Periodic);

    std::vector<float> distanceAlongCurve(tessellatedPoints.size() + size_t(isPeriodic));

    float length = 0.0f;
    for (size_t i = 0; i < tessellatedPoints.size(); i++)
    {
        if (i != 0)
            length += (tessellatedPoints[i] - tessellatedPoints[i - 1]).GetLength();
        distanceAlongCurve[i] = length;
    }

    if (isPeriodic)
    {
        length += (tessellatedPoints[0] - tessellatedPoints.back()).GetLength();
        distanceAlongCurve.back() = length;
        if (tangentsOut != nullptr)
        {
            tangentsOut->push_back((*tangentsOut)[0]);
        }
    }

    if (normalizeLengths && length > 0)
    {
        for (size_t i = 0; i < distanceAlongCurve.size(); i++)
        {
            distanceAlongCurve[i] /= length;
        }
    }

    if (pointsOut != nullptr)
    {
        (*pointsOut) = std::move(tessellatedPoints);
    }

    return distanceAlongCurve;
}


SplinesCurve::SplinesCurve(const pxr::UsdGeomBasisCurves& curvePrim)
{
    mInitialized = false;
    mCurvePrimPath = curvePrim.GetPrim().GetPrimPath();

    // Cache up data
    VtArray<GfVec3f> pointsIn;
    curvePrim.GetPointsAttr().Get(&pointsIn);
    if (pointsIn.empty())
    {
        CARB_LOG_WARN("No points defined for the BasisCurves prim %s.", mCurvePrimPath.GetText());
        return;
    }    

    const size_t pointsInCount = pointsIn.size();

    VtArray<int> curveVertexCounts;
    curvePrim.GetCurveVertexCountsAttr().Get(&curveVertexCounts);
    const size_t curveCount = curveVertexCounts.size();
    
    TfToken basisToken;
    curvePrim.GetBasisAttr().Get(&basisToken);

    TfToken wrapToken;
    curvePrim.GetWrapAttr().Get(&wrapToken);

    eBasisCurveWrap curveWrapType = eBasisCurveWrap::NonPeriodic;
    if (wrapToken == UsdGeomTokens->nonperiodic)
    {
        curveWrapType = eBasisCurveWrap::NonPeriodic;
    }
    else if (wrapToken == UsdGeomTokens->periodic)
    {
        curveWrapType = eBasisCurveWrap::Periodic;
    }
    else if (wrapToken == UsdGeomTokens->pinned)
    {
        curveWrapType = eBasisCurveWrap::Pinned;
    }

    std::vector<GfVec3f> positions, tangents;
    std::vector<int> vertexCountsPerCurve(curveCount == 0 ? 1 : curveCount);
    std::vector<int> segmentCountsPerCurve(curveCount == 0 ? 1 : curveCount);

    for (size_t curveIndex = 0, startIndex = 0; curveIndex < (curveCount == 0 ? 1 : curveCount); curveIndex++)
    {
        const size_t count = (curveCount == 0 ? pointsInCount : curveVertexCounts[curveIndex]);

        std::vector<GfVec3f> controlPoints;
        for (size_t i = startIndex; i < startIndex + count; i++)
        {
            controlPoints.push_back(pointsIn[i]);
        }
        startIndex += count;

        SplineCurve curve(curveWrapType, basisToken);

        std::vector<GfVec3f> tessellatedPositions, tessellatedTangents;
        const bool normalizeLengths = true;
        const std::vector<float> curveDistances = approximateDistanceAlongCurve(
            curve, controlPoints, normalizeLengths, samplesPerSegment, &tessellatedPositions, &tessellatedTangents);
        const size_t segmentCount = curve.getSegmentCount(controlPoints.size());
        std::vector<float> curveSegmentDistances(segmentCount);
        for (size_t i = 0; i < segmentCount; i++)
        {
            const size_t lastOffset = (i == segmentCount - 1 ? 1 : 0);
            curveSegmentDistances[i] = curveDistances[(i + 1) * samplesPerSegment + lastOffset - 1];
        }

        size_t offset = positions.size();
        positions.resize(offset + tessellatedPositions.size());
        tangents.resize(offset + tessellatedTangents.size());
        for (size_t i = 0; i < tessellatedPositions.size(); i++)
        {
            positions[offset + i] = tessellatedPositions[i];
            tangents[offset + i] = tessellatedTangents[i];
        }

        vertexCountsPerCurve[curveIndex] = (int)tessellatedPositions.size();
        segmentCountsPerCurve[curveIndex] = (int)curveSegmentDistances.size();
    }

    mCurvaturePoints.resize(positions.size());
    for (size_t i = 0; i < positions.size(); i++)
    {
        if (i == 0 || i == positions.size() - 1)
        {
            mCurvaturePoints[i] = GfVec3f(FLT_MAX);
        }
        else
        {
            const GfVec3f& previousPoint = positions[i - 1];
            const GfVec3f& currentPoint = positions[i];
            const GfVec3f& nextPoint = positions[i + 1];
            
            // Check if the three points are collinear (forming a straight line)
            GfVec3f v1 = currentPoint - previousPoint;
            GfVec3f v2 = nextPoint - currentPoint;
            
            // Verify vectors have non-zero length before normalizing to prevent NaNs/asserts
            const float v1LengthSq = v1.GetLengthSq();
            const float v2LengthSq = v2.GetLengthSq();
            const float lengthEpsilonSq = 1e-10f;  // Squared epsilon for length check
            if (v1LengthSq < lengthEpsilonSq || v2LengthSq < lengthEpsilonSq)
            {
                // Degenerate case: coincident or nearly coincident points
                mCurvaturePoints[i] = GfVec3f(FLT_MAX);
                continue;
            }
            
            v1.Normalize();
            v2.Normalize();
            const GfVec3f crossProduct = GfCross(v1, v2);
            const float crossProductMagnitude = crossProduct.GetLength();
            
            const float collinearityTolerance = 1e-4f;
            const bool areCollinear = (crossProductMagnitude < collinearityTolerance);
            
            if (areCollinear)
            {
                // Points form a straight line - no curvature
                mCurvaturePoints[i] = GfVec3f(FLT_MAX);
            }
            else
            {
                // Compute the center of the osculating circle/sphere
                // This is the circumcenter of the three points
                const GfVec3f a = currentPoint - previousPoint;
                const GfVec3f b = nextPoint - previousPoint;
                const GfVec3f axb = GfCross(a, b);
                const float axbLengthSq = axb.GetLengthSq();
                
                // Circumcenter formula: C = P1 + [(|b|² * a - |a|² * b) × (a × b)] / (2 * |a × b|²)
                const float aLengthSq = a.GetLengthSq();
                const float bLengthSq = b.GetLengthSq();
                const GfVec3f numerator = -GfCross((bLengthSq * a - aLengthSq * b), axb);
                const GfVec3f circumcenter = previousPoint + numerator / (2.0f * axbLengthSq);
                
                mCurvaturePoints[i] = circumcenter;
            }
        }
    }

    mPoints = positions;
    mTangents = tangents;
    mCurveVertexCounts = vertexCountsPerCurve;
    
#ifdef __AVX__
    const size_t numPoints = mPoints.size();
    mXPoints.resize(numPoints);
    mYPoints.resize(numPoints);
    mZPoints.resize(numPoints);
    for (size_t i = 0; i < numPoints; i++)
    {        
        mXPoints[i] = mPoints[i][0];
        mYPoints[i] = mPoints[i][1];
        mZPoints[i] = mPoints[i][2];
    }
#endif

    mInitialized = true;
}

SplinesCurve::~SplinesCurve()
{

}

void SplinesCurve::draw(omni::physx::OmniRenderBuffer& renderBuffer, const ::physx::PxTransform& tr) const
{
    const size_t numLines = mPoints.size() - 1;
    if (mPoints.size() > 0 && numLines > 0)
    {
        for (size_t i = 0; i < numLines; i++)
        {
            renderBuffer.addLine(::physx::PxDebugLine(tr.transform(omni::physx::toPhysX(mPoints[i])),
                                                      tr.transform(omni::physx::toPhysX(mPoints[i + 1])),
                                                      ::physx::PxDebugColor::eARGB_BLUE));

            if (mCurvaturePoints[i][0] != FLT_MAX)
            {
                renderBuffer.addLine(::physx::PxDebugLine(tr.transform(omni::physx::toPhysX(mPoints[i])),
                                                          tr.transform(omni::physx::toPhysX(mCurvaturePoints[i])),
                                                          ::physx::PxDebugColor::eARGB_GREEN));
            }
        }
    }
}

bool SplinesCurve::getClosestPoint(const pxr::GfVec3f& point, pxr::GfVec3f& pointOnCurveOut, pxr::GfVec3f& tangentOut, pxr::GfVec3f& curvaturePointOut)
{
    bool foundCurve = false;
    GfVec3f pointOnCurve(0), tangentAtPoint(0);
    float closestDistanceSq = std::numeric_limits<float>::max();
    const float smallNumber = 1.e-6f;

    if (!mInitialized)
        return false;

    const GfVec3f* const points = mPoints.data();
    const GfVec3f* const tangents = mTangents.data();
    const GfVec3f* const curvaturePoints = mCurvaturePoints.data();
    const int* curveVertexCounts = mCurveVertexCounts.data();

    GfVec3f curvaturePoint(FLT_MAX);

    if (points == nullptr || tangents == nullptr || curveVertexCounts == nullptr || curvaturePoints == nullptr)
    {
        return false;
    }

    const size_t curveCount = mCurveVertexCounts.size();

    for (size_t curveIndex = 0, pointsStart = 0, segmentsStart = 0; curveIndex < curveCount;
            curveIndex++)
    {
        const GfVec3f* curvePoints = points + pointsStart;
        const GfVec3f* curveTangents = tangents + pointsStart;

        const size_t vertexCount = curveVertexCounts[curveIndex];        

#ifdef __AVX__
        const float* curveXPoints = mXPoints.data() + pointsStart;
        const float* curveYPoints = mYPoints.data() + pointsStart;
        const float* curveZPoints = mZPoints.data() + pointsStart;

        // Process 8 segments at a time using AVX
        const size_t numAVXVertices = vertexCount / 8;
        const size_t remainingVertices = vertexCount % 8;

        // Load point into AVX register (replicate for all lanes)
        __m256 pointX = _mm256_set1_ps(point[0]);
        __m256 pointY = _mm256_set1_ps(point[1]);
        __m256 pointZ = _mm256_set1_ps(point[2]);

        size_t closestIndex = 0;
        float closestT = 0.f;
        float minDistSq = std::numeric_limits<float>::max();

        const __m256 zero_v = _mm256_setzero_ps();
        const __m256 one_v = _mm256_set1_ps(1.0f);

        // Process segments in batches of 8
        for (size_t i = 0; i < numAVXVertices; i++)
        {
            const size_t baseIdx = i * 8;
            
            // Load segment endpoints (8 segments at once)
            __m256 aX = _mm256_loadu_ps(&curveXPoints[baseIdx]);
            __m256 aY = _mm256_loadu_ps(&curveYPoints[baseIdx]);
            __m256 aZ = _mm256_loadu_ps(&curveZPoints[baseIdx]);

            // Load next points (endpoints)
            __m256 bX = _mm256_loadu_ps(&curveXPoints[baseIdx + 1]);
            __m256 bY = _mm256_loadu_ps(&curveYPoints[baseIdx + 1]);
            __m256 bZ = _mm256_loadu_ps(&curveZPoints[baseIdx + 1]);

            // Compute segment vectors
            __m256 abX = _mm256_sub_ps(bX, aX);
            __m256 abY = _mm256_sub_ps(bY, aY);
            __m256 abZ = _mm256_sub_ps(bZ, aZ);

            // Compute squared length of segments using AVX multiply and add
            __m256 abLengthSq = _mm256_add_ps(
                _mm256_mul_ps(abX, abX),
                _mm256_add_ps(
                    _mm256_mul_ps(abY, abY),
                    _mm256_mul_ps(abZ, abZ)
                )
            );

            // Compute dot product of (point - a) with ab
            __m256 pointMinusAX = _mm256_sub_ps(pointX, aX);
            __m256 pointMinusAY = _mm256_sub_ps(pointY, aY);
            __m256 pointMinusAZ = _mm256_sub_ps(pointZ, aZ);

            __m256 dotProduct = _mm256_add_ps(
                _mm256_mul_ps(pointMinusAX, abX),
                _mm256_add_ps(
                    _mm256_mul_ps(pointMinusAY, abY),
                    _mm256_mul_ps(pointMinusAZ, abZ)
                )
            );

            // Compute t = dotProduct / abLengthSq
            __m256 t = _mm256_div_ps(dotProduct, abLengthSq);
            
            // Clamp t to [0,1]
            t = _mm256_max_ps(zero_v, _mm256_min_ps(t, one_v));

            // Compute point on segment using AVX multiply and add
            __m256 pointOnSegmentX = _mm256_add_ps(aX, _mm256_mul_ps(t, abX));
            __m256 pointOnSegmentY = _mm256_add_ps(aY, _mm256_mul_ps(t, abY));
            __m256 pointOnSegmentZ = _mm256_add_ps(aZ, _mm256_mul_ps(t, abZ));

            // Compute squared distance to point
            __m256 diffX = _mm256_sub_ps(pointX, pointOnSegmentX);
            __m256 diffY = _mm256_sub_ps(pointY, pointOnSegmentY);
            __m256 diffZ = _mm256_sub_ps(pointZ, pointOnSegmentZ);

            __m256 distSq = _mm256_add_ps(
                _mm256_mul_ps(diffX, diffX),
                _mm256_add_ps(
                    _mm256_mul_ps(diffY, diffY),
                    _mm256_mul_ps(diffZ, diffZ)
                )
            );

            __m256 minDistSqV = _mm256_set1_ps(minDistSq);
            __m256 res = _mm256_cmp_ps(distSq, minDistSqV, _CMP_LT_OQ); // AVX res = a < b

            __m256 vcmp = _mm256_cmp_ps(res, zero_v, _CMP_EQ_OQ);
            int mask = _mm256_movemask_ps(vcmp);
            bool any_nz = mask != 0xff;

            if (any_nz)
            {
                // Find minimum distance in this batch
                float distances[8];
                _mm256_storeu_ps(distances, distSq);
                float tValues[8];
                _mm256_storeu_ps(tValues, t);

                for (int j = 0; j < 8; j++)
                {
                    if (distances[j] < minDistSq)
                    {
                        minDistSq = distances[j];
                        closestIndex = baseIdx + j;
                        closestT = tValues[j];
                    }
                }
            }
        }

        // Process remaining segments
        for (size_t i = (numAVXVertices * 8) + 1; i < vertexCount; i++)
        {
            const GfVec3f& a = curvePoints[i - 1];
            const GfVec3f& b = curvePoints[i];

            const GfVec3f ab = b - a;
            const float abLengthSq = ab.GetLengthSq();

            float t = 0.f;
            if (abLengthSq > smallNumber)
            {
                t = GfDot(ab, (point - a)) / abLengthSq;
                t = std::max(0.f, std::min(1.f, t));
            }

            const GfVec3f pointOnSegment = a + t * ab;
            const float pointDistSq = (point - pointOnSegment).GetLengthSq();
            
            if (pointDistSq < minDistSq)
            {
                minDistSq = pointDistSq;
                closestIndex = i - 1;
                closestT = t;
            }
        }

        if (minDistSq < closestDistanceSq)
        {
            closestDistanceSq = minDistSq;
            const GfVec3f& a = curvePoints[closestIndex];
            const GfVec3f& b = curvePoints[closestIndex + 1];
            pointOnCurve = a + closestT * (b - a);

            const GfVec3f& ta = curveTangents[closestIndex];
            const GfVec3f& tb = curveTangents[closestIndex + 1];
            tangentAtPoint = ta + closestT * (tb - ta);
            curvaturePoint = (closestT < 0.5f) ? curvaturePoints[closestIndex] : curvaturePoints[closestIndex+1];
        }
#else
        // Original non-AVX implementation
        for (size_t i = 1; i < vertexCount; i++)
        {
            const GfVec3f& a = curvePoints[i - 1];
            const GfVec3f& b = curvePoints[i];

            const GfVec3f ab = b - a;
            const float abLengthSq = ab.GetLengthSq();

            float t = 0.f;
            if (abLengthSq > smallNumber)
            {
                t = GfDot(ab, (point - a)) / abLengthSq;
                t = std::max(0.f, std::min(1.f, t));
            }

            const GfVec3f pointOnSegment = a + t * ab;
            const float pointDistSq = (point - pointOnSegment).GetLengthSq();
            if (pointDistSq < closestDistanceSq)
            {
                closestDistanceSq = pointDistSq;
                pointOnCurve = pointOnSegment;

                const GfVec3f& ta = curveTangents[i - 1];
                const GfVec3f& tb = curveTangents[i];
                tangentAtPoint = ta + t * (tb - ta);

                curvaturePoint = (t < 0.5f) ? curvaturePoints[i - 1] : curvaturePoints[i];
            }
        }
#endif
        pointsStart += vertexCount;
    }

    curvaturePointOut = curvaturePoint;
    pointOnCurveOut = pointOnCurve;
    tangentAtPoint.Normalize();
    tangentOut = tangentAtPoint;
    return true;
}
