// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SPLINE-CURVE-001
 * @covers AC-2 AC-3 AC-4 AC-6 AC-7 AC-8
 */

#include <carb/logging/Log.h>
#include "SplinesCurve.h"

#include <PhysXTools.h>
#include <usdLoad/AttachedStage.h>
#include <omni/physics/parse/KnownTokens.h>

#include <common/foundation/CarbPhysXCast.h>

// getClosestPoint's AVX path below needs __m256/_mm256_* directly; this used to arrive
// transitively via a pxr header pulled in through PhysXTools.h/AttachedStage.h; both are
// pxr-free now, so this is a real, direct dependency that was previously masked. immintrin.h
// is x86-only (absent on aarch64 toolchains); guard it the same way the AVX code paths below
// already are, since an unguarded #include fails at header-resolution time regardless of
// whether __AVX__ ends up defined.
#ifdef __AVX__
#include <immintrin.h>
#endif

using ::physx::PxVec3;

static const size_t samplesPerSegment = 96;

std::vector<float> approximateDistanceAlongCurve(const SplineCurve& curve,
                                                   const std::vector<PxVec3>& controlPoints,
                                                   const bool normalizeLengths,
                                                   size_t stepsPerSegment = 20,
                                                   std::vector<PxVec3>* pointsOut = nullptr,
                                                   std::vector<PxVec3>* tangentsOut = nullptr)
{
    const bool includeSegmentFinalSamples = false;

    // do a tessellation with some relatively high resolution to approximate curve length
    std::vector<PxVec3> tessellatedPoints;
    curve.tessellate(controlPoints.data(), controlPoints.size(), tessellatedPoints, stepsPerSegment,
                     includeSegmentFinalSamples, tangentsOut);

    const bool isPeriodic = (curve.mWrapMode == eBasisCurveWrap::Periodic);

    std::vector<float> distanceAlongCurve(tessellatedPoints.size() + size_t(isPeriodic));

    float length = 0.0f;
    for (size_t i = 0; i < tessellatedPoints.size(); i++)
    {
        if (i != 0)
            length += (tessellatedPoints[i] - tessellatedPoints[i - 1]).magnitude();
        distanceAlongCurve[i] = length;
    }

    if (isPeriodic)
    {
        length += (tessellatedPoints[0] - tessellatedPoints.back()).magnitude();
        distanceAlongCurve.back() = length;
        // Store the closing edge explicitly by repeating the first sample: closest-point search
        // then sees the last -> first edge like any other and never has to wrap an index.
        tessellatedPoints.push_back(tessellatedPoints[0]);
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


SplinesCurve::SplinesCurve(const omni::physx::usdparser::AttachedStage& attachedStage,
                           omni::physics::parse::ObjectKey curveKey)
{
    mInitialized = false;
    const std::string curvePrimPath = attachedStage.textFor(curveKey);

    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (source)
        tok.intern(*source);

    std::vector<carb::Float3> pointsIn;
    omni::physx::internal::getArrayValue(
        attachedStage, curveKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), pointsIn);

    std::vector<int32_t> curveVertexCounts;
    omni::physx::internal::getArrayValue(
        attachedStage, curveKey, tok.curveVertexCounts, omni::physics::parse::ReadTime::defaultTime(), curveVertexCounts);

    // basis/wrap are token-valued attributes; resolve them straight to the source-neutral
    // enums SplineCurve/SplinesCurve::initialize take, via the TokenId comparison against
    // KnownTokens -- no TfToken materialization needed.
    eCurveBasisType basisType = eCurveBasisType::BSpline;
    omni::physics::parse::TokenId basisTokenId;
    if (source && omni::physx::internal::getValue(attachedStage, curveKey, tok.basis,
                                                   omni::physics::parse::ReadTime::defaultTime(), basisTokenId))
    {
        if (basisTokenId == tok.bezier)
            basisType = eCurveBasisType::Bezier;
        else if (basisTokenId == tok.catmullRom)
            basisType = eCurveBasisType::CatmullRom;
    }

    eBasisCurveWrap wrapType = eBasisCurveWrap::NonPeriodic;
    omni::physics::parse::TokenId wrapTokenId;
    if (source && omni::physx::internal::getValue(attachedStage, curveKey, tok.wrap,
                                                   omni::physics::parse::ReadTime::defaultTime(), wrapTokenId))
    {
        if (wrapTokenId == tok.nonperiodic)
            wrapType = eBasisCurveWrap::NonPeriodic;
        else if (wrapTokenId == tok.periodic)
            wrapType = eBasisCurveWrap::Periodic;
        else if (wrapTokenId == tok.pinned)
            wrapType = eBasisCurveWrap::Pinned;
    }

    // `type` (linear | cubic) decides whether the control points are the polyline itself or the
    // CVs of a cubic basis; a linear curve authored as a conveyor path must not be smoothed.
    eCurveType curveType = eCurveType::Cubic;
    omni::physics::parse::TokenId typeTokenId;
    if (source && omni::physx::internal::getValue(attachedStage, curveKey, tok.type,
                                                   omni::physics::parse::ReadTime::defaultTime(), typeTokenId))
    {
        if (typeTokenId == tok.linear)
            curveType = eCurveType::Linear;
    }

    initialize(curvePrimPath, pointsIn, curveVertexCounts, basisType, wrapType, curveType);
}

void SplinesCurve::initialize(const std::string& curvePrimPath,
                              const std::vector<carb::Float3>& pointsIn,
                              const std::vector<int32_t>& curveVertexCounts,
                              eCurveBasisType basisType,
                              eBasisCurveWrap wrapType,
                              eCurveType curveType)
{
    mCurvePrimPath = curvePrimPath;
    if (pointsIn.empty())
    {
        CARB_LOG_WARN("No points defined for the BasisCurves prim %s.", mCurvePrimPath.c_str());
        return;
    }

    const size_t pointsInCount = pointsIn.size();
    const size_t curveCount = curveVertexCounts.size();

    // Internal storage is PhysX math; the source-neutral points come straight off the array read
    // above and are converted here, at the boundary.
    std::vector<PxVec3> positions, tangents;
    std::vector<int> vertexCountsPerCurve(curveCount == 0 ? 1 : curveCount);
    std::vector<int> segmentCountsPerCurve(curveCount == 0 ? 1 : curveCount);
    std::vector<bool> linearPerCurve(curveCount == 0 ? 1 : curveCount, false);

    for (size_t curveIndex = 0, startIndex = 0; curveIndex < (curveCount == 0 ? 1 : curveCount); curveIndex++)
    {
        const size_t count = (curveCount == 0 ? pointsInCount : curveVertexCounts[curveIndex]);

        std::vector<PxVec3> controlPoints;
        for (size_t i = startIndex; i < startIndex + count; i++)
        {
            controlPoints.push_back(omni::physx::toPhysX(pointsIn[i]));
        }
        startIndex += count;

        // Two control points are a straight line whatever the basis or wrap: tessellate them as one
        // open linear segment so they get the same samplesPerSegment sampling the distance table
        // below indexes, and never as a "loop" that would only retrace the same edge.
        const bool twoPoints = (controlPoints.size() == 2);
        const eCurveType effectiveType = twoPoints ? eCurveType::Linear : curveType;
        SplineCurve curve(twoPoints ? eBasisCurveWrap::NonPeriodic : wrapType, basisType, effectiveType);
        linearPerCurve[curveIndex] = (effectiveType == eCurveType::Linear);
        const size_t segmentCount = curve.getSegmentCount(controlPoints.size());
        if (segmentCount == 0)
        {
            CARB_LOG_WARN("BasisCurves prim %s does not define enough control points for its basis and wrap mode.",
                          mCurvePrimPath.c_str());
            return;
        }

        std::vector<PxVec3> tessellatedPositions, tessellatedTangents;
        const bool normalizeLengths = true;
        const std::vector<float> curveDistances = approximateDistanceAlongCurve(
            curve, controlPoints, normalizeLengths, samplesPerSegment, &tessellatedPositions, &tessellatedTangents);
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

    // Samples that carry no curvature point: the first and last sample of EACH curve in the prim
    // (an osculating circle fitted across two curves' boundary is not a belt radius), and every
    // sample of a linear curve (its vertices are corners; the circle through three samples around
    // a corner is tiny and would blow up the contact modifier's radius scaling).
    std::vector<bool> noCurvature(positions.size(), false);
    for (size_t curveIndex = 0, start = 0; curveIndex < vertexCountsPerCurve.size(); ++curveIndex)
    {
        const size_t count = size_t(vertexCountsPerCurve[curveIndex]);
        for (size_t i = start; i < start + count; ++i)
            noCurvature[i] = linearPerCurve[curveIndex] || i == start || i == start + count - 1;
        start += count;
    }

    mCurvaturePoints.resize(positions.size());
    for (size_t i = 0; i < positions.size(); i++)
    {
        if (noCurvature[i])
        {
            mCurvaturePoints[i] = PxVec3(FLT_MAX);
        }
        else
        {
            const PxVec3 previousPoint = positions[i - 1];
            const PxVec3 currentPoint = positions[i];
            const PxVec3 nextPoint = positions[i + 1];

            // Check if the three points are collinear (forming a straight line)
            PxVec3 v1 = currentPoint - previousPoint;
            PxVec3 v2 = nextPoint - currentPoint;

            // Verify vectors have non-zero length before normalizing to prevent NaNs/asserts
            const float v1LengthSq = v1.magnitudeSquared();
            const float v2LengthSq = v2.magnitudeSquared();
            const float lengthEpsilonSq = 1e-10f;  // Squared epsilon for length check
            if (v1LengthSq < lengthEpsilonSq || v2LengthSq < lengthEpsilonSq)
            {
                // Degenerate case: coincident or nearly coincident points
                mCurvaturePoints[i] = PxVec3(FLT_MAX);
                continue;
            }

            v1.normalize();
            v2.normalize();
            const PxVec3 crossProduct = v1.cross(v2);
            const float crossProductMagnitude = crossProduct.magnitude();

            const float collinearityTolerance = 1e-4f;
            const bool areCollinear = (crossProductMagnitude < collinearityTolerance);

            if (areCollinear)
            {
                // Points form a straight line - no curvature
                mCurvaturePoints[i] = PxVec3(FLT_MAX);
            }
            else
            {
                // Compute the center of the osculating circle/sphere
                // This is the circumcenter of the three points
                const PxVec3 a = currentPoint - previousPoint;
                const PxVec3 b = nextPoint - previousPoint;
                const PxVec3 axb = a.cross(b);
                const float axbLengthSq = axb.magnitudeSquared();

                // Circumcenter formula: C = P1 + [(|b|² * a - |a|² * b) × (a × b)] / (2 * |a × b|²)
                const float aLengthSq = a.magnitudeSquared();
                const float bLengthSq = b.magnitudeSquared();
                const PxVec3 numerator = -(bLengthSq * a - aLengthSq * b).cross(axb);
                const PxVec3 circumcenter = previousPoint + numerator / (2.0f * axbLengthSq);

                mCurvaturePoints[i] = circumcenter;
            }
        }
    }

    mPoints = positions;
    mTangents = tangents;
    mCurveVertexCounts = vertexCountsPerCurve;
    mLinear = (curveType == eCurveType::Linear);

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
            renderBuffer.addLine(::physx::PxDebugLine(tr.transform(mPoints[i]), tr.transform(mPoints[i + 1]),
                                                      ::physx::PxDebugColor::eARGB_BLUE));

            if (mCurvaturePoints[i][0] != FLT_MAX)
            {
                renderBuffer.addLine(::physx::PxDebugLine(tr.transform(mPoints[i]), tr.transform(mCurvaturePoints[i]),
                                                          ::physx::PxDebugColor::eARGB_GREEN));
            }
        }
    }
}

bool SplinesCurve::getClosestPoint(const PxVec3& point, PxVec3& pointOnCurveOut, PxVec3& tangentOut, PxVec3& curvaturePointOut)
{
    PxVec3 pointOnCurve(0.0f), tangentAtPoint(0.0f);
    float closestDistanceSq = std::numeric_limits<float>::max();
    const float smallNumber = 1.e-6f;

    if (!mInitialized)
        return false;

    const PxVec3* const points = mPoints.data();
    const PxVec3* const tangents = mTangents.data();
    const PxVec3* const curvaturePoints = mCurvaturePoints.data();
    const int* curveVertexCounts = mCurveVertexCounts.data();

    PxVec3 curvaturePoint(FLT_MAX);

    if (points == nullptr || tangents == nullptr || curveVertexCounts == nullptr || curvaturePoints == nullptr)
    {
        return false;
    }

    const size_t curveCount = mCurveVertexCounts.size();

    for (size_t curveIndex = 0, pointsStart = 0, segmentsStart = 0; curveIndex < curveCount;
            curveIndex++)
    {
        const PxVec3* curvePoints = points + pointsStart;
        const PxVec3* curveTangents = tangents + pointsStart;
        // mCurvaturePoints is prim-global like mPoints; index it per curve too.
        const PxVec3* curveCurvaturePoints = curvaturePoints + pointsStart;

        const size_t vertexCount = curveVertexCounts[curveIndex];
        if (vertexCount < 2)
        {
            pointsStart += vertexCount;
            continue;
        }

#ifdef __AVX__
        const float* curveXPoints = mXPoints.data() + pointsStart;
        const float* curveYPoints = mYPoints.data() + pointsStart;
        const float* curveZPoints = mZPoints.data() + pointsStart;

        // Process 8 edges at a time using AVX. A curve of V samples has V - 1 edges, and a batch
        // starting at baseIdx loads samples baseIdx .. baseIdx + 8, so batch only whole groups of
        // eight EDGES: (V - 1) / 8 batches keeps the last load at index <= V - 1. (V / 8 batches
        // read sample V, one past the end, whenever V is a multiple of eight -- every periodic
        // curve before the closing sample was stored.)
        const size_t numAVXVertices = (vertexCount - 1) / 8;

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
            const PxVec3& a = curvePoints[i - 1];
            const PxVec3& b = curvePoints[i];

            const PxVec3 ab = b - a;
            const float abLengthSq = ab.magnitudeSquared();

            float t = 0.f;
            if (abLengthSq > smallNumber)
            {
                t = ab.dot(point - a) / abLengthSq;
                t = std::max(0.f, std::min(1.f, t));
            }

            const PxVec3 pointOnSegment = a + t * ab;
            const float pointDistSq = (point - pointOnSegment).magnitudeSquared();
            
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
            const PxVec3& a = curvePoints[closestIndex];
            const PxVec3& b = curvePoints[closestIndex + 1];
            pointOnCurve = a + closestT * (b - a);

            // A polyline's tangent is the hit edge itself; blending the samples' tangents would
            // smear a corner's direction change over the last edge before it.
            const PxVec3& ta = curveTangents[closestIndex];
            const PxVec3& tb = curveTangents[closestIndex + 1];
            tangentAtPoint = mLinear ? (b - a) : ta + closestT * (tb - ta);
            curvaturePoint = (closestT < 0.5f) ? curveCurvaturePoints[closestIndex] : curveCurvaturePoints[closestIndex + 1];
        }
#else
        // Original non-AVX implementation
        for (size_t i = 1; i < vertexCount; i++)
        {
            const PxVec3& a = curvePoints[i - 1];
            const PxVec3& b = curvePoints[i];

            const PxVec3 ab = b - a;
            const float abLengthSq = ab.magnitudeSquared();

            float t = 0.f;
            if (abLengthSq > smallNumber)
            {
                t = ab.dot(point - a) / abLengthSq;
                t = std::max(0.f, std::min(1.f, t));
            }

            const PxVec3 pointOnSegment = a + t * ab;
            const float pointDistSq = (point - pointOnSegment).magnitudeSquared();
            if (pointDistSq < closestDistanceSq)
            {
                closestDistanceSq = pointDistSq;
                pointOnCurve = pointOnSegment;

                const PxVec3& ta = curveTangents[i - 1];
                const PxVec3& tb = curveTangents[i];
                tangentAtPoint = mLinear ? ab : ta + t * (tb - ta);

                curvaturePoint = (t < 0.5f) ? curveCurvaturePoints[i - 1] : curveCurvaturePoints[i];
            }
        }
#endif
        pointsStart += vertexCount;
    }

    curvaturePointOut = curvaturePoint;
    pointOnCurveOut = pointOnCurve;
    tangentAtPoint.normalize();
    tangentOut = tangentAtPoint;
    return true;
}
