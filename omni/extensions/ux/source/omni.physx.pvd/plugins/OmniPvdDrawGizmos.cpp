// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdDrawGizmos.h"
#include "OmniPvdDebugDraw.h"
#include "OmniPvdGizmoUtils.h"

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <carb/Framework.h>

using namespace pxr;

namespace
{
    OmniPVDDebugVizData gOmniPVDDebugVizData;
    omni::kit::StageUpdateNode* gStageUpdateNode = nullptr;

    SdfLayerRefPtr gOmniPvdAnonLayer = nullptr;

    ////////////////////////////////////////////////////////////////////////////////
    // Omni PVD USD token definitions
    ////////////////////////////////////////////////////////////////////////////////

    //general
    const TfToken classToken("omni:pvdi:class");
    const TfToken typeToken("omni:pvd:type");

    //bounds
    const TfToken worldBoundsToken("omni:pvd:worldBounds");

    // center of mass / mass coordinate system
    const TfToken actorCMassLocalPoseToken("omni:pvd:cMassLocalPose");

    // PxActor linear velociyy
    const TfToken actorVelocityToken("omni:pvd:linearVelocity");

    //joints
    const TfToken actor0Token("omni:pvdh:actor0");
    const TfToken actor1Token("omni:pvdh:actor1");
    const TfToken actor0LocalPoseToken("omni:pvd:actor0LocalPose");
    const TfToken actor1LocalPoseToken("omni:pvd:actor1LocalPose");

    //prismatic joint
    const TfToken prismaticLimitLowerToken("omni:pvd:limitLower");
    const TfToken prismaticLimitUpperToken("omni:pvd:limitUpper");

    //revolute joint
    const TfToken revoluteLimitLowerToken("omni:pvd:limitLower");
    const TfToken revoluteLimitUpperToken("omni:pvd:limitUpper");

    //spherical joint
    const TfToken sphericalSwingYAngleToken("omni:pvd:swingYAngle");
    const TfToken sphericalSwingZAngleToken("omni:pvd:swingZAngle");

    //distance joint
    const TfToken distanceMinDistanceToken("omni:pvd:minDistance");
    const TfToken distanceMaxDistanceToken("omni:pvd:maxDistance");

    //d6 joint
    const TfToken d6MotionsToken("omni:pvd:motions");
    const TfToken d6LinearLimitLowerToken("omni:pvd:linearLimitLower");
    const TfToken d6LinearLimitUpperToken("omni:pvd:linearLimitUpper");
    const TfToken d6TwistLimitLowerToken("omni:pvd:twistLimitLower");
    const TfToken d6TwistLimitUpperToken("omni:pvd:twistLimitUpper");
    const TfToken d6SwingLimitYAngleToken("omni:pvd:swingLimitYAngle");
    const TfToken d6SwingLimitZAngleToken("omni:pvd:swingLimitZAngle");
    const TfToken d6PyramidSwingLimitYAngleMinToken("omni:pvd:pyramidSwingLimitYAngleMin");
    const TfToken d6PyramidSwingLimitYAngleMaxToken("omni:pvd:pyramidSwingLimitYAngleMax");
    const TfToken d6PyramidSwingLimitZAngleMinToken("omni:pvd:pyramidSwingLimitZAngleMin");
    const TfToken d6PyramidSwingLimitZAngleMaxToken("omni:pvd:pyramidSwingLimitZAngleMax");
    const TfToken d6DistanceLimitValueToken("omni:pvd:distanceLimitValue");

    //articulation joint
    const TfToken motionToken("omni:pvd:motion");
    const TfToken parentLinkToken("omni:pvdh:parentLink");
    const TfToken childLinkToken("omni:pvdh:childLink");
    const TfToken parentTranslationToken("omni:pvd:parentTranslation");
    const TfToken parentRotationToken("omni:pvd:parentRotation");
    const TfToken childTranslationToken("omni:pvd:childTranslation");
    const TfToken childRotationToken("omni:pvd:childRotation");
    const TfToken limitLowToken("omni:pvd:limitLow");
    const TfToken limitHighToken("omni:pvd:limitHigh");

    ////////////////////////////////////////////////////////////////////////////////
    // Helper types
    ////////////////////////////////////////////////////////////////////////////////
    struct RigidTransform
    {
        GfQuatf orientation;
        GfVec3f translation;
    };

    struct JointMotion
    {
        enum Enum
        {
            eLOCKED = 0,
            eLIMITED = 1,
            eFREE = 2
        };
    };

    struct ArticulationAxis
    {
        enum Enum
        {
            eTWIST = 0,
            eSWING1 = 1,
            eSWING2 = 2,
            eX = 3,
            eY = 4,
            eZ = 5
        };
    };

    struct D6Axis
    {
        enum Enum
        {
            eX = 0,
            eY = 1,
            eZ = 2,
            eTWIST = 3,
            eSWING1 = 4,
            eSWING2 = 5
        };
    };

    struct LinearAxis
    {
        enum Enum
        {
            eX = 0,
            eY = 1,
            eZ = 2
        };
        static LinearAxis::Enum from(ArticulationAxis::Enum axis) { return LinearAxis::Enum(axis - ArticulationAxis::eX); }
        static LinearAxis::Enum from(D6Axis::Enum axis) { return LinearAxis::Enum(axis - D6Axis::eX); }
    };

    struct AngularAxis
    {
        enum Enum
        {
            eTWIST = 0,
            eSWING1 = 1,
            eSWING2 = 2
        };
        static AngularAxis::Enum from(ArticulationAxis::Enum axis) { return AngularAxis::Enum(axis - ArticulationAxis::eTWIST); }
        static AngularAxis::Enum from(D6Axis::Enum axis) { return AngularAxis::Enum(axis - D6Axis::eTWIST); }
    };

    using ActorMap = TfHashMap<uint64_t, SdfPath>;

    ////////////////////////////////////////////////////////////////////////////////
    // Colors
    ////////////////////////////////////////////////////////////////////////////////

    uint32_t composeColourARBG(const uint32_t r, const uint32_t g, const uint32_t b)
    {
        const uint32_t finalCol = 0xff000000 | ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff);
        return finalCol;
    }

    const uint32_t sBBoxColor = composeColourARBG(0, 255, 0);
    const uint32_t sLimitColor = 0xBBFF4444;
    const uint32_t sLimitFreeColor = 0xBB44FF44;
    const uint32_t sBlueColor = 0xDB101080;
    const uint32_t sGreenColor = 0xDB108010;
    const uint32_t sRedColor = 0xDB801010;
    const uint32_t sYellowColor = 0xDB808010;

    const uint32_t sAxisColors[] = {
        0xFFAA6060, // X - Red
        0xFF71A376, // Y - Green
        0xFF4F7DA0  // Z - Blue
    };

    const uint32_t sJointAxisColors[] = {
        sRedColor,
        sGreenColor,
        sBlueColor
    };

    ////////////////////////////////////////////////////////////////////////////////
    // Macros for loading custom attributes
    ////////////////////////////////////////////////////////////////////////////////

#define GET_ATTRIBUTE_VALUE_OR_RETURN(type, variable, token, prim, timeCode) \
        UsdAttribute variable##Attr = prim.GetAttribute(token); \
        if (!variable##Attr) \
        { \
            return;\
        } \
        type variable; \
        variable##Attr.Get(&variable, timeCode);

#define GET_ATTRIBUTE_VALUE_OR_RETURN_V(type, variable, token, prim, timeCode, ret) \
        UsdAttribute variable##Attr = prim.GetAttribute(token); \
        if (!variable##Attr) \
        { \
            return ret;\
        } \
        type variable; \
        variable##Attr.Get(&variable, timeCode);

#define GET_ATTRIBUTE_ARRAY_OR_RETURN(type, count, variable, token, prim, timeCode) \
        GET_ATTRIBUTE_VALUE_OR_RETURN(VtArray<type>, variable, token, prim, timeCode); \
        if (variable.size() != count) \
        { \
            return;\
        }

#define GET_ATTRIBUTE_ARRAY_OR_RETURN_V(type, count, variable, token, prim, timeCode, ret) \
        GET_ATTRIBUTE_VALUE_OR_RETURN_V(VtArray<type>, variable, token, prim, timeCode, ret); \
        if (variable.size() != count) \
        { \
            return ret;\
        }

    ////////////////////////////////////////////////////////////////////////////////
    // Specialized functions to load Omni PVD types
    ////////////////////////////////////////////////////////////////////////////////

    template <typename T>
    void getAttributeArrayPVD(VtArray<T>& array, UsdAttribute& attr, double timeStamp)
    {
        VtValue arrayDataValue;
        attr.Get(&array, timeStamp);
    }

    GfQuatf physXQuatToUSD(const float* quat)
    {
        return GfQuatf(quat[3], quat[0], quat[1], quat[2]);
    }

    bool getAttributePosePVD(RigidTransform& pose, const UsdPrim prim, TfToken poseToken, double timeCode)
    {
        GET_ATTRIBUTE_ARRAY_OR_RETURN_V(float, 7, poseArray, poseToken, prim, timeCode, false);
        pose.orientation = physXQuatToUSD(&poseArray[0]);
        pose.translation = GfVec3f(poseArray[4], poseArray[5], poseArray[6]);
        return true;
    }

    bool getAttributePosePVD(RigidTransform& pose, const UsdPrim prim, TfToken positionToken, TfToken orientationToken, double timeCode)
    {
        GET_ATTRIBUTE_ARRAY_OR_RETURN_V(float, 3, positionArray, positionToken, prim, timeCode, false);
        GET_ATTRIBUTE_ARRAY_OR_RETURN_V(float, 4, orientationArray, orientationToken, prim, timeCode, false);
        pose.orientation = physXQuatToUSD(&orientationArray[0]);
        pose.translation = GfVec3f(positionArray[0], positionArray[1], positionArray[2]);
        return true;
    }

    std::array<JointMotion::Enum, 6> getAttributeMotionPVD(UsdPrim prim, TfToken motionsToken, double timeCode)
    {
        std::array<JointMotion::Enum, 6> motions = { JointMotion::eLOCKED, JointMotion::eLOCKED, JointMotion::eLOCKED, JointMotion::eLOCKED, JointMotion::eLOCKED, JointMotion::eLOCKED };
        GET_ATTRIBUTE_ARRAY_OR_RETURN_V(uint32_t, 6, vtMotions, motionsToken, prim, timeCode, motions);
        for (uint32_t i = 0; i < 6; ++i)
        {
            motions[i] = JointMotion::Enum(vtMotions[i]);
        }
        return motions;
    }

    TfToken getAttributeTypePVD(UsdPrim prim, double timeCode)
    {
        GET_ATTRIBUTE_VALUE_OR_RETURN_V(TfToken, types, typeToken, prim, timeCode, TfToken(""));
        return types;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Bounding Box draw function
    ////////////////////////////////////////////////////////////////////////////////

    void drawBoundingBox(OmniPVDDebugLines& debugLines, const UsdPrim prim, double timeCode, float lineWidth)
    {
        GET_ATTRIBUTE_ARRAY_OR_RETURN(float, 6, worldBoundsArray, worldBoundsToken, prim, timeCode);
        debugLines.drawBox(worldBoundsArray.data(), sBBoxColor, lineWidth);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Joint / Articulation Joint draw functions
    ////////////////////////////////////////////////////////////////////////////////

    void drawSquare(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, const GfVec3d& center,
        const GfVec3d normal, float halfExtent, uint32_t color, float lineWidth)
    {
        // create a transform to get default space to the given normal space
        const GfRotation rot(GfVec3d(1.0f, 0.0f, 0.0f), normal);
        const GfMatrix4d localToWorld(rot, center);

        const GfVec4d localPos0(0.0f, halfExtent, halfExtent, 1.0f);
        const GfVec4d worldPos0 = localPos0 * localToWorld;
        const GfVec4f point0 = GfVec4f(worldPos0 * transform);

        const GfVec4d localPos1(0.0f, -halfExtent, halfExtent, 1.0f);
        const GfVec4d worldPos1 = localPos1 * localToWorld;
        const GfVec4f point1 = GfVec4f(worldPos1 * transform);

        const GfVec4d localPos2(0.0f, -halfExtent, -halfExtent, 1.0f);
        const GfVec4d worldPos2 = localPos2 * localToWorld;
        const GfVec4f point2 = GfVec4f(worldPos2 * transform);

        const GfVec4d localPos3(0.0f, halfExtent, -halfExtent, 1.0f);
        const GfVec4d worldPos3 = localPos3 * localToWorld;
        const GfVec4f point3 = GfVec4f(worldPos3 * transform);

        debugLines.drawLine(point0.data(), point1.data(), color, lineWidth);
        debugLines.drawLine(point1.data(), point2.data(), color, lineWidth);
        debugLines.drawLine(point2.data(), point3.data(), color, lineWidth);
        debugLines.drawLine(point3.data(), point0.data(), color, lineWidth);
    }

    void drawDisc(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, const GfVec3d& center,
        const GfVec3d normal, float radius, uint32_t color, uint32_t tesselation, float lineWidth)
    {
        // create a transform to get default space to the given normal space
        const GfRotation rot(GfVec3d(1.0f, 0.0f, 0.0f), normal);
        const GfMatrix4d localToWorld(rot, center);

        const float tessStep = (2.0f * float(M_PI)) / tesselation;

        const GfVec4d localPos(0.0, 0.0, radius, 1.0);
        const GfVec4d worldPos = localPos * localToWorld;
        const GfVec4f point = GfVec4f(worldPos * transform);

        GfVec4f point0 = point;
        GfVec4f point1;
        for (uint32_t i = 1; i < tesselation; ++i)
        {
            const GfVec4d localPos1(0.0, radius * sin(i * tessStep), radius * cos(i * tessStep), 1.0);
            const GfVec4d worldPos1 = localPos1 * localToWorld;
            point1 = GfVec4f(worldPos1 * transform);
            debugLines.drawLine(point0.data(), point1.data(), color, lineWidth);
            point0 = point1;
        }
        debugLines.drawLine(point1.data(), point.data(), color, lineWidth);
    }

    void drawLinearLimit(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, LinearAxis::Enum axis, float lower, float upper, float scale, uint32_t color, float lineWidth)
    {
        GfVec3d capVecLower(0.0f);
        GfVec3d capVecUpper(0.0f);
        if (lower > upper)
        {
            const float lineDist = 10.0f * scale;
            capVecLower[axis] = -lineDist;
            capVecUpper[axis] = lineDist;
        }
        else
        {
            const float radius = 0.6f * scale;
            capVecLower[axis] = lower;
            capVecUpper[axis] = upper;
            GfVec3d normal(0.0);
            normal[axis] = 1.0;
            drawSquare(debugLines, transform, capVecLower, normal, radius, color, lineWidth);
            drawSquare(debugLines, transform, capVecUpper, normal, radius, color, lineWidth);
        }

        GfVec3f center0(transform.Transform(capVecLower));
        GfVec3f center1(transform.Transform(capVecUpper));
        debugLines.drawLine(center0.data(), center1.data(), color, lineWidth);
    }

    void drawAngularLimit(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, AngularAxis::Enum axis, float lower, float upper, bool isLimited, float scale, uint32_t color, float lineWidth)
    {
        const uint32_t tasselationBase = 24;
        uint32_t tesselation = tasselationBase;
        const uint32_t pointsCapacity = tasselationBase * 2 + 2;
        const float radius = 2.0f * scale;
        GfVec3f points[pointsCapacity];

        // Limit re-drawing the circle multiple times if the difference between angles 
        // gets bigger than a full 360 circle, as with fixed tesselation step
        // it will not even resamble a 3D circle anymore.
        const float twoPi = static_cast<float>(M_PI * 2.0f);
        const float reminder = fmod(upper-lower, twoPi);
        float endingAngle = lower + reminder;
        if (upper - lower >= twoPi)
        {
            endingAngle += twoPi;
            tesselation *= 2;
        }
        uint32_t lastPointIndex = tesselation + 2;

        const float tessStep = (endingAngle - lower) / tesselation;

        points[0] = GfVec3f(transform.Transform(GfVec3d(0.0)));

        for (uint32_t i = 0; i < tesselation + 1; ++i)
        {
            if (axis == AngularAxis::eTWIST)
            {
                const float v0 = cos(lower + i * tessStep);
                const float v1 = sin(lower + i * tessStep);
                points[i + 1] = GfVec3f(transform.Transform(GfVec3d(0.0, radius * v0, radius * v1)));
            }
            else if (axis == AngularAxis::eSWING1)
            {
                const float v0 = cos(lower + i * tessStep);
                const float v1 = sin(lower + i * tessStep);
                points[i + 1] = GfVec3f(transform.Transform(GfVec3d(radius * v0, 0.0, radius * v1)));
            }
            else if (axis == AngularAxis::eSWING2)
            {
                const float v0 = cos(lower + (float)M_PI / 2.0f + i * tessStep);
                const float v1 = sin(lower + (float)M_PI / 2.0f + i * tessStep);
                points[i + 1] = GfVec3f(transform.Transform(GfVec3d(radius * v0, radius * v1, 0.0)));
            }

            if (i != 0)
            {
                const GfVec3f& v0 = points[i];
                const GfVec3f& v1 = points[i + 1];
                const GfVec3f& v2 = points[0];
                debugLines.drawLine(v0.data(), v1.data(), color, lineWidth);
            }
            else if (isLimited)
            {
                const GfVec3f& v0 = points[1];
                const GfVec3f& v2 = points[0];
                debugLines.drawLine(v2.data(), v0.data(), color, lineWidth);
            }
        }
        if (isLimited)
        {
            const GfVec3f& v0 = points[lastPointIndex - 1];
            const GfVec3f& v2 = points[0];
            debugLines.drawLine(v2.data(), v0.data(), color, lineWidth);
        }
    }

    void drawDistanceLimit(OmniPVDDebugLines& debugLines, const GfVec3d& pos0, const GfVec3d& pos1, float minDistance, float maxDistance, float scale, uint32_t color, float lineWidth)
    {
        const float radius = 0.6f * scale;
        const uint32_t tesselation = 12;

        // create an axis between the 2 points
        GfVec3d posDiff = pos0 - pos1;

        // fall back on X axis if they are co-positional
        const double dist = posDiff.Normalize();
        if (dist < 0.001)
        {
            posDiff = GfVec3d(1.0, 0.0, 0.0);
        }

        // use the average world position of the two attach points as the reference point for the joint
        // the position of the joint object itself isn't important
        const GfVec3d posAvg = 0.5 * (pos0 + pos1);

        // figure out where the max end points are
        // if there is no max limit, just go slightly beyond the attach points
        GfVec3f max0Pos(pos0 + posDiff);
        GfVec3f max1Pos(pos1 - posDiff);
        if (maxDistance >= 0.0f)
        {
            const GfVec3d offset = 0.5 * double(maxDistance) * posDiff;
            max0Pos = GfVec3f(posAvg + offset);
            max1Pos = GfVec3f(posAvg - offset);
            drawDisc(debugLines, GfMatrix4d(1.0), max0Pos, posDiff, radius, sLimitColor, tesselation, lineWidth);
            drawDisc(debugLines, GfMatrix4d(1.0), max1Pos, posDiff, radius, sLimitColor, tesselation, lineWidth);
        }

        if (minDistance >= 0.0f)
        {
            const GfVec3d offset = 0.5 * double(minDistance) * posDiff;
            const GfVec3f min0Pos(posAvg + offset);
            const GfVec3f min1Pos(posAvg - offset);
            drawDisc(debugLines, GfMatrix4d(1.0), min0Pos, posDiff, radius, sLimitColor, tesselation, lineWidth);
            drawDisc(debugLines, GfMatrix4d(1.0), min1Pos, posDiff, radius, sLimitColor, tesselation, lineWidth);

            // draw lines connecting each of the min/max pairs
            debugLines.drawLine(min0Pos.data(), max0Pos.data(), sLimitColor, lineWidth);
            debugLines.drawLine(min1Pos.data(), max1Pos.data(), sLimitColor, lineWidth);
        }
        else
        {
            // only need to draw a single line connecting to two max values if min isn't specified
            debugLines.drawLine(max0Pos.data(), max1Pos.data(), sLimitColor, lineWidth);
        }
    }

    void drawConeLimit(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, AngularAxis::Enum axis, float yAngle, float zAngle,
        float scale, uint32_t color, uint32_t yColor, uint32_t zColor, float lineWidth)
    {
        const float coneScale = 2.0f * scale;

        const float tanQAngle0 = tanf(yAngle / 4.0f);
        const float tanQAngle1 = tanf(zAngle / 4.0f);

        const GfVec3f origin(transform.Transform(GfVec3d(0.0f, 0.0f, 0.0f)));
        GfVec3f prev = origin;

        //needs to be multiple of 4, to draw main axis
        const uint32_t nbLines = 32;

        for (uint32_t i = 0; i <= nbLines; i++)
        {
            GfVec3f a;
            const float angle = 2.0f * float(M_PI) / nbLines * i;
            const float c = cosf(angle);
            const float s = sinf(angle);

            //TODO fix axis code, axis == 1,2 are not exercised through omni.physx due to local transform adjustments
            //axis two code, (q) seems wrong, looking at the symmetry
            //code is so symmetrical, it should be possible to share for all axis.
            //also see JointAuthoring::drawSphericalJoint
            if (axis == AngularAxis::eTWIST)
            {
                const GfVec3f rv(0.0f, -tanQAngle0 * s, tanQAngle1 * c);
                const float rv2 = rv.GetLengthSq();
                const GfQuatf q = GfQuatf(1.0f - rv2, 0.0f, 2.0f * rv[1], 2.0f * rv[2]) * (1.0f / (1.0f + rv2));
                a = GfRotation(q).TransformDir(GfVec3f(1.0f, 0.0f, 0.0f));
            }
            else if (axis == AngularAxis::eSWING1)
            {
                const GfVec3f rv(tanQAngle1 * c, 0.0f, -tanQAngle0 * s);
                const float rv2 = rv.GetLengthSq();
                const GfQuatf q = GfQuatf(1.0f - rv2, 2.0f * rv[0], 0.0f, 2.0f * rv[2]) * (1.0f / (1.0f + rv2));
                a = GfRotation(q).TransformDir(GfVec3f(0.0f, 1.0f, 0.0f));
            }
            else if (axis == AngularAxis::eSWING2)
            {
                const GfVec3f rv(tanQAngle1 * c, -tanQAngle0 * s, 0.0f);
                const float rv2 = rv.GetLengthSq();
                const GfQuatf q = GfQuatf(1.0f - rv2, 2.0f * rv[1], 2.0f * rv[0], 0.0f) * (1.0f / (1.0f + rv2));
                a = GfRotation(q).TransformDir(GfVec3f(0.0f, 0.0f, 1.0f));
            }

            const GfVec3f v0(transform.Transform(GfVec3d(a[0] * coneScale, a[1] * coneScale, a[2] * coneScale)));
            debugLines.drawLine(v0.data(), prev.data(), sLimitColor, lineWidth);
            bool isQuaterLine = i % (nbLines/4) == 0;
            if (isQuaterLine)
            {
                uint32_t lineColor = i % (nbLines/2) == 0 ? zColor : yColor;
                debugLines.drawLine(origin.data(), v0.data(), lineColor, lineWidth);
            }
            prev = v0;
        }
    }

    void drawD6LinearLimits(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, std::array<JointMotion::Enum, 6> d6Motions, const UsdPrim prim, double timeCode, float scale, float lineWidth)
    {
        //render unlimited and check for limited
        bool anyLimits = false;
        for (D6Axis::Enum axis : { D6Axis::eX, D6Axis::eY, D6Axis::eZ })
        {
            if (d6Motions[axis] == JointMotion::eLIMITED)
            {
                anyLimits = true;
            }
            else if (d6Motions[axis] == JointMotion::eFREE)
            {
                LinearAxis::Enum linearAxis = LinearAxis::from(axis);
                uint32_t color = sJointAxisColors[linearAxis];
                drawLinearLimit(debugLines, transform, linearAxis, 1.0f, -1.0f, scale, color, lineWidth);
            }
        }

        //load linear limits and render limited
        GET_ATTRIBUTE_ARRAY_OR_RETURN(float, 3, d6LinearLimitLowerArray, d6LinearLimitLowerToken, prim, timeCode);
        GET_ATTRIBUTE_ARRAY_OR_RETURN(float, 3, d6LinearLimitUpperArray, d6LinearLimitUpperToken, prim, timeCode);
        for (D6Axis::Enum axis : { D6Axis::eX, D6Axis::eY, D6Axis::eZ })
        {
            LinearAxis::Enum linearAxis = LinearAxis::from(axis);
            uint32_t color = sJointAxisColors[linearAxis];
            if (d6Motions[axis] == JointMotion::eLIMITED)
            {
                float lower = d6LinearLimitLowerArray[axis];
                float upper = d6LinearLimitUpperArray[axis];
                drawLinearLimit(debugLines, transform, linearAxis, lower, upper, scale, color, lineWidth);
            }
        }
    }

    void drawD6TwistLimits(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, JointMotion::Enum motion, const UsdPrim prim, double timeCode, float scale, float lineWidth)
    {
        uint32_t color = sRedColor;
        if (motion == JointMotion::eLIMITED)
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMin, d6TwistLimitLowerToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMax, d6TwistLimitUpperToken, prim, timeCode);
            drawAngularLimit(debugLines, transform, AngularAxis::from(D6Axis::eTWIST), limitMin, limitMax, true, scale, color, lineWidth);
        }
        else if (motion == JointMotion::eFREE)
        {
            drawAngularLimit(debugLines, transform, AngularAxis::from(D6Axis::eTWIST), (float)-M_PI, (float)M_PI, false, scale, color, lineWidth);
        }
    }

    void drawD6Swing1Limits(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, JointMotion::Enum motion, const UsdPrim prim, double timeCode, float scale, float lineWidth)
    {
        uint32_t color = sGreenColor;
        if (motion == JointMotion::eLIMITED)
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMin, d6PyramidSwingLimitYAngleMinToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMax, d6PyramidSwingLimitYAngleMaxToken, prim, timeCode);
            drawAngularLimit(debugLines, transform, AngularAxis::from(D6Axis::eSWING1), limitMin, limitMax, true, scale, color, lineWidth);
        }
        else if (motion == JointMotion::eFREE)
        {
            drawAngularLimit(debugLines, transform, AngularAxis::from(D6Axis::eSWING1), (float)-M_PI, (float)M_PI, false, scale, color, lineWidth);
        }
    }

    void drawD6Swing2Limits(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, JointMotion::Enum motion, const UsdPrim prim, double timeCode, float scale, float lineWidth)
    {
        uint32_t color = sBlueColor;
        if (motion == JointMotion::eLIMITED)
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMin, d6PyramidSwingLimitZAngleMinToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMax, d6PyramidSwingLimitZAngleMaxToken, prim, timeCode);
            drawAngularLimit(debugLines, transform, AngularAxis::from(D6Axis::eSWING2), limitMin, limitMax, true, scale, color, lineWidth);
        }
        else if (motion == JointMotion::eFREE)
        {
            drawAngularLimit(debugLines, transform, AngularAxis::from(D6Axis::eSWING2), (float)-M_PI, (float)M_PI, false, scale, color, lineWidth);
        }
    }

    void drawD6ConeLimits(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, const UsdPrim prim, double timeCode, float scale, float lineWidth)
    {
        GET_ATTRIBUTE_VALUE_OR_RETURN(float, d6SwingLimitYAngle, d6SwingLimitYAngleToken, prim, timeCode);
        GET_ATTRIBUTE_VALUE_OR_RETURN(float, d6SwingLimitZAngle, d6SwingLimitZAngleToken, prim, timeCode);

        uint32_t color = sLimitColor;
        drawConeLimit(debugLines, transform, AngularAxis::eTWIST, d6SwingLimitYAngle, d6SwingLimitZAngle,
            scale, sLimitColor, sGreenColor, sBlueColor, lineWidth);
    }

    void drawD6DistanceLimits(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform0, const GfMatrix4d& transform1, const UsdPrim prim, double timeCode, float scale, float lineWidth)
    {
        GET_ATTRIBUTE_VALUE_OR_RETURN(float, d6DistanceLimitValue, d6DistanceLimitValueToken, prim, timeCode);
        uint32_t color = sYellowColor;
        drawDistanceLimit(debugLines, transform0.ExtractTranslation(), transform1.ExtractTranslation(), -1.0f, d6DistanceLimitValue, scale, color, lineWidth);
    }

    void drawD6Joint(OmniPVDDebugLines& debugLines, const UsdPrim prim, const double timeCode, const GfMatrix4d& transform0, const GfMatrix4d transform1,
        float scale, float lineWidth)
    {
        std::array<JointMotion::Enum, 6> d6Motions = getAttributeMotionPVD(prim, d6MotionsToken, timeCode);
        drawD6LinearLimits(debugLines, transform0, d6Motions, prim, timeCode, scale, lineWidth);

        //TODO render pyramid for swing limits, or render individual axis for cone limit.
        drawD6TwistLimits(debugLines, transform0, d6Motions[D6Axis::eTWIST], prim, timeCode, scale, lineWidth);
        drawD6Swing1Limits(debugLines, transform0, d6Motions[D6Axis::eSWING1], prim, timeCode, scale, lineWidth); 
        drawD6Swing2Limits(debugLines, transform0, d6Motions[D6Axis::eSWING2], prim, timeCode, scale, lineWidth);

        drawD6ConeLimits(debugLines, transform0, prim, timeCode, scale, lineWidth);
        drawD6DistanceLimits(debugLines, transform0, transform1, prim, timeCode, scale, lineWidth);
    }

    void drawAxis(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, float scale, float lineWidth)
    {
        const GfVec3f lineStart(transform.ExtractTranslation());
        for (uint32_t axisIdx = 0; axisIdx < 3; axisIdx++)
        {
            const GfVec3f lineEnd = lineStart + scale * GfVec3f(transform.GetRow3(axisIdx));
            debugLines.drawLine(lineStart.data(), lineEnd.data(), sAxisColors[axisIdx], lineWidth);
        }
    }

    void drawLineEndArrow(OmniPVDDebugLines& debugLines, float scale, float lineWidth, const GfVec3f& lineStart, const GfVec3f& lineEnd, const uint32_t color)
    {
        GfVec3f lineNorm = GfVec3f(lineEnd.data()[0] - lineStart.data()[0], lineEnd.data()[1] - lineStart.data()[1], lineEnd.data()[2] - lineStart.data()[2]);
        float length = lineNorm.GetLength();
        float arrowLength = length * 0.1f;
        if (arrowLength > 20.0f)
        {
            arrowLength = 20.0f;
        }

        float arrowWidth = arrowLength * 0.42f;
        if (arrowWidth < 4.0f)
        {
            arrowWidth = 4.0f;
        }
        lineNorm.Normalize();
        const GfVec3f lineEndArrow = lineEnd + lineNorm * arrowLength;

        debugLines.drawLineVarying(lineEnd.data(), lineEndArrow.data(), color, color, arrowWidth, 0.0f);
    }

    void drawAxisCol(OmniPVDDebugLines& debugLines, const GfMatrix4d& transform, float scale, float lineWidth, const uint32_t* colAxes)
    {
        const GfVec3f lineStart(transform.ExtractTranslation());
        for (uint32_t axisIdx = 0; axisIdx < 3; axisIdx++)
        {
            const GfVec3f lineEnd = lineStart + scale * GfVec3f(transform.GetRow3(axisIdx));
            debugLines.drawLine(lineStart.data(), lineEnd.data(), colAxes[axisIdx], lineWidth);
            drawLineEndArrow(debugLines, scale, lineWidth, lineStart, lineEnd, colAxes[axisIdx]);
        }
    }


    GfMatrix4d getJointActorTransform(const UsdPrim prim, const ActorMap& actorMap, const uint64_t actorHandle,
        const RigidTransform& actorLocalPose, const double timeCode)
    {
        UsdStageWeakPtr stage = prim.GetStage();
        GfMatrix4d actorTransform(1.0);
        auto actorIt = actorMap.find(actorHandle);
        if (actorIt != actorMap.end())
        {
            const SdfPath actorPath = actorIt->second;
            const UsdPrim actorPrim = stage->GetPrimAtPath(actorPath);
            if (actorPrim)
            {
                const UsdGeomXform actorXform(actorPrim);
                if (actorXform)
                {
                    actorTransform = actorXform.ComputeLocalToWorldTransform(timeCode);
                }
            }
        }
        const GfMatrix4d localTransform(GfRotation(actorLocalPose.orientation), actorLocalPose.translation);
        return localTransform * actorTransform;
    }

    void drawJoint(OmniPVDDebugLines& debugLines, const UsdPrim prim, const ActorMap& actorMap, const double timeCode, float scale, float lineWidth)
    {
        TfToken jointType = getAttributeTypePVD(prim, timeCode);
        if (jointType.IsEmpty())
        {
            return;
        }
        GET_ATTRIBUTE_VALUE_OR_RETURN(uint64_t, actor0Handle, actor0Token, prim, timeCode);
        GET_ATTRIBUTE_VALUE_OR_RETURN(uint64_t, actor1Handle, actor1Token, prim, timeCode);

        RigidTransform actor0LocalPose;
        RigidTransform actor1LocalPose;
        bool ret0 = getAttributePosePVD(actor0LocalPose, prim, actor0LocalPoseToken, timeCode);
        bool ret1 = getAttributePosePVD(actor1LocalPose, prim, actor1LocalPoseToken, timeCode);
        if (!ret0 || !ret1)
        {
            return;
        }
        const GfMatrix4d transform0 = getJointActorTransform(prim, actorMap, actor0Handle, actor0LocalPose, timeCode);
        const GfMatrix4d transform1 = getJointActorTransform(prim, actorMap, actor1Handle, actor1LocalPose, timeCode);

        drawAxis(debugLines, transform0, scale, lineWidth);
        drawAxis(debugLines, transform1, scale, lineWidth);

        if (jointType == TfToken("eSPHERICAL"))
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, yAngle, sphericalSwingYAngleToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, zAngle, sphericalSwingZAngleToken, prim, timeCode);
            drawConeLimit(debugLines, transform0, AngularAxis::eTWIST, yAngle, zAngle, scale, sRedColor, sGreenColor, sBlueColor, lineWidth);
        }
        else if (jointType == TfToken("eREVOLUTE"))
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMin, revoluteLimitLowerToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMax, revoluteLimitUpperToken, prim, timeCode);
            AngularAxis::Enum axis(AngularAxis::eTWIST);
            uint32_t color = sJointAxisColors[axis];
            if (limitMin <= limitMax)
            {
                drawAngularLimit(debugLines, transform0, axis, limitMin, limitMax, true, scale, color, lineWidth);
            }
            else
            {
                drawAngularLimit(debugLines, transform0, axis, (float)-M_PI, (float)M_PI, false, scale, color, lineWidth);
            }
        }
        else if (jointType == TfToken("ePRISMATIC"))
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMin, prismaticLimitLowerToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, limitMax, prismaticLimitUpperToken, prim, timeCode);
            uint32_t color = sJointAxisColors[LinearAxis::eX];
            if (limitMin <= limitMax)
            {
                drawLinearLimit(debugLines, transform0, LinearAxis::eX, limitMin, limitMax, scale, color, lineWidth);
            }
            else
            {
                drawLinearLimit(debugLines, transform0, LinearAxis::eX, 1.0f, -1.0f, scale, color, lineWidth);
            }
        }
        else if (jointType == TfToken("eDISTANCE"))
        {
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, minDistance, distanceMinDistanceToken, prim, timeCode);
            GET_ATTRIBUTE_VALUE_OR_RETURN(float, maxDistance, distanceMaxDistanceToken, prim, timeCode);
            GfVec3d pos0 = transform0.ExtractTranslation();
            GfVec3d pos1 = transform1.ExtractTranslation();
            drawDistanceLimit(debugLines, pos0, pos1, minDistance, maxDistance, scale, sLimitColor, lineWidth);
        }
        else if (jointType == TfToken("eD6"))
        {
            drawD6Joint(debugLines, prim, timeCode, transform0, transform1, scale, lineWidth);
        }
    }

    void drawArticulationJoint(OmniPVDDebugLines& debugLines, const UsdPrim prim, const ActorMap& actorMap, const double timeCode, float scale, float lineWidth)
    {
        TfToken jointType = getAttributeTypePVD(prim, timeCode);
        if (jointType.IsEmpty())
        {
            return;
        }
        GET_ATTRIBUTE_VALUE_OR_RETURN(uint64_t, parentLinkHandle, parentLinkToken, prim, timeCode);
        GET_ATTRIBUTE_VALUE_OR_RETURN(uint64_t, childLinkHandle, childLinkToken, prim, timeCode);

        RigidTransform parentLocalPose;
        RigidTransform childLocalPose;
        bool ret0 = getAttributePosePVD(parentLocalPose, prim, parentTranslationToken, parentRotationToken, timeCode);
        bool ret1 = getAttributePosePVD(childLocalPose, prim, childTranslationToken, childRotationToken, timeCode);
        if (!ret0 || !ret1)
        {
            return;
        }
        const GfMatrix4d transform0 = getJointActorTransform(prim, actorMap, parentLinkHandle, parentLocalPose, timeCode);
        const GfMatrix4d transform1 = getJointActorTransform(prim, actorMap, childLinkHandle, childLocalPose, timeCode);

        drawAxis(debugLines, transform0, scale, lineWidth);
        drawAxis(debugLines, transform1, scale, lineWidth);

        std::array<JointMotion::Enum, 6> motions = getAttributeMotionPVD(prim, motionToken, timeCode);
        GET_ATTRIBUTE_ARRAY_OR_RETURN(float, 6, limitsLow, limitLowToken, prim, timeCode);
        GET_ATTRIBUTE_ARRAY_OR_RETURN(float, 6, limitsHigh, limitHighToken, prim, timeCode);

        if (jointType == TfToken("ePRISMATIC"))
        {
            static ArticulationAxis::Enum sInvalid = ArticulationAxis::eTWIST;
            ArticulationAxis::Enum singleNonLocked = sInvalid;
            uint32_t numNonLocked = 0;
            for (ArticulationAxis::Enum axis : { ArticulationAxis::eX, ArticulationAxis::eY, ArticulationAxis::eZ })
            {
                if (motions[axis] != JointMotion::eLOCKED)
                {
                    singleNonLocked = axis;
                    numNonLocked++;
                }
            }

            if (numNonLocked == 1 && singleNonLocked != sInvalid)
            {
                LinearAxis::Enum linearAxis = LinearAxis::from(singleNonLocked);
                uint32_t color = sJointAxisColors[linearAxis];
                if (motions[singleNonLocked] == JointMotion::eLIMITED)
                {
                    float limitMin = limitsLow[singleNonLocked];
                    float limitMax = limitsHigh[singleNonLocked];
                    drawLinearLimit(debugLines, transform0, linearAxis, limitMin, limitMax, scale, color, lineWidth);
                }
                else
                {
                    drawLinearLimit(debugLines, transform0, linearAxis, 1.0f, -1.0f, scale, color, lineWidth);
                }
            }
        }
        else if (jointType == TfToken("eREVOLUTE") || jointType == TfToken("eREVOLUTE_UNWRAPPED"))
        {
            static ArticulationAxis::Enum sInvalid = ArticulationAxis::eX;
            ArticulationAxis::Enum singleNonLocked = sInvalid;
            uint32_t numNonLocked = 0;
            for (ArticulationAxis::Enum axis : { ArticulationAxis::eTWIST, ArticulationAxis::eSWING1, ArticulationAxis::eSWING2 })
            {
                if (motions[axis] != JointMotion::eLOCKED)
                {
                    singleNonLocked = axis;
                    numNonLocked++;
                }
            }

            if (numNonLocked == 1 && singleNonLocked != sInvalid)
            {
                AngularAxis::Enum angularAxis = AngularAxis::from(singleNonLocked);
                uint32_t color = sJointAxisColors[angularAxis];
                if (motions[singleNonLocked] == JointMotion::eLIMITED)
                {
                    float limitMin = limitsLow[singleNonLocked];
                    float limitMax = limitsHigh[singleNonLocked];
                    drawAngularLimit(debugLines, transform0, angularAxis, limitMin, limitMax, true, scale, color, lineWidth);
                }
                else
                {
                    drawAngularLimit(debugLines, transform0, angularAxis, (float)-M_PI, (float)M_PI, false, scale, color, lineWidth);
                }
            }
        }
        else if (jointType == TfToken("eSPHERICAL"))
        {
            //for now just render per axis limits
            for (ArticulationAxis::Enum axis : { ArticulationAxis::eTWIST, ArticulationAxis::eSWING1, ArticulationAxis::eSWING2 })
            {
                AngularAxis::Enum angularAxis = AngularAxis::from(axis);
                uint32_t color = sJointAxisColors[angularAxis];
                if (motions[axis] == JointMotion::eLIMITED)
                {
                    drawAngularLimit(debugLines, transform0, AngularAxis::from(axis), limitsLow[axis], limitsHigh[axis], true, scale, color, lineWidth);
                }
                else if (motions[axis] == JointMotion::eFREE)
                {
                    drawAngularLimit(debugLines, transform0, AngularAxis::from(axis), (float)-M_PI, (float)M_PI, false, scale, color, lineWidth);
                }
            }
        }
    }

////////////////////////////////////////////////////////////////////////////////
// Gathering prims and actor map
////////////////////////////////////////////////////////////////////////////////

    bool isVisiblePrim(UsdPrim prim, double timeCode)
    {
        const UsdGeomImageable imageable(prim);
        if (!imageable) return false;
        TfToken visible;
        imageable.GetVisibilityAttr().Get(&visible, timeCode);
        return (visible == UsdGeomTokens->inherited);
    }

    void gatherPrims(std::vector<SdfPath>* actors, std::vector<SdfPath>* joints, std::vector<SdfPath>* jointsRc, ActorMap* actorMap, UsdPrim prim, double timeCode)
    {        
        static TfToken handleToken("omni:pvdi:handle");
        const SdfPath& path = prim.GetPath();
        UsdAttribute classAttr = prim.GetAttribute(classToken);
        std::string className;
        classAttr.Get(&className);

        if (OmniPvdGizmoUtils::resolveActorType(className) != OmniPvdGizmoUtils::OmniPvdActorType::UndefinedActorType)
        {
            if (actors && isVisiblePrim(prim, timeCode))
            {
                actors->push_back(path);
            }
            if (actorMap)
            {                    
                UsdAttribute handleAttr = prim.GetAttribute(handleToken);
                uint64_t handle;
                handleAttr.Get(&handle);
                actorMap->insert(std::pair<uint64_t, SdfPath>(handle, path));
            }
        }
        else if (OmniPvdGizmoUtils::resolveJointType(className) != OmniPvdGizmoUtils::OmniPvdJointType::UndefinedJointType)
        {
            if (joints && isVisiblePrim(prim, timeCode))
            {
                joints->push_back(path);
            }
        }
        else if (className == "PxArticulationJointReducedCoordinate")
        {
            if (jointsRc && isVisiblePrim(prim, timeCode))
            {
                jointsRc->push_back(path);
            }
        }
        else if (actors && actorMap && (OmniPvdGizmoUtils::resolveGeometricSelectableType(className) != OmniPvdGizmoUtils::OmniPvdGeometricSelectableType::UndefinedGeometricSelectableType))
        {
            // Go up towards the actor and add it in
            prim = prim.GetParent();
            bool rootActorNotFound = true;
            while (prim && rootActorNotFound)
            {
                classAttr = prim.GetAttribute(classToken);
                classAttr.Get(&className);
                if (OmniPvdGizmoUtils::resolveActorType(className) != OmniPvdGizmoUtils::OmniPvdActorType::UndefinedActorType)
                {
                    if (isVisiblePrim(prim, timeCode))
                    {
                        UsdAttribute handleAttr = prim.GetAttribute(handleToken);
                        uint64_t handle;
                        handleAttr.Get(&handle);
                        auto actorIt = actorMap->find(handle);
                        if (actorIt == actorMap->end())
                        {                            
                            const SdfPath& path = prim.GetPath();
                            actors->push_back(path);
                            actorMap->insert(std::pair<uint64_t, SdfPath>(handle, path));
                        }
                    }
                    rootActorNotFound = false;
                }
                else
                {
                    prim = prim.GetParent();
                }
            }
        }
    }
    /*
    void gatherShapes(ActorMap& actorMap, UsdPrim prim, double timeCode)
    {
        const UsdGeomImageable imageable(prim);
        if (imageable)
        {
            const SdfPath& path = prim.GetPath();
            UsdAttribute classAttr = prim.GetAttribute(classToken);
            std::string className;
            classAttr.Get(&className);
            TfToken visible;
            imageable.GetVisibilityAttr().Get(&visible, timeCode);

            if (className == "PxShape")
            {
                static TfToken handleToken("omni:pvdi:handle");
                UsdAttribute handleAttr = prim.GetAttribute(handleToken);
                uint64_t handle;
                handleAttr.Get(&handle);
                actorMap.insert(std::pair<uint64_t, SdfPath>(handle, path));
            }
        }
    }

    // TODO : Get the shapes of the actor and add them to the map
    void gatherActors(ActorMap& actorMap, UsdPrim prim, double timeCode)
    {
        static TfToken handleToken("omni:pvdi:handle");
        //const UsdGeomImageable imageable(prim);
        //if (imageable)
        {            
            UsdAttribute classAttr = prim.GetAttribute(classToken);
            std::string className;
            classAttr.Get(&className);

            if (className == "PxActor" && isVisiblePrim(prim, timeCode))
            {                                
                UsdAttribute handleAttr = prim.GetAttribute(handleToken);
                uint64_t handle;
                handleAttr.Get(&handle);
                const SdfPath& path = prim.GetPath();
                actorMap.insert(std::pair<uint64_t, SdfPath>(handle, path));
            }
            else if (
                (className == "PxGeomBox") ||
                (className == "PxGeomCapsule") ||
                (className == "PxGeomPlane") ||
                (className == "PxGeomSphere") ||
                (className == "convexmesh_ref") ||
                (className == "heightfield_ref") ||
                (className == "trianglemesh_ref")
                )
            {
                // Go up towards the actor and add it in
                prim = prim.GetParent();
                bool rootActorNotFound = true;
                while (prim && rootActorNotFound)
                {
                    classAttr = prim.GetAttribute(classToken);
                    classAttr.Get(&className);
                    if (className == "PxActor")
                    {
                        if (isVisiblePrim(prim, timeCode))
                        {
                            UsdAttribute handleAttr = prim.GetAttribute(handleToken);
                            uint64_t handle;
                            handleAttr.Get(&handle);
                            const SdfPath& path = prim.GetPath();
                            actorMap.insert(std::pair<uint64_t, SdfPath>(handle, path));
                        }
                        rootActorNotFound = false;
                    }
                    else
                    {
                        prim = prim.GetParent();
                    }
                }                
            }

        }        
    }
    */
    bool isVisibleVizMode(OmniPVDDebugVizMode::Enum &vizMode)
    {
        return (vizMode == OmniPVDDebugVizMode::eAll) || (vizMode == OmniPVDDebugVizMode::eSelected);
    }

} //namespace

void crossVec(float* r, float *a, float* b)
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

void applyGizmosDrawCall(float, float, const omni::kit::StageUpdateSettings*, void*)
{
    const uint32_t colPastelRed_4_5 = composeColourARBG(225, 42, 31);
    const uint32_t colPastelRed_5_6 = composeColourARBG(255, 109, 65);
    const uint32_t colPastelRed_3_3 = composeColourARBG(198, 21, 117);
    const uint32_t colPastelRed_2_2 = composeColourARBG(162, 76, 144);

    const uint32_t colPastelGreen_5_2 = composeColourARBG(108, 147, 61);
    const uint32_t colPastelGreen_3_4 = composeColourARBG(25, 140, 146);
    const uint32_t colPastelGreen_2_4 = composeColourARBG(243, 186, 30); // yellow
    const uint32_t colPastelGreen_1_6 = composeColourARBG(243, 212, 152); // white

    const uint32_t colPastelBlue_1_1 = composeColourARBG(191, 221, 229);
    const uint32_t colPastelBlue_1_2 = composeColourARBG(121, 196, 218);
    const uint32_t colPastelBlue_1_3 = composeColourARBG(26, 180, 210);
    const uint32_t colPastelBlue_1_4 = composeColourARBG(111, 169, 216);

    const uint32_t colTFormAxes[3] = { colPastelRed_4_5,  colPastelGreen_5_2 , colPastelBlue_1_1 };
    const uint32_t colMassAxes[3] = { colPastelRed_2_2,  colPastelGreen_3_4 , colPastelBlue_1_3 };
   
    const uint32_t colCollisionPos = colPastelRed_3_3;
    const uint32_t colCollisionNorm = colPastelGreen_2_4;

    const uint32_t colVelocity = colPastelBlue_1_4;

    ////////////////////////////////////////////////////////////////////////////////
    // Is there a USD Stage context?
    ////////////////////////////////////////////////////////////////////////////////
    omni::usd::UsdContext* leContext = omni::usd::UsdContext::getContext();
    UsdStageRefPtr stage = leContext->getStage();
    if (!stage || gOmniPVDDebugVizData.getClosingState())
    {
        gOmniPVDDebugVizData.releaseDebugViz();
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Anything new to process?
    ////////////////////////////////////////////////////////////////////////////////
    uint64_t stageDirtyEvent;
    {
        std::lock_guard<carb::tasking::MutexWrapper> lock(*gOmniPVDDebugVizData.mNextStateMutex);
        stageDirtyEvent = gOmniPVDDebugVizData.mStageDirtyEvent;
    }
    double timeCode = floor(gOmniPVDDebugVizData.mTimeline->getCurrentTime() * stage->GetTimeCodesPerSecond());

    if ((timeCode == gOmniPVDDebugVizData.mProcessedTimeCode) && (gOmniPVDDebugVizData.mProcessedStageDirtyEvent == stageDirtyEvent))
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Nothing new to process
        ////////////////////////////////////////////////////////////////////////////////
        return;
    }

    gOmniPVDDebugVizData.mProcessedTimeCode = timeCode;
    gOmniPVDDebugVizData.mProcessedStageDirtyEvent = stageDirtyEvent;

    ////////////////////////////////////////////////////////////////////////////////
    // Is it an OmniPVD USD Stage : Is there a Prim called "/scenes" or "/Scenes"?
    ////////////////////////////////////////////////////////////////////////////////
    const SdfPath scenesPath = SdfPath("/scenes");
    const SdfPath scenesPathUpper = SdfPath("/Scenes");
    UsdPrim scenesPrim = stage->GetPrimAtPath(scenesPath);
    if (!scenesPrim)
    {
        scenesPrim = stage->GetPrimAtPath(scenesPathUpper);
    }
    if (!scenesPrim)
    {
        gOmniPVDDebugVizData.releaseDebugViz();
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Collect all PxSene Prims in a vector
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<UsdPrim> scenes;
    for (auto child : scenesPrim.GetChildren())
    {
        std::string typeName;
        child.GetAttribute(pxr::TfToken("omni:pvdi:class")).Get(&typeName);
        if (typeName == "PxScene")
        {
            scenes.push_back(child);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Did we collect any PxScene Prims?
    ////////////////////////////////////////////////////////////////////////////////
    if (scenes.size() < 1)
    {
        gOmniPVDDebugVizData.releaseDebugViz();
        return;
    }

    gOmniPVDDebugVizData.updateGizmoVizModes();

    ////////////////////////////////////////////////////////////////////////////////
    // For all gizmos scan them for which update modality they are binned for
    //   None | All | Selected
    ////////////////////////////////////////////////////////////////////////////////
    // For all gizmos that are selection::None
    //   invalidate them
    // For all gizmos that are selection::All
    //   draw gizmos for all Prims that are visible
    // For all gizmos that are selction::Selected
    //   draw gizmos for all Prims that are visible and selected
    ////////////////////////////////////////////////////////////////////////////////

    OmniPVDDebugLines& contactPointGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eContactPoint];
    OmniPVDDebugLines& contactNormalGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eContactNormal];
    OmniPVDDebugLines& contactErrorPointGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eContactErrorPoint];
    OmniPVDDebugLines& contactDistanceGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eContactDistance];
    OmniPVDDebugLines& restDistanceGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eRestDistance];
    OmniPVDDebugLines& boundingBoxGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eBoundingBox];
    OmniPVDDebugLines& jointGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eJoint];
    OmniPVDDebugLines& coordinateSystemGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eCoordinateSystem];
    OmniPVDDebugLines& centerOfMassGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eCenterOfMass];
    OmniPVDDebugLines& velocityGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eVelocity];
    OmniPVDDebugLines& transparencyGizmos = gOmniPVDDebugVizData.mGizmos[OmniPVDDebugGizmoSelection::eTransparency];
    
    float contactGizmoScale = gOmniPVDDebugVizData.mGizmoScale * contactPointGizmos.mScale;
    float centerOfMassGizmoScale = gOmniPVDDebugVizData.mGizmoScale * centerOfMassGizmos.mScale;
    float jointGizmoScale = gOmniPVDDebugVizData.mGizmoScale * jointGizmos.mScale;
    float coordinateSystemGizmoScale = gOmniPVDDebugVizData.mGizmoScale * coordinateSystemGizmos.mScale;
    float velocityGizmoScale = gOmniPVDDebugVizData.mGizmoScale * velocityGizmos.mScale;
    float transparencyGizmoScale = transparencyGizmos.mScale;

    OmniPVDDebugVizMode::Enum boundingBoxesVizMode = boundingBoxGizmos.mRenderedVizMode;
    OmniPVDDebugVizMode::Enum centerOfMassVizMode = centerOfMassGizmos.mRenderedVizMode;    
    OmniPVDDebugVizMode::Enum contactVizMode = contactPointGizmos.mRenderedVizMode;
    OmniPVDDebugVizMode::Enum coordinateSystemVizMode = coordinateSystemGizmos.mRenderedVizMode;
    OmniPVDDebugVizMode::Enum jointsVizMode = jointGizmos.mRenderedVizMode;    
    OmniPVDDebugVizMode::Enum transparencyVizMode = transparencyGizmos.mRenderedVizMode;
    OmniPVDDebugVizMode::Enum velocityVizMode = velocityGizmos.mRenderedVizMode;    
    

    std::vector<SdfPath> actorsAll;
    std::vector<SdfPath> actorsSelected;
    std::vector<SdfPath> jointsAll;
    std::vector<SdfPath> jointsSelected;
    std::vector<SdfPath> jointsRcAll;
    std::vector<SdfPath> jointsRcSelected;

    const bool gatherActorsAll =        
        (boundingBoxesVizMode == OmniPVDDebugVizMode::eAll) ||
        (centerOfMassVizMode == OmniPVDDebugVizMode::eAll) ||
        (contactVizMode == OmniPVDDebugVizMode::eAll) ||
        (coordinateSystemVizMode == OmniPVDDebugVizMode::eAll) ||        
        (transparencyVizMode == OmniPVDDebugVizMode::eAll) ||
        (velocityVizMode == OmniPVDDebugVizMode::eAll);

    const bool gatherActorsSelected =
        (boundingBoxesVizMode == OmniPVDDebugVizMode::eSelected) ||
        (centerOfMassVizMode == OmniPVDDebugVizMode::eSelected) ||
        (contactVizMode == OmniPVDDebugVizMode::eSelected) ||
        (coordinateSystemVizMode == OmniPVDDebugVizMode::eSelected) ||
        (transparencyVizMode == OmniPVDDebugVizMode::eSelected) ||
        (velocityVizMode == OmniPVDDebugVizMode::eSelected);



    bool gatherJointsAll = jointsVizMode == OmniPVDDebugVizMode::eAll;
    bool gatherJointsSelected = jointsVizMode == OmniPVDDebugVizMode::eSelected;
    bool gatherActorRefs = gatherJointsAll || gatherJointsSelected;

    ActorMap actorMap;
    ActorMap actorMapSelected;
    if (gatherActorsAll || gatherJointsAll || gatherActorRefs)
    {
        UsdPrimRange range = stage->Traverse();
        for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const UsdPrim prim = *iter;
            if (prim)
            {
                gatherPrims(gatherActorsAll ? &actorsAll : nullptr,
                            gatherJointsAll ? &jointsAll : nullptr,
                            gatherJointsAll ? &jointsRcAll : nullptr,
                            gatherActorRefs ? &actorMap : nullptr,
                            prim, timeCode);
            }
        }
    }

    if (gatherActorsSelected || gatherJointsSelected)
    {
        std::vector<SdfPath> selectedPaths = leContext->getSelection()->getSelectedPrimPathsV2();
        for (SdfPath path : selectedPaths)
        {
            UsdPrim prim = stage->GetPrimAtPath(path);
            if (prim)
            {
                gatherPrims(gatherActorsSelected ? &actorsSelected : nullptr,
                            gatherJointsSelected ? &jointsSelected : nullptr,
                            gatherJointsSelected ? &jointsRcSelected : nullptr,
                            &actorMapSelected,
                            prim, timeCode);
            }
        }        
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Contact data, needs attribs from the scene (points, normals)
    // Bounding boxes need attribs from actors
    ////////////////////////////////////////////////////////////////////////////////
    // pairsContactFacesIndices | pairsActors | pairsContactSeparations | pairsContactShapes

    const int nbrScenes = (int)scenes.size();
    const float pointLenHalf = contactGizmoScale * 0.25f; // used for the base vector of the contact for example
    const float lineWidth = 2.0f;
    const float lineWidthNorm = 1.5f;
    stage->SetInterpolationType(UsdInterpolationType::UsdInterpolationTypeHeld);
    gOmniPVDDebugVizData.initInterfaces();
    if (!gOmniPVDDebugVizData.mHasInterfaces)
    {
        gOmniPVDDebugVizData.releaseDebugViz();
        return;
    }
    
    if ((contactPointGizmos.mRenderedVizMode == OmniPVDDebugVizMode::eAll) || (contactPointGizmos.mRenderedVizMode == OmniPVDDebugVizMode::eSelected))
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Keep a counter of how many scenes were actually added
        ////////////////////////////////////////////////////////////////////////////////
        int nbrSceneContactsAdded = 0;
        for (int sceneId = 0; sceneId < nbrScenes; sceneId++)
        {
            UsdAttribute pairCountsAttr = scenes[sceneId].GetAttribute(TfToken("omni:pvd:pairCount"));
            UsdAttribute pairsActorsAttr = scenes[sceneId].GetAttribute(TfToken("omni:pvdh:pairsActors"));
            UsdAttribute pairsShapesAttr = scenes[sceneId].GetAttribute(TfToken("omni:pvdh:pairsContactShapes"));
            UsdAttribute pairsContactCountsAttr = scenes[sceneId].GetAttribute(TfToken("omni:pvd:pairsContactCounts"));
            UsdAttribute pairsContactPointsAttr = scenes[sceneId].GetAttribute(TfToken("omni:pvd:pairsContactPoints"));
            UsdAttribute pairsContactNormalsAttr = scenes[sceneId].GetAttribute(TfToken("omni:pvd:pairsContactNormals"));

            bool addThisSceneContacts = false;
            if (pairCountsAttr && pairsContactCountsAttr && pairsContactPointsAttr && pairsContactNormalsAttr && pairsActorsAttr && pairsShapesAttr)
            {
                addThisSceneContacts = true;
            }

            std::vector< double > timeSamples;
            int timeSampleIndex = -1;

            if (addThisSceneContacts)
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Get the frame to sample depending on the current timeline frame value
                ////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////
                // Is there such a timeCode in the timeSamples? Also we want timeCode + 1 for
                // collisions
                ////////////////////////////////////////////////////////////////////////////////        
                pairsContactPointsAttr.GetTimeSamples(&timeSamples);
                double timeCodeFind = timeCode + 1.0;

                ////////////////////////////////////////////////////////////////////////////////
                // So we have possibly an update of contacts
                ////////////////////////////////////////////////////////////////////////////////
                int nbrTimeSamples = static_cast<int>(timeSamples.size());
                for (int i = 0; i < nbrTimeSamples; i++)
                {
                    const double timeCodeFromIndex = timeSamples[i];
                    if (timeCodeFind < timeCodeFromIndex)
                    {
                        break;
                    }
                    if (timeCodeFind == timeCodeFromIndex)
                    {
                        timeSampleIndex = i;
                        break;
                    }
                }
                if (timeSampleIndex < 0)
                {
                    addThisSceneContacts = false;
                }
            }          

            if (addThisSceneContacts)
            {
                ////////////////////////////////////////////////////////////////////////////////
                // Only execute startRenderLines on the first scene instance that has contacts
                ////////////////////////////////////////////////////////////////////////////////
                if (nbrSceneContactsAdded < 1) 
                {
                    const double timeStamp = timeSamples[timeSampleIndex];
                    contactPointGizmos.startRenderLines(timeStamp);
                    contactNormalGizmos.startRenderLines(timeStamp);
                }
                nbrSceneContactsAdded++;

                if (contactPointGizmos.mRenderedVizMode == OmniPVDDebugVizMode::eAll)
                {
                    const double timeStamp = timeSamples[timeSampleIndex];
                    VtArray<float> positions;
                    VtArray<float> normals;
                    getAttributeArrayPVD(positions, pairsContactPointsAttr, timeStamp);
                    getAttributeArrayPVD(normals, pairsContactNormalsAttr, timeStamp);
                    const float normScale = contactGizmoScale;
                    const int nbrPositions = static_cast<int>(positions.size() / 3);
                    float pos0[3];
                    float pos1[3];
                   
                    ////////////////////////////////////////////////////////////////////////////////
                    // Contact normals
                    ////////////////////////////////////////////////////////////////////////////////
                    int nbrNormals = static_cast<int>(normals.size() / 3);
                    if (nbrNormals > nbrPositions)
                    {
                        nbrNormals = nbrPositions;
                    }
                    float posf[3];
                    float dirf[3];
                    for (int s = 0; s < nbrNormals; s++)
                    {
                        const float posX = positions[3 * s + 0];
                        const float posY = positions[3 * s + 1];
                        const float posZ = positions[3 * s + 2];

                        if ((posX != posX) || (posY != posY) || (posX != posX))
                            continue; // NaN! Run!

                        posf[0] = posX;
                        posf[1] = posY;
                        posf[2] = posZ;

                        dirf[0] = normals[3 * s + 0];
                        dirf[1] = normals[3 * s + 1];
                        dirf[2] = normals[3 * s + 2];

                        if (contactNormalGizmos.drawNormal(posf, dirf, normScale, colCollisionNorm, lineWidth))
                        {
                            ////////////////////////////////////////////////////////////////////////////////
                            // Normal is in dirf, now calculate the rotation matrix and get the plane vectors
                            ////////////////////////////////////////////////////////////////////////////////
                            float upVec[3];
                            upVec[0] = 1.0f; upVec[1] = 0.0f; upVec[2] = 0.0f; // (1,0,0)

                            float tanVec0[3];
                            crossVec(tanVec0, dirf, upVec);
                            if ( ( abs(tanVec0[0]) + abs(tanVec0[1]) + abs(tanVec0[2]) ) < 0.001f ) // did it work?
                            {
                                upVec[0] = 0.0f; upVec[1] = 1.0f; upVec[2] = 0.0f; // (0,1,0)
                                crossVec(tanVec0, dirf, upVec);
                            }
                            float tanVec1[3];
                            crossVec(tanVec1, dirf, tanVec0);

                            ////////////////////////////////////////////////////////////////////////////////
                            // Now draw the rotated base:
                            //   tanVec0 as "X" and tanVec1 as "Y"
                            ////////////////////////////////////////////////////////////////////////////////

                            ////////////////////////////////////////////////////////////////////////////////
                            // Contact point : X axis line
                            ////////////////////////////////////////////////////////////////////////////////
                            pos0[0] = posX - pointLenHalf * tanVec0[0];
                            pos0[1] = posY - pointLenHalf * tanVec0[1];
                            pos0[2] = posZ - pointLenHalf * tanVec0[2];

                            pos1[0] = posX + pointLenHalf * tanVec0[0];
                            pos1[1] = posY + pointLenHalf * tanVec0[1];
                            pos1[2] = posZ + pointLenHalf * tanVec0[2];

                            contactPointGizmos.drawLine(pos0, pos1, colCollisionPos, lineWidth);

                            ////////////////////////////////////////////////////////////////////////////////
                            // Contact point : Y axis line
                            ////////////////////////////////////////////////////////////////////////////////
                            pos0[0] = posX - pointLenHalf * tanVec1[0];
                            pos0[1] = posY - pointLenHalf * tanVec1[1];
                            pos0[2] = posZ - pointLenHalf * tanVec1[2];

                            pos1[0] = posX + pointLenHalf * tanVec1[0];
                            pos1[1] = posY + pointLenHalf * tanVec1[1];
                            pos1[2] = posZ + pointLenHalf * tanVec1[2];
                            contactPointGizmos.drawLine(pos0, pos1, colCollisionPos, lineWidth);
                        }
                    }                    
                }
                else if (contactPointGizmos.mRenderedVizMode == OmniPVDDebugVizMode::eSelected)
                {
                    const double timeStamp = timeSamples[timeSampleIndex];
                    VtArray<float> positions;
                    VtArray<float> normals;
                    getAttributeArrayPVD(positions, pairsContactPointsAttr, timeStamp);
                    getAttributeArrayPVD(normals, pairsContactNormalsAttr, timeStamp);
       
                    VtArray<uint32_t> pairsContactCounts;
                    getAttributeArrayPVD(pairsContactCounts, pairsContactCountsAttr, timeStamp);

                    const float normScale = contactGizmoScale;

                    VtArray<uint64_t> actorPairHandles;
                    getAttributeArrayPVD(actorPairHandles, pairsActorsAttr, timeStamp);

                    ////////////////////////////////////////////////////////////////////////////////
                    // Go over the pairs of handles, check the first one, then the second one and
                    // if either of are in the map, then draw the contacts for those.
                    ////////////////////////////////////////////////////////////////////////////////

                    const uint64_t nbrActorPairs = actorPairHandles.size() / 2;

                    float pos0[3];
                    float pos1[3];

                    float posf[3];
                    float dirf[3];

                    uint32_t actorPairOffset = 0;
                    uint32_t contactOffset = 0;

                    for (int p = 0; p< nbrActorPairs; p++)
                    {
                        const uint64_t actor0Handle = actorPairHandles[actorPairOffset + 0];
                        const uint64_t actor1Handle = actorPairHandles[actorPairOffset + 1];

                        bool actorSelectedInPair = false;

                        auto actorIt = actorMapSelected.find(actor0Handle);
                        if (actorIt != actorMapSelected.end())
                        {
                            actorSelectedInPair = true;
                        }
                        if (!actorSelectedInPair)
                        {
                            auto actorIt = actorMapSelected.find(actor1Handle);
                            if (actorIt != actorMapSelected.end())
                            {
                                actorSelectedInPair = true;
                            }
                        }

                        const uint32_t nbrContactsInPair = pairsContactCounts[p];

                        if (actorSelectedInPair)
                        {
                            uint32_t contacTriOffset = contactOffset * 3;
                            for (uint32_t c=0; c < nbrContactsInPair; c++)
                            {                    
                                uint32_t contacTriOffsetLocal = contacTriOffset;
                                contacTriOffset += 3;

                                const float posX = positions[contacTriOffsetLocal + 0];
                                const float posY = positions[contacTriOffsetLocal + 1];
                                const float posZ = positions[contacTriOffsetLocal + 2];

                                if ((posX != posX) || (posY != posY) || (posX != posX))
                                    continue; // NaN! Run!

                                // Get the normal
                                posf[0] = posX;
                                posf[1] = posY;
                                posf[2] = posZ;

                                dirf[0] = normals[contacTriOffsetLocal + 0];
                                dirf[1] = normals[contacTriOffsetLocal + 1];
                                dirf[2] = normals[contacTriOffsetLocal + 2];

                                if (contactNormalGizmos.drawNormal(posf, dirf, normScale, colCollisionNorm, lineWidth))
                                {
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Normal is in dirf, now calculate the rotation matrix and get the plane vectors
                                    ////////////////////////////////////////////////////////////////////////////////
                                    float upVec[3];
                                    upVec[0] = 1.0f; upVec[1] = 0.0f; upVec[2] = 0.0f; // (1,0,0)

                                    float tanVec0[3];
                                    crossVec(tanVec0, dirf, upVec);
                                    if ( ( abs(tanVec0[0]) + abs(tanVec0[1]) + abs(tanVec0[2]) ) < 0.001f ) // did it work?
                                    {
                                        upVec[0] = 0.0f; upVec[1] = 1.0f; upVec[2] = 0.0f; // (0,1,0)
                                        crossVec(tanVec0, dirf, upVec);
                                    }
                                    float tanVec1[3];
                                    crossVec(tanVec1, dirf, tanVec0);

                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Now draw the rotated base:
                                    //   tanVec0 as "X" and tanVec1 as "Y"
                                    ////////////////////////////////////////////////////////////////////////////////

                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Contact point : X axis line
                                    ////////////////////////////////////////////////////////////////////////////////
                                    pos0[0] = posX - pointLenHalf * tanVec0[0];
                                    pos0[1] = posY - pointLenHalf * tanVec0[1];
                                    pos0[2] = posZ - pointLenHalf * tanVec0[2];

                                    pos1[0] = posX + pointLenHalf * tanVec0[0];
                                    pos1[1] = posY + pointLenHalf * tanVec0[1];
                                    pos1[2] = posZ + pointLenHalf * tanVec0[2];

                                    contactPointGizmos.drawLine(pos0, pos1, colCollisionPos, lineWidth);

                                    ////////////////////////////////////////////////////////////////////////////////
                                    // Contact point : Y axis line
                                    ////////////////////////////////////////////////////////////////////////////////
                                    pos0[0] = posX - pointLenHalf * tanVec1[0];
                                    pos0[1] = posY - pointLenHalf * tanVec1[1];
                                    pos0[2] = posZ - pointLenHalf * tanVec1[2];

                                    pos1[0] = posX + pointLenHalf * tanVec1[0];
                                    pos1[1] = posY + pointLenHalf * tanVec1[1];
                                    pos1[2] = posZ + pointLenHalf * tanVec1[2];
                                    contactPointGizmos.drawLine(pos0, pos1, colCollisionPos, lineWidth);
                                }
                            }
                        }
                        contactOffset += nbrContactsInPair;
                        actorPairOffset += 2;
                    }

                    /*
                    const uint64_t nbrContacts = positions.size() / 3;

                    int pairOffset = 0;

                    float pos0[3];
                    float pos1[3];

                    float posf[3];
                    float dirf[3];

                    for (int i = 0; i < nbrContacts; i++)
                    {
                        const uint64_t shape0Handle = shapeHandles[i * 2 + 0];
                        const uint64_t shape1Handle = shapeHandles[i * 2 + 1];

                        bool shapeSelectedInPair = false;

                        auto actorIt = shapesMapSelected.find(shape0Handle);
                        if (actorIt != shapesMapSelected.end())
                        {
                            shapeSelectedInPair = true;
                        }
                        if (!shapeSelectedInPair)
                        {
                            auto actorIt = shapesMapSelected.find(shape1Handle);
                            if (actorIt != shapesMapSelected.end())
                            {
                                shapeSelectedInPair = true;
                            }
                        }

                        if (shapeSelectedInPair)
                        {
                            {
                                const float posX = positions[3 * i + 0];
                                const float posY = positions[3 * i + 1];
                                const float posZ = positions[3 * i + 2];

                                if ((posX != posX) || (posY != posY) || (posX != posX))
                                    continue; // NaN! Run!
                                ////////////////////////////////////////////////////////////////////////////////
                                // Contact point : X axis line
                                ////////////////////////////////////////////////////////////////////////////////
                                pos0[0] = posX - pointLenHalf;
                                pos0[1] = posY;
                                pos0[2] = posZ;

                                pos1[0] = posX + pointLenHalf;
                                pos1[1] = posY;
                                pos1[2] = posZ;

                                contactPointGizmos.drawLine(pos0, pos1, colPos, lineWidth);

                                ////////////////////////////////////////////////////////////////////////////////
                                // Contact point : Y axis line
                                ////////////////////////////////////////////////////////////////////////////////
                                pos0[0] = posX;
                                pos0[1] = posY - pointLenHalf;
                                pos0[2] = posZ;

                                pos1[0] = posX;
                                pos1[1] = posY + pointLenHalf;
                                pos1[2] = posZ;

                                contactPointGizmos.drawLine(pos0, pos1, colPos, lineWidth);
                            }
                            {
                                const float posX = positions[3 * i + 0];
                                const float posY = positions[3 * i + 1];
                                const float posZ = positions[3 * i + 2];

                                if ((posX != posX) || (posY != posY) || (posX != posX))
                                    continue; // NaN! Run!

                                posf[0] = posX;
                                posf[1] = posY;
                                posf[2] = posZ;

                                dirf[0] = normals[3 * i + 0];
                                dirf[1] = normals[3 * i + 1];
                                dirf[2] = normals[3 * i + 2];

                                contactNormalGizmos.drawNormal(posf, dirf, normScale, colNorm, lineWidth);
                            }                
                        }
            
                    }
                    */
                }
            }
        }
        if (nbrSceneContactsAdded > 0)
        {
            contactPointGizmos.initLineInstances();
            contactNormalGizmos.initLineInstances();

            // Init lines for error, distance, rest distance
        }
        else
        {
            contactPointGizmos.releaseLines();            
            contactNormalGizmos.releaseLines();

            contactErrorPointGizmos.releaseLines();
            contactDistanceGizmos.releaseLines();
            restDistanceGizmos.releaseLines();
        }
    }
    else
    {
        contactPointGizmos.releaseLines();
        contactNormalGizmos.releaseLines();

        contactErrorPointGizmos.releaseLines();
        contactDistanceGizmos.releaseLines();
        restDistanceGizmos.releaseLines();
    }

    static UsdGeomXformCache xfCache;
    xfCache.SetTime(timeCode);

    ////////////////////////////////////////////////////////////////////////////////
    // Actor bounding boxes : [...]
    ////////////////////////////////////////////////////////////////////////////////
    if (isVisibleVizMode(boundingBoxesVizMode))
    {
        std::vector<SdfPath>& boundingBoxActors = boundingBoxesVizMode == OmniPVDDebugVizMode::eAll ? actorsAll : actorsSelected;
        if (boundingBoxActors.size())
        {
            VtArray<float> worldBoundsArray;
            boundingBoxGizmos.startRenderLines(timeCode);
            for (SdfPath path: boundingBoxActors)
            {
                UsdPrim prim = stage->GetPrimAtPath(path);
                if (prim)
                {
                    drawBoundingBox(boundingBoxGizmos, prim, timeCode, lineWidth);
                }
            }
            boundingBoxGizmos.initLineInstances();
        }
        else
        {
            boundingBoxGizmos.releaseLines();
        }
    }
    else
    {
        boundingBoxGizmos.releaseLines();
    }

    if (isVisibleVizMode(transparencyVizMode))
    {
        if (gOmniPvdAnonLayer)
        {
            auto layerStack = stage->GetLayerStack(true);
            if (std::find(layerStack.begin(), layerStack.end(), gOmniPvdAnonLayer) == layerStack.end())
            {
                gOmniPvdAnonLayer = nullptr;
            }
        }

        if (!gOmniPvdAnonLayer)
        {
            gOmniPvdAnonLayer = SdfLayer::CreateAnonymous("/omnipvd_anon");
            stage->GetSessionLayer()->InsertSubLayerPath(gOmniPvdAnonLayer->GetIdentifier(), 0);
        }
        pxr::UsdEditContext editContext(stage, gOmniPvdAnonLayer);

        if (gOmniPvdAnonLayer)
        {
            gOmniPvdAnonLayer->Clear();
        }

        if (transparencyVizMode == OmniPVDDebugVizMode::eAll)
        {

        }

        std::vector<SdfPath>& refFrameActors = transparencyVizMode == OmniPVDDebugVizMode::eAll ? actorsAll : actorsSelected;


        if (transparencyVizMode == OmniPVDDebugVizMode::eSelected)
        {
            std::vector<SdfPath> selectedPaths = leContext->getSelection()->getSelectedPrimPathsV2();
            for (SdfPath path : selectedPaths)
            {
                UsdPrim prim = stage->GetPrimAtPath(path);
                if (prim)
                {
                    const UsdGeomImageable imageable(prim);
                    if (imageable)
                    {
                        imageable.GetVisibilityAttr().Set(UsdGeomTokens->invisible);
                    }
                }
            }
        }
    }
    else
    {
        if (gOmniPvdAnonLayer)
        {
            gOmniPvdAnonLayer->Clear();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Coordinate systems : <-|->
    ////////////////////////////////////////////////////////////////////////////////
    if (isVisibleVizMode(coordinateSystemVizMode))
    {
        std::vector<SdfPath>& refFrameActors = coordinateSystemVizMode == OmniPVDDebugVizMode::eAll ? actorsAll : actorsSelected;
        if (refFrameActors.size())
        {
            coordinateSystemGizmos.startRenderLines(timeCode);
            for (SdfPath path : refFrameActors)
            {
                UsdPrim prim = stage->GetPrimAtPath(path);
                if (prim)
                {
                    pxr::GfMatrix4d localToWorld = xfCache.GetLocalToWorldTransform(prim);
                    drawAxisCol(coordinateSystemGizmos, localToWorld, coordinateSystemGizmoScale, lineWidth, colTFormAxes);
                }
            }
            coordinateSystemGizmos.initLineInstances();
        }
        else
        {
            coordinateSystemGizmos.releaseLines();
        }
    }
    else
    {
        coordinateSystemGizmos.releaseLines();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Center Of Mass reference frames : <-[*]->
    ////////////////////////////////////////////////////////////////////////////////
    if (isVisibleVizMode(centerOfMassVizMode))
    {
        std::vector<SdfPath>& centerOfMassActors = centerOfMassVizMode == OmniPVDDebugVizMode::eAll ? actorsAll : actorsSelected;
        if (centerOfMassActors.size())
        {
            centerOfMassGizmos.startRenderLines(timeCode);
            for (SdfPath path : centerOfMassActors)
            {
                UsdPrim prim = stage->GetPrimAtPath(path);
                if (prim)
                {
                    // Check if not static object
                    UsdAttribute classAttr = prim.GetAttribute(classToken);
                    if (classAttr) {
                        std::string className;
                        classAttr.Get(&className);
                        if (className != "PxRigidStatic")
                        {
                            pxr::GfMatrix4d localToWorld = xfCache.GetLocalToWorldTransform(prim);
                            // Extract the mass reference frame
                            RigidTransform actorMassPose;
                            bool ret0 = getAttributePosePVD(actorMassPose, prim, actorCMassLocalPoseToken, timeCode);
                            const GfMatrix4d localMassTransform(GfRotation(actorMassPose.orientation), actorMassPose.translation);
                            const GfMatrix4d globalMassTransform  = localMassTransform * localToWorld;
                            drawAxisCol(centerOfMassGizmos, globalMassTransform, centerOfMassGizmoScale, lineWidth, colMassAxes);
                        }
                    }
                }
            }
            centerOfMassGizmos.initLineInstances();
        }
        else
        {
            centerOfMassGizmos.releaseLines();
        }
    }
    else
    {
        centerOfMassGizmos.releaseLines();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Velocities : ------->
    ////////////////////////////////////////////////////////////////////////////////
    if (isVisibleVizMode(velocityVizMode))
    {
        std::vector<SdfPath>& velocityActors = velocityVizMode == OmniPVDDebugVizMode::eAll ? actorsAll : actorsSelected;
        if (velocityActors.size())
        {
            velocityGizmos.startRenderLines(timeCode);
            VtArray<float> velocityVt;
            for (SdfPath path : velocityActors)
            {
                UsdPrim prim = stage->GetPrimAtPath(path);
                if (prim)
                {
                    // Get the velocity attriibute
                    UsdAttribute velocityAttr = prim.GetAttribute(actorVelocityToken);
                    if (velocityAttr)
                    {
                        velocityAttr.Get(&velocityVt, timeCode);
                        pxr::GfMatrix4d localToWorld = xfCache.GetLocalToWorldTransform(prim);

                        // Extract the mass reference frame
                        RigidTransform actorMassPose;
                        bool ret0 = getAttributePosePVD(actorMassPose, prim, actorCMassLocalPoseToken, timeCode);
                        const GfMatrix4d localMassTransform(GfRotation(actorMassPose.orientation), actorMassPose.translation);

                        const GfMatrix4d globalMassTransform = localMassTransform * localToWorld;

                        const GfVec3f lineStart(globalMassTransform.ExtractTranslation());
                        const GfVec3f velocity = GfVec3f(velocityVt[0], velocityVt[1], velocityVt[2]);
                        
                        const GfVec3f lineEnd = lineStart + velocityGizmoScale * velocity;

                        velocityGizmos.drawLine(lineStart.data(), lineEnd.data(), colVelocity, lineWidth);

                        drawLineEndArrow(velocityGizmos, velocityGizmoScale, lineWidth, lineStart, lineEnd, colVelocity);
                    }
                }
            }
            velocityGizmos.initLineInstances();
        }
        else
        {
            velocityGizmos.releaseLines();
        }
    }
    else
    {
        velocityGizmos.releaseLines();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Joints and Articulation Joints --==()==--
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<SdfPath>& joints = jointsVizMode == OmniPVDDebugVizMode::eAll ? jointsAll : jointsSelected;
    std::vector<SdfPath>& jointsRc = jointsVizMode == OmniPVDDebugVizMode::eAll ? jointsRcAll : jointsRcSelected;
    if (joints.size() + jointsRc.size())
    {
        jointGizmos.startRenderLines(timeCode);
        for (SdfPath path: joints)
        {
            const UsdPrim prim = stage->GetPrimAtPath(path);
            if (prim)
            {
                drawJoint(jointGizmos, prim, actorMap, timeCode, jointGizmoScale, lineWidth);
            }
        }
        for (SdfPath path: jointsRc)
        {
            const UsdPrim prim = stage->GetPrimAtPath(path);
            if (prim)
            {
                drawArticulationJoint(jointGizmos, prim, actorMap, timeCode, jointGizmoScale, lineWidth);
            }
        }
        jointGizmos.initLineInstances();
    }
    else
    {
        jointGizmos.releaseLines();
    }
}

void initGizmoMutex()
{
    gOmniPVDDebugVizData.initMutex();
}

void applyGizmos()
{
    gOmniPVDDebugVizData.initEventSubs();
    gOmniPVDDebugVizData.tickStageDirty();

    omni::kit::StageUpdatePtr stageUpdate = omni::kit::getStageUpdate();
    omni::kit::StageUpdateNodeDesc desc = { 0 };
    desc.displayName = "OmniPVD Gizmos";
    desc.order = omni::kit::update::eIUsdStageUpdatePhysicsUI;
    desc.onPrimAdd = nullptr;
    desc.onPrimOrPropertyChange = nullptr;
    desc.onPrimRemove = nullptr;
    desc.onUpdate = applyGizmosDrawCall;
    desc.onStop = nullptr;
    desc.onResume = nullptr;
    desc.onPause = nullptr;
    if (stageUpdate)
        gStageUpdateNode = stageUpdate->createStageUpdateNode(desc);
}

void selectGizmo(uint32_t selectedGizmo, uint32_t selectionState)
{
    if (selectedGizmo >= OmniPVDDebugGizmoType::eNbrEnums)
    {
        return;
    }
    if (selectionState >= OmniPVDDebugVizMode::eNbrEnums)
    {
        return;
    }
    OmniPVDDebugGizmoType::Enum gizmoTypeEnum = OmniPVDDebugGizmoType::Enum(selectedGizmo);
    OmniPVDDebugVizMode::Enum selectionStateEnum = OmniPVDDebugVizMode::Enum(selectionState);
    switch (gizmoTypeEnum)
    {
    case OmniPVDDebugGizmoType::eBoundingBox:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eBoundingBox, selectionStateEnum);
        break;
    }
    case OmniPVDDebugGizmoType::eContact:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eContactPoint, selectionStateEnum);
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eContactErrorPoint, selectionStateEnum);
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eContactNormal, selectionStateEnum);
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eContactDistance, selectionStateEnum);
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eRestDistance, selectionStateEnum);
        break;
    }
    case OmniPVDDebugGizmoType::eCenterOfMass:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eCenterOfMass, selectionStateEnum);
        break;
    }
    case OmniPVDDebugGizmoType::eVelocity:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eVelocity, selectionStateEnum);
        break;
    }
    case OmniPVDDebugGizmoType::eCoordinateSystem:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eCoordinateSystem, selectionStateEnum);
        break;
    }
    case OmniPVDDebugGizmoType::eJoint:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eJoint, selectionStateEnum);
        break;
    }
    case OmniPVDDebugGizmoType::eTransparency:
    {
        gOmniPVDDebugVizData.setGizmoVizMode(OmniPVDDebugGizmoSelection::eTransparency, selectionStateEnum);
        break;
    }
    default:
    {
        break;
    }
    }

    if (gOmniPVDDebugVizData.mTimelineEvtSub &&
        gOmniPVDDebugVizData.mStageEvtSub[0].get() != carb::eventdispatcher::kInvalidObserver)
    {
        gOmniPVDDebugVizData.tickStageDirty();
    }
}

void setGizmoScale(uint32_t selectedGizmo, float scale)
{
    if (selectedGizmo >= OmniPVDDebugGizmoType::eNbrEnums)
    {
        return;
    }
    OmniPVDDebugGizmoType::Enum gizmoTypeEnum = OmniPVDDebugGizmoType::Enum(selectedGizmo);

    switch (gizmoTypeEnum)
    {
    case OmniPVDDebugGizmoType::eBoundingBox:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eBoundingBox, scale);
        break;
    }
    case OmniPVDDebugGizmoType::eContact:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eContactPoint, scale);
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eContactErrorPoint, scale);
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eContactNormal, scale);
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eContactDistance, scale);
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eRestDistance, scale);
        break;
    }
    case OmniPVDDebugGizmoType::eCenterOfMass:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eCenterOfMass, scale);
        break;
    }
    case OmniPVDDebugGizmoType::eVelocity:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eVelocity, scale);
        break;
    }
    case OmniPVDDebugGizmoType::eCoordinateSystem:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eCoordinateSystem, scale);
        break;
    }
    case OmniPVDDebugGizmoType::eJoint:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eJoint, scale);
        break;
    }
    case OmniPVDDebugGizmoType::eTransparency:
    {
        gOmniPVDDebugVizData.setGizmoScale(OmniPVDDebugGizmoSelection::eTransparency, scale);
        break;
    }
    default:
    {
        break;
    }
    }

    if (gOmniPVDDebugVizData.mTimelineEvtSub &&
        gOmniPVDDebugVizData.mStageEvtSub[0].get() != carb::eventdispatcher::kInvalidObserver)
    {
        gOmniPVDDebugVizData.tickStageDirty();
    }
}

void setGizmoScale(float scale)
{
    gOmniPVDDebugVizData.setGizmoScale(scale);
}

void cleanupDebugViz()
{
    if (gStageUpdateNode)
    {
        omni::kit::StageUpdatePtr stageUpdate = omni::kit::getStageUpdate();
        if (stageUpdate)
        {
            stageUpdate->destroyStageUpdateNode(gStageUpdateNode);
        }
        gStageUpdateNode = nullptr;
    }
    gOmniPVDDebugVizData.releaseEventSubs();
    gOmniPVDDebugVizData.releaseDebugViz();
    gOmniPVDDebugVizData.releaseInterfaces();
}


