// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Types.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
pxr::GfQuatf indexedRotation(uint32_t axis, float s, float c)
{
    float v[3] = { 0, 0, 0 };
    v[axis] = s;
    return pxr::GfQuatf(c, v[0], v[1], v[2]);
}

uint32_t getNextIndex3(uint32_t i)
{
    return (i + 1 + (i >> 1)) & 3;
}


pxr::GfVec3f diagonalize(const pxr::GfMatrix3f& m, pxr::GfQuatf& massFrame)
{
    // jacobi rotation using quaternions (from an idea of Stan Melax, with fix for precision issues)

    const uint32_t MAX_ITERS = 24;

    pxr::GfQuatf q = pxr::GfQuatf(1.0);

    pxr::GfMatrix3f d;
    for (uint32_t i = 0; i < MAX_ITERS; i++)
    {
        pxr::GfMatrix3f axes(q);
        d = axes * m * axes.GetTranspose();

        float d0 = fabs(d[1][2]), d1 = fabs(d[0][2]), d2 = fabs(d[0][1]);
        uint32_t a = uint32_t(d0 > d1 && d0 > d2 ? 0 :
                              d1 > d2            ? 1 :
                                                   2); // rotation axis index, from largest
                                            // off-diagonal
        // element

        uint32_t a1 = getNextIndex3(a), a2 = getNextIndex3(a1);
        if (d[a1][a2] == 0.0f || fabs(d[a1][a1] - d[a2][a2]) > 2e6 * fabs(2.0 * d[a1][a2]))
            break;

        float w = (d[a1][a1] - d[a2][a2]) / (2.0f * d[a1][a2]); // cot(2 * phi), where phi is the rotation angle
        float absw = fabs(w);

        pxr::GfQuatf r;
        if (absw > 1000)
            r = indexedRotation(a, 1 / (4 * w), 1.0f); // h will be very close to 1, so use small angle approx instead
        else
        {
            float t = 1 / (absw + sqrt(w * w + 1)); // absolute value of tan phi
            float h = 1 / sqrt(t * t + 1); // absolute value of cos phi

            r = indexedRotation(a, sqrt((1 - h) / 2) * ((w >= 0.0f) ? 1.0f : -1.0f), sqrt((1 + h) / 2));
        }

        q = (q * r).GetNormalized();
    }

    massFrame = q;
    return pxr::GfVec3f(d.GetColumn(0)[0], d.GetColumn(1)[1], d.GetColumn(2)[2]);
}

class MassProperties
{
public:
    MassProperties() : inertiaTensor(0.0f), centerOfMass(0.0f), mass(1.0f)
    {
        inertiaTensor[0][0] = 1.0;
        inertiaTensor[1][1] = 1.0;
        inertiaTensor[2][2] = 1.0;
    }

    /**
    \brief Construct from individual elements.
    */
    MassProperties(const float m, const pxr::GfMatrix3f& inertiaT, const pxr::GfVec3f& com)
        : inertiaTensor(inertiaT), centerOfMass(com), mass(m)
    {
    }

    /**
    \brief Scale mass properties.

    \param[in] scale The linear scaling factor to apply to the mass properties.
    \return The scaled mass properties.
    */
    MassProperties operator*(const float scale) const
    {
        return MassProperties(mass * scale, inertiaTensor * scale, centerOfMass);
    }

    /**
    \brief Translate the center of mass by a given vector and adjust the inertia tensor accordingly.

    \param[in] t The translation vector for the center of mass.
    */
    void translate(const pxr::GfVec3f& t)
    {
        inertiaTensor = translateInertia(inertiaTensor, mass, t);
        centerOfMass += t;
    }

    /**
    \brief Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.

    \param[in] inertia The inertia tensor to diagonalize.
    \param[out] massFrame The frame the diagonalized tensor refers to.
    \return The entries of the diagonalized inertia tensor.
    */
    static pxr::GfVec3f getMassSpaceInertia(const pxr::GfMatrix3f& inertia, pxr::GfQuatf& massFrame)
    {

        pxr::GfVec3f diagT = diagonalize(inertia, massFrame);
        return diagT;
    }

    /**
    \brief Translate an inertia tensor using the parallel axis theorem

    \param[in] inertia The inertia tensor to translate.
    \param[in] mass The mass of the object.
    \param[in] t The relative frame to translate the inertia tensor to.
    \return The translated inertia tensor.
    */
    static pxr::GfMatrix3f translateInertia(const pxr::GfMatrix3f& inertia, const float mass, const pxr::GfVec3f& t)
    {
        pxr::GfMatrix3f s;
        s.SetColumn(0, pxr::GfVec3f(0, t[2], -t[1]));
        s.SetColumn(1, pxr::GfVec3f(-t[2], 0, t[0]));
        s.SetColumn(2, pxr::GfVec3f(t[1], -t[0], 0));

        pxr::GfMatrix3f translatedIT = s * s.GetTranspose() * mass + inertia;
        return translatedIT;
    }

    /**
    \brief Rotate an inertia tensor around the center of mass

    \param[in] inertia The inertia tensor to rotate.
    \param[in] q The rotation to apply to the inertia tensor.
    \return The rotated inertia tensor.
    */
    static pxr::GfMatrix3f rotateInertia(const pxr::GfMatrix3f& inertia, const pxr::GfQuatf& q)
    {
        pxr::GfMatrix3f m(q);
        pxr::GfMatrix3f rotatedIT = m.GetTranspose() * inertia * m;
        return rotatedIT;
    }

    /**
    \brief Sum up individual mass properties.

    \param[in] props Array of mass properties to sum up.
    \param[in] transforms Reference transforms for each mass properties entry.
    \param[in] count The number of mass properties to sum up.
    \return The summed up mass properties.
    */
    static MassProperties sum(const MassProperties* props, const pxr::GfMatrix4f* transforms, const uint32_t count)
    {
        float combinedMass = 0.0f;
        pxr::GfVec3f combinedCoM(0.0f);
        pxr::GfMatrix3f combinedInertiaT = pxr::GfMatrix3f(0.0f);

        for (uint32_t i = 0; i < count; i++)
        {
            combinedMass += props[i].mass;
            const pxr::GfVec3f comTm = transforms[i].Transform(props[i].centerOfMass);
            combinedCoM += comTm * props[i].mass;
        }

        if (combinedMass > 0.f)
            combinedCoM /= combinedMass;

        for (uint32_t i = 0; i < count; i++)
        {
            const pxr::GfVec3f comTm = transforms[i].Transform(props[i].centerOfMass);
            combinedInertiaT += translateInertia(
                rotateInertia(props[i].inertiaTensor, pxr::GfQuatf(transforms[i].ExtractRotation().GetQuat())),
                props[i].mass, combinedCoM - comTm);
        }

        return MassProperties(combinedMass, combinedInertiaT, combinedCoM);
    }


    pxr::GfMatrix3f inertiaTensor; //!< The inertia tensor of the object.
    pxr::GfVec3f centerOfMass; //!< The center of mass of the object.
    float mass; //!< The mass of the object.
};

} // namespace usdparser
} // namespace physx
} // namespace omni
