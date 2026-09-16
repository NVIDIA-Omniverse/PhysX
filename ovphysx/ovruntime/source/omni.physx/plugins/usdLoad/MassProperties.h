// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-MASS-002
 * @covers AC-1 AC-2
 */

#pragma once

#include <carb/Types.h>

#include <foundation/PxMat33.h>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
::physx::PxQuat indexedRotation(uint32_t axis, float s, float c)
{
    float v[3] = { 0, 0, 0 };
    v[axis] = s;
    return ::physx::PxQuat(v[0], v[1], v[2], c);
}

uint32_t getNextIndex3(uint32_t i)
{
    return (i + 1 + (i >> 1)) & 3;
}


::physx::PxVec3 diagonalize(const ::physx::PxMat33& m, ::physx::PxQuat& massFrame)
{
    // jacobi rotation using quaternions (from an idea of Stan Melax, with fix for precision issues)

    const uint32_t MAX_ITERS = 24;

    ::physx::PxQuat q = ::physx::PxQuat(::physx::PxIdentity);

    ::physx::PxMat33 d(::physx::PxZero);
    for (uint32_t i = 0; i < MAX_ITERS; i++)
    {
        // PxMat33(q) uses the column-vector convention, i.e. it is the transpose of the
        // matrix GfMatrix3f(q) used to produce, but PhysX multiplies column-vector
        // style, so the conjugation transposes swap sides to match.
        const ::physx::PxMat33 axes(q);
        d = axes.getTranspose() * m * axes;

        // d holds the same nine floats the Gf version did, so the operator[]
        // indexing below reads the same elements regardless of symmetry.
        float d0 = fabs(d[1][2]), d1 = fabs(d[0][2]), d2 = fabs(d[0][1]);
        uint32_t a = uint32_t(d0 > d1 && d0 > d2 ? 0 :
                              d1 > d2            ? 1 :
                                                   2); // rotation axis index, from largest off-diagonal element

        uint32_t a1 = getNextIndex3(a), a2 = getNextIndex3(a1);
        if (d[a1][a2] == 0.0f || fabs(d[a1][a1] - d[a2][a2]) > 2e6 * fabs(2.0 * d[a1][a2]))
            break;

        float w = (d[a1][a1] - d[a2][a2]) / (2.0f * d[a1][a2]); // cot(2 * phi), where phi is the rotation angle
        float absw = fabs(w);

        ::physx::PxQuat r;
        if (absw > 1000)
            r = indexedRotation(a, 1 / (4 * w), 1.0f); // h will be very close to 1, so use small angle approx instead
        else
        {
            float t = 1 / (absw + sqrt(w * w + 1)); // absolute value of tan phi
            float h = 1 / sqrt(t * t + 1); // absolute value of cos phi

            r = indexedRotation(a, sqrt((1 - h) / 2) * ((w >= 0.0f) ? 1.0f : -1.0f), sqrt((1 + h) / 2));
        }

        q = (q * r).getNormalized();
    }

    massFrame = q;
    return ::physx::PxVec3(d[0][0], d[1][1], d[2][2]);
}

class MassProperties
{
public:
    MassProperties() : inertiaTensor(::physx::PxIdentity), centerOfMass(0.0f), mass(1.0f)
    {
    }

    /**
    \brief Construct from individual elements.
    */
    MassProperties(const float m, const ::physx::PxMat33& inertiaT, const ::physx::PxVec3& com)
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
    void translate(const ::physx::PxVec3& t)
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
    static ::physx::PxVec3 getMassSpaceInertia(const ::physx::PxMat33& inertia, ::physx::PxQuat& massFrame)
    {

        ::physx::PxVec3 diagT = diagonalize(inertia, massFrame);
        return diagT;
    }

    /**
    \brief Translate an inertia tensor using the parallel axis theorem

    \param[in] inertia The inertia tensor to translate.
    \param[in] mass The mass of the object.
    \param[in] t The relative frame to translate the inertia tensor to.
    \return The translated inertia tensor.
    */
    static ::physx::PxMat33 translateInertia(const ::physx::PxMat33& inertia, const float mass, const ::physx::PxVec3& t)
    {
        const ::physx::PxMat33 s(::physx::PxVec3(0, t[2], -t[1]), ::physx::PxVec3(-t[2], 0, t[0]),
                                 ::physx::PxVec3(t[1], -t[0], 0));

        ::physx::PxMat33 translatedIT = s * s.getTranspose() * mass + inertia;
        return translatedIT;
    }

    /**
    \brief Rotate an inertia tensor around the center of mass

    \param[in] inertia The inertia tensor to rotate.
    \param[in] q The rotation to apply to the inertia tensor.
    \return The rotated inertia tensor.
    */
    static ::physx::PxMat33 rotateInertia(const ::physx::PxMat33& inertia, const ::physx::PxQuat& q)
    {
        // PxMat33(q) holds the same floats as GfMatrix3f(q), but PhysX multiplies
        // column-vector style, so the operands and transposes swap places.
        const ::physx::PxMat33 m(q);
        ::physx::PxMat33 rotatedIT = m * inertia * m.getTranspose();
        return rotatedIT;
    }

    /**
    \brief Sum up individual mass properties.

    \param[in] props Array of mass properties to sum up.
    \param[in] transforms Reference transforms for each mass properties entry.
    \param[in] count The number of mass properties to sum up.
    \return The summed up mass properties.
    */
    static MassProperties sum(const MassProperties* props, const ::physx::PxTransform* transforms, const uint32_t count)
    {
        float combinedMass = 0.0f;
        ::physx::PxVec3 combinedCoM(0.0f);
        ::physx::PxMat33 combinedInertiaT = ::physx::PxMat33(::physx::PxZero);

        for (uint32_t i = 0; i < count; i++)
        {
            combinedMass += props[i].mass;
            const ::physx::PxVec3 comTm = transforms[i].transform(props[i].centerOfMass);
            combinedCoM += comTm * props[i].mass;
        }

        if (combinedMass > 0.f)
            combinedCoM /= combinedMass;

        for (uint32_t i = 0; i < count; i++)
        {
            const ::physx::PxVec3 comTm = transforms[i].transform(props[i].centerOfMass);
            combinedInertiaT += translateInertia(rotateInertia(props[i].inertiaTensor, transforms[i].q), props[i].mass,
                                                 combinedCoM - comTm);
        }

        return MassProperties(combinedMass, combinedInertiaT, combinedCoM);
    }


    ::physx::PxMat33 inertiaTensor; //!< The inertia tensor of the object.
    ::physx::PxVec3 centerOfMass; //!< The center of mass of the object.
    float mass; //!< The mass of the object.
};

} // namespace usdparser
} // namespace physx
} // namespace omni
