// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-MASS-001
 * @covers AC-1 AC-2 AC-3
 */

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cmath>

namespace omni::physics::parse
{

namespace
{

// USD authors centerOfMass in local space; apply the prim's
// local-to-world *scale* (no rotation) so the stored value is in
// world-aligned scaled local units.
inline carb::Float3 applyLocalScale(const IPhysicsSource& src, ObjectKey key, const carb::Float3& v)
{
    Matrix3d rot;
    carb::Float3 scale;
    src.getLocalToWorldRotationAndScale(key, rot, scale);
    return { v.x * scale.x, v.y * scale.y, v.z * scale.z };
}

inline bool finiteFloat3(const carb::Float3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

constexpr float kAlmostZero = 1e-5f;

inline bool quatNonZero(const carb::Float4& q)
{
    return std::fabs(q.x) > kAlmostZero || std::fabs(q.y) > kAlmostZero ||
           std::fabs(q.z) > kAlmostZero || std::fabs(q.w) > kAlmostZero;
}

} // namespace

MassApiData parseMassApi(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    MassApiData out;

    if (!src.hasSchema(key, tok.physicsMassAPI))
        return out;

    {
        float m;
        if (src.getAttribute(key, tok.physicsMass, m) && m > 0.0f)
            out.mass = m;
    }
    {
        float d;
        if (src.getAttribute(key, tok.physicsDensity, d) && d > 0.0f)
            out.density = d;
    }
    {
        carb::Float3 inertia;
        if (src.getAttribute(key, tok.physicsDiagonalInertia, inertia) &&
            (inertia.x > 0.0f || inertia.y > 0.0f || inertia.z > 0.0f))
        {
            out.hasInertia = true;
            out.diagonalInertia = inertia;
        }
    }
    {
        // centerOfMass — USD authors in local space. Legacy applies the
        // prim's local-to-world per-axis scale (no rotation). +/-inf is the
        // sentinel for "unset".
        carb::Float3 com;
        if (src.getAttribute(key, tok.physicsCenterOfMass, com) && finiteFloat3(com))
        {
            out.hasCenterOfMass = true;
            out.centerOfMass = applyLocalScale(src, key, com);
        }
    }
    {
        // principalAxes — USD authors as Float4 quaternion (x, y, z, w).
        // The all-zero quaternion is the sentinel for "unset".
        carb::Float4 axes;
        if (src.getAttribute(key, tok.physicsPrincipalAxes, axes) && quatNonZero(axes))
        {
            out.hasPrincipalAxes = true;
            out.principalAxes = axes;
        }
    }

    return out;
}

} // namespace omni::physics::parse
