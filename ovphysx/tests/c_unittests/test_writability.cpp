// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Tests for ovphysx_writability: the (object type, attribute) classification (REQ-INPUT-COVERAGE-001
// AC-2). It is a static property of the write API, so no instance, query or step is needed and these
// run as plain TEST() with nothing set up.
//
// These are snapshot checks, not a cross-check against the runtime. The per-class TESTs assert
// ovphysx_writability returns the values in kWritabilityTable for a representative subset of pairs.
// Both sides mirror the runtime's answers, so they do not catch a classification drifting from
// OvxPhysicsWrite/Read.cpp. The table and these expectations are updated by hand when those change.
// EveryDeclaredAttributeIsClassifiedSomewhere adds a completeness guard: every attribute the header
// declares must classify for some object type, so an attribute added to the write API but missing
// from the table is caught (it would be UNCLASSIFIED everywhere).

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-2
 * @maps_to TEST-INPUT-COVERAGE-001
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"

#include <cstddef>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>
#include <vector>

namespace
{
// A string literal carries its length in its type, so a literal attribute name needs no std::strlen.
// A runtime name (the header-scan case) already knows its own length and passes it explicitly.
template <size_t N>
ovx_string_or_token_t makeAttr(const char (&name)[N])
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = N - 1;
    return a;
}

ovx_string_or_token_t makeAttr(const char* name, size_t len)
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = len;
    return a;
}

template <size_t N>
ovphysx_writability_t classify(ovphysx_sim_object_type_t type, const char (&name)[N])
{
    ovx_string_or_token_t a = makeAttr(name);
    ovphysx_writability_t w = OVPHYSX_WRITABILITY_UNCLASSIFIED;
    EXPECT_EQ(ovphysx_writability(type, &a, &w).status, OVPHYSX_API_SUCCESS) << type << " / " << name;
    return w;
}
} // namespace

TEST(Writability, RigidBody)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_MASS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_FORCE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_WRENCH), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_LINEAR_ACCELERATION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_INVERSE_MASS), OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, ArticulationLink)
{
    // A link's pose follows from the root pose and DOFs, so it is readable but not writable.
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_MASS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_FORCE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_SIMULATION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, ArticulationJoint)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_TARGET),
              OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DRIVE_TYPE), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    // A limit interval exists only on an eLIMITED axis, so a finite limit on a free axis is refused
    // at write time. The attribute is writable only under that condition (AC-10).
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_LIMIT),
              OVPHYSX_WRITABILITY_CONDITIONAL);
}

TEST(Writability, VehicleWheel)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_DRIVE_TORQUE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_STEER_ANGLE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    // A wheel's pose is derived from the chassis + suspension + steer, so it is read-only.
    EXPECT_EQ(classify(OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, DeformablesAndParticles)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_POINTS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_REST_POINTS), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_DEFORMABLE_SURFACE, OVPHYSX_ATTR_VELOCITIES), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_PARTICLE_SET, OVPHYSX_ATTR_POINTS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS),
              OVPHYSX_WRITABILITY_WRITABLE);
}

TEST(Writability, Tendons)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_LIMIT), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_STIFFNESS), OVPHYSX_WRITABILITY_WRITABLE);
    // A spatial tendon's limit lives on the leaf attachment, so it is not a spatial-tendon attribute.
    EXPECT_EQ(classify(OVPHYSX_OBJECT_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_LIMIT), OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, WholeArticulation)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_JACOBIAN), OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, UnknownAndCrossTypeAreUnclassified)
{
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, "definitelyNotAnAttribute"), OVPHYSX_WRITABILITY_UNCLASSIFIED);
    // jointPosition is an articulation-joint attribute, not a rigid-body one.
    EXPECT_EQ(classify(OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_JOINT_POSITION), OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, RejectsNullArguments)
{
    ovx_string_or_token_t a = makeAttr(OVPHYSX_ATTR_POSITION);
    EXPECT_EQ(ovphysx_writability(OVPHYSX_OBJECT_RIGID_BODY, &a, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_writability_t w = OVPHYSX_WRITABILITY_WRITABLE;
    EXPECT_EQ(ovphysx_writability(OVPHYSX_OBJECT_RIGID_BODY, nullptr, &w).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, OVPHYSX_WRITABILITY_UNCLASSIFIED) << "out must be reset before the null-attribute check";
}

TEST(Writability, RejectsTokenOnlyAttribute)
{
    // A token-only key (no string) is rejected outright, not silently classified as UNCLASSIFIED.
    // Resolving a token would need a path dictionary this scene-free query lacks.
    ovx_string_or_token_t tok{};
    tok.token = 12345; // a non-zero interned token with the string field left empty
    tok.string.ptr = nullptr;
    tok.string.length = 0;

    ovphysx_writability_t w = OVPHYSX_WRITABILITY_WRITABLE;
    EXPECT_EQ(ovphysx_writability(OVPHYSX_OBJECT_RIGID_BODY, &tok, &w).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, RejectsUnknownObjectType)
{
    // An object_type outside ovphysx_sim_object_type_t (here DEFORMABLE_MATERIAL + 1, the shape of a
    // caller who passed a value from the separate OVPHYSX_OBJECT_TYPE_* tensor-binding domain) is
    // rejected, not returned as SUCCESS/UNCLASSIFIED. Otherwise a caller cannot tell an attribute the
    // type does not accept from a type that does not exist.
    ovx_string_or_token_t a = makeAttr(OVPHYSX_ATTR_POSITION);
    const ovphysx_sim_object_type_t bad =
        static_cast<ovphysx_sim_object_type_t>(OVPHYSX_OBJECT_DEFORMABLE_MATERIAL + 1);
    ovphysx_writability_t w = OVPHYSX_WRITABILITY_WRITABLE;
    EXPECT_EQ(ovphysx_writability(bad, &a, &w).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, OVPHYSX_WRITABILITY_UNCLASSIFIED) << "out must be reset before the unknown-type check";
}

// Completeness: every OVPHYSX_ATTR_* the public header declares must classify as something other
// than UNCLASSIFIED for at least one object type. This catches an attribute added to the write API
// but missing from kWritabilityTable, which would be UNCLASSIFIED everywhere and slip past the
// spot-checks above. The attribute vocabulary is read from the header rather than hand-listed, so a
// newly declared attribute is picked up automatically. The check enumerates the public vocabulary
// against the C-layer snapshot and does not re-derive from the runtime.
TEST(Writability, EveryDeclaredAttributeIsClassifiedSomewhere)
{
    std::ifstream header(OVPHYSX_SOURCE_DIR "/include/ovphysx/ovphysx_types.h");
    ASSERT_TRUE(header.is_open()) << "cannot open ovphysx_types.h";
    const std::string text((std::istreambuf_iterator<char>(header)), std::istreambuf_iterator<char>());

    // Matches both the single-line macros and the one line-continued (\) multi-line macro.
    const std::regex re(R"RE(#\s*define\s+OVPHYSX_ATTR_\w+\s+(?:\\\s+)?"([^"]+)")RE");
    std::vector<std::string> orphans;
    size_t count = 0;
    for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it)
    {
        ++count;
        const std::string name = (*it)[1].str();
        bool classifiedSomewhere = false;
        for (int t = 0; t <= 10 && !classifiedSomewhere; ++t)
        {
            ovx_string_or_token_t a = makeAttr(name.c_str(), name.size());
            ovphysx_writability_t w = OVPHYSX_WRITABILITY_UNCLASSIFIED;
            if (ovphysx_writability(static_cast<ovphysx_sim_object_type_t>(t), &a, &w).status == OVPHYSX_API_SUCCESS &&
                w != OVPHYSX_WRITABILITY_UNCLASSIFIED)
                classifiedSomewhere = true;
        }
        if (!classifiedSomewhere)
            orphans.push_back(name);
    }
    EXPECT_GT(count, 0u) << "no OVPHYSX_ATTR_* parsed from the header -- wrong path or regex?";
    EXPECT_TRUE(orphans.empty()) << orphans.size()
                                 << " declared attribute(s) are UNCLASSIFIED for every object type "
                                    "(missing from kWritabilityTable): "
                                 << (orphans.empty() ? std::string() : orphans.front());
}
