// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-INSPECTION-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <cstdint>
#include <vector>
#include <string>
namespace omni::physx::native
{
enum class InspectionResult : uint32_t { Success, Invalid, Stale, Busy, NotFound, Unavailable, Error };
enum class ObjectKind : uint32_t { Body=1, Joint=2, Material=3, Articulation=4, Shape=5 };
struct ObjectReference
{
    uint64_t attachment=0, object=0, key=0, generation=0;
    uint32_t kind=0, instance=UINT32_MAX;
};
struct BodyInspection
{
    ObjectReference identity;
    uint64_t step=0;
    // Type 0 static, 1 dynamic, 2 articulation link. Flags: 1 kinematic,
    // 2 sleeping, 4 CCD enabled, 8 simulation disabled. Locks are SDK axis bits.
    uint32_t type=0, flags=0, locks=0, reserved=0;
    float pose[7]{}, centerOfMass[7]{}, mass=0, inertia[3]{}, linearVelocity[3]{}, angularVelocity[3]{};
};
struct JointInspection
{
    ObjectReference identity, body0, body1;
    uint64_t step=0;
    // Type 0 generic, 1 fixed, 2 revolute, 3 prismatic, 4 spherical,
    // 5 distance, 6 D6, 7 articulation. Flags are SDK constraint bits for
    // ordinary joints; validFields bit 1 identifies those flags/break thresholds.
    uint32_t type=0, flags=0, validFields=0, reserved=0;
    float local0[7]{}, local1[7]{}, world0[7]{}, world1[7]{}, breakForce=0, breakTorque=0;
};
struct MaterialInspection
{
    ObjectReference identity;
    uint64_t step=0;
    float staticFriction=0, dynamicFriction=0, restitution=0;
    uint32_t frictionCombine=0, restitutionCombine=0, flags=0;
};
struct ArticulationDof
{
    ObjectReference link, joint;
    // Low-level cache index; axis uses PxArticulationAxis (twist, swing1,
    // swing2, x, y, z). Angular values radians, translational values scene length.
    uint32_t index=0, axis=0;
    float position=0, velocity=0, acceleration=0, force=0;
};
struct ArticulationInspection
{
    ObjectReference identity;
    uint64_t step=0;
    uint32_t flags=0, sleeping=0;
    float rootPose[7]{}, rootLinearVelocity[3]{}, rootAngularVelocity[3]{};
    std::vector<BodyInspection> links; // native low-level link-index order
    std::vector<ArticulationDof> dofs; // native cache order
};
// Exact-owner idle reads, no completion/stepping side effects. Record identities
// survive unrelated edits but reject replacement even at the same authored path.
// instance UINT32_MAX selects an uninstanced object; ambiguous paths are invalid.
// Copied poses are world xyzw; COM poses are actor-local. All numeric values use
// scene length/mass units and seconds. CPU only for live physics object state;
// GPU/DirectGPU returns Unavailable without invoking unsupported SDK reads.
// Failure preserves output. Callers retain the runtime while invoking these APIs.
InspectionResult resolveObject(uint64_t attachment,const char* path,ObjectKind kind,uint32_t instance,
                               ObjectReference& output) noexcept;
InspectionResult objectPath(const ObjectReference&,std::string&) noexcept;
InspectionResult inspectBody(const ObjectReference&,BodyInspection&) noexcept;
InspectionResult inspectJoint(const ObjectReference&,JointInspection&) noexcept;
InspectionResult inspectMaterial(const ObjectReference&,MaterialInspection&) noexcept;
InspectionResult inspectArticulation(const ObjectReference&,ArticulationInspection&) noexcept;
}
