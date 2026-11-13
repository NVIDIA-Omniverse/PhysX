<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Runtime unit audit

The registry describes the numeric values crossing runtime read/write sessions.
`distance` and `mass` denote the live simulation's chosen length and mass units;
`sec`, `deg` and `rad` denote seconds, degrees and radians. It contains no metrics
assembler or population scale exponents. In particular, `dimensionless` is a
unit label, not the number `1`.

This audit uses the bundled PhysX headers and the runtime accessors below.
SDK paths are relative to the PhysX SDK. Unless qualified with a module name,
runtime accessor paths are under `ovruntime/source/omni.physx/plugins/`.
`omni.physics.parse/` paths are relative to `ovruntime/source/`. The runtime accessors take precedence
where they explicitly convert SDK values.
This is a source audit; the metadata tests do not claim independent live
conformance measurements for every attribute family.

| Quantity | SDK evidence and runtime boundary | Runtime unit |
|---|---|---|
| Position, velocity, angular velocity, mass and inertia | `include/PxRigidBody.h`, `PxRigidActor.h`; `tensors/cpu/CpuRigidBodyView.cpp` getters and matching GPU views | `distance`, `distance / sec`, `rad / sec`, `mass`, `mass * distance^2`; inverse properties use reciprocal units |
| Forces and contact impulses | `include/PxRigidBody.h`, `PxSimulationEventCallback.h`; rigid wrench setters and `OvxPhysicsRead.cpp` contact output | Force: `mass * distance / sec^2`; torque: `mass * distance^2 / sec^2`; linear impulse: `mass * distance / sec`. Contact force uses the runtime's impulse/time conversion; event impulses retain impulse units. |
| Angular joint state, targets and limits | `include/PxArticulationJointReducedCoordinate.h` and `solver/PxSolverDefs.h` use radians. `OvxPhysicsShared.cpp` constructs per-axis conversion records; `tensors/ArticulationDofOvStageRecord.h` defines forward/inverse conversions. | `deg` and `deg / sec` for angular axes; `distance` and `distance / sec` for linear axes. `UsdInterface.cpp` enables angular conversion and disables it only for prismatic axes. |
| Joint drive stiffness and damping | `include/solver/PxSolverDefs.h`, `PxArticulationDrive`; `tensors/base/BaseArticulationView.cpp` applies the inverse angular factor to both gains, for both drive types. | Angular force drives use `mass * distance^2 / (sec^2 * deg)` and `mass * distance^2 / (sec * deg)`. Angular acceleration drives use `rad / (deg * sec^2)` and `rad / (deg * sec)`: the denominator is converted, the SDK output acceleration is not. Linear acceleration gains use `1 / sec^2` and `1 / sec`. |
| Joint effort, armature, friction and motor envelope | `include/solver/PxSolverDefs.h`, `PxPerformanceEnvelope` and `PxJointFrictionParams`; `PxArticulationJointReducedCoordinate.h`; `BaseArticulationView.cpp` property switch | Effort is force or torque; armature is mass or inertia. Viscous friction/resistance is effort per coordinate rate, and speed-effort gradient is its reciprocal. Degree conversion follows the property's explicit policy. |
| Inverse dynamics | `include/PxArticulationReducedCoordinate.h`: `computeDenseJacobian`, `computeMassMatrix`, `computeCoriolisCompensation`, `computeGravityCompensation`, `computeCentroidalMomentumMatrix`; `OvxPhysicsRead.cpp` applies signs but no angular conversion. | Jacobian: row velocity / column velocity. Mass matrix: row effort / column acceleration. Coriolis/gravity: force or torque. The centroidal matrix maps velocity to momentum; its final bias column is force/torque, not momentum. Mechanical mass/inertia block units treat radians as dimensionless, while input angular rates remain `rad / sec`. |
| Fixed tendons | `include/PxArticulationTendon.h`; `omni.physics.parse/ParseFixedTendon.cpp::fillFixedTendonAxis` and `propertiesUpdate/PhysXTendonPropertiesUpdate.cpp` multiply revolute authored gearing by rad-to-deg. CPU/GPU property accessors pass SDK scalars through; joint effort also includes a force coefficient. | Unit authored-gearing angular tendons use `deg`, torque/deg and torque/(deg/sec), assuming unit force coefficient. Unit authored-gearing linear tendons use `distance`, `mass / sec^2`, `mass / sec`. Mixed axes and dimensionful coefficients determine their own common coordinate/effort units; a static descriptor cannot choose these for a live tendon. |
| Spatial tendons | `include/PxArticulationTendon.h`; CPU/GPU tendon property accessors pass SDK scalars through. Lengths are weighted attachment distances. | Offset: `distance`; stiffness/limit stiffness: `mass / sec^2`; damping: `mass / sec`. |
| Vehicle wheel rotation, lateral slip and forces | `include/vehicle/wheel/PxVehicleWheelStates.h`, `vehicle/tire/PxVehicleTireStates.h`; SDK `source/physxvehicle/src/tire/VhTireFunctions.cpp::computeLateralSlip` uses atan; `BaseVehicleView.cpp` reads the corresponding fields directly. | Angles/lateral slip: `rad`; rotation speed: `rad / sec`; longitudinal slip: `dimensionless`; forces and torques use their mechanical units. |
| Deformable material damping | SDK `source/gpusimulationcontroller/src/CUDA/FEMCloth.cu` multiplies surface elasticity/bending damping by dt. `softBodyGM.cu` uses `1 + elasticityDamping * invDt` for volumes. `OvxPhysicsRead.cpp` forwards the getters. | Surface elasticity/bending damping: `1 / sec`; volume elasticity damping: `sec`. These are inferred from the solver formulas, not from parsing dimensions. |
| Deformable bending stiffness and Young's modulus | SDK `PxgFEMCloth.cpp::updateFlexuralStiffnessPerTrianglePair` multiplies input bending stiffness by thickness cubed and a dimensionless geometric ratio. | Both input material stiffness quantities are pressure: `mass / (distance * sec^2)`. The derived flexural stiffness is not the API input. |
| Packed values | Runtime read/write layouts for wrench, ground plane, deformable kinematic target and incoming joint force | Explicit ordered component unit IDs distinguish force, torque, distance and dimensionless flags/normals. Quaternions and direction vectors are dimensionless. |

The bundled headers and runtime conversions establish the contract; use the
current SDK documentation in `physx/documentation/` for background on articulation
drives and tendon dynamics.
