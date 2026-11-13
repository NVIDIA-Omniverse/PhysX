// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-POPULATION-CONTRACT-001
 * @covers AC-11
 */

// The KnownTokens names that are not population contract data, so gen_contract.py cannot generate
// them into KnownTokenNames.inl: values of token-valued columns (the contract names accepted values
// in prose only), USD metadata keys, a collection instance name, and names the runtime reads outside
// parsing. A prim type, API schema, instance or column the parser reads belongs in the contract
// instead (physics_population_domain/README.md); gen_contract.py rejects a name here that the
// contract declares. Define PHYSICS_KNOWN_TOKEN(member, name) before including and undefine it after.

#ifndef PHYSICS_KNOWN_TOKEN
#    error "Define PHYSICS_KNOWN_TOKEN(member, name) before including KnownTokenOtherNames.inl"
#endif

// physxMaterial:frictionCombineMode, restitutionCombineMode and dampingCombineMode values
PHYSICS_KNOWN_TOKEN(average, "average")
PHYSICS_KNOWN_TOKEN(min, "min")
PHYSICS_KNOWN_TOKEN(max, "max")
PHYSICS_KNOWN_TOKEN(multiply, "multiply")

// physxScene:updateType values
PHYSICS_KNOWN_TOKEN(sceneUpdateSynchronous, "Synchronous")
PHYSICS_KNOWN_TOKEN(sceneUpdateAsynchronous, "Asynchronous")
PHYSICS_KNOWN_TOKEN(sceneUpdateDisabled, "Disabled")

// physxScene:collisionSystem: only SAT is compared, PCM is the default
PHYSICS_KNOWN_TOKEN(collisionSystemSAT, "SAT")

// physxScene:solverType values
PHYSICS_KNOWN_TOKEN(solverTypeTGS, "TGS")
PHYSICS_KNOWN_TOKEN(solverTypePGS, "PGS")

// physxScene:broadphaseType values
PHYSICS_KNOWN_TOKEN(broadphaseTypeMBP, "MBP")
PHYSICS_KNOWN_TOKEN(broadphaseTypeSAP, "SAP")
PHYSICS_KNOWN_TOKEN(broadphaseTypeGPU, "GPU")

// physics:approximation values (PhysicsMeshCollisionAPI)
PHYSICS_KNOWN_TOKEN(approximationNone, "none")
PHYSICS_KNOWN_TOKEN(approximationConvexHull, "convexHull")
PHYSICS_KNOWN_TOKEN(approximationBoundingSphere, "boundingSphere")
PHYSICS_KNOWN_TOKEN(approximationBoundingCube, "boundingCube")
PHYSICS_KNOWN_TOKEN(approximationMeshSimplification, "meshSimplification")
PHYSICS_KNOWN_TOKEN(approximationConvexDecomposition, "convexDecomposition")
PHYSICS_KNOWN_TOKEN(approximationSphereFill, "sphereFill")
PHYSICS_KNOWN_TOKEN(approximationSdf, "sdf")

// physxSphereFillCollision:fillMode values
PHYSICS_KNOWN_TOKEN(fillModeFlood, "flood")
PHYSICS_KNOWN_TOKEN(fillModeRaycast, "raycast")
PHYSICS_KNOWN_TOKEN(fillModeSurface, "surface")

// physxSDFMeshCollision:sdfBitsPerSubgridPixel values
PHYSICS_KNOWN_TOKEN(bitsPerPixel8, "BitsPerPixel8")
PHYSICS_KNOWN_TOKEN(bitsPerPixel16, "BitsPerPixel16")
PHYSICS_KNOWN_TOKEN(bitsPerPixel32, "BitsPerPixel32")

// Axis values: physics:axis of the joints, axis of the analytic shapes, physxCharacterController:upAxis
PHYSICS_KNOWN_TOKEN(x, "X")
PHYSICS_KNOWN_TOKEN(y, "Y")
PHYSICS_KNOWN_TOKEN(z, "Z")

// drive:<inst>:physics:type and physxForce:mode value
PHYSICS_KNOWN_TOKEN(acceleration, "acceleration")

// physxCharacterController:climbingMode values
PHYSICS_KNOWN_TOKEN(easy, "easy")
PHYSICS_KNOWN_TOKEN(constrained, "constrained")

// physxCharacterController:nonWalkableMode values
PHYSICS_KNOWN_TOKEN(preventClimbing, "preventClimbing")
PHYSICS_KNOWN_TOKEN(preventClimbingForceSliding, "preventClimbingForceSliding")

// BasisCurves basis and wrap values (spline surface velocity curves)
PHYSICS_KNOWN_TOKEN(bezier, "bezier")
PHYSICS_KNOWN_TOKEN(catmullRom, "catmullRom")
PHYSICS_KNOWN_TOKEN(periodic, "periodic")
PHYSICS_KNOWN_TOKEN(nonperiodic, "nonperiodic")
PHYSICS_KNOWN_TOKEN(pinned, "pinned")

// omniphysics:restBendAnglesDefault value
PHYSICS_KNOWN_TOKEN(flatDefault, "flatDefault")

// USD metadata and customData keys; the contract's columns for them are usd-metadata:<key>
PHYSICS_KNOWN_TOKEN(apiSchemas, "apiSchemas")
PHYSICS_KNOWN_TOKEN(inactiveIds, "inactiveIds")
PHYSICS_KNOWN_TOKEN(metadataLocalSpaceVelocities, "physics:localSpaceVelocities")

// The CollectionAPI instance a PhysicsCollisionGroup names its members with; other collection
// instance names are arbitrary, so CollectionAPI declares no instances
PHYSICS_KNOWN_TOKEN(collidersCollectionName, "colliders")

// Read by scene queries, not by parsing: ovstage population never publishes visibility
PHYSICS_KNOWN_TOKEN(visibility, "visibility")
PHYSICS_KNOWN_TOKEN(invisible, "invisible")
