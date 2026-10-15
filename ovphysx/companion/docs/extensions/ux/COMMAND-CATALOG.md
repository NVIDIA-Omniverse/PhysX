<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Command Catalog

Names are inventoried from the inspected release source. Every row remains a **proposed host migration**, not an implemented adapter. These are AST-discovered command classes, not a claim that every class is exported under its class name. Runtime aliases, inherited signatures, edit-target behavior and return values must be qualified by REQ-UX-COMMANDS-001. Exact source anchors are in the internal source manifest.


| class | arguments | workflow_requirement |
| --- | --- | --- |
| `PhysXAddFollowLookCameraCommand` | `self, subjectPrimPath, cameraPrimPath` | [REQ-UX-CAMERA-001](requirements/features/REQ-UX-CAMERA-001.md) |
| `PhysXAddFollowVelocityCameraCommand` | `self, subjectPrimPath, cameraPrimPath` | [REQ-UX-CAMERA-001](requirements/features/REQ-UX-CAMERA-001.md) |
| `PhysXAddDroneCameraCommand` | `self, subjectPrimPath, cameraPrimPath` | [REQ-UX-CAMERA-001](requirements/features/REQ-UX-CAMERA-001.md) |
| `AddPhysicsSceneCommand` | `self, stage, path` | [REQ-UX-MENU-BASIC-001](requirements/features/REQ-UX-MENU-BASIC-001.md) |
| `AddRigidBodyMaterialCommand` | `self, stage, path, density=None, staticFriction=None, dynamicFriction=None, restitution=None` | [REQ-UX-MENU-MATERIAL-001](requirements/features/REQ-UX-MENU-MATERIAL-001.md) |
| `AddDeformableMaterialCommand` | `self, stage, path, density=None, static_friction=None, dynamic_friction=None, youngs_modulus=None, poissons_ratio=None` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `AddSurfaceDeformableMaterialCommand` | `self, stage, path, density=None, static_friction=None, dynamic_friction=None, youngs_modulus=None, poissons_ratio=None, surface_thickness=None, surface_stretch_stiffness=None, surface_shear_stiffness=None, surface_bend_stiffness=None` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `AddCollisionGroupCommand` | `self, stage, path` | [REQ-UX-COLLISION-FILTER-001](requirements/features/REQ-UX-COLLISION-FILTER-001.md) |
| `AddPairFilterCommand` | `self, stage, primPaths` | [REQ-UX-COLLISION-FILTER-001](requirements/features/REQ-UX-COLLISION-FILTER-001.md) |
| `RemovePairFilterCommand` | `self, stage, primPaths` | [REQ-UX-COLLISION-FILTER-001](requirements/features/REQ-UX-COLLISION-FILTER-001.md) |
| `AddGroundPlaneCommand` | `self, stage, planePath, axis, size, position, color` | [REQ-UX-MENU-BASIC-001](requirements/features/REQ-UX-MENU-BASIC-001.md) |
| `SetRigidBodyCommand` | `self, path, approximationShape=UsdPhysics.Tokens.convexHull, kinematic=False` | [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md) |
| `SetStaticColliderCommand` | `self, path, approximationShape='none'` | [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md) |
| `RemoveRigidBodyCommand` | `self, path` | [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md) |
| `RemoveStaticColliderCommand` | `self, path` | [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md) |
| `CreateJointCommand` | `self, stage, joint_type, from_prim, to_prim` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `CreateJointsCommand` | `self, stage, joint_type, paths, join_to_parent=False` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `SetVolumeDeformableBodyCommand` | `self, prim_path: Sdf.Path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `CreateAutoVolumeDeformableHierarchyCommand` | `self, root_prim_path: typing.Union[str, Sdf.Path], simulation_tetmesh_path: typing.Union[str, Sdf.Path], collision_tetmesh_path: typing.Union[str, Sdf.Path], cooking_src_mesh_path: typing.Union[str, Sdf.Path], simulation_hex_mesh_enabled: bool, cooking_src_simplification_enabled: bool` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `SetSurfaceDeformableBodyCommand` | `self, prim_path: Sdf.Path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `CreateAutoSurfaceDeformableHierarchyCommand` | `self, root_prim_path: typing.Union[str, Sdf.Path], simulation_mesh_path: typing.Union[str, Sdf.Path], cooking_src_mesh_path: typing.Union[str, Sdf.Path], cooking_src_simplification_enabled: bool` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveBaseDeformableBodyComponentCommand` | `self, prim_path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveVolumeDeformableSimComponentCommand` | `self, prim_path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveSurfaceDeformableSimComponentCommand` | `self, prim_path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveDeformablePoseComponentCommand` | `self, prim_path, instance_name` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveAutoDeformableBodyCommand` | `self, prim_path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveAutoDeformableHexahedralMeshCommand` | `self, prim_path: Sdf.Path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `AddAutoDeformableMeshSimplificationCommand` | `self, prim_path: Sdf.Path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `RemoveAutoDeformableMeshSimplificationCommand` | `self, prim_path: Sdf.Path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `CreateAutoDeformableAttachmentCommand` | `self, target_attachment_path: Sdf.Path, attachable0_path: Sdf.Path, attachable1_path: Sdf.Path` | [REQ-UX-DEFORM-HIERARCHY-001](requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md) |
| `ApplyAPISchemaCommand` | `self, api, prim, api_prefix=None, multiple_api_token=None` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `ApplyCodelessAPISchemaCommand` | `self, api, prim, api_prefix=None, multiple_api_token=None` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `UnapplyAPISchemaCommand` | `self, api, prim, api_prefix=None, multiple_api_token=None` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `UnapplyCodelessAPISchemaCommand` | `self, api, prim, api_prefix=None, multiple_api_token=None` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `RemoveAttributeCommand` | `self, attribute, prim` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `RemoveRelationshipCommand` | `self, relationship, prim` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `ChangeAttributeCommand` | `self, attr: Usd.Attribute, widget: Any, value: Any, prev: Any` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `AddDistancePhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemoveDistancePhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `AddFixedPhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemoveFixedPhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `AddRevolutePhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemoveRevolutePhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `AddPrismaticPhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemovePrismaticPhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `AddSphericalPhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemoveSphericalPhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemoveTendonComponentsCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-TENDON-WIDGET-001](requirements/features/REQ-UX-TENDON-WIDGET-001.md) |
| `AddD6PhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `RemoveD6PhysicsJointComponentCommand` | `self, usd_prim: Usd.Prim, component: str` | [REQ-UX-MENU-JOINT-001](requirements/features/REQ-UX-MENU-JOINT-001.md) |
| `AddPhysicsComponentCommand` | `self, usd_prim: Usd.Prim, component: str, multiple_api_token: str=None` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `RemovePhysicsComponentCommand` | `self, usd_prim: Usd.Prim, component: str, multiple_api_token: str=None` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `SetSpatialTendonAttachmentParentCommand` | `self, child_attachment_path: str, parent_attachment_path: str` | [REQ-UX-TENDON-WIDGET-001](requirements/features/REQ-UX-TENDON-WIDGET-001.md) |
| `RemoveSpatialTendonAttachmentAPICommand` | `self, attachment_path: str, attachment_api: str` | [REQ-UX-TENDON-WIDGET-001](requirements/features/REQ-UX-TENDON-WIDGET-001.md) |
| `AddParticleSystemCommand` | `self, target_particle_system_path: Sdf.Path=None, enabled: bool=None, simulation_owner: Sdf.Path=None, contact_offset: float=None, rest_offset: float=None, particle_contact_offset: float=None, solid_rest_offset: float=None, fluid_rest_offset: float=None, enable_ccd: bool=None, solver_position_iteration_count: int=None, max_depenetration_velocity: float=None, wind: Gf.Vec3f=None, max_neighborhood: int=None, max_velocity: float=None, global_self_collision_enabled: bool=None, non_particle_collision_enabled: bool=None` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddDiffuseParticlesCommand` | `self, stage, path, enabled=None, max_diffuse_particle_multiplier=None, threshold=None, lifetime=None, air_drag=None, bubble_drag=None, buoyancy=None, kinetic_energy_weight=None, pressure_weight=None, divergence_weight=None, collision_decay=None, use_accurate_velocity=None` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddParticleAnisotropyCommand` | `self, stage, path, enabled=None, scale=None, min=None, max=None` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddParticleSmoothingCommand` | `self, stage, path, enabled=None, strength=None` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddParticleIsosurfaceCommand` | `self, stage, path, enabled=None, max_vertices=None, max_triangles=None, max_subgrids=None, grid_spacing=None, surface_distance=None, grid_filtering_passes=None, grid_smoothing_radius=None, num_mesh_smoothing_passes=None, num_mesh_normal_smoothing_passes=None` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddParticleSamplingCommand` | `self, prim` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `RemoveParticleSamplingCommand` | `self, stage, prim` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddParticleSetCommand` | `self, prim` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `RemoveParticleSetCommand` | `self, stage, prim` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `AddPBDMaterialCommand` | `self, stage, path, friction=None, particle_friction_scale=None, damping=None, viscosity=None, vorticity_confinement=None, surface_tension=None, cohesion=None, adhesion=None, particle_adhesion_scale=None, adhesion_offset_scale=None, gravity_scale=None, lift=None, drag=None, density=None, cfl_coefficient=None` | [REQ-UX-PARTICLE-ACTIONS-001](requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md) |
| `SetCustomMetadataCommand` | `self, stage, metadata_name, new_values, old_values` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `CookPhysxColliders` | `self, **kwargs` | [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md) |
| `CreateCollidersCommand` | `self, stage, prim_paths` | [REQ-UX-COLLIDER-001](requirements/features/REQ-UX-COLLIDER-001.md) |
| `ClearPhysicsComponentsCommand` | `self, stage, prim_paths` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `ApplyMassAPICommand` | `self, prim, mass, center_of_mass, diagonal_inertia, principal_axes` | [REQ-UX-COMMANDS-001](requirements/features/REQ-UX-COMMANDS-001.md) |
| `PhysXVehicleSetRelationshipCommand` | `self, relationshipPath, targetPath` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleSetArrayEntryCommand` | `self, arrayAttributePath, entries` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleChangeArrayEntryCommand` | `self, arrayAttributePath, entryIndex, value, oldValue=None, fillValue=None, minFillSize=None` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleAddArrayEntryCommand` | `self, arrayAttributePath, entry` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleRemoveArrayEntryCommand` | `self, arrayAttributePath, entryIndex, oldValue=None` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleTireFrictionTableAddCommand` | `self, setPrimAsSelected: bool=True` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleTireFrictionTableAddEntryCommand` | `self, tireFrictionTablePath, materialPath, frictionValue` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleTireFrictionTableRemoveEntryCommand` | `self, tireFrictionTablePath, entryIndex` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleTireFrictionTableChangeEntryCommand` | `self, tireFrictionTablePath, entryIndex, frictionValue, oldFrictionValue=None` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleCommandResponseChangeEntryCommand` | `self, valueArrayPath, indexArrayPath, entryIndex, valueEntry, oldValueEntry, indexEntry, fillValue, maxIndexCount` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleDifferentialChangeEntryCommand` | `self, valueArrayPaths, valueArrayPathIndex, indexArrayPath, entryIndex, valueEntry, oldValueEntry, indexEntry` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleSuspensionFrameTransformsAutocomputeCommand` | `self, primPath` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleControllerEnableInputCommand` | `self, primPath, enabled` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleControllerEnableMouseCommand` | `self, primPath, enabled` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleControllerEnableAutoReverseCommand` | `self, primPath, enabled` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleControllerSetSteeringSensitivityCommand` | `self, primPath, sensitivity` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleControllerSetSteeringFilterTimeCommand` | `self, primPath, timeConstant` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |
| `PhysXVehicleWizardCreateCommand` | `self, vehicleData` | [REQ-UX-VEHICLE-001](requirements/features/REQ-UX-VEHICLE-001.md) |

`PhysicsCommand` is a shared implementation base, not a separate migrated user action. The catalog lists command classes; registration aliases and inherited signatures still require runtime checks in the Kit fixture.
