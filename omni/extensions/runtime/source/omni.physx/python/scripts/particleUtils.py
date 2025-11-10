# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdGeom, Usd, Sdf, Vt, PhysxSchema, UsdPhysics, Gf
import carb
from . import utils
import typing


# common helpers
def create_particles_grid(
    lower, particle_spacing, dim_x, dim_y, dim_z, uniform_particle_velocity=Gf.Vec3f(0.0)
) -> typing.List[Gf.Vec3f]:
    x = lower[0]
    y = lower[1]
    z = lower[2]

    positions = [Gf.Vec3f(0.0)] * dim_x * dim_y * dim_z
    index = 0
    for i in range(dim_x):
        for j in range(dim_y):
            for k in range(dim_z):
                positions[index] = Gf.Vec3f(x, y, z)
                index += 1
                z = z + particle_spacing
            z = lower[2]
            y = y + particle_spacing
        y = lower[1]
        x = x + particle_spacing
    return (positions, [uniform_particle_velocity] * len(positions))


def get_default_particle_system_path(stage: Usd.Stage) -> Sdf.Path:
    if not stage.GetDefaultPrim():
        default_prim_xform = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(default_prim_xform.GetPrim())
    return stage.GetDefaultPrim().GetPath().AppendChild("ParticleSystem")


def get_default_particle_system(stage: Usd.Stage) -> PhysxSchema.PhysxParticleSystem:
    for prim in stage.Traverse():
        particle_system = PhysxSchema.PhysxParticleSystem(prim)
        if particle_system:
            return particle_system

    particle_system_path = get_default_particle_system_path(stage)
    if particle_system_path != Sdf.Path():
        add_physx_particle_system(stage, particle_system_path)
        prim = stage.GetPrimAtPath(particle_system_path)
        return PhysxSchema.PhysxParticleSystem(prim)
    else:
        carb.log_error("Failed to create default particle system.")
        return None

# Particle system ######################################################################################################
def add_physx_particle_system(
    stage,
    particle_system_path,
    particle_system_enabled=None,
    simulation_owner=None,
    contact_offset=None,
    rest_offset=None,
    particle_contact_offset=None,
    solid_rest_offset=None,
    fluid_rest_offset=None,
    enable_ccd=None,
    solver_position_iterations=None,
    max_depenetration_velocity=None,
    wind=None,
    max_neighborhood=None,
    neighborhood_scale=None,
    max_velocity=None,
    global_self_collision_enabled=None,
    non_particle_collision_enabled=None,
) -> PhysxSchema.PhysxParticleSystem:
    """Creates a PhysxSchema.PhysxParticleSystem at particle_system_path on stage.

    Args:
        stage:                                          The stage
        particle_system_path:                           Path where the system should be created at
        ... schema attributes:                          See USD schema for documentation

    Returns:
        The PhysxSchema.PhysxParticleSystem, or None if there was an issue with defining the system.
    """
    assert not stage.GetPrimAtPath(particle_system_path)

    particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, particle_system_path)
    if not particle_system:
        return None

    if particle_system_enabled is not None:
        particle_system.CreateParticleSystemEnabledAttr().Set(particle_system_enabled)
    if simulation_owner is not None:
        particle_system.CreateSimulationOwnerRel().SetTargets([simulation_owner])
    if contact_offset is not None:
        particle_system.CreateContactOffsetAttr().Set(contact_offset)
    if rest_offset is not None:
        particle_system.CreateRestOffsetAttr().Set(rest_offset)
    if particle_contact_offset is not None:
        particle_system.CreateParticleContactOffsetAttr().Set(particle_contact_offset)
    if solid_rest_offset is not None:
        particle_system.CreateSolidRestOffsetAttr().Set(solid_rest_offset)
    if fluid_rest_offset is not None:
        particle_system.CreateFluidRestOffsetAttr().Set(fluid_rest_offset)
    if enable_ccd is not None:
        particle_system.CreateEnableCCDAttr().Set(enable_ccd)
    if solver_position_iterations is not None:
        particle_system.CreateSolverPositionIterationCountAttr().Set(solver_position_iterations)
    if max_depenetration_velocity is not None:
        particle_system.CreateMaxDepenetrationVelocityAttr().Set(max_depenetration_velocity)
    if wind is not None:
        particle_system.CreateWindAttr().Set(wind)
    if max_neighborhood is not None:
        particle_system.CreateMaxNeighborhoodAttr().Set(max_neighborhood)
    if neighborhood_scale is not None:
        particle_system.CreateNeighborhoodScaleAttr().Set(neighborhood_scale)
    if max_velocity is not None:
        particle_system.CreateMaxVelocityAttr().Set(max_velocity)
    if global_self_collision_enabled is not None:
        particle_system.CreateGlobalSelfCollisionEnabledAttr().Set(global_self_collision_enabled)
    if non_particle_collision_enabled is not None:
        particle_system.CreateNonParticleCollisionEnabledAttr().Set(non_particle_collision_enabled)

    return particle_system


# PBD material #########################################################################################################
def add_pbd_particle_material(
    stage,
    path,
    friction=None,
    particle_friction_scale=None,
    damping=None,
    viscosity=None,
    vorticity_confinement=None,
    surface_tension=None,
    cohesion=None,
    adhesion=None,
    particle_adhesion_scale=None,
    adhesion_offset_scale=None,
    gravity_scale=None,
    lift=None,
    drag=None,
    density=None,
    cfl_coefficient=None,
):
    """Deprecated: lift and drag are deprecated and will be removed.
    Applies the PhysxSchema.PhysxPBDMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """

    if not utils.ensureMaterialOnPath(stage, path):
        return False

    material = PhysxSchema.PhysxPBDMaterialAPI.Apply(stage.GetPrimAtPath(path))

    if friction is not None:
        material.CreateFrictionAttr().Set(friction)
    if particle_friction_scale is not None:
        material.CreateParticleFrictionScaleAttr().Set(particle_friction_scale)
    if damping is not None:
        material.CreateDampingAttr().Set(damping)
    if viscosity is not None:
        material.CreateViscosityAttr().Set(viscosity)
    if vorticity_confinement is not None:
        material.CreateVorticityConfinementAttr().Set(vorticity_confinement)
    if surface_tension is not None:
        material.CreateSurfaceTensionAttr().Set(surface_tension)
    if cohesion is not None:
        material.CreateCohesionAttr().Set(cohesion)
    if adhesion is not None:
        material.CreateAdhesionAttr().Set(adhesion)
    if particle_adhesion_scale is not None:
        material.CreateParticleAdhesionScaleAttr().Set(particle_adhesion_scale)
    if adhesion_offset_scale is not None:
        material.CreateAdhesionOffsetScaleAttr().Set(adhesion_offset_scale)
    if gravity_scale is not None:
        material.CreateGravityScaleAttr().Set(gravity_scale)
    if lift is not None:
        material.CreateLiftAttr().Set(lift)
    if drag is not None:
        material.CreateDragAttr().Set(drag)
    if density is not None:
        material.CreateDensityAttr().Set(density)
    if cfl_coefficient is not None:
        material.CreateCflCoefficientAttr().Set(cfl_coefficient)
    return True

def AddPBDMaterialWater(p):
    mpu = UsdGeom.GetStageMetersPerUnit(p.GetStage())
    add_pbd_particle_material(stage=p.GetStage(), path=p.GetPath(), cohesion=0.01, drag=0, lift=0, damping=0, friction=0.1, surface_tension=0.0074*mpu*mpu*mpu, viscosity=0.0000017/(mpu*mpu*mpu), vorticity_confinement=0)    

def AddPBDMaterialViscous(p):
    mpu = UsdGeom.GetStageMetersPerUnit(p.GetStage())
    add_pbd_particle_material(stage=p.GetStage(), path=p.GetPath(), cohesion=0.2, drag=0.1, lift=0.5, damping=0, friction=0.2, surface_tension=0.1*mpu*mpu*mpu, viscosity=0.0017/(mpu*mpu*mpu), vorticity_confinement=0)

# Particle Sets ########################################################################################################
def add_physx_particleset_points(
    stage,
    path,
    positions_list,
    velocities_list,
    widths_list,
    particle_system_path,
    self_collision,
    fluid,
    particle_group,
    particle_mass,
    density,
) -> UsdGeom.Points:
    """Creates a particle set based on a UsdGeom.Points at path on stage.

    Args:
        stage:                          The stage
        path:                           Path where the UsdGeom.Points particle set should be created
        positions_list:                 List of particle positions
        velocities_list:                List of particle velocities
        widths_list:                    List of particle widths
        particle_system_path:           Path to particle system that simulates the set
        self_collision:                 Enable particle-particle collision in the set
        fluid:                          Simulate the particle set as fluid
        particle_group:                 The particle group, see schema API doc
        particle_mass:                  The per-particle mass - total mass of set is num particles * particle_mass
        density:                        The density of the particles - is used to compute particle (set) mass if no mass provided

    Returns:
        The UsdGeom.Points
    """
    assert not stage.GetPrimAtPath(path)
    particlePointsPath = Sdf.Path(path)
    particles = UsdGeom.Points.Define(stage, particlePointsPath)

    positions = Vt.Vec3fArray(positions_list)
    velocities = Vt.Vec3fArray(velocities_list)
    widths = Vt.FloatArray(widths_list)

    particles.GetPointsAttr().Set(positions)
    particles.GetVelocitiesAttr().Set(velocities)
    particles.GetWidthsAttr().Set(widths)

    configure_particle_set(
        particles.GetPrim(),
        particle_system_path,
        self_collision,
        fluid,
        particle_group,
        particle_mass * len(positions_list),
        density,
    )

    return particles


def add_physx_particleset_pointinstancer(
    stage,
    path,
    positions,
    velocities,
    particle_system_path,
    self_collision,
    fluid,
    particle_group,
    particle_mass,
    density,
    num_prototypes: int = 1,
    prototype_indices: list = None,
) -> Usd.Prim:
    """Creates a particle set based on a UsdGeom.PointInstancer at path on stage.

    Args:
        stage:                          The stage
        path:                           Path where the UsdGeom.PointInstancer particle set should be created
        positions:                      List of particle positions
        velocities:                     List of particle velocities
        particle_system_path:           Path to particle system that simulates the set
        self_collision:                 Enable particle-particle collision in the set
        fluid:                          Simulate the particle set as fluid
        particle_group:                  The particle group, see schema API doc
        particle_mass:                   The per-particle mass - total mass of set is num particles * particle_mass
        density:                        The density of the particles - is used to compute particle (set) mass if no mass provided
        num_prototypes:                 The number of render prototypes to create (children of point instancer)
        prototype_indices:              The prototype indices for the particles (same length as positions).
                                        Will default to 0 for all if not provided.

    Returns:
        The created UsdGeom.PointInstancer prim
    """
    prototype_base_path = path.pathString + "/particlePrototype"

    # Create point instancer
    assert not stage.GetPrimAtPath(path)
    instancer = UsdGeom.PointInstancer.Define(stage, path)

    mesh_list = instancer.GetPrototypesRel()

    # Create particle instance prototypes
    for i in range(num_prototypes):
        prototype_path = prototype_base_path + str(i)
        UsdGeom.Sphere.Define(stage, Sdf.Path(prototype_path))
        # add mesh references to point instancer
        mesh_list.AddTarget(Sdf.Path(prototype_path))

    # Set particle instance data
    if prototype_indices is None:
        prototype_indices = [0] * len(positions)
    proto_indices = [x for x in prototype_indices]

    instancer.GetProtoIndicesAttr().Set(proto_indices)
    instancer.GetPositionsAttr().Set(positions)
    instancer.GetVelocitiesAttr().Set(velocities)

    configure_particle_set(
        instancer.GetPrim(),
        particle_system_path,
        self_collision,
        fluid,
        particle_group,
        particle_mass * len(positions),
        density,
    )

    return instancer.GetPrim()


def configure_particle_set(
    particleSetPrim, particleSystemPath, self_collision, fluid, particle_group, mass=0.0, density=0.0
):
    particleSetApi = PhysxSchema.PhysxParticleSetAPI.Apply(particleSetPrim)
    particleApi = PhysxSchema.PhysxParticleAPI(particleSetApi)
    particleApi.CreateSelfCollisionAttr().Set(self_collision)
    particleSetApi.CreateFluidAttr().Set(fluid)
    particleApi.CreateParticleGroupAttr().Set(particle_group)

    if particleSystemPath is not None:
        particleApi.CreateParticleSystemRel().SetTargets([particleSystemPath])

    # Non zero value will take precedence in the parser
    massApi = UsdPhysics.MassAPI.Apply(particleSetPrim)
    massApi.CreateMassAttr(mass)
    massApi.CreateDensityAttr(density)



# Particle Anisotropy ##########################################################################################################
def add_physx_particle_anisotropy(
    stage,
    path,
    enabled=None,
    scale=None,
    min=None,
    max=None,
):
    """Applies the PhysxSchema.PhysxParticleAnisotropyAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the anisotropy API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    anisotropyApi = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(stage.GetPrimAtPath(path))
    if enabled is not None:
        anisotropyApi.CreateParticleAnisotropyEnabledAttr().Set(enabled)
    if scale is not None:
        anisotropyApi.CreateScaleAttr().Set(scale)
    if min is not None:
        anisotropyApi.CreateMinAttr().Set(min)
    if max is not None:
        anisotropyApi.CreateMaxAttr().Set(max)
    return True



# Particle Smoothing ##########################################################################################################
def add_physx_particle_smoothing(
    stage,
    path,
    enabled=None,
    strength=None,
):
    """Applies the PhysxSchema.PhysxParticleSmoothingAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the smoothing API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    smoothingApi = PhysxSchema.PhysxParticleSmoothingAPI.Apply(stage.GetPrimAtPath(path))
    if enabled is not None:
        smoothingApi.CreateParticleSmoothingEnabledAttr().Set(enabled)
    if strength is not None:
        smoothingApi.CreateStrengthAttr().Set(strength)
    return True



# Iso Surface ##########################################################################################################
def add_physx_particle_isosurface(
    stage,
    path,
    enabled=None,
    max_vertices=None,
    max_triangles=None,
    max_subgrids=None,
    grid_spacing=None,
    surface_distance=None,
    grid_filtering_passes=None,
    grid_smoothing_radius=None,
    num_mesh_smoothing_passes=None,
    num_mesh_normal_smoothing_passes=None,
):
    """Applies the PhysxSchema.PhysxParticleIsosurfaceAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the isosurface API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    isosurface = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(stage.GetPrimAtPath(path))
    if enabled is not None:
        isosurface.CreateIsosurfaceEnabledAttr().Set(enabled)
    if max_vertices is not None:
        isosurface.CreateMaxVerticesAttr().Set(max_vertices)
    if max_triangles is not None:
        isosurface.CreateMaxTrianglesAttr().Set(max_triangles)
    if max_subgrids is not None:
        isosurface.CreateMaxSubgridsAttr().Set(max_subgrids)
    if grid_spacing is not None:
        isosurface.CreateGridSpacingAttr().Set(grid_spacing)
    if surface_distance is not None:
        isosurface.CreateSurfaceDistanceAttr().Set(surface_distance)
    if grid_filtering_passes is not None:
        isosurface.CreateGridFilteringPassesAttr().Set(grid_filtering_passes)
    if grid_smoothing_radius is not None:
        isosurface.CreateGridSmoothingRadiusAttr().Set(grid_smoothing_radius)
    if num_mesh_smoothing_passes is not None:
        isosurface.CreateNumMeshSmoothingPassesAttr().Set(num_mesh_smoothing_passes)
    if num_mesh_normal_smoothing_passes is not None:
        isosurface.CreateNumMeshNormalSmoothingPassesAttr().Set(num_mesh_normal_smoothing_passes)
    return True


# Diffuse Particles ####################################################################################################
def add_physx_diffuse_particles(
    stage,
    path,
    enabled=None,
    max_diffuse_particle_multiplier=None,
    threshold=None,
    lifetime=None,
    air_drag=None,
    bubble_drag=None,
    buoyancy=None,
    kinetic_energy_weight=None,
    pressure_weight=None,
    divergence_weight=None,
    collision_decay=None,
    use_accurate_velocity=None,
):
    """Applies the PhysxSchema.PhysxDiffuseParticlesAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the diffuse particle API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    diffuseParticles = PhysxSchema.PhysxDiffuseParticlesAPI.Apply(stage.GetPrimAtPath(path))
    if enabled is not None:
        diffuseParticles.CreateDiffuseParticlesEnabledAttr().Set(enabled)
    if max_diffuse_particle_multiplier is not None:
        diffuseParticles.CreateMaxDiffuseParticleMultiplierAttr().Set(max_diffuse_particle_multiplier)
    if threshold is not None:
        diffuseParticles.CreateThresholdAttr().Set(threshold)
    if lifetime is not None:
        diffuseParticles.CreateLifetimeAttr().Set(lifetime)
    if air_drag is not None:
        diffuseParticles.CreateAirDragAttr().Set(air_drag)
    if bubble_drag is not None:
        diffuseParticles.CreateBubbleDragAttr().Set(bubble_drag)
    if buoyancy is not None:
        diffuseParticles.CreateBuoyancyAttr().Set(buoyancy)
    if kinetic_energy_weight is not None:
        diffuseParticles.CreateKineticEnergyWeightAttr().Set(kinetic_energy_weight)
    if pressure_weight is not None:
        diffuseParticles.CreatePressureWeightAttr().Set(pressure_weight)
    if divergence_weight is not None:
        diffuseParticles.CreateDivergenceWeightAttr().Set(divergence_weight)
    if collision_decay is not None:
        diffuseParticles.CreateCollisionDecayAttr().Set(collision_decay)
    if use_accurate_velocity is not None:
        diffuseParticles.CreateUseAccurateVelocityAttr().Set(use_accurate_velocity)
    return True


# PBD Cloth ############################################################################################################
def add_physx_particle_cloth(
    stage,
    path,
    dynamic_mesh_path,
    particle_system_path,
    spring_stretch_stiffness=None,
    spring_bend_stiffness=None,
    spring_shear_stiffness=None,
    spring_damping=None,
    self_collision=None,
    self_collision_filter=None,
    particle_group=None,
    pressure=None,
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()

    if dynamic_mesh_path:
        prim.GetReferences().AddReference(dynamic_mesh_path)

    cloth_api = PhysxSchema.PhysxParticleClothAPI.Apply(prim)
    cloth_auto_api = PhysxSchema.PhysxAutoParticleClothAPI.Apply(prim)
    particle_api = PhysxSchema.PhysxParticleAPI(cloth_api)

    if self_collision is not None:
        particle_api.CreateSelfCollisionAttr().Set(self_collision)
    if self_collision_filter is not None:
        cloth_api.CreateSelfCollisionFilterAttr().Set(self_collision_filter)
    if particle_group is not None:
        particle_api.CreateParticleGroupAttr().Set(particle_group)

    if pressure is not None:
        cloth_api.CreatePressureAttr().Set(pressure)

    particle_api.CreateParticleSystemRel().SetTargets([particle_system_path])

    if spring_stretch_stiffness is not None:
        cloth_auto_api.CreateSpringStretchStiffnessAttr().Set(spring_stretch_stiffness)

    if spring_bend_stiffness is not None:
        cloth_auto_api.CreateSpringBendStiffnessAttr().Set(spring_bend_stiffness)

    if spring_shear_stiffness is not None:
        cloth_auto_api.CreateSpringShearStiffnessAttr().Set(spring_shear_stiffness)

    if spring_damping is not None:
        cloth_auto_api.CreateSpringDampingAttr().Set(spring_damping)


def add_physx_particle_cloth_with_constraints(
    stage,
    path,
    particle_system_path,
    positions,
    normals,
    rest_positions,
    velocities,
    triangle_indices,
    spring_indices,
    spring_stiffnesses,
    spring_dampings,
    spring_rest_lengths,
    self_collision=None,
    self_collision_filter=None,
    particle_group=None,
    pressure=None,
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release."""

    mesh = UsdGeom.Mesh.Define(stage, path)
    prim = mesh.GetPrim()

    mesh.CreateDoubleSidedAttr().Set(True)

    # Triangle array's vertex count per face is always 3
    vertex_count = 3
    array_size = int(len(triangle_indices) / 3)
    index_array = Vt.IntArray(array_size, vertex_count)
    mesh.CreateFaceVertexCountsAttr().Set(index_array)
    mesh.CreateFaceVertexIndicesAttr().Set(triangle_indices)
    mesh.CreateNormalsAttr().Set(normals)
    mesh.CreatePointsAttr().Set(positions)
    mesh.CreateVelocitiesAttr().Set(velocities)

    cloth_api = PhysxSchema.PhysxParticleClothAPI.Apply(prim)
    particle_api = PhysxSchema.PhysxParticleAPI(cloth_api)
    cloth_api.GetRestPointsAttr().Set(rest_positions)
    cloth_api.CreateSpringIndicesAttr().Set(spring_indices)
    cloth_api.CreateSpringStiffnessesAttr().Set(spring_stiffnesses)
    cloth_api.CreateSpringDampingsAttr().Set(spring_dampings)
    cloth_api.CreateSpringRestLengthsAttr().Set(spring_rest_lengths)
    particle_api.CreateParticleSystemRel().SetTargets([particle_system_path])

    if self_collision is not None:
        particle_api.CreateSelfCollisionAttr().Set(self_collision)
    if self_collision_filter is not None:
        cloth_api.CreateSelfCollisionFilterAttr().Set(self_collision_filter)
    if particle_group is not None:
        particle_api.CreateParticleGroupAttr().Set(particle_group)

    if pressure is not None:
        cloth_api.CreatePressureAttr().Set(pressure)



def create_spring_grid(
    lower,
    dx,
    dy,
    radius,
    positions,
    normals,
    restPositions,
    velocities,
    triangleIndices,
    spring_connections,
    spring_stiffnesses,
    spring_dampings,
    spring_rest_lengths,
    stretch_stiffness,
    stretch_damping,
    bend_stiffness,
    bend_damping,
    shear_stiffness,
    shear_damping,
    use_attachment=False,
    attachment_positions=[],
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release."""

    def _grid_index(x, y, dx):
        return y * dx + x

    def _add_spring_constraint(
        spring_connections, i, j, spring_stiffnesses, stiffness, spring_dampings, damping, spring_rest_lengths, distance
    ):
        spring_connections.append(Gf.Vec2i(i, j))
        spring_stiffnesses.append(stiffness)
        spring_dampings.append(damping)
        spring_rest_lengths.append(distance.GetLength())

    for y in range(dy):
        for x in range(dx):
            pos_offset = Gf.Vec3f(radius * x, radius * y, radius * 1.0)
            positions.append(lower + pos_offset)
            normals.append(Gf.Vec3f(0.0, 0.0, 1.0))
            restPositions.append(lower + pos_offset)
            velocities.append(Gf.Vec3f(0.0, 0.0, 0.0))
            if use_attachment is True:
                if x == 0 or x == (dx - 1):
                    attachment_positions.append(lower + pos_offset)

            if x > 0 and y > 0:
                triangleIndices.append(_grid_index(x - 1, y - 1, dx))
                triangleIndices.append(_grid_index(x, y - 1, dx))
                triangleIndices.append(_grid_index(x, y, dx))
                triangleIndices.append(_grid_index(x - 1, y - 1, dx))
                triangleIndices.append(_grid_index(x, y, dx))
                triangleIndices.append(_grid_index(x - 1, y, dx))

    # horizontal
    for y in range(dy):
        for x in range(dx):
            i = y * dx + x

            if x > 0:
                j = y * dx + x - 1
                _add_spring_constraint(
                    spring_connections,
                    i,
                    j,
                    spring_stiffnesses,
                    stretch_stiffness,
                    spring_dampings,
                    stretch_damping,
                    spring_rest_lengths,
                    positions[i] - positions[j],
                )

            if x > 1 and bend_stiffness > 0.0:
                j = y * dx + x - 2
                _add_spring_constraint(
                    spring_connections,
                    i,
                    j,
                    spring_stiffnesses,
                    bend_stiffness,
                    spring_dampings,
                    bend_damping,
                    spring_rest_lengths,
                    positions[i] - positions[j],
                )

            if y > 0 and x < dx - 1 and shear_stiffness > 0.0:
                j = (y - 1) * dx + x + 1
                _add_spring_constraint(
                    spring_connections,
                    i,
                    j,
                    spring_stiffnesses,
                    shear_stiffness,
                    spring_dampings,
                    shear_damping,
                    spring_rest_lengths,
                    positions[i] - positions[j],
                )

            if y > 0 and x > 0 and shear_stiffness > 0.0:
                j = (y - 1) * dx + x - 1
                _add_spring_constraint(
                    spring_connections,
                    i,
                    j,
                    spring_stiffnesses,
                    shear_stiffness,
                    spring_dampings,
                    shear_damping,
                    spring_rest_lengths,
                    positions[i] - positions[j],
                )

    # vertical
    for x in range(dx):
        for y in range(dy):
            i = y * dx + x

            if y > 0:
                j = (y - 1) * dx + x
                _add_spring_constraint(
                    spring_connections,
                    i,
                    j,
                    spring_stiffnesses,
                    stretch_stiffness,
                    spring_dampings,
                    stretch_damping,
                    spring_rest_lengths,
                    positions[i] - positions[j],
                )

            if y > 1 and bend_stiffness > 0.0:
                j = (y - 2) * dx + x
                _add_spring_constraint(
                    spring_connections,
                    i,
                    j,
                    spring_stiffnesses,
                    bend_stiffness,
                    spring_dampings,
                    bend_damping,
                    spring_rest_lengths,
                    positions[i] - positions[j],
                )


# Poisson Sampling #####################################################################################################
def poisson_sample_mesh(stage: Usd.Stage, prim_path: Sdf.Path):

    prim = stage.GetPrimAtPath(prim_path)

    # get a particle system
    particle_system = get_default_particle_system(stage)
    particle_system_path = particle_system.GetPath()

    particle_system_prim = stage.GetPrimAtPath(particle_system_path)
    if not particle_system_prim or not PhysxSchema.PhysxParticleSystem(particle_system_prim):
        return Sdf.Path()

    samplingApi = PhysxSchema.PhysxParticleSamplingAPI.Apply(prim)
    samplingApi.CreateVolumeAttr().Set(True)
