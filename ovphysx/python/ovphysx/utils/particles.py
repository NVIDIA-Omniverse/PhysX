# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-7 AC-10 AC-12 AC-15

"""Particle system, particle set and PBD material authoring.

A particle simulation needs a ``PhysxParticleSystem`` prim, a set prim carrying
``PhysxParticleSetAPI`` that points back at it, and usually a PBD material. The
helpers here author all three, plus the optional feature APIs (anisotropy,
smoothing, isosurface, diffuse particles) and the sampling API.

Every schema attribute is an optional keyword that defaults to ``None`` and is
only authored when supplied, so an unset attribute keeps the schema fallback
rather than being pinned to a value chosen here.

The PhysX schemas are codeless, so these helpers apply APIs by identifier and
author properties by name; :func:`add_physx_particle_system` answers with the
``Usd.Prim`` it defined. Refer to :mod:`ovphysx.utils.codeless`.
"""

import logging
import typing

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from . import codeless
from .materials import ensure_material_on_path

logger = logging.getLogger(__name__)

__all__ = [
    "create_particles_grid",
    "add_physx_particle_system",
    "add_pbd_particle_material",
    "add_pbd_material_water",
    "add_pbd_material_viscous",
    "add_physx_particleset_points",
    "add_physx_particleset_pointinstancer",
    "configure_particle_set",
    "add_physx_particle_anisotropy",
    "add_physx_particle_smoothing",
    "add_physx_particle_isosurface",
    "add_physx_diffuse_particles",
    "poisson_sample_mesh",
]

# The concrete prim type of a particle system. Codeless, so it is defined by
# type name through Usd.Stage.DefinePrim rather than a typed Define().
PARTICLE_SYSTEM_TYPE_NAME = "PhysxParticleSystem"


def create_particles_grid(
    lower, particle_spacing, dim_x, dim_y, dim_z, uniform_particle_velocity=Gf.Vec3f(0.0)
) -> typing.Tuple[typing.List[Gf.Vec3f], typing.List[Gf.Vec3f]]:
    """Builds a regular grid of particle positions and a matching velocity list.

    Args:
        lower:            The grid's minimum corner.
        particle_spacing: The distance between adjacent particles.
        dim_x:            Particle count along X.
        dim_y:            Particle count along Y.
        dim_z:            Particle count along Z.
        uniform_particle_velocity: The velocity given to every particle.

    Returns:
        A (positions, velocities) tuple.
    """
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


def _get_default_particle_system_path(stage: Usd.Stage) -> Sdf.Path:
    """Returns the conventional particle system path, defining /World if the stage has no default prim.

    Private. Its one caller is :func:`poisson_sample_mesh`.

    Args:
        stage: The stage.
    """
    if not stage.GetDefaultPrim():
        default_prim_xform = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(default_prim_xform.GetPrim())
    return stage.GetDefaultPrim().GetPath().AppendChild("ParticleSystem")


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
) -> typing.Optional[Usd.Prim]:
    """Creates a PhysxParticleSystem prim at particle_system_path on stage.

    Args:
        stage:                                          The stage
        particle_system_path:                           Path where the system should be created at
        ... schema attributes:                          See USD schema for documentation

    An occupied path is rejected with a ``ValueError`` rather than the prim that
    is already there being retyped.

    The concrete codeless prim type must be registered before this function
    mutates the stage.

    Returns:
        The particle system ``Usd.Prim``, or None if the prim could not be
        defined.

    Raises:
        CodelessSchemaError: If ``PhysxParticleSystem`` is not registered.
    """
    if stage.GetPrimAtPath(particle_system_path):
        raise ValueError(f"{particle_system_path} is already held by a prim")

    codeless._require_concrete_prim_definition(PARTICLE_SYSTEM_TYPE_NAME)
    particle_system = stage.DefinePrim(particle_system_path, PARTICLE_SYSTEM_TYPE_NAME)
    if not particle_system:
        return None

    # A particle system is a concrete codeless prim type, so its properties are
    # not namespaced the way an API schema's are.
    codeless.set_attrs(
        particle_system,
        {
            "particleSystemEnabled": particle_system_enabled,
            "contactOffset": contact_offset,
            "restOffset": rest_offset,
            "particleContactOffset": particle_contact_offset,
            "solidRestOffset": solid_rest_offset,
            "fluidRestOffset": fluid_rest_offset,
            "enableCCD": enable_ccd,
            "solverPositionIterationCount": solver_position_iterations,
            "maxDepenetrationVelocity": max_depenetration_velocity,
            "wind": wind,
            "maxNeighborhood": max_neighborhood,
            "neighborhoodScale": neighborhood_scale,
            "maxVelocity": max_velocity,
            "globalSelfCollisionEnabled": global_self_collision_enabled,
            "nonParticleCollisionEnabled": non_particle_collision_enabled,
        },
    )
    if simulation_owner is not None:
        codeless.set_rel(particle_system, "simulationOwner", simulation_owner)

    return particle_system


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
    density=None,
    cfl_coefficient=None,
):
    """Applies the PhysxPBDMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """

    if not ensure_material_on_path(stage, path):
        return False

    prim = stage.GetPrimAtPath(path)
    codeless.apply_api(prim, "PhysxPBDMaterialAPI")
    codeless.set_attrs(
        prim,
        {
            "physxPBDMaterial:friction": friction,
            "physxPBDMaterial:particleFrictionScale": particle_friction_scale,
            "physxPBDMaterial:damping": damping,
            "physxPBDMaterial:viscosity": viscosity,
            "physxPBDMaterial:vorticityConfinement": vorticity_confinement,
            "physxPBDMaterial:surfaceTension": surface_tension,
            "physxPBDMaterial:cohesion": cohesion,
            "physxPBDMaterial:adhesion": adhesion,
            "physxPBDMaterial:particleAdhesionScale": particle_adhesion_scale,
            "physxPBDMaterial:adhesionOffsetScale": adhesion_offset_scale,
            "physxPBDMaterial:gravityScale": gravity_scale,
            "physxPBDMaterial:density": density,
            "physxPBDMaterial:cflCoefficient": cfl_coefficient,
        },
    )
    return True


def add_pbd_material_water(p):
    """Applies a water PBD material preset to a material prim.

    The surface tension and viscosity presets are in SI, so they are converted
    to the stage's units before being authored.

    Args:
        p: The UsdShade.Material prim.
    """
    mpu = UsdGeom.GetStageMetersPerUnit(p.GetStage())
    add_pbd_particle_material(
        stage=p.GetStage(),
        path=p.GetPath(),
        cohesion=0.01,
        damping=0,
        friction=0.1,
        surface_tension=0.0074 * mpu * mpu * mpu,
        viscosity=0.0000017 / (mpu * mpu * mpu),
        vorticity_confinement=0,
    )


def add_pbd_material_viscous(p):
    """Applies a viscous-fluid PBD material preset to a material prim.

    The surface tension and viscosity presets are in SI, so they are converted
    to the stage's units before being authored.

    Args:
        p: The UsdShade.Material prim.
    """
    mpu = UsdGeom.GetStageMetersPerUnit(p.GetStage())
    add_pbd_particle_material(
        stage=p.GetStage(),
        path=p.GetPath(),
        cohesion=0.2,
        damping=0,
        friction=0.2,
        surface_tension=0.1 * mpu * mpu * mpu,
        viscosity=0.0017 / (mpu * mpu * mpu),
        vorticity_confinement=0,
    )


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

    An occupied path is rejected with a ``ValueError`` rather than the prim that
    is already there being retyped.

    Returns:
        The UsdGeom.Points
    """
    if stage.GetPrimAtPath(path):
        raise ValueError(f"{path} is already held by a prim")
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
    path: typing.Union[str, Sdf.Path],
    positions,
    velocities,
    particle_system_path,
    self_collision,
    fluid,
    particle_group,
    particle_mass,
    density,
    num_prototypes: int = 1,
    prototype_indices: typing.Optional[list] = None,
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

    An occupied path is rejected with a ``ValueError`` rather than the prim that
    is already there being retyped.

    Returns:
        The created UsdGeom.PointInstancer prim
    """
    instancerPath = Sdf.Path(path)
    prototype_base_path = instancerPath.pathString + "/particlePrototype"

    if stage.GetPrimAtPath(instancerPath):
        raise ValueError(f"{instancerPath} is already held by a prim")
    instancer = UsdGeom.PointInstancer.Define(stage, instancerPath)

    mesh_list = instancer.GetPrototypesRel()

    for i in range(num_prototypes):
        prototype_path = prototype_base_path + str(i)
        UsdGeom.Sphere.Define(stage, Sdf.Path(prototype_path))
        mesh_list.AddTarget(Sdf.Path(prototype_path))

    if prototype_indices is None:
        prototype_indices = [0] * len(positions)
    proto_indices = list(prototype_indices)

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
    particle_set_prim, particle_system_path, self_collision, fluid, particle_group, mass=0.0, density=0.0
):
    """Applies the particle set and mass APIs that turn a points or instancer prim into particles.

    Args:
        particle_set_prim:  The UsdGeom.Points or UsdGeom.PointInstancer prim.
        particle_system_path: Path to the particle system that simulates the set.
        self_collision:     Enable particle-particle collision in the set.
        fluid:              Simulate the particle set as fluid.
        particle_group:     The particle group, see schema API doc.
        mass:               The total mass of the set.
        density:            The density used when no mass is provided.
    """
    # PhysxParticleSetAPI declares PhysxParticleAPI as a built-in, so applying
    # the set API brings in the physxParticle:* properties of both.
    codeless.apply_api(particle_set_prim, "PhysxParticleSetAPI")
    codeless.set_attrs(
        particle_set_prim,
        {
            "physxParticle:selfCollision": self_collision,
            "physxParticle:fluid": fluid,
            "physxParticle:particleGroup": particle_group,
        },
    )

    if particle_system_path is not None:
        codeless.set_rel(particle_set_prim, "physxParticle:particleSystem", particle_system_path)

    # Non zero value will take precedence in the parser
    massApi = UsdPhysics.MassAPI.Apply(particle_set_prim)
    massApi.CreateMassAttr(mass)
    massApi.CreateDensityAttr(density)


def add_physx_particle_anisotropy(
    stage,
    path,
    enabled=None,
    scale=None,
    min=None,
    max=None,
):
    """Applies the PhysxParticleAnisotropyAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the anisotropy API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    prim = stage.GetPrimAtPath(path)
    codeless.apply_api(prim, "PhysxParticleAnisotropyAPI")
    codeless.set_attrs(
        prim,
        {
            "physxParticleAnisotropy:particleAnisotropyEnabled": enabled,
            "physxParticleAnisotropy:scale": scale,
            "physxParticleAnisotropy:min": min,
            "physxParticleAnisotropy:max": max,
        },
    )
    return True


def add_physx_particle_smoothing(
    stage,
    path,
    enabled=None,
    strength=None,
):
    """Applies the PhysxParticleSmoothingAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the smoothing API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    prim = stage.GetPrimAtPath(path)
    codeless.apply_api(prim, "PhysxParticleSmoothingAPI")
    codeless.set_attrs(
        prim,
        {
            "physxParticleSmoothing:particleSmoothingEnabled": enabled,
            "physxParticleSmoothing:strength": strength,
        },
    )
    return True


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
    """Applies the PhysxParticleIsosurfaceAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the isosurface API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    prim = stage.GetPrimAtPath(path)
    codeless.apply_api(prim, "PhysxParticleIsosurfaceAPI")
    codeless.set_attrs(
        prim,
        {
            "physxParticleIsosurface:isosurfaceEnabled": enabled,
            "physxParticleIsosurface:maxVertices": max_vertices,
            "physxParticleIsosurface:maxTriangles": max_triangles,
            "physxParticleIsosurface:maxSubgrids": max_subgrids,
            "physxParticleIsosurface:gridSpacing": grid_spacing,
            "physxParticleIsosurface:surfaceDistance": surface_distance,
            "physxParticleIsosurface:gridFilteringPasses": grid_filtering_passes,
            "physxParticleIsosurface:gridSmoothingRadius": grid_smoothing_radius,
            "physxParticleIsosurface:numMeshSmoothingPasses": num_mesh_smoothing_passes,
            "physxParticleIsosurface:numMeshNormalSmoothingPasses": num_mesh_normal_smoothing_passes,
        },
    )
    return True


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
    """Applies the PhysxDiffuseParticlesAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to the prim to which the diffuse particle API should be applied to
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    prim = stage.GetPrimAtPath(path)
    codeless.apply_api(prim, "PhysxDiffuseParticlesAPI")
    codeless.set_attrs(
        prim,
        {
            "physxDiffuseParticles:diffuseParticlesEnabled": enabled,
            "physxDiffuseParticles:maxDiffuseParticleMultiplier": max_diffuse_particle_multiplier,
            "physxDiffuseParticles:threshold": threshold,
            "physxDiffuseParticles:lifetime": lifetime,
            "physxDiffuseParticles:airDrag": air_drag,
            "physxDiffuseParticles:bubbleDrag": bubble_drag,
            "physxDiffuseParticles:buoyancy": buoyancy,
            "physxDiffuseParticles:kineticEnergyWeight": kinetic_energy_weight,
            "physxDiffuseParticles:pressureWeight": pressure_weight,
            "physxDiffuseParticles:divergenceWeight": divergence_weight,
            "physxDiffuseParticles:collisionDecay": collision_decay,
            "physxDiffuseParticles:useAccurateVelocity": use_accurate_velocity,
        },
    )
    return True


def poisson_sample_mesh(stage: Usd.Stage, prim_path: Sdf.Path):
    """Marks a mesh for volume particle sampling, creating a particle system if the stage has none.

    Only authors the sampling request; the runtime performs the sampling when
    the stage is simulated.

    The stage's first particle system is reused wherever it sits. Creating one
    defines ``/World`` and makes it the stage default prim when the stage has
    none.

    Args:
        stage:     The stage.
        prim_path: Path to the mesh to sample.

    Returns:
        The path of the particle system that will sample the mesh, or an empty
        ``Sdf.Path`` if no particle system could be obtained.

        A stage that has no particle system and whose default particle system
        path is already held by a prim of another type gets the empty path too,
        rather than that prim being written over.
    """
    prim = stage.GetPrimAtPath(prim_path)

    particle_system_path = Sdf.Path()
    for candidate in stage.Traverse():
        if candidate.GetTypeName() == PARTICLE_SYSTEM_TYPE_NAME:
            particle_system_path = candidate.GetPath()
            break

    if particle_system_path == Sdf.Path():
        particle_system_path = _get_default_particle_system_path(stage)
        if particle_system_path == Sdf.Path():
            logger.error("Failed to create default particle system.")
            return Sdf.Path()
        occupant = stage.GetPrimAtPath(particle_system_path)
        if occupant:
            logger.error(
                f"Failed to create default particle system: {particle_system_path} is already "
                f"held by a {occupant.GetTypeName()} prim."
            )
            return Sdf.Path()
        add_physx_particle_system(stage, particle_system_path)

    particle_system_prim = stage.GetPrimAtPath(particle_system_path)
    if not particle_system_prim or particle_system_prim.GetTypeName() != PARTICLE_SYSTEM_TYPE_NAME:
        return Sdf.Path()

    codeless.apply_api(prim, "PhysxParticleSamplingAPI")
    codeless.set_attr(prim, "physxParticleSampling:volume", True)
    return particle_system_path
