# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils, particleUtils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
import omni.timeline
import numpy as np


class FluidBallEmitterDemo(demo.AsyncDemoBase):
    title = "Paint Ball Emitter"
    category = demo.Categories.PARTICLES
    short_description = "PBD particles paint ball emitter"
    description = "Paint-like PBD fluid balls emitted onto a canvas."

    params = {
        "Single_Particle_Set": demo.CheckboxParam(True),
        "Use_Instancer": demo.CheckboxParam(True),
    }

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 60,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: True,
        "rtx/post/aa/op": 3,
        "rtx/post/dlss/execMode": 0,
    }

    def __init__(self):
        super().__init__(enable_fabric=False, fabric_compatible=False)
        self._time = 0
        self._is_running = False
        self._is_paused = False
        self._rng_seed = 42
        self._rng = np.random.default_rng(self._rng_seed)
        self._ball_spawn_interval = 0.4
        self._next_ball_time = self._ball_spawn_interval
        self._num_balls = 0
        self._num_balls_to_spawn = 25
        self._num_colors = 20
        self._isActive = True
        self._stage = None

    def particle_sphere(self, radius, particleSpacing):
        points = []
        dim = math.ceil(2 * radius / particleSpacing)
        for i in range(dim):
            for j in range(dim):
                for k in range(dim):
                    x = i * particleSpacing - radius + self._rng.uniform(-0.05, 0.05)
                    y = j * particleSpacing - radius + self._rng.uniform(-0.05, 0.05)
                    z = k * particleSpacing - radius + self._rng.uniform(-0.05, 0.05)

                    d2 = x * x + y * y + z * z
                    if d2 < radius * radius:
                        points.append(Gf.Vec3f(x, y, z))

        return points

    def create_colors(self):

        fractions = np.linspace(0.0, 1.0, self._num_colors)
        colors = []

        for frac in fractions:
            colors.append(self.create_color(frac))

        return colors

    def create_color(self, frac):

        # HSL to RGB conversion
        hue = frac
        saturation = 1.0
        luminosity = 0.5

        hue6 = hue * 6.0
        modulo = Gf.Vec3f((hue6 + 0.0) % 6.0, (hue6 + 4.0) % 6.0, (hue6 + 2.0) % 6.0)
        absolute = Gf.Vec3f(abs(modulo[0] - 3.0), abs(modulo[1] - 3.0), abs(modulo[2] - 3.0))
        rgb = Gf.Vec3f(
            Gf.Clampf(absolute[0] - 1.0, 0.0, 1.0),
            Gf.Clampf(absolute[1] - 1.0, 0.0, 1.0),
            Gf.Clampf(absolute[2] - 1.0, 0.0, 1.0),
        )

        linter = Gf.Vec3f(1.0) * (1.0 - saturation) + rgb * saturation
        rgb = luminosity * linter

        return rgb

    @staticmethod
    def extend_array_attribute(attribute, elements):
        attributeArray = attribute.Get()
        if not attributeArray is None:
            array_elements = list(attributeArray)
            array_elements.extend(elements)
            attribute.Set(array_elements)
        else:
            attribute.Set(elements)

    def add_shared_particles(self, positions_list, velocities_list, colorIndex):
        particleSet = PhysxSchema.PhysxParticleSetAPI(self._sharedParticlePrim)
        pointInstancer = UsdGeom.PointInstancer(self._sharedParticlePrim)
        points = UsdGeom.Points(self._sharedParticlePrim)

        #update sim positions first, create attribute as needed.
        #this blocks physics update when changing gfx positions
        if self._useSmoothing:
            simPointsAttr = particleSet.GetSimulationPointsAttr()
            if not simPointsAttr.HasAuthoredValue():
                simPointsAttr.Set(Vt.Vec3fArray([]))
            self.extend_array_attribute(simPointsAttr, positions_list)

        if pointInstancer:
            self.extend_array_attribute(pointInstancer.GetPositionsAttr(), positions_list)
            self.extend_array_attribute(pointInstancer.GetVelocitiesAttr(), velocities_list)
            self.extend_array_attribute(pointInstancer.GetProtoIndicesAttr(), [colorIndex]*len(positions_list))
            self.extend_array_attribute(pointInstancer.GetOrientationsAttr(), [Gf.Quath(1.0, 0.0, 0.0, 0.0)]*len(positions_list))
            self.extend_array_attribute(pointInstancer.GetScalesAttr(), [Gf.Vec3f(1.0)]*len(positions_list))
        elif points:
            self.extend_array_attribute(points.GetPointsAttr(), positions_list)
            self.extend_array_attribute(points.GetVelocitiesAttr(), velocities_list)
            self.extend_array_attribute(points.GetWidthsAttr(), [2*self._fluid_rest_offset]*len(positions_list))
            primVars = points.GetDisplayColorPrimvar()
            primVarsIndicesAttr = primVars.GetIndicesAttr()
            #indices = list(primVarsIndicesAttr.Get())
            self.extend_array_attribute(primVarsIndicesAttr, [colorIndex]*len(positions_list))

    def add_particles(self, positions_list, velocities_list, colorIndex):
        color = Vt.Vec3fArray([self._colors[colorIndex]])
        particlePointsPath = Sdf.Path("/particles" + str(self._num_balls))

        if self._usePointInstancer:
            particlePrim = particleUtils.add_physx_particleset_pointinstancer(
                self._stage,
                particlePointsPath,
                positions_list,
                velocities_list,
                self._particleSystemPath,
                self_collision=True,
                fluid=True,
                particle_group=0,
                particle_mass=0.001,
                density=0.0,
                num_prototypes=0
            )

            prototypeStr = str(particlePointsPath) + "/particlePrototype0"
            gprim = UsdGeom.Sphere.Define(self._stage, Sdf.Path(prototypeStr))
            gprim.CreateDisplayColorAttr(color)
            gprim.CreateRadiusAttr().Set(self._fluid_rest_offset)
            UsdGeom.PointInstancer(particlePrim).GetPrototypesRel().AddTarget(Sdf.Path(prototypeStr))

        else:
            particlePrim = particleUtils.add_physx_particleset_points(
                self._stage,
                particlePointsPath,
                positions_list,
                velocities_list,
                [2*self._fluid_rest_offset]*len(positions_list),
                self._particleSystemPath,
                self_collision=True,
                fluid=True,
                particle_group=0,
                particle_mass=0.001,
                density=0.0
            )
            particlePrim.CreateDisplayColorAttr(color)


    def create_shared_particle_prim(self, stage):
        if not self._useSharedParticleSet or not self._sharedParticlePrim is None:
            return

        particlePointsPath = Sdf.Path("/particles")

        if self._usePointInstancer:

            self._sharedParticlePrim = particleUtils.add_physx_particleset_pointinstancer(
                stage,
                particlePointsPath,
                [],
                [],
                self._particleSystemPath,
                self_collision=True,
                fluid=True,
                particle_group=0,
                particle_mass=0.001,
                density=0.0,
                num_prototypes=0,
            )

            #create prototypes for all colors and store paths
            for i, c in enumerate(self._colors):
                color = Vt.Vec3fArray([c])
                prototypeStr = str(particlePointsPath) + "/particlePrototype" + str(i)
                gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(prototypeStr))
                gprim.CreateDisplayColorAttr(color)
                gprim.CreateRadiusAttr().Set(self._fluid_rest_offset)
                UsdGeom.PointInstancer(self._sharedParticlePrim).GetPrototypesRel().AddTarget(Sdf.Path(prototypeStr))
                #add a dummy particles for each prototype, otherwise hydra complains
                self.add_shared_particles([Gf.Vec3f(-4.0, -1.0 + i*0.2, -0.5)], [Gf.Vec3f(0.0, 0.0, 0.0)], i)

        else:
            self._sharedParticlePrim = particleUtils.add_physx_particleset_points(
                stage,
                particlePointsPath,
                [],
                [],
                [],
                self._particleSystemPath,
                self_collision=True,
                fluid=True,
                particle_group=0,
                particle_mass=0.001,
                density=0.0,
            )

            #unfortunately display color primvar with "vertex" interpolation doesn't seem to work on UsdGeomPoints yet.
            self._sharedParticlePrim.CreateDisplayColorAttr().Set(self._colors)
            self._sharedParticlePrim.CreateDisplayColorPrimvar(interpolation="vertex")
            self._sharedParticlePrim.GetDisplayColorPrimvar().CreateIndicesAttr().Set([])

        maxParticles = self._num_balls_to_spawn*len(self._ball)
        self._sharedParticlePrim.GetPrim().CreateAttribute("physxParticle:maxParticles", Sdf.ValueTypeNames.Int).Set(maxParticles)


    def create_ball(self, stage, pos):
        basePos = Gf.Vec3f(11.0, 12.0, 7.0) + pos
        colorIndex = self._num_balls % self._num_colors

        positions_list = [x + basePos for x in self._ball]
        velocities_list = [Gf.Vec3f(10, 10, 0.0)] * len(positions_list)

        if self._useSharedParticleSet:
            self.create_shared_particle_prim(stage)
            self.add_shared_particles(positions_list, velocities_list, colorIndex)

        else:
            self.add_particles(positions_list, velocities_list, colorIndex)


    def create(self, stage, Single_Particle_Set, Use_Instancer):
        self._stage = stage
        self._setup_callbacks()

        self._useSharedParticleSet = Single_Particle_Set
        self._usePointInstancer = Use_Instancer
        self._useSmoothing = True
        self._useAnisotropy = True

        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, metersPerUnit = 1.0)
        scenePath = defaultPrimPath + "/physicsScene"

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")
        self._particleSystemPath = particleSystemPath

        particleSpacing = 0.18
        restOffset = particleSpacing * 0.9
        solidRestOffset = restOffset
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = max(solidRestOffset + 0.005, fluidRestOffset / 0.6)
        contactOffset = restOffset + 0.005

        self._fluid_rest_offset = fluidRestOffset

        particle_system = particleUtils.add_physx_particle_system(
            stage,
            particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=4,
            simulation_owner=scenePath,
            max_neighborhood=96
        )

        if self._useSmoothing:
            particleUtils.add_physx_particle_smoothing(
                stage,
                particleSystemPath,
                strength=1.0
            )

        if self._usePointInstancer and self._useAnisotropy:
            particleUtils.add_physx_particle_anisotropy(
                stage,
                particleSystemPath,
                scale=1.0,
            )

        # Create a pbd particle material and set it on the particle system
        pbd_particle_material_path = omni.usd.get_stage_next_free_path(stage, "/pbdParticleMaterial", True)
        particleUtils.add_pbd_particle_material(
            stage,
            pbd_particle_material_path,
            cohesion=5,
            viscosity=1000,
            surface_tension=0.02,
            friction=1000,
            damping=0.99,
        )
        physicsUtils.add_physics_material_to_prim(stage, particle_system.GetPrim(), pbd_particle_material_path)

        room = demo.get_demo_room(self, stage)

        # create particle posititions for a ball filled with particles
        self._ball = self.particle_sphere(1.0, fluidRestOffset * 2)
        self._colors = self.create_colors()

        self._sharedParticlePrim = None
        self._session_sub_layer = None

    def create_session_layer(self):
        #create an anonymous session_sub_layer for application of all runtime changes
        rootLayer = self._stage.GetRootLayer()
        self._session_sub_layer = Sdf.Layer.CreateAnonymous()
        self._stage.GetSessionLayer().subLayerPaths.append(self._session_sub_layer.identifier)
        self._stage.SetEditTarget(Usd.EditTarget(self._session_sub_layer))


    def release_session_layer(self):
        #remove session_sub_layer to clear out runtime changes
        self._stage.GetSessionLayer().subLayerPaths.remove(self._session_sub_layer.identifier)
        self._stage.SetEditTarget(self._stage.GetRootLayer())
        self._session_sub_layer = None

    def backup_camera(self):
        cameraPrim = self._stage.GetPrimAtPath("/World/Camera")
        self._camera_trans = cameraPrim.GetAttribute("xformOp:translate").Get()
        self._camera_scale = cameraPrim.GetAttribute("xformOp:scale").Get()
        self._camera_rotate = cameraPrim.GetAttribute("xformOp:rotateYXZ").Get()

    def restore_camera(self):
        cameraPrim = self._stage.GetPrimAtPath("/World/Camera")
        cameraPrim.GetAttribute("xformOp:translate").Set(self._camera_trans)
        cameraPrim.GetAttribute("xformOp:scale").Set(self._camera_scale)
        cameraPrim.GetAttribute("xformOp:rotateYXZ").Set(self._camera_rotate)

    def on_timeline_event(self, e):
        if not self._isActive:
            return
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_running = False
            self._is_paused = False
            self._rng = np.random.default_rng(self._rng_seed)
            self._num_balls = 0
            self._time = 0
            self._next_ball_time = self._ball_spawn_interval
            self._sharedParticlePrim = None
            self.release_session_layer()
            self.restore_camera()

        if e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._is_running = False
            self._is_paused = True

        elif e.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._is_paused:
                self.backup_camera()
                self.create_session_layer()

            self._is_running = True
            self._is_paused = False


    def on_physics_step(self, dt):
        if not self._isActive:
            return
        self._time += dt

    def update(self, stage, dt, viewport, physxIFace):
        if not self._isActive:
            return

        if not self._is_running:
            return

        self.backup_camera()

        if self._num_balls >= self._num_balls_to_spawn:
            return

        if self._time < self._next_ball_time:
            return

        self._next_ball_time = self._time + self._ball_spawn_interval

        self._num_balls += 1
        x = self._rng.uniform(-22, -16)
        y = self._rng.uniform(-22, -16)
        pos = Gf.Vec3f(x, y, 0.0)

        self.create_ball(self._stage, pos)

    def on_shutdown(self):
        self._isActive = False
        self._stage = None
        super().on_shutdown()
