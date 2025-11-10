# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import Usd, UsdLux, UsdGeom, Gf, UsdPhysics, PhysxSchema, Sdf
import omni.physxdemos as demo
import random
import omni.timeline
import carb.settings

AMBIENT_LIGHT_INTENSITY = "/rtx/sceneDb/ambientLightIntensity"
SAMPLED_LIGHTING = "/rtx/directLighting/sampledLighting/enabled"
SHOWLIGHTFLAG = 1 << 8
VIEWPORTDISPLAYOPTIONSPATH = "/persistent/app/viewport/displayOptions"


class RigidBodyCCDDemo(demo.Base):
    title = "RigidBody CCD"
    category = demo.Categories.RIGID_BODIES
    short_description = "Demo showing high velocity small spheres falling in a room"
    description = "Demo showing CCD (Continuous Collision Detection) usage. Press play (space) to run the simulation. Shift left click allows physics objects dragging during simulation."

    def create(self, stage, manual_animation=False):
        # spawn definitions
        self._manualAnimation = manual_animation
        self._numSpheres = 500
        self._currentSphere = 0
        self._spheresList = []
        self._spawnTime = 0.1
        self._spawnTimer = 0.0
        self._spawnRadius = 200.0
        self._timeline = omni.timeline.get_timeline_interface()
        self._lightingSet = False
        self._sampledLighting = False
        self._ambientLightIntensity = 1.0

        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        # setup CCD on a physics scene
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)

        # Spheres pool
        for i in range(self._numSpheres):
            sphereActorPath = defaultPrimPath + "/spheres/sphereActor" + str(i)

            sphereActor = UsdGeom.Xform.Define(stage, sphereActorPath)
            sphereActor.AddTranslateOp().Set(Gf.Vec3f(0.0))

            color = Gf.Vec3f(random.random(), random.random(), random.random())

            # collision
            radius = 2
            sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath + "/sphereCollision")
            sphereGeom.CreateRadiusAttr(radius)
            sphereGeom.CreateExtentAttr([(-radius, -radius, -radius), (radius, radius, radius)])
            sphereGeom.CreateDisplayColorAttr().Set([color])

            # light
            sphereLight = UsdLux.SphereLight.Define(stage, sphereActorPath + "/sphereLight")
            sphereLight.CreateRadiusAttr(radius * 1.1)
            sphereLight.CreateIntensityAttr(30000)
            sphereLight.CreateColorAttr(color)

            imageable = UsdGeom.Imageable(sphereActor)
            imageable.MakeInvisible()

            self._spheresList.append(sphereActor)

        room = demo.get_demo_room(self, stage, zoom = 0.3, enableDefaultLighting = False, staticDynamicRestitution = Gf.Vec3f(0.0, 0.0, 1.0), enableCollisionAudio=False)

    def on_startup(self):
        isregistry = carb.settings.acquire_settings_interface()
        display_options = carb.settings.get_settings().get_as_int(VIEWPORTDISPLAYOPTIONSPATH)
        if display_options & SHOWLIGHTFLAG:
            self._view_lights = True
        else:
            self._view_lights = False

        display_options = display_options & ~SHOWLIGHTFLAG
        isregistry.set_int(VIEWPORTDISPLAYOPTIONSPATH, display_options)

    def on_shutdown(self):
        isregistry = carb.settings.acquire_settings_interface()        
        isregistry.set_bool(SAMPLED_LIGHTING, self._sampledLighting)
        isregistry.set_float(AMBIENT_LIGHT_INTENSITY, self._ambientLightIntensity)


        display_options = carb.settings.get_settings().get_as_int(VIEWPORTDISPLAYOPTIONSPATH)
        if self._view_lights:
            display_options = display_options | SHOWLIGHTFLAG
        else:
            display_options = display_options & ~SHOWLIGHTFLAG
        isregistry.set_int(VIEWPORTDISPLAYOPTIONSPATH, display_options)

    def update(self, stage, dt, viewport, physxIFace):
        if not self._lightingSet:
            isregistry = carb.settings.acquire_settings_interface()
            self._sampledLighting = carb.settings.get_settings().get_as_bool(SAMPLED_LIGHTING)
            self._ambientLightIntensity = carb.settings.get_settings().get_as_float(AMBIENT_LIGHT_INTENSITY)
            isregistry.set_bool(SAMPLED_LIGHTING, True)
            isregistry.set_float(AMBIENT_LIGHT_INTENSITY, 0.01)
            self._lightingSet = True

        if (self._timeline.is_playing() or self._manualAnimation):
            self._spawnTimer = self._spawnTimer + dt
            if self._spawnTimer > self._spawnTime:
                self._spawnTimer = 0.0
                if self._currentSphere >= self._numSpheres - 1:
                    self._currentSphere = 0
                else:
                    self._currentSphere = self._currentSphere + 1

                sphereActor = self._spheresList[self._currentSphere]

                if sphereActor.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI):
                    rigidBodyAPI = UsdPhysics.RigidBodyAPI(sphereActor)
                else:
                    rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(sphereActor.GetPrim())

                    for prim in Usd.PrimRange(sphereActor.GetPrim()):
                        if UsdGeom.Sphere(prim):
                            UsdPhysics.CollisionAPI.Apply(prim)                            

                    imageable = UsdGeom.Imageable(sphereActor)
                    imageable.MakeVisible()

                    # enable CCD on the rigid body
                    physxRbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(sphereActor.GetPrim())
                    physxRbAPI.CreateEnableCCDAttr().Set(True)

                current_pos = Gf.Vec3f(0.0, 0.0, 100.0)
                random_pos = Gf.Vec3f(random.random()*2.0-1.0, random.random()*2.0-1.0, 0.0)
                random_pos.Normalize()
                random_pos = random_pos * (random.random() * self._spawnRadius)
                sphereActor.GetPrim().GetAttribute("xformOp:translate").Set(current_pos + random_pos)

                # set initial velocity                
                rigidBodyAPI.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, -400.0))                
