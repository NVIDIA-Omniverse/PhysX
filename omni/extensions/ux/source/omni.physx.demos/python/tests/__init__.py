# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import omni.physxdemos as demo
import carb.settings

local_mode = carb.settings.get_settings().get("/exts/omni.physx.tests/localMode")

ASSET_DEMOS = [
    "FrankaNutBoltDemo",                # showroom
    "NutsAndBoltsDemo",                 # showroom
    "AnalogDigitalClockDemo",           # showroom
    "FrankaDeformableDemo",
    "ChairStackingDemo",
    "FrankaBrickStackDemo",
    "HoneyDemo",
    "ClothDeckChairDemo",
    # "FluidIsosurfaceGlassBoxDemo",    # Crashing OM-116136
    "MixerDemo",
    "TeddyOnIceDemo",
    "LegoTechnicBuggyRigid",
]

EXCLUDE_FROM_ALL = [
    "TriangleMeshMultiMaterialDemo",    # timeouting
    "DeformableHandDemo",               # timeouts
    "ParticleSamplerDemo",              # causes sporadic GPU crashes
    "RigidBodyCCDDemo",                 # intermittent ERROR_DEVICE_LOST on Linux TC, not reproable locally ..
    "ForceDemo",                        # failing on omni.graph.core dep?
    "MuseumDemo",                       # currently disabled due to issue OM-84698, perhaps enable once fixed
    "ParticlePostProcessingDemo",       # OM-115172
    "FluidIsosurfaceGlassBoxDemo",      # Crashing OM-116136
    "InverseDynamicsTensorAPIDemo",     # OMPE-31498 issue with tensor API when run from here
    "JointBreakDemo",                   # Fails in fabric, NVBug 5116935
]

EXCLUDE_FROM_VISUAL = [
    "OverlapAny",                       # Render seems to be different each time
    "OverlapMeshDemo",                  # debug vis sometimes renders, sometimes not  
    "OverlapMultipleDemo",              # debug vis sometimes renders, sometimes not  
    "TriggerDemo",                      # debug vis sometimes renders, sometimes not
    "RaycastsDemo",                     # overlay is offset on linux
    "SweepsDemo",                       # overlay is offset on linux
    "DeformableBodyAttachmentsDemo",    # OM-45168
    "ParticleInflatableDemo",           # needs better camera angle (demo cam not set for some reason)
    "ParticleClothDemo",                # needs better camera angle (demo cam not set for some reason)
    "ParticleSamplerDemo",              # needs better camera angle (demo cam not set for some reason)
    "ParticlePostProcessingDemo",       # needs better camera angle (demo cam not set for some reason)
    "OverlapShapeDemo",                 # unstable overlay on VP2
    "KaplaArenaDemo",                   # Until MR-37007 (point instancer fix) is merged to Kit
    "RigidBodyRopeDemo",                # Until MR-37007 (point instancer fix) is merged to Kit
]

# running outside of our repo, test one small demo to test access to S3/nucleus
if not local_mode:
    EXCLUDE_FROM_ALL.extend(ASSET_DEMOS)
    ASSET_DEMOS = ["MixerDemo"]


class DirectImportTestDemo(demo.Base):
    category = "TestDemo"


class PhysxDemoBase(TestCase):
    category = TestCategory.Core
    thresholds = {}

    async def _visual_base(self, exclude_list):
        async def do_prepare(scene_class):
            print(scene_class.__module__)
            await self.setup_viewport_test()

        async def do_test(scene_class):
            await self.wait(60)
            await self.step(40, precise=True)
            demo_name = f"{scene_class.__module__.split('.')[-1]}"
            await self.do_visual_test(
                img_name="",
                img_suffix=demo_name,
                threshold=PhysxDemoBase.thresholds.get(demo_name, 0.0005),
            )

        await demo.test("omni.physxdemos", do_prepare, do_test, exclude_list, test_case=self, inspect_offset=2)

    async def _base(self, exclude_list):
        async def do_prepare(scene_class):
            print(scene_class.__module__)

        async def do_steps(scene_class):
            await self.wait(60)
            await self.step(20)

        await demo.test("omni.physxdemos", do_prepare, do_steps, exclude_list, test_case=self, inspect_offset=2)


class PhysxBaseDemosTest(PhysxDemoBase):
    category = TestCategory.Core

    async def test_physics_demos(self):
        exclude_list = EXCLUDE_FROM_ALL + ASSET_DEMOS
        await self._base(exclude_list)

    async def test_physics_visual_demos(self):
        exclude_list = EXCLUDE_FROM_ALL + EXCLUDE_FROM_VISUAL + ASSET_DEMOS
        await self._visual_base(exclude_list)

    async def test_physics_demos_direct_import(self):
        success = False
        async def do_steps(scene_class):
            print(scene_class.__name__)
            nonlocal success
            success = scene_class.__name__ == DirectImportTestDemo.__name__
            await self.wait(60)

        demo.register("omni.physxdemos.tests")
        await demo.test("omni.physxdemos.tests", None, do_steps)
        demo.unregister("omni.physxdemos.tests")
        self.assertTrue(success)


class PhysXVehicleTestDemos(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_vehicle_demos(self):
        async def do_steps(scene_class):
            print(scene_class.__module__)
            await self.wait(60)
            await self.step(20)

        await demo.test("omni.physxvehicle", None, do_steps)


class PhysxXAssetDemosTest(PhysxDemoBase):
    async def test_physics_asset_demos(self):
        async def do_steps(scene_class):
            print(scene_class.__module__)
            await self.wait(60)
            await self.wait(240)

        assets_path = carb.settings.get_settings().get("physics/demoAssetsPath")
        print(f"Demo assets path is: {assets_path}")

        demos = ASSET_DEMOS
        for name in demos:
            with self.subTest(name) as include:
                if include:
                    await demo.test_demo(f"omni.physxdemos.scenes.{name}", None, do_steps)
