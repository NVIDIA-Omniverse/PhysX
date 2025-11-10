# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physxdemos as demo
import omni.usd
import omni.kit
try:
    from omni.physxtestsvisual.utils import TestCase

    class PhysxXGraphDemosTest(TestCase):
        thresholds = {"SceneQueryRaycastDemo": 0.01}  # this is how you can customize tresholds for specific demo
        load_timeout = {"SomeDemo": 10000}
        use_camera_light = {"SceneQueryRaycastDemo": False}
        async def _visual_base(self, exclude_list):
            async def do_prepare(scene_class):
                print(scene_class.__module__)
                await self.setup_viewport_test()
               
            async def do_test(scene_class):
                demo_name: str = f"{scene_class.__module__.split('.')[-1]}"

                usd_context=omni.usd.get_context()

                timeout_counter = PhysxXGraphDemosTest.load_timeout.get(demo_name, 10000)
                while timeout_counter > 0:
                    _, files_loaded, total_files = usd_context.get_stage_loading_status()
                    if files_loaded or total_files:
                        await self.wait(1)
                        timeout_counter -= 1
                    else:
                        break
                self.assertGreater(timeout_counter, 0)

                await self.wait(1)
                await self.step(120)
                await self.wait(60)
                await self.do_visual_test(
                    img_name="",
                    img_suffix=demo_name,
                    use_distant_light=PhysxXGraphDemosTest.use_camera_light.get(demo_name, True),
                    threshold=PhysxXGraphDemosTest.thresholds.get(demo_name, 0.0015),
                )
                await omni.usd.get_context().new_stage_async()
                await omni.kit.app.get_app().next_update_async()
            await demo.test("omni.physxgraph", do_prepare, do_test, exclude_list, test_case=self, inspect_offset=2)

        async def test_physics_graph_demos(self):
            await self._visual_base([])
except Exception:
    pass
