# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
from omni.physxcct import get_physx_cct_interface
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from omni.physxcct.scripts import utils
import omni.physx
import omni.kit.ui_test as ui_test
import omni.ui as ui
from carb.input import KeyboardInput
import omni.timeline

DEFAULT_CAMERA_PATH = "/OmniverseKit_Persp"


class PhysXCctBaseTests(PhysicsKitStageAsyncTestCase):
    async def test_cct_movement(self):
        stage = await self.new_stage()
        path = "/Capsule"
        base_pos = Gf.Vec3f(0, 0, 50)
        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule.AddTranslateOp().Set(base_pos)
        capsule_cctapi = PhysxSchema.PhysxCharacterControllerAPI.Apply(capsule.GetPrim())

        physxcct = get_physx_cct_interface()
        physxcct.enable_first_person(path, DEFAULT_CAMERA_PATH)
        physxcct.disable_gravity(path)

        # fake input
        capsule_cctapi.GetMoveTargetAttr().Set(Gf.Vec3f(1.0))

        # step and check if moved
        await self.step(10)
        self.assertTrue(capsule.GetPrim().GetAttribute("xformOp:translate").Get() != base_pos)

    async def test_cct_velocity(self):
        stage = await self.new_stage()
        path = "/Capsule"
        base_pos = Gf.Vec3f(0, 0, 0)

        capsule = utils.spawn_capsule(stage, path, base_pos)
        cctPhysxAPI = PhysxSchema.PhysxCharacterControllerAPI.Apply(capsule.GetPrim())
        cctPhysxAPI.GetUpAxisAttr().Set(UsdGeom.GetStageUpAxis(stage))

        physxcct = get_physx_cct_interface()
        physxcct.disable_first_person(path)
        physxcct.disable_gravity(path)

        await self.wait(10)

        for ttc in [24, 30, 50, 60]:
            with self.subTest(str(ttc)):
                timecodes = ttc
                stage.SetTimeCodesPerSecond(timecodes)
                print(f"# timecodes per second {timecodes}")
                await self.step(10)
                pos = capsule.GetPrim().GetAttribute("xformOp:translate").Get()
                u = 0
                t = 0

                def on_stage_update(_, dt):
                    nonlocal t, u, pos
                    if u == ttc:
                        tran = capsule.GetPrim().GetAttribute("xformOp:translate").Get() - pos
                        print(f"Sent {t:.4f} offset over {t:.4f}s in {u} stage updates, real offset is {tran[0]:.4f}")
                        self.assertAlmostEqual(tran[0], t, delta=0.05)
                        get_physx_cct_interface().set_move(path, (0, 0, 0))
                    elif u < ttc:
                        t += dt
                        get_physx_cct_interface().set_move(path, (0, dt, 0))
                    u += 1

                stage_update_node = utils.register_stage_update_node(
                    "cct_test",
                    on_update_fn=on_stage_update
                )
                await self.frame(100)
                stage_update_node = None

    async def test_cct_utils_and_action_handling(self):
        stage = await self.new_stage()
        path = "/Capsule"
        base_pos = Gf.Vec3f(0, 0, 50)
        utils.spawn_capsule(stage, path, base_pos)

        capsule_prim = stage.GetPrimAtPath(path)
        self.assertTrue(capsule_prim.IsValid())
        self.assertTrue(capsule_prim.GetAttribute("xformOp:translate").Get() == base_pos)

        cct = utils.CharacterController(path, DEFAULT_CAMERA_PATH, False)
        cct.activate(stage)
        cct.setup_controls(500)

        # OMPE-14540 regression check, this will run updates before simulation is played
        await self.wait(5)

        # fake input by emulating keyboard for some frames creating actions to handle
        base_pos = capsule_prim.GetAttribute("xformOp:translate").Get()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(0, 5):
            await self.wait(1)
            await ui_test.emulate_keyboard_press(KeyboardInput.W)
        timeline.stop()

        print(base_pos)
        print(capsule_prim.GetAttribute("xformOp:translate").Get())

        self.assertTrue(capsule_prim.GetAttribute("xformOp:translate").Get() != base_pos)

    async def _run_cct_reset_transform_test(self, deleteAttributeDuringSim=False):
        stage = await self.new_stage()

        path = "/Capsule"
        init_translation = Gf.Vec3f(0, 0, 50)
        utils.spawn_capsule(stage, path, init_translation)

        xform = UsdGeom.Xform.Get(stage, path)
        xform.ClearXformOpOrder()
        init_rotx = 5.0
        xform.SetResetXformStack(True)
        xform.AddTranslateOp().Set(init_translation)
        xform.AddRotateXOp().Set(init_rotx)

        cct = utils.CharacterController(path, DEFAULT_CAMERA_PATH, False)
        cct.activate(stage)
        cct.setup_controls(500)

        # fake input
        cct.control_state.inputs[utils.ControlAction.FORWARD] = 1

        # step and check if moved
        await self.step(10)
        self.assertTrue(xform.GetPrim().GetAttribute("xformOp:translate").Get() != init_translation)

        sanitized_xform_ops = xform.GetOrderedXformOps()
        self.assertEqual(len(sanitized_xform_ops), 3)
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        for op, opName in zip(sanitized_xform_ops, opNames):
            self.assertEqual(op.GetName(), opName)
        self.assertTrue(xform.GetResetXformStack())

        if deleteAttributeDuringSim:
            xform.GetPrim().RemoveProperty('xformOp:rotateX')

        # reset and check that original ops order is restored and ops attributes as well
        omni.physx.get_physx_interface().reset_simulation()

        reset_xform_ops = xform.GetOrderedXformOps()
        self.assertEqual(len(reset_xform_ops), 2)
        self.assertTrue(xform.GetResetXformStack())

        self.assertEqual(reset_xform_ops[0].GetOpType(), UsdGeom.XformOp.TypeTranslate)
        self.assertEqual(reset_xform_ops[0].Get(), init_translation)

        self.assertEqual(reset_xform_ops[1].GetOpType(), UsdGeom.XformOp.TypeRotateX)
        self.assertEqual(reset_xform_ops[1].Get(), init_rotx)

    async def test_cct_reset_transform(self):
        await self._run_cct_reset_transform_test()

    async def test_cct_reset_transform_delete_xformop(self):
        await self._run_cct_reset_transform_test(deleteAttributeDuringSim=True)

    async def test_cct_ui(self):
        window_name = "PhysX Character Controller"

        await self.new_stage()

        window = ui.Workspace.get_window(window_name)
        if window is None:
            await ui_test.menu_click("Window/Physics/Character Controller")
        else:
            window.visible = True

        activate_button = ui_test.find(f"{window_name}//Frame/**/Button[*].text=='Activate'")
        disable_button = ui_test.find(f"{window_name}//Frame/**/Button[*].text=='Disable'")

        await activate_button.click()

        await self.wait(60)

        await disable_button.click()
