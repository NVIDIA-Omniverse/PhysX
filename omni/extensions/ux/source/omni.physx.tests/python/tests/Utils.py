# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from re import L
from pxr import Gf, UsdGeom
import omni.physx.scripts.physicsUtils as utils
import omni.physx.scripts.utils
import omni.usd
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from omni.physx.scripts.utils import ExpectMessage
from pxr import UsdPhysics
import omni.physx.bindings._physx as physx_bindings
import carb

class UtilsTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def _assert_transform_close(self, transform: Gf.Matrix4d, reference_transform: Gf.Matrix4d, places: int = 2):
        for va, vb in zip(transform, reference_transform):
            for a, b in zip(va, vb):
                self.assertAlmostEqual(a, b, places=places)

    def _assert_quat_close(self, quatA, quatB, places=4):
        self.assertAlmostEqual(quatA.real, quatB.real, places=4)
        for v, s in zip(quatA.imaginary, quatB.imaginary):
            self.assertAlmostEqual(v, s, places=4)

    def _assert_vec_close(self, vecA, vecB, places=4):
        for v, s in zip(vecA, vecB):
            self.assertAlmostEqual(v, s, places=4)

    def _get_xform_with_scale_rotxyz_translate(self):
        xform = self._getNewXform()
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(8, 1, 2))
        xform.AddRotateZYXOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(12, 34, 56))
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(2, 3, 4))
        return xform

    def _get_xform_with_scale_orient_translate(self):
        xform = self._getNewXform()
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 1, 0))
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3d(2, 2, 2))
        return xform

    def _getNewXform(self):
        return UsdGeom.Xform.Define(self._stage, omni.usd.get_stage_next_free_path(self._stage, "/World/xform", False))

    async def test_physics_utils_set_or_add_orient_op_to_xformable(self):
        await self.new_stage()
        # test setting with all possible precisions:
        precisions = [UsdGeom.XformOp.PrecisionDouble, UsdGeom.XformOp.PrecisionFloat, UsdGeom.XformOp.PrecisionHalf]
        initVals = [Gf.Quatd(1, 1, 1, 1), Gf.Quatf(1, 1, 1, 1), Gf.Quath(1, 1, 1, 1)]
        setVals = [Gf.Quatf(100, 0, -4, 1), Gf.Quath(100, 0, -4, 1), Gf.Quatd(100, 0, -4, 1)]
        for p, initVal, setVal in zip(precisions, initVals, setVals):
            xform = self._getNewXform()
            addOp = xform.AddOrientOp(p)
            addOp.Set(initVal)
            # test:
            op = utils.set_or_add_orient_op(xform, setVal)
            self.assertEqual(op, addOp)
            value = op.Get()
            self._assert_quat_close(setVal, value)
            # test creation:
            xformAdd = self._getNewXform()
            op = utils.set_or_add_orient_op(xformAdd, setVal)
            value = op.Get()
            self._assert_quat_close(setVal, value)

    async def test_physics_utils_set_or_add_scale_op_to_xformable(self):
        await self.new_stage()
        # test setting with all possible precisions:
        precisions = [UsdGeom.XformOp.PrecisionDouble, UsdGeom.XformOp.PrecisionFloat, UsdGeom.XformOp.PrecisionHalf]
        initVals = [Gf.Vec3d(1, 1, 1), Gf.Vec3f(1, 1, 1), Gf.Vec3h(1, 1, 1)]
        setVals = [Gf.Vec3f(100, 0, -4), Gf.Vec3h(100, 0, -4), Gf.Vec3d(100, 0, -4)]
        for p, initVal, setVal in zip(precisions, initVals, setVals):
            xform = self._getNewXform()
            addOp = xform.AddScaleOp(p)
            addOp.Set(initVal)
            # test:
            op = utils.set_or_add_scale_op(xform, setVal)
            self.assertEqual(op, addOp)
            value = op.Get()
            self._assert_vec_close(setVal, value, places=6)
            # test creation:
            xformAdd = self._getNewXform()
            op = utils.set_or_add_scale_op(xformAdd, setVal)
            value = op.Get()
            self._assert_vec_close(setVal, value, places=6)

    async def test_physics_utils_set_or_add_transform_op_to_xformable(self):
        await self.new_stage()
        # test setting with all possible precisions:
        precisions = [UsdGeom.XformOp.PrecisionDouble, UsdGeom.XformOp.PrecisionFloat, UsdGeom.XformOp.PrecisionHalf]
        initVals = [Gf.Vec3d(1, 1, 1), Gf.Vec3f(1, 1, 1), Gf.Vec3h(1, 1, 1)]
        setVals = [Gf.Vec3f(100, 0, -4), Gf.Vec3h(100, 0, -4), Gf.Vec3d(100, 0, -4)]
        for p, initVal, setVal in zip(precisions, initVals, setVals):
            xform = self._getNewXform()
            addOp = xform.AddTranslateOp(p)
            addOp.Set(initVal)
            # test:
            op = utils.set_or_add_translate_op(xform, setVal)
            self.assertEqual(op, addOp)
            value = op.Get()
            self._assert_vec_close(setVal, value, places=6)
            # test creation:
            xformAdd = self._getNewXform()
            op = utils.set_or_add_translate_op(xformAdd, setVal)
            value = op.Get()
            self._assert_vec_close(setVal, value, places=6)

    async def test_physics_utils_set_or_add_scale_orient_translate_ops_to_xformable(self):
        await self.new_stage()
        # test setting with all possible precisions:
        precisions = [UsdGeom.XformOp.PrecisionDouble, UsdGeom.XformOp.PrecisionFloat, UsdGeom.XformOp.PrecisionHalf]
        initVals = ((Gf.Vec3d(1, 1, 1), Gf.Quatd(1, 2, 3, 1), Gf.Vec3f(3, 4, 5)),
                    (Gf.Vec3f(1, 1, 1), Gf.Quatf(1, 2, 3, 1), Gf.Vec3f(3, 4, 5)),
                    (Gf.Vec3h(1, 1, 1), Gf.Quath(1, 2, 3, 1), Gf.Vec3h(3, 4, 5)))
        setVals = ((Gf.Vec3f(2, 3, 4), Gf.Quatf(5, 6, 7, 8), Gf.Vec3f(9, 10, 11)),
                   (Gf.Vec3h(2, 3, 4), Gf.Quath(5, 6, 7, 8), Gf.Vec3h(9, 10, 11)),
                   (Gf.Vec3d(2, 3, 4), Gf.Quatd(5, 6, 7, 8), Gf.Vec3d(9, 10, 11)))
        for p, initVal, setVal in zip(precisions, initVals, setVals):
            xform = self._getNewXform()
            addOps = []
            addOp = xform.AddTranslateOp(p)
            addOp.Set(initVal[0])
            addOps.append(addOp)
            addOp = xform.AddOrientOp(p)
            addOp.Set(initVal[1])
            addOps.append(addOp)
            addOp = xform.AddScaleOp(p)
            addOp.Set(initVal[2])
            addOps.append(addOp)
            # test:
            ops = utils.set_or_add_scale_orient_translate(xform, scale=setVal[2], orient=setVal[1], translate=setVal[0])
            for ao, o in zip(addOps, ops):
                self.assertEqual(ao, o)

            self._assert_vec_close(ops[0].Get(), setVal[0])
            self._assert_quat_close(ops[1].Get(), setVal[1])
            self._assert_vec_close(ops[2].Get(), setVal[2])
            # test creation:
            xformAdd = self._getNewXform()
            ops = utils.set_or_add_scale_orient_translate(xformAdd, scale=setVal[2], orient=setVal[1], translate=setVal[0])
            self._assert_vec_close(ops[0].Get(), setVal[0])
            self._assert_quat_close(ops[1].Get(), setVal[1])
            self._assert_vec_close(ops[2].Get(), setVal[2])

    async def test_physics_utils_setup_xform_as_scale_orient_translate(self):
        await self.new_stage()
        xformSrc = self._get_xform_with_scale_rotxyz_translate()
        xformSrc.SetResetXformStack(True)

        tfA = xformSrc.GetLocalTransformation()
        utils.setup_transform_as_scale_orient_translate(xformSrc)
        tfB = xformSrc.GetLocalTransformation()

        stack = xformSrc.GetOrderedXformOps()
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

        for op, opName in zip(stack, opNames):
            self.assertEqual(op.GetName(), opName)

        self._assert_transform_close(tfA, tfB, places=4)
        self.assertTrue(xformSrc.GetResetXformStack())

    async def test_internal_utils_xform_copy_as_scale_orient_translate(self):
        await self.new_stage()
        xformSrc = self._get_xform_with_scale_rotxyz_translate()
        xformSrc.SetResetXformStack(True)
        # setup some crap in dst:
        xformDst = self._getNewXform()
        xformDst.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(-1, 3, 1))
        xformDst.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(100, 0, -4))
        xformDst.AddRotateZYXOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(-43, 52, 100))

        utils.copy_transform_as_scale_orient_translate(xformSrc, xformDst)
        utils.copy_transform_as_scale_orient_translate(xformSrc, xformDst)

        tfA = xformSrc.GetLocalTransformation()
        tfB = xformDst.GetLocalTransformation()

        stack = xformDst.GetOrderedXformOps()
        opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

        for op, opName in zip(stack, opNames):
            self.assertEqual(op.GetName(), opName)

        self._assert_transform_close(tfA, tfB, places=4)
        self.assertTrue(xformDst.GetResetXformStack())

    async def test_internal_utils_expect_message(self):
        message = "This is a testing error message"
        pass_message = "This is a testing message that should be in the output"
        messages = [
            "This is a new testing error message",
            "This is another testing error message",
            "This is yet another testing error message",
        ]

        with self.subTest("single_presence"):
            with ExpectMessage(self, message, True):
                carb.log_error(message)
                carb.log_warn(pass_message)

        with self.subTest("single_absence"):
            with ExpectMessage(self, message, False):
                carb.log_warn(pass_message)

        with self.subTest("multi_presence_expect_at_least_one"):
            with ExpectMessage(self, messages, True, False):
                carb.log_error(messages[1])
                carb.log_warn(pass_message)

        with self.subTest("multi_presence_expect_all"):
            with ExpectMessage(self, messages, True, True):
                for m in messages:
                    carb.log_error(m)
                carb.log_warn(pass_message)

        with self.subTest("multi_absence_expect_at_most_n-1"):
            with ExpectMessage(self, messages, False, False):
                carb.log_error(messages[0])
                carb.log_error(messages[1])
                carb.log_warn(pass_message)

        with self.subTest("multi_absence_expect_nothing"):
            with ExpectMessage(self, messages, False, True):
                carb.log_warn(pass_message)

        with self.subTest("failed_single_presence"):
            with ExpectMessage(self, message, True, result_fn=self.assertFalse):
                ...

        with self.subTest("failed_single_absence"):
            with ExpectMessage(self, message, False, result_fn=self.assertFalse):
                carb.log_warn(message)

        with self.subTest("failed_multi_presence_expect_at_least_one"):
            with ExpectMessage(self, messages, True, False, result_fn=self.assertFalse):
                carb.log_warn(pass_message)

        with self.subTest("failed_multi_presence_expect_all"):
            with ExpectMessage(self, messages, True, True, result_fn=self.assertFalse):
                ...

        with self.subTest("failed_multi_absence_expect_at_most_n-1"):
            with ExpectMessage(self, messages, False, False, result_fn=self.assertFalse):
                for m in messages:
                    carb.log_error(m)
                carb.log_warn(pass_message)

        with self.subTest("failed_multi_absence_expect_nothing"):
            with ExpectMessage(self, messages, False, True, result_fn=self.assertFalse):
                for m in messages:
                    carb.log_warn(m)
