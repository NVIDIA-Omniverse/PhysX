# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import math
import numpy as np
import omni
from pxr import Gf, Sdf, UsdGeom, UsdShade, PhysxSchema, UsdPhysics
import omni.physxdemos as demo
from omni.physx.scripts import deformableUtils, utils, physicsUtils
from omni.physx.scripts.assets_paths import AssetFolders
import omni.physx.bindings._physx as physx_settings_bindings
from omni.physxdemos.utils import franka_helpers
from omni.physxdemos.utils import numpy_utils

deformable_beta_on = carb.settings.get_settings().get_as_bool(physx_settings_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

def orientation_error(desired, current):
    cc = numpy_utils.quat_conjugate(current)
    q_r = numpy_utils.quat_mul(desired, cc)
    return q_r[:, 0:3] * np.sign(q_r[:, 3])[:, None]


def cube_grasping_yaw(q, corners):
    """returns horizontal rotation required to grasp cube"""
    rc = numpy_utils.quat_rotate(q, corners)
    yaw = (np.arctan2(rc[:, 1], rc[:, 0]) - 0.25 * math.pi) % (0.5 * math.pi)
    theta = 0.5 * yaw
    w = np.cos(theta)
    x = np.zeros_like(w)
    y = np.zeros_like(w)
    z = np.sin(theta)
    yaw_quats = np.stack([x, y, z, w], axis=-1)
    return yaw_quats


class FrankaDeformableDemo(demo.AsyncDemoBase):
    title = "Franka Deformable"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Demo showing Franka robot arms interacting with deformable cubes"
    description = "The Franka robot arms use Jacobian-based inverse kinematics to lift and drop the red cube in a bowl of deformable cubes."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    params = {"Num_Frankas": demo.IntParam(4, 1, 20, 1), "Num_Greens": demo.IntParam(9, 0, 9, 1)}

    demo_camera = Sdf.Path("/World/Camera")

    def __init__(self):
        super().__init__(enable_tensor_api=True, enable_fabric=False)
        self._reset_hydra_instancing_on_shutdown = False

        # when looping, box will be dropped after lifting and picking action will repeat
        self.loop = True

        self.asset_paths = {
            "franka": demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "SubUSDs/Franka/franka_alt_fingers.usd"),
            "table": demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "SubUSDs/Table_Rounded/Table_Rounded.usd"),
            "stool": demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "SubUSDs/Franka_Stool/Franka_Stool.usd"),
            "cube": demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "SubUSDs/box_high.usd"),
            "bowl": demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "SubUSDs/bowl_plate.usd"),
            "cube_materials": demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "SubUSDs/deformable_cubes_materials.usd"),
        }
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.FRANKA_DEFORMABLE, "StagingDeformable.usd")

    def on_startup(self):
        sceneGraphInstancingEnabled = carb.settings.get_settings().get("/persistent/omnihydra/useSceneGraphInstancing")
        if not sceneGraphInstancingEnabled:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", True)
            self._reset_hydra_instancing_on_shutdown = True

    def create_jello_cube(self, stage, path, name, position, size, mesh_path, phys_material_path, grfx_material):
        if deformable_beta_on:
            xform_path = path.AppendChild(name)
            xform = UsdGeom.Xform.Define(stage, xform_path)
            skinMesh_path = xform_path.AppendChild("mesh")
            stage.DefinePrim(skinMesh_path).GetReferences().AddReference(mesh_path)
            skinMesh = UsdGeom.Mesh.Define(stage, skinMesh_path)
            skinMesh.AddTranslateOp().Set(position)
            skinMesh.AddOrientOp().Set(Gf.Quatf(1.0))
            skinMesh.AddScaleOp().Set(Gf.Vec3f(size, size, size))

            simMesh_path = xform_path.AppendChild("simMesh")
            collMesh_path = xform_path.AppendChild("collMesh")

            deformableUtils.create_auto_volume_deformable_hierarchy(stage,
                root_prim_path = xform_path,
                simulation_tetmesh_path = simMesh_path,
                collision_tetmesh_path = collMesh_path,
                cooking_src_mesh_path = skinMesh_path,
                simulation_hex_mesh_enabled = True,
                cooking_src_simplification_enabled = True,
                set_visibility_with_guide_purpose = True
            )
            # Set resolution attribute of PhysxAutoDeformableHexahedralMeshAPI
            xform.GetPrim().GetAttribute("physxDeformableBody:resolution").Set(3)
            xform.GetPrim().ApplyAPI("PhysxBaseDeformableBodyAPI")
            xform.GetPrim().GetAttribute("physxDeformableBody:selfCollision").Set(False)
            xform.GetPrim().GetAttribute("physxDeformableBody:solverPositionIterationCount").Set(self.pos_iterations)

            # Set collision mesh properties
            collMeshPrim = stage.GetPrimAtPath(collMesh_path)
            physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(collMeshPrim)
            physxCollisionAPI.GetContactOffsetAttr().Set(0.02)
            physxCollisionAPI.CreateRestOffsetAttr().Set(0.001)

            # Bind material
            physicsUtils.add_physics_material_to_prim(stage, xform.GetPrim(), phys_material_path)
            omni.kit.commands.execute(
                "BindMaterialCommand", prim_path=xform_path, material_path=grfx_material, strength=None
            )
        else:
            cube_path = path.AppendChild(name)
            stage.DefinePrim(cube_path).GetReferences().AddReference(mesh_path)
            skinMesh = UsdGeom.Mesh.Define(stage, cube_path)
            skinMesh.AddTranslateOp().Set(position)
            skinMesh.AddOrientOp().Set(Gf.Quatf(1.0))
            skinMesh.AddScaleOp().Set(Gf.Vec3f(size, size, size))
            deformableUtils.add_physx_deformable_body(
                stage,
                cube_path,
                simulation_hexahedral_resolution=3,
                collision_simplification=True,
                self_collision=False,
                solver_position_iteration_count=self.pos_iterations,
            )
            physicsUtils.add_physics_material_to_prim(stage, skinMesh.GetPrim(), phys_material_path)
            physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(skinMesh.GetPrim())
            physxCollisionAPI.GetContactOffsetAttr().Set(0.02)
            physxCollisionAPI.CreateRestOffsetAttr().Set(0.001)
            omni.kit.commands.execute(
                "BindMaterialCommand", prim_path=cube_path, material_path=grfx_material, strength=None
            )

    def create(self, stage, Num_Frankas, Num_Greens):
        self.defaultPrimPath = stage.GetDefaultPrim().GetPath()
        self.stage = stage
        self.num_envs = Num_Frankas
        self.num_greens = Num_Greens

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, self.defaultPrimPath.AppendChild("physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.807)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim(), False)
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateFrictionOffsetThresholdAttr().Set(0.001)
        physxSceneAPI.CreateFrictionCorrelationDistanceAttr().Set(0.0005)
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(10 * 1024)
        # clamp iterations:
        self.pos_iterations = 20
        self.vel_iterations = 1
        physxSceneAPI.GetMaxPositionIterationCountAttr().Set(self.pos_iterations)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        # setup ground collision plane:
        utils.addPlaneCollider(stage, "/World/physicsGroundPlaneCollider", "Z")

        # deformable material path
        deformable_material_path = omni.usd.get_stage_next_free_path(stage, "/DeformableBodyMaterial", True)
        deformable_material2_path = omni.usd.get_stage_next_free_path(stage, "/DeformableBodyMaterial2", True)

        if deformable_beta_on:
            deformableUtils.add_deformable_material(stage, deformable_material_path,
                youngs_modulus=10000000.0,
                poissons_ratio=0.499,
                dynamic_friction=1.0,
                density=300.0
            )
            mat_prim = stage.GetPrimAtPath(deformable_material_path)
            mat_prim.ApplyAPI("PhysxDeformableMaterialAPI")
            mat_prim.GetAttribute("physxDeformableMaterial:elasticityDamping").Set(0.0001)

            deformableUtils.add_deformable_material(stage, deformable_material2_path,
                youngs_modulus=4000000.0,
                poissons_ratio=0.499,
                dynamic_friction=0.05,
                density=100.0
            )
            mat2_prim = stage.GetPrimAtPath(deformable_material2_path)
            mat2_prim.ApplyAPI("PhysxDeformableMaterialAPI")
            mat2_prim.GetAttribute("physxDeformableMaterial:elasticityDamping").Set(0.005)
        else:
            deformableUtils.add_deformable_body_material(
                stage,
                deformable_material_path,
                youngs_modulus=10000000.0,
                poissons_ratio=0.499,
                damping_scale=0.0,
                elasticity_damping=0.0001,
                dynamic_friction=1.0,
                density=300,
            )
            deformableUtils.add_deformable_body_material(
                stage,
                deformable_material2_path,
                youngs_modulus=4000000.0,
                poissons_ratio=0.499,
                damping_scale=0.0,
                elasticity_damping=0.005,
                dynamic_friction=0.05,
                density=100,
            )

        # Franka finger friction
        mat_prim = UsdShade.Material.Define(stage, "/World/FrankaFingerPhysicsMaterial")
        finger_material = UsdPhysics.MaterialAPI.Apply(mat_prim.GetPrim())
        finger_material.CreateStaticFrictionAttr().Set(1.0)
        finger_material.CreateDynamicFrictionAttr().Set(1.0)

        # position parameters
        table_scale = 0.8
        table_height = 0.762 * table_scale
        table_position = Gf.Vec3f(0.5, 0.0, 0.0)
        env_spacing = 2
        num_envs_per_side = max(1, round(math.sqrt(self.num_envs)))
        row_half_length = (num_envs_per_side - 1) * env_spacing * 0.5

        # create deformable cube parameters
        self.box_size = 0.05
        box_loc = Gf.Vec3f(table_position) + Gf.Vec3f(0.0, 0.0, table_height + self.box_size * 3.5)

        # Position Camera Based on Scene Size
        cam = UsdGeom.Camera.Define(stage, self.demo_camera)
        cam_height = 1.1
        location = Gf.Vec3f(row_half_length + 6.5, row_half_length + 4.0, cam_height)
        target = Gf.Vec3f(0.0, 0.0, cam_height - 0.4)
        delta = target - location
        rotZ = math.degrees(math.atan2(-delta[0], delta[1]))
        rotX = math.degrees(math.atan2(delta[2], math.sqrt(delta[0] ** 2.0 + delta[1] ** 2.0)))
        rotZQ = Gf.Quatf(Gf.Rotation(Gf.Vec3d([0, 0, 1]), rotZ).GetQuat())
        rotXQ = Gf.Quatf(Gf.Rotation(Gf.Vec3d([1, 0, 0]), rotX + 90).GetQuat())
        physicsUtils.setup_transform_as_scale_orient_translate(cam)
        physicsUtils.set_or_add_translate_op(cam, translate=location)
        physicsUtils.set_or_add_orient_op(cam, orient=rotZQ * rotXQ)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100))

        # get materials for deformable cubes
        looks_path = self.defaultPrimPath.AppendChild("Looks")
        stage.DefinePrim(looks_path).GetReferences().AddReference(self.asset_paths["cube_materials"], "/Looks")

        green_glass_path = looks_path.AppendChild("GreenGlass")
        red_glass_path = looks_path.AppendChild("RedGlass")

        # drive and default dof pos:
        self.default_dof_pos, drive_params = self.get_franka_parameters()

        # create grid of frankas and tables:
        envsScopePath = self.defaultPrimPath.AppendPath("envs")
        UsdGeom.Scope.Define(stage, envsScopePath)
        for i in range(self.num_envs):
            # create env:
            col_number = i % num_envs_per_side
            row_number = i // num_envs_per_side
            x_pos = -row_half_length + col_number * env_spacing
            y_pos = -row_half_length + row_number * env_spacing
            env_xform = UsdGeom.Xform.Define(stage, envsScopePath.AppendChild(f"env_{i}"))
            env_xform.AddTranslateOp().Set(Gf.Vec3f(x_pos, y_pos, 0.0))

            # add franka and its stool
            franka_stool_position = Gf.Vec3f(-0.027, 0.0, -0.001)
            stool_path = env_xform.GetPath().AppendChild("franka_stool")
            stage.DefinePrim(stool_path).GetReferences().AddReference(self.asset_paths["stool"])
            stool_xform = UsdGeom.Xform.Get(stage, stool_path)
            utils.setStaticCollider(stool_xform.GetPrim(), approximationShape="boundingCube")
            assert stool_xform
            physicsUtils.set_or_add_translate_op(stool_xform, translate=franka_stool_position)
            franka_path = env_xform.GetPath().AppendChild("franka")
            assert stage.DefinePrim(franka_path).GetReferences().AddReference(self.asset_paths["franka"], "/panda")
            franka_xform = UsdGeom.Xform.Get(stage, franka_path)
            assert franka_xform
            physicsUtils.set_or_add_translate_op(franka_xform, translate=Gf.Vec3f(0.0, 0.0, table_height - 0.4))
            physicsUtils.set_or_add_scale_op(franka_xform, scale=Gf.Vec3f(0.01))
            # setup drives:
            franka_helpers.configure_franka_drives(stage, franka_path, self.default_dof_pos, drive_params)
            # setup iterations:
            physxArticulationAPI = PhysxSchema.PhysxArticulationAPI.Apply(franka_xform.GetPrim())
            physxArticulationAPI.GetSolverPositionIterationCountAttr().Set(self.pos_iterations)
            physxArticulationAPI.GetSolverVelocityIterationCountAttr().Set(self.vel_iterations)
            # setup fingers
            prim = stage.GetPrimAtPath(str(env_xform.GetPath()) + "/franka/panda_leftfinger/geometry")
            bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
            bindingAPI.Bind(mat_prim, UsdShade.Tokens.weakerThanDescendants, "physics")
            prim = stage.GetPrimAtPath(str(env_xform.GetPath()) + "/franka/panda_rightfinger/geometry")
            bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
            bindingAPI.Bind(mat_prim, UsdShade.Tokens.weakerThanDescendants, "physics")
            # set max joint velocity
            joint = PhysxSchema.PhysxJointAPI.Get(
                stage, str(env_xform.GetPath()) + "/franka/panda_hand/panda_finger_joint1"
            )
            joint.GetMaxJointVelocityAttr().Set(1)
            joint = PhysxSchema.PhysxJointAPI.Get(
                stage, str(env_xform.GetPath()) + "/franka/panda_hand/panda_finger_joint2"
            )
            joint.GetMaxJointVelocityAttr().Set(1)

            # add table:
            table_path = env_xform.GetPath().AppendPath("table")
            assert stage.DefinePrim(table_path).GetReferences().AddReference(self.asset_paths["table"])
            xform = UsdGeom.Xform.Get(stage, table_path)
            utils.setStaticCollider(xform.GetPrim(), approximationShape="boundingCube")
            physicsUtils.set_or_add_translate_op(xform, translate=table_position)
            physicsUtils.set_or_add_scale_op(xform, scale=Gf.Vec3f(table_scale))

            # add bowl:
            bowl_path = env_xform.GetPath().AppendPath("bowl")
            assert (
                stage.DefinePrim(bowl_path).GetReferences().AddReference(self.asset_paths["bowl"], "/World/bowl_plate")
            )
            xform = UsdGeom.Xform.Get(stage, bowl_path)
            xform.AddTranslateOp().Set(table_position + Gf.Vec3f(0.0, 0.0, table_height))
            rigidbody_api = PhysxSchema.PhysxRigidBodyAPI.Apply(
                stage.GetPrimAtPath(bowl_path.AppendChild("bowl_plate"))
            )
            rigidbody_api.CreateSolverPositionIterationCountAttr(self.pos_iterations)
            rigidbody_api.CreateSolverVelocityIterationCountAttr(self.vel_iterations)

            # create soft cube
            self.create_jello_cube(
                stage,
                env_xform.GetPath(),
                "box",
                box_loc,
                self.box_size,
                self.asset_paths["cube"],
                deformable_material_path,
                red_glass_path,
            )

            # create extras
            extras_path = env_xform.GetPath().AppendChild("extras")
            ex_spacing = 1.5
            for z in range(3):
                for y in range(3):
                    for x in range(3):
                        if x == 1 & y == 1 & z == 1:
                            continue
                        j = x + y * 3 + z * 3 * 3
                        if j > 13:
                            j -= 1
                        if j >= self.num_greens:
                            break
                        self.create_jello_cube(
                            stage,
                            extras_path,
                            f"extra_{j}",
                            box_loc
                            + Gf.Vec3f((x - 1) * ex_spacing, (y - 1) * ex_spacing, (z - 1) * ex_spacing)
                            * self.box_size,
                            self.box_size,
                            self.asset_paths["cube"],
                            deformable_material2_path,
                            green_glass_path,
                        )
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

    def on_tensor_start(self, tensorApi):
        sim = tensorApi.create_simulation_view("numpy")
        sim.set_subspace_roots("/World/envs/*")
        # franka view
        self.frankas = sim.create_articulation_view("/World/envs/*/franka")
        self.franka_indices = np.arange(self.frankas.count, dtype=np.int32)

        # set default dof pos:
        init_dof_pos = np.stack(self.num_envs * [np.array(self.default_dof_pos, dtype=np.float32)])
        self.frankas.set_dof_positions(init_dof_pos, self.franka_indices)

        # end effector view
        self.hands = sim.create_rigid_body_view("/World/envs/*/franka/panda_hand")

        # get initial hand transforms
        init_hand_transforms = self.hands.get_transforms().copy()
        self.init_pos = init_hand_transforms[:, :3]
        self.init_rot = init_hand_transforms[:, 3:]

        if deformable_beta_on:
            self.boxes = sim.create_volume_deformable_body_view("/World/envs/*/box")
        else:
            self.boxes = sim.create_soft_body_view("/World/envs/*/box")

        # box corner coords, used to determine grasping angle
        box_half_size = 0.5 * self.box_size
        corner_coord = np.array([box_half_size, box_half_size, box_half_size])
        self.corners = np.stack(self.num_envs * [corner_coord])

        # get initial box transforms
        self.init_box_transforms = self.boxes.get_transforms().copy()

        # downward direction
        self.down_dir = np.array([0, 0, -1]).reshape(3, 1)

        # hand orientation for grasping
        self.down_q = np.stack(self.num_envs * [np.array([1.0, 0.0, 0.0, 0.0])])

        # prevent garbage collector from deleting the sim 
        self.sim = sim

    def on_shutdown(self):
        self.frankas = None
        self.hands = None
        self.boxes = None
        self.sim = None

        # disable hydra instancing if they had to be enabled
        if self._reset_hydra_instancing_on_shutdown:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", False)
        super().on_shutdown()

    def on_physics_step(self, dt):
        # get box transforms
        box_transforms = self.boxes.get_transforms()
        box_pos = box_transforms[:, :3]

        # get end effector transforms
        hand_transforms = self.hands.get_transforms()
        hand_pos = hand_transforms[:, :3]
        hand_rot = hand_transforms[:, 3:]

        # get franka DOF states
        dof_pos = self.frankas.get_dof_positions()

        # get franka jacobians
        jacobians = self.frankas.get_jacobians()

        # direction from end effector to box
        to_box = box_pos - hand_pos
        box_dist = np.linalg.norm(to_box, axis=1)
        box_dir = to_box / box_dist[:, None]
        box_dot = (box_dir @ self.down_dir).flatten()

        # how far the hand should be from box for grasping
        grasp_offset = 0.10

        # determine if we're holding the box (grippers are closed and box is near)
        gripper_sep = dof_pos[:, 7] + dof_pos[:, 8]
        gripped = (gripper_sep < self.box_size) & (box_dist < grasp_offset * 2.0)

        # determine if the box is unreachable (too far from initial box position)
        from_start = box_pos - self.init_box_transforms[:, :3]
        start_dist = np.linalg.norm(from_start, axis=1)
        unreachable = start_dist > 0.5

        # if hand is above box, descend to grasp offset; otherwise, seek a position above the box
        above_box = (box_dot >= 0.9) & (box_dist < grasp_offset * 3)
        grasp_pos = box_pos.copy()
        grasp_pos[:, 2] = np.where(above_box, box_pos[:, 2] + grasp_offset, box_pos[:, 2] + grasp_offset * 2.5)

        # if box is gripped, return to initial pose; otherwise get into grasping pose
        goal_pos = np.where(gripped[:, None], self.init_pos, grasp_pos)
        goal_rot = self.down_q

        # compute position and orientation error
        pos_err = goal_pos - hand_pos
        max_err = 0.05  # clamp max position error
        pos_err_mag = (
            np.linalg.norm(pos_err, axis=1)[:, None] + 0.0001
        )  # VR: add a tiny number to prevent division-by-zero warning
        pos_err = np.where(pos_err_mag > max_err, pos_err * max_err / pos_err_mag, pos_err)  # !!!
        orn_err = orientation_error(goal_rot, hand_rot)
        dpose = np.concatenate([pos_err, orn_err], -1)[:, None].transpose(0, 2, 1)

        # jacobian entries corresponding to franka hand
        franka_hand_index = 8  # !!!
        j_eef = jacobians[:, franka_hand_index - 1, :]

        # solve damped least squares
        j_eef_T = np.transpose(j_eef, (0, 2, 1))
        d = 0.05  # damping term
        lmbda = np.eye(6) * (d**2)
        u = (j_eef_T @ np.linalg.inv(j_eef @ j_eef_T + lmbda) @ dpose).reshape(self.num_envs, 9)

        # update position targets
        pos_targets = dof_pos + u  # * 0.3

        # close the grippers when we're near the box; if looping, open them when we return to starting position
        if self.loop:
            close_gripper = np.where(gripped, (box_dist < grasp_offset * 1.5), (box_dist < grasp_offset * 1.1))
        else:
            close_gripper = box_dist < grasp_offset

        grip0 = 0.015
        grip1 = 0.05

        grip_acts = np.where(
            close_gripper[:, None],
            [[grip0, grip0]] * self.num_envs,
            [[grip1, grip1]] * self.num_envs,
        )
        # pos_targets[:, 7:9] = pos_targets[:, 7:9] * 0.3 + grip_acts * 0.7
        pos_targets[:, 7:9] = grip_acts

        # reset the hand if the cube's unreachable
        pos_targets = np.where(unreachable[:, None], self.default_dof_pos, pos_targets)

        # apply position targets
        self.frankas.set_dof_position_targets(pos_targets, self.franka_indices)

    def get_franka_parameters(self):
        default_dof_pos = [0.0, 0.0, 0.0, -0.95, 0.0, 1.12, 0.0, 0.02, 0.02]
        revolute_drive_params = {
            "stiffness": 2000.0,
            "damping": 50.0,
            "maxForce": 3000.0,
        }

        # linear params:
        linear_drive_params = {
            "stiffness": 2000.0,
            "damping": 50.0,
            "maxForce": 3000.0,
        }

        drive_params = [revolute_drive_params] * 7
        drive_params.extend([linear_drive_params] * 2)

        # adjust
        drive_params[5]["stiffness"] = 800.0
        drive_params[6]["stiffness"] = 800.0

        return default_dof_pos, drive_params
