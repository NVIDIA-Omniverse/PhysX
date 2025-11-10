# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import omni
import carb
from pxr import Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics
import omni.physxdemos as demo
from omni.physx.scripts import utils, physicsUtils
import numpy as np
from omni.physx.scripts.assets_paths import AssetFolders
from omni.physxdemos.utils import franka_helpers
from omni.physxdemos.utils import numpy_utils


class FrankaBrickStackDemo(franka_helpers.FrankaDemoBase):
    title = "Franka Brick Stacking"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Franka robot arms assembling brick stacks with SDF collision"
    description = (
        "The brick-to-brick assembly is a demonstration of contact-rich signed-distance-field collision in PhysX. "
        "The arm is controlled with a finite state machine and Jacobian-based inverse kinematics. "
        "You may configure the demo parameters below. "
    )

    params = {}

    def __init__(self):
        super().__init__()

        # franka
        self._stool_height = 0.15
        self._franka_position = Gf.Vec3f(0.269, 0.1778, 0.0)
        # table:
        # position parameters
        self._table_scale = 0.8
        self._table_height = 0.762 * self._table_scale
        self._table_position = Gf.Vec3f(0.5, 0.0, 0.0)
        self._franka_stool_position = Gf.Vec3f(-0.027, 0.0, -0.001)

        # bricks
        self._brick_height = 9.6e-3  # minus dimples, basically stacking height
        self._brick_width = 16.0e-3
        self._brick_length = 32.0e-3

        self._static_brick_plane_length_count = 10  # brick-lengths
        static_brick_half_width = (self._static_brick_plane_length_count - 1) * self._brick_length * 0.5
        self._board_z_offset = 0.2 * self._brick_height
        self._static_bricks_origin = self._table_position + Gf.Vec3f(
            -static_brick_half_width, 0.0, self._table_height - self._brick_height + self._board_z_offset
        )

        self._num_dynamic_bricks_side = 3
        self._board_brick_coordinates = ((7, 5, 0), (12, 7, 1), (13, 5, 1))
        self._num_dynamic_bricks_board = len(self._board_brick_coordinates)
        self._num_dynamic_bricks = self._num_dynamic_bricks_board + self._num_dynamic_bricks_side

        self._gripper_tip_width = 0.02
        self._dynamic_bricks_x_delta = self._brick_width + 1.5 * self._gripper_tip_width
        row_width = self._dynamic_bricks_x_delta * (self._num_dynamic_bricks_side - 1)
        self._dynamic_bricks_side_start_position = Gf.Vec3f(0.5 - row_width * 0.5, -0.1, self._table_height)

        # pick first brick and place on second per entry
        self._pick_and_place_sequence = (
            (0, 1, 1),
            (0, 3, 2),
        )
        self._pick_and_place_index = 0

        # create new rng with seed:
        self._rng_seed = 8
        self._rng = np.random.default_rng(self._rng_seed)

        # sim options:
        self._franka_pos_iterations = 32
        self._pos_iterations = 32
        self._vel_iterations = 1
        self._brick_density = 565.0  # kg / m3  https://www.wired.com/2016/12/heres-much-lego-brick-stepped-worth/
        self._rest_offset = 0.7e-5  # m
        self._contact_offset = 1.0e-2  # m

        self._brick_mass = self._brick_density * self._brick_width * self._brick_width * 2.0 * self._brick_height
        rho = 1.25e-6  # single contact max allowed static penetration
        self._compliant_stiffness = self._gravity_magnitude * self._brick_mass / rho
        self._compliant_damping = 2.0 * math.sqrt(self._compliant_stiffness * self._brick_mass)
        self._sleep_threshold = 0.005
        self._stabilization_threshold = 0.005
        self._max_depenetration_vel = 5.0  # m/s
        self._brick_static_friction = 1.0
        self._brick_dynamic_friction = 1.0

        self._franka_translate = Gf.Vec3f(0.0, 0.0, self._table_height - 0.4)

        # setup asset paths:
        self._set_instanceable = False  # use asset instancing
        self.asset_paths = {
            "franka": demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "SubUSDs/Franka/franka_alt_fingers.usd"),
            "table": demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "SubUSDs/Table_Rounded/Table_Rounded.usd"),
            "stool": demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "SubUSDs/Franka_Stool/Franka_Stool.usd"),
            "twoByTwo": demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "SubUSDs/TwoByTwoSI_Sparse.usd"),
            "twoByFour": demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "SubUSDs/TwoByFourSI_Sparse.usd"),
            "brickMaterials": demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "SubUSDs/BrickMaterials.usd"),
        }
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.FRANKA_BRICK_STACK, "StagingBrickStack.usd")

        self._fsm_dt = 1.0 / 60.0

    def create(self, stage):
        super().create(stage=stage, Number_of_Robots=1)

    def _setup_camera(self):
        self._cam = UsdGeom.Camera.Define(self._stage, self.demo_camera)
        self._cam.AddTranslateOp().Set(Gf.Vec3d(0.5684, 0.1402, 0.6283))
        self._cam.AddRotateXYZOp().Set(Gf.Vec3d(94.6, 0.0, 141.5))
        self._cam.CreateFocalLengthAttr().Set(16.0)
        self._cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000000.0))

    def on_startup(self):
        if not (
            self._set_instanceable and self._enable_fabric
        ):  # we only need scene instancing if fabric and instancing is enabled
            return
        super().on_startup()

    def _setup_simulation(self):
        physxSceneAPI = super()._setup_simulation()

        # friction
        physxSceneAPI.CreateFrictionOffsetThresholdAttr().Set(0.01)
        physxSceneAPI.CreateFrictionCorrelationDistanceAttr().Set(0.0005)

        # GPU
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(10 * 1024)

    def _generate_fsm(self, env_idx):
        return BrickStackFSM(env_idx, self)

    def _add_custom(self, env_path):
        self._add_table(env_path=env_path)
        self._add_static_bricks(env_path=env_path)
        self._add_dynamic_bricks(env_path=env_path)

    def _get_asset_path(self, asset: str):
        return self.asset_paths[asset]

    def on_tensor_start(self, tensorApi):
        sim = super().on_tensor_start(tensorApi=tensorApi)

        self._bricks = sim.create_rigid_body_view("/World/envs/*/dynamic_bricks/dynamic_brick_*")
        self._pick_and_place_index = -1

        self._reset_bricks()

    def on_shutdown(self):
        self._bricks = None
        super().on_shutdown()

    def _reset_bricks(self):
        brick_inds = np.zeros(len(self._franka_indices) * self._num_dynamic_bricks, dtype=np.int32)
        brick_transforms = np.zeros((self._num_envs * self._num_dynamic_bricks, 7), dtype=np.float32)
        for env_no in range(len(self._franka_indices)):
            startInd = self._franka_indices[env_no] * self._num_dynamic_bricks
            brick_inds[env_no * self._num_dynamic_bricks:(env_no + 1) * self._num_dynamic_bricks] = np.arange(
                startInd, startInd + self._num_dynamic_bricks
            )
            brick_transforms[startInd:startInd + self._num_dynamic_bricks, :] = self._dynamic_bricks_init_poses
        self._bricks.set_transforms(brick_transforms, brick_inds)

    def _setup_materials(self):
        physicsMaterialScopePath = super()._setup_materials()

        self._brick_material_path = physicsMaterialScopePath.AppendChild("BrickMaterial")
        utils.addRigidBodyMaterial(
            self._stage,
            self._brick_material_path,
            density=self._brick_density,  # has two overlapping collision shapes so half density = proper weight
            staticFriction=self._brick_static_friction,
            dynamicFriction=self._brick_dynamic_friction,
        )

        matPrim = self._stage.GetPrimAtPath(self._brick_material_path)
        physxMaterialAPI = PhysxSchema.PhysxMaterialAPI.Apply(matPrim)
        physxMaterialAPI.CreateCompliantContactStiffnessAttr().Set(self._compliant_stiffness)
        physxMaterialAPI.CreateCompliantContactDampingAttr().Set(self._compliant_damping)

        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniGlass.mdl",
            mtl_name="OmniGlass",
            mtl_created_list=["/World/Looks/OmniGlass"],
            bind_selected_prims=False,
            select_new_prim=False,
        )

        # brick colors:
        self._dynamic_brick_render_materials_scope_path = self._defaultPrimPath.AppendChild(
            "DynamicBrickRenderMaterials"
        )
        self._stage.DefinePrim(self._dynamic_brick_render_materials_scope_path).GetReferences().AddReference(
            self.asset_paths["brickMaterials"], "/World/DynamicBrickMaterials"
        )
        materialScope = self._stage.GetPrimAtPath(self._dynamic_brick_render_materials_scope_path)
        self._dynamic_brick_render_material_paths = [prim.GetPath() for prim in materialScope.GetChildren()]
        assert len(self._dynamic_brick_render_material_paths)

        static_brick_render_materials_scope_path = self._defaultPrimPath.AppendChild("StaticBrickRenderMaterials")
        self._stage.DefinePrim(static_brick_render_materials_scope_path).GetReferences().AddReference(
            self.asset_paths["brickMaterials"], "/World/StaticBrickMaterials"
        )
        self._static_brick_render_material_path = static_brick_render_materials_scope_path.AppendChild("DarkGray")

    def _get_random_dynamic_brick_render_material_path(self) -> Sdf.Path:
        index = self._rng.integers(len(self._dynamic_brick_render_material_paths))
        return self._dynamic_brick_render_material_paths[index]

    def _add_stool(self, env_path):
        env_xform = UsdGeom.Xform.Get(self._stage, env_path)
        stool_path = env_xform.GetPath().AppendChild("franka_stool")
        self._stage.DefinePrim(stool_path).GetReferences().AddReference(self.asset_paths["stool"])
        stool_xform = UsdGeom.Xform.Get(self._stage, stool_path)
        utils.setStaticCollider(stool_xform.GetPrim(), approximationShape="boundingCube")
        assert stool_xform
        physicsUtils.set_or_add_translate_op(stool_xform, translate=self._franka_stool_position)

    def _add_table(self, env_path):
        env_xform = UsdGeom.Xform.Get(self._stage, env_path)
        table_path = env_xform.GetPath().AppendPath("table")
        assert self._stage.DefinePrim(table_path).GetReferences().AddReference(self.asset_paths["table"])
        xform = UsdGeom.Xform.Get(self._stage, table_path)
        utils.setStaticCollider(xform.GetPrim(), approximationShape="boundingCube")
        physicsUtils.set_or_add_translate_op(xform, translate=self._table_position)
        physicsUtils.set_or_add_scale_op(xform, scale=Gf.Vec3f(self._table_scale))

    def _add_static_bricks(self, env_path):
        brick_xform_path = env_path.AppendChild("static_brick_plane")
        static_brick_xform = UsdGeom.Xform.Define(self._stage, brick_xform_path)
        physicsUtils.set_or_add_translate_op(static_brick_xform, translate=self._static_bricks_origin)
        brick_xform_prim = self._stage.GetPrimAtPath(brick_xform_path)
        physicsUtils.add_physics_material_to_prim(self._stage, brick_xform_prim, self._brick_material_path)
        for i in range(self._static_brick_plane_length_count * self._static_brick_plane_length_count * 2):
            brick_path = brick_xform_path.AppendChild(f"static_brick_{i}")
            self._stage.DefinePrim(brick_path).GetReferences().AddReference(self._get_asset_path("twoByFour"))
            xform = UsdGeom.Xform.Get(self._stage, brick_path)
            assert xform
            row_no = i // self._static_brick_plane_length_count
            col_no = i % self._static_brick_plane_length_count
            brick_pos = Gf.Vec3f(row_no * self._brick_width, col_no * self._brick_length, 0.0)
            sqrt2div2 = math.sqrt(2) * 0.5
            physicsUtils.set_or_add_orient_op(xform, orient=Gf.Quatf(sqrt2div2, Gf.Vec3f(0, 0, sqrt2div2)))
            physicsUtils.set_or_add_translate_op(xform, translate=brick_pos)
            self._setup_brick(brick_path=brick_path, is_kinematic=True)

    def get_next_brick_targets(self) -> tuple:
        self._pick_and_place_index += 1
        if self._pick_and_place_index >= len(self._pick_and_place_sequence):
            return (None, True)
        return (self._pick_and_place_sequence[self._pick_and_place_index], False)

    def _add_dynamic_bricks(self, env_path):
        brick_xform_path = env_path.AppendChild("dynamic_bricks")
        UsdGeom.Xform.Define(self._stage, brick_xform_path)
        brick_xform_prim = self._stage.GetPrimAtPath(brick_xform_path)
        physicsUtils.add_physics_material_to_prim(self._stage, brick_xform_prim, self._brick_material_path)
        self._generate_brick_initial_poses()
        for i in range(self._num_dynamic_bricks):
            brick_path = brick_xform_path.AppendChild(f"dynamic_brick_{i}")
            self._stage.DefinePrim(brick_path).GetReferences().AddReference(self._get_asset_path("twoByFour"))
            xform = UsdGeom.Xform.Get(self._stage, brick_path)
            assert xform
            brick_pos = Gf.Vec3f(self._dynamic_bricks_init_poses[i, :3].tolist())
            w = float(self._dynamic_bricks_init_poses[i, -1])
            z = float(self._dynamic_bricks_init_poses[i, -2])
            physicsUtils.set_or_add_orient_op(xform, orient=Gf.Quatf(w, Gf.Vec3f(0.0, 0.0, z)))
            physicsUtils.set_or_add_translate_op(xform, translate=brick_pos)
            self._setup_brick(brick_path=brick_path, is_kinematic=False)

    def _setup_brick(self, brick_path, is_kinematic=False):
        prim = self._stage.GetPrimAtPath(brick_path)
        prim.SetInstanceable(self._set_instanceable)

        rbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rbAPI.CreateSolverPositionIterationCountAttr(self._pos_iterations)
        rbAPI.CreateSolverVelocityIterationCountAttr(self._vel_iterations)
        rbAPI.CreateMaxDepenetrationVelocityAttr().Set(self._max_depenetration_vel)
        rbAPI.CreateSleepThresholdAttr(self._sleep_threshold)
        rbAPI.CreateStabilizationThresholdAttr(self._stabilization_threshold)

        rbApi = UsdPhysics.RigidBodyAPI.Apply(prim)
        rbApi.CreateRigidBodyEnabledAttr(True)
        rbApi.CreateKinematicEnabledAttr(is_kinematic)

        # get physx collision API:
        assert not self._set_instanceable  # need to work with master if instancing is on
        meshPrim = prim.GetChild("BrickMesh")
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(meshPrim)
        assert physxCollisionAPI.CreateRestOffsetAttr().Set(self._rest_offset)
        assert physxCollisionAPI.CreateContactOffsetAttr().Set(self._contact_offset)

        # set mass and inertia properties of the brick's axis aligned bounding box to improve simulation stability
        massAPI = UsdPhysics.MassAPI.Apply(prim)
        massAPI.CreateMassAttr().Set(0.0032673575915396214)
        massAPI.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1.0427e-07, 3.12684e-07, 3.46431e-07))
        massAPI.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 3.40343e-10, 0.00571))

        if is_kinematic:
            omni.kit.commands.execute(
                "BindMaterialCommand",
                prim_path=brick_path,
                material_path=self._static_brick_render_material_path,
                strength=None,
            )
        else:
            omni.kit.commands.execute(
                "BindMaterialCommand",
                prim_path=brick_path,
                material_path=self._get_random_dynamic_brick_render_material_path(),
                strength=None,
            )

    def _generate_brick_initial_poses(self):
        self._dynamic_bricks_init_poses = np.zeros((self._num_dynamic_bricks, 7), dtype=np.float32)
        self._dynamic_bricks_init_poses[:, 2] = self._dynamic_bricks_side_start_position[2]

        for i in range(self._num_dynamic_bricks_side):
            sqrt2div2 = 0.5 * math.sqrt(2)
            self._dynamic_bricks_init_poses[i, -2:] = sqrt2div2  # quat to rot by 90 in z
            x = self._dynamic_bricks_side_start_position[0] + i * self._dynamic_bricks_x_delta
            y = self._dynamic_bricks_side_start_position[1]
            self._dynamic_bricks_init_poses[i, 0] = x
            self._dynamic_bricks_init_poses[i, 1] = y

        for i in range(self._num_dynamic_bricks_board):
            brick_index = i + self._num_dynamic_bricks_side
            coords = self._board_brick_coordinates[i]
            angle = coords[2] * math.pi * 0.5
            sinAngleDiv2 = math.sin(angle * 0.5)
            cosAngleDiv2 = math.cos(angle * 0.5)
            self._dynamic_bricks_init_poses[brick_index, -2] = sinAngleDiv2
            self._dynamic_bricks_init_poses[brick_index, -1] = cosAngleDiv2

            x = coords[0] * self._brick_width + self._static_bricks_origin[0]
            y = coords[1] * self._brick_width + self._static_bricks_origin[1]
            self._dynamic_bricks_init_poses[brick_index, 0] = x
            self._dynamic_bricks_init_poses[brick_index, 1] = y
            self._dynamic_bricks_init_poses[brick_index, 2] += self._board_z_offset

    def on_physics_step(self, dt):
        carb.profiler.begin(1, "Franka Brick FSM Update")
        self._time += self._sim_dt
        self._fsm_time += self._sim_dt

        if self._fsm_time < self._fsm_dt:
            carb.profiler.end(1)
            return
        self._fsm_time = 0.0

        # fsm runs on CPU:
        # get transforms
        brick_transforms = self._bricks.get_transforms()

        # get end effector transforms
        hand_transforms = self._hands.get_transforms()
        hand_poses = hand_transforms[:, :7]

        # get franka DOF states
        dof_pos = self._frankas_view.get_dof_positions()

        # FSM update
        for env in range(self._num_envs):
            start_bricks = self._num_dynamic_bricks * env
            end_bricks = start_bricks + self._num_dynamic_bricks
            self._fsms[env].update(
                self._fsm_dt, brick_transforms[start_bricks:end_bricks, :], hand_poses[env, :], dof_pos[env, 6:9]
            )
            self._d_pose[env, :] = self._fsms[env].franka_state.pose_delta
            self._grip_sep[env] = self._fsms[env].franka_state.gripper_separation

        self._run_ik(direct_z_rotation=True)

        carb.profiler.end(1)


class BrickStackFSM:
    def __init__(
        self,
        env_idx,
        demo_instance,
    ):
        self._env_idx = env_idx

        self._demo_instance = demo_instance

        self.franka_state = franka_helpers.FrankaState()

        # init states in reset
        self.reset()

        # brick loss detection:
        self._check_for_brick_loss = False
        self._brick_loss_distance_thres = 0.03

        # position parameters:
        # general
        self._hand_down_quat = np.array([1, 0, 0, 0], dtype=np.float32)
        # gripping
        grip_z_offset = 0.003
        self._pickup_pos_offset = np.array(
            [0, 0, 0.103 + grip_z_offset + self._demo_instance._brick_height], dtype=np.float32
        )
        self._pickup_q = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(math.pi / 2.0), self._hand_down_quat)
        self._above_pickup_pos_offset = np.array(
            [0, 0, 0.12 + grip_z_offset + self._demo_instance._brick_height], dtype=np.float32
        )
        # gripper parameters:
        self._pickup_gripper_sep = self._demo_instance._brick_width + 0.004
        self._lift_pos_offset = np.array([0, -0.001, 0.15], dtype=np.float32)
        self._lift_q = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(math.pi / 2.0), self._hand_down_quat)
        self._place_q = self._pickup_q
        self._above_place_pos_offset = self._above_pickup_pos_offset.copy()
        self._place_offset = np.array(
            [0, 0, 0.102 + grip_z_offset + 2.0 * self._demo_instance._brick_height], dtype=np.float32
        )
        self._grip_sep = self._demo_instance._brick_width - 0.003
        self._place_sep = self._demo_instance._brick_width - 0.0015

        self._ungrip_sep = self._demo_instance._brick_width * 2.0

        # tolerances:
        self.franka_state.pos_error_threshold = 3.0e-3
        self.franka_state.rot_error_threshold = 2.5e-2

        # done
        sbsp = demo_instance._static_bricks_origin
        self._done_pos = np.array([sbsp[0] - 0.05, sbsp[1], sbsp[2] + 0.25])
        self._done_q = self._hand_down_quat

    def reset(self):
        self._state = "start"
        self._check_for_brick_loss = False
        self._pick_was_last_pick = False
        self._current_pick_brick_index = -1
        self._current_place_brick_index = -1

    def _target_next_brick(self, brick_poses) -> bool:
        self._check_for_brick_loss = False
        last_pick_brick = self._current_pick_brick_index
        place_info, done = self._demo_instance.get_next_brick_targets()
        if done:
            return True
        self._current_pick_brick_index = place_info[0]
        self._current_place_brick_index = place_info[1]
        pick_stack_height = place_info[2]
        self._pick_was_last_pick = last_pick_brick == self._current_pick_brick_index
        self._current_pick_brick_pose = brick_poses[self._current_pick_brick_index, :].copy()
        self._current_place_brick_pose = brick_poses[self._current_place_brick_index, :].copy()
        self._current_place_brick_pose[2] += self._demo_instance._brick_height * (pick_stack_height - 1)
        return False

    def _get_target_hand_pose_and_quat(self, brick_pose, pos_offset: np.array):
        return (brick_pose[:3] + pos_offset, numpy_utils.quat_mul(brick_pose[3:], self._hand_down_quat))

    def update(
        self,
        fsm_dt,
        brick_poses,
        hand_pose,
        current_hand_dof_pos,
    ):
        current_gripper_sep = current_hand_dof_pos[1] + current_hand_dof_pos[2]

        new_state = self._state

        if self._state == "advance":
            if self.franka_state.advance(hand_pose=hand_pose, dt=fsm_dt):
                new_state = self._state_after_advance
        elif self._state == "start":
            done = self._target_next_brick(brick_poses=brick_poses)
            if done:
                new_state = "go_to_done"
            else:
                new_state = "goto_above_brick_pickup"
        elif self._state == "goto_above_brick_pickup":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_pick_brick_pose, pos_offset=self._above_pickup_pos_offset
            )
            new_state = "advance"
            self._state_after_advance = "go_to_pickup"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=target_pos,
                start_quat=target_q,
                end_quat=target_q,
                duration=2.0,
                use_alignment=True,
            )
        elif self._state == "go_to_pickup":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            pickup_target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_pick_brick_pose, pos_offset=self._pickup_pos_offset
            )
            new_state = "advance"
            self._state_after_advance = "grip_brick"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=pickup_target_pos,
                start_quat=target_q,
                end_quat=target_q,
                duration=1.0,
                use_alignment=True,
            )
        elif self._state == "grip_brick":
            self.franka_state.gripper_separation = self._grip_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_pick_brick_pose, pos_offset=self._pickup_pos_offset
            )
            target_pos, _ = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            self.franka_state.update_pose_delta_from_target(target_pos, target_q, hand_pose)
            gripped = current_gripper_sep < self._demo_instance._brick_width * 1.1
            if gripped:
                new_state = "lift"
        elif self._state == "lift":
            self.franka_state.gripper_separation = self._grip_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_pick_brick_pose, pos_offset=self._lift_pos_offset
            )
            new_state = "advance"
            self._state_after_advance = "go_above_placement"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=self.franka_state.pose_motion.end_pos,
                end_pos=target_pos,
                start_quat=target_q,
                end_quat=target_q,
                duration=1.0,
                use_alignment=True,
            )
        elif self._state == "go_above_placement":
            self.franka_state.gripper_separation = self._grip_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_place_brick_pose, pos_offset=self._above_place_pos_offset
            )
            new_state = "advance"
            self._state_after_advance = "place_brick"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=self.franka_state.pose_motion.end_pos,
                end_pos=target_pos,
                start_quat=target_q,
                end_quat=target_q,
                duration=1.5,
                use_alignment=True,
            )
        elif self._state == "place_brick":
            self.franka_state.gripper_separation = self._place_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_place_brick_pose, pos_offset=self._place_offset
            )
            new_state = "advance"
            self._state_after_advance = "start_ungrip"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=self.franka_state.pose_motion.end_pos,
                end_pos=target_pos,
                start_quat=target_q,
                end_quat=target_q,
                duration=1.5,
                use_alignment=True,
            )
        elif self._state == "start_ungrip":
            self._ungrip_pos = hand_pose[:3].copy()
            new_state = "ungrip"
        elif self._state == "ungrip":
            self.franka_state.gripper_separation = self._ungrip_sep
            _, target_q = self._get_target_hand_pose_and_quat(
                self._current_place_brick_pose, pos_offset=self._place_offset
            )
            target_pos, _ = self.franka_state.offset_target_position_to_align(self._ungrip_pos, hand_pose)
            self.franka_state.update_pose_delta_from_target(target_pos, target_q, hand_pose)
            ungripped = current_gripper_sep > self._ungrip_sep * 0.75
            if ungripped:
                done = self._target_next_brick(brick_poses=brick_poses)
                if done:
                    new_state = "lift_after_last_place"
                elif self._pick_was_last_pick:
                    new_state = "go_to_pickup"
                else:
                    new_state = "lift_after_place"
        elif self._state == "lift_after_last_place":
            self.franka_state.gripper_separation = self._ungrip_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_place_brick_pose, pos_offset=self._lift_pos_offset
            )
            new_state = "advance"
            self._state_after_advance = "go_to_done"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=target_pos + self._place_offset - self._lift_pos_offset,
                end_pos=target_pos,
                start_quat=target_q,
                end_quat=target_q,
                duration=1.0,
                use_alignment=True,
            )
        elif self._state == "lift_after_place":
            self.franka_state.gripper_separation = self._ungrip_sep
            target_pos, target_q = self._get_target_hand_pose_and_quat(
                self._current_place_brick_pose, pos_offset=self._lift_pos_offset
            )
            new_state = "advance"
            self._state_after_advance = "goto_above_brick_pickup"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=target_pos + self._place_offset - self._lift_pos_offset,
                end_pos=target_pos,
                start_quat=self._place_q,
                end_quat=self._place_q,
                duration=1.0,
                use_alignment=True,
            )
        elif self._state == "go_to_done":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            new_state = "advance"
            self._state_after_advance = "done"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=self._done_pos,
                start_quat=self._done_q,
                end_quat=self._done_q,
                duration=2.5,
                use_alignment=True,
            )
        elif self._state == "done":
            self.franka_state.update_pose_delta_from_target(self._done_pos, self._done_q, hand_pose)
        else:
            assert False

        if new_state != self._state:
            self._state = new_state
