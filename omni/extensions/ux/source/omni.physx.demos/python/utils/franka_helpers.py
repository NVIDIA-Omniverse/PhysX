# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
import numpy as np
import carb
import omni

from pxr import Sdf, UsdPhysics, Gf, UsdGeom, PhysxSchema, UsdShade
from omni.physx.scripts import utils, physicsUtils
from usdrt import Gf as usdrt_Gf

import omni.physxdemos as demo

from omni.physxdemos.utils import numpy_utils


def set_position_drive(prim, target_position, stiffness, damping, max_force):
    assert prim

    # set drive type ("angular" or "linear")
    if prim.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        drive = UsdPhysics.DriveAPI.Apply(prim, "linear")

    assert drive
    drive.CreateTargetPositionAttr(target_position)
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.GetDampingAttr().Set(damping)
    drive.CreateMaxForceAttr(max_force)
    drive.GetMaxForceAttr().Set(max_force)
    drive.CreateTypeAttr().Set("force")


def configure_franka_drives(usd_stage, franka_prim_path: Sdf.Path, default_dof_pos=None, drive_params=None):
    """
    default dof pos are the default dof positions in rad or meters in the order:
    [panda_joint_1, ..., panda_joint_7, panda_finger_joint1, panda_finger_joint2]

    the drive params are a list of dicts with entries:
    drive_params[0]["stiffness"]
    drive_params[0]["damping"]
    drive_params[0]["maxForce"]

    The list follows the same order as the default dof positions
    """

    # get fallback defaults if nothing is provided
    ddofpos, ddriveparams = get_default_franka_parameters()
    if default_dof_pos is None:
        default_dof_pos = ddofpos
    if drive_params is None:
        drive_params = ddriveparams

    # rot drives
    for i in range(7):
        set_position_drive(
            usd_stage.GetPrimAtPath(franka_prim_path.AppendPath(f"panda_link{i}/panda_joint{i+1}")),
            math.degrees(default_dof_pos[i]),
            drive_params[i]["stiffness"],
            drive_params[i]["damping"],
            drive_params[i]["maxForce"],
        )
    # linear drives
    for i in range(2):
        set_position_drive(
            usd_stage.GetPrimAtPath(franka_prim_path.AppendPath(f"panda_hand/panda_finger_joint{i+1}")),
            default_dof_pos[7 + i],
            drive_params[7 + i]["stiffness"],
            drive_params[7 + i]["damping"],
            drive_params[7 + i]["maxForce"],
        )


def get_default_franka_parameters():
    """
    Returns default values for the franka degrees of freedom (dof) and the franka joint drives.
    """
    default_dof_pos = [0.0, 0.0, 0.0, -0.95, 0.0, 1.12, 0.0, 0.02, 0.02]
    revolute_drive_params = {
        "stiffness": 400.0,
        "damping": 40.0,
        "maxForce": 100.0,
    }

    # linear params:
    linear_drive_params = {
        "stiffness": 400.0,
        "damping": 40.0,
        "maxForce": 100.0,
    }

    drive_params = [revolute_drive_params] * 7
    drive_params.extend([linear_drive_params] * 2)

    return default_dof_pos, drive_params


class PoseMotion:
    """
    Describes a motion from a start pose to an end pose of the franka hand.

    Parameters:
        start_pos: 3-float array - start position
        end_pos: 3-float array - end position
        start_quat: 4-float array - start rotation quaternion
        end_quat: 4-float array - end rotation quaternion
        duration: float - duration of the motion from the start pose to the end pose
        end_delay: float - hold the pose fixed for end_delay time after reaching the end pose
        use_alignment: bool - toggles usage of `offset_target_position_to_align()`
    """

    def __init__(
        self,
        start_pos: np.array,
        end_pos: np.array,
        start_quat: np.array,
        end_quat: np.array,
        duration: float,
        end_delay: float = 0.0,
        use_alignment=False,
    ):
        self.start_pos = start_pos.copy()
        self.end_pos = end_pos.copy()
        self.start_quat = start_quat.copy()
        self.end_quat = end_quat.copy()
        assert duration > 0.0
        self.duration = duration
        self.acceleration = 4.0 * np.linalg.norm(self.end_pos - self.start_pos) / (duration**2.0)
        self.end_delay = end_delay
        self.use_alignment = use_alignment


class FrankaState:
    """
    Helps calculating a franka hand pose delta which is used in inverse kinematics.

    The pose delta is evaluated based on the current hand pose supplied to `advance()`
    and a pose motion object together with the current time.
    """

    def __init__(self):
        self.alignment_gain = -4.0
        self.pos_error_threshold = 0.0
        self.rot_error_threshold = 0.0
        self.pose_delta = np.zeros(6, dtype=np.float32)
        self.gripper_separation = 0.0
        self.pose_motion = None
        self.current_time = 0.0

    def update_pose_delta_from_target(self, target_pos, target_quat, hand_pose) -> tuple:
        """
        Computes pose delta from target position and orientation and current hand pose

        Inputs:
        target_pos:     target position     1x3 np array (x, y, z)
        target_quat:    target quaterion    1x4 np array (qx, qy, qz, qw)
        hand_pose:      hand pose           1x7 np array (x, y, z, qx, qy, qz, qw)
        """
        self.pose_delta[:3] = target_pos - hand_pose[:3]
        self.pose_delta[3:] = numpy_utils.orientation_error(target_quat, hand_pose[3:])
        return (np.linalg.norm(self.pose_delta[:3]), np.linalg.norm(self.pose_delta[3:]))

    def compute_target_pose(self) -> tuple:
        """
        Helper to evaluate timesampled trajectory.

        Returns: Tuple with
                    [0] target pose
                    [1] flag indicating end of motion time
        """
        if self.current_time > self.pose_motion.duration:
            return (np.concatenate((self.pose_motion.end_pos, self.pose_motion.end_quat)), True)

        motion_dir = self.pose_motion.end_pos - self.pose_motion.start_pos
        dist = np.linalg.norm(self.pose_motion.end_pos - self.pose_motion.start_pos)

        if dist < 0.001:
            return (np.concatenate((self.pose_motion.end_pos, self.pose_motion.end_quat)), True)

        if self.current_time > self.pose_motion.duration * 0.5:
            dt = self.current_time - self.pose_motion.duration * 0.5
            vmax = (self.pose_motion.duration * 0.5) * self.pose_motion.acceleration
            cur_dist = dist * 0.5 + dt * vmax - 0.5 * dt * dt * self.pose_motion.acceleration
        else:
            dt = self.current_time
            cur_dist = 0.5 * dt * dt * self.pose_motion.acceleration

        ratio = cur_dist / dist
        target_pos = self.pose_motion.start_pos + ratio * motion_dir
        target_quat = usdrt_Gf.Slerp(
            ratio, usdrt_Gf.Quatf(self.pose_motion.start_quat), usdrt_Gf.Quatf(self.pose_motion.end_quat)
        )

        return (np.concatenate((target_pos, target_quat)), False)

    def offset_target_position_to_align(self, target_position: np.array, hand_pose: np.array) -> np.array:
        """
        Offsets given target position in horizontal (x, y) position to achieve more precise positioning with
        Jacobian IK.

        Inputs:
        target_position:    Target position     1x3 np array (x, y, z)
        hand_pose:          Hand pose           1x7 np array (x, y, z, qx, qy, qz, qw)

        Returns: Tuple with
                    [0] Offset target position  1x3 np array (x, y, z)
                    [1] Horizontal error norm |target_pos[:2] - hand_pose[:2]|)
        """
        position_error = hand_pose[:3] - target_position
        # offset the target pos to get accurate positioning of the hand:
        offset = np.zeros(3, dtype=np.float32)
        offset[:2] = self.alignment_gain * position_error[:2]
        err_norm = np.linalg.norm(position_error[:2])
        return target_position + offset, err_norm

    def advance(self, hand_pose: np.array, dt: float):
        """
        Advances the franka hand along the pose motion trajectory.

        Input:
        hand_pose:  Current hand pose 1x7 np array (x, y, z, qx, qy, qz, qw)
        dt:         Time delta

        Returns:    True if motion including end delay is completed and the pos error is below a threshold, False otherwise
        """

        self.current_time += dt
        target_pose, at_goal = self.compute_target_pose()
        target_pos = target_pose[:3]
        target_quat = target_pose[3:]
        if self.pose_motion.use_alignment:
            target_pos, _ = self.offset_target_position_to_align(target_pos, hand_pose)
        pos_error, _ = self.update_pose_delta_from_target(target_pos, target_quat, hand_pose)

        return (
            at_goal
            and pos_error < self.pos_error_threshold
            and self.pose_motion.duration + self.pose_motion.end_delay < self.current_time
        )


class FrankaDemoBase(demo.AsyncDemoBase):
    """
    Boilerplate class to make a demo with a franka robot arm.
    """

    category = demo.Categories.NONE

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    params = {
        "Number_of_Robots": demo.IntParam(1, 1, 4, 1),
    }

    demo_camera = Sdf.Path("/World/Camera")

    def __init__(self, enable_fabric=False):
        super().__init__(enable_tensor_api=True, enable_fabric=enable_fabric)

        # SCENE GEOMETRY
        # env (group) spacing:
        self._env_spacing = 2.0

        self._reset_hydra_instancing_on_shutdown = False
        self._time = 0.0
        self._fsm_time = 0.0

        self._franka_translate = Gf.Vec3f(0.0, 0.0, 0.2096)  # location of the franka inside an environment

        self._camera_location_shift = Gf.Vec3f(0.0, 0.0, 0.0)  # shifts the camera from a location centered on all environments
        self._camera_target_shift = Gf.Vec3f(0.0, 0.0, 0.0)  # shifts the camera target away from the origin

        # inverse kinematics (ik)
        self._ik_damping = 0.1

        # sim options:
        self._gravity_magnitude = 9.81
        self._time_steps_per_second = 240
        self._sim_dt = 1.0 / self._time_steps_per_second

        self._franka_pos_iterations = 16
        self._franka_vel_iterations = 1
        self._solver_type = "TGS"

        self._gripper_static_friction = 1.0
        self._gripper_dynamic_friction = 1.0
        self._gripper_density = None
        self._gripper_material_precedence = UsdShade.Tokens.strongerThanDescendants

        self.asset_paths = None  # dict holding the paths of the franka and other usd assets

    def _generate_fsm(self, env_idx):
        """
        Should be used to generate a finite state machine (FSM) object in subclasses.

        This function is called during demo initialization in the create() function.

        Parameters:
            env_idx: index of the environment

        Returns:
            A finite state machine object which can be used in the on_physics_step() function
        """
        pass

    def _configure_drives(self):
        """
        Can be used to configure the franka drive parameters in subclasses.

        This function is called during demo initialization in the create() function.

        Returns:
            None
        """
        pass

    def create(self, stage, Number_of_Robots):
        self._stage = stage
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()
        self._num_envs = Number_of_Robots
        self._num_envs_per_side = max(1, round(math.sqrt(self._num_envs)))
        self._row_half_length = (self._num_envs_per_side - 1) * self._env_spacing * 0.5

        # setup stage sim and materials
        self._setup_stage()
        self._setup_camera()
        self._setup_simulation()
        self._setup_materials()

        # Franka parameters:
        self._default_dof_pos, self._drive_params = get_default_franka_parameters()

        self._configure_drives()

        self._fsms = []

        # create grid of frankas and tables:
        envsScopePath = self._defaultPrimPath.AppendPath("envs")
        UsdGeom.Scope.Define(stage, envsScopePath)
        for i in range(self._num_envs):
            # create env:
            col_number = i % self._num_envs_per_side
            row_number = i // self._num_envs_per_side
            x_pos = -self._row_half_length + col_number * self._env_spacing
            y_pos = -self._row_half_length + row_number * self._env_spacing
            env_path = envsScopePath.AppendChild(f"env_{i}")
            env_xform = UsdGeom.Xform.Define(stage, env_path)
            physicsUtils.set_or_add_translate_op(env_xform, translate=Gf.Vec3f(x_pos, y_pos, 0.0))

            self._add_franka(env_path=env_path)
            self._add_custom(env_path=env_path)

            self._fsms.append(self._generate_fsm(i))

        # dp and gripper sep tensors:
        self._d_pose = np.zeros((self._num_envs, 6), dtype=np.float32)
        self._grip_sep = np.zeros((self._num_envs, 1), dtype=np.float32)
        self._pos_action = np.zeros((self._num_envs, 9), dtype=np.float32)
        omni.usd.get_context().get_selection().set_selected_prim_paths([], False)

    def _setup_simulation(self):
        # Physics scene
        scene = UsdPhysics.Scene.Define(self._stage, self._defaultPrimPath.AppendChild("physicsScene"))

        # setup timing
        self._sim_dt = 1.0 / self._time_steps_per_second

        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(self._gravity_magnitude)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(self._time_steps_per_second)

        return physxSceneAPI

    def _setup_stage(self):
        # setup ground collision plane:
        utils.addPlaneCollider(self._stage, "/World/physicsGroundPlaneCollider", "Z")

        # scale and z-up
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)

    def _setup_camera(self):
        self._cam = UsdGeom.Camera.Define(self._stage, self.demo_camera)

        location = Gf.Vec3f(self._row_half_length, self._row_half_length, 0.0) + self._camera_location_shift
        target = Gf.Vec3f(0.0, 0.0, 0.0) + self._camera_target_shift

        delta = target - location
        rotZ = math.degrees(math.atan2(-delta[0], delta[1]))
        rotX = math.degrees(math.atan2(delta[2], math.sqrt(delta[0] ** 2.0 + delta[1] ** 2.0)))
        rotZQ = Gf.Quatf(Gf.Rotation(Gf.Vec3d([0, 0, 1]), rotZ).GetQuat())
        rotXQ = Gf.Quatf(Gf.Rotation(Gf.Vec3d([1, 0, 0]), rotX + 90).GetQuat())
        physicsUtils.setup_transform_as_scale_orient_translate(self._cam)
        physicsUtils.set_or_add_translate_op(self._cam, translate=location)
        physicsUtils.set_or_add_orient_op(self._cam, orient=rotZQ * rotXQ)
        self._cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100))

    def on_startup(self):
        sceneGraphInstancingEnabled = carb.settings.get_settings().get("/persistent/omnihydra/useSceneGraphInstancing")
        if not sceneGraphInstancingEnabled:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", True)
            self._reset_hydra_instancing_on_shutdown = True

    def on_tensor_start(self, tensorApi):
        sim = tensorApi.create_simulation_view("numpy")
        sim.set_subspace_roots("/World/envs/*")
        # franka view
        self._frankas_view = sim.create_articulation_view("/World/envs/*/franka")
        assert self._frankas_view.count == self._num_envs
        self._franka_indices = np.arange(self._frankas_view.count, dtype=np.int32)

        # end effector view
        self._hands = sim.create_rigid_body_view("/World/envs/*/franka/panda_hand")

        self._time = 0.0
        self._fsm_time = 0.0

        self._reset_frankas()

        # prevent garbage collector from deleting the sim 
        self.sim = sim

        return sim

    def on_shutdown(self):
        self._frankas_view = None
        self._hands = None
        self.sim = None

        # disable hydra instancing if it had to be enabled
        if self._reset_hydra_instancing_on_shutdown:
            carb.settings.get_settings().set("/persistent/omnihydra/useSceneGraphInstancing", False)
        super().on_shutdown()

    def _reset_frankas(self):
        reset_inds = self._franka_indices
        init_dof_pos = np.zeros((self._num_envs, 9), dtype=np.float32)
        for ind in reset_inds:
            self._fsms[ind].reset()
            init_dof_pos[ind, :] = self._default_dof_pos

        self._frankas_view.set_dof_positions(init_dof_pos, reset_inds)
        self._frankas_view.set_dof_velocities(np.zeros((self._num_envs, 9), dtype=np.float32), reset_inds)
        self._frankas_view.set_dof_position_targets(init_dof_pos, reset_inds)

    def _setup_materials(self):
        physicsMaterialScopePath = self._defaultPrimPath.AppendChild("PhysicsMaterials")
        UsdGeom.Scope.Define(self._stage, physicsMaterialScopePath)
        self._gripper_material_path = physicsMaterialScopePath.AppendChild("GripperMaterial")

        utils.addRigidBodyMaterial(
            self._stage,
            self._gripper_material_path,
            density=self._gripper_density,
            staticFriction=self._gripper_static_friction,
            dynamicFriction=self._gripper_dynamic_friction,
        )
        return physicsMaterialScopePath

    def _add_franka(self, env_path):
        franka_path = env_path.AppendChild("franka")
        assert self.asset_paths
        assert self.asset_paths["franka"]
        assert self._stage.DefinePrim(franka_path).GetReferences().AddReference(self.asset_paths["franka"], "/panda")
        franka_xform = UsdGeom.Xform.Get(self._stage, franka_path)
        assert franka_xform
        physicsUtils.set_or_add_translate_op(franka_xform, translate=self._franka_translate)
        physicsUtils.set_or_add_scale_op(franka_xform, scale=Gf.Vec3f(0.01))

        configure_franka_drives(self._stage, franka_path, self._default_dof_pos, self._drive_params)

        physxArticulationAPI = PhysxSchema.PhysxArticulationAPI.Apply(franka_xform.GetPrim())
        physxArticulationAPI.GetSolverPositionIterationCountAttr().Set(self._franka_pos_iterations)
        physxArticulationAPI.GetSolverVelocityIterationCountAttr().Set(self._franka_vel_iterations)

        # setup fingers
        finger_material = UsdShade.Material.Get(self._stage, self._gripper_material_path)
        prim = self._stage.GetPrimAtPath(env_path.AppendPath("franka/panda_leftfinger/geometry"))
        bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
        bindingAPI.Bind(finger_material, self._gripper_material_precedence, "physics")
        prim = self._stage.GetPrimAtPath(env_path.AppendPath("franka/panda_rightfinger/geometry"))
        bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
        bindingAPI.Bind(finger_material, self._gripper_material_precedence, "physics")

    def _add_custom(self, env_path):
        """
        Can be used to add USD objects to the scene in subclasses.

        This function is called during demo initialization in the create() function.

        Parameters:
            env_path: The USD path to the current environment

        Returns:
            None
        """
        pass

    def on_physics_step(self, dt):
        """
        Should be used to update the finite state machines and set the franka drive targets in subclasses.

        This function is called after every simulation step.

        Parameters:
            dt: The physics step time delta

        Returns:
            None
        """
        pass

    def _run_ik(self, direct_z_rotation=False):
        """
        Computes and sets joint drive targets using inverse kinematics.

        Parameters:
            direct_z_rotation: bool - directly set the hand rotation dof based on the rotation error in Z

        Returns:
            None
        """

        dof_pos = self._frankas_view.get_dof_positions()
        # get franka jacobians
        jacobians = self._frankas_view.get_jacobians()
        # jacobian entries corresponding to franka hand
        franka_hand_index = 8  # !!!
        range_extension = 0 if direct_z_rotation else 1
        j_eef = jacobians[:, franka_hand_index - 1, : (5 + range_extension), : (6 + range_extension)]
        j_eef_T = np.transpose(j_eef, (0, 2, 1))
        lmbda = np.eye(5 + range_extension) * (self._ik_damping**2)
        u = (
            j_eef_T
            @ np.linalg.inv(j_eef @ j_eef_T + lmbda)
            @ self._d_pose[:, : (5 + range_extension)].reshape(self._num_envs, 5 + range_extension, 1)
        ).reshape(self._num_envs, 6 + range_extension)
        self._pos_action[:, : (6 + range_extension)] = dof_pos[:, : (6 + range_extension)] + u

        grip_acts = np.concatenate((0.5 * self._grip_sep, 0.5 * self._grip_sep), 1)
        self._pos_action[:, 7:9] = grip_acts

        if direct_z_rotation:
            # directly set the hand rotation dof based on the rotation error in Z
            self._pos_action[:, 6] = dof_pos[:, 6] - self._d_pose[:, 5]

        # apply position targets
        self._frankas_view.set_dof_position_targets(self._pos_action, self._franka_indices)
