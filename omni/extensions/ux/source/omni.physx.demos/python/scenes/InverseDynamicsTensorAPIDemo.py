# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import numpy as np
import math

from pxr import Gf, UsdGeom, PhysxSchema, UsdPhysics
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils

from omni.physxdemos.utils import gripper_helpers
from omni.physxdemos.utils import numpy_utils

class GripperInverseDynamicTensorAPIDemo(gripper_helpers.GripperDemoBase):
    title = "Gripper inverse dynamics"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Gripper picking and moving a box using inverse dynamics functions"
    description = (
        "Demo showing how to use inverse dynamics functions to apply a given trajectory to an articulation."
    )

    def __init__(self):
        super().__init__(enable_fabric=False)

        # gripper param
        self._state = "Opening gripper"
        self._max_root_velocity = 0.1
        self._max_finger_velocity = 0.1
        self._gripper_open_pose = -0.35
        self._gripper_close_pose = 0.0
        self._gripper_top_position = 0.5
        self._gripper_bottom_position = 0.35
        self._gripper_initial_lateral_position = 0.0
        self._gripper_final_lateral_position = 0.3
        self._gripper_depth_position = 0.2
        self._prevPosGripper = 10.0 # arbitrary large value

    def on_startup(self):
        pass

    def create(self, stage):
        # setup the gripper
        super().create(stage=stage)

        # add a box
        boxSize = 0.1
        boxColor = demo.get_primary_color()
        boxPosition = Gf.Vec3f(0.2, 0.05, 0.0)
        defaultPrimPath = stage.GetDefaultPrim().GetPath()
        boxActorPath = defaultPrimPath.AppendChild("boxActor")
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(boxSize)
        half_extent = boxSize / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        translateOp = cubeGeom.AddTranslateOp() 
        translateOp.Set(boxPosition)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0)) 
        cubeGeom.CreateDisplayColorAttr().Set([boxColor])
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(boxActorPath))
        massAPI = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(boxActorPath))
        massAPI.CreateMassAttr(0.005)

    def on_tensor_start(self, tensorApi):
        sim = tensorApi.create_simulation_view("numpy")
        self._boxActor = sim.create_rigid_body_view("/World/boxActor")
        self._gripper = sim.create_articulation_view("/World/gripper/root")
        self._gripper_indices = np.arange(1, dtype=np.int32)

        # prevent garbage collector from deleting the sim 
        self._sim = sim

        return sim

    def on_shutdown(self):
        self._boxActor = None
        self._gripper = None
        self._sim = None
        super().on_shutdown()

    def on_physics_step(self, dt):
        posGripper = self._gripper.get_dof_positions()
        posRootGripper = self._gripper.get_root_transforms()

        if self._state == "Opening gripper":
            # calculate desired acceleration
            expRootPose = np.array([self._gripper_depth_position, self._gripper_top_position, self._gripper_initial_lateral_position])
            expectedJointAccel, expectedRootAccel = self._calculate_desired_acceleration(dt, self._gripper_open_pose, expRootPose)

            # apply joint force and root force required to achieve desired acceleration
            self._apply_inverse_dynamics(expectedJointAccel, expectedRootAccel)

            # check if this stage is finished
            if np.abs(self._gripper_open_pose - posGripper[0][0]) < 0.001:
                self._state = "Moving down"

        elif self._state == "Moving down":
            # calculate desired acceleration
            expRootPose = np.array([self._gripper_depth_position, self._gripper_bottom_position, self._gripper_initial_lateral_position])
            expectedJointAccel, expectedRootAccel = self._calculate_desired_acceleration(dt, self._gripper_open_pose, expRootPose)

            # apply joint force and root force required to achieve desired acceleration
            self._apply_inverse_dynamics(expectedJointAccel, expectedRootAccel)

            # check if this stage is finished
            if np.abs(self._gripper_bottom_position - posRootGripper[0][1]) < 0.001:
                self._state = "Grasping"

        if self._state == "Grasping":
            # calculate desired acceleration
            expRootPose = np.array([self._gripper_depth_position, self._gripper_bottom_position, self._gripper_initial_lateral_position])
            expectedJointAccel, expectedRootAccel = self._calculate_desired_acceleration(dt, self._gripper_close_pose, expRootPose)

            # apply joint force and root force required to achieve desired acceleration
            self._apply_inverse_dynamics(expectedJointAccel, expectedRootAccel)

            # check if this stage is finished (when the gripper is not moving anymore)
            if np.abs(self._prevPosGripper - posGripper[0][0]) < 0.001:
                self._state = "Moving up"

            self._prevPosGripper = posGripper[0][0]

        elif self._state == "Moving up":
            # calculate desired acceleration
            expRootPose = np.array([self._gripper_depth_position, self._gripper_top_position, self._gripper_initial_lateral_position])
            expectedJointAccel, expectedRootAccel = self._calculate_desired_acceleration(dt, self._gripper_close_pose, expRootPose)

            # apply joint force and root force required to achieve desired acceleration
            self._apply_inverse_dynamics(expectedJointAccel, expectedRootAccel)

            # check if this stage is finished
            if np.abs(self._gripper_top_position - posRootGripper[0][1]) < 0.001:
                self._state = "Moving laterally"

        elif self._state == "Moving laterally":
            # calculate desired acceleration
            expRootPose = np.array([self._gripper_depth_position, self._gripper_top_position, self._gripper_final_lateral_position])
            expectedJointAccel, expectedRootAccel = self._calculate_desired_acceleration(dt, self._gripper_close_pose, expRootPose)

            # apply joint force and root force required to achieve desired acceleration
            self._apply_inverse_dynamics(expectedJointAccel, expectedRootAccel)

            # check if this stage is finished
            if np.abs(self._gripper_final_lateral_position - posRootGripper[0][2]) < 0.001:
                self._state = "Dropping"

        elif self._state == "Dropping":
            # calculate desired acceleration
            expRootPose = np.array([self._gripper_depth_position, self._gripper_top_position, self._gripper_final_lateral_position])
            expectedJointAccel, expectedRootAccel = self._calculate_desired_acceleration(dt, self._gripper_open_pose, expRootPose)

            # apply joint force and root force required to achieve desired acceleration
            self._apply_inverse_dynamics(expectedJointAccel, expectedRootAccel)

        super().on_physics_step(dt)

    def _calculate_desired_acceleration(self, dt, expFingerPos, expRootPose):
        # get position and velocity of the gripper
        posGripper = self._gripper.get_dof_positions()
        velGripper = self._gripper.get_dof_velocities()
        posRootGripper = self._gripper.get_root_transforms()
        velRootGripper = self._gripper.get_root_velocities()

        # calculate velocity to achieve desired pose
        predFingerVelocity = (expFingerPos - posGripper[0][0]) / dt
        predRootVelocity = (expRootPose[0:3] - posRootGripper[0][0:3]) / dt

        # clamp velocity to maximum allowed velocity
        predFingerVelocity = np.sign(predFingerVelocity) * np.minimum(np.abs(predFingerVelocity), self._max_finger_velocity)
        predRootVelocity = np.sign(predRootVelocity) * np.minimum(np.abs(predRootVelocity), self._max_root_velocity)

        # correct root angular position
        rootOrientation = np.array([0.0, 0.0, -0.7071068, 0.7071068])
        qErr = numpy_utils.quat_mul(rootOrientation, numpy_utils.quat_conjugate(posRootGripper[0][3:]))
        predRootRotXVelocity = math.atan2(2 * (qErr[3] * qErr[0] + qErr[1] * qErr[2]), 1 - 2 * (qErr[0]**2 + qErr[1]**2)) / dt
        predRootRotYVelocity = math.asin(2 * (qErr[3] * qErr[1] - qErr[2]* qErr[0])) / dt
        predRootRotZVelocity = math.atan2(2 * (qErr[3] * qErr[2] + qErr[0] * qErr[1]), 1 - 2 * (qErr[1]**2 + qErr[2]**2)) / dt

        # calculate desired acceleration
        predFingerAccel = (predFingerVelocity - velGripper[0][0]) / dt
        predRootAccel = (predRootVelocity - velRootGripper[0][0:3]) / dt
        predRootRotXAccel = (predRootRotXVelocity - velRootGripper[0][3]) / dt
        predRootRotYAccel = (predRootRotYVelocity - velRootGripper[0][4]) / dt
        predRootRotZAccel = (predRootRotZVelocity - velRootGripper[0][5]) / dt

        return [predFingerAccel, -predFingerAccel], [predRootAccel[0], predRootAccel[1], predRootAccel[2], predRootRotXAccel, predRootRotYAccel, predRootRotZAccel]

    def _apply_inverse_dynamics(self, expectedJointAccel, expectedRootAccel):
        rootDofs = 6 # Degree of freedoms of the root: 0 for fixed-base articulation, 6 for floaitng-base articulations
        coriolisCompensation = self._gripper.get_coriolis_and_centrifugal_compensation_forces()
        gravityCompensation = self._gripper.get_gravity_compensation_forces()
        massMatrixReshape = self._gripper.get_generalized_mass_matrices().reshape(self._gripper.max_dofs + rootDofs,self._gripper.max_dofs + rootDofs)

        # calculate force to achieve given acceleration
        requiredJointForce = np.matmul(massMatrixReshape[rootDofs:, rootDofs:], expectedJointAccel)
        requiredJointForce += np.matmul(massMatrixReshape[rootDofs:, :rootDofs], expectedRootAccel)
        requiredRootForce = np.matmul(massMatrixReshape[:rootDofs, rootDofs:], expectedJointAccel)
        requiredRootForce += np.matmul(massMatrixReshape[:rootDofs, :rootDofs], expectedRootAccel)

        # add gravity and Coriolis compensation force
        requiredJointForce += coriolisCompensation[0][rootDofs:] + gravityCompensation[0][rootDofs:]
        requiredRootForce += coriolisCompensation[0][:rootDofs] + gravityCompensation[0][:rootDofs]

        # apply required joint force
        self._gripper.set_dof_actuation_forces(requiredJointForce, self._gripper_indices)

        # apply required root force
        forces = np.zeros(self._gripper.max_links * 3)
        torques = np.zeros(self._gripper.max_links * 3)
        forces[0:3] = requiredRootForce[0:3]
        torques[0:3] = requiredRootForce[3:6]
        self._gripper.apply_forces_and_torques_at_position(forces, torques, None, self._gripper_indices, True)
