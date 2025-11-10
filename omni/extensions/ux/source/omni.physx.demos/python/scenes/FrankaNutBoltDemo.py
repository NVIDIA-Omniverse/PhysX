# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import math
from pxr import Gf, Sdf, UsdGeom, Usd, PhysxSchema, UsdPhysics
import omni.physxdemos as demo
from omni.physx.scripts import utils, physicsUtils
import numpy as np
from omni.physx.scripts.assets_paths import AssetFolders
from omni.physxdemos.utils import franka_helpers
from omni.physxdemos.utils import numpy_utils


class FrankaNutBoltDemo(franka_helpers.FrankaDemoBase):
    title = "Franka Nut and Bolt"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Franka robot arms picking and screwing nuts onto bolts"
    description = (
        "The nut and bolt interaction is a demonstration of contact-rich signed-distance-field collision in PhysX. "
        "The nuts are fed with a vibratory feeder to the robot. The arm is controlled with a finite state machine "
        "and Jacobian-based inverse kinematics. You can configure the demo parameters below. "
    )

    params = {
        "Number_of_Robots": demo.IntParam(4, 1, 16, 1),
        "Number_of_Nuts": demo.IntParam(24, 1, 24, 1),
    }

    def __init__(self):
        super().__init__(enable_fabric=True)

        # franka
        self._stool_height = 0.15

        # table and vibra table:
        self._table_position = Gf.Vec3f(0.5, 0.0, 0.0)
        self._table_scale = 0.01
        self._table_height = 0.8393425908831669
        self._franka_translate = Gf.Vec3f(0.269, 0.1778, self._table_height)
        self._tooling_plate_offset = Gf.Vec3f(0.0, 0.0, 0.0)
        self._vibra_table_position_offset = Gf.Vec3f(0.157, -0.1524, 0.0)
        self._vibra_top_offset = Gf.Vec3f(0.0, 0.0, 0.15)
        self._vibra_table_top_to_collider_offset = Gf.Vec3f(0.05, 2.5, -0.59) * 0.01
        # xyz relative to the vibra table where the nut should be picked up
        self._vibra_table_nut_pickup_pos_offset = Gf.Vec3f(0.124, 0.24, 0.158)

        # nut
        self._nut_height = 0.016
        self._nut_spiral_center_vibra_offset = Gf.Vec3f(-0.04, -0.17, 0.01)
        # randomize initial nut and bolt positions
        self._nut_radius = 0.055
        self._nut_height_delta = 0.03
        self._nut_dist_delta = 0.03
        self._mass_nut = 0.065  # kg - needed because tri-mesh mass computation is not yet fully supported

        # pipe and bolt parameters
        self._bolt_length = 0.1
        self._num_bolts = 6
        self._bolt_radius = 0.11
        self._pipe_pos_on_table = Gf.Vec3f(0.2032, 0.381, 0.0)
        self._bolt_z_offset_to_pipe = 0.08

        # camera
        cam_height = 1.5
        self._camera_location_shift = Gf.Vec3f(3.2, 1.5, cam_height)
        self._camera_target_shift = Gf.Vec3f(0.0, 0.0, cam_height - 0.4)

        # randomization
        self._randomize_nut_positions = True
        self._nut_position_noise_minmax = 0.005
        self._rng_seed = 8
        self._rng = np.random.default_rng(self._rng_seed)

        # states
        self._time = 0.0
        self._fsm_time = 0.0
        self._fsm_dt = 1.0 / 60.0

        # some global sim options:
        self._time_steps_per_second = 240
        self._fsm_update_rate = 60
        self._franka_pos_iterations = 4
        self._pos_iterations = 4
        self._vel_iterations = 1

        self._gravity_magnitude = 9.807

        # gripper material
        self._gripper_density = 1000.0

        # setup asset paths:
        self.asset_paths = {
            "franka": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Franka/franka_alt_fingers.usd"),
            "shop_table": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Shop_Table/Shop_Table.usd"),
            "tooling_plate": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Tooling_Plate/Tooling_Plate.usd"),
            "nut": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Nut/M20_Nut_Tight_R256_Franka_SI_Sparse.usd"),
            "bolt": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Bolt/M20_Bolt_Tight_R512_Franka_SI_Sparse.usd"),
            "vibra_table_top": demo.get_demo_asset_path(
                AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/VibrationTable_Top/VibrationTable_Top.usd"
            ),
            "vibra_table_bot": demo.get_demo_asset_path(
                AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/VibrationTable_Base/VibrationTable_Base.usd"
            ),
            "vibra_table_collision": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/VibrationTable_Top_collision.usd"),
            "vibra_table_clamps": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Clamps/Clamps.usd"),
            "pipe": demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "SubUSDs/Pipe/Pipe.usd"),
        }
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.FRANKA_NUT_BOLT, "StagingNutBolt.usd")

    def _configure_drives(self):
        # screw DOF
        self._drive_params[6]["stiffness"] = 200.0
        self._drive_params[6]["damping"] = 40.0
        # gripper fingers
        self._drive_params[7]["stiffness"] = 800.0
        self._drive_params[7]["damping"] = 40.0
        self._drive_params[8]["stiffness"] = 800.0
        self._drive_params[8]["damping"] = 40.0

    def create(self, stage, Number_of_Robots, Number_of_Nuts):
        self._num_nuts = Number_of_Nuts
        super().create(stage=stage, Number_of_Robots=Number_of_Robots)

    def _add_custom(self, env_path):
        self._add_table(env_path)
        self._add_vibra_table(env_path)
        self._add_nuts_and_bolt(env_path, add_debug_nut=self._num_nuts == 2)

    def _add_franka(self, env_path):
        # add franka
        franka_path = env_path.AppendChild("franka")
        self._stage.DefinePrim(franka_path).GetReferences().AddReference(self._get_asset_path("franka"), "/panda")
        franka_xform = UsdGeom.Xform.Get(self._stage, franka_path)
        assert franka_xform
        physicsUtils.set_or_add_translate_op(franka_xform, translate=self._franka_translate)
        physicsUtils.set_or_add_scale_op(franka_xform, scale=Gf.Vec3f(0.01))
        franka_helpers.configure_franka_drives(self._stage, franka_path, self._default_dof_pos, self._drive_params)
        # add fingers to convex group:
        leftFingerPath = franka_path.AppendChild("panda_leftfinger")
        rightFingerPath = franka_path.AppendChild("panda_rightfinger")
        self._convexIncludeRel.AddTarget(leftFingerPath)
        self._convexIncludeRel.AddTarget(rightFingerPath)
        leftFingerGraspPrim = self._stage.GetPrimAtPath(leftFingerPath.AppendPath("geometry/ID27"))
        rightFingerGraspPrim = self._stage.GetPrimAtPath(rightFingerPath.AppendPath("geometry/ID27"))
        physicsUtils.add_physics_material_to_prim(self._stage, leftFingerGraspPrim, self._gripper_material_path)
        physicsUtils.add_physics_material_to_prim(self._stage, rightFingerGraspPrim, self._gripper_material_path)
        # set iterations:
        physxArticulationAPI = PhysxSchema.PhysxArticulationAPI.Apply(franka_xform.GetPrim())
        physxArticulationAPI.GetSolverPositionIterationCountAttr().Set(self._franka_pos_iterations)
        physxArticulationAPI.GetSolverVelocityIterationCountAttr().Set(self._franka_vel_iterations)

    def _generate_fsm(self, env_idx):
        return ScrewFSM(
            vibra_dt=self._sim_dt,
            nut_height=self._nut_height,
            bolt_height=self._bolt_length,
            table_height=self._table_height,
            pipe_height=self._pipe_height,
            vibra_table_nut_pickup_pos=self._vibra_table_nut_pickup_pos_offset + self._vibra_table_position,
            env_idx=env_idx,
            num_nuts=self._num_nuts,
            num_bolts=self._num_bolts,
        )

    def _get_asset_path(self, asset: str):
        return self.asset_paths[asset]

    def _setup_simulation(self):
        physxSceneAPI = super()._setup_simulation()

        self._fsm_update_dt = 1.0 / self._fsm_update_rate

        # friction
        physxSceneAPI.CreateFrictionOffsetThresholdAttr().Set(0.01)
        physxSceneAPI.CreateFrictionCorrelationDistanceAttr().Set(0.0005)

        # GPU
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(10 * 1024)

        # prep collision group setup:
        cgScopePath = self._defaultPrimPath.AppendChild("collisionGroups")
        meshColliderGroupPath = cgScopePath.AppendChild("meshColliders")
        convexColliderGroupPath = cgScopePath.AppendChild("convexColliders")
        boltColliderGroupPath = cgScopePath.AppendChild("boltColliders")
        meshCollisionGroup = UsdPhysics.CollisionGroup.Define(self._stage, meshColliderGroupPath)
        collectionAPI = Usd.CollectionAPI.Apply(meshCollisionGroup.GetPrim(), "colliders")
        self._nutMeshIncludeRel = collectionAPI.CreateIncludesRel()
        convexCollisionGroup = UsdPhysics.CollisionGroup.Define(self._stage, convexColliderGroupPath)
        collectionAPI = Usd.CollectionAPI.Apply(convexCollisionGroup.GetPrim(), "colliders")
        self._convexIncludeRel = collectionAPI.CreateIncludesRel()
        boltCollisionGroup = UsdPhysics.CollisionGroup.Define(self._stage, boltColliderGroupPath)
        collectionAPI = Usd.CollectionAPI.Apply(boltCollisionGroup.GetPrim(), "colliders")
        self._boltMeshIncludeRel = collectionAPI.CreateIncludesRel()

        # invert group logic so only groups that filter each-other will collide:
        physxSceneAPI.CreateInvertCollisionGroupFilterAttr().Set(True)

        # the SDF mesh collider nuts should only collide with the bolts
        filteredRel = meshCollisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget(boltColliderGroupPath)

        # the convex hull nuts should collide with each other, the vibra table, and the grippers
        filteredRel = convexCollisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget(convexColliderGroupPath)

        # the SDF mesh bolt only collides with the SDF mesh nut colliders
        filteredRel = boltCollisionGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget(meshColliderGroupPath)

    def on_tensor_start(self, tensorApi):
        sim = super().on_tensor_start(tensorApi=tensorApi)

        self._nuts = sim.create_rigid_body_view("/World/envs/*/nut*")
        self._bolts = sim.create_rigid_body_view("/World/envs/*/bolt*")
        self._vibra_tables = sim.create_rigid_body_view("/World/envs/*/vibra_table")

        # vibra table states:
        self._vibra_table_transforms = np.stack(
            self._num_envs * [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32)]
        )
        self._vibra_table_position_np = np.array(
            [self._vibra_table_position[0], self._vibra_table_position[1], self._vibra_table_position[2]],
            dtype=np.float32,
        )
        self._vibra_table_indices = np.arange(self._vibra_tables.count, dtype=np.int32)

        # downward direction
        self._down_dir = np.array([0, 0, -1]).reshape(3, 1)

        # hand orientation for grasping
        self._down_q = np.stack(self._num_envs * [np.array([1.0, 0.0, 0.0, 0.0])])

        self._reset_nuts()

    def on_shutdown(self):
        self._bolts = None
        self._nuts = None
        self._vibra_tables = None
        super().on_shutdown()

    def _reset_nuts(self):
        num_reset_envs = len(self._franka_indices)
        nut_inds = np.zeros(num_reset_envs * self._num_nuts, dtype=np.int32)
        nut_transforms = np.zeros((self._num_envs * self._num_nuts, 7), dtype=np.float32)
        for env_no in range(num_reset_envs):
            startInd = self._franka_indices[env_no] * self._num_nuts
            nut_inds[env_no * self._num_nuts:(env_no + 1) * self._num_nuts] = np.arange(
                startInd, startInd + self._num_nuts
            )
            nut_transforms[startInd:startInd + self._num_nuts, :] = self._nut_init_poses
        self._nuts.set_transforms(nut_transforms, nut_inds)

    def _add_table(self, env_path: Sdf.Path):
        table_path = env_path.AppendPath("table")
        self._stage.DefinePrim(table_path).GetReferences().AddReference(self._get_asset_path("shop_table"))
        xform = UsdGeom.Xform.Get(self._stage, table_path)
        assert xform

        mesh_xform = UsdGeom.Xformable.Get(self._stage, table_path.AppendChild("shop_table"))
        assert mesh_xform
        # scale first to get scaled bounds
        physicsUtils.set_or_add_scale_op(mesh_xform, scale=Gf.Vec3f(self._table_scale))
        lb = self._get_local_bounds(xform)
        zmin = lb.GetMin()[2]
        zmax = lb.GetMax()[2]
        self._table_position[2] = -zmin
        self._table_height = zmax

        physicsUtils.set_or_add_translate_op(xform, translate=self._table_position)
        utils.setStaticCollider(xform.GetPrim(), approximationShape="none")
        self._convexIncludeRel.AddTarget(table_path)

        # add tooling plate:
        tooling_path = table_path.AppendChild("tooling_plate")
        self._stage.DefinePrim(tooling_path).GetReferences().AddReference(self._get_asset_path("tooling_plate"))
        xform = UsdGeom.Xform.Get(self._stage, tooling_path)
        assert xform
        mesh_xform = UsdGeom.Xformable.Get(self._stage, tooling_path.AppendChild("tooling_plate"))
        assert mesh_xform
        # scale first to get scaled bounds
        physicsUtils.set_or_add_scale_op(mesh_xform, scale=Gf.Vec3f(self._table_scale))
        lb = self._get_local_bounds(xform)
        zmin = lb.GetMin()[2]
        zmax = lb.GetMax()[2]
        tooling_transform = self._tooling_plate_offset
        tooling_transform[2] = -zmin + self._table_height
        physicsUtils.set_or_add_translate_op(xform, translate=tooling_transform)
        utils.setStaticCollider(mesh_xform.GetPrim(), approximationShape="boundingCube")
        # adjust table height to include the tooling plate
        self._table_height += zmax - zmin
        self._convexIncludeRel.AddTarget(tooling_path)

        # add pipe:
        pipe_path = table_path.AppendChild("pipe")
        self._stage.DefinePrim(pipe_path).GetReferences().AddReference(self._get_asset_path("pipe"))
        xform = UsdGeom.Xform.Get(self._stage, pipe_path)
        assert xform
        mesh_xform = UsdGeom.Xformable.Get(self._stage, pipe_path.AppendChild("Pipe1"))
        assert mesh_xform
        # scale first to get scaled bounds
        physicsUtils.set_or_add_scale_op(mesh_xform, scale=Gf.Vec3f(self._table_scale))
        lb = self._get_local_bounds(xform)
        zmin = lb.GetMin()[2]
        zmax = lb.GetMax()[2]
        self._pipe_height = zmax - zmin
        pipe_transform = self._pipe_pos_on_table
        pipe_transform[2] = -zmin + self._table_height
        physicsUtils.set_or_add_translate_op(xform, translate=pipe_transform)
        physicsUtils.set_or_add_orient_op(xform, orient=Gf.Quatf(0, 0, 0, 1))
        utils.setStaticCollider(mesh_xform.GetPrim(), approximationShape="none")
        self._convexIncludeRel.AddTarget(pipe_path)

    def _setup_materials(self):
        physicsMaterialScopePath = super()._setup_materials()

        nutBoltDensity = 7850.0  # kg / m3

        self._boltPhysicsMaterialPath = physicsMaterialScopePath.AppendChild("BoltMaterial")
        self._nutPhysicsMaterialPath = physicsMaterialScopePath.AppendChild("NutMaterial")
        self._vibraTablePhysicsMaterialPath = physicsMaterialScopePath.AppendChild("VibraTableMaterial")
        utils.addRigidBodyMaterial(
            self._stage,
            self._nutPhysicsMaterialPath,
            density=nutBoltDensity * 0.5,  # has two overlapping collision shapes so half density = proper weight
            staticFriction=0.2,
            dynamicFriction=0.2,
        )
        utils.addRigidBodyMaterial(
            self._stage,
            self._boltPhysicsMaterialPath,
            density=nutBoltDensity,
            staticFriction=0.4,
            dynamicFriction=0.4,
        )
        utils.addRigidBodyMaterial(
            self._stage,
            self._vibraTablePhysicsMaterialPath,
            density=1000.0,
            staticFriction=0.2,
            dynamicFriction=0.2,
        )

    def _add_vibra_table(self, env_path: Sdf.Path):
        # add bottom static part to table xform:
        table_path = env_path.AppendChild("table")
        bottom_part_path = table_path.AppendChild("vibra_table_bottom")
        self._stage.DefinePrim(bottom_part_path).GetReferences().AddReference(self._get_asset_path("vibra_table_bot"))
        xform = UsdGeom.Xform.Get(self._stage, bottom_part_path)
        assert xform
        mesh_xform = UsdGeom.Xformable.Get(self._stage, bottom_part_path.AppendChild("vibration_table_base"))
        assert mesh_xform
        # scale first to get scaled bounds
        physicsUtils.set_or_add_scale_op(mesh_xform, scale=Gf.Vec3f(self._table_scale))
        lb = self._get_local_bounds(xform)
        zmin = lb.GetMin()[2]
        bot_part_pos = self._vibra_table_position_offset
        bot_part_pos[2] = -zmin + self._table_height
        physicsUtils.set_or_add_translate_op(xform, translate=bot_part_pos)
        utils.setStaticCollider(xform.GetPrim(), approximationShape="none")
        self._convexIncludeRel.AddTarget(bottom_part_path)

        # add clamps:
        table_path = env_path.AppendChild("table")
        clamps_path = table_path.AppendChild("vibra_table_clamps")
        self._stage.DefinePrim(clamps_path).GetReferences().AddReference(self._get_asset_path("vibra_table_clamps"))
        xform = UsdGeom.Xform.Get(self._stage, clamps_path)
        assert xform
        physicsUtils.set_or_add_translate_op(xform, translate=-self._table_position)
        utils.setStaticCollider(xform.GetPrim(), approximationShape="none")
        self._convexIncludeRel.AddTarget(clamps_path)

        # now add top which will be the kinematic actor:
        vibra_table_path = env_path.AppendChild("vibra_table")
        xform = UsdGeom.Xform.Define(self._stage, vibra_table_path)
        vibra_kinematic_prim = xform.GetPrim()
        self._vibra_table_position = bot_part_pos + self._table_position
        physicsUtils.set_or_add_translate_op(xform, translate=self._vibra_table_position)
        physicsUtils.set_or_add_orient_op(xform, orient=Gf.Quatf(0, 0, 0, 1))
        rbApi = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        rbApi.CreateRigidBodyEnabledAttr(True)
        rbApi.CreateKinematicEnabledAttr(True)

        top_viz_path = vibra_table_path.AppendChild("visual")
        self._stage.DefinePrim(top_viz_path).GetReferences().AddReference(self._get_asset_path("vibra_table_top"))
        xform = UsdGeom.Xform.Get(self._stage, top_viz_path)
        assert xform
        physicsUtils.set_or_add_translate_op(xform, translate=self._vibra_top_offset)
        physicsUtils.set_or_add_scale_op(xform, scale=Gf.Vec3f(self._table_scale))
        # add collision geometry:
        top_col_path = vibra_table_path.AppendChild("collision")
        self._stage.DefinePrim(top_col_path).GetReferences().AddReference(
            self._get_asset_path("vibra_table_collision"), "/nut_vibration_table/Table"
        )
        xform = UsdGeom.Xform.Get(self._stage, top_col_path)
        assert xform
        offset = self._vibra_top_offset + self._vibra_table_top_to_collider_offset
        physicsUtils.set_or_add_translate_op(xform, translate=offset)
        physicsUtils.add_physics_material_to_prim(self._stage, xform.GetPrim(), self._vibraTablePhysicsMaterialPath)
        self._convexIncludeRel.AddTarget(top_col_path)
        utils.setColliderSubtree(xform.GetPrim())
        vibra_kinematic_prim.SetInstanceable(True)

    def _get_uniform_random_number(self, symmetric_limit) -> float:
        return self._rng.uniform(-symmetric_limit, symmetric_limit)

    def _add_nuts_and_bolt(self, env_path, add_debug_nut=False):
        # create bolts:
        angle_delta = math.pi * 2.0 / self._num_bolts
        for i in range(self._num_bolts):
            bolt_path = env_path.AppendChild(f"bolt{i}")
            self._stage.DefinePrim(bolt_path).GetReferences().AddReference(self._get_asset_path("bolt"))
            xform = UsdGeom.Xform.Get(self._stage, bolt_path)
            xform.GetPrim().SetInstanceable(True)
            assert xform
            bolt_pos = Gf.Vec3f(self._pipe_pos_on_table) + self._table_position
            bolt_pos[0] += math.cos(i * angle_delta) * self._bolt_radius
            bolt_pos[1] += math.sin(i * angle_delta) * self._bolt_radius
            bolt_pos[2] = self._bolt_z_offset_to_pipe + self._table_height
            physicsUtils.set_or_add_translate_op(xform, translate=bolt_pos)
            self._boltMeshIncludeRel.AddTarget(bolt_path)
            physicsUtils.add_physics_material_to_prim(self._stage, xform.GetPrim(), self._boltPhysicsMaterialPath)

        self._generate_nut_initial_poses()
        for nut_idx in range(self._num_nuts):
            nut_path = env_path.AppendChild(f"nut{nut_idx}")
            nut_pos = self._nut_init_poses[nut_idx, :3].copy()
            if add_debug_nut and nut_idx == 0:
                nut_pos[0] = 0.78
                nut_pos[1] = 0.0264
            if add_debug_nut and nut_idx == 1:
                nut_pos[0] = 0.78
                nut_pos[1] = 0.0264 - 0.04
            self._stage.DefinePrim(nut_path).GetReferences().AddReference(self._get_asset_path("nut"))
            xform = UsdGeom.Xform.Get(self._stage, nut_path)
            xform.GetPrim().SetInstanceable(True)
            assert xform
            physicsUtils.set_or_add_translate_op(xform, translate=Gf.Vec3f(nut_pos.tolist()))
            self._convexIncludeRel.AddTarget(nut_path.AppendChild("M20_Nut_Tight_Convex"))
            self._nutMeshIncludeRel.AddTarget(nut_path.AppendChild("M20_Nut_Tight_SDF"))
            self._setup_nut_rb_params(nut_path)
            physicsUtils.add_physics_material_to_prim(self._stage, xform.GetPrim(), self._nutPhysicsMaterialPath)
            massAPI = UsdPhysics.MassAPI.Apply(xform.GetPrim())
            massAPI.CreateMassAttr().Set(self._mass_nut)

    def _setup_nut_rb_params(self, path):
        prim = self._stage.GetPrimAtPath(path)
        physxRBAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physxRBAPI.CreateSolverPositionIterationCountAttr().Set(self._pos_iterations)
        physxRBAPI.CreateSolverVelocityIterationCountAttr().Set(self._vel_iterations)

    def _generate_nut_initial_poses(self):
        self._nut_init_poses = np.zeros((self._num_nuts, 7), dtype=np.float32)
        self._nut_init_poses[:, -1] = 1  # quat to identity
        nut_spiral_center = self._vibra_table_position + self._nut_spiral_center_vibra_offset
        nut_spiral_center += self._vibra_top_offset
        for nut_idx in range(self._num_nuts):
            self._nut_init_poses[nut_idx, :3] = Gf.Vec3f(nut_spiral_center)
            self._nut_init_poses[nut_idx, 0] += self._nut_radius * math.sin(
                math.pi / 3.0 * nut_idx
            ) + self._nut_dist_delta * (nut_idx // 6)
            self._nut_init_poses[nut_idx, 1] += self._nut_radius * math.cos(
                math.pi / 3.0 * nut_idx
            ) + self._nut_dist_delta * (nut_idx // 6)
            self._nut_init_poses[nut_idx, 2] += self._nut_height_delta * (nut_idx // 6)
            if self._randomize_nut_positions:
                self._nut_init_poses[nut_idx, 0] += self._get_uniform_random_number(self._nut_position_noise_minmax)
                self._nut_init_poses[nut_idx, 1] += self._get_uniform_random_number(self._nut_position_noise_minmax)

    def on_physics_step(self, dt):
        carb.profiler.begin(1, "Franka FSM Update")
        self._time += self._sim_dt
        self._fsm_time += self._sim_dt

        # update vibra tables
        for env_idx in range(self._num_envs):
            offsetPos = self._fsms[env_idx].update_vibra_table_fsm(dt)
            self._vibra_table_transforms[env_idx, :3] = self._vibra_table_position_np + offsetPos
        self._vibra_tables.set_kinematic_targets(self._vibra_table_transforms, self._vibra_table_indices)

        if self._fsm_time < self._fsm_update_dt:
            carb.profiler.end(1)
            return
        self._fsm_time = 0.0

        # fsm runs on CPU:
        # get transforms
        nut_transforms = self._nuts.get_transforms()
        bolt_transforms = self._bolts.get_transforms()
        bolt_poses = bolt_transforms[:, :7]

        # get end effector transforms
        hand_transforms = self._hands.get_transforms()
        hand_poses = hand_transforms[:, :7]

        # get franka DOF states
        dof_pos = self._frankas_view.get_dof_positions()

        for env_idx in range(self._num_envs):
            self._fsms[env_idx].update(
                self._fsm_update_dt,
                nut_transforms,
                bolt_poses[env_idx * self._num_bolts:(env_idx + 1) * self._num_bolts, :],
                hand_poses[env_idx, :],
                dof_pos[env_idx, 6:9],
                self._pos_action[env_idx, 6],
            )
            self._d_pose[env_idx, :] = self._fsms[env_idx].franka_state.pose_delta
            self._grip_sep[env_idx] = self._fsms[env_idx].franka_state.gripper_separation

        self._run_ik(direct_z_rotation=True)

        carb.profiler.end(1)

    @staticmethod
    def _get_local_bounds(xform: UsdGeom.Xformable) -> Gf.Range3d:
        im = UsdGeom.Imageable(xform)
        if not im:
            return None
        lb = im.ComputeLocalBound(Usd.TimeCode.Default(), purpose1=UsdGeom.Tokens.default_)
        return lb.GetRange()


class ScrewFSM:
    def __init__(
        self,
        vibra_dt,
        nut_height,
        bolt_height,
        table_height,
        pipe_height,
        vibra_table_nut_pickup_pos,
        env_idx,
        num_nuts,
        num_bolts,
    ):

        self._env_idx = env_idx
        self.franka_state = franka_helpers.FrankaState()

        # geometry
        self._nut_height = nut_height
        self._bolt_height = bolt_height
        self._screw_speed_back = 720.0 / 180.0 * math.pi
        self._screw_speed_on = 360.0 / 180.0 * math.pi
        self._screw_speed = self._screw_speed_on
        self._screw_limit_angle = 150.0 / 180.0 * math.pi
        self._num_nuts = num_nuts
        self._num_bolts = num_bolts

        # states:
        self._state = "goto_nut_pickup"
        self._check_for_screw_reset = False
        self._screw_next_state = "invalid"
        self._target_bolt_idx = -1

        # vibra table fsm
        self._vibra_fsm = VibraFSM(dt=vibra_dt)
        self._target_nut_ind = -1
        self._vibra_off_y = 0.08875
        self._vibra_off_delay_sec = 0.55

        # control / position constants:
        self._pickup_pos_nut_nominal = np.array(vibra_table_nut_pickup_pos, dtype=np.float32)
        self._pickup_pos = self._pickup_pos_nut_nominal.copy()
        self._pickup_pos[2] += 0.09 + self._nut_height
        grip_z_offset = 0.003
        self._grip_offset = np.array([0, 0, 0.09 + grip_z_offset + self._nut_height], dtype=np.float32)
        self._lift_offset = np.array([0, -0.001, 0.17 + self._bolt_height], dtype=np.float32)
        self._above_bolt_offset = np.array([0, 0, 0.15 - grip_z_offset], dtype=np.float32) + self._grip_offset
        self._hand_down_quat = np.array([1, 0, 0, 0], dtype=np.float32)
        self._nut_grab_q = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(math.pi / 2.0), self._hand_down_quat)
        self._liftQ = np.array([1, 0, 0, 0], dtype=np.float32)
        start_screw_angle = 160.0 * math.pi / 180.0
        self._start_screw_quat = numpy_utils.quat_mul(
            numpy_utils.get_z_rot_quat(start_screw_angle), self._hand_down_quat
        )
        self.franka_state.alignment_gain = -4.0

        self._grip_sep = 0.027
        self._screw_sep = 0.035
        self._pickup_gripper_sep = 0.05
        self._rotate_back_gripper_sep = 0.05
        self._nut_screwing_z_offset = 0.002
        self._switch_to_nut_target_height_z_thres = self._bolt_height + 0.1 * self._nut_height
        self._nut_reset_trigger_height = table_height + 0.063
        self._nut_pickup_delta = 0.015  # defines window around pickup pos where we detect nuts to pick up
        self._screw_target_hand_z = None
        self._screw_on_z_offset = -0.002

        self.franka_state.pos_error_threshold = 10.0e-3
        self.franka_state.rot_error_threshold = 2.5e-2
        self._position_above_bolt_nut_err_thres = 1.0e-3
        self._screw_stop_height = table_height + pipe_height + nut_height * 0.505
        self._screw_slowdown_height = self._screw_stop_height + nut_height * 0.25
        self._screw_speed_slowdown_factor = 0.25
        self._post_screw_liftoff_delta_z = bolt_height * 0.6
        self._nut_screw_fail_thres = 0.005

    def reset(self):
        self._state = "goto_nut_pickup"
        self._screw_angle = 0.0
        self._check_for_screw_reset = False
        self._vibra_fsm.reset()
        self._target_nut_ind = -1
        self._target_bolt_idx = -1

    def set_screw_motion(self, start_angle, direction, next_state, screw_gripper_sep):
        self._screw_gripper_sep = screw_gripper_sep
        self._screw_next_state = next_state
        self._screw_dir = 1.0
        if direction != "positive":
            self._screw_dir = -1.0
        angle_to_limit = self._screw_limit_angle * self._screw_dir - start_angle
        num_sixties = int(angle_to_limit / math.pi * 3.0)
        self._screw_angle = start_angle
        self._screw_target_angle = start_angle + num_sixties * math.pi / 3.0
        return "screw_motion"

    def get_hand_orientation_target(self, nut_q, hand_q) -> np.array:
        # get x-axis of nut and project it to xy-plane
        nut_x = numpy_utils.quat_rotate(nut_q.reshape(1, 4), np.array([[1, 0, 0]]))[0]
        nut_angle = np.arctan2(nut_x[0], nut_x[1])
        # same for hand:
        hand_y = numpy_utils.quat_rotate(hand_q.reshape(1, 4), np.array([[0, 1, 0]]))[0]
        hand_angle = np.arctan2(hand_y[0], hand_y[1])
        # angle between the two:
        angle = hand_angle - nut_angle
        # shortest rotation to a face:
        nut_face_interval = math.pi / 6.0
        # error modulo 60 deg to align with face:
        nut_rot_error_z = np.mod(angle + nut_face_interval, math.pi / 3.0) - nut_face_interval
        targetQ = numpy_utils.quat_mul(
            numpy_utils.get_z_rot_quat(math.pi - hand_angle + nut_rot_error_z), self._hand_down_quat
        )
        return targetQ

    def _target_next_bolt(self) -> bool:
        self._vibra_state = "run_feed"
        self._target_nut_ind = -1
        self._target_bolt_idx += 1
        self._check_for_screw_reset = False
        if self._target_bolt_idx >= self._num_bolts:
            self._target_bolt_idx = 0
            return True
        return False

    def update(
        self,
        fsm_dt,
        nut_poses,
        bolt_poses,
        hand_pose,
        current_hand_dof_pos,
        gripper_z_rot_target,
    ):
        nut_pose = None
        if not (self._target_nut_ind < 0):
            nut_pose = nut_poses[self._target_nut_ind, :]
        current_gripper_sep = current_hand_dof_pos[1] + current_hand_dof_pos[2]
        gripper_z_rot = current_hand_dof_pos[0]
        bolt_pose = bolt_poses[self._target_bolt_idx, :]

        new_state = self._state

        if self._state == "advance":
            if self.franka_state.advance(hand_pose=hand_pose, dt=fsm_dt):
                new_state = self._next_state
        elif self._state == "wait_for_nut":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            pos_error, _ = self.franka_state.update_pose_delta_from_target(
                self._pickup_pos, self._nut_grab_q, hand_pose
            )
            # find next nut:
            startInd = self._env_idx * self._num_nuts
            endInd = (self._env_idx + 1) * self._num_nuts
            env_nut_pos = nut_poses[startInd:endInd, :3]
            below = env_nut_pos < self._pickup_pos_nut_nominal + self._nut_pickup_delta
            above = env_nut_pos > self._pickup_pos_nut_nominal - self._nut_pickup_delta
            inside = np.all(above & below, 1)
            if np.any(inside):
                self._target_nut_ind = np.argmax(inside) + startInd
            if not (self._target_nut_ind < 0):
                if self._vibra_fsm.is_stopped() and pos_error < self.franka_state.pos_error_threshold:
                    new_state = "prep_grip"
                elif not self._vibra_fsm.is_stopping():
                    if nut_poses[self._target_nut_ind, 1] > self._vibra_off_y:
                        self._vibra_fsm.stop_feed_after_delay(self._vibra_off_delay_sec)
        elif self._state == "done":
            target_pos = self._pickup_pos.copy()
            target_pos[2] += self._bolt_height
            self.franka_state.update_pose_delta_from_target(target_pos, self._nut_grab_q, hand_pose)
        elif self._state == "goto_nut_pickup":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            if self._target_next_bolt():
                # DONE!
                target_pos = self._pickup_pos.copy()
                target_pos[2] += self._bolt_height
                new_state = "advance"
                self._next_state = "done"
                self.franka_state.current_time = 0.0
                self.franka_state.pose_motion = franka_helpers.PoseMotion(
                    start_pos=hand_pose[:3],
                    end_pos=target_pos,
                    start_quat=self._nut_grab_q,
                    end_quat=self._nut_grab_q,
                    duration=2.0,
                )
            else:
                new_state = "advance"
                self._next_state = "wait_for_nut"
                self.franka_state.current_time = 0.0
                self.franka_state.pose_motion = franka_helpers.PoseMotion(
                    start_pos=hand_pose[:3],
                    end_pos=self._pickup_pos,
                    start_quat=self._nut_grab_q,
                    end_quat=self._nut_grab_q,
                    duration=2.0,
                )
                self._vibra_fsm.start_feed()
        elif self._state == "goto_above_nut_pickup":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            target_pos = self._pickup_pos.copy()
            target_pos[2] += self._bolt_height
            new_state = "advance"
            self._next_state = "goto_nut_pickup"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=target_pos,
                start_quat=self._nut_grab_q,
                end_quat=self._nut_grab_q,
                duration=1.0,
            )
        elif self._state == "prep_grip":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            target_pos = nut_pose[:3] + self._grip_offset
            target_pos[0] = self._pickup_pos_nut_nominal[0]
            # compute nut-hand rotation deviation:
            gripQuat = self.get_hand_orientation_target(nut_pose[3:], hand_pose[3:])
            pos_error, rot_error = self.franka_state.update_pose_delta_from_target(target_pos, gripQuat, hand_pose)
            if pos_error < self.franka_state.pos_error_threshold and rot_error < self.franka_state.rot_error_threshold:
                self._grip_pos = target_pos.copy()
                new_state = "advance"
                self._next_state = "grip"
                self.franka_state.current_time = 0.0
                self.franka_state.pose_motion = franka_helpers.PoseMotion(
                    start_pos=target_pos,
                    end_pos=target_pos,
                    start_quat=gripQuat,
                    end_quat=gripQuat,
                    duration=0.2,
                )
        elif self._state == "grip":
            self.franka_state.gripper_separation = self._grip_sep
            gripQuat = self.get_hand_orientation_target(nut_pose[3:], hand_pose[3:])
            self.franka_state.update_pose_delta_from_target(self._grip_pos, gripQuat, hand_pose)
            gripped = current_gripper_sep < 0.039
            if gripped:
                self._liftQ = gripQuat
                new_state = "lift"
        elif self._state == "lift":
            self.franka_state.gripper_separation = self._grip_sep
            target_pos = nut_pose[:3].copy()
            target_pos[0] = self._pickup_pos_nut_nominal[0]
            target_pos[2] = bolt_pose[2] + self._nut_height * 0.5
            target_pos = target_pos + self._lift_offset
            self._lift_target_pos = target_pos.copy()
            new_state = "advance"
            self._next_state = "go_above_bolt"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=self._grip_pos,
                end_pos=target_pos,
                start_quat=self._liftQ,
                end_quat=self._liftQ,
                duration=1.5,
            )
        elif self._state == "go_above_bolt":
            self.franka_state.gripper_separation = self._grip_sep
            target_pos = bolt_pose[:3]
            target_pos = target_pos + self._above_bolt_offset
            new_state = "advance"
            self._next_state = "position_above_bolt"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=self._lift_target_pos,
                end_pos=target_pos,
                start_quat=self._start_screw_quat,
                end_quat=self._start_screw_quat,
                duration=1.5,
                end_delay=0.0,
            )
            self._check_for_screw_reset = True
        elif self._state == "position_above_bolt":
            self.franka_state.gripper_separation = self._grip_sep
            target_pos = bolt_pose[:3].copy()
            target_pos = target_pos + self._above_bolt_offset
            target_pos, pos_error = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            self.franka_state.update_pose_delta_from_target(target_pos, self._start_screw_quat, hand_pose)
            if pos_error < self._position_above_bolt_nut_err_thres:
                z_delta_nut_bolt = nut_pose[2] - bolt_pose[2] - self._bolt_height - self._nut_height * 0.4
                target_pos[2] -= z_delta_nut_bolt
                target_pos[2] += self._screw_on_z_offset
                new_state = "advance"
                self._next_state = "start_screwing"
                self.franka_state.current_time = 0.0
                self.franka_state.pose_motion = franka_helpers.PoseMotion(
                    start_pos=hand_pose[:3],
                    end_pos=target_pos,
                    start_quat=self._start_screw_quat,
                    end_quat=self._start_screw_quat,
                    duration=1.0,
                    use_alignment=True,
                    end_delay=0.6,
                )
                self._screw_target_hand_z = target_pos[2]
        elif self._state == "start_screwing":
            self.franka_state.gripper_separation = self._screw_sep
            target_pos = bolt_pose[:3].copy()
            target_pos[2] = self._screw_target_hand_z
            target_pos, _ = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            # compute nut-hand rotation deviation:
            pos_error, rot_error = self.franka_state.update_pose_delta_from_target(
                target_pos, self._start_screw_quat, hand_pose
            )
            self._screw_speed = self._screw_speed_on * self._screw_speed_slowdown_factor
            new_state = self.set_screw_motion(gripper_z_rot, "positive", "ungrip_screw", self._screw_sep)
        elif self._state == "screw_motion":
            self.franka_state.gripper_separation = self._screw_gripper_sep
            target_pos = bolt_pose[:3].copy()
            target_pos[2] = self._screw_target_hand_z
            target_pos, _ = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            self.franka_state.update_pose_delta_from_target(target_pos, self._hand_down_quat, hand_pose)
            # control screw angle independently of other DOFs
            screw_speed = self._screw_speed
            is_nut_not_threaded = False  # nut_pose[2] - bolt_pose[2] - self._bolt_height > 0.25 * self._nut_height
            if nut_pose[2] < self._screw_slowdown_height or is_nut_not_threaded:
                screw_speed *= self._screw_speed_slowdown_factor
            self._screw_angle = self._screw_angle + self._screw_dir * fsm_dt * screw_speed
            screw_target_reached = False
            if self._screw_dir < 0:
                screw_target_reached = self._screw_angle < self._screw_target_angle
            else:
                screw_target_reached = self._screw_angle > self._screw_target_angle
            if screw_target_reached:
                self._screw_angle = self._screw_target_angle
            self.franka_state.pose_delta[5] = gripper_z_rot - self._screw_angle
            gripper_z_rot_err = abs(gripper_z_rot - gripper_z_rot_target)

            if nut_pose[2] < self._screw_stop_height:
                new_state = "ungrip_screw_after_screw_done"
            elif screw_target_reached and abs(gripper_z_rot_err) < 0.01:
                new_state = self._screw_next_state
        elif self._state == "ungrip_screw_after_screw_done":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            target_pos = hand_pose[:3]
            target_pos[2] += self._post_screw_liftoff_delta_z
            new_state = "advance"
            self._next_state = "goto_above_nut_pickup"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=target_pos,
                start_quat=self._nut_grab_q,
                end_quat=self._nut_grab_q,
                duration=2.0,
                use_alignment=False,
            )
            self._check_for_screw_reset = False
        elif self._state == "ungrip_screw":
            self.franka_state.gripper_separation = self._rotate_back_gripper_sep
            target_pos = bolt_pose[:3]
            target_pos[2] = self._screw_target_hand_z
            target_pos, _ = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            targetQuat = self.get_hand_orientation_target(nut_pose[3:], hand_pose[3:])
            self.franka_state.update_pose_delta_from_target(target_pos, targetQuat, hand_pose)
            self.franka_state.pose_delta[5] = 0
            un_gripped = current_gripper_sep > self._rotate_back_gripper_sep * 0.98
            if un_gripped:
                self._screw_speed = self._screw_speed_back
                new_state = self.set_screw_motion(
                    gripper_z_rot, "negative", "align_to_nut", self._rotate_back_gripper_sep
                )
        elif self._state == "align_to_nut":
            self.franka_state.gripper_separation = self._pickup_gripper_sep
            target_pos = bolt_pose[:3]
            target_pos[2] = nut_pose[2] + self._nut_screwing_z_offset
            target_pos = target_pos + self._grip_offset
            target_pos, _ = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            # compute nut-hand rotation deviation:
            targetQuat = self.get_hand_orientation_target(nut_pose[3:], hand_pose[3:])
            pos_error, rot_error = self.franka_state.update_pose_delta_from_target(target_pos, targetQuat, hand_pose)
            if pos_error < self.franka_state.pos_error_threshold and rot_error < self.franka_state.rot_error_threshold:
                new_state = "back_to_screw_grip"
        elif self._state == "back_to_screw_grip":
            target_sep = self._screw_sep
            self.franka_state.gripper_separation = target_sep
            target_pos = bolt_pose[:3]
            target_pos[2] = nut_pose[2] + self._nut_screwing_z_offset
            target_pos = target_pos + self._grip_offset
            target_pos, _ = self.franka_state.offset_target_position_to_align(target_pos, hand_pose)
            # compute nut-hand rotation deviation:
            targetQuat = self.get_hand_orientation_target(nut_pose[3:], hand_pose[3:])
            pos_error, rot_error = self.franka_state.update_pose_delta_from_target(target_pos, targetQuat, hand_pose)
            gripped = current_gripper_sep < target_sep * 1.2
            if gripped:
                self._screw_speed = self._screw_speed_on
                new_state = self.set_screw_motion(gripper_z_rot, "positive", "ungrip_screw", self._screw_sep)
                self._screw_target_hand_z = target_pos[2]
        else:
            assert False

        if self._check_for_screw_reset and nut_pose is not None:
            dist = np.linalg.norm(nut_pose[:2] - hand_pose[:2])
            if dist > self._nut_screw_fail_thres:
                new_state = "ungrip_screw_after_screw_done"
                self._check_for_screw_reset = False

        if new_state != self._state:
            self._state = new_state

    def update_vibra_table_fsm(self, dt):
        return self._vibra_fsm.update(dt)


class VibraFSM:
    _amplitudes = {
        "stop": np.array((0.0, 0.0, 0.0), dtype=np.float32),  # [m]
        "run_feed": np.array((0.000, 0.0003, 0.0002), dtype=np.float32),  # [m]
        "backward": np.array((0.000, -0.0003, 0.0002), dtype=np.float32),  # [m]
        "realign": np.array((-0.0003, 0.0, 0.0002), dtype=np.float32),  # [m]
    }
    _motion_frequency = 60.0  # [Hz]

    # configure unblock-cycle:
    _feed_time = 10.0
    _backward_time = 1.5
    _realign_time = 1.5

    def __init__(self, dt):
        self.reset()
        self._dt = dt

    def reset(self):
        self._dt = 1.0 / 240.0
        self._time = 0.0
        self.state = "stop"
        self._after_delay_state = None

    def start_feed(self):
        self.state = "run_feed"
        # kick off unblock cycle
        self._set_delayed_state_change(delay_sec=self._feed_time, nextState="backward")

    def stop_feed_after_delay(self, delay_sec: float):
        self.state = "run_feed"
        self._set_delayed_state_change(delay_sec=delay_sec, nextState="stop")

    def _set_delayed_state_change(self, delay_sec: float, nextState: str):
        self._after_delay_state = nextState
        self._wait_end_time = self._time + delay_sec

    def update(self, dt):
        self._time += dt
        # process wait if necessary
        if self._after_delay_state is not None and self._time > self._wait_end_time:
            self.state = self._after_delay_state
            # auto-unblock cycle
            if self._state == "run_feed":
                self._set_delayed_state_change(delay_sec=self._feed_time, nextState="backward")
            elif self._state == "backward":
                self._set_delayed_state_change(delay_sec=self._backward_time, nextState="realign")
            elif self._state == "realign":
                self._set_delayed_state_change(delay_sec=self._realign_time, nextState="run_feed")
            else:
                self._after_delay_state = None
        return self._motion_amplitude * math.sin(2.0 * math.pi * self._time * self._motion_frequency)

    def is_stopped(self):
        return self._state == "stop"

    def is_stopping(self):
        return self.is_stopped() or self._after_delay_state == "stop"

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        self._state = new_state
        if self._state in self._amplitudes:
            self._motion_amplitude = self._amplitudes[self._state]
        else:
            self._motion_amplitude = self._amplitudes["stop"]
