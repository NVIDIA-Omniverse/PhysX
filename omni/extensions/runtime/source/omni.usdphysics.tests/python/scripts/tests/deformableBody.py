# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from .base import UsdPhysicsBaseTest


class UsdPhysicsDeformableBodyTest(UsdPhysicsBaseTest):

    async def test_volume_deformable_body(self):
        self.fail_on_log_error = True
        self.expected_prims = {}

        # expected volume deformable body
        tetmesh_db_dict = {}
        tetmesh_db_dict["simulation_owner"] = ""
        tetmesh_db_dict["enabled"] = True
        tetmesh_db_dict["kinematic"] = False
        tetmesh_db_dict["start_asleep"] = False
        tetmesh_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        tetmesh_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        tetmesh_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        tetmesh_db_dict["transform_row3"] = carb.Double4(0, 0, 500, 1)
        tetmesh_db_dict["sim_mesh_path"] = "/World/VolumeDeformable"
        tetmesh_db_dict["collision_geom_paths"] = []
        tetmesh_db_dict["skin_geom_paths"] = []
        tetmesh_db_dict["sim_mesh_material_path"] = ""
        tetmesh_db_dict["collision_geom_material_paths"] = []
        tetmesh_db_dict["skin_geom_material_paths"] = []
        self.expected_prims["/World/VolumeDeformable" + "/volumeDeformableBody"] = tetmesh_db_dict

        # expect volume deformable body assembly under xform
        xform_db_dict = {}
        xform_db_dict["simulation_owner"] = ""
        xform_db_dict["enabled"] = True
        xform_db_dict["kinematic"] = False
        xform_db_dict["start_asleep"] = False
        xform_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        xform_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        xform_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        xform_db_dict["transform_row3"] = carb.Double4(0, 0, 500, 1)
        xform_db_dict["sim_mesh_path"] = "/World/VolumeDeformableXform/SimMesh"
        xform_db_dict["collision_geom_paths"] = []
        xform_db_dict["skin_geom_paths"] = ["/World/VolumeDeformableXform/SkinMesh"]
        xform_db_dict["sim_mesh_material_path"] = ""
        xform_db_dict["collision_geom_material_paths"] = []
        xform_db_dict["skin_geom_material_paths"] = [""]
        self.expected_prims["/World/VolumeDeformableXform" + "/volumeDeformableBody"] = xform_db_dict

        # expect volume deformable body assembly under scope
        scope_db_dict = {}
        scope_db_dict["simulation_owner"] = ""
        scope_db_dict["enabled"] = True
        scope_db_dict["kinematic"] = False
        scope_db_dict["start_asleep"] = False
        scope_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        scope_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        scope_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        scope_db_dict["transform_row3"] = carb.Double4(0, 0, 0, 1)
        scope_db_dict["sim_mesh_path"] = "/World/VolumeDeformableScope/SimMesh"
        scope_db_dict["collision_geom_paths"] = []
        scope_db_dict["skin_geom_paths"] = ["/World/VolumeDeformableScope/SkinMesh"]
        scope_db_dict["sim_mesh_material_path"] = ""
        scope_db_dict["collision_geom_material_paths"] = []
        scope_db_dict["skin_geom_material_paths"] = [""]
        self.expected_prims["/World/VolumeDeformableScope" + "/volumeDeformableBody"] = scope_db_dict

        # expect single pose multi purpose
        db_pose_spmp_dict = {}
        db_pose_spmp_dict["sim_mesh_bind_pose_token"] = "singlepose"
        db_pose_spmp_dict["collision_geom_self_collision_filter_pose_tokens"] = ["singlepose"]
        db_pose_spmp_dict["skin_geom_bind_pose_tokens"] = ["singlepose"]
        self.expected_prims["/World/VolumeDeformableWithSinglePoseMultiPurpose" + "/volumeDeformableBody"] = db_pose_spmp_dict

        # expect multi pose single purpose
        db_pose_mpsp_dict = {}
        db_pose_mpsp_dict["sim_mesh_bind_pose_token"] = "multipose0"
        db_pose_mpsp_dict["collision_geom_bind_pose_tokens"] = ["multipose0"]
        db_pose_mpsp_dict["collision_geom_self_collision_filter_pose_tokens"] = ["multipose1"]
        db_pose_mpsp_dict["skin_geom_bind_pose_tokens"] = ["multipose0"]
        self.expected_prims["/World/VolumeDeformableWithMultiPoseSinglePurpose" + "/volumeDeformableBody"] = db_pose_mpsp_dict

        # expect deformable body with collision api
        db_collision_dict = {}
        db_collision_dict["sim_mesh_path"] = "/World/VolumeDeformableCollision"
        db_collision_dict["skin_geom_paths"] = []
        db_collision_dict["collision_geom_paths"] = ["/World/VolumeDeformableCollision"]
        self.expected_prims["/World/VolumeDeformableCollision" + "/volumeDeformableBody"] = db_collision_dict

        # expect deformable body with unified simulation and collision mesh
        db_simcollision_dict = {}
        db_simcollision_dict["sim_mesh_path"] = "/World/VolumeDeformableSimCollision/SimMesh"
        db_simcollision_dict["collision_geom_paths"] = ["/World/VolumeDeformableSimCollision/SimMesh"]
        db_simcollision_dict["skin_geom_paths"] = ["/World/VolumeDeformableSimCollision/SkinMesh"]
        self.expected_prims["/World/VolumeDeformableSimCollision" + "/volumeDeformableBody"] = db_simcollision_dict

        # expect deformable body with skin collision
        db_skincollision_dict = {}
        db_skincollision_dict["sim_mesh_path"] = "/World/VolumeDeformableSkinCollision/SimMesh"
        db_skincollision_dict["collision_geom_paths"] = ["/World/VolumeDeformableSkinCollision/SkinMesh"]
        db_skincollision_dict["skin_geom_paths"] = []
        self.expected_prims["/World/VolumeDeformableSkinCollision" + "/volumeDeformableBody"] = db_skincollision_dict

        # expect deformable body with dedicated collision mesh
        db_dedicatedcollision_dict = {}
        db_dedicatedcollision_dict["sim_mesh_path"] = "/World/VolumeDeformableDedicatedCollision/SimMesh"
        db_dedicatedcollision_dict["collision_geom_paths"] = ["/World/VolumeDeformableDedicatedCollision/CollMesh"]
        db_dedicatedcollision_dict["skin_geom_paths"] = ["/World/VolumeDeformableDedicatedCollision/SkinMesh"]
        self.expected_prims["/World/VolumeDeformableDedicatedCollision" + "/volumeDeformableBody"] = db_dedicatedcollision_dict

        await self.parse("DeformableBody")


    async def test_surface_deformable_body(self):
        self.fail_on_log_error = True
        self.expected_prims = {}

        # expect surface deformable body
        mesh_db_dict = {}
        mesh_db_dict["simulation_owner"] = ""
        mesh_db_dict["enabled"] = True
        mesh_db_dict["kinematic"] = False
        mesh_db_dict["start_asleep"] = False
        mesh_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        mesh_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        mesh_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        mesh_db_dict["transform_row3"] = carb.Double4(0, 0, 500, 1)
        mesh_db_dict["sim_mesh_path"] = "/World/SurfaceDeformable"
        mesh_db_dict["collision_geom_paths"] = []
        mesh_db_dict["skin_geom_paths"] = []
        mesh_db_dict["sim_mesh_material_path"] = ""
        mesh_db_dict["collision_geom_material_paths"] = []
        mesh_db_dict["skin_geom_material_paths"] = []
        self.expected_prims["/World/SurfaceDeformable" + "/surfaceDeformableBody"] = mesh_db_dict

        # expect surface deformable body assembly under xform
        xform_db_dict = {}
        xform_db_dict["simulation_owner"] = ""
        xform_db_dict["enabled"] = True
        xform_db_dict["kinematic"] = False
        xform_db_dict["start_asleep"] = False
        xform_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        xform_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        xform_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        xform_db_dict["transform_row3"] = carb.Double4(0, 0, 500, 1)
        xform_db_dict["sim_mesh_path"] = "/World/SurfaceDeformableXform/SimMesh"
        xform_db_dict["collision_geom_paths"] = []
        xform_db_dict["skin_geom_paths"] = ["/World/SurfaceDeformableXform/SkinMesh"]
        xform_db_dict["sim_mesh_material_path"] = ""
        xform_db_dict["collision_geom_material_paths"] = []
        xform_db_dict["skin_geom_material_paths"] = [""]
        self.expected_prims["/World/SurfaceDeformableXform" + "/surfaceDeformableBody"] = xform_db_dict

        # expect surface deformable body assembly under scope
        scope_db_dict = {}
        scope_db_dict["simulation_owner"] = ""
        scope_db_dict["enabled"] = True
        scope_db_dict["kinematic"] = False
        scope_db_dict["start_asleep"] = False
        scope_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        scope_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        scope_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        scope_db_dict["transform_row3"] = carb.Double4(0, 0, 0, 1)
        scope_db_dict["sim_mesh_path"] = "/World/SurfaceDeformableScope/SimMesh"
        scope_db_dict["collision_geom_paths"] = []
        scope_db_dict["skin_geom_paths"] = ["/World/SurfaceDeformableScope/SkinMesh"]
        scope_db_dict["sim_mesh_material_path"] = ""
        scope_db_dict["collision_geom_material_paths"] = []
        scope_db_dict["skin_geom_material_paths"] = [""]
        self.expected_prims["/World/SurfaceDeformableScope" + "/surfaceDeformableBody"] = scope_db_dict

        # expect single pose multi purpose
        db_pose_spmp_dict = {}
        db_pose_spmp_dict["sim_mesh_bind_pose_token"] = "singlepose"
        db_pose_spmp_dict["collision_geom_self_collision_filter_pose_tokens"] = ["singlepose"]
        db_pose_spmp_dict["skin_geom_bind_pose_tokens"] = ["singlepose"]
        self.expected_prims["/World/SurfaceDeformableWithSinglePoseMultiPurpose" + "/surfaceDeformableBody"] = db_pose_spmp_dict

        # expect multi pose single purpose
        db_pose_mpsp_dict = {}
        db_pose_mpsp_dict["sim_mesh_bind_pose_token"] = "multipose0"
        db_pose_mpsp_dict["collision_geom_bind_pose_tokens"] = ["multipose0"]
        db_pose_mpsp_dict["collision_geom_self_collision_filter_pose_tokens"] = ["multipose1"]
        db_pose_mpsp_dict["skin_geom_bind_pose_tokens"] = ["multipose0"]
        self.expected_prims["/World/SurfaceDeformableWithMultiPoseSinglePurpose" + "/surfaceDeformableBody"] = db_pose_mpsp_dict

        # expect deformable body with collision api
        db_collision_dict = {}
        db_collision_dict["sim_mesh_path"] = "/World/SurfaceDeformableCollision"
        db_collision_dict["skin_geom_paths"] = []
        db_collision_dict["collision_geom_paths"] = ["/World/SurfaceDeformableCollision"]
        self.expected_prims["/World/SurfaceDeformableCollision" + "/surfaceDeformableBody"] = db_collision_dict

        # expect deformable body with unified simulation and collision mesh
        db_simcollision_dict = {}
        db_simcollision_dict["sim_mesh_path"] = "/World/SurfaceDeformableSimCollision/SimMesh"
        db_simcollision_dict["collision_geom_paths"] = ["/World/SurfaceDeformableSimCollision/SimMesh"]
        db_simcollision_dict["skin_geom_paths"] = ["/World/SurfaceDeformableSimCollision/SkinMesh"]
        self.expected_prims["/World/SurfaceDeformableSimCollision" + "/surfaceDeformableBody"] = db_simcollision_dict

        # expect deformable body with skin collision
        db_skincollision_dict = {}
        db_skincollision_dict["sim_mesh_path"] = "/World/SurfaceDeformableSkinCollision/SimMesh"
        db_skincollision_dict["collision_geom_paths"] = ["/World/SurfaceDeformableSkinCollision/SkinMesh"]
        db_skincollision_dict["skin_geom_paths"] = []
        self.expected_prims["/World/SurfaceDeformableSkinCollision" + "/surfaceDeformableBody"] = db_skincollision_dict

        # expect deformable body with dedicated collision mesh
        db_dedicatedcollision_dict = {}
        db_dedicatedcollision_dict["sim_mesh_path"] = "/World/SurfaceDeformableDedicatedCollision/SimMesh"
        db_dedicatedcollision_dict["collision_geom_paths"] = ["/World/SurfaceDeformableDedicatedCollision/CollMesh"]
        db_dedicatedcollision_dict["skin_geom_paths"] = ["/World/SurfaceDeformableDedicatedCollision/SkinMesh"]
        self.expected_prims["/World/SurfaceDeformableDedicatedCollision" + "/surfaceDeformableBody"] = db_dedicatedcollision_dict

        await self.parse("DeformableBody")


    async def test_volume_deformable_body_material(self):
        self.fail_on_log_error = True
        self.expected_prims = {}
        
        # expected volume deformable body with material assignment
        tetmesh_db_dict = {}
        tetmesh_db_dict["simulation_owner"] = ""
        tetmesh_db_dict["enabled"] = True
        tetmesh_db_dict["kinematic"] = False
        tetmesh_db_dict["start_asleep"] = False
        tetmesh_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        tetmesh_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        tetmesh_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        tetmesh_db_dict["transform_row3"] = carb.Double4(0, 0, 0, 1)
        tetmesh_db_dict["sim_mesh_path"] = "/World/VolumeDeformable"
        tetmesh_db_dict["collision_geom_paths"] = []
        tetmesh_db_dict["skin_geom_paths"] = []
        tetmesh_db_dict["sim_mesh_material_path"] = "/World/Materials/DeformableMaterialA"
        tetmesh_db_dict["collision_geom_material_paths"] = []
        tetmesh_db_dict["skin_geom_material_paths"] = []
        self.expected_prims["/World/VolumeDeformable" + "/volumeDeformableBody"] = tetmesh_db_dict

        # expect volume deformable body assembly under xform with material and skin material assignement
        xform_db_dict = {}
        xform_db_dict["simulation_owner"] = ""
        xform_db_dict["enabled"] = True
        xform_db_dict["kinematic"] = False
        xform_db_dict["start_asleep"] = False
        xform_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        xform_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        xform_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        xform_db_dict["transform_row3"] = carb.Double4(0, 0, 0, 1)
        xform_db_dict["sim_mesh_path"] = "/World/VolumeDeformableXform/SimMesh"
        xform_db_dict["collision_geom_paths"] = ["/World/VolumeDeformableXform/CollisionMesh"]
        xform_db_dict["skin_geom_paths"] = ["/World/VolumeDeformableXform/SkinMesh"]
        xform_db_dict["sim_mesh_material_path"] = "/World/Materials/DeformableMaterialB"
        xform_db_dict["collision_geom_material_paths"] = ["/World/Materials/DeformableMaterialC"]
        xform_db_dict["skin_geom_material_paths"] = ["/World/Materials/DeformableMaterialD"]
        self.expected_prims["/World/VolumeDeformableXform" + "/volumeDeformableBody"] = xform_db_dict

        # material A
        material_a_dict = {}
        material_a_dict["youngs_modulus"] = 100000.0
        material_a_dict["poissons_ratio"] = 0.3
        self.expected_prims["/World/Materials/DeformableMaterialA" + "/volumeDeformableMaterial"] = material_a_dict

        # material B
        material_b_dict = {}
        material_b_dict["youngs_modulus"] = 200000.0
        material_b_dict["poissons_ratio"] = 0.4
        self.expected_prims["/World/Materials/DeformableMaterialB" + "/volumeDeformableMaterial"] = material_b_dict

        material_c_dict = {}
        material_c_dict["youngs_modulus"] = 300000.0
        material_c_dict["poissons_ratio"] = 0.5
        self.expected_prims["/World/Materials/DeformableMaterialC" + "/volumeDeformableMaterial"] = material_c_dict

        material_d_dict = {}
        material_d_dict["youngs_modulus"] = 400000.0
        material_d_dict["poissons_ratio"] = 0.6
        self.expected_prims["/World/Materials/DeformableMaterialD" + "/volumeDeformableMaterial"] = material_d_dict

        await self.parse("DeformableBodyMaterial")


    async def test_surface_deformable_body_material(self):
        self.fail_on_log_error = True
        self.expected_prims = {}
        
        # expected surface deformable body with material assignment
        mesh_db_dict = {}
        mesh_db_dict["simulation_owner"] = ""
        mesh_db_dict["enabled"] = True
        mesh_db_dict["kinematic"] = False
        mesh_db_dict["start_asleep"] = False
        mesh_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        mesh_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        mesh_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        mesh_db_dict["transform_row3"] = carb.Double4(0, 0, 0, 1)
        mesh_db_dict["sim_mesh_path"] = "/World/SurfaceDeformable"
        mesh_db_dict["collision_geom_paths"] = []
        mesh_db_dict["skin_geom_paths"] = []
        mesh_db_dict["sim_mesh_material_path"] = "/World/Materials/SurfaceDeformableMaterialA"
        mesh_db_dict["collision_geom_material_paths"] = []
        mesh_db_dict["skin_geom_material_paths"] = []
        self.expected_prims["/World/SurfaceDeformable" + "/surfaceDeformableBody"] = mesh_db_dict

        # expect surface deformable body assembly under xform with material and skin material assignement
        xform_db_dict = {}
        xform_db_dict["simulation_owner"] = ""
        xform_db_dict["enabled"] = True
        xform_db_dict["kinematic"] = False
        xform_db_dict["start_asleep"] = False
        xform_db_dict["transform_row0"] = carb.Double4(1, 0, 0, 0)
        xform_db_dict["transform_row1"] = carb.Double4(0, 1, 0, 0)
        xform_db_dict["transform_row2"] = carb.Double4(0, 0, 1, 0)
        xform_db_dict["transform_row3"] = carb.Double4(0, 0, 0, 1)
        xform_db_dict["sim_mesh_path"] = "/World/SurfaceDeformableXform/SimMesh"
        xform_db_dict["collision_geom_paths"] = ["/World/SurfaceDeformableXform/CollisionMesh"]
        xform_db_dict["skin_geom_paths"] = ["/World/SurfaceDeformableXform/SkinMesh"]
        xform_db_dict["sim_mesh_material_path"] = "/World/Materials/SurfaceDeformableMaterialB"
        xform_db_dict["collision_geom_material_paths"] = ["/World/Materials/SurfaceDeformableMaterialC"]
        xform_db_dict["skin_geom_material_paths"] = ["/World/Materials/SurfaceDeformableMaterialD"]
        self.expected_prims["/World/SurfaceDeformableXform" + "/surfaceDeformableBody"] = xform_db_dict

        # material A
        material_a_dict = {}
        material_a_dict["youngs_modulus"] = 100000.0
        material_a_dict["poissons_ratio"] = 0.3
        material_a_dict["surface_thickness"] = 0.01
        material_a_dict["surface_stretch_stiffness"] = 200000.0
        material_a_dict["surface_shear_stiffness"] = 300000.0
        material_a_dict["surface_bend_stiffness"] = 400000.0
        self.expected_prims["/World/Materials/SurfaceDeformableMaterialA" + "/surfaceDeformableMaterial"] = material_a_dict

        # material B
        material_b_dict = {}
        material_b_dict["youngs_modulus"] = 200000.0
        material_b_dict["poissons_ratio"] = 0.4
        material_b_dict["surface_thickness"] = 0.02
        material_b_dict["surface_stretch_stiffness"] = 300000.0
        material_b_dict["surface_shear_stiffness"] = 400000.0
        material_b_dict["surface_bend_stiffness"] = 500000.0
        self.expected_prims["/World/Materials/SurfaceDeformableMaterialB" + "/surfaceDeformableMaterial"] = material_b_dict

        # material C
        material_c_dict = {}
        material_c_dict["youngs_modulus"] = 300000.0
        material_c_dict["poissons_ratio"] = 0.5
        material_c_dict["surface_thickness"] = 0.03
        material_c_dict["surface_stretch_stiffness"] = 400000.0
        material_c_dict["surface_shear_stiffness"] = 500000.0
        material_c_dict["surface_bend_stiffness"] = 600000.0
        self.expected_prims["/World/Materials/SurfaceDeformableMaterialC" + "/surfaceDeformableMaterial"] = material_c_dict

        # material D
        material_d_dict = {}
        material_d_dict["youngs_modulus"] = 400000.0
        material_d_dict["poissons_ratio"] = 0.6
        material_d_dict["surface_thickness"] = 0.04
        material_d_dict["surface_stretch_stiffness"] = 500000.0
        material_d_dict["surface_shear_stiffness"] = 600000.0
        material_d_dict["surface_bend_stiffness"] = 700000.0
        self.expected_prims["/World/Materials/SurfaceDeformableMaterialD" + "/surfaceDeformableMaterial"] = material_d_dict

        await self.parse("DeformableBodyMaterial")
