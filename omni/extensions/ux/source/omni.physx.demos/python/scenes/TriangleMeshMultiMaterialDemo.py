# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
from omni.physx.scripts.physicsUtils import add_rigid_sphere, add_physics_material_to_prim
from pxr import UsdGeom, UsdShade, Sdf, Gf, Vt, UsdPhysics, PhysicsSchemaTools, PhysxSchema
import omni.physxdemos as demo
from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import ContactEventType


class TriangleMeshMultiMaterialDemo(demo.Base):
    title = "Triangle Mesh Multimaterial"
    category = demo.Categories.MATERIALS
    short_description = "Demo showing different materials assigned to a triangle mesh"
    description = "Demo showing different material restitution behavior when bouncing on a multimaterial triangle mesh, each mesh polygon has a different physics material assigned with different restitution values. Additionally when a sphere hits the mesh face it gets its color assigned through contact notification. Press play (space) to run the simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, hasTable=False, floorOffset=-10.0, zoom=0.5)

        # subscribe to physics contact report event, this callback issued after each simulation step
        self._contact_report_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        sphereCount = 10
        self._colors = []
        self._mat_paths = []
        self._stage = stage

        # Sphere material
        sphereMaterialpath = defaultPrimPath + "/sphereMaterial"
        UsdShade.Material.Define(stage, sphereMaterialpath)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(sphereMaterialpath))
        material.CreateRestitutionAttr().Set(0.9)

        material_scope_path = "/World/Looks"
        UsdGeom.Scope.Define(stage, material_scope_path)

        # Trianglemesh materials
        for i in range(sphereCount):

            mu = 0.0 + (i % 10) * 0.1
            mtl_created = []
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniSurface.mdl",
                mtl_name="OmniSurface",
                mtl_created_list=mtl_created,
                select_new_prim=False,
            )
            mtl_path = mtl_created[0]
            self._mat_paths.append(mtl_path)
            mat_prim = stage.GetPrimAtPath(mtl_path)
            material_prim = UsdShade.Material.Get(stage, mat_prim.GetPath())
            material = UsdPhysics.MaterialAPI.Apply(material_prim.GetPrim())
            material.CreateRestitutionAttr().Set(mu)

            if material_prim:
                shader_mtl_path = stage.DefinePrim("{}/Shader".format(mtl_path), "Shader")
                shader_prim = UsdShade.Shader.Get(stage, shader_mtl_path.GetPath())
                if shader_prim:
                    color = room.get_hsv_color(i / sphereCount)
                    shader_prim.GetPrim().CreateAttribute("inputs:diffuse_reflection_color", Sdf.ValueTypeNames.Color3f).Set(color)
                    self._colors.append(color)

        # Spheres
        stripSize = 40.0
        xOffset = -140.0
        for i in range(sphereCount):
            spherePath = defaultPrimPath + "/spheres/sphere" + str(i)

            size = 10.0
            position = Gf.Vec3f(i * stripSize + xOffset, 0.0, 250.0)

            sphere_prim = add_rigid_sphere(stage, spherePath, size, position)

            # Add material
            add_physics_material_to_prim(stage, sphere_prim, Sdf.Path(sphereMaterialpath))

            # apply contact report
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(sphere_prim)
            contactReportAPI.CreateThresholdAttr().Set(200000)

        # Triangle mesh with multiple materials
        path = defaultPrimPath + "/triangleMesh"
        self._mesh_path = path
        mesh = UsdGeom.Mesh.Define(stage, path)
        halfSize = 200.0

        # Fill in VtArrays
        points = []
        normals = []
        indices = []
        vertexCounts = []

        for i in range(sphereCount):
            subset = UsdGeom.Subset.Define(stage, path + "/subset" + str(i))
            subset.CreateElementTypeAttr().Set("face")
            subset.CreateFamilyNameAttr().Set(UsdShade.Tokens.materialBind)
            subset_indices = [i]
            bindingAPI = UsdShade.MaterialBindingAPI.Apply(subset.GetPrim())
            materialPrim = UsdShade.Material(stage.GetPrimAtPath(self._mat_paths[i]))
            bindingAPI.Bind(materialPrim, UsdShade.Tokens.weakerThanDescendants)

            points.append(Gf.Vec3f(-stripSize / 2.0 + stripSize * i + xOffset, -halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize / 2.0 + stripSize * (i + 1) + xOffset, -halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize / 2.0 + stripSize * (i + 1) + xOffset, halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize / 2.0 + stripSize * i + xOffset, halfSize, 0.0))

            for j in range(4):
                normals.append(Gf.Vec3f(0, 0, 1))
                indices.append(j + i * 4)

            subset.CreateIndicesAttr().Set(subset_indices)
            vertexCounts.append(4)

        mesh.CreateFaceVertexCountsAttr().Set(vertexCounts)
        mesh.CreateFaceVertexIndicesAttr().Set(indices)
        mesh.CreatePointsAttr().Set(points)
        mesh.CreateDoubleSidedAttr().Set(False)
        mesh.CreateNormalsAttr().Set(normals)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("none")

    def on_shutdown(self):
        self._contact_report_sub = None

    def _on_contact_report_event(self, contact_headers, contact_data):
        for contact_header in contact_headers:
            if contact_header.type == ContactEventType.CONTACT_FOUND or contact_header.type == ContactEventType.CONTACT_PERSIST:
                self.collider0 = PhysicsSchemaTools.intToSdfPath(contact_header.collider0)
                self.collider1 = PhysicsSchemaTools.intToSdfPath(contact_header.collider1)

            if self.collider1 == self._mesh_path:
                contact_data_offset = contact_header.contact_data_offset
                num_contact_data = contact_header.num_contact_data
                usdGeom = UsdGeom.Mesh.Get(self._stage, self.collider0)

                for index in range(contact_data_offset, contact_data_offset + num_contact_data, 1):
                    color = Vt.Vec3fArray([self._colors[contact_data[index].face_index1]])
                    usdGeom.GetDisplayColorAttr().Set(color)
