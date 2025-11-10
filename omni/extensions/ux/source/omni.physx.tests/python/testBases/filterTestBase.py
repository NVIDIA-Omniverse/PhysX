# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
from omni.kit import commands
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physx.bindings._physx import ContactEventType
from omni.physx import get_physx_simulation_interface, get_physx_cooking_interface
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, PhysxSchema

class FilterTestBase:

    def on_contact_report_event(self, contact_headers, contact_data):
        for contact_header in contact_headers:
            if contact_header.type == ContactEventType.CONTACT_FOUND:
                self._num_contact_found += 1

    def setup_units_and_scene(self):

        #up axis and units
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        
        #scene with 0 gravity
        utils.execute_and_check(self, "AddPhysicsScene", stage=self._stage, path="/PhysicsScene")
        scene = UsdPhysics.Scene.Get(self._stage, "/PhysicsScene")
        scene.GetGravityMagnitudeAttr().Set(0.0)

        #stepping
        timeStepsPerSecond = 60
        self._time_step = 1.0 / timeStepsPerSecond
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(timeStepsPerSecond)


    def setup_colliders(self, physics_type, is_dyn, ext, pos, vel):
        mov_pfix = "_dyn" if is_dyn else "_stc"
        density = 1.0 if is_dyn else 10000.0
        part_ext = Gf.CompMult(ext, Gf.Vec3f(1.0, 1.0, 0.5))
        part0_pos = Gf.CompMult(part_ext, Gf.Vec3f(0.0, 0.0, 0.5)) 
        part1_pos = -Gf.CompMult(part_ext, Gf.Vec3f(0.0, 0.0, 0.5)) 

        path = Sdf.Path("/" + physics_type + mov_pfix)
        prim = Usd.Prim()
        support_report = True

        if physics_type == 'RigidBody':
            prim = physicsUtils.add_rigid_box(self._stage, path, size=ext, position=pos, density=density, lin_velocity=vel)
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contactReportAPI.CreateThresholdAttr().Set(0)
            
        elif physics_type == 'Collider':
            prim = physicsUtils.add_rigid_box(self._stage, path, size=ext, position=pos, density=0.0)
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contactReportAPI.CreateThresholdAttr().Set(0)

        elif physics_type == 'RigidBodyCompound':
            prim = physicsUtils.add_rigid_xform(self._stage, path, position=pos)
            box0 = physicsUtils.add_box(self._stage, path.AppendChild("collider0"), size=part_ext, position=part0_pos)
            box1 = physicsUtils.add_box(self._stage, path.AppendChild("collider1"), size=part_ext, position=part1_pos)
            UsdPhysics.CollisionAPI.Apply(box0)
            UsdPhysics.CollisionAPI.Apply(box1)
            UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set(vel)
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contactReportAPI.CreateThresholdAttr().Set(0)

        elif physics_type == 'ArticulationLink':
            articulation_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            UsdGeom.Xform.Define(self._stage, articulation_path)
            articulation = UsdPhysics.ArticulationRootAPI.Apply(self._stage.GetPrimAtPath(articulation_path))
            physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(articulation.GetPrim())
            physx_articulation.CreateEnabledSelfCollisionsAttr().Set(False)
            prim = physicsUtils.add_rigid_box(self._stage, articulation_path.AppendChild("link"), size=ext, position=pos, density=density, lin_velocity=vel)
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contactReportAPI.CreateThresholdAttr().Set(0)

        elif physics_type == 'ArticulationLinkCompound':
            articulation_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            UsdGeom.Xform.Define(self._stage, articulation_path)
            articulation = UsdPhysics.ArticulationRootAPI.Apply(self._stage.GetPrimAtPath(articulation_path))
            physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(articulation.GetPrim())
            physx_articulation.CreateEnabledSelfCollisionsAttr().Set(False)            
            box0 = physicsUtils.add_box(self._stage, articulation_path.AppendChild("body0"), size=part_ext, position=part0_pos)
            box1 = physicsUtils.add_box(self._stage, articulation_path.AppendChild("body1"), size=part_ext, position=part1_pos)
            revolute_joint = UsdPhysics.RevoluteJoint.Define(self._stage, articulation_path.AppendChild("joint"))
            revolute_joint.CreateBody0Rel().SetTargets([box0.GetPrimPath()])
            revolute_joint.CreateBody1Rel().SetTargets([box1.GetPrimPath()])
            UsdPhysics.CollisionAPI.Apply(box0)
            UsdPhysics.RigidBodyAPI.Apply(box0)
            UsdPhysics.CollisionAPI.Apply(box1)
            UsdPhysics.RigidBodyAPI.Apply(box1)            
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(box0)
            contactReportAPI.CreateThresholdAttr().Set(0)
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(box1)
            contactReportAPI.CreateThresholdAttr().Set(0)
            prim = articulation.GetPrim()
        # DEPRECATED
        elif physics_type == 'DeformableBody':
            success, tmp_path = commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", half_scale=100.0, u_patches=1, v_patches=1, w_patches=1, u_verts_scale=2, v_verts_scale=2, w_verts_scale=2)
            self.assertTrue(success)
            defbody_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            utils.execute_and_check(self, "MovePrim", path_from=tmp_path, path_to=defbody_path)
            prim = self._stage.GetPrimAtPath(defbody_path)
            scale = Gf.Vec3d(ext*0.5)
            orient = Gf.Quatd()
            trans = Gf.Vec3d(pos)
            physicsUtils.set_or_add_scale_orient_translate(UsdGeom.Xformable(prim), scale, orient, trans)
            utils.execute_and_check(self, "AddDeformableBodyComponent", skin_mesh_path=defbody_path, voxel_resolution=4)
            get_physx_cooking_interface().cook_deformable_body_mesh(str(defbody_path))
            deformablebody = PhysxSchema.PhysxDeformableBodyAPI(prim)
            points = deformablebody.GetSimulationRestPointsAttr().Get()
            velocities = [Gf.Vec3f(0.0)]*len(points)
            for i in range(len(points)):
                velocities[i] = vel
            PhysxSchema.PhysxDeformableAPI(deformablebody).GetSimulationVelocitiesAttr().Set(velocities)
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.GetDensityAttr().Set(density)
            support_report = False

        elif physics_type == 'DeformableSurface':
            resX = int(5*ext[0])
            resY = int(5*ext[1])
            success, tmp_path = commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane", half_scale=100.0, u_patches=1, v_patches=1, u_verts_scale=resX, v_verts_scale=resY)
            self.assertTrue(success)
            defsurface_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            utils.execute_and_check(self, "MovePrim", path_from=tmp_path, path_to=defsurface_path)
            prim = self._stage.GetPrimAtPath(defsurface_path)
            scale = Gf.Vec3d(ext*0.5)
            orient = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), 90.0).GetQuat()
            trans = Gf.Vec3d(pos)
            physicsUtils.set_or_add_scale_orient_translate(UsdGeom.Xformable(prim), scale, orient, trans)
            utils.execute_and_check(self, "AddDeformableSurfaceComponent", mesh_path=defsurface_path)
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            velocities = [Gf.Vec3f(0.0)]*len(points)
            for i in range(len(points)):
                velocities[i] = vel
            defsurface = PhysxSchema.PhysxDeformableSurfaceAPI(prim)
            PhysxSchema.PhysxDeformableAPI(defsurface).GetSimulationVelocitiesAttr().Set(velocities)
            collision = PhysxSchema.PhysxCollisionAPI(prim)
            collision.GetRestOffsetAttr().Set(0.1)
            collision.GetContactOffsetAttr().Set(0.2)
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.GetDensityAttr().Set(density)
            support_report = False
        #~DEPRECATED

        elif physics_type == 'Particles':
            points_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            points = UsdGeom.Points.Define(self._stage, points_path)
            utils.execute_and_check(self, "AddParticleSet", prim=points.GetPrim())
            points.GetPointsAttr().Set([pos])
            points.GetVelocitiesAttr().Set([vel])
            prim = points.GetPrim()
            support_report = False

        return prim, support_report

    def get_filter_prims(self, physics_type, prim, coll_filter):
        filter_prims = []
        hierarchical_types = ['RigidBodyCompound', 'ArticulationLinkCompound']
        if physics_type in hierarchical_types and coll_filter:
            filter_prims.extend(prim.GetChildren())
        elif physics_type == 'Particles':
            particles = PhysxSchema.PhysxParticleAPI(prim)
            particleSystemPath = particles.GetParticleSystemRel().GetTargets()[0]
            filter_prims.append(self._stage.GetPrimAtPath(particleSystemPath))
        else:
            filter_prims.append(prim)
        
        return filter_prims

    @staticmethod
    def get_min_max(prim):
        aabb = Gf.Range3d()
        mesh = UsdGeom.Mesh(prim)
        if mesh:
            #for meshes, let's compute tight bounds, as conservative bounds are not great for rotatated geometry
            transform = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            points = mesh.GetPointsAttr().Get()
            for i in range(len(points)):
                aabb.UnionWith(transform.Transform(Gf.Vec3d(points[i])))
        else:
            imageable = UsdGeom.Imageable(prim)
            obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get()) 
            aabb = obb.ComputeAlignedBox()

        return aabb.GetMin()[2], aabb.GetMax()[2]  
