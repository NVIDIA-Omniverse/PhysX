# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.usd
import carb.settings
from omni.kit import commands
from omni.physx.scripts import physicsUtils, deformableMeshUtils, deformableUtils, particleUtils
from omni.physxtests import utils
from omni.physx.bindings._physx import ContactEventType
from omni.physx import get_physx_simulation_interface, get_physx_cooking_interface
from pxr import Gf, Usd, UsdShade, UsdPhysics, UsdGeom, Sdf, PhysxSchema

def create_transform(translate = Gf.Vec3d(0.0),
                     rotate = Gf.Rotation(Gf.Quatd(1.0)),
                     scale = Gf.Vec3d(1.0),
                     pivot_pos = Gf.Vec3d(0.0),
                     pivot_orient = Gf.Rotation(Gf.Quatd(1.0))):
    return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)

def set_tetmesh_data(tetmesh):
    points, indices = deformableMeshUtils.createTetraVoxelBox(3)
    tet_indices = [Gf.Vec4i(*indices[i:i+4]) for i in range(0, len(indices), 4)]
    tetmesh.GetPointsAttr().Set(points)
    tetmesh.GetTetVertexIndicesAttr().Set(tet_indices)

def set_point_based_velocities(prim, velocity):
    point_based = UsdGeom.PointBased(prim)
    if point_based:
        velocities = [velocity]*len(point_based.GetPointsAttr().Get())
        point_based.GetVelocitiesAttr().Set(velocities)

def setup_volume_material(body_prim, mat_path, density):
    mat = UsdShade.Material.Define(body_prim.GetStage(), mat_path)
    prim = mat.GetPrim()
    prim.ApplyAPI("OmniPhysicsBaseMaterialAPI")
    prim.GetAttribute("omniphysics:staticFriction").Set(10.0)
    prim.GetAttribute("omniphysics:dynamicFriction").Set(10.0)
    prim.GetAttribute("omniphysics:density").Set(density)
    prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
    prim.GetAttribute("omniphysics:youngsModulus").Set(1e8)
    prim.GetAttribute("omniphysics:poissonsRatio").Set(0.45)
    binding = UsdShade.MaterialBindingAPI.Apply(body_prim)
    binding.Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

def setup_surface_material(body_prim, mat_path, density):
    mat = UsdShade.Material.Define(body_prim.GetStage(), mat_path)
    prim = mat.GetPrim()
    prim.ApplyAPI("OmniPhysicsBaseMaterialAPI")
    prim.GetAttribute("omniphysics:dynamicFriction").Set(0.2)
    prim.GetAttribute("omniphysics:density").Set(density)
    prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
    prim.GetAttribute("omniphysics:youngsModulus").Set(5e4)
    prim.GetAttribute("omniphysics:poissonsRatio").Set(0.45)
    prim.ApplyAPI("OmniPhysicsSurfaceDeformableMaterialAPI")
    prim.GetAttribute("omniphysics:surfaceThickness").Set(0.01)
    binding = UsdShade.MaterialBindingAPI.Apply(body_prim)
    binding.Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

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

    def setup_cube_trimesh(self, stage, path, transform: Gf.Transform, dim: int):
        tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(dim)
        skinmesh = UsdGeom.Mesh.Define(stage, path)
        skinmesh.AddTransformOp().Set(transform.GetMatrix())
        skinmesh.GetPointsAttr().Set(tri_points)
        skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
        skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        return skinmesh

    def setup_plane_trimesh(self, stage, path, transform: Gf.Transform, dim: int):
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dim, dim)
        skinmesh = UsdGeom.Mesh.Define(stage, path)
        skinmesh.AddTransformOp().Set(transform.GetMatrix())
        skinmesh.GetPointsAttr().Set(tri_points)
        skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
        skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        return skinmesh, tri_indices

    def setup_colliders_volume_deformable(self, path, ext, density, pos, vel):
        defbody_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
        scale = Gf.Vec3d(ext*0.5)
        translate = Gf.Vec3d(pos)
        transform = create_transform(translate=translate, scale=scale)
        sim_mesh = UsdGeom.TetMesh.Define(self._stage, defbody_path)
        sim_mesh.AddTransformOp().Set(transform.GetMatrix())
        set_tetmesh_data(sim_mesh)

        success = deformableUtils.set_physics_volume_deformable_body(self._stage, defbody_path)
        self.assertTrue(success)

        prim = sim_mesh.GetPrim()
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision = PhysxSchema.PhysxCollisionAPI(prim)
        collision.GetRestOffsetAttr().Set(0.1)
        collision.GetContactOffsetAttr().Set(0.2)

        set_point_based_velocities(prim, vel)

        setup_volume_material(prim, str(defbody_path) + "/material", density)

        return prim

    def setup_colliders_volume_deformable_hierarchy(self, path, ext, density, pos, vel):
        defbody_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
        # Create xform that serves as the root of the volume deformable structure
        volume_deformable = UsdGeom.Xform.Define(self._stage, defbody_path)
        skin_mesh_path = str(defbody_path) + "/skinMesh"
        scale = Gf.Vec3d(ext*0.5)
        translate = Gf.Vec3d(pos)
        transform = create_transform(translate=translate, scale=scale)
        skin_mesh = self.setup_cube_trimesh(self._stage, skin_mesh_path, transform, 2)
        sim_mesh_path = str(defbody_path) + "/simMesh"
        # Apply OmniPhysicsDeformableBodyAPI to root, and setup volume deformable structure
        success = deformableUtils.create_auto_volume_deformable_hierarchy(self._stage,
            root_prim_path = defbody_path,
            simulation_tetmesh_path = sim_mesh_path,
            collision_tetmesh_path = sim_mesh_path,
            cooking_src_mesh_path = skin_mesh.GetPath(),
            simulation_hex_mesh_enabled = False,
            cooking_src_simplification_enabled = True,
            set_visibility_with_guide_purpose = False   # Need render purpose for bound computation
        )
        self.assertTrue(success)

        success = get_physx_cooking_interface().cook_auto_deformable_body(str(defbody_path))
        self.assertTrue(success)

        sim_mesh_prim = self._stage.GetPrimAtPath(sim_mesh_path)

        PhysxSchema.PhysxCollisionAPI.Apply(sim_mesh_prim)
        collision = PhysxSchema.PhysxCollisionAPI(sim_mesh_prim)
        collision.GetRestOffsetAttr().Set(0.1)
        collision.GetContactOffsetAttr().Set(0.2)

        xform_prim = volume_deformable.GetPrim()

        # Apply PhysxBaseDeformableBodyAPI for PhysX specific attributes.
        xform_prim.ApplyAPI("PhysxBaseDeformableBodyAPI")
        xform_prim.GetAttribute("physxDeformableBody:selfCollision").Set(False)

        set_point_based_velocities(sim_mesh_prim, vel)

        setup_volume_material(xform_prim, str(defbody_path) + "/material", density)

        return xform_prim, sim_mesh_prim

    def setup_colliders_surface_deformable(self, path, ext, density, pos, vel):
        defbody_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
        scale = Gf.Vec3d(ext*0.5)
        translate = Gf.Vec3d(pos)
        transform = create_transform(translate=translate, scale=scale)
        sim_mesh, tri_indices = self.setup_plane_trimesh(self._stage, defbody_path, transform, 4)

        success = deformableUtils.set_physics_surface_deformable_body(self._stage, defbody_path)
        self.assertTrue(success)

        prim = sim_mesh.GetPrim()

        PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision = PhysxSchema.PhysxCollisionAPI(prim)
        collision.GetRestOffsetAttr().Set(0.1)
        collision.GetContactOffsetAttr().Set(0.2)

        set_point_based_velocities(prim, vel)

        setup_surface_material(prim, str(defbody_path) + "/material", density)

        return prim

    def setup_colliders_surface_deformable_hierarchy(self, path, ext, density, pos, vel):
        defbody_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
        # Create xform that serves as the root of the surface deformable structure
        surface_deformable = UsdGeom.Xform.Define(self._stage, defbody_path)
        skin_mesh_path = str(defbody_path) + "/skinMesh"
        scale = Gf.Vec3d(ext*0.5)
        translate = Gf.Vec3d(pos)
        transform = create_transform(translate=translate, scale=scale)
        skin_mesh, tri_indices = self.setup_plane_trimesh(self._stage, skin_mesh_path, transform, 4)
        sim_mesh_path = str(defbody_path) + "/simMesh"     
        # Apply OmniPhysicsDeformableBodyAPI to root, and setup surface deformable structure
        success = deformableUtils.create_auto_surface_deformable_hierarchy(self._stage,
            root_prim_path = defbody_path,
            simulation_mesh_path = sim_mesh_path,
            cooking_src_mesh_path = skin_mesh.GetPath(),
            cooking_src_simplification_enabled = True,
            set_visibility_with_guide_purpose = False   # Need render purpose for bound computation
        )
        self.assertTrue(success)

        xform_prim = surface_deformable.GetPrim()     
        # Apply PhysxSurfaceDeformableBodyAPI for PhysX specific attributes.
        xform_prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        xform_prim.GetAttribute("physxDeformableBody:selfCollision").Set(False)
        xform_prim.GetAttribute("physxDeformableBody:enableSpeculativeCCD").Set(True)
        xform_prim.GetAttribute("physxDeformableBody:solverPositionIterationCount").Set(16)
        xform_prim.GetAttribute("physxDeformableBody:collisionPairUpdateFrequency").Set(4)
        xform_prim.GetAttribute("physxDeformableBody:collisionIterationMultiplier").Set(4)

        success = get_physx_cooking_interface().cook_auto_deformable_body(str(defbody_path))
        self.assertTrue(success)

        sim_mesh_prim = self._stage.GetPrimAtPath(sim_mesh_path)

        PhysxSchema.PhysxCollisionAPI.Apply(sim_mesh_prim)
        collision = PhysxSchema.PhysxCollisionAPI(sim_mesh_prim)
        collision.GetRestOffsetAttr().Set(0.1)
        collision.GetContactOffsetAttr().Set(0.2)

        set_point_based_velocities(sim_mesh_prim, vel)

        setup_surface_material(xform_prim, str(defbody_path) + "/material", density)

        return xform_prim, sim_mesh_prim

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
            defbody_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            scale = Gf.Vec3d(ext*0.5)
            translate = Gf.Vec3d(pos)
            transform = create_transform(translate=translate, scale=scale)
            skin_mesh = self.setup_cube_trimesh(self._stage, defbody_path, transform, 4)
            deformableUtils.add_physx_deformable_body(
                self._stage, prim_path=defbody_path, collision_simplification=False, simulation_hexahedral_resolution=4
            )
            get_physx_cooking_interface().cook_deformable_body_mesh(str(defbody_path))
            prim = skin_mesh.GetPrim()
            deformablebody = PhysxSchema.PhysxDeformableBodyAPI(prim)
            points = deformablebody.GetSimulationRestPointsAttr().Get()
            velocities = [Gf.Vec3f(0.0)]*len(points)
            for i in range(len(points)):
                velocities[i] = vel

            collision = PhysxSchema.PhysxCollisionAPI(prim)
            collision.GetRestOffsetAttr().Set(0.1)
            collision.GetContactOffsetAttr().Set(0.2)
            
            PhysxSchema.PhysxDeformableAPI(deformablebody).GetSimulationVelocitiesAttr().Set(velocities)
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.GetDensityAttr().Set(density)
            support_report = False

        elif physics_type == 'DeformableSurface':
            defsurface_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            scale = Gf.Vec3d(ext*0.5)
            translate = Gf.Vec3d(pos)
            transform = create_transform(translate=translate, scale=scale)
            skin_mesh, tri_indices = self.setup_plane_trimesh(self._stage, defsurface_path, transform, 32)
            deformableUtils.add_physx_deformable_surface(
                self._stage, 
                prim_path=defsurface_path,
                simulation_indices=tri_indices,
                solver_position_iteration_count=20,
                vertex_velocity_damping=0.0
            )

            prim = skin_mesh.GetPrim()
            defsurface = PhysxSchema.PhysxDeformableSurfaceAPI(prim)
            points = skin_mesh.GetPointsAttr().Get()
            velocities = [Gf.Vec3f(0.0)]*len(points)
            for i in range(len(points)):
                velocities[i] = vel
            
            PhysxSchema.PhysxDeformableAPI(defsurface).GetSimulationVelocitiesAttr().Set(velocities)
            collision = PhysxSchema.PhysxCollisionAPI(prim)
            collision.GetRestOffsetAttr().Set(0.1)
            collision.GetContactOffsetAttr().Set(0.2)
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.GetDensityAttr().Set(density)
            support_report = False
        #~DEPRECATED

        elif physics_type == 'VolumeDeformableHierarchyXform':
            xform_prim, collision_prim = self.setup_colliders_volume_deformable_hierarchy(path, ext, density, pos, vel)
            prim = xform_prim
            support_report = False

        elif physics_type == 'VolumeDeformableHierarchyCollMesh':
            xform_prim, collision_prim = self.setup_colliders_volume_deformable_hierarchy(path, ext, density, pos, vel)
            prim = collision_prim
            support_report = False

        elif physics_type == 'VolumeDeformableNonHierarchy':
            prim = self.setup_colliders_volume_deformable(path, ext, density, pos, vel)
            support_report = False

        elif physics_type == 'SurfaceDeformableHierarchyXform':
            xform_prim, collision_prim = self.setup_colliders_surface_deformable_hierarchy(path, ext, density, pos, vel)
            prim = xform_prim
            support_report = False

        elif physics_type == 'SurfaceDeformableHierarchyCollMesh':
            xform_prim, collision_prim = self.setup_colliders_surface_deformable_hierarchy(path, ext, density, pos, vel)
            prim = collision_prim
            support_report = False

        elif physics_type == 'SurfaceDeformableNonHierarchy':
            prim = self.setup_colliders_surface_deformable(path, ext, density, pos, vel)
            support_report = False

        elif physics_type == 'Particles':
            points_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, path, True))
            points = UsdGeom.Points.Define(self._stage, points_path)
            prim=points.GetPrim()

            # get a particle system
            particle_system = particleUtils.get_default_particle_system(self._stage)
            particle_system_path = particle_system.GetPath()

            particle_system_prim = self._stage.GetPrimAtPath(particle_system_path)
            if not particle_system_prim or not PhysxSchema.PhysxParticleSystem(particle_system_prim):
                carb.log_error(type(self).__name__ + ": particle system path needs to point to valid PhysxSchema.ParticleSystem.")
                return

            particleUtils.configure_particle_set(prim, particle_system_path, True, True, 0)
            points.GetPointsAttr().Set([pos])
            points.GetVelocitiesAttr().Set([vel])
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
        pointBased = UsdGeom.PointBased(prim)
        if pointBased:
            #for meshes, let's compute tight bounds, as conservative bounds are not great for rotatated geometry
            transform = pointBased.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            points = pointBased.GetPointsAttr().Get()
            for i in range(len(points)):
                aabb.UnionWith(transform.Transform(Gf.Vec3d(points[i])))
        else:
            # We don't use physx reported bounds because they are sometimes too conservative
            imageable = UsdGeom.Imageable(prim)    
            obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.render)
            aabb = obb.ComputeAlignedBox()

        return aabb.GetMin()[2], aabb.GetMax()[2]  
